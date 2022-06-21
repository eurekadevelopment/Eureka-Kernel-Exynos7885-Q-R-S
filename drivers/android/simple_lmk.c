// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2021 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#define pr_fmt(fmt) "simple_lmk: " fmt

#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/oom.h>
#include <linux/vmpressure.h>
#include <linux/sort.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

/* The minimum number of pages to free per reclaim */
#define MIN_FREE_PAGES (CONFIG_ANDROID_SIMPLE_LMK_MINFREE * SZ_1M / PAGE_SIZE)

/* Kill up to this many victims per reclaim */
#define MAX_VICTIMS 16

/* Timeout in jiffies for each reclaim */
#define RECLAIM_EXPIRES msecs_to_jiffies(CONFIG_ANDROID_SIMPLE_LMK_TIMEOUT_MSEC)

struct victim_info {
	struct task_struct *tsk;
	struct mm_struct *mm;
	unsigned long size;
};

static struct victim_info victims[MAX_VICTIMS] __cacheline_aligned_in_smp;
static struct task_struct *pending __cacheline_aligned_in_smp;
static struct task_struct *task_bucket[SHRT_MAX + 1] __cacheline_aligned;
static DECLARE_WAIT_QUEUE_HEAD(oom_waitq);
static DECLARE_COMPLETION(reclaim_done);
static __cacheline_aligned_in_smp DEFINE_RWLOCK(mm_free_lock);
static int nr_victims;
static bool init_done = false;
static atomic_t needs_reclaim = ATOMIC_INIT(0);
static atomic_t nr_killed = ATOMIC_INIT(0);

enum {
	PRIO_APP_FOREGROUND = 10,
	PRIO_APP_BACKGROUND_NOT_CLOSED = 20,
};

static int victim_cmp(const void *lhs_ptr, const void *rhs_ptr)
{
	const struct victim_info *lhs = (typeof(lhs))lhs_ptr;
	const struct victim_info *rhs = (typeof(rhs))rhs_ptr;

	return rhs->size - lhs->size;
}

static void victim_swap(void *lhs_ptr, void *rhs_ptr, int size)
{
	struct victim_info *lhs = (typeof(lhs))lhs_ptr;
	struct victim_info *rhs = (typeof(rhs))rhs_ptr;

	swap(*lhs, *rhs);
}

static unsigned long get_total_mm_pages(struct mm_struct *mm)
{
	unsigned long pages = 0;
	int i;

	for (i = 0; i < NR_MM_COUNTERS; i++)
		pages += get_mm_counter(mm, i);

	return pages;
}

static unsigned long find_victims(int *vindex)
{
	short i, min_adj = SHRT_MAX, max_adj = 0;
	unsigned long pages_found = 0;
	struct task_struct *tsk;

	rcu_read_lock();
	for_each_process (tsk) {
		struct signal_struct *sig;
		short adj;

		/*
		 * Search for suitable tasks with a positive adj (importance).
		 * Since only tasks with a positive adj can be targeted, that
		 * naturally excludes tasks which shouldn't be killed, like init
		 * and kthreads. Although oom_score_adj can still be changed
		 * while this code runs, it doesn't really matter; we just need
		 * a snapshot of the task's adj.
		 */
		sig = tsk->signal;
		adj = READ_ONCE(sig->oom_score_adj);
		if (adj < 0 ||
		    sig->flags & (SIGNAL_GROUP_EXIT | SIGNAL_GROUP_COREDUMP) ||
		    (thread_group_empty(tsk) && tsk->flags & PF_EXITING))
			continue;

		/* Store the task in a linked-list bucket based on its adj */
		tsk->simple_lmk_next = task_bucket[adj];
		task_bucket[adj] = tsk;

		/* Track the min and max adjs to speed up the loop below */
		if (adj > max_adj)
			max_adj = adj;
		if (adj < min_adj)
			min_adj = adj;
	}

	/* Start searching for victims from the highest adj (least important) */
	for (i = max_adj; i >= min_adj; i--) {
		int old_vindex;

		tsk = task_bucket[i];
		if (!tsk)
			continue;

		/* Clear out this bucket for the next time reclaim is done */
		task_bucket[i] = NULL;

		/* Iterate through every task with this adj */
		old_vindex = *vindex;
		do {
			struct task_struct *vtsk;

			vtsk = find_lock_task_mm(tsk);
			if (!vtsk)
				continue;

			/* Store this potential victim away for later */
			victims[*vindex].tsk = vtsk;
			victims[*vindex].mm = vtsk->mm;
			victims[*vindex].size = get_total_mm_pages(vtsk->mm);

			/* Count the number of pages that have been found */
			pages_found += victims[*vindex].size;

			/* Make sure there's space left in the victim array */
			if (++*vindex == MAX_VICTIMS)
				break;
		} while ((tsk = tsk->simple_lmk_next));

		/* Go to the next bucket if nothing was found */
		if (*vindex == old_vindex)
			continue;

		/*
		 * Sort the victims in descending order of size to prioritize
		 * killing the larger ones first.
		 */
		sort(&victims[old_vindex], *vindex - old_vindex,
		     sizeof(*victims), victim_cmp, victim_swap);

		/* Stop when we are out of space or have enough pages found */
		if (*vindex == MAX_VICTIMS || pages_found >= MIN_FREE_PAGES) {
			/* Zero out any remaining buckets we didn't touch */
			if (i > min_adj)
				memset(&task_bucket[min_adj], 0,
				       (i - min_adj) * sizeof(*task_bucket));
			break;
		}
	}
	rcu_read_unlock();

	return pages_found;
}

static int process_victims(int vlen)
{
	unsigned long pages_found = 0;
	int i, nr_to_kill = 0;

	/*
	 * Calculate the number of tasks that need to be killed and quickly
	 * release the references to those that'll live.
	 */
	for (i = 0; i < vlen; i++) {
		struct victim_info *victim = &victims[i];
		struct task_struct *vtsk = victim->tsk;

		/* The victim's mm lock is taken in find_victims; release it */
		if (pages_found >= MIN_FREE_PAGES) {
			task_unlock(vtsk);
		} else {
			pages_found += victim->size;
			nr_to_kill++;
		}
	}

	return nr_to_kill;
}

static void kill_task(struct task_struct *vtsk)
{
	static const struct sched_param sched_zero_prio = { .sched_priority =
								    0 };
	struct task_struct *t;
	/* Accelerate the victim's death by forcing the kill signal */
	do_send_sig_info(SIGKILL, SEND_SIG_FORCED, vtsk, true);

	/* Mark the thread group dead so that other kernel code knows */
	rcu_read_lock();
	for_each_thread (vtsk, t)
		set_tsk_thread_flag(t, TIF_MEMDIE);
	rcu_read_unlock();

	/* Elevate the victim to SCHED_RR with zero RT priority */
	sched_setscheduler_nocheck(vtsk, SCHED_RR, &sched_zero_prio);

	/* Allow the victim to run on any CPU. This won't schedule. */
	set_cpus_allowed_ptr(vtsk, cpu_all_mask);

	/* Signals can't wake frozen tasks; only a thaw operation can */
	__thaw_task(vtsk);

	task_unlock(vtsk);
}

/*
 * Avoid using vmalloc for a small buffer.
 * Should not be used when the size is statically known.
 */

static void *kvmalloc(unsigned long size)
{
	if (size > PAGE_SIZE)
		return vmalloc(size);
	else
		return kmalloc(size, GFP_ATOMIC);
}

static void simple_lmk_kvfree(unsigned long size, void *addr)
{
	if (size > PAGE_SIZE)
		vfree(addr);
	else
		kfree(addr);
}

static void scan_and_kill(void)
{
	int j, i, nr_to_kill, nr_found = 0;
	static int k = 0;
	unsigned long pages_found;
	struct task_struct *tmp;

	if (init_done && k != 0) {
		for (j = 0; j < k; j++) {
			struct task_struct *target = &pending[j];
			if (!target)
				continue;
			pr_info("Process PPID check comm: %s\n", target->comm);
			if (task_ppid_nr(target) == 1) {
				task_lock(target);
				pr_info("Killing process %d with PPID 1!!\n",
					target->pid);
				kill_task(target);
			}
		}
		simple_lmk_kvfree(k * sizeof(struct task_struct), pending);
		k = 0;
	} else {
		init_done = true;
	}

	tmp = kvmalloc(MAX_VICTIMS * sizeof(struct task_struct));
	/* Populate the victims array with tasks sorted by adj and then size */
	pages_found = find_victims(&nr_found);
	if (unlikely(!nr_found)) {
		pr_err("No processes available to kill!\n");
		return;
	}

	/* Minimize the number of victims if we found more pages than needed */
	if (pages_found > MIN_FREE_PAGES) {
		/* First round of processing to weed out unneeded victims */
		nr_to_kill = process_victims(nr_found);

		/*
		 * Try to kill as few of the chosen victims as possible by
		 * sorting the chosen victims by size, which means larger
		 * victims that have a lower adj can be killed in place of
		 * smaller victims with a high adj.
		 */
		sort(victims, nr_to_kill, sizeof(*victims), victim_cmp,
		     victim_swap);

		/* Second round of processing to finally select the victims */
		nr_to_kill = process_victims(nr_to_kill);
	} else {
		/* Too few pages found, so all the victims need to be killed */
		nr_to_kill = nr_found;
	}

	/* Store the final number of victims for simple_lmk_mm_freed() */
	write_lock(&mm_free_lock);
	nr_victims = nr_to_kill;
	write_unlock(&mm_free_lock);

	/* Kill the victims */
	for (i = 0; i < nr_to_kill; i++) {
		struct victim_info *victim = &victims[i];
		struct task_struct *vtsk = victim->tsk;

		// If the task is kthread, don't kill
		if (vtsk->flags & PF_KTHREAD) {
			task_unlock(vtsk);
			continue;
		}

		// The app process' priority values should be correct, but for compatibility.
		if (task_prio(vtsk) >= PRIO_APP_FOREGROUND &&
		    task_prio(vtsk) < PRIO_APP_BACKGROUND_NOT_CLOSED) {
			if (task_ppid_nr(vtsk) == 1) {
				pr_info("Found process with high priority but PPID is 1. Killing!\n");
			} else {
				pr_info("Process %s has high priority %d, maybe foreground app, not killing!\n",
					vtsk->comm, task_prio(vtsk));
				tmp[k] = *vtsk;
				k++;
				task_unlock(vtsk);
				continue;
			}

		} else {
			// Check the oom_score_adj value before kill
			if (vtsk->signal->oom_score_adj < 900) {
				pr_info("Process %s has low priority but oom_score_adj is %d, not killing!\n",
					vtsk->comm,
					vtsk->signal->oom_score_adj);
				tmp[k] = *vtsk;
				k++;
				task_unlock(vtsk);
				continue;
			}
		}

		pr_info("Killing %s with adj %d to free %lu KiB\n", vtsk->comm,
			vtsk->signal->oom_score_adj,
			victim->size << (PAGE_SHIFT - 10));
		kill_task(vtsk);
	}

	/* Wait until all the victims die or until the timeout is reached */
	wait_for_completion_timeout(&reclaim_done, RECLAIM_EXPIRES);

	/* Clean up for future reclaim invocations */
	write_lock(&mm_free_lock);
	reinit_completion(&reclaim_done);
	if (k != 0) {
		pending = kvmalloc(k * sizeof(struct task_struct));
		memcpy(pending, tmp, k * sizeof(struct task_struct));
	} else {
		pr_info("Foreground app count 0, Skip kmalloc\n");
	}
	simple_lmk_kvfree(MAX_VICTIMS * sizeof(struct task_struct), tmp);
	nr_victims = 0;
	nr_killed = (atomic_t)ATOMIC_INIT(0);
	write_unlock(&mm_free_lock);
}

static int simple_lmk_reclaim_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);
	set_freezable();

	while (1) {
		wait_event_freezable(oom_waitq, atomic_read(&needs_reclaim));
		scan_and_kill();
		atomic_set_release(&needs_reclaim, 0);
	}

	return 0;
}

void simple_lmk_mm_freed(struct mm_struct *mm)
{
	int i;

	/* Nothing to do when reclaim is starting or ending */
	if (!read_trylock(&mm_free_lock))
		return;

	for (i = 0; i < nr_victims; i++) {
		if (victims[i].mm == mm) {
			victims[i].mm = NULL;
			if (atomic_inc_return_relaxed(&nr_killed) == nr_victims)
				complete(&reclaim_done);
			break;
		}
	}
	read_unlock(&mm_free_lock);
}

static int simple_lmk_vmpressure_cb(struct notifier_block *nb,
				    unsigned long pressure, void *data)
{
	if (pressure == 100 && !atomic_cmpxchg_acquire(&needs_reclaim, 0, 1))
		wake_up(&oom_waitq);

	return NOTIFY_OK;
}

static struct notifier_block vmpressure_notif = {
	.notifier_call = simple_lmk_vmpressure_cb,
	.priority = INT_MAX
};

/* Initialize Simple LMK when lmkd in Android writes to the minfree parameter */
static int simple_lmk_init_set(const char *val, const struct kernel_param *kp)
{
	static atomic_t init_done = ATOMIC_INIT(0);
	struct task_struct *thread;

	if (!atomic_cmpxchg(&init_done, 0, 1)) {
		thread = kthread_run(simple_lmk_reclaim_thread, NULL,
				     "simple_lmkd");
		BUG_ON(IS_ERR(thread));
		BUG_ON(vmpressure_notifier_register(&vmpressure_notif));
	}

	return 0;
}

static const struct kernel_param_ops simple_lmk_init_ops = {
	.set = simple_lmk_init_set
};

/* Needed to prevent Android from thinking there's no LMK and thus rebooting */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "lowmemorykiller."
module_param_cb(minfree, &simple_lmk_init_ops, NULL, 0200);
