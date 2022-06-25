// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2021 Sultan Alsawaf <sultan@kerneltoast.com>.
 * Copyright (C) 2022- Soo-Hwan Na <whiteshell2544@naver.com>
 */

#define pr_fmt(fmt) "simple_lmk: " fmt

#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/oom.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/string.h>
#include <linux/swap.h>
#include <linux/vmalloc.h>

/* Kill up to this many victims per reclaim */
#define MAX_VICTIMS 64

struct victim_info {
  struct task_struct *tsk;
  struct mm_struct *mm;
  unsigned long size;
};

static struct victim_info victims[MAX_VICTIMS] __cacheline_aligned_in_smp;
static struct task_struct *task_bucket[SHRT_MAX + 1] __cacheline_aligned_in_smp;
static int nr_victims;

static int wanted = 85;

module_param(wanted, int, 0664);

static void simple_lmk_reclaim_work(struct work_struct *data);
static struct workqueue_struct *kill_work_queue;
static DECLARE_DELAYED_WORK(kill_task_work, simple_lmk_reclaim_work);

static int victim_cmp(const void *lhs_ptr, const void *rhs_ptr) {
  const struct victim_info *lhs = (typeof(lhs))lhs_ptr;
  const struct victim_info *rhs = (typeof(rhs))rhs_ptr;

  return rhs->size - lhs->size;
}

static void victim_swap(void *lhs_ptr, void *rhs_ptr, int size) {
  struct victim_info *lhs = (typeof(lhs))lhs_ptr;
  struct victim_info *rhs = (typeof(rhs))rhs_ptr;

  swap(*lhs, *rhs);
}

static unsigned long get_total_mm_pages(struct mm_struct *mm) {
  unsigned long pages = 0;
  int i;

  for (i = 0; i < NR_MM_COUNTERS; i++) pages += get_mm_counter(mm, i);

  return pages;
}

static unsigned long find_victims(int *vindex) {
  short i, min_adj = SHRT_MAX, max_adj = 0;
  unsigned long pages_found = 0;
  struct task_struct *tsk;

  rcu_read_lock();
  for_each_process(tsk) {
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
    if (adj < 0 || sig->flags & (SIGNAL_GROUP_EXIT | SIGNAL_GROUP_COREDUMP) ||
        (thread_group_empty(tsk) && tsk->flags & PF_EXITING))
      continue;

    /* Store the task in a linked-list bucket based on its adj */
    tsk->simple_lmk_next = task_bucket[adj];
    task_bucket[adj] = tsk;

    /* Track the min and max adjs to speed up the loop below */
    if (adj > max_adj) max_adj = adj;
    if (adj < min_adj) min_adj = adj;
  }

  /* Start searching for victims from the highest adj (least important) */
  for (i = max_adj; i >= min_adj; i--) {
    int old_vindex;

    tsk = task_bucket[i];
    if (!tsk) continue;

    /* Clear out this bucket for the next time reclaim is done */
    task_bucket[i] = NULL;

    /* Iterate through every task with this adj */
    old_vindex = *vindex;
    do {
      struct task_struct *vtsk;

      vtsk = find_lock_task_mm(tsk);
      if (!vtsk) continue;

      /* Store this potential victim away for later */
      victims[*vindex].tsk = vtsk;
      victims[*vindex].mm = vtsk->mm;
      victims[*vindex].size = get_total_mm_pages(vtsk->mm);

      /* Count the number of pages that have been found */
      pages_found += victims[*vindex].size;

      /* Make sure there's space left in the victim array */
      if (++*vindex == MAX_VICTIMS) break;
    } while ((tsk = tsk->simple_lmk_next));

    /* Go to the next bucket if nothing was found */
    if (*vindex == old_vindex) continue;

    /*
     * Sort the victims in descending order of size to prioritize
     * killing the larger ones first.
     */
    sort(&victims[old_vindex], *vindex - old_vindex, sizeof(*victims),
         victim_cmp, victim_swap);

    /* Stop when we are out of space or have enough pages found */
    if (*vindex == MAX_VICTIMS) {
      /* Zero out any remaining buckets we didn't touch */
      if (i > min_adj)
        memset(&task_bucket[min_adj], 0, (i - min_adj) * sizeof(*task_bucket));
      break;
    }
  }
  rcu_read_unlock();

  return pages_found;
}

static void kill_task(struct task_struct *vtsk) {
  static const struct sched_param sched_zero_prio = {.sched_priority = 0};
  struct task_struct *t;

  /* Accelerate the victim's death by forcing the kill signal */
  do_send_sig_info(SIGKILL, SEND_SIG_FORCED, vtsk, true);

  /* Mark the thread group dead so that other kernel code knows */
  rcu_read_lock();
  for_each_thread(vtsk, t) set_tsk_thread_flag(t, TIF_MEMDIE);
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

static void *kvmalloc(unsigned long size) {
  if (size > PAGE_SIZE)
    return vmalloc(size);
  else
    return kzalloc(size, GFP_KERNEL);
}

static void simple_lmk_kvfree(unsigned long size, void *ptr) {
  if (size > PAGE_SIZE)
    vfree(ptr);
  else
    kfree(ptr);
}

struct process_data {
  int pid;
  unsigned long score;
};

static struct process_data *processes[MAX_VICTIMS];

static int get_mm_usage(void) {
  static struct sysinfo info;
  static long cached;

  si_meminfo(&info);
  cached = global_page_state(NR_FILE_PAGES) - total_swapcache_pages() -
           info.bufferram;

  if (cached < 0) cached = 0;

  return 100 -
         ((info.freeram + info.bufferram + cached) / (info.totalram / 100));
}

static int compare_mm(const void *arg1, const void *arg2) {
  const struct process_data *first = (struct process_data *)(arg1);
  const struct process_data *second = (struct process_data *)(arg2);
  return second->score - first->score > 0 ? 1 : -1;
}

static void scan_and_kill(void) {
  int i, nr_found = 0, k = 0;
  unsigned long pages_found;
  struct task_struct *tsk;

  sort(processes, MAX_VICTIMS, sizeof(struct process_data *), &compare_mm,
       NULL);
  for (k = 0; k < MAX_VICTIMS; k++) {
    rcu_read_lock();
    for_each_process(tsk) {
      if (tsk->pid == processes[i]->pid) {
        task_lock(tsk);
        kill_task(tsk);
        usleep_range(1500000, 2000000);
      }
    }
    rcu_read_unlock();

    if (get_mm_usage() < wanted) {
      break;
    }
  }

  // Reset the collected data
  for (i = 0; i < MAX_VICTIMS; i++) {
    processes[i]->pid = -1;
    processes[i]->score = -1;
  }

  /* Populate the victims array with tasks sorted by adj and then size */
  pages_found = find_victims(&nr_found);
  if (unlikely(!nr_found)) {
    pr_err("No processes available to kill!\n");
    return;
  }

  /* Store the final number of victims for simple_lmk_mm_freed() */
  nr_victims = nr_found;

  /* Kill the victims */
  for (i = 0; i < nr_victims; i++) {
    struct victim_info *victim = &victims[i];
    struct task_struct *vtsk = victim->tsk;
    unsigned long totalpages = totalram_pages + total_swap_pages;

    processes[i]->pid = vtsk->pid;
    task_unlock(vtsk);
    processes[i]->score =
        oom_badness(vtsk, NULL, NULL, totalpages) * 1000 / totalpages;
    task_lock(vtsk);
    pr_info("%s: comm: %s, pid: %d, prio: %d, score: %lu\n",
            __func__, vtsk->comm,
            processes[i]->pid, task_prio(vtsk), processes[i]->score);
    task_unlock(vtsk);
  }

  nr_victims = 0;
}

static void simple_lmk_reclaim_work(struct work_struct *data) {
  scan_and_kill();
  queue_delayed_work(kill_work_queue, &kill_task_work, 1000);
}

/* Initialize Simple LMK when lmkd in Android writes to the minfree parameter */
static int simple_lmk_init_set(const char *val, const struct kernel_param *kp) {
  int i;

  // Init values
  for (i = 0; i < MAX_VICTIMS; i++) {
    processes[i] = kvmalloc(sizeof(struct process_data));
    processes[i]->pid = -1;
    processes[i]->score = -1;
  }

  kill_work_queue = create_workqueue("simple_lmk");
  queue_delayed_work(kill_work_queue, &kill_task_work, 1000);
  return 0;
}

static const struct kernel_param_ops simple_lmk_init_ops = {
    .set = simple_lmk_init_set};

/* Needed to prevent Android from thinking there's no LMK and thus rebooting */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "lowmemorykiller."
module_param_cb(minfree, &simple_lmk_init_ops, NULL, 0200);
