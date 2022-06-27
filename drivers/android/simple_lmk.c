// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2021 Sultan Alsawaf <sultan@kerneltoast.com>.
 * Copyright (C) 2022- Soo-Hwan Na <whiteshell2544@naver.com>
 */

#define pr_fmt(fmt) "simple_lmk: " fmt

#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/fdtable.h>
#include <linux/freezer.h>
#include <linux/pid_namespace.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
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
#define MAX_FOREGROUND 16

struct victim_info {
  struct task_struct *tsk;
  struct mm_struct *mm;
  unsigned long size;
};

static struct victim_info victims[MAX_VICTIMS] __cacheline_aligned_in_smp;
static struct task_struct *task_bucket[SHRT_MAX + 1] __cacheline_aligned_in_smp;
static int nr_victims;

static int max = 85;

module_param(max, int, 0664);

static void simple_lmk_reclaim_work(struct work_struct *data);
static struct workqueue_struct *kill_work_queue;
static DECLARE_DELAYED_WORK(kill_task_work, simple_lmk_reclaim_work);

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

struct process_data {
  int pid;
  int uid;
  unsigned long score;
};

static struct process_data *processes[MAX_VICTIMS];
static int foreground[MAX_FOREGROUND];

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

static void put_new_foreground (int pid) {
	int tmp[MAX_FOREGROUND], i;
	for (i = 0; i < MAX_FOREGROUND; i++) {
	    if (foreground[i] == pid)
		return;
	}
	for (i = 0; i < MAX_FOREGROUND - 1; i++) {
		tmp[i] = foreground[i + 1];
	}
	tmp[MAX_FOREGROUND - 1] = pid;
	for (i = 0; i < MAX_FOREGROUND; i++) {
		foreground[i] = tmp[i];
	}
}

static bool check_fd_for_hwbinder(struct task_struct *tsk) {
  struct files_struct *current_files;
  struct fdtable *files_table;
  int i = 0;
  struct path files_path;
  char *cwd;
  char *buf = (char *)kmalloc(PAGE_SIZE, GFP_KERNEL);

  // The task is dead here, but we need to skip killing this task, therefore
  // return true.
  if (!pid_alive(tsk)) {
    pr_err("%s: [ERR] task is dead. return\n", __func__);
    kfree(buf);
    return true;
  }

  task_lock(tsk);
  current_files = tsk->files;
  if (!current_files) {
    pr_err("%s: [ERR] task->files is NULL. return\n", __func__);
    kfree(buf);
    task_unlock(tsk);
    return false;
  }
  files_table = files_fdtable(current_files);
  if (!files_table) {
    pr_err("%s: [ERR] files_fdtable return value is NULL. return\n", __func__);
    kfree(buf);
    task_unlock(tsk);
    return false;
  }

  while (files_table->fd[i] && i <= files_table->max_fds) {
    files_path = files_table->fd[i]->f_path;
    cwd = d_path(&files_path, buf, PAGE_SIZE);
    if (!cwd) {
	    i++;
	    continue;
    }
    if (strstr(cwd, "/dev/hwbinder")) {
      pr_info("%s: [INFO] comm: %s has /dev/hwbinder open as fd %d\n", __func__, tsk->comm, i);
      put_new_foreground(tsk->pid);
      kfree(buf);
      task_unlock(tsk);
      return true;
    }
    i++;
  }
  kfree(buf);
  task_unlock(tsk);
  return false;
}

static void scan_and_kill(void) {
  int nr_found = 0, i;
  unsigned long pages_found;
  struct task_struct *tsk;

  sort(processes, MAX_VICTIMS, sizeof(struct process_data *), &compare_mm, NULL);

  for (i = 0; i < MAX_VICTIMS; i++) {
    for_each_process(tsk) {
      if (tsk->pid == processes[i]->pid) {
	struct task_struct *vtsk;
        bool is_foreground = check_fd_for_hwbinder(tsk);
	int ppid, k;

	for (k = 0; k < MAX_FOREGROUND; k++) {
		if (foreground[k] == tsk->pid) {
			is_foreground = true;
			break;
		}
		for_each_process(vtsk) {
			if (foreground[k] == vtsk->pid) {
				if (processes[k]->uid == task_uid(vtsk).val) {
					is_foreground = true;
					break;
				}

				if (task_uid(rcu_dereference(tsk->real_parent)).val == task_uid(vtsk).val) {
					pr_info("%s: [SKIP] comm: %s, pid: %d, due to task parent UID\n", __func__, tsk->comm, tsk->pid);
					is_foreground = true;
					break;
				}
			}
		}
	}
       	rcu_read_lock();
	ppid = pid_alive(tsk) ? task_tgid_nr_ns(rcu_dereference(tsk->real_parent), 
			task_active_pid_ns(tsk)) : 0;

	if (check_fd_for_hwbinder(rcu_dereference(tsk->real_parent))) {
		pr_info("%s: [SKIP] comm: %s, pid: %d, found parent comm: %s, pid: %d\n", __func__, tsk->comm, tsk->pid,
			       rcu_dereference(tsk->real_parent)->comm, rcu_dereference(tsk->real_parent)->pid);
		is_foreground = true;
	}
	rcu_read_unlock();
        if ((is_foreground && get_mm_usage() < max) || strcmp(tsk->comm, "su") == 0){
	  pr_info("%s: [SKIP] comm: %s, is_foreground: %d, uid: %d\n", __func__, tsk->comm, 
			  is_foreground ? 1 : 0, processes[i]->uid);
	  continue;
	}
        rcu_read_lock();
        pr_info("%s: [KILL] comm: %s, pid: %d, ppid: %d, system_mm_usage: %d\n", __func__,
                tsk->comm, tsk->pid, ppid, get_mm_usage());
        task_lock(tsk);
        kill_task(tsk);
        rcu_read_unlock();
	usleep_range(150000, 200000);
      }
    }
  }

  // Reset the collected data
  for (i = 0; i < MAX_VICTIMS; i++) {
    processes[i]->pid = -1;
    processes[i]->uid = -1;
    processes[i]->score = -1;
  }

  /* Populate the victims array with tasks sorted by adj and then size */
  pages_found = find_victims(&nr_found);
  if (unlikely(!nr_found)) {
    pr_err("[ERR] No processes available to kill!\n");
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
    processes[i]->score = oom_badness(vtsk, NULL, NULL, totalpages) * 1000 / totalpages;
    task_lock(vtsk);
    processes[i]->uid = task_uid(vtsk).val;
    pr_info("%s: comm: %s, pid: %d, uid: %d\n", __func__, vtsk->comm, processes[i]->pid, processes[i]->uid);
    task_unlock(vtsk);
  }

  nr_victims = 0;
}

static void simple_lmk_reclaim_work(struct work_struct *data) {
  scan_and_kill();
  queue_delayed_work(kill_work_queue, &kill_task_work, 2500);
}

/* Initialize Simple LMK when lmkd in Android writes to the minfree parameter */
static int simple_lmk_init_set(const char *val, const struct kernel_param *kp) {
  int i;

  // Init values
  for (i = 0; i < MAX_VICTIMS; i++) {
    processes[i] = kvmalloc(sizeof(struct process_data));
    processes[i]->pid = -1;
    processes[i]->uid = -1;
    processes[i]->score = -1;
  }

  for (i = 0; i < MAX_FOREGROUND; i++)
	  foreground[i] = 0;

  kill_work_queue = create_workqueue("simple_lmk");
  queue_delayed_work(kill_work_queue, &kill_task_work, 2500);
  return 0;
}

static const struct kernel_param_ops simple_lmk_init_ops = {
    .set = simple_lmk_init_set
};

/* Needed to prevent Android from thinking there's no LMK and thus rebooting */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "lowmemorykiller."
module_param_cb(minfree, &simple_lmk_init_ops, NULL, 0200);
