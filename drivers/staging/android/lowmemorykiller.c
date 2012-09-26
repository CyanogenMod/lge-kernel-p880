/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/writeback.h>
#include <linux/swap.h>
#include <linux/hardirq.h>

extern void drop_pagecache_sb(struct super_block *sb, void *unused);

static uint32_t lowmem_debug_level = 2;
static uint32_t enable_filecache_check = 150;
static long pick_task_runtime = 10;

static int lowmem_adj[6] = {
	0, 58, 176, 294, 411, 529
};

static long sleep_intervel[6] = {
	4,	4,	3,	2,	0,	0
};
#define SLEEP_INTERVEL_MAX (20)
static int sleep_intervel_size = 6;
static int lowmem_adj_size = 6;
static uint drop_cache_intervel;

static int lowmem_minfree[6] = {
	9*256-21, 14*256+1, 23*256+1, 31*256+1, 41*256+1, 51*256+1
};
static int lowmem_minfree_size = 6;
static long shrink_batch = 10*256;
static ktime_t lowmem_deathpending_timeout;

#define LMK_BUSY (-1)
#define LMK_LOG_TAG "lowmem_shrink "

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			printk(LMK_LOG_TAG x);			\
	} while (0)

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	static DEFINE_SPINLOCK(lowmem_lock);
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	int rem = 0;
	int pages_can_free = 0;
	static int same_count;
	static int busy_count;
	static int busy_count_dropped;
	static int oldpid;
	static int lastpid;
	static ktime_t next_busy_print;
	int tasksize;
	int adj_index;
	long sleep_time = -1;
	int min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int selected_tasksize = 0;
	int selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES);
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM);

	pages_can_free = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);


	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

	if (!enable_filecache_check) {
		rem = pages_can_free/2;
		for (adj_index = 0; adj_index < array_size; adj_index++) {
			if (other_free < lowmem_minfree[adj_index]) {
				min_score_adj = lowmem_adj[adj_index];
				break;
			}
		}
	} else {
		rem = 0;
		for (adj_index = 0; adj_index < array_size; adj_index++) {
			if (other_free < lowmem_minfree[adj_index]) {
				sleep_time = sleep_intervel[adj_index];
				if (other_file < enable_filecache_check*256) {
					min_score_adj = lowmem_adj[adj_index];
					rem = pages_can_free/2;
				}
				break;
			}
		}
	}

	lowmem_print(5, "sleep_intervel info  adj_index %d\
, sleep_invl[0] %ld, array_size %d, sleep_invl[] %ld\n",
		adj_index, sleep_intervel[0], array_size,
		sleep_intervel[adj_index]);
	if (sleep_time > 0 && sleep_time <= SLEEP_INTERVEL_MAX &&
		!current_is_kswapd()) {
		lowmem_print(3, "sleep %ld jiffies\n", sleep_time);
		schedule_timeout(sleep_time);
	}

	if (sc->nr_to_scan > 0)
		lowmem_print(4, "info%s: nr_to_scan %lu, \
gfp_mask %x, other_free %d other_file %d, min_score_adj %d\n",
		current_is_kswapd() ? " swap" : " ", sc->nr_to_scan,
		sc->gfp_mask, other_free,
				other_file, min_score_adj);

	if (sc->nr_to_scan <= 0 || min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(4, "info%s: check return %d,\
nr_to_scan %lu, gfp_mask %x\n", current_is_kswapd() ? " swap" : " ",
	rem, sc->nr_to_scan, sc->gfp_mask);
		return rem;
	}
	selected_oom_score_adj = min_score_adj;

	if (spin_trylock(&lowmem_lock) == 0) {
		if (ktime_us_delta(ktime_get(), next_busy_print) > 0) {
			lowmem_print(3, "busy count %d \
busy_count_dropped %d index %d\n",
			busy_count, busy_count_dropped, adj_index);
			next_busy_print = ktime_add(ktime_get(),
						    ktime_set(5, 0));
			busy_count_dropped = 0;
		}
		busy_count++;
		busy_count_dropped++;
		return LMK_BUSY;
	}
	/* turn of scheduling to protect task list */
	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		int oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		if (test_tsk_thread_flag(p, TIF_MEMDIE) &&
			ktime_us_delta(ktime_get(),
				lowmem_deathpending_timeout) < 0) {
			task_unlock(p);
			same_count++;
			if (p->pid != oldpid || same_count > 1000) {
				lowmem_print(5,
					"terminate %d (%s) oldpid:%d lastpid:%d %ld %d\n",
					p->pid, p->comm, oldpid, lastpid,
					(long)ktime_us_delta(ktime_get(),
						lowmem_deathpending_timeout),
					same_count);
				oldpid = p->pid;
				same_count = 0;
			}
			rcu_read_unlock();
			spin_unlock(&lowmem_lock);
			lowmem_print(3,	"waitkill %d (%s) state:%ld flag:0x%x \
count:%d index %d\n",
				p->pid, p->comm, p->state, p->flags,
				busy_count, adj_index);
			/* wait one jiffie */
			schedule_timeout(2);
			return LMK_BUSY;
		}
		oom_score_adj = p->signal->oom_score_adj;
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}
		if (pick_task_runtime) {
			long time_tmp = p->real_start_time.tv_sec;
			lowmem_print(5, "ignchk task %d (%s) \
run time %ld secs, threshold %ld secs, adj %d\n",
				p->pid, p->comm, time_tmp,
				pick_task_runtime, oom_score_adj);
			if (time_tmp < pick_task_runtime) {
				lowmem_print(3, "ignore task %d(%s) \
run time %ld, oom_score_adj %d\n", p->pid,
				p->comm, time_tmp, oom_score_adj);
				task_unlock(p);
				continue;
			}
		}

		tasksize = get_mm_rss(p->mm);
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(3, "selected %d (%s), adj %d, \
size %d, to kill\n", p->pid, p->comm,
			oom_score_adj, tasksize);
	}
	if (selected) {
		lowmem_print(1, "send sigkill to %d (%s), \
index %d, adj %d, size %d\n", selected->pid, selected->comm,
		adj_index, selected_oom_score_adj, selected_tasksize);
		send_sig(SIGKILL, selected, 0);

		lowmem_deathpending_timeout = ktime_add_ns(ktime_get(),
							   NSEC_PER_SEC/2);
		lowmem_print(4, "selected state:%ld flag:0x%x \
busy:%d %d\n", selected->state, selected->flags,
			     busy_count, oom_killer_disabled);
		lastpid = selected->pid;
		set_tsk_thread_flag(selected, TIF_MEMDIE);
		rem -= selected_tasksize;
	}
	lowmem_print(4, "normal return %lu, %x, return %d\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	rcu_read_unlock();

	if (selected && drop_cache_intervel && (adj_index == 0) &&
		(other_file > 70*256)) {
		static unsigned long last_drop = 10*HZ;
		if (time_after(jiffies, last_drop+(drop_cache_intervel*HZ))) {
		lowmem_print(1, "run drop_cache. \
other_file %d pages\n", other_file);
			last_drop = jiffies;
			spin_unlock(&lowmem_lock);
			iterate_supers(drop_pagecache_sb, NULL);
			return rem;
		}
	}
	spin_unlock(&lowmem_lock);
	if (selected)
		schedule_timeout(2);
	return rem;
}

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16,
	.batch = 10*256
};

static int __init lowmem_init(void)
{
	register_shrinker(&lowmem_shrinker);
	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static int lowmem_oom_adj_to_oom_score_adj(int oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	int oom_adj;
	int oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(2, "convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(2, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_int,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

static int set_shrink_batch(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_long(val, kp);
	if (shrink_batch > 127 && shrink_batch < 8192) {
		lowmem_print(2, "batch %ld => %ld\n",
				 lowmem_shrinker.batch, shrink_batch);
		lowmem_shrinker.batch = shrink_batch;
	}

	return ret;
}

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
__module_param_call(MODULE_PARAM_PREFIX, adj,
		    &lowmem_adj_array_ops,
		    .arr = &__param_arr_adj,
		    S_IRUGO | S_IWUSR, 0);
__MODULE_PARM_TYPE(adj, "array of int");
#else
module_param_array_named(adj, lowmem_adj, int, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(enable_filecache_check, enable_filecache_check,
			uint, S_IRUGO | S_IWUSR);
module_param_named(pick_task_runtime, pick_task_runtime,
			long, S_IRUGO | S_IWUSR);
module_param_array_named(sleep_intervel, sleep_intervel, long,
	&sleep_intervel_size, S_IRUGO | S_IWUSR);
module_param_named(drop_cache_intervel, drop_cache_intervel,
		uint, S_IRUGO | S_IWUSR);
module_param_call(shrink_batch, set_shrink_batch, param_get_long,
		  &shrink_batch, 0644);
__MODULE_PARM_TYPE(shrink_batch, "long");

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

