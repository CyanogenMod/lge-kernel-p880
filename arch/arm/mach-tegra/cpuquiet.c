/*
 * arch/arm/mach-tegra/cpuquiet.c
 *
 * Cpuquiet driver for Tegra3 CPUs
 *
 * Copyright (c) 2012 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_qos_params.h>
#include <linux/cpuquiet.h>

#include "pm.h"
#include "cpu-tegra.h"
#include "clock.h"
#include "cpuquiet.h"

#define CPUQUIET_DEBUG 1

#define INITIAL_STATE		TEGRA_CPQ_IDLE
#define UP_DELAY_MS			70
#define DOWN_DELAY_MS		500

static struct mutex *tegra3_cpu_lock;
static struct workqueue_struct *cpuquiet_wq;
static struct delayed_work cpuquiet_work;
static struct work_struct minmax_work;

static struct kobject *tegra_auto_sysfs_kobject;

static bool no_lp;
static bool enable;
static unsigned long up_delay;
static unsigned long down_delay;
static int mp_overhead = 10;
static unsigned int idle_top_freq;
static unsigned int idle_bottom_freq;
static int lpup_req = 0;
static int lpdown_req = 0;

static struct clk *cpu_clk;
static struct clk *cpu_g_clk;
static struct clk *cpu_lp_clk;

static struct cpumask cr_online_requests;
static cputime64_t lp_on_time;
static unsigned int min_cpus = 1;
static unsigned int max_cpus = CONFIG_NR_CPUS;

#define CPUQUIET_TAG                       "[CPUQUIET]: "
/*
 * LPCPU hysteresis default values
 * we need at least 5 requests to go into lpmode and
 * we need at least 2 requests to come out of lpmode.
 */
#define TEGRA_CPQ_LPCPU_UP_HYS        4
#define TEGRA_CPQ_LPCPU_DOWN_HYS      2

enum {
	TEGRA_CPQ_DISABLED = 0,
	TEGRA_CPQ_IDLE,
	TEGRA_CPQ_SWITCH_TO_LP,
	TEGRA_CPQ_SWITCH_TO_G,
};

static unsigned int best_core_to_turn_up (void) {
    /* mitigate high temperature, 0 -> 3 -> 2 -> 1 */
    if (!cpu_online (3))
        return 3;

    if (!cpu_online (2))
        return 2;

    if (!cpu_online (1))
        return 1;

    /* NOT found, return >= nr_cpu_id */
    return nr_cpu_ids;
}

static inline unsigned int num_cpu_check(unsigned int num)
{
	if (num > CONFIG_NR_CPUS)
		return CONFIG_NR_CPUS;
	if (num < 1)
		return 1;
	return num;
}

unsigned int tegra_cpq_max_cpus(void)
{
	unsigned int max_cpus_qos = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS);	
	unsigned int num = min(max_cpus_qos, max_cpus);
	return num_cpu_check(num);
}

unsigned int tegra_cpq_min_cpus(void)
{
	unsigned int min_cpus_qos = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	unsigned int num = max(min_cpus_qos, min_cpus);
	return num_cpu_check(num);
}

static inline void show_status(const char* extra, cputime64_t on_time, int cpu)
{
#if CPUQUIET_DEBUG
	if(on_time)
		pr_info(CPUQUIET_TAG "%s Mask=[%d.%d%d%d%d]|lp_on_time=%llu\n",
    		extra, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
        	cpu_online(1), cpu_online(2), cpu_online(3), on_time);
	else		
		if(cpu>0)
			pr_info(CPUQUIET_TAG "%s %d Mask=[%d.%d%d%d%d]\n",
    			extra, cpu, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
        		cpu_online(1), cpu_online(2), cpu_online(3));

		else
			pr_info(CPUQUIET_TAG "%s Mask=[%d.%d%d%d%d]\n",
    			extra, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
        		cpu_online(1), cpu_online(2), cpu_online(3));
#endif
}

static int cpq_state;

static int update_core_config(unsigned int cpunumber, bool up)
{
	int ret = -EINVAL;
	unsigned int nr_cpus = num_online_cpus();
	int max_cpus = tegra_cpq_max_cpus();
	int min_cpus = tegra_cpq_min_cpus();

	if (cpq_state == TEGRA_CPQ_DISABLED || cpunumber >= nr_cpu_ids)
		return ret;

	if (up) {
		if(is_lp_cluster()) {
			cpumask_set_cpu(cpunumber, &cr_online_requests);
			ret = -EBUSY;
		} else {
			if (nr_cpus < max_cpus){
				ret = cpu_up(cpunumber);
				show_status("UP", 0, cpunumber);
			}
		}
	} else {
		if (is_lp_cluster()) {
			ret = -EBUSY;
		} else {
			if (nr_cpus > 1 && nr_cpus > min_cpus){
				ret = cpu_down(cpunumber);
				show_status("DOWN", 0, cpunumber);
			}
		}
	}
		
	return ret;
}

static int tegra_quiesence_cpu(unsigned int cpunumber)
{
        return update_core_config(cpunumber, false);
}

static int tegra_wake_cpu(unsigned int cpunumber)
{
        return update_core_config(cpunumber, true);
}

static struct cpuquiet_driver tegra_cpuquiet_driver = {
        .name                   = "tegra",
        .quiesence_cpu          = tegra_quiesence_cpu,
        .wake_cpu               = tegra_wake_cpu,
};

static void apply_core_config(void)
{
	unsigned int cpu;

	if (is_lp_cluster() || cpq_state == TEGRA_CPQ_DISABLED)
		return;

	for_each_cpu_mask(cpu, cr_online_requests) {
		if (cpu < nr_cpu_ids && !cpu_online(cpu))
			if (!tegra_wake_cpu(cpu))
				cpumask_clear_cpu(cpu, &cr_online_requests);
	}
}

static void tegra_cpuquiet_work_func(struct work_struct *work)
{
	int device_busy = -1;
	cputime64_t on_time = 0;

	mutex_lock(tegra3_cpu_lock);

	switch(cpq_state) {
		case TEGRA_CPQ_DISABLED:
		case TEGRA_CPQ_IDLE:
			break;
		case TEGRA_CPQ_SWITCH_TO_G:
			if (is_lp_cluster()) {
				if(!clk_set_parent(cpu_clk, cpu_g_clk)) {
					on_time = ktime_to_ms(ktime_get()) - lp_on_time;
					show_status("LP -> off", on_time, -1);
					/*catch-up with governor target speed */
					tegra_cpu_set_speed_cap(NULL);
					/* process pending core requests*/
					device_busy = 0;
				}
			}
			break;
		case TEGRA_CPQ_SWITCH_TO_LP:
			if (!is_lp_cluster() && !no_lp &&
				!(pm_qos_request(PM_QOS_MIN_ONLINE_CPUS) >= 2)
				&& num_online_cpus() == 1) {
				if (!clk_set_parent(cpu_clk, cpu_lp_clk)) {
					show_status("LP -> on", 0, -1);
					/*catch-up with governor target speed*/
					tegra_cpu_set_speed_cap(NULL);
					device_busy = 1;
					lp_on_time = ktime_to_ms(ktime_get());
				}
			}
			break;
		default:
			pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, cpq_state);
	}

	mutex_unlock(tegra3_cpu_lock);

	if (device_busy == 1) {
		cpuquiet_device_busy();
	} else if (!device_busy) {
		apply_core_config();
		cpuquiet_device_free();
	}
}

static void min_max_constraints_workfunc(struct work_struct *work)
{
	int count = -1;
	bool up = false;
	unsigned int cpu;

	int nr_cpus = num_online_cpus();
/*<<<<<<< HEAD
	int max_cpus = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS) ? : 4;
	int min_cpus = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
*/
	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

// ======= 
	int max_cpus = tegra_cpq_max_cpus();
	int min_cpus = tegra_cpq_min_cpus();
	
//>>>>>>> eda20e2... tegra: correct handling of num cpu constrains
	if (is_lp_cluster())
		return;

	if (nr_cpus < min_cpus) {
		up = true;
		count = min_cpus - nr_cpus;
	} else if (nr_cpus > max_cpus && max_cpus >= min_cpus) {
		count = nr_cpus - max_cpus;
	}

	for (;count > 0; count--) {
		if (up) {
			cpu = best_core_to_turn_up();
			if (cpu < nr_cpu_ids)
				cpu_up(cpu);
			else
				break;
		} else {
			cpu = cpumask_next(0, cpu_online_mask);
			if (cpu < nr_cpu_ids)
				cpu_down(cpu);
			else
				break;
		}
	}
}

static void min_cpus_change(void)
{
	bool g_cluster = false;
/*<<<<<<< HEAD

	if (cpq_state == TEGRA_CPQ_DISABLED)
		return NOTIFY_OK;

	mutex_lock(tegra3_cpu_lock);

	if ((n >= 1) && is_lp_cluster()) {
		/* make sure cpu rate is within g-mode
		 * range before switching */
//======= 
    cputime64_t on_time = 0;
	
	mutex_lock(tegra3_cpu_lock);

	if ((tegra_cpq_min_cpus() >= 1) && is_lp_cluster()) {
		/* make sure cpu rate is within g-mode range before switching */
//>>>>>>> eda20e2... tegra: correct handling of num cpu constrains
		unsigned long speed = max((unsigned long)tegra_getspeed(0),
					clk_get_min_rate(cpu_g_clk) / 1000);
		tegra_update_cpu_speed(speed);

		on_time = ktime_to_ms(ktime_get()) - lp_on_time;
		show_status("LP -> off - min_cpus_notify", on_time, -1);

		clk_set_parent(cpu_clk, cpu_g_clk);
		g_cluster = true;
	}

	tegra_cpu_set_speed_cap(NULL);
	mutex_unlock(tegra3_cpu_lock);

	schedule_work(&minmax_work);

	if (g_cluster)
		cpuquiet_device_free();
}

static int min_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
	pr_info("PM QoS PM_QOS_MIN_ONLINE_CPUS %lu\n", n);

	if (n < 1 || n > CONFIG_NR_CPUS)
		return NOTIFY_OK;
	
	min_cpus_change();

	return NOTIFY_OK;
}

static void max_cpus_change(void)
{	
	if (tegra_cpq_max_cpus() < num_online_cpus())
		schedule_work(&minmax_work);
}

static int max_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
/*<<<<<<< HEAD
	if (cpq_state == TEGRA_CPQ_DISABLED)
		return NOTIFY_OK;

	if (n < num_online_cpus())
		schedule_work(&minmax_work);
=======*/
	pr_info("PM QoS PM_QOS_MAX_ONLINE_CPUS %lu\n", n);

	if (n < 1)
		return NOTIFY_OK;
	
	max_cpus_change();
//>>>>>>> eda20e2... tegra: correct handling of num cpu constrains

	return NOTIFY_OK;
}

void tegra_cpuquiet_force_gmode(void)
{
    cputime64_t on_time = 0;
    unsigned long speed;

	if (!is_g_cluster_present())
		return;

	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

	if (is_lp_cluster() && cpq_state != TEGRA_CPQ_SWITCH_TO_G) {
		mutex_lock(tegra3_cpu_lock);

		/* make sure cpu rate is within g-mode range before switching */
		speed = max((unsigned long)tegra_getspeed(0),
					clk_get_min_rate(cpu_g_clk) / 1000);
		tegra_update_cpu_speed(speed);

		on_time = ktime_to_ms(ktime_get()) - lp_on_time;
		show_status("LP -> off - force", on_time, -1);

		clk_set_parent(cpu_clk, cpu_g_clk);

        lpup_req = 0;
        lpdown_req = 0;

    	mutex_unlock(tegra3_cpu_lock);

		cpuquiet_device_free();
	}
}

void tegra_auto_hotplug_governor(unsigned int cpu_freq, bool suspend)
{
	if (!is_g_cluster_present())
		return;

	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

	if (suspend) {
		cpq_state = TEGRA_CPQ_IDLE;
        lpup_req = 0;
        lpdown_req = 0;
		return;
	}

	if (is_lp_cluster() && pm_qos_request(PM_QOS_MIN_ONLINE_CPUS) >= 2) {
		if (cpq_state != TEGRA_CPQ_SWITCH_TO_G) {
			/* Force switch */
			cpq_state = TEGRA_CPQ_SWITCH_TO_G;
            lpup_req = 0;
            lpdown_req = 0;
			queue_delayed_work(
				cpuquiet_wq, &cpuquiet_work, up_delay);
		}
		return;
	}

	if (is_lp_cluster() && (cpu_freq >= idle_top_freq || no_lp)) {
		lpdown_req++;
		lpup_req = 0;
        if (lpdown_req > TEGRA_CPQ_LPCPU_DOWN_HYS) {
       		cpq_state = TEGRA_CPQ_SWITCH_TO_G;
       		lpdown_req = 0;
			queue_delayed_work(cpuquiet_wq, &cpuquiet_work, up_delay);
		}
	} else if (!is_lp_cluster() && !no_lp &&
		   cpu_freq <= idle_bottom_freq) {
		lpup_req++;
		lpdown_req = 0;
        if (lpup_req > TEGRA_CPQ_LPCPU_UP_HYS) {
			cpq_state = TEGRA_CPQ_SWITCH_TO_LP;
			lpup_req = 0;
			queue_delayed_work(cpuquiet_wq, &cpuquiet_work, down_delay);
		}
	} else {
		cpq_state = TEGRA_CPQ_IDLE;
        lpup_req = 0;
        lpdown_req = 0;
	}
}

static struct notifier_block min_cpus_notifier = {
	.notifier_call = min_cpus_notify,
};

static struct notifier_block max_cpus_notifier = {
	.notifier_call = max_cpus_notify,
};

static void delay_callback(struct cpuquiet_attribute *attr)
{
	unsigned long val;

	if (attr) {
		val = (*((unsigned long *)(attr->param)));
		(*((unsigned long *)(attr->param))) = msecs_to_jiffies(val);
	}
}

static void enable_callback(struct cpuquiet_attribute *attr)
{
	mutex_lock(tegra3_cpu_lock);

	if (!enable && cpq_state != TEGRA_CPQ_DISABLED) {
		cpq_state = TEGRA_CPQ_DISABLED;
		mutex_unlock(tegra3_cpu_lock);
		cancel_delayed_work_sync(&cpuquiet_work);
		pr_info("Tegra cpuquiet clusterswitch disabled\n");
		cpuquiet_device_busy();
		mutex_lock(tegra3_cpu_lock);
	} else if (enable && cpq_state == TEGRA_CPQ_DISABLED) {
		cpq_state = TEGRA_CPQ_IDLE;
		pr_info("Tegra cpuquiet clusterswitch enabled\n");
		tegra_cpu_set_speed_cap(NULL);
		cpuquiet_device_free();
	}

	mutex_unlock(tegra3_cpu_lock);
}

ssize_t show_min_cpus(struct cpuquiet_attribute *cattr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%d\n", min_cpus);

	return out - buf;
}

ssize_t store_min_cpus(struct cpuquiet_attribute *cattr,
					const char *buf, size_t count)
{
	int ret;
	unsigned int n;
	
	ret = sscanf(buf, "%d", &n);

	if ((ret != 1) || n < 1 || n > CONFIG_NR_CPUS)
		return -EINVAL;
	
	min_cpus = n;
	min_cpus_change();
	
	return count;
}

ssize_t show_max_cpus(struct cpuquiet_attribute *cattr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%d\n", max_cpus);

	return out - buf;
}

ssize_t store_max_cpus(struct cpuquiet_attribute *cattr,
					const char *buf, size_t count)
{
	int ret;
	unsigned int n;
	
	ret = sscanf(buf, "%d", &n);

	if ((ret != 1) || n < 1 || n > CONFIG_NR_CPUS)
		return -EINVAL;

	max_cpus = n;	
	max_cpus_change();
		
	return count;
}

CPQ_BASIC_ATTRIBUTE(no_lp, 0644, bool);
CPQ_BASIC_ATTRIBUTE(idle_top_freq, 0644, uint);
CPQ_BASIC_ATTRIBUTE(idle_bottom_freq, 0644, uint);
CPQ_BASIC_ATTRIBUTE(mp_overhead, 0644, int);
CPQ_ATTRIBUTE(up_delay, 0644, ulong, delay_callback);
CPQ_ATTRIBUTE(down_delay, 0644, ulong, delay_callback);
CPQ_ATTRIBUTE(enable, 0644, bool, enable_callback);
//CPQ_ATTRIBUTE_CUSTOM(min_cpus, 0644, show_min_cpus, store_min_cpus);
//CPQ_ATTRIBUTE_CUSTOM(max_cpus, 0644, show_max_cpus, store_max_cpus);

static struct attribute *tegra_auto_attributes[] = {
	&no_lp_attr.attr,
	&up_delay_attr.attr,
	&down_delay_attr.attr,
	&idle_top_freq_attr.attr,
	&idle_bottom_freq_attr.attr,
	&mp_overhead_attr.attr,
	&enable_attr.attr,
//	&min_cpus_attr.attr,
//	&max_cpus_attr.attr,
	NULL,
};

static const struct sysfs_ops tegra_auto_sysfs_ops = {
	.show = cpuquiet_auto_sysfs_show,
	.store = cpuquiet_auto_sysfs_store,
};

static struct kobj_type ktype_sysfs = {
	.sysfs_ops = &tegra_auto_sysfs_ops,
	.default_attrs = tegra_auto_attributes,
};

static int tegra_auto_sysfs(void)
{
	int err;

	tegra_auto_sysfs_kobject = kzalloc(sizeof(*tegra_auto_sysfs_kobject),
					GFP_KERNEL);

	if (!tegra_auto_sysfs_kobject)
		return -ENOMEM;

	err = cpuquiet_kobject_init(tegra_auto_sysfs_kobject, &ktype_sysfs,
				"tegra_cpuquiet");

	if (err)
		kfree(tegra_auto_sysfs_kobject);

	return err;
}

#ifdef CONFIG_DEBUG_FS

struct pm_qos_request_list min_cpu_req;
struct pm_qos_request_list max_cpu_req;

int min_cpus_get(void *data, u64 *val)
{
	*val = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	return 0;
}
int min_cpus_set(void *data, u64 val)
{
	pm_qos_update_request(&min_cpu_req, (s32)val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(min_cpus_fops, min_cpus_get, min_cpus_set, "%llu\n");

int max_cpus_get(void *data, u64 *val)
{
	*val = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS);
	return 0;
}
int max_cpus_set(void *data, u64 val)
{
	pm_qos_update_request(&max_cpu_req, (s32)val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(max_cpus_fops, max_cpus_get, max_cpus_set, "%llu\n");

#endif /* CONFIG_DEBUG_FS */


// int __cpuinit tegra_auto_hotplug_init(struct mutex *cpulock)
int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{
	int err;

	cpu_clk = clk_get_sys(NULL, "cpu");
	cpu_g_clk = clk_get_sys(NULL, "cpu_g");
	cpu_lp_clk = clk_get_sys(NULL, "cpu_lp");

	if (IS_ERR(cpu_clk) || IS_ERR(cpu_g_clk) || IS_ERR(cpu_lp_clk))
		return -ENOENT;

	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, freezable.
	 */
	cpuquiet_wq = alloc_workqueue(
		"cpuquiet", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);

	if (!cpuquiet_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&cpuquiet_work, tegra_cpuquiet_work_func);
	INIT_WORK(&minmax_work, min_max_constraints_workfunc);

	idle_top_freq = clk_get_max_rate(cpu_lp_clk) / 1000;
	idle_bottom_freq = clk_get_min_rate(cpu_g_clk) / 1000;

	up_delay = msecs_to_jiffies(UP_DELAY_MS);
	down_delay = msecs_to_jiffies(DOWN_DELAY_MS);
	cpumask_clear(&cr_online_requests);
	tegra3_cpu_lock = cpu_lock;

	cpq_state = INITIAL_STATE;
	enable = cpq_state == TEGRA_CPQ_DISABLED ? false : true;


	pr_info("Tegra cpuquiet initialized: %s\n",
		(cpq_state == TEGRA_CPQ_DISABLED) ? "disabled" : "enabled");

	if (pm_qos_add_notifier(PM_QOS_MIN_ONLINE_CPUS, &min_cpus_notifier))
		pr_err("%s: Failed to register min cpus PM QoS notifier\n",
			__func__);
	if (pm_qos_add_notifier(PM_QOS_MAX_ONLINE_CPUS, &max_cpus_notifier))
		pr_err("%s: Failed to register max cpus PM QoS notifier\n",
			__func__);

	err = cpuquiet_register_driver(&tegra_cpuquiet_driver);
	if (err) {
		destroy_workqueue(cpuquiet_wq);
		return err;
	}

	err = tegra_auto_sysfs();
	if (err) {
		cpuquiet_unregister_driver(&tegra_cpuquiet_driver);
		destroy_workqueue(cpuquiet_wq);
	}

	return err;
}

void tegra_auto_hotplug_exit(void)
{
	destroy_workqueue(cpuquiet_wq);
	cpuquiet_unregister_driver(&tegra_cpuquiet_driver);
	kobject_put(tegra_auto_sysfs_kobject);

#ifdef CONFIG_DEBUG_FS
	pm_qos_remove_request(&min_cpu_req);
	pm_qos_remove_request(&max_cpu_req);
#endif
}
