/*
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
 *
 */

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/cpuquiet.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <asm/cputime.h>

#include "cpuquiet.h"

struct cpuquiet_cpu_stat {
	cputime64_t time_up_total;
	u64 last_update;
	unsigned int up_down_count;
	struct kobject cpu_kobject;
};

struct cpu_attribute {
	struct attribute attr;
	enum { up_down_count, time_up_total } type;
};

static struct cpuquiet_driver *cpuquiet_curr_driver;
struct cpuquiet_cpu_stat *stats;

#define CPU_ATTRIBUTE(_name) \
	static struct cpu_attribute _name ## _attr = {			\
		.attr =  {.name = __stringify(_name), .mode = 0444 },	\
		.type	= _name,					\
}

CPU_ATTRIBUTE(up_down_count);
CPU_ATTRIBUTE(time_up_total);

static struct attribute *cpu_attributes[] = {
	&up_down_count_attr.attr,
	&time_up_total_attr.attr,
	NULL,
};

static void stats_update(struct cpuquiet_cpu_stat *stat, bool up)
{
	u64 cur_jiffies = get_jiffies_64();
	bool was_up = stat->up_down_count & 0x1;

	if (was_up)
		stat->time_up_total = cputime64_add(stat->time_up_total,
			cputime64_sub(cur_jiffies, stat->last_update));

	if (was_up != up)
		stat->up_down_count++;

	stat->last_update = cur_jiffies;
}

int cpuquiet_quiesence_cpu(unsigned int cpunumber)
{
	int err = -EPERM;

	if (cpuquiet_curr_driver && cpuquiet_curr_driver->quiesence_cpu)
		err = cpuquiet_curr_driver->quiesence_cpu(cpunumber);

	if (!err)
		stats_update(stats + cpunumber, 0);

	return err;
}
EXPORT_SYMBOL(cpuquiet_quiesence_cpu);

int cpuquiet_wake_cpu(unsigned int cpunumber)
{
	int err = -EPERM;

	if (cpuquiet_curr_driver && cpuquiet_curr_driver->wake_cpu)
		err = cpuquiet_curr_driver->wake_cpu(cpunumber);

	if (!err)
		stats_update(stats + cpunumber, 1);

	return err;
}
EXPORT_SYMBOL(cpuquiet_wake_cpu);

static ssize_t stats_sysfs_show(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	struct cpu_attribute *cattr =
		container_of(attr, struct cpu_attribute, attr);
	struct cpuquiet_cpu_stat *stat  =
		container_of(kobj, struct cpuquiet_cpu_stat, cpu_kobject);
	ssize_t len = 0;
	bool was_up = stat->up_down_count & 0x1;

	stats_update(stat, was_up);

	switch (cattr->type) {
	case up_down_count:
		len = sprintf(buf, "%u\n", stat->up_down_count);
		break;
	case time_up_total:
		len =  sprintf(buf, "%llu\n", stat->time_up_total);
		break;
	}

	return len;
}

static const struct sysfs_ops stats_sysfs_ops = {
	.show = stats_sysfs_show,
};

static struct kobj_type ktype_cpu_stats = {
	.sysfs_ops = &stats_sysfs_ops,
	.default_attrs = cpu_attributes,
};

int cpuquiet_register_driver(struct cpuquiet_driver *drv)
{
	int err = -EBUSY;
	unsigned int cpu;
	struct cpuquiet_governor *governor;
	struct sys_device *sys_dev;
	u64 cur_jiffies;

	if (!drv)
		return -EINVAL;

	stats = kzalloc(nr_cpu_ids * sizeof(*stats), GFP_KERNEL);
	if (!stats)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		cur_jiffies = get_jiffies_64();
		stats[cpu].last_update = cur_jiffies;
		if (cpu_online(cpu))
			stats[cpu].up_down_count = 1;
		sys_dev = get_cpu_sysdev(cpu);
		if (sys_dev) {
			cpuquiet_add_dev(sys_dev, cpu);
			cpuquiet_cpu_kobject_init(&stats[cpu].cpu_kobject,
					&ktype_cpu_stats, "stats", cpu);
		}
	}

	mutex_lock(&cpuquiet_lock);
	if (!cpuquiet_curr_driver) {
		err = 0;
		cpuquiet_curr_driver = drv;
#ifdef CONFIG_CPUQUIET_DEFAULT_GOV_BALANCED
        governor = cpuquiet_find_governor("balanced");
#endif
#ifdef CONFIG_CPUQUIET_DEFAULT_GOV_RUNNABLE
        governor = cpuquiet_find_governor("runnable");
#endif
        if (governor == NULL)
            governor = cpuquiet_get_first_governor();
		
		cpuquiet_switch_governor(governor); 
	}
	mutex_unlock(&cpuquiet_lock);

	return err;
}
EXPORT_SYMBOL(cpuquiet_register_driver);

struct cpuquiet_driver *cpuquiet_get_driver(void)
{
	return cpuquiet_curr_driver;
}

void cpuquiet_unregister_driver(struct cpuquiet_driver *drv)
{
	unsigned int cpu;

	if (drv != cpuquiet_curr_driver) {
		WARN(1, "invalid cpuquiet_unregister_driver(%s)\n",
			drv->name);
		return;
	}

	/* stop current governor first */
	cpuquiet_switch_governor(NULL);

	mutex_lock(&cpuquiet_lock);
	cpuquiet_curr_driver = NULL;

	for_each_possible_cpu(cpu) {
		kobject_put(&stats[cpu].cpu_kobject);
		cpuquiet_remove_dev(cpu);
	}

	mutex_unlock(&cpuquiet_lock);
}
EXPORT_SYMBOL(cpuquiet_unregister_driver);
