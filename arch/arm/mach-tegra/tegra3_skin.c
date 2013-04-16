/*
 * arch/arm/mach-tegra/tegra3_skin.c
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <mach/thermal.h>
#include <mach/edp.h>
#include <linux/slab.h>
#include <linux/syscalls.h>

#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"


#ifdef CONFIG_TEGRA_SKIN
extern struct thermal_cooling_device *skin_throttle_cdev;

static long skin_throttle_temp = 35000;
static long polling_period = 1100; /* 1.1 second */
static long cur_skin_temp;
static struct workqueue_struct *workqueue;
static struct delayed_work skin_work;
static struct thermal_zone_device *skin_thz;
static struct nct1008_data *nct;

static long icoeff[] = {-11, -7, -5, -3, -3, -2, -1, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 6, 11, 18};
static long ecoeff[] = {2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, -1, -7};
static long bcoeff[] = {-38, -11, -3, -2, -2, -1, -1, -1, 0, 11, -13, 6, 7, 5, 6, 6, 6, 6, 13, 55};
static int ntemp;
#define IHIST_LEN	(20)
#define EHIST_LEN	(20)
#define BHIST_LEN	(20)
static long ihist[IHIST_LEN];
static long ehist[EHIST_LEN];
static long bhist[IHIST_LEN];

static bool enable = 1;


static int skin_thermal_zone_bind(struct thermal_zone_device *thermal,
				struct thermal_cooling_device *cdevice) {

	if (skin_throttle_cdev == cdevice)
		return thermal_zone_bind_cooling_device(thermal, 0, cdevice);
	return 0;
}

static int skin_thermal_zone_unbind(struct thermal_zone_device *thermal,
				struct thermal_cooling_device *cdevice) {
	if (skin_throttle_cdev == cdevice)
		return thermal_zone_unbind_cooling_device(thermal, 0, cdevice);
	return 0;
}

static int skin_thermal_zone_get_temp(struct thermal_zone_device *thz,
						unsigned long *temp)
{
	*temp = cur_skin_temp;
	return 0;
}

static int skin_thermal_zone_get_trip_type(
			struct thermal_zone_device *thermal,
			int trip,
			enum thermal_trip_type *type) {
	if (trip != 0)
		return -EINVAL;

	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

static int skin_thermal_zone_get_trip_temp(struct thermal_zone_device *thz,
						int trip,
						unsigned long *temp) {
	if (trip != 0)
		return -EINVAL;

	*temp = skin_throttle_temp;

	return 0;
}

static struct thermal_zone_device_ops skin_thermal_zone_ops = {
	.bind = skin_thermal_zone_bind,
	.unbind = skin_thermal_zone_unbind,
	.get_temp = skin_thermal_zone_get_temp,
	.get_trip_type = skin_thermal_zone_get_trip_type,
	.get_trip_temp = skin_thermal_zone_get_trip_temp,
};

static int temp_get(long *itemp, long *etemp, long *btemp)
{
	char buf[64];
	int fd, ret, count;
	mm_segment_t old_fs;

	if ((ret = nct1008_thermal_get_temps(nct, etemp, itemp)))
		return -1;

	/* Hack: should get battery temp from somewhere else */
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open("/sys/class/power_supply/battery/temp", O_RDONLY, 0644);

	if (fd >= 0) {
		count = sys_read(fd, buf, 64);
		if (count < 64)
			buf[count] = '\0';
		ret = kstrtol(buf, 10, btemp);
		sys_close(fd);
	} else {
		ret = -1;
	}
	set_fs(old_fs);

	return ret;
}

static void skin_work_func(struct work_struct *work)
{
	int i, isum = 0, esum = 0, bsum = 0;

	if (temp_get(&ihist[ntemp % IHIST_LEN],
	             &ehist[ntemp % EHIST_LEN],
	             &bhist[ntemp % BHIST_LEN]))
		goto err;

	for (i = 0; i < IHIST_LEN; i++)
		isum += ihist[(ntemp - i + IHIST_LEN) % IHIST_LEN] * icoeff[i];
	for (i = 0; i < EHIST_LEN; i++)
		esum += ehist[(ntemp - i + EHIST_LEN) % EHIST_LEN] * ecoeff[i];
	for (i = 0; i < BHIST_LEN; i++)
		bsum += bhist[(ntemp - i + BHIST_LEN) % BHIST_LEN] * bcoeff[i];

	cur_skin_temp = (isum + esum) / 100 + bsum + 9793;

	ntemp++;

	/*if (cur_skin_temp >= skin_throttle_temp)
		if (skin_thz && !skin_thz->passive)
			thermal_zone_device_update(skin_thz);*/

err:
	queue_delayed_work(workqueue, &skin_work, msecs_to_jiffies(polling_period));
}

static int skin_enable_update(void)
{
	int ret = 0;

	if (enable && !skin_thz) {
		skin_thz = thermal_zone_device_register("skin",
				1, /* trips */
				NULL,
				&skin_thermal_zone_ops,
				0, /* dT/dt */
				1, /* throttle */
				5000,
				0); /* polling delay */
		if (IS_ERR(skin_thz)) {
				skin_thz = NULL;
				ret = -ENODEV;
		}
	} else if (!enable && skin_thz) {
		thermal_zone_device_unregister(skin_thz);
		skin_thz = NULL;
		tegra_skin_throttle_disable();
	}

	return ret;
}

int tegra_skin_set_nct(struct nct1008_data *data)
{
	long itemp, etemp, btemp;
	int i;
	nct = data;

	temp_get(&itemp, &etemp, &btemp);

	for (i = 0; i < EHIST_LEN; i++)
		ehist[i] = etemp;
	for (i = 0; i < IHIST_LEN; i++)
		ihist[i] = itemp;
	for (i = 0; i < BHIST_LEN; i++)
		bhist[i] = btemp;

	skin_enable_update();

	workqueue = alloc_workqueue("skin",
				    WQ_HIGHPRI | WQ_UNBOUND | WQ_RESCUER, 1);
	INIT_DELAYED_WORK(&skin_work, skin_work_func);

	queue_delayed_work(workqueue, &skin_work, msecs_to_jiffies(polling_period));

	return 0;
}

static int skin_enable_set(const char *arg, const struct kernel_param *kp)
{
	int ret = param_set_bool(arg, kp);

	if (ret == 0)
		ret = skin_enable_update();

	return ret;
}

static int skin_enable_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_uint(buffer, kp);
}

static struct kernel_param_ops skin_enable_ops = {
	.set = skin_enable_set,
	.get = skin_enable_get,
};
module_param_cb(enable, &skin_enable_ops, &enable, 0644);

static int skin_temp_set(const char *arg, const struct kernel_param *kp)
{
	int ret = param_set_long(arg, kp);

	return ret;
}

static int skin_temp_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops skin_temp_ops = {
	.set = skin_temp_set,
	.get = skin_temp_get,
};

module_param_cb(cur_skin_temp, &skin_temp_ops, &cur_skin_temp, 0644);

long tegra_get_cur_skin_temp()
{
	return cur_skin_temp;
}


#endif
