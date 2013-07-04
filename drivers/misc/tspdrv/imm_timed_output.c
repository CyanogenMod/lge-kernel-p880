#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>

#include "../../staging/android/timed_output.h"
#include "imm_timed_output.h"

/* strength modification */
#define DEFAULT_STRENGTH  119  /* ~90% */

static unsigned short int pwm_val = 100;
static int8_t strength = DEFAULT_STRENGTH;

static void tspdrv_vib_enable(struct timed_output_dev *dev, int value);

static ssize_t pwm_val_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int count;

	count = sprintf(buf, "%hu\n", pwm_val);
	pr_debug("[VIB] pwm_val: %hu\n", pwm_val);

	return count;
}

static ssize_t pwm_val_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	if (kstrtoul(buf, 0, (unsigned long int*)&pwm_val))
		pr_err("[VIB] %s: error on storing pwm_val\n", __func__);

	pr_info("[VIB] %s: pwm_val=%hu\n", __func__, pwm_val);

	/* make sure new pwm duty is in range */
	if (pwm_val > 100)
		pwm_val = 100;
	else if (pwm_val < 0)
		pwm_val = 0;

	strength = DEFAULT_STRENGTH * pwm_val / 100;
	if (strength == 0)
		tspdrv_vib_enable(NULL, 0);

	return size;
}

static DEVICE_ATTR(pwm_val, S_IRUGO | S_IWUSR, pwm_val_show, pwm_val_store);

static int create_vibrator_sysfs(void)
{
	int ret;
	struct kobject *vibrator_kobj;
	vibrator_kobj = kobject_create_and_add("vibrator", NULL);
	if (unlikely(!vibrator_kobj))
		return -ENOMEM;

	ret = sysfs_create_file(vibrator_kobj, &dev_attr_pwm_val.attr);
	if (unlikely(ret < 0)) {
		pr_err("[VIB] sysfs_create_file failed: %d\n", ret);
		return ret;
	}

	return 0;
}

/* timed_output */
#define MAX_TIMEOUT_MS 15000

static struct hrtimer vib_timer;
static atomic_t vib_state = ATOMIC_INIT(0);
static struct work_struct vibrator_work;
extern int32_t ImmVibeSPI_ForceOut_AmpEnable(u_int8_t);
extern int32_t ImmVibeSPI_ForceOut_AmpDisable(u_int8_t);
extern int32_t ImmVibeSPI_ForceOut_SetSamples(u_int8_t, u_int16_t, u_int16_t, int8_t*);

static void ImmVibeSPI_Control(struct work_struct *work)
{
	if (atomic_read(&vib_state)) {
		ImmVibeSPI_ForceOut_AmpEnable(0);
		ImmVibeSPI_ForceOut_SetSamples(0, 8, 1, &strength);
	}
	else {
		ImmVibeSPI_ForceOut_AmpDisable(0);
	}
}

static void tspdrv_vib_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vib_timer);

	if (value == 0) {
		atomic_set(&vib_state, 0); // Turn Off the vibrator
		schedule_work(&vibrator_work);
 	}
	else {
		if (strength == 0)
			return;

		value = (value > MAX_TIMEOUT_MS ? MAX_TIMEOUT_MS : value);
		/* [LGE_BSP_START][yunmo.yang@lge.com] Unlimit Vibrator Bug fix */
		if (value < 10)
			value = 10;
		/* [LGE_BSP_END][yunmo.yang@lge.com] Unlimit Vibrator Bug fix */

		atomic_set(&vib_state, 1);
		schedule_work(&vibrator_work);

		hrtimer_start(&vib_timer,
			ktime_set(value/ 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}
}

static int tspdrv_vib_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib_timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_sec / 1000;
	}	

	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	atomic_set(&vib_state, 0);
	schedule_work(&vibrator_work);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev timed_dev = {
	.name = "vibrator",
	.get_time = tspdrv_vib_get_time,
	.enable	= tspdrv_vib_enable,
}; 

void __init ImmVibe_timed_output(void)
{
	INIT_WORK(&vibrator_work, ImmVibeSPI_Control);
	hrtimer_init(&vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib_timer.function = vibrator_timer_func;
	atomic_set(&vib_state, 0); //Default is Vibrator OFF
	timed_output_dev_register(&timed_dev);

	create_vibrator_sysfs();
}

MODULE_DESCRIPTION("timed output immersion vibrator device");
MODULE_LICENSE("GPL");
