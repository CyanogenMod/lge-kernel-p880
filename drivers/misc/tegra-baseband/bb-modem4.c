/*
 * drivers/misc/tegra-baseband/bb-modem4.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#define DEBUG

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/suspend.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/tegra-bb-power.h>
#include <mach/usb_phy.h>
#include "bb-power.h"
#include "bb-modem4.h"
// ebs for autosuspend timeout
#include <linux/pm_runtime.h> 
#include <linux/workqueue.h>
#include <linux/module.h>

#define WAKE_TIMEOUT_MS 1000
//                                                              
#define AUTOSUSPEND_PATCH
//#define AUTOSUSPEND_CONTROL_TIMIER
#define MODEM_CRASH_ENABLE

static struct workqueue_struct *workqueue_susp;
static struct work_struct work_shortsusp, work_defaultsusp;

// wakelock ebs 
static struct wake_lock wakelock;
// wakelock ebs 
static DEFINE_MUTEX(modem4_usb_mutex);
static struct work_struct autopm_work;
static struct workqueue_struct *workqueue;

static struct tegra_bb_gpio_data modem4_gpios[] = {
	{ { GPIO_INVALID, GPIOF_OUT_INIT_LOW, "MDM4_RST" }, true },
	{ { GPIO_INVALID, GPIOF_OUT_INIT_LOW, "MDM4_ON" }, true },
	{ { GPIO_INVALID, GPIOF_OUT_INIT_LOW, "MDM4_USB_AWR" }, true },// ebs 
	{ { GPIO_INVALID, GPIOF_IN, "MDM4_USB_CWR" }, true }, //ebs 
	{ { GPIO_INVALID, GPIOF_IN, "MDM4_SPARE" }, true },
	{ { GPIO_INVALID, GPIOF_IN, "MDM4_WDI" }, true },
	{ { GPIO_INVALID, 0, NULL }, false },	/* End of table */
};

static struct sdata modem4_sdata;
static struct opsdata modem4_opdata;
static struct locks modem4_locks;
static struct regulator *hsic_reg;
static bool ehci_registered;
static int dlevel;

static void modem4_autopm_resume_work(struct work_struct *work)
{
        struct usb_interface *iface;

	if (dlevel & DLEVEL_MISC)
		pr_info("%s\n", __func__);
        if (!modem4_opdata.usbdev)
        {
                pr_debug("usbdev = NULL\n");
                return;
        }

	mutex_lock(&modem4_usb_mutex);
	if(modem4_opdata.usbdev) {
		usb_lock_device(modem4_opdata.usbdev);
		iface = usb_ifnum_to_if(modem4_opdata.usbdev, 0);
		if (iface) {
			if (usb_autopm_get_interface(iface) == 0)
				usb_autopm_put_interface(iface);
		} else 
			pr_info("usb interface == NULL\n", __func__);

		usb_unlock_device(modem4_opdata.usbdev);
	}
	mutex_unlock(&modem4_usb_mutex);
}

//                                                         
#ifdef AUTOSUSPEND_CONTROL_TIMIER
#define SHORT_AUTOSUSPEND_DELAY 100
#define DEFAULT_AUTOSUSPEND_DELAY 2000
static void autosuspend_power_shortsusp(struct work_struct *work)
{
 	pr_debug("%s {\n", __func__);
 	if (!modem4_usb_device || !&modem4_usb_device->dev) {
 		pr_err("%s modem4_usb_device is invalid\n", __func__);
 		return;
 	}
 	pm_runtime_set_autosuspend_delay(&modem4_usb_device->dev, SHORT_AUTOSUSPEND_DELAY);
 	pr_debug("} %s\n", __func__);
}
 
static void autosuspend_power_defaultsusp(struct work_struct *work)
{
 	pr_debug("%s {\n", __func__);
 	if (!modem4_usb_device || !&modem4_usb_device->dev) {
 		pr_err("%s modem4_usb_device is invalid\n", __func__);
 		return;
 	}
 	pm_runtime_set_autosuspend_delay(&modem4_usb_device->dev, DEFAULT_AUTOSUSPEND_DELAY);
 	pr_debug("} %s\n", __func__);
}
#endif

static int gpio_wait_timeout(int gpio, int value, int timeout_msec)
{
	int count;
	for (count = 0; count < timeout_msec; ++count) {
		if (gpio_get_value(gpio) == value) {
			if (dlevel & DLEVEL_MISC)
				pr_info("%s Got CWR count =%d\n", __func__, count);
			return 0;
		}
		mdelay(1);
	}
	return -1;
}

static void modem4_setdata(struct opsdata *data)
{
	unsigned long flags;

	spin_lock_irqsave(&modem4_locks.lock, flags);
	if (data) {
		if (data->powerstate != modem4_opdata.powerstate)
			modem4_opdata.powerstate = data->powerstate;
		if (data->usbdev != modem4_opdata.usbdev)
			modem4_opdata.usbdev = data->usbdev;
	}
	spin_unlock_irqrestore(&modem4_locks.lock, flags);
}

static void modem4_getdata(struct opsdata *data)
{
	unsigned long flags;

	spin_lock_irqsave(&modem4_locks.lock, flags);
	if (data) {
		data->powerstate = modem4_opdata.powerstate;
		data->usbdev = modem4_opdata.usbdev;
	}
	spin_unlock_irqrestore(&modem4_locks.lock, flags);
}

static int modem4_getpowerstate(void)
{
	struct opsdata data;

	modem4_getdata(&data);
	return data.powerstate;
}

static void modem4_setpowerstate(int status)
{
	struct opsdata data;

	modem4_getdata(&data);
	data.powerstate = status;
	modem4_setdata(&data);
}

static bool modem4_crashed(void)
{
	bool flag;

	flag = ((gpio_get_value(modem4_sdata.gpio_wdi)) ? false : true);
	return flag;
}

static bool modem4_get_cwr(void)
{
	return gpio_get_value(modem4_sdata.gpio_cwr);
}

static void modem4_set_awr(int value)
{
	if (dlevel & DLEVEL_MISC)
		pr_info("%s awr(%d)\n", __func__, value);
	gpio_set_value(modem4_sdata.gpio_awr, value);
}

static void modem4_gpio_stat(void)
{
	pr_info("%s: AWR:%d, CWR:%d, WDI:%d.\n", __func__,
		gpio_get_value(modem4_sdata.gpio_awr),
		gpio_get_value(modem4_sdata.gpio_cwr),
		gpio_get_value(modem4_sdata.gpio_wdi));
}

static int modem4_apup_handshake(bool checkresponse)
{
	int retval = 0;

	if (dlevel & DLEVEL_MISC)
		pr_info("%s\n", __func__);

	/* Signal AP ready - Drive USB_AWR high. */
	modem4_set_awr(1);

	if (checkresponse) {
		/* Wait for CP ack - by driving USB_CWR high. */
		if (gpio_wait_timeout(modem4_sdata.gpio_cwr, 1,
					WAKE_TIMEOUT_MS) != 0) {
			pr_err("%s: Error: Timeout waiting for modem ack.\n",
					__func__);
			retval = -1;
		} else if (dlevel & DLEVEL_MISC)
			pr_info("%s ack ok\n", __func__) ;
	}
	return retval;
}

static void modem4_apdown_handshake(void)
{
	/* Signal AP going down to modem - Drive USB_AWR low. */
	/* No need to wait for a CP response */
	if (dlevel & DLEVEL_MISC)
		pr_info("%s\n", __func__);
	modem4_set_awr(0);
}

static void modem4_l2_suspend(void)
{
	int modemstate = modem4_getpowerstate();
	
	/* Gets called for two cases :
		a) Port suspend.
		b) Bus suspend. */
	if (wake_lock_active(&wakelock)) {
		wake_unlock(&wakelock);
		pr_debug("%s wakelock released\n", __func__);
	}

	if (dlevel & DLEVEL_PM)
		pr_info("%s.\n", __func__);

	if (modemstate != BBSTATE_L0) {
		pr_err("%s: Error. Unexp modemstate %d. Should be %d.\n",
					 __func__, modemstate, BBSTATE_L0);
		return;
	}
	modem4_setpowerstate(BBSTATE_L2);
}

static void modem4_l2_resume(void)
{
	int modemstate = modem4_getpowerstate();
	bool resumeok = false;

	/* Gets called for two cases :
		a) L2 resume.
		b) bus resume phase of L3 resume. */
	if (!wake_lock_active(&wakelock)) {
		wake_lock(&wakelock);
		pr_debug("%s wakelock\n", __func__);
	}

	if (dlevel & DLEVEL_PM)
		pr_info("%s. cur st(%d)\n", __func__, modemstate);

	/* Gets called for two cases :
		a) L2 resume.
		b) bus resume phase of L3 resume. */
	switch (modemstate) {
	case BBSTATE_L2:
		resumeok = true;
		break;
	case BBSTATE_L3:
		if (modem4_apup_handshake(true) == 0)
			resumeok = true;
		else {
			modem4_gpio_stat();
			pr_err("%s: Error. Modem wakeup from L3 failed.\n",
								 __func__);
			/* TBD: Add code to unregister ehci. */
		}
		break;
	default:
		pr_err("%s: Error. Unexp modemstate %d. Should be %d or %d.\n",
				__func__, modemstate, BBSTATE_L2, BBSTATE_L3);
		break;
	}
	if (resumeok)
		modem4_setpowerstate(BBSTATE_L0);
}

static void modem4_phy_ready(void)
{
	int modemstate = modem4_getpowerstate();

	if (dlevel & DLEVEL_PM)
		pr_info("%s.\n", __func__);

	switch (modemstate) {
	case BBSTATE_UNKNOWN:
		/* Wait for CP to indicate ready - by driving USB_CWR high. */
		if (gpio_wait_timeout(modem4_sdata.gpio_cwr, 1, 10) != 0) {
			pr_info("%s: Timeout 4 modem ready. Maybe 1st enum ?\n",
							 __func__);
			/* For first enumeration don't drive AWR high */
			break;
		}

		/* Signal AP ready - Drive USB_AWR high. */
		pr_info("%s : Driving AWR high.\n", __func__);
		modem4_set_awr(1);
		break;
	default:
		pr_err("%s: Error. Unexp modemstate %d. Should be %d.\n",
				__func__, modemstate, BBSTATE_UNKNOWN);
		break;
	}
}

static void modem4_pre_phy_off(void)
{
	if (dlevel & DLEVEL_PM)
		pr_info("%s.\n", __func__);

	/* Signal AP going down */
	modem4_apdown_handshake();

	/* Gets called for two cases :
		a) L3 suspend.
		b) EHCI unregister. */
	if (modem4_getpowerstate() == BBSTATE_L2)
		modem4_setpowerstate(BBSTATE_L3);
/*	
	if (modem4_opdata.usbdev) {
		modem4_opdata.usbdev->reset_resume = 1;
	}
*/
}

static int modem4_l3_suspend(void)
{
	int pwrstate = modem4_getpowerstate();
	bool wakeup_detected = modem4_get_cwr();
	bool crashed = modem4_crashed();

	if (dlevel & DLEVEL_PM)
		pr_info("%s.\n", __func__);

	/* If modem state during system suspend is not L3 (crashed)
		or modem is initiating a wakeup, abort system suspend. */
	if ((pwrstate != BBSTATE_L3) || wakeup_detected || crashed) {
		if (pwrstate != BBSTATE_L3)
			pr_err("%s: Unexp modemstate %d. Should be %d.\n",
				 __func__, pwrstate, BBSTATE_L3);
		if (wakeup_detected)
			pr_info("%s : CWR high.\n", __func__);
		if (crashed)
			pr_info("%s : WDI low.\n", __func__);
		pr_info("%s: Aborting suspend.\n", __func__);
		return 1;
	}
	return 0;
}

static int modem4_l3_suspend_noirq(void)
{
	return 0;
}

static int modem4_l3_resume(void)
{
	long timeout = HZ/2;

	printk("##@%s\n", __func__);

	if (!wake_lock_active(&wakelock)) {
		wake_lock(&wakelock);
		pr_debug("%s wakelock\n", __func__);
	}

	if (modem4_crashed())
		timeout = HZ;
	if (dlevel & DLEVEL_PM)
		pr_info("%s: Taking timed wakelock for %ld\n",
					 __func__, timeout);
	/* Ensure no pending IPC data before next suspend. */
	wake_lock_timeout(&modem4_locks.wlock, timeout);
	return 0;
}

static int modem4_l3_resume_noirq(void)
{
	return 0;
}

static irqreturn_t modem4_wake_irq(int irq, void *dev_id)
{
	struct opsdata data;
	struct usb_interface *iface;

	if (dlevel & DLEVEL_MISC)
		pr_info("%s detected gpio_cwr high\n", __func__);

	/* if awr high, on CPL2L0/APL3L0 */
	if (gpio_get_value(modem4_gpios[2].data.gpio)) {
		if (!wake_lock_active(&wakelock)) {
			wake_lock(&wakelock);
			pr_debug("%s wakelock\n", __func__);
		}
	}
	modem4_getdata(&data);
	switch (data.powerstate) {
	case BBSTATE_L2:
	if (dlevel & DLEVEL_PM)
		pr_info("%s: Modem wakeup request from L2.\n", __func__);
		/* Resume usb host activity. */
		queue_work(workqueue, &autopm_work);
	break;
	default:
		break;
	}
	return IRQ_HANDLED;
}

static irqreturn_t modem4_notify_crash(int irq, void *dev_id)
{
	printk("[ebs] %s WDI Status = %d\n", __func__, gpio_get_value(modem4_gpios[5].data.gpio));
	printk("=====================\n");
	printk("[ebs] modem-crash | eungbo.shim@lge.com\n");
	printk("=====================\n");

	return IRQ_HANDLED;
}

static int modem4_power(int code)
{
	int retval = 0;
	printk("##@%s\n", __func__);

	switch (code) {
	case PWRSTATE_L2L3:
	retval = modem4_l3_suspend();
	break;
	case PWRSTATE_L2L3_NOIRQ:
	retval = modem4_l3_suspend_noirq();
	break;
	case PWRSTATE_L3L0_NOIRQ:
	retval = modem4_l3_resume_noirq();
	break;
	case PWRSTATE_L3L0:
	retval = modem4_l3_resume();
	break;
	default:
	break;
	}
	return retval;
}

static int modem4_ehci_customize(struct tegra_bb_pdata *pdata)
{
	struct platform_device *pdev;
	struct tegra_usb_platform_data *usb_pdata;
	struct tegra_usb_phy_platform_ops *pops;

	if (dlevel & DLEVEL_MISC)
		pr_info("%s\n", __func__);

	if (pdata && pdata->device) {
		pdev = pdata->device;
		usb_pdata = (struct tegra_usb_platform_data *)
			pdev->dev.platform_data;
		pops = (struct tegra_usb_phy_platform_ops *)
			usb_pdata->ops;

		/* Register PHY platform callbacks */
		pops->post_suspend = modem4_l2_suspend;
		pops->pre_resume = modem4_l2_resume;
		pops->port_power = modem4_phy_ready;
		pops->pre_phy_off = modem4_pre_phy_off;

		/* Override required settings */
		usb_pdata->u_data.host.power_off_on_suspend = 0;

	} else {
		pr_err("%s: Error. Invalid platform data.\n", __func__);
		return 0;
	}
	return 1;
}

static int modem4_load(struct device *dev, int value)
{
	struct tegra_bb_pdata *pdata;
	static struct platform_device *ehci_device;
	struct opsdata data;

	if (dlevel & DLEVEL_SYS_CB)
		pr_info("%s: Called with value : %d\n", __func__, value);

	if (value > 1 || (!ehci_registered && !value) ||
				(ehci_registered && value)) {
		/* Supported values are 0/1. */
		pr_err("%s:  Error. Invalid data. Exiting.\n", __func__);
		return -1;
	}

	pdata = (struct tegra_bb_pdata *) dev->platform_data;
	if (value) {
		/* Register ehci controller */
		ehci_device = pdata->ehci_register();
		if (ehci_device == NULL) {
			pr_err("%s: Error. ehci register failed.\n",
							 __func__);
			return -1;
		}
		ehci_registered = true;
	} else {
		/* Mark usb device invalid */
		mutex_lock(&modem4_usb_mutex);
		data.usbdev = NULL;
		mutex_unlock(&modem4_usb_mutex);
		data.powerstate = BBSTATE_UNKNOWN;
		modem4_setdata(&data);
		ehci_registered = false;

		/* Unregister ehci controller */
		if (ehci_device != NULL)
			pdata->ehci_unregister(ehci_device);

		/* Signal AP going down */
		modem4_apdown_handshake();
	}
	return 0;
}

static int modem4_dlevel(struct device *dev, int value)
{
	if (dlevel & DLEVEL_MISC)
		pr_info("%s\n", __func__);
	if (dlevel & DLEVEL_SYS_CB)
		pr_info("%s: Called with value : %d\n", __func__, value);

	dlevel = value;
	return 0;
}

static int modem4_usbnotify(struct usb_device *udev, bool registered)
{
	struct opsdata data;

	if (registered) {
		mutex_lock(&modem4_usb_mutex);
		data.usbdev = udev;
		mutex_unlock(&modem4_usb_mutex);
		data.powerstate = BBSTATE_L0;
		if (dlevel & DLEVEL_MISC)
			pr_info("%s: Registered.\n", __func__);
	} else {
		mutex_lock(&modem4_usb_mutex);
		data.usbdev = NULL;
		mutex_unlock(&modem4_usb_mutex);
		data.powerstate = BBSTATE_UNKNOWN;
		if (dlevel & DLEVEL_MISC)
			pr_info("%s: Unregistered.\n", __func__);
	}

	modem4_setdata(&data);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int modem4_pmnotify(unsigned long event)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
	if (dlevel & DLEVEL_PM)
		pr_info("%s: SUSPEND_PREPARE notifier.\n", __func__);
	return NOTIFY_OK;
	case PM_POST_SUSPEND:
	if (dlevel & DLEVEL_PM)
		pr_info("%s: POST_SUSPEND notifier.\n", __func__);
	return NOTIFY_OK;
	default:
	if (dlevel & DLEVEL_PM)
		pr_info("%s: Unknown notifier.\n", __func__);
	return NOTIFY_OK;
	}
}
#endif

static struct tegra_bb_gpio_irqdata modem4_gpioirqs[] = {
	{ GPIO_INVALID, "tegra_bb_wake", modem4_wake_irq,
		IRQF_TRIGGER_RISING , true, NULL },
// ebs
#ifdef MODEM_CRASH_ENABLE
	{ GPIO_INVALID, "tegra_cp_crash", modem4_notify_crash,
		IRQF_TRIGGER_FALLING | IRQF_SHARED, false, NULL },
#endif		
// ebs 
		
	{ GPIO_INVALID, NULL, NULL, 0, NULL },	/* End of table */
};

static struct tegra_bb_power_gdata modem4_gdata = {
	.gpio = modem4_gpios,
	.gpioirq = modem4_gpioirqs,
};

static struct tegra_bb_power_mdata modem4_mdata = {
	.vid = 0x045B,
	.pid = 0x020F,
	.wake_capable = true,
#ifdef AUTOSUSPEND_PATCH
	.autosuspend_ready = true,
#else
	.autosuspend_ready = false,
#endif 
};

static struct tegra_bb_power_data modem4_data = {
	.gpio_data = &modem4_gdata,
	.modem_data = &modem4_mdata,
};


static void *modem4_init(void *pdata)
{
	struct tegra_bb_pdata *platdata = (struct tegra_bb_pdata *) pdata;
	union tegra_bb_gpio_id *id = platdata->id;
	struct opsdata data;

	/* Fill the gpio ids allocated by hardware */
	modem4_gpios[0].data.gpio = id->modem4.mdm4_rst;
	modem4_gpios[1].data.gpio = id->modem4.mdm4_on;
	modem4_gpios[2].data.gpio = id->modem4.usb_awr;
	modem4_gpios[3].data.gpio = id->modem4.usb_cwr;
	modem4_gpios[4].data.gpio = id->modem4.mdm4_spare;
	modem4_gpios[5].data.gpio = id->modem4.mdm4_wdi;
	modem4_gpioirqs[0].id = id->modem4.usb_cwr;
	modem4_gpioirqs[0].cookie = (void *)platdata;

#ifdef MODEM_CRASH_ENABLE //ebs 
	modem4_gpioirqs[1].id = id->modem4.mdm4_wdi;
	modem4_gpioirqs[1].cookie = (void *)platdata;
	//state_l2_ebs = 0; 
//	workqueue_susp = alloc_workqueue("autosuspend_power_autosusp",  WQ_UNBOUND | WQ_HIGHPRI | WQ_NON_REENTRANT, 1);

 //   INIT_WORK(&work_shortsusp, autosuspend_power_shortsusp);
  //  INIT_WORK(&work_defaultsusp, autosuspend_power_defaultsusp);
#endif
	/* init wake lock */
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "wakelock_bb_driver");

	if (!platdata->ehci_register || !platdata->ehci_unregister) {
		pr_err("%s - Error: ehci reg/unreg functions missing.\n"
				, __func__);
		return 0;
	}

		modem4_sdata.gpio_awr = modem4_gpios[2].data.gpio;
	modem4_sdata.gpio_cwr = modem4_gpios[3].data.gpio;
	modem4_sdata.gpio_wdi = modem4_gpios[5].data.gpio;
	if (modem4_sdata.gpio_awr == GPIO_INVALID ||
		modem4_sdata.gpio_cwr == GPIO_INVALID ||
		modem4_sdata.gpio_wdi == GPIO_INVALID) {
		pr_err("%s: Error. Invalid gpio data.\n", __func__);
		return 0;
	}

	/* Customize PHY setup/callbacks */
	if (!modem4_ehci_customize(platdata))
		return 0;

	/* init work queue */
        workqueue = create_singlethread_workqueue("modem4_workqueue");
        if (!workqueue) {
                pr_err("%s cannot create workqueue\n", __func__);
                return 0;
        }

	/* Board specific regulator init */
	if (platdata->regulator) {
		hsic_reg = regulator_get(NULL,
					(const char *)platdata->regulator);
		if (IS_ERR_OR_NULL(hsic_reg)) {
			pr_err("%s: Error. regulator_get failed.\n",
							 __func__);
			return 0;
		}

		if (regulator_enable(hsic_reg) < 0) {
			pr_err("%s: Error. regulator_enable failed.\n",
							 __func__);
			return 0;
		}
	}

	spin_lock_init(&modem4_locks.lock);
	wake_lock_init(&modem4_locks.wlock, WAKE_LOCK_SUSPEND,
						"tegra-bb-lock");
	INIT_WORK(&autopm_work, modem4_autopm_resume_work);

	ehci_registered = false;
	data.usbdev = NULL;
	data.powerstate = BBSTATE_UNKNOWN;
	modem4_setdata(&data);
	dlevel = DLEVEL_INIT;
#ifdef DEBUG
	dlevel = DLEVEL_MAX;
#endif

	return (void *) &modem4_data;
}

static void *modem4_deinit(void)
{
        pr_info("%s called.\n", __func__);
	/* destroy work */
	destroy_workqueue(workqueue);
	/* destroy wake lock */
	wake_lock_destroy(&modem4_locks.wlock);

	/* destory wakelock */
	wake_lock_destroy(&wakelock);

	return (void *) &modem4_gdata;
}

static struct tegra_bb_callback modem4_callbacks = {
	.init = modem4_init,
	.deinit = modem4_deinit,
	.load = modem4_load,
	.dlevel = modem4_dlevel,
	.usbnotify = modem4_usbnotify,
#ifdef CONFIG_PM
	.power = modem4_power,
#endif
#ifdef CONFIG_PM_SLEEP
	.pmnotify = modem4_pmnotify,
#endif
};

void *modem4_get_cblist(void)
{
	return (void *) &modem4_callbacks;
}
