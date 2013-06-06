/*
 * arch/arm/mach-tegra/baseband-xmm-power.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

//#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/pm_runtime.h> //20120112 - Nvidia Bug [924425] - L2 Auto Suspend fixed #1
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <mach/usb_phy.h>
#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "baseband-xmm-power.h"

MODULE_LICENSE("GPL");

unsigned long modem_ver = XMM_MODEM_VER_1145; //XMM_MODEM_VER_1121;
EXPORT_SYMBOL(modem_ver);

unsigned long modem_flash=1;		//                               
EXPORT_SYMBOL(modem_flash);

unsigned long modem_pm = 1;
EXPORT_SYMBOL(modem_pm);

unsigned long enum_delay_ms = 1000; /* ignored if !modem_flash */

module_param(modem_ver, ulong, 0644);
MODULE_PARM_DESC(modem_ver,
	"baseband xmm power - modem software version");
module_param(modem_flash, ulong, 0644);
MODULE_PARM_DESC(modem_flash,
	"baseband xmm power - modem flash (1 = flash, 0 = flashless)");
module_param(modem_pm, ulong, 0644);
MODULE_PARM_DESC(modem_pm,
	"baseband xmm power - modem power management (1 = pm, 0 = no pm)");
module_param(enum_delay_ms, ulong, 0644);
MODULE_PARM_DESC(enum_delay_ms,
	"baseband xmm power - delay in ms between modem on and enumeration");

bool enum_success = false;

static bool enable_short_autosuspend = true;
const char *pwrstate_cmt[] = {
	"UNINT",	/* BBXMM_PS_UNINIT */
	"INIT",		/* BBXMM_PS_INIT */
	"L0",		/* BBXMM_PS_L0 */
	"L0TOL2",	/* BBXMM_PS_L0TOL2 */
	"L2",		/* BBXMM_PS_L2 */
	"L2TOL0",	/* BBXMM_PS_L2TOL0 */
	"L2TOL3",	/* BBXMM_PS_L2TOL3 */
	"L3",		/* BBXMM_PS_L3 */
	"L3TOL0",	/* BBXMM_PS_L3TOL0 */
};

static struct usb_device_id xmm_pm_ids[] = {
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID),
	.driver_info = 0 },
	{}
};

static struct gpio tegra_baseband_gpios[] = {
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_RSTn" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_ON"   },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_WAKE" },
	{ -1, GPIOF_IN,            "IPC_AP_WAKE" },
	{ -1, GPIOF_OUT_INIT_HIGH, "IPC_HSIC_ACTIVE" },
	{ -1, GPIOF_IN,            "IPC_HSIC_SUS_REQ" },
};

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

static bool short_autosuspend = false;

enum baseband_xmm_powerstate_t baseband_xmm_powerstate;
static struct workqueue_struct *workqueue;
static struct work_struct init1_work;
static struct work_struct init2_work;
static struct work_struct L2_resume_work;
static struct baseband_power_platform_data *baseband_power_driver_data;
static bool register_hsic_device;
static struct wake_lock wakelock;
static struct usb_device *usbdev;
static bool CP_initiated_L2toL0;
static bool modem_power_on;
static int power_onoff;
//To_Ril-recovery Nvidia_Patch_20111226 [Start]
static unsigned long XYZ = 4500 * 1000000 + 800 * 1000 + 500;
static int enum_repeat = ENUM_REPEAT_TRY_CNT;
//To_Ril-recovery Nvidia_Patch_20111226 [End]

/* 20120112 - Nvidia Bug [924425] - L2 Auto Suspend fixed #1 [Start] */
static int reenable_autosuspend;
/* 20120112 - Nvidia Bug [924425] - L2 Auto Suspend fixed #1 [End] */
static bool wakeup_pending;
static bool modem_sleep_flag;
static spinlock_t xmm_lock;
static DEFINE_MUTEX(xmm_onoff_mutex);

static struct workqueue_struct *workqueue_susp;
static struct work_struct work_shortsusp, work_defaultsusp;
static struct baseband_xmm_power_work_t *baseband_xmm_power_work_usb;

//Move place To_Ril-recovery Nvidia_Patch_20111226
static struct baseband_xmm_power_work_t *baseband_xmm_power_work;
static bool system_suspending;

static bool l2_resume_work_done = false;

static void baseband_xmm_power_L2_resume(void);
static int baseband_xmm_power_driver_handle_resume(struct baseband_power_platform_data *data);
#define CP_RESET_SEQUENCE

static inline void baseband_xmm_power_msleep(u32 t)
{
	/*
	If timer value is between ( 10us - 20ms),
	usleep_range() is recommended.
	Please read Documentation/timers/timers-howto.txt.
	*/
	usleep_range(t*1000, t*1000 + 500);
}

static int baseband_modem_power_on(struct baseband_power_platform_data *data)
{
	pr_debug("!! %s !! \n", __func__ ) ; //To_Ril-recovery Nvidia_Patch_20111226
	/* set IPC_HSIC_ACTIVE active */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 1);
	//pr_debug("ipc_hsic_active -> 1 \n");
	pr_debug("GPIO [W]: Host_active -> 1 \n"); 

	/* reset / power on sequence */
#ifndef CP_RESET_SEQUENCE
	mdelay(40);
	gpio_set_value(data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(data->modem.xmm.bb_on, 1);
	udelay(70);
	gpio_set_value(data->modem.xmm.bb_on, 0);
#else
	gpio_set_value(data->modem.xmm.bb_on, 0);
	mdelay(50);
	gpio_set_value(data->modem.xmm.bb_rst, 0);
	mdelay(200);
	gpio_set_value(data->modem.xmm.bb_rst, 1);	
	mdelay(50);

	gpio_set_value(data->modem.xmm.bb_on, 1);
	udelay(60);
	gpio_set_value(data->modem.xmm.bb_on, 0);
//	mdelay(100);
//	gpio_set_value(data->modem.xmm.bb_on, 1);
#endif
	return 0;
}

extern int forced_abort_hubevent;
static int baseband_xmm_power_on(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	int ret;
				
	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

     if (baseband_xmm_powerstate != BBXMM_PS_UNINIT)
            return -EINVAL;

	/* reset the state machine */
	baseband_xmm_powerstate = BBXMM_PS_INIT;
	modem_sleep_flag = false;

	if (modem_ver < XMM_MODEM_VER_1130)
		ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
	else
		ipc_ap_wake_state = IPC_AP_WAKE_INIT2;

	pr_debug("%s - %d\n", __func__, __LINE__);

	/* register usb host controller */
	if (!modem_flash) {
		pr_debug("%s - %d\n", __func__, __LINE__);
		/* register usb host controller only once */
		if (register_hsic_device) {
			pr_debug("%s: register usb host controller\n",__func__);
			modem_power_on = true;
			if (data->hsic_register)
				data->modem.xmm.hsic_device = data->hsic_register();
			else
				pr_err("%s: hsic_register is missing\n",__func__);
			register_hsic_device = false;
		} else {
			/* register usb host controller */
			if (data->hsic_register)
				data->modem.xmm.hsic_device = data->hsic_register();
			/* turn on modem */
			pr_debug("%s call baseband_modem_power_on\n", __func__);
			baseband_modem_power_on(data);
		}
	} else {                
		if (modem_ver >= XMM_MODEM_VER_1145) {
			pr_debug("%s: ver > 1145: IPC_AP_WAKE_IRQ_READY\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
			baseband_xmm_powerstate = BBXMM_PS_UNINIT;
			CP_initiated_L2toL0 = false;
			enum_repeat = ENUM_REPEAT_TRY_CNT;
			printk("##@%s forced_abort_hubevent <- 0\n", __func__);
			forced_abort_hubevent = 0;
	
			baseband_xmm_power_work->state = BBXMM_WORK_INIT;
			queue_work(workqueue,(struct work_struct *) baseband_xmm_power_work);
		} else
			pr_err("%s: Error, no support cp ver\n", __func__);
	}
    ret = enable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
    if (ret < 0)
           pr_err("%s: enable_irq_wake error\n", __func__);
	pr_debug("%s }\n", __func__);

	return 0;
}

static int baseband_xmm_power_off(struct platform_device *device)
{
	struct baseband_power_platform_data *data;
	int ret;
    unsigned long flags;

    enum_success = false;
    
	pr_debug("%s {\n", __func__);

	if (baseband_xmm_powerstate == BBXMM_PS_UNINIT)
	    return -EINVAL;

	/* check for device / platform data */
	if (!device) {
		pr_err("%s: !device\n", __func__);
		return -EINVAL;
	}
	data = (struct baseband_power_platform_data *)device->dev.platform_data;
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

    ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
    ret = disable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
    if (ret < 0)
             pr_err("%s: disable_irq_wake error\n", __func__);

	if (modem_flash && modem_ver >= XMM_MODEM_VER_1145) {
		/* unregister usb host controller */
		pr_debug("%s: hsic device: %p\n", __func__, data->modem.xmm.hsic_device);
                flush_workqueue(workqueue);
        
		if (data->hsic_unregister && !register_hsic_device)
		{
			data->hsic_unregister(data->modem.xmm.hsic_device);
			register_hsic_device = true;
		}
		else
			pr_err("%s: hsic_unregister is missing\n", __func__);

		/* set IPC_HSIC_ACTIVE low */
		gpio_set_value(baseband_power_driver_data->modem.xmm.ipc_hsic_active, 0);
		//pr_debug("ipc_hsic_active -> 0 \n");
		pr_debug("GPIO [W]: Host_active -> 0 \n"); 

		/* drive bb_rst low */
		gpio_set_value(data->modem.xmm.bb_rst, 0);
		pr_debug("GPIO [W]: CP_rst -> 0 \n"); 
	} else {
		/* unregister usb host controller */
		pr_debug("%s: hsic device: %p\n", __func__, data->modem.xmm.hsic_device);
		if (data->hsic_unregister)
			data->hsic_unregister(data->modem.xmm.hsic_device);
		else
			pr_err("%s: hsic_unregister is missing\n", __func__);

		/* set IPC_HSIC_ACTIVE low */
		gpio_set_value(baseband_power_driver_data->modem.xmm.ipc_hsic_active, 0);
		pr_debug("GPIO [W]: Host_active -> 0 \n"); 

		/* wait 20 ms */
		mdelay(20);

		/* drive bb_rst low */
		gpio_set_value(data->modem.xmm.bb_rst, 0);
		pr_debug("GPIO [W]: CP_rst -> 0 \n"); 		
		mdelay(1);
	}

	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	modem_sleep_flag = false;
	CP_initiated_L2toL0 = false;
    spin_lock_irqsave(&xmm_lock, flags);
    wakeup_pending = false;
	system_suspending = false;
   	spin_unlock_irqrestore(&xmm_lock, flags);		

	pr_debug("%s }\n", __func__);
	return 0;
}

static ssize_t baseband_xmm_onoff(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int pwr;
	int size;
	struct platform_device *device = to_platform_device(dev);

	mutex_lock(&xmm_onoff_mutex);

	pr_debug("%s\n", __func__);

	/* check input */
	if (buf == NULL) {
		pr_err("%s: buf NULL\n", __func__);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}
	pr_debug("%s: count=%d\n", __func__, count);

	/* parse input */
	size = sscanf(buf, "%d", &pwr);
	if (size != 1) {
		pr_err("%s: size=%d -EINVAL\n", __func__, size);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}

	if (power_onoff == pwr) {
		pr_err("%s: Ignored, due to same CP power state(%d)\n", __func__, power_onoff);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}
	power_onoff = pwr;
	pr_debug("%s power_onoff=%d\n", __func__, power_onoff);
	

	if (power_onoff == 0)
		baseband_xmm_power_off(device);
	else if (power_onoff == 1)
		baseband_xmm_power_on(device);

	mutex_unlock(&xmm_onoff_mutex);
	return count;
}

//To_Ril-recovery Nvidia_Patch_20111226 [Start]
static ssize_t baseband_xmm_onoff_show(struct device *dev,
            struct device_attribute *attr,
            char *buf)
{
    int onoff = power_onoff;
    pr_debug("%s, enum(%d), onoff(%d)\n", __func__, enum_success, power_onoff );
    if(enum_success)
        onoff = 2;
              
    return sprintf(buf, "%d", onoff);
}

static DEVICE_ATTR(xmm_onoff, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP , baseband_xmm_onoff_show, baseband_xmm_onoff);

void baseband_xmm_power_switch(bool power_on)
{
	mm_segment_t oldfs;
	struct file *filp;

	pr_debug("%s {\n", __func__);
	pr_debug("power_on(%d)\n", power_on);
		
	/* check if enumeration succeeded */

	if (power_onoff == power_on)
		pr_err("%s: Ignored, due to same CP power state(%d)\n", __func__, power_onoff);
	else {
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		filp = filp_open(XMM_ONOFF_PATH, O_RDWR, 0);
		if (IS_ERR(filp) || (filp == NULL)) {
			pr_err("open xmm_onoff failed %ld\n",PTR_ERR(filp));
		} else {
			if (power_on)
				filp->f_op->write(filp, "1", 1, &filp->f_pos);
			else
				filp->f_op->write(filp, "0", 1, &filp->f_pos);
				
			filp_close(filp, NULL);
		}
		set_fs(oldfs);
	}

	pr_debug("%s }\n", __func__);
}
EXPORT_SYMBOL_GPL(baseband_xmm_power_switch);
//To_Ril-recovery Nvidia_Patch_20111226 [End]

void baseband_xmm_set_power_status(unsigned int status)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value = 0;
    unsigned long flags;

	pr_debug("%s n(%d),o(%d)\n", __func__, status, baseband_xmm_powerstate );

	if (baseband_xmm_powerstate == status)
		return;

	switch (status) {
	case BBXMM_PS_L0:
		if (modem_sleep_flag) {
			pr_debug("%s Resume from L3 without calling resume" "function\n",  __func__);
				baseband_xmm_power_driver_handle_resume(data);
		}	
		pr_debug("PM_ST : L0\n");
		baseband_xmm_powerstate = status;
		if (!wake_lock_active(&wakelock))
			wake_lock(&wakelock);
		value = gpio_get_value(data->modem.xmm.ipc_hsic_active);
		//pr_debug("GPIO [R]: before L0 Host_active = %d \n", value); 
		if (!value) {
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);
			pr_debug("GPIO [W]: L0 Host_active -> 1 \n"); 
		}
		if (modem_power_on) {
			modem_power_on = false;
			baseband_modem_power_on(data);
		}
		pr_debug("gpio host active high->\n");
		break;
		
	case BBXMM_PS_L2:
		pr_debug("PM_ST : L2\n");
		baseband_xmm_powerstate = status;
		spin_lock_irqsave(&xmm_lock, flags);
		if (wakeup_pending) {
			spin_unlock_irqrestore(&xmm_lock, flags);
			baseband_xmm_power_L2_resume();
		 } else {
			spin_unlock_irqrestore(&xmm_lock, flags);
			if (wake_lock_active(&wakelock))
				wake_unlock(&wakelock);
		modem_sleep_flag = true;
		}
		if (short_autosuspend && enable_short_autosuspend && &usbdev->dev) {
			pr_debug("autosuspend delay %d ms, disable short_autosuspend\n",DEFAULT_AUTOSUSPEND_DELAY);
			queue_work(workqueue_susp, &work_defaultsusp);
			short_autosuspend = false;
		}
		break;
		
	case BBXMM_PS_L3:
		if (baseband_xmm_powerstate == BBXMM_PS_L2TOL0) {
				if (!data->modem.xmm.ipc_ap_wake) {
						spin_lock_irqsave(&xmm_lock, flags);
						wakeup_pending = true;
						spin_unlock_irqrestore(&xmm_lock, flags);
					pr_debug("%s: L2 race condition-CP wakeup" " pending\n", __func__);
				}
		}	
		pr_debug("PM_ST : L3\n");
		baseband_xmm_powerstate = status;
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		if (wake_lock_active(&wakelock)) {
			pr_debug("%s: releasing wakelock before L3\n", __func__);
			wake_unlock(&wakelock);
		}
		gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
		pr_debug("GPIO [W]: Host_active -> 0 \n"); 
		break;
	case BBXMM_PS_L2TOL0:
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		/* do this only from L2 state */
		if (baseband_xmm_powerstate == BBXMM_PS_L2) {
			baseband_xmm_powerstate = status;
			pr_debug("BB XMM POWER STATE = %d\n", status);
			baseband_xmm_power_L2_resume();
		}
		baseband_xmm_powerstate = status;
		break;
	default:
		baseband_xmm_powerstate = status;
		break;
	}
	pr_debug("BB XMM POWER STATE = %s(%d)\n", pwrstate_cmt[status], status);
}
EXPORT_SYMBOL_GPL(baseband_xmm_set_power_status);

irqreturn_t baseband_xmm_power_ipc_ap_wake_irq(int irq, void *dev_id)
{
	int value;
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	
	value = gpio_get_value(baseband_power_driver_data->modem.xmm.ipc_ap_wake);
	pr_debug("%s g(%d), wake_st(%d)\n", __func__, value, ipc_ap_wake_state);

	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1" " - got falling edge\n", __func__);
			/* go to IPC_AP_WAKE_INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
			/* queue work */
			queue_work(workqueue, &init1_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1" " - wait for falling edge\n",	__func__);
		}
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_INIT1) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT2" " - wait for rising edge\n",__func__);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT2" " - got rising edge\n",__func__);
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			/* queue work */
			queue_work(workqueue, &init2_work);
		}
	} else {
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			/* First check it a CP ack or CP wake  */
			value = gpio_get_value(data->modem.xmm.ipc_bb_wake);
			if (value) {
				pr_debug("cp ack for bb_wake\n");
				ipc_ap_wake_state = IPC_AP_WAKE_L;
				return IRQ_HANDLED;
			}
			spin_lock(&xmm_lock);
			wakeup_pending = true;
			if (system_suspending) {
				spin_unlock(&xmm_lock);
				pr_debug("Set wakeup_pending = 1 in system_" " suspending!!!\n");
			} else {
				if (baseband_xmm_powerstate == BBXMM_PS_L3) {
					spin_unlock(&xmm_lock);
					pr_debug("PM_ST : CP L3 -> L0\n");
				} else if (baseband_xmm_powerstate == BBXMM_PS_L2) {
					CP_initiated_L2toL0 = true;
					spin_unlock(&xmm_lock);
					baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
				} else {
					CP_initiated_L2toL0 = true;
					spin_unlock(&xmm_lock);
				}
			}
			/* save gpio state */
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			value = gpio_get_value(data->modem.xmm.ipc_hsic_active);
			//pr_debug("GPIO [R]: Host_active = %d \n",value); 
			if (!value) {
				pr_debug("host active low: ignore request\n");
				ipc_ap_wake_state = IPC_AP_WAKE_H;
				return IRQ_HANDLED;
			}
			value = gpio_get_value(data->modem.xmm.ipc_bb_wake);
			//pr_debug("GPIO [R]: Slave_wakeup = %d \n", value); 
			if (value) {
				/* Clear the slave wakeup request */
				gpio_set_value(data->modem.xmm.ipc_bb_wake, 0);
				pr_debug("GPIO [W]: Slave_wake -> 0 \n"); 
			}
			if (reenable_autosuspend && usbdev) {
			      reenable_autosuspend = false;
				struct usb_interface *intf;
				intf = usb_ifnum_to_if(usbdev, 0);
				if (usb_autopm_get_interface_async(intf) >= 0) {
					pr_debug("get_interface_async succeeded"	" - call put_interface\n");
					usb_autopm_put_interface_async(intf);
				} else {
					pr_debug("get_interface_async failed" " - do not call put_interface\n");
				}			      
			}
			modem_sleep_flag = false;
			baseband_xmm_set_power_status(BBXMM_PS_L0);
			if (short_autosuspend && enable_short_autosuspend &&
						baseband_xmm_powerstate == BBXMM_PS_L0 &&
						&usbdev->dev) 
			{
				pr_debug("set autosuspend delay %d ms\n",SHORT_AUTOSUSPEND_DELAY);
				queue_work(workqueue_susp, &work_shortsusp);				
			} 
			/* save gpio state */
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(baseband_xmm_power_ipc_ap_wake_irq);

static void baseband_xmm_power_init1_work(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	if (modem_flash && modem_ver >= XMM_MODEM_VER_1145) {		
		if (register_hsic_device && baseband_power_driver_data->hsic_register) {
			pr_debug("%s: register usb host controller\n", __func__);
			baseband_power_driver_data->modem.xmm.hsic_device = baseband_power_driver_data->hsic_register();
			register_hsic_device = false;

			baseband_xmm_power_work->state =  BBXMM_WORK_INIT_FLASH_PM_VER_GE_1145_RECOVERY;
			queue_work(workqueue, (struct work_struct *) baseband_xmm_power_work);						
		} else {
			pr_err("%s: Error? hsic_register is missing \n", __func__);
		}
	} else {
		/* check if IPC_HSIC_ACTIVE high */
		value = gpio_get_value(baseband_power_driver_data->modem.xmm.ipc_hsic_active);
		//pr_debug("GPIO [R]: Host_active = %d \n",value); 
		if (value != 1) {
			pr_err("%s - expected IPC_HSIC_ACTIVE high!\n", __func__);
			return;
		}

		/* wait 100 ms */
		mdelay(100);

		/* set IPC_HSIC_ACTIVE low */
		gpio_set_value(baseband_power_driver_data->modem.xmm.ipc_hsic_active, 0);
		pr_debug("GPIO [W]: Host_active -> 0 \n"); 
		/* wait 10 ms */
		mdelay(10);

		/* set IPC_HSIC_ACTIVE high */
		gpio_set_value(baseband_power_driver_data->modem.xmm.ipc_hsic_active, 1);
		pr_debug("GPIO [W]: Host_active -> 1 \n"); 
		/* wait 20 ms */
		mdelay(20);
	}
      
	pr_debug("%s }\n", __func__);
}

static void baseband_xmm_power_init2_work(struct work_struct *work)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;

	pr_debug("%s\n", __func__);

	/* check input */
	if (!data)
		return;

	if (modem_flash && modem_ver >= XMM_MODEM_VER_1145) {
		pr_debug("%s modem enumeration done\n", __func__);
		return;
	}

	/* register usb host controller only once */
	if (register_hsic_device) {
		if (data->hsic_register) {
			data->modem.xmm.hsic_device = data->hsic_register();			
		}
		else
			pr_err("%s: hsic_register is missing\n", __func__);		
		register_hsic_device = false;
	}
}


/* Do the work for AP/CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume(void)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value;
	int delay = 1000; /* maxmum delay in msec */
	unsigned long flags;

	pr_debug("%s\n", __func__);

	if (!baseband_power_driver_data)
		return;

	/* claim the wakelock here to avoid any system suspend */
	if (!wake_lock_active(&wakelock))
		wake_lock(&wakelock);
	modem_sleep_flag = false;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	if (CP_initiated_L2toL0)  {		
		pr_debug("PM_ST : CP L2->L0\n");
	    spin_lock_irqsave(&xmm_lock, flags);
		CP_initiated_L2toL0 = false;
	    spin_unlock_irqrestore(&xmm_lock, flags);
		queue_work(workqueue, &L2_resume_work);
	} else {
		/* set the slave wakeup request */
		pr_debug("PM_ST : AP L2->L0\n");
		value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
		//pr_debug("GPIO [R]: Host_wakeup = %d \n",value); 
		if (value) {
			gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);
			pr_debug("GPIO [W]: Slave_wake -> 1 \n"); 
			pr_debug("waiting for host wakeup from CP...\n");
			do {
				mdelay(1);
				value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
				delay--;
			} while ((value) && (delay));
			if (delay)
			{
				//pr_debug("GPIO [R]: Host_wakeup = %d \n",value); 
			}
			else
			{
				pr_debug("PM_ST :!!AP L2->L0 Failed and CP is not response !!\n");
			}
		} else {
			pr_debug("CP already ready\n");		
        }
	}
}

/* Do the work for CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume_work(struct work_struct *work)
{
	struct usb_interface *intf;

	pr_debug("%s {\n", __func__);

	l2_resume_work_done = false;

	if (!usbdev)
	{
		pr_debug("usbdev = %d\n",usbdev);
		return;
	}

	usb_lock_device(usbdev);
	intf = usb_ifnum_to_if(usbdev, 0);

	if(intf == NULL)
	{
		pr_debug("usb_ifnum_to_if's return vaule is NULL !! \n");
	}
	
	if (usb_autopm_get_interface(intf) == 0)
		usb_autopm_put_interface(intf);
	usb_unlock_device(usbdev);

	l2_resume_work_done = true;

	pr_debug("} %s\n", __func__);
}

static void baseband_xmm_power_shortsusp(struct work_struct *work)
{
    pr_debug("%s {\n", __func__);
    if (!usbdev || !&usbdev->dev) {
        pr_err("%s usbdev is invalid\n", __func__);
        return;
    }
    pm_runtime_set_autosuspend_delay(&usbdev->dev, SHORT_AUTOSUSPEND_DELAY);
    pr_debug("} %s\n", __func__);
}

static void baseband_xmm_power_defaultsusp(struct work_struct *work)
{
    pr_debug("%s {\n", __func__);
    if (!usbdev || !&usbdev->dev) {
        pr_err("%s usbdev is invalid\n", __func__);
        return;
    }
    pm_runtime_set_autosuspend_delay(&usbdev->dev, DEFAULT_AUTOSUSPEND_DELAY);
    pr_debug("} %s\n", __func__);
}

static void baseband_xmm_power_reset_on(void)
{
	/* reset / power on sequence */
	pr_debug("!! %s !!\n", __func__);
#ifndef CP_RESET_SEQUENCE
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 0);
	mdelay(40);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 1);
	udelay(70);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 0);
#else
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 0);
	mdelay(50);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 0);
	mdelay(200);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 1);	
	mdelay(50);
	
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 1);
	udelay(60);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 0);
//	mdelay(100);
//	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 1);
#endif
}

static void baseband_xmm_power_flash_pm_ver_ge_1145_recovery
	(struct work_struct *work)
{
	int timeout_500ms = MODEM_ENUM_TIMEOUT_500MS;
	int timeout_200ms = 0;
	long err;
	
	pr_debug("%s {\n", __func__);


	/* check for platform data */
	if (!baseband_power_driver_data)
		return;

	/* waiting ap_wake up */
	while (ipc_ap_wake_state == IPC_AP_WAKE_INIT1 && timeout_500ms--)
		msleep(500);
	
	if (ipc_ap_wake_state != IPC_AP_WAKE_INIT2) {
		pr_err("err : modem boot repeat condition state(%d)\n",ipc_ap_wake_state);
		timeout_200ms = MODEM_ENUM_TIMEOUT_200MS;
	}
	
	/* check if enumeration succeeded */
	/* waiting ttyacm dev to be created */
	do
	{
		mm_segment_t oldfs;
		struct file *filp;
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		filp = filp_open("/dev/ttyACM0",
			O_RDONLY, 0);
		if (IS_ERR(filp) || (filp == NULL)) {
			msleep(200);
			err = PTR_ERR(filp);
		} else {
			pr_debug("ttyACM0 created OK\n");
			enum_success = true;
			enum_repeat = ENUM_REPEAT_TRY_CNT;
			filp_close(filp, NULL);
			break;
		}
		set_fs(oldfs);
	}while (++timeout_200ms <= MODEM_ENUM_TIMEOUT_200MS);

	if (timeout_200ms > MODEM_ENUM_TIMEOUT_200MS)
		pr_err("/dev/ttyACM0 %ld\n", err); 

	/*
	TODO:power on/off routine
	if repeat does not work.
	*/
	if (!enum_repeat)
		pr_debug("failed to enumerate modem software" " - too many retry attempts\n");

	if (!enum_success && enum_repeat) {
		pr_debug("start repeat enum - attempt #%d\n", ENUM_REPEAT_TRY_CNT - --enum_repeat);
		ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;

		/* unregister usb host controller */
		if (baseband_power_driver_data->hsic_unregister && register_hsic_device == false)
			baseband_power_driver_data->hsic_unregister(baseband_power_driver_data->modem.xmm.hsic_device);
		else
			pr_err("%s: hsic is already unregistered\n", __func__);
		register_hsic_device = true;
	
		/* wait X ms */
		msleep(500);

		/* set IPC_HSIC_ACTIVE low */
		gpio_set_value(baseband_power_driver_data->modem.xmm.ipc_hsic_active,0);
		//pr_debug("ipc_hsic_active -> 0 \n");
		pr_debug("GPIO [W]: Host_active -> 0 \n"); 
	}
	
	pr_debug("%s enum(%d), interval(%d, %d)ms }\n", __func__, enum_success,
		(MODEM_ENUM_TIMEOUT_500MS - timeout_500ms)*500,
		timeout_200ms*200);
}

static void baseband_xmm_power_work_func(struct work_struct *work)
{
	struct baseband_xmm_power_work_t *bbxmm_work = (struct baseband_xmm_power_work_t *) work;

	pr_debug("%s\n", __func__);

	switch (bbxmm_work->state) {
	case BBXMM_WORK_UNINIT:
		pr_debug("BBXMM_WORK_UNINIT\n");
		break;
	case BBXMM_WORK_INIT:
		pr_debug("BBXMM_WORK_INIT\n");
		/* go to next state */
		bbxmm_work->state = (modem_flash && !modem_pm)
			? BBXMM_WORK_INIT_FLASH_STEP1
			: (modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASH_PM_STEP1
			: (!modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASHLESS_PM_STEP1
			: BBXMM_WORK_UNINIT;
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_STEP1\n");
		/* register usb host controller */
		pr_debug("%s: register usb host controller\n", __func__);
		if (baseband_power_driver_data->hsic_register)
			baseband_power_driver_data->modem.xmm.hsic_device =
				baseband_power_driver_data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_STEP1\n");

		if (modem_ver >= XMM_MODEM_VER_1145) {
			gpio_set_value(baseband_power_driver_data->
				modem.xmm.ipc_hsic_active, 1);
			//pr_debug("ipc_hsic_active -> 1 \n");
			pr_debug("GPIO [W]: Host_active -> 1 \n"); 
			
			/* reset / power on sequence */
			baseband_xmm_power_reset_on();
			/* set power status as on */
			power_onoff = 1;
			//pr_debug("%s: ver > 1145:" " ipc_hsic_active -> 0\n", __func__);
			gpio_set_value(baseband_power_driver_data->
				modem.xmm.ipc_hsic_active, 0);			
			pr_debug("%s: ver > 1145:" "GPIO [W]: Host_active -> 0 \n",__func__); 
		} else {
			/* [modem ver >= 1130] start with IPC_HSIC_ACTIVE low */
			if (modem_ver >= XMM_MODEM_VER_1130) {
				//pr_debug("%s: ver > 1130:" " ipc_hsic_active -> 0\n", __func__);
				gpio_set_value(baseband_power_driver_data->modem.xmm.ipc_hsic_active, 0);
				pr_debug("%s: ver > 1130:" "GPIO [W]: Host_active -> 0 \n",__func__); 				
			}
			/* reset / power on sequence */
			baseband_xmm_power_reset_on();
			/* set power status as on */
			power_onoff = 1;
			
			/* optional delay
			 * 0 = flashless
			 *   ==> causes next step to enumerate modem boot rom
			 *       (058b / 0041)
			 * some delay > boot rom timeout
			 *   ==> causes next step to enumerate modem software
			 *       (1519 / 0020)
			 *       (requires modem to be flash version, not flashless
			 *       version)
			 */
			if (enum_delay_ms)
				mdelay(enum_delay_ms);
			/* register usb host controller */
			pr_debug("%s: register usb host controller\n", __func__);
			if (baseband_power_driver_data->hsic_register && register_hsic_device == true) {
				baseband_power_driver_data->modem.xmm.hsic_device =
					baseband_power_driver_data->hsic_register();
					
				register_hsic_device = false;
			}
			else
				pr_err("%s: hsic_register is missing\n", __func__);
		}

		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1
			: BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_STEP1\n");
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ
			: BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1\n");
		break;
//To_Ril-recovery Nvidia_Patch_20111226 [Start]
	case BBXMM_WORK_INIT_FLASH_PM_VER_GE_1145_RECOVERY:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_GE_1145_RECOVERY\n");
		baseband_xmm_power_flash_pm_ver_ge_1145_recovery(work);
		break;
//To_Ril-recovery Nvidia_Patch_20111226 [END]
	default:
		break;
	}

}

static void baseband_xmm_device_add_handler(struct usb_device *udev)
{
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id;

	if (intf == NULL)
		return;

	id = usb_match_id(intf, xmm_pm_ids);

	if (id) {
		pr_debug("persist_enabled: %u\n", udev->persist_enabled);
		pr_debug("Add device %d <%s %s>\n", udev->devnum, udev->manufacturer, udev->product);
		usbdev = udev;
		usb_enable_autosuspend(udev);
		pr_debug("enable autosuspend\n");
	}
}

static void baseband_xmm_device_remove_handler(struct usb_device *udev)
{
	if (usbdev == udev) {
		pr_debug("Remove device %d <%s %s>\n", udev->devnum, udev->manufacturer, udev->product);
		usbdev = 0;
	}

}

static int usb_xmm_notify(struct notifier_block *self, unsigned long action,void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		baseband_xmm_device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		baseband_xmm_device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}


static struct notifier_block usb_xmm_nb = {
	.notifier_call = usb_xmm_notify,
};

static int baseband_xmm_power_pm_notifier_event(struct notifier_block *this,
					unsigned long event, void *ptr)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	unsigned long flags;

	if (!data)
		return NOTIFY_DONE;

	pr_debug("%s: event %ld\n", __func__, event);
	switch (event) {
	case PM_SUSPEND_PREPARE:
		pr_debug("%s : PM_SUSPEND_PREPARE\n", __func__);
		if (wake_lock_active(&wakelock)) {
			pr_debug("%s: wakelock was active, aborting suspend\n",__func__);
			return NOTIFY_STOP;
		}

		spin_lock_irqsave(&xmm_lock, flags);
		if (wakeup_pending) {
			wakeup_pending = false;
			spin_unlock_irqrestore(&xmm_lock, flags);
			pr_debug("%s : XMM busy : Abort system suspend\n",__func__);
			return NOTIFY_STOP;
		}
		system_suspending = true;
		spin_unlock_irqrestore(&xmm_lock, flags);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		pr_debug("%s : PM_POST_SUSPEND\n", __func__);
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		if (wakeup_pending &&
			(baseband_xmm_powerstate == BBXMM_PS_L2)) {
			wakeup_pending = false;
			spin_unlock_irqrestore(&xmm_lock, flags);
			pr_debug("%s : Service Pending CP wakeup\n",__func__);
			CP_initiated_L2toL0 = true;
			baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
			return NOTIFY_OK;
		}
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static int baseband_xmm_shutdown_function()
{
	disable_irq(gpio_to_irq(baseband_power_driver_data->modem.xmm.ipc_ap_wake));
	if ( l2_resume_work_done ) {
		flush_workqueue(workqueue_susp);
		flush_workqueue(workqueue);
	} else {
		pr_err("%s:Called reboot_notify during resuming\n", __func__);
	}
	
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 0);
	udelay(200);	
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 0);
	mdelay(20);
}


static struct notifier_block baseband_xmm_power_pm_notifier = {
	.notifier_call = baseband_xmm_power_pm_notifier_event,
};

static int baseband_xmm_reboot_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
	pr_debug("%s\n", __func__);

    switch (event) {
    case SYS_RESTART:
    case SYS_HALT:
    case SYS_POWER_OFF:
		{
			baseband_xmm_shutdown_function();
		}
	    return NOTIFY_OK;
    }
    return NOTIFY_DONE;
}


static struct notifier_block baseband_xmm_reboot_nb = {
	.notifier_call = baseband_xmm_reboot_notify,
};

static int baseband_xmm_power_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)device->dev.platform_data;
	struct device *dev = &device->dev;
	unsigned long flags;	
	int err;

	pr_debug("%s\n", __func__);
	pr_debug("[XMM] enum_delay_ms=%ld\n", enum_delay_ms);

	/* check for platform data */
	if (!data)
		return -ENODEV;

	/* check if supported modem */
	if (data->baseband_type != BASEBAND_XMM) {
		pr_err("unsuppported modem\n");
		return -ENODEV;
	}

	/* save platform data */
	baseband_power_driver_data = data;

	/* create device file */
	err = device_create_file(dev, &dev_attr_xmm_onoff);
	if (err < 0) {
		pr_err("%s - device_create_file failed\n", __func__);
		return -ENODEV;
	}

	/* init wake lock */
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "baseband_xmm_power");

	/* init spin lock */
	spin_lock_init(&xmm_lock);
	/* request baseband gpio(s) */
	tegra_baseband_gpios[0].gpio = baseband_power_driver_data->modem.xmm.bb_rst;
	tegra_baseband_gpios[1].gpio = baseband_power_driver_data->modem.xmm.bb_on;
	tegra_baseband_gpios[2].gpio = baseband_power_driver_data->modem.xmm.ipc_bb_wake;
	tegra_baseband_gpios[3].gpio = baseband_power_driver_data->modem.xmm.ipc_ap_wake;
	tegra_baseband_gpios[4].gpio = baseband_power_driver_data->modem.xmm.ipc_hsic_active;
	tegra_baseband_gpios[5].gpio = baseband_power_driver_data->modem.xmm.ipc_hsic_sus_req;
	err = gpio_request_array(tegra_baseband_gpios,ARRAY_SIZE(tegra_baseband_gpios));
	if (err < 0) {
		pr_err("%s - request gpio(s) failed\n", __func__);
		return -ENODEV;
	}

	/* request baseband irq(s) */
	if (modem_flash && modem_pm) {
		pr_debug("%s: request_irq IPC_AP_WAKE_IRQ\n", __func__);
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;

		err = request_threaded_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake), 
					NULL,
					baseband_xmm_power_ipc_ap_wake_irq, 
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 
					"IPC_AP_WAKE_IRQ", 
					NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",__func__);
			return err;
		}
		
		disable_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake));		

		err = enable_irq_wake(gpio_to_irq(data->modem.xmm.ipc_ap_wake));
	    if (err < 0)
	           pr_err("%s: enable_irq_wake error\n", __func__);
	
		ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
		if (modem_ver >= XMM_MODEM_VER_1145) {
			pr_debug("%s: ver > 1145: AP_WAKE_READY\n", __func__);
			/* ver 1145 or later starts in READY state */
			/* ap_wake keeps low util CP starts to initiate hsic hw. */
			/* ap_wake goes up during cp hsic init and then */
			/* it goes down when cp hsic ready */
			ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;

		} else if (modem_ver >= XMM_MODEM_VER_1130) {
			pr_debug("%s: ver > 1130: AP_WAKE_INIT1\n", __func__);
			/* ver 1130 or later starts in INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
		}
	}

	/* init work queue */
	workqueue = create_singlethread_workqueue("baseband_xmm_power_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}

	workqueue_susp = alloc_workqueue("baseband_xmm_power_autosusp", 
				WQ_UNBOUND | WQ_HIGHPRI | WQ_NON_REENTRANT, 1);

	if (!workqueue_susp) {
		pr_err("cannot create workqueue_susp\n");
		return -1;
	}
	
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *)
		kmalloc(sizeof(struct baseband_xmm_power_work_t), GFP_KERNEL);

	if (!baseband_xmm_power_work) {
		pr_err("cannot allocate baseband_xmm_power_work\n");
		return -1;
	}
	
	INIT_WORK((struct work_struct *) baseband_xmm_power_work, baseband_xmm_power_work_func);
	baseband_xmm_power_work->state = BBXMM_WORK_INIT;
	queue_work(workqueue,(struct work_struct *) baseband_xmm_power_work);

	/* init work objects */
	INIT_WORK(&init1_work, baseband_xmm_power_init1_work);
	INIT_WORK(&init2_work, baseband_xmm_power_init2_work);
	INIT_WORK(&L2_resume_work, baseband_xmm_power_L2_resume_work);
	
	INIT_WORK(&work_shortsusp, baseband_xmm_power_shortsusp);
	INIT_WORK(&work_defaultsusp, baseband_xmm_power_defaultsusp);
	

	/* init state variables */
	register_hsic_device = true;
	CP_initiated_L2toL0 = false;
	spin_lock_irqsave(&xmm_lock, flags);
	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	wakeup_pending = false;
	system_suspending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	usb_register_notify(&usb_xmm_nb);
	register_pm_notifier(&baseband_xmm_power_pm_notifier);
	register_reboot_notifier(&baseband_xmm_reboot_nb);

	enable_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake));

	pr_debug("%s }\n", __func__);
	return 0;
}

static int baseband_xmm_power_driver_remove(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	struct device *dev = &device->dev;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	unregister_pm_notifier(&baseband_xmm_power_pm_notifier);
	usb_unregister_notify(&usb_xmm_nb);

	/* free work structure */
	kfree(baseband_xmm_power_work);
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *) 0;

	/* free baseband irq(s) */
	if (modem_flash && modem_pm) {
		free_irq(gpio_to_irq(baseband_power_driver_data->modem.xmm.ipc_ap_wake), NULL);
	}

	/* free baseband gpio(s) */
	gpio_free_array(tegra_baseband_gpios,ARRAY_SIZE(tegra_baseband_gpios));

	/* destroy wake lock */
	wake_lock_destroy(&wakelock);

	/* delete device file */
	device_remove_file(dev, &dev_attr_xmm_onoff);

	/* destroy wake lock */
	destroy_workqueue(workqueue_susp);
	destroy_workqueue(workqueue);

	/* unregister usb host controller */
	if (data->hsic_unregister)
		data->hsic_unregister(data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	return 0;
}

static int baseband_xmm_power_driver_shutdown(struct platform_device *device)
{
	pr_debug("%s\n", __func__);

	baseband_xmm_shutdown_function();

	return 0;
}

static int baseband_xmm_power_driver_handle_resume(
            			struct baseband_power_platform_data *data)
{

	int value;
	int delay = 1000; /* maxmum delay in msec */ //1000 is Intel recovmmand value
    unsigned long flags;

	value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
	//pr_debug("%s host_wake(%d)\n", __func__, value);
	pr_debug("%s GPIO [R]: Host_wakeup = %d \n",__func__,value); 

    if (!data)
                return 0;

	/* check if modem is on */
	if (power_onoff == 0) {
		pr_debug("%s - flight mode - nop\n", __func__);
		return 0;
	}

	modem_sleep_flag = false;	
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	/* L3->L0 */
	baseband_xmm_set_power_status(BBXMM_PS_L3TOL0);
	value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
	if (value) {
		pr_debug("PM_ST : AP L3 -> L0\n");
		/* wake bb */
		gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);
		pr_debug("GPIO [W]: Slave_wakeup -> 1 \n"); 
		pr_debug("waiting for host wakeup...\n");
		do {
			mdelay(1);
			value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
			delay--;
		} while ((value) && (delay));
		if (delay) {
			//pr_debug("GPIO [R]: Host_wakeup = %d \n",value); 
			pr_debug("%s enable short_autosuspend\n", __func__);
			short_autosuspend = true;
		}
		else
			pr_debug("PM_ST : !!AP L3->L0 Failed and CP is not response!!\n");
	} else {
		pr_debug("PM_ST : CP L3 -> L0\n");
	}
	reenable_autosuspend = true; //20120112 - Nv Bug 924425 - L2 Auto Suspend#1
    
	return 0;
}

#ifdef CONFIG_PM
static int baseband_xmm_power_driver_suspend(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int baseband_xmm_power_driver_resume(struct device *dev)
{
     struct platform_device *pdev = to_platform_device(dev);
     struct baseband_power_platform_data *data
                = (struct baseband_power_platform_data *)pdev->dev.platform_data;

     pr_debug("%s\n", __func__);
     baseband_xmm_power_driver_handle_resume(data);

     return 0;
}

static int baseband_xmm_power_suspend_noirq(struct device *dev)
{
    unsigned long flags;

    pr_debug("%s !!\n", __func__);
    spin_lock_irqsave(&xmm_lock, flags);
	system_suspending = false;
    if (wakeup_pending) {
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);	//                                                                  
		pr_info("%s:**Abort Suspend: reason CP WAKEUP**\n", __func__);
		pr_info("%s:**spin unlock **\n", __func__);
		return -EBUSY;
    }
    spin_unlock_irqrestore(&xmm_lock, flags);
    return 0;
}

static int baseband_xmm_power_resume_noirq(struct device *dev)
{
	int value;
	value = gpio_get_value(baseband_power_driver_data->modem.xmm.ipc_ap_wake);
	//pr_debug("%s host_wake(%d)\n", __func__, value);
	pr_debug("%s GPIO [R]: Host_wakeup = %d \n",__func__,value); 
    return 0;
}

static const struct dev_pm_ops baseband_xmm_power_dev_pm_ops = {
	.suspend_noirq = baseband_xmm_power_suspend_noirq,
	.resume_noirq = baseband_xmm_power_resume_noirq,
	.suspend = baseband_xmm_power_driver_suspend,
	.resume = baseband_xmm_power_driver_resume,
};	
#endif

static struct platform_driver baseband_power_driver = {
	.probe = baseband_xmm_power_driver_probe,
	.remove = baseband_xmm_power_driver_remove,
	.shutdown = baseband_xmm_power_driver_shutdown,
	.driver = {
		       .name = "baseband_xmm_power",
#ifdef CONFIG_PM
	           .pm   = &baseband_xmm_power_dev_pm_ops,
#endif		
	},
};

static int __init baseband_xmm_power_init(void)
{
	if(is_tegra_bootmode())
	{
		pr_debug("%s\n", __func__);
		return platform_driver_register(&baseband_power_driver);
	}
}

static void __exit baseband_xmm_power_exit(void)
{
	if(is_tegra_bootmode())
	{
		pr_debug("%s\n", __func__);
		platform_driver_unregister(&baseband_power_driver);
	}
}

module_init(baseband_xmm_power_init)
module_exit(baseband_xmm_power_exit)
