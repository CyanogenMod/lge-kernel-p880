
/*
 * LGE - CP Watcher Driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 *
 * Author: Kyungsik Lee <kyungsik.lee@lge.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>

#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>

#include <asm/gpio.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/system.h>

#include "../../arch/arm/mach-tegra/gpio-names.h"
#include "../../arch/arm/mach-tegra/lge/x3/include/lge/board-x3-nv.h"


#include <linux/slab.h>         //kzalloc
#include <linux/errno.h>
#include <linux/kernel.h>

#include <linux/wakelock.h> 
#include <linux/rtc.h>


#define NV_DEBUG 0

#if 1 //defined(CONFIG_DEBUG_FS)

#include <linux/debugfs.h>

#define ENABLE_DEBUG_MESSAGE 0x01
#define ENABLE_TEST_MODE 0x02
#define KEYBACKLIGHT_LEDS_GPIO 138 // RIP-73256


#define CPW_SIMPLE_DEBUG

#ifdef CPW_SIMPLE_DEBUG
static int debug_enable_flag = 1;//Debug
#else
static int debug_enable_flag = 0;//Release
#endif

#endif//CONFIG_DEBUG_FS

/*
 * Debug
 */
#define DEBUG_CP
#ifdef DEBUG_CP
#define DBG(x...) if (debug_enable_flag & ENABLE_DEBUG_MESSAGE) { \
						printk(x); \
				  }
#else
#define DBG(x...) do { } while(0)
#endif


#if !defined(TRUE)
#define TRUE 0x01
#endif
#if !defined(FALSE)
#define FALSE 0x00 
#endif


//#define DEBUG_CPWATCHER_REPORT_INPUT
#define DELAY_SECONDS 8
#define REPORT_DELAY (DELAY_SECONDS * 1000)
#define CPWATCHER_DELAY_TIME msecs_to_jiffies(5)

/*
 * Set scancode for hidden menu
 */
//#ifdef CPW_SIMPLE_DEBUG
//#define EVENT_KEY KEY_SEARCH //debug
//#else
#define EVENT_KEY KEY_F24 //194, Need to be changed

#if defined (CONFIG_MODEM_IFX)
#define EVENT_HARD_RESET_KEY	195	//this key number is not used in input.h
#endif
//#endif

#define	HEADSET_PORT 6
#define	HEADSET_PIN 3
#if defined (CONFIG_MODEM_IFX)
#define ENABLE_CP_HARD_RESET
#endif

//static int cp_crash_int_pin		= TEGRA_GPIO_PP1;		//Temporary for Rev.B test
static int cp_crash_int_pin		= TEGRA_GPIO_PS5;		//Rev.C

extern int is_cp_crash;  //RIP-13119 : RIL recovery should be started by USB-disconnection.
// NvU32 ---> unsigned int
// NvU8 ----> unsigned char
struct cpwatcher_dev {

	struct input_dev *input;
        unsigned char onoff;
        unsigned int delay;
        
	struct delayed_work delayed_work_cpwatcher;
	struct mutex lock;
#ifdef DEBUG_CPWATCHER_REPORT_INPUT
	struct task_struct  *task;
#endif
        struct wake_lock wake_lock;

};
static struct cpwatcher_dev *cpwatcher;


static int lge_is_crash_dump_enabled()
{
	char data[2] = {0x00,0x00};

	lge_nvdata_read(LGE_NVDATA_CRASH_DUMP_OFFSET,data,1);
	msleep(100);

	if(data[0] == LGE_NDATA_CRASH_DUMP_ENABLE_VALUE)
		return 1;
	else
		return 0;
}
 //RIP-73256 : [X3] CTS issue : permission error - byeonggeun.kim [START]
static int lge_is_ril_recovery_mode_enabled()
{
	char data[2] = {0x00,0x00};

	lge_nvdata_read(LGE_NVDATA_RIL_RECOVERY_MODE_OFFSET,data,1);
	msleep(100);

	if(data[0] == 0x01)
		return 1;
	else
		return 0;
}
//RIP-73256 : [X3] CTS issue : permission error - byeonggeun.kim [END]

#if 1 //defined(CONFIG_DEBUG_FS)
static int debug_control_set(void *data, u64 val)
{

	if (val & ENABLE_DEBUG_MESSAGE) {
		
		debug_enable_flag |= ENABLE_DEBUG_MESSAGE;
	}
	if (val & ENABLE_TEST_MODE) {
		debug_enable_flag |= ENABLE_TEST_MODE;
		input_report_key(cpwatcher->input, EVENT_KEY, 1);
		input_report_key(cpwatcher->input, EVENT_KEY, 0);
	}

	if (!val) {

		debug_enable_flag = 0;
	}

	return 0;
}


static int debug_control_get(void *data, u64 *val)
{

	*val = debug_enable_flag;
	 
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(debug_fops, 
                        debug_control_get, debug_control_set, "%llu\n");


static int __init cpwatcher_debug_init(void)
{
    //RIP-73256 : [X3] CTS issue : permission error - byeonggeun.kim [START]
	debugfs_create_file("cpwatcher", 0664,
					NULL, NULL, &debug_fops);
    //RIP-73256 : [X3] CTS issue : permission error - byeonggeun.kim [END]
	return 0;
}
#endif//CONFIG_DEBUG_FS

static void platform_release(struct device *dev);

static struct platform_device cp_device = { 
		    .name       = "cpwatcher",
			    .id     = -1,
        {
		.platform_data = NULL, 		
		.release = platform_release,    /* a warning is thrown during rmmod if this is absent */
	},			    
};


static void platform_release(struct device *dev) 
{	
    printk("tspdrv: platform_release.\n");
}

#ifdef CPW_SIMPLE_DEBUG
static unsigned int test_flag = 1;//debug with headset detect
#else
static unsigned int test_flag = 0;
#endif
module_param(test_flag, int, 0);


static void cpwatcher_get_status(unsigned int *pin)
{
	//NvOdmGpioGetState(cpwatcher->hGpio, cpwatcher->hCP_status, pin);
	*pin = gpio_get_value(cp_crash_int_pin);
}

static void cpwatcher_irq_handler(void *dev_id)
{
	struct cpwatcher_dev *dev = cpwatcher;

	if (dev->onoff) {
		printk("[CPW] %s()\n", __FUNCTION__);
		//NvOdmGpioGetState(cpwatcher_dev.hGpio, cpwatcher_dev.hCP_status, &pinValue);
		schedule_delayed_work(&dev->delayed_work_cpwatcher, dev->delay);
	}

	//NvOdmGpioInterruptDone(cpwatcher->hGpioInterrupt);
}


#ifdef DEBUG_CPWATCHER_REPORT_INPUT
static int cpwatcher_thread(void *pd)
{
	unsigned int status = 0;


	for (;;) {
		
		cpwatcher_get_status(&status);

		DBG("[CPW] CP status: %s\n", status? "error" : "available");

		if (status) {//If High, CP error
			input_report_key(cpwatcher->input, EVENT_KEY, 1);
			input_report_key(cpwatcher->input, EVENT_KEY, 0);
			DBG("[CPW] Report Key: %d\n", EVENT_KEY);
		}
		input_sync(cpwatcher->input);

		mdelay(REPORT_DELAY);
	}

	return 0;
}
#endif

extern volatile int time_to_stop; //                                                                                                             

static void cpwatcher_work_func(struct work_struct *wq)
{
	struct cpwatcher_dev *dev = cpwatcher;
	unsigned int status = 0;
    //int ret = 0;
        
	//unsigned char data;				//Blocked due to CS Issue
    struct timeval now;
    struct tm gmt_time;

	char* argv[] = {"/system/bin/ifx_coredump", "CP_CRASH_IRQ", NULL};
	char *envp[] = { "HOME=/",	"PATH=/sbin:/bin:/system/bin",	NULL };	

    do_gettimeofday(&now);
    time_to_tm(now.tv_sec, 0, &gmt_time);
        
    printk("[CPW] cpwatcher_work_func()\n");
    printk(KERN_ERR "%d-%d-%d %d:%d:%d.%ld *\n",
        gmt_time.tm_year + 1900, gmt_time.tm_mon + 1, 
        gmt_time.tm_mday, gmt_time.tm_hour - sys_tz.tz_minuteswest / 60,
        gmt_time.tm_min, gmt_time.tm_sec, now.tv_usec);
        
    time_to_stop = 1; //                                                                                                             
	if (dev->onoff) {

		cpwatcher_get_status(&status);
		printk("[CPW] %s(), status: %d\n", __FUNCTION__, status);

		if (status == 0) {//If High, CP error

            wake_lock(&dev->wake_lock);

            // UPDATE CP_CRASH_COUNT -- //Blocked due to CS Issue
            //                                                            
            //data++;
            //                                                             

            // CHECK CP_CRASH_DUMP OPTION
            //if (lge_is_crash_dump_enabled() != 1) //this line will be enabled rev.e
            {
                input_report_key(dev->input, EVENT_KEY, 1);
                input_report_key(dev->input, EVENT_KEY, 0);
                input_sync(dev->input);
                printk("[CPW] CP Crash : input_report_key(): %d\n", EVENT_KEY);
                is_cp_crash = 1; //RIP-13119 : RIL recovery should be started by USB-disconnection.
                
                //RIP-73256 - S
                if(lge_is_ril_recovery_mode_enabled() != 1)
                {
                    printk("[CPW] CP Crash : lge_is_ril_recovery_mode_enabled() = 1 ...change to CP_USB mode \n");
                    gpio_set_value(KEYBACKLIGHT_LEDS_GPIO, 1);      //Temporary -- Keybacklight on when occurs CP Crash

                    extern void muic_proc_set_cp_usb_force(void);
                    muic_proc_set_cp_usb_force();
                }
                wake_unlock(&dev->wake_lock);
                //RIP-73256 - E
                return;
            }

    //                {
    //                        extern void tegra_ehci_shutdown_global(void);
    //                        tegra_ehci_shutdown_global();
    
    //                }
    
    //        Enabled after Fixing Baudrate Issue 
    //        ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
    //        printk("[CP CRASH IRQ] launch ifx_coredump process ret:%d\n", ret);
                        
		}
	}
}


static ssize_t
cpwatcher_show_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	int result = 0;


	if (!cpwatcher) return 0;

	result  = snprintf(buf, PAGE_SIZE, "%s\n", (cpwatcher->onoff == TRUE)  ? "1":"0");
	
	return result;
}


static ssize_t
cpwatcher_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	
	
	if (!count)
		return -EINVAL;

	mutex_lock(&cpwatcher->lock);
	sscanf(buf, "%d", &onoff);

	if (onoff) {

		cpwatcher->onoff = TRUE;
		printk("[CPW] On\n");
	} else {

		cpwatcher->onoff = FALSE;
		printk("[CPW] Off\n");
	}
	mutex_unlock(&cpwatcher->lock);

	return count;
}

//RIP-73256 : [X3] CTS issue : permission error - byeonggeun.kim [START]
static DEVICE_ATTR(onoff, 0664, cpwatcher_show_onoff, cpwatcher_store_onoff);
//RIP-73256 : [X3] CTS issue : permission error - byeonggeun.kim [END]

static struct attribute *cpwatcher_attributes[] = {
	&dev_attr_onoff.attr,
	NULL,
};


static const struct attribute_group cpwatcher_group = {
	.attrs = cpwatcher_attributes,
};

static int cpwatcher_probe(struct platform_device *pdev)
{
	struct cpwatcher_dev *dev; 
	struct device *dev_sys = &pdev->dev;
	int i, j;
//	NvBool ret_status = 0;
	int ret = 0;

	dev = kzalloc(sizeof(struct cpwatcher_dev), GFP_KERNEL);
	if (!dev) {

		ret = -ENOMEM;
		goto fail_malloc;
	}
	cpwatcher = dev;

	/* Input */
	dev->input = input_allocate_device();
	if (!dev->input) {
		//printk(KERN_ERR "button.c: Not enough memory\n");
		ret = -ENOMEM;
		goto fail_input_allocate;
	}

	dev->input->name = "cp_watcher";
	set_bit(EV_KEY, dev->input->evbit);
	set_bit(EV_SYN, dev->input->evbit);
	set_bit(EVENT_KEY, dev->input->keybit);


	ret = input_register_device(dev->input);
	if (ret) {

		goto fail_input_register;
	}

	dev->onoff= TRUE;
	dev->delay = CPWATCHER_DELAY_TIME;
	INIT_DELAYED_WORK(&dev->delayed_work_cpwatcher, cpwatcher_work_func);

	/* GPIO */
        //Rev.B : temporary port (TEGRA_GPIO_PP1) for verifiy the CP CRASH
        //Rev.C: KB_ROW13 / TEGRA_GPIO_PS5

        tegra_gpio_enable(cp_crash_int_pin);
        
        ret = gpio_request(cp_crash_int_pin, "CP_CRASH_INT");
        if (ret < 0)
        {
             printk("CP_CRASH[1] : Fail to request CP_CRASH_INT enabling\n");
             goto fail_gpio_open;
        }

        ret=gpio_direction_input(cp_crash_int_pin);
        if (ret < 0)
        {
             printk("CP_CRASH[2] : Fail to direct CP_CRASH_INT enabling\n");
             gpio_free(cp_crash_int_pin);
             goto fail_gpio_open;
        }               
	printk("[CPW] CP_CRASH_INT : %d, Line[%d]\n",	gpio_get_value(cp_crash_int_pin),__LINE__); 	
	
        ret = request_irq(TEGRA_GPIO_TO_IRQ(cp_crash_int_pin), cpwatcher_irq_handler,
              IRQF_TRIGGER_FALLING,
              "cpwatcher", dev);

      if (ret < 0) {
              printk("%s Could not allocate APDS990x_INT !\n", __func__);
              goto fail_gpio_open;
      }


#ifdef CPW_CHECK_STATUS_AT_BEGINNING
	/* Check the status at the beginning */
	cpwatcher_get_status(&dev->status);
	printk("[CPW] CP status: %s\n", dev->status? "error" : "available");
	if (dev->status) {//If High, CP error
		input_report_key(dev->input, EVENT_KEY, 1);
		input_report_key(dev->input, EVENT_KEY, 0);
		input_sync(dev->input);
	}
#endif
	
	if (sysfs_create_group(&dev_sys->kobj, &cpwatcher_group)) {

		printk("[CPW] Failed to create sys filesystem\n");
	}

	mutex_init(&dev->lock);
	
#ifdef DEBUG_CPWATCHER_REPORT_INPUT
	/* Create kernel thread */
	dev->task = kthread_create(cpwatcher_thread, 0, "cpwatcher_thread");
	if (!dev->task) {

		printk("[CPW] kthread_create(): fail\n");
		goto fail_input_allocate; 
	}
	wake_up_process(dev->task);
#endif 

        wake_lock_init(&dev->wake_lock, WAKE_LOCK_SUSPEND, "cpwatcher");

    printk("[CPW] CP Watcher Initialization completed\n");

	return 0;

fail_gpio_open:
	input_unregister_device(dev->input);

fail_input_register:
	input_free_device(dev->input);

fail_free_irq:
	free_irq(TEGRA_GPIO_TO_IRQ(cp_crash_int_pin), dev);

fail_input_allocate:
	kfree(dev);

fail_malloc:

	return ret;
}


static int __devexit cpwatcher_remove(struct platform_device *pdev)
{
	int ret = 0;

	input_unregister_device(cpwatcher->input);

	input_free_device(cpwatcher->input);

	free_irq(TEGRA_GPIO_TO_IRQ(cp_crash_int_pin), cpwatcher);
        wake_lock_destroy(&cpwatcher->wake_lock);

	kfree(cpwatcher);

	DBG("[CPW] %s(): success\n", __FUNCTION__);

	return ret;
}


static struct platform_driver cpwatcher_driver = {
	.probe		= cpwatcher_probe,
	.remove		= __devexit_p(cpwatcher_remove),
	.driver		= 
	{
		.name	= "cpwatcher",
		.owner	= THIS_MODULE,
	},
};


static int __init cpwatcher_init(void)
{
    printk("[CPW] CP Watcher Initialization started\n");

#if 1 //defined(CONFIG_DEBUG_FS)
#ifdef DEBUG_CP
	cpwatcher_debug_init();
#endif//DEBUG_CP
#endif//CONFIG_DEBUG_FS


	if (platform_device_register(&cp_device)) {

		printk("[CPW] %s(): fail\n", __FUNCTION__);
	}

	return platform_driver_register(&cpwatcher_driver);
}
module_init(cpwatcher_init);
	
		 
static void __exit cpwatcher_exit(void)
{
	platform_driver_unregister(&cpwatcher_driver);
	printk("[CPW] %s(): success\n", __FUNCTION__);
}
module_exit(cpwatcher_exit);


MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("LGE CP Watcher Driver");
MODULE_LICENSE("GPL");


