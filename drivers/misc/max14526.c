/*
 * drivers/misc/max14526.c
 *
 * Copyright (C) 2010-2011 LG Electronics Inc.
 *
 * Author: Sookyoung Kim <>
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
 */

#include <linux/module.h>
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>			/* __init, __exit */
#include <linux/uaccess.h>		/* copy_from/to_user() */
#include <linux/interrupt.h>		/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <linux/syscalls.h>
#include <asm/system.h>
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>		/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>		/* INIT_WORK() */
#include <linux/wakelock.h>
#include <linux/mutex.h>

#include <linux/power_supply.h>

#include <linux/max14526.h>

struct max14526_status {
	struct i2c_client		*client;
	//struct power_supply		psy_power;
	struct delayed_work 		detect_work;
	struct max14526_platform_data	*pdata;
	muic_mode_t 			mode;
	bool				is_charing;	
	bool				is_boottime;
};

static int max14526_set_ap_uart(struct max14526_status *status);
static int max14526_set_ap_usb(struct max14526_status *status);
static int max14526_set_cp_uart(struct max14526_status *status);
static int max14526_set_cp_usb(struct max14526_status *status);

static int max14526_detect_accessory(struct max14526_status *status);
static void max14526_set_mode(struct max14526_status *status, 
		unsigned char int_stat_value);


static int max14526_i2c_read_byte(struct max14526_status *status, 
				  u8 addr, 
				  u8 *value)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(status->client, (u8)addr);
	if (ret < 0) {
		printk("[MUIC] %s failed.\n", __func__);
		return ret;
	} 
	else {
		*value = (u8) ret;

		return 0;
	}
}

static int max14526_i2c_write_byte(struct max14526_status *status, 
				   u8 addr, 
				   u8 value)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(status->client, (u8)addr, (u8)value);

	if (ret < 0) {
		printk(KERN_INFO "[MUIC] %s failed.\n", __func__);
	}
		
	return ret;
}

static int max14526_init(struct max14526_status *status)
{
	/* Clears Default Switch Position (0x03=0x24) */
	max14526_i2c_write_byte(status, SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); 

	/* Enables 200K pull-up and ADC (0x01=0x12)*/
	max14526_i2c_write_byte(status, CONTROL_1, ID_200 | ADC_EN);

	/* Enables Interrupt and set AUD Click/Pop resistor (0x02=0x50) */
	max14526_i2c_write_byte(status, CONTROL_2, INT_EN);

	return 0;
}

static int max14526_set_ap_uart(struct max14526_status *status)
{
	int ret;

	/* Enables UART Path (0x03=0x09) */
	ret = max14526_i2c_write_byte(status, SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	ret = max14526_i2c_write_byte(status, CONTROL_1, ID_200 | ADC_EN | CP_EN);

	printk(KERN_INFO "[MUIC] muic_proc_set_ap_uart(): AP_UART\n");

	return ret;
}

static int max14526_set_ap_usb(struct max14526_status *status)
{
	int ret;

	ret = max14526_i2c_write_byte(status, SW_CONTROL, OPEN);
	//udelay(100000);

	/* Enables USB Path (0x03=0x00) */
	max14526_i2c_write_byte(status, SW_CONTROL, COMP2_TO_DP2 | COMN1_TO_DN1);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	max14526_i2c_write_byte(status, CONTROL_1, ID_200 | ADC_EN | CP_EN);

	printk(KERN_WARNING "[MUIC] muic_proc_set_ap_usb(): AP_USB\n");

	//ret = muic_i2c_write_byte(CONTROL_2, INT_EN);

	return ret;
}

static int max14526_set_cp_uart(struct max14526_status *status)
{
	int ret;

	/* Connect DP, DM to UART_TX, UART_RX */
	ret = max14526_i2c_write_byte(status, SW_CONTROL, DP_UART | DM_UART);

	printk(KERN_WARNING "[MUIC] muic_proc_set_cp_uart(): CP_UART\n");

	return ret;
}

static int max14526_set_cp_usb(struct max14526_status *status)
{
	s32 ret;

	ret = max14526_i2c_write_byte(status, SW_CONTROL, OPEN);
	udelay(100000);
	
	/* Connect DP, DM to UART_TX, UART_RX */
	ret = max14526_i2c_write_byte(status, SW_CONTROL, DP_UART | DM_UART);
	
	printk(KERN_ERR "[MUIC] muic_proc_set_cp_usb(): CP_USB\n");

	return ret;
}

static void max14526_set_chg_mode(struct max14526_status *status,
				  unsigned char int_stat_value)
{
	unsigned char reg_value;

	printk(KERN_WARNING "[chahee.kim] set_max14526_charger_mode, "
			"int_stat_value = %x \n", int_stat_value );

	if (((int_stat_value & IDNO) == IDNO_0101) || 
			((int_stat_value & IDNO) == IDNO_1011)) {
		/* LG Proprietary TA Detected 180K ohm on ID */
		status->mode = MUIC_MODE_LG_TA;
	} 
	else if ((int_stat_value & IDNO) == IDNO_0110) {
		/* 1A charger detected */
		status->mode = MUIC_MODE_TA_1A;
	} 
	else {
		/* Enable interrpt and charger type detection (0x02=0x42) */
		max14526_i2c_write_byte(status, CONTROL_2, INT_EN | CHG_TYPE);

		/* Read INT_STAT */
		max14526_i2c_read_byte(status, INT_STAT, &reg_value);
		max14526_i2c_read_byte(status, STATUS, &reg_value);

		if (reg_value & DCPORT) {
			/* Dedicated Charger(TA) Detected */
			status->mode = MUIC_MODE_NA_TA;
		} 
		else if (reg_value & CHPORT) {
			status->mode = MUIC_MODE_AP_USB;
			max14526_set_ap_usb(status);
		}
	}
}

static void max14526_set_mode(struct max14526_status *status,
			      unsigned char int_stat_value)
{
	unsigned char reg_value;
	
	printk(KERN_WARNING "[chahee.kim] max14526_set_mode, "
			"int_stat_value = %x \n", int_stat_value);

	if (int_stat_value & CHGDET) {
		max14526_set_chg_mode(status, int_stat_value);
		printk("%s(%d) checkpoint \n", __func__, __LINE__);
	} 
	else if (int_stat_value & VBUS) {
		if ((int_stat_value & IDNO) == IDNO_0101) {
			status->mode = MUIC_MODE_AP_USB;
			max14526_set_ap_usb(status);
		} 
		else {
			/* Standard USB Host Detected (0x03=0x23) */
			max14526_i2c_write_byte(status, SW_CONTROL, 
					COMP2_TO_HZ | COMN1_TO_C1COMP);

			udelay(1000);

			/* Read Status Reigster(0x05) */
			max14526_i2c_read_byte(status, STATUS, &reg_value);

			if (reg_value & C1COMP) {
				/* Charger Detected COMP2/COMN1 to H-Z (0x03=0x24) */
				max14526_i2c_write_byte(status, SW_CONTROL, 
						COMP2_TO_HZ | COMN1_TO_HZ);
				/* Dedicated Charger(TA) Detected */
				status->mode = MUIC_MODE_NA_TA;
			} 
			else {
				/* Turn on USB switches */
				status->mode = MUIC_MODE_AP_USB;
				max14526_set_ap_usb(status);
			}
		}
	} 
	else if ((int_stat_value & IDNO) == IDNO_0010) {
		status->mode = MUIC_MODE_AP_UART;
		max14526_set_ap_uart(status);
	}
	else if ((int_stat_value & IDNO) == IDNO_0101) {
		status->mode = MUIC_MODE_AP_UART;
		max14526_set_ap_uart(status);
	} 
	else {
		/* Accessory Not Supported */
		status->mode = MUIC_MODE_NONE;
		max14526_init(status);
	}

	status->pdata->enable_charger(status->mode);
}


static int max14526_detect_accessory(struct max14526_status *status)
{
	s32 ret = 0;
	int loop = 0;

	u8 int_stat_value;

	/* Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STAT and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STAT and STATUS contents will be held).
	 *
	 * Do this only if muic_max14526_detect_accessory() was called upon IRQ.
	 */
	if (!(status->is_boottime)) {
		for (; loop < 250; loop++)
			udelay(1000);
	}

	/* Reads INT_STAT */
	ret = max14526_i2c_read_byte(status, INT_STAT, &int_stat_value);

	if (ret < 0) {
		printk(KERN_INFO "[chahee.kim] INT_STAT reading failed\n");
		status->mode = MUIC_MODE_UNKNOWN;
		return ret;
	}

	status->mode = int_stat_value & IDNO;
	printk(KERN_INFO "[chahee.kim] IDNO = %d\n", status->mode);


	/* Branches according to the previous muic_mode */
	switch (status->mode) {

		/* MUIC_UNKNOWN is reached in two cases both do not have 
		 * nothing to do with IRQ.
		 * First, at the initialization time where the muic_mode 
		 * is not available yet.
		 * Second, whenever the current muic_mode detection is failed.
		 */
		case MUIC_MODE_UNKNOWN :

		/* If the previous muic_mode was MUIC_NONE,
		 * the only possible condition for a MUIC IRQ is 
		 * plugging in an accessory.
		 */
		case MUIC_MODE_NONE :
			max14526_set_mode(status, int_stat_value);           
			break;

		/* If the previous muic_mode was MUIC_NA_TA, 
		 * MUIC_LG_TA, MUIC_TA_1A, MUIC_INVALID_CHG, 
		 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, 
		 * MUIC_OTG, or MUIC_CP_USB, the only possible 
		 * condition for a MUIC IRQ is plugging out the 
		 * accessory.
		 * 
		 * In this case, initialize MUIC and wait an IRQ.
		 * We don't need to wait 250msec because this is not an erronous case
		 * (we need to reset the facility to set STATUS for an erronous case and
		 * have to wait 250msec) and, if this is not an erronous case, the facility
		 * was already initialized at the system booting.
		 */
		case MUIC_MODE_NA_TA:
		case MUIC_MODE_TA_1A:
		case MUIC_MODE_INVALID_CHG :
			if ((int_stat_value & VBUS) == 0) {		
				// Exit Charger Mode
				status->mode = MUIC_MODE_NONE;
			}
			else {		
				max14526_set_mode(status, int_stat_value);           
			}

			break;

		case MUIC_MODE_LG_TA :
			printk(KERN_WARNING "[chahee.kim] LG_TA");
			if ((int_stat_value & CHGDET) == 0) {		
				status->mode = MUIC_MODE_NONE;				
			}
			else {		
				max14526_set_mode(status, int_stat_value);           
			}
			break;
		case MUIC_MODE_AP_UART :
			if ((int_stat_value & IDNO) == IDNO_1011) {
				/* Exit Factory Mode */
				status->mode = MUIC_MODE_NONE;
			}
			else {
				max14526_set_mode(status, int_stat_value);           
			}
			break;	
		case MUIC_MODE_AP_USB :
			if ((int_stat_value & VBUS) == 0) {
				/* USB Host Removed */
				status->mode = MUIC_MODE_NONE;		
			}
			else {		
				max14526_set_mode(status, int_stat_value);
			}
			break;

		default:
			printk(KERN_WARNING "[chahee.kim] Failed to detect an accessory. Try again!\n");
			status->mode = MUIC_MODE_UNKNOWN;
			status->pdata->enable_charger(status->mode);
			ret = -1;
			break;
	}	


	printk(KERN_WARNING "[chahee.kim] muic_max14526_detect_accessory, "
			"muic_mode = %d \n", status->mode);

	return ret;
}

static void max14526_wq_func(struct work_struct *work)
{
        struct max14526_status *status;
	int ret = 0;

	status = container_of(work, struct max14526_status, detect_work.work);

#if 0
	if (retain_mode != NO_RETAIN) {
		dev_err(&status->client->dev, "Retain mode\n");
		return;
	}
#endif

	ret = max14526_detect_accessory(status);

	printk(KERN_INFO "[chahee.kim] muic_detect_accessory(UPON_IRQ)detedted!!!!\n");
}

static irqreturn_t max14526_irq_handler(int irq, void *data)
{
	struct max14526_status *status = data;

	if (!data) {
		printk(KERN_CRIT "max14526 object is null in %s\n", __func__);
		return IRQ_NONE;
	}

	status = data;

	schedule_delayed_work(&status->detect_work, msecs_to_jiffies(250));

	return IRQ_HANDLED;
}

static int __devinit max14526_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max14526_status *status;
	int ret = 0;
	
	printk(KERN_DEBUG "%s enter!!\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		return -EIO;
	}

	status = kzalloc(sizeof(*status), GFP_KERNEL);
	if (!status) {
		dev_err(&client->dev, "%s failed.\n", __func__);
		return -ENOMEM;
	}

	status->client		= client;
	status->pdata 		= client->dev.platform_data;
	status->mode		= MUIC_MODE_NONE;
	status->is_charing	= false;
	status->is_boottime	= true;

	INIT_DELAYED_WORK(&status->detect_work, max14526_wq_func);

	i2c_set_clientdata(client, status);

	/* gpio init.. */
	if (status->pdata) {
		if(status->pdata->init_gpio) {
			if (status->pdata->init_gpio() < 0) {
				pr_err("%s gpio_init failed\n", __func__);
			}
		}
	}

	ret = request_irq(client->irq, max14526_irq_handler, 
			  IRQF_TRIGGER_FALLING, client->name, 
			  status);
	if (ret < 0) {
		dev_err(&client->dev, "request_irq failed!\n");
		free_irq(client->irq, &client->dev);
		return -ENOSYS;
	}

	/* Prepares a human accessible method to control MUIC */
	//create_muic_proc_file();

	/* Selects one of the possible muic chips */
	//mdelay(70);
	//muic_detect_device();

#if 0
	if (retain_mode == BOOT_AP_USB) {
		muic_mode = MUIC_MODE_AP_USB;
		printk("[chahee.kim] muic_init_device... retain mode = AP_USB\n");
	} 
	else {
#endif
		max14526_init(status);
		max14526_detect_accessory(status);
		status->is_boottime = false;
#if 0
	}
#endif

	/* Makes the interrupt on MUIC INT wake up OMAP which is in suspend mode */
	ret = enable_irq_wake(status->pdata->gpio);
	if (ret < 0) {
		dev_err(&client->dev, "enable_irq_wake failed\n");
		disable_irq_wake(status->pdata->gpio);
		return -ENOSYS;
	}

	dev_info(&client->dev, "%s success!!\n", __func__);

	return ret;
}

static int __devexit max14526_remove(struct i2c_client *client)
{
	struct max14526_status *status = i2c_get_clientdata(client);
	
	free_irq(client->irq, &client->dev);

	status->pdata->deinit_gpio();

	cancel_delayed_work(&status->detect_work);
#if 0
	remove_muic_proc_file();
#endif
	
	return 0;
}

static const struct i2c_device_id max14526_ids[] = {
	{"muic", 0},
	{/* end of list */},
};

#if defined(CONFIG_PM)
static int max14526_suspend(struct i2c_client *client, pm_message_t state)
{
	struct max14526_status *status= i2c_get_clientdata(client);
	client->dev.power.power_state = state;

	max14526_i2c_write_byte(status, CONTROL_2, USB_DET_DIS);
	max14526_i2c_write_byte(status, CONTROL_1, 0x00);
	max14526_i2c_write_byte(status, SW_CONTROL, OPEN);

	cancel_delayed_work(&status->detect_work);

	printk(KERN_INFO "[chahee.kim] MUIC : Suspend \n");

	return 0;
}
		
static int max14526_resume(struct i2c_client *client)
{
	struct max14526_status *status= i2c_get_clientdata(client);
	client->dev.power.power_state = PMSG_ON;

	max14526_init(status);

	printk(KERN_INFO "[chahee.kim] MUIC : Resume \n");
	return 0;
}
#else	// CONFIG_PM
#define max14526_suspend	NULL
#define max14526_resume		NULL
#endif	// CONFIG_PM


static int __init muic_state(char *str)
{    
	//int initial_path = simple_strtol(str, NULL, 0);

#if 0
	if (initial_path == 2)
		retain_mode = BOOT_AP_USB;
	else
		printk("[MUIC] no retain\n");

	printk("[MUIC] muic_state = %d\n", retain_mode);    
#endif
	return 1;
}
__setup("muic_state=", muic_state);



static struct i2c_driver max14526_driver = {
	.probe	 	= max14526_probe,
	.remove	 	= __devexit_p(max14526_remove),
	.resume		= max14526_resume,
	.suspend	= max14526_suspend,
	.id_table	= max14526_ids,
	.driver	 	= {
		.name   = "muic",
		.owner  = THIS_MODULE,
	},
};

static int __init max14526_muic_init(void)
{
	return i2c_add_driver(&max14526_driver);
}

static void __exit max14526_muic_exit(void)
{
	i2c_del_driver(&max14526_driver);
}

module_init(max14526_muic_init);
module_exit(max14526_muic_exit);

MODULE_AUTHOR("Sookyoung Kim <sookyoung.kim@lge.com>");
MODULE_AUTHOR("Yoolje Cho <yoolje.cho@lge.com>");
MODULE_DESCRIPTION("MAX14562 MUIC Driver");
MODULE_LICENSE("GPL");

