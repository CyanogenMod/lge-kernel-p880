/*
 * X3 MUIC driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
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
#include <linux/interrupt.h>	/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <asm/system.h>
#include "../gpio-names.h"

/*
 * kernel/arch/arm/include/asm/gpio.h includes kernel/arch/arm/plat-omap/include/mach/gpio.h which,
 * in turn, includes kernel/include/asm-generic/gpio.h.
 * <mach/gpio.h> declares gpio_get|set_value(), gpio_to_irq().
 * <asm-generic/gpio.h> declares struct gpio_chip, gpio_request(), gpio_free(), gpio_direction_input|output().
 * The actual descriptions are in kernel/drivers/gpio/gpiolib.c and kernel/arch/arm/plat-omap/gpio.c.
 */
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>		/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>
#include <linux/mutex.h>

#include <linux/notifier.h>
#include <linux/reboot.h>

//                                           
#include <linux/power_supply.h>

#include "muic.h"

//                                             
#include "../../../arch/arm/mach-tegra/lge/x3/include/lge/board-x3-nv.h"

//jude84.kim
#include "../../../arch/arm/mach-tegra/board.h"
#include <mach/clk.h>

const char *retain_mode_str[] = {
	"RETAIN_NO",
	"RETAIN_AP_USB",
	"RETAIN_CP_USB",
	"RETAIN_AP_UART",
	"RETAIN_CP_UART",
};

#define MUIC_GPIO	TEGRA_GPIO_PJ0 //TEGRA_GPIO_PX6

static DEFINE_SPINLOCK(muic_spin_lock);
bool muic_irq_already_run;

static struct i2c_client *muic_client;
struct workqueue_struct *muic_wq;
static struct work_struct muic_work;

TYPE_USIF_MODE usif_mode = USIF_AP;
TYPE_DP3T_MODE dp3t_mode = DP3T_NC;
TYPE_MUIC_MODE muic_mode = MUIC_UNKNOWN;
TYPE_CHARGING_MODE charging_mode = CHARGING_NONE;
TYPE_UPON_IRQ  upon_irq  = NOT_UPON_IRQ;

static TYPE_RETAIN_MODE retain_mode = RETAIN_NO;
static TYPE_RETAIN_MODE boot_retain_mode = RETAIN_NO;


const char *muic_mode_str[] = {
	"MUIC_UNKNOWN",			// 0
	"MUIC_NONE",   			// 1
	"MUIC_NA_TA",   		// 2
	"MUIC_LG_TA",   		// 3
	"MUIC_TA_1A", 	  		// 4
	"MUIC_INVALID_CHG",  	// 5
	"MUIC_AP_UART",   		// 6
	"MUIC_CP_UART",			// 7
	"MUIC_AP_USB", 			// 8
	"MUIC_CP_USB",			// 9
	"MUIC_TV_OUT_NO_LOAD",	// 10
	"MUIC_EARMIC",			// 11
	"MUIC_TV_OUT_LOAD",		// 12
	"MUIC_OTG",   			// 13
	"MUIC_MHL",				// 14
	"MUIC_RESERVE1",		// 15
	"MUIC_RESERVE2",		// 16
};



const char *charging_mode_str[] = {
	"CHARGING_UNKNOWN",
	"CHARGING_NONE",
	"CHARGING_NA_TA",
	"CHARGING_LG_TA",
	"CHARGING_TA_1A",
	"CHARGING_INVALID_CHG",
	"CHARGING_USB",
	"CHARGING_FACTORY",
	"CHARGING_MHL",		//                                                      

};


#if defined (MUIC_SLEEP)
struct wake_lock muic_wake_lock;
static char wake_lock_enable = 1;
#endif//


void (*muic_init_device)(TYPE_RESET) = muic_init_unknown;
s32 (*muic_detect_accessory)(s32) = muic_unknown_detect_accessory;

extern void muic_init_ts5usba33402(TYPE_RESET reset);
extern void muic_init_max14526(TYPE_RESET reset);

extern s32 muic_ts5usba33402_detect_accessory(s32 upon_irq);
extern s32 muic_max14526_detect_accessory(s32 upon_irq);


static s32 muic_proc_set_ap_uart(void);
static s32 muic_proc_set_cp_uart(void);
static s32 muic_proc_set_ap_usb(void);
static s32 muic_proc_set_cp_usb(void);

#ifdef CONFIG_PROC_FS
/*
 * --------------------------------------------------------------------
 *  BEGINS: Proc file system related functions for MUIC.
 * --------------------------------------------------------------------
 */
#define	LG_MUIC_PROC_FILE "driver/cmuic"

static struct proc_dir_entry *lg_muic_proc_file;

#ifndef CP_RESET_TEST
static int muic_cp_request(void)
{
	s32 ret = 0;
		
	ret = gpio_request(TEGRA_GPIO_PV1, "TEGRA_GPIO_PR5 switch control 1 GPIO");
	if (ret < 0) {
		printk( "[MUIC] GPIO 11 TEGRA_GPIO_PR5 is already occupied by other driver!\n");
		 return -ENOSYS;
	}

	ret = gpio_direction_output(TEGRA_GPIO_PV1, 0);
	if (ret < 0) {
		printk( "[MUIC] gpio_11 TEGRA_GPIO_PR5 direction initialization failed!\n");
		 return -ENOSYS;
	}
	tegra_gpio_enable(TEGRA_GPIO_PV1);


	ret = gpio_request(TEGRA_GPIO_PO0, "TEGRA_GPIO_PR5 switch control 1 GPIO");
	if (ret < 0) {
		printk( "[MUIC] GPIO 11 TEGRA_GPIO_PR5 is already occupied by other driver!\n");
		 return -ENOSYS;
	}

	ret = gpio_direction_output(TEGRA_GPIO_PO0, 0);
	if (ret < 0) {
		printk( "[MUIC] gpio_11 TEGRA_GPIO_PR5 direction initialization failed!\n");
		 return -ENOSYS;
	}
	tegra_gpio_enable(TEGRA_GPIO_PO0);
	
	return 0;
}


static void muic_cp_reset(void)
{
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);
	
	gpio_set_value(TEGRA_GPIO_PO0, 0);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PV1, 0);
	mdelay(2000);

	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	//gpio_set_value(TEGRA_GPIO_PO0, 0);

	printk(KERN_INFO "[MUIC] muic_cp_reset: TEGRA_GPIO_PO0/PV1 ");
		return;

}

//                                            
void fota_cp_reset(void)
{
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);
	
	gpio_set_value(TEGRA_GPIO_PO0, 0);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PV1, 0);
	mdelay(2000);

	gpio_set_value(TEGRA_GPIO_PV1, 1);
	mdelay(50);
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	mdelay(100);
	//gpio_set_value(TEGRA_GPIO_PO0, 0);

	printk(KERN_INFO "[FOTA] fota_cp_reset: TEGRA_GPIO_PO0/PV1 ");
		return;

}
//                                            


#endif//

#if defined(CONFIG_MHL_TX_SII9244)	//                                               
void muic_set_mhl_mode_detect(void) 
{ 

	printk("[MUIC] muic_set_mhl_mode_detect entry. \n"); 

	/* Connect CP UART signals to AP */ 
	usif_switch_ctrl(USIF_AP); 
	/* 109 		 * AP USB does not pass through DP3T. 110		 
	* Just connect AP UART to MUIC UART.  111		  */

	dp3t_switch_ctrl(DP3T_NC);

	//muic_i2c_write_byte(SW_CONTROL, COMP2_TO_DP2 | COMN1_TO_DN1);  //Path USB
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); //Path OPEN


	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */ 
	//muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);   

	MHL_On(1); 

	//                                                                         


}
#endif//


static ssize_t muic_proc_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
	s32 i;
	u32 val;

	for (i = 0; i <= 5; i++) {
		val = i2c_smbus_read_byte_data(muic_client, (u8)i);
		printk(KERN_INFO "[MUIC] reg 0x%02x, val = 0x%02x\n", i, val);
	}
	
	len = sprintf(buf, "%d\n", muic_mode);
	printk(KERN_INFO "[MUIC] mode = %s, retain = %s\n", muic_mode_str[muic_mode], retain_mode_str[retain_mode]);
	
	return len;
}

#if defined (MUIC_SLEEP)
static void muic_wakeup_lock(void)
{
#if 1
	if(wake_lock_enable)
	{
		switch(charging_mode)
		{
			default:
			case CHARGING_UNKNOWN:
			case CHARGING_NONE:
			case CHARGING_INVALID_CHG:
				wake_unlock(&muic_wake_lock);
				printk(KERN_INFO "[MUIC] wake_unlock(): charging_mode = %s (%d)\n" , charging_mode_str[charging_mode], charging_mode);
				break;
			case CHARGING_NA_TA:
#if !defined (CONFIG_LGE_TA_UNWAKELOCK)
			case CHARGING_LG_TA:
			case CHARGING_TA_1A:
#endif
			case CHARGING_USB:
			case CHARGING_FACTORY:
			case CHARGING_MHL:		//                                                      
				wake_lock(&muic_wake_lock);
				printk(KERN_INFO "[MUIC] wake_lock(): charging_mode = %s (%d)\n" , charging_mode_str[charging_mode], charging_mode);
				break;
		}
	}
	else
	{
		wake_unlock(&muic_wake_lock);
		printk(KERN_INFO "[MUIC] wake_unlock(): always unlock mode\n");
	}
#else
	if( wake_lock_enable && (muic_mode == MUIC_UNKNOWN || muic_mode == MUIC_NONE) )
	{
		wake_unlock  (&muic_wake_lock);
		printk(KERN_INFO "[MUIC] wake_unlock(): muic_mode = %s (%d)\n" , muic_mode_str[muic_mode], muic_mode);
	}else{
		wake_lock(&muic_wake_lock);
		printk(KERN_INFO "[MUIC] wake_lock(): muic_mode = %s (%d)\n" , muic_mode_str[muic_mode], muic_mode);
	}
#endif//
}
#endif//


static ssize_t muic_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	char cmd;
	char nv_msg[1];

	len = sscanf(buf, "%c", &cmd/*, &dummy*/);	//                                       
	printk(KERN_INFO "[MUIC] LGE: MUIC_proc_write, cmd = %c , buf = %s len = %d\n", cmd, buf, len);

	switch (cmd) {

	/* AP_UART mode*/
	case '6' :
		retain_mode = RETAIN_AP_UART;
		muic_proc_set_ap_uart();
		break;
	
	/* CP_UART mode*/
	case '7' :
		retain_mode = RETAIN_CP_UART;
		muic_proc_set_cp_uart();
		break;
			
	/* AP_USB mode*/
	case '8' :
		retain_mode = RETAIN_AP_USB;
		muic_proc_set_ap_usb();
		break;

	/* CP_USB mode*/
	case '9' :
		retain_mode = RETAIN_CP_USB;
		muic_proc_set_cp_usb();
		mdelay(20);
		break;

#ifndef CP_RESET_TEST
	/* CP_DOWNLOAD mode*/
	case '0' :
		retain_mode = RETAIN_CP_USB;
	case 'c' :
		muic_proc_set_cp_usb();
		mdelay(1000);
		muic_cp_reset();
		//mdelay(400);
	break;
	case 'q' :
		muic_cp_request();
		break;
	case 'r' :
		muic_cp_reset();
		break;
#endif//		

	/* Rebooting should be performed by HiddenMenu Application. */
	case '1':
		retain_mode = RETAIN_NO;
		/* NO retain mode after reboot */
		nv_msg[0] = RETAIN_NO;
		lge_nvdata_write(LGE_NVDATA_MUIC_RETENTION_OFFSET, nv_msg, 1);
		break;

	case 'a' :
		/* AP USB retain mode after reboot */
		nv_msg[0] = RETAIN_AP_USB;
		lge_nvdata_write(LGE_NVDATA_MUIC_RETENTION_OFFSET, nv_msg, 1);
		break;
		
	case 'b' :
		/* CP USB retain mode after reboot */
		nv_msg[0] = RETAIN_CP_USB;
		lge_nvdata_write(LGE_NVDATA_MUIC_RETENTION_OFFSET, nv_msg, 1);
		break;

#if defined (MUIC_SLEEP)
	case 'u':
		/* Just unlock the wakelock ! */
		wake_lock_enable = 0;
		muic_wakeup_lock();
		break;
	case 'l':
		/* Just lock the wakelock ! */
		wake_lock_enable = 1;
		muic_wakeup_lock();
		break;
#endif
		
	default : 
			printk(KERN_INFO "[MUIC] LGE: ap30 MUIC invalid command: 6=AP_UART, 7=CP_UART, 8=AP_USB, 9=CP_USB\n");
			break;
	   }

	check_charging_mode();	//check the charging mode, but NOT send to charger !!!

	return len;
}

static struct file_operations lg_muic_proc_ops = {
	.read = muic_proc_read,
	.write = muic_proc_write,
};

static void create_lg_muic_proc_file(void)
{
	lg_muic_proc_file = create_proc_entry(LG_MUIC_PROC_FILE, 0777, NULL);
	if (lg_muic_proc_file) {
#if 0 // kernel 32
		lg_muic_proc_file->owner = THIS_MODULE;
#endif
		lg_muic_proc_file->proc_fops = &lg_muic_proc_ops;
	} else
		printk(KERN_INFO "[MUIC] LGE: X3-AP30 MUIC proc file create failed!\n");
}

static void remove_lg_muic_proc_file(void)
{
	remove_proc_entry(LG_MUIC_PROC_FILE, NULL);
}

/*
 * --------------------------------------------------------------------
 *  ENDS: Proc file system related functions for MUIC.
 * --------------------------------------------------------------------
 */
#endif //CONFIG_PROC_FS

static bool check_cable_ID = false; 	//                                                                         

static bool cable_id_check(s32 value)
{
	if (value & V_VBUS) 
	{
		if ((value & IDNO) == IDNO_0010 || 
			(value & IDNO) == IDNO_0100 ||
			(value & IDNO) == IDNO_1010)
			 return true;
	}
	return false;
}

bool get_cable_type(void)	//                                                                          
{
	return check_cable_ID;
}
EXPORT_SYMBOL(get_cable_type);

void check_charging_mode(void)
{
	s32 value;

	value = i2c_smbus_read_byte_data(muic_client, INT_STAT);
	if (value & V_VBUS) {
		if ((value & IDNO) == IDNO_0010 || 
			(value & IDNO) == IDNO_0100 ||
			(value & IDNO) == IDNO_1001 ||
			(value & IDNO) == IDNO_1010 ||
			(boot_retain_mode == RETAIN_AP_USB && retain_mode == RETAIN_AP_USB))	//                                                                                                
			charging_mode = CHARGING_FACTORY;
		else if (value & CHGDET) 
			charging_mode = CHARGING_LG_TA;
		else
			charging_mode = CHARGING_USB;
	} else
		charging_mode = CHARGING_NONE;

	check_cable_ID = cable_id_check(value);

	//set_muic_charger_detected();	//not done here. Moved to other functions
}
EXPORT_SYMBOL(check_charging_mode);

/* 
 * Linux bug: If a constant value larger than 20000,
 * compiler issues a __bad_udelay error.
 */
void muic_mdelay(u32 microsec)
{
	do {
		udelay(1000);
	} while (microsec--);
}
EXPORT_SYMBOL(muic_mdelay);

#if 0 //                                   
//                                                
TYPE_MUIC_MODE get_muic_mode(void)
{
	return charging_mode;
}
EXPORT_SYMBOL(get_muic_mode);
#endif

#if 0	//                                                                    
//                                             
int muic_send_cable_type(TYPE_MUIC_MODE mode)
{
	// should use platform data..
	struct power_supply *psy=NULL;
	union power_supply_propval value;
	int ret;

	printk("%s : mode = (%d)\n",  __func__, mode);

	switch (mode) {
		case MUIC_AP_USB:
		case MUIC_MHL:
			psy = power_supply_get_by_name("usb");
			value.intval = POWER_SUPPLY_TYPE_USB;
			break;
		case MUIC_NA_TA:
		case MUIC_LG_TA:
		case MUIC_TA_1A:
			psy = power_supply_get_by_name("ac");
			value.intval = POWER_SUPPLY_TYPE_MAINS;
			break;
		case MUIC_CP_UART:
		case MUIC_CP_USB:
		case MUIC_AP_UART:
			psy = power_supply_get_by_name("factory");
			value.intval = POWER_SUPPLY_TYPE_FACTORY;
			break;
		case MUIC_NONE:			
			psy = power_supply_get_by_name("battery");
			value.intval = POWER_SUPPLY_TYPE_BATTERY;
			break;
		default:
			value.intval = POWER_SUPPLY_TYPE_BATTERY;
			break;
	}

	if (!psy) {
		printk("%s: fail to get battery/charger psy\n", __func__);
		return -ENODEV;
	}

	switch(value.intval){
		case POWER_SUPPLY_TYPE_USB:
		case POWER_SUPPLY_TYPE_MAINS:
		case POWER_SUPPLY_TYPE_FACTORY:
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
			break;
		case POWER_SUPPLY_TYPE_BATTERY:
		default:
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
			break;
	}
	return ret;
}
EXPORT_SYMBOL(muic_send_cable_type);
//                                             
#endif

unsigned char chg_flag_muic = 0; //                                                              
int muic_send_charger_type(TYPE_CHARGING_MODE mode)
{
	// should use platform data..
	struct power_supply *psy=NULL;
	union power_supply_propval value;
	int ret;

	printk("[MUIC] %s: charging_mode = %s (%d)\n",  __func__, charging_mode_str[mode], mode);

	switch (mode) {
		case CHARGING_USB:
		case CHARGING_MHL:		//                                                      
			psy = power_supply_get_by_name("usb");
			value.intval = POWER_SUPPLY_TYPE_USB;
			chg_flag_muic = 1;		//                                                                                              
			break;

		case CHARGING_NA_TA:
		case CHARGING_LG_TA:
		case CHARGING_TA_1A:
			psy = power_supply_get_by_name("ac");
			value.intval = POWER_SUPPLY_TYPE_MAINS;
			chg_flag_muic = 1;		//                                                                                              
			break;

		case CHARGING_FACTORY:
			psy = power_supply_get_by_name("factory");
			value.intval = POWER_SUPPLY_TYPE_FACTORY;
			break;

		case CHARGING_NONE:
			psy = power_supply_get_by_name("battery");
			value.intval = POWER_SUPPLY_TYPE_BATTERY;
			break;

		default:
			value.intval = POWER_SUPPLY_TYPE_BATTERY;
			break;
	}

	if (!psy) {
		printk("[MUIC] %s: fail to get battery/charger psy\n", __func__);
		return -ENODEV;
	}

	switch(value.intval){
		case POWER_SUPPLY_TYPE_USB:
		case POWER_SUPPLY_TYPE_MAINS:
		case POWER_SUPPLY_TYPE_FACTORY:
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
			break;
		case POWER_SUPPLY_TYPE_BATTERY:
		default:
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
			break;
	}
	return ret;
}
EXPORT_SYMBOL(muic_send_charger_type);


//                                                          
void set_muic_charger_detected(void)
{
#if 0 // this is for wait event which is not unsing just now. block it temporary..
	//extern wait_queue_head_t muic_event;

	printk(KERN_WARNING "[MUIC] LGE: set_muic_charger_detected\n");

	//atomic_set(&muic_charger_detected,1);
	//wake_up_interruptible(&muic_event);
	muic_chager_event = 1;
	//wake_up(&muic_event);
#endif
	//                                                                      
#if defined (MUIC_SLEEP)
	muic_wakeup_lock();
#endif//
	muic_send_charger_type(charging_mode);	//                               
}
EXPORT_SYMBOL(set_muic_charger_detected);




/*
 * Function: Read the MUIC register whose internal address is addr
 * 			and save the u8 content into value.
 * Return value: Negative errno upon failure, 0 upon success.
 */
s32 muic_i2c_read_byte(u8 addr, u8 *value)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(muic_client, (u8)addr);
	if (ret < 0) {
		printk("[MUIC] muic_i2c_read_byte failed.\n");
		return ret;
	} else {
		*value = (u8)ret;
		return 0;
	}
}
EXPORT_SYMBOL(muic_i2c_read_byte);


/*
 * Function: Write u8 value to the MUIC register whose internal address is addr.
 * Return value: Negative errno upon failure, 0 upon success.
 */
s32 muic_i2c_write_byte(u8 addr, u8 value)
{
	s32 ret;
	ret = i2c_smbus_write_byte_data(muic_client, (u8)addr, (u8)value);
	if (ret < 0)
		printk(KERN_INFO "[MUIC] muic_i2c_write_byte failed.\n");
	return ret;
}
EXPORT_SYMBOL(muic_i2c_write_byte);


void usif_switch_ctrl(TYPE_USIF_MODE mode)
{
#if defined (CONFIG_LGE_MUIC_DP3T)
	if (mode == USIF_AP) {
		gpio_set_value(USIF_IN_1_GPIO, 0);
		printk(KERN_INFO "[MUIC] usif_switch_ctrl, CP UART is connected to AP\n");
	} else if (mode == USIF_DP3T) {
		gpio_set_value(USIF_IN_1_GPIO, 1);
		printk(KERN_INFO "[MUIC] usif_switch_ctrl, CP UART is connected to DP3T (then, MUIC)\n");
	} else {
		/* Just keep the current path */
	}

	usif_mode = mode;
     printk(KERN_INFO "[MUIC] usif_switch_ctrl(): usif_mode = %d\n", usif_mode);
#else
	printk(KERN_INFO "[MUIC] usif_switch_ctrl(): usif_mode   Not suport !!!  \n" );
#endif//	 
}
EXPORT_SYMBOL(usif_switch_ctrl);


void dp3t_switch_ctrl(TYPE_DP3T_MODE mode)
{
	//DP3T_IN_2_GPIO is no more used in X3 because there is NO AP_UART path and using DP2T.
#if defined (CONFIG_LGE_MUIC_DP3T)

	if (mode == DP3T_AP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, AP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, CP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_USB) {
		int gpio_val=0, old_val = 0;
		old_val= gpio_get_value(IFX_USB_VBUS_EN_GPIO);
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 1);
		gpio_set_value(DP3T_IN_1_GPIO, 1);

		gpio_val = gpio_get_value(IFX_USB_VBUS_EN_GPIO);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, CP USB is connected to MUIC UART: ifx_usb_vbus = %d->%d\n",old_val ,gpio_val);
	} else if (mode == DP3T_NC) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, None is connected to MUIC UART\n");
	} else {
		/* Just keep the current path */
	}
	
	dp3t_mode = mode;
	printk(KERN_INFO "[MUIC] dp3t_switch_ctrl(): dp3t_mode = %d\n", dp3t_mode);
#else
	printk(KERN_INFO "[MUIC] dp3t_switch_ctrl(): dp3t Not suport !!! \n" );
#endif//	 
	
}
EXPORT_SYMBOL(dp3t_switch_ctrl);


static s32 muic_proc_set_ap_uart(void)
{
	s32 ret;

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Connect DP, DM to UART_TX, UART_RX */
	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);

	muic_mode = MUIC_AP_UART;	
	charging_mode = CHARGING_UNKNOWN;

	printk(KERN_INFO "[MUIC] muic_proc_set_ap_uart(): AP_UART\n");

	return ret;
}


static s32 muic_proc_set_ap_usb(void)
{
	s32 ret;

	ret = muic_i2c_write_byte(SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* AP USB does not pass through DP3T.
	 * Just connect AP UART to MUIC UART.
	 */
	dp3t_switch_ctrl(DP3T_AP_UART); //                                                                                                   

	/* Connect DP, DM to USB_DP, USB_DM */
	ret = muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);

	muic_mode = MUIC_AP_USB;	
	charging_mode = CHARGING_USB;
	printk(KERN_INFO "[MUIC] muic_proc_set_ap_usb(): AP_USB\n");

	return ret;
}


static s32 muic_proc_set_cp_uart(void)
{
	s32 ret;

	/* Connect CP UART signals to DP3T */
	usif_switch_ctrl(USIF_DP3T);

	/* Connect CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);

	/* Connect DP, DM to UART_TX, UART_RX */
	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);

	muic_mode = MUIC_CP_UART;	
	charging_mode = CHARGING_UNKNOWN;

	printk(KERN_INFO "[MUIC] muic_proc_set_cp_uart(): CP_UART\n");

	return ret;
}

static s32 muic_proc_set_cp_usb(void)
{
	s32 ret;

	ret = muic_i2c_write_byte(SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);//                                                                          

	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */		

	//                                                                                                                             
	//	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN); //15
    //	muic_i2c_write_byte(CONTROL_1,  ID_2P2 | SEMREN );//44


	/* Enable USB Path (0x03=0x00) --- Connect DP, DM to UART_TX, UART_RX */	
	ret = muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);


	muic_mode = MUIC_CP_USB;		
	charging_mode = CHARGING_USB;
	printk(KERN_ERR "[MUIC] muic_proc_set_cp_usb(): CP_USB\n");

	return ret;
}

void muic_proc_set_cp_usb_force(void)
{
	//regardless of wakelock, ONLY muic path switch !
	retain_mode = RETAIN_CP_USB;
	muic_proc_set_cp_usb();
}

void muic_init_unknown(TYPE_RESET x)
{
	printk("[MUIC] muic_init_unknown: You should add codes for new MUIC chip");
	return;
}
s32 muic_unknown_detect_accessory(s32 x)
{
	printk("[MUIC] muic_unknown_detect_accessory: You should add codes for new MUIC chip");
	return -1;
}


static void muic_detect_device(void)
{
	s32 ret;
	u8 muic_device;

	printk(KERN_INFO "[MUIC] muic_detect_device()\n");

	ret = muic_i2c_read_byte(DEVICE_ID, &muic_device);
	if ((muic_device & 0xf0) == TS5USBA33402)
		muic_device = TS5USBA33402;
	else if ((muic_device & 0xf0) == MAX14526)
		muic_device = MAX14526;
	else if ((muic_device & 0xf0) == ANY_VENDOR)
		muic_device = ANY_VENDOR;

	if (muic_device == TS5USBA33402) {
		muic_init_device = muic_init_ts5usba33402;
		muic_detect_accessory = muic_ts5usba33402_detect_accessory;
		printk(KERN_INFO "[MUIC] muic chip: TS5USBA33402\n");
	} else if (muic_device == MAX14526) {
		muic_init_device = muic_init_max14526;
		muic_detect_accessory = muic_max14526_detect_accessory;
		printk(KERN_INFO "[MUIC] muic chip: MAX14526\n");
	} else {
		muic_init_device = muic_init_unknown;
		muic_detect_accessory = muic_unknown_detect_accessory;
		printk(KERN_INFO "[MUIC] muic chip: ANY VENDOR\n");
	}
}


static void muic_work_inside_operation (void) {
    	
	s32 ret = 0;

	printk(KERN_INFO "[MUIC] muic_work_func(): retain_mode = %s (%d) \n", retain_mode_str[retain_mode], retain_mode);

	if (retain_mode == RETAIN_NO) {
		ret = muic_detect_accessory(UPON_IRQ);
		set_muic_charger_detected();	//                      
		printk(KERN_INFO "[MUIC] muic_detect_accessory(UPON_IRQ) result:  muic_mode = %s (%d), charing = %s (%d)\n", 
		   	   muic_mode_str[muic_mode], muic_mode, charging_mode_str[charging_mode], charging_mode);
	}
	else 
	{
		muic_mdelay(250);

		ret = i2c_smbus_read_byte_data(muic_client, INT_STAT);
		if (muic_mode == MUIC_CP_USB)
			muic_proc_set_cp_usb();
		check_charging_mode();

		set_muic_charger_detected();	
		printk(KERN_INFO "[MUIC] Now...path retain mode, int_stat=0x%02X,  muic_mode = %s (%d), charing = %s (%d)\n", 
		   	   ret, muic_mode_str[muic_mode], muic_mode, charging_mode_str[charging_mode], charging_mode);
	}
}

static void muic_work_func(struct work_struct *muic_work)
{
    muic_work_inside_operation();
}

static irqreturn_t muic_interrupt_handler(s32 irq, void *data)
{
	unsigned long flags;
		
	/* Make the interrupt on MUIC INT wake up OMAP which is in suspend mode */
//	spin_lock_irqsave(&muic_spin_lock, flags);
	queue_work(muic_wq, &muic_work);
	muic_irq_already_run = true;
//	spin_unlock_irqrestore(&muic_spin_lock, flags);
	return IRQ_HANDLED;
}


static int muic_reboot_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
    switch (event) {
    case SYS_RESTART:
    case SYS_HALT:
    case SYS_POWER_OFF:
		{	
			printk("%s \n",__func__);
			muic_i2c_write_byte(CONTROL_1, 0x00);
		}
	    return NOTIFY_OK;
    }
    return NOTIFY_DONE;
}


static struct notifier_block muic_reboot_nb = {
	.notifier_call = muic_reboot_notify,
};



/*
 * muic_probe() is called in the middle of booting sequence due to '__init'.
 * '__init' causes muic_probe() to be released after the booting.
 */
static s32 __devinit muic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = 0;

	muic_client = client;

	printk("[MUIC] LG muic_probe() Begin\n");
#if defined (CONFIG_LGE_MUIC_DP3T)

	/* 
	 * Initializes gpio_165 (USIF1_SW).
	 * Checks if other driver already occupied it.
	 */
	ret = gpio_request(USIF_IN_1_GPIO, "USIF switch control GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] GPIO 165 USIF1_SW is already occupied by other driver!\n");
		/*
		 * We know board_cosmo.c:ifx_n721_configure_gpio() performs a gpio_request on this pin first.
		 * Because the prior gpio_request is also for the analog switch control, this is not a confliction.
		 */
		return -ENOSYS;
	}

	ret = gpio_direction_output(USIF_IN_1_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_16 USIF_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(USIF_IN_1_GPIO);

	/*
	 * Initializes gpio_11 (OMAP_UART_SW) and gpio_12 (IFX_UART_SW).
	 * Checks if other driver already occupied them.
	 */
	ret = gpio_request(DP3T_IN_1_GPIO, "DP3T switch control 1 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] GPIO 11 DP3T_IN_1_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(DP3T_IN_1_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_11 DP3T_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(DP3T_IN_1_GPIO);

	ret = gpio_request(IFX_USB_VBUS_EN_GPIO, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(IFX_USB_VBUS_EN_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}
	tegra_gpio_enable(IFX_USB_VBUS_EN_GPIO);
#endif//

	ret = gpio_request(MUIC_GPIO, "pj0");
	if (ret < 0)
	{
		printk(KERN_INFO "[MUIC] MUIC_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret=gpio_direction_input(MUIC_GPIO);
	if (ret < 0)
	{
		printk(KERN_INFO "[MUIC] MUIC_GPIO direction initialization failed!\n");
		gpio_free(MUIC_GPIO);
		return -ENOSYS;
	}
	else
		tegra_gpio_enable(MUIC_GPIO);

#if defined (MUIC_SLEEP)
	wake_lock_init(&muic_wake_lock, WAKE_LOCK_SUSPEND, "muic_lock");
#endif//

	/* Registers MUIC work queue function */
	INIT_WORK(&muic_work, muic_work_func);
	/* 
	 * Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls queue_work() with muic_work_func().
	 * muic_work_func() actually performs the accessory detection.
	 */
	ret = request_irq(client->irq, muic_interrupt_handler, IRQF_TRIGGER_FALLING, client->name, &client->dev);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] MUIC_GPIO IRQ line set up failed!\n");
		free_irq(client->irq, &client->dev);
		return -ENOSYS;
	}

	//disable_irq_wake(MUIC_GPIO);


	/* Prepares a human accessible method to control MUIC */
	create_lg_muic_proc_file();

	/* Selects one of the possible muic chips */
	muic_detect_device();

	/* Initializes MUIC - Finally MUIC INT becomes enabled */
	if (retain_mode == RETAIN_AP_USB) {
		muic_proc_set_ap_usb();	//                                            
		muic_mode = MUIC_AP_USB;
		muic_init_device(DEFAULT);
		check_charging_mode();
		printk("[MUIC] muic_init_device... retain mode = AP_USB\n");
	} else if (retain_mode == RETAIN_CP_USB) {
		muic_proc_set_cp_usb();
		muic_mode = MUIC_CP_USB;
		muic_init_device(DEFAULT);
		check_charging_mode();
		printk("[MUIC] muic_detect_accessory... retain mode = CP_USB\n");
	} else {

		muic_init_device(DEFAULT);	//                                                                      
		muic_detect_accessory(NOT_UPON_IRQ);
	}

	//                     
	set_muic_charger_detected();

#if 1	//                                                                                   
	/* Makes the interrupt on MUIC_GPIO INT wake up AP which is in suspend mode */
	ret = enable_irq_wake(client->irq);
    if (ret < 0) {
		printk(KERN_INFO "[MUIC] MUIC_GPIO wake up source setting failed!\n");
		disable_irq_wake(client->irq);
		return -ENOSYS;
	}
#endif//


     register_reboot_notifier(&muic_reboot_nb);

#if defined(CONFIG_MACH_X3) && defined(CONFIG_ARCH_TEGRA_3x_SOC)
	//jude84.kim For factory. if you need to be move to other file you can.
    if((is_tegra_batteryexistWhenBoot() != 1) && (retain_mode == RETAIN_AP_USB) && (charging_mode == CHARGING_FACTORY) ){
               tegra_cpu_user_cap_set(1000000);
        }
#endif
	printk("[MUIC] muic_probe() Finish\n");

	return ret;
}

static s32 __devexit muic_remove(struct i2c_client *client)
{
	free_irq(client->irq , &client->dev);
	gpio_free(MUIC_GPIO);
	//                                                                                                                         
	i2c_set_clientdata(client, NULL);	// For what?
	remove_lg_muic_proc_file();
	return 0;
}

static s32 muic_suspend(struct i2c_client *client, pm_message_t state)
{
	unsigned long flags;
	
	printk("[MUIC] UIC : suspend is called \n");
	spin_lock_irqsave(&muic_spin_lock, flags);
	muic_irq_already_run = false;
	spin_unlock_irqrestore(&muic_spin_lock, flags);

	client->dev.power.power_state = state;
#if 0	//                                    
	muic_i2c_write_byte(CONTROL_2, USB_DET_DIS);
	muic_i2c_write_byte(CONTROL_1, 0x00);
	muic_i2c_write_byte(SW_CONTROL, OPEN);
//	printk(KERN_INFO "[MUIC] COSMO MUIC : Suspend \n");
#endif
	return 0;
}
		
static s32 muic_resume(struct i2c_client *client)
{
	unsigned long flags;
	printk("[MUIC] UIC : Resume , MUIC_GPIO level %d, muic_irq_already_run %d\n", gpio_get_value(MUIC_GPIO), muic_irq_already_run);
	client->dev.power.power_state = PMSG_ON;
	spin_lock_irqsave(&muic_spin_lock, flags);
	if (gpio_get_value(MUIC_GPIO) == 0 && muic_irq_already_run == false) {
	queue_work(muic_wq, &muic_work);
	}
	spin_unlock_irqrestore(&muic_spin_lock, flags);

#if 0	//                                    
//	printk(KERN_INFO "[MUIC] COSMO MUIC : Resume \n");
	muic_init_device(RESET);
#endif

	return 0;
}

static const struct i2c_device_id muic_ids[] = {
	{"muic" /* "lg_i2c_muic" */, 0},
	{/* end of list */},
};

//                                                            
int fota_ebl_download(void)
{
   return 0;
}
//                                                            


/*                                                                */
static s32 __init muic_state(char *str)
{
        s32 muic_value = simple_strtol(str, NULL, 0);
        retain_mode = muic_value;
        boot_retain_mode = muic_value;	//                                                          
        printk(KERN_INFO "[MUIC] muic_state(): retain_mode = %s (%d)\n", retain_mode_str[retain_mode], retain_mode);
        return 1;
}
__setup("muic_state=", muic_state);

/*                                                                */


/*
 * Allow user space tools to figure out which devices this driver can control.
 * The first parameter should be 'i2c' for i2c client chip drivers.
 */
static struct i2c_driver muic_driver = {
	.probe	  = muic_probe,
	.remove	  = __devexit_p(muic_remove),
	.suspend  = muic_suspend,
	.resume   = muic_resume,
	.id_table = muic_ids,
	.driver	  = {
		.name	= "muic",	 //"lg_i2c_muic",
		.owner	= THIS_MODULE,
	},
};

static s32 __init muic_init(void)
{
    printk(KERN_WARNING "[MUIC] muic_init()\n");
    muic_wq = create_singlethread_workqueue("muic_wq");
    if (!muic_wq) {
        printk(KERN_WARNING "[MUIC] Failed to create a thread(muic_wq)\n");
    }
	return i2c_add_driver(&muic_driver);
}

static void __exit muic_exit(void)
{
#if defined (MUIC_SLEEP)
	wake_lock_destroy(&muic_wake_lock);
#endif//
	i2c_del_driver(&muic_driver);
    destroy_workqueue(muic_wq);
    muic_wq = NULL;
}

module_init(muic_init);
module_exit(muic_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Cosmo MUIC Driver");
MODULE_LICENSE("GPL");

