/*
 * Cosmo MUIC MAX14526 driver
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

#include <max14526.h>


extern TYPE_MUIC_MODE muic_mode;

extern atomic_t muic_charger_detected;


extern s32 muic_i2c_read_byte(u8 addr, u8 *value);
extern s32 muic_i2c_write_byte(u8 addr, u8 value);

extern void muic_udelay(u32 microsec);


void muic_init_max14526(TYPE_RESET reset)
{
	printk(KERN_WARNING "[chahee.kim] max14526_init()\n");

	//                                        
	atomic_set(&muic_charger_detected,0); 

	/* Clears Default Switch Position (0x03=0x24) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); 

	/* Enables 200K pull-up and ADC (0x01=0x12)*/
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN);

	/* Enables Interrupt and set AUD Click/Pop resistor (0x02=0x50) */
	muic_i2c_write_byte(CONTROL_2, INT_EN);

	//CHR_Charger_Update();
}
EXPORT_SYMBOL(muic_init_max14526);


void set_max14526_ap_uart_mode(void) //UART_MODE
{
	printk(KERN_WARNING "[chahee.kim] set_max14526_ap_uart_mode  \n" );


	/* Enables UART Path (0x03=0x09) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN | CP_EN);

	muic_mode = MUIC_AP_UART;
}


void set_max14526_ap_usb_mode(void)
{
	printk(KERN_WARNING "[chahee.kim] set_max14526_ap_usb_mode  \n" );
	//KIMCS TEST	

	/* Enables USB Path (0x03=0x00) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_DP2 | COMN1_TO_DN1);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN | CP_EN);

	muic_mode = MUIC_AP_USB;
}

void set_max14526_charger_mode(unsigned char int_stat_value)
{
	unsigned char reg_value;

	printk(KERN_WARNING "[chahee.kim] set_max14526_charger_mode, int_stat_value = %x \n", int_stat_value );

	if (((int_stat_value & IDNO) == IDNO_0101) || ((int_stat_value & IDNO) == IDNO_1011)) {
		/* LG Proprietary TA Detected 180K ohm on ID */
		muic_mode = MUIC_LG_TA;
	} else if ((int_stat_value & IDNO) == IDNO_0110) {
		/* 1A charger detected */
		muic_mode = MUIC_TA_1A;
	} else {
		/* Enable interrpt and charger type detection (0x02=0x42) */
		muic_i2c_write_byte(CONTROL_2, INT_EN | CHG_TYPE);

		///////////////////////////////////////////////
		// This line should be modified by a customer.
		// 
		// Wait for Interrupt
		//
		////////////////////////////////////////////////

		/* Read INT_STAT */
		muic_i2c_read_byte(INT_STAT, &reg_value);
		muic_i2c_read_byte(STATUS, &reg_value);

		if (reg_value & DCPORT) {
			/* Dedicated Charger(TA) Detected */
			muic_mode = MUIC_NA_TA;
		} else if (reg_value & CHPORT) {
			set_max14526_ap_usb_mode();
		}
	}
}

#ifndef CHARGER_TEST
extern void lge_charger_ta_mode_enable(void);
extern void lge_charger_usb_mode_enable(void);
extern void lge_charger_disable(void);
#endif

void set_max14526_muic_mode(unsigned char int_stat_value)
{
	unsigned char reg_value;
	
	printk(KERN_WARNING "[chahee.kim] set_max14526_muic_mode, "
			"int_stat_value = %x \n", int_stat_value);

	if (int_stat_value & CHGDET) {
		set_max14526_charger_mode(int_stat_value);
#ifndef CHARGER_TEST
		lge_charger_ta_mode_enable();
#endif
	} 
	else if (int_stat_value & VBUS) {
		if ((int_stat_value & IDNO) == IDNO_0101) {
			set_max14526_ap_usb_mode();
#ifndef CHARGER_TEST
			lge_charger_usb_mode_enable();
#endif
		} 
		else {
			/* Standard USB Host Detected (0x03=0x23) */
			muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_C1COMP);

			muic_udelay(1000);

			/* Read Status Reigster(0x05) */
			muic_i2c_read_byte(STATUS, &reg_value);

			if (reg_value & C1COMP) {
				/* Charger Detected COMP2/COMN1 to H-Z (0x03=0x24) */
				muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				/* Dedicated Charger(TA) Detected */
				muic_mode = MUIC_NA_TA;
			} 
			else {
				/* Turn on USB switches */
				set_max14526_ap_usb_mode();
#ifndef CHARGER_TEST
				lge_charger_usb_mode_enable();
#endif
			}
		}
	} 
	else if ((int_stat_value & IDNO) == IDNO_0010) {
		//set_max14526_cp_usb_mode();
		set_max14526_ap_uart_mode();
#ifndef CHARGER_TEST
		lge_charger_disable();
#endif
	}
	else if ((int_stat_value & IDNO) == IDNO_0101) {
		set_max14526_ap_uart_mode();
#ifndef CHARGER_TEST
		lge_charger_disable();
#endif
	} 
	else {
		/* Accessory Not Supported */
		muic_mode = MUIC_NONE;
		muic_init_max14526(DEFAULT);
#ifndef CHARGER_TEST
		lge_charger_disable();
#endif
	}
}


s32 muic_max14526_detect_accessory(s32 upon_irq)
{
	s32 ret = 0;
	int loop = 0;

	u8 int_stat_value;
		
	/* Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STAT and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STAT and STATUS contents will be held).
	 *
	 * Do this only if muic_max14526_detect_accessory() was called upon IRQ. */
	if (upon_irq) { 
		for (; loop < 250; loop++)
			muic_udelay(1000);
	}
	
	/* Reads INT_STAT */
	ret = muic_i2c_read_byte(INT_STAT, &int_stat_value);
	
	if (ret < 0) {
		printk(KERN_INFO "[chahee.kim] INT_STAT reading failed\n");
		muic_mode = MUIC_UNKNOWN;
		return ret;
	}
	
    printk(KERN_INFO "[chahee.kim] IDNO = %d\n", (int_stat_value & IDNO));
  
  	muic_mode = int_stat_value & IDNO;
	/* Branches according to the previous muic_mode */
	switch (muic_mode) {

	/* MUIC_UNKNOWN is reached in two cases both do not have nothing to do with IRQ.
	 * First, at the initialization time where the muic_mode is not available yet.
	 * Second, whenever the current muic_mode detection is failed.
	 */
	case MUIC_UNKNOWN :

	/* If the previous muic_mode was MUIC_NONE,
	 * the only possible condition for a MUIC IRQ is plugging in an accessory.
	 */
	case MUIC_NONE :
		set_max14526_muic_mode(int_stat_value);           
		break;

	/* If the previous muic_mode was MUIC_NA_TA, MUIC_LG_TA, MUIC_TA_1A, MUIC_INVALID_CHG,
	 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB,
	 * the only possible condition for a MUIC IRQ is plugging out the accessory.
	 * 
	 * In this case, initialize MUIC and wait an IRQ.
	 * We don't need to wait 250msec because this is not an erronous case
	 * (we need to reset the facility to set STATUS for an erronous case and
	 * have to wait 250msec) and, if this is not an erronous case, the facility
	 * was already initialized at the system booting.
	 */
	case MUIC_NA_TA:
	case MUIC_TA_1A:
	case MUIC_INVALID_CHG :
		if ((int_stat_value & VBUS) == 0) {		
			// Exit Charger Mode
			muic_mode = MUIC_NONE;
		}else{		
  		     set_max14526_muic_mode(int_stat_value);           
		}
		
		break;
		
	case MUIC_LG_TA :
		printk(KERN_WARNING "[chahee.kim] LG_TA");
		if ((int_stat_value & CHGDET) == 0) {		
			muic_mode = MUIC_NONE;				
		}
		else {		
  		     set_max14526_muic_mode(int_stat_value);           
		}
	
	
		break;
		
	case MUIC_AP_UART :
		if ((int_stat_value & IDNO) == IDNO_1011) {								
			/* Exit Factory Mode */
			muic_mode = MUIC_NONE;
		}
		else {
  		     set_max14526_muic_mode(int_stat_value);           
		}
		break;	
	case MUIC_AP_USB :
		if ((int_stat_value & VBUS) == 0) {
			/* USB Host Removed */
			muic_mode = MUIC_NONE;		
		}else{		
  		     set_max14526_muic_mode(int_stat_value);           
		}
		break;
		
	default:
		printk(KERN_WARNING "[chahee.kim] Failed to detect an accessory. Try again!\n");
#ifndef CHARGER_TEST
		lge_charger_disable();
#endif
		muic_mode = MUIC_UNKNOWN;
		ret = -1;
		break;
	}	

	
	printk(KERN_WARNING "[chahee.kim] muic_max14526_detect_accessory, muic_mode = %d \n", muic_mode );

	//if (muic_mode == MUIC_UNKNOWN || muic_mode == MUIC_NONE) {
	//	muic_init_max14526(DEFAULT);
       // printk(KERN_INFO "[chahee.kim] charging_ic_deactive()\n");
    //}

	return ret;
}
EXPORT_SYMBOL(muic_max14526_detect_accessory);
