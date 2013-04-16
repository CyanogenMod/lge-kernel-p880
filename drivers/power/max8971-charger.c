/* 
 *  MAXIM MAX8971 Charger Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>

#include <linux/power/max8971-charger.h>

#include "../../arch/arm/mach-tegra/board.h"

//                                                                          
//                    
#if defined(CONFIG_MACH_VU10) || defined(CONFIG_MACH_X3)
#include <linux/power/lge_battery.h>
#endif
//                                                                          

//                                                      
#include "../misc/muic/muic.h"
extern TYPE_CHARGING_MODE charging_mode;

#define TEGRA_GPIO_PJ2	74

// define register map
#define MAX8971_REG_CHGINT      	0x0F

#define MAX8971_REG_CHGINT_MASK 	0x01

#define MAX8971_REG_CHG_STAT    	0x02

#define MAX8971_DCV_MASK        	0x80
#define MAX8971_DCV_SHIFT       	7
#define MAX8971_DCI_MASK        	0x40
#define MAX8971_DCI_SHIFT       	6
#define MAX8971_DCOVP_MASK      	0x20
#define MAX8971_DCOVP_SHIFT     	5
#define MAX8971_DCUVP_MASK      	0x10
#define MAX8971_DCUVP_SHIFT     	4
#define MAX8971_CHG_MASK        	0x08
#define MAX8971_CHG_SHIFT		3
#define MAX8971_BAT_MASK		0x04
#define MAX8971_BAT_SHIFT		2
#define MAX8971_THM_MASK		0x02
#define MAX8971_THM_SHIFT		1
#define MAX8971_PWRUP_OK_MASK		0x01
#define MAX8971_PWRUP_OK_SHIFT		0
#define MAX8971_I2CIN_MASK		0x01
#define MAX8971_I2CIN_SHIFT		0

#define MAX8971_REG_DETAILS1		0x03
#define MAX8971_DC_V_MASK		0x80
#define MAX8971_DC_V_SHIFT		7
#define MAX8971_DC_I_MASK		0x40
#define MAX8971_DC_I_SHIFT		6
#define MAX8971_DC_OVP_MASK		0x20
#define MAX8971_DC_OVP_SHIFT		5
#define MAX8971_DC_UVP_MASK		0x10
#define MAX8971_DC_UVP_SHIFT		4
#define MAX8971_THM_DTLS_MASK		0x07
#define MAX8971_THM_DTLS_SHIFT  	0

#define MAX8971_THM_DTLS_COLD		1       // charging suspended(temperature<T1)
#define MAX8971_THM_DTLS_COOL		2       // (T1<temperature<T2)
#define MAX8971_THM_DTLS_NORMAL		3       // (T2<temperature<T3)
#define MAX8971_THM_DTLS_WARM		4       // (T3<temperature<T4)
#define MAX8971_THM_DTLS_HOT		5       // charging suspended(temperature>T4)

#define MAX8971_REG_DETAILS2		0x04
#define MAX8971_BAT_DTLS_MASK		0x30
#define MAX8971_BAT_DTLS_SHIFT		4
#define MAX8971_CHG_DTLS_MASK		0x0F
#define MAX8971_CHG_DTLS_SHIFT		0

#define MAX8971_BAT_DTLS_BATDEAD        0   // VBAT<2.1V
#define MAX8971_BAT_DTLS_TIMER_FAULT    1   // The battery is taking longer than expected to charge
#define MAX8971_BAT_DTLS_BATOK          2   // VBAT is okay.
#define MAX8971_BAT_DTLS_GTBATOV        3   // VBAT > BATOV

#define MAX8971_CHG_DTLS_DEAD_BAT           0   // VBAT<2.1V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_PREQUAL            1   // VBAT<3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CC     2   // VBAT>3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CV     3   // VBAT=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TOP_OFF            4   // VBAT>=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_DONE               5   // VBAT>VBATREG, T>Ttopoff+16s, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TIMER_FAULT        6   // VBAT<VBATOV, TJ<TJSHDN
#define MAX8971_CHG_DTLS_TEMP_SUSPEND       7   // TEMP<T1 or TEMP>T4
#define MAX8971_CHG_DTLS_USB_SUSPEND        8   // charger is off, DC is invalid or chaarger is disabled(USBSUSPEND)
#define MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE    9   // TJ > REGTEMP
#define MAX8971_CHG_DTLS_CHG_OFF            	10  // charger is off and TJ >TSHDN

#define MAX8971_REG_CHGCNTL1		0x05
#define MAX8971_DCMON_DIS_MASK		0x02
#define MAX8971_DCMON_DIS_SHIFT		1
#define MAX8971_USB_SUS_MASK		0x01
#define MAX8971_USB_SUS_SHIFT		0

#define MAX8971_REG_FCHGCRNT		0x06
#define MAX8971_CHGCC_MASK		0x1F
#define MAX8971_CHGCC_SHIFT		0
#define MAX8971_FCHGTIME_MASK		0xE0
#define MAX8971_FCHGTIME_SHIFT         0 // BEFORE 5 it disabled need Change? go to X3-board platform data

#define MAX8971_REG_DCCRNT		0x07
#define MAX8971_CHGRSTRT_MASK		0x40
#define MAX8971_CHGRSTRT_SHIFT		6
#define MAX8971_DCILMT_MASK		0x3F
#define MAX8971_DCILMT_SHIFT		0

#define MAX8971_REG_TOPOFF		0x08
#define MAX8971_TOPOFFTIME_MASK		0xE0
#define MAX8971_TOPOFFTIME_SHIFT	5
#define MAX8971_IFST2P8_MASK		0x10
#define MAX8971_IFST2P8_SHIFT		4
#define MAX8971_TOPOFFTSHLD_MASK	0x0C
#define MAX8971_TOPOFFTSHLD_SHIFT	2
#define MAX8971_CHGCV_MASK		0x03
#define MAX8971_CHGCV_SHIFT		0

#define MAX8971_REG_TEMPREG		0x09
#define MAX8971_REGTEMP_MASK		0xC0
#define MAX8971_REGTEMP_SHIFT		6
#define MAX8971_THM_CNFG_MASK		0x08
#define MAX8971_THM_CNFG_SHIFT		3
//#define MAX8971_THM_CNFG_SHIFT	5
#define MAX8971_SAFETYREG_MASK		0x01
#define MAX8971_SAFETYREG_SHIFT		0

#define MAX8971_REG_PROTCMD		0x0A
#define MAX8971_CHGPROT_MASK		0x0C
#define MAX8971_CHGPROT_SHIFT		2
#define MAX8971_CHGPROT_UNLOCKED	0x03

#define MAX_CHARGER_INT_COUNT 10			//                                                             

//#define MAX8971_TOPOFF_WORKAROUND

//#ifdef MAX8971_TOPOFF_WORKAROUND
//#define MAX8971_TOPOFF_DELAY    ((HZ) * 60 + 0 /* + topoff timer */ )  // 60 seconds + toppoff timer
//#endif

struct max8971_chip {
	struct i2c_client		*client;
	struct power_supply		charger;
	struct max8971_platform_data	*pdata;
	//kkk_test
	struct delayed_work		monitor_work;
#ifdef MAX8971_TOPOFF_WORKAROUND
	struct delayed_work		topoff_work;
	u8				start_topoff_work;
	u8				prev_chg_dtl;
#endif
	int				gpio;
	int				irq;
	int				chg_online;
	int				chg_cable_type;
	int				chg_status;
	bool				chg_enable;
};

static struct max8971_chip	*max8971_chg;
static struct work_struct max8971_wq;

//                                                                          
//                    
#if defined(CONFIG_MACH_VU10) || defined(CONFIG_MACH_X3)
current_limit_property_t current_limit_state = CURRENT_LIMIT_OFF;
extern current_limit_property_t current_limit_request;
#endif
//                                                                          

static int max8971_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0) {
		dev_err(&client->dev, "%s failed : %d\n", __func__, ret);
	}

	return ret;
}

static int max8971_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "%s failed : %d\n", __func__, ret);
	}
	
	return ret;
}

static int max8971_set_bits(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	u8 value = 0;
	int ret;

	ret = max8971_read_reg(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed : %d\n", __func__, ret);
		goto out;
	}
	
	value &= ~mask;
	value |= data;
	ret = max8971_write_reg(client, reg, value);
out:
	return ret;
}

extern unsigned char chg_flag_muic;
extern unsigned char chg_700_flag;
static int max8971_set_reg(struct max8971_chip *chip, int enable)
{
	u8 reg_val= 0;

	// unlock charger protection
	reg_val = chip->pdata->chg_proctection  << MAX8971_CHGPROT_SHIFT;
	max8971_write_reg(chip->client, MAX8971_REG_PROTCMD, reg_val);	
	dev_dbg(&chip->client->dev, "MAX8971_REG_PROTCMD (0x%x) = 0x%x\n", MAX8971_REG_PROTCMD, reg_val);

	if (enable) {
		/* enable charger */
		dev_dbg(&chip->client->dev, "Charing enable..\n");

		// Set fast charge current and timer
		switch(chip->chg_cable_type) {
			case POWER_SUPPLY_TYPE_USB:
				//                                                      
				if( charging_mode != CHARGING_MHL ) {
//                                                                          
//                    
#if defined(CONFIG_MACH_VU10) || defined(CONFIG_MACH_X3)
					if (current_limit_request == CURRENT_LIMIT_400MA) {
						reg_val = ((chip->pdata->chgcc_400 << MAX8971_CHGCC_SHIFT) |
								(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
					} else
#endif
//                                                                          
					{
						reg_val = ((chip->pdata->chgcc_usb500 << MAX8971_CHGCC_SHIFT) |
								(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
					}
				} else {
					reg_val = ((chip->pdata->chgcc_mhl400 << MAX8971_CHGCC_SHIFT) |
							(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
				}
				break;
			case POWER_SUPPLY_TYPE_MAINS:
//                                                                          
//                    
#if defined(CONFIG_MACH_VU10) || defined(CONFIG_MACH_X3)
				if (current_limit_request == CURRENT_LIMIT_400MA) {
					reg_val = ((chip->pdata->chgcc_400 << MAX8971_CHGCC_SHIFT) |
							(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
				} else
#endif
//                                                                          
				{
					reg_val = ((chip->pdata->chgcc_ta << MAX8971_CHGCC_SHIFT) |
							(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
				}
				break;
			
			case POWER_SUPPLY_TYPE_FACTORY:
				reg_val = ((chip->pdata->chgcc_factory << MAX8971_CHGCC_SHIFT) |
						(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
				break;
			default :
				dev_dbg(&chip->client->dev, "Unknown charger cabel type!!!\n");
				reg_val = ((chip->pdata->chgcc_usb500 << MAX8971_CHGCC_SHIFT) |
						(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
				break;
		}
		max8971_write_reg(chip->client, MAX8971_REG_FCHGCRNT, reg_val);

//                                                                          
/*                                              */
#if 0 //defined(CONFIG_MACH_VU10)
		current_limit_state = current_limit_request;
#endif
/*                                              */
//                                                                          

		dev_dbg(&chip->client->dev, "MAX8971_REG_FCHGCRNT(0x%x) = 0x%x\n", MAX8971_REG_FCHGCRNT, reg_val);
		
		// Set input current limit and charger restart threshold
		switch(chip->chg_cable_type) {
			case POWER_SUPPLY_TYPE_USB:
				//                                                      
				if( charging_mode != CHARGING_MHL ) {
				reg_val = ((chip->pdata->chgrstrt << MAX8971_CHGRSTRT_SHIFT) |
				(chip->pdata->dcilmt_usb500 << MAX8971_DCILMT_SHIFT));
				} else {
					reg_val = ((chip->pdata->chgrstrt << MAX8971_CHGRSTRT_SHIFT) |
					(chip->pdata->dcilmt_mhl400 << MAX8971_DCILMT_SHIFT));
				}
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				if(chg_700_flag == 1){
					reg_val = ((chip->pdata->chgrstrt << MAX8971_CHGRSTRT_SHIFT) |
					(chip->pdata->dcilmt_soc700 << MAX8971_DCILMT_SHIFT));
				}else{
				reg_val = ((chip->pdata->chgrstrt << MAX8971_CHGRSTRT_SHIFT) |
				(chip->pdata->dcilmt_ta << MAX8971_DCILMT_SHIFT));
				}
				break;
			case POWER_SUPPLY_TYPE_FACTORY:
				reg_val = ((chip->pdata->chgrstrt << MAX8971_CHGRSTRT_SHIFT) |
				(chip->pdata->dcilmt_factory << MAX8971_DCILMT_SHIFT));
				break;				
			default :
				printk("%s: unknown chager cabel type!!!\n",__func__);
				reg_val = ((chip->pdata->chgrstrt << MAX8971_CHGRSTRT_SHIFT) |
				(chip->pdata->dcilmt_usb500 << MAX8971_DCILMT_SHIFT));
				break;
				break;
		}
		max8971_write_reg(chip->client, MAX8971_REG_DCCRNT, reg_val);
		dev_dbg(&chip->client->dev, "MAX8971_REG_DCCRNT(0x%x)  = 0x%02x\n", MAX8971_REG_DCCRNT, reg_val);
		
		// Set topoff condition
#if 0
		reg_val = ((chip->pdata->topofftime << MAX8971_TOPOFFTIME_SHIFT) |
				(chip->pdata->topofftshld << MAX8971_TOPOFFTSHLD_SHIFT) |
				(chip->pdata->chgcv << MAX8971_CHGCV_SHIFT) |
				(chip->pdata->ifst2p8 << MAX8971_IFST2P8_SHIFT));
#endif
		switch(chip->chg_cable_type) {
			case POWER_SUPPLY_TYPE_USB:
				//                                                      
				if( charging_mode != CHARGING_MHL ) {
				reg_val = ((chip->pdata->topofftime << MAX8971_TOPOFFTIME_SHIFT) |
				(chip->pdata->topofftshld << MAX8971_TOPOFFTSHLD_SHIFT) |
				(chip->pdata->chgcv << MAX8971_CHGCV_SHIFT) |
				(chip->pdata->ifst2p8_usb500 << MAX8971_IFST2P8_SHIFT));
				} else {
					reg_val = ((chip->pdata->topofftime << MAX8971_TOPOFFTIME_SHIFT) |
					(chip->pdata->topofftshld << MAX8971_TOPOFFTSHLD_SHIFT) |
					(chip->pdata->chgcv << MAX8971_CHGCV_SHIFT) |
					(chip->pdata->ifst2p8_mhl400 << MAX8971_IFST2P8_SHIFT));
				}
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				reg_val = ((chip->pdata->topofftime << MAX8971_TOPOFFTIME_SHIFT) |
				(chip->pdata->topofftshld << MAX8971_TOPOFFTSHLD_SHIFT) |
				(chip->pdata->chgcv << MAX8971_CHGCV_SHIFT) |
				(chip->pdata->ifst2p8_ta << MAX8971_IFST2P8_SHIFT));
				break;
			case POWER_SUPPLY_TYPE_FACTORY:
				reg_val = ((chip->pdata->topofftime << MAX8971_TOPOFFTIME_SHIFT) |
				(chip->pdata->topofftshld << MAX8971_TOPOFFTSHLD_SHIFT) |
				(chip->pdata->chgcv << MAX8971_CHGCV_SHIFT) |
				(chip->pdata->ifst2p8_factory<< MAX8971_IFST2P8_SHIFT));
				break;				
			default :
				printk("%s: unknown chager cabel type!!!\n",__func__);
				break;
		}
		max8971_write_reg(chip->client, MAX8971_REG_TOPOFF, reg_val);
		dev_dbg(&chip->client->dev, "MAX8971_REG_TOPOFF(0x%x) = 0x%02x\n", MAX8971_REG_TOPOFF, reg_val);

		// Set temperature condition
//		reg_val = ((chip->pdata->regtemp << MAX8971_REGTEMP_SHIFT) |
//				(chip->pdata->thm_config << MAX8971_THM_CNFG_SHIFT) |
//				(chip->pdata->safetyreg << MAX8971_SAFETYREG_SHIFT));

//		max8971_write_reg(chip->client, MAX8971_REG_TEMPREG, reg_val); 
//		dev_dbg(&chip->client->dev, "MAX8971_REG_TEMPREG(0x%x) = 0x%02x\n", MAX8971_REG_TEMPREG, reg_val);

		// USB Suspend and DC Voltage Monitoring
		// Set DC Voltage Monitoring to Enable and USB Suspend to Disable
		
		if(chg_flag_muic >= MAX_CHARGER_INT_COUNT)		//                                                                    
		{
			//                                                                                           
			if(chip->chg_cable_type == POWER_SUPPLY_TYPE_USB){
			reg_val = (0x0 << MAX8971_DCMON_DIS_SHIFT)
				|(chip->pdata->suspend_usb << MAX8971_USB_SUS_SHIFT);
		}
		}
		else
		{	
			reg_val = (chip->pdata->m_input_vol << MAX8971_DCMON_DIS_SHIFT)
				|(chip->pdata->suspend_usb << MAX8971_USB_SUS_SHIFT);
		}
		max8971_write_reg(chip->client, MAX8971_REG_CHGCNTL1, reg_val); 
		dev_dbg(&chip->client->dev, "MAX8971_REG_CHGCNTL1(0x%x) = 0x%02x\n", MAX8971_REG_CHGCNTL1, reg_val);
		dev_dbg(&chip->client->dev, "chg_flag_muic = %d\n",chg_flag_muic);
	} 
	else {
		/* disable charge */
		max8971_set_bits(chip->client,
				MAX8971_REG_CHGCNTL1,
				MAX8971_USB_SUS_MASK,
				1);
	}
/*                                              */
#if defined(CONFIG_MACH_VU10)
			current_limit_state = current_limit_request;
#endif
/*                                              */

	dev_info(&chip->client->dev,
			"%s\n",
			(enable) ? "Enable charger" : "Disable charger");
	return 0;
}

static int max8971_charger_detail_irq(int irq, void *data, u8 *val)
{
	struct max8971_chip *chip = (struct max8971_chip *)data;
	
	dev_dbg(&chip->client->dev, 
			"val[0] = 0x%x, val[1] = 0x%x, val[2] = 0x%x\n"
			, val[0], val[1], val[2]);

	switch (irq) {
		case MAX8971_IRQ_PWRUP_OK:
			dev_dbg(&chip->client->dev, "Power Up OK Interrupt\n");

				chg_flag_muic = chg_flag_muic+1;			//                                                                       

				if ((val[0] & MAX8971_DCUVP_MASK) == 0) {
				// check DCUVP_OK bit in CGH_STAT
				// Vbus is valid //
				// Mask interrupt regsiter //
				max8971_write_reg(chip->client, MAX8971_REG_PROTCMD, 0xC0);
				max8971_write_reg(chip->client,	MAX8971_REG_CHGINT_MASK,chip->pdata->int_mask);
				max8971_write_reg(chip->client, MAX8971_REG_TOPOFF, 0x62);
				chip->chg_online = 1;
				cancel_delayed_work(&chip->monitor_work);
				schedule_delayed_work(&chip->monitor_work, 2*HZ);
				
				//max8971_set_reg(chip, 1);
				// DC_V valid and start charging
				
				//chip->chg_cable_type = POWER_SUPPLY_TYPE_USB;
				printk("%s: chip->chg_cable_type 	0x%02x\n",__func__,chip->chg_cable_type);
//				max8971_set_reg(chip, 1);
			}
			

			
			break;

		case MAX8971_IRQ_THM:
			dev_dbg(&chip->client->dev,
				"Thermistor Interrupt: details-0x%x\n",
				(val[1] & MAX8971_THM_DTLS_MASK));
			break;

		case MAX8971_IRQ_BAT:
			dev_dbg(&chip->client->dev,
				"Battery Interrupt: details-0x%x\n",
				(val[2] & MAX8971_BAT_DTLS_MASK));

			switch ((val[2] & MAX8971_BAT_MASK)>>MAX8971_BAT_SHIFT) {
				case MAX8971_BAT_DTLS_BATDEAD:
				    break;
				case MAX8971_BAT_DTLS_TIMER_FAULT:
				    break;
				case MAX8971_BAT_DTLS_BATOK:
				    break;
				case MAX8971_BAT_DTLS_GTBATOV:
				    break;
				default:
				    break;
			}
			break;

		case MAX8971_IRQ_CHG:
			dev_info(&chip->client->dev,
				"Fast Charge Interrupt: details-0x%x\n",
				(val[2] & MAX8971_CHG_DTLS_MASK));

			switch (val[2] & MAX8971_CHG_DTLS_MASK) {
				case MAX8971_CHG_DTLS_DEAD_BAT:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_PREQUAL:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_FAST_CHARGE_CC:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_FAST_CHARGE_CV:
					// insert event if a customer need to do something //
#ifdef MAX8971_TOPOFF_WORKAROUND
					chip->chg_online = 1;
					if ((chip->prev_chg_dtl == MAX8971_CHG_DTLS_FAST_CHARGE_CV) 
								&& (chip->start_topoff_work = 0)) {
						max8971_write_reg(chip->client,
								MAX8971_REG_CHGINT_MASK, 0xCE);
						schedule_delayed_work(&chip->topoff_work,
								MAX8971_TOPOFF_DELAY);
						chip->start_topoff_work = 1;
					}
#endif
					break;
				case MAX8971_CHG_DTLS_TOP_OFF:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_DONE:
					// insert event if a customer need to do something //
					// Charging done and charge off automatically
					break;
				case MAX8971_CHG_DTLS_TIMER_FAULT:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_TEMP_SUSPEND:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_USB_SUSPEND:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_CHG_OFF:
					// insert event if a customer need to do something //
					break;
				default:
					break;
			}


#ifdef MAX8971_TOPOFF_WORKAROUND
			if (chip->start_topoff_work == 1 ) {
				chip->prev_chg_dtl = MAX8971_CHG_DTLS_TOP_OFF;
		        }
		        else {
				chip->prev_chg_dtl = val[2] & MAX8971_CHG_DTLS_MASK;
		        }
#endif
		        break;

		case MAX8971_IRQ_DCUVP:
			if ((val[1] & MAX8971_DC_UVP_MASK) == 0) {
				// VBUS is invalid. VDC < VDC_UVLO
//				max8971_set_reg(chip, 0);
#ifdef MAX8971_TOPOFF_WORKAROUND
				if (chip->start_topoff_work == 1) {
					cancel_delayed_work(&chip->topoff_work);
					chip->start_topoff_work = 0;
				}
#endif
			}

			dev_dbg(&chip->client->dev,
				"DC Under voltage Interrupt: details-0x%x\n",
				(val[1] & MAX8971_DC_UVP_MASK));
			break;

		case MAX8971_IRQ_DCOVP:
			if (val[1] & MAX8971_DC_OVP_MASK) {
				// VBUS is invalid. VDC > VDC_OVLO
				max8971_set_reg(chip, 0);
			}

			dev_dbg(&chip->client->dev,
				"DC Over voltage Interrupt: details-0x%x\n",
				(val[1] & MAX8971_DC_OVP_MASK));
			break;

		case MAX8971_IRQ_DCI:
			dev_dbg(&chip->client->dev,
				"DC Input Current Limit Interrupt: details-0x%x\n",
				(val[1] & MAX8971_DC_I_MASK));
			break;

		case MAX8971_IRQ_DCV:
			dev_dbg(&chip->client->dev,
				"DC Input Voltage Limit Interrupt: details-0x%x\n",
				(val[1] & MAX8971_DC_V_MASK));
			break;
	}
	return 0;
}

static irqreturn_t max8971_charger_wq(int irq, void *data)
{
	struct max8971_chip *chip = (struct max8971_chip *)data;
	int irq_val, irq_mask, irq_name;
	u8 val[3];
	printk("max8971_charger_wq()\n");


	irq_val = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
	irq_mask = max8971_read_reg(chip->client, MAX8971_REG_CHGINT_MASK);

	val[0] = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
	val[1] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS1);
	val[2] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);

	for (irq_name = MAX8971_IRQ_PWRUP_OK; irq_name<MAX8971_NR_IRQS; irq_name++) {
		if ((irq_val & (0x01<<irq_name)) && !(irq_mask & (0x01<<irq_name))) {
         printk("Before max8971_charger_detail_irq() \n");	
			max8971_charger_detail_irq(irq_name, data, val);
         printk("After max8971_charger_detail_irq() \n");	
		}
	}
	
	return IRQ_HANDLED;
}

//kkk_test
/*
static void lge_charger_monitor_work(struct work_struct *work)
{
	struct max8971_chip *chip;
	u8 val = 0;
	chip = container_of(work, struct max8971_chip, monitor_work);
	
	val = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);
	dev_dbg(&chip->client->dev, "MAX8971_REG_DETAILS2(0x%x) = 0x%02x\n", MAX8971_REG_DETAILS2, val);
	
	schedule_delayed_work(&chip->monitor_work, 5 * HZ);

	return;
}
*/

static void lge_charger_setting_work(struct work_struct *work)
{
	struct max8971_chip *chip;
	u8 val = 0;
	chip = container_of(work, struct max8971_chip, monitor_work);
	
	max8971_set_reg(chip, 1);

	return;
}


#ifdef MAX8971_TOPOFF_WORKAROUND
static void max8971_topoff_work(struct work_struct *max8971_work)
{
	struct max8971_chip *chip;
	int irq_val, irq_mask, irq_name;
	u8 val[3];

	chip = container_of(max8971_work, struct max8971_chip, topoff_work);


	dev_dbg(&chip->client->dev, "%s\n", __func__);

	chip->start_topoff_work = 0;

	irq_val = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
	irq_mask = max8971_read_reg(chip->client, MAX8971_REG_CHGINT_MASK);

	//printk(KERN_DEBUG "%s(%d), irq_val = %d, irq_mask = %d\n", __func__, __LINE__, irq_val , irq_mask);

	val[0] = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
	val[1] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS1);
	val[2] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);

	//printk(KERN_DEBUG "%s(%d), val[0] = 0x%x, val[1] = 0x%x, val[2] = 0x%x\n"
	//					, __func__, __LINE__, val[0], val[1], val[2]);

	for (irq_name = MAX8971_IRQ_PWRUP_OK; irq_name < MAX8971_NR_IRQS; irq_name++) {
		if ((irq_val & (0x01<<irq_name)) && !(irq_mask & (0x01<<irq_name))) {
			max8971_charger_detail_irq(irq_name, chip, val);
			//printk(KERN_DEBUG "%s(%d), irq_val = %d, irq_mask = %d\n", __func__, __LINE__, irq_val , irq_mask);
		}
	}

	max8971_write_reg(chip->client,
				MAX8971_REG_CHGINT_MASK,
				chip->pdata->int_mask);            
}
#endif

int max8971_stop_charging(void)
{
	// Charger removed
	max8971_chg->chg_online = 0;
	max8971_set_reg(max8971_chg, 0);
	// Disable GSM TEST MODE
	max8971_set_bits(max8971_chg->client,
			MAX8971_REG_TOPOFF,
			MAX8971_IFST2P8_MASK,
			0<<MAX8971_IFST2P8_SHIFT);

	return 0;
}
EXPORT_SYMBOL(max8971_stop_charging);

int max8971_start_charging(unsigned mA)
{
	if (mA == 2800) {
		// GSM TEST MODE
		max8971_set_bits(max8971_chg->client,
				MAX8971_REG_TOPOFF,
				MAX8971_IFST2P8_MASK,
				1<<MAX8971_IFST2P8_SHIFT);
	}
	else {
		// charger inserted
		max8971_chg->chg_online = 1;
		switch(max8971_chg->chg_cable_type)
		{
			case POWER_SUPPLY_TYPE_USB:
				max8971_chg->pdata->chgcc_usb500= FCHG_CURRENT(mA);
				max8971_chg->pdata->chgcc_mhl400= FCHG_CURRENT(mA);		//                                                      
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				max8971_chg->pdata->chgcc_ta= FCHG_CURRENT(mA);
				break;
			default :
				break;
		}
		max8971_set_reg(max8971_chg, 1);
	}
	return 0;
}
EXPORT_SYMBOL(max8971_start_charging);

static int max8971_is_charging(struct max8971_chip *info)
{
	if (info == NULL) {
		printk(KERN_ERR "info object is not initialized..\n");
		return 0;
	}

	dev_dbg(&info->client->dev, "charging enable  = %d\n", info->chg_enable);
	dev_dbg(&info->client->dev, "charging current = %d\n", info->chg_cable_type);
	dev_dbg(&info->client->dev, "charging status  = %d\n", info->chg_status);

	return info->chg_enable;
}
/*                                              */
int max8971_is_charging_enable(void)
{
	int ret, chg_dtls_val;

	ret = max8971_read_reg(max8971_chg->client, MAX8971_REG_DETAILS2);
	chg_dtls_val = (ret & MAX8971_CHG_DTLS_MASK);

	//if (max8971_chg->chg_online)
	//	printk("[CHG] chg_online is %d\n", max8971_chg->chg_online);

	if ((chg_dtls_val == MAX8971_CHG_DTLS_DONE) ||
		(chg_dtls_val == MAX8971_CHG_DTLS_TIMER_FAULT) ||
		(chg_dtls_val == MAX8971_CHG_DTLS_TEMP_SUSPEND) ||
		(chg_dtls_val == MAX8971_CHG_DTLS_USB_SUSPEND) ||
		(chg_dtls_val == MAX8971_CHG_DTLS_CHG_OFF)) {
		printk("[CHG] %s() = 0 (chg_dtls_val = 0x%02X)\n", __func__, chg_dtls_val);
		return 0;
	}
	else {
		printk("[CHG] %s() = 1 (chg_dtls_val = 0x%02X)\n", __func__, chg_dtls_val);
		return 1;
	}
}
EXPORT_SYMBOL(max8971_is_charging_enable);
/*                                              */

static int max8971_enable_charging(struct max8971_chip *info, bool enable)
{
	if (info == NULL || info->pdata == NULL) {
		if (info == NULL) {
			printk("info is null..\n");
		}
		else {
			if (info->pdata == NULL) 
				printk("info->pdata == null\n");
		}
		return -EINVAL;
	}


	dev_info(&info->client->dev, "Charging : %s(%d)\n", 
			enable ? "enable" : "disable", info->chg_cable_type);

	if (enable) {
		switch (info->chg_cable_type) {
			case POWER_SUPPLY_TYPE_USB:
				//                                                      
				if( charging_mode != CHARGING_MHL )
				printk("cable type is POWER_SUPPLY_TYPE_USB\n");
				else
					printk("cable type is POWER_SUPPLY_TYPE_USB(MHL)\n");
				max8971_set_reg(info, enable);
				info->chg_enable = true;
				info->chg_status = POWER_SUPPLY_STATUS_CHARGING;
				info->chg_cable_type = POWER_SUPPLY_TYPE_USB;
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				printk("cable type is POWER_SUPPLY_TYPE_MAINS\n");
				max8971_set_reg(info, enable);
				info->chg_enable = true;
				info->chg_status = POWER_SUPPLY_STATUS_CHARGING;
				info->chg_cable_type = POWER_SUPPLY_TYPE_MAINS;
				break;
			case POWER_SUPPLY_TYPE_FACTORY:				
				printk("cable type is POWER_SUPPLY_TYPE_FACTORY\n");
				//info->pdata->ifst2p8 = 0x1;
				max8971_set_reg(info, enable);				
				info->chg_enable = true;
				info->chg_status = POWER_SUPPLY_STATUS_CHARGING;
				info->chg_cable_type = POWER_SUPPLY_TYPE_FACTORY;				
				break;
			default:
				max8971_set_reg(info, enable);
				info->chg_enable = false;
				info->chg_status = POWER_SUPPLY_STATUS_UNKNOWN;
				break;
		}
	} 
	else {
		max8971_set_reg(info, enable);
		info->chg_enable	= false;
		info->chg_status	= POWER_SUPPLY_STATUS_DISCHARGING;
		info->chg_cable_type	= POWER_SUPPLY_TYPE_BATTERY;
	}

	dev_info(&info->client->dev, "Charging : info->chg_enable = %s(%d)\n", 
			info->chg_enable ? "enable" : "disable", 
			info->chg_status);


	return max8971_is_charging(info);
}

static int max8971_charger_get_property(struct power_supply *psy,
                                        enum power_supply_property psp,
                                        union power_supply_propval *val)
{
	struct max8971_chip *chip = container_of(psy,
					struct max8971_chip,
					charger);
	int ret = 0;
	int chg_dtls_val;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			dev_dbg(&chip->client->dev, "select POWER_SUPPLY_PROP_ONLINE\n");
			val->intval = chip->chg_online;
			break;
		case POWER_SUPPLY_PROP_STATUS:
			dev_dbg(&chip->client->dev, "select POWER_SUPPLY_PROP_STATUS\n");
			ret = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);
		        chg_dtls_val = (ret & MAX8971_CHG_DTLS_MASK);

			if (chip->chg_online) {
				if (chg_dtls_val == MAX8971_CHG_DTLS_DONE) {
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
					dev_dbg(&chip->client->dev, "select POWER_SUPPLY_STATUS_DISCHARGING__A\n");
				}
				else if ((chg_dtls_val == MAX8971_CHG_DTLS_TIMER_FAULT) ||
					(chg_dtls_val == MAX8971_CHG_DTLS_TEMP_SUSPEND) ||
					(chg_dtls_val == MAX8971_CHG_DTLS_USB_SUSPEND) ||
					(chg_dtls_val == MAX8971_CHG_DTLS_CHG_OFF)) {
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
					dev_dbg(&chip->client->dev, "select POWER_SUPPLY_STATUS_NOT_CHARGING__B\n");
				}
				else {
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
					dev_dbg(&chip->client->dev, "select POWER_SUPPLY_STATUS_CHARGING__C\n");
				}
			}
			else {
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				dev_dbg(&chip->client->dev, "select POWER_SUPPLY_STATUS_DISCHARGING__D\n");
			}
		        ret = 0;
			break;	
		default:
			ret = -ENODEV;
			break;
	}
	return ret;
}

static int max8971_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct max8971_chip *info =
				container_of(psy, struct max8971_chip, charger);
	bool enable = false;

	dev_dbg(&info->client->dev, "%s: psp %d, val->intval %d\n",
					__func__, psp, val->intval);

	switch (psp) {
		/* Set charging current */
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			info->chg_cable_type = val->intval;
			dev_dbg(&info->client->dev, 
					"%s: POWER_SUPPLY_PROP_CURRENT_NOW info->chg_cable_type "
					"= 0x%02x\n", __func__, info->chg_cable_type);
			break;
		/* Enable/Disable charging */
		case POWER_SUPPLY_PROP_STATUS:	
			dev_dbg(&info->client->dev, 
					"%s: POWER_SUPPLY_PROP_STATUS info->chg_cable_type "
					"= 0x%02x\n", __func__, info->chg_cable_type);
			enable = (val->intval == POWER_SUPPLY_STATUS_CHARGING);
			max8971_enable_charging(info, enable);
			break;
		default:
			dev_err(&info->client->dev, "%s: default = 0x%02x\n",
					__func__, info->chg_cable_type);
			return -EINVAL;
	}
	return 0;
}

static irqreturn_t max8971_interrupt_handler(s32 irq, void *data)
{
	printk("max8971_interrupt_handler()\n");

	schedule_work(&max8971_wq);
	return IRQ_HANDLED;
}


static enum power_supply_property max8971_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
};

static ssize_t max8971_charger_show_chgcc_ta(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->pdata->chgcc_ta);
}

static ssize_t max8971_charger_store_chgcc_ta(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);
	int val, ret;

	ret = sscanf(buf, "%d", &val);
	if (!ret)
		return ret;

	chip->pdata->chgcc_ta = val;

	if(chip->chg_enable == true)
	{
		max8971_set_reg(chip, 1);
	}

	return count;
}

//chip->pdata->chgcc_ta = 0x08; //400mA
//chip->pdata->chgcc_ta = 0x0A; //500mA
//chip->pdata->chgcc_ta = 0x0C; //600mA
//chip->pdata->chgcc_ta = 0x0E; //700mA
//chip->pdata->chgcc_ta = 0x10; //800mA


static DEVICE_ATTR(chgcc_ta, S_IRUGO | S_IWUSR | S_IRGRP | S_IWGRP,
		   max8971_charger_show_chgcc_ta,
		   max8971_charger_store_chgcc_ta);


static __devinit int max8971_probe(struct i2c_client *client, 
				   const struct i2c_device_id *id)
{

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max8971_chip *chip;
	int ret;


	dev_info(&client->dev, "%s..\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "%s i2c check failed..\n", __func__);
		return -EIO;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	/*                                              */
	max8971_chg = chip;
	/*                                              */

#ifdef MAX8971_TOPOFF_WORKAROUND
	dev_info(&client->dev, "Starting delayed workd queue..\n");
	INIT_DELAYED_WORK(&chip->topoff_work, max8971_topoff_work);
	//printk(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	chip->start_topoff_work = 0;
	chip->prev_chg_dtl = 0;
#endif
	chip->client	= client;
	chip->pdata	= client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	if(chip->pdata->gpio_init)
		chip->pdata->gpio_init();
	else
		printk("max8971_probe : Failed to gpio init\n");
	
	chip->charger.name		= "charger";
	chip->charger.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->charger.properties	= max8971_charger_props;
	chip->charger.num_properties 	= ARRAY_SIZE(max8971_charger_props);
	chip->charger.get_property	= max8971_charger_get_property;
	chip->charger.set_property	= max8971_charger_set_property;

	ret = power_supply_register(&client->dev, &chip->charger);
	if (ret) {
		dev_err(&client->dev, 
				"power supply register failed : %d\n", ret);
		goto err_power_supply_register;
	}
	
	//INIT_WORK(&max8971_wq, max8971_charger_wq);

	//kkk_test
	INIT_DELAYED_WORK_DEFERRABLE(&chip->monitor_work, lge_charger_setting_work);
	//schedule_delayed_work(&chip->monitor_work, 0);

	if(is_tegra_batteryexistWhenBoot() == 1)
	{
	   	 ret = request_threaded_irq(client->irq, NULL, max8971_charger_wq,
	      		  IRQF_ONESHOT | IRQF_TRIGGER_LOW, client->name, chip);
		
		if (ret < 0) {
			printk(KERN_INFO "[max8971_probe] MAX8971_IRQB_GPIO set up failed!\n");
			free_irq(client->irq, &client->dev);
			return -ENOSYS;
		}
	}

/*
	chip->chg_online = 0;
	ret = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
	if (ret >= 0) {
		chip->chg_online = (ret & MAX8971_DCUVP_MASK) ? 0 : 1;

		if (chip->chg_online) {
			// Set IRQ MASK register
			max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);
			max8971_set_reg(chip, 1);
		}
	}
	else {
		dev_err(&client->dev, "i2c reading err : %d\n", ret);
		goto err;
	}
*/

 	ret = device_create_file(&client->dev, &dev_attr_chgcc_ta);
        if (ret < 0) {
                printk("device_create_file error!\n");
                return ret;
	}
	dev_info(&client->dev, "%s finish...\n", __func__);

	return 0;

err:
	free_irq(client->irq, chip);

err_irq_request:
err_gpio_register_2:
	gpio_free(client->irq);

err_gpio_register_1:
	power_supply_unregister(&chip->charger);

err_power_supply_register:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static __devexit int max8971_remove(struct i2c_client *client)
{
	struct max8971_chip *chip = i2c_get_clientdata(client);

	//kkk_test
	//cancel_delayed_work(&chip->monitor_work);

	gpio_free(TEGRA_GPIO_PJ2);
	free_irq(client->irq, chip);
	power_supply_unregister(&chip->charger);
	kfree(chip);

	return 0;
}

/*                                        
                                            
 */
#if 0
//kkk_test
static int max8971_suspend(struct device *dev)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);

	//cancel_delayed_work(&chip->monitor_work);

	return 0;
}

static int max8971_resume(struct device *dev)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);

	//schedule_delayed_work(&chip->monitor_work, 5 * HZ);

	return 0;
}

static const struct dev_pm_ops lge_charger_pm_ops = {
	.suspend	= max8971_suspend,
	.resume		= max8971_resume,
};
#endif

static const struct i2c_device_id max8971_id[] = {
	{ "max8971", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max8971_id);

static struct i2c_driver max8971_i2c_driver = {
	.driver = {
		.name = "max8971",
		/*                                        
                                              
   */
#if 0
		.pm = &lge_charger_pm_ops,
#endif
	},
	.probe		= max8971_probe,
	.remove		= __devexit_p(max8971_remove),
	.id_table	= max8971_id,
};

static int __init max8971_init(void)
{
	return i2c_add_driver(&max8971_i2c_driver);
}

static void __exit max8971_exit(void)
{
	i2c_del_driver(&max8971_i2c_driver);
}


#if defined(CONFIG_MACH_LGHDK) || defined(CONFIG_MACH_X3)
subsys_initcall(max8971_init);
#else
module_init(max8971_init);
#endif
module_exit(max8971_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clark Kim <clark.kim@maxim-ic.com>");
MODULE_DESCRIPTION("Power supply driver for MAX8971");
MODULE_VERSION("3.3");
MODULE_ALIAS("platform:max8971-charger");
