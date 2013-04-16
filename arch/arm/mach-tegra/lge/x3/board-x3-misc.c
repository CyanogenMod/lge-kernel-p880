/*
 * arch/arm/mach-tegra/board-x3.c
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

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-x3.h>
#include <lge/board-x3-misc.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

#if defined(CONFIG_MAX14526_MUIC)
#include <linux/max14526.h>
#endif // CONFIG_MAX14526_MUIC

#include <linux/video/backlight/lm3533_bl.h>

#if defined(CONFIG_MAX8971_CHARGER)
//                                          
#include <linux/power/max8971-charger.h>
#endif

#if defined(CONFIG_BATTERY_MAX17040)	
#include <linux/max17040_battery.h>
#endif

/*                                                                        */
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720) 
#include <linux/regulator/lp8720.h>
static struct lp8720_platform_data lp8720_pdata = {
	.en_gpio_num         = TEGRA_GPIO_PBB4 /*220*/,
};
#endif

#if 0 //                                                       
#include <media/lm3559_flash_led.h>
static struct lm3559_flash_led_platform_data flash_led_data = {
	.gpio_hwen = TEGRA_GPIO_PBB3,
};
#endif

#if defined(CONFIG_ADC_TSC2007)  //                              
#include <linux/tsc2007_adc.h>
#endif


#define CABC_ENABLE

/* backlight platform data*/
struct lge_backlight_platform_data {
	void (*platform_init)(void);
	int gpio;
	unsigned int mode;	/* initial mode */
	int max_current;	/* led max current(0-7F) */
	int initialized;	/* flag which initialize on system boot */
	int version;		/* Chip version number */	
	int max_brightness;
};

//                                             

struct lm3533_bl_platform_data lm3533_pdata = {
	.hwen_gpio		= TEGRA_GPIO_PN6,
	.max_current		= 0x17, // YJChae check point!!
	.min_brightness		= 0x14,
	.default_brightness	= 0x71,
	.max_brightness		= 0xFF,
	.dimming_brightness	= 0x07,
/*
	.gpio = 49,
	.max_current = 0x17, 
	.min_brightness = 0x14, 
	.default_brightness = 0x71, 
	.max_brightness = 0xFF,//0x71, 
	.dimming_brightness = 0x07, // this value should be changed by shoogi.
*/
};



#if defined(CONFIG_MAX8971_CHARGER)
//                                          

#define MAX8971_IRQB_GPIO	TEGRA_GPIO_PJ2

static void max8971_init_gpio(void)
{
	int ret = 0;
	printk("max8971_init_gpio\n");

	ret = gpio_request(MAX8971_IRQB_GPIO, "CHARGER_IRQB_GPIO");
	if (ret < 0) {
		printk("max8971_init_gpio : Failed to gpio request!\n");
		return ret;
	}

	ret = gpio_direction_input(MAX8971_IRQB_GPIO);
	if (ret < 0) {
		printk("max8971_init_gpio : Failed to gpio_direction_input!\n");
		gpio_free(MAX8971_IRQB_GPIO);
		return ret;
	}
	else {
		tegra_gpio_enable(MAX8971_IRQB_GPIO);
	}

	return ret;
}

struct max8971_platform_data max8971_data = {
//                              
//                                                                          
        .chgcc_400      = 0x08,         // 400mA
//                                                                          

	.chgcc_usb500	= 0x0A,		// Fast Charge Current	// USB charging current 500mA
	.chgcc_ta	= 0x12,//0x18,		// Fast Charge Current	// TA charging current 1200mA
	.chgcc_factory 	= 0x1F, 	// Fast Charge Current	// TA charging current 1550mA
	.chgcc_mhl400	= 0x08, 	//                                                                                                            
	.chgcc_soc700	= 0x0E,	//Charging 700mA During  10% Under SOC Contions
	
	.fchgtime	= 0x00,		// Fast Charge Time			//5hrs	
	.chgrstrt	= 0x00,		// Fast Charge Restart Threshold			//150mV
	
	.dcilmt_usb500	= 0x14,//0x14,		// Input Current Limit Selection	//500mA
	//.dcilmt_ta	= 0x28,//0x30,		// Input Current Limit Selection		//1200mA
	.dcilmt_ta		= 0x30,//0x30,//0x30,		// Input Current Limit Selection		//1200mA
	.dcilmt_factory = 0x3F,		// Input Current Limit Selection		//1550mA
	.dcilmt_mhl400	= 0x14,//                                                                                                         
	.dcilmt_soc700	= 0x1D,	//Charging 700mA During  10% Under SOC Contions
	
	.topofftime	= 0x00,		// Top Off Timer Setting				//30min
	.topofftshld	= 0x03,		// Top Off Threshold					//200mA
	.chgcv		= 0x02,		// Charger Termination Voltage : 		//4.35V
	.ifst2p8_usb500	= 0x00,		//Scales Maximum Fast Charge Current to 2.8A.	//disable
	.ifst2p8_ta	= 0x00,		//Scales Maximum Fast Charge Current to 2.8A.	//disable
	.ifst2p8_factory= 0x01,		//Scales Maximum Fast Charge Current to 2.8A.	//disable
	.ifst2p8_mhl400	= 0x00,		//                                                                                                               

	.regtemp	= 0x03,		// Die temperature thermal regulation loop setpoint		//disable temp condition
	.thm_config	= 0x01,		// Thermal monitor configuration				//disable
	.safetyreg	= 0x00,		// JEITA Safety region selection
	
	.int_mask	= 0xFE,		// CHGINT_MASK
	.chg_proctection= 0x3, //Charging protection 	//	unlock
	.m_input_vol	= 0x1, 	//Moniotor input voltage //
	.suspend_usb	= 0x0,	//USB suspend //disable
	.irqb_pgio	= TEGRA_GPIO_TO_IRQ(MAX8971_IRQB_GPIO),	
	.gpio_init	= max8971_init_gpio,
};
#endif /*CONFIG_MAX8971_CHARGER*/


#if defined(CONFIG_MAX14526_MUIC)
//#define MUIC_GPIO	TEGRA_GPIO_PX6
static int max14526_init_gpio(void)
{
	int ret = 0;

	ret = gpio_request(MUIC_GPIO, "muic gpio");
	if (ret < 0) {
		return ret;
	}

	ret = gpio_direction_input(MUIC_GPIO);
	if (ret < 0) {
		gpio_free(MUIC_GPIO);
		return ret;
	}
	else {
		tegra_gpio_enable(MUIC_GPIO);
	}

	return ret;
}

static void max14526_deinit_gpio(void)
{
	gpio_free(MUIC_GPIO);
	return;
}

static int max14526_enable_charger(muic_mode_t mode)
{
	// should use platform data..
	struct power_supply *psy = power_supply_get_by_name("charger");
	union power_supply_propval value;

	if (!psy) {
		printk("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	switch(mode) {
		case MUIC_MODE_UNKNOWN:
		case MUIC_MODE_NONE:
		case MUIC_MODE_INVALID_CHG:
			//value.intval = POWER_SUPPLY_TYPE_BATTERY;
			value.intval = POWER_SUPPLY_STATUS_DISCHARGING;
			printk("%s(%d) checkpoint..\n", __func__, __LINE__);
			break;
		case MUIC_MODE_AP_USB:
		case MUIC_MODE_NA_TA:
		case MUIC_MODE_LG_TA:
			value.intval = POWER_SUPPLY_STATUS_CHARGING;
			printk("%s(%d) checkpoint..\n", __func__, __LINE__);
			break;
		default:
			printk("invalid type:%d\n", mode);
			return -EINVAL;
	}

	//return psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	return psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
}

struct max14526_platform_data max14526_pdata = {
	.init_gpio		= max14526_init_gpio,
	.deinit_gpio		= max14526_deinit_gpio,
	.enable_charger		= max14526_enable_charger,
	.gpio			= MUIC_GPIO,
};
#endif	// CONFIG_MAX14526_MUIC


#if defined(CONFIG_TSPDRV)
struct tspdrv_i2c_platform_data tspdrv_i2c_pdata = {
	.en_gpio		= TEGRA_GPIO_PH1,
	.max_timeout		= 15000,
	.active_low 		= 0,
	.initial_vibrate	= 0,
	.pwm_id 		= 3,
	.period_ns		= 50000,
	.duty_ns		= 1250,	
};
#endif

