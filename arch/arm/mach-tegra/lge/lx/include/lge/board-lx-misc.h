/*
 * The header file for LX MISC
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MACH_TEGRA_LX_MISC_H
#define __MACH_TEGRA_LX_MISC_H

#include <mach-tegra/gpio-names.h>

#if defined(CONFIG_TSPDRV)
#include <linux/tspdrv_i2c.h>
#endif // CONFIG_ISA1200_VIBRATOR

#if defined(CONFIG_MAX14526_MUIC)|| defined(CONFIG_LGE_MUIC)
#define MUIC_GPIO	TEGRA_GPIO_PJ0 //TEGRA_GPIO_PX6
#endif

#if defined(CONFIG_MAX8971_CHARGER)	//                                       
//#define	MAX8971_SLAVE_ADDR	0x6A
#define	MAX8971_SLAVE_ADDR	0x35
#define	MAX8971_IRQB		TEGRA_GPIO_PJ2
#endif	//                                      

#if defined(CONFIG_BATTERY_MAX17043)	//                                        
#define	MAX17043_SLAVE_ADDR	0x36
#endif	//                                  

#if defined(CONFIG_ADC_TSC2007) //                              
#define TSC2007_ADC_SLAVE_ADDR			0x48
#endif

#if defined(CONFIG_BACKLIGHT_LM353X) //YJChae 
extern struct lm3533_bl_platform_data lm3533_pdata;
#endif

#if defined(CONFIG_MAX8971_CHARGER)
extern struct max8971_platform_data max8971_data;
#endif

#if defined(CONFIG_MAX14526_MUIC)
extern struct max14526_platform_data max14526_pdata ;
#endif

#if defined(CONFIG_TSPDRV)
extern struct tspdrv_i2c_platform_data tspdrv_i2c_pdata;
#endif
#endif /* __MACH_TEGRA_LX_MISC_H */
