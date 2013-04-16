/*
 * The header file for X3 INPUT
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MACH_TEGRA_X3_INPUT_H
#define __MACH_TEGRA_X3_INPUT_H

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_COMMON)
#include <linux/input/lge_touch_core.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T1320_HD)
#include <linux/input/touch_synaptics_rmi4_i2c.h>
#endif

#if defined (CONFIG_TOUCHSCREEN_SYNAPTICS_T1320)
#include <linux/synaptics_i2c_rmi.h>
#endif

#if defined (CONFIG_SENSORS_APDS990X)
#include <linux/apds990x.h> 
#endif

#if defined(MPU_SENSORS_MPU6050B1)
#include <linux/mpu.h>
#define SENSOR_MPU_NAME "mpu6050B1"
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T1320_HD)
#define TOUCH_1V8_EN	TEGRA_GPIO_PX4
#define TOUCH_3V0_EN	TEGRA_GPIO_PD3
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_COMMON)
#define TOUCH_1V8_EN	TEGRA_GPIO_PX4
#define TOUCH_3V0_EN	TEGRA_GPIO_PD3
#endif

#if defined( LGE_SENSOR_DCOMPASS)
extern struct ami306_platform_data dcompss_pdata;
#endif

#if defined( LGE_SENSOR_GYROSCOPE)
extern struct k3g_platform_data gyroscope_pdata;
#endif

#if defined (LGE_SENSOR_ACCELEROMETER)
extern struct k3dh_acc_platform_data accelerometer_pdata;
#endif

#if defined(MPU_SENSORS_MPU6050B1) 
extern struct mpu_platform_data mpu6050_data;
extern struct ext_slave_platform_data mpu_compass_data;

#endif

#if defined (CONFIG_SENSORS_APDS990X)
extern struct apds990x_proximity_platform_data x3_prox_data;
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T1320_HD)
extern struct synaptics_ts_platform_data synaptics_platform_data;
#endif

#if defined (CONFIG_TOUCHSCREEN_SYNAPTICS_T1320)	
extern struct synaptics_i2c_rmi_platform_data synaptics_platform_data[];
#endif

#if defined (CONFIG_TOUCHSCREEN_SYNAPTICS_COMMON)	
extern struct touch_platform_data synaptics_pdata;
#endif


#ifdef LGE_SENSOR
#define SENSOR_POWER_OFF_K3DH	0x1
#define SENSOR_POWER_OFF_K3G	0x2
#define SENSOR_POWER_OFF_AMI306	0x4
#define SENSOR_POWER_OFF_VALID	0x7
#endif

#if defined(CONFIG_SENSORS_APDS990X)
int prox_power_on(int sensor);
int prox_power_off(int sensor);
#endif

#if defined (LGE_SENSOR_ACCELEROMETER)||defined (LGE_SENSOR_GYROSCOPE)
int sensor_power_on(int sensor);
int sensor_power_off(int sensor);
#endif

#endif /* __MACH_TEGRA_X3_INPUT_H */
