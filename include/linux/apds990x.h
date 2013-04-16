/*
 * include/linux/star_sensors.h
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * Author:  Raghu Sesha Iyenagar (raghu.seshaiyengar@lge.com)
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

#ifndef _LINUX_APDS990x_H
#define _LINUX_APDS990x_H

#define SENSOR_VCC_NAME_LEN 16
#define APDS990x_DRV_NAME "apds990x"

struct apds990x_proximity_platform_data {
	u16 gpio;
	unsigned long irqflags;
	int irq;
	int (*power_on)(int);
	int (*power_off)(int);
	int *vcc_led_drive;
	int *vcc_sensor;
};

#endif /* _LINUX_APDS990X_H */
