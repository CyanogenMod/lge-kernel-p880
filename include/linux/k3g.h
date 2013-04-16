/* 
 * Copyright (C) 2011 LGE, Inc.
 *
 *  * This program is free software; you can redistribute it and/or modify
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

#ifndef	__K3G_H__
#define	__K3G_H__

#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>

#define	K3G_IOCTL_BASE 80
/** The following define the IOCTL command values via the ioctl macros */
#define	K3G_IOCTL_READ_DATA_XYZ		_IOW(K3G_IOCTL_BASE, 0, int)

/** SENSOR_TYPE **/
enum {
	SENSOR_TYPE_ACCELEROMETER = 1, 	//K3DH
	SENSOR_TYPE_GYROSCOPE,		//K3G
	SENSOR_TYPE_DCOMPASS,		//AMI306		
};

struct k3g_platform_data {
	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(int);
	int (*power_off)(int);
	int (*check_power_off_valid)(int type);
};
#endif	/* __K3G_H__ */
