/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __IMX119_H__
#define __IMX119_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define IMX119_IOCTL_SET_MODE		_IOW('o', 1, struct imx119_mode)
#define IMX119_IOCTL_GET_STATUS		_IOR('o', 2, struct imx119_status)
#define IMX119_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define IMX119_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define IMX119_IOCTL_SET_GAIN		_IOW('o', 5, __u16)
#define IMX119_IOCTL_SET_GROUP_HOLD     _IOW('o', 6, struct imx119_ae)

#define SONY_SENSOR_ID_IMX119			0x0119
struct imx119_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

struct imx119_ae {
	__u32	frame_length;
	__u8	frame_length_enable;
	__u32	coarse_time;
	__u8	coarse_time_enable;
	__s32	gain;
	__u8	gain_enable;
};
struct imx119_status {
	int data;
	int status;
};

#ifdef __KERNEL__
struct imx119_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __IMX119_H__ */

