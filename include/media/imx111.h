/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __IMX111_H__
#define __IMX111_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define IMX111_IOCTL_SET_MODE			_IOW('o', 1, struct imx111_mode)
//#define IMX111_IOCTL_GET_STATUS			_IOR('o', 2, struct imx111_status)
#define IMX111_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define IMX111_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define IMX111_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define IMX111_IOCTL_GET_STATUS			_IOR('o', 6, __u8)
//#define IMX111_IOCTL_GET_OTP     	    _IOR('o', 7, struct imx111_otp_data)
#define IMX111_IOCTL_SET_GROUP_HOLD     _IOW('o', 7, struct imx111_ae)
#define IMX111_IOCTL_SET_POWER_ON       _IOW('o', 8, __u32)
#define IMX111_IOCTL_GET_SENSOR_ID		_IOR('o', 9, __u16)
// 2012-04-06 kyungrae.jo : nvidia fuse id patch
#define IMX111_IOCTL_GET_SENSOR_FUSE_ID		_IOR('o', 10, struct imx111_sensor_fuse_id)

#define SONY_SENSOR_ID_IMX111			0x0111
#define SONY_SENSOR_ID_IMX119			0x0119

struct imx111_otp_data {
	/* Only the first 5 bytes are actually used. */
	__u8 sensor_serial_num[6];
	__u8 part_num[8];
	__u8 lens_id[1];
	__u8 manufacture_id[2];
	__u8 factory_id[2];
	__u8 manufacture_date[9];
	__u8 manufacture_line[2];

	__u32 module_serial_num;
	__u8 focuser_liftoff[2];
	__u8 focuser_macro[2];
	__u8 reserved1[12];
	__u8 shutter_cal[16];
	__u8 reserved2[183];

	/* Big-endian. CRC16 over 0x00-0x41 (inclusive) */
	__u16 crc;
	__u8 reserved3[3];
	__u8 auto_load[2];
} __attribute__ ((packed));

struct imx111_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
	int stereo;
};

struct imx111_ae {
	__u32	frame_length;
	__u8	frame_length_enable;
	__u32	coarse_time;
	__u8	coarse_time_enable;
	__s32	gain;
	__u8	gain_enable;
};


struct imx111_status {
	int data;
	int status;
};

struct imx111_chip_id { 
  __u8 id[7];
};

// 2012-04-06 kyungrae.jo : nvidia fuse id patch
struct imx111_sensor_fuse_id {
  __u8 id[7];
};

#ifdef __KERNEL__
struct imx111_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __IMX111_H__ */

