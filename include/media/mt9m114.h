/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __MT9M114_H__
#define __MT9M114_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9M114_IOCTL_SET_MODE		_IOW('o', 1, struct mt9m114_mode)
#define MT9M114_IOCTL_GET_STATUS		_IOR('o', 2, struct mt9m114_status)

//                                                                           
#if 1
#define MT9M114_IOCTL_SET_COLOR_EFFECT		_IOW('o', 3, unsigned int)
#define MT9M114_IOCTL_SET_WHITE_BALANCE			_IOW('o', 4, unsigned int)
#define MT9M114_IOCTL_SET_EXPOSURE			_IOW('o', 5, int)
#define MT9M114_IOCTL_SET_FPSRANGE			_IOW('o', 6, int)
#define MT9M114_IOCTL_SET_SCENE_MODE        _IOW('o', 7, unsigned int)
#endif
//                                                                           
struct mt9m114_mode {
	int xres;
	int yres;
};

struct mt9m114_status {
	int data;
	int status;
};

//                                          
struct mt9m114_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

struct mt9m114_table_reg {
	const struct mt9m114_i2c_reg_conf * init_tbl;
	const unsigned short inittbl_size;

	const struct mt9m114_i2c_reg_conf * prev_tbl;
	const unsigned short prevtbl_size;

	const struct mt9m114_i2c_reg_conf * snap_tbl;
	const unsigned short snaptbl_size;
};
//                                          
//                                                                           
#if 1
typedef enum {
	YUVCamEffectMode_Color = 0,
	YUVCamEffectMode_WB,
	YUVCamEffectMode_Force32 = 0x7FFFFFFF
} YUVCamEffectMode;

typedef enum {
      YUVCamColorEffect_None= 0,
      YUVCamColorEffect_Negative =3,
      YUVCamColorEffect_Sketch =4,
      YUVCamColorEffect_Solarize = 10,
      YUVCamColorEffect_Posterize = 0x30000000,
      YUVCamColorEffect_Sepia,
      YUVCamColorEffect_Mono,
      YUVCamColorEffect_VendorStartUnused = 0x7F000000, /**< Reserved region for introducing Vendor Extensions */
      YUVCamColorEffect_Aqua,
      YUVCamColorEffect_Force32 = 0x7FFFFFFF
} YUVCamColorEffect;

typedef enum {
      YUVCamWhitebalance_Off = 0,
      YUVCamWhitebalance_Auto = 1,
      YUVCamWhitebalance_SunLight,
      YUVCamWhitebalance_CloudyDayLight,
      YUVCamWhitebalance_Shade,
      YUVCamWhitebalance_Tungsten,
      YUVCamWhitebalance_Fluorescent,
      YUVCamWhitebalance_Incandescent,
      YUVCamWhitebalance_Flash,
      YUVCamWhitebalance_Horizon,
      YUVCamWhitebalance_Force32 = 0x7FFFFFFF
} YUVCamUserWhitebalance;

typedef enum {
    YUVCamSceneMode_Auto = 1,
    YUVCamSceneMode_Night = 8,
    YUVCamSceneMode_Force32 = 0x7FFFFFFF
} YUVCamSceneMode;
#endif
//                                                                           
#ifdef __KERNEL__
struct mt9m114_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __MT9M114_H__ */

