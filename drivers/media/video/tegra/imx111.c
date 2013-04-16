/*
 * imx111.c - imx111 sensor driver
 *
 * Copyright (C) 2011 Google Inc.
 *
 * Contributors:
 *      Rebecca Schultz Zavin <rebecca@android.com>
 *
 * Leverage OV9640.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/tegra_camera.h>   //                                                                                          
#include <media/imx111.h>
#include <linux/gpio.h>
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <linux/regulator/lp8720.h>
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_set_gpio(int onoff);
#endif
#define IMX111_RESET_GPIO       84 //TEGRA_GPIO_PK04

#define IMX111_TABLE_WAIT_MS    0
#define IMX111_TABLE_END        1
#define IMX111_MAX_RETRIES      3

#define IMX111_TABLE_WAIT_MS_TIME 30 //10
#define SIZEOF_I2C_TRANSBUF 32

#if 0
#define IMX111_1080P_SUB_SAMPLING_X3 1 //from X3
#endif

#define IMX111_USE_OB_LEVEL_64 1 //2012-04-16 kyungrae.jo : black level 64

struct imx111_reg {
	u16 addr;
	u16 val;
};

struct imx111_info {
	int mode;
	struct i2c_client *i2c_client;
	struct imx111_platform_data *pdata;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
};

//[START] 2012-04-06 kyungrae.jo : nvidia fuse id patch
static u8 FuseID[7];
static u8 FuseIDAvailable = 0;
//[END] 2012-04-06 kyungrae.jo : nvidia fuse id patch

static struct imx111_reg mode_3280x2464[] = {
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  //PLL
	{0x0305,	0x02},
	{0x0307,	0x53},
	{0x30A4,	0x02},
	{0x303C,	0x4B},

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30B1, 0x03}, //add
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},
#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  //Drive, MIPI output IF	
  {0x0202,  0x07},
  {0x0203,  0x56},
  {0x0205,  0x67},
  {0x0340,  0x09},
  {0x0341,  0xBA},
  {0x0342,  0x0D},
  {0x0343,  0xD0},
  {0x0344,  0x00},
  {0x0345,  0x08},
  {0x0346,  0x00},
  {0x0347,  0x30},
  {0x0348,  0x0C},
  {0x0349,  0xD7},
  {0x034A,  0x09},
  {0x034B,  0xCF},
  {0x034C,  0x0C},
  {0x034D,  0xD0},
  {0x034E,  0x09},
  {0x034F,  0xA0},
  {0x0381,  0x01},
  {0x0383,  0x01},
  {0x0385,  0x01},
  {0x0387,  0x01},
  {0x3033,  0x00},
  {0x303D,  0x00}, //hyojin.an add
  {0x303E,  0x41},
  {0x3040,  0x08},
  {0x3041,  0x97},
  {0x3048,  0x00},
  {0x304C,  0x6F},
  {0x304D,  0x03},
  {0x3064,  0x12},
  {0x3073,  0x00},
  {0x3074,  0x11},
  {0x3075,  0x11},
  {0x3076,  0x11},
  {0x3077,  0x11},
  {0x3079,  0x00},
  {0x307A,  0x00},
  {0x309B,  0x20},
  {0x309C,  0x13},
  {0x309E,  0x00},
  {0x30A0,  0x14},
  {0x30A1,  0x08},
  {0x30AA,  0x03}, //hyojinan modify
  {0x30B2,  0x07},
  {0x30D5,  0x00},
  {0x30D6,  0x85},
  {0x30D7,  0x2A},
  {0x30D8,  0x64},
  {0x30D9,  0x89},
  {0x30DE,  0x00},
  {0x30DF,  0x20},
  {0x3102,  0x10},
  {0x3103,  0x44},
  {0x3104,  0x40},
  {0x3105,  0x00},
  {0x3106,  0x0D},
  {0x3107,  0x01},
  {0x3108,  0x09},
  {0x3109,  0x08},
  {0x310A,  0x0F},
  {0x315C,  0x5D},
  {0x315D,  0x5C},
  {0x316E,  0x5E},
  {0x316F,  0x5D},
  {0x3318,  0x60},
  {0x3348,  0xE0},

  //Standby cancle
	{0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}


};

static struct imx111_reg mode_3280x1848[] = {  
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL                           
	{0x0305,	0x02},               
	{0x0307,	0x3F},               
	{0x30A4,	0x02},               
	{0x303C,	0x4B},               

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},    
  {0x30B1, 0x03}, //add
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},
#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif  
  //Drive, MIPI output IF	
  {0x0340,  0x07},
  {0x0341,  0x52},
  {0x0342,  0x0D},
  {0x0343,  0xD0},
  {0x0344,  0x00},
  {0x0345,  0x08},
  {0x0346,  0x01},
  {0x0347,  0x64},
  {0x0348,  0x0C},
  {0x0349,  0xD7},
  {0x034A,  0x08},
  {0x034B,  0x9B},
  {0x034C,  0x0C},
  {0x034D,  0xD0},
  {0x034E,  0x07},
  {0x034F,  0x38},
  {0x0381,  0x01},
  {0x0383,  0x01},
  {0x0385,  0x01},
  {0x0387,  0x01},
  {0x3033,  0x00},
  {0x303D,  0x00}, //hyojin.an add
  {0x303E,  0x41},
  {0x3040,  0x08},
  {0x3041,  0x97},
  {0x3048,  0x00},
  {0x304C,  0x6F},
  {0x304D,  0x03},
  {0x3064,  0x12},
  {0x3073,  0x00},
  {0x3074,  0x11},
  {0x3075,  0x11},
  {0x3076,  0x11},
  {0x3077,  0x11},
  {0x3079,  0x00},
  {0x307A,  0x00},
  {0x309B,  0x20},
  {0x309C,  0x13},
  {0x309E,  0x00},
  {0x30A0,  0x14},
  {0x30A1,  0x08},
  {0x30AA,  0x03}, //hyojinan modify
  {0x30B2,  0x07},
  {0x30D5,  0x00},
  {0x30D6,  0x85},
  {0x30D7,  0x2A},
  {0x30D8,  0x64},
  {0x30D9,  0x89},
  {0x30DE,  0x00},
  {0x30DF,  0x20},
  {0x3102,  0x10},
  {0x3103,  0x44},
  {0x3104,  0x40},
  {0x3105,  0x00},
  {0x3106,  0x0D},
  {0x3107,  0x01},
  {0x3108,  0x09},
  {0x3109,  0x08},
  {0x310A,  0x0F},
  {0x315C,  0x5D},
  {0x315D,  0x5C},
  {0x316E,  0x5E},
  {0x316F,  0x5D},
  {0x3318,  0x60},
  {0x3348,  0xE0},

  //Standby cancle
  {0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}

};

static struct imx111_reg mode_3280x1098[] = {  
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL                           
	{0x0305,	0x02},               
	{0x0307,	0x33},               
	{0x30A4,	0x02},               
	{0x303C,	0x4B},               

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30B1, 0x03},
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},
#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  //Drive, MIPI output IF	
  {0x0340,    0x04},
  {0x0341,    0x6A},
  {0x0342,    0x0D},
  {0x0343,    0xAC},
  {0x0344,    0x00},
  {0x0345,    0x08},
  {0x0346,    0x01},
  {0x0347,    0xF6},
  {0x0348,    0x0C},
  {0x0349,    0xD7},
  {0x034A,    0x08},
  {0x034B,    0x0B},
  {0x034C,    0x0C},
  {0x034D,    0xD0},
  {0x034E,    0x04},
  {0x034F,    0x4A},
  {0x0381,    0x01},
  {0x0383,    0x01},
  {0x0385,    0x01},
  {0x0387,    0x01},
  {0x3033,    0x00},
  {0x303D,    0x10},
  {0x303E,    0x40},
  {0x3040,    0x08},
  {0x3041,    0x93},
  {0x3048,    0x00},
  {0x304C,    0x67},
  {0x304D,    0x03},
  {0x3064,    0x12},
  {0x3073,    0xE0},
  {0x3074,    0x12},
  {0x3075,    0x12},
  {0x3076,    0x12},
  {0x3077,    0x12},
  {0x3079,    0x2A},
  {0x307A,    0x0A},
  {0x309B,    0x60},
  {0x309E,    0x04},
  {0x30A0,    0x15},
  {0x30A1,    0x08},
  {0x30AA,    0x03},
  {0x30B2,    0x05},
  {0x30D5,    0x00},
  {0x30D6,    0x85},
  {0x30D7,    0x2A},
  {0x30D8,    0x64},
  {0x30D9,    0x89},
  {0x30DE,    0x00},
  {0x30DF,    0x20},
  {0x3102,    0x08},
  {0x3103,    0x1D},
  {0x3104,    0x1E},
  {0x3105,    0x00},
  {0x3106,    0x74},
  {0x3107,    0x00},
  {0x3108,    0x03},
  {0x3109,    0x02},
  {0x310A,    0x03},
  {0x315C,    0x37},
  {0x315D,    0x36},
  {0x316E,    0x38},
  {0x316F,    0x37},
  {0x3318,    0x63},
  {0x3348,    0xE0},

  //Standby cancle
  {0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}

};

static struct imx111_reg mode_2100x1200[] = {  
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL
	{0x0305,	0x02},
	{0x0307,	0x38},
	{0x30A4,	0x02},
	{0x303C,	0x4B},

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30B1, 0x03}, //add  
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},
#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  //Drive, MIPI output IF
  {0x0340,  0x04},
  {0x0341,  0xEC},
  {0x0342,  0x0D},
  {0x0343,  0xD0},
  {0x0344,  0x02},
  {0x0345,  0x56},
  {0x0346,  0x02},
  {0x0347,  0xA8},
  {0x0348,  0x0A},
  {0x0349,  0x89},
  {0x034A,  0x07},
  {0x034B,  0x57},
  {0x034C,  0x08},
  {0x034D,  0x34},
  {0x034E,  0x04},
  {0x034F,  0xB0},
  {0x0381,  0x01},
  {0x0383,  0x01},
  {0x0385,  0x01},
  {0x0387,  0x01},
  {0x3033,  0x00},
  {0x303D,  0x10},
  {0x303E,  0x40},
  {0x3040,  0x08},
  {0x3041,  0x97},
  {0x3048,  0x00},
  {0x304C,  0x6F},
  {0x304D,  0x03},
  {0x3064,  0x12},
  {0x3073,  0x00},
  {0x3074,  0x11},
  {0x3075,  0x11},
  {0x3076,  0x11},
  {0x3077,  0x11},
  {0x3079,  0x00},
  {0x307A,  0x00},
  {0x309B,  0x20},
  {0x309C,  0x13},
  {0x309E,  0x00},
  {0x30A0,  0x14},
  {0x30A1,  0x08},
  {0x30AA,  0x03},
  {0x30B2,  0x07},
  {0x30D5,  0x00},
  {0x30D6,  0x85},
  {0x30D7,  0x2A},
  {0x30D8,  0x64},
  {0x30D9,  0x89},
  {0x30DE,  0x00},
  {0x30DF,  0x20},
  {0x3102,  0x08},
  {0x3103,  0x22},
  {0x3104,  0x20},
  {0x3105,  0x00},
  {0x3106,  0x87},
  {0x3107,  0x00},
  {0x3108,  0x03},
  {0x3109,  0x02},
  {0x310A,  0x03},
  {0x315C,  0x9C},
  {0x315D,  0x9B},
  {0x316E,  0x9D},
  {0x316F,  0x9C},
  {0x3318,  0x62},
  {0x3348,  0xE0},

  //Standby cancle
	{0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}


};

static struct imx111_reg mode_1952x1098[] = {
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL
	{0x0305,	0x02},
	{0x0307,	0x32},
	{0x30A4,	0x02},
	{0x303C,	0x4B},

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30B1, 0x03},
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},
#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  //Drive, MIPI output IF
  {0x0340,  0x07},
  {0x0341,  0x5C},
  {0x0342,  0x0D},
  {0x0343,  0xAC},
  {0x0344,  0x00},
  {0x0345,  0x16},
  {0x0346,  0x01},
  {0x0347,  0x6E},
  {0x0348,  0x0C},
  {0x0349,  0xCB},
  {0x034A,  0x08},
  {0x034B,  0x93},
  {0x034C,  0x07},
  {0x034D,  0xA0},
  {0x034E,  0x04},
  {0x034F,  0x4A},
  {0x0381,  0x01},
  {0x0383,  0x01},
  {0x0385,  0x01},
  {0x0387,  0x01},
  {0x3033,  0x00},
  {0x303D,  0x10}, //hyojin.an add
  {0x303E,  0x00},
  {0x3040,  0x08},
  {0x3041,  0x91},
  {0x3048,  0x00},
  {0x304C,  0x67},
  {0x304D,  0x03},
  {0x3064,  0x10},
  {0x3073,  0xA0},
  {0x3074,  0x12},
  {0x3075,  0x12},
  {0x3076,  0x12},
  {0x3077,  0x11},
  {0x3079,  0x0A},
  {0x307A,  0x0A},
  {0x309B,  0x60},
  //                                           
  {0x309E,  0x04},
  {0x30A0,  0x15},
  {0x30A1,  0x08},
  {0x30AA,  0x03},
  {0x30B2,  0x05},
  {0x30D5,  0x20},
  {0x30D6,  0x85},
  {0x30D7,  0x2A},
  {0x30D8,  0x64},
  {0x30D9,  0x89},
  {0x30DE,  0x00},
  {0x30DF,  0x21},
  {0x3102,  0x08},
  {0x3103,  0x1D},
  {0x3104,  0x1E},
  {0x3105,  0x00},
  {0x3106,  0x74},
  {0x3107,  0x00},
  {0x3108,  0x03},
  {0x3109,  0x02},
  {0x310A,  0x03},
  {0x315C,  0x37},
  {0x315D,  0x36},
  {0x316E,  0x38},
  {0x316F,  0x37},
  {0x3318,  0x63},
  {0x3348,  0xA0},

  //Standby cancle
	{0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}
};


static struct imx111_reg mode_1640x1232[] = {
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL
	{0x0305,	0x02},
	{0x0307,	0x38},
	{0x30A4,	0x02},
	{0x303C,	0x4B},

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30B1, 0x03}, //add  
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},
  
#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  //Drive, MIPI output IF
  {0x0340,  0x04},
  {0x0341,  0xE6},
  {0x0342,  0x0D},
  {0x0343,  0xD0},
  {0x0344,  0x00},
  {0x0345,  0x08},
  {0x0346,  0x00},
  {0x0347,  0x30},
  {0x0348,  0x0C},
  {0x0349,  0xD7},
  {0x034A,  0x09},
  {0x034B,  0xCF},
  {0x034C,  0x06},
  {0x034D,  0x68},
  {0x034E,  0x04},
  {0x034F,  0xD0},
  {0x0381,  0x01},
  {0x0383,  0x03},
  {0x0385,  0x01},
  {0x0387,  0x03},
  {0x3033,  0x00},
  {0x303D,  0x10}, //hyojin.an add
  {0x303E,  0x40},
  {0x3040,  0x08},
  {0x3041,  0x97},
  {0x3048,  0x01},
  {0x304C,  0x6F},
  {0x304D,  0x03},
  {0x3064,  0x12},
  {0x3073,  0x00},
  {0x3074,  0x11},
  {0x3075,  0x11},
  {0x3076,  0x11},
  {0x3077,  0x11},
  {0x3079,  0x00},
  {0x307A,  0x00},
  {0x309B,  0x28},
  {0x309C,  0x13},
  {0x309E,  0x00},
  {0x30A0,  0x14},
  {0x30A1,  0x09},
  {0x30AA,  0x03}, //hyojinan modify
  {0x30B2,  0x05},
  {0x30D5,  0x09},
  {0x30D6,  0x01},
  {0x30D7,  0x01},
  {0x30D8,  0x64},
  {0x30D9,  0x89},
  {0x30DE,  0x02},
  {0x30DF,  0x20},
  {0x3102,  0x08},
  {0x3103,  0x22},
  {0x3104,  0x20},
  {0x3105,  0x00},
  {0x3106,  0x87},
  {0x3107,  0x00},
  {0x3108,  0x03},
  {0x3109,  0x02},
  {0x310A,  0x03},
  {0x315C,  0x9C},
  {0x315D,  0x9B},
  {0x316E,  0x9D},
  {0x316F,  0x9C},
  {0x3318,  0x72},
  {0x3348,  0xE0},
	
  //Standby cancle
  {0x0100, 0x01},
  {0x0101, 0x03},
  
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}


};

static struct imx111_reg mode_1640x924[] = {  
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL
	{0x0305,	0x02},
	{0x0307,	0x2A},
	{0x30A4,	0x02},
	{0x303C,	0x4B},

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30B1, 0x03}, //add  
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},

#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  //Drive, MIPI output IF
  {0x0340,  0x03},
  {0x0341,  0xB2},
  {0x0342,  0x0D},
  {0x0343,  0xD0},
  {0x0344,  0x00},
  {0x0345,  0x08},
  {0x0346,  0x01},
  {0x0347,  0x64},
  {0x0348,  0x0C},
  {0x0349,  0xD7},
  {0x034A,  0x08},
  {0x034B,  0x9B},
  {0x034C,  0x06},
  {0x034D,  0x68},
  {0x034E,  0x03},
  {0x034F,  0x9C},
  {0x0381,  0x01},
  {0x0383,  0x03},
  {0x0385,  0x01},
  {0x0387,  0x03},
  {0x3033,  0x00},
  {0x303D,  0x10}, //hyojin.an add  
  {0x303E,  0x40},
  {0x3040,  0x08},
  {0x3041,  0x97},
  {0x3048,  0x01},
  {0x304C,  0x6F},
  {0x304D,  0x03},
  {0x3064,  0x12},
  {0x3073,  0x00},
  {0x3074,  0x11},
  {0x3075,  0x11},
  {0x3076,  0x11},
  {0x3077,  0x11},
  {0x3079,  0x00},
  {0x307A,  0x00},
  {0x309B,  0x28},
  {0x309C,  0x13},
  {0x309E,  0x00},
  {0x30A0,  0x14},
  {0x30A1,  0x09},
  {0x30AA,  0x03}, //hyojinan modify
  {0x30B2,  0x05},
  {0x30D5,  0x09},
  {0x30D6,  0x01},
  {0x30D7,  0x01},
  {0x30D8,  0x64},
  {0x30D9,  0x89},
  {0x30DE,  0x02},
  {0x30DF,  0x20},
  {0x3102,  0x08},
  {0x3103,  0x22},
  {0x3104,  0x20},
  {0x3105,  0x00},
  {0x3106,  0x87},
  {0x3107,  0x00},
  {0x3108,  0x03},
  {0x3109,  0x02},
  {0x310A,  0x03},
  {0x315C,  0x9C},
  {0x315D,  0x9B},
  {0x316E,  0x9D},
  {0x316F,  0x9C},
  {0x3318,  0x72},
  {0x3348,  0xE0},
	
  //Standby cancle
  {0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}


};

static struct imx111_reg mode_1308x736[] = {  
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL
    {0x0305,  0x02},
    {0x0307,  0x38},
    {0x30A4,  0x02},
    {0x303C,  0x4B},
  
    //Initial(Global)
    {0x3080, 0x50},
    {0x3087, 0x53},
    {0x309D, 0x94},
    {0x30B1, 0x03}, //add  
    {0x30C6, 0x00},
    {0x30C7, 0x00},
    {0x3115, 0x0B},
    {0x3118, 0x30},
    {0x311D, 0x25},
    {0x3121, 0x0A},
    {0x3212, 0xF2},
    {0x3213, 0x0F},
    {0x3215, 0x0F},
    {0x3217, 0x0B},
    {0x3219, 0x0B},
    {0x321B, 0x0D},
    {0x321D, 0x0D},
    {0x32AA, 0x11},
#ifdef IMX111_USE_OB_LEVEL_64
	{0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  
    //Drive, MIPI output IF
    //{0x0340,  0x04},
    //{0x0341,  0x9C},
    {0x0340,  0x09},
//    {0x0341,  0x38},
    {0x0341,  0x41},
    {0x0342,  0x07},
    {0x0343,  0x68},
    {0x0344,  0x01},
    {0x0345,  0x54},
    {0x0346,  0x02},
    {0x0347,  0x20},
    {0x0348,  0x0B},
    {0x0349,  0x8B},
    {0x034A,  0x07},
    {0x034B,  0xDF},
    {0x034C,  0x05},
    {0x034D,  0x1C},
    {0x034E,  0x02},
    {0x034F,  0xE0},
    {0x0381,  0x01},
    {0x0383,  0x01},
    {0x0385,  0x01},
    {0x0387,  0x03},
    {0x3033,  0x84},
    {0x303D,  0x10}, //hyojin.an add
    {0x303E,  0x40},
    {0x3040,  0x08},
    {0x3041,  0x97},
    {0x3048,  0x01},
    {0x304C,  0xD7},
    {0x304D,  0x01},
    {0x3064,  0x12},
    {0x3073,  0x00},
    {0x3074,  0x11},
    {0x3075,  0x11},
    {0x3076,  0x11},
    {0x3077,  0x11},
    {0x3079,  0x00},
    {0x307A,  0x00},
    {0x309B,  0x48},
    {0x309C,  0x12},
    {0x309E,  0x04},
    {0x30A0,  0x14},
    {0x30A1,  0x0A},
    {0x30AA,  0x01}, //hyojinan modify
    {0x30B2,  0x05},
    {0x30D5,  0x04},
    {0x30D6,  0x85},
    {0x30D7,  0x2A},
    {0x30D8,  0x64},
    {0x30D9,  0x89},
    {0x30DE,  0x00},
    {0x30DF,  0x20},
    {0x3102,  0x08},
    {0x3103,  0x22},
    {0x3104,  0x20},
    {0x3105,  0x00},
    {0x3106,  0x87},
    {0x3107,  0x00},
    {0x3108,  0x03},
    {0x3109,  0x02},
    {0x310A,  0x03},
    {0x315C,  0x42},
    {0x315D,  0x41},
    {0x316E,  0x43},
    {0x316F,  0x42},
    {0x3318,  0x62},
    {0x3348,  0xE0},

  //Standby cancle
  {0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}


};


static struct imx111_reg mode_820x614[] = {  
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL
	{0x0305,	0x02},
	{0x0307,	0x38},
	{0x30A4,	0x02},
	{0x303C,	0x4B},

  //Initial(Global)
  {0x3080, 0x50},
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30B1, 0x03}, //add  
  {0x30C6, 0x00},
  {0x30C7, 0x00},
  {0x3115, 0x0B},
  {0x3118, 0x30},
  {0x311D, 0x25},
  {0x3121, 0x0A},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  {0x32AA, 0x11},

#ifdef IMX111_USE_OB_LEVEL_64
  {0x3032, 0x40}, //kyungrae.jo : optical black
#endif
  //Drive, MIPI output IF
  //{0x0340,  0x02},
  //{0x0341,  0x76}, 
  {0x0340,  0x04},
  {0x0341,  0xec},
  {0x0342,  0x0D},
  {0x0343,  0xD0},
  {0x0344,  0x00},
  {0x0345,  0x08},
  {0x0346,  0x00}, // 1 -> 0 : nv patch
  {0x0347,  0x34},
  {0x0348,  0x0C},
  {0x0349,  0xD7},
  {0x034A,  0x09}, //nv patch 8 -> 9
  {0x034B,  0xCB},
  {0x034C,  0x03},
  {0x034D,  0x34},
  {0x034E,  0x02},
  {0x034F,  0x66},
  {0x0381,  0x05},
  {0x0383,  0x03},
  {0x0385,  0x05},
  {0x0387,  0x03},
  {0x3033,  0x00},
  {0x303D,  0x10}, //hyojin.an add  
  {0x303E,  0x40},
  {0x3040,  0x08},
  {0x3041,  0x97},
  {0x3048,  0x01},
  {0x304C,  0x6F},
  {0x304D,  0x03},
  {0x3064,  0x12},
  {0x3073,  0x00},
  {0x3074,  0x11},
  {0x3075,  0x11},
  {0x3076,  0x11},
  {0x3077,  0x11},
  {0x3079,  0x00},
  {0x307A,  0x00},
  {0x309B,  0x28},
  {0x309C,  0x13},
  {0x309E,  0x00},
  {0x30A0,  0x14},
  {0x30A1,  0x09},
  {0x30AA,  0x03}, //hyojinan modify
  {0x30B2,  0x03},
  {0x30D5,  0x09},
  {0x30D6,  0x00},
  {0x30D7,  0x00},
  {0x30D8,  0x00},
  {0x30D9,  0x00},
  {0x30DE,  0x04},
  {0x30DF,  0x20},
  {0x3102,  0x08},
  {0x3103,  0x22},
  {0x3104,  0x20},
  {0x3105,  0x00},
  {0x3106,  0x87},
  {0x3107,  0x00},
  {0x3108,  0x03},
  {0x3109,  0x02},
  {0x310A,  0x03},
  {0x315C,  0x9C},
  {0x315D,  0x9B},
  {0x316E,  0x9D},
  {0x316F,  0x9C},
  {0x3318,  0x7A},
  {0x3348,  0xE0},
	
  //Standby cancle
  {0x0100, 0x01},
  {0x0101, 0x03},
    
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}


};

static struct imx111_reg mode_180x148[] = {  
  {0x0100, 0x00},
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},

  //PLL
	{0x0305,	0x02},
	{0x0307,	0x38},
	{0x30A4,	0x02},
	{0x303C,	0x4B},

  //Initial(Global)
  {0x0340,	0x09},
  {0x0341,	0x3A},
  {0x0342,	0x07},
  {0x0343,	0x68},
  {0x0344,	0x03},
  {0x0345,	0xA0},
  {0x0346,	0x02},
  {0x0347,	0xB0},
  {0x0348,	0x09},
  {0x0349,	0x3F},
  {0x034A,	0x07},
  {0x034B,	0x4F},
  {0x034C,	0x00},
  {0x034D,	0xB4},
  {0x034E,	0x00},
  {0x034F,	0x94},
  {0x0381,	0x05},
  {0x0383,	0x03},
  {0x0385,	0x09},
  {0x0387,	0x07},
  {0x3033,	0x84},
  {0x303D,	0x10},
  {0x303E,	0x40},
  {0x3040,	0x08},
  {0x3041,	0x97},
  {0x3048,	0x01},
  {0x304C,	0xD7},
  {0x304D,	0x01},
  {0x3064,	0x12},
  {0x3073,	0x00},
  {0x3074,	0x11},
  {0x3075,	0x11},
  {0x3076,	0x11},
  {0x3077,	0x11},
  {0x3079,	0x00},
  {0x307A,	0x00},
  {0x309B,	0x48},
  {0x309C,	0x12},
  {0x309E,	0x04},
  {0x30A0,	0x14},
  {0x30A1,	0x0A},
  {0x30AA,	0x00},
  {0x30B2,	0x01},
  {0x30D5,	0x0D},
  {0x30D6,	0x00},
  {0x30D7,	0x00},
  {0x30D8,	0x00},
  {0x30D9,	0x00},
  {0x30DE,	0x04},
  {0x30DF,	0x20},
  {0x3102,	0x05},
  {0x3103,	0x12},
  {0x3104,	0x12},
  {0x3105,	0x00},
  {0x3106,	0x46},
  {0x3107,	0x00},
  {0x3108,	0x03},
  {0x3109,	0x02},
  {0x310A,	0x03},
  {0x315C,	0x48},
  {0x315D,	0x47},
  {0x316E,	0x49},
  {0x316F,	0x48},
  {0x3318,	0x7A},
  {0x3348,	0xE0},
    
  //Standby cancle
  {0x0100, 0x01},
  {0x0101, 0x03},
      
  {IMX111_TABLE_WAIT_MS, IMX111_TABLE_WAIT_MS_TIME},
  {IMX111_TABLE_END, 0x00}
  
  
};


enum {  
  IMX111_MODE_3280x2464,
  IMX111_MODE_3280x1848,
  IMX111_MODE_3280x1098,
  IMX111_MODE_2100x1200,
  IMX111_MODE_1952x1098,
  IMX111_MODE_1640x1232,
  IMX111_MODE_1640x924,  
  IMX111_MODE_1308x736,
  IMX111_MODE_820x614,
  IMX111_MODE_180x148,
};

static struct imx111_reg *imx111_mode_table[] = {  
  [IMX111_MODE_3280x2464] = mode_3280x2464,
  [IMX111_MODE_3280x1848] = mode_3280x1848,
  [IMX111_MODE_3280x1098] = mode_3280x1098,
  [IMX111_MODE_2100x1200] = mode_2100x1200,
  [IMX111_MODE_1952x1098] = mode_1952x1098,
  [IMX111_MODE_1640x1232] = mode_1640x1232,
  [IMX111_MODE_1640x924]  = mode_1640x924,  
  [IMX111_MODE_1308x736]  = mode_1308x736,
  [IMX111_MODE_820x614]   = mode_820x614,
  [IMX111_MODE_180x148]   = mode_180x148,
};

/* 2 regs to program frame length */
static inline void imx111_get_frame_length_regs(struct imx111_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x0340;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x0341;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 3 regs to program coarse time */
static inline void imx111_get_coarse_time_regs(struct imx111_reg *regs,
                                               u32 coarse_time)
{
	regs->addr = 0x0202;
	regs->val = (coarse_time >> 8) & 0x00ff;
	(regs + 1)->addr = 0x0203;
	(regs + 1)->val = (coarse_time) & 0x00ff;
}

/* 1 reg to program gain */
static inline void imx111_get_gain_reg(struct imx111_reg *regs, u16 gain)
{
	regs->addr = 0x205;
	regs->val = gain;
}

static int imx111_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3] = {0x0,}; //                                       

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
	{
		pr_info("imx111_read_reg, err : %d ", err);
		return -EINVAL;
	}

	*val = data[2];

	return 0;
}

static int imx111_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0x00ff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
//		pr_info("imx111: i2c transfer under transferring %x %x\n", addr, val);
		if (err == 1)
			return 0;
		retry++;
		pr_err("imx111: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(3);
	} while (retry <= IMX111_MAX_RETRIES);

	return err;
}

static int imx111_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("imx111: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int imx111_write_table(struct i2c_client *client,
			      const struct imx111_reg table[],
			      const struct imx111_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct imx111_reg *next;
	int i;
	u16 val;

  //                        
	//ret = imx111_write_reg(client, 0x0104, 0x0001);

	//pr_info("imx111: imx111_write_table entered");
	for (next = table; next->addr != IMX111_TABLE_END; next++) {
		if (next->addr == IMX111_TABLE_WAIT_MS) {
			//pr_info("imx111: imx111_write_table : IMX111_TABLE_WAIT_MS ");
			//hyojin.an@lge 2nd camera camp 
			msleep(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = imx111_write_reg(client, next->addr, val);
		if (err) {
			pr_err("imx111: imx111_write_table : err");
			return err;
		}
	}
	return 0;
}

static int imx111_set_mode(struct imx111_info *info, struct imx111_mode *mode)
{
	int sensor_mode;
	int err;  
//	struct imx111_reg reg_list[6];

	//pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
	//	__func__, mode->xres, mode->yres, mode->frame_length,mode->coarse_time, mode->gain);
  pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);

  if ((mode->xres == 3280 || mode->xres == 3264) && (mode->yres == 2464 || mode->yres == 2448))
    sensor_mode = IMX111_MODE_3280x2464;    
  else if ((mode->xres == 3280 || mode->xres == 3264) && (mode->yres == 1848 || mode->yres == 1836))
    sensor_mode = IMX111_MODE_3280x1848;  
  else if ((mode->xres == 1952 || mode->xres == 1920) && (mode->yres == 1098 || mode->yres == 1080))
    sensor_mode = IMX111_MODE_1952x1098;  
  else if ((mode->xres == 1640 || mode->xres == 1440) && (mode->yres == 1232 || mode->yres == 1080))//                                                                      
    sensor_mode = IMX111_MODE_1640x1232;
  else if (mode->xres == 1640 && mode->yres == 924)
    sensor_mode = IMX111_MODE_1640x924;
  else if ((mode->xres == 1308 || mode->xres == 1280) && (mode->yres == 736 || mode->yres == 720))
    sensor_mode = IMX111_MODE_1308x736;
  else if ((mode->xres == 820 || mode->xres == 640) && (mode->yres == 614 || mode->yres == 480))
    sensor_mode = IMX111_MODE_820x614;  
  else if ((mode->xres == 176) && (mode->yres == 144))
    sensor_mode = IMX111_MODE_180x148;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);    
      return -EINVAL;
    }
	tegra_camera_set_size(mode->xres, mode->yres);  //                                                                                          

#if 0
	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	imx111_get_frame_length_regs(reg_list, mode->frame_length);
	imx111_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	imx111_get_gain_reg(reg_list + 5, mode->gain);

	//                                                                                               
  err = imx111_write_table(info->i2c_client, imx111_mode_table[sensor_mode], reg_list, 6);
#else
  err = imx111_write_table(info->i2c_client, imx111_mode_table[sensor_mode], NULL, 0);
#endif
	if (err){
    pr_err("%s: mode_table write error(%d)!!\n", __func__, err);
		return err;
	}

	info->mode = sensor_mode;
	return 0;
}

//static int imx111_get_status(struct imx111_info *info, struct imx111_status *dev_status)
static int imx111_get_status(struct imx111_info *info, u8 *dev_status)
{
  #if 0
	int err;
	*dev_status = 0;
  
  pr_info("%s\n", __func__);
	err = imx111_read_reg(info->i2c_client, 0x205, dev_status);
  pr_info("%s: %u %d################ printing i2c error message\n", __func__, *dev_status, err);
  
	return err;
  #else
  return 0;
  #endif
}


static int imx111_get_chip_id(struct imx111_info *info, struct imx111_chip_id *chip_id)
{
	int err;

	//pr_info("%s\n", __func__);
	err = imx111_read_reg(info->i2c_client, 0x3580, &chip_id->id[6]);
	if(err)
		return err;
	err = imx111_read_reg(info->i2c_client, 0x3581, &chip_id->id[5]);
	if(err)
		return err;
	err = imx111_read_reg(info->i2c_client, 0x3582, &chip_id->id[4]);
	if(err)
		return err;
	err = imx111_read_reg(info->i2c_client, 0x3583, &chip_id->id[3]);
	if(err)
		return err;
	err = imx111_read_reg(info->i2c_client, 0x3584, &chip_id->id[2]);
	if(err)
		return err;
	err = imx111_read_reg(info->i2c_client, 0x3585, &chip_id->id[1]);
	if(err)
		return err;
	err = imx111_read_reg(info->i2c_client, 0x3586, &chip_id->id[0]);
	if(err)
		return err;

	pr_info("CHIP:ID: 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x\n",
			chip_id->id[0], chip_id->id[1], chip_id->id[2], chip_id->id[3],
			chip_id->id[4], chip_id->id[5], chip_id->id[6]);
	return err;
}

//                                                                                                       
static int imx111_set_frame_length(struct imx111_info *info, u32 frame_length)
{
	int ret;
 	struct imx111_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	imx111_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = imx111_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return 0;
}

//                                                                                                     
static int imx111_set_coarse_time(struct imx111_info *info, u32 coarse_time)  
{
	int ret;
	struct imx111_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	imx111_get_coarse_time_regs(reg_list, coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = imx111_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return ret;
}

//                                                                                       
static int imx111_set_gain(struct imx111_info *info, u16 gain)  
{
	int ret;
	struct imx111_reg reg_list;

	imx111_get_gain_reg(&reg_list, gain);
	ret = imx111_write_reg(info->i2c_client, reg_list.addr, reg_list.val);

	return 0;
}

static int imx111_set_group_hold(struct imx111_info *info, struct imx111_ae *ae)
{
	int ret;

	ret = imx111_write_reg(info->i2c_client, 0x0104, 0x0001);
	if (ret)
		return ret;

	if (ae->gain_enable)
		imx111_set_gain(info, ae->gain);
	if (ae->coarse_time_enable)
		imx111_set_coarse_time(info, ae->coarse_time);
	if (ae->frame_length_enable)
		imx111_set_frame_length(info, ae->frame_length);

	ret = imx111_write_reg(info->i2c_client, 0x0104, 0x0000);
	if (ret)
		return ret;

	return 0;
}


static long imx111_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx111_info *info = file->private_data;
	struct imx111_ae ae = {0};
  //pr_info("%s, cmd=%x\n", __func__,cmd);

	switch (cmd) {
	case IMX111_IOCTL_SET_MODE:
	{
		struct imx111_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct imx111_mode))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}

		return imx111_set_mode(info, &mode);
	}
	case IMX111_IOCTL_SET_FRAME_LENGTH:
        ae.frame_length = (u32) arg;
        ae.frame_length_enable = 1;
        return imx111_set_group_hold(info, &ae);
	case IMX111_IOCTL_SET_COARSE_TIME:
        ae.coarse_time = (u32) arg;
        ae.coarse_time_enable = 1;
        return imx111_set_group_hold(info, &ae);
	case IMX111_IOCTL_SET_GAIN:
        ae.gain = (s32) arg;
        ae.gain_enable = 1;
        return imx111_set_group_hold(info, &ae);
	case IMX111_IOCTL_GET_STATUS:
#if 0 //                 
    return 0;
#else
	{
    u16 status = 0;

		//err = imx111_get_status(info, &status);
    err = copy_to_user((void __user *)arg, &status,2);
    if (err) {
      pr_err("%s %d\n", __func__, __LINE__);
      return err;
    }
    return 0;
	}
#endif

	case IMX111_IOCTL_GET_SENSOR_ID:
	{
		struct imx111_chip_id chip_id;
		err = imx111_get_chip_id(info, &chip_id);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &chip_id,
			sizeof(chip_id))) {
			pr_info("%s (imx111_get_chip_id) fail\n", __func__);
			return -EFAULT;
		}
		return 0;
	}

    //[START] 2012-04-06 kyungrae.jo : nvidia fuse id patch
    case IMX111_IOCTL_GET_SENSOR_FUSE_ID:
    {
        if (FuseIDAvailable == 0)
        {
            // read the 7-byte fuse id
            imx111_read_reg(info->i2c_client, 0x3586, &FuseID[0]);
            imx111_read_reg(info->i2c_client, 0x3585, &FuseID[1]);
            imx111_read_reg(info->i2c_client, 0x3584, &FuseID[2]);
            imx111_read_reg(info->i2c_client, 0x3583, &FuseID[3]);
            imx111_read_reg(info->i2c_client, 0x3582, &FuseID[4]);
            imx111_read_reg(info->i2c_client, 0x3581, &FuseID[5]);
            imx111_read_reg(info->i2c_client, 0x3580, &FuseID[6]);

            FuseIDAvailable = 1;
        }
        if (copy_to_user((void __user *)arg, FuseID,
                 7)) {
            pr_info("[CAM] %s %d\n", __func__, __LINE__);
            return -EFAULT;
        }
        return 0;
    }
    //[END] 2012-04-06 kyungrae.jo : nvidia fuse id patch

	case IMX111_IOCTL_SET_GROUP_HOLD:
	{
		if (copy_from_user(&ae,
						(const void __user *)arg,
						sizeof(struct imx111_ae))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
    
		//pr_info("imx111: imx111_ioctl : IMX111_IOCTL_SET_GROUP_HOLD");
		return imx111_set_group_hold(info, &ae);
	}

	//case IMX111_IOCTL_TEST_PATTERN:
	//{
	//	err = imx111_test_pattern(info, (enum imx111_test_pattern) arg);
	//	if (err)
	//		pr_err("%s %d %d\n", __func__, __LINE__, err);
	//	return err;
	//}
	default:
		pr_info("imx111: imx111_ioctl : default");
		return -EINVAL;
	}
	return 0;
}

static struct imx111_info *info;

static int imx111_power_on(void)
{

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
  //pr_info("LP8720 CamPMIC Main Cam Power ON ++\n");
  subpm_set_gpio(1);

  subpm_set_output(LDO3,1);
  subpm_set_output(SWREG,1);
  subpm_set_output(LDO4,1);

#endif
  mdelay(1);
  gpio_direction_output(IMX111_RESET_GPIO, 1);
  udelay(10);
  gpio_set_value(IMX111_RESET_GPIO, 1);
  udelay(100);

	return 0;
}

static int imx111_power_off(void)
{
    gpio_set_value(IMX111_RESET_GPIO, 0);
    udelay(10);
    gpio_direction_output(IMX111_RESET_GPIO, 0);
    udelay(100);

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
    //pr_info("LP8720 CamPMIC Main Cam Power OFF ++\n");

    subpm_set_output(LDO4,0);
    subpm_set_output(SWREG,0);
    subpm_set_output(LDO3,0);
    udelay(100);
    //                                                                                                   
#endif
	return 0;
}



static int imx111_open(struct inode *inode, struct file *file)
{
	//struct imx111_status dev_status;
    u8 dev_status;
	int err;

	pr_info("%s\n", __func__);
	file->private_data = info;
  
  imx111_power_on();
  
	//dev_status.data = 0;
	//dev_status.status = 0;
	err = imx111_get_status(info, &dev_status);
	return err;
}

int imx111_release(struct inode *inode, struct file *file)
{
	pr_info("imx111: imx111_release");
  imx111_power_off();
  
	file->private_data = NULL;
	return 0;
}


static const struct file_operations imx111_fileops = {
	.owner = THIS_MODULE,
	.open = imx111_open,
	.unlocked_ioctl = imx111_ioctl,
	.release = imx111_release,
};

static struct miscdevice imx111_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx111",
	.fops = &imx111_fileops,
};

static int imx111_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("imx111: probing sensor.\n");

	info = kzalloc(sizeof(struct imx111_info), GFP_KERNEL);
	if (!info) {
		pr_err("imx111: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&imx111_device);
	if (err) {
		pr_err("imx111: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;


	i2c_set_clientdata(client, info);
	tegra_gpio_enable(IMX111_RESET_GPIO);
	err = gpio_request(IMX111_RESET_GPIO, "8m_cam_reset");
  if (err < 0)
    pr_err("%s: gpio_request failed for gpio %s\n",
      __func__, "8m_cam_reset");
  
	return 0;
}

static int imx111_remove(struct i2c_client *client)
{
	struct imx111_info *info;
  pr_info("%s\n", __func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&imx111_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id imx111_id[] = {
	{ "imx111", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, imx111_id);

static struct i2c_driver imx111_i2c_driver = {
	.driver = {
		.name = "imx111",
		.owner = THIS_MODULE,
	},
	.probe = imx111_probe,
	.remove = imx111_remove,
	.id_table = imx111_id,
};

static int __init imx111_init(void)
{
	pr_info("imx111 sensor driver loading\n");
	return i2c_add_driver(&imx111_i2c_driver);
}

static void __exit imx111_exit(void)
{
    pr_info("%s\n", __func__);
    //[START] 2012-04-06 kyungrae.jo : nvidia fuse id patch
    FuseIDAvailable = 0;
    pr_info("%s\n", __func__);
    //[END] 2012-04-06 kyungrae.jo : nvidia fuse id patch
    i2c_del_driver(&imx111_i2c_driver);
}

module_init(imx111_init);
module_exit(imx111_exit);

