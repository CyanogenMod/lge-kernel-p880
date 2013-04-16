/*
 * mt9m114.c - mt9m114 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      hyeongjin.kim<hyeongjin.kim@lge.com>
 *
 * Leverage mt9m114.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 640x480. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 640x480
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/mt9m114.h>
#include <linux/gpio.h>
#if 1 //defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <../../../include/linux/regulator/lp8720.h>
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_set_gpio(int onoff);
#endif
//                                                                           
#if 1
static DEFINE_MUTEX(mt9m114_camera_lock);
typedef struct ExposureValueTypeRec{
    int index;
    int range;
}ExposureValueType;

typedef struct FpsRangeTypeRec{
    int low;
    int high;
}FpsRangeType;
#endif
//                                                                           
struct mt9m114_reg {
	u16 addr;
	u16 val;
};

struct mt9m114_info {
	int mode;
	struct i2c_client *i2c_client;
	struct mt9m114_platform_data *pdata;
};

#ifdef MT9M114_DEBUG
	#define mt_info pr_info
#else
	#define mt_info(arg...) do {} while (0)
#endif

#define  mt_err pr_err

#define MT9M114_TABLE_WAIT_MS 0
#define MT9M114_TABLE_END 1
#define MT9M114_MAX_RETRIES 3

static struct mt9m114_reg mode_640x480[] = {
//##### Reset
//DELAY 100
{MT9M114_TABLE_WAIT_MS, 100},
{0x301A, 0x0230}, 	// RESET_REGISTER
//DELAY 100
{MT9M114_TABLE_WAIT_MS, 100},
//REG = 0x301A, 0x0230 	// RESET_REGISTER
{0x301A, 0x0230},

//[Step2-PLL_Timing]
{0x098E, 0x1000}, 	// LOGICAL_ADDRESS_ACCESS
{0x098E, 0xC97E}, 	// LOGICAL_ADDRESS_ACCESS
{0x0990, 0x0100},	// CAM_SYSCTL_PLL_ENABLE		
{0xC980, 0x0660},	//cam_sysctl_pll_divider_m_n = 1632
{0xC982, 0x0700},	//cam_sysctl_pll_divider_p = 1792
//MIPI Timing            
{0xC984, 0x8041},	//cam_port_output_control = 32833
{0xC988, 0x0F00},	//cam_port_mipi_timing_t_hs_zero = 3840
{0xC98A, 0x0B07},	//cam_port_mipi_timing_t_hs_exit_hs_trail = 2823
{0xC98C, 0x0D01},	//cam_port_mipi_timing_t_clk_post_clk_pre = 3329
{0xC98E, 0x071D},	//cam_port_mipi_timing_t_clk_trail_clk_zero = 1821
{0xC990, 0x0006},	//cam_port_mipi_timing_t_lpx = 6
{0xC992, 0x0A0C},	//cam_port_mipi_timing_init_timing = 2572
//Timing_settings        
{0xC800, 0x0004},	//cam_sensor_cfg_y_addr_start = 4
{0xC802, 0x0004},	//cam_sensor_cfg_x_addr_start = 4
{0xC804, 0x03CB},	//cam_sensor_cfg_y_addr_end = 971
{0xC806, 0x050B},	//cam_sensor_cfg_x_addr_end = 1291
{0x098E, 0x4808}, 	// LOGICAL_ADDRESS_ACCESS		===== 32
{0x0990, 0x02DC},	// cam_sensor_cfg_pixclk
{0x0992, 0x6C00},	// cam_sensor_cfg_pixclk
{0xC80C, 0x0001},		//cam_sensor_cfg_row_speed = 1
{0xC80E, 0x00DB},		//cam_sensor_cfg_fine_integ_time_min = 219
{0xC810, 0x05B3},		//cam_sensor_cfg_fine_integ_time_max = 1469
{0xC812, 0x03EE},		//cam_sensor_cfg_frame_length_lines = 1500
{0xC814, 0x0636},		//cam_sensor_cfg_line_length_pck = 1600
{0xC816, 0x0060},	//cam_sensor_cfg_fine_correction = 96
{0xC818, 0x03C3},	//cam_sensor_cfg_cpipe_last_row = 963
{0xC826, 0x0020},	//cam_sensor_cfg_reg_0_data = 32
{0xC834, 0x0000},	//cam_sensor_control_read_mode = 0
{0xC854, 0x0000},	//cam_crop_window_xoffset = 0
{0xC856, 0x0000},	//cam_crop_window_yoffset = 0
{0xC858, 0x0500},	//cam_crop_window_width = 1280              
{0xC85A, 0x03C0},	//cam_crop_window_height = 960              
{0x098E, 0xC85C},	//LOGICAL_ADDRESS_ACCESS			===== 8
{0x0990, 0x0300},	//cam_crop_cropmode = 3	                    
{0xC868, 0x0500},	//cam_output_width = 1280                   
{0xC86A, 0x03C0},	//cam_output_height = 960                   
{0x098E, 0xC878},	//LOGICAL_ADDRESS_ACCESS			===== 8
{0x0990, 0x0000},	//cam_aet_aemode = 0	
{0xC88C, 0x1E02},	//cam_aet_max_frame_rate = 5120
{0xC88E, 0x0A00},	//cam_aet_min_frame_rate = 2560
{0xC914, 0x0000},	//cam_stat_awb_clip_window_xstart = 0
{0xC916, 0x0000},	//cam_stat_awb_clip_window_ystart = 0
{0xC918, 0x04FF},		//cam_stat_awb_clip_window_xend = 1279
{0xC91A, 0x03BF},		//cam_stat_awb_clip_window_yend = 959
{0xC91C, 0x0000},	//cam_stat_ae_initial_window_xstart = 0
{0xC91E, 0x0000},	//cam_stat_ae_initial_window_ystart = 0
{0xC920, 0x00FF},		//cam_stat_ae_initial_window_xend = 255
{0xC922, 0x00BF},		//cam_stat_ae_initial_window_yend = 191
{0x098E, 0xE801},		//LOGICAL_ADDRESS_ACCESS			===== 8
{0x0990, 0x0000},	//cam_aet_aemode = 0	
{0x098E, 0xCC03}, // LOGICAL_ADDRESS_ACCESS [UVC_POWER_LINE_FREQUENCY_CONTROL]			===== 8
{0x0990, 0x0200}, // UVC_POWER_LINE_FREQUENCY_CONTROL  ==> 60Hz : 0x0200 , 50Hz ==>0x0100      
{0x098E, 0xC88B}, // LOGICAL_ADDRESS_ACCESS [CAM_AET_FLICKER_FREQ_HZ]			===== 8
{0x0990, 0x3400}, // CAM_AET_FLICKER_FREQ_HZ ==> 60Hz : 0x3400 , 50Hz ==>0x2C00
//[Step3-Recommended] //Optimization
{0x316A, 0x8270},
{0x316C, 0x8270},
{0x3ED0, 0x2305},
{0x3ED2, 0x77CF},
{0x316E, 0x8202},
{0x3180, 0x87FF},
{0x30D4, 0x6080},
{0xA802, 0x0008},	// AE_TRACK_MODE
{0x3E14, 0xFF39},	
{0x301A, 0x0234},


//[Change-Config]
{0x098E, 0xDC00}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
{0x0990, 0x2800},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
{0x0080, 0x8002}, 	// COMMAND_REGISTER
//DELAY=100

{MT9M114_TABLE_WAIT_MS, 100},

 {MT9M114_TABLE_END, 0x0000}
};

enum {
	MT9M114_MODE_680x480,
};
//                                                                           
#if 1
static struct mt9m114_reg mt9m114_Color_Effect_Normal[]= {
	{0x098E, 0xC874},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0000},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_Color_Effect_MONO[]= {
	{0x098E, 0xC874},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0100},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_Color_Effect_NEGATIVE[]= {
	{0x098E, 0xC874}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
	{0x0990, 0x0300}, 	// CAM_SFX_CONTROL				========== 8
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_Color_Effect_SEPIA[]= {
	{0x098E, 0xC874}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
	{0x0990, 0x0200}, 	// CAM_SFX_CONTROL				========== 8
	{0x098E, 0xC876}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_SEPIA_CR]
	{0x0990, 0x1E00},	// CAM_SFX_SEPIA_CR				========== 8
	{0x098E, 0xC877}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_SEPIA_CB]
	{0x0990, 0xE800},	// CAM_SFX_SEPIA_CR				========== 8
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_Color_Effect_Aqua[]= {
	{0x098E, 0xC874}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
	{0x0990, 0x0200}, 	// CAM_SFX_CONTROL				========== 8
	{0x098E, 0xC876}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_SEPIA_CR]
	{0x0990, 0x0600},	// CAM_SFX_SEPIA_CR				========== 8
	{0x098E, 0xC877}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_SEPIA_CB]
	{0x0990, 0xE000},	// CAM_SFX_SEPIA_CR				========== 8
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_Color_Effect_Sketch[]= {
	{0x098E, 0xC874}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
	{0x0990, 0x0400}, 	// CAM_SFX_CONTROL				========== 8
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};

static struct mt9m114_reg mt9m114_MWB_Auto[]= {
	{0x098E, 0xC909}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_AWBMODE]
	{0x0990, 0x0300}, 	// CAM_AWB_AWBMODE					========== 8
	{0x098E, 0xC90F}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_R_R]
	{0X0990, 0X8000},
	{0x098E, 0xC910}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_G_R]
	{0X0990, 0X8000},
	{0x098E, 0xC911}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_B_R]
	{0X0990, 0X8000},
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_MWB_Incandescent[]= {
	{0x098E, 0xC909}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_AWBMODE]
	{0x0990, 0x0100}, 	// CAM_AWB_AWBMODE					========== 8
	{0x098E, 0xC90F}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_R_R]
	{0X0990, 0X1800},
	{0x098E, 0xC910}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_G_R]
	{0X0990, 0X8000},
	{0x098E, 0xC911}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_B_R]
	{0X0990, 0XA300},
	{0xC8F0, 0x0C80}, 	// CAM_AWB_COLOR_TEMPERATURE
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_MWB_Fluorescent[]= {
	{0x098E, 0xC909}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_AWBMODE]
	{0x0990, 0x0100}, 	// CAM_AWB_AWBMODE					========== 8
	{0x098E, 0xC90F}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_R_R]
	{0X0990, 0X9000},
	{0x098E, 0xC910}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_G_R]
	{0X0990, 0X8000},
	{0x098E, 0xC911}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_B_R]
	{0X0990, 0X9F00},
	{0xC8F0, 0x11E6}, 	// CAM_AWB_COLOR_TEMPERATURE
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_MWB_Daylight[]= {
	{0x098E, 0xC909}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_AWBMODE]
	{0x0990, 0x0100}, 	// CAM_AWB_AWBMODE					========== 8
	{0x098E, 0xC90F}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_R_R]
	{0X0990, 0X8000},
	{0x098E, 0xC910}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_G_R]
	{0X0990, 0X8000},
	{0x098E, 0xC911}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_B_R]
	{0X0990, 0X7800},
	{0xC8F0, 0x1964}, 	// CAM_AWB_COLOR_TEMPERATURE
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_MWB_CloudyDaylight[]= {
	{0x098E, 0xC909}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_AWBMODE]
	{0x0990, 0x0100}, 	// CAM_AWB_AWBMODE					========== 8
	{0x098E, 0xC90F}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_R_R]
	{0X0990, 0x9D00},
	{0x098E, 0xC910}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_G_R]
	{0X0990, 0x8000},
	{0x098E, 0xC911}, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_K_B_R]
	{0X0990, 0x5C00},
	{0xC8F0, 0x1964}, 	// CAM_AWB_COLOR_TEMPERATURE
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_Framerate_7Fps[]= {
    {0x098E, 0x4812},
    {0xC810, 0x15EC},
    {0xC812, 0x04AA},
    {0xC814, 0x166F},
    {0xC88C, 0x0700},
    {0xC88E, 0x0700},
	{0x098E, 0xDC00},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2800},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	{0x0080, 0x8002}, 	// COMMAND_REGISTER
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg mt9m114_Framerate_10Fps[]= {
	{0x098E, 0x4812},
	{0xC810, 0x05BD},
	{0xC812, 0x0BB8},
	{0xC814, 0x0640},
	{0xC88C, 0x0A00},
	{0xC88E, 0x0A00},
	{0x098E, 0xDC00},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2800},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	{0x0080, 0x8002},	// COMMAND_REGISTER
	{MT9M114_TABLE_WAIT_MS, 10},
	{MT9M114_TABLE_END, 0x0000}

};
static struct mt9m114_reg mt9m114_Framerate_15Fps[]= {
	{0x098E, 0x4812},
	{0xC810, 0x05B3},
	{0xC812, 0x07E0},
	{0xC814, 0x0636},
	{0xC88C, 0x1E02},
	{0xC88E, 0x1E02},
	{0x098E, 0xDC00},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2800},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	{0x0080, 0x8002},	// COMMAND_REGISTER
	{MT9M114_TABLE_WAIT_MS, 10},
	{MT9M114_TABLE_END, 0x0000}

};
//10~ 30fps
static struct mt9m114_reg mt9m114_Framerate_VarFps[]= {
    {0x098E, 0x4812},
    {0xC810, 0x05B3},
    {0xC812, 0x03EE},
    {0xC814, 0x0636},
    {0xC88C, 0x1E02},
    {0xC88E, 0x0A00},
	{0x098E, 0xDC00},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2800},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	{0x0080, 0x8002}, 	// COMMAND_REGISTER
    {MT9M114_TABLE_WAIT_MS, 10},
    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_reg *mt9m114_Set_FramerateList[] =
{
	mt9m114_Framerate_7Fps,
	mt9m114_Framerate_10Fps,
	mt9m114_Framerate_15Fps,
	mt9m114_Framerate_VarFps

};
//10~ 30fps
static struct mt9m114_reg mt9m114_ScenMode_Auto[]= {
	    {0x098E, 0x4812},
        {0xC810, 0x05B3},
	    {0xC812, 0x03EE},
	    {0xC814, 0x0636},
        {0xC88C, 0x1E02},
	    {0xC88E, 0x0A00},
		{0x098E, 0xDC00},	// LOGICAL_ADDRESS_ACCESS			===== 8
		{0x0990, 0x2800},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
		{0x0080, 0x8002}, 	// COMMAND_REGISTER
	    {MT9M114_TABLE_WAIT_MS, 10},
	    {MT9M114_TABLE_END, 0x0000}
};
//7.5~30fps
static struct mt9m114_reg mt9m114_ScenMode_Night[]= {
	    {0x098E, 0x4812},
        {0xC810, 0x05B3},
	    {0xC812, 0x03EE},
	    {0xC814, 0x0636},
        {0xC88C, 0x1E02},
	    {0xC88E, 0x0780},
		{0x098E, 0xDC00},	// LOGICAL_ADDRESS_ACCESS			===== 8
		{0x0990, 0x2800},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
		{0x0080, 0x8002}, 	// COMMAND_REGISTER
        {MT9M114_TABLE_WAIT_MS, 10},
	    {MT9M114_TABLE_END, 0x0000}
};
static struct mt9m114_info *info;

static int prev_brightness_mode;
#endif

//                                                                           
static struct mt9m114_reg *mode_table[] = {
	[MT9M114_MODE_680x480] = mode_640x480,
};

static int mt9m114_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

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
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2] << 8 | data[3];

	return 0;
}

static int mt9m114_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		//mt_info("mt9m114: i2c transfer under transferring %x %x\n", addr, val);
		if (err == 1)
			return 0;
		retry++;
		mt_err("mt9m114: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(3);
	} while (retry <= MT9M114_MAX_RETRIES);

	return err;
}

static int mt9m114_write_table(struct i2c_client *client,
			      const struct mt9m114_reg table[],
			      const struct mt9m114_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct mt9m114_reg *next;
	int i;
	u16 val;

	mt_info("mt9m114: mt9m114_write_table entered");
	for (next = table; next->addr != MT9M114_TABLE_END; next++) {
		//mt_info("mt9m114: mt9m114_write_table 1");
		if (next->addr == MT9M114_TABLE_WAIT_MS) {
			mt_info("mt9m114: mt9m114_write_table : MT9M114_TABLE_WAIT_MS ");
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

		//mt_info("mt9m114: mt9m114_write_table 2");
		err = mt9m114_write_reg(client, next->addr, val);
		if (err) {
			mt_err("mt9m114: mt9m114_write_table : err");
			return err;
		}
	}
	return 0;
}

//                                                                           
#if 1
static int mt9m114_set_color_effect(struct mt9m114_info *info, unsigned int color_effect)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, color_effect);

	switch(color_effect)
	{
		case YUVCamColorEffect_None :
			err = mt9m114_write_table(info->i2c_client, mt9m114_Color_Effect_Normal,NULL,0);
			break;

		case YUVCamColorEffect_Negative :
			err = mt9m114_write_table(info->i2c_client, mt9m114_Color_Effect_NEGATIVE,NULL,0);

			break;

		case YUVCamColorEffect_Aqua :
			err = mt9m114_write_table(info->i2c_client, mt9m114_Color_Effect_Aqua,NULL,0);

			break;

		case YUVCamColorEffect_Posterize :
			err = mt9m114_write_table(info->i2c_client, mt9m114_Color_Effect_Sketch,NULL,0);

			break;

		case YUVCamColorEffect_Sepia :
			err = mt9m114_write_table(info->i2c_client, mt9m114_Color_Effect_SEPIA,NULL,0);

			break;

		case YUVCamColorEffect_Mono :
			err = mt9m114_write_table(info->i2c_client, mt9m114_Color_Effect_MONO,NULL,0);

			break;

		default :
			//err = mt9m114_write_table(info->i2c_client, mt9m114_Color_Effect_Normal,NULL,0);

			break;
	}
	err = mt9m114_write_reg(info->i2c_client,0x0080, 0x8004);
	msleep(100);
	if (err < 0)
		return err;
#if 0
	{
		unsigned short test_data = 0, i;
		for(i=0; i<50; i++){ // max delay ==> 500 ms
			rc  = mt9m114_read_reg(mt9m114_client->addr, 0x0080, &test_data);
			if (rc < 0)
				return rc;

			if((test_data & 0x0004)==0)
				break;
			else
				mdelay(10);

			CDBG_VT("### %s :  Refresh Polling set, 0x0080 Reg : 0x%x\n", __func__, test_data);
		}
	}
#endif
	if (err !=0)
		pr_info("%s : Color Effect : %d,  error : %d\n", __func__, color_effect, err);

	return err;
}

static int mt9m114_set_white_balance(struct mt9m114_info *info, unsigned int wb_mode)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, wb_mode);
	switch(wb_mode)
	{
		case YUVCamWhitebalance_Auto :
			err = mt9m114_write_table(info->i2c_client, mt9m114_MWB_Auto,NULL,0);
			break;

		case YUVCamWhitebalance_Incandescent :
			err = mt9m114_write_table(info->i2c_client, mt9m114_MWB_Incandescent,NULL,0);

			break;

		case YUVCamWhitebalance_SunLight :
			err = mt9m114_write_table(info->i2c_client, mt9m114_MWB_Daylight,NULL,0);

			break;
		case YUVCamWhitebalance_CloudyDayLight:
			err = mt9m114_write_table(info->i2c_client, mt9m114_MWB_CloudyDaylight,NULL,0);

			break;

		case YUVCamWhitebalance_Fluorescent :
			err = mt9m114_write_table(info->i2c_client, mt9m114_MWB_Fluorescent,NULL,0);

			break;

		default :
			//err = mt9m114_write_table(info->i2c_client, mt9m114_MWB_Auto,NULL,0);

			break;
	}
	if (err !=0)
		pr_info("%s : White Balance : %d,  error : %d\n", __func__, wb_mode, err);
	return err;
}

static int brightness_table[] = {0x0020, 0x0024, 0x0028, 0x002E, 0x0032, 0x0035, 0x003E, 0x0048, 0x0050, 0x0054, 0x0058, 0x005C, 0x005F}; // 13 step
																		// value < 0x80 --> value should be increased , max value => 0x7F00
							// value > 0x80 --> value should be decreased	, max value => 0x8100
static int gamma_table_sub[] = {0x00C4, 0x00C8, 0x00CC, 0x00D0, 0x00D4, 0x00D8, 0x00DC, 0x00DF, 0x00E3, 0x00E8, 0x00ED, 0x00F2, 0x00F7}; // 13 step
							// 2.2 gamma
static int mt9m114_set_exposure(struct mt9m114_info *info, ExposureValueType *exposure)
{
	int32_t rc = 0;
	int index;

	if(exposure == NULL || exposure->range == 0)
	{
		pr_info("%s : invalid pointer or range value\n", __func__);
	    return -1;
	}
    if ( exposure->index == 0)
		index = 6;
	else
		index = (exposure->index / 5) + 6;

    if(prev_brightness_mode == index)
	{
		pr_info("###  [CHECK]%s: skip this function, brightness_mode -> %d\n", __func__, index);
		return rc;
	}
	pr_info(" %s : exp(%d) index(%d) range(%d)\n", __func__, exposure->index, index, exposure->range);

	rc = mt9m114_write_reg(info->i2c_client,0x098E, 0x4940);
	if (rc < 0)
		return rc;

	// CAM_LL_GAMMA
	rc = mt9m114_write_reg(info->i2c_client,0xC940, gamma_table_sub[index]);
	if (rc < 0)
		return rc;
       // UVC_BRIGHTNESS_CONTROL
	rc = mt9m114_write_reg(info->i2c_client,0xCC0A, brightness_table[index]);
	if (rc < 0)
		return rc;

	prev_brightness_mode = index;

	return rc;
}

static int mt9m114_set_fpsrange(struct mt9m114_info *info, FpsRangeType *fpsRange)
{
	int err = 0;

	int lo = fpsRange->low;
	int hi = fpsRange->high;
	pr_info("%s:lo=%d, hi=%d\n", __func__, lo, hi);
	if (lo == hi){//fixed framerate
		if (lo==7){
			err =  mt9m114_write_table(info->i2c_client,mt9m114_Set_FramerateList[0],NULL,0);
		}
		else if (lo == 10){
			err =  mt9m114_write_table(info->i2c_client,mt9m114_Set_FramerateList[1],NULL,0);
		}
		else if (lo == 15){
			err =  mt9m114_write_table(info->i2c_client,mt9m114_Set_FramerateList[2],NULL,0);
		}

	}
	else{//variable framerate
		err = mt9m114_write_table(info->i2c_client,mt9m114_Set_FramerateList[3],NULL,0);
	}

	if(prev_brightness_mode != 6) // Set again when mode is not center
	{
		pr_info("### mt9m114_set_fpsrange: Set again when mode is not center, value is %d\n ",prev_brightness_mode);
		err = mt9m114_write_reg(info->i2c_client,0x337E, brightness_table[prev_brightness_mode]);
		if (err < 0)
			return err;

		err = mt9m114_write_reg(info->i2c_client,0xC940, gamma_table_sub[prev_brightness_mode]);
		if (err < 0)
			return err;
	}

	return err;
}
static int mt9m114_set_scene_mode(struct mt9m114_info *info, unsigned int scene_mode)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, scene_mode);

	switch(scene_mode)
	{
		case YUVCamSceneMode_Auto :
			err = mt9m114_write_table(info->i2c_client, mt9m114_ScenMode_Auto,NULL,0);
			break;

		case YUVCamSceneMode_Night :
			err = mt9m114_write_table(info->i2c_client, mt9m114_ScenMode_Night,NULL,0);
			break;

		default :
			//err = hi702_write_table(info->i2c_client, hi702_ScenMode_Auto,NULL,0);

			break;
	}

    if(prev_brightness_mode != 6) // Set again when mode is not center
	{
		pr_info("### mt9m114_set_scene_mode: Set again when mode is not center, value is %d\n ",prev_brightness_mode);
		err = mt9m114_write_reg(info->i2c_client,0x337E, brightness_table[prev_brightness_mode]);
		if (err < 0)
			return err;

		err = mt9m114_write_reg(info->i2c_client,0xC940, gamma_table_sub[prev_brightness_mode]);
		if (err < 0)
			return err;
	}
	if (err !=0)
		pr_info("%s : Scene Mode : %d,  error : %d\n", __func__, scene_mode, err);

	return err;
}

#endif
//                                                                           
static int mt9m114_set_mode(struct mt9m114_info *info, struct mt9m114_mode *mode)
{
	int sensor_mode;
	int err;

	mt_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
	if (mode->xres == 640 && mode->yres == 480)
		sensor_mode = MT9M114_MODE_680x480;
	else if (mode->xres == 1280 && mode->yres == 960)
		sensor_mode = MT9M114_MODE_680x480;
	else {
		mt_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	mt_info("mt9m114: mt9m114_set_mode before write table");
	err = mt9m114_write_table(info->i2c_client, mode_table[sensor_mode],
		NULL, 0);
	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int mt9m114_get_status(struct mt9m114_info *info,
		struct mt9m114_status *dev_status)
{
	int err;

	err = mt9m114_write_reg(info->i2c_client, 0x98C, dev_status->data);
	if (err)
		return err;

	err = mt9m114_read_reg(info->i2c_client, 0x0990,
		(u16 *) &dev_status->status);
	if (err)
		return err;

	return err;
}

static long mt9m114_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct mt9m114_info *info = file->private_data;

	mt_info("%s, cmd : %d\n", __func__, cmd);

	switch (cmd) {
	case MT9M114_IOCTL_SET_MODE:
	{
		struct mt9m114_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct mt9m114_mode))) {
			return -EFAULT;
		}

		mt_info("mt9m114: mt9m114_ioctl : MT9M114_IOCTL_SET_MODE");
		return mt9m114_set_mode(info, &mode);
	}
//                                                                           
#if 1
	case MT9M114_IOCTL_SET_COLOR_EFFECT :
	{
		unsigned int color_effect;

		if (copy_from_user(&color_effect,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		//mutex_lock(&mt9m114_camera_lock);
		err = mt9m114_set_color_effect(info, color_effect);
		//mutex_unlock(&mt9m114_camera_lock);
		pr_info("    :MT9M114_IOCTL_SET_COLOR_EFFECT(%d)\n", color_effect);
		return err;

	}
	case MT9M114_IOCTL_SET_WHITE_BALANCE :
	{
		unsigned int white_balance;

		if (copy_from_user(&white_balance,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		//mutex_lock(&mt9m114_camera_lock);
        err = mt9m114_set_white_balance(info, white_balance);
		//mutex_unlock(&mt9m114_camera_lock);
		pr_info("    :MT9M114_IOCTL_SET_WHITE_BALANCE(%d)\n", white_balance);
		return err;

	}

	case MT9M114_IOCTL_SET_EXPOSURE:
	{
		ExposureValueType exposure;

		if (copy_from_user(&exposure,
					(const void __user *)arg,
					sizeof(ExposureValueType))) {
			return -EFAULT;
		}
		//mutex_lock(&mt9m114_camera_lock);
        err = mt9m114_set_exposure(info, &exposure);
		//mutex_unlock(&mt9m114_camera_lock);
		pr_info("    :MT9M114_IOCTL_SET_EXPOSURE(%d/%d)\n", exposure.index, exposure.range);
		return err;

	}

	case MT9M114_IOCTL_SET_FPSRANGE:
	{
		FpsRangeType FpsRange;

		if (copy_from_user(&FpsRange,
					(const void __user *)arg,
					sizeof(FpsRangeType))) {
			return -EFAULT;
		}
		//mutex_lock(&mt9m114_camera_lock);
        err = mt9m114_set_fpsrange(info, &FpsRange);
		//mutex_unlock(&mt9m114_camera_lock);
		pr_info("    :MT9M114_IOCTL_SET_FPSRANGE(%d:%d)\n", FpsRange.low, FpsRange.high);
		return err;
	}
    case MT9M114_IOCTL_SET_SCENE_MODE:
	{
		unsigned int scene_mode;

		if (copy_from_user(&scene_mode,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		//mutex_lock(&mt9m114_camera_lock);
		err = mt9m114_set_scene_mode(info, scene_mode);
		//mutex_unlock(&mt9m114_camera_lock);
		pr_info("    :MT9M114_IOCTL_SET_SCENE_MODE(%d)\n", scene_mode);
		return err;
	}
#endif
//                                                                           
	default:
		mt_info("mt9m114: mt9m114_ioctl : default");
		return -EINVAL;
	}
	return 0;
}

#if 1 //defined(CONFIG_MACH_LGHDK_REV_P3) || defined(MACH_X3_REV_C)
#define SUB_CAM_RESET_N     221 //TEGRA_GPIO_PBB5
#define VT_1V8V_EN          194 //TEGRA_GPIO_PY2
#endif

static int mt9m114_power_on(void)
{
	pr_info(" mt9m114_power_on\n");

    //IOVDD ->DVDD->AVDD->Reset
	subpm_set_gpio(1);

    // IOVDD 1.8v ON
	gpio_direction_output(VT_1V8V_EN, 1); //1.8V, VIF =>should be first
	gpio_set_value(VT_1V8V_EN, 1);
	udelay(1000);

    //DVDD 1.8V ON
	subpm_set_output(LDO1,1);//1.2V, VDIG =>should be 1.8V and should be second
	udelay(1000);
	//AVDD 2.8V ON
	subpm_set_output(LDO2,1);//2.7V, VANA=>should be third
	udelay(1000);

	//Reset low->high
	gpio_direction_output(SUB_CAM_RESET_N, 0);
    udelay(10);
	gpio_set_value(SUB_CAM_RESET_N, 0);
	udelay(10);
	gpio_direction_output(SUB_CAM_RESET_N, 1);
	udelay(10);
	gpio_set_value(SUB_CAM_RESET_N, 1);
    udelay(100);

	return 0;
}

static int mt9m114_power_off(void)
{
	pr_info(" mt9m114_power_off\n");

    //AVDD 2.8V OFF
    subpm_set_output(LDO2,0);
	udelay(400);
	//DVDD 1.8V OFF
    subpm_set_output(LDO1,0);
	udelay(400);
    //IOVDD 1.8V OFF
    gpio_set_value(VT_1V8V_EN, 0);
    udelay(10);
    gpio_direction_output(VT_1V8V_EN, 0);
	udelay(100);

    //Reset low
	gpio_set_value(SUB_CAM_RESET_N, 0);
    udelay(10);
	gpio_direction_output(SUB_CAM_RESET_N, 0);
    udelay(100);

	subpm_set_gpio(0);
	return 0;
}

static int mt9m114_open(struct inode *inode, struct file *file)
{
	struct mt9m114_status dev_status;
	int err;

	mt_info("mt9m114: mt9m114_ioctl : mt9m114_open");
//                                          
#if defined(CONFIG_VU_1_0_GLOBAL)
	pr_info("%s\n", __func__);
#endif
//                                          

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
        else 
                mt9m114_power_on();

	dev_status.data = 0;
	dev_status.status = 0;
	err = mt9m114_get_status(info, &dev_status);
	return err;
}

int mt9m114_release(struct inode *inode, struct file *file)
{
	mt_info("mt9m114: mt9m114_ioctl : mt9m114_release");
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	else
		mt9m114_power_off();

	file->private_data = NULL;
	return 0;
}

static const struct file_operations mt9m114_fileops = {
	.owner = THIS_MODULE,
	.open = mt9m114_open,
	.unlocked_ioctl = mt9m114_ioctl,
	.release = mt9m114_release,
};

static struct miscdevice mt9m114_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mt9m114",
	.fops = &mt9m114_fileops,
};

static int mt9m114_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	mt_info("mt9m114: probing sensor.\n");

	info = kzalloc(sizeof(struct mt9m114_info), GFP_KERNEL);
	if (!info) {
		mt_err("mt9m114: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&mt9m114_device);
	if (err) {
		mt_err("mt9m114: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);

	tegra_gpio_enable(VT_1V8V_EN);
	gpio_request(VT_1V8V_EN, "vt_1.8v_en");
	tegra_gpio_enable(SUB_CAM_RESET_N);
	gpio_request(SUB_CAM_RESET_N, "sub_cam_reset_n");

	return 0;
}

static int mt9m114_remove(struct i2c_client *client)
{
	struct mt9m114_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&mt9m114_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id mt9m114_id[] = {
	{ "mt9m114", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mt9m114_id);

static struct i2c_driver mt9m114_i2c_driver = {
	.driver = {
		.name = "mt9m114",
		.owner = THIS_MODULE,
	},
	.probe = mt9m114_probe,
	.remove = mt9m114_remove,
	.id_table = mt9m114_id,
};

static int __init mt9m114_init(void)
{
	mt_info("mt9m114 sensor driver loading\n");
	return i2c_add_driver(&mt9m114_i2c_driver);
}

static void __exit mt9m114_exit(void)
{
	i2c_del_driver(&mt9m114_i2c_driver);
}

module_init(mt9m114_init);
module_exit(mt9m114_exit);
