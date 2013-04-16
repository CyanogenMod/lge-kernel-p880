/*
 * imx119.c - imx119 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      chen.yingchun<chen.yingchun@lge.com>
 *
 * Leverage imx119.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 1280x1024. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 1280x1024
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/imx119.h>
#include <linux/gpio.h>
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <linux/regulator/lp8720.h>
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_set_gpio(int onoff);
#endif
struct imx119_reg {
	u16 addr;
	u16  val;
};

struct imx119_info {
	int mode;
	struct i2c_client *i2c_client;
	struct imx119_platform_data *pdata;
};

#define IMX119_DEBUG
#ifdef IMX119_DEBUG
	#define imx_info pr_info
#else
	#define imx_info(arg...) do {} while (0)
#endif

#define  imx_err pr_err

#define IMX119_TABLE_WAIT_MS 0
#define IMX119_TABLE_END 1
#define IMX119_MAX_RETRIES 3

static struct imx119_reg mode_1280x1024[] = {
{0x0100, 0x00},
{IMX119_TABLE_WAIT_MS, 100},
//PLL setting EXTCLK=12Mhz 396Mbps
{0x0305, 0x02},
{0x0307, 0x32},
{0x3025, 0x0A},
{0x302B, 0x4B},
{0x3022, 0x02}, //rear divider 1
//{IMX119_TABLE_WAIT_MS, 100}
//Global(Initial) Setting
{0x0112, 0x0A},
{0x0113, 0x0A},
//{0x0101, },
{0x301C, 0x02},
{0x302C, 0x85},
{0x303A, 0xA4},
{0x3108, 0x25},
{0x310A, 0x27},
{0x3122, 0x26},
{0x3138, 0x26},
{0x313A, 0x27},
{0x316D, 0x0A},
{0x308C, 0x00},
{0x302E, 0x8C},
{0x302F, 0x81},
//Mode Setting FULL(All Pixel)
{0x0340, 0x05}, //frame_length_lines
{0x0341, 0x26}, //frame_length_lines
{0x0342, 0x06}, //line_length
{0x0343, 0x10}, //line_length
{0x0346, 0x00}, //y_addr_start
{0x0347, 0x00}, //y_addr_start
{0x034A, 0x04}, //y_addr_end
{0x034B, 0x0F}, //y_addr_end
{0x034C, 0x05}, //x_output_size
{0x034D, 0x10}, //x_output_size
{0x034E, 0x04}, //y_output_size
{0x034F, 0x10}, //y_output_size
{0x0381, 0x01}, //x_even_inc
{0x0383, 0x01}, //x_odd_inc
{0x0385, 0x01}, //y_even_inc
{0x0387, 0x01}, //y_odd_inc
{0x3001, 0x00}, //HMODEADD
{0x3016, 0x02}, //VMODEADD
{0x3060, 0x30}, //CLPOWER_SMIA
{0x30E8, 0x00}, //HADDAVE
{0x3301, 0x01}, //RGCKREQSEL^M
{0x308A, 0x43}, //Low Power
{0x3305, 0x03}, //Mipi global timing
{0x3309, 0x05}, //Mipi global timing
{0x330B, 0x03}, //Mipi global timing
{0x330D, 0x05}, //Mipi global timing
//Streaming
{0x0100, 0x01},
{IMX119_TABLE_WAIT_MS, 100},

{IMX119_TABLE_END, 0x00}
};


static struct imx119_reg mode_144x176[] = {
{0x0100, 0x00},
{IMX119_TABLE_WAIT_MS, 100},
//PLL setting EXTCLK=24Mhz 396Mbps
{0x0305, 0x02},
{0x0307, 0x21},
{0x3025, 0x0A},
{0x302B, 0x4B},
{0x3022, 0x02}, //rear divider 1

//Global(Initial) Setting
{0x0112, 0x0A},
{0x0113, 0x0A},
//{0x0101, },
{0x301C, 0x02},
{0x302C, 0x85},
{0x303A, 0xA4},
{0x3108, 0x25},
{0x310A, 0x27},
{0x3122, 0x26},
{0x3138, 0x26},
{0x313A, 0x27},
{0x316D, 0x0A},
{0x308C, 0x00},
{0x302E, 0x8C},
{0x302F, 0x81},
  
//Mode Setting FULL(All Pixel)
{0x0340, 0x07}, //frame_length_lines
{0x0341, 0x68}, //frame_length_lines
{0x0344, 0x01}, //x_addr_start
{0x0345, 0x68}, //x_addr_start
{0x0346, 0x00}, //y_addr_start
{0x0347, 0xA8}, //y_addr_start
{0x0348, 0x03}, //x_addr_end
{0x0349, 0xA7}, //x_addr_end
{0x034A, 0x03}, //y_addr_end
{0x034B, 0x67}, //y_addr_end
{0x034C, 0x00}, //x_output_size
{0x034D, 0x90}, //x_output_size
{0x034E, 0x00}, //y_output_size
{0x034F, 0xB0}, //y_output_size
{0x0381, 0x05}, //x_even_inc
{0x0383, 0x03}, //x_odd_inc
{0x0385, 0x05}, //y_even_inc
{0x0387, 0x03}, //y_odd_inc
{0x3001, 0x80}, //HMODEADD
{0x3016, 0x42}, //VMODEADD
{0x3060, 0x30}, //CLPOWER_SMIA
{0x30E8, 0x80}, //HADDAVE
{0x3301, 0xC5}, //RGCKREQSEL^M
{0x308A, 0x11}, //Low Power
{0x3305, 0x00}, //Mipi global timing
{0x3309, 0x02}, //Mipi global timing
{0x330B, 0x00}, //Mipi global timing
{0x330D, 0x02}, //Mipi global timing

//Streaming
{0x0100, 0x01},
{IMX119_TABLE_WAIT_MS, 100},
{IMX119_TABLE_END, 0x00}

};



enum {
	IMX119_MODE_1280x1024,
  IMX119_MODE_144x176,
};

static struct imx119_reg *mode_table[] = {
	[IMX119_MODE_1280x1024] = mode_1280x1024,
  [IMX119_MODE_144x176] = mode_144x176,
};

static inline void imx119_get_frame_length_regs(struct imx119_reg *regs,
                                                u32 frame_length)
{
        regs->addr = 0x0340;
        regs->val = (frame_length >> 8) & 0x00ff;
        (regs + 1)->addr = 0x0341;
        (regs + 1)->val = (frame_length) & 0x00ff;
}

static inline void imx119_get_coarse_time_regs(struct imx119_reg *regs,
                                               u32 coarse_time)
{
        regs->addr = 0x0202;
        regs->val = (coarse_time >> 8) & 0x00ff;
        (regs + 1)->addr = 0x0203;
        (regs + 1)->val = (coarse_time) & 0x00ff;
}

static inline void imx119_get_gain_reg(struct imx119_reg *regs, u16 gain)
{
        regs->addr = 0x0205;
        regs->val = gain;
}

#if 0
static int imx119_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

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
		return -EINVAL;

	*val = data[2];

	return 0;
}
#endif

static int imx119_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
//		imx_info("imx119: i2c transfer under transferring %x %x\n", addr, val);
		if (err == 1)
			return 0;
		retry++;
		imx_err("imx119: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(3);
	} while (retry <= IMX119_MAX_RETRIES);

	return err;
}

static int imx119_write_table(struct i2c_client *client,
			      const struct imx119_reg table[],
			      const struct imx119_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct imx119_reg *next;
	int i;
	u16 val;

//	imx_info("imx119: imx119_write_table entered");
	for (next = table; next->addr != IMX119_TABLE_END; next++) {
//		imx_info("imx119: imx119_write_table 1");
		if (next->addr == IMX119_TABLE_WAIT_MS) {
//			imx_info("imx119: imx119_write_table : IMX119_TABLE_WAIT_MS ");
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

//		imx_info("imx119: imx119_write_table 2");
		err = imx119_write_reg(client, next->addr, val);
		if (err) {
			imx_err("imx119: imx119_write_table : err");
			return err;
		}
	}
	return 0;
}

static int imx119_set_mode(struct imx119_info *info, struct imx119_mode *mode)
{
	int sensor_mode;
	int err;

	imx_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
	if (mode->xres == 1280 && mode->yres == 1024)
		sensor_mode = IMX119_MODE_1280x1024;
  else if (mode->xres == 144 && mode->yres == 176)
		sensor_mode = IMX119_MODE_144x176;
	else {
		imx_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

//	imx_info("imx119: imx119_set_mode before write table");
	err = imx119_write_table(info->i2c_client, mode_table[sensor_mode],
		NULL, 0);
	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int imx119_get_status(struct imx119_info *info,
		struct imx119_status *dev_status)
{
	return 0;
}

static int imx119_set_frame_length(struct imx119_info *info, u32 frame_length, bool groupHold)
{
        struct imx119_reg reg_list[2];
        int i = 0;
        int ret;

//	imx_info("imx119: %s", __func__);

    imx119_get_frame_length_regs(reg_list, frame_length);
    imx_info("imx119: frame_length : %d ", frame_length);
    if (groupHold == false)
    {
        ret = imx119_write_reg(info->i2c_client, 0x0104, 0x0001);
        if (ret)
            return ret;
    }

    for (i = 0; i < 2; i++) {
        ret = imx119_write_reg(info->i2c_client, reg_list[i].addr,
        reg_list[i].val);
        if (ret)
            return ret;
    }

    if (groupHold == false)
    {
        ret = imx119_write_reg(info->i2c_client, 0x0104, 0x00000);
        if (ret)
            return ret;
    }

        return 0;
}

static int imx119_set_coarse_time(struct imx119_info *info, u32 coarse_time, bool groupHold)
{
	int ret;

	struct imx119_reg reg_list[3];
	int i = 0;

//	imx_info("imx119: %s", __func__);

	imx119_get_coarse_time_regs(reg_list, coarse_time);
	imx_info("imx119: coarse_time : %d ", coarse_time);

    if (groupHold == false)
    {
        ret = imx119_write_reg(info->i2c_client, 0x0104, 0x0001);
        if (ret)
            return ret;
    }

	for (i = 0; i < 3; i++)	{
		ret = imx119_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

    if (groupHold == false)
    {
	ret = imx119_write_reg(info->i2c_client, 0x0104, 0x00);
        if (ret)
        return ret;
    }

	return 0;
}

static int imx119_set_gain(struct imx119_info *info, u16 gain, bool groupHold)
{
	int ret;
	struct imx119_reg reg_list;

//	imx_info("imx119: %s", __func__);

	imx119_get_gain_reg(&reg_list, gain);

    if (groupHold == false)
    {
        ret = imx119_write_reg(info->i2c_client, 0x0104, 0x0001);
        if (ret)
           return ret;
    }

	ret = imx119_write_reg(info->i2c_client, reg_list.addr, reg_list.val);

    if (groupHold == false)
    {
        ret = imx119_write_reg(info->i2c_client, 0x0104, 0x00);
        if (ret)
            return ret;
    }
    
	return 0;
}

static int imx119_set_group_hold(struct imx119_info *info, struct imx119_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;

	if (count >= 2)
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = imx119_write_reg(info->i2c_client, 0x0104, 0x0001);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		imx119_set_gain(info, ae->gain, groupHoldEnabled);
	if (ae->coarse_time_enable)
		imx119_set_coarse_time(info, ae->coarse_time, groupHoldEnabled);
	if (ae->frame_length_enable)
		imx119_set_frame_length(info, ae->frame_length, groupHoldEnabled);

	if (groupHoldEnabled) {
		ret = imx119_write_reg(info->i2c_client, 0x0104, 0x0000);
		if (ret)
			return ret;
	}

	return 0;
}

static long imx119_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx119_info *info = file->private_data;
	
//	imx_info("imx119: imx119_ioctl ");

	switch (cmd) {
	case IMX119_IOCTL_SET_MODE:
	{
		struct imx119_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct imx119_mode))) {
			return -EFAULT;
		}

		imx_info("imx119: imx119_ioctl : IMX119_IOCTL_SET_MODE");
		return imx119_set_mode(info, &mode);
	}
	case IMX119_IOCTL_SET_FRAME_LENGTH:
		return imx119_set_frame_length(info, (u32)arg, false);
	case IMX119_IOCTL_SET_COARSE_TIME:
		return imx119_set_coarse_time(info, (u32)arg, false);
	case IMX119_IOCTL_SET_GAIN:
		return imx119_set_gain(info, (u16)arg, false);
	case IMX119_IOCTL_GET_STATUS:
	{
    u16 status = 0;
		imx_info("imx119: imx119_ioctl : IMX119_IOCTL_GET_STATUS");

		//err = imx119_get_status(info, &status);
    err = copy_to_user((void __user *)arg, &status,2);
    if (err) {
      pr_err("%s %d\n", __func__, __LINE__);
      return err;
    }
    return 0;    
	}
	case IMX119_IOCTL_SET_GROUP_HOLD:
	{
		struct imx119_ae ae;
		if (copy_from_user(&ae,
						(const void __user *)arg,
						sizeof(struct imx119_ae))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return imx119_set_group_hold(info, &ae);
	}
    default:
		imx_info("imx119: imx119_ioctl : default");
		return -EINVAL;
	}
	return 0;
}

static struct imx119_info *info;

#define SUB_CAM_RESET_N     221 //TEGRA_GPIO_PBB5
#define VT_1V8V_EN          194 //TEGRA_GPIO_PY2

static int imx119_power_on(void)
{

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
  subpm_set_gpio(1);

//1.2V
	subpm_set_output(LDO1,1);

//1.8V
  gpio_direction_output(VT_1V8V_EN, 1);
  gpio_set_value(VT_1V8V_EN, 1);
  udelay(10);

//2.7
	subpm_set_output(LDO2,1);
#endif

	gpio_direction_output(SUB_CAM_RESET_N, 1);
	udelay(10);
	gpio_set_value(SUB_CAM_RESET_N, 1);
	udelay(100);

	return 0;
}

static int imx119_power_off(void)
{
  gpio_set_value(SUB_CAM_RESET_N, 0);
  udelay(100);
  gpio_direction_output(SUB_CAM_RESET_N, 0);
  udelay(10);

  gpio_set_value(VT_1V8V_EN, 0);
  udelay(100);
  gpio_direction_output(VT_1V8V_EN, 0);

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
	subpm_set_output(LDO1,0);
	subpm_set_output(LDO2,0);
	udelay(10);

  subpm_set_gpio(0);
#endif
	return 0;
}

static int imx119_open(struct inode *inode, struct file *file)
{
	struct imx119_status dev_status;
	int err;

	imx_info("imx119: imx119_ioctl : imx119_open");

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	else 
		imx119_power_on();

	dev_status.data = 0;
	dev_status.status = 0;
	err = imx119_get_status(info, &dev_status);
	return err;
}

int imx119_release(struct inode *inode, struct file *file)
{

	imx_info("imx119_release ver 1.2");
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	else 
		imx119_power_off();

	file->private_data = NULL;
	return 0;
}

static const struct file_operations imx119_fileops = {
	.owner = THIS_MODULE,
	.open = imx119_open,
	.unlocked_ioctl = imx119_ioctl,
	.release = imx119_release,
};

static struct miscdevice imx119_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx119",
	.fops = &imx119_fileops,
};

static int imx119_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	imx_info("imx119: probing sensor.\n");

	info = kzalloc(sizeof(struct imx119_info), GFP_KERNEL);
	if (!info) {
		imx_err("imx119: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&imx119_device);
	if (err) {
		imx_err("imx119: Unable to register misc device!\n");
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

static int imx119_remove(struct i2c_client *client)
{
	struct imx119_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&imx119_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id imx119_id[] = {
	{ "imx119", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, imx119_id);

static struct i2c_driver imx119_i2c_driver = {
	.driver = {
		.name = "imx119",
		.owner = THIS_MODULE,
	},
	.probe = imx119_probe,
	.remove = imx119_remove,
	.id_table = imx119_id,
};

static int __init imx119_init(void)
{
	imx_info("imx119 sensor driver loading\n");
	return i2c_add_driver(&imx119_i2c_driver);
}

static void __exit imx119_exit(void)
{
	i2c_del_driver(&imx119_i2c_driver);
}

module_init(imx119_init);
module_exit(imx119_exit);
