/*
* ST M24C08 E2PROM driver.
*
* Copyright (C) 2012 NVIDIA Inc.
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
#include <media/m24c08.h>
#include <linux/gpio.h>
//#include <mach/bssq_cam_pmic.h>
#if 1 //_jkr
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <linux/regulator/lp8720.h>
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_set_gpio(int onoff);
#endif
#endif

#define DEBUG_EEPROM_DATA   (0)
#define EEPROM_PAGE_SIZE (256)
#define EEPROM_WP		83 //                                                                                                     
#define EEPROM_WP_ON		1
#define EEPROM_WP_OFF		0
#define MAX_RETRIES 3

static struct m24c08_info *info = NULL;

/*
This EEPROM has 1K capacity.
The range of reg is from 0 ~ 1023(10bits resolution).
 reg: represents the register address from 0 ~ 1023
 length: 
*/
static int m24c08_read(struct i2c_client *client, u16 reg, u16 length, u8 *data)
{
	u16 reg_addr = reg;
	u16 reg_cnt;
	u8 page_num;
	u8 remainder_reg;
	
	struct i2c_msg msg[2] = {
		{ .flags = 0, .len = 1 },
		{ .flags = I2C_M_RD, .len = 1 },
	};
	if (!client->adapter)
		return -ENODEV;

	for(reg_cnt = 0; reg_cnt < length; reg_cnt ++)
	{
		page_num = ((reg_addr+reg_cnt) / EEPROM_PAGE_SIZE) & (0x03);
		remainder_reg = ((reg_addr+reg_cnt) % EEPROM_PAGE_SIZE) & 0xff;

		msg[0].addr = (client->addr | page_num);
		msg[0].buf = 	&remainder_reg;
		
		msg[1].addr = (client->addr | page_num);
		msg[1].buf = &data[reg_cnt];
		if(i2c_transfer(client->adapter, msg, 2) != 2)
		{
			pr_err("m24c08(R): i2c transfer failed\n");
			return -EIO;
		}
#if DEBUG_EEPROM_DATA
		printk("%s page_num:%x, remainder:%x, addr[0x%x]=>[0x%x]\n", \
			__func__, page_num, remainder_reg, msg[0].addr, data[reg_cnt]);		
#endif // DEBUG_EEPROM_DATA
	}
	return 0;
}

static int m24c08_write(struct i2c_client *client, u16 reg, u16 length, u8 *data)
{
#if 0 //we do not need to write to eeprom
	u16 reg_addr = reg;
	u16 reg_cnt;
	u8 page_num = (reg_addr / EEPROM_PAGE_SIZE) & 0x03;
	u8 msg_data[2];
	int retry;
	
	struct i2c_msg msg[1] = {
		{ .flags = 0, .buf = msg_data, .len = 2 },
	};
	if (!client->adapter)
		return -ENODEV;

	for(reg_cnt = 0; reg_cnt < length; reg_cnt ++)
	{
		retry = 0;
		page_num = ((reg_addr+reg_cnt) / EEPROM_PAGE_SIZE) & 0x03;

		msg[0].addr = (client->addr | page_num);
		msg_data[0] = ((reg_addr+reg_cnt) % EEPROM_PAGE_SIZE) & 0xff;
		msg_data[1] = data[reg_cnt];

		do {
			if(i2c_transfer(client->adapter, msg, 1) == 1)
				break;
			msleep(3);
			retry++;
#if DEBUG_EEPROM_DATA
			pr_err("m24c08: i2c transfer failed, retrying 0x%x 0x%x\n",
			       msg[0].addr, msg_data[1]);
#endif
		} while (retry < MAX_RETRIES);
		if(retry == MAX_RETRIES)
		{
			pr_err("m24c08(W): i2c transfer failed at(page:%d,reg_addr:%d)\n",
					page_num, reg_addr+reg_cnt);
			return -EIO;
		}
#if 0
		u8 read_data;
		if(m24c08_read(client, reg_addr+reg_cnt, 1, &read_data) == 0)
		{
			if(read_data != data[reg_cnt])
			{
				pr_err("Error: (%d)data doesn't match!!\n", reg_addr+reg_cnt);			
			}
		}
#endif
	}
#endif //we do not need to write to eeprom
	return 0;
}

static long m24c08_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct m24c08_info *info = (struct m24c08_info *)file->private_data;

	switch (cmd) {
	case M24C08_IOCTL_GET_E2PROM_DATA:
	{
#if DEBUG_EEPROM_DATA
		pr_info("%s: M24C08_IOCTL_GET_E2PROM_DATA: i2c client(name:%s, addr:0x%08x, flag:0x%08x)\n", __func__, info->i2c_client->name, info->i2c_client->addr, info->i2c_client->flags);
#endif
		if (copy_from_user(&info->reg_info,
			(const void __user *)arg,
			sizeof(struct m24c08_register_info)))
		{
			pr_err("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		
		if(info->reg_info.reg_addr < 0 || info->reg_info.reg_addr > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(r1) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		if((info->reg_info.reg_addr +  info->reg_info.length) < 1 || (info->reg_info.reg_addr +  info->reg_info.length - 1) > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(r2) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		
#if DEBUG_EEPROM_DATA
		printk("[0] %s: Reading data from user level successful\n", __func__);
#endif
		if(m24c08_read(info->i2c_client,
			info->reg_info.reg_addr,
			info->reg_info.length,
			info->reg_info.e2prom_data) == 0)
		{
#if DEBUG_EEPROM_DATA
			printk("[1] %s: Reading success: 0x%0x,\
				transfer data to user level\n", __func__, info->reg_info.e2prom_data[0]);
#endif
			if (copy_to_user((void __user *) arg,
				 			&info->reg_info,
							sizeof(struct m24c08_register_info)))
			{
				pr_err("%s: 0x%x\n", __func__, __LINE__);
				return -EFAULT;
			}
#if DEBUG_EEPROM_DATA
			printk("[2] %s: Transfer success!!\n", __func__);
#endif

		}
		else
		{
			pr_err("%s M24C08_IOCTL_GET_E2PROM_DATA Failure\n", __func__);
			return -EFAULT;
		}
		break;
	}
	case M24C08_IOCTL_PUT_E2PROM_DATA:
	{
#if DEBUG_EEPROM_DATA
		pr_info("%s: M24C08_IOCTL_PUT_E2PROM_DATA\n", __func__);
#endif
		if (copy_from_user(&info->reg_info,
			(const void __user *)arg,
			sizeof(struct m24c08_register_info)))
		{
			pr_err("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		if(info->reg_info.reg_addr < 0 || info->reg_info.reg_addr > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(w1) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		if((info->reg_info.reg_addr +  info->reg_info.length) < 1 || (info->reg_info.reg_addr +  info->reg_info.length - 1) > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(w2) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		
#if DEBUG_EEPROM_DATA
		printk("[0] %s: (W)Writing data from user level successful\n", __func__);
#endif
		if(m24c08_write(info->i2c_client,
			info->reg_info.reg_addr,
			info->reg_info.length,
			info->reg_info.e2prom_data) == 0)
		{
#if DEBUG_EEPROM_DATA
			printk("[1] %s: (W)Writing is successful\n", __func__);
#endif
		}
		else
		{
			return -EFAULT;
		}
		break;
	}	
	default:
		printk("%s: 0x%x DEFAULT IOCTL\n", __func__, __LINE__);	
		return -EINVAL;
	}
	return 0;
}



static int m24c08_open(struct inode *inode, struct file *file)
{
	pr_info("m24c08: open!\n");

#if 1 //_jkr
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
  //pr_info("LP8720 CamPMIC Main Cam Power ON ++\n");
  subpm_set_gpio(1);

  subpm_set_output(LDO3,1);
  subpm_set_output(SWREG,1);
  subpm_set_output(LDO4,1);

#endif
#endif

	gpio_set_value(EEPROM_WP, EEPROM_WP_OFF);
	file->private_data = (struct file *)info;


	pr_info("%s: called\n", __func__);
	return 0;
}

int m24c08_release(struct inode *inode, struct file *file)
{
	pr_info("m24c08: release!\n");
	gpio_set_value(EEPROM_WP, EEPROM_WP_ON);
	file->private_data = NULL;

#if 1 //_jkr
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
        //pr_info("LP8720 CamPMIC Main Cam Power OFF ++\n");
    
        subpm_set_output(LDO4,0);
        subpm_set_output(SWREG,0);
        subpm_set_output(LDO3,0);
        udelay(100);
        //                                                                                                   
#endif
#endif

	return 0;
}


static const struct file_operations m24c08_fileops = {
	.owner = THIS_MODULE,
	.open = m24c08_open,
	.unlocked_ioctl = m24c08_ioctl,
	.release = m24c08_release,
};

static struct miscdevice m24c08_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "m24c08",
	.fops = &m24c08_fileops,
};

static int m24c08_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("m24c08: probing sensor.\n");

	info = kzalloc(sizeof(struct m24c08_info), GFP_KERNEL);
	if (!info) {
		pr_err("m24c08: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&m24c08_device);
	if (err) {
		pr_err("m24c08: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	err = gpio_request(EEPROM_WP, "eeprom_wp");
	if (err) {
		pr_err("*** m24c08: gpio_request failed for eeprom_wp\n");
		kfree(info);
		return err;
	}
	tegra_gpio_enable(EEPROM_WP);
	gpio_direction_output(EEPROM_WP,1);
	gpio_set_value(EEPROM_WP, EEPROM_WP_ON);
	mdelay(5);
	gpio_set_value(EEPROM_WP, EEPROM_WP_OFF);

	info->i2c_client = client;
	i2c_set_clientdata(client, info);

	return 0;
}

static int m24c08_remove(struct i2c_client *client)
{
	struct m24c08_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&m24c08_device);

	if(info)
		kfree(info);
	return 0;
}

static const struct i2c_device_id m24c08_id[] = {
	{ "m24c08", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, m24c08_id);

static struct i2c_driver m24c08_i2c_driver = {
	.driver = {
		.name = "m24c08",
		.owner = THIS_MODULE,
	},
	.probe = m24c08_probe,
	.remove = m24c08_remove,
	.id_table = m24c08_id,
};

static int __init m24c08_init(void)
{
	pr_info("m24c08 sensor driver loading\n");
	i2c_add_driver(&m24c08_i2c_driver);

	return 0;
}

static void __exit m24c08_exit(void)
{
	i2c_del_driver(&m24c08_i2c_driver);
}

module_init(m24c08_init);
module_exit(m24c08_exit);
