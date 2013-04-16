/*
 * linux/drivers/misc/tspdrv/tspdrv_i2c.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 * Author: Euikyeom Kim <euikyeom.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm.h>

#include <linux/tspdrv_i2c.h>

static struct i2c_client *tspdrv_i2c_client = NULL;
struct pwm_device* tspdrv_pwm = NULL;
int tspdrv_gpio_en = -1;

s32 tspdrv_i2c_read_block_data(u8 command, u8 length, u8 *values)
{
	s32 status;
	status = i2c_smbus_read_i2c_block_data(tspdrv_i2c_client, command, length, values);

	if (status < 0)
	{
		printk("%s : I2C read block fail ,  RECOVER !!\n",__func__);
	}

	return status;
}

s32 tspdrv_i2c_read_byte_data(u8 command)
{
	s32 status;
	status = i2c_smbus_read_byte_data(tspdrv_i2c_client, command);

	if (status < 0)
	{
		printk("%s : I2C read byte fail ,  RECOVER !!\n",__func__);
	}
	
	return status;
}

s32 tspdrv_i2c_write_block_data(u8 command, u8 length, const u8 *values)
{
	s32 status;
	
	status = i2c_smbus_write_block_data(tspdrv_i2c_client , command, length, values );
//	printk("***%s : I2C write block status : %d , val[0] :  0x%x, val[1] : 0x%x, ***\n",__func__,status, values[0], values[1]);
	if (status < 0)
	{
		printk("%s : I2C write block fail val[0] : 0x%x, val[1] : 0x%x,  RECOVER !!(%d)\n",__func__, values[0], values[1]);
	}
	
	return status;
}
s32 tspdrv_i2c_write_byte_data(u8 command, u8 value)
{
	s32 status;
	
	status = i2c_smbus_write_byte_data(tspdrv_i2c_client, command, value); /* wake up */
	if (status < 0)
	{
		printk("%s : I2C write byte fail ,  RECOVER !!\n",__func__);
	}
	
	return status;
}

void tspdrv_control_vibrator(u8 onoff)
{
	if(onoff)
	{
		gpio_set_value(tspdrv_gpio_en, 1);
	}
	else
	{
		gpio_set_value(tspdrv_gpio_en, 0);
	}
}

void tspdrv_control_pwm(u8 onoff, int duty_ns, int period_ns)
{
	if(onoff)
	{
	   	pwm_config(tspdrv_pwm, duty_ns, period_ns);		
//		printk("====================pwm_enable====\n");

		pwm_enable(tspdrv_pwm);
	}
	else
	{
		pwm_disable(tspdrv_pwm);
	}
	
}

static int tspdrv_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int ret = 0;	
	struct i2c_adapter *adapter = client->adapter;
	struct tspdrv_i2c_platform_data *pdata;

	printk("tspdrv_i2c_probe !!!\n");
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	if (i2c_get_clientdata(client))
		return -EBUSY;
	
	tspdrv_i2c_client = client;
	pdata = client->dev.platform_data;

	ret = gpio_request(pdata->en_gpio, "vib_en");
	if (ret < 0)
	{
		goto err_gpio;
	}

	ret=gpio_direction_output(pdata->en_gpio, 0);
	if (ret < 0)
	{
		gpio_free(pdata->en_gpio);
		goto err_gpio;
	}
	else
		tegra_gpio_enable(pdata->en_gpio);

	tspdrv_gpio_en = pdata->en_gpio;

	tspdrv_pwm	=	pwm_request(pdata->pwm_id, "vibrator");
	if (IS_ERR(tspdrv_pwm)) {
		goto	err_pwm;
	}

	pwm_disable(tspdrv_pwm);

	return ret;
	
err_check_functionality_failed: 
	printk("tspdrv_i2c I2C error!!\n");
	return ret;
err_gpio:	
	printk("tspdrv_i2c GPIO error!!\n");	
	return ret;
err_pwm:
	printk("tspdrv_i2c PWM error!!\n");	
	return ret;

}

static int tspdrv_i2c_remove(struct i2c_client *client)
{
}

#ifdef CONFIG_PM
static int tspdrv_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int tspdrv_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define tspdrv_i2c_suspend	NULL
#define tspdrv_i2c_resume	NULL
#endif /* CONFIG_PM */
static struct i2c_device_id tspdrv_i2c_id_table[] = {
	{ TSPDRV_I2C_DEVICE_NAME, 4 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tspdrv_i2c_id_table);

static struct i2c_driver tspdrv_i2c_driver = {
	.driver = {
		.name	= TSPDRV_I2C_DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= tspdrv_i2c_probe,
	.remove		= tspdrv_i2c_remove,
	.suspend	= tspdrv_i2c_suspend,
	.resume		= tspdrv_i2c_resume,
	.id_table	= tspdrv_i2c_id_table,
};

static int __init tspdrv_i2c_init(void)
{
	return i2c_add_driver(&tspdrv_i2c_driver);
}

static void __exit tspdrv_i2c_exit(void)
{
	i2c_del_driver(&tspdrv_i2c_driver);
}

subsys_initcall(tspdrv_i2c_init);
module_exit(tspdrv_i2c_exit);

MODULE_DESCRIPTION("Core support for tspdrv_i2c");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jin Park <jinyoungp@nvidia.com>");
