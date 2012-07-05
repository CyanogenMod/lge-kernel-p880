/*
 * drivers/input/touchscreen/tsc2007.c
 *
 * Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>

#include <linux/slab.h>
//#include <linux/input.h>


#include <linux/tsc2007_adc.h>

#define TS_POLL_DELAY			1 /* ms delay between samples */
#define TS_POLL_PERIOD			1 /* ms delay between samples */

#define TSC2007_MEASURE_TEMP0		(0x0 << 4)
#define TSC2007_MEASURE_AUX			(0x2 << 4)
#define TSC2007_MEASURE_TEMP1		(0x4 << 4)
#define TSC2007_ACTIVATE_XN			(0x8 << 4)
#define TSC2007_ACTIVATE_YN			(0x9 << 4)
#define TSC2007_ACTIVATE_YP_XN		(0xa << 4)
#define TSC2007_SETUP				(0xb << 4)
#define TSC2007_MEASURE_X			(0xc << 4)
#define TSC2007_MEASURE_Y			(0xd << 4)
#define TSC2007_MEASURE_Z1			(0xe << 4)
#define TSC2007_MEASURE_Z2			(0xf << 4)

#define TSC2007_POWER_OFF_IRQ_EN	(0x0 << 2)
#define TSC2007_ADC_ON_IRQ_DIS0		(0x1 << 2)
#define TSC2007_ADC_OFF_IRQ_EN		(0x2 << 2)
#define TSC2007_ADC_ON_IRQ_DIS1		(0x3 << 2)

#define TSC2007_12BIT				(0x0 << 1)
#define TSC2007_8BIT				(0x1 << 1)

#define	MAX_12BIT					((1 << 12) - 1)

#define ADC_ON_12BIT	(TSC2007_12BIT | TSC2007_ADC_ON_IRQ_DIS0)

#define READ_Y			(ADC_ON_12BIT | TSC2007_MEASURE_Y)
#define READ_Z1			(ADC_ON_12BIT | TSC2007_MEASURE_Z1)
#define READ_Z2			(ADC_ON_12BIT | TSC2007_MEASURE_Z2)
#define READ_X			(ADC_ON_12BIT | TSC2007_MEASURE_X)
#define PWRDOWN			(TSC2007_12BIT | TSC2007_POWER_OFF_IRQ_EN)

#define TSC_VDEF	1800 // mV

#define DEBUG_ADC		0

struct tsc2007_ADC_chip { 
	struct i2c_client					*client;
	struct delayed_work 				detect_work;
	struct tsc2007_ADC_platform_data	*pdata;
	struct power_supply					battery;

	u16	rev;
	u16 temp;
};

static struct max17043_chip* reference = NULL;

static int tsc2007_ADC_write(struct i2c_client *client, u8 cmd)
{
	int ret;
	struct i2c_msg msg;
	unsigned char data[1];
	
	if(DEBUG_ADC == 1) dev_info(&client->dev, "%s start..\n", __func__);
	data[0] = cmd;
	
	msg.addr = client->addr;
	msg.flags = I2C_M_IGNORE_NAK;
	msg.len = 1;
	msg.buf = data;
	if (DEBUG_ADC == 1) dev_info(&client->dev, "%s : cmd 0x%x\n",__func__, msg.buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_info(&client->dev, "i2c_transfer Write from device fails\n");
		return ret;
	}
	if(DEBUG_ADC == 1) dev_info(&client->dev, "%s end..\n", __func__);
	return 0;
}


//static int tsc2007_ADC_read(struct i2c_client *client, signed char *value)
static int tsc2007_ADC_read(struct i2c_client *client)

{
	struct i2c_msg msg[2];
	u8 data[2];
	int ret;
	u16 value = 0;
	
	if(DEBUG_ADC == 1) dev_info(&client->dev, "%s start..\n", __func__);

	msg[0].addr 	= client->addr;
	msg[0].flags 	= I2C_M_RD;
	msg[0].len  	= 2;
	msg[0].buf 		= &data[0];

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		dev_info(&client->dev, "i2c_transfer Read from device fails\n");
		return 0;
	}
	
	//dev_info(&client->dev, "%s : data[0] = 0x%x\n",__func__,data[0]);
	//dev_info(&client->dev, "%s : data[1] = 0x%x\n",__func__,data[1]);
	
	value = (data[0] << 8) | data[1];
	//dev_info(&client->dev, "%s : value = 0x%x\n",__func__,value);
	if(DEBUG_ADC == 1) dev_info(&client->dev, "%s end..\n", __func__);
	return value;
}

static int tsc2007_ADC_setup(struct i2c_client *client)
{
	int ret = 0;
	ret = tsc2007_ADC_write(client, TSC2007_SETUP);
	if (ret < 0) {
		dev_info(&client->dev, "%s : tsc2007_ADC_write fails..\n",__func__);
		return ret;
	}
	return 0;
}

static int tsc2007_ADC_cmd(struct i2c_client *client, u8 cmd)
{
	int ret = 0;
	
	ret = tsc2007_ADC_write(client, cmd);
	if (ret < 0) {
		dev_info(&client->dev, "%s : tsc2007_ADC_write fails..\n",__func__);
		return ret;
	}
	return 0;
}

static int tsc2007_ADC_rev(struct i2c_client *client)
{
	struct tsc2007_ADC_chip *chip = i2c_get_clientdata(client);
	
	int ret = 0;
	u16 value = 0;
	
	ret = tsc2007_ADC_write(client, TSC2007_MEASURE_X);
	if (ret < 0) {
		dev_info(&client->dev, "%s : tsc2007_ADC_write fails..\n",__func__);
		return ret;
	}

	value = tsc2007_ADC_read(client);
	if (value < 0) {
		dev_info(&client->dev, "%s : tsc2007_ADC_read fails..\n",__func__);
		return value;
	}
	if(DEBUG_ADC == 1) dev_info(&client->dev, "%s : value = 0x%x\n",__func__,value);

	chip->rev = value >> 4;
	dev_info(&client->dev, "%s : chip->rev = 0x%x\n",__func__,chip->rev);
	
	return 0;
}

static int tsc2007_ADC_temp(struct i2c_client *client)
{
	struct tsc2007_ADC_chip *chip = i2c_get_clientdata(client);
	
	int ret = 0;
	u16 value = 0;

	//                                                                                
#if 0 //defined (MACH_X3_REV_B)
		ret = tsc2007_ADC_write(client, TSC2007_MEASURE_Y);
#else
		ret = tsc2007_ADC_write(client, TSC2007_MEASURE_AUX);
#endif	

	if (ret < 0) {
		dev_info(&client->dev, "%s : tsc2007_ADC_write fails..\n",__func__);
		return ret;
	}

	value = tsc2007_ADC_read(client);
	if (value < 0) {
		dev_info(&client->dev, "%s : tsc2007_ADC_read fails..\n",__func__);
		return value;
	}
	if(DEBUG_ADC == 1) dev_info(&client->dev, "%s : value = 0x%x\n",__func__,value);

	value = value >> 4;
	//                                                    
	chip->temp = ((value*TSC_VDEF)/4096); //TSC base MAX Voltage is 4096mV
	//chip->temp = value / 10;
	dev_dbg(&client->dev, "%s : >>>>>>>>>>>>>>>>>>>>> >>> chip->temp = 0x%x\n",__func__,chip->temp);
	
	return 0;
}

static int tsc2007_ADC_Update(struct i2c_client *client)
{
	int ret = 0;
	
	//                                                                                         
	
	if (ret < 0) {
		dev_info(&client->dev, "%s : tsc2007_ADC_setup fails..\n",__func__);
		return ret;
	}
	//tsc2007_ADC_rev(client);
	tsc2007_ADC_temp(client);
	
	return 0;
}

static enum power_supply_property tsc2007_battery_props[] = {
	POWER_SUPPLY_PROP_TEMP_ADC,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
};
static int tsc2007_ADC_get_property(struct power_supply *psy,
	                                	   enum power_supply_property psp,
	                                	   union power_supply_propval *val)
{
	struct tsc2007_ADC_chip *chip = container_of(psy,
				struct tsc2007_ADC_chip, battery);
	
	tsc2007_ADC_Update(chip->client);
	switch (psp) {
		case POWER_SUPPLY_PROP_TEMP_ADC:
			val->intval = chip->temp;
			break;
		case POWER_SUPPLY_PROP_TEMP_AMBIENT:
			val->intval = chip->temp;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
static int __devinit tsc2007_ADC_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tsc2007_ADC_chip *chip;
	int ret = 0;
	
	dev_info(&client->dev, "%s.. start !!\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) { 
		dev_info(&client->dev, "%s i2c_check_functionality failed\n", __func__);
		return -EIO;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_info(&client->dev, "%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	chip->client 	= client;
	chip->pdata		= client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	chip->battery.name		= "tsc2007_adc";

	reference = chip;
	
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= tsc2007_ADC_get_property;
	chip->battery.properties	= tsc2007_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(tsc2007_battery_props);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto err_power_supply_register_failed;
	}
	
	tsc2007_ADC_Update(client);
	dev_info(&client->dev, "%s.. end !!\n", __func__);

	return 0;
err_power_supply_register_failed:
	kfree(chip);
	return ret;

}

static int __devexit tsc2007_ADC_remove(struct i2c_client *client)
{
	return 0;
}

static int tsc2007_ADC_suspend(struct device *dev)
{
	//struct max17043_chip *chip = dev_get_drvdata(dev);

	//cancel_delayed_work(&chip->work);
	//flush_work(&chip->alert_work);

	return 0;
}

static int tsc2007_ADC_resume(struct device *dev)
{
	//struct max17043_chip *chip = dev_get_drvdata(dev);

	return 0;
}

static const struct i2c_device_id tsc2007_ADC_idtable[] = {
	{ "tsc2007_adc", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tsc2007_ADC_idtable);

static struct i2c_driver tsc2007_ADC_driver = {
	.driver = {
		.name	= "tsc2007_adc",
		.owner	= THIS_MODULE,
	},
	.probe		= tsc2007_ADC_probe,
	.remove		= __devexit_p(tsc2007_ADC_remove),
	.suspend	= tsc2007_ADC_suspend,
	.resume		= tsc2007_ADC_resume,
	.id_table	= tsc2007_ADC_idtable,
	
	
};

static int __init tsc2007_ADC_init(void)
{
	return i2c_add_driver(&tsc2007_ADC_driver);
}

static void __exit tsc2007_ADC_exit(void)
{
	i2c_del_driver(&tsc2007_ADC_driver);
}

module_init(tsc2007_ADC_init);
module_exit(tsc2007_ADC_exit);

MODULE_AUTHOR("kipyo.cho <pyocool.cho@lge.com>");
MODULE_DESCRIPTION("TSC2007 ADC Driver");
MODULE_LICENSE("GPL");
