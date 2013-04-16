/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/video/backlight/lm3530_bl.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "../../../arch/arm/mach-tegra/gpio-names.h"

#define I2C_BL_NAME 	"lm3530_bl"

#define BL_ON		1
#define BL_OFF		0

#define LM3530_BL_REG_GENERAL_CONF		0x10
#define LM3530_BL_REG_BRIGHTNESS_RAMP_RATE	0x30	
#define LM3530_BL_REG_ALS_ZONE_INFORMATION	0x40	
#define LM3530_BL_REG_ALS_REGISTOR_SLECT	0x41
#define LM3530_BL_REG_BRIGHTNESS_CONTROL	0xA0
#define LM3530_BL_REG_ZB0			0x60
#define LM3530_BL_REG_ZB1			0x61
#define LM3530_BL_REG_ZB2			0x62
#define LM3530_BL_REG_ZB3			0x63	
#define LM3530_BL_REG_Z0T			0x70	
#define LM3530_BL_REG_Z1T			0x71	
#define LM3530_BL_REG_Z2T			0x72	
#define LM3530_BL_REG_Z3T			0x73	
#define LM3530_BL_REG_Z4T			0x74	

#define LM3530_BL_MAX_BRIGHTNESS		0x7F	// by data sheet
#define LM3530_BL_DEFAULT_BRIGHTNESS 	0x33 //	0x7F	// max brightness
#define CONFIG_LM3530_LEDS_CLASS	

#ifdef CONFIG_LM3530_LEDS_CLASS
#define LEDS_BACKLIGHT_NAME "lcd-backlight"
static void lm3530_bl_late_resume(struct early_suspend * h);
static void lm3530_bl_early_suspend(struct early_suspend * h);
#endif

struct lm3530_device {
	struct i2c_client	*client;
	struct backlight_device	*bl_dev;
#ifdef CONFIG_LM3530_LEDS_CLASS
	struct led_classdev *led;
#endif
	int			hwen_gpio;
	int 			max_current;
	int 			max_brightness;
	int 			min_brightness;
	int 			default_brightness;
	int			cur_main_lcd_level;
	int			saved_main_lcd_level;
	int			bl_status;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct lm3530_device *lm3530dev;
static int saved_level_for_ssd2825;
static const struct i2c_device_id lm3530_bl_id[] = {
	{ I2C_BL_NAME, 0 },
	{ },
};

static int lm3530_bl_write_reg(struct i2c_client *client,
			       unsigned char reg,
			       unsigned char val)
{
	int err;
	u8 buf[2];

	struct i2c_msg msg = {	
		client->addr, 0, 2, buf 
	};

	buf[0] = reg;
	buf[1] = val;
	
	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) {
		dev_err(&client->dev, "i2c write error\n");
	}
	
	return 0;
}


static void lm3530_hw_reset(struct lm3530_device *dev)
{
	int gpio = dev->hwen_gpio;
	printk("YJChae lm3530_hw_reset dev->bl_status : %d\n", dev->bl_status);

	if(dev->bl_status == BL_OFF){
		if (gpio_is_valid(gpio)) {
			gpio_direction_output(gpio, 1);
			gpio_set_value(gpio, 1);
			mdelay(2);
		}
	}
	else{
		if (gpio_is_valid(gpio)) {
			gpio_direction_output(gpio, 0);
			gpio_set_value(gpio, 0);
			mdelay(2);
		}
	}
}


static void lm3530_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm3530_device *dev = i2c_get_clientdata(client);
	int cal_value = 0;
	int min_brightness = dev->min_brightness;
	int max_current = dev->max_current;
	int gpio = dev->hwen_gpio;
	dev->cur_main_lcd_level = level; 
	printk("YJChae lm3530_set_main_current_level dev->bl_status : %d\n", dev->bl_status);

	dev->saved_main_lcd_level = dev->cur_main_lcd_level;
//	printk("YJChae lm3530_set_main_current_level  dev->saved_main_lcd_level=%d \n", dev->saved_main_lcd_level);

	if (level != 0) {
		gpio_set_value(gpio, 1);
#if 0 //YJChae for linear from I project
		cal_value = level * (LM3530_BL_MAX_BRIGHTNESS - min_brightness); 
		cal_value = cal_value / LM3530_BL_MAX_BRIGHTNESS + min_brightness;

		if (cal_value > LM3530_BL_MAX_BRIGHTNESS) 
			cal_value = LM3530_BL_MAX_BRIGHTNESS;
#else
		if(level < 24)
			cal_value = min_brightness;
		else if(level >= 24 && level < 29)
			cal_value = 0x10;
		else if(level >= 29 && level < 35)
			cal_value = 0x16;
		else if(level >= 35 && level < 43)
			cal_value = 0x1E;
		else if(level >= 43 && level < 47)
			cal_value = 0x28;
		else if(level >= 47 && level < 50)
			cal_value = 0x2d;
		else if(level >= 50 && level < 61)
			cal_value = 0x31;			
		else if(level >= 61 && level < 69)
			cal_value = 0x3d;
		else if(level >= 69 && level < 86)
			cal_value = 0x45;
		else if(level >= 86 && level < 113)
			cal_value = 0x56;	
		else if(level >= 113)
			cal_value = 0x71;
			
		if(cal_value > LM3530_BL_MAX_BRIGHTNESS)
			cal_value = LM3530_BL_MAX_BRIGHTNESS;
				
#endif
//		printk("YJChae lm3530_set_main_current_level  cal_value=%d \n", cal_value);

		dev_dbg(&client->dev, "max_current = %d, cal_value = %d\n",
				max_current, cal_value);

		lm3530_bl_write_reg(client, 
				LM3530_BL_REG_GENERAL_CONF, max_current);
		lm3530_bl_write_reg(client, 
				LM3530_BL_REG_BRIGHTNESS_CONTROL, cal_value);
	}
	else {
		lm3530_bl_write_reg(client, 
				LM3530_BL_REG_GENERAL_CONF, 0x00);		
	}	

	mdelay(1);
}

static void lm3530_backlight_on(struct i2c_client *client, int level)
{
	struct lm3530_device *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "backlight on...\n");
//	printk("YJChae lm3530_backlight_on dev->bl_status : %d\n", dev->bl_status);

	lm3530_hw_reset(dev);
//	printk("YJChae lm3530_backlight_on after lm3530_hw_reset dev->bl_status : %d\n", dev->bl_status);

	//printk("YJChae lm3530_backlight_on \n");
	lm3530_set_main_current_level(client, level);

	lm3530_bl_write_reg(client, LM3530_BL_REG_BRIGHTNESS_RAMP_RATE, 0x2d); //fade in, out


		
	dev->bl_status = BL_ON;

	return;
}

static void lm3530_backlight_off(struct i2c_client *client)
{
	struct lm3530_device *dev = i2c_get_clientdata(client);
	int gpio = dev->hwen_gpio;
//	printk("YJChae lm3530_backlight_off dev->bl_status : %d\n", dev->bl_status);

	if (dev->bl_status == BL_OFF) {
		dev_dbg(&client->dev, "Already turned off\n");
		return;
	}
	
	dev_dbg(&client->dev, "backlight off...\n");

	dev->saved_main_lcd_level = dev->cur_main_lcd_level;
	lm3530_set_main_current_level(client, 0);

	gpio_direction_output(gpio, 0);
	gpio_set_value(gpio, 0);
	
	dev->bl_status = BL_OFF;

	msleep(6);

	return;
}

static void lm3530_lcd_backlight_set_level(struct i2c_client *client, int level)
{
	if (level > LM3530_BL_MAX_BRIGHTNESS)
		level = LM3530_BL_MAX_BRIGHTNESS;

	if (client != NULL) {		
		if (level == 0) {
			lm3530_backlight_off(client);
		} 
		else {
			lm3530_backlight_on(client, level);
		}
	}
	else {
		printk("%s : No i2c client\n", __func__);
	}

	return;
}

static ssize_t lm3530_bl_show_lvl(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int r;
	struct i2c_client *client = to_i2c_client(dev); 
	struct lm3530_device *lm3530_dev = i2c_get_clientdata(client);

	r = snprintf(buf, PAGE_SIZE, 
			"LCD Backlight Level is : %d\n", 
			lm3530_dev->cur_main_lcd_level);
	
	return r;
}

static ssize_t lm3530_bl_store_lvl(struct device *dev,
					 struct device_attribute *attr, 
					 const char *buf, 
					 size_t count)
{
	int level;
	struct i2c_client *client = to_i2c_client(dev); 

	if (!count) {
		dev_err(&client->dev, "Invalid argument while storing level\n");
		return -EINVAL;
	}
	
	level = simple_strtoul(buf, NULL, 10);

	lm3530_lcd_backlight_set_level(client, level);
	
	return count;
}


static ssize_t lm3530_bl_show_on_off(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev); 
	struct lm3530_device *lm3530_dev = i2c_get_clientdata(client);
	int r;

	r = snprintf(buf, PAGE_SIZE, 
			"Previous Backlight Status : %s\n", 
			lm3530_dev->bl_status ? "On" : "Off");

	dev_dbg(&client->dev, "previous backlight_status : %s\n",
			lm3530_dev->bl_status ? "On" : "Off");

	return r;
}

struct device* lm3530_bl_dev(void)
{
	return &lm3530dev->client->dev;
}

int lm3530_bl_on_off(struct device *dev,int on_off)
{
	int ret=0;
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3530_device *lm3530_dev = i2c_get_clientdata(client);

	printk("%s *** on_off : %d , lm3530_dev->bl_status:%d , current level:%d, saved level:%d \n",
	__func__, on_off, lm3530_dev->bl_status,lm3530_dev->cur_main_lcd_level,lm3530_dev->saved_main_lcd_level);
	if ((on_off == 1) && (lm3530_dev->bl_status == BL_OFF)) {
		lm3530_backlight_on(client, saved_level_for_ssd2825);
		ret=2;
	}
	else if ((on_off == 0) && (lm3530_dev->bl_status == BL_ON)) {
		saved_level_for_ssd2825=lm3530_dev->cur_main_lcd_level;
		lm3530_backlight_off(client);
		ret=1;
	}
	else if ((on_off == 0) && (lm3530_dev->bl_status == BL_OFF)) ret=0;
	else if ((on_off == 1) && (lm3530_dev->bl_status == BL_ON)) ret=0;

	return ret;
}
EXPORT_SYMBOL(lm3530_bl_on_off);

static ssize_t lm3530_bl_store_on_off(struct device *dev, 
				      struct device_attribute *attr,
				      const char *buf, 
				      size_t count)
{
	int on_off;
	struct i2c_client *client = to_i2c_client(dev); 
	struct lm3530_device *lm3530_dev = i2c_get_clientdata(client);

	if (!count) {
		dev_err(&client->dev, "Invalide arguemnt..\n");
		return -EINVAL;
	}
	
	dev_dbg(&client->dev, "previous backlight_status: %s\n", 
			lm3530_dev->bl_status ? "On" : "Off");
	
	on_off = simple_strtoul(buf, NULL, 10);
	
	dev_dbg(&client->dev, "stored argument is %d", on_off);
	
	if (on_off == 1) {
		lm3530_backlight_on(client, lm3530_dev->saved_main_lcd_level);
	}
	else if (on_off == 0) {
		lm3530_backlight_off(client);
	}
	
	return count;

}
DEVICE_ATTR(lm3530_bl_level, 0664, lm3530_bl_show_lvl, lm3530_bl_store_lvl);
DEVICE_ATTR(lm3530_bl_on_off, 0666, lm3530_bl_show_on_off, lm3530_bl_store_on_off);

#ifdef CONFIG_LM3530_LEDS_CLASS

static int lm3530_bl_get_intensity(struct lm3530_device *drvdata)
{
	return drvdata->saved_main_lcd_level;
}

static void leds_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct lm3530_device *drvdata = dev_get_drvdata(led_cdev->dev->parent);
	int brightness;
	int next;

	if (!drvdata) {
		printk("Error getting drvier data\n");
		return;
	}

	brightness = lm3530_bl_get_intensity(drvdata);	
//	printk("YJChae leds_brightness_set brightness = %d \n" ,brightness);
	next = value;
	if (brightness != next) {
		lm3530_set_main_current_level(drvdata->client, next);
	}
}

static struct led_classdev lm3530_led_dev = {
	.name = LEDS_BACKLIGHT_NAME,
	.brightness_set = leds_brightness_set,
	.max_brightness = LED_FULL,
};
       
#else

static int lm3530_bl_set_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = to_i2c_client(bd->dev.parent);

	dev_dbg(&client->dev, "set_intensity = %d\n", bd->props.brightness);

	lm3530_lcd_backlight_set_level(client, bd->props.brightness);

	return 0;
}

static int lm3530_bl_get_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = to_i2c_client(bd->dev.parent);
	struct lm3530_device *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "get_intensity = %d\n", bd->props.brightness);
	dev_dbg(&client->dev, "dev->cur_main_lcd_level = %d\n", 
			dev->cur_main_lcd_level);

	return dev->cur_main_lcd_level;
}

static struct backlight_ops lm3530_bl_ops = { 
	.update_status	= lm3530_bl_set_intensity,
	.get_brightness	= lm3530_bl_get_intensity,
};
#endif

static int __init lm3530_bl_probe(struct i2c_client *i2c_dev,
			       const struct i2c_device_id *id)
{
	struct lm3530_bl_platform_data *pdata;
	struct lm3530_device *dev;
	int err;

	dev_info(&i2c_dev->dev, "Starting backlight device probing..\n");
	printk("YJChae lm3530_bl_probe : Starting backlight device probing..\n");

	pdata = i2c_dev->dev.platform_data;

	dev = kzalloc(sizeof(struct lm3530_device), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&i2c_dev->dev,"fail alloc for lm3530_device\n");

		return 0;
	}

#ifdef CONFIG_LM3530_LEDS_CLASS
	dev->client = i2c_dev;
	dev->hwen_gpio = pdata->hwen_gpio;
	dev->max_current= pdata->max_current;
	dev->max_brightness= LM3530_BL_MAX_BRIGHTNESS;
	dev->min_brightness= pdata->min_brightness;
	dev->default_brightness= LM3530_BL_DEFAULT_BRIGHTNESS;
	dev->cur_main_lcd_level= LM3530_BL_DEFAULT_BRIGHTNESS;
	dev->saved_main_lcd_level= LM3530_BL_DEFAULT_BRIGHTNESS;
	dev->bl_status = BL_OFF;

	if (led_classdev_register(&i2c_dev->dev, &lm3530_led_dev) == 0) {
		printk("Registering led class dev successfully.\n");
		dev->led = &lm3530_led_dev;
	}
#endif

	i2c_set_clientdata(i2c_dev, dev);
	tegra_gpio_enable(dev->hwen_gpio);
	if (dev->hwen_gpio && 
			gpio_request(dev->hwen_gpio, "lm3530 reset") != 0) {
		return -ENODEV;
	}
	gpio_direction_output(dev->hwen_gpio, 1);
	dev->bl_status = BL_ON; //YJChae
	//printk("YJChae lm3530_bl_probe dev->bl_status : %d\n", dev->bl_status);

	//lm3530_hw_reset(dev);
	//lm3530_backlight_on(i2c_dev, dev->default_brightness);

	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3530_bl_level);
	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3530_bl_on_off);
	lm3530dev=dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	dev->early_suspend.suspend = lm3530_bl_early_suspend;
	dev->early_suspend.resume = lm3530_bl_late_resume;
	dev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 40;
	register_early_suspend(&dev->early_suspend);
#endif

	return 0;
}

static int lm3530_bl_remove(struct i2c_client *i2c_dev)
{
	struct lm3530_device *dev = i2c_get_clientdata(i2c_dev);
	int gpio = dev->hwen_gpio;

 	device_remove_file(&i2c_dev->dev, &dev_attr_lm3530_bl_level);
 	device_remove_file(&i2c_dev->dev, &dev_attr_lm3530_bl_on_off);

	led_classdev_unregister(dev->led);

	if (gpio_is_valid(gpio)) {
		gpio_free(gpio);
	}

	i2c_set_clientdata(i2c_dev, NULL);

	return 0;
}

static int lm3530_bl_shutdown(struct i2c_client *i2c_dev)
{
	printk(KERN_INFO " ***** %s  \n", __func__);
	lm3530_backlight_off(i2c_dev);
	return 0;
}

#if defined(CONFIG_PM)
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3530_bl_late_resume(struct early_suspend * h)
{
	struct lm3530_device *dev = container_of(h, struct lm3530_device,
											 early_suspend);
	printk("%s start\n",__func__);
	
	lm3530_lcd_backlight_set_level(dev->client, dev->saved_main_lcd_level);
//	printk(KERN_INFO "YJChae %s: brightness=%d \n", __func__, dev->saved_main_lcd_level);
}

static void lm3530_bl_early_suspend(struct early_suspend * h)
{
	struct lm3530_device *dev = container_of(h, struct lm3530_device,
											 early_suspend);
	printk("%s start\n",__func__);
	lm3530_lcd_backlight_set_level(dev->client, 0);
}


#else
static int lm3530_bl_resume(struct device *dev)
{
	return 0;
}

static int lm3530_bl_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops lm3530_bl_pm_ops = {
	.suspend	= lm3530_bl_suspend,
	.resume		= lm3530_bl_resume,
};
#endif

#else	/*defined(CONFIG_PM)*/
#define lm3530_bl_resume	NULL
#define lm3530_bl_suspend	NULL
#endif	/*defined(CONFIG_PM)*/

static struct i2c_driver lm3530_driver = {
	.driver 	= {
		.name	= I2C_BL_NAME,
		.owner	= THIS_MODULE,
#if ! defined(CONFIG_PM)
		.pm	= &lm3530_bl_pm_ops,
#endif
	},
	.probe		= lm3530_bl_probe,
	.remove		= lm3530_bl_remove,
	.shutdown	= lm3530_bl_shutdown,
	.id_table	= lm3530_bl_id, 
};

static int __init lcd_backlight_init(void)
{
	return i2c_add_driver(&lm3530_driver);
}

static void __exit lm3530_exit(void)
{
	i2c_del_driver(&lm3530_driver);
}
 
module_init(lcd_backlight_init);

MODULE_DESCRIPTION("LM3530 Backlight Control");
MODULE_AUTHOR("Jaeseong Gim <jaeseong.gim@lge.com>");
MODULE_LICENSE("GPL");

