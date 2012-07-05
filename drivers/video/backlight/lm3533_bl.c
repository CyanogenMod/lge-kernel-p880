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
#include <linux/video/backlight/lm3533_bl.h>

#include "../../../arch/arm/mach-tegra/gpio-names.h"
#include "../../../arch/arm/mach-tegra/board.h"
#include "../../misc/muic/muic.h"

#define I2C_BL_NAME 	"lm3533_bl"

#define BL_ON		1
#define BL_OFF		0

#define LM3533_BL_MAX_BRIGHTNESS	0xFF // 0x7F	// by data sheet
#define LM3533_BL_DEFAULT_BRIGHTNESS 	0x33 // 0x7F	// max brightness
#define LM3533_BL_MIN_BRIGHTNESS	0x02 // 0x7F	// by data sheet

#define CONFIG_LM3533_LEDS_CLASS	

#ifdef CONFIG_LM3533_LEDS_CLASS
#define LEDS_BACKLIGHT_NAME 		"lcd-backlight"
#endif

static unsigned char default_160nit_53percent_lut[256]={
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,
1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,3,3,4,
4,4,4,5,5,5,5,6,6,6,6,7,7,7,8,8,8,9,9,9,
10,10,10,11,11,11,12,12,13,13,13,14,14,15,15,16,16,17,17,17,
18,18,19,19,20,21,21,22,22,23,23,24,24,25,26,26,27,27,28,29,
29,30,31,31,32,33,33,34,35,35,36,37,38,38,39,40,41,41,42,43,
44,45,45,46,47,48,49,50,50,51,52,53,54,55,56,57,58,59,60,61,
61,62,63,64,65,66,67,68,69,71,72,73,74,75,76,77,78,79,80,81,
82,84,85,86,87,88,89,91,92,93,94,95,97,98,99,100,102,103,104,106,
107,108,110,111,112,114,115,116,118,119,120,122,123,125,126,127,129,130,132,133,
135,136,138,139,141,142,144,145,147,148,150,152,153,155,156,158,160,161,163,165,
166,168,170,171,173,175,176,178,180,182,183,185,187,189,190,192,194,196,198,199,
201,203,205,207,209,211,213,214,216,218,220,222,224,226,228,230,
};//default value : adb brightness 146(53%) -> bl driver 67, Max brightness:90%
struct lm3533_device {
	struct i2c_client	*client;
	struct backlight_device	*bl_dev;
#ifdef CONFIG_LM3533_LEDS_CLASS
	struct led_classdev 	*led;
#endif
	int			hwen_gpio;
	int 			max_current;
	int 			max_brightness;
	int 			min_brightness;
	int 			default_brightness;
	int 			dimming_brightness;
	int			cur_main_lcd_level;
	int			saved_main_lcd_level;
	int			bl_status;
	struct mutex 		bl_mutex;
	struct mutex 		bl_mutex_saved;
};

struct lm3533_device *lm3533dev;
static int lm3533_driver_ready = 0;

static const struct i2c_device_id lm3533_bl_id[] = {
	{ I2C_BL_NAME, 0 },
	{ },
};

static int lm3533_bl_write_reg(struct i2c_client *client,
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


static void lm3533_hw_reset(struct lm3533_device *dev)
{
	int gpio = dev->hwen_gpio;
	//printk("lm3533_hw_reset dev->bl_status : %d\n", dev->bl_status);

	if(dev->bl_status == BL_OFF){
		if (gpio_is_valid(gpio)) {
			gpio_direction_output(gpio, 1);
			gpio_set_value(gpio, 1);
			mdelay(2);
		}
	}
}


static void lm3533_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm3533_device *dev = i2c_get_clientdata(client);
	int cal_value = 0;
	int min_brightness = dev->min_brightness;
	int max_brightness = dev->max_brightness;
	int max_current = dev->max_current;
	int gpio = dev->hwen_gpio;

	pr_debug("lm3533_level[BL]:%d \n", level);

	//dev->cur_main_lcd_level = level;
	//dev->saved_main_lcd_level = dev->cur_main_lcd_level;

	mutex_lock(&lm3533dev->bl_mutex);

	if(level!= 0){
		gpio_set_value(gpio, 1);
		//cal_value = level;
		/*if(level >= 1 && level <= 255)
		{
			cal_value = (((level*2250-30*2250)/225)*251 + 4*2250)/2250; //input min=30, Max255,,, Output min 20 ; max 255
			if(cal_value < 165)
				cal_value = (cal_value*108)/100 - 38;
			else
				cal_value = (cal_value*150)/100 - 100;
		}
		else{
			mdelay(1);
			return;
		}*/
		if (level < 0)	level=0;
		if (level >255)	level=255;

		cal_value=default_160nit_53percent_lut[level];

		if (cal_value < LM3533_BL_MIN_BRIGHTNESS){
			cal_value = LM3533_BL_MIN_BRIGHTNESS;
		}
		else if (cal_value > LM3533_BL_MAX_BRIGHTNESS)
		{
			cal_value = LM3533_BL_MAX_BRIGHTNESS;
		}
#if defined(CONFIG_MACH_X3)
		//jude84.kim For Milky U and Factorymode
		if((is_tegra_batteryexistWhenBoot() != 1) && (charging_mode == CHARGING_FACTORY)){
			cal_value = LM3533_BL_MIN_BRIGHTNESS;
		}
#endif
		lm3533_bl_write_reg(client, 0x40, cal_value);
	}
	else if(level == 0){
		cal_value = level;
		lm3533_bl_write_reg(client, 0x40, cal_value);
	}
	mutex_unlock(&lm3533dev->bl_mutex);
	mdelay(1);
}

static void lm3533_backlight_on(struct i2c_client *client, int level)
{
	struct lm3533_device *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "backlight on...\n");

	lm3533_hw_reset(dev);

	lm3533_bl_write_reg(client, 0x10, 0x0); //HVLED 1 & 2 are controlled by Bank A

#if defined(ENABLE_PWM_SIGNAL)        
	lm3533_bl_write_reg(client, 0x14, 0x1); //                                                                 
#else
	lm3533_bl_write_reg(client, 0x14, 0x0); //PWM input is disabled for CABC
#endif
	lm3533_bl_write_reg(client, 0x1A, 0x2); //Linear & Control Bank A is configured for register Current control
	lm3533_bl_write_reg(client, 0x1F, 0x13); //Full-Scale Current (20.2mA)
	lm3533_bl_write_reg(client, 0x27, 0x1); //Control Bank A is enable
#if !defined(CONFIG_MACH_HD5_LGD)        
	lm3533_bl_write_reg(client, 0x2C, 0x0A); //                                                                                   
#else
	lm3533_bl_write_reg(client, 0x2C, 0x0C); //Active High, OVP(24V), Boost Frequency(500 khz)  
#endif

	lm3533_set_main_current_level(client, level);

	//                                                                                          
		
	dev->bl_status = BL_ON;

	return;
}

static void lm3533_backlight_off(struct i2c_client *client)
{
	struct lm3533_device *dev = i2c_get_clientdata(client);
	int gpio = dev->hwen_gpio;

	if (dev->bl_status == BL_OFF) {
		dev_dbg(&client->dev, "Already turned off\n");
		return;
	}

	dev_dbg(&client->dev, "backlight off...\n");

	//dev->saved_main_lcd_level = dev->cur_main_lcd_level;
	lm3533_set_main_current_level(client, 0);

	gpio_set_value(gpio, 0);
	
	dev->bl_status = BL_OFF;

	mdelay(10);

	return;
}

static void lm3533_lcd_backlight_set_level(struct i2c_client *client, int level)
{
	if (level > LM3533_BL_MAX_BRIGHTNESS)
		level = LM3533_BL_MAX_BRIGHTNESS;

	if (client != NULL) {		
		if (level == 0) {
			lm3533_backlight_off(client);
		} 
		else {
			lm3533_backlight_on(client, level);
		}
	}
	else {
		printk("%s : No i2c client\n", __func__);
	}

	return;
}

static ssize_t lm3533_bl_show_lvl(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int r;
	struct i2c_client *client = to_i2c_client(dev); 
	struct lm3533_device *lm3533_dev = i2c_get_clientdata(client);

	r = snprintf(buf, PAGE_SIZE, 
			"LCD Backlight Level is : %d\n", 
			lm3533_dev->cur_main_lcd_level);
	
	return r;
}

static ssize_t lm3533_bl_store_lvl(struct device *dev,
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

	lm3533_lcd_backlight_set_level(client, level);
	
	return count;
}


static ssize_t lm3533_bl_show_on_off(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev); 
	struct lm3533_device *lm3533_dev = i2c_get_clientdata(client);
	int r;

	r = snprintf(buf, PAGE_SIZE, 
			"Previous Backlight Status : %s\n", 
			lm3533_dev->bl_status ? "On" : "Off");

	dev_dbg(&client->dev, "previous backlight_status : %s\n",
			lm3533_dev->bl_status ? "On" : "Off");

	return r;
}

struct device* lm3533_bl_dev(void)
{
	if(lm3533dev != NULL && lm3533dev->client != NULL)	return &lm3533dev->client->dev;
	return NULL;
}
int lm3533_is_ready(void)
{
	return lm3533_driver_ready;
}

int lm3533_bl_on_off(struct device *dev,int on_off)
{
	int ret=0;
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3533_device *lm3533_dev = i2c_get_clientdata(client);

	//printk("%s *** on_off : %d , lm3533_dev->bl_status:%d , current level:%d, saved level:%d \n",
	//__func__, on_off, lm3533_dev->bl_status,lm3533_dev->cur_main_lcd_level,lm3533_dev->saved_main_lcd_level);
	if ((on_off == 1) && (lm3533_dev->bl_status == BL_OFF)) {
		mutex_lock(&lm3533dev->bl_mutex_saved);
		lm3533_backlight_on(client, lm3533_dev->saved_main_lcd_level);
		mutex_unlock(&lm3533dev->bl_mutex_saved);
		ret=2;
	}
	else if ((on_off == 0) && (lm3533_dev->bl_status == BL_ON)) {
		lm3533_backlight_off(client);
		ret=1;
	}
	else if ((on_off == 0) && (lm3533_dev->bl_status == BL_OFF)) ret=0;
	else if ((on_off == 1) && (lm3533_dev->bl_status == BL_ON)) ret=0;

	return ret;
}
EXPORT_SYMBOL(lm3533_bl_on_off);

static int saved_level_for_ssd2825;
int lm3533_bl_onoff_ForHM(struct device *dev,int on_off)
{
	int ret=0;
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3533_device *lm3533_dev = i2c_get_clientdata(client);

	//printk("%s *** on_off : %d , lm3533_dev->bl_status:%d , current level:%d, saved level:%d \n",
	//__func__, on_off, lm3533_dev->bl_status,lm3533_dev->cur_main_lcd_level,lm3533_dev->saved_main_lcd_level);
	if ((on_off == 1) && (lm3533_dev->bl_status == BL_OFF)) {
		lm3533_backlight_on(client, saved_level_for_ssd2825);
		ret=2;
	}
	else if ((on_off == 0) && (lm3533_dev->bl_status == BL_ON)) {
		saved_level_for_ssd2825=lm3533_dev->saved_main_lcd_level;
		lm3533_backlight_off(client);
		ret=1;
	}
	else if ((on_off == 0) && (lm3533_dev->bl_status == BL_OFF)) ret=0;
	else if ((on_off == 1) && (lm3533_dev->bl_status == BL_ON)) ret=0;

	return ret;
}
EXPORT_SYMBOL(lm3533_bl_onoff_ForHM);

static ssize_t lm3533_bl_store_on_off(struct device *dev, 
				      struct device_attribute *attr,
				      const char *buf, 
				      size_t count)
{
	int on_off;
	struct i2c_client *client = to_i2c_client(dev); 
	struct lm3533_device *lm3533_dev = i2c_get_clientdata(client);

	if (!count) {
		dev_err(&client->dev, "Invalide arguemnt..\n");
		return -EINVAL;
	}
	
	dev_dbg(&client->dev, "previous backlight_status: %s\n", 
			lm3533_dev->bl_status ? "On" : "Off");
	
	on_off = simple_strtoul(buf, NULL, 10);
	
	dev_dbg(&client->dev, "stored argument is %d", on_off);
	
	if (on_off == 1) {
		mutex_lock(&lm3533dev->bl_mutex_saved);
		lm3533_backlight_on(client, lm3533_dev->saved_main_lcd_level);
		mutex_unlock(&lm3533dev->bl_mutex_saved);
	}
	else if (on_off == 0) {
		lm3533_backlight_off(client);
	}
	
	return count;

}
DEVICE_ATTR(lm3533_bl_level, 0660, lm3533_bl_show_lvl, lm3533_bl_store_lvl);
DEVICE_ATTR(lm3533_bl_onoff, 0660, lm3533_bl_show_on_off, lm3533_bl_store_on_off);

#ifdef CONFIG_LM3533_LEDS_CLASS

static int lm3533_bl_get_intensity(struct lm3533_device *drvdata)
{
	return drvdata->saved_main_lcd_level;
}

static void leds_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct lm3533_device *drvdata = dev_get_drvdata(led_cdev->dev->parent);
	int brightness;
	int next;

	if (!drvdata) {
		printk("Error getting drvier data\n");
		return;
	}

	mutex_lock(&lm3533dev->bl_mutex_saved);

	drvdata->saved_main_lcd_level = value;
	lm3533_set_main_current_level(drvdata->client, value);

	mutex_unlock(&lm3533dev->bl_mutex_saved);
}

static struct led_classdev lm3533_led_dev = {
	.name = LEDS_BACKLIGHT_NAME,
	.brightness_set = leds_brightness_set,
	.max_brightness = LED_FULL,
};
       
#else

static int lm3533_bl_set_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = to_i2c_client(bd->dev.parent);

	dev_dbg(&client->dev, "set_intensity = %d\n", bd->props.brightness);

	lm3533_lcd_backlight_set_level(client, bd->props.brightness);

	return 0;
}

static int lm3533_bl_get_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = to_i2c_client(bd->dev.parent);
	struct lm3533_device *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "get_intensity = %d\n", bd->props.brightness);
	dev_dbg(&client->dev, "dev->cur_main_lcd_level = %d\n", 
			dev->cur_main_lcd_level);

	return dev->cur_main_lcd_level;
}

static struct backlight_ops lm3533_bl_ops = { 
	.update_status	= lm3533_bl_set_intensity,
	.get_brightness	= lm3533_bl_get_intensity,
};
#endif

static int __devinit lm3533_bl_probe(struct i2c_client *i2c_dev,
			       const struct i2c_device_id *id)
{
	struct lm3533_bl_platform_data *pdata;
	struct lm3533_device *dev;
	int err;

	dev_info(&i2c_dev->dev, "Starting backlight device probing..\n");
	printk("YJChae lm3533_bl_probe : Starting backlight device probing..\n");

	pdata = i2c_dev->dev.platform_data;

	dev = kzalloc(sizeof(struct lm3533_device), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&i2c_dev->dev,"fail alloc for lm3533_device\n");
		return -ENOMEM;
	}

#ifdef CONFIG_LM3533_LEDS_CLASS
	dev->client = i2c_dev;
	dev->hwen_gpio = pdata->hwen_gpio;
	dev->max_current= pdata->max_current;
	dev->max_brightness= LM3533_BL_MAX_BRIGHTNESS;
	dev->min_brightness= pdata->min_brightness;
	dev->default_brightness= LM3533_BL_DEFAULT_BRIGHTNESS;
	dev->cur_main_lcd_level= LM3533_BL_DEFAULT_BRIGHTNESS;
	dev->saved_main_lcd_level= LM3533_BL_DEFAULT_BRIGHTNESS;
	dev->bl_status = BL_OFF;

	err = led_classdev_register(&i2c_dev->dev, &lm3533_led_dev);
	if (err < 0) {
		dev_err(&i2c_dev->dev, "led_classdev_register failed\n");
		goto err_led_classdev_register;
	}

	printk("Registering led class dev successfully.\n");
	dev->led = &lm3533_led_dev;
#endif

	i2c_set_clientdata(i2c_dev, dev);
	tegra_gpio_enable(dev->hwen_gpio);
	if (dev->hwen_gpio && 
			gpio_request(dev->hwen_gpio, "lm3533 reset") != 0) {
		dev_err(&i2c_dev->dev, "gpio_request(%d) failed\n",
				dev->hwen_gpio);
		err = -ENODEV;
		goto err_gpio_request;
	}

	gpio_direction_output(dev->hwen_gpio, 1);
	dev->bl_status = BL_ON; //YJChae

	mutex_init(&dev->bl_mutex);
	mutex_init(&dev->bl_mutex_saved);
	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3533_bl_level);
	if (err < 0) {
		dev_err(&i2c_dev->dev, "device_create_file(level) failed\n");
		goto err_device_create_file_1;
	}
	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3533_bl_onoff);
	if (err < 0) {
		dev_err(&i2c_dev->dev, "device_create_file(onoff) failed\n");
		goto err_device_create_file_2;
	}

	lm3533dev = dev;

	dev_info(&i2c_dev->dev, "lm3533 probed\n");

	lm3533_driver_ready = 1;

	return 0;

err_device_create_file_2:
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3533_bl_level);
err_device_create_file_1:
	gpio_free(dev->hwen_gpio);
err_gpio_request:
	i2c_set_clientdata(i2c_dev, NULL);
#ifdef CONFIG_LM3533_LEDS_CLASS
	led_classdev_unregister(dev->led);
err_led_classdev_register:
#endif
	kfree(dev);
	dev = NULL;

	return err;
}

static int __devexit lm3533_bl_remove(struct i2c_client *i2c_dev)
{
	struct lm3533_device *dev = i2c_get_clientdata(i2c_dev);
	int gpio = dev->hwen_gpio;

 	device_remove_file(&i2c_dev->dev, &dev_attr_lm3533_bl_level);
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3533_bl_onoff);

	led_classdev_unregister(dev->led);

	if (gpio_is_valid(gpio)) {
		gpio_free(gpio);
	}

	i2c_set_clientdata(i2c_dev, NULL);

	return 0;
}

static int lm3533_bl_shutdown(struct i2c_client *i2c_dev)
{
	dev_info(&i2c_dev->dev, "***** %s\n", __func__);
	lm3533_backlight_off(i2c_dev);
	return 0;
}

static struct i2c_driver lm3533_driver = {
	.driver 	= {
		.name	= I2C_BL_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= lm3533_bl_probe,
	.remove		= __devexit_p(lm3533_bl_remove),
	.shutdown	= lm3533_bl_shutdown,
	.id_table	= lm3533_bl_id, 
};

static int __init lm3533_init(void)
{
	return i2c_add_driver(&lm3533_driver);
}

static void __exit lm3533_exit(void)
{
	i2c_del_driver(&lm3533_driver);
}
 
module_init(lm3533_init);
module_exit(lm3533_exit);

MODULE_DESCRIPTION("LM3533 Backlight Control");
MODULE_AUTHOR("Jaeseong Gim <jaeseong.gim@lge.com>");
MODULE_LICENSE("GPL");
