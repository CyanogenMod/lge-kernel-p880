/*
 * lm3559.c - lm3559 flash/torch kernel driver
 *
 * Copyright (C) 2012 LGE Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 
 */



#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <media/lm3559.h>

#define LM3559_I2C_NAME  				"lm3559"

#if 1 //                 
#define LGE_X3_LM3559_FLASH_TORCH_CURRENT_FIXED 1
#endif

/* Register Descriptions */
#define LM3559_REG_ENABLE				      0x10
#define LM3559_REG_GPIO					      0x20
#define LM3559_REG_VLED_MONITOR			  0x30
#define LM3559_REG_ADC_DELAY			    0x31
#define LM3559_REG_VIN_MONITOR			  0x80
#define LM3559_REG_LAST_FLASH			    0x81
#define LM3559_REG_TORCH_BRIGHTNESS	  0xA0
#define LM3559_REG_FLASH_BRIGHTNESS	  0xB0
#define LM3559_REG_FLASH_DURATION		  0xC0
#define LM3559_REG_FLAGS				      0xD0
#define LM3559_REG_CONFIGURATION1		  0xE0
#define LM3559_REG_CONFIGURATION2		  0xF0
#define LM3559_REG_PRIVACY				    0x11
#define LM3559_REG_MESSAGE_INDICATOR	0x12
#define LM3559_REG_INDICATOR_BLINKING	0x13
#define LM3559_REG_PRIVACY_PWM			  0x14


enum{
   LM3559_LED_OFF,
   LM3559_LED_LOW,
   LM3559_LED_HIGH,
   LM3559_LED_MAX
}; 

struct lm3559_info {
	struct i2c_client *i2c_client;
	struct lm3559_platform_data *pdata;
	struct miscdevice miscdev;
};

static struct lm3559_info *info;
static int lm3559_onoff_state = LM3559_POWER_OFF;

#ifndef LGE_X3_LM3559_FLASH_TORCH_CURRENT_FIXED
static unsigned char lm3559_flash_lvl[16] = {  
		
      0x00, //0000 0000 
      0x11, //0001 0001
      0x22, //0010 0010
      0x33, //0011 0011
      0x44, //0100 0100
      0x55, //0101 0101
      0x66, //0110 0110 -> fixed value
      0x77, //0111 0111
      0x88, //1000 1000
      0x99, //1001 1001
      0xaa, //1010 1010
      0xbb, //1011 1011
      0xcc, //1100 1100 
      0xdd, //1101 1101
      0xee, //1110 1110
      0xff, //1111 1111     
};

static unsigned char lm3559_torch_lvl[8] = {  
		
      0x00, //000 000 
      0x09, //001 001
      0x12, //010 010 
      0x1b, //011 011
      0x24, //100 100 -> fixed value
      0x2d, //101 101
      0x36, //110 110 
      0x3f, //111 111
};
#endif

int lm3559_write_reg(struct i2c_client *client, unsigned char addr, unsigned char data)
{
	int err = 0;  
	int retry = 0;

	unsigned char buf[2] ={0,};
	
	struct i2c_msg msg[] = {
		{
			.addr  = client->addr, 
			.flags = 0, 
			.len   = 2, 
			.buf   = buf, 
		},
	};

	buf[0] = addr;
	buf[1] = data;

#if 0	//                 
	if ((err = i2c_transfer(client->adapter, &msg[0], 1)) < 0) {
		dev_err(&client->dev, "i2c write error [%d]\n",err);
	}
#else
  do {
		err = i2c_transfer(client->adapter, &msg[0], 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("lm3559: i2c transfer failed, retrying %x %x\n",addr, data);
		msleep(1);
	} while (retry <= 3);
#endif	
	return err;

}

int lm3559_read_reg(struct i2c_client *client, unsigned char addr, unsigned char *data)
{
	int err = 0;
	unsigned char buf[1] ={0};
	
	struct i2c_msg msgs[] = {	
		{ 
			.addr  = client->addr, 
			.flags = I2C_M_RD, 
			.len   = 1,
			.buf   = buf, 
		},
	};

	buf[0] = addr;
	
	if ((err = i2c_transfer(client->adapter, &msgs[0], 1)) < 0) {
		dev_err(&client->dev, "i2c read error [%d]\n",err);
	}

	*data = buf[0];
	
	return err;
	
}

void lm3559_led_shutdown(struct lm3559_info *info)
{	
	lm3559_write_reg(info->i2c_client,LM3559_REG_ENABLE,0x18);
}

/*	Torch Current
	 000 : 28.125 mA		100 : 140.625 mA	 
	 001 : 56.25 mA 		101 : 168.75 mA
	 010 : 84.375 mA 		110 : 196.875 mA
	 011 : 112.5mA  		111 : 225 mA
*/
void lm3559_enable_torch_mode(struct lm3559_info *info, int state, struct lm3559_param param)
{
#ifndef LGE_X3_LM3559_FLASH_TORCH_CURRENT_FIXED  
  unsigned char level_value;
#endif

	//pr_info("%s: state=%d, arg=%d\n",__func__, state, param.value);

#ifdef LGE_X3_LM3559_FLASH_TORCH_CURRENT_FIXED			
  pr_info("%s:[LGE fixed]  Level: 140.625 (Total 281.25) mA \n",__func__);

  // 0x24 : 100 100 -> 140.625
  lm3559_write_reg(info->i2c_client,LM3559_REG_TORCH_BRIGHTNESS,0x24);  
  udelay(10);
#else
  level_value = (unsigned char)param.value-1;
  lm3559_write_reg(info->i2c_client,LM3559_REG_TORCH_BRIGHTNESS,lm3559_torch_lvl[level_value]);  
  pr_info("%s: Level: %d \n",__func__, level_value);
#endif

	lm3559_write_reg(info->i2c_client,LM3559_REG_ENABLE,0x1A);
  udelay(10);


}

/*	 Flash Current
	 0000 : 56.25 mA		  1000 : 506.25 mA	 
	 0001 : 112.5 mA 		  1001 : 562.5 mA
	 0010 : 168.75 mA 		1010 : 618.75 mA
	 0011 : 225 mA  		  1011 : 675 mA
	 0100 : 281.25 mA		  1100 : 731.25 mA
	 0101 : 337.5 mA		  1101 : 787.5 mA
	 0110 : 393.75 mA		  1110 : 843.75 mA
	 0111 : 450 mA			  1111 : 900 mA
*/
void lm3559_enable_flash_mode(struct lm3559_info *info, int state, struct lm3559_param param)
{
#ifndef LGE_X3_LM3559_FLASH_TORCH_CURRENT_FIXED  	
  unsigned char data = 0;
  unsigned char level_value;
#endif

  //pr_info("%s: state=%d, arg=%d\n",__func__, state, param.value);

#ifdef LGE_X3_LM3559_FLASH_TORCH_CURRENT_FIXED			 
		pr_info("%s:[LGE fixed] Duration: 512ms, Level: 393.75 (Total 787.5) mA \n",__func__);

    lm3559_write_reg(info->i2c_client,LM3559_REG_FLASH_DURATION,0x1f); //                              
    udelay(10);
		//pr_info("[LM3559_LED_HIGH]LM3559_REG_FLASH_BRIGHTNESS \n");
    // 0x66 : 0110 0110 : 393.75 mA 
		lm3559_write_reg(info->i2c_client,LM3559_REG_FLASH_BRIGHTNESS,0x88);	//800mA-->1Achangho.ahn_0314
    udelay(10);
#else
	lm3559_read_reg(info->i2c_client,LM3559_REG_FLASH_DURATION,&data);	

	data = ((data & 0x1F) | 0x1F); /* 1.4A Peak Current & 1024ms Duration*/

	pr_info("%s: FLASH dutation [0x%x]\n",__func__,data);
	
	lm3559_write_reg(info->i2c_client,LM3559_REG_FLASH_DURATION,data);

  //pr_info("[LM3559_LED_LOW]LM3559_REG_FLASH_BRIGHTNESS : %d\n",lm3559_flash_lvl[param.p_value+1]);  
  level_value = (unsigned char)param.value-1;  
  lm3559_write_reg(info->i2c_client,LM3559_REG_FLASH_BRIGHTNESS,lm3559_flash_lvl[level_value]);  
  pr_info("%s: Level: %d \n",__func__, level_value);
#endif
	lm3559_write_reg(info->i2c_client,LM3559_REG_ENABLE,0x1B);
	udelay(10);

}

void lm3559_power_onoff(struct lm3559_info *info, int onoff){

	if(onoff == LM3559_POWER_OFF){
		gpio_set_value(info->pdata->gpio_act, 0);		
    udelay(10);
		gpio_direction_output(info->pdata->gpio_act, 0);
    pr_info("LM3559_POWER_OFF\n");
	}
	else{
    gpio_direction_output(info->pdata->gpio_act, 1);	
    udelay(10);
		gpio_set_value(info->pdata->gpio_act, 1);	
    pr_info("LM3559_POWER_ON\n");
	}
  mdelay(1); /* delay for device startup */
  
  //pr_info("%s: lm3559_onoff_state=%d, onoff=%d\n",__func__, lm3559_onoff_state, onoff);
}

//static DEFINE_SPINLOCK(lm3559_spinlock);

static long lm3559_ioctl(struct file *file,
			   unsigned int cmd,
			   unsigned long arg)
{
	struct lm3559_info *info = file->private_data;
  int rc = 0;

//  	pr_info("%s,cmd=%x, arg=%d\n", __func__,cmd,arg);

	switch (cmd){
    case LM3559_IOCTL_P0WER_CONT:
    {
      int is_onoff = (int)arg;
      //pr_info("[LM3559]LM3559_IOCTL_P0WER_CONT : %d\n",is_onoff);
      if(is_onoff == LM3559_POWER_OFF)
      {
        lm3559_power_onoff(info, LM3559_POWER_OFF);
        lm3559_onoff_state = LM3559_POWER_OFF;
      }
      else
      {        
        lm3559_power_onoff(info, LM3559_POWER_ON);
        lm3559_onoff_state = LM3559_POWER_ON;
      }
    }
    break;

    case LM3559_IOCTL_FLASH_TORCH:
    {
        struct lm3559_param param;
        if (copy_from_user(&param,
               (const void __user *)arg,
               sizeof(struct lm3559_param))) {
          pr_info("%s error %d\n", __func__, __LINE__);
          return -EFAULT;
        }

        if(param.param == LM3559_FLASH_LEVEL)
        {
          
          if(lm3559_onoff_state == LM3559_POWER_OFF){    
            lm3559_power_onoff(info, LM3559_POWER_ON);
            lm3559_onoff_state = LM3559_POWER_ON;
          }
          if(param.value){
            //pr_info("[LM3559]LM3559_FLASH_LEVEL : %d\n",param.value);
            lm3559_enable_flash_mode(info, LM3559_LED_HIGH, param);
          }
        }
        else if(param.param == LM3559_TORCH_LEVEL)
        {
          if(param.value == 0)
          {
            lm3559_power_onoff(info, LM3559_POWER_OFF);
            lm3559_onoff_state = LM3559_POWER_OFF;
            return rc;
          }
            
          if(lm3559_onoff_state == LM3559_POWER_OFF){    
            lm3559_power_onoff(info, LM3559_POWER_ON);
            lm3559_onoff_state = LM3559_POWER_ON;
          }          
          //pr_info("[LM3559]LM3559_TORCH_LEVEL : %d\n",param.value);
          lm3559_enable_torch_mode(info, LM3559_LED_LOW, param);
          
        }
        else
          pr_info("[LM3559]param.param is Wrong: %x\n",param.param);
      }
      break;
      
    default:
      lm3559_onoff_state = LM3559_POWER_OFF;
      rc = -EFAULT;
      break;
	}
    return rc;
  
}

static int lm3559_open(struct inode *inode, struct file *file)
{
  pr_info("%s\n", __func__);

  if(info == NULL)
        pr_info("lm3559_info is NULL\n");

	file->private_data = info;

  
  if(file->private_data == NULL)
        pr_info("file->private_data is NULL\n");

	return 0;
}

static int lm3559_release(struct inode *inode, struct file *file)
{
  pr_info("%s\n", __func__);
  
  if(lm3559_onoff_state == LM3559_POWER_ON)
    lm3559_power_onoff(info, LM3559_POWER_OFF);
    
	file->private_data = NULL;
	return 0;
}

static const struct file_operations lm3559_fileops = {
	.owner = THIS_MODULE,
	.open = lm3559_open,
	.unlocked_ioctl = lm3559_ioctl,
	.release = lm3559_release,
};

static struct miscdevice lm3559_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lm3559",
	.fops = &lm3559_fileops,
};


static ssize_t torch_store(struct device* dev,
		struct device_attribute* attr, const char* buf, size_t count)
{
	int val;
	sscanf(buf, "%ld", &val );

	if (val <= 0) {
		lm3559_power_onoff(info, LM3559_POWER_OFF);
		lm3559_onoff_state = LM3559_POWER_OFF;
	} else {
        	struct lm3559_param param;
		if(lm3559_onoff_state == LM3559_POWER_OFF){    
			param.param = LM3559_TORCH_LEVEL;
			param.value = (val == 2 ? 666 : 1);
			lm3559_power_onoff(info, LM3559_POWER_ON);
			lm3559_onoff_state = LM3559_POWER_ON;
			lm3559_enable_torch_mode(info, LM3559_LED_LOW, param);
		}          
	}

	return  count;
}
static DEVICE_ATTR(torch, 0666, NULL, torch_store);

static int lm3559_remove(struct i2c_client *client)
{
	struct lm3559_info *info = i2c_get_clientdata(client);

  pr_info("%s\n", __func__);

	device_remove_file(&client->dev, &dev_attr_torch);
	misc_deregister(&info->miscdev);
	kfree(info);
	return 0;
}

static int lm3559_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	//struct lm3559_info *info;
	int err;

	pr_info("%s start\n", __func__);

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		//dev_err(&client->dev, "%s: kzalloc error\n", __func__);
    pr_info("%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	err = misc_register(&lm3559_device);
	if (err) {
		pr_err("lm3559: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->i2c_client = client;
  info->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, info);


	tegra_gpio_enable(info->pdata->gpio_act);
	err = gpio_request(info->pdata->gpio_act, "lm3559");
	if (err < 0)
		pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "lm3559");

	err = device_create_file(&client->dev, &dev_attr_torch);
	if(err){
		printk("[lm3559_flash_led:] device create file torch fail!\n");
	}

  //gpio_direction_output(info->pdata->gpio_act, 1);	
  //gpio_set_value(info->pdata->gpio_act, 0);	
	return 0;
}

static const struct i2c_device_id lm3559_id[] = {
	{ "lm3559", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lm3559_id);

static struct i2c_driver lm3559_i2c_driver = {
	.driver = {
		.name = "lm3559",
		.owner = THIS_MODULE,
	},
	.id_table = lm3559_id,
	.probe = lm3559_probe,
	.remove = lm3559_remove,
};

static int __init lm3559_init(void)
{
	return i2c_add_driver(&lm3559_i2c_driver);
}

static void __exit lm3559_exit(void)
{
	i2c_del_driver(&lm3559_i2c_driver);
}

module_init(lm3559_init);
module_exit(lm3559_exit);

