
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <../../include/linux/regulator/lp8720.h>


static unsigned char lp8720_output_status = 0x00; //default on 0x3F
static bool lp8720_gpio_status = 0; //                 
struct i2c_client *lp8720_client=NULL;
static void lp8720_init(void);

/*                                                    */
static int lp8720_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg; 

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) 
		return -EIO;

	*data = buf[0];
	
	return 0;
}

static void lp8720_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int err;

	struct i2c_msg msg;
	u8 buf[2];

	msg.addr = (u16)client->addr;
	msg.flags =0;
	msg.len =2;

	buf[0]=reg;
	buf[1]=data;

	msg.buf = &buf[0];

	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) {
		dev_err(&client->dev, "[lp8720] i2c write error\n");
	}

	return;
}

void subpm_set_output(subpm_output_enum outnum, int onoff)
{
  
	struct lp8720_platform_data *pdata;

	if(lp8720_client == NULL)
	{
		printk(KERN_ERR "[lp8720] %s lp8720_client NULL\n", __func__);
		return;
	}
  
	if(outnum > 5) {
		dev_err(&lp8720_client->dev, "[lp8720] outnum error\n");
		return;
	}  

	if(onoff == 0)
		lp8720_output_status &= ~(1<<outnum);
	else
		lp8720_output_status |= (1<<outnum);


	pdata = lp8720_client->dev.platform_data;
	lp8720_write_reg(lp8720_client, LP8720_OUTPUT_ENABLE, 0x80 | lp8720_output_status);
  udelay(10);

/*                                                                            */
#if 0
	lp8720_read_reg(lp8720_client, LP8720_LDO5_SETTING, &data);
	//printk(KERN_ERR "LP8720_LDO5_SETTING %02x\n", data);
	if (data == 0xff) {
		lp8720_init();
		printk(KERN_ERR "%s LP8720 registers recovered.\n", __func__);
	}
#endif

}

void subpm_set_gpio(int onoff)
{
  struct lp8720_platform_data *pdata = lp8720_client->dev.platform_data;
  
  if(onoff == 0 && lp8720_gpio_status == 1)
  {    
    gpio_direction_output(pdata->en_gpio_num, 0);
    udelay(10);
    gpio_set_value(pdata->en_gpio_num, 0);     
    lp8720_gpio_status = 0;
    pr_info("[lp8720] power off\n");
    udelay(100);
  }
  else if(onoff == 1 && lp8720_gpio_status == 0)
  {
    gpio_direction_output(pdata->en_gpio_num, 1);
    udelay(10);
    gpio_set_value(pdata->en_gpio_num, 1); 
    lp8720_gpio_status = 1;
    pr_info("[lp8720] power on\n");
    udelay(100);
    lp8720_init();
  }
}


EXPORT_SYMBOL(subpm_set_output);
EXPORT_SYMBOL(subpm_set_gpio);

static void lp8720_init(void)
{  
  int ret;
	struct lp8720_platform_data *pdata = lp8720_client->dev.platform_data;

#if defined(CONFIG_VIDEO_MT9M114)
    lp8720_write_reg(lp8720_client, LP8720_LDO1_SETTING, LP8720_NO_STARTUP | 0x11); //1.8v - VT_DIG_1V8
    lp8720_write_reg(lp8720_client, LP8720_LDO2_SETTING, LP8720_NO_STARTUP | 0x19); //2.8v - VT_VANA_V2V8
#endif
#if defined(CONFIG_VIDEO_IMX119)
	lp8720_write_reg(lp8720_client, LP8720_LDO1_SETTING, LP8720_NO_STARTUP | 0x00); //1.2v - VT_DIG_1V2	
	lp8720_write_reg(lp8720_client, LP8720_LDO2_SETTING, LP8720_NO_STARTUP | 0x17); //2.7v - VT_VANA_V2V7
#endif
	lp8720_write_reg(lp8720_client, LP8720_LDO3_SETTING, LP8720_NO_STARTUP | 0x17); //2.7v - 8M_VANA_V2V8 -> (IMX111) 2.7v
	lp8720_write_reg(lp8720_client, LP8720_LDO4_SETTING, LP8720_NO_STARTUP | 0x11); //1.8v - 1.8V_CAM_VIO //hyojin.an 0C->11
	lp8720_write_reg(lp8720_client, LP8720_LDO5_SETTING, LP8720_NO_STARTUP | 0x19); //2.8v - 8M_VCM_V2V8
	lp8720_write_reg(lp8720_client, LP8720_BUCK_SETTING1, LP8720_NO_STARTUP | 0x09);
	lp8720_write_reg(lp8720_client, LP8720_BUCK_SETTING2, 0x09); //1.2v
	lp8720_write_reg(lp8720_client, LP8720_OUTPUT_ENABLE, 0x80 | lp8720_output_status);
	lp8720_write_reg(lp8720_client, LP8720_PULLDOWN_BITS, 0xBF); //don't  reset registers after thermal shutdown.
  udelay(10);

	return;
}

static int __devinit lp8720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int ret;
	struct lp8720_platform_data *pdata;
	printk("[lp8720]  lp8720_probe ver 1.2\n");

	if (i2c_get_clientdata(client))
		return -EBUSY;

	pdata = client->dev.platform_data;
	lp8720_client = client;

	ret = gpio_request(pdata->en_gpio_num, "lp8720");
  if (ret < 0)
  {
    printk("[lp8720] gpio_request : en_gpio_num=%d, ret=%x \n",pdata->en_gpio_num,ret);
  }

  tegra_gpio_enable(pdata->en_gpio_num);
  udelay(10);
  
	//lp8720_init();

	return 0;
}

static int __devexit lp8720_remove(struct i2c_client *client)
{
	//struct lp8720_platform_data *pdata;
	//pdata = client->dev.platform_data;

	printk("%s\n",__func__);
  subpm_set_gpio(0);
	kfree(i2c_get_clientdata(client));

	return 0;
}	

#if 0 //                                               
static int lp8720_suspend(struct i2c_client *client, pm_message_t mesg)
{
	//struct lp8720_platform_data *pdata;
	//pdata = client->dev.platform_data;

  printk("%s\n",__func__);
  
  //                                                                 
	return 0;
}

static int lp8720_resume(struct i2c_client *client)
{
  
	//struct lp8720_platform_data *pdata;
	//pdata = client->dev.platform_data;
  
	printk("%s\n",__func__);

  //                                                                      
  //                                                                 
  
	return 0;
}
#endif

static const struct i2c_device_id lp8720_ids[] = {
	{ LP8720_I2C_NAME, 0 },	/*lp8720*/
	{ /* end of list */ },
};

static struct i2c_driver subpm_lp8720_driver = {
	.probe = lp8720_probe,
	.remove = lp8720_remove,
	//                                                  
	//                                                
	.id_table	= lp8720_ids,
	.driver = {
		.name = LP8720_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init subpm_lp8720_init(void)
{
  printk("[lp8720]  subpm_lp8720_init\n");
	return i2c_add_driver(&subpm_lp8720_driver);
}

static void __exit subpm_lp8720_exit(void)
{
	i2c_del_driver(&subpm_lp8720_driver);
}

module_init(subpm_lp8720_init);
module_exit(subpm_lp8720_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("LP8720 sub pmic Driver");
MODULE_LICENSE("GPL");

