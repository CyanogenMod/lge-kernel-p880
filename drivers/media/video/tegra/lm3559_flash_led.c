#include <linux/module.h>                                                                                            
#include <linux/i2c.h>                                                                                               
#include <linux/delay.h>                                                                                             
#include <linux/pm.h>                                                                                                
#include <mach/gpio.h>                                                                                               
#include <media/lm3559_flash_led.h>
                                                                                                                     
#define setbits(data, masks)		data |=  masks                                                                
#define clrbits(data, masks)		data &= ~masks                                                                
                                                                                                                     
static struct i2c_client *this_client	=	0;                                                                    
static struct lm3559_flash_led_platform_data*	pdata	=	0;                                                            
static unsigned char	flash_brightness	=	7;//                                                       
                                                                                                                     
static int lm3559_flash_led_enable(int status)                                                                                 
{                                                                                                                    
  printk(KERN_INFO "enter %s input value %d \n", __func__, status );

  gpio_direction_output(pdata->gpio_hwen, 1);
	gpio_set_value(pdata->gpio_hwen, (status != 0));                                                              
                                                                                                                     
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
static int lm3559_flash_led_write(unsigned char index, unsigned char data)                                                     
{                                                                                                                    
	printk(KERN_INFO "[lm3559_flash_led] %s, index: 0x%x, data: 0x%x \n", __func__,index, data);
	return i2c_smbus_write_byte_data(this_client,index,data);                                                     
}                                                                                                                    
                                                                                                                     
static int lm3559_flash_led_read(unsigned char index)                                                                          
{   
#if 0
	int ret = i2c_smbus_read_byte_data(this_client,index);                                                           
	printk(KERN_INFO "[lm3559_flash_led] %s, index: 0x%x, data: 0x%2x \n", __func__,index, ret&0x000000ff);
	return ret;
#endif
	return i2c_smbus_read_byte_data(this_client,index);  
}                                                                                                                    
                                                                                                                     
static int lm3559_flash_led_flash(unsigned char timeout)                                                                       
{                                                                                                                    

	unsigned char value;

	printk(KERN_INFO "enter %s\n", __func__);  	

	if (timeout > 0x1f)                                                                                           
		return	-1;                                                                                           

	// Set flash brightness                                                                                       
	lm3559_flash_led_write(FLASH_BRIGHTNESS_REG_INDEX, (flash_brightness << 4) | flash_brightness);                         
	                                                                                                              
	value	=	lm3559_flash_led_read(FLASH_DURATION_REG_INDEX);                                                        
                                                                                                                     
	clrbits(value, 0x1F);                                                                                         
	setbits(value, timeout);                                                                                      
	lm3559_flash_led_write(FLASH_DURATION_REG_INDEX, value); 
	
	value	=	lm3559_flash_led_read(ENABLE_REG_INDEX);                                                                
	setbits(value, 0x03);                                                                                         
	lm3559_flash_led_write(ENABLE_REG_INDEX, value);                                                                        
#if 0
	 lm3559_flash_led_write(0xb0, 0x77);
	 lm3559_flash_led_write(0xc0, 0x6f);
	 lm3559_flash_led_write(0x80, 0xc9);
	 lm3559_flash_led_write(0xa0, 0x3f);
	 lm3559_flash_led_write(0x10, 0x1b);
	 lm3559_flash_led_read(0xd0);
#endif 
	return 0;                                                                                                     
}                                                                                                                    
                                                                                                                     
static void camera_flash(unsigned long timeout)                                                                      
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);

	if (timeout > 1024)                                                                                           
		return	;                                                                                             
                                                                                                                     
	lm3559_flash_led_flash(timeout/32);                                                                                     
}                                                                                                                    
                                                                                                                     
static int lm3559_flash_led_torch(unsigned char brightness)                                                                    
{                                                                                                                    
	unsigned char value;                                                                                          

//	printk(KERN_INFO "enter %s\n", __func__);																													 

	if (brightness > 0x3F)                                                                                        
		brightness = 0x3F;                                                                         

	if(brightness == 0)
	{
		// Off torch
		value = lm3559_flash_led_read(ENABLE_REG_INDEX);																		  
		clrbits(value, 0x03); 
		lm3559_flash_led_write(ENABLE_REG_INDEX,value);
	}
	else
	{
		lm3559_flash_led_write(TORCH_BRIGHTNESS_REG_INDEX, brightness);//set torch brightness 								  
		
		value = lm3559_flash_led_read(ENABLE_REG_INDEX);																		  
		clrbits(value, 0x01);																						  
		setbits(value, 0x02);																						  
		lm3559_flash_led_write(ENABLE_REG_INDEX,value);
	}
	
#if 0
	lm3559_flash_led_write(0xa0, 0x52);//set torch brightness                                                                                                                                                     
	lm3559_flash_led_write(0x10, 0x1a);//set torch brightness
	lm3559_flash_led_read(0xa0);//set torch brightness                                                                                                                                                     
	lm3559_flash_led_read(0x10);//set torch brightness
#endif
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
                                                                                                                     
static ssize_t flash_store(struct device* dev,                                                                       
							struct device_attribute* attr, const char* buf, size_t count) 
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);
	camera_flash(simple_strtoul(buf, NULL, 10));                                                                  
	                                                                                                              
	return count;                                                                                                 
}                                                                                                                    
static DEVICE_ATTR(flash, 0666, NULL, flash_store);                                                                  
                                                                                                                     
static ssize_t flash_brightness_store(struct device* dev,                                                            
							struct device_attribute* attr, const char* buf, size_t count) 
{                                                                                                                    
	unsigned long	value	=	simple_strtoul(buf, NULL, 10);                                                

	printk(KERN_INFO "enter %s\n", __func__);																													 

	if ((0 <= value) && (value < 16))                                                                             
		flash_brightness	=	(unsigned char)value;                                                 
                                                                                                                     
	return	count;                                                                                                
}                                                                                                                    
                                                                                                                     
static ssize_t flash_brightness_show(struct device* dev,                                                             
							struct device_attribute* attr, const char* buf, size_t count) 
{    
	printk(KERN_INFO "enter %s\n", __func__);

	return	snprintf(buf, PAGE_SIZE, "%d\n", flash_brightness);                                                   
}                                                                                                                    
static DEVICE_ATTR(flash_brightness, 0666, flash_brightness_show, flash_brightness_store);                           
                                                                                                                     
static ssize_t torch_store(struct device* dev,                                                                       
							struct device_attribute* attr, const char* buf, size_t count) 
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);

	lm3559_flash_led_torch(simple_strtoul(buf, NULL, 10));                                                                  
                                                                                                                     
	return	count;                                                                                                
}                                                                                                                    
static DEVICE_ATTR(torch, 0666, NULL, torch_store);                                                                  
                                                                                                                     
static ssize_t enable_store(struct device* dev,                                                                      
							 struct device_attribute* attr, const char* buf, size_t count)
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);

	lm3559_flash_led_enable(simple_strtoul(buf, NULL, 10));	
																													 
	return	count;                                                                                                
}                                                                                                                    
                                                                                                                     
static ssize_t enable_show(struct device* dev,                                                                       
							 struct device_attribute* attr, const char* buf, size_t count)
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);

	return	snprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->gpio_hwen));                                   
}                                                                                                                    
static DEVICE_ATTR(enable, 0666, enable_show, enable_store);                                                         
                                                                                                                     
static int lm3559_flash_led_remove(struct i2c_client *client)                                                                  
{ 
  int res;
	printk(KERN_INFO "enter %s\n", __func__);

  
  res = gpio_direction_output(pdata->gpio_hwen, 0);
  if (res < 0)
  {
    printk("[lm3559] gpio_direction_output : gpio_hwen=%d, ret=%x \n",pdata->gpio_hwen,res);
  }
  mdelay(100);

	lm3559_flash_led_enable(0);                                                                                             
                                                                                                                     
	gpio_free(pdata->gpio_hwen);                                                                                  
	                                                                                                              
	if (client != 0) {                                                                                            
		device_remove_file(&client->dev, &dev_attr_enable);                                                   
		device_remove_file(&client->dev, &dev_attr_flash);                                                    
		device_remove_file(&client->dev, &dev_attr_flash_brightness);                                         
		device_remove_file(&client->dev, &dev_attr_torch);                                                    
	}                                                                                                             
                                                                                                                     
	client	=	0;                                                                                            
                                                                                                                     
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
static int lm3559_flash_led_probe(struct i2c_client *client, const struct i2c_device_id *devid)                                
{                                                                                                                    
	int res = 0;                                                                                                  

	printk(KERN_INFO "enter %s\n", __func__);
																													 
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
    printk(KERN_INFO "[lm3559] i2c_check_functionality error \n");
		return	-ENODEV;                                                                                      
	}
  
	this_client	=	client;                                                                               
                                                                                                                     
	pdata	=	client->dev.platform_data;                                                                    

#if 0
	tegra_gpio_enable(pdata->gpio_hwen);
	gpio_request(pdata->gpio_hwen, "flash_en");                                                                   
#else
  
  res = gpio_request(pdata->gpio_hwen, "flash_en"); 
  if (res < 0)
  {
    printk("[lm3559] gpio_request : gpio_hwen=%d, ret=%x \n",pdata->gpio_hwen,res);
  }

  tegra_gpio_enable(pdata->gpio_hwen);
  mdelay(100);
  
  printk("[lm3559] gpio_direction_output : en_gpio_num=%d\n",pdata->gpio_hwen);
  
  res = gpio_direction_output(pdata->gpio_hwen, 1);
  if (res < 0)
  {
    printk("[lm3559] gpio_direction_output : en_gpio_num=%d, ret=%x \n",pdata->gpio_hwen,res);
  }
  mdelay(100);

  gpio_set_value(pdata->gpio_hwen, 0);
#endif

	                                                                                                              
	res = device_create_file(&client->dev, &dev_attr_flash);                                                      
	if(res){                                                                                                      
		printk("[lm3559_flash_led:] device create file flash fail!\n");                                                 
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	res = device_create_file(&client->dev, &dev_attr_flash_brightness);                                           
	if(res){                                                                                                      
		printk("[lm3559_flash_led:] device create file flash_brightness fail!\n");                                      
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	res = device_create_file(&client->dev, &dev_attr_torch);                                                      
	if(res){                                                                                                      
		printk("[lm3559_flash_led:] device create file torch fail!\n");                                                 
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	res = device_create_file(&client->dev, &dev_attr_enable);                                                     
	if(res){                                                                                                      
		printk("[lm3559_flash_led:] device create file enable fail!\n");                                                
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	return res;                                                                                                   
                                                                                                                     
exit:	                                                                                                              
	lm3559_flash_led_remove(this_client);                                                                                   
	this_client	=	0;                                                                                    
	pdata		=	0;                                                                                    
                                                                                                                     
	return	res;                                                                                                  
                                                                                                                     
}                                                                                                                    
                                                                                                                     
static int lm3559_flash_led_suspend(struct i2c_client *client, pm_message_t message)                                           
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);
	
	lm3559_flash_led_enable(0);                                                                                             
                                                                                                                     
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
static int lm3559_flash_led_resume(struct i2c_client *client)                                                                  
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);
	
	lm3559_flash_led_enable(1);                                                                                             
                                                                                                                     
	return 0;                                                                                                     
}                                                                                                                    
                                                                                                                     
static const struct i2c_device_id id_table_lm3559_flash_led[] =                                                                
{                                                                                                                    
	{	lm3559_flash_led_I2C_NAME,	0	},                                                                    
	{},                                                                                                            
};                                                                                                                   
                                                                                                                     
static struct i2c_driver i2c_driver_lm3559_flash_led =                                                                         
{                                                                                                                    
	.driver = {                                                                                                   
		.owner	=	THIS_MODULE,                                                                          
		.name	=	lm3559_flash_led_I2C_NAME,                                                                      
	},                                                                                                            
	.probe		=	lm3559_flash_led_probe,                                                                         
	.remove		=	__devexit_p(lm3559_flash_led_remove),                                                           
	.suspend	=	lm3559_flash_led_suspend,                                                                       
	.resume		=	lm3559_flash_led_resume,                                                                        
	.id_table	=	id_table_lm3559_flash_led,                                                                      
};                                                                                                                   

//                                                            
#include <linux/regulator/consumer.h>
static void lm3559_flash_led_power_init(void)
{
	struct regulator *regulator= NULL;

	regulator   =   regulator_get(NULL, "vddio_vi");
	if (!regulator) {
		printk(KERN_INFO "%s : Power enable failed\n", __func__);
	}
	else {
		regulator_set_voltage(regulator, 1800000, 3000000);
		regulator_enable(regulator);
	}
}

static int __init lm3559_flash_led_init(void)                                                                                  
{	
	printk(KERN_INFO "enter %s\n", __func__);
	//hyojin.an lm3559_flash_led_power_init();
	return i2c_add_driver(&i2c_driver_lm3559_flash_led);	                                                              
}
//                                                                                                                                                                           

static void __exit lm3559_flash_led_exit(void)                                                                                 
{                                                                                                                    
	printk(KERN_INFO "enter %s\n", __func__);
	i2c_del_driver(&i2c_driver_lm3559_flash_led);                                                                           
}

                                                                                                                     
module_init(lm3559_flash_led_init);                                                                                            
module_exit(lm3559_flash_led_exit);                                                                                                                                                                                       
