/***************************************************************************
* 
*   SiI9244 - MHL Transmitter Driver
*
* Copyright (C) (2011, Silicon Image Inc)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation version 2.
*
* This program is distributed ¡°as is¡± WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*****************************************************************************/
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <asm/uaccess.h> 
#include <linux/regulator/consumer.h>

//#include "sii9244_driver.h"
//#include "Common_Def.h"

//                                                            
#include <linux/mutex.h>
//                                  
#include "sii9244_driver.h"

extern void hdmi_common_send_uevent(char *buf);
//                                                   
#include <linux/wakelock.h>
static struct wake_lock mhl_lock;
//                                  
static int mhl_connected = 0;
extern void sii9244_driver_init(void);
extern struct timer_list simg_timer;

#ifdef MHL_CTL
#define MHL_DEV_INPUT_KBMOUSE_EVENT 		"/dev/input/event9"	// for real keyboard & mouse
#undef MHL_DEV_INPUT_KBMOUSE_EVENT
//                                                                                      
#ifdef MHL_HW_DEBUG
extern void set_mhl_ctrl4(int value);
#endif
//                                                                                      

#define DRIVER_NAME "mhl_virtual_kbmouse"
#define DISABLE_CURSOR	10000

#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
static struct input_dev *mhl_virtual_kbmouse;
#endif
int mhl_ms_ptr(signed short x, signed short y);
void mhl_ms_hide_cursor(void);
int mhl_ms_btn(int action);
int mhl_kbd_key(unsigned int tKeyCode, int value);
int MHLRCPtoKeyboard(u8 mhlrcpkey);
void mhl_writeburst_uevent(unsigned short mev);
#endif

struct mhl_work_struct {
	struct work_struct work;
};


#if 0
#define SII_LOG_FUNCTION_NAME_ENTRY             printk(KERN_INFO "[SII9244]## %s() ++ ##\n",  __func__);
#define SII_LOG_FUNCTION_NAME_EXIT              printk(KERN_INFO "[SII9244]## %s() -- ##\n",  __func__);
#else
#define SII_LOG_FUNCTION_NAME_ENTRY
#define SII_LOG_FUNCTION_NAME_EXIT
#endif
#define MHL_REGULATOR

#if defined(MHL_REGULATOR) /* convert gpio to regulator */
static struct regulator *mhl_reg = NULL;
static struct regulator *mhl_1v2 = NULL;
#endif 

struct work_struct sii9244_int_work;
struct workqueue_struct *sii9244_wq = NULL;

struct i2c_driver sii9244_i2c_driver;
struct i2c_client *sii9244_i2c_client = NULL;

struct i2c_driver sii9244a_i2c_driver;
struct i2c_client *sii9244a_i2c_client = NULL;

struct i2c_driver sii9244b_i2c_driver;
struct i2c_client *sii9244b_i2c_client = NULL;

struct i2c_driver sii9244c_i2c_driver;
struct i2c_client *sii9244c_i2c_client = NULL;

//extern bool sii9244_init(void);
//extern void sii9244_mhl_tx_int(void);
//extern void simg_mhl_tx_handler(void);

static struct i2c_device_id sii9244_id[] = {
	{"SII9244", 0},
	{}
};

static struct i2c_device_id sii9244a_id[] = {
	{"SII9244A", 0},
	{}
};

static struct i2c_device_id sii9244b_id[] = {
	{"SII9244B", 0},
	{}
};

static struct i2c_device_id sii9244c_id[] = {
	{"SII9244C", 0},
	{}
};

int MHL_i2c_init = 0;


struct sii9244_state {
	struct i2c_client *client;
};

void sii9244_cfg_power(bool on);

static void sii9244_cfg_gpio(void);

irqreturn_t mhl_int_irq_handler(int irq, void *dev_id);

irqreturn_t mhl_wake_up_irq_handler(int irq, void *dev_id);

void sii9244_interrupt_event_work(struct work_struct *p);

#define MHL_SWITCH_TEST	1

#ifdef MHL_SWITCH_TEST
struct class *sec_mhl;
EXPORT_SYMBOL(sec_mhl);

struct device *mhl_switch;
EXPORT_SYMBOL(mhl_switch);

static ssize_t check_MHL_command(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res = 0;

	SII_LOG_FUNCTION_NAME_ENTRY;
	sii9244_cfg_power(1);
//	res = SiI9244_startTPI();
	count = sprintf(buf,"%d\n", res );
	sii9244_cfg_power(0);
    SII_LOG_FUNCTION_NAME_EXIT;
	return count;

}

static ssize_t change_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);
	int i;
    SII_LOG_FUNCTION_NAME_ENTRY;
    
	SII_DEV_DBG("Change the switch: %ld\n", value);

	if (value == 0) {
		for (i = 0; i <20; i++) {
			SII_DEV_DBG("[MHL] try %d\n", i+1);
			msleep(500);
		}
//		gpio_request(GPIO_MHL_INT, "MHL_INT");
		gpio_direction_input(GPIO_MHL_INT);
		request_irq(gpio_to_irq(GPIO_MHL_INT), mhl_int_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mhl_irq", dev); 
//                                
		sii9244_cfg_power(1);
//		sii9244_init();
//		sii9244_mhl_tx_int();
	} else {	
		sii9244_cfg_power(0);
//		gpio_request(GPIO_MHL_SEL, "MUIC/MHL SEL");
		gpio_direction_output(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
//                               
	}
    SII_LOG_FUNCTION_NAME_EXIT;

	return size;
}

//                                                                                      
#ifdef MHL_HW_DEBUG
static ssize_t change_mhl_ctrl4(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	SII_LOG_FUNCTION_NAME_ENTRY;
	long value = simple_strtoul(buf,NULL,16);
     
	SII_DEV_DBG("the mhl_ctrl4: 0x%x\n", value);

	set_mhl_ctrl4((int)value);

    SII_LOG_FUNCTION_NAME_EXIT;

	return size;
}
#endif
//                                                                                      

void MHL_On(bool on)
{
	static DEFINE_MUTEX(mutex);
    
    SII_LOG_FUNCTION_NAME_ENTRY;

	mutex_lock(&mutex);


	printk("[MHL] USB path change : %d\n", on);
    
	if (on == 1)
    {		
		mhl_connected = true;
		
		if(gpio_get_value(GPIO_MHL_SEL))
		{
			printk("[MHL] GPIO_MHL_SEL : already 1\n");
        }
		else 
        {
			//gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH);
			sii9244_cfg_power(1);
//			sii9244_init();
			sii9244_mhl_tx_int();
 			gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH); //daniel for mass product issue
		}
		wake_lock(&mhl_lock);           //                                                         
	} 
    else
    {
	 	mhl_connected = 0;
		if(!gpio_get_value(GPIO_MHL_SEL))
			printk("[MHL] GPIO_MHL_SEL : already 0\n");
		else 
        {

				gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
				sii9244_cfg_power(0);
		//		gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
			#ifdef MHL_CTL
				mhl_ms_hide_cursor();
				mhl_writeburst_uevent(2);
			#endif
		}
		
		wake_unlock(&mhl_lock);		//                                                         
		wake_lock_timeout(&mhl_lock, 2*HZ);

	}

	mutex_unlock(&mutex);
    SII_LOG_FUNCTION_NAME_EXIT;
}
EXPORT_SYMBOL(MHL_On);

//                                             
bool is_MHL_connected(void)
{
    if(mhl_connected)
        return TRUE;
    
	return FALSE;

}
EXPORT_SYMBOL(is_MHL_connected);
//                                             



static DEVICE_ATTR(mhl_sel, S_IRUGO | S_IWUSR | S_IXOTH, check_MHL_command, change_switch_store);
#endif
//                                                                                      
#ifdef MHL_HW_DEBUG
static DEVICE_ATTR(mhl_ctrl4, S_IRUGO | S_IWUSR | S_IXOTH, NULL, change_mhl_ctrl4);
#endif
//                                                                                      

static ssize_t MHD_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res = 0;
	#if 0

	s3c_gpio_setpull(GPIO_MHL_SEL, S3C_GPIO_PULL_UP);	//MHL_SEL

	gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH);
	

	//TVout_LDO_ctrl(true);
	
	if(!MHD_HW_IsOn())
	{
		sii9244_tpi_init();
		res = MHD_Read_deviceID();
		MHD_HW_Off();		
	}
	else
	{
		sii9244_tpi_init();
		res = MHD_Read_deviceID();
	}

	I2C_WriteByte(0x72, 0xA5, 0xE1);
	res = 0;
	res = I2C_ReadByte(0x72, 0xA5);

	printk(KERN_ERR "A5 res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1B);

	printk(KERN_ERR "Device ID res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1C);

	printk(KERN_ERR "Device Rev ID res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1D);

	printk(KERN_ERR "Device Reserved ID res %x",res);

	printk(KERN_ERR "\n####HDMI_EN1 %x MHL_RST %x GPIO_MHL_SEL %x\n",gpio_get_value(GPIO_MHL_EN),gpio_get_value(GPIO_MHL_RST),gpio_get_value(GPIO_MHL_SEL));

	res = I2C_ReadByte(0x7A, 0x3D);

	res = I2C_ReadByte(0x7A, 0xFF);
		
	s3c_gpio_setpull(GPIO_MHL_SEL, S3C_GPIO_PULL_NONE);	//MHL_SEL

	gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
#endif
	count = sprintf(buf,"%d\n", res );
	//TVout_LDO_ctrl(false);
	return count;
}

static ssize_t MHD_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	SII_DEV_DBG("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(MHD_file, S_IRUGO , MHD_check_read, MHD_check_write);


struct i2c_client* get_sii9244_client(u8 device_id)
{

	struct i2c_client* client_ptr;
    SII_LOG_FUNCTION_NAME_ENTRY;

	if(device_id == 0x72)
		client_ptr = sii9244_i2c_client;
	else if(device_id == 0x7A)
		client_ptr = sii9244a_i2c_client;
	else if(device_id == 0x92)
		client_ptr = sii9244b_i2c_client;
	else if(device_id == 0xC8)
		client_ptr = sii9244c_i2c_client;
	else
		client_ptr = NULL;
    
    SII_LOG_FUNCTION_NAME_EXIT;

	return client_ptr;
}
EXPORT_SYMBOL(get_sii9244_client);

u8 sii9244_i2c_read(struct i2c_client *client, u8 reg)
{
	u8 ret;
	SII_LOG_FUNCTION_NAME_ENTRY;
	if(!MHL_i2c_init)
	{
		SII_DEV_DBG_ERROR("I2C not ready");
		return 0;
	}
	
	i2c_smbus_write_byte(client, reg);
	

	ret = i2c_smbus_read_byte(client);

	//printk(KERN_ERR "#######Read reg %x data %x\n", reg, ret);

	if (ret < 0)
	{
		SII_DEV_DBG_ERROR("i2c read fail");
		return -EIO;
	}
    SII_LOG_FUNCTION_NAME_EXIT;
	return ret;

}
EXPORT_SYMBOL(sii9244_i2c_read);


int sii9244_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
    int rc = 0;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
	if(!MHL_i2c_init)
	{
		SII_DEV_DBG_ERROR("I2C not ready");
		return 0;
	}

	//printk(KERN_ERR "#######Write reg %x data %x\n", reg, data);
	rc = i2c_smbus_write_byte_data(client, reg, data);
	SII_LOG_FUNCTION_NAME_EXIT;
	return rc; 
}
EXPORT_SYMBOL(sii9244_i2c_write);


void sii9244_interrupt_event_work(struct work_struct *p)
{
    SII_LOG_FUNCTION_NAME_ENTRY;

	//                  
	struct mhl_work_struct *mhl_work =
		container_of(p, struct mhl_work_struct, work);

	SII_DEV_DBG("[MHL] sii9244_interrupt_event_work() is called\n");
//	sii9244_interrupt_event();
	simg_mhl_tx_handler();
    
    if(mhl_work != NULL)
    {
        kfree(mhl_work);
        mhl_work = NULL;
    }
    SII_LOG_FUNCTION_NAME_EXIT;
}

#if 0
void mhl_int_irq_handler_sched(void)
{
    SII_LOG_FUNCTION_NAME_ENTRY;
	
	//printk(KERN_ERR "mhl_int_irq_handler_sched() is called\n");
	queue_work(sii9244_wq, &sii9244_int_work);		
    SII_LOG_FUNCTION_NAME_EXIT;
}
#else

void mhl_int_irq_handler_sched(void)
{
	struct mhl_work_struct *mhl_work;
    SII_LOG_FUNCTION_NAME_ENTRY;
    
	mhl_work = kmalloc(sizeof(*mhl_work), GFP_ATOMIC);

	if (mhl_work) 
	{
		INIT_WORK(&mhl_work->work, sii9244_interrupt_event_work);
		queue_work(sii9244_wq, &mhl_work->work);
	} 
    else 
	{
		printk(KERN_ERR "Cannot allocate memory to create work");
	}
    SII_LOG_FUNCTION_NAME_EXIT;
}
#endif


irqreturn_t mhl_int_irq_handler(int irq, void *dev_id)
{
    SII_LOG_FUNCTION_NAME_ENTRY;

    if (gpio_get_value(GPIO_MHL_SEL))	
    {   
        SII_DEV_DBG("irq=%d, dev_id=%p\n", irq, dev_id);
        mhl_int_irq_handler_sched();
    }
    SII_LOG_FUNCTION_NAME_EXIT;
    return IRQ_HANDLED;
}

//                                                                    
static int hdmi_rcp_value;

static ssize_t hdmi_rcp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	long val;
	int status = size;
	char mbuf[120];
	printk(KERN_ERR "hdmi_rcp_store \n");
	if ((strict_strtol(buf, 10, &val) < 0) || (val > 128))
	{
		printk("\n hdmi_rcp_store write error!! valied value is 1~127\n");
		return -EINVAL;
	}
	hdmi_rcp_value = val;
	memset(mbuf, 0, sizeof(mbuf));
	sprintf(mbuf, "MHL_RCP=keycode=%04d",  hdmi_rcp_value);
	hdmi_common_send_uevent(mbuf);
	return status;	
}

static ssize_t hdmi_rcp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n", hdmi_rcp_value);
}
static DEVICE_ATTR(rcp_test, S_IRUGO | S_IWUSR, hdmi_rcp_show, hdmi_rcp_store);

void rcp_cbus_uevent(u8 rcpCode)	
{
	char mbuf[120];
    SII_LOG_FUNCTION_NAME_ENTRY;

	SII_DEV_DBG("RCP Message Recvd , rcpCode =0x%x\n",rcpCode);

	memset(mbuf, 0, sizeof(mbuf));
	sprintf(mbuf, "MHL_RCP=keycode=%04d",  rcpCode);
	hdmi_common_send_uevent(mbuf);
	
    SII_LOG_FUNCTION_NAME_EXIT;
	return;
}
EXPORT_SYMBOL(rcp_cbus_uevent);
//                                                                     
#ifdef MHL_CTL
int mhl_ms_ptr(signed short x, signed short y)	// mouse pointer, here because of speed
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	struct file *fd;
	struct input_event event;
	int ret;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	fd = filp_open(MHL_DEV_INPUT_KBMOUSE_EVENT, O_RDWR, 0);

	if (IS_ERR(fd)) {
		printk("mhl_ms_input, could not open touch device \n");
	       set_fs(fs);

		return 1;
	}

	event.type = EV_REL;
	event.code = REL_X;
	event.value = x;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);
	event.type = EV_REL;
	event.code = REL_Y;
	event.value = y;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);
	event.type = EV_SYN;
	event.code = SYN_REPORT;
	event.value = 0;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	filp_close(fd, NULL);

	set_fs(fs);
#endif
	return 0;
}
EXPORT_SYMBOL(mhl_ms_ptr);
void mhl_ms_hide_cursor(void)	// in case mhl disconnection
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	mhl_ms_ptr(DISABLE_CURSOR,DISABLE_CURSOR);
#endif
}
EXPORT_SYMBOL(mhl_ms_hide_cursor);

int mhl_ms_btn(int action)		// mouse button...only left button, here becasue of speed
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	struct file *fd;
	int ret;
	struct input_event event;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	fd = filp_open(MHL_DEV_INPUT_KBMOUSE_EVENT, O_RDWR, 0);

	if (IS_ERR(fd)) {
		printk("mhl_ms_input, could not open touch device \n");
	       set_fs(fs);

		return 1;
	}

	event.type = EV_KEY;
	event.code = BTN_LEFT;	// = BTN_MOUSE
	event.value = action;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);
	event.type = EV_SYN;
	event.code = SYN_REPORT;
	event.value = 0;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	filp_close(fd, NULL);

	set_fs(fs);
#endif
	return 0;
}
EXPORT_SYMBOL(mhl_ms_btn);

int mhl_kbd_key(unsigned int tKeyCode, int value)	// keyboard key input, here because of speed
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	struct input_event event;
	struct file *fd;
	int ret;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	fd = filp_open(MHL_DEV_INPUT_KBMOUSE_EVENT, O_RDWR, 0);

	if (IS_ERR(fd)) {
		printk("mhl_kbd_key, could not open touch device \n");
	       set_fs(fs);

		return 1;
	}

	// TODO : supported key condition
	event.type = EV_KEY;
	event.code = tKeyCode;
	event.value = value;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	event.type = EV_SYN;
	event.code = SYN_REPORT;
	event.value = 0;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	filp_close(fd, NULL);

	set_fs(fs);
#else
	char mbuf[120];

	memset(mbuf, 0, sizeof(mbuf));
	sprintf(mbuf, "MHL_KEY press=%04d, keycode=%04d", value, tKeyCode);
					
	hdmi_common_send_uevent(mbuf);
#endif
	return 0;
}
EXPORT_SYMBOL(mhl_kbd_key);

#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
static int mhl_virtual_kbmouse_register(void)		// mouse & keyboard emulator...
{
	int rc;
	unsigned short key_cnt = 0x00;

	mhl_virtual_kbmouse = input_allocate_device();
	if (!mhl_virtual_kbmouse) {
		printk("%s: allocation failure for input device\n", __func__);
		return -ENOMEM;
	}

	mhl_virtual_kbmouse->name = DRIVER_NAME;
	mhl_virtual_kbmouse->id.bustype = 0x0000;
	mhl_virtual_kbmouse->id.vendor = 0x0000;
	mhl_virtual_kbmouse->id.product = 0x0001;
	mhl_virtual_kbmouse->id.version = 0x0100;

	mhl_virtual_kbmouse->evbit[0] = BIT_MASK(EV_KEY) |BIT_MASK(EV_REL);
	mhl_virtual_kbmouse->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT);
	mhl_virtual_kbmouse->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
	for(key_cnt = 0x01; key_cnt < KEY_REDO; key_cnt++)
	{
		set_bit(key_cnt, mhl_virtual_kbmouse->keybit);
	}

	rc = input_register_device(mhl_virtual_kbmouse);
	if (rc)
	{
		printk("%s: register failure for input device\n", __func__);
	}

	printk("%s: register success for input device\n", __func__);
	return rc;
}
#endif

void mhl_writeburst_uevent(unsigned short mev)
{
	char env_buf[120];

	memset(env_buf, 0, sizeof(env_buf));

	switch(mev)
	{
		case 1:
			sprintf(env_buf, "hdmi_kbd_on=");
			break;

		case 2:
			sprintf(env_buf, "hdmi_kbd_off=");
			break;

		default:

			break;
	}

       hdmi_common_send_uevent(env_buf);
	return;
}

EXPORT_SYMBOL(mhl_writeburst_uevent);
int MHLRCPtoKeyboard(u8 mhlrcpkey)	// for iot with TV and Monitor
{
	u8 r_action = 0;
	u8 r_keycode = 0x00;
	unsigned short keyCode = 0;

	printk("%s: MHL_RCP: %x\n", __func__, mhlrcpkey);

	if(mhlrcpkey & 0x80)	// release
	{
		r_action = 0;
	}
	else		// press
	{
		r_action = 1;
	}

	r_keycode = (mhlrcpkey & 0x7F);

	if(r_keycode < 0x0F)
	{
		switch(r_keycode)
		{
			case 0x00: keyCode = KEY_ENTER; 			break;
			case 0x01: keyCode = KEY_UP; 				break;
			case 0x02: keyCode = KEY_DOWN; 			break;
			case 0x03: keyCode = KEY_LEFT; 			break;
			case 0x04: keyCode = KEY_RIGHT; 			break;
			case 0x09: keyCode = KEY_MENU; 			break;
			case 0x0D: keyCode = KEY_BACK; 			break;
			default: 									break;
		}
	}
	else if((0x20 <= r_keycode) && (r_keycode<0x2D))
	{
		switch(r_keycode)
		{
			case 0x20: keyCode = KEY_0; 				break;
			case 0x21: keyCode = KEY_1; 				break;
			case 0x22: keyCode = KEY_2; 				break;
			case 0x23: keyCode = KEY_3; 				break;
			case 0x24: keyCode = KEY_4; 				break;
			case 0x25: keyCode = KEY_5; 				break;
			case 0x26: keyCode = KEY_6; 				break;
			case 0x27: keyCode = KEY_7; 				break;
			case 0x28: keyCode = KEY_8; 				break;
			case 0x29: keyCode = KEY_9; 				break;
			case 0x2A: keyCode = KEY_DOT; 			break;
			case 0x2B: keyCode = KEY_ENTER; 			break;
			case 0x2C: keyCode = KEY_BACKSPACE; 		break;
			default: 									break;
		}
	}
	else if((0x30 <= r_keycode) && (r_keycode<0x39))
	{
		switch(r_keycode)
		{
			case 0x37: keyCode = KEY_PAGEUP; 			break;
			case 0x38: keyCode = KEY_PAGEDOWN; 		break;
			default: 									break;
		}
	}
	else if((0x41 <= r_keycode) && (r_keycode<0x4D))
	{
		switch(r_keycode)
		{
			case 0x41: keyCode = KEY_VOLUMEUP; 		break;
			case 0x42: keyCode = KEY_VOLUMEDOWN;	break;
			case 0x43: keyCode = KEY_MUTE; 			break;
			case 0x44: keyCode = KEY_PLAY; 			break;
			case 0x45: keyCode = KEY_STOP; 			break;
			case 0x46: keyCode = KEY_PAUSE; 			break;
			case 0x47: keyCode = KEY_RECORD; 		break;
			case 0x48: keyCode = KEY_REWIND; 		break;
			case 0x49: keyCode = KEY_FASTFORWARD; 	break;
			case 0x4A: keyCode = KEY_EJECTCD; 		break;
			case 0x4B: keyCode = KEY_FORWARD; 		break;
			case 0x4C: keyCode = KEY_PREVIOUSSONG; 	break;
			default: 									break;
		}
		keyCode = 0x00;
	}
	else if((0x50 <= r_keycode) && (r_keycode<0x52))
	{
		// TODO
	}
	else if((0x60 <= r_keycode) && (r_keycode<0x69))
	{
		// TODO
	}
	else if((0x71 <= r_keycode) && (r_keycode<0x76))
	{
		// TODO
	}

	printk("%s: keyCode: %x, %s\n", __func__, keyCode, r_action? "pressed":"released");
	if(keyCode)
	{
		if(r_action == 1)
		{
			mhl_kbd_key(keyCode,r_action);
			mhl_kbd_key(keyCode,0);
		}
		return 1;
	}

	return 0;
}

#endif
irqreturn_t mhl_wake_up_irq_handler(int irq, void *dev_id)
{
    SII_LOG_FUNCTION_NAME_ENTRY;

    if (gpio_get_value(GPIO_MHL_SEL))	
        mhl_int_irq_handler_sched();
    
	SII_LOG_FUNCTION_NAME_EXIT;
	return IRQ_HANDLED;
}

static int sii9244_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* int retval; */

	struct sii9244_state *state;

	struct class *mhl_class;
	struct device *mhl_dev;
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);


	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244 attach success!!!\n");

	sii9244_i2c_client = client;

	MHL_i2c_init = 1;

	mhl_class = class_create(THIS_MODULE, "mhl");
	if (IS_ERR(mhl_class))
	{
		SII_DEV_DBG_ERROR("Failed to create class(mhl)!\n");
	}

	mhl_dev = device_create(mhl_class, NULL, 0, NULL, "mhl_dev");
	if (IS_ERR(mhl_dev))
	{
		SII_DEV_DBG_ERROR("Failed to create device(mhl_dev)!\n");
	}

	if (device_create_file(mhl_dev, &dev_attr_MHD_file) < 0)
		SII_DEV_DBG_ERROR("Failed to create device file(%s)!\n", dev_attr_MHD_file.attr.name);
//                                                                    
	if (device_create_file(mhl_dev, &dev_attr_rcp_test) < 0)
		SII_DEV_DBG_ERROR("Failed to create device file(rcp_test)!\n");
//                                                                     
//                                                                                      
#ifdef MHL_HW_DEBUG
	if (device_create_file(mhl_dev, &dev_attr_mhl_ctrl4) < 0)
		SII_DEV_DBG_ERROR("[MHL] Failed to create file (mhl_ctrl4)\n");
#endif
//                                                                                      
    SII_LOG_FUNCTION_NAME_EXIT;

	return 0;

}



static int __devexit sii9244_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    SII_LOG_FUNCTION_NAME_ENTRY;
    state = i2c_get_clientdata(client);

    //                                
    // ADD : for null defense
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;
}

static int sii9244a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244A attach success!!!\n");

	sii9244a_i2c_client = client;
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;

}



static int __devexit sii9244a_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
    
    state = i2c_get_clientdata(client);
    
    //                                
    // ADD : for null defense    
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;
}

static int sii9244b_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244B attach success!!!\n");

	sii9244b_i2c_client = client;

	SII_LOG_FUNCTION_NAME_EXIT;
	return 0;

}



static int __devexit sii9244b_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
    
    state = i2c_get_clientdata(client);
    
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
    
	return 0;
}


static int sii9244c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
	int ret;
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244C attach success!!!\n");

	sii9244c_i2c_client = client;

	msleep(100);	

	sii9244_wq = create_singlethread_workqueue("sii9244_wq");
	//INIT_WORK(&sii9244_int_work, sii9244_interrupt_event_work);

#if 0
	ret = request_threaded_irq(MHL_INT_IRQ, NULL, mhl_int_irq_handler,
				IRQF_SHARED , "mhl_int", (void *) state); 
#endif
	//                                                                            
	ret = request_threaded_irq(gpio_to_irq(GPIO_MHL_INT), NULL, mhl_int_irq_handler,
//            	IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "mhl_int", (void *) state); 
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "mhl_int", (void *) state); 
//                               

	if (ret) {
		SII_DEV_DBG_ERROR("unable to request irq mhl_int"
					" err:: %d\n", ret);
		return ret;
	}		
	SII_DEV_DBG("MHL int reques successful %d\n", ret);

#if 0
	ret = request_threaded_irq(MHL_WAKEUP_IRQ, NULL,
		mhl_wake_up_irq_handler, IRQF_SHARED,
		"mhl_wake_up", (void *) state);
#endif
//	ret = request_threaded_irq(gpio_to_irq(GPIO_MHL_WAKE_UP), NULL,
//		mhl_wake_up_irq_handler, IRQF_SHARED,
//		"mhl_wake_up", (void *) state);

//                                    
	//ret = request_irq(gpio_to_irq(GPIO_MHL_WAKE_UP), mhl_wake_up_irq_handler, IRQF_TRIGGER_FALLING | IRQF_SHARED, "mhl_wake_up", (void *) state); 
//                                  
//	if (ret < 0){
//		printk(KERN_INFO "[MHL INT] GPIO 160 IRQ line set up failed!\n");
//		free_irq(gpio_to_irq(GPIO_MHL_WAKE_UP), (void *) state);
//	}	

//                              

	if (ret) {
		SII_DEV_DBG_ERROR("unable to request irq mhl_wake_up"
					" err:: %d\n", ret);
		return ret;
	}		
#if defined(MHL_CTL) && defined(MHL_DEV_INPUT_KBMOUSE_EVENT)
	mhl_virtual_kbmouse_register();
#endif
	SII_LOG_FUNCTION_NAME_EXIT;

	return 0;

}



static int __devexit sii9244c_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
    
    state = i2c_get_clientdata(client);
#if defined(MHL_REGULATOR) /* convert gpio to regulator */
	if (mhl_reg) {
		regulator_put(mhl_reg);
	}
	if (mhl_1v2) {
		regulator_put(mhl_1v2);
	}
#endif
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;
}

struct i2c_driver sii9244_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9244",
	},
	.id_table	= sii9244_id,
	.probe	= sii9244_i2c_probe,
	.remove	= __devexit_p(sii9244_remove),
	.command = NULL,
};

struct i2c_driver sii9244a_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9244A",
	},
	.id_table	= sii9244a_id,
	.probe	= sii9244a_i2c_probe,
	.remove	= __devexit_p(sii9244a_remove),
	.command = NULL,
};

struct i2c_driver sii9244b_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9244B",
	},
	.id_table	= sii9244b_id,
	.probe	= sii9244b_i2c_probe,
	.remove	= __devexit_p(sii9244b_remove),
	.command = NULL,
};

struct i2c_driver sii9244c_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9244C",
	},
	.id_table	= sii9244c_id,
	.probe	= sii9244c_i2c_probe,
	.remove	= __devexit_p(sii9244c_remove),
	.command = NULL,
};


extern struct device * fimc_get_active_device(void);


void sii9244_cfg_power(bool on)
{
    SII_LOG_FUNCTION_NAME_ENTRY;
	if(on)
	{
#if defined(MHL_REGULATOR) /* convert gpio to regulator */
		int ret; 

		if (mhl_reg == NULL) {
			printk(KERN_ERR "MHL regulator is not initialized..\n");
			return;
		}

		ret = regulator_enable(mhl_reg);
		if (ret < 0) {
			printk(KERN_ERR "Failed to control regulator..\n");
			return;
		}
		if (mhl_1v2 == NULL) {
			printk(KERN_ERR "MHL 1V2 regulator is not initialized..\n");
			return;
		}
		ret = regulator_enable(mhl_1v2);
		if (ret < 0) {
			printk(KERN_ERR "Failed to control regulator mhl_1v2..\n");
			return;
		}
#else
		gpio_set_value(GPIO_MHL_EN, GPIO_LEVEL_HIGH);
#endif			
						
		mdelay(2);
		gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
		mdelay(2);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
		mdelay(2);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
		mdelay(2);		
	}
	else
	{
#if defined(MHL_REGULATOR) /* convert gpio to regulator */
		int ret; 

		if (mhl_reg == NULL) {
			printk(KERN_ERR "MHL regulator is not initialized..\n");
			return;
		}

		ret = regulator_disable(mhl_reg);
		if (ret < 0) {
			printk(KERN_ERR "Failed to control regulator..\n");
			return;
		}
		if (mhl_1v2 == NULL) {
			printk(KERN_ERR "MHL 1V2 regulator is not initialized..\n");
			return;
		}

		ret = regulator_disable(mhl_1v2);
		if (ret < 0) {
			printk(KERN_ERR "Failed to control regulator mhl_1v2..\n");
			return;
		}
//                                                                    
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
//                                         
#else
		gpio_set_value(GPIO_MHL_EN, GPIO_LEVEL_LOW);
#endif
	}
    SII_LOG_FUNCTION_NAME_EXIT;

	return;	
}


static void sii9244_cfg_gpio()
{
    SII_LOG_FUNCTION_NAME_ENTRY;
	gpio_request(GPIO_MHL_INT, "MHL_INT");
	gpio_direction_input(GPIO_MHL_INT);
	tegra_gpio_enable(GPIO_MHL_INT);
	gpio_request(GPIO_MHL_SEL, "MUIC/MHL SEL");
	gpio_direction_output(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
	tegra_gpio_enable(GPIO_MHL_SEL);


	gpio_request(GPIO_MHL_RST, "MHL_RESET_N");
	gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW);
	tegra_gpio_enable(GPIO_MHL_RST);
//                                                                    
	//gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
//                                         
#if defined(MHL_REGULATOR) /* convert gpio to regulator */
	if (mhl_reg != NULL) {
		printk(KERN_ERR "MHL regulator was already initialized..\n");
		return;
	}

	mhl_reg = regulator_get(NULL, "avdd_vhdmi_vtherm");
	if (IS_ERR_OR_NULL(mhl_reg)) {
		mhl_reg = NULL;
		printk(KERN_ERR "Failed to initialize mhl regulator..\n");
		return;
	}

	if (mhl_1v2 != NULL) {
		printk(KERN_ERR "MHL 1V2 regulator was already initialized..\n");
		return;
	}

	mhl_1v2 = regulator_get(NULL, "mhl_1v2");
	if (IS_ERR_OR_NULL(mhl_1v2)) {
		mhl_1v2 = NULL;
		printk(KERN_ERR "Failed to initialize mhl_1v2 regulator..\n");
		return;
	}
#else
	gpio_request(GPIO_MHL_EN, "MHL_EN");
	gpio_direction_output(GPIO_MHL_EN, GPIO_LEVEL_LOW);
	tegra_gpio_enable(GPIO_MHL_EN);
#endif 

#if defined(MHL_REGULATOR)
	/*no need to disable it, for regulator avdd_vhdmi_vtherm init_state is already 0. */
	//regulator_disable(mhl_reg);
#else
	gpio_set_value(GPIO_MHL_EN, GPIO_LEVEL_LOW);
#endif
    SII_LOG_FUNCTION_NAME_EXIT;
}

static int mhl_open(struct inode *ip, struct file *fp)
{
	SII_DEV_DBG("\n");
	return 0;

}

static int mhl_release(struct inode *ip, struct file *fp)
{
	
	SII_DEV_DBG("\n");
	return 0;
}


static int mhl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	SII_DEV_DBG("\n");

#if 0
	byte data;

	switch(cmd)
	{
		case MHL_READ_RCP_DATA:
			data = GetCbusRcpData();
			ResetCbusRcpData();
			put_user(data,(byte *)arg);
			printk(KERN_ERR "MHL_READ_RCP_DATA read");
			break;
		
		default:
		break;
	}
#endif		
	return 0;
}

static struct file_operations mhl_fops = {
	.owner  = THIS_MODULE,
	.open   = mhl_open,
    	.release = mhl_release,
    	//.ioctl = mhl_ioctl,
};
                 
static struct miscdevice mhl_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "mhl",
    .fops   = &mhl_fops,
};

static int __init sii9244_module_init(void)
{
	int ret;
    SII_LOG_FUNCTION_NAME_ENTRY;
	sii9244_cfg_gpio();

	/* sii9244_cfg_power(1);	//Turn On power to sii9244 
	*/

#ifdef MHL_SWITCH_TEST
	sec_mhl = class_create(THIS_MODULE, "sec_mhl");
	if (IS_ERR(sec_mhl))
		SII_DEV_DBG_ERROR("[MHL] Failed to create class (sec_mhl)\n");

	mhl_switch = device_create(sec_mhl, NULL, 0, NULL, "switch");
	if (IS_ERR(mhl_switch))
		SII_DEV_DBG_ERROR("[MHL] Failed to create device (mhl_switch)\n");
	if (device_create_file(mhl_switch, &dev_attr_mhl_sel) < 0)
		SII_DEV_DBG_ERROR("[MHL] Failed to create file (mhl_sel)\n");
#endif

	ret = misc_register(&mhl_device);
	if(ret) {
		SII_DEV_DBG_ERROR("misc_register failed - mhl \n");
	}

	ret = i2c_add_driver(&sii9244_i2c_driver);
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR("[MHL SII9244] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244] add i2c driver\n");
    }
    
	ret = i2c_add_driver(&sii9244a_i2c_driver);
    
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR( "[MHL SII9244A] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244A] add i2c driver\n");
    }
    
	ret = i2c_add_driver(&sii9244b_i2c_driver);
    
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR("[MHL SII9244B] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244B] add i2c driver\n");
    }

	ret = i2c_add_driver(&sii9244c_i2c_driver);
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR("[MHL SII9244C] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244C] add i2c driver\n");
    }

//                                                           
	wake_lock_init(&mhl_lock, WAKE_LOCK_SUSPEND, "mhl_wake_lock");
//                                                           
    sii9244_driver_init();
    SII_LOG_FUNCTION_NAME_EXIT;

	return ret;	
}
module_init(sii9244_module_init);
static void __exit sii9244_exit(void)
{
	i2c_del_driver(&sii9244_i2c_driver);
	i2c_del_driver(&sii9244a_i2c_driver);
	i2c_del_driver(&sii9244b_i2c_driver);	
	i2c_del_driver(&sii9244c_i2c_driver);

	//                                                           
	wake_lock_destroy(&mhl_lock);
	//                                                           
};
module_exit(sii9244_exit);

MODULE_DESCRIPTION("Sii9244 MHL driver");
MODULE_AUTHOR("Jerry Koo");
MODULE_LICENSE("GPL");
