/* drivers/input/keyboard/lge_touch_driver.c
 *
 * Copyright (C) 2011 LGE. 
 * 
 * Writer: yehan.ahn@lge.com
 *
 * Most lines are copied from touch_driver of STAR-P999.
 * That is written by joseph.jung@lge.com and taewan.kim@lge.com.
 * Please check the STAR-P999 or contact writers if you want more information about original source code.
 *
 * This file is used by LGE_touch_deriver.c.
 * it is related with specific touch-device.
 * You should check the document or contact the writer in order to understand whole process.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/semaphore.h>
#include <linux/irq.h>
#include <linux/freezer.h>

#include <linux/input/lge_touch_synaptics.h>
#include <linux/input/lge_touch_driver.h>

struct touch_device_caps
{
    bool 	IsMultiTouchSupported; // single touch[0] / multi-touch[1]
    bool	IsButtonSupported; // yes[1] / no[0]
    u32 	MaxNumberOfFingerCoordReported;
    bool	IsRelativeDataSupported; // support[1] / not[0]
    u32 	MaxNumberOfRelativeCoordReported;
    u32 	MaxNumberOfWidthReported;
    u32 	MaxNumberOfPressureReported;
    u32 	Gesture;
    bool 	IsWidthSupported;
    bool 	IsPressureSupported;
    bool 	IsFingersSupported;
    u32 	XMinPosition;
    u32 	YMinPosition;
    u32 	XMaxPosition;
    u32 	YMaxPosition;
    u32 	Orientation;
};

struct synaptics_ts_data {
	struct touch_device_caps caps;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct semaphore sem;
	struct task_struct*	task;
	u32 gpio;
	u32 irq_gpio;
	u32 flags;
	u32 reset;
	u32 touch;
	int (*power)(int on);	
};

struct ts_sensor_data
{
	u8 device_status_reg;						//0x13
	u8 interrupt_status_reg;					//0x14
	u8 finger_state_reg[3];						//0x15~0x17

	u8 fingers_data[SYNAPTICS_FINGER_MAX][5];				//0x18 ~ 0x49

	u8 gesture_flag0;							//0x4A
	u8 gesture_flag1;							//0x4B
	u8 pinch_motion_X_flick_distance;			//0x4C
	u8 rotation_motion_Y_flick_distance;		//0x4D
	u8 finger_separation_flick_time;			//0x4E
};

static const struct touch_device_caps Synaptics_Capabilities =
{
	1,	//IsMultiTouchSupported
	1, 	//isButtonSupported
	10,	//MaxNumberOfFingerCoordReported;
	0,	//IsRelativeDataSupported
	0,	//MaxNumberOfRelativeCoordReported
	15,	//MaxNumberOfWidthReported
	0xFF,	//MaxNumberOfPressureReported
	0,	// Gesture
	0,	//IsWidthSupported
	0,	//IsPressureSupported, mandatory for multi-touch
	0,	//IsFingersSupported
	0,	//XMinPosition
	0,	//YMinPosition
	LGE_TOUCH_RESOLUTION_X + 1, //XMaxPosition
	LGE_TOUCH_RESOLUTION_Y + 1, //YMaxPosition
	0,
};

struct button_area{
	u8		b_name;
	u16		b_left;
	u16		b_right;
};

static struct ts_sensor_data ts_reg_data={0};



#define GET_BIT_MASK(_finger_state_reg)	\
		(_finger_state_reg[2] & 0x04)<<7 | (_finger_state_reg[2] & 0x01)<<8 |	\
		(_finger_state_reg[1] & 0x40)<<1 | (_finger_state_reg[1] & 0x10)<<2 |(_finger_state_reg[1] & 0x04)<<3 | (_finger_state_reg[1] & 0x01)<<4 |	\
		(_finger_state_reg[0] & 0x40)>>3 | (_finger_state_reg[0] & 0x10)>>2 |(_finger_state_reg[0] & 0x04)>>1 | (_finger_state_reg[0] & 0x01)

#define GET_INDEX_FROM_MASK(_index, _bit_mask)	\
for(; !((_bit_mask>>_index)&0x01) && _index <= SYNAPTICS_FINGER_MAX; _index++);	\
if(_index <= SYNAPTICS_FINGER_MAX) _bit_mask &= ~(_bit_mask & (1<<(_index)));


#define TS_SNTS_GET_X_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x000007F0) | (u16)(_low_reg&0x0F)))
#define TS_SNTS_GET_Y_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x000007F0) | (u16)((_low_reg >> 4) & 0x0F)))
#define TS_SNTS_GET_WIDTH(_width) \
		((((_width & 0xf0) >> 4) - (_width & 0x0f)) > 0)? (_width & 0xf0) >> 4 : _width & 0x0f
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure

enum {X_HIGH_POSITION=0, Y_HIGH_POSITION, XY_LOW_POSITION, XY_WIDTH, PRESSURE};

#define BUTTON_WIDTH		280
#define BUTTON_NULL_WIDTH	30
#define IS_MENU(_x)			(_x > BUTTON_WIDTH*0 + BUTTON_NULL_WIDTH  && _x < BUTTON_WIDTH*1-BUTTON_NULL_WIDTH)
#define IS_HOME(_x)			(_x > BUTTON_WIDTH*1 + BUTTON_NULL_WIDTH  && _x < BUTTON_WIDTH*2-BUTTON_NULL_WIDTH)
#define IS_BACK(_x)			(_x > BUTTON_WIDTH*2 + BUTTON_NULL_WIDTH  && _x < BUTTON_WIDTH*3-BUTTON_NULL_WIDTH)
#define IS_SEARCH(_x)		(_x > BUTTON_WIDTH*3 + BUTTON_NULL_WIDTH  && _x < BUTTON_WIDTH*4-BUTTON_NULL_WIDTH)
#define IS_PANEL(_y)		(_y >= 0  && _y <= LGE_TOUCH_RESOLUTION_Y)


static const struct button_area button_area_for_cancel[] = 
{
	{KEY_MENU, 0, BUTTON_WIDTH+BUTTON_NULL_WIDTH},
	{KEY_HOME, BUTTON_WIDTH-BUTTON_NULL_WIDTH, BUTTON_WIDTH*2+BUTTON_NULL_WIDTH},
	{KEY_BACK, BUTTON_WIDTH*2-BUTTON_NULL_WIDTH, BUTTON_WIDTH*3+BUTTON_NULL_WIDTH},
	{KEY_SEARCH, BUTTON_WIDTH*3-BUTTON_NULL_WIDTH, LGE_TOUCH_RESOLUTION_X},
};


/* TOUCH_ADJUST_POSITION
 * 
 *  make line more smoothly.
 */
#if defined(TOUCH_ADJUST_POSITION)
#define ADJUST_LEVEL 5
#define SQUARE(x)		((x) * (x))

const u16 ADJUST_FACTOR_LEVEL[ADJUST_LEVEL] = {8,6,4,2,1};
const u16 ADJUST_BASIS_LEVEL[ADJUST_LEVEL] = {5,11,19,29,40};
const u16 ADJUST_FACTOR_BASE = 4;

static void touch_adjust_position(const u16 x_value, const u16 y_value, u16 *adjust_X, u16 *adjust_Y)
{
	u16 distant = int_sqrt(SQUARE(*adjust_X - x_value) + SQUARE(*adjust_Y - y_value));
	u16 i;

	for(i=0; i<ADJUST_LEVEL; i++){
		if(distant <= ADJUST_BASIS_LEVEL[i]){
			*adjust_X = (x_value * ADJUST_FACTOR_LEVEL[i] + *adjust_X * ADJUST_FACTOR_BASE) 
						/ (ADJUST_FACTOR_LEVEL[i] + ADJUST_FACTOR_BASE);
			*adjust_Y = (y_value * ADJUST_FACTOR_LEVEL[i] + *adjust_Y * ADJUST_FACTOR_BASE) 
						/ (ADJUST_FACTOR_LEVEL[i] + ADJUST_FACTOR_BASE);
			break;
		}
	}
	
	if(*adjust_Y < LGE_TOUCH_RESOLUTION_Y+10 && *adjust_Y >= LGE_TOUCH_RESOLUTION_Y)
		*adjust_Y = LGE_TOUCH_RESOLUTION_Y-1;

}
#endif


/* TOUCH_GRIP_SUPPRESSION
 * 
 *  remove edge-touch-area for user's grip.
 */
#if defined(TOUCH_GRIP_SUPPRESSION)
#define READ_BUFFER_LENGTH 20
u8 touch_grip_suppression_value = 0;

#define IGNORE_IF_POSITION_IS_SUPPRESSION_AREA(_x)	\
if(_x < touch_grip_suppression_value || _x > LGE_TOUCH_RESOLUTION_X - touch_grip_suppression_value)	\
	continue;

ssize_t touch_gripsuppression_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", touch_grip_suppression_value);	
	return (ssize_t)(strlen(buf)+1);
}

ssize_t touch_gripsuppression_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	u8 *input;
	u32 value;

	count = count > READ_BUFFER_LENGTH ? READ_BUFFER_LENGTH : count;
	input = kzalloc(((int)count+1), GFP_KERNEL);
	if (!input) {
		return 0;
	}

	memcpy(input, buffer, count);

	input[count] = '\0';
	value = simple_strtoul(&input[0], '\0', 10);

	touch_grip_suppression_value = value;

	kfree(input);
	return count;
}

DEVICE_ATTR(gripsuppression, 0666, touch_gripsuppression_show, touch_gripsuppression_store);
#endif



void device_close (void* h_dev)
{
    struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

#if defined(TOUCH_GRIP_SUPPRESSION)
	device_remove_file(&hTouch->client->dev, &dev_attr_gripsuppression);
#endif

    kfree(hTouch);
}

bool device_open(void** h_touch, struct i2c_client *client, u32 touch)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)0;
	struct p940_synaptics_platform_data *pdata;
	DO_A(hTouch = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL),  err_Device_Open);

    memset(hTouch, 0, sizeof(struct synaptics_ts_data));

	DO_F(i2c_check_functionality(client->adapter, I2C_FUNC_I2C), err_Device_Open);
	hTouch->client = client;

	DO_A(pdata = client->dev.platform_data, err_Device_Open);

	hTouch->flags = pdata->irqflags;
	hTouch->gpio = pdata->gpio;
	hTouch->reset = pdata->reset;
	hTouch->irq_gpio  = gpio_to_irq(pdata->gpio);
	hTouch->power = pdata->power;
	hTouch->touch = touch;

	
	gpio_request(hTouch->reset,"synaptics_ts_reset");
	gpio_direction_output(hTouch->reset, 1);	// OUTPUT 
//                                             
	tegra_gpio_enable(hTouch->reset);
	gpio_set_value(hTouch->reset,1);
//                                             

	memcpy(&hTouch->caps, &Synaptics_Capabilities, sizeof(struct touch_device_caps));

#if defined(TOUCH_GRIP_SUPPRESSION)
	DO_C(device_create_file(&client->dev, &dev_attr_gripsuppression), err_Device_Open);
#endif	

	*h_touch = (void *)hTouch;
  	return true;

err_Device_Open:
	device_close((void* )hTouch);
	return false;
} 

bool inputDevOpen(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	struct input_dev* input_dev;
	
	DO_A(hTouch->input_dev = input_allocate_device(), err_inputDevOpen2);
	input_dev = hTouch->input_dev;
	input_dev->name = LGE_TOUCH_NAME;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, hTouch->caps.XMinPosition, hTouch->caps.XMaxPosition, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, hTouch->caps.YMinPosition, hTouch->caps.YMaxPosition, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, hTouch->caps.MaxNumberOfPressureReported , 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, hTouch->caps.MaxNumberOfWidthReported, 0, 0);

	DO_F(!input_register_device(input_dev), err_inputDevOpen);
	return true;

err_inputDevOpen:
	input_unregister_device(hTouch->input_dev);
err_inputDevOpen2:
	input_free_device(hTouch->input_dev);
	return false;
}

void inputDevClose(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	
	input_unregister_device(hTouch->input_dev);
	input_free_device(hTouch->input_dev);
}

bool inputDevSendABS(void* h_dev, struct touch_data data, u8 isPress)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->input_dev, err_inputDevSendABS);
	if(isPress){
		input_report_abs(hTouch->input_dev, ABS_MT_POSITION_X, data.X_position);
		input_report_abs(hTouch->input_dev, ABS_MT_POSITION_Y, data.Y_position);
		input_report_abs(hTouch->input_dev, ABS_MT_TOUCH_MAJOR, data.pressure);
		input_report_abs(hTouch->input_dev, ABS_MT_WIDTH_MAJOR, data.width);
	}
	input_mt_sync(hTouch->input_dev);
	
	return true;

err_inputDevSendABS:
	return false;
}

bool inputDevSendABSMulti(void* h_dev, struct touch_data* data, u8 total_num)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	int i=0, check=0;
	
	DO_A(hTouch->input_dev, err_inputDevSendABSMulti);

	for(i=0; i<total_num; i++){
		if(IS_PANEL(data[i].Y_position)){
			input_report_abs(hTouch->input_dev, ABS_MT_POSITION_X, data[i].X_position);
			input_report_abs(hTouch->input_dev, ABS_MT_POSITION_Y, data[i].Y_position);
			input_report_abs(hTouch->input_dev, ABS_MT_TOUCH_MAJOR, data[i].pressure);
			input_report_abs(hTouch->input_dev, ABS_MT_WIDTH_MAJOR, data[i].width);
			input_mt_sync(hTouch->input_dev);
			DEBUG_MSG(M, "[TOUCH] X[%d], Y[%d]\n", (int)data[i].X_position, (int)data[i].Y_position);
			check++;
		}
	}

	if(!check){
		DEBUG_MSG(M, "[TOUCH] mt_sync. \n");
		input_mt_sync(hTouch->input_dev);
	}
	
	return true;

err_inputDevSendABSMulti:
	return false;
}


bool inputDevSendButton(void* h_dev, u32 type, u32 isPress)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->input_dev, err_inputDevSendButton);

	input_report_key(hTouch->input_dev, type, isPress);

	return true;

err_inputDevSendButton:
	return false;
}

bool inputDevSendSync(void* h_dev, u8 state)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->input_dev, err_inputDevSendSync);

	if(state != TOUCH_BUTTON_LOCK && state != TOUCH_ABS_LOCK){
		input_sync(hTouch->input_dev);
		DEBUG_MSG(M, "[TOUCH] sync.\n");
	}

	return true;

err_inputDevSendSync:
	return false;
}

void init_touch_inputdev(void* h_dev, struct touch_inputdev_func* h_inputDev)
{
	h_inputDev->open = inputDevOpen;
	h_inputDev->close = inputDevClose;
	h_inputDev->send_ABS = inputDevSendABS;
	h_inputDev->send_ABS_Multi = inputDevSendABSMulti;
	h_inputDev->send_Button = inputDevSendButton;
	h_inputDev->send_Sync = inputDevSendSync;
}

bool taskOpen(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->task = kthread_create(task_handler, (void*)hTouch->touch, "LGE_touch_thread"), err_taskOpen);
	return true;

err_taskOpen:
	return false;	
}

void taskClose(void* h_dev)
{

}

bool taskStart(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->task, err_taskStart);
	wake_up_process( hTouch->task );
	return true;
	
err_taskStart:
	return false;
}

bool taskStop(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_A(hTouch->task, err_taskStop);
	kthread_stop(hTouch->task);
	return true;

err_taskStop:
	return false;
}

void init_touch_task(void* h_dev, struct touch_task_func* h_task)
{
	h_task->open = taskOpen;
	h_task->close = taskClose;
	h_task->start = taskStart;
	h_task->stop = taskStop;
}

void interruptClose(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	free_irq(hTouch->irq_gpio, (void*)hTouch->touch);
}

bool interruptOpen(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	init_MUTEX_LOCKED(&hTouch->sem);

	gpio_request(hTouch->gpio,"synaptics_ts_irq");
	gpio_direction_input(hTouch->gpio);

//                                             
	tegra_gpio_enable(hTouch->gpio);
	mdelay(20);
//                                             
	
	DO_F(!request_irq(hTouch->irq_gpio, interrupt_handler, hTouch->flags, LGE_TOUCH_NAME, (void*)hTouch->touch), err_interruptOpen);

	return true;

err_interruptOpen:
	interruptClose((void*)hTouch);
	return false;
}

bool interruptEnable(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_C(!hTouch->irq_gpio, err_interruptEnable);
	enable_irq(hTouch->irq_gpio);
	return true;
	
err_interruptEnable:
	return false;
}

bool interruptDisable(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_C(!hTouch->irq_gpio, err_interruptDisable);
	disable_irq_nosync(hTouch->irq_gpio);
	return true;
	
err_interruptDisable:
	return false;
}

bool interruptStart(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

    up( &hTouch->sem );
	
	return true;
}

bool interruptWait(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	int ret;

	do{
       ret = down_interruptible(&hTouch->sem);
       if (ret && !try_to_freeze())
            schedule();
    } while (ret);
	return true;
}

void init_touch_interrupt(void* h_dev, struct touch_interrupt_func* h_interrupt)
{
	h_interrupt->open = interruptOpen;
	h_interrupt->close = interruptClose;
	h_interrupt->enable = interruptEnable;
	h_interrupt->disable = interruptDisable;
	h_interrupt->start = interruptStart;
	h_interrupt->wait = interruptWait;
}

bool wait_touch_power_ready(struct i2c_client *client, int num)				
{															
	int retry = num;										
	while (retry-- > 0) {									
		int ret = i2c_smbus_read_byte_data(client, 0xb8);	
		if (ret >= 0)										
			return true;											
		msleep(100);											
	}
	return false;
}

bool powerOn(void* h_dev)
{	
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	if(hTouch->power)
		hTouch->power(1);
	mdelay(100);
	
	gpio_set_value(hTouch->reset, 0);
	mdelay(100);
	gpio_set_value(hTouch->reset, 1);

	DO_F(wait_touch_power_ready(hTouch->client, 10), err_powerOn);	

	return true;
	
err_powerOn:
	return false;
}

void powerOff(void* h_dev)
{	
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	if(hTouch->power)
		hTouch->power(0);
}

bool powerSleep(void* h_dev)
{	
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

//	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DEVICE_CONTROL_REG, SYNAPTICS_CONTROL_SLEEP), err_powerSleep);

	if(hTouch->power)
		hTouch->power(0);
	
	return true;
err_powerSleep:
	return false;
}
bool powerWake(void* h_dev)
{	
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

//	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DEVICE_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP), err_powerWake);

	if(hTouch->power)
		hTouch->power(1);
	mdelay(100);
	
	gpio_set_value(hTouch->reset, 0);
	mdelay(100);
	gpio_set_value(hTouch->reset, 1);

	DO_F(wait_touch_power_ready(hTouch->client, 10), err_powerWake);	
	
	return true;
err_powerWake:
	return false;
}


void init_touch_power(void* h_dev, struct touch_power_func* h_powerCtrl)
{
	h_powerCtrl->on = powerOn;
	h_powerCtrl->off = powerOff;
	h_powerCtrl->sleep = powerSleep;
	h_powerCtrl->wake = powerWake;
}

bool Synaptics_GetData (void* h_dev, struct touch_finger_data* data)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;
	u16 touch_finger_bit_mask=0;
	u8  finger_index=0;
	u8  index=0;
	
	DO_C(i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_INT_STATUS_REG, sizeof(u8)*4, (u8*)(&ts_reg_data.interrupt_status_reg)) != sizeof(u8)*4, err_Synaptics_GetData);
	DEBUG_MSG(M, "[TOUCH] i[%d], 0[%d], 1[%d], 2[%d]", (int)ts_reg_data.interrupt_status_reg, (int)ts_reg_data.finger_state_reg[0], (int)ts_reg_data.finger_state_reg[1], (int)ts_reg_data.finger_state_reg[2]);
		

	if(ts_reg_data.interrupt_status_reg == SYNAPTICS_INT_ABS0){
		touch_finger_bit_mask = GET_BIT_MASK(ts_reg_data.finger_state_reg);
		
		while(touch_finger_bit_mask){
			GET_INDEX_FROM_MASK(finger_index, touch_finger_bit_mask)
			DO_C(i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_REG_FINGER_DATA_START_ADDR + (SYNAPTICS_REG_FINGER_DATA_GAP*finger_index), 
										       SYNAPTICS_REG_FINGER_VALID_DATA_SIZE, ts_reg_data.fingers_data[index]) != SYNAPTICS_REG_FINGER_VALID_DATA_SIZE, err_Synaptics_GetData);

			data->curr_data[index].X_position = TS_SNTS_GET_X_POSITION(ts_reg_data.fingers_data[index][X_HIGH_POSITION], ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
			data->curr_data[index].Y_position = TS_SNTS_GET_Y_POSITION(ts_reg_data.fingers_data[index][Y_HIGH_POSITION], ts_reg_data.fingers_data[index][XY_LOW_POSITION]); 	
			data->curr_data[index].width	= TS_SNTS_GET_WIDTH(ts_reg_data.fingers_data[index][XY_WIDTH]);
			data->curr_data[index].pressure	= TS_SNTS_GET_PRESSURE(ts_reg_data.fingers_data[index][PRESSURE]);

			DEBUG_MSG(M, "[TOUCH] X[%d], Y[%d], Press[%d], Width[%d]", (int)data->curr_data[index].X_position, (int)data->curr_data[index].Y_position, (int)data->curr_data[index].width, (int)data->curr_data[index].pressure);

#if defined(TOUCH_GRIP_SUPPRESSION)
			IGNORE_IF_POSITION_IS_SUPPRESSION_AREA(data->curr_data[index].X_position)
#endif		
			index++;
		}
		data->total_num = index;
	}

	return true;

err_Synaptics_GetData:
	return false;
}

static bool IS_CANCEL(u8 prev_button, u16 x, u16 y)
{	
	int i;
	
	for(i=0; i<ARRAY_SIZE(button_area_for_cancel); i++){
		if(button_area_for_cancel[i].b_name == prev_button)
			break;
	}

	if(i >= ARRAY_SIZE(button_area_for_cancel))	
		return false;

	if(y > 1800 && x > button_area_for_cancel[i].b_left && x < button_area_for_cancel[i].b_right)
		return false;
	
	return true;
}

u8 Synaptics_CheckButton(void* h_dev, struct touch_finger_data* data)
{
	u8 tmp_button = KEY_NULL;
	u8 sync = DO_NOT_ANYTHING;
	u8 total_num = data->total_num;
	u8 prev_button = data->prev_button;
	u8 state = data->state;
	struct touch_data curr_data = data->curr_data[0];

	if(total_num == FINGER_RELEASE) 
		goto err_its_release;
	if(state == TOUCH_BUTTON_LOCK || state == TOUCH_ABS_LOCK) 
		goto err_its_lock;
	if(total_num != SINGLE_FINGER) 
		goto err_its_multi;

	if(IS_PANEL(curr_data.Y_position))			tmp_button = KEY_PANEL;
	else if(IS_MENU(curr_data.X_position))		tmp_button = KEY_MENU;
	else if(IS_HOME(curr_data.X_position))		tmp_button = KEY_HOME;
	else if(IS_BACK(curr_data.X_position))		tmp_button = KEY_BACK;
	else if(IS_SEARCH(curr_data.X_position))	tmp_button = KEY_SEARCH;
	else										tmp_button = KEY_BOUNDARY;

	if(prev_button != KEY_NULL && prev_button != KEY_BOUNDARY){
		if(prev_button == KEY_PANEL){
			if(prev_button != tmp_button) 
				sync = ABS_RELEASE;
			else 
				sync = ABS_PRESS;
		}
		else{
			if(IS_CANCEL(prev_button, curr_data.X_position, curr_data.Y_position))
				sync = BUTTON_CANCEL;
			else 
				sync = DO_NOT_ANYTHING;
		}
	}
	else{
		if(tmp_button == KEY_PANEL) 
			sync = ABS_PRESS;
		else if(tmp_button == KEY_BOUNDARY) 
			sync = DO_NOT_ANYTHING;
		else 
			sync = BUTTON_PRESS;
	}

	if(sync == ABS_PRESS || sync == BUTTON_PRESS)
		data->curr_button = tmp_button;
	else if(sync == BUTTON_RELEASE || sync == BUTTON_CANCEL || sync == ABS_RELEASE)
		data->curr_button = KEY_NULL;
	else;
	
	return sync;

err_its_multi:
	data->curr_button = KEY_PANEL;
	if(prev_button && prev_button != KEY_PANEL)	
		return BUTTON_CANCEL;
	else 
		return ABS_PRESS;

err_its_lock:
	if(state == TOUCH_BUTTON_LOCK)
		return TOUCH_BUTTON_LOCK;
	else{ // if(state == TOUCH_ABS_LOCK){
		if(IS_PANEL(curr_data.Y_position)){
			data->curr_button = KEY_PANEL;
			return ABS_PRESS;
		}
		return TOUCH_ABS_LOCK;
	}
		
err_its_release:
	data->curr_button = KEY_NULL;
	if(prev_button){
		if(prev_button == KEY_PANEL) 
			return ABS_RELEASE;
		else 
			return BUTTON_RELEASE;
	}
	else
		return DO_NOT_ANYTHING;	
}

bool Synaptics_AddJob (void* h_dev, struct touch_finger_data* data, u32 whereis)
{
	int i=0;
	switch(whereis){
		case BEFORE_WHILE:
			set_freezable_with_signal();
			break;
		case AFTER_GET_DATA:	
#if defined(TOUCH_ADJUST_POSITION)
			for(i=0; i<data->total_num; i++){
				touch_adjust_position(data->prev_data[i].X_position, data->prev_data[i].Y_position, 
									  &data->curr_data[i].X_position, &data->curr_data[i].Y_position);
			}
#endif
			break;
		case AFTER_SYNC:
			if(data->total_num == SINGLE_FINGER){
				if(data->state == ABS_RELEASE)
					data->state = TOUCH_ABS_LOCK;
				else if(data->state == BUTTON_RELEASE || data->state == BUTTON_CANCEL)
					data->state = TOUCH_BUTTON_LOCK;
				else;
			}
			break;
		default:
			break;
	}
	return true;
}

void init_touch_finger_data(void* h_dev, struct touch_finger_data_func* h_fingerData)
{
	h_fingerData->get_finger_data = Synaptics_GetData;
	h_fingerData->check_button = Synaptics_CheckButton;
	h_fingerData->additional_job = Synaptics_AddJob;
}

#if defined(TOUCH_FW_UPGRADE)

#include <linux/input/synaptics_ts_firmware.h>

#define TOUCH_FW_COMPARE

static fw_version = 0;

#define WAIT_UNTIL_PIN_READY(_pin)	\
{	\
	while(gpio_get_value(_pin)){	\
		mdelay(1);	\
	}	\
}

#define WAIT_UNTIL_FLASH_CMD_READY(_cond)\
{	\
	u8 flashValue, temp_data;	\
	do{	\
		flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);	\
		temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);	\
	} while(_cond);\
}

#define WAIT_UNTIL_DEVICE_READY(_cond, _pin)\
		WAIT_UNTIL_PIN_READY(_pin)	\
		WAIT_UNTIL_FLASH_CMD_READY(_cond)

#define I2C_R(_state)	DO_SAFE(_state, "I2C READ", return -1)
#define I2C_W(_state)	DO_SAFE(_state, "I2C WRITE", return -1)


static u8 Synaptics_GetFWVersion(struct i2c_client *client)
{
	u8 RMI_Query_BaseAddr;
	u8 FWVersion_Addr;

	u8 SynapticsFirmVersion;

	I2C_R((RMI_Query_BaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_QUERY_BASE_REG)) < 0)
	FWVersion_Addr = RMI_Query_BaseAddr+3;
	I2C_R((SynapticsFirmVersion = i2c_smbus_read_byte_data(client, FWVersion_Addr)) < 0)

	DEBUG_MSG(3, "[TOUCH FW] synaptics_GetFWVersion = %x, [%x,%x]\n", SynapticsFirmVersion, RMI_Query_BaseAddr, FWVersion_Addr)

	return SynapticsFirmVersion;
}

static unsigned long ExtractLongFromHeader(const u8 *SynaImage)  // Endian agnostic 
{
  return((unsigned long)SynaImage[0] +
         (unsigned long)SynaImage[1]*0x100 +
         (unsigned long)SynaImage[2]*0x10000 +
         (unsigned long)SynaImage[3]*0x1000000);
}

static void CalculateChecksum(u16 *data, u16 len, u32 *dataBlock)
{
  unsigned long temp = *data++;
  unsigned long sum1;
  unsigned long sum2;

  *dataBlock = 0xffffffff;

  sum1 = *dataBlock & 0xFFFF;
  sum2 = *dataBlock >> 16;

  while (len--)
  {
    sum1 += temp;    
    sum2 += sum1;    
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);    
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }

  *dataBlock = sum2 << 16 | sum1;
}

static void SpecialCopyEndianAgnostic(u8 *dest, u16 src) 
{
  dest[0] = src%0x100;  //Endian agnostic method
  dest[1] = src/0x100;  
}

#ifdef TOUCH_FW_COMPARE
static int fw_compare(struct i2c_client *client, const u8 BlockDataStartAddr, u16 index, const u16 block_size)
{
	u8  *tmp_block = kmalloc(sizeof(u8)*block_size, GFP_KERNEL);
	u8	i;
	
	if(i2c_smbus_read_i2c_block_data(client, BlockDataStartAddr, sizeof(tmp_block), tmp_block) < sizeof(tmp_block)){
		kfree(tmp_block);
		return -1;
	}
	
	for(i=0; i<sizeof(tmp_block); i++){
		if(unlikely(tmp_block[i] != SynapticsFirmware[index])){
			kfree(tmp_block);
			return -1;
		}
		DEBUG_MSG(1, "[TOUCH FW] [%x] : tmp[%x] / Firm[%x]\n", i, tmp_block[i], SynapticsFirmware[index]);
		index++;
	}

	//mdelay(100);
	kfree(tmp_block);
	return 0;
}
#endif

static int synaptics_ts_fw_upgrade(struct synaptics_ts_data* hTouch)
{
	struct i2c_client *client = hTouch->client;
	
	int i;
	
	u8 TouchFWVersion;

	u8 FlashQueryBaseAddr, FlashDataBaseAddr;
	u8 RMICommandBaseAddr;
	
	u8 BootloaderIDAddr;
	u8 BlockSizeAddr;
	u8 FirmwareBlockCountAddr;
	u8 ConfigBlockCountAddr;

	u8 BlockNumAddr;
	u8 BlockDataStartAddr;
	
	u8 bootloader_id[2];

	u8 temp_array[2], temp_data, m_firmwareImgVersion;
	u8 checkSumCode;

	u16 ts_block_size, ts_config_block_count, ts_fw_block_count;
	u16 m_bootloadImgID;
	
	u32 ts_config_img_size;
	u32 ts_fw_img_size;
	u32 m_fileSize, m_firmwareImgSize, m_configImgSize, m_FirmwareImgFile_checkSum;

	u8 RMI_Query_BaseAddr;
	u8 product_id_addr;

	u8 product_id[7];

	////////////////////////////////////////////////////////////////////////////////////

	DEBUG_MSG(3, "[TOUCH FW] Synaptics_UpgradeFirmware :: TM1576 [START]\n")


	////////////////////////	Product ID Check	///////////////////////////
	
	
	I2C_R((RMI_Query_BaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_QUERY_BASE_REG)) < 0)
	product_id_addr = RMI_Query_BaseAddr+11;

	I2C_R(i2c_smbus_read_i2c_block_data(client, product_id_addr, sizeof(product_id)-1, product_id) < sizeof(product_id)-1)
	product_id[6] = '\0';

	DEBUG_MSG(E, "[TOUCH FW] Touch controller Product ID = %s\n", product_id)
	
	DO_SAFE(strncmp(product_id, &SynapticsFirmware[0x10], 6) != 0, "", return 0)
	DO_SAFE((TouchFWVersion = Synaptics_GetFWVersion(client)) == -1, "", return 0)

	if((TouchFWVersion >= 0x64 && SynapticsFirmware[0x1F] >= 0x64) || (TouchFWVersion < 0x64 && SynapticsFirmware[0x1F] < 0x64))
	{
		DO_SAFE(!(TouchFWVersion < SynapticsFirmware[0x1F]), "FW Upgrade is not needed", return 0)
	}

	////////////////////////	Configuration	///////////////////////////
	I2C_R((FlashQueryBaseAddr =  i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_QUERY_BASE_REG)) < 0)

	BootloaderIDAddr = FlashQueryBaseAddr;
	BlockSizeAddr = FlashQueryBaseAddr + 3;
	FirmwareBlockCountAddr = FlashQueryBaseAddr + 5;
	ConfigBlockCountAddr = FlashQueryBaseAddr + 7;
	
	I2C_R((FlashDataBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_DATA_BASE_REG)) < 0)

	BlockNumAddr = FlashDataBaseAddr;
	BlockDataStartAddr = FlashDataBaseAddr + 2;

	m_fileSize = sizeof(SynapticsFirmware) -1;

	checkSumCode         = ExtractLongFromHeader(&(SynapticsFirmware[0]));
	m_bootloadImgID      = (unsigned int)SynapticsFirmware[4] + (unsigned int)SynapticsFirmware[5]*0x100;
	m_firmwareImgVersion = SynapticsFirmware[7]; 
	m_firmwareImgSize    = ExtractLongFromHeader(&(SynapticsFirmware[8]));
	m_configImgSize      = ExtractLongFromHeader(&(SynapticsFirmware[12]));    

	CalculateChecksum((u16*)&(SynapticsFirmware[4]), (u16)(m_fileSize-4)>>1, &m_FirmwareImgFile_checkSum);

	// Get Current Firmware Information
	I2C_R(i2c_smbus_read_i2c_block_data(client, BlockSizeAddr, sizeof(temp_array), (u8*)&temp_array[0]) < sizeof(temp_array)) 
	ts_block_size = temp_array[0] + (temp_array[1] << 8);

	I2C_R(i2c_smbus_read_i2c_block_data(client,FirmwareBlockCountAddr, sizeof(temp_array), (u8*)&temp_array[0]) < sizeof(temp_array))
	ts_fw_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_fw_img_size = ts_block_size * ts_fw_block_count;

	I2C_R(i2c_smbus_read_i2c_block_data(client, ConfigBlockCountAddr, sizeof(temp_array), (u8*)&temp_array[0]) < sizeof(temp_array)) 
	ts_config_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_config_img_size = ts_block_size * ts_config_block_count;

	I2C_R(i2c_smbus_read_i2c_block_data(client, BootloaderIDAddr, sizeof(bootloader_id), (u8*)&bootloader_id[0]) < sizeof(bootloader_id))

	// Compare
	DO_SAFE(m_fileSize != (0x100+m_firmwareImgSize+m_configImgSize), "", return 0)
	DO_SAFE(m_firmwareImgSize != ts_fw_img_size, "", return 0)
	DO_SAFE(m_configImgSize != ts_config_img_size, "", return 0)
	DO_SAFE(m_firmwareImgVersion == 0 && ((unsigned int)bootloader_id[0] + (unsigned int)bootloader_id[1]*0x100) != m_bootloadImgID, "", return 0)

	////////////////////////	Flash Command - Enable	///////////////////////////
	I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), bootloader_id) < 0)
	WAIT_UNTIL_FLASH_CMD_READY((flashValue & 0x0f) != 0x00)

	I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ENABLE) < 0)
	WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)

	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Program Enable Setup Complete\n")

	////////////////////////	Flash Command  - Eraseall	///////////////////////////
	I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), bootloader_id) < 0)
	I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ERASEALL) < 0)
	
	WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
	
	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Erase Complete\n")

	////////////////////////	F/W Data Write	///////////////////////////
	for(i = 0; i < ts_fw_block_count; ++i)
	{
		temp_array[0] = i & 0xff;
		temp_array[1] = (i & 0xff00) >> 8;

		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockNumAddr, sizeof(temp_array), temp_array) < 0)
		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size, (u8*)&SynapticsFirmware[0x100+i*ts_block_size]) < 0)
#ifdef TOUCH_FW_COMPARE
		if(fw_compare(client, BlockDataStartAddr, 0x100+i*ts_block_size, ts_block_size)){
			DEBUG_MSG(3, "[TOUCH FW] FAIL: Firmware Update[%x]\n", i)
			return -1;
		}
#endif	
		I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_FW_WRITE) < 0)
		WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
	}
	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Firmware Write Complete\n")

	////////////////////////	F/W Config Write	///////////////////////////
	for(i = 0; i < ts_config_block_count; i++)
	{
		SpecialCopyEndianAgnostic(&temp_array[0], i);

		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockNumAddr, sizeof(temp_array), temp_array) < 0)
		I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size, (u8*)&SynapticsFirmware[0x100+m_firmwareImgSize+i*ts_block_size]) < 0)
#ifdef TOUCH_FW_COMPARE
		if(fw_compare(client, BlockDataStartAddr, 0x100+m_firmwareImgSize+i*ts_block_size, ts_block_size)){
			DEBUG_MSG(3, "[TOUCH FW] FAIL: Firmware Update[%x]\n", i)
			return -1;
		}
#endif	
		I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_CONFIG_WRITE) < 0)
		WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
	}
	DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Config Write Complete\n")

	////////////////////////	Reset Touch IC	///////////////////////////
	I2C_W(RMICommandBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_CMD_BASE_REG) < 0)
	
	if(RMICommandBaseAddr){
		I2C_W(i2c_smbus_write_byte_data(client, RMICommandBaseAddr, 0x01) < 0)				
		mdelay(200);

		WAIT_UNTIL_DEVICE_READY((flashValue & 0x0f) != 0x00, hTouch->gpio)

		I2C_R((temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG)) < 0)
	
		// Read F01 Status flash prog, ensure the 6th bit is '0'
		while((temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_DATA_BASE_REG)) != 0);
	}
	else{
		// H/W reset
		gpio_set_value(hTouch->reset, 0);
		mdelay(100);
		gpio_set_value(hTouch->reset, 1);

		wait_touch_power_ready(hTouch->client, 10);
	}
	DEBUG_MSG(E, "[TOUCH] Synaptics_UpgradeFirmware :: Complete!!\n")
	return 0;	
}
#endif

static ssize_t show_fw_ver(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fw_version);
}
static DEVICE_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_fw_ver, NULL);

bool init_touch_device_check(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

#if defined(TOUCH_FW_UPGRADE)
//	DO_C(synaptics_ts_fw_upgrade(hTouch) != 0, err_init_device_check);
	fw_version = Synaptics_GetFWVersion(hTouch->client);
	DO_C(device_create_file(&hTouch->client->dev, &dev_attr_fw_ver), err_init_device_check);
#endif	

	return true;

err_init_device_check:
	return false;
}
bool init_touch_device_setting(void* h_dev)
{
	struct synaptics_ts_data* hTouch = (struct synaptics_ts_data*)h_dev;

	DO_C(i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_INT_STATUS_REG, sizeof(u8)*4, (u8*)(&ts_reg_data.interrupt_status_reg)) != sizeof(u8)*4, err_Specific_Device_Setting);

	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DEVICE_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP), err_Specific_Device_Setting);
	
	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_INTERRUPT_ENABLE_REG, SYNAPTICS_INT_ABS0), err_Specific_Device_Setting);	

	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DELTA_X_THRES_REG, SYNAPTICS_DELTA_THRESHOLD), err_Specific_Device_Setting);	// Delta X
	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_DELTA_Y_THRES_REG, SYNAPTICS_DELTA_THRESHOLD), err_Specific_Device_Setting);	// Delta Y

	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_2D_GESTURE_ENABLE1, 0x00), err_Specific_Device_Setting);		// 2d gesture enable1 = not use
	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_2D_GESTURE_ENABLE2, 0x00), err_Specific_Device_Setting);		// 2d gesture enable2 = not use

	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_REPORT_MODE_REG, 0x01), err_Specific_Device_Setting);		// reduced reporting mode

	DO_C(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_MELT_CONTROL, SYNAPTICS_NO_MELT), err_Specific_Device_Setting);

	return true;
err_Specific_Device_Setting:
	return false;
}
