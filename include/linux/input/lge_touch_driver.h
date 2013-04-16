/* drivers/input/keyboard/lge_touch_driver.h
 *
 * Copyright (C) 2011 LGE. 
 * 
 * Writer: yehan.ahn@lge.com
 *
 * This device driver can be used for any touch device (K36, use native-I2C)
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

#ifndef LGE_TOUCH_DRIVER_H
#define LGE_TOUCH_DRIVER_H

#include <linux/platform_device.h>

#ifndef DEFINE_TOUCH_NAME
#define DEFINE_TOUCH_NAME
#define LGE_TOUCH_NAME "lge_touch"
#endif

#define SYNAPTICS_TOUCH_DEBUG_LEVEL 3

#ifdef SYNAPTICS_TOUCH_DEBUG_LEVEL
#define DEBUG_MSG(_level, args...)  \
		if(_level >= SYNAPTICS_TOUCH_DEBUG_LEVEL)	\
			printk(KERN_INFO args);
#else
#define DEBUG_MSG(_level, args...)
#endif

enum{M=1,	// touch_movement_event_log || check_some_log
	 B,		// touch_button_event_log || sync_event_log
	 E,		// essential
	 ERR,	//error
	 NOTHING
};

#define DO_SAFE(_do, _log, _err_handling)	\
if(unlikely(_do)){	\
	DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, _log);	\
	_err_handling;	\
}

// do safely when allocate...
#define DO_A(_statement, _jump_place)	DO_SAFE((_statement) == NULL, "", goto _jump_place)

// do safely when call the function...
#define DO_F(_statement, _jump_place)	DO_SAFE((_statement) == 0, "", goto _jump_place)

// do safely when compare...
#define DO_C(_statement, _jump_place)	DO_SAFE((_statement), "", goto _jump_place)

enum {KEY_NULL=0, KEY_PANEL, KEY_BOUNDARY};
enum {BEFORE_WHILE=1, AFTER_GET_DATA, BEFORE_SYNC, AFTER_SYNC};
enum {FINGER_RELEASE=0, SINGLE_FINGER, MULTI_FINGER};
enum {DO_NOT_ANYTHING=0, ABS_PRESS, ABS_RELEASE, BUTTON_PRESS, BUTTON_RELEASE, BUTTON_CANCEL, TOUCH_BUTTON_LOCK, TOUCH_ABS_LOCK};
enum {TOUCH_RELEASE=0, TOUCH_PRESS, TOUCH_CANCEL=0xff};


struct touch_data
{
	u16 X_position;
	u16 Y_position;
	u16 width;
	u16 pressure;
};

struct touch_finger_data
{
	u8		total_num;
	struct touch_data	curr_data[10];
	struct touch_data	prev_data[10];
	u8		curr_button;
	u8		prev_button;
	u8		state;
};

struct touch_finger_data_func
{
	bool	(*get_finger_data)	(void* h_dev, struct touch_finger_data* data);
	u8		(*check_button)		(void* h_dev, struct touch_finger_data* data);
	bool	(*additional_job)	(void* h_dev, struct touch_finger_data* data, u32 whereis);
};

struct touch_inputdev_func
{
	bool	(*open)				(void* h_dev);
	void	(*close)			(void* h_dev);
	bool	(*send_ABS)			(void* h_dev, struct touch_data data, u8 isPress);
	bool	(*send_ABS_Multi)	(void* h_dev, struct touch_data* data, u8 total_num);
	bool	(*send_Button)		(void* h_dev, u32 type, u32 isPress);
	bool 	(*send_Sync)		(void* h_dev, u8 state);
};

struct touch_task_func
{
	bool	(*open)		(void* h_dev);
	void	(*close)	(void* h_dev);
	bool	(*start)	(void* h_dev);
	bool	(*stop)		(void* h_dev);
};

struct touch_interrupt_func
{
	bool	(*open)		(void* h_dev);
	void	(*close)	(void* h_dev);
	bool	(*enable)	(void* h_dev);
	bool	(*disable)	(void* h_dev);
	bool	(*start)	(void* h_dev);
	bool	(*wait) 	(void* h_dev);
}; 

struct touch_power_func
{
	bool	(*on)		(void* h_dev);
	void	(*off)		(void* h_dev);
	bool	(*sleep)	(void* h_dev);
	bool	(*wake)		(void* h_dev);
}; 

bool device_open(void** h_touch, struct i2c_client *client, u32 touch);
void device_close(void* h_dev);

void init_touch_inputdev(void* h_dev, struct touch_inputdev_func* h_inputDev);
void init_touch_task(void* h_dev, struct touch_task_func* h_task);
void init_touch_interrupt(void* h_dev, struct touch_interrupt_func* h_interrupt);
void init_touch_power(void* h_dev, struct touch_power_func* h_powerCtrl);
void init_touch_finger_data(void* h_dev, struct touch_finger_data_func* h_fingerData);
bool init_touch_device_check(void* h_dev);
bool init_touch_device_setting(void* h_dev);

irqreturn_t interrupt_handler(int irq, void *dev_id);
int  task_handler(void* arg);

#endif
