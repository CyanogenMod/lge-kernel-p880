/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/jiffies.h>

#include <linux/workqueue.h>
#include <linux/delay.h> 
#include <linux/wakelock.h> 
#include <linux/slab.h>

/*===========================================================================
                DEFINITIONS AND DECLARATIONS FOR MODULE

This section contains definitions for constants, macros, types, variables
and other items needed by this module.
===========================================================================*/

static struct workqueue_struct *synaptics_wq;
static struct i2c_client *hub_ts_client = NULL;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	bool has_relative_report;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];

	uint32_t flags;
	int reported_finger_count;
	int8_t sensitivity_adjust;
	int (*power)(int on);
	unsigned int count;
	int x_lastpt;
	int y_lastpt;
	struct early_suspend early_suspend;
	struct delayed_work init_delayed_work;
	unsigned char product_value;
};

enum key_leds {
	MENU,
	HOME,
	BACK,
	SEARCH,
};

static int init_stabled = -1;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

static uint32_t  Synaptics_Check_Touch_Interrupt_Status();


#define TOUCH_INT_N_GPIO						131//TEGRA_GPIO_PQ3


/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                                 Macros                                  */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/


#define TS_SNTS_GET_X_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)(low_reg&0x0F))
#define TS_SNTS_GET_Y_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)((low_reg&0xF0)/0x10))

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                       CONSTANTS DATA DEFINITIONS                        */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

#define TOUCH_EVENT_NULL						0
#define TOUCH_EVENT_BUTTON						1
#define TOUCH_EVENT_ABS							2



#define SYNAPTICS_FINGER_MAX					2  	


#define SYNAPTICS_TM1576_PRODUCT_ID				"TM1576"
#define SYNAPTICS_TM1576_RESOLUTION_X			1036
#define SYNAPTICS_TM1576_RESOLUTION_Y			1976
#define SYNAPTICS_TM1576_LCD_ACTIVE_AREA		1728
#define SYNAPTICS_TM1576_BUTTON_ACTIVE_AREA		1828

#define SYNAPTICS_TM1702_PRODUCT_ID				"TM1702"
#define SYNAPTICS_TM1702_RESOLUTION_X			1036
#define SYNAPTICS_TM1702_RESOLUTION_Y			1896
#define SYNAPTICS_TM1702_LCD_ACTIVE_AREA		1728
#define SYNAPTICS_TM1702_BUTTON_ACTIVE_AREA		1805

#define SYNAPTICS_TM1738_PRODUCT_ID				"TM1738"
#define SYNAPTICS_TM1738_RESOLUTION_X			1036
#define SYNAPTICS_TM1738_RESOLUTION_Y			1896
#define SYNAPTICS_TM1738_LCD_ACTIVE_AREA			1728
#define SYNAPTICS_TM1738_BUTTON_ACTIVE_AREA		1805



/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                    REGISTER ADDR & SETTING VALUE                        */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

#define SYNAPTICS_DATA_BASE_REG					0x13

#define SYNAPTICS_CONTROL_REG						0x4C	
#define SYNAPTICS_RIM_CONTROL_INTERRUPT_ENABLE	0x4D	

#define SYNAPTICS_DELTA_X_THRES_REG				0x50 	
#define SYNAPTICS_DELTA_Y_THRES_REG				0x51 	

#define SYNAPTICS_RMI_QUERY_BASE_REG			0xE3

#define SYNAPTICS_INT_ABS0						1<<2

#define SYNAPTICS_CONTROL_SLEEP					1<<0
#define SYNAPTICS_CONTROL_NOSLEEP				1<<2



/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                         DATA DEFINITIONS                                */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

typedef struct {
	unsigned char m_QueryBase;
	unsigned char m_CommandBase;
	unsigned char m_ControlBase;
	unsigned char m_DataBase;
	unsigned char m_IntSourceCount;
	unsigned char m_FunctionExists;
} T_RMI4FuncDescriptor;




typedef struct
{
	unsigned char device_status_reg;					
	unsigned char interrupt_status_reg;					
	unsigned char finger_state_reg[3];					

	unsigned char fingers_data[SYNAPTICS_FINGER_MAX][5];	
} ts_sensor_data;

typedef struct {
	unsigned char touch_status[SYNAPTICS_FINGER_MAX];
	unsigned int X_position[SYNAPTICS_FINGER_MAX];
	unsigned int Y_position[SYNAPTICS_FINGER_MAX];
	unsigned char width[SYNAPTICS_FINGER_MAX];
	unsigned char pressure[SYNAPTICS_FINGER_MAX];
} ts_finger_data;


static ts_sensor_data ts_reg_data;
static ts_finger_data prev_ts_data;
static ts_finger_data curr_ts_data;


static uint8_t curr_event_type = TOUCH_EVENT_NULL;
static uint8_t prev_event_type = TOUCH_EVENT_NULL;

static uint16_t pressed_button_type = KEY_RESERVED;

static uint16_t SYNAPTICS_PANEL_MAX_X;
static uint16_t SYNAPTICS_PANEL_MAX_Y;
static uint16_t SYNAPTICS_PANEL_LCD_MAX_Y;
static uint16_t SYNAPTICS_PANEL_BUTTON_MIN_Y;

unsigned char  touch_fw_version = 0;

struct wake_lock ts_wake_lock;

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                           Local Functions                               */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/




s32 synaptics_ts_i2c_read_block_data(struct i2c_client *client, u8 command,
				  u8 length, u8 *values)
{
	s32 status, retry_count = 5;
	status = i2c_smbus_read_i2c_block_data(client, command, length, values);

	if (status < 0)
	{
		struct synaptics_ts_data *ts = i2c_get_clientdata(client);
		printk("%s : I2C read block fail ,  RECOVER !!\n",__func__);


		schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(300));		
	}

	return status;
}

s32 synaptics_ts_i2c_read_byte_data(struct i2c_client *client, u8 command)
{
	s32 status, retry_count = 5;
	status = i2c_smbus_read_byte_data(client, command);

	if (status < 0)
	{
		struct synaptics_ts_data *ts = i2c_get_clientdata(client);
		printk("%s : I2C read byte fail ,  RECOVER !!\n",__func__);


		schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(300));		
	}
	
	return status;
}

s32 synaptics_ts_i2c_write_block_data(struct i2c_client *client, u8 command,
			       u8 length, const u8 *values)
{
	s32 status;
	
	status = i2c_smbus_write_block_data(client , command, length, values );
	
	if (status < 0)
	{
		struct synaptics_ts_data *ts = i2c_get_clientdata(client);
		printk("%s : I2C write block fail ,  RECOVER !!\n",__func__);

		schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(300));		
	}
	
	return status;
}
s32 synaptics_ts_i2c_write_byte_data(struct i2c_client *client, u8 command, u8 value)
{
	s32 status;
	
	status = i2c_smbus_write_byte_data(client, command, value); /* wake up */

	if (status < 0)
	{
		struct synaptics_ts_data *ts = i2c_get_clientdata(client);
		printk("%s : I2C write byte fail ,  RECOVER !!\n",__func__);


		schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(300));		
	}
	
	return status;
}

static void synaptics_ts_init_delayed_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	int ret;

//	printk("synaptics_ts_init_delayed_work() called\n");
	
	if (system_rev < 4)
	{
		uint32_t  pinValue = 0;
		pinValue = Synaptics_Check_Touch_Interrupt_Status();
		if(pinValue)
		{
			uint8_t dummy_read = 0;	
			ts_sensor_data tmp_ts_reg_data;

			synaptics_ts_i2c_read_block_data(hub_ts_client, SYNAPTICS_DATA_BASE_REG, sizeof(tmp_ts_reg_data), (u8 *)&tmp_ts_reg_data);
		}
	}
	
	synaptics_ts_i2c_write_byte_data(hub_ts_client, SYNAPTICS_RIM_CONTROL_INTERRUPT_ENABLE, 0x00); //interrupt disable
	
	disable_irq(gpio_to_irq(hub_ts_client->irq));

	synaptics_ts_i2c_read_block_data(hub_ts_client, SYNAPTICS_DATA_BASE_REG, sizeof(ts_reg_data), (u8 *)&ts_reg_data);
	
	if(system_rev < 4)
	{
		synaptics_ts_i2c_write_byte_data(hub_ts_client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP); /* wake up */
	}
	else
	{
		synaptics_ts_i2c_write_byte_data(hub_ts_client, SYNAPTICS_CONTROL_REG, (SYNAPTICS_CONTROL_NOSLEEP));
	}


	ret = synaptics_ts_i2c_write_byte_data(hub_ts_client, SYNAPTICS_DELTA_X_THRES_REG, /*0x03*/0x01);
	ret = synaptics_ts_i2c_write_byte_data(hub_ts_client, SYNAPTICS_DELTA_Y_THRES_REG, /*0x03*/0x01);

	init_stabled = 1;

	enable_irq(gpio_to_irq(hub_ts_client->irq));
	synaptics_ts_i2c_write_byte_data(hub_ts_client, SYNAPTICS_RIM_CONTROL_INTERRUPT_ENABLE, 0x07); //interrupt enable

}


static /*uint8_t*/uint32_t  Synaptics_Check_Touch_Interrupt_Status()
{
		uint32_t pinValue = 0;
		pinValue = gpio_get_value(TOUCH_INT_N_GPIO);

		return !pinValue;
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	int i;
	int finger_count = 0;

	if(init_stabled != 1)
	{		
//		printk("[TOUCH] synaptics_ts_work_func()  is not inited: Failed \n" );
		return;
	}

//	printk("[TOUCH] synaptics_ts_work_func() : START \n" );

do{	
	synaptics_ts_i2c_read_block_data(ts->client, SYNAPTICS_DATA_BASE_REG, sizeof(ts_reg_data), (u8 *)&ts_reg_data);
	
	if(ts_reg_data.interrupt_status_reg & SYNAPTICS_INT_ABS0)
	{
		for(i = 0; i < SYNAPTICS_FINGER_MAX; i++)
		{
			int check = 1 << ((i%4)*2);

			prev_ts_data.touch_status[i] = curr_ts_data.touch_status[i];
			prev_ts_data.X_position[i] = curr_ts_data.X_position[i];
			prev_ts_data.Y_position[i] = curr_ts_data.Y_position[i];
			prev_ts_data.width[i] = curr_ts_data.width[i];
			prev_ts_data.pressure[i] = curr_ts_data.pressure[i];
			
			if((ts_reg_data.finger_state_reg[i/4] & check) == check)
			{
				curr_ts_data.X_position[i] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.fingers_data[i][0], ts_reg_data.fingers_data[i][2]);
				curr_ts_data.Y_position[i] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.fingers_data[i][1], ts_reg_data.fingers_data[i][2]);
				{

					if ((((ts_reg_data.fingers_data[i][3] & 0xf0) >> 4) - (ts_reg_data.fingers_data[i][3] & 0x0f)) > 0)
						curr_ts_data.width[i] = (ts_reg_data.fingers_data[i][3] & 0xf0) >> 4;
					else
						curr_ts_data.width[i] = ts_reg_data.fingers_data[i][3] & 0x0f;

					curr_ts_data.pressure[i] = ts_reg_data.fingers_data[i][4];

					curr_ts_data.touch_status[i] = 1;

					finger_count++;
				}
			}
			else
			{
				curr_ts_data.touch_status[i] = 0;
			}
		}
		
		for(i = 0; i < SYNAPTICS_FINGER_MAX; i++)
		{
			if(curr_ts_data.touch_status[i])
			{
				if(finger_count == 1 && i == 0)
				{

					if((curr_ts_data.Y_position[i] < SYNAPTICS_PANEL_LCD_MAX_Y && prev_event_type == TOUCH_EVENT_NULL) || prev_event_type == TOUCH_EVENT_ABS)
						curr_event_type = TOUCH_EVENT_ABS;
					else if((curr_ts_data.Y_position[i] >= SYNAPTICS_PANEL_LCD_MAX_Y && prev_event_type == TOUCH_EVENT_NULL) || prev_event_type == TOUCH_EVENT_BUTTON)
						curr_event_type = TOUCH_EVENT_BUTTON;

					if(curr_event_type == TOUCH_EVENT_ABS)
					{
						if(curr_ts_data.Y_position[i] < SYNAPTICS_PANEL_LCD_MAX_Y)
						{
							input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[i]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[i]);
							input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[i]);
							input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, curr_ts_data.width[i]);
							
							input_mt_sync(ts->input_dev);
							pr_debug("[TOUCH-1] (X, Y) = (%d, %d), z = %d, w = %d\n", curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.pressure[i], curr_ts_data.width[i]);
						}
					}
					else if(curr_event_type == TOUCH_EVENT_BUTTON)
					{
						if(curr_ts_data.Y_position[i] > SYNAPTICS_PANEL_BUTTON_MIN_Y)
						{
							if(curr_ts_data.X_position[i] > 35 && curr_ts_data.X_position[i] < 245)
							{
								if(!prev_ts_data.touch_status[i])
								{
									input_report_key(ts->input_dev, KEY_MENU, 1); 
									pr_debug("[TOUCH-2] Key Event KEY = %d, PRESS = %d\n", KEY_MENU, 1);
									pressed_button_type = KEY_MENU;
								}								
							}
							else if(curr_ts_data.X_position[i] > 287 && curr_ts_data.X_position[i] < 497)
							{
								if(!prev_ts_data.touch_status[i])
								{
									input_report_key(ts->input_dev, KEY_HOME, 1);
									pr_debug("[TOUCH-5] Key Event KEY = %d, PRESS = %d\n", KEY_HOME, 1);
									pressed_button_type = KEY_HOME;
								}
								
							}
							else if(curr_ts_data.X_position[i] > 539 && curr_ts_data.X_position[i] < 749) 
							{
								if(!prev_ts_data.touch_status[i])
								{
									input_report_key(ts->input_dev, KEY_BACK, 1); 
									pr_debug("[TOUCH-9] Key Event KEY = %d, PRESS = %d\n", KEY_BACK, 1);
									pressed_button_type = KEY_BACK;
								}
								
							}
							else if(curr_ts_data.X_position[i] > 791 && curr_ts_data.X_position[i] < 1001) 
							{
								if(!prev_ts_data.touch_status[i])
								{
									input_report_key(ts->input_dev, KEY_SEARCH, 1); 
									pr_debug("[TOUCH-13] Key Event KEY = %d, PRESS = %d\n", KEY_SEARCH, 1);
									pressed_button_type = KEY_SEARCH;
								}								
							}
							

						}
						else
						{

								input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[i]);
								input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[i]);
								input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[i]);
								input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, curr_ts_data.width[i]);
								
								input_mt_sync(ts->input_dev);
								pr_debug("[TOUCH-23] (X, Y) = (%d, %d), z = %d, w = %d\n", curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.pressure[i], curr_ts_data.width[i]);

								curr_event_type = TOUCH_EVENT_ABS;

						}
					}
					else
					{
						curr_event_type = TOUCH_EVENT_NULL;
					}
				}
				else 
				{
					curr_event_type = TOUCH_EVENT_ABS;

					if(curr_ts_data.Y_position[i] < SYNAPTICS_PANEL_LCD_MAX_Y)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[i]);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[i]);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[i]);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, curr_ts_data.width[i]);

						input_mt_sync(ts->input_dev);
						pr_debug("[TOUCH-27] (X, Y) = (%d, %d), z = %d, w = %d\n", curr_ts_data.X_position[i], curr_ts_data.Y_position[i], curr_ts_data.pressure[i], curr_ts_data.width[i]);
					}
				}
			}
			else
			{
				if(i == 0)
				{					
					input_report_key(ts->input_dev, pressed_button_type, 0);
					{
							pr_debug("Touch Key LED is working at over Rev.C\n");
					}
					pr_debug("[TOUCH-28] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
				}
			}
		}
		
		if(finger_count == 0)
		{
			prev_event_type = TOUCH_EVENT_NULL;
		}
		else
		{
			prev_event_type = curr_event_type;
		}

	}

	input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);
}while(Synaptics_Check_Touch_Interrupt_Status());
SYNAPTICS_TS_IDLE:
	if (ts->use_irq) {		
		enable_irq(gpio_to_irq(ts->client->irq));
	}
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);

	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL); /* 12.5 msec */

	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

//	printk("synaptics_ts_irq_handler\n");

	disable_irq_nosync(gpio_to_irq(ts->client->irq));
	
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	uint16_t max_x;
	uint16_t max_y;
	uint8_t max_pressure;
	uint8_t max_width;

	char product_id[6];
	uint8_t product_id_addr;
	pr_warning("%s() -- start\n\n\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);	
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	hub_ts_client = client;
	i2c_set_clientdata(client, ts);

	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_DELAYED_WORK(&ts->init_delayed_work, synaptics_ts_init_delayed_work);

  	memset(&ts_reg_data, 0x0, sizeof(ts_sensor_data));
	memset(&prev_ts_data, 0x0, sizeof(ts_finger_data));
  	memset(&curr_ts_data, 0x0, sizeof(ts_finger_data));
	product_id_addr = (synaptics_ts_i2c_read_byte_data(hub_ts_client, SYNAPTICS_RMI_QUERY_BASE_REG)) + 11;
	synaptics_ts_i2c_read_block_data(hub_ts_client, product_id_addr, sizeof(product_id), (u8 *)&product_id[0]);

	if(strncmp(product_id, SYNAPTICS_TM1576_PRODUCT_ID, 6) == 0)
	{
		pr_err("synaptics_ts_probe: product ID : TM1576\n");
		SYNAPTICS_PANEL_MAX_X = SYNAPTICS_TM1576_RESOLUTION_X;
		SYNAPTICS_PANEL_MAX_Y = SYNAPTICS_TM1576_RESOLUTION_Y;
		SYNAPTICS_PANEL_LCD_MAX_Y = SYNAPTICS_TM1576_LCD_ACTIVE_AREA;
		SYNAPTICS_PANEL_BUTTON_MIN_Y = SYNAPTICS_TM1576_BUTTON_ACTIVE_AREA;
		ts->product_value=3; 
	}
	else if(strncmp(product_id, SYNAPTICS_TM1702_PRODUCT_ID, 6) == 0)
	{
		pr_err("synaptics_ts_probe: product ID : TM1702\n");
		SYNAPTICS_PANEL_MAX_X = SYNAPTICS_TM1702_RESOLUTION_X;
		SYNAPTICS_PANEL_MAX_Y = SYNAPTICS_TM1702_RESOLUTION_Y;
		SYNAPTICS_PANEL_LCD_MAX_Y = SYNAPTICS_TM1702_LCD_ACTIVE_AREA;
		SYNAPTICS_PANEL_BUTTON_MIN_Y = SYNAPTICS_TM1702_BUTTON_ACTIVE_AREA;
		ts->product_value=1; 
		
	}
	else if(strncmp(product_id, SYNAPTICS_TM1738_PRODUCT_ID, 6) == 0)
	{
		pr_err("synaptics_ts_probe: product ID : TM1738\n");
		SYNAPTICS_PANEL_MAX_X = SYNAPTICS_TM1738_RESOLUTION_X;
		SYNAPTICS_PANEL_MAX_Y = SYNAPTICS_TM1738_RESOLUTION_Y;
		SYNAPTICS_PANEL_LCD_MAX_Y = SYNAPTICS_TM1738_LCD_ACTIVE_AREA;
		SYNAPTICS_PANEL_BUTTON_MIN_Y = SYNAPTICS_TM1738_BUTTON_ACTIVE_AREA;
		ts->product_value=2; 

	}
	else
	{
		pr_err("synaptics_ts_probe: product ID : error\n");
		SYNAPTICS_PANEL_MAX_X = SYNAPTICS_TM1702_RESOLUTION_X;
		SYNAPTICS_PANEL_MAX_Y = SYNAPTICS_TM1702_RESOLUTION_Y;
		SYNAPTICS_PANEL_LCD_MAX_Y = SYNAPTICS_TM1702_LCD_ACTIVE_AREA;
		SYNAPTICS_PANEL_BUTTON_MIN_Y = SYNAPTICS_TM1702_BUTTON_ACTIVE_AREA;
		ts->product_value=0; 
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		pr_err("synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "hub_synaptics_touch";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	// button
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);

	max_x = SYNAPTICS_PANEL_MAX_X;
	max_y = SYNAPTICS_PANEL_LCD_MAX_Y;
	max_pressure = 0xFF;
	max_width = 0x0F;
 
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, max_pressure, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, max_width, 0, 0);

	pr_info("synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		pr_err("synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	pr_debug("########## irq [%d], irqflags[0x%x]\n", client->irq, IRQF_TRIGGER_FALLING);
	
	synaptics_ts_i2c_write_byte_data(hub_ts_client, SYNAPTICS_RIM_CONTROL_INTERRUPT_ENABLE, 0x00); 

	ret = gpio_request(client->irq, "pq3");
	if (ret < 0)
	{
		return ret;
	}

	ret=gpio_direction_input(client->irq);
	if (ret < 0)
	{
		gpio_free(client->irq);
		return ret;
	}
	else
		tegra_gpio_enable(client->irq);
	
	if (client->irq) {
		ret = request_irq(gpio_to_irq(client->irq), synaptics_ts_irq_handler, 
			IRQF_TRIGGER_FALLING, client->name, ts);

		if (ret == 0) {
			ts->use_irq = 1;
			pr_warning("request_irq\n");
			}
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(500));	
/*
	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		printk(KERN_INFO "[chahee.kim] GPIO 40 MUIC_INT_N wake up source setting failed!\n");
		disable_irq_wake(client->irq);
		return -ENOSYS;
	}
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	pr_warning("synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);


	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(gpio_to_irq(client->irq), ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	init_stabled = 0;
	
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(gpio_to_irq(client->irq));
	else
		hrtimer_cancel(&ts->timer);
	
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) 
		enable_irq(gpio_to_irq(client->irq));

	ret = synaptics_ts_i2c_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_SLEEP); /* sleep */
	if (ret < 0)
		pr_err("synaptics_ts_suspend: synaptics_ts_i2c_write_byte_data failed\n");


	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
//	init_stabled = 1;
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	uint8_t unconfigured_reg = 0; 

	if (ts->use_irq)
		enable_irq(gpio_to_irq(client->irq));

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(500));

  	memset(&ts_reg_data, 0x0, sizeof(ts_sensor_data));
	memset(&prev_ts_data, 0x0, sizeof(ts_finger_data));
  	memset(&curr_ts_data, 0x0, sizeof(ts_finger_data));	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "synaptics_T1320", 0 },
	{ },
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= "synaptics_T1320",
		.owner = THIS_MODULE,
	},
};

static int __devinit synaptics_ts_init(void)
{

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");

	if (!synaptics_wq)
		return -ENOMEM;
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
    
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

