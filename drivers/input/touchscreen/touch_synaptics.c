/* Touch_synaptics.c
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <mach/gpio.h>

#include <linux/input/lge_touch_core.h>
#include <linux/input/touch_synaptics.h>

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100)		
#include "SynaImage325.h"
#else
#include "SynaImage.h"
#endif
#include <linux/regulator/machine.h>

/* RMI4 spec from (RMI4 spec)511-000136-01_revD
 * Function	Purpose									See page
 * $01		RMI Device Control						29
 * $08		BIST(Built-in Self Test)				38
 * $09		BIST(Built-in Self Test)				42
 * $11		2-D TouchPad sensors					46
 * $19		0-D capacitive button sensors			69
 * $30		GPIO/LEDs (includes mechanical buttons)	76
 * $32		Timer									89
 * $34		Flash Memory Management					93
 */

/* RMI4 spec from (RMI4 spec)511-000405-01_revD
 * Function	Purpose									See page      supported IC
 * $01		RMI Device Control						29            S3203
 * $1A		Capacitive button sensors				61
 * $05		Image reporting							69
 * $07		Image reporting							75
 * $08		BIST(Built-in Self Test)				82
 * $09		BIST(Built-in Self Test)				87
 * $11		2-D sensors								93            S3203
 * $19		0-D capacitive button sensors			141           S3203
 * $30		GPIO/LEDs (includes mechanical buttons)	148
 * $31		LEDs									162
 * $34		Flash Memory Management					163           S3203
 * $36		Auxiliary analog to digital conversion  174
 * $54		Test reporting							176           S3203
 */

#define RMI_DEVICE_CONTROL				0x01
#define TOUCHPAD_SENSORS				0x11
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
#define CAPACITIVE_BUTTON_SENSORS		0x1A
#define GPIO_LEDS						0x31
#else
#define CAPACITIVE_BUTTON_SENSORS		0x19
#define GPIO_LEDS						0x30
#endif
#define TIMER							0x32
#define FLASH_MEMORY_MANAGEMENT			0x34
#define ANALOG_CONTROL					0x54

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */

#define MANUFACTURER_ID_REG				(ts->common_dsc.query_base)			/* Manufacturer ID */
#define FW_REVISION_REG					(ts->common_dsc.query_base+3)		/* FW revision */
#define PRODUCT_ID_REG					(ts->common_dsc.query_base+11)		/* Product ID */

#define DEVICE_CONTROL_REG 				(ts->common_dsc.control_base)		/* Device Control */
#define DEVICE_CONTROL_NORMAL_OP		0x00	/* sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_SLEEP 			0x01	/* sleep mode : go to sleep */
#define DEVICE_CONTROL_SPECIFIC			0x02	/* sleep mode : go to doze mode after 5 sec */
#define DEVICE_CONTROL_NOSLEEP			0x04
#define DEVICE_CONTROL_CONFIGURED		0x80

#define INTERRUPT_ENABLE_REG			(ts->common_dsc.control_base+1)		/* Interrupt Enable 0 */

#define DEVICE_STATUS_REG				(ts->common_dsc.data_base)			/* Device Status */
#define DEVICE_FAILURE_MASK				0x03
#define DEVICE_CRC_ERROR_MASK			0x04
#define DEVICE_STATUS_FLASH_PROG		0x40
#define DEVICE_STATUS_UNCONFIGURED		0x80

#define INTERRUPT_STATUS_REG			(ts->common_dsc.data_base+1)		/* Interrupt Status */
#define BUTTON_DATA_REG					(ts->button_dsc.data_base)			/* Button Data */
#define MAX_NUM_OF_BUTTON				4

#define FINGER_STATE_REG				(ts->finger_dsc.data_base)			/* Finger State */
#define FINGER_DATA_REG_START			(ts->finger_dsc.data_base+3)		/* Finger Data Register */
#define FINGER_STATE_MASK				0x03
#define REG_X_POSITION					0
#define REG_Y_POSITION					1
#define REG_YX_POSITION					2
#define REG_WY_WX						3
#define REG_Z							4

#define TWO_D_REPORTING_MODE			(ts->finger_dsc.control_base+0)		/* 2D Reporting Mode */
#define REPORT_MODE_CONTINUOUS			0x00
#define REPORT_MODE_REDUCED				0x01
#define ABS_FILTER						0x08
#define PALM_DETECT_REG 				(ts->finger_dsc.control_base+1)		/* Palm Detect */
#define DELTA_X_THRESH_REG 				(ts->finger_dsc.control_base+2)		/* Delta-X Thresh */
#define DELTA_Y_THRESH_REG 				(ts->finger_dsc.control_base+3)		/* Delta-Y Thresh */
#define SENSOR_MAX_X_POS				(ts->finger_dsc.control_base+6)		/* SensorMaxXPos */
#define SENSOR_MAX_Y_POS				(ts->finger_dsc.control_base+8)		/* SensorMaxYPos */
#define GESTURE_ENABLE_1_REG 			(ts->finger_dsc.control_base+10)	/* Gesture Enables 1 */
#define GESTURE_ENABLE_2_REG 			(ts->finger_dsc.control_base+11)	/* Gesture Enables 2 */

#define BUTTON_DATA_REG					(ts->button_dsc.data_base)			/* Button Data */

#define MELT_CONTROL_REG				0xF0
#define MELT_CONTROL_NO_MELT			0x00
#define MELT_CONTROL_MELT				0x01
#define MELT_CONTROL_NUKE_MELT			0x80

#define DEVICE_COMMAND_REG				(ts->common_dsc.command_base)
#define FINGER_COMMAND_REG				(ts->finger_dsc.command_base)
#define BUTTON_COMMAND_REG				(ts->button_dsc.command_base)

#define FLASH_CONTROL_REG				(ts->flash_dsc.data_base+18)		/* Flash Control */
#define FLASH_STATUS_MASK				0xF0
#define FLASH_CONFIG_ID_REG				(ts->flash_dsc.control_base)		/* Flash Cofig ID */

#define ANALOG_CONTROL_REG				(ts->analog_dsc.control_base)
#define FORCE_FAST_RELAXATION			0x04
#define ANALOG_COMMAND_REG				(ts->analog_dsc.command_base)
#define FORCE_UPDATE					0x04

#define TEST_REPORT_REG					(ts->test_dsc.query_base)
#define TEST_REPORT_DATA_TYPE			0x00
#define TEST_REPORT_DATA_BUFF			0x03
#define TEST_REPORT_DATA_INDEX_LOW		0x01
#define TEST_REPORT_DATA_INDEX_HIGH		0x02
#define TEST_REPORT_CMD_BASE			0xA8
#define TEST_REPORT_QUERY_BASE			0xA9

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
#define BUTTON_THRESHOLD_REG				(ts->button_dsc.control_base) 
#endif

/* PAGE_SELECT_REGs shares their value 
   if it changed in address 0x00FF, 0x01FF and 0x02FF also changed at the same time.*/
#define PAGE_SELECT_REG					0x00FF	 

/* Get user-finger-data from register.
 */
#define TS_SNTS_GET_X_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x0FF0) | (u16)(_low_reg&0x0F)))
#define TS_SNTS_GET_Y_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x0FF0) | (u16)((_low_reg >> 4) & 0x0F)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? (_width & 0xF0) >> 4 : _width & 0x0F
#define TS_SNTS_GET_WIDTH_MINOR(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? _width & 0x0F : (_width & 0xF0) >> 4
#define TS_SNTS_GET_ORIENTATION(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure


/* GET_BIT_MASK & GET_INDEX_FROM_MASK
 *
 * For easily checking the user input.
 * Usually, User use only one or two fingers.
 * However, we should always check all finger-status-register
 * because we can't know the total number of fingers.
 * These Macro will prevent it.
 */
#define GET_BIT_MASK(_finger_status_reg)	\
		(_finger_status_reg[2] & 0x04)<<7 | (_finger_status_reg[2] & 0x01)<<8 |	\
		(_finger_status_reg[1] & 0x40)<<1 | (_finger_status_reg[1] & 0x10)<<2 | \
		(_finger_status_reg[1] & 0x04)<<3 | (_finger_status_reg[1] & 0x01)<<4 |	\
		(_finger_status_reg[0] & 0x40)>>3 | (_finger_status_reg[0] & 0x10)>>2 | \
		(_finger_status_reg[0] & 0x04)>>1 | (_finger_status_reg[0] & 0x01)

#define GET_INDEX_FROM_MASK(_index, _bit_mask, _max_finger)	\
		for(; !((_bit_mask>>_index)&0x01) && _index <= _max_finger; _index++);	\
		if (_index <= _max_finger) _bit_mask &= ~(_bit_mask & (1<<(_index)));

/*
  F54 TEST_REPORT
*/

u8 test_report_type = 0x02; //default delta capacitance
u8 test_report_status = 0;


int synaptics_ts_get_data(struct i2c_client *client, struct t_data* data, struct b_data* button, u8* total_num)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);

	u16 touch_finger_bit_mask=0;
	u8 finger_index=0;
	u8 index=0;
	u8 cnt;

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (unlikely(touch_i2c_read(client, DEVICE_STATUS_REG,
			sizeof(ts->ts_data.interrupt_status_reg),
			&ts->ts_data.device_status_reg) < 0)) {
		TOUCH_ERR_MSG("DEVICE_STATUS_REG read fail\n");
		goto err_synaptics_getdata;
	}

	/* ESD damage check */
	if ((ts->ts_data.device_status_reg & DEVICE_FAILURE_MASK)== DEVICE_FAILURE_MASK) {
		TOUCH_ERR_MSG("ESD damage occured. Reset Touch IC\n");
		goto err_synaptics_device_damage;
	}

	/* Internal reset check */
	if (((ts->ts_data.device_status_reg & DEVICE_STATUS_UNCONFIGURED) >> 7) == 1) {
		TOUCH_ERR_MSG("Touch IC resetted internally. Reconfigure register setting\n");
		goto err_synaptics_device_damage;
	}

	if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG,
			sizeof(ts->ts_data.interrupt_status_reg),
			&ts->ts_data.interrupt_status_reg) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
		goto err_synaptics_getdata;
	}
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
	/* IC bug Exception handling - Interrupt status reg is 0 when interrupt occur */
	if(unlikely(ts->ts_data.interrupt_status_reg == 0)){
		TOUCH_ERR_MSG("Interrupt_status_reg is 0. Something is wrong in IC\n");
		goto err_synaptics_device_damage;
	}
#endif
	if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
		TOUCH_INFO_MSG("Interrupt_status : 0x%x\n", ts->ts_data.interrupt_status_reg);

	/* Because of ESD damage... */
	if (unlikely(ts->ts_data.interrupt_status_reg & ts->interrupt_mask.flash)){
		TOUCH_ERR_MSG("Impossible Interrupt\n");
		goto err_synaptics_device_damage;
	}

	/* Finger */
	if (likely(ts->ts_data.interrupt_status_reg & ts->interrupt_mask.abs)) {
		if (unlikely(touch_i2c_read(client, FINGER_STATE_REG,
				sizeof(ts->ts_data.finger.finger_status_reg),
				ts->ts_data.finger.finger_status_reg) < 0)) {
			TOUCH_ERR_MSG("FINGER_STATE_REG read fail\n");
			goto err_synaptics_getdata;
		}

		touch_finger_bit_mask = GET_BIT_MASK(ts->ts_data.finger.finger_status_reg);
		if (unlikely(touch_debug_mask & DEBUG_GET_DATA)) {
			TOUCH_INFO_MSG("Finger_status : 0x%x, 0x%x, 0x%x\n", ts->ts_data.finger.finger_status_reg[0],
					ts->ts_data.finger.finger_status_reg[1], ts->ts_data.finger.finger_status_reg[2]);
			TOUCH_INFO_MSG("Touch_bit_mask: 0x%x\n", touch_finger_bit_mask);
		}

		while(touch_finger_bit_mask) {
			GET_INDEX_FROM_MASK(finger_index, touch_finger_bit_mask, MAX_NUM_OF_FINGERS)
			if (unlikely(touch_i2c_read(ts->client,
					FINGER_DATA_REG_START + (NUM_OF_EACH_FINGER_DATA_REG * finger_index),
					NUM_OF_EACH_FINGER_DATA_REG,
					ts->ts_data.finger.finger_reg[index]) < 0)) {
				TOUCH_ERR_MSG("FINGER_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}

			data[index].id = finger_index;
			data[index].x_position =
				TS_SNTS_GET_X_POSITION(ts->ts_data.finger.finger_reg[index][REG_X_POSITION],
									   ts->ts_data.finger.finger_reg[index][REG_YX_POSITION]);
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T3203)
			data[index].y_position = ts->pdata->caps->y_max -
				TS_SNTS_GET_Y_POSITION(ts->ts_data.finger.finger_reg[index][REG_Y_POSITION],
									   ts->ts_data.finger.finger_reg[index][REG_YX_POSITION]);
#else			
			data[index].y_position = TS_SNTS_GET_Y_POSITION(ts->ts_data.finger.finger_reg[index][REG_Y_POSITION],
						   ts->ts_data.finger.finger_reg[index][REG_YX_POSITION]);
#endif
			data[index].width_major = TS_SNTS_GET_WIDTH_MAJOR(ts->ts_data.finger.finger_reg[index][REG_WY_WX]);
			data[index].width_minor = TS_SNTS_GET_WIDTH_MINOR(ts->ts_data.finger.finger_reg[index][REG_WY_WX]);
			data[index].width_orientation = TS_SNTS_GET_ORIENTATION(ts->ts_data.finger.finger_reg[index][REG_WY_WX]);
			data[index].pressure = TS_SNTS_GET_PRESSURE(ts->ts_data.finger.finger_reg[index][REG_Z]);

			if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
				TOUCH_INFO_MSG("[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] w_o[%2d] p[%2d]\n",
								finger_index, data[index].x_position, data[index].y_position,
								data[index].width_major, data[index].width_minor,
								data[index].width_orientation, data[index].pressure);

			index++;
		}
		*total_num = index;
		if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
			TOUCH_INFO_MSG("Total_num: %d\n", *total_num);
	}

	 /* Button */
	if (unlikely(ts->button_dsc.id != 0)) {
		if (likely(ts->ts_data.interrupt_status_reg & ts->interrupt_mask.button)) {
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x02) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}

			if (unlikely(touch_i2c_read(client, BUTTON_DATA_REG,
					sizeof(ts->ts_data.button_data_reg),
					&ts->ts_data.button_data_reg) < 0)) {
				TOUCH_ERR_MSG("BUTTON_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}
/*
			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
				TOUCH_DEBUG_MSG("Button register: 0x%x\n", ts->ts_data.button_data_reg);
*/
			if (ts->ts_data.button_data_reg) {
				/* pressed - find first one */
				for (cnt = 0; cnt < ts->pdata->caps->number_of_button; cnt++)
				{
					if ((ts->ts_data.button_data_reg >> cnt) & 0x1) {
						ts->ts_data.button.key_code = ts->pdata->caps->button_name[cnt];
						button->key_code = ts->ts_data.button.key_code;
						button->state = 1;
						break;
					}
				}
			}else {
				/* release */
				button->key_code = ts->ts_data.button.key_code;
				button->state = 0;
			}

			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}
		}
	}
#else
			if (unlikely(touch_i2c_read(client, BUTTON_DATA_REG,
					sizeof(ts->ts_data.button_data_reg),
					&ts->ts_data.button_data_reg) < 0)) {
				TOUCH_ERR_MSG("BUTTON_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}

			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
				TOUCH_DEBUG_MSG("Button register: 0x%x\n", ts->ts_data.button_data_reg);

			if (ts->ts_data.button_data_reg) {
			  TOUCH_DEBUG_MSG("Button pressed : 0x%x\n",ts->ts_data.button_data_reg)
				/* pressed - find first one */
				for (cnt = 0; cnt < ts->pdata->caps->number_of_button; cnt++)
				{
					if ((ts->ts_data.button_data_reg >> cnt) & 0x1) {
						ts->ts_data.button.key_code = ts->pdata->caps->button_name[cnt];
						button->key_code = ts->ts_data.button.key_code;
						button->state = 1;
						TOUCH_DEBUG_MSG("Button pressed, key_code: 0x%x\n", button->key_code);
						break;
					}
				}
			}else {
				/* release */
				button->key_code = ts->ts_data.button.key_code;
				button->state = 0;
			}
		}
	}
#endif 
	return 0;

err_synaptics_device_damage:
err_synaptics_getdata:
	return -EIO;
}

static int read_page_description_table(struct i2c_client* client)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);
	struct ts_function_descriptor buffer;
	unsigned short u_address;

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	memset(&buffer, 0x0, sizeof(struct ts_function_descriptor));

	ts->common_dsc.id = 0;
	ts->finger_dsc.id = 0;
	ts->button_dsc.id = 0;
	ts->flash_dsc.id  = 0;
	ts->analog_dsc.id = 0;

	for(u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(struct ts_function_descriptor)) {
		if (unlikely(touch_i2c_read(client, u_address, sizeof(buffer), (unsigned char *)&buffer) < 0)) {
			TOUCH_ERR_MSG("RMI4 Function Descriptor read fail\n");
			return -EIO;
		}

		if (buffer.id == 0)
			break;

		switch (buffer.id) {
		case RMI_DEVICE_CONTROL:
			ts->common_dsc = buffer;
			break;
		case TOUCHPAD_SENSORS:
			ts->finger_dsc = buffer;
			break;
#if !defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
		case CAPACITIVE_BUTTON_SENSORS:
			ts->button_dsc = buffer;
			break;
#endif 
		case FLASH_MEMORY_MANAGEMENT:
			ts->flash_dsc = buffer;
		}
	}

	
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x01) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	for(u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(struct ts_function_descriptor)) {
		if (unlikely(touch_i2c_read(client, u_address, sizeof(buffer), (unsigned char *)&buffer) < 0)) {
			TOUCH_ERR_MSG("RMI4 Function Descriptor read fail\n");
			return -EIO;
		}

		if (buffer.id == 0)
			break;

		switch (buffer.id) {
		case ANALOG_CONTROL:
			ts->analog_dsc = buffer;
			break;
		}
	}

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x02) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
			}
		for(u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(struct ts_function_descriptor)) {
			if (unlikely(touch_i2c_read(client, u_address, sizeof(buffer), (unsigned char *)&buffer) < 0)) {
				TOUCH_ERR_MSG("RMI4 Function Descriptor read fail\n");
				return -EIO;
			}

			if (buffer.id == 0)
				break;
			switch (buffer.id) {
			case CAPACITIVE_BUTTON_SENSORS:
			ts->button_dsc = buffer;
			break;
			}
		}
#endif	
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	/* set interrupt mask */
	ts->interrupt_mask.flash = 0x1;
	ts->interrupt_mask.status = 0x2;
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
	ts->interrupt_mask.abs = 0x4;
	ts->interrupt_mask.button = 0x20;
#else
	if (ts->button_dsc.id == 0)
		ts->interrupt_mask.abs = 0x4;
	else {
		if (ts->finger_dsc.data_base > ts->button_dsc.data_base) {
			ts->interrupt_mask.abs = 0x8;
			ts->interrupt_mask.button = 0x4;
		} else {
			ts->interrupt_mask.abs = 0x4;
			ts->interrupt_mask.button = 0x8;
		}
	}
#endif
	if(ts->common_dsc.id == 0 || ts->finger_dsc.id == 0 || ts->flash_dsc.id == 0){
		TOUCH_ERR_MSG("common_dsc/finger_dsc/flash_dsc are not initiailized\n");
		return -EPERM;
	}

	if (touch_debug_mask & DEBUG_BASE_INFO)
		TOUCH_INFO_MSG("common[%d] finger[%d] flash[%d] button[%d]\n",
				ts->common_dsc.id, ts->finger_dsc.id, ts->flash_dsc.id, ts->button_dsc.id);

	return 0;
}

static int get_ic_info(struct synaptics_ts_data* ts, struct touch_fw_info* fw_info)
{
#if defined(ARRAYED_TOUCH_FW_BIN)
	int cnt;
#endif

	u8 device_status = 0;
	u8 flash_control = 0;
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 	
	u8 analog_control = 0; 
#endif

	read_page_description_table(ts->client);

	memset(fw_info, 0, sizeof(fw_info));

	if (unlikely(touch_i2c_read(ts->client, FW_REVISION_REG,
			sizeof(fw_info->fw_rev), &fw_info->fw_rev) < 0)) {
		TOUCH_ERR_MSG("FW_REVISION_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, MANUFACTURER_ID_REG,
			sizeof(fw_info->manufacturer_id), &fw_info->manufacturer_id) < 0)) {
		TOUCH_ERR_MSG("MANUFACTURER_ID_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, PRODUCT_ID_REG,
			sizeof(fw_info->product_id) - 1, fw_info->product_id) < 0)) {
		TOUCH_ERR_MSG("PRODUCT_ID_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
			sizeof(fw_info->config_id) - 1, fw_info->config_id) < 0)) {
		TOUCH_ERR_MSG("FLASH_CONFIG_ID_REG read fail\n");
		return -EIO;
	}
	
#if defined(ARRAYED_TOUCH_FW_BIN)
	for (cnt = 0; cnt < sizeof(SynaFirmware)/sizeof(SynaFirmware[0]); cnt++) {
		strncpy(fw_info->fw_image_product_id, &SynaFirmware[cnt][16], 10);
		if (!(strncmp(fw_info->product_id , fw_info->fw_image_product_id, 10)))
			break;
	}
	fw_info->fw_start = (unsigned char *)&SynaFirmware[cnt][0];
	fw_info->fw_size = sizeof(SynaFirmware[0]);
#else
	strncpy(fw_info->fw_image_product_id, &SynaFirmware[16], 10);
	fw_info->fw_start = (unsigned char *)&SynaFirmware[0];
	fw_info->fw_size = sizeof(SynaFirmware);
#endif

	fw_info->fw_image_rev = fw_info->fw_start[31];

	// copy CONFIG_ID of fw image header file
	if(fw_info->fw_size > 0xb104)
	{
		strncpy(fw_info->fw_image_config_id, &SynaFirmware[0xb100], 4);
		TOUCH_INFO_MSG("SynaFirmware : %s\n",fw_info->fw_image_config_id);
	}

	if (unlikely(touch_i2c_read(ts->client, FLASH_CONTROL_REG, sizeof(flash_control), &flash_control) < 0)) {
		TOUCH_ERR_MSG("FLASH_CONTROL_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, DEVICE_STATUS_REG, sizeof(device_status), &device_status) < 0)) {
		TOUCH_ERR_MSG("DEVICE_STATUS_REG read fail\n");
		return -EIO;
	}
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
	if (unlikely(touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, 0x01) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
			}

	if (unlikely(touch_i2c_read(ts->client, ANALOG_CONTROL_REG, sizeof(analog_control), &analog_control) < 0)) {
		TOUCH_ERR_MSG("ANALOG_CONTROL_REG read fail\n");
		return -EIO;
	}
	if (unlikely(touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, 0x00) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
			}
#endif 
	/* Firmware has a problem, so we should firmware-upgrade */
	if(device_status & DEVICE_STATUS_FLASH_PROG
			|| (device_status & DEVICE_CRC_ERROR_MASK) != 0
			|| (flash_control & FLASH_STATUS_MASK) != 0) {
		TOUCH_ERR_MSG("Firmware has a unknown-problem, so it needs firmware-upgrade.\n");
		TOUCH_ERR_MSG("FLASH_CONTROL[%x] DEVICE_STATUS_REG[%x]\n", (u32)flash_control, (u32)device_status);

		fw_info->fw_rev = 0;
	}

	ts->fw_info = fw_info;

	return 0;
}

static int relaxation_ctrl(struct i2c_client *client, u16 value)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);

	if(ts->analog_dsc.id == 0)
		return -EIO;
	
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x01) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, ANALOG_CONTROL_REG, value) < 0)) {
		TOUCH_ERR_MSG("ANALOG_CONTROL_REG write fail\n");
		return -EIO;
	}
	msleep(10);
	if (unlikely(touch_i2c_write_byte(client, ANALOG_COMMAND_REG, FORCE_UPDATE) < 0)) {
		TOUCH_ERR_MSG("force fast relaxation command write fail\n");
		return -EIO;
	}
	
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	return 0;
}

int synaptics_ts_init(struct i2c_client* client, struct touch_fw_info* fw_info)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);

	u8 buf = 0; //                                          

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	if (!ts->is_probed)
		if (unlikely(get_ic_info(ts, fw_info) < 0))
			return -EIO;

	if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
			DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
		TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(client, INTERRUPT_ENABLE_REG,
			1, &buf) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG read fail\n");
		return -EIO;
	}
	if (unlikely(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
			buf | ts->interrupt_mask.abs | ts->interrupt_mask.button) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, GESTURE_ENABLE_1_REG, 0x00) < 0)) {
		TOUCH_ERR_MSG("GESTURE_ENABLE_1_REG write fail\n");
		return -EIO;
	}
	if (unlikely(touch_i2c_write_byte(client, GESTURE_ENABLE_2_REG, 0x00) < 0)) {
		TOUCH_ERR_MSG("GESTURE_ENABLE_2_REG write fail\n");
		return -EIO;
	}

	if(ts->pdata->role->report_mode == CONTINUOUS_REPORT_MODE) {
		if (unlikely(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE,
				REPORT_MODE_CONTINUOUS) < 0)) {
			TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
			return -EIO;
		}
	} else {	/* REDUCED_REPORT_MODE */
		if (unlikely(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE,
				REPORT_MODE_REDUCED) < 0)) {
			TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
			return -EIO;
		}

		if (unlikely(touch_i2c_write_byte(client, DELTA_X_THRESH_REG,
				ts->pdata->role->delta_pos_threshold) < 0)) {
			TOUCH_ERR_MSG("DELTA_X_THRESH_REG write fail\n");
			return -EIO;
		}
		if (unlikely(touch_i2c_write_byte(client, DELTA_Y_THRESH_REG,
				ts->pdata->role->delta_pos_threshold) < 0)) {
			TOUCH_ERR_MSG("DELTA_Y_THRESH_REG write fail\n");
			return -EIO;
		}
	}

	if (unlikely(relaxation_ctrl(client, FORCE_FAST_RELAXATION)) < 0) {
		TOUCH_ERR_MSG("FORCE_FAST_RELAXATION write fail\n");
		return -EIO;
	}
	
	if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
		return -EIO;	// it is critical problem because interrupt will not occur.
	}

	if (unlikely(touch_i2c_read(client, FINGER_STATE_REG, sizeof(ts->ts_data.finger.finger_status_reg),
			ts->ts_data.finger.finger_status_reg) < 0)) {
		TOUCH_ERR_MSG("FINGER_STATE_REG read fail\n");
		return -EIO;	// it is critical problem because interrupt will not occur on some FW.
	}

	ts->is_probed = 1;

	return 0;
}

int synaptics_ts_power(struct i2c_client* client, int power_ctrl)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	switch (power_ctrl) {
	case POWER_OFF:
		if (ts->pdata->pwr->use_regulator) {
			regulator_disable(ts->regulator_vio);
			regulator_disable(ts->regulator_vdd);
		}
		else
			ts->pdata->pwr->power(0);

		break;
	case POWER_ON:
		if (ts->pdata->pwr->use_regulator) {
			regulator_enable(ts->regulator_vdd);
			regulator_enable(ts->regulator_vio);
		}
		else
			ts->pdata->pwr->power(1);

		#if 0 //X3 sora.jin 20120125 block code which cause kernel panic while booting
		/* P2 H/W bug fix */
		if (ts->pdata->reset_pin > 0) {
			msleep(10);
			gpio_set_value(ts->pdata->reset_pin, 0);
			msleep(ts->pdata->role->reset_delay);
			gpio_set_value(ts->pdata->reset_pin, 1);
		}
		#endif
		break;
	case POWER_SLEEP:
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_SLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
			TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
			return -EIO;
		}
		break;
	case POWER_WAKE:
		if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
			TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
			return -EIO;
		}
		break;
	default:
		return -EIO;
		break;
	}

	return 0;
}

int synaptics_ts_probe(struct i2c_client* client)
{
	struct synaptics_ts_data* ts;
	int ret = 0;

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	ts = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if (!ts) {
		TOUCH_ERR_MSG("Can not allocate memory\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	set_touch_handle(client, ts);

	ts->client = client;
	ts->pdata = client->dev.platform_data;

	if (ts->pdata->pwr->use_regulator) {
		ts->regulator_vdd = regulator_get_exclusive(NULL, ts->pdata->pwr->vdd);
		if (IS_ERR(ts->regulator_vdd)) {
			TOUCH_ERR_MSG("FAIL: regulator_get_vdd - %s\n", ts->pdata->pwr->vdd);
			ret = -EPERM;
			goto err_get_vdd_failed;
		}

		ts->regulator_vio = regulator_get_exclusive(NULL, ts->pdata->pwr->vio);
		if (IS_ERR(ts->regulator_vio)) {
			TOUCH_ERR_MSG("FAIL: regulator_get_vio - %s\n", ts->pdata->pwr->vio);
			ret = -EPERM;
			goto err_get_vio_failed;
		}

		if (ts->pdata->pwr->vdd_voltage > 0) {
			ret = regulator_set_voltage(ts->regulator_vdd, ts->pdata->pwr->vdd_voltage, ts->pdata->pwr->vdd_voltage);
			if (ret < 0)
				TOUCH_ERR_MSG("FAIL: VDD voltage setting - (%duV)\n", ts->pdata->pwr->vdd_voltage);
		}

		if (ts->pdata->pwr->vio_voltage > 0) {
			ret = regulator_set_voltage(ts->regulator_vio, ts->pdata->pwr->vio_voltage, ts->pdata->pwr->vio_voltage);
			if (ret < 0)
				TOUCH_ERR_MSG("FAIL: VIO voltage setting - (%duV)\n",ts->pdata->pwr->vio_voltage);
		}
	}

	return ret;

err_get_vio_failed:
	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vdd);
	}
err_get_vdd_failed:
err_alloc_data_failed:
	kfree(ts);
	return ret;
}

void synaptics_ts_remove(struct i2c_client* client)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}

	kfree(ts);
}

int synaptics_ts_fw_upgrade(struct i2c_client* client, const char* fw_path)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);
	int ret = 0;

	ts->is_probed = 0;

	ret = FirmwareUpgrade(ts, fw_path);

	/* update IC info */
	get_ic_info(ts, ts->fw_info);

	return ret;
}

int synaptics_ts_ic_ctrl(struct i2c_client *client, u8 code, u16 value)
{
	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);
	u8 buf = 0;

	switch (code)
	{
	case IC_CTRL_BASELINE:
		switch (value)
		{
		case BASELINE_OPEN:
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x01) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
			}
				if (unlikely(touch_i2c_write_byte(client, ANALOG_CONTROL_REG,
						FORCE_FAST_RELAXATION) < 0)) {
					TOUCH_ERR_MSG("ANALOG_CONTROL_REG write fail\n");
					return -EIO;
				}
				msleep(10);
				if (unlikely(touch_i2c_write_byte(client, ANALOG_COMMAND_REG, FORCE_UPDATE) < 0)) {
					TOUCH_ERR_MSG("force fast relaxation command write fail\n");
					return -EIO;
				}

			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST))
			TOUCH_INFO_MSG("BASELINE_OPEN  ~~~~~~~~\n");
#else			
			if (unlikely(relaxation_ctrl(client, FORCE_FAST_RELAXATION)) < 0) {
				TOUCH_ERR_MSG("FORCE_FAST_RELAXATION write fail\n");
				return -EIO;
			}
#endif 
			break;

		case BASELINE_FIX:
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_LGE_f100) //YJChae 
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x01) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
			}
					if (unlikely(touch_i2c_write_byte(client, ANALOG_CONTROL_REG,
							0x0) < 0)) {
						TOUCH_ERR_MSG("ANALOG_CONTROL_REG write fail\n");
						return -EIO;
					}
					msleep(10);
					if (unlikely(touch_i2c_write_byte(client, ANALOG_COMMAND_REG, FORCE_UPDATE) < 0)) {
						TOUCH_ERR_MSG("force fast relaxation command write fail\n");
						return -EIO;
					}

			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST))
			TOUCH_INFO_MSG("BASELINE_FIX  ~~~~~~~~\n");
#else
			if (unlikely(relaxation_ctrl(client, 0)) < 0) {
				TOUCH_ERR_MSG("FORCE_FAST_RELAXATION write fail\n");
				return -EIO;
			}
#endif 
			break;

		case BASELINE_REBASE:
			if (likely(ts->finger_dsc.id != 0)) {
				if (unlikely(touch_i2c_write_byte(client, FINGER_COMMAND_REG, 0x1) < 0)) {
					TOUCH_ERR_MSG("finger baseline reset command write fail\n");
					return -EIO;
				}
			}
			break;
		default:
			break;
		}
		break;
	case IC_CTRL_READ:
		if (touch_i2c_read(client, value, 1, &buf) < 0) {
			TOUCH_ERR_MSG("IC register read fail\n");
			return -EIO;
		}
		break;
	case IC_CTRL_WRITE:
		if (touch_i2c_write_byte(client, ((value & 0xFF00) >> 8), (value & 0xFF)) < 0) {
			TOUCH_ERR_MSG("IC register write fail\n");
			return -EIO;
		}
		break;
	case IC_CTRL_RESET_CMD:
		if (unlikely(touch_i2c_write_byte(client, DEVICE_COMMAND_REG, 0x1) < 0)) {
			TOUCH_ERR_MSG("IC Reset command write fail\n");
			return -EIO;
		}
		break;
	default:
		break;
	}

	return buf;
}

int synaptics_ts_test_report(struct i2c_client *client, char* buf, size_t * count)
{

	struct synaptics_ts_data* ts =
			(struct synaptics_ts_data*)get_touch_handle(client);


	u8 command;
	u8 command_read;

	short ImageArray[13][23] = {0,};
	u8 ImageBuffer[13*23*2]= {0,};

	int i, j, k;
	int ret = 0;
	int length;
	unsigned short numberOfRows;
	unsigned short numberOfColumns;

	TOUCH_INFO_MSG("%s\n",__func__);


	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	// INITIALIZE 
	command = 0;
	command_read = 0;
    i = 0 ; j = 0 ; k = 0;

	// Enabling only the analog image reporting interrupt, and turn off the rest
	command = 0x08;
	if (touch_i2c_read(client,INTERRUPT_ENABLE_REG,sizeof(command_read), &command_read) < 0) {
		TOUCH_ERR_MSG("IC register read fail : INTERRUPT_ENABLE_REG\n");
		return -EIO;
	} 
	TOUCH_INFO_MSG("%s : INTERRUPT_ENABLE_REG : 0x%x\n",__func__,command_read);

   	if (unlikely(touch_i2c_write_byte(client,INTERRUPT_ENABLE_REG, command_read & command) < 0)) {
			TOUCH_ERR_MSG("Ic register write fail : INTERRUPT_ENABLE_REG\n");
			return -EIO;
	}
	if (touch_i2c_read(client,INTERRUPT_ENABLE_REG,sizeof(command_read), &command_read) < 0) {
		TOUCH_ERR_MSG("IC register read fail : INTERRUPT_ENABLE_REG\n");
		return -EIO;
	} 

	TOUCH_INFO_MSG("%s : INTERRUPT_ENABLE_REG : 0x%x\n",__func__,command_read);

	// Select Page 0x00 -> 0x01 to access F54 test report registers
	command = 0x01; 
   	if (unlikely(touch_i2c_write_byte(client,PAGE_SELECT_REG, command) < 0)) {
			TOUCH_ERR_MSG("Ic register write fail : PAGE_SELECT_REG\n");
			return -EIO;
	}
	if (touch_i2c_read(client,PAGE_SELECT_REG, 1, &command_read) < 0) {
		TOUCH_ERR_MSG("IC register read fail : PAGE_SELECT_REG\n");
		return -EIO;
	}
	TOUCH_INFO_MSG("%s : PAGE_SELECT_REG : 0x%x\n",__func__,command_read);

	k = 0;       
	// Set report mode to read 16-bit image
	command = test_report_type;// 0x02 : delta capacitance report , 0x03 : raw capacitance  

	if (unlikely(touch_i2c_write_byte(client,TEST_REPORT_DATA_TYPE, command) < 0)) {
			TOUCH_ERR_MSG("Ic register write fail : TEST_REPORT_DATA_TYPE\n");
			return -EIO;
	}
	if (touch_i2c_read(client,TEST_REPORT_DATA_TYPE, sizeof(command_read) , &command_read) < 0) {
		TOUCH_ERR_MSG("IC register read fail : TEST_REPORT_DATA_TYPE\n");
		return -EIO;
	}

	TOUCH_INFO_MSG("%s : TEST_REPORT_DATA_TYPE : 0x%x\n",__func__, command_read);

	// data index low / data index high
	command = 0x00;
   	if (unlikely(touch_i2c_write_byte(client,TEST_REPORT_DATA_INDEX_LOW, command) < 0)) {
			TOUCH_ERR_MSG("Ic register write fail : TEST_REPORT_DATA_INDEX_LOW\n");
			return -EIO;
	}
	if (touch_i2c_read(client,TEST_REPORT_DATA_INDEX_LOW, sizeof(command_read) , &command_read) < 0) {
		TOUCH_ERR_MSG("IC register read fail : TEST_REPORT_DATA_INDEX_LOW\n");
		return -EIO;
	}
	TOUCH_INFO_MSG("%s : TEST_REPORT_DATA_INDEX_LOW : 0x%x\n",__func__,command_read);

   	if (unlikely(touch_i2c_write_byte(client,TEST_REPORT_DATA_INDEX_HIGH, command) < 0)) {
			TOUCH_ERR_MSG("Ic register write fail : TEST_REPORT_DATA_INDEX_HIGH\n");
			return -EIO;
	}
	if (touch_i2c_read(client,TEST_REPORT_DATA_INDEX_HIGH, sizeof(command_read) , &command_read) < 0) {
		TOUCH_ERR_MSG("IC register read fail : TEST_REPORT_DATA_INDEX_HIGH\n");
		return -EIO;
	}
	TOUCH_INFO_MSG("%s : TEST_REPORT_DATA_INDEX_HIGH : 0x%x\n",__func__,command_read);


	// Set the GetReport bit to acquire an image
	command = 0x01;	  //F54_AD_Cmd0
   	if (unlikely(touch_i2c_write_byte(client,TEST_REPORT_CMD_BASE, command) < 0)) {
			TOUCH_ERR_MSG("Ic register write fail :TEST_REPORT_CMD_BASE\n");
			return -EIO;
	}

	// Wait until the command is completed
	do {
	   	if (touch_i2c_read(client,TEST_REPORT_CMD_BASE, sizeof(command_read) , &command_read) < 0) {
			TOUCH_ERR_MSG("IC register read fail : TEST_REPORT_CMD_BASE\n");
			return -EIO;
		}

	} while (command_read == 0x01);

	TOUCH_INFO_MSG("%s : TEST_REPORT_CMD_BASE : 0x%x\n",__func__,command_read);

	// READ DATA
	numberOfRows = 13;
	numberOfColumns = 23;
	length = numberOfRows * numberOfColumns * 2;

	if (touch_i2c_read(client,TEST_REPORT_DATA_BUFF, length, &ImageBuffer[0]) < 0) {
		TOUCH_ERR_MSG("IC register read fail : TEST_REPORT_DATA_BUFF\n");
		return -EIO;
	}


	//ret += sprintf(buf+ret, "[imageBuffer][%d]---START- ",length);
	for (i = 0; i < numberOfRows; i++)
	{
		//ret += sprintf(buf+ret, "[imageBuffer][%d][ ",i);
		for (j = 0; j < numberOfColumns; j++)
		{
			ImageArray[i][j] = (short)(ImageBuffer[k] | (ImageBuffer[k+1] << 8));
			//ret += sprintf(buf+ret,"%d %d", ImageBuffer[k],ImageBuffer[k], ImageBuffer[k+1], ImageBuffer[k+1]);
			k = k + 2;
		}    
		//ret += sprintf(buf+ret,"]\n");
	}


	for (i = 0; i < numberOfRows; i++)
	{   
		ret += sprintf(buf+ret, "[%d][ ",i);
		for (j = 0; j < numberOfColumns; j++)
		{
			ret += sprintf(buf+ret,"%d ", ImageArray[i][j]);
		}
		ret += sprintf(buf+ret,"]\n");
	}

	*count = ret;

	return ret;
}

struct touch_device_driver synaptics_ts_driver = {
	.probe 	= synaptics_ts_probe,
	.remove	= synaptics_ts_remove,
	.init  	= synaptics_ts_init,
	.data  	= synaptics_ts_get_data,
	.power 	= synaptics_ts_power,
	.fw_upgrade = synaptics_ts_fw_upgrade,
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.test_report = synaptics_ts_test_report, //F54
};

static int __devinit touch_init(void)
{
	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	return touch_driver_register(&synaptics_ts_driver);
}

static void __exit touch_exit(void)
{
	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	touch_driver_unregister();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("yehan.ahn@lge.com, hyesung.shin@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");

