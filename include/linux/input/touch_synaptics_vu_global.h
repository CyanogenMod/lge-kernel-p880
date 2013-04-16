/* include/linux/lge_touch_core.h
 *
 * Copyright (C) 2012 LGE.
 *
 * Writer: yehan.ahn@lge.com, 	hyesung.shin@lge.com
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

#ifndef LGE_TOUCH_SYNAPTICS_H
#define LGE_TOUCH_SYNAPTICS_H

//#define ARRAYED_TOUCH_FW_BIN

#define NUM_OF_EACH_FINGER_DATA_REG		5
#define MAX_NUM_OF_FINGERS				10

#define DESCRIPTION_TABLE_START			0xe9

#define PAGE_SELECT_REG					0xFF		/* Button exists Page 02 */
#define PAGE_MAX_NUM					4			/* number of page register */

struct function_descriptor {
	u8 	query_base;
	u8 	command_base;
	u8 	control_base;
	u8 	data_base;
	u8 	int_source_count;
	u8 	id;
};

struct ts_ic_function {
	struct function_descriptor dsc;
	u8 	function_page;
};

struct finger_data {
	u8	finger_status_reg[3];
	u8	finger_reg[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];
};

struct button_data {
	u16	key_code;
};

struct cur_touch_data {
	u8	device_status_reg;		/* DEVICE_STATUS_REG */
	u8	interrupt_status_reg;
	u8	button_data_reg;
	struct finger_data	finger;
	struct button_data	button;
};

struct interrupt_bit_mask {
	u8 button;
};

struct synaptics_ts_data {
	u8	is_probed;
	struct regulator*	regulator_vdd;
	struct regulator*	regulator_vio;
	struct i2c_client*	client;
	struct touch_platform_data*		pdata;
	struct ts_ic_function	common_fc;
	struct ts_ic_function	finger_fc;
	struct ts_ic_function	button_fc;
	struct ts_ic_function	analog_fc;	/* FIXME: not used in ClearPad3000 serise */
	struct ts_ic_function	flash_fc;
	struct cur_touch_data		ts_data;
	struct touch_fw_info*	fw_info;
	struct interrupt_bit_mask	interrupt_mask;
};

/* extern function */
extern int FirmwareUpgrade(struct synaptics_ts_data *ts, const char* fw_path);
int synaptics_ts_page_data_read(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data);
int ts_page_data_write(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data);
int synaptics_ts_page_data_write_byte(struct i2c_client *client, u8 page, u8 reg, u8 data);

#endif
