/* lge/com_device/input/touch_synaptics_rmi4_i2c.c
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * Author: hyesung.shin@lge.com
 *
 * Notice: This is synaptic general touch driver for using RMI4.
 *		It not depends on specific regitser map and device spec.
 * 		If you want use this driver, you should completly understand
 * 		synaptics RMI4 register map structure.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/input/touch_synaptics_rmi4_i2c.h>


#define SYNATICS_DEVICE_NAME "synaptics_T1320" //"synaptics_ts"

/* SYNAPTICS_RMI4_I2C Debug mask value
 * usage: echo [debug_mask] > /sys/module/touch_synaptics_rmi4_i2c/parameters/debug_mask
 * All			: 8191 (0x1FFF)
 * No msg		: 32
 * default		: 0
 */
static unsigned int synaptics_rmi4_i2c_debug_mask = \
		SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_STATUS | \
		SYNAPTICS_RMI4_I2C_DEBUG_NONE;

module_param_named(debug_mask, synaptics_rmi4_i2c_debug_mask, int,
		S_IRUGO|S_IWUSR|S_IWGRP);

/* RMI4 spec from (RMI4 spec)511-000136-01_revD
 * Function	Purpose			See page
 * $01		RMI Device Control			29
 * $08		BIST(Built-in Self Test)		38
 * $09		BIST(Built-in Self Test)		42
 * $11		2-D TouchPad sensors			46
 * $19		0-D capacitive button sensors		69
 * $30		GPIO/LEDs (includes mechanical buttons)	76
 * $32		Timer					89
 * $34		Flash Memory Management			93
 */
#define RMI_DEVICE_CONTROL		0x01
#define TOUCHPAD_SENSORS		0x11
#define CAPACITIVE_BUTTON_SENSORS	0x19
#define GPIO_LEDS			0x30
#define TIMER				0x32
#define FLASH_MEMORY_MANAGEMENT		0x34

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
#define DEVICE_STATUS_REG		(ts->common_dsc.data_base)	/* Device Status */
#define DEVICE_STATUS_REG_UNCONFIGURED	0x80
#define DEVICE_FAILURE_MASK		0x03


#define INTERRUPT_STATUS_REG		(ts->common_dsc.data_base+1)	/* Interrupt Status */
#define BUTTON_DATA_REG			(ts->button_dsc.data_base)	/* Button Data */

#define FINGER_STATE_REG		(ts->finger_dsc.data_base)	/* Finger State */
#define FINGER_STATE_MASK		0x03
#define REG_X_POSITION			0
#define REG_Y_POSITION			1
#define REG_YX_POSITION			2
#define REG_WY_WX			3
#define REG_Z				4
#define NUM_OF_EACH_FINGER_DATA_REG	5

#define DEVICE_CONTROL_REG 		(ts->common_dsc.control_base)	/* Device Control */
#define DEVICE_CONTROL_REG_SLEEP 	0x01
#define DEVICE_CONTROL_REG_NOSLEEP	0x02
#define DEVICE_CONTROL_REG_CONFIGURED	0x80

#define PALM_DETECT_REG 		(ts->finger_dsc.control_base+1)		/* Palm Detect */
#define DELTA_X_THRESH_REG 		(ts->finger_dsc.control_base+2)		/* Delta-X Thresh */
#define DELTA_Y_THRESH_REG 		(ts->finger_dsc.control_base+3)		/* Delta-Y Thresh */
#define GESTURE_ENABLE_1_REG 		(ts->finger_dsc.control_base+10)	/* Gesture Enables 1 */
#define GESTURE_ENABLE_2_REG 		(ts->finger_dsc.control_base+11)	/* Gesture Enables 2 */

#define MANUFACTURER_ID_REG		(ts->common_dsc.query_base)			/* Manufacturer ID */
#define FW_REVISION_REG			(ts->common_dsc.query_base+3)		/* FW revision */
#define PRODUCT_ID_REG			(ts->common_dsc.query_base+11)		/* Product ID */

#define MELT_CONTROL_REG		0xF0
#define MELT_CONTROL_REG_NO_MELT	0x00
#define MELT_CONTROL_REG_MELT		0x01
#define MELT_CONTROL_REG_NUKE_MELT	0x80

/* Macro */
#define GET_X_POSITION(high, low) 	((int)(high<<4)|(int)(low&0x0F))
#define GET_Y_POSITION(high, low) 	((int)(high<<4)|(int)((low&0xF0)>>4))

/* General define */
#define TOUCH_PRESSED			1
#define TOUCH_RELEASED			0

/* Define for Area based key button */
#define BUTTON_MARGIN			50
#define TOUCH_BUTTON_PRESSED		2

typedef struct {
	unsigned char device_status_reg;		/* DEVICE_STATUS_REG */
	unsigned char interrupt_status_reg;
	unsigned char button_data_reg;
} ts_sensor_ctrl;

typedef struct {
	unsigned char finger_state_reg[3];		/* 2D_FINGER_STATUS_1_REG ~ */

	/* Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
	 *				X Position [11:4]
	 *				Y Position [11:4]
	 * 		Y Position [3:0] 		X Position [3:0]
	 *				Wy 				Wx
	 *						Z
	 */
	unsigned char finger_data[MAX_NUM_OF_FINGER][NUM_OF_EACH_FINGER_DATA_REG];
} ts_sensor_data;

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

#if defined(CONFIG_LGE_TOUCH_SYNAPTICS_FW_UPGRADE)
extern int FirmwareUpgrade(struct synaptics_ts_data *ts);
extern unsigned char get_fw_image_rev(void);
#endif

static struct workqueue_struct *synaptics_wq;

int get_synaptics_ts_debug_mask(void)
{
	return synaptics_rmi4_i2c_debug_mask;
}
EXPORT_SYMBOL(get_synaptics_ts_debug_mask);

int synaptics_ts_read(struct i2c_client *client, u8 reg, int num, u8 *buf)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = num,
			.buf = buf,
		},
	};

	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		if (printk_ratelimit())
			SYNAPTICS_ERR_MSG("transfer error\n");
		return -EIO;
	} else
		return 0;
}
EXPORT_SYMBOL(synaptics_ts_read);

int synaptics_ts_write(struct i2c_client *client, u8 reg, u8 * buf, int len)
{
	unsigned char send_buf[len + 1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = len+1,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;
	memcpy(&send_buf[1], buf, len);

	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
		if (printk_ratelimit())
			SYNAPTICS_ERR_MSG("transfer error\n");
		return -EIO;
	} else
		return 0;
}
EXPORT_SYMBOL(synaptics_ts_write);

static void synaptics_ts_soft_reset(struct synaptics_ts_data *ts)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(ts->client,
			ts->common_dsc.command_base /* Device command */, 0x1);
	if (ret < 0)
		SYNAPTICS_ERR_MSG("Soft reset command write fail\n");

	queue_delayed_work(synaptics_wq,
			&ts->work,msecs_to_jiffies(ts->pdata->ic_booting_delay));
}

static void synaptics_ts_hard_reset(struct synaptics_ts_data *ts)
{
	int ret = 0;

	/* 1. VIO off
	 * 2. VDD off
	 * 3. Wait more than 10ms
	 * 4. VDD on
	 * 5. VIO on
	 * 6. Initialization
	 */
	if (ts->pdata->power) {
		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
			ret = ts->pdata->power(0, true);
		else
			ret = ts->pdata->power(0, false);

		if (ret < 0)
			SYNAPTICS_ERR_MSG("power on failed\n");

		mdelay(10);

		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
			ret = ts->pdata->power(1, true);
		else
			ret = ts->pdata->power(1, false);

		if (ret < 0)
			SYNAPTICS_ERR_MSG("power on failed\n");
	}

	queue_delayed_work(synaptics_wq,
			&ts->work,msecs_to_jiffies(ts->pdata->ic_booting_delay));
}

#if defined(CONFIG_LGE_TOUCH_SYNAPTICS_FW_UPGRADE)
static void synaptics_ts_fw_upgrade(struct synaptics_ts_data *ts)
{
	int ret = 0;
	struct synaptics_ts_timestamp time_debug;

	if (likely(!ts->is_downloading)) {
		ts->is_downloading = 1;
		ts->is_probed = 0;

		if (likely(!ts->is_suspended)) {
			ts->ic_init = 0;

			if (ts->pdata->use_irq)
				disable_irq_nosync(ts->client->irq);
			else
				hrtimer_cancel(&ts->timer);
		} else {
			if (ts->pdata->power) {
				if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
					ret = ts->pdata->power(1, true);
				else
					ret = ts->pdata->power(1, false);

				if (ret < 0) {
					SYNAPTICS_ERR_MSG("power on failed\n");
				} else {
					msleep(ts->pdata->ic_booting_delay);
				}
			}
		}

		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_UPGRADE_DELAY) {
			memset(&time_debug, 0x0, sizeof(struct synaptics_ts_timestamp));
			atomic_set(&time_debug.ready, 1);
			time_debug.start = cpu_clock(smp_processor_id());
		}

		ret = FirmwareUpgrade(ts);
		if(ret < 0) {
			printk(KERN_ERR "[Touch E] Firmware upgrade Fail!!!\n");
		} else {
			SYNAPTICS_INFO_MSG("Firmware upgrade Complete\n");
		}

		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_UPGRADE_DELAY) {
			if (atomic_read(&time_debug.ready) == 1) {
				time_debug.end = cpu_clock(smp_processor_id());
				time_debug.result_t = time_debug.end - time_debug.start;
				time_debug.rem = do_div(time_debug.result_t , 1000000000);
				SYNAPTICS_DEBUG_MSG("FW upgrade time < %2lu.%06lu\n",
						(unsigned long)time_debug.result_t, time_debug.rem/1000);
				atomic_set(&time_debug.ready, 0);
			}
		}

		if (likely(!ts->is_suspended)) {
			if (ts->pdata->use_irq)
				enable_irq(ts->client->irq);
			else
				hrtimer_start(&ts->timer,
						ktime_set(0, ts->pdata->report_period+(ts->pdata->ic_booting_delay*1000000)),
						HRTIMER_MODE_REL);
		}

		if (likely(!ts->is_suspended)) {
			synaptics_ts_hard_reset(ts);
		} else {
			if (ts->pdata->power) {
				if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
					ret = ts->pdata->power(0, true);
				else
					ret = ts->pdata->power(0, false);

				if (ret < 0)
					SYNAPTICS_ERR_MSG("power on failed\n");
			}
		}

		ts->is_downloading = 0;
	} else {
		SYNAPTICS_ERR_MSG("Firmware Upgrade process is aready working on\n");
	}
}
#endif

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);

	/* ignore irrelevant timer interrupt during IC power on */
	if (likely(ts->ic_init)) {
		queue_delayed_work(synaptics_wq, &ts->work, 0);
		hrtimer_start(&ts->timer,
				ktime_set(0, ts->pdata->report_period), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static ssize_t show_int_gpio_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", gpio_get_value(ts->pdata->i2c_int_gpio));
}

static ssize_t show_fw_revision(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;

	ret = sprintf(buf, "Touch IC Chip Info\n - Product ID: %s\n", ts->product_id);
	ret += sprintf(buf+ret, " - Manufacturer ID: %d, FW revision: %d\n",
			ts->manufcturer_id, ts->fw_rev);
#if defined(CONFIG_LGE_TOUCH_SYNAPTICS_FW_UPGRADE)
	ret += sprintf(buf+ret, " - Kernel wished FW revision: %d,"
		       " FW binary revision: %d\n",
			ts->pdata->fw_ver, get_fw_image_rev());
#endif

	return ret;
}

static ssize_t show_ts_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;


	if (ts->pdata->use_irq) {
		ret = sprintf(buf, "interrupt\n");
	} else {
		ret = sprintf(buf, "polling\n");
	}

	return ret;
}

static ssize_t store_ts_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unsigned char string[5];


	sscanf(buf, "%s", string);

	if (ts->pdata->use_irq)
		disable_irq_nosync(ts->client->irq);
	else
		hrtimer_cancel(&ts->timer);

	if (!strncmp(string, "interrupt", 10)) {
		if (!ts->pdata->use_irq) {
			ts->pdata->use_irq = 1;
			if (!ts->is_suspended)
				enable_irq(ts->client->irq);

			SYNAPTICS_INFO_MSG("Interrupt mode setting\n");
		} else
			return count;
	} else if (!strncmp(string, "polling", 8)) {
		if (ts->pdata->use_irq) {
			ts->pdata->use_irq = 0;

			if (!ts->timer.function) {
				hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
				ts->timer.function = synaptics_ts_timer_func;
			}

			if (!ts->is_suspended)
				hrtimer_start(&ts->timer,
						ktime_set(0, ts->pdata->report_period),
						HRTIMER_MODE_REL);

			SYNAPTICS_INFO_MSG("Polling mode setting\n");
		}
		else
			return count;
	} else {
		SYNAPTICS_INFO_MSG("WARNING: You can use this mode change only interrupt mode booting\n");
		SYNAPTICS_INFO_MSG("Usage: echo [interrupt | polling] > ts_mode\n");
		SYNAPTICS_INFO_MSG(" - interrupt : Change to interrupt mode start\n");
		SYNAPTICS_INFO_MSG(" - polling : Change to polling mode\n");
	}

	return count;
}

static ssize_t show_ts_sens(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;


	ret = sprintf(buf, "Palm Detect threshold (palm_threshold)\t\t\t: 0x%x\n", ts->pdata->palm_threshold);
	ret += sprintf(buf+ret, "Delta Position threshold (delta_pos_threshold)\t\t: 0x%x\n", ts->pdata->delta_pos_threshold);

	return ret;
}

static ssize_t store_ts_sens(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	int ret;
	unsigned char string[5];
	unsigned int value;


	sscanf(buf, "%s %x", string, &value);

	value = 0xFF & value;

	if (!strncmp(string, "palm", 4)) {
		ret = i2c_smbus_write_byte_data(ts->client, PALM_DETECT_REG, (u8)value);
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("PALM_DETECT_REG write fail\n");
			return count;
		} else {
			SYNAPTICS_INFO_MSG("Palm threshold is setted to0x%x\n", (u8)value);
			ts->pdata->palm_threshold = value;
		}
	} else if (!strncmp(string, "delta", 5)) {
		ret = i2c_smbus_write_byte_data(ts->client, DELTA_X_THRESH_REG, (u8)value);
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("DELTA_X_THRESH_REG write fail\n");
			return count;
		} else {
			SYNAPTICS_INFO_MSG("Delta X threshold is setted to 0x%x\n", (u8)value);
			ts->pdata->delta_pos_threshold= value;
		}

		ret = i2c_smbus_write_byte_data(ts->client, DELTA_Y_THRESH_REG, (u8)value);
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("DELTA_Y_THRESH_REG write fail\n");
			return count;
		} else {
			SYNAPTICS_INFO_MSG("Delta Y threshold is setted to 0x%x\n", (u8)value);
			ts->pdata->delta_pos_threshold= value;
		}
	} else {
		SYNAPTICS_INFO_MSG("Usage: echo [palm | delta] value(16)> ts_sens\n");
		SYNAPTICS_INFO_MSG(" - palm : palm threshold register setting\n");
		SYNAPTICS_INFO_MSG(" - delta : X,Y delta threshold register setting\n");
	}

	return count;
}

static ssize_t show_ts_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;
	int i = 0;

	ret = sprintf(buf, "====== Platform data ======\n");
	ret += sprintf(buf+ret, "Operation mode (use_irq)\t\t\t\t: %s (%d)\n",
			ts->pdata->use_irq?"Interrupt":"Polling", ts->pdata->use_irq);
	ret += sprintf(buf+ret, "IRQ flag (irqflags)\t\t\t\t\t: %ld\n", ts->pdata->irqflags);
	//ret += sprintf(buf+ret, "I2C SDA (i2c_sda_gpio)\t\t\t\t\t: %d\n", ts->pdata->i2c_sda_gpio);
	//ret += sprintf(buf+ret, "I2C SCL (ni2c_scl_gpio)\t\t\t\t\t: %d\n", ts->pdata->i2c_scl_gpio);
	ret += sprintf(buf+ret, "I2C Interrupt (i2c_int_gpio)\t\t\t\t: %d\n", ts->pdata->i2c_int_gpio);
	ret += sprintf(buf+ret, "IC booting delay (booting_delay)\t\t\t: %dms\n", ts->pdata->ic_booting_delay);
 	ret += sprintf(buf+ret, "IC interrupt interval (report_period)\t\t\t: %ldns\n", ts->pdata->report_period);
	ret += sprintf(buf+ret, "Number of Finger (num_of_finger)\t\t\t: %d\n", ts->pdata->num_of_finger);
	ret += sprintf(buf+ret, "Number of Button (num_of_button)\t\t\t: %d\n", ts->pdata->num_of_button);
	ret += sprintf(buf+ret, "Button key code\t\t\t\t\t\t: ");
	if (ts->pdata->num_of_button != 0) {
		for (i = 0; i < ts->pdata->num_of_button; i++) {
			ret += sprintf(buf+ret, "button[%d] - %d", i, ts->pdata->button[i]);
			if (i < ts->pdata->num_of_button -1)
				ret += sprintf(buf+ret, "\n\t\t\t\t\t\t\t  ");
		}
		ret += sprintf(buf+ret, "\n");
	}
	ret += sprintf(buf+ret, "Touch 2D area\t\t\t\t\t\t: X - 0 ~ %d (x_max)\n", ts->pdata->x_max);
	ret += sprintf(buf+ret, "\t\t\t\t\t\t\t  Y - 0 ~ %d (y_max)\n", ts->pdata->y_max);
	ret += sprintf(buf+ret, "Needed FW revision (fw_ver)\t\t\t\t: %d\n", ts->pdata->fw_ver);
	ret += sprintf(buf+ret, "Palm Detect threshold (palm_threshold)\t\t\t: 0x%x\n", ts->pdata->palm_threshold);
	ret += sprintf(buf+ret, "Delta Position threshold (delta_pos_threshold)\t\t: 0x%x\n", ts->pdata->delta_pos_threshold);

	ret += sprintf(buf+ret, "\n====== Device data ======\n");
	ret += sprintf(buf+ret, "Download process (is_downloading)\t\t\t: %s (%d)\n",
			ts->is_downloading?"Working":"Idle", ts->is_downloading);
	ret += sprintf(buf+ret, "Suspend status (is_suspended)\t\t\t\t: %s (%d)\n",
			ts->is_suspended?"Suspend":"Idle", ts->is_suspended);
	if (ts->area_detect_type_button)
		ret += sprintf(buf+ret, "2D Button width (button_width)\t\t\t: %d\n", ts->button_width);
	ret += sprintf(buf+ret, "Button previous status (button_prestate)\t\t: ");
	for (i = 0; i < ts->pdata->num_of_button; i++) {
		ret += sprintf(buf+ret, "%d", ts->button_prestate[i]);
		if (i < ts->pdata->num_of_button -1)
			ret += sprintf(buf+ret, ",  ");
	}
	ret += sprintf(buf+ret, "\n");
	ret += sprintf(buf+ret, "Finger previous status (finger_prestate)\t\t: ");
	for (i = 0; i < ts->pdata->num_of_finger; i++) {
		ret += sprintf(buf+ret, "%d", ts->finger_prestate[i]);
		if (i < ts->pdata->num_of_finger -1)
			ret += sprintf(buf+ret, ", ");
	}
	ret += sprintf(buf+ret, "\n");
	ret += sprintf(buf+ret, "IC init status (ic_init)\t\t\t\t: %s (%d)\n",
			ts->ic_init?"OK":"Fail", ts->ic_init);
	ret += sprintf(buf+ret, "Driver Probe status (is_probed)\t\t\t\t: %s (%d)\n",
			ts->is_probed?"OK":"Fail", ts->is_probed);
	ret += sprintf(buf+ret, "No Melt mode setting (melt_mode)\t\t\t: %s (%d)\n",
			ts->melt_mode?"No Melt":"Melt", ts->melt_mode);
	ret += sprintf(buf+ret, "No melt setting drag distance (idle_lock_distance)\t: %d\n",
			ts->idle_lock_distance);
	i = atomic_read(&ts->interrupt_handled);
	ret += sprintf(buf+ret, "Interrupt status (interrupt_handled)\t\t\t: %s (%d)\n",
			i?"Not Handled":"Handled", i);
	ret += sprintf(buf+ret, "IC supported Function\t\t\t\t\t: $%02x, $%02x, $%02x\n",
			ts->common_dsc.id, ts->finger_dsc.id, ts->button_dsc.id);
	ret += sprintf(buf+ret, "Button Type (area_detect_type_button)\t\t\t: %s (%d)\n",
			ts->area_detect_type_button?"2D button":"0D button", ts->area_detect_type_button);
	ret += sprintf(buf+ret, "Finger Interrupt bit mask (int_status_reg_asb0_bit)\t: 0x%02x\n",
			ts->int_status_reg_asb0_bit);
	ret += sprintf(buf+ret, "Button Interrupt bit mask (int_status_reg_button_bit)\t: 0x%02x\n",
			ts->int_status_reg_button_bit);
	ret += sprintf(buf+ret, "Current Finger position (pre_ts_data)\t\t\t: ");
	for (i = 0; i < ts->pdata->num_of_finger; i++) {
		ret += sprintf(buf+ret, "(%d, %d, %d)", ts->pre_ts_data.pos_x[i],
				ts->pre_ts_data.pos_y[i], ts->pre_ts_data.pressure[i]);
		if (i != 0 && (i+1)%2 == 0 && i != ts->pdata->num_of_finger-1)
			ret += sprintf(buf+ret, "\n\t\t\t\t\t\t\t  ");
		else if (i < ts->pdata->num_of_finger -1)
			ret += sprintf(buf+ret, ", ");
	}
	ret += sprintf(buf+ret, "\n");
	ret += sprintf(buf+ret, "IC FW Revision (fw_rev)\t\t\t\t\t: %d\n", ts->fw_rev);
	ret += sprintf(buf+ret, "IC Manufacture ID (manufcturer_id)\t\t\t: %d\n", ts->manufcturer_id);
	ret += sprintf(buf+ret, "IC Product ID (product_id)\t\t\t\t: %s\n", ts->product_id);

	return ret;
}

#if defined(CONFIG_LGE_TOUCH_SYNAPTICS_FW_UPGRADE)
static ssize_t store_fw_upgrade(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	int value = 0;
	int repeat = 0;

	sscanf(buf, "%d %s", &value, ts->fw_path);

	printk(KERN_INFO "\n");
	if(ts->fw_path[0] != 0)
		SYNAPTICS_INFO_MSG("Firmware image path: %s\n",
				ts->fw_path[0] != 0 ? ts->fw_path : "Internal");

	for(repeat = 0; repeat < value; repeat++) {
		msleep(ts->pdata->ic_booting_delay * 2);
		printk(KERN_INFO "\n");
		SYNAPTICS_INFO_MSG("Firmware image upgrade: No.%d", repeat+1);
		synaptics_ts_fw_upgrade(ts);
	}

	memset(ts->fw_path, 0x0, sizeof(ts->fw_path));

	return count;
}
#endif

static ssize_t store_synaptics_ts_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unsigned char string[5];

	sscanf(buf, "%s", string);

	if (!ts->is_suspended) {
		if (!strncmp(string, "soft", 4))
			synaptics_ts_soft_reset(ts);
		else if (!strncmp(string, "hard", 4))
			synaptics_ts_hard_reset(ts);
		else {
			SYNAPTICS_INFO_MSG("Usage: echo [soft | hard] > ts_reset\n");
			SYNAPTICS_INFO_MSG(" - soft : soft reset start\n");
			SYNAPTICS_INFO_MSG(" - hard : hard reset start\n");
		}
	} else
		SYNAPTICS_INFO_MSG("Touch is power off. Don't need reset\n");

	return count;
}

static DEVICE_ATTR(int_gpio, S_IRUGO | S_IWUSR, show_int_gpio_value, NULL);
static DEVICE_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_fw_revision, NULL);
static DEVICE_ATTR(ts_mode, S_IRUGO | S_IWUSR, show_ts_mode, store_ts_mode);
static DEVICE_ATTR(ts_sens, S_IRUGO | S_IWUSR, show_ts_sens, store_ts_sens);
static DEVICE_ATTR(ts_info, S_IRUGO | S_IWUSR, show_ts_info, NULL);
#if defined(CONFIG_LGE_TOUCH_SYNAPTICS_FW_UPGRADE)
static DEVICE_ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, store_fw_upgrade);
#endif
static DEVICE_ATTR(ts_reset, S_IRUGO | S_IWUSR, NULL, store_synaptics_ts_reset);

static struct attribute *synaptics_ts_attributes[] = {
	&dev_attr_int_gpio.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_ts_mode.attr,
	&dev_attr_ts_sens.attr,
	&dev_attr_ts_info.attr,
#if defined(CONFIG_LGE_TOUCH_SYNAPTICS_FW_UPGRADE)
	&dev_attr_fw_upgrade.attr,
#endif
	&dev_attr_ts_reset.attr,
	NULL,
};

static struct attribute_group synaptics_ts_attribute_group = {
	.attrs = synaptics_ts_attributes,
};

static void read_page_description_table(struct synaptics_ts_data *ts)
{
	/* Read config data */
	int ret = 0;
	ts_function_descriptor buffer;
	unsigned short u_address;

	memset(&buffer, 0x0, sizeof(ts_function_descriptor));

	ts->common_dsc.id = 0;
	ts->finger_dsc.id = 0;
	ts->button_dsc.id = 0;
	ts->area_detect_type_button = 0;

	for(u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(ts_function_descriptor)) {
		ret = synaptics_ts_read(ts->client, u_address, sizeof(buffer), (unsigned char *)&buffer);
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("ts_function_descriptor read fail\n");
			return;
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
		case CAPACITIVE_BUTTON_SENSORS:
			ts->button_dsc = buffer;
			break;
		}
	}

	if (ts->button_dsc.id == 0)
		ts->area_detect_type_button = 1; /* no button data register */
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts =
			container_of(to_delayed_work(work), struct synaptics_ts_data, work);
	int ret = 0;
	int width = 0;
	unsigned int f_counter = 0;
	unsigned int b_counter = 0;
	unsigned int reg_num = 0;
	unsigned int finger_order = 0;
	u8 temp;
	char report_enable = 0;
	ts_sensor_ctrl ts_reg_ctrl;
	ts_sensor_data ts_reg_data;
	ts_finger_data curr_ts_data;
	unsigned int button_start = 0;
	unsigned int button_pressed[MAX_NUM_OF_BUTTON][MAX_NUM_OF_FINGER] = {{0}};

	memset(&ts_reg_ctrl, 0x0, sizeof(ts_sensor_ctrl));
	memset(&ts_reg_data, 0x0, sizeof(ts_sensor_data));
	memset(&curr_ts_data, 0x0, sizeof(ts_finger_data));

	if (ts->ic_init) {
		/* read device status */
		ret = synaptics_ts_read(ts->client, DEVICE_STATUS_REG,
				sizeof(unsigned char), (u8 *) &ts_reg_ctrl.device_status_reg);
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("DEVICE_STATUS_REG read fail\n");
			goto exit_work;
		}

		/* read interrupt status */
		ret = synaptics_ts_read(ts->client, INTERRUPT_STATUS_REG,
				sizeof(unsigned char), (u8 *) &ts_reg_ctrl.interrupt_status_reg);
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
			goto exit_work;
		} else {
			if (!(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
					&& (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
					&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
					&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME)
					&& (atomic_read(&ts->int_delay.ready) == 1)) {
				ts->int_delay.end = cpu_clock(smp_processor_id());
				ts->int_delay.result_t = ts->int_delay.end -ts->int_delay.start;
				ts->int_delay.rem = do_div(ts->int_delay.result_t , 1000000);
				SYNAPTICS_DEBUG_MSG("Touch IC interrupt line clear time < %3lu.%03lu\n",
						(unsigned long)ts->int_delay.result_t, ts->int_delay.rem/1000);
			}
		}

		/* read button data */
		if (likely(!ts->area_detect_type_button)) {
			ret = synaptics_ts_read(ts->client, BUTTON_DATA_REG,
					sizeof(unsigned char), (u8 *)&ts_reg_ctrl.button_data_reg);
			if (ret < 0) {
				SYNAPTICS_ERR_MSG("BUTTON_DATA_REG read fail\n");
				goto exit_work;
			}
		}

		/* read finger state & finger data register */
		ret = synaptics_ts_read(ts->client, FINGER_STATE_REG,
				/* read until num of finger data */
				sizeof(ts_reg_data) - ((MAX_NUM_OF_FINGER - ts->pdata->num_of_finger) * NUM_OF_EACH_FINGER_DATA_REG),
				(u8 *) &ts_reg_data.finger_state_reg[0]);
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("FINGER_STATE_REG read fail\n");
			goto exit_work;
		}

		/* ESD damage check */
		if ((ts_reg_ctrl.device_status_reg & DEVICE_FAILURE_MASK)== DEVICE_FAILURE_MASK) {
			SYNAPTICS_ERR_MSG("ESD damage occured. Reset Touch IC\n");
			ts->ic_init = 0;
			synaptics_ts_hard_reset(ts);
			return;
		}

		/* Internal reset check */
		if (((ts_reg_ctrl.device_status_reg & DEVICE_STATUS_REG_UNCONFIGURED) >> 7) == 1) {
			SYNAPTICS_ERR_MSG("Touch IC resetted internally. Reconfigure register setting\n");
			ts->ic_init = 0;
			queue_delayed_work(synaptics_wq, &ts->work, 0);
			return;
		}

		/* finger & button interrupt has no correlation */
		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_STATUS)
			SYNAPTICS_DEBUG_MSG("Interrupt status register: 0x%x\n",
					ts_reg_ctrl.interrupt_status_reg);

		if (ts_reg_ctrl.interrupt_status_reg & ts->int_status_reg_asb0_bit) {	/* Finger */
			for(f_counter = 0; f_counter < ts->pdata->num_of_finger; f_counter++) {
				reg_num = f_counter/4;
				finger_order = f_counter%4;

				if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_REG) {
					if (finger_order == 0)
						SYNAPTICS_DEBUG_MSG("Finger status register%d: 0x%x\n",
								reg_num, ts_reg_data.finger_state_reg[reg_num]);
				}

				if (((ts_reg_data.finger_state_reg[reg_num]>>(finger_order*2))
						& FINGER_STATE_MASK) == 1)
				{
					curr_ts_data.pos_x[f_counter] =
							(int)GET_X_POSITION(ts_reg_data.finger_data[f_counter][REG_X_POSITION],
									ts_reg_data.finger_data[f_counter][REG_YX_POSITION]);
					curr_ts_data.pos_y[f_counter] =
							(int)GET_Y_POSITION(ts_reg_data.finger_data[f_counter][REG_Y_POSITION],
									ts_reg_data.finger_data[f_counter][REG_YX_POSITION]);

					if (((ts_reg_data.finger_data[f_counter][REG_WY_WX] & 0xF0) >> 4)
							> (ts_reg_data.finger_data[f_counter][REG_WY_WX] & 0x0F))
						width = (ts_reg_data.finger_data[f_counter][REG_WY_WX] & 0xF0) >> 4;
					else
						width = ts_reg_data.finger_data[f_counter][REG_WY_WX] & 0x0F;

					curr_ts_data.pressure[f_counter] = ts_reg_data.finger_data[f_counter][REG_Z];

					if (ts->area_detect_type_button) {
						/* touch key sensing via finger position:
						 *	finger	- x:0~x_max, y:0~y_max
						 *	key		- x:0~x_max, y:y_max~end
						 */
						if (curr_ts_data.pos_y[f_counter] >= ts->pdata->y_max + BUTTON_MARGIN) {	/* Button */
							button_start = 0;

							/* check for all finger */
							for(b_counter = 0; b_counter < ts->pdata->num_of_button; b_counter++) {
								if (curr_ts_data.pos_x[f_counter] >= button_start
										&& curr_ts_data.pos_x[f_counter] < button_start+ts->button_width) {
									button_pressed[b_counter][f_counter] = TOUCH_PRESSED;
								} else {
									button_pressed[b_counter][f_counter] = TOUCH_RELEASED;
								}
								button_start = button_start + ts->button_width + BUTTON_MARGIN;
							}
						}
					}

					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.pos_x[f_counter]);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.pos_y[f_counter]);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[f_counter]);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width);
					input_report_key(ts->input_dev, BTN_TOUCH, 1);

					report_enable = 1;

					if (ts->finger_prestate[f_counter] == TOUCH_RELEASED) {
						ts->finger_prestate[f_counter] = TOUCH_PRESSED;
						if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_STATUS)
							SYNAPTICS_INFO_MSG("Finger%d pressed\n", f_counter);

						/* Ghost finger defense code
						 * 	store first finger pressed point
						 */
						if (f_counter == 0 && ts->melt_mode == 0
								&& ts_reg_data.finger_state_reg[0] == 1	/* only 1 finger is pressed */
								&& ts_reg_data.finger_state_reg[1] == 0
								&& ts_reg_data.finger_state_reg[2] == 0) {
							ts->pre_ts_data.pos_x[MAX_NUM_OF_FINGER] = curr_ts_data.pos_x[f_counter];
							ts->pre_ts_data.pos_y[MAX_NUM_OF_FINGER] = curr_ts_data.pos_y[f_counter];
						}
					}

					if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_POSITION)
						SYNAPTICS_INFO_MSG("x:%4d, y:%4d, pressure:%4d\n",
								curr_ts_data.pos_x[f_counter], curr_ts_data.pos_y[f_counter],
								curr_ts_data.pressure[f_counter]);
				} else if (ts->finger_prestate[f_counter] == TOUCH_PRESSED) {
					if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_STATUS)
						SYNAPTICS_INFO_MSG("Finger%d released\n", f_counter);

					ts->finger_prestate[f_counter] = TOUCH_RELEASED;

					if (ts->area_detect_type_button) {
						if (ts->pre_ts_data.pos_y[f_counter] >= ts->pdata->y_max + BUTTON_MARGIN) {	/* Button */
							button_start = 0;

							/* check for all finger */
							for(b_counter = 0; b_counter < ts->pdata->num_of_button; b_counter++) {
								if (ts->pre_ts_data.pos_x[f_counter] >= button_start
										&& ts->pre_ts_data.pos_x[f_counter] < button_start+ts->button_width) {
									button_pressed[b_counter][f_counter] = TOUCH_RELEASED;
								}
								button_start = button_start + ts->button_width + BUTTON_MARGIN;
							}
						}
					}

					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->pre_ts_data.pos_x[f_counter]);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->pre_ts_data.pos_y[f_counter]);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TOUCH_RELEASED);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TOUCH_RELEASED);

					input_report_key(ts->input_dev, BTN_TOUCH, 0);

					report_enable = 1;

					if (ts_reg_data.finger_state_reg[0] == 0
							&& ts_reg_data.finger_state_reg[1] == 0
							&& ts_reg_data.finger_state_reg[2] == 0) {
						/* Ghost finger defense code
						 * 	Melt mode should be set not pressed status.
						 * 	We assuem that if user unlock phone,
						 *	it should be dragging some distance using only one finger.
						 */
						if (f_counter == 0 && ts->melt_mode == 0) { /* only 1 finger is pressed */
							if (abs(ts->pre_ts_data.pos_x[f_counter]-ts->pre_ts_data.pos_x[MAX_NUM_OF_FINGER])
										> ts->idle_lock_distance
									|| abs(ts->pre_ts_data.pos_y[f_counter]-ts->pre_ts_data.pos_y[MAX_NUM_OF_FINGER])
										> ts->idle_lock_distance) {
								if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_STATUS)
									SYNAPTICS_INFO_MSG("MELT mode reg. setting: No Melt\n");

								/* no melt mode */
								ret = i2c_smbus_write_byte_data(ts->client,
										MELT_CONTROL_REG, MELT_CONTROL_REG_NO_MELT);
								if (ret < 0) {
									SYNAPTICS_ERR_MSG("Melt mode setting fail\n");
								} else {
									ts->melt_mode = 1;
								}
							} else {
								ts->pre_ts_data.pos_x[MAX_NUM_OF_FINGER] = 0;
								ts->pre_ts_data.pos_y[MAX_NUM_OF_FINGER] = 0;
							}
						}
					}
				}

				if (report_enable)
					input_mt_sync(ts->input_dev);

				ts->pre_ts_data.pos_x[f_counter] = curr_ts_data.pos_x[f_counter];
				ts->pre_ts_data.pos_y[f_counter] = curr_ts_data.pos_y[f_counter];
				ts->pre_ts_data.pressure[f_counter] = curr_ts_data.pressure[f_counter];

				report_enable = 0;
			}

			input_sync(ts->input_dev);

			if (!(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
					&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
					&& (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
					&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME)) {
				if (atomic_read(&ts->int_delay.ready) == 1) {
					ts->int_delay.end = cpu_clock(smp_processor_id());
					ts->int_delay.result_t = ts->int_delay.end - ts->int_delay.start;
					ts->int_delay.rem = do_div(ts->int_delay.result_t , 1000000);
					SYNAPTICS_DEBUG_MSG("Touch Finger data report done time < %3lu.%03lu\n",
							(unsigned long)ts->int_delay.result_t, ts->int_delay.rem/1000);
				}
			}
		}

		if (likely(!ts->area_detect_type_button)) {
			if (ts_reg_ctrl.interrupt_status_reg & ts->int_status_reg_button_bit) { /* Button */
				if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_REG)
					SYNAPTICS_DEBUG_MSG("Button register: 0x%x\n", ts_reg_ctrl.button_data_reg);

				/* Key detection:
				 *	only send a pressed key event if the button was previously unpressed.
				 *	only send a released key event if the button was previously pressed.
				 */
				for(b_counter = 0; b_counter < ts->pdata->num_of_button; b_counter++) {
					if ((((ts_reg_ctrl.button_data_reg >> b_counter) & 0x1) == 1)		/* press interrupt */
							&& (ts->button_prestate[b_counter] == TOUCH_RELEASED)) {
						ts->button_prestate[b_counter] = TOUCH_PRESSED;	/* pressed */
						report_enable = 1;
					} else if ((((ts_reg_ctrl.button_data_reg >> b_counter) & 0x1) == 0)	/* release interrupt */
							&& ts->button_prestate[b_counter] == TOUCH_PRESSED) {
						ts->button_prestate[b_counter] = TOUCH_RELEASED;	/* released */
						report_enable = 1;
					}

					if (report_enable)
						input_report_key(ts->input_dev, ts->pdata->button[b_counter],
								ts->button_prestate[b_counter]);

					if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_STATUS) {
						if (report_enable)
							SYNAPTICS_INFO_MSG("Touch KEY%d(code:%d) is %s\n",
									b_counter,
									ts->pdata->button[b_counter],
									ts->button_prestate[b_counter]?"pressed":"released");
					}
					report_enable = 0;
				}

				input_sync(ts->input_dev);

				if (!(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
						&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
						&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
						&& (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME)) {
					if (atomic_read(&ts->int_delay.ready) == 1) {
						ts->int_delay.end = cpu_clock(smp_processor_id());
						ts->int_delay.result_t = ts->int_delay.end - ts->int_delay.start;
						ts->int_delay.rem = do_div(ts->int_delay.result_t , 1000000);
						SYNAPTICS_DEBUG_MSG("Touch Button data report done time < %3lu.%03lu\n",
								(unsigned long)ts->int_delay.result_t, ts->int_delay.rem/1000);
					}
				}
			}
		} else {
			/* mark pressed button */
			for(b_counter = 0; b_counter < ts->pdata->num_of_button; b_counter++) {
				for(f_counter = 0; f_counter < ts->pdata->num_of_finger; f_counter++) {
					if (button_pressed[b_counter][f_counter] == TOUCH_PRESSED) {
						button_pressed[b_counter][0] = TOUCH_BUTTON_PRESSED;
						break;
					}
				}
			}

			/* find button event */
			for(b_counter = 0; b_counter < ts->pdata->num_of_button; b_counter++) {
				if (button_pressed[b_counter][0] == TOUCH_BUTTON_PRESSED
						&& (ts->button_prestate[b_counter] == TOUCH_RELEASED))	{
					ts->button_prestate[b_counter] = TOUCH_PRESSED;	/* pressed */
					report_enable = 1;
				} else if (button_pressed[b_counter][0] == TOUCH_RELEASED
						&& (ts->button_prestate[b_counter] == TOUCH_PRESSED)) {
					ts->button_prestate[b_counter] = TOUCH_RELEASED;	/* pressed */
					report_enable = 1;
				}

				if (report_enable)
					input_report_key(ts->input_dev, ts->pdata->button[b_counter],
							ts->button_prestate[b_counter]);

				if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_STATUS) {
					if (report_enable)
						SYNAPTICS_INFO_MSG("Touch KEY%d(code:%d) is %s\n",
								b_counter,
								ts->pdata->button[b_counter],
								ts->button_prestate[b_counter]?"pressed":"released");
				}

				report_enable = 0;
				input_sync(ts->input_dev);

				if (!(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
						&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
						&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
						&& (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME)) {
					if (atomic_read(&ts->int_delay.ready) == 1) {
						ts->int_delay.end = cpu_clock(smp_processor_id());
						ts->int_delay.result_t = ts->int_delay.end - ts->int_delay.start;
						ts->int_delay.rem = do_div(ts->int_delay.result_t , 1000000);
						SYNAPTICS_DEBUG_MSG("Touch Button data report done time < %3lu.%03lu\n",
								(unsigned long)ts->int_delay.result_t, ts->int_delay.rem/1000);
					}
				}
			}
		}

exit_work:
		atomic_dec(&ts->interrupt_handled);
		atomic_inc(&ts->int_delay.ready);

		/* Safety code: Check interrupt line status */
		if (ts->pdata->use_irq != 0) {
			if (gpio_get_value(ts->pdata->i2c_int_gpio) != 1
					&& atomic_read(&ts->interrupt_handled) == 0) {
				/* interrupt line to high by Touch IC */
				ret = synaptics_ts_read(ts->client, INTERRUPT_STATUS_REG, 1, &temp);
				if (ret < 0)
					SYNAPTICS_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");

				/* FIXME:
				 * 	We haven't seen this error case.
				 *	So, can not sure it is OK or have to force re-scanning touch IC.
				 */
				SYNAPTICS_ERR_MSG("WARNING - Safety: Interrupt line isn't set high on time cause unexpected incident\n");
			}
		}

		if ((synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
				|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
				|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
				|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME)) {
			/* clear when all event is released */
			if (likely(!ts->area_detect_type_button)) {
				if (ts_reg_data.finger_state_reg[0] == 0
						&& ts_reg_data.finger_state_reg[1] == 0
						&& ts_reg_data.finger_state_reg[2] == 0
						&& ts_reg_ctrl.button_data_reg == 0
						&& atomic_read(&ts->interrupt_handled) == 0)
					atomic_set(&ts->int_delay.ready, 0);
			} else {
				if (ts_reg_data.finger_state_reg[0] == 0
						&& ts_reg_data.finger_state_reg[1] == 0
						&& ts_reg_data.finger_state_reg[2] == 0
						&& atomic_read(&ts->interrupt_handled) == 0)
					atomic_set(&ts->int_delay.ready, 0);
			}
		}
	} else {
		/* Touch IC init */
		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
			SYNAPTICS_DEBUG_MSG("Touch IC init vale setting\n");

		/* check device existence using I2C read */
		if (!ts->is_probed) {
			/* find register map */
			read_page_description_table(ts);

			/* define button & finger interrupt maks */
			if (likely(!ts->area_detect_type_button)) {
				if (ts->finger_dsc.data_base > ts->button_dsc.data_base) {
					ts->int_status_reg_asb0_bit = 0x8;
					ts->int_status_reg_button_bit = 0x4;
				} else {
					ts->int_status_reg_asb0_bit = 0x4;
					ts->int_status_reg_button_bit = 0x8;
				}
			} else {
				ts->int_status_reg_asb0_bit = 0x4;
			}

			ret = synaptics_ts_read(ts->client, MANUFACTURER_ID_REG, 1, &ts->manufcturer_id);
			if (ret < 0)
				SYNAPTICS_ERR_MSG("Manufcturer ID read fail\n");

			ret = synaptics_ts_read(ts->client, FW_REVISION_REG, 1, &ts->fw_rev);
			if (ret < 0)
				SYNAPTICS_ERR_MSG("FW revision read fail\n");

			ret = synaptics_ts_read(ts->client, PRODUCT_ID_REG, 10, &ts->product_id[0]);
			if (ret < 0)
				SYNAPTICS_ERR_MSG("Product ID read fail\n");

#if defined(CONFIG_LGE_TOUCH_SYNAPTICS_FW_UPGRADE)
#if defined(TEST_BOOTING_TIME_FW_FORCE_UPGRADE)
			{
				static char booting_upgrade_test;

				if(booting_upgrade_test == 0) {
					booting_upgrade_test = 1;
					synaptics_ts_fw_upgrade(ts);
					return;
				}

				SYNAPTICS_INFO_MSG("Synaptics %s(%s): Manufacturer ID=%d, FW revision=%d\n",
						ts->product_id, ts->pdata->use_irq?"Interrupt mode":"Polling mode",
						ts->manufcturer_id, ts->fw_rev);
			}
#else	/* TEST_BOOTING_TIME_FW_FORCE_UPGRADE */
			if ((ts->pdata->fw_ver != ts->fw_rev) ||		/* Touch IC FW revision check */
					(ts->finger_dsc.id == 0)) {			/* Integrity check */
				if (likely(ts->pdata->fw_ver == get_fw_image_rev())) {
					synaptics_ts_fw_upgrade(ts);
					return;
				} else if (unlikely(ts->fw_rev != get_fw_image_rev())) {
					SYNAPTICS_INFO_MSG("There is no suitable Touch F/W image file\n");
				}

				SYNAPTICS_INFO_MSG("Synaptics %s(%s): Manufacturer ID=%d, FW revision=%d\n",
						ts->product_id, ts->pdata->use_irq?"Interrupt mode":"Polling mode",
						ts->manufcturer_id, ts->fw_rev);
			}
#endif	/* TEST_BOOTING_TIME_FW_FORCE_UPGRADE */
#endif
			ts->is_probed = 1;
		}

		ret = i2c_smbus_write_byte_data(ts->client, DEVICE_CONTROL_REG,
				(DEVICE_CONTROL_REG_NOSLEEP |DEVICE_CONTROL_REG_CONFIGURED));
		if (ret < 0)
			SYNAPTICS_ERR_MSG("DEVICE_CONTROL_REG write fail\n");

		ret = i2c_smbus_write_byte_data(ts->client, GESTURE_ENABLE_1_REG, 0x00);
		if (ret < 0)
			SYNAPTICS_ERR_MSG("GESTURE_ENABLE_1_REG write fail\n");
		ret = i2c_smbus_write_byte_data(ts->client, GESTURE_ENABLE_2_REG, 0x00);
		if (ret < 0)
			SYNAPTICS_ERR_MSG("GESTURE_ENABLE_2_REG write fail\n");

		/* sensitive setting */
		ret = i2c_smbus_write_byte_data(ts->client, PALM_DETECT_REG, (u8)ts->pdata->palm_threshold);
		if (ret < 0)
			SYNAPTICS_ERR_MSG("PALM_DETECT_REG write fail\n");

		ret = i2c_smbus_write_byte_data(ts->client, DELTA_X_THRESH_REG, (u8)ts->pdata->delta_pos_threshold);
		if (ret < 0)
			SYNAPTICS_ERR_MSG("DELTA_X_THRESH_REG write fail\n");
		ret = i2c_smbus_write_byte_data(ts->client, DELTA_Y_THRESH_REG, (u8)ts->pdata->delta_pos_threshold);
		if (ret < 0)
			SYNAPTICS_ERR_MSG("DELTA_Y_THRESH_REG write fail\n");

		/* interrupt line to high by Touch IC */
		ret = synaptics_ts_read(ts->client, INTERRUPT_STATUS_REG, 1, &temp);
		if (ret < 0)
			SYNAPTICS_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");

		ts->ic_init = 1;
	}
}

/* irq handler for timer debug */
static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	//printk("<== synaptics_ts_irq_handler==>");
	/* ignore irrelevant interrupt during IC power on */
	if (likely(ts->ic_init)) {
		atomic_inc(&ts->interrupt_handled);

		if ((synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
				|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
				|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
				|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME)) {
			/* IRQ priority is always high better than workqueue's.
			 * So, we can trust these data and debug messages.
			 */
			if (atomic_read(&ts->int_delay.ready) == 0)
				atomic_inc(&ts->int_delay.ready);

			/* We can't trust this 'time debug' message completly.
			 * It has some working time cpu_clock().
			 * But, interrupt interval is too short to ignore this delay.
			 */
			if ((synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
					&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
					&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
					&& !(synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME)) {
				ts->int_delay.end = cpu_clock(smp_processor_id());

				if (ts->int_delay.start != 0
						&& ts->int_delay.end != 0
						/* Show 2nd~11th data: 10 data of debug message */
						&& atomic_read(&ts->int_delay.ready) > 1
						&& atomic_read(&ts->int_delay.ready) < 12) {
					ts->int_delay.result_t = ts->int_delay.end - ts->int_delay.start;
					ts->int_delay.rem = do_div(ts->int_delay.result_t , 1000000);
					SYNAPTICS_DEBUG_MSG("Interrupt interval < %3lu.%03lums\n",
							(unsigned long)ts->int_delay.result_t, ts->int_delay.rem/1000);
				}
				ts->int_delay.start = ts->int_delay.end;
			} else {
				/* See only first interrupt handling duration */
				if (atomic_read(&ts->int_delay.ready) == 1
						&& atomic_read(&ts->interrupt_handled) == 1) {
					ts->int_delay.end = cpu_clock(smp_processor_id());
					ts->int_delay.start = ts->int_delay.end;
				}
			}
		}
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t synaptics_ts_thread_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	//printk("synaptics_ts_thread_irq_handler");
	/* ignore irrelevant interrupt during IC power on */
	if (likely(ts->ic_init)) {
		if (ts->pdata->use_irq)
			disable_irq_nosync(ts->client->irq);

		queue_delayed_work(synaptics_wq, &ts->work, 0);

		if (ts->pdata->use_irq)
			enable_irq(ts->client->irq);
	}

	return IRQ_HANDLED;
}

static void synaptics_ts_dev_init(int qpio_num)
{
	//int rc;
	//rc = gpio_request(qpio_num, "touch_int");
	//if (rc < 0) {
	//	SYNAPTICS_ERR_MSG("Can't get synaptics pen down GPIO\n");
	//	return ;
	//}
	//gpio_direction_input(qpio_num);
	//gpio_set_value(qpio_num, 1);
	printk("synaptics_ts_dev_init=%d",qpio_num);
	tegra_gpio_enable(qpio_num);
	mdelay(20);

	
}

//                                                                    
struct synaptics_ts_data *mhl_ts = NULL;
void hdmi_common_send_keyevent(u8 code)
{
	int key = 0;
	if (mhl_ts == NULL)
	{
		printk("error mhl_ts is null!!! (MHL RCP)\n");
		return;
	}

	switch (code)
	{
		case 0x00:
			key = KEY_ENTER;
			break;
		case 0x0D:
			key = KEY_BACK;
			break;			
		case 0x01:
			key = KEY_UP; 
			break;
		case 0x02:
			key = KEY_DOWN; 
			break;
		case 0x03 :
			key = KEY_LEFT; 
			break;
		case 0x04 :
			key = KEY_RIGHT;
			break;
		case 0x20:
			key = KEY_0;
			break;
		case 0x21:
			key = KEY_1;
			break;
		case 0x22:
			key = KEY_2;
			break;
		case 0x23:
			key = KEY_3;
			break;
		case 0x24:
			key = KEY_4;
			break;
		case 0x25:
			key = KEY_5;
			break;
		case 0x26:
			key = KEY_6;
			break;
		case 0x27:
			key = KEY_7;
			break;
		case 0x28:
			key = KEY_8;
			break;
		case 0x29:
			key = KEY_9;
			break;
		case 0x30:
			key = KEY_VOLUMEUP;
			break;
		case 0x31:
			key = KEY_VOLUMEDOWN;
			break;
		case 0x44:
			key = KEY_PLAYPAUSE; 
			break;
		case 0x45:
			key = KEY_STOP; 
			break;
		case 0x46 :
			key = KEY_PLAYPAUSE;
			break;
		case 0x47 :
			key = KEY_RECORD;
			break;
		case 0x48:
			key = KEY_REWIND; 
			break;
		case 0x49:
			key = KEY_FASTFORWARD; 
			break;
		case 0x4b:
			key = KEY_FASTFORWARD;
			break;
		case 0x4c:
			key = KEY_REWIND; 
			break;
		default :
			key = KEY_RESERVED;
			printk("error!! currently scenario is nothing!!! (MHL RCP) code = %d\n", code);			
	}

	input_report_key(mhl_ts->input_dev, key, 1);
	input_sync(mhl_ts->input_dev);
	input_report_key(mhl_ts->input_dev, key, 0);
	input_sync(mhl_ts->input_dev);
	printk("sending MHL RCP  key value o(%d)\n", key);
}
EXPORT_SYMBOL(hdmi_common_send_keyevent);
//                                                                     

static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_platform_data *pdata;
	struct synaptics_ts_data *ts;
	int ret = 0;
	int count = 0;
	char temp;

	if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
		SYNAPTICS_DEBUG_MSG("\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SYNAPTICS_ERR_MSG("i2c functionality check error\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		SYNAPTICS_ERR_MSG("Can not read platform data\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		SYNAPTICS_ERR_MSG("Can not allocate  memory\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	/* Device data setting */
	ts->pdata = pdata;

	ts->button_width = (ts->pdata->x_max - (ts->pdata->num_of_button - 1) * BUTTON_MARGIN) / ts->pdata->num_of_button;
	ts->idle_lock_distance = min(ts->pdata->x_max, ts->pdata->y_max) / 3;

	INIT_DELAYED_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* gpio init...	*/
	if (ts->pdata->gpio_init) {
		ret = ts->pdata->gpio_init();
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("failed to initialized GPIO\n");
			goto err_gpio_init_failed;
		}
	}
	else {
		SYNAPTICS_ERR_MSG("failed to initialized GPIO : "
				"do not have init func..\n");
		ret = -1;
		goto err_gpio_init_failed;
	}
		

	if (ts->pdata->power) {
		/*
		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
			ret = ts->pdata->power(1, true);
		else
			ret = ts->pdata->power(1, false);
		*/
		ret = ts->pdata->power(1, true);
	
		if (ret < 0) {
			SYNAPTICS_ERR_MSG("power on failed\n");
			goto err_power_failed;
		}
	}

	synaptics_ts_dev_init(ts->pdata->i2c_int_gpio);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		SYNAPTICS_ERR_MSG("Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = SYNATICS_DEVICE_NAME;//"synaptics_ts";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	for(count = 0; count < ts->pdata->num_of_button; count++) {
		set_bit(ts->pdata->button[count], ts->input_dev->keybit);
	}
//                                                                    
	set_bit(KEY_0, ts->input_dev->keybit);
	set_bit(KEY_1, ts->input_dev->keybit);
	set_bit(KEY_2, ts->input_dev->keybit);
	set_bit(KEY_3, ts->input_dev->keybit);
	set_bit(KEY_4, ts->input_dev->keybit);
	set_bit(KEY_5, ts->input_dev->keybit);
	set_bit(KEY_6, ts->input_dev->keybit);
	set_bit(KEY_7, ts->input_dev->keybit);
	set_bit(KEY_8, ts->input_dev->keybit);
	set_bit(KEY_9, ts->input_dev->keybit);

	set_bit(KEY_CHANNELUP, ts->input_dev->keybit);
	set_bit(KEY_CHANNELDOWN, ts->input_dev->keybit);
	set_bit(KEY_VOLUMEUP, ts->input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, ts->input_dev->keybit);

	set_bit(KEY_UP, ts->input_dev->keybit);
	set_bit(KEY_DOWN, ts->input_dev->keybit);
	set_bit(KEY_LEFT, ts->input_dev->keybit);
	set_bit(KEY_RIGHT, ts->input_dev->keybit);
	set_bit(KEY_ENTER, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);

	set_bit(KEY_PLAYPAUSE, ts->input_dev->keybit);
	set_bit(KEY_STOP, ts->input_dev->keybit);
	set_bit(KEY_RECORD, ts->input_dev->keybit);
	set_bit(KEY_REWIND, ts->input_dev->keybit);
	set_bit(KEY_FASTFORWARD, ts->input_dev->keybit);
//                                                                      
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata->y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret < 0) {
		SYNAPTICS_ERR_MSG("Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}
//                                                                    
	mhl_ts = ts;
//                                                                      
	/* interrupt mode */
	if (likely(ts->pdata->use_irq && client->irq)) {
		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
			SYNAPTICS_DEBUG_MSG("irq [%d], irqflags[0x%lx]\n",
					client->irq, ts->pdata->irqflags);

		ret = request_threaded_irq(client->irq, synaptics_ts_irq_handler,
				synaptics_ts_thread_irq_handler,
				ts->pdata->irqflags | IRQF_ONESHOT, client->name, ts);

		if (ret < 0) {
			ts->pdata->use_irq = 0;
			SYNAPTICS_ERR_MSG("request_irq failed. use polling mode\n");
		} else {
			if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
				SYNAPTICS_DEBUG_MSG("request_irq succeed\n");
		}
	} else {
		ts->pdata->use_irq = 0;
	}

	/* using hrtimer case of polling mode */
	if (unlikely(!ts->pdata->use_irq)) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer,
				ktime_set(0, (ts->pdata->report_period * 2) + (ts->pdata->ic_booting_delay*1000000)),
				HRTIMER_MODE_REL);
	}

	if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
		SYNAPTICS_DEBUG_MSG("Start touchscreen %s in %s mode\n",
				ts->input_dev->name, ts->pdata->use_irq ? "interrupt" : "polling");

#if defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	/* Register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &synaptics_ts_attribute_group);
	if (ret < 0) {
		SYNAPTICS_ERR_MSG("Synaptics touchscreen sysfs register failed\n");
		goto exit_sysfs_create_group_failed;
	}

	/* Touch I2C sanity check */
	msleep(ts->pdata->ic_booting_delay);
	ret = synaptics_ts_read(ts->client, DESCRIPTION_TABLE_START, 1, &temp);
	if (ret < 0) {
		SYNAPTICS_ERR_MSG("ts_function_descriptor read fail\n");
		goto exit_sysfs_create_group_failed;
	}

	/* Touch IC init setting */
	queue_delayed_work(synaptics_wq, &ts->work, 0);

	return 0;

exit_sysfs_create_group_failed:
	sysfs_remove_group(&client->dev.kobj, &synaptics_ts_attribute_group);
	if (ts->pdata->use_irq)
		free_irq(ts->client->irq, ts);
	unregister_early_suspend(&ts->early_suspend);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	if (ts->pdata->power)
		ts->pdata->power(0, false);
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
err_gpio_init_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);


	sysfs_remove_group(&client->dev.kobj, &synaptics_ts_attribute_group);
	unregister_early_suspend(&ts->early_suspend);

	if (ts->pdata->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);

	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

#if defined(CONFIG_PM)
static int synaptics_ts_suspend(struct device *device)
{
	if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
		SYNAPTICS_DEBUG_MSG ("\n");

	return 0;
}

static int synaptics_ts_resume(struct device *device)
{
	if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
		SYNAPTICS_DEBUG_MSG ("\n");

	return 0;
}
#endif

static void synaptics_ts_suspend_func(struct synaptics_ts_data *ts)
{
	int ret = 0;
	unsigned int f_counter = 0;
	unsigned int b_counter = 0;
	char report_enable = 0;

	if (ts->pdata->use_irq)
		disable_irq_nosync(ts->client->irq);
	else
		hrtimer_cancel(&ts->timer);

	ret = cancel_delayed_work_sync(&ts->work);

	ret = i2c_smbus_write_byte_data(ts->client,
			DEVICE_CONTROL_REG, DEVICE_CONTROL_REG_SLEEP); /* sleep */

	/* Ghost finger & missed release event defense code
	 * 	Release report if we have not released event until suspend
	 */

	/* Finger check */
	for(f_counter = 0; f_counter < ts->pdata->num_of_finger; f_counter++) {
		if (ts->finger_prestate[f_counter] == TOUCH_PRESSED) {
			input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, ts->pre_ts_data.pos_x[f_counter]);
			input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, ts->pre_ts_data.pos_y[f_counter]);
			input_report_abs(ts->input_dev,
					ABS_MT_TOUCH_MAJOR, TOUCH_RELEASED);
			input_report_abs(ts->input_dev,
					ABS_MT_WIDTH_MAJOR, TOUCH_RELEASED);

			report_enable = 1;
		}
	}

	/* Button check */
	for(b_counter = 0; b_counter < ts->pdata->num_of_button; b_counter++) {
		if (ts->button_prestate[b_counter] == TOUCH_PRESSED) {
			report_enable = 1;
			input_report_key(ts->input_dev,
					ts->pdata->button[b_counter], TOUCH_RELEASED);
		}
	}

	/* Reset finger position data */
	memset(&ts->pre_ts_data, 0x0, sizeof(ts_finger_data));

	if (report_enable) {
		SYNAPTICS_INFO_MSG("Release all pressed event before touch power off\n");
		input_sync(ts->input_dev);

		/* Reset finger & button status data */
		memset(ts->finger_prestate, 0x0, sizeof(char) * ts->pdata->num_of_finger);
		memset(ts->button_prestate, 0x0, sizeof(char) * ts->pdata->num_of_button);
	}

	/* Reset interrupt debug time struct */
	if ((synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_INTERVAL)
			|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_INT_ISR_DELAY)
			|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FINGER_HANDLE_TIME)
			|| (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_BUTTON_HANDLE_TIME))
		memset(&ts->int_delay, 0x0, sizeof(struct synaptics_ts_timestamp));

	if (ts->pdata->power) {
		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
			ret = ts->pdata->power(0, true);
		else
			ret = ts->pdata->power(0, false);

		if (ret < 0) {
			SYNAPTICS_ERR_MSG("power off failed\n");
		} else {
			ts->ic_init = 0;
			ts->melt_mode = 0;
			atomic_set(&ts->interrupt_handled, 0);
		}
	}
}

static void synaptics_ts_resume_func(struct synaptics_ts_data *ts)
{
	int ret = 0;

	if (ts->pdata->power) {
		if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
			ret = ts->pdata->power(1, true);
		else
			ret = ts->pdata->power(1, false);

		if (ret < 0)
			SYNAPTICS_ERR_MSG("power on failed\n");
	}

	queue_delayed_work(synaptics_wq,
			&ts->work,msecs_to_jiffies(ts->pdata->ic_booting_delay));

	if (ts->pdata->use_irq)
		enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer,
				ktime_set(0, ts->pdata->report_period+(ts->pdata->ic_booting_delay*1000000)),
				HRTIMER_MODE_REL);
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts =
			container_of(h, struct synaptics_ts_data, early_suspend);

	if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
		SYNAPTICS_DEBUG_MSG("\n");

	if (likely(!ts->is_downloading))
		synaptics_ts_suspend_func(ts);

	ts->is_suspended = 1;
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts =
			container_of(h, struct synaptics_ts_data, early_suspend);

	if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
		SYNAPTICS_DEBUG_MSG ("\n");

	if (likely(!ts->is_downloading))
		synaptics_ts_resume_func(ts);

	ts->is_suspended = 0;
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{SYNATICS_DEVICE_NAME, 0},
	{},
};

#if defined(CONFIG_PM)
static struct dev_pm_ops synaptics_ts_pm_ops = {
	.suspend 	= synaptics_ts_suspend,
	.resume 	= synaptics_ts_resume,
};
#endif

static struct i2c_driver synaptics_ts_driver = {
	.probe	= synaptics_ts_probe,
	.remove	= synaptics_ts_remove,
	.id_table	= synaptics_ts_id,
	.driver 	= {
		.name	= SYNATICS_DEVICE_NAME,//"synaptics_ts",
		.owner 	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm		= &synaptics_ts_pm_ops,
#endif
	},
};

static int __devinit synaptics_ts_init(void)
{
	int ret = 0;

	if (synaptics_rmi4_i2c_debug_mask & SYNAPTICS_RMI4_I2C_DEBUG_FUNC_TRACE)
		SYNAPTICS_DEBUG_MSG("\n");

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq) {
		SYNAPTICS_ERR_MSG("failed to create singlethread workqueue\n");
		return -ENOMEM;
	}

	ret = i2c_add_driver(&synaptics_ts_driver);;
	if (ret < 0) {
		SYNAPTICS_ERR_MSG("failed to i2c_add_driver\n");
		destroy_workqueue(synaptics_wq);
	}

	return ret;
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);

	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_AUTHOR("Hyesung Shin <hyesung.shin@lge.com>");
MODULE_DESCRIPTION("Synaptics RMI4 protocol based I2C interface Touchscreen Driver");
MODULE_LICENSE("GPL");
