/*
 *  Copyright (C) 2011 LG Electronics
 *  Evan Kim <euikyoem.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __TSPDRV_I2C_H_
#define __TSPDRV_I2C_H_

#define TSPDRV_I2C_SLAVE_ADDR 0x49
#define TSPDRV_I2C_DEVICE_NAME	"tspdrv_i2c"

struct tspdrv_i2c_platform_data {
	int     max_timeout;
	u8      active_low;
	int     initial_vibrate;
	int		pwm_id;
	int		duty_ns;
	int		period_ns;
	int	en_gpio;
};

extern s32 tspdrv_i2c_read_block_data(u8 command, u8 length, u8 *values);
extern s32 tspdrv_i2c_read_byte_data(u8 command);
extern s32 tspdrv_i2c_write_block_data(u8 command, u8 length, const u8 *values);
extern s32 tspdrv_i2c_write_byte_data(u8 command, u8 value);

extern void tspdrv_control_vibrator(u8 onoff);
extern void tspdrv_control_pwm(u8 onoff, int duty_ns, int period_ns);
#endif
