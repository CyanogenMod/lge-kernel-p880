/*
 * Xilinx SPI device driver API and platform data header file
 *
 * Copyright (c) 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _SPI_RGB_BRIDGE_H_
#define _SPI_RGB_BRIDGE_H_

#include <linux/spi/spi.h>

#define STR_RGB_BRIDGE	"rgb_bridge_spi"

struct bridge_platform_data{
	u16 lcd_en;
#if defined(CONFIG_MACH_VU10)
	u16 lcd_en_3v;
#endif
	u16 bridge_en;
	u16 lcd_reset_n;
	u16 bridge_reset_n;
	u8 mode;
	u8 bits_per_word;
	u32 max_speed_hz;
};

int ssd2825_bridge_enable(void);
int ssd2825_bridge_disable(void);

#endif // _SPI_RGB_BRIDGE_H_
