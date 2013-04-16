/*
 * arch/arm/mach-tegra/board-x3.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <mach-tegra/board.h>
#include <lge/board-x3.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

#ifndef __MACH_TEGRA_X3_GPS_H
#define __MACH_TEGRA_X3_GPS_H

struct gps_gpio_platform_data {
	unsigned pwron;		/* PWR_ON GPIO */
	unsigned reset_n;	/* RESET_N GPIO */
//                                                        
	unsigned eclk;      /* 26MHz_GPS_REF_EN GPIO */
//                                                        
#if defined(CONFIG_P940_GPS_LNA_SD_USE)
	unsigned lna_sd;	/* LNA_SD GPIO */
#endif
};

#endif /* __MACH_TEGRA_X3_GPS_H */

extern void x3_gps_init(void);