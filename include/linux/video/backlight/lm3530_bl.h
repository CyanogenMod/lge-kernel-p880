/* 
 * Copyright (C) 2011 LG Electronics, Inc
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

#ifndef __LM3530_BL_H__
#define __LM3530_BL_H__

struct lm3530_bl_platform_data {
	u32	hwen_gpio;
	u32	max_current;	
	u32	min_brightness;
};

#endif // __LM3530_BL_H__
