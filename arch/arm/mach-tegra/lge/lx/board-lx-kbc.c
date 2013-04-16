/*
 * arch/arm/mach-tegra/board-lx-kbc.c
 * Keys configuration for Nvidia tegra3 lx platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include <linux/gpio_keys.h>
#include <linux/dma-mapping.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/board.h>
#include <lge/board-lx.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/wakeups-t3.h>

#if !defined(CONFIG_MACH_LX)
#define LX_ROW_COUNT	2 
#define LX_COL_COUNT	1 

static const u32 kbd_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
};

static const struct matrix_keymap_data keymap_data = {
	.keymap		= kbd_keymap,
	.keymap_size	= ARRAY_SIZE(kbd_keymap),
};

static struct tegra_kbc_wake_key lx_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
	[1] = {
		.row = 1,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data lx_kbc_platform_data = {
	.debounce_cnt = 20 * 32, /* 20 ms debaunce time */
	.repeat_cnt = 1,
	.scan_count = 30,
	.wakeup = true,
	.keymap_data = &keymap_data,
	.wake_cnt = 4,
	.wake_cfg = &lx_wake_cfg[0],
};

#else
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 1,	\
	}

#define PMC_WAKE_STATUS 0x14
#define PMC_WAKE2_STATUS	0x168

static int lx_wakeup_key(void)
{
	u64 status;
	
	status = __raw_readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
	status |= ((u64)readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE2_STATUS)) << 32;

	printk("lx_wakeup_key : status %lu\n", status);

#if 0	
	if (status & TEGRA_WAKE_GPIO_PC7)
		return KEY_POWER;
	else if (status & TEGRA_WAKE_GPIO_PO4) 
		return KEY_VOLUMEDOWN;
	else if (status & TEGRA_WAKE_GPIO_PR3)
		return KEY_HP;
	else
		return KEY_RESERVED;
#else	
	return (status & TEGRA_WAKE_GPIO_PC7) ? KEY_POWER : (status & TEGRA_WAKE_GPIO_PO4) ? KEY_VOLUMEDOWN : KEY_RESERVED;
#endif
}


static struct gpio_keys_button lx_keys[] = {
#if defined(CONFIG_PMIC_POWERKEY)
	[0] = GPIO_KEY(KEY_VOLUMEUP, PO7, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PO4, 1),	
	[2] = GPIO_KEY(KEY_HIGHLIGHER,PR3,0), //YJChae 20120615 add for quickmemo key
#else
	[0] = GPIO_KEY(KEY_VOLUMEUP, PO7, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PO4, 1),	
	[2] = GPIO_KEY(KEY_POWER, PC7, 1),
	[3] = GPIO_KEY(KEY_HIGHLIGHER,PR3,1), //YJChae 20120615 add for quickmemo key wakeup
#endif
};

static struct gpio_keys_platform_data lx_keys_platform_data = {
	.buttons	= lx_keys,
	.nbuttons	= ARRAY_SIZE(lx_keys),	
	.rep		= 0,
	.wakeup_key  = lx_wakeup_key,
};

static struct platform_device lx_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &lx_keys_platform_data,
	},
};
#endif

int __init lx_kbc_init(void)
{
#if defined(CONFIG_MACH_LX)
	int i;	
	int ret = 0;
 
	pr_info("Registering tegra-kbc-gpio\n");

	for (i = 0; i < ARRAY_SIZE(lx_keys); i++)
		tegra_gpio_enable(lx_keys[i].gpio);	

	platform_device_register(&lx_keys_device);
		
#else
	struct tegra_kbc_platform_data *data = &lx_kbc_platform_data;
	int i;
	tegra_kbc_device.dev.platform_data = &lx_kbc_platform_data;
	pr_info("Registering tegra-kbc\n");

	BUG_ON((KBC_MAX_ROW + KBC_MAX_COL) > KBC_MAX_GPIO);
	for (i = 0; i < LX_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].en = true;
	}
	for (i = 0; i < LX_COL_COUNT; i++) {
		data->pin_cfg[i + KBC_MAX_ROW].num = i;
		data->pin_cfg[i + KBC_MAX_ROW].en = true;
	}
	platform_device_register(&tegra_kbc_device);
#endif	
	pr_info("Registering successful tegra-kbc\n");
	return 0;
}

