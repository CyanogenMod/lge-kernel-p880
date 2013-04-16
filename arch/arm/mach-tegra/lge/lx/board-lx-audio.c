/*
 * arch/arm/mach-tegra/board-lx-audio.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <mach/audio.h>
#include <sound/max98088.h>
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>


#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-lx.h>
#include <lge/board-lx-audio.h>

#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

//                                         
#include <sound/pcm.h>
//                                         

//                                  
#if defined(CONFIG_SND_SOC_WM8994)
#include <linux/mfd/wm8994/pdata.h>
#elif defined(CONFIG_SND_SOC_TEGRA_MAX98088)
#include <sound/max98088.h>
#endif 
#include <linux/switch.h>
//                                

/* Equalizer filter coefs generated from the MAXIM MAX98088
 * evkit software tool */
//                                  
#if defined(CONFIG_SND_SOC_WM8994)
static struct wm8994_drc_cfg wm8994_drc_data[] = {
	{
		.name = "AIF1DRC1 Mode",
		.regs = {0x198, 0x84A, 0x818, 0x3EE, 0x1A9},
	},
	{
		.name = "AIF2DRC Mode",
		.regs = {0x198, 0x84E, 0x818, 0x265, 0x187},
	},
};

struct wm8994_pdata wm8994_data = {
	.num_drc_cfgs = 2,
	.drc_cfgs  = &wm8994_drc_data,
};

#elif defined(CONFIG_SND_SOC_TEGRA_MAX98088)
static struct max98088_eq_cfg max98088_eq_cfg[] = {
        {
                .name = "FLAT",
                .rate = 44100,
                .band1 = {0x2000, 0xC002, 0x4000, 0x00E9, 0x0000},
                .band2 = {0x2000, 0xC00F, 0x4000, 0x02BC, 0x0000},
                .band3 = {0x2000, 0xC0A7, 0x4000, 0x0916, 0x0000},
                .band4 = {0x2000, 0xC5C2, 0x4000, 0x1A87, 0x0000},
                .band5 = {0x2000, 0xF6B0, 0x4000, 0x3F51, 0x0000},
        },
        {
                .name = "LOWPASS1K",
                .rate = 44100,
                .band1 = {0x205D, 0xC001, 0x3FEF, 0x002E, 0x02E0},
                .band2 = {0x5B9A, 0xC093, 0x3AB2, 0x088B, 0x1981},
                .band3 = {0x0D22, 0xC170, 0x26EA, 0x0D79, 0x32CF},
                .band4 = {0x0894, 0xC612, 0x01B3, 0x1B34, 0x3FFA},
                .band5 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
        },
        { /* BASS=-12dB, TREBLE=+9dB, Fc=5KHz */
                .name = "HIBOOST",
                .rate = 44100,
                .band1 = {0x0815, 0xC001, 0x3AA4, 0x0003, 0x19A2},
                .band2 = {0x0815, 0xC103, 0x092F, 0x0B55, 0x3F56},
                .band3 = {0x0E0A, 0xC306, 0x1E5C, 0x136E, 0x3856},
                .band4 = {0x2459, 0xF665, 0x0CAA, 0x3F46, 0x3EBB},
                .band5 = {0x5BBB, 0x3FFF, 0xCEB0, 0x0000, 0x28CA},
        },
        { /* BASS=12dB, TREBLE=+12dB */
                .name = "LOUD12DB",
                .rate = 44100,
                .band1 = {0x7FC1, 0xC001, 0x3EE8, 0x0020, 0x0BC7},
                .band2 = {0x51E9, 0xC016, 0x3C7C, 0x033F, 0x14E9},
                .band3 = {0x1745, 0xC12C, 0x1680, 0x0C2F, 0x3BE9},
                .band4 = {0x4536, 0xD7E2, 0x0ED4, 0x31DD, 0x3E42},
                .band5 = {0x7FEF, 0x3FFF, 0x0BAB, 0x0000, 0x3EED},
        },
        {
                .name = "FLAT",
                .rate = 16000,
                .band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
                .band2 = {0x2000, 0xC033, 0x4000, 0x0505, 0x0000},
                .band3 = {0x2000, 0xC268, 0x4000, 0x115F, 0x0000},
                .band4 = {0x2000, 0xDA62, 0x4000, 0x33C6, 0x0000},
                .band5 = {0x2000, 0x4000, 0x4000, 0x0000, 0x0000},
        },
        {
                .name = "LOWPASS1K",
                .rate = 16000,
                .band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
                .band2 = {0x5BE8, 0xC3E0, 0x3307, 0x15ED, 0x26A0},
                .band3 = {0x0F71, 0xD15A, 0x08B3, 0x2BD0, 0x3F67},
                .band4 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
                .band5 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
        },
        { /* BASS=-12dB, TREBLE=+9dB, Fc=2KHz */
                .name = "HIBOOST",
                .rate = 16000,
                .band1 = {0x0815, 0xC001, 0x3BD2, 0x0009, 0x16BF},
                .band2 = {0x080E, 0xC17E, 0xF653, 0x0DBD, 0x3F43},
                .band3 = {0x0F80, 0xDF45, 0xEE33, 0x36FE, 0x3D79},
                .band4 = {0x590B, 0x3FF0, 0xE882, 0x02BD, 0x3B87},
                .band5 = {0x4C87, 0xF3D0, 0x063F, 0x3ED4, 0x3FB1},
        },
        { /* BASS=12dB, TREBLE=+12dB */
                .name = "LOUD12DB",
                .rate = 16000,
                .band1 = {0x7FC1, 0xC001, 0x3D07, 0x0058, 0x1344},
                .band2 = {0x2DA6, 0xC013, 0x3CF1, 0x02FF, 0x138B},
                .band3 = {0x18F1, 0xC08E, 0x244D, 0x0863, 0x34B5},
                .band4 = {0x2BE0, 0xF385, 0x04FD, 0x3EC5, 0x3FCE},
                .band5 = {0x7FEF, 0x4000, 0x0BAB, 0x0000, 0x3EED},
        },
//                                          
		{
				.name = "FACTORYSPEC",
				.rate = 48000,
				.band1 = {0x0809, 0xC001, 0x3E56, 0x000C, 0x0E7B},
				.band2 = {0x0821, 0xC001, 0x3ED1, 0x000C, 0x0C40},
				.band3 = {0x0821, 0xC001, 0x3E1E, 0x0019, 0x0F65},
				.band4 = {0x0839, 0xC001, 0x3EB4, 0x0019, 0x0CCF},
				.band5 = {0x0821, 0xC001, 0x3E7A, 0x0019, 0x0DDF},
		},
		{
				.name = "FACTORYSPEC_HP",
				.rate = 48000,
				.band1 = {0x0809, 0xC001, 0x3FCB, 0x000C, 0x0518},
				.band2 = {0x0809, 0xC001, 0x3FCB, 0x000C, 0x0518},
				.band3 = {0x2000, 0xC322, 0x4000, 0x13C6, 0x0000},
				.band4 = {0x2000, 0xE783, 0x4000, 0x3B20, 0x0000},
				.band5 = {0x2000, 0x4000, 0x4000, 0x0000, 0x0000},
		},
//                                          
};

struct max98088_pdata max98088_pdata = {
        /* equalizer configuration */
        .eq_cfg = max98088_eq_cfg,
        .eq_cfgcnt = ARRAY_SIZE(max98088_eq_cfg),

	/* debounce time */
	.debounce_time_ms = 200,

        /* microphone configuration */
        .digmic_left_mode = 0,  /* 0 = normal analog mic */
        .digmic_right_mode = 0, /* 0 = normal analog mic */

        /* receiver output configuration */
        .receiver_mode = 0,     /* 0 = amplifier, 1 = line output */
};
#endif
//                                             
static struct tegra_asoc_platform_data lx_audio_pdata = {
	.name				= "h2w",
	.gpio_hook  		= TEGRA_GPIO_HP_HOOK,
	.gpio_ear_mic		= TEGRA_GPIO_EAR_MIC,
	.gpio_spkr_en		= -1,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
//                                         
	.gpio_int_mic_en	= TEGRA_GPIO_SUB_MIC,
//                                         
	.gpio_ext_mic_en	= -1,
	.audio_port_id		= {
		[HIFI_CODEC] = 0,
		[BASEBAND] = 2,
		[BT_SCO] = 3,
	},
	.baseband_param		= {
		.rate = 16000,
//                                                          
#if defined(CONFIG_MACH_PEGASUS)
		.channels = 2,
#else
		.channels = 1,
#endif
//                                                          
	},
//                                         
	.hifi_param 	= {
		.i2s_num = 0,
		.rate = SNDRV_PCM_RATE_48000,
		.channels = 2,
	},
	.bt_param		= {
		.i2s_num = 3,
		.rate = SNDRV_PCM_RATE_8000,
		.channels = 1,
	},
//                                         
};

struct gpio_switch_platform_data lx_headset_data = {
	.name = "h2w",
#if 0 // don't control GPIO_PO4
	.gpio = TEGRA_GPIO_PO7,
#else
	.gpio = TEGRA_GPIO_PO4,
#endif
};

struct platform_device tegra_headset_detect_device =
{
	.name = "lx_headset",
	.id	= -1,
	.dev.platform_data = &lx_headset_data,
};

struct platform_device lx_audio_device = {
	.name	= "tegra-snd-max98088",
	.id	= 0,
	.dev	= {
		.platform_data  = &lx_audio_pdata,
	},
};
//                                             
