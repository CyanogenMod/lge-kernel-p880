/*
 * arch/arm/mach-tegra/include/mach/tegra_asoc_pdata.h
 *
 * Copyright (c) 2012, NVIDIA CORPORATION. All rights reserved.
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

#define	HIFI_CODEC		0
#define	BASEBAND		1
#define	BT_SCO			2
#define VOICE_CODEC		3
#define	NUM_I2S_DEVICES		4

#define	TEGRA_DAIFMT_DSP_A		0
#define	TEGRA_DAIFMT_DSP_B		1
#define	TEGRA_DAIFMT_I2S		2
#define	TEGRA_DAIFMT_RIGHT_J		3
#define	TEGRA_DAIFMT_LEFT_J		4

struct i2s_config {
	int audio_port_id;
	int is_i2s_master;
	int i2s_mode;
	int sample_size;
	int rate;
	int channels;
	int bit_clk;
};

//                                         
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
struct hifi_config {
	int i2s_num;
	int rate;
	int channels;
};

struct bt_config {
	int i2s_num;
	int rate;
	int channels;
};
#endif
//                                         

struct tegra_asoc_platform_data {
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
	const char *name;	//                                             
	int gpio_hook;		//                                             
	int gpio_ear_mic;	//                                              
#endif
	const char *codec_name;
	const char *codec_dai_name;
	int gpio_spkr_en;
	int gpio_hp_det;
	int gpio_hp_mute;
	int gpio_int_mic_en;
	int gpio_ext_mic_en;
	unsigned int debounce_time_hp;
	//                                         
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
	struct hifi_config hifi_param;
	struct bt_config bt_param;
#endif
	//                                         
	struct i2s_config i2s_param[NUM_I2S_DEVICES];
};
