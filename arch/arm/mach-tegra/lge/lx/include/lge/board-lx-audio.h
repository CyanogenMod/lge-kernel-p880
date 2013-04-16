/*
 * The header file for LX AUDIO
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MACH_TEGRA_LX_AUDIO_H
#define __MACH_TEGRA_LX_AUDIO_H

#if defined(CONFIG_SND_SOC_WM8994)
extern struct wm8994_pdata wm8994_data;
#elif defined(CONFIG_SND_SOC_TEGRA_MAX98088)
extern struct max98088_pdata max98088_pdata;
#endif
extern struct platform_device lx_audio_device;
extern struct platform_device tegra_headset_detect_device;

#endif /* __MACH_TEGRA_LX_AUDIO_H */
