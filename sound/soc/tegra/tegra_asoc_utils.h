/*
 * tegra_asoc_utils.h - Definitions for Tegra DAS driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2010-12, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __TEGRA_ASOC_UTILS_H__
#define __TEGRA_ASOC_UTILS_H_


#define TEGRA_ALSA_MAX_DEVICES 6
#define TEGRA_DMA_MAX_CHANNELS 32

struct clk;
struct device;

struct tegra_asoc_utils_data {
	struct device *dev;
	struct snd_soc_card *card;
	struct clk *clk_pll_a;
	struct clk *clk_pll_a_out0;
	struct clk *clk_cdev1;
	struct clk *clk_out1;
	struct clk *clk_m;
	struct clk *clk_pll_p_out1;
	int set_baseclock;
	int set_mclk;
	int lock_count;
	int avp_device_id;
	unsigned int avp_dma_addr;
};

int tegra_asoc_utils_set_rate(struct tegra_asoc_utils_data *data, int srate,
				int mclk);
void tegra_asoc_utils_lock_clk_rate(struct tegra_asoc_utils_data *data,
					int lock);
int tegra_asoc_utils_init(struct tegra_asoc_utils_data *data,
				struct device *dev, struct snd_soc_card *card);
int tegra_asoc_utils_set_parent(struct tegra_asoc_utils_data *data,
				int is_i2s_master);
void tegra_asoc_utils_fini(struct tegra_asoc_utils_data *data);
int tegra_asoc_utils_clk_enable(struct tegra_asoc_utils_data *data);
int tegra_asoc_utils_clk_disable(struct tegra_asoc_utils_data *data);
int tegra_asoc_utils_register_ctls(struct tegra_asoc_utils_data *data);

#endif

