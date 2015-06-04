/*
 * arch/arm/mach-tegra/tegra_pmqos.h
 *
 * Copyright (C) 2012 Paul Reioux (aka Faux123)
 *
 * Author:
 *	faux123 <reioux@gmail.com>
 *
 * History:
 *      -original version (Paul Reioux)
 *      -cleaned since oc was reworked (Dennis Rassmann)
 *      -added comment for T3_VARIANT_BOOST (Dennis Rassmann)
 *      -adapted for grouper (Dennis Rassmann)
 *      -removed distinction between 0boost and xboost
 *      -minimized version
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

/* in kHz */
#define BOOST_CPU_FREQ_MIN	1300000
#define CAP_CPU_FREQ_MAX	1700000
#define T3_CPU_MIN_FREQ     51000

#define MIN_CPU_MV 725
#define MAX_CPU_MV 1300
#define MIN_CORE_MV 900
#define MAX_CORE_MV 1400

extern unsigned int tegra_pmqos_boost_freq;
extern unsigned int tegra_pmqos_cap_freq;
extern unsigned int tegra_pmqos_cpu_freq_limits[];
extern unsigned int miss_freq_set;
