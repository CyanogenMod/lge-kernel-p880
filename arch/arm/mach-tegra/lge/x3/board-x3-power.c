/*
 * arch/arm/mach-tegra/board-x3-power.c
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
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/mfd/tps80031.h>
#include <linux/regulator/tps80031-regulator.h>
#include <linux/mfd/aat2870.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/platform_data/tegra_bpc_mgmt.h>
#include <linux/kmsg_dump.h>


#include <mach/edp.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <linux/tps80031-charger.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/board.h>
#include <lge/board-x3.h>
#include <mach-tegra/pm.h>
#include <mach-tegra/wakeups-t3.h>

#if defined(CONFIG_PMIC_POWERKEY)
/* Fucking Power Key... */
#include <linux/interrupt.h>
#include <linux/input.h>

static struct input_dev *max77663_pwrkey; // Fucking Power Key
#endif

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

//#define PMC_DPD_PADS_ORIDE		0x01c
//#define PMC_DPD_PADS_ORIDE_BLINK	(1 << 20)

/************************ TPS80031 based regulator ****************/
#if defined(CONFIG_REGULATOR_TPS80031)

static struct regulator_consumer_supply tps80031_vio_supply[] = {
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc4", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_sd_slot", NULL),

};

static struct regulator_consumer_supply tps80031_smps1_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply tps80031_smps2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply tps80031_smps3_supply[] = {
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
};

static struct regulator_consumer_supply tps80031_smps4_supply[] = {
//	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
//	REGULATOR_SUPPLY("vddio_sd_slot", NULL),
};

static struct regulator_consumer_supply tps80031_vana_supply[] = {
//	REGULATOR_SUPPLY("unused_vana", NULL),
};

static struct regulator_consumer_supply tps80031_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
};

static struct regulator_consumer_supply tps80031_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps80031_ldo3_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_vthrm", NULL),
};

static struct regulator_consumer_supply tps80031_ldo4_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
};

static struct regulator_consumer_supply tps80031_ldo5_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

static struct regulator_consumer_supply tps80031_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply tps80031_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("vdd_pllm", NULL),
	REGULATOR_SUPPLY("vdd_pllu_d", NULL),
	REGULATOR_SUPPLY("vdd_pllx", NULL),
};

static struct regulator_consumer_supply tps80031_ldoln_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply tps80031_ldousb_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", NULL),
};

static struct regulator_consumer_supply tps80031_battery_charge_supply[] = {
	REGULATOR_SUPPLY("usb_bat_chg", NULL),
};

#define TPS_PDATA_INIT(_id, _minmv, _maxmv, _supply_reg, _always_on,	\
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply, 	\
	_flags, _ectrl, _delay)						\
	static struct tps80031_regulator_platform_data pdata_##_id = {	\
		.regulator = {						\
			.constraints = {				\
				.min_uV = (_minmv)*1000,		\
				.max_uV = (_maxmv)*1000,		\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						REGULATOR_MODE_STANDBY),      \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						REGULATOR_CHANGE_STATUS |     \
						REGULATOR_CHANGE_VOLTAGE),    \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uv,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(tps80031_##_id##_supply),	\
			.consumer_supplies = tps80031_##_id##_supply,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_uV =  _init_uV * 1000,				\
		.init_enable = _init_enable,				\
		.init_apply = _init_apply,				\
		.flags = _flags,					\
		.delay_us = _delay,					\
	}

TPS_PDATA_INIT(vio,   600, 2100, 0, 1, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(smps1, 600, 2100, 0, 0, 0, 0, -1, 0, 0, PWR_REQ_INPUT_PREQ2, 0);
TPS_PDATA_INIT(smps2, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(smps3, 600, 2100, 0, 1, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(smps4, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo1, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo2, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo3, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo4, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo5, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo6, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo7, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldoln, 1000, 3300, tps80031_rails(SMPS3), 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldousb, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, USBLDO_INPUT_VSYS, 0);
TPS_PDATA_INIT(vana,  1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0);
//TPS_PDATA_INIT(vbus,  0, 5000, 0, 0, 0, 0, -1, 0, 0, (VBUS_SW_ONLY | VBUS_DISCHRG_EN_PDN), 100000);

static struct tps80031_rtc_platform_data rtc_data = {
	.irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_RTC_ALARM,
	.time = {
		.tm_year = 2011,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 1,
		.tm_min = 2,
		.tm_sec = 3,
	},
};

int battery_charger_init(void *board_data)
{
	int ret;
	ret = gpio_request(TEGRA_GPIO_PF6, "lcd_d14-bat_charge");
	if (ret < 0) {
		pr_err("%s() The gpio_request for battery"
				" charger fails\n", __func__);
	}
	gpio_direction_output(TEGRA_GPIO_PF6, 1);
	tegra_gpio_enable(TEGRA_GPIO_PF6);
	return 0;
}

static struct tps80031_charger_platform_data bcharger_pdata = {
	.max_charge_volt_mV = 4100,
	.max_charge_current_mA = 1000,
	.charging_term_current_mA = 100,
	.watch_time_sec = 100,
	.irq_base = ENT_TPS80031_IRQ_BASE,
	.consumer_supplies = tps80031_battery_charge_supply,
	.num_consumer_supplies = ARRAY_SIZE(tps80031_battery_charge_supply),
	.board_init = battery_charger_init,
	.board_data = NULL,
};

static struct tps80031_bg_platform_data battery_gauge_data = {
	.irq_base = ENT_TPS80031_IRQ_BASE,
	.battery_present = 1,
};

#define TPS_RTC()					\
	{						\
		.id	= 0,				\
		.name	= "rtc_tps80031",		\
		.platform_data = &rtc_data,		\
	}

#define TPS_REG(_id, _data)				\
	{						\
		.id	 = TPS80031_ID_##_id,		\
		.name   = "tps80031-regulator",		\
		.platform_data  = &pdata_##_data,	\
	}

#define TPS_BATTERY()					\
	{						\
		.name   = "tps80031-charger",		\
		.platform_data = &bcharger_pdata,	\
	}
#define TPS_BATTERY_GAUGE()				\
	{						\
		.name   = "tps80031-battery-gauge",	\
		.platform_data = &battery_gauge_data,	\
	}

static struct tps80031_subdev_info tps80031_devs[] = {
	TPS_REG(VIO, vio),
	TPS_REG(SMPS1, smps1),
	TPS_REG(SMPS2, smps2),
	TPS_REG(SMPS3, smps3),
//                                                                    
	TPS_REG(LDO1, ldo1),
	TPS_REG(LDO2, ldo2),
	TPS_REG(LDO3, ldo3),
	TPS_REG(LDO4, ldo4),
	TPS_REG(LDO5, ldo5),
	TPS_REG(LDO6, ldo6),
	TPS_REG(LDO7, ldo7),
	TPS_REG(LDOLN, ldoln),
	TPS_REG(LDOUSB, ldousb),
//                                                                   
//                                                  
	TPS_RTC(),
//                                              
//                                                  
};

struct tps80031_32kclock_plat_data clk32k_pdata = {
	.en_clk32kg = 1,
	.en_clk32kaudio = 1,
};
static struct tps80031_platform_data tps_platform = {
	.num_subdevs	= ARRAY_SIZE(tps80031_devs),
	.subdevs	= tps80031_devs,
	.irq_base	= ENT_TPS80031_IRQ_BASE,
	.gpio_base	= ENT_TPS80031_GPIO_BASE,
	.clk32k_pdata	= &clk32k_pdata,
};
#endif // CONFIG_REGULATOR_TPS80031
/************************ TPS80031 based regulator ****************/


/************************ MAX77663 based regulator ****************/
#if defined(CONFIG_REGULATOR_MAX77663)
static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
};

static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("mhl_1v2", NULL),
};

static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};

static struct max77663_regulator_fps_cfg max77663_fps_cfg[] = {
	{
		.src		= FPS_SRC_0,
		.en_src		= FPS_EN_SRC_EN0,
		.time_period	= FPS_TIME_PERIOD_DEF,
	},
	{
		.src		= FPS_SRC_1,
		.en_src		= FPS_EN_SRC_EN1,
		.time_period	= FPS_TIME_PERIOD_DEF,
	},
	{
		.src		= FPS_SRC_2,
		.en_src		= FPS_EN_SRC_EN0,
		.time_period	= FPS_TIME_PERIOD_DEF,
	},
};

#define MAX77663_PDATA_INIT(_id, _min_uV, _max_uV, _supply_reg,		\
			    _always_on, _boot_on, _apply_uV,		\
			    _init_apply, _init_enable, _init_uV,	\
			    _fps_src, _fps_pu_period, _fps_pd_period, _flags) \
	static struct max77663_regulator_platform_data max77663_regulator_pdata_##_id = \
	{								\
		.init_data = {						\
			.constraints = {				\
				.min_uV = _min_uV,			\
				.max_uV = _max_uV,			\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						     REGULATOR_MODE_STANDBY), \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						   REGULATOR_CHANGE_STATUS |  \
						   REGULATOR_CHANGE_VOLTAGE), \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uV,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(max77663_##_id##_supply),	\
			.consumer_supplies = max77663_##_id##_supply,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_apply = _init_apply,				\
		.init_enable = _init_enable,				\
		.init_uV = _init_uV,					\
		.fps_src = _fps_src,					\
		.fps_pu_period = _fps_pu_period,			\
		.fps_pd_period = _fps_pd_period,			\
		.fps_cfgs = max77663_fps_cfg,				\
		.flags = _flags,					\
	}

#if defined(CONFIG_MFD_MAX77663_SD_FORCED_PWM)
MAX77663_PDATA_INIT(sd0,  600000, 3387500, NULL, 1, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, EN2_CTRL_SD0|SD_FORCED_PWM_MODE);
#else
MAX77663_PDATA_INIT(sd0,  600000, 3387500, NULL, 1, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, EN2_CTRL_SD0);
#endif

#if defined(CONFIG_MFD_MAX77663_SD_FORCED_PWM)
MAX77663_PDATA_INIT(sd1,  800000, 1587500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, SD_FORCED_PWM_MODE);
#else
MAX77663_PDATA_INIT(sd1,  800000, 1587500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);
#endif

/* SD2 must be always turned on because used as pull-up signal for NRST_IO. */
MAX77663_PDATA_INIT(sd2,  600000, 3387500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

#if defined(CONFIG_MFD_MAX77663_SD_FORCED_PWM)
MAX77663_PDATA_INIT(sd3,  600000, 3387500, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, SD_FORCED_PWM_MODE);
#else
MAX77663_PDATA_INIT(sd3,  600000, 3387500, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);
#endif

MAX77663_PDATA_INIT(ldo0, 800000, 2350000, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(ldo1, 800000, 2350000, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);
//                                                       
MAX77663_PDATA_INIT(ldo2, 800000, 3950000, NULL, 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);
MAX77663_PDATA_INIT(ldo3, 3000000, 3000000, NULL, 0, 0, 0,
		    1, 1, 3000000, FPS_SRC_NONE, -1, -1, 0);
/* LDO4 must be always turned on because connected with vdd_rtc. */
MAX77663_PDATA_INIT(ldo4, 800000, 1587500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(ldo5, 800000, 3950000, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);
//
//                                                       
MAX77663_PDATA_INIT(ldo6, 800000, 3950000, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);
MAX77663_PDATA_INIT(ldo7, 800000, 3950000, NULL, 0, 0, 0,
			1, 0, 1250000, FPS_SRC_NONE, -1, -1, 0);
MAX77663_PDATA_INIT(ldo8, 800000, 3950000, max77663_rails(sd2), 0, 0, 0,
			0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

#define MAX77663_REG(_id, _data)				\
	{							\
		.name = "max77663-regulator",			\
		.id = MAX77663_REGULATOR_ID_##_id,		\
		.platform_data = &max77663_regulator_pdata_##_data, \
		.pdata_size = sizeof(max77663_regulator_pdata_##_data), \
	}

#define MAX77663_RTC()						\
	{							\
		.name = "max77663-rtc",				\
		.id = 0,					\
	}

static struct mfd_cell max77663_subdevs[] = {
	MAX77663_REG(SD0, sd0),
	MAX77663_REG(SD1, sd1),
	MAX77663_REG(SD2, sd2),
	MAX77663_REG(SD3, sd3),
	MAX77663_REG(LDO0, ldo0),
	MAX77663_REG(LDO1, ldo1),
	MAX77663_REG(LDO2, ldo2),
	MAX77663_REG(LDO3, ldo3),
	MAX77663_REG(LDO4, ldo4),
	MAX77663_REG(LDO5, ldo5),
	MAX77663_REG(LDO6, ldo6),
	MAX77663_REG(LDO7, ldo7),
	MAX77663_REG(LDO8, ldo8),
	MAX77663_RTC(),
};

//                                                       
struct max77663_gpio_config max77663_gpio_cfg[] = {
	/*
	 * 4 AP usb detect..
	 */
        {
		.gpio		= MAX77663_GPIO1,
		.dir		= GPIO_DIR_OUT,
		.out_drv	= GPIO_OUT_DRV_OPEN_DRAIN,
		/*
                          
                                     
                                            
   */
		.alternate	= GPIO_ALT_DISABLE,

 	},
	/*
	 * 4 sleep clock..
	 */
	{
		.gpio		= MAX77663_GPIO4,
		.dir            = GPIO_DIR_OUT,
		.out_drv        = GPIO_OUT_DRV_PUSH_PULL,
		.alternate	= GPIO_ALT_ENABLE,
	},
};

static struct max77663_platform_data max7763_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,

	.num_gpio_cfgs	= ARRAY_SIZE(max77663_gpio_cfg),
	.gpio_cfgs	= max77663_gpio_cfg,

	.num_subdevs	= ARRAY_SIZE(max77663_subdevs),
	.sub_devices	= max77663_subdevs,
	.flags			= SLP_LPM_ENABLE,
};
#endif  // CONFIG_REGULATOR_MAX77663
/************************ MAX77663 based regulator ****************/


static struct i2c_board_info __initdata x3_regulators[] = {
#if defined(CONFIG_REGULATOR_TPS80031)
	{
		I2C_BOARD_INFO("tps80031", 0x48),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
#endif
#if defined(CONFIG_REGULATOR_MAX77663)
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x1C),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max7763_pdata,
	},
#endif
};

/************************ GPIO based switch regulator ****************/
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)


static struct regulator_consumer_supply gpio_switch_ldo_mhl_en_supply[] = {
        REGULATOR_SUPPLY("avdd_vhdmi_vtherm", NULL),
};
static int gpio_switch_ldo_mhl_en_voltages[] = {3300};

#if 0
static struct regulator_consumer_supply gpio_switch_ldo_sensor_3v0_en_supply[] = {
        REGULATOR_SUPPLY("vdd_nct1008", NULL),
        REGULATOR_SUPPLY("vdd_ina230", NULL),
        REGULATOR_SUPPLY("vdd_3v_ldo", NULL),
};
static int gpio_switch_ldo_sensor_3v0_en_voltages[] = {3000};

static struct regulator_consumer_supply gpio_switch_ldo_sensor_1v8_en_supply[] = {
        REGULATOR_SUPPLY("vlg_1v8_ldo", NULL),
};
static int gpio_switch_ldo_sensor_1v8_en_voltages[] = {1800};
#else
static struct
regulator_consumer_supply gpio_switch_ldo_vtherm_en_supply[] = {
		REGULATOR_SUPPLY("vdd_nct1008", NULL),
		REGULATOR_SUPPLY("vdd_ina230", NULL),
};
static int gpio_switch_ldo_vtherm_en_voltages[] = {3300};

static struct regulator_consumer_supply gpio_switch_ldo_sensor_3v0_en_rev_d_supply[] = {
        REGULATOR_SUPPLY("vdd_nct1008", NULL),
        REGULATOR_SUPPLY("vdd_ina230", NULL),
        REGULATOR_SUPPLY("vdd_3v_ldo", NULL),
};
static int gpio_switch_ldo_sensor_3v0_en_rev_d_voltages[] = {3000};


static struct regulator_consumer_supply gpio_switch_ldo_sensor_1v8_en_rev_d_supply[] = {
        REGULATOR_SUPPLY("vlg_1v8_ldo", NULL),
};
static int gpio_switch_ldo_sensor_1v8_en_rev_d_voltages[] = {1800};


static struct regulator_consumer_supply gpio_switch_ldo_sensor_3v0_en_rev_e_supply[] = {
        REGULATOR_SUPPLY("vdd_nct1008", NULL),
        REGULATOR_SUPPLY("vdd_ina230", NULL),
        REGULATOR_SUPPLY("vdd_3v_ldo", NULL),
};
static int gpio_switch_ldo_sensor_3v0_en_rev_e_voltages[] = {3000};


static struct regulator_consumer_supply gpio_switch_ldo_sensor_1v8_en_rev_e_supply[] = {
        REGULATOR_SUPPLY("vlg_1v8_ldo", NULL),
};
static int gpio_switch_ldo_sensor_1v8_en_rev_e_voltages[] = {1800};

#endif

// +3V3_TPS_VFUSE
static struct
regulator_consumer_supply gpio_switch_ldo_vdd_fuse_3v3_en_supply[] = {
        REGULATOR_SUPPLY("vdd_fuse", NULL),
};
static int gpio_switch_ldo_vdd_fuse_3v3_en_voltages[] = {3300};


/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _name, _input_supply, _gpio_nr, _active_low, \
			_init_state, _pg, _enable, _disable)		\
	static struct gpio_switch_regulator_subdev_data gpio_pdata_##_name =  \
	{								\
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.pin_group	= _pg,					\
		.active_low	= _active_low,				\
		.init_state	= _init_state,				\
		.voltages	= gpio_switch_##_name##_voltages,	\
		.n_voltages	= ARRAY_SIZE(gpio_switch_##_name##_voltages), \
		.num_consumer_supplies =				\
				ARRAY_SIZE(gpio_switch_##_name##_supply), \
		.consumer_supplies = gpio_switch_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}

#if 0

#define MHL_LDO_EN      TEGRA_GPIO_PV2

#if defined(MACH_X3_REV_D)
#define SENSOR_LDO_EN   TEGRA_GPIO_PH2
#define SENSOR_LDO_EN2  TEGRA_GPIO_PH6
#else // After REV.E
#define SENSOR_LDO_EN   TEGRA_GPIO_PX7
#define SENSOR_LDO_EN2  TEGRA_GPIO_PD2
#endif
#define VFUSE_LDO_EN	TEGRA_GPIO_PK7

GREG_INIT(0, ldo_mhl_en, NULL, MHL_LDO_EN, false, 0, 0, 0, 0);
GREG_INIT(1, ldo_sensor_3v0_en, NULL, SENSOR_LDO_EN, false, 0, 0, 0, 0);
GREG_INIT(2, ldo_sensor_1v8_en, NULL, SENSOR_LDO_EN2, false, 0, 0, 0, 0);
GREG_INIT(3, ldo_vdd_fuse_3v3_en, NULL, VFUSE_LDO_EN, false, 0, 0, 0, 0);

#define ADD_GPIO_REG(_name)	(&gpio_pdata_##_name)
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs[] = {
        ADD_GPIO_REG(ldo_mhl_en),
        ADD_GPIO_REG(ldo_sensor_3v0_en),
        ADD_GPIO_REG(ldo_sensor_1v8_en),
        ADD_GPIO_REG(ldo_vdd_fuse_3v3_en),
};

#else


#define MHL_LDO_EN      TEGRA_GPIO_PV2
#define THERM_LDO_EN	TEGRA_GPIO_PK6

#define SENSOR_LDO_EN_REV_D   TEGRA_GPIO_PH2
#define SENSOR_LDO_EN2_REV_D  TEGRA_GPIO_PH6

#define SENSOR_LDO_EN_REV_E   TEGRA_GPIO_PX7
#define SENSOR_LDO_EN2_REV_E  TEGRA_GPIO_PD2

#define VFUSE_LDO_EN	TEGRA_GPIO_PK7

GREG_INIT(0, ldo_mhl_en, NULL, MHL_LDO_EN, false, 0, 0, 0, 0);
GREG_INIT(1, ldo_vtherm_en, NULL, THERM_LDO_EN, false, 0, 0, 0, 0);
GREG_INIT(2, ldo_sensor_3v0_en_rev_d, NULL, SENSOR_LDO_EN_REV_D, false, 0, 0, 0, 0);
GREG_INIT(3, ldo_sensor_1v8_en_rev_d, NULL, SENSOR_LDO_EN2_REV_D, false, 0, 0, 0, 0);
GREG_INIT(4, ldo_sensor_3v0_en_rev_e, NULL, SENSOR_LDO_EN_REV_E, false, 0, 0, 0, 0);
GREG_INIT(5, ldo_sensor_1v8_en_rev_e, NULL, SENSOR_LDO_EN2_REV_E, false, 0, 0, 0, 0);
GREG_INIT(6, ldo_vdd_fuse_3v3_en, NULL, VFUSE_LDO_EN, false, 0, 0, 0, 0);

#define ADD_GPIO_REG(_name)	(&gpio_pdata_##_name)
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs_rev_C[] = {
	ADD_GPIO_REG(ldo_mhl_en),
	ADD_GPIO_REG(ldo_vtherm_en),
	ADD_GPIO_REG(ldo_vdd_fuse_3v3_en),
};

static struct gpio_switch_regulator_subdev_data *gswitch_subdevs_rev_D[] = {
	ADD_GPIO_REG(ldo_mhl_en),
	ADD_GPIO_REG(ldo_sensor_3v0_en_rev_d),
	ADD_GPIO_REG(ldo_sensor_1v8_en_rev_d),
	ADD_GPIO_REG(ldo_vdd_fuse_3v3_en),
};

static struct gpio_switch_regulator_subdev_data *gswitch_subdevs_rev_E[] = {
	ADD_GPIO_REG(ldo_mhl_en),
	ADD_GPIO_REG(ldo_sensor_3v0_en_rev_e),
	ADD_GPIO_REG(ldo_sensor_1v8_en_rev_e),
	ADD_GPIO_REG(ldo_vdd_fuse_3v3_en),
};


static struct gpio_switch_regulator_platform_data  gswitch_pdata_rev_C = {
	.num_subdevs	= ARRAY_SIZE(gswitch_subdevs_rev_C),
	.subdevs	= gswitch_subdevs_rev_C,
};

static struct platform_device gswitch_regulator_pdata_rev_C = {
	.name	= "gpio-switch-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &gswitch_pdata_rev_C,
	},
};

static struct gpio_switch_regulator_platform_data  gswitch_pdata_rev_D = {
	.num_subdevs	= ARRAY_SIZE(gswitch_subdevs_rev_D),
	.subdevs	= gswitch_subdevs_rev_D,
};

static struct platform_device gswitch_regulator_pdata_rev_D = {
	.name	= "gpio-switch-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &gswitch_pdata_rev_D,
	},
};

static struct gpio_switch_regulator_platform_data  gswitch_pdata_rev_E = {
	.num_subdevs	= ARRAY_SIZE(gswitch_subdevs_rev_E),
	.subdevs	= gswitch_subdevs_rev_E,
};

static struct platform_device gswitch_regulator_pdata_rev_E = {
	.name	= "gpio-switch-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &gswitch_pdata_rev_E,
	},
};

static int __init x3_gpio_switch_regulator_init_rev_C(void)
{
	int i;

	for (i = 0; i < gswitch_pdata_rev_C.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata_rev_C.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}
	return platform_device_register(&gswitch_regulator_pdata_rev_C);
}


static int __init x3_gpio_switch_regulator_init_rev_D(void)
{
	int i;

	for (i = 0; i < gswitch_pdata_rev_D.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata_rev_D.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}
	return platform_device_register(&gswitch_regulator_pdata_rev_D);
}

static int __init x3_gpio_switch_regulator_init_rev_E(void)
{
	int i;

	for (i = 0; i < gswitch_pdata_rev_E.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata_rev_E.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}
	return platform_device_register(&gswitch_regulator_pdata_rev_E);
}

#endif


#if 0

static struct gpio_switch_regulator_platform_data  gswitch_pdata = {
	.num_subdevs	= ARRAY_SIZE(gswitch_subdevs),
	.subdevs	= gswitch_subdevs,
};

static struct platform_device gswitch_regulator_pdata = {
	.name	= "gpio-switch-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &gswitch_pdata,
	},
};

static int __init x3_gpio_switch_regulator_init(void)
{
	int i;

	for (i = 0; i < gswitch_pdata.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}
	return platform_device_register(&gswitch_regulator_pdata);
}

#endif

#endif // CONFIG_REGULATOR_GPIO_SWITCH

#if defined(CONFIG_REGULATOR_AAT2870)
/*
 * AAT2870 backlight and regulator
 */
static struct regulator_consumer_supply aat2870_ldoa_supply[] = {
	REGULATOR_SUPPLY("vddio_lcd_1v8", NULL), /*           */
};

static struct regulator_consumer_supply aat2870_ldob_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_3v0", NULL), /*           */
};

static struct regulator_consumer_supply aat2870_ldoc_supply[] = {
	REGULATOR_SUPPLY("vddio_touch_1v8", NULL), /*           */
};

static struct regulator_consumer_supply aat2870_ldod_supply[] = {
	REGULATOR_SUPPLY("vdd_touch_3v0", NULL), /*           */
};

#define AAT2870_PDATA_INIT(_id, _minmv, _maxmv, _supply_reg, _always_on, \
			   _boot_on, _apply_uv)				\
	{							\
		.constraints = {				\
			.min_uV = (_minmv) * 1000,		\
			.max_uV = (_maxmv) * 1000,		\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,		\
			.boot_on = _boot_on,			\
			.apply_uV = _apply_uv,			\
		},						\
		.num_consumer_supplies = ARRAY_SIZE(aat2870_##_id##_supply), \
		.consumer_supplies = aat2870_##_id##_supply,	\
	}

static struct regulator_init_data aat2870_ldoa_data =
			AAT2870_PDATA_INIT(ldoa, 1200, 3300, 0, 1, 1, 0);

static struct regulator_init_data aat2870_ldob_data =
			AAT2870_PDATA_INIT(ldob, 1200, 3300, 0, 1, 1, 0);

static struct regulator_init_data aat2870_ldoc_data =
			AAT2870_PDATA_INIT(ldoc, 1200, 3300, 0, 1, 1, 0);

static struct regulator_init_data aat2870_ldod_data =
			AAT2870_PDATA_INIT(ldod, 1200, 3300, 0, 1, 1, 0);

struct aat2870_bl_platform_data aat2870_bl_pdata = {
	.channels	= AAT2870_BL_CH_ALL,
	.max_current	= AAT2870_CURRENT_19_8,
	.max_brightness	= 255,
};

#define AAT2870_REG(_id, _data)			\
	{					\
		.id = AAT2870_ID_##_id,		\
		.name = "aat2870-regulator",	\
		.platform_data = _data,		\
	}

struct aat2870_subdev_info aat2870_subdevs[] = {
	AAT2870_REG(LDOA, &aat2870_ldoa_data),
	AAT2870_REG(LDOB, &aat2870_ldob_data),
	AAT2870_REG(LDOC, &aat2870_ldoc_data),
	AAT2870_REG(LDOD, &aat2870_ldod_data),
	{
		.id = AAT2870_ID_BL,
		.name = "aat2870-backlight",
		.platform_data = &aat2870_bl_pdata,
	},
};

static int aat2870_init(struct aat2870_data *aat2870)
{
	if (aat2870->en_pin >= 0)
		tegra_gpio_enable(aat2870->en_pin);

	return 0;
}

static void aat2870_uninit(struct aat2870_data *aat2870)
{
	if (aat2870->en_pin >= 0)
		tegra_gpio_disable(aat2870->en_pin);
}

static struct aat2870_platform_data aat2870_pdata = {
	.en_pin		= TEGRA_GPIO_PM7,
	.subdevs	= aat2870_subdevs,
	.num_subdevs	= ARRAY_SIZE(aat2870_subdevs),
	.init		= aat2870_init,
	.uninit		= aat2870_uninit,
};

static struct i2c_board_info __initdata x3_aat2870_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("aat2870", 0x60),
		.platform_data = &aat2870_pdata,
	},
};
#endif

static void x3_power_off(void)
{
	int ret;

	pr_err("Powering off the device\n");

#if defined(CONFIG_MFD_TPS80031)
	/*
	 * power-code should be completed in tps80031
	 */
	ret = tps80031_power_off();
#endif /* defined(CONFIG_MFD_TPS80031) */
#if defined(CONFIG_MFD_MAX77663)
	/*
	 * power-code should be completed in tps80031
	 */
	ret = max77663_power_off();
#endif
	if (ret)
		pr_err("failed to power off\n");

	while(1);
}


int __init x3_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
//	u32 pmc_dpd_pads;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

//	pmc_dpd_pads = readl(pmc + PMC_DPD_PADS_ORIDE);
//	writel(pmc_dpd_pads & ~PMC_DPD_PADS_ORIDE_BLINK , pmc + PMC_DPD_PADS_ORIDE);

#if defined(CONFIG_MFD_TPS80031)
	/* Disable battery charging if power adapter is connected. */
	if (get_power_supply_type() == POWER_SUPPLY_TYPE_MAINS) {
		bcharger_pdata.num_consumer_supplies = 0;
		bcharger_pdata.consumer_supplies = NULL;
		battery_gauge_data.battery_present = 0;
	}
#endif

	i2c_register_board_info(4, x3_regulators,
			ARRAY_SIZE(x3_regulators));

#if defined(CONFIG_REGULATOR_AAT2870)
	i2c_register_board_info(2, x3_aat2870_i2c_board_info,
			ARRAY_SIZE(x3_aat2870_i2c_board_info));
#endif

#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
#if 0
	x3_gpio_switch_regulator_init();
#else
	switch(x3_get_hw_rev_pcb_version())
	{
		case hw_rev_pcb_type_A :
		case hw_rev_pcb_type_B :
		case hw_rev_pcb_type_C :
			x3_gpio_switch_regulator_init_rev_C();
			break;
		case hw_rev_pcb_type_D :
			x3_gpio_switch_regulator_init_rev_D();
			break;
		case hw_rev_pcb_type_E :
		default :
			x3_gpio_switch_regulator_init_rev_E();
			break;
	}
#endif
#endif
	pm_power_off = x3_power_off;
	return 0;
}

static void x3_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void x3_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data x3_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
//	.wake_low	   =  TEGRA_WAKE_GPIO_PV0,
//	.wake_enb	  = TEGRA_WAKE_GPIO_PV0,
	.board_suspend = x3_board_suspend,
	.board_resume = x3_board_resume,
//                                  
        .cpu_wake_freq = CPU_WAKE_FREQ_LOW,
        .cpu_lp2_min_residency = 20000,
};

static void x3_init_deep_sleep_mode(void)
{
#if defined(CONFIG_MACH_X3)
	x3_suspend_data.suspend_mode = TEGRA_SUSPEND_LP0;
#else
	struct board_info bi;
	tegra_get_board_info(&bi);
	if (bi.board_id == BOARD_1205 && bi.fab == X3_FAB_A01)
		x3_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;
#endif
}

int __init x3_suspend_init(void)
{
	x3_init_deep_sleep_mode();
	tegra_init_suspend(&x3_suspend_data);

	return 0;
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init x3_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 4700; /* regular AP30 */
	}
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	tegra_init_system_edp_limits(TEGRA_BPC_CPU_PWR_LIMIT);
	return 0;
}
#endif

static struct tegra_bpc_mgmt_platform_data bpc_mgmt_platform_data = {
	.gpio_trigger = TEGRA_BPC_TRIGGER,
	.bpc_mgmt_timeout = TEGRA_BPC_TIMEOUT,
};

static struct platform_device x3_bpc_mgmt_device = {
	.name		= "tegra-bpc-mgmt",
	.id		= -1,
	.dev		= {
		.platform_data = &bpc_mgmt_platform_data,
	},
};

void __init x3_bpc_mgmt_init(void)
{
#ifdef CONFIG_SMP
	int int_gpio = TEGRA_GPIO_TO_IRQ(TEGRA_BPC_TRIGGER);

	tegra_gpio_enable(TEGRA_BPC_TRIGGER);

	cpumask_setall(&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity_hint(int_gpio,
				&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity(int_gpio, &(bpc_mgmt_platform_data.affinity_mask));
#endif
	platform_device_register(&x3_bpc_mgmt_device);

	return;
}


static irqreturn_t max77663_irq_manualreset_warning(int irq, void *data)
{
	printk("%s\n", __func__);
//	show_state_filter(TASK_UNINTERRUPTIBLE);
//	show_state();  // append debug message
//	BUG();
	kmsg_dump(KMSG_DUMP_PANIC);

	return IRQ_HANDLED;
}

#if defined(CONFIG_PMIC_POWERKEY)
/* Fucking Power Key */
static irqreturn_t max77663_irq_en0_falling(int irq, void *data)
{
	input_report_key(max77663_pwrkey, KEY_POWER, 0);
	input_sync(max77663_pwrkey);
	return IRQ_HANDLED;
}

static irqreturn_t max77663_irq_en0_rising(int irq, void *data)
{
	input_report_key(max77663_pwrkey, KEY_POWER, 1);
	input_sync(max77663_pwrkey);
	return IRQ_HANDLED;
}
#endif

int __init x3_power_late_init(void)
{
	int irq;
	int ret;

	if(is_tegra_bootmode())
	{
		irq = max7763_pdata.irq_base + MAX77663_IRQ_ONOFF_HRDPOWRN;
		ret = request_threaded_irq(irq , NULL, max77663_irq_manualreset_warning,
				IRQF_ONESHOT, "MAX77663_IRQ_ONOFF_HRDPOWRN",
				NULL);
		if (ret) {
			pr_err("%s: Failed to request irq %d\n", __func__, irq);
			return ret;
		}
	}

#if defined(CONFIG_PMIC_POWERKEY)
	max77663_pwrkey = input_allocate_device();
	if (!max77663_pwrkey) {
		pr_err("%s: Failed to allocate input device for power key\n",
				__func__);
		return -ENOMEM;
	}

	max77663_pwrkey->evbit[0] = BIT_MASK(EV_KEY);
	set_bit(KEY_POWER, max77663_pwrkey->keybit);
	max77663_pwrkey->name = "max77663-pwrkey";

	ret = input_register_device(max77663_pwrkey);
	if (ret) {
		pr_err("%s: Failed to register input device for power key\n",
				__func__);
		goto out_free_device;
	}

	irq = max7763_pdata.irq_base + MAX77663_IRQ_ONOFF_EN0_FALLING;
	ret = request_threaded_irq(irq , NULL, max77663_irq_en0_falling,
			IRQF_ONESHOT, "max77663-en0-falling",
			max77663_pwrkey);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__, irq);
		goto out_unregister_device;
	}

	irq = max7763_pdata.irq_base + MAX77663_IRQ_ONOFF_EN0_RISING;
	ret = request_threaded_irq(irq , NULL, max77663_irq_en0_rising,
			IRQF_ONESHOT, "max77663-en0-rising",
			max77663_pwrkey);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__, irq);
		goto out_free_irq;
	}

	return 0;

out_free_irq:
	irq = max7763_pdata.irq_base + MAX77663_IRQ_ONOFF_EN0_FALLING;
	free_irq(irq, max77663_pwrkey);
out_unregister_device:
	input_unregister_device(max77663_pwrkey);
	max77663_pwrkey = NULL;
out_free_device:
	if (max77663_pwrkey)
		input_free_device(max77663_pwrkey);

	return ret;
#endif
}
late_initcall(x3_power_late_init);
