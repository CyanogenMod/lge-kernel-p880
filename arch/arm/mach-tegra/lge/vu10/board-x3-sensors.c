/*
 * arch/arm/mach-tegra/board-x3-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mpu.h>
#include <linux/nct1008.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SENSORS_INA230
#include <linux/platform_data/ina230.h>
#endif
#include <mach/gpio.h>
#include <media/ar0832_main.h>
#include <media/imx111.h>
#include <media/mt9m114.h>
#include <media/tps61050.h>
#include <media/ov9726.h>
#include <mach/edp.h>
#include <linux/slab.h>
#include <mach/thermal.h>


#include <mach-tegra/cpu-tegra.h>
#include <mach-tegra/gpio-names.h>
#include <lge/board-x3.h>
#if defined(CONFIG_VIDEO_IMX119)
#include <media/imx119.h>
#endif

#include <media/lm3559.h>
#include <media/dw9714.h>

static struct lm3559_platform_data flash_led_data = {
	.gpio_act = TEGRA_GPIO_PBB3,
};

static struct dw9714_info focuser_data = {
};
#define X3_NCT1008_IRQ	TEGRA_GPIO_PI5

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;

	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
			 void (*alert_func)(void *),
			 void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data,
						shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
			GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->id = THERMAL_DEVICE_ID_NCT_EXT;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(thermal_device);
}

static struct nct1008_platform_data x3_nct1008_pdata = {
	.supported_hwrev	= true,
	.ext_range		= true,
	.conv_rate		= 0x08,
	/* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.offset			= 8, 
	.probe_callback		= nct1008_probe_callback,
};

static struct i2c_board_info x3_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(X3_NCT1008_IRQ),
		.platform_data = &x3_nct1008_pdata,
	}
};

static void x3_nct1008_init(void)
{
	int ret;

	tegra_gpio_enable(X3_NCT1008_IRQ);
	ret = gpio_request(X3_NCT1008_IRQ, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(X3_NCT1008_IRQ);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(X3_NCT1008_IRQ);
		return;
	}
	
	i2c_register_board_info(4, x3_i2c4_nct1008_board_info,
				ARRAY_SIZE(x3_i2c4_nct1008_board_info));
}

static inline void x3_msleep(u32 t)
{
	/*
	If timer value is between ( 10us - 20ms),
	usleep_range() is recommended.
	Please read Documentation/timers/timers-howto.txt.
	*/
	usleep_range(t*1000, t*1000 + 500);
}

/*
static struct i2c_board_info x3_i2c0_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};
*/

#ifdef CONFIG_SENSORS_ISL29028	
static void x3_isl_init(void)
{
	i2c_register_board_info(0, x3_i2c0_isl_board_info,
				ARRAY_SIZE(x3_i2c0_isl_board_info));
}
#endif

#ifdef CONFIG_STEREO_CAMERA_USE //                        

enum CAMERA_INDEX {
	CAM_REAR_LEFT,
	CAM_REAR_RIGHT,
	CAM_FRONT,
	NUM_OF_CAM
};

struct x3_power_rail {
	struct regulator *cam_reg;
	struct regulator *csi_reg;
};

static struct x3_power_rail ent_vicsi_pwr[NUM_OF_CAM];

static int x3_cam_pwr(enum CAMERA_INDEX cam, bool pwr_on)
{
	struct x3_power_rail *reg_cam = &ent_vicsi_pwr[cam];
	int ret = 0;

	/*
	* SW must turn on 1.8V first then 2.8V
	* SW must turn off 2.8V first then 1.8V
	*/
	if (pwr_on) {
		if (reg_cam->csi_reg == NULL) {
			reg_cam->csi_reg = regulator_get(NULL,
						"avdd_dsi_csi");
			if (IS_ERR_OR_NULL(reg_cam->csi_reg)) {
				pr_err("%s: csi pwr err\n", __func__);
				ret = PTR_ERR(reg_cam->csi_reg);
				goto x3_cam_pwr_fail;
			}
		}

		ret = regulator_enable(reg_cam->csi_reg);
		if (ret) {
			pr_err("%s: enable csi pwr err\n", __func__);
			goto x3_cam_pwr_fail;
		}

		if (reg_cam->cam_reg == NULL) {
			reg_cam->cam_reg = regulator_get(NULL,
						"vddio_cam");
			if (IS_ERR_OR_NULL(reg_cam->cam_reg)) {
				pr_err("%s: vddio pwr err\n", __func__);
				ret = PTR_ERR(reg_cam->cam_reg);
				regulator_disable(reg_cam->csi_reg);
				goto x3_cam_pwr_fail;
			}
		}

		ret = regulator_enable(reg_cam->cam_reg);
		if (ret) {
			pr_err("%s: enable vddio pwr err\n", __func__);
			regulator_disable(reg_cam->csi_reg);
			goto x3_cam_pwr_fail;
		}
	} else {
		if (reg_cam->cam_reg)
			regulator_disable(reg_cam->cam_reg);

		if (reg_cam->csi_reg)
			regulator_disable(reg_cam->csi_reg);
	}
	return 0;

x3_cam_pwr_fail:
	if (!IS_ERR_OR_NULL(reg_cam->cam_reg))
		regulator_put(reg_cam->cam_reg);
	reg_cam->cam_reg = NULL;

	if (!IS_ERR_OR_NULL(reg_cam->csi_reg))
		regulator_put(reg_cam->csi_reg);
	reg_cam->csi_reg = NULL;

	return ret;
}

static int x3_ar0832_ri_power_on(int is_stereo)
{
	int ret = 0;

	ret = x3_cam_pwr(CAM_REAR_RIGHT, true);

	/* Release Reset */
	if (is_stereo) {
		gpio_set_value(CAM1_RST_L_GPIO, 1);
		gpio_set_value(CAM2_RST_L_GPIO, 1);
	} else
		gpio_set_value(CAM1_RST_L_GPIO, 1);
	/*
	It takes 2400 EXTCLK for ar0832 to be ready for I2c.
	EXTCLK is 10 ~ 24MHz. 1 ms should be enough to cover
	at least 2400 EXTCLK within frequency range.
	*/
	x3_msleep(1);

	return ret;
}

static int x3_ar0832_le_power_on(int is_stereo)
{
	int ret = 0;

	pr_info("%s: ++\n", __func__);
	ret = x3_cam_pwr(CAM_REAR_LEFT, true);

	/* Release Reset */
	gpio_set_value(CAM2_RST_L_GPIO, 1);

	/*
	It takes 2400 EXTCLK for ar0832 to be ready for I2c.
	EXTCLK is 10 ~ 24MHz. 1 ms should be enough to cover
	at least 2400 EXTCLK within frequency range.
	*/
	x3_msleep(1);

	/* CSI B is shared between Front camera and Rear Left camera */
	gpio_set_value(CAM_CSI_MUX_SEL_GPIO, 1);

	return ret;
}

static int x3_ar0832_ri_power_off(int is_stereo)
{
	int ret;

	pr_info("%s: ++\n", __func__);
	ret = x3_cam_pwr(CAM_REAR_RIGHT, false);

	/* Assert Reset */
	if (is_stereo) {
		gpio_set_value(CAM1_RST_L_GPIO, 0);
		gpio_set_value(CAM2_RST_L_GPIO, 0);
	} else
		gpio_set_value(CAM1_RST_L_GPIO, 0);

	return ret;
}

static int x3_ar0832_le_power_off(int is_stereo)
{
	int ret;

	pr_info("%s: ++\n", __func__);
	ret = x3_cam_pwr(CAM_REAR_LEFT, false);

	/* Assert Reset */
	gpio_set_value(CAM2_RST_L_GPIO, 0);

	return ret;
}

static int x3_ov9726_power_on(void)
{
	pr_info("ov9726 power on\n");

	/* switch mipi mux to front camera */
	gpio_set_value(CAM_CSI_MUX_SEL_GPIO, CAM_CSI_MUX_SEL_FRONT);
	x3_cam_pwr(CAM_FRONT, true);

	return 0;
}

static int x3_ov9726_power_off(void)
{
	pr_info("ov9726 power off\n");

	x3_cam_pwr(CAM_FRONT, false);

	return 0;
}

struct ov9726_platform_data x3_ov9726_data = {
	.power_on = x3_ov9726_power_on,
	.power_off = x3_ov9726_power_off,
	.gpio_rst = CAM3_RST_L_GPIO,
	.rst_low_active = true,
	.gpio_pwdn = CAM3_PWDN_GPIO,
	.pwdn_low_active = false,
};

static struct tps61050_pin_state x3_tps61050_pinstate = {
	.mask		= 0x0008, /*VGP3*/
	.values		= 0x0008,
};

/* I2C bus becomes active when vdd_1v8_cam is enabled */
static int x3_tps61050_pm(int pwr)
{
	static struct regulator *x3_flash_reg = NULL;
	int ret = 0;

	pr_info("%s: ++%d\n", __func__, pwr);
	switch (pwr) {
	case TPS61050_PWR_OFF:
		if (x3_flash_reg) {
			regulator_disable(x3_flash_reg);
			regulator_put(x3_flash_reg);
			x3_flash_reg = NULL;
		}
		break;
	case TPS61050_PWR_STDBY:
	case TPS61050_PWR_COMM:
	case TPS61050_PWR_ON:
		x3_flash_reg = regulator_get(NULL, "vdd_1v8_cam");
		if (IS_ERR_OR_NULL(x3_flash_reg)) {
			pr_err("%s: failed to get flash pwr\n", __func__);
			return PTR_ERR(x3_flash_reg);
		}
		ret = regulator_enable(x3_flash_reg);
		if (ret) {
			pr_err("%s: failed to enable flash pwr\n", __func__);
			goto fail_regulator_flash_reg;
		}
		x3_msleep(10);
		break;
	default:
		ret = -1;
	}
	return ret;

fail_regulator_flash_reg:
	regulator_put(x3_flash_reg);
	x3_flash_reg = NULL;
	return ret;
}


struct x3_cam_gpio {
	int gpio;
	const char *label;
	int value;
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)	\
	{						\
		.gpio = _gpio,				\
		.label = _label,			\
		.value = _value,			\
	}

static struct x3_cam_gpio x3_cam_gpio_data[] = {
	[0] = TEGRA_CAMERA_GPIO(CAM_CSI_MUX_SEL_GPIO, "cam_csi_sel", 1),
	[1] = TEGRA_CAMERA_GPIO(CAM1_RST_L_GPIO, "cam1_rst_lo", 0),
	[2] = TEGRA_CAMERA_GPIO(CAM2_RST_L_GPIO, "cam2_rst_lo", 0),
	[3] = TEGRA_CAMERA_GPIO(CAM3_RST_L_GPIO, "cam3_rst_lo", 0),
	[4] = TEGRA_CAMERA_GPIO(CAM3_PWDN_GPIO, "cam3_pwdn", 1),
	[5] = TEGRA_CAMERA_GPIO(CAM_FLASH_EN_GPIO, "flash_en", 1),
};

static struct ar0832_platform_data x3_ar0832_ri_data = {
	.power_on = x3_ar0832_ri_power_on,
	.power_off = x3_ar0832_ri_power_off,
	.id = "right",
};

static struct ar0832_platform_data x3_ar0832_le_data = {
	.power_on = x3_ar0832_le_power_on,
	.power_off = x3_ar0832_le_power_off,
	.id = "left",
};

static struct tps61050_platform_data x3_tps61050_data = {
	.cfg		= 0,
	.num		= 1,
	.max_amp_torch	= CAM_FLASH_MAX_TORCH_AMP,
	.max_amp_flash	= CAM_FLASH_MAX_FLASH_AMP,
	.pinstate	= &x3_tps61050_pinstate,
	.init		= NULL,
	.exit		= NULL,
	.pm		= &x3_tps61050_pm,
	.gpio_envm	= NULL,
	.gpio_sync	= NULL,
};
#endif

#if defined(CONFIG_VIDEO_AR0832)
static int x3_ar0832_power_on(void)
{  
#if 0 
  int ret;
  ret = x3_camera_power_on();
  return ret;
#else
  return 0;
#endif
}
  
static int x3_ar0832_power_off(void)
{
#if 0 
  int ret;
  ret = x3_camera_power_off();
  return ret;
#else
  return 0;
#endif
}

struct ar0832_platform_data x3_ar0832_data = {
    .power_on = x3_ar0832_power_on,
    .power_off = x3_ar0832_power_off,
};

#endif //defined(CONFIG_VIDEO_AR0832)

#if defined(CONFIG_VIDEO_IMX111)
#if 0 //hyojin.an 111104 
static int x3_imx111_power_on(void)
{  
#if 0 
  int ret;
  ret = x3_camera_power_on();
  return ret;
#else
  return 0;
#endif
}
  
static int x3_imx111_power_off(void)
{
#if 0 
  int ret;
  ret = x3_camera_power_off();
  return ret;
#else
  return 0;
#endif
}
#endif
struct imx111_platform_data x3_imx111_data = {
	//hyojin.an 111104 .power_on = x3_imx111_power_on,
	//hyojin.an 111104 .power_off = x3_imx111_power_off,
};

#endif //defined(CONFIG_VIDEO_IMX111)

#if defined(CONFIG_VIDEO_MT9M114)

static int x3_mt9m114_power_on(void)
{
#if 0  
	int ret;
	//struct board_info board_info;

  pr_err("%s\n", __func__);
	//tegra_get_board_info(&board_info);
	//sub camera enable
		tegra_gpio_enable(SUB_CAM_EN);
		ret = gpio_request(SUB_CAM_EN, "sub_cam_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "SUB_CAM_EN");

    		tegra_gpio_enable(SUB_CAM_RESET_N);
		ret = gpio_request(SUB_CAM_RESET_N, "sub_cam_reset_n");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "SUB_CAM_RESET_N");

		tegra_gpio_enable(SUB_CAM_PWDN);
		ret = gpio_request(SUB_CAM_PWDN, "sub_cam_pwdn");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "SUB_CAM_PWDN");

		gpio_direction_output(SUB_CAM_EN, 1);

		gpio_set_value(SUB_CAM_EN, 1);
  //I2C switch direction 1: out 0 : in (from AP30)
		gpio_direction_output(CAM_I2C_SEL1, 1);
		gpio_direction_output(CAM_I2C_SEL2, 1);
		gpio_direction_output(CAM_SEL_3D, 0);
    //I2C Switch set value
    gpio_set_value(CAM_I2C_SEL1, 1);
		gpio_set_value(CAM_I2C_SEL2, 1);
		gpio_set_value(CAM_SEL_3D, 0);

  //sub cam direction 1: out 0 : in (from AP30)
		gpio_direction_output(SUB_CAM_RESET_N, 1);
		gpio_direction_output(SUB_CAM_PWDN, 1);

    //sub cam set value
		gpio_set_value(SUB_CAM_RESET_N, 1);
		gpio_set_value(SUB_CAM_PWDN, 1);
#endif
	return 0;
}

static int x3_mt9m114_power_off(void)
{
  pr_err("%s\n", __func__);
	return 0;
}

struct mt9m114_platform_data x3_mt9m114_data = {
//	.power_on = x3_mt9m114_power_on,
//	.power_off = x3_mt9m114_power_off,
};

#endif //#if defined(CONFIG_VIDEO_MT9M114)

#if defined(CONFIG_VIDEO_IMX119)
struct imx119_platform_data x3_imx119_data = {
};
#endif


/*
 * Since ar0832 driver should support multiple devices, slave
 * address should be changed after it is open. Default slave
 * address of ar0832 is 0x36. It will be changed to alternate
 * address defined below when device is open.
 */
 
#if defined(CONFIG_VIDEO_AR0832) // hyojin.an 110624
static struct i2c_board_info ar0832_i2c2_boardinfo[] = {
#ifdef CONFIG_STEREO_CAMERA_USE //                        
	{
		/* 0x30: alternative slave address */
		I2C_BOARD_INFO("ar0832", 0x36),
		.platform_data = &x3_ar0832_ri_data,
	},
	{
		/* 0x31: alternative slave address */
		I2C_BOARD_INFO("ar0832", 0x32),
		.platform_data = &x3_ar0832_le_data,
	},
	{
		I2C_BOARD_INFO("tps61050", 0x33),
		.platform_data = &x3_tps61050_data,
	},
	{
		I2C_BOARD_INFO("ov9726", OV9726_I2C_ADDR >> 1),
		.platform_data = &x3_ov9726_data,
	},
#else
  	{
  		I2C_BOARD_INFO("ar0832", 0x36),
  		.platform_data = &x3_ar0832_data,
  	},
    //hyojin.an 110715 {
    //hyojin.an 110715     I2C_BOARD_INFO("ar0832_focuser", 0x36),
    //hyojin.an 110715 },
#endif
  };
#endif //defined(CONFIG_VIDEO_AR0832)

#if defined(CONFIG_VIDEO_IMX111)
static struct i2c_board_info imx111_i2c2_boardinfo[] = {
      {
        //chen.yingchun 20111221 slave address change for conflict with LM3533 on rev.b board
        //I2C_BOARD_INFO("imx111", 0x36),
        I2C_BOARD_INFO("imx111", 0x10),
        .platform_data = &x3_imx111_data,
      },

      {
         //I2C_BOARD_INFO("imx111_focuser", 0x18),
		I2C_BOARD_INFO("dw9714", 0x0c),
		.platform_data  = &focuser_data,
      },
      
      {
         I2C_BOARD_INFO("lm3559",  0x53),
         .platform_data  = &flash_led_data,
      },
};
#endif

#if defined(CONFIG_VIDEO_MT9M114)
static struct i2c_board_info mt9m114_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("mt9m114", 0x48),
		.platform_data = &x3_mt9m114_data,
	},
};
#endif //#if defined(CONFIG_VIDEO_MT9M114)

#if defined(CONFIG_VIDEO_IMX119)
static struct i2c_board_info imx119_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("imx119", 0x37),
		.platform_data = &x3_imx119_data,
	},
};
#endif

static int x3_cam_init(void)
{
	int ret;
	int i;

	pr_info("%s:++\n", __func__);

#ifdef CONFIG_STEREO_CAMERA_USE //                        
	memset(ent_vicsi_pwr, 0, sizeof(ent_vicsi_pwr));
	for (i = 0; i < ARRAY_SIZE(x3_cam_gpio_data); i++) {
		ret = gpio_request(x3_cam_gpio_data[i].gpio,
				x3_cam_gpio_data[i].label);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
					__func__, i);
			goto fail_free_gpio;
		}
		gpio_direction_output(x3_cam_gpio_data[i].gpio,
				x3_cam_gpio_data[i].value);
		gpio_export(x3_cam_gpio_data[i].gpio, false);
		tegra_gpio_enable(x3_cam_gpio_data[i].gpio);
	}
#endif

#if defined(CONFIG_VIDEO_AR0832)
	i2c_register_board_info(2, ar0832_i2c2_boardinfo,
			ARRAY_SIZE(ar0832_i2c2_boardinfo));
#endif
#if defined(CONFIG_VIDEO_IMX111)
	i2c_register_board_info(2, imx111_i2c2_boardinfo,
			ARRAY_SIZE(imx111_i2c2_boardinfo));
#endif

#if defined(CONFIG_VIDEO_MT9M114)
	i2c_register_board_info(2, mt9m114_i2c2_boardinfo,
			ARRAY_SIZE(mt9m114_i2c2_boardinfo));
#endif

#if defined(CONFIG_VIDEO_IMX119)
	i2c_register_board_info(2, imx119_i2c2_boardinfo,
			ARRAY_SIZE(imx119_i2c2_boardinfo));
#endif

	return 0;

#ifdef CONFIG_STEREO_CAMERA_USE //                        
fail_free_gpio:
	pr_err("%s x3_cam_init failed!\n", __func__);
#endif

#ifdef CONFIG_STEREO_CAMERA_USE //                        
	while (i--)
		gpio_free(x3_cam_gpio_data[i].gpio);
#endif  
	return ret;
}

#ifdef CONFIG_SENSORS_INA230
static struct ina230_platform_data ina230_platform = {
	.rail_name = "VDD_AC_BAT",
	.current_threshold = TEGRA_CUR_MON_THRESHOLD,
	.resistor = TEGRA_CUR_MON_RESISTOR,
	.min_cores_online = TEGRA_CUR_MON_MIN_CORES,
};

static struct i2c_board_info x3_i2c4_ina230_info[] = {
	{
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &ina230_platform,
		.irq = -1,
	},
};

static int __init x3_ina230_init(void)
{
	return i2c_register_board_info(4, x3_i2c4_ina230_info,
				       ARRAY_SIZE(x3_i2c4_ina230_info));
}
#endif

int __init x3_sensors_init(void)
{
	int ret;
#ifdef CONFIG_SENSORS_ISL29028	
	x3_isl_init();
#endif
	x3_nct1008_init();
#ifdef CONFIG_SENSORS_INA230
	x3_ina230_init();
#endif
	ret = x3_cam_init();

	return ret;
}

