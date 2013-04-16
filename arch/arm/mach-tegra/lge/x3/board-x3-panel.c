/*
 * arch/arm/mach-tegra/board-x3-panel.c
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/pwm_backlight.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <linux/spi/spi.h>
#include <mach/hardware.h>
#include <linux/pwm_backlight.h>

#include <mach-tegra/board.h>
#include <lge/board-x3.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>
#include <mach-tegra/tegra3_host1x_devices.h>
#include "../../../../../../drivers/video/tegra/ssd2825/ssd2825_bridge.h"

#include <mach-tegra/wakeups-t3.h>	//                                                                  
#if defined(CONFIG_MACH_RGB_CONVERTOR_SPI)
extern int ssd2825_bridge_enable(void);
extern int ssd2825_bridge_disable(void);
#endif

#define x3_hdmi_hpd	TEGRA_GPIO_PN7
#define TRUE 1
#define FALSE 0
static int x3_hddisplay_on = FALSE;
static struct workqueue_struct *bridge_work_queue;
static struct regulator *x3_hdmi_reg = NULL;
static struct regulator *x3_hdmi_pll = NULL;
static struct regulator *x3_hdmi_vddio = NULL;

//static struct regulator *x3_lgit_hdmi_pll = NULL;

static atomic_t sd_brightness = ATOMIC_INIT(255);

unsigned long long wake_status_backup=0; //                                                                  

static tegra_dc_bl_output x3_bl_output_measured = {
	1, 5, 9, 10, 11, 12, 12, 13,
	13, 14, 14, 15, 15, 16, 16, 17,
	17, 18, 18, 19, 19, 20, 21, 21,
	22, 22, 23, 24, 24, 25, 26, 26,
	27, 27, 28, 29, 29, 31, 31, 32,
	32, 33, 34, 35, 36, 36, 37, 38,
	39, 39, 40, 41, 41, 42, 43, 43,
	44, 45, 45, 46, 47, 47, 48, 49,
	49, 50, 51, 51, 52, 53, 53, 54,
	55, 56, 56, 57, 58, 59, 60, 61,
	61, 62, 63, 64, 65, 65, 66, 67,
	67, 68, 69, 69, 70, 71, 71, 72,
	73, 73, 74, 74, 75, 76, 76, 77,
	77, 78, 79, 79, 80, 81, 82, 83,
	83, 84, 85, 85, 86, 86, 88, 89,
	90, 91, 91, 92, 93, 93, 94, 95,
	95, 96, 97, 97, 98, 99, 99, 100,
	101, 101, 102, 103, 103, 104, 105, 105,
	107, 107, 108, 109, 110, 111, 111, 112,
	113, 113, 114, 115, 115, 116, 117, 117,
	118, 119, 119, 120, 121, 122, 123, 124,
	124, 125, 126, 126, 127, 128, 129, 129,
	130, 131, 131, 132, 133, 133, 134, 135,
	135, 136, 137, 137, 138, 139, 139, 140,
	142, 142, 143, 144, 145, 146, 147, 147,
	148, 149, 149, 150, 151, 152, 153, 153,
	153, 154, 155, 156, 157, 158, 158, 159,
	160, 161, 162, 163, 163, 164, 165, 165,
	166, 166, 167, 168, 169, 169, 170, 170,
	171, 172, 173, 173, 174, 175, 175, 176,
	176, 178, 178, 179, 180, 181, 182, 182,
	183, 184, 185, 186, 186, 187, 188, 188
};

static p_tegra_dc_bl_output bl_output;

static int x3_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);
	int orig_brightness = brightness;

	/* SD brightness is a percentage, 8-bit value. */
	brightness = (brightness * cur_sd_brightness) / 255;
	if (cur_sd_brightness != 255) {
		pr_info("NVSD BL - in: %d, sd: %d, out: %d\n",
			orig_brightness, cur_sd_brightness, brightness);
	}

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = bl_output[brightness];

	return brightness;
}


static struct platform_tegra_pwm_backlight_data x3_disp1_backlight_data = {
	.which_dc	= 0,
	.which_pwm	= TEGRA_PWM_PM1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.notify 	= x3_backlight_notify,
	.period 		= 0xFF,
	.clk_div		= 0x3FF,
	.clk_select 	= 0,
};

static struct platform_device x3_disp1_backlight_device = {
	.name	= "tegra-pwm-bl",
	.id	= -1,
	.dev	= {
		.platform_data = &x3_disp1_backlight_data,
	},
};

static int x3_backlight_init(struct device *dev) {
	return 0;
}

static void x3_backlight_exit(struct device *dev) {
}


static struct platform_pwm_backlight_data x3_backlight_data = {
	.pwm_id		= 2,	//                    
	.max_brightness	= 255,
	.max_current	= 0xB7,
	.dft_brightness	= 224,
	.pwm_period_ns	= 5000000,
	.init		= x3_backlight_init,
	.exit		= x3_backlight_exit,
	.notify		= x3_backlight_notify,
};

static struct platform_device x3_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &x3_backlight_data,
	},
};

static bool first_disp_boot = TRUE;
static int x3_panel_enable(void)
{
	printk("%s -- x3_hddisplay_on:%d \n",__func__,x3_hddisplay_on);
	if(!x3_hddisplay_on){
#if defined(CONFIG_MACH_RGB_CONVERTOR_SPI)
		//printk("system_state:%d, first_disp_boot:%d \n",system_state,first_disp_boot);
		if((system_state != SYSTEM_BOOTING) && (first_disp_boot != TRUE)){
			/*
			 * This will be NULL by x3_bridge_on
			 * when device is woken from LP0, however,
			 * this is needed to be called in
			 * late resume right after early suspend
			 */
			ssd2825_bridge_enable();
			x3_hddisplay_on = TRUE;
			return 0;
		}
		else{
			first_disp_boot = FALSE;
		}
#endif
	x3_hddisplay_on = TRUE;
	}
	return 0;
}

static int x3_postpoweron(void)
{
	//printk("%s -- x3_hddisplay_on:%d \n",__func__,x3_hddisplay_on);
#if 0
	if(!x3_hddisplay_on){
#if defined(CONFIG_MACH_RGB_CONVERTOR_SPI)
		printk("system_state:%d, first_disp_boot:%d \n",system_state,first_disp_boot);
		if((system_state != SYSTEM_BOOTING) && (first_disp_boot != TRUE)){
			ssd2825_bridge_enable();
			x3_hddisplay_on = TRUE;
			return 0;
		}
		else{
			first_disp_boot = FALSE;
		}
#else
	lgit_disp_on();
#endif
	x3_hddisplay_on = TRUE;
	}
#endif
	return 0;
}

static int x3_panel_disable(void)
{
	printk("%s --x3_hddisplay_on:%d \n", __func__,x3_hddisplay_on);
	if(x3_hddisplay_on) {
		//ssd2825_bridge_disable();
		x3_hddisplay_on = FALSE;
	}

	return 0;
}

static int x3_panel_postsuspend(void)
{
	/*
	 * if ssd2825_bridge_disable is called in early suspend,
	 * below will be NULL by x3_bridge_on,
	 * however, there's case dc resume and dc suspend
	 * without lcd late resume/early suspend,
	 * in this case, bridge is enabled in dc_resume,
	 * so, bridge needs to be disabled in dc_suspend.
	 */
	ssd2825_bridge_disable();
	return 0;
}


static int x3_hdmi_vddio_enable(void)
{
	int ret;
	if (!x3_hdmi_vddio) {
		x3_hdmi_vddio = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(x3_hdmi_vddio)) {
			ret = PTR_ERR(x3_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			x3_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(x3_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		regulator_put(x3_hdmi_vddio);
		x3_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int x3_hdmi_vddio_disable(void)
{
	if (x3_hdmi_vddio) {
		regulator_disable(x3_hdmi_vddio);
		regulator_put(x3_hdmi_vddio);
		x3_hdmi_vddio = NULL;
	}
	return 0;
}

static int x3_hdmi_enable(void)
{
	int ret;

	printk("################HDMI LS Output Enable by Heebae##############\n");
	if (!x3_hdmi_pll) {
		x3_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(x3_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			x3_hdmi_pll = NULL;
			regulator_put(x3_hdmi_reg);
			x3_hdmi_reg = NULL;
			return PTR_ERR(x3_hdmi_pll);
		}
	}
	ret = regulator_enable(x3_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int x3_hdmi_disable(void)
{
	printk("################HDMI LS Output Disable by Heebae##############\n");
	regulator_disable(x3_hdmi_pll);
	regulator_put(x3_hdmi_pll);
	x3_hdmi_pll = NULL;

	return 0;
}
static struct resource x3_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by x3_lgit_panel_init() */
		.end	= 0,	/* Filled in by x3_lgit_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource x3_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode x3_panel_modes[] = {
#if defined(CONFIG_MACH_HD7_HITACHI)
	{
		.pclk = 68000000,
		.h_ref_to_sync = 2,
		.v_ref_to_sync = 2,
		.h_sync_width = 4,
		.v_sync_width = 1,
		.h_back_porch = 62,//26,
		.v_back_porch = 3,//4,
		.h_active = 720,
		.v_active = 1280,
		.h_front_porch = 92,//136,
		.v_front_porch = 6,
	},
#elif defined(CONFIG_MACH_VIEW5_HITACHI)
	{
	.pclk = 62000000,
	.h_ref_to_sync = 2,
	.v_ref_to_sync = 2,
	.h_sync_width = 5, //4,
	.v_sync_width = 2,
	.h_back_porch = 81, //80,
	.v_back_porch = 8, //7,
	.h_active = 768,
	.v_active = 1024,
	.h_front_porch = 116,
	.v_front_porch = 24, //27,
	},
#endif
};

static struct tegra_dc_out_pin ssd2825_dc_out_pins[] = {
{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
};

#if 0
static struct tegra_dc_sd_settings x3_sd_settings = {
	.enable = 1, /* Normal mode operation */
	.use_auto_pwm = true,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = true,
	.use_vid_luma = false,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &x3_backlight_device,
};
#endif

static struct tegra_fb_data x3_fb_data = {
    .win        = 0,
#if defined(CONFIG_MACH_VIEW5_HITACHI)
    .xres        = 768,
    .yres        = 1024,
#else
    .xres        = 720,
    .yres        = 1280,
#endif
    .bits_per_pixel    = 32,//16,//24,
    .flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data x3_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1920,
	.yres		= 1080,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out x3_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk = "pll_d2_out0",

	.dcc_bus	= 3,
	.hotplug_gpio	= x3_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= x3_hdmi_enable,
	.disable	= x3_hdmi_disable,


	.postsuspend	= x3_hdmi_vddio_disable,
	.hotplug_init	= x3_hdmi_vddio_enable,

};

static struct tegra_dc_platform_data x3_disp2_pdata = {
	.flags		= 0,
	.default_out	= &x3_disp2_out,
	.fb		= &x3_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};

/*
static u8 x3_dc_out_pin_sel_config[] = {
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1,
};
*/

static void ssd2825_bridge_enable_worker(struct work_struct *work) {
	printk("ssd2825_bridge_enable_worker : %llu\n",wake_status_backup );

	if( (wake_status_backup & TEGRA_WAKE_GPIO_PC7) ||
	    (wake_status_backup & TEGRA_WAKE_GPIO_PO4)
   )
	{
		ssd2825_bridge_enable();
	}
};

static DECLARE_WORK(bridge_work, ssd2825_bridge_enable_worker);

int ssd2825_bridge_enable_queue(void)
{
	queue_work(bridge_work_queue, &bridge_work);
	return 1;
}


static struct tegra_dc_out x3_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
//                                                                       
	.height 		= 105,//                                              
	.width		= 59,//                                              

	.type		= TEGRA_DC_OUT_RGB,
	.parent_clk 	= "pll_p",//"pll_d_out0",
	//.depth		= 24,

	.modes	 	= x3_panel_modes,
	.n_modes 	= ARRAY_SIZE(x3_panel_modes),

	.out_pins	= ssd2825_dc_out_pins,
	.n_out_pins	= ARRAY_SIZE(ssd2825_dc_out_pins),

	.enable		= x3_panel_enable,
	.disable	= x3_panel_disable,
	.postsuspend	= x3_panel_postsuspend,
	.prepoweron	= ssd2825_bridge_enable_queue,
	.postpoweron	= x3_postpoweron,
};

static struct tegra_dc_platform_data x3_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &x3_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &x3_fb_data,
};
static struct nvhost_device x3_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= x3_disp1_resources,
	.num_resources	= ARRAY_SIZE(x3_disp1_resources),
	.dev = {
		.platform_data = &x3_disp1_pdata,
	},
};

static struct nvhost_device x3_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= x3_disp2_resources,
	.num_resources	= ARRAY_SIZE(x3_disp2_resources),
	.dev = {
		.platform_data = &x3_disp2_pdata,
	},
};

static struct nvmap_platform_carveout x3_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by x3_panel_init() */
		.size		= 0,	/* Filled in by x3_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data x3_nvmap_data = {
	.carveouts	= x3_carveouts,
	.nr_carveouts	= ARRAY_SIZE(x3_carveouts),
};

static struct platform_device x3_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &x3_nvmap_data,
	},
};

static struct platform_device *x3_gfx_devices[] __initdata = {
	&x3_nvmap_device,
#if 0
#if IS_EXTERNAL_PWM
	&tegra_pwfm3_device,
#else
	&tegra_pwfm2_device,
#endif
#endif
};

static struct platform_device *x3_bl_devices[]  = {
	&x3_disp1_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend x3_panel_early_suspender;

static struct rgb_bridge_gpio {
	char *name;
	int gpio;
};
static int rgb_bridge_gpio_init;

#if defined(CONFIG_MACH_RGB_CONVERTOR_SPI)
static struct rgb_bridge_gpio rgb_bridge_gpios[] = {
	{ "LCD_RGB_DE",		TEGRA_GPIO_PJ1 },
	{ "LCD_RGB_HSYNC",	TEGRA_GPIO_PJ3 },
	{ "LCD_RGB_VSYNC",	TEGRA_GPIO_PJ4 },
	{ "LCD_RGB_PCLK",	TEGRA_GPIO_PB3 },
};
#else
static int rgb_bridge_gpios[] = { };
#endif /* defined(CONFIG_MACH_RGB_CONVERTOR_SPI) */

static void x3_panel_early_suspend(struct early_suspend *h)
{
	unsigned i;
	printk("%s \n", __func__);
	for (i = 0; i < num_registered_fb; i++){
		if(1 != i)//                                                                                          
			fb_blank(registered_fb[i], FB_BLANK_POWERDOWN);
	}

	if (!rgb_bridge_gpio_init && ARRAY_SIZE(rgb_bridge_gpios) > 0) {
		for (i = 0; i < ARRAY_SIZE(rgb_bridge_gpios); i++)
			gpio_request(rgb_bridge_gpios[i].gpio,
						 rgb_bridge_gpios[i].name);
		rgb_bridge_gpio_init = 1;
	}

	for (i = 0; i < ARRAY_SIZE(rgb_bridge_gpios); i++) {
		tegra_pinmux_set_tristate(
			gpio_to_pingroup[rgb_bridge_gpios[i].gpio],
			TEGRA_TRI_TRISTATE);
		gpio_direction_input(rgb_bridge_gpios[i].gpio);
		tegra_gpio_enable(rgb_bridge_gpios[i].gpio);
	}
}

static void x3_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
	for (i = 0; i < ARRAY_SIZE(rgb_bridge_gpios); i++) {
		tegra_pinmux_set_tristate(
			gpio_to_pingroup[rgb_bridge_gpios[i].gpio],
			TEGRA_TRI_NORMAL);
		tegra_gpio_disable(rgb_bridge_gpios[i].gpio);
	}

	for (i = 0; i < num_registered_fb; i++){
		if(1 != i) // hdmi id is 1
			fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
	}
	printk("%s ended \n", __func__);
}
#endif

#if defined(CONFIG_MACH_RGB_CONVERTOR_SPI)
static struct bridge_platform_data rgb_platform_data1 = {
	.lcd_en		= TEGRA_GPIO_PY0,
	.bridge_en	= TEGRA_GPIO_PB1,
	.lcd_reset_n	= TEGRA_GPIO_PW0,
	.bridge_reset_n	= TEGRA_GPIO_PO2,

#if defined(CONFIG_SPI_SOLOMON_BRIDGE)
	.mode		= SPI_MODE_3,
	.bits_per_word	= 9,
#elif defined(CONFIG_SPI_TOSHIBA_BRIDGE)
	.mode		= SPI_MODE_3,
	.bits_per_word	= 32,
#endif
};

static struct spi_board_info lgd_spi_dev[] = {
	{
		.modalias = STR_RGB_BRIDGE,
		.bus_num = 4,
		.chip_select = 2,
		.max_speed_hz = 1000000,
#if defined(CONFIG_SPI_SOLOMON_BRIDGE)
		.mode = SPI_MODE_3, //clk pol = 1, clk phase = 1
#elif defined(CONFIG_SPI_TOSHIBA_BRIDGE)
		.mode = SPI_MODE_3, //clk pol = 1, clk phase = 1
#endif
		.platform_data = &rgb_platform_data1,

	},
};

#endif

int __init x3_panel_init(void)
{
	int err;
	struct resource *res;

	bl_output = x3_bl_output_measured;

	if (WARN_ON(ARRAY_SIZE(x3_bl_output_measured) != 256))
		pr_err("bl_output array does not have 256 elements\n");

#if !defined(CONFIG_MACH_RGB_CONVERTOR_SPI)
	gpio_init_set();
#endif

	x3_carveouts[1].base = tegra_carveout_start;
	x3_carveouts[1].size = tegra_carveout_size;

	tegra_gpio_enable(x3_hdmi_hpd);
	gpio_request(x3_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(x3_hdmi_hpd);


#ifdef CONFIG_HAS_EARLYSUSPEND
	x3_panel_early_suspender.suspend = x3_panel_early_suspend;
	x3_panel_early_suspender.resume = x3_panel_late_resume;
	x3_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&x3_panel_early_suspender);
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(x3_gfx_devices,
				ARRAY_SIZE(x3_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&x3_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

#if defined(CONFIG_MACH_RGB_CONVERTOR_SPI)
	spi_register_board_info(lgd_spi_dev, ARRAY_SIZE(lgd_spi_dev));
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&x3_disp1_device);

	res = nvhost_get_resource_byname(&x3_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	if (!err)
		err = nvhost_device_register(&x3_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif
	bridge_work_queue =
		create_singlethread_workqueue("bridge_spi_transaction");

	return err;
}
