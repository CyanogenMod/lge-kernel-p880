/*
 * arch/arm/mach-tegra/lge/board-x3-i2c-pin-define.c
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 *
 * Author: dalyong.cha@lge.com
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/spdif.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-x3.h>
#include <lge/board-x3-misc.h>
#include <lge/board-x3-audio.h>
#include <lge/board-x3-input.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>
#include <mach-tegra/fuse.h>

//                                                                 
#if defined(CONFIG_LGE_NFC_PN544)
#include <board-lge-nfc.h>
#endif
//                                                                

//                                                                             
#ifdef CONFIG_I2C_GPIO
#include <linux/i2c-gpio.h>
#endif
//                                                                             

#define SII9244_MHL_ADDRESS 0x39//0x72 MHL tx
#define SII9244A_MHL_ADDRESS 0x3D// 0x7A(TPI) Register변경 하는 i2c
#define SII9244B_MHL_ADDRESS 0x49//0x92 HDMI rx 
#define SII9244C_MHL_ADDRESS 0x64//0xC8(cbus)

/*                                                                        */
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <linux/regulator/lp8720.h>
static struct lp8720_platform_data lp8720_pdata = {
	.en_gpio_num         = TEGRA_GPIO_PBB4 /*220*/,
};
#endif
#if 0 //                                                       
#include <media/lm3559_flash_led.h>
static struct lm3559_flash_led_platform_data flash_led_data = {
	.gpio_hwen = TEGRA_GPIO_PBB3,
};
#endif


static struct i2c_board_info __initdata x3_i2c_bus0_board_info[] = {
//                                             
#if defined(CONFIG_SND_SOC_WM8994)
	{
		I2C_BOARD_INFO("wm8994", 0x1A),
		.platform_data = &wm8994_data,
	},
#elif defined(CONFIG_SND_SOC_TEGRA_MAX98088)
	{
		I2C_BOARD_INFO("max98088", 0x10),
		.platform_data = &max98088_pdata,
	},
#endif
//                                            
#if defined(MPU_SENSORS_MPU6050B1)   
        {
                I2C_BOARD_INFO("mpu6050" , 0x68),
				.irq    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
                .platform_data = &mpu6050_data,
                
        },
        {
                I2C_BOARD_INFO("ami306", 0x0E),
				.irq    = 0,
                .platform_data = &mpu_compass_data,
                
        },        
#endif  

#if defined (LGE_SENSOR_DCOMPASS)
		{
			I2C_BOARD_INFO("ami306", 0x0E),
			.irq =  TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH5),
			.platform_data = &dcompss_pdata,
		},
#endif
#if defined (LGE_SENSOR_ACCELEROMETER)
#if defined(CONFIG_MACH_X3_REV_A)
		{
			I2C_BOARD_INFO("k3dh_acc_misc", 0x19),
			.irq =	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH6),
			.platform_data = &accelerometer_pdata,
		},
#else
		{
			I2C_BOARD_INFO("k3dh_acc_misc", 0x19),
			.irq =	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2),
			.platform_data = &accelerometer_pdata,
		},
#endif
#endif
#if defined (LGE_SENSOR_GYROSCOPE)
		{
			I2C_BOARD_INFO("k3g", 0x69),
			.irq =	-1,		//	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),		// k3g only supported polling style.
			.platform_data = &gyroscope_pdata,
		},	
#endif
#if 0 //defined (CONFIG_SENSORS_APDS990X)
        {
                I2C_BOARD_INFO("apds9900",0x39),
                .irq               = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
                .platform_data = &proximity_pdata, 		//&lghdk_prox_data,
        },
#endif  

#if defined(CONFIG_SENSORS_APDS990X)
#define APDS990X_PROXIMITY_ADDR 0x39
        {
                I2C_BOARD_INFO(APDS990x_DRV_NAME, APDS990X_PROXIMITY_ADDR),
                .irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
                .platform_data	= &x3_prox_data,
        },
#endif  

//                                                                

#if defined(CONFIG_LGE_NFC_PN544)
	NFC_I2C_BOARD_INFO,
#endif
//                                                                

};


static struct i2c_board_info __initdata x3_i2c_bus1_board_info[] = {
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T1320) 
	{
        I2C_BOARD_INFO("synaptics_T1320", 0x20),
        .irq		= TEGRA_GPIO_PQ3,               
        .platform_data	= &synaptics_platform_data,
    },
#elif defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T1320_HD)
	{
		I2C_BOARD_INFO("synaptics_T1320", 0x20),
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PQ3),
		.platform_data	= &synaptics_platform_data,
	},
#elif defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T3000)
	{
		I2C_BOARD_INFO("lge_touch", 0x20),
		//.irq		= TEGRA_GPIO_PQ3,
		.platform_data	= &synaptics_t3000_platform_data,
	},
#elif defined(CONFIG_TOUCHSCREEN_SYNAPTICS_COMMON)
	{
		I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20),
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PQ3),
		.platform_data	= &synaptics_pdata,
	},
#endif

#if defined(CONFIG_MAX14526_MUIC) || defined(CONFIG_LGE_MUIC)
	{
		I2C_BOARD_INFO("muic", 0x44),
		.irq		= TEGRA_GPIO_TO_IRQ(MUIC_GPIO),
#if defined(CONFIG_LGE_MUIC)
		.platform_data	= 0,
#else
		.platform_data	= &max14526_pdata,
#endif//		
	},
#endif // CONFIG_MAX14526_MUIC
#if defined(CONFIG_ADC_TSC2007)  //                              
	{
		I2C_BOARD_INFO("tsc2007_adc", TSC2007_ADC_SLAVE_ADDR),
	},
#endif
#if !defined(CONFIG_MACH_X3_REV_A) 
#define LM3533_BACKLIGHT_ADDRESS 0x36
	{
		I2C_BOARD_INFO("lm3533_bl", LM3533_BACKLIGHT_ADDRESS),
		.platform_data = &lm3533_pdata,
	},
#endif // CONFIG_MACH_HD7_HITACHI
};

static struct i2c_board_info __initdata x3_i2c_bus2_board_info[] = {

#if defined(CONFIG_MACH_X3_REV_A)

#if defined(CONFIG_MACH_BACKLIGHT_38_ADDR)
#define LM3533_BACKLIGHT_ADDRESS 0x38 //0x38
#else
#define LM3533_BACKLIGHT_ADDRESS 0x36 //0x38
#endif

	{
		I2C_BOARD_INFO("lm3533_bl", LM3533_BACKLIGHT_ADDRESS),
		.platform_data = &lm3533_pdata,
	},
#endif //CONFIG_MACH_HD5_LGD

/*                                                                        */
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
	{
		I2C_BOARD_INFO(LP8720_I2C_NAME,  LP8720_I2C_ADDR),
		.platform_data =&lp8720_pdata,
	},
#endif
#if 0 //                                                       
  {
		I2C_BOARD_INFO("lm3559_flash_led",  0x53),
		.platform_data  = &flash_led_data,
  },
#endif
};

static struct i2c_board_info __initdata x3_i2c_bus4_board_info[] = {
#if defined(CONFIG_BATTERY_MAX17040)	
#define MAX17040_SLAVE_ADDR	0x36
	{
		I2C_BOARD_INFO("max17040", MAX17040_SLAVE_ADDR),
		//.platform_data	= &max17040_pdata,
	},
#endif	

#if defined(CONFIG_BATTERY_MAX17043)	//                                        
	{
		I2C_BOARD_INFO("max17043", MAX17043_SLAVE_ADDR),
		.irq                 = TEGRA_GPIO_TO_IRQ(MAX17043_GAUGE_INT),
	},

#endif //                                      

#if defined(CONFIG_MAX8971_CHARGER) //                                       
	//                                          
	{
		I2C_BOARD_INFO("max8971", MAX8971_SLAVE_ADDR),
		.platform_data	= &max8971_data,
		.irq		= TEGRA_GPIO_TO_IRQ(MAX8971_IRQB),		
	},
#endif	//                                     

//                               
#if defined(CONFIG_TSPDRV)
{
                I2C_BOARD_INFO(TSPDRV_I2C_DEVICE_NAME, TSPDRV_I2C_SLAVE_ADDR),
                .type = TSPDRV_I2C_DEVICE_NAME,
                .platform_data = &tspdrv_i2c_pdata,
},
#endif
//                               

};


static struct i2c_board_info __initdata x3_i2c_bus5_board_info[] = {
#if defined(CONFIG_MHL_TX_SII9244)
	{
		I2C_BOARD_INFO("SII9244", SII9244_MHL_ADDRESS),
	},
	{
		I2C_BOARD_INFO("SII9244A", SII9244A_MHL_ADDRESS),
	},
	{
		I2C_BOARD_INFO("SII9244B", SII9244B_MHL_ADDRESS),
	},
	{
		I2C_BOARD_INFO("SII9244C", SII9244C_MHL_ADDRESS),
	},
#endif 
};

static void __init x3_i2c_dev_init(void)
{
	i2c_register_board_info(0, x3_i2c_bus0_board_info,
			ARRAY_SIZE(x3_i2c_bus0_board_info));
	i2c_register_board_info(1, x3_i2c_bus1_board_info,
			ARRAY_SIZE(x3_i2c_bus1_board_info));
	i2c_register_board_info(2, x3_i2c_bus2_board_info, 
			ARRAY_SIZE(x3_i2c_bus2_board_info));
	i2c_register_board_info(4, x3_i2c_bus4_board_info, 
			ARRAY_SIZE(x3_i2c_bus4_board_info));
//                                                                                
	i2c_register_board_info(5, x3_i2c_bus5_board_info,
			ARRAY_SIZE(x3_i2c_bus5_board_info));
//                                                                                
}

static struct tegra_i2c_platform_data x3_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },		/* fastmode */
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 }, //hyojin.an camea camp 100K -> 400K
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data x3_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};
//                                                                             
#ifdef CONFIG_I2C_GPIO
static struct i2c_gpio_platform_data x3_gpio_i2c_platform_data = {
	.sda_pin		= TEGRA_GPIO_PQ7,
	.sda_is_open_drain	= 1,
	.scl_pin		= TEGRA_GPIO_PQ6,
	.scl_is_open_drain	= 1,	
	.scl_is_output_only = 1,
	.udelay			= 5,	
};
#endif
//                                                                             
void __init x3_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &x3_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &x3_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &x3_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &x3_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &x3_i2c5_platform_data;
//                                                                              
#ifdef CONFIG_I2C_GPIO
	tegra_gpio_i2c.dev.platform_data = &x3_gpio_i2c_platform_data;
#endif
//                                                                             
	x3_i2c_dev_init();
//                                                                             
#ifdef CONFIG_I2C_GPIO
	platform_device_register(&tegra_gpio_i2c);
#endif
//                                                                             
	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

