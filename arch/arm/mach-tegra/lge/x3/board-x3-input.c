/*
 * arch/arm/mach-tegra/lge/board-x3-input.c
 *
 * Copyright (C) 2011 LG Electronics, Inc.
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
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-x3.h>
#include <lge/board-x3-input.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>


#if defined( LGE_SENSOR_GYROSCOPE)
#include <linux/k3g.h>
#endif

#if defined( LGE_SENSOR_DCOMPASS)
#include <linux/ami306.h>
#endif

#if defined (LGE_SENSOR_ACCELEROMETER)
#include <linux/k3dh.h>
#endif

#if defined (CONFIG_SENSORS_APDS990X)
#include <linux/apds990x.h> 
#endif

#if defined (CONFIG_TOUCHSCREEN_SYNAPTICS_T1320)	
struct synaptics_i2c_rmi_platform_data synaptics_platform_data[] = {
	{
		.version	= 0x0,
		.irqflags	= IRQF_TRIGGER_FALLING,
	}
};
#elif defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T1320_HD) 

static int synaptics_t1320_gpio_init(void)
{
	int ret = 0;

	ret = gpio_request(TOUCH_3V0_EN, "TOUCH_3V0_EN");
	if (ret < 0){		
		goto failed_second;
	}
	ret = gpio_direction_output(TOUCH_3V0_EN, 0);
	if (ret < 0){
		goto failed_second;
	}
	else {
		tegra_gpio_enable(TOUCH_3V0_EN);
	}
	
	ret = gpio_request(TOUCH_1V8_EN, "TOUCH_1V8_EN");
	if (ret < 0){		
		goto failed_first;
	}
	ret = gpio_direction_output(TOUCH_1V8_EN, 0);
	if (ret < 0){
		goto failed_first;
	}
	else {
		tegra_gpio_enable(TOUCH_1V8_EN);
	}
	
	printk("==> %s : successful\n", __func__);

	return 0;

failed_second:
	printk(KERN_ERR "%s(%d) failed\n", __func__, __LINE__);
	gpio_free(TOUCH_3V0_EN);
failed_first:
	printk(KERN_ERR "%s(%d) failed\n", __func__, __LINE__);
	gpio_free(TOUCH_1V8_EN);
	return ret;
}

static int synaptics_t1320_power_on(int on, bool log_on)
{
	if (log_on) {
		printk("==> %s : %d\n", __func__, on);
	}

	if (on) {
		gpio_set_value(TOUCH_3V0_EN, 1);	
		//mdelay(20);
		gpio_set_value(TOUCH_1V8_EN, 1);
		mdelay(400);
	}
	else {
		gpio_set_value(TOUCH_3V0_EN, 0);
		mdelay(20);
		gpio_set_value(TOUCH_1V8_EN, 0);
		mdelay(20);
	}

	return 1;
}

struct synaptics_ts_platform_data synaptics_platform_data = {
	.use_irq		= 1,
	.irqflags		= IRQF_TRIGGER_FALLING,
	.i2c_int_gpio		= TEGRA_GPIO_PQ3,
	.power			= synaptics_t1320_power_on,
	.gpio_init		= synaptics_t1320_gpio_init,
	.ic_booting_delay	= 400,		/* ms */
	.report_period		= 12500000, 	/* 12.5 msec */
	.num_of_finger		= 10,
	.num_of_button		= 3,
	.button[0]		= KEY_MENU,
	.button[1]		= KEY_HOME,
	.button[2]		= KEY_BACK,
	.x_max			= 1110,
	.y_max			= 1973,
	.fw_ver 		= 1,
	.palm_threshold 	= 0,
	.delta_pos_threshold	= 0,
};
#elif defined(CONFIG_TOUCHSCREEN_SYNAPTICS_T3000)

struct p940_synaptics_platform_data {
	u32 gpio;
	u32 reset;
	unsigned long irqflags;
	int (*power)(int on);	
};

static int touch_3_3v = TEGRA_GPIO_PD3;
//static int touch_1_8v = TEGRA_GPIO_PD3;
int touch_power_control(int on)
{
	printk("==>synaptics_t3000_power_on =%d",on);
	if(on)
	{
		//gpio_set_value(TEGRA_GPIO_PC6,1);
		//mdelay(20);
		gpio_set_value(touch_3_3v,1);	
	}
	else
	{
		gpio_set_value(touch_3_3v,0);
		//mdelay(20);
		//gpio_set_value(TEGRA_GPIO_PC6,0);
	}
	
	return 1;
}

static struct p940_synaptics_platform_data synaptics_t3000_platform_data = {
	.gpio		= TEGRA_GPIO_PQ3,
	.reset		= TEGRA_GPIO_PV6,//OMAP4_GPIO_TOUCH_RESET,
	.irqflags	= IRQF_TRIGGER_FALLING,
	.power		= &touch_power_control,
};

//                                                           
#elif defined (CONFIG_TOUCHSCREEN_SYNAPTICS_COMMON)

static int touch_1V8_en = TEGRA_GPIO_PX4;
static int touch_3V0_en = TEGRA_GPIO_PQ1;

int touch_power_control(int on)
{
	printk("[TOUCH]==>synaptics_touch_ic_power_on =%d\n",on);

	if (on) {
		gpio_set_value(touch_3V0_en, 1);	
		//mdelay(20);
		gpio_set_value(touch_1V8_en, 1);
		//mdelay(100);
	}
	else {
		gpio_set_value(touch_1V8_en, 0);
		mdelay(20);
		gpio_set_value(touch_3V0_en, 0);
		mdelay(20);
	}
	
	return 1;
}

int touch_power_init(void)
{
	int ret = 0;

	if(x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_C)
	{
		touch_1V8_en = TEGRA_GPIO_PX4;
		touch_3V0_en = TEGRA_GPIO_PQ1;		
	}
	else
	{
		touch_1V8_en = TEGRA_GPIO_PX4;
		touch_3V0_en = TEGRA_GPIO_PD3;
	}

	printk("[TOUCH] %s [%d][%d]\n", __func__, touch_1V8_en, touch_3V0_en);

	ret = gpio_request(touch_3V0_en, "TOUCH_3V0_EN");
	if (ret < 0){		
		goto failed_second;
	}
	ret = gpio_direction_output(touch_3V0_en, 0);
	if (ret < 0){
		goto failed_second;
	}
	else {
		tegra_gpio_enable(touch_3V0_en);
	}
	
	ret = gpio_request(touch_1V8_en, "TOUCH_1V8_EN");
	if (ret < 0){		
		goto failed_first;
	}
	ret = gpio_direction_output(touch_1V8_en, 0);
	if (ret < 0){
		goto failed_first;
	}
	else {
		tegra_gpio_enable(touch_1V8_en);
	}
	
       	printk("[TOUCH] %s : successful\n", __func__);

	return 0;

failed_second:
	printk(KERN_ERR "%s(%d) failed\n", __func__, __LINE__);
	gpio_free(touch_3V0_en);
failed_first:
	printk(KERN_ERR "%s(%d) failed\n", __func__, __LINE__);
	gpio_free(touch_1V8_en);
	return ret;
}

struct touch_device_caps touch_caps = {
	.button_support 		= 1,
	.number_of_button 		= 3,
	.button_name 			= {KEY_BACK,KEY_HOME,KEY_MENU},
	.button_margin 			= 10,
	.is_width_supported 	= 1,
	.is_pressure_supported 	= 1,
	.is_id_supported		= 1,
	.max_width 				= 15,
	.max_pressure 			= 0xFF,
	.max_id					= 10,
	.lcd_x					= 720,
	.lcd_y					= 1280,
	.x_max					= 1440,//1100,
	.y_max					= 2780,//1900,

};

struct touch_operation_role touch_role = {
	.operation_mode 	= INTERRUPT_MODE,
	.key_type			= TOUCH_SOFT_KEY, /* rev.a : hard_key, rev.b : soft_key */
	.report_mode			= 0,
	.delta_pos_threshold 	= 0,
	.orientation 		= 0,
	.booting_delay 			= 200,
	.reset_delay		= 20,
	.report_period		= 10000000, 	/* 12.5 msec -> 10.0 msec(X3) */
	.suspend_pwr		= POWER_OFF,
	.resume_pwr		= POWER_ON,
	.jitter_filter_enable	= 1,
	.jitter_curr_ratio		= 28,
	.accuracy_filter_enable	= 1,
	.irqflags 				= IRQF_TRIGGER_FALLING,
};

struct touch_power_module touch_pwr = {
	.use_regulator	= 0,
	.power		= touch_power_control,
	.gpio_init	= touch_power_init,
};

struct touch_platform_data  synaptics_pdata = {
	.int_pin		= TEGRA_GPIO_PQ3,

	.reset_pin		= TEGRA_GPIO_PO1,

	.maker			= "Synaptics",
	.caps	= &touch_caps,
	.role	= &touch_role,
	.pwr	= &touch_pwr,
};
#endif

//                                                                
#if defined(CONFIG_SENSORS_APDS990X)

static int prox_ldo_en = TEGRA_GPIO_PX1;

uint32_t prox_pwr_mask = 0; //don't need but

static s32 x3_apds990x_power_init(void)
{
        s32 ret = 0;

	switch(x3_get_hw_rev_pcb_version())
	{
	case hw_rev_pcb_type_B :
	case hw_rev_pcb_type_C :
		prox_ldo_en = TEGRA_GPIO_PD4;
		break;
	case hw_rev_pcb_type_D :
		prox_ldo_en = TEGRA_GPIO_PR3;
		break;
	default :
		prox_ldo_en = TEGRA_GPIO_PX1;
		break;
	}
		
	printk(KERN_INFO"%s start!![%d][%d] \n",__func__,__LINE__, prox_ldo_en);
	//PROX  LDO Enable : 2.6V & 1.8V
        tegra_gpio_enable(prox_ldo_en);
        ret = gpio_request(prox_ldo_en, "PROX_LDO_EN");
        if (ret < 0) {
                printk("PROX_Sensor[1] : Fail to request Sensor LDO enabling\n");
                return ret;
        }
        ret=gpio_direction_output(prox_ldo_en, 0);
        if (ret < 0)
        {
                printk("PROX_Sensor[2] : Fail to direct Sensor LDO enabling\n");
                gpio_free(prox_ldo_en);
                return ret;
        }       
	
	/* PORXI_LDO_EN */
	//tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT7, TEGRA_PUPD_NORMAL);
	
	printk(KERN_DEBUG "PROX_Sensor[3] : Proximity LDO Enable before toggle "
		"at BOARD: %d, Line[%d]\n", 
	gpio_get_value(prox_ldo_en), __LINE__);
}

/*
int sensor_proximity_power_set(unsigned char onoff)
{
	gpio_set_value(prox_ldo_en, onoff);
	printk(KERN_DEBUG "PROX[%s] Proximity LDO Enable before toggle "
		"at BOARD: %d, Line[%d]\n", 
	__func__,gpio_get_value(prox_ldo_en), __LINE__);
	return 0;	
}
*/
static int prox_common_power_set(unsigned char onoff, int sensor)
{
//	int ret = -EINVAL;

	gpio_set_value(prox_ldo_en,onoff);
	
	printk(" prox_common_power_set : %d, Line[%d]\n",	gpio_get_value(prox_ldo_en),__LINE__);

	if (onoff)
		prox_pwr_mask |= sensor;
	else
		prox_pwr_mask &= ~sensor;	

	return 0;	
}

int prox_power_on(int sensor)
{
    int ret = 0;
	ret = prox_common_power_set(1, sensor);
	return ret;
}
int prox_power_off(int sensor)
{
    int ret = 0;
	ret = prox_common_power_set(0, sensor);
	return ret;	
}

static s32 x3_apds990x_irq_set(void)
{
	s32 ret = 0;
	printk(KERN_INFO"[SENSOR-APDS] %s start(%d, %d)!!!!![%d] \n",__func__, TEGRA_GPIO_PK2, TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2), __LINE__);

	//PROX INT
        tegra_gpio_enable(TEGRA_GPIO_PK2);
	
        ret = gpio_request(TEGRA_GPIO_PK2, "PROX_INT");
        if (ret < 0)
        {
                printk("Sensor[1] : Fail to request Proximity INT enabling\n");
                return ret;
        }
        ret=gpio_direction_input(TEGRA_GPIO_PK2);
        if (ret < 0)
        {
                printk("Sensor[2] : Fail to request Proximity INT enabling\n");
                gpio_free(TEGRA_GPIO_PK2);
                return ret;
        }
}

#endif  
//                                                                                 

struct lge_sensor_int_gpio{
	s16 num;
	const char *name;
	//unsigned config;
};





#if defined(MPU_SENSORS_MPU6050B1)

static int sensors_ldo_en = TEGRA_GPIO_PD0;

static s32 x3_mpuirq_init(void)
{
	s32 ret = 0;

//SENSOR_LDO_EN pin Enable for GYRO 3.0V

	if(x3_get_hw_rev_pcb_version() < hw_rev_pcb_type_D)
	{
	tegra_gpio_enable(sensors_ldo_en);

	ret = gpio_request(sensors_ldo_en, "SENSOR_LDO_EN");
	if (ret < 0)
	{
		printk("Sensor[1] : Fail to request Sensor LDO enabling\n");
		return ret;
	}

	ret=gpio_direction_output(sensors_ldo_en, 1);
	if (ret < 0)
	{
		printk("Sensor[2] : Fail to direct Sensor LDO enabling\n");
		gpio_free(sensors_ldo_en);
		return ret;
	}		
	
	gpio_set_value(sensors_ldo_en,1);		//on
	printk(" mpu-- Sensor_DEBUG[3]: Sensor LDO Enable before toggle at BOARD: %d, Line[%d]\n", gpio_get_value(sensors_ldo_en),__LINE__);
	}

//SENSOR INT
	tegra_gpio_enable(TEGRA_GPIO_PH4);

	ret = gpio_request(TEGRA_GPIO_PH4, "SENSOR_INT");
	if (ret < 0)
	{
		printk("Sensor : Fail to request MOTION INT enabling\n");
		return ret;
	}

	ret=gpio_direction_input(TEGRA_GPIO_PH4);
	if (ret < 0)
	{
		printk("Sensor : Fail to request MOTION INT enabling\n");
		gpio_free(TEGRA_GPIO_PH4);
		return ret;
	}

}

#endif  //MPU_SENSORS_MPU6050B1

//                                                       
#if defined (CONFIG_SENSORS_APDS990X)

struct apds990x_proximity_platform_data x3_prox_data = {
        .gpio		= TEGRA_GPIO_PK2,
      	.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
        .irqflags	= IRQF_TRIGGER_FALLING,
        .power_on 	= prox_power_on,
	.power_off 	= prox_power_off,
};
#endif

//for Sensor of Rev.D Power Board
#if defined(MPU_SENSORS_MPU6050B1)

#include <linux/mpu.h> 

struct mpu_platform_data mpu6050_data = {
        .int_config      = 0x10,
        .level_shifter 	 = 0,
        .orientation     = {  0,  1,  0,
                              -1,  0,  0,
                              0,  0, 1},

};

/* accel */
/*
struct ext_slave_platform_data inv_mpu6050_accel_data = {
                .adapt_num		  = 0,
                .bus              = EXT_SLAVE_BUS_SECONDARY,
                .orientation      = { 0,  1,  0,
                                      -1,  0,  0,
                                      0,  0, 1 },
};
*/  //include in mpu6050

/* compass */
struct ext_slave_platform_data mpu_compass_data = {
                .address		  = 0x0E,
                .adapt_num		  = 0,
                .bus              = EXT_SLAVE_BUS_PRIMARY,
                .orientation      = { 1,  0,  0,
                                      0,  1,  0,
                                      0,  0,  1},
};

//for Sensor of Rev.D Power Board

#endif






int __init x3_sensor_input_init(void)
{

#ifdef MPU_SENSORS_MPU6050B1
	x3_mpuirq_init();
#endif
		

#ifdef CONFIG_SENSORS_APDS990X
	x3_apds990x_power_init();
	x3_apds990x_irq_set();
#endif	

}
	

//                                                                               
