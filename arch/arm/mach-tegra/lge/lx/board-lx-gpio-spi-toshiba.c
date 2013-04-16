/*
 * arch/arm/mach-tegra/lge/board-lx-gpio-spi-toshiba.c
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
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>

//#include "gpio-names.h"
#include <mach-tegra/gpio-names.h>
#include <lge/board-lx-gpio-spi-toshiba.h>

//#define GPIO_SETTING_CHANGE


static int spi_sclk		= TEGRA_GPIO_PZ4;	//LCD_SCK
static int spi_cs		= TEGRA_GPIO_PN4;	//LCD_CS0_N
static int lcd_reset_n		= TEGRA_GPIO_PW0;	//LCD_CS1_N
static int spi_mosi		= TEGRA_GPIO_PZ2; //LCD_SDIN
static int spi_miso		= TEGRA_GPIO_PN5; //LCD SDOUT	//bridge chip out to AP.
static int dsi_bridge_en	= TEGRA_GPIO_PV6;	//brindge enable

void gpio_init_set(void)
{
	int ret;

	printk("gpio_init_set start!!\n");
	
	

	ret = gpio_request(dsi_bridge_en, "pv6");
	if (ret < 0){
		gpio_free(dsi_bridge_en);
		return ret;
	}
	ret=gpio_direction_output(dsi_bridge_en, 0);
	if (ret < 0){
		gpio_free(dsi_bridge_en);
		return ret;
	}
	else
		tegra_gpio_enable(dsi_bridge_en);

/*-
		ret = gpio_request(dsi_osc_en, "panel_osc_en");
	if (ret < 0)
		printk(KERN_INFO "%s: gpio_request failed with %d \n", __func__, ret);

	ret=gpio_direction_output(dsi_osc_en, 0);
	
	tegra_gpio_enable(dsi_osc_en);
	
//*/

	ret = gpio_request(lcd_reset_n, "pw0");
	if (ret < 0){
		gpio_free(lcd_reset_n);
		return ret;
	}
	ret=gpio_direction_output(lcd_reset_n, 0);
	if (ret < 0){
		gpio_free(lcd_reset_n);
		return ret;
	}
	else
		tegra_gpio_enable(lcd_reset_n);

	ret = gpio_request(spi_cs, "pn4");
	if (ret < 0){
		gpio_free(spi_cs);
		return ret;
	}
	ret=gpio_direction_output(spi_cs, 1);
	if (ret < 0){
		gpio_free(spi_cs);
		return ret;
	}
	else
		tegra_gpio_enable(spi_cs);

/*----------------------------------------------*/

	ret = gpio_request(spi_sclk, "pz4");
	if (ret < 0){
		gpio_free(spi_sclk);
		return ret;
	}
	ret=gpio_direction_output(spi_sclk, 1);
	if (ret < 0){
		gpio_free(spi_sclk);
		return ret;
	}
	else
		tegra_gpio_enable(spi_sclk);


/*----------------------------------------------*/

	ret = gpio_request(spi_mosi, "pz2");
	if (ret < 0){
		gpio_free(spi_mosi);
		return ret;
	}
#if defined(GPIO_SETTING_CHANGE)
	ret=gpio_direction_input(spi_mosi);
#else
	ret=gpio_direction_output(spi_mosi, 1);
#endif
	if (ret < 0){
		gpio_free(spi_mosi);
		return ret;
	}
	else
		tegra_gpio_enable(spi_mosi);

/*----------------------------------------------*/
	ret = gpio_request(spi_miso, "pn5");
	if (ret < 0){
		gpio_free(spi_miso);
		return ret;
	}
#if defined(GPIO_SETTING_CHANGE)
	ret=gpio_direction_output(spi_miso, 1);
#else
	ret=gpio_direction_input(spi_miso);
#endif

	if (ret < 0){
		gpio_free(spi_miso);
		return ret;
	}
	else
		tegra_gpio_enable(spi_miso);


#if 0
/*----------------------------------------------*/
	ret = gpio_request(dsi_bridge_vdd_n, "pm5");
	if (ret < 0){
		gpio_free(dsi_bridge_vdd_n);
		return ret;
	}
	ret=gpio_direction_output(dsi_bridge_vdd_n, 1);
	if (ret < 0){
		gpio_free(dsi_bridge_vdd_n);
		return ret;
	}
	else
		tegra_gpio_enable(dsi_bridge_vdd_n);
/*----------------------------------------------*/

	ret = gpio_request(dsi_bridge_vio_n, "pm4");
	if (ret < 0){
		gpio_free(dsi_bridge_vio_n);
		return ret;
	}
	ret=gpio_direction_output(dsi_bridge_vio_n, 1);
	if (ret < 0){
		gpio_free(dsi_bridge_vio_n);
		return ret;
	}
	else
		tegra_gpio_enable(dsi_bridge_vio_n);
/*----------------------------------------------*/


	ret = gpio_request(bridge_osc_n, "pm6");
	if (ret < 0){
		gpio_free(bridge_osc_n);
		return ret;
	}
	ret=gpio_direction_output(bridge_osc_n, 1);
	if (ret < 0){
		gpio_free(bridge_osc_n);
		return ret;
	}
	else
		tegra_gpio_enable(bridge_osc_n);
	
/*----------------------------------------------*/
#endif
	printk("gpio_init_set complete!!\n");
	
}


/*================================================================================
      
      Function : lgit_lcd_reset
      Make : dalyong.cha
      Modify :
      Date : 2011 .5 .25
      Description : LCD RESET
      Parameter : None
      Return : Node
      
================================================================================*/
static void lgit_lcd_reset(void)
{
	mdelay(15);

    gpio_set_value(lcd_reset_n,1);
    mdelay(5);
	gpio_set_value(lcd_reset_n,0);
    mdelay(5);
    gpio_set_value(lcd_reset_n,1);
    mdelay(15);

}

#define T_cycle_value_us		90 

//90

/*================================================================================
      
      Function : spi_write_byte
      Make : dalyong.cha
      Modify :
      Date : 2011 .5 .25
      Description : 실제 SPI emulation part
      Parameter : None
      Return : Node
      
================================================================================*/
static void spi_write_byte(unsigned int val)
{
	int i;
	unsigned int shift = 1;

//	printk(" >>> spi_write_val : %#x \n",  val);


	for (i = 0; i < 32; i++) {


#if defined(GPIO_SETTING_CHANGE)

		if (val & shift_bit32[i])
			gpio_set_value(spi_miso, 1);
		else
			gpio_set_value(spi_miso, 0);
#else
		if (val & shift_bit32[i])
			gpio_set_value(spi_mosi, 1);
		else
			gpio_set_value(spi_mosi, 0);
#endif	
		gpio_set_value(spi_sclk, 0);
		udelay(T_cycle_value_us*4); // ndelay(150); //
		
		gpio_set_value(spi_sclk, 1);
		udelay(T_cycle_value_us*4);  // ndelay(150); //
	}

	// from 0 -> 1 sampling to 1 -> 0 sampling

}



/*================================================================================
      
      Function : send_gpio_to_spi_emulation
      Make : dalyong.cha
      Modify :
      Date : 2011 .5 .25
      Description : SSD2825 Register setting part
      Parameter : None
      Return : Node
      
================================================================================*/
static int send_gpio_to_spi_emulation(unsigned int reg)
{
	//reg = reg | 0x00010000;
	spi_write_byte(reg);

	return 0;

}

/*================================================================================
      
      Function : set_gpio_to_spi_emulation
      Make : dalyong.cha
      Modify :
      Date : 2011 .5 .25
      Description : SSD2825 Data setting part
      Parameter : None
      Return : Node
      
================================================================================*/
static int set_gpio_to_spi_emulation(unsigned int data)
{
	unsigned int cmd_read = 0x00010000;
//	data = data | cmd_read;
	spi_write_byte(data);

	return 0;

}

/*================================================================================
      
      Function : gpio_emulation_spi
      Make : dalyong.cha
      Modify :
      Date : 2011 .5 .25
      Description : register와 data를 분리하는 부분
      Parameter : None
      Return : Node
      
================================================================================*/
static void gpio_emulation_spi(struct spi_cmd_data *cmds, int cnt)
{
	struct spi_cmd_data *cm;
	int i,j;
	unsigned int cmd, data, sending_packit, loop;

	cm = cmds;


	if ( !spi_cs )
		printk( " >>> pin error - spi_cs \n" );
	if ( !spi_sclk )
		printk( " >>> pin error - spi_sclk \n" );
	if ( !spi_mosi )
		printk( " >>> pin error - spi_mosi \n" );
	if ( !spi_miso )
		printk( " >>> pin error - spi_miso \n" );

//	printk(" >>> array cnt : %d \n", cnt);

	for (i = 0; i < cnt; i++) {

		if (cm->payload || cm->dtype) 
		{
			/* Enable the Chip Select - low */
			cmd = 0; data=0; sending_packit=0;
		
			cmd = cm->dtype;
			data = cm->payload[0];
			sending_packit = cmd | data;	
			loop = (int)cm->dlen;

			
			gpio_set_value(spi_cs, 0);
			udelay(33); // ndelay(200);// 

			//printk(" >>> inner array cnt : %d \n",loop);
			
			for( j = 0 ; j < loop ; j+=2 ){

				
				if( j == 0 )
					set_gpio_to_spi_emulation(sending_packit);	
				else{
					sending_packit = 0; cmd=0; data=0;
					cmd = cm->payload[j-1];
					data = cm->payload[j];
					sending_packit = cmd | data;
					
					send_gpio_to_spi_emulation(sending_packit);
				}
			}

			udelay(33); //ndelay(200); //
			gpio_set_value(spi_cs, 1);	
			udelay(T_cycle_value_us*8);	//ndelay(200 * 8); // 
		}
		
		if (cm->wait)
			msleep(cm->wait);
		cm++;
	}
	

}

/*================================================================================
      
      Function : gpio_spi_init
      Make : dalyong.cha
      Modify :
      Date : 2011 .5 .25
      Description : SPI 초기화
      Parameter : None
      Return : Node
      
================================================================================*/
static void gpio_spi_init(void)
{

	gpio_set_value(spi_sclk, 0); // change to 1
	gpio_set_value(spi_mosi, 0);
	gpio_set_value(spi_miso, 0);
	gpio_set_value(spi_cs, 1);
}



void lgit_disp_off(void)
{
	//gpio_emulation_spi(lgit_power_off_set, ARRAY_SIZE(lgit_power_off_set));
}



void lgit_disp_on(void)
{

	mdelay(100);
	

	gpio_spi_init();

	gpio_set_value(dsi_bridge_en, 1);

	//udelay(200);
	
	//gpio_set_value(lcd_reset_n, 1);
	 //mdelay(5);
	//gpio_set_value(lcd_reset_n, 0);
	 mdelay(5);
	gpio_set_value(lcd_reset_n, 1);
	 mdelay(15);

	 //gpio_set_value(dsi_osc_en, 1);

	 mdelay(100);
	 
	//printk("==================== ssd2825 driver LCD on start ==================== \n");
	gpio_emulation_spi(init_toshiba_bridge_sequence, ARRAY_SIZE(init_toshiba_bridge_sequence));
//	lgit_lcd_reset();
	
	//printk("===== mipi LCD on ===== \n");
#if defined(CONFIG_MACH_LX_HD_HITACHI)
	gpio_emulation_spi(toshiba_hitachi_power_on_set, ARRAY_SIZE(toshiba_hitachi_power_on_set));

#elif defined(CONFIG_MACH_LX_HD_LGD)
	gpio_emulation_spi(toshiba_lgd_power_on_set, ARRAY_SIZE(toshiba_lgd_power_on_set));
	gpio_emulation_spi(toshiba_lgd_power_supply_set, ARRAY_SIZE(toshiba_lgd_power_supply_set));
#endif

//                                  
	//printk("===== LGD leave sleep ===== \n");
	

	gpio_emulation_spi(toshiba_lgd_sleep_out_set, ARRAY_SIZE(toshiba_lgd_sleep_out_set));

	
	//printk("===== toshiba HS set ===== \n");
	gpio_emulation_spi(toshiba_end_of_init_sequence, ARRAY_SIZE(toshiba_end_of_init_sequence));
	//mdelay(999);

}

