/*
 * arch/arm/mach-tegra/board-x3-pinmux_rev_c.c
 *
 * Copyright (C) 2010 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/pinmux.h>
#include <mach-tegra/board.h>
#include <lge/board-x3.h>
#include <mach-tegra/gpio-names.h>

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

#define CEC_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}


static __initdata struct tegra_drive_pingroup_config x3_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SET_DRIVE(ATA, DISABLE, DISABLE, DIV_1, 31, 31, FAST, FAST) */
//	SET_DRIVE(DAP2, 	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* All I2C pins are driven to maximum drive strength */
	/* GEN1 I2C */
	SET_DRIVE(DBG,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* GEN2 I2C */
	SET_DRIVE(AT5,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* CAM I2C */
	SET_DRIVE(GME,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* DDC I2C */
	SET_DRIVE(DDC,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* PWR_I2C */
	SET_DRIVE(AO1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* UART3 */
	SET_DRIVE(UART3,	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* GMI_CS0_N, GMI_WP_N */
	SET_DRIVE(AT3,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
};

static __initdata struct tegra_pingroup_config x3_pinmux[] = {

	/* SDMMC3 pinmux micro SD*/
	DEFAULT_PINMUX(SDMMC3_CLK,      SDMMC3,          NORMAL,    NORMAL,     INPUT),		//MICROSD_CLK
	DEFAULT_PINMUX(SDMMC3_CMD,      SDMMC3,          PULL_UP,    NORMAL,     INPUT),	//MICROSD_CMD
	DEFAULT_PINMUX(SDMMC3_DAT0,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),	//MICROSD_DAT0
	DEFAULT_PINMUX(SDMMC3_DAT1,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),	//MICROSD_DAT1
	DEFAULT_PINMUX(SDMMC3_DAT2,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),	//MICROSD_DAT2
	DEFAULT_PINMUX(SDMMC3_DAT3,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),	//MICROSD_DAT3

	/* SDMMC3 pinmux */
	//DEFAULT_PINMUX(SDMMC3_DAT4,     SDMMC3,          PULL_UP,    TRISTATE,  INPUT),
//	DEFAULT_PINMUX(SDMMC3_DAT4,     RSVD,          PULL_UP,    NORMAL,  OUTPUT),
//	DEFAULT_PINMUX(SDMMC3_DAT5,     RSVD,          PULL_UP,    TRISTATE,  OUTPUT),
//	DEFAULT_PINMUX(SDMMC3_DAT6,     SDMMC3,          PULL_UP,    TRISTATE,  INPUT),
//	DEFAULT_PINMUX(SDMMC3_DAT6,     SDMMC3,          NORMAL,     NORMAL,    OUTPUT),   // TOUCH_LDO_EN
//	DEFAULT_PINMUX(SDMMC3_DAT7,     RSVD,          PULL_UP,    TRISTATE,  OUTPUT),
#if 0
	/* eMMC */
	DEFAULT_PINMUX(SDMMC4_CLK,      VI_ALT3,         PULL_UP,    NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_CMD,      I2C3,            PULL_UP,    TRISTATE,  INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          NORMAL,     NORMAL,    INPUT),
	DEFAULT_PINMUX(SDMMC4_RST_N,    POPSDMMC4,       NORMAL,     NORMAL,    INPUT),
#endif
	/* SDMMC4 pinmux */
	DEFAULT_PINMUX(SDMMC4_CLK,      SDMMC4,          NORMAL,    NORMAL,     INPUT),		//eMMC_CLK
	DEFAULT_PINMUX(SDMMC4_CMD,      SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_CMD
	DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT0
	DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT1
	DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT2
	DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT3
	DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT4
	DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT5
	DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT6
	DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),	//eMMC_DAT7
	DEFAULT_PINMUX(SDMMC4_RST_N,    RSVD1,           PULL_DOWN,    NORMAL,     INPUT),	//eMMC_RST_N

	/* I2C1 pinmux */
	I2C_PINMUX(GEN1_I2C_SCL,        I2C1,            NORMAL,     NORMAL,    INPUT,	DEFAULT,	DEFAULT),
	I2C_PINMUX(GEN1_I2C_SDA,        I2C1,            NORMAL,     NORMAL,    INPUT,	DEFAULT,	DEFAULT),

//                                             
	/* I2C2 pinmux */
#if 0//                                                    
	I2C_PINMUX(GEN2_I2C_SCL,        I2C2,            NORMAL,	NORMAL,		INPUT,	DEFAULT,	DEFAULT),//MUIC_INIT
	I2C_PINMUX(GEN2_I2C_SDA,        I2C2,            NORMAL,	NORMAL,		INPUT,	DEFAULT,	DEFAULT),//MUIC_INIT
#else
		I2C_PINMUX(GEN2_I2C_SCL, I2C2,	PULL_UP,	NORMAL, INPUT, DISABLE,  DISABLE),//MUIC_INIT
		I2C_PINMUX(GEN2_I2C_SDA, I2C2, PULL_UP,   NORMAL, INPUT, DISABLE, DISABLE),//MUIC_INIT
#endif//
//                                             

	/* I2C3 pinmux */
	I2C_PINMUX(CAM_I2C_SCL,         I2C3,            NORMAL,    NORMAL,   INPUT,	DEFAULT,	DEFAULT),
	I2C_PINMUX(CAM_I2C_SDA,         I2C3,            NORMAL,    NORMAL,   INPUT,	DEFAULT,	DEFAULT),

	/* I2C4 pinmux */
	I2C_PINMUX(DDC_SCL,             I2C4,            NORMAL,    NORMAL,  INPUT,	DEFAULT,	DEFAULT),//HDMI_EDID
	I2C_PINMUX(DDC_SDA,             I2C4,            NORMAL,    NORMAL,  INPUT,	DEFAULT,	DEFAULT),//HDMI_EDID

	/* Power I2C pinmux */
	I2C_PINMUX(PWR_I2C_SCL,         I2CPWR,          NORMAL,    NORMAL,     INPUT,	DEFAULT,	DEFAULT),
	I2C_PINMUX(PWR_I2C_SDA,         I2CPWR,          NORMAL,    NORMAL,     INPUT,	DEFAULT,	DEFAULT),

	DEFAULT_PINMUX(ULPI_DATA0,      SPI3,            PULL_UP,   TRISTATE,   OUTPUT),     //TOUCH_RESET_N
	/*                                                  */
//	DEFAULT_PINMUX(ULPI_DATA1,      ULPI,      PULL_UP,      NORMAL,      OUTPUT),		//LCD_BRIDGE_RESET_N
	/*                                                  */
	DEFAULT_PINMUX(ULPI_DATA2,      SPI3,            PULL_UP,   TRISTATE,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA3,      SPI3,            PULL_UP,   NORMAL,   INPUT),	//                                            
//                                 
	//                                                                                                                            
	DEFAULT_PINMUX(ULPI_DATA4,      ULPI,            PULL_UP,   NORMAL,   INPUT),	//                                            
//	DEFAULT_PINMUX(ULPI_DATA5,      SPI3,            PULL_UP,   TRISTATE,   INPUT),
//	DEFAULT_PINMUX(ULPI_DATA6,      SPI2,            PULL_UP,   NORMAL,   INPUT),
//	DEFAULT_PINMUX(ULPI_DATA7,      SPI3,            PULL_UP,   TRISTATE,   INPUT),
//                                 
//                                                                                                                                  
	//                                                                                                                              
	DEFAULT_PINMUX(SPI2_MOSI,       SPI2,            PULL_DOWN,  NORMAL,   OUTPUT),	//                                            

	/*                                                  */
	DEFAULT_PINMUX(ULPI_CLK,        RSVD,            PULL_DOWN,    NORMAL,   OUTPUT),		//LCD_EN
	/*                                                  */
	DEFAULT_PINMUX(ULPI_DIR,        RSVD,            PULL_UP,    NORMAL,   OUTPUT),  //                                            

//                                                            
	//DEFAULT_PINMUX(ULPI_NXT,        SPI1,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(ULPI_NXT,        RSVD,            NORMAL,    NORMAL,   OUTPUT),
//                                                          
//                                                      
	DEFAULT_PINMUX(ULPI_STP,        SPI1,            NORMAL,    NORMAL,     OUTPUT),//MUIC_INIT
	DEFAULT_PINMUX(DAP3_FS,         I2S2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP3_DIN,        I2S2,            NORMAL, 	NORMAL,   	INPUT),
	DEFAULT_PINMUX(DAP3_DOUT,       I2S2,            NORMAL,	NORMAL,   	INPUT),
	DEFAULT_PINMUX(DAP3_SCLK,       I2S2,            NORMAL, 	NORMAL, 	INPUT),
	DEFAULT_PINMUX(GPIO_PV2,        OWR,             NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PV3,        RSVD1,           PULL_DOWN, NORMAL,     INPUT),	//                                                                          

	/* SDMMC1 CD gpio */
	DEFAULT_PINMUX(CLK2_OUT,        RSVD1,      PULL_UP, NORMAL,   INPUT),
	DEFAULT_PINMUX(CLK2_REQ,        DAP,             NORMAL, NORMAL,     OUTPUT), //BT_RESET
	DEFAULT_PINMUX(LCD_PWR1,        DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), //                                      
	//DEFAULT_PINMUX(LCD_PWR2,      DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(LCD_WR_N,        DISPLAYA,        NORMAL,    NORMAL,     INPUT),

	/*                                                  */
	DEFAULT_PINMUX(LCD_SCK, 	SPI5,		 NORMAL,	NORMAL, 	INPUT),		// LCD_RGB_SCL
	DEFAULT_PINMUX(LCD_CS0_N,	SPI5,		 NORMAL,	NORMAL, 	INPUT),		// LCD_RGB_CS
	DEFAULT_PINMUX(LCD_CS1_N,	RSVD2,		 PULL_UP,	NORMAL, 	OUTPUT),	// LCD_RESET_N
	DEFAULT_PINMUX(LCD_SDOUT,	SPI5,		 NORMAL,	NORMAL, 	INPUT),		// LCD_RGB_SDO
	DEFAULT_PINMUX(LCD_SDIN,	SPI5,		 NORMAL,	NORMAL, 	INPUT),		// LCD_RGB_SDI
	/*                                                  */

	DEFAULT_PINMUX(LCD_DC0,		RSVD1,		NORMAL,		NORMAL,		OUTPUT),
//                                            

	DEFAULT_PINMUX(LCD_DC1,		RSVD1,			NORMAL,    NORMAL,	OUTPUT),	 //SENSOR_LDO_EN_2 (MPU6050 - VLOGIC)

//                                            

	DEFAULT_PINMUX(GMI_CS7_N,	GMI,		PULL_DOWN,	NORMAL,		OUTPUT),

//                                                                     

	DEFAULT_PINMUX(GMI_CS0_N,		RSVD1,			 PULL_UP,	NORMAL, 	INPUT),  //MUIC_INIT

//                                                                                                                             
	DEFAULT_PINMUX(SPI2_MISO,       GMI,         	NORMAL, NORMAL,     OUTPUT),  //PROXI_LDO_EN (APDS990x - VDD)

	DEFAULT_PINMUX(LCD_PWR0,        DISPLAYA,        NORMAL, NORMAL,   OUTPUT),  //                                      

	/*                                                  */
	DEFAULT_PINMUX(LCD_PCLK,        DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_PCLK
	DEFAULT_PINMUX(LCD_DE,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_DE
	DEFAULT_PINMUX(LCD_HSYNC,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_HSYNC
	DEFAULT_PINMUX(LCD_VSYNC,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_VSYNC
	DEFAULT_PINMUX(LCD_D0,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_2
	DEFAULT_PINMUX(LCD_D1,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_3
	DEFAULT_PINMUX(LCD_D2,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_4
	DEFAULT_PINMUX(LCD_D3,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_5
	DEFAULT_PINMUX(LCD_D4,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_6
	DEFAULT_PINMUX(LCD_D5,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_7
	DEFAULT_PINMUX(LCD_D6,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_2
	DEFAULT_PINMUX(LCD_D7,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_3
	DEFAULT_PINMUX(LCD_D8,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_4
	DEFAULT_PINMUX(LCD_D9,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_5
	DEFAULT_PINMUX(LCD_D10,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_6
	DEFAULT_PINMUX(LCD_D11,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_7
	DEFAULT_PINMUX(LCD_D12,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_2
	DEFAULT_PINMUX(LCD_D13,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_3
	DEFAULT_PINMUX(LCD_D14,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_4
	DEFAULT_PINMUX(LCD_D15,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_5
	DEFAULT_PINMUX(LCD_D16,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_6
	DEFAULT_PINMUX(LCD_D17,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_7
	DEFAULT_PINMUX(LCD_D18,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_0
	DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_B_1
	DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_0
	DEFAULT_PINMUX(LCD_D21,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_G_1
	DEFAULT_PINMUX(LCD_D22,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_0
	DEFAULT_PINMUX(LCD_D23,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),		//LCD_RGB_RGB_R_1
	DEFAULT_PINMUX(LCD_M1,          DISPLAYA,        NORMAL,    TRISTATE,   OUTPUT),		//LCD_MAKER_ID
	/*                                                  */

	//DEFAULT_PINMUX(LCD_SDIN,        DISPLAYA,        PULL_UP,   TRISTATE,   OUTPUT),
	//DEFAULT_PINMUX(LCD_SDOUT,       DISPLAYA,        PULL_UP,   TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(CRT_HSYNC,       RSVD1,             NORMAL,   NORMAL,   INPUT),//                                             
	DEFAULT_PINMUX(CRT_VSYNC,       RSVD1,             NORMAL,   NORMAL,   INPUT),//                                             
	DEFAULT_PINMUX(VI_D0,           RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D1,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D2,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D3,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D4,           VI,              NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(VI_D5,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D7,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D10,          RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_MCLK,         VI,              PULL_UP,    NORMAL,     INPUT),

	/* UART1 */
	DEFAULT_PINMUX(GPIO_PU0,        UARTA,           PULL_UP,    NORMAL,   OUTPUT),	//                                                
	DEFAULT_PINMUX(GPIO_PU1,        UARTA,           PULL_UP,    NORMAL,   INPUT),	//                                                
	DEFAULT_PINMUX(GPIO_PU2,        UARTA,           NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(GPIO_PU3,        UARTA,           NORMAL,    NORMAL,   OUTPUT),
	/* UART2 */
#if 0
	DEFAULT_PINMUX(UART2_RXD,       IRDA,            PULL_UP,   TRISTATE,   INPUT),
	DEFAULT_PINMUX(UART2_TXD,       IRDA,            PULL_UP,   TRISTATE,   INPUT),
	DEFAULT_PINMUX(UART2_RTS_N,     UARTA,           PULL_UP,   TRISTATE,   INPUT),
	DEFAULT_PINMUX(UART2_CTS_N,     UARTA,           PULL_UP,   TRISTATE,   INPUT),
#else
	DEFAULT_PINMUX(UART2_RXD,       IRDA,           PULL_UP,    NORMAL,     INPUT), //                                
	DEFAULT_PINMUX(UART2_TXD,       IRDA,           PULL_UP,    NORMAL,     OUTPUT), //                                
	DEFAULT_PINMUX(UART2_RTS_N,     UARTB,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART2_CTS_N,     UARTB,           NORMAL,    NORMAL,     INPUT),
#endif
	/* UART3 - Bluetooth BCM4330 UART I/F */
	DEFAULT_PINMUX(UART3_TXD,       UARTC,           NORMAL,	NORMAL,		OUTPUT), //BT_RX_AP_TX
	DEFAULT_PINMUX(UART3_RXD,       UARTC,           NORMAL,	NORMAL,		INPUT),  //BT_TX_AP_RX
	DEFAULT_PINMUX(UART3_CTS_N,     UARTC,           NORMAL,	NORMAL,		INPUT),  //BT_RTS_AP_CTS
	DEFAULT_PINMUX(UART3_RTS_N,     UARTC,           NORMAL,	NORMAL,		OUTPUT), //BT_CTS_AP_RTS

	DEFAULT_PINMUX(GPIO_PU4,        PWM1,            NORMAL,    TRISTATE,   INPUT),
//                                                                                                                 
	DEFAULT_PINMUX(GPIO_PU5,        RSVD1,            PULL_DOWN,    NORMAL,   OUTPUT),	//cheol.gwak 0109 host-active
	DEFAULT_PINMUX(GPIO_PU6,        PWM3,            PULL_DOWN,	NORMAL,   INPUT),  //                                       
	//PTEST DEFAULT_PINMUX(GPIO_PU6,        PWM3,            NORMAL,    TRISTATE,   INPUT),

	/* I2S3 - Bluetooth BCM4330 PCM I/F */
	DEFAULT_PINMUX(DAP4_DIN,        I2S3,            NORMAL,    NORMAL,     INPUT),  //BT_PCM_DOUT
	DEFAULT_PINMUX(DAP4_DOUT,       I2S3,            NORMAL,    NORMAL,     INPUT), //BT_PCM_DIN
	DEFAULT_PINMUX(DAP4_FS,         I2S3,            NORMAL,    NORMAL,     INPUT), //BT_PCM_SYNC
	DEFAULT_PINMUX(DAP4_SCLK,       I2S3,            NORMAL,    NORMAL,     INPUT), //BT_PCM_CLK

	/*                                                  */
	DEFAULT_PINMUX(CLK3_OUT,        EXTPERIPH3,      NORMAL,    NORMAL,     INPUT),		//MIPI_BRIDGE_CLK
	/*                                                  */
	DEFAULT_PINMUX(CLK3_REQ,        DEV3,            NORMAL,    NORMAL,     INPUT),

	DEFAULT_PINMUX(GMI_WP_N,        GMI,             PULL_UP,   NORMAL,   INPUT),
	//DEFAULT_PINMUX(GMI_WP_N,        RSVD1,             PULL_UP,   TRISTATE,     INPUT),     // PowerKey

	DEFAULT_PINMUX(GMI_WAIT,        GMI,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_RST_N,       GMI,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_ADV_N,       GMI,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CLK,         GMI,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CS0_N,       GMI,             PULL_UP,   TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_CS1_N,       RSVD1,             PULL_UP,   NORMAL,   INPUT),	   //                               
	DEFAULT_PINMUX(GMI_CS2_N,       RSVD1,             NORMAL,   NORMAL,     OUTPUT), //                                      
	DEFAULT_PINMUX(GMI_CS3_N,       RSVD1,             NORMAL,   NORMAL,     OUTPUT), //                                      
	DEFAULT_PINMUX(GMI_CS4_N,       RSVD1,             PULL_UP,   NORMAL,   INPUT),  //PROX_AMBIENT_INT (APDS990x - INT)
	DEFAULT_PINMUX(GMI_CS6_N,       GMI,             PULL_UP,   NORMAL,     INPUT),

	DEFAULT_PINMUX(GMI_AD0,         GMI,             NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD1,         GMI,             NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD2,         GMI,             NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD3,         GMI,             NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD4,         GMI,             NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD5,         GMI,             NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD6,         GMI,             NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD7,         GMI,             NORMAL,    TRISTATE,   INPUT),
//                                                                
//	DEFAULT_PINMUX(GMI_AD8,         GMI,             PULL_DOWN, TRISTATE,   INPUT),
	DEFAULT_PINMUX(GMI_AD8,         GMI,             PULL_DOWN, NORMAL,   INPUT),
//                                                                
	DEFAULT_PINMUX(GMI_AD9,         GMI,             PULL_DOWN, NORMAL,   INPUT),	//                               
	DEFAULT_PINMUX(GMI_AD10,        GMI,             NORMAL, NORMAL,   OUTPUT), 	//SENSOR_LDO_EN (MPU6050, AMI306 - VDD)
	DEFAULT_PINMUX(GMI_AD11,        PWM3,             NORMAL, NORMAL,   OUTPUT),
//                                 
	DEFAULT_PINMUX(GMI_AD12,        RSVD2,             PULL_UP,    NORMAL,   INPUT),  //SENSOR_INT (MPU6050 - INT)
	DEFAULT_PINMUX(GMI_AD13,        RSVD2,             PULL_UP,    NORMAL,   INPUT),  //COMPASS_READY (AMI306 - DRDY)
//                                 
	DEFAULT_PINMUX(GMI_AD14,        GMI,            NORMAL,    NORMAL,  OUTPUT),     //SENSOR_LDO_EN_2 (MPU6050 - VLOGIC)
	DEFAULT_PINMUX(GMI_AD15,        GMI,             NORMAL,    TRISTATE,   INPUT),
	////////////////////////
	DEFAULT_PINMUX(GMI_A16,         UARTD,             NORMAL,    NORMAL,   OUTPUT),
	DEFAULT_PINMUX(GMI_A17,         UARTD,             NORMAL,    NORMAL,   INPUT),
	/*                                                  */
	DEFAULT_PINMUX(GMI_A18,         GMI,             PULL_DOWN,    NORMAL,   OUTPUT),	//RGB_IC_EN
	/*                                                  */
	DEFAULT_PINMUX(GMI_A19,         UARTD,             NORMAL,    NORMAL,   OUTPUT),

	DEFAULT_PINMUX(GMI_WR_N,        GMI,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_OE_N,        RSVD1,             NORMAL,    TRISTATE,     INPUT), //                                                     
	DEFAULT_PINMUX(GMI_DQS,         GMI,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CAM_MCLK,        VI_ALT2,         PULL_UP,   NORMAL,     INPUT), //                                      
	DEFAULT_PINMUX(GPIO_PBB0,       I2S4,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(GPIO_PBB3,       VGP3,            NORMAL,    NORMAL,   OUTPUT), //                                     
  DEFAULT_PINMUX(GPIO_PBB4,       VGP4,            NORMAL,    NORMAL,   OUTPUT), //hyojin.an 110705 #define SUB_CAM_RESET_N       TEGRA_GPIO_PBB5
  DEFAULT_PINMUX(GPIO_PBB5,       VGP5,            NORMAL,    NORMAL,   OUTPUT), //hyojin.an 110705 #define SUB_CAM_RESET_N       TEGRA_GPIO_PBB5
	DEFAULT_PINMUX(GPIO_PBB6,       VGP6,            NORMAL,    NORMAL,   INPUT),	//                                            
	DEFAULT_PINMUX(GPIO_PBB7,       I2S4,            NORMAL,    TRISTATE,   INPUT),

	DEFAULT_PINMUX(GPIO_PCC1,       RSVD2,            NORMAL,   NORMAL,   OUTPUT),//MUIC_INIT
	DEFAULT_PINMUX(GPIO_PCC2,       RSVD2,            NORMAL,   NORMAL,     OUTPUT),//MUIC_INIT

	DEFAULT_PINMUX(JTAG_RTCK,       RTCK,            NORMAL,    NORMAL,     OUTPUT),

	/*  KBC keys */
	DEFAULT_PINMUX(KB_ROW1,         KBC,             PULL_UP, NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW2,       	RSVD3,			 PULL_DOWN, NORMAL,		OUTPUT),//YJChae  20111026 change KEY_LED for GPIO //DEFAULT_PINMUX(KB_ROW2,         KBC,             PULL_UP, NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL0,         KBC,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL1,         KBC,             NORMAL,   NORMAL,     OUTPUT),  // TOUCH_LDO_EN
	DEFAULT_PINMUX(KB_COL2,         KBC,             PULL_UP,   NORMAL,     INPUT), //TOUCH_MAKER_ID
	DEFAULT_PINMUX(KB_COL3,         KBC,             NORMAL,   NORMAL,     INPUT), //TOUCH_INT_N
	DEFAULT_PINMUX(KB_COL4,         KBC,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL5,         KBC,             PULL_UP,   NORMAL,     INPUT),
//                                                                   
//	DEFAULT_PINMUX(KB_COL6, 		KBC,			 NORMAL,	NORMAL, 	OUTPUT), /* MHL_I2C_SCL */
//	DEFAULT_PINMUX(KB_COL7, 		KBC,			 NORMAL,	NORMAL, 	OUTPUT), /* MHL_I2C_SDA */
//                                         


	DEFAULT_PINMUX(KB_ROW3,         RSVD2,           NORMAL, NORMAL,     OUTPUT),  //PROXI_LDO_EN (APDS990x - VDD)
//                                                      
//	DEFAULT_PINMUX(KB_ROW4,         KBC,             PULL_DOWN, NORMAL,     INPUT),
//	DEFAULT_PINMUX(KB_ROW5,         RSVD,             PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW4,         RSVD3,             PULL_DOWN, NORMAL,     OUTPUT),
//	DEFAULT_PINMUX(KB_ROW5,         RSVD,             NORMAL, NORMAL,     OUTPUT),
//                                           
	DEFAULT_PINMUX(KB_ROW6,         KBC,             PULL_DOWN, NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW7,         KBC,             PULL_DOWN, NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW8,         KBC,             PULL_UP, NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW9,         KBC,             PULL_DOWN, NORMAL,     INPUT),

	DEFAULT_PINMUX(KB_ROW10,        KBC,             PULL_DOWN, NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW11,        KBC,             PULL_DOWN, NORMAL,     OUTPUT), //BT_WAKEUP
	DEFAULT_PINMUX(KB_ROW12,        KBC,             PULL_DOWN, NORMAL,     INPUT),  //BT_HOST_WAKEUP
	DEFAULT_PINMUX(KB_ROW13,        KBC,             PULL_UP, NORMAL,     INPUT),  //CP_CRASH_INT
//                                                                                                                  
	DEFAULT_PINMUX(KB_ROW14,        KBC,            NORMAL,    NORMAL,     INPUT),	 //Suspend request - cheol.gwak 0109

//                                                                                                                  
	DEFAULT_PINMUX(KB_ROW15,        KBC,		     PULL_DOWN, NORMAL,	    OUTPUT), //Slave-wakeup - cheol.gwak 0109

	DEFAULT_PINMUX(GPIO_PV0,        RSVD,            PULL_UP,   NORMAL,     INPUT),  //Host-wakeup - Active low so default is high
//                                                      
//org	DEFAULT_PINMUX(GPIO_PV1,        RSVD,          NORMAL,    NORMAL,   INPUT),//MUIC_INIT
	DEFAULT_PINMUX(GPIO_PV1,        RSVD,          NORMAL,    NORMAL,   OUTPUT),//MUIC_INIT
//                                           

	/*  WLAN  */
//	DEFAULT_PINMUX(GPIO_PV2,        NORMAL,          NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,    NORMAL,   INPUT),	//                                                                           
	DEFAULT_PINMUX(SDMMC1_CMD,      SDMMC1,          PULL_UP,   NORMAL,   INPUT),	//                                                                           
	DEFAULT_PINMUX(SDMMC1_DAT3,     SDMMC1,          PULL_UP,   NORMAL,   INPUT),	//                                                                           
	DEFAULT_PINMUX(SDMMC1_DAT2,     SDMMC1,          PULL_UP,   NORMAL,   INPUT),	//                                                                           
	DEFAULT_PINMUX(SDMMC1_DAT1,     SDMMC1,          PULL_UP,   NORMAL,   INPUT),	//                                                                           
	DEFAULT_PINMUX(SDMMC1_DAT0,     SDMMC1,          PULL_UP,   NORMAL,   INPUT),	//                                                                           

//	DEFAULT_PINMUX(GPIO_PV3,        NORMAL,          NORMAL,    TRISTATE,   INPUT),

	DEFAULT_PINMUX(CLK_32K_OUT,     BLINK,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SYS_CLK_REQ,     SYSCLK,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(OWR,             OWR,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_FS,         I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_DIN,        I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_SCLK,       I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CLK1_REQ,        DAP,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CLK1_OUT,        EXTPERIPH1,      PULL_DOWN, NORMAL,     INPUT),
	DEFAULT_PINMUX(SPDIF_IN,        SPDIF,           NORMAL,    NORMAL,     OUTPUT),	//                                 
	DEFAULT_PINMUX(SPDIF_OUT,       SPDIF,           PULL_UP,   TRISTATE,   OUTPUT),
#ifdef CONFIG_SND_HDA_CODEC_REALTEK
	DEFAULT_PINMUX(DAP2_FS,         HDA,             PULL_DOWN, NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DIN,        HDA,             PULL_DOWN, NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DOUT,       HDA,             PULL_DOWN, NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_SCLK,       HDA,             PULL_DOWN, NORMAL,     INPUT),
#else
	DEFAULT_PINMUX(DAP2_FS,         I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DIN,        I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DOUT,       I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_SCLK,       I2S1,            NORMAL,    NORMAL,     INPUT),
#endif
//                                                                
#if defined(CONFIG_LGE_NFC_PN544)
	DEFAULT_PINMUX(SPI2_CS1_N,      SPI2,            NORMAL,    NORMAL,     INPUT),  //NFC_IRQ
	DEFAULT_PINMUX(SPI1_MOSI,       SPI2,            NORMAL,    NORMAL,     OUTPUT), // TOUCH_LDO_EN2
	DEFAULT_PINMUX(SPI1_SCK,        SPI2,            NORMAL,    NORMAL,     OUTPUT), //NFC_VEN
//	DEFAULT_PINMUX(SDMMC3_DAT4,     RSVD,            NORMAL,    NORMAL,     OUTPUT), //NFC_MODE
	DEFAULT_PINMUX(KB_ROW0,         RSVD3,             NORMAL, NORMAL,     OUTPUT),	//NFC_MODE
#else
	DEFAULT_PINMUX(SPI2_CS1_N,      SPI2,            PULL_UP,   TRISTATE,   INPUT),
	DEFAULT_PINMUX(SPI1_MOSI,       SPI2,            NORMAL,    NORMAL,     OUTPUT), // TOUCH_LDO_EN2
	DEFAULT_PINMUX(SPI1_SCK,        SPI2,            PULL_UP,   TRISTATE,   INPUT),
#endif
//                                                                
//                                  
#if 1
//	DEFAULT_PINMUX(SPI1_CS0_N,      RSVD,            NORMAL,   NORMAL,   INPUT),
#else
	DEFAULT_PINMUX(SPI1_CS0_N,      SPI2,            PULL_UP,   TRISTATE,   INPUT),
#endif
//                                
	//DEFAULT_PINMUX(SPI1_MISO,       SPI1,            PULL_DOWN, TRISTATE,   INPUT),
	DEFAULT_PINMUX(SPI1_MISO,       RSVD3,             NORMAL, NORMAL,   OUTPUT), 		//SENSOR_LDO_EN (MPU6050, AMI306 - VDD, THEMAL, CURRENT IC)
	DEFAULT_PINMUX(PEX_L0_PRSNT_N,  PCIE,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(PEX_L0_RST_N,    PCIE,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(PEX_L0_CLKREQ_N, PCIE,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(PEX_WAKE_N,      PCIE,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(PEX_L1_PRSNT_N,  PCIE,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L1_RST_N,    PCIE,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(PEX_L1_CLKREQ_N, PCIE,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L2_PRSNT_N,  PCIE,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(PEX_L2_RST_N,    PCIE,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(PEX_L2_CLKREQ_N, PCIE,            NORMAL,    TRISTATE,   INPUT),
//	DEFAULT_PINMUX(HDMI_CEC,        CEC,             NORMAL,    NORMAL,     INPUT),
	CEC_PINMUX(HDMI_CEC,            CEC,             NORMAL,    TRISTATE,   OUTPUT, DEFAULT, DISABLE),
	DEFAULT_PINMUX(HDMI_INT,        RSVD0,           NORMAL,    TRISTATE,   INPUT),

	/* Gpios */
	/* SDMMC1 CD gpio */
	DEFAULT_PINMUX(GMI_IORDY,       RSVD1,             PULL_UP,    NORMAL,     INPUT),//                                             
	/* SDMMC1 WP gpio */
	DEFAULT_PINMUX(VI_D11,          RSVD1,           PULL_UP,    NORMAL,     INPUT),

	/* Power rails GPIO */
	VI_PINMUX(VI_D6,           VI,              NORMAL,    NORMAL,     OUTPUT, DISABLE, DISABLE),
	VI_PINMUX(VI_D8,           SDMMC2,          NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_D9,           SDMMC2,          NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_PCLK,         RSVD1,           PULL_UP,   TRISTATE,   INPUT,  DISABLE, ENABLE),
	VI_PINMUX(VI_HSYNC,        RSVD1,           NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_VSYNC,        RSVD1,           NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
};



int __init x3_pinmux_init(void)
{
	tegra_pinmux_config_table(x3_pinmux, ARRAY_SIZE(x3_pinmux));
	tegra_drive_pinmux_config_table(x3_drive_pinmux, ARRAY_SIZE(x3_drive_pinmux));
	return 0;
}
