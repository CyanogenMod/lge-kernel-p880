/*
 * Xilinx SPI device driver API and platform data header file
 *
 * Copyright (c) 2009 Intel Corporation
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _SPI_SOLOMON_TABLE_H_
#define _SPI_SOLOMON_TABLE_H_

#define MIPI_NON_BURST_MODE  //default(2/27)
//#define HITACHI_CABC_ENABLE
//#define CURRENT_BIAS_MIPI_OUTPUT  //default : no use(2/27)
#define HITACHI_GAMMA_S_CURVE
#define GAMMA_3

/*----------------------------------------
*	DEFINITION FOR 5 INCH DISPLAY
-----------------------------------------*/
#define HITACHI_INVERSION_SELECT
//#define HITACHI_POWER_SETTING
#define COLUMN_INVERSION
#define HITACHI_DIGITAL_CONTRAST_ADJ

// VIEW5_MANUFACTURE_COMMAND_ACCESS
// enable : before specific setting
//#define VIEW5_MANUFACTURE_COMMAND_ACCESS

// VIEW5_CABC_ON
// enable : CABC ON
//#define VIEW5_CABC_ON

// CABC Reduction Ration
//#define CABC_REDUCTION_10
//#define CABC_REDUCTION_20
//#define CABC_REDUCTION_30
//#define CABC_REDUCTION_40

#include "ssd2825_bridge.h"

struct spi_cmd_data16{
	unsigned short delay;
	unsigned short value;
};

struct spi_cmd_data16 solomon_init_sequence_set[] = {
		{0, 0x00B1},
		{0, 0x0105},//{0, 0x0104},	//Horizontal Pulse Width
		{0, 0x0102},			//Vertical Pulse Width

		{0, 0x00B2},
		{0, 0x0156},//{0, 0x0154}, // HBP + width(Because of the Non-burst mode)
		{0, 0x010A},//{0, 0x0109}, // VBP + width(Because of the Non-burst mode)

		{0, 0x00B3},
		{0, 0x0174}, //HFP
		{0, 0x0118}, //{0, 0x011B},//VFP

//	LCD Resolution !!
		{0, 0x00B4},
		{0, 0x0100},
		{0, 0x0103},

		{0, 0x00B5},
		{0, 0x0100},
		{0, 0x0104},

#ifdef MIPI_NON_BURST_MODE
		{0, 0x00B6},
		{0, 0x0107}, // default Non-burst mode:{0, 0x0103},
		{0, 0x0100}, // default Non-burst mode:{0, 0x0100},
#else
		{0, 0x00B6},
		{0, 0x010B}, // burst mode for power consumption:{0, 0x010B},
		{0, 0x0100}, // burst mode for power consumption:{0, 0x0100},
#endif
		{0, 0x00DE},
		{0, 0x0103},
		{0, 0x0100},

		{0, 0x00D6},
		{0, 0x0104},
		{0, 0x0100},

#ifdef CURRENT_BIAS_MIPI_OUTPUT
		{0, 0x00D8},
		{0, 0x011C},
		//{0, 0x0112}, // Default
		//{0, 0x011E}, // Max
		{0, 0x0102}, // Min
#endif
		{0, 0x00B9},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},

        {0, 0x00C9},
        {0, 0x0103},
        {0, 0x0121},

		{0, 0x00BA},
		//{0, 0x0112},  // 12h (432Mbps)
		//{0, 0x0180},  // 80h (432Mbps)
		//{0, 0x0138},    // 38h (448Mbps)
		//{0, 0x0183},    // 83h (448Mbps)
		//{0, 0x01AB},    // 513Mbps 0xC8AB
		//{0, 0x01C8},    // 513Mbps 0xC8AB
		//{0, 0x0111},	// 408bps
		//{0, 0x0180},	// 408Mbps
		//{0, 0x0119},	// 600Mbps 0xC8AB
		//{0, 0x01C0},	// 600Mbps 0xC8AB
		{0, 0x01D7},	// 430Mbps 0x8CD7
		{0, 0x018C},	// 430Mbps 0x8CD7
		//{0, 0x01B9},	// 370Mbps 0x8CB9
		//{0, 0x018C},	// 370Mbps 0x8CB9
		//{0, 0x01CD},	// 410Mbps 0x8CCD
		//{0, 0x018C},	// 410Mbps 0x8CCD
		//{0, 0x01F8},	// 496Mbps 0xCCF8
		//{0, 0x01CC},	// 496Mbps 0xCCF8
		//{0, 0x01E1},	// 450Mbps 0x8CE1
		//{0, 0x018C},	// 450Mbps 0x8CE1
		//{0, 0x0169},	// 420Mbps 0x8669
		//{0, 0x0186},	// 420Mbps 0x8669


		{0, 0x00BB},
		{0, 0x0109}, //{0, 0x0108}, //change for Hitach ESD
		{0, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B8},
		{0, 0x0100},
		{10, 0x0100},

		{0, 0x00B7},
		{0, 0x0142},
		{0, 0x0103},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		/*exit_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0111},{80, 0x0100},

		/*set_address_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0136},{20, 0x0100},

		/*set_pixel_format*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x013A},{0, 0x0170},

		{0, 0x00B7},
		{0, 0x0102},
		{0, 0x0103},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

#if 0
		/*CABC OFF*/
 		/*Manufacturer Command Access Protect Off*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0104},

		{0, 0x00BC},{0, 0x0106},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B8},{0, 0x0100},{0, 0x011A},
		{0, 0x0118},{0, 0x0102},{0, 0x0140},

		/*Manufacturer Command Access Protect On*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0103},
#endif

		/*Manufacturer Command Access Protect Off*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0104},

#if defined(HITACHI_DIGITAL_CONTRAST_ADJ)
		//Digital Contrast Adjustment
		{0, 0x00BC},{0, 0x0104},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01CC},{0, 0x01DC},{0, 0x01B4},
		{0, 0x01FF},

#endif

#if defined(HITACHI_GAMMA_S_CURVE)

#if defined(GAMMA_3)
		/* DO *** S-curve */
		/*gamma setting A - Gamma Setting*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x010B},{0, 0x010D},
		{0, 0x0110},{0, 0x0114},{0, 0x0113},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0112},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},{0, 0x010A},{0, 0x010C},
		{0, 0x0110},{0, 0x0114},{0, 0x0113},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0112},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},
#elif defined(GAMMA_2)
		/* DV2 *** S-curve */
		/*gamma setting A - Gamma Setting*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x0100},{0, 0x0105},
		{0, 0x010B},{0, 0x010F},{0, 0x0111},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0118},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},{0, 0x0100},{0, 0x0105},
		{0, 0x010B},{0, 0x010F},{0, 0x0111},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0118},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},
#else
		/* Bring Up 1st *** S-curve */
		/*gamma setting A - Gamma Setting*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x0100},{0, 0x0106},
		{0, 0x010A},{0, 0x010F},{0, 0x0114},
		{0, 0x011F},{0, 0x011F},{0, 0x0117},
		{0, 0x0112},{0, 0x010C},{0, 0x0109},
		{0, 0x0106},{0, 0x0100},{0, 0x0106},
		{0, 0x010A},{0, 0x010F},{0, 0x0114},
		{0, 0x011F},{0, 0x011F},{0, 0x0117},
		{0, 0x0112},{0, 0x010C},{0, 0x0109},
		{0, 0x0106},
#endif
#endif //HITACHI_GAMMA_S_CURVE

//                                                                           
#if defined(HITACHI_INVERSION_SELECT)
		//Panel Driving Setting
		{0, 0x00BC},{0, 0x0109},{0, 0x0100},
		{0, 0x00BF},
#if defined(COLUMN_INVERSION)
		{0, 0x01C1},{0, 0x0100},{0, 0x0150},//Column
		{0, 0x0103},{0, 0x0122},{0, 0x0116},
		{0, 0x0106},{0, 0x0160},{0, 0x0111},
#else
		{0, 0x01C1},{0, 0x0100},{0, 0x0110},//2Lines
		{0, 0x0103},{0, 0x0122},{0, 0x0116},
		{0, 0x0106},{0, 0x0160},{0, 0x0101},
#endif
#endif
//                                                                           

//                                                                                           
#if defined(HITACHI_POWER_SETTING)
		//Panel Driving Setting
		{0, 0x00BC},{0, 0x0109},{0, 0x0100},
		{0, 0x00BF},
#if defined(COLUMN_INVERSION)
		{0, 0x01D1},{0, 0x018E},{0, 0x0127},//Column
		{0, 0x0144},{0, 0x0163},{0, 0x0197},
		{0, 0x0163},{0, 0x01C9},{0, 0x0106},
#else
		{0, 0x01D1},{0, 0x016E},{0, 0x0129},//2DOT
		{0, 0x0144},{0, 0x0142},{0, 0x0175},
		{0, 0x0173},{0, 0x01EB},{0, 0x0106},
#endif
#endif
//                                                                                           

		/*Manufacturer Command Access Protect On*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0103},

		{0, 0x00B7},
		{0, 0x0142},
		{0, 0x0103},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		/*Display on*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0129},{10, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00B7},
		{0, 0x0149},
		{0, 0x0103},
};

static struct spi_cmd_data16 solomon_power_off_set[] = {
		{0, 0x00B7}, {0, 0x0149}, {0, 0x0103},
		{0, 0x00BC}, {0, 0x0102}, {0, 0x0100},
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100}, //set_display_off
		{0, 0x00BF}, {0, 0x0110}, {100, 0x0100}, //enter_sleep_mode
		{0, 0x00B7}, {0, 0x0100}, {0, 0x0103},
		{0, 0x00B9}, {0, 0x0100}, {0, 0x0100},
};

struct spi_cmd_data16 solomon_reg_read_set[] = {
		{0, 0x00C0}, //SW reset
		{0, 0x0100},
		{0, 0x0101},

		{0, 0x00B1},
		{0, 0x0105},//{0, 0x0104},	//Horizontal Pulse Width
		{0, 0x0102},				//Vertical Pulse Width

		{0, 0x00B2},
		{0, 0x0156},//{0, 0x0154}, // HBP + width(Because of the Non-burst mode)
		{0, 0x010A},//{0, 0x0109}, // VBP + width(Because of the Non-burst mode)

		{0, 0x00B3},
		{0, 0x0174}, //HFP
		{0, 0x0118}, //{0, 0x011B},//VFP

		// 0xB4 : 0x0300 : 768
		{0, 0x00B4},
		{0, 0x0100},
		{0, 0x0103},
	    // 0xB5 : 0x0400 : 1024
		{0, 0x00B5},
		{0, 0x0100},
		{0, 0x0104},

#ifdef MIPI_NON_BURST_MODE
		{0, 0x00B6},
		{0, 0x0107}, // default Non-burst mode:{0, 0x0103},
		{0, 0x0100}, // default Non-burst mode:{0, 0x0100},
#else
		{0, 0x00B6},
		{0, 0x010B}, // burst mode for power consumption:{0, 0x010B},
		{0, 0x0100}, // burst mode for power consumption:{0, 0x0100},
#endif

		{0, 0x00DE},
		{0, 0x0103},
		{0, 0x0100},

		{0, 0x00D6},
		{0, 0x0104},
		{0, 0x0100},

#ifdef CURRENT_BIAS_MIPI_OUTPUT
		{0, 0x00D8},
		{0, 0x011C},
		//{0, 0x0112}, // Default
		//{0, 0x011E}, // Max
		{0, 0x0102}, // Min
#endif

		{0, 0x00B9},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00C9},
		{0, 0x0103},
		{0, 0x0121},

		{0, 0x00BA},
		//{0, 0x0112},  // 12h (432Mbps)
		//{0, 0x0180},  // 80h (432Mbps)
		//{0, 0x0138},    // 38h (448Mbps) //448Mbps 0x8338
		//{0, 0x0183},    // 83h (448Mbps)
		//{0, 0x01AB},    // 513Mbps 0xC8AB
		//{0, 0x01C8},    // 513Mbps 0xC8AB
		{0, 0x01D7},	// 430Mbps 0x8CD7
		{0, 0x018C},	// 430Mbps 0x8CD7
		//{0, 0x01B9},	// 370Mbps 0x8CB9
		//{0, 0x018C},	// 370Mbps 0x8CB9
		//{0, 0x01CD},	// 410Mbps 0x8CCD
		//{0, 0x018C},	// 410Mbps 0x8CCD
		//{0, 0x01F8},	// 496Mbps 0xCCF8
		//{0, 0x018C},	// 496Mbps 0x8CF8
		//{0, 0x01E1},	// 450Mbps 0x8CE1
		//{0, 0x018C},	// 450Mbps 0x8CE1
		//{0, 0x0169},	// 420Mbps 0x8669
		//{0, 0x0186},	// 420Mbps 0x8669

		{0, 0x00BB},
		{0, 0x0109}, //{0, 0x0108}, //change for Hitach ESD
		{0, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B8},
		{0, 0x0100},
		{10, 0x0100},

		{0, 0x00B7},
		{0, 0x0142},
		{0, 0x0103},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		/*exit_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0111},{80, 0x0100},

		/*set_address_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0136},{20, 0x0100},

		/*set_pixel_format*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x013A},{0, 0x0170},

		/*Solomon_leave_sleep_sequence*/
		{0, 0x00B7},{0, 0x0102},{0, 0x0103},			//CHECK !!!!
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		//{0, 0x00BC},{0, 0x0100},{0, 0x0100},

#if 0
		/*CABC OFF*/
 		/*Manufacturer Command Access Protect Off*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0104},

		{0, 0x00BC},{0, 0x0106},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B8},{0, 0x0100},{0, 0x011A},
		{0, 0x0118},{0, 0x0102},{0, 0x0140},

		/*Manufacturer Command Access Protect On*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0103},
#endif

		/*Manufacturer Command Access Protect Off*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0104},

#if defined(HITACHI_DIGITAL_CONTRAST_ADJ)
		//Digital Contrast Adjustment
		{0, 0x00BC},{0, 0x0104},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01CC},{0, 0x01DC},{0, 0x01B4},//{0, 0x01D4},
		{0, 0x01FF},

#endif

#if defined(HITACHI_GAMMA_S_CURVE)

#if defined(GAMMA_3)
		/* DO *** S-curve */
		/*gamma setting A - Gamma Setting*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x010B},{0, 0x010D},
		{0, 0x0110},{0, 0x0114},{0, 0x0113},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0112},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},{0, 0x010A},{0, 0x010C},
		{0, 0x0110},{0, 0x0114},{0, 0x0113},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0112},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},
#elif defined(GAMMA_2)
		/* DV2 *** S-curve */
		/*gamma setting A - Gamma Setting*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x0100},{0, 0x0105},
		{0, 0x010B},{0, 0x010F},{0, 0x0111},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0118},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},{0, 0x0100},{0, 0x0105},
		{0, 0x010B},{0, 0x010F},{0, 0x0111},
		{0, 0x011D},{0, 0x0120},{0, 0x0118},
		{0, 0x0118},{0, 0x0109},{0, 0x0107},
		{0, 0x0106},
#else
		/* Bring Up 1st *** S-curve */
		/*gamma setting A - Gamma Setting*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x0100},{0, 0x0106},
		{0, 0x010A},{0, 0x010F},{0, 0x0114},
		{0, 0x011F},{0, 0x011F},{0, 0x0117},
		{0, 0x0112},{0, 0x010C},{0, 0x0109},
		{0, 0x0106},{0, 0x0100},{0, 0x0106},
		{0, 0x010A},{0, 0x010F},{0, 0x0114},
		{0, 0x011F},{0, 0x011F},{0, 0x0117},
		{0, 0x0112},{0, 0x010C},{0, 0x0109},
		{0, 0x0106},
#endif
#endif //HITACHI_GAMMA_S_CURVE

//                                                                           
#if defined(HITACHI_INVERSION_SELECT)
		//Panel Driving Setting
		{0, 0x00BC},{0, 0x0109},{0, 0x0100},
		{0, 0x00BF},
#if defined(COLUMN_INVERSION)
		{0, 0x01C1},{0, 0x0100},{0, 0x0150},//Column
		{0, 0x0103},{0, 0x0122},{0, 0x0116},
		{0, 0x0106},{0, 0x0160},{0, 0x0111},
#else
		{0, 0x01C1},{0, 0x0100},{0, 0x0110},//2Lines
		{0, 0x0103},{0, 0x0122},{0, 0x0116},
		{0, 0x0106},{0, 0x0160},{0, 0x0101},
#endif
#endif
//                                                                           

//                                                                                           
#if defined(HITACHI_POWER_SETTING)
		//Panel Driving Setting
		{0, 0x00BC},{0, 0x0109},{0, 0x0100},
		{0, 0x00BF},
#if defined(COLUMN_INVERSION)
		{0, 0x01D1},{0, 0x018E},{0, 0x0127},//Column
		{0, 0x0144},{0, 0x0163},{0, 0x0197},
		{0, 0x0163},{0, 0x01C9},{0, 0x0106},
#else
		{0, 0x01D1},{0, 0x016E},{0, 0x0129},//2DOT
		{0, 0x0144},{0, 0x0142},{0, 0x0175},
		{0, 0x0173},{0, 0x01EB},{0, 0x0106},
#endif
#endif
//                                                                                           

		/*Manufacturer Command Access Protect On*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0103},

		//change by DCS, display on
		{0, 0x00B7},{0, 0x0142},{0, 0x0103}, // change by generic
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},

		/*Display on*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0129},{10, 0x0100},

		{0, 0x00B9},{0, 0x0101},{0, 0x0100},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00B7},{0, 0x0149},{0, 0x0103},

};

struct spi_cmd_data16 solomon_reg_read_set2[] = {
		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00FF},
		//{0, 0x00FA},
};

struct spi_cmd_data16 solomon_reg_read_set3[] = {
		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C6},
		//{0, 0x00FA},
};

/*DCS packets in LP mode*/
struct spi_cmd_data16 solomon_reg_read_set4[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x01C2}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x010C}, //Hitachi LCD get_pixel_format: 0Ch => 07h
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

/*DCS packets in HS mode*/
struct spi_cmd_data16 solomon_reg_read_set5[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x01C3}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x010A}, //Hitachi LCD get_power_mode: 0Ah => 1Ch
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

/*Generic Packets in LP mode*/
struct spi_cmd_data16 solomon_reg_read_set6[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00B7},{0, 0x0102},{0, 0x0103},
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},{0, 0x01B0},{0, 0x0104},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x0182}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01BF}, //Hitachi LCD Device Code Read(BFh) => 01h,22h,93h,28h,01h
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

/*Generic Packets in HS mode*/
struct spi_cmd_data16 solomon_reg_read_set7[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00B7},{0, 0x0102},{0, 0x0103},
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},{0, 0x01B0},{0, 0x0104},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x0183}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/

		{0, 0x00BC},
		{0, 0x0101}, //0x0100 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01BF},//Hitachi LCD Device Code Read(BFh) => 01h,22h,93h,28h,01h
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

struct spi_cmd_data16 solomon_reg_read_set8[] = {
		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00B0},
		//{0, 0x00FA},
};

struct spi_cmd_data16 solomon_reg_read_set9[] = {
		{0, 0x00B7},{0, 0x0102},{0, 0x0103},
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},{0, 0x01B0},{0, 0x0103},
};

const char ssd2825_reg_set[]={
0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xDD,0xB7,0xB8,
0xB9,0xBA,0xD5,0xBB,0xBC,0xBD,0xBE,0xBF,0xC0,0xC1,
0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,
0xCC,0xCD,0xCE,0xCF,0xD0,0xD1,0xD2,0xD3,0xD4,0xD6,
0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDE,0xDF,0xE0,0xE1,
0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,
0xFF};
/*DCS packets in HS mode*/
struct spi_cmd_data16 solomon_reg_read_set4_1[] = {
		{0, 0x00C1},
		{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		//{0, 0x01C2}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x01C9}, // keeping video mode on
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
};
#endif // _SPI_SOLOMON_TABLE_H_
