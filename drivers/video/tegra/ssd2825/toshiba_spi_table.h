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

#ifndef _SPI_TOSHIBA_TABLE_H_
#define _SPI_TOSHIBA_TABLE_H_

#include <ssd2825_bridge.h>

#define SPLIT_WRITE 1

typedef struct spi_cmd_data32{
	unsigned short delay;
	unsigned int   value;
};

#define BRIDGE_INIT \
		{0,	0x00020001}, \
		{0,	0x00020000}, \
		{0,	0x00160413}, \
		{0,	0x00180413}, \
		{0,	0x01400000}, \
		{0,	0x01420000}, \
		{0,	0x01440000}, \
		{0,	0x01460000}, \
		{0,	0x01480000}, \
		{0,	0x014a0000}, \
		{0,	0x014c0000}, \
		{0,	0x014e0000}, \
		{0,	0x01500000}, \
		{0,	0x01520000}, \
		{0,	0x02140003}, \
		{0,	0x02160000}, \
		{0,	0x02181103}, \
		{0,	0x021a0000}, \
		{0,	0x021c0000}, \
		{0,	0x021e0000}, \
		{0,	0x02200104}, \
		{0,	0x02220000}, \
		{0,	0x02280007}, \
		{0,	0x022a0000}, \
		{0,	0x022c0002}, \
		{0,	0x022e0000}, \
		{0,	0x0234001f}, \
		{0,	0x02360000}, \
		{0,	0x02380000}, \
		{0,	0x023a0000}, \
		{0,	0x02040001}, \
		{0,	0x02060000}, \
		{0,	0x06200001}, \
		{0,	0x06220012}, \
		{0,	0x06260500}, \
		{0,	0x062800f0}, \
		{0,	0x062c0870}, \
		{0,	0x05180001}, \
		{0,	0x051a0000}, \
		{0,	0x05000087}, \
		{0,	0x0502a300} 


struct spi_cmd_data32 toshiba_init_sequence_set_updated[] = {
		BRIDGE_INIT,
		
		//6-2 display initial set
		//step 1: e0=43,00,80, 00,00
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc=6
		{0,	0x061043e0},
		{0,	0x06128000},
		{0,	0x06140000},
		{0,	0x06000001},  //start tx
		
		//step 2: b5=14,20,40, 00,00
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc
		{0,	0x061014b5},
		{0,	0x06124020},
		{0,	0x06140000},
		{0,	0x06000001},  //start tx
		
		//step 3: b6=01,16,0f, 16,13
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc
		{0,	0x061001b6},
		{0,	0x06120f16},
		{0,	0x06141316},
		{0,	0x06000001},  //start tx
		
		//Tosh WAR from 9/6
		//step 4: d0=00,30,56, 01,12,04,34, 10,01
		{0,	0x00080001},  //use data ID in reg 0x50
		{0,	0x00500039},  //packet ID=0x39
		{0,	0x0022000a},  //10 bytes to send
		{0,	0x00e083ff},  //debug pattern generator, not documented
		{0,	0x00e20148},
		{0,	0x00e4007f},
		
		//10 bytes to send: d0=00,30,56, 01,12,04,34, 10,01
		{0,	0x00e800d0},
		{0,	0x00e85630},
		{0,	0x00e81201},
		{0,	0x00e83404},
		{0,	0x00e80110},
		{0,	0x00e80000},
		
		{0, 0x00e0c000},  //start to send
		{0, 0x00e00000},  //stop right away to prevent multiple output
		//end Tosh WAR
		
		BRIDGE_INIT,  //after each use of WAR
		//Tosh WAR from 9/6
		//step 5: d1=02,51,56, 06,08,00,47, 12,02
		{0,	0x00080001},  //use data ID in reg 0x50
		{0,	0x00500039},  //packet ID=0x39
		{0,	0x0022000a},  //10 bytes to send
		{0,	0x00e083ff},  //debug pattern generator, not documented
		{0,	0x00e20148},
		{0,	0x00e4007f},
		
		//10 bytes to send: d1=02,51,56, 06,08,00,47, 12,02
		{0,	0x00e802d1},
		{0,	0x00e85651},
		{0,	0x00e80806},
		{0,	0x00e84700},
		{0,	0x00e80212},
		{0,	0x00e80000},
		
		{0,	0x00e0c000},  //start to send
		{0,	0x00e00000},  //stop right away to prevent multiple output
		//end Tosh WAR

		BRIDGE_INIT,  //after each use of WAR

		//Tosh WAR from 9/6
		//step 6: d2=00,30,56, 01,12,04,34, 10,01
		{0,	0x00080001},  //use data ID in reg 0x50
		{0,	0x00500039},  //packet ID=0x39
		{0,	0x0022000a},  //10 bytes to send
		{0,	0x00e083ff},  //debug pattern generator, not documented
		{0,	0x00e20148},
		{0,	0x00e4007f},
		
		//10 bytes to send: d2=00,30,56, 01,12,04,34, 10,01
		{0,	0x00e800d2},
		{0,	0x00e85630},
		{0,	0x00e81201},
		{0,	0x00e83404},
		{0,	0x00e80110},
		{0,	0x00e80000},
		
		{0,	0x00e0c000},  //start to send
		{0,	0x00e00000},  //stop right away to prevent multiple output
		//end Tosh WAR
				
		
		BRIDGE_INIT,  //after each use of WAR
		//Tosh WAR from 9/6
		//step 7: d3=00,66,76, 04,02,02,42, 02,03
		{0,	0x00080001},  //use data ID in reg 0x50
		{0,	0x00500039},  //packet ID=0x39
		{0,	0x0022000a},  //10 bytes to send
		{0,	0x00e083ff},  //debug pattern generator, not documented
		{0,	0x00e20148},
		{0,	0x00e4007f},
		
		//10 bytes to send: d3=00,66,76, 04,02,02,42, 02,03
		{0,	0x00e800d3},
		{0,	0x00e87666},
		{0,	0x00e80204},
		{0,	0x00e84202},
		{0,	0x00e80302},
		{0,	0x00e80000},
		
		{0,	0x00e0c000},  //start to send
		{0,	0x00e00000},  //stop right away to prevent multiple output
		//end Tosh WAR
				
		BRIDGE_INIT,  //after each use of WAR
		//Tosh WAR from 9/6
		//step 8: d4=00,30,56, 01,12,04,34, 10,01
		{0,	0x00080001},  //use data ID in reg 0x50
		{0,	0x00500039},  //packet ID=0x39
		{0,	0x0022000a},  //10 bytes to send
		{0,	0x00e083ff},  //debug pattern generator, not documented
		{0,	0x00e20148},
		{0,	0x00e4007f},
		
		//10 bytes to send: d4=00,30,56, 01,12,04,34, 10,01
		{0,	0x00e800d4},
		{0,	0x00e85630},
		{0,	0x00e81201},
		{0,	0x00e83404},
		{0,	0x00e80110},
		{0,	0x00e80000},
		
		{0,	0x00e0c000},  //start to send
		{0,	0x00e00000},  //stop right away to prevent multiple output
		//end Tosh WAR
		//***************************
		
		
		BRIDGE_INIT,  //after each use of WAR
		//***************************
		//Tosh WAR from 9/6
		//step 9: d5=00,66,76, 04,02,02,42, 02,03
		{0,	0x00080001},  //use data ID in reg 0x50
		{0,	0x00500039},  //packet ID=0x39
		{0,	0x0022000a},  //10 bytes to send
		{0,	0x00e083ff},  //debug pattern generator, not documented
		{0,	0x00e20148},
		{0,	0x00e4007f},
		
		//10 bytes to send: d5=00,66,76, 04,02,02,42, 02,03
		{0,	0x00e800d5},
		{0,	0x00e87666},
		{0,	0x00e80204},
		{0,	0x00e84202},
		{0,	0x00e80302},
		{0,	0x00e80000},
		
		{0,	0x00e0c000},  //start to send
		{0,	0x00e00000},  //stop right away to prevent multiple output
		//end Tosh WAR
			
		//step 10: 70=07
		{0,	0x06021015},  //DCS Short write, 1 parameter
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100770},
		{0,	0x06000001},  //start tx
		
		//step 11: 71=00,00,01, 01
		{0,	0x06024039},  //DCS long write
		{0,	0x06040005},  //wc
		{0,	0x06100071},
		{0,	0x06120100},
		{0,	0x06140001},
		{0,	0x06000001},  //start tx
		
		//step 12: 72=01,0f
		{0,	0x06024039},  //DCS long write
		{0,	0x06040003},  //wc=3
		{0,	0x06100172},
		{0,	0x0612000f},
		{0,	0x06000001},  //start tx
		
		
		//step 13: 73=34,55,00
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06103473},
		{0,	0x06120055},
		{0,	0x06000001},  //start tx
		
		//step 14: 74=04,01,07
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06100474},
		{0,	0x06120107},
		{0,	0x06000001},  //start tx
		
		//step 15: 75=03,0f,07
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06100375},
		{0,	0x0612070f},
		{0,	0x06000001},  //start tx
		
		//step 16: 76=07,00,05
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06100776},
		{0,	0x06120500},
		{0,	0x06000001},  //start tx
		
		
		//6-3 power supply set
		//step 1: c0=01,08
		{0,	0x06024039},  //DCS long write
		{0,	0x06040003},  //wc
		{0,	0x061001c0},
		{0,	0x06120008},
		{0,	0x06000001},  //start tx
		
		
		BRIDGE_INIT,
		//***************************
		//Tosh WAR from 9/6
		//step 2: c3=00,08,00, 00,00,67,88, 32,02
		{0,	0x00080001},  //use data ID in reg 0x50
		{0,	0x00500039},  //packet ID=0x39
		{0,	0x0022000a},  //10 bytes to send
		{0,	0x00e083ff},  //debug pattern generator, not documented
		{0,	0x00e20148},
		{0,	0x00e4007f},
		
		//10 bytes to send: c3=00,08,00, 00,00,67,88, 32,02
		{0,	0x00e800c3},
		{0,	0x00e80008},
		{0,	0x00e80000},
		{0,	0x00e88867},
		{0,	0x00e80232},
		{0,	0x00e80000},
		
		{0,	0x00e0c000},  //start to send
		{0,	0x00e00000},  //stop right away to prevent multiple output
		//end Tosh WAR
		
		//step 3: c4=22,24,19, 19,41
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc
		{0,	0x061022c4},
		{0,	0x06121924},
		{0,	0x06144119},
		{0,	0x06000001},  //start tx
		
		//step 4: f9=00
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061000f9},
		{0,	0x06000001},  //start tx
		
		
		//**********************************************
		//new in updated LG init from gpio_spi.h *******
		//**********************************************
		
		//c1=00
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061000c1},
		{0,	0x06000001},  //start tx
		
		//c2=02
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061002c2},
		{0,	0x06000001},  //start tx
		
		//c2=06
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061006c2},
		{0,	0x06000001},  //start tx
		
		//c2=4e
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06104ec2},
		{0,	0x06000001},  //start tx
		
		//6-4 sleep out & display on set
		//step 1: 11=no parameter
		{0,	0x06021005},  //DCS short write, 0 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100011},
		{150,	0x06000001},  //start tx
		
		//step 2: wait 7 frames or more

		
		//step 3: 29=no parameter
		{0,	0x06021005},  //DCS short write, 0 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100029},
		{0,	0x06000001},  //start tx
		
		////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////
		//
		{0,	0x00163138},  //step 3  //pll 24.545 MHz
		{0,	0x00180813},  //step 4
		////////////////////////////////////////////////////////////


		{0,	0x05008000},  //step 28
		{0,	0x0502c300},
		//spi_write32(0x00040064)  //step 29  //originally 0064

		{0,	0x00060060},  //step 5  //orig 0x82
		{0,	0x00080037},  //step 6
		{0,	0x0050003e},  //step 7
		{0,	0x00040044},  //step 29  
		//                                                                                          
};



struct spi_cmd_data32 toshiba_init_sequence_set[] = {
		{0,	0x00020001},  //step 1
		{0,	0x00020000},  //step 2
		//{0,	0x00162010},  //step 3 // changed for catch the flicking
		{0,	0x00163138},  //step 3 // before 0x00163138( 24.545MHz ) to left 0x0016212b( 19.2MHz)
		{0,	0x00180813},  //step 4 
		{0,	0x00060082},  //                          
		{0,	0x00080037},  //step 6
		{0,	0x0050003e},  //step 7
		{0,	0x01400000},  //step 8
		{0,	0x01420000},
		{0,	0x01440000},  //step 9
		{0,	0x01460000},
		{0,	0x01480000},  //step 10
		{0,	0x014a0000},
		{0,	0x014c0000},  //step 11
		{0,	0x014e0000},
		{0,	0x01500000},  //step 12
		{0,	0x01520000},
#if defined(CONFIG_MACH_LGHDK_HD_HITACHI)
		{0,	0x0214000A},  //step 13  
#elif defined(CONFIG_MACH_LGHDK_HD_LGD)	|| defined(CONFIG_MACH_X3_HD_LGD)
		{0,	0x02140003},  // 2->tlpx=51.2ns, 3->tlpx=68.81ns, tclkpre=628ns
#endif
		{0,	0x02160000},      //MSB
		//W1_SPI_WRITE32(0x02180303)  //step 14
		//W1_SPI_WRITE32(0x021a0000)
#if defined(CONFIG_MACH_LGHDK_HD_HITACHI)
		{0,	0x02180503},  //step 14 //ken-sue 8/13 4:12pm
#elif defined(CONFIG_MACH_LGHDK_HD_LGD)	|| defined(CONFIG_MACH_X3_HD_LGD)
		{0, 0x02181103},  //step 14	//cbareng 8/31 -> 318.4ns TCLK-PREPARE+TCLK-ZERO   
#endif
		{0,	0x021a0000},

#if defined(CONFIG_MACH_LGHDK_HD_LGD) || defined(CONFIG_MACH_X3_HD_LGD)	
		{0, 0x021c0000},
		{0, 0x021e0000},
#endif

		//cbareng 8/16 add PHY timing changes
		{0,	0x02200104},  //15:8=THS_ZEROCNT, 7:0=THS_PREPARECNT, THS_PREPARE=72.7ns (in spec)
		{0,	0x02220000},
		
#if defined(CONFIG_MACH_LGHDK_HD_HITACHI)		
		{0,	0x02280010},  //step 15
#elif defined(CONFIG_MACH_LGHDK_HD_LGD)	|| defined(CONFIG_MACH_X3_HD_LGD)
		{0,	0x02280007},  //step 15  //cbareng 8/31 b=252.8ns, a=236.8ns, 7=185.6ns
#endif
		{0,	0x022a0000},
		
#if defined(CONFIG_MACH_LGHDK_HD_LGD) || defined(CONFIG_MACH_X3_HD_LGD)	
		//cbareng 8/31 added 0x022c for THS-TRAIL
		{0, 0x022c0002},  //2=default->97.2ns, spec min 60ns+(n*4*UI)=~68ns
		{0, 0x022e0000},
#endif
		{0,	0x0234001f},  //step 16
		{0,	0x02360000},
		
#if defined(CONFIG_MACH_LGHDK_HD_HITACHI)		
		{0,	0x02380001},  //step 17  //cbareng 8/16 add PHY timing changes
		{0,	0x023a0000},
#elif defined(CONFIG_MACH_LGHDK_HD_LGD) || defined(CONFIG_MACH_X3_HD_LGD)
	 	{0,	0x02380000},  //step 17  //non-continuous clock
	 	{0,	0x023a0000},
#endif
				
		{0,	0x02040001},  //step 18
		{0,	0x02060000},
		{0,	0x06200001}, //step 19
		//W1_SPI_WRITE32(0x06220004)  //step 20
		{0,	0x06220012},  //step 20  //ken-sue 8/15
		//W1_SPI_WRITE32(0x0624000e)  //step 21  //e  //ken-sue 8/15 comment out
		{0,	0x06260500},  //step 22
		{0,	0x062800f0},  //step 23  //f0 per Tosh calc
		//W1_SPI_WRITE32(0x062a----)  //step 24 - skip per Toshiba spreadsheet
		{0,	0x062c0870},  //step 25
		{0,	0x05180001},  //step 26
		{0,	0x051a0000},
		//2 writes below added by Chris Cheng/Toshiba
		// set 768 to LP mode        :cc 
		{0,	0x05000001},  // added step 
		{0,	0x0502a300}, 

#if defined(CONFIG_MACH_LGHDK_HD_LGD) || defined(CONFIG_MACH_X3_HD_LGD)
		{0,	0x05000087},  //step 27  //move before panel_init to send all commands in HS mode over 4 data lanes
		{0,	0x0502a300},  //HS
#endif


#if defined(CONFIG_MACH_LGHDK_HD_HITACHI)


		{0,	0x06021005},
		{0,	0x06040000},
		{0,	0x06100036},
		{0,	0x06000001},

		{0,	0x06021005},
		{0,	0x06040000},
		{0,	0x0610603A},
		{0,	0x06000001},



#elif defined(CONFIG_MACH_LGHDK_HD_LGD) || defined(CONFIG_MACH_X3_HD_LGD)


		//bridge steps 27-29 moved after panel init per Toshiba

		// ******************************************************************
		// START PANEL INIT *************************************************
		// ******************************************************************

		//LG panel init sequence
		// TO DO
		//6-2 display initial set
		//step 1: e0=43,00,80, 00,00
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc=6
		{0,	0x061043e0},
		{0,	0x06128000},
		{0,	0x06140000},
		{0,	0x06000001},  //start tx

		//step 2: b5=14,20,40, 00,00
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc
		{0,	0x061014b5},
		{0,	0x06124020},
		{0,	0x06140000},
		{0,	0x06000001},  //start tx

		//step 3: b6=01,16,0f, 16,13
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc
		{0,	0x061001b6},
		{0,	0x06120f16},
		{0,	0x06141316},
		{0,	0x06000001},  //start tx

#if 0 //no gamma 0907 by cbareng & beobki.chung

		//step 4: d0=00,30,56, 01,12,04,34, 10,01
		{0,	0x06024039},  //DCS long write
		//W1_SPI_WRITE32(0x0604000A)  //wc
		{0,	0x06040008},  //wc
		{0,	0x061000d0},
		{0,	0x06125630},
		{0,	0x06141201},
		{0,	0x06163404},
		{0,	0x06000001},  //start tx

		//if (%split_write == 1)
#if SPLIT_WRITE
		{0,	0x06024039},  //DCS long write
		{0,	0x06040002},  //wc
		{0,	0x06100110},
		{0,	0x06000001},  //start tx


#endif
		//step 5: d1=02,51,56, 06,08,00,47, 12,02
		{0,	0x06024039},  //DCS long write
		//W1_SPI_WRITE32(0x0604000A)  //wc
		{0,	0x06040008},  //wc
		{0,	0x061002d1},
		{0,	0x06125651},
		{0,	0x06140806},
		{0,	0x06164700},
		{0,	0x06000001},  //start tx

		//if (%split_write == 1)
#if SPLIT_WRITE
		{0,	0x06024039},  //DCS long write
		{0,	0x06040002},  //wc
		{0,	0x06100212},
		{0,	0x06000001},  //start tx
#endif


		//step 6: d2=00,30,56, 01,12,04,34, 10,01
		{0,	0x06024039},  //DCS long write
		//W1_SPI_WRITE32(0x0604000A)  //wc
		{0,	0x06040008},  //wc
		{0,	0x061000d2},
		{0,	0x06125630},
		{0,	0x06141201},
		{0,	0x06163404},
		{0,	0x06000001},  //start tx

		//if (%split_write == 1)
#if SPLIT_WRITE
		{0,	0x06024039},  //DCS long write
		{0,	0x06040002},  //wc
		{0,	0x06100110},
		{0,	0x06000001},  //start tx
#endif

		//step 7: d3=00,66,76, 04,02,02,42, 02,03
		{0,	0x06024039},  //DCS long write
		//W1_SPI_WRITE32(0x0604000A)  //wc
		{0,	0x06040008},  //wc
		{0,	0x061002d3},
		{0,	0x06125651},
		{0,	0x06140806},
		{0,	0x06164700},
		{0,	0x06000001},  //start tx

		//if (%split_write == 1)
#if SPLIT_WRITE
		{0,	0x06024039},  //DCS long write
		{0,	0x06040002},  //wc
		{0,	0x06100212},
		{0,	0x06000001},  //start tx
#endif

		//step 8: d4=00,30,56, 01,12,04,34, 10,01
		{0,	0x06024039},  //DCS long write
		//W1_SPI_WRITE32(0x0604000A)  //wc
		{0,	0x06040008},  //wc
		{0,	0x061000d4},
		{0,	0x06125630},
		{0,	0x06141201},
		{0,	0x06163404},
		{0,	0x06000001},  //start tx

		//if (%split_write == 1)
#if SPLIT_WRITE
		{0,	0x06024039},  //DCS long write
		{0,	0x06040002},  //wc  // change>>
		{0,	0x06100110},
		{0,	0x06000001},  //start tx
#endif

		//step 9: d5=00,66,76, 04,02,02,42, 02,03
		{0,	0x06024039},  //DCS long write
		//W1_SPI_WRITE32(0x0604000A)  //wc
		{0,	0x06040008},  //wc
		{0,	0x061002d5},
		{0,	0x06125651},
		{0,	0x06140806},
		{0,	0x06164700},
		{0,	0x06000001},  //start tx

		//if (%split_write == 1)
#if SPLIT_WRITE
		{0,	0x06024039},  //DCS long write
		{0,	0x06040002},  //wc
		{0,	0x06100212},
		{0,	0x06000001},  //start tx
#endif

#endif

		//step 10: 70=07
		{0,	0x06021015},  //DCS Short write, 1 parameter
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100770},
		{0,	0x06000001},  //start tx

		//step 11: 71=00,00,01, 01
		{0,	0x06024039},  //DCS long write
		{0,	0x06040005},  //wc
		{0,	0x06100071},
		{0,	0x06120100},
		{0,	0x06140001},
		{0,	0x06000001},  //start tx

		//step 12: 72=01,0f
		{0,	0x06024039},  //DCS long write
		{0,	0x06040003},  //wc=3
		{0,	0x06100172},
		{0,	0x0612000f},
		{0,	0x06000001},  //start tx
		                         

		//step 13: 73=34,55,00
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06103473},
		{0,	0x06120055},
		{0,	0x06000001},  //start tx

		//step 14: 74=04,01,07
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06100474},
		{0,	0x06120107},
		{0,	0x06000001},  //start tx

		//step 15: 75=03,0f,07
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06100375},
		{0,	0x0612070f},
		{0,	0x06000001},  //start tx

		//step 16: 76=07,00,05
		{0,	0x06024039},  //DCS long write
		{0,	0x06040004},  //wc
		{0,	0x06100776},
		{0,	0x06120500},
		{0,	0x06000001},  //start tx


		//6-3 power supply set
		//step 1: c0=01,08
		{0,	0x06024039},  //DCS long write
		{0,	0x06040003},  //wc
		{0,	0x061001c0},
		{0,	0x06120008},
		{0,	0x06000001},  //start tx

		//step 2: c3=00,08,00, 00,00,67,88, 32,02
		{0,	0x06024039},  //DCS long write
		{0,	0x06040008},  //wc
		{0,	0x061000c3},
		{0,	0x06120008},
		{0,	0x06140000},
		{0,	0x06168867},
		{0,	0x06000001},  //start tx

		//if (%split_write == 1)
#if SPLIT_WRITE
		{0,	0x06024039},  //DCS long write
		{0,	0x06040002},  //wc
		{0,	0x06100232},  // changes 110907
		{0,	0x06000001},  //start tx
#endif

		//step 3: c4=22,24,19, 19,41
		{0,	0x06024039},  //DCS long write
		{0,	0x06040006},  //wc
		{0,	0x061022c4},
		{0,	0x06121924},
		{0,	0x06144119},
		{0,	0x06000001},  //start tx

		//step 4: f9=00
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061000f9},
		{0,	0x06000001},  //start tx


		//**********************************************
		//new in updated LG init from gpio_spi.h *******
		//**********************************************

		//c1=00
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061000c1},
		{0,	0x06000001},  //start tx

		//c2=02
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061002c2},
		{0,	0x06000001},  //start tx

		//c2=06
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061006c2},
		{0,	0x06000001},  //start tx

		//c2=4e
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06104ec2},
		{0,	0x06000001},  //start tx

		//**********************************************
		//end updated LG init from gpio_spi.h **********
		//**********************************************

#endif
		//6-4 sleep out & display on set
		//step 1: 11=no parameter
		{0,	0x06021005},  //DCS short write, 0 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100011},
		{150,	0x06000001},  //start tx

		//step 2: wait 7 frames or more



		//step 3: 29=no parameter
		{0,	0x06021005},  //DCS short write, 0 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100029},
		{0,	0x06000001},  //start tx

		// ******************************************************************
		// END PANEL INIT ***************************************************
		// ******************************************************************




		//bridge steps 27-29 moved after panel init per Toshiba
		{0,	0x05000087},  //step 27
		{0,	0x0502a300},
		{0,	0x05008000},  //step 28
		{0,	0x0502c300},
		//W1_SPI_WRITE32(0x00040064)  //step 29  //originally 0064
		{0,	0x00040044},  //step 29  //originally 0064 ken-sue 8/13

};


static struct spi_cmd_data32 toshiba_power_off_set[] = {
	
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100028},
		{100,	0x06000001},  //start tx

		/* Enter Sleep */
		// 10 : 00 
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x06100010},
		{150,	0x06000001},  //start tx

		/* Deep Stanby */
		// C1 : 00 
		{0,	0x06021015},  //DCS short write, 1 parameters
		{0,	0x06040000},  //wc=0 for short write
		{0,	0x061000C1},
		{0,	0x06000001},  //start tx
		
};

#endif // _SPI_TOSHIBA_TABLE_H_
