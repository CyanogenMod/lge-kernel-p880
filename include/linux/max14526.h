/*
 * The only header file for Hub TI TS5USBA33402 MUIC driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __MAX14526_MUIC_H__
#define __MAX14526_MUIC_H__

#include <linux/platform_device.h>

/* 
 * INT_STAT bits : settle down time since MUIC INT falls on TI chip
 * STATUS bits   : settle down time since MUIC INT falls on TI chip 
 * DP, DM path   : settle down time since SW_CONTROL writing on TI chip 
 *
 */
#define TD_INT_STAT	70000	
#define TD_STATUS	250000	
#define TD_DP_DM	1000	

/* I2C addresses of MUIC internal registers */
#define	DEVICE_ID	(u8)0x00
#define	CONTROL_1	(u8)0x01
#define	CONTROL_2	(u8)0x02
#define	SW_CONTROL	(u8)0x03
#define	INT_STAT	(u8)0x04
#define	STATUS		(u8)0x05

/* Masks for the each bit of CONTROL_1 register */
#define	ID_2P2		(u8)0x40
#define	ID_620		(u8)0x20
#define	ID_200		(u8)0x10
#define	VLDO		(u8)0x08
#define	SEMREN		(u8)0x04
#define	ADC_EN		(u8)0x02
#define	CP_EN		(u8)0x01

/* Masks for the each bit of CONTROL_2 register */
#define	INTPOL		(u8)0x80
#define	INT_EN		(u8)0x40
#define	MIC_LP		(u8)0x20
#define	CP_AUD		(u8)0x10
#define	CHG_TYPE	(u8)0x02
#define	USB_DET_DIS	(u8)0x01

/* Masks for the each bit of SW_CONTROL register */
#define	MIC_ON		(u8)0x40
#define DP		(u8)0x38
#define DM		(u8)0x07

/* DP, DM settings */
#define DP_USB		(u8)0x00
#define	DP_UART		(u8)0x08
#define	DP_AUDIO	(u8)0x10
#define	DP_OPEN		(u8)0x38
#define DM_USB		(u8)0x00
#define	DM_UART		(u8)0x01
#define	DM_AUDIO	(u8)0x02
#define	DM_OPEN		(u8)0x07


/* Combined masks of SW_CONTROLl register */
#define USB		DP_USB   | DM_USB 	/* 0x00 */
#define UART 		DP_UART  | DM_UART 	/* 0x09 */
#define AUDIO		DP_AUDIO | DM_AUDIO	/* 0x12 */
#define OPEN		DP_OPEN  | DM_OPEN 	/* 0x3f */


/* Masks for the each bit of INT_STATUS register */
#define	CHGDET		(u8)0x80
#define	MR_COMP		(u8)0x40
#define	SENDEND		(u8)0x20
#define	VBUS		(u8)0x10
#define	IDNO		(u8)0x0f

/* Masks for the each bit of STATUS register */
#define	DCPORT		(u8)0x80
#define	CHPORT		(u8)0x40
#define C1COMP      	(u8)0x01	/* only on MAXIM */


//                     
/* IDNO */
#define IDNO_0000	0x00
#define IDNO_0001	0x01
#define IDNO_0010	0x02
#define IDNO_0011	0x03
#define IDNO_0100	0x04
#define IDNO_0101	0x05
#define IDNO_0110	0x06
#define IDNO_0111	0x07
#define IDNO_1000	0x08
#define IDNO_1001	0x09
#define IDNO_1010	0x0A
#define IDNO_1011	0x0B

/* DP2 */
#define COMP2_TO_DP2  	0x00
#define COMP2_TO_U2   	0x08
#define COMP2_TO_AUD2 	0x10
#define COMP2_TO_HZ   	0x20

/* DM */
#define COMN1_TO_DN1  	0x00
#define COMN1_TO_U1   	0x01
#define COMN1_TO_AUD1 	0x02
#define COMN1_TO_C1COMP 0x03
#define COMN1_TO_HZ   	0x04

/* MUIC chip vendor */
#define TS5USBA33402 	0x10
#define MAX14526 	0x20
#define ANY_VENDOR 	0xff


typedef enum {
	MUIC_MODE_UNKNOWN,		// 0
	MUIC_MODE_NONE,			// 1 - No accessory is plugged in.
	MUIC_MODE_NA_TA,		// 2 - Not used actually. 
					//     Special TA for North America.
	MUIC_MODE_LG_TA,		// 3
	MUIC_MODE_TA_1A,		// 4 
	MUIC_MODE_INVALID_CHG,		// 5
	MUIC_MODE_AP_UART,		// 6
	MUIC_MODE_CP_UART,		// 7
	MUIC_MODE_AP_USB,		// 8
	MUIC_MODE_CP_USB,		// 9
	MUIC_MODE_TV_OUT_NO_LOAD,	// 10 - Not used.
	MUIC_MODE_EARMIC,		// 11
	MUIC_MODE_TV_OUT_LOAD,		// 12 - Not used.
	MUIC_MODE_OTG,			// 13 - Not used.
	MUIC_MODE_RESERVE1,		// 14
	MUIC_MODE_RESERVE2,		// 15
	MUIC_MODE_RESERVE3,		// 16
	MUIC_MODE_MODE_NO,		// 17
} muic_mode_t;

struct max14526_platform_data {
	int 	(*init_gpio)(void);
	void 	(*deinit_gpio)(void);
	int	(*enable_charger)(muic_mode_t mode);
	int	gpio;
};

typedef enum {
	NOT_UPON_IRQ,	// 0
	UPON_IRQ,		// 1
} TYPE_UPON_IRQ;

typedef enum {
	DEFAULT,	/* 0 - Just apply the default register settings. */
	RESET,		/* 1 - Fully reset the MUIC. It takes 250msec. */
} TYPE_RESET;

typedef enum {
	NO_RETAIN,
	RETAIN,
	BOOT_AP_USB,
	BOOT_CP_USB,
} TYPE_RETAIN_MODE;
int get_muic_charger_detected(void);


#endif // __MAX14526_H__








