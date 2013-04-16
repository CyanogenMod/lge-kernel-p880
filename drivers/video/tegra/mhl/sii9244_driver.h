
/*********************************************************************************/
/*  Copyright (c) 2002-2011, Silicon Image, Inc.  All rights reserved.           */
/*  No part of this work may be reproduced, modified, distributed, transmitted,  */
/*  transcribed, or translated into any language or computer format, in any form */
/*  or by any means without written permission of: Silicon Image, Inc.,          */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                         */
/*********************************************************************************/

#include "../../../../arch/arm/mach-tegra/gpio-names.h"
#define FEATURE_SiI9244_DEVICE

#define MHL_CTL		1
//                                                                                      
#define MHL_HW_DEBUG	1
//                                                                                      

// MHL GPIO Pins
#define GPIO_LEVEL_LOW		0
#define GPIO_LEVEL_HIGH		1
#define GPIO_MHL_INT		TEGRA_GPIO_PV7
#define GPIO_MHL_SEL		TEGRA_GPIO_PS2//146
#define GPIO_MHL_EN			TEGRA_GPIO_PV2
#define GPIO_MHL_RST		TEGRA_GPIO_PEE1//162
//#define GPIO_MHL_WAKE_UP	TEGRA_GPIO_PR3//160
// End


#ifndef bool
#define bool char
#endif

#define BIT0                    0x01
#define BIT1                    0x02
#define BIT2                    0x04
#define BIT3                    0x08
#define BIT4                    0x10
#define BIT5                    0x20
#define BIT6                    0x40
#define BIT7                    0x80

#define MSG_ALWAYS              0x00
#define MSG_STAT                0x01
#define MSG_DBG                 0x02

// see include/i2c_slave_addrs.h

#define SET_BITS    0xFF
#define CLEAR_BITS  0x00


#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif



#define	PAGE_0_0X72			0x72
#define	PAGE_1_0x7A			0x7A
#define	PAGE_2_0x92			0x92
#define	PAGE_CBUS_0XC8		0xC8

// Device Power State
#define MHL_DEV_UNPOWERED		0x00
#define MHL_DEV_INACTIVE		0x01
#define MHL_DEV_QUIET			0x03
#define MHL_DEV_ACTIVE			0x04

// Version that this chip supports
#define	MHL_VER_MAJOR		(0x01 << 4)	// bits 4..7
#define	MHL_VER_MINOR		0x00		// bits 0..3
#define MHL_VERSION						(MHL_VER_MAJOR | MHL_VER_MINOR)

//Device Category
#define MHL_CAP_DEV_STATE 0x00
#define MHL_CAP_MHL_VERSION 0x01
#define	MHL_DEV_CATEGORY_OFFSET				0x02
#define MHL_CAP_ADOPTER_ID_H  0x03
#define MHL_CAP_ADOPTER_ID_L  0x04
#define MHL_CAP_VID_LINK_MODE 0x05
#define MHL_CAP_AUD_LINK_MODE 0x06
#define MHL_CAP_VIDEO_TYPE 0x07
#define MHL_CAP_LOG_DEV_MAP 0x08
#define MHL_CAP_BANDWIDTH 0x09
#define MHL_CAP_FEATURE_FLAG 0x0A
#define MHL_CAP_DEVICE_ID_H 0x0B
#define MHL_CAP_DEVICE_ID_L 0x0C
#define MHL_CAP_SCRATCHPAD_SIZE 0x0D
#define MHL_CAP_INT_STAT_SIZE 0x0E
#define MHL_CAP_RESERVED  0x0F


#define	MHL_DEV_CATEGORY_POW_BIT			(BIT4)

#define	MHL_DEV_CAT_SOURCE					0x02
#define MHL_SINK_W_POW  0x11
#define MHL_SINK_WO_POW 0x01
#define MHL_DONGLE_W_POW 0x13
#define MHL_DONGLE_WO_POW 0x03


//Video Link Mode
#define	MHL_DEV_VID_LINK_SUPPRGB444			0x01
#define	MHL_DEV_VID_LINK_SUPPYCBCR444		0x02
#define	MHL_DEV_VID_LINK_SUPPYCBCR422		0x04
#define	MHL_DEV_VID_LINK_PPIXEL				0x08
#define	MHL_DEV_VID_LINK_SUPP_ISLANDS		0x10

//Audio Link Mode Support
#define	MHL_DEV_AUD_LINK_2CH				0x01
#define	MHL_DEV_AUD_LINK_8CH				0x02


//Feature Flag in the devcap
#define	MHL_DEV_FEATURE_FLAG_OFFSET			0x0A
#define	MHL_FEATURE_RCP_SUPPORT				BIT0	// Dongles have freedom to not support RCP
#define	MHL_FEATURE_RAP_SUPPORT				BIT1	// Dongles have freedom to not support RAP
#define	MHL_FEATURE_SP_SUPPORT				BIT2	// Dongles have freedom to not support SCRATCHPAD

/*
#define		MHL_POWER_SUPPLY_CAPACITY		16		// 160 mA current
#define		MHL_POWER_SUPPLY_PROVIDED		16		// 160mA 0r 0 for Wolverine.
#define		MHL_HDCP_STATUS					0		// Bits set dynamically
*/

// VIDEO TYPES
#define		MHL_VT_GRAPHICS					0x00
#define		MHL_VT_PHOTO					0x02
#define		MHL_VT_CINEMA					0x04
#define		MHL_VT_GAMES					0x08
#define		MHL_SUPP_VT						0x80

//Logical Dev Map
#define	MHL_DEV_LD_DISPLAY					(0x01 << 0)
#define	MHL_DEV_LD_VIDEO					(0x01 << 1)
#define	MHL_DEV_LD_AUDIO					(0x01 << 2)
#define	MHL_DEV_LD_MEDIA					(0x01 << 3)
#define	MHL_DEV_LD_TUNER					(0x01 << 4)
#define	MHL_DEV_LD_RECORD					(0x01 << 5)
#define	MHL_DEV_LD_SPEAKER					(0x01 << 6)
#define	MHL_DEV_LD_GUI						(0x01 << 7)

//Bandwidth
#define	MHL_BANDWIDTH_LIMIT					22		// 225 MHz


#define MHL_STATUS_REG_CONNECTED_RDY        0x30
#define MHL_STATUS_REG_LINK_MODE            0x31

#define	MHL_STATUS_DCAP_RDY					BIT0

#define MHL_STATUS_CLK_MODE_MASK            0x07
#define MHL_STATUS_CLK_MODE_PACKED_PIXEL    0x02
#define MHL_STATUS_CLK_MODE_NORMAL          0x03
#define MHL_STATUS_PATH_EN_MASK             0x08
#define MHL_STATUS_PATH_ENABLED             0x08
#define MHL_STATUS_PATH_DISABLED            0x00
#define MHL_STATUS_MUTED_MASK               0x10

#define MHL_RCHANGE_INT                     0x20
#define MHL_DCHANGE_INT                     0x21

#define	MHL_INT_DCAP_CHG					BIT0
#define MHL_INT_DSCR_CHG                    BIT1
#define MHL_INT_REQ_WRT                     BIT2
#define MHL_INT_GRT_WRT                     BIT3

// On INTR_1 the EDID_CHG is located at BIT 0
#define	MHL_INT_EDID_CHG					BIT1

#define		MHL_INT_AND_STATUS_SIZE			0x33		// This contains one nibble each - max offset
#define		MHL_SCRATCHPAD_SIZE				16
#define		MHL_MAX_BUFFER_SIZE				MHL_SCRATCHPAD_SIZE	// manually define highest number

///////////////////////////////////////////////////////////////////////////////
//
// CBUS register defintions
//
#define REG_CBUS_INTR_STATUS            0x08
#define BIT_DDC_ABORT                   (BIT2)    /* Responder aborted DDC command at translation layer */
#define BIT_MSC_MSG_RCV                 (BIT3)    /* Responder sent a VS_MSG packet (response data or command.) */
#define BIT_MSC_XFR_DONE                (BIT4)    /* Responder sent ACK packet (not VS_MSG) */
#define BIT_MSC_XFR_ABORT               (BIT5)    /* Command send aborted on TX side */
#define BIT_MSC_ABORT                   (BIT6)    /* Responder aborted MSC command at translation layer */

#define REG_CBUS_INTR_ENABLE            0x09

#define REG_DDC_ABORT_REASON        	0x0C
#define REG_CBUS_BUS_STATUS             0x0A
#define BIT_BUS_CONNECTED                   0x01
#define BIT_LA_VAL_CHG                      0x02

#define REG_PRI_XFR_ABORT_REASON        0x0D

#define REG_CBUS_PRI_FWR_ABORT_REASON   0x0E
#define	CBUSABORT_BIT_REQ_MAXFAIL			(0x01 << 0)
#define	CBUSABORT_BIT_PROTOCOL_ERROR		(0x01 << 1)
#define	CBUSABORT_BIT_REQ_TIMEOUT			(0x01 << 2)
#define	CBUSABORT_BIT_UNDEFINED_OPCODE		(0x01 << 3)
#define	CBUSSTATUS_BIT_CONNECTED			(0x01 << 6)
#define	CBUSABORT_BIT_PEER_ABORTED			(0x01 << 7)

#define REG_CBUS_PRI_START              0x12
#define BIT_TRANSFER_PVT_CMD                0x01
#define BIT_SEND_MSC_MSG                    0x02
#define	MSC_START_BIT_MSC_CMD		        (0x01 << 0)
#define	MSC_START_BIT_VS_CMD		        (0x01 << 1)
#define	MSC_START_BIT_READ_REG		        (0x01 << 2)
#define	MSC_START_BIT_WRITE_REG		        (0x01 << 3)
#define	MSC_START_BIT_WRITE_BURST	        (0x01 << 4)

#define REG_CBUS_PRI_ADDR_CMD           0x13
#define REG_CBUS_PRI_WR_DATA_1ST        0x14
#define REG_CBUS_PRI_WR_DATA_2ND        0x15
#define REG_CBUS_PRI_RD_DATA_1ST        0x16
#define REG_CBUS_PRI_RD_DATA_2ND        0x17


#define REG_CBUS_PRI_VS_CMD             0x18
#define REG_CBUS_PRI_VS_DATA            0x19

#define	REG_MSC_WRITE_BURST_LEN         0x20       // only for WRITE_BURST
#define	MSC_REQUESTOR_DONE_NACK         	(0x01 << 6)      

#define	REG_CBUS_MSC_RETRY_INTERVAL			0x1A		// default is 16
#define	REG_CBUS_DDC_FAIL_LIMIT				0x1C		// default is 5
#define	REG_CBUS_MSC_FAIL_LIMIT				0x1D		// default is 5
#define	REG_CBUS_MSC_INT2_STATUS        	0x1E
#define REG_CBUS_MSC_INT2_ENABLE             	0x1F
#define	MSC_INT2_REQ_WRITE_MSC              (0x01 << 0)	// Write REG data written.
#define	MSC_INT2_HEARTBEAT_MAXFAIL          (0x01 << 1)	// Retry threshold exceeded for sending the Heartbeat

#define	REG_MSC_WRITE_BURST_LEN         0x20       // only for WRITE_BURST

#define	REG_MSC_HEARTBEAT_CONTROL       0x21       // Periodic heart beat. TX sends GET_REV_ID MSC command
#define	MSC_HEARTBEAT_PERIOD_MASK		    0x0F	// bits 3..0
#define	MSC_HEARTBEAT_FAIL_LIMIT_MASK	    0x70	// bits 6..4
#define	MSC_HEARTBEAT_ENABLE			    0x80	// bit 7

#define REG_MSC_TIMEOUT_LIMIT           0x22
#define	MSC_TIMEOUT_LIMIT_MSB_MASK	        (0x0F)	        // default is 1
#define	MSC_LEGACY_BIT					    (0x01 << 7)	    // This should be cleared.

#define	REG_CBUS_LINK_CONTROL_1				0x30	// 
#define	REG_CBUS_LINK_CONTROL_2				0x31	// 
#define	REG_CBUS_LINK_CONTROL_3				0x32	// 
#define	REG_CBUS_LINK_CONTROL_4				0x33	// 
#define	REG_CBUS_LINK_CONTROL_5				0x34	// 
#define	REG_CBUS_LINK_CONTROL_6				0x35	// 
#define	REG_CBUS_LINK_CONTROL_7				0x36	// 
#define REG_CBUS_LINK_STATUS_1          	0x37
#define REG_CBUS_LINK_STATUS_2          	0x38
#define	REG_CBUS_LINK_CONTROL_8				0x39	// 
#define	REG_CBUS_LINK_CONTROL_9				0x3A	// 
#define	REG_CBUS_LINK_CONTROL_10				0x3B	//
#define	REG_CBUS_LINK_CONTROL_11				0x3C	// 
#define	REG_CBUS_LINK_CONTROL_12				0x3D	// 


#define REG_CBUS_LINK_CTRL9_0           0x3A
#define REG_CBUS_LINK_CTRL9_1           0xBA

#define	REG_CBUS_DRV_STRENGTH_0				0x40	// 
#define	REG_CBUS_DRV_STRENGTH_1				0x41	// 
#define	REG_CBUS_ACK_CONTROL				0x42	// 
#define	REG_CBUS_CAL_CONTROL				0x43	// Calibration

#define REG_CBUS_SCRATCHPAD_0           0xC0
#define REG_CBUS_DEVICE_CAP_0           0x80
#define REG_CBUS_DEVICE_CAP_1           0x81
#define REG_CBUS_DEVICE_CAP_2           0x82
#define REG_CBUS_DEVICE_CAP_3           0x83
#define REG_CBUS_DEVICE_CAP_4           0x84
#define REG_CBUS_DEVICE_CAP_5           0x85
#define REG_CBUS_DEVICE_CAP_6           0x86
#define REG_CBUS_DEVICE_CAP_7           0x87
#define REG_CBUS_DEVICE_CAP_8           0x88
#define REG_CBUS_DEVICE_CAP_9           0x89
#define REG_CBUS_DEVICE_CAP_A           0x8A
#define REG_CBUS_DEVICE_CAP_B           0x8B
#define REG_CBUS_DEVICE_CAP_C           0x8C
#define REG_CBUS_DEVICE_CAP_D           0x8D
#define REG_CBUS_DEVICE_CAP_E           0x8E
#define REG_CBUS_DEVICE_CAP_F           0x8F
#define REG_CBUS_SET_INT_0				0xA0
#define REG_CBUS_SET_INT_1				0xA1
#define REG_CBUS_SET_INT_2				0xA2
#define REG_CBUS_SET_INT_3				0xA3
#define REG_CBUS_WRITE_STAT_0        	0xB0
#define REG_CBUS_WRITE_STAT_1        	0xB1
#define REG_CBUS_WRITE_STAT_2        	0xB2
#define REG_CBUS_WRITE_STAT_3        	0xB3


// DEVCAP we will initialize to
#define	MHL_LOGICAL_DEVICE_MAP		(MHL_DEV_LD_AUDIO | MHL_DEV_LD_VIDEO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_GUI )
enum
{
    MHL_MSC_MSG_RCP             = 0x10,     // RCP sub-command
    MHL_MSC_MSG_RCPK            = 0x11,     // RCP Acknowledge sub-command
    MHL_MSC_MSG_RCPE            = 0x12,     // RCP Error sub-command
    MHL_MSC_MSG_RAP             = 0x20,     // Mode Change Warning sub-command
    MHL_MSC_MSG_RAPK            = 0x21,     // MCW Acknowledge sub-command
};

#define	RCPE_NO_ERROR				0x00
#define	RCPE_INEEFECTIVE_KEY_CODE	0x01
#define	RCPE_BUSY					0x02
//
// MHL spec related defines
//
enum
{
	MHL_ACK						= 0x33,	// Command or Data byte acknowledge
	MHL_NACK					= 0x34,	// Command or Data byte not acknowledge
	MHL_ABORT					= 0x35,	// Transaction abort
	MHL_WRITE_STAT				= 0x60 | 0x80,	// 0xE0 - Write one status register strip top bit
	MHL_SET_INT					= 0x60,	// Write one interrupt register
	MHL_READ_DEVCAP				= 0x61,	// Read one register
	MHL_GET_STATE				= 0x62,	// Read CBUS revision level from follower
	MHL_GET_VENDOR_ID			= 0x63,	// Read vendor ID value from follower.
	MHL_SET_HPD					= 0x64,	// Set Hot Plug Detect in follower
	MHL_CLR_HPD					= 0x65,	// Clear Hot Plug Detect in follower
	MHL_SET_CAP_ID				= 0x66,	// Set Capture ID for downstream device.
	MHL_GET_CAP_ID				= 0x67,	// Get Capture ID from downstream device.
	MHL_MSC_MSG					= 0x68,	// VS command to send RCP sub-commands
	MHL_GET_SC1_ERRORCODE		= 0x69,	// Get Vendor-Specific command error code.
	MHL_GET_DDC_ERRORCODE		= 0x6A,	// Get DDC channel command error code.
	MHL_GET_MSC_ERRORCODE		= 0x6B,	// Get MSC command error code.
	MHL_WRITE_BURST				= 0x6C,	// Write 1-16 bytes to responder's scratchpad.
	MHL_GET_SC3_ERRORCODE		= 0x6D,	// Get channel 3 command error code.
};

#define	MHL_RAP_CONTENT_ON		0x10	// Turn content streaming ON.
#define	MHL_RAP_CONTENT_OFF		0x11	// Turn content streaming OFF.

///////////////////////////////////////////////////////////////////////////////
//
// MHL Timings applicable to this driver.
//
//
#define	T_SRC_VBUS_CBUS_TO_STABLE	(200)	// 100 - 1000 milliseconds. Per MHL 1.0 Specs
#define	T_SRC_WAKE_PULSE_WIDTH_1	(20)	// 20 milliseconds. Per MHL 1.0 Specs
#define	T_SRC_WAKE_PULSE_WIDTH_2	(60)	// 60 milliseconds. Per MHL 1.0 Specs

#define	T_SRC_WAKE_TO_DISCOVER		(500)	// 100 - 1000 milliseconds. Per MHL 1.0 Specs

#define T_SRC_VBUS_CBUS_T0_STABLE 	(500)

// Allow RSEN to stay low this much before reacting.
// Per specs between 100 to 200 ms
#define	T_SRC_RSEN_DEGLITCH			(100)	// (150)

// Wait this much after connection before reacting to RSEN (300-500ms)
// Per specs between 300 to 500 ms
// #define	T_SRC_RXSENSE_CHK			(400)		// 400 -> 300 -> 200
#define	T_SRC_RXSENSE_CHK			(300)



typedef enum
{
      FLAGS_SCRATCHPAD_BUSY         = 0x01
    , FLAGS_REQ_WRT_PENDING         = 0x02
    , FLAGS_WRITE_BURST_PENDING     = 0x04
    , FLAGS_RCP_READY               = 0x08
    , FLAGS_HAVE_DEV_CATEGORY       = 0x10
    , FLAGS_HAVE_DEV_FEATURE_FLAGS  = 0x20
    , FLAGS_SENT_DCAP_RDY           = 0x40
    , FLAGS_SENT_PATH_EN            = 0x80
}MiscFlags_e;





typedef enum        
{
    NO_MHL_STATUS = 0x00,
    MHL_INIT_DONE,
    MHL_WAITING_RGND_DETECT,
    MHL_CABLE_CONNECT,
    MHL_DISCOVERY_START,
    MHL_DISCOVERY_END,
    MHL_DISCOVERY_SUCCESS, 
    MHL_DISCOVERY_FAIL,
    MHL_RSEN_GLITCH,
    MHL_RSEN_LOW,
}MHL_status_enum_type;


typedef struct cbus_dev {
	spinlock_t		lock;
  u8 reqstatus;
  u8 retrycount;
  u8 command;
  u8 offsetdata;
  u8 length;

  union{
  u8 msgdata[16];
  u8 *pdata;
  }payload_u; 
  
	struct work_struct	cbus_work;
}cbus_send_cmd_type;

#ifdef MHL_CTL
#define		MHD_SCRATCHPAD_SIZE				16
#define		MHD_MAX_BUFFER_SIZE				MHD_SCRATCHPAD_SIZE	// manually define highest number
typedef struct 
{
	u8 mscScratchpadData[16];
} mhl_ctl_data_t;
#endif

typedef struct _cbus_cmd{
  bool cmd_send;
  bool cmd_done;
  u8 command;
  u8 offset;
  u16 lenght;
  union{
    u8 data[16];
    u8 *pdata;
  }payload;
  
  struct _cbus_cmd *prev;
  struct _cbus_cmd *next;
}cbus_cmd_node;

typedef struct{
  u8 intr4_mask_value;
  u8 intr1_mask_value;
  u8 intr_cbus1_mask_value;
  u8 intr_cbus2_mask_value;
  MHL_status_enum_type mhl_status;
  u8 linkmode;
  u8 connected_ready;
  bool cbus_connected;
  bool sink_hpd;
  bool rgnd_1k;
  u8 rsen_check_available;
}mhl_tx_status_type;


typedef struct{
  u8 mhl_ver;
  u8 dev_type;
  u16 adopter_id;
  u8 vid_link_mode;
  u8 aud_link_mode;
  u8 video_type;
  u8 log_dev_map;
  u8 bandwidth;
  u8 feature_flag;
  u16 device_id;
  u8 scratchpad_size;
  u8 int_stat_size;
  
  bool rcp_support;
  bool rap_support;
  bool sp_support;
}mhl_rx_cap_type;

typedef enum{
  RSEN_400M_TIMER = 0x00,

  TIMER_MAX= 0xFF
}mhl_timer_type;

void sii9244_mhl_tx_int(void);
void simg_mhl_tx_handler(void);
void sii9244_cfg_power(bool on);
void MHL_On(bool on);

#define MHL_WAKEUP_TIMER_MODIFY
#if 0
#define SII_DEV_DBG_ERROR(fmt, args...)         printk(KERN_ERR "===== [SII9244 Exception] [%s][%d] :: "fmt, __func__, __LINE__, ##args);
#define SII_DEV_DBG(fmt, args...) printk(KERN_DEBUG "[SII9244][%s] :: " fmt, __func__, ##args);
#else
#define SII_DEV_DBG_ERROR(fmt, args...)
#define SII_DEV_DBG(fmt, args...)
#endif


#ifdef FEATURE_JW_DATA_PROTOCOL
extern mhl_rx_cap_type mhl_reciever_cap;

extern void MhlTxSetInt( u8 regToWrite,u8  mask, u8 priority);
extern void cbus_cmd_bind(u8 command, u8 offset, u8* data, u16 data_lenth, bool priority);
extern void read_cbus_data(u8 offset, unsigned char cnt, unsigned char *data);
#endif
