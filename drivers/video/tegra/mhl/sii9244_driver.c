/*===========================================================================

  SiI9244 Driver Processor


  DESCRIPTION
  This file explains the SiI9024A initialization and call the virtual main function.


  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             
  No part of this work may be reproduced, modified, distributed, transmitted,    
  transcribed, or translated into any language or computer format, in any form   
  or by any means without written permission of: Silicon Image, Inc.,            
  1060 East Arques Avenue, Sunnyvale, California 94085                           
  ===========================================================================*/


/*===========================================================================

  EDIT HISTORY FOR FILE

  when              who                         what, where, why
  --------        ---                        ----------------------------------------------------------
  2010/10/25    Daniel Lee(Philju)      Initial version of file, SIMG Korea 
  ===========================================================================*/

/*===========================================================================
  INCLUDE FILES FOR MODULE
  ===========================================================================*/


#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <asm/uaccess.h> 
#include <linux/hrtimer.h>
#include <linux/ktime.h>


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mm.h>
// #include <asm/hardware.h>
// #include <asm/irq.h>
// #include <asm/arch/irqs.h>       
#include <linux/slab.h>
// #include <asm/arch-pxa/pxa-regs.h>
// #include <linux/interrupt.h>
// #include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/types.h>
// #include <asm/uaccess.h>

#include <mach/gpio.h>

//                  
#include <linux/time.h>
#include <linux/poll.h>

/*
#include "SiI9244_Reg.h"
//#include "Common_Def.h"
#include "SiI9244_I2C_master.h"
#include "SiI9244_I2C_slave_add.h"
#include "si_cbusDefs.h"
#include "si_cbus_regs.h"
#include "si_cbus.h"
#include "si_apiCbus.h"
*/

#include "sii9244_driver.h"

#if 0
#define SII_LOG_FUNCTION_NAME_ENTRY             printk(KERN_INFO "[SII9244]## %s() ++ ##\n",  __func__);
#define SII_LOG_FUNCTION_NAME_EXIT              printk(KERN_INFO "[SII9244]## %s() -- ##\n",  __func__);
#else
#define SII_LOG_FUNCTION_NAME_ENTRY 
#define SII_LOG_FUNCTION_NAME_EXIT  

#endif

//                  
extern struct i2c_client *sii9244_i2c_client;
extern struct i2c_client *sii9244a_i2c_client;
extern struct i2c_client *sii9244b_i2c_client;
extern struct i2c_client *sii9244c_i2c_client;

// for MHL CTS Nov16
//extern void hdmi_restart(void);
extern void rcp_cbus_uevent(u8);		//Subhransu


/* ========================================================

========================================================*/


/* GPIO Pin Output Set Registers */
// jh.koo
#ifdef UNKNOWN_ARCH
#define GPSR0		GPIO_REG(BANK_OFF(0) + 0x18)


/* GPIO Pin Output Clear Registers */
#define GPCR0		GPIO_REG(BANK_OFF(0) + 0x24)


#define GPIO_2		2 // GPIO2
#define GPIO_3		3 // GPIO3
#define GPIO_4		4 // GPIO4
#define GPIO_5		5 // GPIO5

#define MASK_GPIO_2		( 1 << GPIO_2 )
#define MASK_GPIO_3		( 1 << GPIO_3 )
#define MASK_GPIO_4		( 1 << GPIO_4 )
#define MASK_GPIO_5		( 1 << GPIO_5 )
#endif


// #define	SIMG_NINT		IRQ_GPIO(5)
#define	SIMG_NINT			gpio_to_irq(GPIO_MHL_INT)


//static void simg_interrupt_cb(void);
//                                              
static void simg_interrupt_cb(struct work_struct *simg_work);

#ifndef FEATURE_SiI9244_DEVICE
#error "~~~~~~~~~~~~~~~~"
#endif 
	
#ifdef FEATURE_SiI9244_DEVICE
static void mhl_tx_init(void);
static void WriteInitialRegisterValues (void);  
void CbusReset (void);
static void InitCBusRegs (void);
static u8 CBusProcessErrors (u8 intStatus);
int I2C_WriteByte(u8 deviceID, u8 offset, u8 value);
u8 I2C_ReadByte(u8 deviceID, u8 offset);
void TX_GO2D3 (void);
void TX_GoToD0 (void);
//void simg_mhl_tx_handler(void);
void process_waiting_rgnd(void);
void simg_TmdsControl (bool enable);
int process_mhl_discovery_start(void);
u8 TX_RGND_check (void);
void process_mhl_discovery_success(void);
void cbus_cmd_bind(u8 command, u8 offset, u8* data, u16 data_lenth, bool priority);

//                                
// ADD : wrong used work queue function.
// mismatch the dev address pointer.
//static void cbus_cmd_send(void* _dev);
static void cbus_cmd_send(struct work_struct *work);

void mhltx_got_status(u8 status0, u8 status1);
void mhltx_set_status(u8 command, u8 offset, u8 value);
void cbus_cmd_done(void);
void cbus_cmd_put_workqueue(cbus_cmd_node* node_p);
void mhltx_got_mhlintr(u8 data0, u8 data1);
void SiiMhlTxRapkSend(void);
void SiiMhlTxRcpeSend(u8 erro_code, u8 key_code);
void SiiMhlTxRcpkSend(u8 keycode);
void proccess_msc_msg_handle(void);
void MhlTxSetInt( u8 regToWrite,u8  mask, u8 priority);
void delete_all_cbus_cmd(void);
void simg_init_timer(u8 data, unsigned int msecs);
void simg_timer_handler(unsigned long timer_type);
void simg_timer_hanler_cb(void* _data);
static void simg_chip_rst(void);
void ReadModifyWriteCBUS(u8 Offset, u8 Mask, u8 Value);
void ReadModifyWritePage0(u8 Offset, u8 Mask, u8 Data);
void WriteBytePage0 (u8 Offset, u8 Data);
void sii9244_mhl_tx_int(void);
void WriteBytePage0 (u8 Offset, u8 Data);
u8 ReadBytePage0 (u8 Offset);
void process_cbus_interrrupt(void);
void sii9244_driver_init(void);
#ifdef MHL_CTL
void SiiMhlTxReadScratchpad(void);
void SiiMhlTxWriteScratchpad(u8 *wdata);
void MhlControl(void);
extern void hdmi_common_send_uevent(char *buf);
extern int mhl_ms_ptr(signed short x, signed short y);
extern void mhl_ms_hide_cursor(void);
extern int mhl_ms_btn(int action);
extern int mhl_kbd_key(unsigned int tKeyCode, int value);
extern int MHLRCPtoKeyboard(u8 mhlrcpkey);
extern void mhl_writeburst_uevent(unsigned short mev);
static unsigned short kbd_key_pressed[2];
#endif
#endif/*FEATURE_SiI9244_DEVICE*/
//                                                                                      
#ifdef MHL_HW_DEBUG
int mhl_ctrl4 = 0xEB;
#endif
//                                                                                      

//                  
//static DEFINE_SPINLOCK(mhl_timer_lock);
int resen_check_init = FALSE;
int rsen_timer = FALSE;
struct rsen_check_delayed_work  {	
	struct delayed_work		work;
};
struct rsen_check_delayed_work rsen_check_control;
struct timer_list simg_timer;

#include <linux/mutex.h>
static DEFINE_MUTEX(cbus_cmd_bind_mutex);		//                  
extern bool is_MHL_connected(void);


/* ========================================================

========================================================*/

struct simg_state {
	struct i2c_client *client;
};

#ifdef FEATURE_SiI9244_DEVICE
#define	I2C_READ_MODIFY_WRITE(saddr,offset,mask)	I2C_WriteByte(saddr, offset, I2C_ReadByte(saddr, offset) | (mask));
#define ReadModifyWriteByteCBUS(offset,andMask,orMask)  WriteByteCBUS(offset,(ReadByteCBUS(offset)&andMask) | orMask)

#define	SET_BIT(saddr,offset,bitnumber)		I2C_READ_MODIFY_WRITE(saddr,offset, (1<<bitnumber))
#define	CLR_BIT(saddr,offset,bitnumber)		I2C_WriteByte(saddr, offset, I2C_ReadByte(saddr, offset) & ~(1<<bitnumber))


#define	DISABLE_DISCOVERY				CLR_BIT(PAGE_0_0X72, 0x90, 0);
#define	ENABLE_DISCOVERY				SET_BIT(PAGE_0_0X72, 0x90, 0);

#define STROBE_POWER_ON                    CLR_BIT(PAGE_0_0X72, 0x90, 1);

#define RSEN_HIGH BIT2


/*========================================================================*/
//
//	Look for interrupts on INTR4 (Register 0x74)
//		7 = RSVD		(reserved)
//		6 = RGND Rdy	(interested)
//		5 = VBUS Low	(ignore)	
//		4 = CBUS LKOUT	(interested)
//		3 = USB EST		(interested)
//		2 = MHL EST		(interested)
//		1 = RPWR5V Change	(ignore)
//		0 = SCDT Change	(only if necessary)
//
#define	INTR_4_DESIRED_MASK				(BIT0 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6) 
#define USB_EST_INTR4 BIT3
#define MHL_EST_INTR4 BIT2
#define CBUS_LOCKOUT BIT4
#define RGND_RDY BIT6
#define SCDT_CHANGE BIT0
#define	MASK_INTR_4_INTERRUPTS		I2C_WriteByte(PAGE_0_0X72, 0x78, INTR_4_DESIRED_MASK)
#define	UNMASK_INTR_4_INTERRUPTS	I2C_WriteByte(PAGE_0_0X72, 0x78, 0x00)
#define RGND_INTR4_INTERRUPT      I2C_WriteByte(PAGE_0_0X72, 0x78, BIT6)
#define WRITE_INTR4_VALUE(x)      I2C_WriteByte(PAGE_0_0X72, 0x78, x)  

/*========================================================================*/
//	Look for interrupts on INTR_2 (Register 0x72)
//		7 = bcap done			(ignore)
//		6 = parity error		(ignore)
//		5 = ENC_EN changed		(ignore)
//		4 = no premable			(ignore)
//		3 = ACR CTS changed		(ignore)
//		2 = ACR Pkt Ovrwrt		(ignore)
//		1 = TCLK_STBL changed	(interested)
//		0 = Vsync				(ignore)

#define	INTR_2_DESIRED_MASK				(BIT1)
#define	MASK_INTR_2_INTERRUPTS		I2C_WriteByte(PAGE_0_0X72, 0x76, INTR_2_DESIRED_MASK)
#define	UNMASK_INTR_2_INTERRUPTS			I2C_WriteByte(PAGE_0_0X72, 0x76, 0x00)

/*========================================================================*/
//	Look for interrupts on INTR_1 (Register 0x71)
//		7 = RSVD		(reserved)
//		6 = MDI_HPD		(interested)
//		5 = RSEN CHANGED(interested)	
//		4 = RSVD		(reserved)
//		3 = RSVD		(reserved)
//		2 = RSVD		(reserved)
//		1 = RSVD		(reserved)
//		0 = RSVD		(reserved)

#define	INTR_1_DESIRED_MASK				(BIT5|BIT6) 
#define MDI_HPD (BIT6)
#define RSEN_CHANG (BIT5) 
#define	MASK_INTR_1_INTERRUPTS		    I2C_WriteByte(PAGE_0_0X72, 0x75, INTR_1_DESIRED_MASK)
#define	UNMASK_INTR_1_INTERRUPTS			I2C_WriteByte(PAGE_0_0X72, 0x75, 0x00)
#define WRITE_INTR1_VALUE(x)          I2C_WriteByte(PAGE_0_0X72, 0x75, x)  

/*========================================================================*/
//	Look for interrupts on CBUS:CBUS_INTR_STATUS [0xC8:0x08]
//		7 = RSVD			(reserved)
//		6 = MSC_RESP_ABORT	(interested)
//		5 = MSC_REQ_ABORT	(interested)	
//		4 = MSC_REQ_DONE	(interested)
//		3 = MSC_MSG_RCVD	(interested)
//		2 = DDC_ABORT		(interested)
//		1 = RSVD			(reserved)
//		0 = rsvd			(reserved)
#define	INTR_CBUS1_DESIRED_MASK			(BIT2 | BIT3 | BIT4 | BIT5 | BIT6)
#define MSC_RESP_ABORT BIT6
#define MSC_REQ_ABORT BIT5
#define MSC_REQ_DONE BIT4
#define MSC_MSG_RCVD BIT3 
#define DDC_ABORT BIT2
#define	MASK_CBUS1_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x09, INTR_CBUS1_DESIRED_MASK)
#define	UNMASK_CBUS1_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x09, 0x00)
#define WRITE_CBUS1_VALUE(x)  I2C_WriteByte(PAGE_CBUS_0XC8, 0x09, x) 

/*========================================================================*/
//                                                               
#ifdef MHL_CTL
#define	INTR_CBUS2_DESIRED_MASK			(BIT0 | BIT2 | BIT3)
#else
#define	INTR_CBUS2_DESIRED_MASK			(BIT2 | BIT3)
#endif
//                                                               
#define	MASK_CBUS2_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x1F, INTR_CBUS2_DESIRED_MASK)
#define	UNMASK_CBUS2_INTERRUPTS			I2C_WriteByte(PAGE_CBUS_0XC8, 0x1F, 0x00)
#define WRITE_CBUS2_VALUE(x)  I2C_WriteByte(PAGE_CBUS_0XC8, 0x1F, x) 
#ifdef MHL_CTL
#define	MAGIC_CANVAS_X	1280
#define	MAGIC_CANVAS_Y	720

static int	tvCtl_x = MAGIC_CANVAS_X;
static int	tvCtl_y = MAGIC_CANVAS_Y;
#endif
/*========================================================================*/
#define	MHL_MAX_RCP_KEY_CODE	(0x7F + 1)	// inclusive

u8 rcpSupportTable [MHL_MAX_RCP_KEY_CODE] = {
	(MHL_DEV_LD_GUI),		// 0x00 = Select
	(MHL_DEV_LD_GUI),		// 0x01 = Up
	(MHL_DEV_LD_GUI),		// 0x02 = Down
	(MHL_DEV_LD_GUI),		// 0x03 = Left
	(MHL_DEV_LD_GUI),		// 0x04 = Right
	0, 0, 0, 0,				// 05-08 Reserved
	(MHL_DEV_LD_GUI),		// 0x09 = Root Menu
	0, 0, 0,				// 0A-0C Reserved
	(MHL_DEV_LD_GUI),		// 0x0D = Select
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0E-1F Reserved
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Numeric keys 0x20-0x29
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	0,						// 0x2A = Dot
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Enter key = 0x2B
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Clear key = 0x2C
	0, 0, 0,				// 2D-2F Reserved
	(MHL_DEV_LD_TUNER),		// 0x30 = Channel Up
	(MHL_DEV_LD_TUNER),		// 0x31 = Channel Dn
	(MHL_DEV_LD_TUNER),		// 0x32 = Previous Channel
	(MHL_DEV_LD_AUDIO),		// 0x33 = Sound Select
	0,						// 0x34 = Input Select
	0,						// 0x35 = Show Information
	0,						// 0x36 = Help
	0,						// 0x37 = Page Up
	0,						// 0x38 = Page Down
	0, 0, 0, 0, 0, 0, 0,	// 0x39-0x3F Reserved
	0,						// 0x40 = Undefined

	(MHL_DEV_LD_SPEAKER),	// 0x41 = Volume Up
	(MHL_DEV_LD_SPEAKER),	// 0x42 = Volume Down
	(MHL_DEV_LD_SPEAKER),	// 0x43 = Mute
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x44 = Play
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x45 = Stop
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x46 = Pause
	(MHL_DEV_LD_RECORD),	// 0x47 = Record
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x48 = Rewind
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x49 = Fast Forward
	(MHL_DEV_LD_MEDIA),		// 0x4A = Eject
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),	// 0x4B = Forward
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),	// 0x4C = Backward
	0, 0, 0,				// 4D-4F Reserved
	0,						// 0x50 = Angle
	0,						// 0x51 = Subpicture
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 52-5F Reserved
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x60 = Play Function
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x61 = Pause the Play Function
	(MHL_DEV_LD_RECORD),	// 0x62 = Record Function
	(MHL_DEV_LD_RECORD),	// 0x63 = Pause the Record Function
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x64 = Stop Function

	(MHL_DEV_LD_SPEAKER),	// 0x65 = Mute Function
	(MHL_DEV_LD_SPEAKER),	// 0x66 = Restore Mute Function
	0, 0, 0, 0, 0, 0, 0, 0, 0, 	                        // 0x67-0x6F Undefined or reserved
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 		// 0x70-0x7F Undefined or reserved
};

#ifdef MHL_CTL
static	mhl_ctl_data_t	mhl_ctl_data;
#endif
mhl_rx_cap_type mhl_reciever_cap;
mhl_tx_status_type mhl_status_value;
cbus_cmd_node *cbus_cmd_head = NULL, *cbus_cmd_tail = NULL;

#endif /*FEATURE_SiI9244_DEVICE*/

//              
#if 0
static struct work_struct simg_interrupt_work = {
	simg_interrupt_cb
};
#else
static struct work_struct simg_interrupt_work;
#endif

struct i2c_client* get_simgI2C_client(u8 device_id)
{

	struct i2c_client* client_ptr;

	if(device_id == 0x72)
		client_ptr = sii9244_i2c_client;
	else if(device_id == 0x7A)
		client_ptr = sii9244a_i2c_client;
	else if(device_id == 0x92)
		client_ptr = sii9244b_i2c_client;
	else if(device_id == 0xC8)
		client_ptr = sii9244c_i2c_client;
	else
		client_ptr = NULL;
    

	return client_ptr;
}





/*=============================================================


=============================================================*/

//               
u8 gpio_outb( u8 set, u8 gpio_no )
{
//               
/*
  if(set)
  {
  	GPSR0 |= ( gpio_no );	
  }
  else
  {
	  GPCR0  = GPCR0 | (gpio_no);
  }
*/
	return 0;
}

/*=============================================================


=============================================================*/

#define MSECS_TO_JIFFIES(ms) (((ms)*HZ+999)/1000)

signed long simg_schedule_timeout_uninterruptible(signed long timeout)
{
    signed long rc = 0;

	SII_LOG_FUNCTION_NAME_ENTRY;
	__set_current_state(TASK_UNINTERRUPTIBLE);
	rc = schedule_timeout(timeout);
	SII_LOG_FUNCTION_NAME_EXIT;
	//                                       
	return rc;
}


void simg_msleep(unsigned int msecs)
{
	unsigned long timeout = MSECS_TO_JIFFIES(msecs) + 1;

	while (timeout)
	{
		timeout = simg_schedule_timeout_uninterruptible(timeout);
	}
}

/*=============================================================


=============================================================*/

// added
void simg_init_timer(u8 data, unsigned int msecs)
{
  //init_timer(&simg_timer);
  //simg_timer.function = simg_timer_handler;
  simg_timer.data = data;
  simg_timer.expires = jiffies + (MSECS_TO_JIFFIES(msecs) + 1); // 2 * HZ;
  add_timer(&simg_timer);
}
EXPORT_SYMBOL(simg_timer);		// kibum.leelge.com

//                                                     
//                                                  
static irqreturn_t simg_int_irq_handler(int irq, void *devid)
{
    SII_LOG_FUNCTION_NAME_ENTRY;
 	schedule_work(&simg_interrupt_work);
    SII_LOG_FUNCTION_NAME_EXIT;
	return IRQ_HANDLED;		//                  
}

void simg_interrupt_enable(bool flag)
{
//    int ret=0;
//    struct simg_state *state = NULL;
    SII_LOG_FUNCTION_NAME_ENTRY;

	if(flag == TRUE)
	{
		//printk(KERN_INFO "simg_interrupt enable \n");  
		//    set_GPIO_IRQ_edge( GPIO_5, GPIO_FALLING_EDGE   ); 
		//    ret = request_irq(SIMG_NINT, simg_int_irq_handler, SA_INTERRUPT  /* 0*/, "SIMG Interrupt"  , (void*)state);	
		//ret = request_threaded_irq(gpio_to_irq(GPIO_MHL_INT), NULL, simg_int_irq_handler,
		//		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "SIMG Interrupt", (void *) state); 
		//
		//if(ret != 0)
		//{
		 //   free_irq(SIMG_NINT, NULL);
		//}

		enable_irq(gpio_to_irq(GPIO_MHL_INT));		//                  
	}
	else
	{
		//SII_DEV_DBG("free irq 321 but now not free.\n");
		//free_irq(SIMG_NINT, NULL);
		disable_irq(gpio_to_irq(GPIO_MHL_INT));		//                  
	}
    SII_LOG_FUNCTION_NAME_EXIT;
}


#ifndef FEATURE_SiI9244_DEVICE
#error "~~~~~~~~~~~~~~~~"
#endif 

#ifdef FEATURE_SiI9244_DEVICE

u8 I2C_ReadByte(u8 deviceID, u8 offset)
{
    struct i2c_msg msg[2];
    u8 reg_buf[] = { offset };
    u8 data_buf[] = { 0 };
    int err;

    struct i2c_client* client_ptr = get_simgI2C_client(deviceID);

    if(!client_ptr)
    {
        SII_DEV_DBG_ERROR("[MHL]I2C_ReadByte error %x\n",deviceID); 
        return 0;	
    }

	//                                      
	if(is_MHL_connected() == false)
	{
        //SII_DEV_DBG("[MHL] mhl disconnected\n");
      	  return 0;			
	}
  
    msg[0].addr = client_ptr->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = reg_buf;

    msg[1].addr = client_ptr->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = data_buf;

    err = i2c_transfer(client_ptr->adapter, msg, 2);

    if (err < 0) {
        SII_DEV_DBG_ERROR("I2C err: %d\n", err);
        return 0xFF; //err;
    }  

    //printk(KERN_INFO "$$$$ I2C 7A data :%x @@@ %x\n",*data_buf, data_buf[0] );

    return *data_buf;
}



int I2C_WriteByte(u8 deviceID, u8 offset, u8 value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	struct i2c_client* client_ptr = get_simgI2C_client(deviceID);

	//                                      
	if(is_MHL_connected() == false)
	{
        //SII_DEV_DBG("[MHL] mhl disconnected\n");
      	  return 0;			
	}
	
    msg->addr = client_ptr->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = offset;
	data[1] = value;

	err = i2c_transfer(client_ptr->adapter, msg, 1);
	
	if (err >= 0)
	{
	    return 0;
    }
  
	SII_DEV_DBG("I2C write err: %d\n", err);
	return err;
}


u8 ReadByteCBUS (u8 Offset)
{
	return I2C_ReadByte(PAGE_CBUS_0XC8, Offset);
}


void WriteByteCBUS(u8 Offset, u8 Data)
{
	I2C_WriteByte(PAGE_CBUS_0XC8, Offset, Data);
}


void read_cbus_data(u8 offset, unsigned char cnt, unsigned char *data)
{
  unsigned char i;

  for(i=0; i<cnt; i++)
  {
    *data++ = ReadByteCBUS(offset+i);
  }
}


u8 ReadBytePage0 (u8 Offset)
{
	return I2C_ReadByte(PAGE_0_0X72, Offset);
}

void WriteBytePage0 (u8 Offset, u8 Data)
{
	I2C_WriteByte(PAGE_0_0X72, Offset, Data);
}


void ReadModifyWritePage0(u8 Offset, u8 Mask, u8 Data)
{

	u8 Temp;

	Temp = ReadBytePage0(Offset);		// Read the current value of the register.
	Temp &= ~Mask;					// Clear the bits that are set in Mask.
	Temp |= (Data & Mask);			// OR in new value. Apply Mask to Value for safety.
	WriteBytePage0(Offset, Temp);		// Write new value back to register.
}

void ReadModifyWriteCBUS(u8 Offset, u8 Mask, u8 Value)
{
  u8 Temp;

  Temp = ReadByteCBUS(Offset);
  Temp &= ~Mask;
  Temp |= (Value & Mask);
  WriteByteCBUS(Offset, Temp);
}


static void simg_chip_rst(void)
{
  //skip power on in this test code
  SII_DEV_DBG("simg_power RST ####\n");

#ifdef UNKNOWN_ARCH
/*
  gpio_outb(FALSE,MASK_GPIO_3); 
  simg_msleep(10);
  gpio_outb(TRUE,MASK_GPIO_3); 
 */
#else
//	sii9244_cfg_power(1);
	gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW);
	gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
	mdelay(2);
	gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
	mdelay(2);
	gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
#endif
  
}

//static void simg_interrupt_cb(void)
//                                               
static void simg_interrupt_cb(struct work_struct *simg_work)
{
    static int function_start = 1;   //for test....		// for MHL CTS
    SII_LOG_FUNCTION_NAME_ENTRY;
    if(function_start == 0)
    {
        function_start = 1;
        // test code start....
        simg_interrupt_enable(FALSE);
        simg_msleep(1000);
        SII_DEV_DBG("SiI9244 function start ## **\n");
        //mdelay(1000);
        simg_msleep(1000);      
        simg_chip_rst();
        //simg_interrupt_enable(TRUE);
        sii9244_mhl_tx_int();    
        simg_interrupt_enable(TRUE);
    }
    else
    {
        SII_DEV_DBG(" ##### (Line:%d) simg_mhl_tx_handler START #####\n", (int)__LINE__ );

#if 1
        simg_mhl_tx_handler();
#else

        if(simg_mhl_tx_handler())
        {      
            printk(KERN_INFO " ##### (Line:%d) simg_mhl_tx_handler error #####\n", (int)__LINE__ );
            simg_interrupt_enable(FALSE);
            gpio_interrupt_pin_input_mode();

            while(read_interrupt_pin_level() == 0x00)
            {
                simg_mhl_tx_handler();
            }

            //printk(KERN_INFO " ##### (Line:%d) simg_mhl_tx_handler END ##### \n", (int)__LINE__ );
            simg_interrupt_enable(TRUE);

        }
#endif
     SII_DEV_DBG(  " ##### (Line:%d) simg_mhl_tx_handler Final END ##### \n", (int)__LINE__ );
  
    }
    SII_LOG_FUNCTION_NAME_EXIT;
}

static void mhl_tx_init(void)
{
    //                                                   
    //                                        
    sii9244_mhl_tx_int(); 
    //                                                 
}


static u8 CBusProcessErrors (u8 intStatus)
{
    u8 result          = 0;
    u8 mscAbortReason  = 0;
//    u8 ddcAbortReason  = 0;

    /* At this point, we only need to look at the abort interrupts. */

    intStatus &= (BIT_MSC_ABORT | BIT_MSC_XFR_ABORT);

    if ( intStatus )
    {
        /* If transfer abort or MSC abort, clear the abort reason register. */
// for MHL CTS Nov.18 - block ddc abort
#if 0
        if( intStatus & BIT_DDC_ABORT )
        {
            result = ddcAbortReason = ReadByteCBUS( REG_DDC_ABORT_REASON );
            SII_DEV_DBG_ERROR("CBUS DDC ABORT happened, reason:: %02X\n", (int)(ddcAbortReason));
        }
#endif
        if ( intStatus & BIT_MSC_XFR_ABORT )
        {
            //result = mscAbortReason = ReadByteCBUS( REG_DDC_ABORT_REASON );
            result = mscAbortReason = ReadByteCBUS( REG_PRI_XFR_ABORT_REASON );	//                  

            SII_DEV_DBG_ERROR("CBUS:: MSC Transfer ABORTED. Clearing 0x0D\n");
            WriteByteCBUS( REG_PRI_XFR_ABORT_REASON, 0xFF );
        }
        if ( intStatus & BIT_MSC_ABORT )
        {
	      result = mscAbortReason = ReadByteCBUS( REG_CBUS_PRI_FWR_ABORT_REASON );	//                  
            SII_DEV_DBG_ERROR("CBUS:: MSC Peer sent an ABORT. Clearing 0x0E\n");
            WriteByteCBUS( REG_CBUS_PRI_FWR_ABORT_REASON, 0xFF );
        }

        // Now display the abort reason.

        if ( mscAbortReason != 0 )
        {
            SII_DEV_DBG("CBUS:: Reason for ABORT is ....0x%02X = ", (int)mscAbortReason );

            if ( mscAbortReason & CBUSABORT_BIT_REQ_MAXFAIL)
            {
                SII_DEV_DBG_ERROR( "Requestor MAXFAIL - retry threshold exceeded\n");
            }
            if ( mscAbortReason & CBUSABORT_BIT_PROTOCOL_ERROR)
            {
                SII_DEV_DBG_ERROR( "Protocol Error\n");
            }
            if ( mscAbortReason & CBUSABORT_BIT_REQ_TIMEOUT)
            {
                SII_DEV_DBG_ERROR( "Requestor translation layer timeout\n");
            }
            if ( mscAbortReason & CBUSABORT_BIT_PEER_ABORTED)
            {
                SII_DEV_DBG_ERROR( "Peer sent an abort\n");
            }
            if ( mscAbortReason & CBUSABORT_BIT_UNDEFINED_OPCODE)
            {
                SII_DEV_DBG_ERROR( "Undefined opcode\n");
            }
        }
    }
    return result;
}




void simg_mhl_tx_handler(void)
{
    u8 tmp;
    SII_LOG_FUNCTION_NAME_ENTRY;

    switch (mhl_status_value.mhl_status)
    {
        case MHL_WAITING_RGND_DETECT:
            SII_DEV_DBG(  "MHL_INIT_DONE \n");
            process_waiting_rgnd();
            break;

        case MHL_DISCOVERY_FAIL:
            SII_DEV_DBG("MHL_DISCOVERY_FAIL\n");          
            break;

        case MHL_CABLE_CONNECT:
            SII_DEV_DBG(  "(Line:%d) MHL_RGND_DETECT \n", (int)__LINE__ );
            break;

        case MHL_DISCOVERY_START:
            SII_DEV_DBG( "(Line:%d) MHL_DISCOVERY_START \n", (int)__LINE__ );
            tmp = ReadByteCBUS(0x08);
            WriteByteCBUS(0x08, tmp);
            tmp = ReadByteCBUS(0x1E);
            WriteByteCBUS(0x1E, tmp);    		
            process_mhl_discovery_start();
            break;

        case MHL_DISCOVERY_SUCCESS:
        case MHL_RSEN_LOW:
            SII_DEV_DBG(  "(Line:%d) MHL_DISCOVERY_SUCCESS \n", (int)__LINE__ );          
            process_mhl_discovery_success();
            break;

        default:
            SII_DEV_DBG_ERROR(  "(Line:%d) DEAFAULT:%x \n", (int)__LINE__, mhl_status_value.mhl_status );          
            break;
    }
    
    SII_LOG_FUNCTION_NAME_EXIT;
}
EXPORT_SYMBOL(simg_mhl_tx_handler);



static void ForceUsbIdSwitchOpen ( void )
{
  DISABLE_DISCOVERY; 		// Disable CBUS discovery

  ReadModifyWritePage0(0x95, BIT6, BIT6);	// Force USB ID switch to open
  //WriteBytePage0(0x92, 0x86);
  WriteBytePage0(0x92, 0xa6);		// for CTS 20111118
  ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	// Force upstream HPD to 0 when not in MHL mode.
}


static void ReleaseUsbIdSwitchOpen ( void )
{
    SII_LOG_FUNCTION_NAME_ENTRY;
	simg_msleep(50); // per spec
	// Release USB ID switch
	ReadModifyWritePage0(0x95, BIT6, 0x00);
	ENABLE_DISCOVERY;
    SII_LOG_FUNCTION_NAME_EXIT;
}


static	void	ApplyDdcAbortSafety(void)
{
    u8 bTemp, bPost;
    SII_LOG_FUNCTION_NAME_ENTRY;

	WriteByteCBUS(0x29, 0xFF);  // clear the ddc abort counter
	bTemp = ReadByteCBUS(0x29);  // get the counter
	simg_msleep(3);
	bPost = ReadByteCBUS(0x29);  // get another value of the counter

	if (bPost > (bTemp + 50))
	{
        CbusReset();
        InitCBusRegs();
        ForceUsbIdSwitchOpen();
        ReleaseUsbIdSwitchOpen();

        I2C_WriteByte(PAGE_0_0X72, 0xA0, 0xD0);
        simg_TmdsControl(FALSE);

        sii9244_mhl_tx_int(); 
	}
    SII_LOG_FUNCTION_NAME_EXIT;
}


void process_cbus_interrrupt(void)
{
    u8 intr_cbus1, intr_cbus2;
    SII_LOG_FUNCTION_NAME_ENTRY;

    intr_cbus1 = ReadByteCBUS(0x08);
    SII_DEV_DBG(  "intr_cbus1 = %x\n",intr_cbus1);
    WriteByteCBUS(0x08, intr_cbus1); //clear interrupt.

	if (intr_cbus1 & BIT2)
	{
		ApplyDdcAbortSafety();
	}

	if((intr_cbus1 & BIT3))
	{
        SII_DEV_DBG(  "Drv: MSC_MSG/RAP Received\n");
        proccess_msc_msg_handle();
	}
  
	if((intr_cbus1 & BIT5) || (intr_cbus1 & BIT6))	// MSC_REQ_ABORT or MSC_RESP_ABORT
	{
	    u8 cbus_error = 0;
		SII_DEV_DBG (  "((cbusInt & BIT5) || (cbusInt & BIT6)) \n");	
		cbus_error = CBusProcessErrors(intr_cbus1);
#if 0	// change for MHL CTS Nov.16
        if(intr_cbus1)
        {
            delete_all_cbus_cmd(); 
            mhl_tx_init();
            SII_DEV_DBG_ERROR("intr_cbus1 is true\n");
            return;
        }
#endif
	}
 
	// MSC_REQ_DONE received.
	if(intr_cbus1 & BIT4)
	{
        cbus_cmd_done();
	}

    intr_cbus2 = ReadByteCBUS(0x1E);
    SII_DEV_DBG(  "intr_cbus2 = %x\n",intr_cbus2);
    WriteByteCBUS(0x1E, intr_cbus2); //clear interrupt.  
    
    if(intr_cbus2 & INTR_CBUS2_DESIRED_MASK)
    {
#ifdef MHL_CTL
        if(intr_cbus2& BIT0)
        {
            SII_DEV_DBG("intr_cbus2& BIT0\n");
            SiiMhlTxReadScratchpad();
        }
#endif    
        if(intr_cbus2& BIT2)
        {
            u8 cbus_intr_set0, cbus_intr_set1;
            SII_DEV_DBG("intr_cbus2& BIT2\n");

            cbus_intr_set0 = ReadByteCBUS(0xA0);
            WriteByteCBUS( 0xA0, cbus_intr_set0);

            cbus_intr_set1 = ReadByteCBUS(0xA1);
            WriteByteCBUS( 0xA1, cbus_intr_set1);

            mhltx_got_mhlintr( cbus_intr_set0, cbus_intr_set1 );      
        }

        if(intr_cbus2& BIT3)    
        {
            u8 status[2];

            SII_DEV_DBG("intr_cbus2& BIT3\n");

            status[0] = ReadByteCBUS(0xB0);
            WriteByteCBUS( 0xB0, 0xFF);

            status[1] = ReadByteCBUS(0xB1);
            WriteByteCBUS( 0xB1, 0xFF);      
            mhltx_got_status(status[0], status[1]);  		
        }
    }   
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
}

void process_mhl_discovery_success(void)
{
    u8 intr4, intr1;
    SII_LOG_FUNCTION_NAME_ENTRY;

    intr1 = I2C_ReadByte(PAGE_0_0X72, 0x71);
    I2C_WriteByte(PAGE_0_0X72, 0x71, intr1); //clear interrupt.

    SII_DEV_DBG(  "intr1:%x, mask:%x\n", intr1 , mhl_status_value.intr1_mask_value); 
  
    if(intr1&RSEN_CHANG)
    {
        u8 sys_stat =0;

        sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
        SII_DEV_DBG( "RSEN_CHANG sys_stat: %x ~ \n", sys_stat);

        if(((sys_stat & RSEN_HIGH) == 0)&&(mhl_status_value.rsen_check_available == TRUE))
        {
            SII_DEV_DBG(  "RSEN LOW ~ \n");
            mhl_status_value.mhl_status = MHL_RSEN_LOW;

            simg_msleep(110);

            sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
            SII_DEV_DBG(  "sys_stat: %x ~ \n", sys_stat);

            if((sys_stat & RSEN_HIGH) == 0)
            {
                SII_DEV_DBG(  "RSEN Really LOW ~ \n");
		    printk("2army, process_mhl_discovery_success -> delete_all_cbus_cmd\n"); 
			
                delete_all_cbus_cmd(); 
                ForceUsbIdSwitchOpen();
                simg_msleep(50);			//                  
                ReleaseUsbIdSwitchOpen();

                //customer need to change the USB PATH
				// for MHL CST - Jan 16
                MHL_On(0);
//				sii9244_mhl_tx_int();

                goto exit_func;
            }
			else
			{
				if(rsen_timer)
				{
					del_timer(&simg_timer);
					rsen_timer = FALSE;
				}
				printk(KERN_INFO "RSEN HIGH ~ \n");
				mhl_status_value.mhl_status = MHL_DISCOVERY_SUCCESS;
			}

        }
        else
        {
			if(rsen_timer)
			{
				del_timer(&simg_timer);
				rsen_timer = FALSE;
			}      
            SII_DEV_DBG(  "RSEN HIGH ~ \n");
        }
    }

    if(intr1&MDI_HPD)
    {
        u8 tmp;

        tmp = ReadByteCBUS(0x0D);
        SII_DEV_DBG("======== HPD :%x  ~ \n", tmp);

        if(tmp & 0x40)
        {
            mhl_status_value.sink_hpd = TRUE;
            mhltx_set_status(MHL_WRITE_STAT, MHL_STATUS_REG_LINK_MODE,mhl_status_value.linkmode );
            simg_TmdsControl(TRUE); 
        }
        else
        {
            mhl_status_value.sink_hpd = FALSE;
            simg_TmdsControl(FALSE);  
        }

    } 

    intr4 = I2C_ReadByte(PAGE_0_0X72, 0x74);
    I2C_WriteByte(PAGE_0_0X72, 0x74, intr4); //clear interrupt.

    //(CBUS_LOCKOUT|SCDT_CHANGE)
    if(intr4&SCDT_CHANGE)
    {
        SII_DEV_DBG_ERROR("SCDT_CHANGE !!!\n");  //currently not handle...
    }

    if(intr4&CBUS_LOCKOUT)
    {
        SII_DEV_DBG_ERROR(  "CBUS_LOCKOUT !!!\n"); 
        ForceUsbIdSwitchOpen();
        ReleaseUsbIdSwitchOpen();
        return;
    }

    process_cbus_interrrupt();

    printk("[MHL] process_mhl_discovery_success: ************** Normal finish... **************\n");
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
  
exit_func:
    SII_DEV_DBG_ERROR(  "Goto exit_func \n" ); 
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
}


void proccess_msc_msg_handle(void)
{
    u8 subcmd, msg_data;

    subcmd = ReadByteCBUS( 0x18 );
    msg_data = ReadByteCBUS( 0x19 );  

    switch(subcmd)
    {
        case	MHL_MSC_MSG_RAP:

            if( MHL_RAP_CONTENT_ON == msg_data)
            {
                simg_TmdsControl(TRUE);
            }
            else if( MHL_RAP_CONTENT_OFF == msg_data)
            {
                simg_TmdsControl( FALSE );
            }

            SiiMhlTxRapkSend();
            break;

        case	MHL_MSC_MSG_RCP:
		#ifdef MHL_CTL
			if(((0x41<msg_data) && (msg_data<0x4D)) ||
				((0x00<=msg_data) && (msg_data<0x0E)) ||
				((0x1F<msg_data) && (msg_data<0x2C)) )
			{
		#endif
            SII_DEV_DBG(  "\n MhlTx:%d MHL_MSC_MSG_RCP Key:%x \n",(int)__LINE__, msg_data );

            if(((int)msg_data < MHL_MAX_RCP_KEY_CODE) 
							&& (MHL_LOGICAL_DEVICE_MAP & rcpSupportTable [msg_data & 0x7F] ))
            {
                // need to send RCP keycode...
				rcp_cbus_uevent(msg_data);	//MHL v1 //NAGSM_Android_SEL_Kernel_Aakash_20101126

				printk(KERN_ERR "Key Code:%d \n",msg_data);       
				SiiMhlTxRcpkSend(msg_data);
            }
            else
            {
                SiiMhlTxRcpeSend( RCPE_INEEFECTIVE_KEY_CODE, msg_data );
            }
		#ifdef MHL_CTL
			}
			else
			{
				if(msg_data > 0)
				{
					SiiMhlTxRcpkSend(msg_data);	// MHL_CTL, RCP ACK...right after app key process...but just test here
				}
				else
				{
					SiiMhlTxRcpeSend( RCPE_INEEFECTIVE_KEY_CODE, msg_data );
				}
			}
			#endif
						
            break;

        case	MHL_MSC_MSG_RCPK:
            SII_DEV_DBG(  "MhlTx:%d SiiMhlTxGetEvents RCPK\n",(int)__LINE__ );
            break;

        case	MHL_MSC_MSG_RCPE:
            SII_DEV_DBG(  "MhlTx:%d SiiMhlTxGetEvents MHL_MSC_MSG_RCPE \n",(int)__LINE__ );
            break;

        case	MHL_MSC_MSG_RAPK:
            SII_DEV_DBG( "MhlTx:%d SiiMhlTxGetEvents RAPK\n",(int)__LINE__ );
            break;

        default:
            // Any freak value here would continue with no event to app
            SII_DEV_DBG_ERROR( "default Case\n");
            break;
    }
    return;
}


void SiiMhlTxRcpeSend(u8 erro_code, u8 key_code)
{
  u8 data[3];

  data[0] = MHL_MSC_MSG_RCPE;
  data[1] = erro_code;
  data[2] = key_code;
  
  cbus_cmd_bind(MHL_MSC_MSG,0x00,data,0x03, FALSE ); //don't copy keycode data.
}

void SiiMhlTxRcpkSend(u8 keycode)
{
  u8 data[2];

  data[0] = MHL_MSC_MSG_RCPK;
  data[1] = keycode;
  cbus_cmd_bind(MHL_MSC_MSG,0x00,data,0x02, FALSE );
}


void SiiMhlTxRapkSend(void)
{
  u8 data[2];

  data[0] = MHL_MSC_MSG_RAPK;
  data[1] = 0x00;
  cbus_cmd_bind(MHL_MSC_MSG,0x00,data,0x02, FALSE );
}


void mhltx_got_mhlintr(u8 data0, u8 data1)
{
    u8 tmp[0];

    if(MHL_INT_DCAP_CHG & data0)
    {
        SII_DEV_DBG(  "(Line:%d)  MHL_INT_DCAP_CHG \n", (int)__LINE__ );      
        cbus_cmd_bind(MHL_READ_DEVCAP, MHL_DEV_CATEGORY_OFFSET,tmp,0,FALSE);	
        cbus_cmd_bind(MHL_READ_DEVCAP,MHL_CAP_FEATURE_FLAG,tmp, 0, FALSE);
    }
#ifdef MHL_CTL
    if( MHL_INT_DSCR_CHG & data0)
    {
        SII_DEV_DBG(  "(Line:%d)  MHL_INT_DSCR_CHG \n", (int)__LINE__ ); 
        SiiMhlTxReadScratchpad();
        //c_DSCR_CHG_handle_func();
    }
  
    if( MHL_INT_REQ_WRT  & data0)
    {
        SII_DEV_DBG(  "(Line:%d)  MHL_INT_REQ_WRT \n", (int)__LINE__ ); 
        MhlTxSetInt( MHL_RCHANGE_INT, MHL_INT_GRT_WRT, FALSE);
        //SiiMhlTxWritePeerRegister(MHL_SET_INT, MHL_RCHANGE_INT, MHL_INT_GRT_WRT);
    }
#endif  
    if( MHL_INT_GRT_WRT  & data0)
    {
        SII_DEV_DBG(  "(Line:%d)  MHL_INT_GRT_WRT \n", (int)__LINE__ ); 
        //c_GRT_WRT_handle_func();
    }

    if(MHL_INT_EDID_CHG & data1)
    {
        SII_DEV_DBG(  "(Line:%d)  MHL_INT_EDID_CHG \n", (int)__LINE__ ); 

		// for MHL CTS Nov16
		
		//hdmi_restart();

        ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	// Force upstream HPD to 0 when not in MHL mode.
        simg_msleep(110);
        SET_BIT(PAGE_0_0X72, 0x79,  5);
        CLR_BIT(PAGE_0_0X72, 0x79, 4);
    }  
}


#ifdef MHL_CTL
#if 0
void SiiMhlTxDrvGetScratchPad (u8 startReg,u8 *pData,u8 length)
{
    u16 i;
    for (i = 0; i < length;++i,++startReg)
    {
        *pData++ = ReadByteCBUS(0xC0 + startReg);
    }
}
#endif
void SiiMhlTxReadScratchpad(void)
{
	int m_idx = 0;

	memset(mhl_ctl_data.mscScratchpadData, 0x00, MHD_SCRATCHPAD_SIZE);

	for(m_idx=0; m_idx<(MHD_SCRATCHPAD_SIZE/2); m_idx++)
	{
		mhl_ctl_data.mscScratchpadData[m_idx] = ReadByteCBUS(0xC0+m_idx);
		SII_DEV_DBG("%s, [%d] %x\n", __func__, m_idx, mhl_ctl_data.mscScratchpadData[m_idx]);
	}

	// TODO : clear scratchpad register
	MhlControl();
}
#if 0
void SiiMhlTxWriteScratchpad(u8 *wdata)
{
	printk("%s\n", __func__);
	cbus_cmd_bind(MHL_WRITE_BURST,0x40,wdata,MHD_MAX_BUFFER_SIZE, TRUE );
}
#else
void SiiMhlTxWriteScratchpad(u8 *wdata)
{
	int i;
	printk("%s\n", __func__);
	
	WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD & 0xFF), 0x40);
	WriteByteCBUS( (REG_MSC_WRITE_BURST_LEN & 0xFF), MHD_MAX_BUFFER_SIZE -1 );
	// Now copy all bytes from array to local scratchpad
	for ( i = 0; i < MHD_MAX_BUFFER_SIZE; i++ )
	{
		WriteByteCBUS( (REG_CBUS_SCRATCHPAD_0 & 0xFF) + i, wdata[i] );
	}
	WriteByteCBUS( REG_CBUS_PRI_START & 0xFF, MSC_START_BIT_WRITE_BURST );
	
}
#endif
void MhlControl(void)
{
	u8 category = 0;
	u8 command = 0;
	u8 action = 0;
	u8 r_action = 0;
	int x_position = 0;
	int y_position = 0;
	unsigned short keyCode = 0;
	u8 plugResp[MHD_MAX_BUFFER_SIZE];
	signed char r_lval = 0;
	signed char r_uval = 0;
	int fdr = 3;
	int tmp_x = 0;
	int tmp_y = 0;

	category = mhl_ctl_data.mscScratchpadData[0];

	if(category == 0x01)
	{
		command =  mhl_ctl_data.mscScratchpadData[1];
		if(command == 0x01)			// Magic Motion Remote Controller -> absolute position, button
		{
			action = mhl_ctl_data.mscScratchpadData[2];
			x_position = (int)((mhl_ctl_data.mscScratchpadData[3] <<8) | mhl_ctl_data.mscScratchpadData[4]);
			y_position = (int)((mhl_ctl_data.mscScratchpadData[5] <<8) | mhl_ctl_data.mscScratchpadData[6]);

			if((0 < tvCtl_x) && (0 < tvCtl_y))
			{
				x_position = x_position*MAGIC_CANVAS_X/tvCtl_x;
				y_position = y_position*MAGIC_CANVAS_Y/tvCtl_y;

				if(x_position>MAGIC_CANVAS_X) x_position = MAGIC_CANVAS_X;
				if(y_position>MAGIC_CANVAS_Y) y_position = MAGIC_CANVAS_Y;
			}

			if(action == 0x01)	r_action = 1;
			else if(action == 0x02) r_action = 0;
			else return;

			#if 0
			mhl_ts_input(x_position, y_position, r_action);
			#else
			{
				char mbuf[120];

				memset(mbuf, 0, sizeof(mbuf));
				printk("%s ->magic RC, action:%d, (%d,%d) \n",__func__,r_action,x_position,y_position);
				sprintf(mbuf, "MHL_CTL=action=%04d, x_pos=%04d, y_pos=%04d", r_action,x_position,y_position);
			    hdmi_common_send_uevent(mbuf);
			}
			#endif
		}
		else if(command == 0x02)		// mouse -> relative position + mouse button
		{
			action = mhl_ctl_data.mscScratchpadData[2];

			r_lval = 0;
			r_uval = 0;
			r_lval = (signed char) mhl_ctl_data.mscScratchpadData[4];
			r_uval = (signed char) mhl_ctl_data.mscScratchpadData[3];
			x_position = (int)((r_uval<<8) | r_lval);

			r_lval = 0;
			r_uval = 0;
			r_lval = (signed char) mhl_ctl_data.mscScratchpadData[6];
			r_uval = (signed char) mhl_ctl_data.mscScratchpadData[5];
			y_position = (int)((r_uval<<8) | r_lval);

			printk("%s -> mouse, action:%d, (%d,%d) \n",__func__, action,x_position,y_position);

			if(((x_position> 100) || (x_position< -100)) ||((y_position> 100) || (y_position< -100)))
				fdr = 5;
			else if(((-30 < x_position) && (x_position < 30)) && ((-30 < y_position) && (y_position < 30)))
				fdr = 1;
			else
				fdr = 3;

			fdr = 7;
			x_position = fdr*x_position;
			y_position = fdr*y_position;

			if(action == 0x00)	// cursor
			{
				mhl_ms_ptr(x_position, y_position);
				return;
			}

			if((action == 0x03)) r_action = 1;
			else if((action == 0x02)) r_action = 0;
			else return;

			mhl_ms_ptr(x_position, y_position);
			mhl_ms_btn(r_action);
		}
		else if(command == 0x03)		// keyboard -> keycode includeing MENU/HOME/BACK
		{
			action = mhl_ctl_data.mscScratchpadData[2];
			keyCode = ((mhl_ctl_data.mscScratchpadData[3] <<8) | mhl_ctl_data.mscScratchpadData[4]);

			if((keyCode == KEY_F11) || (keyCode == KEY_F12))	// F11:ZoomIn, F12:ZoomOut
			{
				char mbuf[120];

				if(action == 0x01)
				{
					if(keyCode == KEY_F11) r_action = 1;
					else r_action = 0;

					x_position = 9999;
					y_position = 9999;
					memset(mbuf, 0, sizeof(mbuf));
					printk("%s ->magic RC, action:%d, (%d,%d) \n",__func__,r_action,x_position,y_position);
					sprintf(mbuf, "MHL_CTL=action=%04d, x_pos=%04d, y_pos=%04d", r_action,x_position,y_position);

				    hdmi_common_send_uevent(mbuf);
				}
				return;
			}

			if(action == 0x01)
			{
				r_action = 1;

				if((kbd_key_pressed[0]>>8) == 0x00)
				{
					if(((kbd_key_pressed[1] >>8) == 0x01) && ((kbd_key_pressed[1]&0xFF) == keyCode))
					{
						printk("%s -> keyboard, duplicated pressed, %x \n",__func__, keyCode);
						return;
					}
					kbd_key_pressed[0] = (0x0100 | keyCode);
				}
				else
				{
					if((kbd_key_pressed[1] >>8) == 0x01)
					{
						printk("%s -> keyboard, duplicated pressed, %x \n",__func__, keyCode);
						return;
					}
					kbd_key_pressed[1] = (0x0100 | keyCode);
				}

#if 0
				if(kbd_key_pressed == 0x01)
				{
					printk("%s -> keyboard, duplicated pressed, %d \n",__func__, keyCode);
					return;
				}
				else
				{
					kbd_key_pressed = 0x01;
				}
#endif
			}
			else if(action == 0x02)
			{
				r_action = 0;

				if(((kbd_key_pressed[0]>>8) == 0x01) && ((kbd_key_pressed[0]&0xFF) == keyCode))
				{
					kbd_key_pressed[0] = 0x0000;
					mhl_kbd_key(keyCode,r_action);
					return;
				}
				if(((kbd_key_pressed[1]>>8) == 0x01) && ((kbd_key_pressed[1]&0xFF) == keyCode))
				{
					kbd_key_pressed[1] = 0x0000;
					mhl_kbd_key(keyCode,r_action);
					return;
				}

				printk("%s -> keyboard, duplicated released, %x \n",__func__, keyCode);
				return;


#if 0
				if(kbd_key_pressed == 0x00)
				{
					printk("%s -> keyboard, duplicated released, %d \n",__func__, keyCode);
					return;
				}
				else
				{
					kbd_key_pressed = 0x00;
				}
#endif
			}
			else return;

			mhl_kbd_key(keyCode,r_action);
		}
		else
		{
			SII_DEV_DBG("MhlControl : not defined message\n");
		}
	}
	else if(category == 0x02)
	{
		command =  mhl_ctl_data.mscScratchpadData[1];
		action = mhl_ctl_data.mscScratchpadData[2];

		if(command > 0x03)
		{
			SII_DEV_DBG("MhlControl : not defined message\n");
			return;
		}

		if(command == 0x01)	// TV magic motion remote controller
		{
			if(action == 0x01)
			{
				tmp_x = (int)((mhl_ctl_data.mscScratchpadData[3] <<8) | mhl_ctl_data.mscScratchpadData[4]);
				tmp_y = (int)((mhl_ctl_data.mscScratchpadData[5] <<8) | mhl_ctl_data.mscScratchpadData[6]);

				printk("%s -> TV control canvas (%d, %d) \n",__func__, tmp_x, tmp_y);
				if((((MAGIC_CANVAS_X/2) < tmp_x) && (tmp_x < 2*MAGIC_CANVAS_X))
					&& (((MAGIC_CANVAS_Y/2) < tmp_y) && (tmp_y < 2*MAGIC_CANVAS_Y)))
				{
					tvCtl_x = tmp_x;
					tvCtl_y = tmp_y;
				}
				else
				{
					tvCtl_x = MAGIC_CANVAS_X;
					tvCtl_y = MAGIC_CANVAS_Y;
				}
			}
		}
		else if(command == 0x02)	// mouse
		{
			if(action == 0x02)
			{
				mhl_ms_hide_cursor();
				printk("%s -> mouse UNPLUGGED.\n",__func__);
			}
			else;
		}
		else if(command == 0x03)	// keyboard
		{
			kbd_key_pressed[0] = 0x0000;
			kbd_key_pressed[1] = 0x0000;

			if(action == 0x01)
			{
				mhl_writeburst_uevent(1);
				printk("%s -> keyboard PLUGGED.\n",__func__);
			}
			else if(action == 0x02)
			{
				mhl_writeburst_uevent(2);
				printk("%s -> keyboard UNPLUGGED.\n",__func__);
			}
			else;
		}
		else;

		memcpy(plugResp, mhl_ctl_data.mscScratchpadData, MHD_MAX_BUFFER_SIZE);
		plugResp[3] = 0x01;
		plugResp[4] = 0x00;
		plugResp[5] = 0x00;
		plugResp[6] = 0x00;
		SiiMhlTxWriteScratchpad(plugResp);
	}
	else
	{
		SII_DEV_DBG("MhlControl : not defined message\n");
		return;
	}
}

#endif

void mhl_path_enable(bool path_en)
{
    u8 data[16];
    u16 data_size;

    SII_DEV_DBG(  "(Line:%d)  MHL_STATUS_PATH_ENABLED !!! path_en=%d\n", (int)__LINE__, path_en ); 
    
    //                                
    // ADD : init data    
    memset(data, 0x00, sizeof(data));

    if(path_en)
    {
        mhl_status_value.linkmode |= MHL_STATUS_PATH_ENABLED;
    }
    else
    {
        mhl_status_value.linkmode &= ~MHL_STATUS_PATH_ENABLED;
    }

    data[0] = mhl_status_value.linkmode;
    data_size = 1;
    cbus_cmd_bind(MHL_WRITE_STAT, MHL_STATUS_REG_LINK_MODE,data,data_size, FALSE);
}


void mhltx_got_status(u8 status0, u8 status1)
{
    u8 data[16];
    u16 data_size;
    
    //                                
    // MOD : init data
    memset(data, 0x00, sizeof(data));
    data_size = 0;

    if((MHL_STATUS_PATH_ENABLED & status1) &&
    !(mhl_status_value.linkmode & MHL_STATUS_PATH_ENABLED))
    {
        mhl_path_enable(TRUE);
    }
    else if(!(MHL_STATUS_PATH_ENABLED & status1) &&
    (mhl_status_value.linkmode & MHL_STATUS_PATH_ENABLED))
    {
        mhl_path_enable(FALSE);
    }
   

    if(MHL_STATUS_DCAP_RDY & status0)
    {
        SII_DEV_DBG("MHL_STATUS_DCAP_RDY\n"); 
        cbus_cmd_bind(MHL_READ_DEVCAP,MHL_DEV_CATEGORY_OFFSET,data, data_size, FALSE);
        cbus_cmd_bind(MHL_READ_DEVCAP,MHL_CAP_FEATURE_FLAG,data, data_size, FALSE);
    }

}


void mhltx_set_status(u8 command, u8 offset, u8 value)
{
  u8 data[16];
  u16 data_size;

  memset(data, 0x00, (sizeof(u8)*16));
  data[0] = value;
  data_size = 1;  
  
  cbus_cmd_bind(command,offset,data,data_size, FALSE);
}


void cbus_cmd_done(void)
{
    cbus_cmd_node *current_p;
    cbus_cmd_node *next_p;

    SII_LOG_FUNCTION_NAME_ENTRY;

    current_p = cbus_cmd_head->next;
    current_p->cmd_send = FALSE;
    current_p->cmd_done = TRUE;
    
    SII_DEV_DBG("CMD:%x\n", current_p->command);

    switch( current_p->command)
    {
        case MHL_WRITE_STAT:
            //read data..
            
            current_p->lenght = 0x01;
            current_p->payload.data[0] = ReadByteCBUS(0x16);             
            SII_DEV_DBG("Data: %x\n",current_p->payload.data[0]);
            break;
    
        case MHL_READ_DEVCAP:  

            switch(current_p->offset)
            {
                case MHL_CAP_MHL_VERSION:
                    SII_DEV_DBG("MHL_CAP_MHL_VERSION \n");          
                    mhl_reciever_cap.mhl_ver = ReadByteCBUS(0x16);
                    break;

                case MHL_DEV_CATEGORY_OFFSET:
                {
                    u8 cap_data =0;
                    cap_data = (0x1F & ReadByteCBUS(0x16));

                    switch(cap_data)
                    {
                        case MHL_DEV_CAT_SOURCE:
                            SII_DEV_DBG("MHL_DEV_CAT_SOURCE \n");          
                            mhl_reciever_cap.mhl_ver = MHL_DEV_CAT_SOURCE;
                            break;

                        case MHL_SINK_W_POW:
                            SII_DEV_DBG("MHL_SINK_W_POW \n");          
                            mhl_reciever_cap.mhl_ver = MHL_SINK_W_POW;

                            //mhl tx doesn't need power out
                            break;

                        case MHL_SINK_WO_POW:
                            SII_DEV_DBG("MHL_SINK_WO_POW \n");                            
                            mhl_reciever_cap.mhl_ver = MHL_SINK_WO_POW;
                            //this sink capability is wrong... Sink should have a POW bit....

                            break;

                        case MHL_DONGLE_W_POW:
                            SII_DEV_DBG("MHL_DONGLE_W_POW \n");                            
                            mhl_reciever_cap.mhl_ver = MHL_DONGLE_W_POW;
                            //mhl tx doesn't need power out
                            break;

                        case MHL_DONGLE_WO_POW:
                            SII_DEV_DBG("MHL_DONGLE_WO_POW \n");                            
                            mhl_reciever_cap.mhl_ver = MHL_DONGLE_WO_POW;
                            // No dongle power ...
                            break;

                        default:
                            SII_DEV_DBG("DEFAULT \n");                          
                            mhl_reciever_cap.mhl_ver = cap_data;
                            break;             

                    }
                }        
                break;
				
                case MHL_CAP_ADOPTER_ID_H:
                    SII_DEV_DBG("MHL_CAP_ADOPTER_ID_H \n");           
                    mhl_reciever_cap.adopter_id = ReadByteCBUS(0x16);;
                    mhl_reciever_cap.adopter_id <<= 8;
                    break;

                case MHL_CAP_ADOPTER_ID_L:
                    SII_DEV_DBG("MHL_CAP_ADOPTER_ID_L \n");           
                    mhl_reciever_cap.adopter_id |= ReadByteCBUS(0x16);;
                    break;

                case MHL_CAP_VID_LINK_MODE:
                    SII_DEV_DBG("MHL_CAP_VID_LINK_MODE \n");           
                    mhl_reciever_cap.vid_link_mode = (0x3F & ReadByteCBUS(0x16));
                    break;

                case MHL_CAP_AUD_LINK_MODE:
                    SII_DEV_DBG("MHL_CAP_AUD_LINK_MODE \n");                     
                    mhl_reciever_cap.aud_link_mode = (0x03 & ReadByteCBUS(0x16));
                    break;
              
                case MHL_CAP_VIDEO_TYPE:
                    SII_DEV_DBG("MHL_CAP_VIDEO_TYPE \n");                     
                    mhl_reciever_cap.video_type = (0x8F & ReadByteCBUS(0x16));
                    break;

                case MHL_CAP_LOG_DEV_MAP:
                    SII_DEV_DBG("MHL_CAP_LOG_DEV_MAP \n");                     
                    mhl_reciever_cap.log_dev_map = ReadByteCBUS(0x16);
                    break;

                case MHL_CAP_BANDWIDTH:
                    SII_DEV_DBG("MHL_CAP_BANDWIDTH \n");           
                    mhl_reciever_cap.bandwidth = ReadByteCBUS(0x16);
                    break;

                case MHL_CAP_FEATURE_FLAG:                 
                    mhl_reciever_cap.feature_flag = (0x07 & ReadByteCBUS(0x16));
                    SII_DEV_DBG("MHL_CAP_FEATURE_FLAG:%x \n", mhl_reciever_cap.feature_flag );   
                    
                    mhl_reciever_cap.rcp_support = (mhl_reciever_cap.feature_flag & MHL_FEATURE_RCP_SUPPORT)?TRUE:FALSE;
                    mhl_reciever_cap.rap_support = (mhl_reciever_cap.feature_flag & MHL_FEATURE_RAP_SUPPORT)?TRUE:FALSE;
                    mhl_reciever_cap.sp_support = (mhl_reciever_cap.feature_flag & MHL_FEATURE_SP_SUPPORT)?TRUE:FALSE;
                    break;

                case MHL_CAP_DEVICE_ID_H:
                    SII_DEV_DBG("MHL_CAP_DEVICE_ID_H \n");                     
                    mhl_reciever_cap.device_id = ReadByteCBUS(0x16);
                    mhl_reciever_cap.device_id <<= 8;
                    break;

                case MHL_CAP_DEVICE_ID_L:
                    SII_DEV_DBG("MHL_CAP_DEVICE_ID_L \n");                     
                    mhl_reciever_cap.device_id |= ReadByteCBUS(0x16);

                    break;

                case MHL_CAP_SCRATCHPAD_SIZE:
                    SII_DEV_DBG("MHL_CAP_SCRATCHPAD_SIZE \n");                     
                    mhl_reciever_cap.scratchpad_size = ReadByteCBUS(0x16);

                    break;

                case MHL_CAP_INT_STAT_SIZE:
                    SII_DEV_DBG("MHL_CAP_INT_STAT_SIZE \n");                     
                    mhl_reciever_cap.int_stat_size = ReadByteCBUS(0x16);
                    break;
          
                case MHL_CAP_DEV_STATE:
                case MHL_CAP_RESERVED:          
                default:
                    SII_DEV_DBG("Line:%d : DEFAULT \n", (int)__LINE__ );           
                    break;
            }
            break;

        case MHL_MSC_MSG:  

            SII_DEV_DBG("MHL_MSC_MSG \n");
            // check the previous data
            if(current_p->payload.data[0] == MHL_MSC_MSG_RCPE)
            {
	         mutex_lock(&cbus_cmd_bind_mutex);	//                  
                next_p = current_p->next;
                cbus_cmd_head->next = next_p;
                next_p->prev = cbus_cmd_head;

		   mutex_unlock(&cbus_cmd_bind_mutex);	//                  
		   
                kfree(current_p);

                if(next_p == cbus_cmd_tail)
                {
                    SII_DEV_DBG("Line: %d (next_p == cbus_cmd_tail) \n",(int)__LINE__);            
                }   
                else
                {
                    cbus_cmd_put_workqueue(next_p); 
                }

                SII_DEV_DBG("(current_p->payload.data[0] == MHL_MSC_MSG_RCPE)\n");
                SiiMhlTxRcpkSend(current_p->payload.data[2]);

                return ;
            }

            break;

        case MHL_WRITE_BURST:

		mutex_lock(&cbus_cmd_bind_mutex);	//                  
            next_p = current_p->next;
            cbus_cmd_head->next = next_p;
            next_p->prev = cbus_cmd_head;
		mutex_unlock(&cbus_cmd_bind_mutex);	//                  
		
            kfree(current_p);

            if(next_p == cbus_cmd_tail)
            {
                SII_DEV_DBG("Line:%d (next_p == cbus_cmd_tail) \n", (int)__LINE__ );            
            }   
            else
            {
                cbus_cmd_put_workqueue(next_p); 
            }

            SII_DEV_DBG(  "cbus_cmd_done MHL_WRITE_BURST \n");                
            MhlTxSetInt( MHL_RCHANGE_INT,MHL_INT_DSCR_CHG,FALSE);         

            return ;

            break;

    case MHL_SET_INT:  

        SII_DEV_DBG("MHL_SET_INT offset[%x]\n",current_p->offset);
        if((current_p->offset == MHL_RCHANGE_INT )&&(current_p->payload.data[0]==MHL_INT_DSCR_CHG ))
        {
            //Write burst final step... Req->GRT->Write->DSCR
            SII_DEV_DBG("MHL_RCHANGE_INT&MHL_INT_DSCR_CHG \n");
        }
        else if((current_p->offset ==MHL_RCHANGE_INT )&&(current_p->payload.data[0]==MHL_INT_DCAP_CHG ))
        {
	      mutex_lock(&cbus_cmd_bind_mutex);	//                  
            next_p = current_p->next;
            cbus_cmd_head->next = next_p;
            next_p->prev = cbus_cmd_head;
	      mutex_unlock(&cbus_cmd_bind_mutex);	//                  
		
            kfree(current_p);

            if(next_p == cbus_cmd_tail)
            {
                SII_DEV_DBG("Line:%d (next_p == cbus_cmd_tail) \n", (int)__LINE__ );            
            }   
            else
            {
                cbus_cmd_put_workqueue(next_p); 
            }          

            mhl_status_value.connected_ready |= MHL_STATUS_DCAP_RDY;
            mhltx_set_status(MHL_WRITE_STAT, MHL_STATUS_REG_CONNECTED_RDY,mhl_status_value.connected_ready );

            return ;
        }

        break;   
      
        default:
            SII_DEV_DBG(  "\n cbus_cmd_done default \n");
            return;
            break;
    }

    mutex_lock(&cbus_cmd_bind_mutex);	//                  
    next_p = current_p->next;
    cbus_cmd_head->next = next_p;
    next_p->prev = cbus_cmd_head;
    mutex_unlock(&cbus_cmd_bind_mutex);	//                  

    kfree(current_p);
    if(next_p == cbus_cmd_tail)
    {
        SII_DEV_DBG(" line %d(next_p == cbus_cmd_tail) \n", (int)__LINE__ );
        return;
    }       
    
    cbus_cmd_put_workqueue(next_p);  
    SII_LOG_FUNCTION_NAME_EXIT;
}


void MhlTxSetInt( u8 regToWrite,u8  mask, u8 priority)
{
    u8 data[1];

    data[0] = mask;
    cbus_cmd_bind(MHL_SET_INT,regToWrite,data,0x01,priority );
    return;
}

void cbus_cmd_put_workqueue(cbus_cmd_node* node_p)
{
    cbus_send_cmd_type *dev = NULL;
    SII_LOG_FUNCTION_NAME_ENTRY;
    
    if(node_p->cmd_send == TRUE)
    {
        SII_DEV_DBG_ERROR("COMMAND already was sent \n"); 
        return ;
    }
    
    //                                
    // MOD : lockup issue
    //dev = (cbus_send_cmd_type*)kmalloc(sizeof(cbus_send_cmd_type), GFP_KERNEL);
    dev = (cbus_send_cmd_type*)kmalloc(sizeof(cbus_send_cmd_type), GFP_ATOMIC);

    if (dev == NULL) 
    {
        SII_DEV_DBG_ERROR("(dev == NULL) \n"); 
        return ;
    }

    SII_DEV_DBG("*** send:%x, cmd:%x, offset:%x data[0]:%x data[1]:%x \n", 
        node_p->cmd_send, node_p->command, node_p->offset, node_p->payload.data[0],node_p->payload.data[1]); 
  
    memset(dev, 0x00, sizeof(cbus_send_cmd_type));
  
    node_p->cmd_send = TRUE;
    dev->command = node_p->command;
    dev->offsetdata = node_p->offset;
    dev->length = node_p->lenght;
    
    memset(dev->payload_u.msgdata, 0x00, 16);
    memcpy(dev->payload_u.msgdata, node_p->payload.data, node_p->lenght);      
    
    //SII_DEV_DBG("*** dev Addr :: %x cbus_work Addr :: %x\n", dev , &dev->cbus_work); 
    SII_DEV_DBG("node_p->command:%x, dev->command:%x\n", 
            node_p->command, dev->command); 

	SII_DEV_DBG("dev Addr :: %p\n", dev); 
	SII_DEV_DBG("cbus_work Addr :: %p\n", &dev->cbus_work); 
    //  INIT_WORK (&dev->cbus_work, cbus_cmd_send, dev);
    INIT_WORK (&dev->cbus_work, cbus_cmd_send);
    schedule_work(&dev->cbus_work);
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
}


void delete_all_cbus_cmd(void)
{
    cbus_cmd_node *p;
    cbus_cmd_node *s;

  mutex_lock(&cbus_cmd_bind_mutex);	//                  
    p = cbus_cmd_head->next;

    while(p != cbus_cmd_tail)
    {
        s = p;
        p = p->next;
        p ->prev = cbus_cmd_head;
        kfree(s);
    }

    cbus_cmd_head->next = cbus_cmd_tail;
    cbus_cmd_tail->prev = cbus_cmd_head;

  mutex_unlock(&cbus_cmd_bind_mutex);	//                  
    return;
}




void cbus_cmd_bind(u8 command, u8 offset, u8* data, u16 data_lenth, bool priority)
{
    cbus_cmd_node *new_node=NULL;
    cbus_cmd_node *s;
    SII_LOG_FUNCTION_NAME_ENTRY;

    new_node = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);

    if (new_node == NULL) 
    {
        SII_DEV_DBG_ERROR("new_node == NULL)\n"); 
        return;
    }

    mutex_lock(&cbus_cmd_bind_mutex);
	
    //                                
    // ADD : init data    
    memset(new_node, 0x00, sizeof(cbus_cmd_node));

    if(priority)
    {
        SII_DEV_DBG("CBUS command urgent priority \n");

        s = cbus_cmd_head->next;

        if(s == cbus_cmd_tail)
        {
            new_node->next = cbus_cmd_tail;
            new_node->prev = cbus_cmd_head;
            cbus_cmd_tail->prev = new_node;
            cbus_cmd_head->next = new_node;
        }
        else
        {
            if(s->cmd_send == TRUE)
            {
                new_node->next = s->next;
                new_node->prev = s;
                s->next->prev = new_node;
                s->next = new_node;
            }
            else
            {
                new_node->next = s;
                new_node->prev = cbus_cmd_head;
                s->prev = new_node;
                cbus_cmd_head->next = new_node;
            }
        }
    }
    else
    {
        SII_DEV_DBG("CBUS command Normal priority \n");

        s = cbus_cmd_tail->prev;
        new_node->next = cbus_cmd_tail;
        new_node->prev = s; 
        cbus_cmd_tail->prev = new_node;      
        s->next = new_node; 
    }
    
    new_node->command = command;  
    new_node->offset = offset;

    if(data_lenth)
    {
        memcpy(new_node->payload.data, data, data_lenth);
    }

    if(MHL_MSC_MSG_RCPE == data[0]) 
    {
        data_lenth = (data_lenth -1);
    }

    new_node->lenght = data_lenth;  
    SII_DEV_DBG("priority: %d lenght: %d, %d \n", priority, new_node->lenght, data_lenth);
    
    new_node->cmd_send = FALSE;
    new_node->cmd_done = FALSE; 
    if(cbus_cmd_head->next->cmd_send == TRUE)
    {
        mutex_unlock(&cbus_cmd_bind_mutex);
        SII_DEV_DBG("Already command sending~~~ \n");
        return;
    }  
    mutex_unlock(&cbus_cmd_bind_mutex);
    cbus_cmd_put_workqueue(new_node);
    
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
}




//                                
// MOD : wrong used work queue function.
// mismatch the dev address pointer.
//static void cbus_cmd_send(void* _dev)
static void cbus_cmd_send(struct work_struct *work)    
{
    //                                
    // MOD : wrong used work queue function.
    // mismatch the dev address pointer.
    //cbus_send_cmd_type *dev = (cbus_send_cmd_type *)_dev;
	cbus_send_cmd_type *dev = container_of(work, cbus_send_cmd_type, cbus_work);
    
    u8 startbit = 0;
    bool result = TRUE;

    SII_LOG_FUNCTION_NAME_ENTRY;
    
    //                                
    // ADD : null defense     
    if(!dev)
    {
        SII_DEV_DBG_ERROR("dev is NULL\n");
        return;
    }
    
    SII_DEV_DBG("*** dev->command:%x\n", dev->command);
    //    dev->command, dev->offsetdata, dev->payload_u.msgdata[0]);
#ifdef MHL_CTL
	if(dev->command != MHL_WRITE_BURST)
	{
#endif 
    WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD&0xFF), dev->offsetdata); 	// set offset  
    WriteByteCBUS( (REG_CBUS_PRI_WR_DATA_1ST & 0xFF), dev->payload_u.msgdata[0]);
#ifdef MHL_CTL
	}
#endif
    switch(dev->command)
    {
        case MHL_SET_INT:	// Set one interrupt register = 0x60
            startbit = MSC_START_BIT_WRITE_REG;
            break;

        case MHL_WRITE_STAT:	// Write one status register = 0x60 | 0x80
            startbit = MSC_START_BIT_WRITE_REG;
            break;

        case MHL_READ_DEVCAP:	// Read one device capability register = 0x61
            startbit = MSC_START_BIT_READ_REG;
            break;

        case MHL_GET_STATE:			// 0x62 -
        case MHL_GET_VENDOR_ID:		// 0x63 - for vendor id	
        case MHL_SET_HPD:			// 0x64	- Set Hot Plug Detect in follower
        case MHL_CLR_HPD:			// 0x65	- Clear Hot Plug Detect in follower
        case MHL_GET_SC1_ERRORCODE:		// 0x69	- Get channel 1 command error code
        case MHL_GET_DDC_ERRORCODE:		// 0x6A	- Get DDC channel command error code.
        case MHL_GET_MSC_ERRORCODE:		// 0x6B	- Get MSC command error code.
        case MHL_GET_SC3_ERRORCODE:		// 0x6D	- Get channel 3 command error code.
            WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD & 0xFF), dev->command );
            startbit = MSC_START_BIT_MSC_CMD;
            break;

        case MHL_MSC_MSG:
            WriteByteCBUS( (REG_CBUS_PRI_WR_DATA_2ND & 0xFF), dev->payload_u.msgdata[1]);
            WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD & 0xFF), dev->command );
            startbit = MSC_START_BIT_VS_CMD;
            break;

        case MHL_WRITE_BURST:
            printk("======command :%x, offset:%x, data 0:%x, data 1:%x\n", dev->command, dev->offsetdata, dev->payload_u.msgdata[0],dev->payload_u.msgdata[1]);
		#ifdef MHL_CTL
            WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD & 0xFF), dev->offsetdata);
		#else
            WriteByteCBUS( (REG_CBUS_PRI_ADDR_CMD & 0xFF), dev->offsetdata + 0x40 );
		#endif
            ReadModifyWriteCBUS((REG_MSC_WRITE_BURST_LEN & 0xFF),0x0F, dev->length -1 );

            // Now copy all bytes from array to local scratchpad
            if (NULL == dev->payload_u.pdata)
            {
                SII_DEV_DBG_ERROR("Put pointer to WRITE_BURST data in req.pdatabytes!!!\n\n");
            }
            else
            {
                int i;

                SII_DEV_DBG("Writing data into scratchpad (Lenth: %d)\n\n", dev->length);

                for ( i = 0; i < dev->length; i++ )
                {
                    u8 tmp=0;
                    tmp = dev->payload_u.msgdata[i];

                    SII_DEV_DBG("BURST DATA:%x \n", tmp); 
                    WriteByteCBUS( (REG_CBUS_SCRATCHPAD_0 & 0xFF) + i, tmp );
                }
            }
            startbit = MSC_START_BIT_WRITE_BURST;
            break;

        default:
            SII_DEV_DBG_ERROR("unsupport CMD\n\n");
            result = FALSE;
            break;
    }

    if ( result )
    {
        WriteByteCBUS( REG_CBUS_PRI_START & 0xFF, startbit );
    }
    else
    {
        SII_DEV_DBG_ERROR("Send failed\n\n");
    }

    if(dev)
        kfree(dev);
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
}


//added
#if 0
void simg_timer_handler(unsigned long timer_type)
{
  u8 sys_stat =0;	//                  
///  unsigned long flags; 
		
  //                                                                    
  
  printk(KERN_INFO "\n#### simg_timer_handler %d ####\n", (int) timer_type);

  switch(timer_type)
  {
   case RSEN_400M_TIMER: 
   {
    //for CTS 3.3.14.3
      //                                           
    
      //rsen = I2C_ReadByte(PAGE_0_0X72, 0x09);
      
//                                                                     
	
      sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);	//                  
      
      if(((sys_stat & RSEN_HIGH) == 0)&&(mhl_status_value.rsen_check_available == TRUE))
      {
        printk(KERN_INFO "RSEN LOW ~ \n");
        mhl_status_value.mhl_status = MHL_RSEN_LOW;

        simg_msleep(110);		//remove from Simg

        sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
        printk(KERN_INFO "sys_stat: %x ~ \n", sys_stat);

        if((sys_stat & RSEN_HIGH) == 0)
        {
          printk(KERN_INFO "RSEN Really LOW ~ \n");
          delete_all_cbus_cmd(); 
          ForceUsbIdSwitchOpen();
          simg_msleep(50);	 //                               
          ReleaseUsbIdSwitchOpen();

//                                                                        
          //customer need to change the USB PATH

		  // for MHL CST - Nov16
          MHL_On(0);

          return ;
        }
      
      }      
//                                                                           
   }
    break;

   default:

    break;
  }

}
#else
void simg_timer_schdule(struct work_struct *work)
{
	//for CTS 3.3.14.3
	u8 sys_stat =0;

	sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);

	if(((sys_stat & RSEN_HIGH) == 0)&&(mhl_status_value.rsen_check_available == TRUE))
	{
		printk(KERN_INFO "RSEN LOW ~ \n");
    
		simg_msleep(110);

		sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
		printk(KERN_INFO "sys_stat: %x ~ \n", sys_stat);

		if((sys_stat & RSEN_HIGH) == 0)
		{
			printk(KERN_INFO "RSEN Really LOW ~ \n");
			printk("2army, simg_timer_schdule -> delete_all_cbus_cmd\n");
			delete_all_cbus_cmd(); 
			ForceUsbIdSwitchOpen();
			simg_msleep(50);
			ReleaseUsbIdSwitchOpen();

			//customer need to change the USB PATH
			MHL_On(0);
			//sii9244_mhl_tx_int();
		}
	}    
}

void simg_timer_handler(unsigned long timer_type)
{
//	  del_timer(&simg_timer);
	

	if (!resen_check_init)	
	{		
		INIT_DELAYED_WORK_DEFERRABLE(&rsen_check_control.work, simg_timer_schdule);
		resen_check_init = TRUE;	
	}		
	schedule_delayed_work(&rsen_check_control.work, 0);
	rsen_timer = FALSE;
}
#endif
int process_mhl_discovery_start(void)
{
    u8 int4Status, intr1;

    intr1 = I2C_ReadByte(PAGE_0_0X72, 0x71);
    I2C_WriteByte(PAGE_0_0X72, 0x71, intr1); //clear interrupt.
    
    SII_DEV_DBG("intr1:%x , intr1_mask_value:%x \n", intr1 , mhl_status_value.intr1_mask_value); 

    if(intr1&RSEN_CHANG)
    {
        u8 sys_stat =0;

        sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
        SII_DEV_DBG("sys_stat: %x ~ \n", sys_stat);

        if(((sys_stat & RSEN_HIGH) == 0)&&(mhl_status_value.rsen_check_available == TRUE))
        {
            SII_DEV_DBG("RSEN LOW ~ \n");
            mhl_status_value.mhl_status = MHL_RSEN_LOW;

            simg_msleep(110);

            sys_stat = I2C_ReadByte(PAGE_0_0X72, 0x09);
            SII_DEV_DBG(  "sys_stat: %x ~ \n", sys_stat);

            if((sys_stat & RSEN_HIGH) == 0)
            {
                SII_DEV_DBG("RSEN Really LOW ~ \n");
		    printk("2army, process_mhl_discovery_start (1) -> delete_all_cbus_cmd\n");
                delete_all_cbus_cmd(); 
                ForceUsbIdSwitchOpen();
                simg_msleep(50);
                ReleaseUsbIdSwitchOpen();
                //        sii9244_mhl_tx_int();

                // check!! USB path change NEED CHECK!! - jh.koo
                // for MHL CST - Jan16
                MHL_On(0);
				//sii9244_mhl_tx_int();

                return 1;
            }    
			else 
			{
				if(rsen_timer)
				{
					del_timer(&simg_timer);
					rsen_timer = FALSE;
				}
				printk(KERN_INFO "RSEN HIGH ~ \n");
				mhl_status_value.mhl_status = MHL_DISCOVERY_SUCCESS;
			}
        }
        else
        {
			if(rsen_timer){
				del_timer(&simg_timer);
				rsen_timer = FALSE;
			}
            SII_DEV_DBG("RSEN HIGH ~ \n");
        }      
    
    }

#if 1

    mhl_status_value.cbus_connected = ReadByteCBUS(0x0a);	// added

    int4Status = I2C_ReadByte(PAGE_0_0X72, (0x74));	// read status
    SII_DEV_DBG("int4Status = %02X \n ", (int)int4Status);

    if(! (int4Status & ( MHL_EST_INTR4 | USB_EST_INTR4) ) && (mhl_status_value.cbus_connected))
    {
        simg_msleep(1);
        int4Status = I2C_ReadByte(PAGE_0_0X72, (0x74));	// read status	
        SII_DEV_DBG("int4Status = %02X \n ", (int)int4Status);	
    }
    I2C_WriteByte(PAGE_0_0X72, (0x74), int4Status);	// clear all interrupts
#else

	//                  
	{
		int retry_count = 2;
		int i = 0;
		for(i=0;i<retry_count;i++)
		{
			int4Status = I2C_ReadByte(PAGE_0_0X72, (0x74));	// read status
			SII_DEV_DBG("int4Status = %02X , retry_count=%d\n ", (int)int4Status, i);
			if(int4Status != 0x00)
				break;
			msleep(10);
		}
		I2C_WriteByte(PAGE_0_0X72, (0x74), int4Status);	// clear all interrupts
	}
	SII_DEV_DBG("int4Status = %02X\n ", (int)int4Status);
#endif
    //SII_DEV_DBG("INT PAGE_0_0X72:0x71 :%x \n" ,I2C_ReadByte(PAGE_0_0X72, 0x71));
    //SII_DEV_DBG("INT ReadByteCBUS:0x08 :%x \n" ,ReadByteCBUS(0x08));
    //SII_DEV_DBG("INT ReadByteCBUS:0x1E :%x \n" ,ReadByteCBUS(0x1E));  

    mhl_status_value.cbus_connected = ReadByteCBUS(0x0a);

    //SII_DEV_DBG("int4Status:%d cbus_connected:%d \n", int4Status, mhl_status_value.cbus_connected );
 
    if((int4Status & MHL_EST_INTR4)&&(mhl_status_value.cbus_connected))
    {

        SII_DEV_DBG("MHL EST SUCCESS ~ \n");

        mhl_status_value.mhl_status = MHL_DISCOVERY_SUCCESS;


        I2C_WriteByte(PAGE_0_0X72, 0xA0, 0x10);
        WriteByteCBUS(0x07, 0xF2);  // CBUS DDC byte handshake mode    
        // Enable segment pointer safety
        SET_BIT(PAGE_CBUS_0XC8, 0x44, 1);    
        ENABLE_DISCOVERY;    

        mhl_status_value.rsen_check_available = TRUE;

        MhlTxSetInt( MHL_RCHANGE_INT, MHL_INT_DCAP_CHG, FALSE);
		// added
		if(rsen_timer == FALSE){
			rsen_timer = TRUE;
			simg_init_timer(RSEN_400M_TIMER,T_SRC_RXSENSE_CHK); //for CTS 3.3.14.3
		}		

        mhl_status_value.intr1_mask_value = (RSEN_CHANG|MDI_HPD); //RSEN_CHANG; 
        mhl_status_value.intr4_mask_value = (CBUS_LOCKOUT);
        mhl_status_value.intr_cbus1_mask_value = INTR_CBUS1_DESIRED_MASK;
        mhl_status_value.intr_cbus2_mask_value = INTR_CBUS2_DESIRED_MASK;

        WRITE_INTR4_VALUE(mhl_status_value.intr4_mask_value);
        WRITE_INTR1_VALUE(mhl_status_value.intr1_mask_value);
        WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
        WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value); 

        //SII_DEV_DBG("INT PAGE_0_0X72:0x74 :%x \n" ,I2C_ReadByte(PAGE_0_0X72, 0x74));
        //SII_DEV_DBG("INT PAGE_0_0X72:0x71 :%x \n" ,I2C_ReadByte(PAGE_0_0X72, 0x71));
        //SII_DEV_DBG("INT ReadByteCBUS:0x08 :%x \n" ,ReadByteCBUS(0x08));
        //SII_DEV_DBG("INT ReadByteCBUS:0x1E :%x \n" ,ReadByteCBUS(0x1E));  

        process_cbus_interrrupt();
    
    }
    else if((int4Status & USB_EST_INTR4)&&(mhl_status_value.rgnd_1k == TRUE))
    {
        SII_DEV_DBG("MHL EST FAIL #1 ~ \n");
        mhl_status_value.mhl_status = MHL_DISCOVERY_FAIL;
	  printk("2army, process_mhl_discovery_start (2)  -> delete_all_cbus_cmd\n");
        delete_all_cbus_cmd(); 
        sii9244_mhl_tx_int();
    }
    else if(mhl_status_value.cbus_connected == 0x01)
    {
        SII_DEV_DBG("MHL EST Not interrupt ~ \n");
            mhl_status_value.intr_cbus1_mask_value = INTR_CBUS1_DESIRED_MASK;
            mhl_status_value.intr_cbus2_mask_value = INTR_CBUS2_DESIRED_MASK;
            WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
            WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value);    
            process_cbus_interrrupt();

    } 
    else
    {
        SII_DEV_DBG_ERROR("What Case????\n");
        return -1;
    }
    
    return 1;
}


void simg_TmdsControl (bool enable)
{  
    if( enable )
    { 
        SII_DEV_DBG("Drv: TMDS Output Enabled\n");
        SET_BIT(PAGE_0_0X72, 0x80, 4);
        //HPD OVEREN = 0 :: SET_HPD status
        CLR_BIT(PAGE_0_0X72, 0x79, 4);
    }
    else
    {
        CLR_BIT(PAGE_0_0X72, 0x80, 4);
        SII_DEV_DBG("Drv: TMDS Ouput Disabled\n");
    }
}


void process_waiting_rgnd(void)
{
    u8 int4Status;
    SII_LOG_FUNCTION_NAME_ENTRY;

    int4Status = I2C_ReadByte(PAGE_0_0X72, 0x74);	// read status

    SII_DEV_DBG ("** Int4Isr : %x \n", (int)int4Status);

    if((0xFF == int4Status)||(0xFA == int4Status))
    {
        simg_msleep(2);

        int4Status = I2C_ReadByte(PAGE_0_0X72, (0x74));	// read status
        SII_DEV_DBG ("** Int4Isr 2 : %x \n", (int)int4Status);

        if((0xFF == int4Status)||(0xFA == int4Status))
        {
            sii9244_mhl_tx_int();
            return ;
        }    
    }
  
    if(int4Status & RGND_RDY)
    {
        mhl_status_value.mhl_status = MHL_CABLE_CONNECT;

        TX_GoToD0(); 

        if(TX_RGND_check()!= 0x02)
        {
            printk("2army, process_waiting_rgnd (1)  -> delete_all_cbus_cmd\n");
            delete_all_cbus_cmd(); 

            //customer need to change the USB PATH
            // for MHL CST - Jan16
			MHL_On(0);
		 	//sii9244_mhl_tx_int();

            return ;

        }

        I2C_WriteByte(PAGE_0_0X72, 0x74, int4Status);	// clear all interrupts        

        mhl_status_value.intr4_mask_value = (USB_EST_INTR4|MHL_EST_INTR4);
        mhl_status_value.intr1_mask_value = RSEN_CHANG;
        mhl_status_value.intr_cbus1_mask_value = 0x00;
        mhl_status_value.intr_cbus2_mask_value = 0x00;

        WRITE_INTR4_VALUE(mhl_status_value.intr4_mask_value);
        WRITE_INTR1_VALUE(mhl_status_value.intr1_mask_value);
        WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
        WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value);    
    } 
    else
    {
        printk("2army, sii9244_mhl_tx_int (2) -> delete_all_cbus_cmd\n");
        delete_all_cbus_cmd(); 
        mhl_tx_init();
    }
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
}


void TX_GoToD0 (void)
{
    SII_LOG_FUNCTION_NAME_ENTRY;
    WriteInitialRegisterValues();

    // Force Power State to ON
    STROBE_POWER_ON
    mhl_status_value.mhl_status = MHL_DISCOVERY_START;     
    SII_LOG_FUNCTION_NAME_EXIT;
    return;
}

#ifdef MHL_WAKEUP_TIMER_MODIFY 
/* To use hrtimer*/
#define	MS_TO_NS(x)	(x * 1000000)

DECLARE_WAIT_QUEUE_HEAD(wake_wq);

static struct hrtimer hr_wake_timer;

static bool wakeup_time_expired;

static bool hrtimer_initialized;
static bool first_timer;

enum hrtimer_restart hrtimer_wakeup_callback(struct hrtimer *timer)
{
	wake_up(&wake_wq);
	wakeup_time_expired = true;
//	hrtimer_cancel(&hr_wake_timer);
	return HRTIMER_NORESTART;
}


void start_hrtimer_ms(unsigned long delay_in_ms)
{
	ktime_t ktime;
	ktime = ktime_set(0, MS_TO_NS(delay_in_ms));

	wakeup_time_expired = false;
//	hrtimer_init(&hr_wake_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	if (first_timer)
		first_timer = false;
	else
		hrtimer_cancel(&hr_wake_timer);

//	hr_wake_timer.function = &hrtimer_wakeup_callback;
	hrtimer_start(&hr_wake_timer, ktime, 0x1);
}
#endif

static void CbusWakeUpPulseGenerator(void)
{

#ifdef MHL_WAKEUP_TIMER_MODIFY 
		if (!hrtimer_initialized) {
			hrtimer_init(&hr_wake_timer, 1, 0x1);
			hr_wake_timer.function = &hrtimer_wakeup_callback;
			hrtimer_initialized = true;
			first_timer = true;
		}
			
		//
		// I2C method
		//
		//I2C_WriteByte(SA_TX_Page0_Primary, 0x92, (I2C_ReadByte(SA_TX_Page0_Primary, 0x92) | 0x10));
	
		// Start the pulse
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
		start_hrtimer_ms(19);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
		start_hrtimer_ms(19);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
		start_hrtimer_ms(19);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
		start_hrtimer_ms(60);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
		start_hrtimer_ms(19);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
		start_hrtimer_ms(19);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
		start_hrtimer_ms(19);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
		I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
	
		start_hrtimer_ms(T_SRC_WAKE_TO_DISCOVER);
		wait_event_interruptible(wake_wq, wakeup_time_expired);
	
#else

    //
    // I2C method
    //
    // Start the pulse
    SII_DEV_DBG("(CbusWakeUpPulseGenerator )\n");
    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
    simg_msleep(T_SRC_WAKE_PULSE_WIDTH_1);	// adjust for code path

    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
    simg_msleep(T_SRC_WAKE_PULSE_WIDTH_1);	// adjust for code path

    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
    simg_msleep(T_SRC_WAKE_PULSE_WIDTH_1 );	// adjust for code path

    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
    simg_msleep(T_SRC_WAKE_PULSE_WIDTH_2 );	// adjust for code path

    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
    simg_msleep(T_SRC_WAKE_PULSE_WIDTH_1 );	// adjust for code path

    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));
    simg_msleep(T_SRC_WAKE_PULSE_WIDTH_1 );	// adjust for code path

    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) | 0xC0));
    simg_msleep(20);

    I2C_WriteByte(PAGE_0_0X72, 0x96, (I2C_ReadByte(PAGE_0_0X72, 0x96) & 0x3F));

    simg_msleep(T_SRC_WAKE_TO_DISCOVER);
#endif

    return;
}


u8 TX_RGND_check (void)
{
	u8 rgndImpedance;

	rgndImpedance = I2C_ReadByte(PAGE_0_0X72, 0x99) & 0x03;
	SII_DEV_DBG("Drv: RGND = %02X \n ", (int)rgndImpedance);

	if (0x02 == rgndImpedance)
	{
		SII_DEV_DBG("(MHL Device)\n");

        mhl_status_value.rgnd_1k = TRUE;

        // Select CBUS drive float.
        SET_BIT(PAGE_0_0X72, 0x95, 5);    

        simg_msleep(T_SRC_VBUS_CBUS_TO_STABLE);

        CbusWakeUpPulseGenerator();   
	}
	else
	{
		SII_DEV_DBG("(Non-MHL Device)\n");
        mhl_status_value.rgnd_1k = FALSE;
	}

    return rgndImpedance;
  
}


void CbusReset (void)
{
	u8		idx;

	SET_BIT(PAGE_0_0X72, 0x05, 3);

    simg_msleep(2);

    CLR_BIT(PAGE_0_0X72, 0x05, 3);

//                                                
/*  
	for(idx=0; idx < 4; idx++)
	{
		// Enable WRITE_STAT interrupt for writes to all 4 MSC Status registers.
		I2C_WriteByte(PAGE_CBUS_0XC8, 0xE0 + idx, 0xFF);

		// Enable SET_INT interrupt for writes to all 4 MSC Interrupt registers.
		I2C_WriteByte(PAGE_CBUS_0XC8, 0xF0 + idx, 0xFF);
	}
*/
//                                                

}


static void InitCBusRegs (void)
{
	u8		regval;

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x07, 0xF2);          // new default is for MHL mode
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x40, 0x03); 			// CBUS Drive Strength
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x42, 0x06); 			// CBUS DDC interface ignore segment pointer
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x36, 0x0C);

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x3D, 0xFD);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x1C, 0x01);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x1D, 0x0F);          // MSC_RETRY_FAIL_LIM
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x44, 0x02);

	// Setup MHL TX devcap
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x80, 0); 
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x81, MHL_VERSION);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x82, (MHL_DEV_CAT_SOURCE));

// CTS 6.3.1.1
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x83, (u8)(322 >>   8));	// CDF check
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x84, (u8)(322 & 0xFF));	// CDF check
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x83, 0x0);
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x84, 0x0);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x83, 0x02); // ADOPTER_ID_H 2
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x84, 0x40); // ADOPTER_ID_L 64

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x85, MHL_DEV_VID_LINK_SUPPRGB444);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x86, MHL_DEV_AUD_LINK_2CH);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x87, 0);										// not for source
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x88, MHL_LOGICAL_DEVICE_MAP);

//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x89, 0);	// bandwidth 0 -> 5	// CDF check		// not for source
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x89, 5);										// not for source
	
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8A, (MHL_FEATURE_RCP_SUPPORT | MHL_FEATURE_RAP_SUPPORT |MHL_FEATURE_SP_SUPPORT));

// CTS 6.3.1.1
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8B, (u8)(0x9244>>   8));	 // CDF check
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8C, (u8)(0x9244& 0xFF)); // CDF check			// reserved
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8B, 0x0);
//	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8C, 0x0);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8B, 0x5C); // DEVICE_ID_H 92
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8C, 0x2C); // DEVICE_ID_H 44


	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8D, 16);
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8E, MHL_INT_AND_STATUS_SIZE);	// CDF check
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x8F, 0);										//reserved

    // Make bits 2,3 (initiator timeout) to 1,1 for register CBUS_LINK_CONTROL_2
    regval = I2C_ReadByte(PAGE_CBUS_0XC8, REG_CBUS_LINK_CONTROL_2 );
    regval = (regval | 0x0C);
    I2C_WriteByte(PAGE_CBUS_0XC8,REG_CBUS_LINK_CONTROL_2, regval);

//                           
///    // Clear legacy bit on Wolverine TX.
///    regval = I2C_ReadByte(PAGE_CBUS_0XC8, REG_MSC_TIMEOUT_LIMIT);
///    regval &= ~MSC_TIMEOUT_LIMIT_MSB_MASK;
///    regval |= 0x0F;
///    I2C_WriteByte(PAGE_CBUS_0XC8, REG_MSC_TIMEOUT_LIMIT, (regval & MSC_TIMEOUT_LIMIT_MSB_MASK));

    // Set NMax to 1
    I2C_WriteByte(PAGE_CBUS_0XC8, REG_CBUS_LINK_CONTROL_1, 0x01);
    ReadModifyWriteCBUS( REG_CBUS_LINK_CONTROL_11, BIT5 | BIT4 | BIT3, BIT5 | BIT4);

    ReadModifyWriteCBUS( REG_MSC_TIMEOUT_LIMIT, 0x0F, 0x0D);

    ReadModifyWriteCBUS(0x2E, BIT4 | BIT2 | BIT0, BIT4 | BIT2 | BIT0);

}


///////////////////////////////////////////////////////////////////////////
// WriteInitialRegisterValues
//
//
///////////////////////////////////////////////////////////////////////////
//                             
//SA_TX_Page1_Primary -> PAGE_1_0x7A
//SA_TX_HDMI_RX_Primary -> PAGE_2_0x92
//SA_TX_Page0_Primary -> PAGE_0_0X72
//SA_TX_CBUS_Primary -> PAGE_CBUS_0XC8
#if 0
void WriteInitialRegisterValues ( void )
{
	SII_DEV_DBG("Drv: WriteInitialRegisterValues\n");
	// Power Up
	I2C_WriteByte(PAGE_1_0x7A, 0x3D, 0x3F);	// Power up CVCC 1.2V core
	I2C_WriteByte(PAGE_2_0x92, 0x11, 0x01);	// Enable TxPLL Clock
	I2C_WriteByte(PAGE_2_0x92, 0x12, 0x15);	// Enable Tx Clock Path & Equalizer
	I2C_WriteByte(PAGE_0_0X72, 0x08, 0x35);	// Power Up TMDS Tx Core

	// Analog PLL Control
	I2C_WriteByte(PAGE_2_0x92, 0x10, 0xC1);	// bits 5:4 = 2b00 as per characterization team.
	I2C_WriteByte(PAGE_2_0x92, 0x17, 0x03);	// PLL Calrefsel
	I2C_WriteByte(PAGE_2_0x92, 0x1A, 0x20);	// VCO Cal
	I2C_WriteByte(PAGE_2_0x92, 0x22, 0x8A);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x23, 0x6A);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x24, 0xAA);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x25, 0xCA);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x26, 0xEA);	// Auto EQ
	I2C_WriteByte(PAGE_2_0x92, 0x4C, 0xA0);	// Manual zone control
	I2C_WriteByte(PAGE_2_0x92, 0x4D, 0x00);	// PLL Mode Value

	I2C_WriteByte(PAGE_0_0X72, 0x80, 0x34);	// Enable Rx PLL Clock Value
	I2C_WriteByte(PAGE_2_0x92, 0x45, 0x44);	// Rx PLL BW value from I2C
	I2C_WriteByte(PAGE_2_0x92, 0x31, 0x0A);	// Rx PLL BW ~ 4MHz
	I2C_WriteByte(PAGE_0_0X72, 0xA0, 0xD0);
	I2C_WriteByte(PAGE_0_0X72, 0xA1, 0xFC);	// Disable internal MHL driver
#ifdef CONFIG_TARGET_LOCALE_KOR
	I2C_WriteByte(PAGE_0_0X72, 0xA3, 0xED);
#else
	I2C_WriteByte(PAGE_0_0X72, 0xA3, 0xFB);
#endif
	I2C_WriteByte(PAGE_0_0X72, 0xA6, 0x0C);


	I2C_WriteByte(PAGE_0_0X72, 0x2B, 0x01);	// Enable HDCP Compliance safety

	//
	// CBUS & Discovery
	// CBUS discovery cycle time for each drive and float = 150us
	//
	//ReadModifyWriteTPI(0x90, BIT_3 | BIT_2, BIT_3);
	ReadModifyWritePage0(0x90, BIT3 | BIT2, BIT3);

	I2C_WriteByte(PAGE_0_0X72, 0x91, 0xA5);		// Clear bit 6 (reg_skip_rgnd)


	// Changed from 66 to 65 for 94[1:0] = 01 = 5k reg_cbusmhl_pup_sel
	//I2C_WriteByte(PAGE_0_0X72, 0x94, 0x65);			// 1.8V CBUS VTH & GND threshold
    I2C_WriteByte(PAGE_0_0X72, 0x94, 0x75);			// 1.8V CBUS VTH & GND threshold

	//set bit 2 and 3, which is Initiator Timeout
	I2C_WriteByte(PAGE_CBUS_0XC8, 0x31, I2C_ReadByte(PAGE_CBUS_0XC8, 0x31) | 0x0c);

	//I2C_WriteByte(PAGE_0_0X72, 0xA5, 0xAC);			// RGND Hysterisis.
    I2C_WriteByte(PAGE_0_0X72, 0xA5, 0xA0);			
	SII_DEV_DBG("Drv: MHL 1.0 Compliant Clock\n");

	// RGND & single discovery attempt (RGND blocking)
	I2C_WriteByte(PAGE_0_0X72, 0x95, 0x31);

	// use 1K and 2K setting
	//I2C_WriteByte(PAGE_0_0X72, 0x96, 0x22);
	// Use VBUS path of discovery state machine
	I2C_WriteByte(PAGE_0_0X72, 0x97, 0x00);

	//ReadModifyWriteTPI(0x95, BIT6, BIT6);		// Force USB ID switch to open
	ReadModifyWritePage0(0x95, BIT6, BIT6);		// Force USB ID switch to open

	//
	// For MHL compliance we need the following settings for register 93 and 94
	// Bug 20686
	//
	// To allow RGND engine to operate correctly.
	//
	// When moving the chip from D2 to D0 (power up, init regs) the values should be
	// 94[1:0] = 01  reg_cbusmhl_pup_sel[1:0] should be set for 5k
	// 93[7:6] = 10  reg_cbusdisc_pup_sel[1:0] should be set for 10k (default)
	// 93[5:4] = 00  reg_cbusidle_pup_sel[1:0] = open (default)
	//

	//WriteByteTPI(0x92, 0x86);				//
	WriteBytePage0(0x92, 0x86);				//
	// change from CC to 8C to match 5K
	//WriteByteTPI(0x93, 0x8C);				// Disable CBUS pull-up during RGND measurement
	WriteBytePage0(0x93, 0x8C);				// Disable CBUS pull-up during RGND measurement
	//NAGSM_Android_SEL_Kernel_Aakash_20101214
 	//ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);	// Force upstream HPD to 0 when not in MHL mode.
	//ReadModifyWriteTPI(0x79, BIT_1 | BIT_2, 0); //Set interrupt active high

	msleep(25);
	//ReadModifyWriteTPI(0x95, BIT_6, 0x00);		// Release USB ID switch
	ReadModifyWritePage0(0x95, BIT6, 0x00);		// Release USB ID switch

	I2C_WriteByte(PAGE_0_0X72, 0x90, 0x27);			// Enable CBUS discovery

	//if (settingMode9290 != pin9290_938x) {
	// Reset CBus to clear state
	CbusReset();
	//}

	InitCBusRegs();

	// Enable Auto soft reset on SCDT = 0
	I2C_WriteByte(PAGE_0_0X72, 0x05, 0x04);

	// HDMI Transcode mode enable
	I2C_WriteByte(PAGE_0_0X72, 0x0D, 0x1C);

 	//I2C_WriteByte(PAGE_0_0X72, 0x78, RGND_RDY_EN);
}
#else
static void WriteInitialRegisterValues (void)
{
	SII_DEV_DBG("Drv: WriteInitialRegisterValues\n");
	// Power Up
	I2C_WriteByte(PAGE_1_0x7A, 0x3D, 0x3F);	
	I2C_WriteByte(PAGE_2_0x92, 0x11, 0x01);	
	I2C_WriteByte(PAGE_2_0x92, 0x12, 0x15);	
	I2C_WriteByte(PAGE_0_0X72, 0x08, 0x35);	

	// Analog PLL Control
	I2C_WriteByte(PAGE_2_0x92, 0x10, 0xC1);	
	I2C_WriteByte(PAGE_2_0x92, 0x17, 0x03);	
	I2C_WriteByte(PAGE_2_0x92, 0x1A, 0x20);	
	I2C_WriteByte(PAGE_2_0x92, 0x22, 0x8A);	
	I2C_WriteByte(PAGE_2_0x92, 0x23, 0x6A);	
	I2C_WriteByte(PAGE_2_0x92, 0x24, 0xAA);	
	I2C_WriteByte(PAGE_2_0x92, 0x25, 0xCA);	
	I2C_WriteByte(PAGE_2_0x92, 0x26, 0xEA);	
	I2C_WriteByte(PAGE_2_0x92, 0x4C, 0xA0);	
	I2C_WriteByte(PAGE_2_0x92, 0x4D, 0x00);	

	I2C_WriteByte(PAGE_0_0X72, 0x80, 0x24);	
	I2C_WriteByte(PAGE_2_0x92, 0x45, 0x44);	
	I2C_WriteByte(PAGE_2_0x92, 0x31, 0x0A);	
	I2C_WriteByte(PAGE_0_0X72, 0xA0, 0xD0);
	I2C_WriteByte(PAGE_0_0X72, 0xA1, 0xFC);	

//                                                                                      
#ifdef MHL_HW_DEBUG
	I2C_WriteByte(PAGE_0_0X72, 0xA3, mhl_ctrl4);
	printk(KERN_ERR "[HDMI]MHL CTRL4 is 0x%x !!!!!!!!!!!!!!!!!!!!!!!!\n",mhl_ctrl4);
#else
	I2C_WriteByte(PAGE_0_0X72, 0xA3, 0xEB);
#endif
//                                                                                      
	I2C_WriteByte(PAGE_0_0X72, 0xA6, 0x0C);

	I2C_WriteByte(PAGE_0_0X72, 0x2B, 0x01);	

	ReadModifyWritePage0(0x90, BIT3 | BIT2, BIT2);


	I2C_WriteByte(PAGE_0_0X72, 0x91, 0xA5);		
	I2C_WriteByte(PAGE_0_0X72, 0x94, 0x77);			
	//                                                                                                     

	I2C_WriteByte(PAGE_CBUS_0XC8, 0x31, I2C_ReadByte(PAGE_CBUS_0XC8, 0x31) | 0x0c);
	I2C_WriteByte(PAGE_0_0X72, 0xA5, 0xA0); 

	SII_DEV_DBG("Drv: MHL 1.0 Compliant Clock\n");

	I2C_WriteByte(PAGE_0_0X72, 0x95, 0x71);
	I2C_WriteByte(PAGE_0_0X72, 0x97, 0x00);


	//WriteBytePage0(0x92, 0x86);				
	WriteBytePage0(0x92, 0x86);							// for CTS 20111118
	WriteBytePage0(0x93, 0x8C);				

    ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	// Force upstream HPD to 0 when not in MHL mode.
    SII_DEV_DBG("Drv: Upstream HPD Acquired - driven low.\n");

#ifdef ACTIV_HIGH_INT
	ReadModifyWritePage0(0x79, BIT_1 | BIT_2, 0); //Set interrupt active high
#endif/*ACTIV_HIGH_INT*/

    simg_msleep(25);
    ReadModifyWritePage0(0x95, BIT6, 0x00);		// Release USB ID switch

    I2C_WriteByte(PAGE_0_0X72, 0x90, 0x27);			// Enable CBUS discovery
	CbusReset();
	InitCBusRegs();

	I2C_WriteByte(PAGE_0_0X72, 0x05, 0x04);

	I2C_WriteByte(PAGE_0_0X72, 0x0D, 0x1C);

}
#endif

void TX_GO2D3 (void)
{
  SII_DEV_DBG(" TX_GO2D3 \n" );

  ForceUsbIdSwitchOpen();

  ReadModifyWritePage0(0x93, BIT7 | BIT6 | BIT5 | BIT4, 0);
  ReadModifyWritePage0(0x94, BIT1 | BIT0, 0);
  ReleaseUsbIdSwitchOpen();
  
  ReadModifyWritePage0(0x79, BIT5 | BIT4, BIT4);	//Force HPD to LOW

  I2C_WriteByte(PAGE_2_0x92, 0x01, 0x03);
  CLR_BIT(PAGE_1_0x7A, 0x3D, 0);  
}

//                                                                                      
#ifdef MHL_HW_DEBUG
void set_mhl_ctrl4(int value)
{
	mhl_ctrl4 = value;
} EXPORT_SYMBOL(set_mhl_ctrl4);
#endif
//                                                                                      
void sii9244_mhl_tx_int(void)
{
    SII_DEV_DBG("sii9244_mhl_tx_int\n" );

    mhl_status_value.mhl_status = NO_MHL_STATUS;
    mhl_status_value.cbus_connected = FALSE;
    mhl_status_value.sink_hpd = FALSE;
    mhl_status_value.linkmode = 0x03; 
    mhl_status_value.connected_ready = 0;
    mhl_status_value.rsen_check_available = FALSE;

    mhl_status_value.intr4_mask_value = RGND_RDY;
    mhl_status_value.intr1_mask_value = 0x00;
    mhl_status_value.intr_cbus1_mask_value = 0x00;
    mhl_status_value.intr_cbus2_mask_value = 0x00;
    mhl_status_value.rgnd_1k = FALSE;

    memset(&mhl_reciever_cap, 0x00, sizeof(mhl_rx_cap_type));

    //                                
    // ADD : for mem leak & null defense
    if(cbus_cmd_head && cbus_cmd_tail)
    {
    	  printk("2army, sii9244_mhl_tx_int -> delete_all_cbus_cmd\n");
        delete_all_cbus_cmd();
    }

    mutex_lock(&cbus_cmd_bind_mutex);	//                  
    //                                
    // ADD : for mem leak & null defense
    if(cbus_cmd_head)
        kfree(cbus_cmd_head);

    //                                
    // ADD : for mem leak & null defense    
    if(cbus_cmd_tail)
        kfree(cbus_cmd_tail);    
    
    cbus_cmd_head = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);
    cbus_cmd_tail = (cbus_cmd_node*)kmalloc(sizeof(cbus_cmd_node), GFP_KERNEL);

    cbus_cmd_head->next = cbus_cmd_tail;
    cbus_cmd_head->prev = cbus_cmd_head;
    cbus_cmd_tail->next = cbus_cmd_tail;
    cbus_cmd_tail->prev = cbus_cmd_head;

    cbus_cmd_head->cmd_send = 0;
    cbus_cmd_tail->cmd_send = 0;
    mutex_unlock(&cbus_cmd_bind_mutex);	//                  

    WriteInitialRegisterValues();

    WRITE_INTR4_VALUE(mhl_status_value.intr4_mask_value);
    WRITE_INTR1_VALUE(mhl_status_value.intr1_mask_value);
    WRITE_CBUS1_VALUE(mhl_status_value.intr_cbus1_mask_value);
    WRITE_CBUS2_VALUE(mhl_status_value.intr_cbus2_mask_value);
  
	TX_GO2D3();
    mhl_status_value.mhl_status =  MHL_WAITING_RGND_DETECT;  

}
EXPORT_SYMBOL(sii9244_mhl_tx_int);


//                  
void sii9244_driver_init(void)
{
	SII_DEV_DBG(" sii9244_driver_init\n" );
	//                                                                          

	//                                      
  	init_timer(&simg_timer);
  	simg_timer.function = simg_timer_handler;
	
}
EXPORT_SYMBOL(sii9244_driver_init);

#endif /*FEATURE_SiI9244_DEVICE*/

