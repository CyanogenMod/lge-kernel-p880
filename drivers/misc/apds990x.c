/*
 *  apds990x.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2011, 2012 LGE, Inc.
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#define SYNC_WITH_NEW_X3   1

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
//#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>


#include <linux/types.h>		/* kernel data types */
#include <linux/kernel.h>		/* printk() */
#include <linux/uaccess.h>		/* copy_from/to_user() */
#include <asm/system.h>


#include <asm/gpio.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/proc_fs.h>

#include <linux/gpio.h>
#include <linux/apds990x.h>

#include <linux/notifier.h>
#include <linux/reboot.h>

#include "../../arch/arm/mach-tegra/gpio-names.h"
#include "../../arch/arm/mach-tegra/lge/x3/include/lge/board-x3-nv.h"
#include "../../arch/arm/mach-tegra/lge/x3/include/lge/board-x3.h"

#if 0
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#endif 

#define NO_FIRST_INTERRUPT_BUGFIX 1

#define APDS990x_DRV_NAME	"apds990x"
#define DRIVER_VERSION		"1.0.4"

//#define APDS990x_INT		IRQ_EINT20

//Proximity Distance : 4.5Cm
#define APDS990x_PS_DETECTION_THRESHOLD						900
#define APDS990x_PS_HSYTERESIS_THRESHOLD					820

#define APDS990x_PS_DEFAULT_PPCOUNT_REV_B					0x07			// 7-Pulse for proximity 
#define APDS990x_PS_DEFAULT_CROSSTALK_VAL_REV_B				150

#define APDS990x_PS_DEFAULT_PPCOUNT_REV_D					0x03			// 4-Pulse for proximity 
#define APDS990x_PS_DEFAULT_CROSSTALK_VAL_REV_D				650


//Calculation parameters
#define APDS990x_PS_DETECTION_THRESHOLD_BASE				130
#define APDS990x_PS_HSYTERESIS_THRESHOLD_SUBTRACT_VAL		40

//ALS - BLACK WINDOW
#define APDS990x_BLACK_ALS_DEFAULT_GA						271
#define APDS990x_BLACK_ALS_DEFAULT_COE_B					231
#define APDS990x_BLACK_ALS_DEFAULT_COE_C					34
#define APDS990x_BLACK_ALS_DEFAULT_COE_D					65

//ALS - WHITE WINDOW
#define APDS990x_WHITE_ALS_DEFAULT_GA						243
#define APDS990x_WHITE_ALS_DEFAULT_COE_B					199
#define APDS990x_WHITE_ALS_DEFAULT_COE_C					71
#define APDS990x_WHITE_ALS_DEFAULT_COE_D					129


#define APDS990x_ALS_THRESHOLD_HSYTERESIS	20	/* 20 = 20% */

/* Change History 
 *
 * 1.0.1	Functions apds990x_show_rev(), apds990x_show_id() and apds990x_show_status()
 *			have missing CMD_BYTE in the i2c_smbus_read_byte_data(). APDS-990x needs
 *			CMD_BYTE for i2c write/read byte transaction.
 *
 *
 * 1.0.2	Include PS switching threshold level when interrupt occurred
 *
 *
 * 1.0.3	Implemented ISR and delay_work, correct PS threshold storing
 *
 * 1.0.4	Added Input Report Event
 */

/*
 * Defines
 */

#define APDS990x_ENABLE_REG	0x00
#define APDS990x_ATIME_REG	0x01
#define APDS990x_PTIME_REG	0x02
#define APDS990x_WTIME_REG	0x03
#define APDS990x_AILTL_REG	0x04
#define APDS990x_AILTH_REG	0x05
#define APDS990x_AIHTL_REG	0x06
#define APDS990x_AIHTH_REG	0x07
#define APDS990x_PILTL_REG	0x08
#define APDS990x_PILTH_REG	0x09
#define APDS990x_PIHTL_REG	0x0A
#define APDS990x_PIHTH_REG	0x0B
#define APDS990x_PERS_REG	0x0C
#define APDS990x_CONFIG_REG	0x0D
#define APDS990x_PPCOUNT_REG	0x0E
#define APDS990x_CONTROL_REG	0x0F
#define APDS990x_REV_REG	0x11
#define APDS990x_ID_REG		0x12
#define APDS990x_STATUS_REG	0x13
#define APDS990x_CDATAL_REG	0x14
#define APDS990x_CDATAH_REG	0x15 //
#define APDS990x_IRDATAL_REG	0x16
#define APDS990x_IRDATAH_REG	0x17 //
#define APDS990x_PDATAL_REG	0x18
#define APDS990x_PDATAH_REG	0x19 //

#define CMD_BYTE		0x80
#define CMD_WORD		0xA0
#define CMD_SPECIAL		0xE0

#define CMD_CLR_PS_INT		0xE5
#define CMD_CLR_ALS_INT		0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

#define APDS900_SENSOR_DEBUG 	1
#if APDS900_SENSOR_DEBUG
#define DEBUG_MSG(args...)  printk(args)
#else
#define DEBUG_MSG(args...)
#endif

// Interrupt define
//#define  APDS990x_IRQ;		//(82) //IRQ_EINT(27)
#define APDS990x_ATTB 		(S5PV210_GPH3(3))

#define STORE_INTERRUPT_SELECT_PROX 	0x02
#define STORE_INTERRUPT_SELECT_ALS 	0x04

static int APDS990x_IRQ;		//(82) //IRQ_EINT(27)

/*                     
                                                                        
                 
               
               
*/
enum {
  DEBUG_ERR_CHECK     = 1U << 0,
  DEBUG_USER_ERROR    = 1U << 1,
  DEBUG_FUNC_TRACE    = 1U << 2,
  DEBUG_DEV_STATUS    = 1U << 3,
  DEBUG_DEV_DEBOUNCE  = 1U << 4,
  DEBUG_GEN_INFO      = 1U << 5,
  DEBUG_INTR_INFO     = 1U << 6,
};

static unsigned int debug_mask = DEBUG_USER_ERROR;		//All Log : 0x7F


module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/*
 * Structs
 */

struct apds990x_data {
	struct i2c_client *client;
	//struct mutex update_lock;
	struct delayed_work	dwork;	/* for PS interrupt */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct apds990x_proximity_platform_data *pdata;

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold; /* always lower than ps_threshold */
	unsigned int ps_detection;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;			/* to store PS data */
	unsigned int cross_talk;		/* cross_talk value */

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;			/* to store ALS data */

	unsigned int als_gain;			/* needed for Lux calculation */
	unsigned int als_poll_delay;		/* needed for light sensor polling : micro-second (us) */
	unsigned int als_atime;			/* storage for als integratiion time */
	unsigned int undersunlight;

	unsigned char irq_wake;
    unsigned int ps_default_result;
	unsigned int ps_cal_result;

	unsigned int is_white;
};

/*
 * Global data
 */
static int apds990x_proxidata = 0;
static int apds990x_luxdata = 0;
static int apds990x_enable_backup = 0;

static struct i2c_client *this_client = 0;
static int cross_talk_val = 0;
static int ppcount_val = 0;

// GA, COE_B, COE_C, COE_D each multiply 100 and round off to the nearest whole number

static int PS_DEFAULT_PPCOUNT = APDS990x_PS_DEFAULT_PPCOUNT_REV_D;
static int PS_DEFAULT_CROSSTALK_VAL = APDS990x_PS_DEFAULT_CROSSTALK_VAL_REV_D;

static int GA = APDS990x_WHITE_ALS_DEFAULT_GA; 		/* 2.485 --- 0.48 without glass window ..old: 271*/
static int COE_B = APDS990x_WHITE_ALS_DEFAULT_COE_B;		/* 1.973 --- 2.23 without glass window ..old: 231*/
static int COE_C = APDS990x_WHITE_ALS_DEFAULT_COE_C;		/* 0.694 --- 0.70 without glass window .. old: 34*/
static int COE_D = APDS990x_WHITE_ALS_DEFAULT_COE_D;		/* 1.322 --- 1.42 without glass window .. old: 65*/

/*
 * Management functions
 */
static int apds990x_init_client(struct i2c_client *client);
static int apds990x_init_client_for_cal(struct i2c_client *client);

static void apds990x_Set_PS_Threshold_Adding_Cross_talk(struct i2c_client *client, int cal_data);

static s32 __init prox_cal_data(char *str)
{
		sscanf(str,"%d,%d ",&cross_talk_val, &ppcount_val);

        printk("[apds990x] get_prox_cal_data(): cross_talk_val = %d, ppcount_val = %d\n", cross_talk_val, ppcount_val);
        return 1;
}
__setup("prox_cal_data=", prox_cal_data);


static int apds990x_set_command(struct i2c_client *client, int command)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;
		
	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	//mutex_unlock(&data->update_lock);

	return ret;
}

static int apds990x_set_atime(struct i2c_client *client, int atime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_ATIME_REG, atime);
	//mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}

static int apds990x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_PTIME_REG, ptime);
	//mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds990x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_WTIME_REG, wtime);
	//mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds990x_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_AILTL_REG, threshold);
	//mutex_unlock(&data->update_lock);
	
	data->ailt = threshold;

	return ret;
}

static int apds990x_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_AIHTL_REG, threshold);
	//mutex_unlock(&data->update_lock);
	
	data->aiht = threshold;

	return ret;
}

static int apds990x_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_PILTL_REG, threshold);
	//mutex_unlock(&data->update_lock);
	
	data->pilt = threshold;

	return ret;
}

static int apds990x_set_piht(struct i2c_client *client, int threshold)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_PIHTL_REG, threshold);
	//mutex_unlock(&data->update_lock);
	
	data->piht = threshold;

	return ret;
}

static int apds990x_set_pers(struct i2c_client *client, int pers)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_PERS_REG, pers);
	//mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds990x_set_config(struct i2c_client *client, int config)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_CONFIG_REG, config);
	//mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds990x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_PPCOUNT_REG, ppcount);
	//mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds990x_set_control(struct i2c_client *client, int control)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;

	//mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_CONTROL_REG, control);
	//mutex_unlock(&data->update_lock);

	data->control = control;

	/* obtain ALS gain value */
	if ((data->control&0x03) == 0x00) /* 1X Gain */
		data->als_gain = 1;
	else if ((data->control&0x03) == 0x01) /* 8X Gain */
		data->als_gain = 8;
	else if ((data->control&0x03) == 0x02) /* 16X Gain */
		data->als_gain = 16;
	else  /* 120X Gain */
		data->als_gain = 120;

	return ret;
}

static int apds990x_set_enable(struct i2c_client *client, int enable)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int ret;
#if defined(APDS900_SENSOR_DEBUG)
	int at,pt,wt,ppc,cfg,pers;
#endif
	//Reliability check       
	ret = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_CONTROL_REG);
    	
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error %d in reading reg 0x%x\n",  __func__, ret, CMD_BYTE|APDS990x_CONTROL_REG);
		return ret;
	}    
	
#if defined(APDS900_SENSOR_DEBUG)
	if(DEBUG_DEV_DEBOUNCE & debug_mask) 
	{
		//Check regs.
		at = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_ATIME_REG);

		pt = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_PTIME_REG);

		wt = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_WTIME_REG);
		
		ppc = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_PPCOUNT_REG);		//notice: ppcount is changed..
		
		cfg = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_CONFIG_REG);

		pers = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_PERS_REG);

		printk("[%s]### en: %d, ctl: %d(%d), at: %d, pt: %d, wt: %d, pp: %d, conf: %d, pers: %d, \n",__func__, \
			enable, ret, data->control,at,pt,wt, ppc, cfg, pers);
	}

#endif
	if(data->control != ret)
	{
		//paremeter (re)init
		// sensor is in disabled mode but all the configurations are preset
		
#if defined(APDS900_SENSOR_DEBUG)
        	//if(DEBUG_DEV_DEBOUNCE & debug_mask) 
                printk("[%s]### control: %d, ret: %d ",__func__, data->control, ret);
#endif

		apds990x_set_atime(client, data->als_atime);	// 100.64ms ALS integration time
		apds990x_set_ptime(client, 0xFF);	// 2.72ms Prox integration time
		apds990x_set_wtime(client, 0xFF);	// 2.72ms Wait time
		apds990x_set_ppcount(client, data->ppcount);	// 7-Pulse for proximity        // old:8
		apds990x_set_config(client, 0);		// no long wait
		apds990x_set_control(client, 0x20);	// 100mA, IR-diode, 1X PGAIN, 1X AGAIN
	
//		apds990x_set_pilt(client, 1023);	// to force first Near-to-Far interrupt
//		apds990x_set_piht(client, 0);
		//apds990x_set_piht(client, APDS990x_PS_DETECTION_THRESHOLD);		//minhyup.park change

		//data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD;
		//data->ps_hysteresis_threshold = APDS990x_PS_HSYTERESIS_THRESHOLD;
		apds990x_Set_PS_Threshold_Adding_Cross_talk(client,data->cross_talk);
		data->ps_detection = 1;			// we are forcing Near-to-Far interrupt, so this is defaulted to 1

#ifdef NO_FIRST_INTERRUPT_BUGFIX
		apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
		apds990x_set_aiht(client, 0);		// in order get current ALS reading
#endif		
		//apds990x_set_pers(client, 0x33);  // 5 consecutive Interrupt persistence old : /* 3 persistence */
		apds990x_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
										// 16-Feb-2012 KK
										// Force both ALS PS interrupt every ALS PS conversion cycle
										// instead of comparing threshold value
	}
	
    //mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_ENABLE_REG, enable);
    //mutex_unlock(&data->update_lock);

	data->enable = enable;

	return ret;
}

static int LuxCalculation(struct i2c_client *client, int cdata, int irdata)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int luxValue=0;

	int IAC1=0;
	int IAC2=0;
	int IAC=0;

	// GA, COE_B, COE_C, COE_D each multiply 100 and round off to the nearest whole number
	int DF=52;

	IAC1 = (cdata - (COE_B*irdata)/100);	// re-adjust COE_B to avoid 2 decimal point
	IAC2 = ((COE_C*cdata)/100 - (COE_D*irdata)/100); // re-adjust COE_C and COE_D to void 2 decimal point

	if (IAC1 > IAC2)
		IAC = IAC1;
	else if (IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;

	luxValue = ((IAC*GA*DF)/100)/(((272*(256-data->atime))/100)*data->als_gain);
#if defined(APDS900_SENSOR_DEBUG)
	if(DEBUG_DEV_DEBOUNCE & debug_mask)  
		printk("%s : cdata[%d], irdata[%d] ++++ IAC[%d] atime[%d] als_gain[%d] *** \n",__func__,cdata,irdata,IAC,data->atime,data->als_gain);
#endif

	return luxValue;
}

void apds990x_swap(int *x, int *y)
{
	int temp = *x;
	*x = *y;
	*y = temp;
}

/* Find cross talk value. when run a All Auto Test. you have to save the cross-talk value. */
static int apds990x_Run_Cross_talk_Calibration(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned int sum_of_pdata = 0,temp_pdata[20];
	unsigned int i=0,j=0,ArySize = 20,cal_check_flag = 0;
	unsigned int white_check_flag = 0;

RE_CALIBRATION:
	
	apds990x_init_client_for_cal(client);
    apds990x_set_ppcount(client, data->ppcount);
	apds990x_set_enable(client, 0x0D);/* Enable PS and Wait */

#if defined(CHECK_WHITE_AND_BLACK_WIN)
	if (white_check_flag == 0)
		apds990x_set_ppcount(client, 0x07);		//is Black?
	else
		apds990x_set_ppcount(client, 0x04);		//is white?
#endif

	mdelay(100);

	for(i =0;i<20;i++)	{
		mdelay(6);
		temp_pdata[i] = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_PDATAL_REG);
#if defined(APDS900_SENSOR_DEBUG)
		if(DEBUG_DEV_DEBOUNCE & debug_mask)
			printk("%s : during cal... temp_pdata[%d] = %d \n",__func__,i,temp_pdata[i]);
#endif
	}

	// pdata sorting
	for(i=0; i<ArySize-1; i++)
		for(j=i+1; j<ArySize; j++)
			if(temp_pdata[i] > temp_pdata[j])
				apds990x_swap(temp_pdata+i, temp_pdata+j);

	for (i = 5;i<15;i++)
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	
	data->cross_talk = sum_of_pdata/10;


#if defined(APDS900_SENSOR_DEBUG)
    if(DEBUG_DEV_DEBOUNCE & debug_mask)  
         printk("during cal... data->cross_talk = %d \n",data->cross_talk);
#endif
	
	if (data->cross_talk > 720)
	{
#if defined(APDS900_SENSOR_DEBUG)
		if(DEBUG_DEV_DEBOUNCE & debug_mask)  
			printk("during cal... white_check_flag[%d], cal_check_flag[%d] \n", white_check_flag, cal_check_flag);
#endif	

#if defined(CHECK_WHITE_AND_BLACK_WIN)
		//black or white
		if (white_check_flag == 0)
		{
			white_check_flag = 1;
			goto RE_CALIBRATION;
		}
#endif
		if (cal_check_flag == 0)
		{
			cal_check_flag = 1;
			sum_of_pdata = 0;
			goto RE_CALIBRATION;
		}
		else
		{
			apds990x_set_enable(client,0x00); /* Power Off */
			apds990x_set_ppcount(client, PS_DEFAULT_PPCOUNT);
			return -1;
		}
	}

#if defined(CHECK_WHITE_AND_BLACK_WIN)
	if(white_check_flag == 1)
		data->is_white = 1;
	else
		data->is_white = 0;
#endif

	data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD_BASE + data->cross_talk;
	data->ps_hysteresis_threshold = data->ps_threshold - APDS990x_PS_HSYTERESIS_THRESHOLD_SUBTRACT_VAL;

	ppcount_val = data->ppcount;

	apds990x_set_enable(client,0x00); /* Power Off */
	return data->cross_talk; //Save the cross-talk to the non-volitile memory in the phone.  
}

/* Apply cross-talk value that find Cross-talk Calibration to threshold */
static void apds990x_Set_PS_Threshold_Adding_Cross_talk(struct i2c_client *client, int cal_data)
{
	struct apds990x_data *data = i2c_get_clientdata(client);

	if (cal_data>720)
		cal_data = 720;
	if (cal_data<0)
		cal_data = 0;
	
	data->cross_talk = cal_data;
	data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD_BASE + cal_data;
	data->ps_hysteresis_threshold = data->ps_threshold - APDS990x_PS_HSYTERESIS_THRESHOLD_SUBTRACT_VAL;

    apds990x_set_pilt(client, 1023);	// to force first Near-to-Far interrupt
	apds990x_set_piht(client, 0);

}

static void apds990x_change_ps_threshold(struct i2c_client *client)
{
	int irdata=0;
	struct apds990x_data *data = i2c_get_clientdata(client);

	data->ps_data =	i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_PDATAL_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_IRDATAL_REG);

	if ( (data->ps_data > data->pilt) && (data->ps_data >= data->piht) && (irdata != (100*(1024*(256-data->atime)))/100)) {
		/* FAR-to-NEAR */
		data->ps_detection = 1;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 3);/* FAR-to-NEAR detection */	
		input_sync(data->input_dev_ps);

		apds990x_set_pilt(client, data->ps_hysteresis_threshold);
		apds990x_set_piht(client, 1023);

		data->pilt = data->ps_hysteresis_threshold;
		data->piht = 1023;

		printk("\n ===> far-to-NEAR -- distance: 3 \n\n");
	}
	else if ( (data->ps_data <= data->pilt) && (data->ps_data < data->piht) ) {
		/* NEAR-to-FAR */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 10);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		apds990x_set_pilt(client, 0);
		apds990x_set_piht(client, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		printk("\n ===> near-to-FAR -- distance: 10 \n\n");
	}
	else if ( (irdata == (100*(1024*(256-data->atime)))/100) && (data->ps_detection == 1) ) {
		/* under strong ambient light*/
		/*NEAR-to-FAR */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 10);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		/*Keep the threshold NEAR condition to prevent the frequent Interrupt under strong ambient light*/
		apds990x_set_pilt(client, data->ps_hysteresis_threshold);
		apds990x_set_piht(client, 1023);
			
		data->pilt = data->ps_hysteresis_threshold;
		data->piht = 1023;

		printk("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light *\n");
		printk("\n ===> near-to-FAR\n\n");
	}
	
	else if ( (data->pilt == 1023) && (data->piht == 0) )
	{
		/* this is the first near-to-far forced interrupt */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 10);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		apds990x_set_pilt(client, 0);
		apds990x_set_piht(client, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;
		printk("[SENSOR-APDS] near-to-FAR(4)\n\n");
	}

	apds990x_proxidata = data->ps_detection;
}

static void apds990x_change_als_threshold(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int cdata, irdata;
	int luxValue=0;

	//printk("[SENSOR-APDS] %s \n",__func__);

	cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_CDATAL_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_IRDATAL_REG);

	if(data->enable_als_sensor==1){
                luxValue = LuxCalculation(client, cdata, irdata);

		//check low light level
		luxValue = luxValue<0 ? 0 : luxValue;

                luxValue = luxValue>0 ? luxValue : 0;
                luxValue = luxValue<20000 ? luxValue : 20000;

                if(cdata == (100*(1024*(256-data->atime)))/100)
                	luxValue=20000;

        	apds990x_luxdata = luxValue;

        	input_report_abs(data->input_dev_als, ABS_MISC, luxValue); // report the lux level
        	input_sync(data->input_dev_als);

	         printk("[SENSOR-APDS] LuxValue = %d \n",luxValue); 
#if defined(APDS900_SENSOR_DEBUG)
                if(DEBUG_DEV_DEBOUNCE & debug_mask)
                {	
                        DEBUG_MSG("[apds990x] LuxValue = %d \n",luxValue);
                }    	
#endif
        	data->als_data = cdata;

        	data->als_threshold_l = (data->als_data * (100-APDS990x_ALS_THRESHOLD_HSYTERESIS) ) /100;
        	data->als_threshold_h = (data->als_data * (100+APDS990x_ALS_THRESHOLD_HSYTERESIS) ) /100;

                //if (data->als_threshold_h >= 65535) data->als_threshold_h = 65535;
                if (data->als_threshold_h >= (100*(1024*(256-data->atime)))/100) data->als_threshold_h = (100*(1024*(256-data->atime)))/100;

#ifdef ALC_THRESHOLD_CONDITION_CHANGE
                /* Set the different ALS interrupt threshold depending on the lux level */		
                if(luxValue < 100){
                        data->als_threshold_l = (data->als_data * (100-40) ) /100;
                        data->als_threshold_h = (data->als_data * (100+40) ) /100;
                }
                if(luxValue < 50){
                        data->als_threshold_l = (data->als_data * (100-50) ) /100;
                        data->als_threshold_h = (data->als_data * (100+50) ) /100;
                }
                if(luxValue < 20){
                        data->als_threshold_l = 0;
                        data->als_threshold_h = (data->als_data * (100+100) ) /100;
                }
#endif                
        }
        /*Set ALS threshold when PS is only enabled*/
        else if(data->enable_ps_sensor==1 && data->enable_als_sensor==0){
                if(cdata==(100*(1024*(256-data->atime)))/100){
                        data->als_threshold_l = (99*(1024*(256-data->atime)))/100;
                        data->als_threshold_h = (100*(1024*(256-data->atime)))/100;
                }
                else {
                        data->als_threshold_l = 0;
                        data->als_threshold_h = (99*(1024*(256-data->atime)))/100;
                }
        }


	apds990x_set_ailt(client,data->als_threshold_l);
	apds990x_set_aiht(client,data->als_threshold_h);
		
}

static void apds990x_reschedule_work(struct apds990x_data *data,
					  unsigned long delay)
{
	//unsigned long flags;

	//spin_lock_irqsave(&data->update_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, delay);

//	spin_unlock_irqrestore(&data->update_lock, flags);
}

/* PS interrupt routine */
static void apds990x_work_handler(struct work_struct *work)
{
	struct apds990x_data *data = container_of(work, struct apds990x_data, dwork.work);
	struct i2c_client *client=data->client;
	int status=0;
	int cdata=0;
	int pdata=0;
	int irdata=0;
	int ailt=0,aiht=0;
	int pilt=0,piht=0;

	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_STATUS_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_IRDATAL_REG);

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_ENABLE_REG, 1);	/* disable 990x's ADC first */

#if defined(APDS900_SENSOR_DEBUG)
	if(DEBUG_DEV_DEBOUNCE & debug_mask)
		DEBUG_MSG("status = %x\n", status);
#endif	

#ifdef SYNC_WITH_NEW_X3
   apds990x_set_pers(client, 0x33);	// 16-Feb-2012 KK
										// repeat this because of the force first interrupt
#endif
	
	if ((status & data->enable & 0x30) == 0x30) {
		/* both PS and ALS are interrupted */
		apds990x_change_als_threshold(client);


		/* check if this is triggered by strong ambient light */	
		if (irdata != (100*(1024*(256-data->atime)))/100){
			apds990x_change_ps_threshold(client);
			}
		else {
			if (data->ps_detection == 1) {
				apds990x_change_ps_threshold(client);				
				
				printk("* Triggered by strong ambient light\n");	
				printk("\n ===> near-to-FAR\n");
			}
			else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrup under strong ambient light*/
				apds990x_set_pilt(client, data->ps_hysteresis_threshold);
				apds990x_set_piht(client, 1023);			
				data->pilt = data->ps_hysteresis_threshold;
				data->piht = 1023;
				printk("* [SENSOR-APDS]Set PS Threshold NEAR condition to prevent the frequent Interrup under strong ambient light\n");
				printk("* [SENSOR-APDS]Triggered by strong ambient light\n\n");
				printk("\n [SENSOR-APDS]==> maintain FAR \n\n");
			}
		}

		apds990x_set_command(client, 2);	/* 2 = CMD_CLR_PS_ALS_INT */
	}
	else if ((status & data->enable & 0x20) == 0x20) {
		/* only PS is interrupted */
		
		/* check if this is triggered by strong ambient light */
		if (irdata != (100*(1024*(256-data->atime)))/100){
			apds990x_change_ps_threshold(client);
			}
		else {
			if (data->ps_detection == 1) {
				apds990x_change_ps_threshold(client);	
		        printk("* [SENSOR-APDS]Triggered by background ambient noise\n");	
				printk("\n [SENSOR-APDS]===> near-to-FAR\n");
					
			}
			else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrup under strong ambient light*/
				apds990x_set_pilt(client, data->ps_hysteresis_threshold);
				apds990x_set_piht(client, 1023);
				
				data->pilt = data->ps_hysteresis_threshold;
				data->piht = 1023;
				printk("* [SENSOR-APDS]Set PS Threshold NEAR condition to prevent the frequent Interrup under strong ambient light\n");
				printk("* [SENSOR-APDS]Triggered by strong ambient light\n");
				printk("\n [SENSOR-APDS]==> maintain FAR \n\n");

			}
		}
		apds990x_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
	}
	else if ((status & data->enable & 0x10) == 0x10) {
		/* only ALS is interrupted */
		apds990x_change_als_threshold(client);

		if(data->enable_ps_sensor==1){
			/* check if this is triggered by strong ambient light */
			if (irdata != (100*(1024*(256-data->atime)))/100){
				apds990x_change_ps_threshold(client);
				data->undersunlight = 0;
				}
			else{								 
				if (data->ps_detection == 1) {
					apds990x_change_ps_threshold(client);			
			
					printk("* [SENSOR-APDS]Triggered by background ambient noise\n");
					printk("\n [SENSOR-APDS]===> near-to-FAR\n");
				}
				else {
			
					/*Keep the threshold NEAR condition to prevent the frequent Interrup under strong ambient light*/
									
					apds990x_set_pilt(client, data->ps_hysteresis_threshold);
					apds990x_set_piht(client, 1023);									
					data->pilt = data->ps_hysteresis_threshold;
					data->piht = 1023;
					printk("* [SENSOR-APDS]Set PS Threshold NEAR condition to prevent the frequent Interrup under strong ambient light\n");
					printk("* [SENSOR-APDS]Triggered by strong ambient light\n");
					printk("\n [SENSOR-APDS]==> maintain FAR \n\n");
				}
			}
		}
		apds990x_set_command(client, 1);	/* 1 = CMD_CLR_ALS_INT */
	}

#if defined(APDS900_SENSOR_DEBUG)
	if(DEBUG_DEV_DEBOUNCE & debug_mask)
	{
		pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_PDATAL_REG);
		cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_CDATAL_REG);
		irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_IRDATAL_REG);
		
		ailt = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_AILTL_REG);
		aiht = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_AIHTL_REG);
		pilt = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_PILTL_REG);
		piht = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_PIHTL_REG);

		printk("[SENSOR-APDS] cdata = %d, irdata = %d, pdata = %d\n", cdata,irdata,pdata);
		printk("[SENSOR-APDS] ailt = %d, aiht = %d\n", ailt,aiht);
		printk("[SENSOR-APDS] pilt = %d, piht = %d\n", pilt,piht);

		printk("--------------------\n");
	}
#endif

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_ENABLE_REG, data->enable);	
}

/* assume this is ISR */
static irqreturn_t apds990x_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds990x_data *data = i2c_get_clientdata(client);

#if defined(APDS900_SENSOR_DEBUG)
	if(DEBUG_DEV_DEBOUNCE & debug_mask)
		DEBUG_MSG("==> apds990x_interrupt\n");
#endif	
	apds990x_reschedule_work(data, 0);

	return IRQ_HANDLED;
}

/*
 * SysFS support
 */

static ssize_t apds990x_show_run_calibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->cross_talk);
}

static ssize_t apds990x_store_run_calibration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	int ret;

	ret = apds990x_Run_Cross_talk_Calibration(client);

    //Store Calibration data
    if(  0 <= ret && ret < 720 )
    {
        DEBUG_MSG("%s Succes cross-talk :  %d, ppcount(%d), is_white(%d) \n", __func__, ret, data->ppcount, data->is_white);
        lge_nvdata_write(LGE_NVDATA_PROXIMITY_PPCOUNT_OFFSET, &ppcount_val, 1);
        lge_nvdata_write(LGE_NVDATA_PROXIMITY_CROSS_TALK_CALIBRATION_OFFSET, &ret, 2);
		data->ps_cal_result = 1;

#if defined(CHECK_WHITE_AND_BLACK_WIN)
		if(data->is_white == 1)
		{
			GA = APDS990x_WHITE_ALS_DEFAULT_GA; 		/* 2.485 --- 0.48 without glass window ..old: 271*/
			COE_B = APDS990x_WHITE_ALS_DEFAULT_COE_B;		/* 1.973 --- 2.23 without glass window ..old: 231*/
			COE_C = APDS990x_WHITE_ALS_DEFAULT_COE_C;		/* 0.694 --- 0.70 without glass window .. old: 34*/
			COE_D = APDS990x_WHITE_ALS_DEFAULT_COE_D;		/* 1.322 --- 1.42 without glass window .. old: 65*/
		}
		else
		{
			GA = APDS990x_BLACK_ALS_DEFAULT_GA; 		/* 2.485 --- 0.48 without glass window ..old: 271*/
			COE_B = APDS990x_BLACK_ALS_DEFAULT_COE_B;		/* 1.973 --- 2.23 without glass window ..old: 231*/
			COE_C = APDS990x_BLACK_ALS_DEFAULT_COE_C;		/* 0.694 --- 0.70 without glass window .. old: 34*/
			COE_D = APDS990x_BLACK_ALS_DEFAULT_COE_D;		/* 1.322 --- 1.42 without glass window .. old: 65*/
		}
#endif

    }
	else
    {
		DEBUG_MSG("%s Fail error :  %d\n", __func__, ret);
		data->ps_cal_result = 0;
	}

	return count;
}

static DEVICE_ATTR(run_calibration,  0660,
		   apds990x_show_run_calibration, apds990x_store_run_calibration);

 static ssize_t apds990x_show_default_calibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->ps_default_result);
}

static ssize_t apds990x_store_default_calibration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	int default_cal = PS_DEFAULT_CROSSTALK_VAL;
	int default_ppcount = PS_DEFAULT_PPCOUNT;

	lge_nvdata_write(LGE_NVDATA_PROXIMITY_PPCOUNT_OFFSET, &default_ppcount, 1);
	lge_nvdata_write(LGE_NVDATA_PROXIMITY_CROSS_TALK_CALIBRATION_OFFSET, &default_cal, 2);
	
	if(default_cal < 0)
	{
		DEBUG_MSG("%s Fail error :  %d\n", __func__, default_cal);
		data->ps_default_result = 0;
	}
	else
	{
		DEBUG_MSG("%s Succes cross-talk :  %d\n", __func__, default_cal);
        data->ps_cal_result = 0;
		data->ps_default_result = 1;
		data->ppcount = PS_DEFAULT_PPCOUNT;
		data->cross_talk = PS_DEFAULT_CROSSTALK_VAL;
		data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD_BASE + data->cross_talk;
		data->ps_hysteresis_threshold = data->ps_threshold - APDS990x_PS_HSYTERESIS_THRESHOLD_SUBTRACT_VAL;
	}

	return count;
}
static DEVICE_ATTR(default_calibration,  0660,
		   apds990x_show_default_calibration, apds990x_store_default_calibration);

static ssize_t apds990x_show_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int status = 0;

	status = i2c_smbus_read_byte_data( client, CMD_BYTE | APDS990x_STATUS_REG);
	if (status <0) {
		dev_err(&client->dev, "%s: i2c error %d in reading reg 0x%x\n", 
			__func__, status, CMD_BYTE|APDS990x_STATUS_REG);
		return status;
	}
	return sprintf(buf, "%x\n", status);
}
static DEVICE_ATTR(status, 0660, apds990x_show_status, NULL);


static ssize_t apds990x_show_atime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%x\n", data->atime);
}

static ssize_t apds990x_store_atime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds990x_set_atime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(atime,  0660,
		   apds990x_show_atime, apds990x_store_atime);

static ssize_t apds990x_show_ptime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%x\n", data->ptime);
}

static ssize_t apds990x_store_ptime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds990x_set_ptime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ptime,  0660,
		   apds990x_show_ptime, apds990x_store_ptime);

static ssize_t apds990x_show_wtime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%x\n", data->wtime);
}

static ssize_t apds990x_store_wtime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds990x_set_wtime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(wtime,  0660,
		   apds990x_show_wtime, apds990x_store_wtime);


static ssize_t apds990x_show_pers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%x\n", data->pers);
}

static ssize_t apds990x_store_pers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds990x_set_pers(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(pers,  0660,
		   apds990x_show_pers, apds990x_store_pers);


static ssize_t apds990x_show_ppcount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ppcount);
}

static ssize_t apds990x_store_ppcount(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds990x_set_ppcount(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ppcount,  0660,
		   apds990x_show_ppcount, apds990x_store_ppcount);

static ssize_t apds990x_show_control(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%x\n", data->control);
}

static ssize_t apds990x_store_control(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds990x_set_control(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(control,  0660,
		   apds990x_show_control, apds990x_store_control);

static ssize_t apds990x_show_ps_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_threshold);
}

static ssize_t apds990x_store_ps_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t ps_th)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->ps_threshold = val;

	return ps_th;
}

static DEVICE_ATTR(ps_threshold,  0660,
		   apds990x_show_ps_threshold, apds990x_store_ps_threshold);

static ssize_t apds990x_show_ps_hysteresis_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_hysteresis_threshold);
}

static ssize_t apds990x_store_ps_hysteresis_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t ps_hy_th)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->ps_hysteresis_threshold = val;

	return ps_hy_th;
}

static DEVICE_ATTR(ps_hysteresis,  0660,
		   apds990x_show_ps_hysteresis_threshold, apds990x_store_ps_hysteresis_threshold);

static ssize_t apds990x_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	printk("%s: Check the enable_ps_sensor!! ( %d)\n", __func__, __LINE__);
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds990x_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	//printk(KERN_ERR, "%s: Check the enable_ps_sensor!! ( %ld)\n", __func__, __LINE__);
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

 	/*
	printk("[prox] val= %x\n",val); //minhyup.park
	if ((val != 0) && (val != 1)) {
		printk("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}*/
	if(val & STORE_INTERRUPT_SELECT_PROX){
		if(val & 1) {
			printk("\n ===>  apds990x_store_enable_ps_sensor(turn on) === \n\n");
			//turn on p sensor
			if (data->enable_ps_sensor==0) {
				data->enable_ps_sensor= 1;
			
				apds990x_set_enable(client,0); /* Power Off */
				apds990x_set_atime(client, 0xdb);  /* 100.64ms */ //old : 0xdb 
				apds990x_set_ptime(client, 0xff); /* 2.72ms */
				apds990x_set_ppcount(client, data->ppcount); /* 7-pulse */ // old:8
				apds990x_set_control(client, 0x20); /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */
				//apds990x_set_pilt(client, 1023);	// to force first Near-to-Far interrupt
				//apds990x_set_piht(client, 0);
				//apds990x_set_piht(client, APDS990x_PS_DETECTION_THRESHOLD); //minhyup.park change

				//data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD;
				//data->ps_hysteresis_threshold = APDS990x_PS_HSYTERESIS_THRESHOLD;
				apds990x_Set_PS_Threshold_Adding_Cross_talk(client,data->cross_talk);
				data->ps_detection = 1;			// we are forcing Near-to-Far interrupt, so this is defaulted to 1
				//apds990x_set_ailt( client, 0);
				//apds990x_set_aiht( client, (95*(1024*(256-data->atime)))/100);
				//apds990x_set_pers(client, 0x33);  // 5 consecutive Interrupt persistence old : /* 3 persistence */
				apds990x_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
												// 16-Feb-2012 KK
												// Force both ALS PS interrupt every ALS PS conversion cycle
												// instead of comparing threshold value
                 //fixed abnormal proximity near issue at daylight 
				apds990x_set_atime(client, data->als_atime);  /* previous als poll delay */
#ifdef NO_FIRST_INTERRUPT_BUGFIX
				apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
				apds990x_set_aiht(client, 0);		// in order get current ALS reading
#endif					
				apds990x_set_enable(client, 0x3F);	 /* enable PS and ALS interrupt and WAIT */

			}

		} 
		else {
			printk("\n ===>  apds990x_store_enable_ps_sensor(turn off) === \n\n");
			//turn off p sensor - kk 25 Apr 2011 we can't turn off the entire sensor, the light sensor may be needed by HAL
			//printk("[prox] turn off p sensor: [val : %x] \n",val);
			data->enable_ps_sensor = 0;

			if (data->enable_als_sensor) {
				// reconfigute light sensor setting			
				apds990x_set_enable(client,0); /* Power Off */
				apds990x_set_atime(client, data->als_atime);  /* previous als poll delay */
				//apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
				//apds990x_set_aiht(client, 0);		// in order get current ALS reading
				apds990x_set_control(client, 0x20); /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */
				//apds990x_set_pers(client, 0x33);  // 5 consecutive Interrupt persistence old : /* 3 persistence */
				apds990x_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
													// 16-Feb-2012 KK
													// Force both PS and ALS interrupt every PS ALS conversion cycle
													// instead of comparing threshold value
				apds990x_set_enable(client, 0x1B);	 /* only enable light sensor and WAIT */
			}
			else {
				apds990x_set_enable(client, 0);
			}
		}
	}
	return count;
}

static DEVICE_ATTR(enable_ps_sensor, 0660,
				   apds990x_show_enable_ps_sensor, apds990x_store_enable_ps_sensor);

static ssize_t apds990x_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds990x_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	
	//printk("[SENSOR-APDS]%s: enable als sensor ( %ld)\n", __func__, val);
	/*
	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}*/
	if(val & STORE_INTERRUPT_SELECT_ALS){
		if(val & 1) {
			//turn on light  sensor
			if (data->enable_als_sensor==0) {
				data->enable_als_sensor = 1;
			
				apds990x_set_enable(client,0); /* Power Off */
			
				apds990x_set_atime(client, data->als_atime);  
#ifdef NO_FIRST_INTERRUPT_BUGFIX
				apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
				apds990x_set_aiht(client, 0);		// in order get current ALS reading
#endif			
				apds990x_set_control(client, 0x20); /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */
				//apds990x_set_pers(client, 0x33);  // 5 consecutive Interrupt persistence old : /* 3 persistence */
				apds990x_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
												// 16-Feb-2012 KK
												// Force both ALS PS interrupt every ALS PS conversion cycle
												// instead of comparing threshold value	
			
				if (data->enable_ps_sensor) {
					//apds990x_set_ailt( client, 0);
					//apds990x_set_aiht( client, (95*(1024*(256-data->atime)))/100);

					apds990x_set_ptime(client, 0xff); /* 2.72ms */
					apds990x_set_ppcount(client, data->ppcount); /* 7-pulse */  // old:8
					apds990x_set_enable(client, 0x3F);	 /* if prox sensor was activated previously */
				}
				else {
					apds990x_set_enable(client, 0x1B);	 /* only enable light sensor and WAIT*/
				}	
			}
		}
		else {
			//turn off light sensor
			// what if the p sensor is active?
			data->enable_als_sensor = 0;
			if (data->enable_ps_sensor) {
				apds990x_set_enable(client,0); /* Power Off */
				apds990x_set_atime(client, 0xdb);  /* 100.64ms */ //old : 0xdb 
				apds990x_set_ptime(client, 0xff); /* 2.72ms */
				apds990x_set_ppcount(client, data->ppcount); /* 8-pulse */  // old:8
				apds990x_set_control(client, 0x20); /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */
							
				//apds990x_set_ailt( client, 0);
				//apds990x_set_aiht( client, (95*(1024*(256-data->atime)))/100);
#ifdef NO_FIRST_INTERRUPT_BUGFIX
				apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
				apds990x_set_aiht(client, 0);		// in order get current ALS reading
#endif				
//				apds990x_set_pers(client, 0x33);  // 5 consecutive Interrupt persistence old : /* 3 persistence */
				//apds990x_set_enable(client, 0x3F);	 /* enable als + prox sensor with interrupt */			
				apds990x_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
												// 16-Feb-2012 KK
												// Force both ALS PS interrupt every ALS PS conversion cycle
												// instead of comparing threshold value	
			
				apds990x_set_enable(client, 0x25);	 /* only prox sensor with interrupt */			
			}
			else {
				apds990x_set_enable(client, 0);
			}
		}
	}
	return count;
}

static DEVICE_ATTR(enable_als_sensor, 0660,
				   apds990x_show_enable_als_sensor, apds990x_store_enable_als_sensor);

static ssize_t apds990x_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->als_poll_delay*1000);	// return in micro-second
}

static ssize_t apds990x_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;
	int poll_delay=0;

	if (val<5000)
		val = 5000;	// minimum 5ms

	data->als_poll_delay = val/1000;	// convert us => ms
		
	poll_delay = 256 - (val/2720);	// the minimum is 2.72ms = 2720 us, maximum is 696.32ms
	if (poll_delay >= 256)
		data->als_atime = 255;
	else if (poll_delay < 0)
		data->als_atime = 0;
	else
		data->als_atime = poll_delay;
	
	ret = apds990x_set_atime(client, data->als_atime);
	
	if (ret < 0)
		return ret;
	
	return count;
}

static DEVICE_ATTR(als_poll_delay, 0660,
				   apds990x_show_als_poll_delay, apds990x_store_als_poll_delay);

static ssize_t apds990x_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
    
	return sprintf(buf, "%x\n", data->enable);
}

static ssize_t apds990x_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret = 0;

	ret = apds990x_set_enable(client, val);

	if (ret < 0)
		return ret;

	return count;
}
static DEVICE_ATTR(enable_all, 0660, apds990x_show_enable, apds990x_store_enable);


static ssize_t apds990x_show_proxidata(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n",apds990x_proxidata);
}

static ssize_t apds990x_store_proxidata(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)                                                                                  
{
        struct i2c_client *client = to_i2c_client(dev);
        struct apds990x_data *data = i2c_get_clientdata(client);
        unsigned long val = simple_strtoul(buf, NULL, 10);

	input_report_abs(data->input_dev_ps, ABS_DISTANCE, val);        /* NEAR-to-FAR detection. 3: Near, 10: FAR */	
	input_sync(data->input_dev_ps);

	return count;
}
                 

static DEVICE_ATTR(proxidata, 0660, apds990x_show_proxidata, apds990x_store_proxidata);

static ssize_t apds990x_show_luxdata(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n",apds990x_luxdata);
}

static ssize_t apds990x_store_luxdata(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)											 
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

        input_report_abs(data->input_dev_als, ABS_MISC, val); // report the lux level
        input_sync(data->input_dev_als);


	return count;
}

 
static DEVICE_ATTR(luxdata, 0660, apds990x_show_luxdata, apds990x_store_luxdata);

static ssize_t apds990x_show_pdata(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->ps_data);
}

static DEVICE_ATTR(ps_data, 0660, apds990x_show_pdata, NULL);

static ssize_t apds990x_show_is_white(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->is_white);
}

static DEVICE_ATTR(is_white, 0660, apds990x_show_is_white, NULL);

static ssize_t apds990x_show_ps_cal_result(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds990x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_cal_result);
}

static DEVICE_ATTR(ps_cal_result, 0660, apds990x_show_ps_cal_result, NULL);

                 
static struct attribute *apds990x_attributes[] = {

    &dev_attr_ps_cal_result.attr,
	&dev_attr_is_white.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_default_calibration.attr,
	&dev_attr_status.attr,
	&dev_attr_atime.attr,
	&dev_attr_ptime.attr,
	&dev_attr_wtime.attr,
	&dev_attr_pers.attr,
	&dev_attr_ppcount.attr,
	&dev_attr_control.attr,
	&dev_attr_ps_threshold.attr,
	&dev_attr_ps_hysteresis.attr,
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_enable_all.attr,
	&dev_attr_proxidata.attr,
	&dev_attr_luxdata.attr,
	NULL
};

static const struct attribute_group apds990x_attr_group = {
	.attrs = apds990x_attributes,
};

/*
 * Initialization function
 */

static int apds990x_init_client(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int err;
	int id;

        err = apds990x_set_enable(client, 0);

	if (err < 0) {
		printk("[SENSOR-APDS] %s failed(%d) !!!!!\n", __func__, err);
		return err;
	}	
	
	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_ID_REG);
	if (id == 0x20) {
		printk("[SENSOR-APDS]APDS-9901\n");
	}
	else if (id == 0x29) {
		printk("[SENSOR-APDS]APDS-990x\n");
	}
	else {
		printk("[SENSOR-APDS]Neither APDS-9901 nor APDS-9901\n");
		return -EIO;
	}

	apds990x_set_atime(client, data->als_atime);	// 100.64ms ALS integration time
	apds990x_set_ptime(client, 0xFF);	// 2.72ms Prox integration time
	apds990x_set_wtime(client, 0xFF);	// 2.72ms Wait time

	apds990x_set_ppcount(client, data->ppcount);	
	apds990x_set_config(client, 0);		// no long wait
	apds990x_set_control(client, 0x20);	// 100mA, IR-diode, 1X PGAIN, 1X AGAIN

//	apds990x_set_pilt(client, 1023);	// to force first Near-to-Far interrupt
//	apds990x_set_piht(client, 0);
//	apds990x_set_piht(client, APDS990x_PS_DETECTION_THRESHOLD);		//minhyup.park change

//	data->ps_threshold = APDS990x_PS_DETECTION_THRESHOLD;
//	data->ps_hysteresis_threshold = APDS990x_PS_HSYTERESIS_THRESHOLD;
	apds990x_Set_PS_Threshold_Adding_Cross_talk(client,data->cross_talk);
	
	data->ps_detection = 1;			// we are forcing Near-to-Far interrupt, so this is defaulted to 1

#ifdef NO_FIRST_INTERRUPT_BUGFIX
	apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
	apds990x_set_aiht(client, 0);		// in order get current ALS reading
#endif
/*
	apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
	apds990x_set_aiht(client, 0);		// in order get current ALS reading
*/
//	apds990x_set_pers(client, 0x33);  // 5 consecutive Interrupt persistence old : /* 3 persistence */
	apds990x_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
										// 16-Feb-2012 KK
										// Force both ALS PS interrupt every ALS PS conversion cycle
										// instead of comparing threshold value

	// sensor is in disabled mode but all the configurations are preset

	return 0;
}


/*
 * Initialization function without ppcount
 */

static int apds990x_init_client_for_cal(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);
	int err;
	int id;

	err = apds990x_set_enable(client, 0);

	if (err < 0)
		return err;
	
	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_ID_REG);
	if (id == 0x20) {
		printk("[SENSOR-APDS]APDS-9901\n");
	}
	else if (id == 0x29) {
		printk("[SENSOR-APDS]APDS-990x\n");
	}
	else {
		printk("[SENSOR-APDS]Neither APDS-9901 nor APDS-9901\n");
		return -EIO;
	}

	apds990x_set_atime(client, data->als_atime);	// 100.64ms ALS integration time
	apds990x_set_ptime(client, 0xFF);	// 2.72ms Prox integration time
	apds990x_set_wtime(client, 0xFF);	// 2.72ms Wait time

	apds990x_set_config(client, 0);		// no long wait
	apds990x_set_control(client, 0x20);	// 100mA, IR-diode, 1X PGAIN, 1X AGAIN

	apds990x_Set_PS_Threshold_Adding_Cross_talk(client,data->cross_talk);
	
	data->ps_detection = 1;			// we are forcing Near-to-Far interrupt, so this is defaulted to 1

#ifdef NO_FIRST_INTERRUPT_BUGFIX
	apds990x_set_ailt(client, 0xFFFF);	// to force first ALS interrupt
	apds990x_set_aiht(client, 0);		// in order get current ALS reading
#endif
	apds990x_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
										// 16-Feb-2012 KK
										// Force both ALS PS interrupt every ALS PS conversion cycle
										// instead of comparing threshold value

	// sensor is in disabled mode but all the configurations are preset

	return 0;
}


static int apds990x_reboot_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
    switch (event) {
    case SYS_RESTART:
    case SYS_HALT:
    case SYS_POWER_OFF:
		{	
			struct apds990x_data *apds990x_mdata = i2c_get_clientdata(this_client);
			apds990x_mdata->pdata->power_off(1);

		}
	    return NOTIFY_OK;
    }
    return NOTIFY_DONE;
}


static struct notifier_block apds990x_reboot_nb = {
	.notifier_call = apds990x_reboot_notify,
};


/*
 * I2C init/probing/exit functions
 */
static int __devinit apds990x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds990x_data *data;
	struct apds990x_proximity_platform_data *pdata;
	
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "[SENSOR-APDS]%s: not supported for I2C SMBUS\n",
				__func__);
		return -EIO;
	}
	
	this_client = client;

	/* device data setting */
	pdata = client->dev.platform_data;
	if(pdata == NULL) 
	{
		dev_err(&client->dev,
				"[SENSOR-APDS]%s: failed to read apds990x platform data\n",
				__func__);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct apds990x_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "[SENSOR-APDS]%s: failed to allocate memory\n",
				__func__);
		return -ENOMEM;
	}
	data->client = client;
	data->pdata = client->dev.platform_data;

	APDS990x_IRQ = data->pdata->irq;
        
	i2c_set_clientdata(client, data);

	pdata->power_on(1);
	mdelay(5); // minimum 4.5msec over

	data->enable = 0;	/* default mode is standard */
	data->ps_threshold = 0;
	data->ps_hysteresis_threshold = 0;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	data->als_poll_delay = 100;	// default to 100ms
	data->als_atime	= 0xdb;		// work in conjuction with als_poll_delay  //old : 0xdb 

	data->is_white = 0;			// white after Rev.D

        printk("[SENSOR-APDS]%s : get cross-talk = %d , ppcount_val = %d \n",__func__,cross_talk_val, ppcount_val);
        
        if(cross_talk_val > 0 && cross_talk_val < 1000)
                data->cross_talk = cross_talk_val;
        else
        	data->cross_talk = PS_DEFAULT_CROSSTALK_VAL; 	// default value. : Get the cross-talk value from the memory. This value is saved during the cross-talk calibration

		
        if(ppcount_val > 0 && ppcount_val < 9)
			data->ppcount = ppcount_val;
		else
			data->ppcount = PS_DEFAULT_PPCOUNT;	// default value. : white - 4
		
        data->irq_wake	= 0;	                // irq_wake
        
	printk("[SENSOR-APDS] apds990x_probe enable(%d) = %d\n", data->enable, APDS990x_IRQ);

	//mutex_init(&data->update_lock);

	irq_set_irq_type(APDS990x_IRQ, IRQ_TYPE_EDGE_FALLING);

#if 0
	if (gpio_is_valid(APDS990x_ATTB))
	{
		if(gpio_request( APDS990x_ATTB, "APDS990x_ATTB")) {
			printk(KERN_ERR "failed to request GPH3 for APDS990x_ATTB..\n");
			return -1;
		}
		gpio_direction_input(APDS990x_ATTB);
		s3c_gpio_setpull(APDS990x_ATTB, S3C_GPIO_PULL_NONE);
		
		gpio_free(APDS990x_ATTB);
	}
#endif
	err = request_irq(APDS990x_IRQ, apds990x_interrupt,
			IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING,
			APDS990x_DRV_NAME, (void *)client);
	if (err < 0) {
		dev_err(&client->dev,
				"[SENSOR-APDS]%s: Could not allocate APDS990x_INT !\n",
				__func__);
		goto exit_kfree;
	}

	INIT_DELAYED_WORK(&data->dwork, apds990x_work_handler);

	dev_info(&client->dev, "[SENSOR-APDS]%s: interrupt is hooked\n", __func__);

	/* Initialize the APDS990x chip */
	err = apds990x_init_client(client);
	if (err < 0) {
		dev_err(&client->dev, "[SENSOR-APDS]%s: apds990x_init_client() failed\n",
				__func__);
		goto exit_free_irq;
	}

	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"[SENSOR-APDS]%s: Failed to allocate input device als\n",
				__func__);
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"[SENSOR-APDS]%s: Failed to allocate input device ps\n",
				__func__);
		goto exit_free_dev_als;
	}
	
	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_als->name = "light";
	data->input_dev_ps->name = "proximity";

	err = input_register_device(data->input_dev_als);
	if (err < 0) {
		dev_err(&client->dev,
				"[SENSOR-APDS]%s: Unable to register input device als: %s\n",
			       __func__, data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(data->input_dev_ps);
	if (err < 0) {
		dev_err(&client->dev,
				"[SENSOR-APDS]%s: Unable to register input device ps: %s\n",
				__func__, data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds990x_attr_group);
	if (err < 0) {
		dev_err(&client->dev, "[SENSOR-APDS]%s: sysfs_create_group() failed\n",
				__func__);
		goto exit_unregister_dev_ps;
	}

	register_reboot_notifier(&apds990x_reboot_nb);

	//pdata->power_off(1); // need to move apds990x_suspend()
	printk("[SENSOR-APDS]%s prior data->enable_als_sensor = %d LINE[%d]\n",__func__,data->enable_als_sensor,__LINE__);
	printk("[SENSOR-APDS]%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
	
	
	printk("-----------------------------\n");
	printk("apds990x driver initialized!!\n");
	printk("-----------------------------\n");

	return 0;

exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);	
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_dev_als:
	input_free_device(data->input_dev_als);
exit_free_irq:
	free_irq(APDS990x_IRQ, client);	
exit_kfree:
	i2c_set_clientdata(client, NULL);
	kfree(data);
	data = NULL;
	return err;
}

static int __devexit apds990x_remove(struct i2c_client *client)
{
	struct apds990x_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);

	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_ps);
	
	input_free_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);

	free_irq(APDS990x_IRQ, client);

	/* Power down the device */
	apds990x_set_enable(client, 0);

	i2c_set_clientdata(client, NULL);

	kfree(data);
	data = NULL;

	return 0;
}

#if defined(CONFIG_PM)
//static int apds990x_suspend(struct device *device)
static int apds990x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds990x_data *apds990x_mdata = i2c_get_clientdata(client);

#if defined(APDS900_SENSOR_DEBUG)
	//if(DEBUG_DEV_DEBOUNCE & debug_mask)  
        printk("[s]### apds990x_suspend1, apds990x_mdata->enable=%d, LDO: %d\n", apds990x_mdata->enable, gpio_get_value(TEGRA_GPIO_PD4));
#endif

	//                                                                            
	// race condition bw interrupt handler adding work and this suspend routine cancelling work.
	disable_irq(APDS990x_IRQ);

        apds990x_enable_backup = apds990x_mdata->enable;	

	/*                                        
                                                 
                                   
  */
	flush_delayed_work_sync(&apds990x_mdata->dwork);

	if(apds990x_mdata->enable == 0x3F || apds990x_mdata->enable == 0x25)
	{
                if(!enable_irq_wake(APDS990x_IRQ))
		{
#if defined(APDS900_SENSOR_DEBUG)
		if(DEBUG_DEV_DEBOUNCE & debug_mask)    
                        printk("### apds990x_suspend1 -- before setting irq_wake [%d] \n",apds990x_mdata->irq_wake);
#endif                        
                        apds990x_mdata->irq_wake = 1;
                }
                
		apds990x_set_enable(client, 0x25);
	}
	else 
	{
		apds990x_set_enable(client, 0);
		apds990x_mdata->pdata->power_off(1);          //                                         
	} 

	return 0;
}

//static int apds990x_resume(struct device *device)
static int apds990x_resume(struct i2c_client *client)
{
        struct apds990x_data *apds990x_mdata = i2c_get_clientdata(client);

#if defined(APDS900_SENSOR_DEBUG)
	//if(DEBUG_DEV_DEBOUNCE & debug_mask) 
        printk("[s]### apds990x_resume, LDO: %d , apds990x_mdata->enable=%d, apds990x_enable_backup=%d\n", gpio_get_value(TEGRA_GPIO_PD4),apds990x_mdata->enable, apds990x_enable_backup);
#endif

	if(apds990x_enable_backup == 0x3F || apds990x_enable_backup == 0x25) 
        {
#if defined(APDS900_SENSOR_DEBUG)
		if(DEBUG_DEV_DEBOUNCE & debug_mask)        	
                	printk("### apds990x_resume1 -- irq_wake [%d] \n", apds990x_mdata->irq_wake);
#endif                
                if(apds990x_mdata->irq_wake) 
                {
                         disable_irq_wake(APDS990x_IRQ);
                         apds990x_mdata->irq_wake = 0;
                }  
        }
        else 
        {
        	apds990x_mdata->pdata->power_on(1);
			mdelay(5);
        }

		apds990x_set_enable(client, apds990x_enable_backup);

	/*                                        
                                           
                         
  */
	//schedule_delayed_work(&apds990x_mdata->dwork, 0);
	apds990x_reschedule_work(apds990x_mdata, 0);

       //printk("[e] ### apds990x_resume, apds990x_mdata->enable=%d, apds990x_enable_backup=%d\n", papds990x_mdata->enable, apds990x_enable_backup);

	//                                                                                                       
	// race condition bw interrupt handler adding work and the suspend routine cancelling work.
	enable_irq(APDS990x_IRQ);

        return 0;
}

#else

#define apds990x_suspend	NULL
#define apds990x_resume		NULL

#endif /* CONFIG_PM */


static const struct i2c_device_id apds990x_id[] = {
	{ APDS990x_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds990x_id);


#if 0 //defined(CONFIG_PM)
static struct dev_pm_ops apds990x_pm_ops = {
	.suspend = apds990x_suspend,
	.resume = apds990x_resume,
};
#endif

static struct i2c_driver apds990x_driver = {
	.driver = {
		.name	= APDS990x_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= apds990x_probe,
	.remove	= __devexit_p(apds990x_remove),
	.suspend = apds990x_suspend,	/* optional */
	.resume = apds990x_resume,	/* optional */
	.id_table = apds990x_id,
};

static int __init apds990x_init(void)
{
	if(x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_C)
	{
		PS_DEFAULT_PPCOUNT = APDS990x_PS_DEFAULT_PPCOUNT_REV_D;
		PS_DEFAULT_CROSSTALK_VAL = APDS990x_PS_DEFAULT_CROSSTALK_VAL_REV_D;

		GA = APDS990x_WHITE_ALS_DEFAULT_GA;		/* 2.485 --- 0.48 without glass window ..old: 271*/
		COE_B = APDS990x_WHITE_ALS_DEFAULT_COE_B;		/* 1.973 --- 2.23 without glass window ..old: 231*/
		COE_C = APDS990x_WHITE_ALS_DEFAULT_COE_C;		/* 0.694 --- 0.70 without glass window .. old: 34*/
		COE_D = APDS990x_WHITE_ALS_DEFAULT_COE_D;		/* 1.322 --- 1.42 without glass window .. old: 65*/
	}
	else
	{
		PS_DEFAULT_PPCOUNT = APDS990x_PS_DEFAULT_PPCOUNT_REV_B;
		PS_DEFAULT_CROSSTALK_VAL = APDS990x_PS_DEFAULT_CROSSTALK_VAL_REV_B;	

		GA = APDS990x_BLACK_ALS_DEFAULT_GA; 		/* 2.485 --- 0.48 without glass window ..old: 271*/
		COE_B = APDS990x_BLACK_ALS_DEFAULT_COE_B;		/* 1.973 --- 2.23 without glass window ..old: 231*/
		COE_C = APDS990x_BLACK_ALS_DEFAULT_COE_C;		/* 0.694 --- 0.70 without glass window .. old: 34*/
		COE_D = APDS990x_BLACK_ALS_DEFAULT_COE_D;		/* 1.322 --- 1.42 without glass window .. old: 65*/		
	}

	return i2c_add_driver(&apds990x_driver);
}

static void __exit apds990x_exit(void)
{
	i2c_del_driver(&apds990x_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS990x ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds990x_init);
module_exit(apds990x_exit);
