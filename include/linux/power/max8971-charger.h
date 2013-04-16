/*
 * MAX8971 Charger Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MAX8971_CHARGER_H
#define __LINUX_MAX8971_CHARGER_H


#define FCHG_CURRENT(x) ((x-250)/50+5)  // 0 and from 250mA to 1550mA in 50mA steps.

enum {
    MAX8971_FCHGTIME_DISABLE,
    MAX8974_FCHGTIME_4HRS,
    MAX8974_FCHGTIME_5HRS,  
    MAX8974_FCHGTIME_6HRS,  
    MAX8974_FCHGTIME_7HRS,  
    MAX8974_FCHGTIME_8HRS,  
    MAX8974_FCHGTIME_9HRS,  
    MAX8974_FCHGTIME_10HRS,  
};

enum {
    MAX8971_TOPOFFTIME_0MIN,
    MAX8971_TOPOFFTIME_10MIN,
    MAX8971_TOPOFFTIME_20MIN,
    MAX8971_TOPOFFTIME_30MIN,
    MAX8971_TOPOFFTIME_40MIN,
    MAX8971_TOPOFFTIME_50MIN,
    MAX8971_TOPOFFTIME_60MIN,
    MAX8971_TOPOFFTIME_70MIN,
};

enum {
    MAX8971_TOPOFFTSHLD_50mA,
    MAX8971_TOPOFFTSHLD_100mA,
    MAX8971_TOPOFFTSHLD_150mA,
    MAX8971_TOPOFFTSHLD_200mA,
};

enum {
    MAX8971_CHGCV_4P20V,
    MAX8971_CHGCV_4P10V,
    MAX8971_CHGCV_4P35V,
};

enum {
    MAX8971_CHGRSTRT_150mV,
    MAX8971_CHGRSTRT_100mV,
};

#define DCI_LIMIT(x) ((x<=100) ? 0 :           \
					  (x>=250 && x<=1550) ? ((x-250)/25+10) : -EINVAL)

enum {
    MAX8971_REGTEMP_105degree,
    MAX8971_REGTEMP_90degree,
    MAX8971_REGTEMP_120degree,
    MAX8971_REGTEMP_DISABLE,
};

enum {
    MAX8971_THM_CNFG_CONTINUOUS,
    MAX8971_THM_CNFG_NOT_MONITOR,
};

enum {
    MAX8971_SAFETYREG_REGION1,
    MAX8971_SAFETYREG_REGION2,
};

#define MAX8971_CHGPROT_LOCKED      0x00
#define MAX8971_CHGPROT_UNLOCKED    0x03

/* IRQ definitions */
enum {
	MAX8971_IRQ_PWRUP_OK =0,
	MAX8971_IRQ_THM,
	MAX8971_IRQ_BAT,
	MAX8971_IRQ_CHG,
	MAX8971_IRQ_DCUVP,
	MAX8971_IRQ_DCOVP,
	MAX8971_IRQ_DCI,
	MAX8971_IRQ_DCV,      
	MAX8971_NR_IRQS,
};

struct max8971_platform_data {
//                                                                          
//                    
#if defined(CONFIG_MACH_VU10) || defined(CONFIG_MACH_X3)
	u8	chgcc_400;
#endif
//                                                                          
	u8	chgcc_usb500;       // Fast Charge Current
	u8	chgcc_ta;           // Fast Charge Current
	u8	chgcc_factory;           // Fast Charge Current
	u8	chgcc_mhl400;       //                                                                              
	u8	chgcc_soc700;
	
	u8	fchgtime;           // Fast Charge Time

	u8	chgrstrt;           // Fast Charge Restart Threshold
	u8	dcilmt_usb500;      // Input Current Limit Selection	
	u8	dcilmt_ta;          // Input Current Limit Selection
	u8	dcilmt_factory;          // Input Current Limit Selection
	u8	dcilmt_mhl400;      //                                                                                        
	u8	dcilmt_soc700;

	u8	topofftime;         // Top Off Timer Setting 
	u8	topofftshld;        // Done Current Threshold
	u8	chgcv;              // Charger Termination Voltage
	u8	ifst2p8_usb500;              // Scales Maximum Fast Charge Current to 2.8A.
	u8	ifst2p8_ta;              // Scales Maximum Fast Charge Current to 2.8A.
	u8	ifst2p8_factory;              // Scales Maximum Fast Charge Current to 2.8A.
	u8	ifst2p8_mhl400;              //                                                                                                      

	u8	regtemp;            // Die temperature thermal regulation loop setpoint
	u8	safetyreg;          // JEITA Safety region selection
	u8	thm_config;         // Thermal monitor configuration

	u8	int_mask;           // CHGINT_MASK
	u32	irqb_pgio;		//IRQB GPIO
	u32  chg_proctection; //CHG PROTECTION

	u8	m_input_vol;	//monitor input voltage 
	u8	suspend_usb;
	int 	(*gpio_init)(void);
};

int max8971_stop_charging(void);
int max8971_start_charging(unsigned mA);
#endif
