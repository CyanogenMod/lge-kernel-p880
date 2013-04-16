/*
 * arch/arm/mach-tegra/board-lx.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_LX_H
#define _MACH_TEGRA_BOARD_LX_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps80031.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/syscalls.h>

#define MISC_PARTITION				"/dev/block/platform/sdhci-tegra.3/by-name/MSC"
#define MISC_MSG_LENGTH 32
#define MISC_MSG_BASE_OFFSET 0x1000
// modify this file and /android/vendor/lge/terga/core/system/fastboot/lge_boot/inc/lge_boot_utils.h
typedef enum{
    msg_type_misc = 0,
    msg_type_webdn,
    msg_type_muic_path,
    msg_type_smpl,
    msg_type_gang,
    msg_type_chg,
    msg_type_bootcause,	
    msg_type_max_count
}misc_msg_type;
// modify this file and /android/vendor/lge/terga/core/system/fastboot/lge_boot/inc/lge_boot_utils.h
#define MISC_MSG_COUNT msg_type_max_count	// MAX number of MISC messages
#define MISC_MSG_SIZE MISC_MSG_COUNT * MISC_MSG_LENGTH


int lx_charge_init(void);
int lx_sdhci_init(void);
int lx_pinmux_init(void);
int lx_panel_init(void);
int lx_sensors_init(void);
int touch_init(void);
int lx_kbc_init(void);
int lx_emc_init(void);
int lx_regulator_init(void);
int lx_modem_init(void);
int lx_suspend_init(void);
int lx_edp_init(void);
void lx_bpc_mgmt_init(void);
int lx_sensor_input_init(void);
int get_misc_msg(misc_msg_type msg, char* misc_msg, int size);
int set_misc_msg(misc_msg_type msg, char* misc_msg, int size);


/*****************External GPIO tables ******************/
/* External peripheral gpio base. */
#define ENT_TPS80031_GPIO_BASE	   TEGRA_NR_GPIOS
#define ENT_TPS80031_GPIO_REGEN1 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN1)
#define ENT_TPS80031_GPIO_REGEN2 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN2)
#define ENT_TPS80031_GPIO_SYSEN	 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_SYSEN)
#define ENT_TPS80031_GPIO_END	(ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_NR)

/*****************External Interrupt tables ******************/
/* External peripheral irq base */
#define ENT_TPS80031_IRQ_BASE	TEGRA_NR_IRQS
#define ENT_TPS80031_IRQ_END  (ENT_TPS80031_IRQ_BASE + TPS80031_INT_NR)

/* Audio-related GPIOs */
#define TEGRA_GPIO_HP_DET	TEGRA_GPIO_PBB6		//                                            

#define TEGRA_GPIO_HP_HOOK	TEGRA_GPIO_PO5		//                                             
#define TEGRA_GPIO_EAR_MIC	TEGRA_GPIO_PX0		//                                             
//                                         
#define TEGRA_GPIO_SUB_MIC	TEGRA_GPIO_PI6
//                                         

#define BOARD_1205		(0x0C05)
#define BOARD_E1197		(0x0B61)
#define LX_FAB_A01	(0x01)
#define SKU_BATTERY_SUPPORT	(1 << 8)

#define MODEM_PWR_ON		TEGRA_GPIO_PO0
#define MODEM_RESET		TEGRA_GPIO_PV1

#define CP2AP_ACK1_HOST_ACTIVE			TEGRA_GPIO_PU5		// host active		AP2CP_ACK1
#define CP2AP_ACK2_HOST_WAKEUP			TEGRA_GPIO_PV0		// host wake-up 	CP2AP_ACK2
#define AP2CP_ACK2_SUSPEND_REQ			TEGRA_GPIO_PS6		// suspend request 	CP2AP_ACK1
#define AP2CP_ACK1_SLAVE_WAKEUP			TEGRA_GPIO_PS7		// slave wake-up 	AP2CP_ACK2

#if defined(CONFIG_MFD_MAX77663)
#define MAX77663_GPIO_BASE		TEGRA_NR_GPIOS
#define MAX77663_IRQ_BASE		TEGRA_NR_IRQS
#endif /* defined(CONFIG_MFD_MAX77663) */

#define TDIODE_OFFSET	(9000)	/* in millicelsius */

/* Battery Peak Current Management */
#define TEGRA_BPC_TRIGGER		TEGRA_GPIO_PO3
#define TEGRA_BPC_TIMEOUT		200 /* ms */
#define TEGRA_BPC_CPU_PWR_LIMIT	0 /* in mW, (0 disables) */

#define TEGRA_CUR_MON_THRESHOLD		2000
#define TEGRA_CUR_MON_RESISTOR		10
#define TEGRA_CUR_MON_MIN_CORES		2

//                                   
enum tegra_bb_type {
	TEGRA_BB_PH450 = 1,
	TEGRA_BB_XMM6260,
	TEGRA_BB_M7400,
#if defined(CONFIG_MACH_PEGASUS) && defined(CONFIG_TEGRA_BB_MODEM4)
	TEGRA_BB_MODEM4,
#endif	
};

#if defined(CONFIG_MACH_PEGASUS) && defined(CONFIG_TEGRA_BB_MODEM4)
#define BB_GPIO_MDM4_ON		TEGRA_GPIO_PX1 /* LCD_D0 */
#define BB_GPIO_MDM4_AWR	TEGRA_GPIO_PS1 /* LCD_D3 */
#define BB_GPIO_MDM4_CWR	TEGRA_GPIO_PS0
#define BB_GPIO_MDM4_WDI	TEGRA_GPIO_PS5
#define BB_GPIO_MDM4_SPARE	TEGRA_GPIO_PV1
#define BB_GPIO_MDM4_RST	TEGRA_GPIO_PX2 /* LCD_D2 */
#endif
//                                   

/* Indicate the pwm of backlight, DC pwm or external pwm3. */
#define IS_EXTERNAL_PWM		0

#endif /*_MACH_TEGRA_BOARD_X3_H */
