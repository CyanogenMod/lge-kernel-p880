/*
 * arch/arm/mach-tegra/board-lx-baseband.c
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/wakelock.h>
#include <linux/platform_data/tegra_usb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pinmux.h>
#include <mach/usb_phy.h>
#include <linux/memblock.h>
#include <linux/slab.h>

#include <lge/board-lx.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>
#include <mach-tegra/baseband-xmm-power.h>
//                                         
#if defined(CONFIG_MACH_PEGASUS)
#include <mach/tegra-bb-power.h> //dalyong.cha
#endif	
//                                         

#define XMM6260_GPIO_BB_RST			MODEM_RESET
#define XMM6260_GPIO_BB_ON			MODEM_PWR_ON
#define XMM6260_GPIO_IPC_BB_WAKE		AP2CP_ACK1_SLAVE_WAKEUP
#define XMM6260_GPIO_IPC_AP_WAKE		CP2AP_ACK2_HOST_WAKEUP
#define XMM6260_GPIO_IPC_HSIC_ACTIVE		CP2AP_ACK1_HOST_ACTIVE
#define XMM6260_GPIO_IPC_HSIC_SUS_REQ		AP2CP_ACK2_SUSPEND_REQ

static int lx_usb_hsic_postsuspend(void);
static int lx_usb_hsic_preresume(void);
static int lx_usb_hsic_phy_ready(void);
static int lx_usb_hsic_phy_off(void);

//#define ebs 

struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,                               
	.modem = {                                                       
		.xmm = {                                             
			.bb_rst = XMM6260_GPIO_BB_RST,                
			.bb_on = XMM6260_GPIO_BB_ON,                    
			.ipc_bb_wake = XMM6260_GPIO_IPC_BB_WAKE, 
			.ipc_ap_wake = XMM6260_GPIO_IPC_AP_WAKE,          
			.ipc_hsic_active = XMM6260_GPIO_IPC_HSIC_ACTIVE,     
			.ipc_hsic_sus_req = XMM6260_GPIO_IPC_HSIC_SUS_REQ,    
			.hsic_device = &tegra_ehci2_device,                   
		},                                                       
	},                                                               
};                                                                      

																		
static struct platform_device tegra_baseband_power_device = {           
	.name = "baseband_xmm_power",                                        
	.id = -1,                                                        
	.dev = {                                                         
		.platform_data = &tegra_baseband_power_data,             
	},                                                               
};                                                                      
                                                                        
static struct platform_device tegra_baseband_power2_device = {          
	.name = "baseband_xmm_power2",                                       
	.id = -1,                                                        
	.dev = {                                                         
		.platform_data = &tegra_baseband_power_data,             
	},                                                               
};                                                 

//                                                                                 
#if defined(CONFIG_TEGRA_BB_MODEM4) 
static struct tegra_usb_phy_platform_ops hsic_rmc_plat_ops = {
	.post_suspend = lx_usb_hsic_postsuspend,
	.pre_resume = lx_usb_hsic_preresume,
	.port_power = lx_usb_hsic_phy_ready, // How to handle port_power api?  //ebs_wondering
	//.usb_phy_ready = lx_usb_hsic_phy_ready,
	.post_phy_off = lx_usb_hsic_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_rmc_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
	},
	.u_cfg.hsic = {
		.sync_start_delay = 9,
		.idle_wait_delay = 17,
		.term_range_adj = 0,
		.elastic_underrun_limit = 16,
		.elastic_overrun_limit = 16,
	},
	.ops = &hsic_rmc_plat_ops,
};
#else

static struct tegra_uhsic_config uhsic_phy_config = {
	.enable_gpio = (-1), //TPS80031_GPIO_SMPS3,
	.reset_gpio = (-1),
#if defined(CONFIG_MACH_LX_FLASHLESS)	
	.bb_data = &tegra_baseband_power_data,
#endif	
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
	.postsuspend = lx_usb_hsic_postsuspend,
	.preresume = lx_usb_hsic_preresume,
	.usb_phy_ready = lx_usb_hsic_phy_ready,
	.post_phy_off = lx_usb_hsic_phy_off,
};

static struct tegra_ehci_platform_data tegra_ehci_uhsic_pdata = {
	.phy_type = TEGRA_USB_PHY_TYPE_HSIC,
	.phy_config = &uhsic_phy_config,
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 0,
};
#endif 
//                                                                                

/// ebs 
static int lx_usb_hsic_postsuspend(void)
{
	printk("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
	return 0;
}


static int lx_usb_hsic_preresume(void)
{
	printk("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
	return 0;
}

//                                             
static int lx_usb_hsic_phy_ready(void)
{
	printk("%s\n", __func__);
#if defined(CONFIG_TEGRA_BB_XMM_POWER)
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#elif defined(CONFIG_TEGRA_BB_MODEM4)
#endif
	return 0;
}
//                                             

static int lx_usb_hsic_phy_off(void)
{
	printk("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L3);
#endif
	return 0;
}
//                                                     
struct platform_device *tegra_usb_hsic_host_register(void)
{
	struct platform_device *pdev;

	int val;

//	dump_stack();
	pdev = platform_device_alloc(tegra_ehci2_device.name,
		tegra_ehci2_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci2_device.resource,
		tegra_ehci2_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci2_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci2_device.dev.coherent_dma_mask;

	val = platform_device_add_data(pdev, &tegra_ehci2_hsic_rmc_pdata,
			sizeof(struct tegra_usb_platform_data));
	if (val)
		goto error;

	val = platform_device_add(pdev);
	if (val)
		goto error;

	return pdev;

error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}
//                                                     

void tegra_usb_hsic_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

//                                                        
#ifdef temp_block
static int __init tegra_uhsic_init(void)
{
	tegra_ehci2_device.dev.platform_data = &tegra_ehci_uhsic_pdata;

	
	/* enable baseband gpio(s) */										   
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.bb_rst);  
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.bb_on);	   
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_bb_wake); 
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_ap_wake); 
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_hsic_active);
	tegra_gpio_enable(tegra_baseband_power_data.modem.xmm.ipc_hsic_sus_req);

	tegra_baseband_power_data.hsic_register =
					&tegra_usb_hsic_host_register;
	tegra_baseband_power_data.hsic_unregister =
					&tegra_usb_hsic_host_unregister;

	platform_device_register(&tegra_baseband_power_device); 
//	platform_device_register(&tegra_baseband_power2_device);
	return 0;
}
#endif 
//                                                         

#if defined(CONFIG_MACH_PEGASUS) && defined(CONFIG_TEGRA_BB_MODEM4)

static union tegra_bb_gpio_id modem4_gpio_id = {
	.modem4 = {
		.mdm4_rst = BB_GPIO_MDM4_RST,
		.mdm4_on = BB_GPIO_MDM4_ON,
		.usb_awr = BB_GPIO_MDM4_AWR,
		.usb_cwr = BB_GPIO_MDM4_CWR,
		.mdm4_spare = BB_GPIO_MDM4_SPARE,
		.mdm4_wdi = BB_GPIO_MDM4_WDI,

	},
};

static struct tegra_bb_pdata modem4_pdata = {
	.id = &modem4_gpio_id,
	.device = &tegra_ehci2_device,
	.ehci_register = tegra_usb_hsic_host_register,
	.ehci_unregister = tegra_usb_hsic_host_unregister,
	.bb_id = TEGRA_BB_MODEM4,
	.regulator = "vddio_hsic",
};

static struct platform_device tegra_baseband_modem4_device = {
	.name = "tegra_baseband_power",
	.id = -1,
	.dev = {
		.platform_data = &modem4_pdata,
	},
};
#endif
int __init lx_baseband_init(void)
{
#if defined(CONFIG_MACH_PEGASUS) && defined(CONFIG_TEGRA_BB_MODEM4)
	tegra_ehci2_hsic_rmc_pdata.u_data.host.power_off_on_suspend = 0;
	tegra_ehci2_device.dev.platform_data= &tegra_ehci2_hsic_rmc_pdata;
	platform_device_register(&tegra_baseband_modem4_device);
#else
	int ret;
	
	printk("##@%s  \n", __func__ ) ;
	
// EBS for temp block we don't consider XMM Project.
	//tegra_uhsic_init();
// EBS for temp block we don't consider XMM Project.

#endif
	return 0;

}

