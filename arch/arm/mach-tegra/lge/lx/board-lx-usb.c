/*
 * arch/arm/mach-tegra/board-lx.c
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

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/fsl_devices.h>

#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-lx.h>
#include <lge/board-lx-usb.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

#if defined(CONFIG_MFD_MAX77663)
#include <linux/mfd/max77663-core.h>
#endif

//                                                    
#if 0
#define USB_MANUFACTURER_NAME	"NVIDIA"
#define USB_PRODUCT_ID_RNDIS	0x7103
#define USB_VENDOR_ID			0x0955
#endif//
//                                            

static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
#if 1
	.nluns		= 2,
	.vendor  = USB_MANUFACTURER_NAME,
	.product = USB_DEVICE_NAME,
#else
	.vendor = "NVIDIA",
	.product = "Tegra 3",
	.nluns = 1,
#endif//	
};

struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};

//                                                                
#if CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET
static char *usb_functions_all     [] = {
#ifdef CONFIG_USB_ANDROID_MTP
	"mtp",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
        "accessory",
#endif
	"acm",
	"serial",
	"ecm",
	"usb_mass_storage",
	"adb"
};
static char *usb_functions_PID_61a6[] = {    "usb_mass_storage",    "adb",};
#ifdef CONFIG_USB_ANDROID_MTP
static char *usb_functions_PID_61f9[] = {    "mtp",   "adb",   };
#endif
static char *usb_functions_PID_61fa[] = {    "acm",   "serial",   "usb_mass_storage", "adb",};
static char *usb_functions_PID_61fc[] = {    "acm",   "serial",   "ecm",  "usb_mass_storage" , "adb", };
static char *usb_functions_PID_61fd[] = {    "acm",   "serial",   "ecm",  "usb_mass_storage",};
static char *usb_functions_PID_61c5[] = {    "usb_mass_storage",};

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
/* CDROM storage only mode(Autorun default mode) */
static char *usb_functions_PID_91c8[] = {    "usb_cdrom_storage",};
static char *usb_functions_lge_charge_only[] =  { "charge_only",};
#endif

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = {    "accessory",};
static char *usb_functions_accessory_adb[] = {"accessory","adb",};
#endif

#else   //                                     

static char *usb_functions_mtp_ums[] = { "mtp", "usb_mass_storage" };
static char *usb_functions_adb[] = { "mtp", "adb", "usb_mass_storage" };

#ifdef CONFIG_USB_ANDROID_RNDIS
static char *usb_functions_rndis[] = { "rndis" };
static char *usb_functions_rndis_adb[] = { "rndis", "adb" };
#endif

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif
	"mtp",
	"adb",
	"usb_mass_storage"
};
#endif  //                                     

static struct android_usb_product usb_products[] = {
#if   CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET  
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		.product_id 	= 0x61fc,
		.num_functions	= ARRAY_SIZE(usb_functions_PID_61fc),
		.functions		= usb_functions_PID_61fc,
	},
	{
		.product_id = 0x61FD,
		.num_functions = ARRAY_SIZE(usb_functions_PID_61fd),
		.functions = usb_functions_PID_61fd,
	},
#endif
	{
		.product_id 	= 0x61fa,
		.num_functions	= ARRAY_SIZE(usb_functions_PID_61fa),
		.functions				= usb_functions_PID_61fa,
	},
	{
		.product_id = 0x61a6,
		.num_functions = ARRAY_SIZE(usb_functions_PID_61a6),
		.functions = usb_functions_PID_61a6,
	},
#ifdef CONFIG_USB_ANDROID_MTP
	{
		.product_id = 0x61C7,
		.num_functions = ARRAY_SIZE(usb_functions_PID_61c7),
		.functions = usb_functions_PID_61c7,
	},
	{
		.product_id = 0x61F9,
		.num_functions = ARRAY_SIZE(usb_functions_PID_61f9),
		.functions = usb_functions_PID_61f9,
	},
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	/* NOTE : USB Accessory mode has google's vid and pid */
	{
		.vendor_id		= USB_ACCESSORY_VENDOR_ID,
		.product_id 	= USB_ACCESSORY_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory),
		.functions		= usb_functions_accessory,
	},
	{
		.vendor_id		= USB_ACCESSORY_VENDOR_ID,
		.product_id 	= USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_adb),
		.functions		= usb_functions_accessory_adb,
	},
#endif
	{
		.product_id = 0x61C5,
		.num_functions = ARRAY_SIZE(usb_functions_PID_61c5),
		.functions = usb_functions_PID_61c5,
	},
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
	{
		/* FIXME: This pid is just for test */
		.product_id = 0x91C8,
		.num_functions = ARRAY_SIZE(usb_functions_PID_91c8),
		.functions = usb_functions_PID_91c8,
	},
	{
		.product_id = 0x61A6,
		.num_functions = ARRAY_SIZE(usb_functions_PID_61a6),
		.functions = usb_functions_PID_61a6,
	},
	{
		/* Charge only doesn't have no specific pid */
		.product_id = 0xFFFF,
		.num_functions = ARRAY_SIZE(usb_functions_lge_charge_only),
		.functions = usb_functions_lge_charge_only,
	},
#endif
#else   //                                     
	{
		.product_id     = 0x7102,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_ums),
		.functions      = usb_functions_mtp_ums,
	},
	{
		.product_id     = 0x7100,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory),
		.functions	= usb_functions_accessory,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_adb),
		.functions	= usb_functions_accessory_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id     = USB_PRODUCT_ID_RNDIS,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
		.functions      = usb_functions_rndis,
	},
	{
		.product_id     = USB_PRODUCT_ID_RNDIS,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
#endif
#endif  //                                     
};

/* standard android USB platform data */
struct android_usb_platform_data andusb_plat = {
	.vendor_id              = 0x1004, 
	.product_id             = USB_PRODUCT_ID_DEFAULT,
	.manufacturer_name      = USB_MANUFACTURER_NAME,
	.product_name			=  USB_DEVICE_NAME ,
	.serial_number			= NULL,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),  //                                                         
	.functions = usb_functions_all,                  //                                                         
};

struct platform_device androidusb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data  = &andusb_plat,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	.ethaddr = {0, 0, 0, 0, 0, 0},
	.vendorID = USB_VENDOR_ID,
	.vendorDescr = USB_MANUFACTURER_NAME,
};

static struct platform_device rndis_device = {
	.name   = "rndis",
	.id     = -1,
	.dev    = {
		.platform_data  = &rndis_pdata,
	},
};
#endif

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {

#if defined(CONFIG_MFD_TPS80031)
		.vbus_pmu_irq = ENT_TPS80031_IRQ_BASE +
				TPS80031_INT_VBUS_DET,
		.vbus_gpio = -1,
#elif defined(CONFIG_MFD_MAX77663)	//                                                      
		.vbus_pmu_irq = MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_ACOK_FALLING,
		.vbus_gpio = MAX77663_GPIO_BASE + MAX77663_GPIO1,
#else
		.vbus_pmu_irq = -1,
		.vbus_irq = -1,
#endif
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.builtin_host_disabled = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "usb_vbus",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};


#ifdef CONFIG_USB_OTG
static struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name,
		tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
		tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data),
		GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci1_utmi_pdata[0],
				sizeof(struct tegra_usb_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}
#endif

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
	
};

#ifdef CONFIG_USB_ANDROID_RNDIS
#define SERIAL_NUMBER_LENGTH 20
static char usb_serial_num[SERIAL_NUMBER_LENGTH];
#endif

void lx_usb_init(void)
{
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

}
