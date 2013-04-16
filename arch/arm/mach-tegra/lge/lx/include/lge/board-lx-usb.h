/*
 * The header file for LX USB
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __MACH_TEGRA_LX_USB_H
#define __MACH_TEGRA_LX_USB_H

#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>
#include <linux/fsl_devices.h>
#include <mach/usb_phy.h>

#if 1
#define USB_MANUFACTURER_NAME	"LGE"
#define USB_DEVICE_NAME         "LX"

#define USB_PRODUCT_ID_DEFAULT	0x61fc
#define USB_PRODUCT_ID_RNDIS	0x61fc
#define USB_VENDOR_ID			0x1004
#endif//


extern struct platform_device tegra_usb_fsg_device;
extern struct platform_device androidusb_device;
extern struct android_usb_platform_data andusb_plat;

extern void lx_usb_init(void);

#endif /* __MACH_TEGRA_LX_USB_H */
