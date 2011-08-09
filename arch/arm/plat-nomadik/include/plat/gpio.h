/*
 * Structures and registers for GPIO access in the Nomadik SoC
 *
 * Copyright (C) 2008 STMicroelectronics
 *     Author: Prafulla WADASKAR <prafulla.wadaskar@st.com>
 * Copyright (C) 2009 Alessandro Rubini <rubini@unipv.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_PLAT_GPIO_H
#define __ASM_PLAT_GPIO_H

#include <asm-generic/gpio.h>

/*
 * These currently cause a function call to happen, they may be optimized
 * if needed by adding cpu-specific defines to identify blocks
 * (see mach-pxa/include/mach/gpio.h as an example using GPLR etc)
 */
#define gpio_get_value  __gpio_get_value
#define gpio_set_value  __gpio_set_value
#define gpio_cansleep   __gpio_cansleep
#define gpio_to_irq     __gpio_to_irq

#endif /* __ASM_PLAT_GPIO_H */
