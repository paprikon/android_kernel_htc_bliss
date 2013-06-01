/* linux/include/asm-arm/arch-msm/htc_onewire.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_ONEWIRE_H
#define __ASM_ARCH_MSM_ONEWIRE_H

#include <linux/types.h>

#define OWE_ERR(fmt, args...) \
	printk(KERN_ERR "[OWE:ERR] " fmt, ## args)
#define OWE_WARN(fmt, args...) \
	printk(KERN_WARNING "[OWE] " fmt, ## args)
#define OWE_I(fmt, args...) \
	printk(KERN_INFO "[OWE] " fmt, ## args)
#define OWE_D(fmt, args...) \
	printk(KERN_DEBUG "[OWE] " fmt, ## args)

#define ONEWIRE_NAME "htc_onewire"

struct msm_onewire_platform_data {
	int gpio_one_wire;
	void (*onewire_gpio_config)(void);
};

void onewire_detect_start(int status);

#endif
