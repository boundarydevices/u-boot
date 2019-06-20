/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/

/* platform header */
/*
 * (C) Copyright 2010 Amlogic, Inc
 *
 * Victor Wan, victor.wan@amlogic.com,
 * 2010-03-24 @ Shanghai
 *
 */

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <asm/arch/register.h>

/* A3,CS2,M3 chip, PORT_A is OTG, work as ROM Boot port */
#ifdef __USE_PORT_B
#define PORT_REG_OFFSET   0x80000
#else
#define PORT_REG_OFFSET   0
#endif

#if (defined CONFIG_TXLX_USB)
#if ((defined CONFIG_USB_XHCI))
#define DWC_REG_BASE   0xff400000
#else
#define DWC_REG_BASE  (0xff500000 + PORT_REG_OFFSET)
#endif
#else
#if ((defined CONFIG_USB_XHCI))
#define DWC_REG_BASE   0xc9100000
#else
#define DWC_REG_BASE  (0xC9000000 + PORT_REG_OFFSET)
#endif
#endif

#define PREI_USB_PHY_A_POR      (1 << 0)
#define PREI_USB_PHY_B_POR      (1 << 1)
#define PREI_USB_PHY_CLK_SEL    (7 << 5)
#define PREI_USB_PHY_CLK_GATE 	(1 << 8)
#define PREI_USB_PHY_B_AHB_RSET     (1 << 11)
#define PREI_USB_PHY_B_CLK_RSET     (1 << 12)
#define PREI_USB_PHY_B_PLL_RSET     (1 << 13)
#define PREI_USB_PHY_A_AHB_RSET     (1 << 17)
#define PREI_USB_PHY_A_CLK_RSET     (1 << 18)
#define PREI_USB_PHY_A_PLL_RSET     (1 << 19)
#define PREI_USB_PHY_A_DRV_VBUS     (1 << 20)
#define PREI_USB_PHY_B_DRV_VBUS			(1 << 21)
#define PREI_USB_PHY_B_CLK_DETECT   (1 << 22)
#define PREI_USB_PHY_CLK_DIV        (0x7f << 24)
#define PREI_USB_PHY_A_CLK_DETECT   (1 << 31)

#define PREI_USB_PHY_A_REG3_IDDIG_OVR	(1 << 23)
#define PREI_USB_PHY_A_REG3_IDDIG_VAL	(1 << 24)

#define PREI_USB_PHY_B_REG4_IDDIG_OVR	(1 << 23)
#define PREI_USB_PHY_B_REG4_IDDIG_VAL	(1 << 24)

#define IREG_TIMER_E_COUNT            0x2655


#define flush_cpu_cache()


#define dwc_write_reg32(x, v) 	(*(volatile uint32_t *)(unsigned long)(x + DWC_REG_BASE))=v
#define dwc_read_reg32(x) (*(volatile uint32_t*)(unsigned long)(x + DWC_REG_BASE))
#define dwc_modify_reg32(x, c, s) 	(*(volatile uint32_t *)(x + DWC_REG_BASE))=( ((dwc_read_reg32(x)) & (~c)) | (s))

#define get_unaligned(ptr)      (  ((unsigned long)ptr & 3) ? \
                                (((__u8 *)ptr)[0] | (((__u8 *)ptr)[1]<<8) | (((__u8 *)ptr)[2]<<16) | (((__u8 *)ptr)[3]<<24)) : \
                                (*(uint32_t*)ptr) )
#define get_unaligned_16(ptr)				(((__u8 *)ptr)[0] | (((__u8 *)ptr)[1]<<8))
#define get_unaligned_32(ptr)				(((__u8 *)ptr)[0] | (((__u8 *)ptr)[1]<<8) | (((__u8 *)ptr)[2]<<16) | (((__u8 *)ptr)[3]<<24))

#define EXT_CLOCK	0
#define INT_CLOCK	1

#define USB_ROM_CONN_TIMEOUT		5*1000*1000


/* Meet with spec */
#define USB_ROM_VER_MAJOR	0
#define USB_ROM_STAGE_MAJOR	0
#define USB_ROM_STAGE_MINOR	16

#ifdef CONFIG_M6
#define USB_ROM_VER_MINOR	8
#else
#define USB_ROM_VER_MINOR	7
#endif

#define PRINTF(x...)	do{}while(0)

#define ERR(x...)       printf(x)
#define DBG(x...)       PRINTF(x)
#define USB_ERR(x...)	printf("USBErr:%d", __LINE__),printf(x)
#define USB_DBG(x...)   PRINTF(x)


void f_set_usb_phy_config(void);
#ifdef CONFIG_USB_DEVICE_V2
void set_usb_phy21_tuning_fb(void);
#endif

void usb_parameter_init(int timeout);
int chip_utimer_set(int val);
int chip_watchdog(void);
#define udelay __udelay
#define wait_ms(a) udelay(a*1000);
int update_utime(void);
int get_utime(void);

#endif
