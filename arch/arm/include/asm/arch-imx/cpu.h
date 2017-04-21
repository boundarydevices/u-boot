/*
 * (C) Copyright 2014 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#define MXC_CPU_MX51		0x51
#define MXC_CPU_MX53		0x53
#define MXC_CPU_MX6SL		0x60
#define MXC_CPU_MX6DL		0x61
#define MXC_CPU_MX6SX		0x62
#define MXC_CPU_MX6Q		0x63
#define MXC_CPU_MX6D		0x64
#define MXC_CPU_MX6SOLO		0x65 /* dummy ID */

#define CS0_128					0
#define CS0_64M_CS1_64M				1
#define CS0_64M_CS1_32M_CS2_32M			2
#define CS0_32M_CS1_32M_CS2_32M_CS3_32M		3

u32 get_imx_reset_cause(void);
