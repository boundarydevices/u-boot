/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
.section ".text.init", "x"

#include <linux/linkage.h>

#define L2X0_CTRL	0x100
#define L2_BASE_ADDR	0x00A02000

/*
 * Disable L2Cache because ROM will turn it on when uboot is started via usb download.
 * There are cache coherency problems if cache is on when Linux kernel
 * expects it to be off.
 */
.macro disable_l2_cache
	ldr	r1, =L2_BASE_ADDR
	mov	r0, #0x0
	str	r0, [r1, #L2X0_CTRL]
.endm /* init_l2cc */

.macro init_arm_errata
	/* ARM erratum ID #743622 */
	mrc	p15, 0, r10, c15, c0, 1		/* read diagnostic register */
	orr	r10, r10, #1 << 6		/* set bit #6 */
	/* ARM erratum ID #751472 */
	orr	r10, r10, #1 << 11		/* set bit #11 */
	mcr	p15, 0, r10, c15, c0, 1		/* write diagnostic register */
.endm

ENTRY(lowlevel_init)
	init_arm_errata
	disable_l2_cache
	mov pc, lr
ENDPROC(lowlevel_init)
