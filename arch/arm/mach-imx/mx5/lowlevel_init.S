/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *
 * (C) Copyright 2009 Freescale Semiconductor, Inc.
 */

#include <config.h>
#include <asm/arch/imx-regs.h>
#include <generated/asm-offsets.h>
#include <linux/linkage.h>

.section ".text.init", "x"

.macro init_l1cc r_zero
	mcr	p15, 0, \r_zero, c8, c7, 0	@ invalidate I + D TLBs
	mcr	p15, 0, \r_zero, c2, c0, 2	@ TTB control register

	mcr	p15, 2, \r_zero, c0, c0, 0
	mrc	p15, 1, r0, c0, c0, 0

	ldr	r1, =0x7fff
	and	r2, r1, r0, lsr #13

	ldr	r1, =0x3ff

	and	r3, r1, r0, lsr #3	@ NumWays - 1
	add	r2, r2, #1		@ NumSets

	and	r0, r0, #0x7
	add	r0, r0, #4		@ SetShift

	clz	r1, r3			@ WayShift
	add	r7, r3, #1		@ NumWays
1:	sub	r2, r2, #1		@ NumSets--
	mov	r3, r7			@ Temp = NumWays
2:	subs	r3, r3, #1		@ Temp--
	mov	r5, r3, lsl r1
	mov	r6, r2, lsl r0
	orr	r5, r5, r6
@ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
	mcr	p15, 0, r5, c7, c6, 2
	bgt	2b
	cmp	r2, #0
	bgt	1b
	dsb
	isb
.endm

.macro init_arm_erratum
	/* ARM erratum ID #468414 */
	mrc 15, 0, r1, c1, c0, 1
	orr r1, r1, #(1 << 5)    /* enable L1NEON bit */
	mcr 15, 0, r1, c1, c0, 1

	/*
	 * Workaround for arm errata #709718
	 * ARM: Load and Store operations on the shared device memory
	 * regions may not complete in program order
	 */
	/* Setup PRRR so device is always mapped to non-shared */
	mrc p15, 0, r1, c10, c2, 0
	bic r1, r1, #3 << 0x10
	mcr p15, 0, r1, c10, c2, 0
.endm

.macro 	orr_mx51_tape2_or_lower	r_dest, value, r_zero
#if defined(CONFIG_MX51)
	ldr r0, =IIM_BASE_ADDR
	ldr r0, [r0, #IIM_SREV]
	cmp r0, #0x10		/* 0x10 is for TO3 */
	orrlo \r_dest, \r_dest, #\value
#endif
.endm

/*
 * L2CC Cache setup/invalidation/disable
 */
.macro init_l2cc r_zero
	/* explicitly disable L2 cache */
	mrc 15, 0, r0, c1, c0, 1
	bic r0, r0, #0x2
	mcr 15, 0, r0, c1, c0, 1	/* Write Auxiliary Control Register */

	/* reconfigure L2 cache aux control reg */
	mov r1, #0xC4			/* tag/data RAM */
	orr r1, r1, #(1 << 24) |	/* disable write allocate delay */ \
		 (1 << 23) |		/* disable write allocate combine */ \
		 (1 << 22)		/* disable write allocate */

	/* disable write combine for TO 2 and lower revs */
	orr_mx51_tape2_or_lower	r1, (1 << 25), \r_zero

	/* Write L2 Cache Auxiliary Control Register */
	mcr 15, 1, r1, c9, c0, 2

	/* enable L2 cache */
	mrc 15, 0, r0, c1, c0, 1
	orr r0, r0, #2
	mcr 15, 0, r0, c1, c0, 1

.endm /* init_l2cc */

/* AIPS setup - setup MPROTx and PACR registers. */
.macro init_aips r_zero, r_aips1, r_aips2
	/*
	 * Set all MPROTx to be non-bufferable, trusted for R/W,
	 * not forced to user-mode.
	 */
	ldr r1, =0x77777777
	str r1, [\r_aips1, #0x0]
	str r1, [\r_aips1, #0x4]
	str \r_zero, [\r_aips1, #0x40]
	str \r_zero, [\r_aips1, #0x44]
	str \r_zero, [\r_aips1, #0x48]
	str \r_zero, [\r_aips1, #0x4c]
	str \r_zero, [\r_aips1, #0x50]
	str r1, [\r_aips2, #0x0]
	str r1, [\r_aips2, #0x4]
	/*
	 * Clear the on and off peripheral modules Supervisor Protect bit
	 * for SDMA to access them. Did not change the AIPS control registers
	 * (offset 0x20) access type
	 */
.endm /* init_aips */

/* M4IF setup */
.macro init_m4if r_zero, r_m4if
#ifdef CONFIG_MX51
	/* VPU and IPU given higher priority (0x4)
	 * IPU accesses with ID=0x1 given highest priority (=0xA)
	 */
	ldr r1, =0x00000203
	str r1, [\r_m4if, #0x40]

	str \r_zero, [\r_m4if, #0x44]

	ldr r1, =0x00120125
	str r1, [\r_m4if, #0x9C]

	ldr r1, =0x001901A3
	str r1, [\r_m4if, #0x48]

#endif
.endm /* init_m4if */

.macro setup_pll_errata r_pll, freq
	ldr r1, =0x00001236
	str r1, [\r_pll, #PLL_DP_CTL]    /* Restart PLL with PLM=1 */
1:	ldr r1, [\r_pll, #PLL_DP_CTL]    /* Wait for lock */
	ands r1, r1, #0x1
	beq 1b

	ldr r1, =\freq
	str r1, [\r_pll, #PLL_DP_MFN]    /* Modify MFN value */
	str r1, [\r_pll, #PLL_DP_HFS_MFN]

	mov r1, #0x1
	str r1, [\r_pll, #PLL_DP_CONFIG] /* Reload MFN value */

2:	ldr r1, [\r_pll, #PLL_DP_CONFIG]
	tst r1, #1
	bne 2b

	ldr r1, =100		     /* Wait at least 4 us */
3:	subs r1, r1, #1
	bge 3b
.endm

.macro pll_op_mfd_mfn r_pll, dp_op, dp_mfd, dp_mfn, dp_ctl
	mov r0, #\dp_op
	str r0, [\r_pll, #PLL_DP_OP]
	str r0, [\r_pll, #PLL_DP_HFS_OP]

	mov r0, #\dp_mfd
	str r0, [\r_pll, #PLL_DP_MFD]
	str r0, [\r_pll, #PLL_DP_HFS_MFD]

	mov r0, #\dp_mfn
	str r0, [\r_pll, #PLL_DP_MFN]
	str r0, [\r_pll, #PLL_DP_HFS_MFN]

	ldr r0, =\dp_ctl
	str r0, [\r_pll, #PLL_DP_CTL] /* Set DPLL ON (set UPEN bit) */
1:	ldr r0, [\r_pll, #PLL_DP_CTL]
	tst r0, #0x1
	beq 1b
.endm

.macro divisor_change_wait r_ccm
	/* make sure divider effective */
1:	ldr r0, [\r_ccm, #CLKCTL_CDHIPR]
	cmp r0, #0x0
	bne 1b
.endm

#define setup_pll(pll, freq) \
		pll_op_mfd_mfn pll, DP_OP_##freq, DP_MFD_##freq, \
		DP_MFN_##freq, DP_CTL_##freq

#define setup_pll_(pll, freq) setup_pll(pll, freq)

#ifndef CONFIG_SYS_PLL1_FREQ
#define CONFIG_SYS_PLL1_FREQ 800
#endif

#ifndef CONFIG_SYS_DDR_CLKSEL
#define CONFIG_SYS_DDR_CLKSEL 0
#endif

#define ARM_ICGC	0x14

#ifdef CONFIG_MX51
	/* Disable IPU and HSC handshake */
#define IPU_HANDSHAKE_DISABLE	0x60000
#define CCGR6_DISABLE_VAL	0x00000300
#define CBCDR_VAL		0x19239145
#define CBCMR_PER_PLL3		0x000010C0 | CONFIG_SYS_DDR_CLKSEL
#define CBCMR_PER_PLL2		0x000020C0 | CONFIG_SYS_DDR_CLKSEL
#define PLL3_TEMP_FREQ		665
#define PLL2_FREQ		665
#define ARM_ICGC_VAL		0x00000725
#else
	/* Disable IPU handshake */
#define IPU_HANDSHAKE_DISABLE	(1<<21)
#define CCGR6_DISABLE_VAL	0x0F00030F
#define CBCDR_VAL		0x00888945
#define CBCMR_PER_PLL3		0x00015154
#define CBCMR_PER_PLL2		0x00016154
#define PLL3_TEMP_FREQ	 	400
#define PLL2_FREQ		400
#define ARM_ICGC_VAL		0x00000124
#endif

.macro init_clock r_zero, r_ccm, r_pll1, r_pll2, r_pll3, r_pll4

	mov r1, #IPU_HANDSHAKE_DISABLE
	str r1, [\r_ccm, #CLKCTL_CCDR]

	mov r1, #0x2000
	sub r1, r1, #1			/* =0x1fff */
	str r1, [\r_ccm,#CLKCTL_CCMR]	/* Turn on oscillator */

1:	ldr r1, [\r_ccm, #CLKCTL_CSR]
	and r1, r1, #0x22
	cmp r1, #0x22			/* wait for Oscillator and FPM ready */
	bne 1b

/* disable auto restart */
	str     \r_zero, [\r_pll1, #PLL_DP_CONFIG]
	str     \r_zero, [\r_pll2, #PLL_DP_CONFIG]
	str     \r_zero, [\r_pll3, #PLL_DP_CONFIG]

	/* Gate of clocks to the peripherals first */
	ldr r1, =0x3FFFFFFF
	str r1, [\r_ccm, #CLKCTL_CCGR0]
	str \r_zero, [\r_ccm, #CLKCTL_CCGR1]
	str \r_zero, [\r_ccm, #CLKCTL_CCGR2]
	str \r_zero, [\r_ccm, #CLKCTL_CCGR3]

	ldr r1, =0x00030000
	str r1, [\r_ccm, #CLKCTL_CCGR4]
	ldr r1, =0x00FFF030
	str r1, [\r_ccm, #CLKCTL_CCGR5]
	ldr r1, =CCGR6_DISABLE_VAL
	str r1, [\r_ccm, #CLKCTL_CCGR6]
#ifdef CONFIG_MX53
	str \r_zero, [\r_ccm, #CLKCTL_CCGR7]
#endif

	ldr r3, =CBCDR_VAL
#ifdef CONFIG_MX51
	/* Make sure to switch the DDR away from PLL 1 */
	/* keep the initial ddr podf value */
	ldr r2, [\r_ccm, #CLKCTL_CBCDR]
	str r3, [\r_ccm, #CLKCTL_CBCDR]
	divisor_change_wait \r_ccm
#endif

	/* Switch ARM to step clock (clock source 4) */
	ldr r0, [\r_ccm, #CLKCTL_CCSR]
	/* deselect step clock for glitch free update */
	bic r0, r0, #4
	str r0, [\r_ccm, #CLKCTL_CCSR]
	/* select clock source 4 as source for step clock */
	str \r_zero, [\r_ccm, #CLKCTL_CCSR]

	/* switch arm to step clock */
	mov r1, #0x4
	str r1, [\r_ccm, #CLKCTL_CCSR]

#if defined(CONFIG_MX51_PLL_ERRATA)
	setup_pll(\r_pll1, 864)
	setup_pll_errata \r_pll1, DP_MFN_800_DIT
#else
	setup_pll_(\r_pll1, CONFIG_SYS_PLL1_FREQ)
#endif

	/* Switch ARM back to PLL 1 */
	str \r_zero, [\r_ccm, #CLKCTL_CCSR]
	setup_pll_(\r_pll3, PLL3_TEMP_FREQ)

	/* Switch peripheral to PLL 3 */
	ldr r1, =CBCMR_PER_PLL3
	str r1, [\r_ccm, #CLKCTL_CBCMR]
	/* [25] 0 -> 1 (periph was pll2, now apm) */
	orr r1, r3, #1<<25
	str r1, [\r_ccm, #CLKCTL_CBCDR]
	divisor_change_wait \r_ccm

	setup_pll_(\r_pll2, PLL2_FREQ)

	/* Switch peripheral to PLL2 */
	/* [25] 1 -> 0 (periph was apm, now pll2) */
	str r3, [\r_ccm, #CLKCTL_CBCDR]

	ldr r1, =CBCMR_PER_PLL2
	str r1, [\r_ccm, #CLKCTL_CBCMR]

	setup_pll(\r_pll3, 216)
#ifdef CONFIG_MX53
	setup_pll(\r_pll4, 455)
#endif
	/* Set the platform clock dividers */
	ldr r0, =ARM_BASE_ADDR
	ldr r1, =ARM_ICGC_VAL
	str r1, [r0, #ARM_ICGC]

	mov r1, #0
	/* Run 3.0 at Full speed, for other TO's wait till we increase VDDGP */
	orr_mx51_tape2_or_lower	r1, 1, \r_zero
	str r1, [\r_ccm, #CLKCTL_CACRR]

#ifdef CONFIG_MX51
	/* setup the rest */
	/* Use lp_apm (24MHz) source for perclk */
	ldr r1, =0x000020C2 | CONFIG_SYS_DDR_CLKSEL
	str r1, [\r_ccm, #CLKCTL_CBCMR]
	/* ddr clock from PLL 1, all perclk dividers are 1 since using 24MHz */
	ldr r1, =(CONFIG_SYS_CLKTL_CBCDR & ~(7 << 27))
	/* keep the initial ddr podf value */
	and r2, r2, #(7 << 27)
	orr r1, r1, r2
	str r1, [\r_ccm, #CLKCTL_CBCDR]

	/* Use PLL 2 for UART's, get 66.5MHz from it */
	ldr r1, =0xA5A2A020
	str r1, [\r_ccm, #CLKCTL_CSCMR1]
	ldr r1, =0x00C30321
	str r1, [\r_ccm, #CLKCTL_CSCDR1]
#else
	/* change uart clk parent to pll2 */
	ldr	r1, [\r_ccm, #CLKCTL_CSCMR1]
	and	r1, r1, #0xfcffffff
	orr	r1, r1, #0x01000000
	str	r1, [\r_ccm, #CLKCTL_CSCMR1]

	/* make uart div = 6 */
	ldr	r1, [\r_ccm, #CLKCTL_CSCDR1]
	and	r1, r1, #0xffffffc0
	orr	r1, r1, #0x0a
	str	r1, [\r_ccm, #CLKCTL_CSCDR1]
#endif
	divisor_change_wait \r_ccm

	/* Restore the default values in the Gate registers */
	ldr r1, =0xFFFFFFFF
	str r1, [\r_ccm, #CLKCTL_CCGR0]
	str r1, [\r_ccm, #CLKCTL_CCGR1]
	str r1, [\r_ccm, #CLKCTL_CCGR2]
	str r1, [\r_ccm, #CLKCTL_CCGR3]
	str r1, [\r_ccm, #CLKCTL_CCGR4]
	str r1, [\r_ccm, #CLKCTL_CCGR5]
	str r1, [\r_ccm, #CLKCTL_CCGR6]
#ifdef CONFIG_MX53
	str	r1, [\r_ccm, #CLKCTL_CCGR7]
#endif

	str \r_zero, [\r_ccm, #CLKCTL_CCDR]

	/* for cko - for ARM div by 8 */
	mov r1, #0x000A0000
	add r1, r1, #0x00000F0
	str r1, [\r_ccm, #CLKCTL_CCOSR]
.endm

ENTRY(lowlevel_init)
	mov r10, lr
	mov r4, #0	/* Fix R4 to 0 */

#if defined(CONFIG_SYS_MAIN_PWR_ON)
	ldr r0, =GPIO1_BASE_ADDR
	ldr r1, [r0, #0x0]
	orr r1, r1, #1 << 23
	str r1, [r0, #0x0]
	ldr r1, [r0, #0x4]
	orr r1, r1, #1 << 23
	str r1, [r0, #0x4]
#endif

	init_arm_erratum

	init_l1cc r4
	init_l2cc r4

	ldr r5, =AIPS1_BASE_ADDR
	ldr r6, =AIPS2_BASE_ADDR
	init_aips r4, r5, r6

	ldr r5, =M4IF_BASE_ADDR
	init_m4if r4, r5

	ldr r5, =CCM_BASE_ADDR
	ldr r6, =PLL1_BASE_ADDR
	ldr r7, =PLL2_BASE_ADDR
	ldr r8, =PLL3_BASE_ADDR
#ifdef CONFIG_MX53
	ldr r9, =PLL4_BASE_ADDR
#endif
	init_clock r4, r5, r6, r7, r8, r9

	mov pc, r10
ENDPROC(lowlevel_init)
