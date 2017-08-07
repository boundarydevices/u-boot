/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  relocate - common relocation function for ARM U-Boot
 *
 *  Copyright (c) 2013  Albert ARIBAUD <albert.u.boot@aribaud.net>
 */

#include <asm-offsets.h>
#include <asm/assembler.h>
#include <config.h>
#include <elf.h>
#include <linux/linkage.h>
#ifdef CONFIG_CPU_V7M
#include <asm/armv7m.h>
#endif

/*
 * Default/weak exception vectors relocation routine
 *
 * This routine covers the standard ARM cases: normal (0x00000000),
 * high (0xffff0000) and VBAR. SoCs which do not comply with any of
 * the standard cases must provide their own, strong, version.
 */

	.section	.text.relocate_vectors,"ax",%progbits
	.weak		relocate_vectors

ENTRY(relocate_vectors)

#ifdef CONFIG_CPU_V7M
	/*
	 * On ARMv7-M we only have to write the new vector address
	 * to VTOR register.
	 */
	ldr	r0, [r9, #GD_RELOCADDR]	/* r0 = gd->relocaddr */
	ldr	r1, =V7M_SCB_BASE
	str	r0, [r1, V7M_SCB_VTOR]
#else
#ifdef CONFIG_HAS_VBAR
	/*
	 * If the ARM processor has the security extensions,
	 * use VBAR to relocate the exception vectors.
	 */
	ldr	r0, [r9, #GD_RELOCADDR]	/* r0 = gd->relocaddr */
	mcr     p15, 0, r0, c12, c0, 0  /* Set VBAR */
#else
	/*
	 * Copy the relocated exception vectors to the
	 * correct address
	 * CP15 c1 V bit gives us the location of the vectors:
	 * 0x00000000 or 0xFFFF0000.
	 */
	ldr	r0, [r9, #GD_RELOCADDR]	/* r0 = gd->relocaddr */
	mrc	p15, 0, r2, c1, c0, 0	/* V bit (bit[13]) in CP15 c1 */
	ands	r2, r2, #(1 << 13)
	ldreq	r1, =0x00000000		/* If V=0 */
	ldrne	r1, =0xFFFF0000		/* If V=1 */
	ldmia	r0!, {r2-r8,r10}
	stmia	r1!, {r2-r8,r10}
	ldmia	r0!, {r2-r8,r10}
	stmia	r1!, {r2-r8,r10}
#endif
#endif
	bx	lr

ENDPROC(relocate_vectors)

/*
 * void relocate_code(addr_moni)
 *
 * This function relocates the monitor code.
 *
 * NOTE:
 * To prevent the code below from containing references with an R_ARM_ABS32
 * relocation record type, we never refer to linker-defined symbols directly.
 * Instead, we declare literals which contain their relative location with
 * respect to relocate_code, and at run time, add relocate_code back to them.
 */

ENTRY(relocate_code)
	stmdb   sp!, {lr}
	ldr	r1, =__image_copy_start	/* r1 <- SRC &__image_copy_start */
	subs	r4, r0, r1		/* r4 <- relocation offset */
	beq	relocate_done		/* skip relocation */
	ldr	r2, =__image_copy_end	/* r2 <- SRC &__image_copy_end */

copy_loop:
	ldmia	r1!, {r10-r11}		/* copy from source address [r1]    */
	stmia	r0!, {r10-r11}		/* copy to   target address [r0]    */
	cmp	r1, r2			/* until source end address [r2]    */
	blo	copy_loop

	/*
	 * fix .rel.dyn relocations
	 */
	ldr	r2, =__rel_dyn_start	/* r2 <- SRC &__rel_dyn_start */
	ldr	r3, =__rel_dyn_end	/* r3 <- SRC &__rel_dyn_end */
fixloop:
	ldmia	r2!, {r0-r1}		/* (r0,r1) <- (SRC location,fixup) */
	and	r1, r1, #0xff
	cmp	r1, #R_ARM_RELATIVE
	bne	fixnext

	tst	r0, #3
	beq	1f
	sub	r0, r2, #8
	bl	prhex
	mov	r0, #':'
	bl	pr_c
	ldr	r0,[r2, #-8]
	bl	prhex
	mov	r0, #'_'
	bl	pr_c
	ldr	r0,[r2, #-4]
	bl	prhex
	b	fixnext
1:
	/* relative fix: increase location by offset */
	add	r0, r0, r4
	ldr	r1, [r0]
	add	r1, r1, r4
	str	r1, [r0]
fixnext:
	cmp	r2, r3
	blo	fixloop

relocate_done:

#ifdef __XSCALE__
	/*
	 * On xscale, icache must be invalidated and write buffers drained,
	 * even with cache disabled - 4.2.7 of xscale core developer's manual
	 */
	mcr	p15, 0, r0, c7, c7, 0	/* invalidate icache */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer */
#endif

	/* ARMv4- don't know bx lr but the assembler fails to see that */

#ifdef __ARM_ARCH_4__
	mov	pc, lr
#else
	ldmia	sp!, {pc}
#endif

ENDPROC(relocate_code)

ENTRY(prhex)
	stmdb   sp!, {r0-r6,lr}
	mov	r6, #8
	mov	r5, r0
1:	mov	r0, r5, lsr #28
	cmp	r0, #10
	addlt	r0, r0, #'0'
	addge	r0, r0, #'a' - 10
	bl	pr_c
	mov	r5, r5, lsl #4
	subs	r6, r6, #1
	bne	1b

	mov	r0,#'\n'
	bl	pr_c
	mov	r0,#'\r'
	bl	pr_c
	ldmia	sp!, {r0-r6,pc}
ENDPROC(prhex)

#define UTXD		0x40
#define UTS		0xb4
#define UTS_TXEMPTY	(1<<6)

ENTRY(pr_c)
	stmdb   sp!, {r0-r1,lr}
	ldr	r1, =CONFIG_MXC_UART_BASE
	str	r0, [r1,#UTXD]

1:	ldr	r0, [r1,#UTS]
	tst	r0, #UTS_TXEMPTY
	beq	1b
	ldmia	sp!, {r0-r1,pc}
ENDPROC(pr_c)

#if 0
ENTRY(verify_relocate_table)
	stmdb   sp!, {lr}
	/*
	 * check .rel.dyn relocations
	 */
	ldr	r2, =__rel_dyn_start	/* r2 <- SRC &__rel_dyn_start */
	ldr	r3, =__rel_dyn_end	/* r3 <- SRC &__rel_dyn_end */
1:
	ldmia	r2!, {r0-r1}		/* (r0,r1) <- (SRC location,fixup) */
	and	r1, r1, #0xff
	cmp	r1, #R_ARM_RELATIVE
	bne	3f

	tst	r0, #3
	beq	2f
	sub	r0, r2, #8
	bl	prhex
	mov	r0, #':'
	bl	pr_c
	ldr	r0,[r2, #-8]
	bl	prhex
	mov	r0, #'_'
	bl	pr_c
	ldr	r0,[r2, #-4]
	bl	prhex
	mov	r0, #1
	ldmia	sp!, {pc}
2:
3:
	cmp	r2, r3
	blo	1b

	mov	r0, #0
	ldmia	sp!, {pc}

ENDPROC(verify_relocate_table)
#endif
