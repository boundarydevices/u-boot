/*
 * Copyright Boundary Devices
 *
 * SPDX-License-Identifier: GPL-2.0+
 */
#include <config.h>
#include <linux/kernel.h>
#include <asm/arch/ddr.h>
#include <asm/arch/lpddr4_define.h>

#ifdef WR_POST_EXT_3200
#define CH1_VAL_INIT4	((LPDDR4_MR3 << 16) | 0x00020008)
#else
#define CH1_VAL_INIT4	((LPDDR4_MR3 << 16) | 8)
#endif

#ifdef CONFIG_IMX8MN
#define IBASE_DIFF	1
#else
#define IBASE_DIFF	0
#endif

#if CONFIG_DDR_MB == 1024
	/* Address map is from MSB 28: r15, r14, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R0	0x0000001F
#define CH1_VAL_DDRC_ADDRMAP6_R0	(0x06060606 + IBASE_DIFF * 0x01010101)
	/* Address map is from MSB 28: cs, r14, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R1	(0x00000015 + IBASE_DIFF * 0x01)
#define CH1_VAL_DDRC_ADDRMAP6_R1	(0x0F060606 + IBASE_DIFF * 0x010101)

#elif CONFIG_DDR_MB == 1536
	/* Address map is from MSB 29: r15, r14, cs, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R1	(0x00000014 + IBASE_DIFF * 0x01)
#define CH1_VAL_DDRC_ADDRMAP6_R1	(0x47070606 + IBASE_DIFF * 0x01010101)

#elif CONFIG_DDR_MB == 2048
	/* Address map is from MSB 29: cs, r15, r14, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R1	(0x00000016 + IBASE_DIFF * 0x01)
#define CH1_VAL_DDRC_ADDRMAP6_R1	(0x06060606 + IBASE_DIFF * 0x01010101)
#else
#error unsupported memory size
#endif

#define LPDDR4_CS_R0	0x1	/* 0 rank bits, 1 chip select */
#define LPDDR4_CS_R1	0x3	/* 1 rank bit, 2 chip selects */

#if (CONFIG_DDR_RANK_BITS == 0) || !defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#ifdef CH1_VAL_DDRC_ADDRMAP0_R0
#define CH1_LPDDR4_CS			LPDDR4_CS_R0
#define CH1_VAL_DDRC_ADDRMAP0		CH1_VAL_DDRC_ADDRMAP0_R0
#define CH1_VAL_DDRC_ADDRMAP6		CH1_VAL_DDRC_ADDRMAP6_R0
#else
#error unsupported memory rank/size
#endif
/*
 * rank0 will succeed, even if really rank 1, so we need
 * to probe memory if rank0 succeeds
 */
#if defined(CH1_VAL_DDRC_ADDRMAP0_R0) && defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#define CH1_LPDDR4_CS_NEW		LPDDR4_CS_R1
#define CH1_VAL_DDRC_ADDRMAP0_NEW	CH1_VAL_DDRC_ADDRMAP0_R1
#define CH1_VAL_DDRC_ADDRMAP6_NEW	CH1_VAL_DDRC_ADDRMAP6_R1
#endif

#elif (CONFIG_DDR_RANK_BITS == 1) || !defined(CH1_VAL_DDRC_ADDRMAP0_R0)
#ifdef CH1_VAL_DDRC_ADDRMAP0_R1
#define CH1_LPDDR4_CS			LPDDR4_CS_R1
#define CH1_VAL_DDRC_ADDRMAP0		CH1_VAL_DDRC_ADDRMAP0_R1
#define CH1_VAL_DDRC_ADDRMAP6		CH1_VAL_DDRC_ADDRMAP6_R1
#else
#error unsupported memory rank/size
#endif

#if defined(CH1_VAL_DDRC_ADDRMAP0_R0) && defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#define CH1_LPDDR4_CS_NEW		LPDDR4_CS_R0
#define CH1_VAL_DDRC_ADDRMAP0_NEW	CH1_VAL_DDRC_ADDRMAP0_R0
#define CH1_VAL_DDRC_ADDRMAP6_NEW	CH1_VAL_DDRC_ADDRMAP6_R0
#endif

#else
#error unsupported rank bits
#endif

#if (CONFIG_DDR_CHANNEL_CNT == 1)
#if (CONFIG_DDR_RANK_BITS == 0) && !defined(CH1_VAL_DDRC_ADDRMAP0_R0)
#error unsupported options
#endif
#if (CONFIG_DDR_RANK_BITS == 1) && !defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#error unsupported options
#endif
#endif
