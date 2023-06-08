// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 */

#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <init.h>
#include <wdt.h>
#include <dm/uclass-internal.h>
#include <linux/arm-smccc.h>

int arch_cpu_init(void)
{
	icache_enable();

	return 0;
}

void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}

#ifdef MTK_SIP_PARTNAME_ID
/**
 * mediatek_sip_part_name - get the part name
 *
 * Retrieve the part name of platform description.
 * This only applicable to SoCs that support SIP partname
 * SMC call.
 *
 * Return:
 * * > 0 - the part name invoked
 * * 0   - error or no part name invoked
 */
u32 mediatek_sip_part_name(void)
{
	struct arm_smccc_res res __maybe_unused;
	u32 ret = 0;

	arm_smccc_smc(MTK_SIP_PARTNAME_ID, 0, 0, 0, 0, 0, 0, 0, &res);
	ret = res.a1;

	if (res.a0)
		return 0;
	else
		return ret;
}
#else
u32 mediatek_sip_part_name(void)
{
	return 0;
}
#endif

