/*
 * (C) Copyright 2009
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/imx-common/sys_proto.h>
#ifdef CONFIG_MX6SX
int arch_auxiliary_core_up(u32 core_id, u32 boot_private_data);
#endif
