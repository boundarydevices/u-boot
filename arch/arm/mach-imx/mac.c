// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017 NXP
 *
 * Peng Fan <peng.fan@nxp.com>
 */

#include <common.h>
#include <asm/arch/imx-regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <errno.h>

struct imx_mac_fuse {
	u32 mac_addr0;
	u32 rsvd0[3];
	u32 mac_addr1;
	u32 rsvd1[3];
	u32 mac_addr2;
	u32 rsvd2[7];
};

#define MAC_FUSE_MX6_OFFSET	0x620
#define MAC_FUSE_MX7_OFFSET	0x640

void imx_get_mac_from_fuse(int dev_id, unsigned char *mac)
{
	struct imx_mac_fuse *fuse;
	u32 offset;
	bool has_second_mac;
	u32 value_high, value_low;

	offset = is_mx6() ? MAC_FUSE_MX6_OFFSET : MAC_FUSE_MX7_OFFSET;
	fuse = (struct imx_mac_fuse *)(ulong)(OCOTP_BASE_ADDR + offset);
	has_second_mac = is_mx7() || is_mx6sx() || is_mx6ul() || is_mx6ull();

	if (has_second_mac && dev_id == 1) {
		value_high = readl(&fuse->mac_addr2);
		value_low = readl(&fuse->mac_addr1) >> 16;

		value_low |= (value_high << 16);
		value_high >>= 16;
		if (value_low | value_high) {
			dev_id--;
			goto valid;
		}
	}
	value_high = readl(&fuse->mac_addr1);
	value_low = readl(&fuse->mac_addr0);
valid:
	if ((dev_id > 0) && (value_low | value_high)) {
		u32 prev = value_low;
		value_low += dev_id;
		if (value_low < prev)
			value_high++;
	}
	mac[0] = value_high >> 8;
	mac[1] = value_high;
	mac[2] = value_low >> 24;
	mac[3] = value_low >> 16;
	mac[4] = value_low >> 8;
	mac[5] = value_low;
}
