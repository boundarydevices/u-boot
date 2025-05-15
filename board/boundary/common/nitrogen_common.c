// SPDX-License-Identifier:     GPL-2.0+
// Copyright (C) 2025, Ezurio LLC.

#include <asm/arch/sys_proto.h>
#include <env.h>
#include <net.h>
#include <stdio.h>
#include <vsprintf.h>

void bd_setserialnumber(void) {
	char serialbuf[13];
	unsigned char mac[8];

	imx_get_mac_from_fuse(0, mac);
	if (!is_valid_ethaddr(mac)) {
		printf("fuse not set, can't set serial\n");
		return;
	}

	snprintf(serialbuf, sizeof(serialbuf),
		 "%02x%02x%02x%02x%02x%02x", mac[0],
		 mac[1], mac[2], mac[3],
		 mac[4], mac[5]);
	printf("serial: %s\n", serialbuf);
	env_set("serial#", serialbuf);
}

/* Function to increment a 6-byte MAC address */
void bd_incrementmacaddress(unsigned char mac[6])
{
	// Increment the address byte by byte
	for (int i = 5; i >= 0; i--) {
		mac[i]++;
		if (mac[i] > 255) { // Check for overflow
			mac[i] = 0;
		} else {
			break; // No need to increment further
		}
	}
}
