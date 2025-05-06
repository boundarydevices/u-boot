/*
 * Copyright (C) 2025, Ezurio LLC.
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#include <env.h>
#include <stdio.h>
#include <vsprintf.h>

void bd_setserialnumber(unsigned char mac[6]) {
	char serialbuf[13];

	snprintf(serialbuf, sizeof(serialbuf),
		 "%02x%02x%02x%02x%02x%02x", mac[0],
		 mac[1], mac[2], mac[3],
		 mac[4], mac[5]);
	printf("serial: %s\n", serialbuf);
	env_set("serial#", serialbuf);
}

/* Function to increment a 6-byte MAC address */
void bd_incrementmacaddress(unsigned char mac[6]) {
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
