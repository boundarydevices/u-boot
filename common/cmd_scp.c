/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/

#include <config.h>
#include <common.h>
#include <asm/arch/io.h>
#include <command.h>
#include <malloc.h>
#include <asm/arch/mailbox.h>

static const char * const channel_names[] = {
	"command",
	"accel",
	"charger",
	"chipset",
	"clock",
	"dma",
	"events",
	"gesture",
	"gpio",
	"hostcmd",
	"i2c",
	"keyboard",
	"keyscan",
	"lidangle",
	"lightbar",
	"lpc",
	"motionlid",
	"motionsense",
	"pdhostcmd",
	"port80",
	"pwm",
	"spi",
	"switch",
	"system",
	"task",
	"thermal",
	"usb",
	"usbms",
	"usbcharge",
	"usbpd",
	"vboot",
	"hook",
};

int do_open_scp_log(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int i = 0;
	for (i = 0;i < ARRAY_SIZE(channel_names); i++) {
		if (!strcmp(argv[1],channel_names[i]))
			break;
	}
	if (i < ARRAY_SIZE(channel_names)) {
		open_scp_log(1 << i);
		printf("open SCP channel %s log!\n",channel_names[i]);
	} else {
		printf("Error paramters\n");
		printf("open paramets is following:\n");
		for (i = 0;i < ARRAY_SIZE(channel_names); i++)
			printf("%s\n",channel_names[i]);
	}
	return 0;
}

U_BOOT_CMD(
	open_scp_log,	2,	1,	do_open_scp_log,
	"print SCP messgage",
	"print SCP log"
);
