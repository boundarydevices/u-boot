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

#include <common.h>
#include <errno.h>
#include <environment.h>
#include <asm/saradc.h>

#ifndef CONFIG_SARADC_CH
#define CONFIG_SARADC_CH  2
#endif

inline int get_source_key(int channel)
{
	int adc_chan = channel;
    int adc_val = get_adc_sample_gxbb(adc_chan);
    //printf("get_source_key (%d) at channel (%d)\n", adc_val,adc_chan);
    return adc_val;
}

static void check_auto_update(void)
{
	int source_key_value = -1;
	int times=40;
	while (times--) {
		udelay(100000);
		source_key_value = get_source_key(CONFIG_SARADC_CH);
		if ((source_key_value >= 0) && (source_key_value < 40)) {
			//printk("press update key!\n");
		} else {
			return ;
		}
	}

	printf("press update key!\n");
	run_command ("run update", 0);
}

static void press_key_into_update(void)
{
	saradc_enable();
	check_auto_update();
	//saradc_disable();
}

static int do_forceupdate(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	press_key_into_update();
	return 0;
}

U_BOOT_CMD(
	forceupdate,	1,	1,	do_forceupdate,
	"forceupdate",
	"forceupdate - press adc key before power on\n"
);
