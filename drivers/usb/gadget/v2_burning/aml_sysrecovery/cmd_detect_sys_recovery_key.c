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
#include <amlogic/gpio.h>

#define debugP(fmt...) //printf("L%d:", __LINE__),printf(fmt)
#define errorP(fmt...) printf("[ERR]L%d:", __LINE__),printf(fmt)
#define MsgP(fmt...)   printf("[msg]"fmt)

static int store_key_open(const char* keyName, const char* keyType)
{
        int pin = -1;

        pin=gpioname_to_pin(keyName);
        if (pin<0) {
                errorP("wrong gpio name %s\n",keyName);
                return -1;
        }
        udelay(100);

        return pin;
}

static int is_sys_recovery_key_pressed(int hKey)
{
        int val = -1;

        val=amlogic_get_value(hKey);

        return val != 1;
}

static int assert_key_is_pressed_in_a_period(unsigned nMillSeconds, const char* keyName, const char* keyType)
{
        unsigned start = 0;
        int hKey = -1;

        hKey = store_key_open(keyName, keyType);
        if (hKey < 0) {
                errorP("Fail to init key for aml_sysrecovery, hKey=%d\n", hKey);
                return __LINE__;
        }

        if (!is_sys_recovery_key_pressed(hKey)) {
                return __LINE__;
        }

        MsgP("pin=%d\n",hKey);
        start = get_timer(0);
        while (is_sys_recovery_key_pressed(hKey))
        {
                const unsigned pressTime = (unsigned)get_timer(start) ;
                if (pressTime > nMillSeconds) {
                        MsgP("store key pressed time %d[ms]\n", pressTime);
                        return 0;
                }
        }
        if (!is_sys_recovery_key_pressed(hKey)) {
                MsgP("key released in time %u[ms]\n", (unsigned)get_timer(start));
                return __LINE__;
        }

        return 1;//restore key released in time @nMillSeconds
}

//test If the recovery_key pressed time >= @nMillSeconds
int do_sys_rec_key(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    int rcode = 0;
    unsigned nMillSeconds = 0;
    const char* keyName =  NULL;
    const char* keyType = "gpio";

    if (2 > argc) {
        cmd_usage(cmdtp);
        return __LINE__;
    }

    nMillSeconds = simple_strtoul(argv[1], NULL, 0);
    keyName = argc > 2 ? argv[2] :"GPIOX_16";
    rcode = assert_key_is_pressed_in_a_period(nMillSeconds, keyName, keyType);

    return rcode;
}


U_BOOT_CMD(
   get_restore_key,      //command name
   5,               //maxargs
   1,               //repeatable
   do_sys_rec_key,   //command function
   "check if user press sys_recovery key",           //description
   "Usage: sys_recovery nMillSeconds [GPIOX_16] [key_type]\n"   //usage
);

