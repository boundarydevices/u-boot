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
#include "platform.h"
#include "usb_pcd.h"

#include "usb_pcd.c"
#include "platform.c"
#include "dwc_pcd.c"
#include "dwc_pcd_irq.c"

DECLARE_GLOBAL_DATA_PTR;

int v2_usbburning(unsigned timeout)
{
        int cfg = EXT_CLOCK;

#if defined(CONFIG_SILENT_CONSOLE)
        gd->flags &= ~GD_FLG_SILENT;
#endif
        printf("InUsbBurn\n");
        set_usb_phy_config(cfg);
        usb_parameter_init(timeout);
        if (usb_pcd_init()) {
                /*printf("Fail in usb_pcd_init\n");*/
                return __LINE__;
        }

#if (MESON_CPU_TYPE_MESON8 <= MESON_CPU_TYPE)
        //AML_WATCH_DOG_DISABLE(); //disable watchdog
#endif// #if (MESON_CPU_TYPE_MESON8 <= MESON_CPU_TYPE)

        while (1)
        {
                //watchdog_clear();		//Elvis Fool
                if (usb_pcd_irq())
                        break;
        }
        return 0;
}

int do_v2_usbtool (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    int rc = 0;
    unsigned timeout            = (2 <= argc) ? simple_strtoul(argv[1], NULL, 0) : 0;
    //if get burning tool identify command in pcToolWaitTime, then auto jump into burning mode
    unsigned pcToolWaitTime     = (3 <= argc) ? simple_strtoul(argv[2], NULL, 0) : 0;

    optimus_work_mode_set(OPTIMUS_WORK_MODE_USB_UPDATE);
    setenv(_ENV_TIME_OUT_TO_AUTO_BURN, pcToolWaitTime ? argv[2] : "");

    rc = v2_usbburning(timeout);
    /*close_usb_phy_clock(0);*/

    return rc;
}


U_BOOT_CMD(
	update,	3,	0,	do_v2_usbtool,
	"Enter v2 usbburning mode",
	"usbburning timeout"
);

