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


//is the uboot loaded from usb otg
int is_tpl_loaded_from_usb(void);

//is the uboot loaded from sdcard mmc 0
//note only sdmmc supported by romcode when external device boot
int is_tpl_loaded_from_ext_sdmmc(void);

//Check if uboot loaded from external sdmmc or usb otg
int aml_burn_check_uboot_loaded_for_burn(int flag);

int aml_burn_factory_producing(int flag, bd_t* bis);

//usb producing mode, if tpl loaded from usb pc tool and auto enter producing mode
int aml_try_factory_usb_burning(int flag, bd_t* bis);

//Auto enter sdcard burning if booted from sdcard and aml_sdc_burn.ini existed
int aml_try_factory_sdcard_burning(int flag, bd_t* bis);


