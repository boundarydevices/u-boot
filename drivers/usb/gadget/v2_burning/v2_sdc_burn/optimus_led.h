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

#ifndef __OPTIMUS_LED_H__
#define __OPTIMUS_LED_H__

#define LED_TYPE_PWM        0xabcd

#if CONFIG_SD_BURNING_SUPPORT_LED

int optimus_led_open(int ledType);//open the led for show burning states

int optimus_led_close(void);

int optimus_led_show_in_process_of_burning(void);

int optimus_led_show_burning_success(void);

int optimus_led_show_burning_failure(void);

#else
#define optimus_led_open(ledType)                   0
#define optimus_led_close()                         0
#define optimus_led_show_in_process_of_burning()    do{}while(0)
#define optimus_led_show_burning_success()          do{}while(0)
#define optimus_led_show_burning_failure()          do{}while(0)

#endif// #if CONFIG_SD_BURNING_SUPPORT_LED

#endif//ifndef __OPTIMUS_LED_H__

