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

#ifndef __AML_LED_H__
#define __AML_LED_H__

#define LED_TIMER_INTERVAL 10 //ms

#define LED_EVENT_NULL 0
#define LED_EVENT_OFF 1
#define LED_EVENT_ON 2
#define LED_EVENT_FLASH 3
#define LED_EVENT_BREATH 4

#define SHUTDOWN_MODE 0
#define SUSPEND_RESUME_MODE 1
#define RECOVERY_MODE 2


enum led_workmode {
	LWM_OFF,
	LWM_ON,
	LWM_FLASH,
	LWM_BREATH,
	LWM_NULL,
};

/* s,b,w type is enum led_workmode */
#define lwm_set_standby(mode, s) do {mode |= (s) << 0;} while(0)
#define lwm_set_booting(mode, b) do {mode |= (b) << 4;} while(0)
#define lwm_set_working(mode, w) do {mode |= (w) << 8;} while(0)
#define lwm_set_suspend(mode, s) do {mode |= (s) << 12;} while(0)

#define lwm_get_standby(mode) (((mode) >> 0) & 0xF)
#define lwm_get_booting(mode) (((mode) >> 4) & 0xF)
#define lwm_get_working(mode) (((mode) >> 8) & 0xF)
#define lwm_get_suspend(mode) (((mode) >> 12) & 0xF)

struct coord {
	int x;
	int y;
};

struct aml_led_config {
	int off_brightness;
	int on_brightness;
	int flash_off_brightness;
	int flash_off_time;
	int flash_on_brightness;
	int flash_on_time;
	struct coord *breath_inflections;
	int breath_inflections_num;
	void (*set_brightness)(int brightness);
};

#define LED_EVENT_BUF_SIZE 3
struct aml_led {
	int event[LED_EVENT_BUF_SIZE];
	int event_data[LED_EVENT_BUF_SIZE];
	int state;
	int brightness;
	int time;
	int count;
	struct aml_led_config *config;
};

void aml_led_init(struct aml_led *led, struct aml_led_config *config);
void aml_led_timer_proc(struct aml_led *led);
void aml_led_event(struct aml_led *led, int event, int event_data);

#endif