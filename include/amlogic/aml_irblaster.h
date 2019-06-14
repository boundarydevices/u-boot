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

#ifndef __AML_IRBLATER_H
#define __AML_IRBLATER_H

#define MAX_WINDOWS_LEN 512
struct aml_irblaster_drv_s {
	unsigned int protocol;
	unsigned int frequency;
	unsigned int sendvalue;
	unsigned int windows[MAX_WINDOWS_LEN];
	unsigned int windows_num;
	unsigned int dutycycle;
	unsigned int openflag;
	int (*open)(void);
	int (*close)(void);
	int (*test)(unsigned int);
	int (*send)(unsigned int);
	int (*setprotocol)(char *);
	const char *(*getprocotol)(void);
	int (*setfrequency)(unsigned int);
	unsigned int (*getfrequency)(void);
	void (*print_windows)(void);
	int (*read_reg)(volatile unsigned int *, unsigned int);
	int (*write_reg)(volatile unsigned int *, unsigned int);
};

struct aml_irblaster_drv_s *aml_irblaster_get_driver(void);
#endif


