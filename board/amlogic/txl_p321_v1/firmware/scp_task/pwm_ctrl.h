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



static int pwm_voltage_table[][2] = {
	{ 0x1c0000,  870},
	{ 0x1b0001,  880},
	{ 0x1a0002,  890},
	{ 0x190003,  900},
	{ 0x180004,  910},
	{ 0x170005,  920},
	{ 0x160006,  930},
	{ 0x150007,  940},
	{ 0x140008,  950},
	{ 0x130009,  960},
	{ 0x12000a,  970},
	{ 0x11000b,  980},
	{ 0x10000c,  990},
	{ 0x0f000d, 1000},
	{ 0x0e000e, 1010},
	{ 0x0d000f, 1020},
	{ 0x0c0010, 1030},
	{ 0x0b0011, 1040},
	{ 0x0a0012, 1050},
	{ 0x090013, 1060},
	{ 0x080014, 1070},
	{ 0x070015, 1080},
	{ 0x060016, 1090},
	{ 0x050017, 1100},
	{ 0x040018, 1110},
	{ 0x030019, 1120},
	{ 0x02001a, 1130},
	{ 0x01001b, 1140},
	{ 0x00001c, 1150}
};
