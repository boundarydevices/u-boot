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

.text

.syntax unified
.code 16

.global bss_init
.thumb_func

bss_init:
	mov r0, #0
	ldr r1, _bss_start
	ldr r2, _bss_end
bss_loop:
	cmp r1, r2
	it lt
	strlt r0, [r1], #4
	blt bss_loop
	bx lr

.align 2
_bss_start:
.long _bssstart
_bss_end:
.long _bssend