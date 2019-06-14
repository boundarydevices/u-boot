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

#ifndef VOUT_H
#define VOUT_H

#include <amlogic/vinfo.h>

#define VOUT_VIU1_SEL    1
#define VOUT_VIU2_SEL    2

enum viu_mux_e {
	VIU_MUX_ENCL = 0,
	VIU_MUX_ENCI,
	VIU_MUX_ENCP,
	VIU_MUX_MAX,
};

void vout_init(void);
void vout_vinfo_dump(void);
int vout_get_current_vmode(void);
int vout_get_current_axis(int *axis);
void vout_set_current_vmode(int mode);
struct vinfo_s *vout_get_current_vinfo(void);
extern void vout_viu_mux(int viu_sel, int venc_sel);
extern unsigned long get_fb_addr(void);
#endif

