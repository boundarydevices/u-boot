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

#ifndef _VPP_INC_H_
#define _VPP_INC_H_

extern void vpp_init(void);
void vpp_pq_init(int brightness, int contrast, int sat, int hue);
void vpp_pq_load(void);

#define VPP_CM_RGB    0   /* same as COLOR_FMT_RGB444*/
#define VPP_CM_YUV    2   /* same as COLOR_FMT_YUV444*/

extern void vpp_matrix_update(int type);
extern void vpp_viu2_matrix_update(int type);

enum vpp_gamma_sel_e {
	VPP_GAMMA_R = 0,
	VPP_GAMMA_G,
	VPP_GAMMA_B
};

extern void vpp_load_gamma_table(unsigned short *data, unsigned int len, enum vpp_gamma_sel_e flag);
extern void vpp_init_lcd_gamma_table(void);
extern void vpp_disable_lcd_gamma_table(void);
extern void vpp_enable_lcd_gamma_table(void);
extern void hdr_tx_pkt_cb(void);
#endif
