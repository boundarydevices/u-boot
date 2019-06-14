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

#ifndef _RENDER_API_H_
#define _RENDER_API_H_

#include "minui.h"

void set_fastboot_flag(int flag);
int screen_init(void);
void screen_uninit(void);
int gr_init_ext_font(const char* font, GRFont** dest);
int surface_loadbmp(GRSurface** surface, const char* filename);
void surface_disaplay(GRSurface* surface, int sx, int sy, int dx, int dy);
void screen_setcolor(unsigned int color);
void screen_drawtextline(const GRFont* font, int x, int y, const char *s, bool bold);
void screen_fillrect(int x, int y, int w, int h);
void screen_update(void);

#endif
