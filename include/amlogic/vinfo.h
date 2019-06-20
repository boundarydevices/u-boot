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

#ifndef __VINFO_H_
#define __VINFO_H_

struct vinfo_s {
	ushort width;  /* Number of columns (i.e. 160) */
	ushort height; /* Number of rows (i.e. 100) */
	ushort field_height; /* for interlace */
	u_char vl_bpix; /* Bits per pixel, 0 = 1 */

	void *vd_base; /* Start of framebuffer memory */

	void *vd_console_address; /* Start of console buffer */
	short console_col;
	short console_row;

	int vd_color_fg;
	int vd_color_bg;

	ushort *cmap; /* Pointer to the colormap */
	void *priv; /* Pointer to driver-specific data */
};


/************************************************************************/
/* ** BITMAP DISPLAY SUPPORT						*/
/************************************************************************/
#if defined(CONFIG_CMD_BMP) || defined(CONFIG_SPLASH_SCREEN)
# include <bmp_layout.h>
# include <asm/byteorder.h>
#endif

/*
 *  Information about displays we are using. This is for configuring
 *  the LCD controller and memory allocation. Someone has to know what
 *  is connected, as we can't autodetect anything.
 */
#define CONFIG_SYS_HIGH	0	/* Pins are active high			*/
#define CONFIG_SYS_LOW		1	/* Pins are active low			*/

#define LCD_COLOR2	2
#define LCD_COLOR4	4
#define LCD_COLOR8	8
#define LCD_COLOR16	16
#define LCD_COLOR24	24
#define LCD_COLOR32	32


/*----------------------------------------------------------------------*/
#if defined(CONFIG_LCD_INFO_BELOW_LOGO)
# define LCD_INFO_X		0
# define LCD_INFO_Y		(BMP_LOGO_HEIGHT + VIDEO_FONT_HEIGHT)
#elif defined(CONFIG_LCD_LOGO)
# define LCD_INFO_X		(BMP_LOGO_WIDTH + 4 * VIDEO_FONT_WIDTH)
# define LCD_INFO_Y		(VIDEO_FONT_HEIGHT)
#else
# define LCD_INFO_X		(VIDEO_FONT_WIDTH)
# define LCD_INFO_Y		(VIDEO_FONT_HEIGHT)
#endif

/* Calculate nr. of bits per pixel  and nr. of colors */
#define NBITS(bit_code)		(bit_code)
#define NCOLORS(bit_code)	(1 << NBITS(bit_code))

/************************************************************************/
/* ** CONSOLE CONSTANTS							*/
/************************************************************************/
#if LCD_BPP == LCD_MONOCHROME

/*
 * Simple black/white definitions
 */
# define CONSOLE_COLOR_BLACK	0
# define CONSOLE_COLOR_WHITE	1	/* Must remain last / highest	*/

#elif LCD_BPP == LCD_COLOR8

/*
 * 8bpp color definitions
 */
# define CONSOLE_COLOR_BLACK	0
# define CONSOLE_COLOR_RED	1
# define CONSOLE_COLOR_GREEN	2
# define CONSOLE_COLOR_YELLOW	3
# define CONSOLE_COLOR_BLUE	4
# define CONSOLE_COLOR_MAGENTA	5
# define CONSOLE_COLOR_CYAN	6
# define CONSOLE_COLOR_GREY	14
# define CONSOLE_COLOR_WHITE	15	/* Must remain last / highest	*/

#elif LCD_BPP == LCD_COLOR24
/*
 * 24bpp color definitions
 */
# define CONSOLE_COLOR_BLACK	 0
# define CONSOLE_COLOR_RED 	0x0000ff
# define CONSOLE_COLOR_GREEN	0x00ff00
# define CONSOLE_COLOR_YELLOW	0x00ffff
# define CONSOLE_COLOR_BLUE	0xff0000
# define CONSOLE_COLOR_MAGENTA	0xff00ff
# define CONSOLE_COLOR_CYAN	0xffff00
# define CONSOLE_COLOR_GREY	0x808080
# define CONSOLE_COLOR_WHITE	0xffffff	/* Must remain last / highest	*/

#else

/*
 * 16bpp color definitions
 */
# define CONSOLE_COLOR_BLACK	0x0000
# define CONSOLE_COLOR_RED 		0xf800
# define CONSOLE_COLOR_GREEN	0x07e0
# define CONSOLE_COLOR_YELLOW	0xffe0
# define CONSOLE_COLOR_BLUE		0x001f
# define CONSOLE_COLOR_MAGENTA	0xf81f
# define CONSOLE_COLOR_CYAN		0x07ff
# define CONSOLE_COLOR_WHITE	0xffff	/* Must remain last / highest	*/

#endif /* color definitions */

/************************************************************************/
#ifndef PAGE_SIZE
# define PAGE_SIZE	4096
#endif

/************************************************************************/
/* ** CONSOLE DEFINITIONS & FUNCTIONS					*/
/************************************************************************/
#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
# define CONSOLE_ROWS		((info->vl_row-BMP_LOGO_HEIGHT) \
					/ VIDEO_FONT_HEIGHT)
#else
# define CONSOLE_ROWS		(info->vl_row / VIDEO_FONT_HEIGHT)
#endif

#define CONSOLE_COLS		(panel_info.vl_col / VIDEO_FONT_WIDTH)
#define CONSOLE_ROW_SIZE	(VIDEO_FONT_HEIGHT * lcd_line_length)
#define CONSOLE_ROW_FIRST	(info->vd_console_address)
#define CONSOLE_ROW_SECOND	(info->vd_console_address + CONSOLE_ROW_SIZE)
#define CONSOLE_ROW_LAST	(info->vd_console_address + CONSOLE_SIZE \
					- CONSOLE_ROW_SIZE)
#define CONSOLE_SIZE		(CONSOLE_ROW_SIZE * CONSOLE_ROWS)
#define CONSOLE_SCROLL_SIZE	(CONSOLE_SIZE - CONSOLE_ROW_SIZE)

# define COLOR_MASK(c)		(c)

/************************************************************************/

#endif
