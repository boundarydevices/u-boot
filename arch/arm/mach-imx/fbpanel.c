/*
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <command.h>
#include <errno.h>
#include <asm/arch/clock.h>
#if !defined(CONFIG_IMX8ULP)
#include <asm/arch/crm_regs.h>
#endif
#if !defined(CONFIG_MX51) && !defined(CONFIG_MX53) && !defined(CONFIG_MX7D) && !defined(CONFIG_IMX8M) && !defined(CONFIG_IMX8ULP)
#include <asm/arch/mxc_hdmi.h>
#endif
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/io.h>
#include <div64.h>
#include <dm/device.h>
#include <dm/lists.h>
#include <dm/uclass.h>
#include <dt-bindings/gpio/gpio.h>
#include <env.h>
#include <i2c.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <log.h>
#include <malloc.h>
#include <video_fb.h>

#if !defined(CONFIG_MX51) && !defined(CONFIG_MX53) && !defined(CONFIG_MX6UL) && !defined(CONFIG_MX6ULL) && !defined(CONFIG_MX7D) && !defined(CONFIG_IMX8M) && !defined(CONFIG_IMX8ULP)
#define LVDS_SUPPORT
#endif

/*
 * The format of the string is
 * [*][off|mode_str_name[:["dtb_alias",["dtb_alias",]][18|24][m][j][s][b][:][S|D|b|P|l|o|d|s|h|f|a|v|B|M|U|c|L|E|4|3|2|1][Gn][Rn][e][x[x_vals][p[pwm_period]]]:timings]]
 *
 * * means select this display for u-boot to initialize.
 * "dtb_alias" : dtb alias to set status to okay, use to enable touch screen drivers, panel drivers(sn65)
 * 18 : pix_fmt = IPU_PIX_FMT_RGB666
 * 24 : pix_fmt = IPU_PIX_FMT_RGB24
 * m : mode_str is used, fdt set fb_xxx mode_str [mode_str_name]
 * j : display uses JEIDA, fdt set ldb/lvds-channel@n fsl,data-mapping jeida
 * s : display uses SPLITMODE, fdt set ldb split-mode 1
 * b : pix_fmt = IPU_PIX_FMT_BGR24
 * S : This display requires SPI initialization
 * A : use alternate pwm
 * D : Change dtb backlight level, fdt set backlight_lvds brightness-levels <0 1 2 3 4 5 6 7 8 9 10>
 * b : Change dtb backlight level, low active, fdt set backlight_lvds brightness-levels <10 9 8 7 6 5 4 3 2 1 0>
 * P : Set pinctrl, fdt get value pin pinctrl_##[mode_str_name] phandle; fdt set fb_mipi pinctrl-0 <${pin}>; fdt set fb_mipi pinctrl-names default
 * l : Set low active enable gpio, fdt get value gp gpio##[gpio/32] phandle;fdt set fb_mipi enable-gpios <${gp} [gpio&0x1f] 1>
 * o : set open drain gpio, fdt get value gp gpio##[gpio/32] phandle;fdt set fb_mipi enable-gpios <${gp} [gpio&0x1f] 6>
 * d : Set enable gpio, fdt get value gp gpio##[gpio/32] phandle;fdt set fb_mipi enable-gpios <${gp} [gpio&0x1f] 0>
 * s : mipi skip_eot, fdt set mipi mode-skip-eot
 * h : fdt set mipi mode-video-hbp-disable
 * f : fdt set mipi mode-video-hfp-disable
 * a : fdt set mipi mode-video-hsa-disable
 * v : mipi mode video, fdt set mipi mode-video
 * B : mipi video burst mode, fdt set mipi mode-video-burst
 * M : mipi byte clock is a multiple of pixel clock, fdt set mipi mode-video-mbc
 * U : mipi sync pulse, fdt set mipi mode-video-sync-pulse
 * c : mipi commands, fdt get value cmds mipi_cmds_##[mode_str_name] phandle;fdt set mipi mipi-cmds <${cmds}>
 * L : backlight enable low active, fdt get value gp gpio##[bkgpio/32] phandle;fdt set backlight_lvds enable-gpios <${gp} [bkgpio&0x1f] 1>
 * E : backlight enable, fdt get value gp gpio##[bkgpio/32] phandle;fdt set backlight_lvds enable-gpios <${gp} [bkgpio&0x1f] 0>
 * I : use IPU clock
 * 4 : mipi 4 lanes
 * 3 : mipi 3 lanes
 * 2 : mipi 2 lanes
 * 1 : mipi 1 lane
 * G : enable gpio
 * R : reset gpio
 * e : call board_pre_enable
 * x : x_vals: bus_num,addr,bus_gp,bus_gp_delay_ms,bkgpio,min_hs_clock_multiple
 * 	bus_num, i2c bus number
 *      addr_num, hex i2c bus address
 *      bus_gp, bus enable gpio before probe
 *      bus_gp_delay_ms, delay after enable before probe
 *      bkgpio, backlight enable gpio
 *      min_hs_clock_multiple, mipi hs clk * 2(because of ddr) >= pixel clk * min_hs_clock_multiple
 *      enable_high_duration_us - high time of short pulse before leaving enable high
 *      enable_low_duration_us - low time of short pulse before leaving enable high
 *      power_gp, power enable gpio
 *      power_delay_ms, delay needed after turning on power
 * p : pwm_period, fdt get value pwm pwm_lvds phandle; fdt set backlight_lvds pwms <${pwm} 0 [pwm_period]>
 * timings : clock-frequency,hactive,vactive,hback-porch,hfront-porch,vback-porch,vfront-porch,hsync-len,vsync-len,
 *
 */
/*
 * This creates commands strings to work on dtb files.
 * i.e. if you define the following environment variables
 *
 * setenv fb_hdmi 1920x1080M@60
 * setenv fb_lcd *off
 * setenv fb_lvds lg1280x800
 *
 * Note: the "*" means u-boot should enable this display, in this case none.
 *
 * you'll get the follow strings if cmd_frozen is not defined.
 * cmd_hdmi=fdt set fb_hdmi status okay;fdt set fb_hdmi mode_str 1920x1080M@60;
 *
 * cmd_lcd=fdt set fb_lcd status disabled
 *
 * cmd_lvds=fdt set fb_lvds status okay;
 * 	fdt set fb_lvds interface_pix_fmt RGB24;
 * 	fdt set ldb/lvds-channel@0 fsl,data-width <24>;
 * 	fdt set ldb/lvds-channel@0 fsl,data-mapping jeida;
 * 	fdt rm ldb split-mode;
 * 	fdt set t_lvds clock-frequency <71107200>;
 * 	fdt set t_lvds hactive <1280>;
 * 	fdt set t_lvds vactive <800>;
 * 	fdt set t_lvds hback-porch <48>;
 * 	fdt set t_lvds hfront-porch <80>;
 * 	fdt set t_lvds vback-porch <15>;
 * 	fdt set t_lvds vfront-porch <2>;
 * 	fdt set t_lvds hsync-len <32>;
 * 	fdt set t_lvds vsync-len <6>;
 *
 * cmd_lvds2=fdt set fb_lvds2 status disabled
 *
 *
 * These strings are then used in 6x_bootscript to configure displays
 *
 * The following aliases should be defined in the dtb if possible
 * fb_hdmi, fb_lcd, fb_lvds, fb_lvds2
 * lcd
 * ldb
 * t_lvds, t_lvds2
 */

#define is_mipi(fb) ((fb == FB_MIPI) || (fb == FB_MIPI2))

static const char *const fbnames[] = {
[FB_HDMI] = "fb_hdmi",
[FB_LCD] = "fb_lcd",
[FB_LCD2] = "fb_lcd2",
[FB_LVDS] = "fb_lvds",
[FB_LVDS2] = "fb_lvds2",
[FB_MIPI] = "fb_mipi",
[FB_MIPI2] = "fb_mipi2",
#if defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP)
[FB_MIPI_BRIDGE] = "mipi_dsi_bridge",
#else
[FB_MIPI_BRIDGE] = "mipi_dsi",
#endif
};

static const char *const fbnames_name[] = {
[FB_HDMI] = "fb_hdmi_name",
[FB_LCD] = "fb_lcd_name",
[FB_LCD2] = "fb_lcd2_name",
[FB_LVDS] = "fb_lvds_name",
[FB_LVDS2] = "fb_lvds2_name",
[FB_MIPI] = "fb_mipi_name",
[FB_MIPI2] = "fb_mipi2_name",
};

static const char *const timings_names[] = {
[FB_HDMI] = "t_hdmi",
[FB_LCD] = "t_lcd",
[FB_LCD2] = "t_lcd2",
[FB_LVDS] = "t_lvds",
[FB_LVDS2] = "t_lvds2",
[FB_MIPI] = "t_mipi",
[FB_MIPI2] = "t_mipi2",
};

static const char *const ch_names[] = {
[FB_HDMI] = "",
[FB_LCD] = "",
[FB_LCD2] = "",
[FB_LVDS] = "ldb/lvds-channel@0",
[FB_LVDS2] = "ldb/lvds-channel@1",
[FB_MIPI] = "",
[FB_MIPI2] = "",
};

static const char *const cmd_fbnames[] = {
[FB_HDMI] = "cmd_hdmi",
[FB_LCD] = "cmd_lcd",
[FB_LCD2] = "cmd_lcd2",
[FB_LVDS] = "cmd_lvds",
[FB_LVDS2] = "cmd_lvds2",
[FB_MIPI] = "cmd_mipi",
[FB_MIPI2] = "cmd_mipi2",
};

static const char *const backlight_names[] = {
[FB_HDMI] = "backlight_hdmi",
[FB_LCD] = "backlight_lcd",
[FB_LCD2] = "backlight_lcd2",
[FB_LVDS] = "backlight_lvds",
[FB_LVDS2] = "backlight_lvds2",
[FB_MIPI] = "backlight_mipi",
[FB_MIPI2] = "backlight_mipi2",
};

static const char backlight_alt[] = "backlight_alt";

static const char *const pwm_names[] = {
[FB_HDMI] = "pwm_hdmi",
[FB_LCD] = "pwm_lcd",
[FB_LCD2] = "pwm_lcd2",
[FB_LVDS] = "pwm_lvds",
[FB_LVDS2] = "pwm_lvds2",
[FB_MIPI] = "pwm_mipi",
[FB_MIPI2] = "pwm_mipi2",
};

static const char *const short_names[] = {
[FB_HDMI] = "hdmi",
[FB_LCD] = "lcd",
[FB_LCD2] = "lcd2",
[FB_LVDS] = "lvds",
[FB_LVDS2] = "lvds2",
[FB_MIPI] = "mipi",
[FB_MIPI2] = "mipi2",
};

static const char *new_aliases[FB_COUNT][2];

static const char *const aliases[] = {
[FBTS_NONE] = NULL,
[FBTS_ATMEL_MT] = "ts_atmel_mt",
[FBTS_CRTOUCH] = "ts_crtouch",
[FBTS_CYTTSP5] = "ts_cyttsp5",
[FBTS_EGALAX] = "ts_egalax",
[FBTS_EXC3000] = "ts_exc3000",
[FBTS_FT5X06] = "ts_ft5x06",
[FBTS_FT5X06_2] = "ts_ft5x06_2",
[FBTS_FT5X06_3] = "ts_ft5x06_3",
[FBTS_FT7250] = "ts_ft7250",
[FBTS_FUSION7] = "ts_fusion7",
[FBTS_GOODIX] = "ts_goodix",
[FBTS_GOODIX2] = "ts_goodix2",
[FBTS_GOODIX3] = "ts_goodix3",
[FBTS_GSL1680] = "ts_gsl1680",
[FBTS_ILI210X] = "ts_ili210x",
[FBTS_ILI251X] = "ts_ili251x",
[FBTS_ST1633I] = "ts_st1633i",
[FBTS_TSC2004] = "ts_tsc2004",
[FBTS_LVDS_ATMEL_MT] = "ts_lvds_atmel_mt",
[FBTS_LVDS_EGALAX] = "ts_lvds_egalax",
[FBTS_LVDS_FT5X06] = "ts_lvds_ft5x06",
[FBTS_LVDS_FT5X06_2] = "ts_lvds_ft5x06_2",
[FBTS_LVDS_GOODIX] = "ts_lvds_goodix",
[FBTS_LVDS_GOODIX2] = "ts_lvds_goodix2",
[FBTS_LVDS_ILI251X] = "ts_lvds_ili251x",
[FBP_MIPI_TO_LVDS] = "mipi_to_lvds",
[FBP_LT8912] = "lt8912",
[FBP_LT8912_2] = "lt8912_2",
[FBP_SPI_LCD] = "spi_lcd",
[FBP_PCA9540] = "pca9540",
[FBP_PCA9540_2] = "pca9540_2",
[FBP_PCA9546] = "pca9546",
[FBP_BACKLIGHT_MIPI_ALT] = "backlight_mipi_alt",
[FBP_PWM_MIPI_ALT] = "pwm_mipi_alt",
};

static const char *const timings_properties[] = {
"clock-frequency",
"hactive",
"vactive",
"hback-porch",
"hfront-porch",
"vback-porch",
"vfront-porch",
"hsync-len",
"vsync-len",
};

static const int timings_offsets[] = {
	offsetof(struct fb_videomode_f, pixclock_f),
	offsetof(struct fb_videomode_f, xres),
	offsetof(struct fb_videomode_f, yres),
	offsetof(struct fb_videomode_f, left_margin),
	offsetof(struct fb_videomode_f, right_margin),
	offsetof(struct fb_videomode_f, upper_margin),
	offsetof(struct fb_videomode_f, lower_margin),
	offsetof(struct fb_videomode_f, hsync_len),
	offsetof(struct fb_videomode_f, vsync_len),
};

static void __board_pre_enable(const struct display_info_t *di)
{
}

static void __board_enable_hdmi(const struct display_info_t *di, int enable)
{
}

static void __board_enable_lcd(const struct display_info_t *di, int enable)
{
}

static void __board_enable_lvds(const struct display_info_t *di, int enable)
{
}

static void __board_enable_lvds2(const struct display_info_t *di, int enable)
{
}

static void __board_enable_mipi(const struct display_info_t *di, int enable)
{
}

static void __board_enable_mipi2(const struct display_info_t *di, int enable)
{
}

void board_pre_enable(const struct display_info_t *di)
	__attribute__((weak, alias("__board_pre_enable")));
void board_enable_hdmi(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_hdmi")));
void board_enable_lcd(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lcd")));
void board_enable_lvds(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lvds")));
void board_enable_lvds2(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lvds2")));
void board_enable_mipi(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_mipi")));
void board_enable_mipi2(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_mipi2")));

static unsigned get_fb_available_mask(const struct display_info_t *di, int cnt)
{
	unsigned mask = 0;
	int i;

	for (i = 0; i < cnt; i++, di++)
		mask |= (1 << di->fbtype);

	return mask;
}
static const char bgr24[] = "BGR24";
static const char rgb24[] = "RGB24";
static const char rgb565[] = "RGB565";
static const char rgb666[] = "RGB666";
static const char yuyv16[] = "YUYV16";
static const char str_m101nwwb[] = "m101nwwb";

static char lvds_enabled;
static const char brightness_levels_low_active[] = "<10 9 8 7 6 5 4 3 2 1 0>";
static const char brightness_levels_high_active[] = "<0 1 2 3 4 5 6 7 8 9 10>";

#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP) || defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP)
#else
static struct fb_videomode g_vmode;

static u32 freq_to_period(u32 val)
{
	u64 lval = 1000000000000ULL;

	if (val < (233 * 16)) {
		val = 233 * 16;
		printf("invalid pixel clock frequency\n");
	}
	do_div(lval, val);
	return (u32)lval;
}
#endif

static const char *get_alias(const struct display_info_t *di, unsigned fb, int i)
{
	unsigned a = di->enable_alias[i];

	if (a < ARRAY_SIZE(aliases))
		return aliases[a];
	return new_aliases[fb][i];
}

#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
void *heap;
int heap_remaining;

static void *get_space(int len)
{
	void *p = heap;

	len = (len + 3) & ~0x3;
	if (heap_remaining < len) {
		p = malloc(len * 256);
		if (!p)
			return p;
		heap_remaining = len * 256;
	}
	heap_remaining -= len;
	heap = p + len;
	return p;
}

ofnode ofnode_path_err(const char *path)
{
	ofnode node = ofnode_path(path);

	if (!ofnode_valid(node))
		printf("missing node %s\n", path);
	return node;
}
#endif


static void set_local_property_u32(const char *path, const char *propname,
		u32 val)
{
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	u32 old_val = 0;
	int ret;
	ofnode node = ofnode_path_err(path);
	__be32 *pvalue;

	if (!ofnode_valid(node))
		return;
	ret = ofnode_read_u32(node, propname, &old_val);
	if (!ret && (old_val == val))
		return;

	debug("%s:old %u, new %u\n", __func__, old_val, val);
	pvalue = get_space(sizeof(u32));;
	if (!pvalue)
		return;
	*pvalue = cpu_to_be32(val);

	ofnode_write_prop(node, propname, sizeof(u32), pvalue);
#endif
}

static int set_property_u32(char *buf, int size, const char *path,
		const char *propname, u32 val)
{
	int sz = snprintf(buf, size, "fdt set %s %s <%u>;", path,
			propname, val);
	set_local_property_u32(path, propname, val);
	return sz;
}

static int set_property_joined_u32(char *buf, int size, const char *path1, const char *path2,
		const char *propname, u32 val)
{
	char cpath[80];

	snprintf(cpath, sizeof(cpath), "%s%s", path1, path2);
	return set_property_u32(buf, size, cpath, propname, val);
}

static int set_cmd_property(const struct display_info_t *di, char *buf, int size,
		const char *mode_name, const char *propname, u32 val)
{
	if (di->fbflags & FBF_MIPI_CMDS) {
		return set_property_joined_u32(buf, size,
				"mipi_cmds_", mode_name,
				propname, val);
	} else {
		return set_property_u32(buf, size, "mipi",
				propname, val);
	}
}

static int set_property_u32_env(char *buf, int size, const char *path,
		const char *propname, u32 val, const char *env)
{
	int sz = snprintf(buf, size, "fdt set %s %s <${%s}>;", path,
			propname, env);
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	u32 old_val = 0;
	int ret;
	ofnode node = ofnode_path_err(path);
	__be32 *pvalue;

	if (!ofnode_valid(node))
		return sz;
	ret = ofnode_read_u32(node, propname, &old_val);
	if (!ret && (old_val == val))
		return sz;

	debug("%s:old %u, new %u\n", __func__, old_val, val);
	pvalue = get_space(sizeof(u32));;
	if (!pvalue)
		return sz;
	*pvalue = cpu_to_be32(val);

	ofnode_write_prop(node, propname, sizeof(u32), pvalue);
#endif
	return sz;
}

static int set_property_u32_env_parms(char *buf, int size, const char *path,
		const char *propname, u32 val, const char *env, u32 p1, u32 p2, u32 p3)
{
	int sz;
	int cnt;
	char parm_buf[36];

	if (p3 != 0xffffffff) {
		snprintf(parm_buf, sizeof(parm_buf), " %u %u %u", p1, p2, p3);
		cnt = 4;
	} else if (p2 != 0xffffffff) {
		snprintf(parm_buf, sizeof(parm_buf), " %u %u", p1, p2);
		cnt = 3;
	} else if (p1 != 0xffffffff) {
		snprintf(parm_buf, sizeof(parm_buf), " %u", p1);
		cnt = 2;
	} else {
		parm_buf[0] = 0;
		cnt = 1;
	}
	sz = snprintf(buf, size, "fdt set %s %s <${%s}%s>;", path,
			propname, env, parm_buf);
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	u32 old[] = {0, 0, 0, 0};
	u32 new[] = {val, p1, p2, p3};
	int space;
	int ret;
	int i;
	ofnode node = ofnode_path_err(path);
	__be32 *pvalue;

	if (!ofnode_valid(node))
		return sz;
	ret = ofnode_read_u32_array(node, propname, old, cnt);
	if (!ret) {
		i = 0;
		while (old[i] == new[i]) {
			i++;
			if (i >= cnt)
				return sz;
		}
	}

	debug("%s:old %u %u %u %u, new %u %u %u %u\n", __func__, old[0], old[1], old[2], old[3], val, p1, p2, p3);
	space = sizeof(u32) * cnt;
	pvalue = get_space(space);
	if (!pvalue)
		return sz;
	for (i = 0; i < cnt; i++)
		pvalue[i] = cpu_to_be32(new[i]);

	ofnode_write_prop(node, propname, space, pvalue);
#endif
	return sz;
}

#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
int process_values(const char *s, fdt32_t *dst, int cnt)
{
	int found = 0;

	while (*s) {
		unsigned long tmp;
		const char *start;
		char *end;

		while (*s == ' ')
			s++;
		if (*s == '>')
			break;
		start = s;
		tmp = simple_strtoul(s, &end, 0);
		s = end;
		if (dst) {
			if (found < cnt)
				*dst++ = cpu_to_fdt32(tmp);
		}
		/* If the ptr didn't advance, something went wrong */
		if (s <= start)
			break;
		found++;
	}
	return found;
}
#endif

static int set_property_str(char *buf, int size, const char *path,
		const char *propname, const char *val)
{
	int sz = snprintf(buf, size, "fdt set %s %s %s;", path, propname, val);
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	ofnode node = ofnode_path_err(path);

	if (!ofnode_valid(node))
		return sz;
	if (val[0] == '<') {
		int cnt;
		__be32 *pvalue;

		val++;
		cnt = process_values(val, NULL, 0);
		if (cnt) {
			pvalue = get_space(sizeof(u32) * cnt);
			process_values(val, pvalue, cnt);
			ofnode_write_prop(node, propname, sizeof(u32) * cnt, pvalue);
		}
	} else {
		ofnode_write_string(node, propname, val);
	}
#endif
	return sz;
}

static int set_property(char *buf, int size, const char *path, const char *propname)
{
	int sz = snprintf(buf, size, "fdt set %s %s;", path, propname);
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	ofnode node = ofnode_path_err(path);

	if (!ofnode_valid(node))
		return sz;
	ofnode_write_prop(node, propname, 0, NULL);
#endif
	return sz;
}

static int set_property_set_rm(char *buf, int size, const char *path, const char *propname, bool set)
{
	int sz;

	if (set)
		return set_property(buf, size, path, propname);
	sz = snprintf(buf, size, "fdt rm %s %s;", path, propname);
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	ofnode node = ofnode_path_err(path);

	if (!ofnode_valid(node))
		return sz;
	ofnode_remove_prop(node, propname);
#endif
	return sz;
}

static int set_status(char *buf, int size, const char *path, bool enable)
{
	int sz = snprintf(buf, size, "fdt set %s status %s;", path,
			enable ? "okay" : "disabled");
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	ofnode node = ofnode_path_err(path);
	int ret;

	if (!ofnode_valid(node))
		return sz;
	ret = ofnode_set_enabled(node, enable);
	if (ret) {
		printf("status change failed for %s\n", path);
	} else if (enable) {
		ofnode pnode = ofnode_get_parent(node);
		struct udevice *parent;
		struct udevice *dev;

		ret = device_find_by_ofnode(node, &dev);
		debug("%s: %s: device_find_by_ofnode=%d\n", __func__,
				ofnode_get_name(node), ret);
		if (ret) {
			/* not device yet */
			ret = device_find_by_ofnode(pnode, &parent);
			debug("%s: parent - %s: device_find_by_ofnode=%d\n",
				__func__, ofnode_get_name(pnode), ret);
			if (!ret)
				lists_bind_fdt(parent, node, NULL, NULL, false);
		}
	}
#endif
	return sz;
}

static int set_status_indexed(char *buf, int size, const char *path, int index, bool enable)
{
	char cpath[80];

	snprintf(cpath, sizeof(cpath), "%s%d", path, index);
	return set_status(buf, size, cpath, enable);
}

static int get_value_joined_str(char *buf, int size, u32* pval,
		const char *env, const char *path1, const char *path2,
		const char* propname)
{
	int sz = snprintf(buf, size, "fdt get value %s %s%s %s;", env,
			path1, path2, propname);
#if CONFIG_IS_ENABLED(DM_VIDEO) && CONFIG_IS_ENABLED(OF_LIVE)
	char cpath[80];
	ofnode node;

	snprintf(cpath, sizeof(cpath), "%s%s", path1, path2);
	node = ofnode_path_err(cpath);
	if (!ofnode_valid(node)) {
		debug("%s: not found %s\n", __func__, cpath);
		return sz;
	}

	ofnode_read_u32(node, propname, pval);
#endif
	return sz;
}

static int get_value_path_indexed(char *buf, int size, u32* pval,
		const char *env, const char *path1, u32 index,
		const char* propname)
{
	char path2[16];

	snprintf(path2, sizeof(path2), "%u", index);
	return get_value_joined_str(buf, size, pval, env,
			path1, path2, propname);
}

static int set_property_gpio(char *buf, int size, const char *path, const char *propname, int i, int flags)
{
	u32 gp = 0;
	int sz = get_value_path_indexed(buf, size, &gp, "gp", "gpio",
			(i >> 5), "phandle");
	buf += sz;
	size -= sz;
	sz += set_property_u32_env_parms(buf, size, path,
			propname, gp, "gp", i & 0x1f, flags, -1);
	return sz;
}

static int scan_for_alias(const struct display_info_t *di, int alias)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(di->enable_alias); i++) {
		if (di->enable_alias[i] == alias)
			return 0;
	}
	return -ENOENT;
}

static void setup_cmd_fb(unsigned fb, const struct display_info_t *di, char *buf, int size)
{
	const char *mode_str = NULL;
	int i;
	int sz;
	int interface_width;
	const char *buf_start = buf;
	const struct fb_videomode_f *mode;
	const char * fmt;
	char mode_name[80];
	int mipi_alt_backlight = 0;
#if defined(CONFIG_IMX8M) || defined(CONFIG_IMX8ULP)
	unsigned fb1 = is_mipi(fb) ? FB_MIPI_BRIDGE : fb;
#else
	unsigned fb1 = fb;
#endif

	if (env_get("cmd_frozen") && env_get(cmd_fbnames[fb]))
		return;		/* don't override if already set */

	if (fb == FB_LVDS)
		lvds_enabled = 0;
	if (!di) {
		const char *name = env_get(fbnames[fb]);
		if (name) {
			if (name[0] == '*')
				name++;
			if (strcmp(name, "off")) {
				/* Not off and not in list, assume mode_str value */
				mode_str = name;
			}
		}
		if (!mode_str) {
#ifdef CONFIG_IMX8MP
			/*
			 * for 8MP fb_lvds == fb_lvds2, so skip for lvds2
			 * so that lvds can be on
			 */
			if (fb != FB_LVDS2)
#endif
			{
				sz = set_status(buf, size, fbnames[fb1], false);
				buf += sz;
				size -= sz;
			}
			if (is_mipi(fb)) {
				sz = set_status(buf, size, "lcdif", false);
			} else if (fb == FB_LCD) {
				sz = set_status(buf, size, "lcd", 0);
			} else if ((fb == FB_LVDS) || (fb == FB_LVDS2)) {
				sz = set_status_indexed(buf, size, "ldb/lvds-channel@", fb - FB_LVDS, 0);
			}
#if defined(CONFIG_VIDEO_IMX8M_DCSS)
			else if (fb == FB_HDMI) {
				sz = set_status(buf, size, "dcss", false);
				buf += sz;
				size -= sz;
				sz = set_status(buf, size, "sound_hdmi", false);
			}
#elif defined(CONFIG_VIDEO_IMX8MP_HDMI)
			else if (fb == FB_HDMI) {
				sz = set_status(buf, size, "sound_hdmi", false);
			}
#endif
			env_set(cmd_fbnames[fb], buf_start);
			env_set(fbnames_name[fb], "");
			return;
		}
	} else {
		i = 0;
		while (i < sizeof(mode_name) - 1) {
			mode_name[i] = di->mode.name[i];
			if (!mode_name[i])
				break;
			i++;
		}
		mode_name[i] = 0;
		while (i) {
			char c;

			i--;
			c = mode_name[i];
			if (((c >= '0') && (c <= '9')) || (c == 'd'))
				continue;
			if (c == '-')
				mode_name[i] = 0;
			break;
		}
		if (di->fbflags & FBF_MODESTR)
			mode_str = mode_name;
		if (strcmp(mode_name, "tm070jdhg30") == 0)
			strncpy(mode_name, str_m101nwwb, sizeof(str_m101nwwb));
	}

	if (fb == FB_LVDS)
		lvds_enabled = 1;

	if (di->fbflags & FBF_ALT_PWM) {
		sz = set_status(buf, size, backlight_names[fb], false);
		buf += sz;
		size -= sz;
		sz = set_status(buf, size, pwm_names[fb], false);
		buf += sz;
		size -= sz;
		sz = set_status(buf, size, backlight_alt, true);
		buf += sz;
		size -= sz;
	} else if (fb != FB_HDMI) {
		if (!is_mipi(fb) || scan_for_alias(di, FBP_BACKLIGHT_MIPI_ALT)) {
			sz = set_status(buf, size, backlight_names[fb], true);
			buf += sz;
			size -= sz;
		}
	}
	if (is_mipi(fb)) {
#if defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP)
		sz = set_status(buf, size, "mipi_dsi", true);
		buf += sz;
		size -= sz;
#endif
#if defined(CONFIG_IMX8M)
		sz = set_status(buf, size, "lcdif", true);
		buf += sz;
		size -= sz;
#if defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP)
		sz = set_status(buf, size, "mipi_dsi_phy", true);
		buf += sz;
		size -= sz;
#endif
#endif
		if (di) {
			if ((di->addr_num == 0x2c) &&
				(di->enable_alias[0] != FBP_MIPI_TO_LVDS)) {
				sz = set_status(buf, size, aliases[FBP_MIPI_TO_LVDS], true);
				buf += sz;
				size -= sz;
			}
		}
	}
	sz = set_status(buf, size, fbnames[fb1], true);
	buf += sz;
	size -= sz;
	if (fb != fb1) {
		sz = set_status(buf, size, fbnames[fb], true);
		buf += sz;
		size -= sz;
	}
	if ((fb == FB_LVDS) || (fb == FB_LVDS2)) {
		sz = set_status(buf, size, "ldb", true);
		buf += sz;
		size -= sz;

		sz = set_status(buf, size, ch_names[fb], true);
		buf += sz;
		size -= sz;
	}

	if ((fb == FB_LVDS2) && !lvds_enabled) {
		sz = set_property(buf, size, "ldb/lvds-channel@1", "primary");
		buf += sz;
		size -= sz;
	}

	if (!di)
		goto set_variables;

	for (i = 0; i < ARRAY_SIZE(di->enable_alias); i++) {
		const char *s = get_alias(di, fb, i);

		if (!s)
			continue;

		if (s == aliases[FBP_PWM_MIPI_ALT]) {
			/* disable the old, before enabling the new, in case they are the same */
			sz = set_status(buf, size, "pwm_mipi", false);
			buf += sz;
			size -= sz;
		}

		sz = set_status(buf, size, s, true);
		buf += sz;
		size -= sz;

		if (s == aliases[FBP_BACKLIGHT_MIPI_ALT]) {
			/* point fb_mipi to backligh_mipi2 */
			/* fdt get value blm2 backlight_mipi2 phandle;fdt set fb_mipi backlight <${blm2}> */
			u32 blm2 = 0;

			sz = get_value_joined_str(buf, size, &blm2, "blm2", s,
					"", "phandle");
			buf += sz;
			size -= sz;
			sz = set_property_u32_env(buf, size, fbnames[fb],
				"backlight", blm2, "blm2");
			buf += sz;
			size -= sz;
			mipi_alt_backlight = 1;
		}
	}

	interface_width = 18;
	if (di->pixfmt == IPU_PIX_FMT_RGB24) {
		fmt = rgb24;
		interface_width = 24;
	} else if (di->pixfmt == IPU_PIX_FMT_BGR24) {
		fmt = bgr24;
		interface_width = 24;
	} else if (di->pixfmt == IPU_PIX_FMT_YUYV) {
		fmt = yuyv16;
	} else if (di->pixfmt == IPU_PIX_FMT_RGB565) {
		fmt = rgb565;
	} else {
		fmt = rgb666;
	}

	if ((fb >= FB_LCD) && !is_mipi(fb)) {
#ifdef CONFIG_VIDEO_MXS
		sz = set_property_u32(buf, size, short_names[fb], "bus-width",
				interface_width);
#else
		sz = set_property_str(buf, size, fbnames[fb],
				"interface_pix_fmt", fmt);
#endif
		buf += sz;
		size -= sz;
	}

	if ((fb == FB_LCD) || (fb == FB_LCD2)) {
		sz = set_property_str(buf, size, short_names[fb],
				"default_ifmt", fmt);
		buf += sz;
		size -= sz;

		sz = set_status(buf, size, short_names[fb], true);
		buf += sz;
		size -= sz;
	}

	if ((fb == FB_LVDS) || (fb == FB_LVDS2)) {
		sz = set_property_u32(buf, size, ch_names[fb],
				"fsl,data-width", interface_width);
		buf += sz;
		size -= sz;

		sz = set_property_str(buf, size, ch_names[fb],
				"fsl,data-mapping",
				(di->fbflags & FBF_JEIDA) ? "jeida" : "spwg");
		buf += sz;
		size -= sz;

		if (di->fbflags & FBF_SPLITMODE) {
			sz = set_property_str(buf, size, "ldb",
					"split-mode", "1");
			buf += sz;
			size -= sz;
			sz = set_property_str(buf, size, "ldb",
					"fsl,dual-channel", "1");
			buf += sz;
			size -= sz;
		}
	} else if (is_mipi(fb)) {
		sz = set_property_str(buf, size, fbnames[fb], "dsi-format",
			(interface_width == 24) ? "rgb888" : "rgb666");
		buf += sz;
		size -= sz;

		if ((di->addr_num == 0x2c) || (di->fbflags & (FBF_JEIDA | FBF_SPLITMODE))) {
			if (interface_width == 24) {
				sz = set_property(buf, size, aliases[FBP_MIPI_TO_LVDS],
					(di->fbflags & FBF_JEIDA) ? "jeida" :
					"spwg");
				buf += sz;
				size -= sz;
			}
			if (di->fbflags & FBF_SPLITMODE) {
				sz = set_property(buf, size, aliases[FBP_MIPI_TO_LVDS],
						"split-mode");
				buf += sz;
				size -= sz;
			}
		}
	}

	if (di->fbflags & FBF_PINCTRL) {
		u32 pin = 0;

		sz = get_value_joined_str(buf, size, &pin, "pin", "pinctrl_",
				mode_name, "phandle");
		buf += sz;
		size -= sz;
		sz = set_property_u32_env(buf, size, fbnames[fb1],
			"pinctrl-0", pin, "pin");
		buf += sz;
		size -= sz;
		sz = set_property_str(buf, size, fbnames[fb1],
			"pinctrl-names", "default");
		buf += sz;
		size -= sz;
	}

	if (di->fbflags & FBF_SN65_ALT) {
		u32 alt = 0;

		sz = get_value_joined_str(buf, size, &alt, "alt", "mipi_to_lvds_alt",
				"", "phandle");
		buf += sz;
		size -= sz;
		sz = set_property_u32_env(buf, size, fbnames[fb],
			"sn65dsi83", alt, "alt");
		buf += sz;
		size -= sz;
	}

	if (di->fbflags & FBF_ENABLE_GPIOS_DTB) {
		u32 gp = 0;

		i = di->enable_gp;
		sz = get_value_path_indexed(buf, size, &gp, "gp", "gpio",
				(i >> 5), "phandle");
		buf += sz;
		size -= sz;
		sz = set_property_u32_env_parms(buf, size, fbnames[fb],
			"enable-gpios", gp, "gp", i & 0x1f,
			((di->fbflags & FBF_ENABLE_GPIOS_ACTIVE_LOW) ?
				GPIO_ACTIVE_LOW : GPIO_ACTIVE_HIGH) |
			((di->fbflags & FBF_ENABLE_GPIOS_OPEN_DRAIN) ?
				GPIO_OPEN_DRAIN : 0), -1);
		buf += sz;
		size -= sz;
	}

	i = di->reset_gp;
	if (i) {
		sz = set_property_gpio(buf, size, fbnames[fb], "reset-gpios", i, GPIO_ACTIVE_LOW);
		buf += sz;
		size -= sz;
	}

	if (is_mipi(fb) || (di->fbflags & FBF_MODE_VIDEO)) {
		char mipi_cmd[90];
		char *p;

		sz = set_property_set_rm(buf, size, "mipi", "mode-skip-eot",
				(di->fbflags & FBF_MODE_SKIP_EOT));
		buf += sz;
		size -= sz;

		if (di->fbflags & FBF_MIPI_CMDS) {
			snprintf(mipi_cmd, sizeof(mipi_cmd) - 1, "mipi_cmds_%s",
				mode_name);
			p = mipi_cmd;
		} else {
			p = "mipi";
		}

		if (di->fbflags & FBF_DSI_HBP) {
			sz = set_property_set_rm(buf, size, p,
				"mode-video-hbp-disable",
				(di->fbflags & FBF_MODE_VIDEO_BURST));
			buf += sz;
			size -= sz;
		}
		if (di->fbflags & FBF_DSI_HFP) {
			sz = set_property_set_rm(buf, size, p,
				"mode-video-hfp-disable",
				(di->fbflags & FBF_MODE_VIDEO_BURST));
			buf += sz;
			size -= sz;
		}

		if (di->fbflags & FBF_DSI_HSA) {
			sz = set_property_set_rm(buf, size, p,
				"mode-video-hsa-disable",
				(di->fbflags & FBF_MODE_VIDEO_BURST));
			buf += sz;
			size -= sz;
		}

		sz = set_property_set_rm(buf, size, "mipi", "mode-video",
				(di->fbflags & FBF_MODE_VIDEO));
		buf += sz;
		size -= sz;

		sz = set_property_set_rm(buf, size, "mipi", "mode-video-burst",
				(di->fbflags & FBF_MODE_VIDEO_BURST));
		buf += sz;
		size -= sz;

		if (di->fbflags & FBF_MODE_VIDEO_MBC) {
			sz = set_property(buf, size, "mipi", "mode-video-mbc");
			buf += sz;
			size -= sz;
		}
		if (di->fbflags & FBF_MODE_VIDEO_SYNC_PULSE) {
			sz = set_property(buf, size, "mipi",
					"mode-video-sync-pulse");
			buf += sz;
			size -= sz;
		}
		if (di->fbflags & FBF_MIPI_CMDS) {
			u32 cmds;

			sz = get_value_joined_str(buf, size, &cmds, "cmds", "mipi_cmds_",
					mode_name, "phandle");
			buf += sz;
			size -= sz;
			sz = set_property_u32_env(buf, size, "mipi",
				"mipi-cmds", cmds, "cmds");
			buf += sz;
			size -= sz;
		}

		sz = set_property_u32(buf, size, "mipi", "dsi-lanes",
				(di->fbflags >> FBF_DSI_LANE_SHIFT) & 7);
		buf += sz;
		size -= sz;
	}

	if (di->pwm_period) {
		u32 pwm = 0;

		i = di->enable_gp;
		sz = get_value_joined_str(buf, size, &pwm, "pwm",
				mipi_alt_backlight ? aliases[FBP_PWM_MIPI_ALT] : pwm_names[fb],
				"", "phandle");
		buf += sz;
		size -= sz;
		sz = set_property_u32_env_parms(buf, size, mipi_alt_backlight ? aliases[FBP_BACKLIGHT_MIPI_ALT] : backlight_names[fb],
			"pwms", pwm, "pwm", 0, di->pwm_period, 0);
		buf += sz;
		size -= sz;
	}
	if ((di->fbflags & FBF_BKLIT_EN_DTB) && di->backlight_en_gp) {
		sz = set_property_gpio(buf, size, backlight_names[fb],
				"enable-gpios", di->backlight_en_gp,
				(di->fbflags & FBF_BKLIT_EN_LOW_ACTIVE) ?
				GPIO_ACTIVE_LOW : GPIO_ACTIVE_HIGH);
		buf += sz;
		size -= sz;
	}
	if (di->min_hs_clock_multiple) {
		sz = set_cmd_property(di, buf, size, mode_name,
			"min-hs-clock-multiple", di->min_hs_clock_multiple);
		buf += sz;
		size -= sz;
	}
	if (di->enable_high_duration_us) {
		sz = set_cmd_property(di, buf, size, mode_name,
			"enable-high-duration-us", di->enable_high_duration_us);
		buf += sz;
		size -= sz;
	}
	if (di->enable_low_duration_us) {
		sz = set_cmd_property(di, buf, size, mode_name,
			"enable-low-duration-us", di->enable_low_duration_us);
		buf += sz;
		size -= sz;
	}
	if (di->power_gp) {
		sz = set_property_gpio(buf, size, fbnames[fb], "power-gpios",
			di->power_gp, GPIO_ACTIVE_HIGH);
		buf += sz;
		size -= sz;
	}
	if (di->power_delay_ms) {
		sz = set_cmd_property(di, buf, size, mode_name,
			"power-delay-ms", di->power_delay_ms);
		buf += sz;
		size -= sz;
	}
	if (di->fbflags & FBF_BKLIT_DTB) {
		sz = set_property_str(buf, size, backlight_names[fb], "brightness-levels",
			(di->fbflags & FBF_BKLIT_LOW_ACTIVE) ?
				brightness_levels_low_active :
				brightness_levels_high_active);
		buf += sz;
		size -= sz;
	}

	if (mode_str && (fb != FB_HDMI))
		goto set_variables;

	mode = &di->mode;
	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *p = (u32 *)((char *)mode + timings_offsets[i]);
		u32 val;

		val = *p;
		if (!mode_str) {
			sz = set_property_u32(buf, size, timings_names[fb], timings_properties[i], val);
			buf += sz;
			size -= sz;
		} else {
			set_local_property_u32(timings_names[fb], timings_properties[i], val);
		}
	}
set_variables:
	if (mode_str)
		set_property_str(buf, size, fbnames[fb], "mode_str", mode_str);
	env_set(cmd_fbnames[fb], buf_start);
	env_set(fbnames_name[fb], di ? di->mode.name : mode_str);
}

static const struct display_info_t *find_panel(const struct display_info_t *di, int cnt, unsigned fb, const char *name)
{
	int i;

	for (i = 0; i < cnt; i++, di++) {
		if ((fb == di->fbtype) && !strcmp(name, di->mode.name))
			return di;
	}
	return NULL;
}

static char g_mode_str[FB_COUNT][80];
static struct display_info_t g_di_temp[FB_COUNT];

int fbp_detect_i2c(struct display_info_t const *di)
{
	int ret;
#ifdef CONFIG_DM_I2C
	struct udevice *bus;
#endif
#ifdef CONFIG_DM_VIDEO
	int ret_request;
#endif

#ifdef CONFIG_DM_VIDEO
	if (di->bus_gp) {
		ret_request = gpio_request(di->bus_gp, "bus_gp");
	}
#endif
	if (di->bus_gp) {
		gpio_direction_output(di->bus_gp, 1);
		if (di->bus_gp_delay_ms)
			mdelay(di->bus_gp_delay_ms);
	}
#ifdef CONFIG_DM_I2C
	ret = uclass_get_device_by_seq(UCLASS_I2C, di->bus_num, &bus);
	if (!ret) {
		ret = i2c_probe_chip(bus, di->addr_num, 0);
		if (ret && (di->addr_num == 0x4a)) {
			/* atmel's address can be 0x26 the 1st time */
			ret = i2c_probe_chip(bus, 0x26, 0);
		}
	}
#else
	ret = i2c_set_bus_num(di->bus_num);
	if (!ret) {
		ret = i2c_probe(di->addr_num);
		if (ret && (di->addr_num == 0x4a)) {
			/* atmel's address can be 0x26 the 1st time */
			ret = i2c_probe(0x26);
		}
	}
#endif

	if (di->bus_gp)
		gpio_set_value(di->bus_gp, 0);
#ifdef CONFIG_DM_VIDEO
	if (di->bus_gp && !ret_request) {
		gpio_free(di->bus_gp);
	}
#endif
	return (ret == 0);
}

#if !defined(CONFIG_IMX8M) && !defined(CONFIG_IMX8ULP) && !defined(CONFIG_MX51) && !defined(CONFIG_MX53)
static int calc_gcd(int a, int b)
{
	int n;

	if (!a)
		return b;
	while (b) {
		if (a > b) {
			n = a;
			a = b;
			b = n;
		}
		n = b / a;
		b -= n * a;
	}
	return a;
}
#endif

#ifdef LVDS_SUPPORT
#ifdef CONFIG_MX6SX
static void reparent_lvds(int fbtype, int new_parent)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int ldb_sel_shift = 9;
	u32 reg = readl(&ccm->cs2cdr);

	reg &= ~(7 << ldb_sel_shift);

	reg |= new_parent << ldb_sel_shift;
	writel(reg, &ccm->cs2cdr);
	readl(&ccm->cs2cdr);	/* wait for write */
}
#else
static void reparent_lvds(int fbtype, int new_parent)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int ldb_sel_shift = (fbtype == FB_LVDS2) ? 12 : 9;
	u32 reg;

	reg = readl(&ccm->cbcmr);
	reg &= ~MXC_CCM_CBCMR_PERIPH2_CLK2_SEL;
	writel(reg, &ccm->cbcmr);

	/* Set MMDC_CH1 mask bit */
	reg = readl(&ccm->ccdr);
	reg |= MXC_CCM_CCDR_MMDC_CH1_HS_MASK;
	writel(reg, &ccm->ccdr);

	/*
	 * Set the periph2_clk_sel to the top mux so that
	 * mmdc_ch1 is from pll3_sw_clk.
	 */
	reg = readl(&ccm->cbcdr);
	reg |= MXC_CCM_CBCDR_PERIPH2_CLK_SEL;
	writel(reg, &ccm->cbcdr);

	/* Wait for the clock switch */
	while (readl(&ccm->cdhipr) != 0) {
		udelay(100);
	}

	/* Disable pll3_sw_clk by selecting the bypass clock source */
	reg = readl(&ccm->ccsr);
	reg |= MXC_CCM_CCSR_PLL3_SW_CLK_SEL;
	writel(reg, &ccm->ccsr);

	/* Set the ldb_di0/1_clk to 1xxb, to change to lower mux */
	reg = readl(&ccm->cs2cdr);
	reg |= (4 << ldb_sel_shift);
	writel(reg, &ccm->cs2cdr);
	readl(&ccm->cs2cdr);	/* wait for write */

	/* Set the ldb_di0/1_clk to 100b, change upper mux to pll5 */
	reg &= ~(3 << ldb_sel_shift);
	reg |= new_parent << ldb_sel_shift;
	writel(reg, &ccm->cs2cdr);
	readl(&ccm->cs2cdr);	/* wait for write */

	/* select upper mux, pll 5 clock */
	reg &= ~(4 << ldb_sel_shift);
	writel(reg, &ccm->cs2cdr);
	readl(&ccm->cs2cdr);	/* wait for write */

	/* Unbypass pll3_sw_clk */
	reg = readl(&ccm->ccsr);
	reg &= ~MXC_CCM_CCSR_PLL3_SW_CLK_SEL;
	writel(reg, &ccm->ccsr);

	/*
	 * Set the periph2_clk_sel back to the bottom mux so that
	 * mmdc_ch1 is from its original parent.
	 */
	reg = readl(&ccm->cbcdr);
	reg &= ~MXC_CCM_CBCDR_PERIPH2_CLK_SEL;
	writel(reg, &ccm->cbcdr);

	/* Wait for the clock switch */
	while (readl(&ccm->cdhipr)) {
		udelay(100);
	}

	/* Clear MMDC_CH1 mask bit */
	reg = readl(&ccm->ccdr);
	reg &= ~MXC_CCM_CCDR_MMDC_CH1_HS_MASK;
	writel(reg, &ccm->ccdr);
}
#endif
#endif

#if defined(CONFIG_MX51) || defined(CONFIG_MX53)
int get_pll3_clock(void);

static void setup_clock(struct display_info_t const *di)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 desired_freq, freq;
	u32 pll3_freq = get_pll3_clock();
	u32 n, m;
	u32 i;
	u32 best = 8;
	u32 best_diff = ~0;
	u32 diff;
	u32 reg;

	if (di->fbflags & FBF_USE_IPU_CLOCK)
		return;

	desired_freq = di->mode.pixclock_f * 16;

	for (i = 8; i > 0 ; i--) {
		n = pll3_freq / i;
		m = n / desired_freq;
		if (!m || (m & 1))
			continue;
		if (m > 16)
			break;
		freq = n / m;
		if (freq >= desired_freq)
			diff = freq - desired_freq;
		else
			diff = desired_freq - freq;

		if (best_diff > diff) {
			best_diff = diff;
			best = i;
		}
	}
	reg = readl(&ccm->cdcdr);
	reg &= ~(7 << 6);
	reg |= (best - 1) << 6;
	writel(reg, &ccm->cdcdr);

#ifdef CONFIG_VIDEO_IPUV3
	ipu_set_ldb_clock(pll3_freq / best);	/* used for all ext clocks */
#endif
}

#elif !defined(CONFIG_IMX8M) && !defined(CONFIG_IMX8ULP)
static void setup_clock(struct display_info_t const *di)
{
#if defined(CONFIG_MX7D)
	struct mxc_ccm_anatop_reg *ccm_anatop = (struct mxc_ccm_anatop_reg *)
						 ANATOP_BASE_ADDR;
	u32 target;
#else
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 reg;
	u32 pll_video;
#ifdef CONFIG_VIDEO_IPUV3
	u32 out_freq;
#endif
#endif
	u32 desired_freq;
	int post_div = 2;	/* 0: /4, 1: /2, 2: /1 */
	int misc2_div = 0;	/* 0: /1, 1: /2, 3: /4 */
	int lvds = ((di->fbtype == FB_LVDS) | (di->fbtype == FB_LVDS2)) ? 1 : 0;
	int timeout = 1000;
	int multiplier;
	int gcd;
	int num, denom;
	int ipu_div = 7;
	int x = 1;

	if (di->fbflags & FBF_USE_IPU_CLOCK)
		return;

#if defined(CONFIG_MX6SX)
	clrbits_le32(&ccm->CCGR2, MXC_CCM_CCGR2_LCD_MASK);
	/* gate LCDIF2/LCDIF1/LDB_DI0 */
	clrbits_le32(&ccm->CCGR3, 0x3f << 8);
#elif defined(CONFIG_MX6ULL)
	clrbits_le32(&ccm->CCGR2, MXC_CCM_CCGR2_LCD_MASK);
	/* gate lcdif1 pix clock */
	clrbits_le32(&ccm->CCGR3, 0x3 << 10);
#elif defined(CONFIG_MX7D)
	clock_enable(CCGR_LCDIF, 0);
#else
	/* gate ipu1_di0_clk */
	clrbits_le32(&ccm->CCGR3, MXC_CCM_CCGR3_LDB_DI0_MASK |
			MXC_CCM_CCGR3_LDB_DI1_MASK |
			MXC_CCM_CCGR3_IPU1_IPU_DI0_MASK);
#endif

#if defined(CONFIG_MX7D)
	/* Power up video PLL and disable its output */
	writel(CCM_ANALOG_PLL_VIDEO_CLR_ENABLE_CLK_MASK |
		CCM_ANALOG_PLL_VIDEO_CLR_POWERDOWN_MASK |
		CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_MASK |
		CCM_ANALOG_PLL_VIDEO_CLR_DIV_SELECT_MASK |
		CCM_ANALOG_PLL_VIDEO_CLR_POST_DIV_SEL_MASK |
		CCM_ANALOG_PLL_VIDEO_CLR_TEST_DIV_SELECT_MASK,
		&ccm_anatop->pll_video_clr);
#else
	pll_video = readl(&ccm->analog_pll_video);
	pll_video &= BM_ANADIG_PLL_VIDEO_ENABLE;
	pll_video |= BM_ANADIG_PLL_VIDEO_POWERDOWN;
	writel(pll_video, &ccm->analog_pll_video);
#endif

	desired_freq = di->mode.pixclock_f;
	pr_debug("desired_freq=%d\n", desired_freq);
#ifdef CONFIG_VIDEO_IPUV3
	out_freq = desired_freq;
#endif

#ifdef LVDS_SUPPORT
	if (lvds) {
		reparent_lvds(di->fbtype, 0);
		desired_freq *= 7;
		if (di->fbflags & FBF_SPLITMODE)
			desired_freq >>= 1;

	}
#endif
	pr_debug("desired_freq=%d\n", desired_freq);

#define MIN_FREQ 648000000	/* 24000000 * 27 */

	if (!(is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D)) ||
			soc_rev() > CHIP_REV_1_0) {
		while (desired_freq < MIN_FREQ) {
			desired_freq <<= 1;
			if (--post_div <= 0)
				break;
		}
		pr_debug("desired_freq=%d\n", desired_freq);

		while (desired_freq < MIN_FREQ) {
			desired_freq <<= 1;
			misc2_div = (misc2_div << 1) + 1;
			if (misc2_div >= 3)
				break;
		}
		pr_debug("desired_freq=%d\n", desired_freq);
	};

	if (!lvds) {
		num = (MIN_FREQ - 1) / desired_freq + 1;
		x = (num + 7) / 8;
		ipu_div =  (num - 1) / x + 1;
		desired_freq *= ipu_div * x;
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6ULL)
		if (x > 8) {
			printf("div changed from %d to 64\n", num);
			ipu_div = 8;
			x = 8;
			desired_freq = MIN_FREQ;
		}
#endif
	} else if (desired_freq < MIN_FREQ) {
		printf("desired_freq=%d is too low\n", desired_freq);
	}
#ifdef CONFIG_VIDEO_IPUV3
	ipu_set_ldb_clock(out_freq * x);	/* used for all ext clocks */
#endif
	pr_debug("desired_freq=%d ipu div=%d x=%d\n", desired_freq, ipu_div, x);

	multiplier = desired_freq / 24000000;
	if (multiplier > 54) {
		multiplier = 54;
		desired_freq = 24000000 * 54;
	}

#if defined(CONFIG_MX7D)
	writel(CCM_ANALOG_PLL_VIDEO_SET_DIV_SELECT(multiplier) |
		CCM_ANALOG_PLL_VIDEO_SET_TEST_DIV_SELECT(post_div) |
		CCM_ANALOG_PLL_VIDEO_SET_POST_DIV_SEL(misc2_div),
		&ccm_anatop->pll_video_set);
#else
	reg = readl(&ccm->pmu_misc2);
	reg &= ~(BF_PMU_MISC2_VIDEO_DIV(3));
	reg |= BF_PMU_MISC2_VIDEO_DIV(misc2_div);
	writel(reg, &ccm->pmu_misc2);

	pll_video &= ~(BM_ANADIG_PLL_VIDEO_DIV_SELECT |
		 BM_ANADIG_PLL_VIDEO_POST_DIV_SELECT);
	pll_video |= BF_ANADIG_PLL_VIDEO_DIV_SELECT(multiplier) |
		BF_ANADIG_PLL_VIDEO_POST_DIV_SELECT(post_div);
	writel(pll_video, &ccm->analog_pll_video);
#endif
	desired_freq -= multiplier * 24000000;
	gcd = calc_gcd(desired_freq, 24000000);
	pr_debug("gcd=%d desired_freq=%d 24000000\n", gcd, desired_freq);
	num = desired_freq / gcd;
	denom = 24000000 / gcd;
	pr_debug("desired_freq=%d multiplier=%d %d/%d\n", desired_freq, multiplier, num, denom);

#if defined(CONFIG_MX7D)
	writel(num, &ccm_anatop->pll_video_num);
	writel(denom, &ccm_anatop->pll_video_denom);

	while (!(readl(&ccm_anatop->pll_video) & CCM_ANALOG_PLL_VIDEO_LOCK_MASK)) {
		udelay(100);
		if (--timeout < 0) {
			printf("Warning: video pll lock timeout!\n");
			break;
		}
	}
#else
	writel(num, &ccm->analog_pll_video_num);
	writel(denom, &ccm->analog_pll_video_denom);

	pll_video &= ~BM_ANADIG_PLL_VIDEO_POWERDOWN;
	writel(pll_video, &ccm->analog_pll_video);

	while (!(readl(&ccm->analog_pll_video) & BM_ANADIG_PLL_VIDEO_LOCK)) {
		udelay(100);
		if (--timeout < 0) {
			printf("Warning: video pll lock timeout!\n");
			break;
		}
	}

#endif

#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6ULL)
	/* Select pll5 clock for ldb di0 */
	clrbits_le32(&ccm->cs2cdr, MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK);

	/* lvds:
	 *   lcdif1 from ldb_di0
	 * lcd:
	 *   lcdif1 from pre-muxed lcdif1 clock / ipu_div
	 *   pre-muxed from PLL5
	 */
	clrsetbits_le32(&ccm->cscdr2,
		(MXC_CCM_CSCDR2_LCDIF1_CLK_SEL_MASK |
		MXC_CCM_CSCDR2_LCDIF1_PRE_DIV_MASK |
		MXC_CCM_CSCDR2_LCDIF1_PRED_SEL_MASK),
		(((lvds ? 3 : 0) << MXC_CCM_CSCDR2_LCDIF1_CLK_SEL_OFFSET) |
		((ipu_div - 1) << MXC_CCM_CSCDR2_LCDIF1_PRE_DIV_OFFSET) |
		(2 << MXC_CCM_CSCDR2_LCDIF1_PRED_SEL_OFFSET)));

	/* Set the post divider */
	clrsetbits_le32(&ccm->cbcmr,
			MXC_CCM_CBCMR_LCDIF1_PODF_MASK,
			((x - 1) <<
			MXC_CCM_CBCMR_LCDIF1_PODF_OFFSET));

	setbits_le32(&ccm->CCGR2, MXC_CCM_CCGR2_LCD_MASK);
	/* enable lcdif1/ ldb_di0 (if lvds)*/
	setbits_le32(&ccm->CCGR3, MXC_CCM_CCGR3_LCDIF1_PIX_MASK |
		MXC_CCM_CCGR3_DISP_AXI_MASK |
		(lvds ? MXC_CCM_CCGR3_LDB_DI0_MASK : 0));

#elif defined(CONFIG_MX7D)
	writel(CCM_ANALOG_PLL_VIDEO_CLR_ENABLE_CLK_MASK,
			&ccm_anatop->pll_video_set);

	target = CLK_ROOT_ON | LCDIF_PIXEL_CLK_ROOT_FROM_PLL_VIDEO_MAIN_CLK |
		 CLK_ROOT_PRE_DIV((ipu_div - 1)) | CLK_ROOT_POST_DIV((x - 1));
	clock_set_target_val(LCDIF_PIXEL_CLK_ROOT, target);
	clock_enable(CCGR_LCDIF, 1);

#else
	reg = readl(&ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK |
		 MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK |
		 MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK);
	reg |= ((lvds ? ((di->fbtype == FB_LVDS2) ?
			 CHSCCDR_CLK_SEL_LDB_DI1 : CHSCCDR_CLK_SEL_LDB_DI0)
		: CHSCCDR_CLK_SEL_IPU1_DI0)
			<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET) |
		((ipu_div - 1) << MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET) |
		(CHSCCDR_IPU_PRE_CLK_PLL5 << MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &ccm->chsccdr);

	/* enable ipu1_di0_clk */
	setbits_le32(&ccm->CCGR3, MXC_CCM_CCGR3_IPU1_IPU_DI0_MASK |
			(lvds ? ((di->fbtype == FB_LVDS2) ?
				 MXC_CCM_CCGR3_LDB_DI1_MASK : MXC_CCM_CCGR3_LDB_DI0_MASK)
			 : 0));
#endif
	/* Enable PLL out */
#if !defined(CONFIG_MX7D)
	pll_video &= ~BM_ANADIG_PLL_VIDEO_BYPASS;
	pll_video |= BM_ANADIG_PLL_VIDEO_ENABLE;
	writel(pll_video, &ccm->analog_pll_video);
#endif
}
#else
static void setup_clock(struct display_info_t const *di)
{
#ifdef CONFIG_HDMI_CLK2
	struct anamix_pll *ana_pll = (struct anamix_pll *)ANATOP_BASE_ADDR;

	if (di->fbtype == FB_HDMI) {
		writel(0x01, &ana_pll->pllout_monitor_cfg);
		writel(0x11, &ana_pll->pllout_monitor_cfg);
	}
#endif
}
#endif

void fbp_enable_fb(struct display_info_t const *di, int enable)
{
#ifdef LVDS_SUPPORT
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg, cscmr2;
	u32 tmp;
#endif
	if (di->enable_gp && !enable)
		gpio_direction_output(di->enable_gp, enable ^
			((di->fbflags & FBF_ENABLE_GPIOS_ACTIVE_LOW) ? 1 : 0));
	switch (di->fbtype) {
#ifdef CONFIG_IMX_HDMI
	case FB_HDMI:
		imx_enable_hdmi_phy();
		board_enable_hdmi(di, enable);
		break;
#endif
	case FB_LCD2:
	case FB_LCD:
		board_enable_lcd(di, enable);
		break;
#ifdef LVDS_SUPPORT
	case FB_LVDS:
#ifdef CONFIG_MX6SX
#define GPR_LDB	6
#else
#define GPR_LDB	2
#endif
		reg = readl(&iomux->gpr[GPR_LDB]);
		cscmr2 = readl(&mxc_ccm->cscmr2);
		reg &= ~(IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT |
			 IOMUXC_GPR2_BIT_MAPPING_CH0_JEIDA |
			 IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT |
 			 IOMUXC_GPR2_BIT_MAPPING_CH1_JEIDA |
			 IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
			 IOMUXC_GPR2_LVDS_CH1_MODE_MASK |
			 IOMUXC_GPR2_SPLIT_MODE_EN_MASK);
		tmp = 0;
		if ((di->pixfmt == IPU_PIX_FMT_RGB24) ||
				(di->pixfmt == IPU_PIX_FMT_BGR24))
			tmp |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
		if (di->fbflags & FBF_JEIDA)
			tmp |= IOMUXC_GPR2_BIT_MAPPING_CH0_JEIDA;
		if (di->fbflags & FBF_SPLITMODE) {
			tmp |= tmp << 2;
			tmp |= IOMUXC_GPR2_SPLIT_MODE_EN_MASK |
			       IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;

			cscmr2 &= ~(MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV |
				 MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV);

		} else {
			cscmr2 |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
		}
		writel(cscmr2, &mxc_ccm->cscmr2);
		if (enable)
			reg |= tmp | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;

		writel(reg, &iomux->gpr[GPR_LDB]);
		board_enable_lvds(di, enable);
		break;
#if !defined(CONFIG_MX6SX)
	case FB_LVDS2:
		reg = readl(&iomux->gpr[2]);
		cscmr2 = readl(&mxc_ccm->cscmr2);
		reg &= ~(IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT |
			 IOMUXC_GPR2_BIT_MAPPING_CH1_JEIDA |
			 IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
			 IOMUXC_GPR2_LVDS_CH1_MODE_MASK |
			 IOMUXC_GPR2_SPLIT_MODE_EN_MASK);
		if ((di->pixfmt == IPU_PIX_FMT_RGB24) ||
				(di->pixfmt == IPU_PIX_FMT_BGR24))
			reg |= IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT;
		if (di->fbflags & FBF_JEIDA)
			reg |= IOMUXC_GPR2_BIT_MAPPING_CH1_JEIDA;

		cscmr2 |= MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
		writel(cscmr2, &mxc_ccm->cscmr2);

		if (enable)
			reg |= IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
		writel(reg, &iomux->gpr[2]);
		board_enable_lvds2(di, enable);
		break;
#endif
#endif
	case FB_MIPI:
		board_enable_mipi(di, enable);
		break;
	case FB_MIPI2:
		board_enable_mipi2(di, enable);
		break;
	}
	if (di->enable_gp && enable)
		gpio_direction_output(di->enable_gp, enable ^
			((di->fbflags & FBF_ENABLE_GPIOS_ACTIVE_LOW) ? 1 : 0));
}

static void imx_prepare_display(void)
{
#ifdef LVDS_SUPPORT
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;
#if !defined(CONFIG_MX6SX)
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	enable_ipu_clock();
#ifdef CONFIG_IMX_HDMI
	imx_setup_hdmi();
#endif
#endif
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		| MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
#ifdef CONFIG_MX6SX
	/* Select pll5 clock for ldb di0 */
#else
	/* set LDB0, LDB1 clk select to 011/011 */
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
#endif
	writel(reg, &mxc_ccm->cs2cdr);

#ifndef CONFIG_MX6SX
	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
#endif
#endif
}

struct flags_check {
	char	c;
	int	flag;
};

static struct flags_check fc1[] = {
	{ 'm', FBF_MODESTR},
	{ 'j', FBF_JEIDA},
	{ 's', FBF_SPLITMODE},
	{ 0, 0},
};

static struct flags_check fc2[] = {
	{ 'S', FBF_SPI},
	{ 'A', FBF_ALT_PWM},
	{ 'D', FBF_BKLIT_DTB},
	{ 'b', FBF_BKLIT_LOW_ACTIVE},
	{ 'P', FBF_PINCTRL},
	{ 'l', FBF_ENABLE_GPIOS_ACTIVE_LOW},
	{ 'o', FBF_ENABLE_GPIOS_OPEN_DRAIN},
	{ 'd', FBF_ENABLE_GPIOS_DTB},
	{ 't', FBF_SN65_ALT},
	{ 's', FBF_MODE_SKIP_EOT},
	{ 'h', FBF_DSI_HBP},
	{ 'f', FBF_DSI_HFP},
	{ 'a', FBF_DSI_HSA},
	{ 'v', FBF_MODE_VIDEO},
	{ 'B', FBF_MODE_VIDEO_BURST},
	{ 'M', FBF_MODE_VIDEO_MBC},
	{ 'U', FBF_MODE_VIDEO_SYNC_PULSE},
	{ 'c', FBF_MIPI_CMDS},
	{ 'L', FBF_BKLIT_EN_LOW_ACTIVE},
	{ 'E', FBF_BKLIT_EN_DTB},
	{ 'I', FBF_USE_IPU_CLOCK},
	{ '4', FBF_DSI_LANES_4},
	{ '3', FBF_DSI_LANES_3},
	{ '2', FBF_DSI_LANES_2},
	{ '1', FBF_DSI_LANES_1},
	{ 0, 0},
};

struct di_parse {
	u8 offset;
	u8 base;
};

static const struct di_parse di_parsing[] = {
	{ offsetof(struct display_info_t, bus_num), 16 },
	{ offsetof(struct display_info_t, addr_num), 16 },
	{ offsetof(struct display_info_t, bus_gp), 10 },
	{ offsetof(struct display_info_t, bus_gp_delay_ms), 10 },
	{ offsetof(struct display_info_t, backlight_en_gp), 10 },
	{ offsetof(struct display_info_t, min_hs_clock_multiple), 10 },
	{ offsetof(struct display_info_t, enable_high_duration_us), 10 },
	{ offsetof(struct display_info_t, enable_low_duration_us), 10 },
	{ offsetof(struct display_info_t, power_gp), 10 },
	{ offsetof(struct display_info_t, power_delay_ms), 10 },
};

static void check_flags(struct display_info_t *di, const char **pp, struct flags_check *fc_base)
{
	const char *p = *pp;
	char c = *p;

	while (1) {
		struct flags_check *fc = fc_base;

		while (1) {
			if (!fc->c) {
				*pp = p;
				return;
			}
			if (c == fc->c) {
				di->fbflags |= fc->flag;
				p++;
				c = *p;
				break;
			}
			fc++;
		}
	}
}

static int code_flags(char *p, int size, int flags, struct flags_check *fc_base)
{
	struct flags_check *fc = fc_base;
	int cnt = 0;

	while (flags) {
		if (!fc->c)
			break;
		if ((fc->flag & flags) == fc->flag) {
			if (size) {
				*p++ = fc->c;
				size--;
				cnt++;
			}
			flags &= ~fc->flag;
		}
		fc++;
	}
	return cnt;
}

const char *check_alias(const char *p, struct display_info_t *di, unsigned fb, int index)
{
	if (*p >= '9') {
		int len = 0;
		int i = 0;

		while (p[len] != ',') {
			if (!p[len]) {
				printf("expecting ending quote\n");
				return NULL;
			}
			len++;
		}
		if (!len) {
			printf("empty string\n");
			return NULL;
		}

		i = 1;
		while (1) {
			const char *s;

			if (i >= ARRAY_SIZE(aliases)) {
				/* Allocate memory to hold alias name */
				const char *old;
				char *q;

				old = new_aliases[fb][index];
				new_aliases[fb][index] = NULL;
				if (old)
					free((void *)old);

				q = malloc(len + 1);
				if (!q)
					return NULL;
				memcpy(q, p, len);
				q[len] = 0;
				new_aliases[fb][index] = q;
				i = 255;
				break;
			}
			s = aliases[i];
			if (!memcmp(p, s, len) && !s[len]) {
				break;
			}
			i++;
		}
		di->enable_alias[index] = i;
		p += len + 1;
		if (*p == ' ')
			p++;
	}
	return p;
}

static const struct display_info_t * parse_mode(
		const struct display_info_t *gdi, int cnt, const char *p,
		unsigned fb, unsigned *prefer)
{
	char c;
	char *endp;
	unsigned value;
	int i;
	struct display_info_t *di;
	char *mode_str = g_mode_str[fb];
	int pix_fmt = IPU_PIX_FMT_RGB24;

	if (*p == '*') {
		p++;
		*prefer = 1;
	}
	if (!strcmp(p, "off")) {
		*prefer |= 2;
		return NULL;
	}

	i = 0;
	while (i < 80 - 1) {
		c = *p;
		if (c)
			p++;
		if (!c || (c == ':')) {
			break;
		}
		mode_str[i++] = c;
	}
	mode_str[i] = 0;
	c = *p;
	if (!c) {
		return find_panel(gdi, cnt, fb, mode_str);
	}
	di = &g_di_temp[fb];
	memset(di, 0, sizeof(*di));

	di->fbtype = fb;
	di->mode.name = mode_str;
	di->enable = fbp_enable_fb;

	for (i = 0; i < ARRAY_SIZE(di->enable_alias); i++) {
		p = check_alias(p, di, fb, i);
		if (!p)
			return NULL;
	}

	value = simple_strtoul(p, &endp, 10);
	if (endp <= p) {
		printf("expecting 18|24\n");
		return NULL;
	}
	if ((value != 18) && (value != 24)) {
		printf("expecting 18|24, found %d\n", value);
		return NULL;
	}
	p = endp;

	check_flags(di, &p, fc1);
	c = *p;

	if (c == 'b') {
		pix_fmt = IPU_PIX_FMT_BGR24;
		p++;
		c = *p;
	}
	di->pixfmt = (value == 24) ? pix_fmt : IPU_PIX_FMT_RGB666;

	if (c == ',')
		p++;
	check_flags(di, &p, fc2);
	c = *p;

	if (c == 'G') {
		p++;
		value = simple_strtoul(p, &endp, 10);
		if (endp <= p) {
			printf("expecting gpio number\n");
			return NULL;
		}
		p = endp;
		di->enable_gp = value;
		c = *p;
	}
	if (c == 'R') {
		p++;
		value = simple_strtoul(p, &endp, 10);
		if (endp <= p) {
			printf("expecting gpio number\n");
			return NULL;
		}
		p = endp;
		di->reset_gp = value;
		c = *p;
	}
	if (c == 'e') {
		di->pre_enable = board_pre_enable;
		p++;
		c = *p;
	}
	if (c == 'x') {
		p++;
		for (i = 0; i < ARRAY_SIZE(di_parsing); i++) {
			unsigned char *dest = (unsigned char *)((char *)di + di_parsing[i].offset);
			u32 val;

			if (*p == ' ')
				p++;
			val = simple_strtoul(p, &endp, di_parsing[i].base);
			if (endp <= p) {
				printf("expecting integer:%s\n", p);
				return NULL;
			}
			if (val >= 0x256)
				printf("truncating:%x to %x\n", val, (u8)val);
			*dest = (u8)val;
			p = endp;
			if (*p != ',')
				break;
			p++;
		}
		c = *p;
	}
	if (c == 'p') {
		p++;
		value = simple_strtoul(p, &endp, 10);
		if (endp <= p) {
			printf("expecting period of pwm\n");
			return NULL;
		}
		p = endp;
		di->pwm_period = value;
		c = *p;
	}
	if (c != ':') {
		printf("expected ':', %s\n", p);
		return NULL;
	}
	p++;

	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *dest = (u32 *)((char *)&di->mode + timings_offsets[i]);
		u32 val;

		val = simple_strtoul(p, &endp, 10);
		if (endp <= p) {
			printf("expecting integer:%s\n", p);
			return NULL;
		}
		*dest = val;
		p = endp;
		if (*p == ',')
			p++;
		if (*p == ' ')
			p++;
	}

	if (!(di->fbflags & FBF_USE_IPU_CLOCK))
		di->mode.sync |= FB_SYNC_EXT;
	if (*p) {
		printf("extra parameters found:%s\n", p);
		return NULL;
	}
	return di;
}

static const struct display_info_t *find_disp(const struct display_info_t *di,
		int cnt, unsigned fb, unsigned *prefer)
{
	int i;
	const char *name = env_get(fbnames[fb]);

	if (name) {
		di = parse_mode(di, cnt, name, fb, prefer);
		if (di)
			return di;
		if (!(*prefer & 2))
			printf("No match, assuming mode_str: %s\n", name);
		return NULL;
	}
	/* No specific name requested, lets probe */
	for (i = 0; i < cnt; i++, di++) {
		if (fb == di->fbtype) {
			if (di->detect && di->detect(di)) {
				printf("auto-detected panel %s\n", di->mode.name);
				return di;
			}
		}
	}
	return NULL;
}

static const struct display_info_t *find_first_disp(
		const struct display_info_t *di, int cnt)
{
	int i;
	unsigned skip_mask = 0;

	for (i = 0; i < cnt; i++, di++) {
		if (di->fbtype >= FB_COUNT)
			continue;
		if (skip_mask & (1 << di->fbtype))
			continue;
		if (env_get(fbnames[di->fbtype])) {
			skip_mask |= (1 << di->fbtype);
			continue;
		}
		return di;
	}
	return NULL;
}

static void str_mode(char *p, int size, const struct display_info_t *di, unsigned prefer)
{
	int count;
	int i, last;
	int interface_width = 18;
	u8 separator = 'x';

	if (prefer) {
		*p++ = '*';
		size--;
	}
	if (!di) {
		count = snprintf(p, size, "off");
		if (size > count) {
			p += count;
			size -= count;
		}
		*p = 0;
		return;
	}
	count = snprintf(p, size, "%s:", di->mode.name);
	if (size > count) {
		p += count;
		size -= count;
	}

	for (i = 0; i < ARRAY_SIZE(di->enable_alias); i++) {
		const char *s = get_alias(di, di->fbtype, i);

		if (s) {
			count = snprintf(p, size, "%s,", s);
			p += count;
			size -= count;
		}
	}
	if ((di->pixfmt == IPU_PIX_FMT_RGB24) ||
			(di->pixfmt == IPU_PIX_FMT_BGR24))
		interface_width = 24;
	count = snprintf(p, size, "%d", interface_width);
	if (size > count) {
		p += count;
		size -= count;
	}

	count = code_flags(p, size, di->fbflags, fc1);
	p += count;
	size -= count;

	if (di->pixfmt == IPU_PIX_FMT_BGR24) {
		*p++ = 'b';
		size--;
	}
	*p++ = ',';
	count = code_flags(p, size, di->fbflags, fc2);
	p += count;
	size -= count;

	if (di->enable_gp) {
		count = snprintf(p, size, "G%u", di->enable_gp);
		if (size > count) {
			p += count;
			size -= count;
		}
	}
	if (di->reset_gp) {
		count = snprintf(p, size, "R%u", di->reset_gp);
		if (size > count) {
			p += count;
			size -= count;
		}
	}
	if (di->pre_enable) {
		*p++ = 'e';
		size--;
	}
	for (last = ARRAY_SIZE(di_parsing) - 1; last >= 0 ; last--) {
		unsigned char *src = (unsigned char *)((char *)di + di_parsing[last].offset);
		unsigned char val = *src;

		if (val)
			break;
	}
	for (i = 0; i <= last; i++) {
		unsigned char *src = (unsigned char *)((char *)di + di_parsing[i].offset);
		unsigned char val = *src;

		count = snprintf(p, size, (di_parsing[i].base == 16) ? "%c%x" :
				"%c%u", separator, val);
		separator = ',';
		if (size > count) {
			p += count;
			size -= count;
		}
	}
	if (di->pwm_period) {
		count = snprintf(p, size, "p%d", di->pwm_period);
		if (size > count) {
			p += count;
			size -= count;
		}
	}
	*p++ = ':';
	size--;

	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *src = (u32 *)((char *)&di->mode + timings_offsets[i]);
		u32 val;

		val = *src;
		if (i && (size > 1)) {
			*p++ = ',';
			size--;
		}
		count = snprintf(p, size, "%d", val);
		if (size > count) {
			p += count;
			size -= count;
		}
	}
	*p = 0;
}

static void print_mode(const struct display_info_t *di, int *len)
{
	int i;
	char format_buf[16];
	const struct fb_videomode_f* mode = &di->mode;
	int fb = di->fbtype;
	char buf[256];

	str_mode(buf, sizeof(buf), di, 0);
	printf("%s: %s\n\x09", short_names[fb], buf);


	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *p = (u32 *)((char *)mode + timings_offsets[i]);
		u32 val;

		val = *p;
		snprintf(format_buf, sizeof(format_buf), " %c%du", '%', len[i]);
		printf(format_buf, val);
	}
	printf("\n");
}

static void print_modes(const struct display_info_t *di, int cnt, unsigned mask)
{
	int i;
	int len[ARRAY_SIZE(timings_properties)];

	/* Print heading */
	printf("\x09");
	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		printf(" %s", timings_properties[i]);
		len[i] = strlen(timings_properties[i]);
	}
	printf("\n");

	for (i = 0; i < cnt; i++, di++) {
		if (mask & (1 << di->fbtype))
			print_mode(di, len);
	}
}

static void setup_env_cmds(const struct display_info_t *gdi, int cnt,
		const struct display_info_t **displays)
{
	char *buf = malloc(4096);

	debug("%s: enter\n", __func__);
	if (buf) {
		unsigned mask = get_fb_available_mask(gdi, cnt);
		unsigned fb;

		for (fb = 0; fb < FB_COUNT; fb++) {
			if (mask & (1 << fb)) {
				if (!displays[fb]) {
					unsigned prefer = 0;
					const char *name = env_get(fbnames[fb]);

					if (name)
						displays[fb] = parse_mode(gdi, cnt, name, fb, &prefer);
				}
				setup_cmd_fb(fb, displays[fb], buf, 4096);
			}
		}
		free(buf);
	}
}

static const struct display_info_t *select_display(
		const struct display_info_t *gdi, int cnt,
		const struct display_info_t **displays)
{
	const struct display_info_t *di = NULL;
	unsigned prefer_mask = 0;
	unsigned fb;

	for (fb = 0; fb < FB_COUNT; fb++) {
		unsigned prefer = 0;

		displays[fb] = find_disp(gdi, cnt, fb, &prefer);
		if (prefer & 1)
			prefer_mask |= (1 << fb);
	}
	fb = ffs(prefer_mask) - 1;
	if (fb < FB_COUNT) {
		di = displays[fb];
	} else {
		/* default to the 1st one in the list*/
		for (fb = 0; fb < FB_COUNT; fb++) {
			if (!di) {
				di = displays[fb];
			} else {
				if (displays[fb] && (displays[fb] < di))
					di = displays[fb];
			}
		}
		if (!di) {
			di = find_first_disp(gdi, cnt);
			if (di)
				displays[di->fbtype] = di;
		}
	}
	setup_env_cmds(gdi, cnt, displays);
	return di;
}

static const struct display_info_t *g_displays;
static int g_display_cnt;

static const struct display_info_t *g_di_active;

void board_video_enable(void)
{
	const struct display_info_t *di = g_di_active;
	if (di && di->enable)
		di->enable(di, 1);
}

static int init_display(const struct display_info_t *di)
{
	int ret;

	if (di->pre_enable)
		di->pre_enable(di);
	setup_clock(di);
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP) || defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP)
	ret = 0;
#elif defined(CONFIG_VIDEO_MXS)
	memcpy(&g_vmode, &di->mode, sizeof(g_vmode));
	g_vmode.pixclock = freq_to_period(di->mode.pixclock_f);
	ret = mxsfb_init(&g_vmode, di->pixfmt);
#else
	ret = 0;
	if (!is_mipi(di->fbtype)) {
		memcpy(&g_vmode, &di->mode, sizeof(g_vmode));
		g_vmode.pixclock = freq_to_period(di->mode.pixclock_f);
		ret = ipuv3_fb_init(&g_vmode, (di->fbtype == FB_LCD2) ? 1 : 0,
			di->pixfmt);
	}
#endif
	if (ret) {
		printf("LCD %s cannot be configured: %d\n", di->mode.name, ret);
		return -EINVAL;
	}
	printf("Display: %s:%s (%ux%u)\n", short_names[di->fbtype],
			di->mode.name, di->mode.xres, di->mode.yres);
	g_di_active = di;
	return di->fbtype;
}

static int do_fbpanel(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	const char *fbname;
	int fb = -1;
	const char *p;
	char *buf;
	unsigned prefer;
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP) || defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP)
#else
	int ret = 0;
#endif
	const struct display_info_t* di = g_displays;
	int cnt = g_display_cnt;

	if (argc < 2) {
		print_modes(di, cnt, (1 << FB_COUNT) - 1);
                return 0;
	}
	fbname = argv[1];
	for (i = 0; i < ARRAY_SIZE(short_names); i++) {
		if (!strcmp(short_names[i], fbname)) {
			fb = i;
			break;
		}
	}
	if (fb < 0)
		return CMD_RET_USAGE;

	if (argc < 3) {
		print_modes(di, cnt, 1 << fb);
		return 0;
	}
	p = argv[2];
	di = parse_mode(di, cnt, p, fb, &prefer);
	if (!di && !(prefer & 2))
		return 1;

	buf = malloc(4096);
	if (buf) {
		str_mode(buf, 256, di, prefer & 1);
		env_set(fbnames[fb], buf);

		setup_cmd_fb(fb, di, buf, 4096);
		free(buf);
	}
	if (!(prefer & 1))
		return 0;
	{
		const struct display_info_t* di1;
		di1 = g_di_active;
		if (di1 && di1->enable) {
			di1->enable(di1, 0);
			g_di_active = NULL;
		}
	}
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP) || defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP)
	return 0;
#elif defined(CONFIG_VIDEO_IPUV3)
	ipuv3_fb_shutdown();
	if (!di)
		return 0;
	ret = init_display(di);
	if (ret < 0)
		return ret;
	ipuv3_fb_init2();
#elif defined(CONFIG_VIDEO_MXS)
	lcdif_power_down();
	if (!di)
		return 0;
	ret = init_display(di);
	if (ret < 0)
		return ret;
	mxsfb_init2();
#endif
	return 0;
}

U_BOOT_CMD(fbpanel, 3, 0, do_fbpanel,
           "show/set panel names available",
           "fbpanel [hdmi|lcd|lvds|lvds2] [\"[*]mode_str[:[m][j][s][18|24][S][e][xhexbus,hexaddr][pnnn]:pixclkfreq,xres,yres,hback-porch,hfront-porch,vback-porch,vfront-porch,hsync,vsync]\"]\n"
           "\n"
           "fbpanel  - show all panels");

static const struct display_info_t *g_disp[FB_COUNT];

int fbpanel_video_init(void)
{
	const struct display_info_t *di = select_display(g_displays, g_display_cnt, g_disp);
	int ret;

	if (!di)
		return -EINVAL;
	ret = init_display(di);
#ifndef CONFIG_DM_VIDEO
	if (is_mipi(di->fbtype))
		ret = -EINVAL;
#endif
	return ret;
}

int board_video_skip(void)
{
	int ret = fbpanel_video_init();

	return (ret < 0) ? ret : 0;
}

void fbp_setup_display(const struct display_info_t *displays, int cnt)
{
	g_displays = displays;
	g_display_cnt = cnt;
	imx_prepare_display();
}

void fbp_setup_env_cmds(void)
{
	int fb;

	debug("%s: %u\n", __func__, g_display_cnt);
	for (fb = 0; fb < FB_COUNT; fb++) {
		if (g_disp[fb]) {
			setup_env_cmds(g_displays, g_display_cnt, g_disp);
			return;
		}
	}
}
