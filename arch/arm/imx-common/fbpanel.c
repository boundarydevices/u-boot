/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/errno.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/gpio.h>
#include <asm/imx-common/fbpanel.h>
#include <asm/io.h>
#include <div64.h>
#include <i2c.h>
#include <malloc.h>
#include <video_fb.h>
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
static const char *const fbnames[] = {
[FB_HDMI] = "fb_hdmi",
[FB_LCD] = "fb_lcd",
[FB_LVDS] = "fb_lvds",
[FB_LVDS2] = "fb_lvds2"
};

static const char *const timings_names[] = {
[FB_HDMI] = "t_hdmi",
[FB_LCD] = "t_lcd",
[FB_LVDS] = "t_lvds",
[FB_LVDS2] = "t_lvds2"
};

static const char *const ch_names[] = {
[FB_HDMI] = "",
[FB_LCD] = "",
[FB_LVDS] = "ldb/lvds-channel@0",
[FB_LVDS2] = "ldb/lvds-channel@1"
};

static const char *const cmd_fbnames[] = {
[FB_HDMI] = "cmd_hdmi",
[FB_LCD] = "cmd_lcd",
[FB_LVDS] = "cmd_lvds",
[FB_LVDS2] = "cmd_lvds2"
};

static const char *const short_names[] = {
[FB_HDMI] = "hdmi",
[FB_LCD] = "lcd",
[FB_LVDS] = "lvds",
[FB_LVDS2] = "lvds2"
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
	offsetof(struct fb_videomode, pixclock),
	offsetof(struct fb_videomode, xres),
	offsetof(struct fb_videomode, yres),
	offsetof(struct fb_videomode, left_margin),
	offsetof(struct fb_videomode, right_margin),
	offsetof(struct fb_videomode, upper_margin),
	offsetof(struct fb_videomode, lower_margin),
	offsetof(struct fb_videomode, hsync_len),
	offsetof(struct fb_videomode, vsync_len),
};

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

void board_enable_hdmi(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_hdmi")));
void board_enable_lcd(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lcd")));
void board_enable_lvds(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lvds")));
void board_enable_lvds2(const struct display_info_t *di, int enable)
	__attribute__((weak, alias("__board_enable_lvds2")));

static unsigned get_fb_available_mask(const struct display_info_t *di, int cnt)
{
	unsigned mask = 0;
	int i;

	for (i = 0; i < cnt; i++, di++)
		mask |= (1 << di->fbtype);

	return mask;
}
static const char rgb24[] = "RGB24";
static const char rgb666[] = "RGB666";
static const char yuyv16[] = "YUYV16";

static void setup_cmd_fb(unsigned fb, const struct display_info_t *di, char *buf, int size)
{
	const char *mode_str = NULL;
	int i;
	int sz;
	const char *buf_start = buf;
	const struct fb_videomode *mode;
	const char * fmt;

	if (getenv("cmd_frozen") && getenv(cmd_fbnames[fb]))
		return;		/* don't override if already set */

	if (!di) {
		const char *name = getenv(fbnames[fb]);
		if (name) {
			if (name[0] == '*')
				name++;
			if (strcmp(name, "off")) {
				/* Not off and not in list, assume mode_str value */
				mode_str = name;
			}
		}
		if (!mode_str) {
			snprintf(buf, size, "fdt set %s status disabled", fbnames[fb]);
			setenv(cmd_fbnames[fb], buf);
			return;
		}
	} else {
		if (di->fbflags & FBF_MODESTR)
			mode_str = di->mode.name;
	}

	sz = snprintf(buf, size, "fdt set %s status okay;", fbnames[fb]);
	buf += sz;
	size -= sz;

	if (di->pixfmt == IPU_PIX_FMT_RGB24)
		fmt = rgb24;
	else if (di->pixfmt == IPU_PIX_FMT_YUYV)
		fmt = yuyv16;
	else
		fmt = rgb666;

	if (di && ((fb == FB_LCD) || (fb == FB_LVDS) || (fb == FB_LVDS2))) {
#ifdef CONFIG_MX6SX
		sz = snprintf(buf, size, "fdt set %s bus-width <%u>;", short_names[fb],
				(di->pixfmt == IPU_PIX_FMT_RGB24) ? 24 : 18);

#else
		sz = snprintf(buf, size, "fdt set %s interface_pix_fmt %s;",
				fbnames[fb], fmt);
#endif
		buf += sz;
		size -= sz;
	}

	if (di && (fb == FB_LCD)) {
		sz = snprintf(buf, size, "fdt set lcd default_ifmt %s;", fmt);
		buf += sz;
		size -= sz;
	}

	if (di && ((fb == FB_LVDS) || (fb == FB_LVDS2))) {

		sz = snprintf(buf, size, "fdt set %s fsl,data-width <%u>;",
				ch_names[fb],
				(di->pixfmt == IPU_PIX_FMT_RGB24) ? 24 : 18);
		buf += sz;
		size -= sz;

		sz = snprintf(buf, size, "fdt set %s fsl,data-mapping %s;",
				ch_names[fb],
				(di->fbflags & FBF_JEIDA) ? "jeida" : "spwg");
		buf += sz;
		size -= sz;

		if (di->fbflags & FBF_SPLITMODE) {
			sz = snprintf(buf, size, "fdt set ldb split-mode 1;");
			buf += sz;
			size -= sz;
		}
	}

	if (mode_str) {
		snprintf(buf, size, "fdt set %s mode_str %s;", fbnames[fb], mode_str);
		setenv(cmd_fbnames[fb], buf_start);
		return;
	}

	mode = &di->mode;
	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *p = (u32 *)((char *)mode + timings_offsets[i]);
		u32 val;

		if (i == 0) {
			u64 lval = 1000000000000ULL;

			do_div(lval, mode->pixclock);
			val = (u32)lval;
		} else {
			val = *p;
		}
		sz = snprintf(buf, size, "fdt set %s %s <%u>;", timings_names[fb], timings_properties[i], val);
		buf += sz;
		size -= sz;
	}
	setenv(cmd_fbnames[fb], buf_start);
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

static char g_mode_str[4][80];
static struct display_info_t g_di_temp[FB_COUNT];

int fbp_detect_i2c(struct display_info_t const *di)
{
	int ret;
	int gp = di->bus >> 8;

	if (gp)
		gpio_set_value(gp, 1);
	ret = i2c_set_bus_num(di->bus & 0xff);
	if (ret == 0)
		ret = i2c_probe(di->addr);
	if (gp)
		gpio_set_value(gp, 0);
	return (ret == 0);
}

void fbp_enable_fb(struct display_info_t const *di, int enable)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg, cscmr2;
	u32 tmp;

	switch (di->fbtype) {
#ifdef CONFIG_IMX_HDMI
	case FB_HDMI:
		imx_enable_hdmi_phy();
		board_enable_hdmi(di, enable);
		break;
#endif
	case FB_LCD:
		board_enable_lcd(di, enable);
		break;
	case FB_LVDS:
		reg = readl(&iomux->gpr[2]);
		cscmr2 = readl(&mxc_ccm->cscmr2);
		reg &= ~(IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT |
			 IOMUXC_GPR2_BIT_MAPPING_CH0_JEIDA |
			 IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT |
 			 IOMUXC_GPR2_BIT_MAPPING_CH1_JEIDA |
			 IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
			 IOMUXC_GPR2_LVDS_CH1_MODE_MASK |
			 IOMUXC_GPR2_SPLIT_MODE_EN_MASK);
		tmp = 0;
		if (di->pixfmt == IPU_PIX_FMT_RGB24)
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

		writel(reg, &iomux->gpr[2]);
		board_enable_lvds(di, enable);
		break;
	case FB_LVDS2:
		reg = readl(&iomux->gpr[2]);
		cscmr2 = readl(&mxc_ccm->cscmr2);
		reg &= ~(IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT |
			 IOMUXC_GPR2_BIT_MAPPING_CH1_JEIDA |
			 IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
			 IOMUXC_GPR2_LVDS_CH1_MODE_MASK |
			 IOMUXC_GPR2_SPLIT_MODE_EN_MASK);
		if (di->pixfmt == IPU_PIX_FMT_RGB24)
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
	}
}

static void imx_prepare_display(void)
{
#ifndef CONFIG_MX6SX
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
#ifdef CONFIG_IMX_HDMI
	imx_setup_hdmi();
#endif
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK;
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

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

	if (c == 'm') {
		di->fbflags |= FBF_MODESTR;
		p++;
		c = *p;
	}
	if (c == 'j') {
		di->fbflags |= FBF_JEIDA;
		p++;
		c = *p;
	}
	if (c == 's') {
		di->fbflags |= FBF_SPLITMODE;
		p++;
		c = *p;
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
	di->pixfmt = (value == 24) ? IPU_PIX_FMT_RGB24 : IPU_PIX_FMT_RGB666;
	c = *p;
	if (*p != ':') {
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
		if (i == 0) {
			u64 lval = 1000000000000ULL;

			do_div(lval, val);
			val = (u32)lval;
		}
		*dest = val;
		p = endp;
		if (*p == ',')
			p++;
		if (*p == ' ')
			p++;
	}
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
	const char *name = getenv(fbnames[fb]);

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
		if (getenv(fbnames[di->fbtype])) {
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
	int i;

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
	if (di->fbflags & FBF_MODESTR) {
		*p++ = 'm';
		size--;
	}
	if (di->fbflags & FBF_JEIDA) {
		*p++ = 'j';
		size--;
	}
	if (di->fbflags & FBF_SPLITMODE) {
		*p++ = 's';
		size--;
	}
	count = snprintf(p, size, "%d:", (di->pixfmt == IPU_PIX_FMT_RGB24) ? 24 : 18);
	if (size > count) {
		p += count;
		size -= count;
	}

	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *src = (u32 *)((char *)&di->mode + timings_offsets[i]);
		u32 val;

		if (i == 0) {
			u64 lval = 1000000000000ULL;

			do_div(lval, di->mode.pixclock);
			val = (u32)lval;
		} else {
			val = *src;
			if (size > 1) {
				*p++ = ',';
				size--;
			}
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
	const struct fb_videomode* mode = &di->mode;
	int fb = di->fbtype;
	char buf[256];

	str_mode(buf, sizeof(buf), di, 0);
	printf("%s: %s\n\x09", short_names[fb], buf);


	for (i = 0; i < ARRAY_SIZE(timings_properties); i++) {
		u32 *p = (u32 *)((char *)mode + timings_offsets[i]);
		u32 val;

		if (i == 0) {
			u64 lval = 1000000000000ULL;

			do_div(lval, mode->pixclock);
			val = (u32)lval;
		} else {
			val = *p;
		}
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

static const struct display_info_t *select_display(
		const struct display_info_t *gdi, int cnt)
{
	const struct display_info_t *disp[FB_COUNT];
	const struct display_info_t *di = NULL;
	unsigned prefer_mask = 0;
	unsigned fb;
	char *buf = malloc(4096);

	for (fb = 0; fb < ARRAY_SIZE(disp); fb++) {
		unsigned prefer = 0;

		disp[fb] = find_disp(gdi, cnt, fb, &prefer);
		if (prefer & 1)
			prefer_mask |= (1 << fb);
	}
	fb = ffs(prefer_mask) - 1;
	if (fb < ARRAY_SIZE(disp)) {
		di = disp[fb];
	} else {
		/* default to the 1st one in the list*/
		for (fb = 0; fb < ARRAY_SIZE(disp); fb++) {
			if (!di) {
				di = disp[fb];
			} else {
				if (disp[fb] && ((unsigned)disp[fb]
						< (unsigned)di))
					di = disp[fb];
			}
		}
		if (!di) {
			di = find_first_disp(gdi, cnt);
			if (di)
				disp[di->fbtype] = di;
		}
	}

	if (buf) {
		unsigned mask = get_fb_available_mask(gdi, cnt);

		for (fb = 0; fb < ARRAY_SIZE(disp); fb++) {
			if (mask & (1 << fb))
				setup_cmd_fb(fb, disp[fb], buf, 4096);
		}
		free(buf);
	}
	return di;
}

static const struct display_info_t *g_displays;
static int g_display_cnt;

#ifndef CONFIG_MX6SX
static const struct display_info_t *g_di_active;
#endif

void board_video_enable(void)
{
#ifndef CONFIG_MX6SX
	const struct display_info_t *di = g_di_active;
	if (di && di->enable)
		di->enable(di, 1);
#endif
}

static int init_display(const struct display_info_t *di)
{
#ifndef CONFIG_MX6SX
	int ret = ipuv3_fb_init(&di->mode, 0, di->pixfmt);
	if (ret) {
		printf("LCD %s cannot be configured: %d\n", di->mode.name, ret);
		return -EINVAL;
	}
	printf("Display: %s:%s (%ux%u)\n", short_names[di->fbtype],
			di->mode.name, di->mode.xres, di->mode.yres);
	g_di_active = di;
#endif
	return 0;
}

static int do_fbpanel(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	const char *fbname;
	int fb = -1;
	const char *p;
	char *buf;
	unsigned prefer;
	int ret = 0;
	const struct display_info_t* di = g_displays;
	int cnt = g_display_cnt;

	if (argc < 2) {
		print_modes(di, cnt, 0xf);
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
		setenv(fbnames[fb], buf);

		setup_cmd_fb(fb, di, buf, 4096);
		free(buf);
	}
	if (!(prefer & 1))
		return 0;
#ifndef CONFIG_MX6SX
	{
		const struct display_info_t* di1;
		di1 = g_di_active;
		if (di1 && di1->enable) {
			di1->enable(di1, 0);
			g_di_active = NULL;
		}
	}
	ipuv3_fb_shutdown();
	if (!di)
		return 0;
	ret = init_display(di);
	if (ret)
		return ret;
	ipuv3_fb_init2();
#endif
	return ret;
}

U_BOOT_CMD(fbpanel, 3, 0, do_fbpanel,
           "show/set panel names available",
           "fbpanel [hdmi|lcd|lvds|lvds2] [\"[*]mode_str[:[m][j][s][18|24]:pixclkfreq,xres,yres,hback-porch,hfront-porch,vback-porch,vfront-porch,hsync,vsync]\"]\n"
           "\n"
           "fbpanel  - show all panels");

int board_video_skip(void)
{
	const struct display_info_t *di = select_display(g_displays, g_display_cnt);

	if (!di)
		return -EINVAL;
	return init_display(di);
}

void fbp_setup_display(const struct display_info_t *displays, int cnt)
{
	g_displays = displays;
	g_display_cnt = cnt;
	imx_prepare_display();
}
