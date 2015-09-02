/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#if !defined(CONFIG_MX51) && !defined(CONFIG_MX53)
#include <asm/arch/mxc_hdmi.h>
#endif
#include <asm/arch/sys_proto.h>
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
[FB_LCD2] = "fb_lcd2",
[FB_LVDS] = "fb_lvds",
[FB_LVDS2] = "fb_lvds2"
};

static const char *const timings_names[] = {
[FB_HDMI] = "t_hdmi",
[FB_LCD] = "t_lcd",
[FB_LCD2] = "t_lcd2",
[FB_LVDS] = "t_lvds",
[FB_LVDS2] = "t_lvds2"
};

static const char *const ch_names[] = {
[FB_HDMI] = "",
[FB_LCD] = "",
[FB_LCD2] = "",
[FB_LVDS] = "ldb/lvds-channel@0",
[FB_LVDS2] = "ldb/lvds-channel@1"
};

static const char *const cmd_fbnames[] = {
[FB_HDMI] = "cmd_hdmi",
[FB_LCD] = "cmd_lcd",
[FB_LCD2] = "cmd_lcd2",
[FB_LVDS] = "cmd_lvds",
[FB_LVDS2] = "cmd_lvds2"
};

static const char *const backlight_names[] = {
[FB_HDMI] = "backlight_hdmi",
[FB_LCD] = "backlight_lcd",
[FB_LCD2] = "backlight_lcd2",
[FB_LVDS] = "backlight_lvds",
[FB_LVDS2] = "backlight_lvds2"
};

static const char *const pwm_names[] = {
[FB_HDMI] = "pwm_hdmi",
[FB_LCD] = "pwm_lcd",
[FB_LCD2] = "pwm_lcd2",
[FB_LVDS] = "pwm_lvds",
[FB_LVDS2] = "pwm_lvds2"
};

static const char *const short_names[] = {
[FB_HDMI] = "hdmi",
[FB_LCD] = "lcd",
[FB_LCD2] = "lcd2",
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

static unsigned get_fb_available_mask(const struct display_info_t *di, int cnt)
{
	unsigned mask = 0;
	int i;

	for (i = 0; i < cnt; i++, di++)
		mask |= (1 << di->fbtype);

	return mask;
}
static const char rgb24[] = "RGB24";
static const char rgb565[] = "RGB565";
static const char rgb666[] = "RGB666";
static const char yuyv16[] = "YUYV16";

static char lvds_enabled;

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

	if (fb == FB_LVDS)
		lvds_enabled = 0;
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
			sz = snprintf(buf, size, "fdt set %s status disabled", fbnames[fb]);
			buf += sz;
			size -= sz;
			if (fb == FB_LCD)
				sz = snprintf(buf, size, ";fdt set lcd status disabled");
			if ((fb == FB_LVDS) || (fb == FB_LVDS2)) {
				sz = snprintf(buf, size, ";fdt set ldb/lvds-channel@%d status disabled", fb - FB_LVDS);
			}
			setenv(cmd_fbnames[fb], buf_start);
			return;
		}
	} else {
		if (di->fbflags & FBF_MODESTR)
			mode_str = di->mode.name;
	}

	if (fb == FB_LVDS)
		lvds_enabled = 1;
	sz = snprintf(buf, size, "fdt set %s status okay;", fbnames[fb]);
	buf += sz;
	size -= sz;
	if ((fb == FB_LVDS2) && !lvds_enabled) {
		sz = snprintf(buf, size, "fdt set ldb/lvds-channel@1 primary;");
		buf += sz;
		size -= sz;
	}

	if (di->pixfmt == IPU_PIX_FMT_RGB24)
		fmt = rgb24;
	else if (di->pixfmt == IPU_PIX_FMT_YUYV)
		fmt = yuyv16;
	else if (di->pixfmt == IPU_PIX_FMT_RGB565)
		fmt = rgb565;
	else
		fmt = rgb666;

	if (di && (fb >= FB_LCD)) {
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

	if (di && ((fb == FB_LCD) || (fb == FB_LCD2))) {
		sz = snprintf(buf, size, "fdt set %s default_ifmt %s;",
				short_names[fb], fmt);
		buf += sz;
		size -= sz;

		sz = snprintf(buf, size, "fdt set %s status okay;",
				short_names[fb]);
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

	if (di && di->pwm_period) {
		sz = snprintf(buf, size, "fdt get value pwm %s phandle; fdt set %s pwms <${pwm} 0x%x 0x%x>;",
				pwm_names[fb], backlight_names[fb], 0, di->pwm_period);
		buf += sz;
		size -= sz;
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

static char g_mode_str[FB_COUNT][80];
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

int calc_gcd(int a, int b)
{
	int n;

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

#if !defined(CONFIG_MX51) && !defined(CONFIG_MX53)
void reparent_lvds(int fbtype, int new_parent)
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

#if defined(CONFIG_MX51) || defined(CONFIG_MX53)
int get_pll3_clock(void);

void setup_clock(struct display_info_t const *di)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u64 lval = 1000000000000ULL;
	u32 desired_freq, freq;
	u32 pll3_freq = get_pll3_clock();
	u32 n, m;
	u32 i;
	u32 best = 8;
	u32 best_diff = ~0;
	u32 diff;
	u32 reg;

	if (!(di->mode.sync & FB_SYNC_EXT))
		return;

	do_div(lval, di->mode.pixclock);
	desired_freq = ((u32)lval) * 16;

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

	ipu_set_ldb_clock(pll3_freq / best);	/* used for all ext clocks */
}

#else
void setup_clock(struct display_info_t const *di)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 desired_freq;
	u32 out_freq;
	int lvds = ((di->fbtype == FB_LVDS) | (di->fbtype == FB_LVDS2)) ? 1 : 0;
	u64 lval = 1000000000000ULL;
	int timeout = 1000;
	int post_div = 2;
	int misc2_div = 0;
	int multiplier;
	int gcd;
	int num, denom;
	u32 pll_video;
	u32 reg;
	int ipu_div = 7;
	int x = 1;

	if (!(di->mode.sync & FB_SYNC_EXT))
		return;

#ifdef CONFIG_MX6SX
	clrbits_le32(&ccm->CCGR2, MXC_CCM_CCGR2_LCD_MASK);
	clrbits_le32(&ccm->CCGR5, 0x3f << 8);
#else
	/* gate ipu1_di0_clk */
	clrbits_le32(&ccm->CCGR3, MXC_CCM_CCGR3_LDB_DI0_MASK |
			MXC_CCM_CCGR3_LDB_DI1_MASK |
			MXC_CCM_CCGR3_IPU1_IPU_DI0_MASK);
#endif

	pll_video = readl(&ccm->analog_pll_video);
	pll_video &= BM_ANADIG_PLL_VIDEO_ENABLE;
	pll_video |= BM_ANADIG_PLL_VIDEO_POWERDOWN;
	writel(pll_video, &ccm->analog_pll_video);

	do_div(lval, di->mode.pixclock);
	desired_freq = (u32)lval;
	debug("desired_freq=%d\n", desired_freq);
	out_freq = desired_freq;

	if (lvds) {
		reparent_lvds(di->fbtype, 0);
		desired_freq *= 7;
		if (di->fbflags & FBF_SPLITMODE)
			desired_freq >>= 1;

	}

	debug("desired_freq=%d\n", desired_freq);

#define MIN_FREQ 648000000	/* 24000000 * 27 */

	if (!(is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D)) ||
			soc_rev() > CHIP_REV_1_0) {
		while (desired_freq < MIN_FREQ) {
			desired_freq <<= 1;
			if (--post_div <= 0)
				break;
		}
		debug("desired_freq=%d\n", desired_freq);

		while (desired_freq < MIN_FREQ) {
			desired_freq <<= 1;
			misc2_div = (misc2_div << 1) + 1;
			if (misc2_div >= 3)
				break;
		}
		debug("desired_freq=%d\n", desired_freq);
	};
	if (!lvds) {
		num = (MIN_FREQ - 1) / desired_freq + 1;
		x = (num + 7) / 8;
		ipu_div =  (num - 1) / x + 1;
		desired_freq *= ipu_div * x;
	} else if (desired_freq < MIN_FREQ) {
		printf("desired_freq=%d is too low\n", desired_freq);
	}
	ipu_set_ldb_clock(out_freq * x);	/* used for all ext clocks */
	debug("desired_freq=%d ipu div=%d x=%d\n", desired_freq, ipu_div, x);

	multiplier = desired_freq / 24000000;
	if (multiplier > 54) {
		multiplier = 54;
		desired_freq = 24000000 * 54;
	}

	reg = readl(&ccm->pmu_misc2);
	reg &= ~(BF_PMU_MISC2_VIDEO_DIV(3));
	reg |= BF_PMU_MISC2_VIDEO_DIV(misc2_div);
	writel(reg, &ccm->pmu_misc2);

	pll_video &= ~(BM_ANADIG_PLL_VIDEO_DIV_SELECT |
		 BM_ANADIG_PLL_VIDEO_POST_DIV_SELECT);
	pll_video |= BF_ANADIG_PLL_VIDEO_DIV_SELECT(multiplier) |
		BF_ANADIG_PLL_VIDEO_POST_DIV_SELECT(post_div);
	writel(pll_video, &ccm->analog_pll_video);

	desired_freq -= multiplier * 24000000;
	gcd = calc_gcd(desired_freq, 24000000);
	debug("gcd=%d desired_freq=%d 24000000\n", gcd, desired_freq);
	num = desired_freq / gcd;
	denom = 24000000 / gcd;
	debug("desired_freq=%d multiplier=%d %d/%d\n", desired_freq, multiplier, num, denom);

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

	pll_video &= ~BM_ANADIG_PLL_VIDEO_BYPASS;
	pll_video |= BM_ANADIG_PLL_VIDEO_ENABLE;
	writel(pll_video, &ccm->analog_pll_video);

#ifdef CONFIG_MX6SX
	reg = readl(&ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK);
	writel(reg, &ccm->cs2cdr);

	reg = readl(&ccm->cscdr2);
	reg &= ~((7 << 9) | (7 << 12) | (7 << 15));
	reg |= ((2 << 15) | ((lvds ? 3 : 0) << 9) | (ipu_div - 1) << 12);
	writel(reg, &ccm->cscdr2);

	setbits_le32(&ccm->CCGR2, MXC_CCM_CCGR2_LCD_MASK);
	setbits_le32(&ccm->CCGR5, (3 << 10) | (lvds ? (3 << 12) : 0));
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
}
#endif

void fbp_enable_fb(struct display_info_t const *di, int enable)
{
#if !defined(CONFIG_MX51) && !defined(CONFIG_MX53)
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg, cscmr2;
	u32 tmp;
#endif
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
#if !defined(CONFIG_MX51) && !defined(CONFIG_MX53)
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
#endif
	}
}

static void imx_prepare_display(void)
{
#ifndef CONFIG_MX6SX
#if !defined(CONFIG_MX51) && !defined(CONFIG_MX53)
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

	if (c == 'S') {
		di->fbflags |= FBF_SPI;
		p++;
		c = *p;
	}
	if (c == 'e') {
		di->pre_enable = board_pre_enable;
		p++;
		c = *p;
	}
	if (c == 'x') {
		p++;
		value = simple_strtoul(p, &endp, 16);
		if (endp <= p) {
			printf("expecting bus\n");
			return NULL;
		}
		p = endp;
		di->bus = value;
		c = *p;
		if (c == ',') {
			p++;
			value = simple_strtoul(p, &endp, 16);
			if (endp <= p) {
				printf("expecting bus addr\n");
				return NULL;
			}
			p = endp;
			di->addr = value;
			c = *p;
		}
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
	count = snprintf(p, size, "%d", (di->pixfmt == IPU_PIX_FMT_RGB24) ? 24 : 18);
	if (size > count) {
		p += count;
		size -= count;
	}
	if (di->fbflags & FBF_SPI) {
		*p++ = 'S';
		size--;
	}
	if (di->pre_enable) {
		*p++ = 'e';
		size--;
	}
	if (di->bus || di->addr) {
		count = snprintf(p, size, "x%x,%x", di->bus, di->addr);
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
	int ret;

	if (di->pre_enable)
		di->pre_enable(di);
	setup_clock(di);
	ret = ipuv3_fb_init(&di->mode, (di->fbtype == FB_LCD2) ? 1 : 0,
			di->pixfmt);
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
           "fbpanel [hdmi|lcd|lvds|lvds2] [\"[*]mode_str[:[m][j][s][18|24][S][e][xhexbus,hexaddr][pnnn]:pixclkfreq,xres,yres,hback-porch,hfront-porch,vback-porch,vfront-porch,hsync,vsync]\"]\n"
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
