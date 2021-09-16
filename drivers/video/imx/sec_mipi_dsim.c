/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <asm/io.h>
#include <linux/bsearch.h>
#include <linux/bug.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/unaligned.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>
#include <div64.h>
#include <video_bridge.h>
#include <panel.h>
#include <dsi_host.h>
#include <asm/arch/gpio.h>
#include <dm/device-internal.h>
#include "sec_mipi_dphy_ln14lpp.h"

#define DRIVER_NAME "sec_mipi_dsim"

/* dsim registers */
#define DSIM_VERSION			0x00
#define DSIM_STATUS			0x04
#define DSIM_RGB_STATUS			0x08
#define DSIM_SWRST			0x0c
#define DSIM_CLKCTRL			0x10
#define DSIM_TIMEOUT			0x14
#define DSIM_CONFIG			0x18
#define DSIM_ESCMODE			0x1c
#define DSIM_MDRESOL			0x20
#define DSIM_MVPORCH			0x24
#define DSIM_MHPORCH			0x28
#define DSIM_MSYNC			0x2c
#define DSIM_SDRESOL			0x30
#define DSIM_INTSRC			0x34
#define DSIM_INTMSK			0x38

/* packet */
#define DSIM_PKTHDR			0x3c
#define DSIM_PAYLOAD			0x40
#define DSIM_RXFIFO			0x44
#define DSIM_FIFOTHLD			0x48
#define DSIM_FIFOCTRL			0x4c
#define DSIM_MEMACCHR			0x50
#define DSIM_MULTI_PKT			0x78

/* pll control */
#define DSIM_PLLCTRL_1G			0x90
#define DSIM_PLLCTRL			0x94
#define DSIM_PLLCTRL1			0x98
#define DSIM_PLLCTRL2			0x9c
#define DSIM_PLLTMR			0xa0

/* dphy */
#define DSIM_PHYTIMING			0xb4
#define DSIM_PHYTIMING1			0xb8
#define DSIM_PHYTIMING2			0xbc

/* reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

/* register bit fields */
#define STATUS_PLLSTABLE		BIT(31)
#define STATUS_SWRSTRLS			BIT(20)
#define STATUS_TXREADYHSCLK		BIT(10)
#define STATUS_ULPSCLK			BIT(9)
#define STATUS_STOPSTATECLK		BIT(8)
#define STATUS_GET_ULPSDAT(x)		REG_GET(x,  7,  4)
#define STATUS_GET_STOPSTATEDAT(x)	REG_GET(x,  3,  0)

#define RGB_STATUS_CMDMODE_INSEL	BIT(31)
#define RGB_STATUS_GET_RGBSTATE(x)	REG_GET(x, 12,  0)

#define CLKCTRL_TXREQUESTHSCLK		BIT(31)
#define CLKCTRL_DPHY_SEL_1G		BIT(29)
#define CLKCTRL_DPHY_SEL_1P5G		(0x0 << 29)
#define CLKCTRL_ESCCLKEN		BIT(28)
#define CLKCTRL_PLLBYPASS		BIT(27)
#define CLKCTRL_BYTECLKSRC_DPHY_PLL	REG_PUT(0, 26, 25)
#define CLKCTRL_BYTECLKEN		BIT(24)
#define CLKCTRL_SET_LANEESCCLKEN(x)	REG_PUT(x, 23, 19)
#define CLKCTRL_SET_ESCPRESCALER(x)	REG_PUT(x, 15,  0)

#define TIMEOUT_SET_BTAOUT(x)		REG_PUT(x, 23, 16)
#define TIMEOUT_SET_LPDRTOUT(x)		REG_PUT(x, 15,  0)

#define CONFIG_NON_CONTINUOUS_CLOCK_LANE	BIT(31)
#define CONFIG_CLKLANE_STOP_START	BIT(30)
#define CONFIG_MFLUSH_VS		BIT(29)
#define CONFIG_EOT_R03			BIT(28)
#define CONFIG_SYNCINFORM		BIT(27)
#define CONFIG_BURSTMODE		BIT(26)
#define CONFIG_VIDEOMODE		BIT(25)
#define CONFIG_AUTOMODE			BIT(24)
#define CONFIG_HSEDISABLEMODE		BIT(23)
#define CONFIG_HFPDISABLEMODE		BIT(22)
#define CONFIG_HBPDISABLEMODE		BIT(21)
#define CONFIG_HSADISABLEMODE		BIT(20)
#define CONFIG_SET_MAINVC(x)		REG_PUT(x, 19, 18)
#define CONFIG_SET_SUBVC(x)		REG_PUT(x, 17, 16)
#define CONFIG_SET_MAINPIXFORMAT(x)	REG_PUT(x, 14, 12)
#define CONFIG_SET_SUBPIXFORMAT(x)	REG_PUT(x, 10,  8)
#define CONFIG_SET_NUMOFDATLANE(x)	REG_PUT(x,  6,  5)
#define CONFIG_SET_LANEEN(x)		REG_PUT(x,  4,  0)

#define ESCMODE_SET_STOPSTATE_CNT(X)	REG_PUT(x, 31, 21)
#define ESCMODE_FORCESTOPSTATE		BIT(20)
#define ESCMODE_FORCEBTA		BIT(16)
#define ESCMODE_CMDLPDT			BIT(7)
#define ESCMODE_TXLPDT			BIT(6)
#define ESCMODE_TXTRIGGERRST		BIT(5)

#define MDRESOL_MAINSTANDBY		BIT(31)
#define MDRESOL_SET_MAINVRESOL(x)	REG_PUT(x, 27, 16)
#define MDRESOL_SET_MAINHRESOL(x)	REG_PUT(x, 11,  0)

#define MVPORCH_SET_CMDALLOW(x)		REG_PUT(x, 31, 28)
#define MVPORCH_SET_STABLEVFP(x)	REG_PUT(x, 26, 16)
#define MVPORCH_SET_MAINVBP(x)		REG_PUT(x, 10,  0)

#define MHPORCH_SET_MAINHFP(x)		REG_PUT(x, 31, 16)
#define MHPORCH_SET_MAINHBP(x)		REG_PUT(x, 15,  0)

#define MSYNC_SET_MAINVSA(x)		REG_PUT(x, 31, 22)
#define MSYNC_SET_MAINHSA(x)		REG_PUT(x, 15,  0)

#define INTSRC_PLLSTABLE		BIT(31)
#define INTSRC_SWRSTRELEASE		BIT(30)
#define INTSRC_SFRPLFIFOEMPTY		BIT(29)
#define INTSRC_SFRPHFIFOEMPTY		BIT(28)
#define INTSRC_FRAMEDONE		BIT(24)
#define INTSRC_LPDRTOUT			BIT(21)
#define INTSRC_TATOUT			BIT(20)
#define INTSRC_RXDATDONE		BIT(18)
#define INTSRC_RXTE			BIT(17)
#define INTSRC_RXACK			BIT(16)
#define INTSRC_MASK			(INTSRC_PLLSTABLE	|	\
					 INTSRC_SWRSTRELEASE	|	\
					 INTSRC_SFRPLFIFOEMPTY	|	\
					 INTSRC_SFRPHFIFOEMPTY	|	\
					 INTSRC_FRAMEDONE	|	\
					 INTSRC_LPDRTOUT	|	\
					 INTSRC_TATOUT		|	\
					 INTSRC_RXDATDONE	|	\
					 INTSRC_RXTE		|	\
					 INTSRC_RXACK)

#define INTMSK_MSKPLLSTABLE		BIT(31)
#define INTMSK_MSKSWRELEASE		BIT(30)
#define INTMSK_MSKSFRPLFIFOEMPTY	BIT(29)
#define INTMSK_MSKSFRPHFIFOEMPTY	BIT(28)
#define INTMSK_MSKFRAMEDONE		BIT(24)
#define INTMSK_MSKLPDRTOUT		BIT(21)
#define INTMSK_MSKTATOUT		BIT(20)
#define INTMSK_MSKRXDATDONE		BIT(18)
#define INTMSK_MSKRXTE			BIT(17)
#define INTMSK_MSKRXACK			BIT(16)

#define PKTHDR_SET_DATA1(x)		REG_PUT(x, 23, 16)
#define PKTHDR_GET_DATA1(x)		REG_GET(x, 23, 16)
#define PKTHDR_SET_DATA0(x)		REG_PUT(x, 15,  8)
#define PKTHDR_GET_DATA0(x)		REG_GET(x, 15,  8)
#define PKTHDR_GET_WC(x)		REG_GET(x, 23,  8)
#define PKTHDR_SET_DI(x)		REG_PUT(x,  7,  0)
#define PKTHDR_GET_DI(x)		REG_GET(x,  7,  0)
#define PKTHDR_SET_DT(x)		REG_PUT(x,  5,  0)
#define PKTHDR_GET_DT(x)		REG_GET(x,  5,  0)
#define PKTHDR_SET_VC(x)		REG_PUT(x,  7,  6)
#define PKTHDR_GET_VC(x)		REG_GET(x,  7,  6)

#define FIFOCTRL_FULLRX			BIT(25)
#define FIFOCTRL_EMPTYRX		BIT(24)
#define FIFOCTRL_FULLHSFR		BIT(23)
#define FIFOCTRL_EMPTYHSFR		BIT(22)
#define FIFOCTRL_FULLLSFR		BIT(21)
#define FIFOCTRL_EMPTYLSFR		BIT(20)
#define FIFOCTRL_FULLHMAIN		BIT(11)
#define FIFOCTRL_EMPTYHMAIN		BIT(10)
#define FIFOCTRL_FULLLMAIN		BIT(9)
#define FIFOCTRL_EMPTYLMAIN		BIT(8)
#define FIFOCTRL_NINITRX		BIT(4)
#define FIFOCTRL_NINITSFR		BIT(3)
#define FIFOCTRL_NINITI80		BIT(2)
#define FIFOCTRL_NINITSUB		BIT(1)
#define FIFOCTRL_NINITMAIN		BIT(0)

#define PLLCTRL_DPDNSWAP_CLK		BIT(25)
#define PLLCTRL_DPDNSWAP_DAT		BIT(24)
#define PLLCTRL_PLLEN			BIT(23)
#define PLLCTRL_SET_PMS(x)		REG_PUT(x, 19,  1)
   #define PLLCTRL_SET_P(x)		REG_PUT(x, 18, 13)
   #define PLLCTRL_SET_M(x)		REG_PUT(x, 12,  3)
   #define PLLCTRL_SET_S(x)		REG_PUT(x,  2,  0)

#define PHYTIMING_SET_M_TLPXCTL(x)	REG_PUT(x, 15,  8)
#define PHYTIMING_SET_M_THSEXITCTL(x)	REG_PUT(x,  7,  0)

#define PHYTIMING1_SET_M_TCLKPRPRCTL(x)	 REG_PUT(x, 31, 24)
#define PHYTIMING1_SET_M_TCLKZEROCTL(x)	 REG_PUT(x, 23, 16)
#define PHYTIMING1_SET_M_TCLKPOSTCTL(x)	 REG_PUT(x, 15,  8)
#define PHYTIMING1_SET_M_TCLKTRAILCTL(x) REG_PUT(x,  7,  0)

#define PHYTIMING2_SET_M_THSPRPRCTL(x)	REG_PUT(x, 23, 16)
#define PHYTIMING2_SET_M_THSZEROCTL(x)	REG_PUT(x, 15,  8)
#define PHYTIMING2_SET_M_THSTRAILCTL(x)	REG_PUT(x,  7,  0)

#define dsim_read(dsim, reg)		readl(dsim->base + reg)
#define dsim_write(dsim, val, reg)	writel(val, dsim->base + reg)

/* fixed phy ref clk rate */
#define PHY_REF_CLK		12000

#define MAX_MAIN_HRESOL		2047
#define MAX_MAIN_VRESOL		2047
#define MAX_SUB_HRESOL		1024
#define MAX_SUB_VRESOL		1024

/* in KHZ */
#define MAX_ESC_CLK_FREQ	20000

/* dsim all irqs index */
#define PLLSTABLE		1
#define SWRSTRELEASE		2
#define SFRPLFIFOEMPTY		3
#define SFRPHFIFOEMPTY		4
#define SYNCOVERRIDE		5
#define BUSTURNOVER		6
#define FRAMEDONE		7
#define LPDRTOUT		8
#define TATOUT			9
#define RXDATDONE		10
#define RXTE			11
#define RXACK			12
#define ERRRXECC		13
#define ERRRXCRC		14
#define ERRESC3			15
#define ERRESC2			16
#define ERRESC1			17
#define ERRESC0			18
#define ERRSYNC3		19
#define ERRSYNC2		20
#define ERRSYNC1		21
#define ERRSYNC0		22
#define ERRCONTROL3		23
#define ERRCONTROL2		24
#define ERRCONTROL1		25
#define ERRCONTROL0		26

#define MIPI_FIFO_TIMEOUT		250000 /* 250ms */

/* Dispmix Control & GPR Registers */
#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#ifdef CONFIG_IMX8MN
#define MIPI_DSI_I_PRESETn_SFT_EN		BIT(0) | BIT(1)
#else
   #define MIPI_DSI_I_PRESETn_SFT_EN		BIT(5)
#endif
#define DISPLAY_MIX_CLK_EN_CSR			0x04

#ifdef CONFIG_IMX8MN
#define MIPI_DSI_PCLK_SFT_EN		 BIT(0)
#define MIPI_DSI_CLKREF_SFT_EN		 BIT(1)
#else
   #define MIPI_DSI_PCLK_SFT_EN			BIT(8)
   #define MIPI_DSI_CLKREF_SFT_EN		BIT(9)
#endif
#define GPR_MIPI_RESET_DIV			0x08
   /* Clock & Data lanes reset: Active Low */
   #define GPR_MIPI_S_RESETN			BIT(16)
   #define GPR_MIPI_M_RESETN			BIT(17)

#define	PS2KHZ(ps)	(1000000000UL / (ps))

#define MIPI_HFP_PKT_OVERHEAD	6
#define MIPI_HBP_PKT_OVERHEAD	6
#define MIPI_HSA_PKT_OVERHEAD	6

#define to_sec_mipi_dsim(dsi) container_of(dsi, struct sec_mipi_dsim, dsi_host)

/* DSIM PLL configuration from spec:
 *
 * Fout(DDR) = (M * Fin) / (P * 2^S), so Fout / Fin = M / (P * 2^S)
 * Fin_pll   = Fin / P     (6 ~ 12 MHz)
 * S: [2:0], M: [12:3], P: [18:13], so
 * TODO: 'S' is in [0 ~ 3], 'M' is in, 'P' is in [1 ~ 33]
 *
 */

/* used for CEA standard modes */
struct dsim_hblank_par {
	char *name;		/* drm display mode name */
	int vrefresh;
	int hfp_wc;
	int hbp_wc;
	int hsa_wc;
	int lanes;
};

/* DPHY PLL structure */
struct sec_mipi_dsim_range {
	uint32_t min;
	uint32_t max;
};

struct sec_mipi_dsim_pll {
	struct sec_mipi_dsim_range p;
	struct sec_mipi_dsim_range m;
	struct sec_mipi_dsim_range s;
	struct sec_mipi_dsim_range k;
	struct sec_mipi_dsim_range fin;
	struct sec_mipi_dsim_range fpref;
	struct sec_mipi_dsim_range fvco;
};

static int dphy_timing_default_cmp(const void *key, const void *elt)
{
	const struct sec_mipi_dsim_dphy_timing *_key = key;
	const struct sec_mipi_dsim_dphy_timing *_elt = elt;

	/* find an element whose 'bit_clk' is equal to the
	 * the key's 'bit_clk' value or, the difference
	 * between them is less than 5.
	 */
	if (abs((int)(_elt->bit_clk - _key->bit_clk)) <= 5)
		return 0;

	if (_key->bit_clk < _elt->bit_clk)
		/* search bottom half */
		return 1;
	else
		/* search top half */
		return -1;
}

/*
 * DSIM PLL_1432X setting guide from spec:
 *
 * Fout(bitclk) = ((m + k / 65536) * Fin) / (p * 2^s), and
 * p = P[5:0], m = M[9:0], s = S[2:0], k = K[15:0];
 *
 * Fpref = Fin / p
 * Fin: [6MHz ~ 300MHz], Fpref: [2MHz ~ 30MHz]
 *
 * Fvco = ((m + k / 65536) * Fin) / p
 * Fvco: [1050MHz ~ 2100MHz]
 *
 * 1 <= P[5:0] <= 63, 64 <= M[9:0] <= 1023,
 * 0 <= S[2:0] <=  5, -32768 <= K[15:0] <= 32767
 *
 */

const struct sec_mipi_dsim_pll pll_1432x = {
	.p      = { .min = 1,           .max = 63,      },
	.m      = { .min = 64,          .max = 1023,    },
	.s      = { .min = 0,           .max = 5,       },
	.k      = { .min = 0,           .max = 32768,   },      /* abs(k) */
	.fin    = { .min = 6000,        .max = 300000,  },      /* in KHz */
	.fpref  = { .min = 2000,        .max = 30000,   },      /* in KHz */
	.fvco   = { .min = 1050000,     .max = 2100000, },      /* in KHz */
};

struct dsim_pll_pms {
	uint32_t bit_clk;	/* kHz */
	uint32_t p;
	uint32_t m;
	uint32_t s;
	uint32_t k;
};

struct sec_mipi_dsim {
	void __iomem *base;
	struct udevice *dev;

	struct clk *clk_cfg;
	struct clk *clk_pllref;
	struct clk *clk_pixel;

	/* kHz clocks */
	uint64_t pix_clk;
	uint64_t bit_clk;
	uint32_t pref_clk;			/* phy ref clock rate in KHz */

	unsigned int lanes;
	unsigned int channel;			/* virtual channel */
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
	unsigned int c_pms;
	struct dsim_pll_pms pms;
	unsigned long ref_clk;
	struct clk	*dsi_clk;
	unsigned long	frequency;
	int clk_pllref_enable;
	int enabled;
	unsigned long def_pix_clk;
	unsigned long byte_clock;
	unsigned long pixelclock;
	uint32_t hsmult;
	uint32_t mipi_dsi_multiple;


	 struct mipi_dsi_device *device;
	 uint32_t max_data_lanes;
	 uint64_t max_data_rate;

	 struct mipi_dsi_host dsi_host;

	 struct display_timing timings;
};

/*
 * continued fraction
 *      2  1  2  1  2
 * 0 1  2  3  8 11 30
 * 1 0  1  1  3  4 11
 */
static void get_best_ratio_bigger(unsigned long *pnum, unsigned long *pdenom, unsigned max_n, unsigned max_d)
{
	unsigned long a = *pnum;
	unsigned long b = *pdenom;
	unsigned long c;
	unsigned n0 = 0;
	unsigned n1 = 1;
	unsigned d0 = 1;
	unsigned d1 = 0;
	unsigned _n = 0;
	unsigned _d = 1;
	unsigned whole;

	while (b) {
		whole = a / b;
		/* n0/d0 is the earlier term */
		n0 = n0 + (n1 * whole);
		d0 = d0 + (d1 * whole);

		c = a - (b * whole);
		a = b;
		b = c;

		if (b) {
			/* n1/d1 is the earlier term */
			whole = a / b;
			_n = n1 + (n0 * whole);
			_d = d1 + (d0 * whole);
		} else {
			_n = n0;
			_d = d0;
		}
		debug("%s: cf=%i %d/%d, %d/%d\n", __func__, whole, n0, d0, _n, _d);
		if ((_n > max_n) || (_d > max_d)) {
			unsigned h;

			h = n0;
			if (h) {
				_n = max_n - n1;
				_n /= h;
				if (whole > _n)
					whole = _n;
			}
			h = d0;
			if (h) {
				_d = max_d - d1;
				_d /= h;
				if (whole > _d)
					whole = _d;
			}
			_n = n1 + (n0 * whole);
			_d = d1 + (d0 * whole);
			debug("%s: b=%ld, n=%d of %d, d=%d of %d\n", __func__, b, _n, max_n, _d, max_d);
			if (!_d) {
				/* Don't choose infinite for a bigger ratio */
				_n = n0 + 1;
				_d = d0;
				pr_err("%s: %d/%d is too big\n", __func__, _n, _d);
			}
			break;
		}

		if (!b)
			break;
		n1 = _n;
		d1 = _d;
		c = a - (b * whole);
		a = b;
		b = c;
	}

	*pnum = _n;
	*pdenom = _d;
}

#if 0
static const struct dsim_hblank_par *sec_mipi_dsim_get_hblank_par(const char *name,
								  int vrefresh,
								  int lanes)
{
	int i, size;
	const struct dsim_hblank_par *hpar, *hblank;

	pr_debug("%s: name=%s, vrefresh=%d, lanes=%d\n", __func__, name, vrefresh, lanes);
	if (unlikely(!name))
		return NULL;

	switch (lanes) {
	case 1:
		return NULL;
	case 2:
		hblank = hblank_2lanes;
		size   = ARRAY_SIZE(hblank_2lanes);
		break;
	case 4:
		hblank = hblank_4lanes;
		size   = ARRAY_SIZE(hblank_4lanes);
		break;
	default:
		pr_err("No hblank data for mode %s with %d lanes\n",
		       name, lanes);
		return NULL;
	}

	for (i = 0; i < size; i++) {
		hpar = &hblank[i];

		if (!strcmp(name, hpar->name)) {
			if (vrefresh != hpar->vrefresh)
				continue;

			/* found */
			return hpar;
		}
	}

	return NULL;
}

#endif

static int _sec_mipi_dsim_pll_enable(struct sec_mipi_dsim *dsim, int enable)
{
	if (dsim->clk_pllref_enable == enable)
		return 0;

	if (enable)
		clk_prepare_enable(dsim->clk_pllref);
	else
		clk_disable_unprepare(dsim->clk_pllref);
	dsim->clk_pllref_enable = enable;
	debug("%s: enable=%d\n", __func__, enable);
	return 1;
}

#define MIN_FREQ	6000000

unsigned fixup_div(unsigned long clk, unsigned div)
{
	unsigned max = clk / MIN_FREQ;

	while ((div >= max * 7 / 2) && !(div % 7))
		div /= 7;
	while ((div >= max * 5 / 2) && !(div % 5))
		div /= 5;
	while ((div >= max * 3 / 2) && !(div % 3))
		div /= 3;
	while ((div >= max * 2 / 2) && !(div % 2))
		div >>= 1;

	while (div > max) {
		if (!(div % 2))
			div >>= 1;
		else if (!(div % 3))
			div /=  3;
		else if (!(div % 5))
			div /= 5;
		else if (!(div % 7))
			div /= 7;
		else
			div = max;
	}
	return div;
}

static int sec_mipi_choose_ref_clk(struct sec_mipi_dsim *dsim, unsigned long pix_clk)
{
	unsigned long ref_clk, bit_clk;
	struct clk *clk_ref_parent;
	unsigned bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);
	unsigned div = 0;
	unsigned ref_parent_clk;

	if (bpp < 0)
		return -EINVAL;

	bit_clk = DIV_ROUND_UP_ULL((u64)pix_clk * bpp, dsim->lanes);
	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_MBC) {
		unsigned n = (bpp + (dsim->lanes * 8) - 1) / (dsim->lanes * 8);
		unsigned bit_clkm = pix_clk * n * 8;

		if (bit_clk < bit_clkm) {
			bit_clk = bit_clkm;
			debug("%s: %ld = %ld * 8 * %d(MBC)\n", __func__, bit_clk, pix_clk, n);
		}
	}
	if (dsim->mipi_dsi_multiple) {
		bit_clk += dsim->mipi_dsi_multiple - 1;
		bit_clk /= dsim->mipi_dsi_multiple;
		bit_clk *= dsim->mipi_dsi_multiple;
	}
	clk_ref_parent = clk_get_parent(dsim->clk_pllref);
	if (IS_ERR(clk_ref_parent)) {
		printf("%s: clk_get_parent(%ld) failed %ld\n", __func__,
				dsim->clk_pllref->id, PTR_ERR(clk_ref_parent));
		clk_ref_parent = NULL;
	}

	ref_parent_clk = ref_clk = bit_clk;
	if (clk_ref_parent) {
		ref_parent_clk = clk_get_rate(clk_ref_parent);

		if (ref_parent_clk)
			div = (ref_parent_clk + (pix_clk >> 1)) / pix_clk;

		debug("%s: ref_clk=%ld, ref_parent_clk=%d, pix_clk=%ld, div=%d\n",
			__func__, ref_clk, ref_parent_clk, pix_clk, div);
	}
	debug("%s: %ld = %d/%d\n", __func__, ref_clk, ref_parent_clk, div);
	if (div) {
		div = fixup_div(ref_parent_clk, div);
		ref_clk = (ref_parent_clk + (div >> 1)) / div;
		debug("%s: %ld = %d/%d\n", __func__, ref_clk, ref_parent_clk, div);
	} else {
		while (ref_clk < MIN_FREQ)
			ref_clk <<= 1;
	}
	while (ref_clk > 6000000 * 33) {
		ref_clk >>= 1;
	}
	return ref_clk;
}

static int sec_mipi_dsim_set_pref_rate(struct sec_mipi_dsim *dsim, unsigned long pix_clk)
{
	int ret;
	uint32_t rate, diff;
	uint32_t ref_clk;
	struct udevice *dev = dsim->dev;
	const struct sec_mipi_dsim_pll *dpll = &pll_1432x;
	const struct sec_mipi_dsim_range *fin_range = &dpll->fin;

	ret = dev_read_u32(dev, "pref-rate", &rate);
	if (ret < 0) {
		dev_dbg(dev, "no valid rate assigned for pref clock\n");
		ref_clk = sec_mipi_choose_ref_clk(dsim, pix_clk);
	} else {
		if (unlikely(rate < fin_range->min || rate > fin_range->max)) {
			dev_warn(dev, "pref-rate get is invalid: %uKHz\n",
				 rate);
			ref_clk = PHY_REF_CLK * 1000;
		} else
			ref_clk = rate * 1000;
	}

	while (1) {
		if (dsim->clk_pllref_enable) {
			_sec_mipi_dsim_pll_enable(dsim, 0);
			ret = clk_set_rate(dsim->clk_pllref, ref_clk);
			_sec_mipi_dsim_pll_enable(dsim, 1);
		} else {
			ret = clk_set_rate(dsim->clk_pllref, ref_clk);
		}
		rate = ret;
		diff = (rate > ref_clk) ? rate - ref_clk : ref_clk - rate;
		if (diff > 500)
			dev_err(dev, "failed to set pll ref clock(%ld) rate %d %d\n", dsim->clk_pllref->id, ref_clk, ret);

		rate = clk_get_rate(dsim->clk_pllref);
		debug("%s: ref_clk=%d %d\n", __func__, ref_clk, rate);
		if (unlikely(!rate)) {
			dev_err(dev, "failed to get pll ref clock rate\n");
			return -EINVAL;
		}

		diff = (rate > ref_clk) ? rate - ref_clk : ref_clk - rate;
		if (diff <= 500) {
			dsim->pref_clk = rate / 1000;
			dsim->ref_clk = rate;
			return 0;

		}
		if (unlikely(ref_clk == PHY_REF_CLK * 1000))
			break;

		dev_warn(dev, "invalid assigned rate for pref: %uKHz\n",
			 dsim->pref_clk);
		dev_warn(dev, "use default pref rate instead: %uKHz\n",
			 PHY_REF_CLK);

		ref_clk = PHY_REF_CLK * 1000;
	}
	/* set default rate failed */
	dev_err(dev, "no valid pll ref clock rate\n");
	return -EINVAL;
}

int sec_mipi_dsim_pll_enable(struct sec_mipi_dsim *dsim, int enable);

static int sec_mipi_dsim_bridge_clk_set(struct sec_mipi_dsim *dsim)
{
	int bpp;
	uint64_t pix_clk, bit_clk;

	bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);
	if (bpp < 0)
		return -EINVAL;

	pix_clk = dsim->timings.pixelclock.typ;
	bit_clk = DIV_ROUND_UP_ULL(pix_clk * bpp, dsim->lanes);

#if 0
	if (bit_clk > dsim->max_data_rate) {
		printf("request bit clk freq exceeds lane's maximum value\n");
		return -EINVAL;
	}
#endif

	dsim->pix_clk = DIV_ROUND_UP_ULL(pix_clk, 1000);
	dsim->bit_clk = DIV_ROUND_UP_ULL(bit_clk, 1000);

	debug("%s: bitclk %llu pixclk %llu\n", __func__, dsim->bit_clk, dsim->pix_clk);
	sec_mipi_dsim_pll_enable(dsim, 1);

	return 0;
}

static int sec_mipi_dsim_bridge_enable(struct sec_mipi_dsim *dsim);

/* For now, dsim only support one device attached */
static int sec_mipi_dsim_host_attach(struct mipi_dsi_host *host,
				     struct mipi_dsi_device *dsi)
{
	struct sec_mipi_dsim *dsim = to_sec_mipi_dsim(host);

	if (!dsi->lanes || dsi->lanes > dsim->max_data_lanes) {
		printf("invalid data lanes number\n");
		return -EINVAL;
	}

	if (!(dsi->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		printf("unsupported dsi mode %lx\n", dsi->mode_flags);
		return -EINVAL;
	}

	if (dsi->format != MIPI_DSI_FMT_RGB888 &&
	    dsi->format != MIPI_DSI_FMT_RGB565 &&
	    dsi->format != MIPI_DSI_FMT_RGB666 &&
	    dsi->format != MIPI_DSI_FMT_RGB666_PACKED) {
		printf("unsupported pixel format: %#x\n", dsi->format);
		return -EINVAL;
	}

	dsim->lanes	 = dsi->lanes;
	dsim->channel	 = dsi->channel;
	dsim->format	 = dsi->format;
	dsim->mode_flags = dsi->mode_flags;
	dsim->def_pix_clk = dsi->def_pix_clk;
	dsim->hsmult	 = dsi->hsmult;
	dsim->mipi_dsi_multiple = dsi->mipi_dsi_multiple;
	dsim->max_data_rate  = 1500000000ULL;

	debug("lanes %u, channel %u, format 0x%x, mode_flags 0x%lx\n", dsim->lanes,
		dsim->channel, dsim->format, dsim->mode_flags);

	sec_mipi_dsim_bridge_clk_set(dsim);
	sec_mipi_dsim_bridge_enable(dsim);

	return 0;
}

static void sec_mipi_dsim_config_clkctrl(struct sec_mipi_dsim *dsim);

static int sec_mipi_dsim_host_enable_lpm(struct mipi_dsi_host *host)
{
	struct sec_mipi_dsim *dsim = to_sec_mipi_dsim(host);

	/* config esc clock, byte clock and etc */
	sec_mipi_dsim_config_clkctrl(dsim);

	return 0;
}

static void sec_mipi_dsim_set_standby(struct sec_mipi_dsim *dsim, bool standby);

static int sec_mipi_dsim_host_enable_frame(struct mipi_dsi_host *host)
{
	struct sec_mipi_dsim *dsim = to_sec_mipi_dsim(host);

	/* enable data transfer of dsim */
	sec_mipi_dsim_set_standby(dsim, true);
	return 0;
}

static void sec_mipi_dsim_config_cmd_lpm(struct sec_mipi_dsim *dsim,
					 bool enable)
{
	uint32_t escmode;

	escmode = dsim_read(dsim, DSIM_ESCMODE);

	if (enable)
		escmode |= ESCMODE_CMDLPDT;
	else
		escmode &= ~ESCMODE_CMDLPDT;

	dsim_write(dsim, escmode, DSIM_ESCMODE);
}

static void sec_mipi_dsim_write_pl_to_sfr_fifo(struct sec_mipi_dsim *dsim,
					       const void *payload,
					       size_t length)
{
	uint32_t pl_data;

	if (!length)
		return;

	while (length >= 4) {
		pl_data = get_unaligned_le32(payload);
		dsim_write(dsim, pl_data, DSIM_PAYLOAD);
		payload += 4;
		length -= 4;
	}

	pl_data = 0;
	switch (length) {
	case 3:
		pl_data |= ((u8 *)payload)[2] << 16;
		/* fall through */
	case 2:
		pl_data |= ((u8 *)payload)[1] << 8;
		/* fall through */
	case 1:
		pl_data |= ((u8 *)payload)[0];
		dsim_write(dsim, pl_data, DSIM_PAYLOAD);
		break;
	}
}

static void sec_mipi_dsim_write_ph_to_sfr_fifo(struct sec_mipi_dsim *dsim,
					       void *header,
					       bool use_lpm)
{
	uint32_t pkthdr;

	pkthdr = PKTHDR_SET_DATA1(((u8 *)header)[2])	| /* WC MSB  */
		 PKTHDR_SET_DATA0(((u8 *)header)[1])	| /* WC LSB  */
		 PKTHDR_SET_DI(((u8 *)header)[0]);	  /* Data ID */

	dsim_write(dsim, pkthdr, DSIM_PKTHDR);
}

static int sec_mipi_dsim_read_pl_from_sfr_fifo(struct sec_mipi_dsim *dsim,
					       void *payload,
					       size_t length)
{
	uint8_t data_type;
	uint16_t word_count = 0;
	uint32_t fifoctrl, ph, pl;
	int extra;
	unsigned char *dst = payload;

	fifoctrl = dsim_read(dsim, DSIM_FIFOCTRL);

	if (WARN_ON(fifoctrl & FIFOCTRL_EMPTYRX))
		return -EINVAL;

	ph = dsim_read(dsim, DSIM_RXFIFO);
	data_type = PKTHDR_GET_DT(ph);
	switch (data_type) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		dev_err(dsim->dev, "peripheral report error: (0-7)%x, (8-15)%x 0x%08x\n",
			PKTHDR_GET_DATA0(ph), PKTHDR_GET_DATA1(ph), ph);
		fifoctrl = dsim_read(dsim, DSIM_FIFOCTRL);
		if (!(fifoctrl & FIFOCTRL_EMPTYRX)) {
			ph = dsim_read(dsim, DSIM_RXFIFO);
			dev_err(dsim->dev, "0x%08x\n", ph);
		}
		return -EPROTO;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		if (!WARN_ON(length < 2)) {
			dst[1] = PKTHDR_GET_DATA1(ph);
			word_count++;
		}
		/* fall through */
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		dst[0] = PKTHDR_GET_DATA0(ph);
		word_count++;
		length = word_count;
		break;
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
		word_count = PKTHDR_GET_WC(ph);
		extra = 0;
		if (word_count > length) {
			extra = ((word_count + 3) >> 2) - ((length + 3) >> 2);
			dev_err(dsim->dev, "invalid receive buffer length, %d vs %ld, 0x%08x\n", word_count, length, ph);
			word_count = length;
			length = -EINVAL;
		} else {
			length = word_count;
		}

		while (word_count >= 4) {
			pl = dsim_read(dsim, DSIM_RXFIFO);
			dst[0] = pl & 0xff;
			dst[1] = (pl >> 8)  & 0xff;
			dst[2] = (pl >> 16) & 0xff;
			dst[3] = (pl >> 24) & 0xff;
			dst += 4;
			word_count -= 4;
		}

		if (word_count > 0) {
			pl = dsim_read(dsim, DSIM_RXFIFO);

			switch (word_count) {
			case 3:
				dst[2] = (pl >> 16) & 0xff;
				/* fall through */
			case 2:
				dst[1] = (pl >> 8) & 0xff;
				/* fall through */
			case 1:
				dst[0] = pl & 0xff;
				break;
			}
		}
		while (extra) {
			pl = dsim_read(dsim, DSIM_RXFIFO);
			dev_err(dsim->dev, "extra, 0x%x\n", pl);
			extra--;
		}

		break;
	default:
		return -EINVAL;
	}

	return length;
}

static int sec_mipi_dsim_wait_for_src(struct sec_mipi_dsim *dsim, unsigned src, unsigned long timeout)
{
	uint32_t intsrc;

	do {
		intsrc = dsim_read(dsim, DSIM_INTSRC);
		if (intsrc) {
			dsim_write(dsim, intsrc, DSIM_INTSRC);
			if (intsrc & src)
				return 0;
		}

		udelay(1);
	} while (--timeout);

	return -ETIMEDOUT;
}

static ssize_t sec_mipi_dsim_host_transfer(struct mipi_dsi_host *host,
					   const struct mipi_dsi_msg *msg)
{
	int ret, nb_bytes;
	bool use_lpm;
	struct mipi_dsi_packet packet;
	struct sec_mipi_dsim *dsim = to_sec_mipi_dsim(host);

	if ((msg->rx_buf && !msg->rx_len) || (msg->rx_len && !msg->rx_buf))
		return -EINVAL;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret) {
		dev_err(dsim->dev, "failed to create dsi packet: %d\n", ret);
		return ret;
	}

	/* config LPM for CMD TX */
	use_lpm = msg->flags & MIPI_DSI_MSG_USE_LPM ? true : false;
	sec_mipi_dsim_config_cmd_lpm(dsim, use_lpm);

	if (packet.payload_length) {		/* Long Packet case */
		/* write packet payload */
		sec_mipi_dsim_write_pl_to_sfr_fifo(dsim,
						   packet.payload,
						   packet.payload_length);

		/* write packet header */
		sec_mipi_dsim_write_ph_to_sfr_fifo(dsim,
						   packet.header,
						   use_lpm);

		ret = sec_mipi_dsim_wait_for_src(dsim, INTSRC_SFRPLFIFOEMPTY,
						MIPI_FIFO_TIMEOUT);
		if (ret) {
			dev_err(dsim->dev, "wait tx done timeout!\n");
			return -EBUSY;
		}
	} else {
		/* write packet header */
		sec_mipi_dsim_write_ph_to_sfr_fifo(dsim,
						   packet.header,
						   use_lpm);

		ret = sec_mipi_dsim_wait_for_src(dsim, INTSRC_SFRPHFIFOEMPTY,
						MIPI_FIFO_TIMEOUT);
		if (ret) {
			dev_err(dsim->dev, "wait pkthdr tx done time out\n");
			return -EBUSY;
		}
	}

	/* read packet payload */
	if (unlikely(msg->rx_buf)) {
		ret = sec_mipi_dsim_wait_for_src(dsim, INTSRC_RXDATDONE,
						  MIPI_FIFO_TIMEOUT);
		if (ret) {
			dev_err(dsim->dev, "wait rx done time out\n");
			return -EBUSY;
		}

		ret = sec_mipi_dsim_read_pl_from_sfr_fifo(dsim,
							  msg->rx_buf,
							  msg->rx_len);
		if (ret < 0)
			return ret;
		nb_bytes = msg->rx_len;
	} else {
		nb_bytes = packet.size;
	}

	return nb_bytes;

}

static const struct mipi_dsi_host_ops sec_mipi_dsim_host_ops = {
	.attach = sec_mipi_dsim_host_attach,
	.enable_lpm = sec_mipi_dsim_host_enable_lpm,
	.enable_frame = sec_mipi_dsim_host_enable_frame,
	.transfer = sec_mipi_dsim_host_transfer,
};

static int sec_mipi_dsim_wait_pll_stable(struct sec_mipi_dsim *dsim)
{
	uint32_t status;
	ulong start;

	start = get_timer(0);	/* Get current timestamp */

	do {
		status = dsim_read(dsim, DSIM_STATUS);
		if (status & STATUS_PLLSTABLE)
			return 0;
	} while (get_timer(0) < (start + 100)); /* Wait 100ms */

	return -ETIMEDOUT;
}

static int sec_mipi_dsim_config_pll(struct sec_mipi_dsim *dsim)
{
	int ret;
	uint32_t pllctrl = 0, status, data_lanes_en, stop;

	dsim_write(dsim, 0x8000, DSIM_PLLTMR);

	/* TODO: config dp/dn swap if requires */

	pllctrl |= PLLCTRL_SET_PMS(dsim->c_pms) | PLLCTRL_PLLEN;
	debug("%s: pllctrl=%x\n", __func__, pllctrl);
	dsim_write(dsim, pllctrl, DSIM_PLLCTRL);

	ret = sec_mipi_dsim_wait_pll_stable(dsim);
	if (ret) {
		printf("wait for pll stable time out\n");
		return ret;
	}

	/* wait for clk & data lanes to go to stop state */
	mdelay(1);

	data_lanes_en = (0x1 << dsim->lanes) - 1;
	status = dsim_read(dsim, DSIM_STATUS);
	if (!(status & STATUS_STOPSTATECLK)) {
		printf("clock is not in stop state\n");
		return -EBUSY;
	}

	stop = STATUS_GET_STOPSTATEDAT(status);
	if ((stop & data_lanes_en) != data_lanes_en) {
		printf("one or more data lanes is not in stop state\n");
		return -EBUSY;
	}

	return 0;
}

static unsigned pix_to_delay_byte_clocks(struct sec_mipi_dsim *dsim, unsigned pixels, int base, int min,
		unsigned *pix_cnt, unsigned *hs_clk_cnt)
{
	unsigned n;
	unsigned long a;

	*pix_cnt += pixels;
	a = *pix_cnt;
	a *= dsim->byte_clock;
	a += (dsim->pixelclock >> 1);
	a /= dsim->pixelclock;
	n = a;
	debug("%s:pix_cnt = %d, byte_clock = %ld, pixelclock = %ld, n=%d\n", __func__, *pix_cnt, dsim->byte_clock, dsim->pixelclock, n);

//	n *= dsim->lanes;
	n -= *hs_clk_cnt;

	if (n >= base + min)
		n -= base;
	else
		n = min;
	*hs_clk_cnt += n + base;

	return n;
}

static unsigned pix_to_delay_byte_clocks_burst(struct sec_mipi_dsim *dsim, unsigned pixels, unsigned bpp,
		unsigned *pix_cnt, unsigned *hs_clk_cnt)
{
	unsigned n;

	*pix_cnt += pixels;
	n = ((pixels * bpp) + (dsim->lanes * 8) - 1) / (dsim->lanes * 8);
//	n *= dsim->lanes;
	*hs_clk_cnt += n;

	return n;
}

static void sec_mipi_dsim_set_main_mode(struct sec_mipi_dsim *dsim)
{
	uint32_t bpp, hfp_wc, hbp_wc, hsa_wc;
	uint32_t mdresol = 0, mvporch = 0, mhporch = 0, msync = 0;
	struct display_timing *timings = &dsim->timings;
	unsigned pix_cnt = 0;
	unsigned hs_clk_cnt = 0;

	debug("%s: hfp=%d hbp=%d hsync=%d byte_clock=%ld pixelclock=%ld\n", __func__,
		timings->hfront_porch.typ, timings->hback_porch.typ, timings->hsync_len.typ,
		dsim->byte_clock, dsim->pixelclock);
	mdresol |= MDRESOL_SET_MAINVRESOL(timings->vactive.typ) |
		   MDRESOL_SET_MAINHRESOL(timings->hactive.typ);
	debug("%s: mdresol=%x\n", __func__, mdresol);
	dsim_write(dsim, mdresol, DSIM_MDRESOL);

	mvporch |= MVPORCH_SET_MAINVBP(timings->vback_porch.typ)    |
		   MVPORCH_SET_STABLEVFP(timings->vfront_porch.typ) |
		   MVPORCH_SET_CMDALLOW(0xf);
	debug("%s: mvporch=%x\n", __func__, mvporch);
	dsim_write(dsim, mvporch, DSIM_MVPORCH);

	bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);

	/* calculate hfp & hbp word counts */
	if (1) {
		hbp_wc = pix_to_delay_byte_clocks(dsim, timings->hback_porch.typ, (dsim->lanes <= 1) ? 13 : 7, 1, &pix_cnt, &hs_clk_cnt);
		pix_to_delay_byte_clocks_burst(dsim, timings->hactive.typ, bpp, &pix_cnt, &hs_clk_cnt);
		hfp_wc = pix_to_delay_byte_clocks(dsim, timings->hfront_porch.typ, (dsim->lanes <= 1) ? 11 : 4, 0, &pix_cnt, &hs_clk_cnt);
		hsa_wc = pix_to_delay_byte_clocks(dsim, timings->hsync_len.typ, 10/dsim->lanes, 0, &pix_cnt, &hs_clk_cnt);
	} else {
#if 0
		hbp_wc = dsim->hpar->hbp_wc;
		hfp_wc = dsim->hpar->hfp_wc;
		hsa_wc = dsim->hpar->hsa_wc;
#endif
	}

	mhporch |= MHPORCH_SET_MAINHFP(hfp_wc) |
		   MHPORCH_SET_MAINHBP(hbp_wc);

	debug("%s: mhporch=%x\n", __func__, mhporch);
	dsim_write(dsim, mhporch, DSIM_MHPORCH);

	msync |= MSYNC_SET_MAINVSA(timings->vsync_len.typ) |
		 MSYNC_SET_MAINHSA(hsa_wc);

	debug("%s: hfp_wc %u hbp_wc %u hsa_wc %u\n", __func__, hfp_wc, hbp_wc, hsa_wc);

	debug("%s: msync=%x\n", __func__, msync);
	dsim_write(dsim, msync, DSIM_MSYNC);
}

static void sec_mipi_dsim_config_dpi(struct sec_mipi_dsim *dsim)
{
	uint32_t config = 0, rgb_status = 0, data_lanes_en;

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO)
		rgb_status &= ~RGB_STATUS_CMDMODE_INSEL;
	else
		rgb_status |= RGB_STATUS_CMDMODE_INSEL;

	dsim_write(dsim, rgb_status, DSIM_RGB_STATUS);

	if (dsim->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		config |= CONFIG_NON_CONTINUOUS_CLOCK_LANE;
		config |= CONFIG_CLKLANE_STOP_START;
	}

	if (dsim->mode_flags & MIPI_DSI_MODE_VSYNC_FLUSH)
		config |= CONFIG_MFLUSH_VS;

	/* disable EoT packets in HS mode */
	if (dsim->mode_flags & MIPI_DSI_MODE_EOT_PACKET)
		config |= CONFIG_EOT_R03;

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO) {
		config |= CONFIG_VIDEOMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			config |= CONFIG_BURSTMODE;

		else if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			config |= CONFIG_SYNCINFORM;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_AUTO_VERT)
			config |= CONFIG_AUTOMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HSE)
			config |= CONFIG_HSEDISABLEMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HFP)
			config |= CONFIG_HFPDISABLEMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HBP)
			config |= CONFIG_HBPDISABLEMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HSA)
			config |= CONFIG_HSADISABLEMODE;
	}

	config |= CONFIG_SET_MAINVC(dsim->channel);

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO) {
		switch (dsim->format) {
		case MIPI_DSI_FMT_RGB565:
			config |= CONFIG_SET_MAINPIXFORMAT(0x4);
			break;
		case MIPI_DSI_FMT_RGB666_PACKED:
			config |= CONFIG_SET_MAINPIXFORMAT(0x5);
			break;
		case MIPI_DSI_FMT_RGB666:
			config |= CONFIG_SET_MAINPIXFORMAT(0x6);
			break;
		case MIPI_DSI_FMT_RGB888:
			config |= CONFIG_SET_MAINPIXFORMAT(0x7);
			break;
		default:
			config |= CONFIG_SET_MAINPIXFORMAT(0x7);
			break;
		}
	}

	/* config data lanes number and enable lanes */
	data_lanes_en = (0x1 << dsim->lanes) - 1;
	config |= CONFIG_SET_NUMOFDATLANE(dsim->lanes - 1);
	config |= CONFIG_SET_LANEEN(0x1 | data_lanes_en << 1);

	debug("DSIM config 0x%x\n", config);

	dsim_write(dsim, config, DSIM_CONFIG);
}

static void sec_mipi_dsim_config_dphy(struct sec_mipi_dsim *dsim)
{
	struct sec_mipi_dsim_dphy_timing key = { 0 };
	const struct sec_mipi_dsim_dphy_timing *match = NULL;
	uint32_t phytiming = 0, phytiming1 = 0, phytiming2 = 0, timeout = 0;
	uint32_t hactive, vactive;
	struct display_timing *t = &dsim->timings;

	const struct sec_mipi_dsim_dphy_timing *dphy_timing = dphy_timing_ln14lpp_v1p2;
	uint32_t num_dphy_timing = ARRAY_SIZE(dphy_timing_ln14lpp_v1p2);

	key.bit_clk = DIV_ROUND_CLOSEST_ULL(dsim->bit_clk, 1000);

	/* '1280x720@60Hz' mode with 2 data lanes
	 * requires special fine tuning for DPHY
	 * TIMING config according to the tests.
	 */
	if (dsim->lanes == 2) {
		hactive = t->hactive.typ;
		vactive = t->vactive.typ;

		if (hactive == 1280 && vactive == 720) {
			u32 refresh = t->pixelclock.typ /
				((hactive + t->hfront_porch.typ + t->hback_porch.typ + t->hsync_len.typ) *
				(vactive + t->vfront_porch.typ + t->vback_porch.typ + t->vsync_len.typ));
			if (refresh == 60)
				key.bit_clk >>= 1;
		}
	}

	match = bsearch(&key, dphy_timing, num_dphy_timing,
			sizeof(struct sec_mipi_dsim_dphy_timing),
			dphy_timing_default_cmp);
	if (!match)
		return;

	debug("%s:bit_clk=%d, prepare=%d zero=%d post=%d trail=%d"
		" hs_prepare=%d hs_zero=%d hs_trail=%d lpx=%d hs_exit=%d\n",
		__func__, match->bit_clk, match->clk_prepare, match->clk_zero,
		match->clk_post, match->clk_trail,
		match->hs_prepare, match->hs_zero, match->hs_trail,
		match->lpx, match->hs_exit);
	phytiming  |= PHYTIMING_SET_M_TLPXCTL(match->lpx)	|
		      PHYTIMING_SET_M_THSEXITCTL(match->hs_exit);
	debug("%s: phytiming=%x\n", __func__, phytiming);
	dsim_write(dsim, phytiming, DSIM_PHYTIMING);

	phytiming1 |= PHYTIMING1_SET_M_TCLKPRPRCTL(match->clk_prepare)	|
		      PHYTIMING1_SET_M_TCLKZEROCTL(match->clk_zero)	|
		      PHYTIMING1_SET_M_TCLKPOSTCTL(match->clk_post)	|
		      PHYTIMING1_SET_M_TCLKTRAILCTL(match->clk_trail);
	debug("%s: phytiming1=%x\n", __func__, phytiming1);
	dsim_write(dsim, phytiming1, DSIM_PHYTIMING1);

	phytiming2 |= PHYTIMING2_SET_M_THSPRPRCTL(match->hs_prepare)	|
		      PHYTIMING2_SET_M_THSZEROCTL(match->hs_zero)	|
		      PHYTIMING2_SET_M_THSTRAILCTL(match->hs_trail);
	debug("%s: phytiming2=%x\n", __func__, phytiming2);
	dsim_write(dsim, phytiming2, DSIM_PHYTIMING2);

	timeout |= TIMEOUT_SET_BTAOUT(0xff)	|
		   TIMEOUT_SET_LPDRTOUT(0xff);
	debug("%s: timeout=%x\n", __func__, timeout);
	dsim_write(dsim, timeout, DSIM_TIMEOUT);
}

static void sec_mipi_dsim_init_fifo_pointers(struct sec_mipi_dsim *dsim)
{
	uint32_t fifoctrl, fifo_ptrs;

	fifoctrl = dsim_read(dsim, DSIM_FIFOCTRL);

	fifo_ptrs = FIFOCTRL_NINITRX	|
		    FIFOCTRL_NINITSFR	|
		    FIFOCTRL_NINITI80	|
		    FIFOCTRL_NINITSUB	|
		    FIFOCTRL_NINITMAIN;

	fifoctrl &= ~fifo_ptrs;
	dsim_write(dsim, fifoctrl, DSIM_FIFOCTRL);
	udelay(500);

	fifoctrl |= fifo_ptrs;
	dsim_write(dsim, fifoctrl, DSIM_FIFOCTRL);
	udelay(500);
}

static void sec_mipi_dsim_config_clkctrl(struct sec_mipi_dsim *dsim)
{
	uint32_t clkctrl = 0, data_lanes_en;
	uint32_t byte_clk, esc_prescaler;

	clkctrl |= CLKCTRL_TXREQUESTHSCLK;
#if 0
	/* 0 means using 1.5Gbps PHY */
	if (dsim->bit_clk <= 700000)
		clkctrl |= CLKCTRL_DPHY_SEL_1G;
	clkctrl &= ~CLKCTRL_PLLBYPASS;
#endif
	clkctrl |= CLKCTRL_ESCCLKEN;


	clkctrl |= CLKCTRL_BYTECLKSRC_DPHY_PLL;

	clkctrl |= CLKCTRL_BYTECLKEN;

	data_lanes_en = (0x1 << dsim->lanes) - 1;
	clkctrl |= CLKCTRL_SET_LANEESCCLKEN(0x1 | data_lanes_en << 1);

	/* calculate esc prescaler from byte clock:
	 * EscClk = ByteClk / EscPrescaler;
	 */
	byte_clk = dsim->bit_clk >> 3;
	esc_prescaler = DIV_ROUND_UP(byte_clk, MAX_ESC_CLK_FREQ);
	clkctrl |= CLKCTRL_SET_ESCPRESCALER(esc_prescaler);
	debug("%s: bit_clk=%lld, clkctrl=%x\n", __func__, dsim->bit_clk, clkctrl);

	dsim_write(dsim, clkctrl, DSIM_CLKCTRL);
}

static void sec_mipi_dsim_set_standby(struct sec_mipi_dsim *dsim,
				      bool standby)
{
	uint32_t mdresol = 0;

	mdresol = dsim_read(dsim, DSIM_MDRESOL);

	if (standby)
		mdresol |= MDRESOL_MAINSTANDBY;
	else
		mdresol &= ~MDRESOL_MAINSTANDBY;

	dsim_write(dsim, mdresol, DSIM_MDRESOL);
	debug("%s: mdresol=%x\n", __func__, mdresol);
}

static int sec_mipi_dsim_get_pms(struct sec_mipi_dsim *dsim, unsigned long bit_clk, unsigned long ref_clk)
{
	struct dsim_pll_pms *pms = &dsim->pms;
	unsigned long b = bit_clk;
	unsigned long numerator;
	unsigned long denominator;
	unsigned p,m, p_min, p_max;
	unsigned int c_pms;
	int shift = 0;
	int s;
	int i;

#if 0
	bit_clk = bit_clk * 4 / 3;
#endif
#define INPUT_MIN_FREQ	6000000
#define INPUT_MAX_FREQ	12000000
	/* Table 13-51 */
	/* Fin 6-200 MHz */
	/* Fin_pll = Fin /p = 6-12 MHz */
	/* VCO_out = Fin_pll * m = 350 MHz to 750 MHz */
	/* Fout = VCO_out >> s = 37.5 MHz to 750 MHz */
	/* p ranges between 1 and 33 */
	/* m ranges between 25 and 125 */
	/* s ranges between 1 and 7 */
	p_max = ref_clk / INPUT_MIN_FREQ;
	if (p_max > 33)
		p_max = 33;
	p_min = (ref_clk + INPUT_MAX_FREQ - 1) / INPUT_MAX_FREQ;
	if (!p_min)
		p_min = 1;
	do {
		numerator = bit_clk << shift;
		/*
		 * Increase ref clock so that finding bigger will also find
		 * very slightly smaller. Needed for rounding errors in
		 * (pixel clock * 24)
		 */
		denominator = ref_clk + 2;
		get_best_ratio_bigger(&numerator, &denominator, 125, p_max << (7 - shift));
		denominator <<= shift;
		s = __ffs(denominator);
		if (s > 7)
			s = 7;
		p = denominator >> s;
		m = numerator;
		while (s && !(m & 1)) {
			m >>= 1;
			s--;
		}
		if ((denominator >= p_min) && (p <= p_max))
			break;
		i = 2;
		while ((m * i <= 125) && ((p * i) <= p_max)) {
			if ((p << s) * i >= p_min) {
				p *= i;
				m *= i;
				break;
			}
			i++;
		}
		if (((p << s) >= p_min) && (p <= p_max))
			break;
		shift++;
	} while (shift < 8);

	debug("%s: %ld/%ld = %ld/%ld p_min=%d p_max=%d p=%d m=%d s=%d \n", __func__, numerator, denominator, bit_clk, ref_clk, p_min, p_max, p, m, s);
	if (!p) {
		printf("%s: bit_clk=%ld ref_clk=%ld, numerator=%ld, denominator=%ld\n",
			__func__, bit_clk, ref_clk, numerator, denominator);
		return -EINVAL;
	}
	while (p < p_min) {
		if (s)
			s--;
		else if (m <= 125/2)
			m <<= 1;
		else {
			m = (m * p_min + p - 1) / p;
			p = p_min;
			break;
		}
		p <<= 1;
	}
	while (m > 125) {
		if (s) {
			s--;
		} else if ((p >> 1) >= p_min) {
			p >>= 1;
		} else {
			m = 125;
			break;
		}
		m = (m + 1) >> 1;
	}
#define OUTPUT_MIN_FREQ	350000000
#define OUTPUT_MAX_FREQ	750000000
	while ((ref_clk * m < OUTPUT_MIN_FREQ * p) || (m < 25)) {
		if (m <= 125/2)
			m <<= 1;
		else if (p >> 1)
			p >>= 1;
		else
			break;
		s++;
	}
	while (ref_clk * m > OUTPUT_MAX_FREQ * p) {
		if (!s)
			break;
		if ((p << 1) <= p_max)
			p <<= 1;
		else
			m = (m + 1) >> 1;
		s--;
	}

	if (p < 1  || p > 33 ||
	    m < 25 || m > 125 ||
	    s < 0  || s > 7) {
		printf("%s: bit_clk=%ld ref_clk=%ld, p=%d, m=%d, s=%d\n",
			__func__, bit_clk, ref_clk, p, m, s);
		return -EINVAL;
	}
	bit_clk = (ref_clk * m / p) >> s;
	c_pms =	PLLCTRL_SET_P(p) |
		    PLLCTRL_SET_M(m) |
		    PLLCTRL_SET_S(s);
	dsim->c_pms = c_pms;
	if ((pms->p != p) || (pms->m != m) || (pms->s != s) || (pms->bit_clk != bit_clk)) {
		pms->p = p;
		pms->m = m;
		pms->s = s;
		pms->bit_clk = bit_clk;
		debug("%s: bit_clk=%ld %ld ref_clk=%ld, p=%d, m=%d, s=%d, lanes=%d\n",
			__func__, b, bit_clk, ref_clk, p, m, s, dsim->lanes);
	}
	/* Divided by 2 because mipi output clock is DDR */
	dsim->frequency = bit_clk / 2;
	dsim->byte_clock = bit_clk >> 3;
	dsim->bit_clk = DIV_ROUND_UP_ULL(bit_clk, 1000);
	return 0;
}

static int _sec_mipi_dsim_check_pll_out(struct sec_mipi_dsim *dsim)
{
	int bpp;
	unsigned long pix_clk;
	uint32_t bit_clk;
	int ret;

	bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);
	if (bpp < 0)
		return -EINVAL;

	pix_clk = clk_get_rate(dsim->clk_pixel);
	if (!pix_clk)
		pix_clk = dsim->def_pix_clk;

	bit_clk = DIV_ROUND_UP_ULL((u64)pix_clk * bpp, dsim->lanes);
	debug("%s: %d = %ld * %d / %d\n", __func__, bit_clk, pix_clk, bpp, dsim->lanes);

	if (bit_clk > dsim->max_data_rate) {
		pix_clk = dsim->def_pix_clk;
		bit_clk = DIV_ROUND_UP_ULL((u64)pix_clk * bpp, dsim->lanes);
		debug("%s:aa %d = %ld * %d / %d\n", __func__, bit_clk, pix_clk, bpp, dsim->lanes);
		if (bit_clk > dsim->max_data_rate) {
			dev_err(dsim->dev,
					"requested bit clk freq exceeds lane's maximum value\n");
			return -EINVAL;
		}
	}
	/* set suitable rate for phy ref clock */
	ret = sec_mipi_dsim_set_pref_rate(dsim, pix_clk);

	dsim->pixelclock = pix_clk;
	dsim->pix_clk = DIV_ROUND_UP_ULL(pix_clk, 1000);
	dsim->bit_clk = DIV_ROUND_UP_ULL(bit_clk, 1000);

	debug("%s: %d = %ld * %d / %d\n", __func__, bit_clk, pix_clk, bpp, dsim->lanes);
	if (dsim->hsmult) {
		unsigned bit_clkm = pix_clk * dsim->hsmult;

		if (bit_clk < bit_clkm) {
			bit_clk = bit_clkm;
			debug("%s: %d = %ld * %d (min)\n", __func__, bit_clk, pix_clk, dsim->hsmult);
		}
	}
	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_MBC) {
		unsigned n = (bpp + (dsim->lanes * 8) - 1) / (dsim->lanes * 8);
		unsigned bit_clkm = pix_clk * n * 8;

		if (bit_clk < bit_clkm) {
			bit_clk = bit_clkm;
			debug("%s: %d = %ld * 8 * %d(MBC)\n", __func__, bit_clk, pix_clk, n);
		}
	}
	if (dsim->mipi_dsi_multiple) {
		bit_clk += dsim->mipi_dsi_multiple - 1;
		bit_clk /= dsim->mipi_dsi_multiple;
		bit_clk *= dsim->mipi_dsi_multiple;
	}
	if (!(bit_clk % pix_clk))
		dsim->hsmult = bit_clk / pix_clk;
	else
		dsim->hsmult = 0;
	ret = sec_mipi_dsim_get_pms(dsim, bit_clk, dsim->ref_clk);
	if (ret < 0)
		return ret;
	if (dsim->dsi_clk) {
		debug("%s: %d = %ld * %d / %d, freq=%ld\n", __func__, bit_clk, pix_clk,
				bpp, dsim->lanes, dsim->frequency);
		clk_set_rate(dsim->dsi_clk, dsim->frequency);
	}

	return 0;
}

static int sec_mipi_dsim_bridge_enable(struct sec_mipi_dsim *dsim)
{
	int ret;

	/* At this moment, the dsim bridge's preceding encoder has
	 * already been enabled. So the dsim can be configed here
	 */

	/* config main display mode */
	sec_mipi_dsim_set_main_mode(dsim);

	/* config dsim dpi */
	sec_mipi_dsim_config_dpi(dsim);

	/* config dsim pll */
	ret = sec_mipi_dsim_config_pll(dsim);
	if (ret) {
		printf("dsim pll config failed: %d\n", ret);
		return ret;
	}

	/* config dphy timings */
	sec_mipi_dsim_config_dphy(dsim);

	/* initialize FIFO pointers */
	sec_mipi_dsim_init_fifo_pointers(dsim);

	return 0;
}

static int sec_mipi_dsim_init(struct udevice *dev,
			    struct mipi_dsi_device *device,
			    struct display_timing *timings,
			    unsigned int max_data_lanes,
			    const struct mipi_dsi_phy_ops *phy_ops)
{
	struct sec_mipi_dsim *dsim = dev_get_priv(dev);
	int ret;
	struct clk *clk_ref_parent;

	dsim->dev = dev;
	dsim->max_data_lanes = max_data_lanes;
	dsim->device = device;
	dsim->dsi_host.ops = &sec_mipi_dsim_host_ops;
	device->host = &dsim->dsi_host;

	dsim->base = (void *)dev_read_addr(device->dev);
	if ((fdt_addr_t)dsim->base == FDT_ADDR_T_NONE) {
		dev_err(device->dev, "dsim dt register address error\n");
		return -EINVAL;
	}

	dsim->timings = *timings;

	dsim->dsi_clk = devm_clk_get(dev, "dsim-clk");
	if (IS_ERR(dsim->dsi_clk)) {
		ret = PTR_ERR(dsim->dsi_clk);
		dev_err(dev, "Unable to get dsim-clk: %d\n", ret);
		return ret;
	}

	dsim->clk_pllref = devm_clk_get(dev, "pll-ref");
	if (IS_ERR(dsim->clk_pllref)) {
		ret = PTR_ERR(dsim->clk_pllref);
		dev_err(dev, "Unable to get pll-ref: %d\n", ret);
		return ret;
	}

	dsim->clk_cfg = devm_clk_get(dev, "cfg");
	if (IS_ERR(dsim->clk_cfg)) {
		ret = PTR_ERR(dsim->clk_cfg);
		dev_err(dev, "Unable to get configuration clock: %d\n", ret);
		return ret;
	}
	clk_ref_parent = clk_get_parent(dsim->clk_pllref);
	if (IS_ERR(clk_ref_parent)) {
		printf("%s: clk_get_parent(%ld) failed %ld\n", __func__,
				dsim->clk_pllref->id, PTR_ERR(clk_ref_parent));
		clk_ref_parent = NULL;
	}

	dsim->clk_pixel = devm_clk_get(dev, "pixel_clock");
	if (IS_ERR(dsim->clk_pixel)) {
		dev_warn(dev, "Unable to get pixel clock: %ld\n", PTR_ERR(dsim->clk_pixel));
		dsim->clk_pixel = NULL;
	} else {
		if (timings->pixelclock.typ) {
			ret = clk_set_rate(dsim->clk_pixel, timings->pixelclock.typ);
			debug("%s: rate %u, ret=%d\n", __func__, timings->pixelclock.typ, ret);
		}
	}

	return 0;
}

static void sec_mipi_dsim_disable_clkctrl(struct sec_mipi_dsim *dsim)
{
	uint32_t clkctrl;

	clkctrl = dsim_read(dsim, DSIM_CLKCTRL);

	clkctrl &= ~CLKCTRL_TXREQUESTHSCLK;

	clkctrl &= ~CLKCTRL_ESCCLKEN;

	clkctrl &= ~CLKCTRL_BYTECLKEN;

	dsim_write(dsim, clkctrl, DSIM_CLKCTRL);
}

static void sec_mipi_dsim_disable_pll(struct sec_mipi_dsim *dsim)
{
	uint32_t pllctrl;

	pllctrl  = dsim_read(dsim, DSIM_PLLCTRL);

	pllctrl &= ~PLLCTRL_PLLEN;

	dsim_write(dsim, pllctrl, DSIM_PLLCTRL);
}

static int sec_mipi_dsim_enable(struct udevice *dev)
{
	struct sec_mipi_dsim *dsim = dev_get_priv(dev);

	_sec_mipi_dsim_pll_enable(dsim, 1);
	return 0;
}

static int sec_mipi_dsim_disable(struct udevice *dev)
{
	uint32_t intsrc;
	struct sec_mipi_dsim *dsim = dev_get_priv(dev);

	if (!dsim)
		return 0;
	/* disable data transfer of dsim */
	sec_mipi_dsim_set_standby(dsim, false);

	/* disable esc clock & byte clock */
	sec_mipi_dsim_disable_clkctrl(dsim);

	/* disable dsim pll */
	sec_mipi_dsim_disable_pll(dsim);

	/* Clear all intsrc */
	intsrc = dsim_read(dsim, DSIM_INTSRC);
	dsim_write(dsim, intsrc, DSIM_INTSRC);
	_sec_mipi_dsim_pll_enable(dsim, 0);

	return 0;
}

struct dsi_host_ops sec_mipi_dsim_ops = {
	.init = sec_mipi_dsim_init,
	.enable = sec_mipi_dsim_enable,
	.disable = sec_mipi_dsim_disable,
};

int sec_mipi_dsim_pll_enable(struct sec_mipi_dsim *dsim, int enable)
{
	int ret = _sec_mipi_dsim_check_pll_out(dsim);

	if (ret < 0)
		return ret;
	debug("%s:enable=%d was %d\n", __func__, enable, dsim->enabled);
	dsim->enabled = enable;
	if (enable)
		clk_prepare_enable(dsim->clk_cfg);
	else
		clk_disable_unprepare(dsim->clk_cfg);
	_sec_mipi_dsim_pll_enable(dsim, enable);
	return 0;
}

static int sec_mipi_dsim_probe(struct udevice *dev)
{
	return 0;
}

static const struct udevice_id sec_mipi_dsim_ids[] = {
	{ .compatible = "samsung,sec-mipi-dsi" },
	{ }
};

U_BOOT_DRIVER(sec_mipi_dsim) = {
	.name			= "sec_mipi_dsim",
	.id			= UCLASS_DSI_HOST,
	.of_match		= sec_mipi_dsim_ids,
	.probe			= sec_mipi_dsim_probe,
	.remove 		= sec_mipi_dsim_disable,
	.ops			= &sec_mipi_dsim_ops,
	.priv_auto_alloc_size	= sizeof(struct sec_mipi_dsim),
};
