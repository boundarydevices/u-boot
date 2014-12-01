/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#define CONFIG_MX6QDL
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <splash.h>
#include <usb/ehci-fsl.h>
#ifdef CONFIG_MX6Q
#include "pads-insp.h"
#endif
#if defined(CONFIG_MX6DL) || defined(CONFIG_MX6S)
#define FOR_DL_SOLO
#include "pads-insp.h"
#endif

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_MX6DL) || defined(CONFIG_MX6S)
#ifdef CONFIG_MX6Q
#define GET_MX6_REF(ref) (is_cpu_type(MXC_CPU_MX6Q) ? mx6q_##ref : mx6dl_solo_##ref)
#define IOMUX_SETUP(list)  iomux_setup(mx6q_##list, ARRAY_SIZE(mx6q_##list), \
		mx6dl_solo_##list, ARRAY_SIZE(mx6dl_solo_##list))

int iomux_setup(iomux_v3_cfg_t *mx6q_pad_list, int mx6q_pad_cnt,
               iomux_v3_cfg_t *mx6dl_solo_pad_list, int mx6dl_solo_pad_cnt)
{
	int mx6q = is_cpu_type(MXC_CPU_MX6Q);
	iomux_v3_cfg_t *p =  mx6q ? mx6q_pad_list : mx6dl_solo_pad_list;
	int cnt = mx6q ? mx6q_pad_cnt : mx6dl_solo_pad_cnt;

	return imx_iomux_v3_setup_multiple_pads(p, cnt);
}
#else
#define GET_MX6_REF(ref) (mx6dl_solo_##ref)
#define IOMUX_SETUP(list)  imx_iomux_v3_setup_multiple_pads(   \
		mx6dl_solo_##list, ARRAY_SIZE(mx6dl_solo_##list))
#endif
#else
#define GET_MX6_REF(ref) (mx6q_##ref)
#define IOMUX_SETUP(list)  imx_iomux_v3_setup_multiple_pads(   \
		mx6q_##list, ARRAY_SIZE(mx6q_##list))
#endif

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port == 0)
		gpio_set_value(GP_USB_OTG_PWR, on);
	if (port == 1)
		gpio_set_value(GP_USB_H1_PWR, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = (cfg->esdhc_base == USDHC3_BASE_ADDR) ? GP_SD3_CD : -1;
	if (gp_cd < 0)
		return 1;
	gpio_direction_input(gp_cd);
	return !gpio_get_value(gp_cd);
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	usdhc_cfg[0].max_bus_width = 4;
	usdhc_cfg[1].max_bus_width = 8;

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			break;
		case 1:
			gpio_set_value(GP_EMMC_RESET, 1);
			break;
		default:
		       printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
		       return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_CS1 : -1;
}

#endif

int board_eth_init(bd_t *bis)
{
#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	usb_eth_initialize(bis);
#endif
	return 0;
}


int splash_screen_prepare(void)
{
	char *env_loadsplash;

	if (!getenv("splashimage") || !getenv("splashsize")) {
		return -1;
	}

	env_loadsplash = getenv("loadsplash");
	if (env_loadsplash == NULL) {
		printf("Environment variable loadsplash not found!\n");
		return -1;
	}

	if (run_command_list(env_loadsplash, -1, 0)) {
		printf("failed to run loadsplash %s\n\n", env_loadsplash);
		return -1;
	}

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
static void do_enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

static int detect_none(struct display_info_t const *dev)
{
	return 1;
}

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
}

static void enable_rgb(struct display_info_t const *dev)
{
	IOMUX_SETUP(rgb_pads);
	gpio_direction_output(GP_RGB_BACKLIGHT, 1);
}

struct display_info_t const displays[] = {{
	.bus	= 1,
	.addr	= 0x50,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_i2c,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x48,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_none,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "LSA40AT9001",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 600,
		.pixclock       = (800+10+46+210) * 1000 / (600+1+23+12) / 60,
		.left_margin    = 46,
		.right_margin   = 210,
		.upper_margin   = 23,
		.lower_margin   = 12,
		.hsync_len      = 10,
		.vsync_len      = 1,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

size_t display_count = ARRAY_SIZE(displays);

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();
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

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif

static unsigned short gpios_out_low[] = {
	GP_ADV7180_RESET,
	GP_J6_I2C_EN,
	GP_HDMI_I2C_EN,
	GP_TSC2004_RESET,
	GP_PWM1,
	GP_RGB_BACKLIGHT,
	GP_CAMERA_POWER_DOWN,
	GP_CAMERA_RX_EN,
	GP_SGTL5000_MUTE,
	GP_USB_H1_PWR,
	GP_USB_OTG_PWR,
	GP_WL_EN,		/* disable wireless */
	GP_BT_EN,	 	/* disable bluetooth */
	GP_EMMC_RESET,
};

static unsigned short gpios_out_high[] = {
	GP_MAIN_POWER_EN,
	GP_ECSPI1_CS1,
};

static unsigned short gpios_in[] = {
	GP_MAIN_POWER_BUTTON,
	GP_ADV7180_IRQ,
	GP_INSP_GP1,
	GP_INSP_GP2,
	GP_INSP_GP3,
	GP_INSP_GP4,
	GP_INSP_GP5,
	GP_INSP_GP6,
	GP_INSP_GP7,
	GP_INSP_GP8,
	GP_INSP_GP9,
	GP_INSP_GP10,
	GP_TSC2004_IRQ,
	GP_RTC_RV4162_IRQ,
	GP_CAMERA_LOCK,
	GP_HEADPHONE_DET,
	GP_WL_IRQ,
	GP_SD3_CD,
	GP_SD3_WP,
};

static void set_gpios_in(unsigned short *p, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_input(*p++);
}

static void set_gpios(unsigned short *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	IOMUX_SETUP(insp_pads);
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
	struct i2c_pads_info *p = GET_MX6_REF(i2c_pad_info);
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u8 orig_i2c_bus;
	u8 val8;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &p[0]);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &p[1]);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &p[2]);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(2);
	val8 = 0x7f;	/* 4.0A source */
	i2c_write(0x69, 0xc0, 1, &val8, 1);
	val8 = 0x0c;	/* Protection allow 0xb9 write */
	i2c_write(0x69, 0xbd, 1, &val8, 1);
	val8 = 0x14;	/* 1A charge */
	i2c_write(0x69, 0xb9, 1, &val8, 1);
	i2c_set_bus_num(orig_i2c_bus);

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

static char const *board_type = "uninitialized";

int checkboard(void)
{
	puts("Board: insp\n");
	board_type = "insp";
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"input1",	GP_INSP_GP1,	'1'},
	{"input2",	GP_INSP_GP2,	'2'},
	{"power",	GP_MAIN_POWER_BUTTON,	'P'},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_get_value(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	setenv("keybd", envvalue);
	return numpressed == 0;
}

U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);

#ifdef CONFIG_PREBOOT
static char const kbd_magic_prefix[] = "key_magic";
static char const kbd_command_prefix[] = "key_cmd";

static void preboot_keys(void)
{
	int numpressed;
	char keypress[ARRAY_SIZE(buttons)+1];
	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = getenv("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = getenv(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = getenv(cmd_name);
			if (cmd) {
				setenv("preboot", cmd);
				return;
			}
		}
	}
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int misc_init_r(void)
{
#ifdef CONFIG_PREBOOT
	preboot_keys();
#endif

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();
	setenv("cpu",get_imx_type((cpurev & 0xFF000) >> 12));
	if (0 == getenv("board"))
		setenv("board",board_type);
	return 0;
}
