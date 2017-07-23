/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <command.h>
#include <asm/arch/clock.h>
#if !defined(CONFIG_MX7D) && !defined(CONFIG_MX51) && !defined(CONFIG_MX6ULL) && !defined(CONFIG_IMX8M)
#include <asm/arch/iomux.h>
#endif
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/sata.h>
#include <env.h>
#include <i2c.h>
#include <linux/fb.h>
#include <net.h>
#include <version.h>
#include <usb.h>
#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
#include <dwc3-uboot.h>
#include <linux/usb/dwc3.h>
#endif

#include "bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define MAX_LOW_SIZE	(0x100000000ULL - CONFIG_SYS_SDRAM_BASE)
#define SDRAM_SIZE	((1ULL * CONFIG_DDR_MB) << 20)

#if SDRAM_SIZE > MAX_LOW_SIZE
#define MEM_SIZE	MAX_LOW_SIZE
#else
#define MEM_SIZE	SDRAM_SIZE
#endif

#if defined(CONFIG_IMX8M)
int board_phys_sdram_size(phys_size_t *sdram_size)
{
	*sdram_size = SDRAM_SIZE;
	return 0;
}

ulong board_get_usable_ram_top(ulong total_size)
{
	return MEM_SIZE + CONFIG_SYS_SDRAM_BASE;
}
#endif

#if !defined(CONFIG_IMX8M)
int dram_init(void)
{
#if defined(CONFIG_MX51) || defined(CONFIG_MX7D)
	gd->ram_size = MEM_SIZE;
#else
	gd->ram_size = imx_ddr_size();
#endif
//	printf("%s:%p *%p=0x%lx\n", __func__, gd, &gd->ram_size, gd->ram_size);

	return 0;
}
#endif

#if defined(CONFIG_BOARD_POSTCLK_INIT) && defined(CONFIG_IMX8M)
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif


#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
#ifdef CONFIG_IMX8M_DRAM_INLINE_ECC
	int rc;
	phys_addr_t ecc0_start = 0xb0000000;
	phys_addr_t ecc1_start = 0x130000000;
	phys_addr_t ecc2_start = 0x1b0000000;
	size_t ecc_size = 0x10000000;

	rc = add_res_mem_dt_node(blob, "ecc", ecc0_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc0 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc1_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc1 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc2_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc2 reserved-memory node.\n");
		return rc;
	}
#endif
	return 0;
}
#endif

#if defined(CONFIG_IMX8MM)
int board_usb_init(int index, enum usb_init_type init)
{
	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, false);
	return 0;
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)

#define USB_PHY_CTRL0			0xF0040
#define USB_PHY_CTRL0_REF_SSP_EN	BIT(2)

#define USB_PHY_CTRL1			0xF0044
#define USB_PHY_CTRL1_RESET		BIT(0)
#define USB_PHY_CTRL1_COMMONONN		BIT(1)
#define USB_PHY_CTRL1_ATERESET		BIT(3)
#define USB_PHY_CTRL1_VDATSRCENB0	BIT(19)
#define USB_PHY_CTRL1_VDATDETENB0	BIT(20)

#define USB_PHY_CTRL2			0xF0048
#define USB_PHY_CTRL2_TXENABLEN0	BIT(8)

#ifdef CONFIG_IMX8MP
#define USB_PHY_CTRL6			0xF0058

#define HSIO_GPR_BASE                               (0x32F10000U)
#define HSIO_GPR_REG_0                              (HSIO_GPR_BASE)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT    (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN          (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)
#endif

static struct dwc3_device dwc3_device_data = {
#ifdef CONFIG_SPL_BUILD
	.maximum_speed = USB_SPEED_HIGH,
#else
	.maximum_speed = USB_SPEED_SUPER,
#endif
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
//	.power_down_scale = 2,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
#ifdef CONFIG_IMX8MQ
	struct dwc3 *dwc3_reg = (struct dwc3 *)(dwc3->base + DWC3_REG_OFFSET);
#endif
	u32 val;

#ifdef CONFIG_IMX8MP
	/* enable usb clock via hsio gpr */
	val = readl(HSIO_GPR_REG_0);
	val |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
	writel(val, HSIO_GPR_REG_0);

	/* USB3.0 PHY signal fsel for 100M ref */
	val = readl(dwc3->base + USB_PHY_CTRL0);
	val = (val & 0xfffff81f) | (0x2a<<5);
	writel(val, dwc3->base + USB_PHY_CTRL0);

	val = readl(dwc3->base + USB_PHY_CTRL6);
	val &=~0x1;
	writel(val, dwc3->base + USB_PHY_CTRL6);

#endif
	val = readl(dwc3->base + USB_PHY_CTRL1);
	val &= ~(USB_PHY_CTRL1_VDATSRCENB0 | USB_PHY_CTRL1_VDATDETENB0 |
			USB_PHY_CTRL1_COMMONONN);
	val |= USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET;
	writel(val, dwc3->base + USB_PHY_CTRL1);

	val = readl(dwc3->base + USB_PHY_CTRL0);
	val |= USB_PHY_CTRL0_REF_SSP_EN;
	writel(val, dwc3->base + USB_PHY_CTRL0);

	val = readl(dwc3->base + USB_PHY_CTRL2);
	val |= USB_PHY_CTRL2_TXENABLEN0;
	writel(val, dwc3->base + USB_PHY_CTRL2);

	val = readl(dwc3->base + USB_PHY_CTRL1);
	val &= ~(USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET);
	writel(val, dwc3->base + USB_PHY_CTRL1);

#ifdef CONFIG_IMX8MQ
	val = readl(&dwc3_reg->g_ctl);
	val &= ~(DWC3_GCTL_PWRDNSCALE_MASK);
	val |= DWC3_GCTL_PWRDNSCALE(2);

	writel(val, &dwc3_reg->g_ctl);
#endif
}

int __weak board_usb_hub_gpio_init(void)
{
	return 0;
}

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
	imx8m_usb_power(index, true);

	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
		return ret;
	}

	if (index == 1) {
		int gp = board_usb_hub_gpio_init();
		/* Release HUB reset */
		if (gp) {
			gpio_request(gp, "usb1_rst");
			gpio_direction_output(gp, 1);
		}
	}

	return 0;
}
int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE)
		dwc3_uboot_exit(index);

	imx8m_usb_power(index, false);

	return ret;
}
#endif

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

void set_gpios_in(const unsigned short *p, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_input(*p++);
}

void set_gpios(const unsigned short *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

#if defined(CONFIG_FSL_ESDHC_IMX) && !CONFIG_IS_ENABLED(DM_MMC)
int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = cfg->gp_cd;

	if (!gp_cd)
		return 1;	/* eMMC always present */
	if (gp_cd == -1)
		return 0;	/* always skip */
	return !gpio_get_value(gp_cd);
}

#if defined(CONFIG_MX51)
#define BASE1 MMC_SDHC1_BASE_ADDR
#define BASE2 MMC_SDHC2_BASE_ADDR
#define CNT CONFIG_SYS_FSL_ESDHC_NUM
#elif defined(CONFIG_IMX8M)
#define BASE1 USDHC1_BASE_ADDR
#define BASE2 USDHC2_BASE_ADDR
#if defined(CONFIG_IMX8MM)||defined(CONFIG_IMX8MN)||defined(CONFIG_IMX8MP)
#define BASE3 USDHC3_BASE_ADDR
#endif
#define CNT CONFIG_SYS_FSL_USDHC_NUM
#else
#define BASE1 USDHC1_BASE_ADDR
#define BASE2 USDHC2_BASE_ADDR
#define BASE3 USDHC3_BASE_ADDR
#ifndef CONFIG_MX7D
#define BASE4 USDHC4_BASE_ADDR
#endif
#define CNT CONFIG_SYS_FSL_USDHC_NUM
#endif

int board_mmc_init(struct bd_info *bis)
{
	int ret;
	u32 index = 0;

	for (index = 0; index < CNT; index++) {
		struct fsl_esdhc_cfg *cfg = &board_usdhc_cfg[index];

		if (cfg->esdhc_base == BASE1) {
#ifdef CONFIG_IMX8MP
			init_clk_usdhc(0);
#endif
			cfg->sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		} else if (cfg->esdhc_base == BASE2) {
#ifdef CONFIG_IMX8MP
			init_clk_usdhc(1);
#endif
			cfg->sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
#ifdef BASE3
		} else if (cfg->esdhc_base == BASE3) {
#ifdef CONFIG_IMX8MP
			init_clk_usdhc(2);
#endif
			cfg->sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
#endif
#ifdef BASE4
		} else if (cfg->esdhc_base == BASE4) {
			cfg->sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
#endif
		} else {
#if defined(CONFIG_IMX8M)
			printf("unknown esdhc base %llx\n", cfg->esdhc_base);
#else
			printf("unknown esdhc base %lx\n", cfg->esdhc_base);
#endif
			break;
		}

		if (cfg->gp_cd)
			gpio_request(cfg->gp_cd, "usdhcx_cd");

		if (cfg->gp_reset) {
			gpio_request(cfg->gp_reset, "usdhcx_reset");
			gpio_set_value(cfg->gp_reset, 1); /* release reset */
		}

		ret = fsl_esdhc_initialize(bis, cfg);
		if (ret)
			return ret;
	}
	return 0;
}
#endif

#ifdef CONFIG_SPLASH_SCREEN
int splash_screen_prepare(void)
{
	char *env_loadsplash;

	if (!env_get("splashimage") || !env_get("splashsize")) {
		return -1;
	}

	env_loadsplash = env_get("loadsplash");
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
#endif

#ifdef CONFIG_CMD_FBPANEL
int board_cfb_skip(void)
{
	return NULL != env_get("novideo");
}
#endif

void common_board_init(const struct i2c_pads_info *p, int i2c_bus_cnt, int otg_id,
		const struct display_info_t *displays, int display_cnt,
		int gp_hd_detect)
{
	int i;
#ifdef IOMUXC_GPR1_OTG_ID_MASK
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* needed for i.mx6q, duallite doesn't need it, but doesn't hurt */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_OTG_ID_MASK, otg_id);
#endif
	p += i2c_get_info_entry_offset();

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < i2c_bus_cnt; i++) {
	        setup_i2c(p->bus_index, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}
#ifdef CONFIG_FAN53526
	fan53526_init();
#endif
#ifdef CONFIG_MAX77823
	max77823_init();
#endif
#ifdef CONFIG_TAMPER
	check_tamper();
#endif

#ifdef CONFIG_CMD_SATA
	if (!gp_hd_detect || gpio_get_value(gp_hd_detect))
		setup_sata();
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
}

int misc_init_r(void)
{
#if defined(CONFIG_PREBOOT) && !defined(CONFIG_ARCH_IMX8M)
	board_preboot_keys();
#endif

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	/*
	 * Not really needed as script checks for magic value in memory,
	 * but shouldn't hurt.
	 */
	env_set_hex("reset_cause", get_imx_reset_cause());

#ifdef CONFIG_MX7D
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);
#endif
	return 0;
}

#ifdef CONFIG_PRINT_TIME_RV4162

/* RV4162 particulars */
#define RTC_I2CADDR 0x68
#define RTC_YEAR 7
#define RTC_MON  6
#define RTC_DAY  5
#define RTC_HOUR 3
#define RTC_MIN  2
#define RTC_SEC  1

static int frombcd(u8 val)
{
	return (10*(val>>4))+(val&0x0f);
}

static void print_time_rv4162(void)
{
	u8 orig_i2c_bus;

	orig_i2c_bus = i2c_get_bus_num();
	/* display date and time from RTC */
	if (!i2c_set_bus_num(0) &&
			!i2c_probe(RTC_I2CADDR)) {
		u8 buffer[16];
		int ret = i2c_read(RTC_I2CADDR, 0, 1, buffer, sizeof(buffer));

		if (ret)
			printf("Error %d reading RTC\n", ret);
		else
			printf("time: %04u-%02u-%02u %02u:%02u:%02u UTC\n",
			       2000+frombcd(buffer[RTC_YEAR]),
			       frombcd(buffer[RTC_MON]&0x1f),
			       frombcd(buffer[RTC_DAY]),
			       frombcd(buffer[RTC_HOUR]),
			       frombcd(buffer[RTC_MIN]),
			       frombcd(buffer[RTC_SEC]));
	}
	i2c_set_bus_num(orig_i2c_bus);
}
#else
static void print_time_rv4162(void) {}
#endif

#ifdef CONFIG_FEC_MXC
#define ADDMAC_OFFSET	0x800000
#else
#define ADDMAC_OFFSET	0
#endif

#if defined(CONFIG_ENV_WLMAC) || defined(CONFIG_ENV_BD_ADDR)
static void addmac_env(const char* env_var)
{
	unsigned char mac_address[8];
	char macbuf[20];

	if (!env_get(env_var)) {
		imx_get_mac_from_fuse(ADDMAC_OFFSET, mac_address);
		if (is_valid_ethaddr(mac_address)) {
			snprintf(macbuf, sizeof(macbuf), "%pM", mac_address);
			env_set(env_var, macbuf);
		}
	}
}
#endif

#if defined(CONFIG_CMD_FASTBOOT) || defined(CONFIG_CMD_DFU)
static void addserial_env(const char* env_var)
{
	unsigned char mac_address[8];
	char serialbuf[20];

	if (!env_get(env_var)) {
		imx_get_mac_from_fuse(0, mac_address);
		snprintf(serialbuf, sizeof(serialbuf), "%02x%02x%02x%02x%02x%02x",
			 mac_address[0], mac_address[1], mac_address[2],
			 mac_address[3], mac_address[4], mac_address[5]);
		env_set(env_var, serialbuf);
	}
}
#endif

#ifdef CONFIG_SYS_BOARD
#define board_type CONFIG_SYS_BOARD
#else
/* CANNOT be in BSS section, will clobber relocation table */
const char *board_type = (void*)1;
#endif

int checkboard(void)
{
#ifdef CONFIG_SYS_BOARD
	puts("Board: " CONFIG_SYS_BOARD "\n");
#else
	board_type = board_get_board_type();
	puts("Board: ");
	puts(board_type);
	puts("\n");
#endif
	return 0;
}

static const char str_uboot_release[] = "uboot_release";
static const char cur_uboot_release[] = PLAIN_VERSION;

void __weak board_env_init(void) {}

#if defined(CONFIG_CMD_FASTBOOT) || defined(CONFIG_CMD_DFU)
int __weak board_fastboot_key_pressed(void)
{
	return 0;
}
#endif

#if defined(CONFIG_FSL_FASTBOOT) && defined(CONFIG_ANDROID_RECOVERY)
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif

int bdcommon_env_init(void)
{
	char *uboot_release;
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	int cpurev = get_cpu_rev();

#if defined(CONFIG_IMX8MQ)
	env_set("soc", "imx8mq");
#else
	env_set("soc", CONFIG_SYS_SOC);
#endif
#if !defined(CONFIG_IMX8M)
	env_set("cpu", get_imx_type((cpurev & 0x3FF000) >> 12));
#endif
	env_set("imx_cpu", get_imx_type((cpurev & 0x3FF000) >> 12));
	/*
	 * These lines are specific to nitrogen6x, as
	 * everyone else has board in their default environment.
	 */
	if (!env_get("board"))
		env_set("board", board_type);
	env_set("uboot_defconfig", CONFIG_DEFCONFIG);
#endif

#ifdef CONFIG_ENV_WLMAC
	addmac_env("wlmac");
#endif
#ifdef CONFIG_ENV_BD_ADDR
	addmac_env("bd_addr");
#endif
#if defined(CONFIG_CMD_FASTBOOT) || defined(CONFIG_CMD_DFU)
	addserial_env("serial#");
	if (board_fastboot_key_pressed()) {
		printf("Starting fastboot...\n");
		env_set("preboot", "fastboot 0");
	}
#endif

#if !defined(CONFIG_ENV_IS_NOWHERE)
	uboot_release = env_get(str_uboot_release);
	if (!uboot_release || strcmp(cur_uboot_release, uboot_release)) {
		env_set(str_uboot_release, cur_uboot_release);
		if (uboot_release) {
			/*
			 * if already saved in environment, correct value
			 */
			env_save();
		}
	}
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_env_cmds();
#endif
	board_env_init();
	board_eth_addresses();
	return 0;
}

void board_env_set_default(void)
{
	bdcommon_env_init();
}

void init_usb_clk(int usbno);

int board_late_init(void)
{
#if defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8MP)
	init_usb_clk(0);
	init_usb_clk(1);
#endif
#ifdef CONFIG_BOARD_LATE_SPECIFIC_INIT
	board_late_specific_init();
#endif
	print_time_rv4162();
#if (defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP)) && defined(CONFIG_CMD_FBPANEL)
	/* No display yet for mini/nano so need to setup cmd_... */
	board_video_skip();
#endif
	return bdcommon_env_init();
}
