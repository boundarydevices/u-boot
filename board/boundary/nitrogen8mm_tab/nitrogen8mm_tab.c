/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/clock.h>
#if defined(CONFIG_IMX8MM)
#include <asm/arch/imx8mm_pins.h>
#elif defined(CONFIG_IMX8MN)
#include <asm/arch/imx8mn_pins.h>
#endif
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/dma.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/video.h>
#include <dm.h>
#include <env.h>
#include <i2c.h>
#include <linux/delay.h>
#include <spl.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, WDOG_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, UART_PAD_CTRL),

/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x116),

#define GPIRQ_GT911 			IMX_GPIO_NR(3, 6)
	IOMUX_PAD_CTRL(NAND_DATA00__GPIO3_IO6, 0x1d6),
#define GP_GT911_RESET			IMX_GPIO_NR(3, 7)
	IOMUX_PAD_CTRL(NAND_DATA01__GPIO3_IO7, 0x149),

#define GP_CSI1_1MIPI_PWDN	IMX_GPIO_NR(3, 9)
	IOMUX_PAD_CTRL(NAND_DATA03__GPIO3_IO9, 0x141),
#define GP_CSI1_1MIPI_RESET	IMX_GPIO_NR(3, 1)
	IOMUX_PAD_CTRL(NAND_CE0_B__GPIO3_IO1, 0x101),

#define GP_CSI1_2MIPI_PWDN	IMX_GPIO_NR(3, 16)
	IOMUX_PAD_CTRL(NAND_READY_B__GPIO3_IO16, 0x141),
#define GP_CSI1_2MIPI_RESET	IMX_GPIO_NR(3, 0)
	IOMUX_PAD_CTRL(NAND_ALE__GPIO3_IO0, 0x101),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(SAI5_RXD1__GPIO3_IO22, 0x09),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x80),

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x41),

#define GP_I2C2_MUX_RESET	IMX_GPIO_NR(5, 11)
	IOMUX_PAD_CTRL(ECSPI2_MOSI__GPIO5_IO11, 0x100),

#define GP_REG_3P7V		IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(SAI1_MCLK__GPIO4_IO20, 0x100),

	/* EC2x Modem control */
#define GP_EC21_RESET		IMX_GPIO_NR(4, 10)
	IOMUX_PAD_CTRL(SAI1_TXFS__GPIO4_IO10, 0x140),
#define GP_EC21_USB_BOOT	IMX_GPIO_NR(4, 11)
	IOMUX_PAD_CTRL(SAI1_TXC__GPIO4_IO11 , 0x100),
#define GP_EC21_POWER_KEY	IMX_GPIO_NR(4, 18)
	IOMUX_PAD_CTRL(SAI1_TXD6__GPIO4_IO18, 0x140),
#define GP_EC21_USIM_DETECT	IMX_GPIO_NR(4, 17)
	IOMUX_PAD_CTRL(SAI1_TXD5__GPIO4_IO17, 0x100),
#define GP_EC21_WAKE_UP		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(SAI1_TXD3__GPIO4_IO15, 0x100),
#define GP_EC21_AP_READY	IMX_GPIO_NR(4, 14)
	IOMUX_PAD_CTRL(SAI1_TXD2__GPIO4_IO14, 0x100),
#define GP_EC21_ACTIVE_STATUS	IMX_GPIO_NR(4, 19)
	IOMUX_PAD_CTRL(SAI1_TXD7__GPIO4_IO19, 0x1c0),
#define GP_EC21_NET_STAT	IMX_GPIO_NR(4, 12)
	IOMUX_PAD_CTRL(SAI1_TXD0__GPIO4_IO12, 0x80),
#define GP_EC21_NET_MODE	IMX_GPIO_NR(4, 13)
	IOMUX_PAD_CTRL(SAI1_TXD1__GPIO4_IO13, 0x80),
#define GP_EC21_RI		IMX_GPIO_NR(4, 16)
	IOMUX_PAD_CTRL(SAI1_TXD4__GPIO4_IO16, 0x80),

#define GP_REG_5V_EN		IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(NAND_RE_B__GPIO3_IO15, 0x100),
#define GP_FLASH_EN		IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x100),
#define GP_TORCH_EN		IMX_GPIO_NR(1, 5)
	IOMUX_PAD_CTRL(GPIO1_IO05__GPIO1_IO5, 0x100),
#define GPIRQ_PN7150		IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, 0x182),
};

void release_i2c2_mux_reset(void)
{
	int gp_mux_reset = GP_I2C2_MUX_RESET;
	int req_mux_reset;

	req_mux_reset = gpio_request(gp_mux_reset, "i2c2_mux_reset");
	gpio_direction_output(gp_mux_reset, 1);
	if (!req_mux_reset)
		gpio_free(gp_mux_reset);
}

static int limit_table[] = { 500, 500, 1500, 3000 };

struct udevice *select_i2c2d(void)
{
	struct udevice *bus;
	struct i2c_msg msg[1];
	struct dm_i2c_ops *ops;
	unsigned char txbuf[4];
	int ret;

	release_i2c2_mux_reset();
	ret = uclass_get_device_by_seq(UCLASS_I2C, 1, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return NULL;
	}

	ops = i2c_get_ops(bus);
	/* Select last bus on tca9546, i2c2d */
	txbuf[0] = 0x08;
	msg[0].addr = 0x70;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = txbuf;
	ret = ops->xfer(bus, msg, 1);
	if (ret < 0)
		printf("tca9546 bus select failed(%d)\n", ret);
	udelay(10);
	return bus;
}

void board_init_charger_limit(void)
{
	struct udevice *bus;
	struct i2c_msg msg[2];
	struct dm_i2c_ops *ops;
	unsigned char txbuf[8];
	unsigned char rxbuf[4];
	unsigned char cc_status0;
	int ret;
	int i;

	bus = select_i2c2d();
	if (!bus)
		return;

	ops = i2c_get_ops(bus);
	msg[0].addr = 0x25;
	msg[0].flags = 0;
	msg[0].buf = txbuf;

	txbuf[0] = 0x0c;
	msg[0].len = 1;
	msg[1].addr = 0x25;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = rxbuf;

	/* read status */
	ret = ops->xfer(bus, msg, 2);
	if (ret < 0) {
		printf("max77958 cc_status0 read failed(%d)\n", ret);
		return;
	}
	if (!(rxbuf[0] & 0x07))
		return;
	cc_status0 = rxbuf[0];

	/* Read role */
	txbuf[0] = 0x0f;
	ret = ops->xfer(bus, msg, 2);
	if (ret < 0) {
		printf("max77958 pd_status1 read failed(%d)\n", ret);
		rxbuf[0] = 0;
	}

	if (rxbuf[0] & 0x40)
		return;
	/* We are a sink, Set the limit */
	i = (cc_status0 >> 4) & 0x03;
	max77975_set_chrgin_limit(limit_table[i]);
}

int board_usb_otg_mode(struct udevice *dev)
{
	struct udevice *bus;
	struct i2c_msg msg[2];
	struct dm_i2c_ops *ops;
	unsigned char txbuf[8];
	unsigned char rxbuf[4];
	unsigned char cc_status0;
	int ret;
	int i;

	bus = select_i2c2d();
	if (!bus)
		return USB_INIT_UNKNOWN;

	ops = i2c_get_ops(bus);
	msg[0].addr = 0x25;
	msg[0].flags = 0;
	msg[0].buf = txbuf;
#if 0
	/*
	 * cc control1 write
	 * Enable SOURCE mode for DRP
	 * i2c mw 25 21.1 0c;i2c mw 25 22.1 c3;i2c mw 25 41.1 00;
	 */
	txbuf[0] = 0x21;
	txbuf[1] = 0x0c;
	txbuf[2] = 0xc3;
	msg[0].len = 3;
	ret = ops->xfer(bus, msg, 1);
	if (ret < 0)
		printf("cc control1 write(%d)\n", ret);
	mdelay(1);

	txbuf[0] = 0x41;
	txbuf[1] = 0x00;
	msg[0].len = 2;
	ret = ops->xfer(bus, msg, 1);
	if (ret < 0)
		printf("cc control1 write41(%d)\n", ret);
	mdelay(1);
#endif

	/* Wait for attachment */
	txbuf[0] = 0x0c;
	msg[0].len = 1;
	msg[1].addr = 0x25;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = rxbuf;
	i = 0;
	while (1) {
		/* read status */
		ret = ops->xfer(bus, msg, 2);
		if (ret < 0) {
			printf("max77958 cc_status0 read failed(%d)\n", ret);
			rxbuf[0] = 0;
		}
		if (rxbuf[0] & 0x07) {
			cc_status0 = rxbuf[0];
			break;
		}
		i++;
		if (i >= 5) {
			debug("%s: %0x, unknown\n", __func__, rxbuf[0]);
			return USB_INIT_UNKNOWN;
		}
		mdelay(10);
	}
	/* Read role */
	txbuf[0] = 0x0f;
	ret = ops->xfer(bus, msg, 2);
	if (ret < 0) {
		printf("max77958 pd_status1 read failed(%d)\n", ret);
		rxbuf[0] = 0;
	}

	/* Turn on power if source */
	max77975_otg_power((rxbuf[0] & 0x40) ? 1 : 0);

	if (!(rxbuf[0] & 0x40)) {
		/* We can charge. Set the limit */
		i = (cc_status0 >> 4) & 0x03;
		max77975_set_chrgin_limit(limit_table[i]);
	}

	/*
	 * enable switches DP/DP2, DN/DN1
	 * i2c mw 25 21.1 06;i2c mw 25 22.1 09;i2c mw 25 41.1 00;
	 */
	txbuf[0] = 0x21;
	txbuf[1] = 0x06;
	txbuf[2] = 0x09;
	msg[0].len = 3;
	ret = ops->xfer(bus, msg, 1);
	if (ret < 0)
		printf("control1 write(%d)\n", ret);
	mdelay(1);

	txbuf[0] = 0x41;
	txbuf[1] = 0x00;
	msg[0].len = 2;
	ret = ops->xfer(bus, msg, 1);
	if (ret < 0)
		printf("control1 write41(%d)\n", ret);

	if (rxbuf[0] & 0x80) {
		debug("%s: host %02x\n", __func__, rxbuf[0]);
		return USB_INIT_HOST;
	}
	debug("%s: peripheral %02x\n", __func__, rxbuf[0]);
	return USB_INIT_DEVICE;
}

static const struct gpio_reserve gpios_to_reserve[] = {
	{ GP_REG_3P7V, GPIOD_OUT_LOW, GRF_FREE, "3p7v", },
	{ GP_REG_5V_EN, GPIOD_OUT_HIGH, GRF_FREE, "reg-5v", },
	{ GP_FLASH_EN, GPIOD_OUT_LOW, 0, "flash-en", },
	{ GP_TORCH_EN, GPIOD_OUT_LOW, 0, "torch-en", },
	{ GPIRQ_PN7150, GPIOD_IN, 0, "irq-pn7150", },
	{ GP_EMMC_RESET, GPIOD_OUT_HIGH, GRF_FREE, "emmc-reset", },
	{ GP_EC21_RESET, GPIOD_OUT_LOW, 0, "ec21-reset", },
	{ GP_EC21_USB_BOOT, GPIOD_OUT_LOW, 0, "ec21-usb-boot", },
	{ GP_EC21_POWER_KEY, GPIOD_IN, 0, "ec21-power-key", },
	{ GP_EC21_USIM_DETECT, GPIOD_OUT_LOW, 0, "ec21-usim-detect", },
	{ GP_EC21_WAKE_UP, GPIOD_OUT_LOW, 0, "ec21-wake-up", },
	{ GP_EC21_AP_READY, GPIOD_OUT_LOW, 0, "ec21-ap-ready", },
	{ GP_EC21_ACTIVE_STATUS, GPIOD_IN, 0, "ec21-active-status", },
	{ GP_EC21_NET_STAT, GPIOD_IN, 0, "ec21-net-stat", },
	{ GP_EC21_NET_MODE, GPIOD_IN, 0, "ec21-net-mode", },
	{ GP_EC21_RI, GPIOD_IN, 0, "ec21-ri", },
	{ GP_GT911_RESET, GPIOD_OUT_LOW, 0, "gt911_reset", },
	{ GPIRQ_GT911, GPIOD_IN, 0, "gt911_irq", },
	{ GP_CSI1_1MIPI_RESET, GPIOD_OUT_LOW, 0, "csi1_1mipi_reset", },
	{ GP_CSI1_1MIPI_PWDN, GPIOD_OUT_HIGH, 0, "csi1_1mipi_pwdn", },
	{ GP_CSI1_2MIPI_RESET, GPIOD_OUT_LOW, 0, "csi1_2mipi_reset", },
	{ GP_CSI1_2MIPI_PWDN, GPIOD_OUT_HIGH, 0, "csi1_2mipi_pwdn", },
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	set_wdog_reset(wdog);
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
int board_detect_gt911(struct display_info_t const *di)
{
	int ret;
	struct udevice *dev, *chip;

#ifdef CONFIG_DM_VIDEO
	if (di->bus_gp)
		gpio_request(di->bus_gp, "bus_gp");
#endif
	if (di->bus_gp)
		gpio_direction_output(di->bus_gp, 1);
	gpio_set_value(GP_GT911_RESET, 0);
	mdelay(20);
	gpio_direction_output(GPIRQ_GT911, di->addr_num == 0x14 ? 1 : 0);
	udelay(100);
	gpio_set_value(GP_GT911_RESET, 1);
	mdelay(6);
	gpio_set_value(GPIRQ_GT911, 0);
	mdelay(50);
	gpio_direction_input(GPIRQ_GT911);
	ret = uclass_get_device(UCLASS_I2C, di->bus_num, &dev);
	if (!ret) {
		ret = dm_i2c_probe(dev, di->addr_num, 0x0, &chip);
		if (ret && di->bus_gp)
			gpio_direction_input(di->bus_gp);
	}
#ifdef CONFIG_DM_VIDEO
	if (di->bus_gp)
		gpio_free(di->bus_gp);
#endif
	return (ret == 0);
}

static const struct display_info_t displays[] = {
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(3, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpios_reserve(gpios_to_reserve, ARRAY_SIZE(gpios_to_reserve));
#if defined(CONFIG_MXC_SPI) && !defined(CONFIG_DM_SPI)
	setup_spi();
#endif

#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif

#ifdef CONFIG_MAX77975
	release_i2c2_mux_reset();
	max77975_init();
	board_init_charger_limit();
#endif
#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

void board_poweroff(void)
{
	struct snvs_regs *snvs = (struct snvs_regs *)(SNVS_BASE_ADDR);

	writel(0x60, &snvs->lpcr);
	mdelay(500);
}

static int _do_poweroff(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();
	return 0;
}

U_BOOT_CMD(
	poweroff, 70, 0, _do_poweroff,
	"power down board",
	""
);
