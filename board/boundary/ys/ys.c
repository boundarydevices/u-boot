/*
 * Copyright (C) 2017 Boundary Devices, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <linux/sizes.h>
#include <malloc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include "../padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define CSI_PAD_CTL	PAD_CTL_DSE_120ohm

#define I2C_PAD_CTRL    (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_ODE)

#define LCDIF_PAD_CTL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC2_PAD_CTRL (PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC2_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

static const iomux_v3_cfg_t init_pads[] = {
	/* ECSPI1 (serial nor eeprom) */
	IOMUX_PAD_CTRL(KEY_COL1__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW0__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_COL0__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(2, 16)
	IOMUX_PAD_CTRL(KEY_ROW1__GPIO2_IO_16, WEAK_PULLUP),

	/* ECSPI2 */
	IOMUX_PAD_CTRL(SD4_CLK__ECSPI2_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__ECSPI2_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA1__ECSPI2_SCLK, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA3__ECSPI2_RDY, SPI_PAD_CTRL),
#define GP_ECSPI2_CS	IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(SD4_DATA0__GPIO6_IO_14, WEAK_PULLUP),

	/* ECSPI3 */
	IOMUX_PAD_CTRL(SD4_DATA6__ECSPI3_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA5__ECSPI3_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA4__ECSPI3_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI3_CS	IMX_GPIO_NR(6, 21)
	IOMUX_PAD_CTRL(SD4_DATA7__GPIO6_IO_21, WEAK_PULLUP),

	/* ECSPI5 */
	IOMUX_PAD_CTRL(QSPI1A_SS1_B__ECSPI5_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(QSPI1A_DQS__ECSPI5_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(QSPI1B_SS1_B__ECSPI5_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI5_CS	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(QSPI1B_DQS__GPIO4_IO_28, SPI_PAD_CTRL),

	/* enet phy */
	IOMUX_PAD_CTRL(ENET1_MDC__ENET1_MDC, PAD_CTRL_ENET_MDC),
	IOMUX_PAD_CTRL(ENET1_MDIO__ENET1_MDIO, PAD_CTRL_ENET_MDIO),

	/* fec1 */
	IOMUX_PAD_CTRL(RGMII1_TD0__ENET1_TX_DATA_0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII1_TD1__ENET1_TX_DATA_1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII1_TD2__ENET1_TX_DATA_2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII1_TD3__ENET1_TX_DATA_3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII1_TXC__ENET1_RGMII_TXC, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII1_TX_CTL__ENET1_TX_EN, PAD_CTRL_ENET_TX),
	/* AR8035 PHY Reset */
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(2, 7)
	IOMUX_PAD_CTRL(ENET2_CRS__GPIO2_IO_7, WEAK_PULLUP),
#define GP_RGMII_PHY_INT	IMX_GPIO_NR(2, 4)
	IOMUX_PAD_CTRL(ENET1_RX_CLK__GPIO2_IO_4, WEAK_PULLUP),
	IOMUX_PAD_CTRL(ENET1_TX_CLK__GPIO2_IO_5, WEAK_PULLUP),

	/* fec2 */
	IOMUX_PAD_CTRL(RGMII2_TD0__ENET2_TX_DATA_0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII2_TD1__ENET2_TX_DATA_1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII2_TD2__ENET2_TX_DATA_2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII2_TD3__ENET2_TX_DATA_3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII2_TXC__ENET2_RGMII_TXC, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII2_TX_CTL__ENET2_TX_EN, PAD_CTRL_ENET_TX),
	/* AR8035 PHY Reset */
#define GP_RGMII2_PHY_RESET	IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(ENET2_COL__GPIO2_IO_6, WEAK_PULLUP),
#define GP_RGMII2_PHY_INT	IMX_GPIO_NR(2, 8)
	IOMUX_PAD_CTRL(ENET2_RX_CLK__GPIO2_IO_8, WEAK_PULLUP),
	IOMUX_PAD_CTRL(ENET2_TX_CLK__GPIO2_IO_9, WEAK_PULLUP),

	/* hogs - GPIO */
#define GP_TERM_ON_OFF		IMX_GPIO_NR(1, 13)
	IOMUX_PAD_CTRL(GPIO1_IO13__GPIO1_IO_13, WEAK_PULLDN_OUTPUT),
#define GP_485_TERM_CTRL	IMX_GPIO_NR(3, 1)
	IOMUX_PAD_CTRL(LCD1_DATA00__GPIO3_IO_1, WEAK_PULLDN_OUTPUT),
#define GP_RESET_DSP_N		IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(LCD1_DATA02__GPIO3_IO_3, WEAK_PULLDN_OUTPUT),
#define GP_485_DIR		IMX_GPIO_NR(3, 4)
	IOMUX_PAD_CTRL(LCD1_DATA03__GPIO3_IO_4, WEAK_PULLDN_OUTPUT),
#define GP_PWR_SYNC		IMX_GPIO_NR(3, 6)
	IOMUX_PAD_CTRL(LCD1_DATA05__GPIO3_IO_6, WEAK_PULLDN_OUTPUT),
#define GP_SERVICE_LED		IMX_GPIO_NR(3, 7)
	IOMUX_PAD_CTRL(LCD1_DATA06__GPIO3_IO_7, WEAK_PULLDN_OUTPUT),
#define GP_NETWORK_LED		IMX_GPIO_NR(3, 9)
	IOMUX_PAD_CTRL(LCD1_DATA08__GPIO3_IO_9, WEAK_PULLDN_OUTPUT),
#define GP_POWER_OK_VDSP	IMX_GPIO_NR(3, 10)
	IOMUX_PAD_CTRL(LCD1_DATA09__GPIO3_IO_10, WEAK_PULLDN_OUTPUT),
#define GP_SYSTEM_LED		IMX_GPIO_NR(3, 11)
	IOMUX_PAD_CTRL(LCD1_DATA10__GPIO3_IO_11, WEAK_PULLDN_OUTPUT),
#define GP_CUST_START		IMX_GPIO_NR(3, 12)
	IOMUX_PAD_CTRL(LCD1_DATA11__GPIO3_IO_12, WEAK_PULLDN_OUTPUT),

	/* hogs - Test points */
	IOMUX_PAD_CTRL(QSPI1A_SS0_B__GPIO4_IO_22, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(QSPI1B_SS0_B__GPIO4_IO_30, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(QSPI1B_SCLK__GPIO4_IO_29, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(QSPI1B_DATA0__GPIO4_IO_24, WEAK_PULLDN_OUTPUT),

	/* PCIe */
#define GP_PCIE_RESET	IMX_GPIO_NR(4, 7)
	IOMUX_PAD_CTRL(NAND_DATA03__GPIO4_IO_7, WEAK_PULLUP),
#define GP_PCIE_DISABLE	IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(NAND_DATA04__GPIO4_IO_8, WEAK_PULLUP),
#define GP_PCIE_WAKE	IMX_GPIO_NR(4, 9)
	IOMUX_PAD_CTRL(NAND_DATA05__GPIO4_IO_9, WEAK_PULLUP),

	/* uart1 */
	IOMUX_PAD_CTRL(GPIO1_IO04__UART1_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO1_IO05__UART1_RX, UART_PAD_CTRL),

	/* uart2 */
	IOMUX_PAD_CTRL(GPIO1_IO06__UART2_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO1_IO07__UART2_RX, UART_PAD_CTRL),

	/* uart3 */
	IOMUX_PAD_CTRL(NAND_DATA07__UART3_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA06__UART3_RX, UART_PAD_CTRL),

	/* uart5 */
	IOMUX_PAD_CTRL(KEY_COL3__UART5_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW3__UART5_RX, UART_PAD_CTRL),
#define GP_RS485_RXEN		IMX_GPIO_NR(4, 22)
	IOMUX_PAD_CTRL(GPIO1_IO12__GPIO1_IO_12, WEAK_PULLUP),

	/* USB OTG1 */
	IOMUX_PAD_CTRL(GPIO1_IO08__USB_OTG1_OC, WEAK_PULLUP),
	IOMUX_PAD_CTRL(GPIO1_IO10__ANATOP_OTG1_ID, WEAK_PULLUP),
#define GP_USB_OTG1_PWR		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO_9, WEAK_PULLDN_OUTPUT),

	/* USB OTG2 */
#define GP_USB_HUB_RESET	IMX_GPIO_NR(4, 26)
	IOMUX_PAD_CTRL(QSPI1B_DATA2__GPIO4_IO_26, OUTPUT_40OHM),
#define GP_USB_HOST_PWR_EN	IMX_GPIO_NR(1, 11)
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO_11, OUTPUT_40OHM),

	/* usdhc2 - micro SD */
	IOMUX_PAD_CTRL(SD2_CLK__USDHC2_CLK, USDHC2_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__USDHC2_CMD, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA0__USDHC2_DATA0, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA1__USDHC2_DATA1, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA2__USDHC2_DATA2, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA3__USDHC2_DATA3, USDHC2_PAD_CTRL),
#define GP_USDHC2_CD	IMX_GPIO_NR(2, 12)
	IOMUX_PAD_CTRL(KEY_COL2__GPIO2_IO_12, WEAK_PULLUP),

	/* usdhc3 - eMMC */
	IOMUX_PAD_CTRL(SD3_CLK__USDHC3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__USDHC3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA0__USDHC3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA1__USDHC3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA2__USDHC3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA3__USDHC3_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA4__USDHC3_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA5__USDHC3_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA6__USDHC3_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DATA7__USDHC3_DATA7, USDHC_PAD_CTRL),
#define GP_EMMC_RESET	IMX_GPIO_NR(2, 17)
	IOMUX_PAD_CTRL(KEY_ROW2__GPIO2_IO_17, WEAK_PULLDN_OUTPUT),
};

#include "../eth.c"

static struct i2c_pads_info i2c_pads[] = {
	I2C_PADS_INFO_ENTRY(I2C1, GPIO1_IO00, 1, 0, GPIO1_IO01, 1, 1, I2C_PAD_CTRL),
	I2C_PADS_INFO_ENTRY(I2C2, GPIO1_IO02, 1, 2, GPIO1_IO03, 1, 3, I2C_PAD_CTRL),
	I2C_PADS_INFO_ENTRY(I2C3, KEY_COL4, 2, 14, KEY_ROW4, 2, 19, I2C_PAD_CTRL),
};

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : (cs >> 8) ? (cs >> 8) : -1;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;
	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
			port * 4);
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	/* Reset USB hub */
	gpio_direction_output(GP_USB_HUB_RESET, 0);
	mdelay(2);
	gpio_set_value(GP_USB_HUB_RESET, 1);
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		gpio_set_value(GP_USB_HOST_PWR_EN, on);
	else
		gpio_set_value(GP_USB_OTG1_PWR, on);
	return 0;
}
#endif

static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC2_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC3_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = (cfg->esdhc_base == USDHC2_BASE_ADDR) ? GP_USDHC2_CD : -1;

	if (gp_cd < 0)
		return 1;	/* emmc is present */
	gpio_direction_input(gp_cd);
	return !gpio_get_value(gp_cd);
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			break;
		case 1:
			gpio_set_value(GP_EMMC_RESET, 1); /* release reset */
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) then supported by the board (%d)\n",
				index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}

	return 0;
}

static const unsigned short gpios_out_low[] = {
	GP_485_DIR,
	GP_485_TERM_CTRL,
	GP_CUST_START,
	GP_EMMC_RESET,
	GP_NETWORK_LED,
	GP_PCIE_RESET,
	GP_POWER_OK_VDSP,
	GP_PWR_SYNC,
	GP_RESET_DSP_N,
	GP_RGMII_PHY_RESET,
	GP_RGMII2_PHY_RESET,
	GP_SERVICE_LED,
	GP_SYSTEM_LED,
	GP_TERM_ON_OFF,
	GP_USB_HUB_RESET,
	GP_USB_OTG1_PWR,
	GP_USB_HOST_PWR_EN,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,
	GP_ECSPI2_CS,
	GP_ECSPI3_CS,
	GP_ECSPI5_CS,
	GP_RS485_RXEN,
};

static const unsigned short gpios_in[] = {
	GP_RGMII_PHY_INT,
	GP_RGMII2_PHY_INT,
	GP_USDHC2_CD,
	GP_PCIE_WAKE,
	GP_PCIE_DISABLE,
};

static void set_gpios_in(const unsigned short *p, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_input(*p++);
}

static void set_gpios(const unsigned short *p, int cnt, int val)
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
	SETUP_IOMUX_PADS(init_pads);
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
	int i;
	struct i2c_pads_info *p = i2c_pads + i2c_get_info_entry_offset();

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < 3; i++) {
		setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}
	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();
	setenv("cpu",get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board", "ys");
	setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}

static const struct boot_mode board_boot_modes[] = {
	{"mmc0", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"mmc1", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL, 0},
};

int misc_init_r(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}
int checkboard(void)
{
	puts("Board: YS\n");
	return 0;
}
