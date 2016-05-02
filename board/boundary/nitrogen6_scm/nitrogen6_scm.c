/*
 * Copyright (C) 2015 Boundary Devices, Inc.
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
#include <asm/imx-common/fbpanel.h>
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

DECLARE_GLOBAL_DATA_PTR;

#define CSI_PAD_CTL	PAD_CTL_DSE_120ohm

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_HIGH | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define ENET_RXD_DN_PAD_CTRL  (PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_HIGH | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define ENET_RXD_UP_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_HIGH | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define ESAI_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_ODE)

#define LCDIF_PAD_CTL	PAD_CTL_DSE_120ohm

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC1_PAD_CTRL (PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC1_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC2_PAD_CTRL (PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC2_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_80ohm | PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLDN_OUTPUT (PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP_OUTPUT (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_SLOW)

static const iomux_v3_cfg_t init_pads[] = {
	/* bt_rfkill */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(2, 18)
	IOMUX_PAD_CTRL(KEY_ROW3__GPIO2_IO_18, WEAK_PULLDN),

	/* CSI */
	IOMUX_PAD_CTRL(CSI_MCLK__CSI1_MCLK, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_PIXCLK__CSI1_PIXCLK, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_VSYNC__CSI1_VSYNC, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_HSYNC__CSI1_HSYNC, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA00__CSI1_DATA_2, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA01__CSI1_DATA_3, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA02__CSI1_DATA_4, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA03__CSI1_DATA_5, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA04__CSI1_DATA_6, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA05__CSI1_DATA_7, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA06__CSI1_DATA_8, CSI_PAD_CTL),
	IOMUX_PAD_CTRL(CSI_DATA07__CSI1_DATA_9, CSI_PAD_CTL),

#define GP_OV5642_RESET		IMX_GPIO_NR(7, 4)
	IOMUX_PAD_CTRL(SD3_DATA2__GPIO7_IO_4, WEAK_PULLDN),
#define GP_OV5642_PWRDN		IMX_GPIO_NR(7, 5)
	IOMUX_PAD_CTRL(SD3_DATA3__GPIO7_IO_5, WEAK_PULLUP),

	/* ECSPI1 (serial nor eeprom) */
	IOMUX_PAD_CTRL(KEY_COL1__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW0__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_COL0__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(2, 16)
	IOMUX_PAD_CTRL(KEY_ROW1__GPIO2_IO_16, WEAK_PULLUP),

	/* ESAI */
	IOMUX_PAD_CTRL(NAND_WE_B__ESAI_TX5_RX0, ESAI_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_CE0_B__ESAI_TX_CLK, ESAI_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_READY_B__ESAI_TX1, ESAI_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_RE_B__ESAI_TX_FS, ESAI_PAD_CTRL),

	/* gpio - keys */
#define GP_GPIOKEY_POWER	IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(QSPI1B_DATA3__GPIO4_IO_27, WEAK_PULLUP),

	/* gpio - output */
#define GP_POWER_OFF		IMX_GPIO_NR(4, 26)
	IOMUX_PAD_CTRL(QSPI1B_DATA2__GPIO4_IO_26, WEAK_PULLDN),

	/* hogs - Test points */
#define GP_TP16		IMX_GPIO_NR(7, 6)
	IOMUX_PAD_CTRL(SD3_DATA4__GPIO7_IO_6, WEAK_PULLUP),
#define GP_TP17		IMX_GPIO_NR(7, 7)
	IOMUX_PAD_CTRL(SD3_DATA5__GPIO7_IO_7, WEAK_PULLUP),
#define GP_TP18		IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(SD4_DATA0__GPIO6_IO_14, WEAK_PULLUP),
#define GP_TP19		IMX_GPIO_NR(6, 15)
	IOMUX_PAD_CTRL(SD4_DATA1__GPIO6_IO_15, WEAK_PULLUP),

#define GP_I2C3_J4_RESET	IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO_8, WEAK_PULLUP),
#define GPIRQ_I2C3_J4	IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO_9, WEAK_PULLUP),

	/* i2c4a - max77818 */
#define GPIRQ_MAX77818_INTB		IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_CLK__GPIO7_IO_0, WEAK_PULLUP),
#define GPIRQ_MAX77818_WCINOKB		IMX_GPIO_NR(7, 8)
	IOMUX_PAD_CTRL(SD3_DATA6__GPIO7_IO_8, WEAK_PULLUP),
#define GPIRQ_MAX77818_INOKB	IMX_GPIO_NR(7, 9)
	IOMUX_PAD_CTRL(SD3_DATA7__GPIO7_IO_9, WEAK_PULLUP),

	/* PWM1 for rgb panel */
#define GP_BACKLIGHT_RGB 	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO_10, WEAK_PULLDN_OUTPUT),

	/* reg_5v_en */
#define GP_5V_BST_EN	IMX_GPIO_NR(6, 12)
	IOMUX_PAD_CTRL(SD4_CLK__GPIO6_IO_12, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN	IMX_GPIO_NR(2, 13)
	IOMUX_PAD_CTRL(KEY_COL3__GPIO2_IO_13, WEAK_PULLDN),
	/* 32K clock, off for now */
#define GP_WLAN_LF_CLK		IMX_GPIO_NR(1, 11)
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO_11, WEAK_PULLDN_OUTPUT),

	/* uart1 */
	IOMUX_PAD_CTRL(GPIO1_IO04__UART1_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO1_IO05__UART1_RX, UART_PAD_CTRL),

	/* uart2 */
	IOMUX_PAD_CTRL(GPIO1_IO06__UART2_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO1_IO07__UART2_RX, UART_PAD_CTRL),

	/* uart3 - wifi */
	IOMUX_PAD_CTRL(NAND_DATA07__UART3_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA06__UART3_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA05__UART3_CTS_B, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(NAND_DATA04__UART3_RTS_B, UART_PAD_CTRL),

	/* uart5 */
	IOMUX_PAD_CTRL(SD4_DATA5__UART5_TX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DATA4__UART5_RX, UART_PAD_CTRL),

	/* USB OTG */
#define GP_USB_OTG1_OC		IMX_GPIO_NR(4, 29)
	IOMUX_PAD_CTRL(QSPI1B_SCLK__GPIO4_IO_29, WEAK_PULLUP),
#define GP_USB_OTG1_ID		IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(QSPI1B_DQS__GPIO4_IO_28, WEAK_PULLUP),
#define GP_USB_OTG1_PWR		IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(QSPI1B_DATA0__GPIO4_IO_24, WEAK_PULLDN_OUTPUT),

	/* USB OTG2 */
#define GP_USB_OTG2_OC		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(QSPI1B_SS1_B__GPIO4_IO_31, WEAK_PULLDN_OUTPUT),
#define GP_USB_OTG2_PWR		IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(QSPI1B_DATA1__GPIO4_IO_25, WEAK_PULLDN_OUTPUT),

	/* usdhc1 - microSD */
	IOMUX_PAD_CTRL(SD1_CLK__USDHC1_CLK, USDHC1_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CMD__USDHC1_CMD, USDHC1_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA0__USDHC1_DATA0, USDHC1_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA1__USDHC1_DATA1, USDHC1_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA2__USDHC1_DATA2, USDHC1_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DATA3__USDHC1_DATA3, USDHC1_PAD_CTRL),
#define GP_USDHC1_CD	IMX_GPIO_NR(4, 30)
	IOMUX_PAD_CTRL(QSPI1B_SS0_B__GPIO4_IO_30, WEAK_PULLUP),

	/* usdhc2 - wifi */
	IOMUX_PAD_CTRL(SD2_CLK__USDHC2_CLK, USDHC2_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__USDHC2_CMD, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA0__USDHC2_DATA0, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA1__USDHC2_DATA1, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA2__USDHC2_DATA2, USDHC2_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DATA3__USDHC2_DATA3, USDHC2_PAD_CTRL),

	/* WLAN wifi silex */
#define GPIRQ_WLAN		IMX_GPIO_NR(2, 14)
	IOMUX_PAD_CTRL(KEY_COL4__GPIO2_IO_14, WEAK_PULLDN),
#define GP_WLAN_CLK_REQ		IMX_GPIO_NR(2, 12)
	IOMUX_PAD_CTRL(KEY_COL2__GPIO2_IO_12, WEAK_PULLDN),
#define GP_WLAN_QOW		IMX_GPIO_NR(2, 17)
	IOMUX_PAD_CTRL(KEY_ROW2__GPIO2_IO_17, WEAK_PULLDN),
#define GP_BT_HOST_WAKE 	IMX_GPIO_NR(1, 13)
	IOMUX_PAD_CTRL(GPIO1_IO13__GPIO1_IO_13, WEAK_PULLDN),
};


#ifdef CONFIG_CMD_FBPANEL
static const iomux_v3_cfg_t rgb_pads[] = {
	/* LCDIF1 */
	IOMUX_PAD_CTRL(LCD1_CLK__LCDIF1_CLK, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_ENABLE__LCDIF1_ENABLE, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_HSYNC__LCDIF1_HSYNC, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_VSYNC__LCDIF1_VSYNC, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_RESET__LCDIF1_RESET, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA00__LCDIF1_DATA_0, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA01__LCDIF1_DATA_1, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA02__LCDIF1_DATA_2, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA03__LCDIF1_DATA_3, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA04__LCDIF1_DATA_4, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA05__LCDIF1_DATA_5, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA06__LCDIF1_DATA_6, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA07__LCDIF1_DATA_7, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA08__LCDIF1_DATA_8, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA09__LCDIF1_DATA_9, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA10__LCDIF1_DATA_10, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA11__LCDIF1_DATA_11, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA12__LCDIF1_DATA_12, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA13__LCDIF1_DATA_13, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA14__LCDIF1_DATA_14, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA15__LCDIF1_DATA_15, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA16__LCDIF1_DATA_16, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA17__LCDIF1_DATA_17, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA18__LCDIF1_DATA_18, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA19__LCDIF1_DATA_19, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA20__LCDIF1_DATA_20, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA21__LCDIF1_DATA_21, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA22__LCDIF1_DATA_22, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA23__LCDIF1_DATA_23, LCDIF_PAD_CTL),
};
#endif

static const iomux_v3_cfg_t rgb_gpio_pads[] = {
	/* LCDIF1 */
	IOMUX_PAD_CTRL(LCD1_CLK__GPIO3_IO_0, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_ENABLE__GPIO3_IO_25, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_HSYNC__GPIO3_IO_26, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_VSYNC__GPIO3_IO_28, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_RESET__GPIO3_IO_27, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA00__GPIO3_IO_1, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA01__GPIO3_IO_2, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA02__GPIO3_IO_3, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA03__GPIO3_IO_4, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA04__GPIO3_IO_5, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA05__GPIO3_IO_6, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA06__GPIO3_IO_7, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA07__GPIO3_IO_8, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA08__GPIO3_IO_9, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA09__GPIO3_IO_10, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA10__GPIO3_IO_11, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA11__GPIO3_IO_12, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA12__GPIO3_IO_13, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA13__GPIO3_IO_14, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA14__GPIO3_IO_15, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA15__GPIO3_IO_16, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA16__GPIO3_IO_17, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA17__GPIO3_IO_18, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA18__GPIO3_IO_19, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA19__GPIO3_IO_20, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA20__GPIO3_IO_21, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA21__GPIO3_IO_22, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA22__GPIO3_IO_23, LCDIF_PAD_CTL),
	IOMUX_PAD_CTRL(LCD1_DATA23__GPIO3_IO_24, LCDIF_PAD_CTL),
};

static struct i2c_pads_info i2c_pads[] = {
	I2C_PADS_INFO_ENTRY(I2C2, GPIO1_IO02, 1, 2, GPIO1_IO03, 1, 3, I2C_PAD_CTRL),	/* PMIC */
	I2C_PADS_INFO_ENTRY(I2C3, ENET2_RX_CLK, 2, 8, ENET2_TX_CLK, 2, 9, I2C_PAD_CTRL), /* J4 touch */
	I2C_PADS_INFO_ENTRY(I2C4, SD3_DATA0, 7, 2, SD3_DATA1, 7, 3, I2C_PAD_CTRL),	/* PCA9540B switch, charger/ov5642 */
};

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);
	return 0;
}


int board_eth_init(bd_t *bis)
{
#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	if (!getenv("eth1addr"))
		setenv("eth1addr", getenv("usbnet_devaddr"));
	usb_eth_initialize(bis);
#endif
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
	return 0;
}

int board_ehci_power(int port, int on)
{
	gpio_set_value(port ? GP_USB_OTG2_PWR: GP_USB_OTG1_PWR, on);
	return 0;
}
#endif

static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC1_BASE_ADDR, .max_bus_width = 4},
};

int board_mmc_getcd(struct mmc *mmc)
{
	int gp_cd = GP_USDHC1_CD;

	if (gp_cd < 0)
		return 1;	/* emmc is present */
	gpio_direction_input(gp_cd);
	return !gpio_get_value(gp_cd);
}

int board_mmc_init(bd_t *bis)
{
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}

#ifdef CONFIG_CMD_FBPANEL
#if 0
void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
	gpio_direction_output(GP_LVDS_ENABLE, enable);
}

void board_enable_lcd(const struct display_info_t *di, int enable)
{
	if (enable)
		SETUP_IOMUX_PADS(rgb_pads);
	else
		SETUP_IOMUX_PADS(rgb_gpio_pads);
	gpio_direction_output(GP_BACKLIGHT_RGB, enable);
}
#endif

static const struct display_info_t displays[] = {
	/* tsc2004 */
	VDF_DC050WX(LCD, "DC050WX", RGB24, 0, NULL, 2, 0x48),
	VDF_CLAA_WVGA(LCD, "CLAA-WVGA", RGB666, 0, fbp_detect_i2c, 2, 0x48),
	VDF_SHARP_WVGA(LCD, "sharp-wvga", RGB24, 0, NULL, 2, 0x48),
	VDF_QVGA(LCD, "qvga", RGB24, 0, NULL, 2, 0x48),
	VDF_AT035GT_07ET3(LCD, "AT035GT-07ET3", RGB24, 0, NULL, 2, 0x48),

	VDF_1280_720M_60(LCD, "1280x720M@60", RGB24, 0, fbp_detect_i2c, 2, 0x50),
	VDF_1920_1080M_60(LCD, "1920x1080M@60", RGB24, 0, NULL, 2, 0x50),
	VDF_1024_768M_60(LCD, "1024x768M@60", RGB24, 0, NULL, 2, 0x50),

	/* fusion7 specific touchscreen */
	VDF_FUSION7(LCD, "fusion7", RGB666, 0, fbp_detect_i2c, 2, 0x10),

	VDF_LSA40AT9001(LCD, "LSA40AT9001", RGB24, 0, NULL, 0, 0x00),
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}

#endif

static const unsigned short gpios_out_low[] = {
	GP_POWER_OFF,		/* 0 - on */
	GP_5V_BST_EN,		/* 0 - off */
	GP_USB_OTG1_PWR,	/* 0 - off */
	GP_USB_OTG2_PWR,	/* 0 - off */
	GP_OV5642_RESET,
	GP_I2C3_J4_RESET,
	GP_BACKLIGHT_RGB,
	GP_REG_WLAN_EN,
	GP_BT_RFKILL_RESET,
	GP_WLAN_LF_CLK,
};

static const unsigned short gpios_out_high[] = {
	GP_OV5642_PWRDN,
	GP_ECSPI1_NOR_CS,
};

static const unsigned short gpios_in[] = {
	GP_TP16,
	GP_TP17,
	GP_TP18,
	GP_TP19,
	GP_GPIOKEY_POWER,
	GPIRQ_MAX77818_INTB,
	GPIRQ_MAX77818_WCINOKB,
	GPIRQ_MAX77818_INOKB,
	GPIRQ_I2C3_J4,
	GP_USB_OTG1_OC,
	GP_USB_OTG1_ID,
	GP_USB_OTG2_OC,
	GP_USDHC1_CD,
	GPIRQ_WLAN,
	GP_WLAN_CLK_REQ,
	GP_WLAN_QOW,
	GP_BT_HOST_WAKE,
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
	SETUP_IOMUX_PADS(rgb_gpio_pads);
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

void board_poweroff(void)
{
	gpio_set_value(GP_POWER_OFF, 1);
	mdelay(500);
}

int board_init(void)
{
	int i;
	int ret;
	struct i2c_pads_info *p = i2c_pads + i2c_get_info_entry_offset();
	u8 orig_i2c_bus;
	u8 val8;
	u8 buf[2];

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < 3; i++) {
		setup_i2c(i + 1, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(3);

#define I2C_ADDR_CHARGER	0x69
	val8 = 0x7f;	/* 4.0A source */
	i2c_write(I2C_ADDR_CHARGER, 0xc0, 1, &val8, 1);
	val8 = 0x0c;	/* Protection allow 0xb9 write */
	i2c_write(I2C_ADDR_CHARGER, 0xbd, 1, &val8, 1);
	val8 = 0x14;	/* 1A charge */
	i2c_write(I2C_ADDR_CHARGER, 0xb9, 1, &val8, 1);
	val8 = 0x27;	/* enable charging from otg */
	i2c_write(I2C_ADDR_CHARGER, 0xc3, 1, &val8, 1);
	val8 = 0x5;	/* enable charging mode */
	i2c_write(I2C_ADDR_CHARGER, 0xb7, 1, &val8, 1);

#define I2C_ADDR_FUELGAUGE	0x36
#define MAX77823_REG_VCELL	0x09
	ret = i2c_read(I2C_ADDR_FUELGAUGE, MAX77823_REG_VCELL, 1, buf, 2);
	if (!ret) {
		u32 v = (buf[1] << 8) | buf[0];

		v = (v >> 3) * 625;
		printf("battery voltage = %d uV\n", v);
		if (v < 3000000) {
			printf("voltage = %d uV too low, powering off\n", v);
			board_poweroff();
		}
	} else {
		printf("error reading battery voltage\n");
	}
	i2c_set_bus_num(orig_i2c_bus);

#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();
	setenv("cpu",get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board", "nitrogen6_scm");
	setenv("uboot_defconfig", CONFIG_DEFCONFIG);
#ifdef CONFIG_CMD_FBPANEL
	board_video_skip();
#endif

	return 0;
}

static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",        MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"mmc1",        MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,          0},
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
	puts("Board: nitrogen6_scm\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"Power",	GP_GPIOKEY_POWER,	'P'},
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

static int _do_poweroff(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();
	return 0;
}

U_BOOT_CMD(
	poweroff, 70, 0, _do_poweroff,
	"power down board",
	""
);

