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
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/spi.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <asm/arch/crm_regs.h>
#include <input.h>
#include <usb/ehci-ci.h>
#include "../common/bd_common.h"
#include "../common/padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL	(PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

/*
 *
 */
static iomux_v3_cfg_t const init_pads[] = {
	/* AUDMUX - mu609 usb modem */
	IOMUX_PAD_CTRL(CSI0_DAT7__AUD3_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT4__AUD3_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT5__AUD3_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT6__AUD3_TXFS, AUD_PAD_CTRL),

	/* ECSPI1 */
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS		IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLUP),

	/* ENET pads that don't change for PHY reset */
	IOMUX_PAD_CTRL(ENET_MDIO__ENET_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET_MDC, PAD_CTRL_ENET_MDC),
	IOMUX_PAD_CTRL(RGMII_TXC__RGMII_TXC, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD0__RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD1__RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD2__RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD3__RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TX_CTL__RGMII_TX_CTL, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_REF_CLK__ENET_TX_CLK, PAD_CTRL_ENET_TX),
	/* pin 42 PHY nRST */
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 27)
	IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, WEAK_PULLUP),
#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 28)
	IOMUX_PAD_CTRL(ENET_TX_EN__GPIO1_IO28, WEAK_PULLUP),

	/* FLEXCAN */
	IOMUX_PAD_CTRL(KEY_COL2__FLEXCAN1_TX, WEAK_PULLUP),
	IOMUX_PAD_CTRL(KEY_ROW2__FLEXCAN1_RX, WEAK_PULLUP),
#define GP_FLEXCAN_STANDBY	IMX_GPIO_NR(1, 2)
	IOMUX_PAD_CTRL(GPIO_2__GPIO1_IO02, WEAK_PULLUP),

	/* gpio_keys */
#define GP_S0_FACTORY_RESET	IMX_GPIO_NR(4, 6)
	IOMUX_PAD_CTRL(KEY_COL0__GPIO4_IO06, WEAK_PULLUP),	/* S0: factory reset */
#define GP_J57_INPUT	IMX_GPIO_NR(6, 6)
	IOMUX_PAD_CTRL(EIM_A23__GPIO6_IO06, WEAK_PULLUP),	/* J57: pin3 input switch */
#define GP_S1_LOOPBACK	IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, WEAK_PULLUP),	/* S1:1 - Loopback request switch */
#define GP_S1_DIAG1	IMX_GPIO_NR(4, 7)
	IOMUX_PAD_CTRL(KEY_ROW0__GPIO4_IO07, WEAK_PULLUP),	/* S1:2 - Diagnostic Switch 1 */
#define GP_S1_DIAG2	IMX_GPIO_NR(4, 8)
	IOMUX_PAD_CTRL(KEY_COL1__GPIO4_IO08, WEAK_PULLUP),	/* S1:3 - Diagnostic Switch 2 */
#define GP_S1_INPUT	IMX_GPIO_NR(2, 27)
	IOMUX_PAD_CTRL(EIM_LBA__GPIO2_IO27, WEAK_PULLUP),	/* S1:4 */

	/* i2c1 rtc rv4162 */
#define GPIRQ_RTC_RV4162	IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__GPIO4_IO15, WEAK_PULLUP),

	/* led outputs*/
	/*
	 * From antenna connector toward USB OTG
	 * connector, there are four LEDS in the
	 * order listed below.
	 */
#define GP_LED0		IMX_GPIO_NR(2, 19)
	IOMUX_PAD_CTRL(EIM_A19__GPIO2_IO19, WEAK_PULLDN),	/* Led 4 */
#define GP_LED1		IMX_GPIO_NR(2, 20)
	IOMUX_PAD_CTRL(EIM_A18__GPIO2_IO20, WEAK_PULLDN),	/* Led 3 */
#define GP_LEDRED	IMX_GPIO_NR(2, 22)
	IOMUX_PAD_CTRL(EIM_A16__GPIO2_IO22, WEAK_PULLDN),	/* Led 1 */
#define GP_LED2		IMX_GPIO_NR(2, 21)
	IOMUX_PAD_CTRL(EIM_A17__GPIO2_IO21, WEAK_PULLDN),	/* Led 2 */
#define GP_RXACT	IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO_3__GPIO1_IO03, WEAK_PULLUP),	/* RX_ACT led */
#define GP_TXACT	IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04, WEAK_PULLUP),	/* TX_ACT led */

	/* reg_usbotg_vbus */
#define GP_REG_USBOTG	IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN),

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART3 */
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),
	/* RS485 RX Enable */
#define GP_UART3_RX_EN	IMX_GPIO_NR(2, 16)
	IOMUX_PAD_CTRL(EIM_A22__GPIO2_IO16, WEAK_PULLDN),
	/* RS485 TX Enable */
#define GP_UART3_TX_EN	IMX_GPIO_NR(2, 17)
	IOMUX_PAD_CTRL(EIM_A21__GPIO2_IO17, WEAK_PULLDN),
	/* RS485/RS232 Select 2.5V */
#define GP_UART3_RS485_EN	IMX_GPIO_NR(2, 18)
	IOMUX_PAD_CTRL(EIM_A20__GPIO2_IO18, WEAK_PULLDN),
	/* ON - meaning depends on others */
#define GP_UART3_AON		IMX_GPIO_NR(7, 13)
	IOMUX_PAD_CTRL(GPIO_18__GPIO7_IO13, WEAK_PULLDN),

	/* UART4 - mu609 */
	IOMUX_PAD_CTRL(CSI0_DAT12__UART4_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT13__UART4_RX_DATA, UART_PAD_CTRL),

	/* USBH1 */
#define GP_USBH1_HUB_RESET	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, WEAK_PULLDN),
#define GP_AX88772A_RESET	IMX_GPIO_NR(2, 25)
	IOMUX_PAD_CTRL(EIM_OE__GPIO2_IO25, WEAK_PULLDN),

	/* USBH1 - mu609*/
#define GP_MODEM_RESET	IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLUP), 	/* Modem nRESET */
#define GP_MODEM_OFF	IMX_GPIO_NR(2, 5)
	IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, OUTPUT_40OHM),   	/* Modem OFF */
	IOMUX_PAD_CTRL(NANDF_D7__GPIO2_IO07, WEAK_PULLUP), 	/* Modem Sleep stat */
	IOMUX_PAD_CTRL(NANDF_WP_B__GPIO6_IO09, OUTPUT_40OHM),	/* Modem Wakeup Out */
	IOMUX_PAD_CTRL(NANDF_RB0__GPIO6_IO10, WEAK_PULLUP), 	/* Modem Wakeup In */

	/* USBOTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, USDHC_PAD_CTRL),	/* USBOTG ID pin */
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),	/* USBOTG OC pin */

	/* USDHC4 */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT4__SD4_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT5__SD4_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT6__SD4_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT7__SD4_DATA7, USDHC_PAD_CTRL),
#define GP_EMMC_RESET		IMX_GPIO_NR(2, 0)
	IOMUX_PAD_CTRL(NANDF_D0__GPIO2_IO00, WEAK_PULLUP), /* RESET (rev 1) */
};

static const struct i2c_pads_info i2c_pads[] = {
	/* I2C2 RV4162 RTC */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
};
#define I2C_BUS_CNT	1


#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	if (port) {
		/* Reset USB hub */
		gpio_direction_output(GP_USBH1_HUB_RESET, 0);
		mdelay(2);
		gpio_set_value(GP_USBH1_HUB_RESET, 1);
	}
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_REG_USBOTG, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC_IMX
struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC4_BASE_ADDR, .bus_width = 8, .gp_reset = GP_EMMC_RESET},
};
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}
#endif

static const unsigned short gpios_out_low[] = {
	GP_RGMII_PHY_RESET,
	GP_LED0,
	GP_LED1,
	GP_LEDRED,
	GP_LED2,
	GP_REG_USBOTG, 			/* disable USB otg power */
	GP_MODEM_RESET,			/* assert MODEM nRESET */
	GP_EMMC_RESET,			/* assert eMMC reset*/
	GP_UART3_RX_EN,
	GP_UART3_TX_EN,
	GP_UART3_RS485_EN,
	GP_UART3_AON,
	GP_USBH1_HUB_RESET,
	GP_AX88772A_RESET,		/* disable USB ethernet */
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,		/* SS1 of spi nor */
	GP_FLEXCAN_STANDBY,
	GP_RXACT,
	GP_TXACT,
	GP_MODEM_OFF,			/* assert MODEM off */
};

static const unsigned short gpios_in[] = {
	GPIRQ_ENET_PHY,
	GP_S0_FACTORY_RESET,		/* S0: factory reset */
	GP_J57_INPUT,			/* J57: pin3 input switch */
	GP_S1_LOOPBACK,			/* S1:1 - Loopback request switch */
	GP_S1_DIAG1,			/* S1:2 - Diagnostic Switch 1 */
	GP_S1_DIAG2,			/* S1:3 - Diagnostic Switch 2 */
	GP_S1_INPUT,			/* S1:4 */
	GPIRQ_RTC_RV4162,
};

int board_early_init_f(void)
{
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	SETUP_IOMUX_PADS(init_pads);
	return 0;
}

int board_init(void)
{
	common_board_init(i2c_pads, I2C_BUS_CNT, IOMUXC_GPR1_OTG_ID_GPIO1,
			NULL, 0, 0);
	return 0;
}

const struct button_key board_buttons[] = {
	{"factory",	GP_S0_FACTORY_RESET,	'F', 1},
	{"input",	GP_J57_INPUT,	'I', 1},
#if 0
	{"D1",	GP_S1_LOOPBACK,	'1', 0},	/* S1:1 - Loopback request switch */
	{"D2",	GP_S1_DIAG1,	'2', 0},	/* S1:2 - Diagnostic Switch 1 */
	{"D3",	GP_S1_DIAG2,	'3', 0},	/* S1:3 - Diagnostic Switch 2 */
	{"D4",	GP_S1_INPUT,	'4', 0},	/* S1:4 */
#endif
	{NULL, 0, 0, 0},
};

#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)}, /* usdhc4 */
	{NULL,		0},
};
#endif

#define PROGRESS_BITS 3

static int const leds[] = {
	GP_LED0,
	GP_LED1,
	GP_LEDRED,
	GP_LED2
};

void gzwrite_progress_init(u64 expected_size)
{
	int i;
	putc('\n');
	for (i = 0; i < ARRAY_SIZE(leds); i++)
		gpio_direction_output(leds[i], 0);
}

void gzwrite_progress(int iteration,
		     u64 bytes_written,
		     u64 total_bytes)
{
	int i;
	if (0 == (iteration & 3))
		printf("%llu/%llu\r", bytes_written, total_bytes);

	for (i = 0; i < 2; i++)
		gpio_set_value(leds[i], (iteration & 1) == i);
}

void gzwrite_progress_finish(int returnval, /* 0 == success */
			    u64 totalwritten,
                            u64 totalsize,
                            u32 expected_crc,
                            u32 calculated_crc)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(leds); i++)
		gpio_set_value(leds[i], 0);

	if (0 == returnval) {
		printf("\n\t%llu bytes, crc 0x%08x\n",
		       totalwritten, calculated_crc);
		gpio_set_value(leds[3], 1);
	} else {
		printf("\n\tuncompressed %llu of %llu\n"
		       "\tcrcs == 0x%08x/0x%08x\n",
		       totalwritten, totalsize,
		       expected_crc, calculated_crc);
		gpio_set_value(leds[2], 1);
	}
}
