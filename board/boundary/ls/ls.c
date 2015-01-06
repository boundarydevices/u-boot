/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014, Boundary Devices <info@boundarydevices.com>
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
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <splash.h>
#include <usb/ehci-fsl.h>

DECLARE_GLOBAL_DATA_PTR;

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define CEC_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_PUS_22K_UP | PAD_CTL_ODE | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_CLK_PAD_CTRL (PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (USDHC_CLK_PAD_CTRL | PAD_CTL_PUS_47K_UP)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLDN_OUTPUT (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |	PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP_OUTPUT (PAD_CTL_PUS_100K_UP |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_SLOW)


static const iomux_v3_cfg_t ls_pads[] = {
//	IOMUX_PAD_CTRL(GPIO_8__XTALOSC_REF_CLK_32K, OUTPUT_40OHM),	/* TiWi, WM5102, GTM609W */

	/* Accelerometer (MPU-9250) (i2c3) */
#define GP_ACCEL_IRQ		IMX_GPIO_NR(5, 30)
	IOMUX_PAD_CTRL(CSI0_DAT12__GPIO5_IO30, WEAK_PULLUP),

	/* AUDMUX  - GSM */
	IOMUX_PAD_CTRL(CSI0_DAT7__AUD3_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT4__AUD3_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT5__AUD3_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT6__AUD3_TXFS, AUD_PAD_CTRL),

	/* AUDMUX  - wm5102 */
	IOMUX_PAD_CTRL(DI0_PIN4__AUD6_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__AUD6_TXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN2__AUD6_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN3__AUD6_TXFS, AUD_PAD_CTRL),

	/* Camera (mipi) - LM3555 strobe/flash */
#ifdef CONFIG_NEW_REV
#define GP_CAM_STROBE		IMX_GPIO_NR(3, 28)
	IOMUX_PAD_CTRL(EIM_D28__GPIO3_IO28, WEAK_PULLDN_OUTPUT),
#define GP_CAM_TORCH		IMX_GPIO_NR(3, 30)
	IOMUX_PAD_CTRL(EIM_D30__GPIO3_IO30, WEAK_PULLDN_OUTPUT),
#else
#define GP_CAM_STROBE		IMX_GPIO_NR(3, 18)
	IOMUX_PAD_CTRL(EIM_D18__GPIO3_IO18, WEAK_PULLDN_OUTPUT),
#define GP_CAM_TORCH		IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLDN_OUTPUT),
#endif

#define GP_CAM_MIPI_RESET	IMX_GPIO_NR(1, 30)
	IOMUX_PAD_CTRL(ENET_TXD0__GPIO1_IO30, WEAK_PULLDN_OUTPUT),
#define GP_CAM_MIPI_EN		IMX_GPIO_NR(1, 29)
	IOMUX_PAD_CTRL(ENET_TXD1__GPIO1_IO29, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(GPIO_3__CCM_CLKO2, OUTPUT_40OHM),	/* XCLK */

	/* Display (Braille) */
#define GP_DISP_DATA_BUF_EN	IMX_GPIO_NR(5, 28)
	IOMUX_PAD_CTRL(CSI0_DAT10__GPIO5_IO28, WEAK_PULLDN_OUTPUT),
#define GP_DISP_MISO	IMX_GPIO_NR(5, 18)
	IOMUX_PAD_CTRL(CSI0_PIXCLK__GPIO5_IO18, WEAK_PULLUP),
#define GP_DISP_MOSI		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, WEAK_PULLDN_OUTPUT),
#define GP_DISP_CLK		IMX_GPIO_NR(4, 16)
	IOMUX_PAD_CTRL(DI0_DISP_CLK__GPIO4_IO16, WEAK_PULLDN_OUTPUT),
#define GP_DISP_STROBE		IMX_GPIO_NR(5, 8)
	IOMUX_PAD_CTRL(DISP0_DAT14__GPIO5_IO08, WEAK_PULLDN_OUTPUT),
#define GP_DISP_HV_EN		IMX_GPIO_NR(2, 24)
	IOMUX_PAD_CTRL(EIM_CS1__GPIO2_IO24, WEAK_PULLDN_OUTPUT),

#define GP_DISP_KEY_1		IMX_GPIO_NR(3, 6)
	IOMUX_PAD_CTRL(EIM_DA6__GPIO3_IO06, WEAK_PULLUP),
#define GP_DISP_KEY_2		IMX_GPIO_NR(3, 7)
	IOMUX_PAD_CTRL(EIM_DA7__GPIO3_IO07, WEAK_PULLUP),
#define GP_DISP_KEY_3		IMX_GPIO_NR(3, 8)
	IOMUX_PAD_CTRL(EIM_DA8__GPIO3_IO08, WEAK_PULLUP),
#define GP_DISP_KEY_4		IMX_GPIO_NR(3, 9)
	IOMUX_PAD_CTRL(EIM_DA9__GPIO3_IO09, WEAK_PULLUP),
#define GP_DISP_KEY_5		IMX_GPIO_NR(3, 10)
	IOMUX_PAD_CTRL(EIM_DA10__GPIO3_IO10, WEAK_PULLUP),
#define GP_DISP_KEY_6		IMX_GPIO_NR(3, 11)
	IOMUX_PAD_CTRL(EIM_DA11__GPIO3_IO11, WEAK_PULLUP),
#define GP_DISP_KEY_SHIFT	IMX_GPIO_NR(3, 12)
	IOMUX_PAD_CTRL(EIM_DA12__GPIO3_IO12, WEAK_PULLUP),
#define GP_DISP_KEY_SPACE	IMX_GPIO_NR(3, 13)
	IOMUX_PAD_CTRL(EIM_DA13__GPIO3_IO13, WEAK_PULLUP),
#define GP_DISP_KEY_CTRL	IMX_GPIO_NR(3, 14)
	IOMUX_PAD_CTRL(EIM_DA14__GPIO3_IO14, WEAK_PULLUP),
#define GP_DISP_LED_RED		IMX_GPIO_NR(5, 9)
	IOMUX_PAD_CTRL(DISP0_DAT15__GPIO5_IO09, WEAK_PULLDN_OUTPUT),
#define GP_DISP_LED_GREEN	IMX_GPIO_NR(5, 10)
	IOMUX_PAD_CTRL(DISP0_DAT16__GPIO5_IO10, WEAK_PULLDN_OUTPUT),

	/* Keypad */
	IOMUX_PAD_CTRL(KEY_COL0__KEY_COL0, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(KEY_COL1__KEY_COL1, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(KEY_COL2__KEY_COL2, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(KEY_COL3__KEY_COL3, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(KEY_COL4__KEY_COL4, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(GPIO_19__KEY_COL5, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(KEY_ROW0__KEY_ROW0, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(KEY_ROW1__KEY_ROW1, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(KEY_ROW2__KEY_ROW2, WEAK_PULLDN_OUTPUT),

	/* ECSPI1 pads (serial nor eeprom) */
#ifdef CONFIG_NEW_REV
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLUP),
#else
	IOMUX_PAD_CTRL(DISP0_DAT22__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT21__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT20__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(5, 17)
	IOMUX_PAD_CTRL(DISP0_DAT23__GPIO5_IO17, WEAK_PULLUP),
#endif

	/* ECSPI3 pads GSM (GTM609W) */
	IOMUX_PAD_CTRL(DISP0_DAT2__ECSPI3_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT1__ECSPI3_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT0__ECSPI3_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI3_GSM		IMX_GPIO_NR(4, 24)
	IOMUX_PAD_CTRL(DISP0_DAT3__GPIO4_IO24, WEAK_PULLUP),

	/* ECSPI5 pads sound(wm5102) */
	IOMUX_PAD_CTRL(SD1_DAT0__ECSPI5_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CMD__ECSPI5_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CLK__ECSPI5_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI5_WM5102	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(SD1_DAT1__GPIO1_IO17, WEAK_PULLUP),

	/* GSM sim (GTM609W) */
#define GP_GSM_SIM_RESET	IMX_GPIO_NR(5, 19)
	IOMUX_PAD_CTRL(CSI0_MCLK__GPIO5_IO19, WEAK_PULLUP),
#define GP_GSM_HOST_WAKE_WWAN	IMX_GPIO_NR(5, 20)
	IOMUX_PAD_CTRL(CSI0_DATA_EN__GPIO5_IO20, WEAK_PULLUP),
#define GP_GSM_PWR_EN		IMX_GPIO_NR(6, 2)
	IOMUX_PAD_CTRL(CSI0_DAT16__GPIO6_IO02, WEAK_PULLUP),
#define GP_GSM_RESET		IMX_GPIO_NR(6, 3)
	IOMUX_PAD_CTRL(CSI0_DAT17__GPIO6_IO03, WEAK_PULLUP),
#define GP_GSM_ON_OFF		IMX_GPIO_NR(6, 4)
	IOMUX_PAD_CTRL(CSI0_DAT18__GPIO6_IO04, WEAK_PULLUP),
#define GP_GSM_HOST_WAKE	IMX_GPIO_NR(6, 5)
	IOMUX_PAD_CTRL(CSI0_DAT19__GPIO6_IO05, WEAK_PULLUP),
//	IOMUX_PAD_CTRL(GPIO_8__XTALOSC_REF_CLK_32K, OUTPUT_40OHM),	/* slow clock */

	/* GPS */
#define GP_GPS_HEARTBEAT	IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(DISP0_DAT6__GPIO4_IO27, WEAK_PULLDN_OUTPUT),
#define GP_GPS_IRQ		IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(DISP0_DAT7__GPIO4_IO28, WEAK_PULLDN_OUTPUT),
#define GP_GPS_RESET		IMX_GPIO_NR(4, 30)
	IOMUX_PAD_CTRL(DISP0_DAT9__GPIO4_IO30, WEAK_PULLDN_OUTPUT),

	/* HDMI CEC */
	IOMUX_PAD_CTRL(EIM_A25__HDMI_TX_CEC_LINE, CEC_PAD_CTRL),

	/* I2C1 - rv4162(rtc), */

	/* I2C2 - J12(touch connector), */
#define GP_I2C2_HDMI_EN	IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(DISP0_DAT11__GPIO5_IO05, WEAK_PULLDN_OUTPUT),
#define GP_I2C2_CAMERA_EN	IMX_GPIO_NR(1, 31)
	IOMUX_PAD_CTRL(ENET_MDC__GPIO1_IO31, WEAK_PULLDN_OUTPUT),
#ifdef CONFIG_NEW_REV
#define GP_I2C2_FLASH_EN	IMX_GPIO_NR(2, 31)
	IOMUX_PAD_CTRL(EIM_EB3__GPIO2_IO31, WEAK_PULLDN_OUTPUT),
#else
#define GP_I2C2_FLASH_EN	IMX_GPIO_NR(3, 17)
	IOMUX_PAD_CTRL(EIM_D17__GPIO3_IO17, WEAK_PULLDN_OUTPUT),
#endif

	/* I2C3 - MPR21 cap touch, MPU-9250(accelerometer) */
#define GP_I2C3_ACCEL_EN	IMX_GPIO_NR(5, 31)
	IOMUX_PAD_CTRL(CSI0_DAT13__GPIO5_IO31, WEAK_PULLDN_OUTPUT),
#define GP_I2C3_MAX77818_EN	IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(DISP0_DAT4__GPIO4_IO25, WEAK_PULLDN_OUTPUT),

	/* MAX77818, ends with B(active low) */
#define GP_MAX77818_INOKB		IMX_GPIO_NR(4, 26)	/* C5 */
	IOMUX_PAD_CTRL(DISP0_DAT5__GPIO4_IO26, WEAK_PULLUP),
#define GP_MAX77818_INTB		IMX_GPIO_NR(5, 6)	/* C3 */
	IOMUX_PAD_CTRL(DISP0_DAT12__GPIO5_IO06, WEAK_PULLUP),
#define GP_MAX77818_WCINOKB		IMX_GPIO_NR(5, 7)	/* A5 */
	IOMUX_PAD_CTRL(DISP0_DAT13__GPIO5_IO07, WEAK_PULLUP),

	/* Power */
#define GP_MAIN_ON_OFF		IMX_GPIO_NR(3, 20)	/* input to or gate */
	IOMUX_PAD_CTRL(EIM_D20__GPIO3_IO20, WEAK_PULLUP),
#define GP_ON_OFF_IRQ		IMX_GPIO_NR(5, 11)
	IOMUX_PAD_CTRL(DISP0_DAT17__GPIO5_IO11, WEAK_PULLUP),

	/* PMIC */
#define GP_PMIC_C0_OUT		IMX_GPIO_NR(2, 28)
	IOMUX_PAD_CTRL(EIM_EB0__GPIO2_IO28, WEAK_PULLUP),
#define GP_PMIC_C1_OUT		IMX_GPIO_NR(2, 29)
	IOMUX_PAD_CTRL(EIM_EB1__GPIO2_IO29, WEAK_PULLUP),
#define GP_PMIC_C2_OUT		IMX_GPIO_NR(3, 0)
	IOMUX_PAD_CTRL(EIM_DA0__GPIO3_IO00, WEAK_PULLUP),
#define GP_PMIC_C3_OUT		IMX_GPIO_NR(3, 1)
	IOMUX_PAD_CTRL(EIM_DA1__GPIO3_IO01, WEAK_PULLUP),
#define GP_PMIC_C4_OUT		IMX_GPIO_NR(2, 25)
	IOMUX_PAD_CTRL(EIM_OE__GPIO2_IO25, WEAK_PULLUP),
#define GP_PMIC_C5_OUT		IMX_GPIO_NR(2, 27)
	IOMUX_PAD_CTRL(EIM_LBA__GPIO2_IO27, WEAK_PULLUP),
#define GP_PMIC_R0_IN		IMX_GPIO_NR(3, 2)
	IOMUX_PAD_CTRL(EIM_DA2__GPIO3_IO02, WEAK_PULLUP),
#define GP_PMIC_R1_IN		IMX_GPIO_NR(3, 3)
	IOMUX_PAD_CTRL(EIM_DA3__GPIO3_IO03, WEAK_PULLUP),
#define GP_PMIC_R2_IN		IMX_GPIO_NR(3, 4)
	IOMUX_PAD_CTRL(EIM_DA4__GPIO3_IO04, WEAK_PULLUP),
#define GP_PMIC_R3_IN		IMX_GPIO_NR(3, 5)
	IOMUX_PAD_CTRL(EIM_DA5__GPIO3_IO05, WEAK_PULLUP),
#define GP_PMIC_R4_IN		IMX_GPIO_NR(2, 23)
	IOMUX_PAD_CTRL(EIM_CS0__GPIO2_IO23, WEAK_PULLUP),
#define GP_PMIC_R5_IN		IMX_GPIO_NR(2, 26)
	IOMUX_PAD_CTRL(EIM_RW__GPIO2_IO26, WEAK_PULLUP),
#define GP_PMIC_KEYPAD_LOCK	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(EIM_WAIT__GPIO5_IO00, WEAK_PULLUP),

	/* PWM1 */
#define GP_PWM1_BUZZER		IMX_GPIO_NR(4, 29)
	IOMUX_PAD_CTRL(DISP0_DAT8__GPIO4_IO29, WEAK_PULLDN_OUTPUT),

	/* Regulator - HDMI, USBH1_DN3, WM5102 speaker, braille display */
#define GP_REG_5V_EN	IMX_GPIO_NR(7, 8)
	IOMUX_PAD_CTRL(SD3_RST__GPIO7_IO08, WEAK_PULLUP_OUTPUT),

	/* rtc */
#define GP_RTC_RV4162_IRQ	IMX_GPIO_NR(5, 12)
	IOMUX_PAD_CTRL(DISP0_DAT18__GPIO5_IO12, WEAK_PULLUP),

	/* Test points */
#define GP_TP6			IMX_GPIO_NR(1, 4)
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04, WEAK_PULLUP),	/* 1.8V */
#define GP_TP7			IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLUP),	/* 1.8V */
#define GP_TP24			IMX_GPIO_NR(6, 31)
	IOMUX_PAD_CTRL(EIM_BCLK__GPIO6_IO31, WEAK_PULLUP),	/* 1.8V */
#define GP_TP26			IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(EIM_DA15__GPIO3_IO15, WEAK_PULLUP),	/* 1.8V */
#define GP_TP29			IMX_GPIO_NR(1, 19)
	IOMUX_PAD_CTRL(SD1_DAT2__GPIO1_IO19, WEAK_PULLUP),	/* 1.8V */
#define GP_TP30			IMX_GPIO_NR(1, 21)
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLUP),	/* 1.8V */
#define GP_TP92			IMX_GPIO_NR(6, 10)
	IOMUX_PAD_CTRL(NANDF_RB0__GPIO6_IO10, WEAK_PULLUP),	/* 2.6V */
#define GP_TP93			IMX_GPIO_NR(1, 27)
	IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, WEAK_PULLUP),	/* 2.6V */

#ifdef CONFIG_NEW_REV
#define GP_TP25			IMX_GPIO_NR(5, 14)
	IOMUX_PAD_CTRL(DISP0_DAT20__GPIO5_IO14, OUTPUT_40OHM),	/* tri-state */
#define GP_TP27			IMX_GPIO_NR(5, 15)
	IOMUX_PAD_CTRL(DISP0_DAT21__GPIO5_IO15, OUTPUT_40OHM),	/* tri-state */
#define GP_TP28			IMX_GPIO_NR(5, 16)
	IOMUX_PAD_CTRL(DISP0_DAT22__GPIO5_IO16, OUTPUT_40OHM),	/* tri-state */
#define GP_TP107		IMX_GPIO_NR(5, 17)
	IOMUX_PAD_CTRL(DISP0_DAT23__GPIO5_IO17, OUTPUT_40OHM),	/* tri-state */
#else
#define GP_TP25			IMX_GPIO_NR(2, 31)
	IOMUX_PAD_CTRL(EIM_EB3__GPIO2_IO31, OUTPUT_40OHM),	/* tri-state */
#define GP_TP27			IMX_GPIO_NR(3, 28)
	IOMUX_PAD_CTRL(EIM_D28__GPIO3_IO28, OUTPUT_40OHM),	/* tri-state */
#define GP_TP28			IMX_GPIO_NR(3, 30)
	IOMUX_PAD_CTRL(EIM_D30__GPIO3_IO30, OUTPUT_40OHM),	/* tri-state */
#define GP_TP107		IMX_GPIO_NR(4, 13)
	IOMUX_PAD_CTRL(KEY_ROW3__GPIO4_IO13, OUTPUT_40OHM),	/* tri-state */
#endif

	/* UART1 */
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),

	/* UART2 */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART3 for wl1271 */
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D23__UART3_CTS_B, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D31__UART3_RTS_B, UART_PAD_CTRL),

	/* UART5 - gps */
	IOMUX_PAD_CTRL(CSI0_DAT14__UART5_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT15__UART5_RX_DATA, UART_PAD_CTRL),

	/* USBH1 */
#define GP_USBH1_HUB_RESET	IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(DISP0_DAT10__GPIO4_IO31, WEAK_PULLDN_OUTPUT),

	/* USB OTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, WEAK_PULLUP),
	IOMUX_PAD_CTRL(EIM_D21__USB_OTG_OC, WEAK_PULLUP),
#define GP_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, WEAK_PULLDN_OUTPUT),

	/* USDHC2 - TiWi wl1271 pads */
	IOMUX_PAD_CTRL(SD2_CLK__SD2_CLK, USDHC_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_CMD__SD2_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT0__SD2_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__SD2_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__SD2_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__SD2_DATA3, USDHC_PAD_CTRL),
#define GP_WL12XX_WL_IRQ	IMX_GPIO_NR(6, 11)
	IOMUX_PAD_CTRL(NANDF_CS0__GPIO6_IO11, WEAK_PULLDN),
#define GP_WL12XX_WL_ENABLE	IMX_GPIO_NR(6, 15)
	IOMUX_PAD_CTRL(NANDF_CS2__GPIO6_IO15, OUTPUT_40OHM),
#define GP_WL12XX_BT_ENABLE	IMX_GPIO_NR(6, 16)
	IOMUX_PAD_CTRL(NANDF_CS3__GPIO6_IO16, OUTPUT_40OHM),
//	IOMUX_PAD_CTRL(GPIO_8__XTALOSC_REF_CLK_32K, OUTPUT_40OHM),	/* slow clock */

	/* USDHC3 - micro SD card */
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
#define GP_SD3_WP		IMX_GPIO_NR(7, 1)
	IOMUX_PAD_CTRL(SD3_DAT4__GPIO7_IO01, WEAK_PULLUP),
#define GP_SD3_CD		IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP),

	/* USDHC4 - eMMC */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_CLK_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT4__SD4_DATA4, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT5__SD4_DATA5, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT6__SD4_DATA6, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT7__SD4_DATA7, USDHC_PAD_CTRL),
#define GP_EMMC_RESET		IMX_GPIO_NR(2, 6)
	IOMUX_PAD_CTRL(NANDF_D6__GPIO2_IO06, WEAK_PULLUP),

	/* wm5102 */
#define GP_WM5102_LINE_MIC_DET		IMX_GPIO_NR(2, 21)
	IOMUX_PAD_CTRL(EIM_A17__GPIO2_IO21, WEAK_PULLUP),
#define GP_WM5102_IRQ			IMX_GPIO_NR(2, 20)
	IOMUX_PAD_CTRL(EIM_A18__GPIO2_IO20, WEAK_PULLUP),
#define GP_WM5102_RESET			IMX_GPIO_NR(6, 8)
	IOMUX_PAD_CTRL(NANDF_ALE__GPIO6_IO08, WEAK_PULLDN_OUTPUT),
#define GP_WM5102_LDO_EN		IMX_GPIO_NR(6, 7)
	IOMUX_PAD_CTRL(NANDF_CLE__GPIO6_IO07, WEAK_PULLDN_OUTPUT),
	IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, OUTPUT_40OHM),	/* MCLK1 */
//	IOMUX_PAD_CTRL(GPIO_8__XTALOSC_REF_CLK_32K, OUTPUT_40OHM),	/* MCLK2 */
};

/*
 *
 */
static struct i2c_pads_info i2c_pads[] = {
	/* I2C1, rv4162 */
	I2C_PADS_INFO_ENTRY(I2C1, CSI0_DAT9, 5, 27, CSI0_DAT8, 5, 26, I2C_PAD_CTRL),
#ifdef CONFIG_NEW_REV
	I2C_PADS_INFO_ENTRY(I2C2, EIM_EB2, 2, 30, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
#else
	I2C_PADS_INFO_ENTRY(I2C2, EIM_EB2, 2, 30, EIM_D16, 3, 16, I2C_PAD_CTRL),
#endif
	I2C_PADS_INFO_ENTRY(I2C3, GPIO_5, 1, 05, GPIO_16, 7, 11, I2C_PAD_CTRL),
};

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);
//	printf("%s:%p *%p=0x%lx\n", __func__, gd, &gd->ram_size, gd->ram_size);
	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	return 0;
}

int board_ehci_power(int port, int on)
{
	int gp = port ? GP_USBH1_HUB_RESET : GP_USB_OTG_PWR;
	gpio_set_value(gp, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{.esdhc_base = USDHC3_BASE_ADDR, .max_bus_width = 4},
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int gp_cd = GP_SD3_CD;

	if (cfg->esdhc_base != USDHC3_BASE_ADDR)
		return 1;	/* eMMC always present */
	gpio_direction_input(gp_cd);
	return !gpio_get_value(gp_cd);
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

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
	int gp = (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
	return gp;
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

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
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

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif


static unsigned short gpios_out_low[] = {
	GP_CAM_STROBE,
	GP_CAM_TORCH,
	GP_CAM_MIPI_RESET,
	GP_CAM_MIPI_EN,
	GP_DISP_HV_EN,
	GP_DISP_DATA_BUF_EN,
	GP_DISP_LED_RED,
	GP_DISP_LED_GREEN,
	GP_GSM_SIM_RESET,
	GP_GSM_PWR_EN,
	GP_GSM_RESET,
	GP_GSM_ON_OFF,
	GP_GPS_RESET,
	GP_I2C2_HDMI_EN,
	GP_I2C2_CAMERA_EN,
	GP_I2C2_FLASH_EN,
	GP_I2C3_ACCEL_EN,
	GP_I2C3_MAX77818_EN,
	GP_PWM1_BUZZER,
	GP_USBH1_HUB_RESET,
	GP_USB_OTG_PWR,
	GP_WL12XX_WL_ENABLE,
	GP_WL12XX_BT_ENABLE,
	GP_EMMC_RESET,
	GP_WM5102_RESET,
	GP_WM5102_LDO_EN,
};

static unsigned short gpios_out_high[] = {
	GP_MAIN_ON_OFF,
	GP_ECSPI1_NOR_CS,
	GP_ECSPI3_GSM,
	GP_ECSPI5_WM5102,
	GP_REG_5V_EN,
};

static unsigned short gpios_in[] = {
	GP_ACCEL_IRQ,
	GP_DISP_KEY_1,
	GP_DISP_KEY_2,
	GP_DISP_KEY_3,
	GP_DISP_KEY_4,
	GP_DISP_KEY_5,
	GP_DISP_KEY_6,
	GP_DISP_KEY_SHIFT,
	GP_DISP_KEY_SPACE,
	GP_DISP_KEY_CTRL,
	GP_DISP_MISO,
	GP_DISP_MOSI,
	GP_DISP_CLK,
	GP_DISP_STROBE,
	GP_GSM_HOST_WAKE_WWAN,
	GP_GSM_HOST_WAKE,
	GP_GPS_HEARTBEAT,
	GP_GPS_IRQ,
	GP_MAX77818_INOKB,
	GP_MAX77818_INTB,
	GP_MAX77818_WCINOKB,
	GP_ON_OFF_IRQ,
	GP_PMIC_C0_OUT,
	GP_PMIC_C1_OUT,
	GP_PMIC_C2_OUT,
	GP_PMIC_C3_OUT,
	GP_PMIC_C4_OUT,
	GP_PMIC_C5_OUT,
	GP_PMIC_R0_IN,
	GP_PMIC_R1_IN,
	GP_PMIC_R2_IN,
	GP_PMIC_R3_IN,
	GP_PMIC_R4_IN,
	GP_PMIC_R5_IN,
	GP_PMIC_KEYPAD_LOCK,
	GP_RTC_RV4162_IRQ,
	GP_TP6,
	GP_TP7,
	GP_TP24,
	GP_TP26,
	GP_TP29,
	GP_TP30,
	GP_TP92,
	GP_TP93,
	GP_TP25,
	GP_TP27,
	GP_TP28,
	GP_TP107,
	GP_WL12XX_WL_IRQ,
	GP_SD3_WP,
	GP_SD3_CD,
	GP_WM5102_LINE_MIC_DET,
	GP_WM5102_IRQ,
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

	SETUP_IOMUX_PADS(ls_pads);
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
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	for (i = 0; i < 3; i++) {
		setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f, p);
		p += I2C_PADS_INFO_ENTRY_SPACING;
	}
#if 1
	{
		u8 orig_i2c_bus;
		u8 val8;

		orig_i2c_bus = i2c_get_bus_num();
		i2c_set_bus_num(2);

		gpio_set_value(GP_I2C3_MAX77818_EN, 1);
		val8 = 0x7f;	/* 4.0A source */
		i2c_write(0x69, 0xc0, 1, &val8, 1);
		val8 = 0x0c;	/* Protection allow 0xb9 write */
		i2c_write(0x69, 0xbd, 1, &val8, 1);
		val8 = 0x14;	/* 1A charge */
		i2c_write(0x69, 0xb9, 1, &val8, 1);
		gpio_set_value(GP_I2C3_MAX77818_EN, 0);

		i2c_set_bus_num(orig_i2c_bus);
	}
#endif

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: LS\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"b1",		GP_TP6,	'1'},
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
	{"mmc1",	MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},	/* 8-bit eMMC */
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
	setenv("board","ls");
	return 0;
}
