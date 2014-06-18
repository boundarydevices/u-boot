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
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/fbpanel.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
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
#include <usb/ehci-ci.h>

DECLARE_GLOBAL_DATA_PTR;

#define PADCFG_INPUT		(PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define PADCFG_INPUT_L		(PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define PADCFG_INPUT_DN		(PADCFG_INPUT | PAD_CTL_PUS_100K_DOWN)
#define PADCFG_INPUT_UP		(PADCFG_INPUT | PAD_CTL_PUS_100K_UP)
#define PADCFG_INPUT_L_UP	(PADCFG_INPUT_L | PAD_CTL_PUS_100K_UP)

#define AUD_PAD_CTRL	(PADCFG_INPUT_L_UP | PAD_CTL_SRE_FAST)
#define CSI_PAD_CTRL	(PADCFG_INPUT_UP | PAD_CTL_SRE_FAST)
#define UART_PAD_CTRL	(PADCFG_INPUT_UP | PAD_CTL_SRE_FAST)

#define PADCFG_FLOAT_IRQ (PADCFG_INPUT)

#define USDHC_PAD_CTRL_50MHZ (PAD_CTL_PUS_47K_UP |		\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define OTGID_PAD_CTRL		USDHC_PAD_CTRL_50MHZ
#define HDMICEC_PAD_CTRL	(PADCFG_INPUT_UP | PAD_CTL_ODE | PAD_CTL_SRE_SLOW)
#define ENET_PAD_CTRL		PADCFG_INPUT_UP

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
		PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)
#define OUTPUT_40OHM_UP	(OUTPUT_40OHM | PAD_CTL_PUS_100K_UP)
#define OUTPUT_40OHM_DN	(OUTPUT_40OHM | PAD_CTL_PUS_100K_DOWN)
#define SPDIF_PAD_CTRL	WEAK_PULLUP

/*
 *
 */
static const iomux_v3_cfg_t init_pads[] = {
	/* Audio - GS2971, WM5102 */
	IOMUX_PAD_CTRL(DISP0_DAT23__AUD4_RXD, AUD_PAD_CTRL),	/* pin J3 - AOUT1/2 */
	IOMUX_PAD_CTRL(SD2_CMD__AUD4_RXC, AUD_PAD_CTRL),		/* pin J4 - ACLK*/
	IOMUX_PAD_CTRL(DISP0_DAT18__AUD4_RXFS, AUD_PAD_CTRL),	/* pin H4 - WCLK*/

	/* Audio - TC3587 mipi hdmi input */
	IOMUX_PAD_CTRL(DISP0_DAT13__AUD5_RXFS, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT14__AUD5_RXC, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT19__AUD5_RXD, AUD_PAD_CTRL),

	/* Audio - WM5102 */
	IOMUX_PAD_CTRL(DI0_PIN2__AUD6_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN3__AUD6_TXFS, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN4__AUD6_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__AUD6_TXC, AUD_PAD_CTRL),

	/* WM5102 */
//	IOMUX_PAD_CTRL(SD1_CLK__OSC32K_32K_OUT, OUTPUT_40OHM),	/* MCLK2 */
	IOMUX_PAD_CTRL(NANDF_CS2__CCM_CLKO2, OUTPUT_40OHM),	/* MCLK1 */
#define GP_WM5102_RESET		IMX_GPIO_NR(5, 9)
	IOMUX_PAD_CTRL(DISP0_DAT15__GPIO5_IO09, OUTPUT_40OHM),
#define GPIRQ_WM5102		IMX_GPIO_NR(5, 10)
	IOMUX_PAD_CTRL(DISP0_DAT16__GPIO5_IO10, WEAK_PULLUP),
#define GP_WM5102_LDO_EN	IMX_GPIO_NR(5, 11)
	IOMUX_PAD_CTRL(DISP0_DAT17__GPIO5_IO11, OUTPUT_40OHM),

	/* camera - video0 - ADV7180 - I2C3, crystal 28.636 MHz */
	IOMUX_PAD_CTRL(CSI0_DAT12__IPU1_CSI0_DATA12, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT13__IPU1_CSI0_DATA13, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT14__IPU1_CSI0_DATA14, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT15__IPU1_CSI0_DATA15, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT16__IPU1_CSI0_DATA16, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT17__IPU1_CSI0_DATA17, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT18__IPU1_CSI0_DATA18, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT19__IPU1_CSI0_DATA19, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_PIXCLK__IPU1_CSI0_PIXCLK, CSI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_MCLK__GPIO5_IO19, WEAK_PULLUP),	/* Hsync */
	IOMUX_PAD_CTRL(CSI0_VSYNC__GPIO5_IO21, WEAK_PULLUP),	/* Vsync */
#define GP_ADV7180_RESET	IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__GPIO4_IO15, OUTPUT_40OHM),
#define GPIRQ_ADV7180		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLUP),

	/* camera - video1 - TC3587 mipi hdmi input */
#define GP_TC3587_VIDEO_DETECT	IMX_GPIO_NR(3, 23)
	IOMUX_PAD_CTRL(EIM_D23__GPIO3_IO23, WEAK_PULLUP),	/* J13 Pin 1 - Video detect */
#define GP_TC3587_HPD_IN	IMX_GPIO_NR(3, 29)
	IOMUX_PAD_CTRL(EIM_D29__GPIO3_IO29, WEAK_PULLUP),		/* J13, pin 20 */
#define GP_TC3587_RESET		IMX_GPIO_NR(6, 11)
	IOMUX_PAD_CTRL(NANDF_CS0__GPIO6_IO11, OUTPUT_40OHM),
#define GPIRQ_TC3587		IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(NANDF_CS1__GPIO6_IO14, PADCFG_FLOAT_IRQ),

	/* camera - video2 - GS2971(SDI) on CSI1 */
#if defined(CONFIG_MX6S) || defined(CONFIG_MX6DL)
	/* Dualite/Solo doesn't have IPU2 */
	IOMUX_PAD_CTRL(EIM_A24__IPU1_CSI1_DATA19, CSI_PAD_CTRL),	/* GPIO2[30] */
	IOMUX_PAD_CTRL(EIM_A23__IPU1_CSI1_DATA18, CSI_PAD_CTRL),	/* GPIO6[6] */
	IOMUX_PAD_CTRL(EIM_A22__IPU1_CSI1_DATA17, CSI_PAD_CTRL),	/* GPIO2[16] */
	IOMUX_PAD_CTRL(EIM_A21__IPU1_CSI1_DATA16, CSI_PAD_CTRL),	/* GPIO2[17] */
	IOMUX_PAD_CTRL(EIM_A20__IPU1_CSI1_DATA15, CSI_PAD_CTRL),	/* GPIO2[18] */
	IOMUX_PAD_CTRL(EIM_A19__IPU1_CSI1_DATA14, CSI_PAD_CTRL),	/* GPIO2[19] */
	IOMUX_PAD_CTRL(EIM_A18__IPU1_CSI1_DATA13, CSI_PAD_CTRL),	/* GPIO2[20] */
	IOMUX_PAD_CTRL(EIM_A17__IPU1_CSI1_DATA12, CSI_PAD_CTRL),	/* GPIO2[21] */
	IOMUX_PAD_CTRL(EIM_EB0__IPU1_CSI1_DATA11, CSI_PAD_CTRL),	/* GPIO2[28] */
	IOMUX_PAD_CTRL(EIM_EB1__IPU1_CSI1_DATA10, CSI_PAD_CTRL),	/* GPIO2[29] */
	IOMUX_PAD_CTRL(EIM_DA0__IPU1_CSI1_DATA09, CSI_PAD_CTRL),	/* GPIO3[0] */
	IOMUX_PAD_CTRL(EIM_DA1__IPU1_CSI1_DATA08, CSI_PAD_CTRL),	/* GPIO3[1] */
	IOMUX_PAD_CTRL(EIM_DA2__IPU1_CSI1_DATA07, CSI_PAD_CTRL),	/* GPIO3[2] */
	IOMUX_PAD_CTRL(EIM_DA3__IPU1_CSI1_DATA06, CSI_PAD_CTRL),	/* GPIO3[3] */
	IOMUX_PAD_CTRL(EIM_DA4__IPU1_CSI1_DATA05, CSI_PAD_CTRL),	/* GPIO3[4] */
	IOMUX_PAD_CTRL(EIM_DA5__IPU1_CSI1_DATA04, CSI_PAD_CTRL),	/* GPIO3[5] */
	IOMUX_PAD_CTRL(EIM_DA6__IPU1_CSI1_DATA03, CSI_PAD_CTRL),	/* GPIO3[6] */
	IOMUX_PAD_CTRL(EIM_DA7__IPU1_CSI1_DATA02, CSI_PAD_CTRL),	/* GPIO3[7] */
	IOMUX_PAD_CTRL(EIM_DA8__IPU1_CSI1_DATA01, CSI_PAD_CTRL),	/* GPIO3[8] */
	IOMUX_PAD_CTRL(EIM_DA9__IPU1_CSI1_DATA00, CSI_PAD_CTRL),	/* GPIO3[9] */
	IOMUX_PAD_CTRL(EIM_A16__IPU1_CSI1_PIXCLK, CSI_PAD_CTRL),	/* GPIO2[22] */
#else
	IOMUX_PAD_CTRL(EIM_A24__IPU2_CSI1_DATA19, CSI_PAD_CTRL),	/* GPIO2[30] */
	IOMUX_PAD_CTRL(EIM_A23__IPU2_CSI1_DATA18, CSI_PAD_CTRL),	/* GPIO6[6] */
	IOMUX_PAD_CTRL(EIM_A22__IPU2_CSI1_DATA17, CSI_PAD_CTRL),	/* GPIO2[16] */
	IOMUX_PAD_CTRL(EIM_A21__IPU2_CSI1_DATA16, CSI_PAD_CTRL),	/* GPIO2[17] */
	IOMUX_PAD_CTRL(EIM_A20__IPU2_CSI1_DATA15, CSI_PAD_CTRL),	/* GPIO2[18] */
	IOMUX_PAD_CTRL(EIM_A19__IPU2_CSI1_DATA14, CSI_PAD_CTRL),	/* GPIO2[19] */
	IOMUX_PAD_CTRL(EIM_A18__IPU2_CSI1_DATA13, CSI_PAD_CTRL),	/* GPIO2[20] */
	IOMUX_PAD_CTRL(EIM_A17__IPU2_CSI1_DATA12, CSI_PAD_CTRL),	/* GPIO2[21] */
	IOMUX_PAD_CTRL(EIM_EB0__IPU2_CSI1_DATA11, CSI_PAD_CTRL),	/* GPIO2[28] */
	IOMUX_PAD_CTRL(EIM_EB1__IPU2_CSI1_DATA10, CSI_PAD_CTRL),	/* GPIO2[29] */
	IOMUX_PAD_CTRL(EIM_DA0__IPU2_CSI1_DATA09, CSI_PAD_CTRL),	/* GPIO3[0] */
	IOMUX_PAD_CTRL(EIM_DA1__IPU2_CSI1_DATA08, CSI_PAD_CTRL),	/* GPIO3[1] */
	IOMUX_PAD_CTRL(EIM_DA2__IPU2_CSI1_DATA07, CSI_PAD_CTRL),	/* GPIO3[2] */
	IOMUX_PAD_CTRL(EIM_DA3__IPU2_CSI1_DATA06, CSI_PAD_CTRL),	/* GPIO3[3] */
	IOMUX_PAD_CTRL(EIM_DA4__IPU2_CSI1_DATA05, CSI_PAD_CTRL),	/* GPIO3[4] */
	IOMUX_PAD_CTRL(EIM_DA5__IPU2_CSI1_DATA04, CSI_PAD_CTRL),	/* GPIO3[5] */
	IOMUX_PAD_CTRL(EIM_DA6__IPU2_CSI1_DATA03, CSI_PAD_CTRL),	/* GPIO3[6] */
	IOMUX_PAD_CTRL(EIM_DA7__IPU2_CSI1_DATA02, CSI_PAD_CTRL),	/* GPIO3[7] */
	IOMUX_PAD_CTRL(EIM_DA8__IPU2_CSI1_DATA01, CSI_PAD_CTRL),	/* GPIO3[8] */
	IOMUX_PAD_CTRL(EIM_DA9__IPU2_CSI1_DATA00, CSI_PAD_CTRL),	/* GPIO3[9] */
	IOMUX_PAD_CTRL(EIM_A16__IPU2_CSI1_PIXCLK, CSI_PAD_CTRL),	/* GPIO2[22] - pin A8 */
#endif
	/* Not used, but MUST be in GPIO mode */
	IOMUX_PAD_CTRL(EIM_DA10__GPIO3_IO10, WEAK_PULLUP),	/* CSI1_DATA_EN not used (pin B5 stat2) */
	IOMUX_PAD_CTRL(EIM_DA11__GPIO3_IO11, WEAK_PULLUP),	/* pin A5 stat0, hsync */
	IOMUX_PAD_CTRL(EIM_DA12__GPIO3_IO12, WEAK_PULLUP),	/* pin A6 stat1, vsync */

#define GP_GS2971_SMPTE_BYPASS	IMX_GPIO_NR(2, 27)
	IOMUX_PAD_CTRL(EIM_LBA__GPIO2_IO27, WEAK_PULLUP),		/* pin G7 - i/o SMPTE bypass */
#define GP_GS2971_RESET		IMX_GPIO_NR(3, 13)
	IOMUX_PAD_CTRL(EIM_DA13__GPIO3_IO13, OUTPUT_40OHM),	/* 0 - pin C7 - reset */
#define GP_GS2971_DVI_LOCK	IMX_GPIO_NR(3, 14)
	IOMUX_PAD_CTRL(EIM_DA14__GPIO3_IO14, WEAK_PULLUP),		/* pin B6 - stat3 - DVI_LOCK */
#define GP_GS2971_DATA_ERR	IMX_GPIO_NR(3, 15)
	IOMUX_PAD_CTRL(EIM_DA15__GPIO3_IO15, WEAK_PULLUP),		/* pin C6 - stat5 - DATA error */
#define GP_GS2971_LB_CONT	IMX_GPIO_NR(3, 20)
	IOMUX_PAD_CTRL(EIM_D20__GPIO3_IO20, WEAK_PULLUP),		/* pin A3 - LB control - float, analog input */
#define GP_GS2971_Y_1ANC	IMX_GPIO_NR(4, 26)
	IOMUX_PAD_CTRL(DISP0_DAT5__GPIO4_IO26, WEAK_PULLUP),	/* pin C5 - stat4 - 1ANC - Y signal detect */
#define GP_GS2971_RC_BYPASS	IMX_GPIO_NR(4, 27)
	IOMUX_PAD_CTRL(DISP0_DAT6__GPIO4_IO27, OUTPUT_40OHM),	/* 0 - pin G3 - RC bypass - output is buffered(low) */
#define GP_GS2971_IOPROC_EN	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(DISP0_DAT7__GPIO4_IO28, OUTPUT_40OHM),	/* 0 - pin H8 - io(A/V) processor enable */
#define GP_GS2971_AUDIO_EN	IMX_GPIO_NR(4, 29)
	IOMUX_PAD_CTRL(DISP0_DAT8__GPIO4_IO29, OUTPUT_40OHM),	/* 0 - pin H3 - Audio Enable */
#define GP_GS2971_TIM_861	IMX_GPIO_NR(4, 30)
	IOMUX_PAD_CTRL(DISP0_DAT9__GPIO4_IO30, OUTPUT_40OHM),	/* 0 - pin H5 - TIM861 timing format, 1-use HSYNC/VSYNC */
#define GP_GS2971_SW_EN		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(DISP0_DAT10__GPIO4_IO31, OUTPUT_40OHM),	/* 0 - pin D7 - SW_EN - line lock enable */
#define GP_GS2971_STANDBY	IMX_GPIO_NR(5, 0)
	IOMUX_PAD_CTRL(EIM_WAIT__GPIO5_IO00, OUTPUT_40OHM),		/* 1 - pin K2 - Standby */
#define GP_GS2971_DVB_ASI	IMX_GPIO_NR(5, 5)
	IOMUX_PAD_CTRL(DISP0_DAT11__GPIO5_IO05, WEAK_PULLUP),	/* pin G8 i/o DVB_ASI */

	/* ECSPI1 */
	IOMUX_PAD_CTRL(EIM_D17__ECSPI1_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D18__ECSPI1_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D16__ECSPI1_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI1_NOR_CS	IMX_GPIO_NR(3, 19)
	IOMUX_PAD_CTRL(EIM_D19__GPIO3_IO19, WEAK_PULLUP),

	/* ECSPI2 - GS2971 */
	IOMUX_PAD_CTRL(EIM_OE__ECSPI2_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_CS1__ECSPI2_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_CS0__ECSPI2_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI2_GS2971_CS	IMX_GPIO_NR(2, 26)
	IOMUX_PAD_CTRL(EIM_RW__GPIO2_IO26, OUTPUT_40OHM_UP),

	/* ECSPI3 - WM5102 */
	IOMUX_PAD_CTRL(DISP0_DAT2__ECSPI3_MISO, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT1__ECSPI3_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT0__ECSPI3_SCLK, SPI_PAD_CTRL),
#define GP_ECSPI3_WM5102_CS	IMX_GPIO_NR(4, 25)
	IOMUX_PAD_CTRL(DISP0_DAT4__GPIO4_IO25, OUTPUT_40OHM_UP),

	/* ENET pads that don't change for PHY reset */
	IOMUX_PAD_CTRL(ENET_MDIO__ENET_MDIO, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(ENET_MDC__ENET_MDC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TXC__RGMII_TXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD0__RGMII_TD0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD1__RGMII_TD1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD2__RGMII_TD2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TD3__RGMII_TD3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_TX_CTL__RGMII_TX_CTL, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(ENET_REF_CLK__ENET_TX_CLK, ENET_PAD_CTRL),
	/* pin 42 PHY nRST */
#define GP_ENET_PHY_RESET	IMX_GPIO_NR(1, 27)
	IOMUX_PAD_CTRL(ENET_RXD0__GPIO1_IO27, OUTPUT_40OHM),	/* Micrel RGMII Phy Reset */
#define GP_ENET_PHY_INT		IMX_GPIO_NR(1, 28)
	IOMUX_PAD_CTRL(ENET_TX_EN__GPIO1_IO28, WEAK_PULLUP),	/* Micrel RGMII Phy Interrupt */

	/* HDMI */
	IOMUX_PAD_CTRL(KEY_ROW2__HDMI_TX_CEC_LINE, HDMICEC_PAD_CTRL),

#define GP_TP101_ALERT		IMX_GPIO_NR(4, 5)
	IOMUX_PAD_CTRL(GPIO_19__GPIO4_IO05, WEAK_PULLUP),

	/* Hogs */
#define GP_ANX7738_RESET	IMX_GPIO_NR(2, 0)
	IOMUX_PAD_CTRL(NANDF_D0__GPIO2_IO00, OUTPUT_40OHM_DN),

	/* i2c1 mux enables */
#define GP_I2C1_MUX_KL04	IMX_GPIO_NR(5, 27)
	IOMUX_PAD_CTRL(CSI0_DAT9__GPIO5_IO27, OUTPUT_40OHM_DN),
#define GP_I2C1_MUX_ANX7814	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(SD2_CLK__GPIO1_IO10, OUTPUT_40OHM_DN),
#define GP_I2C1_MUX_AX7738	IMX_GPIO_NR(2, 1)
	IOMUX_PAD_CTRL(NANDF_D1__GPIO2_IO01, OUTPUT_40OHM_DN),
#define GP_I2C1_MUX_TC3587	IMX_GPIO_NR(6, 16)
	IOMUX_PAD_CTRL(NANDF_CS3__GPIO6_IO16, OUTPUT_40OHM_DN),

	/* i2c1 anx7814 - hdmi to mydp transmitter */
#define GP_ANX7814_P_DWN       IMX_GPIO_NR(1, 13)
	IOMUX_PAD_CTRL(SD2_DAT2__GPIO1_IO13, OUTPUT_40OHM_UP),
#define GP_ANX7814_RESET       IMX_GPIO_NR(2, 5)
	IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, OUTPUT_40OHM_DN),
#define GP_ANX7814_CBL_DET     IMX_GPIO_NR(2, 3)
	IOMUX_PAD_CTRL(NANDF_D3__GPIO2_IO03, WEAK_PULLDN),
#define GP_ANX7814_HDMI_INTR   IMX_GPIO_NR(1, 14)
	IOMUX_PAD_CTRL(SD2_DAT1__GPIO1_IO14, WEAK_PULLDN),

	/* i2c2 mux enables */
#define GP_I2C2_MUX_ANX7814_DDC	 IMX_GPIO_NR(1, 15)
	IOMUX_PAD_CTRL(SD2_DAT0__GPIO1_IO15, OUTPUT_40OHM_DN),

	/* PCIe */
#define GP_PCIE_RESET		IMX_GPIO_NR(5, 2)
	IOMUX_PAD_CTRL(EIM_A25__GPIO5_IO02, OUTPUT_40OHM),
#define GP_PCIE_RADIO_ON	IMX_GPIO_NR(6, 10)
	IOMUX_PAD_CTRL(NANDF_RB0__GPIO6_IO10, OUTPUT_40OHM),

	/* UART1 - J2 - PTT connector */
	IOMUX_PAD_CTRL(SD3_DAT7__UART1_TX_DATA, UART_PAD_CTRL),	/* J2, pin 9 */
	IOMUX_PAD_CTRL(SD3_DAT6__UART1_RX_DATA, UART_PAD_CTRL),	/* J2, pin 10 */
	IOMUX_PAD_CTRL(EIM_EB2__GPIO2_IO30, WEAK_PULLUP),		/* J2, pin 7: PTT_R */
	IOMUX_PAD_CTRL(EIM_EB3__GPIO2_IO31, WEAK_PULLUP),		/* J2, pin 6: PTT_L */

	/* UART2 - debug */
	IOMUX_PAD_CTRL(EIM_D26__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D27__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART3 - relay J3 */
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),	/* J3, pin 5 */
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),	/* J3, pin 6 */
#define GP_RELAY_DETECT		IMX_GPIO_NR(7, 2)
	IOMUX_PAD_CTRL(SD3_CMD__GPIO7_IO02, PADCFG_FLOAT_IRQ),	/* J3 pin 7 */
#define GP_J3_CARRIER_SENSE	IMX_GPIO_NR(7, 4)
	IOMUX_PAD_CTRL(SD3_DAT0__GPIO7_IO04, PADCFG_FLOAT_IRQ),


#define GP_PTT_ON	IMX_GPIO_NR(7, 5)
	IOMUX_PAD_CTRL(SD3_DAT1__GPIO7_IO05, PADCFG_FLOAT_IRQ),

#define GP_J11_HDMI_POWER_EN	IMX_GPIO_NR(7, 3)
	IOMUX_PAD_CTRL(SD3_CLK__GPIO7_IO03, PADCFG_INPUT_L_UP),	/* power on J11 pin 3 */

	/* UART4 - GPS */
	IOMUX_PAD_CTRL(KEY_COL0__UART4_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_ROW0__UART4_RX_DATA, UART_PAD_CTRL),
#define GP_GPS_RESET	IMX_GPIO_NR(7, 1)
	IOMUX_PAD_CTRL(SD3_DAT4__GPIO7_IO01, PADCFG_FLOAT_IRQ),
#define GPIRQ_GPS	IMX_GPIO_NR(7, 0)
	IOMUX_PAD_CTRL(SD3_DAT5__GPIO7_IO00, WEAK_PULLUP),
#define GP_GPS_HEARTBEAT IMX_GPIO_NR(4, 10)
	IOMUX_PAD_CTRL(KEY_COL2__GPIO4_IO10, WEAK_PULLUP),

	/* UART5 - J6 data connector */
	IOMUX_PAD_CTRL(KEY_COL1__UART5_TX_DATA, UART_PAD_CTRL),	/* J6, pin 5 */
	IOMUX_PAD_CTRL(KEY_ROW1__UART5_RX_DATA, UART_PAD_CTRL),	/* J6, pin 6 */
	IOMUX_PAD_CTRL(SD1_CMD__GPIO1_IO18, WEAK_PULLUP),		/* J6, pin 8 - Data detect */
#define GP_J6_POWER_EN	IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(SD1_DAT0__GPIO1_IO16, WEAK_PULLUP),		/* J6, Power enable */
	IOMUX_PAD_CTRL(GPIO_4__GPIO1_IO04, WEAK_PULLUP),		/* J6, pin 16 */

	/* USBH1 */
	IOMUX_PAD_CTRL(EIM_D30__USB_H1_OC, WEAK_PULLUP),	/* low indicates over current */
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	IOMUX_PAD_CTRL(GPIO_17__GPIO7_IO12, OUTPUT_40OHM),	/* USB Hub Reset for USB2512 4 port hub */

	/* USBOTG */
	IOMUX_PAD_CTRL(GPIO_1__USB_OTG_ID, OTGID_PAD_CTRL),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),	/* low indicates over current */
#define GP_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(EIM_D22__GPIO3_IO22, OUTPUT_40OHM),

	/* USDHC4 - eMMC */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT4__SD4_DATA4, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT5__SD4_DATA5, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT6__SD4_DATA6, USDHC_PAD_CTRL_50MHZ),
	IOMUX_PAD_CTRL(SD4_DAT7__SD4_DATA7, USDHC_PAD_CTRL_50MHZ),
#define GP_EMMC_RESET	IMX_GPIO_NR(6, 7)
	IOMUX_PAD_CTRL(NANDF_CLE__GPIO6_IO07, OUTPUT_40OHM),	/* eMMC reset */

	/* 1-wire J11 pin 22, J3 pin 8, J6 pin 9*/
#define GP_1WIRE_EN	IMX_GPIO_NR(1, 21)
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, WEAK_PULLUP),

	/* Microcontroller KL04Z32TFK4 on I2C1 */
#define GP_KL04_SWD_CLK	IMX_GPIO_NR(1, 0)
	IOMUX_PAD_CTRL(GPIO_0__GPIO1_IO00, WEAK_PULLDN),	/* SWD_CLK */
#define GP_KL04_SWD_IO	IMX_GPIO_NR(1, 2)
	IOMUX_PAD_CTRL(GPIO_2__GPIO1_IO02, WEAK_PULLUP),	/* SWD_IO */
#define GP_KL04_RESET	IMX_GPIO_NR(5, 20)
	IOMUX_PAD_CTRL(CSI0_DATA_EN__GPIO5_IO20, WEAK_PULLDN),
#define GP_KL04_PROGRAM	IMX_GPIO_NR(5, 22)
	IOMUX_PAD_CTRL(CSI0_DAT4__GPIO5_IO22, WEAK_PULLDN),
#define GPIRQ_KL04	IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, WEAK_PULLUP),

	/* Make sure these no-connects don't wiggle */
	IOMUX_PAD_CTRL(DI0_DISP_CLK__GPIO4_IO16, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT12__GPIO5_IO06, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT20__GPIO5_IO14, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT21__GPIO5_IO15, WEAK_PULLUP),
	IOMUX_PAD_CTRL(DISP0_DAT22__GPIO5_IO16, WEAK_PULLUP),
};

static const iomux_v3_cfg_t enet_pads1[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
#define GP_PHY_AD2		IMX_GPIO_NR(6, 30)
	IOMUX_PAD_CTRL(RGMII_RXC__GPIO6_IO30, OUTPUT_40OHM),
	/* pin 32 - 1 - (MODE0) all */
#define GP_PHY_MODE0		IMX_GPIO_NR(6, 25)
	IOMUX_PAD_CTRL(RGMII_RD0__GPIO6_IO25, OUTPUT_40OHM),
	/* pin 31 - 1 - (MODE1) all */
#define GP_PHY_MODE1		IMX_GPIO_NR(6, 27)
	IOMUX_PAD_CTRL(RGMII_RD1__GPIO6_IO27, OUTPUT_40OHM),
	/* pin 28 - 1 - (MODE2) all */
#define GP_PHY_MODE2		IMX_GPIO_NR(6, 28)
	IOMUX_PAD_CTRL(RGMII_RD2__GPIO6_IO28, OUTPUT_40OHM),
	/* pin 27 - 1 - (MODE3) all */
#define GP_PHY_MODE3		IMX_GPIO_NR(6, 29)
	IOMUX_PAD_CTRL(RGMII_RD3__GPIO6_IO29, OUTPUT_40OHM),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
#define GP_PHY_CLK125		IMX_GPIO_NR(6, 24)
	IOMUX_PAD_CTRL(RGMII_RX_CTL__GPIO6_IO24, OUTPUT_40OHM),
};

static const iomux_v3_cfg_t enet_pads2[] = {
	IOMUX_PAD_CTRL(RGMII_RXC__RGMII_RXC, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD0__RGMII_RD0, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD1__RGMII_RD1, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD2__RGMII_RD2, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RD3__RGMII_RD3, ENET_PAD_CTRL),
	IOMUX_PAD_CTRL(RGMII_RX_CTL__RGMII_RX_CTL, ENET_PAD_CTRL),
};

/*
 *
 */
static struct i2c_pads_info i2c_pads[] = {
	/* I2C1, SGTL5000 */
	I2C_PADS_INFO_ENTRY(I2C1, EIM_D21, 3, 21, EIM_D28, 3, 28, I2C_PAD_CTRL),
	/* I2C2 Camera, MIPI */
	I2C_PADS_INFO_ENTRY(I2C2, KEY_COL3, 4, 12, KEY_ROW3, 4, 13, I2C_PAD_CTRL),
	/* I2C3, J15 - RGB connector */
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
	/* Reset USB hub */
	if (port) {
		gpio_set_value(GP_USB_HUB_RESET, 0);
		mdelay(2);
		gpio_set_value(GP_USB_HUB_RESET, 1);
	}
	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_USB_OTG_PWR, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[] = {
	{.esdhc_base = USDHC4_BASE_ADDR, .max_bus_width = 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;	/* eMMC always present */
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
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
#endif

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? GP_ECSPI1_NOR_CS : -1;
}

int board_phy_config(struct phy_device *phydev)
{
	/* min rx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
	/* min tx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
	/* max rx/tx clock delay, min rx/tx control */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static void setup_iomux_enet(void)
{
	gpio_direction_output(GP_ENET_PHY_RESET, 0);
	gpio_direction_output(GP_PHY_AD2, 1);
	gpio_direction_output(GP_PHY_MODE0, 1);
	gpio_direction_output(GP_PHY_MODE1, 1);
	gpio_direction_output(GP_PHY_MODE2, 1);
	gpio_direction_output(GP_PHY_MODE3, 1);
	gpio_direction_output(GP_PHY_CLK125, 1);
	SETUP_IOMUX_PADS(enet_pads1);

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(GP_ENET_PHY_RESET, 1);

	SETUP_IOMUX_PADS(enet_pads2);
	udelay(100);	/* Wait 100 us before using mii interface */
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}
#endif

#ifdef CONFIG_CI_UDC
	/* For otg ethernet*/
	if (!getenv("eth1addr"))
		setenv("eth1addr", getenv("usbnet_devaddr"));
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

static const struct display_info_t displays[] = {
	/* hdmi */
	VD_1280_720M_60(HDMI, fbp_detect_i2c, 1, 0x50),
	VD_1920_1080M_60(HDMI, NULL, 1, 0x50),
	VD_1024_768M_60(HDMI, NULL, 1, 0x50),
};

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}
#endif

static const unsigned short gpios_out_low[] = {
	GP_KL04_RESET,		/* inverted before kl04, not in reset */
	GP_KL04_PROGRAM,	/* inverted before kl04 */
	GP_EMMC_RESET,		/* hold in reset */
	GP_WM5102_RESET,
	GP_WM5102_LDO_EN,
	GP_USB_OTG_PWR,		/* disable USB otg power */
	GP_USB_HUB_RESET,	/* disable hub */
	GP_J6_POWER_EN,
	GP_ENET_PHY_RESET,
	GP_PCIE_RADIO_ON,
	GP_PCIE_RESET,
	GP_GS2971_RESET,
	GP_GS2971_RC_BYPASS,
	GP_GS2971_IOPROC_EN,
	GP_GS2971_AUDIO_EN,
	GP_GS2971_TIM_861,
	GP_GS2971_SW_EN,
	GP_GS2971_DVB_ASI,
	GP_ANX7738_RESET,
	GP_TC3587_RESET,
	GP_I2C1_MUX_KL04,
	GP_I2C1_MUX_ANX7814,
	GP_I2C1_MUX_AX7738,
	GP_I2C1_MUX_TC3587,
	GP_ANX7814_RESET,
	GP_I2C2_MUX_ANX7814_DDC,
};

static const unsigned short gpios_out_high[] = {
	GP_ECSPI1_NOR_CS,
	GP_ECSPI2_GS2971_CS,
	GP_ECSPI3_WM5102_CS,
	GP_GS2971_STANDBY,
	GP_ANX7814_P_DWN,
};

static const unsigned short gpios_in[] = {
	GP_KL04_SWD_CLK,
	GP_KL04_SWD_IO,
	GP_TP101_ALERT,
	GP_GS2971_SMPTE_BYPASS,
	GP_GS2971_DVI_LOCK,
	GP_GS2971_DATA_ERR,
	GP_GS2971_LB_CONT,
	GP_GS2971_Y_1ANC,
	GPIRQ_KL04,
	GPIRQ_WM5102,
	GPIRQ_TC3587,
	GP_ANX7814_CBL_DET,
	GP_ANX7814_HDMI_INTR,
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

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
	return 0;
}


int checkboard(void)
{
	puts("Board: PER\n");
	return 0;
}

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
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

int board_late_init(void)
{
	int cpurev = get_cpu_rev();

	setenv("cpu", get_imx_type((cpurev & 0xFF000) >> 12));
	if (!getenv("board"))
		setenv("board", "per");
	setenv("uboot_defconfig", CONFIG_DEFCONFIG);
	return 0;
}
