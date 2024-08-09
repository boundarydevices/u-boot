// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Ezurio
 */

#include <common.h>
#include <efi_loader.h>
#include <env.h>
#include <init.h>
#include <malloc.h>
#include <errno.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <linux/bitops.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/pfuze.h"
#include <usb.h>
#include <dwc3-uboot.h>

DECLARE_GLOBAL_DATA_PTR;

#define PAD_CTRL_ENET_MDC        (PAD_CTL_DSE3)
#define PAD_CTRL_ENET_MDIO       (PAD_CTL_DSE3 | PAD_CTL_ODE)
#define PAD_CTRL_ENET_TX 0x1f

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE)

#define WEAK_PULLUP     ( \
        PAD_CTL_HYS \
        )

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

static iomux_v3_cfg_t const init_pads[] = {
#define GP_I2C4_SN65DSI83_IRQ		IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
#define GP_LCD133_070_RESET		IMX_GPIO_NR(1, 1)
#define GP_TCXD070_BKL_EN		IMX_GPIO_NR(1, 1)
	IMX8MQ_PAD_GPIO1_IO01__GPIO1_IO1 | MUX_PAD_CTRL(0x16),

#define GPIRQ_TS_GT911 			IMX_GPIO_NR(3, 12)
#define GPIRQ_LCD133_TOUCH		IMX_GPIO_NR(3, 12)
	IMX8MQ_PAD_NAND_DATA06__GPIO3_IO12 | MUX_PAD_CTRL(0xc3),
#define GP_TS_GT911_RESET		IMX_GPIO_NR(3, 13)
#define GP_ST1633_RESET			IMX_GPIO_NR(3, 13)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(3, 13)
#define GP_TS_FT7250_RESET		IMX_GPIO_NR(3, 13)
#define GP_TS_ILI251X_RESET		IMX_GPIO_NR(3, 13)
	IMX8MQ_PAD_NAND_DATA07__GPIO3_IO13 | MUX_PAD_CTRL(0x43),

#define GP_ARM_DRAM_VSEL		IMX_GPIO_NR(3, 24)
	IMX8MQ_PAD_SAI5_RXD3__GPIO3_IO24 | MUX_PAD_CTRL(0x16),
#define GP_DRAM_1P1_VSEL		IMX_GPIO_NR(2, 11)
	IMX8MQ_PAD_SD1_STROBE__GPIO2_IO11 | MUX_PAD_CTRL(0x16),
#define GP_SOC_GPU_VPU_VSEL		IMX_GPIO_NR(2, 20)
	IMX8MQ_PAD_SD2_WP__GPIO2_IO20 | MUX_PAD_CTRL(0x16),

#define GP_BACKLIGHT_MIPI		IMX_GPIO_NR(5, 3)
	IMX8MQ_PAD_SPDIF_TX__GPIO5_IO3 | MUX_PAD_CTRL(0x4),

#define GP_FASTBOOT_KEY			IMX_GPIO_NR(1, 7)
	IMX8MQ_PAD_GPIO1_IO07__GPIO1_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),

#define GP_I2C1_PCA9546_RESET		IMX_GPIO_NR(1, 8)
	IMX8MQ_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(0x49),

#define GP_TC358762_EN			IMX_GPIO_NR(3, 15)
#define GP_SC18IS602B_RESET		IMX_GPIO_NR(3, 15)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(3, 15)
#define GP_MIPI_RESET			IMX_GPIO_NR(3, 15)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(3, 15)
	IMX8MQ_PAD_NAND_RE_B__GPIO3_IO15 | MUX_PAD_CTRL(0x6),


#define GP_EMMC_RESET			IMX_GPIO_NR(2, 10)
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(0x41),

#define GPIRQ_CSI1_TC3587		IMX_GPIO_NR(3, 3)
#define GP_CSI1_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(3, 3)
	IMX8MQ_PAD_NAND_CE2_B__GPIO3_IO3 | MUX_PAD_CTRL(0x45),
#define GP_CSI1_OV5640_MIPI_RESET	IMX_GPIO_NR(3, 17)
	IMX8MQ_PAD_NAND_WE_B__GPIO3_IO17 | MUX_PAD_CTRL(0x05),

#define GPIRQ_CSI2_TC3587		IMX_GPIO_NR(3, 2)
#define GP_CSI2_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(3, 2)
	IMX8MQ_PAD_NAND_CE1_B__GPIO3_IO2 | MUX_PAD_CTRL(0x45),
#define GP_CSI2_OV5640_MIPI_RESET	IMX_GPIO_NR(2, 19)
	IMX8MQ_PAD_SD2_RESET_B__GPIO2_IO19 |MUX_PAD_CTRL(0x05),
#ifdef CONFIG_FEC_MXC
	/* PHY - AR8035 */
	IMX8MQ_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(PAD_CTRL_ENET_MDIO),
	IMX8MQ_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(PAD_CTRL_ENET_MDC),
	IMX8MQ_PAD_ENET_TX_CTL__ENET_RGMII_TX_CTL | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MQ_PAD_ENET_TD0__ENET_RGMII_TD0 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MQ_PAD_ENET_TD1__ENET_RGMII_TD1 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MQ_PAD_ENET_TD2__ENET_RGMII_TD2 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MQ_PAD_ENET_TD3__ENET_RGMII_TD3 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MQ_PAD_ENET_TXC__ENET_RGMII_TXC | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
#endif
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 9)
	IMX8MQ_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP),
#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 11)
	IMX8MQ_PAD_GPIO1_IO11__GPIO1_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),
};

int dm_usb_gadget_handle_interrupts(struct udevice *dev)
{
	dwc3_uboot_handle_interrupt(dev);
	return 0;
}

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));


	gpio_request(GP_BACKLIGHT_MIPI, "backlight_mipi");
	gpio_request(GP_ARM_DRAM_VSEL, "arm_vsel");
	gpio_request(GP_DRAM_1P1_VSEL, "dram_vsel");
	gpio_request(GP_SOC_GPU_VPU_VSEL, "soc_vsel");
	gpio_request(GP_EMMC_RESET, "emmc_reset");
	gpio_request(GP_I2C1_PCA9546_RESET, "pca9546_reset");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
#endif
	gpio_request(GPIRQ_CSI1_TC3587, "csi1_tc3587");
	gpio_request(GP_CSI1_OV5640_MIPI_RESET, "csi1_ov5640_reset");
	gpio_request(GPIRQ_CSI2_TC3587, "csi2_tc3587");
	gpio_request(GP_CSI2_OV5640_MIPI_RESET, "csi2_ov5640_reset");

	gpio_direction_output(GP_BACKLIGHT_MIPI, 0);
	gpio_direction_output(GP_ARM_DRAM_VSEL, 0);
	gpio_direction_output(GP_DRAM_1P1_VSEL, 0);
	gpio_direction_output(GP_SOC_GPU_VPU_VSEL, 0);
	gpio_direction_output(GP_EMMC_RESET, 1);
	gpio_direction_output(GP_I2C1_PCA9546_RESET, 0);
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	gpio_direction_input(GPIRQ_CSI1_TC3587);
	gpio_direction_output(GP_CSI1_OV5640_MIPI_RESET, 0);
	gpio_direction_input(GPIRQ_CSI2_TC3587);
	gpio_direction_output(GP_CSI2_OV5640_MIPI_RESET, 0);

	return 0;
}

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_hub_gpio_init(void)
{
#define GP_USB1_HUB_RESET	IMX_GPIO_NR(1, 14)
	imx_iomux_v3_setup_pad(IMX8MQ_PAD_GPIO1_IO14__GPIO1_IO14 |
			MUX_PAD_CTRL(WEAK_PULLUP));
	return GP_USB1_HUB_RESET;
}
#endif

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_init(void)
{
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
#endif
	gpio_request(GP_TS_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_TS_GT911, "gt911_irq");
	gpio_direction_output(GP_TS_GT911_RESET, 0);

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
#endif
	return 0;
}

int board_fastboot_key_pressed(void)
{
	gpio_request(GP_FASTBOOT_KEY, "fastboot_key");
	gpio_direction_input(GP_FASTBOOT_KEY);
	return !gpio_get_value(GP_FASTBOOT_KEY);
}

void board_env_init(void)
{
#ifdef CONFIG_DM_VIDEO
	int ret = gpio_request(GP_LCD133_070_RESET, "bus_gp");

	if (!ret) {
		/*
		 * A mipi panel may have requested, only modify if not owned by
		 * sn65/ltk08
		 */
		/* An unmodified panel has reset connected directly to 1.8V, so make input */
		gpio_direction_input(GP_LCD133_070_RESET);
		gpio_free(GP_LCD133_070_RESET);
	}
#else
	/* An unmodified panel has reset connected directly to 1.8V, so make input */
	gpio_direction_input(GP_LCD133_070_RESET);
#endif
	/*
	 * If touchscreen reset is low, display will not initialize, but runs fine
	 * after init independent of gpio level
	 */
	gpio_direction_output(GP_TS_FT7250_RESET, 1); /* GP_TS_GT911_RESET */
}
