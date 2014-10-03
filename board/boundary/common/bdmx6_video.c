/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/video.h>
#include <asm/io.h>
#include <i2c.h>
#include "bdmx6_video.h"

/* Special MXCFB sync flags are here. */
#include "../drivers/video/mxcfb.h"

#ifdef CONFIG_IMX_HDMI
static void do_enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}
#endif

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus)) &&
		(0 == i2c_probe(dev->addr)));
}

#ifdef CONFIG_BDMX6_LVDS0

static void enable_lvds_backlight(void)
{
#ifdef CONFIG_BDMX6_LVDS0_PWM
	gpio_direction_output(CONFIG_BDMX6_LVDS0_PWM, 1);
#endif
#ifdef CONFIG_BDMX6_LVDS0_ENABLE
	gpio_direction_output(CONFIG_BDMX6_LVDS0_PWM, 1);
#endif
}

static void enable_lvds_spwg(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
	enable_lvds_backlight();
}

static void enable_lvds_jeida(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_JEIDA;
	writel(reg, &iomux->gpr[2]);
	enable_lvds_backlight();
}
#endif

#ifdef CONFIG_BDMX6_RGB
static void enable_rgb(struct display_info_t const *dev)
{
#ifdef CONFIG_BDMX6_RGB_PWM
	gpio_direction_output(CONFIG_BDMX6_RGB_PWM, 1);
#endif
#ifdef CONFIG_BDMX6_RGB_ENABLE
	gpio_direction_output(CONFIG_BDMX6_RGB_ENABLE, 1);
#endif
}
#endif

#ifdef CONFIG_BDMX6_DISPLAYS
struct display_info_t const displays[] = {
#ifdef CONFIG_IMX_HDMI
{
	.bus	= CONFIG_BDMX6_HDMI_I2C_BUS,
	.addr	= 0x50,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_i2c,
	.enable	= do_enable_hdmi,
	.mode	= BDMX6_HDMI_SVGA
},
#endif
#ifdef CONFIG_BDMX6_LVDS0
{
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds_jeida,
	.mode	= BDMX6_LVDS_WXGA
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds_spwg,
	.mode	= BDMX6_LVDS_WXGA_S
}, {
	.bus	= CONFIG_BDMX6_LVDS0_I2C_BUS,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds_spwg,
	.mode	= BDMX6_LVDS_HANNSTAR
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= NULL,
	.enable	= enable_lvds_spwg,
	.mode	= BDMX6_LVDS_LG_9_7
}, {
	.bus	= CONFIG_BDMX6_LVDS0_I2C_BUS,
	.addr	= 0x38,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds_spwg,
	.mode	= BDMX6_LVDS_WSVGA
}, {
	.bus	= CONFIG_BDMX6_LVDS0_I2C_BUS,
	.addr	= 0x41,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds_spwg,
	.mode	= BDMX6_LVDS_AMP_1024X600
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= NULL,
	.enable	= enable_lvds_spwg,
	.mode	= BDMX6_LVDS_WVGA_SPWG
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds_jeida,
	.mode	= BDMX6_LVDS_WVGA_JEIDA
},
#endif
#ifdef CONFIG_BDMX6_RGB
{
	.bus	= CONFIG_BDMX6_RGB_I2C_BUS,
	.addr	= 0x10,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_i2c,
	.enable	= enable_rgb,
	.mode	= BDMX6_RGB_FUSION7
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= BDMX6_RGB_SVGA
}, {
	.bus	= CONFIG_BDMX6_LVDS0_I2C_BUS,
	.addr	= 0x48,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_i2c,
	.enable	= enable_rgb,
	.mode	= BDMX6_RGB_WVGA
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= BDMX6_RGB_HITACHI_HVGA
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= BDMX6_RGB_WQVGA
}, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= BDMX6_RGB_QVGA
}
#endif
};
size_t display_count = ARRAY_SIZE(displays);
#endif

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}


