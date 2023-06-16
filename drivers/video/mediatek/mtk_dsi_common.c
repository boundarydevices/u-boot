// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek dsi common entrance and function
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <video.h>
#include <video_bridge.h>
#include <linux/delay.h>
#include <string.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <asm/gpio.h>

#include "mtk_panel.h"
#include "mtk_dsi_common.h"

/* use MTK_DSI_DEBUG to enable dsi debug logs
 * #define MTK_DSI_DEBUG
 */

#ifdef MTK_DSI_DEBUG
#define dsi_printf(string, args...) printf("[DSI]"string, ##args)
#else
#define dsi_printf(string, args...)
#endif

unsigned char *fb;

unsigned int mtk_dsi_get_bits_per_pixel(u32 format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return 16;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return 18;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB888:
		return 24;
	}
	printf("%s: Warning: Unknown format %d, assuming 24 bpp\n",
	       __func__, format);
	return 24;
}

int mtk_dsi_get_data_rate(u32 bits_per_pixel, u32 lanes, struct videomode *vm)
{
	/* data_rate = pixel_clock * bits_per_pixel * mipi_ratio / lanes
	 * Note pixel_clock comes in kHz and returned data_rate is in Mbps.
	 * mipi_ratio is the clk coefficient to balance the pixel clk in MIPI
	 * for older platforms which do not have complete implementation in HFP.
	 * Newer platforms should just set that to 1.0 (100 / 100).
	 */
	int data_rate;

	if (vm->pll_clk)
		data_rate = vm->pll_clk * 2;
	else
		data_rate = vm->pixelclock * bits_per_pixel *
			MTK_DSI_MIPI_RATIO_NUMERATOR /
			(1000 * lanes * MTK_DSI_MIPI_RATIO_DENOMINATOR);

	dsi_printf("DSI data_rate: %d Mbps\n", data_rate);

	if (data_rate < MTK_DSI_DATA_RATE_MIN_MHZ) {
		printf("data rate (%dMbps) must be >=%dMbps.\n"
		       "Please check the pixel clock (%u), bits per pixel(%u),\n"
		       "mipi_ratio (%d%%) and number of lanes (%d)\n",
		       data_rate, MTK_DSI_DATA_RATE_MIN_MHZ,
		       vm->pixelclock, bits_per_pixel,
		       (100 * MTK_DSI_MIPI_RATIO_NUMERATOR /
			MTK_DSI_MIPI_RATIO_DENOMINATOR), lanes);
		return -1;
	}
	return data_rate;
}

bool mtk_dsi_is_read_command(u32 type)
{
	switch (type) {
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
	case MIPI_DSI_DCS_READ:
		return true;
	}
	return false;
}

int mtk_dsi_set_timing_reg(u32 mode_flags, u32 format, u32 lanes, struct videomode *vm)
{
	int data_rate;
	u32 bits_per_pixel = mtk_dsi_get_bits_per_pixel(format);
	struct mtk_phy_timing phy_timing;

	data_rate = mtk_dsi_get_data_rate(bits_per_pixel, lanes, vm);
	if (data_rate < 0)
		return -1;
	dsi_printf("dsi mode_flags=0x%X\n", mode_flags);
	dsi_printf("htotal:%d\n", vm->hactive + vm->hfront_porch + vm->hback_porch + vm->hsync_len);
	dsi_printf("vtotal:%d\n", vm->vactive + vm->vfront_porch + vm->vback_porch + vm->vsync_len);
	dsi_printf("pixel_clock:%d\n", vm->pixelclock);

	mtk_dsi_configure_mipi_tx(data_rate, lanes);

	mtk_dsi_reset();
	mtk_dsi_set_mode(0);
	mtk_dsi_phy_timing(data_rate, &phy_timing);
	mtk_dsi_rxtx_control(mode_flags, lanes);
	mdelay(1);
	mtk_dsi_reset_dphy();
	mtk_dsi_clk_hs_mode_disable();
	mtk_dsi_config_vdo_timing(mode_flags, format, lanes, vm, &phy_timing);
	mtk_dsi_clk_hs_mode_enable();

	return 0;
}

void mtk_dsi_send_init_commands(const u8 *buf)
{
	if (!buf)
		return;
	const struct lcm_init_command *init = (const void *)buf;

	/*
	 * The given commands should be in a buffer containing a packed array of
	 * lcm_init_command and each element may be in variable size so we have
	 * to parse and scan.
	 */

	for (; init->cmd != LCM_END_CMD; init = (const void *)buf) {
		/*
		 * For some commands like DELAY, the init->len should not be
		 * counted for buf.
		 */
		buf += sizeof(*init);

		u32 cmd = init->cmd, len = init->len;
		u32 type;

		switch (cmd) {
		case LCM_DELAY_CMD:
			mdelay(len);
			continue;

		case LCM_DCS_CMD:
			switch (len) {
			case 0:
				return;
			case 1:
				type = MIPI_DSI_DCS_SHORT_WRITE;
				break;
			case 2:
				type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
				break;
			default:
				type = MIPI_DSI_DCS_LONG_WRITE;
				break;
			}
			break;

		case LCM_GENERIC_CMD:
			switch (len) {
			case 0:
				type = MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM;
				break;
			case 1:
				type = MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM;
				break;
			case 2:
				type = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM;
				break;
			default:
				type = MIPI_DSI_GENERIC_LONG_WRITE;
				break;
			}
			break;

		default:
			printf("%s: Unknown cmd: %d,\n"
			       "abort panel initialization.\n", __func__, cmd);
			return;
		}
		buf += len;
		mtk_dsi_cmdq(init->data, len, type);
	}
}

static void mtk_dsi_init(struct panel_description *panel_desc)
{
	struct dsi_info *dsi;

	if (panel_desc->power_on)
		panel_desc->power_on();

	if (panel_desc->backlight_enable)
		panel_desc->backlight_enable();

	dsi = panel_desc->s->dsi;
	mtk_dsi_set_timing_reg(dsi->flag, dsi->format, dsi->lanes, &panel_desc->s->vm);

	mtk_dsi_send_init_commands(panel_desc->s->init_cmd);
	mtk_dsi_set_mode(dsi->flag);
	mtk_dsi_start();

#ifdef MTK_DSI_DEBUG
	mtk_dsi_dump();
#endif
}

static int mtk_dsi_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_priv *priv = dev_get_uclass_priv(dev);
	struct panel_description *panel_desc = NULL;
	int ret;

	panel_get_desc(&panel_desc);
	if (!panel_desc) {
		printf("Warning: %s cannot get panel_get_desc\n", __func__);
		return -ENODEV;
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &panel_desc->reset_gpio, GPIOD_IS_OUT);
	if (ret) {
		printf("Warning: %s cannot get reset GPIO: ret=%d\n", ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "dcdc-gpios", 0, &panel_desc->dcdc_en_gpio, GPIOD_IS_OUT);
	if (ret) {
		printf("Warning: cannot get dcdc GPIO: ret=%d\n", ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "enable-gpios", 0, &panel_desc->enable_gpio, GPIOD_IS_OUT);
	if (ret) {
		printf("Warning: cannot get enable GPIO: ret=%d\n", ret);
		return ret;
	}

	if (panel_desc->backlight_enable) {
		ret = clk_get_by_name(dev, "main", &panel_desc->pwm_main);
		if (ret)
			printf("Warning: cannot get pwm_main: ret=%d\n", ret);

		ret = clk_get_by_name(dev, "mm", &panel_desc->pwm_mm);
		if (ret)
			printf("Warning: cannot get pwm_mm: ret=%d\n", ret);
	}

	mtk_dsi_init(panel_desc);

	priv->bpix = VIDEO_BPP32;
	priv->xsize = panel_desc->s->vm.hactive;
	priv->ysize = panel_desc->s->vm.vactive;
	priv->line_length = panel_desc->s->vm.hactive * VNBYTES(VIDEO_BPP32);

	mtk_maindisp_set_w_h(priv->xsize, priv->ysize);
	mt_maindisp_set_fb_addr(plat->base);
	mtk_maindisp_update(0, 0, priv->xsize, priv->ysize);

	video_set_flush_dcache(dev, true);

	return 0;
}

static int mtk_dsi_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	plat->size = MTK_DSI_MAX_WIDTH * MTK_DSI_MAX_HEIGHT * VNBYTES(VIDEO_BPP32);

	fb = (unsigned char *)malloc(plat->size * sizeof(unsigned char));
	if (!fb)
		printf("UBOOT: %s possibly use system framebuffer\n", __func__);

	plat->base = (ulong)fb;

	return 0;
}

static int mtk_dsi_remove(struct udevice *dev)
{
	struct panel_description *panel_desc;

	panel_get_desc(&panel_desc);

	mtk_maindisp_reset();

	if (panel_desc->power_off)
		panel_desc->power_off();

	if (dm_gpio_is_valid(&panel_desc->reset_gpio))
		dm_gpio_free(dev, &panel_desc->reset_gpio);
	if (dm_gpio_is_valid(&panel_desc->dcdc_en_gpio))
		dm_gpio_free(dev, &panel_desc->dcdc_en_gpio);
	if (dm_gpio_is_valid(&panel_desc->enable_gpio))
		dm_gpio_free(dev, &panel_desc->enable_gpio);

	return 0;
}

static const struct udevice_id mtk_dsi_ids[] = {
	{ .compatible = "mediatek,mt8188-dsi" },
	{}
};

U_BOOT_DRIVER(mtk_dsi) = {
	.name	   = "mtk_dsi",
	.id	   = UCLASS_VIDEO,
	.of_match  = mtk_dsi_ids,
	.bind	   = mtk_dsi_bind,
	.probe	   = mtk_dsi_probe,
	.remove    = mtk_dsi_remove,
	.flags	   = DM_FLAG_PRE_RELOC | DM_FLAG_LEAVE_PD_ON,
};
