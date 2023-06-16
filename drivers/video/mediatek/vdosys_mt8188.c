// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek mt8188 vdosys driver
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#include <asm/io.h>
#include <dm.h>
#include <dm/device_compat.h>
#include "disp_reg_mt8188.h"

#define MTK_FB_ALIGNMENT 32
#define DISP_REG_SET(addr, val) writel(val, addr)
#define DISP_REG_SET_BITS(addr, val) setbits_le32(addr, val)
#define DISP_REG_CLR_BITS(addr, val) clrbits_le32(addr, val)

#define LOGO_BYTE_PER_PIXEL (4)

/* use MTK_MAINDISP_DEBUG to enable maindisp debug logs
 * #define MTK_MAINDISP_DEBUG
 */

#ifdef MTK_MAINDISP_DEBUG
#define maindisp_printf(string, args...) printf("[MAINDISP]"string, ##args)
#else
#define maindisp_printf(string, args...)
#endif

static u32 _maindisp_output_width;
static u32 _maindisp_output_height;
static u32 _logobase;

struct mtk_maindisp_priv {
	void __iomem *base;
};

static void mtk_maindisp_config_ovl(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset;

	reg_offset = (engine_num == 0) ? 0 : (OVL1_BASE - OVL0_BASE);

	DISP_REG_SET(reg_offset + DISP_REG_OVL_ROI_SIZE, ((width & 0x1fff) |
				 (height & 0x1fff) << 16));
	DISP_REG_SET(reg_offset + DISP_REG_OVL_ROI_BGCLR, 0xff000000);
	DISP_REG_SET(reg_offset + DISP_REG_OVL_SRC_CON, 0x00000001);

	DISP_REG_SET(reg_offset + DISP_REG_OVL_L0_CON, 0x008020FF);
	DISP_REG_SET(reg_offset + DISP_REG_OVL_L0_PITCH, width * LOGO_BYTE_PER_PIXEL);

	DISP_REG_SET(reg_offset + DISP_REG_OVL_L0_SRC_SIZE, ((width & 0x1fff) |
				 (height & 0x1fff) << 16));
	DISP_REG_SET(reg_offset + DISP_REG_OVL_L0_OFFSET, 0x00000000);
	if (engine_num == 0)
		DISP_REG_SET(reg_offset + DISP_REG_OVL_L0_ADDR, _logobase);
	else
		DISP_REG_SET(reg_offset + DISP_REG_OVL_L0_ADDR, _logobase +
					 (width * LOGO_BYTE_PER_PIXEL));

	DISP_REG_SET(reg_offset + DISP_REG_OVL_RDMA0_CTRL, 0x00000001);

	DISP_REG_SET(reg_offset + DISP_REG_OVL_DATAPATH_CON, 0x04000001);
	DISP_REG_SET(reg_offset + DISP_REG_OVL_EN, 0x00000001);
}

static void mtk_maindisp_config_rdma(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset;

	reg_offset = (engine_num == 0) ? 0 : (DISP_RDMA1_BASE - DISP_RDMA0_BASE);

	DISP_REG_SET(reg_offset + DISP_REG_RDMA_GLOBAL_CON, 0x00000001);
	DISP_REG_SET(reg_offset + DISP_REG_RDMA_SIZE_CON_0, (width & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_REG_RDMA_SIZE_CON_1, (height & 0x1fff));

	DISP_REG_SET(reg_offset + DISP_REG_RDMA_FIFO_CON, 0x07800000);
	DISP_REG_SET(reg_offset + DISP_REG_RDMA_MEM_GMC_SETTING_0, 0x85ed05b2);
	DISP_REG_SET(reg_offset + DISP_REG_RDMA_MEM_GMC_SETTING_1, 0x85b2053e);
	DISP_REG_SET(reg_offset + DISP_REG_RDMA_MEM_GMC_SETTING_2, 0x00000100);
	DISP_REG_SET(reg_offset + DISP_REG_RDMA_SRAM_SEL, 0);
	DISP_REG_SET(reg_offset + DISP_REG_RDMA_STALL_CG_CON, 0);
}

static void mtk_maindisp_config_color(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset;

	reg_offset = (engine_num == 0) ? 0 : (COLOR1_BASE - COLOR0_BASE);

	DISP_REG_SET(reg_offset + DISP_COLOR_INTERNAL_IP_WIDTH, (width & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_COLOR_INTERNAL_IP_HEIGHT, (height & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_COLOR_CFG_MAIN, 0x02000080);
	DISP_REG_SET(reg_offset + DISP_COLOR_CM1_EN, 0x00000000);
	DISP_REG_SET(reg_offset + DISP_COLOR_CM2_EN, 0x00000000);
	DISP_REG_SET(reg_offset + DISP_COLOR_START, 0x00000013);
}

static void mtk_maindisp_config_ccorr(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset;

	reg_offset = (engine_num == 0) ? 0 : (CCORR1_BASE - CCORR0_BASE);

	DISP_REG_SET(reg_offset + DISP_REG_CCORR_SIZE, (width & 0x1fff) << 16 |
				 (height & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_REG_CCORR_CFG, 0x00000401);
	DISP_REG_SET(reg_offset + DISP_REG_CCORR_EN, 0x00000001);
}

static void mtk_maindisp_config_aal(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset;

	reg_offset = (engine_num == 0) ? 0 : (DISP_AAL1_BASE - DISP_AAL0_BASE);

	DISP_REG_SET(reg_offset + DISP_AAL_SIZE, (width & 0x1fff) << 16 | (height & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_AAL_OUTPUT_SIZE, (width & 0x1fff) << 16 | (height & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_AAL_EN, 0x00000001);
	DISP_REG_SET(reg_offset + DISP_AAL_CFG, 0x00000111);
}

static void mtk_maindisp_config_gamma(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset;

	reg_offset = (engine_num == 0) ? 0 : (DISP_GAMMA1_BASE - DISP_GAMMA0_BASE);

	DISP_REG_SET(reg_offset + DISP_REG_GAMMA_SIZE, (width & 0x1fff) << 16 |
				 (height & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_REG_GAMMA_CFG, 0x00000101);
	DISP_REG_SET(reg_offset + DISP_REG_GAMMA_EN, 0x00000001);
}

static void mtk_maindisp_config_postmask(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset = 0;

	DISP_REG_SET(reg_offset + DISP_REG_POSTMASK_SIZE, (width & 0x1fff) << 16 |
				 (height & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_REG_POSTMASK_CFG, 0x147);
	DISP_REG_SET(reg_offset + DISP_REG_POSTMASK_EN, 0x1);
}

static void mtk_maindisp_config_dither(u32 engine_num, u32 width, u32 height)
{
	unsigned int reg_offset;

	reg_offset = (engine_num == 0) ? 0 : (DITHER1_BASE - DITHER0_BASE);

	DISP_REG_SET(reg_offset + DISP_REG_DITHER_SIZE, (width & 0x1fff) << 16 |
				 (height & 0x1fff));
	DISP_REG_SET(reg_offset + DISP_REG_DITHER_CFG, 0x00000001);
	DISP_REG_SET(reg_offset + DISP_REG_DITHER_EN, 0x00000001);
	DISP_REG_SET(reg_offset + DISP_REG_DITHER_14, 0x00000000);
	DISP_REG_SET(reg_offset + DISP_REG_DITHER_6, 0x00003004);
	DISP_REG_SET(reg_offset + DISP_REG_DITHER_5, 0x00000000);
}

static void mtk_maindisp_config_merge(u32 width, u32 height)
{
	DISP_REG_SET(DISP_MERGE_CFG_0, ((height << 16) | width));
	DISP_REG_SET(DISP_MERGE_CFG_1, ((height << 16) | width));

	DISP_REG_SET(DISP_MERGE_CFG_4, ((height << 16) | width));
	DISP_REG_SET(DISP_MERGE_CFG_12, 0x6);

	DISP_REG_SET(DISP_MERGE_CFG_24, ((height << 16) | width));
	DISP_REG_SET(DISP_MERGE_CFG_25, ((height << 16) | width));
	DISP_REG_SET(DISP_MERGE_CFG_26, ((height << 16) | width));
	DISP_REG_SET(DISP_MERGE_CFG_27, ((height << 16) | width));
	DISP_REG_SET(DISP_MERGE_ENABLE, 0x00000001);
}

static void mtk_maindisp_config_mux(void)
{
	u32 width = 0, height = 0;

	mtk_maindisp_get_w_h(&width, &height);

	DISP_REG_SET(MT8188_VDO0_OVL_MOUT_EN, 0x1);
	DISP_REG_SET(MT8188_VDO0_DISP_RDMA_SEL, 0x0);
	DISP_REG_SET(MT8188_VDO0_DSI0_SEL, 0x1);
	DISP_REG_SET(MT8188_VDO0_DISP_DITHER0_SEL, 0x1);
	DISP_REG_SET(MT8188_VDO0_VPP_MERGE_SEL, 0x0);
	DISP_REG_SET(MT8188_VDO0_DSC_WRAP_SEL, 0x0);
}

static void mtk_maindisp_config_mutex(void)
{
	DISP_REG_SET(MM_MUTEX_BASE + 0x030, 0x10001fd);
	DISP_REG_SET(MM_MUTEX_BASE + 0x034, 0x0);
	DISP_REG_SET(MM_MUTEX_BASE + 0x02c, 0x81);
	DISP_REG_SET(MM_MUTEX_BASE + 0x034, 0x0);
	DISP_REG_SET(MM_MUTEX_BASE + 0x020, 0x1);
}

static void mtk_maindisp_init(void)
{
	/* config VDOSYS0 CG to support dsi0 */
	DISP_REG_SET(VDOSYS0_CONFIG_BASE + 0x108, 0x34280551);
	DISP_REG_SET(VDOSYS0_CONFIG_BASE + 0x118, 0xfc01);

	/* Force enable SMI_LARB1 clk on vpp0_cg for OVL1 */
	DISP_REG_SET(DISPSYS_SMI_LARB0_BASE + 0x78, 0xffffffff);

	/* Disable Larb 0 IOMMU */
	DISP_REG_SET(DISPSYS_SMI_LARB0_BASE + 0x388, 0x0);
}

void mtk_maindisp_reset(void)
{
	/* reset dsi0 via VDOSYS0 reset bit */
	DISP_REG_CLR_BITS(VDOSYS0_CONFIG_BASE + 0x190, 0x200000);
	DISP_REG_SET_BITS(VDOSYS0_CONFIG_BASE + 0x190, 0x200000);
}

void mtk_maindisp_set_w_h(u32 width, u32 height)
{
	_maindisp_output_width = width;
	_maindisp_output_height = height;
}

void mtk_maindisp_get_w_h(u32 *width, u32 *height)
{
	*width = _maindisp_output_width;
	*height = _maindisp_output_height;
}

void mt_maindisp_set_fb_addr(u32 addr)
{
	_logobase = addr;
}

u32 mt_maindisp_get_fb_addr(void)
{
	return _logobase;
}

void mtk_maindisp_update(u32 x, u32 y, u32 width, u32 height)
{
	maindisp_printf("%s x = %d y = %d width = %d height = %d\n", __func__, x, y, width, height);

	if (width == 0 || height == 0)
		return;
	mtk_maindisp_init();
	mtk_maindisp_config_mux();

	mtk_maindisp_config_ovl(0, width, height);
	mtk_maindisp_config_rdma(0, width, height);
	mtk_maindisp_config_color(0, width, height);
	mtk_maindisp_config_ccorr(0, width, height);
	mtk_maindisp_config_aal(0, width, height);
	mtk_maindisp_config_gamma(0, width, height);
	mtk_maindisp_config_postmask(0, width, height);
	mtk_maindisp_config_dither(0, width, height);
	mtk_maindisp_config_merge(width, height);

	mtk_maindisp_config_mutex();
}

static int mtk_maindisp_probe(struct udevice *dev)
{
	struct mtk_maindisp_priv *maindisp = dev_get_priv(dev);

	maindisp->base = dev_remap_addr(dev);
	if (IS_ERR(maindisp->base))
		return PTR_ERR(maindisp->base);

	return 0;
}

static const struct udevice_id mtk_maindisp_ids[] = {
	{ .compatible = "mediatek,mt8188-mmsys" },
	{}
};

U_BOOT_DRIVER(mtk_maindisp) = {
	.name	   = "mtk_maindisp",
	.id	   = UCLASS_MISC,
	.of_match  = mtk_maindisp_ids,
	.probe	   = mtk_maindisp_probe,
	.priv_auto = sizeof(struct mtk_maindisp_priv),
};
