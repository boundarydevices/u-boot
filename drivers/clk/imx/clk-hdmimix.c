// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 NXP.
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <dt-bindings/clock/imx8mp-clock.h>
#include <log.h>

#include "clk.h"

struct clk_hdmimix_data {
	struct clk *clks[IMX8MP_CLK_HDMIMIX_END];
};

static const char *imx_hdmi_phy_clks_sels[] = { "clock-osc-24m", "video_pll1_out",};
static const char *imx_lcdif_clks_sels[] = { "dummy", "hdmi_glb_pix", };
static const char *imx_hdmi_pipe_clks_sels[] = {"dummy","hdmi_glb_pix", };

#define BASE_HDMI_CLK	(IMX8MP_CLK_END + IMX8MP_CLK_AUDIOMIX_END)

static struct clk *get_clock(struct clk_hdmimix_data *priv, unsigned id)
{
	struct clk *c;
	int ret;

	if ((id >= BASE_HDMI_CLK) && (id < (BASE_HDMI_CLK + IMX8MP_CLK_HDMIMIX_END))) {
		c = priv->clks[id - BASE_HDMI_CLK];
		if (c)
			return c;
	}
	ret = clk_get_by_id(id, &c);
	if (ret)
		return ERR_PTR(ret);
	return c;
}

int imx8mp_hdmi_clk_of_xlate(struct clk *clk, struct ofnode_phandle_args *args)
{
	if (args->args_count == 1) {
		if (args->args[0] < IMX8MP_CLK_HDMIMIX_END) {
			clk->id = args->args[0] + BASE_HDMI_CLK;
			return 0;
		}
	}
	return -EINVAL;
}

static ulong imx8mp_hdmi_clk_get_rate(struct clk *clk)
{
	struct clk_hdmimix_data *priv = dev_get_priv(clk->dev);
	struct clk *c;

	debug("%s(#%lu)\n", __func__, clk->id);

	c = get_clock(priv, clk->id);
	if (IS_ERR(c))
		return PTR_ERR(c);

	return clk_get_rate(c);
}

static ulong imx8mp_hdmi_clk_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk_hdmimix_data *priv = dev_get_priv(clk->dev);
	struct clk *c;

	debug("%s(#%lu), rate: %lu\n", __func__, clk->id, rate);

	c = get_clock(priv, clk->id);
	if (IS_ERR(c))
		return PTR_ERR(c);

	return clk_set_rate(c, rate);
}

static int __imx8mp_hdmi_clk_enable(struct clk *clk, bool enable)
{
	struct clk_hdmimix_data *priv = dev_get_priv(clk->dev);
	struct clk *c;
	int ret;

	debug("%s(#%lu) en: %d\n", __func__, clk->id, enable);

	c = get_clock(priv, clk->id);
	if (IS_ERR(c))
		return PTR_ERR(c);

	if (enable)
		ret = clk_enable(c);
	else
		ret = clk_disable(c);

	return ret;
}

static int imx8mp_hdmi_clk_disable(struct clk *clk)
{
	return __imx8mp_hdmi_clk_enable(clk, 0);
}

static int imx8mp_hdmi_clk_enable(struct clk *clk)
{
	return __imx8mp_hdmi_clk_enable(clk, 1);
}

static int hdmi_clk_set_parent(struct clk *c, struct clk *cp)
{
	int ret = clk_set_parent(c, cp);
	if (!ret) {
		list_del(&c->dev->sibling_node);
		list_add_tail(&c->dev->sibling_node, &cp->dev->child_head);
		c->dev->parent = cp->dev;
	} else {
		printf("%s: %s %s: %d failed\n", __func__,
				c->dev->name, cp->dev->name, ret);
	}
	debug("%s(#%lu)%s, parent: (%lu)%s\n", __func__, c->id, c->dev->name,
			cp->id, cp->dev->name);
	return ret;
}

static int imx8mp_hdmi_clk_set_parent(struct clk *clk, struct clk *parent)
{
	struct clk_hdmimix_data *priv = dev_get_priv(clk->dev);
	struct clk *c, *cp;
	int ret;

	debug("%s(#%lu), parent: %lu\n", __func__, clk->id, parent->id);

	c = get_clock(priv, clk->id);
	if (IS_ERR(c))
		return PTR_ERR(c);
	cp = get_clock(priv, parent->id);
	if (IS_ERR(cp))
		return PTR_ERR(cp);

	ret = hdmi_clk_set_parent(c, cp);
	return ret;
}

static inline void _clk_dm1(struct clk_hdmimix_data *priv, ulong id, struct clk *clk)
{
	if (!IS_ERR(clk))
		clk->id = id + BASE_HDMI_CLK;
	priv->clks[id] = clk;
}

#define clk_dm1(id, clk) _clk_dm1(priv, id, clk)

static int imx_hdmimix_clk_probe(struct udevice *dev)
{
	struct clk_hdmimix_data *priv = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	struct clk clk_tmp;
	struct clk clk_24m;
	void __iomem *base;
	int ret;

	debug("%s\n", __func__);
	base = (void *)dev_read_addr(dev);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk_dm1(IMX8MP_CLK_HDMIMIX_GLOBAL_XTAL24M_CLK, imx_dev_clk_gate(dev, "hdmi_glb_24m",     "hdmi_24m",      base + 0x40, 4));

	ret = clk_get_by_name_nodev(np, "osc_24m", &clk_24m);
	if (ret)
		debug("%s: osc_24m clock failed\n", __func__);

	clk_dm1(IMX8MP_CLK_HDMIMIX_GLOBAL_APB_CLK, imx_dev_clk_gate(dev, "hdmi_glb_apb",     "hdmi_apb",      base + 0x40, 0));
	clk_dm1(IMX8MP_CLK_HDMIMIX_GLOBAL_B_CLK, imx_dev_clk_gate(dev, "hdmi_glb_b",       "hdmi_axi",      base + 0x40, 1));
	clk_dm1(IMX8MP_CLK_HDMIMIX_GLOBAL_REF266M_CLK, imx_dev_clk_gate(dev, "hdmi_glb_ref_266m","hdmi_ref_266m", base + 0x40, 2));
	clk_dm1(IMX8MP_CLK_HDMIMIX_GLOBAL_XTAL32K_CLK, imx_dev_clk_gate(dev, "hdmi_glb_32k",     "clock-osc-32k", base + 0x40, 5));

	clk_dm1(IMX8MP_CLK_HDMIMIX_IRQS_STEER_CLK, imx_dev_clk_gate(dev, "hdmi_irq_steer",   "hdmi_glb_apb",  base + 0x40, 9));
	clk_dm1(IMX8MP_CLK_HDMIMIX_NOC_HDMI_CLK, imx_dev_clk_gate(dev, "hdmi_noc",         "hdmi_glb_apb",  base + 0x40, 10));
	clk_dm1(IMX8MP_CLK_HDMIMIX_NOC_HDCP_CLK, imx_dev_clk_gate(dev, "hdcp_noc",         "hdmi_glb_apb",  base + 0x40, 11));
	clk_dm1(IMX8MP_CLK_HDMIMIX_LCDIF_APB_CLK, imx_dev_clk_gate(dev, "lcdif3_apb",       "hdmi_glb_apb",  base + 0x40, 16));
	clk_dm1(IMX8MP_CLK_HDMIMIX_LCDIF_B_CLK, imx_dev_clk_gate(dev, "lcdif3_b",         "hdmi_glb_b",    base + 0x40, 17));
	clk_dm1(IMX8MP_CLK_HDMIMIX_LCDIF_PDI_CLK, imx_dev_clk_gate(dev, "lcdif3_pdi",       "hdmi_glb_apb",  base + 0x40, 18));
	clk_dm1(IMX8MP_CLK_HDMIMIX_LCDIF_SPU_CLK, imx_dev_clk_gate(dev, "lcdif3_spu",       "hdmi_glb_apb",  base + 0x40, 20));

	clk_dm1(IMX8MP_CLK_HDMIMIX_FDCC_REF_CLK, imx_dev_clk_gate(dev, "hdmi_fdcc_ref",    "hdmi_fdcc_tst", base + 0x50, 2));
	clk_dm1(IMX8MP_CLK_HDMIMIX_HRV_MWR_APB_CLK, imx_dev_clk_gate(dev, "hrv_mwr_apb",       "hdmi_glb_apb", base + 0x50, 3));
#if 0
	clk_dm1(IMX8MP_CLK_HDMIMIX_HRV_MWR_B_CLK, imx_dev_clk_gate(dev, "hrv_mwr_b",         "hdmi_glb_axi", base + 0x50, 4));
#endif
	clk_dm1(IMX8MP_CLK_HDMIMIX_HRV_MWR_CEA_CLK, imx_dev_clk_gate(dev, "hrv_mwr_cea",       "hdmi_glb_apb", base + 0x50, 5));
	clk_dm1(IMX8MP_CLK_HDMIMIX_VSFD_CEA_CLK, imx_dev_clk_gate(dev, "vsfd_cea",          "hdmi_glb_apb", base + 0x50, 6));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_HPI_CLK, imx_dev_clk_gate(dev, "hdmi_tx_hpi",       "hdmi_glb_apb", base + 0x50, 13));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_APB_CLK, imx_dev_clk_gate(dev, "hdmi_tx_apb",       "hdmi_glb_apb", base + 0x50, 14));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_CEC_CLK, imx_dev_clk_gate(dev, "hdmi_cec",          "hdmi_glb_32k", base + 0x50, 15));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_ESM_CLK, imx_dev_clk_gate(dev, "hdmi_esm",     "hdmi_glb_ref_266m", base + 0x50, 16));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_GPA_CLK, imx_dev_clk_gate(dev, "hdmi_tx_gpa",       "hdmi_glb_apb", base + 0x50, 17));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_SFR_CLK, imx_dev_clk_gate(dev, "hdmi_tx_sfr",       "hdmi_glb_apb", base + 0x50, 19));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_SKP_CLK, imx_dev_clk_gate(dev, "hdmi_tx_skp",       "hdmi_glb_apb", base + 0x50, 20));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_PREP_CLK, imx_dev_clk_gate(dev, "hdmi_tx_prep",      "hdmi_glb_apb", base + 0x50, 21));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_PHY_APB_CLK, imx_dev_clk_gate(dev, "hdmi_phy_apb",      "hdmi_glb_apb", base + 0x50, 22));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_SEC_MEM_CLK, imx_dev_clk_gate(dev, "hdmi_sec_mem", "hdmi_glb_ref_266m", base + 0x50, 25));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_TRNG_SKP_CLK, imx_dev_clk_gate(dev, "hdmi_trng_skp",     "hdmi_glb_apb", base + 0x50, 27));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_TRNG_APB_CLK, imx_dev_clk_gate(dev, "hdmi_trng_apb",     "hdmi_glb_apb", base + 0x50, 30));

	clk_dm1(IMX8MP_CLK_HDMIMIX_HTXPHY_CLK_SEL, imx_dev_clk_mux(dev, "hdmi_phy_sel", base + 0x50, 10, 1, imx_hdmi_phy_clks_sels, ARRAY_SIZE(imx_hdmi_phy_clks_sels)));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_PHY_INT_CLK, imx_dev_clk_gate(dev, "hdmi_phy_int",      "hdmi_phy_sel", base + 0x50, 24));

	/* probe parent of hdmi_glb_pix, child of above hdmi_glb_24m */
	ret = clk_get_by_name_nodev(np, "hdmi_phy", &clk_tmp);
	if (ret)
		debug("%s: hdmi_phy clock failed\n", __func__);

	clk_dm1(IMX8MP_CLK_HDMIMIX_GLOBAL_TX_PIX_CLK, imx_dev_clk_gate(dev, "hdmi_glb_pix",     "hdmi_phy",      base + 0x40, 7));
	clk_dm1(IMX8MP_CLK_HDMIMIX_LCDIF_PIX_CLK, imx_dev_clk_gate(dev, "lcdif3_pxl",       "hdmi_glb_pix",  base + 0x40, 19));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_PIXEL_CLK, imx_dev_clk_gate(dev, "hdmi_tx_pix",       "hdmi_glb_pix", base + 0x50, 18));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_VID_LINK_PIX_CLK, imx_dev_clk_gate(dev, "hdmi_vid_pix",     "hdmi_glb_pix", base + 0x50, 28));

	clk_dm1(IMX8MP_CLK_HDMIMIX_LCDIF_CLK_SEL, imx_dev_clk_mux(dev, "lcdif_clk_sel", base + 0x50, 11, 1, imx_lcdif_clks_sels, ARRAY_SIZE(imx_lcdif_clks_sels)));
	clk_dm1(IMX8MP_CLK_HDMIMIX_TX_PIPE_CLK_SEL, imx_dev_clk_mux(dev, "hdmi_pipe_sel", base + 0x50, 12, 1, imx_hdmi_pipe_clks_sels, ARRAY_SIZE(imx_hdmi_pipe_clks_sels)));

	/* hdmi/lcdif pixel clock parent to hdmi phy */
	hdmi_clk_set_parent(priv->clks[IMX8MP_CLK_HDMIMIX_TX_PIPE_CLK_SEL], priv->clks[IMX8MP_CLK_HDMIMIX_GLOBAL_TX_PIX_CLK]);
	hdmi_clk_set_parent(priv->clks[IMX8MP_CLK_HDMIMIX_LCDIF_CLK_SEL], priv->clks[IMX8MP_CLK_HDMIMIX_GLOBAL_TX_PIX_CLK]);
	/* hdmi ref clock from 24MHz */
	hdmi_clk_set_parent(priv->clks[IMX8MP_CLK_HDMIMIX_HTXPHY_CLK_SEL], &clk_24m);

	return 0;
}

static struct clk_ops imx8mp_hdmi_clk_ops = {
	.of_xlate = imx8mp_hdmi_clk_of_xlate,
	.set_rate = imx8mp_hdmi_clk_set_rate,
	.get_rate = imx8mp_hdmi_clk_get_rate,
	.enable = imx8mp_hdmi_clk_enable,
	.disable = imx8mp_hdmi_clk_disable,
	.set_parent = imx8mp_hdmi_clk_set_parent,
};

static const struct udevice_id imx_hdmimix_clk_of_match[] = {
	{ .compatible = "fsl,imx8mp-hdmimix-clk" },
	{ /* Sentinel */ },
};

U_BOOT_DRIVER(imx_hdmimix_clk) = {
	.name = "imx-hdmimix-clk",
	.id = UCLASS_CLK,
	.of_match = imx_hdmimix_clk_of_match,
	.ops = &imx8mp_hdmi_clk_ops,
	.probe = imx_hdmimix_clk_probe,
	.priv_auto = sizeof(struct clk_hdmimix_data),
};
