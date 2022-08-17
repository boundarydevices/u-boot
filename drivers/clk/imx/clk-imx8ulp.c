// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include <common.h>
#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <log.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <dt-bindings/clock/imx8ulp-clock.h>

#include "clk.h"
struct clk8ulp_data {
	unsigned base_id;
	unsigned dev_id;
	void __iomem *base;
	ofnode np;
	const struct imx8ulp_clk_probe *cps;
	unsigned num_clks;
	struct clk **clks;
};

enum {
	dev_none = 0,
	dev_cgc1 = 1 << 12,
	dev_cgc2 = 2 << 12,
	dev_pcc3 = 3 << 12,
	dev_pcc4 = 4 << 12,
	dev_pcc5 = 5 << 12,
	dev_end = 6
};

#define DEV_ID(a)	(a >> 12)
#define CLK_ID(a)	(a & 0xfff)

static const unsigned short pll_pre_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_FROSC,
};
static const unsigned short a35_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_FROSC,
	dev_cgc1 | IMX8ULP_CLK_SPLL2,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
static const unsigned short nic_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_FROSC,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
static const unsigned short pcc3_periph_bus_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_LPOSC,
	dev_cgc1 | IMX8ULP_CLK_SOSC_DIV2,
	dev_cgc1 | IMX8ULP_CLK_FROSC_DIV2,
	dev_cgc1 | IMX8ULP_CLK_XBAR_DIVBUS,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1_DIV1,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0_DIV2,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0_DIV1,
};
static const unsigned short pcc4_periph_bus_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_LPOSC,
	dev_cgc1 | IMX8ULP_CLK_SOSC_DIV2,
	dev_cgc1 | IMX8ULP_CLK_FROSC_DIV2,
	dev_cgc1 | IMX8ULP_CLK_XBAR_DIVBUS,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_VCODIV,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0_DIV1,
};
static const unsigned short pcc4_periph_plat_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_SOSC_DIV1,
	dev_cgc1 | IMX8ULP_CLK_FROSC_DIV1,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD3_DIV2,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD3_DIV1,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD2_DIV2,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD2_DIV1,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1_DIV2,
};
static const unsigned short pcc5_periph_bus_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_LPOSC,
	dev_cgc1 | IMX8ULP_CLK_SOSC_DIV2,
	dev_cgc1 | IMX8ULP_CLK_FROSC_DIV2,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc2 | IMX8ULP_CLK_PLL4_VCODIV,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD3_DIV1,
};
static const unsigned short pcc5_periph_plat_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD3_DIV2,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD2_DIV2,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD2_DIV1,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1_DIV2,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1_DIV1,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD0_DIV2,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD0_DIV1,
};
static const unsigned short hifi_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_FROSC,
	dev_cgc2 | IMX8ULP_CLK_PLL4,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD0,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
static const unsigned short ddr_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_FROSC,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc2 | IMX8ULP_CLK_PLL4,
	dev_cgc2 | IMX8ULP_CLK_PLL4,
	dev_cgc2 | IMX8ULP_CLK_PLL4,
	dev_cgc2 | IMX8ULP_CLK_PLL4,
};
static const unsigned short lpav_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_FROSC,
	dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
static const unsigned short sai45_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1_DIV1,
	dev_cgc1 | IMX8ULP_CLK_AUD_CLK1,
	dev_cgc2 | IMX8ULP_CLK_AUD_CLK2,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
};
static const unsigned short sai67_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1_DIV1,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_AUD_CLK1,
	dev_cgc2 | IMX8ULP_CLK_AUD_CLK2,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
static const unsigned short aud_clk1_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
static const unsigned short aud_clk2_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
#ifdef CONFIG_DM_ETH
static const unsigned short enet_ts_sels[] = {
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_EXT_TS_SEL,
	dev_cgc1 | IMX8ULP_CLK_ROSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_SOSC,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
	dev_cgc1 | IMX8ULP_CLK_DUMMY,
};
#endif
static const unsigned short xbar_divbus[] = {
	dev_cgc1 | IMX8ULP_CLK_XBAR_DIVBUS,
};
static const unsigned short nic_per_divplat[] = {
	dev_cgc1 | IMX8ULP_CLK_NIC_PER_DIVPLAT,
};
static const unsigned short lpav_axi_div[] = {
	dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV,
};
static const unsigned short lpav_bus_div[] = {
	dev_cgc2 | IMX8ULP_CLK_LPAV_BUS_DIV,
};

static inline void clk_dm1(struct clk8ulp_data *priv, ulong id, struct clk *clk)
{
	if (id >= priv->num_clks) {
		printf("%s:id (%ld) too large\n", __func__, id);
		return;
	}
	if (!IS_ERR(clk))
		clk->id = id + priv->base_id;
	priv->clks[id] = clk;
}

static inline struct clk* imx_clk_obtain_fixed(ofnode np, const char *name)
{
	struct clk clk_tmp;
	struct clk *pclk;
	int ret = clk_get_by_name_nodev(np, name, &clk_tmp);

	if (!ret) {
		pclk = dev_get_clk_ptr(clk_tmp.dev);
		return pclk;
	}
	return NULL;
}

enum {
	TYPE_imx_clk_obtain_fixed = 1,
	TYPE_imx_clk_mux_flags,
	TYPE_imx_clk_mux2,
	TYPE_imx_clk_pllv4,
	TYPE_imx_clk_pfdv2,
	TYPE_imx_clk_gate,
	TYPE_imx_clk_gate_dis,
	TYPE_imx_clk_gate_flags,
	TYPE_imx_clk_divider,
	TYPE_imx_clk_divider_flags,
	TYPE_imx_clk_divider_closest,
	TYPE_imx_clk_composite,
	TYPE_imx_clk_fixed_factor,
};

struct clk8ulp_data *imx8ulp_data[dev_end];

struct imx8ulp_clk_probe {
	unsigned char type;
#define FLAG_mux_present	BIT(0)
#define FLAG_rate_present	BIT(1)
#define FLAG_gate_present	BIT(2)
#define FLAG_swrst		BIT(3)
	unsigned char composite_flags;
	unsigned char *name;
	union {
		const char *parent;
		const unsigned short *parents;
	};
	unsigned num_parents;
	unsigned short base_offset;
	unsigned short parent_dev_clk_id;
	union {
		unsigned char shift;
		unsigned char idx;
		unsigned char mult;
	};
	union {
		unsigned char width;
		unsigned char div;
	};
	unsigned flags;
};

static void imx8ulp_probe(struct clk8ulp_data *priv, unsigned clk_id);

const char* imx8ulp_get_name(struct clk8ulp_data *priv, const struct imx8ulp_clk_probe *cp, unsigned short dci)
{
	unsigned dev_id;
	unsigned clk_id;
	struct clk8ulp_data *p;

	dev_id = DEV_ID(dci);
	clk_id = CLK_ID(dci);

	if (dev_id >= ARRAY_SIZE(imx8ulp_data))
		goto parent_error;
	p = imx8ulp_data[dev_id];

	if (!p) {
		printf("%s: add dependence on dev %d for %d\n", __func__, dev_id, priv->dev_id);
		return NULL;
	}
	if (clk_id >= p->num_clks)
		goto parent_error;

	if (!p->clks[clk_id])
		imx8ulp_probe(p, clk_id);
	if (p->clks[clk_id])
		return p->clks[clk_id]->dev->name;
	printf("%s: parent clk missing %d:%d\n", __func__, dev_id, clk_id);
	return NULL;

parent_error:
	printf("%s: invalid parent id 0x%x %d:%d for %s\n", __func__, dci, dev_id, clk_id, cp->name);
	return NULL;
}

static void imx8ulp_probe(struct clk8ulp_data *priv, unsigned clk_id)
{
	struct clk *clk = NULL;
	const struct imx8ulp_clk_probe *cp = &priv->cps[clk_id];
	const char *parent = NULL;
	const char **parents = NULL;

	if (priv->clks[clk_id]) {
		printf("%s: %s probe not needed\n", __func__, cp->name);
		return;
	}
	if (cp->num_parents) {
		int i;

		parents = kzalloc(cp->num_parents * sizeof(char *), GFP_KERNEL);
		if (!parents) {
			printf("%s: %s oom\n", __func__, cp->name);
			return;
		}
		for (i = 0; i < cp->num_parents; i++) {
			parent = imx8ulp_get_name(priv, cp, cp->parents[i]);
			if (!parent)
				return;
			parents[i] = parent;
		}
		parent = NULL;
	} else if (cp->parent_dev_clk_id) {
		parent = imx8ulp_get_name(priv, cp, cp->parent_dev_clk_id);
		if (!parent)
			return;
	} else {
		struct udevice *parent_dev;
		int ret;

		parent = cp->parent;
		if (parent) {
			ret = uclass_get_device_by_name(UCLASS_CLK, parent, &parent_dev);
			if (ret)
				printf("%s: %s b not found %d\n", __func__, parent, ret);
		}
	}

	switch (cp->type) {
	case TYPE_imx_clk_obtain_fixed:
		clk = imx_clk_obtain_fixed(priv->np, cp->name);
		break;
	case TYPE_imx_clk_mux_flags:
		clk = imx_clk_mux_flags(cp->name, priv->base + cp->base_offset,
			cp->shift, cp->width, parents, cp->num_parents, cp->flags);
		break;
	case TYPE_imx_clk_mux2:
		clk = imx_clk_mux2(cp->name, priv->base + cp->base_offset,
			cp->shift, cp->width, parents, cp->num_parents);
		break;
	case TYPE_imx_clk_pllv4:
		clk = imx_clk_pllv4(IMX_PLLV4_IMX8ULP, cp->name, parent, priv->base + cp->base_offset);
		break;
	case TYPE_imx_clk_pfdv2:
		clk = imx_clk_pfdv2(IMX_PFDV2_IMX8ULP, cp->name, parent, priv->base + cp->base_offset, cp->idx);
		break;
	case TYPE_imx_clk_gate:
		clk = imx_clk_gate(cp->name, parent, priv->base + cp->base_offset, cp->shift);
		break;
	case TYPE_imx_clk_gate_dis:
		clk = imx_clk_gate_dis(cp->name, parent, priv->base + cp->base_offset, cp->shift);
		break;
	case TYPE_imx_clk_gate_flags:
		clk = imx_clk_gate_flags(cp->name, parent, priv->base + cp->base_offset, cp->shift, cp->flags);
		break;
	case TYPE_imx_clk_divider:
		clk = imx_clk_divider(cp->name, parent, priv->base + cp->base_offset, cp->shift, cp->width);
		break;
	case TYPE_imx_clk_divider_flags:
		clk = imx_clk_divider_flags(cp->name, parent, priv->base + cp->base_offset, cp->shift, cp->width, cp->flags);
		break;
	case TYPE_imx_clk_divider_closest:
		clk = imx_clk_divider_closest(cp->name, parent, priv->base + cp->base_offset, cp->shift, cp->width);
		break;
	case TYPE_imx_clk_composite:
		clk = imx8ulp_clk_composite(cp->name, parents, cp->num_parents, (cp->composite_flags & FLAG_mux_present) ? true : false,
			(cp->composite_flags & FLAG_rate_present) ? true : false, (cp->composite_flags & FLAG_gate_present) ? true : false,
				priv->base + cp->base_offset, (cp->composite_flags & FLAG_swrst) ? true : false);
		break;
	case TYPE_imx_clk_fixed_factor:
		clk = imx_clk_fixed_factor(cp->name, parent, cp->mult, cp->div);
		break;
	default:
		printf("%s: clk %d:%d undefined\n", __func__, priv->dev_id, clk_id);
		break;
	}
	if (clk)
		clk_dm1(priv, clk_id, clk);
	return;
}

static const struct imx8ulp_clk_probe imx8ulp_clk_cgc1_init[] = {
[IMX8ULP_CLK_DUMMY		 ] = { .type = TYPE_imx_clk_obtain_fixed, .name = "dummy", },
[IMX8ULP_CLK_ROSC		 ] = { .type = TYPE_imx_clk_obtain_fixed, .name = "rosc", },
[IMX8ULP_CLK_FROSC		 ] = { .type = TYPE_imx_clk_obtain_fixed, .name = "frosc", },
[IMX8ULP_CLK_LPOSC		 ] = { .type = TYPE_imx_clk_obtain_fixed, .name = "lposc", },
[IMX8ULP_CLK_SOSC		 ] = { .type = TYPE_imx_clk_obtain_fixed, .name = "sosc", },
[IMX8ULP_CLK_EXT_TS_SEL		 ] = { .type = TYPE_imx_clk_obtain_fixed, .name = "ext_ts_clk", },

[IMX8ULP_CLK_SPLL2_PRE_SEL	 ] = { .type = TYPE_imx_clk_mux_flags, .name = "spll2_pre_sel", .base_offset = 0x510,
		.shift = 0, .width = 1, .parents = pll_pre_sels, .num_parents = ARRAY_SIZE(pll_pre_sels), .flags = CLK_SET_PARENT_GATE, },
[IMX8ULP_CLK_SPLL3_PRE_SEL	 ] = { .type = TYPE_imx_clk_mux_flags, .name = "spll3_pre_sel", .base_offset = 0x610,
		.shift = 0, .width = 1, .parents = pll_pre_sels, .num_parents = ARRAY_SIZE(pll_pre_sels), .flags = CLK_SET_PARENT_GATE, },

[IMX8ULP_CLK_A35_SEL		 ] = { .type = TYPE_imx_clk_mux2, .name = "a35_sel", .base_offset = 0x14, .shift = 28, .width = 2, .parents = a35_sels, .num_parents = ARRAY_SIZE(a35_sels), },
[IMX8ULP_CLK_NIC_SEL		 ] = { .type = TYPE_imx_clk_mux2, .name = "nic_sel", .base_offset = 0x34, .shift = 28, .width = 2, .parents = nic_sels, .num_parents = ARRAY_SIZE(nic_sels), },
[IMX8ULP_CLK_AUD_CLK1		 ] = { .type = TYPE_imx_clk_mux2, .name = "aud_clk1", .base_offset = 0x900, .shift = 0, .width = 3, .parents = aud_clk1_sels, .num_parents = ARRAY_SIZE(aud_clk1_sels), },
[IMX8ULP_CLK_SAI4_SEL		 ] = { .type = TYPE_imx_clk_mux2, .name = "sai4_sel", .base_offset = 0x904, .shift = 0, .width = 2, .parents = sai45_sels, .num_parents = ARRAY_SIZE(sai45_sels), },
[IMX8ULP_CLK_SAI5_SEL		 ] = { .type = TYPE_imx_clk_mux2, .name = "sai5_sel", .base_offset = 0x904, .shift = 8, .width = 2, .parents = sai45_sels, .num_parents = ARRAY_SIZE(sai45_sels), },
#ifdef CONFIG_DM_ETH
[IMX8ULP_CLK_ENET_TS_SEL	 ] = { .type = TYPE_imx_clk_mux2, .name = "enet_ts", .base_offset = 0x700, 24, 3, .parents = enet_ts_sels, .num_parents = ARRAY_SIZE(enet_ts_sels), },
#endif

[IMX8ULP_CLK_SPLL2		 ] = { .type = TYPE_imx_clk_pllv4, .name = "spll2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL2_PRE_SEL, .base_offset = 0x500, },
[IMX8ULP_CLK_SPLL3		 ] = { .type = TYPE_imx_clk_pllv4, .name = "spll3", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PRE_SEL, .base_offset = 0x600, },

[IMX8ULP_CLK_SPLL3_PFD0		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "spll3_pfd0", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_VCODIV, .base_offset = 0x614, .idx = 0, },
[IMX8ULP_CLK_SPLL3_PFD1		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "spll3_pfd1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_VCODIV, .base_offset = 0x614, .idx = 1, },
[IMX8ULP_CLK_SPLL3_PFD2		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "spll3_pfd2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_VCODIV, .base_offset = 0x614, .idx = 2, },
[IMX8ULP_CLK_SPLL3_PFD3		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "spll3_pfd3", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_VCODIV, .base_offset = 0x614, .idx = 3, },

[IMX8ULP_CLK_SPLL3_PFD0_DIV1_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd0_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0, .base_offset = 0x608, .shift = 7, },
[IMX8ULP_CLK_SPLL3_PFD0_DIV2_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd0_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0, .base_offset = 0x608, .shift = 15, },
[IMX8ULP_CLK_SPLL3_PFD1_DIV1_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd1_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1, .base_offset = 0x608, .shift = 23, },
[IMX8ULP_CLK_SPLL3_PFD1_DIV2_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd1_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1, .base_offset = 0x608, .shift = 31, },
[IMX8ULP_CLK_SPLL3_PFD2_DIV1_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd2_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD2, .base_offset = 0x60c, .shift = 7, },
[IMX8ULP_CLK_SPLL3_PFD2_DIV2_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd2_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD2, .base_offset = 0x60c, .shift = 15, },
[IMX8ULP_CLK_SPLL3_PFD3_DIV1_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd3_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD3, .base_offset = 0x60c, .shift = 23, },
[IMX8ULP_CLK_SPLL3_PFD3_DIV2_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "spll3_pfd3_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD3, .base_offset = 0x60c, .shift = 31, },
[IMX8ULP_CLK_SOSC_DIV1_GATE	 ] = { .type = TYPE_imx_clk_gate_dis, .name = "sosc_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC, .base_offset = 0x108, .shift = 7, },
[IMX8ULP_CLK_SOSC_DIV2_GATE	 ] = { .type = TYPE_imx_clk_gate_dis, .name = "sosc_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC, .base_offset = 0x108, .shift = 15, },
[IMX8ULP_CLK_SOSC_DIV3_GATE	 ] = { .type = TYPE_imx_clk_gate_dis, .name = "sosc_div3_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC, .base_offset = 0x108, .shift = 23, },
[IMX8ULP_CLK_FROSC_DIV1_GATE	 ] = { .type = TYPE_imx_clk_gate_dis, .name = "frosc_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC, .base_offset = 0x208, .shift = 7, },
[IMX8ULP_CLK_FROSC_DIV2_GATE	 ] = { .type = TYPE_imx_clk_gate_dis, .name = "frosc_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC, .base_offset = 0x208, .shift = 15, },
[IMX8ULP_CLK_FROSC_DIV3_GATE	 ] = { .type = TYPE_imx_clk_gate_dis, .name = "frosc_div3_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC, .base_offset = 0x208, .shift = 23, },

[IMX8ULP_CLK_SPLL3_VCODIV	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_vcodiv", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3, .base_offset = 0x604, .shift = 0, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD0_DIV1	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd0_div1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0_DIV1_GATE, .base_offset = 0x608, .shift = 0, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD0_DIV2	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd0_div2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD0_DIV2_GATE, .base_offset = 0x608, .shift = 8, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD1_DIV1	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd1_div1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1_DIV1_GATE, .base_offset = 0x608, .shift = 16, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD1_DIV2	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd1_div2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD1_DIV2_GATE, .base_offset = 0x608, .shift = 24, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD2_DIV1	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd2_div1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD2_DIV1_GATE, .base_offset = 0x60c, .shift = 0, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD2_DIV2	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd2_div2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD2_DIV2_GATE, .base_offset = 0x60c, .shift = 8, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD3_DIV1	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd3_div1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD3_DIV1_GATE, .base_offset = 0x60c, .shift = 16, .width = 6, },
[IMX8ULP_CLK_SPLL3_PFD3_DIV2	 ] = { .type = TYPE_imx_clk_divider, .name = "spll3_pfd3_div2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SPLL3_PFD3_DIV2_GATE, .base_offset = 0x60c, .shift = 24, .width = 6, },
[IMX8ULP_CLK_SOSC_DIV1		 ] = { .type = TYPE_imx_clk_divider, .name = "sosc_div1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC_DIV1_GATE, .base_offset = 0x108, .shift = 0, .width = 6, },
[IMX8ULP_CLK_SOSC_DIV2		 ] = { .type = TYPE_imx_clk_divider, .name = "sosc_div2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC_DIV2_GATE, .base_offset = 0x108, .shift = 8, .width = 6, },
[IMX8ULP_CLK_SOSC_DIV3		 ] = { .type = TYPE_imx_clk_divider, .name = "sosc_div3", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC_DIV3_GATE, .base_offset = 0x108, .shift = 16, .width = 6, },
[IMX8ULP_CLK_FROSC_DIV1		 ] = { .type = TYPE_imx_clk_divider, .name = "frosc_div1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC_DIV1_GATE, .base_offset = 0x208, .shift = 0, .width = 6, },
[IMX8ULP_CLK_FROSC_DIV2		 ] = { .type = TYPE_imx_clk_divider, .name = "frosc_div2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC_DIV2_GATE, .base_offset = 0x208, .shift = 8, .width = 6, },
[IMX8ULP_CLK_FROSC_DIV3		 ] = { .type = TYPE_imx_clk_divider, .name = "frosc_div3", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC_DIV3_GATE, .base_offset = 0x208, .shift = 16, .width = 6, },

[IMX8ULP_CLK_A35_DIV		 ] = { .type = TYPE_imx_clk_divider_flags, .name = "a35_div", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_A35_SEL, .base_offset = 0x14, .shift = 21, .width = 6, .flags = CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, },
[IMX8ULP_CLK_NIC_AD_DIVPLAT	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "nic_ad_divplat", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_NIC_SEL, .base_offset = 0x34, .shift = 21, .width = 6, .flags = CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, },
[IMX8ULP_CLK_NIC_PER_DIVPLAT	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "nic_per_divplat", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_NIC_AD_DIVPLAT, .base_offset = 0x34, .shift = 14, .width = 6, .flags = CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, },
[IMX8ULP_CLK_XBAR_AD_DIVPLAT	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "xbar_ad_divplat", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_NIC_AD_DIVPLAT, .base_offset = 0x38, .shift = 14, .width = 6, .flags = CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, },
[IMX8ULP_CLK_XBAR_DIVBUS	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "xbar_divbus", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_NIC_AD_DIVPLAT, .base_offset = 0x38, .shift = 7, .width = 6, .flags = CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, },
[IMX8ULP_CLK_XBAR_AD_SLOW	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "xbar_ad_slow", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_NIC_AD_DIVPLAT, .base_offset = 0x38, .shift = 0, .width = 6, .flags = CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, },
};

static const struct imx8ulp_clk_probe imx8ulp_clk_cgc2_init[] = {
[IMX8ULP_CLK_DSI_PHY_REF	 ] = { .type = TYPE_imx_clk_obtain_fixed, .name = "dsi_phy_ref", },

[IMX8ULP_CLK_PLL4_PRE_SEL	 ] = { .type = TYPE_imx_clk_mux_flags, .name = "pll4_pre_sel", .base_offset = 0x610,
	.shift = 0, .width = 1, .parents = pll_pre_sels, .num_parents = ARRAY_SIZE(pll_pre_sels), .flags = CLK_SET_PARENT_GATE, },
[IMX8ULP_CLK_HIFI_SEL		 ] = { .type = TYPE_imx_clk_mux_flags, .name = "hifi_sel", .base_offset = 0x14,
	.shift = 28, .width = 3, .parents = hifi_sels, .num_parents = ARRAY_SIZE(hifi_sels), .flags = CLK_SET_PARENT_GATE, },
[IMX8ULP_CLK_DDR_SEL		 ] = { .type = TYPE_imx_clk_mux_flags, .name = "ddr_sel", .base_offset = 0x40,
	.shift = 28, .width = 3, .parents = ddr_sels, .num_parents = ARRAY_SIZE(ddr_sels), .flags = CLK_GET_RATE_NOCACHE, },
[IMX8ULP_CLK_LPAV_AXI_SEL	 ] = { .type = TYPE_imx_clk_mux_flags, .name = "lpav_sel", .base_offset = 0x3c,
	.shift = 28, .width = 2, .parents = lpav_sels, .num_parents = ARRAY_SIZE(lpav_sels), .flags = CLK_SET_PARENT_GATE, },

[IMX8ULP_CLK_AUD_CLK2		 ] = { .type = TYPE_imx_clk_mux2, .name = "aud_clk2", .base_offset = 0x900, .shift = 0, .width = 3, .parents = aud_clk2_sels, .num_parents = ARRAY_SIZE(aud_clk2_sels), },
[IMX8ULP_CLK_SAI6_SEL		 ] = { .type = TYPE_imx_clk_mux2, .name = "sai6_sel", .base_offset = 0x904, .shift = 0, .width = 3, .parents = sai67_sels, .num_parents = ARRAY_SIZE(sai67_sels), },
[IMX8ULP_CLK_SAI7_SEL		 ] = { .type = TYPE_imx_clk_mux2, .name = "sai7_sel", .base_offset = 0x904, .shift = 8, .width = 3, .parents = sai67_sels, .num_parents = ARRAY_SIZE(sai67_sels), },
[IMX8ULP_CLK_SPDIF_SEL		 ] = { .type = TYPE_imx_clk_mux2, .name = "spdif_sel", .base_offset = 0x910, .shift = 0, .width = 3, .parents = sai67_sels, .num_parents = ARRAY_SIZE(sai67_sels), },

[IMX8ULP_CLK_PLL4		 ] = { .type = TYPE_imx_clk_pllv4, .name = "pll4", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PRE_SEL, .base_offset = 0x600, },

[IMX8ULP_CLK_PLL4_PFD0		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "pll4_pfd0", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_VCODIV, .base_offset = 0x614, .idx = 0, },
[IMX8ULP_CLK_PLL4_PFD1		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "pll4_pfd1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_VCODIV, .base_offset = 0x614, .idx = 1, },
[IMX8ULP_CLK_PLL4_PFD2		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "pll4_pfd2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_VCODIV, .base_offset = 0x614, .idx = 2, },
[IMX8ULP_CLK_PLL4_PFD3		 ] = { .type = TYPE_imx_clk_pfdv2, .name = "pll4_pfd3", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_VCODIV, .base_offset = 0x614, .idx = 3, },

[IMX8ULP_CLK_PLL4_PFD0_DIV1_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd0_div1_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD0, .base_offset = 0x608, .shift = 7, },
[IMX8ULP_CLK_PLL4_PFD0_DIV2_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd0_div2_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD0, .base_offset = 0x608, .shift = 15, },
[IMX8ULP_CLK_PLL4_PFD1_DIV1_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd1_div1_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1, .base_offset = 0x608, .shift = 23, },
[IMX8ULP_CLK_PLL4_PFD1_DIV2_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd1_div2_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1, .base_offset = 0x608, .shift = 31, },
[IMX8ULP_CLK_PLL4_PFD2_DIV1_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd2_div1_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD2, .base_offset = 0x60c, .shift = 7, },
[IMX8ULP_CLK_PLL4_PFD2_DIV2_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd2_div2_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD2, .base_offset = 0x60c, .shift = 15, },
[IMX8ULP_CLK_PLL4_PFD3_DIV1_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd3_div1_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD3, .base_offset = 0x60c, .shift = 23, },
[IMX8ULP_CLK_PLL4_PFD3_DIV2_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "pll4_pfd3_div2_gate", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD3, .base_offset = 0x60c, .shift = 31, },
[IMX8ULP_CLK_CGC2_SOSC_DIV1_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "cgc2_sosc_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC, .base_offset = 0x108, .shift = 7, },
[IMX8ULP_CLK_CGC2_SOSC_DIV2_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "cgc2_sosc_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC, .base_offset = 0x108, .shift = 15, },
[IMX8ULP_CLK_CGC2_SOSC_DIV3_GATE ] = { .type = TYPE_imx_clk_gate_dis, .name = "cgc2_sosc_div3_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC, .base_offset = 0x108, .shift = 23, },
[IMX8ULP_CLK_CGC2_FROSC_DIV1_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "cgc2_frosc_div1_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC, .base_offset = 0x208, .shift = 7, },
[IMX8ULP_CLK_CGC2_FROSC_DIV1_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "cgc2_frosc_div2_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC, .base_offset = 0x208, .shift = 15, },
[IMX8ULP_CLK_CGC2_FROSC_DIV3_GATE] = { .type = TYPE_imx_clk_gate_dis, .name = "cgc2_frosc_div3_gate", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_FROSC, .base_offset = 0x208, .shift = 23, },

[IMX8ULP_CLK_PLL4_VCODIV	 ] = { .type = TYPE_imx_clk_divider, .name = "pll4_vcodiv", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4, .base_offset = 0x604, .shift = 0, .width = 6, },
[IMX8ULP_CLK_HIFI_DIVCORE	 ] = { .type = TYPE_imx_clk_divider, .name = "hifi_core_div", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_HIFI_SEL, .base_offset = 0x14, .shift = 21, .width = 6, },
[IMX8ULP_CLK_HIFI_DIVPLAT	 ] = { .type = TYPE_imx_clk_divider, .name = "hifi_plat_div", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_HIFI_DIVCORE, .base_offset = 0x14, .shift = 14, .width = 6, },
[IMX8ULP_CLK_CGC2_SOSC_DIV1	 ] = { .type = TYPE_imx_clk_divider, .name = "cgc2_sosc_div1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_CGC2_SOSC_DIV1_GATE, .base_offset = 0x108, .shift = 0, .width = 6, },
[IMX8ULP_CLK_CGC2_SOSC_DIV2	 ] = { .type = TYPE_imx_clk_divider, .name = "cgc2_sosc_div2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_CGC2_SOSC_DIV2_GATE, .base_offset = 0x108, .shift = 8, .width = 6, },
[IMX8ULP_CLK_CGC2_SOSC_DIV3	 ] = { .type = TYPE_imx_clk_divider, .name = "cgc2_sosc_div3", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_CGC2_SOSC_DIV3_GATE, .base_offset = 0x108, .shift = 16, .width = 6, },
[IMX8ULP_CLK_CGC2_FROSC_DIV1	 ] = { .type = TYPE_imx_clk_divider, .name = "cgc2_frosc_div1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_CGC2_FROSC_DIV1_GATE, .base_offset = 0x208, .shift = 0, .width = 6, },
[IMX8ULP_CLK_CGC2_FROSC_DIV2	 ] = { .type = TYPE_imx_clk_divider, .name = "cgc2_frosc_div2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_CGC2_FROSC_DIV1_GATE, .base_offset = 0x208, .shift = 8, .width = 6, },
[IMX8ULP_CLK_CGC2_FROSC_DIV3	 ] = { .type = TYPE_imx_clk_divider, .name = "cgc2_frosc_div3", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_CGC2_FROSC_DIV3_GATE, .base_offset = 0x208, .shift = 16, .width = 6, },

[IMX8ULP_CLK_DDR_DIV		 ] = { .type = TYPE_imx_clk_divider_flags, .name = "ddr_div", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_DDR_SEL, .base_offset = 0x40, .shift = 21, .width = 6, .flags = CLK_IS_CRITICAL | CLK_GET_RATE_NOCACHE, },
[IMX8ULP_CLK_LPAV_AXI_DIV	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "lpav_axi_div", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_SEL, .base_offset = 0x3c, .shift = 21, .width = 6, .flags = CLK_IS_CRITICAL, },
[IMX8ULP_CLK_LPAV_AHB_DIV	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "lpav_ahb_div", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x3c, .shift = 14, .width = 6, .flags = CLK_IS_CRITICAL, },
[IMX8ULP_CLK_LPAV_BUS_DIV	 ] = { .type = TYPE_imx_clk_divider_flags, .name = "lpav_bus_div", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x3c, .shift = 7, .width = 6, .flags = CLK_IS_CRITICAL, },

[IMX8ULP_CLK_PLL4_PFD0_DIV1	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd0_div1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD0_DIV1_GATE, .base_offset = 0x608, .shift = 0, .width = 6, },
[IMX8ULP_CLK_PLL4_PFD0_DIV2	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd0_div2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD0_DIV2_GATE, .base_offset = 0x608, .shift = 8, .width = 6, },
[IMX8ULP_CLK_PLL4_PFD1_DIV1	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd1_div1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1_DIV1_GATE, .base_offset = 0x608, .shift = 16, .width = 6, },
[IMX8ULP_CLK_PLL4_PFD1_DIV2	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd1_div2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD1_DIV2_GATE, .base_offset = 0x608, .shift = 24, .width = 6, },
[IMX8ULP_CLK_PLL4_PFD2_DIV1	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd2_div1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD2_DIV1_GATE, .base_offset = 0x60c, .shift = 0, .width = 6, },
[IMX8ULP_CLK_PLL4_PFD2_DIV2	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd2_div2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD2_DIV2_GATE, .base_offset = 0x60c, .shift = 8, .width = 6, },
[IMX8ULP_CLK_PLL4_PFD3_DIV1	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd3_div1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD3_DIV1_GATE, .base_offset = 0x60c, .shift = 16, .width = 6, },
[IMX8ULP_CLK_PLL4_PFD3_DIV2	 ] = { .type = TYPE_imx_clk_divider_closest, .name = "pll4_pfd3_div2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_PLL4_PFD3_DIV2_GATE, .base_offset = 0x60c, .shift = 24, .width = 6, },
};

static const struct imx8ulp_clk_probe imx8ulp_clk_pcc3_init[] = {
[IMX8ULP_CLK_WDOG3	 ] = { .type = TYPE_imx_clk_composite, .name = "wdog3", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xa8, },
[IMX8ULP_CLK_WDOG4	 ] = { .type = TYPE_imx_clk_composite, .name = "wdog4", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xac, },
[IMX8ULP_CLK_LPIT1	 ] = { .type = TYPE_imx_clk_composite, .name = "lpit1", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xc8, },
[IMX8ULP_CLK_TPM4	 ] = { .type = TYPE_imx_clk_composite, .name = "tpm4", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xcc, },
[IMX8ULP_CLK_FLEXIO1	 ] = { .type = TYPE_imx_clk_composite, .name = "flexio1", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xd4, },
[IMX8ULP_CLK_I3C2	 ] = { .type = TYPE_imx_clk_composite, .name = "i3c2", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xd8, },
[IMX8ULP_CLK_LPI2C4	 ] = { .type = TYPE_imx_clk_composite, .name = "lpi2c4", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xdc, },
[IMX8ULP_CLK_LPI2C5	 ] = { .type = TYPE_imx_clk_composite, .name = "lpi2c5", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xe0, },
[IMX8ULP_CLK_LPUART4	 ] = { .type = TYPE_imx_clk_composite, .name = "lpuart4", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xe4, },
[IMX8ULP_CLK_LPUART5	 ] = { .type = TYPE_imx_clk_composite, .name = "lpuart5", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xe8, },
[IMX8ULP_CLK_LPSPI4	 ] = { .type = TYPE_imx_clk_composite, .name = "lpspi4", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xec, },
[IMX8ULP_CLK_LPSPI5	 ] = { .type = TYPE_imx_clk_composite, .name = "lpspi5", .parents = pcc3_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc3_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xf0, },

[IMX8ULP_CLK_DMA1_MP	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_mp", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x4, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH0	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch0", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x8, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH1	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch1", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0xc, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH2	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch2", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x10, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH3	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch3", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x14, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH4	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch4", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x18, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH5	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch5", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x1c, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH6	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch6", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x20, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH7	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch7", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x24, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH8	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch8", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x28, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH9	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch9", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x2c, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH10	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch10", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x30, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH11	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch11", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x34, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH12	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch12", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x38, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH13	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch13", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x3c, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH14	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch14", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x40, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH15	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch15", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x44, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH16	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch16", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x48, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH17	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch17", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x4c, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH18	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch18", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x50, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH19	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch19", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x54, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH20	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch20", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x58, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH21	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch21", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x5c, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH22	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch22", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x60, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH23	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch23", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x64, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH24	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch24", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x68, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH25	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch25", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x6c, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH26	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch26", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x70, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH27	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch27", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x74, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH28	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch28", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x78, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH29	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch29", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x7c, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH30	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch30", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x80, .shift = 30, },
[IMX8ULP_CLK_DMA1_CH31	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma1_ch31", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x84, .shift = 30, },
[IMX8ULP_CLK_MU3_A	 ] = { .type = TYPE_imx_clk_gate, .name = "mu3_a", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x8c, .shift = 30, },

[IMX8ULP_CLK_MU0_B	 ] = { .type = TYPE_imx_clk_gate_flags, .name = "mu0_b", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_AD_DIVPLAT, .base_offset = 0x88, .shift = 30, .flags = CLK_IS_CRITICAL, },
[IMX8ULP_CLK_TPM5	 ] = { .type = TYPE_imx_clk_gate_flags, .name = "tpm5", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_SOSC_DIV2,  .base_offset = 0xd0, .shift = 30, .flags = CLK_IS_CRITICAL, },
};

static const struct imx8ulp_clk_probe imx8ulp_clk_pcc4_init[] = {
[IMX8ULP_CLK_FLEXSPI2	 ] = { .type = TYPE_imx_clk_composite, .name = "flexspi2", .parents = pcc4_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc4_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x4, },
[IMX8ULP_CLK_TPM6	 ] = { .type = TYPE_imx_clk_composite, .name = "tpm6", .parents = pcc4_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc4_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x8, },
[IMX8ULP_CLK_TPM7	 ] = { .type = TYPE_imx_clk_composite, .name = "tpm7", .parents = pcc4_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc4_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xc, },
[IMX8ULP_CLK_LPI2C6	 ] = { .type = TYPE_imx_clk_composite, .name = "lpi2c6", .parents = pcc4_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc4_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x10, },
[IMX8ULP_CLK_LPI2C7	 ] = { .type = TYPE_imx_clk_composite, .name = "lpi2c7", .parents = pcc4_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc4_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x14, },
[IMX8ULP_CLK_LPUART6	 ] = { .type = TYPE_imx_clk_composite, .name = "lpuart6", .parents = pcc4_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc4_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x18, },
[IMX8ULP_CLK_LPUART7	 ] = { .type = TYPE_imx_clk_composite, .name = "lpuart7", .parents = pcc4_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc4_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x1c, },
[IMX8ULP_CLK_SAI4	 ] = { .type = TYPE_imx_clk_composite, .name = "sai4", .parents = xbar_divbus, .num_parents = ARRAY_SIZE(xbar_divbus), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0x20, }, /* sai ipg, NOT from sai sel */
[IMX8ULP_CLK_SAI5	 ] = { .type = TYPE_imx_clk_composite, .name = "sai5", .parents = xbar_divbus, .num_parents = ARRAY_SIZE(xbar_divbus), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0x24, }, /* sai ipg */
[IMX8ULP_CLK_USDHC0	 ] = { .type = TYPE_imx_clk_composite, .name = "usdhc0", .parents = pcc4_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc4_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x34, },
[IMX8ULP_CLK_USDHC1	 ] = { .type = TYPE_imx_clk_composite, .name = "usdhc1", .parents = pcc4_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc4_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x38, },
[IMX8ULP_CLK_USDHC2	 ] = { .type = TYPE_imx_clk_composite, .name = "usdhc2", .parents = pcc4_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc4_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x3c, },
[IMX8ULP_CLK_USB0	 ] = { .type = TYPE_imx_clk_composite, .name = "usb0", .parents = nic_per_divplat, .num_parents = ARRAY_SIZE(nic_per_divplat), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0x40, },
[IMX8ULP_CLK_USB0_PHY	 ] = { .type = TYPE_imx_clk_composite, .name = "usb0_phy", .parents = xbar_divbus, .num_parents = ARRAY_SIZE(xbar_divbus), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0x44, },
[IMX8ULP_CLK_USB1	 ] = { .type = TYPE_imx_clk_composite, .name = "usb1", .parents = nic_per_divplat, .num_parents = ARRAY_SIZE(nic_per_divplat), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0x48, },
[IMX8ULP_CLK_USB1_PHY	 ] = { .type = TYPE_imx_clk_composite, .name = "usb1_phy", .parents = xbar_divbus, .num_parents = ARRAY_SIZE(xbar_divbus), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0x4c, },
#ifdef CONFIG_DM_ETH
[IMX8ULP_CLK_ENET	 ] = { .type = TYPE_imx_clk_composite, .name = "enet", .parents = nic_per_divplat, .num_parents = ARRAY_SIZE(nic_per_divplat), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0x54, },
#endif

[IMX8ULP_CLK_PCTLE	 ] = { .type = TYPE_imx_clk_gate, .name = "pctle", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_DIVBUS, .base_offset = 0x28, .shift = 30, },
[IMX8ULP_CLK_PCTLF	 ] = { .type = TYPE_imx_clk_gate, .name = "pctlf", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_DIVBUS, .base_offset = 0x2c, .shift = 30, },
[IMX8ULP_CLK_USB_XBAR	 ] = { .type = TYPE_imx_clk_gate, .name = "usb_xbar", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_XBAR_DIVBUS, .base_offset = 0x50, .shift = 30, },
[IMX8ULP_CLK_RGPIOE	 ] = { .type = TYPE_imx_clk_gate, .name = "rgpioe", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_NIC_PER_DIVPLAT, .base_offset = 0x78, .shift = 30, },
[IMX8ULP_CLK_RGPIOF	 ] = { .type = TYPE_imx_clk_gate, .name = "rgpiof", .parent_dev_clk_id = dev_cgc1 | IMX8ULP_CLK_NIC_PER_DIVPLAT, .base_offset = 0x7c, .shift = 30, },
};

static const struct imx8ulp_clk_probe imx8ulp_clk_pcc5_init[] = {
[IMX8ULP_CLK_DMA2_MP	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_mp", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x0, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH0	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch0", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x4, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH1	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch1", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x8, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH2	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch2", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0xc, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH3	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch3", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x10, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH4	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch4", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x14, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH5	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch5", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x18, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH6	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch6", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x1c, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH7	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch7", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x20, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH8	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch8", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x24, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH9	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch9", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x28, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH10	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch10", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x2c, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH11	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch11", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x30, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH12	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch12", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x34, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH13	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch13", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x38, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH14	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch14", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x3c, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH15	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch15", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x40, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH16	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch16", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x44, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH17	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch17", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x48, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH18	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch18", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x4c, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH19	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch19", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x50, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH20	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch20", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x54, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH21	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch21", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x58, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH22	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch22", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x5c, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH23	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch23", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x60, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH24	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch24", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x64, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH25	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch25", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x68, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH26	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch26", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x6c, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH27	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch27", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x70, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH28	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch28", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x74, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH29	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch29", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x78, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH30	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch30", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x7c, .shift = 30, },
[IMX8ULP_CLK_DMA2_CH31	 ] = { .type = TYPE_imx_clk_gate, .name = "pcc_dma2_ch31", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x80, .shift = 30, },
[IMX8ULP_CLK_AVD_SIM	 ] = { .type = TYPE_imx_clk_gate, .name = "avd_sim", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_BUS_DIV, .base_offset = 0x94, .shift = 30, },
[IMX8ULP_CLK_MU2_B	 ] = { .type = TYPE_imx_clk_gate, .name = "mu2_b", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_BUS_DIV, .base_offset = 0x84, .shift = 30, },
[IMX8ULP_CLK_MU3_B	 ] = { .type = TYPE_imx_clk_gate, .name = "mu3_b", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_BUS_DIV, .base_offset = 0x88, .shift = 30, },
[IMX8ULP_CLK_RGPIOD	 ] = { .type = TYPE_imx_clk_gate, .name = "rgpiod", .parent_dev_clk_id = dev_cgc2 | IMX8ULP_CLK_LPAV_AXI_DIV, .base_offset = 0x114, .shift = 30, },

[IMX8ULP_CLK_TPM8	 ] = { .type = TYPE_imx_clk_composite, .name = "tpm8", .parents = pcc5_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc5_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xa0, },
[IMX8ULP_CLK_SAI6	 ] = { .type = TYPE_imx_clk_composite, .name = "sai6", .parents = lpav_bus_div, .num_parents = ARRAY_SIZE(lpav_bus_div), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0xa4, },
[IMX8ULP_CLK_SAI7	 ] = { .type = TYPE_imx_clk_composite, .name = "sai7", .parents = lpav_bus_div, .num_parents = ARRAY_SIZE(lpav_bus_div), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0xa8, },
[IMX8ULP_CLK_SPDIF	 ] = { .type = TYPE_imx_clk_composite, .name = "spdif", .parents = lpav_bus_div, .num_parents = ARRAY_SIZE(lpav_bus_div), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0xac, },
[IMX8ULP_CLK_ISI	 ] = { .type = TYPE_imx_clk_composite, .name = "isi", .parents = lpav_axi_div, .num_parents = ARRAY_SIZE(lpav_axi_div), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0xb0, },
[IMX8ULP_CLK_CSI_REGS	 ] = { .type = TYPE_imx_clk_composite, .name = "csi_regs", .parents = lpav_bus_div, .num_parents = ARRAY_SIZE(lpav_bus_div), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0xb4, },
[IMX8ULP_CLK_CSI	 ] = { .type = TYPE_imx_clk_composite, .name = "csi", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xbc, },
[IMX8ULP_CLK_DSI	 ] = { .type = TYPE_imx_clk_composite, .name = "dsi", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xc0, },
[IMX8ULP_CLK_WDOG5	 ] = { .type = TYPE_imx_clk_composite, .name = "wdog5", .parents = pcc5_periph_bus_sels, .num_parents = ARRAY_SIZE(pcc5_periph_bus_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xc8, },
[IMX8ULP_CLK_EPDC	 ] = { .type = TYPE_imx_clk_composite, .name = "epdc", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xcc, },
[IMX8ULP_CLK_PXP	 ] = { .type = TYPE_imx_clk_composite, .name = "pxp", .parents = lpav_axi_div, .num_parents = ARRAY_SIZE(lpav_axi_div), .composite_flags = FLAG_gate_present | FLAG_swrst, .base_offset = 0xd0, },
[IMX8ULP_CLK_GPU2D	 ] = { .type = TYPE_imx_clk_composite, .name = "gpu2d", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xf0, },
[IMX8ULP_CLK_GPU3D	 ] = { .type = TYPE_imx_clk_composite, .name = "gpu3d", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xf4, },
[IMX8ULP_CLK_DC_NANO	 ] = { .type = TYPE_imx_clk_composite, .name = "dc_nano", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0xf8, },
[IMX8ULP_CLK_CSI_CLK_UI	 ] = { .type = TYPE_imx_clk_composite, .name = "csi_clk_ui", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x10c, },
[IMX8ULP_CLK_CSI_CLK_ESC ] = { .type = TYPE_imx_clk_composite, .name = "csi_clk_esc", .parents = pcc5_periph_plat_sels, .num_parents = ARRAY_SIZE(pcc5_periph_plat_sels), .composite_flags = FLAG_mux_present | FLAG_rate_present | FLAG_gate_present | FLAG_swrst, .base_offset = 0x110, },

[IMX8ULP_CLK_DSI_TX_ESC	 ] = { .type = TYPE_imx_clk_fixed_factor, .name = "mipi_dsi_tx_esc", .parent_dev_clk_id = dev_pcc5 | IMX8ULP_CLK_DSI, .mult = 1, .div = 4, },
};

struct imx8ulp_clk_init {
	int	dev_id;
	int	base_id;
	int	num_clks;
	const struct imx8ulp_clk_probe *cps;
};

static int imx8ulp_clk_ofdata_to_platdata(struct udevice *dev)
{
	struct clk8ulp_data *priv = dev_get_priv(dev);
	struct imx8ulp_clk_init *pdata;

	pdata = (struct imx8ulp_clk_init *)dev->driver_data;
	priv->np = dev_ofnode(dev);
	priv->base = (void *)dev_read_addr(dev);
	debug("%s: base=%lx\n", __func__, (long)priv->base);
	priv->base_id = pdata->base_id;
	priv->dev_id = pdata->dev_id;
	priv->num_clks = pdata->num_clks;
	priv->cps = pdata->cps;
	if (priv->clks)
		printf("%s: already allocated\n", __func__);
	else
		priv->clks = kzalloc(sizeof(struct clk *) * priv->num_clks, GFP_KERNEL);

	if (imx8ulp_data[priv->dev_id])
		printf("%s: already set %d %p %p\n", __func__, priv->dev_id, imx8ulp_data[priv->dev_id], priv);
	imx8ulp_data[priv->dev_id] = priv;

	return 0;
}

static int imx8ulp_clk_probe(struct udevice *dev)
{
	struct clk_bulk clks;
	int ret;

	ret = clk_get_bulk(dev, &clks);

	return 0;
}

const struct imx8ulp_clk_init imx8ulp_cgc1 = {
	.dev_id = DEV_ID(dev_cgc1),
	.base_id = 0,
	.num_clks = IMX8ULP_CLK_CGC1_END,
	.cps = imx8ulp_clk_cgc1_init,
};

const struct imx8ulp_clk_init imx8ulp_cgc2 = {
	.dev_id = DEV_ID(dev_cgc2),
	.base_id = IMX8ULP_CLK_CGC1_END,
	.num_clks = IMX8ULP_CLK_CGC2_END,
	.cps = imx8ulp_clk_cgc2_init,
};

const struct imx8ulp_clk_init imx8ulp_pcc3 = {
	.dev_id = DEV_ID(dev_pcc3),
	.base_id = IMX8ULP_CLK_CGC1_END + IMX8ULP_CLK_CGC2_END,
	.num_clks = IMX8ULP_CLK_PCC3_END,
	.cps = imx8ulp_clk_pcc3_init,
};

const struct imx8ulp_clk_init imx8ulp_pcc4 = {
	.dev_id = DEV_ID(dev_pcc4),
	.base_id = IMX8ULP_CLK_CGC1_END + IMX8ULP_CLK_CGC2_END + IMX8ULP_CLK_PCC3_END,
	.num_clks = IMX8ULP_CLK_PCC4_END,
	.cps = imx8ulp_clk_pcc4_init,
};

const struct imx8ulp_clk_init imx8ulp_pcc5 = {
	.dev_id = DEV_ID(dev_pcc5),
	.base_id = IMX8ULP_CLK_CGC1_END + IMX8ULP_CLK_CGC2_END + IMX8ULP_CLK_PCC3_END + IMX8ULP_CLK_PCC4_END,
	.num_clks = IMX8ULP_CLK_PCC5_END,
	.cps = imx8ulp_clk_pcc5_init,
};

static const struct udevice_id imx8ulp_clk_dt_ids[] = {
	{ .compatible = "fsl,imx8ulp-cgc1", .data = (long)&imx8ulp_cgc1 },
	{ .compatible = "fsl,imx8ulp-cgc2", .data = (long)&imx8ulp_cgc2 },
	{ .compatible = "fsl,imx8ulp-pcc3", .data = (long)&imx8ulp_pcc3 },
	{ .compatible = "fsl,imx8ulp-pcc4", .data = (long)&imx8ulp_pcc4 },
	{ .compatible = "fsl,imx8ulp-pcc5", .data = (long)&imx8ulp_pcc5 },
	{ /* sentinel */ },
};

static int imx8ulp_of_xlate(struct clk *clk,
				struct ofnode_phandle_args *args)
{
	struct clk8ulp_data *priv;
	unsigned i;

	if (!clk)
		return -EINVAL;
	if (!clk->dev)
		return -EINVAL;
	priv = dev_get_priv(clk->dev);
	if (!priv)
		return -EINVAL;
	if (args->args_count > 1) {
		debug("Invalid args_count: %d\n", args->args_count);
		return -EINVAL;
	}

	clk->id = priv->base_id;
	if (args->args_count) {
		i = args->args[0];
		if (i < priv->num_clks) {
			clk->id += i;
			if (!priv->clks)
				return -EINVAL;
			if (!priv->clks[i])
				imx8ulp_probe(priv, i);
		} else if (priv->num_clks) {
			printf("%s: error, too big %d >= %d %s\n", __func__, args->args[0], priv->num_clks, dev_read_name(clk->dev));
		}
	}

	clk->data = 0;

	return 0;
}

static const struct clk_ops imx8ulp_clk_ops = {
	.of_xlate = imx8ulp_of_xlate,
	.round_rate = ccf_clk_round_rate,
	.set_rate = ccf_clk_set_rate,
	.get_rate = ccf_clk_get_rate,
	.set_parent = ccf_clk_set_parent,
	.enable = ccf_clk_enable,
	.disable = ccf_clk_disable,
};

U_BOOT_DRIVER(imx8ulp_clk) = {
	.name = "clk_imx8ulp",
	.id = UCLASS_CLK,
	.of_match = imx8ulp_clk_dt_ids,
	.of_to_plat = imx8ulp_clk_ofdata_to_platdata,
	.ops = &imx8ulp_clk_ops,
	.probe = imx8ulp_clk_probe,
	.flags = DM_FLAG_PRE_RELOC,
        .priv_auto = sizeof(struct clk8ulp_data),
};
