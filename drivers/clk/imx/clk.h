// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 DENX Software Engineering
 * Lukasz Majewski, DENX Software Engineering, lukma@denx.de
 */
#ifndef __MACH_IMX_CLK_H
#define __MACH_IMX_CLK_H

#include <linux/clk-provider.h>

enum imx_pllv3_type {
	IMX_PLLV3_GENERIC,
	IMX_PLLV3_SYS,
	IMX_PLLV3_USB,
	IMX_PLLV3_USB_VF610,
	IMX_PLLV3_AV,
	IMX_PLLV3_ENET,
	IMX_PLLV3_ENET_IMX7,
	IMX_PLLV3_SYS_VF610,
	IMX_PLLV3_DDR_IMX7,
};

enum imx_pll14xx_type {
	PLL_1416X,
	PLL_1443X,
};

/* NOTE: Rate table should be kept sorted in descending order. */
struct imx_pll14xx_rate_table {
	unsigned int rate;
	unsigned int pdiv;
	unsigned int mdiv;
	unsigned int sdiv;
	unsigned int kdiv;
};

struct imx_pll14xx_clk {
	enum imx_pll14xx_type type;
	const struct imx_pll14xx_rate_table *rate_table;
	int rate_count;
	int flags;
};

extern struct imx_pll14xx_clk imx_1416x_pll;
extern struct imx_pll14xx_clk imx_1443x_pll;
extern struct imx_pll14xx_clk imx_1443x_dram_pll;
struct clk *imx_clk_cpu(const char *name, const char *parent_name,
		struct clk *div, struct clk *mux, struct clk *pll,
		struct clk *step);

struct clk *imx_dev_clk_hw_pll14xx(struct udevice *dev, const char *name,
			    const char *parent_name, void __iomem *base,
			    const struct imx_pll14xx_clk *pll_clk);
#define imx_clk_pll14xx(name, parent_name, base, pll_clk) \
	imx_dev_clk_hw_pll14xx(NULL, name, parent_name, base, pll_clk)

struct clk *imx_clk_frac_pll(const char *name, const char *parent_name,
			     void __iomem *base);

struct clk *imx_clk_sccg_pll(const char *name,
				const char * const *parent_names,
				u8 num_parents,
				u8 parent, u8 bypass1, u8 bypass2,
				void __iomem *base,
				unsigned long flags);

struct clk *clk_register_gate2(struct udevice *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 bit_idx, u8 cgr_val,
		u8 clk_gate_flags);

struct clk *imx_clk_pllv3(enum imx_pllv3_type type, const char *name,
			  const char *parent_name, void __iomem *base,
			  u32 div_mask);

static inline struct clk *imx_clk_gate2(const char *name, const char *parent,
					void __iomem *reg, u8 shift)
{
	return clk_register_gate2(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0x3, 0);
}

static inline struct clk *imx_clk_gate2_flags(const char *name, const char *parent,
					void __iomem *reg, u8 shift, unsigned long flags)
{
	return clk_register_gate2(NULL, name, parent, flags | CLK_SET_RATE_PARENT, reg,
			shift, 0x3, 0);
}

static inline struct clk *imx_clk_gate3_flags(const char *name,
		const char *parent, void __iomem *reg, u8 shift,
		unsigned long flags)
{
	return clk_register_gate(NULL, name, parent,
			flags | CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0, NULL);
}

static inline struct clk *imx_clk_gate4(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate2(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0x3, 0);
}

static inline struct clk *imx_clk_gate4_flags(const char *name,
		const char *parent, void __iomem *reg, u8 shift,
		unsigned long flags)
{
	return clk_register_gate2(NULL, name, parent,
			flags | CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0x3, 0);
}

static inline struct clk *imx_clk_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk *imx_clk_divider(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u8 width)
{
	return clk_register_divider(NULL, name, parent, CLK_SET_RATE_PARENT,
			reg, shift, width, 0);
}

static inline struct clk *imx_clk_divider_closest(const char *name,
		const char *parent, void __iomem *reg, u8 shift, u8 width)
{
	return clk_register_divider(NULL, name, parent, 0,
				       reg, shift, width, CLK_DIVIDER_ROUND_CLOSEST);
}

static inline struct clk *
imx_clk_busy_divider(const char *name, const char *parent, void __iomem *reg,
		     u8 shift, u8 width, void __iomem *busy_reg, u8 busy_shift)
{
	return clk_register_divider(NULL, name, parent, CLK_SET_RATE_PARENT,
			reg, shift, width, 0);
}

static inline struct clk *imx_clk_divider2(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u8 width)
{
	return clk_register_divider(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, width, 0);
}

static inline struct clk *imx_clk_divider_flags(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u8 width, unsigned long flags)
{
	return clk_register_divider(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, width, flags);
}

struct clk *imx_clk_pfd(const char *name, const char *parent_name,
			void __iomem *reg, u8 idx);

struct clk *imx_clk_fixup_mux(const char *name, void __iomem *reg,
			      u8 shift, u8 width, const char * const *parents,
			      int num_parents, void (*fixup)(u32 *val));

static inline struct clk *imx_clk_mux_flags(const char *name,
			void __iomem *reg, u8 shift, u8 width,
			const char * const *parents, int num_parents,
			unsigned long flags)
{
	return clk_register_mux(NULL, name, parents, num_parents,
				flags | CLK_SET_RATE_NO_REPARENT, reg, shift,
				width, 0);
}

static inline struct clk *imx_clk_mux2_flags(const char *name,
		void __iomem *reg, u8 shift, u8 width,
		const char * const *parents,
		int num_parents, unsigned long flags)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			flags | CLK_SET_RATE_NO_REPARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, width, 0);
}

static inline struct clk *imx_clk_mux(const char *name, void __iomem *reg,
			u8 shift, u8 width, const char * const *parents,
			int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0);
}

static inline struct clk *imx_dev_clk_mux(struct udevice *dev, const char *name,
			void __iomem *reg, u8 shift, u8 width,
			const char * const *parents, int num_parents)
{
	return clk_register_mux(dev, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT | CLK_SET_PARENT_GATE,
			reg, shift, width, 0);
}

static inline struct clk *
imx_clk_busy_mux(const char *name, void __iomem *reg, u8 shift, u8 width,
		 void __iomem *busy_reg, u8 busy_shift,
		 const char * const *parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0);
}

static inline struct clk *imx_clk_mux2(const char *name, void __iomem *reg,
			u8 shift, u8 width, const char * const *parents,
			int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, width, 0);
}

static inline struct clk *imx_clk_gate(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, NULL);
}

static inline struct clk *imx_clk_gate_dis(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, CLK_GATE_SET_TO_DISABLE, NULL);
}

static inline struct clk *imx_dev_clk_gate(struct udevice *dev,
		const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(dev, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, NULL);
}

static inline struct clk *imx_clk_gate_flags(const char *name, const char *parent,
		void __iomem *reg, u8 shift, unsigned long flags)
{
	return clk_register_gate(NULL, name, parent, flags | CLK_SET_RATE_PARENT, reg,
			shift, 0, NULL);
}

static inline struct clk *imx_clk_gate2_shared2(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u32 * share_count)
{
	return clk_register_gate2(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0x3, 0);
}

static inline struct clk *imx_clk_gate3(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0, NULL);
}

struct clk *imx8m_clk_composite_flags(const char *name,
		const char * const *parent_names,
		int num_parents, void __iomem *reg, unsigned long flags);

#define __imx8m_clk_composite(name, parent_names, reg, flags) \
	imx8m_clk_composite_flags(name, parent_names, \
		ARRAY_SIZE(parent_names), reg, \
		flags | CLK_SET_RATE_NO_REPARENT | CLK_OPS_PARENT_ENABLE)

#define imx8m_clk_composite(name, parent_names, reg) \
	__imx8m_clk_composite(name, parent_names, reg, 0)

#define imx8m_clk_composite_critical(name, parent_names, reg) \
	__imx8m_clk_composite(name, parent_names, reg, CLK_IS_CRITICAL)

struct clk *imx8ulp_clk_composite(const char *name,
				     const char * const *parent_names,
				     int num_parents, bool mux_present,
				     bool rate_present, bool gate_present,
				     void __iomem *reg, bool has_swrst);

enum imx_pllv4_type {
	IMX_PLLV4_IMX7ULP,
	IMX_PLLV4_IMX8ULP,
};

struct clk *imx_clk_pllv4(enum imx_pllv4_type type, const char *name,
		 const char *parent_name, void __iomem *base);

enum imx_pfdv2_type {
	IMX_PFDV2_IMX7ULP,
	IMX_PFDV2_IMX8ULP,
};

struct clk *imx_clk_pfdv2(enum imx_pfdv2_type type, const char *name,
	 const char *parent_name, void __iomem *reg, u8 idx);


#endif /* __MACH_IMX_CLK_H */
