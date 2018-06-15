/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * Derived from linux/arch/mips/bcm63xx/cpu.c:
 *	Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 *	Copyright (C) 2009 Florian Fainelli <florian@openwrt.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <cpu.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#define REV_CHIPID_SHIFT		16
#define REV_CHIPID_MASK			(0xffff << REV_CHIPID_SHIFT)
#define REV_LONG_CHIPID_SHIFT		12
#define REV_LONG_CHIPID_MASK		(0xfffff << REV_LONG_CHIPID_SHIFT)
#define REV_REVID_SHIFT			0
#define REV_REVID_MASK			(0xff << REV_REVID_SHIFT)

#define REG_BCM6328_OTP			0x62c
#define BCM6328_TP1_DISABLED		BIT(9)

#define REG_BCM6328_MISC_STRAPBUS	0x1a40
#define STRAPBUS_6328_FCVO_SHIFT	7
#define STRAPBUS_6328_FCVO_MASK		(0x1f << STRAPBUS_6328_FCVO_SHIFT)

#define REG_BCM6348_PERF_MIPSPLLCFG	0x34
#define MIPSPLLCFG_6348_M1CPU_SHIFT	6
#define MIPSPLLCFG_6348_M1CPU_MASK	(0x7 << MIPSPLLCFG_6348_M1CPU_SHIFT)
#define MIPSPLLCFG_6348_N2_SHIFT	15
#define MIPSPLLCFG_6348_N2_MASK		(0x1F << MIPSPLLCFG_6348_N2_SHIFT)
#define MIPSPLLCFG_6348_N1_SHIFT	20
#define MIPSPLLCFG_6348_N1_MASK		(0x7 << MIPSPLLCFG_6348_N1_SHIFT)

#define REG_BCM6358_DDR_DMIPSPLLCFG	0x12b8
#define DMIPSPLLCFG_6358_M1_SHIFT	0
#define DMIPSPLLCFG_6358_M1_MASK	(0xff << DMIPSPLLCFG_6358_M1_SHIFT)
#define DMIPSPLLCFG_6358_N1_SHIFT	23
#define DMIPSPLLCFG_6358_N1_MASK	(0x3f << DMIPSPLLCFG_6358_N1_SHIFT)
#define DMIPSPLLCFG_6358_N2_SHIFT	29
#define DMIPSPLLCFG_6358_N2_MASK	(0x7 << DMIPSPLLCFG_6358_N2_SHIFT)

#define REG_BCM63268_MISC_STRAPBUS	0x1814
#define STRAPBUS_63268_FCVO_SHIFT	21
#define STRAPBUS_63268_FCVO_MASK	(0xf << STRAPBUS_63268_FCVO_SHIFT)

struct bmips_cpu_priv;

struct bmips_cpu_hw {
	int (*get_cpu_desc)(struct bmips_cpu_priv *priv, char *buf, int size);
	ulong (*get_cpu_freq)(struct bmips_cpu_priv *);
	int (*get_cpu_count)(struct bmips_cpu_priv *);
};

struct bmips_cpu_priv {
	void __iomem *regs;
	const struct bmips_cpu_hw *hw;
};

/* Specific CPU Ops */
static int bmips_short_cpu_desc(struct bmips_cpu_priv *priv, char *buf,
				int size)
{
	unsigned short cpu_id;
	unsigned char cpu_rev;
	u32 val;

	val = readl_be(priv->regs);
	cpu_id = (val & REV_CHIPID_MASK) >> REV_CHIPID_SHIFT;
	cpu_rev = (val & REV_REVID_MASK) >> REV_REVID_SHIFT;

	snprintf(buf, size, "BCM%04X%02X", cpu_id, cpu_rev);

	return 0;
}

static int bmips_long_cpu_desc(struct bmips_cpu_priv *priv, char *buf,
				int size)
{
	unsigned int cpu_id;
	unsigned char cpu_rev;
	u32 val;

	val = readl_be(priv->regs);
	cpu_id = (val & REV_LONG_CHIPID_MASK) >> REV_LONG_CHIPID_SHIFT;
	cpu_rev = (val & REV_REVID_MASK) >> REV_REVID_SHIFT;

	snprintf(buf, size, "BCM%05X%02X", cpu_id, cpu_rev);

	return 0;
}

static ulong bcm3380_get_cpu_freq(struct bmips_cpu_priv *priv)
{
	return 333000000;
}

static ulong bcm6328_get_cpu_freq(struct bmips_cpu_priv *priv)
{
	unsigned int mips_pll_fcvo;

	mips_pll_fcvo = readl_be(priv->regs + REG_BCM6328_MISC_STRAPBUS);
	mips_pll_fcvo = (mips_pll_fcvo & STRAPBUS_6328_FCVO_MASK)
			>> STRAPBUS_6328_FCVO_SHIFT;

	switch (mips_pll_fcvo) {
	case 0x12:
	case 0x14:
	case 0x19:
		return 160000000;
	case 0x1c:
		return 192000000;
	case 0x13:
	case 0x15:
		return 200000000;
	case 0x1a:
		return 384000000;
	case 0x16:
		return 400000000;
	default:
		return 320000000;
	}
}

static ulong bcm6338_get_cpu_freq(struct bmips_cpu_priv *priv)
{
	return 240000000;
}

static ulong bcm6348_get_cpu_freq(struct bmips_cpu_priv *priv)
{
	unsigned int tmp, n1, n2, m1;

	tmp = readl_be(priv->regs + REG_BCM6348_PERF_MIPSPLLCFG);
	n1 = (tmp & MIPSPLLCFG_6348_N1_MASK) >> MIPSPLLCFG_6348_N1_SHIFT;
	n2 = (tmp & MIPSPLLCFG_6348_N2_MASK) >> MIPSPLLCFG_6348_N2_SHIFT;
	m1 = (tmp & MIPSPLLCFG_6348_M1CPU_MASK) >> MIPSPLLCFG_6348_M1CPU_SHIFT;

	return (16 * 1000000 * (n1 + 1) * (n2 + 2)) / (m1 + 1);
}

static ulong bcm6358_get_cpu_freq(struct bmips_cpu_priv *priv)
{
	unsigned int tmp, n1, n2, m1;

	tmp = readl_be(priv->regs + REG_BCM6358_DDR_DMIPSPLLCFG);
	n1 = (tmp & DMIPSPLLCFG_6358_N1_MASK) >> DMIPSPLLCFG_6358_N1_SHIFT;
	n2 = (tmp & DMIPSPLLCFG_6358_N2_MASK) >> DMIPSPLLCFG_6358_N2_SHIFT;
	m1 = (tmp & DMIPSPLLCFG_6358_M1_MASK) >> DMIPSPLLCFG_6358_M1_SHIFT;

	return (16 * 1000000 * n1 * n2) / m1;
}

static ulong bcm63268_get_cpu_freq(struct bmips_cpu_priv *priv)
{
	unsigned int mips_pll_fcvo;

	mips_pll_fcvo = readl_be(priv->regs + REG_BCM63268_MISC_STRAPBUS);
	mips_pll_fcvo = (mips_pll_fcvo & STRAPBUS_63268_FCVO_MASK)
			>> STRAPBUS_63268_FCVO_SHIFT;

	switch (mips_pll_fcvo) {
	case 0x3:
	case 0xe:
		return 320000000;
	case 0xa:
		return 333000000;
	case 0x2:
	case 0xb:
	case 0xf:
		return 400000000;
	default:
		return 0;
	}
}

static int bcm6328_get_cpu_count(struct bmips_cpu_priv *priv)
{
	u32 val = readl_be(priv->regs + REG_BCM6328_OTP);

	if (val & BCM6328_TP1_DISABLED)
		return 1;
	else
		return 2;
}

static int bcm6345_get_cpu_count(struct bmips_cpu_priv *priv)
{
	return 1;
}

static int bcm6358_get_cpu_count(struct bmips_cpu_priv *priv)
{
	return 2;
}

static const struct bmips_cpu_hw bmips_cpu_bcm3380 = {
	.get_cpu_desc = bmips_short_cpu_desc,
	.get_cpu_freq = bcm3380_get_cpu_freq,
	.get_cpu_count = bcm6358_get_cpu_count,
};

static const struct bmips_cpu_hw bmips_cpu_bcm6328 = {
	.get_cpu_desc = bmips_long_cpu_desc,
	.get_cpu_freq = bcm6328_get_cpu_freq,
	.get_cpu_count = bcm6328_get_cpu_count,
};

static const struct bmips_cpu_hw bmips_cpu_bcm6338 = {
	.get_cpu_desc = bmips_short_cpu_desc,
	.get_cpu_freq = bcm6338_get_cpu_freq,
	.get_cpu_count = bcm6345_get_cpu_count,
};

static const struct bmips_cpu_hw bmips_cpu_bcm6348 = {
	.get_cpu_desc = bmips_short_cpu_desc,
	.get_cpu_freq = bcm6348_get_cpu_freq,
	.get_cpu_count = bcm6345_get_cpu_count,
};

static const struct bmips_cpu_hw bmips_cpu_bcm6358 = {
	.get_cpu_desc = bmips_short_cpu_desc,
	.get_cpu_freq = bcm6358_get_cpu_freq,
	.get_cpu_count = bcm6358_get_cpu_count,
};

static const struct bmips_cpu_hw bmips_cpu_bcm63268 = {
	.get_cpu_desc = bmips_long_cpu_desc,
	.get_cpu_freq = bcm63268_get_cpu_freq,
	.get_cpu_count = bcm6358_get_cpu_count,
};

/* Generic CPU Ops */
static int bmips_cpu_get_desc(struct udevice *dev, char *buf, int size)
{
	struct bmips_cpu_priv *priv = dev_get_priv(dev);
	const struct bmips_cpu_hw *hw = priv->hw;

	return hw->get_cpu_desc(priv, buf, size);
}

static int bmips_cpu_get_info(struct udevice *dev, struct cpu_info *info)
{
	struct bmips_cpu_priv *priv = dev_get_priv(dev);
	const struct bmips_cpu_hw *hw = priv->hw;

	info->cpu_freq = hw->get_cpu_freq(priv);
	info->features = BIT(CPU_FEAT_L1_CACHE);
	info->features |= BIT(CPU_FEAT_MMU);
	info->features |= BIT(CPU_FEAT_DEVICE_ID);

	return 0;
}

static int bmips_cpu_get_count(struct udevice *dev)
{
	struct bmips_cpu_priv *priv = dev_get_priv(dev);
	const struct bmips_cpu_hw *hw = priv->hw;

	return hw->get_cpu_count(priv);
}

static int bmips_cpu_get_vendor(struct udevice *dev, char *buf, int size)
{
	snprintf(buf, size, "Broadcom");

	return 0;
}

static const struct cpu_ops bmips_cpu_ops = {
	.get_desc = bmips_cpu_get_desc,
	.get_info = bmips_cpu_get_info,
	.get_count = bmips_cpu_get_count,
	.get_vendor = bmips_cpu_get_vendor,
};

/* BMIPS CPU driver */
int bmips_cpu_bind(struct udevice *dev)
{
	struct cpu_platdata *plat = dev_get_parent_platdata(dev);

	plat->cpu_id = fdtdec_get_int(gd->fdt_blob, dev_of_offset(dev),
		"reg", -1);
	plat->device_id = read_c0_prid();

	return 0;
}

int bmips_cpu_probe(struct udevice *dev)
{
	struct bmips_cpu_priv *priv = dev_get_priv(dev);
	const struct bmips_cpu_hw *hw =
		(const struct bmips_cpu_hw *)dev_get_driver_data(dev);
	fdt_addr_t addr;
	fdt_size_t size;

	addr = devfdt_get_addr_size_index(dev_get_parent(dev), 0, &size);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->regs = ioremap(addr, size);
	priv->hw = hw;

	return 0;
}

static const struct udevice_id bmips_cpu_ids[] = {
	{
		.compatible = "brcm,bcm3380-cpu",
		.data = (ulong)&bmips_cpu_bcm3380,
	}, {
		.compatible = "brcm,bcm6328-cpu",
		.data = (ulong)&bmips_cpu_bcm6328,
	}, {
		.compatible = "brcm,bcm6338-cpu",
		.data = (ulong)&bmips_cpu_bcm6338,
	}, {
		.compatible = "brcm,bcm6348-cpu",
		.data = (ulong)&bmips_cpu_bcm6348,
	}, {
		.compatible = "brcm,bcm6358-cpu",
		.data = (ulong)&bmips_cpu_bcm6358,
	}, {
		.compatible = "brcm,bcm63268-cpu",
		.data = (ulong)&bmips_cpu_bcm63268,
	},
	{ /* sentinel */ }
};

U_BOOT_DRIVER(bmips_cpu_drv) = {
	.name = "bmips_cpu",
	.id = UCLASS_CPU,
	.of_match = bmips_cpu_ids,
	.bind = bmips_cpu_bind,
	.probe = bmips_cpu_probe,
	.priv_auto_alloc_size = sizeof(struct bmips_cpu_priv),
	.ops = &bmips_cpu_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

#ifdef CONFIG_DISPLAY_CPUINFO
int print_cpuinfo(void)
{
	struct cpu_info cpu;
	struct udevice *dev;
	int err;
	char desc[100];

	err = uclass_get_device(UCLASS_CPU, 0, &dev);
	if (err)
		return 0;

	err = cpu_get_info(dev, &cpu);
	if (err)
		return 0;

	err = cpu_get_desc(dev, desc, sizeof(desc));
	if (err)
		return 0;

	printf("Chip ID: %s, MIPS: ", desc);
	print_freq(cpu.cpu_freq, "\n");

	return 0;
}
#endif
