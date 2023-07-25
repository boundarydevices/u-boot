#ifdef CONFIG_MT6397
#include <linux/mfd/mt6397/core.h>
#include <linux/mfd/mt6397/registers.h>

#define MT6397_RTC_BASE		0xe000
#define MT6397_RTC_SIZE		0x3e

static const struct resource mt6397_rtc_resources[] = {
	DEFINE_RES_MEM(MT6397_RTC_BASE, MT6397_RTC_SIZE),
	DEFINE_RES_IRQ(MT6397_IRQ_RTC),
};

static const struct resource mt6397_keys_resources[] = {
	DEFINE_RES_IRQ_NAMED(MT6397_IRQ_PWRKEY, "powerkey"),
	DEFINE_RES_IRQ_NAMED(MT6397_IRQ_HOMEKEY, "homekey"),
};

static const struct mfd_cell mt6397_devs[] = {
	{
		.name = "mt6397-rtc",
		.num_resources = ARRAY_SIZE(mt6397_rtc_resources),
		.resources = mt6397_rtc_resources,
		.of_compatible = "mediatek,mt6397-rtc",
	}, {
		.name = "mt6397-regulator",
		.of_compatible = "mediatek,mt6397-regulator",
	}, {
		.name = "mt6397-codec",
		.of_compatible = "mediatek,mt6397-codec",
	}, {
		.name = "mt6397-clk",
		.of_compatible = "mediatek,mt6397-clk",
	}, {
		.name = "mt6397-pinctrl",
		.of_compatible = "mediatek,mt6397-pinctrl",
	}, {
		.name = "mtk-pmic-keys",
		.num_resources = ARRAY_SIZE(mt6397_keys_resources),
		.resources = mt6397_keys_resources,
		.of_compatible = "mediatek,mt6397-keys"
	}
};

static const struct chip_data mt6397_core = {
	.cid_addr = MT6397_CID,
	.cid_shift = 0,
	.cells = mt6397_devs,
	.cell_size = ARRAY_SIZE(mt6397_devs),
};

#define PMIC_MT6397_COMPAT	{ .compatible = "mediatek,mt6397", .data = (ulong)&mt6397_core, },
#else
#define PMIC_MT6397_COMPAT
#endif
