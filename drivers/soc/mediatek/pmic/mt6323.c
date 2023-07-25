#ifdef CONFIG_MT6323
#include <linux/mfd/mt6323/core.h>
#include <linux/mfd/mt6323/registers.h>

#define MT6323_RTC_BASE		0x8000
#define MT6323_RTC_SIZE		0x40

static const struct resource mt6323_rtc_resources[] = {
	DEFINE_RES_MEM(MT6323_RTC_BASE, MT6323_RTC_SIZE),
	DEFINE_RES_IRQ(MT6323_IRQ_STATUS_RTC),
};

#define MT6323_PWRC_BASE	0x8000
#define MT6323_PWRC_SIZE	0x40

static const struct resource mt6323_keys_resources[] = {
	DEFINE_RES_IRQ_NAMED(MT6323_IRQ_STATUS_PWRKEY, "powerkey"),
	DEFINE_RES_IRQ_NAMED(MT6323_IRQ_STATUS_FCHRKEY, "homekey"),
};

static const struct resource mt6323_pwrc_resources[] = {
	DEFINE_RES_MEM(MT6323_PWRC_BASE, MT6323_PWRC_SIZE),
};

static const struct mfd_cell mt6323_devs[] = {
	{
		.name = "mt6323-rtc",
		.num_resources = ARRAY_SIZE(mt6323_rtc_resources),
		.resources = mt6323_rtc_resources,
		.of_compatible = "mediatek,mt6323-rtc",
	}, {
		.name = "mt6323-regulator",
		.of_compatible = "mediatek,mt6323-regulator"
	}, {
		.name = "mt6323-led",
		.of_compatible = "mediatek,mt6323-led"
	}, {
		.name = "mtk-pmic-keys",
		.num_resources = ARRAY_SIZE(mt6323_keys_resources),
		.resources = mt6323_keys_resources,
		.of_compatible = "mediatek,mt6323-keys"
	}, {
		.name = "mt6323-pwrc",
		.num_resources = ARRAY_SIZE(mt6323_pwrc_resources),
		.resources = mt6323_pwrc_resources,
		.of_compatible = "mediatek,mt6323-pwrc"
	},
};

static const struct chip_data mt6323_core = {
	.cid_addr = MT6323_CID,
	.cid_shift = 0,
	.cells = mt6323_devs,
	.cell_size = ARRAY_SIZE(mt6323_devs),
};

#define PMIC_MT6323_COMPAT	{ .compatible = "mediatek,mt6323", .data = (ulong)&mt6323_core, },
#else
#define PMIC_MT6323_COMPAT
#endif
