#ifdef CONFIG_MT6380
/* The MT6380 PMIC only implements a regulator, so we bind it
 * directly instead of using a MFD.
 */
#define NEED_PWRAP_REGMAP32
static int pwrap_read32(struct pmic_wrapper *wrp, u32 adr, u32 *rdata);
static int pwrap_write32(struct pmic_wrapper *wrp, u32 adr, u32 wdata);
static const struct regmap_config pwrap_regmap_config32;

static const struct pwrap_slv_type pmic_mt6380 = {
	.dew_regs = NULL,
	.type = PMIC_MT6380,
	.regmap = &pwrap_regmap_config32,
	.caps = 0,
	.pwrap_read = pwrap_read32,
	.pwrap_write = pwrap_write32,
};

#define MT6380_COMPAT	{ .compatible = "mediatek,mt6380-regulator", .data = (ulong)&pmic_mt6380, },
#else
#define MT6380_COMPAT
#endif
