#ifdef CONFIG_MT6357
static const u32 mt6357_regs[] = {
	[PWRAP_DEW_DIO_EN] =            0x040A,
	[PWRAP_DEW_READ_TEST] =         0x040C,
	[PWRAP_DEW_WRITE_TEST] =        0x040E,
	[PWRAP_DEW_CRC_EN] =            0x0412,
	[PWRAP_DEW_CRC_VAL] =           0x0414,
	[PWRAP_DEW_CIPHER_KEY_SEL] =    0x0418,
	[PWRAP_DEW_CIPHER_IV_SEL] =     0x041A,
	[PWRAP_DEW_CIPHER_EN] =         0x041C,
	[PWRAP_DEW_CIPHER_RDY] =        0x041E,
	[PWRAP_DEW_CIPHER_MODE] =       0x0420,
	[PWRAP_DEW_CIPHER_SWRST] =      0x0422,
	[PWRAP_DEW_RDDMY_NO] =          0x0424,
};

#define NEED_PWRAP_REGMAP16
static int pwrap_read16(struct pmic_wrapper *wrp, u32 adr, u32 *rdata);
static int pwrap_write16(struct pmic_wrapper *wrp, u32 adr, u32 wdata);
static const struct regmap_config pwrap_regmap_config16;

static const struct pwrap_slv_type pmic_mt6357 = {
	.dew_regs = mt6357_regs,
	.type = PMIC_MT6357,
	.regmap = &pwrap_regmap_config16,
	.caps = PWRAP_SLV_CAP_SPI | PWRAP_SLV_CAP_DUALIO,
	.pwrap_read = pwrap_read16,
	.pwrap_write = pwrap_write16,
};

#define MT6357_COMPAT	{ .compatible = "mediatek,mt6357", .data = (ulong)&pmic_mt6357, },
#else
#define MT6357_COMPAT
#endif
