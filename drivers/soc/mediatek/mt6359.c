#ifdef CONFIG_MT6359
static const u32 mt6359_regs[] = {
	[PWRAP_DEW_RG_EN_RECORD] =	0x040a,
	[PWRAP_DEW_DIO_EN] =		0x040c,
	[PWRAP_DEW_READ_TEST] =		0x040e,
	[PWRAP_DEW_WRITE_TEST] =	0x0410,
	[PWRAP_DEW_CRC_SWRST] =		0x0412,
	[PWRAP_DEW_CRC_EN] =		0x0414,
	[PWRAP_DEW_CRC_VAL] =		0x0416,
	[PWRAP_DEW_CIPHER_KEY_SEL] =	0x0418,
	[PWRAP_DEW_CIPHER_IV_SEL] =	0x041a,
	[PWRAP_DEW_CIPHER_EN] =		0x041c,
	[PWRAP_DEW_CIPHER_RDY] =	0x041e,
	[PWRAP_DEW_CIPHER_MODE] =	0x0420,
	[PWRAP_DEW_CIPHER_SWRST] =	0x0422,
	[PWRAP_DEW_RDDMY_NO] =		0x0424,
	[PWRAP_DEW_RECORD_CMD0] =	0x0428,
	[PWRAP_DEW_RECORD_CMD1] =	0x042a,
	[PWRAP_DEW_RECORD_CMD2] =	0x042c,
	[PWRAP_DEW_RECORD_CMD3] =	0x042e,
	[PWRAP_DEW_RECORD_CMD4] =	0x0430,
	[PWRAP_DEW_RECORD_CMD5] =	0x0432,
	[PWRAP_DEW_RECORD_WDATA0] =	0x0434,
	[PWRAP_DEW_RECORD_WDATA1] =	0x0436,
	[PWRAP_DEW_RECORD_WDATA2] =	0x0438,
	[PWRAP_DEW_RECORD_WDATA3] =	0x043a,
	[PWRAP_DEW_RECORD_WDATA4] =	0x043c,
	[PWRAP_DEW_RECORD_WDATA5] =	0x043e,
	[PWRAP_DEW_RG_ADDR_TARGET] =	0x0440,
	[PWRAP_DEW_RG_ADDR_MASK] =	0x0442,
	[PWRAP_DEW_RG_WDATA_TARGET] =	0x0444,
	[PWRAP_DEW_RG_WDATA_MASK] =	0x0446,
	[PWRAP_DEW_RG_SPI_RECORD_CLR] =	0x0448,
	[PWRAP_DEW_RG_CMD_ALERT_CLR] =	0x0448,
	[PWRAP_SPISLV_KEY] =		0x044a,
};

#define NEED_PWRAP_REGMAP16
static int pwrap_read16(struct pmic_wrapper *wrp, u32 adr, u32 *rdata);
static int pwrap_write16(struct pmic_wrapper *wrp, u32 adr, u32 wdata);
static const struct regmap_config pwrap_regmap_config16;

static const struct pwrap_slv_type pmic_mt6359 = {
	.dew_regs = mt6359_regs,
	.type = PMIC_MT6359,
	.regmap = &pwrap_regmap_config16,
	.caps = PWRAP_SLV_CAP_DUALIO,
	.pwrap_read = pwrap_read16,
	.pwrap_write = pwrap_write16,
};

#define MT6359_COMPAT	{ .compatible = "mediatek,mt6359", .data = (ulong)&pmic_mt6359, },
#else
#define MT6359_COMPAT
#endif
