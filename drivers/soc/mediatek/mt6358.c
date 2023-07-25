#ifdef CONFIG_MT6358
static const u32 mt6358_regs[] = {
	[PWRAP_SMT_CON1] =		0x0030,
	[PWRAP_DRV_CON1] =		0x0038,
	[PWRAP_FILTER_CON0] =		0x0040,
	[PWRAP_GPIO_PULLEN0_CLR] =	0x0098,
	[PWRAP_RG_SPI_CON0] =		0x0408,
	[PWRAP_RG_SPI_RECORD0] =	0x040a,
	[PWRAP_DEW_DIO_EN] =		0x040c,
	[PWRAP_DEW_READ_TEST]	=	0x040e,
	[PWRAP_DEW_WRITE_TEST]	=	0x0410,
	[PWRAP_DEW_CRC_EN] =		0x0414,
	[PWRAP_DEW_CIPHER_KEY_SEL] =	0x041a,
	[PWRAP_DEW_CIPHER_IV_SEL] =	0x041c,
	[PWRAP_DEW_CIPHER_EN]	=	0x041e,
	[PWRAP_DEW_CIPHER_RDY] =	0x0420,
	[PWRAP_DEW_CIPHER_MODE] =	0x0422,
	[PWRAP_DEW_CIPHER_SWRST] =	0x0424,
	[PWRAP_RG_SPI_CON2] =		0x0432,
	[PWRAP_RG_SPI_CON3] =		0x0434,
	[PWRAP_RG_SPI_CON4] =		0x0436,
	[PWRAP_RG_SPI_CON5] =		0x0438,
	[PWRAP_RG_SPI_CON6] =		0x043a,
	[PWRAP_RG_SPI_CON7] =		0x043c,
	[PWRAP_RG_SPI_CON8] =		0x043e,
	[PWRAP_RG_SPI_CON13] =		0x0448,
	[PWRAP_SPISLV_KEY] =		0x044a,
};

#define NEED_PWRAP_REGMAP16
static int pwrap_read16(struct pmic_wrapper *wrp, u32 adr, u32 *rdata);
static int pwrap_write16(struct pmic_wrapper *wrp, u32 adr, u32 wdata);
static const struct regmap_config pwrap_regmap_config16;

static const struct pwrap_slv_type pmic_mt6358 = {
	.dew_regs = mt6358_regs,
	.type = PMIC_MT6358,
	.regmap = &pwrap_regmap_config16,
	.caps = PWRAP_SLV_CAP_SPI | PWRAP_SLV_CAP_DUALIO,
	.pwrap_read = pwrap_read16,
	.pwrap_write = pwrap_write16,
};

#define MT6358_COMPAT	{ .compatible = "mediatek,mt6358", .data = (ulong)&pmic_mt6358, },
#else
#define MT6358_COMPAT
#endif
