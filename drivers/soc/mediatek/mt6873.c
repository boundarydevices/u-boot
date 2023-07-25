#ifdef CONFIG_MT6873
static int mt6873_regs[] = {
	[PWRAP_INIT_DONE2] =		0x0,
	[PWRAP_TIMER_EN] =		0x3E0,
	[PWRAP_INT_EN] =		0x448,
	[PWRAP_WACS2_CMD] =		0xC80,
	[PWRAP_SWINF_2_WDATA_31_0] =	0xC84,
	[PWRAP_SWINF_2_RDATA_31_0] =	0xC94,
	[PWRAP_WACS2_VLDCLR] =		0xCA4,
	[PWRAP_WACS2_RDATA] =		0xCA8,
};

static const struct pmic_wrapper_type pwrap_mt6873 = {
	.regs = mt6873_regs,
	.type = PWRAP_MT6873,
	.arb_en_all = 0x777f,
	.int_en_all = BIT(4) | BIT(5),
	.int1_en_all = 0,
	.spi_w = PWRAP_MAN_CMD_SPI_WRITE,
	.wdt_src = PWRAP_WDT_SRC_MASK_ALL,
	.caps = PWRAP_CAP_ARB,
	.init_reg_clock = pwrap_common_init_reg_clock,
	.init_soc_specific = NULL,
};

#define WRAP_MT6873_COMPAT	{ .compatible = "mediatek,mt6873-pwrap", .data = (ulong)&pwrap_mt6873, },
#else
#define WRAP_MT6873_COMPAT
#endif
