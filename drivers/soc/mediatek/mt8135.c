#ifdef CONFIG_MT8135
static int mt8135_regs[] = {
	[PWRAP_MUX_SEL] =		0x0,
	[PWRAP_WRAP_EN] =		0x4,
	[PWRAP_DIO_EN] =		0x8,
	[PWRAP_SIDLY] =			0xc,
	[PWRAP_CSHEXT] =		0x10,
	[PWRAP_CSHEXT_WRITE] =		0x14,
	[PWRAP_CSHEXT_READ] =		0x18,
	[PWRAP_CSLEXT_START] =		0x1c,
	[PWRAP_CSLEXT_END] =		0x20,
	[PWRAP_STAUPD_PRD] =		0x24,
	[PWRAP_STAUPD_GRPEN] =		0x28,
	[PWRAP_STAUPD_MAN_TRIG] =	0x2c,
	[PWRAP_STAUPD_STA] =		0x30,
	[PWRAP_EVENT_IN_EN] =		0x34,
	[PWRAP_EVENT_DST_EN] =		0x38,
	[PWRAP_WRAP_STA] =		0x3c,
	[PWRAP_RRARB_INIT] =		0x40,
	[PWRAP_RRARB_EN] =		0x44,
	[PWRAP_RRARB_STA0] =		0x48,
	[PWRAP_RRARB_STA1] =		0x4c,
	[PWRAP_HARB_INIT] =		0x50,
	[PWRAP_HARB_HPRIO] =		0x54,
	[PWRAP_HIPRIO_ARB_EN] =		0x58,
	[PWRAP_HARB_STA0] =		0x5c,
	[PWRAP_HARB_STA1] =		0x60,
	[PWRAP_MAN_EN] =		0x64,
	[PWRAP_MAN_CMD] =		0x68,
	[PWRAP_MAN_RDATA] =		0x6c,
	[PWRAP_MAN_VLDCLR] =		0x70,
	[PWRAP_WACS0_EN] =		0x74,
	[PWRAP_INIT_DONE0] =		0x78,
	[PWRAP_WACS0_CMD] =		0x7c,
	[PWRAP_WACS0_RDATA] =		0x80,
	[PWRAP_WACS0_VLDCLR] =		0x84,
	[PWRAP_WACS1_EN] =		0x88,
	[PWRAP_INIT_DONE1] =		0x8c,
	[PWRAP_WACS1_CMD] =		0x90,
	[PWRAP_WACS1_RDATA] =		0x94,
	[PWRAP_WACS1_VLDCLR] =		0x98,
	[PWRAP_WACS2_EN] =		0x9c,
	[PWRAP_INIT_DONE2] =		0xa0,
	[PWRAP_WACS2_CMD] =		0xa4,
	[PWRAP_WACS2_RDATA] =		0xa8,
	[PWRAP_WACS2_VLDCLR] =		0xac,
	[PWRAP_INT_EN] =		0xb0,
	[PWRAP_INT_FLG_RAW] =		0xb4,
	[PWRAP_INT_FLG] =		0xb8,
	[PWRAP_INT_CLR] =		0xbc,
	[PWRAP_SIG_ADR] =		0xc0,
	[PWRAP_SIG_MODE] =		0xc4,
	[PWRAP_SIG_VALUE] =		0xc8,
	[PWRAP_SIG_ERRVAL] =		0xcc,
	[PWRAP_CRC_EN] =		0xd0,
	[PWRAP_EVENT_STA] =		0xd4,
	[PWRAP_EVENT_STACLR] =		0xd8,
	[PWRAP_TIMER_EN] =		0xdc,
	[PWRAP_TIMER_STA] =		0xe0,
	[PWRAP_WDT_UNIT] =		0xe4,
	[PWRAP_WDT_SRC_EN] =		0xe8,
	[PWRAP_WDT_FLG] =		0xec,
	[PWRAP_DEBUG_INT_SEL] =		0xf0,
	[PWRAP_CIPHER_KEY_SEL] =	0x134,
	[PWRAP_CIPHER_IV_SEL] =		0x138,
	[PWRAP_CIPHER_LOAD] =		0x13c,
	[PWRAP_CIPHER_START] =		0x140,
	[PWRAP_CIPHER_RDY] =		0x144,
	[PWRAP_CIPHER_MODE] =		0x148,
	[PWRAP_CIPHER_SWRST] =		0x14c,
	[PWRAP_DCM_EN] =		0x15c,
	[PWRAP_DCM_DBC_PRD] =		0x160,
};

static int pwrap_mt8135_init_soc_specific(struct pmic_wrapper *wrp)
{
	/* enable pwrap events and pwrap bridge in AP side */
	pwrap_writel(wrp, 0x1, PWRAP_EVENT_IN_EN);
	pwrap_writel(wrp, 0xffff, PWRAP_EVENT_DST_EN);
	writel(0x7f, wrp->bridge_base + PWRAP_MT8135_BRIDGE_IORD_ARB_EN);
	writel(0x1, wrp->bridge_base + PWRAP_MT8135_BRIDGE_WACS3_EN);
	writel(0x1, wrp->bridge_base + PWRAP_MT8135_BRIDGE_WACS4_EN);
	writel(0x1, wrp->bridge_base + PWRAP_MT8135_BRIDGE_WDT_UNIT);
	writel(0xffff, wrp->bridge_base + PWRAP_MT8135_BRIDGE_WDT_SRC_EN);
	writel(0x1, wrp->bridge_base + PWRAP_MT8135_BRIDGE_TIMER_EN);
	writel(0x7ff, wrp->bridge_base + PWRAP_MT8135_BRIDGE_INT_EN);

	/* enable PMIC event out and sources */
	if (pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_EVENT_OUT_EN],
			0x1) ||
	    pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_EVENT_SRC_EN],
			0xffff)) {
		dev_err(wrp->dev, "enable dewrap fail\n");
		return -EFAULT;
	}

	return 0;
}

static const struct pmic_wrapper_type pwrap_mt8135 = {
	.regs = mt8135_regs,
	.type = PWRAP_MT8135,
	.arb_en_all = 0x1ff,
	.int_en_all = ~(u32)(BIT(31) | BIT(1)),
	.int1_en_all = 0,
	.spi_w = PWRAP_MAN_CMD_SPI_WRITE,
	.wdt_src = PWRAP_WDT_SRC_MASK_ALL,
	.caps = PWRAP_CAP_BRIDGE | PWRAP_CAP_RESET | PWRAP_CAP_DCM,
	.init_reg_clock = pwrap_common_init_reg_clock,
	.init_soc_specific = pwrap_mt8135_init_soc_specific,
};

#define WRAP_MT8135_COMPAT	{ .compatible = "mediatek,mt8135-pwrap", .data = (ulong)&pwrap_mt8135, },
#else
#define WRAP_MT8135_COMPAT
#endif
