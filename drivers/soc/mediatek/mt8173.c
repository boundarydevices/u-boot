#ifdef CONFIG_MT8173
static int mt8173_regs[] = {
	[PWRAP_MUX_SEL] =		0x0,
	[PWRAP_WRAP_EN] =		0x4,
	[PWRAP_DIO_EN] =		0x8,
	[PWRAP_SIDLY] =			0xc,
	[PWRAP_RDDMY] =			0x10,
	[PWRAP_SI_CK_CON] =		0x14,
	[PWRAP_CSHEXT_WRITE] =		0x18,
	[PWRAP_CSHEXT_READ] =		0x1c,
	[PWRAP_CSLEXT_START] =		0x20,
	[PWRAP_CSLEXT_END] =		0x24,
	[PWRAP_STAUPD_PRD] =		0x28,
	[PWRAP_STAUPD_GRPEN] =		0x2c,
	[PWRAP_STAUPD_MAN_TRIG] =	0x40,
	[PWRAP_STAUPD_STA] =		0x44,
	[PWRAP_WRAP_STA] =		0x48,
	[PWRAP_HARB_INIT] =		0x4c,
	[PWRAP_HARB_HPRIO] =		0x50,
	[PWRAP_HIPRIO_ARB_EN] =		0x54,
	[PWRAP_HARB_STA0] =		0x58,
	[PWRAP_HARB_STA1] =		0x5c,
	[PWRAP_MAN_EN] =		0x60,
	[PWRAP_MAN_CMD] =		0x64,
	[PWRAP_MAN_RDATA] =		0x68,
	[PWRAP_MAN_VLDCLR] =		0x6c,
	[PWRAP_WACS0_EN] =		0x70,
	[PWRAP_INIT_DONE0] =		0x74,
	[PWRAP_WACS0_CMD] =		0x78,
	[PWRAP_WACS0_RDATA] =		0x7c,
	[PWRAP_WACS0_VLDCLR] =		0x80,
	[PWRAP_WACS1_EN] =		0x84,
	[PWRAP_INIT_DONE1] =		0x88,
	[PWRAP_WACS1_CMD] =		0x8c,
	[PWRAP_WACS1_RDATA] =		0x90,
	[PWRAP_WACS1_VLDCLR] =		0x94,
	[PWRAP_WACS2_EN] =		0x98,
	[PWRAP_INIT_DONE2] =		0x9c,
	[PWRAP_WACS2_CMD] =		0xa0,
	[PWRAP_WACS2_RDATA] =		0xa4,
	[PWRAP_WACS2_VLDCLR] =		0xa8,
	[PWRAP_INT_EN] =		0xac,
	[PWRAP_INT_FLG_RAW] =		0xb0,
	[PWRAP_INT_FLG] =		0xb4,
	[PWRAP_INT_CLR] =		0xb8,
	[PWRAP_SIG_ADR] =		0xbc,
	[PWRAP_SIG_MODE] =		0xc0,
	[PWRAP_SIG_VALUE] =		0xc4,
	[PWRAP_SIG_ERRVAL] =		0xc8,
	[PWRAP_CRC_EN] =		0xcc,
	[PWRAP_TIMER_EN] =		0xd0,
	[PWRAP_TIMER_STA] =		0xd4,
	[PWRAP_WDT_UNIT] =		0xd8,
	[PWRAP_WDT_SRC_EN] =		0xdc,
	[PWRAP_WDT_FLG] =		0xe0,
	[PWRAP_DEBUG_INT_SEL] =		0xe4,
	[PWRAP_DVFS_ADR0] =		0xe8,
	[PWRAP_DVFS_WDATA0] =		0xec,
	[PWRAP_DVFS_ADR1] =		0xf0,
	[PWRAP_DVFS_WDATA1] =		0xf4,
	[PWRAP_DVFS_ADR2] =		0xf8,
	[PWRAP_DVFS_WDATA2] =		0xfc,
	[PWRAP_DVFS_ADR3] =		0x100,
	[PWRAP_DVFS_WDATA3] =		0x104,
	[PWRAP_DVFS_ADR4] =		0x108,
	[PWRAP_DVFS_WDATA4] =		0x10c,
	[PWRAP_DVFS_ADR5] =		0x110,
	[PWRAP_DVFS_WDATA5] =		0x114,
	[PWRAP_DVFS_ADR6] =		0x118,
	[PWRAP_DVFS_WDATA6] =		0x11c,
	[PWRAP_DVFS_ADR7] =		0x120,
	[PWRAP_DVFS_WDATA7] =		0x124,
	[PWRAP_SPMINF_STA] =		0x128,
	[PWRAP_CIPHER_KEY_SEL] =	0x12c,
	[PWRAP_CIPHER_IV_SEL] =		0x130,
	[PWRAP_CIPHER_EN] =		0x134,
	[PWRAP_CIPHER_RDY] =		0x138,
	[PWRAP_CIPHER_MODE] =		0x13c,
	[PWRAP_CIPHER_SWRST] =		0x140,
	[PWRAP_DCM_EN] =		0x144,
	[PWRAP_DCM_DBC_PRD] =		0x148,
};

static int pwrap_mt8173_init_soc_specific(struct pmic_wrapper *wrp)
{
	/* PMIC_DEWRAP enables */
	if (pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_EVENT_OUT_EN],
			0x1) ||
	    pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_EVENT_SRC_EN],
			0xffff)) {
		dev_err(wrp->dev, "enable dewrap fail\n");
		return -EFAULT;
	}

	return 0;
}

static const struct pmic_wrapper_type pwrap_mt8173 = {
	.regs = mt8173_regs,
	.type = PWRAP_MT8173,
	.arb_en_all = 0x3f,
	.int_en_all = ~(u32)(BIT(31) | BIT(1)),
	.int1_en_all = 0,
	.spi_w = PWRAP_MAN_CMD_SPI_WRITE,
	.wdt_src = PWRAP_WDT_SRC_MASK_NO_STAUPD,
	.caps = PWRAP_CAP_RESET | PWRAP_CAP_DCM,
	.init_reg_clock = pwrap_common_init_reg_clock,
	.init_soc_specific = pwrap_mt8173_init_soc_specific,
};

#define WRAP_MT8173_COMPAT	{ .compatible = "mediatek,mt8173-pwrap", .data = (ulong)&pwrap_mt8173, },
#else
#define WRAP_MT8173_COMPAT
#endif
