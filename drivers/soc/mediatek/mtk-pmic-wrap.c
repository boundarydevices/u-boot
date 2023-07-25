// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: Flora Fu, MediaTek
 */
#include <common.h>
#include <clk.h>
#include <display.h>
#include <div64.h>
#include <asm/io.h>
#include <dm/device.h>
#include <dm/read.h>
#include <dm/device_compat.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include "mtk-pmic-wrap.h"

static u32 pwrap_readl(struct pmic_wrapper *wrp, enum pwrap_regs reg)
{
	return readl(wrp->base + wrp->master->regs[reg]);
}

static void pwrap_writel(struct pmic_wrapper *wrp, u32 val, enum pwrap_regs reg)
{
	debug("%s: %d: (0x%x) 0x%x\n", __func__, reg, wrp->master->regs[reg], val);
	writel(val, wrp->base + wrp->master->regs[reg]);
}

static u32 pwrap_get_fsm_state(struct pmic_wrapper *wrp)
{
	u32 val;

	val = pwrap_readl(wrp, PWRAP_WACS2_RDATA);
	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_ARB))
		return PWRAP_GET_WACS_ARB_FSM(val);
	else
		return PWRAP_GET_WACS_FSM(val);
}

static bool pwrap_is_fsm_idle(struct pmic_wrapper *wrp)
{
	return pwrap_get_fsm_state(wrp) == PWRAP_WACS_FSM_IDLE;
}

static bool pwrap_is_fsm_vldclr(struct pmic_wrapper *wrp)
{
	return pwrap_get_fsm_state(wrp) == PWRAP_WACS_FSM_WFVLDCLR;
}

/*
 * Timeout issue sometimes caused by the last read command
 * failed because pmic wrap could not got the FSM_VLDCLR
 * in time after finishing WACS2_CMD. It made state machine
 * still on FSM_VLDCLR and timeout next time.
 * Check the status of FSM and clear the vldclr to recovery the
 * error.
 */
static inline void pwrap_leave_fsm_vldclr(struct pmic_wrapper *wrp)
{
	if (pwrap_is_fsm_vldclr(wrp))
		pwrap_writel(wrp, 1, PWRAP_WACS2_VLDCLR);
}

static bool pwrap_is_sync_idle(struct pmic_wrapper *wrp)
{
	return pwrap_readl(wrp, PWRAP_WACS2_RDATA) & PWRAP_STATE_SYNC_IDLE0;
}

static bool pwrap_is_fsm_idle_and_sync_idle(struct pmic_wrapper *wrp)
{
	u32 val = pwrap_readl(wrp, PWRAP_WACS2_RDATA);

	return (PWRAP_GET_WACS_FSM(val) == PWRAP_WACS_FSM_IDLE) &&
		(val & PWRAP_STATE_SYNC_IDLE0);
}

static int pwrap_wait_for_state(struct pmic_wrapper *wrp,
		bool (*fp)(struct pmic_wrapper *))
{
	unsigned long timeout;

	timeout = timer_get_us() + 10000;

	do {
		if (time_after(timer_get_us(), timeout))
			return fp(wrp) ? 0 : -ETIMEDOUT;
		if (fp(wrp))
			return 0;
	} while (1);
}

static int pwrap_read(struct pmic_wrapper *wrp, u32 adr, u32 *rdata)
{
	return wrp->slave->pwrap_read(wrp, adr, rdata);
}

static int pwrap_write(struct pmic_wrapper *wrp, u32 adr, u32 wdata)
{
	return wrp->slave->pwrap_write(wrp, adr, wdata);
}

static int pwrap_regmap_read(struct regmap *map, uint offset, void *valp, size_t val_len)
{
	return pwrap_read(map->priv, offset, valp);
}

static int pwrap_regmap_write(struct regmap *map, uint offset, const void *valp, size_t val_len)
{
	u32 val = 0;

	switch (val_len) {
	case REGMAP_SIZE_8:
		val = *(u8 *)valp;
		break;
	case REGMAP_SIZE_16:
		val = *(u16 *)valp;
		break;
	case REGMAP_SIZE_32:
		val = *(u32 *)valp;
		break;
	}
	return pwrap_write(map->priv, offset, val);
}

static int pwrap_reset_spislave(struct pmic_wrapper *wrp)
{
	int ret, i;

	pwrap_writel(wrp, 0, PWRAP_HIPRIO_ARB_EN);
	pwrap_writel(wrp, 0, PWRAP_WRAP_EN);
	pwrap_writel(wrp, 1, PWRAP_MUX_SEL);
	pwrap_writel(wrp, 1, PWRAP_MAN_EN);
	pwrap_writel(wrp, 0, PWRAP_DIO_EN);

	pwrap_writel(wrp, wrp->master->spi_w | PWRAP_MAN_CMD_OP_CSL,
			PWRAP_MAN_CMD);
	pwrap_writel(wrp, wrp->master->spi_w | PWRAP_MAN_CMD_OP_OUTS,
			PWRAP_MAN_CMD);
	pwrap_writel(wrp, wrp->master->spi_w | PWRAP_MAN_CMD_OP_CSH,
			PWRAP_MAN_CMD);

	for (i = 0; i < 4; i++)
		pwrap_writel(wrp, wrp->master->spi_w | PWRAP_MAN_CMD_OP_OUTS,
				PWRAP_MAN_CMD);

	ret = pwrap_wait_for_state(wrp, pwrap_is_sync_idle);
	if (ret) {
		dev_err(wrp->dev, "%s fail, ret=%d\n", __func__, ret);
		return ret;
	}

	pwrap_writel(wrp, 0, PWRAP_MAN_EN);
	pwrap_writel(wrp, 0, PWRAP_MUX_SEL);

	return 0;
}

/*
 * pwrap_init_sidly - configure serial input delay
 *
 * This configures the serial input delay. We can configure 0, 2, 4 or 6ns
 * delay. Do a read test with all possible values and chose the best delay.
 */
static int pwrap_init_sidly(struct pmic_wrapper *wrp)
{
	u32 rdata;
	u32 i;
	u32 pass = 0;
	signed char dly[16] = {
		-1, 0, 1, 0, 2, -1, 1, 1, 3, -1, -1, -1, 3, -1, 2, 1
	};

	for (i = 0; i < 4; i++) {
		pwrap_writel(wrp, i, PWRAP_SIDLY);
		pwrap_read(wrp, wrp->slave->dew_regs[PWRAP_DEW_READ_TEST],
			   &rdata);
		if (rdata == PWRAP_DEW_READ_TEST_VAL) {
			dev_dbg(wrp->dev, "[Read Test] pass, SIDLY=%x\n", i);
			pass |= 1 << i;
		}
	}

	if (dly[pass] < 0) {
		dev_err(wrp->dev, "sidly pass range 0x%x not continuous\n",
				pass);
		return -EIO;
	}

	pwrap_writel(wrp, dly[pass], PWRAP_SIDLY);

	return 0;
}

static int pwrap_init_dual_io(struct pmic_wrapper *wrp)
{
	int ret;
	u32 rdata;

	/* Enable dual IO mode */
	pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_DIO_EN], 1);

	/* Check IDLE & INIT_DONE in advance */
	ret = pwrap_wait_for_state(wrp,
				   pwrap_is_fsm_idle_and_sync_idle);
	if (ret) {
		dev_err(wrp->dev, "%s fail, ret=%d\n", __func__, ret);
		return ret;
	}

	pwrap_writel(wrp, 1, PWRAP_DIO_EN);

	/* Read Test */
	pwrap_read(wrp,
		   wrp->slave->dew_regs[PWRAP_DEW_READ_TEST], &rdata);
	if (rdata != PWRAP_DEW_READ_TEST_VAL) {
		dev_err(wrp->dev,
			"Read failed on DIO mode: 0x%04x!=0x%04x\n",
			PWRAP_DEW_READ_TEST_VAL, rdata);
		return -EFAULT;
	}

	return 0;
}

/*
 * pwrap_init_chip_select_ext is used to configure CS extension time for each
 * phase during data transactions on the pwrap bus.
 */
static void pwrap_init_chip_select_ext(struct pmic_wrapper *wrp, u8 hext_write,
				       u8 hext_read, u8 lext_start,
				       u8 lext_end)
{
	/*
	 * After finishing a write and read transaction, extends CS high time
	 * to be at least xT of BUS CLK as hext_write and hext_read specifies
	 * respectively.
	 */
	pwrap_writel(wrp, hext_write, PWRAP_CSHEXT_WRITE);
	pwrap_writel(wrp, hext_read, PWRAP_CSHEXT_READ);

	/*
	 * Extends CS low time after CSL and before CSH command to be at
	 * least xT of BUS CLK as lext_start and lext_end specifies
	 * respectively.
	 */
	pwrap_writel(wrp, lext_start, PWRAP_CSLEXT_START);
	pwrap_writel(wrp, lext_end, PWRAP_CSLEXT_END);
}

static int pwrap_common_init_reg_clock(struct pmic_wrapper *wrp)
{
	switch (wrp->master->type) {
	case PWRAP_MT8173:
		pwrap_init_chip_select_ext(wrp, 0, 4, 2, 2);
		break;
	case PWRAP_MT8135:
		pwrap_writel(wrp, 0x4, PWRAP_CSHEXT);
		pwrap_init_chip_select_ext(wrp, 0, 4, 0, 0);
		break;
	default:
		break;
	}

	return 0;
}

static bool pwrap_is_cipher_ready(struct pmic_wrapper *wrp)
{
	return pwrap_readl(wrp, PWRAP_CIPHER_RDY) & 1;
}

static bool pwrap_is_pmic_cipher_ready(struct pmic_wrapper *wrp)
{
	u32 rdata;
	int ret;

	ret = pwrap_read(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_RDY],
			 &rdata);
	if (ret)
		return false;

	return rdata == 1;
}

static int pwrap_init_cipher(struct pmic_wrapper *wrp)
{
	int ret;
	u32 rdata = 0;

	pwrap_writel(wrp, 0x1, PWRAP_CIPHER_SWRST);
	pwrap_writel(wrp, 0x0, PWRAP_CIPHER_SWRST);
	pwrap_writel(wrp, 0x1, PWRAP_CIPHER_KEY_SEL);
	pwrap_writel(wrp, 0x2, PWRAP_CIPHER_IV_SEL);

	switch (wrp->master->type) {
	case PWRAP_MT8135:
		pwrap_writel(wrp, 1, PWRAP_CIPHER_LOAD);
		pwrap_writel(wrp, 1, PWRAP_CIPHER_START);
		break;
	case PWRAP_MT2701:
	case PWRAP_MT6765:
	case PWRAP_MT6779:
	case PWRAP_MT6797:
	case PWRAP_MT8173:
	case PWRAP_MT8365:
	case PWRAP_MT8516:
		pwrap_writel(wrp, 1, PWRAP_CIPHER_EN);
		break;
	case PWRAP_MT7622:
		pwrap_writel(wrp, 0, PWRAP_CIPHER_EN);
		break;
	case PWRAP_MT6873:
	case PWRAP_MT8183:
	case PWRAP_MT8188:
	case PWRAP_MT8195:
		break;
	}

	/* Config cipher mode @PMIC */
	pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_SWRST], 0x1);
	pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_SWRST], 0x0);
	pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_KEY_SEL], 0x1);
	pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_IV_SEL], 0x2);

	switch (wrp->slave->type) {
	case PMIC_MT6397:
		pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_LOAD],
			    0x1);
		pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_START],
			    0x1);
		break;
	case PMIC_MT6323:
	case PMIC_MT6351:
	case PMIC_MT6357:
		pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_EN],
			    0x1);
		break;
	default:
		break;
	}

	/* wait for cipher data ready@AP */
	ret = pwrap_wait_for_state(wrp, pwrap_is_cipher_ready);
	if (ret) {
		dev_err(wrp->dev, "cipher data ready@AP fail, ret=%d\n", ret);
		return ret;
	}

	/* wait for cipher data ready@PMIC */
	ret = pwrap_wait_for_state(wrp, pwrap_is_pmic_cipher_ready);
	if (ret) {
		dev_err(wrp->dev,
			"timeout waiting for cipher data ready@PMIC\n");
		return ret;
	}

	/* wait for cipher mode idle */
	pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CIPHER_MODE], 0x1);
	ret = pwrap_wait_for_state(wrp, pwrap_is_fsm_idle_and_sync_idle);
	if (ret) {
		dev_err(wrp->dev, "cipher mode idle fail, ret=%d\n", ret);
		return ret;
	}

	pwrap_writel(wrp, 1, PWRAP_CIPHER_MODE);

	/* Write Test */
	if (pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_WRITE_TEST],
			PWRAP_DEW_WRITE_TEST_VAL) ||
	    pwrap_read(wrp, wrp->slave->dew_regs[PWRAP_DEW_WRITE_TEST],
		       &rdata) ||
	    (rdata != PWRAP_DEW_WRITE_TEST_VAL)) {
		dev_err(wrp->dev, "rdata=0x%04X\n", rdata);
		return -EFAULT;
	}

	return 0;
}

static int pwrap_init_security(struct pmic_wrapper *wrp)
{
	int ret;

	/* Enable encryption */
	ret = pwrap_init_cipher(wrp);
	if (ret)
		return ret;

	/* Signature checking - using CRC */
	if (pwrap_write(wrp,
			wrp->slave->dew_regs[PWRAP_DEW_CRC_EN], 0x1))
		return -EFAULT;

	pwrap_writel(wrp, 0x1, PWRAP_CRC_EN);
	pwrap_writel(wrp, 0x0, PWRAP_SIG_MODE);
	pwrap_writel(wrp, wrp->slave->dew_regs[PWRAP_DEW_CRC_VAL],
		     PWRAP_SIG_ADR);
	pwrap_writel(wrp,
		     wrp->master->arb_en_all, PWRAP_HIPRIO_ARB_EN);

	return 0;
}

static int pwrap_init(struct pmic_wrapper *wrp)
{
	int ret;

	if (wrp->rstc_bridge)
		reset_assert(wrp->rstc_bridge);
	if (wrp->rstc)
		reset_assert(wrp->rstc);

	if (wrp->rstc)
		reset_deassert(wrp->rstc);
	if (wrp->rstc_bridge)
		reset_deassert(wrp->rstc_bridge);

	if (wrp->master->type == PWRAP_MT8173) {
		/* Enable DCM */
		pwrap_writel(wrp, 3, PWRAP_DCM_EN);
		pwrap_writel(wrp, 0, PWRAP_DCM_DBC_PRD);
	}

	if (HAS_CAP(wrp->slave->caps, PWRAP_SLV_CAP_SPI)) {
		/* Reset SPI slave */
		ret = pwrap_reset_spislave(wrp);
		if (ret)
			return ret;
	}

	pwrap_writel(wrp, 1, PWRAP_WRAP_EN);

	pwrap_writel(wrp, wrp->master->arb_en_all, PWRAP_HIPRIO_ARB_EN);

	pwrap_writel(wrp, 1, PWRAP_WACS2_EN);

	ret = wrp->master->init_reg_clock(wrp);
	if (ret)
		return ret;

	if (HAS_CAP(wrp->slave->caps, PWRAP_SLV_CAP_SPI)) {
		/* Setup serial input delay */
		ret = pwrap_init_sidly(wrp);
		if (ret)
			return ret;
	}

	if (HAS_CAP(wrp->slave->caps, PWRAP_SLV_CAP_DUALIO)) {
		/* Enable dual I/O mode */
		ret = pwrap_init_dual_io(wrp);
		if (ret)
			return ret;
	}

	if (HAS_CAP(wrp->slave->caps, PWRAP_SLV_CAP_SECURITY)) {
		/* Enable security on bus */
		ret = pwrap_init_security(wrp);
		if (ret)
			return ret;
	}

	if (wrp->master->type == PWRAP_MT8135)
		pwrap_writel(wrp, 0x7, PWRAP_RRARB_EN);

	pwrap_writel(wrp, 0x1, PWRAP_WACS0_EN);
	pwrap_writel(wrp, 0x1, PWRAP_WACS1_EN);
	pwrap_writel(wrp, 0x1, PWRAP_WACS2_EN);
	pwrap_writel(wrp, 0x5, PWRAP_STAUPD_PRD);
	pwrap_writel(wrp, 0xff, PWRAP_STAUPD_GRPEN);

	if (wrp->master->init_soc_specific) {
		ret = wrp->master->init_soc_specific(wrp);
		if (ret)
			return ret;
	}

	/* Setup the init done registers */
	pwrap_writel(wrp, 1, PWRAP_INIT_DONE2);
	pwrap_writel(wrp, 1, PWRAP_INIT_DONE0);
	pwrap_writel(wrp, 1, PWRAP_INIT_DONE1);

	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_BRIDGE)) {
		writel(1, wrp->bridge_base + PWRAP_MT8135_BRIDGE_INIT_DONE3);
		writel(1, wrp->bridge_base + PWRAP_MT8135_BRIDGE_INIT_DONE4);
	}

	return 0;
}

static const struct regmap_ops pwrap_regmap_ops = {
	.read = pwrap_regmap_read,
	.write = pwrap_regmap_write,
};

#include "mt2701.c"
#include "mt6323.c"
#include "mt6351.c"
#include "mt6357.c"
#include "mt6358.c"
#include "mt6359.c"
#include "mt6380.c"
#include "mt6397.c"
#include "mt6765.c"
#include "mt6779.c"
#include "mt6797.c"
#include "mt6873.c"
#include "mt7622.c"
#include "mt8135.c"
#include "mt8173.c"
#include "mt8183.c"
#include "mt8188.c"
#include "mt8195.c"
#include "mt8365.c"
#include "mt8516.c"

#ifdef NEED_PWRAP_REGMAP16
static int pwrap_read16(struct pmic_wrapper *wrp, u32 adr, u32 *rdata)
{
	int ret;
	u32 val;

	ret = pwrap_wait_for_state(wrp, pwrap_is_fsm_idle);
	if (ret) {
		pwrap_leave_fsm_vldclr(wrp);
		debug("%s: ret=%d\n", __func__, ret);
		return ret;
	}

	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_ARB))
		val = adr;
	else
		val = (adr >> 1) << 16;
	pwrap_writel(wrp, val, PWRAP_WACS2_CMD);

	ret = pwrap_wait_for_state(wrp, pwrap_is_fsm_vldclr);
	if (ret) {
		debug("%s: b ret=%d\n", __func__, ret);
		return ret;
	}
	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_ARB))
		val = pwrap_readl(wrp, PWRAP_SWINF_2_RDATA_31_0);
	else
		val = pwrap_readl(wrp, PWRAP_WACS2_RDATA);
	*rdata = PWRAP_GET_WACS_RDATA(val);

	pwrap_writel(wrp, 1, PWRAP_WACS2_VLDCLR);
	debug("%s: 0x%x: 0x%x\n", __func__, adr, *rdata);

	return 0;
}

static int pwrap_write16(struct pmic_wrapper *wrp, u32 adr, u32 wdata)
{
	int ret;

	debug("%s: 0x%x: 0x%x\n", __func__, adr, wdata);
	ret = pwrap_wait_for_state(wrp, pwrap_is_fsm_idle);
	if (ret) {
		pwrap_leave_fsm_vldclr(wrp);
		return ret;
	}

	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_ARB)) {
		pwrap_writel(wrp, wdata, PWRAP_SWINF_2_WDATA_31_0);
		pwrap_writel(wrp, BIT(29) | adr, PWRAP_WACS2_CMD);
	} else {
		pwrap_writel(wrp, BIT(31) | ((adr >> 1) << 16) | wdata,
			     PWRAP_WACS2_CMD);
	}

	return 0;
}

static const struct regmap_config pwrap_regmap_config16 = {
	.width = REGMAP_SIZE_16,
	.r_start = 0,
	.r_size = 0xffff,
	.ops = &pwrap_regmap_ops,
};
#endif

#ifdef NEED_PWRAP_REGMAP32
static int pwrap_read32(struct pmic_wrapper *wrp, u32 adr, u32 *rdata)
{
	int ret, msb;

	*rdata = 0;
	for (msb = 0; msb < 2; msb++) {
		ret = pwrap_wait_for_state(wrp, pwrap_is_fsm_idle);
		if (ret) {
			pwrap_leave_fsm_vldclr(wrp);
			return ret;
		}

		pwrap_writel(wrp, ((msb << 30) | (adr << 16)),
			     PWRAP_WACS2_CMD);

		ret = pwrap_wait_for_state(wrp, pwrap_is_fsm_vldclr);
		if (ret)
			return ret;

		*rdata += (PWRAP_GET_WACS_RDATA(pwrap_readl(wrp,
			   PWRAP_WACS2_RDATA)) << (16 * msb));

		pwrap_writel(wrp, 1, PWRAP_WACS2_VLDCLR);
	}

	return 0;
}

static int pwrap_write32(struct pmic_wrapper *wrp, u32 adr, u32 wdata)
{
	int ret, msb, rdata;

	for (msb = 0; msb < 2; msb++) {
		ret = pwrap_wait_for_state(wrp, pwrap_is_fsm_idle);
		if (ret) {
			pwrap_leave_fsm_vldclr(wrp);
			return ret;
		}

		pwrap_writel(wrp, (1 << 31) | (msb << 30) | (adr << 16) |
			     ((wdata >> (msb * 16)) & 0xffff),
			     PWRAP_WACS2_CMD);

		/*
		 * The pwrap_read operation is the requirement of hardware used
		 * for the synchronization between two successive 16-bit
		 * pwrap_writel operations composing one 32-bit bus writing.
		 * Otherwise, we'll find the result fails on the lower 16-bit
		 * pwrap writing.
		 */
		if (!msb)
			pwrap_read(wrp, adr, &rdata);
	}

	return 0;
}

static const struct regmap_config pwrap_regmap_config32 = {
	.width = REGMAP_SIZE_32,
	.r_start = 0,
	.r_size = 0xffff,
	.ops = &pwrap_regmap_ops,
};
#endif

static const struct udevice_id of_slave_match_tbl[] = {
	MT6323_COMPAT
	MT6351_COMPAT
	MT6357_COMPAT
	MT6358_COMPAT
	MT6359_COMPAT
	MT6380_COMPAT
	MT6397_COMPAT
	{
		/* sentinel */
	}
};

static const struct udevice_id of_pwrap_match_tbl[] = {
	WRAP_MT2701_COMPAT
	WRAP_MT6765_COMPAT
	WRAP_MT6779_COMPAT
	WRAP_MT6797_COMPAT
	WRAP_MT6873_COMPAT
	WRAP_MT7622_COMPAT
	WRAP_MT8135_COMPAT
	WRAP_MT8173_COMPAT
	WRAP_MT8183_COMPAT
	WRAP_MT8188_COMPAT
	WRAP_MT8195_COMPAT
	WRAP_MT8365_COMPAT
	WRAP_MT8516_COMPAT
	{
		/* sentinel */
	}
};

static int of_match_node(const struct udevice_id *of_match,
		const struct udevice_id **of_idp,
		ofnode node)
{
	const char *compat;
	const char *compat_list;
	int i, compat_length;
	const struct udevice_id *p;

	if (!of_match)
		return -ENOENT;
	if (!ofnode_valid(node))
		return -ENOENT;

	compat_list = ofnode_get_property(node, "compatible", &compat_length);

	for (i = 0; i < compat_length; i += strlen(compat) + 1) {
		compat = compat_list + i;
		p = of_match;

		while (p->compatible) {
			if (!strcmp(p->compatible, compat)) {
				*of_idp = of_match;
				return 0;
			}
			of_match++;
		}
	}
	return -ENOENT;
}

static int pwrap_probe(struct udevice *dev)
{
	int ret;
	u32 mask_done;
	struct pmic_wrapper *wrp = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	ofnode child = ofnode_first_subnode(np);
	const struct udevice_id *of_slave_id = NULL;
	struct resource res;

	debug("%s: %s\n", __func__, dev->name);
	if (ofnode_valid(child))
		of_match_node(of_slave_match_tbl, &of_slave_id, child);

	if (!of_slave_id) {
		dev_err(dev, "slave pmic should be defined in dts\n");
		return -EINVAL;
	}

	wrp->master = (const struct pmic_wrapper_type *)dev_get_driver_data(dev);
	wrp->slave = (const struct pwrap_slv_type *)of_slave_id->data;
	wrp->dev = dev;

	ret = ofnode_read_resource_byname(np, "pwrap", &res);
	if (ret)
		return ret;
	wrp->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (IS_ERR(wrp->base))
		return PTR_ERR(wrp->base);

	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_RESET)) {
		wrp->rstc = devm_reset_control_get(dev, "pwrap");
		if (IS_ERR(wrp->rstc)) {
			ret = PTR_ERR(wrp->rstc);
			dev_err(dev, "cannot get pwrap reset: %d\n", ret);
			return ret;
		}
	}

	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_BRIDGE)) {
		ret = ofnode_read_resource_byname(np, "pwrap-bridge", &res);
		if (ret)
			return ret;
		wrp->bridge_base = devm_ioremap(dev, res.start, resource_size(&res));
		if (IS_ERR(wrp->bridge_base))
			return PTR_ERR(wrp->bridge_base);

		wrp->rstc_bridge = devm_reset_control_get(dev,
							  "pwrap-bridge");
		if (IS_ERR(wrp->rstc_bridge)) {
			ret = PTR_ERR(wrp->rstc_bridge);
			dev_err(dev,
				"cannot get pwrap-bridge reset: %d\n", ret);
			return ret;
		}
	}

	wrp->clk_spi = devm_clk_get(dev, "spi");
	if (IS_ERR(wrp->clk_spi)) {
		dev_err(dev, "failed to get SPI clock: %ld\n",
			PTR_ERR(wrp->clk_spi));
		return PTR_ERR(wrp->clk_spi);
	}

	wrp->clk_wrap = devm_clk_get(dev, "wrap");
	if (IS_ERR(wrp->clk_wrap)) {
		dev_err(dev, "failed to get WRAP clock: %ld\n",
			PTR_ERR(wrp->clk_wrap));
		return PTR_ERR(wrp->clk_wrap);
	}

	wrp->clk_wrap_sys = devm_clk_get_optional(dev, "wrap_sys");
	wrp->clk_wrap_tmr = devm_clk_get_optional(dev, "wrap_tmr");

	ret = clk_prepare_enable(wrp->clk_spi);
	if (ret)
		return ret;

	ret = clk_prepare_enable(wrp->clk_wrap);
	if (ret)
		goto err_out1;

	ret = clk_prepare_enable(wrp->clk_wrap_sys);
	if (ret)
		goto err_out2;

	ret = clk_prepare_enable(wrp->clk_wrap_tmr);
	if (ret)
		goto err_out3;

	/* Enable internal dynamic clock */
	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_DCM)) {
		pwrap_writel(wrp, 1, PWRAP_DCM_EN);
		pwrap_writel(wrp, 0, PWRAP_DCM_DBC_PRD);
	}

	/*
	 * The PMIC could already be initialized by the bootloader.
	 * Skip initialization here in this case.
	 */
	if (!pwrap_readl(wrp, PWRAP_INIT_DONE2)) {
		ret = pwrap_init(wrp);
		if (ret) {
			dev_err(dev, "init failed with %d\n", ret);
			goto err_out4;
		}
	}

	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_ARB))
		mask_done = PWRAP_STATE_INIT_DONE1;
	else
		mask_done = PWRAP_STATE_INIT_DONE0;

	if (!(pwrap_readl(wrp, PWRAP_WACS2_RDATA) & mask_done)) {
		dev_err(dev, "initialization isn't finished\n");
		ret = -ENODEV;
		goto err_out4;
	}

	/* Initialize watchdog, may not be done by the bootloader */
	if (!HAS_CAP(wrp->master->caps, PWRAP_CAP_ARB))
		pwrap_writel(wrp, 0xf, PWRAP_WDT_UNIT);

	/*
	 * Since STAUPD was not used on mt8173 platform,
	 * so STAUPD of WDT_SRC which should be turned off
	 */
	pwrap_writel(wrp, wrp->master->wdt_src, PWRAP_WDT_SRC_EN);
	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_WDT_SRC1))
		pwrap_writel(wrp, wrp->master->wdt_src, PWRAP_WDT_SRC_EN_1);

	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_ARB))
		pwrap_writel(wrp, 0x3, PWRAP_TIMER_EN);
	else
		pwrap_writel(wrp, 0x1, PWRAP_TIMER_EN);

	pwrap_writel(wrp, wrp->master->int_en_all, PWRAP_INT_EN);
	/*
	 * We add INT1 interrupt to handle starvation and request exception
	 * If we support it, we should enable it here.
	 */
	if (HAS_CAP(wrp->master->caps, PWRAP_CAP_INT1_EN))
		pwrap_writel(wrp, wrp->master->int1_en_all, PWRAP_INT1_EN);

	wrp->regmap = devm_regmap_init(dev, NULL, NULL, wrp->slave->regmap);
	if (IS_ERR(wrp->regmap)) {
		ret = PTR_ERR(wrp->regmap);
		goto err_out2;
	}
	wrp->regmap->priv = wrp;
	return 0;

err_out4:
	clk_disable_unprepare(wrp->clk_wrap_tmr);
err_out3:
	clk_disable_unprepare(wrp->clk_wrap_sys);
err_out2:
	clk_disable_unprepare(wrp->clk_wrap);
err_out1:
	clk_disable_unprepare(wrp->clk_spi);

	return ret;
}

U_BOOT_DRIVER(mtk_pmic_wrap) = {
	.name = "mt-pmic-pwrap",
	.id = UCLASS_MISC,
	.of_match = of_pwrap_match_tbl,
	.probe = pwrap_probe,
	.priv_auto = sizeof(struct pmic_wrapper),
};
