// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <edid.h>
#include <i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <power/regulator.h>
#include <video_bridge.h>

#define HDMI_INFOFRAME_HEADER_SIZE  4
#define HDMI_AVI_INFOFRAME_SIZE    13

#define IT66121_EDID_SLEEP			20000
#define IT66121_EDID_TIMEOUT			200000

#define IT66121_VENDOR_ID0			0x54
#define IT66121_VENDOR_ID1			0x49
#define IT66121_DEVICE_ID0			0x12
#define IT66121_DEVICE_ID1			0x06
#define IT66121_DEVICE_MASK			0x0F
#define IT66121_EDID_FIFO_SIZE			32

#define IT66121_SW_RST_REG			0x04
#define IT66121_SW_RST_REF			BIT(5)
#define IT66121_SW_RST_VID			BIT(3)
#define IT66121_SW_RST_HDCP			BIT(0)

#define IT66121_INT_REG				0x05
#define IT66121_INT_TX_CLK_OFF			BIT(0)

#define IT66121_INT_STATUS1_REG			0x06
#define IT66121_INT_STATUS1_DDC_BUSHANG		BIT(2)

#define IT66121_CLK_BANK_REG			0x0F
#define IT66121_CLK_BANK_PWROFF_RCLK		BIT(6)
#define IT66121_CLK_BANK_PWROFF_TXCLK		BIT(4)

#define IT66121_MASTER_SEL_REG			0x10
#define IT66121_MASTER_SEL_HOST			BIT(0)

#define IT66121_DDC_HEADER_REG			0x11
#define IT66121_DDC_HEADER_EDID			0xA0

#define IT66121_DDC_OFFSET_REG			0x12

#define IT66121_DDC_BYTE_REG			0x13

#define IT66121_DDC_SEGMENT_REG			0x14

#define IT66121_DDC_COMMAND_REG			0x15
#define IT66121_DDC_COMMAND_EDID_READ		0x3
#define IT66121_DDC_COMMAND_FIFO_CLR		0x9
#define IT66121_DDC_COMMAND_ABORT		0xF

#define IT66121_DDC_STATUS_REG			0x16
#define IT66121_DDC_STATUS_NOACK		BIT(5)
#define IT66121_DDC_STATUS_WAIT_BUS		BIT(4)
#define IT66121_DDC_STATUS_ARBI_LOSE		BIT(3)

#define IT66121_DDC_RD_FIFO_REG			0x17

#define IT66121_HDCP_REG			0x20
#define IT66121_HDCP_CPDESIRED			BIT(0)

#define IT66121_AFE_DRV_REG			0x61
#define IT66121_AFE_DRV_PWD			BIT(5)
#define IT66121_AFE_DRV_RST			BIT(4)

#define IT66121_AFE_XP_REG			0x62
#define IT66121_AFE_XP_GAINBIT			BIT(7)
#define IT66121_AFE_XP_PWDPLL			BIT(6)
#define IT66121_AFE_XP_ERO			BIT(4)
#define IT66121_AFE_XP_RESETB			BIT(3)
#define IT66121_AFE_XP_PWDI			BIT(2)

#define IT66121_AFE_IP_REG			0x64
#define IT66121_AFE_IP_GAINBIT			BIT(7)
#define IT66121_AFE_IP_PWDPLL			BIT(6)
#define IT66121_AFE_IP_CKSEL_1			BIT(4)
#define IT66121_AFE_IP_ER0			BIT(3)
#define IT66121_AFE_IP_RESETB			BIT(2)
#define IT66121_AFE_IP_EC1			BIT(0)

#define IT66121_AFE_XP_EC1_REG			0x68
#define IT66121_AFE_XP_EC1_LOWCLK		BIT(4)

#define IT66121_INPUT_MODE_REG			0x70
#define IT66121_INPUT_MODE_DDR			BIT(2)

#define IT66121_INPUT_CSC_REG			0x72
#define IT66121_INPUT_CSC_NO_CONV		0x00

#define IT66121_HDMI_MODE_REG			0xC0
#define IT66121_HDMI_MODE_HDMI			BIT(0)

#define IT66121_AV_MUTE_REG			0xC1
#define IT66121_AV_MUTE_ON			BIT(0)

#define IT66121_PKT_GEN_CTRL_REG		0xC6
#define IT66121_PKT_GEN_CTRL_ON			BIT(0)
#define IT66121_PKT_GEN_CTRL_RPT		BIT(1)

#define IT66121_AVI_INFO_PKT_REG		0xCD
#define IT66121_AVI_INFO_PKT_ON			BIT(0)
#define IT66121_AVI_INFO_PKT_RPT		BIT(1)

#define IT66121_AVIINFO_DB1_REG			0x158
#define IT66121_AVIINFO_DB2_REG			0x159
#define IT66121_AVIINFO_DB3_REG			0x15A
#define IT66121_AVIINFO_DB4_REG			0x15B
#define IT66121_AVIINFO_DB5_REG			0x15C
#define IT66121_AVIINFO_CSUM_REG		0x15D
#define IT66121_AVIINFO_DB6_REG			0x15E
#define IT66121_AVIINFO_DB7_REG			0x15F
#define IT66121_AVIINFO_DB8_REG			0x160
#define IT66121_AVIINFO_DB9_REG			0x161
#define IT66121_AVIINFO_DB10_REG		0x162
#define IT66121_AVIINFO_DB11_REG		0x163
#define IT66121_AVIINFO_DB12_REG		0x164
#define IT66121_AVIINFO_DB13_REG		0x165

struct it66121_ctx {
	struct udevice *dev;
	struct udevice *vcn33_supply;
	struct udevice *vcn18_supply;
	struct udevice *vrf12_supply;
	struct udevice *chip;
	u8 edid[EDID_SIZE];
};

static int it66121_read(struct it66121_ctx *ctx, uint reg, u8 *val)
{
	return dm_i2c_read(ctx->chip, reg, val, sizeof(u8));
}

static int it66121_write(struct it66121_ctx *ctx, uint reg, u8 val)
{
	u8 tmp = val;

	return dm_i2c_write(ctx->chip, reg, &tmp, 1);
}

static int it66121_update_bits(struct it66121_ctx *ctx, uint reg, u8 mask, u8 val)
{
	u8 orig, tmp;
	int ret;

	ret = it66121_read(ctx, reg, &orig);
	if (ret)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	return dm_i2c_write(ctx->chip, reg, &tmp, 1);
}

static int it66121_power_on(struct it66121_ctx *ctx)
{
	int ret;

	ret = it66121_update_bits(ctx, IT66121_CLK_BANK_REG, IT66121_CLK_BANK_PWROFF_RCLK, 0);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_INT_REG, IT66121_INT_TX_CLK_OFF, 0);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_DRV_REG, IT66121_AFE_DRV_PWD, 0);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_XP_REG,
				  IT66121_AFE_XP_PWDI | IT66121_AFE_XP_PWDPLL, 0);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_IP_REG, IT66121_AFE_IP_PWDPLL, 0);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_DRV_REG, IT66121_AFE_DRV_RST, 0);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_XP_REG, IT66121_AFE_XP_RESETB,
				  IT66121_AFE_XP_RESETB);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_IP_REG, IT66121_AFE_IP_RESETB,
				  IT66121_AFE_IP_RESETB);
	if (ret)
		return ret;

	return ret;
}

static int it66121_soft_reset(struct it66121_ctx *ctx)
{
	return it66121_update_bits(ctx, IT66121_SW_RST_REG, IT66121_SW_RST_REF,
				   IT66121_SW_RST_REF);
}

static int it66121_preamble_ddc(struct it66121_ctx *ctx)
{
	return it66121_write(ctx, IT66121_MASTER_SEL_REG, IT66121_MASTER_SEL_HOST);
}

static inline int it66121_wait_ddc_ready(struct it66121_ctx *ctx)
{
	u64 timeout_us = IT66121_EDID_TIMEOUT;
	u64 delay = IT66121_EDID_SLEEP;
	bool complete = false;
	u8 val;
	int ret;

	do {
		ret = it66121_read(ctx, IT66121_DDC_STATUS_REG, &val);
		if (ret) {
			dev_err(ctx->dev, "status: %d\n", ret);
			return ret;
		}

		if (val)
			complete = true;

		udelay(delay);
		timeout_us -= delay;
	} while (!complete && timeout_us);

	if (val & (IT66121_DDC_STATUS_NOACK | IT66121_DDC_STATUS_WAIT_BUS |
		   IT66121_DDC_STATUS_ARBI_LOSE))
		return -EAGAIN;

	return 0;
}

static int it66121_clear_ddc_fifo(struct it66121_ctx *ctx)
{
	int ret;

	ret = it66121_preamble_ddc(ctx);
	if (ret)
		return ret;

	return it66121_write(ctx, IT66121_DDC_COMMAND_REG, IT66121_DDC_COMMAND_FIFO_CLR);
}

static int it66121_abort_ddc_ops(struct it66121_ctx *ctx)
{
	int ret;
	u8 swreset, cpdesire;

	ret = it66121_read(ctx, IT66121_SW_RST_REG, &swreset);
	if (ret)
		return ret;

	ret = it66121_read(ctx, IT66121_HDCP_REG, &cpdesire);
	if (ret)
		return ret;

	ret = it66121_write(ctx, IT66121_HDCP_REG, cpdesire & (~IT66121_HDCP_CPDESIRED & 0xFF));
	if (ret)
		return ret;

	ret = it66121_write(ctx, IT66121_SW_RST_REG, swreset | IT66121_SW_RST_HDCP);
	if (ret)
		return ret;

	ret = it66121_preamble_ddc(ctx);
	if (ret)
		return ret;

	ret = it66121_write(ctx, IT66121_DDC_COMMAND_REG, IT66121_DDC_COMMAND_ABORT);
	if (ret)
		return ret;

	return it66121_wait_ddc_ready(ctx);
}

static int it66121_read_edid(struct udevice *dev, u8 *buf, int size)
{
	struct it66121_ctx *ctx = dev_get_priv(dev);
	u8 val;
	int remain = size;
	int offset = 0;
	int ret, cnt;

	if (size > EDID_SIZE)
		size = EDID_SIZE;

	ret = it66121_read(ctx, IT66121_INT_STATUS1_REG, &val);
	if (ret)
		return ret;

	if (val & IT66121_INT_STATUS1_DDC_BUSHANG) {
		ret = it66121_abort_ddc_ops(ctx);
		if (ret)
			return ret;
	}

	ret = it66121_clear_ddc_fifo(ctx);
	if (ret)
		return ret;

	while (remain > 0) {
		cnt = (remain > IT66121_EDID_FIFO_SIZE) ? IT66121_EDID_FIFO_SIZE : remain;
		ret = it66121_preamble_ddc(ctx);
		if (ret)
			return ret;

		ret = it66121_write(ctx, IT66121_DDC_COMMAND_REG, IT66121_DDC_COMMAND_FIFO_CLR);
		if (ret)
			return ret;

		ret = it66121_wait_ddc_ready(ctx);
		if (ret)
			return ret;

		ret = it66121_read(ctx, IT66121_INT_STATUS1_REG, &val);
		if (ret)
			return ret;

		if (val & IT66121_INT_STATUS1_DDC_BUSHANG) {
			ret = it66121_abort_ddc_ops(ctx);
			if (ret)
				return ret;
		}

		ret = it66121_preamble_ddc(ctx);
		if (ret)
			return ret;

		ret = it66121_write(ctx, IT66121_DDC_HEADER_REG, IT66121_DDC_HEADER_EDID);
		if (ret)
			return ret;

		ret = it66121_write(ctx, IT66121_DDC_OFFSET_REG, offset);
		if (ret)
			return ret;

		ret = it66121_write(ctx, IT66121_DDC_BYTE_REG, cnt);
		if (ret)
			return ret;

		ret = it66121_write(ctx, IT66121_DDC_SEGMENT_REG, 0);
		if (ret)
			return ret;

		ret = it66121_write(ctx, IT66121_DDC_COMMAND_REG, IT66121_DDC_COMMAND_EDID_READ);
		if (ret)
			return ret;

		offset += cnt;
		remain -= cnt;
		mdelay(20);

		ret = it66121_wait_ddc_ready(ctx);
		if (ret)
			return ret;

		do {
			ret = it66121_read(ctx, IT66121_DDC_RD_FIFO_REG, &val);
			if (ret)
				return ret;
			*(buf++) = val;
			cnt--;
		} while (cnt > 0);
	}

	return size;
}

static int it66121_fire_afe(struct it66121_ctx *ctx)
{
	return it66121_write(ctx, IT66121_AFE_DRV_REG, 0);
}

static int it66121_configure_afe(struct it66121_ctx *ctx)
{
	int ret;

	ret = it66121_write(ctx, IT66121_AFE_DRV_REG, IT66121_AFE_DRV_RST);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_XP_REG,
				  IT66121_AFE_XP_GAINBIT |
				  IT66121_AFE_XP_ERO |
				  IT66121_AFE_XP_RESETB,
				  IT66121_AFE_XP_ERO | IT66121_AFE_XP_RESETB);
	if (ret)
		return ret;

	ret = it66121_write(ctx, IT66121_AFE_IP_REG,
			    IT66121_AFE_IP_CKSEL_1 |
			    IT66121_AFE_IP_ER0 |
			    IT66121_AFE_IP_RESETB |
			    IT66121_AFE_IP_EC1);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_AFE_XP_EC1_REG,
				  IT66121_AFE_XP_EC1_LOWCLK,
				  IT66121_AFE_XP_EC1_LOWCLK);
	if (ret)
		return ret;

	/* Clear reset flags */
	ret = it66121_update_bits(ctx, IT66121_SW_RST_REG,
				  IT66121_SW_RST_REF | IT66121_SW_RST_VID,
				  ~(IT66121_SW_RST_REF | IT66121_SW_RST_VID) & 0xFF);
	if (ret)
		return ret;

	return it66121_fire_afe(ctx);
}

static int it66121_configure_input(struct it66121_ctx *ctx)
{
	int ret;

	ret = it66121_write(ctx, IT66121_INPUT_MODE_REG, IT66121_INPUT_MODE_DDR);
	if (ret)
		return ret;

	return it66121_write(ctx, IT66121_INPUT_CSC_REG, IT66121_INPUT_CSC_NO_CONV);
}

static int it66121_bridge_mode_set(struct it66121_ctx *ctx)
{
	const u16 avi_info_reg[HDMI_AVI_INFOFRAME_SIZE] = {
		IT66121_AVIINFO_DB1_REG,
		IT66121_AVIINFO_DB2_REG,
		IT66121_AVIINFO_DB3_REG,
		IT66121_AVIINFO_DB4_REG,
		IT66121_AVIINFO_DB5_REG,
		IT66121_AVIINFO_DB6_REG,
		IT66121_AVIINFO_DB7_REG,
		IT66121_AVIINFO_DB8_REG,
		IT66121_AVIINFO_DB9_REG,
		IT66121_AVIINFO_DB10_REG,
		IT66121_AVIINFO_DB11_REG,
		IT66121_AVIINFO_DB12_REG,
		IT66121_AVIINFO_DB13_REG
	};
	u8 avi_info_frame[HDMI_INFOFRAME_HEADER_SIZE + HDMI_AVI_INFOFRAME_SIZE] = {
		0x82, 0x02, 0x0d, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	u8 checksum = 0;
	int ret, i;

	avi_info_frame[5] = 0x18; /* 4 : 3 */

	for (i = 0; i < ARRAY_SIZE(avi_info_frame); i++)
		checksum += avi_info_frame[i];
	avi_info_frame[3] = 0x100 - checksum;

	/* Write new AVI infoframe packet */
	for (i = 0; i < HDMI_AVI_INFOFRAME_SIZE; i++) {
		ret = it66121_write(ctx, avi_info_reg[i],
				    avi_info_frame[i + HDMI_INFOFRAME_HEADER_SIZE]);
		if (ret)
			return ret;
	}
	ret = it66121_write(ctx, IT66121_AVIINFO_CSUM_REG, avi_info_frame[3]);
	if (ret)
		return ret;

	/* Enable AVI infoframe */
	ret = it66121_write(ctx, IT66121_AVI_INFO_PKT_REG,
			    IT66121_AVI_INFO_PKT_ON | IT66121_AVI_INFO_PKT_RPT);
	if (ret)
		return ret;

	/* Set TX mode to HDMI */
	ret = it66121_write(ctx, IT66121_HDMI_MODE_REG, IT66121_HDMI_MODE_HDMI);
	if (ret)
		return ret;

	ret = it66121_update_bits(ctx, IT66121_CLK_BANK_REG, IT66121_CLK_BANK_PWROFF_TXCLK,
				  IT66121_CLK_BANK_PWROFF_TXCLK);
	if (ret)
		return ret;

	ret = it66121_configure_input(ctx);
	if (ret)
		return ret;

	ret = it66121_configure_afe(ctx);
	if (ret)
		return ret;

	it66121_update_bits(ctx, IT66121_CLK_BANK_REG, IT66121_CLK_BANK_PWROFF_TXCLK,
			    ~IT66121_CLK_BANK_PWROFF_TXCLK & 0xFF);

	return 0;
}

static int it66121_bridge_enable(struct it66121_ctx *ctx)
{
	int ret;

	ret = it66121_update_bits(ctx, IT66121_AV_MUTE_REG, IT66121_AV_MUTE_ON,
				  (~IT66121_AV_MUTE_ON & 0xFF));
	if (ret)
		return ret;

	return it66121_write(ctx, IT66121_PKT_GEN_CTRL_REG,
			     IT66121_PKT_GEN_CTRL_ON | IT66121_PKT_GEN_CTRL_RPT);
}

static int it66121_attach(struct udevice *dev)
{
	struct it66121_ctx *ctx = dev_get_priv(dev);

	return it66121_bridge_enable(ctx);
}

static int it66121_read_id(struct it66121_ctx *ctx)
{
	u8 ids[4];
	int i;

	for (i = 0; i < 4; i++)
		it66121_read(ctx, i, ids + i);

	if (ids[0] != IT66121_VENDOR_ID0 ||
	    ids[1] != IT66121_VENDOR_ID1 ||
	    ids[2] != IT66121_DEVICE_ID0 ||
	    ((ids[3] & IT66121_DEVICE_MASK) != IT66121_DEVICE_ID1)) {
		dev_err(ctx->dev, "Could not read valid device/vendor id\n");
		return -ENXIO;
	}

	return 0;
}

static int it66121_probe(struct udevice *dev)
{
	struct it66121_ctx *ctx = dev_get_priv(dev);
	struct dm_i2c_chip *i2c_chip;
	int ret;

	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	ctx->dev = dev;

	/* regulators */
	ret = device_get_supply_regulator(dev, "vcn33-supply", &ctx->vcn33_supply);
	if (ret) {
		dev_err(dev, "No vcn33 supply\n");
		return ret;
	}
	regulator_set_enable(ctx->vcn33_supply, true);

	ret = device_get_supply_regulator(dev, "vcn18-supply", &ctx->vcn18_supply);
	if (ret) {
		dev_err(dev, "No vcn18 supply\n");
		return ret;
	}
	regulator_set_enable(ctx->vcn18_supply, true);

	ret = device_get_supply_regulator(dev, "vrf12-supply", &ctx->vrf12_supply);
	if (ret) {
		dev_err(dev, "No vrf12 supply\n");
		return ret;
	}
	regulator_set_enable(ctx->vrf12_supply, true);

	/* reset hardware bridge */
	video_bridge_set_active(dev, true);

	/* detect device through i2c */
	i2c_chip = dev_get_parent_plat(dev);
	if (IS_ERR(i2c_chip))
		return PTR_ERR(i2c_chip);

	ret = i2c_get_chip_for_busnum(0, i2c_chip->chip_addr, 1, &ctx->chip);
	if (ret) {
		dev_err(dev, "cannot find device\n");
		return -ENXIO;
	}

	ret = it66121_read_id(ctx);
	if (ret)
		return ret;

	ret = it66121_power_on(ctx);
	if (ret)
		return ret;

	ret = it66121_soft_reset(ctx);
	if (ret)
		return ret;

	ret = it66121_bridge_mode_set(ctx);
	if (ret)
		return ret;

	return 0;
}

struct video_bridge_ops it66121_ops = {
	.attach	   = it66121_attach,
	.read_edid = it66121_read_edid,
};

static const struct udevice_id it66121_ids[] = {
	{ .compatible = "ite,it66121" },
	{}
};

U_BOOT_DRIVER(it66121) = {
	.name	   = "it66121",
	.id	   = UCLASS_VIDEO_BRIDGE,
	.of_match  = it66121_ids,
	.probe	   = it66121_probe,
	.ops	   = &it66121_ops,
	.priv_auto = sizeof(struct it66121_ctx),
};
