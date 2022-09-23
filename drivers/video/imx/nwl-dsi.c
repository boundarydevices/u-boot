// SPDX-License-Identifier: GPL-2.0+
/*
 * NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */
#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dsi_host.h>
#include <div64.h>
#include <generic-phy.h>

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/string.h>
#include <malloc.h>

#include <panel.h>
#include <phy-mipi-dphy.h>
#include <regmap.h>
#include <syscon.h>
#include <video_bridge.h>

#include "nwl-drv.h"
#include "nwl-dsi.h"

/*
 * PKT_CONTROL format:
 * [15: 0] - word count
 * [17:16] - virtual channel
 * [23:18] - data type
 * [24]	   - LP or HS select (0 - LP, 1 - HS)
 * [25]	   - perform BTA after packet is sent
 * [26]	   - perform BTA only, no packet tx
 */
#define NWL_DSI_WC(x)		FIELD_PREP(GENMASK(15, 0), (x))
#define NWL_DSI_TX_VC(x)	FIELD_PREP(GENMASK(17, 16), (x))
#define NWL_DSI_TX_DT(x)	FIELD_PREP(GENMASK(23, 18), (x))
#define NWL_DSI_HS_SEL(x)	FIELD_PREP(GENMASK(24, 24), (x))
#define NWL_DSI_BTA_TX(x)	FIELD_PREP(GENMASK(25, 25), (x))
#define NWL_DSI_BTA_NO_TX(x)	FIELD_PREP(GENMASK(26, 26), (x))

/*
 * RX_PKT_HEADER format:
 * [15: 0] - word count
 * [21:16] - data type
 * [23:22] - virtual channel
 */
#define NWL_DSI_RX_DT(x)	FIELD_GET(GENMASK(21, 16), (x))
#define NWL_DSI_RX_VC(x)	FIELD_GET(GENMASK(23, 22), (x))

/* DSI Video mode */
#define NWL_DSI_VM_BURST_MODE_WITH_SYNC_PULSES		0
#define NWL_DSI_VM_NON_BURST_MODE_WITH_SYNC_EVENTS	BIT(0)
#define NWL_DSI_VM_BURST_MODE				BIT(1)

/* * DPI color coding */
#define NWL_DSI_DPI_16_BIT_565_PACKED	0
#define NWL_DSI_DPI_16_BIT_565_ALIGNED	1
#define NWL_DSI_DPI_16_BIT_565_SHIFTED	2
#define NWL_DSI_DPI_18_BIT_PACKED	3
#define NWL_DSI_DPI_18_BIT_ALIGNED	4
#define NWL_DSI_DPI_24_BIT		5

/* * DPI Pixel format */
#define NWL_DSI_PIXEL_FORMAT_16  0
#define NWL_DSI_PIXEL_FORMAT_18  BIT(0)
#define NWL_DSI_PIXEL_FORMAT_18L BIT(1)
#define NWL_DSI_PIXEL_FORMAT_24  (BIT(0) | BIT(1))

enum transfer_direction {
	DSI_PACKET_SEND,
	DSI_PACKET_RECEIVE,
};

struct nwl_dsi_transfer {
	const struct mipi_dsi_msg *msg;
	struct mipi_dsi_packet packet;

	int status; /* status of transmission */
	enum transfer_direction direction;
	bool need_bta;
	u8 cmd;
	u16 rx_word_count;
	size_t tx_len; /* in bytes */
	size_t rx_len; /* in bytes */
};

static int nwl_dsi_write(struct nwl_dsi *dsi, unsigned int reg, u32 val)
{
	int ret;

	ret = regmap_write(dsi->regmap, reg, val);
	if (ret < 0)
		debug("Failed to write NWL DSI reg 0x%x: %d\n", reg, ret);
	return ret;
}

static u32 nwl_dsi_read(struct nwl_dsi *dsi, u32 reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(dsi->regmap, reg, &val);
	if (ret < 0)
		debug("Failed to read NWL DSI reg 0x%x: %d\n", reg, ret);

	return val;
}

static u32 nwl_dsi_get_dpi_pixel_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return NWL_DSI_PIXEL_FORMAT_16;
	case MIPI_DSI_FMT_RGB666:
		return NWL_DSI_PIXEL_FORMAT_18L;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return NWL_DSI_PIXEL_FORMAT_18;
	case MIPI_DSI_FMT_RGB888:
		return NWL_DSI_PIXEL_FORMAT_24;
	default:
		return -EINVAL;
	}
}

static int nwl_dsi_cvt_pixels_to_hs_byte_clocks(struct nwl_dsi *dsi, int pixels, int base, int min,
		unsigned *pix_cnt, unsigned *hs_clk_cnt)
{
	int n;
	unsigned long a;

	*pix_cnt += pixels;
	a = *pix_cnt;
	a *= dsi->bitclk;
	a += (dsi->pixclock * 4);
	a /= (dsi->pixclock * 8);
	n = a;
	debug("%s:pix_cnt = %d, bitclk = %d, pixel = %d, n=%d\n", __func__,
		*pix_cnt, dsi->bitclk, dsi->pixclock, n);

	n *= dsi->lanes;
	n -= *hs_clk_cnt;

	if (n >= base + min)
		n -= base;
	else
		n = min;
	*hs_clk_cnt += n + base;

	return n;
}

static int nwl_dsi_cvt_pixels_to_hs_byte_clocks_burst(struct nwl_dsi *dsi, int pixels, int bpp,
		unsigned *pix_cnt, unsigned *hs_clk_cnt)
{
	int n;

	*pix_cnt += pixels;
	n = ((pixels * bpp) + (dsi->lanes * 8) - 1) / (dsi->lanes * 8);
	n *= dsi->lanes;
	*hs_clk_cnt += n;

	return n;
}

#define PSEC_PER_SEC 1000000000000LL
/*
 * ps2bc - Picoseconds to byte clock cycles
 */
static u32 ps2bc(struct nwl_dsi *dsi, unsigned long long ps)
{
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	return DIV_ROUND_UP(ps * dsi->timings.pixelclock.typ * bpp,
			    dsi->lanes * 8 * PSEC_PER_SEC);
}

/*
 * ui2bc - UI time periods to byte clock cycles
 */
static u32 ui2bc(unsigned int ui)
{
	return DIV_ROUND_UP(ui, BITS_PER_BYTE);
}

/*
 * us2bc - micro seconds to lp clock cycles
 */
static u32 us2lp(u32 lp_clk_rate, unsigned long us)
{
	return DIV_ROUND_UP(us * lp_clk_rate, 1000000);
}

static int nwl_dsi_config_host(struct nwl_dsi *dsi)
{
	u32 cycles;
	struct phy_configure_opts_mipi_dphy *cfg = &dsi->phy_cfg.mipi_dphy;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	debug("DSI Lanes %d\n", dsi->lanes);
	nwl_dsi_write(dsi, NWL_DSI_CFG_NUM_LANES, dsi->lanes - 1);

	if (dsi->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		nwl_dsi_write(dsi, NWL_DSI_CFG_NONCONTINUOUS_CLK, 0x01);
		nwl_dsi_write(dsi, NWL_DSI_CFG_AUTOINSERT_EOTP, 0x01);
	} else {
		nwl_dsi_write(dsi, NWL_DSI_CFG_NONCONTINUOUS_CLK, 0x00);
		nwl_dsi_write(dsi, NWL_DSI_CFG_AUTOINSERT_EOTP, 0x00);
	}

	/* values in byte clock cycles */
	cycles = ui2bc(cfg->clk_pre);
	debug("cfg_t_pre: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_T_PRE, cycles);
	cycles = ps2bc(dsi, cfg->lpx + cfg->clk_prepare + cfg->clk_zero);
	debug("cfg_tx_gap (pre): 0x%x\n", cycles);
	cycles += ui2bc(cfg->clk_pre);
	debug("cfg_tx_gap: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_T_POST, cycles);
	cycles = ps2bc(dsi, cfg->hs_exit);
	debug("cfg_tx_gap: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_TX_GAP, cycles);

	nwl_dsi_write(dsi, NWL_DSI_CFG_EXTRA_CMDS_AFTER_EOTP, 0x01);
	nwl_dsi_write(dsi, NWL_DSI_CFG_HTX_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, NWL_DSI_CFG_LRX_H_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, NWL_DSI_CFG_BTA_H_TO_COUNT, 0x00);
	/* In LP clock cycles */
	cycles = us2lp(cfg->lp_clk_rate, cfg->wakeup);
	debug("cfg_twakeup: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_TWAKEUP, cycles);

	return 0;
}

static int nwl_dsi_config_dpi(struct nwl_dsi *dsi)
{
	u32 color_format, mode;
	bool burst_mode;
	int hfront_porch, hback_porch, hactive, vfront_porch, vback_porch;
	int hsync_len, vsync_len;
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	hactive = dsi->timings.hactive.typ;
	hfront_porch = dsi->timings.hfront_porch.typ;
	hsync_len = dsi->timings.hsync_len.typ;
	hback_porch = dsi->timings.hback_porch.typ;

	vfront_porch = dsi->timings.vfront_porch.typ;
	vsync_len = dsi->timings.vsync_len.typ;
	vback_porch = dsi->timings.vback_porch.typ;

	debug("hfront_porch = %d\n", hfront_porch);
	debug("hback_porch = %d\n", hback_porch);
	debug("hsync_len = %d\n", hsync_len);
	debug("hdisplay = %d\n", dsi->timings.hactive.typ);
	debug("vfront_porch = %d\n", vfront_porch);
	debug("vback_porch = %d\n", vback_porch);
	debug("vsync_len = %d\n", vsync_len);
	debug("vactive = %d\n", dsi->timings.vactive.typ);
	debug("clock = %d kHz\n", dsi->timings.pixelclock.typ);

	color_format = nwl_dsi_get_dpi_pixel_format(dsi->format);
	if (color_format < 0) {
		debug("Invalid color format 0x%x\n",
			      dsi->format);
		return color_format;
	}
	debug("pixel fmt = %d\n", dsi->format);

	nwl_dsi_write(dsi, NWL_DSI_INTERFACE_COLOR_CODING, NWL_DSI_DPI_24_BIT);
	nwl_dsi_write(dsi, NWL_DSI_PIXEL_FORMAT, color_format);
	/*
	 * Adjusting input polarity based on the video mode results in
	 * a black screen so always pick active low:
	 * (and polarity active high in LCDIF)
	 */
	nwl_dsi_write(dsi, NWL_DSI_VSYNC_POLARITY,
		      NWL_DSI_VSYNC_POLARITY_ACTIVE_LOW);
	/*
	 * Our dsi-rgb converter needs 0 regardless of input polarity
	 */
	nwl_dsi_write(dsi, NWL_DSI_HSYNC_POLARITY,
		      NWL_DSI_HSYNC_POLARITY_ACTIVE_LOW);

	burst_mode = (dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_BURST) &&
		     !(dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE);

	if (burst_mode) {
		if (hback_porch <= 8) {
			hback_porch = 1;
			hfront_porch = 7;
			hsync_len = 4;
		} else {
			hback_porch = 2;
			hfront_porch = 7;
			hsync_len = 6;
		}
		nwl_dsi_write(dsi, NWL_DSI_VIDEO_MODE, NWL_DSI_VM_BURST_MODE);
		/*
		 *
		 * minimum of
		 * (hactive - x)/dpi_rate = hactive / mipi_rate
		 * (hactive - x) = hactive * dpi_rate / mipi_rate
		 * x = hactive - hactive * dpi_rate / mipi_rate
		 * x = hactive(1 - dpi_rate/mipi_rate)
		 *
		 * So, if dpi_rate/mipi_rate = 3/4, hactive = 1920
		 * 1920(1 - 3/4) = 1920 / 4 = 480
		 * 1280/4 = 320
		 *
		 * maximum of about h. blanking period more.(just a guess)
		 */
		nwl_dsi_write(dsi, NWL_DSI_PIXEL_FIFO_SEND_LEVEL, 0 ? 480 : 256);
	} else {
		unsigned pix_cnt = 0;
		unsigned hs_clk_cnt = 0;

		debug("%s: hbp=%d hfp=%d hsa=%d\n", __func__,
			hback_porch, hfront_porch, hsync_len);
		hback_porch = nwl_dsi_cvt_pixels_to_hs_byte_clocks(dsi, hback_porch,
				(dsi->lanes > 1) ? 14 : 11, 4, &pix_cnt,
				&hs_clk_cnt);
		nwl_dsi_cvt_pixels_to_hs_byte_clocks_burst(dsi, hactive,
				bpp, &pix_cnt, &hs_clk_cnt);
		hfront_porch = nwl_dsi_cvt_pixels_to_hs_byte_clocks(dsi, hfront_porch,
				(dsi->lanes > 1) ? 7 : 11, 2, &pix_cnt,
				&hs_clk_cnt);
		hsync_len = nwl_dsi_cvt_pixels_to_hs_byte_clocks(dsi, hsync_len,
				10, 2, &pix_cnt, &hs_clk_cnt);

		debug("%s: hbp=%d hfp=%d hsa=%d %c\n", __func__,
			hback_porch, hfront_porch, hsync_len,
			(dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) ? 'p' : 'e');
		mode = ((dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) ?
				NWL_DSI_VM_BURST_MODE_WITH_SYNC_PULSES :
				NWL_DSI_VM_NON_BURST_MODE_WITH_SYNC_EVENTS);
		nwl_dsi_write(dsi, NWL_DSI_VIDEO_MODE, mode);
		nwl_dsi_write(dsi, NWL_DSI_PIXEL_FIFO_SEND_LEVEL, hactive);
	}

	nwl_dsi_write(dsi, NWL_DSI_HFP, hfront_porch);
	nwl_dsi_write(dsi, NWL_DSI_HBP, hback_porch);
	nwl_dsi_write(dsi, NWL_DSI_HSA, hsync_len);

	nwl_dsi_write(dsi, NWL_DSI_ENABLE_MULT_PKTS, 0x0);
	nwl_dsi_write(dsi, NWL_DSI_BLLP_MODE, 0x1);
	nwl_dsi_write(dsi, NWL_DSI_USE_NULL_PKT_BLLP, 0x0);
	nwl_dsi_write(dsi, NWL_DSI_VC, 0x0);

	nwl_dsi_write(dsi, NWL_DSI_PIXEL_PAYLOAD_SIZE, dsi->timings.hactive.typ);
	nwl_dsi_write(dsi, NWL_DSI_VACTIVE, dsi->timings.vactive.typ - 1);
	nwl_dsi_write(dsi, NWL_DSI_VBP, vback_porch);
	nwl_dsi_write(dsi, NWL_DSI_VFP, vfront_porch);

	return 0;
}

static void nwl_dsi_init_interrupts(struct nwl_dsi *dsi)
{
	u32 irq_enable;

	nwl_dsi_write(dsi, NWL_DSI_IRQ_MASK, 0xffffffff);
	nwl_dsi_write(dsi, NWL_DSI_IRQ_MASK2, 0x7);

	irq_enable = ~(u32)(NWL_DSI_TX_PKT_DONE_MASK |
			    NWL_DSI_RX_PKT_HDR_RCVD_MASK |
			    NWL_DSI_TX_FIFO_OVFLW_MASK |
			    NWL_DSI_HS_TX_TIMEOUT_MASK);

	nwl_dsi_write(dsi, NWL_DSI_IRQ_MASK, irq_enable);
}

static int nwl_dsi_host_attach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);

	debug("lanes=%u, format=0x%x flags=0x%lx\n", device->lanes,
		     device->format, device->mode_flags);

	if (device->lanes < 1 || device->lanes > 4)
		return -EINVAL;

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->dsi_mode_flags = device->mode_flags;

	return 0;
}

static int nwl_dsi_host_detach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);

	dsi->lanes = 0;
	dsi->format = 0;
	dsi->dsi_mode_flags = 0;

	return 0;
}

static bool nwl_dsi_read_packet(struct nwl_dsi *dsi, u32 status)
{
	struct nwl_dsi_transfer *xfer = dsi->xfer;
	u8 *payload = xfer->msg->rx_buf;
	u32 val;
	u16 word_count;
	u8 channel;
	u8 data_type;

	xfer->status = 0;

	if (xfer->rx_word_count == 0) {
		if (!(status & NWL_DSI_RX_PKT_HDR_RCVD))
			return false;
		/* Get the RX header and parse it */
		val = nwl_dsi_read(dsi, NWL_DSI_RX_PKT_HEADER);
		word_count = NWL_DSI_WC(val);
		channel = NWL_DSI_RX_VC(val);
		data_type = NWL_DSI_RX_DT(val);

		if (channel != xfer->msg->channel) {
			debug("[%02X] Channel mismatch (%u != %u)\n",
				xfer->cmd, channel, xfer->msg->channel);
			return true;
		}

		switch (data_type) {
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			/* Fall through */
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			if (xfer->msg->rx_len > 1) {
				/* read second byte */
				payload[1] = word_count >> 8;
				++xfer->rx_len;
			}
			/* Fall through */
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
			/* Fall through */
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			if (xfer->msg->rx_len > 0) {
				/* read first byte */
				payload[0] = word_count & 0xff;
				++xfer->rx_len;
			}
			xfer->status = xfer->rx_len;
			return true;
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			word_count &= 0xff;
			debug("[%02X] DSI error report: 0x%02x\n",
				      xfer->cmd, word_count);
			xfer->status = -EPROTO;
			return true;
		}

		if (word_count > xfer->msg->rx_len) {
			debug("[%02X] Receive buffer too small: %lu (< %u)\n",
				xfer->cmd, xfer->msg->rx_len, word_count);
			return true;
		}

		xfer->rx_word_count = word_count;
	} else {
		/* Set word_count from previous header read */
		word_count = xfer->rx_word_count;
	}

	/* If RX payload is not yet received, wait for it */
	if (!(status & NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD))
		return false;

	/* Read the RX payload */
	while (word_count >= 4) {
		val = nwl_dsi_read(dsi, NWL_DSI_RX_PAYLOAD);
		payload[0] = (val >> 0) & 0xff;
		payload[1] = (val >> 8) & 0xff;
		payload[2] = (val >> 16) & 0xff;
		payload[3] = (val >> 24) & 0xff;
		payload += 4;
		xfer->rx_len += 4;
		word_count -= 4;
	}

	if (word_count > 0) {
		val = nwl_dsi_read(dsi, NWL_DSI_RX_PAYLOAD);
		switch (word_count) {
		case 3:
			payload[2] = (val >> 16) & 0xff;
			++xfer->rx_len;
			/* Fall through */
		case 2:
			payload[1] = (val >> 8) & 0xff;
			++xfer->rx_len;
			/* Fall through */
		case 1:
			payload[0] = (val >> 0) & 0xff;
			++xfer->rx_len;
			break;
		}
	}

	xfer->status = xfer->rx_len;

	return true;
}

static bool nwl_dsi_finish_transmission(struct nwl_dsi *dsi, u32 status)
{
	struct nwl_dsi_transfer *xfer = dsi->xfer;
	bool end_packet = false;

	if (!xfer)
		return false;

	if (status & NWL_DSI_TX_FIFO_OVFLW) {
		printf("%s: tx fifo overflow\n", __func__);
		return false;
	}

	if (status & NWL_DSI_HS_TX_TIMEOUT) {
		printf("%s: HS tx timeout\n", __func__);
		return false;
	}

	if (xfer->direction == DSI_PACKET_SEND &&
	    status & NWL_DSI_TX_PKT_DONE) {
		xfer->status = xfer->tx_len;
		end_packet = true;
	} else if (status & NWL_DSI_DPHY_DIRECTION &&
		   ((status & (NWL_DSI_RX_PKT_HDR_RCVD |
			       NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD)))) {
		end_packet = nwl_dsi_read_packet(dsi, status);
	}

	return end_packet;
}

static void nwl_dsi_begin_transmission(struct nwl_dsi *dsi)
{
	struct nwl_dsi_transfer *xfer = dsi->xfer;
	struct mipi_dsi_packet *pkt = &xfer->packet;
	const u8 *payload;
	size_t length;
	u16 word_count;
	u8 hs_mode;
	u32 val;
	u32 hs_workaround = 0;

	/* Send the payload, if any */
	length = pkt->payload_length;
	payload = pkt->payload;

	while (length >= 4) {
		val = *(u32 *)payload;
		hs_workaround |= !(val & 0xFFFF00);
		nwl_dsi_write(dsi, NWL_DSI_TX_PAYLOAD, val);
		payload += 4;
		length -= 4;
	}
	/* Send the rest of the payload */
	val = 0;
	switch (length) {
	case 3:
		val |= payload[2] << 16;
		/* Fall through */
	case 2:
		val |= payload[1] << 8;
		/* Fall through */
	case 1:
		if (!(val & 0xFFFF00) &&
		    (dsi->quirks & E11418_HS_MODE_QUIRK)) {
			if (length > 2) {
				hs_workaround |= 1;
			} else {
				/*
				 * Set bit 23 of unused byte of payload to avoid
				 * triggering quirk
				 */
				val |= BIT(23);
			}
		}
		val |= payload[0];
		nwl_dsi_write(dsi, NWL_DSI_TX_PAYLOAD, val);
		break;
	}
	xfer->tx_len = pkt->payload_length;

	/*
	 * Send the header
	 * header[0] = Virtual Channel + Data Type
	 * header[1] = Word Count LSB (LP) or first param (SP)
	 * header[2] = Word Count MSB (LP) or second param (SP)
	 */
	word_count = pkt->header[1] | (pkt->header[2] << 8);
	if (hs_workaround && (dsi->quirks & E11418_HS_MODE_QUIRK)) {
		debug("Using hs mode workaround for cmd 0x%x\n", xfer->cmd);
		hs_mode = 1;
	} else {
		hs_mode = (xfer->msg->flags & MIPI_DSI_MSG_USE_LPM) ? 0 : 1;
	}
	val = NWL_DSI_WC(word_count) | NWL_DSI_TX_VC(xfer->msg->channel) |
	      NWL_DSI_TX_DT(xfer->msg->type) | NWL_DSI_HS_SEL(hs_mode) |
	      NWL_DSI_BTA_TX(xfer->need_bta);
	nwl_dsi_write(dsi, NWL_DSI_PKT_CONTROL, val);

	/* Send packet command */
	nwl_dsi_write(dsi, NWL_DSI_SEND_PACKET, 0x1);
}

int nwl_dsi_poll_for_completion(struct nwl_dsi *dsi)
{
	u32 irq_status;
	bool done = false;
	ulong now;
	ulong start;
	ulong elapsed;

	now = start = get_timer(0);
	while (1) {
		irq_status = nwl_dsi_read(dsi, NWL_DSI_IRQ_STATUS);

		if (irq_status & NWL_DSI_TX_PKT_DONE ||
		    irq_status & NWL_DSI_RX_PKT_HDR_RCVD ||
		    irq_status & NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD) {
			done = nwl_dsi_finish_transmission(dsi, irq_status);
			if (!done)
				debug("%s: irq_status=0x%x cmd=0x%02x\n", __func__, irq_status, dsi->xfer->cmd);

		}
		if (done)
			break;
		elapsed = now - start;
		if (elapsed >= CONFIG_SYS_HZ/2) {
			printf("%s: timed out, irq_status=0x%x cmd=%02x\n", __func__, irq_status, dsi->xfer->cmd);
			break;
		}
		udelay(10);
		now = get_timer(0);
	}
	return 0;
}

static ssize_t nwl_dsi_host_transfer(struct mipi_dsi_host *dsi_host,
				     const struct mipi_dsi_msg *msg)
{
	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);
	struct nwl_dsi_transfer xfer;
	ssize_t ret = 0;

	/* Create packet to be sent */
	dsi->xfer = &xfer;
	ret = mipi_dsi_create_packet(&xfer.packet, msg);
	if (ret < 0) {
		dsi->xfer = NULL;
		return ret;
	}

	if ((msg->type & MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM ||
	     msg->type & MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM ||
	     msg->type & MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM ||
	     msg->type & MIPI_DSI_DCS_READ) &&
	    msg->rx_len > 0 && msg->rx_buf != NULL)
		xfer.direction = DSI_PACKET_RECEIVE;
	else
		xfer.direction = DSI_PACKET_SEND;

	xfer.need_bta = (xfer.direction == DSI_PACKET_RECEIVE);
	xfer.need_bta |= (msg->flags & MIPI_DSI_MSG_REQ_ACK) ? 1 : 0;
	xfer.msg = msg;
	xfer.status = -ETIMEDOUT;
	xfer.rx_word_count = 0;
	xfer.rx_len = 0;
	xfer.cmd = 0x00;
	if (msg->tx_len > 0)
		xfer.cmd = ((u8 *)(msg->tx_buf))[0];

	ret = clk_prepare_enable(dsi->rx_esc_clk);
	if (ret < 0) {
		debug("Failed to enable rx_esc clk: %zd\n", ret);
		return ret;
	}
	//debug("Enabled rx_esc clk @%lu Hz\n", clk_get_rate(dsi->rx_esc_clk));

	/* Initiate the DSI packet transmision */
	nwl_dsi_begin_transmission(dsi);

	nwl_dsi_poll_for_completion(dsi);

	clk_disable_unprepare(dsi->rx_esc_clk);
	dsi->xfer = NULL;

	return ret;
}

static int nwl_dsi_host_enable_frame(struct mipi_dsi_host *dsi_host)
{
	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);

	/* enable data transfer of dsim */
	nwl_dsi_start_frame(dsi);
	return 0;
}

const struct mipi_dsi_host_ops nwl_mipi_dsi_host_ops = {
	.attach = nwl_dsi_host_attach,
	.enable_frame = nwl_dsi_host_enable_frame,
	.detach = nwl_dsi_host_detach,
	.transfer = nwl_dsi_host_transfer,
};

int nwl_dsi_enable(struct nwl_dsi *dsi)
{
	union phy_configure_opts *phy_cfg = &dsi->phy_cfg;
	int ret;

	if (!dsi->lanes) {
		debug("Need DSI lanes: %d\n", dsi->lanes);
		return -EINVAL;
	}

	ret = generic_phy_init(&dsi->phy);
	if (ret < 0) {
		debug("Failed to init DSI phy: %d\n", ret);
		return ret;
	}

	ret = generic_phy_configure(&dsi->phy, phy_cfg);
	if (ret < 0) {
		debug("Failed to configure DSI phy: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(dsi->tx_esc_clk);
	if (ret < 0) {
		debug("Failed to enable tx_esc clk: %d\n",
			      ret);
		return ret;
	}
	debug("Enabled tx_esc clk @%lu Hz\n", clk_get_rate(dsi->tx_esc_clk));

	ret = nwl_dsi_config_host(dsi);
	if (ret < 0) {
		debug("Failed to set up DSI: %d", ret);
		return ret;
	}

	ret = nwl_dsi_config_dpi(dsi);
	if (ret < 0) {
		debug("Failed to set up DPI: %d", ret);
		return ret;
	}

	ret = generic_phy_power_on(&dsi->phy);
	if (ret < 0) {
		debug("Failed to power on DPHY (%d)\n", ret);
		return ret;
	}

	nwl_dsi_init_interrupts(dsi);

	return 0;
}

int nwl_dsi_disable(struct nwl_dsi *dsi)
{
	debug("Disabling clocks and phy\n");

	generic_phy_power_off(&dsi->phy);
	generic_phy_exit(&dsi->phy);

	/* Disabling the clock before the phy breaks enabling dsi again */
	clk_disable_unprepare(dsi->tx_esc_clk);

	return 0;
}
