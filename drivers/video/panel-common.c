// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */
#include <common.h>
#include <asm/gpio.h>
#include <backlight.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dt-bindings/display/simple_panel_mipi_cmds.h>
#include <i2c.h>
#include <linux/delay.h>
#include <log.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <power/regulator.h>
#include <spi.h>
#include <video_link.h>
#include "panel-sn65dsi83.h"

struct cmds {
	const u8* cmds;
	unsigned length;
};

struct interface_cmds {
	struct cmds i2c;
	struct cmds mipi;
	struct cmds spi;
};

struct panel_common {
	bool prepared;
	bool enabled;
	bool no_hpd;
	long mode_flags;

	struct mipi_dsi_device *dsi;
	u32 min_hs_clock_multiple;
	u32 mipi_dsi_multiple;

	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
	struct gpio_desc *gd_enable;
	struct {
		unsigned int prepare;
		unsigned int hpd_absent_delay;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
		unsigned int before_backlight_on;
	} delay;
	unsigned enable_high_duration_us;
	unsigned enable_low_duration_us;
	unsigned power_delay_ms;
	unsigned mipi_delay_between_cmds;
	u32 bpc;

	struct udevice *backlight;
	struct udevice *supply;
	struct udevice *linked_panel;
	struct clk *mipi_clk;

	struct gpio_desc *gpd_prepare_enable;
	struct gpio_desc *gpd_power;
	struct gpio_desc *gpd_display_enable;
	struct gpio_desc *reset;
	struct display_timing timings;

	struct udevice *spi;
	unsigned spi_max_frequency;
	struct udevice *i2c;
	unsigned i2c_max_frequency;
	unsigned i2c_address;
	unsigned char spi_9bit;
	unsigned spi_bits;
	struct interface_cmds cmds_init;
	struct interface_cmds cmds_enable;
	struct interface_cmds cmds_enable2;
	struct interface_cmds cmds_disable;
	struct panel_sn65dsi83 sn65;
	/* Keep a mulitple of 9 */
	unsigned char tx_buf[63] __attribute__((aligned(64)));
	unsigned char rx_buf[63] __attribute__((aligned(64)));

	struct gpio_desc gds_prepare_enable;
	struct gpio_desc gds_reset;
	struct gpio_desc gds_power;
	struct gpio_desc gds_enable;
	struct gpio_desc gds_display_enable;
};

#if CONFIG_IS_ENABLED(DM_SPI)
static int spi_send(struct panel_common *panel, int rx)
{
        struct spi_slave *slave;
	u8 *p = panel->tx_buf;
	int ret;
	int len;

	if (!panel->spi || !panel->spi_bits)
		return 0;

	if (panel->spi_max_frequency) {
		slave = dev_get_parent_priv(panel->spi);
		slave->max_hz = panel->spi_max_frequency;
	}
	len = (panel->spi_bits + 7) >> 3;
	ret = dm_spi_claim_bus(panel->spi);
	if (ret)
		goto err;

	panel->spi_bits = 0;
        ret = dm_spi_xfer(panel->spi, len, panel->tx_buf,
        		  (rx) ? panel->rx_buf : NULL,
                          SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret)
		goto err;

        dm_spi_release_bus(panel->spi);
	debug("spi(%d), (%d)%02x %02x %02x %02x %02x %02x  "
		"%02x %02x %02x %02x %02x %02x\n",
		ret, len, p[0], p[1], p[2], p[3], p[4], p[5],
		p[6], p[7], p[8], p[9], p[10], p[11]);
	return ret;

err:
	dm_spi_release_bus(panel->spi);
	printf("spi(%d), (%d)%02x %02x %02x %02x %02x %02x\n",
		ret, len, p[0], p[1], p[2], p[3], p[4], p[5]);
	return ret;
}

static int store_9bit(struct panel_common *panel, const u8 *p, unsigned l)
{
	int i = 0;
	u8 * buf = panel->tx_buf;
	unsigned val = 0;
	int bits = l * 9;
	int v_bits;
	int ret = 0;

	i = panel->spi_bits;
	if ((i + bits) > sizeof(panel->tx_buf) * 8) {
		ret = spi_send(panel, 0);
		i = 0;
		if (bits > sizeof(panel->tx_buf) * 8) {
			printf("too many bytes (%d)\n", l);
			return -EINVAL;
		}
	}

	panel->spi_bits = i + bits;
	buf += i >> 3;
	v_bits = (i & 7);
	if (v_bits) {
		bits += v_bits;
		val = *buf;
		val >>= (8 - v_bits);
	}
	val = (val << 9);
	v_bits += 9;
	if (l) {
		val |= *p++;
		l--;
	}
	while (bits > 0) {
		*buf++ = val >> (v_bits - 8);
		bits -= 8;
		v_bits -= 8;
		if (v_bits < 8) {
			val = (val << 9);
			v_bits += 9;
			if (l) {
				val |= 0x100;
				val |= *p++;
				l--;
			}
		}
	}
	return ret;
}

static int store_high(struct panel_common *panel, int bits)
{
	int i;
	u8 * buf = panel->tx_buf;
	unsigned val = 0;
	int v_bits;

	i = panel->spi_bits;
	if ((i + bits) > sizeof(panel->tx_buf) * 8) {
		printf("too many bits (%d)\n", bits);
		return -EINVAL;
	}

	panel->spi_bits = i + bits;

	buf += i >> 3;
	v_bits = (i & 7);
	if (v_bits) {
		bits += v_bits;
		val = *buf;
		val >>= (8 - v_bits);
	}

	while (bits > 0) {
		if (v_bits < 8) {
			val = (val << 24) | 0xffffff;
			v_bits += 24;
		}
		*buf++ = val >> (v_bits - 8);
		bits -= 8;
		v_bits -= 8;
	}
	return 0;
}

static void extract_data(u8 *dst, unsigned bytes, u8 * buf, unsigned start_bit)
{
	unsigned val = 0;
	int v_bits;

	buf += start_bit >> 3;
	v_bits = (start_bit & 7);
	if (v_bits) {
		val = *buf++;
		v_bits = 8 - v_bits;
	}
	while (bytes > 0) {
		if (v_bits < 8) {
			val = (val << 8) | *buf++;
			v_bits += 8;
		}
		*dst++ = val >> (v_bits - 8);
		v_bits -= 8;
		bytes--;
	}
}
#endif

int common_i2c_write(struct panel_common *panel, const u8 *tx, unsigned tx_len)
{
	u8 *buf = panel->tx_buf;
	u8 tmp;
	struct i2c_msg msg;
	struct dm_i2c_ops *ops = i2c_get_ops(panel->i2c);
	int ret;

	if (tx_len > sizeof(panel->tx_buf))
		return -EINVAL;
	memcpy(buf, tx, tx_len);
	if (tx_len >= 2) {
		tmp = buf[0];
		buf[0] = buf[1];
		buf[1] = tmp;
	}

	msg.addr = panel->i2c_address;
	msg.flags = 0;
	msg.len = tx_len;
	msg.buf = buf;
	ret = ops->xfer(panel->i2c, &msg, 1);
	if (ret < 0) {
		mdelay(10);
		ret = ops->xfer(panel->i2c, &msg, 1);
	}
	return ret < 0 ? ret : 0;
}

int common_i2c_read(struct panel_common *panel, const u8 *tx, int tx_len, u8 *rx, unsigned rx_len)
{
	u8 *buf = panel->tx_buf;
	u8 tmp;
	struct dm_i2c_ops *ops = i2c_get_ops(panel->i2c);
	struct i2c_msg msgs[2];
	int ret;

	if (tx_len > sizeof(panel->tx_buf))
		return -EINVAL;
	if (rx_len > sizeof(panel->rx_buf))
		return -EINVAL;
	memcpy(buf, tx, tx_len);
	if (tx_len >= 2) {
		tmp = buf[0];
		buf[0] = buf[1];
		buf[1] = tmp;
	}
	msgs[0].addr = panel->i2c_address;
	msgs[0].flags = 0;
	msgs[0].len = tx_len;
	msgs[0].buf = buf;

	msgs[1].addr = panel->i2c_address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = rx_len;
	msgs[1].buf = rx;
	ret = ops->xfer(panel->i2c, msgs, 2);
	return ret < 0 ? ret : 0;
}

#define TYPE_MIPI	0
#define TYPE_I2C	1
#define TYPE_SPI	2

#define lptxtime_ns 75
#define prepare_ns 100
#define zero_ns 250

static int send_cmd_list(struct panel_common *panel, struct cmds *mc, int type, const char *id)
{
	struct display_timing *dm = &panel->timings;
	struct mipi_dsi_device *dsi;
	const u8 *cmd = mc->cmds;
	unsigned length = mc->length;
	u8 data[8];
	u8 cmd_buf[32];
	const u8 *p;
	unsigned l;
	unsigned len;
	unsigned mask;
	u32 mipi_clk_rate = 0;
	unsigned long tmp;
	int ret;
	int generic;
	int match = 0;
	u64 readval, matchval;
	int skip = 0;
	int match_index;
	int i;

	debug("%s:%d %d\n", __func__, length, type);
	if (!cmd || !length)
		return 0;

	panel->spi_bits = 0;
	dsi = panel->dsi;
	while (1) {
		len = *cmd++;
		length--;
		if (!length)
			break;
		if ((len >= S_IF_1_LANE) && (len <= S_IF_4_LANES)) {
			int lane_match = 1 + len - S_IF_1_LANE;

			if (lane_match != dsi->lanes)
				skip = 1;
			continue;
		} else if (len == S_IF_BURST) {
			if (!(dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST))
				skip = 1;
			continue;
		} else if (len == S_IF_NONBURST) {
			if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
				skip = 1;
			continue;
		}
		generic = len & 0x80;
		len &= 0x7f;

		p = cmd;
		l = len;
		ret = 0;
		if ((len < S_DELAY) || (len == S_DCS_LENGTH)
				|| (len == S_DCS_BUF)) {
			if (len == S_DCS_LENGTH) {
				len = *cmd++;
				length--;
				p = cmd;
				l = len;
			} else if (len == S_DCS_BUF) {
				l = *cmd++;
				length--;
				if (l > 32)
					l = 32;
				p = cmd_buf;
				len = 0;
			} else {
				p = cmd;
				l = len;
			}
			if (length < len) {
				printf("Unexpected end of data\n");
				break;
			}
			if (!skip) {
				if (type == TYPE_I2C) {
					ret = common_i2c_write(panel, p, l);
#if CONFIG_IS_ENABLED(DM_SPI)
				} else if (type == TYPE_SPI) {
					if (panel->spi_9bit) {
						ret = store_9bit(panel, p, l);
					} else {
						if (l < sizeof(panel->tx_buf)) {
							memcpy(panel->tx_buf, p, l);
							panel->spi_bits = l * 8;
							ret = spi_send(panel, 0);
						} else {
							ret = -EINVAL;
						}
					}
#endif
				} else {
					if (generic) {
						ret = mipi_dsi_generic_write(dsi, p, l);
					} else {
						ret = mipi_dsi_dcs_write_buffer(dsi, p, l);
					}
					if (panel->mipi_delay_between_cmds)
						mdelay(panel->mipi_delay_between_cmds);
				}
			}
		} else if (len == S_MRPS) {
			if (type == TYPE_MIPI) {
				ret = mipi_dsi_set_maximum_return_packet_size(
					dsi, cmd[0]);
			}
			l = len = 1;
		} else if ((len >= S_DCS_READ1) && (len <= S_DCS_READ8)) {
			data[0] = data[1] = data[2] = data[3] = 0;
			data[4] = data[5] = data[6] = data[7] = 0;
			len = len - S_DCS_READ1 + 1;
			match_index = generic ? 2 : 1;
			if (!skip) {
				if (type == TYPE_I2C) {
					ret = common_i2c_read(panel, cmd, match_index, data, len);
#if CONFIG_IS_ENABLED(DM_SPI)
				} else if (type == TYPE_SPI) {
					if (panel->spi_9bit) {
						spi_send(panel, 0);
						store_9bit(panel, cmd, match_index);
					} else {
						memcpy(panel->tx_buf, cmd, match_index);
						panel->spi_bits = match_index * 8;
					}
					l = panel->spi_bits;
					store_high(panel, len * 8);
					ret = spi_send(panel, 1);
					extract_data(data, len, panel->rx_buf, l);
#endif
				} else if (generic) {
					ret =  mipi_dsi_generic_read(dsi, cmd, 2, data, len);
					/* if an error is pending before BTA, an error report can happen */
					if (ret == -EPROTO)
						ret =  mipi_dsi_generic_read(dsi, cmd, 2, data, len);
					ret = 0;
				} else {
					ret =  mipi_dsi_dcs_read(dsi, cmd[0], data, len);
					if (ret == -EPROTO)
						ret =  mipi_dsi_dcs_read(dsi, cmd[0], data, len);
				}
				readval = 0;
				matchval = 0;
				for (i = 0; i < len; i++) {
					readval |= ((u64)data[i]) << (i << 3);
					matchval |= ((u64)cmd[match_index + i]) << (i << 3);
				}
				if (generic) {
					debug("Read (%d)%s GEN: (%04x) 0x%llx cmp 0x%llx\n",
						ret, id, cmd[0] + (cmd[1] << 8), readval, matchval);
				} else {
					debug("Read (%d)%s DCS: (%02x) 0x%llx cmp 0x%llx\n",
						ret, id, cmd[0], readval, matchval);
				}
				if (readval != matchval)
					match = -EINVAL;
			}
			l = len = len + match_index;
		} else if (len == S_DELAY) {
			if (!skip) {
#if CONFIG_IS_ENABLED(DM_SPI)
				if (type == TYPE_SPI)
					spi_send(panel, 0);
#endif
				mdelay(cmd[0]);
			}
			len = 1;
			if (length <= len)
				break;
			cmd += len;
			length -= len;
			l = len = 0;
		} else if (len >= S_CONST) {
			int scmd, dest_start, dest_len, src_start;
			unsigned val;

			scmd = len;
			dest_start = cmd[0];
			dest_len = cmd[1];
			if (scmd == S_CONST) {
				val = cmd[2] | (cmd[3] << 8) | (cmd[4] << 16) | (cmd[5] << 24);
				len = 6;
				src_start = 0;
			} else {
				src_start = cmd[2];
				len = 3;
				switch (scmd) {
				case S_HSYNC:
					val = dm->hsync_len.typ;
					break;
				case S_HBP:
					val = dm->hback_porch.typ;
					break;
				case S_HACTIVE:
					val = dm->hactive.typ;
					break;
				case S_HFP:
					val = dm->hfront_porch.typ;
					break;
				case S_VSYNC:
					val = dm->vsync_len.typ;
					break;
				case S_VBP:
					val = dm->vback_porch.typ;
					break;
				case S_VACTIVE:
					val = dm->vactive.typ;
					break;
				case S_VFP:
					val = dm->vfront_porch.typ;
					break;
				case S_LPTXTIME:
					val = 3;
					if (!mipi_clk_rate && panel->mipi_clk)
						mipi_clk_rate = clk_get_rate(panel->mipi_clk);
					if (!mipi_clk_rate) {
						printf("Unknown mipi_clk_rate\n");
						break;
					}
					/* val = ROUND(lptxtime_ns * mipi_clk_rate/4  /1000000000) */
					tmp = lptxtime_ns;
					tmp *= mipi_clk_rate;
					tmp += 2000000000;
					tmp /= 4000000000;
					val = (unsigned)tmp;
					pr_debug("%s:lptxtime=%d\n", __func__, val);
					if (val > 2047)
						val = 2047;
					break;
				case S_CLRSIPOCOUNT:
					val = 5;
					if (!mipi_clk_rate && panel->mipi_clk)
						mipi_clk_rate = clk_get_rate(panel->mipi_clk);
					if (!mipi_clk_rate) {
						printf("Unknown mipi_clk_rate\n");
						break;
					}
					/* clrsipocount = ROUNDUP((prepare_ns + zero_ns/2) * mipi_clk_rate/4 /1000000000) - 5 */
					tmp = prepare_ns + (zero_ns >> 1) ;
					tmp *= mipi_clk_rate;
					tmp += 4000000000 - 1;
					tmp /= 4000000000;
					val = (unsigned)tmp - 5;
					pr_debug("%s:clrsipocount=%d\n", __func__, val);
					if (val > 63)
						val = 63;
					break;
				default:
					printf("Unknown scmd 0x%x0x\n", scmd);
					val = 0;
					break;
				}
			}
			val >>= src_start;
			while (dest_len && dest_start < 256) {
				l = dest_start & 7;
				mask = (dest_len < 8) ? ((1 << dest_len) - 1) : 0xff;
				cmd_buf[dest_start >> 3] &= ~(mask << l);
				cmd_buf[dest_start >> 3] |= (val << l);
				l = 8 - l;
				dest_start += l;
				val >>= l;
				if (dest_len >= l)
					dest_len -= l;
				else
					dest_len = 0;
			}
			l = 0;
		}
		if (ret < 0) {
			if (l >= 6) {
				printf("Failed to send %s (%d), (%d)%02x %02x: %02x %02x %02x %02x\n",
					id, ret, l, p[0], p[1], p[2], p[3], p[4], p[5]);
			} else if (l >= 2) {
				printf("Failed to send %s (%d), (%d)%02x %02x\n",
					id, ret, l, p[0], p[1]);
			} else {
				printf("Failed to send %s (%d), (%d)%02x\n",
					id, ret, l, p[0]);
			}
			return ret;
		} else {
			if (!skip) {
				if (l >= 18) {
					debug("Sent %s (%d), (%d)%02x %02x: %02x %02x %02x %02x"
							"  %02x %02x %02x %02x"
							"  %02x %02x %02x %02x"
							"  %02x %02x %02x %02x\n",
						id, ret, l, p[0], p[1], p[2], p[3], p[4], p[5],
						p[6], p[7], p[8], p[9],
						p[10], p[11], p[12], p[13],
						p[14], p[15], p[16], p[17]);
				} else if (l >= 6) {
					debug("Sent %s (%d), (%d)%02x %02x: %02x %02x %02x %02x\n",
						id, ret, l, p[0], p[1], p[2], p[3], p[4], p[5]);
				} else if (l >= 2) {
					debug("Sent %s (%d), (%d)%02x %02x\n",
						id, ret, l, p[0], p[1]);
				} else if (l) {
					debug("Sent %s (%d), (%d)%02x\n",
						id, ret, l, p[0]);
				}
			}
		}
		if (length < len) {
			printf("Unexpected end of data\n");
			break;
		}
		cmd += len;
		length -= len;
		if (!length)
			break;
		skip = 0;
	}
#if CONFIG_IS_ENABLED(DM_SPI)
	if (!match && type == TYPE_SPI)
		match = spi_send(panel, 0);
#endif
	return match;
};

static int send_all_cmd_lists(struct panel_common *panel, struct interface_cmds *msc)
{
	int ret = 0;

	if (panel->i2c)
		ret = send_cmd_list(panel,  &msc->i2c, TYPE_I2C, "i2c");
	if (!ret)
		ret = send_cmd_list(panel,  &msc->mipi, TYPE_MIPI, "mipi");
#if CONFIG_IS_ENABLED(DM_SPI)
	if (!ret && panel->spi)
		ret = send_cmd_list(panel,  &msc->spi, TYPE_SPI, "spi");
#endif
	return ret;
}

static int _panel_set_backlight(struct panel_common *p, int percent)
{
	int ret;

	debug("%s: start, backlight = '%s'\n", __func__, p->backlight ? p->backlight->name : "none");
	if (p->gd_enable)
		dm_gpio_set_value(p->gd_enable, (percent > 0) ? true : false);
	if (p->backlight) {
		ret = backlight_set_brightness(p->backlight, percent);
		debug("%s: done, ret = %d\n", __func__, ret);
		if (ret)
			return ret;
	}
	if (p->linked_panel)
		panel_set_backlight(p->linked_panel, percent);
	return 0;
}


static int panel_common_disable(struct panel_common *p)
{
	struct mipi_dsi_device *dsi;

	if (!p->enabled)
		return 0;

	_panel_set_backlight(p, BACKLIGHT_OFF);
	if (p->delay.disable)
		mdelay(p->delay.disable);

	if (p->gpd_display_enable)
		dm_gpio_set_value(p->gpd_display_enable, false);

	dsi = p->dsi;
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	send_all_cmd_lists(p, &p->cmds_disable);
	sn65_disable(&p->sn65);

	p->enabled = false;

	return 0;
}

static int panel_common_unprepare(struct panel_common *p)
{
	if (!p->prepared)
		return 0;

	if (p->delay.unprepare)
		mdelay(p->delay.unprepare);
	dm_gpio_set_value(p->reset, true);
	dm_gpio_set_value(p->gpd_prepare_enable, false);

	regulator_set_enable(p->supply, false);

	p->prepared = false;

	return 0;
}

static int panel_common_prepare(struct panel_common *p)
{
	struct mipi_dsi_device *dsi;
	unsigned int delay;
	int ret;

	if (p->prepared)
		return 0;

	if (IS_ENABLED(CONFIG_DM_REGULATOR) && p->supply) {
		debug("%s: Enable regulator '%s'\n", __func__, p->supply->name);
		ret = regulator_set_enable(p->supply, true);
		if (ret)
			return ret;
	}

	if (p->gpd_power) {
		dm_gpio_set_value(p->gpd_power, true);
		mdelay(p->power_delay_ms);
	}
	if (p->gpd_prepare_enable) {
		if (p->enable_high_duration_us) {
			dm_gpio_set_value(p->gpd_prepare_enable, true);
			udelay(p->enable_high_duration_us);
		}
		if (p->enable_low_duration_us) {
			dm_gpio_set_value(p->gpd_prepare_enable, false);
			udelay(p->enable_low_duration_us);
		}
		dm_gpio_set_value(p->gpd_prepare_enable, true);
	}
	if (p->reset)
		dm_gpio_set_value(p->reset, false);

	delay = p->delay.prepare;
	if (p->no_hpd)
		delay += p->delay.hpd_absent_delay;
	if (delay)
		mdelay(delay);

	dsi = p->dsi;
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = send_all_cmd_lists(p, &p->cmds_init);
	if (ret) {
		regulator_set_enable(p->supply, false);
		return ret;
	}
	p->prepared = true;

	return 0;
}

static int panel_common_enable(struct panel_common *p)
{
	struct mipi_dsi_device *dsi;
	int ret;

	if (p->enabled)
		return 0;

	dsi = p->dsi;
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = send_all_cmd_lists(p, &p->cmds_enable);
	if (ret < 0)
		goto fail;
	if (p->delay.enable)
		mdelay(p->delay.enable);
	sn65_enable(&p->sn65);
	p->enabled = true;

	return 0;
fail:
	if (p->reset)
		dm_gpio_set_value(p->reset, true);
	if (p->gpd_prepare_enable)
		dm_gpio_set_value(p->gpd_prepare_enable, false);
	return ret;
}

static int panel_common_enable2(struct panel_common *p)
{
	struct mipi_dsi_device *dsi;
	int ret;

	dsi = p->dsi;
	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = send_all_cmd_lists(p, &p->cmds_enable2);
	if (ret < 0)
		goto fail;

	sn65_enable2(&p->sn65);
	if (p->gpd_display_enable)
		dm_gpio_set_value(p->gpd_display_enable, true);
	return 0;
fail:
	p->enabled = false;
	if (p->reset)
		dm_gpio_set_value(p->reset, true);
	if (p->gpd_prepare_enable)
		dm_gpio_set_value(p->gpd_prepare_enable, false);
	return ret;
}

static int common_panel_init(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *dsi = plat->device;
	struct panel_common *p = dev_get_priv(dev);
	int ret;

	debug("%s:\n", __func__);
	/* fill characteristics of DSI data link */
	if (dsi) {
		dsi->lanes = p->lanes;
		dsi->format = p->format;
		dsi->mode_flags = p->mode_flags;
		dsi->def_pix_clk = p->timings.pixelclock.typ;
		dsi->hsmult	 = p->min_hs_clock_multiple;
		dsi->mipi_dsi_multiple = p->mipi_dsi_multiple;
		debug("%s: min_hs_clock_multiple=%d\n", __func__, dsi->hsmult);
		ret = mipi_dsi_attach(dsi);
	}

	if (!ret && p->linked_panel)
		panel_init(p->linked_panel);

	p->dsi = dsi;
	ret = panel_common_prepare(p);
	if (ret)
		return ret;
	return ret;
}

static int common_panel_enable(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *dsi = plat->device;
	struct panel_common *p = dev_get_priv(dev);
	int ret;

	debug("%s: start, backlight = '%s'\n", __func__,
		p->backlight ? p->backlight->name : "none");
	p->dsi = dsi;
	ret = mipi_dsi_enable_lpm(dsi);
	if (ret && (ret != -ENOSYS))
		return ret;
	ret = panel_common_enable(p);
	if (ret)
		return ret;
	ret = mipi_dsi_enable_frame(dsi);
	if (ret && (ret != -ENOSYS))
		return ret;
	ret = panel_common_enable2(p);
	if (ret)
		return ret;
	if (p->gd_enable)
		dm_gpio_set_value(p->gd_enable, true);
	return 0;
}

static int common_panel_enable_backlight(struct udevice *dev)
{
	struct panel_common *p = dev_get_priv(dev);
	int ret;

	if (!p->enabled)
		return 0;
	if (p->delay.before_backlight_on)
		mdelay(p->delay.before_backlight_on);
	if (p->backlight) {
		ret = backlight_enable(p->backlight);
		debug("%s: done, ret = %d\n", __func__, ret);
		if (ret)
			return ret;
	}
	if (p->linked_panel)
		panel_enable_backlight(p->linked_panel);
	return 0;
}

static int common_panel_set_backlight(struct udevice *dev, int percent)
{
	struct panel_common *p = dev_get_priv(dev);

	return _panel_set_backlight(p, percent);
}

static int common_panel_get_display_timing(struct udevice *dev,
					    struct display_timing *timings)
{
	struct panel_common *p = dev_get_priv(dev);

	memcpy(timings, &p->timings, sizeof(*timings));
	return 0;
}

void check_for_cmds(ofnode np, const char *dt_name, struct cmds *mc)
{
	const void *data;
	int data_len;

	/* Check for mipi command arrays */
	if (!ofnode_get_property(np, dt_name, &data_len) || !data_len)
		return;

	data = ofnode_read_u8_array_ptr(np, dt_name, data_len);
	if (!data) {
		pr_info("failed to read %s from DT\n", dt_name);
		return;
	}
	mc->cmds = data;
	mc->length = data_len;
}

static void init_common(ofnode np, struct panel_common *panel)
{
	long mode_flags = 0;
	ofnode_read_u32(np, "delay-prepare", &panel->delay.prepare);
	ofnode_read_u32(np, "delay-enable", &panel->delay.enable);
	ofnode_read_u32(np, "delay-disable", &panel->delay.disable);
	ofnode_read_u32(np, "delay-unprepare", &panel->delay.unprepare);
	ofnode_read_u32(np, "delay-before-backlight-on", &panel->delay.before_backlight_on);
	ofnode_read_u32(np, "min-hs-clock-multiple", &panel->min_hs_clock_multiple);
	ofnode_read_u32(np, "mipi-dsi-multiple", &panel->mipi_dsi_multiple);
	ofnode_read_u32(np, "mipi-delay-between-cmds", &panel->mipi_delay_between_cmds);
	ofnode_read_u32(np, "enable-high-duration-us", &panel->enable_high_duration_us);
	ofnode_read_u32(np, "enable-low-duration-us", &panel->enable_low_duration_us);
	ofnode_read_u32(np, "power-delay-ms", &panel->power_delay_ms);
	if (ofnode_read_bool(np, "mode-clock-non-continuous"))
		mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;
	if (ofnode_read_bool(np, "mode-skip-eot"))
		mode_flags |= MIPI_DSI_MODE_EOT_PACKET;
	if (ofnode_read_bool(np, "mode-video"))
		mode_flags |= MIPI_DSI_MODE_VIDEO;
	if (ofnode_read_bool(np, "mode-video-burst"))
		mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
	if (ofnode_read_bool(np, "mode-video-hbp-disable"))
		mode_flags |= MIPI_DSI_MODE_VIDEO_HBP;
	if (ofnode_read_bool(np, "mode-video-hfp-disable"))
		mode_flags |= MIPI_DSI_MODE_VIDEO_HFP;
	if (ofnode_read_bool(np, "mode-video-hsa-disable"))
		mode_flags |= MIPI_DSI_MODE_VIDEO_HSA;
	if (ofnode_read_bool(np, "mode-video-sync-pulse"))
		mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	if (ofnode_read_bool(np, "mode-video-hse"))
		mode_flags |= MIPI_DSI_MODE_VIDEO_HSE;
	if (ofnode_read_bool(np, "mode-video-mbc"))
		mode_flags |= MIPI_DSI_MODE_VIDEO_MBC;
	panel->mode_flags |= mode_flags;
	debug("%s: delays: %d %d %d %d, "
		"min_hs_clock_multiple=%d mipi_dsi_multiple=%d\n", __func__,
		panel->delay.prepare, panel->delay.enable,
		panel->delay.disable, panel->delay.unprepare,
		panel->min_hs_clock_multiple, panel->mipi_dsi_multiple);

#if 0  && defined(DEBUG)
	debug("%s: %s %lx %lx\n", __func__, ofnode_get_name(np), mode_flags, panel->mode_flags);
	{
		struct ofprop prop;
		int ret;
		for (ret = ofnode_get_first_property(np, &prop); !ret;
			ret = ofnode_get_next_property(&prop)) {
			const char *name = NULL;
			int len;

			ofnode_get_property_by_prop(&prop, &name, &len);
			if (name)
				printf("%s: '%s' %lu\n", __func__, name, strlen(name));
		}
	}
#endif
}

static int common_panel_ofdata_to_platdata(struct udevice *dev)
{
	struct panel_common *panel = dev_get_priv(dev);
	const char *bf;
	ofnode np = dev_ofnode(dev);
	ofnode cmds_np;
	ofnode sn65_np;
	u32 bridge_de_active;
	u32 bridge_sync_active;
	struct clk *mipi_clk;
	int err;
	int ret;

	debug("%s:\n", __func__);
	ret = dev_read_u32(dev, "dsi-lanes", &panel->lanes);
	if (ret) {
		printf("Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	panel->enabled = false;
	panel->prepared = false;

	if (ofnode_read_u32(np, "bridge-de-active", &bridge_de_active)) {
		bridge_de_active = -1;
	}
	if (ofnode_read_u32(np, "bridge-sync-active", &bridge_sync_active)) {
		bridge_sync_active = -1;
	}

	if (err < 0)
		return err;

	ofnode_decode_display_timing(np, 0, &panel->timings);

	if (bridge_de_active <= 1) {
		panel->timings.flags &= ~DISPLAY_FLAGS_DE_HIGH;
		if (bridge_de_active)
			panel->timings.flags |= DISPLAY_FLAGS_DE_HIGH;
	}

	if (bridge_sync_active <= 1) {
		panel->timings.flags &= ~(DISPLAY_FLAGS_HSYNC_HIGH |
				DISPLAY_FLAGS_VSYNC_HIGH);
		if (bridge_sync_active)
			panel->timings.flags |=  DISPLAY_FLAGS_HSYNC_HIGH |
				DISPLAY_FLAGS_VSYNC_HIGH;
	}
	bf = ofnode_read_string(np, "bus-format");
	if (!bf) {
		printf("bus-format missing\n");
		return -EINVAL;
	}
	if (!strcmp(bf, "rgb888")) {
		panel->format = MIPI_DSI_FMT_RGB888;
	} else if (!strcmp(bf, "rgb666")) {
		panel->format = MIPI_DSI_FMT_RGB666;
	} else {
		printf("unknown bus-format %s\n", bf);
		return -EINVAL;
	}
	init_common(np, panel);
	ofnode_read_u32(np, "bits-per-color", &panel->bpc);
	ret = ofnode_parse_phandle(np, "mipi-cmds", &cmds_np);
	if (!ret) {
		debug("%s: %s\n", __func__, ofnode_get_name(cmds_np));
		ret = gpio_request_by_name_nodev(cmds_np, "display-enable-gpios", 0, &panel->gds_display_enable,
					   GPIOD_IS_OUT);
		if (ret) {
			debug("%s: Warning: cannot get display-enable-gpios: ret=%d\n",
			      __func__, ret);
			if (ret != -ENOENT)
				return ret;
		} else {
			panel->gpd_display_enable = &panel->gds_display_enable;
		}

		ret = uclass_get_device_by_ofnode_prop(UCLASS_I2C, cmds_np,
				"i2c-bus", &panel->i2c);
		if (ret && (ret != -ENOENT))
			printf("!!!i2c-bus not found %d\n", ret);

		ret = uclass_get_device_by_ofnode_prop(UCLASS_SPI, cmds_np,
				"spi", &panel->spi);
		if (ret && (ret != -ENOENT))
			printf("!!!spi not found %d\n", ret);

		if (panel->i2c) {
			check_for_cmds(cmds_np, "i2c-cmds-init",
				       &panel->cmds_init.i2c);
			check_for_cmds(cmds_np, "i2c-cmds-enable",
				       &panel->cmds_enable.i2c);
			check_for_cmds(cmds_np, "i2c-cmds-enable2",
				       &panel->cmds_enable2.i2c);
			check_for_cmds(cmds_np, "i2c-cmds-disable",
				       &panel->cmds_disable.i2c);
			ofnode_read_u32(cmds_np, "i2c-address", &panel->i2c_address);
			ofnode_read_u32(cmds_np, "i2c-max-frequency", &panel->i2c_max_frequency);
		}
		check_for_cmds(cmds_np, "mipi-cmds-init",
			       &panel->cmds_init.mipi);
		check_for_cmds(cmds_np, "mipi-cmds-enable",
			       &panel->cmds_enable.mipi);
		/* enable 2 is after frame data transfer has started */
		check_for_cmds(cmds_np, "mipi-cmds-enable2",
			       &panel->cmds_enable2.mipi);
		check_for_cmds(cmds_np, "mipi-cmds-disable",
			       &panel->cmds_disable.mipi);

		if (panel->spi) {
			if (ofnode_read_bool(cmds_np, "spi-9-bit"))
				panel->spi_9bit = 1;
			check_for_cmds(cmds_np, "spi-cmds-init",
			       &panel->cmds_init.spi);
			check_for_cmds(cmds_np, "spi-cmds-enable",
			       &panel->cmds_enable.spi);
			check_for_cmds(cmds_np, "spi-cmds-enable2",
			       &panel->cmds_enable2.spi);
			check_for_cmds(cmds_np, "spi-cmds-disable",
			       &panel->cmds_disable.spi);
			ofnode_read_u32(cmds_np, "spi-max-frequency", &panel->spi_max_frequency);
		}
		init_common(cmds_np, panel);
	}
	pr_info("%s: delay %d %d, %d %d\n", __func__,
		panel->delay.prepare, panel->delay.enable,
		panel->delay.disable, panel->delay.unprepare);

	mipi_clk = devm_clk_get_optional(dev, "mipi_clk");
	if (IS_ERR(mipi_clk)) {
		err = PTR_ERR(mipi_clk);
		dev_dbg(dev, "%s:devm_clk_get mipi_clk  %d\n", __func__, err);
		return err;
	}
	panel->mipi_clk = mipi_clk;

	panel->no_hpd = ofnode_read_bool(np, "no-hpd");

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
						   "power-supply", &panel->supply);
		if (ret) {
			debug("%s: Warning: cannot get power supply: ret=%d\n",
			      __func__, ret);
			if (ret != -ENOENT)
				return ret;
		}
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &panel->gds_reset,
				   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (ret) {
		debug("Warning: cannot get reset GPIO\n");
		if (ret != -ENOENT)
			return ret;
	} else {
		panel->reset = &panel->gds_reset;
	}

	ret = gpio_request_by_name(dev, "power-gpios", 0, &panel->gds_power,
				   GPIOD_IS_OUT);
	if (ret) {
		debug("Warning: cannot get power GPIO\n");
		if (ret != -ENOENT)
			return ret;
	} else {
		panel->gpd_power = &panel->gds_power;
		dm_gpio_set_value(panel->gpd_power, false);
	}

	ret = gpio_request_by_name(dev, "enable-gpios", 0, &panel->gds_prepare_enable,
				   GPIOD_IS_OUT);
	if (ret) {
		debug("%s: Warning: cannot get enable GPIO: ret=%d\n",
		      __func__, ret);
		if (ret != -ENOENT)
			return ret;
	} else {
		panel->gpd_prepare_enable = &panel->gds_prepare_enable;
		dm_gpio_set_value(panel->gpd_prepare_enable, false);
	}

	ret = gpio_request_by_name(dev, "on-gpios", 0, &panel->gds_enable,
				   GPIOD_IS_OUT);
	if (ret) {
		debug("%s: Warning: cannot get on GPIO: ret=%d\n",
		      __func__, ret);
		if (ret != -ENOENT)
			return log_ret(ret);
	} else {
		panel->gd_enable = &panel->gds_enable;
	}

	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &panel->backlight);
	if (ret) {
		debug("%s: Cannot get backlight: ret=%d\n", __func__, ret);
		if (ret != -ENOENT)
			return ret;
	}
	ret = ofnode_parse_phandle(np, "sn65dsi83", &sn65_np);
	if (!ret) {
		if (ofnode_is_available(sn65_np)) {
			panel->sn65.mipi_clk = mipi_clk;
			ret = sn65dsi83_ofdata_to_platdata(dev, &panel->sn65, np, sn65_np);
		}
	}
	return 0;
}

static int common_panel_probe(struct udevice *dev)
{
	struct panel_common *p = dev_get_priv(dev);
	int ret;

	debug("%s:\n", __func__);
	/* reset panel */
	if (p->reset) {
		ret = dm_gpio_set_value(p->reset, true);
		if (ret)
			printf("reset gpio fails to set true\n");
		mdelay(100);
	}

	p->linked_panel = video_link_get_next_device(dev);
	if (p->linked_panel &&
		device_get_uclass_id(p->linked_panel) != UCLASS_PANEL) {
		debug("get panel device error\n");
		return -ENODEV;
	}
	ret = 0;
	if (p->sn65.i2c) {
		ret = sn65_init(&p->sn65);
	}
	return ret;
}

static int common_panel_disable(struct udevice *dev)
{
	struct panel_common *p = dev_get_priv(dev);

	panel_common_disable(p);
	panel_common_unprepare(p);
	if (p->linked_panel)
		panel_uninit(p->linked_panel);
	sn65_remove(&p->sn65);
	return 0;
}

static const struct panel_ops common_panel_ops = {
	.init			= common_panel_init,
	.enable			= common_panel_enable,
	.enable_backlight	= common_panel_enable_backlight,
	.get_display_timing	= common_panel_get_display_timing,
	.set_backlight		= common_panel_set_backlight,
};

static const struct udevice_id common_panel_ids[] = {
	{ .compatible = "panel,common" },
	{ }
};

U_BOOT_DRIVER(panel_common) = {
	.name	= "panel_common",
	.id	= UCLASS_PANEL,
	.of_match = common_panel_ids,
	.ops	= &common_panel_ops,
	.of_to_plat	= common_panel_ofdata_to_platdata,
	.probe		= common_panel_probe,
	.remove		= common_panel_disable,
	.plat_auto	= sizeof(struct mipi_dsi_panel_plat),
	.priv_auto	= sizeof(struct panel_common),
};
