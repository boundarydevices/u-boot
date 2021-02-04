/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <common.h>
#include <asm/gpio.h>
#include <clk.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <errno.h>
#include <i2c.h>
#include <linux/delay.h>
#include <log.h>
#include <panel.h>
#include <power/regulator.h>
#include <watchdog.h>

#define EDID_ADDR 0x50
#define LT8912_REG_CHIP_REVISION_0 (0x00)
#define LT8912_REG_CHIP_REVISION_1 (0x01)

#define LT8912_VAL_CHIP_REVISION_0 (0x12)
#define LT8912_VAL_CHIP_REVISION_1 (0xB2)
#define LT8912_DSI_CEC_I2C_ADDR_REG (0xE1)
#define LT8912_RESET_DELAY (100)

#define PINCTRL_STATE_ACTIVE    "pmx_lt8912_active"
#define PINCTRL_STATE_SUSPEND   "pmx_lt8912_suspend"

#define MDSS_MAX_PANEL_LEN      256
#define EDID_SEG_SIZE 0x100
/* size of audio and speaker info Block */
#define AUDIO_DATA_SIZE 32

#define MAIN_LANE_EN	0x40

#define CEC_DSI_SETTLE	0x11
#define CEC_DSI_LANE	0x13
#define CEC_DSI_HSYNC	0x18
#define CEC_DSI_VSYNC	0x19
#define CEC_DSI_HACTIVE	0x1c
#define CEC_DSI_HTOTAL	0x34
#define CEC_DSI_VTOTAL	0x36
#define CEC_DSI_VBP	0x38
#define CEC_DSI_VFP	0x3a
#define CEC_DSI_HBP	0x3c
#define CEC_DSI_HFP	0x3e

/* 0x94 interrupts */
#define HPD_INT_ENABLE           BIT(7)
#define MONITOR_SENSE_INT_ENABLE BIT(6)
#define ACTIVE_VSYNC_EDGE        BIT(5)
#define AUDIO_FIFO_FULL          BIT(4)
#define EDID_READY_INT_ENABLE    BIT(2)

#define MAX_WAIT_TIME (100)
#define MAX_RW_TRIES (3)

/* 0x95 interrupts */
#define CEC_TX_READY             BIT(5)
#define CEC_TX_ARB_LOST          BIT(4)
#define CEC_TX_RETRY_TIMEOUT     BIT(3)
#define CEC_TX_RX_BUF3_READY     BIT(2)
#define CEC_TX_RX_BUF2_READY     BIT(1)
#define CEC_TX_RX_BUF1_READY     BIT(0)

#define HPD_INTERRUPTS           (HPD_INT_ENABLE | \
					MONITOR_SENSE_INT_ENABLE)
#define EDID_INTERRUPTS          EDID_READY_INT_ENABLE
#define CEC_INTERRUPTS           (CEC_TX_READY | \
					CEC_TX_ARB_LOST | \
					CEC_TX_RETRY_TIMEOUT | \
					CEC_TX_RX_BUF3_READY | \
					CEC_TX_RX_BUF2_READY | \
					CEC_TX_RX_BUF1_READY)

#define CFG_HPD_INTERRUPTS       BIT(0)
#define CFG_EDID_INTERRUPTS      BIT(1)
#define CFG_CEC_INTERRUPTS       BIT(3)

#define MAX_OPERAND_SIZE	14
#define CEC_MSG_SIZE            (MAX_OPERAND_SIZE + 2)


enum lt8912_i2c_addr {
	I2C_ADDR_MAIN = 0x48,
	I2C_ADDR_CEC_DSI = 0x49,
	I2C_ADDR_I2S = 0x4a,
};

enum lt8912_cec_buf {
	LT8912_CEC_BUF1,
	LT8912_CEC_BUF2,
	LT8912_CEC_BUF3,
	LT8912_CEC_BUF_MAX,
};

struct lt8912_reg_cfg {
	u8 i2c_addr;
	u8 reg;
	u8 val;
	int sleep_in_ms;
};

struct lt {
	struct udevice *dev;
	struct gpio_desc *gd_reset;
	struct udevice	*avdd;
	struct display_timing	timings;
	bool audio;
	bool disable_gpios;
	bool is_power_on;
	void *edid_data;
	u8 edid_buf[EDID_SEG_SIZE];
	int prev_connected;
	ofnode disp_dsi;
	struct udevice *ddc;
	ulong start_time;
	u8 video_mode;
	u8 last_reg;
	u8 last_reg_cec;
	u8 dsi_lanes;
	struct gpio_desc gds_reset;
};

static struct lt8912_reg_cfg lt8912_init_setup[] = {
/* Digital clock en*/
	/* power down */
	{I2C_ADDR_MAIN, 0x08, 0xff, 0},
	/* HPD override */
	{I2C_ADDR_MAIN, 0x09, 0xff, 0},
	/* color space */
	{I2C_ADDR_MAIN, 0x0a, 0xff, 0},
	/* Fixed */
	{I2C_ADDR_MAIN, 0x0b, 0x7c, 0},
	/* HDCP */
	{I2C_ADDR_MAIN, 0x0c, 0xff, 0},
	{I2C_ADDR_MAIN, 0x42, 0x04, 0},

/*Tx Analog*/
	/* Fixed */
	{I2C_ADDR_MAIN, 0x31, 0xb1, 0},  
	/* V1P2 */
	{I2C_ADDR_MAIN, 0x32, 0xb1, 0}, 
	/* Fixed */
	{I2C_ADDR_MAIN, 0x33, 0x0e, 0},  
	/* Fixed */
	{I2C_ADDR_MAIN, 0x37, 0x00, 0},
	/* Fixed */
	{I2C_ADDR_MAIN, 0x38, 0x22, 0},
	/* Fixed */
	{I2C_ADDR_MAIN, 0x60, 0x82, 0},
/*Cbus Analog*/
	/* Fixed */
	{I2C_ADDR_MAIN, 0x39, 0x45, 0},
	{I2C_ADDR_MAIN, 0x3a, 0x00, 0}, //20180718
	{I2C_ADDR_MAIN, 0x3b, 0x00, 0},
/*HDMI Pll Analog*/	
	{I2C_ADDR_MAIN, 0x44, 0x31, 0},
	/* Fixed */
	{I2C_ADDR_MAIN, 0x55, 0x44, 0},
	/* Fixed */
	{I2C_ADDR_MAIN, 0x57, 0x01, 0},
	{I2C_ADDR_MAIN, 0x5a, 0x02, 0},
/*MIPI Analog*/
	/* Fixed */
	{I2C_ADDR_MAIN, 0x3e, 0xd6, 0},  //0xde.  //0xf6 = pn swap
	{I2C_ADDR_MAIN, 0x3f, 0xd4, 0},
	{I2C_ADDR_MAIN, 0x41, 0x3c, 0},
	{I2C_ADDR_MAIN, 0xB2, 0x00, 0},	/* 0 DVI, 1 HDMI */
	{I2C_ADDR_CEC_DSI, 0x12,0x04, 0},
};

static struct lt8912_reg_cfg lt8912_mipi_basic_set[] = {
	{I2C_ADDR_CEC_DSI, 0x14,0x00, 0}, 
	{I2C_ADDR_CEC_DSI, 0x15,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x1a,0x03, 0}, 
	{I2C_ADDR_CEC_DSI, 0x1b,0x03, 0}, 
};

static struct lt8912_reg_cfg lt8912_ddsconfig[] = {
	{I2C_ADDR_CEC_DSI, 0x4e,0xff, 0},
	{I2C_ADDR_CEC_DSI, 0x4f,0x56, 0},
	{I2C_ADDR_CEC_DSI, 0x50,0x69, 0},
	{I2C_ADDR_CEC_DSI, 0x51,0x80, 0},
	/*
	{I2C_ADDR_CEC_DSI, 0x1f,0x90, 0},
	{I2C_ADDR_CEC_DSI, 0x20,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x21,0x68, 0},
	{I2C_ADDR_CEC_DSI, 0x22,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x23,0x5E, 0},
	{I2C_ADDR_CEC_DSI, 0x24,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x25,0x54, 0},
	{I2C_ADDR_CEC_DSI, 0x26,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x27,0x90, 0},
	{I2C_ADDR_CEC_DSI, 0x28,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x29,0x68, 0},
	{I2C_ADDR_CEC_DSI, 0x2a,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x2b,0x5E, 0},
	{I2C_ADDR_CEC_DSI, 0x2c,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x2d,0x54, 0},
	{I2C_ADDR_CEC_DSI, 0x2e,0x01, 0},
	*/
	{I2C_ADDR_CEC_DSI, 0x1f,0x5e, 0},
	{I2C_ADDR_CEC_DSI, 0x20,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x21,0x2c, 0},
	{I2C_ADDR_CEC_DSI, 0x22,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x23,0xfa, 0},
	{I2C_ADDR_CEC_DSI, 0x24,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x25,0xc8, 0},
	{I2C_ADDR_CEC_DSI, 0x26,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x27,0x5e, 0},
#if 1
	{I2C_ADDR_CEC_DSI, 0x28,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x29,0x2c, 0},
	{I2C_ADDR_CEC_DSI, 0x2a,0x01, 0},
	{I2C_ADDR_CEC_DSI, 0x2b,0xfa, 0},
	{I2C_ADDR_CEC_DSI, 0x2c,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x2d,0xc8, 0},
	{I2C_ADDR_CEC_DSI, 0x2e,0x00, 0},
#endif
	{I2C_ADDR_CEC_DSI, 0x42,0x64, 0},
	{I2C_ADDR_CEC_DSI, 0x43,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x44,0x04, 0},
	{I2C_ADDR_CEC_DSI, 0x45,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x46,0x59, 0},
	{I2C_ADDR_CEC_DSI, 0x47,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x48,0xf2, 0},
	{I2C_ADDR_CEC_DSI, 0x49,0x06, 0},
	{I2C_ADDR_CEC_DSI, 0x4a,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x4b,0x72, 0},
	{I2C_ADDR_CEC_DSI, 0x4c,0x45, 0},
	{I2C_ADDR_CEC_DSI, 0x4d,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x52,0x08, 0},
	{I2C_ADDR_CEC_DSI, 0x53,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x54,0xb2, 0},
	{I2C_ADDR_CEC_DSI, 0x55,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x56,0xe4, 0},
	{I2C_ADDR_CEC_DSI, 0x57,0x0d, 0},
	{I2C_ADDR_CEC_DSI, 0x58,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x59,0xe4, 0},
	{I2C_ADDR_CEC_DSI, 0x5a,0x8a, 0},
	{I2C_ADDR_CEC_DSI, 0x5b,0x00, 0},
	{I2C_ADDR_CEC_DSI, 0x5c,0x34, 0},
	{I2C_ADDR_CEC_DSI, 0x1e,0x4f, 0},
	{I2C_ADDR_CEC_DSI, 0x51,0x00, 0},
};

static struct lt8912_reg_cfg lt8912_rxlogicres[] = {
	{I2C_ADDR_MAIN, 0x03,0x7f, 100},
	{I2C_ADDR_MAIN, 0x03,0xff, 0},
};

static struct lt8912_reg_cfg I2S_cfg[] = {
	{I2C_ADDR_MAIN, 0xB2, 0x01, 0},
	{I2C_ADDR_I2S, 0x06, 0x08, 0},
	{I2C_ADDR_I2S, 0x07, 0xf0, 0},

	{I2C_ADDR_I2S, 0x0f, 0x28, 0}, //Audio 16bit, 48K 
	
	{I2C_ADDR_I2S, 0x34, 0xe2, 0}, //sclk = 64fs, 0xd2; sclk = 32fs, 0xe2.
};


static struct lt8912_reg_cfg lt8912_lvds_bypass_cfg[] = {
	 //lvds power up
	{I2C_ADDR_MAIN, 0x44, 0x30, 0},
	{I2C_ADDR_MAIN, 0x51, 0x05, 0},

	 //core pll bypass
	{I2C_ADDR_MAIN, 0x50, 0x24, 0},//cp=50uA
	{I2C_ADDR_MAIN, 0x51, 0x2d, 0},//Pix_clk as reference,second order passive LPF PLL
	{I2C_ADDR_MAIN, 0x52, 0x04, 0},//loopdiv=0;use second-order PLL
	{I2C_ADDR_MAIN, 0x69, 0x0e, 0},//CP_PRESET_DIV_RATIO
	{I2C_ADDR_MAIN, 0x69, 0x8e, 0},
	{I2C_ADDR_MAIN, 0x6a, 0x00, 0},
	{I2C_ADDR_MAIN, 0x6c, 0xb8, 0},//RGD_CP_SOFT_K_EN,RGD_CP_SOFT_K[13:8]
	{I2C_ADDR_MAIN, 0x6b, 0x51, 0},

	{I2C_ADDR_MAIN, 0x04, 0xfb, 0},//core pll reset
	{I2C_ADDR_MAIN, 0x04, 0xff, 0},

	//scaler bypass
	{I2C_ADDR_MAIN, 0x7f, 0x00, 0},//disable scaler
	{I2C_ADDR_MAIN, 0xa8, 0x13, 0},//0x13 : JEIDA, 0x33:VSEA


	{I2C_ADDR_MAIN, 0x02, 0xf7, 0},//lvds pll reset
	{I2C_ADDR_MAIN, 0x02, 0xff, 0},
	{I2C_ADDR_MAIN, 0x03, 0xcf, 0},
	{I2C_ADDR_MAIN, 0x03, 0xff, 0},
};


static int lt8912_write(struct lt *lt, u8 addr, u8 reg, u8 val)
{
	struct i2c_msg msgs[1];
	int ret = 0;
	u8 wbuf[4] = {0};

	if (!lt) {
		debug("%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	wbuf[0] = reg;
	wbuf[1] = val;

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = wbuf;

	ret = dm_i2c_xfer(lt->dev, msgs, 1);
	if (ret < 0) {
		printf("%s:addr=%x reg=%x ret=%d\n", __func__, addr, reg, ret);
		return ret;
	}
	return 0;
}

static int lt8912_write_w(struct lt *lt, u8 addr, u8 reg, u16 val)
{
	struct i2c_msg msgs[1];
	int ret = 0;
	u8 wbuf[4] = {0};

	wbuf[0] = reg;
	wbuf[1] = val & 0xff;
	wbuf[2] = val >> 8;

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 3;
	msgs[0].buf = wbuf;

	ret = dm_i2c_xfer(lt->dev, msgs, 1);
	if (ret < 0) {
		printf("%s:addr=%x reg=%x ret=%d\n", __func__, addr, reg, ret);
		return ret;
	}
	return 0;
}

static int lt8912_read(struct lt *lt, u8 addr, u8 reg, u8 *rx_buf, u32 size)
{
	uint8_t tx_buf[4];
	struct i2c_msg msgs[2];
	int ret;

	tx_buf[0] = reg;
	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = tx_buf;

	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = size;
	msgs[1].buf = rx_buf;
	ret = dm_i2c_xfer(lt->dev, msgs, 2);
	if (ret) {
		debug("%s reg(%x), failed(%d)\n", __func__, reg, ret);
	}
	return ret < 0 ? ret : 0;
}

static int lt8912_read_ddc(struct lt *lt, u8 addr, u8 reg, u8 *rx_buf, u32 size)
{
	struct dm_i2c_ops *ops;
	uint8_t tx_buf[4];
	struct i2c_msg msgs[2];
	int ret;

	if (!lt->ddc)
		return lt8912_read(lt, addr, reg, rx_buf, size);

	tx_buf[0] = reg;
	ops = i2c_get_ops(lt->ddc);

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = tx_buf;

	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = size;
	msgs[1].buf = rx_buf;
	ret = ops->xfer(lt->ddc, msgs, 2);
	return ret < 0 ? ret : 0;
}

static void lt8912_write_array(struct lt *lt,
	struct lt8912_reg_cfg *cfg, int size)
{
	int ret = 0;
	int i;

	size = size / sizeof(struct lt8912_reg_cfg);
	for (i = 0; i < size; i++) {
		ret = lt8912_write(lt, cfg[i].i2c_addr,
			cfg[i].reg, cfg[i].val);
		if (ret != 0){
			printf("%s: lt reg write %02X to %02X failed.\n",
				__func__, cfg[i].val, cfg[i].reg);
		}
		if (cfg[i].sleep_in_ms)
			mdelay(cfg[i].sleep_in_ms);
	}
}

static int lt8912_read_device_rev(struct lt *lt)
{
	u8 rev = 0;
	int ret;

	ret = lt8912_read(lt, I2C_ADDR_MAIN, LT8912_REG_CHIP_REVISION_0,
                                                        &rev, 1);
	if (ret || rev != LT8912_VAL_CHIP_REVISION_0) {
		printf("check chip revision not match reg = 0x%x, val = 0x%x expected 0x%x, %d\n",
			LT8912_REG_CHIP_REVISION_0, rev,
			LT8912_VAL_CHIP_REVISION_0, ret);
		return -ENODEV;
	}
	debug("%s: LT8912_REG_CHIP_REVISION_0= %d\n", __func__, rev);
	ret = lt8912_read(lt, I2C_ADDR_MAIN, LT8912_REG_CHIP_REVISION_1,
			  &rev, 1);
	if (ret || rev != LT8912_VAL_CHIP_REVISION_1) {
		printf("check chip revision not match reg = 0x%x, val = 0x%x expected 0x%x, %d\n",
			LT8912_REG_CHIP_REVISION_1, rev,
			LT8912_VAL_CHIP_REVISION_1, ret);
		return -ENODEV;
	}
	debug("%s: LT8912_REG_CHIP_REVISION_1= %d\n", __func__, rev);

	return ret;
}

static int lt8912_gpio_configure(struct lt *lt, bool on)
{
	dm_gpio_set_value(lt->gd_reset, on ? 0 : 1);
	return 0;
}

enum hpd_callback_event {
	HPD_CONNECT,
	HPD_DISCONNECT,
};

int lt8912_read_edid(struct lt *lt, u32 size, u8 *edid_buf)
{
	u32 read_size = size / 2;
	u8 edid_addr = 0;
	int ndx;
	int ret;

	if (!lt || !edid_buf)
		return 0;

	udelay(10 * 1000);

	ret = lt8912_read(lt, I2C_ADDR_MAIN, 0x43, &edid_addr, 1);
	if (ret < 0)
		return ret;

	debug("%s: edid address 0x%x\n", __func__, edid_addr);

	ret = lt8912_read_ddc(lt, EDID_ADDR, 0x00, edid_buf, read_size);
	if (ret < 0)
		return ret;

	ret = lt8912_read_ddc(lt, EDID_ADDR, read_size,
		edid_buf + read_size, read_size);
	if (ret < 0)
		return ret;

	for (ndx = 0; ndx < size; ndx += 4)
		debug("%s: EDID[%02x-%02x] %02x %02x %02x %02x\n",
			__func__, ndx, ndx + 3,
			edid_buf[ndx + 0], edid_buf[ndx + 1],
			edid_buf[ndx + 2], edid_buf[ndx + 3]);

	return ret;
}

static int lt8912_video_setup(struct lt *lt)
{
	u32 hactive, h_total, hpw, hfp, hbp;
	u32 vactive, v_total, vpw, vfp, vbp;
	u8 settle = 0x08;

	hactive = lt->timings.hactive.typ;
	hfp = lt->timings.hfront_porch.typ;
	hpw = lt->timings.hsync_len.typ;
	hbp = lt->timings.hback_porch.typ;
	h_total = hactive + hfp + hpw + hbp;

	vactive = lt->timings.vactive.typ;
	vfp = lt->timings.vfront_porch.typ;
	vpw = lt->timings.vsync_len.typ;
	vbp = lt->timings.vback_porch.typ;
	v_total = vactive + vfp + vpw + vbp;

	debug("h_total 0x%x, xres 0x%x, hfp 0x%x, hpw 0x%x, hbp 0x%x, lanes %d\n",
		h_total, hactive, hfp, hpw, hbp, lt->dsi_lanes);

	debug("v_total 0x%x, yres 0x%x, vfp 0x%x, vpw 0x%x, vbp 0x%x\n",
		v_total, vactive, vfp, vpw, vbp);

	if (vactive == 480)
		settle = 4;
	else if (vactive == 720)
		settle = 0x0a;
	else if (vactive == 1080)
		settle = 0x0a;
	else if (vactive == 800)
		settle = 0x08;
	else
		printf("%s: unsupported yres = %d\n", __func__, vactive);

	lt8912_write(lt, I2C_ADDR_CEC_DSI, 0x10, 0x01);
	lt8912_write(lt, I2C_ADDR_CEC_DSI, CEC_DSI_SETTLE, settle);
	lt8912_write(lt, I2C_ADDR_CEC_DSI, CEC_DSI_HSYNC, hpw);
	lt8912_write(lt, I2C_ADDR_CEC_DSI, CEC_DSI_VSYNC, vpw);
	lt8912_write_w(lt, I2C_ADDR_CEC_DSI, CEC_DSI_HACTIVE, hactive);
	lt8912_write(lt, I2C_ADDR_CEC_DSI, 0x2f, 0x0c);

	lt8912_write_w(lt, I2C_ADDR_CEC_DSI, CEC_DSI_HTOTAL, h_total);
	lt8912_write_w(lt, I2C_ADDR_CEC_DSI, CEC_DSI_VTOTAL, v_total);
	lt8912_write_w(lt, I2C_ADDR_CEC_DSI, CEC_DSI_VBP, vbp);
	lt8912_write_w(lt, I2C_ADDR_CEC_DSI, CEC_DSI_VFP, vfp);
	lt8912_write_w(lt, I2C_ADDR_CEC_DSI, CEC_DSI_HBP, hbp);
	lt8912_write_w(lt, I2C_ADDR_CEC_DSI, CEC_DSI_HFP, hfp);
	return vactive;
}

static int lt8912_video_on(struct lt *lt, bool on, u32 flags)
{
	int ret = -EINVAL;
	int yres;

	debug("%s: enter\n", __func__);


	yres = lt8912_video_setup(lt);
	if (yres < 0) {
		ret = yres;
		goto end;
	}

	lt8912_write_array(lt, lt8912_ddsconfig,
				sizeof(lt8912_ddsconfig));

	lt8912_write_array(lt, lt8912_rxlogicres,
				sizeof(lt8912_rxlogicres));

	lt8912_write_array(lt,lt8912_lvds_bypass_cfg,
	 			sizeof(lt8912_lvds_bypass_cfg));

	ret = 0;

end:
	return ret;
}

/* Device Operations */
static int lt8912_power_on(struct lt *lt, bool on, u32 flags)
{
	int ret = -EINVAL;

	debug("%s: %d\n", __func__, on);

	if (on && !lt->is_power_on) {
		u32 lanes = lt->dsi_lanes;

		lt8912_write_array(lt, lt8912_init_setup,
					sizeof(lt8912_init_setup));

		lt8912_write(lt, I2C_ADDR_CEC_DSI, CEC_DSI_LANE,
			lanes & 3);
		lt8912_write(lt, I2C_ADDR_MAIN, MAIN_LANE_EN,
			5 | (((1 << lanes) - 1) << (7 - lanes)));

		lt8912_write_array(lt, lt8912_mipi_basic_set,
                                sizeof(lt8912_mipi_basic_set));

		lt->is_power_on = true;
	} else if (!on) {
		/* power down hdmi */
		lt->is_power_on = false;
	}

	return ret;
}

static void lt8912_check_hpd_work(struct lt *lt)
{
	int ret;
	int connected = 0;
	u8 reg_val = 0;

	/* Check if cable is already connected.
	 * Since lt8912_irq line is edge triggered,
	 * if cable is already connected by this time
	 * it won't trigger HPD interrupt.
	 */
	lt8912_read(lt, I2C_ADDR_MAIN, 0xC1, &reg_val, 1);

	connected  = (reg_val & BIT(7));
	if (lt->prev_connected != connected) {
		lt->prev_connected = connected;
		debug("%s: connected = %d\n", __func__, connected);

		if (connected) {
			ret = lt8912_read_edid(lt, sizeof(lt->edid_buf),
				lt->edid_buf);
			if (ret)
				printf("%s: edid read failed\n", __func__);

			printf("%s: Rx CONNECTED\n", __func__);
			lt8912_power_on(lt, 1, 0);
			lt8912_video_on(lt, 1, 0);
		} else {
			printf("%s: Rx DISCONNECTED\n", __func__);

		} 
	}
}

struct lt *g_lt;

void lt8912_poll(void)
{
	struct lt *lt = g_lt;
	ulong elapsed;
	ulong now;

	now = get_timer(0);
	elapsed = now - lt->start_time;
	if (elapsed >= CONFIG_SYS_HZ/2) {
		lt8912_check_hpd_work(lt);
		lt->start_time = now;
	}
}

static int lt8912_enable_backlight(struct udevice *dev)
{
	struct lt *lt = dev_get_priv(dev);
	int ret;

	debug("%s:\n", __func__);
	ret = lt8912_gpio_configure(lt, true);
	if (ret) {
		printf("%s: Failed to configure GPIOs\n", __func__);
		return ret;
	}
	mdelay(20);

	ret = lt8912_read_device_rev(lt);
	if (ret) {
		printf("%s: Failed to read chip rev\n", __func__);
		return ret;
	}

	if (lt->audio) {
		debug("%s: enabling default audio configs\n", __func__);
		lt8912_write_array(lt, I2S_cfg, sizeof(I2S_cfg));
	}

	lt8912_power_on(lt, 1, 0);
	lt8912_video_on(lt, 1, 0);

	lt->start_time = get_timer(0);
	g_lt = lt;
	set_poll_rtn(lt8912_poll);
	return 0;
}

static int lt8912_set_backlight(struct udevice *dev, int percent)
{
	debug("%s:\n", __func__);
	if (percent) {
		lt8912_enable_backlight(dev);
	} else {
		set_poll_rtn(NULL);
	}
	return 0;
}

static int lt8912_get_display_timing(struct udevice *dev,
					    struct display_timing *timings)
{
	struct lt *lt = dev_get_priv(dev);

	memcpy(timings, &lt->timings, sizeof(*timings));
	return 0;
}

static int of_parse_phandle(ofnode np, const char *propname, ofnode *ph)
{
	struct ofnode_phandle_args phandle;
	int ret;

	ret = ofnode_parse_phandle_with_args(np, propname, NULL, 0, 0, &phandle);
	if (ret) {
		printf("Can't find %s property (%d)\n", propname, ret);
		return ret;
	}
	*ph = phandle.node;
	return 0;
}

static int lt8912_ofdata_to_platdata(struct udevice *dev)
{
	struct lt *lt = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	struct udevice *reg;
	ofnode ddc;
	int ret = 0;
	u32 dsi_lanes;

	debug("!!!%s:\n", __func__);

	lt->dev = dev;
	lt->audio = ofnode_read_bool(np, "adi,enable-audio");

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &lt->gds_reset,
			GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (ret) {
		debug("%s: Warning: cannot get reset-gpios: ret=%d\n",
		      __func__, ret);
		if (ret != -ENOENT) {
			debug("reset-gpios %d\n", ret);
			return log_ret(ret);
		}
	} else {
		lt->gd_reset = &lt->gds_reset;
		debug("%s: %s %d %lx\n", __func__, lt->gd_reset->dev->name,
			lt->gd_reset->offset, lt->gd_reset->flags);
	}

	ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
					   "avdd-supply", &reg);
	if (!ret) {
		lt->avdd = reg;
	} else if (ret != -ENOENT) {
		debug("%s: Failed to enable avdd: ret=%d\n", __func__, ret);
		return ret;
	}

	ret = of_parse_phandle(np, "display-dsi", &lt->disp_dsi);
	if (ret) {
		debug("display-dsi %d\n", ret);
		return ret;
	}
	if (ofnode_read_u32(lt->disp_dsi, "dsi-lanes", &dsi_lanes) < 0)
		return -EINVAL;
	if (dsi_lanes < 1 || dsi_lanes > 4)
		return -EINVAL;
	lt->dsi_lanes = dsi_lanes;

	ret = ofnode_decode_display_timing(lt->disp_dsi, 0, &lt->timings);
	if (ret < 0) {
		debug("ofnode_decode_display_timing %d\n", ret);
		return ret;
	}
	ret = of_parse_phandle(np, "ddc-i2c-bus", &ddc);
	if (!ret) {
		ret = uclass_get_device_by_ofnode(UCLASS_I2C, ddc, &lt->ddc);

		if (ret) {
			debug("%s: ddc failed ret=%d\n", __func__, ret);
		}
		return ret;
	}
	return 0;
}

static int lt8912_probe(struct udevice *dev)
{
	struct lt *lt = dev_get_priv(dev);
	int ret;

	if (lt->avdd) {
		ret = regulator_set_enable(lt->avdd, true);
		if (ret) {
			debug("%s: Regulator avdd enable failed ret=%d\n", __func__, ret);
		}
	}
	return 0;
}

static const struct panel_ops lt8912_ops = {
	.enable_backlight	= lt8912_enable_backlight,
	.get_display_timing	= lt8912_get_display_timing,
	.set_backlight		= lt8912_set_backlight,
};

static const struct udevice_id lt8912_dt_match[] = {
	{.compatible = "lontium,lt8912"},
	{}
};

U_BOOT_DRIVER(lt8912) = {
	.name	= "lt8912",
	.id	= UCLASS_PANEL,
	.of_match = lt8912_dt_match,
	.ofdata_to_platdata	= lt8912_ofdata_to_platdata,
	.probe	= lt8912_probe,
	.ops	= &lt8912_ops,
	.priv_auto_alloc_size	= sizeof(struct lt),
};
