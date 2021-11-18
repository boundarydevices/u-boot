// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2021 Boundary Devices
 */

#include <common.h>
#include <asm/gpio.h>
#include <backlight.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <errno.h>
#include <i2c.h>
#include <linux/delay.h>
#include <log.h>
#include <malloc.h>

/* registers */
#define BD2606_REG_LEDACNT	0x00	/* LEDACNT Register */
#define BD2606_REG_LEDBCNT	0x01	/* LEDBCNT Register */
#define BD2606_REG_LEDCCNT	0x02	/* LEDCCNT Register */
#define BD2606_REG_LEDPWRCNT 0x03	/* LEDPWRCNT Register */


#define BD2606_LEDPWRCNT_MASK  0x7F  /* Mask for LED PWR Control register */
#define BD2606_LED_MASK		0x3F	/*   LED string enables mask */
#define BD2606_LED_BITS		0x06	/*   Bits of LED string enables */
#define BD2606_LED_LEDA1	0x01
#define BD2606_LED_LEDA2	0x02
#define BD2606_LED_LEDB1	0x04
#define BD2606_LED_LEDB2	0x08
#define BD2606_LED_LEDC1	0x10
#define BD2606_LED_LEDC2	0x20


#define MAX_BRIGHTNESS		63   /* 64 steps */
#define INIT_BRIGHT		10

enum bd2606_chip_id {
	BD2606
};

struct bd2606_backlight_priv {
	struct udevice *dev;
	const char *name;
	u32	enabled;
	u32	default_brightness;
	u32	brightness;
	u32	leden, leden_curr;
};

static int bd2606_write(struct bd2606_backlight_priv *priv, u8 reg, u8 val)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(priv->dev);
	uint8_t buf[4];
	struct i2c_msg msg;
	int ret;

	debug("%s:reg=%02x val=%02x\n", __func__, reg, val);
	msg.addr = chip->chip_addr;
	msg.flags = 0;
	buf[0] = reg;
	buf[1] = val;
	msg.buf = buf;
	msg.len = 2;
	ret = dm_i2c_xfer(priv->dev, &msg, 1);
	if (ret) {
		debug("%s reg(%x)=%x, failed(%d)\n", __func__, reg, val, ret);
	}
	return ret;
}

static u8 bd2606_read(struct bd2606_backlight_priv *priv, u8 reg)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(priv->dev);
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];
	struct i2c_msg msgs[2];
	int ret;

	tx_buf[0] = reg;
	msgs[0].addr = chip->chip_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = tx_buf;

	msgs[1].addr = chip->chip_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = rx_buf;
	ret = dm_i2c_xfer(priv->dev, msgs, 2);
	if (ret) {
		debug("%s reg(%x), failed(%d)\n", __func__, reg, ret);
		return ret;
	}
	return rx_buf[0];
}

static int bd2606_update_field(struct bd2606_backlight_priv *priv, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bd2606_read(priv, reg);
	if (ret < 0) {
		debug("%s:failed to read 0x%.2x\n", __func__, reg);
		return ret;
	}
	tmp = (u8)ret;
	tmp &= ~mask;
	tmp |= data & mask;
	return bd2606_write(priv, reg, tmp);
}

static int bd2606_set_brightness(struct bd2606_backlight_priv *priv, u32 brightness)
{
	int ret;
	u8 val;

	/* lower nibble of brightness goes in upper nibble of LSB register */
	val = (brightness & 0x3F); //<< BD2606_WLED_ISET_LSB_SHIFT;

	ret = bd2606_write(priv, BD2606_REG_LEDACNT, val);
	ret = bd2606_write(priv, BD2606_REG_LEDBCNT, val);
	ret = bd2606_write(priv, BD2606_REG_LEDCCNT, val);
	return ret;
}

static int bd2606_backlight_enable(struct udevice *dev)
{
	struct bd2606_backlight_priv *priv = dev_get_priv(dev);
	int level = priv->brightness;
	int leden;
	int ret;

	debug("%s:level=0x%02x\n", __func__, level);
	leden = level ? priv->leden : 0;
	if (priv->leden_curr != leden) {
		priv->leden_curr = leden;
		debug("%s: leden=0x%02x\n", __func__, leden);
		bd2606_update_field(priv, BD2606_REG_LEDPWRCNT,
			BD2606_LED_MASK, leden);
	}

	ret = bd2606_set_brightness(priv, level);
	if (ret)
		return log_ret(ret);
	priv->enabled = level ? 1 : 0;

	return 0;
}

static int bd2606_backlight_set_brightness(struct udevice *dev, int percent)
{
	struct bd2606_backlight_priv *priv = dev_get_priv(dev);
	int level;

	debug("%s:percent=%d default_brightness=0x%02x\n", __func__, percent, priv->default_brightness);
	if (percent == BACKLIGHT_OFF)
		percent = 0;

	if (percent == BACKLIGHT_DEFAULT) {
		level = priv->default_brightness;
	} else {
		level = 63 * percent / 100;
	}
	priv->brightness = level;
	return bd2606_backlight_enable(dev);
}

static int bd2606_backlight_ofdata_to_platdata(struct udevice *dev)
{
	struct bd2606_backlight_priv *priv = dev_get_priv(dev);
	int i, ret, num_entry;
	u32 sources[BD2606_LED_BITS];
	int size;
	int leden = BD2606_LED_MASK; /* all on is default */


	i = dev_read_u32_default(dev, "default-brightness", INIT_BRIGHT);
	priv->default_brightness = i;
	priv->brightness = i;

	size = dev_read_size(dev,"led-sources");
	if (size >= 4) {
		num_entry = size / sizeof(fdt32_t);
		if (num_entry > ARRAY_SIZE(sources))
			num_entry = ARRAY_SIZE(sources);
		ret = dev_read_u32_array(dev, "led-sources", sources, num_entry);
		if (ret < 0) {
			debug("%s: led-sources node is invalid.\n", __func__);
			return -EINVAL;
		}
		leden = 0;
		/* for each enable in source, set bit in led enable */
		for (i = 0; i < num_entry; i++) {
			leden |= (1 << sources[i]);
			debug("%s: src=%d leden=0x%02x\n", __func__, sources[i], leden);
		}
	}
	priv->leden = leden;
	priv->dev = dev;
	return 0;
}

static int bd2606_backlight_probe(struct udevice *dev)
{
	struct bd2606_backlight_priv *priv = dev_get_priv(dev);

	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	bd2606_update_field(priv, BD2606_REG_LEDPWRCNT,
		BD2606_LED_MASK, 0);
	return 0;
}

static const struct backlight_ops bd2606_backlight_ops = {
	.enable		= bd2606_backlight_enable,
	.set_brightness	= bd2606_backlight_set_brightness,
};

static const struct udevice_id bd2606_backlight_ids[] = {
	{ .compatible = "rohm,bd2606" },
	{ }
};

U_BOOT_DRIVER(bd2606_backlight) = {
	.name	= "bd2606_backlight",
	.id	= UCLASS_PANEL_BACKLIGHT,
	.of_match = bd2606_backlight_ids,
	.ops	= &bd2606_backlight_ops,
	.ofdata_to_platdata	= bd2606_backlight_ofdata_to_platdata,
	.probe		= bd2606_backlight_probe,
	.priv_auto_alloc_size	= sizeof(struct bd2606_backlight_priv),
};
