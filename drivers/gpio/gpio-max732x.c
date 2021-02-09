// SPDX-License-Identifier: GPL-2.0-only
/*
 *  MAX732x I2C Port Expander with 8/16 I/O
 *
 *  Copyright (C) 2007 Marvell International Ltd.
 *  Copyright (C) 2008 Jack Ren <jack.ren@marvell.com>
 *  Copyright (C) 2008 Eric Miao <eric.miao@marvell.com>
 *  Copyright (C) 2015 Linus Walleij <linus.walleij@linaro.org>
 *
 *  Derived from drivers/gpio/pca953x.c
 */

#include <common.h>
#include <errno.h>
#include <dm.h>
#include <fdtdec.h>
#include <i2c.h>
#include <malloc.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/bitops.h>


/*
 * Each port of MAX732x (including MAX7319) falls into one of the
 * following three types:
 *
 *   - Push Pull Output
 *   - Input
 *   - Open Drain I/O
 *
 * designated by 'O', 'I' and 'P' individually according to MAXIM's
 * datasheets. 'I' and 'P' ports are interrupt capables, some with
 * a dedicated interrupt mask.
 *
 * There are two groups of I/O ports, each group usually includes
 * up to 8 I/O ports, and is accessed by a specific I2C address:
 *
 *   - Group A : by I2C address 0b'110xxxx
 *   - Group B : by I2C address 0b'101xxxx
 *
 * where 'xxxx' is decided by the connections of pin AD2/AD0.  The
 * address used also affects the initial state of output signals.
 *
 * Within each group of ports, there are five known combinations of
 * I/O ports: 4I4O, 4P4O, 8I, 8P, 8O, see the definitions below for
 * the detailed organization of these ports. Only Goup A is interrupt
 * capable.
 *
 * GPIO numbers start from 'gpio_base + 0' to 'gpio_base + 8/16',
 * and GPIOs from GROUP_A are numbered before those from GROUP_B
 * (if there are two groups).
 *
 * NOTE: MAX7328/MAX7329 are drop-in replacements for PCF8574/a, so
 * they are not supported by this driver.
 */

#define PORT_NONE	0x0	/* '/' No Port */
#define PORT_OUTPUT	0x1	/* 'O' Push-Pull, Output Only */
#define PORT_INPUT	0x2	/* 'I' Input Only */
#define PORT_OPENDRAIN	0x3	/* 'P' Open-Drain, I/O */

#define IO_4I4O		0x5AA5	/* O7 O6 I5 I4 I3 I2 O1 O0 */
#define IO_4P4O		0x5FF5	/* O7 O6 P5 P4 P3 P2 O1 O0 */
#define IO_8I		0xAAAA	/* I7 I6 I5 I4 I3 I2 I1 I0 */
#define IO_8P		0xFFFF	/* P7 P6 P5 P4 P3 P2 P1 P0 */
#define IO_8O		0x5555	/* O7 O6 O5 O4 O3 O2 O1 O0 */

#define GROUP_A(x)	((x) & 0xffff)	/* I2C Addr: 0b'110xxxx */
#define GROUP_B(x)	((x) << 16)	/* I2C Addr: 0b'101xxxx */

#define INT_NONE	0x0	/* No interrupt capability */
#define INT_NO_MASK	0x1	/* Has interrupts, no mask */
#define INT_INDEP_MASK	0x2	/* Has interrupts, independent mask */
#define INT_MERGED_MASK 0x3	/* Has interrupts, merged mask */

#define INT_CAPS(x)	(((uint64_t)(x)) << 32)

enum {
	MAX7319,
	MAX7320,
	MAX7321,
	MAX7322,
	MAX7323,
	MAX7324,
	MAX7325,
	MAX7326,
	MAX7327,
};

static uint64_t max732x_features[] = {
	[MAX7319] = GROUP_A(IO_8I) | INT_CAPS(INT_MERGED_MASK),
	[MAX7320] = GROUP_B(IO_8O),
	[MAX7321] = GROUP_A(IO_8P) | INT_CAPS(INT_NO_MASK),
	[MAX7322] = GROUP_A(IO_4I4O) | INT_CAPS(INT_MERGED_MASK),
	[MAX7323] = GROUP_A(IO_4P4O) | INT_CAPS(INT_INDEP_MASK),
	[MAX7324] = GROUP_A(IO_8I) | GROUP_B(IO_8O) | INT_CAPS(INT_MERGED_MASK),
	[MAX7325] = GROUP_A(IO_8P) | GROUP_B(IO_8O) | INT_CAPS(INT_NO_MASK),
	[MAX7326] = GROUP_A(IO_4I4O) | GROUP_B(IO_8O) | INT_CAPS(INT_MERGED_MASK),
	[MAX7327] = GROUP_A(IO_4P4O) | GROUP_B(IO_8O) | INT_CAPS(INT_NO_MASK),
};

struct max732x_chip {
	struct udevice *dev;
	int addr;
	int addr_a;
	int addr_b;

	unsigned int	mask_group_a;
	unsigned int	dir_input;
	unsigned int	dir_output;

	struct mutex	lock;
	uint8_t		reg_out[2];
};

static int max732x_writeb(struct max732x_chip *chip, int group_a, uint8_t val)
{
	uint8_t buf[4];
	struct i2c_msg msg;
	int ret;

	msg.addr = group_a ? chip->addr_a : chip->addr_b;
	msg.flags = 0;
	buf[0] = val;
	msg.buf = buf;
	msg.len = 1;
	ret = dm_i2c_xfer(chip->dev, &msg, 1);
	if (ret)
		debug("%s group_a(%x)=%x, failed(%d)\n", __func__, group_a, val, ret);
	return ret;
}

static int max732x_readb(struct max732x_chip *chip, int group_a, uint8_t *val)
{
	uint8_t rx_buf[4];
	struct i2c_msg msg;
	int ret;

	msg.addr = group_a ? chip->addr_a : chip->addr_b;
	msg.flags = I2C_M_RD;
	msg.len = 1;
	msg.buf = rx_buf;
	ret = dm_i2c_xfer(chip->dev, &msg, 1);
	if (ret) {
		debug("%s group_a(%x), failed(%d)\n", __func__, group_a, ret);
		return ret;
	}
	*val = rx_buf[0];
	return 0;
}

static inline int is_group_a(struct max732x_chip *chip, unsigned off)
{
	return (1u << off) & chip->mask_group_a;
}

static int max732x_chip_get_value(struct max732x_chip *chip, unsigned offset)
{
	uint8_t reg_val;
	int ret;

	ret = max732x_readb(chip, is_group_a(chip, offset), &reg_val);
	if (ret < 0)
		return ret;

	return !!(reg_val & (1u << (offset & 0x7)));
}

static void max732x_chip_set_mask(struct max732x_chip *chip, unsigned base, int mask,
				  int val)
{
	uint8_t reg_out;
	int ret;

	reg_out = (base > 7) ? chip->reg_out[1] : chip->reg_out[0];
	reg_out = (reg_out & ~mask) | (val & mask);

	ret = max732x_writeb(chip, is_group_a(chip, base), reg_out);
	if (ret < 0)
		return;

	/* update the shadow register then */
	if (base > 7)
		chip->reg_out[1] = reg_out;
	else
		chip->reg_out[0] = reg_out;
}

static void max732x_chip_set_value(struct max732x_chip *chip, unsigned offset, int val)
{
	unsigned base = offset & ~0x7;
	uint8_t mask = 1u << (offset & 0x7);

	max732x_chip_set_mask(chip, base, mask, val << (offset & 0x7));
}

static void max732x_chip_set_multiple(struct max732x_chip *chip,
				      unsigned long *mask, unsigned long *bits)
{
	unsigned mask_lo = mask[0] & 0xff;
	unsigned mask_hi = (mask[0] >> 8) & 0xff;

	if (mask_lo)
		max732x_chip_set_mask(chip, 0, mask_lo, bits[0] & 0xff);
	if (mask_hi)
		max732x_chip_set_mask(chip, 8, mask_hi, (bits[0] >> 8) & 0xff);
}

static int max732x_direction_input(struct udevice *dev, uint offset)
{
	struct max732x_chip *chip = dev_get_platdata(dev);
	unsigned int mask = 1u << offset;

	if ((mask & chip->dir_input) == 0) {
		debug("%s port %d is output only\n", dev->name, offset);
		return -EACCES;
	}

	/*
	 * Open-drain pins must be set to high impedance (which is
	 * equivalent to output-high) to be turned into an input.
	 */
	if ((mask & chip->dir_output))
		max732x_chip_set_value(chip, offset, 1);

	return 0;
}

static int max732x_direction_output(struct udevice *dev, uint offset,
		int val)
{
	struct max732x_chip *chip = dev_get_platdata(dev);
	unsigned int mask = 1u << offset;

	if ((mask & chip->dir_output) == 0) {
		debug("%s port %d is input only\n", dev->name, offset);
		return -EACCES;
	}

	max732x_chip_set_value(chip, offset, val);
	return 0;
}

static int max732x_get_value(struct udevice *dev, unsigned offset)
{
	return max732x_chip_get_value(dev_get_platdata(dev), offset);
}

static int max732x_set_value(struct udevice *dev, unsigned offset, int val)
{
	max732x_chip_set_value(dev_get_platdata(dev), offset, val);
	return 0;
}

static int max732x_get_function(struct udevice *dev, uint offset)
{
	struct max732x_chip *chip = dev_get_platdata(dev);
	unsigned int mask = 1u << offset;
	u32 out_val;

	if ((mask & chip->dir_input) == 0) {
		return GPIOF_OUTPUT;
	}
	if ((mask & chip->dir_output) == 0) {
		return GPIOF_INPUT;
	}
	out_val = (chip->reg_out[1] << 8) | chip->reg_out[0];
	if (mask & out_val)
		return GPIOF_INPUT;
	return GPIOF_OUTPUT;
}

static int max732x_xlate(struct udevice *dev, struct gpio_desc *desc,
			 struct ofnode_phandle_args *args)
{
	desc->offset = args->args[0];
	desc->flags = args->args[1] & GPIO_ACTIVE_LOW ? GPIOD_ACTIVE_LOW : 0;

	return 0;
}

static const struct dm_gpio_ops max732x_ops = {
	.direction_input	= max732x_direction_input,
	.direction_output	= max732x_direction_output,
	.get_value		= max732x_get_value,
	.set_value		= max732x_set_value,
	.get_function		= max732x_get_function,
	.xlate			= max732x_xlate,
};

static int max732x_setup_gpio(struct max732x_chip *chip,
		ulong driver_data, unsigned gpio_start)
{
	uint32_t id_data = (uint32_t)max732x_features[driver_data];
	int i, port = 0;

	for (i = 0; i < 16; i++, id_data >>= 2) {
		unsigned int mask = 1 << port;

		switch (id_data & 0x3) {
		case PORT_OUTPUT:
			chip->dir_output |= mask;
			break;
		case PORT_INPUT:
			chip->dir_input |= mask;
			break;
		case PORT_OPENDRAIN:
			chip->dir_output |= mask;
			chip->dir_input |= mask;
			break;
		default:
			continue;
		}

		if (i < 8)
			chip->mask_group_a |= mask;
		port++;
	}

	return port;
}

static int max732x_probe(struct udevice *dev)
{
	struct max732x_chip *chip = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	char name[32];
	char *str;
	ofnode np = dev_ofnode(dev);
	uint16_t addr_a, addr_b;
	int addr;
	ulong driver_data;
	int ret, nr_port;
	u32 out_set;
	unsigned long mask, val;

	addr = dev_read_addr(dev);
	debug("!!!%s: %x\n", __func__, addr);
	if (addr == 0)
		return -ENODEV;

	chip->addr = addr;
	chip->dev = dev;
	driver_data = dev_get_driver_data(dev);

	nr_port = max732x_setup_gpio(chip, driver_data, -1);

	addr_a = (chip->addr & 0x0f) | 0x60;
	addr_b = (chip->addr & 0x0f) | 0x50;

	switch (chip->addr & 0x70) {
	case 0x60:
		chip->addr_a = addr_a;
		if (nr_port > 8) {
			chip->addr_b = addr_b;
		}
		break;
	case 0x50:
		chip->addr_b = addr_b;
		if (nr_port > 8) {
			chip->addr_a = addr_a;
		}
		break;
	default:
		printf("invalid I2C address specified %02x\n", addr);
		return -EINVAL;
	}

	ret = max732x_readb(chip, is_group_a(chip, 0), &chip->reg_out[0]);
	if (ret)
		return ret;
	if (nr_port > 8) {
		ret = max732x_readb(chip, is_group_a(chip, 8), &chip->reg_out[1]);
		if (ret)
			return ret;
	}

	snprintf(name, sizeof(name), "gpio@%x_", chip->addr);
	str = strdup(name);
	if (!str)
		return -ENOMEM;
	uc_priv->bank_name = str;
	uc_priv->gpio_count = nr_port;

	/* set the output IO default voltage */
	if (!ofnode_read_u32(np, "out-default", &out_set)) {
		mask = out_set & chip->dir_output;
		val = out_set >> 16;
		max732x_chip_set_multiple(chip, &mask, &val);
	}

	return 0;
}

static const struct udevice_id max732x_ids[] = {
	{ .compatible = "maxim,max7319", .data = MAX7319 },
	{ .compatible = "maxim,max7320", .data = MAX7320 },
	{ .compatible = "maxim,max7321", .data = MAX7321 },
	{ .compatible = "maxim,max7322", .data = MAX7322 },
	{ .compatible = "maxim,max7323", .data = MAX7323 },
	{ .compatible = "maxim,max7324", .data = MAX7324 },
	{ .compatible = "maxim,max7325", .data = MAX7325 },
	{ .compatible = "maxim,max7326", .data = MAX7326 },
	{ .compatible = "maxim,max7327", .data = MAX7327 },
	{ }
};

U_BOOT_DRIVER(max732x) = {
	.name		= "max732x",
	.id		= UCLASS_GPIO,
	.ops		= &max732x_ops,
	.probe		= max732x_probe,
	.platdata_auto_alloc_size = sizeof(struct max732x_chip),
	.of_match	= max732x_ids,
};
