/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <i2c.h>
#include "bd_common.h"

void fan53526_init(void)
{
	int ret;
	u8 orig_i2c_bus;
	u8 val8;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_FAN53526);
#define I2C_ADDR_FAN53526	0x60
	/*
	 * Vout = 600mV + n * 6.25 mV
	 * n = (Vout - 600000uV) / 6250uV
	 */
	/* max is 1.39375 V */
	/*
	 * VDD_ARM_IN and VDD_SOC_IN must be at least 125 mV higher than the
	 * LDO Output Set Point for correct voltage regulation
	 */
	val8 = DIV_ROUND_UP((1393750 - 600000), 6250);
	val8 |= 0x80;	/* Enable output */
	ret = i2c_write(I2C_ADDR_FAN53526, 0, 1, &val8, 1);
	if (ret < 0)
		printf("%s: i2c write(0) failed(%d)\n", __func__, ret);
	ret = i2c_write(I2C_ADDR_FAN53526, 1, 1, &val8, 1);
	if (ret < 0)
		printf("%s: i2c write(1) failed(%d)\n", __func__, ret);

	i2c_set_bus_num(orig_i2c_bus);
}
