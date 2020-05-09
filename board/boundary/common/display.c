/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/gpio.h>
#include <dm.h>
#include <i2c.h>
#include "bd_common.h"

int board_detect_lcd133(struct display_info_t const *di)
{
#ifdef CONFIG_DM_I2C
	struct udevice *bus;
	struct udevice *i2c_dev;
#else
	u8 orig_i2c_bus;
	u8 val8;
#endif
	int ret;

	if (di->bus_gp) {
		gpio_set_value(di->bus_gp, 1);
		if (di->bus_gp_delay_ms)
			mdelay(di->bus_gp_delay_ms);
	}
#ifdef CONFIG_DM_I2C
	ret = uclass_get_device_by_seq(UCLASS_I2C, di->bus_num, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
	} else {
		ret = dm_i2c_probe(bus, di->addr_num, 0, &i2c_dev);
		if (ret == 0) {
			ret = dm_i2c_reg_write(i2c_dev, 0, 0xf);	/* 5.5V to vpos */
			if (ret < 0) {
				printf("%s: i2c write(0) vpos failed(%d)\n", __func__, ret);
			} else {
				/* now VNEG -6V output */
				ret = dm_i2c_reg_write(i2c_dev, 1, 0xf);	/* -5.5V to vneg */
				if (ret < 0) {
					printf("%s: i2c write(0) vneg failed(%d)\n", __func__, ret);
				}
			}
		}
	}
#else
	orig_i2c_bus = i2c_get_bus_num();
	ret = i2c_set_bus_num(di->bus_num);
	if (ret == 0)
		ret = i2c_probe(di->addr_num);
	if (ret == 0) {
		val8 = 0xf;	/* VPOS 5.5V output */
		ret = i2c_write(di->addr_num, 0, 1, &val8, 1);
		if (ret < 0) {
			printf("%s: i2c write(0) vpos failed(%d)\n", __func__, ret);
		} else {
			/* now VNEG -5.5V output */
			ret = i2c_write(di->addr_num, 1, 1, &val8, 1);
			if (ret < 0) {
				printf("%s: i2c write(0) vneg failed(%d)\n", __func__, ret);
			}
		}
	}
	i2c_set_bus_num(orig_i2c_bus);
#endif
	if (di->bus_gp)
		gpio_set_value(di->bus_gp, 0);
	return (ret == 0);
}

