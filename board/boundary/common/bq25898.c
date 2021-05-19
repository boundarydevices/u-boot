/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/arch/crm_regs.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <i2c.h>
#include <linux/delay.h>
#include <log.h>
#include "bd_common.h"

void bq25898_init(int i2c_bus, int i2c_address)
{
	int retry;
	int ret;
	u8 tx_buf[4];
	u8 rx_buf[4];
#ifdef CONFIG_DM_I2C
	struct udevice *bus;
	struct i2c_msg msg[2];
	struct dm_i2c_ops *ops;
#else
	u8 orig_i2c_bus;
#endif

/*
 * i2c dev 2;
 * i2c mw 6b 14 80; sleep .1;
 * i2c mw 6b 4 1f
 * i2c mw 6b 2 2; sleep .1;
 * i2c mw 6b 0 7f;
 * i2c mw 6b 7 8d;
 */

#ifdef CONFIG_DM_I2C
	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
	} else {
		ops = i2c_get_ops(bus);
		msg[0].addr = i2c_address;
		msg[0].flags = 0;
		msg[0].len = 2;
		msg[0].buf = tx_buf;

		msg[1].addr = i2c_address;
		msg[1].flags = I2C_M_RD;
		msg[1].len = 1;
		msg[1].buf = rx_buf;

		/* reset chip */
		tx_buf[0] = 0x14;
		tx_buf[1] = 0x80;
		ret = ops->xfer(bus, msg, 1);
		if (ret < 0)
			printf("%s: reset ret=%d\n", __func__, ret);

		msg[0].len = 1;
		retry = 0;
		do {
			ret = ops->xfer(bus, msg, 2);
			if (ret < 0)
				printf("%s: reg14=0x%x ret=%d\n", __func__,
						rx_buf[0], ret);
			if ((ret >= 0) && ((rx_buf[0] & 0x80) == 0))
				break;
			udelay(10);
		} while (++retry < 10);

		msg[0].len = 2;
		tx_buf[0] = 4;
		tx_buf[1] = 0x1f;
		ret = ops->xfer(bus, msg, 1);
		if (ret < 0)
			printf("%s: fast charge ret=%d\n", __func__, ret);

		/* Force DPDM */
		tx_buf[0] = 2;
		tx_buf[1] = 2;
		ret = ops->xfer(bus, msg, 1);
		if (ret < 0)
			printf("%s: force_dpdm ret=%d\n", __func__, ret);

		msg[0].len = 1;
		retry = 0;
		do {
			ret = ops->xfer(bus, msg, 2);
			if (ret < 0)
				printf("%s: reg2=0x%x ret=%d\n", __func__, rx_buf[0], ret);
			if ((ret >= 0) && ((rx_buf[0] & 0x2) == 0))
				break;
			udelay(10);
		} while (++retry < 10);

		msg[0].len = 2;
		tx_buf[0] = 0;
		tx_buf[1] = 0x7f;
		ret = ops->xfer(bus, msg, 1);
		if (ret < 0)
			printf("%s: iilim ret=%d\n", __func__, ret);

		tx_buf[0] = 7;
		tx_buf[1] = 0x8d;
		ret = ops->xfer(bus, msg, 1);
		if (ret < 0)
			printf("%s: watchdog ret=%d\n", __func__, ret);
	}
#else
	orig_i2c_bus = i2c_get_bus_num();
	ret = i2c_set_bus_num(i2c_bus);
	tx_buf[0] = 0x80;
	ret = i2c_write(i2c_address, 0x14, 1, tx_buf, 1);
	if (ret < 0)
		printf("%s: reset ret=%d\n", __func__, ret);

	retry = 0;
	do {
		i2c_read(i2c_address, 0x14, 1, rx_buf, 1);
		if (ret < 0)
			printf("%s: reg14=0x%x ret=%d\n", __func__,
					rx_buf[0], ret);
		if ((ret >= 0) && ((rx_buf[0] & 0x80) == 0))
			break;
		udelay(10);
	} while (++retry < 10);

	tx_buf[0] = 0x1f;
	ret = i2c_write(i2c_address, 0x04, 1, tx_buf, 1);
	if (ret < 0)
		printf("%s: fast charge ret=%d\n", __func__, ret);

	tx_buf[0] = 0x02;
	ret = i2c_write(i2c_address, 0x02, 1, tx_buf, 1);
	if (ret < 0)
		printf("%s: force_dpdm ret=%d\n", __func__, ret);

	retry = 0;
	do {
		i2c_read(i2c_address, 0x02, 1, rx_buf, 1);
		if (ret < 0)
			printf("%s: reg2=0x%x ret=%d\n", __func__, rx_buf[0], ret);
		if ((ret >= 0) && ((rx_buf[0] & 0x02) == 0))
			break;
		udelay(10);
	} while (++retry < 10);

	tx_buf[0] = 0x7f;
	ret = i2c_write(i2c_address, 0x0, 1, tx_buf, 1);
	if (ret < 0)
		printf("%s: iilim ret=%d\n", __func__, ret);

	tx_buf[0] = 0x8d;
	ret = i2c_write(i2c_address, 0x07, 1, tx_buf, 1);
	if (ret < 0)
		printf("%s: watchdog ret=%d\n", __func__, ret);

	i2c_set_bus_num(orig_i2c_bus);
#endif
}
