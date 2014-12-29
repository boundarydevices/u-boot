/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __ASM_ARCH_MXC_MXC_I2C_H__
#define __ASM_ARCH_MXC_MXC_I2C_H__
#include <asm/imx-common/iomux-v3.h>
#include <asm/arch/sys_proto.h>

struct i2c_pin_ctrl {
	iomux_v3_cfg_t i2c_mode;
	iomux_v3_cfg_t gpio_mode;
	unsigned char gp;
	unsigned char spare;
};

struct i2c_pads_info {
	struct i2c_pin_ctrl scl;
	struct i2c_pin_ctrl sda;
};

#define I2C_PADS_INFO_CPU(cpu, i2cnum, scl_pad, scl_bank, scl_gp,	       \
		sda_pad, sda_bank, sda_gp, ctrl) {			       \
    .scl = {								       \
	.i2c_mode = NEW_PAD_CTRL(cpu##_PAD_##scl_pad##__##i2cnum##_SCL, ctrl), \
	.gpio_mode = NEW_PAD_CTRL(					       \
		cpu##_PAD_##scl_pad##__GPIO##scl_bank##_IO##scl_gp, ctrl),     \
	.gp = IMX_GPIO_NR(scl_bank, scl_gp)				       \
    },									       \
    .sda = {								       \
	.i2c_mode = NEW_PAD_CTRL(cpu##_PAD_##sda_pad##__##i2cnum##_SDA, ctrl), \
	.gpio_mode = NEW_PAD_CTRL(					       \
		cpu##_PAD_##sda_pad##__GPIO##sda_bank##_IO##sda_gp, ctrl),     \
	.gp = IMX_GPIO_NR(sda_bank, sda_gp)				       \
    }									       \
}

#if defined(CONFIG_MX6QDL)
#define I2C_PADS_INFO_ENTRY(i2cnum, scl_pad, scl_bank, scl_gp,		\
		sda_pad, sda_bank, sda_gp, ctrl)			\
	I2C_PADS_INFO_CPU(MX6Q, i2cnum, scl_pad, scl_bank, scl_gp,	\
		sda_pad, sda_bank, sda_gp, ctrl),			\
	I2C_PADS_INFO_CPU(MX6DL, i2cnum, scl_pad, scl_bank, scl_gp,	\
		sda_pad, sda_bank, sda_gp, ctrl)
#define I2C_PADS_INFO_ENTRY_SPACING 2
#else
#define I2C_PADS_INFO_ENTRY(i2cnum, scl_pad, scl_bank, scl_gp,		\
		sda_pad, sda_bank, sda_gp, ctrl)			\
	I2C_PADS_INFO_CPU(MX6, i2cnum, scl_pad, scl_bank, scl_gp,	\
		sda_pad, sda_bank, sda_gp, ctrl)
#define I2C_PADS_INFO_ENTRY_SPACING 1
#endif


#if defined(CONFIG_MX6QDL)
#define I2C_PADS(name, scl_i2c, scl_gpio, scl_gp, sda_i2c, sda_gpio, sda_gp) \
	struct i2c_pads_info mx6q_##name = {		\
		.scl = {				\
			.i2c_mode = MX6Q_##scl_i2c,	\
			.gpio_mode = MX6Q_##scl_gpio,	\
			.gp = scl_gp,			\
		},					\
		.sda = {				\
			.i2c_mode = MX6Q_##sda_i2c,	\
			.gpio_mode = MX6Q_##sda_gpio,	\
			.gp = sda_gp,			\
		}					\
	};						\
	struct i2c_pads_info mx6s_##name = {		\
		.scl = {				\
			.i2c_mode = MX6DL_##scl_i2c,	\
			.gpio_mode = MX6DL_##scl_gpio,	\
			.gp = scl_gp,			\
		},					\
		.sda = {				\
			.i2c_mode = MX6DL_##sda_i2c,	\
			.gpio_mode = MX6DL_##sda_gpio,	\
			.gp = sda_gp,			\
		}					\
	};


#define I2C_PADS_INFO(name)	\
	(is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D)) ? \
					&mx6q_##name : &mx6s_##name
#endif

void setup_i2c(unsigned i2c_index, int speed, int slave_addr,
		struct i2c_pads_info *p);
void bus_i2c_init(void *base, int speed, int slave_addr,
		int (*idle_bus_fn)(void *p), void *p);
int bus_i2c_read(void *base, uchar chip, uint addr, int alen, uchar *buf,
		int len);
int bus_i2c_write(void *base, uchar chip, uint addr, int alen,
		const uchar *buf, int len);

static inline int i2c_get_info_entry_offset(void) {
#if defined(CONFIG_MX6QDL)
	int cpu = get_cpu_type();

	if ((cpu != MXC_CPU_MX6Q) && (cpu != MXC_CPU_MX6D))
		return 1;
#endif
	return 0;
}

#endif
