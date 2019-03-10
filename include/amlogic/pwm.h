/*
 * Amlogic I2C controller Diver
 *
 * Copyright (C) 2018 Amlogic Corporation
 *
 * Licensed under the GPL-2 or later.
 *
 */

#ifndef __PWM_H__
#define __PWM_H__

/*
 * @pwm_index: Controller Index.
 * @reg: Controller registers address.
 */

#define MESON_PWM0		0
#define MESON_PWM1		1
#define MESON_PWM2		2
#define MESON_PWM3		3

#define NO_DOUBLE_CHANNEL	0
#define IS_DOUBLE_CHANNEL	1
#define NO_BLINK 		0
#define IS_BLINK		1

enum {
	PWM_AB		= 0x0,
	PWM_CD		= 0x1,
	PWM_EF		= 0x2,
	PWMAO_AB	= 0x3,
	PWMAO_CD	= 0x4,
};

struct meson_pwm_platdata {
	unsigned int pwm_index;
	ulong 	reg;
	bool	is_double_channel;
	bool	is_blink;
};

#endif /* __PWM_H__ */
