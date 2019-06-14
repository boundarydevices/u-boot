/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/

#ifndef PWM_MESON_H
#define PWM_MESON_H

#define NSEC_PER_SEC 1000000000ULL

struct meson_pwm_reg {
	u32 dar;/* A/C/E Duty Register */
	u32 dbr;/* B/D/F Duty Register */
	u32 miscr;/* misc Register */
	u32 dsr;/*DS Register*/
	u32 tr;/*times Register*/
	u32 da2r;/* A2/C2/E2 Duty Register */
	u32 db2r;/* B2/D2/F2 Duty Register */
	u32 br;/*Blink Register*/
};

enum pwm_polarity {
	PWM_POLARITY_NORMAL,
	PWM_POLARITY_INVERSED,
};

struct meson_pwm_state {
	unsigned int period;
	unsigned int duty_cycle;
	unsigned int hi;
	unsigned int lo;
	unsigned int pre_div;
	enum pwm_polarity polarity;
	bool enabled;
};

/*
 * Same as above but for u64 dividends. divisor must be a 32-bit
 * number.
 */
#define DIV_ROUND_CLOSEST_ULL(x, divisor)(		\
{							\
	typeof(divisor) __d = divisor;			\
	unsigned long long _tmp = (x) + (__d) / 2;	\
	do_div(_tmp, __d);				\
	_tmp;						\
}							\
)

#endif
