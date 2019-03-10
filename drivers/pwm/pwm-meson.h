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
