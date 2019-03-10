/*
 * header file for pwm driver.
 *
 * Copyright (c) 2011 samsung electronics
 * Donghwa Lee <dh09.lee@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _pwm_h_
#define _pwm_h_

/* struct pwm_ops: Operations for the PWM uclass */
struct pwm_ops {
	/**
	 * set_config() - Set the PWM configuration
	 *
	 * @dev:	PWM device to update
	 * @channel:	PWM channel to update
	 * @period_ns:	PWM period in nanoseconds
	 * @duty_ns:	PWM duty period in nanoseconds
	 * @return 0 if OK, -ve on error
	 */
	int (*set_config)(struct udevice *dev, uint channel, uint period_ns,
			  uint duty_ns);

	/**
	 * set_enable() - Enable or disable the PWM
	 *
	 * @dev:	PWM device to update
	 * @channel:	PWM channel to update
	 * @enable:	true to enable, false to disable
	 * @return 0 if OK, -ve on error
	 */
	int (*set_enable)(struct udevice *dev, uint channel, bool enable);
	/**
	 * set_invert() - Set the PWM invert
	 *
	 * @dev:        PWM device to update
	 * @channel:    PWM channel to update
	 * @polarity:   true to invert, false to keep normal polarity
	 * @return 0 if OK, -ve on error
	 */
	int (*set_invert)(struct udevice *dev, uint channel, bool polarity);
#ifdef CONFIG_PWM_MESON
	/**
	 * set_times() - Set the PWM times
	 *
	 * @dev:        PWM device to update
	 * @channel:    PWM channel to update
	 * @times: 	  	dual channel to set times
	 * @return 0 if OK, -ve on error
	 */
	int (*set_times)(struct udevice *dev, uint channel, uint times);
	/**
	 * set_blink_times() - Set the PWM blink times
	 *
	 * @dev:        PWM device to update
	 * @channel:    PWM channel to update
	 * @times:   	set blink times
	 * @return 0 if OK, -ve on error
	 */
	int (*set_blink_times)(struct udevice *dev, uint channel, uint times);
	/**
	 * set_blink_enable() - Enable or disable the PWM blink
	 *
	 * @dev:        PWM device to update
	 * @channel:    PWM channel to update
	 * @enable:   	true to enable, false to disable
	 * @return 0 if OK, -ve on error
	 */
	int (*set_blink_enable)(struct udevice *dev, uint channel, bool enable);
#endif
};

#define pwm_get_ops(dev)	((struct pwm_ops *)(dev)->driver->ops)

/**
 * pwm_set_config() - Set the PWM configuration
 *
 * @dev:	PWM device to update
 * @channel:	PWM channel to update
 * @period_ns:	PWM period in nanoseconds
 * @duty_ns:	PWM duty period in nanoseconds
 * @return 0 if OK, -ve on error
 */
int pwm_set_config(struct udevice *dev, uint channel, uint period_ns,
		   uint duty_ns);

/**
 * pwm_set_enable() - Enable or disable the PWM
 *
 * @dev:	PWM device to update
 * @channel:	PWM channel to update
 * @enable:	true to enable, false to disable
 * @return 0 if OK, -ve on error
 */
int pwm_set_enable(struct udevice *dev, uint channel, bool enable);

/**
 * pwm_set_invert() - Set pwm default polarity
 *
 * @dev:	PWM device to update
 * @channel:	PWM channel to update
 * @polarity:	true to invert, false to keep normal polarity
 * @return 0 if OK, -ve on error
 */
int pwm_set_invert(struct udevice *dev, uint channel, bool polarity);
#ifdef CONFIG_PWM_MESON
/**
 * pwm_set_times() - Set the PWM times
 *
 * @dev:		PWM device to update
 * @channel:	PWM channel to update
 * @times:		dual channel to set times
 * @return 0 if OK, -ve on error
 */
int pwm_set_times(struct udevice *dev, uint channel, uint times);

/**
 * pwm_set_blink_times() - Set the PWM blink times
 *
 * @dev:		PWM device to update
 * @channel:	PWM channel to update
 * @times:		set blink times
 * @return 0 if OK, -ve on error
 */
int pwm_set_blink_times(struct udevice *dev, uint channel, uint times);

/**
 * pwm_set_blink_enable() - Enable or disable the PWM blink
 *
 * @dev:		PWM device to update
 * @channel:	PWM channel to update
 * @enable: 	true to enable, false to disable
 * @return 0 if OK, -ve on error
 */
int pwm_set_blink_enable(struct udevice *dev, uint channel, bool enable);
#endif
/* Legacy interface */
#ifndef CONFIG_DM_PWM
int	pwm_init		(int pwm_id, int div, int invert);
int	pwm_config		(int pwm_id, int duty_ns, int period_ns);
int	pwm_enable		(int pwm_id);
void	pwm_disable		(int pwm_id);
#endif

#endif /* _pwm_h_ */
