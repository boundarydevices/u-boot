// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>
#include <pwm.h>
#include <asm/errno.h>

int pwm_set_invert(struct udevice *dev, uint channel, bool polarity)
{
	struct pwm_ops *ops = pwm_get_ops(dev);

	if (!ops->set_invert)
		return -ENOSYS;

	return ops->set_invert(dev, channel, polarity);
}

int pwm_set_config(struct udevice *dev, uint channel, uint period_ns,
		   uint duty_ns)
{
	struct pwm_ops *ops = pwm_get_ops(dev);

	if (!ops->set_config)
		return -ENOSYS;

	return ops->set_config(dev, channel, period_ns, duty_ns);
}

int pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct pwm_ops *ops = pwm_get_ops(dev);

	if (!ops->set_enable)
		return -ENOSYS;

	return ops->set_enable(dev, channel, enable);
}
#ifdef CONFIG_PWM_MESON
int pwm_set_times(struct udevice *dev, uint channel, uint times)
{
	struct pwm_ops *ops = pwm_get_ops(dev);

	if (!ops->set_enable)
		return -ENOSYS;

	return ops->set_times(dev, channel, times);
}

int pwm_set_blink_times(struct udevice *dev, uint channel, uint times)
{
	struct pwm_ops *ops = pwm_get_ops(dev);

	if (!ops->set_enable)
		return -ENOSYS;

	return ops->set_blink_times(dev, channel, times);
}

int pwm_set_blink_enable(struct udevice *dev, uint channel, bool enable)
{
	struct pwm_ops *ops = pwm_get_ops(dev);

	if (!ops->set_enable)
		return -ENOSYS;

	return ops->set_blink_enable(dev, channel, enable);
}
#endif
UCLASS_DRIVER(pwm) = {
	.id		= UCLASS_PWM,
	.name		= "pwm",
};
