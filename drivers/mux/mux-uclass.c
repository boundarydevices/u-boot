// SPDX-License-Identifier: GPL-2.0
/*
 * Multiplexer subsystem
 *
 * Copyright (C) 2017 Axentia Technologies AB
 *
 * Author: Peter Rosin <peda@axentia.se>
 */

#define DEBUG
#include <common.h>
#include <dm.h>
#include <dm/devres.h>
#include <errno.h>

#include <linux/err.h>
#include <linux/mux/consumer.h>
#include <linux/mux/driver.h>

/*
 * The idle-as-is "state" is not an actual state that may be selected, it
 * only implies that the state should not be changed. So, use that state
 * as indication that the cached state of the multiplexer is unknown.
 */
#define MUX_CACHE_UNKNOWN MUX_IDLE_AS_IS


/**
 * mux_chip_alloc() - Allocate a mux-chip.
 * @dev: The parent device implementing the mux interface.
 * @controllers: The number of mux controllers to allocate for this chip.
 * @sizeof_priv: Size of extra memory area for private use by the caller.
 *
 * After allocating the mux-chip with the desired number of mux controllers
 * but before registering the chip, the mux driver is required to configure
 * the number of valid mux states in the mux_chip->mux[N].states members and
 * the desired idle state in the returned mux_chip->mux[N].idle_state members.
 * The default idle state is MUX_IDLE_AS_IS. The mux driver also needs to
 * provide a pointer to the operations struct in the mux_chip->ops member
 * before registering the mux-chip with mux_chip_register.
 *
 * Return: A pointer to the new mux-chip, or an ERR_PTR with a negative errno.
 */
struct mux_chip *mux_chip_alloc(struct udevice *dev,
				unsigned int controllers)
{
	struct mux_chip *mux_chip = dev_get_uclass_priv(dev);
	struct mux_control *mux;
	int ret;
	int i;

	ret = dev->seq;
	if (ret) {
		pr_err("muxchipX failed to get a device id\n");
		return ERR_PTR(ret);
	}

	mux = kzalloc(controllers * sizeof(*mux_chip->mux), GFP_KERNEL);
	if (!mux_chip)
		return ERR_PTR(-ENOMEM);

	mux_chip->dev = dev;
	mux_chip->mux = mux;
	mux_chip->controllers = controllers;

	for (i = 0; i < controllers; ++i) {
		struct mux_control *mc = &mux[i];

		mc->chip = mux_chip;
		mc->cached_state = MUX_CACHE_UNKNOWN;
		mc->idle_state = MUX_IDLE_AS_IS;
	}

	return mux_chip;
}

static int mux_control_set(struct mux_control *mux, int state)
{
	struct mux_control_ops *ops = mux_get_ops(mux->chip->dev);
	int ret;

	ret = ops->set(mux, state);

	mux->cached_state = ret < 0 ? MUX_CACHE_UNKNOWN : state;

	return ret;
}

/**
 * mux_chip_register() - Register a mux-chip, thus readying the controllers
 *			 for use.
 * @mux_chip: The mux-chip to register.
 *
 * Do not retry registration of the same mux-chip on failure. You should
 * instead put it away with mux_chip_free() and allocate a new one, if you
 * for some reason would like to retry registration.
 *
 * Return: Zero on success or a negative errno on error.
 */
int mux_chip_register(struct mux_chip *mux_chip)
{
	int i;
	int ret = 0;

	for (i = 0; i < mux_chip->controllers; ++i) {
		struct mux_control *mux = &mux_chip->mux[i];

		if (mux->idle_state == mux->cached_state)
			continue;

		ret = mux_control_set(mux, mux->idle_state);
		if (ret < 0) {
			dev_err(&mux_chip->dev, "unable to set idle state\n");
			return ret;
		}
	}

	return ret;
}

/**
 * mux_chip_free() - Free the mux-chip for good.
 * @mux_chip: The mux-chip to free.
 *
 * mux_chip_free() reverses the effects of mux_chip_alloc().
 */
void mux_chip_free(struct mux_chip *mux_chip)
{
}

static void devm_mux_chip_release(struct udevice *dev, void *res)
{
	struct mux_chip *mux_chip = *(struct mux_chip **)res;

	mux_chip_free(mux_chip);
}

/**
 * devm_mux_chip_alloc() - Resource-managed version of mux_chip_alloc().
 * @dev: The parent device implementing the mux interface.
 * @controllers: The number of mux controllers to allocate for this chip.
 * @sizeof_priv: Size of extra memory area for private use by the caller.
 *
 * See mux_chip_alloc() for more details.
 *
 * Return: A pointer to the new mux-chip, or an ERR_PTR with a negative errno.
 */
struct mux_chip *devm_mux_chip_alloc(struct udevice *dev,
				     unsigned int controllers)
{
	struct mux_chip **ptr, *mux_chip;

	ptr = devres_alloc(devm_mux_chip_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	mux_chip = mux_chip_alloc(dev, controllers);
	if (IS_ERR(mux_chip)) {
		devres_free(ptr);
		return mux_chip;
	}

	*ptr = mux_chip;
	devres_add(dev, ptr);

	return mux_chip;
}

static void devm_mux_chip_reg_release(struct udevice *dev, void *res)
{
}

/**
 * devm_mux_chip_register() - Resource-managed version mux_chip_register().
 * @dev: The parent device implementing the mux interface.
 * @mux_chip: The mux-chip to register.
 *
 * See mux_chip_register() for more details.
 *
 * Return: Zero on success or a negative errno on error.
 */
int devm_mux_chip_register(struct udevice *dev,
			   struct mux_chip *mux_chip)
{
	struct mux_chip **ptr;
	int res;

	ptr = devres_alloc(devm_mux_chip_reg_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	res = mux_chip_register(mux_chip);
	if (res) {
		devres_free(ptr);
		return res;
	}

	*ptr = mux_chip;
	devres_add(dev, ptr);

	return res;
}

/**
 * mux_control_states() - Query the number of multiplexer states.
 * @mux: The mux-control to query.
 *
 * Return: The number of multiplexer states.
 */
unsigned int mux_control_states(struct mux_control *mux)
{
	return mux->states;
}

static int __mux_control_select(struct mux_control *mux, int state)
{
	int ret;

	if (state < 0 || state >= mux->states)
		return -EINVAL;

	if (mux->cached_state == state)
		return 0;

	ret = mux_control_set(mux, state);
	if (ret >= 0)
		return 0;

	/* The mux update failed, try to revert if appropriate... */
	if (mux->idle_state != MUX_IDLE_AS_IS)
		mux_control_set(mux, mux->idle_state);

	return ret;
}

/**
 * mux_control_select() - Select the given multiplexer state.
 * @mux: The mux-control to request a change of state from.
 * @state: The new requested state.
 *
 * On successfully selecting the mux-control state, it will be locked until
 * there is a call to mux_control_deselect(). If the mux-control is already
 * selected when mux_control_select() is called, the caller will be blocked
 * until mux_control_deselect() is called (by someone else).
 *
 * Therefore, make sure to call mux_control_deselect() when the operation is
 * complete and the mux-control is free for others to use, but do not call
 * mux_control_deselect() if mux_control_select() fails.
 *
 * Return: 0 when the mux-control state has the requested state or a negative
 * errno on error.
 */
int mux_control_select(struct mux_control *mux, unsigned int state)
{
	int ret;

	ret = __mux_control_select(mux, state);

	return ret;
}

/**
 * mux_control_try_select() - Try to select the given multiplexer state.
 * @mux: The mux-control to request a change of state from.
 * @state: The new requested state.
 *
 * On successfully selecting the mux-control state, it will be locked until
 * mux_control_deselect() called.
 *
 * Therefore, make sure to call mux_control_deselect() when the operation is
 * complete and the mux-control is free for others to use, but do not call
 * mux_control_deselect() if mux_control_try_select() fails.
 *
 * Return: 0 when the mux-control state has the requested state or a negative
 * errno on error. Specifically -EBUSY if the mux-control is contended.
 */
int mux_control_try_select(struct mux_control *mux, unsigned int state)
{
	int ret;

	ret = __mux_control_select(mux, state);

	return ret;
}

/**
 * mux_control_deselect() - Deselect the previously selected multiplexer state.
 * @mux: The mux-control to deselect.
 *
 * It is required that a single call is made to mux_control_deselect() for
 * each and every successful call made to either of mux_control_select() or
 * mux_control_try_select().
 *
 * Return: 0 on success and a negative errno on error. An error can only
 * occur if the mux has an idle state. Note that even if an error occurs, the
 * mux-control is unlocked and is thus free for the next access.
 */
int mux_control_deselect(struct mux_control *mux)
{
	int ret = 0;

	if (mux->idle_state != MUX_IDLE_AS_IS &&
	    mux->idle_state != mux->cached_state)
		ret = mux_control_set(mux, mux->idle_state);

	return ret;
}

/* Note this function returns a reference to the mux_chip dev. */
static struct mux_chip *of_find_mux_chip_by_node(ofnode np)
{
	struct mux_chip *mc = NULL;
	struct udevice *dev = NULL;
	int ret;

	ret = uclass_get_device_by_ofnode(UCLASS_MUX, np,  &dev);
	if (!ret)
		mc = dev_get_uclass_priv(dev);

	return mc;
}

/**
 * mux_control_get() - Get the mux-control for a device.
 * @dev: The device that needs a mux-control.
 * @mux_name: The name identifying the mux-control.
 *
 * Return: A pointer to the mux-control, or an ERR_PTR with a negative errno.
 */
struct mux_control *mux_control_get(struct udevice *dev, const char *mux_name)
{
	ofnode np = dev_ofnode(dev);
	struct ofnode_phandle_args args;
	struct mux_chip *mux_chip;
	unsigned int controller;
	int index = 0;
	int ret;

	if (mux_name) {
		index = ofnode_stringlist_search(np, "mux-control-names",
						 mux_name);
		if (index < 0) {
			debug("mux controller '%s' not found\n",
				mux_name);
			return ERR_PTR(index);
		}
	}

	ret = ofnode_parse_phandle_with_args(np, "mux-controls",
			"#mux-control-cells", 0, index, &args);
	if (ret) {
		debug("%s: failed to get mux-control %s(%i)\n",
			ofnode_get_name(np), mux_name ?: "", index);
		return ERR_PTR(ret);
	}

	mux_chip = of_find_mux_chip_by_node(args.node);
	if (!mux_chip)
		return ERR_PTR(-EPROBE_DEFER);

	if (args.args_count > 1 ||
	    (!args.args_count && (mux_chip->controllers > 1))) {
		debug("%s: wrong #mux-control-cells for %s\n",
			ofnode_get_name(np), ofnode_get_name(args.node));
		return ERR_PTR(-EINVAL);
	}

	controller = 0;
	if (args.args_count)
		controller = args.args[0];

	if (controller >= mux_chip->controllers) {
		debug("%s: bad mux controller %u specified in %s\n",
			ofnode_get_name(np), controller, ofnode_get_name(args.node));
		return ERR_PTR(-EINVAL);
	}

	return &mux_chip->mux[controller];
}

/**
 * mux_control_put() - Put away the mux-control for good.
 * @mux: The mux-control to put away.
 *
 * mux_control_put() reverses the effects of mux_control_get().
 */
void mux_control_put(struct mux_control *mux)
{
}

static void devm_mux_control_release(struct udevice *dev, void *res)
{
	struct mux_control *mux = *(struct mux_control **)res;

	mux_control_put(mux);
}

/**
 * devm_mux_control_get() - Get the mux-control for a device, with resource
 *			    management.
 * @dev: The device that needs a mux-control.
 * @mux_name: The name identifying the mux-control.
 *
 * Return: Pointer to the mux-control, or an ERR_PTR with a negative errno.
 */
struct mux_control *devm_mux_control_get(struct udevice *dev,
					 const char *mux_name)
{
	struct mux_control **ptr, *mux;

	ptr = devres_alloc(devm_mux_control_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	mux = mux_control_get(dev, mux_name);
	if (IS_ERR(mux)) {
		devres_free(ptr);
		return mux;
	}

	*ptr = mux;
	devres_add(dev, ptr);

	return mux;
}

UCLASS_DRIVER(mux) = {
	.id		= UCLASS_MUX,
	.name		= "mux",
	.per_device_auto_alloc_size	= sizeof(struct mux_chip),
};
