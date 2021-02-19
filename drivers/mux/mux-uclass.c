// SPDX-License-Identifier: GPL-2.0
/*
 * Multiplexer subsystem
 *
 * Based on the linux multiplexer framework
 *
 * Copyright (C) 2017 Axentia Technologies AB
 * Author: Peter Rosin <peda@axentia.se>
 *
 * Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
 * Jean-Jacques Hiblot <jjhiblot@ti.com>
 */

#define LOG_CATEGORY UCLASS_MUX

#include <common.h>
#include <dm.h>
#include <mux-internal.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <dt-bindings/mux/mux.h>
#include <linux/bug.h>
#include <errno.h>

#include <linux/err.h>
#include <linux/mux/driver.h>
#include <mux.h>

/*
 * The idle-as-is "state" is not an actual state that may be selected, it
 * only implies that the state should not be changed. So, use that state
 * as indication that the cached state of the multiplexer is unknown.
 */
#define MUX_CACHE_UNKNOWN MUX_IDLE_AS_IS

/**
 * mux_control_ops() - Get the mux_control ops.
 * @dev: The client device.
 *
 * Return: A pointer to the 'mux_control_ops' of the device.
 */
static inline const struct mux_control_ops *mux_dev_ops(struct udevice *dev)
{
	return (const struct mux_control_ops *)dev->driver->ops;
}

/**
 * mux_control_set() - Set the state of the given mux controller.
 * @mux: A multiplexer control
 * @state: The new requested state.
 *
 * Return: 0 if OK, or a negative error code.
 */
static int mux_control_set(struct mux_control *mux, int state)
{
	int ret = mux_dev_ops(mux->dev)->set(mux, state);

	mux->cached_state = ret < 0 ? MUX_CACHE_UNKNOWN : state;

	return ret;
}

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
	int i;

	mux = kzalloc(controllers * sizeof(*mux_chip->mux), GFP_KERNEL);
	if (!mux_chip)
		return ERR_PTR(-ENOMEM);

	mux_chip->mux = mux;
	mux_chip->controllers = controllers;

	for (i = 0; i < controllers; ++i) {
		struct mux_control *mc = &mux[i];

		mc->cached_state = MUX_CACHE_UNKNOWN;
		mc->idle_state = MUX_IDLE_AS_IS;
	}

	return mux_chip;
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
			dev_err(mux->dev, "unable to set idle state\n");
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

unsigned int mux_control_states(struct mux_control *mux)
{
	return mux->states;
}

/**
 * __mux_control_select() - Select the given multiplexer state.
 * @mux: The mux-control to request a change of state from.
 * @state: The new requested state.
 *
 * Try to set the mux to the requested state. If not, try to revert if
 * appropriate.
 */
static int __mux_control_select(struct mux_control *mux, int state)
{
	int ret;

	if (WARN_ON(state < 0 || state >= mux->states))
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

	if (mux->in_use)
		return -EBUSY;

	ret = __mux_control_select(mux, state);

	if (ret < 0)
		return ret;

	mux->in_use = true;

	return 0;
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

	mux->in_use = false;

	return ret;
}

static int mux_of_xlate_default(struct mux_chip *mux_chip,
				struct ofnode_phandle_args *args,
				struct mux_control **muxp)
{
	struct mux_control *mux;
	int id;

	log_debug("%s(muxp=%p)\n", __func__, muxp);

	if (args->args_count > 1) {
		debug("Invalid args_count: %d\n", args->args_count);
		return -EINVAL;
	}

	if (args->args_count)
		id = args->args[0];
	else
		id = 0;

	if (id >= mux_chip->controllers) {
		pr_err("bad mux controller %u specified in %s\n",
			id, ofnode_get_name(args->node));
		return -ERANGE;
	}

	mux = &mux_chip->mux[id];
	mux->id = id;
	*muxp = mux;
	return 0;
}

/**
 * mux_get_by_indexed_prop() - Get a mux control by integer index
 * @dev: The client device.
 * @prop_name: Name of the device tree property.
 * @index: The index of the mux to get
 * @mux: A pointer to the 'mux_control' struct to initialize.
 *
 * Return: 0 of OK, -errno otherwise.
 */
static int mux_get_by_indexed_prop(struct udevice *dev, const char *prop_name,
				   int index, struct mux_control **mux)
{
	int ret;
	struct ofnode_phandle_args args;
	struct udevice *dev_mux;
	const struct mux_control_ops *ops;
	struct mux_chip *mux_chip;

	log_debug("%s(dev=%p, index=%d, mux=%p)\n", __func__, dev, index, mux);

	ret = dev_read_phandle_with_args(dev, prop_name, "#mux-control-cells",
					 0, index, &args);
	if (ret) {
		debug("%s: fdtdec_parse_phandle_with_args failed: err=%d\n",
		      __func__, ret);
		return ret;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_MUX, args.node, &dev_mux);
	if (ret) {
		debug("%s: uclass_get_device_by_ofnode failed: err=%d\n",
		      __func__, ret);
		return ret;
	}

	mux_chip = dev_get_uclass_priv(dev_mux);

	ops = mux_dev_ops(dev_mux);
	if (ops->of_xlate)
		ret = ops->of_xlate(mux_chip, &args, mux);
	else
		ret = mux_of_xlate_default(mux_chip, &args, mux);
	if (ret) {
		debug("of_xlate() failed: %d\n", ret);
		return ret;
	}
	(*mux)->dev = dev_mux;

	return 0;
}

int mux_get_by_index(struct udevice *dev, int index, struct mux_control **mux)
{
	return mux_get_by_indexed_prop(dev, "mux-controls", index, mux);
}

int mux_control_get(struct udevice *dev, const char *name,
		    struct mux_control **mux)
{
	int index;

	debug("%s(dev=%p, name=%s, mux=%p)\n", __func__, dev, name, mux);

	index = dev_read_stringlist_search(dev, "mux-control-names", name);
	if (index < 0) {
		debug("fdt_stringlist_search() failed: %d\n", index);
		return index;
	}

	return mux_get_by_index(dev, index, mux);
}

void mux_control_put(struct mux_control *mux)
{
	mux_control_deselect(mux);
}

/**
 * devm_mux_control_release() - Release the given managed mux.
 * @dev: The client device.
 * @res: Pointer to the mux to be released.
 *
 * This function is called by devres to release the mux. It reverses the
 * effects of mux_control_get().
 */
static void devm_mux_control_release(struct udevice *dev, void *res)
{
	mux_control_put(*(struct mux_control **)res);
}

struct mux_control *devm_mux_control_get(struct udevice *dev, const char *id)
{
	int rc;
	struct mux_control **mux;

	mux = devres_alloc(devm_mux_control_release,
			   sizeof(struct mux_control *), __GFP_ZERO);
	if (unlikely(!mux))
		return ERR_PTR(-ENOMEM);

	rc = mux_control_get(dev, id, mux);
	if (rc)
		return ERR_PTR(rc);

	devres_add(dev, mux);
	return *mux;
}

int mux_alloc_controllers(struct udevice *dev, unsigned int controllers)
{
	int i;
	struct mux_chip *mux_chip = dev_get_uclass_priv(dev);

	mux_chip->mux = devm_kmalloc(dev,
				     sizeof(struct mux_control) * controllers,
				     __GFP_ZERO);
	if (!mux_chip->mux)
		return -ENOMEM;

	mux_chip->controllers = controllers;

	for (i = 0; i < mux_chip->controllers; ++i) {
		struct mux_control *mux = &mux_chip->mux[i];

		mux->dev = dev;
		mux->cached_state = MUX_CACHE_UNKNOWN;
		mux->idle_state = MUX_IDLE_AS_IS;
		mux->in_use = false;
		mux->id = i;
	}

	return 0;
}

static int mux_uclass_post_probe(struct udevice *dev)
{
	int i, ret;
	struct mux_chip *mux_chip = dev_get_uclass_priv(dev);

	/* Set all mux controllers to their idle state. */
	for (i = 0; i < mux_chip->controllers; ++i) {
		struct mux_control *mux = &mux_chip->mux[i];

		if (mux->idle_state == mux->cached_state)
			continue;

		ret = mux_control_set(mux, mux->idle_state);
		if (ret < 0) {
			dev_err(dev, "unable to set idle state\n");
			return ret;
		}
	}
	return 0;
}

int dm_mux_init(void)
{
	struct uclass *uc;
	struct udevice *dev;
	int ret;

	ret = uclass_get(UCLASS_MUX, &uc);
	if (ret < 0) {
		log_debug("unable to get MUX uclass\n");
		return ret;
	}
	uclass_foreach_dev(dev, uc) {
		if (dev_read_bool(dev, "u-boot,mux-autoprobe")) {
			ret = device_probe(dev);
			if (ret)
				log_debug("unable to probe device %s\n",
					  dev->name);
		}
	}

	return 0;
}

UCLASS_DRIVER(mux) = {
	.id		= UCLASS_MUX,
	.name		= "mux",
	.post_probe	= mux_uclass_post_probe,
	.per_device_auto	= sizeof(struct mux_chip),
};
