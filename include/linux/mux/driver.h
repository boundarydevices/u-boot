/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mux/driver.h - definitions for the multiplexer driver interface
 *
 * Copyright (C) 2017 Axentia Technologies AB
 *
 * Author: Peter Rosin <peda@axentia.se>
 */

#ifndef _LINUX_MUX_DRIVER_H
#define _LINUX_MUX_DRIVER_H

#include <dt-bindings/mux/mux.h>

struct mux_chip;
struct mux_control;

/**
 * struct mux_control_ops -	Mux controller operations for a mux chip.
 * @set:			Set the state of the given mux controller.
 */
struct mux_control_ops {
	int (*set)(struct mux_control *mux, int state);
};

/**
 * struct mux_control -	Represents a mux controller.
 * @lock:		Protects the mux controller state.
 * @chip:		The mux chip that is handling this mux controller.
 * @cached_state:	The current mux controller state, or -1 if none.
 * @states:		The number of mux controller states.
 * @idle_state:		The mux controller state to use when inactive, or one
 *			of MUX_IDLE_AS_IS and MUX_IDLE_DISCONNECT.
 *
 * Mux drivers may only change @states and @idle_state, and may only do so
 * between allocation and registration of the mux controller. Specifically,
 * @cached_state is internal to the mux core and should never be written by
 * mux drivers.
 */
struct mux_control {
	struct mux_chip *chip;
	int cached_state;

	unsigned int states;
	int idle_state;
};

/**
 * struct mux_chip -	Represents a chip holding mux controllers.
 * @controllers:	Number of mux controllers handled by the chip.
 * @mux:		Array of mux controllers that are handled.
 * @dev:		Device structure.
 * @id:			Used to identify the device internally.
 * @ops:		Mux controller operations.
 */
struct mux_chip {
	unsigned int controllers;
	struct mux_control *mux;
	struct udevice *dev;
	int id;
};

#define mux_get_ops(dev) ((struct mux_control_ops *)(dev)->driver->ops)

struct mux_chip *mux_chip_alloc(struct udevice *dev,
				unsigned int controllers);
int mux_chip_register(struct mux_chip *mux_chip);
void mux_chip_unregister(struct mux_chip *mux_chip);
void mux_chip_free(struct mux_chip *mux_chip);

struct mux_chip *devm_mux_chip_alloc(struct udevice *dev,
				     unsigned int controllers);
int devm_mux_chip_register(struct udevice *dev, struct mux_chip *mux_chip);

/**
 * mux_control_get_index() - Get the index of the given mux controller
 * @mux: The mux-control to get the index for.
 *
 * Return: The index of the mux controller within the mux chip the mux
 * controller is a part of.
 */
static inline unsigned int mux_control_get_index(struct mux_control *mux)
{
	return mux - mux->chip->mux;
}

#endif /* _LINUX_MUX_DRIVER_H */
