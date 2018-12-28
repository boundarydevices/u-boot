/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifdef CONFIG_MX6SX
#include "padctrl-mx6sx.h"
#elif defined(CONFIG_MX7D)
#include "padctrl-mx7d.h"
#elif defined(CONFIG_MX51)
#include "padctrl-mx51.h"
#elif defined(CONFIG_IMX8M)
#include "padctrl-mx8m.h"
#else
#include "padctrl-mx6.h"
#endif
