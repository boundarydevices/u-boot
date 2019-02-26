/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifdef CONFIG_MX6SX
#include "padctrl-mx6sx.h"
#elif defined(CONFIG_MX6ULL)
#include "padctrl-mx6ull.h"
#elif defined(CONFIG_MX7D)
#include "padctrl-mx7d.h"
#elif defined(CONFIG_MX51)
#include "padctrl-mx51.h"
#elif defined(CONFIG_IMX8MM)
#include "padctrl-imx8mm.h"
#elif defined(CONFIG_IMX8MQ)
#include "padctrl-imx8mq.h"
#else
#include "padctrl-mx6.h"
#endif
