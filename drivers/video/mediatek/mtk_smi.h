/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#ifndef _MTK_SMI_H
#define _MTK_SMI_H

int mtk_smi_larb_enable(struct udevice *dev);
int mtk_smi_larb_disable(struct udevice *dev);

#endif
