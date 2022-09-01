/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#ifndef _MTK_DISP_RDMA_H
#define _MTK_DISP_RDMA_H

void mtk_rdma_config(struct udevice *dev, struct display_timing timing);
void mtk_rdma_layer(struct udevice *dev, struct video_uc_plat *plat, struct display_timing timing);
void mtk_rdma_start(struct udevice *dev);
int mtk_rdma_enable(struct udevice *dev);
int mtk_rdma_disable(struct udevice *dev);

#endif
