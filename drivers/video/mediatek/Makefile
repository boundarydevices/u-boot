# SPDX-License-Identifier: GPL-2.0
#
# Copyright (c) 2023 MediaTek Inc.
#
# Author: Tommy Chen <tommyyl.chen@mediatek.com>

ifeq ($(CONFIG_VIDEO_MEDIATEK),y)
	ifeq ($(CONFIG_TARGET_MT8365),y)
		obj-y += \
			mtk_ddp.o \
			mtk_dpi.o \
			mtk_disp_mutex.o \
			mtk_disp_rdma.o \
			mtk_smi.o
	else
		obj-y += mtk_dsi_common.o
		obj-$(CONFIG_TARGET_MT8188) += \
			dsi_mt8188.o \
			vdosys_mt8188.o \
			disp_pwm_mt8188.o
	endif
endif
