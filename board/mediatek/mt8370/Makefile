# SPDX-License-Identifier:	GPL-2.0

obj-y += mt8370_evk.o

ifeq ($(CONFIG_VIDEO_MEDIATEK),y)
	obj-y += mt8370_evk_panel.o
	ccflags-y += -I$(srctree)/drivers/video/mediatek/
endif
