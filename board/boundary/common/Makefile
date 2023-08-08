# SPDX-License-Identifier:	GPL-2.0+
#
ifdef CONFIG_DEFCONFIG
obj-y  := bd_common.o eth.o
ifneq ($(CONFIG_ARCH_IMX8M)$(CONFIG_ARCH_IMX8ULP)$(CONFIG_ARCH_MEDIATEK),y)
obj-$(CONFIG_CMD_KBD)  += cmd_kbd.o
endif
endif
ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifeq ($(CONFIG_DDR_CHANNEL_CNT),1)
obj-$(CONFIG_IMX8MM) += lpddr4_timing_8mm_ch1.o
ifeq ($(CONFIG_DDR_CHANNEL_CNT_1_OR_2),y)
obj-$(CONFIG_IMX8MM) += lpddr4_timing_8mm.o
endif
else
ifeq ($(CONFIG_LPDDR4_AB),y)
obj-$(CONFIG_IMX8MM) += lpddr4_timing_8mm_ab.o
else
obj-$(CONFIG_IMX8MM) += lpddr4_timing_8mm.o
ifeq ($(CONFIG_DDR_CHANNEL_CNT_1_OR_2),y)
obj-$(CONFIG_IMX8MM) += lpddr4_timing_8mm_ch1.o
endif
endif
endif
obj-$(CONFIG_IMX8MN) += lpddr4_timing_8mn.o
obj-$(CONFIG_IMX8MP) += lpddr4_timing_8mp.o
obj-$(CONFIG_IMX8MQ) += lpddr4_timing_8mq.o
obj-$(CONFIG_IMX8ULP) += lpddr4_timing_8ulp.o
endif
obj-$(CONFIG_CMD_FBPANEL) += display.o
obj-$(CONFIG_BQ25898) += bq25898.o
obj-$(CONFIG_MAX77823)  += max77823.o
obj-$(CONFIG_MAX77975)  += max77975.o
obj-$(CONFIG_FAN53526)  += fan53526.o
obj-$(CONFIG_CMD_OTG_CABLE)  += cmd_otg_cable.o
