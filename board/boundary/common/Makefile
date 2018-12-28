# SPDX-License-Identifier:	GPL-2.0+
#
obj-y  := eth.o
ifneq ($(CONFIG_ARCH_IMX8M),y)
obj-y  += bd_common.o
obj-$(CONFIG_CMD_KBD)  += cmd_kbd.o
endif
obj-$(CONFIG_MAX77823)  += max77823.o
obj-$(CONFIG_FAN53526)  += fan53526.o
obj-$(CONFIG_CMD_OTG_CABLE)  += cmd_otg_cable.o
