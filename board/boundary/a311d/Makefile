# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2020 Boundary Devices

obj-y	:= a311d.o 
dtb-y += a311d.dtb

# Add any required device tree compiler flags here
DTC_FLAGS += -a 0x8

PHONY += dtbs
dtbs: $(addprefix $(obj)/, $(dtb-y))
	@:

clean-files := $(dtb-y)
