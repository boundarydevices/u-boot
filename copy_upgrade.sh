#!/bin/sh

if [ $# -lt 1 ]; then
	echo "Error, missing a parameter:"
	echo "$0 <mount_path>"
	exit 1
fi

grep ^CONFIG_ARM64 .config > /dev/null
if [ $? -eq 0 ]; then
	ARCH=arm64
else
	ARCH=arm
fi

uboot_defconfig=`grep CONFIG_DEFCONFIG include/config.h|sed -e 's/#define CONFIG_DEFCONFIG[^"]\{1,\}"\([^"]\{1,\}\)"/\1/'`

./tools/mkimage -A $ARCH -O linux -T script -C none \
	-a 0 -e 0 -n "update script" \
	-d board/boundary/bootscripts/upgrade.txt upgrade.scr

if [ -f u-boot.imx ]; then
	cp u-boot.imx u-boot.$uboot_defconfig
fi

if [ -f flash.bin ]; then
	cp flash.bin u-boot.$uboot_defconfig
fi

if [ ! -f u-boot.$uboot_defconfig ]; then
	echo "Couldn't find u-boot.$uboot_defconfig!"
	exit 1
fi

mv -v -t $1/ u-boot.$uboot_defconfig upgrade.scr
