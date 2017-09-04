#!/bin/sh

if [ $# -lt 1 ]; then
	echo "Error, missing a parameter:"
	echo "$0 <mount_path>"
	exit 1
fi

./tools/mkimage -A arm -O linux -T script -C none \
	-a 0 -e 0 -n "update script" \
	-d board/boundary/nitrogen6x/6x_upgrade.txt 6x_upgrade

./tools/mkimage -A arm -O linux -T script -C none \
	-a 0 -e 0 -n "update script" \
	-d board/boundary/bootscripts/upgrade.txt upgrade.scr

uboot_defconfig=`grep CONFIG_DEFCONFIG include/config.h|sed -e 's/#define CONFIG_DEFCONFIG[^"]\{1,\}"\([^"]\{1,\}\)"/\1/'`

cp u-boot.imx u-boot.$uboot_defconfig
mv -v -t $1/ u-boot.$uboot_defconfig 6x_upgrade upgrade.scr
