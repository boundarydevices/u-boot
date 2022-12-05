#!/bin/bash

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

uboot_defconfig=`grep CONFIG_DEFCONFIG .config|sed -e 's/CONFIG_DEFCONFIG[^"]\{1,\}"\([^"]\{1,\}\)"/\1/'`

if [ "${ARCH}" == "arm64" ] ; then
	uboot=flash.bin
else
	cnt=`sed -n "/CONFIG_OF_CONTROL=/=" .config`
	if [ "${cnt}" != "" ] ; then
		uboot=u-boot-dtb.imx
	else
		uboot=u-boot.imx
	fi
fi

./tools/mkimage -A $ARCH -O linux -T script -C none \
	-a 0 -e 0 -n "update script" \
	-d board/boundary/bootscripts/upgrade.txt upgrade.scr

if [ -f $uboot ]; then
	cp $uboot u-boot.$uboot_defconfig
else
	echo "Couldn't find $uboot!"
fi

if [ ! -f u-boot.$uboot_defconfig ]; then
	echo "Couldn't find u-boot.$uboot_defconfig!"
	exit 1
fi

mv -v -t $1/ u-boot.$uboot_defconfig upgrade.scr
