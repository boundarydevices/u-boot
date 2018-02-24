savedir=../u-boot-images
mkdir -p $savedir
rm $savedir/*
boards=$(cd configs && grep -l DEFCONFIG * | sed 's/_defconfig.*$//');
cpus=`grep -c '^processor' /proc/cpuinfo` ;
jobs=`expr $cpus + 2` ;
numboards=0;
numsuccess=0;
numfailures=0;
dd if=/dev/zero of=test.bin bs=4096 count=1
for board in $boards ; do
    if [ $board != "mx6qsabrelite" ] ; then
	make distclean ; make ${board}_defconfig
	cfgfile=$(sed -n -e's/.\{1,\}IMX_CONFIG=\([^,]\{1,\}\),.*$/\1/p' configs/${board}_defconfig)
	make include/autoconf.mk

	arm-linux-gnueabihf-gcc -E  \
	 -nostdinc -isystem /usr/lib/gcc-cross/arm-linux-gnueabihf/5/include -Iinclude   \
	 -I./arch/arm/include -include ./include/linux/kconfig.h -D__KERNEL__ -D__UBOOT__    \
	 -D__ARM__ -marm -mno-thumb-interwork  -mabi=aapcs-linux  -mword-relocations  -fno-pic  \
	 -mno-unaligned-access  -ffunction-sections -fdata-sections -fno-common -ffixed-r9  \
	 -msoft-float   -pipe  -march=armv7-a -D__LINUX_ARM_ARCH__=7    \
	 -x c -o test.cfgtmp $cfgfile

	mkimage -n test.cfgtmp -T imximage -e 0x17800000 -d test.bin test.imx
	od -Ax -tx4 --endian=big test.imx >$savedir/${board}.txt

	if [ -e  $savedir/${board}.txt ] ; then
		numsuccess=`expr $numsuccess + 1`;
	fi
	numboards=`expr $numboards + 1`;
    fi
done
make distclean ;
echo -e "\n\n\nbuilt for ${numboards} boards. ${numsuccess} succeeded and ${numfailures} failed";
