#!/bin/bash
# SPDX-License-Identifier: GPL-2.0+
#
# script to check whether the file exists in imximage.cfg for i.MX8M
#

file=$1

post_process=$2
dest_file=`awk '/^LOADER/ {print $2}' $file`

blobs=`awk '/^SIGNED_HDMI/ {print $2}' $file`
for f in $blobs; do
	if [ ! -f $f ]; then
		echo "WARNING '$f' not found, resulting binary is not-functional" >&2
		exit 1
	fi
done

blobs=`awk '/^DDR_FW/ {print $2}' $file`
i=0
for f in $blobs; do
	if [ ! -f $f ]; then
		echo "WARNING '$f' not found, resulting binary is not-functional" >&2
		exit 1
	fi
	ddrbin[$i]=$f
	let i=$i+1
done

if [ $i != 4 ] ; then
	echo "wrong number of DDR_FW entries" $i >&2
	exit 1
fi

dd if=spl/u-boot-spl.bin of=spl/u-boot-spl.bin.pad bs=4 conv=sync
objcopy -I binary -O binary --pad-to 0x8000 --gap-fill=0x0 ${ddrbin[0]} ${ddrbin[0]}.pad
objcopy -I binary -O binary --pad-to 0x4000 --gap-fill=0x0 ${ddrbin[1]} ${ddrbin[1]}.pad
objcopy -I binary -O binary --pad-to 0x8000 --gap-fill=0x0 ${ddrbin[2]} ${ddrbin[2]}.pad
echo creating file ${dest_file}
cat spl/u-boot-spl.bin.pad ${ddrbin[0]}.pad ${ddrbin[1]}.pad ${ddrbin[2]}.pad ${ddrbin[3]} > ${dest_file}
rm -f spl/u-boot-spl.bin.pad ${ddrbin[0]}.pad ${ddrbin[1]}.pad ${ddrbin[2]}.pad

exit 0
