#!/bin/bash
set -e

if [ $# -lt 1 ]; then
	echo "Usage: $0 <path to Image> [loadaddr (optional)]"
	exit 1
fi

if [ ! -f $1 ]; then
	echo "$1 doesn't exist!"
	exit 1
fi
IMAGE=$1

if [ $# -lt 2 ]; then
	if [ ! -f u-boot.cfg ]; then
		echo "Missing u-boot.cfg! Please make sure to build U-Boot or provide the loadaddr"
		exit 1
	fi
	# get default LOADADDR
	LOADADDR=`awk '/define CONFIG_LOADADDR/{print $3}' u-boot.cfg`
else
	LOADADDR=$2
fi

if [ -z "$CST_BIN" ] || [ ! -f $CST_BIN ]; then
	echo "Missing CST_BIN variable pointing to cst binary!"
	exit 1
fi

if [ -z "$SIGN_KEY" ] || [ ! -f $SIGN_KEY ]; then
	echo "Missing SIGN_KEY variable pointing to cst binary!"
	exit 1
fi

if [ -z "$IMG_KEY" ] || [ ! -f $IMG_KEY ]; then
	echo "Missing IMG_KEY variable pointing to cst binary!"
	exit 1
fi

if [ -z "$SRK_TABLE" ] || [ ! -f $SRK_TABLE ]; then
	echo "Missing SRK_TABLE variable pointing to cst binary!"
	exit 1
fi

echo Extracting size from Image header...
SIZE="0x`od -t x4 -j 0x10 -N 0x4 --endian=little $IMAGE | head -n1 | awk '{print $2}'`"
IVTOFFSET=$SIZE
IVTSIZE="0x20"
printf -v CSFOFFSET '%#x' "$((IVTOFFSET + IVTSIZE))"

echo Padding Image file...
objcopy -I binary -O binary --pad-to=$SIZE --gap-fill=0x00 $IMAGE Image-pad.bin

echo Generating IVT header...
cp doc/imx/habv4/csf_examples/mx8m_mx8mm/template_genIVT.txt genIVT.pl
printf -v IVTADDRESS '%#x' "$((LOADADDR + IVTOFFSET))"
printf -v CSFADDRESS '%#x' "$((LOADADDR + CSFOFFSET))"
sed -i "s|_LOADADDR_|$LOADADDR|g" genIVT.pl
sed -i "s|_IVTADDRESS_|$IVTADDRESS|g" genIVT.pl
sed -i "s|_CSFADDRESS_|$CSFADDRESS|g" genIVT.pl
chmod +x genIVT.pl
./genIVT.pl
cat Image-pad.bin ivt.bin > Image-pad-ivt.bin

echo Generating CSF binary...
# copy templates and update values
cp doc/imx/habv4/csf_examples/mx8m_mx8mm/template_csf_image.txt csf_image.txt
sed -i "s|_SIGN_KEY_|$SIGN_KEY|g" csf_image.txt
sed -i "s|_IMG_KEY_|$IMG_KEY|g" csf_image.txt
sed -i "s|_SRK_TABLE_|$SRK_TABLE|g" csf_image.txt
# update IMAGE values
IMAGE_START_ADDR=`awk '/sld hab block/{print $4}' flash.log`
IMAGE_OFFSET=`awk '/sld hab block/{print $5}' flash.log`
IMAGE_LENGTH=`awk '/sld hab block/{print $6}' flash.log`
sed -i "s|_IMAGE_START_ADDR_|$LOADADDR|g" csf_image.txt
sed -i "s|_IMAGE_OFFSET_|0x0|g" csf_image.txt
sed -i "s|_IMAGE_LENGTH_|$CSFOFFSET|g" csf_image.txt
# generate signatures
$CST_BIN -i csf_image.txt -o csf_image.bin
# copy signatures into binary
cat Image-pad-ivt.bin csf_image.bin > Image-signed.bin
rm Image-pad*

echo "Image-signed.bin is ready!"
