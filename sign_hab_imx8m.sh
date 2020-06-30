#!/bin/bash
set -e

for i in flash.bin flash.log .config; do
	if [ ! -f $i ]; then
		echo "Missing $i! Please make sure to 'make flash.bin' first"
		exit 1
	fi
done

if [ -z "$CST_BIN" ] || [ ! -f $CST_BIN ]; then
	echo "Missing CST_BIN variable!"
	exit 1
fi

if [ -z "$SIGN_KEY" ] || [ ! -f $SIGN_KEY ]; then
	echo "Missing SIGN_KEY variable!"
	exit 1
fi

if [ -z "$IMG_KEY" ] || [ ! -f $IMG_KEY ]; then
	echo "Missing IMG_KEY variable!"
	exit 1
fi

if [ -z "$SRK_TABLE" ] || [ ! -f $SRK_TABLE ]; then
	echo "Missing SRK_TABLE variable!"
	exit 1
fi

# source config file for convenience
source .config

# get FIT HAB signing offsets
for i in $CONFIG_OF_LIST; do
	DTBS="arch/arm/dts/$i.dtb $DTBS"
done
./print_fit_hab.sh $DTBS > fit.log

# copy templates and update values
cp doc/imx/habv4/csf_examples/mx8m_mx8mm/template_csf_spl.txt csf_spl.txt
cp doc/imx/habv4/csf_examples/mx8m_mx8mm/template_csf_fit.txt csf_fit.txt
sed -i "s|_SIGN_KEY_|$SIGN_KEY|g" csf_*.txt
sed -i "s|_IMG_KEY_|$IMG_KEY|g" csf_*.txt
sed -i "s|_SRK_TABLE_|$SRK_TABLE|g" csf_*.txt

# update SPL values
SPL_START_ADDR=`awk '/spl hab block/{print $4}' flash.log`
SPL_OFFSET=`awk '/spl hab block/{print $5}' flash.log`
SPL_LENGTH=`awk '/spl hab block/{print $6}' flash.log`
sed -i "s|_SPL_START_ADDR_|$SPL_START_ADDR|g" csf_spl.txt
sed -i "s|_SPL_OFFSET_|$SPL_OFFSET|g" csf_spl.txt
sed -i "s|_SPL_LENGTH_|$SPL_LENGTH|g" csf_spl.txt

# update FIT values
FIT_START_ADDR=`awk '/sld hab block/{print $4}' flash.log`
FIT_OFFSET=`awk '/sld hab block/{print $5}' flash.log`
FIT_LENGTH=`awk '/sld hab block/{print $6}' flash.log`
sed -i "s|_FIT_START_ADDR_|$FIT_START_ADDR|g" csf_fit.txt
sed -i "s|_FIT_OFFSET_|$FIT_OFFSET|g" csf_fit.txt
sed -i "s|_FIT_LENGTH_|$FIT_LENGTH|g" csf_fit.txt
UBOOT_START_ADDR=`awk 'NR==1{print $1}' fit.log`
UBOOT_OFFSET=`awk 'NR==1{print $2}' fit.log`
UBOOT_LENGTH=`awk 'NR==1{print $3}' fit.log`
sed -i "s|_UBOOT_START_ADDR_|$UBOOT_START_ADDR|g" csf_fit.txt
sed -i "s|_UBOOT_OFFSET_|$UBOOT_OFFSET|g" csf_fit.txt
sed -i "s|_UBOOT_LENGTH_|$UBOOT_LENGTH|g" csf_fit.txt
DTB_START_ADDR=`awk 'NR==2{print $1}' fit.log`
DTB_OFFSET=`awk 'NR==2{print $2}' fit.log`
DTB_LENGTH=`awk 'NR==2{print $3}' fit.log`
sed -i "s|_DTB_START_ADDR_|$DTB_START_ADDR|g" csf_fit.txt
sed -i "s|_DTB_OFFSET_|$DTB_OFFSET|g" csf_fit.txt
sed -i "s|_DTB_LENGTH_|$DTB_LENGTH|g" csf_fit.txt
ATF_START_ADDR=`awk 'NR==3{print $1}' fit.log`
ATF_OFFSET=`awk 'NR==3{print $2}' fit.log`
ATF_LENGTH=`awk 'NR==3{print $3}' fit.log`
sed -i "s|_ATF_START_ADDR_|$ATF_START_ADDR|g" csf_fit.txt
sed -i "s|_ATF_OFFSET_|$ATF_OFFSET|g" csf_fit.txt
sed -i "s|_ATF_LENGTH_|$ATF_LENGTH|g" csf_fit.txt

# generate signatures
$CST_BIN -i csf_spl.txt -o csf_spl.bin
$CST_BIN -i csf_fit.txt -o csf_fit.bin

# copy signatures into binary
CSF_SPL_OFFSET=`awk '/csf_off/{print $2}' flash.log | head -n 1`
CSF_FIT_OFFSET=`awk '/csf_off/{print $2}' flash.log | tail -n 1`
cp flash.bin signed_flash.bin
dd if=csf_spl.bin of=signed_flash.bin seek=$(($CSF_SPL_OFFSET)) bs=1 conv=notrunc
dd if=csf_fit.bin of=signed_flash.bin seek=$(($CSF_FIT_OFFSET)) bs=1 conv=notrunc

echo "signed_flash.bin is ready!"
