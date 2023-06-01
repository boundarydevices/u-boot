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

# retrieve the current script path in case it is executed from another location
SCRIPT_PATH=$(dirname $0)

# source config file for convenience
source .config

# copy templates and update values
cp $SCRIPT_PATH/doc/imx/habv4/csf_examples/mx8m_mx8mm/template_csf_spl.txt csf_spl.txt
cp $SCRIPT_PATH/doc/imx/habv4/csf_examples/mx8m_mx8mm/template_csf_fit.txt csf_fit.txt
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
if grep -Eq "^CONFIG_IMX8MQ=y$" .config; then
TEE_LOAD_ADDR=0xfe000000
ATF_LOAD_ADDR=0x00910000
elif grep -Eq "^CONFIG_IMX8MM=y$" .config; then
TEE_LOAD_ADDR=0xbe000000
ATF_LOAD_ADDR=0x00920000
elif grep -Eq "^CONFIG_IMX8MN=y$" .config; then
TEE_LOAD_ADDR=0xbe000000
ATF_LOAD_ADDR=0x00960000
elif grep -Eq "^CONFIG_IMX8MP=y$" .config; then
TEE_LOAD_ADDR=0x56000000
ATF_LOAD_ADDR=0x00970000
fi
BLOCK_LEN="0x200"
IVTOFFSET="0x1000"
IVTSIZE="0x20"
FIT_OFFSET=`awk '/File.*image-pos/{print $9}' flash.log | tail -n 1`
FIT_SIZE="0x`awk '/fit/{print $3}' itb.map | head -n 1`"
FIT_UBOOT_OFFSET="0x`awk '/uboot/{print $2}' itb.map | head -n 1`"
FIT_UBOOT_SIZE="0x`awk '/uboot/{print $3}' itb.map | head -n 1`"
FIT_ATF_OFFSET="0x`awk '/atf/{print $2}' itb.map | head -n 1`"
FIT_ATF_SIZE="0x`awk '/atf/{print $3}' itb.map | head -n 1`"
FIT_DTB_OFFSET="0x`awk '/fdt/{print $2}' itb.map | head -n 1`"
FIT_DTB_SIZE="0x`awk '/fdt/{print $3}' itb.map | head -n 1`"
FIT_TEE_OFFSET="0x`awk '/tee/{print $2}' itb.map | head -n 1`"
FIT_TEE_SIZE="0x`awk '/tee/{print $3}' itb.map | head -n 1`"
printf -v FIT_START_ADDR '%#x' "$((CONFIG_SYS_TEXT_BASE - BLOCK_LEN - CONFIG_CSF_SIZE - IVTOFFSET - CONFIG_SYS_CACHELINE_SIZE))"
printf -v FIT_LENGTH '%#x' "$((IVTOFFSET + IVTSIZE))"
sed -i "s|_FIT_START_ADDR_|$FIT_START_ADDR|g" csf_fit.txt
sed -i "s|_FIT_OFFSET_|$FIT_OFFSET|g" csf_fit.txt
sed -i "s|_FIT_LENGTH_|$FIT_LENGTH|g" csf_fit.txt
printf -v UBOOT_OFFSET '%#x' "$((FIT_OFFSET + FIT_UBOOT_OFFSET))"
sed -i "s|_UBOOT_START_ADDR_|$CONFIG_SYS_TEXT_BASE|g" csf_fit.txt
sed -i "s|_UBOOT_OFFSET_|$UBOOT_OFFSET|g" csf_fit.txt
sed -i "s|_UBOOT_LENGTH_|$FIT_UBOOT_SIZE|g" csf_fit.txt
printf -v DTB_START_ADDR '%#x' "$((CONFIG_SYS_TEXT_BASE + FIT_UBOOT_SIZE))"
printf -v DTB_OFFSET '%#x' "$((FIT_OFFSET + FIT_DTB_OFFSET))"
sed -i "s|_DTB_START_ADDR_|$DTB_START_ADDR|g" csf_fit.txt
sed -i "s|_DTB_OFFSET_|$DTB_OFFSET|g" csf_fit.txt
sed -i "s|_DTB_LENGTH_|$FIT_DTB_SIZE|g" csf_fit.txt
printf -v ATF_OFFSET '%#x' "$((FIT_OFFSET + FIT_ATF_OFFSET))"
sed -i "s|_ATF_START_ADDR_|$ATF_LOAD_ADDR|g" csf_fit.txt
sed -i "s|_ATF_OFFSET_|$ATF_OFFSET|g" csf_fit.txt
sed -i "s|_ATF_LENGTH_|$FIT_ATF_SIZE|g" csf_fit.txt
if [ "$FIT_TEE_OFFSET" != "0x" ]; then
	printf -v TEE_OFFSET '%#x' "$((FIT_OFFSET + FIT_TEE_OFFSET))"
	sed -i "s|_TEE_START_ADDR_|$TEE_LOAD_ADDR|g" csf_fit.txt
	sed -i "s|_TEE_OFFSET_|$TEE_OFFSET|g" csf_fit.txt
	sed -i "s|_TEE_LENGTH_|$FIT_TEE_SIZE|g" csf_fit.txt
else
	sed -i "/.*_TEE_START_ADDR.*/d" csf_fit.txt
fi

echo Generating IVT header for FIT...
printf -v IVTADDRESS '%#x' "$((FIT_START_ADDR + IVTOFFSET))"
printf -v CSFADDRESS '%#x' "$((IVTADDRESS + IVTSIZE))"
cp $SCRIPT_PATH/doc/imx/habv4/csf_examples/mx8m_mx8mm/template_genIVT.txt genIVT.pl
sed -i "s|_LOADADDR_|$FIT_START_ADDR|g" genIVT.pl
sed -i "s|_IVTADDRESS_|$IVTADDRESS|g" genIVT.pl
sed -i "s|_CSFADDRESS_|$CSFADDRESS|g" genIVT.pl
chmod +x genIVT.pl
./genIVT.pl
dd if=ivt.bin of=flash.bin seek=$((FIT_OFFSET + IVTOFFSET)) bs=1 conv=notrunc

# generate signatures
$CST_BIN -i csf_spl.txt -o csf_spl.bin
$CST_BIN -i csf_fit.txt -o csf_fit.bin

# copy signatures into binary
CSF_SPL_OFFSET=`awk '/csf_off/{print $2}' flash.log | head -n 1`
CSF_FIT_OFFSET=$((FIT_OFFSET + IVTOFFSET + IVTSIZE))
cp flash.bin signed_flash.bin
dd if=csf_spl.bin of=signed_flash.bin seek=$(($CSF_SPL_OFFSET)) bs=1 conv=notrunc
dd if=csf_fit.bin of=signed_flash.bin seek=$(($CSF_FIT_OFFSET)) bs=1 conv=notrunc

echo "signed_flash.bin is ready!"
