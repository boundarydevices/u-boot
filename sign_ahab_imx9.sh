#!/bin/bash
set -e

for i in flash.bin binman.log; do
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

if [ -z "$SRK_TABLE" ] || [ ! -f $SRK_TABLE ]; then
	echo "Missing SRK_TABLE variable!"
	exit 1
fi

# retrieve the current script path in case it is executed from another location
SCRIPT_PATH=$(dirname $0)

# copy templates and update values
echo "Copying CSF templates..."
cp -v $SCRIPT_PATH/doc/imx/ahab/csf_examples/csf_boot_image.txt .
cp -v $SCRIPT_PATH/doc/imx/ahab/csf_examples/csf_uboot_atf.txt .
sed -i "s|_SRK_TABLE_|$SRK_TABLE|g" csf_*.txt
sed -i "s|_SIGN_KEY_|$SIGN_KEY|g" csf_*.txt

echo "Signing 3rd container (U-Boot + TF-A)..."
UBOOT_START_OFFSET=`grep "parsing u-boot-container.cfgout" binman.log -A 12 -m 1 | awk '/offset:/{print $NF}'`
UBOOT_SIG_OFFSET=`grep "parsing u-boot-container.cfgout" binman.log -A 12 -m 1 | awk '/offset is at/{print $NF}'`
sed -i "s|_UBOOT_START_OFFSET_|$UBOOT_START_OFFSET|g" csf_uboot_atf.txt
sed -i "s|_UBOOT_SIG_OFFSET_|$UBOOT_SIG_OFFSET|g" csf_uboot_atf.txt
$CST_BIN -i csf_uboot_atf.txt -o signed-u-boot-container.img

echo "Copy signed 3rd container back to flash.bin..."
UBOOT_FLASH_OFFSET=`awk '/blob-ext@2.*offset.*to/{print $NF}' binman.log`
dd if=signed-u-boot-container.img of=flash.bin seek=$((UBOOT_FLASH_OFFSET)) bs=1 conv=notrunc

echo "Signing 1st & 2nd container (ELE, M33, SPL).."
BOOT_START_OFFSET=`grep "parsing spl/u-boot-spl.cfgout" binman.log -A 12 -m 1 | awk '/offset:/{print $NF}'`
BOOT_SIG_OFFSET=`grep "parsing spl/u-boot-spl.cfgout" binman.log -A 12 -m 1 | awk '/offset is at/{print $NF}'`
sed -i "s|_BOOT_START_OFFSET_|$BOOT_START_OFFSET|g" csf_boot_image.txt
sed -i "s|_BOOT_SIG_OFFSET_|$BOOT_SIG_OFFSET|g" csf_boot_image.txt
$CST_BIN -i csf_boot_image.txt -o signed-flash.bin

echo "signed-flash.bin is ready!"
