#!/bin/sh
# SPDX-License-Identifier: GPL-2.0+
# Copyright 2024 Ezurio LLC

FW_PATH=$PWD/.fw
FW_IMX=firmware-imx-8.25-27879f8
FW_ELE=firmware-ele-imx-0.1.3-4b30ee5
FW_UPW=firmware-upower-1.3.1
NXP_DL=http://www.nxp.com/lgfiles/NMG/MAD/YOCTO/

download_extract_imx_fw() {
	if [ -d "$1" ]; then
		echo "$PWD/$1/ already exists"
	else
		if [ -e "$1.bin" ]; then
			echo "$PWD/$1.bin already exists"
		else
			echo "Downloading $1.bin"
			wget $NXP_DL/$1.bin
		fi
		echo "Extracting $1.bin"
		sh $1.bin --force --auto-accept
	fi
}

which wget 2>&1 > /dev/null
if [ ! $? -eq 0 ]; then
	echo "Missing the wget binary, you must install it first!"
	exit 1;
fi

# Create and move to fw sub-folder
mkdir -p $FW_PATH
cd $FW_PATH

# Download & extract all the firmware archives necessary
download_extract_imx_fw $FW_IMX
download_extract_imx_fw $FW_ELE
download_extract_imx_fw $FW_UPW

# Copy firmware files
cd -
cp -v $FW_PATH/firmware-imx-*/firmware/ddr/synopsys/lpddr4_* .
cp -v $FW_PATH/firmware-imx-*/firmware/hdmi/cadence/signed_* .
cp -v $FW_PATH/firmware-ele-*/mx*img .
cp -v $FW_PATH/firmware-upower-*/upower_a1.bin upower.bin
