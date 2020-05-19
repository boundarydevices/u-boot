#!/bin/bash

BL32="tee.bin"

let fit_off=0x60000

if grep -Eq "^CONFIG_IMX8MQ=y$" .config; then
TEE_LOAD_ADDR=0xfe000000
ATF_LOAD_ADDR=0x00910000
VERSION=v1
BL31=bl31-iMX8MQ.bin
elif grep -Eq "^CONFIG_IMX8MM=y$" .config; then
TEE_LOAD_ADDR=0xbe000000
ATF_LOAD_ADDR=0x00920000
VERSION=v1
BL31=bl31-iMX8MM.bin
elif grep -Eq "^CONFIG_IMX8MN=y$" .config; then
TEE_LOAD_ADDR=0xbe000000
ATF_LOAD_ADDR=0x00960000
VERSION=v2
BL31=bl31-iMX8MN.bin
fi

if [ -z "$BL31" ]; then
	echo "ERROR: BL31 name is not set" >&2
	exit 0
fi

# keep backward compatibility
[ -z "$TEE_LOAD_ADDR" ] && TEE_LOAD_ADDR="0xfe000000"

if [ -z "$ATF_LOAD_ADDR" ]; then
	echo "ERROR: BL31 load address is not set" >&2
	exit 0
fi

if [ "$VERSION" = "v1" ]; then
	let ivt_off=0x400
else
	let ivt_off=0x0
fi

if [ "$BOOT_DEV" = "flexspi" ] || [ ${fit_off} == 0 ]; then
	# We dd flash.bin to 0 offset for flexspi
	let uboot_sign_off=$((fit_off + 0x3000))
else
	# We dd flash.bin to 33KB "0x8400" offset, so need minus 0x8400
	let uboot_sign_off=$((fit_off - 0x8000 - ivt_off + 0x3000))
fi

let uboot_size=$(ls -lct u-boot-nodtb.bin | awk '{print $5}')
let uboot_load_addr=0x40200000

let last_sign_off=$(((uboot_sign_off + uboot_size + 3) & ~3))
let last_load_addr=$((uboot_load_addr + uboot_size))

uboot_size=`printf "0x%X" ${uboot_size}`
uboot_sign_off=`printf "0x%X" ${uboot_sign_off}`
uboot_load_addr=`printf "0x%X" ${uboot_load_addr}`

echo ${uboot_load_addr} ${uboot_sign_off} ${uboot_size}

if [ $# -ge 1 ]; then
	DTBS="$*"
else
	source .config
	for i in $CONFIG_OF_LIST; do
		DTBS="arch/arm/dts/$i.dtb $DTBS"
	done
fi

cnt=0
for dtname in $DTBS
do
	if [ ${cnt} -ge 0 ]
	then
		let fdt${cnt}_size=$(ls -lct $dtname | awk '{print $5}')

		let fdt${cnt}_sign_off=$((last_sign_off))
		let fdt${cnt}_load_addr=$((last_load_addr))
		let last_size=$((fdt${cnt}_size))

		fdt_size=`printf "0x%X" ${last_size}`
		fdt_sign_off=`printf "0x%X" ${last_sign_off}`
		fdt_load_addr=`printf "0x%X" ${last_load_addr}`

		let last_sign_off=$(((last_sign_off + fdt${cnt}_size + 3) & ~3))
		let last_load_addr=$((last_load_addr + fdt${cnt}_size))

		echo ${fdt_load_addr} ${fdt_sign_off} ${fdt_size}
	fi

	cnt=$((cnt+1))
done

let atf_sign_off=$((last_sign_off))
let atf_load_addr=$ATF_LOAD_ADDR
let atf_size=$(ls -lct $BL31 | awk '{print $5}')

if [ ! -f $BL32 ]; then
	let tee_size=0x0
	let tee_sign_off=$((atf_sign_off + atf_size))
else
	let tee_size=$(ls -lct tee.bin | awk '{print $5}')

	let tee_sign_off=$(((atf_sign_off + atf_size + 3) & ~3))
	let tee_load_addr=$TEE_LOAD_ADDR
fi

tee_size=`printf "0x%X" ${tee_size}`
tee_sign_off=`printf "0x%X" ${tee_sign_off}`
tee_load_addr=`printf "0x%X" ${tee_load_addr}`

atf_size=`printf "0x%X" ${atf_size}`
atf_sign_off=`printf "0x%X" ${atf_sign_off}`
atf_load_addr=`printf "0x%X" ${atf_load_addr}`
echo ${atf_load_addr} ${atf_sign_off} ${atf_size}

if [ ${tee_size} != 0x0 ]
then
	echo ${tee_load_addr} ${tee_sign_off} ${tee_size}
fi
