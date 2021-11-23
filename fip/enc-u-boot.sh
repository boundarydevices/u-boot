#!/bin/bash
set -e
cd `dirname "$0"`
rm -f bl33.bin u-boot.bin u-boot.bin.sd.bin
cp ../u-boot.bin bl33.bin
./blx_fix.sh bl30.bin zero_tmp bl30_zero.bin bl301.bin bl301_zero.bin bl30_new.bin bl30
./blx_fix.sh bl2.bin zero_tmp bl2_zero.bin acs.bin bl21_zero.bin bl2_new.bin bl2

./aml_encrypt_g12b.bin --bl30sig --input bl30_new.bin         --output bl30_new.bin.g12a.enc --level v3
./aml_encrypt_g12b.bin --bl3sig --input bl30_new.bin.g12a.enc --output bl30_new.bin.enc --level v3 --type bl30
./aml_encrypt_g12b.bin --bl3sig --input bl31.img 		  --output bl31.img.enc --level v3 --type bl31
./aml_encrypt_g12b.bin --bl3sig --input bl33.bin --compress lz4 --output bl33.bin.enc --level v3 --type bl33 --compress lz4
./aml_encrypt_g12b.bin --bl2sig --input bl2_new.bin		--output bl2.n.bin.sig 
./aml_encrypt_g12b.bin --bootmk --output u-boot.bin --bl2 bl2.n.bin.sig --bl30 bl30_new.bin.enc --bl31 bl31.img.enc --bl33 bl33.bin.enc \
	--ddrfw1 ddr4_1d.fw --ddrfw2 ddr4_2d.fw --ddrfw3 ddr3_1d.fw --ddrfw4 piei.fw --ddrfw5 lpddr4_1d.fw --ddrfw6 lpddr4_2d.fw \
	--ddrfw7 diag_lpddr4.fw --ddrfw8 aml_ddr.fw --ddrfw9 lpddr3_1d.fw --level v3

rm bl2.n.bin.sig bl30_new.bin.enc bl30_new.bin.g12a.enc bl31.img.enc bl33.bin.enc u-boot.bin.usb.bl2 u-boot.bin.usb.tpl
rm bl21_zero.bin  bl2_zero.bin  bl301_zero.bin  bl30_zero.bin bl2_new.bin  bl30_new.bin

#DEV=/dev/sdc
#make distclean
#make a311d_defconfig
#make u-boot.bin -j4
#fip/enc-u-boot.sh
#sudo dd if=fip/u-boot.bin.sd.bin of=$DEV conv=fsync,notrunc bs=512 skip=1 seek=1; sudo dd if=fip/u-boot.bin.sd.bin of=$DEV conv=fsync,notrunc bs=1 count=444

