Boot scripts
============

This folder contains all the boot scripts for booting / upgrading any OS:
* `bootscript-mainline.txt`: boot script for any OS using mainline-based kernel
* `bootscript-ubuntu.txt`: boot script for Ubuntu OS using NXP-based kernel
* `bootscript-yocto.txt`: boot script for Yocto OS using NXP-based kernel
* `net_upgrade_fs.txt`: downloads `${upgrade_file}` over TFTP and flashes it into `${upgrade_device}`
* `net_upgradeu_fs.txt`: downloads `${net_upgrade_fs}` over TFTP and executes it
* `net_upgradeu.txt`: downloads `u-boot.${uboot_defconfig}` over TFTP and flashes it into NOR flash
* `prog_fuses.txt`: program the fuses for both MAC address and boot selection
* `upgrade.txt`: downloads `u-boot.${uboot_defconfig}` from local media and flashes it into NOR flash

Those `.txt` files need to be transformed into U-Boot scripts (`.scr`) using `mkimage`.

Here is an example for generating a `boot.scr` for 32-bit platforms (i.MX5/6/7) running Yocto OS:
```
mkimage -A arm -O linux -T script -C none -a 0 -e 0 -n "bootscript" -d bootscript-yocto.txt boot.scr
```
Here is an example for generating a `boot.scr` for 64-bit platforms (i.MX8) running Yocto OS:
```
mkimage -A arm64 -O linux -T script -C none -a 0 -e 0 -n "bootscript" -d bootscript-yocto.txt boot.scr
```
