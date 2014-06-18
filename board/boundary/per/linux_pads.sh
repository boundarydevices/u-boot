#!/bin/sh
echo $1
echo 's/GPIO([1-7])_IO0([0-9])/GPIO_\\1_\\2/' >t.sed
echo 's/GPIO([1-7])_IO([1-3][0-9])/GPIO_\\1_\\2/' >>t.sed
echo 's/GPIO_1__USB_OTG_ID/GPIO_1__USBOTG_ID/' >>t.sed
echo 's/KEY_COL4__USB_OTG_OC/KEY_COL4__USBOH3_USBOTG_OC/' >>t.sed
echo 's/EIM_D30__USB_H1_OC/EIM_D30__USBOH3_USBH1_OC/' >>t.sed
echo 's/UART([1-6]_[RT]X)_DATA/UART\\1D/' >>t.sed
echo 's/__RGMII_/__ENET_RGMII_/' >>t.sed
echo 's/(IPU[12]_CSI[01]_D)ATA0/\\1_/' >>t.sed
echo 's/(IPU[12]_CSI[01]_D)ATA([1-9])/\\1_\\2/' >>t.sed
echo 's/__AUD/__AUDMUX_AUD/' >>t.sed
sed -r -f t.sed $1 >$1.linux_pads
