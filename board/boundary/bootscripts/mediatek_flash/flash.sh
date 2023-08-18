#!/bin/bash


# Script to flash the Mediatek Tungsten SMARC with u-boot

# May also just re-program u-boot on the board


PATH_TO_UBOOT_BIN=../../../../u-boot.bin

PATH_TO_LK_BIN=lk.bin

PATH_TO_FIP_BIN=fip.bin

PATH_TO_WIC=bd.wic.img

PATH_TO_BL2=bl2.img

SWD=$PWD

WDIR=""

JUST_FIP=0


help()
{
   echo ""
   echo "Usage: $0 -y [-r] [PATH TO U-BOOT bin file]"
   echo 
   echo "-y to skip the press enter to continue"
   echo "-r to just re-program fip.bin (u-boot)"
   echo "PATH is optional."
   echo
   exit 1
}

end_clean()
{
   if [ "$PWD" == "$WDIR" ]
      then
         rm -rf *
	 cd $SWD
	 rmdir $WDIR
   fi
   exit $1
}

prog_tungsten()
{
   aiot-bootrom
   RESULT=$?
   if [ $RESULT -ne 0 ]
   then
      echo "Error ($RESULT) running aiot-bootrom."
      end_clean $RESULT
   fi

   if [ $JUST_FIP -eq 0 ]
   then
      fastboot flash mmc0 $WIC_BIN
      if [ $RESULT -ne 0 ]
      then
         echo "Error ($RESULT) flashing WIC."
         end_clean $RESULT
      fi
   fi

   fastboot flash bootloaders $FIP_BIN
   RESULT=$?
   if [ $RESULT -ne 0 ]
   then
      echo "Error ($RESULT) flashing FIP."
      end_clean $RESULT
   fi

   if [ $JUST_FIP -eq 0 ]
   then
      fastboot flash mmc0boot0 $BL2_BIN
      if [ $RESULT -ne 0 ]
      then
         echo "Error ($RESULT) flashing BL2."
         end_clean $RESULT
      fi
   fi
}

# Required programs:

x=$(command -v aiot-bootrom)
if [ -z "$x" ]
then
   echo "aiot-bootrom not found. please install and try again.";
   echo "See : https://mediatek.gitlab.io/aiot/doc/aiot-dev-guide/master/sw/yocto/get-started/env-setup/flash-env-linux.html";
   exit 1
fi

x=$(command -v fiptool)
if [ -z "$x" ]
then
   echo "fiptool not found. please install and try again.";
   echo "sudo apt install arm-trusted-firmware-tools";
   exit 1
fi

x=$(command -v fastboot)
if [ -z "$x" ]
then
   echo "fastboot not found. please install and try again.";
   echo "On Ubuntu : sudo apt update; sudo apt-get install android-tools-fastboot"
   exit 1
fi
#

FETCH=curl
x=$(command -v curl)
if [ -z "$x" ]
then
   echo "curl not found.";
   x=$(command -v wget)
   if [ -z "$x" ]
   then
      echo "wget not found. Please install curl or wget";
      echo "On Ubuntu : sudo apt update; sudo apt-get install curl"
      exit 1
   else
      FETCH="wget -O - "
   fi
fi

PAUSE=1
if [ "$1" == "-y" ]
then
   PAUSE=0
   shift
fi

if [ "$1" == "-h" ]||[ "$1" == "--help" ]
then
   help
fi

if [ "$1" == "-r" ]
then
   JUST_FIP=1
   echo "Just re-program fip.bin (u-boot)"
   shift
fi

if [ -n "$1" ]
then
   PATH_TO_UBOOT_BIN="$1"
fi

if [ ! -r $PATH_TO_UBOOT_BIN ]
then
   echo "Unable to read u-boot file [$PATH_TO_UBOOT_BIN]."
   exit 1
fi

if [ ! -r $PATH_TO_FIP_BIN ]
then
   echo "Unable to read fip.bin [$PATH_TO_FIP_BIN]."
   exit 1
fi

if [ ! -r $PATH_TO_LK_BIN ]
then
   echo "Unable to read lk.bin [$PATH_TO_LK_BIN]. Attempting to download it..."
   $FETCH 'https://gitlab.com/mediatek/aiot/rity/lk-prebuilt/-/raw/main/genio-700-evk/lk.bin' > lk.bin
   RESULT=$?
   if [ $RESULT -ne 0 ]
   then
      echo "Error ($RESULT) downloading the lk.bin."
      exit 1
   fi
fi

UBOOT_BIN=`basename $PATH_TO_UBOOT_BIN`

FIP_BIN=`basename $PATH_TO_FIP_BIN`

### Now we can try to create a working directory and update the fip

WDIR=`mktemp -d`

echo "Created temporary directory in $WDIR"

cp $PATH_TO_UBOOT_BIN ${WDIR}/

cp $PATH_TO_FIP_BIN ${WDIR}/

cp $PATH_TO_LK_BIN ${WDIR}/

cd ${WDIR}

fiptool update --nt-fw ${UBOOT_BIN} ${FIP_BIN}

RESULT=$?

if [ $RESULT -ne 0 ]
then
   echo "Error ($RESULT) updating the fip."
   end_clean $RESULT
else
   echo "Updated the fip."
fi

if [ $PAUSE -ne 0 ]
then
   echo
   echo "Make sure the USB-C cable is connected"
   echo "That is is in DL mode ([DL] 00009C40 00000000 010701 or similar on serial terminal)"
   echo
   echo "Press ENTER to Flash now. Or CTRL+C to abort"
   echo
   echo "   Run this script with -h for help"
   read tmp
fi

if [ $JUST_FIP -eq 1 ]
then
   prog_tungsten
   echo "Done."
   end_clean $RESULT
fi

cd $SWD

if [ ! -r $PATH_TO_WIC ]
then
   echo "Unable to read wic file [$PATH_TO_WIC]."
   cd ${WDIR}
   end_clean 1
else
   cp $PATH_TO_WIC ${WDIR}/
   WIC_BIN=`basename $PATH_TO_WIC`
fi

if [ ! -r $PATH_TO_BL2 ]
then
   echo "Unable to read BL2 file [$PATH_TO_BL2]."
   cd ${WDIR}
   end_clean 1
else
   cp $PATH_TO_BL2 ${WDIR}/
   BL2_BIN=`basename $PATH_TO_BL2`
fi

cd ${WDIR}
prog_tungsten

echo "Done."
end_clean 0
