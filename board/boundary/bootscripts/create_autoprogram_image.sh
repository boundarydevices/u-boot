#!/bin/bash

TEMP=/tmp

x1=$(command -v mkimage)
if [ -z "$x1" ]
then
   echo "mkimage not found. please install and try again.";
   exit 1
fi

x2=$(command -v mkfs.vfat)
if [ -z "$x2" ]
then
   echo "mkfs.vfat not found. please install and try again.";
   exit 1
fi

x3=$(command -v fallocate)
if [ -z "$x3" ]
then
   echo "fallocate not found. please install and try again.";
   exit 1
fi

x4=$(command -v mmd)
if [ -z "$x4" ]
then
   echo "mmd not found. please install and try again.";
   exit 1
fi

x5=$(command -v mcopy)
if [ -z "$x5" ]
then
   echo "mcopy not found. please install and try again.";
   exit 1
fi

help()
{
   echo ""
   echo "Usage: $0 0 source.img.gz out.img"
   echo "First parameter should be a digit. 0 if destination is mmc 0 and 1 if destination is mmc 1."
   echo "Second parameter should be the path of the source image file you want to flash to emmc."
   echo "Third parameter should be the path of the output image file which will be created for you."
   exit 1
}

if [[ $1 == *[^[:digit:]]* ]]
then
	echo "first parameter is not a digit";
fi

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]
then
   echo "Some or all of the parameters are empty";
   help
fi

echo "|||| Your gzdest is: mmc $1";
echo "|||| Your source image file is: $2";
echo "|||| Your output image file name will be: $3";

MYSIZE=$(expr $(stat -c%s "$2") / 1024 / 1024 + 8)

mkdir $TEMP

rm -r $3 $TEMP/temp.txt $TEMP/temp2.txt $TEMP/boot.scr $TEMP/$3 $TEMP/out_image/; sync

cp dir2fat32.sh image-gzwrite.txt split-for-gzwrite.sh $2 $TEMP; sync
cd $TEMP

echo "setenv gzdest \"mmc $1\"" | cat - image-gzwrite.txt > temp.txt;sync
echo "setenv imagefile \"$2\"" | cat - $TEMP/temp.txt > temp2.txt; sync

echo "|||| 1- creating bootscript... ";

mkimage -A arm -O linux -T script -C none -a 0 -e 0 -n "bootscript" -d temp2.txt boot.scr; sync

echo "|||| 1- creating bootscript is done ";

mkdir out_image; sync

cp boot.scr out_image/; sync

echo "|||| 2- splitting the input image file... ";

./split-for-gzwrite.sh $2; sync

echo "|||| 2- splitting the input image file is done ";

echo "|||| 3- copying image parts to out_image directory... ";

cp $2.* out_image/; sync

echo "|||| 3- copying image parts to out_image directory is done ";

echo "|||| 4- creating the output fat32 image file... ";

./dir2fat32.sh $3 $MYSIZE out_image; sync

echo "|||| 4- creating the output fat32 image file is done ";

cd -

cp $TEMP/$3 .; sync

echo "|||| 5- removing temporary created files and folders... ";

rm -r $TEMP/temp.txt $TEMP/temp2.txt $TEMP/boot.scr $TEMP/$2 $TEMP/$2.* $TEMP/$3; sync

echo "|||| 5- removing temporary created files and folders... ";

echo "|||| PROCESS COMPLETED. Please flash out.img to an sdcard and use it to auto flash emmc."
