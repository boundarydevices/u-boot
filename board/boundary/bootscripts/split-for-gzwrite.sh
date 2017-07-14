#!/bin/bash
let i=0
let offset=0
if [ -z "$1" ] ; then
	echo specify file to split
	exit
fi

zcat $1 >t.tmp
while [ 1 ]
do
	dd if=t.tmp ibs=1M count=512 skip=$offset | gzip >$1.$i
	let size=`zcat $1.$i | wc -c`
	echo file $1.$i size is $size
	if [ $size -lt 536870912 ] ; then
		echo last is $size
		break;
	fi

	let i=$i+1
	let offset=$offset+512
done
rm t.tmp

