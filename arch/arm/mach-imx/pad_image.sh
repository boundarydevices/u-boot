#!/bin/bash

if [ $# -eq 1 ]; then
	let size=$(wc -c $1 | awk '{print $1}')
	let padded_size=$(((size + 15) & ~15))

	if [ ${size} != ${padded_size} ]
	then
		echo $1 "is padded to" ${padded_size}
		objcopy -I binary -O binary --pad-to ${padded_size} $1
	fi
elif [ $# -eq 2 ]; then
	let size_1=$(wc -c $1 | awk '{print $1}')
	let size_2=$(wc -c $2 | awk '{print $1}')
	let padded_size=$(((size_1 + size_2 + 15) & ~15))

	if [ $((size_1 + size_2)) != ${padded_size} ]
	then
		echo $1 "+" $2 "are padded to" ${padded_size}
		objcopy -I binary -O binary --pad-to $((padded_size - size_1)) $2
	fi
fi
