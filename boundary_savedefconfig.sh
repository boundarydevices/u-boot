#!/bin/bash
# syntax ./boundary_savedefconfig "commit message"
#Separate files by board and commit changes after "savedefconfig"

commit_msg=$1;
boards=`ls -d board/boundary/* | sed 's.board/boundary/..'`;

numboards=0;
numsuccess=0;
numfailures=0;
skipped=0;
for board in ${boards} ; do
	update_cnt=0;
	target="";
	defconfigs="";
	hfile="";
	if [ -e board/boundary/${board}/Kconfig ] ; then
		targets=`grep '^if TARGET_' board/boundary/${board}/Kconfig | sed 's.if ..' | sed 's. ||. .'`;
		echo board=${board} target=${targets};
		hfile=`grep -A1 SYS_CONFIG_NAME board/boundary/${board}/Kconfig | grep default | sed 's.default "..' | sed 's."..' | sed 's/[ \x09]*//'`;
		for target in ${targets} ; do
			defconfigs="${defconfigs} `git grep -w CONFIG_${target} configs/ | sed 's.configs/..'| sed 's/_defconfig:.*$//'`";
		done
	fi
	for defconfig in ${defconfigs} ; do
		make ${defconfig}_defconfig;
		make savedefconfig;
		cp defconfig configs/${defconfig}_defconfig;
		echo updated ${defconfig}_defconfig;
		git update-index configs/${defconfig}_defconfig;
		update_cnt=`expr $update_cnt + 1`;
	done
	if [ x${hfile} == x ] ; then
		echo hfile=${hfile} board=${board} target=${target}
	else
		git update-index include/configs/${hfile}.h
		git update-index board/boundary/${board}/*.c
		git update-index board/boundary/${board}/*
		echo "${board}: ${update_cnt} defconfigs updated";
		numsuccess=`expr $numsuccess + 1`;
		git c -m"${board}: ${commit_msg}";
		numboards=`expr $numboards + 1`;
	fi
done
echo -e "\n\ninsert for ${numboards} boards. ${numsuccess} succeeded and ${numfailures} failed, ${skipped} skipped";
