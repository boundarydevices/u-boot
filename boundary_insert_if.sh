#!/bin/bash
# syntax ./boundary_insert_if CONFIG_1 CONFIG_2
#insert CONFIG_1 if CONFIG_2 is already present
#i.e. ./boundary_insert_if CONFIG_SPI_FLASH_GIGADEVICE CONFIG_SPI_FLASH_SST

insert_configs=$1;
check_config=$2;

boards=`ls -d board/boundary/* | sed 's.board/boundary/..'`;

numboards=0;
numsuccess=0;
numfailures=0;
skipped=0;
for board in ${boards} ; do
	update_cnt=0;
	already_there=0;
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
	board_cfgs=":"
	for defconfig in ${defconfigs} ; do
		cfgs=""
		if [[ ${insert_configs} == *=\"* ]] ; then
			if [[ ${insert_configs} == *=* ]] ; then
				cnt=`sed -n "/${insert_configs}/=" configs/${defconfig}_defconfig`
			else
				cnt=`sed -n "/${insert_configs}=/=" configs/${defconfig}_defconfig`
			fi
			if [ "${cnt}" != "" ] ; then
				already_there=`expr $already_there + 1`;
			else
				cfgs="${cfgs} ${insert_configs}"
			fi
		else
			for insert_config in ${insert_configs} ; do
				if [[ ${insert_config} == *=* ]] ; then
					cnt=`sed -n "/${insert_config}/=" configs/${defconfig}_defconfig`
				else
					cnt=`sed -n "/${insert_config}=/=" configs/${defconfig}_defconfig`
				fi
				if [ "${cnt}" != "" ] ; then
					already_there=`expr $already_there + 1`;
				else
					cfgs="${cfgs} ${insert_config}"
				fi
			done
		fi
		cnt=`sed -n "/${check_config}=/=" configs/${defconfig}_defconfig`
		if [ "${cnt}" == "" ] ; then
			cfgs=""
		fi

		if [ "${cfgs}" != "" ] ; then
			make ${defconfig}_defconfig;
			if [[ ${insert_configs} == *=\"* ]] ; then
				if [[ ${insert_configs} == *=* ]] ; then
					echo "${insert_configs}" >>.config;
				else
					echo "${insert_configs}=y" >>.config;
				fi
			else
				for insert_config in ${cfgs} ; do
					if [[ ${insert_config} == *=* ]] ; then
						echo "${insert_config}" >>.config;
					else
						echo "${insert_config}=y" >>.config;
					fi
				done
			fi
			make savedefconfig;
			diff -q defconfig configs/${defconfig}_defconfig;
			if [ $? -eq 0 ] ; then
				already_there=`expr $already_there + 1`;
			else
				if [ $? -eq 1 ] ; then
					cp defconfig configs/${defconfig}_defconfig;
					echo updated ${defconfig}_defconfig;
#					git update-index configs/${defconfig}_defconfig;
					update_cnt=`expr $update_cnt + 1`;
					for insert_config in ${cfgs} ; do
						if [ `expr "${board_cfgs}" : "[A-Z0-9_:]*:${insert_config}:"` -eq 0 ] ; then
							cnt=`sed -n "/${insert_config}=/=" configs/${defconfig}_defconfig`
							if [ "${cnt}" != "" ] ; then
								board_cfgs="${board_cfgs}:${insert_config}:"
							fi
						fi
					done
				else
					numfailures=`expr $numfailures + 1`;
					echo -e "\n\n\n!!!!!!!! insert failure for ${defconfig}_defconfig !!!!!!!!!!!!\n\n";
					read line;
				fi
			fi
		fi
	done
	if [ ${update_cnt} != "0" ] ; then
		echo "${board}: ${update_cnt} defconfigs updated, ${already_there} already there";
		numsuccess=`expr $numsuccess + 1`;
		if [ ${board_cfgs} != ":" ] ; then
#			git c -m"${board}: add ${board_cfgs//::/ } to defconfigs";
			echo updating ${board} ${board_cfgs//::/ }
		else
#			git c -m"${board}: reorder defconfigs";
			echo reorder ${board} ${board_cfgs//::/ }
		fi
	else
		skipped=`expr $skipped + 1`;
	fi
	numboards=`expr $numboards + 1`;
done
echo -e "\n\ninsert for ${numboards} boards. ${numsuccess} succeeded and ${numfailures} failed, ${skipped} skipped";
