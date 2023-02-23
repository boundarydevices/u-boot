#!/bin/bash
# ./lpddr4.sh lpddr4_timing.c 4g 8mq

if [ $# -lt 1 ]; then
    echo "missing filename, ie. lpddr4_timing.c"
    exit 1;
fi
if [ $# -lt 2 ]; then
    echo "missing memory size arg, 2g,2gr0,4g,8g"
    exit 1;
fi
if [ $# -lt 3 ]; then
    echo "missing processor arg, 8mm,8mn,8mp,8mq,8ulp"
    exit 1;
fi
mem=$2
processor=$3

echo $1
sed -E -i \
 -e "s/\x0d$//" \
 -e "s/[ \x09]*$//" \
 -e "s/ \x09/\x09/" \
 -e '${/^$/d;}' \
 $1

#stage 1
echo stage 1
sed -E -i \
 -e 's/\{([0-9a-zA-Z])/\{ \1/' \
 -e 's/,0x/, 0x/' \
 -e 's/([0-9a-fA-F]{1,})\}/\1 \}/' \
 -e 's/0x([0-9a-fA-F]{1,}[ ,\(\)\+\-\*])/0x\L\1\E/g' \
 -e 's/0x([0-9a-f]{1} )/0x0000000\1/' \
 -e 's/0x([0-9a-f]{2} )/0x000000\1/' \
 -e 's/0x([0-9a-f]{3} )/0x00000\1/' \
 -e 's/0x([0-9a-f]{4} )/0x0000\1/' \
 -e 's/0x([0-9a-f]{5} )/0x000\1/' \
 -e 's/0x([0-9a-f]{6} )/0x00\1/' \
 -e 's/0x([0-9a-f]{7} )/0x0\1/' \
 -e 's/0x([0-9a-f]{1},)/0x00000\1/' \
 -e 's/0x([0-9a-f]{2},)/0x0000\1/' \
 -e 's/0x([0-9a-f]{3},)/0x000\1/' \
 -e 's/0x([0-9a-f]{4},)/0x00\1/' \
 -e 's/0x([0-9a-f]{5},)/0x0\1/' \
 -e 's/ddr_/lpddr4_/' \
 -e 's/lpddr4_memory_map/ddr_memory_map/' \
 -e '/^#include <linux\/kernel.h>/d' \
 -e 's/^#include <asm\/arch\/ddr.h>/#include "lpddr4_timing_ch2.h"/' \
 -e 's/\{ 0x0([0-9a-f]{4,5}),/\{ 0x\1,/' \
 -e 's/\{ 0x0([0-9a-f]{4,5}),/\{ 0x\1,/' \
 -e 's/0x000000([0-9a-f]{2}[ ,\(\)\+\-\*])/0x\1/' \
 -e 's/0x0000([0-9a-f]{4}[ ,\(\)\+\-\*])/0x\1/' \
 -e 's/0x0([0-9]{1}[ ,\(\)\+\-\*])/\1/' \
 -e 's/^struct dram_cfg_param /static struct dram_cfg_param /' \
 -e 's/^struct dram_fsp_msg/static struct dram_fsp_msg/' \
 -e 's/ paremeter / parameter /' \
$1

#stage 2
echo stage 2
if [ ${processor} == "8ulp" ] ; then
	echo skipping defines
#	sed -E -i -f lpddr4_sed_defines_8ulp.txt $1
else
	sed -E -i -f lpddr4_sed_defines.txt $1
fi

#stage 3
echo stage 3
sed -E -i \
 -e 's/ DDRC_INIT4\(0\), 0x00310000 / DDRC_INIT4\(0\), CH2_VAL_INIT4 /' \
 -e 's/ DDRC_FREQ1_INIT4\(0\), 0x00310000 / DDRC_FREQ1_INIT4\(0\), CH2_VAL_INIT4 /' \
 -e 's/ DDRC_FREQ2_INIT4\(0\), 0x00310000 / DDRC_FREQ2_INIT4\(0\), CH2_VAL_INIT4 /' \
 -e 's/ 0x54006, 0x11 / 0x54006, LPDDR4_PHY_VREF_VALUE /' \
 -e 's/ 0x54009, 0xc8 / 0x54009, LPDDR4_HDT_CTL_3200_1D /' \
 -e '/DDRC_PWRCTL\(0\), 1/ i \\t/* selfref_en=1, SDRAM enter self-refresh state */' \
 -e '/DDRC_ADDRMAP0\(0\),/ i\\' \
 -e '/DDRC_ADDRMAP0\(0\),/ i \\t/* address mapping */' \
 -e '/DDRC_ADDRMAP4\(0\), 0x1f1f/ i \\t/* addrmap_col_b10 and addrmap_col_b11 set to de-activated (5-bit width) */' \
 -e '/DDRC_ADDRMAP1\(0\), 0x00080808/ i \\t/* bank interleave */' \
 -e '/DDRC_ADDRMAP1\(0\), 0x00080808/ i \\t/* addrmap_bank_b2, addrmap_bank_b1, addrmap_bank_b0 */' \
 -e '/DDRC_ADDRMAP5\(0\), 0x07070707/ i \\t/* addrmap_row_b11, addrmap_row_b10_b2, addrmap_row_b1, addrmap_row_b0 */' \
 -e '/DDRC_ADDRMAP6\(0\),/ i \\t/* addrmap_row_b15, addrmap_row_b14, addrmap_row_b13, addrmap_row_b12 */' \
 -e 's/ dram_timing / dram_timing_ch2 /' \
$1

match=0
if [ "${mem}" == "2g" ] ; then
	echo 2g
	sed -E -i \
	 -e 's/0x5402c, 3 /0x5402c, CH2_LPDDR4_CS /' \
	 -e 's/0x54012, 0x0310 /0x54012, 0x10 | (CH2_LPDDR4_CS << 8) /' \
	 -e 's/ DDRC_MSTR\(0\), 0xa3080020 / DDRC_MSTR\(0\), 0xa0080020 | \(CH2_LPDDR4_CS << 24\) /' \
	 -e 's/ DDRC_ADDRMAP0\(0\), 0x16 / DDRC_ADDRMAP0\(0\), CH2_VAL_DDRC_ADDRMAP0 /' \
	 -e 's/ DDRC_ADDRMAP6\(0\), 0x0f070707 / DDRC_ADDRMAP6\(0\), CH2_VAL_DDRC_ADDRMAP6 /' \
	 -e 's/ DDRC_ADDRMAP7\(0\), 0x0f0f / DDRC_ADDRMAP7\(0\), CH2_VAL_DDRC_ADDRMAP7 /' \
	 -e 's/ DDRC_RFSHTMG\(0\), 0x007a00b4 / DDRC_RFSHTMG\(0\), CH2_VAL_DDRC_RFSHTMG /' \
	 -e 's/ DDRC_DRAMTMG14\(0\), 0xbc / DDRC_DRAMTMG14\(0\), CH2_VAL_DDRC_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ1_RFSHTMG\(0\), 0x000c0012 / DDRC_FREQ1_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ1_RFSHTMG /' \
	 -e 's/ DDRC_FREQ1_DRAMTMG14\(0\), 0x13 / DDRC_FREQ1_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ1_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ2_RFSHTMG\(0\), 0x00030005 / DDRC_FREQ2_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ2_RFSHTMG /' \
	 -e 's/ DDRC_FREQ2_DRAMTMG14\(0\), 5 / DDRC_FREQ2_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ2_DRAMTMG14 /' \
	$1
	match=1
fi
if [ "${mem}" == "2gr0" ] ; then
	echo 2gr0
	sed -E -i \
	 -e 's/0x5402c, 1 /0x5402c, CH2_LPDDR4_CS /' \
	 -e 's/0x54012, 0x0110 /0x54012, 0x10 | (CH2_LPDDR4_CS << 8) /' \
	 -e 's/ DDRC_MSTR\(0\), 0xa1080020 / DDRC_MSTR\(0\), 0xa0080020 | \(CH2_LPDDR4_CS << 24\) /' \
	 -e 's/ DDRC_ADDRMAP0\(0\), 0x1f / DDRC_ADDRMAP0\(0\), CH2_VAL_DDRC_ADDRMAP0 /' \
	 -e 's/ DDRC_ADDRMAP6\(0\), 0x07070707 / DDRC_ADDRMAP6\(0\), CH2_VAL_DDRC_ADDRMAP6 /' \
	 -e 's/ DDRC_ADDRMAP7\(0\), 0x0f0f / DDRC_ADDRMAP7\(0\), CH2_VAL_DDRC_ADDRMAP7 /' \
	 -e 's/ DDRC_RFSHTMG\(0\), 0x007a0118 / DDRC_RFSHTMG\(0\), CH2_VAL_DDRC_RFSHTMG /' \
	 -e 's/ DDRC_DRAMTMG14\(0\), 0x0120 / DDRC_DRAMTMG14\(0\), CH2_VAL_DDRC_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ1_RFSHTMG\(0\), 0x000c001c / DDRC_FREQ1_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ1_RFSHTMG /' \
	 -e 's/ DDRC_FREQ1_DRAMTMG14\(0\), 0x1d / DDRC_FREQ1_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ1_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ2_RFSHTMG\(0\), 0x00030007 / DDRC_FREQ2_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ2_RFSHTMG /' \
	 -e 's/ DDRC_FREQ2_DRAMTMG14\(0\), 8 / DDRC_FREQ2_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ2_DRAMTMG14 /' \
	$1
	match=1
fi
if [ "${mem}" == "4g" ] ; then
	echo 4g
	sed -E -i \
	 -e 's/0x5402c, 3 /0x5402c, CH2_LPDDR4_CS /' \
	 -e 's/0x54012, 0x0310 /0x54012, 0x10 | (CH2_LPDDR4_CS << 8) /' \
	 -e 's/ DDRC_MSTR\(0\), 0xa3080020 / DDRC_MSTR\(0\), 0xa0080020 | \(CH2_LPDDR4_CS << 24\) /' \
	 -e 's/ DDRC_ADDRMAP0\(0\), 0x17 / DDRC_ADDRMAP0\(0\), CH2_VAL_DDRC_ADDRMAP0 /' \
	 -e 's/ DDRC_ADDRMAP6\(0\), 0x07070707 / DDRC_ADDRMAP6\(0\), CH2_VAL_DDRC_ADDRMAP6 /' \
	 -e 's/ DDRC_ADDRMAP7\(0\), 0x0f0f / DDRC_ADDRMAP7\(0\), CH2_VAL_DDRC_ADDRMAP7 /' \
	 -e 's/ DDRC_RFSHTMG\(0\), 0x007a0118 / DDRC_RFSHTMG\(0\), CH2_VAL_DDRC_RFSHTMG /' \
	 -e 's/ DDRC_DRAMTMG14\(0\), 0x0120 / DDRC_DRAMTMG14\(0\), CH2_VAL_DDRC_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ1_RFSHTMG\(0\), 0x000c001c / DDRC_FREQ1_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ1_RFSHTMG /' \
	 -e 's/ DDRC_FREQ1_DRAMTMG14\(0\), 0x1d / DDRC_FREQ1_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ1_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ2_RFSHTMG\(0\), 0x00030007 / DDRC_FREQ2_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ2_RFSHTMG /' \
	 -e 's/ DDRC_FREQ2_DRAMTMG14\(0\), 8 / DDRC_FREQ2_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ2_DRAMTMG14 /' \
	$1
	match=1
fi
if [ "${mem}" == "8g" ] ; then
	echo 8g
	sed -E -i \
	 -e 's/0x5402c, 3 /0x5402c, CH2_LPDDR4_CS /' \
	 -e 's/0x54012, 0x0310 /0x54012, 0x10 | (CH2_LPDDR4_CS << 8) /' \
	 -e 's/ DDRC_MSTR\(0\), 0xa3080020 / DDRC_MSTR\(0\), 0xa0080020 | \(CH2_LPDDR4_CS << 24\) /' \
	 -e 's/ DDRC_ADDRMAP0\(0\), 0x18 / DDRC_ADDRMAP0\(0\), CH2_VAL_DDRC_ADDRMAP0 /' \
	 -e 's/ DDRC_ADDRMAP6\(0\), 0x07070707 / DDRC_ADDRMAP6\(0\), CH2_VAL_DDRC_ADDRMAP6 /' \
	 -e 's/ DDRC_ADDRMAP7\(0\), 0x0f07 / DDRC_ADDRMAP7\(0\), CH2_VAL_DDRC_ADDRMAP7 /' \
	 -e 's/ DDRC_RFSHTMG\(0\), 0x007a017c / DDRC_RFSHTMG\(0\), CH2_VAL_DDRC_RFSHTMG /' \
	 -e 's/ DDRC_DRAMTMG14\(0\), 0x0184 / DDRC_DRAMTMG14\(0\), CH2_VAL_DDRC_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ1_RFSHTMG\(0\), 0x000c0026 / DDRC_FREQ1_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ1_RFSHTMG /' \
	 -e 's/ DDRC_FREQ1_DRAMTMG14\(0\), 0x27 / DDRC_FREQ1_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ1_DRAMTMG14 /' \
	 -e 's/ DDRC_FREQ2_RFSHTMG\(0\), 0x0003000a / DDRC_FREQ2_RFSHTMG\(0\), CH2_VAL_DDRC_FREQ2_RFSHTMG /' \
	 -e 's/ DDRC_FREQ2_DRAMTMG14\(0\), 0x0a / DDRC_FREQ2_DRAMTMG14\(0\), CH2_VAL_DDRC_FREQ2_DRAMTMG14 /' \
	$1
	match=1
fi
if [ $match == 0 ] ; then
	echo $2 is invalid
	exit 1
fi

#stage 4
#!/bin/bash
if [ ${processor} != "8ulp" ] ; then
	echo stage 4
	sed -E \
	 -e '\|^/\* ddr phy trained csr |,/};/!d' $1 >t1.tmp
	sed -E -i \
	 -e 's/static struct dram_cfg_param lpddr4_ddrphy_trained_csr/struct dram_cfg_param ddrphy_trained_csr/' \
	 -e 's/0x0080,/0x80,/' t1.tmp

	sed -E \
	 -e '\|^/\* ddr phy trained csr |,/};/!d' drivers/ddr/imx/imx8m/ddrphy_csr.c >t2.tmp
	diff -u t1.tmp t2.tmp
	if [ $? -ne 0 ]; then
		echo aborting
		exit 1
	fi
	rm t1.tmp t2.tmp
fi

sed -E -i \
 -e '/ddrphy_trained_csr =/d' \
 -e '/ddrphy_trained_csr_num =/d' \
 -e '\|^/\* ddr phy trained csr |,/};/d' \
$1
cp $1 board/boundary/common/lpddr4_timing_${processor}.c
