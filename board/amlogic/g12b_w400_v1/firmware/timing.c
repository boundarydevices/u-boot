
/*
 * board/khadas/kvim3/firmware/timing.c
 *
 * Copyright (C) 2019 Wesion, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <asm/arch/secure_apb.h>
#include <asm/arch/timing.h>
#include <asm/arch/ddr_define.h>



/* ddr config support multiple configs for boards which use same bootloader:
 * config steps:
 * 1. add a new data struct in __ddr_setting[]
 * 2. config correct board_id, ddr_type, freq, etc..
 */


/* CAUTION!! */
/* Confirm ddr configs with hardware designer,
 * if you don't know how to config, then don't edit it
 */

/* Key configs */
/*
 * board_id: check hardware adc config
 * dram_rank_config:
 *            #define CONFIG_DDR_CHL_AUTO					0xF
 *            #define CONFIG_DDR0_16BIT_CH0					0x1
 *            #define CONFIG_DDR0_16BIT_RANK01_CH0			0x4
 *            #define CONFIG_DDR0_32BIT_RANK0_CH0			0x2
 *            #define CONFIG_DDR0_32BIT_RANK01_CH01			0x3
 *            #define CONFIG_DDR0_32BIT_16BIT_RANK0_CH0		0x5
 *            #define CONFIG_DDR0_32BIT_16BIT_RANK01_CH0	0x6
 * DramType:
 *            #define CONFIG_DDR_TYPE_DDR3					0
 *            #define CONFIG_DDR_TYPE_DDR4					1
 *            #define CONFIG_DDR_TYPE_LPDDR4				2
 *            #define CONFIG_DDR_TYPE_LPDDR3				3
 * DRAMFreq:
 *            {pstate0, pstate1, pstate2, pstate3} //more than one pstate means use dynamic freq
 *
 */

ddr_set_t __ddr_setting[] = {
{
	/* lpddr4x SK hynix H9HCNNNCPMALHR-NEE 2 RANKs 4GB */
	/* lpddr4x SK hynix H9HCNNNCPMMLHR-NME 2 RANKs 4GB */
	.board_id				= CONFIG_BOARD_ID_MASK,
	.version				= 1,
	.dram_rank_config		= CONFIG_DDR0_32BIT_RANK01_CH01,
	.ddr_rfc_type			= DDR_RFC_TYPE_LPDDR4_8Gbx1,
	.DramType				= CONFIG_DDR_TYPE_LPDDR4,
	.DRAMFreq				= {1608, 0, 0, 0},
	.ddr_base_addr			= CFG_DDR_BASE_ADDR,
	.ddr_start_offset		= CFG_DDR_START_OFFSET,
	.imem_load_addr			= 0xFFFC0000, //sram
	.dmem_load_size			= 0x1000, //4K

	.DisabledDbyte			= 0xf0,
	.Is2Ttiming				= 0,
	.HdtCtrl				= 0xa,
	.dram_cs0_size_MB		= 0xffff,//1024,
	.dram_cs1_size_MB		= 0xffff,//1024,
	.training_SequenceCtrl	= {0x131f,0x61}, //ddr3 0x21f 0x31f
	.phy_odt_config_rank	= {0x0,0x0}, //use 0x23 0x13  compatibility with 1rank and 2rank //targeting rank 0. [3:0] is used //for write ODT [7:4] is used for //read ODT
	.dfi_odt_config			= 0,  //use 0d0d compatibility with 1rank and 2rank  //0808
	.PllBypassEn			= 0, //bit0-ps0,bit1-ps1
	.ddr_rdbi_wr_enable		= 0,
	.clk_drv_ohm			= 40,
	.cs_drv_ohm				= 40,
	.ac_drv_ohm				= 40,
	.soc_data_drv_ohm_p		= 40,
	.soc_data_drv_ohm_n		= 40,
	.soc_data_odt_ohm_p		= 40,
	.soc_data_odt_ohm_n		= 40,	//4layer 40/60;
	.dram_data_drv_ohm		= 40, //lpddr4 sdram only240/1-6
	.dram_data_odt_ohm		= 60,
	.dram_ac_odt_ohm		= 80,//120,// 120,  //take care if use lpddr4x ,rank0 and rank1 both will be enable on die ca odt
	.lpddr4_dram_vout_voltage_1_3_2_5_setting   = 1,///1, 1/3vddq     0 2/5 vddq
	.soc_clk_slew_rate		= 0x3ff,//0x253,
	.soc_cs_slew_rate		= 0x100,//0x253,
	.soc_ac_slew_rate		= 0x100,//0x253,
	.soc_data_slew_rate		= 0x1ff,
	.vref_output_permil		= 250,// (5500/2*6)*130/100,// 260,//200,
	.vref_receiver_permil	= 180,//(250*6/11)*110/100,
	.vref_dram_permil		= 180,//300,//200,//(250*11/6),
	//.vref_reverse			= 0,
	.ac_trace_delay			= {00,0x0,0,0,0,0,0x0,00},
	.ac_pinmux				= {00,00},
	.ddr_dmc_remap			= {
							[0] = ( 5 |  6 << 5 |  7 << 10 |  8<< 15 | 9<< 20 | 10 << 25 ),
							[1] = ( 11|  0 << 5 |  0 << 10 | 15 << 15 | 16 << 20 | 17 << 25 ),
							[2] = ( 18| 19 << 5 | 20 << 10 | 21 << 15 | 22 << 20 | 23 << 25 ),
							[3] = ( 24| 25 << 5 | 26 << 10 | 27 << 15 | 28 << 20 | 29 << 25 ),
							[4] = ( 30| 12 << 5 | 13 << 10 |  14<< 15 |  0 << 20 |  0 << 25 ),
	},
	.ddr_lpddr34_ca_remap	= {00,00},
	.ddr_lpddr34_dq_remap	= {3,2,0,1,7,6,5,4, 10,9,14,11,8,12,13,15, 20,21,23,22,18,17,19,16, 28,26,25,24,31,30,27,29},
	.dram_rtt_nom_wr_park	= {00,00},

	/* pll ssc config:
	 *
	 *   pll_ssc_mode = (1<<20) | (1<<8) | ([strength] << 4) | [mode],
	 *      ppm = strength * 500
	 *      mode: 0=center, 1=up, 2=down
	 *
	 *   eg:
	 *     1. config 1000ppm center ss. then mode=0, strength=2
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (2 << 4) | 0,
	 *     2. config 3000ppm down ss. then mode=2, strength=6
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (6 << 4) | 2,
	 */
	.pll_ssc_mode			= (1<<20) | (1<<8) | (2<<4) | 0,//center_ssc_1000ppm
	.ddr_func				= DDR_FUNC,
	.magic					= DRAM_CFG_MAGIC,
	.bitTimeControl_2d      = 1,
	.fast_boot[0]			= 0,
	.enable_lpddr4x_mode	= 1,
},
{
	/* lpddr4x SK hynix H9HCNNNBKMMLHR-NME 1 RANK 2GB */
	.board_id				= CONFIG_BOARD_ID_MASK,
	.version				= 1,
	.dram_rank_config		= CONFIG_DDR0_32BIT_RANK0_CH01,
	.ddr_rfc_type			= DDR_RFC_TYPE_LPDDR4_8Gbx1,
	.DramType				= CONFIG_DDR_TYPE_LPDDR4,
	.DRAMFreq				= {1608, 0, 0, 0},
	.ddr_base_addr			= CFG_DDR_BASE_ADDR,
	.ddr_start_offset		= CFG_DDR_START_OFFSET,
	.imem_load_addr			= 0xFFFC0000, //sram
	.dmem_load_size			= 0x1000, //4K

	.DisabledDbyte			= 0xf0,
	.Is2Ttiming				= 0,
	.HdtCtrl				= 0xa,
	.dram_cs0_size_MB		= 0xffff,//1024,
	.dram_cs1_size_MB		= 0x0,//1024,
	.training_SequenceCtrl	= {0x131f,0x61}, //ddr3 0x21f 0x31f
	.phy_odt_config_rank	= {0x0,0x0}, //use 0x23 0x13  compatibility with 1rank and 2rank //targeting rank 0. [3:0] is used //for write ODT [7:4] is used for //read ODT
	.dfi_odt_config			= 0,  //use 0d0d compatibility with 1rank and 2rank  //0808
	.PllBypassEn			= 0, //bit0-ps0,bit1-ps1
	.ddr_rdbi_wr_enable		= 0,
	.clk_drv_ohm			= 40,
	.cs_drv_ohm				= 40,
	.ac_drv_ohm				= 40,
	.soc_data_drv_ohm_p		= 40,
	.soc_data_drv_ohm_n		= 40,
	.soc_data_odt_ohm_p		= 40,
	.soc_data_odt_ohm_n		= 40,	//4layer 40/60;
	.dram_data_drv_ohm		= 40, //lpddr4 sdram only240/1-6
	.dram_data_odt_ohm		= 60,
	.dram_ac_odt_ohm		= 80,//120,// 120,  //take care if use lpddr4x ,rank0 and rank1 both will be enable on die ca odt
	.lpddr4_dram_vout_voltage_1_3_2_5_setting   = 1,///1, 1/3vddq     0 2/5 vddq
	.soc_clk_slew_rate		= 0x3ff,//0x253,
	.soc_cs_slew_rate		= 0x100,//0x253,
	.soc_ac_slew_rate		= 0x100,//0x253,
	.soc_data_slew_rate		= 0x1ff,
	.vref_output_permil		= 250,// (5500/2*6)*130/100,// 260,//200,
	.vref_receiver_permil	= 180,//(250*6/11)*110/100,
	.vref_dram_permil		= 180,//300,//200,//(250*11/6),
	//.vref_reverse			= 0,
	.ac_trace_delay			= {00,0x0,0,0,0,0,0x0,00},
	.ac_pinmux				= {00,00},
	.ddr_dmc_remap			= {
							[0] = ( 5 |  6 << 5 |  7 << 10 |  8<< 15 | 9<< 20 | 10 << 25 ),
							[1] = ( 11|  0 << 5 |  0 << 10 | 15 << 15 | 16 << 20 | 17 << 25 ),
							[2] = ( 18| 19 << 5 | 20 << 10 | 21 << 15 | 22 << 20 | 23 << 25 ),
							[3] = ( 24| 25 << 5 | 26 << 10 | 27 << 15 | 28 << 20 | 29 << 25 ),
							[4] = ( 30| 12 << 5 | 13 << 10 |  14<< 15 |  0 << 20 |  0 << 25 ),
	},
	.ddr_lpddr34_ca_remap	= {00,00},
	.ddr_lpddr34_dq_remap	= {3,2,0,1,7,6,5,4, 10,9,14,11,8,12,13,15, 20,21,23,22,18,17,19,16, 28,26,25,24,31,30,27,29},
	.dram_rtt_nom_wr_park	= {00,00},

	/* pll ssc config:
	 *
	 *   pll_ssc_mode = (1<<20) | (1<<8) | ([strength] << 4) | [mode],
	 *      ppm = strength * 500
	 *      mode: 0=center, 1=up, 2=down
	 *
	 *   eg:
	 *     1. config 1000ppm center ss. then mode=0, strength=2
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (2 << 4) | 0,
	 *     2. config 3000ppm down ss. then mode=2, strength=6
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (6 << 4) | 2,
	 */
	.pll_ssc_mode			= (1<<20) | (1<<8) | (2<<4) | 0,//center_ssc_1000ppm
	.ddr_func				= DDR_FUNC,
	.magic					= DRAM_CFG_MAGIC,
	.bitTimeControl_2d      = 1,
	.fast_boot[0]			= 0,
	.enable_lpddr4x_mode	= 1,
},
{
	/* lpddr4 Samsumg K4F6E3S4HM-MGCJ 1 RANK */
	.board_id				= CONFIG_BOARD_ID_MASK,
	.version				= 1,
	.dram_rank_config		= CONFIG_DDR0_32BIT_RANK0_CH01,
	.ddr_rfc_type			= DDR_RFC_TYPE_LPDDR4_8Gbx1,
	.DramType				= CONFIG_DDR_TYPE_LPDDR4,
	.DRAMFreq				= {1608, 0, 0, 0},
	.ddr_base_addr			= CFG_DDR_BASE_ADDR,
	.ddr_start_offset		= CFG_DDR_START_OFFSET,
	.imem_load_addr			= 0xFFFC0000, //sram
	.dmem_load_size			= 0x1000, //4K

	.DisabledDbyte			= 0xf0,
	.Is2Ttiming				= 0,
	.HdtCtrl				= 0xa,
	.dram_cs0_size_MB		= 0xffff,//1024,
	.dram_cs1_size_MB		= 0,//1024,
	.training_SequenceCtrl	= {0x131f,0x61}, //ddr3 0x21f 0x31f
	.phy_odt_config_rank	= {0x00,0x00}, //use 0x23 0x13  compatibility with 1rank and 2rank //targeting rank 0. [3:0] is used //for write ODT [7:4] is used for //read ODT
	.dfi_odt_config			= 0x0d0d,  //use 0d0d compatibility with 1rank and 2rank  //0808
	.PllBypassEn			= 0, //bit0-ps0,bit1-ps1
	.ddr_rdbi_wr_enable		= 0,
	.clk_drv_ohm			= 40,
	.cs_drv_ohm				= 40,
	.ac_drv_ohm				= 40,
	.soc_data_drv_ohm_p		= 40,
	.soc_data_drv_ohm_n		= 40,
	.soc_data_odt_ohm_p		= 0,
	.soc_data_odt_ohm_n		= 120,
	.dram_data_drv_ohm		= 40, //lpddr4 sdram only240/1-6
	.dram_data_odt_ohm		= 120,
	.dram_ac_odt_ohm		= 120,
	.lpddr4_dram_vout_voltage_1_3_2_5_setting = 1,///1, 1/3vddq     0 2/5 vddq
	.soc_clk_slew_rate		= 0x3ff,//0x253,
	.soc_cs_slew_rate		= 0x100,//0x253,
	.soc_ac_slew_rate		= 0x100,//0x253,
	.soc_data_slew_rate		= 0x1ff,
	.vref_output_permil		= 350,//200,
	.vref_receiver_permil	= 0,
	.vref_dram_permil		= 0,
	//.vref_reverse			= 0,
	.ac_trace_delay			= {00,0x0,0,0,0,0,0x0,00},
	//.ac_trace_delay		= {32,32,32,32,32,32,32,32,32,32},
	.ac_pinmux				= {00,00},
	.ddr_dmc_remap			= {
							[0] = ( 5 |  6 << 5 |  7 << 10 |  8<< 15 | 9<< 20 | 10 << 25 ),
							[1] = ( 11|  0 << 5 |  0 << 10 | 15 << 15 | 16 << 20 | 17 << 25 ),
							[2] = ( 18| 19 << 5 | 20 << 10 | 21 << 15 | 22 << 20 | 23 << 25 ),
							[3] = ( 24| 25 << 5 | 26 << 10 | 27 << 15 | 28 << 20 | 29 << 25 ),
							[4] = ( 30| 12 << 5 | 13 << 10 |  14<< 15 |  0 << 20 |  0 << 25 ),
	},
	.ddr_lpddr34_ca_remap	= {00,00},
	.ddr_lpddr34_dq_remap	= {3,2,0,1,7,6,5,4, 10,9,14,11,8,12,13,15, 20,21,23,22,18,17,19,16, 28,26,25,24,31,30,27,29},
	.dram_rtt_nom_wr_park	= {00,00},

	/* pll ssc config:
	 *
	 *   pll_ssc_mode = (1<<20) | (1<<8) | ([strength] << 4) | [mode],
	 *      ppm = strength * 500
	 *      mode: 0=center, 1=up, 2=down
	 *
	 *   eg:
	 *     1. config 1000ppm center ss. then mode=0, strength=2
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (2 << 4) | 0,
	 *     2. config 3000ppm down ss. then mode=2, strength=6
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (6 << 4) | 2,
	 */
	.pll_ssc_mode			= (1<<20) | (1<<8) | (2<<4) | 0,//center_ssc_1000ppm
	.ddr_func				= DDR_FUNC,
	.magic					= DRAM_CFG_MAGIC,
	.diagnose				= CONFIG_DIAGNOSE_DISABLE,
	.bitTimeControl_2d      = 1,
	.fast_boot[0]           = 1,
	.rever1                 = 0,
},
{
	/* lpddr4 Samsumg K4F6E3D4HB-MGCJ 2 RANKs */
	.board_id				= CONFIG_BOARD_ID_MASK,
	.version				= 1,
	.dram_rank_config		= CONFIG_DDR0_32BIT_RANK01_CH01,
	.ddr_rfc_type			= DDR_RFC_TYPE_LPDDR4_8Gbx1,
	.DramType				= CONFIG_DDR_TYPE_LPDDR4,
	.DRAMFreq				= {1608, 0, 0, 0},
	.ddr_base_addr			= CFG_DDR_BASE_ADDR,
	.ddr_start_offset		= CFG_DDR_START_OFFSET,
	.imem_load_addr			= 0xFFFC0000, //sram
	.dmem_load_size			= 0x1000, //4K

	.DisabledDbyte			= 0xf0,
	.Is2Ttiming				= 0,
	.HdtCtrl				= 0xa,
	.dram_cs0_size_MB		= 0xffff,//1024,
	.dram_cs1_size_MB		= 0xffff,//1024,
	.training_SequenceCtrl	= {0x131f,0x61}, //ddr3 0x21f 0x31f
	.phy_odt_config_rank	= {0x23,0x13}, //use 0x23 0x13  compatibility with 1rank and 2rank //targeting rank 0. [3:0] is used //for write ODT [7:4] is used for //read ODT
	.dfi_odt_config			= 0x0d0d,  //use 0d0d compatibility with 1rank and 2rank  //0808
	.PllBypassEn			= 0, //bit0-ps0,bit1-ps1
	.ddr_rdbi_wr_enable		= 0,
	.clk_drv_ohm			= 40,
	.cs_drv_ohm				= 40,
	.ac_drv_ohm				= 40,
	.soc_data_drv_ohm_p		= 40,
	.soc_data_drv_ohm_n		= 40,
	.soc_data_odt_ohm_p		= 0,
	.soc_data_odt_ohm_n		= 120,
	.dram_data_drv_ohm		= 40, //lpddr4 sdram only240/1-6
	.dram_data_odt_ohm		= 120,
	.dram_ac_odt_ohm		= 120,
	.lpddr4_dram_vout_voltage_1_3_2_5_setting = 1,///1, 1/3vddq     0 2/5 vddq
	.soc_clk_slew_rate		= 0x3ff,//0x253,
	.soc_cs_slew_rate		= 0x100,//0x253,
	.soc_ac_slew_rate		= 0x100,//0x253,
	.soc_data_slew_rate		= 0x1ff,
	.vref_output_permil		= 350,//200,
	.vref_receiver_permil	= 0,
	.vref_dram_permil		= 0,
	//.vref_reverse			= 0,
	.ac_trace_delay			= {00,0x0,0,0,0,0,0x0,00},
	//.ac_trace_delay		= {32,32,32,32,32,32,32,32,32,32},
	.ac_pinmux				= {00,00},
	.ddr_dmc_remap			= {
							[0] = ( 5 |  6 << 5 |  7 << 10 |  8<< 15 | 9<< 20 | 10 << 25 ),
							[1] = ( 11|  0 << 5 |  0 << 10 | 15 << 15 | 16 << 20 | 17 << 25 ),
							[2] = ( 18| 19 << 5 | 20 << 10 | 21 << 15 | 22 << 20 | 23 << 25 ),
							[3] = ( 24| 25 << 5 | 26 << 10 | 27 << 15 | 28 << 20 | 29 << 25 ),
							[4] = ( 30| 12 << 5 | 13 << 10 |  14<< 15 |  0 << 20 |  0 << 25 ),
	},
	.ddr_lpddr34_ca_remap	= {00,00},
	.ddr_lpddr34_dq_remap	= {3,2,0,1,7,6,5,4, 10,9,14,11,8,12,13,15, 20,21,23,22,18,17,19,16, 28,26,25,24,31,30,27,29},
	.dram_rtt_nom_wr_park	= {00,00},

	/* pll ssc config:
	 *
	 *   pll_ssc_mode = (1<<20) | (1<<8) | ([strength] << 4) | [mode],
	 *      ppm = strength * 500
	 *      mode: 0=center, 1=up, 2=down
	 *
	 *   eg:
	 *     1. config 1000ppm center ss. then mode=0, strength=2
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (2 << 4) | 0,
	 *     2. config 3000ppm down ss. then mode=2, strength=6
	 *        .pll_ssc_mode = (1<<20) | (1<<8) | (6 << 4) | 2,
	 */
	.pll_ssc_mode			= (1<<20) | (1<<8) | (2<<4) | 0,//center_ssc_1000ppm
	.ddr_func				= DDR_FUNC,
	.magic					= DRAM_CFG_MAGIC,
	.diagnose				= CONFIG_DIAGNOSE_DISABLE,
},
};

pll_set_t __pll_setting = {
	.cpu_clk				= CONFIG_CPU_CLK / 24 * 24,
#ifdef CONFIG_PXP_EMULATOR
	.pxp					= 1,
#else
	.pxp					= 0,
#endif
	.spi_ctrl				= 0,
	.lCustomerID			= CONFIG_AML_CUSTOMER_ID,
#ifdef CONFIG_DEBUG_MODE
	.debug_mode				= CONFIG_DEBUG_MODE,
	.ddr_clk_debug			= CONFIG_DDR_CLK_DEBUG,
	.cpu_clk_debug			= CONFIG_CPU_CLK_DEBUG,
#endif
};

ddr_reg_t __ddr_reg[] = {
	/* demo, user defined override register */
	{0xaabbccdd, 0, 0, 0, 0, 0},
	{0x11223344, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0},
};

#define VCCK_VAL				CONFIG_VCCK_INIT_VOLTAGE
#define VDDEE_VAL				CONFIG_VDDEE_INIT_VOLTAGE
/* VCCK PWM table */
#if   (VCCK_VAL == 800)
	#define VCCK_VAL_REG	0x00150007
#elif (VCCK_VAL == 810)
	#define VCCK_VAL_REG	0x00140008
#elif (VCCK_VAL == 820)
	#define VCCK_VAL_REG	0x00130009
#elif (VCCK_VAL == 830)
	#define VCCK_VAL_REG	0x0012000a
#elif (VCCK_VAL == 840)
	#define VCCK_VAL_REG	0x0011000b
#elif (VCCK_VAL == 850)
	#define VCCK_VAL_REG	0x0010000c
#elif (VCCK_VAL == 860)
	#define VCCK_VAL_REG	0x000f000d
#elif (VCCK_VAL == 870)
	#define VCCK_VAL_REG	0x000e000e
#elif (VCCK_VAL == 880)
	#define VCCK_VAL_REG	0x000d000f
#elif (VCCK_VAL == 890)
	#define VCCK_VAL_REG	0x000c0010
#elif (VCCK_VAL == 900)
	#define VCCK_VAL_REG	0x000b0011
#elif (VCCK_VAL == 910)
	#define VCCK_VAL_REG	0x000a0012
#elif (VCCK_VAL == 920)
	#define VCCK_VAL_REG	0x00090013
#elif (VCCK_VAL == 930)
	#define VCCK_VAL_REG	0x00080014
#elif (VCCK_VAL == 940)
	#define VCCK_VAL_REG	0x00070015
#elif (VCCK_VAL == 950)
	#define VCCK_VAL_REG	0x00060016
#elif (VCCK_VAL == 960)
	#define VCCK_VAL_REG	0x00050017
#elif (VCCK_VAL == 970)
	#define VCCK_VAL_REG	0x00040018
#elif (VCCK_VAL == 980)
	#define VCCK_VAL_REG	0x00030019
#elif (VCCK_VAL == 990)
	#define VCCK_VAL_REG	0x0002001a
#elif (VCCK_VAL == 1000)
	#define VCCK_VAL_REG	0x0001001b
#elif (VCCK_VAL == 1010)
	#define VCCK_VAL_REG	0x0000001c
#else
	#error "VCCK val out of range\n"
#endif

/* VDDEE_VAL_REG0: VDDEE PWM table  0.67v-0.97v*/
/* VDDEE_VAL_REG1: VDDEE PWM table  0.69v-0.89v*/
#if    (VDDEE_VAL == 800)
	#define VDDEE_VAL_REG0	0x0010000c
	#define VDDEE_VAL_REG1	0x0008000a
#elif (VDDEE_VAL == 810)
	#define VDDEE_VAL_REG0	0x000f000d
	#define VDDEE_VAL_REG1	0x0007000b
#elif (VDDEE_VAL == 820)
	#define VDDEE_VAL_REG0	0x000e000e
	#define VDDEE_VAL_REG1	0x0006000c
#elif (VDDEE_VAL == 830)
	#define VDDEE_VAL_REG0	0x000d000f
	#define VDDEE_VAL_REG1	0x0005000d
#elif (VDDEE_VAL == 840)
	#define VDDEE_VAL_REG0	0x000c0010
	#define VDDEE_VAL_REG1	0x0004000e
#elif (VDDEE_VAL == 850)
	#define VDDEE_VAL_REG0	0x000b0011
	#define VDDEE_VAL_REG1	0x00030019
#elif (VDDEE_VAL == 860)
	#define VDDEE_VAL_REG0	0x000a0012
	#define VDDEE_VAL_REG1	0x00020010
#elif (VDDEE_VAL == 870)
	#define VDDEE_VAL_REG0	0x00090013
	#define VDDEE_VAL_REG1	0x00010011
#elif (VDDEE_VAL == 880)
	#define VDDEE_VAL_REG0	0x00080012
	#define VDDEE_VAL_REG1	0x00000012
#else
	#error "VDDEE val out of range\n"
#endif

/* for PWM use */
/* PWM driver check http://scgit.amlogic.com:8080/#/c/38093/ */
#define GPIO_O_EN_N_REG3    ((0xff634400 + (0x19 << 2)))
#define GPIO_O_REG3     ((0xff634400 + (0x1a << 2)))
#define GPIO_I_REG3     ((0xff634400 + (0x1b << 2)))
#define GPIO_O_EN_N_REG1    ((0xff634400 + (0x13 << 2)))
#define GPIO_O_REG1     ((0xff634400 + (0x14 << 2)))
#define AO_PIN_MUX_REG0 ((0xff800000 + (0x05 << 2)))
#define AO_PIN_MUX_REG1 ((0xff800000 + (0x06 << 2)))

bl2_reg_t __bl2_reg[] = {
	/* demo, user defined override register */
	/* eg: PWM init */

	/* PWM_AO_D */
	/* VCCK_VAL_REG: check PWM table */
	{AO_PWM_PWM_D,        VCCK_VAL_REG,            0xffffffff,   0, BL2_INIT_STAGE_1, 0},
	{AO_PWM_MISC_REG_CD,  ((1 << 23) | (1 << 1)),  (0x7f << 16), 0, BL2_INIT_STAGE_1, 0},
	{AO_PIN_MUX_REG1,     (3 << 20),               (0xF << 20),  0, BL2_INIT_STAGE_1, 0},

	/* set BOOT_9 input */
	//{PAD_PULL_UP_EN_REG0, 1 << 9,			1 << 9,   0, BL2_INIT_STAGE_1, 0},

	/* PWM_AO_B */
	/* VDDEE init start */
	/* step1: CHK HW */
	{(uint64_t)P_ASSIST_POR_CONFIG,  7,            0,            0, BL2_INIT_STAGE_PWM_CHK_HW,           0},

	/* step2: match PWM config */
	/* GPIO9[BIT7]=H use PWM_CFG0(0.67v-0.97v), =L use PWM_CFG1(0.69v-0.89v) */
	{0x1,                 PWM_CFG0,                0,            0, BL2_INIT_STAGE_PWM_CFG_GROUP,        0},
	{0x0,                 PWM_CFG1,                0,            0, BL2_INIT_STAGE_PWM_CFG_GROUP,        0},

	/* step3: config PWM */
	/* VDDEE_VAL_REG0: VDDEE PWM table  0.67v-0.97v*/
	{AO_PWM_PWM_B,        VDDEE_VAL_REG0,          0xffffffff,   0, BL2_INIT_STAGE_PWM_INIT | PWM_CFG0,  0},
	{AO_PWM_MISC_REG_AB,  ((1 << 23) | (1 << 1)),  (0x7f << 16), 0, BL2_INIT_STAGE_PWM_INIT | PWM_CFG0,  0},
	{AO_PIN_MUX_REG1,     (3 << 16),               (0xF << 16),  0, BL2_INIT_STAGE_PWM_INIT | PWM_CFG0,  0},
	/* VDDEE_VAL_REG1: VDDEE PWM table  0.69v-0.89v*/
	{AO_PWM_PWM_B,        VDDEE_VAL_REG1,          0xffffffff,   0, BL2_INIT_STAGE_PWM_INIT | PWM_CFG1,  0},
	{AO_PWM_MISC_REG_AB,  ((1 << 23) | (1 << 1)),  (0x7f << 16), 0, BL2_INIT_STAGE_PWM_INIT | PWM_CFG1,  0},
	{AO_PIN_MUX_REG1,     (3 << 16),               (0xF << 16),  0, BL2_INIT_STAGE_PWM_INIT | PWM_CFG1,  0},
	/* VDDEE init done */

	/* Enable 5V_EN */
	{GPIO_O_EN_N_REG3,    (1 << 8),                (1 << 8),     0, BL2_INIT_STAGE_1, 0},
	{GPIO_O_REG3,         (1 << 8),                (1 << 8),   0, BL2_INIT_STAGE_1, 0},
	/* Enable CPUA ,control by GPIOC_7 */
	{GPIO_O_EN_N_REG1,    (1 << 7),                (1 << 7),     0, BL2_INIT_STAGE_1, 0},
	{GPIO_O_REG1,         (1 << 7),                (1 << 7),   0, BL2_INIT_STAGE_1, 0},
	/* Enable VCCK */
	{AO_SEC_REG0,         (1 << 0),                (1 << 0),     0, BL2_INIT_STAGE_1, 0},
	{AO_GPIO_O,           (1 << 31),               (1 << 31),    0, BL2_INIT_STAGE_1, 0},
	/* Init sys led*/
	{AO_GPIO_O_EN_N,      (0 << 4),               (1 << 4),    0, BL2_INIT_STAGE_1, 0},
	{AO_GPIO_O,           (1 << 4),               (1 << 4),    0, BL2_INIT_STAGE_1, 0},
};
