/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2017-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE SOFTWARE IS PROVIDED "AS IS",
 * WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 *
 * API_AFE.h
 *
 ******************************************************************************
 */

#ifndef API_AFE_H_
#define API_AFE_H_
#include "util.h"

typedef enum {
	AFE_LINK_RATE_1_6 = 0x6,  /* 1.62 Gb/s */
	AFE_LINK_RATE_2_1 = 0x8,  /* 2.16 Gb/s */
	AFE_LINK_RATE_2_4 = 0x9,  /* 2.43 Gb/s */
	AFE_LINK_RATE_2_7 = 0xA,  /* 2.70 Gb/s */
	AFE_LINK_RATE_3_2 = 0xC,  /* 3.24 Gb/s */
	AFE_LINK_RATE_4_3 = 0x10, /* 4.32 Gb/s */
	AFE_LINK_RATE_5_4 = 0x14, /* 5.40 Gb/s */
	AFE_LINK_RATE_8_1 = 0x1E, /* 8.10 Gb/s */
} ENUM_AFE_LINK_RATE;

/* Some of the PHY programming sequences */
/* depend on the reference clock frequency. */
/* Variable of this type is used to control */
/* the programming flow. */
typedef enum {
	REFCLK_24MHZ,
	REFCLK_27MHZ
} REFCLK_FREQ;

typedef enum {
	CLK_RATIO_1_1,
	CLK_RATIO_5_4,
	CLK_RATIO_3_2,
	CLK_RATIO_2_1,
	CLK_RATIO_1_2,
	CLK_RATIO_5_8,
	CLK_RATIO_3_4
} clk_ratio_t;

typedef struct {
	u32 value;
	u8 lsb;
	u8 msb;
} reg_field_t;

unsigned char AFE_check_rate_supported(ENUM_AFE_LINK_RATE rate);
void afe_write(unsigned int offset, unsigned short val);
unsigned short afe_read(unsigned int offset);
void AFE_init(int num_lanes, ENUM_AFE_LINK_RATE link_rate);
void AFE_power(int num_lanes, ENUM_AFE_LINK_RATE link_rate);

/*extern int cdn_phapb_read(unsigned int addr, unsigned int *value);*/
/*extern int cdn_phapb_write(unsigned int addr, unsigned int value);*/
void set_field_value(reg_field_t *reg_field, u32 value);
int set_reg_value(reg_field_t reg_field);

#endif

