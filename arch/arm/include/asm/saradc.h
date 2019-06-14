/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/

#ifndef __AML_SARADC_H__
#define __AML_SARADC_H__

enum{AML_ADC_CHAN_0 = 0, AML_ADC_CHAN_1, AML_ADC_CHAN_2, AML_ADC_CHAN_3,
	 AML_ADC_CHAN_4,	 AML_ADC_CHAN_5, AML_ADC_CHAN_6, AML_ADC_CHAN_7,
	 AML_ADC_SARADC_CHAN_NUM,
};

enum{AML_ADC_NO_AVG = 0,  AML_ADC_SIMPLE_AVG_1, AML_ADC_SIMPLE_AVG_2,
	 AML_ADC_SIMPLE_AVG_4,AML_ADC_SIMPLE_AVG_8, AML_ADC_MEDIAN_AVG_8,
};

#define AML_ADC_CHAN_XP	AML_ADC_CHAN_0
#define AML_ADC_CHAN_YP	AML_ADC_CHAN_1
#define AML_ADC_CHAN_XN	AML_ADC_CHAN_2
#define AML_ADC_CHAN_YN	AML_ADC_CHAN_3

int saradc_probe(void);
int saradc_enable(void);
int saradc_disable(void);
int get_adc_sample_gxbb(int chan);
int get_adc_sample_gxbb_12bit(int chan);
void saradc_sample_test(void);

#endif /*__AML_SARADC_H__*/
