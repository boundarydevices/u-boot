/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */

#ifndef _MU_H_
#define _MU_H_
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __MWERKS__
#pragma push
#pragma ANSI_strict off
#endif
#ifdef __ghs__
#pragma ghs nowarning 618
#endif
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <stdint.h>
typedef volatile unsigned int vuint32_t;

/****************************************************************************/
/*                              MODULE: MU                    */
/****************************************************************************/

/***************************Configuration Registers**************************/



typedef union MU_VER_union_tag {       /* VER Register */

  vuint32_t R;
  struct{
    vuint32_t FEATURE   :16; // FEATURE Number
    vuint32_t MINOR     : 8; // MINOR Number
    vuint32_t MAJOR     : 8; // MAJOR Number
  } B;
} MU_VER_tag;


typedef union MU_PAR_union_tag {       /* PAR Register */

  vuint32_t R;
  struct{
    vuint32_t TR_NUM    : 8; // TR_NUM Number
    vuint32_t RR_NUM    : 8; // RR_NUM Number
    vuint32_t GIR_NUM   : 8; // GIR_NUM Number
    vuint32_t FLAG_WIDTH: 8; // FLAG_WIDTH Number
  } B;
} MU_PAR_tag;


typedef union MU_CR_union_tag {       /* CR Register */

  vuint32_t R;
  struct{
    vuint32_t MUR       : 1; // MUR Number
    vuint32_t MURIE     : 1; // MURIE Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_CR_tag;


typedef union MU_SR_union_tag {       /* SR Register */

  vuint32_t R;
  struct{
    vuint32_t MURS      : 1; // MURS Number
    vuint32_t MURIP     : 1; // MURIP Number
    vuint32_t EP        : 1; // EP Number
    vuint32_t FUP       : 1; // FUP Number
    vuint32_t GIRP      : 1; // GIRP Number
    vuint32_t TEP       : 1; // TEP Number
    vuint32_t RFP       : 1; // RFP Number
    vuint32_t CEP       : 1; // CEP Number
    vuint32_t rsrv_1    :24; // rsrv_1 Number
  } B;
} MU_SR_tag;


typedef union MU_CCR0_union_tag {       /* CCR0 Register */

  vuint32_t R;
  struct{
    vuint32_t NMI       : 1; // NMI Number
    vuint32_t HR        : 1; // HR Number
    vuint32_t HRM       : 1; // HRM Number
    vuint32_t CLKE      : 1; // CLKE Number
    vuint32_t RSTH      : 1; // RSTH Number
    vuint32_t BOOT      : 2; // BOOT Number
    vuint32_t rsrv_1    :25; // rsrv_1 Number
  } B;
} MU_CCR0_tag;


typedef union MU_CIER0_union_tag {       /* CIER0 Register */

  vuint32_t R;
  struct{
    vuint32_t rsrv_1    : 1; // rsrv_1 Number
    vuint32_t HRIE      : 1; // HRIE Number
    vuint32_t RUNIE     : 1; // RUNIE Number
    vuint32_t RAIE      : 1; // RAIE Number
    vuint32_t HALTIE    : 1; // HALTIE Number
    vuint32_t WAITIE    : 1; // WAITIE Number
    vuint32_t STOPIE    : 1; // STOPIE Number
    vuint32_t PDIE      : 1; // PDIE Number
    vuint32_t rsrv_2    :24; // rsrv_2 Number
  } B;
} MU_CIER0_tag;


typedef union MU_CSSR0_union_tag {       /* CSSR0 Register */

  vuint32_t R;
  struct{
    vuint32_t NMIC      : 1; // NMIC Number
    vuint32_t HRIP      : 1; // HRIP Number
    vuint32_t RUN       : 1; // RUN Number
    vuint32_t RAIP      : 1; // RAIP Number
    vuint32_t HALT      : 1; // HALT Number
    vuint32_t WAIT      : 1; // WAIT Number
    vuint32_t STOP      : 1; // STOP Number
    vuint32_t PD        : 1; // PD Number
    vuint32_t rsrv_1    :24; // rsrv_1 Number
  } B;
} MU_CSSR0_tag;


typedef union MU_CSR0_union_tag {       /* CSR0 Register */

  vuint32_t R;
  struct{
    vuint32_t rsrv_1    : 1; // rsrv_1 Number
    vuint32_t HRIP      : 1; // HRIP Number
    vuint32_t RUN       : 1; // RUN Number
    vuint32_t RAIP      : 1; // RAIP Number
    vuint32_t HALT      : 1; // HALT Number
    vuint32_t WAIT      : 1; // WAIT Number
    vuint32_t STOP      : 1; // STOP Number
    vuint32_t PD        : 1; // PD Number
    vuint32_t rsrv_2    :24; // rsrv_2 Number
  } B;
} MU_CSR0_tag;


typedef union MU_FCR_union_tag {       /* FCR Register */

  vuint32_t R;
  struct{
    vuint32_t F0        : 1; // F0 Number
    vuint32_t F1        : 1; // F1 Number
    vuint32_t F2        : 1; // F2 Number
    vuint32_t rsrv_1    :29; // rsrv_1 Number
  } B;
} MU_FCR_tag;


typedef union MU_FSR_union_tag {       /* FSR Register */

  vuint32_t R;
  struct{
    vuint32_t F0        : 1; // F0 Number
    vuint32_t F1        : 1; // F1 Number
    vuint32_t F2        : 1; // F2 Number
    vuint32_t rsrv_1    :29; // rsrv_1 Number
  } B;
} MU_FSR_tag;


typedef union MU_GIER_union_tag {       /* GIER Register */

  vuint32_t R;
  struct{
    vuint32_t GIE0      : 1; // GIE0 Number
    vuint32_t GIE1      : 1; // GIE1 Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_GIER_tag;


typedef union MU_GCR_union_tag {       /* GCR Register */

  vuint32_t R;
  struct{
    vuint32_t GIR0      : 1; // GIR0 Number
    vuint32_t GIR1      : 1; // GIR1 Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_GCR_tag;


typedef union MU_GSR_union_tag {       /* GSR Register */

  vuint32_t R;
  struct{
    vuint32_t GIP0      : 1; // GIP0 Number
    vuint32_t GIP1      : 1; // GIP1 Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_GSR_tag;


typedef union MU_TCR_union_tag {       /* TCR Register */

  vuint32_t R;
  struct{
    vuint32_t TIE0      : 1; // TIE0 Number
    vuint32_t TIE1      : 1; // TIE1 Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_TCR_tag;


typedef union MU_TSR_union_tag {       /* TSR Register */

  vuint32_t R;
  struct{
    vuint32_t TE0       : 1; // TE0 Number
    vuint32_t TE1       : 1; // TE1 Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_TSR_tag;


typedef union MU_RCR_union_tag {       /* RCR Register */

  vuint32_t R;
  struct{
    vuint32_t RIE0      : 1; // RIE0 Number
    vuint32_t RIE1      : 1; // RIE1 Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_RCR_tag;


typedef union MU_RSR_union_tag {       /* RSR Register */

  vuint32_t R;
  struct{
    vuint32_t RF0       : 1; // RF0 Number
    vuint32_t RF1       : 1; // RF1 Number
    vuint32_t rsrv_1    :30; // rsrv_1 Number
  } B;
} MU_RSR_tag;


typedef union MU_TR0_union_tag {       /* TR0 Register */

  vuint32_t R;
  struct{
    vuint32_t TR_DATA   :32; // TR_DATA Number
  } B;
} MU_TR0_tag;


typedef union MU_TR1_union_tag {       /* TR1 Register */

  vuint32_t R;
  struct{
    vuint32_t TR_DATA   :32; // TR_DATA Number
  } B;
} MU_TR1_tag;


typedef union MU_RR0_union_tag {       /* RR0 Register */

  vuint32_t R;
  struct{
    vuint32_t RR_DATA   :32; // RR_DATA Number
  } B;
} MU_RR0_tag;


typedef union MU_RR1_union_tag {       /* RR1 Register */

  vuint32_t R;
  struct{
    vuint32_t RR_DATA   :32; // RR_DATA Number
  } B;
} MU_RR1_tag;



struct MU_tag  {


  MU_VER_tag              VER       ;      //VER Register
  MU_PAR_tag              PAR       ;      //PAR Register
  MU_CR_tag               CR        ;      //CR Register
  MU_SR_tag               SR        ;      //SR Register
  MU_CCR0_tag             CCR0      ;      //CCR0 Register
  MU_CIER0_tag            CIER0     ;      //CIER0 Register
  MU_CSSR0_tag            CSSR0     ;      //CSSR0 Register
  MU_CSR0_tag             CSR0      ;      //CSR0 Register
  uint8_t MU_reserved0[224];
  MU_FCR_tag              FCR       ;      //FCR Register
  MU_FSR_tag              FSR       ;      //FSR Register
  uint8_t MU_reserved1[8];
  MU_GIER_tag             GIER      ;      //GIER Register
  MU_GCR_tag              GCR       ;      //GCR Register
  MU_GSR_tag              GSR       ;      //GSR Register
  uint8_t MU_reserved2[4];
  MU_TCR_tag              TCR       ;      //TCR Register
  MU_TSR_tag              TSR       ;      //TSR Register
  MU_RCR_tag              RCR       ;      //RCR Register
  MU_RSR_tag              RSR       ;      //RSR Register
  uint8_t MU_reserved3[208];
  MU_TR0_tag              TR[2]       ;      //TR0 Register
  //MU_TR1_tag              TR1       ;      //TR1 Register
  uint8_t MU_reserved4[120];
  MU_RR0_tag              RR[2]       ;      //RR0 Register
  //MU_RR1_tag              RR1       ;      //RR1 Register

};



#ifdef __MWERKS__
#pragma pop
#endif
#ifdef __ghs__
#pragma ghs endnowarning
#endif
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#ifdef  __cplusplus
}
#endif
#endif /* ifdef _MU_H_ */

