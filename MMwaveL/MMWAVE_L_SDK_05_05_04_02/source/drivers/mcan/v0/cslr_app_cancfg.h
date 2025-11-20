/********************************************************************
 * Copyright (C) 2022-23 Texas Instruments Incorporated.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Name        : cslr_app_cancfg.h
*/
#ifndef CSLR_APP_CANCFG_H_
#define CSLR_APP_CANCFG_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t SS_PID;
    volatile uint32_t SS_CTRL;
    volatile uint32_t SS_STAT;
    volatile uint32_t SS_ICS;
    volatile uint32_t SS_IRS;
    volatile uint32_t SS_IECS;
    volatile uint32_t SS_IE;
    volatile uint32_t SS_IES;
    volatile uint32_t SS_EOI;
    volatile uint32_t SS_EXT_TS_PS;
    volatile uint32_t SS_EXT_TS_USIC;
    volatile uint8_t  Resv_512[468];
    volatile uint32_t CREL;
    volatile uint32_t ENDN;
    volatile uint32_t CUST;
    volatile uint32_t DBTP;
    volatile uint32_t TEST;
    volatile uint32_t RWD;
    volatile uint32_t CCCR;
    volatile uint32_t NBTP;
    volatile uint32_t TSCC;
    volatile uint32_t TSCV;
    volatile uint32_t TOCC;
    volatile uint32_t TOCV;
    volatile uint32_t RES00;
    volatile uint32_t RES01;
    volatile uint32_t RES02;
    volatile uint32_t RES03;
    volatile uint32_t ECR;
    volatile uint32_t PSR;
    volatile uint32_t TDCR;
    volatile uint32_t RES04;
    volatile uint32_t IR;
    volatile uint32_t IE;
    volatile uint32_t ILS;
    volatile uint32_t ILE;
    volatile uint32_t RES05;
    volatile uint32_t RES06;
    volatile uint32_t RES07;
    volatile uint32_t RES08;
    volatile uint32_t RES09;
    volatile uint32_t RES10;
    volatile uint32_t RES11;
    volatile uint32_t RES12;
    volatile uint32_t GFC;
    volatile uint32_t SIDFC;
    volatile uint32_t XIDFC;
    volatile uint32_t RES13;
    volatile uint32_t XIDAM;
    volatile uint32_t HPMS;
    volatile uint32_t NDAT1;
    volatile uint32_t NDAT2;
    volatile uint32_t RXF0C;
    volatile uint32_t RXF0S;
    volatile uint32_t RXF0A;
    volatile uint32_t RXBC;
    volatile uint32_t RXF1C;
    volatile uint32_t RXF1S;
    volatile uint32_t RXF1A;
    volatile uint32_t RXESC;
    volatile uint32_t TXBC;
    volatile uint32_t TXFQS;
    volatile uint32_t TXESC;
    volatile uint32_t TXBRP;
    volatile uint32_t TXBAR;
    volatile uint32_t TXBCR;
    volatile uint32_t TXBTO;
    volatile uint32_t TXBCF;
    volatile uint32_t TXBTIE;
    volatile uint32_t TXBCIE;
    volatile uint32_t RES14;
    volatile uint32_t RES15;
    volatile uint32_t TXEFC;
    volatile uint32_t TXEFS;
    volatile uint32_t TXEFA;
    volatile uint32_t RES16;
} CSL_app_cancfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_APP_CANCFG_SS_PID                                                  (0x00000000U)
#define CSL_APP_CANCFG_SS_CTRL                                                 (0x00000004U)
#define CSL_APP_CANCFG_SS_STAT                                                 (0x00000008U)
#define CSL_APP_CANCFG_SS_ICS                                                  (0x0000000CU)
#define CSL_APP_CANCFG_SS_IRS                                                  (0x00000010U)
#define CSL_APP_CANCFG_SS_IECS                                                 (0x00000014U)
#define CSL_APP_CANCFG_SS_IE                                                   (0x00000018U)
#define CSL_APP_CANCFG_SS_IES                                                  (0x0000001CU)
#define CSL_APP_CANCFG_SS_EOI                                                  (0x00000020U)
#define CSL_APP_CANCFG_SS_EXT_TS_PS                                            (0x00000024U)
#define CSL_APP_CANCFG_SS_EXT_TS_USIC                                          (0x00000028U)
#define CSL_APP_CANCFG_CREL                                                    (0x00000200U)
#define CSL_APP_CANCFG_ENDN                                                    (0x00000204U)
#define CSL_APP_CANCFG_CUST                                                    (0x00000208U)
#define CSL_APP_CANCFG_DBTP                                                    (0x0000020CU)
#define CSL_APP_CANCFG_TEST                                                    (0x00000210U)
#define CSL_APP_CANCFG_RWD                                                     (0x00000214U)
#define CSL_APP_CANCFG_CCCR                                                    (0x00000218U)
#define CSL_APP_CANCFG_NBTP                                                    (0x0000021CU)
#define CSL_APP_CANCFG_TSCC                                                    (0x00000220U)
#define CSL_APP_CANCFG_TSCV                                                    (0x00000224U)
#define CSL_APP_CANCFG_TOCC                                                    (0x00000228U)
#define CSL_APP_CANCFG_TOCV                                                    (0x0000022CU)
#define CSL_APP_CANCFG_RES00                                                   (0x00000230U)
#define CSL_APP_CANCFG_RES01                                                   (0x00000234U)
#define CSL_APP_CANCFG_RES02                                                   (0x00000238U)
#define CSL_APP_CANCFG_RES03                                                   (0x0000023CU)
#define CSL_APP_CANCFG_ECR                                                     (0x00000240U)
#define CSL_APP_CANCFG_PSR                                                     (0x00000244U)
#define CSL_APP_CANCFG_TDCR                                                    (0x00000248U)
#define CSL_APP_CANCFG_RES04                                                   (0x0000024CU)
#define CSL_APP_CANCFG_IR                                                      (0x00000250U)
#define CSL_APP_CANCFG_IE                                                      (0x00000254U)
#define CSL_APP_CANCFG_ILS                                                     (0x00000258U)
#define CSL_APP_CANCFG_ILE                                                     (0x0000025CU)
#define CSL_APP_CANCFG_RES05                                                   (0x00000260U)
#define CSL_APP_CANCFG_RES06                                                   (0x00000264U)
#define CSL_APP_CANCFG_RES07                                                   (0x00000268U)
#define CSL_APP_CANCFG_RES08                                                   (0x0000026CU)
#define CSL_APP_CANCFG_RES09                                                   (0x00000270U)
#define CSL_APP_CANCFG_RES10                                                   (0x00000274U)
#define CSL_APP_CANCFG_RES11                                                   (0x00000278U)
#define CSL_APP_CANCFG_RES12                                                   (0x0000027CU)
#define CSL_APP_CANCFG_GFC                                                     (0x00000280U)
#define CSL_APP_CANCFG_SIDFC                                                   (0x00000284U)
#define CSL_APP_CANCFG_XIDFC                                                   (0x00000288U)
#define CSL_APP_CANCFG_RES13                                                   (0x0000028CU)
#define CSL_APP_CANCFG_XIDAM                                                   (0x00000290U)
#define CSL_APP_CANCFG_HPMS                                                    (0x00000294U)
#define CSL_APP_CANCFG_NDAT1                                                   (0x00000298U)
#define CSL_APP_CANCFG_NDAT2                                                   (0x0000029CU)
#define CSL_APP_CANCFG_RXF0C                                                   (0x000002A0U)
#define CSL_APP_CANCFG_RXF0S                                                   (0x000002A4U)
#define CSL_APP_CANCFG_RXF0A                                                   (0x000002A8U)
#define CSL_APP_CANCFG_RXBC                                                    (0x000002ACU)
#define CSL_APP_CANCFG_RXF1C                                                   (0x000002B0U)
#define CSL_APP_CANCFG_RXF1S                                                   (0x000002B4U)
#define CSL_APP_CANCFG_RXF1A                                                   (0x000002B8U)
#define CSL_APP_CANCFG_RXESC                                                   (0x000002BCU)
#define CSL_APP_CANCFG_TXBC                                                    (0x000002C0U)
#define CSL_APP_CANCFG_TXFQS                                                   (0x000002C4U)
#define CSL_APP_CANCFG_TXESC                                                   (0x000002C8U)
#define CSL_APP_CANCFG_TXBRP                                                   (0x000002CCU)
#define CSL_APP_CANCFG_TXBAR                                                   (0x000002D0U)
#define CSL_APP_CANCFG_TXBCR                                                   (0x000002D4U)
#define CSL_APP_CANCFG_TXBTO                                                   (0x000002D8U)
#define CSL_APP_CANCFG_TXBCF                                                   (0x000002DCU)
#define CSL_APP_CANCFG_TXBTIE                                                  (0x000002E0U)
#define CSL_APP_CANCFG_TXBCIE                                                  (0x000002E4U)
#define CSL_APP_CANCFG_RES14                                                   (0x000002E8U)
#define CSL_APP_CANCFG_RES15                                                   (0x000002ECU)
#define CSL_APP_CANCFG_TXEFC                                                   (0x000002F0U)
#define CSL_APP_CANCFG_TXEFS                                                   (0x000002F4U)
#define CSL_APP_CANCFG_TXEFA                                                   (0x000002F8U)
#define CSL_APP_CANCFG_RES16                                                   (0x000002FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* SS_PID */

#define CSL_APP_CANCFG_SS_PID_MINOR_MASK                                       (0x0000003FU)
#define CSL_APP_CANCFG_SS_PID_MINOR_SHIFT                                      (0x00000000U)
#define CSL_APP_CANCFG_SS_PID_MINOR_RESETVAL                                   (0x00000001U)
#define CSL_APP_CANCFG_SS_PID_MINOR_MAX                                        (0x0000003FU)

#define CSL_APP_CANCFG_SS_PID_CUSTOM_MASK                                      (0x000000C0U)
#define CSL_APP_CANCFG_SS_PID_CUSTOM_SHIFT                                     (0x00000006U)
#define CSL_APP_CANCFG_SS_PID_CUSTOM_RESETVAL                                  (0x00000000U)
#define CSL_APP_CANCFG_SS_PID_CUSTOM_MAX                                       (0x00000003U)

#define CSL_APP_CANCFG_SS_PID_MAJOR_MASK                                       (0x00000700U)
#define CSL_APP_CANCFG_SS_PID_MAJOR_SHIFT                                      (0x00000008U)
#define CSL_APP_CANCFG_SS_PID_MAJOR_RESETVAL                                   (0x00000001U)
#define CSL_APP_CANCFG_SS_PID_MAJOR_MAX                                        (0x00000007U)

#define CSL_APP_CANCFG_SS_PID_RTL_MASK                                         (0x0000F800U)
#define CSL_APP_CANCFG_SS_PID_RTL_SHIFT                                        (0x0000000BU)
#define CSL_APP_CANCFG_SS_PID_RTL_RESETVAL                                     (0x0000000BU)
#define CSL_APP_CANCFG_SS_PID_RTL_MAX                                          (0x0000001FU)

#define CSL_APP_CANCFG_SS_PID_MODULE_ID_MASK                                   (0x0FFF0000U)
#define CSL_APP_CANCFG_SS_PID_MODULE_ID_SHIFT                                  (0x00000010U)
#define CSL_APP_CANCFG_SS_PID_MODULE_ID_RESETVAL                               (0x000008E0U)
#define CSL_APP_CANCFG_SS_PID_MODULE_ID_MAX                                    (0x00000FFFU)

#define CSL_APP_CANCFG_SS_PID_BU_MASK                                          (0x30000000U)
#define CSL_APP_CANCFG_SS_PID_BU_SHIFT                                         (0x0000001CU)
#define CSL_APP_CANCFG_SS_PID_BU_RESETVAL                                      (0x00000002U)
#define CSL_APP_CANCFG_SS_PID_BU_MAX                                           (0x00000003U)

#define CSL_APP_CANCFG_SS_PID_SCHEME_MASK                                      (0xC0000000U)
#define CSL_APP_CANCFG_SS_PID_SCHEME_SHIFT                                     (0x0000001EU)
#define CSL_APP_CANCFG_SS_PID_SCHEME_RESETVAL                                  (0x00000001U)
#define CSL_APP_CANCFG_SS_PID_SCHEME_MAX                                       (0x00000003U)

#define CSL_APP_CANCFG_SS_PID_RESETVAL                                         (0x68E05901U)

/* SS_CTRL */

#define CSL_APP_CANCFG_SS_CTRL_NU_MASK                                         (0x00000007U)
#define CSL_APP_CANCFG_SS_CTRL_NU_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_SS_CTRL_NU_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_CTRL_NU_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_SS_CTRL_DBGSUSP_FREE_MASK                               (0x00000008U)
#define CSL_APP_CANCFG_SS_CTRL_DBGSUSP_FREE_SHIFT                              (0x00000003U)
#define CSL_APP_CANCFG_SS_CTRL_DBGSUSP_FREE_RESETVAL                           (0x00000001U)
#define CSL_APP_CANCFG_SS_CTRL_DBGSUSP_FREE_MAX                                (0x00000001U)

#define CSL_APP_CANCFG_SS_CTRL_WAKEUPREGEN_MASK                                (0x00000010U)
#define CSL_APP_CANCFG_SS_CTRL_WAKEUPREGEN_SHIFT                               (0x00000004U)
#define CSL_APP_CANCFG_SS_CTRL_WAKEUPREGEN_RESETVAL                            (0x00000000U)
#define CSL_APP_CANCFG_SS_CTRL_WAKEUPREGEN_MAX                                 (0x00000001U)

#define CSL_APP_CANCFG_SS_CTRL_AUTOWAKEUP_MASK                                 (0x00000020U)
#define CSL_APP_CANCFG_SS_CTRL_AUTOWAKEUP_SHIFT                                (0x00000005U)
#define CSL_APP_CANCFG_SS_CTRL_AUTOWAKEUP_RESETVAL                             (0x00000000U)
#define CSL_APP_CANCFG_SS_CTRL_AUTOWAKEUP_MAX                                  (0x00000001U)

#define CSL_APP_CANCFG_SS_CTRL_EXT_TS_CNTR_EN_MASK                             (0x00000040U)
#define CSL_APP_CANCFG_SS_CTRL_EXT_TS_CNTR_EN_SHIFT                            (0x00000006U)
#define CSL_APP_CANCFG_SS_CTRL_EXT_TS_CNTR_EN_RESETVAL                         (0x00000000U)
#define CSL_APP_CANCFG_SS_CTRL_EXT_TS_CNTR_EN_MAX                              (0x00000001U)

#define CSL_APP_CANCFG_SS_CTRL_NU0_MASK                                        (0xFFFFFF80U)
#define CSL_APP_CANCFG_SS_CTRL_NU0_SHIFT                                       (0x00000007U)
#define CSL_APP_CANCFG_SS_CTRL_NU0_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_SS_CTRL_NU0_MAX                                         (0x01FFFFFFU)

#define CSL_APP_CANCFG_SS_CTRL_RESETVAL                                        (0x00000008U)

/* SS_STAT */

#define CSL_APP_CANCFG_SS_STAT_NU_MASK                                         (0x00000001U)
#define CSL_APP_CANCFG_SS_STAT_NU_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_SS_STAT_NU_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_STAT_NU_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_SS_STAT_MMI_DONE_MASK                                   (0x00000002U)
#define CSL_APP_CANCFG_SS_STAT_MMI_DONE_SHIFT                                  (0x00000001U)
#define CSL_APP_CANCFG_SS_STAT_MMI_DONE_RESETVAL                               (0x00000001U)
#define CSL_APP_CANCFG_SS_STAT_MMI_DONE_MAX                                    (0x00000001U)

#define CSL_APP_CANCFG_SS_STAT_EN_FDOE_MASK                                    (0x00000004U)
#define CSL_APP_CANCFG_SS_STAT_EN_FDOE_SHIFT                                   (0x00000002U)
#define CSL_APP_CANCFG_SS_STAT_EN_FDOE_RESETVAL                                (0x00000001U)
#define CSL_APP_CANCFG_SS_STAT_EN_FDOE_MAX                                     (0x00000001U)

#define CSL_APP_CANCFG_SS_STAT_NU1_MASK                                        (0xFFFFFFF8U)
#define CSL_APP_CANCFG_SS_STAT_NU1_SHIFT                                       (0x00000003U)
#define CSL_APP_CANCFG_SS_STAT_NU1_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_SS_STAT_NU1_MAX                                         (0x1FFFFFFFU)

#define CSL_APP_CANCFG_SS_STAT_RESETVAL                                        (0x00000006U)

/* SS_ICS */

#define CSL_APP_CANCFG_SS_ICS_ICS_MASK                                         (0x00000001U)
#define CSL_APP_CANCFG_SS_ICS_ICS_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_SS_ICS_ICS_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_ICS_ICS_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_SS_ICS_NU2_MASK                                         (0xFFFFFFFEU)
#define CSL_APP_CANCFG_SS_ICS_NU2_SHIFT                                        (0x00000001U)
#define CSL_APP_CANCFG_SS_ICS_NU2_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_ICS_NU2_MAX                                          (0x7FFFFFFFU)

#define CSL_APP_CANCFG_SS_ICS_RESETVAL                                         (0x00000000U)

/* SS_IRS */

#define CSL_APP_CANCFG_SS_IRS_IRS_MASK                                         (0x00000001U)
#define CSL_APP_CANCFG_SS_IRS_IRS_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_SS_IRS_IRS_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_IRS_IRS_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_SS_IRS_NU3_MASK                                         (0xFFFFFFFEU)
#define CSL_APP_CANCFG_SS_IRS_NU3_SHIFT                                        (0x00000001U)
#define CSL_APP_CANCFG_SS_IRS_NU3_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_IRS_NU3_MAX                                          (0x7FFFFFFFU)

#define CSL_APP_CANCFG_SS_IRS_RESETVAL                                         (0x00000000U)

/* SS_IECS */

#define CSL_APP_CANCFG_SS_IECS_IECS_MASK                                       (0x00000001U)
#define CSL_APP_CANCFG_SS_IECS_IECS_SHIFT                                      (0x00000000U)
#define CSL_APP_CANCFG_SS_IECS_IECS_RESETVAL                                   (0x00000000U)
#define CSL_APP_CANCFG_SS_IECS_IECS_MAX                                        (0x00000001U)

#define CSL_APP_CANCFG_SS_IECS_NU4_MASK                                        (0xFFFFFFFEU)
#define CSL_APP_CANCFG_SS_IECS_NU4_SHIFT                                       (0x00000001U)
#define CSL_APP_CANCFG_SS_IECS_NU4_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_SS_IECS_NU4_MAX                                         (0x7FFFFFFFU)

#define CSL_APP_CANCFG_SS_IECS_RESETVAL                                        (0x00000000U)

/* SS_IE */

#define CSL_APP_CANCFG_SS_IE_IE_MASK                                           (0x00000001U)
#define CSL_APP_CANCFG_SS_IE_IE_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_SS_IE_IE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_SS_IE_IE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_SS_IE_NU5_MASK                                          (0xFFFFFFFEU)
#define CSL_APP_CANCFG_SS_IE_NU5_SHIFT                                         (0x00000001U)
#define CSL_APP_CANCFG_SS_IE_NU5_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_SS_IE_NU5_MAX                                           (0x7FFFFFFFU)

#define CSL_APP_CANCFG_SS_IE_RESETVAL                                          (0x00000000U)

/* SS_IES */

#define CSL_APP_CANCFG_SS_IES_IES_MASK                                         (0x00000001U)
#define CSL_APP_CANCFG_SS_IES_IES_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_SS_IES_IES_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_IES_IES_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_SS_IES_NU6_MASK                                         (0xFFFFFFFEU)
#define CSL_APP_CANCFG_SS_IES_NU6_SHIFT                                        (0x00000001U)
#define CSL_APP_CANCFG_SS_IES_NU6_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_IES_NU6_MAX                                          (0x7FFFFFFFU)

#define CSL_APP_CANCFG_SS_IES_RESETVAL                                         (0x00000000U)

/* SS_EOI */

#define CSL_APP_CANCFG_SS_EOI_EOI_MASK                                         (0x000000FFU)
#define CSL_APP_CANCFG_SS_EOI_EOI_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_SS_EOI_EOI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_EOI_EOI_MAX                                          (0x000000FFU)

#define CSL_APP_CANCFG_SS_EOI_NU7_MASK                                         (0xFFFFFF00U)
#define CSL_APP_CANCFG_SS_EOI_NU7_SHIFT                                        (0x00000008U)
#define CSL_APP_CANCFG_SS_EOI_NU7_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SS_EOI_NU7_MAX                                          (0x00FFFFFFU)

#define CSL_APP_CANCFG_SS_EOI_RESETVAL                                         (0x00000000U)

/* SS_EXT_TS_PS */

#define CSL_APP_CANCFG_SS_EXT_TS_PS_PRESCALE_MASK                              (0x00FFFFFFU)
#define CSL_APP_CANCFG_SS_EXT_TS_PS_PRESCALE_SHIFT                             (0x00000000U)
#define CSL_APP_CANCFG_SS_EXT_TS_PS_PRESCALE_RESETVAL                          (0x00000000U)
#define CSL_APP_CANCFG_SS_EXT_TS_PS_PRESCALE_MAX                               (0x00FFFFFFU)

#define CSL_APP_CANCFG_SS_EXT_TS_PS_NU8_MASK                                   (0xFF000000U)
#define CSL_APP_CANCFG_SS_EXT_TS_PS_NU8_SHIFT                                  (0x00000018U)
#define CSL_APP_CANCFG_SS_EXT_TS_PS_NU8_RESETVAL                               (0x00000000U)
#define CSL_APP_CANCFG_SS_EXT_TS_PS_NU8_MAX                                    (0x000000FFU)

#define CSL_APP_CANCFG_SS_EXT_TS_PS_RESETVAL                                   (0x00000000U)

/* SS_EXT_TS_USIC */

#define CSL_APP_CANCFG_SS_EXT_TS_USIC_EXT_TS_INTR_CNTR_MASK                    (0x0000001FU)
#define CSL_APP_CANCFG_SS_EXT_TS_USIC_EXT_TS_INTR_CNTR_SHIFT                   (0x00000000U)
#define CSL_APP_CANCFG_SS_EXT_TS_USIC_EXT_TS_INTR_CNTR_RESETVAL                (0x00000000U)
#define CSL_APP_CANCFG_SS_EXT_TS_USIC_EXT_TS_INTR_CNTR_MAX                     (0x0000001FU)

#define CSL_APP_CANCFG_SS_EXT_TS_USIC_NU9_MASK                                 (0xFFFFFFE0U)
#define CSL_APP_CANCFG_SS_EXT_TS_USIC_NU9_SHIFT                                (0x00000005U)
#define CSL_APP_CANCFG_SS_EXT_TS_USIC_NU9_RESETVAL                             (0x00000000U)
#define CSL_APP_CANCFG_SS_EXT_TS_USIC_NU9_MAX                                  (0x07FFFFFFU)

#define CSL_APP_CANCFG_SS_EXT_TS_USIC_RESETVAL                                 (0x00000000U)

/* CREL */

#define CSL_APP_CANCFG_CREL_DAY_MASK                                           (0x000000FFU)
#define CSL_APP_CANCFG_CREL_DAY_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_CREL_DAY_RESETVAL                                       (0x00000008U)
#define CSL_APP_CANCFG_CREL_DAY_MAX                                            (0x000000FFU)

#define CSL_APP_CANCFG_CREL_MON_MASK                                           (0x0000FF00U)
#define CSL_APP_CANCFG_CREL_MON_SHIFT                                          (0x00000008U)
#define CSL_APP_CANCFG_CREL_MON_RESETVAL                                       (0x00000006U)
#define CSL_APP_CANCFG_CREL_MON_MAX                                            (0x000000FFU)

#define CSL_APP_CANCFG_CREL_YEAR_MASK                                          (0x000F0000U)
#define CSL_APP_CANCFG_CREL_YEAR_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_CREL_YEAR_RESETVAL                                      (0x00000008U)
#define CSL_APP_CANCFG_CREL_YEAR_MAX                                           (0x0000000FU)

#define CSL_APP_CANCFG_CREL_SUBSTEP_MASK                                       (0x00F00000U)
#define CSL_APP_CANCFG_CREL_SUBSTEP_SHIFT                                      (0x00000014U)
#define CSL_APP_CANCFG_CREL_SUBSTEP_RESETVAL                                   (0x00000003U)
#define CSL_APP_CANCFG_CREL_SUBSTEP_MAX                                        (0x0000000FU)

#define CSL_APP_CANCFG_CREL_STEP_MASK                                          (0x0F000000U)
#define CSL_APP_CANCFG_CREL_STEP_SHIFT                                         (0x00000018U)
#define CSL_APP_CANCFG_CREL_STEP_RESETVAL                                      (0x00000002U)
#define CSL_APP_CANCFG_CREL_STEP_MAX                                           (0x0000000FU)

#define CSL_APP_CANCFG_CREL_REL_MASK                                           (0xF0000000U)
#define CSL_APP_CANCFG_CREL_REL_SHIFT                                          (0x0000001CU)
#define CSL_APP_CANCFG_CREL_REL_RESETVAL                                       (0x00000003U)
#define CSL_APP_CANCFG_CREL_REL_MAX                                            (0x0000000FU)

#define CSL_APP_CANCFG_CREL_RESETVAL                                           (0x32380608U)

/* ENDN */

#define CSL_APP_CANCFG_ENDN_ETV_MASK                                           (0xFFFFFFFFU)
#define CSL_APP_CANCFG_ENDN_ETV_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_ENDN_ETV_RESETVAL                                       (0x87654321U)
#define CSL_APP_CANCFG_ENDN_ETV_MAX                                            (0xFFFFFFFFU)

#define CSL_APP_CANCFG_ENDN_RESETVAL                                           (0x87654321U)

/* CUST */

#define CSL_APP_CANCFG_CUST_CUST_MASK                                          (0xFFFFFFFFU)
#define CSL_APP_CANCFG_CUST_CUST_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_CUST_CUST_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CUST_CUST_MAX                                           (0xFFFFFFFFU)

#define CSL_APP_CANCFG_CUST_RESETVAL                                           (0x00000000U)

/* DBTP */

#define CSL_APP_CANCFG_DBTP_DSJW_MASK                                          (0x0000000FU)
#define CSL_APP_CANCFG_DBTP_DSJW_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_DBTP_DSJW_RESETVAL                                      (0x00000003U)
#define CSL_APP_CANCFG_DBTP_DSJW_MAX                                           (0x0000000FU)

#define CSL_APP_CANCFG_DBTP_DTSEG2_MASK                                        (0x000000F0U)
#define CSL_APP_CANCFG_DBTP_DTSEG2_SHIFT                                       (0x00000004U)
#define CSL_APP_CANCFG_DBTP_DTSEG2_RESETVAL                                    (0x00000003U)
#define CSL_APP_CANCFG_DBTP_DTSEG2_MAX                                         (0x0000000FU)

#define CSL_APP_CANCFG_DBTP_DTSEG1_MASK                                        (0x00001F00U)
#define CSL_APP_CANCFG_DBTP_DTSEG1_SHIFT                                       (0x00000008U)
#define CSL_APP_CANCFG_DBTP_DTSEG1_RESETVAL                                    (0x0000000AU)
#define CSL_APP_CANCFG_DBTP_DTSEG1_MAX                                         (0x0000001FU)

#define CSL_APP_CANCFG_DBTP_NU11_MASK                                          (0x0000E000U)
#define CSL_APP_CANCFG_DBTP_NU11_SHIFT                                         (0x0000000DU)
#define CSL_APP_CANCFG_DBTP_NU11_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_DBTP_NU11_MAX                                           (0x00000007U)

#define CSL_APP_CANCFG_DBTP_DBRP_MASK                                          (0x001F0000U)
#define CSL_APP_CANCFG_DBTP_DBRP_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_DBTP_DBRP_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_DBTP_DBRP_MAX                                           (0x0000001FU)

#define CSL_APP_CANCFG_DBTP_NU12_MASK                                          (0x00600000U)
#define CSL_APP_CANCFG_DBTP_NU12_SHIFT                                         (0x00000015U)
#define CSL_APP_CANCFG_DBTP_NU12_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_DBTP_NU12_MAX                                           (0x00000003U)

#define CSL_APP_CANCFG_DBTP_TDC_MASK                                           (0x00800000U)
#define CSL_APP_CANCFG_DBTP_TDC_SHIFT                                          (0x00000017U)
#define CSL_APP_CANCFG_DBTP_TDC_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_DBTP_TDC_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_DBTP_NU13_MASK                                          (0xFF000000U)
#define CSL_APP_CANCFG_DBTP_NU13_SHIFT                                         (0x00000018U)
#define CSL_APP_CANCFG_DBTP_NU13_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_DBTP_NU13_MAX                                           (0x000000FFU)

#define CSL_APP_CANCFG_DBTP_RESETVAL                                           (0x00000A33U)

/* TEST */

#define CSL_APP_CANCFG_TEST_NU14_MASK                                          (0x0000000FU)
#define CSL_APP_CANCFG_TEST_NU14_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_TEST_NU14_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TEST_NU14_MAX                                           (0x0000000FU)

#define CSL_APP_CANCFG_TEST_LBCK_MASK                                          (0x00000010U)
#define CSL_APP_CANCFG_TEST_LBCK_SHIFT                                         (0x00000004U)
#define CSL_APP_CANCFG_TEST_LBCK_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TEST_LBCK_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_TEST_TX_MASK                                            (0x00000060U)
#define CSL_APP_CANCFG_TEST_TX_SHIFT                                           (0x00000005U)
#define CSL_APP_CANCFG_TEST_TX_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_TEST_TX_MAX                                             (0x00000003U)

#define CSL_APP_CANCFG_TEST_RX_MASK                                            (0x00000080U)
#define CSL_APP_CANCFG_TEST_RX_SHIFT                                           (0x00000007U)
#define CSL_APP_CANCFG_TEST_RX_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_TEST_RX_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_TEST_NU15_MASK                                          (0xFFFFFF00U)
#define CSL_APP_CANCFG_TEST_NU15_SHIFT                                         (0x00000008U)
#define CSL_APP_CANCFG_TEST_NU15_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TEST_NU15_MAX                                           (0x00FFFFFFU)

#define CSL_APP_CANCFG_TEST_RESETVAL                                           (0x00000000U)

/* RWD */

#define CSL_APP_CANCFG_RWD_WDC_MASK                                            (0x000000FFU)
#define CSL_APP_CANCFG_RWD_WDC_SHIFT                                           (0x00000000U)
#define CSL_APP_CANCFG_RWD_WDC_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_RWD_WDC_MAX                                             (0x000000FFU)

#define CSL_APP_CANCFG_RWD_WDV_MASK                                            (0x0000FF00U)
#define CSL_APP_CANCFG_RWD_WDV_SHIFT                                           (0x00000008U)
#define CSL_APP_CANCFG_RWD_WDV_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_RWD_WDV_MAX                                             (0x000000FFU)

#define CSL_APP_CANCFG_RWD_NU16_MASK                                           (0xFFFF0000U)
#define CSL_APP_CANCFG_RWD_NU16_SHIFT                                          (0x00000010U)
#define CSL_APP_CANCFG_RWD_NU16_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_RWD_NU16_MAX                                            (0x0000FFFFU)

#define CSL_APP_CANCFG_RWD_RESETVAL                                            (0x00000000U)

/* CCCR */

#define CSL_APP_CANCFG_CCCR_INIT_MASK                                          (0x00000001U)
#define CSL_APP_CANCFG_CCCR_INIT_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_CCCR_INIT_RESETVAL                                      (0x00000001U)
#define CSL_APP_CANCFG_CCCR_INIT_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_CCCR_CCE_MASK                                           (0x00000002U)
#define CSL_APP_CANCFG_CCCR_CCE_SHIFT                                          (0x00000001U)
#define CSL_APP_CANCFG_CCCR_CCE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_CCCR_CCE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_CCCR_ASM_MASK                                           (0x00000004U)
#define CSL_APP_CANCFG_CCCR_ASM_SHIFT                                          (0x00000002U)
#define CSL_APP_CANCFG_CCCR_ASM_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_CCCR_ASM_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_CCCR_CSA_MASK                                           (0x00000008U)
#define CSL_APP_CANCFG_CCCR_CSA_SHIFT                                          (0x00000003U)
#define CSL_APP_CANCFG_CCCR_CSA_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_CCCR_CSA_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_CCCR_CSR_MASK                                           (0x00000010U)
#define CSL_APP_CANCFG_CCCR_CSR_SHIFT                                          (0x00000004U)
#define CSL_APP_CANCFG_CCCR_CSR_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_CCCR_CSR_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_CCCR_MON_MASK                                           (0x00000020U)
#define CSL_APP_CANCFG_CCCR_MON_SHIFT                                          (0x00000005U)
#define CSL_APP_CANCFG_CCCR_MON_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_CCCR_MON_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_CCCR_DAR_MASK                                           (0x00000040U)
#define CSL_APP_CANCFG_CCCR_DAR_SHIFT                                          (0x00000006U)
#define CSL_APP_CANCFG_CCCR_DAR_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_CCCR_DAR_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_CCCR_TEST_MASK                                          (0x00000080U)
#define CSL_APP_CANCFG_CCCR_TEST_SHIFT                                         (0x00000007U)
#define CSL_APP_CANCFG_CCCR_TEST_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CCCR_TEST_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_CCCR_FDOE_MASK                                          (0x00000100U)
#define CSL_APP_CANCFG_CCCR_FDOE_SHIFT                                         (0x00000008U)
#define CSL_APP_CANCFG_CCCR_FDOE_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CCCR_FDOE_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_CCCR_BRSE_MASK                                          (0x00000200U)
#define CSL_APP_CANCFG_CCCR_BRSE_SHIFT                                         (0x00000009U)
#define CSL_APP_CANCFG_CCCR_BRSE_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CCCR_BRSE_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_CCCR_NU17_MASK                                          (0x00000C00U)
#define CSL_APP_CANCFG_CCCR_NU17_SHIFT                                         (0x0000000AU)
#define CSL_APP_CANCFG_CCCR_NU17_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CCCR_NU17_MAX                                           (0x00000003U)

#define CSL_APP_CANCFG_CCCR_PXHD_MASK                                          (0x00001000U)
#define CSL_APP_CANCFG_CCCR_PXHD_SHIFT                                         (0x0000000CU)
#define CSL_APP_CANCFG_CCCR_PXHD_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CCCR_PXHD_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_CCCR_EFBI_MASK                                          (0x00002000U)
#define CSL_APP_CANCFG_CCCR_EFBI_SHIFT                                         (0x0000000DU)
#define CSL_APP_CANCFG_CCCR_EFBI_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CCCR_EFBI_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_CCCR_TXP_MASK                                           (0x00004000U)
#define CSL_APP_CANCFG_CCCR_TXP_SHIFT                                          (0x0000000EU)
#define CSL_APP_CANCFG_CCCR_TXP_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_CCCR_TXP_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_CCCR_NU18_MASK                                          (0xFFFF8000U)
#define CSL_APP_CANCFG_CCCR_NU18_SHIFT                                         (0x0000000FU)
#define CSL_APP_CANCFG_CCCR_NU18_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_CCCR_NU18_MAX                                           (0x0001FFFFU)

#define CSL_APP_CANCFG_CCCR_RESETVAL                                           (0x00000001U)

/* NBTP */

#define CSL_APP_CANCFG_NBTP_NTSEG2_MASK                                        (0x0000007FU)
#define CSL_APP_CANCFG_NBTP_NTSEG2_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_NBTP_NTSEG2_RESETVAL                                    (0x00000003U)
#define CSL_APP_CANCFG_NBTP_NTSEG2_MAX                                         (0x0000007FU)

#define CSL_APP_CANCFG_NBTP_NU19_MASK                                          (0x00000080U)
#define CSL_APP_CANCFG_NBTP_NU19_SHIFT                                         (0x00000007U)
#define CSL_APP_CANCFG_NBTP_NU19_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_NBTP_NU19_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_NBTP_NTSEG1_MASK                                        (0x0000FF00U)
#define CSL_APP_CANCFG_NBTP_NTSEG1_SHIFT                                       (0x00000008U)
#define CSL_APP_CANCFG_NBTP_NTSEG1_RESETVAL                                    (0x0000000AU)
#define CSL_APP_CANCFG_NBTP_NTSEG1_MAX                                         (0x000000FFU)

#define CSL_APP_CANCFG_NBTP_NBRP_MASK                                          (0x01FF0000U)
#define CSL_APP_CANCFG_NBTP_NBRP_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_NBTP_NBRP_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_NBTP_NBRP_MAX                                           (0x000001FFU)

#define CSL_APP_CANCFG_NBTP_NSJW_MASK                                          (0xFE000000U)
#define CSL_APP_CANCFG_NBTP_NSJW_SHIFT                                         (0x00000019U)
#define CSL_APP_CANCFG_NBTP_NSJW_RESETVAL                                      (0x00000003U)
#define CSL_APP_CANCFG_NBTP_NSJW_MAX                                           (0x0000007FU)

#define CSL_APP_CANCFG_NBTP_RESETVAL                                           (0x06000A03U)

/* TSCC */

#define CSL_APP_CANCFG_TSCC_TSS_MASK                                           (0x00000003U)
#define CSL_APP_CANCFG_TSCC_TSS_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_TSCC_TSS_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TSCC_TSS_MAX                                            (0x00000003U)

#define CSL_APP_CANCFG_TSCC_NU20_MASK                                          (0x0000FFFCU)
#define CSL_APP_CANCFG_TSCC_NU20_SHIFT                                         (0x00000002U)
#define CSL_APP_CANCFG_TSCC_NU20_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TSCC_NU20_MAX                                           (0x00003FFFU)

#define CSL_APP_CANCFG_TSCC_TCP_MASK                                           (0x000F0000U)
#define CSL_APP_CANCFG_TSCC_TCP_SHIFT                                          (0x00000010U)
#define CSL_APP_CANCFG_TSCC_TCP_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TSCC_TCP_MAX                                            (0x0000000FU)

#define CSL_APP_CANCFG_TSCC_NU21_MASK                                          (0xFFF00000U)
#define CSL_APP_CANCFG_TSCC_NU21_SHIFT                                         (0x00000014U)
#define CSL_APP_CANCFG_TSCC_NU21_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TSCC_NU21_MAX                                           (0x00000FFFU)

#define CSL_APP_CANCFG_TSCC_RESETVAL                                           (0x00000000U)

/* TSCV */

#define CSL_APP_CANCFG_TSCV_TSC_MASK                                           (0x0000FFFFU)
#define CSL_APP_CANCFG_TSCV_TSC_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_TSCV_TSC_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TSCV_TSC_MAX                                            (0x0000FFFFU)

#define CSL_APP_CANCFG_TSCV_NU22_MASK                                          (0xFFFF0000U)
#define CSL_APP_CANCFG_TSCV_NU22_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_TSCV_NU22_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TSCV_NU22_MAX                                           (0x0000FFFFU)

#define CSL_APP_CANCFG_TSCV_RESETVAL                                           (0x00000000U)

/* TOCC */

#define CSL_APP_CANCFG_TOCC_ETOC_MASK                                          (0x00000001U)
#define CSL_APP_CANCFG_TOCC_ETOC_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_TOCC_ETOC_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TOCC_ETOC_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_TOCC_TOS_MASK                                           (0x00000006U)
#define CSL_APP_CANCFG_TOCC_TOS_SHIFT                                          (0x00000001U)
#define CSL_APP_CANCFG_TOCC_TOS_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TOCC_TOS_MAX                                            (0x00000003U)

#define CSL_APP_CANCFG_TOCC_NU23_MASK                                          (0x0000FFF8U)
#define CSL_APP_CANCFG_TOCC_NU23_SHIFT                                         (0x00000003U)
#define CSL_APP_CANCFG_TOCC_NU23_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TOCC_NU23_MAX                                           (0x00001FFFU)

#define CSL_APP_CANCFG_TOCC_TOP_MASK                                           (0xFFFF0000U)
#define CSL_APP_CANCFG_TOCC_TOP_SHIFT                                          (0x00000010U)
#define CSL_APP_CANCFG_TOCC_TOP_RESETVAL                                       (0x0000FFFFU)
#define CSL_APP_CANCFG_TOCC_TOP_MAX                                            (0x0000FFFFU)

#define CSL_APP_CANCFG_TOCC_RESETVAL                                           (0xFFFF0000U)

/* TOCV */

#define CSL_APP_CANCFG_TOCV_TOC_MASK                                           (0x0000FFFFU)
#define CSL_APP_CANCFG_TOCV_TOC_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_TOCV_TOC_RESETVAL                                       (0x0000FFFFU)
#define CSL_APP_CANCFG_TOCV_TOC_MAX                                            (0x0000FFFFU)

#define CSL_APP_CANCFG_TOCV_NU24_MASK                                          (0xFFFF0000U)
#define CSL_APP_CANCFG_TOCV_NU24_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_TOCV_NU24_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TOCV_NU24_MAX                                           (0x0000FFFFU)

#define CSL_APP_CANCFG_TOCV_RESETVAL                                           (0x0000FFFFU)

/* RES00 */

#define CSL_APP_CANCFG_RES00_RES00_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES00_RES00_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES00_RES00_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES00_RES00_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES00_RESETVAL                                          (0x00000000U)

/* RES01 */

#define CSL_APP_CANCFG_RES01_RES01_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES01_RES01_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES01_RES01_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES01_RES01_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES01_RESETVAL                                          (0x00000000U)

/* RES02 */

#define CSL_APP_CANCFG_RES02_RES02_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES02_RES02_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES02_RES02_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES02_RES02_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES02_RESETVAL                                          (0x00000000U)

/* RES03 */

#define CSL_APP_CANCFG_RES03_RES03_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES03_RES03_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES03_RES03_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES03_RES03_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES03_RESETVAL                                          (0x00000000U)

/* ECR */

#define CSL_APP_CANCFG_ECR_TEC_MASK                                            (0x000000FFU)
#define CSL_APP_CANCFG_ECR_TEC_SHIFT                                           (0x00000000U)
#define CSL_APP_CANCFG_ECR_TEC_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_ECR_TEC_MAX                                             (0x000000FFU)

#define CSL_APP_CANCFG_ECR_REC_MASK                                            (0x00007F00U)
#define CSL_APP_CANCFG_ECR_REC_SHIFT                                           (0x00000008U)
#define CSL_APP_CANCFG_ECR_REC_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_ECR_REC_MAX                                             (0x0000007FU)

#define CSL_APP_CANCFG_ECR_RP_MASK                                             (0x00008000U)
#define CSL_APP_CANCFG_ECR_RP_SHIFT                                            (0x0000000FU)
#define CSL_APP_CANCFG_ECR_RP_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_ECR_RP_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_ECR_CEL_MASK                                            (0x00FF0000U)
#define CSL_APP_CANCFG_ECR_CEL_SHIFT                                           (0x00000010U)
#define CSL_APP_CANCFG_ECR_CEL_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_ECR_CEL_MAX                                             (0x000000FFU)

#define CSL_APP_CANCFG_ECR_NU25_MASK                                           (0xFF000000U)
#define CSL_APP_CANCFG_ECR_NU25_SHIFT                                          (0x00000018U)
#define CSL_APP_CANCFG_ECR_NU25_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ECR_NU25_MAX                                            (0x000000FFU)

#define CSL_APP_CANCFG_ECR_RESETVAL                                            (0x00000000U)

/* PSR */

#define CSL_APP_CANCFG_PSR_LEC_MASK                                            (0x00000007U)
#define CSL_APP_CANCFG_PSR_LEC_SHIFT                                           (0x00000000U)
#define CSL_APP_CANCFG_PSR_LEC_RESETVAL                                        (0x00000007U)
#define CSL_APP_CANCFG_PSR_LEC_MAX                                             (0x00000007U)

#define CSL_APP_CANCFG_PSR_ACT_MASK                                            (0x00000018U)
#define CSL_APP_CANCFG_PSR_ACT_SHIFT                                           (0x00000003U)
#define CSL_APP_CANCFG_PSR_ACT_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_PSR_ACT_MAX                                             (0x00000003U)

#define CSL_APP_CANCFG_PSR_EP_MASK                                             (0x00000020U)
#define CSL_APP_CANCFG_PSR_EP_SHIFT                                            (0x00000005U)
#define CSL_APP_CANCFG_PSR_EP_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_PSR_EP_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_PSR_EW_MASK                                             (0x00000040U)
#define CSL_APP_CANCFG_PSR_EW_SHIFT                                            (0x00000006U)
#define CSL_APP_CANCFG_PSR_EW_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_PSR_EW_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_PSR_BO_MASK                                             (0x00000080U)
#define CSL_APP_CANCFG_PSR_BO_SHIFT                                            (0x00000007U)
#define CSL_APP_CANCFG_PSR_BO_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_PSR_BO_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_PSR_DLEC_MASK                                           (0x00000700U)
#define CSL_APP_CANCFG_PSR_DLEC_SHIFT                                          (0x00000008U)
#define CSL_APP_CANCFG_PSR_DLEC_RESETVAL                                       (0x00000007U)
#define CSL_APP_CANCFG_PSR_DLEC_MAX                                            (0x00000007U)

#define CSL_APP_CANCFG_PSR_RESI_MASK                                           (0x00000800U)
#define CSL_APP_CANCFG_PSR_RESI_SHIFT                                          (0x0000000BU)
#define CSL_APP_CANCFG_PSR_RESI_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_PSR_RESI_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_PSR_RBRS_MASK                                           (0x00001000U)
#define CSL_APP_CANCFG_PSR_RBRS_SHIFT                                          (0x0000000CU)
#define CSL_APP_CANCFG_PSR_RBRS_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_PSR_RBRS_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_PSR_RFDF_MASK                                           (0x00002000U)
#define CSL_APP_CANCFG_PSR_RFDF_SHIFT                                          (0x0000000DU)
#define CSL_APP_CANCFG_PSR_RFDF_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_PSR_RFDF_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_PSR_PXE_MASK                                            (0x00004000U)
#define CSL_APP_CANCFG_PSR_PXE_SHIFT                                           (0x0000000EU)
#define CSL_APP_CANCFG_PSR_PXE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_PSR_PXE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_PSR_NU26_MASK                                           (0x00008000U)
#define CSL_APP_CANCFG_PSR_NU26_SHIFT                                          (0x0000000FU)
#define CSL_APP_CANCFG_PSR_NU26_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_PSR_NU26_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_PSR_TDCV_MASK                                           (0x007F0000U)
#define CSL_APP_CANCFG_PSR_TDCV_SHIFT                                          (0x00000010U)
#define CSL_APP_CANCFG_PSR_TDCV_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_PSR_TDCV_MAX                                            (0x0000007FU)

#define CSL_APP_CANCFG_PSR_NU27_MASK                                           (0xFF800000U)
#define CSL_APP_CANCFG_PSR_NU27_SHIFT                                          (0x00000017U)
#define CSL_APP_CANCFG_PSR_NU27_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_PSR_NU27_MAX                                            (0x000001FFU)

#define CSL_APP_CANCFG_PSR_RESETVAL                                            (0x00000707U)

/* TDCR */

#define CSL_APP_CANCFG_TDCR_TDCF_MASK                                          (0x0000007FU)
#define CSL_APP_CANCFG_TDCR_TDCF_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_TDCR_TDCF_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TDCR_TDCF_MAX                                           (0x0000007FU)

#define CSL_APP_CANCFG_TDCR_NU28_MASK                                          (0x00000080U)
#define CSL_APP_CANCFG_TDCR_NU28_SHIFT                                         (0x00000007U)
#define CSL_APP_CANCFG_TDCR_NU28_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TDCR_NU28_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_TDCR_TDCO_MASK                                          (0x00007F00U)
#define CSL_APP_CANCFG_TDCR_TDCO_SHIFT                                         (0x00000008U)
#define CSL_APP_CANCFG_TDCR_TDCO_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TDCR_TDCO_MAX                                           (0x0000007FU)

#define CSL_APP_CANCFG_TDCR_NU29_MASK                                          (0xFFFF8000U)
#define CSL_APP_CANCFG_TDCR_NU29_SHIFT                                         (0x0000000FU)
#define CSL_APP_CANCFG_TDCR_NU29_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TDCR_NU29_MAX                                           (0x0001FFFFU)

#define CSL_APP_CANCFG_TDCR_RESETVAL                                           (0x00000000U)

/* RES04 */

#define CSL_APP_CANCFG_RES04_RES04_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES04_RES04_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES04_RES04_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES04_RES04_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES04_RESETVAL                                          (0x00000000U)

/* IR */

#define CSL_APP_CANCFG_IR_RF0N_MASK                                            (0x00000001U)
#define CSL_APP_CANCFG_IR_RF0N_SHIFT                                           (0x00000000U)
#define CSL_APP_CANCFG_IR_RF0N_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF0N_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_RF0W_MASK                                            (0x00000002U)
#define CSL_APP_CANCFG_IR_RF0W_SHIFT                                           (0x00000001U)
#define CSL_APP_CANCFG_IR_RF0W_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF0W_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_RF0F_MASK                                            (0x00000004U)
#define CSL_APP_CANCFG_IR_RF0F_SHIFT                                           (0x00000002U)
#define CSL_APP_CANCFG_IR_RF0F_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF0F_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_RF0L_MASK                                            (0x00000008U)
#define CSL_APP_CANCFG_IR_RF0L_SHIFT                                           (0x00000003U)
#define CSL_APP_CANCFG_IR_RF0L_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF0L_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_RF1N_MASK                                            (0x00000010U)
#define CSL_APP_CANCFG_IR_RF1N_SHIFT                                           (0x00000004U)
#define CSL_APP_CANCFG_IR_RF1N_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF1N_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_RF1W_MASK                                            (0x00000020U)
#define CSL_APP_CANCFG_IR_RF1W_SHIFT                                           (0x00000005U)
#define CSL_APP_CANCFG_IR_RF1W_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF1W_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_RF1F_MASK                                            (0x00000040U)
#define CSL_APP_CANCFG_IR_RF1F_SHIFT                                           (0x00000006U)
#define CSL_APP_CANCFG_IR_RF1F_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF1F_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_RF1L_MASK                                            (0x00000080U)
#define CSL_APP_CANCFG_IR_RF1L_SHIFT                                           (0x00000007U)
#define CSL_APP_CANCFG_IR_RF1L_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_RF1L_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_HPM_MASK                                             (0x00000100U)
#define CSL_APP_CANCFG_IR_HPM_SHIFT                                            (0x00000008U)
#define CSL_APP_CANCFG_IR_HPM_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_HPM_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_TC_MASK                                              (0x00000200U)
#define CSL_APP_CANCFG_IR_TC_SHIFT                                             (0x00000009U)
#define CSL_APP_CANCFG_IR_TC_RESETVAL                                          (0x00000000U)
#define CSL_APP_CANCFG_IR_TC_MAX                                               (0x00000001U)

#define CSL_APP_CANCFG_IR_TCF_MASK                                             (0x00000400U)
#define CSL_APP_CANCFG_IR_TCF_SHIFT                                            (0x0000000AU)
#define CSL_APP_CANCFG_IR_TCF_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_TCF_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_TFE_MASK                                             (0x00000800U)
#define CSL_APP_CANCFG_IR_TFE_SHIFT                                            (0x0000000BU)
#define CSL_APP_CANCFG_IR_TFE_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_TFE_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_TEFN_MASK                                            (0x00001000U)
#define CSL_APP_CANCFG_IR_TEFN_SHIFT                                           (0x0000000CU)
#define CSL_APP_CANCFG_IR_TEFN_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_TEFN_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_TEFW_MASK                                            (0x00002000U)
#define CSL_APP_CANCFG_IR_TEFW_SHIFT                                           (0x0000000DU)
#define CSL_APP_CANCFG_IR_TEFW_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_TEFW_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_TEFF_MASK                                            (0x00004000U)
#define CSL_APP_CANCFG_IR_TEFF_SHIFT                                           (0x0000000EU)
#define CSL_APP_CANCFG_IR_TEFF_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_TEFF_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_TEFL_MASK                                            (0x00008000U)
#define CSL_APP_CANCFG_IR_TEFL_SHIFT                                           (0x0000000FU)
#define CSL_APP_CANCFG_IR_TEFL_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_TEFL_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_TSW_MASK                                             (0x00010000U)
#define CSL_APP_CANCFG_IR_TSW_SHIFT                                            (0x00000010U)
#define CSL_APP_CANCFG_IR_TSW_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_TSW_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_MRAF_MASK                                            (0x00020000U)
#define CSL_APP_CANCFG_IR_MRAF_SHIFT                                           (0x00000011U)
#define CSL_APP_CANCFG_IR_MRAF_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_MRAF_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IR_TOO_MASK                                             (0x00040000U)
#define CSL_APP_CANCFG_IR_TOO_SHIFT                                            (0x00000012U)
#define CSL_APP_CANCFG_IR_TOO_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_TOO_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_DRX_MASK                                             (0x00080000U)
#define CSL_APP_CANCFG_IR_DRX_SHIFT                                            (0x00000013U)
#define CSL_APP_CANCFG_IR_DRX_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_DRX_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_BEC_MASK                                             (0x00100000U)
#define CSL_APP_CANCFG_IR_BEC_SHIFT                                            (0x00000014U)
#define CSL_APP_CANCFG_IR_BEC_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_BEC_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_BEU_MASK                                             (0x00200000U)
#define CSL_APP_CANCFG_IR_BEU_SHIFT                                            (0x00000015U)
#define CSL_APP_CANCFG_IR_BEU_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_BEU_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_ELO_MASK                                             (0x00400000U)
#define CSL_APP_CANCFG_IR_ELO_SHIFT                                            (0x00000016U)
#define CSL_APP_CANCFG_IR_ELO_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_ELO_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_EP_MASK                                              (0x00800000U)
#define CSL_APP_CANCFG_IR_EP_SHIFT                                             (0x00000017U)
#define CSL_APP_CANCFG_IR_EP_RESETVAL                                          (0x00000000U)
#define CSL_APP_CANCFG_IR_EP_MAX                                               (0x00000001U)

#define CSL_APP_CANCFG_IR_EW_MASK                                              (0x01000000U)
#define CSL_APP_CANCFG_IR_EW_SHIFT                                             (0x00000018U)
#define CSL_APP_CANCFG_IR_EW_RESETVAL                                          (0x00000000U)
#define CSL_APP_CANCFG_IR_EW_MAX                                               (0x00000001U)

#define CSL_APP_CANCFG_IR_BO_MASK                                              (0x02000000U)
#define CSL_APP_CANCFG_IR_BO_SHIFT                                             (0x00000019U)
#define CSL_APP_CANCFG_IR_BO_RESETVAL                                          (0x00000000U)
#define CSL_APP_CANCFG_IR_BO_MAX                                               (0x00000001U)

#define CSL_APP_CANCFG_IR_WDI_MASK                                             (0x04000000U)
#define CSL_APP_CANCFG_IR_WDI_SHIFT                                            (0x0000001AU)
#define CSL_APP_CANCFG_IR_WDI_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_WDI_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_PEA_MASK                                             (0x08000000U)
#define CSL_APP_CANCFG_IR_PEA_SHIFT                                            (0x0000001BU)
#define CSL_APP_CANCFG_IR_PEA_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_PEA_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_PED_MASK                                             (0x10000000U)
#define CSL_APP_CANCFG_IR_PED_SHIFT                                            (0x0000001CU)
#define CSL_APP_CANCFG_IR_PED_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_PED_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_ARA_MASK                                             (0x20000000U)
#define CSL_APP_CANCFG_IR_ARA_SHIFT                                            (0x0000001DU)
#define CSL_APP_CANCFG_IR_ARA_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IR_ARA_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IR_NU30_MASK                                            (0xC0000000U)
#define CSL_APP_CANCFG_IR_NU30_SHIFT                                           (0x0000001EU)
#define CSL_APP_CANCFG_IR_NU30_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IR_NU30_MAX                                             (0x00000003U)

#define CSL_APP_CANCFG_IR_RESETVAL                                             (0x00000000U)

/* IE */

#define CSL_APP_CANCFG_IE_RF0NE_MASK                                           (0x00000001U)
#define CSL_APP_CANCFG_IE_RF0NE_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_IE_RF0NE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF0NE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_RF0WE_MASK                                           (0x00000002U)
#define CSL_APP_CANCFG_IE_RF0WE_SHIFT                                          (0x00000001U)
#define CSL_APP_CANCFG_IE_RF0WE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF0WE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_RF0FE_MASK                                           (0x00000004U)
#define CSL_APP_CANCFG_IE_RF0FE_SHIFT                                          (0x00000002U)
#define CSL_APP_CANCFG_IE_RF0FE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF0FE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_RF0LE_MASK                                           (0x00000008U)
#define CSL_APP_CANCFG_IE_RF0LE_SHIFT                                          (0x00000003U)
#define CSL_APP_CANCFG_IE_RF0LE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF0LE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_RF1NE_MASK                                           (0x00000010U)
#define CSL_APP_CANCFG_IE_RF1NE_SHIFT                                          (0x00000004U)
#define CSL_APP_CANCFG_IE_RF1NE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF1NE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_RF1WE_MASK                                           (0x00000020U)
#define CSL_APP_CANCFG_IE_RF1WE_SHIFT                                          (0x00000005U)
#define CSL_APP_CANCFG_IE_RF1WE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF1WE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_RF1FE_MASK                                           (0x00000040U)
#define CSL_APP_CANCFG_IE_RF1FE_SHIFT                                          (0x00000006U)
#define CSL_APP_CANCFG_IE_RF1FE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF1FE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_RF1LE_MASK                                           (0x00000080U)
#define CSL_APP_CANCFG_IE_RF1LE_SHIFT                                          (0x00000007U)
#define CSL_APP_CANCFG_IE_RF1LE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_RF1LE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_HPME_MASK                                            (0x00000100U)
#define CSL_APP_CANCFG_IE_HPME_SHIFT                                           (0x00000008U)
#define CSL_APP_CANCFG_IE_HPME_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_HPME_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_TCE_MASK                                             (0x00000200U)
#define CSL_APP_CANCFG_IE_TCE_SHIFT                                            (0x00000009U)
#define CSL_APP_CANCFG_IE_TCE_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IE_TCE_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IE_TCFE_MASK                                            (0x00000400U)
#define CSL_APP_CANCFG_IE_TCFE_SHIFT                                           (0x0000000AU)
#define CSL_APP_CANCFG_IE_TCFE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_TCFE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_TFEE_MASK                                            (0x00000800U)
#define CSL_APP_CANCFG_IE_TFEE_SHIFT                                           (0x0000000BU)
#define CSL_APP_CANCFG_IE_TFEE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_TFEE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_TEFNE_MASK                                           (0x00001000U)
#define CSL_APP_CANCFG_IE_TEFNE_SHIFT                                          (0x0000000CU)
#define CSL_APP_CANCFG_IE_TEFNE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_TEFNE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_TEFWE_MASK                                           (0x00002000U)
#define CSL_APP_CANCFG_IE_TEFWE_SHIFT                                          (0x0000000DU)
#define CSL_APP_CANCFG_IE_TEFWE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_TEFWE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_TEFFE_MASK                                           (0x00004000U)
#define CSL_APP_CANCFG_IE_TEFFE_SHIFT                                          (0x0000000EU)
#define CSL_APP_CANCFG_IE_TEFFE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_TEFFE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_TEFLE_MASK                                           (0x00008000U)
#define CSL_APP_CANCFG_IE_TEFLE_SHIFT                                          (0x0000000FU)
#define CSL_APP_CANCFG_IE_TEFLE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_TEFLE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_TSWE_MASK                                            (0x00010000U)
#define CSL_APP_CANCFG_IE_TSWE_SHIFT                                           (0x00000010U)
#define CSL_APP_CANCFG_IE_TSWE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_TSWE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_MRAFE_MASK                                           (0x00020000U)
#define CSL_APP_CANCFG_IE_MRAFE_SHIFT                                          (0x00000011U)
#define CSL_APP_CANCFG_IE_MRAFE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_IE_MRAFE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_IE_TOOE_MASK                                            (0x00040000U)
#define CSL_APP_CANCFG_IE_TOOE_SHIFT                                           (0x00000012U)
#define CSL_APP_CANCFG_IE_TOOE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_TOOE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_DRX_MASK                                             (0x00080000U)
#define CSL_APP_CANCFG_IE_DRX_SHIFT                                            (0x00000013U)
#define CSL_APP_CANCFG_IE_DRX_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IE_DRX_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IE_BECE_MASK                                            (0x00100000U)
#define CSL_APP_CANCFG_IE_BECE_SHIFT                                           (0x00000014U)
#define CSL_APP_CANCFG_IE_BECE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_BECE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_BEUE_MASK                                            (0x00200000U)
#define CSL_APP_CANCFG_IE_BEUE_SHIFT                                           (0x00000015U)
#define CSL_APP_CANCFG_IE_BEUE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_BEUE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_ELOE_MASK                                            (0x00400000U)
#define CSL_APP_CANCFG_IE_ELOE_SHIFT                                           (0x00000016U)
#define CSL_APP_CANCFG_IE_ELOE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_ELOE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_EPE_MASK                                             (0x00800000U)
#define CSL_APP_CANCFG_IE_EPE_SHIFT                                            (0x00000017U)
#define CSL_APP_CANCFG_IE_EPE_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IE_EPE_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IE_EWE_MASK                                             (0x01000000U)
#define CSL_APP_CANCFG_IE_EWE_SHIFT                                            (0x00000018U)
#define CSL_APP_CANCFG_IE_EWE_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IE_EWE_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IE_BOE_MASK                                             (0x02000000U)
#define CSL_APP_CANCFG_IE_BOE_SHIFT                                            (0x00000019U)
#define CSL_APP_CANCFG_IE_BOE_RESETVAL                                         (0x00000000U)
#define CSL_APP_CANCFG_IE_BOE_MAX                                              (0x00000001U)

#define CSL_APP_CANCFG_IE_WDIE_MASK                                            (0x04000000U)
#define CSL_APP_CANCFG_IE_WDIE_SHIFT                                           (0x0000001AU)
#define CSL_APP_CANCFG_IE_WDIE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_WDIE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_PEAE_MASK                                            (0x08000000U)
#define CSL_APP_CANCFG_IE_PEAE_SHIFT                                           (0x0000001BU)
#define CSL_APP_CANCFG_IE_PEAE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_PEAE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_PEDE_MASK                                            (0x10000000U)
#define CSL_APP_CANCFG_IE_PEDE_SHIFT                                           (0x0000001CU)
#define CSL_APP_CANCFG_IE_PEDE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_PEDE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_ARAE_MASK                                            (0x20000000U)
#define CSL_APP_CANCFG_IE_ARAE_SHIFT                                           (0x0000001DU)
#define CSL_APP_CANCFG_IE_ARAE_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_ARAE_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_IE_NU31_MASK                                            (0xC0000000U)
#define CSL_APP_CANCFG_IE_NU31_SHIFT                                           (0x0000001EU)
#define CSL_APP_CANCFG_IE_NU31_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_IE_NU31_MAX                                             (0x00000003U)

#define CSL_APP_CANCFG_IE_RESETVAL                                             (0x00000000U)

/* ILS */

#define CSL_APP_CANCFG_ILS_RF0NL_MASK                                          (0x00000001U)
#define CSL_APP_CANCFG_ILS_RF0NL_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF0NL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF0NL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_RF0WL_MASK                                          (0x00000002U)
#define CSL_APP_CANCFG_ILS_RF0WL_SHIFT                                         (0x00000001U)
#define CSL_APP_CANCFG_ILS_RF0WL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF0WL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_RF0FL_MASK                                          (0x00000004U)
#define CSL_APP_CANCFG_ILS_RF0FL_SHIFT                                         (0x00000002U)
#define CSL_APP_CANCFG_ILS_RF0FL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF0FL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_RF0LL_MASK                                          (0x00000008U)
#define CSL_APP_CANCFG_ILS_RF0LL_SHIFT                                         (0x00000003U)
#define CSL_APP_CANCFG_ILS_RF0LL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF0LL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_RF1NL_MASK                                          (0x00000010U)
#define CSL_APP_CANCFG_ILS_RF1NL_SHIFT                                         (0x00000004U)
#define CSL_APP_CANCFG_ILS_RF1NL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF1NL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_RF1WL_MASK                                          (0x00000020U)
#define CSL_APP_CANCFG_ILS_RF1WL_SHIFT                                         (0x00000005U)
#define CSL_APP_CANCFG_ILS_RF1WL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF1WL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_RF1FL_MASK                                          (0x00000040U)
#define CSL_APP_CANCFG_ILS_RF1FL_SHIFT                                         (0x00000006U)
#define CSL_APP_CANCFG_ILS_RF1FL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF1FL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_RF1LL_MASK                                          (0x00000080U)
#define CSL_APP_CANCFG_ILS_RF1LL_SHIFT                                         (0x00000007U)
#define CSL_APP_CANCFG_ILS_RF1LL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_RF1LL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_HPML_MASK                                           (0x00000100U)
#define CSL_APP_CANCFG_ILS_HPML_SHIFT                                          (0x00000008U)
#define CSL_APP_CANCFG_ILS_HPML_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_HPML_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_TCL_MASK                                            (0x00000200U)
#define CSL_APP_CANCFG_ILS_TCL_SHIFT                                           (0x00000009U)
#define CSL_APP_CANCFG_ILS_TCL_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_ILS_TCL_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_ILS_TCFL_MASK                                           (0x00000400U)
#define CSL_APP_CANCFG_ILS_TCFL_SHIFT                                          (0x0000000AU)
#define CSL_APP_CANCFG_ILS_TCFL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_TCFL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_TFEL_MASK                                           (0x00000800U)
#define CSL_APP_CANCFG_ILS_TFEL_SHIFT                                          (0x0000000BU)
#define CSL_APP_CANCFG_ILS_TFEL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_TFEL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_TEFNL_MASK                                          (0x00001000U)
#define CSL_APP_CANCFG_ILS_TEFNL_SHIFT                                         (0x0000000CU)
#define CSL_APP_CANCFG_ILS_TEFNL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_TEFNL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_TEFWL_MASK                                          (0x00002000U)
#define CSL_APP_CANCFG_ILS_TEFWL_SHIFT                                         (0x0000000DU)
#define CSL_APP_CANCFG_ILS_TEFWL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_TEFWL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_TEFFL_MASK                                          (0x00004000U)
#define CSL_APP_CANCFG_ILS_TEFFL_SHIFT                                         (0x0000000EU)
#define CSL_APP_CANCFG_ILS_TEFFL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_TEFFL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_TEFLL_MASK                                          (0x00008000U)
#define CSL_APP_CANCFG_ILS_TEFLL_SHIFT                                         (0x0000000FU)
#define CSL_APP_CANCFG_ILS_TEFLL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_TEFLL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_TSWL_MASK                                           (0x00010000U)
#define CSL_APP_CANCFG_ILS_TSWL_SHIFT                                          (0x00000010U)
#define CSL_APP_CANCFG_ILS_TSWL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_TSWL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_MRAFL_MASK                                          (0x00020000U)
#define CSL_APP_CANCFG_ILS_MRAFL_SHIFT                                         (0x00000011U)
#define CSL_APP_CANCFG_ILS_MRAFL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILS_MRAFL_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILS_TOOL_MASK                                           (0x00040000U)
#define CSL_APP_CANCFG_ILS_TOOL_SHIFT                                          (0x00000012U)
#define CSL_APP_CANCFG_ILS_TOOL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_TOOL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_DRXL_MASK                                           (0x00080000U)
#define CSL_APP_CANCFG_ILS_DRXL_SHIFT                                          (0x00000013U)
#define CSL_APP_CANCFG_ILS_DRXL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_DRXL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_BECL_MASK                                           (0x00100000U)
#define CSL_APP_CANCFG_ILS_BECL_SHIFT                                          (0x00000014U)
#define CSL_APP_CANCFG_ILS_BECL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_BECL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_BEUL_MASK                                           (0x00200000U)
#define CSL_APP_CANCFG_ILS_BEUL_SHIFT                                          (0x00000015U)
#define CSL_APP_CANCFG_ILS_BEUL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_BEUL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_ELOL_MASK                                           (0x00400000U)
#define CSL_APP_CANCFG_ILS_ELOL_SHIFT                                          (0x00000016U)
#define CSL_APP_CANCFG_ILS_ELOL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_ELOL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_EPL_MASK                                            (0x00800000U)
#define CSL_APP_CANCFG_ILS_EPL_SHIFT                                           (0x00000017U)
#define CSL_APP_CANCFG_ILS_EPL_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_ILS_EPL_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_ILS_EWL_MASK                                            (0x01000000U)
#define CSL_APP_CANCFG_ILS_EWL_SHIFT                                           (0x00000018U)
#define CSL_APP_CANCFG_ILS_EWL_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_ILS_EWL_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_ILS_BOL_MASK                                            (0x02000000U)
#define CSL_APP_CANCFG_ILS_BOL_SHIFT                                           (0x00000019U)
#define CSL_APP_CANCFG_ILS_BOL_RESETVAL                                        (0x00000000U)
#define CSL_APP_CANCFG_ILS_BOL_MAX                                             (0x00000001U)

#define CSL_APP_CANCFG_ILS_WDIL_MASK                                           (0x04000000U)
#define CSL_APP_CANCFG_ILS_WDIL_SHIFT                                          (0x0000001AU)
#define CSL_APP_CANCFG_ILS_WDIL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_WDIL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_PEAL_MASK                                           (0x08000000U)
#define CSL_APP_CANCFG_ILS_PEAL_SHIFT                                          (0x0000001BU)
#define CSL_APP_CANCFG_ILS_PEAL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_PEAL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_PEDL_MASK                                           (0x10000000U)
#define CSL_APP_CANCFG_ILS_PEDL_SHIFT                                          (0x0000001CU)
#define CSL_APP_CANCFG_ILS_PEDL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_PEDL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_ARAL_MASK                                           (0x20000000U)
#define CSL_APP_CANCFG_ILS_ARAL_SHIFT                                          (0x0000001DU)
#define CSL_APP_CANCFG_ILS_ARAL_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_ARAL_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_ILS_NU32_MASK                                           (0xC0000000U)
#define CSL_APP_CANCFG_ILS_NU32_SHIFT                                          (0x0000001EU)
#define CSL_APP_CANCFG_ILS_NU32_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILS_NU32_MAX                                            (0x00000003U)

#define CSL_APP_CANCFG_ILS_RESETVAL                                            (0x00000000U)

/* ILE */

#define CSL_APP_CANCFG_ILE_EINT0_MASK                                          (0x00000001U)
#define CSL_APP_CANCFG_ILE_EINT0_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_ILE_EINT0_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILE_EINT0_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILE_EINT1_MASK                                          (0x00000002U)
#define CSL_APP_CANCFG_ILE_EINT1_SHIFT                                         (0x00000001U)
#define CSL_APP_CANCFG_ILE_EINT1_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_ILE_EINT1_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_ILE_NU33_MASK                                           (0xFFFFFFFCU)
#define CSL_APP_CANCFG_ILE_NU33_SHIFT                                          (0x00000002U)
#define CSL_APP_CANCFG_ILE_NU33_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_ILE_NU33_MAX                                            (0x3FFFFFFFU)

#define CSL_APP_CANCFG_ILE_RESETVAL                                            (0x00000000U)

/* RES05 */

#define CSL_APP_CANCFG_RES05_RES05_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES05_RES05_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES05_RES05_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES05_RES05_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES05_RESETVAL                                          (0x00000000U)

/* RES06 */

#define CSL_APP_CANCFG_RES06_RES06_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES06_RES06_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES06_RES06_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES06_RES06_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES06_RESETVAL                                          (0x00000000U)

/* RES07 */

#define CSL_APP_CANCFG_RES07_RES07_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES07_RES07_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES07_RES07_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES07_RES07_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES07_RESETVAL                                          (0x00000000U)

/* RES08 */

#define CSL_APP_CANCFG_RES08_RES08_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES08_RES08_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES08_RES08_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES08_RES08_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES08_RESETVAL                                          (0x00000000U)

/* RES09 */

#define CSL_APP_CANCFG_RES09_RES09_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES09_RES09_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES09_RES09_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES09_RES09_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES09_RESETVAL                                          (0x00000000U)

/* RES10 */

#define CSL_APP_CANCFG_RES10_RES10_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES10_RES10_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES10_RES10_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES10_RES10_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES10_RESETVAL                                          (0x00000000U)

/* RES11 */

#define CSL_APP_CANCFG_RES11_RES11_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES11_RES11_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES11_RES11_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES11_RES11_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES11_RESETVAL                                          (0x00000000U)

/* RES12 */

#define CSL_APP_CANCFG_RES12_RES12_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES12_RES12_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES12_RES12_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES12_RES12_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES12_RESETVAL                                          (0x00000000U)

/* GFC */

#define CSL_APP_CANCFG_GFC_RRFE_MASK                                           (0x00000001U)
#define CSL_APP_CANCFG_GFC_RRFE_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_GFC_RRFE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_GFC_RRFE_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_GFC_RRFS_MASK                                           (0x00000002U)
#define CSL_APP_CANCFG_GFC_RRFS_SHIFT                                          (0x00000001U)
#define CSL_APP_CANCFG_GFC_RRFS_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_GFC_RRFS_MAX                                            (0x00000001U)

#define CSL_APP_CANCFG_GFC_ANFE_MASK                                           (0x0000000CU)
#define CSL_APP_CANCFG_GFC_ANFE_SHIFT                                          (0x00000002U)
#define CSL_APP_CANCFG_GFC_ANFE_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_GFC_ANFE_MAX                                            (0x00000003U)

#define CSL_APP_CANCFG_GFC_ANFS_MASK                                           (0x00000030U)
#define CSL_APP_CANCFG_GFC_ANFS_SHIFT                                          (0x00000004U)
#define CSL_APP_CANCFG_GFC_ANFS_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_GFC_ANFS_MAX                                            (0x00000003U)

#define CSL_APP_CANCFG_GFC_NU34_MASK                                           (0xFFFFFFC0U)
#define CSL_APP_CANCFG_GFC_NU34_SHIFT                                          (0x00000006U)
#define CSL_APP_CANCFG_GFC_NU34_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_GFC_NU34_MAX                                            (0x03FFFFFFU)

#define CSL_APP_CANCFG_GFC_RESETVAL                                            (0x00000000U)

/* SIDFC */

#define CSL_APP_CANCFG_SIDFC_NU35_MASK                                         (0x00000003U)
#define CSL_APP_CANCFG_SIDFC_NU35_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_SIDFC_NU35_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SIDFC_NU35_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_SIDFC_FLSSA_S_MASK                                      (0x0000FFFCU)
#define CSL_APP_CANCFG_SIDFC_FLSSA_S_SHIFT                                     (0x00000002U)
#define CSL_APP_CANCFG_SIDFC_FLSSA_S_RESETVAL                                  (0x00000000U)
#define CSL_APP_CANCFG_SIDFC_FLSSA_S_MAX                                       (0x00003FFFU)

#define CSL_APP_CANCFG_SIDFC_LSS_S_MASK                                        (0x00FF0000U)
#define CSL_APP_CANCFG_SIDFC_LSS_S_SHIFT                                       (0x00000010U)
#define CSL_APP_CANCFG_SIDFC_LSS_S_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_SIDFC_LSS_S_MAX                                         (0x000000FFU)

#define CSL_APP_CANCFG_SIDFC_NU36_MASK                                         (0xFF000000U)
#define CSL_APP_CANCFG_SIDFC_NU36_SHIFT                                        (0x00000018U)
#define CSL_APP_CANCFG_SIDFC_NU36_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_SIDFC_NU36_MAX                                          (0x000000FFU)

#define CSL_APP_CANCFG_SIDFC_RESETVAL                                          (0x00000000U)

/* XIDFC */

#define CSL_APP_CANCFG_XIDFC_NU37_MASK                                         (0x00000003U)
#define CSL_APP_CANCFG_XIDFC_NU37_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_XIDFC_NU37_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_XIDFC_NU37_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_XIDFC_FLSSA_X_MASK                                      (0x0000FFFCU)
#define CSL_APP_CANCFG_XIDFC_FLSSA_X_SHIFT                                     (0x00000002U)
#define CSL_APP_CANCFG_XIDFC_FLSSA_X_RESETVAL                                  (0x00000000U)
#define CSL_APP_CANCFG_XIDFC_FLSSA_X_MAX                                       (0x00003FFFU)

#define CSL_APP_CANCFG_XIDFC_LSS_X_MASK                                        (0x007F0000U)
#define CSL_APP_CANCFG_XIDFC_LSS_X_SHIFT                                       (0x00000010U)
#define CSL_APP_CANCFG_XIDFC_LSS_X_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_XIDFC_LSS_X_MAX                                         (0x0000007FU)

#define CSL_APP_CANCFG_XIDFC_NU38_MASK                                         (0xFF800000U)
#define CSL_APP_CANCFG_XIDFC_NU38_SHIFT                                        (0x00000017U)
#define CSL_APP_CANCFG_XIDFC_NU38_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_XIDFC_NU38_MAX                                          (0x000001FFU)

#define CSL_APP_CANCFG_XIDFC_RESETVAL                                          (0x00000000U)

/* RES13 */

#define CSL_APP_CANCFG_RES13_RES13_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES13_RES13_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES13_RES13_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES13_RES13_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES13_RESETVAL                                          (0x00000000U)

/* XIDAM */

#define CSL_APP_CANCFG_XIDAM_EIDM_MASK                                         (0x1FFFFFFFU)
#define CSL_APP_CANCFG_XIDAM_EIDM_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_XIDAM_EIDM_RESETVAL                                     (0x1FFFFFFFU)
#define CSL_APP_CANCFG_XIDAM_EIDM_MAX                                          (0x1FFFFFFFU)

#define CSL_APP_CANCFG_XIDAM_NU39_MASK                                         (0xE0000000U)
#define CSL_APP_CANCFG_XIDAM_NU39_SHIFT                                        (0x0000001DU)
#define CSL_APP_CANCFG_XIDAM_NU39_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_XIDAM_NU39_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_XIDAM_RESETVAL                                          (0x1FFFFFFFU)

/* HPMS */

#define CSL_APP_CANCFG_HPMS_BIDX_MASK                                          (0x0000003FU)
#define CSL_APP_CANCFG_HPMS_BIDX_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_HPMS_BIDX_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_HPMS_BIDX_MAX                                           (0x0000003FU)

#define CSL_APP_CANCFG_HPMS_MSI_MASK                                           (0x000000C0U)
#define CSL_APP_CANCFG_HPMS_MSI_SHIFT                                          (0x00000006U)
#define CSL_APP_CANCFG_HPMS_MSI_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_HPMS_MSI_MAX                                            (0x00000003U)

#define CSL_APP_CANCFG_HPMS_FIDX_MASK                                          (0x00007F00U)
#define CSL_APP_CANCFG_HPMS_FIDX_SHIFT                                         (0x00000008U)
#define CSL_APP_CANCFG_HPMS_FIDX_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_HPMS_FIDX_MAX                                           (0x0000007FU)

#define CSL_APP_CANCFG_HPMS_FLST_MASK                                          (0x00008000U)
#define CSL_APP_CANCFG_HPMS_FLST_SHIFT                                         (0x0000000FU)
#define CSL_APP_CANCFG_HPMS_FLST_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_HPMS_FLST_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_HPMS_NU40_MASK                                          (0xFFFF0000U)
#define CSL_APP_CANCFG_HPMS_NU40_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_HPMS_NU40_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_HPMS_NU40_MAX                                           (0x0000FFFFU)

#define CSL_APP_CANCFG_HPMS_RESETVAL                                           (0x00000000U)

/* NDAT1 */

#define CSL_APP_CANCFG_NDAT1_ND0_31_MASK                                       (0xFFFFFFFFU)
#define CSL_APP_CANCFG_NDAT1_ND0_31_SHIFT                                      (0x00000000U)
#define CSL_APP_CANCFG_NDAT1_ND0_31_RESETVAL                                   (0x00000000U)
#define CSL_APP_CANCFG_NDAT1_ND0_31_MAX                                        (0xFFFFFFFFU)

#define CSL_APP_CANCFG_NDAT1_RESETVAL                                          (0x00000000U)

/* NDAT2 */

#define CSL_APP_CANCFG_NDAT2_ND32_63_MASK                                      (0xFFFFFFFFU)
#define CSL_APP_CANCFG_NDAT2_ND32_63_SHIFT                                     (0x00000000U)
#define CSL_APP_CANCFG_NDAT2_ND32_63_RESETVAL                                  (0x00000000U)
#define CSL_APP_CANCFG_NDAT2_ND32_63_MAX                                       (0xFFFFFFFFU)

#define CSL_APP_CANCFG_NDAT2_RESETVAL                                          (0x00000000U)

/* RXF0C */

#define CSL_APP_CANCFG_RXF0C_NU41_MASK                                         (0x00000003U)
#define CSL_APP_CANCFG_RXF0C_NU41_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_NU41_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_NU41_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_RXF0C_F0SA_MASK                                         (0x00007FFCU)
#define CSL_APP_CANCFG_RXF0C_F0SA_SHIFT                                        (0x00000002U)
#define CSL_APP_CANCFG_RXF0C_F0SA_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_F0SA_MAX                                          (0x00001FFFU)

#define CSL_APP_CANCFG_RXF0C_NU42_MASK                                         (0x00008000U)
#define CSL_APP_CANCFG_RXF0C_NU42_SHIFT                                        (0x0000000FU)
#define CSL_APP_CANCFG_RXF0C_NU42_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_NU42_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF0C_F0S_MASK                                          (0x007F0000U)
#define CSL_APP_CANCFG_RXF0C_F0S_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_RXF0C_F0S_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_F0S_MAX                                           (0x0000007FU)

#define CSL_APP_CANCFG_RXF0C_NU42_1_MASK                                       (0x00800000U)
#define CSL_APP_CANCFG_RXF0C_NU42_1_SHIFT                                      (0x00000017U)
#define CSL_APP_CANCFG_RXF0C_NU42_1_RESETVAL                                   (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_NU42_1_MAX                                        (0x00000001U)

#define CSL_APP_CANCFG_RXF0C_F0WM_MASK                                         (0x7F000000U)
#define CSL_APP_CANCFG_RXF0C_F0WM_SHIFT                                        (0x00000018U)
#define CSL_APP_CANCFG_RXF0C_F0WM_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_F0WM_MAX                                          (0x0000007FU)

#define CSL_APP_CANCFG_RXF0C_F0OM_MASK                                         (0x80000000U)
#define CSL_APP_CANCFG_RXF0C_F0OM_SHIFT                                        (0x0000001FU)
#define CSL_APP_CANCFG_RXF0C_F0OM_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0C_F0OM_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF0C_RESETVAL                                          (0x00000000U)

/* RXF0S */

#define CSL_APP_CANCFG_RXF0S_F0FL_MASK                                         (0x0000007FU)
#define CSL_APP_CANCFG_RXF0S_F0FL_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_F0FL_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_F0FL_MAX                                          (0x0000007FU)

#define CSL_APP_CANCFG_RXF0S_NU43_MASK                                         (0x00000080U)
#define CSL_APP_CANCFG_RXF0S_NU43_SHIFT                                        (0x00000007U)
#define CSL_APP_CANCFG_RXF0S_NU43_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_NU43_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF0S_F0GI_MASK                                         (0x00003F00U)
#define CSL_APP_CANCFG_RXF0S_F0GI_SHIFT                                        (0x00000008U)
#define CSL_APP_CANCFG_RXF0S_F0GI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_F0GI_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF0S_NU44_MASK                                         (0x0000C000U)
#define CSL_APP_CANCFG_RXF0S_NU44_SHIFT                                        (0x0000000EU)
#define CSL_APP_CANCFG_RXF0S_NU44_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_NU44_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_RXF0S_F0PI_MASK                                         (0x003F0000U)
#define CSL_APP_CANCFG_RXF0S_F0PI_SHIFT                                        (0x00000010U)
#define CSL_APP_CANCFG_RXF0S_F0PI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_F0PI_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF0S_NU45_MASK                                         (0x00C00000U)
#define CSL_APP_CANCFG_RXF0S_NU45_SHIFT                                        (0x00000016U)
#define CSL_APP_CANCFG_RXF0S_NU45_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_NU45_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_RXF0S_F0F_MASK                                          (0x01000000U)
#define CSL_APP_CANCFG_RXF0S_F0F_SHIFT                                         (0x00000018U)
#define CSL_APP_CANCFG_RXF0S_F0F_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_F0F_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_RXF0S_RF0L_MASK                                         (0x02000000U)
#define CSL_APP_CANCFG_RXF0S_RF0L_SHIFT                                        (0x00000019U)
#define CSL_APP_CANCFG_RXF0S_RF0L_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_RF0L_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF0S_NU46_MASK                                         (0xFC000000U)
#define CSL_APP_CANCFG_RXF0S_NU46_SHIFT                                        (0x0000001AU)
#define CSL_APP_CANCFG_RXF0S_NU46_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0S_NU46_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF0S_RESETVAL                                          (0x00000000U)

/* RXF0A */

#define CSL_APP_CANCFG_RXF0A_F0AI_MASK                                         (0x0000003FU)
#define CSL_APP_CANCFG_RXF0A_F0AI_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_RXF0A_F0AI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0A_F0AI_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF0A_NU47_MASK                                         (0xFFFFFFC0U)
#define CSL_APP_CANCFG_RXF0A_NU47_SHIFT                                        (0x00000006U)
#define CSL_APP_CANCFG_RXF0A_NU47_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF0A_NU47_MAX                                          (0x03FFFFFFU)

#define CSL_APP_CANCFG_RXF0A_RESETVAL                                          (0x00000000U)

/* RXBC */

#define CSL_APP_CANCFG_RXBC_NU48_MASK                                          (0x00000003U)
#define CSL_APP_CANCFG_RXBC_NU48_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_RXBC_NU48_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_RXBC_NU48_MAX                                           (0x00000003U)

#define CSL_APP_CANCFG_RXBC_RBSA_MASK                                          (0x0000FFFCU)
#define CSL_APP_CANCFG_RXBC_RBSA_SHIFT                                         (0x00000002U)
#define CSL_APP_CANCFG_RXBC_RBSA_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_RXBC_RBSA_MAX                                           (0x00003FFFU)

#define CSL_APP_CANCFG_RXBC_NU49_MASK                                          (0xFFFF0000U)
#define CSL_APP_CANCFG_RXBC_NU49_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_RXBC_NU49_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_RXBC_NU49_MAX                                           (0x0000FFFFU)

#define CSL_APP_CANCFG_RXBC_RESETVAL                                           (0x00000000U)

/* RXF1C */

#define CSL_APP_CANCFG_RXF1C_NU499_MASK                                        (0x00000003U)
#define CSL_APP_CANCFG_RXF1C_NU499_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_NU499_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_NU499_MAX                                         (0x00000003U)

#define CSL_APP_CANCFG_RXF1C_F1SA_MASK                                         (0x00007FFCU)
#define CSL_APP_CANCFG_RXF1C_F1SA_SHIFT                                        (0x00000002U)
#define CSL_APP_CANCFG_RXF1C_F1SA_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_F1SA_MAX                                          (0x00001FFFU)

#define CSL_APP_CANCFG_RXF1C_NU50_MASK                                         (0x00008000U)
#define CSL_APP_CANCFG_RXF1C_NU50_SHIFT                                        (0x0000000FU)
#define CSL_APP_CANCFG_RXF1C_NU50_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_NU50_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF1C_F1S_MASK                                          (0x007F0000U)
#define CSL_APP_CANCFG_RXF1C_F1S_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_RXF1C_F1S_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_F1S_MAX                                           (0x0000007FU)

#define CSL_APP_CANCFG_RXF1C_NU50_1_MASK                                       (0x00800000U)
#define CSL_APP_CANCFG_RXF1C_NU50_1_SHIFT                                      (0x00000017U)
#define CSL_APP_CANCFG_RXF1C_NU50_1_RESETVAL                                   (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_NU50_1_MAX                                        (0x00000001U)

#define CSL_APP_CANCFG_RXF1C_F1WM_MASK                                         (0x7F000000U)
#define CSL_APP_CANCFG_RXF1C_F1WM_SHIFT                                        (0x00000018U)
#define CSL_APP_CANCFG_RXF1C_F1WM_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_F1WM_MAX                                          (0x0000007FU)

#define CSL_APP_CANCFG_RXF1C_F1OM_MASK                                         (0x80000000U)
#define CSL_APP_CANCFG_RXF1C_F1OM_SHIFT                                        (0x0000001FU)
#define CSL_APP_CANCFG_RXF1C_F1OM_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1C_F1OM_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF1C_RESETVAL                                          (0x00000000U)

/* RXF1S */

#define CSL_APP_CANCFG_RXF1S_F1FL_MASK                                         (0x0000007FU)
#define CSL_APP_CANCFG_RXF1S_F1FL_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_F1FL_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_F1FL_MAX                                          (0x0000007FU)

#define CSL_APP_CANCFG_RXF1S_NU51_MASK                                         (0x00000080U)
#define CSL_APP_CANCFG_RXF1S_NU51_SHIFT                                        (0x00000007U)
#define CSL_APP_CANCFG_RXF1S_NU51_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_NU51_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF1S_F1GI_MASK                                         (0x00003F00U)
#define CSL_APP_CANCFG_RXF1S_F1GI_SHIFT                                        (0x00000008U)
#define CSL_APP_CANCFG_RXF1S_F1GI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_F1GI_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF1S_NU52_MASK                                         (0x0000C000U)
#define CSL_APP_CANCFG_RXF1S_NU52_SHIFT                                        (0x0000000EU)
#define CSL_APP_CANCFG_RXF1S_NU52_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_NU52_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_RXF1S_F1PI_MASK                                         (0x003F0000U)
#define CSL_APP_CANCFG_RXF1S_F1PI_SHIFT                                        (0x00000010U)
#define CSL_APP_CANCFG_RXF1S_F1PI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_F1PI_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF1S_NU53_MASK                                         (0x00C00000U)
#define CSL_APP_CANCFG_RXF1S_NU53_SHIFT                                        (0x00000016U)
#define CSL_APP_CANCFG_RXF1S_NU53_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_NU53_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_RXF1S_F1F_MASK                                          (0x01000000U)
#define CSL_APP_CANCFG_RXF1S_F1F_SHIFT                                         (0x00000018U)
#define CSL_APP_CANCFG_RXF1S_F1F_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_F1F_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_RXF1S_RF1L_MASK                                         (0x02000000U)
#define CSL_APP_CANCFG_RXF1S_RF1L_SHIFT                                        (0x00000019U)
#define CSL_APP_CANCFG_RXF1S_RF1L_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_RF1L_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXF1S_NU54_MASK                                         (0xFC000000U)
#define CSL_APP_CANCFG_RXF1S_NU54_SHIFT                                        (0x0000001AU)
#define CSL_APP_CANCFG_RXF1S_NU54_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1S_NU54_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF1S_RESETVAL                                          (0x00000000U)

/* RXF1A */

#define CSL_APP_CANCFG_RXF1A_F1AI_MASK                                         (0x0000003FU)
#define CSL_APP_CANCFG_RXF1A_F1AI_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_RXF1A_F1AI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1A_F1AI_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_RXF1A_NU55_MASK                                         (0xFFFFFFC0U)
#define CSL_APP_CANCFG_RXF1A_NU55_SHIFT                                        (0x00000006U)
#define CSL_APP_CANCFG_RXF1A_NU55_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXF1A_NU55_MAX                                          (0x03FFFFFFU)

#define CSL_APP_CANCFG_RXF1A_RESETVAL                                          (0x00000000U)

/* RXESC */

#define CSL_APP_CANCFG_RXESC_F0DS_MASK                                         (0x00000007U)
#define CSL_APP_CANCFG_RXESC_F0DS_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_RXESC_F0DS_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXESC_F0DS_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_RXESC_NU56_MASK                                         (0x00000008U)
#define CSL_APP_CANCFG_RXESC_NU56_SHIFT                                        (0x00000003U)
#define CSL_APP_CANCFG_RXESC_NU56_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXESC_NU56_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXESC_F1DS_MASK                                         (0x00000070U)
#define CSL_APP_CANCFG_RXESC_F1DS_SHIFT                                        (0x00000004U)
#define CSL_APP_CANCFG_RXESC_F1DS_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXESC_F1DS_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_RXESC_NU57_MASK                                         (0x00000080U)
#define CSL_APP_CANCFG_RXESC_NU57_SHIFT                                        (0x00000007U)
#define CSL_APP_CANCFG_RXESC_NU57_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXESC_NU57_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_RXESC_RBDS_MASK                                         (0x00000700U)
#define CSL_APP_CANCFG_RXESC_RBDS_SHIFT                                        (0x00000008U)
#define CSL_APP_CANCFG_RXESC_RBDS_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXESC_RBDS_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_RXESC_NU58_MASK                                         (0xFFFFF800U)
#define CSL_APP_CANCFG_RXESC_NU58_SHIFT                                        (0x0000000BU)
#define CSL_APP_CANCFG_RXESC_NU58_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_RXESC_NU58_MAX                                          (0x001FFFFFU)

#define CSL_APP_CANCFG_RXESC_RESETVAL                                          (0x00000000U)

/* TXBC */

#define CSL_APP_CANCFG_TXBC_NU59_MASK                                          (0x00000003U)
#define CSL_APP_CANCFG_TXBC_NU59_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_TXBC_NU59_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBC_NU59_MAX                                           (0x00000003U)

#define CSL_APP_CANCFG_TXBC_TBSA_MASK                                          (0x0000FFFCU)
#define CSL_APP_CANCFG_TXBC_TBSA_SHIFT                                         (0x00000002U)
#define CSL_APP_CANCFG_TXBC_TBSA_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBC_TBSA_MAX                                           (0x00003FFFU)

#define CSL_APP_CANCFG_TXBC_NDTB_MASK                                          (0x003F0000U)
#define CSL_APP_CANCFG_TXBC_NDTB_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_TXBC_NDTB_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBC_NDTB_MAX                                           (0x0000003FU)

#define CSL_APP_CANCFG_TXBC_NU60_MASK                                          (0x00C00000U)
#define CSL_APP_CANCFG_TXBC_NU60_SHIFT                                         (0x00000016U)
#define CSL_APP_CANCFG_TXBC_NU60_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBC_NU60_MAX                                           (0x00000003U)

#define CSL_APP_CANCFG_TXBC_TFQS_MASK                                          (0x3F000000U)
#define CSL_APP_CANCFG_TXBC_TFQS_SHIFT                                         (0x00000018U)
#define CSL_APP_CANCFG_TXBC_TFQS_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBC_TFQS_MAX                                           (0x0000003FU)

#define CSL_APP_CANCFG_TXBC_TFQM_MASK                                          (0x40000000U)
#define CSL_APP_CANCFG_TXBC_TFQM_SHIFT                                         (0x0000001EU)
#define CSL_APP_CANCFG_TXBC_TFQM_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBC_TFQM_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_TXBC_NU61_MASK                                          (0x80000000U)
#define CSL_APP_CANCFG_TXBC_NU61_SHIFT                                         (0x0000001FU)
#define CSL_APP_CANCFG_TXBC_NU61_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBC_NU61_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_TXBC_RESETVAL                                           (0x00000000U)

/* TXFQS */

#define CSL_APP_CANCFG_TXFQS_TFFL_MASK                                         (0x0000003FU)
#define CSL_APP_CANCFG_TXFQS_TFFL_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_TFFL_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_TFFL_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_TXFQS_NU62_MASK                                         (0x000000C0U)
#define CSL_APP_CANCFG_TXFQS_NU62_SHIFT                                        (0x00000006U)
#define CSL_APP_CANCFG_TXFQS_NU62_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_NU62_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_TXFQS_TFGI_MASK                                         (0x00001F00U)
#define CSL_APP_CANCFG_TXFQS_TFGI_SHIFT                                        (0x00000008U)
#define CSL_APP_CANCFG_TXFQS_TFGI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_TFGI_MAX                                          (0x0000001FU)

#define CSL_APP_CANCFG_TXFQS_NU63_MASK                                         (0x0000E000U)
#define CSL_APP_CANCFG_TXFQS_NU63_SHIFT                                        (0x0000000DU)
#define CSL_APP_CANCFG_TXFQS_NU63_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_NU63_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_TXFQS_TFQPI_MASK                                        (0x001F0000U)
#define CSL_APP_CANCFG_TXFQS_TFQPI_SHIFT                                       (0x00000010U)
#define CSL_APP_CANCFG_TXFQS_TFQPI_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_TFQPI_MAX                                         (0x0000001FU)

#define CSL_APP_CANCFG_TXFQS_TFQF_MASK                                         (0x00200000U)
#define CSL_APP_CANCFG_TXFQS_TFQF_SHIFT                                        (0x00000015U)
#define CSL_APP_CANCFG_TXFQS_TFQF_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_TFQF_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_TXFQS_NU64_MASK                                         (0xFFC00000U)
#define CSL_APP_CANCFG_TXFQS_NU64_SHIFT                                        (0x00000016U)
#define CSL_APP_CANCFG_TXFQS_NU64_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXFQS_NU64_MAX                                          (0x000003FFU)

#define CSL_APP_CANCFG_TXFQS_RESETVAL                                          (0x00000000U)

/* TXESC */

#define CSL_APP_CANCFG_TXESC_TBDS_MASK                                         (0x00000007U)
#define CSL_APP_CANCFG_TXESC_TBDS_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_TXESC_TBDS_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXESC_TBDS_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_TXESC_NU65_MASK                                         (0xFFFFFFF8U)
#define CSL_APP_CANCFG_TXESC_NU65_SHIFT                                        (0x00000003U)
#define CSL_APP_CANCFG_TXESC_NU65_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXESC_NU65_MAX                                          (0x1FFFFFFFU)

#define CSL_APP_CANCFG_TXESC_RESETVAL                                          (0x00000000U)

/* TXBRP */

#define CSL_APP_CANCFG_TXBRP_TRP_MASK                                          (0xFFFFFFFFU)
#define CSL_APP_CANCFG_TXBRP_TRP_SHIFT                                         (0x00000000U)
#define CSL_APP_CANCFG_TXBRP_TRP_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXBRP_TRP_MAX                                           (0xFFFFFFFFU)

#define CSL_APP_CANCFG_TXBRP_RESETVAL                                          (0x00000000U)

/* TXBAR */

#define CSL_APP_CANCFG_TXBAR_AR_MASK                                           (0xFFFFFFFFU)
#define CSL_APP_CANCFG_TXBAR_AR_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_TXBAR_AR_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TXBAR_AR_MAX                                            (0xFFFFFFFFU)

#define CSL_APP_CANCFG_TXBAR_RESETVAL                                          (0x00000000U)

/* TXBCR */

#define CSL_APP_CANCFG_TXBCR_CR_MASK                                           (0xFFFFFFFFU)
#define CSL_APP_CANCFG_TXBCR_CR_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_TXBCR_CR_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TXBCR_CR_MAX                                            (0xFFFFFFFFU)

#define CSL_APP_CANCFG_TXBCR_RESETVAL                                          (0x00000000U)

/* TXBTO */

#define CSL_APP_CANCFG_TXBTO_TO_MASK                                           (0xFFFFFFFFU)
#define CSL_APP_CANCFG_TXBTO_TO_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_TXBTO_TO_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TXBTO_TO_MAX                                            (0xFFFFFFFFU)

#define CSL_APP_CANCFG_TXBTO_RESETVAL                                          (0x00000000U)

/* TXBCF */

#define CSL_APP_CANCFG_TXBCF_CF_MASK                                           (0xFFFFFFFFU)
#define CSL_APP_CANCFG_TXBCF_CF_SHIFT                                          (0x00000000U)
#define CSL_APP_CANCFG_TXBCF_CF_RESETVAL                                       (0x00000000U)
#define CSL_APP_CANCFG_TXBCF_CF_MAX                                            (0xFFFFFFFFU)

#define CSL_APP_CANCFG_TXBCF_RESETVAL                                          (0x00000000U)

/* TXBTIE */

#define CSL_APP_CANCFG_TXBTIE_TIE_MASK                                         (0xFFFFFFFFU)
#define CSL_APP_CANCFG_TXBTIE_TIE_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_TXBTIE_TIE_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXBTIE_TIE_MAX                                          (0xFFFFFFFFU)

#define CSL_APP_CANCFG_TXBTIE_RESETVAL                                         (0x00000000U)

/* TXBCIE */

#define CSL_APP_CANCFG_TXBCIE_CFIE_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_TXBCIE_CFIE_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_TXBCIE_CFIE_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_TXBCIE_CFIE_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_TXBCIE_RESETVAL                                         (0x00000000U)

/* RES14 */

#define CSL_APP_CANCFG_RES14_RES14_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES14_RES14_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES14_RES14_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES14_RES14_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES14_RESETVAL                                          (0x00000000U)

/* RES15 */

#define CSL_APP_CANCFG_RES15_RES15_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES15_RES15_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES15_RES15_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES15_RES15_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES15_RESETVAL                                          (0x00000000U)

/* TXEFC */

#define CSL_APP_CANCFG_TXEFC_NU66_MASK                                         (0x00000003U)
#define CSL_APP_CANCFG_TXEFC_NU66_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_TXEFC_NU66_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFC_NU66_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_TXEFC_EFSA_MASK                                         (0x0000FFFCU)
#define CSL_APP_CANCFG_TXEFC_EFSA_SHIFT                                        (0x00000002U)
#define CSL_APP_CANCFG_TXEFC_EFSA_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFC_EFSA_MAX                                          (0x00003FFFU)

#define CSL_APP_CANCFG_TXEFC_EFS_MASK                                          (0x003F0000U)
#define CSL_APP_CANCFG_TXEFC_EFS_SHIFT                                         (0x00000010U)
#define CSL_APP_CANCFG_TXEFC_EFS_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXEFC_EFS_MAX                                           (0x0000003FU)

#define CSL_APP_CANCFG_TXEFC_NU67_MASK                                         (0x00C00000U)
#define CSL_APP_CANCFG_TXEFC_NU67_SHIFT                                        (0x00000016U)
#define CSL_APP_CANCFG_TXEFC_NU67_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFC_NU67_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_TXEFC_EFWM_MASK                                         (0x3F000000U)
#define CSL_APP_CANCFG_TXEFC_EFWM_SHIFT                                        (0x00000018U)
#define CSL_APP_CANCFG_TXEFC_EFWM_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFC_EFWM_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_TXEFC_NU68_MASK                                         (0xC0000000U)
#define CSL_APP_CANCFG_TXEFC_NU68_SHIFT                                        (0x0000001EU)
#define CSL_APP_CANCFG_TXEFC_NU68_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFC_NU68_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_TXEFC_RESETVAL                                          (0x00000000U)

/* TXEFS */

#define CSL_APP_CANCFG_TXEFS_EFFL_MASK                                         (0x0000003FU)
#define CSL_APP_CANCFG_TXEFS_EFFL_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_EFFL_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_EFFL_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_TXEFS_NU69_MASK                                         (0x000000C0U)
#define CSL_APP_CANCFG_TXEFS_NU69_SHIFT                                        (0x00000006U)
#define CSL_APP_CANCFG_TXEFS_NU69_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_NU69_MAX                                          (0x00000003U)

#define CSL_APP_CANCFG_TXEFS_EFGI_MASK                                         (0x00001F00U)
#define CSL_APP_CANCFG_TXEFS_EFGI_SHIFT                                        (0x00000008U)
#define CSL_APP_CANCFG_TXEFS_EFGI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_EFGI_MAX                                          (0x0000001FU)

#define CSL_APP_CANCFG_TXEFS_NU70_MASK                                         (0x0000E000U)
#define CSL_APP_CANCFG_TXEFS_NU70_SHIFT                                        (0x0000000DU)
#define CSL_APP_CANCFG_TXEFS_NU70_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_NU70_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_TXEFS_EFPI_MASK                                         (0x001F0000U)
#define CSL_APP_CANCFG_TXEFS_EFPI_SHIFT                                        (0x00000010U)
#define CSL_APP_CANCFG_TXEFS_EFPI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_EFPI_MAX                                          (0x0000001FU)

#define CSL_APP_CANCFG_TXEFS_NU71_MASK                                         (0x00E00000U)
#define CSL_APP_CANCFG_TXEFS_NU71_SHIFT                                        (0x00000015U)
#define CSL_APP_CANCFG_TXEFS_NU71_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_NU71_MAX                                          (0x00000007U)

#define CSL_APP_CANCFG_TXEFS_EFF_MASK                                          (0x01000000U)
#define CSL_APP_CANCFG_TXEFS_EFF_SHIFT                                         (0x00000018U)
#define CSL_APP_CANCFG_TXEFS_EFF_RESETVAL                                      (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_EFF_MAX                                           (0x00000001U)

#define CSL_APP_CANCFG_TXEFS_TEFL_MASK                                         (0x02000000U)
#define CSL_APP_CANCFG_TXEFS_TEFL_SHIFT                                        (0x00000019U)
#define CSL_APP_CANCFG_TXEFS_TEFL_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_TEFL_MAX                                          (0x00000001U)

#define CSL_APP_CANCFG_TXEFS_NU72_MASK                                         (0xFC000000U)
#define CSL_APP_CANCFG_TXEFS_NU72_SHIFT                                        (0x0000001AU)
#define CSL_APP_CANCFG_TXEFS_NU72_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFS_NU72_MAX                                          (0x0000003FU)

#define CSL_APP_CANCFG_TXEFS_RESETVAL                                          (0x00000000U)

/* TXEFA */

#define CSL_APP_CANCFG_TXEFA_EFAI_MASK                                         (0x0000001FU)
#define CSL_APP_CANCFG_TXEFA_EFAI_SHIFT                                        (0x00000000U)
#define CSL_APP_CANCFG_TXEFA_EFAI_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFA_EFAI_MAX                                          (0x0000001FU)

#define CSL_APP_CANCFG_TXEFA_NU73_MASK                                         (0xFFFFFFE0U)
#define CSL_APP_CANCFG_TXEFA_NU73_SHIFT                                        (0x00000005U)
#define CSL_APP_CANCFG_TXEFA_NU73_RESETVAL                                     (0x00000000U)
#define CSL_APP_CANCFG_TXEFA_NU73_MAX                                          (0x07FFFFFFU)

#define CSL_APP_CANCFG_TXEFA_RESETVAL                                          (0x00000000U)

/* RES16 */

#define CSL_APP_CANCFG_RES16_RES16_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_CANCFG_RES16_RES16_SHIFT                                       (0x00000000U)
#define CSL_APP_CANCFG_RES16_RES16_RESETVAL                                    (0x00000000U)
#define CSL_APP_CANCFG_RES16_RES16_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_CANCFG_RES16_RESETVAL                                          (0x00000000U)

#define MCAN_STD_ID_MASK                                                       (0x7FFU)
#define MCAN_STD_ID_SHIFT                                                      (18U)

/* User defined Ranges */
#define CSL_APP_CANCFG_NBTP_NTSEG1_MIN                                         (0x1U)

#define CSL_MCAN_CCCR_CSA_SHIFT                                                (3U)
#define CSL_MCAN_CCCR_CSA_MASK                                                 (0x00000008U)
#define CSL_MCAN_CCCR_CSA_NO_ACK                                               (0U)
#define CSL_MCAN_CCCR_CSA_ACK                                                  (1U)

#define CSL_MCAN_NDAT1_CLEAR                                                   (0xFFFFFFFFU)
#define CSL_MCAN_NDAT2_CLEAR                                                   (0xFFFFFFFFU)

#define MCAN_MCAN_MSG_MEM                                                      (0x0U)

/* MCAN IP related Macros */
#define MCAN_TX_BUFFER_MAX_NUM                                                 (32U)
#define MCAN_RX_BUFFER_MAX_NUM                                                 (64U)
#define MCAN_RX_FIFO_0_MAX_NUM                                                 (64U)
#define MCAN_RX_FIFO_1_MAX_NUM                                                 (64U)
#define MCAN_BAUD_RATE_PRESCALER_MAX_NUM                                       (0x200U)

#ifdef __cplusplus
}
#endif
#endif
