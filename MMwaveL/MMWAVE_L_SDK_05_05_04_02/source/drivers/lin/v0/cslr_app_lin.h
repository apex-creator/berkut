/*
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
*/
#ifndef CSLR_APP_LIN_H_
#define CSLR_APP_LIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <drivers/hw_include/soc_config.h>
#include <drivers/hw_include/cslr_lin.h>


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t SCIGCR0;
    volatile uint32_t SCIGCR1;
    volatile uint32_t SCIGCR2;
    volatile uint32_t SCISETINT;
    volatile uint32_t SCICLEARINT;
    volatile uint32_t SCISETINTLVL;
    volatile uint32_t SCICLEARINTLVL;
    volatile uint32_t SCIFLR;
    volatile uint32_t SCIINTVECT0;
    volatile uint32_t SCIINTVECT1;
    volatile uint32_t SCIFORMAT;
    volatile uint32_t BRSR;
    volatile uint32_t SCIED;
    volatile uint32_t SCIRD;
    volatile uint32_t SCITD;
    volatile uint32_t SCIPIO0;
    volatile uint32_t SCIPIO1;
    volatile uint32_t SCIPIO2;
    volatile uint32_t SCIPIO3;
    volatile uint32_t SCIPIO4;
    volatile uint32_t SCIPIO5;
    volatile uint32_t SCIPIO6;
    volatile uint32_t SCIPIO7;
    volatile uint32_t SCIPIO8;
    volatile uint32_t LINCOMP;
    volatile uint32_t LINRD0;
    volatile uint32_t LINRD1;
    volatile uint32_t LINMASK;
    volatile uint32_t LINID;
    volatile uint32_t LINTD0;
    volatile uint32_t LINTD1;
    volatile uint32_t MBRSR;
    volatile uint32_t SCIPIO9;
    volatile uint32_t RESERVED;
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
    volatile uint32_t IODFTCTRL;
} CSL_app_linRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_APP_LIN_SCIGCR0                                                    (0x00000000U)
#define CSL_APP_LIN_SCIGCR1                                                    (0x00000004U)
#define CSL_APP_LIN_SCIGCR2                                                    (0x00000008U)
#define CSL_APP_LIN_SCISETINT                                                  (0x0000000CU)
#define CSL_APP_LIN_SCICLEARINT                                                (0x00000010U)
#define CSL_APP_LIN_SCISETINTLVL                                               (0x00000014U)
#define CSL_APP_LIN_SCICLEARINTLVL                                             (0x00000018U)
#define CSL_APP_LIN_SCIFLR                                                     (0x0000001CU)
#define CSL_APP_LIN_SCIINTVECT0                                                (0x00000020U)
#define CSL_APP_LIN_SCIINTVECT1                                                (0x00000024U)
#define CSL_APP_LIN_SCIFORMAT                                                  (0x00000028U)
#define CSL_APP_LIN_BRSR                                                       (0x0000002CU)
#define CSL_APP_LIN_SCIED                                                      (0x00000030U)
#define CSL_APP_LIN_SCIRD                                                      (0x00000034U)
#define CSL_APP_LIN_SCITD                                                      (0x00000038U)
#define CSL_APP_LIN_SCIPIO0                                                    (0x0000003CU)
#define CSL_APP_LIN_SCIPIO1                                                    (0x00000040U)
#define CSL_APP_LIN_SCIPIO2                                                    (0x00000044U)
#define CSL_APP_LIN_SCIPIO3                                                    (0x00000048U)
#define CSL_APP_LIN_SCIPIO4                                                    (0x0000004CU)
#define CSL_APP_LIN_SCIPIO5                                                    (0x00000050U)
#define CSL_APP_LIN_SCIPIO6                                                    (0x00000054U)
#define CSL_APP_LIN_SCIPIO7                                                    (0x00000058U)
#define CSL_APP_LIN_SCIPIO8                                                    (0x0000005CU)
#define CSL_APP_LIN_LINCOMP                                                    (0x00000060U)
#define CSL_APP_LIN_LINRD0                                                     (0x00000064U)
#define CSL_APP_LIN_LINRD1                                                     (0x00000068U)
#define CSL_APP_LIN_LINMASK                                                    (0x0000006CU)
#define CSL_APP_LIN_LINID                                                      (0x00000070U)
#define CSL_APP_LIN_LINTD0                                                     (0x00000074U)
#define CSL_APP_LIN_LINTD1                                                     (0x00000078U)
#define CSL_APP_LIN_MBRSR                                                      (0x0000007CU)
#define CSL_APP_LIN_SCIPIO9                                                    (0x00000080U)
#define CSL_APP_LIN_RESERVED                                                   (0x00000084U)
#define CSL_APP_LIN_RESERVED1                                                  (0x00000088U)
#define CSL_APP_LIN_RESERVED2                                                  (0x0000008CU)
#define CSL_APP_LIN_IODFTCTRL                                                  (0x00000090U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* SCIGCR0 */

#define CSL_APP_LIN_SCIGCR0_RESET_MASK                                         (0x00000001U)
#define CSL_APP_LIN_SCIGCR0_RESET_SHIFT                                        (0x00000000U)
#define CSL_APP_LIN_SCIGCR0_RESET_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR0_RESET_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIGCR0_RESERVED_MASK                                      (0x0000FFFEU)
#define CSL_APP_LIN_SCIGCR0_RESERVED_SHIFT                                     (0x00000001U)
#define CSL_APP_LIN_SCIGCR0_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIGCR0_RESERVED_MAX                                       (0x00007FFFU)

#define CSL_APP_LIN_SCIGCR0_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIGCR0_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIGCR0_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIGCR0_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIGCR0_RESETVAL                                           (0x00000000U)

/* SCIGCR1 */

#define CSL_APP_LIN_SCIGCR1_COMMMODE_MASK                                      (0x00000001U)
#define CSL_APP_LIN_SCIGCR1_COMMMODE_SHIFT                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_COMMMODE_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_COMMMODE_MAX                                       (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_TIMINGMODE_MASK                                    (0x00000002U)
#define CSL_APP_LIN_SCIGCR1_TIMINGMODE_SHIFT                                   (0x00000001U)
#define CSL_APP_LIN_SCIGCR1_TIMINGMODE_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_TIMINGMODE_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_PARITYENA_MASK                                     (0x00000004U)
#define CSL_APP_LIN_SCIGCR1_PARITYENA_SHIFT                                    (0x00000002U)
#define CSL_APP_LIN_SCIGCR1_PARITYENA_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_PARITYENA_MAX                                      (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_PARITY_MASK                                        (0x00000008U)
#define CSL_APP_LIN_SCIGCR1_PARITY_SHIFT                                       (0x00000003U)
#define CSL_APP_LIN_SCIGCR1_PARITY_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_PARITY_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_STOP_MASK                                          (0x00000010U)
#define CSL_APP_LIN_SCIGCR1_STOP_SHIFT                                         (0x00000004U)
#define CSL_APP_LIN_SCIGCR1_STOP_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_STOP_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_CLK_COMMANDER_MASK                                    (0x00000020U)
#define CSL_APP_LIN_SCIGCR1_CLK_COMMANDER_SHIFT                                   (0x00000005U)
#define CSL_APP_LIN_SCIGCR1_CLK_COMMANDER_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_CLK_COMMANDER_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_LINMODE_MASK                                       (0x00000040U)
#define CSL_APP_LIN_SCIGCR1_LINMODE_SHIFT                                      (0x00000006U)
#define CSL_APP_LIN_SCIGCR1_LINMODE_RESETVAL                                   (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_LINMODE_MAX                                        (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_SWNRST_MASK                                        (0x00000080U)
#define CSL_APP_LIN_SCIGCR1_SWNRST_SHIFT                                       (0x00000007U)
#define CSL_APP_LIN_SCIGCR1_SWNRST_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_SWNRST_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_SLEEP_MASK                                         (0x00000100U)
#define CSL_APP_LIN_SCIGCR1_SLEEP_SHIFT                                        (0x00000008U)
#define CSL_APP_LIN_SCIGCR1_SLEEP_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_SLEEP_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_ADAPT_MASK                                         (0x00000200U)
#define CSL_APP_LIN_SCIGCR1_ADAPT_SHIFT                                        (0x00000009U)
#define CSL_APP_LIN_SCIGCR1_ADAPT_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_ADAPT_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_MBUFMODE_MASK                                      (0x00000400U)
#define CSL_APP_LIN_SCIGCR1_MBUFMODE_SHIFT                                     (0x0000000AU)
#define CSL_APP_LIN_SCIGCR1_MBUFMODE_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_MBUFMODE_MAX                                       (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_CTYPE_MASK                                         (0x00000800U)
#define CSL_APP_LIN_SCIGCR1_CTYPE_SHIFT                                        (0x0000000BU)
#define CSL_APP_LIN_SCIGCR1_CTYPE_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_CTYPE_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_HGENCTRL_MASK                                      (0x00001000U)
#define CSL_APP_LIN_SCIGCR1_HGENCTRL_SHIFT                                     (0x0000000CU)
#define CSL_APP_LIN_SCIGCR1_HGENCTRL_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_HGENCTRL_MAX                                       (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_STOPEXTFRAME_MASK                                  (0x00002000U)
#define CSL_APP_LIN_SCIGCR1_STOPEXTFRAME_SHIFT                                 (0x0000000DU)
#define CSL_APP_LIN_SCIGCR1_STOPEXTFRAME_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_STOPEXTFRAME_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_RESERVED_MASK                                      (0x0000C000U)
#define CSL_APP_LIN_SCIGCR1_RESERVED_SHIFT                                     (0x0000000EU)
#define CSL_APP_LIN_SCIGCR1_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_RESERVED_MAX                                       (0x00000003U)

#define CSL_APP_LIN_SCIGCR1_LOOPBACK_MASK                                      (0x00010000U)
#define CSL_APP_LIN_SCIGCR1_LOOPBACK_SHIFT                                     (0x00000010U)
#define CSL_APP_LIN_SCIGCR1_LOOPBACK_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_LOOPBACK_MAX                                       (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_CONT_MASK                                          (0x00020000U)
#define CSL_APP_LIN_SCIGCR1_CONT_SHIFT                                         (0x00000011U)
#define CSL_APP_LIN_SCIGCR1_CONT_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_CONT_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_RESERVED1_MASK                                     (0x00FC0000U)
#define CSL_APP_LIN_SCIGCR1_RESERVED1_SHIFT                                    (0x00000012U)
#define CSL_APP_LIN_SCIGCR1_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_RESERVED1_MAX                                      (0x0000003FU)

#define CSL_APP_LIN_SCIGCR1_RXENA_MASK                                         (0x01000000U)
#define CSL_APP_LIN_SCIGCR1_RXENA_SHIFT                                        (0x00000018U)
#define CSL_APP_LIN_SCIGCR1_RXENA_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_RXENA_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_TXENA_MASK                                         (0x02000000U)
#define CSL_APP_LIN_SCIGCR1_TXENA_SHIFT                                        (0x00000019U)
#define CSL_APP_LIN_SCIGCR1_TXENA_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_TXENA_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIGCR1_RESERVED2_MASK                                     (0xFC000000U)
#define CSL_APP_LIN_SCIGCR1_RESERVED2_SHIFT                                    (0x0000001AU)
#define CSL_APP_LIN_SCIGCR1_RESERVED2_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIGCR1_RESERVED2_MAX                                      (0x0000003FU)

#define CSL_APP_LIN_SCIGCR1_RESETVAL                                           (0x00000000U)

/* SCIGCR2 */

#define CSL_APP_LIN_SCIGCR2_POWERDOWN_MASK                                     (0x00000001U)
#define CSL_APP_LIN_SCIGCR2_POWERDOWN_SHIFT                                    (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_POWERDOWN_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_POWERDOWN_MAX                                      (0x00000001U)

#define CSL_APP_LIN_SCIGCR2_RESERVED_MASK                                      (0x000000FEU)
#define CSL_APP_LIN_SCIGCR2_RESERVED_SHIFT                                     (0x00000001U)
#define CSL_APP_LIN_SCIGCR2_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_RESERVED_MAX                                       (0x0000007FU)

#define CSL_APP_LIN_SCIGCR2_GENWU_MASK                                         (0x00000100U)
#define CSL_APP_LIN_SCIGCR2_GENWU_SHIFT                                        (0x00000008U)
#define CSL_APP_LIN_SCIGCR2_GENWU_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_GENWU_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIGCR2_RESERVED1_MASK                                     (0x0000FE00U)
#define CSL_APP_LIN_SCIGCR2_RESERVED1_SHIFT                                    (0x00000009U)
#define CSL_APP_LIN_SCIGCR2_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_RESERVED1_MAX                                      (0x0000007FU)

#define CSL_APP_LIN_SCIGCR2_SC_MASK                                            (0x00010000U)
#define CSL_APP_LIN_SCIGCR2_SC_SHIFT                                           (0x00000010U)
#define CSL_APP_LIN_SCIGCR2_SC_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_SC_MAX                                             (0x00000001U)

#define CSL_APP_LIN_SCIGCR2_CC_MASK                                            (0x00020000U)
#define CSL_APP_LIN_SCIGCR2_CC_SHIFT                                           (0x00000011U)
#define CSL_APP_LIN_SCIGCR2_CC_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_CC_MAX                                             (0x00000001U)

#define CSL_APP_LIN_SCIGCR2_RESERVED2_MASK                                     (0xFFFC0000U)
#define CSL_APP_LIN_SCIGCR2_RESERVED2_SHIFT                                    (0x00000012U)
#define CSL_APP_LIN_SCIGCR2_RESERVED2_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIGCR2_RESERVED2_MAX                                      (0x00003FFFU)

#define CSL_APP_LIN_SCIGCR2_RESETVAL                                           (0x00000000U)

/* SCISETINT */

#define CSL_APP_LIN_SCISETINT_SETBRKDTINT_MASK                                 (0x00000001U)
#define CSL_APP_LIN_SCISETINT_SETBRKDTINT_SHIFT                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETBRKDTINT_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETBRKDTINT_MAX                                  (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETWAKEUPINT_MASK                                (0x00000002U)
#define CSL_APP_LIN_SCISETINT_SETWAKEUPINT_SHIFT                               (0x00000001U)
#define CSL_APP_LIN_SCISETINT_SETWAKEUPINT_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETWAKEUPINT_MAX                                 (0x00000001U)

#define CSL_APP_LIN_SCISETINT_RESERVED_MASK                                    (0x0000000CU)
#define CSL_APP_LIN_SCISETINT_RESERVED_SHIFT                                   (0x00000002U)
#define CSL_APP_LIN_SCISETINT_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_RESERVED_MAX                                     (0x00000003U)

#define CSL_APP_LIN_SCISETINT_SETTIMEOUTINT_MASK                               (0x00000010U)
#define CSL_APP_LIN_SCISETINT_SETTIMEOUTINT_SHIFT                              (0x00000004U)
#define CSL_APP_LIN_SCISETINT_SETTIMEOUTINT_RESETVAL                           (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETTIMEOUTINT_MAX                                (0x00000001U)

#define CSL_APP_LIN_SCISETINT_RESERVED1_MASK                                   (0x00000020U)
#define CSL_APP_LIN_SCISETINT_RESERVED1_SHIFT                                  (0x00000005U)
#define CSL_APP_LIN_SCISETINT_RESERVED1_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_SCISETINT_RESERVED1_MAX                                    (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETTOAWUSINT_MASK                                (0x00000040U)
#define CSL_APP_LIN_SCISETINT_SETTOAWUSINT_SHIFT                               (0x00000006U)
#define CSL_APP_LIN_SCISETINT_SETTOAWUSINT_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETTOAWUSINT_MAX                                 (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETTOA3WUSINT_MASK                               (0x00000080U)
#define CSL_APP_LIN_SCISETINT_SETTOA3WUSINT_SHIFT                              (0x00000007U)
#define CSL_APP_LIN_SCISETINT_SETTOA3WUSINT_RESETVAL                           (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETTOA3WUSINT_MAX                                (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETTXINT_MASK                                    (0x00000100U)
#define CSL_APP_LIN_SCISETINT_SETTXINT_SHIFT                                   (0x00000008U)
#define CSL_APP_LIN_SCISETINT_SETTXINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETTXINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETRXINT_MASK                                    (0x00000200U)
#define CSL_APP_LIN_SCISETINT_SETRXINT_SHIFT                                   (0x00000009U)
#define CSL_APP_LIN_SCISETINT_SETRXINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETRXINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_RESERVED2_MASK                                   (0x00001C00U)
#define CSL_APP_LIN_SCISETINT_RESERVED2_SHIFT                                  (0x0000000AU)
#define CSL_APP_LIN_SCISETINT_RESERVED2_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_SCISETINT_RESERVED2_MAX                                    (0x00000007U)

#define CSL_APP_LIN_SCISETINT_SETIDINT_MASK                                    (0x00002000U)
#define CSL_APP_LIN_SCISETINT_SETIDINT_SHIFT                                   (0x0000000DU)
#define CSL_APP_LIN_SCISETINT_SETIDINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETIDINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_RESERVED3_MASK                                   (0x0000C000U)
#define CSL_APP_LIN_SCISETINT_RESERVED3_SHIFT                                  (0x0000000EU)
#define CSL_APP_LIN_SCISETINT_RESERVED3_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_SCISETINT_RESERVED3_MAX                                    (0x00000003U)

#define CSL_APP_LIN_SCISETINT_SET_TX_DMA_MASK                                  (0x00010000U)
#define CSL_APP_LIN_SCISETINT_SET_TX_DMA_SHIFT                                 (0x00000010U)
#define CSL_APP_LIN_SCISETINT_SET_TX_DMA_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SET_TX_DMA_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_MASK                                  (0x00020000U)
#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_SHIFT                                 (0x00000011U)
#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_ALL_MASK                              (0x00040000U)
#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_ALL_SHIFT                             (0x00000012U)
#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_ALL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SET_RX_DMA_ALL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINT_RESERVED4_MASK                                   (0x00F80000U)
#define CSL_APP_LIN_SCISETINT_RESERVED4_SHIFT                                  (0x00000013U)
#define CSL_APP_LIN_SCISETINT_RESERVED4_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_SCISETINT_RESERVED4_MAX                                    (0x0000001FU)

#define CSL_APP_LIN_SCISETINT_SETPEINT_MASK                                    (0x01000000U)
#define CSL_APP_LIN_SCISETINT_SETPEINT_SHIFT                                   (0x00000018U)
#define CSL_APP_LIN_SCISETINT_SETPEINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETPEINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETOEINT_MASK                                    (0x02000000U)
#define CSL_APP_LIN_SCISETINT_SETOEINT_SHIFT                                   (0x00000019U)
#define CSL_APP_LIN_SCISETINT_SETOEINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETOEINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETFEINT_MASK                                    (0x04000000U)
#define CSL_APP_LIN_SCISETINT_SETFEINT_SHIFT                                   (0x0000001AU)
#define CSL_APP_LIN_SCISETINT_SETFEINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETFEINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETNREINT_MASK                                   (0x08000000U)
#define CSL_APP_LIN_SCISETINT_SETNREINT_SHIFT                                  (0x0000001BU)
#define CSL_APP_LIN_SCISETINT_SETNREINT_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETNREINT_MAX                                    (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETISFEINT_MASK                                  (0x10000000U)
#define CSL_APP_LIN_SCISETINT_SETISFEINT_SHIFT                                 (0x0000001CU)
#define CSL_APP_LIN_SCISETINT_SETISFEINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETISFEINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETCEINT_MASK                                    (0x20000000U)
#define CSL_APP_LIN_SCISETINT_SETCEINT_SHIFT                                   (0x0000001DU)
#define CSL_APP_LIN_SCISETINT_SETCEINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETCEINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETPBEINT_MASK                                   (0x40000000U)
#define CSL_APP_LIN_SCISETINT_SETPBEINT_SHIFT                                  (0x0000001EU)
#define CSL_APP_LIN_SCISETINT_SETPBEINT_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETPBEINT_MAX                                    (0x00000001U)

#define CSL_APP_LIN_SCISETINT_SETBEINT_MASK                                    (0x80000000U)
#define CSL_APP_LIN_SCISETINT_SETBEINT_SHIFT                                   (0x0000001FU)
#define CSL_APP_LIN_SCISETINT_SETBEINT_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCISETINT_SETBEINT_MAX                                     (0x00000001U)

#define CSL_APP_LIN_SCISETINT_RESETVAL                                         (0x00000000U)

/* SCICLEARINT */

#define CSL_APP_LIN_SCICLEARINT_CLRBRKDTINT_MASK                               (0x00000001U)
#define CSL_APP_LIN_SCICLEARINT_CLRBRKDTINT_SHIFT                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRBRKDTINT_RESETVAL                           (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRBRKDTINT_MAX                                (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRWAKEUPINT_MASK                              (0x00000002U)
#define CSL_APP_LIN_SCICLEARINT_CLRWAKEUPINT_SHIFT                             (0x00000001U)
#define CSL_APP_LIN_SCICLEARINT_CLRWAKEUPINT_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRWAKEUPINT_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_RESERVED_MASK                                  (0x0000000CU)
#define CSL_APP_LIN_SCICLEARINT_RESERVED_SHIFT                                 (0x00000002U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED_MAX                                   (0x00000003U)

#define CSL_APP_LIN_SCICLEARINT_CLRTIMEOUTINT_MASK                             (0x00000010U)
#define CSL_APP_LIN_SCICLEARINT_CLRTIMEOUTINT_SHIFT                            (0x00000004U)
#define CSL_APP_LIN_SCICLEARINT_CLRTIMEOUTINT_RESETVAL                         (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRTIMEOUTINT_MAX                              (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_RESERVED1_MASK                                 (0x00000020U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED1_SHIFT                                (0x00000005U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED1_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED1_MAX                                  (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRTOAWUSINT_MASK                              (0x00000040U)
#define CSL_APP_LIN_SCICLEARINT_CLRTOAWUSINT_SHIFT                             (0x00000006U)
#define CSL_APP_LIN_SCICLEARINT_CLRTOAWUSINT_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRTOAWUSINT_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRTOA3WUSINT_MASK                             (0x00000080U)
#define CSL_APP_LIN_SCICLEARINT_CLRTOA3WUSINT_SHIFT                            (0x00000007U)
#define CSL_APP_LIN_SCICLEARINT_CLRTOA3WUSINT_RESETVAL                         (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRTOA3WUSINT_MAX                              (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRTXINT_MASK                                  (0x00000100U)
#define CSL_APP_LIN_SCICLEARINT_CLRTXINT_SHIFT                                 (0x00000008U)
#define CSL_APP_LIN_SCICLEARINT_CLRTXINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRTXINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRRXINT_MASK                                  (0x00000200U)
#define CSL_APP_LIN_SCICLEARINT_CLRRXINT_SHIFT                                 (0x00000009U)
#define CSL_APP_LIN_SCICLEARINT_CLRRXINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRRXINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_RESERVED2_MASK                                 (0x00001C00U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED2_SHIFT                                (0x0000000AU)
#define CSL_APP_LIN_SCICLEARINT_RESERVED2_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED2_MAX                                  (0x00000007U)

#define CSL_APP_LIN_SCICLEARINT_CLRIDINT_MASK                                  (0x00002000U)
#define CSL_APP_LIN_SCICLEARINT_CLRIDINT_SHIFT                                 (0x0000000DU)
#define CSL_APP_LIN_SCICLEARINT_CLRIDINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRIDINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_RESERVED3_MASK                                 (0x0000C000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED3_SHIFT                                (0x0000000EU)
#define CSL_APP_LIN_SCICLEARINT_RESERVED3_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED3_MAX                                  (0x00000003U)

#define CSL_APP_LIN_SCICLEARINT_CLRTXDMA_MASK                                  (0x00010000U)
#define CSL_APP_LIN_SCICLEARINT_CLRTXDMA_SHIFT                                 (0x00000010U)
#define CSL_APP_LIN_SCICLEARINT_CLRTXDMA_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRTXDMA_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRRXDMA_MASK                                  (0x00020000U)
#define CSL_APP_LIN_SCICLEARINT_CLRRXDMA_SHIFT                                 (0x00000011U)
#define CSL_APP_LIN_SCICLEARINT_CLRRXDMA_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRRXDMA_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_RESERVED4_MASK                                 (0x00040000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED4_SHIFT                                (0x00000012U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED4_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED4_MAX                                  (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_RESERVED5_MASK                                 (0x00F80000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED5_SHIFT                                (0x00000013U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED5_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_RESERVED5_MAX                                  (0x0000001FU)

#define CSL_APP_LIN_SCICLEARINT_CLRPEINT_MASK                                  (0x01000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRPEINT_SHIFT                                 (0x00000018U)
#define CSL_APP_LIN_SCICLEARINT_CLRPEINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRPEINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLROEINT_MASK                                  (0x02000000U)
#define CSL_APP_LIN_SCICLEARINT_CLROEINT_SHIFT                                 (0x00000019U)
#define CSL_APP_LIN_SCICLEARINT_CLROEINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLROEINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRFEINT_MASK                                  (0x04000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRFEINT_SHIFT                                 (0x0000001AU)
#define CSL_APP_LIN_SCICLEARINT_CLRFEINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRFEINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRNREINT_MASK                                 (0x08000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRNREINT_SHIFT                                (0x0000001BU)
#define CSL_APP_LIN_SCICLEARINT_CLRNREINT_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRNREINT_MAX                                  (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRISFEINT_MASK                                (0x10000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRISFEINT_SHIFT                               (0x0000001CU)
#define CSL_APP_LIN_SCICLEARINT_CLRISFEINT_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRISFEINT_MAX                                 (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRCEINT_MASK                                  (0x20000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRCEINT_SHIFT                                 (0x0000001DU)
#define CSL_APP_LIN_SCICLEARINT_CLRCEINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRCEINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRPBEINT_MASK                                 (0x40000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRPBEINT_SHIFT                                (0x0000001EU)
#define CSL_APP_LIN_SCICLEARINT_CLRPBEINT_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRPBEINT_MAX                                  (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_CLRBEINT_MASK                                  (0x80000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRBEINT_SHIFT                                 (0x0000001FU)
#define CSL_APP_LIN_SCICLEARINT_CLRBEINT_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCICLEARINT_CLRBEINT_MAX                                   (0x00000001U)

#define CSL_APP_LIN_SCICLEARINT_RESETVAL                                       (0x00000000U)

/* SCISETINTLVL */

#define CSL_APP_LIN_SCISETINTLVL_SETBRKDTINTLVL_MASK                           (0x00000001U)
#define CSL_APP_LIN_SCISETINTLVL_SETBRKDTINTLVL_SHIFT                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETBRKDTINTLVL_RESETVAL                       (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETBRKDTINTLVL_MAX                            (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETWAKEUPINTLVL_MASK                          (0x00000002U)
#define CSL_APP_LIN_SCISETINTLVL_SETWAKEUPINTLVL_SHIFT                         (0x00000001U)
#define CSL_APP_LIN_SCISETINTLVL_SETWAKEUPINTLVL_RESETVAL                      (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETWAKEUPINTLVL_MAX                           (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_RESERVED_MASK                                 (0x0000000CU)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED_SHIFT                                (0x00000002U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED_MAX                                  (0x00000003U)

#define CSL_APP_LIN_SCISETINTLVL_SETTIMEOUTINTLVL_MASK                         (0x00000010U)
#define CSL_APP_LIN_SCISETINTLVL_SETTIMEOUTINTLVL_SHIFT                        (0x00000004U)
#define CSL_APP_LIN_SCISETINTLVL_SETTIMEOUTINTLVL_RESETVAL                     (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETTIMEOUTINTLVL_MAX                          (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_RESERVED1_MASK                                (0x00000020U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED1_SHIFT                               (0x00000005U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED1_MAX                                 (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETTOAWUSINTLVL_MASK                          (0x00000040U)
#define CSL_APP_LIN_SCISETINTLVL_SETTOAWUSINTLVL_SHIFT                         (0x00000006U)
#define CSL_APP_LIN_SCISETINTLVL_SETTOAWUSINTLVL_RESETVAL                      (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETTOAWUSINTLVL_MAX                           (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETTOA3WUSINTLVL_MASK                         (0x00000080U)
#define CSL_APP_LIN_SCISETINTLVL_SETTOA3WUSINTLVL_SHIFT                        (0x00000007U)
#define CSL_APP_LIN_SCISETINTLVL_SETTOA3WUSINTLVL_RESETVAL                     (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETTOA3WUSINTLVL_MAX                          (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETTXINTLVL_MASK                              (0x00000100U)
#define CSL_APP_LIN_SCISETINTLVL_SETTXINTLVL_SHIFT                             (0x00000008U)
#define CSL_APP_LIN_SCISETINTLVL_SETTXINTLVL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETTXINTLVL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETRXINTOVO_MASK                              (0x00000200U)
#define CSL_APP_LIN_SCISETINTLVL_SETRXINTOVO_SHIFT                             (0x00000009U)
#define CSL_APP_LIN_SCISETINTLVL_SETRXINTOVO_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETRXINTOVO_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_RESERVED2_MASK                                (0x00001C00U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED2_SHIFT                               (0x0000000AU)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED2_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED2_MAX                                 (0x00000007U)

#define CSL_APP_LIN_SCISETINTLVL_SETIDINTLVL_MASK                              (0x00002000U)
#define CSL_APP_LIN_SCISETINTLVL_SETIDINTLVL_SHIFT                             (0x0000000DU)
#define CSL_APP_LIN_SCISETINTLVL_SETIDINTLVL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETIDINTLVL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_RESERVED3_MASK                                (0x0000C000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED3_SHIFT                               (0x0000000EU)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED3_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED3_MAX                                 (0x00000003U)

#define CSL_APP_LIN_SCISETINTLVL_RESERVED4_MASK                                (0x00030000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED4_SHIFT                               (0x00000010U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED4_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED4_MAX                                 (0x00000003U)

#define CSL_APP_LIN_SCISETINTLVL_RESERVED5_MASK                                (0x00040000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED5_SHIFT                               (0x00000012U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED5_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED5_MAX                                 (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_RESERVED6_MASK                                (0x00F80000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED6_SHIFT                               (0x00000013U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED6_RESETVAL                            (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_RESERVED6_MAX                                 (0x0000001FU)

#define CSL_APP_LIN_SCISETINTLVL_SETPEINTLVL_MASK                              (0x01000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETPEINTLVL_SHIFT                             (0x00000018U)
#define CSL_APP_LIN_SCISETINTLVL_SETPEINTLVL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETPEINTLVL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETOEINTLVL_MASK                              (0x02000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETOEINTLVL_SHIFT                             (0x00000019U)
#define CSL_APP_LIN_SCISETINTLVL_SETOEINTLVL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETOEINTLVL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETFEINTLVL_MASK                              (0x04000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETFEINTLVL_SHIFT                             (0x0000001AU)
#define CSL_APP_LIN_SCISETINTLVL_SETFEINTLVL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETFEINTLVL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETNREINTLVL_MASK                             (0x08000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETNREINTLVL_SHIFT                            (0x0000001BU)
#define CSL_APP_LIN_SCISETINTLVL_SETNREINTLVL_RESETVAL                         (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETNREINTLVL_MAX                              (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETISFEINTLVL_MASK                            (0x10000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETISFEINTLVL_SHIFT                           (0x0000001CU)
#define CSL_APP_LIN_SCISETINTLVL_SETISFEINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETISFEINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETCEINTLVL_MASK                              (0x20000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETCEINTLVL_SHIFT                             (0x0000001DU)
#define CSL_APP_LIN_SCISETINTLVL_SETCEINTLVL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETCEINTLVL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETPBEINTLVL_MASK                             (0x40000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETPBEINTLVL_SHIFT                            (0x0000001EU)
#define CSL_APP_LIN_SCISETINTLVL_SETPBEINTLVL_RESETVAL                         (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETPBEINTLVL_MAX                              (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_SETBEINTLVL_MASK                              (0x80000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETBEINTLVL_SHIFT                             (0x0000001FU)
#define CSL_APP_LIN_SCISETINTLVL_SETBEINTLVL_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCISETINTLVL_SETBEINTLVL_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCISETINTLVL_RESETVAL                                      (0x00000000U)

/* SCICLEARINTLVL */

#define CSL_APP_LIN_SCICLEARINTLVL_CLRBRKDTINTLVL_MASK                         (0x00000001U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRBRKDTINTLVL_SHIFT                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRBRKDTINTLVL_RESETVAL                     (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRBRKDTINTLVL_MAX                          (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRWAKEUPINTLVL_MASK                        (0x00000002U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRWAKEUPINTLVL_SHIFT                       (0x00000001U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRWAKEUPINTLVL_RESETVAL                    (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRWAKEUPINTLVL_MAX                         (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED_MASK                               (0x0000000CU)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED_SHIFT                              (0x00000002U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED_RESETVAL                           (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED_MAX                                (0x00000003U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRTIMEOUTINTLVL_MASK                       (0x00000010U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTIMEOUTINTLVL_SHIFT                      (0x00000004U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTIMEOUTINTLVL_RESETVAL                   (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTIMEOUTINTLVL_MAX                        (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED1_MASK                              (0x00000020U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED1_SHIFT                             (0x00000005U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED1_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED1_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOAWUSINTLVL_MASK                        (0x00000040U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOAWUSINTLVL_SHIFT                       (0x00000006U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOAWUSINTLVL_RESETVAL                    (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOAWUSINTLVL_MAX                         (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOA3WUSINTLVL_MASK                       (0x00000080U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOA3WUSINTLVL_SHIFT                      (0x00000007U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOA3WUSINTLVL_RESETVAL                   (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTOA3WUSINTLVL_MAX                        (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRTXINTLVL_MASK                            (0x00000100U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTXINTLVL_SHIFT                           (0x00000008U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTXINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRTXINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRRXINTLVL_MASK                            (0x00000200U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRRXINTLVL_SHIFT                           (0x00000009U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRRXINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRRXINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED2_MASK                              (0x00001C00U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED2_SHIFT                             (0x0000000AU)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED2_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED2_MAX                               (0x00000007U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRIDINTLVL_MASK                            (0x00002000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRIDINTLVL_SHIFT                           (0x0000000DU)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRIDINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRIDINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED3_MASK                              (0x0000C000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED3_SHIFT                             (0x0000000EU)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED3_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED3_MAX                               (0x00000003U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED4_MASK                              (0x00030000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED4_SHIFT                             (0x00000010U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED4_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED4_MAX                               (0x00000003U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED5_MASK                              (0x00040000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED5_SHIFT                             (0x00000012U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED5_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED5_MAX                               (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED6_MASK                              (0x00F80000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED6_SHIFT                             (0x00000013U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED6_RESETVAL                          (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_RESERVED6_MAX                               (0x0000001FU)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRPEINTLVL_MASK                            (0x01000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRPEINTLVL_SHIFT                           (0x00000018U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRPEINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRPEINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLROEINTLVL_MASK                            (0x02000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLROEINTLVL_SHIFT                           (0x00000019U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLROEINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLROEINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRFEINTLVL_MASK                            (0x04000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRFEINTLVL_SHIFT                           (0x0000001AU)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRFEINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRFEINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRNREINTLVL_MASK                           (0x08000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRNREINTLVL_SHIFT                          (0x0000001BU)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRNREINTLVL_RESETVAL                       (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRNREINTLVL_MAX                            (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRISFEINTLVL_MASK                          (0x10000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRISFEINTLVL_SHIFT                         (0x0000001CU)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRISFEINTLVL_RESETVAL                      (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRISFEINTLVL_MAX                           (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRCEINTLVL_MASK                            (0x20000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRCEINTLVL_SHIFT                           (0x0000001DU)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRCEINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRCEINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRPBEINTLVL_MASK                           (0x40000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRPBEINTLVL_SHIFT                          (0x0000001EU)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRPBEINTLVL_RESETVAL                       (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRPBEINTLVL_MAX                            (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_CLRBEINTLVL_MASK                            (0x80000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRBEINTLVL_SHIFT                           (0x0000001FU)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRBEINTLVL_RESETVAL                        (0x00000000U)
#define CSL_APP_LIN_SCICLEARINTLVL_CLRBEINTLVL_MAX                             (0x00000001U)

#define CSL_APP_LIN_SCICLEARINTLVL_RESETVAL                                    (0x00000000U)

/* SCIFLR */

#define CSL_APP_LIN_SCIFLR_BRKDT_MASK                                          (0x00000001U)
#define CSL_APP_LIN_SCIFLR_BRKDT_SHIFT                                         (0x00000000U)
#define CSL_APP_LIN_SCIFLR_BRKDT_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIFLR_BRKDT_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIFLR_WAKEUP_MASK                                         (0x00000002U)
#define CSL_APP_LIN_SCIFLR_WAKEUP_SHIFT                                        (0x00000001U)
#define CSL_APP_LIN_SCIFLR_WAKEUP_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIFLR_WAKEUP_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIFLR_IDLE_MASK                                           (0x00000004U)
#define CSL_APP_LIN_SCIFLR_IDLE_SHIFT                                          (0x00000002U)
#define CSL_APP_LIN_SCIFLR_IDLE_RESETVAL                                       (0x00000001U)
#define CSL_APP_LIN_SCIFLR_IDLE_MAX                                            (0x00000001U)

#define CSL_APP_LIN_SCIFLR_BUSY_MASK                                           (0x00000008U)
#define CSL_APP_LIN_SCIFLR_BUSY_SHIFT                                          (0x00000003U)
#define CSL_APP_LIN_SCIFLR_BUSY_RESETVAL                                       (0x00000000U)
#define CSL_APP_LIN_SCIFLR_BUSY_MAX                                            (0x00000001U)

#define CSL_APP_LIN_SCIFLR_TIMEOUT_MASK                                        (0x00000010U)
#define CSL_APP_LIN_SCIFLR_TIMEOUT_SHIFT                                       (0x00000004U)
#define CSL_APP_LIN_SCIFLR_TIMEOUT_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIFLR_TIMEOUT_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIFLR_RESERVED_MASK                                       (0x00000020U)
#define CSL_APP_LIN_SCIFLR_RESERVED_SHIFT                                      (0x00000005U)
#define CSL_APP_LIN_SCIFLR_RESERVED_RESETVAL                                   (0x00000000U)
#define CSL_APP_LIN_SCIFLR_RESERVED_MAX                                        (0x00000001U)

#define CSL_APP_LIN_SCIFLR_TOAWUS_MASK                                         (0x00000040U)
#define CSL_APP_LIN_SCIFLR_TOAWUS_SHIFT                                        (0x00000006U)
#define CSL_APP_LIN_SCIFLR_TOAWUS_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIFLR_TOAWUS_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIFLR_TOA3WUS_MASK                                        (0x00000080U)
#define CSL_APP_LIN_SCIFLR_TOA3WUS_SHIFT                                       (0x00000007U)
#define CSL_APP_LIN_SCIFLR_TOA3WUS_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIFLR_TOA3WUS_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIFLR_TXRDY_MASK                                          (0x00000100U)
#define CSL_APP_LIN_SCIFLR_TXRDY_SHIFT                                         (0x00000008U)
#define CSL_APP_LIN_SCIFLR_TXRDY_RESETVAL                                      (0x00000001U)
#define CSL_APP_LIN_SCIFLR_TXRDY_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIFLR_RXRDY_MASK                                          (0x00000200U)
#define CSL_APP_LIN_SCIFLR_RXRDY_SHIFT                                         (0x00000009U)
#define CSL_APP_LIN_SCIFLR_RXRDY_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIFLR_RXRDY_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIFLR_TXWAKE_MASK                                         (0x00000400U)
#define CSL_APP_LIN_SCIFLR_TXWAKE_SHIFT                                        (0x0000000AU)
#define CSL_APP_LIN_SCIFLR_TXWAKE_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIFLR_TXWAKE_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIFLR_TXEMPTY_MASK                                        (0x00000800U)
#define CSL_APP_LIN_SCIFLR_TXEMPTY_SHIFT                                       (0x0000000BU)
#define CSL_APP_LIN_SCIFLR_TXEMPTY_RESETVAL                                    (0x00000001U)
#define CSL_APP_LIN_SCIFLR_TXEMPTY_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIFLR_RXWAKE_MASK                                         (0x00001000U)
#define CSL_APP_LIN_SCIFLR_RXWAKE_SHIFT                                        (0x0000000CU)
#define CSL_APP_LIN_SCIFLR_RXWAKE_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIFLR_RXWAKE_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIFLR_IDTXFLAG_MASK                                       (0x00002000U)
#define CSL_APP_LIN_SCIFLR_IDTXFLAG_SHIFT                                      (0x0000000DU)
#define CSL_APP_LIN_SCIFLR_IDTXFLAG_RESETVAL                                   (0x00000000U)
#define CSL_APP_LIN_SCIFLR_IDTXFLAG_MAX                                        (0x00000001U)

#define CSL_APP_LIN_SCIFLR_IDRXFLAG_MASK                                       (0x00004000U)
#define CSL_APP_LIN_SCIFLR_IDRXFLAG_SHIFT                                      (0x0000000EU)
#define CSL_APP_LIN_SCIFLR_IDRXFLAG_RESETVAL                                   (0x00000000U)
#define CSL_APP_LIN_SCIFLR_IDRXFLAG_MAX                                        (0x00000001U)

#define CSL_APP_LIN_SCIFLR_RESERVED1_MASK                                      (0x00008000U)
#define CSL_APP_LIN_SCIFLR_RESERVED1_SHIFT                                     (0x0000000FU)
#define CSL_APP_LIN_SCIFLR_RESERVED1_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIFLR_RESERVED1_MAX                                       (0x00000001U)

#define CSL_APP_LIN_SCIFLR_RESERVED2_MASK                                      (0x00FF0000U)
#define CSL_APP_LIN_SCIFLR_RESERVED2_SHIFT                                     (0x00000010U)
#define CSL_APP_LIN_SCIFLR_RESERVED2_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIFLR_RESERVED2_MAX                                       (0x000000FFU)

#define CSL_APP_LIN_SCIFLR_PE_MASK                                             (0x01000000U)
#define CSL_APP_LIN_SCIFLR_PE_SHIFT                                            (0x00000018U)
#define CSL_APP_LIN_SCIFLR_PE_RESETVAL                                         (0x00000000U)
#define CSL_APP_LIN_SCIFLR_PE_MAX                                              (0x00000001U)

#define CSL_APP_LIN_SCIFLR_OE_MASK                                             (0x02000000U)
#define CSL_APP_LIN_SCIFLR_OE_SHIFT                                            (0x00000019U)
#define CSL_APP_LIN_SCIFLR_OE_RESETVAL                                         (0x00000000U)
#define CSL_APP_LIN_SCIFLR_OE_MAX                                              (0x00000001U)

#define CSL_APP_LIN_SCIFLR_FE_MASK                                             (0x04000000U)
#define CSL_APP_LIN_SCIFLR_FE_SHIFT                                            (0x0000001AU)
#define CSL_APP_LIN_SCIFLR_FE_RESETVAL                                         (0x00000000U)
#define CSL_APP_LIN_SCIFLR_FE_MAX                                              (0x00000001U)

#define CSL_APP_LIN_SCIFLR_NRE_MASK                                            (0x08000000U)
#define CSL_APP_LIN_SCIFLR_NRE_SHIFT                                           (0x0000001BU)
#define CSL_APP_LIN_SCIFLR_NRE_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_SCIFLR_NRE_MAX                                             (0x00000001U)

#define CSL_APP_LIN_SCIFLR_ISFE_MASK                                           (0x10000000U)
#define CSL_APP_LIN_SCIFLR_ISFE_SHIFT                                          (0x0000001CU)
#define CSL_APP_LIN_SCIFLR_ISFE_RESETVAL                                       (0x00000000U)
#define CSL_APP_LIN_SCIFLR_ISFE_MAX                                            (0x00000001U)

#define CSL_APP_LIN_SCIFLR_CE_MASK                                             (0x20000000U)
#define CSL_APP_LIN_SCIFLR_CE_SHIFT                                            (0x0000001DU)
#define CSL_APP_LIN_SCIFLR_CE_RESETVAL                                         (0x00000000U)
#define CSL_APP_LIN_SCIFLR_CE_MAX                                              (0x00000001U)

#define CSL_APP_LIN_SCIFLR_PBE_MASK                                            (0x40000000U)
#define CSL_APP_LIN_SCIFLR_PBE_SHIFT                                           (0x0000001EU)
#define CSL_APP_LIN_SCIFLR_PBE_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_SCIFLR_PBE_MAX                                             (0x00000001U)

#define CSL_APP_LIN_SCIFLR_BE_MASK                                             (0x80000000U)
#define CSL_APP_LIN_SCIFLR_BE_SHIFT                                            (0x0000001FU)
#define CSL_APP_LIN_SCIFLR_BE_RESETVAL                                         (0x00000000U)
#define CSL_APP_LIN_SCIFLR_BE_MAX                                              (0x00000001U)

#define CSL_APP_LIN_SCIFLR_RESETVAL                                            (0x00000904U)

/* SCIINTVECT0 */

#define CSL_APP_LIN_SCIINTVECT0_INTVECT0_MASK                                  (0x0000001FU)
#define CSL_APP_LIN_SCIINTVECT0_INTVECT0_SHIFT                                 (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT0_INTVECT0_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT0_INTVECT0_MAX                                   (0x0000001FU)

#define CSL_APP_LIN_SCIINTVECT0_RESERVED_MASK                                  (0x0000FFE0U)
#define CSL_APP_LIN_SCIINTVECT0_RESERVED_SHIFT                                 (0x00000005U)
#define CSL_APP_LIN_SCIINTVECT0_RESERVED_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT0_RESERVED_MAX                                   (0x000007FFU)

#define CSL_APP_LIN_SCIINTVECT0_RESERVED1_MASK                                 (0xFFFF0000U)
#define CSL_APP_LIN_SCIINTVECT0_RESERVED1_SHIFT                                (0x00000010U)
#define CSL_APP_LIN_SCIINTVECT0_RESERVED1_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT0_RESERVED1_MAX                                  (0x0000FFFFU)

#define CSL_APP_LIN_SCIINTVECT0_RESETVAL                                       (0x00000000U)

/* SCIINTVECT1 */

#define CSL_APP_LIN_SCIINTVECT1_INTVECT1_MASK                                  (0x0000001FU)
#define CSL_APP_LIN_SCIINTVECT1_INTVECT1_SHIFT                                 (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT1_INTVECT1_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT1_INTVECT1_MAX                                   (0x0000001FU)

#define CSL_APP_LIN_SCIINTVECT1_RESERVED_MASK                                  (0x0000FFE0U)
#define CSL_APP_LIN_SCIINTVECT1_RESERVED_SHIFT                                 (0x00000005U)
#define CSL_APP_LIN_SCIINTVECT1_RESERVED_RESETVAL                              (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT1_RESERVED_MAX                                   (0x000007FFU)

#define CSL_APP_LIN_SCIINTVECT1_RESERVED1_MASK                                 (0xFFFF0000U)
#define CSL_APP_LIN_SCIINTVECT1_RESERVED1_SHIFT                                (0x00000010U)
#define CSL_APP_LIN_SCIINTVECT1_RESERVED1_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_SCIINTVECT1_RESERVED1_MAX                                  (0x0000FFFFU)

#define CSL_APP_LIN_SCIINTVECT1_RESETVAL                                       (0x00000000U)

/* SCIFORMAT */

#define CSL_APP_LIN_SCIFORMAT_CHAR_MASK                                        (0x00000007U)
#define CSL_APP_LIN_SCIFORMAT_CHAR_SHIFT                                       (0x00000000U)
#define CSL_APP_LIN_SCIFORMAT_CHAR_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIFORMAT_CHAR_MAX                                         (0x00000007U)

#define CSL_APP_LIN_SCIFORMAT_RESERVED_MASK                                    (0x0000FFF8U)
#define CSL_APP_LIN_SCIFORMAT_RESERVED_SHIFT                                   (0x00000003U)
#define CSL_APP_LIN_SCIFORMAT_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_SCIFORMAT_RESERVED_MAX                                     (0x00001FFFU)

#define CSL_APP_LIN_SCIFORMAT_LENGTH_MASK                                      (0x00070000U)
#define CSL_APP_LIN_SCIFORMAT_LENGTH_SHIFT                                     (0x00000010U)
#define CSL_APP_LIN_SCIFORMAT_LENGTH_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIFORMAT_LENGTH_MAX                                       (0x00000007U)

#define CSL_APP_LIN_SCIFORMAT_RESERVED1_MASK                                   (0xFFF80000U)
#define CSL_APP_LIN_SCIFORMAT_RESERVED1_SHIFT                                  (0x00000013U)
#define CSL_APP_LIN_SCIFORMAT_RESERVED1_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_SCIFORMAT_RESERVED1_MAX                                    (0x00001FFFU)

#define CSL_APP_LIN_SCIFORMAT_RESETVAL                                         (0x00000000U)

/* BRSR */

#define CSL_APP_LIN_BRSR_SCI_LIN_PSL_MASK                                      (0x0000FFFFU)
#define CSL_APP_LIN_BRSR_SCI_LIN_PSL_SHIFT                                     (0x00000000U)
#define CSL_APP_LIN_BRSR_SCI_LIN_PSL_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_BRSR_SCI_LIN_PSL_MAX                                       (0x0000FFFFU)

#define CSL_APP_LIN_BRSR_SCI_LIN_PSH_MASK                                      (0x00FF0000U)
#define CSL_APP_LIN_BRSR_SCI_LIN_PSH_SHIFT                                     (0x00000010U)
#define CSL_APP_LIN_BRSR_SCI_LIN_PSH_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_BRSR_SCI_LIN_PSH_MAX                                       (0x000000FFU)

#define CSL_APP_LIN_BRSR_M_MASK                                                (0x0F000000U)
#define CSL_APP_LIN_BRSR_M_SHIFT                                               (0x00000018U)
#define CSL_APP_LIN_BRSR_M_RESETVAL                                            (0x00000000U)
#define CSL_APP_LIN_BRSR_M_MAX                                                 (0x0000000FU)

#define CSL_APP_LIN_BRSR_U_MASK                                                (0x70000000U)
#define CSL_APP_LIN_BRSR_U_SHIFT                                               (0x0000001CU)
#define CSL_APP_LIN_BRSR_U_RESETVAL                                            (0x00000000U)
#define CSL_APP_LIN_BRSR_U_MAX                                                 (0x00000007U)

#define CSL_APP_LIN_BRSR_RESERVED_MASK                                         (0x80000000U)
#define CSL_APP_LIN_BRSR_RESERVED_SHIFT                                        (0x0000001FU)
#define CSL_APP_LIN_BRSR_RESERVED_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_BRSR_RESERVED_MAX                                          (0x00000001U)

#define CSL_APP_LIN_BRSR_RESETVAL                                              (0x00000000U)

/* SCIED */

#define CSL_APP_LIN_SCIED_ED_MASK                                              (0x000000FFU)
#define CSL_APP_LIN_SCIED_ED_SHIFT                                             (0x00000000U)
#define CSL_APP_LIN_SCIED_ED_RESETVAL                                          (0x00000000U)
#define CSL_APP_LIN_SCIED_ED_MAX                                               (0x000000FFU)

#define CSL_APP_LIN_SCIED_RESERVED_MASK                                        (0xFFFFFF00U)
#define CSL_APP_LIN_SCIED_RESERVED_SHIFT                                       (0x00000008U)
#define CSL_APP_LIN_SCIED_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIED_RESERVED_MAX                                         (0x00FFFFFFU)

#define CSL_APP_LIN_SCIED_RESETVAL                                             (0x00000000U)

/* SCIRD */

#define CSL_APP_LIN_SCIRD_RD_MASK                                              (0x000000FFU)
#define CSL_APP_LIN_SCIRD_RD_SHIFT                                             (0x00000000U)
#define CSL_APP_LIN_SCIRD_RD_RESETVAL                                          (0x00000000U)
#define CSL_APP_LIN_SCIRD_RD_MAX                                               (0x000000FFU)

#define CSL_APP_LIN_SCIRD_RESERVED_MASK                                        (0xFFFFFF00U)
#define CSL_APP_LIN_SCIRD_RESERVED_SHIFT                                       (0x00000008U)
#define CSL_APP_LIN_SCIRD_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIRD_RESERVED_MAX                                         (0x00FFFFFFU)

#define CSL_APP_LIN_SCIRD_RESETVAL                                             (0x00000000U)

/* SCITD */

#define CSL_APP_LIN_SCITD_TD_MASK                                              (0x000000FFU)
#define CSL_APP_LIN_SCITD_TD_SHIFT                                             (0x00000000U)
#define CSL_APP_LIN_SCITD_TD_RESETVAL                                          (0x00000000U)
#define CSL_APP_LIN_SCITD_TD_MAX                                               (0x000000FFU)

#define CSL_APP_LIN_SCITD_RESERVED_MASK                                        (0xFFFFFF00U)
#define CSL_APP_LIN_SCITD_RESERVED_SHIFT                                       (0x00000008U)
#define CSL_APP_LIN_SCITD_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCITD_RESERVED_MAX                                         (0x00FFFFFFU)

#define CSL_APP_LIN_SCITD_RESETVAL                                             (0x00000000U)

/* SCIPIO0 */

#define CSL_APP_LIN_SCIPIO0_CLKFUNC_MASK                                       (0x00000001U)
#define CSL_APP_LIN_SCIPIO0_CLKFUNC_SHIFT                                      (0x00000000U)
#define CSL_APP_LIN_SCIPIO0_CLKFUNC_RESETVAL                                   (0x00000000U)
#define CSL_APP_LIN_SCIPIO0_CLKFUNC_MAX                                        (0x00000001U)

#define CSL_APP_LIN_SCIPIO0_RXFUNC_MASK                                        (0x00000002U)
#define CSL_APP_LIN_SCIPIO0_RXFUNC_SHIFT                                       (0x00000001U)
#define CSL_APP_LIN_SCIPIO0_RXFUNC_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIPIO0_RXFUNC_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIPIO0_TXFUNC_MASK                                        (0x00000004U)
#define CSL_APP_LIN_SCIPIO0_TXFUNC_SHIFT                                       (0x00000002U)
#define CSL_APP_LIN_SCIPIO0_TXFUNC_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIPIO0_TXFUNC_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIPIO0_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO0_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO0_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO0_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO0_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO0_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO0_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO0_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO0_RESETVAL                                           (0x00000000U)

/* SCIPIO1 */

#define CSL_APP_LIN_SCIPIO1_CLKDIR_MASK                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO1_CLKDIR_SHIFT                                       (0x00000000U)
#define CSL_APP_LIN_SCIPIO1_CLKDIR_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIPIO1_CLKDIR_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIPIO1_RXDIR_MASK                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO1_RXDIR_SHIFT                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO1_RXDIR_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO1_RXDIR_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO1_TXDIR_MASK                                         (0x00000004U)
#define CSL_APP_LIN_SCIPIO1_TXDIR_SHIFT                                        (0x00000002U)
#define CSL_APP_LIN_SCIPIO1_TXDIR_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO1_TXDIR_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO1_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO1_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO1_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO1_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO1_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO1_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO1_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO1_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO1_RESETVAL                                           (0x00000000U)

/* SCIPIO2 */

#define CSL_APP_LIN_SCIPIO2_CLKIN_MASK                                         (0x00000001U)
#define CSL_APP_LIN_SCIPIO2_CLKIN_SHIFT                                        (0x00000000U)
#define CSL_APP_LIN_SCIPIO2_CLKIN_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO2_CLKIN_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO2_RXIN_MASK                                          (0x00000002U)
#define CSL_APP_LIN_SCIPIO2_RXIN_SHIFT                                         (0x00000001U)
#define CSL_APP_LIN_SCIPIO2_RXIN_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIPIO2_RXIN_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIPIO2_TXIN_MASK                                          (0x00000004U)
#define CSL_APP_LIN_SCIPIO2_TXIN_SHIFT                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO2_TXIN_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIPIO2_TXIN_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIPIO2_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO2_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO2_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO2_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO2_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO2_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO2_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO2_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO2_RESETVAL                                           (0x00000000U)

/* SCIPIO3 */

#define CSL_APP_LIN_SCIPIO3_CLKOUT_MASK                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO3_CLKOUT_SHIFT                                       (0x00000000U)
#define CSL_APP_LIN_SCIPIO3_CLKOUT_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIPIO3_CLKOUT_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIPIO3_RXOUT_MASK                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO3_RXOUT_SHIFT                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO3_RXOUT_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO3_RXOUT_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO3_TXOUT_MASK                                         (0x00000004U)
#define CSL_APP_LIN_SCIPIO3_TXOUT_SHIFT                                        (0x00000002U)
#define CSL_APP_LIN_SCIPIO3_TXOUT_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO3_TXOUT_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO3_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO3_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO3_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO3_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO3_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO3_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO3_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO3_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO3_RESETVAL                                           (0x00000000U)

/* SCIPIO4 */

#define CSL_APP_LIN_SCIPIO4_CLKSET_MASK                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO4_CLKSET_SHIFT                                       (0x00000000U)
#define CSL_APP_LIN_SCIPIO4_CLKSET_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIPIO4_CLKSET_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIPIO4_RXSET_MASK                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO4_RXSET_SHIFT                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO4_RXSET_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO4_RXSET_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO4_TXSET_MASK                                         (0x00000004U)
#define CSL_APP_LIN_SCIPIO4_TXSET_SHIFT                                        (0x00000002U)
#define CSL_APP_LIN_SCIPIO4_TXSET_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO4_TXSET_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO4_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO4_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO4_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO4_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO4_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO4_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO4_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO4_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO4_RESETVAL                                           (0x00000000U)

/* SCIPIO5 */

#define CSL_APP_LIN_SCIPIO5_CLKCLR_MASK                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO5_CLKCLR_SHIFT                                       (0x00000000U)
#define CSL_APP_LIN_SCIPIO5_CLKCLR_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_SCIPIO5_CLKCLR_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIPIO5_RXCLR_MASK                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO5_RXCLR_SHIFT                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO5_RXCLR_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO5_RXCLR_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO5_TXCLR_MASK                                         (0x00000004U)
#define CSL_APP_LIN_SCIPIO5_TXCLR_SHIFT                                        (0x00000002U)
#define CSL_APP_LIN_SCIPIO5_TXCLR_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO5_TXCLR_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO5_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO5_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO5_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO5_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO5_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO5_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO5_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO5_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO5_RESETVAL                                           (0x00000000U)

/* SCIPIO6 */

#define CSL_APP_LIN_SCIPIO6_CLKDR_MASK                                         (0x00000001U)
#define CSL_APP_LIN_SCIPIO6_CLKDR_SHIFT                                        (0x00000000U)
#define CSL_APP_LIN_SCIPIO6_CLKDR_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO6_CLKDR_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO6_RXPDR_MASK                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO6_RXPDR_SHIFT                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO6_RXPDR_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO6_RXPDR_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO6_TXPDR_MASK                                         (0x00000004U)
#define CSL_APP_LIN_SCIPIO6_TXPDR_SHIFT                                        (0x00000002U)
#define CSL_APP_LIN_SCIPIO6_TXPDR_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO6_TXPDR_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO6_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO6_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO6_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO6_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO6_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO6_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO6_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO6_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO6_RESETVAL                                           (0x00000000U)

/* SCIPIO7 */

#define CSL_APP_LIN_SCIPIO7_CLKPD_MASK                                         (0x00000001U)
#define CSL_APP_LIN_SCIPIO7_CLKPD_SHIFT                                        (0x00000000U)
#define CSL_APP_LIN_SCIPIO7_CLKPD_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO7_CLKPD_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO7_RXPD_MASK                                          (0x00000002U)
#define CSL_APP_LIN_SCIPIO7_RXPD_SHIFT                                         (0x00000001U)
#define CSL_APP_LIN_SCIPIO7_RXPD_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIPIO7_RXPD_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIPIO7_TXPD_MASK                                          (0x00000004U)
#define CSL_APP_LIN_SCIPIO7_TXPD_SHIFT                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO7_TXPD_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIPIO7_TXPD_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIPIO7_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO7_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO7_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO7_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO7_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO7_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO7_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO7_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO7_RESETVAL                                           (0x00000000U)

/* SCIPIO8 */

#define CSL_APP_LIN_SCIPIO8_CLKPSL_MASK                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO8_CLKPSL_SHIFT                                       (0x00000000U)
#define CSL_APP_LIN_SCIPIO8_CLKPSL_RESETVAL                                    (0x00000001U)
#define CSL_APP_LIN_SCIPIO8_CLKPSL_MAX                                         (0x00000001U)

#define CSL_APP_LIN_SCIPIO8_RXPSL_MASK                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO8_RXPSL_SHIFT                                        (0x00000001U)
#define CSL_APP_LIN_SCIPIO8_RXPSL_RESETVAL                                     (0x00000001U)
#define CSL_APP_LIN_SCIPIO8_RXPSL_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO8_TXPSL_MASK                                         (0x00000004U)
#define CSL_APP_LIN_SCIPIO8_TXPSL_SHIFT                                        (0x00000002U)
#define CSL_APP_LIN_SCIPIO8_TXPSL_RESETVAL                                     (0x00000001U)
#define CSL_APP_LIN_SCIPIO8_TXPSL_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO8_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO8_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO8_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO8_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO8_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO8_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO8_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO8_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO8_RESETVAL                                           (0x00000007U)

/* LINCOMP */

#define CSL_APP_LIN_LINCOMP_SBREAK_MASK                                        (0x00000007U)
#define CSL_APP_LIN_LINCOMP_SBREAK_SHIFT                                       (0x00000000U)
#define CSL_APP_LIN_LINCOMP_SBREAK_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_LINCOMP_SBREAK_MAX                                         (0x00000007U)

#define CSL_APP_LIN_LINCOMP_RESERVED_MASK                                      (0x000000F8U)
#define CSL_APP_LIN_LINCOMP_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_LINCOMP_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_LINCOMP_RESERVED_MAX                                       (0x0000001FU)

#define CSL_APP_LIN_LINCOMP_SDEL_MASK                                          (0x00000300U)
#define CSL_APP_LIN_LINCOMP_SDEL_SHIFT                                         (0x00000008U)
#define CSL_APP_LIN_LINCOMP_SDEL_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_LINCOMP_SDEL_MAX                                           (0x00000003U)

#define CSL_APP_LIN_LINCOMP_RESERVED1_MASK                                     (0x0000FC00U)
#define CSL_APP_LIN_LINCOMP_RESERVED1_SHIFT                                    (0x0000000AU)
#define CSL_APP_LIN_LINCOMP_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_LINCOMP_RESERVED1_MAX                                      (0x0000003FU)

#define CSL_APP_LIN_LINCOMP_RESERVED2_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_LINCOMP_RESERVED2_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_LINCOMP_RESERVED2_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_LINCOMP_RESERVED2_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_LINCOMP_RESETVAL                                           (0x00000000U)

/* LINRD0 */

#define CSL_APP_LIN_LINRD0_RD3_MASK                                            (0x000000FFU)
#define CSL_APP_LIN_LINRD0_RD3_SHIFT                                           (0x00000000U)
#define CSL_APP_LIN_LINRD0_RD3_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD0_RD3_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD0_RD2_MASK                                            (0x0000FF00U)
#define CSL_APP_LIN_LINRD0_RD2_SHIFT                                           (0x00000008U)
#define CSL_APP_LIN_LINRD0_RD2_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD0_RD2_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD0_RD1_MASK                                            (0x00FF0000U)
#define CSL_APP_LIN_LINRD0_RD1_SHIFT                                           (0x00000010U)
#define CSL_APP_LIN_LINRD0_RD1_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD0_RD1_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD0_RD0_MASK                                            (0xFF000000U)
#define CSL_APP_LIN_LINRD0_RD0_SHIFT                                           (0x00000018U)
#define CSL_APP_LIN_LINRD0_RD0_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD0_RD0_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD0_RESETVAL                                            (0x00000000U)

/* LINRD1 */

#define CSL_APP_LIN_LINRD1_RD7_MASK                                            (0x000000FFU)
#define CSL_APP_LIN_LINRD1_RD7_SHIFT                                           (0x00000000U)
#define CSL_APP_LIN_LINRD1_RD7_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD1_RD7_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD1_RD6_MASK                                            (0x0000FF00U)
#define CSL_APP_LIN_LINRD1_RD6_SHIFT                                           (0x00000008U)
#define CSL_APP_LIN_LINRD1_RD6_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD1_RD6_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD1_RD5_MASK                                            (0x00FF0000U)
#define CSL_APP_LIN_LINRD1_RD5_SHIFT                                           (0x00000010U)
#define CSL_APP_LIN_LINRD1_RD5_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD1_RD5_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD1_RD4_MASK                                            (0xFF000000U)
#define CSL_APP_LIN_LINRD1_RD4_SHIFT                                           (0x00000018U)
#define CSL_APP_LIN_LINRD1_RD4_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINRD1_RD4_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINRD1_RESETVAL                                            (0x00000000U)

/* LINMASK */

#define CSL_APP_LIN_LINMASK_TXIDMASK_MASK                                      (0x000000FFU)
#define CSL_APP_LIN_LINMASK_TXIDMASK_SHIFT                                     (0x00000000U)
#define CSL_APP_LIN_LINMASK_TXIDMASK_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_LINMASK_TXIDMASK_MAX                                       (0x000000FFU)

#define CSL_APP_LIN_LINMASK_RESERVED_MASK                                      (0x0000FF00U)
#define CSL_APP_LIN_LINMASK_RESERVED_SHIFT                                     (0x00000008U)
#define CSL_APP_LIN_LINMASK_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_LINMASK_RESERVED_MAX                                       (0x000000FFU)

#define CSL_APP_LIN_LINMASK_RXIDMASK_MASK                                      (0x00FF0000U)
#define CSL_APP_LIN_LINMASK_RXIDMASK_SHIFT                                     (0x00000010U)
#define CSL_APP_LIN_LINMASK_RXIDMASK_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_LINMASK_RXIDMASK_MAX                                       (0x000000FFU)

#define CSL_APP_LIN_LINMASK_RESERVED1_MASK                                     (0xFF000000U)
#define CSL_APP_LIN_LINMASK_RESERVED1_SHIFT                                    (0x00000018U)
#define CSL_APP_LIN_LINMASK_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_LINMASK_RESERVED1_MAX                                      (0x000000FFU)

#define CSL_APP_LIN_LINMASK_RESETVAL                                           (0x00000000U)

/* LINID */

#define CSL_APP_LIN_LINID_IDBYTE_MASK                                          (0x000000FFU)
#define CSL_APP_LIN_LINID_IDBYTE_SHIFT                                         (0x00000000U)
#define CSL_APP_LIN_LINID_IDBYTE_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_LINID_IDBYTE_MAX                                           (0x000000FFU)

#define CSL_APP_LIN_LINID_IDRESPONDERTASKBYTE_MASK                                 (0x0000FF00U)
#define CSL_APP_LIN_LINID_IDRESPONDERTASKBYTE_SHIFT                                (0x00000008U)
#define CSL_APP_LIN_LINID_IDRESPONDERTASKBYTE_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_LINID_IDRESPONDERTASKBYTE_MAX                                  (0x000000FFU)

#define CSL_APP_LIN_LINID_RECEIVEDID_MASK                                      (0x00FF0000U)
#define CSL_APP_LIN_LINID_RECEIVEDID_SHIFT                                     (0x00000010U)
#define CSL_APP_LIN_LINID_RECEIVEDID_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_LINID_RECEIVEDID_MAX                                       (0x000000FFU)

#define CSL_APP_LIN_LINID_RESERVED_MASK                                        (0xFF000000U)
#define CSL_APP_LIN_LINID_RESERVED_SHIFT                                       (0x00000018U)
#define CSL_APP_LIN_LINID_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_LINID_RESERVED_MAX                                         (0x000000FFU)

#define CSL_APP_LIN_LINID_RESETVAL                                             (0x00000000U)

/* LINTD0 */

#define CSL_APP_LIN_LINTD0_TD3_MASK                                            (0x000000FFU)
#define CSL_APP_LIN_LINTD0_TD3_SHIFT                                           (0x00000000U)
#define CSL_APP_LIN_LINTD0_TD3_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD0_TD3_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD0_TD2_MASK                                            (0x0000FF00U)
#define CSL_APP_LIN_LINTD0_TD2_SHIFT                                           (0x00000008U)
#define CSL_APP_LIN_LINTD0_TD2_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD0_TD2_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD0_TD1_MASK                                            (0x00FF0000U)
#define CSL_APP_LIN_LINTD0_TD1_SHIFT                                           (0x00000010U)
#define CSL_APP_LIN_LINTD0_TD1_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD0_TD1_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD0_TD0_MASK                                            (0xFF000000U)
#define CSL_APP_LIN_LINTD0_TD0_SHIFT                                           (0x00000018U)
#define CSL_APP_LIN_LINTD0_TD0_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD0_TD0_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD0_RESETVAL                                            (0x00000000U)

/* LINTD1 */

#define CSL_APP_LIN_LINTD1_TD7_MASK                                            (0x000000FFU)
#define CSL_APP_LIN_LINTD1_TD7_SHIFT                                           (0x00000000U)
#define CSL_APP_LIN_LINTD1_TD7_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD1_TD7_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD1_TD6_MASK                                            (0x0000FF00U)
#define CSL_APP_LIN_LINTD1_TD6_SHIFT                                           (0x00000008U)
#define CSL_APP_LIN_LINTD1_TD6_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD1_TD6_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD1_TD5_MASK                                            (0x00FF0000U)
#define CSL_APP_LIN_LINTD1_TD5_SHIFT                                           (0x00000010U)
#define CSL_APP_LIN_LINTD1_TD5_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD1_TD5_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD1_TD4_MASK                                            (0xFF000000U)
#define CSL_APP_LIN_LINTD1_TD4_SHIFT                                           (0x00000018U)
#define CSL_APP_LIN_LINTD1_TD4_RESETVAL                                        (0x00000000U)
#define CSL_APP_LIN_LINTD1_TD4_MAX                                             (0x000000FFU)

#define CSL_APP_LIN_LINTD1_RESETVAL                                            (0x00000000U)

/* MBRSR */

#define CSL_APP_LIN_MBRSR_MBR_MASK                                             (0x00001FFFU)
#define CSL_APP_LIN_MBRSR_MBR_SHIFT                                            (0x00000000U)
#define CSL_APP_LIN_MBRSR_MBR_RESETVAL                                         (0x00000DACU)
#define CSL_APP_LIN_MBRSR_MBR_MAX                                              (0x00001FFFU)

#define CSL_APP_LIN_MBRSR_RESERVED_MASK                                        (0xFFFFE000U)
#define CSL_APP_LIN_MBRSR_RESERVED_SHIFT                                       (0x0000000DU)
#define CSL_APP_LIN_MBRSR_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_APP_LIN_MBRSR_RESERVED_MAX                                         (0x0007FFFFU)

#define CSL_APP_LIN_MBRSR_RESETVAL                                             (0x00000DACU)

/* SCIPIO9 */

#define CSL_APP_LIN_SCIPIO9_CLKSL_MASK                                         (0x00000001U)
#define CSL_APP_LIN_SCIPIO9_CLKSL_SHIFT                                        (0x00000000U)
#define CSL_APP_LIN_SCIPIO9_CLKSL_RESETVAL                                     (0x00000000U)
#define CSL_APP_LIN_SCIPIO9_CLKSL_MAX                                          (0x00000001U)

#define CSL_APP_LIN_SCIPIO9_RXSL_MASK                                          (0x00000002U)
#define CSL_APP_LIN_SCIPIO9_RXSL_SHIFT                                         (0x00000001U)
#define CSL_APP_LIN_SCIPIO9_RXSL_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIPIO9_RXSL_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIPIO9_TXSL_MASK                                          (0x00000004U)
#define CSL_APP_LIN_SCIPIO9_TXSL_SHIFT                                         (0x00000002U)
#define CSL_APP_LIN_SCIPIO9_TXSL_RESETVAL                                      (0x00000000U)
#define CSL_APP_LIN_SCIPIO9_TXSL_MAX                                           (0x00000001U)

#define CSL_APP_LIN_SCIPIO9_RESERVED_MASK                                      (0x0000FFF8U)
#define CSL_APP_LIN_SCIPIO9_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_APP_LIN_SCIPIO9_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_SCIPIO9_RESERVED_MAX                                       (0x00001FFFU)

#define CSL_APP_LIN_SCIPIO9_RESERVED1_MASK                                     (0xFFFF0000U)
#define CSL_APP_LIN_SCIPIO9_RESERVED1_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_SCIPIO9_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_SCIPIO9_RESERVED1_MAX                                      (0x0000FFFFU)

#define CSL_APP_LIN_SCIPIO9_RESETVAL                                           (0x00000000U)

/* RESERVED */

#define CSL_APP_LIN_RESERVED_RESERVED_MASK                                     (0xFFFFFFFFU)
#define CSL_APP_LIN_RESERVED_RESERVED_SHIFT                                    (0x00000000U)
#define CSL_APP_LIN_RESERVED_RESERVED_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_RESERVED_RESERVED_MAX                                      (0xFFFFFFFFU)

#define CSL_APP_LIN_RESERVED_RESETVAL                                          (0x00000000U)

/* RESERVED1 */

#define CSL_APP_LIN_RESERVED1_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_APP_LIN_RESERVED1_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_APP_LIN_RESERVED1_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_RESERVED1_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_APP_LIN_RESERVED1_RESETVAL                                         (0x00000000U)

/* RESERVED2 */

#define CSL_APP_LIN_RESERVED2_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_APP_LIN_RESERVED2_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_APP_LIN_RESERVED2_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_RESERVED2_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_APP_LIN_RESERVED2_RESETVAL                                         (0x00000000U)

/* IODFTCTRL */

#define CSL_APP_LIN_IODFTCTRL_RXPENA_MASK                                      (0x00000001U)
#define CSL_APP_LIN_IODFTCTRL_RXPENA_SHIFT                                     (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_RXPENA_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_RXPENA_MAX                                       (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_LPBENA_MASK                                      (0x00000002U)
#define CSL_APP_LIN_IODFTCTRL_LPBENA_SHIFT                                     (0x00000001U)
#define CSL_APP_LIN_IODFTCTRL_LPBENA_RESETVAL                                  (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_LPBENA_MAX                                       (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_RESERVED_MASK                                    (0x000000FCU)
#define CSL_APP_LIN_IODFTCTRL_RESERVED_SHIFT                                   (0x00000002U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED_MAX                                     (0x0000003FU)

#define CSL_APP_LIN_IODFTCTRL_IODFTENA_MASK                                    (0x00000F00U)
#define CSL_APP_LIN_IODFTCTRL_IODFTENA_SHIFT                                   (0x00000008U)
#define CSL_APP_LIN_IODFTCTRL_IODFTENA_RESETVAL                                (0x00000005U)
#define CSL_APP_LIN_IODFTCTRL_IODFTENA_MAX                                     (0x0000000FU)

#define CSL_APP_LIN_IODFTCTRL_RESERVED1_MASK                                   (0x0000F000U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED1_SHIFT                                  (0x0000000CU)
#define CSL_APP_LIN_IODFTCTRL_RESERVED1_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED1_MAX                                    (0x0000000FU)

#define CSL_APP_LIN_IODFTCTRL_TXSHIFT_MASK                                     (0x00070000U)
#define CSL_APP_LIN_IODFTCTRL_TXSHIFT_SHIFT                                    (0x00000010U)
#define CSL_APP_LIN_IODFTCTRL_TXSHIFT_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_TXSHIFT_MAX                                      (0x00000007U)

#define CSL_APP_LIN_IODFTCTRL_PINSAMPLEMASK_MASK                               (0x00180000U)
#define CSL_APP_LIN_IODFTCTRL_PINSAMPLEMASK_SHIFT                              (0x00000013U)
#define CSL_APP_LIN_IODFTCTRL_PINSAMPLEMASK_RESETVAL                           (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_PINSAMPLEMASK_MAX                                (0x00000003U)

#define CSL_APP_LIN_IODFTCTRL_RESERVED2_MASK                                   (0x00E00000U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED2_SHIFT                                  (0x00000015U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED2_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED2_MAX                                    (0x00000007U)

#define CSL_APP_LIN_IODFTCTRL_BRKDTERRENA_MASK                                 (0x01000000U)
#define CSL_APP_LIN_IODFTCTRL_BRKDTERRENA_SHIFT                                (0x00000018U)
#define CSL_APP_LIN_IODFTCTRL_BRKDTERRENA_RESETVAL                             (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_BRKDTERRENA_MAX                                  (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_PERRENA_MASK                                     (0x02000000U)
#define CSL_APP_LIN_IODFTCTRL_PERRENA_SHIFT                                    (0x00000019U)
#define CSL_APP_LIN_IODFTCTRL_PERRENA_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_PERRENA_MAX                                      (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_FERRENA_MASK                                     (0x04000000U)
#define CSL_APP_LIN_IODFTCTRL_FERRENA_SHIFT                                    (0x0000001AU)
#define CSL_APP_LIN_IODFTCTRL_FERRENA_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_FERRENA_MAX                                      (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_RESERVED3_MASK                                   (0x08000000U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED3_SHIFT                                  (0x0000001BU)
#define CSL_APP_LIN_IODFTCTRL_RESERVED3_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_RESERVED3_MAX                                    (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_ISFERRENA_MASK                                   (0x10000000U)
#define CSL_APP_LIN_IODFTCTRL_ISFERRENA_SHIFT                                  (0x0000001CU)
#define CSL_APP_LIN_IODFTCTRL_ISFERRENA_RESETVAL                               (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_ISFERRENA_MAX                                    (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_CERRENA_MASK                                     (0x20000000U)
#define CSL_APP_LIN_IODFTCTRL_CERRENA_SHIFT                                    (0x0000001DU)
#define CSL_APP_LIN_IODFTCTRL_CERRENA_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_CERRENA_MAX                                      (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_PBERRENA_MASK                                    (0x40000000U)
#define CSL_APP_LIN_IODFTCTRL_PBERRENA_SHIFT                                   (0x0000001EU)
#define CSL_APP_LIN_IODFTCTRL_PBERRENA_RESETVAL                                (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_PBERRENA_MAX                                     (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_BERRENA_MASK                                     (0x80000000U)
#define CSL_APP_LIN_IODFTCTRL_BERRENA_SHIFT                                    (0x0000001FU)
#define CSL_APP_LIN_IODFTCTRL_BERRENA_RESETVAL                                 (0x00000000U)
#define CSL_APP_LIN_IODFTCTRL_BERRENA_MAX                                      (0x00000001U)

#define CSL_APP_LIN_IODFTCTRL_RESETVAL                                         (0x00000500U)

#ifdef __cplusplus
}
#endif

#endif
