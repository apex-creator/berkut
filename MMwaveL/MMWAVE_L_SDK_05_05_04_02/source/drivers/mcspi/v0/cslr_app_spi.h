/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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

/**
*
*  \file   cslr_app_spi.h
*
*  \brief  register-level header file for MCSPI
*
**/

#ifndef CSLR_APP_SPI_H_
#define CSLR_APP_SPI_H_

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
    volatile uint32_t HL_REV;
    volatile uint32_t HL_HWINFO;
    volatile uint8_t  Resv_16[8];
    volatile uint32_t HL_SYSCONFIG;
    volatile uint8_t  Resv_256[236];
    volatile uint32_t REVISION;
    volatile uint8_t  Resv_272[12];
    volatile uint32_t SYSCONFIG;
    volatile uint32_t SYSSTATUS;
    volatile uint32_t IRQSTATUS;
    volatile uint32_t IRQENABLE;
    volatile uint32_t WAKEUPENABLE;
    volatile uint32_t SYST;
    volatile uint32_t MODULCTRL;
    volatile uint32_t CH0CONF;
    volatile uint32_t CH0STAT;
    volatile uint32_t CH0CTRL;
    volatile uint32_t TX0;
    volatile uint32_t RX0;
    volatile uint32_t CH1CONF;
    volatile uint32_t CH1STAT;
    volatile uint32_t CH1CTRL;
    volatile uint32_t TX1;
    volatile uint32_t RX1;
    volatile uint32_t CH2CONF;
    volatile uint32_t CH2STAT;
    volatile uint32_t CH2CTRL;
    volatile uint32_t TX2;
    volatile uint32_t RX2;
    volatile uint32_t CH3CONF;
    volatile uint32_t CH3STAT;
    volatile uint32_t CH3CTRL;
    volatile uint32_t TX3;
    volatile uint32_t RX3;
    volatile uint32_t XFERLEVEL;
    volatile uint32_t DAFTX;
    volatile uint8_t  Resv_416[28];
    volatile uint32_t DAFRX;
} CSL_app_spiRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_APP_SPI_HL_REV                                                     (0x00000000U)
#define CSL_APP_SPI_HL_HWINFO                                                  (0x00000004U)
#define CSL_APP_SPI_HL_SYSCONFIG                                               (0x00000010U)
#define CSL_APP_SPI_REVISION                                                   (0x00000100U)
#define CSL_APP_SPI_SYSCONFIG                                                  (0x00000110U)
#define CSL_APP_SPI_SYSSTATUS                                                  (0x00000114U)
#define CSL_APP_SPI_IRQSTATUS                                                  (0x00000118U)
#define CSL_APP_SPI_IRQENABLE                                                  (0x0000011CU)
#define CSL_APP_SPI_WAKEUPENABLE                                               (0x00000120U)
#define CSL_APP_SPI_SYST                                                       (0x00000124U)
#define CSL_APP_SPI_MODULCTRL                                                  (0x00000128U)
#define CSL_APP_SPI_CH0CONF                                                    (0x0000012CU)
#define CSL_APP_SPI_CH0STAT                                                    (0x00000130U)
#define CSL_APP_SPI_CH0CTRL                                                    (0x00000134U)
#define CSL_APP_SPI_TX0                                                        (0x00000138U)
#define CSL_APP_SPI_RX0                                                        (0x0000013CU)
#define CSL_APP_SPI_CH1CONF                                                    (0x00000140U)
#define CSL_APP_SPI_CH1STAT                                                    (0x00000144U)
#define CSL_APP_SPI_CH1CTRL                                                    (0x00000148U)
#define CSL_APP_SPI_TX1                                                        (0x0000014CU)
#define CSL_APP_SPI_RX1                                                        (0x00000150U)
#define CSL_APP_SPI_CH2CONF                                                    (0x00000154U)
#define CSL_APP_SPI_CH2STAT                                                    (0x00000158U)
#define CSL_APP_SPI_CH2CTRL                                                    (0x0000015CU)
#define CSL_APP_SPI_TX2                                                        (0x00000160U)
#define CSL_APP_SPI_RX2                                                        (0x00000164U)
#define CSL_APP_SPI_CH3CONF                                                    (0x00000168U)
#define CSL_APP_SPI_CH3STAT                                                    (0x0000016CU)
#define CSL_APP_SPI_CH3CTRL                                                    (0x00000170U)
#define CSL_APP_SPI_TX3                                                        (0x00000174U)
#define CSL_APP_SPI_RX3                                                        (0x00000178U)
#define CSL_APP_SPI_XFERLEVEL                                                  (0x0000017CU)
#define CSL_APP_SPI_DAFTX                                                      (0x00000180U)
#define CSL_APP_SPI_DAFRX                                                      (0x000001A0U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* HL_REV */

#define CSL_APP_SPI_HL_REV_Y_MINOR_MASK                                        (0x0000003FU)
#define CSL_APP_SPI_HL_REV_Y_MINOR_SHIFT                                       (0x00000000U)
#define CSL_APP_SPI_HL_REV_Y_MINOR_RESETVAL                                    (0x0000000BU)
#define CSL_APP_SPI_HL_REV_Y_MINOR_MAX                                         (0x0000003FU)

#define CSL_APP_SPI_HL_REV_CUSTOM_MASK                                         (0x000000C0U)
#define CSL_APP_SPI_HL_REV_CUSTOM_SHIFT                                        (0x00000006U)
#define CSL_APP_SPI_HL_REV_CUSTOM_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_HL_REV_CUSTOM_MAX                                          (0x00000003U)

#define CSL_APP_SPI_HL_REV_X_MAJOR_MASK                                        (0x00000700U)
#define CSL_APP_SPI_HL_REV_X_MAJOR_SHIFT                                       (0x00000008U)
#define CSL_APP_SPI_HL_REV_X_MAJOR_RESETVAL                                    (0x00000002U)
#define CSL_APP_SPI_HL_REV_X_MAJOR_MAX                                         (0x00000007U)

#define CSL_APP_SPI_HL_REV_R_RTL_MASK                                          (0x0000F800U)
#define CSL_APP_SPI_HL_REV_R_RTL_SHIFT                                         (0x0000000BU)
#define CSL_APP_SPI_HL_REV_R_RTL_RESETVAL                                      (0x00000003U)
#define CSL_APP_SPI_HL_REV_R_RTL_MAX                                           (0x0000001FU)

#define CSL_APP_SPI_HL_REV_FUNC_MASK                                           (0x0FFF0000U)
#define CSL_APP_SPI_HL_REV_FUNC_SHIFT                                          (0x00000010U)
#define CSL_APP_SPI_HL_REV_FUNC_RESETVAL                                       (0x00000030U)
#define CSL_APP_SPI_HL_REV_FUNC_MAX                                            (0x00000FFFU)

#define CSL_APP_SPI_HL_REV_RSVD_MASK                                           (0x30000000U)
#define CSL_APP_SPI_HL_REV_RSVD_SHIFT                                          (0x0000001CU)
#define CSL_APP_SPI_HL_REV_RSVD_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_HL_REV_RSVD_MAX                                            (0x00000003U)

#define CSL_APP_SPI_HL_REV_SCHEME_MASK                                         (0xC0000000U)
#define CSL_APP_SPI_HL_REV_SCHEME_SHIFT                                        (0x0000001EU)
#define CSL_APP_SPI_HL_REV_SCHEME_RESETVAL                                     (0x00000001U)
#define CSL_APP_SPI_HL_REV_SCHEME_MAX                                          (0x00000003U)

#define CSL_APP_SPI_HL_REV_RESETVAL                                            (0x40301A0BU)

/* HL_HWINFO */

#define CSL_APP_SPI_HL_HWINFO_USEFIFO_MASK                                     (0x00000001U)
#define CSL_APP_SPI_HL_HWINFO_USEFIFO_SHIFT                                    (0x00000000U)
#define CSL_APP_SPI_HL_HWINFO_USEFIFO_RESETVAL                                 (0x00000001U)
#define CSL_APP_SPI_HL_HWINFO_USEFIFO_MAX                                      (0x00000001U)

#define CSL_APP_SPI_HL_HWINFO_FFNBYTE_MASK                                     (0x0000003EU)
#define CSL_APP_SPI_HL_HWINFO_FFNBYTE_SHIFT                                    (0x00000001U)
#define CSL_APP_SPI_HL_HWINFO_FFNBYTE_RESETVAL                                 (0x00000010U)
#define CSL_APP_SPI_HL_HWINFO_FFNBYTE_MAX                                      (0x0000001FU)

#define CSL_APP_SPI_HL_HWINFO_RETMODE_MASK                                     (0x00000040U)
#define CSL_APP_SPI_HL_HWINFO_RETMODE_SHIFT                                    (0x00000006U)
#define CSL_APP_SPI_HL_HWINFO_RETMODE_RESETVAL                                 (0x00000000U)
#define CSL_APP_SPI_HL_HWINFO_RETMODE_MAX                                      (0x00000001U)

#define CSL_APP_SPI_HL_HWINFO_RSVD_MASK                                        (0xFFFFFF80U)
#define CSL_APP_SPI_HL_HWINFO_RSVD_SHIFT                                       (0x00000007U)
#define CSL_APP_SPI_HL_HWINFO_RSVD_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_HL_HWINFO_RSVD_MAX                                         (0x01FFFFFFU)

#define CSL_APP_SPI_HL_HWINFO_RESETVAL                                         (0x00000021U)

/* HL_SYSCONFIG */

#define CSL_APP_SPI_HL_SYSCONFIG_SOFTRESET_MASK                                (0x00000001U)
#define CSL_APP_SPI_HL_SYSCONFIG_SOFTRESET_SHIFT                               (0x00000000U)
#define CSL_APP_SPI_HL_SYSCONFIG_SOFTRESET_RESETVAL                            (0x00000000U)
#define CSL_APP_SPI_HL_SYSCONFIG_SOFTRESET_MAX                                 (0x00000001U)

#define CSL_APP_SPI_HL_SYSCONFIG_FREEEMU_MASK                                  (0x00000002U)
#define CSL_APP_SPI_HL_SYSCONFIG_FREEEMU_SHIFT                                 (0x00000001U)
#define CSL_APP_SPI_HL_SYSCONFIG_FREEEMU_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_HL_SYSCONFIG_FREEEMU_MAX                                   (0x00000001U)

#define CSL_APP_SPI_HL_SYSCONFIG_IDLEMODE_MASK                                 (0x0000000CU)
#define CSL_APP_SPI_HL_SYSCONFIG_IDLEMODE_SHIFT                                (0x00000002U)
#define CSL_APP_SPI_HL_SYSCONFIG_IDLEMODE_RESETVAL                             (0x00000002U)
#define CSL_APP_SPI_HL_SYSCONFIG_IDLEMODE_MAX                                  (0x00000003U)

#define CSL_APP_SPI_HL_SYSCONFIG_RSVD_MASK                                     (0xFFFFFFF0U)
#define CSL_APP_SPI_HL_SYSCONFIG_RSVD_SHIFT                                    (0x00000004U)
#define CSL_APP_SPI_HL_SYSCONFIG_RSVD_RESETVAL                                 (0x00000000U)
#define CSL_APP_SPI_HL_SYSCONFIG_RSVD_MAX                                      (0x0FFFFFFFU)

#define CSL_APP_SPI_HL_SYSCONFIG_RESETVAL                                      (0x00000008U)

/* REVISION */

#define CSL_APP_SPI_REVISION_REV_MASK                                          (0x000000FFU)
#define CSL_APP_SPI_REVISION_REV_SHIFT                                         (0x00000000U)
#define CSL_APP_SPI_REVISION_REV_RESETVAL                                      (0x0000002BU)
#define CSL_APP_SPI_REVISION_REV_MAX                                           (0x000000FFU)

#define CSL_APP_SPI_REVISION_RESERVED_13_MASK                                  (0xFFFFFF00U)
#define CSL_APP_SPI_REVISION_RESERVED_13_SHIFT                                 (0x00000008U)
#define CSL_APP_SPI_REVISION_RESERVED_13_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_REVISION_RESERVED_13_MAX                                   (0x00FFFFFFU)

#define CSL_APP_SPI_REVISION_RESETVAL                                          (0x0000002BU)

/* SYSCONFIG */

#define CSL_APP_SPI_SYSCONFIG_AUTOIDLE_MASK                                    (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_AUTOIDLE_SHIFT                                   (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_AUTOIDLE_RESETVAL                                (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_AUTOIDLE_MAX                                     (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_AUTOIDLE_OFF                                     (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_AUTOIDLE_ON                                      (0x00000001U)

#define CSL_APP_SPI_SYSCONFIG_SOFTRESET_MASK                                   (0x00000002U)
#define CSL_APP_SPI_SYSCONFIG_SOFTRESET_SHIFT                                  (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_SOFTRESET_RESETVAL                               (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_SOFTRESET_MAX                                    (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_SOFTRESET_OFF                                    (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_SOFTRESET_ON                                     (0x00000001U)

#define CSL_APP_SPI_SYSCONFIG_ENAWAKEUP_MASK                                   (0x00000004U)
#define CSL_APP_SPI_SYSCONFIG_ENAWAKEUP_SHIFT                                  (0x00000002U)
#define CSL_APP_SPI_SYSCONFIG_ENAWAKEUP_RESETVAL                               (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_ENAWAKEUP_MAX                                    (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_ENAWAKEUP_NOWAKEUP                               (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_ENAWAKEUP_ON                                     (0x00000001U)

#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_MASK                                   (0x00000018U)
#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_SHIFT                                  (0x00000003U)
#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_RESETVAL                               (0x00000002U)
#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_MAX                                    (0x00000003U)
#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_FORCE                                  (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_NO                                     (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_SMART                                  (0x00000002U)
#define CSL_APP_SPI_SYSCONFIG_SIDLEMODE_SMART_IDLE_WAKEUP                      (0x00000003U)

#define CSL_APP_SPI_SYSCONFIG_RESERVED_15_MASK                                 (0x000000E0U)
#define CSL_APP_SPI_SYSCONFIG_RESERVED_15_SHIFT                                (0x00000005U)
#define CSL_APP_SPI_SYSCONFIG_RESERVED_15_RESETVAL                             (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_RESERVED_15_MAX                                  (0x00000007U)

#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_MASK                               (0x00000300U)
#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_SHIFT                              (0x00000008U)
#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_RESETVAL                           (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_MAX                                (0x00000003U)
#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_NONE                               (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_OCP                                (0x00000001U)
#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_FUNC                               (0x00000002U)
#define CSL_APP_SPI_SYSCONFIG_CLOCKACTIVITY_BOTH                               (0x00000003U)

#define CSL_APP_SPI_SYSCONFIG_RESERVED_14_MASK                                 (0xFFFFFC00U)
#define CSL_APP_SPI_SYSCONFIG_RESERVED_14_SHIFT                                (0x0000000AU)
#define CSL_APP_SPI_SYSCONFIG_RESERVED_14_RESETVAL                             (0x00000000U)
#define CSL_APP_SPI_SYSCONFIG_RESERVED_14_MAX                                  (0x003FFFFFU)

#define CSL_APP_SPI_SYSCONFIG_RESETVAL                                         (0x00000015U)

/* SYSSTATUS */

#define CSL_APP_SPI_SYSSTATUS_RESETDONE_MASK                                   (0x00000001U)
#define CSL_APP_SPI_SYSSTATUS_RESETDONE_SHIFT                                  (0x00000000U)
#define CSL_APP_SPI_SYSSTATUS_RESETDONE_RESETVAL                               (0x00000001U)
#define CSL_APP_SPI_SYSSTATUS_RESETDONE_MAX                                    (0x00000001U)

#define CSL_APP_SPI_SYSSTATUS_RESERVED_16_MASK                                 (0xFFFFFFFEU)
#define CSL_APP_SPI_SYSSTATUS_RESERVED_16_SHIFT                                (0x00000001U)
#define CSL_APP_SPI_SYSSTATUS_RESERVED_16_RESETVAL                             (0x00000000U)
#define CSL_APP_SPI_SYSSTATUS_RESERVED_16_MAX                                  (0x7FFFFFFFU)

#define CSL_APP_SPI_SYSSTATUS_RESETVAL                                         (0x00000001U)

/* IRQSTATUS */

#define CSL_APP_SPI_IRQSTATUS_TX0_EMPTY_MASK                                   (0x00000001U)
#define CSL_APP_SPI_IRQSTATUS_TX0_EMPTY_SHIFT                                  (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX0_EMPTY_RESETVAL                               (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX0_EMPTY_MAX                                    (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_TX0_UNDERFLOW_MASK                               (0x00000002U)
#define CSL_APP_SPI_IRQSTATUS_TX0_UNDERFLOW_SHIFT                              (0x00000001U)
#define CSL_APP_SPI_IRQSTATUS_TX0_UNDERFLOW_RESETVAL                           (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX0_UNDERFLOW_MAX                                (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RX0_FULL_MASK                                    (0x00000004U)
#define CSL_APP_SPI_IRQSTATUS_RX0_FULL_SHIFT                                   (0x00000002U)
#define CSL_APP_SPI_IRQSTATUS_RX0_FULL_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RX0_FULL_MAX                                     (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RX0_OVERFLOW_MASK                                (0x00000008U)
#define CSL_APP_SPI_IRQSTATUS_RX0_OVERFLOW_SHIFT                               (0x00000003U)
#define CSL_APP_SPI_IRQSTATUS_RX0_OVERFLOW_RESETVAL                            (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RX0_OVERFLOW_MAX                                 (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_TX1_EMPTY_MASK                                   (0x00000010U)
#define CSL_APP_SPI_IRQSTATUS_TX1_EMPTY_SHIFT                                  (0x00000004U)
#define CSL_APP_SPI_IRQSTATUS_TX1_EMPTY_RESETVAL                               (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX1_EMPTY_MAX                                    (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_TX1_UNDERFLOW_MASK                               (0x00000020U)
#define CSL_APP_SPI_IRQSTATUS_TX1_UNDERFLOW_SHIFT                              (0x00000005U)
#define CSL_APP_SPI_IRQSTATUS_TX1_UNDERFLOW_RESETVAL                           (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX1_UNDERFLOW_MAX                                (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RX1_FULL_MASK                                    (0x00000040U)
#define CSL_APP_SPI_IRQSTATUS_RX1_FULL_SHIFT                                   (0x00000006U)
#define CSL_APP_SPI_IRQSTATUS_RX1_FULL_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RX1_FULL_MAX                                     (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RESERVED_10_MASK                                 (0x00000080U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_10_SHIFT                                (0x00000007U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_10_RESETVAL                             (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_10_MAX                                  (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_TX2_EMPTY_MASK                                   (0x00000100U)
#define CSL_APP_SPI_IRQSTATUS_TX2_EMPTY_SHIFT                                  (0x00000008U)
#define CSL_APP_SPI_IRQSTATUS_TX2_EMPTY_RESETVAL                               (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX2_EMPTY_MAX                                    (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_TX2_UNDERFLOW_MASK                               (0x00000200U)
#define CSL_APP_SPI_IRQSTATUS_TX2_UNDERFLOW_SHIFT                              (0x00000009U)
#define CSL_APP_SPI_IRQSTATUS_TX2_UNDERFLOW_RESETVAL                           (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX2_UNDERFLOW_MAX                                (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RX2_FULL_MASK                                    (0x00000400U)
#define CSL_APP_SPI_IRQSTATUS_RX2_FULL_SHIFT                                   (0x0000000AU)
#define CSL_APP_SPI_IRQSTATUS_RX2_FULL_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RX2_FULL_MAX                                     (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RESERVED_9_MASK                                  (0x00000800U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_9_SHIFT                                 (0x0000000BU)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_9_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_9_MAX                                   (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_TX3_EMPTY_MASK                                   (0x00001000U)
#define CSL_APP_SPI_IRQSTATUS_TX3_EMPTY_SHIFT                                  (0x0000000CU)
#define CSL_APP_SPI_IRQSTATUS_TX3_EMPTY_RESETVAL                               (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX3_EMPTY_MAX                                    (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_TX3_UNDERFLOW_MASK                               (0x00002000U)
#define CSL_APP_SPI_IRQSTATUS_TX3_UNDERFLOW_SHIFT                              (0x0000000DU)
#define CSL_APP_SPI_IRQSTATUS_TX3_UNDERFLOW_RESETVAL                           (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_TX3_UNDERFLOW_MAX                                (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RX3_FULL_MASK                                    (0x00004000U)
#define CSL_APP_SPI_IRQSTATUS_RX3_FULL_SHIFT                                   (0x0000000EU)
#define CSL_APP_SPI_IRQSTATUS_RX3_FULL_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RX3_FULL_MAX                                     (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RESERVED_7_MASK                                  (0x00008000U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_7_SHIFT                                 (0x0000000FU)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_7_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_7_MAX                                   (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_WKS_MASK                                         (0x00010000U)
#define CSL_APP_SPI_IRQSTATUS_WKS_SHIFT                                        (0x00000010U)
#define CSL_APP_SPI_IRQSTATUS_WKS_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_WKS_MAX                                          (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_EOW_MASK                                         (0x00020000U)
#define CSL_APP_SPI_IRQSTATUS_EOW_SHIFT                                        (0x00000011U)
#define CSL_APP_SPI_IRQSTATUS_EOW_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_EOW_MAX                                          (0x00000001U)

#define CSL_APP_SPI_IRQSTATUS_RESERVED_8_MASK                                  (0xFFFC0000U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_8_SHIFT                                 (0x00000012U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_8_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQSTATUS_RESERVED_8_MAX                                   (0x00003FFFU)

#define CSL_APP_SPI_IRQSTATUS_RESETVAL                                         (0x00000000U)

/* IRQENABLE */

#define CSL_APP_SPI_IRQENABLE_TX0_EMPTY_ENABLE_MASK                            (0x00000001U)
#define CSL_APP_SPI_IRQENABLE_TX0_EMPTY_ENABLE_SHIFT                           (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX0_EMPTY_ENABLE_RESETVAL                        (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX0_EMPTY_ENABLE_MAX                             (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_MASK                        (0x00000002U)
#define CSL_APP_SPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_SHIFT                       (0x00000001U)
#define CSL_APP_SPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_MAX                         (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RX0_FULL_ENABLE_MASK                             (0x00000004U)
#define CSL_APP_SPI_IRQENABLE_RX0_FULL_ENABLE_SHIFT                            (0x00000002U)
#define CSL_APP_SPI_IRQENABLE_RX0_FULL_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RX0_FULL_ENABLE_MAX                              (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RX0_OVERFLOW_ENABLE_MASK                         (0x00000008U)
#define CSL_APP_SPI_IRQENABLE_RX0_OVERFLOW_ENABLE_SHIFT                        (0x00000003U)
#define CSL_APP_SPI_IRQENABLE_RX0_OVERFLOW_ENABLE_RESETVAL                     (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RX0_OVERFLOW_ENABLE_MAX                          (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_TX1_EMPTY_ENABLE_MASK                            (0x00000010U)
#define CSL_APP_SPI_IRQENABLE_TX1_EMPTY_ENABLE_SHIFT                           (0x00000004U)
#define CSL_APP_SPI_IRQENABLE_TX1_EMPTY_ENABLE_RESETVAL                        (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX1_EMPTY_ENABLE_MAX                             (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_MASK                        (0x00000020U)
#define CSL_APP_SPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_SHIFT                       (0x00000005U)
#define CSL_APP_SPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_MAX                         (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RX1_FULL_ENABLE_MASK                             (0x00000040U)
#define CSL_APP_SPI_IRQENABLE_RX1_FULL_ENABLE_SHIFT                            (0x00000006U)
#define CSL_APP_SPI_IRQENABLE_RX1_FULL_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RX1_FULL_ENABLE_MAX                              (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RESERVED_3_MASK                                  (0x00000080U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_3_SHIFT                                 (0x00000007U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_3_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_3_MAX                                   (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_TX2_EMPTY_ENABLE_MASK                            (0x00000100U)
#define CSL_APP_SPI_IRQENABLE_TX2_EMPTY_ENABLE_SHIFT                           (0x00000008U)
#define CSL_APP_SPI_IRQENABLE_TX2_EMPTY_ENABLE_RESETVAL                        (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX2_EMPTY_ENABLE_MAX                             (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_MASK                        (0x00000200U)
#define CSL_APP_SPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_SHIFT                       (0x00000009U)
#define CSL_APP_SPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_MAX                         (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RX2_FULL_ENABLE_MASK                             (0x00000400U)
#define CSL_APP_SPI_IRQENABLE_RX2_FULL_ENABLE_SHIFT                            (0x0000000AU)
#define CSL_APP_SPI_IRQENABLE_RX2_FULL_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RX2_FULL_ENABLE_MAX                              (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RESERVED_6_MASK                                  (0x00000800U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_6_SHIFT                                 (0x0000000BU)
#define CSL_APP_SPI_IRQENABLE_RESERVED_6_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_6_MAX                                   (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_TX3_EMPTY_ENABLE_MASK                            (0x00001000U)
#define CSL_APP_SPI_IRQENABLE_TX3_EMPTY_ENABLE_SHIFT                           (0x0000000CU)
#define CSL_APP_SPI_IRQENABLE_TX3_EMPTY_ENABLE_RESETVAL                        (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX3_EMPTY_ENABLE_MAX                             (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_MASK                        (0x00002000U)
#define CSL_APP_SPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_SHIFT                       (0x0000000DU)
#define CSL_APP_SPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_MAX                         (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RX3_FULL_ENABLE_MASK                             (0x00004000U)
#define CSL_APP_SPI_IRQENABLE_RX3_FULL_ENABLE_SHIFT                            (0x0000000EU)
#define CSL_APP_SPI_IRQENABLE_RX3_FULL_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RX3_FULL_ENABLE_MAX                              (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RESERVED_4_MASK                                  (0x00008000U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_4_SHIFT                                 (0x0000000FU)
#define CSL_APP_SPI_IRQENABLE_RESERVED_4_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_4_MAX                                   (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_WKE_MASK                                         (0x00010000U)
#define CSL_APP_SPI_IRQENABLE_WKE_SHIFT                                        (0x00000010U)
#define CSL_APP_SPI_IRQENABLE_WKE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_WKE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_EOW_ENABLE_MASK                                  (0x00020000U)
#define CSL_APP_SPI_IRQENABLE_EOW_ENABLE_SHIFT                                 (0x00000011U)
#define CSL_APP_SPI_IRQENABLE_EOW_ENABLE_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_EOW_ENABLE_MAX                                   (0x00000001U)

#define CSL_APP_SPI_IRQENABLE_RESERVED_5_MASK                                  (0xFFFC0000U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_5_SHIFT                                 (0x00000012U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_5_RESETVAL                              (0x00000000U)
#define CSL_APP_SPI_IRQENABLE_RESERVED_5_MAX                                   (0x00003FFFU)

#define CSL_APP_SPI_IRQENABLE_RESETVAL                                         (0x00000000U)

/* WAKEUPENABLE */

#define CSL_APP_SPI_WAKEUPENABLE_WKEN_MASK                                     (0x00000001U)
#define CSL_APP_SPI_WAKEUPENABLE_WKEN_SHIFT                                    (0x00000000U)
#define CSL_APP_SPI_WAKEUPENABLE_WKEN_RESETVAL                                 (0x00000000U)
#define CSL_APP_SPI_WAKEUPENABLE_WKEN_MAX                                      (0x00000001U)

#define CSL_APP_SPI_WAKEUPENABLE_RESERVED_18_MASK                              (0xFFFFFFFEU)
#define CSL_APP_SPI_WAKEUPENABLE_RESERVED_18_SHIFT                             (0x00000001U)
#define CSL_APP_SPI_WAKEUPENABLE_RESERVED_18_RESETVAL                          (0x00000000U)
#define CSL_APP_SPI_WAKEUPENABLE_RESERVED_18_MAX                               (0x7FFFFFFFU)

#define CSL_APP_SPI_WAKEUPENABLE_RESETVAL                                      (0x00000000U)

/* SYST */

#define CSL_APP_SPI_SYST_SPIEN_0_MASK                                          (0x00000001U)
#define CSL_APP_SPI_SYST_SPIEN_0_SHIFT                                         (0x00000000U)
#define CSL_APP_SPI_SYST_SPIEN_0_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_SYST_SPIEN_0_MAX                                           (0x00000001U)

#define CSL_APP_SPI_SYST_SPIEN_1_MASK                                          (0x00000002U)
#define CSL_APP_SPI_SYST_SPIEN_1_SHIFT                                         (0x00000001U)
#define CSL_APP_SPI_SYST_SPIEN_1_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_SYST_SPIEN_1_MAX                                           (0x00000001U)

#define CSL_APP_SPI_SYST_SPIEN_2_MASK                                          (0x00000004U)
#define CSL_APP_SPI_SYST_SPIEN_2_SHIFT                                         (0x00000002U)
#define CSL_APP_SPI_SYST_SPIEN_2_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_SYST_SPIEN_2_MAX                                           (0x00000001U)

#define CSL_APP_SPI_SYST_SPIEN_3_MASK                                          (0x00000008U)
#define CSL_APP_SPI_SYST_SPIEN_3_SHIFT                                         (0x00000003U)
#define CSL_APP_SPI_SYST_SPIEN_3_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_SYST_SPIEN_3_MAX                                           (0x00000001U)

#define CSL_APP_SPI_SYST_SPIDAT_0_MASK                                         (0x00000010U)
#define CSL_APP_SPI_SYST_SPIDAT_0_SHIFT                                        (0x00000004U)
#define CSL_APP_SPI_SYST_SPIDAT_0_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_SYST_SPIDAT_0_MAX                                          (0x00000001U)

#define CSL_APP_SPI_SYST_SPIDAT_1_MASK                                         (0x00000020U)
#define CSL_APP_SPI_SYST_SPIDAT_1_SHIFT                                        (0x00000005U)
#define CSL_APP_SPI_SYST_SPIDAT_1_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_SYST_SPIDAT_1_MAX                                          (0x00000001U)

#define CSL_APP_SPI_SYST_SPICLK_MASK                                           (0x00000040U)
#define CSL_APP_SPI_SYST_SPICLK_SHIFT                                          (0x00000006U)
#define CSL_APP_SPI_SYST_SPICLK_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_SYST_SPICLK_MAX                                            (0x00000001U)

#define CSL_APP_SPI_SYST_WAKD_MASK                                             (0x00000080U)
#define CSL_APP_SPI_SYST_WAKD_SHIFT                                            (0x00000007U)
#define CSL_APP_SPI_SYST_WAKD_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_SYST_WAKD_MAX                                              (0x00000001U)

#define CSL_APP_SPI_SYST_SPIDATDIR0_MASK                                       (0x00000100U)
#define CSL_APP_SPI_SYST_SPIDATDIR0_SHIFT                                      (0x00000008U)
#define CSL_APP_SPI_SYST_SPIDATDIR0_RESETVAL                                   (0x00000000U)
#define CSL_APP_SPI_SYST_SPIDATDIR0_MAX                                        (0x00000001U)

#define CSL_APP_SPI_SYST_SPIDATDIR1_MASK                                       (0x00000200U)
#define CSL_APP_SPI_SYST_SPIDATDIR1_SHIFT                                      (0x00000009U)
#define CSL_APP_SPI_SYST_SPIDATDIR1_RESETVAL                                   (0x00000000U)
#define CSL_APP_SPI_SYST_SPIDATDIR1_MAX                                        (0x00000001U)

#define CSL_APP_SPI_SYST_SPIENDIR_MASK                                         (0x00000400U)
#define CSL_APP_SPI_SYST_SPIENDIR_SHIFT                                        (0x0000000AU)
#define CSL_APP_SPI_SYST_SPIENDIR_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_SYST_SPIENDIR_MAX                                          (0x00000001U)

#define CSL_APP_SPI_SYST_SSB_MASK                                              (0x00000800U)
#define CSL_APP_SPI_SYST_SSB_SHIFT                                             (0x0000000BU)
#define CSL_APP_SPI_SYST_SSB_RESETVAL                                          (0x00000000U)
#define CSL_APP_SPI_SYST_SSB_MAX                                               (0x00000001U)
#define CSL_APP_SPI_SYST_SSB_OFF                                               (0x00000000U)
#define CSL_APP_SPI_SYST_SSB_SETTHEMALL                                        (0x00000001U)

#define CSL_APP_SPI_SYST_RESERVED_17_MASK                                      (0xFFFFF000U)
#define CSL_APP_SPI_SYST_RESERVED_17_SHIFT                                     (0x0000000CU)
#define CSL_APP_SPI_SYST_RESERVED_17_RESETVAL                                  (0x00000000U)
#define CSL_APP_SPI_SYST_RESERVED_17_MAX                                       (0x000FFFFFU)

#define CSL_APP_SPI_SYST_RESETVAL                                              (0x00000000U)

/* MODULCTRL */

#define CSL_APP_SPI_MODULCTRL_SINGLE_MASK                                      (0x00000001U)
#define CSL_APP_SPI_MODULCTRL_SINGLE_SHIFT                                     (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_SINGLE_RESETVAL                                  (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_SINGLE_MAX                                       (0x00000001U)
#define CSL_APP_SPI_MODULCTRL_SINGLE_MULTI                                     (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_SINGLE_SINGLE                                    (0x00000001U)

#define CSL_APP_SPI_MODULCTRL_PIN34_MASK                                       (0x00000002U)
#define CSL_APP_SPI_MODULCTRL_PIN34_SHIFT                                      (0x00000001U)
#define CSL_APP_SPI_MODULCTRL_PIN34_RESETVAL                                   (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_PIN34_MAX                                        (0x00000001U)
#define CSL_APP_SPI_MODULCTRL_PIN34_3PINMODE                                   (0x00000001U)
#define CSL_APP_SPI_MODULCTRL_PIN34_4PINMODE                                   (0x00000000U)

#define CSL_APP_SPI_MODULCTRL_MS_MASK                                          (0x00000004U)
#define CSL_APP_SPI_MODULCTRL_MS_SHIFT                                         (0x00000002U)
#define CSL_APP_SPI_MODULCTRL_MS_RESETVAL                                      (0x00000001U)
#define CSL_APP_SPI_MODULCTRL_MS_MAX                                           (0x00000001U)
#define CSL_APP_SPI_MODULCTRL_MS_MASTER                                        (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_MS_SLAVE                                         (0x00000001U)

#define CSL_APP_SPI_MODULCTRL_SYSTEM_TEST_MASK                                 (0x00000008U)
#define CSL_APP_SPI_MODULCTRL_SYSTEM_TEST_SHIFT                                (0x00000003U)
#define CSL_APP_SPI_MODULCTRL_SYSTEM_TEST_RESETVAL                             (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_SYSTEM_TEST_MAX                                  (0x00000001U)

#define CSL_APP_SPI_MODULCTRL_INITDLY_MASK                                     (0x00000070U)
#define CSL_APP_SPI_MODULCTRL_INITDLY_SHIFT                                    (0x00000004U)
#define CSL_APP_SPI_MODULCTRL_INITDLY_RESETVAL                                 (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_INITDLY_MAX                                      (0x00000007U)
#define CSL_APP_SPI_MODULCTRL_INITDLY_NODELAY                                  (0x00000000U)

#define CSL_APP_SPI_MODULCTRL_MOA_MASK                                         (0x00000080U)
#define CSL_APP_SPI_MODULCTRL_MOA_SHIFT                                        (0x00000007U)
#define CSL_APP_SPI_MODULCTRL_MOA_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_MOA_MAX                                          (0x00000001U)

#define CSL_APP_SPI_MODULCTRL_FDAA_MASK                                        (0x00000100U)
#define CSL_APP_SPI_MODULCTRL_FDAA_SHIFT                                       (0x00000008U)
#define CSL_APP_SPI_MODULCTRL_FDAA_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_FDAA_MAX                                         (0x00000001U)

#define CSL_APP_SPI_MODULCTRL_RESERVED_11_MASK                                 (0xFFFFFE00U)
#define CSL_APP_SPI_MODULCTRL_RESERVED_11_SHIFT                                (0x00000009U)
#define CSL_APP_SPI_MODULCTRL_RESERVED_11_RESETVAL                             (0x00000000U)
#define CSL_APP_SPI_MODULCTRL_RESERVED_11_MAX                                  (0x007FFFFFU)

#define CSL_APP_SPI_MODULCTRL_RESETVAL                                         (0x00000004U)

/* CH0CONF */

#define CSL_APP_SPI_CH0CONF_PHA_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_PHA_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH0CONF_PHA_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH0CONF_PHA_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH0CONF_POL_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH0CONF_POL_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH0CONF_POL_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH0CONF_POL_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH0CONF_CLKD_MASK                                          (0x0000003CU)
#define CSL_APP_SPI_CH0CONF_CLKD_SHIFT                                         (0x00000002U)
#define CSL_APP_SPI_CH0CONF_CLKD_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_CLKD_MAX                                           (0x0000000FU)

#define CSL_APP_SPI_CH0CONF_EPOL_MASK                                          (0x00000040U)
#define CSL_APP_SPI_CH0CONF_EPOL_SHIFT                                         (0x00000006U)
#define CSL_APP_SPI_CH0CONF_EPOL_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_EPOL_MAX                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_EPOL_ACTIVELOW                                     (0x00000001U)
#define CSL_APP_SPI_CH0CONF_EPOL_ACTIVEHIGH                                    (0x00000000U)

#define CSL_APP_SPI_CH0CONF_WL_MASK                                            (0x00000F80U)
#define CSL_APP_SPI_CH0CONF_WL_SHIFT                                           (0x00000007U)
#define CSL_APP_SPI_CH0CONF_WL_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH0CONF_WL_MAX                                             (0x0000001FU)

#define CSL_APP_SPI_CH0CONF_TRM_MASK                                           (0x00003000U)
#define CSL_APP_SPI_CH0CONF_TRM_SHIFT                                          (0x0000000CU)
#define CSL_APP_SPI_CH0CONF_TRM_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH0CONF_TRM_MAX                                            (0x00000003U)
#define CSL_APP_SPI_CH0CONF_TRM_TRANSONLY                                      (0x00000002U)
#define CSL_APP_SPI_CH0CONF_TRM_RSVD                                           (0x00000003U)
#define CSL_APP_SPI_CH0CONF_TRM_TRANSRECEI                                     (0x00000000U)
#define CSL_APP_SPI_CH0CONF_TRM_RECEIVONLY                                     (0x00000001U)

#define CSL_APP_SPI_CH0CONF_DMAW_MASK                                          (0x00004000U)
#define CSL_APP_SPI_CH0CONF_DMAW_SHIFT                                         (0x0000000EU)
#define CSL_APP_SPI_CH0CONF_DMAW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_DMAW_MAX                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_DMAW_ENABLED                                       (0x00000001U)
#define CSL_APP_SPI_CH0CONF_DMAW_DISABLED                                      (0x00000000U)

#define CSL_APP_SPI_CH0CONF_DMAR_MASK                                          (0x00008000U)
#define CSL_APP_SPI_CH0CONF_DMAR_SHIFT                                         (0x0000000FU)
#define CSL_APP_SPI_CH0CONF_DMAR_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_DMAR_MAX                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_DMAR_ENABLED                                       (0x00000001U)
#define CSL_APP_SPI_CH0CONF_DMAR_DISABLED                                      (0x00000000U)

#define CSL_APP_SPI_CH0CONF_DPE0_MASK                                          (0x00010000U)
#define CSL_APP_SPI_CH0CONF_DPE0_SHIFT                                         (0x00000010U)
#define CSL_APP_SPI_CH0CONF_DPE0_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_DPE0_MAX                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_DPE0_DISABLED                                      (0x00000001U)
#define CSL_APP_SPI_CH0CONF_DPE0_ENABLED                                       (0x00000000U)

#define CSL_APP_SPI_CH0CONF_DPE1_MASK                                          (0x00020000U)
#define CSL_APP_SPI_CH0CONF_DPE1_SHIFT                                         (0x00000011U)
#define CSL_APP_SPI_CH0CONF_DPE1_RESETVAL                                      (0x00000001U)
#define CSL_APP_SPI_CH0CONF_DPE1_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH0CONF_IS_MASK                                            (0x00040000U)
#define CSL_APP_SPI_CH0CONF_IS_SHIFT                                           (0x00000012U)
#define CSL_APP_SPI_CH0CONF_IS_RESETVAL                                        (0x00000001U)
#define CSL_APP_SPI_CH0CONF_IS_MAX                                             (0x00000001U)
#define CSL_APP_SPI_CH0CONF_IS_LINE0                                           (0x00000000U)
#define CSL_APP_SPI_CH0CONF_IS_LINE1                                           (0x00000001U)

#define CSL_APP_SPI_CH0CONF_TURBO_MASK                                         (0x00080000U)
#define CSL_APP_SPI_CH0CONF_TURBO_SHIFT                                        (0x00000013U)
#define CSL_APP_SPI_CH0CONF_TURBO_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH0CONF_TURBO_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH0CONF_FORCE_MASK                                         (0x00100000U)
#define CSL_APP_SPI_CH0CONF_FORCE_SHIFT                                        (0x00000014U)
#define CSL_APP_SPI_CH0CONF_FORCE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH0CONF_FORCE_MAX                                          (0x00000001U)
#define CSL_APP_SPI_CH0CONF_FORCE_DEASSERT                                     (0x00000000U)
#define CSL_APP_SPI_CH0CONF_FORCE_ASSERT                                       (0x00000001U)

#define CSL_APP_SPI_CH0CONF_SPIENSLV_MASK                                      (0x00600000U)
#define CSL_APP_SPI_CH0CONF_SPIENSLV_SHIFT                                     (0x00000015U)
#define CSL_APP_SPI_CH0CONF_SPIENSLV_RESETVAL                                  (0x00000000U)
#define CSL_APP_SPI_CH0CONF_SPIENSLV_MAX                                       (0x00000003U)
#define CSL_APP_SPI_CH0CONF_SPIENSLV_SPIEN2                                    (0x00000002U)
#define CSL_APP_SPI_CH0CONF_SPIENSLV_SPIEN3                                    (0x00000003U)
#define CSL_APP_SPI_CH0CONF_SPIENSLV_SPIEN0                                    (0x00000000U)
#define CSL_APP_SPI_CH0CONF_SPIENSLV_SPIEN1                                    (0x00000001U)

#define CSL_APP_SPI_CH0CONF_SBE_MASK                                           (0x00800000U)
#define CSL_APP_SPI_CH0CONF_SBE_SHIFT                                          (0x00000017U)
#define CSL_APP_SPI_CH0CONF_SBE_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH0CONF_SBE_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH0CONF_SBPOL_MASK                                         (0x01000000U)
#define CSL_APP_SPI_CH0CONF_SBPOL_SHIFT                                        (0x00000018U)
#define CSL_APP_SPI_CH0CONF_SBPOL_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH0CONF_SBPOL_MAX                                          (0x00000001U)
#define CSL_APP_SPI_CH0CONF_SBPOL_LOWLEVEL                                     (0x00000000U)
#define CSL_APP_SPI_CH0CONF_SBPOL_HIGHLEVEL                                    (0x00000001U)

#define CSL_APP_SPI_CH0CONF_TCS0_MASK                                          (0x06000000U)
#define CSL_APP_SPI_CH0CONF_TCS0_SHIFT                                         (0x00000019U)
#define CSL_APP_SPI_CH0CONF_TCS0_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_TCS0_MAX                                           (0x00000003U)
#define CSL_APP_SPI_CH0CONF_TCS0_THREECYCLEDLY                                 (0x00000003U)
#define CSL_APP_SPI_CH0CONF_TCS0_TWOCYCLEDLY                                   (0x00000002U)
#define CSL_APP_SPI_CH0CONF_TCS0_ONECYCLEDLY                                   (0x00000001U)
#define CSL_APP_SPI_CH0CONF_TCS0_ZEROCYCLEDLY                                  (0x00000000U)

#define CSL_APP_SPI_CH0CONF_FFEW_MASK                                          (0x08000000U)
#define CSL_APP_SPI_CH0CONF_FFEW_SHIFT                                         (0x0000001BU)
#define CSL_APP_SPI_CH0CONF_FFEW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_FFEW_MAX                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_FFEW_FFENABLED                                     (0x00000001U)
#define CSL_APP_SPI_CH0CONF_FFEW_FFDISABLED                                    (0x00000000U)

#define CSL_APP_SPI_CH0CONF_FFER_MASK                                          (0x10000000U)
#define CSL_APP_SPI_CH0CONF_FFER_SHIFT                                         (0x0000001CU)
#define CSL_APP_SPI_CH0CONF_FFER_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_FFER_MAX                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_FFER_FFENABLED                                     (0x00000001U)
#define CSL_APP_SPI_CH0CONF_FFER_FFDISABLED                                    (0x00000000U)

#define CSL_APP_SPI_CH0CONF_CLKG_MASK                                          (0x20000000U)
#define CSL_APP_SPI_CH0CONF_CLKG_SHIFT                                         (0x0000001DU)
#define CSL_APP_SPI_CH0CONF_CLKG_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH0CONF_CLKG_MAX                                           (0x00000001U)
#define CSL_APP_SPI_CH0CONF_CLKG_ONECYCLE                                      (0x00000001U)
#define CSL_APP_SPI_CH0CONF_CLKG_POWERTWO                                      (0x00000000U)

#define CSL_APP_SPI_CH0CONF_RESERVED_0_MASK                                    (0xC0000000U)
#define CSL_APP_SPI_CH0CONF_RESERVED_0_SHIFT                                   (0x0000001EU)
#define CSL_APP_SPI_CH0CONF_RESERVED_0_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH0CONF_RESERVED_0_MAX                                     (0x00000003U)

#define CSL_APP_SPI_CH0CONF_RESETVAL                                           (0x00060000U)

/* CH0STAT */

#define CSL_APP_SPI_CH0STAT_RXS_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH0STAT_RXS_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH0STAT_RXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH0STAT_RXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH0STAT_TXS_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH0STAT_TXS_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH0STAT_TXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH0STAT_TXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH0STAT_EOT_MASK                                           (0x00000004U)
#define CSL_APP_SPI_CH0STAT_EOT_SHIFT                                          (0x00000002U)
#define CSL_APP_SPI_CH0STAT_EOT_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH0STAT_EOT_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH0STAT_TXFFE_MASK                                         (0x00000008U)
#define CSL_APP_SPI_CH0STAT_TXFFE_SHIFT                                        (0x00000003U)
#define CSL_APP_SPI_CH0STAT_TXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH0STAT_TXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH0STAT_TXFFF_MASK                                         (0x00000010U)
#define CSL_APP_SPI_CH0STAT_TXFFF_SHIFT                                        (0x00000004U)
#define CSL_APP_SPI_CH0STAT_TXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH0STAT_TXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH0STAT_RXFFE_MASK                                         (0x00000020U)
#define CSL_APP_SPI_CH0STAT_RXFFE_SHIFT                                        (0x00000005U)
#define CSL_APP_SPI_CH0STAT_RXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH0STAT_RXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH0STAT_RXFFF_MASK                                         (0x00000040U)
#define CSL_APP_SPI_CH0STAT_RXFFF_SHIFT                                        (0x00000006U)
#define CSL_APP_SPI_CH0STAT_RXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH0STAT_RXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH0STAT_RESERVED_2_MASK                                    (0xFFFFFF80U)
#define CSL_APP_SPI_CH0STAT_RESERVED_2_SHIFT                                   (0x00000007U)
#define CSL_APP_SPI_CH0STAT_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH0STAT_RESERVED_2_MAX                                     (0x01FFFFFFU)

#define CSL_APP_SPI_CH0STAT_RESETVAL                                           (0x00000000U)

/* CH0CTRL */

#define CSL_APP_SPI_CH0CTRL_EN_MASK                                            (0x00000001U)
#define CSL_APP_SPI_CH0CTRL_EN_SHIFT                                           (0x00000000U)
#define CSL_APP_SPI_CH0CTRL_EN_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH0CTRL_EN_MAX                                             (0x00000001U)
#define CSL_APP_SPI_CH0CTRL_EN_ACT                                             (0x00000001U)
#define CSL_APP_SPI_CH0CTRL_EN_NACT                                            (0x00000000U)

#define CSL_APP_SPI_CH0CTRL_RESERVED_1_MASK                                    (0x000000FEU)
#define CSL_APP_SPI_CH0CTRL_RESERVED_1_SHIFT                                   (0x00000001U)
#define CSL_APP_SPI_CH0CTRL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH0CTRL_RESERVED_1_MAX                                     (0x0000007FU)

#define CSL_APP_SPI_CH0CTRL_EXTCLK_MASK                                        (0x0000FF00U)
#define CSL_APP_SPI_CH0CTRL_EXTCLK_SHIFT                                       (0x00000008U)
#define CSL_APP_SPI_CH0CTRL_EXTCLK_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_CH0CTRL_EXTCLK_MAX                                         (0x000000FFU)

#define CSL_APP_SPI_CH0CTRL_RESERVED_2_MASK                                    (0xFFFF0000U)
#define CSL_APP_SPI_CH0CTRL_RESERVED_2_SHIFT                                   (0x00000010U)
#define CSL_APP_SPI_CH0CTRL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH0CTRL_RESERVED_2_MAX                                     (0x0000FFFFU)

#define CSL_APP_SPI_CH0CTRL_RESETVAL                                           (0x00000000U)

/* TX0 */

#define CSL_APP_SPI_TX0_TDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_TX0_TDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_TX0_TDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_TX0_TDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_TX0_RESETVAL                                               (0x00000000U)

/* RX0 */

#define CSL_APP_SPI_RX0_RDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_RX0_RDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_RX0_RDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_RX0_RDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_RX0_RESETVAL                                               (0x00000000U)

/* CH1CONF */

#define CSL_APP_SPI_CH1CONF_PHA_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH1CONF_PHA_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH1CONF_PHA_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH1CONF_PHA_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH1CONF_POL_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH1CONF_POL_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH1CONF_POL_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH1CONF_POL_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH1CONF_CLKD_MASK                                          (0x0000003CU)
#define CSL_APP_SPI_CH1CONF_CLKD_SHIFT                                         (0x00000002U)
#define CSL_APP_SPI_CH1CONF_CLKD_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_CLKD_MAX                                           (0x0000000FU)

#define CSL_APP_SPI_CH1CONF_EPOL_MASK                                          (0x00000040U)
#define CSL_APP_SPI_CH1CONF_EPOL_SHIFT                                         (0x00000006U)
#define CSL_APP_SPI_CH1CONF_EPOL_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_EPOL_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_WL_MASK                                            (0x00000F80U)
#define CSL_APP_SPI_CH1CONF_WL_SHIFT                                           (0x00000007U)
#define CSL_APP_SPI_CH1CONF_WL_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH1CONF_WL_MAX                                             (0x0000001FU)

#define CSL_APP_SPI_CH1CONF_TRM_MASK                                           (0x00003000U)
#define CSL_APP_SPI_CH1CONF_TRM_SHIFT                                          (0x0000000CU)
#define CSL_APP_SPI_CH1CONF_TRM_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH1CONF_TRM_MAX                                            (0x00000003U)

#define CSL_APP_SPI_CH1CONF_DMAW_MASK                                          (0x00004000U)
#define CSL_APP_SPI_CH1CONF_DMAW_SHIFT                                         (0x0000000EU)
#define CSL_APP_SPI_CH1CONF_DMAW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_DMAW_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_DMAR_MASK                                          (0x00008000U)
#define CSL_APP_SPI_CH1CONF_DMAR_SHIFT                                         (0x0000000FU)
#define CSL_APP_SPI_CH1CONF_DMAR_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_DMAR_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_DPE0_MASK                                          (0x00010000U)
#define CSL_APP_SPI_CH1CONF_DPE0_SHIFT                                         (0x00000010U)
#define CSL_APP_SPI_CH1CONF_DPE0_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_DPE0_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_DPE1_MASK                                          (0x00020000U)
#define CSL_APP_SPI_CH1CONF_DPE1_SHIFT                                         (0x00000011U)
#define CSL_APP_SPI_CH1CONF_DPE1_RESETVAL                                      (0x00000001U)
#define CSL_APP_SPI_CH1CONF_DPE1_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_IS_MASK                                            (0x00040000U)
#define CSL_APP_SPI_CH1CONF_IS_SHIFT                                           (0x00000012U)
#define CSL_APP_SPI_CH1CONF_IS_RESETVAL                                        (0x00000001U)
#define CSL_APP_SPI_CH1CONF_IS_MAX                                             (0x00000001U)

#define CSL_APP_SPI_CH1CONF_TURBO_MASK                                         (0x00080000U)
#define CSL_APP_SPI_CH1CONF_TURBO_SHIFT                                        (0x00000013U)
#define CSL_APP_SPI_CH1CONF_TURBO_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH1CONF_TURBO_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH1CONF_FORCE_MASK                                         (0x00100000U)
#define CSL_APP_SPI_CH1CONF_FORCE_SHIFT                                        (0x00000014U)
#define CSL_APP_SPI_CH1CONF_FORCE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH1CONF_FORCE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH1CONF_RESERVED_1_MASK                                    (0x00600000U)
#define CSL_APP_SPI_CH1CONF_RESERVED_1_SHIFT                                   (0x00000015U)
#define CSL_APP_SPI_CH1CONF_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH1CONF_RESERVED_1_MAX                                     (0x00000003U)

#define CSL_APP_SPI_CH1CONF_SBE_MASK                                           (0x00800000U)
#define CSL_APP_SPI_CH1CONF_SBE_SHIFT                                          (0x00000017U)
#define CSL_APP_SPI_CH1CONF_SBE_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH1CONF_SBE_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH1CONF_SBPOL_MASK                                         (0x01000000U)
#define CSL_APP_SPI_CH1CONF_SBPOL_SHIFT                                        (0x00000018U)
#define CSL_APP_SPI_CH1CONF_SBPOL_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH1CONF_SBPOL_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH1CONF_TCS1_MASK                                          (0x06000000U)
#define CSL_APP_SPI_CH1CONF_TCS1_SHIFT                                         (0x00000019U)
#define CSL_APP_SPI_CH1CONF_TCS1_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_TCS1_MAX                                           (0x00000003U)

#define CSL_APP_SPI_CH1CONF_FFEW_MASK                                          (0x08000000U)
#define CSL_APP_SPI_CH1CONF_FFEW_SHIFT                                         (0x0000001BU)
#define CSL_APP_SPI_CH1CONF_FFEW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_FFEW_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_FFER_MASK                                          (0x10000000U)
#define CSL_APP_SPI_CH1CONF_FFER_SHIFT                                         (0x0000001CU)
#define CSL_APP_SPI_CH1CONF_FFER_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_FFER_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_CLKG_MASK                                          (0x20000000U)
#define CSL_APP_SPI_CH1CONF_CLKG_SHIFT                                         (0x0000001DU)
#define CSL_APP_SPI_CH1CONF_CLKG_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH1CONF_CLKG_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH1CONF_RESERVED_0_MASK                                    (0xC0000000U)
#define CSL_APP_SPI_CH1CONF_RESERVED_0_SHIFT                                   (0x0000001EU)
#define CSL_APP_SPI_CH1CONF_RESERVED_0_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH1CONF_RESERVED_0_MAX                                     (0x00000003U)

#define CSL_APP_SPI_CH1CONF_RESETVAL                                           (0x00060000U)

/* CH1STAT */

#define CSL_APP_SPI_CH1STAT_RXS_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH1STAT_RXS_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH1STAT_RXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH1STAT_RXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH1STAT_TXS_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH1STAT_TXS_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH1STAT_TXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH1STAT_TXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH1STAT_EOT_MASK                                           (0x00000004U)
#define CSL_APP_SPI_CH1STAT_EOT_SHIFT                                          (0x00000002U)
#define CSL_APP_SPI_CH1STAT_EOT_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH1STAT_EOT_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH1STAT_TXFFE_MASK                                         (0x00000008U)
#define CSL_APP_SPI_CH1STAT_TXFFE_SHIFT                                        (0x00000003U)
#define CSL_APP_SPI_CH1STAT_TXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH1STAT_TXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH1STAT_TXFFF_MASK                                         (0x00000010U)
#define CSL_APP_SPI_CH1STAT_TXFFF_SHIFT                                        (0x00000004U)
#define CSL_APP_SPI_CH1STAT_TXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH1STAT_TXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH1STAT_RXFFE_MASK                                         (0x00000020U)
#define CSL_APP_SPI_CH1STAT_RXFFE_SHIFT                                        (0x00000005U)
#define CSL_APP_SPI_CH1STAT_RXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH1STAT_RXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH1STAT_RXFFF_MASK                                         (0x00000040U)
#define CSL_APP_SPI_CH1STAT_RXFFF_SHIFT                                        (0x00000006U)
#define CSL_APP_SPI_CH1STAT_RXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH1STAT_RXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH1STAT_RESERVED_2_MASK                                    (0xFFFFFF80U)
#define CSL_APP_SPI_CH1STAT_RESERVED_2_SHIFT                                   (0x00000007U)
#define CSL_APP_SPI_CH1STAT_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH1STAT_RESERVED_2_MAX                                     (0x01FFFFFFU)

#define CSL_APP_SPI_CH1STAT_RESETVAL                                           (0x00000000U)

/* CH1CTRL */

#define CSL_APP_SPI_CH1CTRL_EN_MASK                                            (0x00000001U)
#define CSL_APP_SPI_CH1CTRL_EN_SHIFT                                           (0x00000000U)
#define CSL_APP_SPI_CH1CTRL_EN_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH1CTRL_EN_MAX                                             (0x00000001U)

#define CSL_APP_SPI_CH1CTRL_RESERVED_1_MASK                                    (0x000000FEU)
#define CSL_APP_SPI_CH1CTRL_RESERVED_1_SHIFT                                   (0x00000001U)
#define CSL_APP_SPI_CH1CTRL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH1CTRL_RESERVED_1_MAX                                     (0x0000007FU)

#define CSL_APP_SPI_CH1CTRL_EXTCLK_MASK                                        (0x0000FF00U)
#define CSL_APP_SPI_CH1CTRL_EXTCLK_SHIFT                                       (0x00000008U)
#define CSL_APP_SPI_CH1CTRL_EXTCLK_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_CH1CTRL_EXTCLK_MAX                                         (0x000000FFU)

#define CSL_APP_SPI_CH1CTRL_RESERVED_2_MASK                                    (0xFFFF0000U)
#define CSL_APP_SPI_CH1CTRL_RESERVED_2_SHIFT                                   (0x00000010U)
#define CSL_APP_SPI_CH1CTRL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH1CTRL_RESERVED_2_MAX                                     (0x0000FFFFU)

#define CSL_APP_SPI_CH1CTRL_RESETVAL                                           (0x00000000U)

/* TX1 */

#define CSL_APP_SPI_TX1_TDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_TX1_TDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_TX1_TDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_TX1_TDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_TX1_RESETVAL                                               (0x00000000U)

/* RX1 */

#define CSL_APP_SPI_RX1_RDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_RX1_RDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_RX1_RDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_RX1_RDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_RX1_RESETVAL                                               (0x00000000U)

/* CH2CONF */

#define CSL_APP_SPI_CH2CONF_PHA_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH2CONF_PHA_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH2CONF_PHA_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH2CONF_PHA_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH2CONF_POL_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH2CONF_POL_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH2CONF_POL_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH2CONF_POL_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH2CONF_CLKD_MASK                                          (0x0000003CU)
#define CSL_APP_SPI_CH2CONF_CLKD_SHIFT                                         (0x00000002U)
#define CSL_APP_SPI_CH2CONF_CLKD_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_CLKD_MAX                                           (0x0000000FU)

#define CSL_APP_SPI_CH2CONF_EPOL_MASK                                          (0x00000040U)
#define CSL_APP_SPI_CH2CONF_EPOL_SHIFT                                         (0x00000006U)
#define CSL_APP_SPI_CH2CONF_EPOL_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_EPOL_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_WL_MASK                                            (0x00000F80U)
#define CSL_APP_SPI_CH2CONF_WL_SHIFT                                           (0x00000007U)
#define CSL_APP_SPI_CH2CONF_WL_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH2CONF_WL_MAX                                             (0x0000001FU)

#define CSL_APP_SPI_CH2CONF_TRM_MASK                                           (0x00003000U)
#define CSL_APP_SPI_CH2CONF_TRM_SHIFT                                          (0x0000000CU)
#define CSL_APP_SPI_CH2CONF_TRM_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH2CONF_TRM_MAX                                            (0x00000003U)

#define CSL_APP_SPI_CH2CONF_DMAW_MASK                                          (0x00004000U)
#define CSL_APP_SPI_CH2CONF_DMAW_SHIFT                                         (0x0000000EU)
#define CSL_APP_SPI_CH2CONF_DMAW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_DMAW_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_DMAR_MASK                                          (0x00008000U)
#define CSL_APP_SPI_CH2CONF_DMAR_SHIFT                                         (0x0000000FU)
#define CSL_APP_SPI_CH2CONF_DMAR_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_DMAR_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_DPE0_MASK                                          (0x00010000U)
#define CSL_APP_SPI_CH2CONF_DPE0_SHIFT                                         (0x00000010U)
#define CSL_APP_SPI_CH2CONF_DPE0_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_DPE0_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_DPE1_MASK                                          (0x00020000U)
#define CSL_APP_SPI_CH2CONF_DPE1_SHIFT                                         (0x00000011U)
#define CSL_APP_SPI_CH2CONF_DPE1_RESETVAL                                      (0x00000001U)
#define CSL_APP_SPI_CH2CONF_DPE1_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_IS_MASK                                            (0x00040000U)
#define CSL_APP_SPI_CH2CONF_IS_SHIFT                                           (0x00000012U)
#define CSL_APP_SPI_CH2CONF_IS_RESETVAL                                        (0x00000001U)
#define CSL_APP_SPI_CH2CONF_IS_MAX                                             (0x00000001U)

#define CSL_APP_SPI_CH2CONF_TURBO_MASK                                         (0x00080000U)
#define CSL_APP_SPI_CH2CONF_TURBO_SHIFT                                        (0x00000013U)
#define CSL_APP_SPI_CH2CONF_TURBO_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH2CONF_TURBO_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH2CONF_FORCE_MASK                                         (0x00100000U)
#define CSL_APP_SPI_CH2CONF_FORCE_SHIFT                                        (0x00000014U)
#define CSL_APP_SPI_CH2CONF_FORCE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH2CONF_FORCE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH2CONF_RESERVED_1_MASK                                    (0x00600000U)
#define CSL_APP_SPI_CH2CONF_RESERVED_1_SHIFT                                   (0x00000015U)
#define CSL_APP_SPI_CH2CONF_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH2CONF_RESERVED_1_MAX                                     (0x00000003U)

#define CSL_APP_SPI_CH2CONF_SBE_MASK                                           (0x00800000U)
#define CSL_APP_SPI_CH2CONF_SBE_SHIFT                                          (0x00000017U)
#define CSL_APP_SPI_CH2CONF_SBE_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH2CONF_SBE_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH2CONF_SBPOL_MASK                                         (0x01000000U)
#define CSL_APP_SPI_CH2CONF_SBPOL_SHIFT                                        (0x00000018U)
#define CSL_APP_SPI_CH2CONF_SBPOL_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH2CONF_SBPOL_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH2CONF_TCS2_MASK                                          (0x06000000U)
#define CSL_APP_SPI_CH2CONF_TCS2_SHIFT                                         (0x00000019U)
#define CSL_APP_SPI_CH2CONF_TCS2_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_TCS2_MAX                                           (0x00000003U)

#define CSL_APP_SPI_CH2CONF_FFEW_MASK                                          (0x08000000U)
#define CSL_APP_SPI_CH2CONF_FFEW_SHIFT                                         (0x0000001BU)
#define CSL_APP_SPI_CH2CONF_FFEW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_FFEW_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_FFER_MASK                                          (0x10000000U)
#define CSL_APP_SPI_CH2CONF_FFER_SHIFT                                         (0x0000001CU)
#define CSL_APP_SPI_CH2CONF_FFER_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_FFER_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_CLKG_MASK                                          (0x20000000U)
#define CSL_APP_SPI_CH2CONF_CLKG_SHIFT                                         (0x0000001DU)
#define CSL_APP_SPI_CH2CONF_CLKG_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH2CONF_CLKG_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH2CONF_RESERVED_0_MASK                                    (0xC0000000U)
#define CSL_APP_SPI_CH2CONF_RESERVED_0_SHIFT                                   (0x0000001EU)
#define CSL_APP_SPI_CH2CONF_RESERVED_0_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH2CONF_RESERVED_0_MAX                                     (0x00000003U)

#define CSL_APP_SPI_CH2CONF_RESETVAL                                           (0x00060000U)

/* CH2STAT */

#define CSL_APP_SPI_CH2STAT_RXS_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH2STAT_RXS_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH2STAT_RXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH2STAT_RXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH2STAT_TXS_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH2STAT_TXS_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH2STAT_TXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH2STAT_TXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH2STAT_EOT_MASK                                           (0x00000004U)
#define CSL_APP_SPI_CH2STAT_EOT_SHIFT                                          (0x00000002U)
#define CSL_APP_SPI_CH2STAT_EOT_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH2STAT_EOT_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH2STAT_TXFFE_MASK                                         (0x00000008U)
#define CSL_APP_SPI_CH2STAT_TXFFE_SHIFT                                        (0x00000003U)
#define CSL_APP_SPI_CH2STAT_TXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH2STAT_TXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH2STAT_TXFFF_MASK                                         (0x00000010U)
#define CSL_APP_SPI_CH2STAT_TXFFF_SHIFT                                        (0x00000004U)
#define CSL_APP_SPI_CH2STAT_TXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH2STAT_TXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH2STAT_RXFFE_MASK                                         (0x00000020U)
#define CSL_APP_SPI_CH2STAT_RXFFE_SHIFT                                        (0x00000005U)
#define CSL_APP_SPI_CH2STAT_RXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH2STAT_RXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH2STAT_RXFFF_MASK                                         (0x00000040U)
#define CSL_APP_SPI_CH2STAT_RXFFF_SHIFT                                        (0x00000006U)
#define CSL_APP_SPI_CH2STAT_RXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH2STAT_RXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH2STAT_RESERVED_2_MASK                                    (0xFFFFFF80U)
#define CSL_APP_SPI_CH2STAT_RESERVED_2_SHIFT                                   (0x00000007U)
#define CSL_APP_SPI_CH2STAT_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH2STAT_RESERVED_2_MAX                                     (0x01FFFFFFU)

#define CSL_APP_SPI_CH2STAT_RESETVAL                                           (0x00000000U)

/* CH2CTRL */

#define CSL_APP_SPI_CH2CTRL_EN_MASK                                            (0x00000001U)
#define CSL_APP_SPI_CH2CTRL_EN_SHIFT                                           (0x00000000U)
#define CSL_APP_SPI_CH2CTRL_EN_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH2CTRL_EN_MAX                                             (0x00000001U)

#define CSL_APP_SPI_CH2CTRL_RESERVED_1_MASK                                    (0x000000FEU)
#define CSL_APP_SPI_CH2CTRL_RESERVED_1_SHIFT                                   (0x00000001U)
#define CSL_APP_SPI_CH2CTRL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH2CTRL_RESERVED_1_MAX                                     (0x0000007FU)

#define CSL_APP_SPI_CH2CTRL_EXTCLK_MASK                                        (0x0000FF00U)
#define CSL_APP_SPI_CH2CTRL_EXTCLK_SHIFT                                       (0x00000008U)
#define CSL_APP_SPI_CH2CTRL_EXTCLK_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_CH2CTRL_EXTCLK_MAX                                         (0x000000FFU)

#define CSL_APP_SPI_CH2CTRL_RESERVED_2_MASK                                    (0xFFFF0000U)
#define CSL_APP_SPI_CH2CTRL_RESERVED_2_SHIFT                                   (0x00000010U)
#define CSL_APP_SPI_CH2CTRL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH2CTRL_RESERVED_2_MAX                                     (0x0000FFFFU)

#define CSL_APP_SPI_CH2CTRL_RESETVAL                                           (0x00000000U)

/* TX2 */

#define CSL_APP_SPI_TX2_TDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_TX2_TDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_TX2_TDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_TX2_TDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_TX2_RESETVAL                                               (0x00000000U)

/* RX2 */

#define CSL_APP_SPI_RX2_RDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_RX2_RDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_RX2_RDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_RX2_RDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_RX2_RESETVAL                                               (0x00000000U)

/* CH3CONF */

#define CSL_APP_SPI_CH3CONF_PHA_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH3CONF_PHA_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH3CONF_PHA_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH3CONF_PHA_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH3CONF_POL_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH3CONF_POL_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH3CONF_POL_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH3CONF_POL_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH3CONF_CLKD_MASK                                          (0x0000003CU)
#define CSL_APP_SPI_CH3CONF_CLKD_SHIFT                                         (0x00000002U)
#define CSL_APP_SPI_CH3CONF_CLKD_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_CLKD_MAX                                           (0x0000000FU)

#define CSL_APP_SPI_CH3CONF_EPOL_MASK                                          (0x00000040U)
#define CSL_APP_SPI_CH3CONF_EPOL_SHIFT                                         (0x00000006U)
#define CSL_APP_SPI_CH3CONF_EPOL_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_EPOL_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_WL_MASK                                            (0x00000F80U)
#define CSL_APP_SPI_CH3CONF_WL_SHIFT                                           (0x00000007U)
#define CSL_APP_SPI_CH3CONF_WL_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH3CONF_WL_MAX                                             (0x0000001FU)

#define CSL_APP_SPI_CH3CONF_TRM_MASK                                           (0x00003000U)
#define CSL_APP_SPI_CH3CONF_TRM_SHIFT                                          (0x0000000CU)
#define CSL_APP_SPI_CH3CONF_TRM_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH3CONF_TRM_MAX                                            (0x00000003U)

#define CSL_APP_SPI_CH3CONF_DMAW_MASK                                          (0x00004000U)
#define CSL_APP_SPI_CH3CONF_DMAW_SHIFT                                         (0x0000000EU)
#define CSL_APP_SPI_CH3CONF_DMAW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_DMAW_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_DMAR_MASK                                          (0x00008000U)
#define CSL_APP_SPI_CH3CONF_DMAR_SHIFT                                         (0x0000000FU)
#define CSL_APP_SPI_CH3CONF_DMAR_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_DMAR_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_DPE0_MASK                                          (0x00010000U)
#define CSL_APP_SPI_CH3CONF_DPE0_SHIFT                                         (0x00000010U)
#define CSL_APP_SPI_CH3CONF_DPE0_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_DPE0_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_DPE1_MASK                                          (0x00020000U)
#define CSL_APP_SPI_CH3CONF_DPE1_SHIFT                                         (0x00000011U)
#define CSL_APP_SPI_CH3CONF_DPE1_RESETVAL                                      (0x00000001U)
#define CSL_APP_SPI_CH3CONF_DPE1_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_IS_MASK                                            (0x00040000U)
#define CSL_APP_SPI_CH3CONF_IS_SHIFT                                           (0x00000012U)
#define CSL_APP_SPI_CH3CONF_IS_RESETVAL                                        (0x00000001U)
#define CSL_APP_SPI_CH3CONF_IS_MAX                                             (0x00000001U)

#define CSL_APP_SPI_CH3CONF_TURBO_MASK                                         (0x00080000U)
#define CSL_APP_SPI_CH3CONF_TURBO_SHIFT                                        (0x00000013U)
#define CSL_APP_SPI_CH3CONF_TURBO_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH3CONF_TURBO_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH3CONF_FORCE_MASK                                         (0x00100000U)
#define CSL_APP_SPI_CH3CONF_FORCE_SHIFT                                        (0x00000014U)
#define CSL_APP_SPI_CH3CONF_FORCE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH3CONF_FORCE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH3CONF_RESERVED_1_MASK                                    (0x00600000U)
#define CSL_APP_SPI_CH3CONF_RESERVED_1_SHIFT                                   (0x00000015U)
#define CSL_APP_SPI_CH3CONF_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH3CONF_RESERVED_1_MAX                                     (0x00000003U)

#define CSL_APP_SPI_CH3CONF_SBE_MASK                                           (0x00800000U)
#define CSL_APP_SPI_CH3CONF_SBE_SHIFT                                          (0x00000017U)
#define CSL_APP_SPI_CH3CONF_SBE_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH3CONF_SBE_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH3CONF_SBPOL_MASK                                         (0x01000000U)
#define CSL_APP_SPI_CH3CONF_SBPOL_SHIFT                                        (0x00000018U)
#define CSL_APP_SPI_CH3CONF_SBPOL_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH3CONF_SBPOL_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH3CONF_TCS3_MASK                                          (0x06000000U)
#define CSL_APP_SPI_CH3CONF_TCS3_SHIFT                                         (0x00000019U)
#define CSL_APP_SPI_CH3CONF_TCS3_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_TCS3_MAX                                           (0x00000003U)

#define CSL_APP_SPI_CH3CONF_FFEW_MASK                                          (0x08000000U)
#define CSL_APP_SPI_CH3CONF_FFEW_SHIFT                                         (0x0000001BU)
#define CSL_APP_SPI_CH3CONF_FFEW_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_FFEW_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_FFER_MASK                                          (0x10000000U)
#define CSL_APP_SPI_CH3CONF_FFER_SHIFT                                         (0x0000001CU)
#define CSL_APP_SPI_CH3CONF_FFER_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_FFER_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_CLKG_MASK                                          (0x20000000U)
#define CSL_APP_SPI_CH3CONF_CLKG_SHIFT                                         (0x0000001DU)
#define CSL_APP_SPI_CH3CONF_CLKG_RESETVAL                                      (0x00000000U)
#define CSL_APP_SPI_CH3CONF_CLKG_MAX                                           (0x00000001U)

#define CSL_APP_SPI_CH3CONF_RESERVED_0_MASK                                    (0xC0000000U)
#define CSL_APP_SPI_CH3CONF_RESERVED_0_SHIFT                                   (0x0000001EU)
#define CSL_APP_SPI_CH3CONF_RESERVED_0_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH3CONF_RESERVED_0_MAX                                     (0x00000003U)

#define CSL_APP_SPI_CH3CONF_RESETVAL                                           (0x00060000U)

/* CH3STAT */

#define CSL_APP_SPI_CH3STAT_RXS_MASK                                           (0x00000001U)
#define CSL_APP_SPI_CH3STAT_RXS_SHIFT                                          (0x00000000U)
#define CSL_APP_SPI_CH3STAT_RXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH3STAT_RXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH3STAT_TXS_MASK                                           (0x00000002U)
#define CSL_APP_SPI_CH3STAT_TXS_SHIFT                                          (0x00000001U)
#define CSL_APP_SPI_CH3STAT_TXS_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH3STAT_TXS_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH3STAT_EOT_MASK                                           (0x00000004U)
#define CSL_APP_SPI_CH3STAT_EOT_SHIFT                                          (0x00000002U)
#define CSL_APP_SPI_CH3STAT_EOT_RESETVAL                                       (0x00000000U)
#define CSL_APP_SPI_CH3STAT_EOT_MAX                                            (0x00000001U)

#define CSL_APP_SPI_CH3STAT_TXFFE_MASK                                         (0x00000008U)
#define CSL_APP_SPI_CH3STAT_TXFFE_SHIFT                                        (0x00000003U)
#define CSL_APP_SPI_CH3STAT_TXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH3STAT_TXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH3STAT_TXFFF_MASK                                         (0x00000010U)
#define CSL_APP_SPI_CH3STAT_TXFFF_SHIFT                                        (0x00000004U)
#define CSL_APP_SPI_CH3STAT_TXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH3STAT_TXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH3STAT_RXFFE_MASK                                         (0x00000020U)
#define CSL_APP_SPI_CH3STAT_RXFFE_SHIFT                                        (0x00000005U)
#define CSL_APP_SPI_CH3STAT_RXFFE_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH3STAT_RXFFE_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH3STAT_RXFFF_MASK                                         (0x00000040U)
#define CSL_APP_SPI_CH3STAT_RXFFF_SHIFT                                        (0x00000006U)
#define CSL_APP_SPI_CH3STAT_RXFFF_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_CH3STAT_RXFFF_MAX                                          (0x00000001U)

#define CSL_APP_SPI_CH3STAT_RESERVED_2_MASK                                    (0xFFFFFF80U)
#define CSL_APP_SPI_CH3STAT_RESERVED_2_SHIFT                                   (0x00000007U)
#define CSL_APP_SPI_CH3STAT_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH3STAT_RESERVED_2_MAX                                     (0x01FFFFFFU)

#define CSL_APP_SPI_CH3STAT_RESETVAL                                           (0x00000000U)

/* CH3CTRL */

#define CSL_APP_SPI_CH3CTRL_EN_MASK                                            (0x00000001U)
#define CSL_APP_SPI_CH3CTRL_EN_SHIFT                                           (0x00000000U)
#define CSL_APP_SPI_CH3CTRL_EN_RESETVAL                                        (0x00000000U)
#define CSL_APP_SPI_CH3CTRL_EN_MAX                                             (0x00000001U)

#define CSL_APP_SPI_CH3CTRL_RESERVED_1_MASK                                    (0x000000FEU)
#define CSL_APP_SPI_CH3CTRL_RESERVED_1_SHIFT                                   (0x00000001U)
#define CSL_APP_SPI_CH3CTRL_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH3CTRL_RESERVED_1_MAX                                     (0x0000007FU)

#define CSL_APP_SPI_CH3CTRL_EXTCLK_MASK                                        (0x0000FF00U)
#define CSL_APP_SPI_CH3CTRL_EXTCLK_SHIFT                                       (0x00000008U)
#define CSL_APP_SPI_CH3CTRL_EXTCLK_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_CH3CTRL_EXTCLK_MAX                                         (0x000000FFU)

#define CSL_APP_SPI_CH3CTRL_RESERVED_2_MASK                                    (0xFFFF0000U)
#define CSL_APP_SPI_CH3CTRL_RESERVED_2_SHIFT                                   (0x00000010U)
#define CSL_APP_SPI_CH3CTRL_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_APP_SPI_CH3CTRL_RESERVED_2_MAX                                     (0x0000FFFFU)

#define CSL_APP_SPI_CH3CTRL_RESETVAL                                           (0x00000000U)

/* TX3 */

#define CSL_APP_SPI_TX3_TDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_TX3_TDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_TX3_TDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_TX3_TDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_TX3_RESETVAL                                               (0x00000000U)

/* RX3 */

#define CSL_APP_SPI_RX3_RDATA_MASK                                             (0xFFFFFFFFU)
#define CSL_APP_SPI_RX3_RDATA_SHIFT                                            (0x00000000U)
#define CSL_APP_SPI_RX3_RDATA_RESETVAL                                         (0x00000000U)
#define CSL_APP_SPI_RX3_RDATA_MAX                                              (0xFFFFFFFFU)

#define CSL_APP_SPI_RX3_RESETVAL                                               (0x00000000U)

/* XFERLEVEL */

#define CSL_APP_SPI_XFERLEVEL_AEL_MASK                                         (0x000000FFU)
#define CSL_APP_SPI_XFERLEVEL_AEL_SHIFT                                        (0x00000000U)
#define CSL_APP_SPI_XFERLEVEL_AEL_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_XFERLEVEL_AEL_MAX                                          (0x000000FFU)

#define CSL_APP_SPI_XFERLEVEL_AFL_MASK                                         (0x0000FF00U)
#define CSL_APP_SPI_XFERLEVEL_AFL_SHIFT                                        (0x00000008U)
#define CSL_APP_SPI_XFERLEVEL_AFL_RESETVAL                                     (0x00000000U)
#define CSL_APP_SPI_XFERLEVEL_AFL_MAX                                          (0x000000FFU)

#define CSL_APP_SPI_XFERLEVEL_WCNT_MASK                                        (0xFFFF0000U)
#define CSL_APP_SPI_XFERLEVEL_WCNT_SHIFT                                       (0x00000010U)
#define CSL_APP_SPI_XFERLEVEL_WCNT_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_XFERLEVEL_WCNT_MAX                                         (0x0000FFFFU)

#define CSL_APP_SPI_XFERLEVEL_RESETVAL                                         (0x00000000U)

/* DAFTX */

#define CSL_APP_SPI_DAFTX_DAFTDATA_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_SPI_DAFTX_DAFTDATA_SHIFT                                       (0x00000000U)
#define CSL_APP_SPI_DAFTX_DAFTDATA_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_DAFTX_DAFTDATA_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_SPI_DAFTX_RESETVAL                                             (0x00000000U)

/* DAFRX */

#define CSL_APP_SPI_DAFRX_DAFRDATA_MASK                                        (0xFFFFFFFFU)
#define CSL_APP_SPI_DAFRX_DAFRDATA_SHIFT                                       (0x00000000U)
#define CSL_APP_SPI_DAFRX_DAFRDATA_RESETVAL                                    (0x00000000U)
#define CSL_APP_SPI_DAFRX_DAFRDATA_MAX                                         (0xFFFFFFFFU)

#define CSL_APP_SPI_DAFRX_RESETVAL                                             (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
