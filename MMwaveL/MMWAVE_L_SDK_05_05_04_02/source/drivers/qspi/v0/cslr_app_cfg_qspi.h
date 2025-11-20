/********************************************************************
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
 *  Name        : cslr_app_cfg_qspi.h
*/
#ifndef CSLR_APP_CFG_QSPI_H_
#define CSLR_APP_CFG_QSPI_H_

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
    volatile uint32_t PID;
    volatile uint32_t MSS_QSPI_RESERVED1;
    volatile uint32_t MSS_QSPI_RESERVED2;
    volatile uint32_t MSS_QSPI_RESERVED3;
    volatile uint32_t SYSCONFIG;
    volatile uint32_t MSS_QSPI_RESERVED4;
    volatile uint32_t MSS_QSPI_RESERVED5;
    volatile uint32_t MSS_QSPI_RESERVED6;
    volatile uint32_t INTR_STATUS_RAW_SET;
    volatile uint32_t INTR_STATUS_ENABLED_CLEAR;
    volatile uint32_t INTR_ENABLE_SET;
    volatile uint32_t INTR_ENABLE_CLEAR;
    volatile uint32_t INTC_EOI;
    volatile uint32_t MSS_QSPI_RESERVED7;
    volatile uint32_t MSS_QSPI_RESERVED8;
    volatile uint32_t MSS_QSPI_RESERVED9;
    volatile uint32_t SPI_CLOCK_CNTRL;
    volatile uint32_t SPI_DC;
    volatile uint32_t SPI_CMD;
    volatile uint32_t SPI_STATUS;
    volatile uint32_t SPI_DATA;
    volatile uint32_t SPI_SETUP0;
    volatile uint32_t SPI_SETUP1;
    volatile uint32_t SPI_SETUP2;
    volatile uint32_t SPI_SETUP3;
    volatile uint32_t SPI_SWITCH;
    volatile uint32_t SPI_DATA1;
    volatile uint32_t SPI_DATA2;
    volatile uint32_t SPI_DATA3;
} CSL_app_cfg_qspiRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_APP_CFG_QSPI_PID                                                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED1                                    (0x00000004U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED2                                    (0x00000008U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED3                                    (0x0000000CU)
#define CSL_APP_CFG_QSPI_SYSCONFIG                                             (0x00000010U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED4                                    (0x00000014U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED5                                    (0x00000018U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED6                                    (0x0000001CU)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET                                   (0x00000020U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR                             (0x00000024U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET                                       (0x00000028U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR                                     (0x0000002CU)
#define CSL_APP_CFG_QSPI_INTC_EOI                                              (0x00000030U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED7                                    (0x00000034U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED8                                    (0x00000038U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED9                                    (0x0000003CU)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL                                       (0x00000040U)
#define CSL_APP_CFG_QSPI_SPI_DC                                                (0x00000044U)
#define CSL_APP_CFG_QSPI_SPI_CMD                                               (0x00000048U)
#define CSL_APP_CFG_QSPI_SPI_STATUS                                            (0x0000004CU)
#define CSL_APP_CFG_QSPI_SPI_DATA                                              (0x00000050U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0                                            (0x00000054U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1                                            (0x00000058U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2                                            (0x0000005CU)
#define CSL_APP_CFG_QSPI_SPI_SETUP3                                            (0x00000060U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH                                            (0x00000064U)
#define CSL_APP_CFG_QSPI_SPI_DATA1                                             (0x00000068U)
#define CSL_APP_CFG_QSPI_SPI_DATA2                                             (0x0000006CU)
#define CSL_APP_CFG_QSPI_SPI_DATA3                                             (0x00000070U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_APP_CFG_QSPI_PID_MINOR_MASK                                        (0x0000003FU)
#define CSL_APP_CFG_QSPI_PID_MINOR_SHIFT                                       (0x00000000U)
#define CSL_APP_CFG_QSPI_PID_MINOR_RESETVAL                                    (0x00000000U)
#define CSL_APP_CFG_QSPI_PID_MINOR_MAX                                         (0x0000003FU)

#define CSL_APP_CFG_QSPI_PID_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_APP_CFG_QSPI_PID_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_APP_CFG_QSPI_PID_CUSTOM_RESETVAL                                   (0x00000000U)
#define CSL_APP_CFG_QSPI_PID_CUSTOM_MAX                                        (0x00000003U)

#define CSL_APP_CFG_QSPI_PID_MAJOR_MASK                                        (0x00000700U)
#define CSL_APP_CFG_QSPI_PID_MAJOR_SHIFT                                       (0x00000008U)
#define CSL_APP_CFG_QSPI_PID_MAJOR_RESETVAL                                    (0x00000000U)
#define CSL_APP_CFG_QSPI_PID_MAJOR_MAX                                         (0x00000007U)

#define CSL_APP_CFG_QSPI_PID_RTL_MASK                                          (0x0000F800U)
#define CSL_APP_CFG_QSPI_PID_RTL_SHIFT                                         (0x0000000BU)
#define CSL_APP_CFG_QSPI_PID_RTL_RESETVAL                                      (0x00000000U)
#define CSL_APP_CFG_QSPI_PID_RTL_MAX                                           (0x0000001FU)

#define CSL_APP_CFG_QSPI_PID_FUNC_MASK                                         (0x0FFF0000U)
#define CSL_APP_CFG_QSPI_PID_FUNC_SHIFT                                        (0x00000010U)
#define CSL_APP_CFG_QSPI_PID_FUNC_RESETVAL                                     (0x00000F40U)
#define CSL_APP_CFG_QSPI_PID_FUNC_MAX                                          (0x00000FFFU)

#define CSL_APP_CFG_QSPI_PID_RESERVED_MASK                                     (0x30000000U)
#define CSL_APP_CFG_QSPI_PID_RESERVED_SHIFT                                    (0x0000001CU)
#define CSL_APP_CFG_QSPI_PID_RESERVED_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_PID_RESERVED_MAX                                      (0x00000003U)

#define CSL_APP_CFG_QSPI_PID_SCHEME_MASK                                       (0xC0000000U)
#define CSL_APP_CFG_QSPI_PID_SCHEME_SHIFT                                      (0x0000001EU)
#define CSL_APP_CFG_QSPI_PID_SCHEME_RESETVAL                                   (0x00000001U)
#define CSL_APP_CFG_QSPI_PID_SCHEME_MAX                                        (0x00000003U)

#define CSL_APP_CFG_QSPI_PID_RESETVAL                                          (0x4F400000U)

/* MSS_QSPI_RESERVED1 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED1_RESERVED_1_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED1_RESERVED_1_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED1_RESERVED_1_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED1_RESERVED_1_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED1_RESETVAL                           (0x00000000U)

/* MSS_QSPI_RESERVED2 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED2_RESERVED_2_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED2_RESERVED_2_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED2_RESERVED_2_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED2_RESERVED_2_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED2_RESETVAL                           (0x00000000U)

/* MSS_QSPI_RESERVED3 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED3_RESERVED_3_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED3_RESERVED_3_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED3_RESERVED_3_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED3_RESERVED_3_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED3_RESETVAL                           (0x00000000U)

/* SYSCONFIG */

#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED1_MASK                              (0x00000003U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED1_SHIFT                             (0x00000000U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED1_RESETVAL                          (0x00000000U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED1_MAX                               (0x00000003U)

#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_MASK                               (0x0000000CU)
#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_SHIFT                              (0x00000002U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_RESETVAL                           (0x00000002U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_MAX                                (0x00000003U)

#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED2_MASK                              (0x00000030U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED2_SHIFT                             (0x00000004U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED2_RESETVAL                          (0x00000000U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED2_MAX                               (0x00000003U)

#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED3_MASK                              (0xFFFFFFC0U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED3_SHIFT                             (0x00000006U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED3_RESETVAL                          (0x00000000U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_RESERVED3_MAX                               (0x03FFFFFFU)

#define CSL_APP_CFG_QSPI_SYSCONFIG_RESETVAL                                    (0x00000008U)

#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_FORCE_IDLE                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_NO_IDLE                            (0x00000001U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_SMART_IDLE                         (0x00000002U)
#define CSL_APP_CFG_QSPI_SYSCONFIG_IDLEMODE_SMART_IDLE_WAKEUP_CAPABLE          (0x00000003U)

/* MSS_QSPI_RESERVED4 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED4_RESERVED_4_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED4_RESERVED_4_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED4_RESERVED_4_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED4_RESERVED_4_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED4_RESETVAL                           (0x00000000U)

/* MSS_QSPI_RESERVED5 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED5_RESERVED_5_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED5_RESERVED_5_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED5_RESERVED_5_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED5_RESERVED_5_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED5_RESETVAL                           (0x00000000U)

/* MSS_QSPI_RESERVED6 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED6_RESERVED_6_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED6_RESERVED_6_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED6_RESERVED_6_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED6_RESERVED_6_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED6_RESETVAL                           (0x00000000U)

/* INTR_STATUS_RAW_SET */

#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_MASK                     (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_SHIFT                    (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_RESETVAL                 (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_FIRQ_RAW_MAX                      (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_MASK                     (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_SHIFT                    (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_RESETVAL                 (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_WIRQ_RAW_MAX                      (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_RESERVED_MASK                     (0xFFFFFFFCU)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_RESERVED_SHIFT                    (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_RESERVED_RESETVAL                 (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_RESERVED_MAX                      (0x3FFFFFFFU)

#define CSL_APP_CFG_QSPI_INTR_STATUS_RAW_SET_RESETVAL                          (0x00000000U)

/* INTR_STATUS_ENABLED_CLEAR */

#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_MASK               (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_SHIFT              (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_RESETVAL           (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_MAX                (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_MASK               (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_SHIFT              (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_RESETVAL           (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_MAX                (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_RESERVED_MASK               (0xFFFFFFFCU)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_RESERVED_SHIFT              (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_RESERVED_RESETVAL           (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_RESERVED_MAX                (0x3FFFFFFFU)

#define CSL_APP_CFG_QSPI_INTR_STATUS_ENABLED_CLEAR_RESETVAL                    (0x00000000U)

/* INTR_ENABLE_SET */

#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_FIRQ_ENA_SET_MASK                     (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_FIRQ_ENA_SET_SHIFT                    (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_FIRQ_ENA_SET_RESETVAL                 (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_FIRQ_ENA_SET_MAX                      (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_WIRQ_ENA_SET_MASK                     (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_WIRQ_ENA_SET_SHIFT                    (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_WIRQ_ENA_SET_RESETVAL                 (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_WIRQ_ENA_SET_MAX                      (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_RESERVED_MASK                         (0xFFFFFFFCU)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_RESERVED_SHIFT                        (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_RESERVED_RESETVAL                     (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_RESERVED_MAX                          (0x3FFFFFFFU)

#define CSL_APP_CFG_QSPI_INTR_ENABLE_SET_RESETVAL                              (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_FIRQ_ENA_CLR_MASK                   (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_FIRQ_ENA_CLR_SHIFT                  (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_FIRQ_ENA_CLR_RESETVAL               (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_FIRQ_ENA_CLR_MAX                    (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_WIRQ_ENA_CLR_MASK                   (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_WIRQ_ENA_CLR_SHIFT                  (0x00000001U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_WIRQ_ENA_CLR_RESETVAL               (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_WIRQ_ENA_CLR_MAX                    (0x00000001U)

#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_RESERVED_MASK                       (0xFFFFFFFCU)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_RESERVED_SHIFT                      (0x00000002U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_RESERVED_RESETVAL                   (0x00000000U)
#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_RESERVED_MAX                        (0x3FFFFFFFU)

#define CSL_APP_CFG_QSPI_INTR_ENABLE_CLEAR_RESETVAL                            (0x00000000U)

/* INTC_EOI */

#define CSL_APP_CFG_QSPI_INTC_EOI_EOI_VECTOR_MASK                              (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_INTC_EOI_EOI_VECTOR_SHIFT                             (0x00000000U)
#define CSL_APP_CFG_QSPI_INTC_EOI_EOI_VECTOR_RESETVAL                          (0x00000000U)
#define CSL_APP_CFG_QSPI_INTC_EOI_EOI_VECTOR_MAX                               (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_INTC_EOI_RESETVAL                                     (0x00000000U)

/* MSS_QSPI_RESERVED7 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED7_RESERVED_7_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED7_RESERVED_7_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED7_RESERVED_7_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED7_RESERVED_7_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED7_RESETVAL                           (0x00000000U)

/* MSS_QSPI_RESERVED8 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED8_RESERVED_8_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED8_RESERVED_8_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED8_RESERVED_8_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED8_RESERVED_8_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED8_RESETVAL                           (0x00000000U)

/* MSS_QSPI_RESERVED9 */

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED9_RESERVED_9_MASK                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED9_RESERVED_9_SHIFT                   (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED9_RESERVED_9_RESETVAL                (0x00000000U)
#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED9_RESERVED_9_MAX                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_MSS_QSPI_RESERVED9_RESETVAL                           (0x00000000U)

/* SPI_CLOCK_CNTRL */

#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_DCLK_DIV_MASK                         (0x0000FFFFU)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_DCLK_DIV_SHIFT                        (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_DCLK_DIV_RESETVAL                     (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_DCLK_DIV_MAX                          (0x0000FFFFU)

#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_RESERVED_MASK                         (0x7FFF0000U)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_RESERVED_SHIFT                        (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_RESERVED_RESETVAL                     (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_RESERVED_MAX                          (0x00007FFFU)

#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_CLKEN_MASK                            (0x80000000U)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_CLKEN_SHIFT                           (0x0000001FU)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_CLKEN_RESETVAL                        (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_CLKEN_MAX                             (0x00000001U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_ON            (0x00000001U)
#define CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_OFF           (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_CLOCK_CNTRL_RESETVAL                              (0x00000000U)

/* SPI_DC */

#define CSL_APP_CFG_QSPI_SPI_DC_CKP0_MASK                                      (0x00000001U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP0_SHIFT                                     (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP0_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP0_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP0_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP0_DATA_INACTIVE                  (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_CSP0_MASK                                      (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP0_SHIFT                                     (0x00000001U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP0_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP0_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CSP0_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP0_ACTIVE_HIGH                    (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_DC_CKPH0_MASK                                     (0x00000004U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH0_SHIFT                                    (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH0_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH0_MAX                                      (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH0_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_DD0_MASK                                       (0x00000018U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD0_SHIFT                                      (0x00000003U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD0_RESETVAL                                   (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD0_MAX                                        (0x00000003U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED1_MASK                                 (0x000000E0U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED1_SHIFT                                (0x00000005U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED1_RESETVAL                             (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED1_MAX                                  (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_DC_CKP1_MASK                                      (0x00000100U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP1_SHIFT                                     (0x00000008U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP1_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP1_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP1_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP1_DATA_INACTIVE                  (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_CSP1_MASK                                      (0x00000200U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP1_SHIFT                                     (0x00000009U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP1_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP1_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CSP1_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP1_ACTIVE_HIGH                    (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_DC_CKPH1_MASK                                     (0x00000400U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH1_SHIFT                                    (0x0000000AU)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH1_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH1_MAX                                      (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH1_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_DD1_MASK                                       (0x00001800U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD1_SHIFT                                      (0x0000000BU)
#define CSL_APP_CFG_QSPI_SPI_DC_DD1_RESETVAL                                   (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD1_MAX                                        (0x00000003U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD1_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED2_MASK                                 (0x0000E000U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED2_SHIFT                                (0x0000000DU)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED2_RESETVAL                             (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED2_MAX                                  (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_DC_CKP2_MASK                                      (0x00010000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP2_SHIFT                                     (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP2_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP2_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP2_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP2_DATA_INACTIVE                  (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_CSP2_MASK                                      (0x00020000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP2_SHIFT                                     (0x00000011U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP2_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP2_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CSP2_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP2_ACTIVE_HIGH                    (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_DC_CKPH2_MASK                                     (0x00040000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH2_SHIFT                                    (0x00000012U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH2_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH2_MAX                                      (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH2_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_DD2_MASK                                       (0x00180000U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD2_SHIFT                                      (0x00000013U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD2_RESETVAL                                   (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD2_MAX                                        (0x00000003U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD2_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED3_MASK                                 (0x00E00000U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED3_SHIFT                                (0x00000015U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED3_RESETVAL                             (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED3_MAX                                  (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_DC_CKP3_MASK                                      (0x01000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP3_SHIFT                                     (0x00000018U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP3_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKP3_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP3_DATA_ACTIVE                    (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKP3_DATA_INACTIVE                  (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_CSP3_MASK                                      (0x02000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP3_SHIFT                                     (0x00000019U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP3_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CSP3_MAX                                       (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CSP3_ACTIVE_LOW                     (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CSP3_ACTIVE_HIGH                    (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_DC_CKPH3_MASK                                     (0x04000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH3_SHIFT                                    (0x0000001AU)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH3_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_CKPH3_MAX                                      (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_0_SHIFT_OUT_FALLING_EDGE  (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_0_SHIFT_OUT_RISING_EDGE   (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_1_SHIFT_OUT_FALLING_EDGE  (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_CKPH3_CKP_1_SHIFT_OUT_RISING_EDGE   (0x00000000U)

#define CSL_APP_CFG_QSPI_SPI_DC_DD3_MASK                                       (0x18000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD3_SHIFT                                      (0x0000001BU)
#define CSL_APP_CFG_QSPI_SPI_DC_DD3_RESETVAL                                   (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_DD3_MAX                                        (0x00000003U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_0              (0x00000000U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_1              (0x00000001U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_2              (0x00000002U)
#define CSL_QSPI_SPI_DC_REG_DD3_CS_TO_DATA_DELAY_3              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED4_MASK                                 (0xE0000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED4_SHIFT                                (0x0000001DU)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED4_RESETVAL                             (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DC_RESERVED4_MAX                                  (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_DC_RESETVAL                                       (0x00000000U)

/* SPI_CMD */

#define CSL_APP_CFG_QSPI_SPI_CMD_FLEN_MASK                                     (0x00000FFFU)
#define CSL_APP_CFG_QSPI_SPI_CMD_FLEN_SHIFT                                    (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_FLEN_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_FLEN_MAX                                      (0x00000FFFU)

#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED1_MASK                                (0x00003000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED1_SHIFT                               (0x0000000CU)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED1_MAX                                 (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_CMD_WIRQ_MASK                                     (0x00004000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_WIRQ_SHIFT                                    (0x0000000EU)
#define CSL_APP_CFG_QSPI_SPI_CMD_WIRQ_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_WIRQ_MAX                                      (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_CMD_FIRQ_MASK                                     (0x00008000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_FIRQ_SHIFT                                    (0x0000000FU)
#define CSL_APP_CFG_QSPI_SPI_CMD_FIRQ_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_FIRQ_MAX                                      (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_CMD_CMD_MASK                                      (0x00070000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_CMD_SHIFT                                     (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_CMD_CMD_RESETVAL                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_CMD_MAX                                       (0x00000007U)
#define CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_READ_SINGLE           (0x00000001U)
#define CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_WRITE_SINGLE          (0x00000002U)
#define CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_READ_DUAL             (0x00000003U)
#define CSL_QSPI_SPI_CMD_REG_CMD_THREE_PIN_READ_SINGLE          (0x00000005U)
#define CSL_QSPI_SPI_CMD_REG_CMD_THREE_PIN_WRITE_SINGLE         (0x00000006U)
#define CSL_QSPI_SPI_CMD_REG_CMD_SIX_PIN_READ_QUAD              (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_CMD_WLEN_MASK                                     (0x03F80000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_WLEN_SHIFT                                    (0x00000013U)
#define CSL_APP_CFG_QSPI_SPI_CMD_WLEN_RESETVAL                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_WLEN_MAX                                      (0x0000007FU)

#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED2_MASK                                (0x0C000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED2_SHIFT                               (0x0000001AU)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED2_RESETVAL                            (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED2_MAX                                 (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_CMD_CSNUM_MASK                                    (0x30000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_CSNUM_SHIFT                                   (0x0000001CU)
#define CSL_APP_CFG_QSPI_SPI_CMD_CSNUM_RESETVAL                                (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_CSNUM_MAX                                     (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED3_MASK                                (0xC0000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED3_SHIFT                               (0x0000001EU)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED3_RESETVAL                            (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_CMD_RESERVED3_MAX                                 (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_CMD_RESETVAL                                      (0x00000000U)

/* SPI_STATUS */

#define CSL_APP_CFG_QSPI_SPI_STATUS_BUSY_MASK                                  (0x00000001U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_BUSY_SHIFT                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_BUSY_RESETVAL                              (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_BUSY_MAX                                   (0x00000001U)
#define CSL_QSPI_SPI_STATUS_REG_BUSY_BUSY                       (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_STATUS_WC_MASK                                    (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_WC_SHIFT                                   (0x00000001U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_WC_RESETVAL                                (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_WC_MAX                                     (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_STATUS_FC_MASK                                    (0x00000004U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_FC_SHIFT                                   (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_FC_RESETVAL                                (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_FC_MAX                                     (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED1_MASK                             (0x0000FFF8U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED1_SHIFT                            (0x00000003U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED1_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED1_MAX                              (0x00001FFFU)

#define CSL_APP_CFG_QSPI_SPI_STATUS_WDCNT_MASK                                 (0x0FFF0000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_WDCNT_SHIFT                                (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_WDCNT_RESETVAL                             (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_WDCNT_MAX                                  (0x00000FFFU)

#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED2_MASK                             (0xF0000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED2_SHIFT                            (0x0000001CU)
#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED2_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_STATUS_RESERVED2_MAX                              (0x0000000FU)

#define CSL_APP_CFG_QSPI_SPI_STATUS_RESETVAL                                   (0x00000000U)

/* SPI_DATA */

#define CSL_APP_CFG_QSPI_SPI_DATA_DATA_MASK                                    (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_SPI_DATA_DATA_SHIFT                                   (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA_DATA_RESETVAL                                (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA_DATA_MAX                                     (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_SPI_DATA_RESETVAL                                     (0x00000000U)

/* SPI_SETUP0 */

#define CSL_APP_CFG_QSPI_SPI_SETUP0_RCMD_MASK                                  (0x000000FFU)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RCMD_SHIFT                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RCMD_RESETVAL                              (0x00000003U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_A_BYTES_MASK                           (0x00000300U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_A_BYTES_SHIFT                          (0x00000008U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_A_BYTES_RESETVAL                       (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_A_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BYTES_MASK                           (0x00000C00U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BYTES_SHIFT                          (0x0000000AU)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BYTES_RESETVAL                       (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_READ_TYPE_MASK                             (0x00003000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_READ_TYPE_SHIFT                            (0x0000000CU)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_READ_TYPE_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_READ_TYPE_MAX                              (0x00000003U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ           (0x00000000U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_DUAL_READ             (0x00000001U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ_TYPE      (0x00000002U)
#define CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_QUAD_READ             (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED1_MASK                             (0x0000C000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED1_SHIFT                            (0x0000000EU)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED1_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED1_MAX                              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_WCMD_MASK                                  (0x00FF0000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_WCMD_SHIFT                                 (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_WCMD_RESETVAL                              (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_WCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BITS_MASK                            (0x1F000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BITS_SHIFT                           (0x00000018U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BITS_RESETVAL                        (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_NUM_D_BITS_MAX                             (0x0000001FU)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED2_MASK                             (0xE0000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED2_SHIFT                            (0x0000001DU)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED2_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESERVED2_MAX                              (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_SETUP0_RESETVAL                                   (0x00020203U)

/* SPI_SETUP1 */

#define CSL_APP_CFG_QSPI_SPI_SETUP1_RCMD_MASK                                  (0x000000FFU)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RCMD_SHIFT                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RCMD_RESETVAL                              (0x00000003U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_A_BYTES_MASK                           (0x00000300U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_A_BYTES_SHIFT                          (0x00000008U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_A_BYTES_RESETVAL                       (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_A_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BYTES_MASK                           (0x00000C00U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BYTES_SHIFT                          (0x0000000AU)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BYTES_RESETVAL                       (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_READ_TYPE_MASK                             (0x00003000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_READ_TYPE_SHIFT                            (0x0000000CU)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_READ_TYPE_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_READ_TYPE_MAX                              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED1_MASK                             (0x0000C000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED1_SHIFT                            (0x0000000EU)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED1_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED1_MAX                              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_WCMD_MASK                                  (0x00FF0000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_WCMD_SHIFT                                 (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_WCMD_RESETVAL                              (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_WCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BITS_MASK                            (0x1F000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BITS_SHIFT                           (0x00000018U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BITS_RESETVAL                        (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_NUM_D_BITS_MAX                             (0x0000001FU)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED2_MASK                             (0xE0000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED2_SHIFT                            (0x0000001DU)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED2_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESERVED2_MAX                              (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_SETUP1_RESETVAL                                   (0x00020203U)

/* SPI_SETUP2 */

#define CSL_APP_CFG_QSPI_SPI_SETUP2_RCMD_MASK                                  (0x000000FFU)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RCMD_SHIFT                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RCMD_RESETVAL                              (0x00000003U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_A_BYTES_MASK                           (0x00000300U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_A_BYTES_SHIFT                          (0x00000008U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_A_BYTES_RESETVAL                       (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_A_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BYTES_MASK                           (0x00000C00U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BYTES_SHIFT                          (0x0000000AU)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BYTES_RESETVAL                       (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_READ_TYPE_MASK                             (0x00003000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_READ_TYPE_SHIFT                            (0x0000000CU)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_READ_TYPE_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_READ_TYPE_MAX                              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED1_MASK                             (0x0000C000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED1_SHIFT                            (0x0000000EU)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED1_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED1_MAX                              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_WCMD_MASK                                  (0x00FF0000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_WCMD_SHIFT                                 (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_WCMD_RESETVAL                              (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_WCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BITS_MASK                            (0x1F000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BITS_SHIFT                           (0x00000018U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BITS_RESETVAL                        (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_NUM_D_BITS_MAX                             (0x0000001FU)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED2_MASK                             (0xE0000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED2_SHIFT                            (0x0000001DU)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED2_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESERVED2_MAX                              (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_SETUP2_RESETVAL                                   (0x00020203U)

/* SPI_SETUP3 */

#define CSL_APP_CFG_QSPI_SPI_SETUP3_RCMD_MASK                                  (0x000000FFU)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RCMD_SHIFT                                 (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RCMD_RESETVAL                              (0x00000003U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_A_BYTES_MASK                           (0x00000300U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_A_BYTES_SHIFT                          (0x00000008U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_A_BYTES_RESETVAL                       (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_A_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BYTES_MASK                           (0x00000C00U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BYTES_SHIFT                          (0x0000000AU)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BYTES_RESETVAL                       (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BYTES_MAX                            (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_READ_TYPE_MASK                             (0x00003000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_READ_TYPE_SHIFT                            (0x0000000CU)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_READ_TYPE_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_READ_TYPE_MAX                              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED1_MASK                             (0x0000C000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED1_SHIFT                            (0x0000000EU)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED1_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED1_MAX                              (0x00000003U)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_WCMD_MASK                                  (0x00FF0000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_WCMD_SHIFT                                 (0x00000010U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_WCMD_RESETVAL                              (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_WCMD_MAX                                   (0x000000FFU)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BITS_MASK                            (0x1F000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BITS_SHIFT                           (0x00000018U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BITS_RESETVAL                        (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_NUM_D_BITS_MAX                             (0x0000001FU)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED2_MASK                             (0xE0000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED2_SHIFT                            (0x0000001DU)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED2_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESERVED2_MAX                              (0x00000007U)

#define CSL_APP_CFG_QSPI_SPI_SETUP3_RESETVAL                                   (0x00020203U)

/* SPI_SWITCH */

#define CSL_APP_CFG_QSPI_SPI_SWITCH_MMPT_S_MASK                                (0x00000001U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_MMPT_S_SHIFT                               (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_MMPT_S_RESETVAL                            (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_MMPT_S_MAX                                 (0x00000001U)
#define CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_CFG_PORT             (0x00000000U)
#define CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_MM_PORT              (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_SWITCH_MM_INT_EN_MASK                             (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_MM_INT_EN_SHIFT                            (0x00000001U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_MM_INT_EN_RESETVAL                         (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_MM_INT_EN_MAX                              (0x00000001U)
#define CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_MM_MODE_INTR_DISABLED  (0x00000000U)
#define CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_MM_MODE_INTR_ENABLED  (0x00000001U)

#define CSL_APP_CFG_QSPI_SPI_SWITCH_RESERVED_MASK                              (0xFFFFFFFCU)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_RESERVED_SHIFT                             (0x00000002U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_SWITCH_RESERVED_MAX                               (0x3FFFFFFFU)

#define CSL_APP_CFG_QSPI_SPI_SWITCH_RESETVAL                                   (0x00000000U)

/* SPI_DATA1 */

#define CSL_APP_CFG_QSPI_SPI_DATA1_DATA_MASK                                   (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_SPI_DATA1_DATA_SHIFT                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA1_DATA_RESETVAL                               (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA1_DATA_MAX                                    (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_SPI_DATA1_RESETVAL                                    (0x00000000U)

/* SPI_DATA2 */

#define CSL_APP_CFG_QSPI_SPI_DATA2_DATA_MASK                                   (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_SPI_DATA2_DATA_SHIFT                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA2_DATA_RESETVAL                               (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA2_DATA_MAX                                    (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_SPI_DATA2_RESETVAL                                    (0x00000000U)

/* SPI_DATA3 */

#define CSL_APP_CFG_QSPI_SPI_DATA3_DATA_MASK                                   (0xFFFFFFFFU)
#define CSL_APP_CFG_QSPI_SPI_DATA3_DATA_SHIFT                                  (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA3_DATA_RESETVAL                               (0x00000000U)
#define CSL_APP_CFG_QSPI_SPI_DATA3_DATA_MAX                                    (0xFFFFFFFFU)

#define CSL_APP_CFG_QSPI_SPI_DATA3_RESETVAL                                    (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
