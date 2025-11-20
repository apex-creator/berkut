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
 *  Name        : cslr_app_rcm.h
*/
#ifndef CSLR_APP_RCM_H_
#define CSLR_APP_RCM_H_

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
    volatile uint32_t APP_CPU_CLKCTL;
    volatile uint32_t APP_CPU_CLKSTAT;
    volatile uint32_t APP_CAN_CLKCTL;
    volatile uint32_t APP_CAN_CLKSTAT;
    volatile uint32_t APP_SPI_CLKCTL;
    volatile uint32_t APP_SPI_CLKSTAT;
    volatile uint32_t APP_SPI_BUSIF_CLKCTL;
    volatile uint32_t APP_SPI_BUSIF_CLKSTAT;
    volatile uint32_t APP_QSPI_CLKCTL;
    volatile uint32_t APP_QSPI_CLKSTAT;
    volatile uint32_t TOPSS_CLKCTL;
    volatile uint32_t TOPSS_CLKSTAT;
    volatile uint32_t APP_RTI_CLKCTL;
    volatile uint32_t APP_RTI_CLKSTAT;
    volatile uint32_t APP_WD_CLKCTL;
    volatile uint32_t APP_WD_CLKSTAT;
    volatile uint32_t APP_UART_0_CLKCTL;
    volatile uint32_t APP_UART_0_CLKSTAT;
    volatile uint32_t APP_UART_1_CLKCTL;
    volatile uint32_t APP_UART_1_CLKSTAT;
    volatile uint32_t APP_I2C_CLKCTL;
    volatile uint32_t APP_I2C_CLKSTAT;
    volatile uint32_t APP_LIN_CLKCTL;
    volatile uint32_t APP_LIN_CLKSTAT;
    volatile uint32_t RESERVED0;
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
    volatile uint32_t RESERVED3;
    volatile uint32_t IPCFGCLKGATE0;
    volatile uint32_t IPCFGCLKGATE1;
    volatile uint32_t IPCFGCLKGATE2;
    volatile uint32_t BLOCKRESET0;
    volatile uint32_t BLOCKRESET1;
    volatile uint32_t BLOCKRESET2;
    volatile uint32_t PLATFORM_SIGNATURE;
    volatile uint32_t POWERMODE;
    volatile uint32_t RST_WFICHECK;
    volatile uint32_t RST_ASSERTDLY;
    volatile uint32_t RST2ASSERTDLY;
    volatile uint32_t RST_FSM_TRIG;
    volatile uint32_t RST_CAUSE;
    volatile uint32_t RST_CAUSE_CLR;
    volatile uint32_t XTALCLK_CLK_GATE;
    volatile uint32_t XTALCLKX2_CLK_GATE;
    volatile uint32_t APLLDIV2_CLK_GATE;
    volatile uint32_t DFT_APPSS_LSTC_CLK_GATE;
    volatile uint32_t DFT_APPSS_LSTC_VBUSP_CLK_GATE;
    volatile uint32_t APP_ROM_CLOCK_GATE;
    volatile uint32_t APP_RAM1_CLOCK_GATE;
    volatile uint32_t APP_RAM2_CLOCK_GATE;
    volatile uint32_t APP_RAM3_CLOCK_GATE;
    volatile uint32_t CFG_XBARA_DYNAMIC_CG;
    volatile uint32_t CFG_TPTC1_DYNAMIC_CG;
    volatile uint32_t CFG_TPTC2_DYNAMIC_CG;
    volatile uint32_t CFG_XBARA_SET_DYNAMIC_CG;
    volatile uint32_t CFG_TPTC1_SET_DYNAMIC_CG;
    volatile uint32_t CFG_TPTC2_SET_DYNAMIC_CG;
    volatile uint32_t CM4_FORCE_HCLK_GATE;
    volatile uint32_t LIN_SCI_DIV;
    volatile uint32_t APP_LSTC_EN;
    volatile uint8_t  Resv_4104[3860];
    volatile uint32_t LOCK0_KICK0;
    volatile uint32_t LOCK0_KICK1;
    volatile uint32_t INTR_RAW_STATUS;
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR;
    volatile uint32_t INTR_ENABLE;
    volatile uint32_t INTR_ENABLE_CLEAR;
    volatile uint32_t EOI;
    volatile uint32_t FAULT_ADDRESS;
    volatile uint32_t FAULT_TYPE_STATUS;
    volatile uint32_t FAULT_ATTR_STATUS;
    volatile uint32_t FAULT_CLEAR;
} CSL_app_rcmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_APP_RCM_PID                                                        (0x00000000U)
#define CSL_APP_RCM_APP_CPU_CLKCTL                                             (0x00000004U)
#define CSL_APP_RCM_APP_CPU_CLKSTAT                                            (0x00000008U)
#define CSL_APP_RCM_APP_CAN_CLKCTL                                             (0x0000000CU)
#define CSL_APP_RCM_APP_CAN_CLKSTAT                                            (0x00000010U)
#define CSL_APP_RCM_APP_SPI_CLKCTL                                             (0x00000014U)
#define CSL_APP_RCM_APP_SPI_CLKSTAT                                            (0x00000018U)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKCTL                                       (0x0000001CU)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKSTAT                                      (0x00000020U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL                                            (0x00000024U)
#define CSL_APP_RCM_APP_QSPI_CLKSTAT                                           (0x00000028U)
#define CSL_APP_RCM_TOPSS_CLKCTL                                               (0x0000002CU)
#define CSL_APP_RCM_TOPSS_CLKSTAT                                              (0x00000030U)
#define CSL_APP_RCM_APP_RTI_CLKCTL                                             (0x00000034U)
#define CSL_APP_RCM_APP_RTI_CLKSTAT                                            (0x00000038U)
#define CSL_APP_RCM_APP_WD_CLKCTL                                              (0x0000003CU)
#define CSL_APP_RCM_APP_WD_CLKSTAT                                             (0x00000040U)
#define CSL_APP_RCM_APP_UART_0_CLKCTL                                          (0x00000044U)
#define CSL_APP_RCM_APP_UART_0_CLKSTAT                                         (0x00000048U)
#define CSL_APP_RCM_APP_UART_1_CLKCTL                                          (0x0000004CU)
#define CSL_APP_RCM_APP_UART_1_CLKSTAT                                         (0x00000050U)
#define CSL_APP_RCM_APP_I2C_CLKCTL                                             (0x00000054U)
#define CSL_APP_RCM_APP_I2C_CLKSTAT                                            (0x00000058U)
#define CSL_APP_RCM_APP_LIN_CLKCTL                                             (0x0000005CU)
#define CSL_APP_RCM_APP_LIN_CLKSTAT                                            (0x00000060U)
#define CSL_APP_RCM_RESERVED0                                                  (0x00000064U)
#define CSL_APP_RCM_RESERVED1                                                  (0x00000068U)
#define CSL_APP_RCM_RESERVED2                                                  (0x0000006CU)
#define CSL_APP_RCM_RESERVED3                                                  (0x00000070U)
#define CSL_APP_RCM_IPCFGCLKGATE0                                              (0x00000074U)
#define CSL_APP_RCM_IPCFGCLKGATE1                                              (0x00000078U)
#define CSL_APP_RCM_IPCFGCLKGATE2                                              (0x0000007CU)
#define CSL_APP_RCM_BLOCKRESET0                                                (0x00000080U)
#define CSL_APP_RCM_BLOCKRESET1                                                (0x00000084U)
#define CSL_APP_RCM_BLOCKRESET2                                                (0x00000088U)
#define CSL_APP_RCM_PLATFORM_SIGNATURE                                         (0x0000008CU)
#define CSL_APP_RCM_POWERMODE                                                  (0x00000090U)
#define CSL_APP_RCM_RST_WFICHECK                                               (0x00000094U)
#define CSL_APP_RCM_RST_ASSERTDLY                                              (0x00000098U)
#define CSL_APP_RCM_RST2ASSERTDLY                                              (0x0000009CU)
#define CSL_APP_RCM_RST_FSM_TRIG                                               (0x000000A0U)
#define CSL_APP_RCM_RST_CAUSE                                                  (0x000000A4U)
#define CSL_APP_RCM_RST_CAUSE_CLR                                              (0x000000A8U)
#define CSL_APP_RCM_XTALCLK_CLK_GATE                                           (0x000000ACU)
#define CSL_APP_RCM_XTALCLKX2_CLK_GATE                                         (0x000000B0U)
#define CSL_APP_RCM_APLLDIV2_CLK_GATE                                          (0x000000B4U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_CLK_GATE                                    (0x000000B8U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_VBUSP_CLK_GATE                              (0x000000BCU)
#define CSL_APP_RCM_APP_ROM_CLOCK_GATE                                         (0x000000C0U)
#define CSL_APP_RCM_APP_RAM1_CLOCK_GATE                                        (0x000000C4U)
#define CSL_APP_RCM_APP_RAM2_CLOCK_GATE                                        (0x000000C8U)
#define CSL_APP_RCM_APP_RAM3_CLOCK_GATE                                        (0x000000CCU)
#define CSL_APP_RCM_CFG_XBARA_DYNAMIC_CG                                       (0x000000D0U)
#define CSL_APP_RCM_CFG_TPTC1_DYNAMIC_CG                                       (0x000000D4U)
#define CSL_APP_RCM_CFG_TPTC2_DYNAMIC_CG                                       (0x000000D8U)
#define CSL_APP_RCM_CFG_XBARA_SET_DYNAMIC_CG                                   (0x000000DCU)
#define CSL_APP_RCM_CFG_TPTC1_SET_DYNAMIC_CG                                   (0x000000E0U)
#define CSL_APP_RCM_CFG_TPTC2_SET_DYNAMIC_CG                                   (0x000000E4U)
#define CSL_APP_RCM_CM4_FORCE_HCLK_GATE                                        (0x000000E8U)
#define CSL_APP_RCM_LIN_SCI_DIV                                                (0x000000ECU)
#define CSL_APP_RCM_APP_LSTC_EN                                                (0x000000F0U)
#define CSL_APP_RCM_LOCK0_KICK0                                                (0x00001008U)
#define CSL_APP_RCM_LOCK0_KICK1                                                (0x0000100CU)
#define CSL_APP_RCM_INTR_RAW_STATUS                                            (0x00001010U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR                                  (0x00001014U)
#define CSL_APP_RCM_INTR_ENABLE                                                (0x00001018U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR                                          (0x0000101CU)
#define CSL_APP_RCM_EOI                                                        (0x00001020U)
#define CSL_APP_RCM_FAULT_ADDRESS                                              (0x00001024U)
#define CSL_APP_RCM_FAULT_TYPE_STATUS                                          (0x00001028U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS                                          (0x0000102CU)
#define CSL_APP_RCM_FAULT_CLEAR                                                (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_APP_RCM_PID_PID_MINOR_MASK                                         (0x0000003FU)
#define CSL_APP_RCM_PID_PID_MINOR_SHIFT                                        (0x00000000U)
#define CSL_APP_RCM_PID_PID_MINOR_RESETVAL                                     (0x00000014U)
#define CSL_APP_RCM_PID_PID_MINOR_MAX                                          (0x0000003FU)

#define CSL_APP_RCM_PID_PID_CUSTOM_MASK                                        (0x000000C0U)
#define CSL_APP_RCM_PID_PID_CUSTOM_SHIFT                                       (0x00000006U)
#define CSL_APP_RCM_PID_PID_CUSTOM_RESETVAL                                    (0x00000000U)
#define CSL_APP_RCM_PID_PID_CUSTOM_MAX                                         (0x00000003U)

#define CSL_APP_RCM_PID_PID_MAJOR_MASK                                         (0x00000700U)
#define CSL_APP_RCM_PID_PID_MAJOR_SHIFT                                        (0x00000008U)
#define CSL_APP_RCM_PID_PID_MAJOR_RESETVAL                                     (0x00000002U)
#define CSL_APP_RCM_PID_PID_MAJOR_MAX                                          (0x00000007U)

#define CSL_APP_RCM_PID_PID_MISC_MASK                                          (0x0000F800U)
#define CSL_APP_RCM_PID_PID_MISC_SHIFT                                         (0x0000000BU)
#define CSL_APP_RCM_PID_PID_MISC_RESETVAL                                      (0x00000000U)
#define CSL_APP_RCM_PID_PID_MISC_MAX                                           (0x0000001FU)

#define CSL_APP_RCM_PID_PID_MSB16_MASK                                         (0xFFFF0000U)
#define CSL_APP_RCM_PID_PID_MSB16_SHIFT                                        (0x00000010U)
#define CSL_APP_RCM_PID_PID_MSB16_RESETVAL                                     (0x00006180U)
#define CSL_APP_RCM_PID_PID_MSB16_MAX                                          (0x0000FFFFU)

#define CSL_APP_RCM_PID_RESETVAL                                               (0x61800214U)

/* APP_CPU_CLKCTL */

#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_RESERVED_MASK                (0x0000000FU)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_RESERVED_SHIFT               (0x00000000U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_RESERVED_RESETVAL            (0x00000000U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_RESERVED_MAX                 (0x0000000FU)

#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_SRCSEL_MASK                  (0x0000FFF0U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_SRCSEL_SHIFT                 (0x00000004U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_SRCSEL_RESETVAL              (0x00000000U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_SRCSEL_MAX                   (0x00000FFFU)

#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_DIVR_MASK                    (0x0FFF0000U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_DIVR_SHIFT                   (0x00000010U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_DIVR_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_DIVR_MAX                     (0x00000FFFU)

#define CSL_APP_RCM_APP_CPU_CLKCTL_RESETVAL                                    (0x00000000U)

/* APP_CPU_CLKSTAT */

#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRDIVR_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRDIVR_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRDIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRDIVR_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRCLK_MASK               (0x00000FF0U)
#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRCLK_SHIFT              (0x00000004U)
#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRCLK_RESETVAL           (0x00000001U)
#define CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRCLK_MAX                (0x000000FFU)

#define CSL_APP_RCM_APP_CPU_CLKSTAT_RESETVAL                                   (0x00000010U)

/* APP_CAN_CLKCTL */

#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_GATE_MASK                    (0x0000000FU)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_GATE_SHIFT                   (0x00000000U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_GATE_RESETVAL                (0x00000007U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_GATE_MAX                     (0x0000000FU)

#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_SRCSEL_MASK                  (0x0000FFF0U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_SRCSEL_SHIFT                 (0x00000004U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_SRCSEL_RESETVAL              (0x00000000U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_SRCSEL_MAX                   (0x00000FFFU)

#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_DIVR_MASK                    (0x0FFF0000U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_DIVR_SHIFT                   (0x00000010U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_DIVR_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_DIVR_MAX                     (0x00000FFFU)

#define CSL_APP_RCM_APP_CAN_CLKCTL_RESETVAL                                    (0x00000007U)

/* APP_CAN_CLKSTAT */

#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRDIVR_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRDIVR_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRDIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRDIVR_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRCLK_MASK               (0x00000FF0U)
#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRCLK_SHIFT              (0x00000004U)
#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRCLK_RESETVAL           (0x00000000U)
#define CSL_APP_RCM_APP_CAN_CLKSTAT_APP_CAN_CLKSTAT_CURRCLK_MAX                (0x000000FFU)

#define CSL_APP_RCM_APP_CAN_CLKSTAT_RESETVAL                                   (0x00000000U)

/* APP_SPI_CLKCTL */

#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_GATE_MASK                    (0x0000000FU)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_GATE_SHIFT                   (0x00000000U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_GATE_RESETVAL                (0x00000007U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_GATE_MAX                     (0x0000000FU)

#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_SRCSEL_MASK                  (0x0000FFF0U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_SRCSEL_SHIFT                 (0x00000004U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_SRCSEL_RESETVAL              (0x00000000U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_SRCSEL_MAX                   (0x00000FFFU)

#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_DIVR_MASK                    (0x0FFF0000U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_DIVR_SHIFT                   (0x00000010U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_DIVR_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_DIVR_MAX                     (0x00000FFFU)

#define CSL_APP_RCM_APP_SPI_CLKCTL_RESETVAL                                    (0x00000007U)

/* APP_SPI_CLKSTAT */

#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRDIVR_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRDIVR_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRDIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRDIVR_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRCLK_MASK               (0x00000FF0U)
#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRCLK_SHIFT              (0x00000004U)
#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRCLK_RESETVAL           (0x00000000U)
#define CSL_APP_RCM_APP_SPI_CLKSTAT_APP_SPI_CLKSTAT_CURRCLK_MAX                (0x000000FFU)

#define CSL_APP_RCM_APP_SPI_CLKSTAT_RESETVAL                                   (0x00000000U)

/* APP_SPI_BUSIF_CLKCTL */

#define CSL_APP_RCM_APP_SPI_BUSIF_CLKCTL_APP_SPI_BUSIF_CLKCTL_DIVR_MASK        (0x00000FFFU)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKCTL_APP_SPI_BUSIF_CLKCTL_DIVR_SHIFT       (0x00000000U)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKCTL_APP_SPI_BUSIF_CLKCTL_DIVR_RESETVAL    (0x00000000U)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKCTL_APP_SPI_BUSIF_CLKCTL_DIVR_MAX         (0x00000FFFU)

#define CSL_APP_RCM_APP_SPI_BUSIF_CLKCTL_RESETVAL                              (0x00000000U)

/* APP_SPI_BUSIF_CLKSTAT */

#define CSL_APP_RCM_APP_SPI_BUSIF_CLKSTAT_APP_SPI_BUSIF_CLKSTAT_CURRDIVR_MASK  (0x0000000FU)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKSTAT_APP_SPI_BUSIF_CLKSTAT_CURRDIVR_SHIFT (0x00000000U)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKSTAT_APP_SPI_BUSIF_CLKSTAT_CURRDIVR_RESETVAL (0x00000000U)
#define CSL_APP_RCM_APP_SPI_BUSIF_CLKSTAT_APP_SPI_BUSIF_CLKSTAT_CURRDIVR_MAX   (0x0000000FU)

#define CSL_APP_RCM_APP_SPI_BUSIF_CLKSTAT_RESETVAL                             (0x00000000U)

/* APP_QSPI_CLKCTL */

#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_GATE_MASK                  (0x0000000FU)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_GATE_SHIFT                 (0x00000000U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_GATE_RESETVAL              (0x00000007U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_GATE_MAX                   (0x0000000FU)

#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_SRCSEL_MASK                (0x0000FFF0U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_SRCSEL_SHIFT               (0x00000004U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_SRCSEL_RESETVAL            (0x00000000U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_SRCSEL_MAX                 (0x00000FFFU)

#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_DIVR_MASK                  (0x0FFF0000U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_DIVR_SHIFT                 (0x00000010U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_DIVR_RESETVAL              (0x00000000U)
#define CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_DIVR_MAX                   (0x00000FFFU)

#define CSL_APP_RCM_APP_QSPI_CLKCTL_RESETVAL                                   (0x00000007U)

/* APP_QSPI_CLKSTAT */

#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRDIVR_MASK            (0x0000000FU)
#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRDIVR_SHIFT           (0x00000000U)
#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRDIVR_RESETVAL        (0x00000000U)
#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRDIVR_MAX             (0x0000000FU)

#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRCLK_MASK             (0x00000FF0U)
#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRCLK_SHIFT            (0x00000004U)
#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRCLK_RESETVAL         (0x00000000U)
#define CSL_APP_RCM_APP_QSPI_CLKSTAT_APP_QSPI_CLKSTAT_CURRCLK_MAX              (0x000000FFU)

#define CSL_APP_RCM_APP_QSPI_CLKSTAT_RESETVAL                                  (0x00000000U)

/* TOPSS_CLKCTL */

#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_GATE_MASK                        (0x0000000FU)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_GATE_SHIFT                       (0x00000000U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_GATE_RESETVAL                    (0x00000000U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_GATE_MAX                         (0x0000000FU)

#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_SRCSEL_MASK                      (0x0000FFF0U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_SRCSEL_SHIFT                     (0x00000004U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_SRCSEL_RESETVAL                  (0x00000000U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_SRCSEL_MAX                       (0x00000FFFU)

#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_DIVR_MASK                        (0x0FFF0000U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_DIVR_SHIFT                       (0x00000010U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_DIVR_RESETVAL                    (0x00000000U)
#define CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_DIVR_MAX                         (0x00000FFFU)

#define CSL_APP_RCM_TOPSS_CLKCTL_RESETVAL                                      (0x00000000U)

/* TOPSS_CLKSTAT */

#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRDIVR_MASK                  (0x0000000FU)
#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRDIVR_SHIFT                 (0x00000000U)
#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRDIVR_RESETVAL              (0x00000000U)
#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRDIVR_MAX                   (0x0000000FU)

#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRCLK_MASK                   (0x00000FF0U)
#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRCLK_SHIFT                  (0x00000004U)
#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRCLK_RESETVAL               (0x00000001U)
#define CSL_APP_RCM_TOPSS_CLKSTAT_TOPSS_CLKSTAT_CURRCLK_MAX                    (0x000000FFU)

#define CSL_APP_RCM_TOPSS_CLKSTAT_RESETVAL                                     (0x00000010U)

/* APP_RTI_CLKCTL */

#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_GATE_MASK                    (0x0000000FU)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_GATE_SHIFT                   (0x00000000U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_GATE_RESETVAL                (0x00000007U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_GATE_MAX                     (0x0000000FU)

#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_SRCSEL_MASK                  (0x0000FFF0U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_SRCSEL_SHIFT                 (0x00000004U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_SRCSEL_RESETVAL              (0x00000000U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_SRCSEL_MAX                   (0x00000FFFU)

#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_DIVR_MASK                    (0x0FFF0000U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_DIVR_SHIFT                   (0x00000010U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_DIVR_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_DIVR_MAX                     (0x00000FFFU)

#define CSL_APP_RCM_APP_RTI_CLKCTL_RESETVAL                                    (0x00000007U)

/* APP_RTI_CLKSTAT */

#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRDIVR_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRDIVR_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRDIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRDIVR_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRCLK_MASK               (0x00000FF0U)
#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRCLK_SHIFT              (0x00000004U)
#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRCLK_RESETVAL           (0x00000000U)
#define CSL_APP_RCM_APP_RTI_CLKSTAT_APP_RTI_CLKSTAT_CURRCLK_MAX                (0x000000FFU)

#define CSL_APP_RCM_APP_RTI_CLKSTAT_RESETVAL                                   (0x00000000U)

/* APP_WD_CLKCTL */

#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_GATE_MASK                      (0x0000000FU)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_GATE_SHIFT                     (0x00000000U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_GATE_RESETVAL                  (0x00000007U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_GATE_MAX                       (0x0000000FU)

#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_SRCSEL_MASK                    (0x0000FFF0U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_SRCSEL_SHIFT                   (0x00000004U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_SRCSEL_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_SRCSEL_MAX                     (0x00000FFFU)

#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_DIVR_MASK                      (0x0FFF0000U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_DIVR_SHIFT                     (0x00000010U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_DIVR_RESETVAL                  (0x00000000U)
#define CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_DIVR_MAX                       (0x00000FFFU)

#define CSL_APP_RCM_APP_WD_CLKCTL_RESETVAL                                     (0x00000007U)

/* APP_WD_CLKSTAT */

#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRDIVR_MASK                (0x0000000FU)
#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRDIVR_SHIFT               (0x00000000U)
#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRDIVR_RESETVAL            (0x00000000U)
#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRDIVR_MAX                 (0x0000000FU)

#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRCLK_MASK                 (0x00000FF0U)
#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRCLK_SHIFT                (0x00000004U)
#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRCLK_RESETVAL             (0x00000000U)
#define CSL_APP_RCM_APP_WD_CLKSTAT_APP_WD_CLKSTAT_CURRCLK_MAX                  (0x000000FFU)

#define CSL_APP_RCM_APP_WD_CLKSTAT_RESETVAL                                    (0x00000000U)

/* APP_UART_0_CLKCTL */

#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_GATE_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_GATE_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_GATE_RESETVAL          (0x00000007U)
#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_GATE_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_DIVR_MASK              (0x0FFF0000U)
#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_DIVR_SHIFT             (0x00000010U)
#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_DIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_DIVR_MAX               (0x00000FFFU)

#define CSL_APP_RCM_APP_UART_0_CLKCTL_RESETVAL                                 (0x00000007U)

/* APP_UART_0_CLKSTAT */

#define CSL_APP_RCM_APP_UART_0_CLKSTAT_APP_UART_0_CLKSTAT_CURRDIVR_MASK        (0x0000000FU)
#define CSL_APP_RCM_APP_UART_0_CLKSTAT_APP_UART_0_CLKSTAT_CURRDIVR_SHIFT       (0x00000000U)
#define CSL_APP_RCM_APP_UART_0_CLKSTAT_APP_UART_0_CLKSTAT_CURRDIVR_RESETVAL    (0x00000000U)
#define CSL_APP_RCM_APP_UART_0_CLKSTAT_APP_UART_0_CLKSTAT_CURRDIVR_MAX         (0x0000000FU)

#define CSL_APP_RCM_APP_UART_0_CLKSTAT_RESETVAL                                (0x00000000U)

/* APP_UART_1_CLKCTL */

#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_GATE_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_GATE_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_GATE_RESETVAL          (0x00000007U)
#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_GATE_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_DIVR_MASK              (0x0FFF0000U)
#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_DIVR_SHIFT             (0x00000010U)
#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_DIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_DIVR_MAX               (0x00000FFFU)

#define CSL_APP_RCM_APP_UART_1_CLKCTL_RESETVAL                                 (0x00000007U)

/* APP_UART_1_CLKSTAT */

#define CSL_APP_RCM_APP_UART_1_CLKSTAT_APP_UART_1_CLKSTAT_CURRDIVR_MASK        (0x0000000FU)
#define CSL_APP_RCM_APP_UART_1_CLKSTAT_APP_UART_1_CLKSTAT_CURRDIVR_SHIFT       (0x00000000U)
#define CSL_APP_RCM_APP_UART_1_CLKSTAT_APP_UART_1_CLKSTAT_CURRDIVR_RESETVAL    (0x00000000U)
#define CSL_APP_RCM_APP_UART_1_CLKSTAT_APP_UART_1_CLKSTAT_CURRDIVR_MAX         (0x0000000FU)

#define CSL_APP_RCM_APP_UART_1_CLKSTAT_RESETVAL                                (0x00000000U)

/* APP_I2C_CLKCTL */

#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_GATE_MASK                    (0x0000000FU)
#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_GATE_SHIFT                   (0x00000000U)
#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_GATE_RESETVAL                (0x00000007U)
#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_GATE_MAX                     (0x0000000FU)

#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_DIVR_MASK                    (0x0FFF0000U)
#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_DIVR_SHIFT                   (0x00000010U)
#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_DIVR_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_DIVR_MAX                     (0x00000FFFU)

#define CSL_APP_RCM_APP_I2C_CLKCTL_RESETVAL                                    (0x00000007U)

/* APP_I2C_CLKSTAT */

#define CSL_APP_RCM_APP_I2C_CLKSTAT_APP_I2C_CLKSTAT_CURRDIVR_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_I2C_CLKSTAT_APP_I2C_CLKSTAT_CURRDIVR_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_I2C_CLKSTAT_APP_I2C_CLKSTAT_CURRDIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_I2C_CLKSTAT_APP_I2C_CLKSTAT_CURRDIVR_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_I2C_CLKSTAT_RESETVAL                                   (0x00000000U)

/* APP_LIN_CLKCTL */

#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_GATE_MASK                    (0x0000000FU)
#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_GATE_SHIFT                   (0x00000000U)
#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_GATE_RESETVAL                (0x00000007U)
#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_GATE_MAX                     (0x0000000FU)

#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_DIVR_MASK                    (0x0FFF0000U)
#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_DIVR_SHIFT                   (0x00000010U)
#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_DIVR_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_DIVR_MAX                     (0x00000FFFU)

#define CSL_APP_RCM_APP_LIN_CLKCTL_RESETVAL                                    (0x00000007U)

/* APP_LIN_CLKSTAT */

#define CSL_APP_RCM_APP_LIN_CLKSTAT_APP_LIN_CLKSTAT_CURRDIVR_MASK              (0x0000000FU)
#define CSL_APP_RCM_APP_LIN_CLKSTAT_APP_LIN_CLKSTAT_CURRDIVR_SHIFT             (0x00000000U)
#define CSL_APP_RCM_APP_LIN_CLKSTAT_APP_LIN_CLKSTAT_CURRDIVR_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_APP_LIN_CLKSTAT_APP_LIN_CLKSTAT_CURRDIVR_MAX               (0x0000000FU)

#define CSL_APP_RCM_APP_LIN_CLKSTAT_RESETVAL                                   (0x00000000U)

/* RESERVED0 */

#define CSL_APP_RCM_RESERVED0_RESERVED0_RWRES_MASK                             (0x000000FFU)
#define CSL_APP_RCM_RESERVED0_RESERVED0_RWRES_SHIFT                            (0x00000000U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_RWRES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_RWRES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED0_RESERVED0_RORES_MASK                             (0x0000FF00U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_RORES_SHIFT                            (0x00000008U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_RORES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_RORES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED0_RESERVED0_WPHRES_MASK                            (0xFF000000U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_WPHRES_SHIFT                           (0x00000018U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_WPHRES_RESETVAL                        (0x00000000U)
#define CSL_APP_RCM_RESERVED0_RESERVED0_WPHRES_MAX                             (0x000000FFU)

#define CSL_APP_RCM_RESERVED0_RESETVAL                                         (0x00000000U)

/* RESERVED1 */

#define CSL_APP_RCM_RESERVED1_RESERVED1_RWRES_MASK                             (0x000000FFU)
#define CSL_APP_RCM_RESERVED1_RESERVED1_RWRES_SHIFT                            (0x00000000U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_RWRES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_RWRES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED1_RESERVED1_RORES_MASK                             (0x0000FF00U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_RORES_SHIFT                            (0x00000008U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_RORES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_RORES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED1_RESERVED1_WPHRES_MASK                            (0xFF000000U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_WPHRES_SHIFT                           (0x00000018U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_WPHRES_RESETVAL                        (0x00000000U)
#define CSL_APP_RCM_RESERVED1_RESERVED1_WPHRES_MAX                             (0x000000FFU)

#define CSL_APP_RCM_RESERVED1_RESETVAL                                         (0x00000000U)

/* RESERVED2 */

#define CSL_APP_RCM_RESERVED2_RESERVED2_RWRES_MASK                             (0x000000FFU)
#define CSL_APP_RCM_RESERVED2_RESERVED2_RWRES_SHIFT                            (0x00000000U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_RWRES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_RWRES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED2_RESERVED2_RORES_MASK                             (0x0000FF00U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_RORES_SHIFT                            (0x00000008U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_RORES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_RORES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED2_RESERVED2_WPHRES_MASK                            (0xFF000000U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_WPHRES_SHIFT                           (0x00000018U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_WPHRES_RESETVAL                        (0x00000000U)
#define CSL_APP_RCM_RESERVED2_RESERVED2_WPHRES_MAX                             (0x000000FFU)

#define CSL_APP_RCM_RESERVED2_RESETVAL                                         (0x00000000U)

/* RESERVED3 */

#define CSL_APP_RCM_RESERVED3_RESERVED3_RWRES_MASK                             (0x000000FFU)
#define CSL_APP_RCM_RESERVED3_RESERVED3_RWRES_SHIFT                            (0x00000000U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_RWRES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_RWRES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED3_RESERVED3_RORES_MASK                             (0x0000FF00U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_RORES_SHIFT                            (0x00000008U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_RORES_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_RORES_MAX                              (0x000000FFU)

#define CSL_APP_RCM_RESERVED3_RESERVED3_WPHRES_MASK                            (0xFF000000U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_WPHRES_SHIFT                           (0x00000018U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_WPHRES_RESETVAL                        (0x00000000U)
#define CSL_APP_RCM_RESERVED3_RESERVED3_WPHRES_MAX                             (0x000000FFU)

#define CSL_APP_RCM_RESERVED3_RESETVAL                                         (0x00000000U)

/* IPCFGCLKGATE0 */

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_XBARA_MASK                     (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_XBARA_SHIFT                    (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_XBARA_RESETVAL                 (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_XBARA_MAX                      (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI_MASK                  (0x00000038U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI_SHIFT                 (0x00000003U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI_RESETVAL              (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI_MAX                   (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A0_MASK                   (0x000001C0U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A0_SHIFT                  (0x00000006U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A0_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A0_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1_MASK                   (0x00000E00U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1_SHIFT                  (0x00000009U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPCC_A_MASK                    (0x00007000U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPCC_A_SHIFT                   (0x0000000CU)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPCC_A_RESETVAL                (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPCC_A_MAX                     (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_ESM_MASK                   (0x00038000U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_ESM_SHIFT                  (0x0000000FU)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_ESM_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_ESM_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_RTI_MASK                   (0x001C0000U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_RTI_SHIFT                  (0x00000012U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_RTI_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_RTI_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_WD_MASK                    (0x00E00000U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_WD_SHIFT                   (0x00000015U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_WD_RESETVAL                (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_WD_MAX                     (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC_MASK                   (0x07000000U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC_SHIFT                  (0x00000018U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C_MASK                   (0x38000000U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C_SHIFT                  (0x0000001BU)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE0_RESETVAL                                     (0x3FFFFFFFU)

/* IPCFGCLKGATE1 */

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_0_MASK                (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_0_SHIFT               (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_0_RESETVAL            (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_0_MAX                 (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1_MASK                (0x00000038U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1_SHIFT               (0x00000003U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1_RESETVAL            (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1_MAX                 (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0_MASK                 (0x000001C0U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0_SHIFT                (0x00000006U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0_RESETVAL             (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0_MAX                  (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1_MASK                 (0x00000E00U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1_SHIFT                (0x00000009U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1_RESETVAL             (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1_MAX                  (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CAN_MASK                   (0x00007000U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CAN_SHIFT                  (0x0000000CU)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CAN_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CAN_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_LIN_MASK                   (0x00038000U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_LIN_SHIFT                  (0x0000000FU)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_LIN_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_LIN_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM_MASK                   (0x001C0000U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM_SHIFT                  (0x00000012U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC_MASK                   (0x00E00000U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC_SHIFT                  (0x00000015U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC_RESETVAL               (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC_MAX                    (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CTRL_MASK                  (0x07000000U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CTRL_SHIFT                 (0x00000018U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CTRL_RESETVAL              (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CTRL_MAX                   (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_RES_MASK                       (0x38000000U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_RES_SHIFT                      (0x0000001BU)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_RES_RESETVAL                   (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_RES_MAX                        (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE1_RESETVAL                                     (0x3FFFFFFFU)

/* IPCFGCLKGATE2 */

#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_GIO_MASK                       (0x00000007U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_GIO_SHIFT                      (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_GIO_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_GIO_MAX                        (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_RS232_MASK                     (0x00000038U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_RS232_SHIFT                    (0x00000003U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_RS232_RESETVAL                 (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_RS232_MAX                      (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS_MASK                     (0x000001C0U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS_SHIFT                    (0x00000006U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS_RESETVAL                 (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS_MAX                      (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR5_MASK                      (0x00000E00U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR5_SHIFT                     (0x00000009U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR5_RESETVAL                  (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR5_MAX                       (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR6_MASK                      (0x00007000U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR6_SHIFT                     (0x0000000CU)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR6_RESETVAL                  (0x00000000U)
#define CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_PCR6_MAX                       (0x00000007U)

#define CSL_APP_RCM_IPCFGCLKGATE2_RESETVAL                                     (0x00000000U)

/* BLOCKRESET0 */

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_HWASS_MASK                         (0x00000007U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_HWASS_SHIFT                        (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_HWASS_RESETVAL                     (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_HWASS_MAX                          (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_QSPI_MASK                      (0x00000038U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_QSPI_SHIFT                     (0x00000003U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_QSPI_RESETVAL                  (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_QSPI_MAX                       (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A0_MASK                       (0x000001C0U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A0_SHIFT                      (0x00000006U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A0_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A0_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A1_MASK                       (0x00000E00U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A1_SHIFT                      (0x00000009U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A1_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPTC_A1_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPCC_A_MASK                        (0x00007000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPCC_A_SHIFT                       (0x0000000CU)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPCC_A_RESETVAL                    (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_TPCC_A_MAX                         (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_ESM_MASK                       (0x00038000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_ESM_SHIFT                      (0x0000000FU)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_ESM_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_ESM_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_RTI_MASK                       (0x001C0000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_RTI_SHIFT                      (0x00000012U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_RTI_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_RTI_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_WD_MASK                        (0x00E00000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_WD_SHIFT                       (0x00000015U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_WD_RESETVAL                    (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_WD_MAX                         (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_DCC_MASK                       (0x07000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_DCC_SHIFT                      (0x00000018U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_DCC_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_DCC_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_I2C_MASK                       (0x38000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_I2C_SHIFT                      (0x0000001BU)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_I2C_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_I2C_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET0_RESETVAL                                       (0x00000000U)

/* BLOCKRESET1 */

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_0_MASK                    (0x00000007U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_0_SHIFT                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_0_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_0_MAX                     (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_1_MASK                    (0x00000038U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_1_SHIFT                   (0x00000003U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_1_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_UART_1_MAX                     (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_0_MASK                     (0x000001C0U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_0_SHIFT                    (0x00000006U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_0_RESETVAL                 (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_0_MAX                      (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_1_MASK                     (0x00000E00U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_1_SHIFT                    (0x00000009U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_1_RESETVAL                 (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_SPI_1_MAX                      (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CAN_MASK                       (0x00007000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CAN_SHIFT                      (0x0000000CU)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CAN_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CAN_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_LIN_MASK                       (0x00038000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_LIN_SHIFT                      (0x0000000FU)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_LIN_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_LIN_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_PWM_MASK                       (0x001C0000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_PWM_SHIFT                      (0x00000012U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_PWM_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_PWM_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CRC_MASK                       (0x00E00000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CRC_SHIFT                      (0x00000015U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CRC_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CRC_MAX                        (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CTRL_MASK                      (0x07000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CTRL_SHIFT                     (0x00000018U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CTRL_RESETVAL                  (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_APP_CTRL_MAX                       (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_TOPSS_MASK                         (0x38000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_TOPSS_SHIFT                        (0x0000001BU)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_TOPSS_RESETVAL                     (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET1_BLOCKRESET1_TOPSS_MAX                          (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET1_RESETVAL                                       (0x00000000U)

/* BLOCKRESET2 */

#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RESERVED_MASK                      (0x00000007U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RESERVED_SHIFT                     (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RESERVED_RESETVAL                  (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RESERVED_MAX                       (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RS232_MASK                         (0x00000038U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RS232_SHIFT                        (0x00000003U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RS232_RESETVAL                     (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_RS232_MAX                          (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_FRC_MASK                           (0x000001C0U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_FRC_SHIFT                          (0x00000006U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_FRC_RESETVAL                       (0x00000000U)
#define CSL_APP_RCM_BLOCKRESET2_BLOCKRESET2_FRC_MAX                            (0x00000007U)

#define CSL_APP_RCM_BLOCKRESET2_RESETVAL                                       (0x00000000U)

/* PLATFORM_SIGNATURE */

#define CSL_APP_RCM_PLATFORM_SIGNATURE_PLATFORM_SIGNATURE_SIGNATURE_MASK       (0xFFFFFFFFU)
#define CSL_APP_RCM_PLATFORM_SIGNATURE_PLATFORM_SIGNATURE_SIGNATURE_SHIFT      (0x00000000U)
#define CSL_APP_RCM_PLATFORM_SIGNATURE_PLATFORM_SIGNATURE_SIGNATURE_RESETVAL   (0x7432150CU)
#define CSL_APP_RCM_PLATFORM_SIGNATURE_PLATFORM_SIGNATURE_SIGNATURE_MAX        (0xFFFFFFFFU)

#define CSL_APP_RCM_PLATFORM_SIGNATURE_RESETVAL                                (0x7432150CU)

/* POWERMODE */

#define CSL_APP_RCM_POWERMODE_POWERMODE_SLEEP_MASK                             (0x00000001U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_SLEEP_SHIFT                            (0x00000000U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_SLEEP_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_SLEEP_MAX                              (0x00000001U)

#define CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP_MASK                         (0x00000002U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP_SHIFT                        (0x00000001U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP_RESETVAL                     (0x00000000U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP_MAX                          (0x00000001U)

#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_SLEEP_STATUS_MASK                  (0x00000004U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_SLEEP_STATUS_SHIFT                 (0x00000002U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_SLEEP_STATUS_RESETVAL              (0x00000000U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_SLEEP_STATUS_MAX                   (0x00000001U)

#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_DEEPSLEEP_STATUS_MASK              (0x00000008U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_DEEPSLEEP_STATUS_SHIFT             (0x00000003U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_DEEPSLEEP_STATUS_RESETVAL          (0x00000000U)
#define CSL_APP_RCM_POWERMODE_POWERMODE_CM3_DEEPSLEEP_STATUS_MAX               (0x00000001U)

#define CSL_APP_RCM_POWERMODE_RESETVAL                                         (0x00000000U)

/* RST_WFICHECK */

#define CSL_APP_RCM_RST_WFICHECK_RST_WFICHECK_CPU_MASK                         (0x00000007U)
#define CSL_APP_RCM_RST_WFICHECK_RST_WFICHECK_CPU_SHIFT                        (0x00000000U)
#define CSL_APP_RCM_RST_WFICHECK_RST_WFICHECK_CPU_RESETVAL                     (0x00000000U)
#define CSL_APP_RCM_RST_WFICHECK_RST_WFICHECK_CPU_MAX                          (0x00000007U)

#define CSL_APP_RCM_RST_WFICHECK_RESETVAL                                      (0x00000000U)

/* RST_ASSERTDLY */

#define CSL_APP_RCM_RST_ASSERTDLY_RST_ASSERTDLY_COMMON_MASK                    (0x000000FFU)
#define CSL_APP_RCM_RST_ASSERTDLY_RST_ASSERTDLY_COMMON_SHIFT                   (0x00000000U)
#define CSL_APP_RCM_RST_ASSERTDLY_RST_ASSERTDLY_COMMON_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_RST_ASSERTDLY_RST_ASSERTDLY_COMMON_MAX                     (0x000000FFU)

#define CSL_APP_RCM_RST_ASSERTDLY_RESETVAL                                     (0x00000000U)

/* RST2ASSERTDLY */

#define CSL_APP_RCM_RST2ASSERTDLY_RST2ASSERTDLY_CPU_MASK                       (0x0000FF00U)
#define CSL_APP_RCM_RST2ASSERTDLY_RST2ASSERTDLY_CPU_SHIFT                      (0x00000008U)
#define CSL_APP_RCM_RST2ASSERTDLY_RST2ASSERTDLY_CPU_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_RST2ASSERTDLY_RST2ASSERTDLY_CPU_MAX                        (0x000000FFU)

#define CSL_APP_RCM_RST2ASSERTDLY_RESETVAL                                     (0x00000000U)

/* RST_FSM_TRIG */

#define CSL_APP_RCM_RST_FSM_TRIG_RST_FSM_TRIG_CPU_MASK                         (0x00000007U)
#define CSL_APP_RCM_RST_FSM_TRIG_RST_FSM_TRIG_CPU_SHIFT                        (0x00000000U)
#define CSL_APP_RCM_RST_FSM_TRIG_RST_FSM_TRIG_CPU_RESETVAL                     (0x00000000U)
#define CSL_APP_RCM_RST_FSM_TRIG_RST_FSM_TRIG_CPU_MAX                          (0x00000007U)

#define CSL_APP_RCM_RST_FSM_TRIG_RESETVAL                                      (0x00000000U)

/* RST_CAUSE */

#define CSL_APP_RCM_RST_CAUSE_RST_CAUSE_COMMON_MASK                            (0x000000FFU)
#define CSL_APP_RCM_RST_CAUSE_RST_CAUSE_COMMON_SHIFT                           (0x00000000U)
#define CSL_APP_RCM_RST_CAUSE_RST_CAUSE_COMMON_RESETVAL                        (0x00000003U)
#define CSL_APP_RCM_RST_CAUSE_RST_CAUSE_COMMON_MAX                             (0x000000FFU)

#define CSL_APP_RCM_RST_CAUSE_RESETVAL                                         (0x00000003U)

/* RST_CAUSE_CLR */

#define CSL_APP_RCM_RST_CAUSE_CLR_RST_CAUSE_CLR_CPU_MASK                       (0x00000007U)
#define CSL_APP_RCM_RST_CAUSE_CLR_RST_CAUSE_CLR_CPU_SHIFT                      (0x00000000U)
#define CSL_APP_RCM_RST_CAUSE_CLR_RST_CAUSE_CLR_CPU_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_RST_CAUSE_CLR_RST_CAUSE_CLR_CPU_MAX                        (0x00000007U)

#define CSL_APP_RCM_RST_CAUSE_CLR_RESETVAL                                     (0x00000000U)

/* XTALCLK_CLK_GATE */

#define CSL_APP_RCM_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_MASK    (0x00000007U)
#define CSL_APP_RCM_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_SHIFT   (0x00000000U)
#define CSL_APP_RCM_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_APP_RCM_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_XTALCLK_CLK_GATE_MAX     (0x00000007U)

#define CSL_APP_RCM_XTALCLK_CLK_GATE_RESETVAL                                  (0x00000000U)

/* XTALCLKX2_CLK_GATE */

#define CSL_APP_RCM_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_MASK (0x00000007U)
#define CSL_APP_RCM_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_SHIFT (0x00000000U)
#define CSL_APP_RCM_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_APP_RCM_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_XTALCLKX2_CLK_GATE_MAX (0x00000007U)

#define CSL_APP_RCM_XTALCLKX2_CLK_GATE_RESETVAL                                (0x00000000U)

/* APLLDIV2_CLK_GATE */

#define CSL_APP_RCM_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_MASK (0x00000007U)
#define CSL_APP_RCM_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_SHIFT (0x00000000U)
#define CSL_APP_RCM_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_APP_RCM_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_APLLDIV2_CLK_GATE_MAX  (0x00000007U)

#define CSL_APP_RCM_APLLDIV2_CLK_GATE_RESETVAL                                 (0x00000000U)

/* DFT_APPSS_LSTC_CLK_GATE */

#define CSL_APP_RCM_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_MASK (0x00000007U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_SHIFT (0x00000000U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_RESETVAL (0x00000007U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_DFT_APPSS_LSTC_CLK_GATE_MAX (0x00000007U)

#define CSL_APP_RCM_DFT_APPSS_LSTC_CLK_GATE_RESETVAL                           (0x00000007U)

/* DFT_APPSS_LSTC_VBUSP_CLK_GATE */

#define CSL_APP_RCM_DFT_APPSS_LSTC_VBUSP_CLK_GATE_DFT_APPSS_LSTC_VBUSP_CLK_GATE_ENABLE_MASK (0x00000007U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_VBUSP_CLK_GATE_DFT_APPSS_LSTC_VBUSP_CLK_GATE_ENABLE_SHIFT (0x00000000U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_VBUSP_CLK_GATE_DFT_APPSS_LSTC_VBUSP_CLK_GATE_ENABLE_RESETVAL (0x00000007U)
#define CSL_APP_RCM_DFT_APPSS_LSTC_VBUSP_CLK_GATE_DFT_APPSS_LSTC_VBUSP_CLK_GATE_ENABLE_MAX (0x00000007U)

#define CSL_APP_RCM_DFT_APPSS_LSTC_VBUSP_CLK_GATE_RESETVAL                     (0x00000007U)

/* APP_ROM_CLOCK_GATE */

#define CSL_APP_RCM_APP_ROM_CLOCK_GATE_APP_ROM_CLOCK_GATE_ENABLE_MASK          (0x00000007U)
#define CSL_APP_RCM_APP_ROM_CLOCK_GATE_APP_ROM_CLOCK_GATE_ENABLE_SHIFT         (0x00000000U)
#define CSL_APP_RCM_APP_ROM_CLOCK_GATE_APP_ROM_CLOCK_GATE_ENABLE_RESETVAL      (0x00000000U)
#define CSL_APP_RCM_APP_ROM_CLOCK_GATE_APP_ROM_CLOCK_GATE_ENABLE_MAX           (0x00000007U)

#define CSL_APP_RCM_APP_ROM_CLOCK_GATE_RESETVAL                                (0x00000000U)

/* APP_RAM1_CLOCK_GATE */

#define CSL_APP_RCM_APP_RAM1_CLOCK_GATE_APP_RAM1_CLOCK_GATE_ENABLE_MASK        (0x00000007U)
#define CSL_APP_RCM_APP_RAM1_CLOCK_GATE_APP_RAM1_CLOCK_GATE_ENABLE_SHIFT       (0x00000000U)
#define CSL_APP_RCM_APP_RAM1_CLOCK_GATE_APP_RAM1_CLOCK_GATE_ENABLE_RESETVAL    (0x00000000U)
#define CSL_APP_RCM_APP_RAM1_CLOCK_GATE_APP_RAM1_CLOCK_GATE_ENABLE_MAX         (0x00000007U)

#define CSL_APP_RCM_APP_RAM1_CLOCK_GATE_RESETVAL                               (0x00000000U)

/* APP_RAM2_CLOCK_GATE */

#define CSL_APP_RCM_APP_RAM2_CLOCK_GATE_APP_RAM2_CLOCK_GATE_ENABLE_MASK        (0x00000007U)
#define CSL_APP_RCM_APP_RAM2_CLOCK_GATE_APP_RAM2_CLOCK_GATE_ENABLE_SHIFT       (0x00000000U)
#define CSL_APP_RCM_APP_RAM2_CLOCK_GATE_APP_RAM2_CLOCK_GATE_ENABLE_RESETVAL    (0x00000000U)
#define CSL_APP_RCM_APP_RAM2_CLOCK_GATE_APP_RAM2_CLOCK_GATE_ENABLE_MAX         (0x00000007U)

#define CSL_APP_RCM_APP_RAM2_CLOCK_GATE_RESETVAL                               (0x00000000U)

/* APP_RAM3_CLOCK_GATE */

#define CSL_APP_RCM_APP_RAM3_CLOCK_GATE_APP_RAM3_CLOCK_GATE_ENABLE_MASK        (0x00000007U)
#define CSL_APP_RCM_APP_RAM3_CLOCK_GATE_APP_RAM3_CLOCK_GATE_ENABLE_SHIFT       (0x00000000U)
#define CSL_APP_RCM_APP_RAM3_CLOCK_GATE_APP_RAM3_CLOCK_GATE_ENABLE_RESETVAL    (0x00000000U)
#define CSL_APP_RCM_APP_RAM3_CLOCK_GATE_APP_RAM3_CLOCK_GATE_ENABLE_MAX         (0x00000007U)

#define CSL_APP_RCM_APP_RAM3_CLOCK_GATE_RESETVAL                               (0x00000000U)

/* CFG_XBARA_DYNAMIC_CG */

#define CSL_APP_RCM_CFG_XBARA_DYNAMIC_CG_CFG_XBARA_DYNAMIC_CG_ENABLE_MASK      (0x00000007U)
#define CSL_APP_RCM_CFG_XBARA_DYNAMIC_CG_CFG_XBARA_DYNAMIC_CG_ENABLE_SHIFT     (0x00000000U)
#define CSL_APP_RCM_CFG_XBARA_DYNAMIC_CG_CFG_XBARA_DYNAMIC_CG_ENABLE_RESETVAL  (0x00000000U)
#define CSL_APP_RCM_CFG_XBARA_DYNAMIC_CG_CFG_XBARA_DYNAMIC_CG_ENABLE_MAX       (0x00000007U)

#define CSL_APP_RCM_CFG_XBARA_DYNAMIC_CG_RESETVAL                              (0x00000000U)

/* CFG_TPTC1_DYNAMIC_CG */

#define CSL_APP_RCM_CFG_TPTC1_DYNAMIC_CG_CFG_TPTC1_DYNAMIC_CG_ENABLE_MASK      (0x00000007U)
#define CSL_APP_RCM_CFG_TPTC1_DYNAMIC_CG_CFG_TPTC1_DYNAMIC_CG_ENABLE_SHIFT     (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC1_DYNAMIC_CG_CFG_TPTC1_DYNAMIC_CG_ENABLE_RESETVAL  (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC1_DYNAMIC_CG_CFG_TPTC1_DYNAMIC_CG_ENABLE_MAX       (0x00000007U)

#define CSL_APP_RCM_CFG_TPTC1_DYNAMIC_CG_RESETVAL                              (0x00000000U)

/* CFG_TPTC2_DYNAMIC_CG */

#define CSL_APP_RCM_CFG_TPTC2_DYNAMIC_CG_CFG_TPTC2_DYNAMIC_CG_ENABLE_MASK      (0x00000007U)
#define CSL_APP_RCM_CFG_TPTC2_DYNAMIC_CG_CFG_TPTC2_DYNAMIC_CG_ENABLE_SHIFT     (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC2_DYNAMIC_CG_CFG_TPTC2_DYNAMIC_CG_ENABLE_RESETVAL  (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC2_DYNAMIC_CG_CFG_TPTC2_DYNAMIC_CG_ENABLE_MAX       (0x00000007U)

#define CSL_APP_RCM_CFG_TPTC2_DYNAMIC_CG_RESETVAL                              (0x00000000U)

/* CFG_XBARA_SET_DYNAMIC_CG */

#define CSL_APP_RCM_CFG_XBARA_SET_DYNAMIC_CG_CFG_XBARA_SET_DYNAMIC_CG_SET_MASK (0x00000001U)
#define CSL_APP_RCM_CFG_XBARA_SET_DYNAMIC_CG_CFG_XBARA_SET_DYNAMIC_CG_SET_SHIFT (0x00000000U)
#define CSL_APP_RCM_CFG_XBARA_SET_DYNAMIC_CG_CFG_XBARA_SET_DYNAMIC_CG_SET_RESETVAL (0x00000000U)
#define CSL_APP_RCM_CFG_XBARA_SET_DYNAMIC_CG_CFG_XBARA_SET_DYNAMIC_CG_SET_MAX  (0x00000001U)

#define CSL_APP_RCM_CFG_XBARA_SET_DYNAMIC_CG_RESETVAL                          (0x00000000U)

/* CFG_TPTC1_SET_DYNAMIC_CG */

#define CSL_APP_RCM_CFG_TPTC1_SET_DYNAMIC_CG_CFG_TPTC1_SET_DYNAMIC_CG_SET_MASK (0x00000001U)
#define CSL_APP_RCM_CFG_TPTC1_SET_DYNAMIC_CG_CFG_TPTC1_SET_DYNAMIC_CG_SET_SHIFT (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC1_SET_DYNAMIC_CG_CFG_TPTC1_SET_DYNAMIC_CG_SET_RESETVAL (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC1_SET_DYNAMIC_CG_CFG_TPTC1_SET_DYNAMIC_CG_SET_MAX  (0x00000001U)

#define CSL_APP_RCM_CFG_TPTC1_SET_DYNAMIC_CG_RESETVAL                          (0x00000000U)

/* CFG_TPTC2_SET_DYNAMIC_CG */

#define CSL_APP_RCM_CFG_TPTC2_SET_DYNAMIC_CG_CFG_TPTC2_SET_DYNAMIC_CG_SET_MASK (0x00000001U)
#define CSL_APP_RCM_CFG_TPTC2_SET_DYNAMIC_CG_CFG_TPTC2_SET_DYNAMIC_CG_SET_SHIFT (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC2_SET_DYNAMIC_CG_CFG_TPTC2_SET_DYNAMIC_CG_SET_RESETVAL (0x00000000U)
#define CSL_APP_RCM_CFG_TPTC2_SET_DYNAMIC_CG_CFG_TPTC2_SET_DYNAMIC_CG_SET_MAX  (0x00000001U)

#define CSL_APP_RCM_CFG_TPTC2_SET_DYNAMIC_CG_RESETVAL                          (0x00000000U)

/* CM4_FORCE_HCLK_GATE */

#define CSL_APP_RCM_CM4_FORCE_HCLK_GATE_CM4_FORCE_HCLK_GATE_ENABLE_MASK        (0x00000007U)
#define CSL_APP_RCM_CM4_FORCE_HCLK_GATE_CM4_FORCE_HCLK_GATE_ENABLE_SHIFT       (0x00000000U)
#define CSL_APP_RCM_CM4_FORCE_HCLK_GATE_CM4_FORCE_HCLK_GATE_ENABLE_RESETVAL    (0x00000000U)
#define CSL_APP_RCM_CM4_FORCE_HCLK_GATE_CM4_FORCE_HCLK_GATE_ENABLE_MAX         (0x00000007U)

#define CSL_APP_RCM_CM4_FORCE_HCLK_GATE_RESETVAL                               (0x00000000U)

/* LIN_SCI_DIV */

#define CSL_APP_RCM_LIN_SCI_DIV_LIN_SCI_DIV_VAL_MASK                           (0x000000FFU)
#define CSL_APP_RCM_LIN_SCI_DIV_LIN_SCI_DIV_VAL_SHIFT                          (0x00000000U)
#define CSL_APP_RCM_LIN_SCI_DIV_LIN_SCI_DIV_VAL_RESETVAL                       (0x00000000U)
#define CSL_APP_RCM_LIN_SCI_DIV_LIN_SCI_DIV_VAL_MAX                            (0x000000FFU)

#define CSL_APP_RCM_LIN_SCI_DIV_RESETVAL                                       (0x00000000U)

/* APP_LSTC_EN */

#define CSL_APP_RCM_APP_LSTC_EN_APP_LSTC_EN_ENABLE_MASK                        (0x00000001U)
#define CSL_APP_RCM_APP_LSTC_EN_APP_LSTC_EN_ENABLE_SHIFT                       (0x00000000U)
#define CSL_APP_RCM_APP_LSTC_EN_APP_LSTC_EN_ENABLE_RESETVAL                    (0x00000001U)
#define CSL_APP_RCM_APP_LSTC_EN_APP_LSTC_EN_ENABLE_MAX                         (0x00000001U)

#define CSL_APP_RCM_APP_LSTC_EN_RESETVAL                                       (0x00000001U)

/* LOCK0_KICK0 */

#define CSL_APP_RCM_LOCK0_KICK0_LOCK0_KICK0_MASK                               (0xFFFFFFFFU)
#define CSL_APP_RCM_LOCK0_KICK0_LOCK0_KICK0_SHIFT                              (0x00000000U)
#define CSL_APP_RCM_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                           (0x00000000U)
#define CSL_APP_RCM_LOCK0_KICK0_LOCK0_KICK0_MAX                                (0xFFFFFFFFU)

#define CSL_APP_RCM_LOCK0_KICK0_RESETVAL                                       (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_APP_RCM_LOCK0_KICK1_LOCK0_KICK1_MASK                               (0xFFFFFFFFU)
#define CSL_APP_RCM_LOCK0_KICK1_LOCK0_KICK1_SHIFT                              (0x00000000U)
#define CSL_APP_RCM_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                           (0x00000000U)
#define CSL_APP_RCM_LOCK0_KICK1_LOCK0_KICK1_MAX                                (0xFFFFFFFFU)

#define CSL_APP_RCM_LOCK0_KICK1_RESETVAL                                       (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_APP_RCM_INTR_RAW_STATUS_PROT_ERR_MASK                              (0x00000001U)
#define CSL_APP_RCM_INTR_RAW_STATUS_PROT_ERR_SHIFT                             (0x00000000U)
#define CSL_APP_RCM_INTR_RAW_STATUS_PROT_ERR_RESETVAL                          (0x00000000U)
#define CSL_APP_RCM_INTR_RAW_STATUS_PROT_ERR_MAX                               (0x00000001U)

#define CSL_APP_RCM_INTR_RAW_STATUS_ADDR_ERR_MASK                              (0x00000002U)
#define CSL_APP_RCM_INTR_RAW_STATUS_ADDR_ERR_SHIFT                             (0x00000001U)
#define CSL_APP_RCM_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                          (0x00000000U)
#define CSL_APP_RCM_INTR_RAW_STATUS_ADDR_ERR_MAX                               (0x00000001U)

#define CSL_APP_RCM_INTR_RAW_STATUS_KICK_ERR_MASK                              (0x00000004U)
#define CSL_APP_RCM_INTR_RAW_STATUS_KICK_ERR_SHIFT                             (0x00000002U)
#define CSL_APP_RCM_INTR_RAW_STATUS_KICK_ERR_RESETVAL                          (0x00000000U)
#define CSL_APP_RCM_INTR_RAW_STATUS_KICK_ERR_MAX                               (0x00000001U)

#define CSL_APP_RCM_INTR_RAW_STATUS_PROXY_ERR_MASK                             (0x00000008U)
#define CSL_APP_RCM_INTR_RAW_STATUS_PROXY_ERR_SHIFT                            (0x00000003U)
#define CSL_APP_RCM_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                         (0x00000000U)
#define CSL_APP_RCM_INTR_RAW_STATUS_PROXY_ERR_MAX                              (0x00000001U)

#define CSL_APP_RCM_INTR_RAW_STATUS_RESETVAL                                   (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK            (0x00000001U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT           (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL        (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX             (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK            (0x00000002U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT           (0x00000001U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL        (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX             (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK            (0x00000004U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT           (0x00000002U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL        (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX             (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK           (0x00000008U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT          (0x00000003U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL       (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX            (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLED_STATUS_CLEAR_RESETVAL                         (0x00000000U)

/* INTR_ENABLE */

#define CSL_APP_RCM_INTR_ENABLE_PROT_ERR_EN_MASK                               (0x00000001U)
#define CSL_APP_RCM_INTR_ENABLE_PROT_ERR_EN_SHIFT                              (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_PROT_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_PROT_ERR_EN_MAX                                (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_ADDR_ERR_EN_MASK                               (0x00000002U)
#define CSL_APP_RCM_INTR_ENABLE_ADDR_ERR_EN_SHIFT                              (0x00000001U)
#define CSL_APP_RCM_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_ADDR_ERR_EN_MAX                                (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_KICK_ERR_EN_MASK                               (0x00000004U)
#define CSL_APP_RCM_INTR_ENABLE_KICK_ERR_EN_SHIFT                              (0x00000002U)
#define CSL_APP_RCM_INTR_ENABLE_KICK_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_KICK_ERR_EN_MAX                                (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_PROXY_ERR_EN_MASK                              (0x00000008U)
#define CSL_APP_RCM_INTR_ENABLE_PROXY_ERR_EN_SHIFT                             (0x00000003U)
#define CSL_APP_RCM_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_PROXY_ERR_EN_MAX                               (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_RESETVAL                                       (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                     (0x00000001U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                    (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                      (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                     (0x00000002U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                    (0x00000001U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                      (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                     (0x00000004U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                    (0x00000002U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                      (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                    (0x00000008U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                   (0x00000003U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_APP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_APP_RCM_INTR_ENABLE_CLEAR_RESETVAL                                 (0x00000000U)

/* EOI */

#define CSL_APP_RCM_EOI_EOI_VECTOR_MASK                                        (0x000000FFU)
#define CSL_APP_RCM_EOI_EOI_VECTOR_SHIFT                                       (0x00000000U)
#define CSL_APP_RCM_EOI_EOI_VECTOR_RESETVAL                                    (0x00000000U)
#define CSL_APP_RCM_EOI_EOI_VECTOR_MAX                                         (0x000000FFU)

#define CSL_APP_RCM_EOI_RESETVAL                                               (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_APP_RCM_FAULT_ADDRESS_FAULT_ADDR_MASK                              (0xFFFFFFFFU)
#define CSL_APP_RCM_FAULT_ADDRESS_FAULT_ADDR_SHIFT                             (0x00000000U)
#define CSL_APP_RCM_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                          (0x00000000U)
#define CSL_APP_RCM_FAULT_ADDRESS_FAULT_ADDR_MAX                               (0xFFFFFFFFU)

#define CSL_APP_RCM_FAULT_ADDRESS_RESETVAL                                     (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                          (0x0000003FU)
#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                         (0x00000000U)
#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                      (0x00000000U)
#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                           (0x0000003FU)

#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_NS_MASK                            (0x00000040U)
#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                           (0x00000006U)
#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                        (0x00000000U)
#define CSL_APP_RCM_FAULT_TYPE_STATUS_FAULT_NS_MAX                             (0x00000001U)

#define CSL_APP_RCM_FAULT_TYPE_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                        (0x000000FFU)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                       (0x00000000U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                    (0x00000000U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                         (0x000000FFU)

#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                       (0x000FFF00U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                      (0x00000008U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                   (0x00000000U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                        (0x00000FFFU)

#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_XID_MASK                           (0xFFF00000U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                          (0x00000014U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                       (0x00000000U)
#define CSL_APP_RCM_FAULT_ATTR_STATUS_FAULT_XID_MAX                            (0x00000FFFU)

#define CSL_APP_RCM_FAULT_ATTR_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_CLEAR */

#define CSL_APP_RCM_FAULT_CLEAR_FAULT_CLR_MASK                                 (0x00000001U)
#define CSL_APP_RCM_FAULT_CLEAR_FAULT_CLR_SHIFT                                (0x00000000U)
#define CSL_APP_RCM_FAULT_CLEAR_FAULT_CLR_RESETVAL                             (0x00000000U)
#define CSL_APP_RCM_FAULT_CLEAR_FAULT_CLR_MAX                                  (0x00000001U)

#define CSL_APP_RCM_FAULT_CLEAR_RESETVAL                                       (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
