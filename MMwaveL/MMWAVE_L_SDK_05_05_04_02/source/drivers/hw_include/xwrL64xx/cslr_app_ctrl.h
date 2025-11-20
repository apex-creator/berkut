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
 *  Name        : cslr_app_ctrl.h
*/
#ifndef CSLR_APP_CTRL_H_
#define CSLR_APP_CTRL_H_

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
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t HW_REG2;
    volatile uint32_t HW_REG3;
    volatile uint32_t HW_REG4;
    volatile uint32_t HW_REG5;
    volatile uint32_t HW_REG6;
    volatile uint32_t HW_REG7;
    volatile uint32_t APPSS_SW_INT;
    volatile uint32_t APPSS_IPC_RFS;
    volatile uint32_t APPSS_CAPEVNT_SEL;
    volatile uint32_t APPSS_DMA_REQ_SEL;
    volatile uint32_t APPSS_DMA1_REQ_SEL;
    volatile uint32_t APPSS_IRQ_REQ_SEL;
    volatile uint32_t APPSS_SPI_TRIG_SRC;
    volatile uint32_t APPSS_RAM1A_MEM_INIT;
    volatile uint32_t APPSS_RAM1A_MEM_INIT_DONE;
    volatile uint32_t APPSS_RAM1A_MEM_INIT_STATUS;
    volatile uint32_t APPSS_RAM2A_MEM_INIT;
    volatile uint32_t APPSS_RAM2A_MEM_INIT_DONE;
    volatile uint32_t APPSS_RAM2A_MEM_INIT_STATUS;
    volatile uint32_t APPSS_RAM3A_MEM_INIT;
    volatile uint32_t APPSS_RAM3A_MEM_INIT_DONE;
    volatile uint32_t APPSS_RAM3A_MEM_INIT_STATUS;
    volatile uint32_t HWASS_SHRD_RAM0_MEM_INIT;
    volatile uint32_t HWASS_SHRD_RAM0_MEM_INIT_DONE;
    volatile uint32_t HWASS_SHRD_RAM0_MEM_INIT_STATUS;
    volatile uint32_t HWASS_SHRD_RAM1_MEM_INIT;
    volatile uint32_t HWASS_SHRD_RAM1_MEM_INIT_DONE;
    volatile uint32_t HWASS_SHRD_RAM1_MEM_INIT_STATUS;
    volatile uint32_t APPSS_TPCC_MEMINIT_START;
    volatile uint32_t APPSS_TPCC_MEMINIT_DONE;
    volatile uint32_t APPSS_TPCC_MEMINIT_STATUS;
    volatile uint32_t APPSS_SPIA_CFG;
    volatile uint32_t APPSS_SPIB_CFG;
    volatile uint32_t APPSS_EPWM_CFG;
    volatile uint32_t RESERVED;
    volatile uint32_t APPSS_MCAN_FE_AND_LIN_INTR_SEL;
    volatile uint32_t APPSS_MCANA_INT_CLR;
    volatile uint32_t APPSS_MCANA_INT_MASK;
    volatile uint32_t APPSS_MCANA_INT_STAT;
    volatile uint32_t APPSS_CM4_GLOBAL_CONFIG;
    volatile uint32_t RESERVED1;
    volatile uint32_t APPSS_CM4_ROM_ECLIPSE;
    volatile uint32_t APPSS_CM4_STATUS_REG;
    volatile uint32_t APPSS_AHB_CTRL;
    volatile uint32_t ESM_GATING0;
    volatile uint32_t ESM_GATING1;
    volatile uint32_t ESM_GATING2;
    volatile uint32_t ESM_GATING3;
    volatile uint32_t ESM_GATING4;
    volatile uint32_t ESM_GATING5;
    volatile uint32_t ESM_GATING6;
    volatile uint32_t ESM_GATING7;
    volatile uint32_t APPSS_CM4_HALT;
    volatile uint32_t APPSS_CM4_EVENT;
    volatile uint32_t SPIA_IO_CFG;
    volatile uint32_t SPIB_IO_CFG;
    volatile uint32_t SPI_HOST_IRQ;
    volatile uint32_t TPTC_DBS_CONFIG;
    volatile uint32_t TPCC_PARITY_CTRL;
    volatile uint32_t TPCC_PARITY_STATUS;
    volatile uint32_t APPSS_DBG_ACK_CTL0;
    volatile uint32_t DEBUGSS_CSETB_FLUSH;
    volatile uint32_t CPSW_CONTROL;
    volatile uint32_t APPSS_ERRAGG_MASK0;
    volatile uint32_t APPSS_ERRAGG_STATUS0;
    volatile uint8_t  Resv_400[128];
    volatile uint32_t APPSS_TPCC_A_ERRAGG_MASK;
    volatile uint32_t APPSS_TPCC_A_ERRAGG_STATUS;
    volatile uint32_t APPSS_TPCC_A_ERRAGG_STATUS_RAW;
    volatile uint8_t  Resv_532[120];
    volatile uint32_t APPSS_TPCC_A_INTAGG_MASK;
    volatile uint32_t APPSS_TPCC_A_INTAGG_STATUS;
    volatile uint32_t APPSS_TPCC_A_INTAGG_STATUS_RAW;
    volatile uint8_t  Resv_628[84];
    volatile uint32_t APPSS_TPCC_B_ERRAGG_MASK;
    volatile uint32_t APPSS_TPCC_B_ERRAGG_STATUS;
    volatile uint32_t APPSS_TPCC_B_ERRAGG_STATUS_RAW;
    volatile uint8_t  Resv_748[108];
    volatile uint32_t APPSS_TPCC_B_INTAGG_MASK;
    volatile uint32_t APPSS_TPCC_B_INTAGG_STATUS;
    volatile uint32_t APPSS_TPCC_B_INTAGG_STATUS_RAW;
    volatile uint32_t APPSS_MPU_ERRAGG_MASK;
    volatile uint32_t APPSS_MPU_ERRAGG_STATUS;
    volatile uint32_t APPSS_MPU_ERRAGG_STATUS_RAW;
    volatile uint32_t APPSS_QSPI_CONFIG;
    volatile uint32_t APPSS_CTI_TRIG_SEL;
    volatile uint32_t APPSS_DBGSS_CTI_TRIG_SEL;
    volatile uint32_t APPSS_BOOT_INFO_REG0;
    volatile uint32_t APPSS_BOOT_INFO_REG1;
    volatile uint32_t APPSS_BOOT_INFO_REG2;
    volatile uint32_t APPSS_BOOT_INFO_REG3;
    volatile uint32_t APPSS_BOOT_INFO_REG4;
    volatile uint32_t APPSS_BOOT_INFO_REG5;
    volatile uint32_t APPSS_BOOT_INFO_REG6;
    volatile uint32_t APPSS_BOOT_INFO_REG7;
    volatile uint32_t APPSS_TPTC_ECCAGGR_CLK_CNTRL;
    volatile uint32_t APPSS_TPTC_BOUNDARY_CFG;
    volatile uint32_t APPSS_TPTC_XID_REORDER_CFG;
    volatile uint32_t HW_SYNC_FE_CTRL;
    volatile uint32_t HW_SPARE_REG1;
    volatile uint32_t HW_SPARE_REG2;
    volatile uint32_t HW_SPARE_REG3;
    volatile uint32_t NERROR_MASK;
    volatile uint32_t HW_SPARE_RW0;
    volatile uint32_t HW_SPARE_RW1;
    volatile uint32_t HW_SPARE_RW2;
    volatile uint32_t HW_SPARE_RW3;
    volatile uint32_t HW_SPARE_RW4;
    volatile uint32_t HW_SPARE_RW5;
    volatile uint32_t HW_SPARE_RO0;
    volatile uint32_t HW_SPARE_RO1;
    volatile uint32_t HW_SPARE_RO2;
    volatile uint32_t HW_SPARE_RO3;
    volatile uint32_t HW_SPARE_REC;
    volatile uint32_t APP_CTRL;
    volatile uint32_t WIC_CTRL;
    volatile uint32_t WIC_STAT_CLR;
    volatile uint32_t WIC_STAT;
    volatile uint32_t WICEN;
    volatile uint32_t FORCEFCLKACTIVE;
    volatile uint32_t FECSS_CLK_GATE;
    volatile uint32_t APPSS_SHARED_MEM_CLK_GATE;
    volatile uint32_t APPSS_MEM_INIT_SLICE_SEL;
    volatile uint32_t APPSS_QSPI_CHAR_EXT_CLK_EN;
    volatile uint32_t APPSS_QSPI_EXT_CLK_EN;
    volatile uint32_t SPI1_SMART_IDLE;
    volatile uint32_t SPI2_SMART_IDLE;
    volatile uint32_t CAN_SMART_IDLE;
    volatile uint32_t LIN_SMART_IDLE;
    volatile uint32_t HWASS_CLK_GATE;
    volatile uint32_t CFG_TIMEOUT_PCR3;
    volatile uint32_t RESERVED0;
    volatile uint32_t APPSS_ERRAGG_MASK1;
    volatile uint32_t APPSS_ERRAGG_STATUS1;
    volatile uint32_t FORCEHCLKACTIVE;
    volatile uint32_t APPSS_RAM1_OWRITE_ERR;
    volatile uint32_t APPSS_RAM1_OWRITE_ERR_ADDR;
    volatile uint32_t APPSS_RAM2_OWRITE_ERR;
    volatile uint32_t APPSS_RAM2_OWRITE_ERR_ADDR;
    volatile uint32_t APPSS_RAM3_OWRITE_ERR;
    volatile uint32_t APPSS_RAM3_OWRITE_ERR_ADDR;
    volatile uint32_t APPSS_SHRD_RAM_OWRITE_ERR;
    volatile uint32_t APPSS_SHRD_RAM_OWRITE_ERR_ADDR;
    volatile uint32_t APPSS_OWRITE_ERR_AGGR;
    volatile uint32_t HW_SPARE_RW6;
    volatile uint32_t HW_SPARE_RW7;
    volatile uint32_t HW_SPARE_RW8;
    volatile uint32_t HW_SPARE_RW9;
    volatile uint8_t  Resv_4104[3076];
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
} CSL_app_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_APP_CTRL_PID                                                       (0x00000000U)
#define CSL_APP_CTRL_HW_REG0                                                   (0x00000004U)
#define CSL_APP_CTRL_HW_REG1                                                   (0x00000008U)
#define CSL_APP_CTRL_HW_REG2                                                   (0x0000000CU)
#define CSL_APP_CTRL_HW_REG3                                                   (0x00000010U)
#define CSL_APP_CTRL_HW_REG4                                                   (0x00000014U)
#define CSL_APP_CTRL_HW_REG5                                                   (0x00000018U)
#define CSL_APP_CTRL_HW_REG6                                                   (0x0000001CU)
#define CSL_APP_CTRL_HW_REG7                                                   (0x00000020U)
#define CSL_APP_CTRL_APPSS_SW_INT                                              (0x00000024U)
#define CSL_APP_CTRL_APPSS_IPC_RFS                                             (0x00000028U)
#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL                                         (0x0000002CU)
#define CSL_APP_CTRL_APPSS_DMA_REQ_SEL                                         (0x00000030U)
#define CSL_APP_CTRL_APPSS_DMA1_REQ_SEL                                        (0x00000034U)
#define CSL_APP_CTRL_APPSS_IRQ_REQ_SEL                                         (0x00000038U)
#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC                                        (0x0000003CU)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT                                      (0x00000040U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_DONE                                 (0x00000044U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_STATUS                               (0x00000048U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT                                      (0x0000004CU)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_DONE                                 (0x00000050U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_STATUS                               (0x00000054U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT                                      (0x00000058U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_DONE                                 (0x0000005CU)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_STATUS                               (0x00000060U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT                                  (0x00000064U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE                             (0x00000068U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_STATUS                           (0x0000006CU)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT                                  (0x00000070U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE                             (0x00000074U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_STATUS                           (0x00000078U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START                                  (0x0000007CU)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE                                   (0x00000080U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS                                 (0x00000084U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG                                            (0x00000088U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG                                            (0x0000008CU)
#define CSL_APP_CTRL_APPSS_EPWM_CFG                                            (0x00000090U)
#define CSL_APP_CTRL_RESERVED                                                  (0x00000094U)
#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL                            (0x00000098U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_CLR                                       (0x0000009CU)
#define CSL_APP_CTRL_APPSS_MCANA_INT_MASK                                      (0x000000A0U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_STAT                                      (0x000000A4U)
#define CSL_APP_CTRL_APPSS_CM4_GLOBAL_CONFIG                                   (0x000000A8U)
#define CSL_APP_CTRL_RESERVED1                                                 (0x000000ACU)
#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE                                     (0x000000B0U)
#define CSL_APP_CTRL_APPSS_CM4_STATUS_REG                                      (0x000000B4U)
#define CSL_APP_CTRL_APPSS_AHB_CTRL                                            (0x000000B8U)
#define CSL_APP_CTRL_ESM_GATING0                                               (0x000000BCU)
#define CSL_APP_CTRL_ESM_GATING1                                               (0x000000C0U)
#define CSL_APP_CTRL_ESM_GATING2                                               (0x000000C4U)
#define CSL_APP_CTRL_ESM_GATING3                                               (0x000000C8U)
#define CSL_APP_CTRL_ESM_GATING4                                               (0x000000CCU)
#define CSL_APP_CTRL_ESM_GATING5                                               (0x000000D0U)
#define CSL_APP_CTRL_ESM_GATING6                                               (0x000000D4U)
#define CSL_APP_CTRL_ESM_GATING7                                               (0x000000D8U)
#define CSL_APP_CTRL_APPSS_CM4_HALT                                            (0x000000DCU)
#define CSL_APP_CTRL_APPSS_CM4_EVENT                                           (0x000000E0U)
#define CSL_APP_CTRL_SPIA_IO_CFG                                               (0x000000E4U)
#define CSL_APP_CTRL_SPIB_IO_CFG                                               (0x000000E8U)
#define CSL_APP_CTRL_SPI_HOST_IRQ                                              (0x000000ECU)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG                                           (0x000000F0U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL                                          (0x000000F4U)
#define CSL_APP_CTRL_TPCC_PARITY_STATUS                                        (0x000000F8U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0                                        (0x000000FCU)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH                                       (0x00000100U)
#define CSL_APP_CTRL_CPSW_CONTROL                                              (0x00000104U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0                                        (0x00000108U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0                                      (0x0000010CU)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK                                  (0x00000190U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS                                (0x00000194U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW                            (0x00000198U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK                                  (0x00000214U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS                                (0x00000218U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW                            (0x0000021CU)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK                                  (0x00000274U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS                                (0x00000278U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW                            (0x0000027CU)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK                                  (0x000002ECU)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS                                (0x000002F0U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW                            (0x000002F4U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK                                     (0x000002F8U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS                                   (0x000002FCU)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW                               (0x00000300U)
#define CSL_APP_CTRL_APPSS_QSPI_CONFIG                                         (0x00000304U)
#define CSL_APP_CTRL_APPSS_CTI_TRIG_SEL                                        (0x00000308U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL                                  (0x0000030CU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG0                                      (0x00000310U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG1                                      (0x00000314U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG2                                      (0x00000318U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG3                                      (0x0000031CU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG4                                      (0x00000320U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG5                                      (0x00000324U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG6                                      (0x00000328U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG7                                      (0x0000032CU)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL                              (0x00000330U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG                                   (0x00000334U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG                                (0x00000338U)
#define CSL_APP_CTRL_HW_SYNC_FE_CTRL                                           (0x0000033CU)
#define CSL_APP_CTRL_HW_SPARE_REG1                                             (0x00000340U)
#define CSL_APP_CTRL_HW_SPARE_REG2                                             (0x00000344U)
#define CSL_APP_CTRL_HW_SPARE_REG3                                             (0x00000348U)
#define CSL_APP_CTRL_NERROR_MASK                                               (0x0000034CU)
#define CSL_APP_CTRL_HW_SPARE_RW0                                              (0x00000350U)
#define CSL_APP_CTRL_HW_SPARE_RW1                                              (0x00000354U)
#define CSL_APP_CTRL_HW_SPARE_RW2                                              (0x00000358U)
#define CSL_APP_CTRL_HW_SPARE_RW3                                              (0x0000035CU)
#define CSL_APP_CTRL_HW_SPARE_RW4                                              (0x00000360U)
#define CSL_APP_CTRL_HW_SPARE_RW5                                              (0x00000364U)
#define CSL_APP_CTRL_HW_SPARE_RO0                                              (0x00000368U)
#define CSL_APP_CTRL_HW_SPARE_RO1                                              (0x0000036CU)
#define CSL_APP_CTRL_HW_SPARE_RO2                                              (0x00000370U)
#define CSL_APP_CTRL_HW_SPARE_RO3                                              (0x00000374U)
#define CSL_APP_CTRL_HW_SPARE_REC                                              (0x00000378U)
#define CSL_APP_CTRL_APP_CTRL                                                  (0x0000037CU)
#define CSL_APP_CTRL_WIC_CTRL                                                  (0x00000380U)
#define CSL_APP_CTRL_WIC_STAT_CLR                                              (0x00000384U)
#define CSL_APP_CTRL_WIC_STAT                                                  (0x00000388U)
#define CSL_APP_CTRL_WICEN                                                     (0x0000038CU)
#define CSL_APP_CTRL_FORCEFCLKACTIVE                                           (0x00000390U)
#define CSL_APP_CTRL_FECSS_CLK_GATE                                            (0x00000394U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE                                 (0x00000398U)
#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL                                  (0x0000039CU)
#define CSL_APP_CTRL_APPSS_QSPI_CHAR_EXT_CLK_EN                                (0x000003A0U)
#define CSL_APP_CTRL_APPSS_QSPI_EXT_CLK_EN                                     (0x000003A4U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE                                           (0x000003A8U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE                                           (0x000003ACU)
#define CSL_APP_CTRL_CAN_SMART_IDLE                                            (0x000003B0U)
#define CSL_APP_CTRL_LIN_SMART_IDLE                                            (0x000003B4U)
#define CSL_APP_CTRL_HWASS_CLK_GATE                                            (0x000003B8U)
#define CSL_APP_CTRL_CFG_TIMEOUT_PCR3                                          (0x000003BCU)
#define CSL_APP_CTRL_RESERVED0                                                 (0x000003C0U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1                                        (0x000003C4U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1                                      (0x000003C8U)
#define CSL_APP_CTRL_FORCEHCLKACTIVE                                           (0x000003CCU)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR                                     (0x000003D0U)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_ADDR                                (0x000003D4U)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR                                     (0x000003D8U)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_ADDR                                (0x000003DCU)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR                                     (0x000003E0U)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_ADDR                                (0x000003E4U)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR                                 (0x000003E8U)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_ADDR                            (0x000003ECU)
#define CSL_APP_CTRL_APPSS_OWRITE_ERR_AGGR                                     (0x000003F0U)
#define CSL_APP_CTRL_HW_SPARE_RW6                                              (0x000003F4U)
#define CSL_APP_CTRL_HW_SPARE_RW7                                              (0x000003F8U)
#define CSL_APP_CTRL_HW_SPARE_RW8                                              (0x000003FCU)
#define CSL_APP_CTRL_HW_SPARE_RW9                                              (0x00000400U)
#define CSL_APP_CTRL_LOCK0_KICK0                                               (0x00001008U)
#define CSL_APP_CTRL_LOCK0_KICK1                                               (0x0000100CU)
#define CSL_APP_CTRL_INTR_RAW_STATUS                                           (0x00001010U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR                                 (0x00001014U)
#define CSL_APP_CTRL_INTR_ENABLE                                               (0x00001018U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR                                         (0x0000101CU)
#define CSL_APP_CTRL_EOI                                                       (0x00001020U)
#define CSL_APP_CTRL_FAULT_ADDRESS                                             (0x00001024U)
#define CSL_APP_CTRL_FAULT_TYPE_STATUS                                         (0x00001028U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS                                         (0x0000102CU)
#define CSL_APP_CTRL_FAULT_CLEAR                                               (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_APP_CTRL_PID_PID_MINOR_MASK                                        (0x0000003FU)
#define CSL_APP_CTRL_PID_PID_MINOR_SHIFT                                       (0x00000000U)
#define CSL_APP_CTRL_PID_PID_MINOR_RESETVAL                                    (0x00000014U)
#define CSL_APP_CTRL_PID_PID_MINOR_MAX                                         (0x0000003FU)

#define CSL_APP_CTRL_PID_PID_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_APP_CTRL_PID_PID_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_APP_CTRL_PID_PID_CUSTOM_RESETVAL                                   (0x00000000U)
#define CSL_APP_CTRL_PID_PID_CUSTOM_MAX                                        (0x00000003U)

#define CSL_APP_CTRL_PID_PID_MAJOR_MASK                                        (0x00000700U)
#define CSL_APP_CTRL_PID_PID_MAJOR_SHIFT                                       (0x00000008U)
#define CSL_APP_CTRL_PID_PID_MAJOR_RESETVAL                                    (0x00000002U)
#define CSL_APP_CTRL_PID_PID_MAJOR_MAX                                         (0x00000007U)

#define CSL_APP_CTRL_PID_PID_MISC_MASK                                         (0x0000F800U)
#define CSL_APP_CTRL_PID_PID_MISC_SHIFT                                        (0x0000000BU)
#define CSL_APP_CTRL_PID_PID_MISC_RESETVAL                                     (0x00000000U)
#define CSL_APP_CTRL_PID_PID_MISC_MAX                                          (0x0000001FU)

#define CSL_APP_CTRL_PID_PID_MSB16_MASK                                        (0xFFFF0000U)
#define CSL_APP_CTRL_PID_PID_MSB16_SHIFT                                       (0x00000010U)
#define CSL_APP_CTRL_PID_PID_MSB16_RESETVAL                                    (0x00006180U)
#define CSL_APP_CTRL_PID_PID_MSB16_MAX                                         (0x0000FFFFU)

#define CSL_APP_CTRL_PID_RESETVAL                                              (0x61800214U)

/* HW_REG0 */

#define CSL_APP_CTRL_HW_REG0_HW_REG0_HWREG0_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG0_HW_REG0_HWREG0_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG0_HW_REG0_HWREG0_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG0_HW_REG0_HWREG0_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG0_RESETVAL                                          (0x00000000U)

/* HW_REG1 */

#define CSL_APP_CTRL_HW_REG1_HW_REG1_HWREG1_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG1_HW_REG1_HWREG1_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG1_HW_REG1_HWREG1_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG1_HW_REG1_HWREG1_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG1_RESETVAL                                          (0x00000000U)

/* HW_REG2 */

#define CSL_APP_CTRL_HW_REG2_HW_REG2_HWREG2_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG2_HW_REG2_HWREG2_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG2_HW_REG2_HWREG2_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG2_HW_REG2_HWREG2_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG2_RESETVAL                                          (0x00000000U)

/* HW_REG3 */

#define CSL_APP_CTRL_HW_REG3_HW_REG3_HWREG3_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG3_HW_REG3_HWREG3_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG3_HW_REG3_HWREG3_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG3_HW_REG3_HWREG3_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG3_RESETVAL                                          (0x00000000U)

/* HW_REG4 */

#define CSL_APP_CTRL_HW_REG4_HW_REG4_HWREG4_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG4_HW_REG4_HWREG4_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG4_HW_REG4_HWREG4_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG4_HW_REG4_HWREG4_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG4_RESETVAL                                          (0x00000000U)

/* HW_REG5 */

#define CSL_APP_CTRL_HW_REG5_HW_REG5_HWREG5_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG5_HW_REG5_HWREG5_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG5_HW_REG5_HWREG5_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG5_HW_REG5_HWREG5_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG5_RESETVAL                                          (0x00000000U)

/* HW_REG6 */

#define CSL_APP_CTRL_HW_REG6_HW_REG6_HWREG6_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG6_HW_REG6_HWREG6_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG6_HW_REG6_HWREG6_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG6_HW_REG6_HWREG6_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG6_RESETVAL                                          (0x00000000U)

/* HW_REG7 */

#define CSL_APP_CTRL_HW_REG7_HW_REG7_HWREG7_MASK                               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_REG7_HW_REG7_HWREG7_SHIFT                              (0x00000000U)
#define CSL_APP_CTRL_HW_REG7_HW_REG7_HWREG7_RESETVAL                           (0x00000000U)
#define CSL_APP_CTRL_HW_REG7_HW_REG7_HWREG7_MAX                                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_REG7_RESETVAL                                          (0x00000000U)

/* APPSS_SW_INT */

#define CSL_APP_CTRL_APPSS_SW_INT_APPSS_SW_INT_PULSE_MASK                      (0x0000000FU)
#define CSL_APP_CTRL_APPSS_SW_INT_APPSS_SW_INT_PULSE_SHIFT                     (0x00000000U)
#define CSL_APP_CTRL_APPSS_SW_INT_APPSS_SW_INT_PULSE_RESETVAL                  (0x00000000U)
#define CSL_APP_CTRL_APPSS_SW_INT_APPSS_SW_INT_PULSE_MAX                       (0x0000000FU)

#define CSL_APP_CTRL_APPSS_SW_INT_RESETVAL                                     (0x00000000U)

/* APPSS_IPC_RFS */

#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_HOST_INTR_MASK                (0x0000000FU)
#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_HOST_INTR_SHIFT               (0x00000000U)
#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_HOST_INTR_RESETVAL            (0x00000000U)
#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_HOST_INTR_MAX                 (0x0000000FU)

#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_COMMAND_MASK                  (0xFFFFFFF0U)
#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_COMMAND_SHIFT                 (0x00000004U)
#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_COMMAND_RESETVAL              (0x00000000U)
#define CSL_APP_CTRL_APPSS_IPC_RFS_APPSS_IPC_RFS_COMMAND_MAX                   (0x0FFFFFFFU)

#define CSL_APP_CTRL_APPSS_IPC_RFS_RESETVAL                                    (0x00000000U)

/* APPSS_CAPEVNT_SEL */

#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC0_MASK             (0x00000FFFU)
#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC0_SHIFT            (0x00000000U)
#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC0_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC0_MAX              (0x00000FFFU)

#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC1_MASK             (0x00FFF000U)
#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC1_SHIFT            (0x0000000CU)
#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC1_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_APPSS_CAPEVNT_SEL_SRC1_MAX              (0x00000FFFU)

#define CSL_APP_CTRL_APPSS_CAPEVNT_SEL_RESETVAL                                (0x00000000U)

/* APPSS_DMA_REQ_SEL */

#define CSL_APP_CTRL_APPSS_DMA_REQ_SEL_APPSS_DMA_REQ_SEL_SELECT_MASK           (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_DMA_REQ_SEL_APPSS_DMA_REQ_SEL_SELECT_SHIFT          (0x00000000U)
#define CSL_APP_CTRL_APPSS_DMA_REQ_SEL_APPSS_DMA_REQ_SEL_SELECT_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_DMA_REQ_SEL_APPSS_DMA_REQ_SEL_SELECT_MAX            (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_DMA_REQ_SEL_RESETVAL                                (0x00000000U)

/* APPSS_DMA1_REQ_SEL */

#define CSL_APP_CTRL_APPSS_DMA1_REQ_SEL_APPSS_DMA1_REQ_SEL_SELECT_MASK         (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_DMA1_REQ_SEL_APPSS_DMA1_REQ_SEL_SELECT_SHIFT        (0x00000000U)
#define CSL_APP_CTRL_APPSS_DMA1_REQ_SEL_APPSS_DMA1_REQ_SEL_SELECT_RESETVAL     (0x00000000U)
#define CSL_APP_CTRL_APPSS_DMA1_REQ_SEL_APPSS_DMA1_REQ_SEL_SELECT_MAX          (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_DMA1_REQ_SEL_RESETVAL                               (0x00000000U)

/* APPSS_IRQ_REQ_SEL */

#define CSL_APP_CTRL_APPSS_IRQ_REQ_SEL_APPSS_IRQ_REQ_SEL_SELECT_MASK           (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_IRQ_REQ_SEL_APPSS_IRQ_REQ_SEL_SELECT_SHIFT          (0x00000000U)
#define CSL_APP_CTRL_APPSS_IRQ_REQ_SEL_APPSS_IRQ_REQ_SEL_SELECT_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_IRQ_REQ_SEL_APPSS_IRQ_REQ_SEL_SELECT_MAX            (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_IRQ_REQ_SEL_RESETVAL                                (0x00000000U)

/* APPSS_SPI_TRIG_SRC */

#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIA_MASK      (0x00000003U)
#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIA_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIA_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIA_MAX       (0x00000003U)

#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIB_MASK      (0x07FF0000U)
#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIB_SHIFT     (0x00000010U)
#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIB_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_APPSS_SPI_TRIG_SRC_TRIG_SPIB_MAX       (0x000007FFU)

#define CSL_APP_CTRL_APPSS_SPI_TRIG_SRC_RESETVAL                               (0x00000000U)

/* APPSS_RAM1A_MEM_INIT */

#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_APPSS_RAM1A_MEM_INIT_MEM_INIT_MASK   (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_APPSS_RAM1A_MEM_INIT_MEM_INIT_SHIFT  (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_APPSS_RAM1A_MEM_INIT_MEM_INIT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_APPSS_RAM1A_MEM_INIT_MEM_INIT_MAX    (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_RESETVAL                             (0x00000000U)

/* APPSS_RAM1A_MEM_INIT_DONE */

#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_DONE_APPSS_RAM1A_MEM_INIT_DONE_MEM_INIT_DONE_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_DONE_APPSS_RAM1A_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_DONE_APPSS_RAM1A_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_DONE_APPSS_RAM1A_MEM_INIT_DONE_MEM_INIT_DONE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_DONE_RESETVAL                        (0x00000000U)

/* APPSS_RAM1A_MEM_INIT_STATUS */

#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_STATUS_APPSS_RAM1A_MEM_INIT_STATUS_MEM_STATUS_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_STATUS_APPSS_RAM1A_MEM_INIT_STATUS_MEM_STATUS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_STATUS_APPSS_RAM1A_MEM_INIT_STATUS_MEM_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_STATUS_APPSS_RAM1A_MEM_INIT_STATUS_MEM_STATUS_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM1A_MEM_INIT_STATUS_RESETVAL                      (0x00000000U)

/* APPSS_RAM2A_MEM_INIT */

#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_APPSS_RAM2A_MEM_INIT_MEM_INIT_MASK   (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_APPSS_RAM2A_MEM_INIT_MEM_INIT_SHIFT  (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_APPSS_RAM2A_MEM_INIT_MEM_INIT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_APPSS_RAM2A_MEM_INIT_MEM_INIT_MAX    (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_RESETVAL                             (0x00000000U)

/* APPSS_RAM2A_MEM_INIT_DONE */

#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_DONE_APPSS_RAM2A_MEM_INIT_DONE_MEM_INIT_DONE_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_DONE_APPSS_RAM2A_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_DONE_APPSS_RAM2A_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_DONE_APPSS_RAM2A_MEM_INIT_DONE_MEM_INIT_DONE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_DONE_RESETVAL                        (0x00000000U)

/* APPSS_RAM2A_MEM_INIT_STATUS */

#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_STATUS_APPSS_RAM2A_MEM_INIT_STATUS_MEM_STATUS_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_STATUS_APPSS_RAM2A_MEM_INIT_STATUS_MEM_STATUS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_STATUS_APPSS_RAM2A_MEM_INIT_STATUS_MEM_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_STATUS_APPSS_RAM2A_MEM_INIT_STATUS_MEM_STATUS_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM2A_MEM_INIT_STATUS_RESETVAL                      (0x00000000U)

/* APPSS_RAM3A_MEM_INIT */

#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_APPSS_RAM3A_MEM_INIT_MEM_INIT_MASK   (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_APPSS_RAM3A_MEM_INIT_MEM_INIT_SHIFT  (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_APPSS_RAM3A_MEM_INIT_MEM_INIT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_APPSS_RAM3A_MEM_INIT_MEM_INIT_MAX    (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_RESETVAL                             (0x00000000U)

/* APPSS_RAM3A_MEM_INIT_DONE */

#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_DONE_APPSS_RAM3A_MEM_INIT_DONE_MEM_INIT_DONE_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_DONE_APPSS_RAM3A_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_DONE_APPSS_RAM3A_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_DONE_APPSS_RAM3A_MEM_INIT_DONE_MEM_INIT_DONE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_DONE_RESETVAL                        (0x00000000U)

/* APPSS_RAM3A_MEM_INIT_STATUS */

#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_STATUS_APPSS_RAM3A_MEM_INIT_STATUS_MEM_STATUS_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_STATUS_APPSS_RAM3A_MEM_INIT_STATUS_MEM_STATUS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_STATUS_APPSS_RAM3A_MEM_INIT_STATUS_MEM_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_STATUS_APPSS_RAM3A_MEM_INIT_STATUS_MEM_STATUS_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM3A_MEM_INIT_STATUS_RESETVAL                      (0x00000000U)

/* HWASS_SHRD_RAM0_MEM_INIT */

#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_HWASS_SHRD_RAM0_MEM_INIT_MEM_INIT_MASK (0x00000001U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_HWASS_SHRD_RAM0_MEM_INIT_MEM_INIT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_HWASS_SHRD_RAM0_MEM_INIT_MEM_INIT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_HWASS_SHRD_RAM0_MEM_INIT_MEM_INIT_MAX (0x00000001U)

#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_RESETVAL                         (0x00000000U)

/* HWASS_SHRD_RAM0_MEM_INIT_DONE */

#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE_MASK (0x00000001U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE_MAX (0x00000001U)

#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_RESETVAL                    (0x00000000U)

/* HWASS_SHRD_RAM0_MEM_INIT_STATUS */

#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_STATUS_HWASS_SHRD_RAM0_MEM_INIT_STATUS_MEM_STATUS_MASK (0x00000001U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_STATUS_HWASS_SHRD_RAM0_MEM_INIT_STATUS_MEM_STATUS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_STATUS_HWASS_SHRD_RAM0_MEM_INIT_STATUS_MEM_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_STATUS_HWASS_SHRD_RAM0_MEM_INIT_STATUS_MEM_STATUS_MAX (0x00000001U)

#define CSL_APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_STATUS_RESETVAL                  (0x00000000U)

/* HWASS_SHRD_RAM1_MEM_INIT */

#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_HWASS_SHRD_RAM1_MEM_INIT_MEM_INIT_MASK (0x00000001U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_HWASS_SHRD_RAM1_MEM_INIT_MEM_INIT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_HWASS_SHRD_RAM1_MEM_INIT_MEM_INIT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_HWASS_SHRD_RAM1_MEM_INIT_MEM_INIT_MAX (0x00000001U)

#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_RESETVAL                         (0x00000000U)

/* HWASS_SHRD_RAM1_MEM_INIT_DONE */

#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE_MASK (0x00000001U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE_MAX (0x00000001U)

#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_RESETVAL                    (0x00000000U)

/* HWASS_SHRD_RAM1_MEM_INIT_STATUS */

#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_STATUS_HWASS_SHRD_RAM1_MEM_INIT_STATUS_MEM_STATUS_MASK (0x00000001U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_STATUS_HWASS_SHRD_RAM1_MEM_INIT_STATUS_MEM_STATUS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_STATUS_HWASS_SHRD_RAM1_MEM_INIT_STATUS_MEM_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_STATUS_HWASS_SHRD_RAM1_MEM_INIT_STATUS_MEM_STATUS_MAX (0x00000001U)

#define CSL_APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_STATUS_RESETVAL                  (0x00000000U)

/* APPSS_TPCC_MEMINIT_START */

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_START_RESETVAL                         (0x00000000U)

/* APPSS_TPCC_MEMINIT_DONE */

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_DONE_RESETVAL                          (0x00000000U)

/* APPSS_TPCC_MEMINIT_STATUS */

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_APPSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_MEMINIT_STATUS_RESETVAL                        (0x00000000U)

/* APPSS_SPIA_CFG */

#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIASYNC2SEN_MASK           (0x00000001U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIASYNC2SEN_SHIFT          (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIASYNC2SEN_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIASYNC2SEN_MAX            (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_MASK     (0x00000100U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_SHIFT    (0x00000008U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_MAX      (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_TRIG_GATE_EN_MASK      (0x00010000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_TRIG_GATE_EN_SHIFT     (0x00000010U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_TRIG_GATE_EN_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_TRIG_GATE_EN_MAX       (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_IODFT_EN_MASK          (0x10000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_IODFT_EN_SHIFT         (0x0000001CU)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_IODFT_EN_RESETVAL      (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIA_CFG_APPSS_SPIA_CFG_SPIA_IODFT_EN_MAX           (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIA_CFG_RESETVAL                                   (0x00000000U)

/* APPSS_SPIB_CFG */

#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIBSYNC2SEN_MASK           (0x00000001U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIBSYNC2SEN_SHIFT          (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIBSYNC2SEN_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIBSYNC2SEN_MAX            (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_MASK     (0x00000100U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_SHIFT    (0x00000008U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_MAX      (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_TRIG_GATE_EN_MASK      (0x00010000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_TRIG_GATE_EN_SHIFT     (0x00000010U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_TRIG_GATE_EN_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_TRIG_GATE_EN_MAX       (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_IODFT_EN_MASK          (0x10000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_IODFT_EN_SHIFT         (0x0000001CU)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_IODFT_EN_RESETVAL      (0x00000000U)
#define CSL_APP_CTRL_APPSS_SPIB_CFG_APPSS_SPIB_CFG_SPIB_IODFT_EN_MAX           (0x00000001U)

#define CSL_APP_CTRL_APPSS_SPIB_CFG_RESETVAL                                   (0x00000000U)

/* APPSS_EPWM_CFG */

#define CSL_APP_CTRL_APPSS_EPWM_CFG_APPSS_EPWM_CFG_EPWM_CONFIG_MASK            (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_EPWM_CFG_APPSS_EPWM_CFG_EPWM_CONFIG_SHIFT           (0x00000000U)
#define CSL_APP_CTRL_APPSS_EPWM_CFG_APPSS_EPWM_CFG_EPWM_CONFIG_RESETVAL        (0x0F000000U)
#define CSL_APP_CTRL_APPSS_EPWM_CFG_APPSS_EPWM_CFG_EPWM_CONFIG_MAX             (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_EPWM_CFG_RESETVAL                                   (0x0F000000U)

/* RESERVED */

#define CSL_APP_CTRL_RESERVED_RESERVED_GIO_CONFIG_MASK                         (0xFFFFFFFFU)
#define CSL_APP_CTRL_RESERVED_RESERVED_GIO_CONFIG_SHIFT                        (0x00000000U)
#define CSL_APP_CTRL_RESERVED_RESERVED_GIO_CONFIG_RESETVAL                     (0x00000000U)
#define CSL_APP_CTRL_RESERVED_RESERVED_GIO_CONFIG_MAX                          (0xFFFFFFFFU)

#define CSL_APP_CTRL_RESERVED_RESETVAL                                         (0x00000000U)

/* APPSS_MCAN_FE_AND_LIN_INTR_SEL */

#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_MCAN_FE_SEL_MASK (0x00000007U)
#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_MCAN_FE_SEL_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_MCAN_FE_SEL_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_MCAN_FE_SEL_MAX (0x00000007U)

#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_LIN_INTR_SEL_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_LIN_INTR_SEL_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_LIN_INTR_SEL_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_LIN_INTR_SEL_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_MCAN_FE_AND_LIN_INTR_SEL_RESETVAL                   (0x00000000U)

/* APPSS_MCANA_INT_CLR */

#define CSL_APP_CTRL_APPSS_MCANA_INT_CLR_APPSS_MCANA_INT_CLR_MCAN_INT_CLR_MASK (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_MCANA_INT_CLR_APPSS_MCANA_INT_CLR_MCAN_INT_CLR_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_CLR_APPSS_MCANA_INT_CLR_MCAN_INT_CLR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_CLR_APPSS_MCANA_INT_CLR_MCAN_INT_CLR_MAX  (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_MCANA_INT_CLR_RESETVAL                              (0x00000000U)

/* APPSS_MCANA_INT_MASK */

#define CSL_APP_CTRL_APPSS_MCANA_INT_MASK_APPSS_MCANA_INT_MASK_MCAN_INT_MASK_MASK (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_MCANA_INT_MASK_APPSS_MCANA_INT_MASK_MCAN_INT_MASK_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_MASK_APPSS_MCANA_INT_MASK_MCAN_INT_MASK_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_MASK_APPSS_MCANA_INT_MASK_MCAN_INT_MASK_MAX (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_MCANA_INT_MASK_RESETVAL                             (0x00000000U)

/* APPSS_MCANA_INT_STAT */

#define CSL_APP_CTRL_APPSS_MCANA_INT_STAT_APPSS_MCANA_INT_STAT_MCAN_INT_STATUS_MASK (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_MCANA_INT_STAT_APPSS_MCANA_INT_STAT_MCAN_INT_STATUS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_STAT_APPSS_MCANA_INT_STAT_MCAN_INT_STATUS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MCANA_INT_STAT_APPSS_MCANA_INT_STAT_MCAN_INT_STATUS_MAX (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_MCANA_INT_STAT_RESETVAL                             (0x00000000U)

/* APPSS_CM4_GLOBAL_CONFIG */

#define CSL_APP_CTRL_APPSS_CM4_GLOBAL_CONFIG_APPSS_CM4_GLOBAL_CONFIG_TEINIT_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_CM4_GLOBAL_CONFIG_APPSS_CM4_GLOBAL_CONFIG_TEINIT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_GLOBAL_CONFIG_APPSS_CM4_GLOBAL_CONFIG_TEINIT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_GLOBAL_CONFIG_APPSS_CM4_GLOBAL_CONFIG_TEINIT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_CM4_GLOBAL_CONFIG_RESETVAL                          (0x00000000U)

/* RESERVED1 */

#define CSL_APP_CTRL_RESERVED1_RESERVED1_RES_MASK                              (0xFFFFFFFFU)
#define CSL_APP_CTRL_RESERVED1_RESERVED1_RES_SHIFT                             (0x00000000U)
#define CSL_APP_CTRL_RESERVED1_RESERVED1_RES_RESETVAL                          (0x00000000U)
#define CSL_APP_CTRL_RESERVED1_RESERVED1_RES_MAX                               (0xFFFFFFFFU)

#define CSL_APP_CTRL_RESERVED1_RESETVAL                                        (0x00000000U)

/* APPSS_CM4_ROM_ECLIPSE */

#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_MASK  (0x00000007U)
#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_MAX   (0x00000007U)

#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_WAIT_MASK (0x00000700U)
#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_WAIT_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_WAIT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_APPSS_CM4_ROM_ECLIPSE_MEMSWAP_WAIT_MAX (0x00000007U)

#define CSL_APP_CTRL_APPSS_CM4_ROM_ECLIPSE_RESETVAL                            (0x00000000U)

/* APPSS_CM4_STATUS_REG */

#define CSL_APP_CTRL_APPSS_CM4_STATUS_REG_APPSS_CM4_STATUS_REG_MEMSWAP_MASK    (0x00000001U)
#define CSL_APP_CTRL_APPSS_CM4_STATUS_REG_APPSS_CM4_STATUS_REG_MEMSWAP_SHIFT   (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_STATUS_REG_APPSS_CM4_STATUS_REG_MEMSWAP_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_STATUS_REG_APPSS_CM4_STATUS_REG_MEMSWAP_MAX     (0x00000001U)

#define CSL_APP_CTRL_APPSS_CM4_STATUS_REG_RESETVAL                             (0x00000000U)

/* APPSS_AHB_CTRL */

#define CSL_APP_CTRL_APPSS_AHB_CTRL_APPSS_AHB_CTRL_CPU0_AHB_INIT_MASK          (0x00000001U)
#define CSL_APP_CTRL_APPSS_AHB_CTRL_APPSS_AHB_CTRL_CPU0_AHB_INIT_SHIFT         (0x00000000U)
#define CSL_APP_CTRL_APPSS_AHB_CTRL_APPSS_AHB_CTRL_CPU0_AHB_INIT_RESETVAL      (0x00000001U)
#define CSL_APP_CTRL_APPSS_AHB_CTRL_APPSS_AHB_CTRL_CPU0_AHB_INIT_MAX           (0x00000001U)

#define CSL_APP_CTRL_APPSS_AHB_CTRL_RESETVAL                                   (0x00000001U)

/* ESM_GATING0 */

#define CSL_APP_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING0_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING1 */

#define CSL_APP_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING1_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING2 */

#define CSL_APP_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING2_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING3 */

#define CSL_APP_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING3_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING4 */

#define CSL_APP_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING4_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING5 */

#define CSL_APP_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING5_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING6 */

#define CSL_APP_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING6_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING7 */

#define CSL_APP_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define CSL_APP_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define CSL_APP_CTRL_ESM_GATING7_RESETVAL                                      (0xFFFFFFFFU)

/* APPSS_CM4_HALT */

#define CSL_APP_CTRL_APPSS_CM4_HALT_APPSS_CM4_HALT_HALT_MASK                   (0x00000007U)
#define CSL_APP_CTRL_APPSS_CM4_HALT_APPSS_CM4_HALT_HALT_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_HALT_APPSS_CM4_HALT_HALT_RESETVAL               (0x00000007U)
#define CSL_APP_CTRL_APPSS_CM4_HALT_APPSS_CM4_HALT_HALT_MAX                    (0x00000007U)

#define CSL_APP_CTRL_APPSS_CM4_HALT_RESETVAL                                   (0x00000007U)

/* APPSS_CM4_EVENT */

#define CSL_APP_CTRL_APPSS_CM4_EVENT_APPSS_CM4_EVENT_CPU0_EVENT_MASK           (0x00000007U)
#define CSL_APP_CTRL_APPSS_CM4_EVENT_APPSS_CM4_EVENT_CPU0_EVENT_SHIFT          (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_EVENT_APPSS_CM4_EVENT_CPU0_EVENT_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_CM4_EVENT_APPSS_CM4_EVENT_CPU0_EVENT_MAX            (0x00000007U)

#define CSL_APP_CTRL_APPSS_CM4_EVENT_RESETVAL                                  (0x00000000U)

/* SPIA_IO_CFG */

#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_MASK                     (0x00000001U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_SHIFT                    (0x00000000U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_RESETVAL                 (0x00000000U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_MAX                      (0x00000001U)

#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_MASK                       (0x00000100U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_SHIFT                      (0x00000008U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_RESETVAL                   (0x00000000U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_MAX                        (0x00000001U)

#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_MASK               (0x00010000U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_SHIFT              (0x00000010U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_MAX                (0x00000001U)

#define CSL_APP_CTRL_SPIA_IO_CFG_RESETVAL                                      (0x00000000U)

/* SPIB_IO_CFG */

#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_MASK                     (0x00000001U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_SHIFT                    (0x00000000U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_RESETVAL                 (0x00000000U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_MAX                      (0x00000001U)

#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_MASK                       (0x00000100U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_SHIFT                      (0x00000008U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_RESETVAL                   (0x00000000U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_MAX                        (0x00000001U)

#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_MASK               (0x00010000U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_SHIFT              (0x00000010U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_MAX                (0x00000001U)

#define CSL_APP_CTRL_SPIB_IO_CFG_RESETVAL                                      (0x00000000U)

/* SPI_HOST_IRQ */

#define CSL_APP_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_MASK                   (0x00000001U)
#define CSL_APP_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_RESETVAL               (0x00000000U)
#define CSL_APP_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_MAX                    (0x00000001U)

#define CSL_APP_CTRL_SPI_HOST_IRQ_RESETVAL                                     (0x00000000U)

/* TPTC_DBS_CONFIG */

#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_MASK              (0x00000003U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_SHIFT             (0x00000000U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_RESETVAL          (0x00000002U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_MAX               (0x00000003U)

#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_MASK              (0x00000030U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_SHIFT             (0x00000004U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_RESETVAL          (0x00000001U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_MAX               (0x00000003U)

#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_MASK              (0x00000300U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_SHIFT             (0x00000008U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_RESETVAL          (0x00000001U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_MAX               (0x00000003U)

#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_MASK              (0x00003000U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_SHIFT             (0x0000000CU)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_RESETVAL          (0x00000001U)
#define CSL_APP_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_MAX               (0x00000003U)

#define CSL_APP_CTRL_TPTC_DBS_CONFIG_RESETVAL                                  (0x00001112U)

/* TPCC_PARITY_CTRL */

#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_MASK   (0x00000001U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_SHIFT  (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_MAX    (0x00000001U)

#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_MASK (0x00000010U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_SHIFT (0x00000004U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_MAX (0x00000001U)

#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_MASK   (0x00000100U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_SHIFT  (0x00000008U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_MAX    (0x00000001U)

#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_MASK (0x00001000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_SHIFT (0x0000000CU)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_MAX (0x00000001U)

#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_MASK (0x00010000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_MAX (0x00000001U)

#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_MASK (0x00100000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_SHIFT (0x00000014U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_MAX (0x00000001U)

#define CSL_APP_CTRL_TPCC_PARITY_CTRL_RESETVAL                                 (0x00000000U)

/* TPCC_PARITY_STATUS */

#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_MASK (0x000000FFU)
#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_SHIFT (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_MAX (0x000000FFU)

#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_MASK (0x00FF0000U)
#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_MAX (0x000000FFU)

#define CSL_APP_CTRL_TPCC_PARITY_STATUS_RESETVAL                               (0x00000000U)

/* APPSS_DBG_ACK_CTL0 */

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_CAN_MASK            (0x00000001U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_CAN_SHIFT           (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_CAN_RESETVAL        (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_CAN_MAX             (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_RTI_MASK            (0x00000010U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_RTI_SHIFT           (0x00000004U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_RTI_RESETVAL        (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_RTI_MAX             (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_WDT_MASK            (0x00000100U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_WDT_SHIFT           (0x00000008U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_WDT_RESETVAL        (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_WDT_MAX             (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_MCRC_MASK           (0x00001000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_MCRC_SHIFT          (0x0000000CU)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_MCRC_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_MCRC_MAX            (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_I2C_MASK            (0x00010000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_I2C_SHIFT           (0x00000010U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_I2C_RESETVAL        (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_I2C_MAX             (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIA_MASK           (0x00100000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIA_SHIFT          (0x00000014U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIA_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIA_MAX            (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIB_MASK           (0x01000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIB_SHIFT          (0x00000018U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIB_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_SCIB_MAX            (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_LIN_MASK            (0x10000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_LIN_SHIFT           (0x0000001CU)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_LIN_RESETVAL        (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_LIN_MAX             (0x00000001U)

#define CSL_APP_CTRL_APPSS_DBG_ACK_CTL0_RESETVAL                               (0x00000000U)

/* DEBUGSS_CSETB_FLUSH */

#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_MASK (0x00000001U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_SHIFT (0x00000000U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_MAX (0x00000001U)

#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_MASK (0x00000100U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_SHIFT (0x00000008U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_MAX (0x00000001U)

#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_MASK (0x00000200U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_SHIFT (0x00000009U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_MAX (0x00000001U)

#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_MASK   (0x00000400U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_SHIFT  (0x0000000AU)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_MAX    (0x00000001U)

#define CSL_APP_CTRL_DEBUGSS_CSETB_FLUSH_RESETVAL                              (0x00000000U)

/* CPSW_CONTROL */

#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_MASK             (0x00000001U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_SHIFT            (0x00000000U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_MAX              (0x00000001U)

#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_MASK          (0x00000100U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_SHIFT         (0x00000008U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_RESETVAL      (0x00000000U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_MAX           (0x00000001U)

#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_MASK             (0x00010000U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_SHIFT            (0x00000010U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_MAX              (0x00000001U)

#define CSL_APP_CTRL_CPSW_CONTROL_RESETVAL                                     (0x00000000U)

/* APPSS_ERRAGG_MASK0 */

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_RD_MASK     (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_RD_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_RD_MAX      (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_WR_MASK     (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_WR_SHIFT    (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_RCM_WR_MAX      (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_RD_MASK    (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_RD_SHIFT   (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_RD_MAX     (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_WR_MASK    (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_WR_SHIFT   (0x00000003U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_CTRL_WR_MAX     (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_RD_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_RD_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_RD_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_WR_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_WR_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_IDALLOC_WR_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_RD_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_RD_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_WR_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_WR_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_ADCBUFF_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_RD_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_RD_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_RD_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_WR_MASK (0x00000200U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_WR_SHIFT (0x00000009U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_PLLDIG_CTRL_WR_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_RD_MASK  (0x00000400U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_RD_SHIFT (0x0000000AU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_RD_MAX   (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_WR_MASK  (0x00000800U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_WR_SHIFT (0x0000000BU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOPSS_CTRL_WR_MAX   (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_RD_MASK   (0x00001000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_RD_SHIFT  (0x0000000CU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_RD_MAX    (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_WR_MASK   (0x00002000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_WR_SHIFT  (0x0000000DU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APLL_CTRL_WR_MAX    (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_RD_MASK (0x00004000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_RD_SHIFT (0x0000000EU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_RD_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_WR_MASK (0x00008000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_WR_SHIFT (0x0000000FU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FRAME_TIMER_WR_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_RD_MASK    (0x00010000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_RD_SHIFT   (0x00000010U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_RD_MAX     (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_WR_MASK    (0x00020000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_WR_SHIFT   (0x00000011U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_PRCM_WR_MAX     (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_RD_MASK (0x00040000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_RD_SHIFT (0x00000012U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_WR_MASK (0x00080000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_WR_SHIFT (0x00000013U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_TOP_EFUSE_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_MPU_MASK        (0x00100000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_MPU_SHIFT       (0x00000014U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_MPU_RESETVAL    (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_MPU_MAX         (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_AHB_MASK        (0x00200000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_AHB_SHIFT       (0x00000015U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_AHB_RESETVAL    (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_AHB_MAX         (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_SHARED_MEM_MASK (0x00400000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_SHARED_MEM_SHIFT (0x00000016U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_SHARED_MEM_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_APP_SHARED_MEM_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FEC_ERRORAGG_MASK   (0x00800000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FEC_ERRORAGG_SHIFT  (0x00000017U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FEC_ERRORAGG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_APPSS_ERRAGG_MASK0_FEC_ERRORAGG_MAX    (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK0_RESETVAL                               (0x00000000U)

/* APPSS_ERRAGG_STATUS0 */

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_RD_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_RD_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_RD_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_WR_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_WR_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_RCM_WR_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_RD_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_RD_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_WR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_WR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_RD_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_RD_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_WR_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_WR_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_IDALLOC_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_RD_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_RD_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_WR_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_WR_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_ADCBUFF_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_RD_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_RD_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_WR_MASK (0x00000200U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_WR_SHIFT (0x00000009U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_PLLDIG_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_RD_MASK (0x00000400U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_RD_SHIFT (0x0000000AU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_WR_MASK (0x00000800U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_WR_SHIFT (0x0000000BU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOPSS_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_RD_MASK (0x00001000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_RD_SHIFT (0x0000000CU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_WR_MASK (0x00002000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_WR_SHIFT (0x0000000DU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APLL_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_RD_MASK (0x00004000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_RD_SHIFT (0x0000000EU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_WR_MASK (0x00008000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_WR_SHIFT (0x0000000FU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FRAME_TIMER_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_RD_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_RD_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_WR_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_WR_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_PRCM_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_RD_MASK (0x00040000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_RD_SHIFT (0x00000012U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_RD_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_WR_MASK (0x00080000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_WR_SHIFT (0x00000013U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_TOP_EFUSE_CTRL_WR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_MPU_RD_MASK (0x00100000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_MPU_RD_SHIFT (0x00000014U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_MPU_RD_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_MPU_RD_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_AHB_WR_MASK (0x00200000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_AHB_WR_SHIFT (0x00000015U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_AHB_WR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_AHB_WR_MAX  (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_SHARED_MEM_ERR_MASK (0x00400000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_SHARED_MEM_ERR_SHIFT (0x00000016U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_SHARED_MEM_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_APP_SHARED_MEM_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FEC_ERRORAGG_MASK (0x00800000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FEC_ERRORAGG_SHIFT (0x00000017U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FEC_ERRORAGG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_APPSS_ERRAGG_STATUS0_FEC_ERRORAGG_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS0_RESETVAL                             (0x00000000U)

/* APPSS_TPCC_A_ERRAGG_MASK */

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_APPSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_MASK_RESETVAL                         (0x00000000U)

/* APPSS_TPCC_A_ERRAGG_STATUS */

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_APPSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RESETVAL                       (0x00000000U)

/* APPSS_TPCC_A_ERRAGG_STATUS_RAW */

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_APPSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_ERRAGG_STATUS_RAW_RESETVAL                   (0x00000000U)

/* APPSS_TPCC_A_INTAGG_MASK */

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A0_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A0_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A1_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A1_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_APPSS_TPCC_A_INTAGG_MASK_TPTC_A1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK_RESETVAL                         (0x00000000U)

/* APPSS_TPCC_A_INTAGG_STATUS */

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A0_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A1_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A1_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_APPSS_TPCC_A_INTAGG_STATUS_TPTC_A1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RESETVAL                       (0x00000000U)

/* APPSS_TPCC_A_INTAGG_STATUS_RAW */

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_APPSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS_RAW_RESETVAL                   (0x00000000U)

/* APPSS_TPCC_B_ERRAGG_MASK */

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_MASK (0x00004000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_SHIFT (0x0000000EU)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_APPSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_MASK_RESETVAL                         (0x00000000U)

/* APPSS_TPCC_B_ERRAGG_STATUS */

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_MASK (0x00004000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_SHIFT (0x0000000EU)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_APPSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RESETVAL                       (0x00000000U)

/* APPSS_TPCC_B_ERRAGG_STATUS_RAW */

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_MASK (0x00004000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_SHIFT (0x0000000EU)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_APPSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_ERRAGG_STATUS_RAW_RESETVAL                   (0x00000000U)

/* APPSS_TPCC_B_INTAGG_MASK */

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B0_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B0_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B1_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B1_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_APPSS_TPCC_B_INTAGG_MASK_TPTC_B1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK_RESETVAL                         (0x00000000U)

/* APPSS_TPCC_B_INTAGG_STATUS */

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B0_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B0_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B1_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B1_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_APPSS_TPCC_B_INTAGG_STATUS_TPTC_B1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RESETVAL                       (0x00000000U)

/* APPSS_TPCC_B_INTAGG_STATUS_RAW */

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_MASK (0x00020000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_SHIFT (0x00000011U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_APPSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS_RAW_RESETVAL                   (0x00000000U)

/* APPSS_MPU_ERRAGG_MASK */

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_FECSS_MPU_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_FECSS_MPU_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_FECSS_MPU_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_APPSS_MPU_ERRAGG_MASK_FECSS_MPU_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_MASK_RESETVAL                            (0x00000000U)

/* APPSS_MPU_ERRAGG_STATUS */

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_FECSS_MPU_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_FECSS_MPU_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_FECSS_MPU_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_APPSS_MPU_ERRAGG_STATUS_FECSS_MPU_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RESETVAL                          (0x00000000U)

/* APPSS_MPU_ERRAGG_STATUS_RAW */

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_FECSS_MPU_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_FECSS_MPU_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_FECSS_MPU_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_APPSS_MPU_ERRAGG_STATUS_RAW_FECSS_MPU_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_MPU_ERRAGG_STATUS_RAW_RESETVAL                      (0x00000000U)

/* APPSS_QSPI_CONFIG */

#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_EXT_CLK_MASK          (0x00000001U)
#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_EXT_CLK_SHIFT         (0x00000000U)
#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_EXT_CLK_RESETVAL      (0x00000000U)
#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_EXT_CLK_MAX           (0x00000001U)

#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_CLK_LOOPBACK_MASK     (0x00000100U)
#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_CLK_LOOPBACK_SHIFT    (0x00000008U)
#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_CLK_LOOPBACK_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_APPSS_QSPI_CONFIG_CLK_LOOPBACK_MAX      (0x00000001U)

#define CSL_APP_CTRL_APPSS_QSPI_CONFIG_RESETVAL                                (0x00000000U)

/* APPSS_CTI_TRIG_SEL */

#define CSL_APP_CTRL_APPSS_CTI_TRIG_SEL_APPSS_CTI_TRIG_SEL_TRIG8_SEL_MASK      (0x000000FFU)
#define CSL_APP_CTRL_APPSS_CTI_TRIG_SEL_APPSS_CTI_TRIG_SEL_TRIG8_SEL_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_APPSS_CTI_TRIG_SEL_APPSS_CTI_TRIG_SEL_TRIG8_SEL_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_CTI_TRIG_SEL_APPSS_CTI_TRIG_SEL_TRIG8_SEL_MAX       (0x000000FFU)

#define CSL_APP_CTRL_APPSS_CTI_TRIG_SEL_RESETVAL                               (0x00000000U)

/* APPSS_DBGSS_CTI_TRIG_SEL */

#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG1_MASK (0x000000FFU)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG1_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG1_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG1_MAX (0x000000FFU)

#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG2_MASK (0x0000FF00U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG2_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG2_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG2_MAX (0x000000FFU)

#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG3_MASK (0x00FF0000U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG3_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG3_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_APPSS_DBGSS_CTI_TRIG_SEL_TRIG3_MAX (0x000000FFU)

#define CSL_APP_CTRL_APPSS_DBGSS_CTI_TRIG_SEL_RESETVAL                         (0x00000000U)

/* APPSS_BOOT_INFO_REG0 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG0_APPSS_BOOT_INFO_REG0_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG0_APPSS_BOOT_INFO_REG0_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG0_APPSS_BOOT_INFO_REG0_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG0_APPSS_BOOT_INFO_REG0_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG0_RESETVAL                             (0x00000000U)

/* APPSS_BOOT_INFO_REG1 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG1_APPSS_BOOT_INFO_REG1_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG1_APPSS_BOOT_INFO_REG1_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG1_APPSS_BOOT_INFO_REG1_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG1_APPSS_BOOT_INFO_REG1_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG1_RESETVAL                             (0x00000000U)

/* APPSS_BOOT_INFO_REG2 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG2_APPSS_BOOT_INFO_REG2_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG2_APPSS_BOOT_INFO_REG2_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG2_APPSS_BOOT_INFO_REG2_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG2_APPSS_BOOT_INFO_REG2_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG2_RESETVAL                             (0x00000000U)

/* APPSS_BOOT_INFO_REG3 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG3_APPSS_BOOT_INFO_REG3_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG3_APPSS_BOOT_INFO_REG3_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG3_APPSS_BOOT_INFO_REG3_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG3_APPSS_BOOT_INFO_REG3_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG3_RESETVAL                             (0x00000000U)

/* APPSS_BOOT_INFO_REG4 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG4_APPSS_BOOT_INFO_REG4_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG4_APPSS_BOOT_INFO_REG4_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG4_APPSS_BOOT_INFO_REG4_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG4_APPSS_BOOT_INFO_REG4_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG4_RESETVAL                             (0x00000000U)

/* APPSS_BOOT_INFO_REG5 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG5_APPSS_BOOT_INFO_REG5_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG5_APPSS_BOOT_INFO_REG5_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG5_APPSS_BOOT_INFO_REG5_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG5_APPSS_BOOT_INFO_REG5_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG5_RESETVAL                             (0x00000000U)

/* APPSS_BOOT_INFO_REG6 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG6_APPSS_BOOT_INFO_REG6_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG6_APPSS_BOOT_INFO_REG6_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG6_APPSS_BOOT_INFO_REG6_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG6_APPSS_BOOT_INFO_REG6_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG6_RESETVAL                             (0x00000000U)

/* APPSS_BOOT_INFO_REG7 */

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG7_APPSS_BOOT_INFO_REG7_CONFIG_MASK     (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG7_APPSS_BOOT_INFO_REG7_CONFIG_SHIFT    (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG7_APPSS_BOOT_INFO_REG7_CONFIG_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG7_APPSS_BOOT_INFO_REG7_CONFIG_MAX      (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_BOOT_INFO_REG7_RESETVAL                             (0x00000000U)

/* APPSS_TPTC_ECCAGGR_CLK_CNTRL */

#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_RESETVAL (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_RESETVAL (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_RESETVAL (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPTC_ECCAGGR_CLK_CNTRL_RESETVAL                     (0x00000007U)

/* APPSS_TPTC_BOUNDARY_CFG */

#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MASK (0x0000003FU)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_RESETVAL (0x00000013U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MAX (0x0000003FU)

#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_MASK (0x00003F00U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_RESETVAL (0x00000013U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_MAX (0x0000003FU)

#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_MASK (0x003F0000U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_RESETVAL (0x00000013U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_MAX (0x0000003FU)

#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B1_SIZE_MASK (0x3F000000U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B1_SIZE_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B1_SIZE_RESETVAL (0x00000013U)
#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_APPSS_TPTC_BOUNDARY_CFG_TPTC_B1_SIZE_MAX (0x0000003FU)

#define CSL_APP_CTRL_APPSS_TPTC_BOUNDARY_CFG_RESETVAL                          (0x13131313U)

/* APPSS_TPTC_XID_REORDER_CFG */

#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_MASK (0x00010000U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_SHIFT (0x00000010U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B1_DISABLE_MASK (0x01000000U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B1_DISABLE_SHIFT (0x00000018U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B1_DISABLE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_APPSS_TPTC_XID_REORDER_CFG_TPTC_B1_DISABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_TPTC_XID_REORDER_CFG_RESETVAL                       (0x00000000U)

/* HW_SYNC_FE_CTRL */

#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_MASK              (0x00000001U)
#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_SHIFT             (0x00000000U)
#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_MASK              (0x00000100U)
#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_SHIFT             (0x00000008U)
#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SYNC_FE_CTRL_RESETVAL                                  (0x00000000U)

/* HW_SPARE_REG1 */

#define CSL_APP_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_MASK                       (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_SHIFT                      (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_RESETVAL                   (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_MAX                        (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_REG1_RESETVAL                                    (0x00000000U)

/* HW_SPARE_REG2 */

#define CSL_APP_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_MASK                       (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_SHIFT                      (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_RESETVAL                   (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_MAX                        (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_REG2_RESETVAL                                    (0x00000000U)

/* HW_SPARE_REG3 */

#define CSL_APP_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_MASK                       (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_SHIFT                      (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_RESETVAL                   (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_MAX                        (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_REG3_RESETVAL                                    (0x00000000U)

/* NERROR_MASK */

#define CSL_APP_CTRL_NERROR_MASK_NERROR_MASK_MASK_MASK                         (0x00000001U)
#define CSL_APP_CTRL_NERROR_MASK_NERROR_MASK_MASK_SHIFT                        (0x00000000U)
#define CSL_APP_CTRL_NERROR_MASK_NERROR_MASK_MASK_RESETVAL                     (0x00000001U)
#define CSL_APP_CTRL_NERROR_MASK_NERROR_MASK_MASK_MAX                          (0x00000001U)

#define CSL_APP_CTRL_NERROR_MASK_RESETVAL                                      (0x00000001U)

/* HW_SPARE_RW0 */

#define CSL_APP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW1 */

#define CSL_APP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW2 */

#define CSL_APP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW3 */

#define CSL_APP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW4 */

#define CSL_APP_CTRL_HW_SPARE_RW4_HW_SPARE_RW4_HW_SPARE_RW4_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW4_HW_SPARE_RW4_HW_SPARE_RW4_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW4_HW_SPARE_RW4_HW_SPARE_RW4_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW4_HW_SPARE_RW4_HW_SPARE_RW4_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW4_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW5 */

#define CSL_APP_CTRL_HW_SPARE_RW5_HW_SPARE_RW5_HW_SPARE_RW5_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW5_HW_SPARE_RW5_HW_SPARE_RW5_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW5_HW_SPARE_RW5_HW_SPARE_RW5_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW5_HW_SPARE_RW5_HW_SPARE_RW5_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW5_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO0 */

#define CSL_APP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RO0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO1 */

#define CSL_APP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RO1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO2 */

#define CSL_APP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RO2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO3 */

#define CSL_APP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RO3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_REC */

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK              (0x00000001U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT             (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK              (0x00000002U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT             (0x00000001U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK              (0x00000004U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT             (0x00000002U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK              (0x00000008U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT             (0x00000003U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK              (0x00000010U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT             (0x00000004U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK              (0x00000020U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT             (0x00000005U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK              (0x00000040U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT             (0x00000006U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK              (0x00000080U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT             (0x00000007U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK              (0x00000100U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT             (0x00000008U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK              (0x00000200U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT             (0x00000009U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX               (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK             (0x00000400U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT            (0x0000000AU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK             (0x00000800U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT            (0x0000000BU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK             (0x00001000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT            (0x0000000CU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK             (0x00002000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT            (0x0000000DU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK             (0x00004000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT            (0x0000000EU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK             (0x00008000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT            (0x0000000FU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK             (0x00010000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT            (0x00000010U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK             (0x00020000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT            (0x00000011U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK             (0x00040000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT            (0x00000012U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK             (0x00080000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT            (0x00000013U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK             (0x00100000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT            (0x00000014U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK             (0x00200000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT            (0x00000015U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK             (0x00400000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT            (0x00000016U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK             (0x00800000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT            (0x00000017U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK             (0x01000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT            (0x00000018U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK             (0x02000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT            (0x00000019U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK             (0x04000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT            (0x0000001AU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK             (0x08000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT            (0x0000001BU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK             (0x10000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT            (0x0000001CU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK             (0x20000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT            (0x0000001DU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK             (0x40000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT            (0x0000001EU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK             (0x80000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT            (0x0000001FU)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX              (0x00000001U)

#define CSL_APP_CTRL_HW_SPARE_REC_RESETVAL                                     (0x00000000U)

/* APP_CTRL */

#define CSL_APP_CTRL_APP_CTRL_APP_CTRL_ECC_DISABLE_2K_RAM_MASK                 (0x00000001U)
#define CSL_APP_CTRL_APP_CTRL_APP_CTRL_ECC_DISABLE_2K_RAM_SHIFT                (0x00000000U)
#define CSL_APP_CTRL_APP_CTRL_APP_CTRL_ECC_DISABLE_2K_RAM_RESETVAL             (0x00000001U)
#define CSL_APP_CTRL_APP_CTRL_APP_CTRL_ECC_DISABLE_2K_RAM_MAX                  (0x00000001U)

#define CSL_APP_CTRL_APP_CTRL_RESETVAL                                         (0x00000001U)

/* WIC_CTRL */

#define CSL_APP_CTRL_WIC_CTRL_WIC_CTRL_WICMASK_MASK                            (0xFFFFFFFFU)
#define CSL_APP_CTRL_WIC_CTRL_WIC_CTRL_WICMASK_SHIFT                           (0x00000000U)
#define CSL_APP_CTRL_WIC_CTRL_WIC_CTRL_WICMASK_RESETVAL                        (0x00000000U)
#define CSL_APP_CTRL_WIC_CTRL_WIC_CTRL_WICMASK_MAX                             (0xFFFFFFFFU)

#define CSL_APP_CTRL_WIC_CTRL_RESETVAL                                         (0x00000000U)

/* WIC_STAT_CLR */

#define CSL_APP_CTRL_WIC_STAT_CLR_WIC_STAT_CLR_WICSTATCLR_MASK                 (0xFFFFFFFFU)
#define CSL_APP_CTRL_WIC_STAT_CLR_WIC_STAT_CLR_WICSTATCLR_SHIFT                (0x00000000U)
#define CSL_APP_CTRL_WIC_STAT_CLR_WIC_STAT_CLR_WICSTATCLR_RESETVAL             (0x00000000U)
#define CSL_APP_CTRL_WIC_STAT_CLR_WIC_STAT_CLR_WICSTATCLR_MAX                  (0xFFFFFFFFU)

#define CSL_APP_CTRL_WIC_STAT_CLR_RESETVAL                                     (0x00000000U)

/* WIC_STAT */

#define CSL_APP_CTRL_WIC_STAT_WIC_STAT_WICSTAT_MASK                            (0xFFFFFFFFU)
#define CSL_APP_CTRL_WIC_STAT_WIC_STAT_WICSTAT_SHIFT                           (0x00000000U)
#define CSL_APP_CTRL_WIC_STAT_WIC_STAT_WICSTAT_RESETVAL                        (0x00000000U)
#define CSL_APP_CTRL_WIC_STAT_WIC_STAT_WICSTAT_MAX                             (0xFFFFFFFFU)

#define CSL_APP_CTRL_WIC_STAT_RESETVAL                                         (0x00000000U)

/* WICEN */

#define CSL_APP_CTRL_WICEN_WICEN_WICEN_MASK                                    (0x00000001U)
#define CSL_APP_CTRL_WICEN_WICEN_WICEN_SHIFT                                   (0x00000000U)
#define CSL_APP_CTRL_WICEN_WICEN_WICEN_RESETVAL                                (0x00000001U)
#define CSL_APP_CTRL_WICEN_WICEN_WICEN_MAX                                     (0x00000001U)

#define CSL_APP_CTRL_WICEN_RESETVAL                                            (0x00000001U)

/* FORCEFCLKACTIVE */

#define CSL_APP_CTRL_FORCEFCLKACTIVE_FORCEFCLKACTIVE_FORCEFCLKACTIVE_MASK      (0x00000001U)
#define CSL_APP_CTRL_FORCEFCLKACTIVE_FORCEFCLKACTIVE_FORCEFCLKACTIVE_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_FORCEFCLKACTIVE_FORCEFCLKACTIVE_FORCEFCLKACTIVE_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_FORCEFCLKACTIVE_FORCEFCLKACTIVE_FORCEFCLKACTIVE_MAX       (0x00000001U)

#define CSL_APP_CTRL_FORCEFCLKACTIVE_RESETVAL                                  (0x00000000U)

/* FECSS_CLK_GATE */

#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP1_MASK                   (0x00000007U)
#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP1_SHIFT                  (0x00000000U)
#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP1_RESETVAL               (0x00000000U)
#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP1_MAX                    (0x00000007U)

#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP2_MASK                   (0x00000038U)
#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP2_SHIFT                  (0x00000003U)
#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP2_RESETVAL               (0x00000000U)
#define CSL_APP_CTRL_FECSS_CLK_GATE_FECSS_CLK_GATE_GRP2_MAX                    (0x00000007U)

#define CSL_APP_CTRL_FECSS_CLK_GATE_RESETVAL                                   (0x00000000U)

/* APPSS_SHARED_MEM_CLK_GATE */

#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_HWA_ENABLE_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_HWA_ENABLE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_HWA_ENABLE_RESETVAL (0x00000001U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_HWA_ENABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_APP_ENABLE_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_APP_ENABLE_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_APP_ENABLE_RESETVAL (0x00000001U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM0_APP_ENABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_HWA_ENABLE_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_HWA_ENABLE_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_HWA_ENABLE_RESETVAL (0x00000001U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_HWA_ENABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_APP_ENABLE_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_APP_ENABLE_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_APP_ENABLE_RESETVAL (0x00000001U)
#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_APPSS_SHARED_MEM_CLK_GATE_MEM1_APP_ENABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE_RESETVAL                        (0x0000000FU)

/* APPSS_MEM_INIT_SLICE_SEL */

#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK1_MASK (0x00000007U)
#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK1_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK1_RESETVAL (0x00000007U)
#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK1_MAX (0x00000007U)

#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK2_MASK (0x00000018U)
#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK2_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK2_RESETVAL (0x00000003U)
#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_APPSS_MEM_INIT_SLICE_SEL_CFG_BANK2_MAX (0x00000003U)

#define CSL_APP_CTRL_APPSS_MEM_INIT_SLICE_SEL_RESETVAL                         (0x0000001FU)

/* APPSS_QSPI_CHAR_EXT_CLK_EN */

#define CSL_APP_CTRL_APPSS_QSPI_CHAR_EXT_CLK_EN_APPSS_QSPI_CHAR_EXT_CLK_EN_ENABLE_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_QSPI_CHAR_EXT_CLK_EN_APPSS_QSPI_CHAR_EXT_CLK_EN_ENABLE_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_QSPI_CHAR_EXT_CLK_EN_APPSS_QSPI_CHAR_EXT_CLK_EN_ENABLE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_QSPI_CHAR_EXT_CLK_EN_APPSS_QSPI_CHAR_EXT_CLK_EN_ENABLE_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_QSPI_CHAR_EXT_CLK_EN_RESETVAL                       (0x00000000U)

/* APPSS_QSPI_EXT_CLK_EN */

#define CSL_APP_CTRL_APPSS_QSPI_EXT_CLK_EN_APPSS_QSPI_EXT_CLK_EN_ENABLE_MASK   (0x00000001U)
#define CSL_APP_CTRL_APPSS_QSPI_EXT_CLK_EN_APPSS_QSPI_EXT_CLK_EN_ENABLE_SHIFT  (0x00000000U)
#define CSL_APP_CTRL_APPSS_QSPI_EXT_CLK_EN_APPSS_QSPI_EXT_CLK_EN_ENABLE_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_QSPI_EXT_CLK_EN_APPSS_QSPI_EXT_CLK_EN_ENABLE_MAX    (0x00000001U)

#define CSL_APP_CTRL_APPSS_QSPI_EXT_CLK_EN_RESETVAL                            (0x00000000U)

/* SPI1_SMART_IDLE */

#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ENABLE_MASK               (0x00000001U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ENABLE_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ENABLE_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ENABLE_MAX                (0x00000001U)

#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_MASK                  (0x00000002U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_SHIFT                 (0x00000001U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_RESETVAL              (0x00000000U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_MAX                   (0x00000001U)

#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_AUTO_EN_MASK              (0x00000004U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_AUTO_EN_SHIFT             (0x00000002U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_AUTO_EN_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_AUTO_EN_MAX               (0x00000001U)

#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_MASK               (0x00000008U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_SHIFT              (0x00000003U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_MAX                (0x00000001U)

#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_RAW_MASK              (0x00000010U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_RAW_SHIFT             (0x00000004U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_RAW_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ACK_RAW_MAX               (0x00000001U)

#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_RAW_MASK           (0x00000020U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_RAW_SHIFT          (0x00000005U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_RAW_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_WAKEUP_RAW_MAX            (0x00000001U)

#define CSL_APP_CTRL_SPI1_SMART_IDLE_RESETVAL                                  (0x00000000U)

/* SPI2_SMART_IDLE */

#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ENABLE_MASK               (0x00000001U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ENABLE_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ENABLE_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ENABLE_MAX                (0x00000001U)

#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_MASK                  (0x00000002U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_SHIFT                 (0x00000001U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_RESETVAL              (0x00000000U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_MAX                   (0x00000001U)

#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_AUTO_EN_MASK              (0x00000004U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_AUTO_EN_SHIFT             (0x00000002U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_AUTO_EN_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_AUTO_EN_MAX               (0x00000001U)

#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_MASK               (0x00000008U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_SHIFT              (0x00000003U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_MAX                (0x00000001U)

#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_RAW_MASK              (0x00000010U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_RAW_SHIFT             (0x00000004U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_RAW_RESETVAL          (0x00000000U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_ACK_RAW_MAX               (0x00000001U)

#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_RAW_MASK           (0x00000020U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_RAW_SHIFT          (0x00000005U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_RAW_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_SPI2_SMART_IDLE_SPI2_SMART_IDLE_WAKEUP_RAW_MAX            (0x00000001U)

#define CSL_APP_CTRL_SPI2_SMART_IDLE_RESETVAL                                  (0x00000000U)

/* CAN_SMART_IDLE */

#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ENABLE_MASK                 (0x00000001U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ENABLE_SHIFT                (0x00000000U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ENABLE_RESETVAL             (0x00000000U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ENABLE_MAX                  (0x00000001U)

#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_MASK                    (0x00000002U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_SHIFT                   (0x00000001U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_RESETVAL                (0x00000000U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_MAX                     (0x00000001U)

#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_AUTO_EN_MASK                (0x00000004U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_AUTO_EN_SHIFT               (0x00000002U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_AUTO_EN_RESETVAL            (0x00000000U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_AUTO_EN_MAX                 (0x00000001U)

#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_MASK                 (0x00000008U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_SHIFT                (0x00000003U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_RESETVAL             (0x00000000U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_MAX                  (0x00000001U)

#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_RAW_MASK                (0x00000010U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_RAW_SHIFT               (0x00000004U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_RAW_RESETVAL            (0x00000000U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_ACK_RAW_MAX                 (0x00000001U)

#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_RAW_MASK             (0x00000020U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_RAW_SHIFT            (0x00000005U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_RAW_RESETVAL         (0x00000000U)
#define CSL_APP_CTRL_CAN_SMART_IDLE_CAN_SMART_IDLE_WAKEUP_RAW_MAX              (0x00000001U)

#define CSL_APP_CTRL_CAN_SMART_IDLE_RESETVAL                                   (0x00000000U)

/* LIN_SMART_IDLE */

#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ENABLE_MASK                 (0x00000001U)
#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ENABLE_SHIFT                (0x00000000U)
#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ENABLE_RESETVAL             (0x00000000U)
#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ENABLE_MAX                  (0x00000001U)

#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ACK_MASK                    (0x00000002U)
#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ACK_SHIFT                   (0x00000001U)
#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ACK_RESETVAL                (0x00000000U)
#define CSL_APP_CTRL_LIN_SMART_IDLE_LIN_SMART_IDLE_ACK_MAX                     (0x00000001U)

#define CSL_APP_CTRL_LIN_SMART_IDLE_RESETVAL                                   (0x00000000U)

/* HWASS_CLK_GATE */

#define CSL_APP_CTRL_HWASS_CLK_GATE_HWASS_CLK_GATE_ENABLE_MASK                 (0x00000007U)
#define CSL_APP_CTRL_HWASS_CLK_GATE_HWASS_CLK_GATE_ENABLE_SHIFT                (0x00000000U)
#define CSL_APP_CTRL_HWASS_CLK_GATE_HWASS_CLK_GATE_ENABLE_RESETVAL             (0x00000000U)
#define CSL_APP_CTRL_HWASS_CLK_GATE_HWASS_CLK_GATE_ENABLE_MAX                  (0x00000007U)

#define CSL_APP_CTRL_HWASS_CLK_GATE_RESETVAL                                   (0x00000000U)

/* CFG_TIMEOUT_PCR3 */

#define CSL_APP_CTRL_CFG_TIMEOUT_PCR3_CFG_TIMEOUT_PCR3_VALUE_MASK              (0xFFFFFFFFU)
#define CSL_APP_CTRL_CFG_TIMEOUT_PCR3_CFG_TIMEOUT_PCR3_VALUE_SHIFT             (0x00000000U)
#define CSL_APP_CTRL_CFG_TIMEOUT_PCR3_CFG_TIMEOUT_PCR3_VALUE_RESETVAL          (0x00000FFFU)
#define CSL_APP_CTRL_CFG_TIMEOUT_PCR3_CFG_TIMEOUT_PCR3_VALUE_MAX               (0xFFFFFFFFU)

#define CSL_APP_CTRL_CFG_TIMEOUT_PCR3_RESETVAL                                 (0x00000FFFU)

/* RESERVED0 */

#define CSL_APP_CTRL_RESERVED0_RESERVED0_RESERVED_MASK                         (0x0000FFFFU)
#define CSL_APP_CTRL_RESERVED0_RESERVED0_RESERVED_SHIFT                        (0x00000000U)
#define CSL_APP_CTRL_RESERVED0_RESERVED0_RESERVED_RESETVAL                     (0x00000000U)
#define CSL_APP_CTRL_RESERVED0_RESERVED0_RESERVED_MAX                          (0x0000FFFFU)

#define CSL_APP_CTRL_RESERVED0_RESETVAL                                        (0x00000000U)

/* APPSS_ERRAGG_MASK1 */

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER1_POWER_DOWN_ACCESS_ERR_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER1_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER1_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER1_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER2_POWER_DOWN_ACCESS_ERR_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER2_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER2_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER2_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER3_POWER_DOWN_ACCESS_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER3_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER3_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER3_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER4_POWER_DOWN_ACCESS_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER4_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER4_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER4_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER5_POWER_DOWN_ACCESS_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER5_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER5_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER5_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER6_POWER_DOWN_ACCESS_ERR_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER6_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER6_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER6_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER7_POWER_DOWN_ACCESS_ERR_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER7_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER7_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER7_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER8_POWER_DOWN_ACCESS_ERR_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER8_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER8_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER8_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER9_POWER_DOWN_ACCESS_ERR_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER9_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER9_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER9_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER10_POWER_DOWN_ACCESS_ERR_MASK (0x00000200U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER10_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000009U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER10_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER10_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER11_POWER_DOWN_ACCESS_ERR_MASK (0x00000400U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER11_POWER_DOWN_ACCESS_ERR_SHIFT (0x0000000AU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER11_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER11_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER12_POWER_DOWN_ACCESS_ERR_MASK (0x00000800U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER12_POWER_DOWN_ACCESS_ERR_SHIFT (0x0000000BU)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER12_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_APPSS_ERRAGG_MASK1_CLUSTER12_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_MASK1_RESETVAL                               (0x00000000U)

/* APPSS_ERRAGG_STATUS1 */

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER1_POWER_DOWN_ACCESS_ERR_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER1_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER1_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER1_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER2_POWER_DOWN_ACCESS_ERR_MASK (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER2_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000001U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER2_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER2_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER3_POWER_DOWN_ACCESS_ERR_MASK (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER3_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000002U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER3_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER3_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER4_POWER_DOWN_ACCESS_ERR_MASK (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER4_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000003U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER4_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER4_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER5_POWER_DOWN_ACCESS_ERR_MASK (0x00000010U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER5_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000004U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER5_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER5_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER6_POWER_DOWN_ACCESS_ERR_MASK (0x00000020U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER6_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000005U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER6_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER6_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER7_POWER_DOWN_ACCESS_ERR_MASK (0x00000040U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER7_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000006U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER7_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER7_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER8_POWER_DOWN_ACCESS_ERR_MASK (0x00000080U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER8_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000007U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER8_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER8_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER9_POWER_DOWN_ACCESS_ERR_MASK (0x00000100U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER9_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000008U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER9_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER9_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER10_POWER_DOWN_ACCESS_ERR_MASK (0x00000200U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER10_POWER_DOWN_ACCESS_ERR_SHIFT (0x00000009U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER10_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER10_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER11_POWER_DOWN_ACCESS_ERR_MASK (0x00000400U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER11_POWER_DOWN_ACCESS_ERR_SHIFT (0x0000000AU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER11_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER11_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER12_POWER_DOWN_ACCESS_ERR_MASK (0x00000800U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER12_POWER_DOWN_ACCESS_ERR_SHIFT (0x0000000BU)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER12_POWER_DOWN_ACCESS_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_APPSS_ERRAGG_STATUS1_CLUSTER12_POWER_DOWN_ACCESS_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_ERRAGG_STATUS1_RESETVAL                             (0x00000000U)

/* FORCEHCLKACTIVE */

#define CSL_APP_CTRL_FORCEHCLKACTIVE_FORCEHCLKACTIVE_FORCEHCLKACTIVE_MASK      (0x00000001U)
#define CSL_APP_CTRL_FORCEHCLKACTIVE_FORCEHCLKACTIVE_FORCEHCLKACTIVE_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_FORCEHCLKACTIVE_FORCEHCLKACTIVE_FORCEHCLKACTIVE_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_FORCEHCLKACTIVE_FORCEHCLKACTIVE_FORCEHCLKACTIVE_MAX       (0x00000001U)

#define CSL_APP_CTRL_FORCEHCLKACTIVE_RESETVAL                                  (0x00000000U)

/* APPSS_RAM1_OWRITE_ERR */

#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_APPSS_RAM1_OWRITE_ERR_ERR_MASK      (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_APPSS_RAM1_OWRITE_ERR_ERR_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_APPSS_RAM1_OWRITE_ERR_ERR_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_APPSS_RAM1_OWRITE_ERR_ERR_MAX       (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_RESETVAL                            (0x00000000U)

/* APPSS_RAM1_OWRITE_ERR_ADDR */

#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_ADDR_APPSS_RAM1_OWRITE_ERR_ADDR_ADDRESS_MASK (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_ADDR_APPSS_RAM1_OWRITE_ERR_ADDR_ADDRESS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_ADDR_APPSS_RAM1_OWRITE_ERR_ADDR_ADDRESS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_ADDR_APPSS_RAM1_OWRITE_ERR_ADDR_ADDRESS_MAX (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_RAM1_OWRITE_ERR_ADDR_RESETVAL                       (0x00000000U)

/* APPSS_RAM2_OWRITE_ERR */

#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_APPSS_RAM2_OWRITE_ERR_ERR_MASK      (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_APPSS_RAM2_OWRITE_ERR_ERR_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_APPSS_RAM2_OWRITE_ERR_ERR_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_APPSS_RAM2_OWRITE_ERR_ERR_MAX       (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_RESETVAL                            (0x00000000U)

/* APPSS_RAM2_OWRITE_ERR_ADDR */

#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_ADDR_APPSS_RAM2_OWRITE_ERR_ADDR_ADDRESS_MASK (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_ADDR_APPSS_RAM2_OWRITE_ERR_ADDR_ADDRESS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_ADDR_APPSS_RAM2_OWRITE_ERR_ADDR_ADDRESS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_ADDR_APPSS_RAM2_OWRITE_ERR_ADDR_ADDRESS_MAX (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_RAM2_OWRITE_ERR_ADDR_RESETVAL                       (0x00000000U)

/* APPSS_RAM3_OWRITE_ERR */

#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_APPSS_RAM3_OWRITE_ERR_ERR_MASK      (0x00000001U)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_APPSS_RAM3_OWRITE_ERR_ERR_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_APPSS_RAM3_OWRITE_ERR_ERR_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_APPSS_RAM3_OWRITE_ERR_ERR_MAX       (0x00000001U)

#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_RESETVAL                            (0x00000000U)

/* APPSS_RAM3_OWRITE_ERR_ADDR */

#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_ADDR_APPSS_RAM3_OWRITE_ERR_ADDR_ADDRESS_MASK (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_ADDR_APPSS_RAM3_OWRITE_ERR_ADDR_ADDRESS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_ADDR_APPSS_RAM3_OWRITE_ERR_ADDR_ADDRESS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_ADDR_APPSS_RAM3_OWRITE_ERR_ADDR_ADDRESS_MAX (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_RAM3_OWRITE_ERR_ADDR_RESETVAL                       (0x00000000U)

/* APPSS_SHRD_RAM_OWRITE_ERR */

#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_APPSS_SHRD_RAM_OWRITE_ERR_ERR_MASK (0x00000001U)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_APPSS_SHRD_RAM_OWRITE_ERR_ERR_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_APPSS_SHRD_RAM_OWRITE_ERR_ERR_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_APPSS_SHRD_RAM_OWRITE_ERR_ERR_MAX (0x00000001U)

#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_RESETVAL                        (0x00000000U)

/* APPSS_SHRD_RAM_OWRITE_ERR_ADDR */

#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_ADDRESS_MASK (0xFFFFFFFFU)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_ADDRESS_SHIFT (0x00000000U)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_ADDRESS_RESETVAL (0x00000000U)
#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_ADDRESS_MAX (0xFFFFFFFFU)

#define CSL_APP_CTRL_APPSS_SHRD_RAM_OWRITE_ERR_ADDR_RESETVAL                   (0x00000000U)

/* APPSS_OWRITE_ERR_AGGR */

#define CSL_APP_CTRL_APPSS_OWRITE_ERR_AGGR_APPSS_OWRITE_ERR_AGGR_ERR_MASK      (0x00000001U)
#define CSL_APP_CTRL_APPSS_OWRITE_ERR_AGGR_APPSS_OWRITE_ERR_AGGR_ERR_SHIFT     (0x00000000U)
#define CSL_APP_CTRL_APPSS_OWRITE_ERR_AGGR_APPSS_OWRITE_ERR_AGGR_ERR_RESETVAL  (0x00000000U)
#define CSL_APP_CTRL_APPSS_OWRITE_ERR_AGGR_APPSS_OWRITE_ERR_AGGR_ERR_MAX       (0x00000001U)

#define CSL_APP_CTRL_APPSS_OWRITE_ERR_AGGR_RESETVAL                            (0x00000000U)

/* HW_SPARE_RW6 */

#define CSL_APP_CTRL_HW_SPARE_RW6_HW_SPARE_RW6_HW_SPARE_RW6_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW6_HW_SPARE_RW6_HW_SPARE_RW6_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW6_HW_SPARE_RW6_HW_SPARE_RW6_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW6_HW_SPARE_RW6_HW_SPARE_RW6_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW6_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW7 */

#define CSL_APP_CTRL_HW_SPARE_RW7_HW_SPARE_RW7_HW_SPARE_RW7_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW7_HW_SPARE_RW7_HW_SPARE_RW7_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW7_HW_SPARE_RW7_HW_SPARE_RW7_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW7_HW_SPARE_RW7_HW_SPARE_RW7_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW7_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW8 */

#define CSL_APP_CTRL_HW_SPARE_RW8_HW_SPARE_RW8_HW_SPARE_RW8_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW8_HW_SPARE_RW8_HW_SPARE_RW8_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW8_HW_SPARE_RW8_HW_SPARE_RW8_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW8_HW_SPARE_RW8_HW_SPARE_RW8_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW8_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW9 */

#define CSL_APP_CTRL_HW_SPARE_RW9_HW_SPARE_RW9_HW_SPARE_RW9_MASK               (0xFFFFFFFFU)
#define CSL_APP_CTRL_HW_SPARE_RW9_HW_SPARE_RW9_HW_SPARE_RW9_SHIFT              (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW9_HW_SPARE_RW9_HW_SPARE_RW9_RESETVAL           (0x00000000U)
#define CSL_APP_CTRL_HW_SPARE_RW9_HW_SPARE_RW9_HW_SPARE_RW9_MAX                (0xFFFFFFFFU)

#define CSL_APP_CTRL_HW_SPARE_RW9_RESETVAL                                     (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_APP_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                              (0xFFFFFFFFU)
#define CSL_APP_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                             (0x00000000U)
#define CSL_APP_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                          (0x00000000U)
#define CSL_APP_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                               (0xFFFFFFFFU)

#define CSL_APP_CTRL_LOCK0_KICK0_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_APP_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                              (0xFFFFFFFFU)
#define CSL_APP_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                             (0x00000000U)
#define CSL_APP_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                          (0x00000000U)
#define CSL_APP_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                               (0xFFFFFFFFU)

#define CSL_APP_CTRL_LOCK0_KICK1_RESETVAL                                      (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_APP_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                             (0x00000001U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                            (0x00000000U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                         (0x00000000U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                              (0x00000001U)

#define CSL_APP_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                             (0x00000002U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                            (0x00000001U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                         (0x00000000U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                              (0x00000001U)

#define CSL_APP_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                             (0x00000004U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                            (0x00000002U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                         (0x00000000U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                              (0x00000001U)

#define CSL_APP_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                            (0x00000008U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                           (0x00000003U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                        (0x00000000U)
#define CSL_APP_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                             (0x00000001U)

#define CSL_APP_CTRL_INTR_RAW_STATUS_RESETVAL                                  (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK           (0x00000001U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT          (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX            (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK           (0x00000002U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT          (0x00000001U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX            (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK           (0x00000004U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT          (0x00000002U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL       (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX            (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK          (0x00000008U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT         (0x00000003U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL      (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX           (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLED_STATUS_CLEAR_RESETVAL                        (0x00000000U)

/* INTR_ENABLE */

#define CSL_APP_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                              (0x00000001U)
#define CSL_APP_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                             (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                               (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                              (0x00000002U)
#define CSL_APP_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                             (0x00000001U)
#define CSL_APP_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                               (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                              (0x00000004U)
#define CSL_APP_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                             (0x00000002U)
#define CSL_APP_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                               (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                             (0x00000008U)
#define CSL_APP_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                            (0x00000003U)
#define CSL_APP_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                         (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                              (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_RESETVAL                                      (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                    (0x00000001U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                   (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                    (0x00000002U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                   (0x00000001U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                    (0x00000004U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                   (0x00000002U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                   (0x00000008U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                  (0x00000003U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                    (0x00000001U)

#define CSL_APP_CTRL_INTR_ENABLE_CLEAR_RESETVAL                                (0x00000000U)

/* EOI */

#define CSL_APP_CTRL_EOI_EOI_VECTOR_MASK                                       (0x000000FFU)
#define CSL_APP_CTRL_EOI_EOI_VECTOR_SHIFT                                      (0x00000000U)
#define CSL_APP_CTRL_EOI_EOI_VECTOR_RESETVAL                                   (0x00000000U)
#define CSL_APP_CTRL_EOI_EOI_VECTOR_MAX                                        (0x000000FFU)

#define CSL_APP_CTRL_EOI_RESETVAL                                              (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_APP_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                             (0xFFFFFFFFU)
#define CSL_APP_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                            (0x00000000U)
#define CSL_APP_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                         (0x00000000U)
#define CSL_APP_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                              (0xFFFFFFFFU)

#define CSL_APP_CTRL_FAULT_ADDRESS_RESETVAL                                    (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                         (0x0000003FU)
#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                        (0x00000000U)
#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                     (0x00000000U)
#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                          (0x0000003FU)

#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                           (0x00000040U)
#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                          (0x00000006U)
#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                       (0x00000000U)
#define CSL_APP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                            (0x00000001U)

#define CSL_APP_CTRL_FAULT_TYPE_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                       (0x000000FFU)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                      (0x00000000U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                   (0x00000000U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                        (0x000000FFU)

#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                      (0x000FFF00U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                     (0x00000008U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                  (0x00000000U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                       (0x00000FFFU)

#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                          (0xFFF00000U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                         (0x00000014U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                      (0x00000000U)
#define CSL_APP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                           (0x00000FFFU)

#define CSL_APP_CTRL_FAULT_ATTR_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_CLEAR */

#define CSL_APP_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                                (0x00000001U)
#define CSL_APP_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                               (0x00000000U)
#define CSL_APP_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                            (0x00000000U)
#define CSL_APP_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                                 (0x00000001U)

#define CSL_APP_CTRL_FAULT_CLEAR_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
