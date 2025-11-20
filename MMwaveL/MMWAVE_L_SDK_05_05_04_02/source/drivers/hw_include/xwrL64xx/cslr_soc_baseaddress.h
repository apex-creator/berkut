/********************************************************************
*
* SOC memory map header file
*
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
#ifndef CSLR_SOC_BASEADDRESS_H
#define CSLR_SOC_BASEADDRESS_H

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Global addresses in unified address space */
#define CSL_APP_CPU_RAM_U_BASE                (0x00400000U)
#define CSL_APP_RAM_U_BASE                    (0x22400000U)
#define CSL_APP_CPU_SHARED_RAM_U_BASE         (0x00480000U)
#define CSL_APP_SHARED_RAM_U_BASE             (0x22480000U)
#define CSL_FEC_RAM_U_BASE                    (0x21200000U)
#define CSL_FEC_SHARED_RAM_U_BASE             (0x21208000U)
#define CSL_FEC_CTRL_U_BASE                   (0x52000000U)
#define CSL_APP_HWA_ADCBUF_RD_U_BASE          (0x55060000U)
#define CSL_APP_HWA_ADCBUF_WR_U_BASE          (0x55070000U)
#define CSL_APP_HWA_RAM_U_BASE                (0x60000000U)
#define CSL_APP_NVIC_U_BASE                   (0xE000E100U)
#define CSL_APP_TPCC_A_U_BASE                 (0x56000000U)
#define CSL_APP_TPCC_A_REG_U_BASE             (0x56001000U)
#define CSL_APP_TPCC_A_PARAM_U_BASE           (0x56004000U)
#define CSL_APP_CTRL_U_BASE                   (0x56060000U)
#define CSL_APP_HWA_ADCBUF_CTRL_U_BASE        (0x56080000U)
#define CSL_APP_TPCC_B_U_BASE                 (0x55080000U)
#define CSL_APP_TPCC_B_REG_U_BASE             (0x55081000U)
#define CSL_APP_TPCC_B_PARAM_U_BASE           (0x55084000U)
#define CSL_APP_RTIA_U_BASE                   (0x56F7F000U)
#define CSL_APP_WD_U_BASE                     (0x56F7F400U)
#define CSL_APP_ESM_U_BASE                    (0x56F7FC00U)
#define CSL_APP_UART0_U_BASE                  (0x53F7F000U)
#define CSL_APP_UART1_U_BASE                  (0x57F7F000U)
#define CSL_APP_I2C_U_BASE                    (0x57F7F800U)
#define CSL_APP_GIO_U_BASE                    (0x5AF7FC00U)
#define CSL_APP_HWA_PARAM_U_BASE              (0x55014000U)
#define CSL_APP_HWA_DMA0_U_BASE               (0x55000000U)
#define CSL_APP_HWA_DMA1_U_BASE               (0x55008000U)
#define CSL_APP_HWA_WINDOW_RAM_U_BASE         (0x55018000U)
#define CSL_MCU_MCSPI0_CFG_BASE               (0x53F7F400U)
#define CSL_MCU_MCSPI1_CFG_BASE               (0x57F7F400U)
#define CSL_APP_ETPWM_U_BASE                  (0x57F7FC00U)
#define CSL_APP_CFG_QSPI_U_BASE               (0x78000000U)
#define CSL_APP_QSPI_EXT_FLASH_U_BASE         (0x70000000U)
#define CSL_APP_MCAN_MSG_RAM_U_BASE           (0x53020000U)
#define CSL_APP_MCAN_MSG_RAM_U_SIZE           (0x0011000U)
#define CSL_APP_MCAN_ECC_U_BASE               (0x53F7FC00U)
#define CSL_APP_MCAN_CFG_U_BASE               (0x53F7F800U)
#define CSL_APP_LIN_U_BASE                    (0x53000000U)
#define CSL_APP_CRC_U_BASE                    (0x54020000U)
#define CSL_APP_CPU_RAM_SIZE                  (0x00080000U)
#define CSL_APP_CPU_SHARED_RAM_U_SIZE         (0x00040000U)

//#define CSL_APP_HWA_PARAM_U_BASE              (0x55014000U)
#define CSL_APP_HWA_CFG_U_BASE                (0x55010000U)
//#define CSL_APP_HWA_WINDOW_RAM_U_BASE         (0x55018000U)
#define CSL_APP_HWA_WINDOW_RAM_U_SIZE         (0x0001000U)
#define CSL_APP_HWA_MC_PING_RAM_U_BASE        (0x5501A000U)
#define CSL_APP_HWA_MC_PING_RAM_U_SIZE        (0x0000800U)
#define CSL_APP_HWA_MC_PONG_RAM_U_BASE        (0x5501C000U)
#define CSL_APP_HWA_MC_PONG_RAM_U_SIZE        (0x0000800U)
//#define CSL_APP_HWA_DMA0_U_BASE               (0x55000000U)
#define CSL_APP_HWA_DMA0_U_SIZE               (0x00008000U)
//#define CSL_APP_HWA_DMA1_U_BASE               (0x55008000U)
#define CSL_APP_HWA_DMA1_U_SIZE               (0x00008000U)

#define CSL_APP_TPTC_A0_U_BASE                (0x54000000U)
#define CSL_APP_TPTC_A1_U_BASE                (0x54010000U)
#define CSL_APP_TPTC_B0_U_BASE                (0x55020000U)
#define CSL_APP_TPTC_B1_U_BASE                (0x55040000U)

/* System control registers base address */
#define CSL_TOPSS_CTRL_U_BASE                 (0x5B020000U)
#define CSL_APP_RCM_U_BASE                    (0x56040000U)
#define CSL_TOP_IO_MUX_U_BASE                 (0x5A000000U)
#define CSL_APP_UART_0_U_BASE                 (0x53F7F000U)
#define CSL_TOP_PRCM_U_BASE                   (0x5A040000U)
#define CSL_TOP_EFUSE_U_BASE                  (0x5A020000U)
#define CSL_FRAME_COUNTER_U_BASE              (0x5B000000U)
#define CSL_PLLDIG_CTRL_U_BASE                (0x5B040000U)
#define CSL_APLL_CTRL_U_BASE                  (0x5B060000U)
#define CSL_EFUSE_FARM_U_BASE                 (0x5C060000U)
/*************************************** ***************************/


/* HWA DMA Bank definitions */
#define CSL_APP_HWA_BANK_SIZE   (0x4000U)

#define CSL_APP_HWA_DMA0_RAM_BANK0_BASE     CSL_APP_HWA_DMA0_U_BASE
#define CSL_APP_HWA_DMA0_RAM_BANK1_BASE     (CSL_APP_HWA_DMA0_RAM_BANK0_BASE + CSL_APP_HWA_BANK_SIZE)
#define CSL_APP_HWA_DMA0_RAM_BANK2_BASE     (CSL_APP_HWA_DMA0_RAM_BANK1_BASE + CSL_APP_HWA_BANK_SIZE)
#define CSL_APP_HWA_DMA0_RAM_BANK3_BASE     (CSL_APP_HWA_DMA0_RAM_BANK2_BASE + CSL_APP_HWA_BANK_SIZE)

#define CSL_APP_HWA_DMA1_RAM_BANK0_BASE     CSL_APP_HWA_DMA1_U_BASE
#define CSL_APP_HWA_DMA1_RAM_BANK1_BASE     (CSL_APP_HWA_DMA1_RAM_BANK0_BASE + CSL_APP_HWA_BANK_SIZE)
#define CSL_APP_HWA_DMA1_RAM_BANK2_BASE     (CSL_APP_HWA_DMA1_RAM_BANK1_BASE + CSL_APP_HWA_BANK_SIZE)
#define CSL_APP_HWA_DMA1_RAM_BANK3_BASE     (CSL_APP_HWA_DMA1_RAM_BANK2_BASE + CSL_APP_HWA_BANK_SIZE)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_BASEADDRESS_H_ */
