/*
 *  Copyright (C) 2020-23 Texas Instruments Incorporated
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

#ifndef CSLR_SOC_DEFINES_H_
#define CSLR_SOC_DEFINES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor CSL_CoreID
 * \name Core ID's of core or CPUs present on this SOC
 *
 * @{
 */
#define CSL_CORE_ID_M4FSS0_0                          (0U)
#define CSL_CORE_ID_MAX                               (1U)
/** @} */

/** @brief Number of UART instances */
#define CSL_APSS_UART_PER_CNT                         (2U)

/** @brief Number of MIBSPI instances */
#define CSL_APPSS_MCSPI_PER_CNT                       (1U)

/** @brief Number of I2C instances */
#define CSL_APPSS_I2C_CNT                             (1U)

/** @brief Number of LIN instances */
#define CSL_APPSS_LIN_CNT                             (1U)

/** @brief Number of QSPI instances */
#define CSL_APPSS_QSPI_CNT                            (1U)

/** @brief Number of MCAN instances */
#define CSL_APPSS_MCAN_CNT                            (1U)

/** @brief Number of PWM instances */
#define CSL_APPSS_PWM_CNT                             (1U)

/*
 * This represents the maximum supported in a SOC across all instances of EDMA
 */
/** @brief Number of DMA Channels */
#define SOC_EDMA_NUM_DMACH                            (64U)
/** @brief Number of QDMA Channels */
#define SOC_EDMA_NUM_QDMACH                           (8U)
/** @brief Number of PaRAM Sets available */
#define SOC_EDMA_NUM_PARAMSETS                        (128U)
/** @brief Number of Event Queues available */
#define SOC_EDMA_NUM_EVQUE                            (2U)
/** @brief Support for Channel to PaRAM Set mapping */
#define SOC_EDMA_CHMAPEXIST                           (1U)
/** @brief Number of EDMA Regions */
#define SOC_EDMA_NUM_REGIONS                          (8U)
/** @brief Support for Memory Protection */
#define SOC_EDMA_MEMPROTECT                           (1U)

/**
 * \brief  MCAN Maximum Message RAM words
 */
#define MCAN_MSG_RAM_MAX_WORD_COUNT                   (4352U)

/*! @brief  Maximum number of Rx Dma buffers. */
#define MCAN_MAX_RX_DMA_BUFFERS                       (3U)

/*! @brief  Maximum number of Tx Dma buffers. */
#define MCAN_MAX_TX_DMA_BUFFERS                       (3U)

/* ESM number of groups */
#define ESM_NUM_GROUP_MAX                             (3U)
#define ESM_NUM_INTR_PER_GROUP                        (128U)

/** @brief DSP TPCC A EVENT MAP */
#define EDMA_APPSS_TPCC_A_EVT_SPI1_DMA_RX_REQ         0
#define EDMA_APPSS_TPCC_A_EVT_SPI1_DMA_TX_REQ         1
#define EDMA_APPSS_TPCC_A_EVT_SPI2_DMA_RX_REQ         2
#define EDMA_APPSS_TPCC_A_EVT_SPI2_DMA_TX_REQ         3
#define EDMA_APPSS_TPCC_A_EVT_SCI1_DMA_RX_REQ         4
#define EDMA_APPSS_TPCC_A_EVT_SCI1_DMA_TX_REQ         5
#define EDMA_APPSS_TPCC_A_EVT_LIN_DMA_RX_REQ          6
#define EDMA_APPSS_TPCC_A_EVT_LIN_DMA_TX_REQ          7
#define EDMA_APPSS_TPCC_A_EVT_MCAN_DMA_REQ0           8
#define EDMA_APPSS_TPCC_A_EVT_MCAN_DMA_REQ1           9
#define EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT1            10
#define EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT2            11
#define EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT3            12
#define EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT4            13
#define EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT5            14
#define EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT6            15
#define EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT7            16
#define EDMA_APPSS_TPCC_A_EVT_I2C_DMA_REQ0            17
#define EDMA_APPSS_TPCC_A_EVT_I2C_DMA_REQ1            18
#define EDMA_APPSS_TPCC_A_GIO_INT0                    19
#define EDMA_APPSS_TPCC_A_GIO_INT1                    20
#define EDMA_APPSS_TPCC_A_APP_RTI1_DMA_REQ0           21
#define EDMA_APPSS_TPCC_A_APP_RTI1_DMA_REQ1           22
#define EDMA_APPSS_TPCC_A_APP_RTI1_DMA_REQ2           23
#define EDMA_APPSS_TPCC_A_APP_RTI1_DMA_REQ3           24
#define EDMA_APPSS_TPCC_A_APP_RTI2_DMA_REQ0           25
#define EDMA_APPSS_TPCC_A_APP_RTI2_DMA_REQ1           26
#define EDMA_APPSS_TPCC_A_APP_RTI2_DMA_REQ2           27
#define EDMA_APPSS_TPCC_A_APP_RTI2_DMA_REQ3           28
#define EDMA_APPSS_TPCC_A_MCRC_DMA_REQ0               29
#define EDMA_APPSS_TPCC_A_MCRC_DMA_REQ1               30
#define EDMA_APPSS_TPCC_A_QSPI_DMA_REQ                31
#define EDMA_APPSS_TPCC_A_PWM_DMA_REQ0                32
#define EDMA_APPSS_TPCC_A_PWM_DMA_REQ1                33
#define EDMA_APPSS_TPCC_A_SCI2_DMA_RX_REQ             34
#define EDMA_APPSS_TPCC_A_SCI2_DMA_TX_REQ             35
#define EDMA_APPSS_TPCC_A_FRAMETIMER_FRAME_START      36
#define EDMA_APPSS_TPCC_A_CHIP_AVAIL_IRQ              37
#define EDMA_APPSS_TPCC_A_CHIRPTIMER_CHIRP_END        38
#define EDMA_APPSS_TPCC_A_CHIRPTIMER_CHIRP_START      39
#define EDMA_APPSS_TPCC_A_CHIRPTIMER_FRAME_END        40
#define EDMA_APPSS_TPCC_A_ADC_VALID_START             41
#define EDMA_APPSS_TPCC_A_DTHE_SHA_DMA_REQ0           42
#define EDMA_APPSS_TPCC_A_DTHE_SHA_DMA_REQ1           43
#define EDMA_APPSS_TPCC_A_DTHE_SHA_DMA_REQ2           44
#define EDMA_APPSS_TPCC_A_DTHE_SHA_DMA_REQ3           45
#define EDMA_APPSS_TPCC_A_DTHE_SHA_DMA_REQ4           46
#define EDMA_APPSS_TPCC_A_DTHE_SHA_DMA_REQ5           47
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ0           48
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ1           49
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ2           50
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ3           51
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ4           52
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ5           53
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ6           54
#define EDMA_APPSS_TPCC_A_DTHE_AES_DMA_REQ7           55
#define EDMA_APPSS_TPCC_A_EVT_FREE_0                  56
#define EDMA_APPSS_TPCC_A_FEC_INTR0                   57
#define EDMA_APPSS_TPCC_A_FEC_INTR1                   58
#define EDMA_APPSS_TPCC_A_FEC_INTR2                   59
#define EDMA_APPSS_TPCC_A_FEC_INTR3                   60
#define EDMA_APPSS_TPCC_A_EVT_FREE_1                  61
#define EDMA_APPSS_TPCC_A_SP1_SPI2_DMA_RX_REQ         62
#define EDMA_APPSS_TPCC_A_SP1_SPI2_DMA_TX_REQ         63

/** @brief DSP TPCC B EVENT MAP */
#define EDMA_APPSS_TPCC_B_EVT_FRAMETIMER_FRAME_START  0
#define EDMA_APPSS_TPCC_B_EVT_CHIRP_AVAIL_IRQ         1
#define EDMA_APPSS_TPCC_B_EVT_CHIRPTIMER_CHIRP_END    2
#define EDMA_APPSS_TPCC_B_EVT_CHIRPTIMER_CHIRP_START  3
#define EDMA_APPSS_TPCC_B_EVT_CHIRPTIMER_FRAME_END    4
#define EDMA_APPSS_TPCC_B_EVT_ADC_VALID_START         5
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ0            6
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ1            7
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ2            8
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ3            9
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ4            10
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ5            11
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ6            12
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ7            13
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ8            14
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ9            15
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ10           16
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ11           17
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ12           18
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ13           19
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ14           20
#define EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ15           21
#define EDMA_APPSS_TPCC_B_EVT_HWA_LOOP_INT            22
#define EDMA_APPSS_TPCC_B_EVT_HWA_PARAMDONE_INT       23
#define EDMA_APPSS_TPCC_B_EVT_SPI1_DMA_RX_REQ         24
#define EDMA_APPSS_TPCC_B_EVT_SPI1_DMA_TX_REQ         25
#define EDMA_APPSS_TPCC_B_EVT_SPI2_DMA_RX_REQ         26
#define EDMA_APPSS_TPCC_B_EVT_SPI2_DMA_TX_REQ         27
#define EDMA_APPSS_TPCC_B_EVT_FREE_0                  28
#define EDMA_APPSS_TPCC_B_EVT_FREE_1                  29
#define EDMA_APPSS_TPCC_B_EVT_FREE_2                  30
#define EDMA_APPSS_TPCC_B_EVT_FREE_3                  31
#define EDMA_APPSS_TPCC_B_EVT_FREE_4                  32
#define EDMA_APPSS_TPCC_B_EVT_FREE_5                  33
#define EDMA_APPSS_TPCC_B_EVT_FREE_6                  34
#define EDMA_APPSS_TPCC_B_EVT_FREE_7                  35
#define EDMA_APPSS_TPCC_B_EVT_FREE_8                  36
#define EDMA_APPSS_TPCC_B_EVT_FREE_9                  37
#define EDMA_APPSS_TPCC_B_EVT_FREE_10                 38
#define EDMA_APPSS_TPCC_B_EVT_FREE_11                 39
#define EDMA_APPSS_TPCC_B_EVT_FREE_12                 40
#define EDMA_APPSS_TPCC_B_EVT_FREE_13                 41
#define EDMA_APPSS_TPCC_B_EVT_FREE_14                 42
#define EDMA_APPSS_TPCC_B_EVT_FREE_15                 43
#define EDMA_APPSS_TPCC_B_EVT_FREE_16                 44
#define EDMA_APPSS_TPCC_B_EVT_FREE_17                 45
#define EDMA_APPSS_TPCC_B_EVT_FREE_18                 46
#define EDMA_APPSS_TPCC_B_EVT_FREE_19                 47
#define EDMA_APPSS_TPCC_B_EVT_FREE_20                 48
#define EDMA_APPSS_TPCC_B_EVT_FREE_21                 49
#define EDMA_APPSS_TPCC_B_EVT_FREE_22                 50
#define EDMA_APPSS_TPCC_B_EVT_FREE_23                 51
#define EDMA_APPSS_TPCC_B_EVT_FREE_24                 52
#define EDMA_APPSS_TPCC_B_EVT_FREE_25                 53
#define EDMA_APPSS_TPCC_B_EVT_FREE_26                 54
#define EDMA_APPSS_TPCC_B_EVT_FREE_27                 55
#define EDMA_APPSS_TPCC_B_EVT_FREE_28                 56
#define EDMA_APPSS_TPCC_B_EVT_FREE_29                 57
#define EDMA_APPSS_TPCC_B_EVT_FREE_30                 58
#define EDMA_APPSS_TPCC_B_EVT_FREE_31                 59
#define EDMA_APPSS_TPCC_B_EVT_FREE_32                 60
#define EDMA_APPSS_TPCC_B_EVT_FREE_33                 61
#define EDMA_APPSS_TPCC_B_EVT_FREE_34                 62
#define EDMA_APPSS_TPCC_B_EVT_FREE_35                 63

#define EDMA_APPSS_TPCC_A_NUM_PARAM_SETS  (128U)
#define EDMA_APPSS_TPCC_A_NUM_DMA_CHANS   (64U)
#define EDMA_APPSS_TPCC_A_NUM_TC          (2U)

#define EDMA_APPSS_TPCC_B_NUM_PARAM_SETS  (128U)
#define EDMA_APPSS_TPCC_B_NUM_DMA_CHANS   (64U)
#define EDMA_APPSS_TPCC_B_NUM_TC          (2U)

#define EDMA_TPCC_ERRAGG_TPCC_EERINT__POS   (0U)
#define EDMA_TPCC_INTAGG_TPCC_INTG__POS     (0U)
#define EDMA_TPCC_ERRAGG_TPTC_MIN_ERR__POS  (2U) /* position of the lowest TC Id, others are higher */

#define EDMA_APPSS_NUM_CC        4

#define EDMA_APPSS_MAX_NUM_TC     CSL_MAX(EDMA_APPSS_TPCC_A_NUM_TC, \
                                CSL_MAX(EDMA_APPSS_TPCC_B_NUM_TC, \
                                CSL_MAX(EDMA_APPSS_TPCC_C_NUM_TC, \
                                EDMA_RCSS_TPCC_A_NUM_TC)))

#define EDMA_MSS_NUM_CC        6

#define EDMA_MSS_MAX_NUM_TC     CSL_MAX(EDMA_MSS_TPCC_A_NUM_TC, \
                                CSL_MAX(EDMA_MSS_TPCC_B_NUM_TC, \
                                CSL_MAX(EDMA_APPSS_TPCC_A_NUM_TC, \
                                CSL_MAX(EDMA_APPSS_TPCC_B_NUM_TC, \
                                CSL_MAX(EDMA_APPSS_TPCC_C_NUM_TC, \
                                EDMA_RCSS_TPCC_A_NUM_TC)))))

/***********************************************************************
 * Peripheral number of instance definition
 ***********************************************************************/
#define HWA_NUM_INSTANCES               (1U)

/*! \brief number of HWA memory banks */
#define SOC_HWA_NUM_MEM_BANKS           (4U)
/*! \brief number of HWA parameter sets */
#define SOC_HWA_NUM_PARAM_SETS          (32U)
/*! \brief number of HWA MDA channels */
#define SOC_HWA_NUM_DMA_CHANNEL         (16U)
/*! \brief number of HWA memory size in bytes */
#define SOC_HWA_MEM_SIZE                (CSL_APP_HWA_BANK_SIZE * SOC_HWA_NUM_MEM_BANKS)

/***********************************************************************
 * HWA Hardware trigger source definitions
 ***********************************************************************/
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_0_LINE_END  (0U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_1_LINE_END  (1U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_2_LINE_END  (2U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_3_LINE_END  (3U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_4_LINE_END  (4U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_5_LINE_END  (5U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_6_LINE_END  (6U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_7_LINE_END  (7U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_FRAME_START_0       (8U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_FRAME_START_1       (9U)

#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_0_LINE_END  (10U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_1_LINE_END  (11U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_2_LINE_END  (12U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_3_LINE_END  (13U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_4_LINE_END  (14U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_5_LINE_END  (15U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_6_LINE_END  (16U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_7_LINE_END  (17U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_FRAME_START_0       (18U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_FRAME_START_1       (19U)


/***********************************************************************
 * MSS - CLOCK settings
 ***********************************************************************/
 /* Sys_vclk : 160MHz */
#define APPSS_SYS_VCLK                  160000000U


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*! @brief R5F to Hardware Accelerator address translation macro. */
//#define ADDR_TRANSLATE_CPU_TO_HWA(x)  (uint16_t)(((uint32_t)(x) - SOC_XWR18XX_MSS_HWA_MEM0_BASE_ADDRESS) & 0x0000FFFFU)


/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* CSLR_SOC_DEFINES_H_ */
