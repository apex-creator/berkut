/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file     mcan.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of MCAN.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <drivers/mcan/v0/lld/mcan_lld.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/AddrTranslateP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief  MCAN MSG RAM BANK number for ECC AGGR.
 */
#define MCANSS_MSG_RAM_NUM                                       (1U)

/**
 * \brief  Maximum Number of Rx Buffers.
 */
#define MCANSS_RX_BUFFER_MAX                                     (64U)

/**
 * \brief  Maximum Number of Tx Buffers.
 */
#define MCANSS_TX_BUFFER_MAX                                     (32U)

/**
 * \brief  Macro for standard Message ID filter.
 */
#define MCANSS_STD_ID_FILTER_SIZE_WORDS                          (1U)

/**
 * \brief  Macro for extended Message ID filter.
 */
#define MCANSS_EXT_ID_FILTER_SIZE_WORDS                          (2U)

/**
 * \brief  Macro for Tx Event FIFO element size.
 */
#define MCANSS_TX_EVENT_FIFO_SIZE_WORDS                          (2U)


/**
 * \brief  Macro for Interrupt Line enable mask.
 */
#define MCANSS_INTR_LINE_EN_MASK   ((CSL_APP_CANCFG_ILE_EINT0_MASK | CSL_APP_CANCFG_ILE_EINT1_MASK))

/**
 * \brief  Mask and shift for Tx Buffers elements.
 */
#define MCANSS_TX_BUFFER_ELEM_ID_SHIFT                           (0U)
#define MCANSS_TX_BUFFER_ELEM_ID_MASK                            (0x1FFFFFFFU)
#define MCANSS_TX_BUFFER_ELEM_RTR_SHIFT                          (29U)
#define MCANSS_TX_BUFFER_ELEM_RTR_MASK                           (0x20000000U)
#define MCANSS_TX_BUFFER_ELEM_XTD_SHIFT                          (30U)
#define MCANSS_TX_BUFFER_ELEM_XTD_MASK                           (0x40000000U)
#define MCANSS_TX_BUFFER_ELEM_ESI_SHIFT                          (31U)
#define MCANSS_TX_BUFFER_ELEM_ESI_MASK                           (0x80000000U)
#define MCANSS_TX_BUFFER_ELEM_DLC_SHIFT                          (16U)
#define MCANSS_TX_BUFFER_ELEM_DLC_MASK                           (0x000F0000U)
#define MCANSS_TX_BUFFER_ELEM_BRS_SHIFT                          (20U)
#define MCANSS_TX_BUFFER_ELEM_BRS_MASK                           (0x00100000U)
#define MCANSS_TX_BUFFER_ELEM_FDF_SHIFT                          (21U)
#define MCANSS_TX_BUFFER_ELEM_FDF_MASK                           (0x00200000U)
#define MCANSS_TX_BUFFER_ELEM_EFC_SHIFT                          (23U)
#define MCANSS_TX_BUFFER_ELEM_EFC_MASK                           (0x00800000U)
#define MCANSS_TX_BUFFER_ELEM_MM_SHIFT                           (24U)
#define MCANSS_TX_BUFFER_ELEM_MM_MASK                            (0xFF000000U)

/**
 * \brief  Mask and shift for Rx Buffers elements.
 */
#define MCANSS_RX_BUFFER_ELEM_ID_SHIFT                           (0U)
#define MCANSS_RX_BUFFER_ELEM_ID_MASK                            (0x1FFFFFFFU)
#define MCANSS_RX_BUFFER_ELEM_RTR_SHIFT                          (29U)
#define MCANSS_RX_BUFFER_ELEM_RTR_MASK                           (0x20000000U)
#define MCANSS_RX_BUFFER_ELEM_XTD_SHIFT                          (30U)
#define MCANSS_RX_BUFFER_ELEM_XTD_MASK                           (0x40000000U)
#define MCANSS_RX_BUFFER_ELEM_ESI_SHIFT                          (31U)
#define MCANSS_RX_BUFFER_ELEM_ESI_MASK                           (0x80000000U)
#define MCANSS_RX_BUFFER_ELEM_RXTS_SHIFT                         (0U)
#define MCANSS_RX_BUFFER_ELEM_RXTS_MASK                          (0x0000FFFFU)
#define MCANSS_RX_BUFFER_ELEM_DLC_SHIFT                          (16U)
#define MCANSS_RX_BUFFER_ELEM_DLC_MASK                           (0x000F0000U)
#define MCANSS_RX_BUFFER_ELEM_BRS_SHIFT                          (20U)
#define MCANSS_RX_BUFFER_ELEM_BRS_MASK                           (0x00100000U)
#define MCANSS_RX_BUFFER_ELEM_FDF_SHIFT                          (21U)
#define MCANSS_RX_BUFFER_ELEM_FDF_MASK                           (0x00200000U)
#define MCANSS_RX_BUFFER_ELEM_FIDX_SHIFT                         (24U)
#define MCANSS_RX_BUFFER_ELEM_FIDX_MASK                          (0x7F000000U)
#define MCANSS_RX_BUFFER_ELEM_ANMF_SHIFT                         (31U)
#define MCANSS_RX_BUFFER_ELEM_ANMF_MASK                          (0x80000000U)

/**
 * \brief  Mask and shift for Standard Message ID Filter Elements.
 */
#define MCANSS_STD_ID_FILTER_SFID2_SHIFT                         (0U)
#define MCANSS_STD_ID_FILTER_SFID2_MASK                          (0x000003FFU)
#define MCANSS_STD_ID_FILTER_SFID1_SHIFT                         (16U)
#define MCANSS_STD_ID_FILTER_SFID1_MASK                          (0x03FF0000U)
#define MCANSS_STD_ID_FILTER_SFEC_SHIFT                          (27U)
#define MCANSS_STD_ID_FILTER_SFEC_MASK                           (0x38000000U)
#define MCANSS_STD_ID_FILTER_SFT_SHIFT                           (30U)
#define MCANSS_STD_ID_FILTER_SFT_MASK                            (0xC0000000U)

/**
 * \brief  Extended Message ID Filter Element.
 */
#define MCANSS_EXT_ID_FILTER_EFID2_SHIFT                        (0U)
#define MCANSS_EXT_ID_FILTER_EFID2_MASK                         (0x1FFFFFFFU)
#define MCANSS_EXT_ID_FILTER_EFID1_SHIFT                        (0U)
#define MCANSS_EXT_ID_FILTER_EFID1_MASK                         (0x1FFFFFFFU)
#define MCANSS_EXT_ID_FILTER_EFEC_SHIFT                         (29U)
#define MCANSS_EXT_ID_FILTER_EFEC_MASK                          (0xE0000000U)
#define MCANSS_EXT_ID_FILTER_EFT_SHIFT                          (30U)
#define MCANSS_EXT_ID_FILTER_EFT_MASK                           (0xC0000000U)

/**
 * \brief  Mask and shift for Tx Event FIFO elements.
 */
#define MCANSS_TX_EVENT_FIFO_ELEM_ID_SHIFT                      (0U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ID_MASK                       (0x1FFFFFFFU)
#define MCANSS_TX_EVENT_FIFO_ELEM_RTR_SHIFT                     (29U)
#define MCANSS_TX_EVENT_FIFO_ELEM_RTR_MASK                      (0x20000000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_XTD_SHIFT                     (30U)
#define MCANSS_TX_EVENT_FIFO_ELEM_XTD_MASK                      (0x40000000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ESI_SHIFT                     (31U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ESI_MASK                      (0x80000000U)

#define MCANSS_TX_EVENT_FIFO_ELEM_TXTS_SHIFT                    (0U)
#define MCANSS_TX_EVENT_FIFO_ELEM_TXTS_MASK                     (0x0000FFFFU)
#define MCANSS_TX_EVENT_FIFO_ELEM_DLC_SHIFT                     (16U)
#define MCANSS_TX_EVENT_FIFO_ELEM_DLC_MASK                      (0x000F0000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_BRS_SHIFT                     (20U)
#define MCANSS_TX_EVENT_FIFO_ELEM_BRS_MASK                      (0x00100000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_FDF_SHIFT                     (21U)
#define MCANSS_TX_EVENT_FIFO_ELEM_FDF_MASK                      (0x00200000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ET_SHIFT                      (22U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ET_MASK                       (0x00C00000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_MM_SHIFT                      (24U)
#define MCANSS_TX_EVENT_FIFO_ELEM_MM_MASK                       (0xFF000000U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct MCAN_OffsetAddr_t {
    int32_t     mcanSsOffset;
    int32_t     mcanCfgOffset;
} MCAN_OffsetAddr;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   This API will unblock write access to write protected registers.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 *
 * \return  None.
 */
static void MCAN_writeProtectedRegAccessUnlock(uint32_t baseAddr);

/**
 * \brief   This API will block write access to write protected registers.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 *
 * \return  None.
 */
static void MCAN_writeProtectedRegAccessLock(uint32_t baseAddr);

/**
 * \brief   This API will load the register from ECC memory bank.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   regOffset       Offset of the register to read.
 *
 * \return  None.
 */
static void MCAN_eccLoadRegister(uint32_t baseAddr, uint32_t regOffset);

/**
 * \brief   This API will read the message object from Message RAM.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_RxBufElement.
 *
 * \return  None.
 */
static void MCAN_readMsg(uint32_t           baseAddr,
                         uint32_t           elemAddr,
                         MCAN_RxBufElement *elem);

/**
 * \brief   This API will read the message object from Message RAM. This uses
 *          the MCAN_RxBufElementNoCpy structure which has the data field as
 *          a pointer. Needed to avoid one copy of the data/payload in the MCAL
 *          CAN driver and replace it by a pointer assignment.
 *          Note that as the data is a pointer here hence corruption of data is
 *          possible in case you exceed the payload size.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_RxBufElementNoCpy.
 *
 * \return  None.
 */
static void MCAN_readMsgNoCpy(uint32_t           baseAddr,
                         uint32_t           elemAddr,
                         MCAN_RxBufElementNoCpy *elem);

/**
 * \brief   This API will write the message object to Message RAM.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_TxBufElement.
 *
 * \return  None.
 */
static void MCAN_writeMsg(uint32_t                 baseAddr,
                          uint32_t                 elemAddr,
                          const MCAN_TxBufElement *elem);

/**
 * \brief   This API will write the message object to Message RAM. This uses
 *          the MCAN_TxBufElementNoCpy structure which has the data field as
 *          a pointer. Needed to avoid one copy of the data/payload in the MCAL
 *          CAN driver and replace it by a pointer assignment.
 *          Note that as the data is a pointer here hence corruption of data is
 *          possible in case you exceed the payload size.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_TxBufElementNoCpy.
 *
 * \return  None.
 */
static void MCAN_writeMsgNoCpy(uint32_t                 baseAddr,
                          uint32_t                 elemAddr,
                          const MCAN_TxBufElementNoCpy *elem);

/**
 * \brief   This API will return ECC Configuration Register Base Address.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 *
 * \return  offset          ECC Configuration Register Base Address.
 *
 */
static uint32_t MCAN_getECCRegionAddr(uint32_t baseAddr);

/**
 * \brief   This API will return MCAN Message RAM Base Address.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 *
 * \return  offset          Message RAM Base Address.
 *
 */
static const MCAN_OffsetAddr* MCAN_getOffsetAddr(uint32_t baseAddr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* User will provide only the MsgRam Base address.
 * All Offsets are calculated compared to MsgRam Base address. */

static const MCAN_OffsetAddr gMcanOffsetAddr =
{
    .mcanSsOffset       = ((int32_t) CSL_APP_MCAN_CFG_U_BASE                   - (int32_t) CSL_APP_MCAN_MSG_RAM_U_BASE),
    .mcanCfgOffset      = ((int32_t) CSL_APP_MCAN_CFG_U_BASE                   - (int32_t) CSL_APP_MCAN_MSG_RAM_U_BASE),
};

/* payload depending on 'dlc'  field. */
static const uint32_t gDataSize[16]  = {0,  1,  2,  3,  4,  5,  6, 7, 8,
                                        12, 16, 20, 24, 32, 48, 64};
/* message object stored in Message RAM. */
static const uint32_t gMsgObjSize[8] = {4, 5, 6, 7, 8, 10, 14, 18};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static uint32_t MCAN_SsAddr(uint32_t baseAddr);
static uint32_t MCAN_SsAddr(uint32_t baseAddr)
{
    int64_t mcanSsAddr = (uint64_t)NULL;
    const MCAN_OffsetAddr *offsetAddr = MCAN_getOffsetAddr(baseAddr);
    if (offsetAddr != NULL)
    {
        mcanSsAddr = (int64_t) baseAddr + offsetAddr->mcanSsOffset;
    }
    return ((uint32_t) mcanSsAddr);
}

static uint32_t MCAN_CfgAddr(uint32_t baseAddr);
static uint32_t MCAN_CfgAddr(uint32_t baseAddr)
{
    int64_t mcanCfgAddr = (uint64_t)NULL;
    const MCAN_OffsetAddr *offsetAddr = MCAN_getOffsetAddr(baseAddr);
    if (offsetAddr != NULL)
    {
        mcanCfgAddr = (int64_t) baseAddr + offsetAddr->mcanCfgOffset;
    }
    return ((uint32_t) mcanCfgAddr);
}

void MCAN_lld_reset(uint32_t baseAddr)
{
    return;
}

uint32_t MCAN_lld_isInReset(uint32_t baseAddr)
{
    return ((uint32_t) FALSE);
}

uint32_t MCAN_lld_isFDOpEnable(uint32_t baseAddr)
{
    uint32_t fdoe;
    uint32_t state;

    fdoe = HW_RD_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_STAT,
                         CSL_APP_CANCFG_SS_STAT_EN_FDOE);
    if (1U == fdoe)
    {
        state = (uint32_t) TRUE;
    }
    else
    {
        state = (uint32_t) FALSE;
    }
    return state;
}

uint32_t MCAN_lld_isMemInitDone(uint32_t baseAddr)
{
    uint32_t memInit;
    uint32_t state;

    memInit = HW_RD_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_STAT,
                            CSL_APP_CANCFG_SS_STAT_MMI_DONE);
    if (1U == memInit)
    {
        state = (uint32_t) TRUE;
    }
    else
    {
        state = (uint32_t) FALSE;
    }
    return state;
}

void MCAN_lld_setOpMode(uint32_t baseAddr, uint32_t mode)
{

    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_INIT, mode);
}

uint32_t MCAN_lld_getOpMode(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_INIT));
}

int32_t MCAN_lld_init(uint32_t baseAddr, const MCAN_InitParams *initParams)
{
    int32_t  status;
    uint32_t regVal;

    /* Configure MCAN wakeup and clock stop controls */
    regVal = HW_RD_REG32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_CTRL);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_SS_CTRL_WAKEUPREGEN,
                   initParams->wkupReqEnable);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_SS_CTRL_AUTOWAKEUP,
                   initParams->autoWkupEnable);
    HW_WR_REG32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_CTRL, regVal);

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    /* Configure MCAN mode(FD vs Classic CAN operation) and controls */
    regVal = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_CCCR_FDOE,
                   initParams->fdMode);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_CCCR_BRSE,
                   initParams->brsEnable);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_CCCR_TXP,
                   initParams->txpEnable);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_CCCR_EFBI,
                   initParams->efbi);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_CCCR_PXHD,
                   initParams->pxhddisable);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANCFG_CCCR_DAR,
                   initParams->darEnable);
    HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, regVal);

    if ((CSL_APP_CANCFG_TDCR_TDCF_MAX >= initParams->tdcConfig.tdcf) &&
        (CSL_APP_CANCFG_TDCR_TDCO_MAX >= initParams->tdcConfig.tdco) &&
        (CSL_APP_CANCFG_RWD_WDC_MAX >= initParams->wdcPreload))
    {
        /* Configure Transceiver Delay Compensation */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TDCR,
                      CSL_APP_CANCFG_TDCR_TDCF,
                      initParams->tdcConfig.tdcf);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TDCR,
                      CSL_APP_CANCFG_TDCR_TDCO,
                      initParams->tdcConfig.tdco);
        /* Configure MSG RAM watchdog counter preload value */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RWD,
                      CSL_APP_CANCFG_RWD_WDC,
                      initParams->wdcPreload);
        /* Enable/Disable Transceiver Delay Compensation */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                      CSL_APP_CANCFG_DBTP_TDC,
                      initParams->tdcEnable);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);

    return status;
}

int32_t MCAN_lld_config(uint32_t baseAddr, const MCAN_ConfigParams *configParams)
{
    int32_t status;

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    /* Configure MCAN control registers */
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR,
                  CSL_APP_CANCFG_CCCR_MON,
                  configParams->monEnable);
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR,
                  CSL_APP_CANCFG_CCCR_ASM,
                  configParams->asmEnable);
    /* Configure Global Filter */
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_GFC,
                  CSL_APP_CANCFG_GFC_RRFE,
                  configParams->filterConfig.rrfe);
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_GFC,
                  CSL_APP_CANCFG_GFC_RRFS,
                  configParams->filterConfig.rrfs);
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_GFC,
                  CSL_APP_CANCFG_GFC_ANFE,
                  configParams->filterConfig.anfe);
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_GFC,
                  CSL_APP_CANCFG_GFC_ANFS,
                  configParams->filterConfig.anfs);

    if ((CSL_APP_CANCFG_TSCC_TCP_MAX >= configParams->tsPrescalar) &&
        (CSL_APP_CANCFG_TOCC_TOP_MAX >= configParams->timeoutPreload))
    {
        /* Configure Time-stamp counter */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TSCC,
                      CSL_APP_CANCFG_TSCC_TSS,
                      configParams->tsSelect);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TSCC,
                      CSL_APP_CANCFG_TSCC_TCP,
                      (configParams->tsPrescalar - 1U));
        /* Configure Time-out counter */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TOCC,
                      CSL_APP_CANCFG_TOCC_TOS,
                      configParams->timeoutSelect);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TOCC,
                      CSL_APP_CANCFG_TOCC_TOP,
                      configParams->timeoutPreload);
        /* Enable Time-out counter */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TOCC,
                      CSL_APP_CANCFG_TOCC_ETOC,
                      configParams->timeoutCntEnable);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);

    return status;
}

void MCAN_lld_eccConfig(uint32_t                    baseAddr,
                    const MCAN_ECCConfigParams *configParams)
{
    uint32_t regVal;
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);

    MCAN_eccLoadRegister(baseAddr, CSL_APP_CANECC_CTRL);
    regVal = HW_RD_REG32(eccAggrBaseAddr + CSL_APP_CANECC_CTRL);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANECC_CTRL_ECC_CHK,
                   configParams->enableChk);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANECC_CTRL_ECC_EN,
                   configParams->enable);
    HW_SET_FIELD32(regVal,
                   CSL_APP_CANECC_CTRL_EN_RMW,
                   configParams->enableRdModWr);
    HW_WR_REG32(eccAggrBaseAddr + CSL_APP_CANECC_CTRL, regVal);
}

int32_t MCAN_lld_setBitTime(uint32_t                    baseAddr,
                        const MCAN_BitTimingParams *configParams)
{
    int32_t status;

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    if ((CSL_APP_CANCFG_NBTP_NSJW_MAX >= configParams->nomSynchJumpWidth) &&
        (CSL_APP_CANCFG_NBTP_NTSEG2_MAX >= configParams->nomTimeSeg2) &&
        (CSL_APP_CANCFG_NBTP_NTSEG1_MIN <= configParams->nomTimeSeg1) &&
        (CSL_APP_CANCFG_NBTP_NTSEG1_MAX >= configParams->nomTimeSeg1) &&
        (CSL_APP_CANCFG_NBTP_NBRP_MAX >= configParams->nomRatePrescalar))
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                      CSL_APP_CANCFG_NBTP_NSJW,
                      configParams->nomSynchJumpWidth);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                      CSL_APP_CANCFG_NBTP_NTSEG2,
                      configParams->nomTimeSeg2);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                      CSL_APP_CANCFG_NBTP_NTSEG1,
                      configParams->nomTimeSeg1);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                      CSL_APP_CANCFG_NBTP_NBRP,
                      configParams->nomRatePrescalar);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }
    if (CSL_PASS == status)
    {
        if ((CSL_APP_CANCFG_DBTP_DSJW_MAX >= configParams->dataSynchJumpWidth) &&
            (CSL_APP_CANCFG_DBTP_DTSEG2_MAX >= configParams->dataTimeSeg2) &&
            (CSL_APP_CANCFG_DBTP_DTSEG1_MAX >= configParams->dataTimeSeg1) &&
            (CSL_APP_CANCFG_DBTP_DBRP_MAX >= configParams->dataRatePrescalar))
        {
            HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                          CSL_APP_CANCFG_DBTP_DSJW,
                          configParams->dataSynchJumpWidth);
            HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                          CSL_APP_CANCFG_DBTP_DTSEG2,
                          configParams->dataTimeSeg2);
            HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                          CSL_APP_CANCFG_DBTP_DTSEG1,
                          configParams->dataTimeSeg1);
            HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                          CSL_APP_CANCFG_DBTP_DBRP,
                          configParams->dataRatePrescalar);
            status = CSL_PASS;
        }
        else
        {
            status = CSL_EFAIL;
        }
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);
    return status;
}

int32_t MCAN_lld_msgRAMConfig(uint32_t                       baseAddr,
                          const MCAN_MsgRAMConfigParams *msgRAMConfigParams)
{
    int32_t  status;
    uint32_t elemNum = 0U;

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    /* Configure Message Filters section */
    if (0U != msgRAMConfigParams->lss)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_SIDFC,
                      CSL_APP_CANCFG_SIDFC_FLSSA_S,
                      (msgRAMConfigParams->flssa >> 2U));
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_SIDFC,
                      CSL_APP_CANCFG_SIDFC_LSS_S,
                      msgRAMConfigParams->lss);
    }
    if (0U != msgRAMConfigParams->lse)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_XIDFC,
                      CSL_APP_CANCFG_XIDFC_FLSSA_X,
                      (msgRAMConfigParams->flesa >> 2U));
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_XIDFC,
                      CSL_APP_CANCFG_XIDFC_LSS_X,
                      msgRAMConfigParams->lse);
    }
    /* Configure Rx FIFO 0 section */
    if (0U != msgRAMConfigParams->rxFIFO0Cnt)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0C,
                      CSL_APP_CANCFG_RXF0C_F0SA,
                      (msgRAMConfigParams->rxFIFO0StartAddr >> 2U));
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0C,
                      CSL_APP_CANCFG_RXF0C_F0S,
                      msgRAMConfigParams->rxFIFO0Cnt);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0C,
                      CSL_APP_CANCFG_RXF0C_F0WM,
                      msgRAMConfigParams->rxFIFO0WaterMark);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0C,
                      CSL_APP_CANCFG_RXF0C_F0OM,
                      msgRAMConfigParams->rxFIFO0OpMode);
        /* Configure Rx FIFO0 elements size */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXESC,
                      CSL_APP_CANCFG_RXESC_F0DS,
                      msgRAMConfigParams->rxFIFO0ElemSize);
    }
    /* Configure Rx FIFO 1 section */
    if (0U != msgRAMConfigParams->rxFIFO1Cnt)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1C,
                      CSL_APP_CANCFG_RXF1C_F1SA,
                      (msgRAMConfigParams->rxFIFO1StartAddr >> 2U));
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1C,
                      CSL_APP_CANCFG_RXF1C_F1S,
                      msgRAMConfigParams->rxFIFO1Cnt);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1C,
                      CSL_APP_CANCFG_RXF1C_F1WM,
                      msgRAMConfigParams->rxFIFO1WaterMark);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1C,
                      CSL_APP_CANCFG_RXF1C_F1OM,
                      msgRAMConfigParams->rxFIFO1OpMode);
        /* Configure Rx FIFO1 elements size */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXESC,
                      CSL_APP_CANCFG_RXESC_F1DS,
                      msgRAMConfigParams->rxFIFO1ElemSize);
    }
    /* Configure Rx Buffer Start Address */
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXBC,
                  CSL_APP_CANCFG_RXBC_RBSA,
                  (msgRAMConfigParams->rxBufStartAddr >> 2U));
    /* Configure Rx Buffer elements size */
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXESC,
                  CSL_APP_CANCFG_RXESC_RBDS,
                  msgRAMConfigParams->rxBufElemSize);
    /* Configure Tx Event FIFO section */
    if (0U != msgRAMConfigParams->txEventFIFOCnt)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFC,
                      CSL_APP_CANCFG_TXEFC_EFSA,
                      (msgRAMConfigParams->txEventFIFOStartAddr >> 2U));
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFC,
                      CSL_APP_CANCFG_TXEFC_EFS,
                      msgRAMConfigParams->txEventFIFOCnt);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFC,
                      CSL_APP_CANCFG_TXEFC_EFWM,
                      msgRAMConfigParams->txEventFIFOWaterMark);
    }
    /* Configure Tx Buffer and FIFO/Q section */
    elemNum = msgRAMConfigParams->txBufCnt + msgRAMConfigParams->txFIFOCnt;
    if ((MCANSS_TX_BUFFER_MAX >= elemNum) &&
        ((0U != msgRAMConfigParams->txBufCnt) ||
         (0U != msgRAMConfigParams->txFIFOCnt)))
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBC,
                      CSL_APP_CANCFG_TXBC_TBSA,
                      (msgRAMConfigParams->txStartAddr >> 2U));
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBC,
                      CSL_APP_CANCFG_TXBC_NDTB,
                      msgRAMConfigParams->txBufCnt);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBC,
                      CSL_APP_CANCFG_TXBC_TFQS,
                      msgRAMConfigParams->txFIFOCnt);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBC,
                      CSL_APP_CANCFG_TXBC_TFQM,
                      msgRAMConfigParams->txBufMode);
        /* Configure Tx Buffer/FIFO0/FIFO1 elements size */
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXESC,
                      CSL_APP_CANCFG_TXESC_TBDS,
                      msgRAMConfigParams->txBufElemSize);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }

    MCAN_writeProtectedRegAccessLock(baseAddr);

    return status;
}

int32_t MCAN_lld_setExtIDAndMask(uint32_t baseAddr, uint32_t idMask)
{
    int32_t status;

    if (CSL_APP_CANCFG_XIDAM_EIDM_MAX >= idMask)
    {
        MCAN_writeProtectedRegAccessUnlock(baseAddr);

        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_XIDAM,
                      CSL_APP_CANCFG_XIDAM_EIDM,
                      idMask);

        MCAN_writeProtectedRegAccessLock(baseAddr);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }
    return status;
}

uint32_t MCAN_lld_getWriteMsgElemAddress(uint32_t                 baseAddr,
                                     uint32_t                 memType,
                                     uint32_t                 bufNum)
{
    uint32_t startAddr = 0U, elemSize = 0U, elemAddr = 0U;
    uint32_t idx       = 0U, enableMod = 0U;

    if (MCAN_MEM_TYPE_BUF == memType)
    {
        idx       = bufNum;
        enableMod = 1U;
    }
    if (MCAN_MEM_TYPE_FIFO == memType)
    {
        idx       = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXFQS, CSL_APP_CANCFG_TXFQS_TFQPI);
        enableMod = 1U;
    }
    if (1U == enableMod)
    {
        startAddr = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBC,
                                  CSL_APP_CANCFG_TXBC_TBSA);
        elemSize = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXESC,
                                 CSL_APP_CANCFG_TXESC_TBDS);
        startAddr = (uint32_t) (startAddr << 2U);
        elemSize  = gMsgObjSize[elemSize];
        elemSize *= 4U;
        elemAddr  = startAddr + (elemSize * idx);
        elemAddr += MCAN_MCAN_MSG_MEM;
    }
    return elemAddr;
}

void MCAN_lld_writeMsgRam(uint32_t                 baseAddr,
                      uint32_t                 memType,
                      uint32_t                 bufNum,
                      const MCAN_TxBufElement *elem)
{
    uint32_t elemAddr;
    elemAddr = MCAN_lld_getWriteMsgElemAddress(baseAddr, memType, bufNum);
    MCAN_writeMsg(baseAddr, elemAddr, elem);
}

void MCAN_lld_writeMsgRamNoCpy(uint32_t                 baseAddr,
                      uint32_t                 memType,
                      uint32_t                 bufNum,
                      const MCAN_TxBufElementNoCpy *elem)
{
    uint32_t elemAddr;
    elemAddr = MCAN_lld_getWriteMsgElemAddress(baseAddr, memType, bufNum);
    MCAN_writeMsgNoCpy(baseAddr, elemAddr, elem);
}

int32_t MCAN_lld_txBufAddReq(uint32_t baseAddr, uint32_t bufNum)
{
    int32_t  status;
    uint32_t regVal;

    if (MCANSS_TX_BUFFER_MAX > bufNum)
    {
        regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBAR);
        regVal |= ((uint32_t) 1U << bufNum);
        HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBAR, regVal);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }
    return status;
}

void  MCAN_lld_getNewDataStatus(uint32_t              baseAddr,
                            MCAN_RxNewDataStatus *newDataStatus)
{

    newDataStatus->statusLow  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NDAT1);
    newDataStatus->statusHigh = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NDAT2);
}

void  MCAN_lld_clearNewDataStatus(uint32_t                    baseAddr,
                              const MCAN_RxNewDataStatus *newDataStatus)
{

    HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NDAT1, newDataStatus->statusLow);
    HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NDAT2, newDataStatus->statusHigh);
}

uint32_t MCAN_lld_getReadMsgElemAddress(uint32_t    baseAddr,
                                    uint32_t    memType,
                                    uint32_t    bufNum,
                                    uint32_t    fifoNum)
{
    uint32_t startAddr = 0U, elemSize = 0U, elemAddr = 0U;
    uint32_t enableMod = 0U, idx = 0U;

    if ((MCAN_MEM_TYPE_BUF == memType) && (MCAN_RX_BUFFER_MAX_NUM > bufNum))
    {
        startAddr = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXBC,
                                  CSL_APP_CANCFG_RXBC_RBSA);
        elemSize = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXESC,
                                 CSL_APP_CANCFG_RXESC_RBDS);
        idx       = bufNum;
        enableMod = 1U;
    }
    if (MCAN_MEM_TYPE_FIFO == memType)
    {
        switch (fifoNum)
        {
            case MCAN_RX_FIFO_NUM_0:
                startAddr = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0C,
                                          CSL_APP_CANCFG_RXF0C_F0SA);
                elemSize = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXESC,
                                         CSL_APP_CANCFG_RXESC_F0DS);
                idx = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0S,
                                    CSL_APP_CANCFG_RXF0S_F0GI);
                enableMod = 1U;
                break;
            case MCAN_RX_FIFO_NUM_1:
                startAddr = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1C,
                                          CSL_APP_CANCFG_RXF1C_F1SA);
                elemSize = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXESC,
                                         CSL_APP_CANCFG_RXESC_F1DS);
                idx = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1S,
                                    CSL_APP_CANCFG_RXF1S_F1GI);
                enableMod = 1U;
                break;
            default:
                /* Invalid option */
                break;
        }
    }
    if (1U == enableMod)
    {
        startAddr = (uint32_t) (startAddr << 2U);
        elemSize  = gMsgObjSize[elemSize];
        elemSize *= 4U;
        elemAddr  = startAddr + (elemSize * idx);
        elemAddr += MCAN_MCAN_MSG_MEM;
    }
    return elemAddr;
}

void MCAN_lld_readMsgRam(uint32_t           baseAddr,
                     uint32_t           memType,
                     uint32_t           bufNum,
                     uint32_t           fifoNum,
                     MCAN_RxBufElement *elem)
{
    uint32_t elemAddr = 0U;
    elemAddr = MCAN_lld_getReadMsgElemAddress(baseAddr, memType, bufNum, fifoNum);
    if (elemAddr != 0)
    {
        MCAN_readMsg(baseAddr, elemAddr, elem);
    }

}

void MCAN_lld_readMsgRamNoCpy(uint32_t           baseAddr,
                          uint32_t           memType,
                          uint32_t           bufNum,
                          uint32_t           fifoNum,
                          MCAN_RxBufElementNoCpy *elem)
{
    uint32_t elemAddr = 0U;
    elemAddr = MCAN_lld_getReadMsgElemAddress(baseAddr, memType, bufNum, fifoNum);
    if (elemAddr != 0)
    {
        MCAN_readMsgNoCpy(baseAddr, elemAddr, elem);
    }
}

void MCAN_lld_readTxEventFIFO(uint32_t           baseAddr,
                          MCAN_TxEventFIFOElement *txEventElem)
{
    uint32_t startAddr = 0U, elemSize = 0U, elemAddr = 0U;
    uint32_t idx = 0U, regVal;

    startAddr = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFC,
                              CSL_APP_CANCFG_TXEFC_EFSA);
    elemSize = MCANSS_TX_EVENT_FIFO_SIZE_WORDS;
    idx = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFS,
                        CSL_APP_CANCFG_TXEFS_EFGI);

    startAddr = (uint32_t) (startAddr << 2U);
    elemSize *= 4U;
    elemAddr  = startAddr + (elemSize * idx);
    elemAddr += MCAN_MCAN_MSG_MEM;

    regVal   = HW_RD_REG32(baseAddr + elemAddr);
    txEventElem->id = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_ID_MASK)
                           >> MCANSS_TX_EVENT_FIFO_ELEM_ID_SHIFT);
    txEventElem->rtr = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_RTR_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_RTR_SHIFT);
    txEventElem->xtd = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_XTD_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_XTD_SHIFT);
    txEventElem->esi = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_ESI_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_ESI_SHIFT);
    elemAddr  += 4U;
    regVal     = HW_RD_REG32(baseAddr + elemAddr);
    txEventElem->txts = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_TXTS_MASK)
                             >> MCANSS_TX_EVENT_FIFO_ELEM_TXTS_SHIFT);
    txEventElem->dlc = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_DLC_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_DLC_SHIFT);
    txEventElem->brs = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_BRS_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_BRS_SHIFT);
    txEventElem->fdf = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_FDF_MASK)
                            >> MCANSS_TX_EVENT_FIFO_ELEM_FDF_SHIFT);
    txEventElem->et = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_ET_MASK)
                             >> MCANSS_TX_EVENT_FIFO_ELEM_ET_SHIFT);
    txEventElem->mm = (uint32_t) ((regVal & MCANSS_TX_EVENT_FIFO_ELEM_MM_MASK)
                             >> MCANSS_TX_EVENT_FIFO_ELEM_MM_SHIFT);
}

void MCAN_lld_addStdMsgIDFilter(uint32_t                          baseAddr,
                            uint32_t                          filtNum,
                            const MCAN_StdMsgIDFilterElement *elem)
{
    uint32_t startAddr, elemAddr, regVal;

    startAddr = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_SIDFC,
                              CSL_APP_CANCFG_SIDFC_FLSSA_S);
    startAddr = (uint32_t) (startAddr << 2U);
    elemAddr  = startAddr + (filtNum * MCANSS_STD_ID_FILTER_SIZE_WORDS * 4U);
    elemAddr += MCAN_MCAN_MSG_MEM;

    regVal  = 0U;
    regVal |= (uint32_t) (elem->sfid2 << MCANSS_STD_ID_FILTER_SFID2_SHIFT);
    regVal |= (uint32_t) (elem->sfid1 << MCANSS_STD_ID_FILTER_SFID1_SHIFT);
    regVal |= (uint32_t) (elem->sfec << MCANSS_STD_ID_FILTER_SFEC_SHIFT);
    regVal |= (uint32_t) (elem->sft << MCANSS_STD_ID_FILTER_SFT_SHIFT);
    HW_WR_REG32(baseAddr + elemAddr, regVal);
}

void MCAN_lld_addExtMsgIDFilter(uint32_t                          baseAddr,
                            uint32_t                          filtNum,
                            const MCAN_ExtMsgIDFilterElement *elem)
{
    uint32_t startAddr, elemAddr, regVal;

    startAddr = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_XIDFC,
                              CSL_APP_CANCFG_XIDFC_FLSSA_X);
    startAddr = (uint32_t) (startAddr << 2U);
    elemAddr  = startAddr + (filtNum * MCANSS_EXT_ID_FILTER_SIZE_WORDS * 4U);
    elemAddr += MCAN_MCAN_MSG_MEM;

    regVal  = 0U;
    regVal |= (uint32_t) (elem->efid1 << MCANSS_EXT_ID_FILTER_EFID1_SHIFT);
    regVal |= (uint32_t) (elem->efec << MCANSS_EXT_ID_FILTER_EFEC_SHIFT);
    HW_WR_REG32(baseAddr + elemAddr, regVal);

    elemAddr += 4U;
    regVal    = 0U;
    regVal   |= (uint32_t) (elem->efid2 << MCANSS_EXT_ID_FILTER_EFID2_SHIFT);
    regVal   |= (uint32_t) (elem->eft << MCANSS_EXT_ID_FILTER_EFT_SHIFT);
    HW_WR_REG32(baseAddr + elemAddr, regVal);
}

void MCAN_lld_lpbkModeEnable(uint32_t baseAddr,
                         uint32_t lpbkMode,
                         uint32_t enable)
{
    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    if (TRUE == enable)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_TEST, 0x1U);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TEST,
                      CSL_APP_CANCFG_TEST_LBCK,
                      enable);
        if (MCAN_LPBK_MODE_INTERNAL == lpbkMode)
        {
            HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR,
                          CSL_APP_CANCFG_CCCR_MON,
                          0x1U);
        }
    }
    else
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TEST,
                      CSL_APP_CANCFG_TEST_LBCK,
                      enable);
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_TEST, 0x0U);
        if (MCAN_LPBK_MODE_INTERNAL == lpbkMode)
        {
            HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR,
                          CSL_APP_CANCFG_CCCR_MON,
                          0x0U);
        }
    }
    MCAN_writeProtectedRegAccessLock(baseAddr);
}

void  MCAN_lld_getErrCounters(uint32_t           baseAddr,
                          MCAN_ErrCntStatus *errCounter)
{

    errCounter->transErrLogCnt = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ECR,
                                               CSL_APP_CANCFG_ECR_TEC);
    errCounter->recErrCnt = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ECR,
                                          CSL_APP_CANCFG_ECR_REC);
    errCounter->rpStatus = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ECR,
                                         CSL_APP_CANCFG_ECR_RP);
    errCounter->canErrLogCnt = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ECR,
                                             CSL_APP_CANCFG_ECR_CEL);
}

void  MCAN_lld_getProtocolStatus(uint32_t             baseAddr,
                             MCAN_ProtocolStatus *protStatus)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_PSR);
    protStatus->lastErrCode   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_LEC);
    protStatus->act           = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_ACT);
    protStatus->errPassive    = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_EP);
    protStatus->warningStatus = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_EW);
    protStatus->busOffStatus  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_BO);
    protStatus->dlec          = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_DLEC);
    protStatus->resi          = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_RESI);
    protStatus->rbrs          = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_RBRS);
    protStatus->rfdf          = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_RFDF);
    protStatus->pxe           = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_PXE);
    protStatus->tdcv          = HW_GET_FIELD(regVal, CSL_APP_CANCFG_PSR_TDCV);
}

void MCAN_lld_enableIntr(uint32_t baseAddr, uint32_t intrMask, uint32_t enable)
{
    uint32_t regVal;

    if (TRUE == enable)
    {
        regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_IE);
        regVal |= intrMask;
        HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_IE, regVal);
    }
    else
    {
        regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_IE);
        regVal &= ~intrMask;
        HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_IE, regVal);
    }
}

void MCAN_lld_selectIntrLine(uint32_t baseAddr,
                         uint32_t intrMask,
                         uint32_t lineNum)
{
    uint32_t regVal;

    if (MCAN_INTR_LINE_NUM_0 == lineNum)
    {
        regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ILS);
        regVal &= ~intrMask;
        HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ILS, regVal);
    }
    else
    {
        regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ILS);
        regVal |= intrMask;
        HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ILS, regVal);
    }
}

uint32_t MCAN_lld_getIntrLineSelectStatus(uint32_t baseAddr)
{

    return (HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ILS));
}

void MCAN_lld_enableIntrLine(uint32_t baseAddr,
                         uint32_t lineNum,
                         uint32_t enable)
{
    uint32_t regVal, tempLineNum;

    tempLineNum = lineNum & MCANSS_INTR_LINE_EN_MASK;
    regVal   = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ILE);
    regVal  &= ~((uint32_t) 0x1U << tempLineNum);
    regVal  |= (uint32_t) (enable << tempLineNum);
    HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ILE, regVal);
}

uint32_t MCAN_lld_getIntrStatus(uint32_t baseAddr)
{

    return (HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_IR));
}

void MCAN_lld_clearIntrStatus(uint32_t baseAddr, uint32_t intrMask)
{
    HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_IR, intrMask);
}

void  MCAN_lld_getHighPriorityMsgStatus(uint32_t                  baseAddr,
                                    MCAN_HighPriorityMsgInfo *hpm)
{

    hpm->bufIdx = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_HPMS,
                                CSL_APP_CANCFG_HPMS_BIDX);
    hpm->msi = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_HPMS,
                             CSL_APP_CANCFG_HPMS_MSI);
    hpm->filterIdx = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_HPMS,
                                   CSL_APP_CANCFG_HPMS_FIDX);
    hpm->filterList = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_HPMS,
                                    CSL_APP_CANCFG_HPMS_FLST);
}

void MCAN_lld_getRxFIFOStatus(uint32_t           baseAddr,
                          MCAN_RxFIFOStatus *fifoStatus)
{
    uint32_t regVal;

    switch (fifoStatus->num)
    {
        case MCAN_RX_FIFO_NUM_0:
            regVal = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0S);
            fifoStatus->fillLvl  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF0S_F0FL);
            fifoStatus->getIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF0S_F0GI);
            fifoStatus->putIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF0S_F0PI);
            fifoStatus->fifoFull = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF0S_F0F);
            fifoStatus->msgLost  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF0S_RF0L);
            break;
        case MCAN_RX_FIFO_NUM_1:
            regVal = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1S);
            fifoStatus->fillLvl  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF1S_F1FL);
            fifoStatus->getIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF1S_F1GI);
            fifoStatus->putIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF1S_F1PI);
            fifoStatus->fifoFull = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF1S_F1F);
            fifoStatus->msgLost  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_RXF1S_RF1L);
            break;
        default:
            /* Invalid option */
            break;
    }
}

int32_t MCAN_lld_writeRxFIFOAck(uint32_t baseAddr,
                            uint32_t fifoNum,
                            uint32_t idx)
{
    int32_t  status;
    uint32_t size;

    switch (fifoNum)
    {
        case MCAN_RX_FIFO_NUM_0:
            size = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0C,
                                 CSL_APP_CANCFG_RXF0C_F0S);
            if (size >= idx)
            {
                HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF0A,
                              CSL_APP_CANCFG_RXF0A_F0AI,
                              idx);
                status = CSL_PASS;
            }
            else
            {
                status = CSL_EFAIL;
            }
            break;
        case MCAN_RX_FIFO_NUM_1:
            size = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1C,
                                 CSL_APP_CANCFG_RXF1C_F1S);
            if (size >= idx)
            {
                HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_RXF1A,
                              CSL_APP_CANCFG_RXF1A_F1AI,
                              idx);
                status = CSL_PASS;
            }
            else
            {
                status = CSL_EFAIL;
            }
            break;
        default:
            status = CSL_EFAIL;
            break;
    }

    return status;
}

void MCAN_lld_getTxFIFOQueStatus(uint32_t           baseAddr,
                             MCAN_TxFIFOStatus *fifoStatus)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXFQS);
    fifoStatus->freeLvl  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXFQS_TFFL);
    fifoStatus->getIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXFQS_TFGI);
    fifoStatus->putIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXFQS_TFQPI);
    fifoStatus->fifoFull = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXFQS_TFQF);
}

uint32_t MCAN_lld_getTxBufReqPend(uint32_t baseAddr)
{

    return (HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBRP));
}

int32_t MCAN_lld_txBufCancellationReq(uint32_t baseAddr, uint32_t buffNum)
{
    int32_t  status;
    uint32_t regVal;

    if (MCANSS_TX_BUFFER_MAX > buffNum)
    {
        regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBCR);
        regVal |= ((uint32_t) 1U << buffNum);
        HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBCR, regVal);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }
    return status;
}

uint32_t MCAN_lld_getTxBufTransmissionStatus(uint32_t baseAddr)
{

    return (HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBTO));
}

uint32_t MCAN_lld_txBufCancellationStatus(uint32_t baseAddr)
{

    return (HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBCF));
}

int32_t MCAN_lld_txBufTransIntrEnable(uint32_t baseAddr,
                                  uint32_t bufNum,
                                  uint32_t enable)
{
    int32_t  status;
    uint32_t regVal;

    if (MCANSS_TX_BUFFER_MAX > bufNum)
    {
        if (TRUE == enable)
        {
            regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBTIE);
            regVal |= ((uint32_t) 1U << bufNum);
            HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBTIE, regVal);
        }
        else
        {
            regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBTIE);
            regVal &= ~((uint32_t) 0x1U << bufNum);
            HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBTIE, regVal);
        }
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }
    return status;
}

int32_t MCAN_lld_txBufCancellationIntrEnable(uint32_t baseAddr,
                                            uint32_t bufNum,
                                            uint32_t enable)
{
    int32_t  status;
    uint32_t regVal;

    if (MCANSS_TX_BUFFER_MAX > bufNum)
    {
        if (TRUE == enable)
        {
            regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBCIE);
            regVal |= ((uint32_t) 0x1U << bufNum);
            HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBCIE, regVal);
        }
        else
        {
            regVal  = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBCIE);
            regVal &= ~((uint32_t) 0x1U << bufNum);
            HW_WR_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXBCIE, regVal);
        }
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }
    return status;
}

void MCAN_lld_getTxEventFIFOStatus(uint32_t                baseAddr,
                               MCAN_TxEventFIFOStatus *fifoStatus)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFS);
    fifoStatus->fillLvl  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXEFS_EFFL);
    fifoStatus->getIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXEFS_EFGI);
    fifoStatus->putIdx   = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXEFS_EFPI);
    fifoStatus->fifoFull = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXEFS_EFF);
    fifoStatus->eleLost  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_TXEFS_TEFL);
}

void MCAN_lld_addClockStopRequest(uint32_t baseAddr, uint32_t enable)
{

    if(TRUE == enable)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_CSR, 0x1U);
    }
    else
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_CSR, 0x0U);
    }
}

int32_t MCAN_lld_writeTxEventFIFOAck(uint32_t baseAddr, uint32_t idx)
{
    int32_t  status;
    uint32_t size;

    size = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFC,
                         CSL_APP_CANCFG_TXEFC_EFS);
    if (size >= idx)
    {
        HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TXEFA,
                      CSL_APP_CANCFG_TXEFA_EFAI,
                      idx);
        status = CSL_PASS;
    }
    else
    {
        status = CSL_EFAIL;
    }

    return status;
}

void MCAN_lld_eccForceError(uint32_t                      baseAddr,
                        const MCAN_ECCErrForceParams *eccErr)
{
    uint32_t regVal;
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    if((eccErr->errType == MCAN_ECC_ERR_TYPE_SEC) ||
       (eccErr->errType == MCAN_ECC_ERR_TYPE_DED))
    {
        MCAN_eccLoadRegister(baseAddr, CSL_APP_CANECC_ERR_CTRL1);
        regVal = HW_RD_REG32(eccAggrBaseAddr + CSL_APP_CANECC_ERR_CTRL1);
        HW_SET_FIELD32(regVal, CSL_APP_CANECC_ERR_CTRL1_ECC_ROW,
                       eccErr->rowNum);

        HW_WR_REG32(eccAggrBaseAddr + CSL_APP_CANECC_ERR_CTRL1, regVal);
        MCAN_eccLoadRegister(baseAddr, CSL_APP_CANECC_CTRL);
        regVal = HW_RD_REG32(eccAggrBaseAddr + CSL_APP_CANECC_CTRL);
        HW_SET_FIELD32(regVal,
                       CSL_APP_CANECC_CTRL_FORCE_N_ROW,
                       eccErr->errForce);
        HW_SET_FIELD32(regVal,
                       CSL_APP_CANECC_CTRL_ERROR_ONCE,
                       eccErr->errOnce);
        if(eccErr->errType == MCAN_ECC_ERR_TYPE_SEC)
        {
            HW_SET_FIELD32(regVal,
                           CSL_APP_CANECC_CTRL_FORCE_SEC,
                           0x1U);
        }
        else
        {
            HW_SET_FIELD32(regVal,
                           CSL_APP_CANECC_CTRL_FORCE_DED,
                           0x1U);
        }

        HW_WR_REG32(eccAggrBaseAddr + CSL_APP_CANECC_CTRL, regVal);
    }
}

void MCAN_lld_eccGetErrorStatus(uint32_t           baseAddr,
                            MCAN_ECCErrStatus *eccErr)
{
    uint32_t regVal;
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    MCAN_eccLoadRegister(baseAddr, CSL_APP_CANECC_ERR_STAT1);
    regVal = HW_RD_REG32(eccAggrBaseAddr + CSL_APP_CANECC_ERR_STAT1);
    eccErr->secErr = HW_GET_FIELD(regVal,
    		CSL_APP_CANECC_ERR_STAT1_ECC_SEC);
    eccErr->dedErr = HW_GET_FIELD(regVal,
    		CSL_APP_CANECC_ERR_STAT1_ECC_DED);
}

void MCAN_lld_eccClearErrorStatus(uint32_t baseAddr, uint32_t errType)
{
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    MCAN_eccLoadRegister(baseAddr, CSL_APP_CANECC_ERR_STAT1);
    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_ERR_STAT1,
            		CSL_APP_CANECC_ERR_STAT1_CLR_ECC_SEC,
                          0x1U);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_ERR_STAT1,
            		CSL_APP_CANECC_ERR_STAT1_CLR_ECC_DED,
                          0x1U);
            break;
        default:
            /* Invalid option */
            break;
    }
}

void MCAN_lld_eccWriteEOI(uint32_t baseAddr, uint32_t errType)
{
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_SEC_EOI_REG,
            		CSL_APP_CANECC_SEC_EOI_REG_SEC_EOI_WR,
                          0x1U);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_DED_EOI_REG,
            		CSL_APP_CANECC_DED_EOI_REG_DED_EOI_WR,
                          0x1U);
            break;
        default:
            /* Invalid option */
            break;
    }
}

void MCAN_lld_eccEnableIntr(uint32_t baseAddr, uint32_t errType, uint32_t enable)
{
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    if (TRUE == enable)
    {
        switch (errType)
        {
            case MCAN_ECC_ERR_TYPE_SEC:
                HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_SEC_ENABLE_SET_REG0,
                		CSL_APP_CANECC_SEC_ENABLE_SET_REG0_SEC_EN_SET,
                              0x1U);
                break;
            case MCAN_ECC_ERR_TYPE_DED:
                HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_DED_ENABLE_SET_REG0,
                		CSL_APP_CANECC_DED_ENABLE_SET_REG0_DED_EN_SET,
                              0x1U);
                break;
            default:
                /* Invalid option */
                break;
        }
    }
    else
    {
        switch (errType)
        {
            case MCAN_ECC_ERR_TYPE_SEC:
                HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_SEC_ENABLE_CLR_REG0,
                		CSL_APP_CANECC_SEC_ENABLE_CLR_REG0_SEC_EN_CLR,
                              0x1U);
                break;
            case MCAN_ECC_ERR_TYPE_DED:
                HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_DED_ENABLE_CLR_REG0,
                		CSL_APP_CANECC_DED_ENABLE_CLR_REG0_DED_EN_CLR,
                              0x1U);
                break;
            default:
                /* Invalid option */
                break;
        }
    }
}

uint32_t MCAN_lld_eccGetIntrStatus(uint32_t baseAddr, uint32_t errType)
{
    uint32_t retVal = 0U;

    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            retVal = HW_RD_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_SEC_STATUS_REG0,
            		CSL_APP_CANECC_SEC_STATUS_REG0_SEC_PEND);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            retVal = HW_RD_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_DED_STATUS_REG0,
            		CSL_APP_CANECC_DED_STATUS_REG0_DED_PEND);
            break;
        default:
            retVal = 0U;
            break;
    }
    return retVal;
}

void MCAN_lld_eccClearIntrStatus(uint32_t baseAddr, uint32_t errType)
{
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    switch (errType)
    {
        case MCAN_ECC_ERR_TYPE_SEC:
            HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_SEC_STATUS_REG0,
            		CSL_APP_CANECC_SEC_STATUS_REG0_SEC_PEND,
                          0x1U);
            break;
        case MCAN_ECC_ERR_TYPE_DED:
            HW_WR_FIELD32(eccAggrBaseAddr + CSL_APP_CANECC_DED_STATUS_REG0,
            		CSL_APP_CANECC_DED_STATUS_REG0_DED_PEND,
                          0x1U);
            break;
        default:
            break;
    }
}

void MCAN_lld_extTSCounterConfig(uint32_t baseAddr,
                             uint32_t prescalar)
{

    HW_WR_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_EXT_TS_PS,
    		CSL_APP_CANCFG_SS_EXT_TS_PS_PRESCALE, (prescalar - 1U));
}

void MCAN_lld_extTSCounterEnable(uint32_t baseAddr, uint32_t enable)
{

    HW_WR_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_CTRL,
                  CSL_APP_CANCFG_SS_CTRL_EXT_TS_CNTR_EN,
                  enable);
}

void MCAN_lld_extTSEnableIntr(uint32_t baseAddr, uint32_t enable)
{

    if (TRUE == enable)
    {
        HW_WR_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_IE,
        		CSL_APP_CANCFG_SS_IE_IE,
                      1U);
    }
    else
    {
        HW_WR_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_IECS,
        		CSL_APP_CANCFG_SS_IECS_IECS,
                      1U);
    }
}

void MCAN_lld_extTSWriteEOI(uint32_t baseAddr)
{

    HW_WR_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_EOI,
    		CSL_APP_CANCFG_SS_EOI_EOI,
                  0x1U);
}

uint32_t MCAN_lld_extTSGetUnservicedIntrCount(uint32_t baseAddr)
{
    return (HW_RD_FIELD32(baseAddr +
    		CSL_APP_CANCFG_SS_EXT_TS_USIC,
			CSL_APP_CANCFG_SS_EXT_TS_USIC_EXT_TS_INTR_CNTR));
}

/* ========================================================================== */
/*                          Advance Functions                                 */
/* ========================================================================== */

void MCAN_lld_getRevisionId(uint32_t baseAddr, MCAN_RevisionId *revId)
{
    uint32_t regVal;

    regVal        = HW_RD_REG32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_PID);
    revId->minor  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_SS_PID_MINOR);
    revId->custom = HW_GET_FIELD(regVal, CSL_APP_CANCFG_SS_PID_CUSTOM);
    revId->major  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_SS_PID_MAJOR);
    revId->rtlRev = HW_GET_FIELD(regVal, CSL_APP_CANCFG_SS_PID_RTL);
    revId->modId  = HW_GET_FIELD(regVal, CSL_APP_CANCFG_SS_PID_MODULE_ID);
    revId->bu     = HW_GET_FIELD(regVal, CSL_APP_CANCFG_SS_PID_BU);
    revId->scheme = HW_GET_FIELD(regVal, CSL_APP_CANCFG_SS_PID_SCHEME);

    regVal         = HW_RD_REG32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CREL);
    revId->day     = HW_GET_FIELD(regVal, CSL_APP_CANCFG_CREL_DAY);
    revId->mon     = HW_GET_FIELD(regVal, CSL_APP_CANCFG_CREL_MON);
    revId->year    = HW_GET_FIELD(regVal, CSL_APP_CANCFG_CREL_YEAR);
    revId->subStep = HW_GET_FIELD(regVal, CSL_APP_CANCFG_CREL_SUBSTEP);
    revId->step    = HW_GET_FIELD(regVal, CSL_APP_CANCFG_CREL_STEP);
    revId->rel     = HW_GET_FIELD(regVal, CSL_APP_CANCFG_CREL_REL);
}

uint32_t MCAN_lld_getClockStopAck(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_CSR));
}

void MCAN_lld_extTSSetRawStatus(uint32_t baseAddr)
{

    HW_WR_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_IRS,
    		CSL_APP_CANCFG_SS_IRS_IRS,
                  1U);
}

void MCAN_lld_extTSClearRawStatus(uint32_t baseAddr)
{

    HW_WR_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_ICS,
    		CSL_APP_CANCFG_SS_ICS_ICS,
                  1U);
}

uint32_t MCAN_lld_getRxPinState(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TEST, CSL_APP_CANCFG_TEST_RX));
}

void MCAN_lld_setTxPinState(uint32_t baseAddr, uint32_t state)
{

    MCAN_writeProtectedRegAccessUnlock(baseAddr);

    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_TEST, 0x1U);
    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TEST,
                  CSL_APP_CANCFG_TEST_TX,
                  state);

    MCAN_writeProtectedRegAccessLock(baseAddr);
}

uint32_t MCAN_lld_getTxPinState(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TEST, CSL_APP_CANCFG_TEST_TX));
}

uint32_t MCAN_lld_getTSCounterVal(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TSCV, CSL_APP_CANCFG_TSCV_TSC));
}

uint32_t MCAN_lld_getClkStopAck(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_CSA));
}

void MCAN_lld_getBitTime(uint32_t              baseAddr,
                     MCAN_BitTimingParams *configParams)
{

    configParams->nomSynchJumpWidth = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                                                    CSL_APP_CANCFG_NBTP_NSJW);
    configParams->nomTimeSeg2 = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                                              CSL_APP_CANCFG_NBTP_NTSEG2);
    configParams->nomTimeSeg1 = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                                              CSL_APP_CANCFG_NBTP_NTSEG1);
    configParams->nomRatePrescalar = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_NBTP,
                                                   CSL_APP_CANCFG_NBTP_NBRP);

    configParams->dataSynchJumpWidth = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                                                     CSL_APP_CANCFG_DBTP_DSJW);
    configParams->dataTimeSeg2 = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                                               CSL_APP_CANCFG_DBTP_DTSEG2);
    configParams->dataTimeSeg1 = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                                               CSL_APP_CANCFG_DBTP_DTSEG1);
    configParams->dataRatePrescalar = HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_DBTP,
                                                    CSL_APP_CANCFG_DBTP_DBRP);
}

void MCAN_lld_resetTSCounter(uint32_t baseAddr)
{

    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TSCV, CSL_APP_CANCFG_TSCV_TSC, 0x0U);
}

uint32_t MCAN_lld_getTOCounterVal(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_TOCV, CSL_APP_CANCFG_TOCV_TOC));
}

void MCAN_lld_eccAggrGetRevisionId(uint32_t baseAddr, MCAN_ECCAggrRevisionId *revId)
{
    uint32_t regVal;
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    regVal        = HW_RD_REG32(eccAggrBaseAddr + CSL_APP_CANECC_REV);
    revId->minor  = HW_GET_FIELD(regVal, CSL_APP_CANECC_REV_REVMIN);
    revId->custom = HW_GET_FIELD(regVal, CSL_APP_CANECC_REV_CUSTOM);
    revId->major  = HW_GET_FIELD(regVal, CSL_APP_CANECC_REV_REVMAJ);
    revId->rtlRev = HW_GET_FIELD(regVal, CSL_APP_CANECC_REV_REVRTL);
    revId->modId  = HW_GET_FIELD(regVal, CSL_APP_CANECC_REV_MODULE_ID);
    revId->bu     = HW_GET_FIELD(regVal, CSL_APP_CANECC_REV_BU);
    revId->scheme = HW_GET_FIELD(regVal, CSL_APP_CANECC_REV_SCHEME);
}

void MCAN_lld_eccWrapGetRevisionId(uint32_t baseAddr, MCAN_ECCWrapRevisionId *revId)
{
    return;
}

uint32_t MCAN_lld_extTSIsIntrEnable(uint32_t baseAddr)
{
    uint32_t status;

    if (1U == HW_RD_FIELD32(MCAN_SsAddr(baseAddr) + CSL_APP_CANCFG_SS_IES,
    		CSL_APP_CANCFG_SS_IES_IES))
    {
        status = (uint32_t) TRUE;
    }
    else
    {
        status = (uint32_t) FALSE;
    }

    return status;
}

uint32_t MCAN_lld_getEndianVal(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_ENDN, CSL_APP_CANCFG_ENDN_ETV));
}

uint32_t MCAN_lld_getExtIDANDMassk(uint32_t baseAddr)
{

    return (HW_RD_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_XIDAM, CSL_APP_CANCFG_XIDAM_EIDM));
}

/* ========================================================================== */
/*                          Internal Functions                                */
/* ========================================================================== */

static void MCAN_writeProtectedRegAccessUnlock(uint32_t baseAddr)
{

    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_CCE, 0x1U);
}

static void MCAN_writeProtectedRegAccessLock(uint32_t baseAddr)
{

    HW_WR_FIELD32(MCAN_CfgAddr(baseAddr) + CSL_APP_CANCFG_CCCR, CSL_APP_CANCFG_CCCR_CCE, 0x0U);
}

static void MCAN_eccLoadRegister(uint32_t baseAddr, uint32_t regOffset)
{
    uint32_t regVal = 0U, offset;
    uint32_t eccAggrBaseAddr;

    eccAggrBaseAddr = MCAN_getECCRegionAddr (baseAddr);
    offset  = regOffset & 0xFFU;
    regVal |= ((uint32_t)MCANSS_MSG_RAM_NUM << CSL_APP_CANECC_VECTOR_ECC_VEC_SHIFT);
    regVal |= (offset << CSL_APP_CANECC_VECTOR_RD_SVBUS_ADDR_SHIFT);
    regVal |= ((uint32_t)1U << CSL_APP_CANECC_VECTOR_RD_SVBUS_SHIFT);
    HW_WR_REG32(eccAggrBaseAddr + CSL_APP_CANECC_VECTOR, regVal);
    while (CSL_APP_CANECC_VECTOR_RD_SVBUS_DONE_MASK !=
           (HW_RD_REG32(eccAggrBaseAddr + CSL_APP_CANECC_VECTOR) &
            CSL_APP_CANECC_VECTOR_RD_SVBUS_DONE_MASK))
    {}
}

static void MCAN_readMsg(uint32_t           baseAddr,
                         uint32_t           elemAddr,
                         MCAN_RxBufElement *elem)
{
    uint32_t regVal = 0U, loopCnt = 0U;
    uint32_t tempElemAddr = elemAddr;

    regVal   = HW_RD_REG32(baseAddr + tempElemAddr);
    elem->id = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ID_MASK)
                           >> MCANSS_RX_BUFFER_ELEM_ID_SHIFT);
    elem->rtr = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RTR_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_RTR_SHIFT);
    elem->xtd = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_XTD_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_XTD_SHIFT);
    elem->esi = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ESI_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_ESI_SHIFT);

    tempElemAddr  += 4U;
    regVal     = HW_RD_REG32(baseAddr + tempElemAddr);
    elem->rxts = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RXTS_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_RXTS_SHIFT);
    elem->dlc = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_DLC_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_DLC_SHIFT);
    elem->brs = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_BRS_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_BRS_SHIFT);
    elem->fdf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FDF_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_FDF_SHIFT);
    elem->fidx = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FIDX_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_FIDX_SHIFT);
    elem->anmf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ANMF_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_ANMF_SHIFT);
    tempElemAddr += 4U;

    loopCnt = 0U;
    /* Reading words from message RAM and forming payload bytes out of it */
    while ((4U <= (gDataSize[elem->dlc] - loopCnt)) &&
           (0U != (gDataSize[elem->dlc] - loopCnt)))
    {
        regVal = HW_RD_REG32(baseAddr + tempElemAddr);
        elem->data[loopCnt]       = (uint8_t)(regVal & 0x000000FFU);
        elem->data[(loopCnt + 1U)] = (uint8_t)((regVal & 0x0000FF00U) >> 8U);
        elem->data[(loopCnt + 2U)] = (uint8_t)((regVal & 0x00FF0000U) >> 16U);
        elem->data[(loopCnt + 3U)] = (uint8_t)((regVal & 0xFF000000U) >> 24U);
        tempElemAddr += 4U;
        loopCnt  += 4U;
    }
    /* Reading remaining bytes from message RAM */
    if (0U < (gDataSize[elem->dlc] - loopCnt))
    {
        regVal = HW_RD_REG32(baseAddr + tempElemAddr);
        elem->data[loopCnt]       = (uint8_t)(regVal & 0x000000FFU);
        elem->data[(loopCnt + 1U)] = (uint8_t)((regVal & 0x0000FF00U) >> 8U);
        elem->data[(loopCnt + 2U)] = (uint8_t)((regVal & 0x00FF0000U) >> 16U);
    }
}

static void MCAN_readMsgNoCpy(uint32_t           baseAddr,
                         uint32_t           elemAddr,
                         MCAN_RxBufElementNoCpy *elem)
{
    uint32_t regVal = 0U, loopCnt = 0U;
    uint32_t tempElemAddr = elemAddr;

    regVal   = HW_RD_REG32(baseAddr + tempElemAddr);
    elem->id = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ID_MASK)
                           >> MCANSS_RX_BUFFER_ELEM_ID_SHIFT);
    elem->rtr = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RTR_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_RTR_SHIFT);
    elem->xtd = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_XTD_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_XTD_SHIFT);
    elem->esi = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ESI_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_ESI_SHIFT);

    tempElemAddr  += 4U;
    regVal     = HW_RD_REG32(baseAddr + tempElemAddr);
    elem->rxts = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RXTS_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_RXTS_SHIFT);
    elem->dlc = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_DLC_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_DLC_SHIFT);
    elem->brs = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_BRS_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_BRS_SHIFT);
    elem->fdf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FDF_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_FDF_SHIFT);
    elem->fidx = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FIDX_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_FIDX_SHIFT);
    elem->anmf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ANMF_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_ANMF_SHIFT);
    tempElemAddr += 4U;

    /* Dest, Src, num_of_bytes */
    loopCnt = gDataSize[elem->dlc];

    /*
     * sizeof(void *) == sizeof(uint64_t) in case of 64 bit architecture while
     * sizeof(void *) == sizeof(uint32_t) in case of 32 bit architecture
     * typecasting to (void *) will give an error in case if proper typecasting
     * of (baseAddr + tempElemAddr) is not done before.
     */
#ifndef __aarch64__
    memcpy((void *)elem->data, (void *)(baseAddr + tempElemAddr), loopCnt);
#else
    memcpy((void *)elem->data, (void *)((uint64_t)(baseAddr + tempElemAddr)), loopCnt);
#endif
}

static void MCAN_writeMsg(uint32_t                 baseAddr,
                          uint32_t                 elemAddr,
                          const MCAN_TxBufElement *elem)
{
    uint32_t regVal = 0, loopCnt = 0U;
    uint32_t tempElemAddr = elemAddr;

    regVal  = 0U;
    regVal |= (((uint32_t) (elem->id << MCANSS_TX_BUFFER_ELEM_ID_SHIFT)) |
               ((uint32_t) (elem->rtr << MCANSS_TX_BUFFER_ELEM_RTR_SHIFT)) |
               ((uint32_t) (elem->xtd << MCANSS_TX_BUFFER_ELEM_XTD_SHIFT)) |
               ((uint32_t) (elem->esi << MCANSS_TX_BUFFER_ELEM_ESI_SHIFT)));
    HW_WR_REG32(baseAddr + tempElemAddr, regVal);
    tempElemAddr += 4U;

    regVal  = 0U;
    regVal |= ((uint32_t) (elem->dlc << MCANSS_TX_BUFFER_ELEM_DLC_SHIFT)) |
              ((uint32_t) (elem->brs << MCANSS_TX_BUFFER_ELEM_BRS_SHIFT)) |
              ((uint32_t) (elem->fdf << MCANSS_TX_BUFFER_ELEM_FDF_SHIFT)) |
              ((uint32_t) (elem->efc << MCANSS_TX_BUFFER_ELEM_EFC_SHIFT)) |
              ((uint32_t) (elem->mm << MCANSS_TX_BUFFER_ELEM_MM_SHIFT));
    HW_WR_REG32(baseAddr + tempElemAddr, regVal);
    tempElemAddr += 4U;

    loopCnt = 0U;
    /* Framing words out of the payload bytes and writing it to message RAM */
    while ((4U <= (gDataSize[elem->dlc] - loopCnt)) &&
           (0U != (gDataSize[elem->dlc] - loopCnt)))
    {
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + tempElemAddr, regVal);
        tempElemAddr += 4U;
        loopCnt  += 4U;
    }
    /* Framing a word out of remaining payload bytes and writing it to
     * message RAM */
    if (0U < (gDataSize[elem->dlc] - loopCnt))
    {
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + tempElemAddr, regVal);
    }
}

void MCAN_lld_writeDmaHeader( const void* data, const MCAN_TxBufElement *elem)
{
    *((uint32_t*)(data))= (((uint32_t) ((elem->id << MCANSS_TX_BUFFER_ELEM_ID_SHIFT)   & MCANSS_TX_BUFFER_ELEM_ID_MASK)) |
               ((uint32_t) ((elem->rtr << MCANSS_TX_BUFFER_ELEM_RTR_SHIFT) & MCANSS_TX_BUFFER_ELEM_RTR_MASK)) |
               ((uint32_t) ((elem->xtd << MCANSS_TX_BUFFER_ELEM_XTD_SHIFT) & MCANSS_TX_BUFFER_ELEM_XTD_MASK)) |
               ((uint32_t) ((elem->esi << MCANSS_TX_BUFFER_ELEM_ESI_SHIFT) & MCANSS_TX_BUFFER_ELEM_ESI_MASK)));

    *((uint32_t*)(data+4))= (((uint32_t)((elem->dlc << MCANSS_TX_BUFFER_ELEM_DLC_SHIFT) & MCANSS_TX_BUFFER_ELEM_DLC_MASK)) |
              ((uint32_t) ((elem->brs << MCANSS_TX_BUFFER_ELEM_BRS_SHIFT) & MCANSS_TX_BUFFER_ELEM_BRS_MASK)) |
              ((uint32_t) ((elem->fdf << MCANSS_TX_BUFFER_ELEM_FDF_SHIFT) & MCANSS_TX_BUFFER_ELEM_FDF_MASK)) |
              ((uint32_t) ((elem->efc << MCANSS_TX_BUFFER_ELEM_EFC_SHIFT) & MCANSS_TX_BUFFER_ELEM_EFC_MASK)) |
              ((uint32_t) ((elem->mm << MCANSS_TX_BUFFER_ELEM_MM_SHIFT) & MCANSS_TX_BUFFER_ELEM_MM_MASK)));
}

static void MCAN_writeMsgNoCpy(uint32_t                 baseAddr,
                          uint32_t                 elemAddr,
                          const MCAN_TxBufElementNoCpy *elem)
{
    uint32_t regVal = 0, loopCnt = 0U;
    uint32_t tempElemAddr = elemAddr;

    regVal  = 0U;
    regVal |= (((uint32_t) (elem->id << MCANSS_TX_BUFFER_ELEM_ID_SHIFT)) |
               ((uint32_t) (elem->rtr << MCANSS_TX_BUFFER_ELEM_RTR_SHIFT)) |
               ((uint32_t) (elem->xtd << MCANSS_TX_BUFFER_ELEM_XTD_SHIFT)) |
               ((uint32_t) (elem->esi << MCANSS_TX_BUFFER_ELEM_ESI_SHIFT)));
    HW_WR_REG32(baseAddr + tempElemAddr, regVal);
    tempElemAddr += 4U;

    regVal  = 0U;
    regVal |= ((uint32_t) (elem->dlc << MCANSS_TX_BUFFER_ELEM_DLC_SHIFT)) |
              ((uint32_t) (elem->brs << MCANSS_TX_BUFFER_ELEM_BRS_SHIFT)) |
              ((uint32_t) (elem->fdf << MCANSS_TX_BUFFER_ELEM_FDF_SHIFT)) |
              ((uint32_t) (elem->efc << MCANSS_TX_BUFFER_ELEM_EFC_SHIFT)) |
              ((uint32_t) (elem->mm << MCANSS_TX_BUFFER_ELEM_MM_SHIFT));
    HW_WR_REG32(baseAddr + tempElemAddr, regVal);
    tempElemAddr += 4U;

    loopCnt = 0U;
    /* Framing words out of the payload bytes and writing it to message RAM */
    while ((4U <= (gDataSize[elem->dlc] - loopCnt)) &&
           (0U != (gDataSize[elem->dlc] - loopCnt)))
    {
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + tempElemAddr, regVal);
        tempElemAddr += 4U;
        loopCnt  += 4U;
    }
    /* Framing a word out of remaining payload bytes and writing it to
     * message RAM */
    if (0U < (gDataSize[elem->dlc] - loopCnt))
    {
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + tempElemAddr, regVal);
    }

#if defined (ARM_V7_R) || defined (__ARM_ARCH_7R__)
    /**
     * The MCAN MSG RAM is configured to be non-shareable device memory instead of
     * strongly ordered memory. Add a dsb instruction after the last write
     * to be sure that before exiting the function the writes have indeed
     * done.
     * DSB instruction specific to R5 hence guard by __ARM_ARCH_7R__ flag
     */
    __asm__ __volatile__ ( "dsb sy"  "\n\t": : : "memory");
#endif
}

static uint32_t MCAN_getECCRegionAddr(uint32_t baseAddr)
{
    uint64_t eccAggrBase = 0U;
    switch ((uint64_t) baseAddr)
    {
        default:
            eccAggrBase = CSL_APP_MCAN_ECC_U_BASE;
            break;
    }
    return (uint32_t) eccAggrBase;
}

static const MCAN_OffsetAddr* MCAN_getOffsetAddr(uint32_t baseAddr)
{
    const MCAN_OffsetAddr *offsetAddr;
    switch ((uint64_t) baseAddr)
    {
        case CSL_APP_MCAN_MSG_RAM_U_BASE:
            offsetAddr = &gMcanOffsetAddr;
            break;
        default:
            offsetAddr = NULL;
            break;
    }
    return offsetAddr;
}

/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */
void MCAN_lld_initTxBufElement(MCAN_TxBufElement *txMsg)
{
    uint32_t i;

    if (txMsg != NULL)
    {
        /* Standard message identifier 11 bit, stored into ID[28-18] */
        txMsg->id  = ((0x01U & MCAN_STD_ID_MASK) << MCAN_STD_ID_SHIFT);
        /* Transmit data frame */
        txMsg->rtr = FALSE;
        /* Standard message id */
        txMsg->xtd = FALSE;
        txMsg->esi = FALSE;
        /* Payload size is 64 bytes */
        txMsg->dlc = MCAN_DATA_SIZE_64BYTES;
        /* Bit Rate Switch */
        txMsg->brs = TRUE;
        /* CAN FD Frame Format */
        txMsg->fdf = TRUE;
        txMsg->efc = TRUE;
        txMsg->mm  = 0xAAU;
        for (i = 0U; i < MCAN_MAX_PAYLOAD_BYTES; i++)
        {
            txMsg->data[i] = 0U;
        }
    }

    return;
}

void MCAN_lld_initOperModeParams(MCAN_InitParams *initParams)
{
    if (initParams != NULL)
    {
        /* Initialize MCAN Operating Mode params */
        initParams->fdMode          = TRUE;
        initParams->brsEnable       = TRUE;
        initParams->txpEnable       = FALSE;
        initParams->efbi            = FALSE;
        initParams->pxhddisable     = FALSE;
        initParams->darEnable       = TRUE;
        initParams->wkupReqEnable   = TRUE;
        initParams->autoWkupEnable  = TRUE;
        initParams->emulationEnable = TRUE;
        initParams->emulationFAck   = FALSE;
        initParams->clkStopFAck     = FALSE;
        initParams->wdcPreload      = 0xFFU;
        initParams->tdcEnable       = TRUE;
        initParams->tdcConfig.tdcf  = 0xAU;
        initParams->tdcConfig.tdco  = 0x6U;
    }

    return;
}

void MCAN_lld_initGlobalFilterConfigParams(MCAN_ConfigParams *configParams)
{
    if (configParams != NULL)
    {
        /* Initialize MCAN Config params */
        configParams->monEnable         = FALSE;
        configParams->asmEnable         = FALSE;
        configParams->tsPrescalar       = 0xFU;
        configParams->tsSelect          = FALSE;
        configParams->timeoutSelect     = MCAN_TIMEOUT_SELECT_CONT;
        configParams->timeoutPreload    = 0xFFFFU;
        configParams->timeoutCntEnable  = FALSE;
        /* Accept Non Matching Frames and Reject Remote frames */
        configParams->filterConfig.rrfs = TRUE;
        configParams->filterConfig.rrfe = TRUE;
        configParams->filterConfig.anfe = MCAN_RX_FIFO_NUM_1;
        configParams->filterConfig.anfs = MCAN_RX_FIFO_NUM_1;
    }

    return;
}

void MCAN_lld_initMsgRamConfigParams(MCAN_MsgRAMConfigParams *msgRAMConfigParams)
{
    if (msgRAMConfigParams != NULL)
    {
        /* Initialize Message RAM Sections Configuration Parameters */
        msgRAMConfigParams->flssa                = 0U;
        msgRAMConfigParams->lss                  = 0U;
        msgRAMConfigParams->flesa                = 0U;
        msgRAMConfigParams->lse                  = 0U;
        msgRAMConfigParams->txStartAddr          = 0U;
        msgRAMConfigParams->txBufCnt             = 0U;
        msgRAMConfigParams->txFIFOCnt            = 0U;
        msgRAMConfigParams->txBufMode            = 0U;
        msgRAMConfigParams->txEventFIFOStartAddr = 0U;
        msgRAMConfigParams->txEventFIFOCnt       = 0U;
        msgRAMConfigParams->txEventFIFOWaterMark = 0U;
        msgRAMConfigParams->rxFIFO0StartAddr     = 0U;
        msgRAMConfigParams->rxFIFO0Cnt           = 0U;
        msgRAMConfigParams->rxFIFO0WaterMark     = 0U;
        msgRAMConfigParams->rxFIFO0OpMode        = 0U;
        msgRAMConfigParams->rxFIFO1StartAddr     = 0U;
        msgRAMConfigParams->rxFIFO1Cnt           = 0U;
        msgRAMConfigParams->rxFIFO1WaterMark     = 0U;
        msgRAMConfigParams->rxFIFO1OpMode        = 0U;
        msgRAMConfigParams->rxBufStartAddr       = 0U;
        msgRAMConfigParams->rxBufElemSize        = MCAN_ELEM_SIZE_64BYTES;
        msgRAMConfigParams->rxFIFO0ElemSize      = MCAN_ELEM_SIZE_64BYTES;
        msgRAMConfigParams->rxFIFO1ElemSize      = MCAN_ELEM_SIZE_64BYTES;
        msgRAMConfigParams->txBufElemSize        = MCAN_ELEM_SIZE_64BYTES;
    }

    return;
}

int32_t MCAN_lld_calcMsgRamParamsStartAddr(MCAN_MsgRAMConfigParams *msgRAMConfigParams)
{
    uint32_t startAddr, msgRAMSizeConfigured = 0U;
    int32_t  status = CSL_PASS;

    if (msgRAMConfigParams != NULL)
    {
        /* Calculate message RAM size configured and check for out of bound
         * error */
        msgRAMSizeConfigured  = ((msgRAMConfigParams->lss) *
                                  MCAN_MSG_RAM_STD_ELEM_SIZE);
        msgRAMSizeConfigured += ((msgRAMConfigParams->lse) *
                                  MCAN_MSG_RAM_EXT_ELEM_SIZE);
        msgRAMSizeConfigured += ((msgRAMConfigParams->txBufCnt) *
                                  MCAN_MSG_RAM_TX_RX_ELEM_SIZE);
        msgRAMSizeConfigured += ((msgRAMConfigParams->txFIFOCnt) *
                                  MCAN_MSG_RAM_TX_RX_ELEM_SIZE);
        msgRAMSizeConfigured += ((msgRAMConfigParams->txEventFIFOCnt) *
                                  MCAN_MSG_RAM_EXT_ELEM_SIZE);
        msgRAMSizeConfigured += ((msgRAMConfigParams->rxFIFO0Cnt) *
                                  MCAN_MSG_RAM_TX_RX_ELEM_SIZE);
        msgRAMSizeConfigured += ((msgRAMConfigParams->rxFIFO1Cnt) *
                                  MCAN_MSG_RAM_TX_RX_ELEM_SIZE);

        if (msgRAMSizeConfigured <= MCAN_MSG_RAM_MAX_WORD_COUNT)
        {
            /* Compute the start Address and populate the Message RAM configuration
             * parameters */
            startAddr = 0U;

            /* 11-bit filter configuration */
            /* Assign only if configured */
            if (msgRAMConfigParams->lss != 0U)
            {
                msgRAMConfigParams->flssa = startAddr;
            }

            /* 29-bit filter configuration */
            startAddr += ((msgRAMConfigParams->lss) *
                           MCAN_MSG_RAM_STD_ELEM_SIZE * 4U);
            if (msgRAMConfigParams->lse != 0U)
            {
                msgRAMConfigParams->flesa = startAddr;
            }

            /* Tx buffer configuration */
            startAddr += ((msgRAMConfigParams->lse) *
                           MCAN_MSG_RAM_EXT_ELEM_SIZE * 4U);
            if ((msgRAMConfigParams->txBufCnt  != 0U) ||
                (msgRAMConfigParams->txFIFOCnt != 0U))
            {
                msgRAMConfigParams->txStartAddr = startAddr;
                msgRAMConfigParams->txBufElemSize = MCAN_ELEM_SIZE_64BYTES;
            }

            /* Tx Event FIFO configuration */
            startAddr += ((msgRAMConfigParams->txBufCnt) *
                           MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
            startAddr += ((msgRAMConfigParams->txFIFOCnt) *
                           MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
            if (msgRAMConfigParams->txEventFIFOCnt != 0U)
            {
                msgRAMConfigParams->txEventFIFOStartAddr = startAddr;
            }

            /* Rx FIFO 0 configuration */
            startAddr += ((msgRAMConfigParams->txEventFIFOCnt) *
                           MCAN_MSG_RAM_EXT_ELEM_SIZE * 4U);
            if (msgRAMConfigParams->rxFIFO0Cnt != 0U)
            {
                msgRAMConfigParams->rxFIFO0StartAddr = startAddr;
                msgRAMConfigParams->rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES;
            }

            /* Rx FIFO 1 configuration */
            startAddr += ((msgRAMConfigParams->rxFIFO0Cnt) *
                           MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
            if (msgRAMConfigParams->rxFIFO1Cnt != 0U)
            {
                msgRAMConfigParams->rxFIFO1StartAddr = startAddr;
                msgRAMConfigParams->rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES;
            }

            /* Rx Buffer configuration */
            startAddr += ((msgRAMConfigParams->rxFIFO1Cnt) *
                           MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
            msgRAMConfigParams->rxBufStartAddr = startAddr;
            msgRAMConfigParams->rxBufElemSize = MCAN_ELEM_SIZE_64BYTES;
        }
        else
        {
            status = CSL_EFAIL;
        }
    }
    else
    {
        status = CSL_EFAIL;
    }

    return status;
}
