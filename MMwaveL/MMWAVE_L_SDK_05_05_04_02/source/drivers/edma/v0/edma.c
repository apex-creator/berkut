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
 *  \file   edma.c
 *
 *  \brief  This file contains device abstraction layer APIs for the EDMA device.
 *          There are APIs here to enable the EDMA instance, set the required
 *          configurations for communication, transmit or receive data.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <drivers/edma.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>

//TODO: The below global variables come from edma cfg xdt template 
//EDMA_Config gEdmaConfig[2];
//uint32_t    gEdmaConfigNum;


/* ========================================================================== */
/*                        Static Function Declaration                         */
/* ========================================================================== */
static int32_t EDMAInitialize (uint32_t baseAddr, const EDMA_InitParams *initParam);
static int32_t EDMADeinitialize (uint32_t baseAddr, const EDMA_InitParams *initParam);
static void EDMA_transferCompletionMasterIsrFxn(void *args);
static int32_t EDMA_allocResource(EDMA_Handle handle, uint32_t resType, uint32_t *resId);
static int32_t allocResource(const EDMA_Attrs *attrs, EDMA_Object *object, uint32_t *resId, uint32_t resType);
static int32_t EDMA_freeResource(EDMA_Handle handle, uint32_t resType, uint32_t *resId);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
* Design: MMWLPSDK-754
*/
void EDMA_InitParams_init (EDMA_InitParams *initParam)
{
    uint32_t i;

    if (initParam != NULL)
    {
        initParam->regionId     = 0U;
        initParam->queNum       = 0U;
        initParam->ownResource.qdmaCh      = 0xFFU;
        initParam->initParamSet = FALSE;
        for (i = 0U; i < (SOC_EDMA_NUM_DMACH/32U); i++)
        {
            initParam->ownResource.dmaCh[i]      = 0xFFFFFFFFU;
            initParam->reservedDmaCh[i] = 0U;
        }
        for (i = 0U; i < (EDMA_NUM_TCC/32U); i++)
        {
            initParam->ownResource.tcc[i]      = 0xFFFFFFFFU;
        }
        for (i = 0U; i < (SOC_EDMA_NUM_PARAMSETS/32U); i++)
        {
            initParam->ownResource.paramSet[i]      = 0xFFFFFFFFU;
        }
    }
}

/**
* Design: MMWLPSDK-819
*/
static int32_t EDMAInitialize (uint32_t baseAddr, const EDMA_InitParams *initParam)
{
    int32_t retVal = SystemP_SUCCESS;
    EDMACCPaRAMEntry paramSet;
    uint32_t count = 0U;
    uint32_t i     = 0U;
    uint32_t qnumValue;

    if (initParam == NULL)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        /* Clear the Event miss Registers                                     */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, initParam->ownResource.dmaCh[0]);
    #if SOC_EDMA_NUM_DMACH > 32
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, initParam->ownResource.dmaCh[1]);
    #endif
        HW_WR_REG32(baseAddr + EDMA_TPCC_QEMCR, initParam->ownResource.qdmaCh);

        /* Clear CCERR register.                                               */
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR, EDMA_SET_ALL_BITS);

        /* Disable and clear DMA events for all own dma channels */
        for (i = 0; i < SOC_EDMA_NUM_DMACH; i++)
        {
            if (((1U << (i%32U)) & initParam->ownResource.dmaCh[i/32U]) != 0U)
            {
                EDMADisableDmaEvtRegion(baseAddr, initParam->regionId, i);
                EDMAClrEvtRegion(baseAddr, initParam->regionId, i);
                EDMAClrMissEvtRegion(baseAddr, initParam->regionId, i);
            }
        }

        /* Disable and clear channel interrupts for all own dma channels */
        for (i = 0; i < EDMA_NUM_TCC; i++)
        {
            if (((1U << (i%32U)) & initParam->ownResource.tcc[i/32U]) != 0U)
            {
                EDMADisableEvtIntrRegion(baseAddr, initParam->regionId, i);
                EDMAClrIntrRegion(baseAddr, initParam->regionId, i);
            }
        }

        /* Disable and clear channel interrupts for all own qdma channels */
        for (i = 0; i < SOC_EDMA_NUM_QDMACH; i++)
        {
            if (((1U << i) & initParam->ownResource.qdmaCh) != 0U)
            {
                EDMADisableQdmaEvtRegion(baseAddr, initParam->regionId, i);
                EDMAQdmaClrMissEvtRegion(baseAddr, initParam->regionId, i);
            }
        }
        /* FOR TYPE EDMA*/
        /* Enable the own DMA (0 - 64) channels in the DRAE and DRAEH register */
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(initParam->regionId), initParam->ownResource.dmaCh[0]);
    #if SOC_EDMA_NUM_DMACH > 32
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(initParam->regionId), initParam->ownResource.dmaCh[1]);
    #endif
        /* Enable the own TCCs also for the region in DRAE and DRAEH register */
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(initParam->regionId), initParam->ownResource.tcc[0]);
    #if SOC_EDMA_NUM_DMACH > 32
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(initParam->regionId), initParam->ownResource.tcc[1]);
    #endif
        #if(SOC_EDMA_CHMAPEXIST)
        {
            for (i = 0U; i < SOC_EDMA_NUM_DMACH; i++)
            {
                if (((1U << (i%32U)) & initParam->ownResource.dmaCh[i/32U]) != 0U)
                {
                    /* All channels are one to one mapped with the params */
                    HW_WR_REG32(baseAddr + EDMA_TPCC_DCHMAPN(i), i << 5);
                }
            }
        }
        #endif

        /* Initialize the DMA Queue Number Registers                            */
        for (count = 0U; count < SOC_EDMA_NUM_DMACH; count++)
        {
            if (((1U << (count%32U)) & initParam->ownResource.dmaCh[count/32U]) != 0U)
            {
                qnumValue  = HW_RD_REG32(baseAddr + (EDMA_TPCC_DMAQNUMN((count >> 3U))));
                qnumValue &= EDMACC_DMAQNUM_CLR(count);
                qnumValue |= EDMACC_DMAQNUM_SET(count, initParam->queNum);
                HW_WR_REG32(baseAddr + (EDMA_TPCC_DMAQNUMN((count >> 3U))), qnumValue);
            }
        }

        /* FOR TYPE QDMA */
        /* Enable the QDMA (0 - 64) channels in the DRAE register                */
        HW_WR_REG32(baseAddr + EDMA_TPCC_QRAEN(initParam->regionId), initParam->ownResource.qdmaCh);

        /* Initialize the QDMA Queue Number Registers                           */
        for (count = 0U; count < SOC_EDMA_NUM_QDMACH; count++)
        {
            if (((1U << count) & initParam->ownResource.qdmaCh) != 0U)
            {
                qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
                qnumValue &= EDMACC_QDMAQNUM_CLR(count);
                qnumValue |= EDMACC_QDMAQNUM_SET(count, initParam->queNum);
                HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
            }
        }

        if (initParam->initParamSet == TRUE)
        {
            EDMACCPaRAMEntry_init(&paramSet);
            /* cleanup Params, note h/w reset state is all 0s, must be done after
            disabling/clearning channel events (in particular QDMA) */
            for (count = 0; count < SOC_EDMA_NUM_PARAMSETS; count++)
            {
                if (((1U << (count%32U)) & initParam->ownResource.paramSet[count/32U]) != 0U)
                {
                    EDMASetPaRAM(baseAddr, count, &paramSet);
                }
            }
        }
    }
    return retVal;
}

/**
* Design: MMWLPSDK-755
*/
void EDMACCPaRAMEntry_init(EDMACCPaRAMEntry *paramEntry)
{
    if (paramEntry != NULL)
    {
        /* Initialize all PaRAM entries as 0 */
        paramEntry->opt = 0;
        paramEntry->srcAddr = 0;
        paramEntry->aCnt = 0;
        paramEntry->bCnt = 0;
        paramEntry->destAddr = 0;
        paramEntry->srcBIdx = 0;
        paramEntry->destBIdx = 0;
        paramEntry->linkAddr = 0;
        paramEntry->bCntReload = 0;
        paramEntry->srcCIdx = 0;
        paramEntry->destCIdx = 0;
        paramEntry->cCnt = 0;
        paramEntry->destBIdxExt = 0;
        paramEntry->srcBIdxExt = 0;
    }
}

/**
* Design: MMWLPSDK-795
*/
uint32_t EDMAPeripheralIdGet(uint32_t baseAddr)
{
    return (HW_RD_REG32(baseAddr + EDMA_TPCC_PID));
}

/**
* Design: MMWLPSDK-756
*/
void EDMAEnableChInShadowRegRegion(uint32_t baseAddr,
                                    uint32_t regionId,
                                    uint32_t chType,
                                    uint32_t chNum)
{
    uint32_t draeValue;
    /* Allocate the DMA/QDMA channel */
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* FOR TYPE EDMA*/
        if (chNum < 32U)
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId));
            /* Enable the DMA channel in the DRAE registers */
            draeValue |= (uint32_t) 0x01U << chNum;
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId), draeValue);
        }
        else
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId));

            /* Enable the DMA channel in the DRAEH registers */
            draeValue |= (uint32_t) 0x01U << (chNum - 32U);
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId), draeValue);
        }
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* FOR TYPE QDMA */
        /* Enable the QDMA channel in the DRAE/DRAEH registers */
        draeValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId));
        draeValue |= (uint32_t) 0x01U << chNum;
        HW_WR_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId), draeValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

/**
* Design: MMWLPSDK-757
*/
void EDMADisableChInShadowRegRegion(uint32_t baseAddr,
                                     uint32_t regionId,
                                     uint32_t chType,
                                     uint32_t chNum)
{
    uint32_t draeValue;
    /* Allocate the DMA/QDMA channel */
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* FOR TYPE EDMA*/
        if (chNum < 32U)
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId));
            /* Enable the DMA channel in the DRAE registers */
            draeValue &= ~((uint32_t) 0x01U << chNum);
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(regionId), draeValue);
        }
        else
        {
            draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId));
            /* Enable the DMA channel in the DRAEH registers */
            draeValue &= ~((uint32_t) 0x01U << (chNum - 32U));
            HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(regionId), draeValue);
        }
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* FOR TYPE QDMA */
        draeValue = HW_RD_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId));
        /* Enable the QDMA channel in the DRAE/DRAEH registers */
        draeValue &= ~((uint32_t) 0x01U) << chNum;
        HW_WR_REG32(baseAddr + EDMA_TPCC_QRAEN(regionId), draeValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

/**
* Design: MMWLPSDK-758
*/
void EDMAChannelToParamMap(uint32_t baseAddr,
                            uint32_t channel,
                            uint32_t paramSet)

{
    #if(SOC_EDMA_CHMAPEXIST)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_DCHMAPN(channel), paramSet << 5U);
    }
    #endif
}

/**
* Design: MMWLPSDK-759
*/
void EDMAMapChToEvtQ(uint32_t baseAddr,
                      uint32_t chType,
                      uint32_t chNum,
                      uint32_t evtQNum)
{
    uint32_t qnumValue;
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* Associate DMA Channel to Event Queue                             */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U));
        qnumValue &= EDMACC_DMAQNUM_CLR(chNum);
        qnumValue |= EDMACC_DMAQNUM_SET(chNum, evtQNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U), qnumValue);
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* Associate QDMA Channel to Event Queue                            */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
        qnumValue &= EDMACC_QDMAQNUM_CLR(chNum);
        qnumValue |= EDMACC_QDMAQNUM_SET(chNum, evtQNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

/**
* Design: MMWLPSDK-760
*/
void EDMAUnmapChToEvtQ(uint32_t baseAddr,
                        uint32_t chType,
                        uint32_t chNum)
{
    uint32_t qnumValue;
    if (EDMA_CHANNEL_TYPE_DMA == chType)
    {
        /* Unmap DMA Channel to Event Queue                                */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U));
        qnumValue &= EDMACC_DMAQNUM_CLR(chNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_DMAQNUMN(chNum >> 3U), qnumValue);
    }
    else if (EDMA_CHANNEL_TYPE_QDMA == chType)
    {
        /* Unmap QDMA Channel to Event Queue                               */
        qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
        qnumValue &= EDMACC_QDMAQNUM_CLR(chNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

/**
* Design: MMWLPSDK-761
*/
void EDMAMapQdmaChToPaRAM(uint32_t        baseAddr,
                           uint32_t        chNum,
                           const uint32_t *paRAMId)
{
    uint32_t qchmapValue;
    /* Map Parameter RAM Set Number for specified channelId             */
    qchmapValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum));
    qchmapValue &= EDMACC_QCHMAP_PAENTRY_CLR;
    qchmapValue |= (uint32_t) EDMACC_QCHMAP_PAENTRY_SET(*paRAMId);
    HW_WR_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum), qchmapValue);
}

/**
* Design: MMWLPSDK-762
*/
uint32_t EDMAGetMappedPaRAM(uint32_t baseAddr,
                             uint32_t chNum,
                             uint32_t chType,
                             uint32_t *paramId)
{
    uint32_t retVal = FALSE;
    if ((EDMA_CHANNEL_TYPE_DMA == chType) &&
        (chNum <= SOC_EDMA_NUM_DMACH))
    {

        /* Bug Fix - Changed to == */
        #if(SOC_EDMA_CHMAPEXIST)
        {
            *paramId = HW_RD_FIELD32(baseAddr + EDMA_TPCC_DCHMAPN(chNum),
                                    EDMA_TPCC_DCHMAPN_PAENTRY);
        }
        #else
        {
            *paramId = chNum;
        }
        retVal = TRUE;
        #endif
    }
    else if ((EDMA_CHANNEL_TYPE_QDMA == chType) &&
             (chNum <= SOC_EDMA_NUM_QDMACH))
    {
        *paramId = HW_RD_FIELD32(baseAddr + EDMA_TPCC_QCHMAPN(chNum),
                                EDMA_TPCC_QCHMAPN_PAENTRY);
        retVal = TRUE;
    }
    else
    {
        /*An error will be generated automatically.*/
    }
    return retVal;
}

/**
* Design: MMWLPSDK-763
*/
void EDMASetQdmaTrigWord(uint32_t baseAddr,
                          uint32_t chNum,
                          uint32_t trigWord)
{
    uint32_t qchmapValue;
    qchmapValue = HW_RD_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum));
    /* Clear QDMA Trigger word value */
    qchmapValue &= EDMACC_QCHMAP_TRWORD_CLR;
    /* Set the Trigger Word */
    qchmapValue |= EDMACC_QCHMAP_TRWORD_SET(trigWord);
    HW_WR_REG32(baseAddr + EDMA_TPCC_QCHMAPN(chNum), qchmapValue);
}

/**
* Design: MMWLPSDK-764
*/
void EDMAClrMissEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /*clear SECR to clean any previous NULL request */
        HW_WR_REG32(baseAddr + EDMA_TPCC_SECR(
                        regionId), (uint32_t) 0x01U << chNum);

        /*clear EMCR to clean any previous NULL request */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, (uint32_t) 0x01U << chNum);
    }
    else
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_SECRH(regionId),
                    (uint32_t) 0x01U << (chNum - 32U));
        /*clear EMCRH to clean any previous NULL request */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, (uint32_t) 0x01U << (chNum - 32U));
    }
}

/**
* Design: MMWLPSDK-765
*/
void EDMAQdmaClrMissEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    /*clear SECR to clean any previous NULL request  */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QSECR(
                    regionId), (uint32_t) 0x01U << chNum);

    /*clear EMCR to clean any previous NULL request  */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QEMCR, (uint32_t) 0x01U << chNum);
}

/**
* Design: MMWLPSDK-766
*/
void EDMAClrCCErr(uint32_t baseAddr, uint32_t flags)
{
    /* (CCERRCLR) - clear channel controller error register */
    HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR, flags);
}

/**
* Design: MMWLPSDK-767
*/
void EDMASetEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (ESR) - set corresponding bit to set a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ESR(
                        regionId), (uint32_t) 0x01U << chNum);
        //HW_WR_REG32((0x56001010U), (uint32_t) 0x2U);
        //HW_WR_REG32((0x56001010U), (uint32_t) (0x01U << chNum));
    }
    else
    {
        /* (ESRH) - set corresponding bit to set a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ESRH(regionId),
                    (uint32_t) 0x01U << (chNum - 32U));
    }
}

/**
* Design: MMWLPSDK-768
*/
void EDMAClrEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (ECR) - set corresponding bit to clear a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ECR(
                        regionId), (uint32_t) 0x01U << chNum);
    }
    else
    {
        /* (ECRH) - set corresponding bit to clear a event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_ECRH(regionId),
                    (uint32_t) 0x01U << (chNum - 32U));
    }
}

/**
* Design: MMWLPSDK-769
*/
void EDMAEnableDmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (EESR) - set corresponding bit to enable DMA event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EESR(
                        regionId), (uint32_t) 0x01U << chNum);
    }
    else
    {
        /* (EESRH) - set corresponding bit to enable DMA event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EESRH(regionId),
                    (uint32_t) 0x01U << (chNum - 32U));
    }
}

/**
* Design: MMWLPSDK-770
*/
void EDMADisableDmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* (EECR) - set corresponding bit to disable event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EECR(
                        regionId), (uint32_t) 0x01U << chNum);
    }
    else
    {
        /* (EECRH) - set corresponding bit to disable event */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EECRH(
                        regionId), (uint32_t) 0x01U << (chNum - 32U));
    }
}

/**
* Design: MMWLPSDK-771
*/
void EDMAEnableQdmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    /* (QEESR) - set corresponding bit to enable QDMA event                 */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QEESR(
                    regionId), (uint32_t) 0x01U << chNum);
}

/**
* Design: MMWLPSDK-772
*/
void EDMADisableQdmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    /* (QEESR) - set corresponding bit to disable QDMA event                 */
    HW_WR_REG32(baseAddr + EDMA_TPCC_QEECR(
                    regionId), (uint32_t) 0x01U << chNum);
}

/**
* Design: MMWLPSDK-792
*/
uint32_t EDMAGetCCErrStatus(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0U;
    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_CCERR);

    return intrStatusVal;
}

/**
* Design: MMWLPSDK-773
*/
uint32_t EDMAGetIntrStatusRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrStatusVal = 0U;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IPR(regionId));

    return intrStatusVal;
}

/**
* Design: MMWLPSDK-796
*/
uint32_t EDMAIntrStatusHighGetRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrStatusVal = 0U;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IPRH(regionId));

    return intrStatusVal;
}

/**
* Design: MMWLPSDK-797
*/
uint32_t EDMAReadIntrStatusRegion(uint32_t baseAddr, uint32_t regionId, uint32_t tccNum)
{
    uint32_t intrStatus = 0U;

    if(tccNum < 32U)
    {
        if ((EDMAGetIntrStatusRegion(baseAddr, regionId) & (0x1U << tccNum)) ==
            (0x1U << tccNum))
        {
            intrStatus = 1;
        }
    }
    else
    {
        if ((EDMAIntrStatusHighGetRegion(baseAddr, regionId) &
               (0x1U << (tccNum - 32U))) ==
               (0x1U << (tccNum - 32U)))
        {
            intrStatus = 1;
        }
    }
    return intrStatus;
}

/**
* Design: MMWLPSDK-793
*/
uint32_t EDMAGetErrIntrStatus(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0U;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_EMR);

    return intrStatusVal;
}

/**
* Design: MMWLPSDK-798
*/
uint32_t EDMAErrIntrHighStatusGet(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0U;

    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_EMRH);

    return intrStatusVal;
}

/**
* Design: MMWLPSDK-794
*/
uint32_t EDMAQdmaGetErrIntrStatus(uint32_t baseAddr)
{
    uint32_t intrStatusVal = 0U;
    intrStatusVal = HW_RD_REG32(baseAddr + EDMA_TPCC_QEMR);

    return intrStatusVal;
}

/**
* Design: MMWLPSDK-774
*/
void EDMAEnableEvtIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /*  Interrupt Enable Set Register (IESR) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IESR(
                        regionId), (uint32_t) 0x01U << chNum);
    }
    else
    {
        /*  Interrupt Enable Set Register (IESRH) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IESRH(regionId),
                    (uint32_t) 0x01U << (chNum - 32U));
    }
}

/**
* Design: MMWLPSDK-775
*/
void EDMADisableEvtIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum)
{
    if (chNum < 32U)
    {
        /* Interrupt Enable Clear Register (IECR) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IECR(
                        regionId), (uint32_t) 0x01U << chNum);
    }
    else
    {
        /* Interrupt Enable Clear Register (IECRH) */
        HW_WR_REG32(baseAddr + EDMA_TPCC_IECRH(regionId),
                    (uint32_t) 0x01U << (chNum - 32U));
    }
}

/**
* Design: MMWLPSDK-776
*/
void EDMAClrIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t value)
{
    if (value < 32U)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_ICR(
                        regionId), (uint32_t) 1U << value);
    }
    else
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_ICRH(regionId), (uint32_t) 1U <<
                    (value - 32U));
    }
}

/**
* Design: MMWLPSDK-777
*/
uint32_t EDMAGetEnabledIntrRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrEnableVal = 0U;

    intrEnableVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IER(regionId));

    return intrEnableVal;
}

/**
* Design: MMWLPSDK-778
*/
uint32_t EDMAGetEnabledIntrHighRegion(uint32_t baseAddr, uint32_t regionId)
{
    uint32_t intrEnableVal = 0U;

    intrEnableVal = HW_RD_REG32(baseAddr + EDMA_TPCC_IERH(regionId));

    return intrEnableVal;
}

/**
* Design: MMWLPSDK-779
*/
void EDMAGetPaRAM(uint32_t           baseAddr,
                   uint32_t           paRAMId,
                   EDMACCPaRAMEntry *currPaRAM)
{
    uint32_t  i = 0U;
    uint32_t sr;
    uint32_t *ds = (uint32_t *) currPaRAM;

    sr = baseAddr + EDMA_TPCC_OPT(paRAMId);

    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        *ds = HW_RD_REG32(sr);
        ds++;
        sr+= (uint32_t)sizeof(uint32_t);
    }
}

/**
* Design: MMWLPSDK-780
*/
void EDMAQdmaGetPaRAM(uint32_t           baseAddr,
                       uint32_t           paRAMId,
                       EDMACCPaRAMEntry *currPaRAM)
{
    uint32_t  i = 0U;
    uint32_t *ds     = (uint32_t *) currPaRAM;
    uint32_t  sr     = baseAddr + EDMA_TPCC_OPT(paRAMId);

    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        *ds = HW_RD_REG32(sr);
        ds++;
        sr+= (uint32_t)sizeof(uint32_t);
    }
}


/**
* Design: MMWLPSDK-781
*/
void EDMASetPaRAM(uint32_t           baseAddr,
                   uint32_t           paRAMId,
                   const EDMACCPaRAMEntry *newPaRAM)
{
    uint32_t           i  = 0U;
    uint32_t          *sr = (uint32_t *) newPaRAM;
    volatile uint32_t  ds;
    uint32_t           dsAddr =baseAddr + EDMA_TPCC_OPT(paRAMId);
    volatile EDMACCPaRAMEntry *paramcp = (EDMACCPaRAMEntry *)newPaRAM;
    ds = (uint32_t ) (dsAddr);
    

    if(paramcp->destBIdx <0)
    {
        paramcp->destBIdxExt = (int8_t)0xFF;
    }
    else
    {
        paramcp->destBIdxExt = (int8_t)0x00;
    }

    if(paramcp->srcBIdx <0)
    {
        paramcp->srcBIdxExt = (int8_t)0xFF;
    }
    else
    {
        paramcp->srcBIdxExt = (int8_t)0x00;
    }
    
    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        HW_WR_REG32(ds, *sr);
        ds+= (uint32_t)sizeof(uint32_t);
        sr++;
    }

    
}

/**
* Design: MMWLPSDK-782
*/
void EDMAQdmaSetPaRAM(uint32_t           baseAddr,
                       uint32_t           paRAMId,
                       const EDMACCPaRAMEntry *newPaRAM)
{
    uint32_t  i  = 0U;
    uint32_t *sr = (uint32_t *) newPaRAM;
    uint32_t  ds;
    uint32_t  dsAddr =baseAddr + EDMA_TPCC_OPT(paRAMId);

    ds = (uint32_t ) (dsAddr);

    for (i = 0; i < EDMACC_PARAM_ENTRY_FIELDS; i++)
    {
        HW_WR_REG32(ds, *sr);
        ds+= (uint32_t)sizeof(uint32_t);
        sr++;
    }
}

/**
* Design: MMWLPSDK-783
*/
void EDMAQdmaSetPaRAMEntry(uint32_t baseAddr,
                            uint32_t paRAMId,
                            uint32_t paRAMEntry,
                            uint32_t newPaRAMEntryVal)
{
    EDMADmaSetPaRAMEntry(baseAddr, paRAMId,
                          paRAMEntry, newPaRAMEntryVal);
}

/**
* Design: MMWLPSDK-784
*/
uint32_t EDMAQdmaGetPaRAMEntry(uint32_t baseAddr,
                                uint32_t paRAMId,
                                uint32_t paRAMEntry)
{
    return EDMADmaGetPaRAMEntry(baseAddr, paRAMId,
                          paRAMEntry);
}

/**
* Design: MMWLPSDK-785
*/
void EDMADmaSetPaRAMEntry(uint32_t baseAddr,
                            uint32_t paRAMId,
                            uint32_t paRAMEntry,
                            uint32_t newPaRAMEntryVal)
{
    if (paRAMEntry <= EDMACC_PARAM_ENTRY_CCNT)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_OPT(paRAMId) +
                    (paRAMEntry * 0x04U), newPaRAMEntryVal);
    }
}

/**
* Design: MMWLPSDK-786
*/
uint32_t EDMADmaGetPaRAMEntry(uint32_t baseAddr,
                                uint32_t paRAMId,
                                uint32_t paRAMEntry)
{
    uint32_t paRAMEntryVal = 0U;
    if (paRAMEntry <= EDMACC_PARAM_ENTRY_CCNT)
    {
        paRAMEntryVal = HW_RD_REG32(baseAddr + EDMA_TPCC_OPT(paRAMId) +
                                    (paRAMEntry * 0x04U));
    }
    return (paRAMEntryVal);
}

/**
* Design: MMWLPSDK-787
*/
uint32_t EDMAConfigureChannelRegion(uint32_t baseAddr,
                                     uint32_t regionId,
                                     uint32_t chType,
                                     uint32_t chNum,
                                     uint32_t tccNum,
                                     uint32_t paramId,
                                     uint32_t evtQNum)
{
    uint32_t optValue;
    uint32_t retVal = FALSE;
    if (((EDMA_CHANNEL_TYPE_DMA == chType) && (chNum < SOC_EDMA_NUM_DMACH)) ||
        ((EDMA_CHANNEL_TYPE_QDMA == chType) && (chNum < SOC_EDMA_NUM_QDMACH)))
    {
        /* Enable the DMA channel in the enabled in the shadow region
         * specific register
         */
        EDMAEnableChInShadowRegRegion(baseAddr, regionId, chType, chNum);

        EDMAMapChToEvtQ(baseAddr, chType, chNum, evtQNum);

        if (EDMA_CHANNEL_TYPE_DMA == chType)
        {
            EDMAChannelToParamMap(baseAddr, chNum, paramId);
        }
        else
        {
            EDMAMapQdmaChToPaRAM(baseAddr, chNum, &paramId);
        }

        optValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_OPT(paramId));
        optValue &= EDMACC_OPT_TCC_CLR;
        optValue |= EDMACC_OPT_TCC_SET(tccNum);
        HW_WR_REG32(baseAddr + EDMA_TPCC_OPT(paramId), optValue);

        retVal = (uint32_t) TRUE;
    }
    return retVal;
}

/**
* Design: MMWLPSDK-788
*/
uint32_t EDMAFreeChannelRegion(uint32_t baseAddr,
                                uint32_t regionId,
                                uint32_t chType,
                                uint32_t chNum,
                                uint32_t trigMode,
                                uint32_t tccNum,
                                uint32_t evtQNum)
{
    uint32_t retVal = FALSE;
    if ((chNum < SOC_EDMA_NUM_DMACH) &&
        ((EDMA_CHANNEL_TYPE_DMA == chType) || (EDMA_CHANNEL_TYPE_QDMA == chType)))
    {
        EDMADisableTransferRegion(baseAddr, regionId, chNum, trigMode);
        /* Disable the DMA channel in the shadow region specific register
         */
        EDMADisableChInShadowRegRegion(baseAddr, regionId, chType, chNum);

        EDMAUnmapChToEvtQ(baseAddr, chType, chNum);

        retVal = (uint32_t) TRUE;
    }
    return retVal;
}

/**
* Design: MMWLPSDK-789
*/
uint32_t EDMAEnableTransferRegion(uint32_t baseAddr,
                                   uint32_t regionId,
                                   uint32_t chNum,
                                   uint32_t trigMode)
{
    uint32_t retVal = FALSE;
    switch (trigMode)
    {
        case EDMA_TRIG_MODE_MANUAL:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                EDMASetEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_QDMA:
            if (chNum < SOC_EDMA_NUM_QDMACH)
            {
                EDMAEnableQdmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_EVENT:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                /*clear SECR & EMCR to clean any previous NULL request    */
                EDMAClrMissEvtRegion(baseAddr, regionId, chNum);

                /* Set EESR to enable event                               */
                EDMAEnableDmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        default:
            retVal = (uint32_t) FALSE;
            break;
    }
    return retVal;
}

/**
* Design: MMWLPSDK-790
*/
uint32_t EDMADisableTransferRegion(uint32_t baseAddr,
                                    uint32_t regionId,
                                    uint32_t chNum,
                                    uint32_t trigMode)
{
    uint32_t retVal = FALSE;
    switch (trigMode)
    {
        case EDMA_TRIG_MODE_MANUAL:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                EDMAClrEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_QDMA:
            if (chNum < SOC_EDMA_NUM_QDMACH)
            {
                EDMADisableQdmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        case EDMA_TRIG_MODE_EVENT:
            if (chNum < SOC_EDMA_NUM_DMACH)
            {
                /*clear SECR & EMCR to clean any previous NULL request    */
                EDMAClrMissEvtRegion(baseAddr, regionId, chNum);

                /* Set EESR to enable event                               */
                EDMADisableDmaEvtRegion(baseAddr, regionId, chNum);
                retVal = (uint32_t) TRUE;
            }
            break;

        default:
            retVal = (uint32_t) FALSE;
            break;
    }
    return retVal;
}

/**
* Design: MMWLPSDK-791
*/
void EDMAClearErrorBitsRegion(uint32_t baseAddr,
                               uint32_t regionId,
                               uint32_t chNum,
                               uint32_t evtQNum)
{
    if (chNum < SOC_EDMA_NUM_DMACH)
    {
        if (chNum < 32U)
        {
            HW_WR_REG32(baseAddr + EDMA_TPCC_EECR(
                            regionId), (uint32_t) 0x01U << chNum);
            /* Write to EMCR to clear the corresponding EMR bit */
            HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, (uint32_t) 0x01U << chNum);
            /* Clears the SER */
            HW_WR_REG32(baseAddr + EDMA_TPCC_SECR(
                            regionId), (uint32_t) 0x01U << chNum);
        }
        else
        {
            HW_WR_REG32(baseAddr +
                        EDMA_TPCC_EECRH(regionId), (uint32_t) 0x01U <<
                        (chNum - 32U));
            /* Write to EMCR to clear the corresponding EMR bit */
            HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, (uint32_t) 0x01U <<
                        (chNum - 32U));
            /* Clears the SER */
            HW_WR_REG32(baseAddr +
                        EDMA_TPCC_SECRH(regionId), (uint32_t) 0x01U <<
                        (chNum - 32U));
        }
    }

    /* Clear the global CC Error Register */
    if (0U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD0_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
    else if (1U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD1_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
#if SOC_EDMA_NUM_EVQUE > 2
    else if (2U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD2_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
#if SOC_EDMA_NUM_EVQUE > 3
    else if (3U == evtQNum)
    {
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR,
                    EDMA_TPCC_CCERRCLR_QTHRXCD3_MASK |
                    EDMA_TPCC_CCERRCLR_TCERR_MASK);
    }
#endif
#endif
    else
    {
        /*Error will be generated automatically*/
    }
}
/**
* Design: MMWLPSDK-820
*/
static int32_t EDMADeinitialize (uint32_t baseAddr, const EDMA_InitParams *initParam)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t count = 0U;
    uint32_t qnumValue;

    if (initParam == NULL)
    {
        retVal = SystemP_FAILURE;
    }
    if (retVal == SystemP_SUCCESS)
    {
        /* Disable the DMA (0 - 62) channels in the DRAE register */
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEM(
                        initParam->regionId), EDMA_CLR_ALL_BITS);
        HW_WR_REG32(baseAddr + EDMA_TPCC_DRAEHM(
                        initParam->regionId), EDMA_CLR_ALL_BITS);

        EDMAClrCCErr(baseAddr, EDMACC_CLR_TCCERR);

        /* Clear the Event miss Registers                      */
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCR, initParam->ownResource.dmaCh[0]);
    #if SOC_EDMA_NUM_DMACH > 32
        HW_WR_REG32(baseAddr + EDMA_TPCC_EMCRH, initParam->ownResource.dmaCh[1]);
    #endif
        /* Clear CCERR register */
        HW_WR_REG32(baseAddr + EDMA_TPCC_CCERRCLR, initParam->ownResource.qdmaCh);

        /* Disable and clear channel interrupts for all dma channels */
        for (count = 0; count < EDMA_NUM_TCC; count++)
        {
            if (((1U << (count%32U)) & initParam->ownResource.tcc[count/32U]) != 0U)
            {
                EDMADisableEvtIntrRegion(baseAddr, initParam->regionId, count);
                EDMAClrIntrRegion(baseAddr, initParam->regionId, count);
            }
        }
        /* Disable and clear channel interrupts for all qdma channels */
        for (count = 0; count < SOC_EDMA_NUM_QDMACH; count++)
        {
            if (((1U << count) & initParam->ownResource.qdmaCh) != 0U)
            {
                EDMADisableQdmaEvtRegion(baseAddr, initParam->regionId, count);
                EDMAQdmaClrMissEvtRegion(baseAddr, initParam->regionId, count);
            }
        }

        /* Deinitialize the Queue Number Registers */
        for (count = 0; count < SOC_EDMA_NUM_DMACH; count++)
        {
            if (((1U << (count%32U)) & initParam->ownResource.dmaCh[count/32U]) != 0U)
            {
                qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_DMAQNUMN((count >> 3U)));
                qnumValue &= EDMACC_DMAQNUM_CLR(count);
                HW_WR_REG32(baseAddr + EDMA_TPCC_DMAQNUMN((count >> 3U)), qnumValue);
            }
        }

        for (count = 0; count < SOC_EDMA_NUM_QDMACH; count++)
        {
            if (((1U << count) & initParam->ownResource.qdmaCh) != 0U)
            {
                qnumValue  = HW_RD_REG32(baseAddr + EDMA_TPCC_QDMAQNUM);
                qnumValue &= EDMACC_QDMAQNUM_CLR(count);
                HW_WR_REG32(baseAddr + EDMA_TPCC_QDMAQNUM, qnumValue);
            }
        }
    }
    return retVal;
}

/**
* Design: MMWLPSDK-799
*/
void EDMAChainChannel(uint32_t baseAddr,
                       uint32_t chId1,
                       uint32_t chId2,
                       uint32_t chainOptions)
{
    EDMACCPaRAMEntry *currPaRAM     = NULL;
    uint32_t           currPaRAMAddr = baseAddr + EDMA_TPCC_OPT(chId1);
    uint32_t           optVal;
    uintptr_t          optAddr;

    /* Get param set for the channel Id passed*/
    currPaRAM = (EDMACCPaRAMEntry *) (currPaRAMAddr);

    optAddr    = (uintptr_t) &currPaRAM->opt;
    optVal = HW_RD_REG32((uint32_t) optAddr);
    optVal &= ~(EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
               EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK);
    optVal |= chainOptions;
    optVal &= ~EDMA_TPCC_OPT_TCC_MASK;
    optVal |= (chId2 << EDMA_TPCC_OPT_TCC_SHIFT) & EDMA_TPCC_OPT_TCC_MASK;
    HW_WR_REG32((uint32_t) optAddr, optVal);
}

/**
* Design: MMWLPSDK-800
*/
void EDMALinkChannel(uint32_t baseAddr,
                      uint32_t paRAMId1,
                      uint32_t paRAMId2)
{
    EDMACCPaRAMEntry *currPaRAM1;
    EDMACCPaRAMEntry *currPaRAM2;
    uint32_t           optVal1, optVal2;
    uint32_t           currPaRAMAddr1 = baseAddr + EDMA_TPCC_OPT(paRAMId1);
    uint32_t           currPaRAMAddr2 = baseAddr + EDMA_TPCC_OPT(paRAMId2);
    uintptr_t          lnkAddr;

    /* Get param set for the paRAMId1 passed*/
    currPaRAM1 = (EDMACCPaRAMEntry *) (currPaRAMAddr1);

    /* Update the Link field with lch2 PaRAM set */
    lnkAddr = (uintptr_t) &currPaRAM1->linkAddr;
    HW_WR_REG16(lnkAddr,
        (uint16_t) ((baseAddr + EDMA_TPCC_OPT(paRAMId2)) & (uint16_t) 0x0FFFF));

    /* Get param set for the paRAMId2 passed*/
    currPaRAM2 = (EDMACCPaRAMEntry *) (currPaRAMAddr2);

    /*Updated TCC value of param2 with that of param1*/
    optVal1 = HW_RD_REG32((uint32_t) &currPaRAM1->opt);
    optVal2 = HW_RD_REG32((uint32_t) &currPaRAM2->opt);
    optVal2 &= ~EDMA_TPCC_OPT_TCC_MASK;
    optVal2 |= optVal1 & EDMA_TPCC_OPT_TCC_MASK;
    HW_WR_REG32((uint32_t) &currPaRAM2->opt, optVal2);
}

/**
* Design: MMWLPSDK-801
*/
void EDMA_init(void)
{
    uint32_t        cnt;
    EDMA_Object    *object;
    uint32_t        baseAddr;
    
    /* Init each driver instance object */
    for (cnt = 0U; cnt < gEdmaConfigNum; cnt++)
    {
        /* initialize object varibles */
        object = gEdmaConfig[cnt].object;
        DebugP_assert(NULL != object);
        memset(object, 0, sizeof(EDMA_Object));
        /* Get the edma base address. */
        baseAddr = gEdmaConfig[cnt].attrs->baseAddr;
        /* Initialize the EDMA hardware. */
        EDMAInitialize(baseAddr, &gEdmaConfig[cnt].attrs->initPrms);
    }

    return;
}

/**
* Design: MMWLPSDK-802
*/
void EDMA_deinit(void)
{
    uint32_t        cnt;
    EDMA_Object    *object;
    uint32_t        baseAddr;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gEdmaConfigNum; cnt++)
    {
        /* initialize object varibles */
        object = gEdmaConfig[cnt].object;
        DebugP_assert(NULL != object);
        memset(object, 0, sizeof(EDMA_Object));
        /* Get the edma base address. */
        baseAddr = gEdmaConfig[cnt].attrs->baseAddr;
        /* Initialize the EDMA hardware. */
        EDMADeinitialize(baseAddr, &gEdmaConfig[cnt].attrs->initPrms);
    }
    return;
}

/**
* Design: MMWLPSDK-803
*/
EDMA_Handle EDMA_open(uint32_t index, const EDMA_Params *prms)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Handle         handle = NULL;
    EDMA_Config        *config = NULL;
    EDMA_Object        *object    = NULL;
    HwiP_Params         hwiPrms;
    
    /* Check index */
    if(index >= gEdmaConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gEdmaConfig[index];
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_assert(NULL != config->object);
        DebugP_assert(NULL != config->attrs);
        object = config->object;
        if(TRUE == object->isOpen)
        {
            /* Handle is already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Init state */
        object->handle = (EDMA_Handle) config;
        DebugP_assert(NULL != prms);

        /* Store the open params in driver object. */
        memmove(&object->openPrms, prms, sizeof(EDMA_Params));

        if (prms->intrEnable == TRUE)
        {
            /* Register the master ISR. */
            /* Enable the aggregated interrupt. */
            HW_WR_REG32(config->attrs->intrAggEnableAddr, config->attrs->intrAggEnableMask);

            /* Register interrupt */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum   = config->attrs->compIntrNumber;
            hwiPrms.callback = &EDMA_transferCompletionMasterIsrFxn;
            hwiPrms.args     = (void *)object->handle;
            status = HwiP_construct(&object->hwiObj, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
            object->hwiHandle = &object->hwiObj;
        }
        object->firstIntr = NULL;
        object->isOpen = TRUE;

        handle = (EDMA_Handle) config;
    }
    return handle;
}

/**
* Design: MMWLPSDK-805
*/
void EDMA_close(EDMA_Handle handle)
{
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    config = (EDMA_Config *) handle;

    if((NULL != config) &&
       (config->object != NULL) &&
       (config->object->isOpen != (uint32_t)FALSE))
    {
        object = config->object;
        attrs = config->attrs;

        DebugP_assert(NULL != object);
        DebugP_assert(NULL != attrs);

        if(NULL != object->hwiHandle)
        {
            HwiP_destruct(&object->hwiObj);
            object->hwiHandle = NULL;
        }

        object->isOpen = FALSE;
    }
}

/**
* Design: MMWLPSDK-804
*/
EDMA_Handle EDMA_getHandle(uint32_t index)
{
    EDMA_Handle         handle = NULL;
    /* Check index */
    if(index < gEdmaConfigNum)
    {
        EDMA_Object *object;

        object = gEdmaConfig[index].object;
        if((object != NULL) && (TRUE == object->isOpen))
        {
            /* valid handle */
            handle = (EDMA_Handle)&(gEdmaConfig[index]);
        }
    }

    return handle;
}

/**
* Design: MMWLPSDK-806
*/
uint32_t EDMA_isInterruptEnabled(EDMA_Handle handle)
{
    EDMA_Object    *object = ((EDMA_Config *)handle)->object;
    EDMA_Params    *openParams = &(object->openPrms);
    return (openParams->intrEnable);
}

/**
* Design: MMWLPSDK-899
*/
static int32_t EDMA_validateIntrObject(Edma_IntrObject *intrObj)
{
    int32_t             status = SystemP_SUCCESS;
    if (intrObj->cbFxn == NULL)
    {
        /* Callback function cant be NULL for registering interrupt. */
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        /* TODO: validate tccnum */
    }
    return status;
}

/**
* Design: MMWLPSDK-807
*/
int32_t EDMA_registerIntr(EDMA_Handle handle, Edma_IntrObject *intrObj)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState;

    if ((handle == NULL) || (intrObj == NULL))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        status = EDMA_validateIntrObject(intrObj);
    }
    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if((NULL != config) &&
        (config->object != NULL) &&
        (config->object->isOpen != (uint32_t)FALSE) &&
        (config->object->openPrms.intrEnable != FALSE))
        {
            object = config->object;
            attrs = config->attrs;

            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);

            intrState = HwiP_disable();
            if (object->firstIntr == NULL)
            {
                object->firstIntr = intrObj;
                intrObj->nextIntr = NULL;
                intrObj->prevIntr = NULL;
            }
            else
            {
                /* Insert the intrObj at the end of the list. */
                Edma_IntrObject *tempObj;
                tempObj = (Edma_IntrObject*)(object->firstIntr);
                while (tempObj->nextIntr != NULL)
                {
                    tempObj = (Edma_IntrObject*)(tempObj->nextIntr);
                }
                tempObj->nextIntr = intrObj;
                intrObj->nextIntr = NULL;
                intrObj->prevIntr = tempObj;
            }
            HwiP_restore (intrState);

            /* Enable the tcc interrupt bit. */
            EDMAEnableEvtIntrRegion(attrs->baseAddr, attrs->initPrms.regionId, intrObj->tccNum);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    return status;
}

/**
* Design: MMWLPSDK-808
*/
int32_t EDMA_unregisterIntr(EDMA_Handle handle, Edma_IntrObject *intrObj)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState;

    if ((handle == NULL) || (intrObj == NULL))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if((NULL != config) &&
        (config->object != NULL) &&
        (config->object->isOpen != (uint32_t)FALSE))
        {
            Edma_IntrObject *tempObj;
            uint32_t                  elementFound = (uint32_t)FALSE;
            object = config->object;
            attrs = config->attrs;

            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);

            intrState = HwiP_disable();
            /* Find the intrObj in the list. */
            if (object->firstIntr == intrObj)
            {
                tempObj = (Edma_IntrObject*)(object->firstIntr);
                object->firstIntr = tempObj->nextIntr;
                elementFound = TRUE;
            }
            else
            {
                tempObj = (Edma_IntrObject*)(object->firstIntr);
                while ((tempObj != NULL) && (tempObj != intrObj))
                {
                    tempObj = (Edma_IntrObject*) (tempObj->nextIntr);
                }
                if (tempObj == intrObj)
                {
                    Edma_IntrObject *prevObj;
                    /* Element is found. Remove current object from list. */
                    prevObj =(Edma_IntrObject*)(tempObj->prevIntr);
                    prevObj->nextIntr = tempObj->nextIntr;
                    elementFound = TRUE;
                }
            }
            HwiP_restore (intrState);

            if (elementFound == TRUE)
            {
                intrObj->prevIntr = NULL;
                intrObj->nextIntr = NULL;
                /* Disable the tcc interrupt bit. */
                EDMADisableEvtIntrRegion(attrs->baseAddr, attrs->initPrms.regionId, intrObj->tccNum);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    return status;
}

/**
* Design: MMWLPSDK-809
*/
uint32_t EDMA_getBaseAddr(EDMA_Handle handle)
{
    EDMA_Config        *config;
    const EDMA_Attrs   *attrs;
    uint32_t            baseAddr = 0U;

    if (handle != NULL)
    {
        config = (EDMA_Config *) handle;

        if((config->object != NULL) &&
           (config->object->isOpen != (uint32_t)FALSE))
        {
             attrs = config->attrs;
             DebugP_assert(NULL != attrs);
             baseAddr = attrs->baseAddr;
        }
    }
    return baseAddr;
}

/**
* Design: MMWLPSDK-810
*/
uint32_t EDMA_getRegionId(EDMA_Handle handle)
{
    EDMA_Config        *config;
    uint32_t            regionId = SOC_EDMA_NUM_REGIONS;

    if (handle != NULL)
    {
        config = (EDMA_Config *) handle;

        if((config->object != NULL) &&
           (config->object->isOpen != (uint32_t)FALSE))
        {
            regionId = config->attrs->initPrms.regionId;
        }
    }
    return regionId;
}

/**
* Design: MMWLPSDK-811
*/
int32_t EDMA_allocDmaChannel(EDMA_Handle handle, uint32_t *dmaCh)
{
    return (EDMA_allocResource(handle, EDMA_RESOURCE_TYPE_DMA, dmaCh));
}

/**
* Design: MMWLPSDK-812
*/
int32_t EDMA_allocQdmaChannel(EDMA_Handle handle, uint32_t *qdmaCh)
{
    return (EDMA_allocResource(handle, EDMA_RESOURCE_TYPE_QDMA, qdmaCh));
}

/**
* Design: MMWLPSDK-813
*/
int32_t EDMA_allocTcc(EDMA_Handle handle, uint32_t *tcc)
{
    return (EDMA_allocResource(handle, EDMA_RESOURCE_TYPE_TCC, tcc));
}

/**
* Design: MMWLPSDK-814
*/
int32_t EDMA_allocParam(EDMA_Handle handle, uint32_t *param)
{
    return (EDMA_allocResource(handle, EDMA_RESOURCE_TYPE_PARAM, param));
}

/**
* Design: MMWLPSDK-822
*/
static int32_t EDMA_allocResource(EDMA_Handle handle, uint32_t resType, uint32_t *resId)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;

    if ((handle == NULL) || (resId == NULL))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if((NULL != config) &&
        (config->object != NULL) &&
        (config->object->isOpen != (uint32_t)FALSE))
        {
            object = config->object;
            attrs = config->attrs;

            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);
            switch (resType)
            {
                case EDMA_RESOURCE_TYPE_DMA:
                    status = allocResource(attrs, object, resId, EDMA_RESOURCE_TYPE_DMA);
                    break;
                case EDMA_RESOURCE_TYPE_QDMA:
                    status = allocResource(attrs, object, resId, EDMA_RESOURCE_TYPE_QDMA);
                    break;
                case EDMA_RESOURCE_TYPE_TCC:
                    status = allocResource(attrs, object, resId, EDMA_RESOURCE_TYPE_TCC);
                    break;
                case EDMA_RESOURCE_TYPE_PARAM:
                    status = allocResource(attrs, object, resId, EDMA_RESOURCE_TYPE_PARAM);
                    break;
                default:
                    DebugP_assert(FALSE);
                    break;
            }
        }
    }
    return status;
}

/**
* Design: MMWLPSDK-823
*/
static int32_t allocResource(const EDMA_Attrs *attrs, EDMA_Object *object, uint32_t *resId, uint32_t resType)
{
    uint32_t    i,j;
    bool flag;
    uintptr_t   intrState;
    int32_t     status = SystemP_SUCCESS;
    uint32_t    *allocPtr;
    const uint32_t *ownPtr, *reservedPtr;
    uint32_t    resPtrLen, maxRes;
    switch (resType)
    {
        case EDMA_RESOURCE_TYPE_DMA:
            allocPtr = &object->allocResource.dmaCh[0];
            ownPtr = &attrs->initPrms.ownResource.dmaCh[0];
            reservedPtr = &attrs->initPrms.reservedDmaCh[0];
            resPtrLen = SOC_EDMA_NUM_DMACH/32U;
            maxRes = SOC_EDMA_NUM_DMACH;
            break;
        case EDMA_RESOURCE_TYPE_QDMA:
            allocPtr = &object->allocResource.qdmaCh;
            ownPtr = &attrs->initPrms.ownResource.qdmaCh;
            resPtrLen = 1;
            maxRes = SOC_EDMA_NUM_QDMACH;
            break;
        case EDMA_RESOURCE_TYPE_TCC:
            allocPtr = &object->allocResource.tcc[0];
            ownPtr = &attrs->initPrms.ownResource.tcc[0];
            resPtrLen = SOC_EDMA_NUM_DMACH/32U;
            maxRes = SOC_EDMA_NUM_DMACH;
            break;
        case EDMA_RESOURCE_TYPE_PARAM:
            allocPtr = &object->allocResource.paramSet[0];
            ownPtr = &attrs->initPrms.ownResource.paramSet[0];
            resPtrLen = SOC_EDMA_NUM_PARAMSETS/32U;
            maxRes = SOC_EDMA_NUM_PARAMSETS;
            break;
        default:
            status = SystemP_FAILURE;
            break;
    }

    if (status == SystemP_SUCCESS)
    {
        /* set the status to failure.
           If allocation is successful status will be updated. */
        status = SystemP_FAILURE;
        intrState = HwiP_disable();
        if (*resId == EDMA_RESOURCE_ALLOC_ANY)
        {
            /* Find available resource. */
            for (i=0; i < resPtrLen; i++)
            {
                flag = false;
                for (j=0U; j<32U; j++)
                {
                    if (resType == EDMA_RESOURCE_TYPE_DMA)
                    {
                        /* Check if the dma channel is owned and available and not reserved. */
                        if (((ownPtr[i] & (1U << j)) != 0U) &&
                            ((reservedPtr[i] & (1U << j)) == 0U) &&
                            ((allocPtr[i] & (1U << j)) == 0U))
                        {
                            *resId = (i * 32U) + j;
                            allocPtr[i] |= 1U << (j%32U);
                            status = SystemP_SUCCESS;
                            flag =true;
                        }
                    }
                    else
                    {
                        /* Check if the resource is owned and available. */
                        if (((ownPtr[i] & (1U << j)) !=  0U) &&
                            ((allocPtr[i] & (1U << j)) == 0U))
                        {
                            *resId = (i * 32U) + j;
                            allocPtr[i] |= 1U << (j%32U);
                            status = SystemP_SUCCESS;
                            flag=true;
                        }
                    }
                    if(flag)
                    {
                        j = 32U;
                        flag = false;
                    }
                }
                if (*resId != EDMA_RESOURCE_ALLOC_ANY)
                {
                    flag=true;
                }
                if(flag)
                {
                  i=resPtrLen;
                }
            }
        }
        else
        {
            /* Check if the resource is already allocated. */
            if ((*resId < maxRes) &&
                ((ownPtr[*resId/32U] & (1U << (*resId%32U))) != 0U) &&
                ((allocPtr[*resId/32U] & (1U << (*resId%32U))) == 0U))
            {
                allocPtr[*resId/32U] |= 1U << (*resId%32U);
                status = SystemP_SUCCESS;
            }
        }
        HwiP_restore(intrState);
    }
    return status;
}

/**
* Design: MMWLPSDK-815
*/
int32_t EDMA_freeDmaChannel(EDMA_Handle handle, uint32_t *dmaCh)
{
    return (EDMA_freeResource(handle, EDMA_RESOURCE_TYPE_DMA, dmaCh));
}

/**
* Design: MMWLPSDK-816
*/
int32_t EDMA_freeQdmaChannel(EDMA_Handle handle, uint32_t *qdmaCh)
{
    return (EDMA_freeResource(handle, EDMA_RESOURCE_TYPE_QDMA, qdmaCh));
}

/**
* Design: MMWLPSDK-817
*/
int32_t EDMA_freeTcc(EDMA_Handle handle, uint32_t *tcc)
{
    return (EDMA_freeResource(handle, EDMA_RESOURCE_TYPE_TCC, tcc));
}

/**
* Design: MMWLPSDK-818
*/
int32_t EDMA_freeParam(EDMA_Handle handle, uint32_t *param)
{
    return (EDMA_freeResource(handle, EDMA_RESOURCE_TYPE_PARAM, param));
}

/**
* Design: MMWLPSDK-824
*/
static int32_t EDMA_freeResource(EDMA_Handle handle, uint32_t resType, uint32_t *resId)
{
    int32_t             status = SystemP_SUCCESS;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    uintptr_t           intrState;

    if ((handle == NULL) || (resId == NULL))
    {
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) &&
        (((resType == EDMA_RESOURCE_TYPE_DMA  ) && (*resId >= SOC_EDMA_NUM_DMACH    )) ||
        ((resType == EDMA_RESOURCE_TYPE_QDMA ) && (*resId >= SOC_EDMA_NUM_QDMACH   )) ||
        ((resType == EDMA_RESOURCE_TYPE_TCC  ) && (*resId >= SOC_EDMA_NUM_DMACH    )) ||
        ((resType == EDMA_RESOURCE_TYPE_PARAM) && (*resId >= SOC_EDMA_NUM_PARAMSETS))))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        config = (EDMA_Config *) handle;

        if((NULL != config) &&
        (config->object != NULL) &&
        (config->object->isOpen != (uint32_t)FALSE))
        {
            object = config->object;
            attrs = config->attrs;

            DebugP_assert(NULL != object);
            DebugP_assert(NULL != attrs);
            intrState = HwiP_disable();
            switch (resType)
            {
                case EDMA_RESOURCE_TYPE_DMA:
                    object->allocResource.dmaCh[*resId/32U] &= (~(1U << (*resId%32U)));
                    break;
                case EDMA_RESOURCE_TYPE_QDMA:
                    object->allocResource.qdmaCh &= (~(1U << (*resId%32U)));
                    break;
                case EDMA_RESOURCE_TYPE_TCC:
                    object->allocResource.tcc[*resId/32U] &= (~(1U << (*resId%32U)));
                    break;
                case EDMA_RESOURCE_TYPE_PARAM:
                    object->allocResource.paramSet[*resId/32U] &= (~(1U << (*resId%32U)));
                    break;
                default:
                    status = SystemP_FAILURE;
                    break;
            }
            HwiP_restore(intrState);
        }
    }
    return status;
}

/**
* Design: MMWLPSDK-821
*/
static void EDMA_transferCompletionMasterIsrFxn(void *args)
{
    EDMA_Handle         handle = (EDMA_Handle) args;
    EDMA_Config        *config;
    EDMA_Object        *object;
    const EDMA_Attrs   *attrs;
    Edma_IntrObject *intrObj;
    uint32_t baseAddr, regionId;
    uint32_t intrLow, intrHigh;

    DebugP_assert(NULL != handle);
    config = (EDMA_Config *) handle;

    object = config->object;
    attrs = config->attrs;
    DebugP_assert(NULL != object);
    DebugP_assert(NULL != attrs);

    baseAddr = attrs->baseAddr;
    regionId = attrs->initPrms.regionId;

    intrLow = EDMAGetIntrStatusRegion(baseAddr, regionId);
    intrHigh = EDMAIntrStatusHighGetRegion(baseAddr, regionId);

    intrObj = (Edma_IntrObject*)(object->firstIntr);
    while ((intrObj != NULL) && ((intrLow != 0U) || (intrHigh != 0U)))
    {
        if ((intrObj->tccNum < 32U) && ((intrLow & (1U << intrObj->tccNum)) != 0U))
        {
            EDMAClrIntrRegion(baseAddr, regionId, intrObj->tccNum);
            intrLow &= ~(1U << intrObj->tccNum);
            intrObj->cbFxn(intrObj, intrObj->appData);
        }
        if ((intrObj->tccNum >= 32U) && ((intrHigh & (1U << (intrObj->tccNum - 32U))) != 0U))
        {
            EDMAClrIntrRegion(baseAddr, regionId, intrObj->tccNum);
            intrHigh &= ~(1U << (intrObj->tccNum - 32U));
            intrObj->cbFxn(intrObj, intrObj->appData);
        }
        /* Get next intr Obj. */
        intrObj = (Edma_IntrObject*)(intrObj->nextIntr);
    }

    /* Clear the aggregator interrupt */
    HW_WR_REG32(attrs->intrAggStatusAddr, attrs->intrAggClearMask);
    /* re evaluate the edma interrupt. */
    HW_WR_FIELD32(baseAddr + EDMA_TPCC_IEVAL(regionId), EDMA_TPCC_IEVAL_EVAL, 1);
}
