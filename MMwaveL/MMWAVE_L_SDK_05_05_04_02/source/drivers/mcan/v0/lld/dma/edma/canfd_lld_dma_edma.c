/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 */

/**
 *  \file canfd_lld_dma_edma.c
 *
 *  \brief File containing EDMA Driver APIs implementation for MCAN.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/soc.h>
#include <drivers/edma.h>
#include <drivers/mcan/v0/lld/dma/edma/canfd_lld_dma_edma.h>
#include <drivers/mcan/v0/lld/canfd_lld.h>
#include <kernel/dpl/CacheP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Transmit EDMA channel event queue number                           */
#define EDMA_CANFD_TX_EVT_QUEUE_NO                  (0U)
/** \brief Receive EDMA channel event queue number                            */
#define EDMA_CANFD_RX_EVT_QUEUE_NO                  (1U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void CANFD_lld_dmaTxCallBack(CANFDLLD_MessageObject* ptrCanMsgObj);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CANFD_lld_dmaOpen(CANFDLLD_Handle canfdHandle, CANFDLLD_DmaChConfig dmaChCfg)
{
    int32_t status = SystemP_FAILURE;
    CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)dmaChCfg;
    uint32_t            baseAddr, regionId;
    uint32_t            isEdmaInterruptEnabled;
    EDMA_Handle         canfdEdmaHandle;
    CANFDLLD_Object         *object;

    object      = (CANFDLLD_Object *) canfdHandle;
    canfdEdmaHandle = (EDMA_Handle) object->canfdDmaHandle;

    if(canfdEdmaHandle != NULL)
    {
        /* Read base address of allocated EDMA instance */
        baseAddr = EDMA_getBaseAddr(canfdEdmaHandle);

        /* Read the region ID of the EDMA instance */
        regionId = EDMA_getRegionId(canfdEdmaHandle);

        /* Store the EDMA parameters */
        edmaChCfg->edmaBaseAddr = baseAddr;
        edmaChCfg->edmaRegionId = regionId;

        /* Check if interrupt is enabled */
        isEdmaInterruptEnabled = EDMA_isInterruptEnabled(canfdEdmaHandle);

        if((baseAddr != 0) && (regionId < SOC_EDMA_NUM_REGIONS) && (isEdmaInterruptEnabled == TRUE))
        {
            /* Validate the EDMA parameters for MCAN */
            edmaChCfg->isOpen = TRUE;
            edmaChCfg->edmaTxChAlloc = 0U;
            edmaChCfg->edmaRxChAlloc = 0U;
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

int32_t CANFD_lld_createDmaTxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj)
{
    uint32_t i;
    int32_t status = SystemP_SUCCESS, chAllocStatus;
    EDMA_Handle canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
    CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

    /* Check the free Tx dma event to program */
    for (i = 0; i < MCAN_MAX_TX_DMA_BUFFERS; i++)
    {
        if ((edmaChCfg->edmaTxChAlloc & (1<<i)) == 0)
        {
            edmaChCfg->edmaTxChAlloc |= (1<<i);
            ptrCanMsgObj->dmaEventNo = i;
            break;
        }
    }
    if (i == MCAN_MAX_TX_DMA_BUFFERS)
    {
        /* Error: Unable to allocate the memory */
        status = SystemP_OUT_OF_RESOURCES;
    }
    if (status == SystemP_SUCCESS)
    {
        /* Allocate EDMA channel for CANFD TX transfer */
        chAllocStatus = EDMA_allocDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo]));
        status += chAllocStatus;

        /* Allocate EDMA TCC for CANFD TX transfer */
        edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocTcc(canfdEdmaHandle, &(edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo]));

        /* Allocate a Param ID for CANFD TX transfer */
        edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocParam(canfdEdmaHandle, &(edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo]));

        if(status == SystemP_SUCCESS)
        {
            EDMAConfigureChannelRegion(edmaChCfg->edmaBaseAddr, edmaChCfg->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
            edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo], edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo],
            edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo], EDMA_CANFD_TX_EVT_QUEUE_NO);
        }

        if (status != SystemP_SUCCESS)
        {
            /* Free all allocated resources of edma */
            if ((chAllocStatus == SystemP_SUCCESS) && (edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY))
            {
                EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo]));
            }
            if (edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo]));
            }
            if (edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo]));
            }
        }
    }

    return status;
}

int32_t CANFD_lld_deleteDmaTxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj)
{
    int32_t status = SystemP_SUCCESS;
    EDMA_Handle canfdEdmaHandle;
    CANFDLLD_EdmaChConfig *edmaChCfg;

    if ((ptrCanFdObj == NULL) || (ptrCanMsgObj == NULL))
    {
        status = SystemP_INVALID_PARAM;
    }
    if (status == SystemP_SUCCESS)
    {
        canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
        edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        if ((canfdEdmaHandle == NULL) || (edmaChCfg == NULL))
        {
            status = SystemP_INVALID_PARAM;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        /* Free all allocated resources of edma */
        if (edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
        {
            EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo]));
        }
        if (edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
        {
            EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo]));
            edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        }
        if (edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
        {
            EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo]));
            edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        }
    }
    return status;
}

void CANFD_lld_dmaTxCallBack(CANFDLLD_MessageObject* ptrCanMsgObj)
{
    uint8_t *currentDataPtr;

    if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum == (ptrCanMsgObj->dmaMsgConfig.numMsgs))
    {
        currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);
        currentDataPtr += (ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg * ptrCanMsgObj->dmaMsgConfig.currentMsgNum);

        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;

        /* All msgs are transmitted. */
        uint32_t baseAddr, regionId, dmaTxCh;
        CANFDLLD_Object *ptrCanFdObj = ptrCanMsgObj->ptrCanFdObj;
        CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        baseAddr               = edmaChCfg->edmaBaseAddr;
        regionId               = edmaChCfg->edmaRegionId;
        dmaTxCh    = edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo];

        /* Disable event trigger transfer */
        EDMADisableTransferRegion(baseAddr, regionId, dmaTxCh,
                EDMA_TRIG_MODE_EVENT);
        CANFD_dmaTxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_TX_COMPLETION_FINAL);
    }
    else if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum < (ptrCanMsgObj->dmaMsgConfig.numMsgs))
    {
        currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);
        currentDataPtr += (ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg * ptrCanMsgObj->dmaMsgConfig.currentMsgNum);

        ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
        CANFD_dmaTxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_TX_COMPLETION_INTERMEDIATE);
    }
    else
    {
        /* Callback for all msgs are called. This should not be called. */
    }
}

__attribute__((weak)) void CANFD_dmaTxCompletionCallback(CANFDLLD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{

}

int32_t CANFD_lld_configureDmaTx(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj, uint32_t dataLengthPerMsg, uint32_t numMsgs, const void* data)
{
    uint32_t            status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaTxCh, tccTx, paramTx;
    EDMACCPaRAMEntry    edmaTxParam;
    CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
    uint32_t            srcAddr, dstAddr;

    /* Store the current Tx msg. */
    ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg = dataLengthPerMsg;
    ptrCanMsgObj->dmaMsgConfig.numMsgs          = numMsgs;
    ptrCanMsgObj->dmaMsgConfig.data             = data;
    ptrCanMsgObj->dmaMsgConfig.currentMsgNum    = 1;

    /* Fetch the EDMA paramters for CANFD transfer */
    baseAddr   = edmaChCfg->edmaBaseAddr;
    regionId   = edmaChCfg->edmaRegionId;
    dmaTxCh    = edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo];
    tccTx      = edmaChCfg->edmaTxTcc[ptrCanMsgObj->dmaEventNo];
    paramTx    = edmaChCfg->edmaTxParam[ptrCanMsgObj->dmaEventNo];

    /* First msg is already copied to msg ram. Program the dma to transfer from second msg. */
    srcAddr = (uint32_t) data + dataLengthPerMsg;
    /* Get the buffer address in message ram. */
    dstAddr = MCAN_lld_getWriteMsgElemAddress(ptrCanFdObj->regBaseAddress, MCAN_MEM_TYPE_BUF,  ptrCanMsgObj->txElement);
    /* Add base address to the offset. */
    dstAddr += ptrCanFdObj->regBaseAddress;
    /* First 8 bytes are for header which will remain common for all msgs.
       Data should be updated from dstAddr + 8. */
    dstAddr += 8;

    /* Transmit param set configuration */
    EDMACCPaRAMEntry_init(&edmaTxParam);
    edmaTxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void*) srcAddr);
    edmaTxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t *) dstAddr);
    edmaTxParam.aCnt          = (uint16_t) dataLengthPerMsg;
    edmaTxParam.bCnt          = (uint16_t) numMsgs;
    edmaTxParam.cCnt          = (uint16_t) 1;
    edmaTxParam.bCntReload    = (uint16_t) edmaTxParam.bCnt;
    edmaTxParam.srcBIdx       = (int16_t) edmaTxParam.aCnt;
    edmaTxParam.destBIdx      = (int16_t) 0;
    edmaTxParam.srcCIdx       = (int16_t) 0;
    edmaTxParam.destCIdx      = (int16_t) 0;
    edmaTxParam.linkAddr      = 0xFFFFU;
    edmaTxParam.opt           = 0;
    edmaTxParam.opt          |=
        ((tccTx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);

    /* Write Tx param set */
    EDMASetPaRAM(baseAddr, paramTx, &edmaTxParam);

    /* Set event trigger to start CANFD TX transfer */
    EDMAEnableTransferRegion(baseAddr, regionId, dmaTxCh,
         EDMA_TRIG_MODE_EVENT);
    return status;
}

int32_t CANFD_lld_cancelDmaTx(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj)
{
    uint32_t            status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaTxCh;
    CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

    /* Fetch the EDMA paramters for CANFD transfer */
    baseAddr   = edmaChCfg->edmaBaseAddr;
    regionId   = edmaChCfg->edmaRegionId;
    dmaTxCh    = edmaChCfg->edmaTxChId[ptrCanMsgObj->dmaEventNo];

    /* Set event trigger to start CANFD TX transfer */
    EDMADisableTransferRegion(baseAddr, regionId, dmaTxCh,
         EDMA_TRIG_MODE_EVENT);
    return status;
}

static void CANFD_edmaIsrRx(Edma_IntrHandle intrHandle, void *args)
{
    CANFDLLD_MessageObject* ptrCanMsgObj = (CANFDLLD_MessageObject *)(args);
    uint8_t *currentDataPtr;

    ptrCanMsgObj->dmaMsgConfig.currentMsgNum++;
    currentDataPtr = (uint8_t *)(ptrCanMsgObj->dmaMsgConfig.data);
    currentDataPtr += (ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg * ptrCanMsgObj->dmaMsgConfig.currentMsgNum);

    if (ptrCanMsgObj->dmaMsgConfig.currentMsgNum == (ptrCanMsgObj->dmaMsgConfig.numMsgs))
    {
        /* All msgs are transmitted. */
        uint32_t baseAddr, regionId, dmaRxCh;
        CANFDLLD_Object *ptrCanFdObj = ptrCanMsgObj->ptrCanFdObj;
        CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        baseAddr               = edmaChCfg->edmaBaseAddr;
        regionId               = edmaChCfg->edmaRegionId;
        dmaRxCh    = edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo];

        /* Disable event trigger transfer */
        EDMADisableTransferRegion(baseAddr, regionId, dmaRxCh,
                EDMA_TRIG_MODE_EVENT);
        CANFD_dmaRxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_RX_COMPLETION_FINAL);
    }
    else
    {
        CANFD_dmaRxCompletionCallback(ptrCanMsgObj, (void *)(currentDataPtr), CANFD_DMA_RX_COMPLETION_INTERMEDIATE);
    }
}

__attribute__((weak)) void CANFD_dmaRxCompletionCallback(CANFDLLD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{

}

int32_t CANFD_lld_createDmaRxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj)
{
    int32_t status = SystemP_SUCCESS, chAllocStatus;
    EDMA_Handle canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
    CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

    if (status == SystemP_SUCCESS)
    {
        /* First MCAN_MAX_TX_DMA_BUFFERS are reserved for the dma mode and one of them will be allocated.
           edmaRxChId[] array will have the corresponding dma channel as configured by crossbar. */
        ptrCanMsgObj->dmaEventNo = ptrCanMsgObj->txElement;
        /* Allocate EDMA channel for CANFD RX transfer */
        chAllocStatus = EDMA_allocDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo]));
        status += chAllocStatus;

        /* Allocate EDMA TCC for CANFD RX transfer */
        edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocTcc(canfdEdmaHandle, &(edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo]));

        /* Allocate a Param ID for CANFD RX transfer */
        edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        status += EDMA_allocParam(canfdEdmaHandle, &(edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo]));

        if(status == SystemP_SUCCESS)
        {
            EDMAConfigureChannelRegion(edmaChCfg->edmaBaseAddr, edmaChCfg->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
            edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo], edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo],
            edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo], EDMA_CANFD_RX_EVT_QUEUE_NO);

            /* Register RX interrupt */
            edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo].tccNum = edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo];
            edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo].cbFxn  = &CANFD_edmaIsrRx;
            edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo].appData = (void *) ptrCanMsgObj;
            status += EDMA_registerIntr(canfdEdmaHandle, &(edmaChCfg->edmaIntrObjRx[ptrCanMsgObj->dmaEventNo]));
        }

        if (status != SystemP_SUCCESS)
        {
            /* Free all allocated resources of edma */
            if ((chAllocStatus == SystemP_SUCCESS) && (edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY))
            {
                EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo]));
            }
            if (edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo]));
            }
            if (edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
            {
                EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo]));
            }
        }
    }

    return status;
}

int32_t CANFD_lld_deleteDmaRxMsgObject(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj)
{
    int32_t status = SystemP_SUCCESS;
    EDMA_Handle canfdEdmaHandle;
    CANFDLLD_EdmaChConfig *edmaChCfg;

    if ((ptrCanFdObj == NULL) || (ptrCanMsgObj == NULL))
    {
        status = SystemP_INVALID_PARAM;
    }
    if (status == SystemP_SUCCESS)
    {
        canfdEdmaHandle = (EDMA_Handle) ptrCanFdObj->canfdDmaHandle;
        edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;

        if ((canfdEdmaHandle == NULL) || (edmaChCfg == NULL))
        {
            status = SystemP_INVALID_PARAM;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        /* Free all allocated resources of edma */
        if (edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
        {
            EDMA_freeDmaChannel(canfdEdmaHandle, &(edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo]));
        }
        if (edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
        {
            EDMA_freeTcc(canfdEdmaHandle, &(edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo]));
            edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        }
        if (edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] != EDMA_RESOURCE_ALLOC_ANY)
        {
            EDMA_freeParam(canfdEdmaHandle, &(edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo]));
            edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo] = EDMA_RESOURCE_ALLOC_ANY;
        }
    }
    return status;
}

int32_t CANFD_lld_configureDmaRx(CANFDLLD_Object *ptrCanFdObj, CANFDLLD_MessageObject* ptrCanMsgObj, uint32_t dataLengthPerMsg, uint32_t numMsgs, const void* data)
{
    uint32_t status = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    uint32_t            dmaRxCh, tccRx, paramRx;
    EDMACCPaRAMEntry    edmaRxParam;
    CANFDLLD_EdmaChConfig *edmaChCfg = (CANFDLLD_EdmaChConfig *)ptrCanFdObj->canfdDmaChCfg;
    uint32_t            srcAddr, dstAddr;

    /* Store the current Rx msg. */
    ptrCanMsgObj->dmaMsgConfig.dataLengthPerMsg = dataLengthPerMsg;
    ptrCanMsgObj->dmaMsgConfig.numMsgs          = numMsgs;
    ptrCanMsgObj->dmaMsgConfig.data             = data;
    ptrCanMsgObj->dmaMsgConfig.currentMsgNum    = 0;

    /* Fetch the EDMA paramters for CANFD transfer */
    baseAddr               = edmaChCfg->edmaBaseAddr;
    regionId               = edmaChCfg->edmaRegionId;
    dmaRxCh    = edmaChCfg->edmaRxChId[ptrCanMsgObj->dmaEventNo];
    tccRx      = edmaChCfg->edmaRxTcc[ptrCanMsgObj->dmaEventNo];
    paramRx    = edmaChCfg->edmaRxParam[ptrCanMsgObj->dmaEventNo];

    /* Get the buffer address in message ram. */
    srcAddr = MCAN_lld_getReadMsgElemAddress(ptrCanFdObj->regBaseAddress, MCAN_MEM_TYPE_BUF, ptrCanMsgObj->rxElement, 0);
    /* Add base address to the offset. */
    srcAddr += ptrCanFdObj->regBaseAddress;
    /* Program the dma to receive the msg. */
    dstAddr = (uint32_t) data;

    /* Transmit param set configuration */
    EDMACCPaRAMEntry_init(&edmaRxParam);
    edmaRxParam.srcAddr       = (uint32_t) SOC_virtToPhy((void*) srcAddr);
    edmaRxParam.destAddr      = (uint32_t) SOC_virtToPhy((uint8_t *) dstAddr);
    edmaRxParam.aCnt          = (uint16_t) dataLengthPerMsg;
    edmaRxParam.bCnt          = (uint16_t) numMsgs;
    edmaRxParam.cCnt          = (uint16_t) 1;
    edmaRxParam.bCntReload    = (uint16_t) edmaRxParam.bCnt;
    edmaRxParam.srcBIdx       = (int16_t) 0;
    edmaRxParam.destBIdx      = (int16_t) edmaRxParam.aCnt;
    edmaRxParam.srcCIdx       = (int16_t) 0;
    edmaRxParam.destCIdx      = (int16_t) 0;
    edmaRxParam.linkAddr      = 0xFFFFU;
    edmaRxParam.opt           = 0;
    edmaRxParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((tccRx << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    /* Write Rx param set */
    EDMASetPaRAM(baseAddr, paramRx, &edmaRxParam);

    /* Set event trigger to start CANFD Rx transfer */
    EDMAEnableTransferRegion(baseAddr, regionId, dmaRxCh,
         EDMA_TRIG_MODE_EVENT);
    return status;
}

