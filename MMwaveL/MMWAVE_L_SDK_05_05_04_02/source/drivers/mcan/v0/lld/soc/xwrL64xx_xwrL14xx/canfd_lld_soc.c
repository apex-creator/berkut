/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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
 *   @file  canfd_lld_soc.c
 *
 *   @brief
 *      The file implements the Controller Area Network Driver Flexible data.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/mcan/v0/lld/mcan_lld.h>
#include <drivers/mcan/v0/lld/canfd_lld.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CANFD_lld_writeDma(CANFDLLD_MsgObjHandle handle, uint32_t id, CANFDLLD_MCANFrameType frameType, uint32_t dataLengthPerMsg, uint32_t numMsgs, void* data)
{
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    CANFDLLD_Object*           ptrCanFdObj;
    int32_t                    retVal = SystemP_SUCCESS;
    uint32_t                   baseAddr;
    MCAN_TxBufElement          txBuffElem;
    uint32_t                   index;
    uint8_t                    padSize = 0U;

    /* Get the message object pointer */
    ptrCanMsgObj = (CANFDLLD_MessageObject*)handle;
    if ((ptrCanMsgObj == NULL) || (data == NULL) || (dataLengthPerMsg < 1U) || (dataLengthPerMsg > 64U) || (numMsgs < 1U))
    {
        retVal = SystemP_INVALID_PARAM;
    }
    if (retVal == SystemP_SUCCESS)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFDLLD_Object*)ptrCanMsgObj->ptrCanFdObj;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_INVALID_PARAM;
        }
    }
    if (retVal == SystemP_SUCCESS)
    {
        if ((ptrCanFdObj->canfdDmaHandle == NULL ) || (ptrCanMsgObj->enableDmaMode != TRUE))
        {
            retVal = SystemP_INVALID_PARAM;
        }
    }
    if (retVal == SystemP_SUCCESS)
    {
        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Check for pending messages */
        index = (uint32_t)1U << ptrCanMsgObj->txElement;
        if (index == (MCAN_lld_getTxBufReqPend(baseAddr) & index))
        {
            retVal = SystemP_BUSY;
        }
    }
    if (retVal == SystemP_SUCCESS)
    {
        /* populate the Tx buffer message element */
        txBuffElem.rtr = 0;
        txBuffElem.esi = 0;
        txBuffElem.efc = 0;
        txBuffElem.mm = 0;

        if(frameType == CANFDLLD_MCANFrameType_CLASSIC)
        {
            txBuffElem.brs = 0;
            txBuffElem.fdf = 0;
        }
        else
        {
            txBuffElem.brs = 1U;
            txBuffElem.fdf = 1U;
        }
        /* Populate the Id */
        if (ptrCanMsgObj->msgIdType == CANFDLLD_MCANXidType_11_BIT)
        {
            txBuffElem.xtd = CANFDLLD_MCANXidType_11_BIT;
            txBuffElem.id = (id & STD_MSGID_MASK) << STD_MSGID_SHIFT;
        }
        else
        {
            txBuffElem.xtd = CANFDLLD_MCANXidType_29_BIT;
            txBuffElem.id = id & XTD_MSGID_MASK;
        }

        /* Copy the data of first message */
        memcpy ((void*)&txBuffElem.data, data, dataLengthPerMsg);

        /* Compute the DLC value */
        for(index = 0U ; index < 16U ; index++)
        {
            if(dataLengthPerMsg <= ptrCanFdObj->mcanDataSize[index])
            {
                txBuffElem.dlc = index;
                padSize = ptrCanFdObj->mcanDataSize[index] - dataLengthPerMsg;
                break;
            }
        }
        txBuffElem.dlc = index;
        if (index == 16)
        {
            retVal = SystemP_INVALID_PARAM;
        }
        else
        {
            /* Pad the unused data in payload */
            index = dataLengthPerMsg;
            while (padSize != 0)
            {
                txBuffElem.data[index++] = (uint8_t)0xCCU;
                padSize--;
            }
            /* Copy the first msg in msg ram. Subsequent msgs are written by the dma. */
            MCAN_lld_writeMsgRam(baseAddr, MCAN_MEM_TYPE_BUF, ptrCanMsgObj->txElement, &txBuffElem);

            /* Configure the dma to copy the subsequent msgs */
            CANFD_lld_configureDmaTx(ptrCanFdObj, ptrCanMsgObj, dataLengthPerMsg, numMsgs, data);

            MCAN_lld_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);

            ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = 1;

            /* Increment the stats */
            ptrCanMsgObj->messageProcessed++;
        }
    }
    return retVal;
}

int32_t CANFD_lld_writeDmaTriggerNext(CANFDLLD_MsgObjHandle handle)
{
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    CANFDLLD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    uint32_t                baseAddr;
    uint32_t                index;

    /* Get the message object pointer */
    ptrCanMsgObj = (CANFDLLD_MessageObject*)handle;
    if (ptrCanMsgObj == NULL)
    {
        retVal = SystemP_INVALID_PARAM;
    }
    if (retVal == SystemP_SUCCESS)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFDLLD_Object*)ptrCanMsgObj->ptrCanFdObj;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_INVALID_PARAM;
        }
    }
    if (retVal == SystemP_SUCCESS)
    {
        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Check for pending messages */
        index = (uint32_t)1U << ptrCanMsgObj->txElement;
        if (index == (MCAN_lld_getTxBufReqPend(baseAddr) & index))
        {
            retVal = SystemP_BUSY;
        }
    }
    if (retVal == SystemP_SUCCESS)
    {
        MCAN_lld_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);

        ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = 1;
    }
    return retVal;
}

uint32_t CANFD_lld_getFilterEventConfig(uint32_t eventNum)
{
    return ((eventNum + 1) & 0x7) << 6;;
}
