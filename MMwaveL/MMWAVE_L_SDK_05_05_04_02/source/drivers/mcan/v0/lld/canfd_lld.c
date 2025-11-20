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
 *   @file  canfd_lld.c
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
#include <kernel/dpl/SystemP.h>
#include <drivers/mcan/v0/lld/canfd_lld.h>

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void CANFD_processFIFOElements(CANFDLLD_Object* ptrCanFdObj, MCAN_RxFIFONum fifoNum);
static int32_t CANFD_configMessageRAM(uint32_t baseAddr, const CANFDLLD_MCANMsgRAMCfgParams* configParams);

void CANFD_lld_dmaTxCallBack(CANFDLLD_MessageObject* ptrCanMsgObj);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  @b Description
 *  @n
 *      Function processes the Rx FIFO and calls application callback function to receive data.
 *
 *  @param[in]  ptrCanFdMCB
 *      Pointer to the CANFD driver MCB
 *  @param[in]  fifoNum
 *      Rx FiFO number
 *
 *  @retval
 *      Success -   Handle to the CAN Driver
 *  @retval
 *      Error   -   NULL
 */

static void CANFD_processFIFOElements(CANFDLLD_Object* ptrCanFdObj, MCAN_RxFIFONum fifoNum)
{
    MCAN_RxFIFOStatus       fifoStatus;
    uint32_t                fillLevel;
    uint32_t                index;
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    uint32_t                baseAddr;

    baseAddr = ptrCanFdObj->regBaseAddress;

    /* Get the FIFO status */
    fifoStatus.num = (uint32_t)fifoNum;
    MCAN_lld_getRxFIFOStatus(baseAddr, &fifoStatus);
    fillLevel = fifoStatus.fillLvl;

    for(index = 0; index < fillLevel; index++)
    {
        MCAN_lld_readMsgRam(baseAddr, MCAN_MEM_TYPE_FIFO, fifoStatus.getIdx, (uint32_t)fifoNum, &ptrCanFdObj->rxBuffElem);

        /* Get the message object pointer */
        ptrCanMsgObj = ptrCanFdObj->rxMapping[ptrCanFdObj->rxBuffElem.fidx];

        /* Increment the number of interrupts received */
        ptrCanMsgObj->interruptsRxed++;

        /* Call the registered callback. */
        CANFD_lld_dataAppCallBack((CANFDLLD_MsgObjHandle)ptrCanMsgObj, CANFDLLD_Reason_RX);

        /* Acknowledge the data read */
        MCAN_lld_writeRxFIFOAck(baseAddr, (uint32_t)fifoNum, fifoStatus.getIdx);

        MCAN_lld_getRxFIFOStatus(baseAddr, &fifoStatus);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is the registered interrupt 0 ISR for the CANFD Driver.
 *
 *  @param[in]  arg
 *      Argument which is registered with the OS while registering the ISR
 *
 *  \ingroup CANFD_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CANFD_lld_int0Isr (CANFDLLD_Object* ptrCanFdObj)
{
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    uint32_t                baseAddr;
    uint32_t                intrStatus = 0;
    uint32_t                index, status, buffIndex;
    MCAN_RxNewDataStatus    newDataStatus;

    /* Increment the number of interrupts received */
    ptrCanFdObj->interrupts++;
    baseAddr = ptrCanFdObj->regBaseAddress;

    if(baseAddr != 0)
    {
        intrStatus = MCAN_lld_getIntrStatus(baseAddr);
        MCAN_lld_clearIntrStatus(baseAddr, intrStatus);
    }

    /* Process Bus-Off condition */
    if ((intrStatus & MCAN_INTR_SRC_BUS_OFF_STATUS) == MCAN_INTR_SRC_BUS_OFF_STATUS)
    {
        /* Increment the number of interrupts received */
        ptrCanFdObj->busOffInterrupts++;

        ptrCanFdObj->state = CANFDLLD_DriverState_STOPPED;

        /* Call the registered callback. */
        CANFD_lld_errStatusAppCallBack((CANFDLLD_Handle)ptrCanFdObj, CANFDLLD_Reason_BUSOFF, NULL);
    }

    /* Process Protocol error in data phase condition */
    if ((intrStatus & MCAN_INTR_SRC_PROTOCOL_ERR_DATA) == MCAN_INTR_SRC_PROTOCOL_ERR_DATA)
    {
        /* Increment the number of interrupts received */
        ptrCanFdObj->protoDataErrInterrupts++;

        /* Call the registered callback. */
        CANFD_lld_errStatusAppCallBack((CANFDLLD_Handle)ptrCanFdObj, CANFDLLD_Reason_PROTOCOL_ERR_DATA_PHASE, NULL);
    }

    /* Process Protocol error in arbitration phase condition */
    if ((intrStatus & MCAN_INTR_SRC_PROTOCOL_ERR_ARB) == MCAN_INTR_SRC_PROTOCOL_ERR_ARB)
    {
        /* Increment the number of interrupts received */
        ptrCanFdObj->protoArbErrInterrupts++;

        /* Call the registered callback. */
        CANFD_lld_errStatusAppCallBack((CANFDLLD_Handle)ptrCanFdObj, CANFDLLD_Reason_PROTOCOL_ERR_ARB_PHASE, NULL);
    }

    /* Process Transmit complete interrrupt */
    if ((intrStatus & MCAN_INTR_SRC_TRANS_COMPLETE) == MCAN_INTR_SRC_TRANS_COMPLETE)
    {
        status = MCAN_lld_getTxBufTransmissionStatus(baseAddr);

        /* Process all 32 Tx buffers */
        for(index = 0; index < MCAN_MAX_TX_BUFFERS; index++)
        {
            buffIndex = ((uint32_t)1U << index);
            if(buffIndex == (status & buffIndex))
            {
                /* Get the message object pointer */
                ptrCanMsgObj = ptrCanFdObj->txMapping[index];

                if (ptrCanMsgObj != NULL)
                {
                    /* Increment the number of interrupts received */
                    ptrCanMsgObj->interruptsRxed++;

                    if(ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] == 1)
                    {
                        ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = 0;
                        if (ptrCanMsgObj->enableDmaMode == TRUE)
                        {
                            /* Call dma callback */
                            CANFD_lld_dmaTxCallBack((CANFDLLD_MsgObjHandle)ptrCanMsgObj);
                        }
                        else
                        {
                            /* Call the registered callback. */
                            CANFD_lld_dataAppCallBack((CANFDLLD_MsgObjHandle)ptrCanMsgObj, CANFDLLD_Reason_TX_COMPLETION);
                        }
                    }
                }
            }
            status = (status & ~buffIndex);
            if (status == 0)
            {
                break;
            }
        }
    }

    /* Process Receive buffer interrupt */
    if ((intrStatus & MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG) == MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG)
    {
        /* Get the new data status */
        MCAN_lld_getNewDataStatus(baseAddr, &newDataStatus);

        /* Clear NewData status to accept new messages */
        MCAN_lld_clearNewDataStatus(baseAddr, &newDataStatus);

        /* Process the low 32 buffers */
        status = newDataStatus.statusLow;
        index = 0;
        while (status != 0)
        {
            if ((status & 1U) == 1U)
            {
                /* Get the message object pointer */
                ptrCanMsgObj = ptrCanFdObj->rxMapping[index];

                /* Increment the number of interrupts received */
                ptrCanMsgObj->interruptsRxed++;

                /* In non dma mode read the message and give callback.
                   In dma mode the msgram is read using dma. Only clear the new data status here. */
                if (ptrCanMsgObj->enableDmaMode != TRUE)
                {
                    /* Read the pending data */
                    MCAN_lld_readMsgRam(baseAddr, ptrCanMsgObj->memType, ptrCanMsgObj->rxElement, 0, &ptrCanFdObj->rxBuffElem);

                    /* Call the registered callback. */
                    CANFD_lld_dataAppCallBack((CANFDLLD_MsgObjHandle)ptrCanMsgObj, CANFDLLD_Reason_RX);
                }
            }
            index++;
            status = (status >> 1U);
        }
    }

    /* Process Receive FIFO interrupts */
    if ((intrStatus & MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) == MCAN_INTR_SRC_RX_FIFO0_NEW_MSG)
    {
        /* Process FIFO 0 */
        CANFD_processFIFOElements(ptrCanFdObj, MCAN_RX_FIFO_NUM_0);
    }

    if ((intrStatus & MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) == MCAN_INTR_SRC_RX_FIFO1_NEW_MSG)
    {
        /* Process FIFO 1 */
        CANFD_processFIFOElements(ptrCanFdObj, MCAN_RX_FIFO_NUM_1);
    }

    /* Process Transmit cancel interrrupt */
    if ((intrStatus & MCAN_INTR_SRC_TRANS_CANCEL_FINISH) == MCAN_INTR_SRC_TRANS_CANCEL_FINISH)
    {
        status = MCAN_lld_txBufCancellationStatus(baseAddr);

        /* Process all 32 Tx buffers */
        for(index = 0; index < MCAN_MAX_TX_BUFFERS; index++)
        {
            buffIndex = ((uint32_t)1U << index);
            if(buffIndex == (status & buffIndex))
            {
                /* Get the message object pointer */
                ptrCanMsgObj = ptrCanFdObj->txMapping[index];

                /* Increment the number of interrupts received */
                ptrCanMsgObj->interruptsRxed++;

                if(ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] == 1)
                {
                    ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = 0;

                    /* Call the registered callback. */
                    CANFD_lld_dataAppCallBack((CANFDLLD_MsgObjHandle)ptrCanMsgObj, CANFDLLD_Reason_TX_CANCELED);
                }
            }
            status = (status & ~buffIndex);
            if (status == 0)
            {
                break;
            }
        }
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered interrupt 1 ISR for the CANFD Driver.
 *
 *  @param[in]  arg
 *      Argument which is registered with the OS while registering the ISR
 *
 *  \ingroup CANFD_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CANFD_lld_int1Isr (CANFDLLD_Object* ptrCanFdObj)
{
    uint32_t            baseAddr;
    MCAN_ECCErrStatus   errStatusResp;
    CANFDLLD_ErrStatusResp errStatusInfo;

    /* Get the pointer to the CAN Driver Block */
    baseAddr = ptrCanFdObj->regBaseAddress;

    /* Increment the number of interrupts received */
    ptrCanFdObj->eccInterrupts++;

    /* Read ECC error status */
    MCAN_lld_eccGetErrorStatus(baseAddr, &errStatusResp);
    if (errStatusResp.secErr == 1U)
    {
        MCAN_lld_eccClearErrorStatus(baseAddr, CANFDLLD_MCANECCErrType_SEC);
    }
    if (errStatusResp.dedErr == 1U)
    {
        MCAN_lld_eccClearErrorStatus(baseAddr, CANFDLLD_MCANECCErrType_DED);
    }

    /* Call the registered callback. */
    errStatusInfo.u.eccErrStatus.bit1 = errStatusResp.bit1;
    errStatusInfo.u.eccErrStatus.bit2 = errStatusResp.bit2;
    errStatusInfo.u.eccErrStatus.dedErr = errStatusResp.dedErr;
    errStatusInfo.u.eccErrStatus.row    = errStatusResp.row;
    errStatusInfo.u.eccErrStatus.secErr = errStatusResp.secErr;
    CANFD_lld_errStatusAppCallBack((CANFDLLD_Handle)ptrCanFdObj, CANFDLLD_Reason_ECC_ERROR, &errStatusInfo);

    MCAN_lld_eccWriteEOI(baseAddr, CANFDLLD_MCANECCErrType_SEC);
    MCAN_lld_eccWriteEOI(baseAddr, CANFDLLD_MCANECCErrType_DED);

    return;
}

/**
 *  @b Description
 *  @n
 *      Function initializes the CANFD driver instance with the specified hardware attributes.
 *      It resets and configures the MCAN module, sets up the Message RAM and ECC Aggregator.
 *      It configures the CANFD driver with the control parameters.
 *
 *  @param[in]  ptrCanFdMCB
 *      Pointer to the CANFD driver MCB
 *  @param[in]  configParams
 *      CAN module configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Handle to the CAN Driver
 *  @retval
 *      Error   -   NULL
 */

static int32_t CANFD_configMessageRAM(uint32_t baseAddr, const CANFDLLD_MCANMsgRAMCfgParams* configParams)
{
    MCAN_MsgRAMConfigParams msgRAMConfigParams;
    uint32_t                startAddr;

    /* Compute the start Address and populate the Message RAM configuration parameters */
    startAddr = 0;

    /* Tx buffer configuration */
    msgRAMConfigParams.txStartAddr = startAddr;
    msgRAMConfigParams.txBufCnt = configParams->txBufNum;
    msgRAMConfigParams.txFIFOCnt = configParams->txFIFOSize;
    msgRAMConfigParams.txBufMode = 1U;
    msgRAMConfigParams.txBufElemSize = CANFDLLD_MCANElemSize_64BYTES;

    /* Tx Event FIFO configuration */
    msgRAMConfigParams.txEventFIFOStartAddr = 0;
    msgRAMConfigParams.txEventFIFOCnt = 0;
    msgRAMConfigParams.txEventFIFOWaterMark = 0;

    /* Rx Buffer configuration */
    startAddr += (configParams->txBufNum * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    startAddr += (configParams->txFIFOSize * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    msgRAMConfigParams.rxBufStartAddr = startAddr;
    msgRAMConfigParams.rxBufElemSize = CANFDLLD_MCANElemSize_64BYTES;

    /* 11-bit filter configuration */
    startAddr += (64 * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    msgRAMConfigParams.flssa = startAddr;
    msgRAMConfigParams.lss = configParams->lss;

    /* 29-bit filter configuration */
    startAddr += ((configParams->lss + 1U) * MCAN_MSG_RAM_STD_ELEM_SIZE * 4U);
    msgRAMConfigParams.flesa = startAddr;
    msgRAMConfigParams.lse = configParams->lse;

    /* Rx FIFO 0 configuration */
    startAddr += ((configParams->lse + 1U) * MCAN_MSG_RAM_EXT_ELEM_SIZE * 4U);
    msgRAMConfigParams.rxFIFO0StartAddr = startAddr;
    msgRAMConfigParams.rxFIFO0Cnt = configParams->rxFIFO0size;
    msgRAMConfigParams.rxFIFO0WaterMark = 0;
    msgRAMConfigParams.rxFIFO0ElemSize = CANFDLLD_MCANElemSize_64BYTES;
    msgRAMConfigParams.rxFIFO0OpMode = configParams->rxFIFO0OpMode;

    /* Rx FIFO 1 configuration */
    startAddr += ((configParams->rxFIFO0size + 1U) * MCAN_MSG_RAM_TX_RX_ELEM_SIZE * 4U);
    msgRAMConfigParams.rxFIFO1StartAddr = startAddr;
    msgRAMConfigParams.rxFIFO1Cnt = configParams->rxFIFO1size;
    msgRAMConfigParams.rxFIFO1WaterMark = 0;
    msgRAMConfigParams.rxFIFO1ElemSize = CANFDLLD_MCANElemSize_64BYTES;
    msgRAMConfigParams.rxFIFO1OpMode = configParams->rxFIFO1OpMode;

    /* Configure Message RAM */
    return (MCAN_lld_msgRAMConfig(baseAddr, &msgRAMConfigParams));
}

static void CANFD_lld_initCslMcanInitParams(const CANFDLLD_MCANInitParams* configParams, MCAN_InitParams *cslMcanInitPrms)
{
    cslMcanInitPrms->autoWkupEnable  = configParams->autoWkupEnable;
    cslMcanInitPrms->brsEnable       = configParams->brsEnable;
    cslMcanInitPrms->clkStopFAck     = configParams->clkStopFAck;
    cslMcanInitPrms->darEnable       = configParams->darEnable;
    cslMcanInitPrms->efbi            = configParams->efbi;
    cslMcanInitPrms->emulationEnable = configParams->emulationEnable;
    cslMcanInitPrms->emulationFAck   = configParams->emulationFAck;
    cslMcanInitPrms->fdMode          = configParams->fdMode;
    cslMcanInitPrms->pxhddisable     = configParams->pxhddisable;
    cslMcanInitPrms->tdcEnable       = configParams->tdcEnable;
    cslMcanInitPrms->txpEnable       = configParams->txpEnable;
    cslMcanInitPrms->wdcPreload      = configParams->wdcPreload;
    cslMcanInitPrms->wkupReqEnable   = configParams->wkupReqEnable;
    cslMcanInitPrms->tdcConfig.tdcf  = configParams->tdcConfig.tdcf;
    cslMcanInitPrms->tdcConfig.tdco  = configParams->tdcConfig.tdco;
}

static void CANFD_lld_initCslMcanConfigParams(const CANFDLLD_MCANInitParams* configParams, MCAN_ConfigParams *cslMcanConfigPrms)
{
    cslMcanConfigPrms->asmEnable         = configParams->asmEnable;
    cslMcanConfigPrms->monEnable         = configParams->monEnable;
    cslMcanConfigPrms->timeoutCntEnable  = configParams->timeoutCntEnable;
    cslMcanConfigPrms->timeoutPreload    = configParams->timeoutPreload;
    cslMcanConfigPrms->timeoutSelect     = configParams->timeoutSelect;
    cslMcanConfigPrms->tsPrescalar       = configParams->tsPrescalar;
    cslMcanConfigPrms->tsSelect          = configParams->tsSelect;
    cslMcanConfigPrms->filterConfig.anfe = configParams->filterConfig.anfe;
    cslMcanConfigPrms->filterConfig.anfs = configParams->filterConfig.anfs;
    cslMcanConfigPrms->filterConfig.rrfe = configParams->filterConfig.rrfe;
    cslMcanConfigPrms->filterConfig.rrfs = configParams->filterConfig.rrfs;
}

static void CANFD_lld_initCslMcanECCConfigParams(const CANFDLLD_MCANInitParams* configParams, MCAN_ECCConfigParams *cslMcanEccConfigPrms)
{
    cslMcanEccConfigPrms->enable = configParams->eccConfig.enable;
    cslMcanEccConfigPrms->enableChk = configParams->eccConfig.enableChk;
    cslMcanEccConfigPrms->enableRdModWr = configParams->eccConfig.enableRdModWr;
}

/**
 *  @b Description
 *  @n
 *      Function initializes the CANFD driver instance with the specified hardware attributes.
 *      It resets and configures the MCAN module, sets up the Message RAM and ECC Aggregator.
 *      It configures the CANFD driver with the control parameters.
 *
 *  @param[in]  instanceId
 *      CANFD Instance Id.
 *      The valid values are platform specific.
 *  @param[in]  configParams
 *      CANFD module configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Handle to the CANFD Driver
 *  @retval
 *      Error   -   NULL
 */

int32_t CANFD_lld_init(CANFDLLD_Handle hCanfd)
{
    uint32_t                baseAddr;
    MCAN_RxNewDataStatus    newDataStatus;
    volatile uint32_t       status = SystemP_SUCCESS;
    MCAN_InitParams         cslMcanInitPrms;
    MCAN_ConfigParams       cslMcanConfigPrms;
    MCAN_ECCConfigParams    cslMcanEccConfigPrms;

    CANFDLLD_Object         *ptrCanFdObj = NULL;

    if(hCanfd == NULL)
    {
        status = SystemP_INVALID_PARAM;
    }
    else
    {
        ptrCanFdObj = (CANFDLLD_Object *) hCanfd;
        if(ptrCanFdObj->state != CANFDLLD_DriverState_UNINIT)
        {
            status = SystemP_FAILURE;
        }

        if (status == SystemP_SUCCESS)
        {
            LLD_PARAMS_CHECK(ptrCanFdObj->regBaseAddress != 0);
        }
        if (status == SystemP_SUCCESS)
        {
            baseAddr = ptrCanFdObj->regBaseAddress;

            /* Check if Message RAM initialization is done or not TODO add timeout */
            while (MCAN_lld_isMemInitDone(baseAddr) != 1)
            {
            }

            /* Reset the MCAN module. */
            MCAN_lld_reset(baseAddr);

            /* Wait while MCAN is in reset. TODO add timeout */
            while (MCAN_lld_isInReset(baseAddr) == 1U)
            {
            }

            /* Put MCAN in SW initialization mode */
            MCAN_lld_setOpMode(baseAddr, MCAN_OPERATION_MODE_SW_INIT);

            /* Wait while MCAN is not in init mode. TODO add timeout */
            while (MCAN_lld_getOpMode(baseAddr) != MCAN_OPERATION_MODE_SW_INIT)
            {
            }

            /* Initialize the MCAN module */
            CANFD_lld_initCslMcanInitParams(&ptrCanFdObj->initParams, &cslMcanInitPrms);
            status  = MCAN_lld_init (baseAddr, &cslMcanInitPrms);

            /* Configure the MCAN module */
            if (status == SystemP_SUCCESS)
            {
                CANFD_lld_initCslMcanConfigParams(&ptrCanFdObj->initParams, &cslMcanConfigPrms);
                status  = MCAN_lld_config (baseAddr, &cslMcanConfigPrms);
            }

            /* Configure the Message RAM */
            if (status == SystemP_SUCCESS)
            {
                status  = CANFD_configMessageRAM(baseAddr, &ptrCanFdObj->initParams.msgRAMConfig);
            }

            /* Initialize dma mode if the dma handle is not NULL */
            if (status == SystemP_SUCCESS)
            {
                if (ptrCanFdObj->canfdDmaHandle != NULL)
                {
                    status  = CANFD_lld_dmaOpen(ptrCanFdObj, ptrCanFdObj->canfdDmaChCfg);
                }
            }

            if (status == SystemP_SUCCESS)
            {
                /* Configure the ECC Aggregator */
                CANFD_lld_initCslMcanECCConfigParams(&ptrCanFdObj->initParams, &cslMcanEccConfigPrms);
                MCAN_lld_eccConfig(baseAddr, &cslMcanEccConfigPrms);

                /* Clear all pending error flags and status */
                MCAN_lld_clearIntrStatus(baseAddr, MCAN_INTR_MASK);
                newDataStatus.statusLow  = CSL_MCAN_NDAT1_CLEAR;
                newDataStatus.statusHigh = CSL_MCAN_NDAT2_CLEAR;
                MCAN_lld_clearNewDataStatus(baseAddr, &newDataStatus);

                if (ptrCanFdObj->initParams.eccConfig.enable == 1U)
                {
                    /* Enable ECC Interrupt */
                    MCAN_lld_eccEnableIntr(baseAddr, CANFDLLD_MCANECCErrType_SEC, 1U);
                    MCAN_lld_eccEnableIntr(baseAddr, CANFDLLD_MCANECCErrType_DED, 1U);
                }

                /* Enable the MCAN interrupts */
                MCAN_lld_enableIntr(baseAddr, MCAN_INTR_MASK, 1U);
                MCAN_lld_selectIntrLine(baseAddr, MCAN_INTR_MASK, MCAN_INTR_LINE_NUM_0);
                MCAN_lld_enableIntrLine(baseAddr, MCAN_INTR_LINE_NUM_0, 1U);

                /* Put MCAN in opertional mode */
                MCAN_lld_setOpMode(baseAddr, MCAN_OPERATION_MODE_NORMAL);

                /* Wait while MCAN is not in operational mode. TODO add timeout */
                while (MCAN_lld_getOpMode(baseAddr) != MCAN_OPERATION_MODE_NORMAL)
                {
                }

                /* Initialize the datalength to DLC mapping */
                ptrCanFdObj->mcanDataSize[0] = 0;
                ptrCanFdObj->mcanDataSize[1] = 1U;
                ptrCanFdObj->mcanDataSize[2] = 2U;
                ptrCanFdObj->mcanDataSize[3] = 3U;
                ptrCanFdObj->mcanDataSize[4] = 4U;
                ptrCanFdObj->mcanDataSize[5] = 5U;
                ptrCanFdObj->mcanDataSize[6] = 6U;
                ptrCanFdObj->mcanDataSize[7] = 7U;
                ptrCanFdObj->mcanDataSize[8] = 8U;
                ptrCanFdObj->mcanDataSize[9] = 12U;
                ptrCanFdObj->mcanDataSize[10] = 16U;
                ptrCanFdObj->mcanDataSize[11] = 20U;
                ptrCanFdObj->mcanDataSize[12] = 24U;
                ptrCanFdObj->mcanDataSize[13] = 32U;
                ptrCanFdObj->mcanDataSize[14] = 48U;
                ptrCanFdObj->mcanDataSize[15] = 64U;

                /* Initialize the CAN driver state */
                ptrCanFdObj->state = CANFDLLD_DriverState_STARTED;
            }
        }
    }
    return status;
}

/**
 *  @b Description
 *  @n
 *      Function closes the CANFD driver instance and cleanups all the memory allocated by the CANFD driver.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_deInit(CANFDLLD_Handle handle)
{
    uint32_t            index;
    int32_t             retVal = SystemP_INVALID_PARAM;
    CANFDLLD_Object         *ptrCanFdObj = NULL;

    if(handle != NULL)
    {
        ptrCanFdObj = (CANFDLLD_Object *) handle;

        /* Update the driver state */
        ptrCanFdObj->state = CANFDLLD_DriverState_STOPPED;

        /* Delete the message objects */
        for (index = 0; index < MCAN_MAX_MSG_OBJECTS; index++)
        {
            if (ptrCanFdObj->msgObjectHandle[index] != NULL)
            {
                ptrCanFdObj->msgObjectHandle[index] = NULL;
            }
        }
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function configures the bit time parameters for the CANFD module.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in]  bitTimeParams
 *      Bit time configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_configBitTime(CANFDLLD_Handle handle, const CANFDLLD_MCANBitTimingParams* bitTimeParams)
{
    MCAN_BitTimingParams    mcanBitTimingParams;
    uint32_t                baseAddr;
    int32_t                 retVal = SystemP_SUCCESS;

    CANFDLLD_Object         *ptrCanFdObj = NULL;

    if((handle == NULL) || (bitTimeParams == NULL))
    {
        retVal = SystemP_INVALID_PARAM;
    }
    if(retVal == SystemP_SUCCESS)
    {
        ptrCanFdObj = (CANFDLLD_Object *) handle;

        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Put MCAN in SW initialization mode */
        MCAN_lld_setOpMode(baseAddr, MCAN_OPERATION_MODE_SW_INIT);

        /* Wait while MCAN is not in init mode. TODO add timeout */
        do
        {
        } while (MCAN_lld_getOpMode(baseAddr) != MCAN_OPERATION_MODE_SW_INIT);

        /* Calculate the MCAN bit timing parameters */
        mcanBitTimingParams.nomRatePrescalar   = bitTimeParams->nomBrp - 1U;
        mcanBitTimingParams.nomTimeSeg1        = bitTimeParams->nomPropSeg + bitTimeParams->nomPseg1 - 1U;
        mcanBitTimingParams.nomTimeSeg2        = bitTimeParams->nomPseg2 - 1U;
        mcanBitTimingParams.nomSynchJumpWidth  = bitTimeParams->nomSjw - 1U;

        mcanBitTimingParams.dataRatePrescalar  = bitTimeParams->dataBrp - 1U;
        mcanBitTimingParams.dataTimeSeg1       = bitTimeParams->dataPropSeg + bitTimeParams->dataPseg1 - 1U;
        mcanBitTimingParams.dataTimeSeg2       = bitTimeParams->dataPseg2 - 1U;
        mcanBitTimingParams.dataSynchJumpWidth = bitTimeParams->dataSjw - 1U;

        /* Set the bit timing values */
        retVal = MCAN_lld_setBitTime(baseAddr, &mcanBitTimingParams);

        /* Put MCAN in opertional mode */
        MCAN_lld_setOpMode(baseAddr, MCAN_OPERATION_MODE_NORMAL);

        /* Wait while MCAN is not in operational mode. TODO add timeout */
        do
        {
        } while (MCAN_lld_getOpMode(baseAddr) != MCAN_OPERATION_MODE_NORMAL);
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function configures the receive or transmit message object.
 *      It also enables Tx completion and Tx cancelation interrupts.
 *      The callback function will be invoked on data transmit complete for transmit message objects
 *      OR
 *      upon receiving data for receive message objects. The application MUST then call CANFD_lld_read() API to process the received data.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in]  msgObjectParams
 *      Message Object configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Handle to the message object.
 *  @retval
 *      Error   -   NULL
 */
int32_t CANFD_lld_createMsgObject(CANFDLLD_Handle handle, CANFDLLD_MessageObject* ptrCanMsgObj)
{
    uint32_t                    baseAddr;
    MCAN_StdMsgIDFilterElement  stdMsgIdFilter;
    MCAN_ExtMsgIDFilterElement  extMsgIdFilter;
    uint32_t                    i;
    int32_t                     retVal = SystemP_INVALID_PARAM;

    /* Get the pointer to the CAN Driver Block */
    CANFDLLD_Object         *ptrCanFdObj = NULL;

    if ((handle != NULL) && (ptrCanMsgObj != NULL))
    {
        retVal = SystemP_SUCCESS;

        ptrCanFdObj = (CANFDLLD_Object *) handle;

        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Save the specified parameters */
        ptrCanMsgObj->ptrCanFdObj = ptrCanFdObj;

        for (i = 0; i < MCAN_MAX_MSG_OBJECTS; i++)
        {
            if (ptrCanFdObj->msgObjectHandle[i] == NULL)
            {
                break;
            }
        }
        if (i == MCAN_MAX_MSG_OBJECTS)
        {
            /* Error: Unable to allocate the memory */
            retVal = SystemP_OUT_OF_RESOURCES;
        }
        else
        {
            /* Store the message object handle for book keeping */
            ptrCanMsgObj->messageObjNum = i;
            ptrCanFdObj->msgObjectHandle[i] = ptrCanMsgObj;
        }

        /* Configure the Tx Message Id */
        if ((retVal == SystemP_SUCCESS) && (ptrCanMsgObj->direction == CANFDLLD_Direction_TX))
        {
            uint32_t txElemStart, txElemEnd;
            if (ptrCanFdObj->canfdDmaHandle != NULL)
            {
                /* dma mode enabled in can driver,
                 tx elements 0 to MCAN_MAX_TX_DMA_BUFFERS are reserved for dma mode. */
                if (ptrCanMsgObj->enableDmaMode == TRUE)
                {
                    /* msg obj created for dma mode, allocate from 0 to MCAN_MAX_TX_DMA_BUFFERS */
                    txElemStart = 0;
                    txElemEnd = MCAN_MAX_TX_DMA_BUFFERS;
                }
                else
                {
                    /* msg obj created for non dma mode, allocate after MCAN_MAX_TX_DMA_BUFFERS */
                    txElemStart = MCAN_MAX_TX_DMA_BUFFERS;
                    txElemEnd = MCAN_MAX_TX_MSG_OBJECTS;
                }
            }
            else
            {
                /* dma mode not enabled in can driver,
                 any available tx buffer elements can be allocated. */
                txElemStart = 0;
                txElemEnd = MCAN_MAX_TX_MSG_OBJECTS;
            }

            /* Get a free Tx element */
            for (i = txElemStart; i < txElemEnd; i++)
            {
                if (ptrCanFdObj->txMapping[i] == NULL)
                {
                    break;
                }
            }
            if (i == MCAN_MAX_TX_MSG_OBJECTS)
            {
                /* Error: Unable to allocate the memory */
                retVal = SystemP_OUT_OF_RESOURCES;
                ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;
            }
            else
            {
                /* Store the tx to message object handle mapping */
                ptrCanFdObj->txMapping[i] = ptrCanMsgObj;
                ptrCanMsgObj->txElement = i;

                /* Store the message RAM memory access type */
                ptrCanMsgObj->memType = MCAN_MEM_TYPE_BUF;

                if (ptrCanMsgObj->enableDmaMode == TRUE)
                {
                    CANFD_lld_createDmaTxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }

                /* Enable Tx completion interrupt*/
                MCAN_lld_txBufTransIntrEnable(baseAddr, i, 1U);

                /* Enable Tx cancelation interrupt*/
                MCAN_lld_txBufCancellationIntrEnable(baseAddr, i, 1U);
            }
        }

        /* Configure the Rx Message Id */
        if ((retVal == SystemP_SUCCESS) && (ptrCanMsgObj->direction == CANFDLLD_Direction_RX))
        {
            /* Get a free Rx element */
            for (i = 0; i < MCAN_MAX_RX_MSG_OBJECTS; i++)
            {
                if (ptrCanFdObj->rxMapping[i] == NULL)
                {
                    break;
                }
            }
            if (i == MCAN_MAX_RX_MSG_OBJECTS)
            {
                /* Error: Unable to allocate the memory */
                retVal = SystemP_OUT_OF_RESOURCES;
                ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;
            }
            else
            {
                /* Store the rx to message object handle mapping */
                ptrCanFdObj->rxMapping[i] = ptrCanMsgObj;
                ptrCanMsgObj->rxElement = i;

                /* Store the message RAM memory access type */
                ptrCanMsgObj->memType = MCAN_MEM_TYPE_BUF;

                if (ptrCanMsgObj->enableDmaMode == TRUE)
                {
                    CANFD_lld_createDmaRxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }

                /* Add the filter to message RAM */
                if (ptrCanMsgObj->msgIdType == CANFDLLD_MCANXidType_11_BIT)
                {
                    stdMsgIdFilter.sfid1 = ptrCanMsgObj->startMsgId & STD_MSGID_MASK;
                    stdMsgIdFilter.sfid2 = i;
                    if (ptrCanMsgObj->enableDmaMode == TRUE)
                    {
                        /* Configure the filter element number to be used for this msgId.
                        sfid2[8:6] has the dma event number configuration. */
                        stdMsgIdFilter.sfid2 |= CANFD_lld_getFilterEventConfig(ptrCanMsgObj->dmaEventNo);
                    }

                    /* Store the message in rx buffer */
                    stdMsgIdFilter.sfec = 0x7U;
                    stdMsgIdFilter.sft = 0;
                    MCAN_lld_addStdMsgIDFilter(baseAddr, i, &stdMsgIdFilter);
                }
                else
                {
                    extMsgIdFilter.efid1 = ptrCanMsgObj->startMsgId & XTD_MSGID_MASK;
                    extMsgIdFilter.efid2 = i;
                    if (ptrCanMsgObj->enableDmaMode == TRUE)
                    {
                        /* Configure the filter element number to be used for this msgId.
                        efid2[8:6] has the dma event number configuration. */
                        extMsgIdFilter.efid2 |= CANFD_lld_getFilterEventConfig(ptrCanMsgObj->dmaEventNo);
                    }

                    /* Store the message in rx buffer */
                    extMsgIdFilter.efec = 0x7U;
                    extMsgIdFilter.eft = 0;
                    MCAN_lld_addExtMsgIDFilter(baseAddr, i, &extMsgIdFilter);
                }
            }
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function configures a receive message objects for a range of message Identifiersmessage object.
 *      It also enables Rx interrupts.
 *      The callback function will be invoked upon receiving data for receive message objects.
 *      The application MUST then call CANFD_lld_read() API to process the received data.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in]  msgObjectParams
 *      Message Object configuration parameters
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   Handle to the message object.
 *  @retval
 *      Error   -   NULL
 */
int32_t CANFD_lld_createRxRangeMsgObject(CANFDLLD_Handle handle, CANFDLLD_MessageObject* ptrCanMsgObj)
{
    uint32_t                    baseAddr;
    MCAN_StdMsgIDFilterElement  stdMsgIdFilter;
    MCAN_ExtMsgIDFilterElement  extMsgIdFilter;
    uint32_t                    i;
    int32_t                     retVal;

    /* Get the pointer to the CAN Driver Block */
    CANFDLLD_Object         *ptrCanFdObj = NULL;

    if ((handle == NULL) || (ptrCanMsgObj == NULL)  || (ptrCanMsgObj->startMsgId > ptrCanMsgObj->endMsgId))
    {
        retVal = SystemP_INVALID_PARAM;
    }
    else
    {
        retVal = SystemP_SUCCESS;

        ptrCanFdObj = (CANFDLLD_Object *) handle;

        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Save the specified parameters */
        ptrCanMsgObj->ptrCanFdObj = ptrCanFdObj;

        {
            /* Save the specified parameters */
            ptrCanMsgObj->ptrCanFdObj = ptrCanFdObj;
            ptrCanMsgObj->direction = CANFDLLD_Direction_RX;

            for (i = 0; i < MCAN_MAX_MSG_OBJECTS; i++)
            {
                if (ptrCanFdObj->msgObjectHandle[i] == NULL)
                {
                    break;
                }
            }
            if (i == MCAN_MAX_MSG_OBJECTS)
            {
                /* Error: Unable to allocate the memory */
                retVal = SystemP_OUT_OF_RESOURCES;
            }
            else
            {
                /* Store the message object handle for book keeping */
                ptrCanMsgObj->messageObjNum = i;
                ptrCanFdObj->msgObjectHandle[i] = ptrCanMsgObj;
            }

            /* Configure the Rx Message Id */
            if (retVal == SystemP_SUCCESS)
            {
                /* Get a free Rx element */
                for (i = 0; i < MCAN_MAX_RX_MSG_OBJECTS; i++)
                {
                    if (ptrCanFdObj->rxMapping[i] == NULL)
                    {
                        break;
                    }
                }
                if (i == MCAN_MAX_RX_MSG_OBJECTS)
                {
                    /* Error: Unable to allocate the memory */
                    retVal = SystemP_OUT_OF_RESOURCES;
                    ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;
                }
                else
                {
                    /* Store the rx to message object handle mapping */
                    ptrCanFdObj->rxMapping[i] = ptrCanMsgObj;
                    ptrCanMsgObj->rxElement = i;

                    /* Store the message RAM memory access type */
                    ptrCanMsgObj->memType = MCAN_MEM_TYPE_FIFO;

                    /* Add the filter to message RAM */
                    if (ptrCanMsgObj->msgIdType == CANFDLLD_MCANXidType_11_BIT)
                    {
                        stdMsgIdFilter.sfid1 = ptrCanMsgObj->startMsgId & STD_MSGID_MASK;
                        stdMsgIdFilter.sfid2 = ptrCanMsgObj->endMsgId & STD_MSGID_MASK;

                        /* Store the message in FIFO */
                        stdMsgIdFilter.sfec = (ptrCanFdObj->useFifoNum + 1U);
                        stdMsgIdFilter.sft = 0;
                        MCAN_lld_addStdMsgIDFilter(baseAddr, i, &stdMsgIdFilter);
                    }
                    else
                    {
                        extMsgIdFilter.efid1 = ptrCanMsgObj->startMsgId & XTD_MSGID_MASK;
                        extMsgIdFilter.efid2 = ptrCanMsgObj->endMsgId & XTD_MSGID_MASK;

                        /* Store the message in FIFO */
                        extMsgIdFilter.efec = (ptrCanFdObj->useFifoNum + 1U);
                        extMsgIdFilter.eft = 0;
                        MCAN_lld_addExtMsgIDFilter(baseAddr, i, &extMsgIdFilter);
                    }

                    /* Toggle the FIFO number for the next message Id */
                    ptrCanFdObj->useFifoNum = 1U - ptrCanFdObj->useFifoNum;
                }
            }
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function deletes a message object.
 *
 *  @param[in]  handle
 *      Handle to the message object
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_deleteMsgObject(CANFDLLD_MsgObjHandle handle)
{
    CANFDLLD_MessageObject*  ptrCanMsgObj;
    CANFDLLD_Object*      ptrCanFdObj;
    int32_t             retVal = 0;

    ptrCanMsgObj = (CANFDLLD_MessageObject*)handle;
    if (ptrCanMsgObj == NULL)
    {
        retVal = SystemP_INVALID_PARAM;
    }
    else
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFDLLD_Object*)ptrCanMsgObj->ptrCanFdObj;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_INVALID_PARAM;
        }
        else
        {
            ptrCanFdObj->msgObjectHandle[ptrCanMsgObj->messageObjNum] = NULL;

            if (ptrCanMsgObj->direction == CANFDLLD_Direction_TX)
            {
                ptrCanFdObj->txMapping[ptrCanMsgObj->txElement] = NULL;
                if (ptrCanMsgObj->enableDmaMode == TRUE)
                {
                    CANFD_lld_deleteDmaTxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }

            }

            if (ptrCanMsgObj->direction == CANFDLLD_Direction_RX)
            {
                ptrCanFdObj->rxMapping[ptrCanMsgObj->rxElement] = NULL;
                if (ptrCanMsgObj->enableDmaMode == TRUE)
                {
                    CANFD_lld_deleteDmaRxMsgObject(ptrCanFdObj, ptrCanMsgObj);
                }
            }
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function used by the application to transmit data.
 *
 *  @param[in]  handle
 *      Handle to the message object
 *  @param[in]  id
 *      Message Identifier
 *  @param[in]  frameType
 *      Frame type - Classic or FD
 *  @param[in]  dataLength
 *      Data Length to be transmitted.
 *      Valid values: 1 to 64 bytes.
 *  @param[in]  data
 *      Data to be transmitted
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_write(CANFDLLD_MsgObjHandle handle, uint32_t id, CANFDLLD_MCANFrameType frameType, uint32_t dataLength, const uint8_t* data)
{
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    CANFDLLD_Object*           ptrCanFdObj;
    int32_t                    retVal = 0;
    uint32_t                   baseAddr;
    MCAN_TxBufElement          txBuffElem;
    uint32_t                   index;
    uint8_t                    padSize = 0U;

    /* Get the message object pointer */
    ptrCanMsgObj = (CANFDLLD_MessageObject*)handle;
    if ((ptrCanMsgObj == NULL) || (data == NULL) || (dataLength < 1U) || (dataLength > 64U))
    {
        retVal = SystemP_INVALID_PARAM;
    }
    else
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFDLLD_Object*)ptrCanMsgObj->ptrCanFdObj;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_INVALID_PARAM;
        }
        else
        {
            baseAddr = ptrCanFdObj->regBaseAddress;

            /* Check for pending messages */
            index = (uint32_t)1U << ptrCanMsgObj->txElement;
            if (index == (MCAN_lld_getTxBufReqPend(baseAddr) & index))
            {
                retVal = SystemP_BUSY;
            }
            else
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

                /* Copy the data */
                memcpy ((void*)&txBuffElem.data, data, dataLength);

                /* Compute the DLC value */
                for(index = 0U ; index < 16U ; index++)
                {
                    if(dataLength <= ptrCanFdObj->mcanDataSize[index])
                    {
                        txBuffElem.dlc = index;
                        padSize = ptrCanFdObj->mcanDataSize[index] - dataLength;
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
                    index = dataLength;
                    while (padSize != 0)
                    {
                        txBuffElem.data[index++] = (uint8_t)0xCCU;
                        padSize--;
                    }

                    MCAN_lld_writeMsgRam(baseAddr, MCAN_MEM_TYPE_BUF, ptrCanMsgObj->txElement, &txBuffElem);

                    MCAN_lld_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);

                    ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = 1;

                    /* Increment the stats */
                    ptrCanMsgObj->messageProcessed++;
                }
            }
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function used by the application to cancel a pending data transmit.
 *
 *  @param[in]  handle
 *      Handle to the message object
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_writeCancel(CANFDLLD_MsgObjHandle handle)
{
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    CANFDLLD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;
    uint32_t                baseAddr;

    /* Get the message object pointer */
    ptrCanMsgObj = (CANFDLLD_MessageObject*)handle;
    if (ptrCanMsgObj == NULL)
    {
        retVal = SystemP_INVALID_PARAM;
    }
    else
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFDLLD_Object*)ptrCanMsgObj->ptrCanFdObj;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_INVALID_PARAM;
        }
        else
        {
            baseAddr = ptrCanFdObj->regBaseAddress;

            /* Cancel the pending transmit */
            MCAN_lld_txBufCancellationReq(baseAddr, ptrCanMsgObj->txElement);

            if (ptrCanMsgObj->enableDmaMode == TRUE)
            {
                CANFD_lld_cancelDmaTx(ptrCanFdObj, ptrCanMsgObj);
            }
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function is used by the application to get the CAN message from message RAM using a receive message object.
 *      NOTE: This API must ONLY be called from the callback context.
 *
 *  @param[in]  handle
 *      Handle to the message object
 *  @param[out]  id
 *      Message Identifier
 *  @param[out]  ptrFrameType
 *      Frame type - Classic or FD
 *  @param[out]  idType
 *      Meassage Id type - 11 bit standard or 29 bit extended
 *  @param[out]  ptrDataLength
 *      Data Length of the received frame.
 *      Valid values: 1 to 64 bytes.
 *  @param[out]  data
 *      Received data.
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_read(CANFDLLD_MsgObjHandle handle, uint32_t* id, CANFDLLD_MCANFrameType* ptrFrameType, CANFDLLD_MCANXidType* idType, uint32_t* ptrDataLength, uint8_t* data)
{
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    CANFDLLD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;

    /* Get the message object pointer */
    ptrCanMsgObj = (CANFDLLD_MessageObject*)handle;
    if ((ptrCanMsgObj == NULL) || (id == NULL) || (ptrDataLength == NULL) || (data == NULL))
    {
        retVal = SystemP_INVALID_PARAM;
    }
    else
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFDLLD_Object*)ptrCanMsgObj->ptrCanFdObj;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_INVALID_PARAM;
        }
        else
        {
            /* Get the data length from DLC */
            *ptrDataLength = ptrCanFdObj->mcanDataSize[ptrCanFdObj->rxBuffElem.dlc];

            /* Get the message Identifier */
            if(ptrCanFdObj->rxBuffElem.xtd == 1U)
            {
                /* Received frame with Extended ID */
                *id = (uint32_t)(ptrCanFdObj->rxBuffElem.id);
                *idType = CANFDLLD_MCANXidType_29_BIT;
            }
            else
            {
                /* Received frame with Standard ID */
                *id = (uint32_t)((ptrCanFdObj->rxBuffElem.id >> 18U) & 0x7FFU);
                *idType = CANFDLLD_MCANXidType_11_BIT;
            }

            /* Get the frame type */
            if(ptrCanFdObj->rxBuffElem.fdf == 1U)
            {
                /* FD frame Received */
                *ptrFrameType = CANFDLLD_MCANFrameType_FD;
            }
            else
            {
                /* Classic frame Received */
                *ptrFrameType = CANFDLLD_MCANFrameType_CLASSIC;
            }

            /* Copy the data */
            memcpy ((void *)data, ptrCanFdObj->rxBuffElem.data, *ptrDataLength);

            /* Increment the stats */
            ptrCanMsgObj->messageProcessed++;
        }
    }
    return retVal;
}

int32_t CANFD_lld_readDmaConfig(CANFDLLD_MsgObjHandle handle, void* data, uint32_t numDmaRxBuf)
{
    CANFDLLD_MessageObject*    ptrCanMsgObj;
    CANFDLLD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_SUCCESS;

    /* Get the message object pointer */
    ptrCanMsgObj = (CANFDLLD_MessageObject*)handle;
    if ((ptrCanMsgObj == NULL) || (data == NULL))
    {
        retVal = SystemP_INVALID_PARAM;
    }
    else
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFDLLD_Object*)ptrCanMsgObj->ptrCanFdObj;

        if (ptrCanFdObj == NULL)
        {
            retVal = SystemP_INVALID_PARAM;
        }
        else
        {
            retVal = CANFD_lld_configureDmaRx(ptrCanFdObj, ptrCanMsgObj, sizeof(CANFDLLD_DmaRxBuf), numDmaRxBuf, data);
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function is used by the application to get the error and status information from the driver.
 *
 *  @param[in]  handle
 *      Handle to the CAN Driver
 *  @param[out] ptrOptInfo
 *      Option info in TLV format which is populated with the requested information
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_getOptions(CANFDLLD_Handle handle, CANFDLLD_OptionTLV* ptrOptInfo)
{

    CANFDLLD_Object*               ptrCanFdObj;
    CANFDLLD_MessageObject*        ptrCanMsgObj;
    CANFDLLD_MCANErrCntStatus*     ptrErrCounter;
    CANFDLLD_MCANProtocolStatus*   ptrProtoStatus;
    CANFDLLD_MCANMsgObjectStats*   ptrMsgObjectStats;
    int32_t                     retVal = 0;
    MCAN_ErrCntStatus           cslErrStatus;
    MCAN_ProtocolStatus         cslProtocolStatus;

    /* Get the pointer to the CAN Driver Block */
    ptrCanFdObj = (CANFDLLD_Object*)handle;
    if ((ptrCanFdObj == NULL) || (ptrOptInfo == NULL))
    {
        retVal = SystemP_INVALID_PARAM;
    }
    else
    {
        /* Process the supported options */
        switch (ptrOptInfo->type)
        {
            case CANFDLLD_Option_MCAN_ERROR_COUNTER:
            {
                /* Sanity Check: Validate the arguments. */
                if (ptrOptInfo->length != sizeof(CANFDLLD_MCANErrCntStatus))
                {
                    retVal = SystemP_INVALID_PARAM;
                    goto endGetOptions;
                }

                ptrErrCounter = (CANFDLLD_MCANErrCntStatus*) ptrOptInfo->value;
                if (ptrErrCounter == NULL)
                {
                    retVal = SystemP_INVALID_PARAM;
                    goto endGetOptions;
                }

                /* Populate the stats */
                MCAN_lld_getErrCounters (ptrCanFdObj->regBaseAddress, &cslErrStatus);
                ptrErrCounter->canErrLogCnt = cslErrStatus.canErrLogCnt;
                ptrErrCounter->recErrCnt    = cslErrStatus.recErrCnt;
                ptrErrCounter->rpStatus     = cslErrStatus.rpStatus;
                ptrErrCounter->transErrLogCnt = cslErrStatus.transErrLogCnt;
                break;
            }
            case CANFDLLD_Option_MCAN_PROTOCOL_STATUS:
            {
                /* Sanity Check: Validate the arguments. */
                if (ptrOptInfo->length != sizeof(CANFDLLD_MCANProtocolStatus))
                {
                    retVal = SystemP_INVALID_PARAM;
                    goto endGetOptions;
                }

                ptrProtoStatus = (CANFDLLD_MCANProtocolStatus*) ptrOptInfo->value;
                if (ptrProtoStatus == NULL)
                {
                    retVal = SystemP_INVALID_PARAM;
                    goto endGetOptions;
                }

                /* Populate the stats */
                MCAN_lld_getProtocolStatus (ptrCanFdObj->regBaseAddress, &cslProtocolStatus);
                ptrProtoStatus->act           = cslProtocolStatus.act;
                ptrProtoStatus->busOffStatus  = cslProtocolStatus.busOffStatus;
                ptrProtoStatus->dlec          = cslProtocolStatus.dlec;
                ptrProtoStatus->errPassive    = cslProtocolStatus.errPassive;
                ptrProtoStatus->lastErrCode   = cslProtocolStatus.lastErrCode;
                ptrProtoStatus->pxe           = cslProtocolStatus.pxe;
                ptrProtoStatus->rbrs          = cslProtocolStatus.rbrs;
                ptrProtoStatus->resi          = cslProtocolStatus.resi;
                ptrProtoStatus->rfdf          = cslProtocolStatus.rfdf;
                ptrProtoStatus->tdcv          = cslProtocolStatus.tdcv;
                ptrProtoStatus->warningStatus = cslProtocolStatus.warningStatus;
                break;
            }
            case CANFDLLD_Option_MCAN_MSG_OBJECT_STATS:
            {
                /* Sanity Check: Validate the arguments. */
                if (ptrOptInfo->length != sizeof(CANFDLLD_MCANMsgObjectStats))
                {
                    retVal = SystemP_INVALID_PARAM;
                    goto endGetOptions;
                }

                ptrMsgObjectStats = (CANFDLLD_MCANMsgObjectStats*) ptrOptInfo->value;
                if (ptrMsgObjectStats == NULL)
                {
                    retVal = SystemP_INVALID_PARAM;
                    goto endGetOptions;
                }

                ptrCanMsgObj = (CANFDLLD_MessageObject*) ptrMsgObjectStats->handle;
                if (ptrCanMsgObj == NULL)
                {
                    retVal = SystemP_INVALID_PARAM;
                    goto endGetOptions;
                }

                /* Populate the stats */
                ptrMsgObjectStats->startMsgIdentifier = ptrCanMsgObj->startMsgId;
                ptrMsgObjectStats->endMsgIdentifier = ptrCanMsgObj->endMsgId;
                ptrMsgObjectStats->direction = ptrCanMsgObj->direction;
                ptrMsgObjectStats->interruptsRxed = ptrCanMsgObj->interruptsRxed;
                ptrMsgObjectStats->messageProcessed = ptrCanMsgObj->messageProcessed;
                break;
            }
            default:
            {
                /* Option is NOT supported */
                retVal = SystemP_INVALID_PARAM;
                break;
            }
        }
    }

endGetOptions:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function is used by the application to configure the driver options.
 *
 *  @param[in]  handle
 *      Handle to the CANFD Driver
 *  @param[in] ptrOptInfo
 *      Option info in TLV format which is used to configure the driver
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CANFD_lld_setOptions(CANFDLLD_Handle handle, const CANFDLLD_OptionTLV* ptrOptInfo)
{
    CANFDLLD_Object*                   ptrCanFdObj;
    uint32_t                        baseAddr;
    CANFDLLD_MCANLoopbackCfgParams*    ptrMcanLoopBackCfg;
    int32_t                         retVal = SystemP_SUCCESS;
    /* Get the pointer to the CAN Driver Block */
    ptrCanFdObj = (CANFDLLD_Object*)handle;
    if ((ptrCanFdObj == NULL) || (ptrOptInfo == NULL))
    {
        retVal = SystemP_INVALID_PARAM;
        goto endSetOptions;
    }

    /* Process the supported options */
    switch (ptrOptInfo->type)
    {
        case CANFDLLD_Option_MCAN_MODE:
        {
            /* Sanity Check: Validate the arguments. */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                retVal = SystemP_INVALID_PARAM;
                goto endSetOptions;
            }

            baseAddr = ptrCanFdObj->regBaseAddress;

            if ((*(uint8_t*)ptrOptInfo->value == 1U) || (*(uint8_t*)ptrOptInfo->value == 0U))
            {
                /* Put MCAN in soecified mode */
                MCAN_lld_setOpMode(baseAddr, (uint32_t)(*(uint8_t*)ptrOptInfo->value));

                /* Wait while MCAN is not in init mode. TODO add timeout */
                do
                {
                } while (MCAN_lld_getOpMode(baseAddr) != *(uint8_t*)ptrOptInfo->value);
            }
            else
            {
                retVal = SystemP_INVALID_PARAM;
            }
            break;
        }
        case CANFDLLD_Option_MCAN_LOOPBACK:
        {
            /* Sanity Check: Validate the arguments. */
            if (ptrOptInfo->length != sizeof(CANFDLLD_MCANLoopbackCfgParams))
            {
                retVal = SystemP_INVALID_PARAM;
                goto endSetOptions;
            }

            baseAddr = ptrCanFdObj->regBaseAddress;

            ptrMcanLoopBackCfg = (CANFDLLD_MCANLoopbackCfgParams*) ptrOptInfo->value;

            /* Put MCAN in SW initialization mode */
            MCAN_lld_setOpMode(baseAddr, MCAN_OPERATION_MODE_SW_INIT);

            /* Wait while MCAN is not in init mode. TODO add timeout */
            do
            {
            } while (MCAN_lld_getOpMode(baseAddr) != MCAN_OPERATION_MODE_SW_INIT);

            /* Disable loopback mode */
            if (ptrMcanLoopBackCfg->enable == 0)
            {
                MCAN_lld_lpbkModeEnable(baseAddr, ptrMcanLoopBackCfg->mode, 0U);
            }
            else
            {
                /* Enable loopback mode */
                MCAN_lld_lpbkModeEnable(baseAddr, ptrMcanLoopBackCfg->mode, 1U);
            }
            /* Put MCAN in opertional mode */
            MCAN_lld_setOpMode(baseAddr, MCAN_OPERATION_MODE_NORMAL);

            /* Wait while MCAN is not in operational mode. TODO add timeout */
            do
            {
            } while (MCAN_lld_getOpMode(baseAddr) != MCAN_OPERATION_MODE_NORMAL);

            break;
        }
        case CANFDLLD_Option_MCAN_POWER_DOWN:
        {
            /* Sanity Check: Validate the arguments. */
            if (ptrOptInfo->length != sizeof(uint8_t))
            {
                retVal = SystemP_INVALID_PARAM;
                goto endSetOptions;
            }

            baseAddr = ptrCanFdObj->regBaseAddress;

            if (*(uint8_t*)ptrOptInfo->value == 1U)
            {
                /* Request a clock stop to enter local power down */
                MCAN_lld_addClockStopRequest(baseAddr, 1U);

                /* Wait for power down to be successful */
                do
                {
                } while (MCAN_lld_getClkStopAck(baseAddr) != CSL_MCAN_CCCR_CSA_ACK);

                /* Update the state information */
                ptrCanFdObj->state = CANFDLLD_DriverState_SLEEP;
            }
            else if (*(uint8_t*)ptrOptInfo->value == 0U)
            {
                /* Turn on the local clocks to wakeup from local power down */
                MCAN_lld_addClockStopRequest(baseAddr, 0);

                /* Wait for wake up to be successful */
                do
                {
                } while (MCAN_lld_getClkStopAck(baseAddr) != CSL_MCAN_CCCR_CSA_NO_ACK);

                /* Put MCAN in opertional mode */
                MCAN_lld_setOpMode(baseAddr, MCAN_OPERATION_MODE_NORMAL);

                /* Wait while MCAN is not in operational mode. TODO add timeout */
                do
                {
                } while (MCAN_lld_getOpMode(baseAddr) != MCAN_OPERATION_MODE_NORMAL);

                /* Update the state information */
                ptrCanFdObj->state = CANFDLLD_DriverState_STARTED;
            }
            else
            {
                retVal = SystemP_INVALID_PARAM;
            }
            break;
        }
        default:
        {
            /* Option is NOT supported */
            retVal = SystemP_INVALID_PARAM;
            break;
        }
    }
endSetOptions:
    return retVal;
}


__attribute__((weak)) void CANFD_lld_dataAppCallBack(CANFDLLD_MsgObjHandle handle, CANFDLLD_Reason reason)
{
    return;
}

__attribute__((weak)) void CANFD_lld_errStatusAppCallBack(CANFDLLD_Handle handle, CANFDLLD_Reason reason, CANFDLLD_ErrStatusResp* errStatusResp)
{
    return;
}
