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

/* This example demonstrates the CAN message transmission and reception in
 * digital loop back mode with the following configuration.
 *
 * CAN FD Message Format.
 * Message ID Type is extended, Msg Id 0x29E.
 * Configures DMA channels for transmission and receiption.
 * Arbitration Bit Rate 1Mbps.
 * Data Bit Rate 5Mbps.
 * 
 *
 * Message is transmitted and received back internally using internal loopback
 * mode. When the received message id and the data matches with the transmitted
 * one, then the example is completed.
 *
 */

#include <stdio.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/mcan.h>
#include <drivers/mcan/v0/lld/canfd_lld.h>
#include <drivers/mcan/v0/lld/dma/edma/canfd_lld_dma_edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/** \brief Number of messages sent */
#define MCAN_APP_TEST_MESSAGE_COUNT         10U

/** \brief Number of messages sent */
#define MCAN_APP_TEST_DATA_SIZE             64U

/* CANFD Driver objects */
CANFDLLD_Object                gCanfdObj;
CANFDLLD_EdmaChConfig          gCanfdEdmaChConfig;
static HwiP_Object             gMcanHwiObject;

CANFDLLD_MessageObject         txMsgObject;
CANFDLLD_MessageObject         rxMsgObject;

volatile uint32_t           gTxDoneFlag = 0, gRxDoneFlag = 0;
volatile uint32_t           iterationCount = 0U;
uint32_t                    gErrGetData = 0U;
uint32_t                    gErrDataMissMatchCnt = 0U;
uint32_t                    txDataLength, rxDataLength, testDataLength = 0U;
uint8_t                     txData[MCAN_APP_TEST_MESSAGE_COUNT * MCAN_APP_TEST_DATA_SIZE] = {0};
uint8_t                     rxData[(MCAN_MSG_HEADER_SIZE * MCAN_APP_TEST_MESSAGE_COUNT) + (MCAN_APP_TEST_MESSAGE_COUNT * MCAN_APP_TEST_DATA_SIZE)] = {0};

static void MCANAppInitParams(CANFDLLD_MCANInitParams* mcanCfgParams);
static void App_mcanCompareMsg(uint8_t *txMsg, uint8_t *rxMsg);

void mcan_loopback_dma_lld_main(void *args)
{
    CANFDLLD_Handle                canHandle;
    CANFDLLD_MsgObjHandle          txMsgObjHandle;
    CANFDLLD_MsgObjHandle          rxMsgObjHandle;
    int32_t                        retVal = 0;
    CANFDLLD_OptionTLV             optionTLV;
    CANFDLLD_MCANBitTimingParams   mcanBitTimingParams;
    CANFDLLD_MCANLoopbackCfgParams mcanloopbackParams;
    uint32_t                       i;
    HwiP_Params                    hwiPrms;
    int32_t                        status = SystemP_SUCCESS;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* initialize the TX data buffers */
    for (i=0; i<MCAN_APP_TEST_MESSAGE_COUNT * MCAN_APP_TEST_DATA_SIZE; i++)
    {
        txData[i] = i%256;
    }

    /* initialize the RX data buffers */
    memset (rxData, 0, sizeof (rxData));

    DebugP_log("[MCAN] Loopback DMA mode, application started ...\r\n");

    gTxDoneFlag = 0;

    /* Base address macro is taken from syscfg generated file ti_drivers_config.h based on instance selected. */
    gCanfdObj.regBaseAddress = CONFIG_MCAN0_BASE_ADDR;
    gCanfdObj.state = CANFDLLD_DriverState_UNINIT;
    MCANAppInitParams (&gCanfdObj.initParams);

    gCanfdObj.canfdDmaHandle = gEdmaHandle[0];
    gCanfdEdmaChConfig.edmaRxChId[0] = EDMA_APPSS_TPCC_A_EVT_MCAN_FE_INT1;
    gCanfdEdmaChConfig.edmaTxChId[0] = EDMA_APPSS_TPCC_A_EVT_MCAN_DMA_REQ0;
    gCanfdObj.canfdDmaChCfg = (CANFDLLD_DmaChConfig *)(&gCanfdEdmaChConfig);

    canHandle = (CANFDLLD_Handle)(&gCanfdObj);

    /* Initialize the CANFD driver */
    retVal = CANFD_lld_init(canHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD Module Initialization failed \n");
        return;
    }

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_MCAN0_INTR;
    hwiPrms.callback    = (void*)&CANFD_lld_int0Isr;
    hwiPrms.args        = &gCanfdObj;
    status              = HwiP_construct(&gMcanHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configuring 1Mbps and 5Mbps as nominal and data bit-rate respectively
        Prop seg: 8
        Ph seg 1: 6
        Ph Seg2 : 5
        Sync jump: 1
        BRP(Baud rate Prescaler): 2

        Nominal Bit rate = (80)/(((8+6+5)+1)*BRP) = 1Mhz

        Timing Params for Data Bit rate:
        Prop seg: 2
        Ph seg 1: 2
        Ph Seg2 : 3
        Sync jump: 1
        BRP(Baud rate Prescaler): 1

        Nominal Bit rate = (80)/(((2+2+3)+1)*BRP) = 5Mhz
    */

    mcanBitTimingParams.nomBrp      = 0x4U;
    mcanBitTimingParams.nomPropSeg  = 0x8U;
    mcanBitTimingParams.nomPseg1    = 0x6U;
    mcanBitTimingParams.nomPseg2    = 0x5U;
    mcanBitTimingParams.nomSjw      = 0x1U;

    mcanBitTimingParams.dataBrp     = 0x2U;
    mcanBitTimingParams.dataPropSeg = 0x2U;
    mcanBitTimingParams.dataPseg1   = 0x2U;
    mcanBitTimingParams.dataPseg2   = 0x3U;
    mcanBitTimingParams.dataSjw     = 0x1U;

    /* Configure the CAN driver */
    retVal = CANFD_lld_configBitTime (canHandle, &mcanBitTimingParams);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD Module configure bit time failed \n");
        return;
    }

    /* Enable Internal Loopback mode */
    mcanloopbackParams.enable = 1U;
    mcanloopbackParams.mode = CANFDLLD_MCANLoopBackMode_INTERNAL;

    optionTLV.type = CANFDLLD_Option_MCAN_LOOPBACK;
    optionTLV.length = sizeof(CANFDLLD_MCANLoopbackCfgParams);
    optionTLV.value = (void*) &mcanloopbackParams;

    retVal =  CANFD_lld_setOptions(canHandle, &optionTLV);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD set option Loopback failed \n");
        return;
    }

    /* Setup the receive message object */
    rxMsgObject.direction = CANFDLLD_Direction_RX;
    rxMsgObject.msgIdType = CANFDLLD_MCANXidType_29_BIT;
    rxMsgObject.startMsgId = 0x29E;
    rxMsgObject.endMsgId = 0x29E;
    rxMsgObject.enableDmaMode = TRUE;

    retVal = CANFD_lld_createMsgObject (canHandle, &rxMsgObject);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Rx message object failed\n");
        return;
    }
    rxMsgObjHandle = &rxMsgObject;

    /* Setup the transmit message object */
    txMsgObject.direction = CANFDLLD_Direction_TX;
    txMsgObject.msgIdType = CANFDLLD_MCANXidType_29_BIT;
    txMsgObject.startMsgId = 0x29E;
    txMsgObject.endMsgId = 0x29E;
    txMsgObject.enableDmaMode = TRUE;

    retVal = CANFD_lld_createMsgObject (canHandle, &txMsgObject);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD create Tx message object failed\n");
        return;
    }
    txMsgObjHandle = &txMsgObject;

    retVal = CANFD_lld_readDmaConfig(rxMsgObjHandle, rxData, MCAN_APP_TEST_MESSAGE_COUNT);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD read dma config failed\n");
        return;
    }
    retVal = CANFD_lld_writeDma(txMsgObjHandle, txMsgObject.startMsgId, CANFDLLD_MCANFrameType_FD, MCAN_APP_TEST_DATA_SIZE, MCAN_APP_TEST_MESSAGE_COUNT, &txData[0]);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD write dma config failed\n");
        return;
    }

    while (iterationCount != MCAN_APP_TEST_MESSAGE_COUNT - 1)
    {
        {
            while (gTxDoneFlag == 0);
            gTxDoneFlag = 0;

            /* Send next data over Tx message object */
            retVal = CANFD_lld_writeDmaTriggerNext (txMsgObjHandle);
            if (retVal != SystemP_SUCCESS)
            {
                DebugP_log ("Error: CANFD transmit next data for iteration %d failed \n", iterationCount);
                return;
            }
        }

        iterationCount++;
    }

    /* Wait for RX to complete*/
    while (gRxDoneFlag != MCAN_APP_TEST_MESSAGE_COUNT);

    /* Verify messages */
    App_mcanCompareMsg(txData, rxData);

    retVal = CANFD_lld_deleteMsgObject(txMsgObjHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Tx message object failed \n");
        return;
    }

    retVal = CANFD_lld_deleteMsgObject(rxMsgObjHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD delete Rx message object failed \n");
        return;
    }

    retVal = CANFD_lld_deInit(canHandle);
    if (retVal != SystemP_SUCCESS)
    {
        DebugP_log ("Error: CANFD deinit failed \n");
        return;
    }

    DebugP_log("[MCAN] Internal loopback testing for %d iterations Passed\n", iterationCount);
    DebugP_log("All tests have passed!!\r\n");

    return;
}

static void MCANAppInitParams(CANFDLLD_MCANInitParams* mcanCfgParams)
{
    /*Intialize MCAN Config Params*/
    memset (mcanCfgParams, 0, sizeof (CANFDLLD_MCANInitParams));

    mcanCfgParams->fdMode          = 0x1U;
    mcanCfgParams->brsEnable       = 0x1U;
    mcanCfgParams->txpEnable       = 0x0U;
    mcanCfgParams->efbi            = 0x0U;
    mcanCfgParams->pxhddisable     = 0x0U;
    mcanCfgParams->darEnable       = 0x1U;
    mcanCfgParams->wkupReqEnable   = 0x1U;
    mcanCfgParams->autoWkupEnable  = 0x1U;
    mcanCfgParams->emulationEnable = 0x0U;
    mcanCfgParams->emulationFAck   = 0x0U;
    mcanCfgParams->clkStopFAck     = 0x0U;
    mcanCfgParams->wdcPreload      = 0x0U;
    mcanCfgParams->tdcEnable       = 0x1U;
    mcanCfgParams->tdcConfig.tdcf  = 0U;
    mcanCfgParams->tdcConfig.tdco  = 8U;
    mcanCfgParams->monEnable       = 0x0U;
    mcanCfgParams->asmEnable       = 0x0U;
    mcanCfgParams->tsPrescalar     = 0x0U;
    mcanCfgParams->tsSelect        = 0x0U;
    mcanCfgParams->timeoutSelect   = CANFDLLD_MCANTimeOutSelect_CONT;
    mcanCfgParams->timeoutPreload  = 0x0U;
    mcanCfgParams->timeoutCntEnable= 0x0U;
    mcanCfgParams->filterConfig.rrfe        = 0x1U;
    mcanCfgParams->filterConfig.rrfs        = 0x1U;
    mcanCfgParams->filterConfig.anfe        = 0x3U;
    mcanCfgParams->filterConfig.anfs        = 0x3U;
    mcanCfgParams->msgRAMConfig.lss         = 127U;
    mcanCfgParams->msgRAMConfig.lse         = 64U;
    mcanCfgParams->msgRAMConfig.txBufNum    = 32U;
    mcanCfgParams->msgRAMConfig.txFIFOSize  = 0U;
    mcanCfgParams->msgRAMConfig.txBufMode   = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOSize         = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOWaterMark    = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0size             = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO0OpMode           = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0waterMark        = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO1size             = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1waterMark        = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO1OpMode           = 0U;

    mcanCfgParams->eccConfig.enable         = 1U;
    mcanCfgParams->eccConfig.enableChk      = 1U;
    mcanCfgParams->eccConfig.enableRdModWr  = 1U;

    mcanCfgParams->errInterruptEnable   = 1U;
    mcanCfgParams->dataInterruptEnable  = 1U;
}

static void App_mcanCompareMsg(uint8_t *txMsg, uint8_t *rxMsg)
{
    uint32_t i, j;

    for (i = 0U; i < MCAN_APP_TEST_MESSAGE_COUNT; i++)
    {
        rxMsg += MCAN_MSG_HEADER_SIZE;

        for(j = 0U; j < MCAN_APP_TEST_DATA_SIZE; j++)
        {
            if (*txMsg != *rxMsg)
            {
                DebugP_logError("Data mismatch !!!\r\n");
                DebugP_assert(FALSE);
            }
            txMsg++;
            rxMsg++;
        }
    }

    return;
}

void CANFD_dmaTxCompletionCallback(CANFDLLD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{
    if (ptrCanMsgObj == &txMsgObject){
        gTxDoneFlag = 1;
        return;
    }
}

void CANFD_dmaRxCompletionCallback(CANFDLLD_MessageObject* ptrCanMsgObj, void *data, uint32_t completionType)
{
    if (ptrCanMsgObj == &rxMsgObject){
        gRxDoneFlag += 1;
        return;
    }
}
