/*
 *  NOTE:
 *      (C) Copyright 2018 - 2023 Texas Instruments, Inc.
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
 *   @file  zoomproc.c
 *
 *   @brief
 *      Implements Zoom FFT data processing Unit using HWA.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

// #define SOC_AWR294X

/* MCU+SDK Include files */
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/edma.h>

/* Data Path Include files */
#include <datapath/dpu/zoomproc/v0/zoomproc.h>

/* MATH utils library Include files */
#include <utils/mathutils/mathutils.h>

/* User defined heap memory and handle */
#define ZOOMPROC_HEAP_MEM_SIZE  (sizeof(ZoomProcObj))

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1
#define DEBUG_CHECK_EDMA     0

#define DPU_ZOOMPROC_MEM_BANK_INDX_SRC   0
#define DPU_ZOOMPROC_MEM_BANK_INDX_DST_P0   1
#define DPU_ZOOMPROC_MEM_BANK_INDX_DST_P1   2
#define DPU_ZOOMPROC_MEM_BANK_INDX_DST_P2   3

#define DPU_ZOOMPROC_SRCADDR   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(zoomProcObj->hwaMemBankAddr[DPU_ZOOMPROC_MEM_BANK_INDX_SRC])
#define DPU_ZOOMPROC_DSTADDR_P0   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(zoomProcObj->hwaMemBankAddr[DPU_ZOOMPROC_MEM_BANK_INDX_DST_P0])
#define DPU_ZOOMPROC_DSTADDR_P1   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(zoomProcObj->hwaMemBankAddr[DPU_ZOOMPROC_MEM_BANK_INDX_DST_P1])
#define DPU_ZOOMPROC_DSTADDR_P2   HWADRV_HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(zoomProcObj->hwaMemBankAddr[DPU_ZOOMPROC_MEM_BANK_INDX_DST_P2])

static uint8_t gZoomProcHeapMem[ZOOMPROC_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

#if DEBUG_CHECK_EDMA
static SemaphoreP_Object gEdmaTestDoneSem;
Edma_IntrObject     intrObj;
#endif

HWA_ParamConfig gHwaParams = {0};

/**************************************************************************
 ************************ Internal Functions Prototype       **********************
 **************************************************************************/
static void zoomProcDoneIsrCallback(void * arg);
/*static void zoomProc_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode);*/
void zoomProc_EDMA_transferCompletionCallbackFxn(Edma_IntrHandle intrHandle,
   void *args);

#if DEBUG_CHECK_EDMA
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);
#endif

static int32_t zoomProc_ConfigHWA
(
    ZoomProcObj     *zoomProcObj,
    uint8_t     destChan
);

static int32_t zoomProc_TriggerHWA
(
    ZoomProcObj     *zoomProcObj
);

static int32_t zoomProc_ConfigEDMA_DataOut
(
    ZoomProcObj         *zoomProcObj,
    DPU_ZoomProc_HW_Resources *pHwConfig
);
static int32_t zoomProc_ConfigEDMA_DataIn
(
    ZoomProcObj         *zoomProcObj,
    DPU_ZoomProc_HW_Resources *pHwConfig
);
static int32_t zoomProc_ConifgureHwaEdma
(
    ZoomProcObj          *zoomProcObj,
    DPU_ZoomProc_HW_Resources *pHwConfig
);

/**************************************************************************
 ************************ZoomProc Internal Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 *
 *  @param[in]  arg                 Argument to the callback function
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
volatile uint32_t hwazoomdoneisr = 0;
static void zoomProcDoneIsrCallback(void * arg)
{
    hwazoomdoneisr++;
    if (arg != NULL) 
    {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/* TODO: Cleanup - Internal debug function */
#if DEBUG_CHECK_EDMA
volatile uint32_t EdmaInCallbackcnt = 0;
static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    EdmaInCallbackcnt++;
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}
#endif

/**
 *  @b Description
 *  @n
 *      EDMA processing completion call back function as per EDMA API.
 *
 *  @param[in]  arg                     Argument to the callback function
 *  @param[in]  transferCompletionCode  EDMA transfer complete code
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
volatile uint32_t EdmaZoomCallbackcnt = 0;
void zoomProc_EDMA_transferCompletionCallbackFxn(Edma_IntrHandle intrHandle,
   void *args)
{
    ZoomProcObj     *zoomProcObj;

    /* Get zoomProc object */
    zoomProcObj = (ZoomProcObj *)args;

    EdmaZoomCallbackcnt++;
    if (intrHandle->tccNum == zoomProcObj->dataOutZoomChan)
    {
        zoomProcObj->numEdmaDataOutCnt++;
        SemaphoreP_post(&zoomProcObj->edmaDoneSemaHandle);
    }
}

/**
 *  @b Description
 *  @n
 *      Allocates Shawdow paramset
 */
static void allocateEDMAShadowChannel(EDMA_Handle handle, uint32_t *param)
{
    int32_t testStatus = SystemP_SUCCESS;

    testStatus = EDMA_allocParam(handle, param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    return;
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA to perform zoom FFT
 *
 *  @param[in]  zoomProcObj                  Pointer to zoomProc object
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_ConfigHWACommon
(
    ZoomProcObj     *zoomProcObj
)
{
    HWA_CommonConfig    hwaCommonConfig;
    int32_t             retVal;


    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
                               HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                               HWA_COMMONCONFIG_MASK_LFSRSEED;

    hwaCommonConfig.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaCommonConfig.fftConfig.lfsrSeed = 0x1234567; /*Some non-zero value*/
    hwaCommonConfig.numLoops = 1;
    hwaCommonConfig.paramStartIdx = zoomProcObj->hwaCfg.paramSetStartIdx;
    hwaCommonConfig.paramStopIdx = zoomProcObj->hwaCfg.paramSetStartIdx + zoomProcObj->hwaCfg.numParamSet - 1U;
    
    /* HWA will input data from M0 memory*/
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    
    retVal = HWA_configCommon(zoomProcObj->initParms.hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(zoomProcObj->initParms.hwaHandle,
                                        zoomProcDoneIsrCallback,
                                        (void*)&zoomProcObj->hwaDoneSemaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA to perform zoom FFT
 *
 *  @param[in]  zoomProcObj                  Pointer to zoomProc object
 *  @param[in]  destChan                     Destination channel id
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_ConfigHWA
(
    ZoomProcObj     *zoomProcObj,
    uint8_t     destChan)
{
    uint16_t    hwaMemSrcOffset;
    uint16_t    hwaMemDstOffset;

    HWA_InterruptConfig     paramISRConfig;
    int32_t                 errCode = 0;
    uint32_t                hwParamsetIdx;
    HWA_ParamConfig         hwaParamCfg;
    HWA_Handle              hwaHandle;
    DPU_ZoomProc_StaticConfig   *pDPParams;

    hwaMemSrcOffset     = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(zoomProcObj->hwaMemBankAddr[0]);
    hwaMemDstOffset     = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(zoomProcObj->hwaMemBankAddr[2]);
    

    hwaHandle = zoomProcObj->initParms.hwaHandle;
    pDPParams = &zoomProcObj->params;
    hwParamsetIdx = zoomProcObj->hwaCfg.paramSetStartIdx;
    
    memset(&hwaParamCfg,0,sizeof(hwaParamCfg));

    if (zoomProcObj->hwaCfg.dataInputMode == DPU_ZoomProc_InputMode_ISOLATED)
    {
        /* EDMA IN for ADC data */
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg.dmaTriggerSrc = zoomProcObj->dataInTrigger;
        hwaParamCfg.complexMultiply.cmpMulArgs.dft.startFreq = *zoomProcObj->zoomInRbin;
    }
    else
    {
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.complexMultiply.cmpMulArgs.dft.startFreq = 0; //TWID INCR REG - to be updated based on peak freq loc for zoom
    }
            
    hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_SLOW_DFT;
    hwaParamCfg.complexMultiply.cmpMulArgs.dft.freqIncrement = mathUtils_ceilLog2(pDPParams->zoomFftSize); //FFT SIZE
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;

    hwaParamCfg.source.srcAddr = hwaMemSrcOffset;
    hwaParamCfg.source.srcAcnt = pDPParams->ADCBufData.dataProperty.numAdcSamples - 1;
    hwaParamCfg.source.srcAIdx = sizeof(uint16_t);
    hwaParamCfg.source.srcBcnt = pDPParams->numDftBins - 1;
    hwaParamCfg.source.srcBIdx = 0;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcConjugate = 0;
    hwaParamCfg.source.srcScale = 8;


    hwaParamCfg.dest.dstAddr =  hwaMemDstOffset;
    hwaParamCfg.dest.dstAcnt = 1 - 1; //1 DFT output bin 'k' at a time
    hwaParamCfg.dest.dstAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstBIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.dest.dstConjugate = 0;
    hwaParamCfg.dest.dstScale = 8;
    hwaParamCfg.dest.dstSkipInit = 0;
    
    /* Save HWA parameters */
    memcpy((void *)&gHwaParams, (void *)&hwaParamCfg, sizeof(HWA_ParamConfig));

    errCode = HWA_configParamSet(hwaHandle,
                                    hwParamsetIdx,
                                    &hwaParamCfg,NULL);
    if (errCode != 0)
    {
        goto exit;
    }
    errCode = HWA_disableParamSetInterrupt(hwaHandle, hwParamsetIdx,
            HWA_PARAMDONE_INTERRUPT_TYPE_CPU |HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
    if (errCode != 0)
    {
        goto exit;
    }
    
    /* enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChan;

    errCode = HWA_enableParamSetInterrupt(hwaHandle, hwParamsetIdx, &paramISRConfig);
    if (errCode != 0)
    {
        goto exit;
    }
    
    hwParamsetIdx++;
    zoomProcObj->hwaCfg.numParamSet = hwParamsetIdx - zoomProcObj->hwaCfg.paramSetStartIdx;

exit:
    return(errCode);
}

/**
 *  @b Description
 *  @n
 *      Trigger HWA for zoom processing.
 *
 *  @param[in]  zoomProcObj              Pointer to zoomProc object
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_TriggerHWA
(
    ZoomProcObj     *zoomProcObj
)
{
    int32_t             retVal = 0;
    HWA_Handle          hwaHandle;

    /* Get HWA driver handle */
    hwaHandle = zoomProcObj->initParms.hwaHandle;

    /* Configure HWA common parameters */
    retVal = zoomProc_ConfigHWACommon(zoomProcObj);
    if(retVal < 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(hwaHandle, 1);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}


/**
 *  @b Description
 *  @n
 *      EDMA configuration for zoomProc data output
 *
 *  @param[in]  zoomProcObj              Pointer to zoomProc object
 *  @param[in]  pHwConfig                Pointer to zoomProc hardware resources
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_ConfigEDMA_DataOut
(
    ZoomProcObj         *zoomProcObj,
    DPU_ZoomProc_HW_Resources *pHwConfig
)
{
    int32_t                 errorCode = SystemP_SUCCESS;
    EDMA_Handle             handle;
    DPEDMA_syncABCfg        syncABCfg;
    DPEDMA_ChainingCfg      chainingCfg;
    DPU_ZoomProc_StaticConfig       *DPParams= &zoomProcObj->params;
    bool                    isIntermediateTransferCompletionEnabled;
    bool                    isTransferCompletionEnabled;
    Edma_EventCallback      transferCompletionCallbackFxn;
    

    handle = pHwConfig->edmaHandle;
        
    /**************************************************************************
    *  Configure EDMA to copy from HWA memory to DFT out memory
    *************************************************************************/

    syncABCfg.aCount = DPParams->numDftBins * sizeof(cmplx32ImRe_t);
    syncABCfg.bCount = 1;
    syncABCfg.cCount = 1;
    syncABCfg.srcBIdx = 0;
    syncABCfg.srcCIdx = 0;
    syncABCfg.dstBIdx = 0;
    syncABCfg.dstCIdx = 0;
    syncABCfg.srcAddress = (uint32_t)zoomProcObj->hwaMemBankAddr[2];
    syncABCfg.destAddress= (uint32_t)zoomProcObj->zoomDftOut;

    chainingCfg.chainingChan = pHwConfig->edmaOutCfg.dataOutZoom.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled = false;
        
    
    isIntermediateTransferCompletionEnabled = false;
    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = zoomProc_EDMA_transferCompletionCallbackFxn;

    errorCode = DPEDMA_configSyncAB(handle,
            &pHwConfig->edmaOutCfg.dataOutZoom,
            &chainingCfg,
            &syncABCfg,
            true,    /* isEventTriggered */
            isIntermediateTransferCompletionEnabled,
            isTransferCompletionEnabled,
            transferCompletionCallbackFxn,
            (void*)zoomProcObj,
            pHwConfig->intrObj);

    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      EDMA configuration for zoomProc data in when EDMA is used to copy data from 
 *  ADCBuf to HWA memory
 *
 *  @param[in]  zoomProcObj              Pointer to zoomProc object handle
 *  @param[in]  pHwConfig                Pointer to zoomProc hardware resources
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_ConfigEDMA_DataIn
(
    ZoomProcObj         *zoomProcObj,
    DPU_ZoomProc_HW_Resources *pHwConfig
)
{
    int32_t             errorCode = SystemP_SUCCESS;
    bool                retVal;
    EDMA_Handle         handle = pHwConfig->edmaHandle;
    uint16_t            bytePerRxChan;
    DPU_ZoomProc_StaticConfig   *DPParams= &zoomProcObj->params;
    uint32_t            dmaCh, tcc, param, chType;
    bool                isEventTriggered;
    uint32_t            baseAddr, regionId;
    
    uint32_t            hwaInpBuff[2];
    EDMACCPaRAMEntry    shadowParam;
    uint32_t            linkChId0;

    
    baseAddr = EDMA_getBaseAddr(handle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(handle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);
    
    if (zoomProcObj->hwaCfg.dataInputMode == DPU_ZoomProc_InputMode_ISOLATED)
    {
        /* EDMA IN for ADC data */
        isEventTriggered = true;
    }
    else
    {
        isEventTriggered = false;
    }

    /* Get zoomProc Configuration */
    //handle = zoomProcObj->edmaHandle;
    chType = EDMA_CHANNEL_TYPE_DMA;
    dmaCh  = pHwConfig->edmaInCfg.dataIn.channel;
    param  = pHwConfig->edmaInCfg.dataIn.channel;
    tcc    = pHwConfig->edmaInCfg.dataInSignature.channel;
    isEventTriggered = true;

    /* Config Dummy channel and paramset */
    errorCode = DPEDMA_configDummyChannel(handle, chType, &dmaCh, &tcc, &param);

    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }

    //real ADC samples only supported
    bytePerRxChan = DPParams->ADCBufData.dataProperty.numAdcSamples * (DPParams->numVirtualAntennas/DPParams->numTxAntennas) * sizeof(uint16_t);
    
    hwaInpBuff[0] = zoomProcObj->hwaMemBankAddr[0];
    hwaInpBuff[1] = zoomProcObj->hwaMemBankAddr[1];

    /* Get LinkChan from configuraiton */
    linkChId0 = pHwConfig->edmaInCfg.dataIn.channelShadow;
    allocateEDMAShadowChannel(handle, &linkChId0);
    
    /* Program Shadow Param Sets */
    EDMACCPaRAMEntry_init(&shadowParam);

    memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
    shadowParam.srcAddr = (uint32_t) zoomProcObj->ADCdataBuf;
    shadowParam.destAddr = (uint32_t) hwaInpBuff[0];
    shadowParam.aCnt = bytePerRxChan;
    shadowParam.bCnt = 1;
    shadowParam.cCnt = 1;
    shadowParam.bCntReload = shadowParam.bCnt;
    shadowParam.srcBIdx = 0;
    shadowParam.destBIdx = 0;
    shadowParam.srcCIdx = 0;
    shadowParam.destCIdx = 0;

    shadowParam.opt          |=
        (EDMA_OPT_TCCHEN_MASK |
            ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
            ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));

    EDMASetPaRAM(baseAddr, pHwConfig->edmaInCfg.dataIn.channelShadow, 
                    &shadowParam);
        

    /* Link shadow Param sets */
    if ((errorCode = DPEDMA_linkParamSets(handle,
                                          param,
                                          linkChId0)) != SystemP_SUCCESS)
    {
        goto exit;
    }
    if ((errorCode = DPEDMA_linkParamSets(handle,
                                          linkChId0,
                                          linkChId0)) != SystemP_SUCCESS)
    {
        goto exit;
    }
    
    /* Bring in the first shadow param set  */
    retVal = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);
    
    if(isEventTriggered)
    {
        retVal = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_EVENT);
    }

    if (retVal != TRUE)
    {
        errorCode = DPU_ZOOMPROC_EINTERNAL;
        goto exit;
    }

    /*************************************************/
    /* Generate Hot Signature to trigger paramset   */
    /*************************************************/
    errorCode = DPEDMAHWA_configTwoHotSignature(handle,
                                                &pHwConfig->edmaInCfg.dataInSignature,
                                                zoomProcObj->initParms.hwaHandle,
                                                zoomProcObj->dataInTrigger,
                                                zoomProcObj->dataInTrigger,
                                                false);

    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      zoomProc configuration
 *
 *  @param[in]  zoomProcObj                 Pointer to zoomProc object
 *  @param[in]  pHwConfig                    Pointer to zoomProc hardware resources
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_ConifgureHwaEdma
(
    ZoomProcObj          *zoomProcObj,
    DPU_ZoomProc_HW_Resources *pHwConfig
)
{
    HWA_Handle          hwaHandle;
    int32_t             retVal = 0;
    uint8_t             destChan;
    uint8_t             edmaOutChan;

    hwaHandle = zoomProcObj->initParms.hwaHandle;

    edmaOutChan = pHwConfig->edmaOutCfg.dataOutZoom.channel;

    /* Get HWA destination channel id for ping*/
    retVal = HWA_getDMAChanIndex(hwaHandle, edmaOutChan, &destChan);
    if (retVal != 0)
    {
        goto exit;
    }
    
    /* Data input EDMA configuration */
    if (zoomProcObj->hwaCfg.dataInputMode == DPU_ZoomProc_InputMode_ISOLATED)
    {
        /* EDMA IN for ADC data */
        zoomProc_ConfigEDMA_DataIn(zoomProcObj, pHwConfig);
    }
    
    /* Zoom FFT configuration in HWA */
    retVal = zoomProc_ConfigHWA(zoomProcObj,
                                destChan);
    if(retVal < 0)
    {
        goto exit;
    }


    /* Data output EDMA configuration */
    retVal = zoomProc_ConfigEDMA_DataOut(zoomProcObj,
                                             pHwConfig);
    if (retVal < 0)
    {
        goto exit;
    }

exit:
    return (retVal);
}


/**
 *  @b Description
 *  @n
 *      Internal function to parse zoomProc configuration and save in internal zoomProc object
 *
 *  @param[in]  zoomProcObj              Pointer to zoomProc object
 *  @param[in]  pConfigIn                 Pointer to zoomProc configuration structure
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_ParseConfig
(
    ZoomProcObj         *zoomProcObj,
    DPU_ZoomProc_Config  *pConfigIn
)
{
    int32_t                 retVal = 0;
    DPU_ZoomProc_StaticConfig   *pStaticCfg, *params;

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    params    = &zoomProcObj->params;

    /* Save datapath parameters */
    memcpy((void *)params, (void *)pStaticCfg, sizeof(DPU_ZoomProc_StaticConfig));

    /* Save buffers */
    zoomProcObj->ADCdataBuf        = (cmplx16ImRe_t *)pStaticCfg->ADCBufData.data;
    zoomProcObj->isTestMode      = pStaticCfg->isTestMode;
    zoomProcObj->zoomDftOut      = (cmplx32ImRe_t *)pConfigIn->hwRes.zoomDftOut;
    zoomProcObj->zoomInRbin      = (uint16_t *)pConfigIn->hwRes.zoomInRbin;
    
    /* Prepare internal hardware resources = trigger source matches its  paramset index */
    zoomProcObj->edmaHandle      = pConfigIn->hwRes.edmaHandle;
    zoomProcObj->dataInTrigger   = pConfigIn->hwRes.hwaCfg.dmaTrigSrcChan;
    zoomProcObj->dataOutTrigger  = 0;
    zoomProcObj->dataOutZoomChan = pConfigIn->hwRes.edmaOutCfg.dataOutZoom.channel;

    /* Save hardware resources that will be used at runtime */
    memcpy((void *)&zoomProcObj->hwaCfg, (void *)&pConfigIn->hwRes.hwaCfg, sizeof(DPU_ZoomProc_HwaConfig));

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA/EDMA to perform zoom FFT
 *
 *  @param[in]  zoomProcObj              Pointer to zoomProc object
 *  @param[in]  pHwConfig                 Pointer to zoomProc hardware resources
 *
 *  \ingroup    DPU_ZOOMPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t zoomProc_HardwareConfig
(
    ZoomProcObj         *zoomProcObj,
    DPU_ZoomProc_HW_Resources *pHwConfig
)
{
    int32_t                 retVal = 0;


    retVal =zoomProc_ConifgureHwaEdma(zoomProcObj, pHwConfig);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**************************************************************************
 ************************ZoomProc External APIs **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is zoomProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              Pointer to DPU init parameters
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_ZOOMPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid zoomProc handle
 *  @retval
 *      Error       - NULL
 */
DPU_ZoomProc_Handle DPU_ZoomProc_init
(
    DPU_ZoomProc_InitParams     *initParams,
    int32_t*                        errCode
)
{
    ZoomProcObj     *zoomProcObj = NULL;
    // SemaphoreP_Params   semParams;
    HWA_MemInfo         hwaMemInfo;
    uint8_t             index;
    int32_t             status = SystemP_SUCCESS;

    *errCode = 0;

    if( (initParams == NULL) ||
       (initParams->hwaHandle == NULL) )
    {
        *errCode = DPU_ZOOMPROC_EINVAL;
        goto exit;
    }

    /* Allocate Memory for zoomProc */
    zoomProcObj = (ZoomProcObj*)&gZoomProcHeapMem;
    if(zoomProcObj == NULL)
    {
        *errCode = DPU_ZOOMPROC_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)zoomProcObj, 0, sizeof(ZoomProcObj));

    memcpy((void *)&zoomProcObj->initParms, initParams, sizeof(DPU_ZoomProc_InitParams));

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(initParams->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {
        goto exit;
    }

    for (index = 0; index < hwaMemInfo.numBanks; index++)
    {
        zoomProcObj->hwaMemBankAddr[index] = hwaMemInfo.baseAddress + index * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    status = SemaphoreP_constructBinary(&zoomProcObj->edmaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_ZOOMPROC_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done */
    status = SemaphoreP_constructBinary(&zoomProcObj->hwaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_ZOOMPROC_ESEMA;
        goto exit;
    }

exit:
    if(*errCode < 0)
    {
        zoomProcObj = (DPU_ZoomProc_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((DPU_ZoomProc_Handle)zoomProcObj);

}


/**
 *  @b Description
 *  @n
 *      The function is zoomProc DPU config function. It saves buffer pointer and configurations 
 *  including system resources and configures HWA and EDMA for runtime zoom processing.
 *  
 *  @pre    DPU_ZoomProc_init() has been called
 *
 *  @param[in]  handle                  zoomProc DPU handle
 *  @param[in]  pConfigIn               Pointer to zoomProc configuration data structure
 *
 *  \ingroup    DPU_ZOOMPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_ZoomProc_config
(
    DPU_ZoomProc_Handle  handle,
    DPU_ZoomProc_Config  *pConfigIn
)
{
    ZoomProcObj                 *zoomProcObj;
    DPU_ZoomProc_StaticConfig   *pStaticCfg;
    HWA_Handle                      hwaHandle;
    int32_t                         retVal = 0;

    zoomProcObj = (ZoomProcObj *)handle;
    if(zoomProcObj == NULL)
    {
        retVal = DPU_ZOOMPROC_EINVAL;
        goto exit;
    }

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    hwaHandle = zoomProcObj->initParms.hwaHandle;

#if DEBUG_CHECK_PARAMS
    {
        /* Validate params */
        if(!pConfigIn ||
          !(pConfigIn->hwRes.edmaHandle)
          )
        {
            retVal = DPU_ZOOMPROC_EINVAL;
            goto exit;
        }
    }
    /* Parameter check: validate Adc data interface configuration
        Support:
            - 1 chirp per chirpEvent
            - Complex 16bit ADC data in IMRE format
     */
    if( ((pStaticCfg->ADCBufData.dataProperty.dataFmt != DPIF_DATAFORMAT_COMPLEX16_IMRE) && (pStaticCfg->ADCBufData.dataProperty.dataFmt != DPIF_DATAFORMAT_REAL16)) ||      //Support real ADC
       (pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent != 1U) )
    {
        retVal = DPU_ZOOMPROC_EINTERNAL;
        goto exit;
    }

    /* Parameter check: radarcube buffer Size */
    if ( pConfigIn->hwRes.zoomDftOutSize != (pStaticCfg->numDftBins* sizeof(cmplx32ImRe_t)) )
    {
        retVal = DPU_ZOOMPROC_EINTERNAL;
        goto exit;
    }

#endif

    retVal = zoomProc_ParseConfig(zoomProcObj, pConfigIn);
    if (retVal < 0)
    {
        goto exit;
    }

    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Reset the internal state of the HWA */
    retVal = HWA_reset(hwaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Clear stats */
    zoomProcObj->numProcess = 0U;

    /* Initial configuration of zoomProc */
    retVal = zoomProc_HardwareConfig(zoomProcObj, &pConfigIn->hwRes);

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is zoomProc DPU process function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @pre    DPU_ZoomProc_init() has been called
 *
 *  @param[in]  handle                  zoomProc DPU handle
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_ZOOMPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_ZoomProc_process
(
    DPU_ZoomProc_Handle     handle,
    DPU_ZoomProc_OutParams  *outParams
)
{
    ZoomProcObj     *zoomProcObj;
    int32_t         retVal = 0;
    int32_t         coarseIndStart, i, max_ind;
    cmplx32ImRe_t   *interpOutsAddr;
    float           power, maxP;
    float           fpower[3], interpIndx, maxIndexFine, fineFreqEst;
    uint32_t        baseAddr, regionId;

    zoomProcObj = (ZoomProcObj *)handle;
    if ((zoomProcObj == NULL) ||
        (outParams == NULL))
    {
        retVal = DPU_ZOOMPROC_EINVAL;
        goto exit;
    }

    baseAddr = EDMA_getBaseAddr(zoomProcObj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(zoomProcObj->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    interpOutsAddr = zoomProcObj->zoomDftOut;

    /* Set the DFT Start freq based on the 1D FFT peak loc */
    if (zoomProcObj->hwaCfg.dataInputMode != DPU_ZoomProc_InputMode_ISOLATED)
    {
        gHwaParams.complexMultiply.cmpMulArgs.dft.startFreq = *zoomProcObj->zoomInRbin;
        
        retVal = HWA_configParamSet(zoomProcObj->initParms.hwaHandle,
                                    zoomProcObj->hwaCfg.paramSetStartIdx,
                                    &gHwaParams,NULL);
        if (retVal != 0)
        {
            goto exit;
        }
    }
    
    retVal = zoomProc_TriggerHWA(zoomProcObj);
    if (retVal != 0)
    {
        goto exit;
    }

    if (zoomProcObj->hwaCfg.dataInputMode == DPU_ZoomProc_InputMode_ISOLATED && zoomProcObj->isTestMode)
    {
        if (EDMAEnableTransferRegion(baseAddr, regionId, EDMA_APPSS_TPCC_B_EVT_CHIRP_AVAIL_IRQ, EDMA_TRIG_MODE_MANUAL) != 1)
        {
            goto exit;
        }
    }

    /* Set inProgress state */
    zoomProcObj->inProgress = true;
    outParams->endOfChirp = false;
    

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    /* wait for the all paramSets done interrupt */
    SemaphoreP_pend(&zoomProcObj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /**********************************************/
    /* WAIT FOR EDMA INTERRUPT                    */
    /**********************************************/
    SemaphoreP_pend(&zoomProcObj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /* Zoom FFT is done, disable Done interrupt */
    HWA_disableDoneInterrupt(zoomProcObj->initParms.hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(zoomProcObj->initParms.hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    zoomProcObj->numProcess++;

    /* Following stats is not available for zoomProc */
    //outParams->stats.processingTime = 0;
    //outParams->stats.waitTime= 0;
    
    outParams->endOfChirp = true;

    if (*zoomProcObj->params.peakLoc < zoomProcObj->params.zoomSamplesOneSide)
    {
        coarseIndStart = 0;
    }
    else if (*zoomProcObj->params.peakLoc > (zoomProcObj->params.numRangeBins - zoomProcObj->params.zoomSamplesOneSide))
    {
        coarseIndStart  =   zoomProcObj->params.numRangeBins - 2 * zoomProcObj->params.zoomSamplesOneSide;
    }
    else
    {
        coarseIndStart  =   *zoomProcObj->params.peakLoc - zoomProcObj->params.zoomSamplesOneSide;
    }

    /* Find peak mag and index in DFT output */
    maxP    =   0.f;
    for (i = 0; i < 2 * zoomProcObj->params.zoomSamplesOneSide * zoomProcObj->params.interpFactor; i++)
    {
        power   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;
        if (power > maxP)
        {
            maxP    =   power;
            max_ind =   i;
        }
    }

    i           =   max_ind - 1;
    fpower[0]   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;
    i           =   max_ind;
    fpower[1]   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;
    i           =   max_ind + 1;
    fpower[2]   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;

    interpIndx  =   0.5f * (fpower[0] - fpower[2])/(fpower[0] + fpower[2] -2.f * fpower[1]);

    maxIndexFine    =   (float)(zoomProcObj->params.interpFactor * coarseIndStart + max_ind + 1) + interpIndx;
    fineFreqEst     =   maxIndexFine * zoomProcObj->params.freqRes/zoomProcObj->params.interpFactor;
    
    outParams->zoomRangeMeas   =   fineFreqEst * zoomProcObj->params.zoomRngFactor;

    /* Clear inProgress state */
    zoomProcObj->inProgress = false;

exit:

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is zoomProc DPU deinit function. It frees the resources used for the DPU.
 *
 *  @pre    DPU_ZoomProc_init() has been called
 *
 *  @param[in]  handle           zoomProc DPU handle
 *
 *  \ingroup    DPU_ZOOMPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_ZoomProc_deinit
(
    DPU_ZoomProc_Handle     handle
)
{
    ZoomProcObj     *zoomProcObj;
    int32_t             retVal = 0;

    /* Sanity Check */
    zoomProcObj = (ZoomProcObj *)handle;
    if(zoomProcObj == NULL)
    {
        retVal = DPU_ZOOMPROC_EINVAL;
        goto exit;
    }

    /* Delete Semaphores */
    SemaphoreP_destruct(&zoomProcObj->edmaDoneSemaHandle);
    SemaphoreP_destruct(&zoomProcObj->hwaDoneSemaHandle);

exit:

    return (retVal);
}
