/*
 *  Copyright (C) 2025 Texas Instruments Incorporated 
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
 *  This example performs HWA driver test using software trigger mode.
 *
 * The HWA source memory is filled with sample data and destination memory
 * contains the final CFAR output.
 *
 * The PaRAM set is initialized with proper configuration and assigned to a
 * EDMA channel for transfer.
 *
 * The final HWA done interrupt is enabled, to transfer control to the 
 * processor/ARM.
 *
 * After the CFAR is completed, data validation is performed by comparing
 * source and destination memory, against the reference output. If the 
 * equality test is successful, the test was successful.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/edma.h>
#include <drivers/hwa.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include "hwa_fft.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Value for A count*/
#define EDMA_TEST_A_COUNT           (0x200)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (0x1)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (0x1)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

#define PI_ 3.14159265
/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */

#define hwa_src                     CSL_APP_HWA_DMA0_RAM_BANK0_BASE
#define hwa_dst                     CSL_APP_HWA_DMA0_RAM_BANK3_BASE

#define DATA_ADDRESS                CSL_APP_RAM_U_BASE

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))

/*
 * ISR Callback
 */
static void HWAFFT_doneCallback(void *arg);

static uint32_t HWAFFT_log2Approx(uint32_t x);

HWA_Handle              gHwaHandle;
struct HWAFFT_Object    gHwaFftObject;
Edma_IntrObject         intrObj, intrObjOut;

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTestDoneSem, gEdmaTestDoneSemOut;

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);

/* Test: Reference FFT output */
uint16_t testinput[32]=
{
    2564, 2709 ,2448,2502,2336,4216,2522,2071,2658,2347,2846,4216,2476,2679,2504,4216,2568,2460,2641,2386,2770,2118,2148,2346,2589,2509,2589,2445,2225,2610,2218,2610
};
uint16_t testoutput[12]=
{
    2386,0,4098,0,2386,0,4101,0,2347,0,4103,0
};

int32_t checkTestResult(uint16_t *testOut, uint16_t *refOut, uint32_t numSamples)
{
    int32_t status = SystemP_SUCCESS;
    int32_t i;

    for (i = 0; i < 4*numSamples; i++) 
    {
        if (testOut[i] != refOut[i])
        {
            status = SystemP_FAILURE;
            break;
        }
    }


    return status;
}


void hwa_cfar_os(void *args)
{
    uint32_t            baseAddr, regionId;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            loopCnt = 0;
    uint16_t             *src, *dst, *hwaDst;
    HWA_ParamConfig     *paramCfg;
    HWA_CommonConfig    *commonCfg;
    EDMACCPaRAMEntry   *rdEdmaPrms;
    EDMACCPaRAMEntry   *wrEdmaPrms;
    int32_t             *rdMemBankAddr;
    int32_t             *wrMemBankAddr;
    uint32_t            numSamples = 32U;
    struct              HWAFFT_ResObject    *resObj;
    struct              HWAFFT_Object   *fftObj = &gHwaFftObject;
    uint32_t                peakCnt=0;

    
    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    commonCfg = &fftObj->commonCfg;

    DebugP_log("HWA CFAR-OS Test Started \r\n");

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /*
     * Resource Init
     */
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_FFT];
    resObj->paramIdx        = 0U;
    
    resObj->rdDmaCh = (1U);
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &resObj->rdDmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    
    resObj->wrDmaCh = (2U);
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &resObj->wrDmaCh);
    DebugP_assert(status == SystemP_SUCCESS);

    resObj->rdTcc = (1U);
    status = EDMA_allocTcc(gEdmaHandle[0], &resObj->rdTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    
    resObj->wrTcc = (2U);
    status = EDMA_allocTcc(gEdmaHandle[0], &resObj->wrTcc);
    DebugP_assert(status == SystemP_SUCCESS);

    resObj->rdParamId = (1U);
    status = EDMA_allocParam(gEdmaHandle[0], &resObj->rdParamId);
    DebugP_assert(status == SystemP_SUCCESS);
    
    resObj->wrParamId = (2U);
    status = EDMA_allocParam(gEdmaHandle[0], &resObj->wrParamId);
    DebugP_assert(status == SystemP_SUCCESS);

    resObj->rdMemBankAddr   = hwa_src;
    resObj->wrMemBankAddr   = hwa_dst;
    
    ///*****   APPSS RAM  *****///
    src = (uint16_t *)(DATA_ADDRESS + 0x20000);
    dst = (uint16_t *)(DATA_ADDRESS + 0x21000);
    hwaDst = (uint16_t *)(CSL_APP_HWA_DMA0_RAM_BANK3_BASE);
    
    /* First half of FFT input sequence: sine wave */
    for(loopCnt = 0U; loopCnt < numSamples; loopCnt++)
    {
       src[loopCnt] = testinput[loopCnt];
    }

    /* Open HWA driver */
    gHwaHandle = HWA_open(0, NULL, &status);
    if(gHwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open HWA instance. Error: %d\n", status);
        DebugP_assert(gHwaHandle == NULL);
    }
    /*
     * State variables
     */
    fftObj->hwaHandle       = gHwaHandle;
    fftObj->edmaBaseAddr    = baseAddr;
    fftObj->edmaRegionId    = regionId;
    fftObj->fixedPointBuf   = (int32_t *)(DATA_ADDRESS + 0x20000);
    
    /* Request channel */
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         resObj->rdDmaCh, resObj->rdTcc, resObj->rdParamId, EDMA_TEST_EVT_QUEUE_NO);

        
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         resObj->wrDmaCh, resObj->wrTcc, resObj->wrParamId, EDMA_TEST_EVT_QUEUE_NO);

    /* Program Read Param Set */
    rdEdmaPrms = &resObj->rdDmaParam;
    EDMACCPaRAMEntry_init(rdEdmaPrms);
    rdEdmaPrms->linkAddr      = (0xFFFFU);
    
    /* Program Write Param Set */
    wrEdmaPrms = &resObj->wrDmaParam;
    EDMACCPaRAMEntry_init(wrEdmaPrms);
    wrEdmaPrms->linkAddr      = (0xFFFFU);

    /* Init param set */
    paramCfg = &resObj->paramCfg;
    memset(paramCfg , 0, sizeof(*paramCfg));
    paramCfg->triggerMode = HWA_TRIG_MODE_SOFTWARE;
    paramCfg->accelMode = HWA_ACCELMODE_CFAR;
    paramCfg->source.srcAcnt = 16 - 1U;
    paramCfg->source.srcAIdx = 2*sizeof(int16_t);
    paramCfg->source.srcBIdx = 2;
    paramCfg->source.srcBcnt = 1U;  
    paramCfg->source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    paramCfg->source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObj->rdMemBankAddr);
    paramCfg->source.srcScale = 0x8;
    paramCfg->source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    
    paramCfg->dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObj->wrMemBankAddr);
    paramCfg->dest.dstAcnt = 2048 - 1U;
    paramCfg->dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    paramCfg->dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    paramCfg->dest.dstAIdx = 8; 
    paramCfg->dest.dstBIdx = 0U;    /* dont care */
    paramCfg->dest.dstScale = 8U;

    paramCfg->accelModeArgs.cfarMode.peakGroupEn = 0;
    paramCfg->accelModeArgs.cfarMode.operMode = HWA_CFAR_OPER_MODE_LOG_INPUT_REAL;
    paramCfg->accelModeArgs.cfarMode.cyclicModeEn = 0;
    paramCfg->accelModeArgs.cfarMode.numGuardCells = 1;
    paramCfg->accelModeArgs.cfarMode.nAvgMode = HWA_NOISE_AVG_MODE_CFAR_OS;
    paramCfg->accelModeArgs.cfarMode.cfarOsKvalue = 0x3;
    paramCfg->accelModeArgs.cfarMode.numNoiseSamplesRight = 4;
    paramCfg->accelModeArgs.cfarMode.numNoiseSamplesLeft =  4;
    paramCfg->accelModeArgs.cfarMode.outputMode = 0x2;

    #if (abs(hwa_src - hwa_dst) < CSL_APP_HWA_BANK_SIZE)
        /* MEMACCESS config params warning */
        CUSTOM_WARNING("HWA Param src and dst addresses shall not be from same memory bank");
    #endif
    
    status += HWA_configParamSet(fftObj->hwaHandle, resObj->paramIdx, paramCfg, NULL);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("Error: HWA_configParamSet failed with error: %d!!\r\n", status);
    }
    
    if(SystemP_SUCCESS == status)
    {
        /* Init Common Params */
        memset(commonCfg, 0, sizeof(*commonCfg));
        commonCfg->configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_CFARTHRESHOLDSCALE;
        commonCfg->cfarConfig.cfarThresholdScale = 800;
        commonCfg->paramStartIdx = 0U;  /* Will be overwritten */
        commonCfg->paramStopIdx = 0U;   /* Will be overwritten */
        commonCfg->numLoops = 1U;
        status = HWA_configCommon(fftObj->hwaHandle, commonCfg);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error: HWA_configCommon failed with error: %d\r\n", status);
        }
    }

    #if 1
    if(SystemP_SUCCESS == status)
    {
        /* Enable done interrupt */
        status = SemaphoreP_constructBinary(&fftObj->doneSem, 0);
        DebugP_assert(status == SystemP_SUCCESS);
        status = HWA_enableDoneInterrupt(fftObj->hwaHandle, HWAFFT_doneCallback, fftObj);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error: HWA_enableDoneInterrupt failed with error: %d\r\n", status);
        }
    }
    #endif

    DebugP_assert(status == SystemP_SUCCESS);
    
    rdMemBankAddr = (int32_t *) resObj->rdMemBankAddr;
    wrMemBankAddr = (int32_t *) resObj->wrMemBankAddr;
    
    
    /* Initiate 2D DMA */
    rdEdmaPrms->srcAddr   = (uint32_t) (src);
    rdEdmaPrms->destAddr  = (uint32_t) (rdMemBankAddr);
    rdEdmaPrms->aCnt      = (uint16_t) (numSamples * sizeof(int16_t));
    rdEdmaPrms->bCnt      = (uint16_t) EDMA_TEST_B_COUNT;
    rdEdmaPrms->cCnt      = (uint16_t) EDMA_TEST_C_COUNT;
    rdEdmaPrms->srcBIdx   = (int16_t) 0;
    rdEdmaPrms->destBIdx  = (int16_t) 0;
    rdEdmaPrms->opt       = 0x301000U;
    
    
    wrEdmaPrms->srcAddr   = (uint32_t) (wrMemBankAddr);
    wrEdmaPrms->destAddr  = (uint32_t) (dst);
    wrEdmaPrms->aCnt      = (uint16_t) (numSamples * 2*sizeof(int16_t)); 
    wrEdmaPrms->bCnt      = (uint16_t) EDMA_TEST_B_COUNT;
    wrEdmaPrms->cCnt      = (uint16_t) EDMA_TEST_C_COUNT;
    wrEdmaPrms->srcBIdx   = (int16_t) 0;
    wrEdmaPrms->destBIdx  = (int16_t) 0;
    wrEdmaPrms->opt       = 0x302000U;
    
    EDMASetPaRAM(baseAddr, resObj->rdParamId, rdEdmaPrms);
    EDMASetPaRAM(baseAddr, resObj->wrParamId, wrEdmaPrms);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSemOut, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupts */
    intrObj.tccNum = resObj->rdTcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    intrObjOut.tccNum = resObj->wrTcc;
    intrObjOut.cbFxn  = &EDMA_regionIsrFxn;
    intrObjOut.appData = (void *) &gEdmaTestDoneSemOut;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObjOut);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Enable HWA */
    status += HWA_enable(fftObj->hwaHandle, 1U);

    /*************************************/
    
    /* DMA Trigger and wait */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMAEnableTransferRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, resObj->rdDmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    status += HWA_setSoftwareTrigger(fftObj->hwaHandle);
    SemaphoreP_pend(&fftObj->doneSem, SystemP_WAIT_FOREVER);
    
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMAEnableTransferRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, resObj->wrDmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSemOut, SystemP_WAIT_FOREVER);
    }

    /* Read the output PEAK COUNT */
    peakCnt = HW_RD_REG32(0x550100E8);
    peakCnt = (peakCnt & (0xFFFU));
    /* Disable HWA */
    status += HWA_enable(fftObj->hwaHandle, 0U);
    DebugP_assert(SystemP_SUCCESS == status);

    status += HWA_close(gHwaHandle);

    status = EDMA_unregisterIntr(gEdmaHandle[0], &intrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);
    
    status = EDMA_unregisterIntr(gEdmaHandle[0], &intrObjOut);
    SemaphoreP_destruct(&gEdmaTestDoneSemOut);
    
    DebugP_assert(status == SystemP_SUCCESS);


    /* Free channel */
    EDMAFreeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        resObj->rdDmaCh, EDMA_TRIG_MODE_MANUAL, resObj->rdTcc, EDMA_TEST_EVT_QUEUE_NO);
    EDMAFreeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        resObj->wrDmaCh, EDMA_TRIG_MODE_MANUAL, resObj->wrTcc, EDMA_TEST_EVT_QUEUE_NO);


    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Test failed!!\r\n");
        goto exit;
    }

    status = checkTestResult(hwaDst, testoutput, peakCnt);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[HWA-CFAR-OS] Test Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

exit:
    Board_driversClose();
    Drivers_close();
    return;
}

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}

/*
 * ISR Callback
 */
static void HWAFFT_doneCallback(void *arg)
{
    struct HWAFFT_Object   *fftObj;

    if(arg != NULL)
    {
        fftObj = (struct HWAFFT_Object *)arg;
        SemaphoreP_post(&fftObj->doneSem);
    }

    return;
}

static uint32_t HWAFFT_log2Approx(uint32_t x)
{
    uint32_t idx, detectFlag = 0U;

    if(x < 2U)
    {
        idx = 0U;
    }
    else
    {
        idx = 32U;
        while((detectFlag == 0U) || (idx == 0U))
        {
            if(x & 0x80000000U)
            {
                detectFlag = 1;
            }
            x <<= 1U;
            idx--;
        }
    }

    return (idx);
}