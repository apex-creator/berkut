/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 * contains the final compression-decompression output.
 *
 * The PaRAM set is initialized with proper configuration and assigned to a
 * EDMA channel for transfer.
 *
 * The final HWA done interrupt is enabled, to transfer control to the 
 * processor/ARM.
 *
 * After the compression-decompression is completed, data validation is performed
 * by comparing source and destination memory. If the equality test is successful,
 * the test was successful.
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
#define EDMA_TEST_A_COUNT           (1024U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (1U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

#define PI_                         3.14159265
#define HWA_CMP_RATIO_BW 		    (8U)     /*!<  fractional bitwidth of the 'compression ratio'.  */
#define HWA_CMP_50P_RATIO           (2U << (HWA_CMP_RATIO_BW-2))     /*!<  'Compression Ratio' corresponding to 50 %.  */

/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define hwa_src                     CSL_APP_HWA_DMA0_RAM_BANK0_BASE
#define hwa_dst                     CSL_APP_HWA_DMA0_RAM_BANK1_BASE
/* Compression output dest bank used as input for decompression */ 
#define hwa_src_dcmp                CSL_APP_HWA_DMA0_RAM_BANK1_BASE
#define hwa_dst_dcmp                CSL_APP_HWA_DMA0_RAM_BANK3_BASE

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))
/*
 * ISR Callback
 */
static void HWA_Test_ParamSetISR_Callback(uint32_t paramSet, void *arg);
static void HWAFFT_doneCallback(void *arg);

HWA_Handle              gHwaHandle;
struct HWAFFT_Object    gHwaFftObject;
Edma_IntrObject         intrObj, intrObjOut, intrObjOutDcmp;
HWA_InterruptConfig     paramISRConfig;

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTestDoneSem, gEdmaTestDoneSemOut, gEdmaTestDoneSemOutDcmp;

void  cfgEGEParamListDopplerProc(uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth);

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);

void hwa_compression(void *args)
{
    uint32_t            baseAddr, regionId;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            loopCnt = 0;
    uint16_t            *dst;
    int16_t             *src, *dstDcmp;
    HWA_ParamConfig     *paramCfg, *paramCfgDcmp;
    HWA_CommonConfig    *commonCfg;
    DSSHWACCRegs        *ctrlBaseAddr;
    DSSHWACCPARAMRegs   *paramBaseAddr;
    EDMACCPaRAMEntry   *rdEdmaPrms;
    EDMACCPaRAMEntry   *wrEdmaPrms, *wrEdmaPrmsDcmp;
    uint16_t            *wrMemBankAddr;
    int16_t             *rdMemBankAddr, *wrMemBankAddrDcmp;
    uint32_t            numSamples = 512U; /* Will be overwritten */
    struct              HWAFFT_ResObject    *resObj, *resObjDcmp;
    struct              HWAFFT_Object   *fftObj = &gHwaFftObject;
    float               f = 1210937.5; //in Hz
    
    /* Compression configurations */
    uint16_t            compressionRatio = (uint16_t) (((float)0.5) * (1 << HWA_CMP_RATIO_BW));
    uint16_t            numSamplesPerBlockIn = 8;
    uint16_t            numSamplesPerBlockOut = (uint16_t) ((numSamplesPerBlockIn * (uint32_t)compressionRatio) >> HWA_CMP_RATIO_BW);
    uint16_t            numBlocks = numSamples/numSamplesPerBlockIn;
    
    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    commonCfg = &fftObj->commonCfg;

    DebugP_log("HWA Test Started \r\n");

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /*
     * Resource Init for compression
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

    /*
     * Resource Init for Decompression
     */

    resObjDcmp = &fftObj->resObj[HWAFFT_RES_IDX_IFFT];
    resObjDcmp->paramIdx        = 1U;
    
    resObjDcmp->wrDmaCh = (4U);
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &resObjDcmp->wrDmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    
    resObjDcmp->wrTcc = (4U);
    status = EDMA_allocTcc(gEdmaHandle[0], &resObjDcmp->wrTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    
    resObjDcmp->wrParamId = (4U);
    status = EDMA_allocParam(gEdmaHandle[0], &resObjDcmp->wrParamId);
    DebugP_assert(status == SystemP_SUCCESS);

    resObjDcmp->rdMemBankAddr   = hwa_dst;
    resObjDcmp->wrMemBankAddr   = hwa_dst_dcmp;
    
    ///*****   APPSS RAM  *****///
    src = (int16_t *)(CSL_APP_RAM_U_BASE + 0x20000);
    dst = (uint16_t *)(CSL_APP_RAM_U_BASE + 0x21000);
    dstDcmp = (int16_t *)(CSL_APP_RAM_U_BASE + 0x22000);
    
    for(loopCnt = 0U; loopCnt < numSamples; loopCnt++)
    {
       src[loopCnt] = ((1 << 8)*sin(2*PI_*(f/10000000)*loopCnt));
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
    fftObj->fixedPointBuf   = (int32_t *)(CSL_APP_RAM_U_BASE + 0x20000);
    
    /* Request channel */
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         resObj->rdDmaCh, resObj->rdTcc, resObj->rdParamId, EDMA_TEST_EVT_QUEUE_NO);

    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         resObj->wrDmaCh, resObj->wrTcc, resObj->wrParamId, EDMA_TEST_EVT_QUEUE_NO);
        
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         resObjDcmp->wrDmaCh, resObjDcmp->wrTcc, resObjDcmp->wrParamId, EDMA_TEST_EVT_QUEUE_NO);

    /* Program Read Param Set */
    rdEdmaPrms = &resObj->rdDmaParam;
    EDMACCPaRAMEntry_init(rdEdmaPrms);
    rdEdmaPrms->linkAddr      = (0xFFFFU);
    
    /* Program Write Param Set */
    wrEdmaPrms = &resObj->wrDmaParam;
    EDMACCPaRAMEntry_init(wrEdmaPrms);
    wrEdmaPrms->linkAddr      = (0xFFFFU);
    
    /* Program Write Param Set */
    wrEdmaPrmsDcmp = &resObjDcmp->wrDmaParam;
    EDMACCPaRAMEntry_init(wrEdmaPrmsDcmp);
    wrEdmaPrmsDcmp->linkAddr      = (0xFFFFU);

    /* Init Compression param set */
    paramCfg = &resObj->paramCfg;
    memset(paramCfg , 0, sizeof(*paramCfg));
    paramCfg->triggerMode = HWA_TRIG_MODE_SOFTWARE;
    paramCfg->accelMode = HWA_ACCELMODE_COMPRESS;
    
    paramCfg->source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObj->rdMemBankAddr);
    paramCfg->source.srcSign = HWA_SAMPLES_SIGNED;
    paramCfg->source.srcAcnt = numSamplesPerBlockIn - 1;
    paramCfg->source.srcAIdx = sizeof(uint16_t);
    paramCfg->source.srcBcnt = numBlocks - 1;
    paramCfg->source.srcBIdx = numSamplesPerBlockIn*sizeof(uint16_t);
    paramCfg->source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg->source.srcScale = 0U;
    paramCfg->source.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    paramCfg->source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    paramCfg->source.srcConjugate = HWA_FEATURE_BIT_DISABLE;

    paramCfg->dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObj->wrMemBankAddr);
    paramCfg->dest.dstSkipInit = 0U;
    paramCfg->dest.dstAcnt = numSamplesPerBlockOut - 1;
    paramCfg->dest.dstAIdx = sizeof(uint16_t);
    paramCfg->dest.dstBIdx = numSamplesPerBlockOut*sizeof(uint16_t);
    paramCfg->dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
    paramCfg->dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg->dest.dstSign = HWA_SAMPLES_UNSIGNED;
    paramCfg->dest.dstScale = 0U;
    paramCfg->dest.dstConjugate = HWA_FEATURE_BIT_DISABLE;

    paramCfg->accelModeArgs.compressMode.compressDecompress  = HWA_CMP_DCMP_COMPRESS;
    paramCfg->accelModeArgs.compressMode.method   = HWA_COMPRESS_METHOD_EGE;
    paramCfg->accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    paramCfg->accelModeArgs.compressMode.passSelect  = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    paramCfg->accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    paramCfg->accelModeArgs.compressMode.scaleFactorBW = 4;
    paramCfg->accelModeArgs.compressMode.EGEKarrayLength = 3;

    paramCfg->complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    #if (abs(hwa_src - hwa_dst) < CSL_APP_HWA_BANK_SIZE)
        /* MEMACCESS config params warning */
        CUSTOM_WARNING("HWA Param src and dst addresses shall not be from same memory bank");
    #endif
    
    status += HWA_configParamSet(fftObj->hwaHandle, resObj->paramIdx, paramCfg, NULL);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("Error: HWA_configParamSet failed with error: %d!!\r\n", status);
    }

    /* Init Decompression param set */
    paramCfgDcmp = &resObjDcmp->paramCfg;
    memset(paramCfgDcmp , 0, sizeof(*paramCfgDcmp));
    paramCfgDcmp->triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    paramCfgDcmp->accelMode = HWA_ACCELMODE_COMPRESS;
    
    paramCfgDcmp->source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObjDcmp->rdMemBankAddr);
    paramCfgDcmp->source.srcSign = HWA_SAMPLES_UNSIGNED;
    paramCfgDcmp->source.srcAcnt = numSamplesPerBlockOut - 1;
    paramCfgDcmp->source.srcAIdx = sizeof(uint16_t);
    paramCfgDcmp->source.srcBcnt = numBlocks - 1;
    paramCfgDcmp->source.srcBIdx = numSamplesPerBlockOut*sizeof(uint16_t);
    paramCfgDcmp->source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfgDcmp->source.srcScale = 0U;
    paramCfgDcmp->source.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    paramCfgDcmp->source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    paramCfgDcmp->source.srcConjugate = HWA_FEATURE_BIT_DISABLE;

    paramCfgDcmp->dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObjDcmp->wrMemBankAddr);
    paramCfgDcmp->dest.dstSkipInit = 0U;
    paramCfgDcmp->dest.dstAcnt = numSamplesPerBlockIn - 1;
    paramCfgDcmp->dest.dstAIdx = sizeof(uint16_t);
    paramCfgDcmp->dest.dstBIdx = numSamplesPerBlockIn*sizeof(uint16_t);
    paramCfgDcmp->dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
    paramCfgDcmp->dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfgDcmp->dest.dstSign = HWA_SAMPLES_SIGNED;
    paramCfgDcmp->dest.dstScale = 0U;
    paramCfgDcmp->dest.dstConjugate = HWA_FEATURE_BIT_DISABLE;

    paramCfgDcmp->accelModeArgs.compressMode.compressDecompress  = HWA_CMP_DCMP_DECOMPRESS;
    paramCfgDcmp->accelModeArgs.compressMode.method   = HWA_COMPRESS_METHOD_EGE;
    paramCfgDcmp->accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_DISABLE;
    paramCfgDcmp->accelModeArgs.compressMode.passSelect  = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    paramCfgDcmp->accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    paramCfgDcmp->accelModeArgs.compressMode.scaleFactorBW = 4;
    paramCfgDcmp->accelModeArgs.compressMode.EGEKarrayLength = 3;

    paramCfgDcmp->complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    #if (abs(hwa_src_dcmp - hwa_dst_dcmp) < CSL_APP_HWA_BANK_SIZE)
        /* MEMACCESS config params warning */
        CUSTOM_WARNING("HWA Param src and dst addresses shall not be from same memory bank");
    #endif
    
    status += HWA_configParamSet(fftObj->hwaHandle, resObjDcmp->paramIdx, paramCfgDcmp, NULL);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("Error: HWA_configParamSet failed with error: %d!!\r\n", status);
    }
    
    if(SystemP_SUCCESS == status)
    {
        /* Init Common Params */
        memset(commonCfg, 0, sizeof(*commonCfg));
        commonCfg->configMask = HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                                HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM |
                                HWA_COMMONCONFIG_MASK_NUMLOOPS;
        commonCfg->fftConfig.twidDitherEnable = 0U;

        cfgEGEParamListDopplerProc(&commonCfg->compressMode.EGEKparam[0], compressionRatio, HWA_SAMPLES_WIDTH_16BIT);

        commonCfg->paramStartIdx = 0U;
        commonCfg->paramStopIdx = 1U;
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
        /* Enable param done interrupt */
        status = SemaphoreP_constructBinary(&fftObj->paramDoneSem, 0);
        DebugP_assert(status == SystemP_SUCCESS);
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU;
        paramISRConfig.cpu.callbackFn = HWA_Test_ParamSetISR_Callback;
        paramISRConfig.cpu.callbackArg = &fftObj->paramDoneSem;
        status = HWA_enableParamSetInterrupt(fftObj->hwaHandle, resObj->paramIdx, &paramISRConfig);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error: HWA_enableParamDoneInterrupt failed for param no: %d\r\n", resObj->paramIdx);
        }
        
        /* Enable HWA done interrupt */
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
    
    /* Compression EDMA config */
    rdMemBankAddr = (int16_t *) resObj->rdMemBankAddr;
    wrMemBankAddr = (uint16_t *) resObj->wrMemBankAddr;
    
    
    /* Initiate 2D DMA */
    rdEdmaPrms->srcAddr   = (uint32_t) (src);
    rdEdmaPrms->destAddr  = (uint32_t) (rdMemBankAddr);
    rdEdmaPrms->aCnt      = (uint16_t) EDMA_TEST_A_COUNT;
    rdEdmaPrms->bCnt      = (uint16_t) EDMA_TEST_B_COUNT;
    rdEdmaPrms->cCnt      = (uint16_t) EDMA_TEST_C_COUNT;
    rdEdmaPrms->srcBIdx   = (int16_t) 0;
    rdEdmaPrms->destBIdx  = (int16_t) 0;
    rdEdmaPrms->opt       = 0x301000U;
    
    
    wrEdmaPrms->srcAddr   = (uint32_t) (wrMemBankAddr);
    wrEdmaPrms->destAddr  = (uint32_t) (dst);
    wrEdmaPrms->aCnt      = (uint16_t) (EDMA_TEST_A_COUNT/2); //for real and complex
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

    /* Decompression EDMA config */
    wrMemBankAddrDcmp = (int16_t *) resObjDcmp->wrMemBankAddr;
    
    wrEdmaPrmsDcmp->srcAddr   = (uint32_t) (wrMemBankAddrDcmp);
    wrEdmaPrmsDcmp->destAddr  = (uint32_t) (dstDcmp);
    wrEdmaPrmsDcmp->aCnt      = (uint16_t) EDMA_TEST_A_COUNT; //for real and complex
    wrEdmaPrmsDcmp->bCnt      = (uint16_t) EDMA_TEST_B_COUNT;
    wrEdmaPrmsDcmp->cCnt      = (uint16_t) EDMA_TEST_C_COUNT;
    wrEdmaPrmsDcmp->srcBIdx   = (int16_t) 0;
    wrEdmaPrmsDcmp->destBIdx  = (int16_t) 0;
    wrEdmaPrmsDcmp->opt       = 0x304000U;
    
    EDMASetPaRAM(baseAddr, resObjDcmp->wrParamId, wrEdmaPrmsDcmp);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSemOutDcmp, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    intrObjOutDcmp.tccNum = resObjDcmp->wrTcc;
    intrObjOutDcmp.cbFxn  = &EDMA_regionIsrFxn;
    intrObjOutDcmp.appData = (void *) &gEdmaTestDoneSemOutDcmp;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObjOutDcmp);
    DebugP_assert(status == SystemP_SUCCESS);
    

    /******** HWA FFT invoke *********/

    ctrlBaseAddr = HWA_getCommonCtrlAddr(fftObj->hwaHandle);
    DebugP_assert(ctrlBaseAddr != NULL);
    paramBaseAddr = HWA_getParamSetAddr(fftObj->hwaHandle, resObj->paramIdx);
    DebugP_assert(paramBaseAddr != NULL);
    
    /* Common control update */
    CSL_FINSR(ctrlBaseAddr->HWACCREG16, HWACCREG16_PARAMSTART_BIT_END, HWACCREG16_PARAMSTART_BIT_START, resObj->paramIdx);
    CSL_FINSR(ctrlBaseAddr->HWACCREG16, HWACCREG16_PARAMSTOP_BIT_END, HWACCREG16_PARAMSTOP_BIT_START, resObjDcmp->paramIdx);

    /* Enable HWA */
    status += HWA_enable(fftObj->hwaHandle, 1U);

    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMAEnableTransferRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, resObj->rdDmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    status += HWA_setSoftwareTrigger(fftObj->hwaHandle);
    SemaphoreP_pend(&fftObj->paramDoneSem, SystemP_WAIT_FOREVER);
    
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMAEnableTransferRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, resObj->wrDmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSemOut, SystemP_WAIT_FOREVER);
    }

    SemaphoreP_pend(&fftObj->doneSem, SystemP_WAIT_FOREVER);
    
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMAEnableTransferRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, resObjDcmp->wrDmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSemOutDcmp, SystemP_WAIT_FOREVER);
    }
    
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
    EDMAFreeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        resObjDcmp->wrDmaCh, EDMA_TRIG_MODE_MANUAL, resObjDcmp->wrTcc, EDMA_TEST_EVT_QUEUE_NO);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[EDMA-HWA] Test Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

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

/* ISR */
static void HWA_Test_ParamSetISR_Callback(uint32_t paramSet, void *arg)
{
    SemaphoreP_Object *semHandle;

    if(arg != NULL)
    {
        semHandle = (SemaphoreP_Object *)arg;
        SemaphoreP_post(semHandle);
    }
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

/**
 *  @b Description
 *  @n
 *      The function creates the k-array list for EGE compression. 
 *
 *  @param[in, out]  cmpEGEArr 		pointer to array for the k-array list. 
 *  @param[in]  compressionRatio 	the desired compression ratio.  
 *  @param[in]  bitwidth 		the input formatter bitwidth (16/32-bit)
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
void  cfgEGEParamListDopplerProc(uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth)
{ 
	uint32_t ik;

	if (srcWidth == HWA_SAMPLES_WIDTH_16BIT)
	{
		if (compressionRatio == HWA_CMP_50P_RATIO)
		{
			cmpEGEArr[0] = 6; 
			cmpEGEArr[1] = 7; 
			cmpEGEArr[2] = 8; 
			cmpEGEArr[3] = 9; 
			cmpEGEArr[4] = 10; 
			cmpEGEArr[5] = 11; 
			cmpEGEArr[6] = 13; 
			cmpEGEArr[7] = 15; 
		}
		else
		{
			for (ik = 0; ik < HWA_CMP_K_ARR_LEN; ik ++)
			{
				cmpEGEArr[ik] = 2 * ik + 1;
			}

		}
	}
	else
	{
		for (ik = 0; ik < HWA_CMP_K_ARR_LEN; ik ++)
		{
			cmpEGEArr[ik] = 4 * ik + 1;
		}

	}
}
