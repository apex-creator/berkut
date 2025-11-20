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
 *  This example performs RangeProc DPU test.
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
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <datapath/dpu/rangeproc/v0/rangeprochwa.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define EDMA_TEST_EVT_QUEUE_NO      (0U)
#define MAX_NUM_TX_ANTENNA          (2U)
#define MAX_NUM_RX_ANTENNA          (3U)
#define MAX_NUM_RANGEBIN            (128U)
#define MAX_NUM_ADCSAMPLE_PERCHIRP  (128U)
#define MAX_NUM_CHIRPS_PERFRAME     (64U)

#define DPC_OBJDET_EDMA_SHADOW_BASE                                    SOC_EDMA_NUM_DMACH
#define DPC_OBJDET_HWA_WINDOW_RAM_OFFSET                               0

/* Range DPU */
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH                             EDMA_APPSS_TPCC_B_EVT_CHIRP_AVAIL_IRQ
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PING                    (DPC_OBJDET_EDMA_SHADOW_BASE + 0)
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PONG                    (DPC_OBJDET_EDMA_SHADOW_BASE + 1)
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_EVENT_QUE                      0
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_CH                         EDMA_APPSS_TPCC_B_EVT_FREE_0
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_SHADOW                     (DPC_OBJDET_EDMA_SHADOW_BASE + 2)
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_EVENT_QUE                  0

#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_CH                 EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ0
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_SHADOW             (DPC_OBJDET_EDMA_SHADOW_BASE + 3)
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_EVENT_QUE          0

#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_CH                 EDMA_APPSS_TPCC_B_EVT_FREE_1
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_SHADOW             (DPC_OBJDET_EDMA_SHADOW_BASE + 4)
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_EVENT_QUE          0

#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_CH                     EDMA_APPSS_TPCC_B_EVT_FREE_2
#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_0               (DPC_OBJDET_EDMA_SHADOW_BASE + 5)
#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_1               (DPC_OBJDET_EDMA_SHADOW_BASE + 6)
#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_EVENT_QUE              0

#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_CH                 EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ1
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_SHADOW             (DPC_OBJDET_EDMA_SHADOW_BASE + 7)
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_EVENT_QUE          0

#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_CH                 EDMA_APPSS_TPCC_B_EVT_FREE_3
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_SHADOW             (DPC_OBJDET_EDMA_SHADOW_BASE + 8)
#define DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_EVENT_QUE          0

#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_CH                     EDMA_APPSS_TPCC_B_EVT_FREE_4
#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_0               (DPC_OBJDET_EDMA_SHADOW_BASE + 9)
#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_1               (DPC_OBJDET_EDMA_SHADOW_BASE + 10)
#define DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_EVENT_QUE              0
#define MMW_DEMO_TEST_ADC_BUFF_SIZE                                    1024  

#define PI_ 3.14159265

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))

FILE * fileId;
FILE * fileId_read;
HWA_Handle              gHwaHandle;
DPU_RangeProcHWA_Handle rangeProcDpuHandle;
DPU_RangeProcHWA_Config rangeProcDpuCfg;

uint8_t testFeatureStatus;

uint8_t checkResultFlag;

uint8_t finalResults ;

Edma_IntrObject     intrObj;

/*! ADC samples buffer when reading ADC data from file */
uint8_t gAdcTestBuff[MMW_DEMO_TEST_ADC_BUFF_SIZE] __attribute__((section(".l3")));

uint32_t window1DCoef[MAX_NUM_ADCSAMPLE_PERCHIRP] __attribute__((section(".l3")));

int16_t radarCube[MAX_NUM_ADCSAMPLE_PERCHIRP * MAX_NUM_CHIRPS_PERFRAME * 6] __attribute((section(".l3")));

int16_t radarCubeRef[MAX_NUM_ADCSAMPLE_PERCHIRP * MAX_NUM_CHIRPS_PERFRAME * 6] __attribute((section(".l3")));

typedef struct rangeProcTestConfig_t_ {
    uint32_t numTxAntennas;
    uint32_t numRxAntennas;
    uint32_t numVirtualAntennas;
    uint32_t numAdcSamples;
    uint32_t numRangeBins;
    uint32_t dpuNumRangeBins;
    uint32_t numChirpsPerFrame;
    uint32_t numChirpsPerFrameRef;
    uint32_t numFrames;
    uint8_t  cmplxIQswapFlag;    
    uint8_t  rxChanInterleave;
    uint8_t  radarCubeLayoutFmt;
} rangeProcTestConfig_t;


int32_t checkTestResult(int16_t *testOut, int16_t *refOut, int32_t numSamples)
{
    int32_t status = SystemP_SUCCESS;
    int32_t i;
    
    for (i = 0; i < numSamples; i++) //complex samples
    {
        if (testOut[i] != refOut[i])
        {
            status = SystemP_FAILURE;
            break;
        }
    }


    return status;
}

/**
*  @b Description
*  @n
*    Based on the test configuration, set up the range processing DPU configurations
*/

void Test_setProfile(rangeProcTestConfig_t * testConfig)
{
    DPU_RangeProcHWA_HW_Resources  *pHwConfig;
    DPU_RangeProcHWA_StaticConfig  * params;
    uint32_t index;
    uint32_t      bytesPerRxChan;
    
    pHwConfig = &rangeProcDpuCfg.hwRes; 
    params = &rangeProcDpuCfg.staticCfg; 
    
    params->numTxAntennas = 2; //testConfig->numTxAntennas;
    params->numVirtualAntennas = 6; //testConfig->numTxAntennas * testConfig->numRxAntennas;
    params->numRangeBins = testConfig->numAdcSamples/2; //testConfig->numRangeBins;
    params->numChirpsPerFrame = testConfig->numChirpsPerFrame;
    params->numDopplerChirpsPerFrame = params->numChirpsPerFrame/params->numTxAntennas;
    params->numDopplerChirpsPerProc = params->numDopplerChirpsPerFrame;
    params->isBpmEnabled = TRUE;
    /* windowing */
    params->windowSize = sizeof(uint32_t) * ((testConfig->numAdcSamples +1 ) / 2); //symmetric window, for real samples
    params->ADCBufData.dataProperty.numAdcSamples = testConfig->numAdcSamples;
    params->ADCBufData.dataProperty.numRxAntennas = testConfig->numRxAntennas;

    params->rangeFFTtuning.fftOutputDivShift = 2;
    params->rangeFFTtuning.numLastButterflyStagesToScale = 0; /* no scaling needed as ADC is 16-bit and we have 8 bits to grow */  
    params->enableMajorMotion = 1;
    params->enableMinorMotion = 0;
    params->numMinorMotionChirpsPerFrame = 0;
    params->numFramesPerMinorMotProc = 0;

    params->rangeFftSize = testConfig->numAdcSamples;
    
    bytesPerRxChan = testConfig->numAdcSamples * sizeof(uint16_t);
    bytesPerRxChan = (bytesPerRxChan + 15) / 16 * 16;

    for (index = 0; index < testConfig->numRxAntennas; index++)
    {
        params->ADCBufData.dataProperty.rxChanOffset[index] = index * bytesPerRxChan;
    }

    params->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
    /* Data Input EDMA */
    pHwConfig->edmaInCfg.dataIn.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    pHwConfig->edmaInCfg.dataIn.channelShadow[0]   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PING;
    pHwConfig->edmaInCfg.dataIn.channelShadow[1]   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PONG;
    pHwConfig->edmaInCfg.dataIn.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_EVENT_QUE;
    pHwConfig->edmaInCfg.dataInSignature.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_CH;
    pHwConfig->edmaInCfg.dataInSignature.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_SHADOW;
    pHwConfig->edmaInCfg.dataInSignature.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_EVENT_QUE;
    pHwConfig->intrObj = &intrObj;

    /* Data Output EDMA */
    pHwConfig->edmaOutCfg.path[0].evtDecim.channel = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_CH;
    pHwConfig->edmaOutCfg.path[0].evtDecim.channelShadow[0] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_0;
    pHwConfig->edmaOutCfg.path[0].evtDecim.channelShadow[1] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_1;
    pHwConfig->edmaOutCfg.path[0].evtDecim.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_EVENT_QUE;
    
    pHwConfig->edmaOutCfg.path[1].evtDecim.channel = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].evtDecim.channelShadow[0] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_0;
    pHwConfig->edmaOutCfg.path[1].evtDecim.channelShadow[1] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_1;
    pHwConfig->edmaOutCfg.path[1].evtDecim.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_EVENT_QUE;
    
    pHwConfig->edmaOutCfg.path[0].dataOutMinor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_CH;
    pHwConfig->edmaOutCfg.path[0].dataOutMinor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_SHADOW;
    pHwConfig->edmaOutCfg.path[0].dataOutMinor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_EVENT_QUE;
    
    pHwConfig->edmaOutCfg.path[1].dataOutMinor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].dataOutMinor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_SHADOW;
    pHwConfig->edmaOutCfg.path[1].dataOutMinor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[0].dataOutMajor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_CH;
    pHwConfig->edmaOutCfg.path[0].dataOutMajor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_SHADOW;
    pHwConfig->edmaOutCfg.path[0].dataOutMajor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_EVENT_QUE;
    
    pHwConfig->edmaOutCfg.path[1].dataOutMajor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].dataOutMajor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_SHADOW;
    pHwConfig->edmaOutCfg.path[1].dataOutMajor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_EVENT_QUE;
   
    /* radar cube*/
    pHwConfig->radarCube.dataSize = testConfig->numRangeBins * testConfig->numTxAntennas * testConfig->numRxAntennas * sizeof(cmplx16ReIm_t) * params->numDopplerChirpsPerProc;
    pHwConfig->radarCube.datafmt = DPIF_RADARCUBE_FORMAT_6;

}

/**
*  @b Description
*  @n
*    Generate the windowing coefficients, only support BLACKMAN
*/
void Test_1DwindowCoef_gen(uint32_t numAdcSamples)
{ 

#if 1    
    float ephyR, ephyI;
    float phi = 2 * PI_ / ((float) numAdcSamples - 1);
    
    
    uint32_t ii;
    float ephyR2, ephyI2;
    float a0, a1, a2;
    float winValue;
    int32_t winValueFixed;
    float cosValue, sinValue, cosValue2, sinValue2;
    float temp;
    float initR, initI;
    
    a0 = 0.42;
    a1 = 0.5;
    a2 = 0.08;
    
    cosValue = 1.f;
    sinValue = 0.f; 
    cosValue2 = 1.f;
    sinValue2 = 0.f;

    initR  = cos(phi);
    initI  = sin(phi);
    ephyR = initR;
    ephyI = initI;
    ephyR2 = 1.f - (initI * initI) * 2.f; /* cos(2a)=1-2*sin(a)^2 */
    ephyI2 = 2.f * initR * initI; /* sin(2a)=2*sin(a)*cos(a) */

        
    /* window is whole length for hwa*/
    for (ii = 0; ii < numAdcSamples; ii++)
    {
        winValue = a0 - a1 * cosValue + a2 * cosValue2 ;
        winValue = winValue * (1<<17) + 0.5;
        winValueFixed = (uint32_t) winValue;
        if (winValueFixed >= (1<<17))
        {
            winValueFixed = (1<<17) - 1;
        }
        temp = cosValue;
        cosValue = cosValue * ephyR - sinValue * ephyI;
        sinValue = temp * ephyI + sinValue * ephyR;
        
        temp = cosValue2;
        cosValue2 = cosValue2 * ephyR2 - sinValue2 * ephyI2;
        sinValue2 = temp * ephyI2 + sinValue2 * ephyR2;
        window1DCoef[ii] = winValueFixed;
    }
#endif

}

void rangeProcDpuTest_hwaInit()
{
    int32_t status = SystemP_SUCCESS;
    
    /* Open HWA driver */
    gHwaHandle = HWA_open(0, NULL, &status);
    if(gHwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open HWA instance. Error: %d\n", status);
        DebugP_assert(gHwaHandle == NULL);
    }

}

void rangeProcDpuTest_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_RangeProcHWA_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    rangeProcDpuHandle =  DPU_RangeProcHWA_init (&initParams, &errorCode);
    if (rangeProcDpuHandle == NULL)
    {
        DebugP_log ("Debug: RangeProc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*        set up the range processing DPU configurations, do not change for each test 
*/
void rangeProcDpuTest_dpuCfg()
{
    DPU_RangeProcHWA_HW_Resources * pHwConfig;
    
    memset((void *)&rangeProcDpuCfg, 0, sizeof(DPU_RangeProcHWA_Config));
        
    
    /* hwi configuration */
    pHwConfig = &rangeProcDpuCfg.hwRes; 
        
    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = 0;
    pHwConfig->hwaCfg.numParamSet = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;
    pHwConfig->hwaCfg.hwaWinRamOffset  = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET; 
    pHwConfig->hwaCfg.hwaWinSym = 1;
    pHwConfig->hwaCfg.dataInputMode = DPU_RangeProcHWA_InputMode_ISOLATED;

    /* edma configuration */
    pHwConfig->edmaHandle  = gEdmaHandle[0];  
    /* edma configuration depends on the interleave or non-interleave */
    
    /* windowing buffer is fixed, size will change*/
    rangeProcDpuCfg.staticCfg.window =  (int32_t *)&window1DCoef[0];
    
    /* adc buffer buffer, format fixed, interleave, size will change */
    rangeProcDpuCfg.staticCfg.ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_REAL16;
    rangeProcDpuCfg.staticCfg.ADCBufData.dataProperty.adcBits = 2U; // 12-bit only
    rangeProcDpuCfg.staticCfg.ADCBufData.dataProperty.numChirpsPerChirpEvent = 1U;
    rangeProcDpuCfg.staticCfg.ADCBufData.data = (void *) gAdcTestBuff;
    
    /* radar cube */
    rangeProcDpuCfg.hwRes.radarCube.data  = (cmplx16ImRe_t *) &radarCube[0];
    
}

void rangeproc_test_main(void *args)
{
    uint32_t            baseAddr, regionId;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i; 
    uint32_t            testCount=0;
    int32_t             retVal = 0;
    int32_t             errorCode = 0;
    rangeProcTestConfig_t testConfig;
    bool                endOfFile = false;
    uint32_t            numAdcSamplesPerEvt;
    uint32_t            numReadSamples;

    DPU_RangeProcHWA_OutParams outParms;

    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    DebugP_log("DPU Test Started \r\n");
    
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    rangeProcDpuTest_hwaInit();
    rangeProcDpuTest_dpuInit();
    /* config the range proc DPU */
    rangeProcDpuTest_dpuCfg();

    /* start the test */
    fileId = fopen("..\\..\\..\\..\\common\\testBench\\major_motion\\adc_data_0001_CtestAdc6Ant.bin", "rb");
    if (fileId == NULL)
    {
        printf("Error:  Cannot open adc_data_0001_CtestAdc6Ant.bin !\n");
        exit(0);
    }

    /* read in test config */
    testConfig.numRxAntennas = 3;
    fread(&testConfig.numAdcSamples, sizeof(uint32_t),1,fileId);
    fread(&testConfig.numVirtualAntennas, sizeof(uint32_t),1,fileId);
    fread(&testConfig.numChirpsPerFrame, sizeof(uint32_t),1,fileId);
    fread(&testConfig.numFrames, sizeof(uint32_t),1,fileId);

    testConfig.numTxAntennas = testConfig.numVirtualAntennas/testConfig.numRxAntennas;
    testConfig.numRangeBins = testConfig.numAdcSamples/2; //real only input 
    testConfig.numChirpsPerFrameRef = testConfig.numChirpsPerFrame;

    numAdcSamplesPerEvt = (testConfig.numAdcSamples * testConfig.numRxAntennas);

    if ((testConfig.numTxAntennas > MAX_NUM_TX_ANTENNA) || (testConfig.numRangeBins > MAX_NUM_RANGEBIN) || (testConfig.numChirpsPerFrame > MAX_NUM_CHIRPS_PERFRAME))
    {
        DebugP_log("Wrong test configurations \n");
        testFeatureStatus = 0;
        finalResults = 0;
        goto exit;
    }

    testConfig.cmplxIQswapFlag = 0; /* for arm, not support IQ swap */
    
    DebugP_log("numTxAntennas = %d\r", testConfig.numTxAntennas);
    DebugP_log("numRangeBins = %d\r", testConfig.numRangeBins);  
    DebugP_log("numChirpsPerFrame = %d\n", testConfig.numChirpsPerFrame);
    DebugP_log("\n");
    
    /* generate the 1D window coefficients */
    Test_1DwindowCoef_gen(testConfig.numAdcSamples);
    
    testFeatureStatus = 1;    
    
    Test_setProfile(&testConfig);

    checkResultFlag = 1;
    
    retVal = DPU_RangeProcHWA_config (rangeProcDpuHandle,
                                        &rangeProcDpuCfg);

    if(retVal < 0)
    {
        DebugP_log("DEBUG: RANGE DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* control the rangeproc hwa*/
    retVal = DPU_RangeProcHWA_control(rangeProcDpuHandle, DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
    if(retVal < 0)
    {
        /* Not Expected */
        DebugP_log("RangeProc DPU control error %d\n", retVal);
        DebugP_assert(0);
    }

    /* Read chirps from the file */
    for(i = 0; i < testConfig.numChirpsPerFrame; i++)
    {
        if (!endOfFile)
        {
            /* Read one chirp of ADC samples and to put data in ADC test buffer */
            numReadSamples = fread(gAdcTestBuff, sizeof(uint16_t),  numAdcSamplesPerEvt, fileId); 
            if (numReadSamples != numAdcSamplesPerEvt)
            {
                endOfFile = true;
            }
        }
                    
        /* Manual trigger to simulate chirp avail irq */
        errorCode = EDMAEnableTransferRegion(
                        baseAddr, regionId, EDMA_APPSS_TPCC_B_EVT_CHIRP_AVAIL_IRQ, EDMA_TRIG_MODE_MANUAL); //EDMA_TRIG_MODE_EVENT
        if (errorCode != 1)
        {
            DebugP_log("Error: EDMA start Transfer returned %d\n",errorCode);
            return;
        }
        
        ClockP_usleep(1000); //1s sleep

    } /* end of chirp loop */

    memset((void *)&outParms, 0, sizeof(DPU_RangeProcHWA_OutParams));

    retVal = DPU_RangeProcHWA_process(rangeProcDpuHandle, &outParms);

    /* check the result */
    DebugP_log("\n... DPU Finish, Check Cube data ....  : \n\n");
    if (checkResultFlag == 1)
    {
        testConfig.dpuNumRangeBins = rangeProcDpuCfg.staticCfg.numRangeBins;
       
    }
    else  
    {
        DebugP_log("Test Cube Data FAILED!\n");
        testFeatureStatus = 0;
        finalResults = 0;
    }


    DebugP_log("Test #%d finished!\n\r", testCount);
    testCount++;
        
    DebugP_log("\n");

exit:
    fclose(fileId);
     

    if(status == SystemP_SUCCESS)
    {
        fileId_read = fopen("..\\..\\..\\..\\common\\testBench\\major_motion\\adc_data_0001_CtestAdc6Ant_demo_RadarCube.bin", "rb");
        if (fileId_read == NULL)
        {
            printf("Error:  adc_data_0001_CtestAdc6Ant_demo_RadarCube.bin !\n");
            exit(0);
        }
        fread((uint16_t *)&radarCubeRef, testConfig.numRangeBins * testConfig.numRxAntennas  * testConfig.numChirpsPerFrame * sizeof(cmplx16ReIm_t),  1, fileId_read);
        status = checkTestResult(radarCube, radarCubeRef, testConfig.numRangeBins * testConfig.numRxAntennas  * testConfig.numChirpsPerFrame * sizeof(uint16_t));
        if(status == SystemP_SUCCESS)
        {
            DebugP_log("All tests have passed!!\r\n");
        }
        else
        {
            DebugP_log("Some tests have failed!!\r\n");
        }
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
    return;
}


