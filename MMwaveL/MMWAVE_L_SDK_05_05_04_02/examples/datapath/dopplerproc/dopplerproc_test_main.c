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
 *  This example performs dopplerProc DPU test.
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
#include <datapath/dpu/dopplerproc/v0/dopplerprochwa.h>
#include <utils/mathutils/mathutils.h>
#include <drivers/hw_include/cslr_adcbuf.h>
#include <kernel/dpl/CycleCounterP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define MATLAB_MODEL_BITMATCHING    1U

#define EDMA_TEST_EVT_QUEUE_NO      (0U)
#define MAX_NUM_TX_ANTENNA          (2U)
#define MAX_NUM_RX_ANTENNA          (3U)
#define MAX_NUM_RANGEBIN            (64U)
#define MAX_NUM_ADCSAMPLE_PERCHIRP  (64U)
#define MAX_NUM_CHIRPS_PERFRAME     (128U)

#define DPC_OBJDET_EDMA_SHADOW_BASE                                    SOC_EDMA_NUM_DMACH
#define DPC_OBJDET_HWA_WINDOW_RAM_OFFSET                               0

/* Doppler DPU */
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_CH_PING                        EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ10
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_CH_PONG                        EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ11
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SHADOW_PING                    (DPC_OBJDET_EDMA_SHADOW_BASE + 0)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SHADOW_PONG                    (DPC_OBJDET_EDMA_SHADOW_BASE + 1)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_EVENT_QUE_PING                 0
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_EVENT_QUE_PONG                 0

#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_CH_PING                    EDMA_APPSS_TPCC_B_EVT_FREE_0
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_SHADOW_PING                (DPC_OBJDET_EDMA_SHADOW_BASE + 2)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_CH_PONG                    EDMA_APPSS_TPCC_B_EVT_FREE_1
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_SHADOW_PONG                (DPC_OBJDET_EDMA_SHADOW_BASE + 3)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_EVENT_QUE_PING             0
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_EVENT_QUE_PONG             0

#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_CH             EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ0
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_SHADOW         (DPC_OBJDET_EDMA_SHADOW_BASE + 4)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_EVENT_QUE      0

#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_CH             EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ1
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_SHADOW         (DPC_OBJDET_EDMA_SHADOW_BASE + 5)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_EVENT_QUE      0


#define MMW_DEMO_TEST_1DFFT_BUFF_SIZE                                    MAX_NUM_CHIRPS_PERFRAME * MAX_NUM_RANGEBIN * MAX_NUM_TX_ANTENNA * MAX_NUM_RX_ANTENNA
#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE                              MATHUTILS_WIN_HANNING
#define DPC_OBJDET_QFORMAT_DOPPLER_FFT                                   17
#define NUM_DOPPLER_BINS_ZERO_OUT                                        5
#define THRESH_VAL                                                       50 // Threshold above which it will be counted as error

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))

FILE * fileId;
FILE * fileId_write;
FILE * fileId_read;
HWA_Handle              gHwaHandle;
DPU_DopplerProcHWA_Handle dopplerProcDpuHandle;
DPU_DopplerProcHWA_Config dopplerProcDpuCfg;

uint8_t testFeatureStatus;

uint8_t checkResultFlag;

uint8_t finalResults ;

Edma_IntrObject     intrObj;

/*! Doppler samples buffer when reading 1D-FFT data from file */
uint32_t rangeFFT[MMW_DEMO_TEST_1DFFT_BUFF_SIZE] __attribute__((section(".l3"), aligned(32)));
uint32_t window2DCoef[MAX_NUM_CHIRPS_PERFRAME] __attribute__((section(".data"), aligned(32)));
uint32_t detMatrix[MAX_NUM_ADCSAMPLE_PERCHIRP * MAX_NUM_CHIRPS_PERFRAME] __attribute__((section(".data"), aligned(32)));
uint32_t detMatrixRef[MAX_NUM_ADCSAMPLE_PERCHIRP * MAX_NUM_CHIRPS_PERFRAME] __attribute__((section(".l3"), aligned(32)));

typedef struct dopplerProcTestConfig_t_ {
    uint32_t numTxAntennas;
    uint32_t numRxAntennas;
    uint32_t numVirtualAntennas;
    uint32_t numRangeBins;
    uint32_t numDopplerChirps;
    uint32_t numFrames;   
} dopplerProcTestConfig_t;

static uint32_t HWAFFT_log2Approx(uint32_t x);

uint32_t Cycleprofiler_getTimeStamp(void)
{
    uint32_t *frameRefTimer;
    frameRefTimer = (uint32_t *) 0x5B000020;
    return *frameRefTimer;
}

/**
*  @b Description
*  @n
*    Based on the test configuration, set up the Doppler processing DPU configurations
*/
void Test_setProfile(dopplerProcTestConfig_t * testConfig)
{
    DPU_DopplerProcHWA_HW_Resources  *pHwConfig;
    DPU_DopplerProcHWA_StaticConfig  *params;
    
    pHwConfig = &dopplerProcDpuCfg.hwRes; 
    params = &dopplerProcDpuCfg.staticCfg; 
    
    /* overwrite the DPIF_commonParams with the test configuration*/
    params->numTxAntennas = testConfig->numTxAntennas;
    params->numRxAntennas = testConfig->numRxAntennas;
    params->numVirtualAntennas = testConfig->numTxAntennas * testConfig->numRxAntennas;
    params->numRangeBins = testConfig->numRangeBins;
    params->numDopplerChirps = testConfig->numDopplerChirps;
    params->log2NumDopplerBins = HWAFFT_log2Approx(params->numDopplerChirps);
    params->numDopplerBins = pow(2,params->log2NumDopplerBins);
    params->doppFFT_is16b = 0; //Takes into account of 16-bit complex Doppler FFT
    
    /* windowing */
    pHwConfig->hwaCfg.windowSize = sizeof(uint32_t) * ((params->numDopplerBins + 1) / 2); //symmetric window, for real samples
    
    /* radar cube after 1D-FFT */
    // pHwConfig->radar_1D_FFT_Cube.data = (void *)rangeFFT;
    pHwConfig->radar_1D_FFT_Cube.dataSize = testConfig->numRangeBins * testConfig->numTxAntennas * testConfig->numRxAntennas * sizeof(cmplx16ReIm_t) * params->numDopplerChirps;
    pHwConfig->radar_1D_FFT_Cube.datafmt = DPIF_RADARCUBE_FORMAT_6;
    
    /* detection Matrix after 2D-FFT and non-coherent addition*/
    pHwConfig->detMatrix.datafmt = DPIF_DETMATRIX_FORMAT_1;
    pHwConfig->detMatrix.dataSize = params->numRangeBins * params->numDopplerBins * sizeof(cmplx16ReIm_t);

    /* Data Input EDMA */
    pHwConfig->edmaCfg.edmaIn.ping.channel        = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_CH_PING;
    pHwConfig->edmaCfg.edmaIn.pong.channel        = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_CH_PONG;
    pHwConfig->edmaCfg.edmaIn.ping.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SHADOW_PING;
    pHwConfig->edmaCfg.edmaIn.pong.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaIn.ping.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaIn.pong.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_EVENT_QUE_PONG;

    /* Hot signature EDMA */
    pHwConfig->edmaCfg.edmaHotSig.ping.channel    = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_CH_PING;
    pHwConfig->edmaCfg.edmaHotSig.pong.channel    = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_CH_PONG;
    pHwConfig->edmaCfg.edmaHotSig.ping.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_SHADOW_PING;
    pHwConfig->edmaCfg.edmaHotSig.pong.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaHotSig.ping.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaHotSig.pong.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_EVENT_QUE_PING;
    pHwConfig->intrObj = &intrObj;

    /* Data Output EDMA */
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.ping.channel = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_CH;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.pong.channel = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_CH;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.ping.channelShadow = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_SHADOW;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.pong.channelShadow = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_SHADOW;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.ping.eventQueue = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_EVENT_QUE;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.pong.eventQueue = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_EVENT_QUE;
}

/**
*  @b Description
*  @n
*    To generate the window for Doppler FFT - Hanning window is used in gesture/K2O application
*/
void Test_2DwindowCoef_gen(uint32_t numDopplerChirps)
{
    mathUtils_genWindow((uint32_t *)window2DCoef,
                        numDopplerChirps,
                        numDopplerChirps/2,
                        DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE,
                        DPC_OBJDET_QFORMAT_DOPPLER_FFT);
}

/**
*  @b Description
*  @n
*    Function to initialize the HWA used in this Doppler test
*/
void dopplerProcDpuTest_hwaInit()
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

/**
*  @b Description
*  @n
*    Function to initialize Doppler DPU
*/
void dopplerProcDpuTest_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_DopplerProcHWA_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    dopplerProcDpuHandle =  DPU_DopplerProcHWA_init (&initParams, &errorCode);
    if (dopplerProcDpuHandle == NULL)
    {
        DebugP_log ("Debug: Doppler Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*        set up the doppler processing DPU configurations, do not change for each test 
*/
void dopplerProcDpuTest_dpuCfg()
{
    DPU_DopplerProcHWA_HW_Resources * pHwConfig;
    
    memset((void *)&dopplerProcDpuCfg, 0, sizeof(DPU_DopplerProcHWA_Config));
        
    
    /* hwi configuration */
    pHwConfig = &dopplerProcDpuCfg.hwRes; 
        
    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = 0;
    pHwConfig->hwaCfg.winRamOffset  = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET; 
    pHwConfig->hwaCfg.winSym = 1;
    pHwConfig->hwaCfg.dmaTrigSrcPingChan = 0;
    pHwConfig->hwaCfg.dmaTrigSrcPongChan = 1;

    /* edma configuration */
    pHwConfig->edmaCfg.edmaHandle  = gEdmaHandle[0];  
    
    /* windowing buffer is fixed, size will change */
    dopplerProcDpuCfg.hwRes.hwaCfg.window =  (int32_t *)&window2DCoef[0];

    /* radar 1D-FFT cube */
    dopplerProcDpuCfg.hwRes.radar_1D_FFT_Cube.data = (void *)rangeFFT;    
    
    /* radar detection Matrix after 2D-FFT and summation */
    dopplerProcDpuCfg.hwRes.detMatrix.data  = (uint32_t *) &detMatrix[0];
    
}


/**
*  @b Description
*  @n
*    main function from which Doppler processing test-case begins
*/
void dopplerproc_test_main(void *args)
{
    volatile uint32_t   baseAddr, regionId, startCycle, cycleCount;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i,j,k;
    uint32_t            testCount=0;
    int32_t             retVal = 0;
    uint32_t            errCount = 0;
    dopplerProcTestConfig_t testConfig={0};
    bool                endOfFile = false;
    uint32_t            rangeFFT_size_perChirp;
    uint32_t            numRead_1DFFT_samples, numDopplerBins;;

    DPU_DopplerProcHWA_OutParams outParms;

    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    DebugP_log("DPU Test Started \r\n");
    
    SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT|SOC_RCM_MEMINIT_TPCCA_INIT|SOC_RCM_MEMINIT_TPCCB_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT);

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Initialize the HWA for Doppler DPU unit-testing */
    dopplerProcDpuTest_hwaInit();
    /* Initialize the Doppler DPU for unit-testing */
    dopplerProcDpuTest_dpuInit();
    /* config the Doppler proc DPU */
    dopplerProcDpuTest_dpuCfg();

    /* start the test by reading in the range FFT binary file*/
    fileId = fopen("..\\..\\..\\..\\common\\testBench\\dopplerproc\\rangeFFT_data_final2.bin", "rb");
    if (fileId == NULL)
    {
        printf("Error:  Cannot open rangeFFT_bin_data.bin !\n");
        exit(0);
    }   

    /* read in test config */
    fread(&testConfig.numTxAntennas, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numRxAntennas, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numRangeBins, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numDopplerChirps, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numFrames, sizeof(uint16_t),1,fileId);
    if(MATLAB_MODEL_BITMATCHING)
    {   
        fileId_write = fopen("..\\..\\..\\..\\common\\testBench\\dopplerproc\\detMatrix.bin", "wb");
        if (fileId_write == NULL)
        {
            printf("Error:  Cannot open detMatrix.bin !\n");
            exit(0);
        }
        numDopplerBins = pow(2,HWAFFT_log2Approx(testConfig.numDopplerChirps));
        fileId_read = fopen("..\\..\\..\\..\\common\\testBench\\dopplerproc\\detMatrixRef.bin", "rb");
        if (fileId_read == NULL)
        {
            printf("Error:  Cannot open detMatrixRef.bin !\n");
            exit(0);
        }
    }

    testConfig.numVirtualAntennas = testConfig.numTxAntennas * testConfig.numRxAntennas;

    rangeFFT_size_perChirp = (testConfig.numRangeBins * testConfig.numRxAntennas * testConfig.numTxAntennas * sizeof(uint16_t));

    if ((testConfig.numTxAntennas > MAX_NUM_TX_ANTENNA) || (testConfig.numRangeBins > MAX_NUM_RANGEBIN) || (testConfig.numDopplerChirps > MAX_NUM_CHIRPS_PERFRAME))
    {
        DebugP_log("Wrong test configurations \n");
        testFeatureStatus = 0;
        finalResults = 0;
        goto exit;
    }
    
    DebugP_log("numTxAntennas = %d\r", testConfig.numTxAntennas);
    DebugP_log("numRxAntennas = %d\r", testConfig.numRxAntennas);
    DebugP_log("numRangeBins = %d\r", testConfig.numRangeBins);  
    DebugP_log("numDopplerChirps = %d\n", testConfig.numDopplerChirps);
    DebugP_log("numFrames = %d\n", testConfig.numFrames);
    DebugP_log("\n");
    
    /* generate the 1D window coefficients */
    Test_2DwindowCoef_gen(testConfig.numDopplerChirps);
    
    testFeatureStatus = 1;    
    
    Test_setProfile(&testConfig);

    checkResultFlag = 1;
    
    retVal = DPU_DopplerProcHWA_config(dopplerProcDpuHandle, &dopplerProcDpuCfg);

    if(retVal < 0)
    {
        DebugP_log("DEBUG: DOPPLER DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Read rangeFFT data and put in HWA_SS_RAM */
    for(i = 0; i < testConfig.numFrames; ++i)
    {   
        DebugP_log("Frame %d data read started\n",i);
        for(j=0;j<testConfig.numDopplerChirps;++j)
        {
            numRead_1DFFT_samples = fread(((uint16_t *)&rangeFFT[0]+j*rangeFFT_size_perChirp), sizeof(uint16_t), rangeFFT_size_perChirp, fileId); //Take in cmplx16ImRe_t samples after 1D-FFT
            if (numRead_1DFFT_samples != rangeFFT_size_perChirp)
            {
                endOfFile = true;
                goto exit;
            }
            if(j%10 == 0)
            {
                DebugP_log("Reading Data From File.. %d percent\n", j * 100 / testConfig.numDopplerChirps);
            }
        }/* end of chirp loop */

        memset((void *)&outParms, 0, sizeof(DPU_DopplerProcHWA_OutParams));
        /* Call the compute function */
        startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
        retVal = DPU_DopplerProcHWA_process(dopplerProcDpuHandle, &outParms);
        if(retVal < 0)
        {
            DebugP_log("DEBUG: DOPPLER DPU process return error:%d \n", retVal);
            DebugP_assert(0);
        }
        cycleCount = CycleCounterP_getCount32()-startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
        DebugP_log("Measured processing time (dopplerproc) = CPU cycles = %d !!!\r\n", cycleCount);

        if(MATLAB_MODEL_BITMATCHING)
        {
            for(j=0;j<testConfig.numRangeBins;++j)
            {
                fwrite(((uint8_t *)&detMatrix[j*numDopplerBins]), 4, numDopplerBins, fileId_write);
                if(j%10 == 0)
                {
                    DebugP_log("Writing Data to File.. %d percent\n", j * 100 / testConfig.numRangeBins);
                }
            }
            DebugP_log("Writing Output Data to File done\n");
            //fwrite(detMatrix, sizeof(detMatrix), 1, fileId_write);
            //Traverse through detMatrix to find where the ref value is exceeded
            fread((uint32_t *)&detMatrixRef, numDopplerBins*testConfig.numRangeBins*sizeof(uint32_t),  1, fileId_read);
            for(j=0;j<testConfig.numRangeBins;++j)
            {
                for(k=NUM_DOPPLER_BINS_ZERO_OUT;k<numDopplerBins-NUM_DOPPLER_BINS_ZERO_OUT;++k)
                {
                    if(abs((int32_t)(detMatrixRef[j*numDopplerBins+k]-detMatrix[j*numDopplerBins+k]))>THRESH_VAL)
                    {
                        ++errCount;
                        DebugP_log("Difference: %d at Doppler-Bin:%d and Range-Bin:%d !!\r\n", abs((int32_t)(detMatrixRef[j*numDopplerBins+k]-detMatrix[j*numDopplerBins+k])),k,j);
                    }
                }
            }
        }
        DebugP_log("Total no of points in detection Matrix compared: %d!!\r\n", (numDopplerBins-2*NUM_DOPPLER_BINS_ZERO_OUT-1)*testConfig.numRangeBins);
        DebugP_log("Total no of points failed: %d!!\r\n", errCount);
        DebugP_log("Overal frame performance: %%%.2f!!\r\n", (float)(((numDopplerBins-2*NUM_DOPPLER_BINS_ZERO_OUT-1)*testConfig.numRangeBins-errCount) / (float)((numDopplerBins-2*NUM_DOPPLER_BINS_ZERO_OUT-1)*testConfig.numRangeBins)) * 100.0f);
        DebugP_log("Doppler DPU processing for frame %d done\n",i);
    } /* end of frame loop */

    DebugP_log("Test #%d finished!\n\r", testCount);
    testCount++;
        
    DebugP_log("\n");

exit:

    fclose(fileId);
    fclose(fileId_write);
    fclose(fileId_read);
    if(endOfFile==true)
    {
        DebugP_log("The input binary file is not in the right format!\n");
    }

    if(status == SystemP_SUCCESS)
    {
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

static uint32_t HWAFFT_log2Approx(uint32_t x)
{
    uint16_t idx;
    if(x < 2U)
    {
        idx = 0U;
        
    }
    for(idx=1;idx<10;++idx)
    {
        if(x<=pow(2,idx))
        {
            break;
        }
    }
    return idx; 
}
