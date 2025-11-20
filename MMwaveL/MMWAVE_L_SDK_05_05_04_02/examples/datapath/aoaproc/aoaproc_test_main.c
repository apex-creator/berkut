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
 *  This example performs aoaProc DPU test.
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
#include <datapath/dpu/aoaproc/v0/aoaprochwa.h>
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

#define DPC_OBJDET_EDMA_SHADOW_BASE                                          SOC_EDMA_NUM_DMACH
#define DPC_OBJDET_HWA_WINDOW_RAM_OFFSET                                     0

/* AoA DPU */
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_CH_PING                        EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ10
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_CH_PONG                        EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ11
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SHADOW_PING                    (DPC_OBJDET_EDMA_SHADOW_BASE + 0)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SHADOW_PONG                    (DPC_OBJDET_EDMA_SHADOW_BASE + 1)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_EVENT_QUE_PING                 0
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_EVENT_QUE_PONG                 0

#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_CH_PING                    EDMA_APPSS_TPCC_B_EVT_FREE_0
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_SHADOW_PING                (DPC_OBJDET_EDMA_SHADOW_BASE + 2)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_CH_PONG                    EDMA_APPSS_TPCC_B_EVT_FREE_1
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_SHADOW_PONG                (DPC_OBJDET_EDMA_SHADOW_BASE + 3)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_EVENT_QUE_PING             0
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_EVENT_QUE_PONG             0

#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_CH_PING                            EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ2
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_CH_PONG                            EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ3
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SHADOW_PING                        (DPC_OBJDET_EDMA_SHADOW_BASE + 4)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SHADOW_PONG                        (DPC_OBJDET_EDMA_SHADOW_BASE + 5)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_EVENT_QUE_PING                     0
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_EVENT_QUE_PONG                     0

#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_CH_PING                        EDMA_APPSS_TPCC_B_EVT_FREE_2
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_SHADOW_PING                    (DPC_OBJDET_EDMA_SHADOW_BASE + 6)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_CH_PONG                        EDMA_APPSS_TPCC_B_EVT_FREE_3
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_SHADOW_PONG                    (DPC_OBJDET_EDMA_SHADOW_BASE + 7)
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_EVENT_QUE_PING                 0
#define DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_EVENT_QUE_PONG                 0

#define DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PING_CH                        EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ12
#define DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PING_SHADOW                    (DPC_OBJDET_EDMA_SHADOW_BASE + 8)
#define DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PING_EVENT_QUE                 0

#define DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PONG_CH                        EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ13
#define DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PONG_SHADOW                    (DPC_OBJDET_EDMA_SHADOW_BASE + 9)
#define DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PONG_EVENT_QUE                 0


#define MMW_DEMO_TEST_1DFFT_BUFF_SIZE                                        MAX_NUM_CHIRPS_PERFRAME * MAX_NUM_RANGEBIN * MAX_NUM_TX_ANTENNA * MAX_NUM_RX_ANTENNA
#define DPC_DPU_AOAPROC_DOPPLERFFT_WINDOW_TYPE                               MATHUTILS_WIN_HANNING
#define DPC_OBJDET_QFORMAT_DOPPLER_FFT                                       17

#define MAX_NUM_RANGE_BINS 40 //This is the range bin length kept for K2O
#define MAX_NUM_POINTS     50
#define NUM_ELEVATION_ELEMENTS 2
#define NUM_AZIMUTH_ELEMENTS 4

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))

FILE * fileId, *fileId2;
HWA_Handle              gHwaHandle;
DPU_AoAProcHWA_Handle   aoaProcDpuHandle;
DPU_AoAProcHWA_Config   aoaProcDpuCfg;

uint8_t testFeatureStatus;

uint8_t checkResultFlag;

uint8_t finalResults ;

Edma_IntrObject     intrObj;


uint32_t rangeFFT[MMW_DEMO_TEST_1DFFT_BUFF_SIZE] __attribute__((section(".l3"), aligned(32)));// Doppler samples buffer when reading 1D-FFT data from file
uint32_t window2DCoef[MAX_NUM_CHIRPS_PERFRAME] __attribute__((section(".data"), aligned(32)));// Array for storing the window coefficients
cmplx32ImRe_t vectMult[MAX_NUM_CHIRPS_PERFRAME] __attribute__((section(".data"), aligned(32)));// Array for storing the Vector Multiplication Coefficients for Doppler Compensation
uint32_t maxaoa_index[MAX_NUM_POINTS] __attribute__((section(".data"), aligned(32)));// Array for storing the max of elevation-azimuth heatmap after AoA processing

cmplx32ImRe_t angleMatrix[MAX_NUM_POINTS*NUM_AZIMUTH_ELEMENTS*NUM_ELEVATION_ELEMENTS] __attribute__((section(".l3"), aligned(32)));// Temporary array to store the antenna array on which AoA processing is to be done
uint16_t rangeGatesCount[MAX_NUM_POINTS] __attribute__((section(".data")));// Variable to store the range gates count based on MAX_NUM_POINTS in the detection Matrix
uint16_t angleIndexDopplerCnt[MAX_NUM_RANGE_BINS] __attribute__((section(".data")));// Array to store the number of MAX POINTS for a range index
uint16_t rangeIndexArray[MAX_NUM_RANGE_BINS] __attribute__((section(".data")));// Array to store the range indices on which Doppler is to be applied
int16_t dopplerIndexArray[MAX_NUM_POINTS*MAX_NUM_RANGE_BINS] __attribute__((section(".data")));// Array to store Doppler index corresponding to range index for which Angle FFT is to be applied
cmplx32ImRe_t antenna_array[NUM_AZIMUTH_ELEMENTS*NUM_ELEVATION_ELEMENTS*MAX_NUM_POINTS] __attribute__((section(".data")));// Temporary array to store the antenna array based on antenna array arrangement

uint16_t elev_index_gt[MAX_NUM_POINTS] __attribute__((section(".l3"), aligned(32)));// Array to store the ground-truth elevation index 
uint16_t azim_index_gt[MAX_NUM_POINTS] __attribute__((section(".l3"), aligned(32)));// Array to store the ground-truth azimuth index 
uint16_t elev_index[MAX_NUM_POINTS] __attribute__((section(".l3"), aligned(32)));// Array to store the calculated elevation index
uint16_t azim_index[MAX_NUM_POINTS] __attribute__((section(".l3"), aligned(32)));// Array to store the calculated azimuth index

uint16_t AoANumLoops[1] __attribute__((section(".data")));
uint8_t DopplerCompensationEnabled[1] __attribute__((section(".data")));

typedef struct aoaProcTestConfig_t_ {
    uint32_t numTxAntennas;
    uint32_t numRxAntennas;
    uint32_t numVirtualAntennas;
    uint32_t numRangeBins;
    uint32_t numDopplerChirps;
    uint32_t numFrames;
    uint8_t rangeGatesCount;   
} aoaProcTestConfig_t;

static uint32_t HWAFFT_log2Approx(uint32_t x);

/**
*  @b Description
*  @n
*    Reference function to make sure the computed elevation and azimuth indices match with MATLAB model
*/
void compare_ip_op()
{
    uint8_t i, errcnt = 0; 

    for(i=0;i<MAX_NUM_POINTS;++i)
    {
        elev_index[i] = ceil((maxaoa_index[i]*1.0+1)/aoaProcDpuCfg.staticCfg.azimuthFFTSize)-1;
        azim_index[i] = (maxaoa_index[i]+1)%aoaProcDpuCfg.staticCfg.azimuthFFTSize-1;
        if((elev_index[i]!=elev_index_gt[i]) || (azim_index[i]!=azim_index_gt[i]))
        {
            ++errcnt;
            DebugP_log ("At %d th index difference: elev_idx=%d , azim_idx=%d \n", i, elev_index_gt[i]-elev_index[i], azim_index_gt[i]-azim_index[i]);
        }
    }
    DebugP_log ("The number of points differing from %d points are: %d points \n", MAX_NUM_POINTS, errcnt);
}

/**
*  @b Description
*  @n
*    Based on the test configuration, set up the AoA processing DPU configurations
*/
void Test_setProfile(aoaProcTestConfig_t * testConfig)
{
    DPU_AoAProcHWA_HW_Resources  *pHwConfig;
    DPU_AoAProcHWA_StaticConfig  *params;
    DPU_AoAProcHWA_maxDetMatrix_Resources *detMatrix_res;
    
    pHwConfig = &aoaProcDpuCfg.hwRes; 
    params = &aoaProcDpuCfg.staticCfg;
    detMatrix_res = &aoaProcDpuCfg.maxDetmatrix_res;
    
    /* overwrite the DPIF_commonParams with the test configuration*/
    params->numTxAntennas = testConfig->numTxAntennas;
    params->numRxAntennas = testConfig->numRxAntennas;
    params->numVirtualAntennas = testConfig->numTxAntennas * testConfig->numRxAntennas;
    params->numRangeBins = testConfig->numRangeBins;
    params->numDopplerChirps = testConfig->numDopplerChirps;
    params->log2NumDopplerBins = HWAFFT_log2Approx(params->numDopplerChirps);
    params->numDopplerBins = pow(2,params->log2NumDopplerBins);
    params->doppFFT_is16b = 0; //Takes into account of 16-bit complex Doppler FFT
    params->elevationFFTSize = 32; //Via CLI
    params->azimuthFFTSize = 32; //Via CLI
    params->log2NumElevationBins = HWAFFT_log2Approx(params->elevationFFTSize);
    params->log2NumAzimuthBins = HWAFFT_log2Approx(params->azimuthFFTSize);

    /* windowing */
    pHwConfig->hwaCfg.windowSize = sizeof(uint32_t) * ((params->numDopplerBins + 1) / 2); //symmetric window, for real samples
    pHwConfig->hwaCfg.vectorRamSize = sizeof(cmplx32ImRe_t)*params->numDopplerBins;
    
    /* radar cube after 1D-FFT */
    pHwConfig->radar_1D_FFT_Cube.dataSize = testConfig->numRangeBins * testConfig->numTxAntennas * testConfig->numRxAntennas * sizeof(cmplx16ReIm_t) * params->numDopplerChirps;
    pHwConfig->radar_1D_FFT_Cube.datafmt = DPIF_RADARCUBE_FORMAT_6;

    /* Antenna Array elements and max num of points for the test case*/
    detMatrix_res->antenna_array_elements = NUM_AZIMUTH_ELEMENTS*NUM_ELEVATION_ELEMENTS;
    detMatrix_res->max_num_points = MAX_NUM_POINTS; //Via CLI

    params->numCols_Antenna = NUM_AZIMUTH_ELEMENTS;
    params->numRows_Antenna = NUM_ELEVATION_ELEMENTS;

    pHwConfig->hwaCfg.AoAhwaNumLoops[0] = MAX_NUM_POINTS/2 + MAX_NUM_POINTS%2;
    pHwConfig->hwaCfg.isDopplerPhaseCompensationEnabled[0]=1;

    /* Data Input EDMA Doppler*/
    pHwConfig->edmaCfg.edmaIn_rangeFFT.ping.channel        = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_CH_PING;
    pHwConfig->edmaCfg.edmaIn_rangeFFT.pong.channel        = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_CH_PONG;
    pHwConfig->edmaCfg.edmaIn_rangeFFT.ping.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SHADOW_PING;
    pHwConfig->edmaCfg.edmaIn_rangeFFT.pong.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaIn_rangeFFT.ping.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaIn_rangeFFT.pong.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_EVENT_QUE_PONG;

    /* Hot signature EDMA Doppler*/
    pHwConfig->edmaCfg.edmaHotSigDoppler.ping.channel    = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_CH_PING;
    pHwConfig->edmaCfg.edmaHotSigDoppler.pong.channel    = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_CH_PONG;
    pHwConfig->edmaCfg.edmaHotSigDoppler.ping.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_SHADOW_PING;
    pHwConfig->edmaCfg.edmaHotSigDoppler.pong.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaHotSigDoppler.ping.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaHotSigDoppler.pong.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_DOPPLER_SIG_EVENT_QUE_PING;

    /* Data Input EDMA AoA*/
    pHwConfig->edmaCfg.edmaIn_dopplerFFT.ping.channel        = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_CH_PING;
    pHwConfig->edmaCfg.edmaIn_dopplerFFT.pong.channel        = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_CH_PONG;
    pHwConfig->edmaCfg.edmaIn_dopplerFFT.ping.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SHADOW_PING;
    pHwConfig->edmaCfg.edmaIn_dopplerFFT.pong.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaIn_dopplerFFT.ping.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaIn_dopplerFFT.pong.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_EVENT_QUE_PONG;

    /* Hot signature EDMA AoA*/
    pHwConfig->edmaCfg.edmaHotSigAoA.ping.channel    = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_CH_PING;
    pHwConfig->edmaCfg.edmaHotSigAoA.pong.channel    = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_CH_PONG;
    pHwConfig->edmaCfg.edmaHotSigAoA.ping.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_SHADOW_PING;
    pHwConfig->edmaCfg.edmaHotSigAoA.pong.channelShadow  = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaHotSigAoA.ping.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaHotSigAoA.pong.eventQueue     = DPC_OBJDET_DPU_AOAPROC_EDMAIN_AOA_SIG_EVENT_QUE_PING;
    pHwConfig->intrObj = &intrObj;

    /* Data Output EDMA */
    pHwConfig->edmaCfg.edmaOut_maxAoA.ping.channel = DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PING_CH;
    pHwConfig->edmaCfg.edmaOut_maxAoA.pong.channel = DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PONG_CH;
    pHwConfig->edmaCfg.edmaOut_maxAoA.ping.channelShadow = DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PING_SHADOW;
    pHwConfig->edmaCfg.edmaOut_maxAoA.pong.channelShadow = DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PONG_SHADOW;
    pHwConfig->edmaCfg.edmaOut_maxAoA.ping.eventQueue = DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PING_EVENT_QUE;
    pHwConfig->edmaCfg.edmaOut_maxAoA.pong.eventQueue = DPC_OBJDET_DPU_AOAPROC_EDMAOUT_MAXAOA_PONG_EVENT_QUE;
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
                        DPC_DPU_AOAPROC_DOPPLERFFT_WINDOW_TYPE,
                        DPC_OBJDET_QFORMAT_DOPPLER_FFT);
}

/**
*  @b Description
*  @n
*    To generate the Vector Multiplication table for Doppler compensation
*/
void Test_VectorMult_gen(uint32_t numDopplerChirps)
{   
    int32_t amp,i;
    float t1,t2,coeff;
    int16_t numDopplerBins = pow(2,HWAFFT_log2Approx(numDopplerChirps));

    t1 = 42*pow(10,-6); //time between chirp1 Tx1 and chirp1 tx2 : Via CLI
    t2 = 200*pow(10,-6);//time between chirp1 Tx1 and chirp2 tx1 : Via CLI
    amp = pow(2,20)-1;//21 bits quantization

    for(i=0;i<numDopplerBins;++i)
    {
        if(i<=64)
        {
            coeff = (2*PI_*t1*i)/(t2*numDopplerBins);
        }
        else
        {
            coeff = (2*PI_*t1*(i-numDopplerBins))/(t2*numDopplerBins);
        }
        vectMult[i].real = floor(amp*sin(coeff));
        vectMult[i].imag = floor(amp*cos(coeff));
    }
}

/**
*  @b Description
*  @n
*    Function to initialize the HWA used in this AoA test
*/
void aoaProcDpuTest_hwaInit()
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
*    Function to initialize aoa DPU
*/
void aoaProcDpuTest_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_AoAProcHWA_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    aoaProcDpuHandle =  DPU_AoAProcHWA_init (&initParams, &errorCode);
    if (aoaProcDpuHandle == NULL)
    {
        DebugP_log ("Debug: AoA Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*        set up the AoA processing DPU configurations, do not change for each test 
*/
void aoaProcDpuTest_dpuCfg()
{
    DPU_AoAProcHWA_HW_Resources *pHwConfig;
    
    memset((void *)&aoaProcDpuCfg, 0, sizeof(DPU_AoAProcHWA_Config));
    /* hwi configuration */
    pHwConfig = &aoaProcDpuCfg.hwRes; 
        
    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = 0;
    pHwConfig->hwaCfg.winRamOffset  = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET; 
    pHwConfig->hwaCfg.winSym = 1;
    pHwConfig->hwaCfg.vecRamOffset = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET;
    pHwConfig->hwaCfg.hwaDmaTriggerDoppPing = 0;
    pHwConfig->hwaCfg.hwaDmaTriggerDoppPong = 1;
    pHwConfig->hwaCfg.hwaDmaTriggerAoAPing = 2;
    pHwConfig->hwaCfg.hwaDmaTriggerAoAPong = 3;


    /* edma configuration */
    pHwConfig->edmaCfg.edmaHandle  = gEdmaHandle[0];   
    
    /* windowing buffer is fixed, size will change */
    aoaProcDpuCfg.hwRes.hwaCfg.window =  (int32_t *)&window2DCoef[0];

    /* Vector RAM buffer is fixed, size will change */
    aoaProcDpuCfg.hwRes.hwaCfg.vectorRam = (cmplx32ImRe_t *)&vectMult[0];

    aoaProcDpuCfg.hwRes.hwaCfg.AoAhwaNumLoops = (uint16_t *)&AoANumLoops[0];

    aoaProcDpuCfg.hwRes.hwaCfg.isDopplerPhaseCompensationEnabled = (uint8_t *)&DopplerCompensationEnabled[0];

    /* radar 1D-FFT cube */
    aoaProcDpuCfg.hwRes.radar_1D_FFT_Cube.data = (void *)rangeFFT;    
    
    /* radar angle Matrix after 2D-FFT */
    aoaProcDpuCfg.hwRes.angleMat.data  = (void *)angleMatrix;

    /* max of angle Matrix */
    aoaProcDpuCfg.hwRes.maxVal_elev_azim = (uint32_t *)&maxaoa_index[0];

    aoaProcDpuCfg.maxDetmatrix_res.rangeGatesCount = (uint16_t *)&rangeGatesCount[0];

    /* range index array */
    aoaProcDpuCfg.maxDetmatrix_res.range_idx_arr = (uint16_t *)&rangeIndexArray[0];

    /* Angle index Doppler Count array */
    aoaProcDpuCfg.maxDetmatrix_res.angle_idx_doppler_count_arr = (uint16_t *)&angleIndexDopplerCnt[0];

    /* Angle where Doppler index coresponding to max of range gate (where FFT is taken) is stored */
    aoaProcDpuCfg.maxDetmatrix_res.doppler_idx_arr = (int16_t *)&dopplerIndexArray[0];

    /*Array to store the antenna array elements before azimuth-elevation FFT */
    aoaProcDpuCfg.maxDetmatrix_res.antenna_array = (cmplx32ImRe_t *)&antenna_array[0];
}


/**
*  @b Description
*  @n
*    main function from which AoA processing test-case begins
*/
void aoaproc_test_main(void *args)
{
    volatile uint32_t            baseAddr, regionId, startCycle1, startCycle2, cycleCount1, cycleCount2;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i,j;
    uint32_t            testCount=0;
    int32_t             retVal = 0;
    aoaProcTestConfig_t testConfig = {0U};
    bool                endOfFile = false;
    uint32_t            rangeFFT_size_perChirp;
    uint32_t            numRead_1DFFT_samples;

    DPU_AoAProcHWA_OutParams outParms;

    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    DebugP_log("DPU Test Started \r\n");
    
    SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT|SOC_RCM_MEMINIT_TPCCA_INIT|SOC_RCM_MEMINIT_TPCCB_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT);

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Initialize the HWA for AoA DPU unit-testing */
    aoaProcDpuTest_hwaInit();
    /* Initialize the AoA DPU for unit-testing */
    aoaProcDpuTest_dpuInit();
    /* config the AoA proc DPU */
    aoaProcDpuTest_dpuCfg();

    /* Enable and reset CPU cycle counter */
    CycleCounterP_reset();

    /* start the test by reading in the range FFT binary file*/
    fileId = fopen("..\\..\\..\\..\\common\\testBench\\aoaproc\\rangeFFT_data_final2.bin", "rb");
    fileId2 = fopen("..\\..\\..\\..\\common\\testBench\\aoaproc\\max_data_final_50pts.bin", "rb");
    if (fileId == NULL)
    {
        printf("Error:  Cannot open rangeFFT_data.bin !\n");
        exit(0);
    }
    if (fileId2 == NULL)
    {
        printf("Error:  Cannot open max_data.bin !\n");
        exit(0);
    }    

    /* read in test config */
    fread(&testConfig.numTxAntennas, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numRxAntennas, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numRangeBins, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numDopplerChirps, sizeof(uint16_t),1,fileId);
    fread(&testConfig.numFrames, sizeof(uint16_t),1,fileId);

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

    /* Generate Vector Multiplication Table for Doppler Compensation */
    Test_VectorMult_gen(testConfig.numDopplerChirps);
    
    testFeatureStatus = 1;    
    
    Test_setProfile(&testConfig);

    checkResultFlag = 1;

    /* Read rangeFFT data and put in HWA_SS_RAM */
    for(i = 0; i < testConfig.numFrames; ++i)
    {   

        fread((uint16_t *)&aoaProcDpuCfg.maxDetmatrix_res.rangeGatesCount[0], sizeof(uint16_t),1,fileId2);
        fread((uint16_t *)&aoaProcDpuCfg.maxDetmatrix_res.range_idx_arr[0], sizeof(uint16_t),MAX_NUM_RANGE_BINS,fileId2);
        fread((uint16_t *)&aoaProcDpuCfg.maxDetmatrix_res.angle_idx_doppler_count_arr[0], sizeof(uint16_t),MAX_NUM_RANGE_BINS,fileId2);
        fread((uint16_t *)&aoaProcDpuCfg.maxDetmatrix_res.doppler_idx_arr[0], sizeof(uint16_t),MAX_NUM_RANGE_BINS*MAX_NUM_POINTS,fileId2);
        retVal = DPU_AoAProcHWA_config(aoaProcDpuHandle, &aoaProcDpuCfg);
        if(retVal < 0)
        {
            DebugP_log("DEBUG: DOPPLER DPU config return error:%d \n", retVal);
            DebugP_assert(0);
        }
        DebugP_log("Frame %d data read started\n",i);
        for(j=0;j<testConfig.numDopplerChirps;++j)
        {
            numRead_1DFFT_samples = fread(((uint16_t *)&rangeFFT[0]+j*rangeFFT_size_perChirp), sizeof(uint16_t),  rangeFFT_size_perChirp, fileId); //Take in cmplx16ImRe_t samples after 1D-FFT
            if (numRead_1DFFT_samples != rangeFFT_size_perChirp)
            {
                endOfFile = true;
                goto exit;
            }
            if(j%10 == 0)
            {
                DebugP_log("Reading 1D-FFT Data From File.. %d percent\n", j * 100 / testConfig.numDopplerChirps);
            }
        }/* end of chirp loop */
        fread((uint16_t *)&azim_index_gt[0], sizeof(uint16_t), MAX_NUM_POINTS, fileId2);
        fread((uint16_t *)&elev_index_gt[0], sizeof(uint16_t), MAX_NUM_POINTS, fileId2);

        memset((void *)&outParms, 0, sizeof(DPU_AoAProcHWA_OutParams));
        startCycle1 = CycleCounterP_getCount32();
        retVal = DPU_AoAProcHWA_Dopplerprocess(aoaProcDpuHandle, &outParms);
        cycleCount1 = CycleCounterP_getCount32();
        if(retVal < 0)
        {
            DebugP_log("DEBUG: DOPPLER DPU process return error:%d \n", retVal);
            DebugP_assert(0);
        }
        DebugP_log("Doppler processing for frame %d done\n",i);
        memset((void *)&outParms, 0, sizeof(DPU_AoAProcHWA_OutParams));
        startCycle2 = CycleCounterP_getCount32();
        retVal = DPU_AoAProcHWA_AoAprocess(aoaProcDpuHandle, &outParms);
        cycleCount2 = CycleCounterP_getCount32();
        if(retVal < 0)
        {
            DebugP_log("DEBUG: DOPPLER DPU process return error:%d \n", retVal);
            DebugP_assert(0);
        }
        DebugP_log("AoA processing for frame %d done\n",i);
        DebugP_log("Measured processing time (AoAProc-Doppler) = CPU cycles = %d !!!\r\n", cycleCount1-startCycle1);
        DebugP_log("Measured processing time (AoAProc-AoA) = CPU cycles = %d !!!\r\n", cycleCount2-startCycle2);
        if(MATLAB_MODEL_BITMATCHING)
        {
            compare_ip_op();
        }


    } /* end of frame loop */

    DebugP_log("Test #%d finished!\n\r", testCount);
    testCount++;
        
    DebugP_log("\n");

exit:

    fclose(fileId);
    fclose(fileId2);
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
