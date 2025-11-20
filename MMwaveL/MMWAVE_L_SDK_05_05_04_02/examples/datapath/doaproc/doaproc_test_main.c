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
 *  This example performs doaProc DPU test.
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
#include <datapath/dpu/doaproc/v0/doaproc.h>
/* MATH utils library Include files */
#include <utils/mathutils/mathutils.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define EDMA_TEST_EVT_QUEUE_NO      (0U)
#define MAX_NUM_TX_ANTENNA          (2U)
#define MAX_NUM_RX_ANTENNA          (3U)
#define MAX_NUM_RANGEBIN            (64U)
#define MAX_NUM_ADCSAMPLE_PERCHIRP  (128U)
#define MAX_NUM_CHIRPS_PERFRAME     (64U)
#define MAX_AZ_FFT_SIZE             (64U)

#define DPC_OBJDET_EDMA_SHADOW_BASE                                    SOC_EDMA_NUM_DMACH
#define DPC_OBJDET_HWA_WINDOW_RAM_OFFSET                               0


/* DoA DPU */
#define DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_CH                         EDMA_APPSS_TPCC_B_EVT_FREE_0
#define DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_SHADOW                     (DPC_OBJDET_EDMA_SHADOW_BASE + 11)
#define DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_EVENT_QUE                  0

#define DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_CH                         EDMA_APPSS_TPCC_B_EVT_FREE_1
#define DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_SHADOW                     (DPC_OBJDET_EDMA_SHADOW_BASE + 12)
#define DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_EVENT_QUE                  0

#define DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_CH                        EDMA_APPSS_TPCC_B_EVT_FREE_2
#define DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_SHADOW                    (DPC_OBJDET_EDMA_SHADOW_BASE + 13)
#define DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_EVENT_QUE                 0

#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_CH                  EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ0
#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_SHADOW              (DPC_OBJDET_EDMA_SHADOW_BASE + 14)
#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_EVENT_QUE           0

#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_CH              EDMA_APPSS_TPCC_B_EVT_FREE_3
#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_SHADOW          (DPC_OBJDET_EDMA_SHADOW_BASE + 15)
#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_EVENT_QUE       0

#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_CH               EDMA_APPSS_TPCC_B_EVT_FREE_4
#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_SHADOW           (DPC_OBJDET_EDMA_SHADOW_BASE + 16)
#define DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_EVENT_QUE        0

#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_CH          EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ1
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_SHADOW      (DPC_OBJDET_EDMA_SHADOW_BASE + 17)
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_EVENT_QUE   0

#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_CH                      EDMA_APPSS_TPCC_B_EVT_FREE_5
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_SHADOW                  (DPC_OBJDET_EDMA_SHADOW_BASE + 18)
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_EVENT_QUE               0

#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_CH                EDMA_APPSS_TPCC_B_EVT_FREE_6
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_SHADOW            (DPC_OBJDET_EDMA_SHADOW_BASE + 19)
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_EVENT_QUE         0

#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_CH             EDMA_APPSS_TPCC_B_EVT_FREE_7
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_SHADOW         (DPC_OBJDET_EDMA_SHADOW_BASE + 20)
#define DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_EVENT_QUE      0

#define  DPC_OBJDET_DPIF_RADARCUBE_FORMAT_6                            6
#define DPC_DPU_DPIF_DETMATRIX_FORMAT_2                                2

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))

FILE * fileId;
FILE * fileId_read;
HWA_Handle              gHwaHandle;
DPU_DoaProc_Handle doaProcDpuHandle;
DPU_DoaProc_Config doaProcDpuCfg;

uint8_t testFeatureStatus;

uint8_t finalResults ;

Edma_IntrObject     intrObj;

int16_t radarCubeData[MAX_NUM_ADCSAMPLE_PERCHIRP * MAX_NUM_CHIRPS_PERFRAME * 6] __attribute((section(".l3")));
uint32_t detMatrixData[MAX_NUM_RANGEBIN * MAX_AZ_FFT_SIZE] __attribute((section(".l3")));
uint32_t detMatrixRef[MAX_NUM_RANGEBIN * MAX_AZ_FFT_SIZE] __attribute((section(".l3")));
uint8_t dopplerIndexMatrixData[MAX_NUM_RANGEBIN * MAX_AZ_FFT_SIZE] __attribute((section(".l3")));
uint8_t elevationIndexMatrixData[MAX_NUM_RANGEBIN * MAX_AZ_FFT_SIZE] __attribute((section(".l3")));
int32_t windowBuffer[MAX_AZ_FFT_SIZE] __attribute((section(".l3")));
//cmplx32ImRe_t interLoopDataBuffer[2 * MAX_AZ_FFT_SIZE * MAX_NUM_CHIRPS_PERFRAME] __attribute((section(".adcbuf")));
uint32_t interLoopDataBufferMem[2 * MAX_AZ_FFT_SIZE * MAX_NUM_CHIRPS_PERFRAME] __attribute((section(".l3")));


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
    uint32_t  azimuthFftSize;
   // uint32_t numDopplerBins;
    uint8_t  cmplxIQswapFlag;    
    uint8_t  rxChanInterleave;
    uint8_t  radarCubeLayoutFmt;
} rangeProcTestConfig_t;

typedef struct gAntGeo_t_ {
    uint32_t row;
    uint32_t col;
} gAntGeo_t;

int32_t checkTestResult(uint32_t *testOut, uint32_t *refOut, int32_t numSamples)
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

int32_t MmwDemo_cfgDopplerParamMapping(DPU_DoaProc_HWA_Option_Cfg *dopplerParamCfg, uint32_t mappingOption)
{
    int32_t ind, indNext, indNextPrev;
    int32_t row, col;
    int32_t dopParamInd;
    int32_t state;
    int16_t BT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int16_t DT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int16_t SCAL[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int8_t  DONE[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int32_t retVal = 0;
    int32_t rowOffset;
    gAntGeo_t  AntGeo[6];
    AntGeo[0].row = 0; 
    AntGeo[0].col = 0;
    AntGeo[1].row = 1;
    AntGeo[1].col = 1;
    AntGeo[2].row = 0;
    AntGeo[2].col = 2;
    AntGeo[3].row = 0;
    AntGeo[3].col = 1;
    AntGeo[4].row = 1;
    AntGeo[4].col = 2;
    AntGeo[5].row = 0;
    AntGeo[5].col = 3;
    

    if (mappingOption == 0)
    {
        /*For AOA DPU, Output is */
        rowOffset =  4;
    }
    else if (mappingOption == 1)
    {
        rowOffset =  5 * 4;
    }
    else
    {
        retVal = -2884;
        goto exit;
    }

    /* Initialize tables */
    for (ind = 0; ind < (2 * 4); ind++)
    {
        BT[ind] = 0;
        SCAL[ind] = 0;
        DONE[ind] = 0;
    }

    for (ind = 0; ind < (2 * 3); ind++)
    {
        row = AntGeo[ind].row;
        col = AntGeo[ind].col;
        BT[row * 4 + col] = ind;
        SCAL[row * 4 + col] = 1;
    }
    for (row = 0; row < 2; row++)
    {
        for (col = 0; col < 4; col++)
        {
            ind = row * 4 + col;
            DT[ind] = row * rowOffset + col;
        }
    }


    /* Configure Doppler HWA mapping params for antenna mapping */
    dopParamInd = 0;
    dopplerParamCfg->numDopFftParams = 0;
    for (ind = 0; ind < (2 * 4); ind++)
    {
        if (!DONE[ind])
        {
            if(dopParamInd == DPU_DOA_PROC_MAX_NUM_DOP_FFFT_PARAMS)
            {
                retVal = -2884;
                goto exit;
            }

            DONE[ind] = 1;
            dopplerParamCfg->numDopFftParams++;
            dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt = 1;
            dopplerParamCfg->dopFftCfg[dopParamInd].scale = SCAL[ind];
            if (dopplerParamCfg->dopFftCfg[dopParamInd].scale == 0)
            {
                dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = 0;
            }
            else
            {
                dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = BT[ind];
            }
            dopplerParamCfg->dopFftCfg[dopParamInd].dstAddrOffset = DT[ind];
            state = 1;//STATE_SECOND:
            for (indNext = ind+1; indNext < (2 * 4); indNext++)
            {

                if (!DONE[indNext] && (dopplerParamCfg->dopFftCfg[dopParamInd].scale == SCAL[indNext]))
                {
                    switch (state)
                    {
                        case 1://STATE_SECOND:
                            dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                            DONE[indNext] = 1;
                            if (SCAL[indNext] == 1)
                            {
                                dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = BT[indNext] - dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset;
                            }
                            else
                            {
                                dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = 0;
                            }
                            dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx = DT[indNext] - DT[ind];
                            indNextPrev = indNext;
                            state = 2;//STATE_NEXT:
                            break;
                        case 2://STATE_NEXT:
                            if (SCAL[indNext] == 1)
                            {
                                if ((dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx == (BT[indNext] - BT[indNextPrev])) &&
                                    (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev])))
                                {
                                    DONE[indNext] = 1;
                                    dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                    indNextPrev = indNext;
                                }
                            }
                            else
                            {
                                if (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev]))
                                {
                                    DONE[indNext] = 1;
                                    dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                    indNextPrev = indNext;
                                }
                            }
                            break;
                    }
                }
            }
            dopParamInd++;
        }
    }

    dopplerParamCfg->numDopFftParams = dopParamInd;

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*    Based on the test configuration, set up the range processing DPU configurations
*/
int32_t Test_setProfile(rangeProcTestConfig_t * testConfig)
{
    DPIF_RadarCube radarCube;
    DPIF_RadarCube radarCubeMinMot;
    DPIF_DetMatrix detMatrix[2];
    DPIF_DetMatrix dopplerIndexMatrix;
    DPIF_DetMatrix elevationIndexMatrix;
    DPU_DoaProc_EdmaCfg *edmaCfg;
    DPU_DoaProc_HwaCfg *hwaCfg;
    int32_t winGenLen, i;
    int32_t retVal = 0;
    DPU_DoaProc_HW_Resources  *hwRes;
    DPU_DoaProc_StaticConfig  *doaStaticCfg;
    
    hwRes = &doaProcDpuCfg.hwRes; 
    doaStaticCfg = &doaProcDpuCfg.staticCfg; 
    edmaCfg = &hwRes->edmaCfg;
    hwaCfg = &hwRes->hwaCfg;
    
    /* overwrite the DPIF_commonParams with the test configuration*/
    doaStaticCfg->numTxAntennas = 2; //testConfig->numTxAntennas;
    doaStaticCfg->numRxAntennas = 3; //staticCfg->ADCBufData.dataProperty.numRxAntennas;
    doaStaticCfg->numVirtualAntennas = 6; //testConfig->numTxAntennas * testConfig->numRxAntennas;
    doaStaticCfg->numRangeBins = testConfig->numAdcSamples/2; //testConfig->numRangeBins;
    doaStaticCfg->numAntRow = 2;
    doaStaticCfg->numAntCol = 4;
    doaStaticCfg->numDopplerChirps   = testConfig->numChirpsPerFrame/doaStaticCfg->numTxAntennas; //staticCfg->numDopplerChirpsPerProc;
    doaStaticCfg->numMinorMotionChirpsPerFrame = 0; //staticCfg->numMinorMotionChirpsPerFrame; //   staticCfg->numDopplerChirps / staticCfg->numFramesPerMinorMotProc;
    doaStaticCfg->numFrmPerMinorMotProc = 0;
    doaStaticCfg->numDopplerBins     = mathUtils_pow2roundup(doaStaticCfg->numDopplerChirps); //staticCfg->numDopplerBins;
    doaStaticCfg->log2NumDopplerBins = mathUtils_ceilLog2(doaStaticCfg->numDopplerBins);
    
    doaStaticCfg->selectCoherentPeakInDopplerDim = 0; //staticCfg->selectCoherentPeakInDopplerDim;
    doaStaticCfg->angleDimension        = 2; //staticCfg->angleDimension;
    doaStaticCfg->isDetMatrixLogScale   = false; //staticCfg->isDetMatrixLogScale;
    doaStaticCfg->azimuthFftSize        = testConfig->azimuthFftSize; //staticCfg->azimuthFftSize;
    doaStaticCfg->elevationFftSize      = 2; //staticCfg->elevationFftSize;
    doaStaticCfg->isStaticClutterRemovalEnabled = 1; //dynCfg->staticClutterRemovalCfg.enabled;
    doaStaticCfg->isRxChGainPhaseCompensationEnabled   = 0; //staticCfg->isRxChGainPhaseCompensationEnabled;
    doaStaticCfg->doaRangeLoopType = 0; //staticCfg->doaRangeLoopType;
    doaStaticCfg->enableMajorMotion = 1;

    retVal = MmwDemo_cfgDopplerParamMapping(&hwaCfg->doaRngGateCfg, 0);
    if (retVal < 0)
    {
        goto exit;
    }

    /* L3 allocations */
    /* L3 - radar cube */
    if (doaStaticCfg->enableMajorMotion)
    {
        radarCube.dataSize = doaStaticCfg->numRangeBins * (testConfig->numChirpsPerFrame/doaStaticCfg->numTxAntennas) *
                             doaStaticCfg->numVirtualAntennas * sizeof(cmplx16ReIm_t);
        radarCube.data = (void*)radarCubeData;
        if (radarCube.data == NULL)
        {
            retVal = -1;
            goto exit;
        }
        radarCube.datafmt = DPC_OBJDET_DPIF_RADARCUBE_FORMAT_6;
    }
    else
    {
        radarCube.dataSize = 0;
        radarCube.data = NULL;
        radarCube.datafmt = DPC_OBJDET_DPIF_RADARCUBE_FORMAT_6;
    }
    
    /* L3 - radar cube for minor motion detection */
    if (doaStaticCfg->enableMajorMotion == 0)
    {
        radarCubeMinMot.dataSize = doaStaticCfg->numRangeBins * (testConfig->numChirpsPerFrame/doaStaticCfg->numTxAntennas) *
                             doaStaticCfg->numVirtualAntennas * sizeof(cmplx16ReIm_t);
        radarCubeMinMot.data = (void*)radarCubeData;
        if (radarCubeMinMot.data == NULL)
        {
            retVal = -2;
            goto exit;
        }
        radarCubeMinMot.datafmt = DPC_OBJDET_DPIF_RADARCUBE_FORMAT_6;
    }
    else
    {
        radarCubeMinMot.dataSize = 0;
        radarCubeMinMot.data = NULL;
        radarCubeMinMot.datafmt = DPC_OBJDET_DPIF_RADARCUBE_FORMAT_6;
    }
    
    /* L3 - Detection Matrix */
    for (i=0; i<2; i++)
    {
        if (((i==0) && doaStaticCfg->enableMajorMotion) || ((i==1) && (doaStaticCfg->enableMajorMotion == 0)))
        {
            if(doaStaticCfg->isDetMatrixLogScale)
            {
                detMatrix[i].dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint16_t);
            }
            else
            {
                detMatrix[i].dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint32_t);
            }
            detMatrix[i].data = detMatrixData;
            if (detMatrix[i].data == NULL)
            {
                retVal = -3;
                goto exit;
            }
            detMatrix[i].datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;

            #if 0
            rangeProfile[i] = DPC_ObjDet_MemPoolAlloc(CoreLocalRamObj,
                                                        staticCfg->numRangeBins * sizeof(uint32_t),
                                                        sizeof(uint32_t));
            if (obj->rangeProfile[i] == NULL)
            {
                retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_PROFILE;
                goto exit;
            }
            #endif
        }
        else
        {
            detMatrix[i].dataSize = 0;
            detMatrix[i].data = NULL;
            //rangeProfile[i] = NULL;
        }
    }


    /* L3 - Doppler Index Matrix */
    if ((doaStaticCfg->selectCoherentPeakInDopplerDim == 1) ||
        (doaStaticCfg->selectCoherentPeakInDopplerDim == 2))
    {
        if (doaStaticCfg->angleDimension == 2)
        {
            /* 2D-case -  with elevation */
            dopplerIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * doaStaticCfg->elevationFftSize * sizeof(uint8_t);
        }
        else
        {
            /* 1D-case -  no elevation */
            dopplerIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint8_t);
        }
        dopplerIndexMatrix.data = dopplerIndexMatrixData;
        if (dopplerIndexMatrix.data == NULL)
        {
            retVal = -4;
            goto exit;
        }
        dopplerIndexMatrix.datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;
    }
    else
    {
        /* Non-coherent combining along Doppler dimension, Doppler output = 0 */
        dopplerIndexMatrix.dataSize = 0;
        dopplerIndexMatrix.data = NULL;
        dopplerIndexMatrix.datafmt = 0;
    }

    if (doaStaticCfg->angleDimension == 2)
    {
        /* L3 - Elevation Index Matrix */
        elevationIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint8_t);
        elevationIndexMatrix.data = elevationIndexMatrixData;
        if (elevationIndexMatrix.data == NULL)
        {
            retVal = -5;
            goto exit;
        }
        elevationIndexMatrix.datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;
    }
    else
    {
        elevationIndexMatrix.dataSize = 0;
        elevationIndexMatrix.data = NULL;
        elevationIndexMatrix.datafmt = 0;
    }

    /* hwRes - copy these structures */
    hwRes->radarCube = radarCube;   //ToDo remove from here it is passed through the process function, set to NULL
    hwRes->radarCubeMinMot = radarCubeMinMot; //ToDo remove from here it is passed through the process function, set to NULL
    //hwRes->detMatrix.data = NULL;  //ToDo remove from here it is passed through the process function, set to NULL
    hwRes->detMatrix = detMatrix[0];
    hwRes->dopplerIndexMatrix = dopplerIndexMatrix;
    hwRes->elevationIndexMatrix = elevationIndexMatrix;

    /* hwRes - edmaCfg */
    edmaCfg->edmaHandle = gEdmaHandle[0];

    /* edmaIn - ping - minor motion*/
    edmaCfg->edmaIn.chunk[0].channel =            DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_CH; 
    edmaCfg->edmaIn.chunk[0].channelShadow =      DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_SHADOW;
    edmaCfg->edmaIn.chunk[0].eventQueue =         DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_EVENT_QUE;

    /* edmaIn - pong - minor motion*/
    edmaCfg->edmaIn.chunk[1].channel =            DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_CH; 
    edmaCfg->edmaIn.chunk[1].channelShadow =      DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_SHADOW; 
    edmaCfg->edmaIn.chunk[1].eventQueue =         DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_EVENT_QUE;

    /* edmaHotSig */
    edmaCfg->edmaHotSig.channel =             DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_CH; 
    edmaCfg->edmaHotSig.channelShadow =       DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_SHADOW;
    edmaCfg->edmaHotSig.eventQueue =          DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_EVENT_QUE;




    /* edmaOut - Detection Matrix */
    edmaCfg->edmaDetMatOut.channel =           DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_CH; 
    edmaCfg->edmaDetMatOut.channelShadow =     DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_SHADOW;
    edmaCfg->edmaDetMatOut.eventQueue =        DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_EVENT_QUE;

    /* edmaOut - Elevation Index Matrix */
    edmaCfg->elevIndMatOut.channel =       DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_CH; 
    edmaCfg->elevIndMatOut.channelShadow = DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_SHADOW;
    edmaCfg->elevIndMatOut.eventQueue =    DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_EVENT_QUE;

    /* edmaOut - Doppler Index Matrix */
    edmaCfg->dopIndMatOut.channel =        DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_CH; 
    edmaCfg->dopIndMatOut.channelShadow =  DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_SHADOW;
    edmaCfg->dopIndMatOut.eventQueue =     DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_EVENT_QUE;

    edmaCfg->edmaInterLoopOut.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_CH; 
    edmaCfg->edmaInterLoopOut.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_SHADOW;
    edmaCfg->edmaInterLoopOut.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_EVENT_QUE;

    edmaCfg->edmaInterLoopIn.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_CH; 
    edmaCfg->edmaInterLoopIn.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_SHADOW;
    edmaCfg->edmaInterLoopIn.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_EVENT_QUE;

    edmaCfg->edmaInterLoopHotSig.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_CH; 
    edmaCfg->edmaInterLoopHotSig.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_SHADOW;
    edmaCfg->edmaInterLoopHotSig.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_EVENT_QUE;

    edmaCfg->edmaInterLoopChainBack.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_CH; 
    edmaCfg->edmaInterLoopChainBack.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_SHADOW;
    edmaCfg->edmaInterLoopChainBack.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_EVENT_QUE;
    edmaCfg->intrObj = &intrObj;

    /* hwaCfg */
    hwaCfg->hwaMemInpAddr = CSL_APP_HWA_DMA0_RAM_BANK3_BASE; 
    hwaCfg->numParamSets = 0;  //The number depends on the configuration, will be populated by DPU_DoaProc_config()
    hwaCfg->paramSetStartIdx = 0;

    /* hwaCfg - window */
    //Share FFT window between azimuth and elevation FFT, window = [+1 -1 +1 -1 ... ]
    winGenLen = (doaStaticCfg->azimuthFftSize > doaStaticCfg->elevationFftSize) ? doaStaticCfg->azimuthFftSize : doaStaticCfg->elevationFftSize;
    hwaCfg->windowSize = winGenLen * sizeof(int32_t);
    /*if (windowBuffer == NULL)
    {
        retVal = -6;
        goto exit;
    }*/

    /*Alternate 1,-1,...*/
    for (i=0; i<winGenLen; i++)
    {
        windowBuffer[i] = (1 - 2 * (i & 0x1)) * ((1<<17) - 1);
    }
    hwaCfg->window = (int32_t *)windowBuffer;
    hwaCfg->winRamOffset = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET;
    hwaCfg->winSym = HWA_FFT_WINDOW_NONSYMMETRIC;

    /* Rx compensation coefficients */
    #if 0
    {
        int32_t rxInd, txInd;
        int32_t ind = 0;
        doaStaticCfg->compRxChanCfg.rangeBias = commonCfg->compRxChanCfg.rangeBias;
        for (txInd = 0; txInd < doaStaticCfg->numTxAntennas; txInd++)
        {
            for (rxInd = 0; rxInd < doaStaticCfg->numRxAntennas; rxInd++)
            {
                doaStaticCfg->compRxChanCfg.rxChPhaseComp[ind++] = commonCfg->compRxChanCfg.rxChPhaseComp[rxInd + staticCfg->txAntOrder[txInd] * SYS_COMMON_NUM_RX_CHANNEL];
            }
        }
    }
    #endif

#if 1
    if (doaStaticCfg->doaRangeLoopType == DPU_DOAPROC_RANGE_LOOP_CPU_CONTROL)
    {
        //External range loop
        hwRes->interLoopDataBuffer =  (cmplx32ImRe_t *) interLoopDataBufferMem;
        if (hwRes->interLoopDataBuffer == NULL)
        {
            retVal = -5;
            goto exit;
        }
    }
    else
    {
        hwRes->interLoopDataBuffer = NULL;
    }
#endif

    hwRes->interLoopDataBuffer = NULL;

#if 0
    /* Allocate memory for the save location HWA param sets */
    hwRes->hwaCfg.hwaParamsSaveLoc.sizeBytes = DPU_DOAPROC_MAX_NUM_HWA_PARAMSET * HWA_NUM_REG_PER_PARAM_SET * sizeof(uint32_t);
    hwRes->hwaCfg.hwaParamsSaveLoc.data = DPC_ObjDet_MemPoolAlloc(CoreLocalRamObj, hwRes->hwaCfg.hwaParamsSaveLoc.sizeBytes, sizeof(uint32_t));
    if (hwRes->hwaCfg.hwaParamsSaveLoc.data == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__HWA_PARAM_SAVE_LOCATION;
        goto exit;
    }
#endif

exit:
return retVal;
}

void doaProcDpuTest_hwaInit()
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

void doaProcDpuTest_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_DoaProc_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    doaProcDpuHandle =  DPU_DoaProc_init(&initParams, &errorCode);
    if (doaProcDpuHandle == NULL)
    {
        DebugP_log ("Debug: DoaProc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*        set up the range processing DPU configurations, do not change for each test 
*/
void doaProcDpuTest_dpuCfg()
{
    DPU_DoaProc_HW_Resources * pHwConfig;
    
    memset((void *)&doaProcDpuCfg, 0, sizeof(DPU_DoaProc_Config));
        
    /* hwi configuration */
    pHwConfig = &doaProcDpuCfg.hwRes; 
    pHwConfig->hwaCfg.paramSetStartIdx = 5;
}

void doaproc_test_main(void *args)
{
    uint32_t            baseAddr, regionId;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            testCount=0;
    int32_t             retVal = 0;
    rangeProcTestConfig_t testConfig;
    DPU_DoaProc_RadarCubeSource radarCubeSrc;
    uint32_t numSamples, numSamplesPing, numSamplesPong;
    DPU_DoaProc_OutParams outParms;
    DPIF_RadarCube radarCube;
    DPU_DoaProc_HW_Resources  *hwRes;
    
    hwRes = &doaProcDpuCfg.hwRes; 

    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    DebugP_log("DPU Test Started \r\n");
    
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    doaProcDpuTest_hwaInit();
    doaProcDpuTest_dpuInit();
    /* config the range proc DPU */
    doaProcDpuTest_dpuCfg();

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
    fread(&testConfig.numTxAntennas, sizeof(uint32_t),1,fileId);
    fread(&testConfig.numChirpsPerFrame, sizeof(uint32_t),1,fileId);
    fread(&testConfig.numFrames, sizeof(uint32_t),1,fileId);

    testConfig.numTxAntennas = testConfig.numTxAntennas/testConfig.numRxAntennas;
    testConfig.numRangeBins = testConfig.numAdcSamples/2;  
    testConfig.numChirpsPerFrameRef = testConfig.numChirpsPerFrame;
    testConfig.numVirtualAntennas = testConfig.numTxAntennas * testConfig.numRxAntennas;
    testConfig.azimuthFftSize = 64;


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

    testFeatureStatus = 1;    
    
    retVal = Test_setProfile(&testConfig);

    radarCube = doaProcDpuCfg.hwRes.radarCube;
    //Major motion
    numSamples = testConfig.numRxAntennas * testConfig.numTxAntennas * (testConfig.numChirpsPerFrame/testConfig.numTxAntennas);
    numSamplesPing = numSamples;
    numSamplesPong = 0;
    radarCubeSrc.chunk[0].srcAddress = (uint32_t) radarCube.data;
    radarCubeSrc.chunk[0].dstAddress = hwRes->hwaCfg.hwaMemInpAddr;
    radarCubeSrc.chunk[0].Bcnt_Acnt = (numSamplesPing << 16)   | sizeof(cmplx16ImRe_t);

    radarCubeSrc.chunk[1].srcAddress = (uint32_t) radarCube.data;
    radarCubeSrc.chunk[1].dstAddress = hwRes->hwaCfg.hwaMemInpAddr;
    radarCubeSrc.chunk[1].Bcnt_Acnt = (numSamplesPong << 16)  | sizeof(cmplx16ImRe_t);
    
    retVal = DPU_DoaProc_config (doaProcDpuHandle,
                                    &doaProcDpuCfg);

    if(retVal < 0)
    {
        DebugP_log("DEBUG: DOA DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    fileId = fopen("..\\..\\..\\..\\common\\testBench\\major_motion\\adc_data_0001_CtestAdc6Ant_demo_RadarCube.bin", "rb");
    if (fileId == NULL)
    {
        printf("Error:  Cannot open adc_data_0001_CtestAdc6Ant_demo_RadarCube.bin !\n");
        exit(0);
    }

    fread((uint16_t *)&radarCubeData, testConfig.numRangeBins * testConfig.numRxAntennas  * testConfig.numChirpsPerFrame * sizeof(cmplx16ReIm_t), 1, fileId);

    memset((void *)&outParms, 0, sizeof(DPU_DoaProc_OutParams));

    retVal = DPU_DoaProc_process(doaProcDpuHandle, &radarCubeSrc, &hwRes->detMatrix, &outParms);
    
    /* check the result */
    DebugP_log("\n... DPU Finished, Check data ....  : \n\n");

    DebugP_log("Test #%d finished!\n\r", testCount);
    testCount++;
        
exit:

    fclose(fileId);

    if(status == SystemP_SUCCESS)
    {
        fileId_read = fopen("..\\..\\..\\..\\common\\testBench\\major_motion\\detmat_demo.bin", "rb");
        if (fileId_read == NULL)
        {
            printf("Error:  detmat_demo.bin !\n");
            exit(0);
        }
        fread((uint32_t *)&detMatrixRef, testConfig.numRangeBins * testConfig.azimuthFftSize * sizeof(uint32_t),  1, fileId_read);
        status = checkTestResult(detMatrixData, detMatrixRef, testConfig.azimuthFftSize * testConfig.numTxAntennas * sizeof(uint32_t));
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
