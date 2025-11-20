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
 *  This example performs zoomproc DPU test.
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
#include <datapath/dpu/zoomproc/v0/zoomproc.h>
/* MATH utils library Include files */
#include <utils/mathutils/mathutils.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define EDMA_TEST_EVT_QUEUE_NO      (0U)
#define MAX_NUM_TX_ANTENNA          (1U)
#define MAX_NUM_RX_ANTENNA          (1U)
#define MAX_NUM_RANGEBIN            (1024U)
#define MAX_NUM_ADCSAMPLE_PERCHIRP  (1024U)
#define MAX_NUM_CHIRPS_PERFRAME     (1U)

/* Zoom DPU */
#define DPC_OBJDET_EDMA_SHADOW_BASE                                   SOC_EDMA_NUM_DMACH
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_CH                             EDMA_APPSS_TPCC_B_EVT_CHIRP_AVAIL_IRQ
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SHADOW                         (DPC_OBJDET_EDMA_SHADOW_BASE + 0)
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_EVENT_QUE                      0
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SIG_CH                         EDMA_APPSS_TPCC_B_EVT_FREE_0
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SIG_SHADOW                     (DPC_OBJDET_EDMA_SHADOW_BASE + 2)
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SIG_EVENT_QUE                  0

#define DPC_OBJDET_DPU_ZOOMPROC_EDMAOUT_CH                            EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ0
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAOUT_SIG_SHADOW                    (DPC_OBJDET_EDMA_SHADOW_BASE + 3)
#define DPC_OBJDET_DPU_ZOOMPROC_EDMAOUT_EVENT_QUE                     0

#define MMW_DEMO_TEST_ADC_BUFF_SIZE                                   512

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))

HWA_Handle gHwaHandle;
DPU_ZoomProc_Handle zoomProcDpuHandle;
DPU_ZoomProc_Config zoomProcDpuCfg;

uint8_t testFeatureStatus;

uint8_t checkResultFlag;

uint8_t finalResults ;

Edma_IntrObject     intrObj;

/*! ADC samples buffer when reading ADC data from file */
int16_t gAdcTestBuff[MMW_DEMO_TEST_ADC_BUFF_SIZE] __attribute((section(".l3"))) = {0,	8,	1,	-8,	-1,	8,	2,	-8,	-3,	7,	3,	-7,	-4,	7,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	6,	-4,	-7,	4,	7,	-3,	-7,	3,	8,	-2,	-8,	1,	8,	-1,	-8,	0,	8,	1,	-8,	-2,	8,	2,	-8,	-3,	7,	3,	-7,	-4,	7,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	7,	-4,	-7,	4,	7,	-3,	-8,	2,	8,	-2,	-8,	1,	8,	0,	-8,	0,	8,	1,	-8,	-2,	8,	2,	-8,	-3,	7,	4,	-7,	-4,	7,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	7,	-4,	-7,	4,	7,	-3,	-8,	2,	8,	-2,	-8,	1,	8,	0,	-8,	0,	8,	1,	-8,	-2,	8,	2,	-7,	-3,	7,	4,	-7,	-4,	7,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	7,	-4,	-7,	3,	7,	-3,	-8,	2,	8,	-1,	-8,	1,	8,	0,	-8,	-1,	8,	1,	-8,	-2,	8,	3,	-7,	-3,	7,	4,	-7,	-4,	6,	5,	-6,	-6,	6,	6,	-5,	-6,	4,	7,	-4,	-7,	3,	7,	-3,	-8,	2,	8,	-1,	-8,	1,	8,	0,	-8,	-1,	8,	1,	-8,	-2,	8,	3,	-7,	-3,	7,	4,	-7,	-5,	6,	5,	-6,	-6,	5,	6,	-5,	-7,	4,	7,	-4,	-7,	3,	7,	-3,	-8,	2,	8,	-1,	-8,	0,	8,	0,	-8,	-1,	8,	2,	-8,	-2,	8,	3,	-7,	-4,	7,	4,	-7,	-5,	6,	5,	-6,	-6,	5,	6,	-5,	-7,	4,	7,	-4,	-7,	3,	8,	-2,	-8,	2,	8,	-1,	-8,	0,	8,	0,	-8,	-1,	8,	2,	-8,	-2,	8,	3,	-7,	-4,	7,	4,	-7,	-5,	6,	5,	-6,	-6,	5,	6,	-5,	-7,	4,	7,	-4,	-7,	3,	8,	-2,	-8,	2,	8,	-1,	-8,	0,	8,	0,	-8,	-1,	8,	2,	-8,	-3,	7,	3,	-7,	-4,	7,	4,	-7,	-5,	6,	5,	-6,	-6,	5,	6,	-5,	-7,	4,	7,	-3,	-7,	3,	8,	-2,	-8,	1,	8,	-1,	-8,	0,	8,	1,	-8,	-1,	8,	2,	-8,	-3,	7,	3,	-7,	-4,	7,	4,	-6,	-5,	6,	6,	-6,	-6,	5,	6,	-4,	-7,	4,	7,	-3,	-7,	3,	8,	-2,	-8,	1,	8,	-1,	-8,	0,	8,	1,	-8,	-1,	8,	2,	-8,	-3,	7,	3,	-7,	-4,	7,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	7,	-4,	-7,	4,	7,	-3,	-7,	2,	8,	-2,	-8,	1,	8,	0,	-8,	0,	8,	1,	-8,	-2,	8,	2,	-8,	-3,	7,	4,	-7,	-4,	7,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	7,	-4,	-7,	4,	7,	-3,	-8,	2,	8,	-2,	-8,	1,	8,	0,	-8,	0,	8,	1,	-8,	-2,	8,	2,	-8,	-3,	7,	4,	-7,	-4,	7,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	7,	-4,	-7,	3,	7,	-3,	-8,	2,	8,	-2,	-8,	1,	8,	0,	-8,	-1,	8,	1,	-8,	-2,	8,	3,	-7,	-3,	7,	4,	-7,	-4,	6,	5,	-6,	-5,	6,	6,	-5,	-6,	5,	7,	-4,	-7,	3,	7,	-3,	-8,	2,	8,	-1,	-8,	1,	8};
//int16_t gAdcTestBuff[MMW_DEMO_TEST_ADC_BUFF_SIZE] __attribute((section(".l3"))) = {0,	0,	1,	1,	2,	2,	3,	3,	4,	4,	5,	5,	5,	6,	6,	6,	7,	7,	7,	7,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	7,	7,	7,	7,	6,	6,	6,	5,	5,	5,	4,	4,	3,	3,	3,	2,	2,	1,	1,	0,	0,	-1,	-1,	-2,	-2,	-3,	-3,	-4,	-4,	-5,	-5,	-5,	-6,	-6,	-6,	-7,	-7,	-7,	-7,	-7,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-7,	-7,	-7,	-7,	-6,	-6,	-6,	-6,	-5,	-5,	-4,	-4,	-4,	-3,	-3,	-2,	-2,	-1,	-1,	0,	0,	1,	1,	2,	2,	3,	3,	4,	4,	4,	5,	5,	6,	6,	6,	7,	7,	7,	7,	7,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	7,	7,	7,	7,	7,	6,	6,	6,	5,	5,	4,	4,	4,	3,	3,	2,	2,	1,	1,	0,	0,	-1,	-1,	-2,	-2,	-3,	-3,	-4,	-4,	-4,	-5,	-5,	-6,	-6,	-6,	-6,	-7,	-7,	-7,	-7,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-7,	-7,	-7,	-7,	-7,	-6,	-6,	-6,	-5,	-5,	-5,	-4,	-4,	-3,	-3,	-2,	-2,	-1,	-1,	0,	0,	1,	1,	2,	2,	3,	3,	3,	4,	4,	5,	5,	5,	6,	6,	6,	7,	7,	7,	7,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	7,	7,	7,	7,	6,	6,	6,	5,	5,	5,	4,	4,	3,	3,	2,	2,	1,	1,	0,	0,	0,	-1,	-1,	-2,	-2,	-3,	-3,	-4,	-4,	-5,	-5,	-5,	-6,	-6,	-6,	-7,	-7,	-7,	-7,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-7,	-7,	-7,	-7,	-6,	-6,	-6,	-5,	-5,	-5,	-4,	-4,	-3,	-3,	-3,	-2,	-2,	-1,	-1,	0,	0,	1,	1,	2,	2,	3,	3,	4,	4,	5,	5,	5,	6,	6,	6,	7,	7,	7,	7,	7,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	7,	7,	7,	7,	6,	6,	6,	6,	5,	5,	4,	4,	4,	3,	3,	2,	2,	1,	1,	0,	0,	-1,	-1,	-2,	-2,	-3,	-3,	-4,	-4,	-4,	-5,	-5,	-6,	-6,	-6,	-7,	-7,	-7,	-7,	-7,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-7,	-7,	-7,	-7,	-7,	-6,	-6,	-6,	-5,	-5,	-4,	-4,	-4,	-3,	-3,	-2,	-2,	-1,	-1,	0,	0,	1,	1,	2,	2,	3,	3,	4,	4,	4,	5,	5,	6,	6,	6,	6,	7,	7,	7,	7,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	8,	7,	7,	7,	7,	7,	6,	6,	6,	5,	5,	5,	4,	4,	3,	3,	2,	2,	1,	1,	0,	0,	-1,	-1,	-2,	-2,	-3,	-3,	-3,	-4,	-4,	-5,	-5,	-5,	-6,	-6,	-6,	-7,	-7,	-7,	-7,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-8,	-7,	-7,	-7,	-7,	-6,	-6,	-6,	-5,	-5,	-5,	-4,	-4,	-3,	-3,	-2,	-2,	-1,	-1,	0};
/*Bench data - 0.55m*/
//int16_t gAdcTestBuff[MMW_DEMO_TEST_ADC_BUFF_SIZE] __attribute((section(".l3"))) = {-175,	-144,	-73,	0,	58,	74,	49,	80,	89,	157,	234,	257,	170,	104,	84,	135,	145,	124,	122,	12,	-41,	-46,	-44,	-63,	-100,	-129,	-107,	-160,	-137,	-111,	-167,	-172,	-182,	-120,	-86,	-76,	-52,	23,	29,	33,	35,	49,	117,	171,	247,	227,	184,	134,	147,	135,	117,	45,	-45,	-113,	-115,	-90,	-103,	-180,	-240,	-242,	-293,	-243,	-149,	-112,	-124,	-86,	-49,	46,	91,	137,	215,	228,	253,	271,	268,	261,	264,	254,	271,	201,	191,	163,	98,	38,	-76,	-161,	-233,	-246,	-223,	-195,	-237,	-278,	-326,	-299,	-239,	-190,	-157,	-130,	-94,	-31,	61,	144,	185,	123,	152,	184,	219,	247,	252,	214,	161,	115,	96,	106,	80,	80,	63,	30,	-42,	-52,	-48,	-27,	-92,	-119,	-83,	-52,	-15,	-14,	-33,	-76,	-67,	-53,	-8,	-14,	4,	-24,	-58,	-76,	-58,	-39,	-49,	-64,	-98,	-91,	-132,	-95,	-72,	-61,	-61,	-69,	-46,	11,	9,	18,	60,	62,	107,	124,	187,	221,	255,	223,	163,	103,	89,	92,	75,	63,	-4,	-54,	-118,	-141,	-181,	-188,	-283,	-330,	-305,	-268,	-192,	-175,	-175,	-194,	-200,	-180,	-74,	-11,	46,	31,	15,	21,	16,	-8,	-32,	6,	42,	64,	49,	92,	57,	9,	-9,	-9,	0,	-18,	-13,	22,	18,	-37,	-40,	-104,	-153,	-128,	-86,	-73,	-91,	-126,	-125,	-154,	-187,	-199,	-228,	-268,	-266,	-208,	-194,	-196,	-225,	-196,	-191,	-143,	-62,	-24,	-9,	-3,	33,	50,	32,	36,	59,	27,	-8,	-35,	-47,	-53,	-54,	-76,	-119,	-144,	-176,	-165,	-172,	-158,	-182,	-216,	-230,	-211,	-202,	-153,	-169,	-185,	-168,	-151,	-104,	-95,	-106,	-132,	-129,	-119,	-80,	-84,	-74,	-70,	-46,	-5,	20,	7,	-35,	-45,	-73,	-104,	-163,	-142,	-144,	-153,	-162,	-192,	-218,	-277,	-282,	-254,	-242,	-266,	-187,	-151,	-105,	-89,	-74,	-82,	-101,	-59,	17,	44,	17,	24,	-35,	-51,	-41,	-25,	-30,	-53,	-77,	-101,	-142,	-190,	-190,	-199,	-186,	-186,	-199,	-195,	-154,	-153,	-134,	-116,	-114,	-99,	-68,	-22,	34,	31,	-14,	-34,	-63,	-65,	-87,	-89,	-132,	-169,	-161,	-136,	-161,	-225,	-260,	-248,	-191,	-142,	-108,	-91,	-69,	-58,	-15,	10,	60,	102,	123,	138,	152,	173,	142,	128,	82,	43,	3,	-59,	-106,	-127,	-152,	-190,	-256,	-316,	-327,	-310,	-279,	-276,	-251,	-242,	-220,	-191,	-141,	-77,	-43,	-1,	48,	102,	117,	130,	159,	185,	186,	176,	172,	154,	149,	131,	93,	34,	-9,	-74,	-95,	-99,	-143,	-160,	-205,	-251,	-281,	-267,	-238,	-208,	-197,	-191,	-159,	-128,	-69,	-27,	5,	28,	52,	96,	102,	108,	114,	132,	115,	84,	59,	48,	19,	-11,	-8,	-52,	-81,	-111,	-160,	-185,	-169,	-165,	-167,	-192,	-198,	-168,	-114,	-145,	139,	358,	232,	191,	78,	-114,	-67,	140,	144,	6,	-161,	38,	146,	-53,	-131,	117,	36,	-154,	70,	62,	-167,	91,	8,	-135,	143,	-128,	33,	17,	-109,	115,	-149,	111,	-121,	60,	-69,	15,	-33,	-12,	-19,	-20,	-9,	-19,	-14,	-15,	-15,	-15,	-19,	-14,	-18,	-13,	-14,	-10,	-17,	-12,	-14,	-16,	-14,	-15,	-18,	-14,	-13,	-14,	-14,	-15,	-18,	-15,	-15,	-14,	-17,	-14};
int16_t gZoomDftOut[MMW_DEMO_TEST_ADC_BUFF_SIZE] __attribute((section(".l3")));

uint16_t gZoomInRbin;
uint16_t gPeakLoc;

typedef struct zoomProcTestConfig_t_ {
    uint32_t numTxAntennas;
    uint32_t numRxAntennas;
    uint32_t numAdcSamples;
    //uint32_t numRangeBins;
    float maxBeatFreq;
    float freqSlopeConst;
    float freqRes;
    float zoomRngFactor;
    uint16_t numDftBins;
    uint16_t zoomFftSize;
    uint16_t zoomSamplesOneSide;
    uint16_t *peakLoc;
    uint32_t numChirpsPerFrame;
    uint32_t numFrames;
} zoomProcTestConfig_t;

/**
*  @b Description
*  @n
*    Based on the test configuration, set up the range processing DPU configurations
*/
void Test_setProfile(zoomProcTestConfig_t * testConfig)
{
    DPU_ZoomProc_HW_Resources  *pHwConfig;
    DPU_ZoomProc_StaticConfig  * params;
    
    pHwConfig = &zoomProcDpuCfg.hwRes; 
    params = &zoomProcDpuCfg.staticCfg; 
    
    /* overwrite the DPIF_commonParams with the test configuration*/
    params->isTestMode = true;
    params->numTxAntennas = testConfig->numTxAntennas;
    params->numVirtualAntennas = testConfig->numTxAntennas * testConfig->numRxAntennas;
    params->freqRes = testConfig->freqRes;
    params->zoomRngFactor = testConfig->zoomRngFactor;
    params->numRangeBins = testConfig->numAdcSamples/2; //testConfig->numRangeBins;
    params->numDftBins = testConfig->numDftBins;
    params->zoomFftSize = testConfig->zoomFftSize;
    params->zoomSamplesOneSide = testConfig->zoomSamplesOneSide;
    params->interpFactor = (testConfig->zoomFftSize >> mathUtils_ceilLog2(testConfig->numAdcSamples));
    params->numChirpsPerFrame = testConfig->numChirpsPerFrame;
    params->peakLoc = testConfig->peakLoc;
    
    params->ADCBufData.dataProperty.numAdcSamples = testConfig->numAdcSamples;
    //params->ADCBufData.dataProperty.numRxAntennas = testConfig->numRxAntennas;
    
    params->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
    /* Data Input EDMA */
    pHwConfig->edmaInCfg.dataIn.channel         = DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_CH;
    pHwConfig->edmaInCfg.dataIn.channelShadow   = DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SHADOW;
    pHwConfig->edmaInCfg.dataIn.eventQueue      = DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_EVENT_QUE;
    
    pHwConfig->edmaInCfg.dataInSignature.channel         = DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SIG_CH;
    pHwConfig->edmaInCfg.dataInSignature.channelShadow   = DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SIG_SHADOW;
    pHwConfig->edmaInCfg.dataInSignature.eventQueue      = DPC_OBJDET_DPU_ZOOMPROC_EDMAIN_SIG_EVENT_QUE;
    pHwConfig->intrObj = &intrObj;

    /* Data Output EDMA */
    pHwConfig->edmaOutCfg.dataOutZoom.channel = DPC_OBJDET_DPU_ZOOMPROC_EDMAOUT_CH;
    pHwConfig->edmaOutCfg.dataOutZoom.channelShadow = DPC_OBJDET_DPU_ZOOMPROC_EDMAOUT_SIG_SHADOW;
    pHwConfig->edmaOutCfg.dataOutZoom.eventQueue = DPC_OBJDET_DPU_ZOOMPROC_EDMAOUT_EVENT_QUE;

    pHwConfig->zoomDftOutSize = testConfig->numDftBins * sizeof(cmplx32ImRe_t);

    if (gPeakLoc < testConfig->zoomSamplesOneSide)
    {
        gZoomInRbin = 0;
    }

    else if (gPeakLoc > (params->numRangeBins - testConfig->zoomSamplesOneSide))
    {
        gZoomInRbin = (params->numRangeBins - 2 * testConfig->zoomSamplesOneSide) * params->interpFactor;
    }

    else
    {
        gZoomInRbin = (gPeakLoc - testConfig->zoomSamplesOneSide) * params->interpFactor; // = 32(P-1) + K - 64
    }

    pHwConfig->zoomInRbin = (uint16_t *) &gZoomInRbin;
   
}

void zoomProcDpuTest_hwaInit()
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

void zoomProcDpuTest_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_ZoomProc_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    zoomProcDpuHandle =  DPU_ZoomProc_init (&initParams, &errorCode);
    if (zoomProcDpuHandle == NULL)
    {
        DebugP_log ("Debug: ZoomProc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*        set up the range processing DPU configurations, do not change for each test 
*/
void zoomProcDpuTest_dpuCfg()
{
    DPU_ZoomProc_HW_Resources * pHwConfig;
    
    memset((void *)&zoomProcDpuCfg, 0, sizeof(DPU_ZoomProc_Config));
        
    
    /* hwi configuration */
    pHwConfig = &zoomProcDpuCfg.hwRes; 
        
    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = 0;
    pHwConfig->hwaCfg.numParamSet = DPU_ZOOMPROC_NUM_HWA_PARAM_SETS;
    pHwConfig->hwaCfg.dataInputMode = DPU_ZoomProc_InputMode_ISOLATED;
    pHwConfig->hwaCfg.dmaTrigSrcChan = 0;

    /* edma configuration */
    pHwConfig->edmaHandle  = gEdmaHandle[0];  
    /* edma configuration depends on the interleave or non-interleave */
    
    /* adc buffer buffer, format fixed, interleave, size will change */
    zoomProcDpuCfg.staticCfg.ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_REAL16;
    zoomProcDpuCfg.staticCfg.ADCBufData.dataProperty.adcBits = 2U; // 12-bit only
    zoomProcDpuCfg.staticCfg.ADCBufData.dataProperty.numChirpsPerChirpEvent = 1U;
    zoomProcDpuCfg.staticCfg.ADCBufData.data = (void *) gAdcTestBuff;
    
    /* DFT out */
    zoomProcDpuCfg.hwRes.zoomDftOut  = (cmplx32ImRe_t *) gZoomDftOut;
}

void zoomproc_test_main(void *args)
{
    uint32_t            baseAddr, regionId;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            testCount=0;
    int32_t             retVal = 0;
    zoomProcTestConfig_t testConfig;
    
    DPU_ZoomProc_OutParams outParms;

    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    DebugP_log("DPU Test Started \r\n");

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    zoomProcDpuTest_hwaInit();
    zoomProcDpuTest_dpuInit();
    /* config the range proc DPU */
    zoomProcDpuTest_dpuCfg();

    /* read in test config */
    testConfig.numRxAntennas = 1;
    testConfig.numAdcSamples = 512;
    testConfig.numTxAntennas = 1;
    testConfig.numChirpsPerFrame = 1;
    testConfig.numFrames = 1;
    testConfig.numDftBins = 128;
    testConfig.maxBeatFreq = 5; //MHz - sampling rate
    testConfig.freqSlopeConst = 33.71; // MHz/usec

    testConfig.freqRes = (testConfig.maxBeatFreq * 1.0e6)/(testConfig.numAdcSamples);
    testConfig.zoomRngFactor = (float)((3.0e8)/(2.0 * (testConfig.freqSlopeConst * 1.0e12)));
    
    testConfig.zoomFftSize = 16384;
    testConfig.zoomSamplesOneSide = 2;
    gPeakLoc = 125 - 1;
    testConfig.peakLoc = &gPeakLoc;
    
    if ((testConfig.numTxAntennas > MAX_NUM_TX_ANTENNA) || ((testConfig.numAdcSamples/2) > MAX_NUM_RANGEBIN) || (testConfig.numChirpsPerFrame > MAX_NUM_CHIRPS_PERFRAME))
    {
        DebugP_log("Wrong test configurations \n");
        testFeatureStatus = 0;
        finalResults = 0;
        goto exit;
    }
    
    DebugP_log("numTxAntennas = %d\r", testConfig.numTxAntennas);
    DebugP_log("numRangeBins = %d\r", (testConfig.numAdcSamples/2));  
    DebugP_log("numChirpsPerFrame = %d\n", testConfig.numChirpsPerFrame);
    DebugP_log("\n");
    
    testFeatureStatus = 1;    
    
    Test_setProfile(&testConfig);

    checkResultFlag = 1;
    
    retVal = DPU_ZoomProc_config (zoomProcDpuHandle,
                                        &zoomProcDpuCfg);

    if(retVal < 0)
    {
        DebugP_log("DEBUG: Zoom DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    memset((void *)&outParms, 0, sizeof(DPU_ZoomProc_OutParams));

    retVal = DPU_ZoomProc_process(zoomProcDpuHandle, &outParms);

    /* check the result */
    DebugP_log("\n... DPU Finish, Check Cube data ....  : \n\n");
    if (checkResultFlag == 1)
    {
        testConfig.numDftBins = zoomProcDpuCfg.staticCfg.numDftBins;
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

    if(status == SystemP_SUCCESS && (abs(outParms.zoomRangeMeas - 5.4101) < 0.0001))
    {
        DebugP_log("[Zoomproc] Test Completed!!\r\n");
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

