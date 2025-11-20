/**
 *   @file  aoa2dproc.c
 *
 *   @brief
 *      Implements Data path range gate processing Unit using HWA.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2023 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK driver/common Include Files */
#include <common/syscommon.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include <drivers/hw_include/hw_types.h>

/* Utils */
#include <utils/mathutils/mathutils.h>

/* Data Path Include files */
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpu/aoa2dproc/v0/aoa2dproc.h>
#include <datapath/dpu/aoa2dproc/v0/aoa2dprocinternal.h>

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

#define DPU_AOA2DPROC_BANK_0   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[0])
#define DPU_AOA2DPROC_BANK_1   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[1])
#define DPU_AOA2DPROC_BANK_2   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[2])
#define DPU_AOA2DPROC_BANK_3   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[3])

/* HWA ping/pong buffers offset */
#define DPU_AOA2DPROC_SRC_PING_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[0])
#define DPU_AOA2DPROC_SRC_PONG_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[1])
#define DPU_AOA2DPROC_DST_PING_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[2])
#define DPU_AOA2DPROC_DST_PONG_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[3])

#define DPU_AOA2DPROC_AZIMUTH_OUTPUT_BASE_OFFSET   DPU_AOA2DPROC_BANK_0


/**
 * @brief   Detection matrix data format ToDo should be defined in SDK
 */
#define DPU_AOA2DPROC_DPIF_DETMATRIX_FORMAT_2 2

/* User defined heap memory and handle */
#define AOA2DPROC_HEAP_MEM_SIZE  (sizeof(DPU_Aoa2dProc_Obj))

static uint8_t gAoa2dProcHeapMem[AOA2DPROC_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/*===========================================================
 *                    Internal Functions
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *     The function creates the list of active range gates (the range gates with point-cloud points) with the
 *     number of detected points for each active range gate). This is calculated based on the input list of point-cloud 
 *     points defined as {rangeInd, dopplerInd}, where the list is arranged with range indices rangeInd  in increasing order.
 *      HWA processing completion call back function.
 *  \ingroup    DPU_AOA2D_INTERNAL_FUNCTION
 */
void aoa2dProc_findRangeGates(DPIF_CFARRngDopDetListElement * detRngDopList, uint32_t numDetectedPoints, DPIF_DetectedRangeGates *rangeGates)
{
    uint32_t rangeGateIdx = 0;
    uint32_t pointIdx = 0;
    uint32_t maxNumRangeGates = rangeGates->maxNumRangeGates;
    DPIF_DetectedRangeGate *rngGateArray = rangeGates->array;

    if (numDetectedPoints == 0)
    {
       goto exit;
    }

    /*Set the first range gate */
    rngGateArray[rangeGateIdx].rangeIdx = detRngDopList[pointIdx].rangeIdx;
    rngGateArray[rangeGateIdx].numDopplerBins = 1;
    rangeGateIdx++;

    for (int pointIdx = 1; pointIdx < numDetectedPoints; pointIdx++)
    {
       if (detRngDopList[pointIdx].rangeIdx == detRngDopList[pointIdx-1].rangeIdx)
       {
           rngGateArray[rangeGateIdx-1].numDopplerBins++;
       }
       else
       {
           rngGateArray[rangeGateIdx].rangeIdx = detRngDopList[pointIdx].rangeIdx;
           rngGateArray[rangeGateIdx].numDopplerBins = 1;
           rangeGateIdx++;
           if (rangeGateIdx == maxNumRangeGates)
           {
               break;
           }
       }
    }

exit:
    rangeGates->numRangeGates = rangeGateIdx;
}


/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function.
 *  \ingroup    DPU_AOA2D_INTERNAL_FUNCTION
 */
static void aoa2dProc_hwaDoneIsrCallback(void * arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA completion call back function.
 *
 *  @param[in] intrHandle    EDMA Interrupt handle
 *  @param[in] arg           Input argument is pointer to semaphore object
 *
 *  \ingroup    DPU_AOA2D_INTERNAL_FUNCTION
 */
static void aoa2dProc_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object *)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      Configures HWA for Doppler processing.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_AOA2D_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
static inline int32_t aoa2dProc_configHwa
(
    DPU_Aoa2dProc_Obj      *obj,
    DPU_Aoa2dProc_Config   *cfg
)
{
    HWA_ParamConfig         hwaParamCfg;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;

    uint32_t                numAntRow, numAntCol;

    uint16_t skipOutputDopplerBins;
    uint16_t numAzimuthBins;
    uint16_t guardLen;
    uint16_t winLen;
    uint8_t  noiseDivShift;
    uint32_t hwaMemOffset;



    skipOutputDopplerBins = 0;

#if 0  //ToDo
    /* Check if we have the correct number of paramsets.*/
    if(cfg->hwRes.hwaCfg.numParamSets != (2 * cfg->staticCfg.numTxAntennas + 2))
    {
        retVal = DPU_AOA2DPROC_EHWARES;
        goto exit;
    }
#endif

    numAntRow = cfg->staticCfg.numAntRow;
    numAntCol = cfg->staticCfg.numAntCol;


    /********************************************************************************/
    /* First HWA loop for Doppler calculation */
    /********************************************************************************/
    /* Currently no scaling in Doppler FFT. */
    obj->dopFftSumDiv = 0; //mathUtils_ceilLog2(cfg->staticCfg.numDopplerBins);

    paramsetIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;

    obj->dopplerHwaComp.startIdx = (uint16_t) paramsetIdx;

    if (cfg->staticCfg.isCompressionEnabled)
    {
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        /****** Ping paramset of Doppler FFT for 1st Range bin (by EDMA in) ******/
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg.dmaTriggerSrc = cfg->hwRes.hwaCfg.dmaTrigSrcChan;
        hwaParamCfg.accelMode = HWA_ACCELMODE_COMPRESS;

        hwaParamCfg.source.srcAddr = DPU_AOA2DPROC_BANK_1;
        hwaParamCfg.source.srcAcnt = 1 - 1;
        hwaParamCfg.source.srcBcnt = (cfg->staticCfg.numVirtualAntennas * cfg->staticCfg.numDopplerChirps) - 1;

        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        if (cfg->staticCfg.compressCfg.compressionFactor  == 2)
        {
            hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
            hwaParamCfg.source.srcBIdx = sizeof(uint32_t);
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
            hwaParamCfg.source.srcScale = 0;
        }
        else if (cfg->staticCfg.compressCfg.compressionFactor == 4)
        {
            hwaParamCfg.source.srcAIdx = sizeof(uint16_t);
            hwaParamCfg.source.srcBIdx = sizeof(uint16_t);
            hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
            hwaParamCfg.source.srcScale = 8;
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.source.srcConjugate = 0; //no conjugate

        hwaParamCfg.dest.dstAddr = DPU_AOA2DPROC_BANK_0;
        hwaParamCfg.dest.dstAcnt = 1 - 1;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.dest.dstAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstScale = 0;

        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0; //no conjugate
        hwaParamCfg.dest.dstSkipInit = 0;

        hwaParamCfg.accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_DECOMPRESS;
        hwaParamCfg.accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_ENABLE;  // Enable dither to suppress quantization spurs
        hwaParamCfg.accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
        hwaParamCfg.accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_BFP;
        hwaParamCfg.accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
        if (cfg->staticCfg.compressCfg.compressionFactor  == 2)
        {
            hwaParamCfg.accelModeArgs.compressMode.BFPMantissaBW = 7;
            hwaParamCfg.accelModeArgs.compressMode.scaleFactorBW = 4;
        }
        else if (cfg->staticCfg.compressCfg.compressionFactor == 4)
        {
            hwaParamCfg.accelModeArgs.compressMode.BFPMantissaBW = 3;
            hwaParamCfg.accelModeArgs.compressMode.scaleFactorBW = 4;
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }
        obj->decompressHwaParamSetPtr = (uint32_t *) HWA_getParamSetAddr(obj->hwaHandle, paramsetIdx);
        obj->decompressHwaParamSetIdx = paramsetIdx;
        /* Save param set for even range gate */
        memcpy(&obj->decompressHwaParams[0][0], obj->decompressHwaParamSetPtr, DPU_AOA2DPROC_HWA_NUM_REGS_PER_PARAM_SET*sizeof(uint32_t));

        hwaParamCfg.dest.dstAcnt = 2 - 1;
        hwaParamCfg.dest.dstSkipInit = 1;
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    paramsetIdx,
                                    &hwaParamCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }
        /* Save param set for odd range gate */
        memcpy(&obj->decompressHwaParams[1][0], obj->decompressHwaParamSetPtr, DPU_AOA2DPROC_HWA_NUM_REGS_PER_PARAM_SET*sizeof(uint32_t));


        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        paramsetIdx++;
    }
    /********************************************************************************/
    /*                    Doppler FFT Configuration                                 */
    /********************************************************************************/
    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    if (cfg->staticCfg.isCompressionEnabled)
    {
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    }
    else
    {
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_SOFTWARE;
    }
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
    hwaParamCfg.source.srcAddr =  DPU_AOA2DPROC_BANK_0;
    hwaParamCfg.source.srcAcnt = cfg->staticCfg.numBpmDopplerChirps - 1; //size in samples - 1

    hwaParamCfg.source.srcAIdx = cfg->staticCfg.numRxAntennas * sizeof(cmplx16ImRe_t);
    hwaParamCfg.source.srcBcnt = cfg->staticCfg.numRxAntennas - 1;
    hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t);
    hwaParamCfg.source.srcShift = 0;
    hwaParamCfg.source.srcCircShiftWrap = 0;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 8;
    hwaParamCfg.source.bpmEnable = 0;
    hwaParamCfg.source.bpmPhase = 0;

    hwaParamCfg.dest.dstAddr = DPU_AOA2DPROC_BANK_1;
    hwaParamCfg.dest.dstAcnt = cfg->staticCfg.bpmDopplerFftSize - 1; //this is samples - 1
    hwaParamCfg.dest.dstAIdx = cfg->staticCfg.numRxAntennas * sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstBIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.dest.dstConjugate = 0; //no conjugate
    hwaParamCfg.dest.dstScale = 8;
    hwaParamCfg.dest.dstSkipInit = skipOutputDopplerBins; //Doppler zero skipped - CLUTTER RMOVAL

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2BpmDopplerFftSize;

    /* scaling is disabled in all stages */
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.bpmDopplerFftSize - 1) >> 5;


    hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaParamCfg.accelModeArgs.fftMode.windowEn = 1;
    hwaParamCfg.accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.dopplerWindow.winRamOffset;
    hwaParamCfg.accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.dopplerWindow.winSym;
    hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    retVal = HWA_configParamSet(obj->hwaHandle,
                                paramsetIdx,
                                &hwaParamCfg, NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                          paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
    paramsetIdx++;
    obj->dopplerHwaComp.numParams = paramsetIdx - obj->dopplerHwaComp.startIdx;

    /********************************************************************************/
    /* Second HWA loop for angle calculation */
    /********************************************************************************/
    if (cfg->staticCfg.angleDimension > 0)
    {
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
    }
    else
    {
        numAzimuthBins = 1;
    }

    /* Save intermediate addresses */
    hwaMemOffset = numAntRow * numAntCol * sizeof(cmplx32ImRe_t);
    if(cfg->staticCfg.angleDimension == 2)
    {
        /* Azimuth and elevation */
        obj->hwaAngleProcOut.azimFftOutPtr = (int32_t *) (obj->hwaMemBankAddr[2] + hwaMemOffset); //Complex output
        obj->hwaAngleProcOut.azimSpectElevIndPtr = (DPIF_HWA_STATS_MaxOutput *) (obj->hwaMemBankAddr[3] + hwaMemOffset);
        hwaMemOffset += numAntRow * numAzimuthBins * sizeof(cmplx32ImRe_t);
        obj->hwaAngleProcOut.azimMaxPeakAzimIndPtr = (DPIF_HWA_STATS_MaxOutput *) (obj->hwaMemBankAddr[2] + hwaMemOffset);
        hwaMemOffset += sizeof(DPIF_HWA_STATS_MaxOutput);
        obj->hwaAngleProcOut.azimPeakList =  (DPIF_HWA_CFAR_DetOutput *) (obj->hwaMemBankAddr[2] + hwaMemOffset);
    }
    else
    {
        /* Azimuth only */
        obj->hwaAngleProcOut.azimFftOutPtr = (int32_t *) (obj->hwaMemBankAddr[2] + hwaMemOffset); //Magnitude output
        obj->hwaAngleProcOut.azimSpectElevIndPtr = NULL;
        obj->hwaAngleProcOut.azimMaxPeakAzimIndPtr = (DPIF_HWA_STATS_MaxOutput *) (obj->hwaMemBankAddr[3] + hwaMemOffset);
        hwaMemOffset += sizeof(DPIF_HWA_STATS_MaxOutput);
        obj->hwaAngleProcOut.azimPeakList =  (DPIF_HWA_CFAR_DetOutput *) (obj->hwaMemBankAddr[3] + hwaMemOffset);
    }

    obj->angleHwaComp.startIdx = (uint16_t) paramsetIdx;
    /********************************************************************************/
    /*                    Rx channel phase compensation                             */
    /********************************************************************************/
    memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_SOFTWARE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
    hwaParamCfg.source.srcAddr =  DPU_AOA2DPROC_BANK_2;
    hwaParamCfg.source.srcAcnt = (numAntRow * numAntCol) -1;

    hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBcnt = 1 - 1;
    hwaParamCfg.source.srcBIdx = 0;
    hwaParamCfg.source.srcShift = 0;
    hwaParamCfg.source.srcCircShiftWrap = 0;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcConjugate = 0;
    hwaParamCfg.source.srcScale = 0;
    hwaParamCfg.source.bpmEnable = 0;
    hwaParamCfg.source.bpmPhase = 0;

    hwaParamCfg.dest.dstAddr = DPU_AOA2DPROC_BANK_3;
    hwaParamCfg.dest.dstAcnt = (numAntRow * numAntCol) - 1; //this is samples - 1
    hwaParamCfg.dest.dstAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstBIdx = 0;
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.dest.dstConjugate = 0;
    hwaParamCfg.dest.dstScale = 8;

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
#if 1
    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;
#else//ToDo REMOVE_THIS just for HWA verification
    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER;
    hwaParamCfg.complexMultiply.cmpMulArgs.twidIncrement = 455;
#endif
    retVal = HWA_configParamSet(obj->hwaHandle,
                                paramsetIdx,
                                &hwaParamCfg, NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                          paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
    paramsetIdx++;


    /***************************************************************/
    /******************* Configure Azimuth FFT *********************/
    /***************************************************************/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.source.srcAddr = DPU_AOA2DPROC_BANK_3;
    hwaParamCfg.source.srcAcnt = numAntCol - 1;
    hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBIdx = numAntCol * sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBcnt = (numAntRow) - 1;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 0;

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    if (numAzimuthBins > 1)
    {
        hwaParamCfg.accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    }
    else
    {
        hwaParamCfg.accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_DISABLE;
    }
    hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//assumes power of 2;
    hwaParamCfg.accelModeArgs.fftMode.windowEn = 1; //Azimuth and elavation share FFT window. Window is 1,-1,1,-1... to achieve "fffshift" in spectral domain
    hwaParamCfg.accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.angleWindow.winRamOffset;
    hwaParamCfg.accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.angleWindow.winSym;
    hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //ToDo Tweak this
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    if (cfg->staticCfg.angleDimension == 2)
    {
        /*With Elevation*/
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.dest.dstAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimFftOutPtr); //DPU_AOA2DPROC_BANK_2 + (numAntRow + numAntCol * sizeof(cmplx32ImRe_t));
        hwaParamCfg.dest.dstAcnt = numAzimuthBins - 1;
        hwaParamCfg.dest.dstAIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    }
    else
    {
        /*No Elevation*/
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
        hwaParamCfg.dest.dstAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimFftOutPtr); //DPU_AOA2DPROC_BANK_2 + (numAntRow + numAntCol * sizeof(cmplx32ImRe_t));
        hwaParamCfg.dest.dstAcnt = numAzimuthBins - 1;
        hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
        hwaParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(uint32_t);
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    }


    hwaParamCfg.dest.dstConjugate = 0; //no conjugate
    hwaParamCfg.dest.dstScale = 8;

    retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
    if (retVal != 0)
    {
      goto exit;
    }
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                          paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
      goto exit;
    }

    paramsetIdx++;


    if(cfg->staticCfg.angleDimension == 2)
    {
        /*****************************************************************/
        /******************* Configure Elevation FFT *********************/
        /*****************************************************************/
        memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));

        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimFftOutPtr); //DPU_AOA2DPROC_BANK_2  + (numAntRow + numAntCol * sizeof(cmplx32ImRe_t));
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcAcnt = numAntRow - 1;
        hwaParamCfg.source.srcBcnt = numAzimuthBins - 1;
        hwaParamCfg.source.srcAIdx = numAzimuthBins * sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcBIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0; //no conjugate
        hwaParamCfg.source.srcScale = 0;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.elevationFftSize);//assumes power of 2;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 1;   //Azimuth and elavation share FFT window. Window is 1,-1,1,-1... to achieve "fffshift" in spectral domain
        hwaParamCfg.accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.angleWindow.winRamOffset;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.angleWindow.winSym;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //ToDo Tweak this
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;

        hwaParamCfg.dest.dstAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimSpectElevIndPtr); //(uint16_t) DPU_AOA2DPROC_BANK_3 + 0x100;//ToDo REMOVE_OFFSET;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;


        retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
        if (retVal != 0)
        {
          goto exit;
        }
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
        goto exit;
        }
        paramsetIdx++;
    }


    /**********************************************************************/
    /* Single Maximum in azimuth dimension (need for side lobe threshold) */
    /**********************************************************************/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));

    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    if(cfg->staticCfg.angleDimension == 2)
    {
        hwaParamCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimSpectElevIndPtr) + sizeof(uint32_t); //DPU_AOA2DPROC_BANK_3 + sizeof(uint32_t)  + (numAntRow + numAntCol * sizeof(cmplx32ImRe_t)); //point to second element i.e. peak value. (the first is elevation index)
        hwaParamCfg.source.srcAIdx = sizeof(DPU_Aoa2dProc_HwaMaxOutput);
    }
    else
    {
        hwaParamCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimFftOutPtr); //DPU_AOA2DPROC_BANK_2  + (numAntRow + numAntCol * sizeof(cmplx32ImRe_t)); //point to second element i.e. peak value. (the first is elevation index)
        hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
    }
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcAcnt = numAzimuthBins - 1;
    hwaParamCfg.source.srcBcnt = 1 - 1;
    hwaParamCfg.source.srcBIdx = 0;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 0;

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;
    hwaParamCfg.dest.dstAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimMaxPeakAzimIndPtr);//(uint16_t) DPU_AOA2DPROC_BANK_2 + 0x2000;

    hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
    hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
    hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
    hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
    hwaParamCfg.dest.dstScale = 8;
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;

    retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
    if (retVal != 0)
    {
      goto exit;
    }
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                          paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
    goto exit;
    }
    paramsetIdx++;


    /*****************************************************************/
    /******** Configure CFAR to pickup maxima                *********/
    /*****************************************************************/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    //Cyclic mode
    guardLen = 0;
    winLen = 2;
    noiseDivShift = 2;
    hwaParamCfg.source.srcAcnt = (numAzimuthBins - 1) + (guardLen + winLen) + (guardLen + winLen);
    hwaParamCfg.source.srcShift = numAzimuthBins - (guardLen + winLen);
    hwaParamCfg.source.srcCircShiftWrap = mathUtils_floorLog2(numAzimuthBins);

    if (cfg->staticCfg.angleDimension == 2)
    {
        hwaParamCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimSpectElevIndPtr) + sizeof(uint32_t); //DPU_AOA2DPROC_BANK_3 + sizeof(uint32_t) + 0x100;//ToDo REMOVE_OFFSET;
        hwaParamCfg.source.srcAIdx = sizeof(DPU_Aoa2dProc_HwaMaxOutput);
    }
    else
    {
        hwaParamCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimFftOutPtr); //DPU_AOA2DPROC_BANK_2;
        hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
    }
    hwaParamCfg.source.srcBIdx = 0;
    hwaParamCfg.source.srcBcnt = 1  - 1;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.source.srcConjugate = 0;
    hwaParamCfg.source.srcScale = 0;

    hwaParamCfg.accelMode = HWA_ACCELMODE_CFAR;
    hwaParamCfg.accelModeArgs.cfarMode.operMode = HWA_CFAR_OPER_MODE_MAG_INPUT_REAL;
    hwaParamCfg.accelModeArgs.cfarMode.peakGroupEn = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.cfarMode.numGuardCells = guardLen;
    hwaParamCfg.accelModeArgs.cfarMode.nAvgDivFactor = noiseDivShift;
    hwaParamCfg.accelModeArgs.cfarMode.cyclicModeEn = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg.accelModeArgs.cfarMode.nAvgMode = HWA_NOISE_AVG_MODE_CFAR_CA;
    hwaParamCfg.accelModeArgs.cfarMode.numNoiseSamplesRight = winLen >> 1;
    hwaParamCfg.accelModeArgs.cfarMode.numNoiseSamplesLeft =  winLen >> 1;
    hwaParamCfg.accelModeArgs.cfarMode.outputMode = HWA_CFAR_OUTPUT_MODE_I_PEAK_IDX_Q_CUT;

    hwaParamCfg.dest.dstAddr =  HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaAngleProcOut.azimPeakList); // (uint16_t) DPU_AOA2DPROC_BANK_2 + 0x2008; //ToDo REEMOVE_OFFSET

    hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
    hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
    hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
    hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
    hwaParamCfg.dest.dstScale = 8;

    retVal = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &hwaParamCfg, NULL);
    if (retVal != 0)
    {
      goto exit;
    }
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                          paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
      goto exit;
    }

    paramsetIdx++;
    obj->angleHwaComp.numParams = (uint16_t) paramsetIdx - obj->angleHwaComp.startIdx;

exit:
    return(retVal);
 }

/**
 *  @b Description
 *  @n
 *  EDMA configuration.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_AOA2D_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static inline int32_t aoa2dProc_configEdma
(
    DPU_Aoa2dProc_Obj      *obj,
    DPU_Aoa2dProc_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    cmplx16ImRe_t       *radarCubeBase = (cmplx16ImRe_t *)cfg->hwRes.radarCube.data;
    int16_t             sampleLenInBytes = sizeof(cmplx16ImRe_t);
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;
    bool isTransferCompletionEnabled;
    Edma_EventCallback transferCompletionCallbackFxn;
    void* transferCompletionCallbackFxnArg;

    if(obj == NULL)
    {
        retVal = DPU_AOA2DPROC_EINVAL;
        goto exit;
    }

    /*****************************************************************************************/
    /**************                     PROGRAM DMA INPUT                    *****************/
    /*****************************************************************************************/
    if (!cfg->staticCfg.isCompressionEnabled)
    {
        chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn.channel;
        chainingCfg.isIntermediateChainingEnabled = false;
        chainingCfg.isFinalChainingEnabled        = false;
    }
    else
    {
        chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.channel;
        chainingCfg.isIntermediateChainingEnabled = false;
        chainingCfg.isFinalChainingEnabled        = true;
    }
    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = aoa2dProc_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&(obj->edmaDoneSemaHandle);

    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]); //This is temporary, it will be reloaded on the fly
    if (!cfg->staticCfg.isCompressionEnabled)
    {
        syncABCfg.destAddress = (uint32_t) cfg->hwRes.hwaCfg.hwaMemInpAddr;
        syncABCfg.aCount      = sampleLenInBytes;
        syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.bpmDopplerFftSize;
        syncABCfg.cCount      = 1;
        syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sampleLenInBytes;
        syncABCfg.srcCIdx     = 0;
        syncABCfg.dstBIdx     = sampleLenInBytes;
        syncABCfg.dstCIdx     = 0;
    }
    else
    {
        syncABCfg.destAddress = (uint32_t) cfg->hwRes.hwaCfg.hwaMemOutAddr;;
        if (cfg->staticCfg.compressCfg.compressionFactor == 2)
        {
            syncABCfg.aCount      = sizeof(uint32_t);
            syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins/2 * sizeof(uint32_t);
            syncABCfg.dstBIdx     = sizeof(uint32_t);
        }
        else if(cfg->staticCfg.compressCfg.compressionFactor == 4)
        {
            syncABCfg.aCount      = sizeof(uint16_t);
            syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins/2 * sizeof(uint16_t);
            syncABCfg.dstBIdx     = sizeof(uint16_t);
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
        syncABCfg.srcCIdx     = 0;
        syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.bpmDopplerFftSize;
        syncABCfg.cCount      = 1;
        syncABCfg.dstCIdx     = 0;

    }
    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 isTransferCompletionEnabled,
                                 transferCompletionCallbackFxn,
                                 transferCompletionCallbackFxnArg,
                                 cfg->hwRes.edmaCfg.intrObj);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    if (cfg->staticCfg.isCompressionEnabled)
    {
        retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                                    &cfg->hwRes.edmaCfg.edmaHotSig,
                                                    obj->hwaHandle,
                                                    cfg->hwRes.hwaCfg.dmaTrigSrcChan,
                                                    false);
        if (retVal != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
exit:
    return(retVal);
} 

/*===========================================================
 *                    Doppler Proc External APIs
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      dopplerProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]   initCfg Pointer to initial configuration parameters
 *  @param[out]  errCode Pointer to errCode generates by the API
 *
 *  \ingroup    DPU_AOA2DPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
DPU_Aoa2dProc_Handle DPU_Aoa2dProc_init
(
    DPU_Aoa2dProc_InitParams *initCfg,
    int32_t                    *errCode
)
{
    DPU_Aoa2dProc_Obj  *obj = NULL;
    HWA_MemInfo             hwaMemInfo;
    uint32_t                i;
    int32_t                 status = SystemP_SUCCESS;

    *errCode       = 0;
    
    if((initCfg == NULL) || (initCfg->hwaHandle == NULL))
    {
        *errCode = DPU_AOA2DPROC_EINVAL;
        goto exit;
    }    

    /* Allocate memory */
    obj = (DPU_Aoa2dProc_Obj*)&gAoa2dProcHeapMem;
    if(obj == NULL)
    {
        *errCode = DPU_AOA2DPROC_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)obj, 0U, sizeof(DPU_Aoa2dProc_Obj));
    
    // RPMF printf("AOA2D DPU: (DPU_Aoa2dProc_Obj *) 0x%08x\n", (uint32_t) obj);

    /* Save init config params */
    obj->hwaHandle   = initCfg->hwaHandle;

    /* Create DPU semaphores */
    status = SemaphoreP_constructBinary(&obj->edmaDoneSemaHandle, 0);
    if(SystemP_SUCCESS != status)
    {
        *errCode = DPU_AOA2DPROC_ESEMA;
        goto exit;
    }

    status = SemaphoreP_constructBinary(&obj->hwaDoneSemaHandle, 0);
    if(SystemP_SUCCESS != status)
    {
        *errCode = DPU_AOA2DPROC_ESEMA;
        goto exit;
    }

    /* Populate HWA base addresses and offsets. This is done only once, at init time.*/
    *errCode =  HWA_getHWAMemInfo(obj->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {       
        goto exit;
    }
    
    /* check if we have enough memory banks*/
    if(hwaMemInfo.numBanks < DPU_AOA2DPROC_NUM_HWA_MEMBANKS)
    {    
        *errCode = DPU_AOA2DPROC_EHWARES;
        goto exit;
    }
    
    for (i = 0; i < DPU_AOA2DPROC_NUM_HWA_MEMBANKS; i++)
    {
        obj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress + i * hwaMemInfo.bankSize;
    }
    
exit:    

    if(*errCode < 0)
    {
        if(obj != NULL)
        {
            obj = NULL;
        }
    }
   return ((DPU_Aoa2dProc_Handle)obj);
}

void aoa2dProc_mapRxChanCompensationCoefs(DPU_Aoa2dProc_Config    *cfg, cmplx32ImRe_t *rxChCompVirtArray)
{
    uint16_t src, dst;
    int32_t i, k;
    int32_t scale;
    cmplx32ImRe_t *rxChPhaseComp = (cmplx32ImRe_t *) &cfg->staticCfg.compRxChanCfg.rxChPhaseComp;
    int32_t numParam = cfg->hwRes.hwaCfg.aoa2dRngGateCfg.numDopFftParams;
    DPU_Aoa2dProc_HWA_Doppler_Fft_Cfg *paramCfg = cfg->hwRes.hwaCfg.aoa2dRngGateCfg.dopFftCfg;

    for (i=0; i < numParam; i++)
    {
        for (k=0; k < paramCfg[i].srcBcnt; k++)
        {
            src = paramCfg[i].srcAddrOffset + k * paramCfg[i].srcBidx;
            dst = paramCfg[i].dstAddrOffset + k * paramCfg[i].dstBidx;
            scale = paramCfg[i].scale;
            rxChCompVirtArray[dst].real = scale * rxChPhaseComp[src].real;
            rxChCompVirtArray[dst].imag = scale * rxChPhaseComp[src].imag;
        }
    }
}

/**
 *  @b Description
 *  @n  Convert angle Fov to number of skipped indices.
 *
 *  \ingroup    DPU_CFARPROC_INTERNAL_FUNCTION
 *
 *
 * @param[in]  angle Angle in radians
 *
 * @param[in]  fftSize Angle FFT size
 *
 * @param[in]  skipFromLeft true: return number of skipped bins from left, false: return number of skipped bins from right
 *
 * @param[out]  skipBins Number of skipped bins from left or right side
 *
 */
int16_t  aoa2dProc_ConvertFovToIdx(float angle, uint16_t fftSize, bool skipFromLeft)
{
#define PI_OVER_180 0.01745329252

    float temp;
    int32_t ind;
    int16_t numSkippedBins;

    temp = fftSize/2 * sin(angle * PI_OVER_180);
    if(temp < 0)
    {
        ind = (int32_t) (temp - 0.5);
    }
    else
    {
        ind = (int32_t) (temp + 0.5);
    }

    if(skipFromLeft)
    {
        numSkippedBins = fftSize/2 + ind;
        if (numSkippedBins < 1)
        {
            numSkippedBins = 1;
        }
    }
    else
    {
        numSkippedBins = fftSize/2 - 1 - ind;
        if (numSkippedBins < 0)
        {
            numSkippedBins = 0;
        }
    }

    return numSkippedBins;
}

/**
 *  @b Description
 *  @n  Convert angle field of view to indices
 *
 *  \ingroup    DPU_AOA2DPROC_INTERNAL_FUNCTION
 *
 *
 * @param[in]  fovAoaCfg Field of view configuration
 *
 * @param[in]  azimuthFftSize Azimuth FFT size
 *
 * @param[in]  elevationFftSize Elevation FFT size
 *
 * @param[out] detectionCfg detection configuration
 *
 */
void aoa2dProc_ConfigAoaFov(DPU_Aoa2dProc_AoaFovCfg *fovAoaCfg,
                                  uint16_t azimuthFftSize,
                                  uint16_t elevationFftSize,
                                  DPU_Aoa2dProc_detectionCfg *detectionCfg)
{

    detectionCfg->skipLeftAzim = aoa2dProc_ConvertFovToIdx(fovAoaCfg->minAzimuthDeg, azimuthFftSize, true);
    detectionCfg->skipRightAzim = aoa2dProc_ConvertFovToIdx(fovAoaCfg->maxAzimuthDeg, azimuthFftSize, false);

    detectionCfg->skipLeftElev = aoa2dProc_ConvertFovToIdx(fovAoaCfg->minElevationDeg, elevationFftSize, true);
    detectionCfg->skipRightElev = aoa2dProc_ConvertFovToIdx(fovAoaCfg->maxElevationDeg, elevationFftSize, false);
}


/**
  *  @b Description
  *  @n
  *   AOA2D DPU configuration
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    DPU_AOA2DPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_Aoa2dProc_config
(
    DPU_Aoa2dProc_Handle    handle,
    DPU_Aoa2dProc_Config    *cfg
)
{
    DPU_Aoa2dProc_Obj   *obj;
    int32_t                  retVal = 0;
    cmplx32ImRe_t rxChComp[DPU_AOA2D_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];


    obj = (DPU_Aoa2dProc_Obj *)handle;
    if(obj == NULL)
    {
        retVal = DPU_AOA2DPROC_EINVAL;
        goto exit;
    }

#if 0
    if (cfg->staticCfg.angleDimension == 2)
    {
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
        numElevationBins = cfg->staticCfg.elevationFftSize;
    }
    else if (cfg->staticCfg.angleDimension == 1)
    {
        numAzimuthBins = cfg->staticCfg.azimuthFftSize;
        numElevationBins = 1;
    }
    else
    {
        numAzimuthBins = 1;
        numElevationBins = 1;
    }
#endif
    
#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle ||
       !cfg->hwRes.hwaCfg.angleWindow.window ||
       !cfg->hwRes.hwaCfg.dopplerWindow.window ||
       !cfg->hwRes.radarCube.data
      )
    {
        retVal = DPU_AOA2DPROC_EINVAL;
        goto exit;
    }

    /* Check if radar cube format is supported by DPU*/
    if(cfg->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_6)
    {
        retVal = DPU_AOA2DPROC_ECUBEFORMAT;
        goto exit;
    }

    obj->radarCube = cfg->hwRes.radarCube;

    /* Check if radar cube column fits into one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_AOA2DPROC_EEXCEEDHWAMEM;
        goto exit;
    }


#endif

    /* Save necessary parameters to DPU object that will be used during Process time */
    /* EDMA parameters needed to trigger first EDMA transfer*/
    obj->edmaHandle  = cfg->hwRes.edmaCfg.edmaHandle;
    memcpy((void*)(&obj->edmaIn), (void *)(&cfg->hwRes.edmaCfg.edmaIn), sizeof(DPEDMA_ChanCfg));
    
    /* Convert FOV to FFT indices */
    aoa2dProc_ConfigAoaFov(cfg->staticCfg.fovAoaCfg,
                               cfg->staticCfg.azimuthFftSize,
                               cfg->staticCfg.elevationFftSize,
                               &obj->detectionCfg);



    /* Copy antenna mapping table */
    obj->aoa2dRngGateCfg = cfg->hwRes.hwaCfg.aoa2dRngGateCfg;

    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    /* HWA Doppler FFT window configuration */
    retVal = HWA_configRam(obj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)cfg->hwRes.hwaCfg.dopplerWindow.window,
                           cfg->hwRes.hwaCfg.dopplerWindow.windowSize, //size in bytes
                           cfg->hwRes.hwaCfg.dopplerWindow.winRamOffset * sizeof(int32_t));
    if (retVal != 0)
    {
        goto exit;
    }
    /* HWA Azimuth/Elevation FFT window configuration */
    retVal = HWA_configRam(obj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)cfg->hwRes.hwaCfg.angleWindow.window,
                           cfg->hwRes.hwaCfg.angleWindow.windowSize, //size in bytes
                           cfg->hwRes.hwaCfg.angleWindow.winRamOffset * sizeof(int32_t));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Map rx channel compensation coefficients into row x column 2D virtual array antenna array */
    aoa2dProc_mapRxChanCompensationCoefs(cfg, (cmplx32ImRe_t *) &rxChComp);

    /* HWA rx phase compensation */
    retVal = HWA_configRam(obj->hwaHandle,
                           HWA_RAM_TYPE_INTERNAL_RAM,
                           (uint8_t *) &rxChComp,
                           cfg->staticCfg.numAntRow * cfg->staticCfg.numAntCol * sizeof(cmplx32ImRe_t), //size in bytes
                           0);
    if (retVal != 0)
    {
        goto exit;
    }
    
    /*******************************/
    /**  Configure HWA            **/
    /*******************************/
    retVal = aoa2dProc_configHwa(obj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }
                    
    /*******************************/
    /**  Configure EDMA           **/
    /*******************************/    
    retVal = aoa2dProc_configEdma(obj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Copy the whole configuration to the instance */
    obj->config = *cfg;
exit:
    return retVal;
}

/**
*  @b Description
*  @n Configures Input EDMA
*
*  @param[in]   obj             DPU internal object
*  @param[in]   radarCubeSrc  Structure descriptor of the input radar cube
*
*  \ingroup    DPU_AOA2DPROC_INTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t aoa2dProc_ConfigAndTriggerEdmaInput(DPU_Aoa2dProc_Obj *obj,
                                                uint16_t rangeIdx)
{

    int32_t retVal = 0;
    uint32_t startAddress;
    cmplx16ImRe_t *radarCube = (cmplx16ImRe_t *) obj->radarCube.data;

    if (!obj->config.staticCfg.isCompressionEnabled)
    {
        startAddress = (uint32_t) SOC_virtToPhy(&radarCube[rangeIdx]);
    }
    else
    {
        if (obj->config.staticCfg.compressCfg.compressionFactor == 2)
        {
            startAddress = (uint32_t) SOC_virtToPhy(&radarCube[0]) + rangeIdx/obj->config.staticCfg.compressCfg.numComplexElements * sizeof(uint32_t);
        }
        else if (obj->config.staticCfg.compressCfg.compressionFactor == 4)
        {
            startAddress = (uint32_t) SOC_virtToPhy(&radarCube[0]) + rangeIdx/obj->config.staticCfg.compressCfg.numComplexElements * sizeof(uint16_t);
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
    }
    retVal = DPEDMA_setSourceAddress(obj->edmaHandle,
                                     obj->edmaIn.channel,
                                     startAddress);

    retVal = DPEDMA_startTransfer(obj->edmaHandle, obj->edmaIn.channel);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }


exit:
    return retVal;
}


/**
*  @b Description
*
*  @param[in]   obj        DPU object
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t aoa2dProc_ConfigAndTriggerHwaDopplerComputation
(
        DPU_Aoa2dProc_Obj * obj
)
{

    int32_t             retVal = 0;
    HWA_CommonConfig    hwaCommonConfig;


    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;

    hwaCommonConfig.numLoops      = 1;
    hwaCommonConfig.paramStartIdx = obj->dopplerHwaComp.startIdx;
    hwaCommonConfig.paramStopIdx  = obj->dopplerHwaComp.startIdx + obj->dopplerHwaComp.numParams - 1;
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(obj->hwaHandle,1);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}

/**
*  @b Description
*
*  @param[in]   obj        DPU object
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t aoa2dProc_ConfigAndTriggerHwaAngleComp
(
        DPU_Aoa2dProc_Obj * obj
)
{

    int32_t             retVal = 0;
    HWA_CommonConfig    hwaCommonConfig;


    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_CFARTHRESHOLDSCALE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;

    hwaCommonConfig.numLoops      = 1;
    hwaCommonConfig.paramStartIdx = obj->angleHwaComp.startIdx;
    hwaCommonConfig.paramStopIdx  = obj->angleHwaComp.startIdx + obj->angleHwaComp.numParams - 1;
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    /* Threshold is set to one, since we are searching for maximum peaks using CFAR with peak grouping enabled */
    hwaCommonConfig.cfarConfig.cfarThresholdScale = 16;


    retVal = HWA_configCommon(obj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(obj->hwaHandle,1);
    if (retVal != 0)
    {
        goto exit;
    }

    retVal = HWA_setSoftwareTrigger(obj->hwaHandle);
    if (retVal != 0)
    {
        goto exit;
    }
exit:
    return retVal;
}

uint32_t aoa2dProc_Calculate_XYZ(DPU_Aoa2dProc_Obj *obj,
                       DPU_Aoa2dProc_OutParams *outParams,
                       DPIF_CFARRngDopDetListElement *rngDopDetListElement,
                       uint32_t peakInd,
                       uint32_t numDetectedPoints)
{
    int32_t selAzimMin = obj->detectionCfg.skipLeftAzim;
    int32_t selAzimMax = obj->config.staticCfg.azimuthFftSize - obj->detectionCfg.skipRightAzim - 1;
    float azimBinOffset;
    uint32_t azimFftMask = obj->config.staticCfg.azimuthFftSize - 1;
    int32_t azimLeftInd, azimRightInd;

    uint16_t azimuthFftSize = obj->config.staticCfg.azimuthFftSize;
    uint16_t elevationFftSize = obj->config.staticCfg.elevationFftSize;

    uint32_t *azimuthSpectrum1D = (uint32_t *) obj->hwaAngleProcOut.azimFftOutPtr;
    DPIF_HWA_STATS_MaxOutput *azimuthSpectrum2D = obj->hwaAngleProcOut.azimSpectElevIndPtr;
    uint32_t azimuthIdx = obj->hwaAngleProcOut.azimPeakList[peakInd].cellIdx;
    uint32_t azimuthPeak = obj->hwaAngleProcOut.azimPeakList[peakInd].cut;
    uint32_t azimuthMaxPeak = obj->hwaAngleProcOut.azimMaxPeakAzimIndPtr->peak;
    int32_t sideLobeThreshold;
    int32_t elevationIdx;

    int32_t selElevMin;
    int32_t selElevMax;

    int32_t azimuthSgnIdx;
    int32_t elevationSgnIdx;

    float x, y, z;
    float Wx, Wz;
    float range;
    float temp;
    float denom;

    float lambdaOverDistX = obj->config.staticCfg.lambdaOverDistX;
    float lambdaOverDistZ = obj->config.staticCfg.lambdaOverDistZ;

    if (numDetectedPoints == outParams->detObjOutMaxSize)
    {
        goto exit;
    }

    if (obj->config.staticCfg.angleDimension == 2)
    {
        selElevMin = obj->detectionCfg.skipLeftElev;
        selElevMax = obj->config.staticCfg.elevationFftSize - obj->detectionCfg.skipRightElev - 1;
    }


    /* Check if it is inside azimuth and range FOV */
    if ((azimuthIdx < selAzimMin) || (azimuthIdx > selAzimMax))
    {
        goto exit;
    }

    azimBinOffset = 0.;

    azimLeftInd = (azimuthIdx-1) & azimFftMask;
    azimRightInd = (azimuthIdx+1) & azimFftMask;

    /*If peak is greater than side-lobe threshold*/
    sideLobeThreshold  = (int32_t) ((azimuthMaxPeak * obj->config.staticCfg.sideLobeThresholdScaleQ8) >> DPU_AOA2DPROC_SHIFT_Q8);
    if(!(azimuthPeak >= sideLobeThreshold))
    {
        goto exit;
    }

    /* get elevation index */
    if (obj->config.staticCfg.angleDimension == 2)
    {
        elevationIdx = azimuthSpectrum2D[azimuthIdx].maxInd;
        if ((elevationIdx < selElevMin) || (elevationIdx > selElevMax))
        {
            goto exit;
        }
    }
    else
    {
        elevationIdx = (int32_t)(elevationFftSize>>1);
    }

    /* Interpolate in azimuth domain */
    if (obj->config.staticCfg.enableInterpAzimuthDom)
    {
        float xLeft;
        float xRight;
        float x;
        if (obj->config.staticCfg.angleDimension == 2)
        {
            xLeft =  azimuthSpectrum2D[azimLeftInd].peak;
            xRight = azimuthSpectrum2D[azimRightInd].peak;
        }
        else
        {
            xLeft = azimuthSpectrum1D[azimLeftInd];
            xRight = azimuthSpectrum1D[azimRightInd];
        }
        x = (float) azimuthPeak;
        denom =  (4.*x - 2.*xLeft - 2.*xRight);
        if (denom != 0)
        {
            azimBinOffset = (xRight-xLeft) / denom;
        }
        else
        {
            azimBinOffset = 0;
        }
    }

    azimuthSgnIdx = (int32_t ) azimuthIdx - (int32_t)(azimuthFftSize>>1);
    elevationSgnIdx = (int32_t ) elevationIdx - (int32_t)(elevationFftSize>>1);

    range = rngDopDetListElement->range;
    Wx = lambdaOverDistX * ((float) azimuthSgnIdx + azimBinOffset) / azimuthFftSize;
    x = range * Wx;

    Wz = - lambdaOverDistZ * (float) elevationSgnIdx / elevationFftSize;

    z = range * Wz;

    temp = range*range - x*x - z*z;
    if (temp > 0)
    {
        y = sqrt(temp);
    }
    else
    {
        y = 0;
    }

    outParams->detObjOut[numDetectedPoints].x = x;
    outParams->detObjOut[numDetectedPoints].y = y;
    outParams->detObjOut[numDetectedPoints].z = z;
    outParams->detObjOut[numDetectedPoints].velocity = rngDopDetListElement->doppler;
    outParams->detObjOut[numDetectedPoints].noise = rngDopDetListElement->noisedB;
    outParams->detObjOut[numDetectedPoints].snr = rngDopDetListElement->snrdB;


    numDetectedPoints++;
exit:
    return numDetectedPoints;
}

void aoa2dProc_CopyMapAntSymbols(DPU_Aoa2dProc_Obj *obj, int16_t detPointDopInd)
{
    cmplx32ImRe_t *srcHwaMem = (cmplx32ImRe_t *) obj->hwaMemBankAddr[1];
    cmplx32ImRe_t *dstHwaMem = (cmplx32ImRe_t *) obj->hwaMemBankAddr[2];
    uint32_t numParam = obj->aoa2dRngGateCfg.numDopFftParams;
    DPU_Aoa2dProc_HWA_Doppler_Fft_Cfg *paramCfg = obj->aoa2dRngGateCfg.dopFftCfg;
    uint32_t i,k;
    uint32_t src, dst;
    uint32_t numRxAnt = obj->config.staticCfg.numRxAntennas;
    uint32_t ddmOffset = obj->config.staticCfg.numDopplerBins;
    uint32_t numRow = obj->config.staticCfg.numAntRow;
    uint32_t numCol = obj->config.staticCfg.numAntCol;
    cmplx32ImRe_t srcScratchPad[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
    cmplx32ImRe_t dstScratchPad[DPU_AOA2D_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    uint32_t ddmMask = obj->config.staticCfg.bpmDopplerFftSize - 1;
    uint32_t rdInd;

    /* Copy from HWA Memory to local memory */
    rdInd = detPointDopInd & ddmMask;
    memcpy(&srcScratchPad[0],        &srcHwaMem[rdInd * numRxAnt], numRxAnt * sizeof(cmplx32ImRe_t));
    rdInd = (rdInd + ddmOffset) & ddmMask;
    memcpy(&srcScratchPad[numRxAnt], &srcHwaMem[rdInd * numRxAnt], numRxAnt * sizeof(cmplx32ImRe_t));

    /* Map. (zero positions are ignored since the rx compensation coefficients are zeroing these positions. ToDo make single forward LUT */
    for (i=0; i < numParam; i++)
    {
        for (k=0; k < paramCfg[i].srcBcnt; k++)
        {
            src = paramCfg[i].srcAddrOffset + k * paramCfg[i].srcBidx;
            dst = paramCfg[i].dstAddrOffset + k * paramCfg[i].dstBidx;
            dstScratchPad[dst] = srcScratchPad[src];
        }
    }

    /* Copy from local memory to HWA Memory */
    memcpy(dstHwaMem, dstScratchPad, (numRow * numCol) * sizeof(cmplx32ImRe_t));
}

/**
*  @b Description
*  @n AOA2D DPU process function. Computes the range/azimuth detection matrix
*
*  @param[in]   handle        DPU handle
*  @param[in]   radarCubeSrc  Structure descriptor of the input radar cube
*  @param[in]   detMatrix     Pointer to output detection matrix.
*  @param[out]  outParams     Output parameters.
*
*  \ingroup    DPU_AOA2DPROC_EXTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t DPU_Aoa2dProc_process
(
    DPU_Aoa2dProc_Handle    handle,
    DPIF_RadarCube *radarCubeSrc,
    DPIF_CFARRngDopDetListElement *detRngDopList,
    uint32_t numDetPoints,
    DPIF_DetectedRangeGates *detectedRangeGates, //ToDo this should be scratch memory
    DPIF_DetMatrix *detMatrix,
    DPU_Aoa2dProc_OutParams *outParams
)
{
    //volatile uint32_t   startTime;

    DPU_Aoa2dProc_Obj *obj;
    int32_t             retVal = 0;
    uint16_t rngGateInIdx, rngGateOutIdx, dopplerIdx, pointIdx;
    uint16_t numAzimuthPeaks;
    int16_t detPointDopInd;
    uint32_t numDetPointsOut = 0;
    uint32_t peakInd;
    DPIF_CFARRngDopDetListElement *rngDopDetListElement;



    obj = (DPU_Aoa2dProc_Obj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_AOA2DPROC_EINVAL;
        goto exit;
    }
    /* Set inProgress state */
    obj->inProgress = true;


    outParams->numDetectedPoints = 0;
    if (numDetPoints == 0)
    {
        goto exit;
    }


    //startTime = Cycleprofiler_getTimeStamp();
    /* Prepare range gates for AoA */
    aoa2dProc_findRangeGates(detRngDopList, numDetPoints,  detectedRangeGates);


    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(obj->hwaHandle,
                                     aoa2dProc_hwaDoneIsrCallback,
                                     (void*)&obj->hwaDoneSemaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

    pointIdx = 0;
    rngGateInIdx = 0;

    if (!obj->config.staticCfg.isCompressionEnabled)
    {
        /* Configure first range gate (RG) and copy to M0 HWA memory */
        aoa2dProc_ConfigAndTriggerEdmaInput(obj, detectedRangeGates->array[rngGateInIdx++].rangeIdx);
    }
    /* Loop through all detected range gates */
    for (rngGateOutIdx = 0; rngGateOutIdx < detectedRangeGates->numRangeGates; rngGateOutIdx++)
    {

        if (obj->config.staticCfg.isCompressionEnabled)
        {
            /* Disable the HWA */
            retVal = HWA_enable(obj->hwaHandle, 0);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Load appropriate HWA param set for decompression */
            memcpy(obj->decompressHwaParamSetPtr,
                   &obj->decompressHwaParams[detectedRangeGates->array[rngGateInIdx].rangeIdx & 0x1][0],
                   DPU_AOA2DPROC_HWA_NUM_REGS_PER_PARAM_SET * sizeof(uint32_t));

            /* Trigger Doppler FFT */
            aoa2dProc_ConfigAndTriggerHwaDopplerComputation(obj);

            /* Configure first range gate (RG) and copy to M0 HWA memory */
            aoa2dProc_ConfigAndTriggerEdmaInput(obj, detectedRangeGates->array[rngGateInIdx++].rangeIdx);
        }
        else if (!obj->config.staticCfg.isCompressionEnabled)
        {
            /* Make sure RG transfer is completed */
            SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER); //ToDo enable interrupt after EDMAin, remove hot signature, remove tirgger channel  cfg doppler param to run immediate

            /* Disable the HWA */
            retVal = HWA_enable(obj->hwaHandle, 0);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Trigger Doppler FFT */
            aoa2dProc_ConfigAndTriggerHwaDopplerComputation(obj);


            retVal = HWA_setSoftwareTrigger(obj->hwaHandle);
            if (retVal != 0)
            {
                goto exit;
            }
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;

        }


        /* Wait for HWA Doppler FFT completion, output is in M1 */
        SemaphoreP_pend(&obj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

        /* Disable the HWA */
        retVal = HWA_enable(obj->hwaHandle, 0);
        if (retVal != 0)
        {
            goto exit;
        }

        if (!obj->config.staticCfg.isCompressionEnabled)
        {
            if (rngGateInIdx < detectedRangeGates->numRangeGates)
            {
                /* Trigger copy of the next range gate from radar cube to M0 HWA memory */
                aoa2dProc_ConfigAndTriggerEdmaInput(obj, detectedRangeGates->array[rngGateInIdx++].rangeIdx);
            }
        }

        /* Process all Doppler detected points at the current RG */
        for (dopplerIdx = 0; dopplerIdx < detectedRangeGates->array[rngGateOutIdx].numDopplerBins; dopplerIdx++)
        {
            detPointDopInd = detRngDopList[pointIdx].dopplerIdx;
            rngDopDetListElement = &detRngDopList[pointIdx];

            /* Copy and Map Antenna symbols from M1 to M2 */
            aoa2dProc_CopyMapAntSymbols(obj, detPointDopInd);


            /* Config and trigger HWA */
            aoa2dProc_ConfigAndTriggerHwaAngleComp(obj);

            /* Wait for HWA Azimuth/Elevation completion */
            SemaphoreP_pend(&obj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

            /* Disable the HWA */
            retVal = HWA_enable(obj->hwaHandle, 0);
            if (retVal != 0)
            {
                goto exit;
            }
            /* Read number o CFAR points - number of peaks */
            HWA_readCFARPeakCountReg(obj->hwaHandle,
                                     (uint8_t *) &numAzimuthPeaks,
                                     sizeof(uint16_t));
            for (peakInd = 0; peakInd < numAzimuthPeaks; peakInd++)
            {
                /* Process all detected peaks in azimuth domain - calculate XYZ coordinates */
                numDetPointsOut = aoa2dProc_Calculate_XYZ(obj, outParams, rngDopDetListElement, peakInd, numDetPointsOut);
            }
            pointIdx++;
        }
    }

    outParams->numDetectedPoints = numDetPointsOut;
exit:
    outParams->stats.numProcess++;
    //outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime;

    if (obj != NULL)
    {
        obj->inProgress = false;
    }    
    
    return retVal;
}

/**
  *  @b Description
  *  @n
  *  Doppler DPU deinit 
  *
  *  @param[in]   handle   DPU handle.
  *
  *  \ingroup    DPU_AOA2DPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_Aoa2dProc_deinit(DPU_Aoa2dProc_Handle handle)
{
    int32_t     retVal = 0;
    
    /* Free memory */
    if(handle == NULL)
    {
        retVal = DPU_AOA2DPROC_EINVAL;
    }
    
    return retVal;
}


/**
  *  @b Description
  *  @n
  *   Returns number of allocated HWA Param sets
  *
  *  @param[in]   handle     DPU handle.
  *  @param[out]   cfg       Number of allocated HWA Param sets
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0
  */
int32_t DPU_Aoa2dProc_GetNumUsedHwaParamSets
(
    DPU_Aoa2dProc_Handle    handle,
    uint8_t *numUsedHwaParamSets
)
{
    DPU_Aoa2dProc_Obj *obj;
    int32_t retVal = 0;

    obj = (DPU_Aoa2dProc_Obj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_AOA2DPROC_EINVAL;
        goto exit;
    }
    *numUsedHwaParamSets = (uint8_t) (obj->dopplerHwaComp.numParams + obj->angleHwaComp.numParams);
exit:
    return retVal;
}