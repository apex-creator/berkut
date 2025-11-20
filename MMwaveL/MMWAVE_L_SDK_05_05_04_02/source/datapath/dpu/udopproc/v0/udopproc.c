/**
 *   @file  udopproc.c
 *
 *   @brief
 *      Implements Data path Micro Doppler processing
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2022-2023 Texas Instruments, Inc.
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
#include <float.h>

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
#include <datapath/dpu/udopproc/v0/udopproc.h>
#include <datapath/dpu/udopproc/v0/udopprocinternal.h>

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

#define DPU_UDOPPROC_BANK_0   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[0])
#define DPU_UDOPPROC_BANK_1   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[1])
#define DPU_UDOPPROC_BANK_2   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[2])
#define DPU_UDOPPROC_BANK_3   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[3])

/* HWA ping/pong buffers offset */
#define DPU_UDOPPROC_SRC_PING_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[0])
#define DPU_UDOPPROC_SRC_PONG_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[1])
#define DPU_UDOPPROC_DST_PING_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[2])
#define DPU_UDOPPROC_DST_PONG_OFFSET   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(obj->hwaMemBankAddr[3])

#define DPU_UDOPPROC_AZIMUTH_OUTPUT_BASE_OFFSET   DPU_UDOPPROC_BANK_0

/**
 * @brief   Detection matrix data format ToDo should be defined in SDK
 */
#define DPU_UDOPPROC_DPIF_DETMATRIX_FORMAT_2 2

/**
 * @brief
 *  Configuration parameters for each HWA param set performing the Doppler FFTs and mappng output into 
 *  row-col array as input to azimuth/elevation FFTs
 *
 * @details
 * Input antenna format to Doppler FFTs per range bin is X[antInd][chirpInd]
 * Output antenna format is Y[dopplerInd][rowInd][colInd]
 * HWA param set can perform only fixed source and destination increment (B dimension)
 *
 */
typedef struct DPU_uDopProc_HWA_Doppler_Fft_Cfg_t
{
    /*! @brief   Number of input columns (doppler FFTs) in the input matrix to process */
    uint16_t    srcBcnt;

    /*! @brief   Column offset in the input matrix */
    int16_t     srcAddrOffset;

    /*! @brief  column increment in number of columns in source matrix */
    int16_t     srcBidx;

    /*! @brief   Scale in pre compensation */
    int16_t     scale;

    /*! @brief  column/row offset in the destination matrix dstAddrOffset = colOffset + rowOffset*numColumns */
    int16_t     dstAddrOffset;

    /*! @brief  column increment in number of columns in destination matrix */
    int16_t     dstBidx;
} DPU_uDopProc_HWA_Doppler_Fft_Cfg;

typedef struct DPU_uDopProc_HWA_Option_Cfg_t
{
    uint8_t numDopFftParams;
    DPU_uDopProc_HWA_Doppler_Fft_Cfg dopFftCfg[4];
} DPU_uDopProc_HWA_Option_Cfg;

/* User defined heap memory and handle */
#define MICRO_DOPPROC_HEAP_MEM_SIZE  (sizeof(DPU_uDopProc_Obj))

uint8_t guDopProcHeapMem[MICRO_DOPPROC_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

extern uint32_t Cycleprofiler_getTimeStamp(void);

/**
 *  @b Description
 *  @n
 *      EDMA completion call back function.
 *
 *  @param[in] intrHandle    EDMA Interrupt handle
 *  @param[in] arg           Input argument is pointer to semaphore object
 *
 *  \ingroup    DPU_UDOP_INTERNAL_FUNCTION
 */
static void uDopProc_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
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
 *  \ingroup    DPU_UDOP_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
static inline int32_t uDopProc_configHwa
(
    DPU_uDopProc_Obj      *obj,
    DPU_uDopProc_Config   *cfg
)
{
    HWA_ParamConfig         hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;

    DPU_DoaProc_HWA_Option_Cfg *rngGateCfg; //ToDo replace this with common structure
    uint32_t                idx;
    uint32_t                numAntRow, numAntCol;

    uint16_t numOutputDopplerBins;
    uint16_t skipOutputDopplerBins;

    uint8_t destChan;

    numOutputDopplerBins = cfg->staticCfg.numDopplerBins;
    skipOutputDopplerBins = 0;
    uint16_t numAccAzimBins = 1; //default set to one but will reconfigured per target micro doppler calculation



    numAntRow = cfg->staticCfg.numAntRow;
    numAntCol = cfg->staticCfg.numAntCol;
    rngGateCfg = &cfg->hwRes.doaRngGateCfg;

    /* Save HWA butterfly scaling bit masks for Doppler and Azimuth FFTs */
#if 1 //ToDo Revisit
    {
        uint8_t nDopScal;
        uint8_t nAzimScal;
        uint8_t nMicDopAccScal;
        int32_t expectedNumAzimAccBins = 1;

        nDopScal = mathUtils_ceilLog2(cfg->staticCfg.numDopplerBins);
        nAzimScal = mathUtils_ceilLog2(cfg->staticCfg.azimuthFftSize);
        if(obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
        {
            nMicDopAccScal = mathUtils_ceilLog2(obj->numRangeAccumBins * expectedNumAzimAccBins);
        }
        else
        {
            nMicDopAccScal = 0;
        }


        if (nMicDopAccScal <= nAzimScal)
        {
            obj->butterflyScalingAzimuth = (1<<nMicDopAccScal) - 1;
            obj->butterflyScalingDoppler = ((1<<nDopScal)-1) >> 5;
        }
        else
        {
            obj->butterflyScalingAzimuth = (1<<nAzimScal) - 1;
            obj->butterflyScalingDoppler = ((1<<(nDopScal + (nMicDopAccScal-nAzimScal)))-1) >> 5;
        }
    }
#else
    obj->butterflyScalingAzimuth = 0;
    obj->butterflyScalingDoppler = ((cfg->staticCfg.numDopplerBins)-1) >> 5;
#endif

    paramsetIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;
    if (cfg->staticCfg.isRxChGainPhaseCompensationEnabled)
    {
        /********************************************************************************/
        /*                    Rx channel phase compensation                             */
        /********************************************************************************/
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg.dmaTriggerSrc = cfg->hwRes.hwaCfg.dmaTrigSrcChan;


        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_UDOPPROC_BANK_1;
        hwaParamCfg.source.srcAcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) - 1; //ToDo we compensate only needed antennas (6 or 4), but place 8 apart on the destination

        hwaParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = cfg->staticCfg.numDopplerChirps - 1;
        hwaParamCfg.source.srcBIdx = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0;
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_UDOPPROC_BANK_0;
        hwaParamCfg.dest.dstAcnt = (cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas) - 1; //this is samples - 1
        hwaParamCfg.dest.dstAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0;
        hwaParamCfg.dest.dstScale = 0;

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;

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
    }
    /********************************************************************************/
    /*                    Doppler FFT Configuration                                 */
    /********************************************************************************/
    for (idx = 0; idx < rngGateCfg->numDopFftParams; idx++)
    {
        memset((void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));
        if ((idx == 0) && (!cfg->staticCfg.isRxChGainPhaseCompensationEnabled))
        {
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
            hwaParamCfg.dmaTriggerSrc = cfg->hwRes.hwaCfg.dmaTrigSrcChan;
        }
        else
        {
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            hwaParamCfg.dmaTriggerSrc = 0;
        }
        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT;
        hwaParamCfg.source.srcAddr =  DPU_UDOPPROC_BANK_0 + rngGateCfg->dopFftCfg[idx].srcAddrOffset * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = rngGateCfg->dopFftCfg[idx].srcBcnt - 1;
        hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t) * rngGateCfg->dopFftCfg[idx].srcBidx;
        hwaParamCfg.source.srcShift = 0;
        hwaParamCfg.source.srcCircShiftWrap = 0;
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0; //no conjugate
        hwaParamCfg.source.srcScale = 8;
        hwaParamCfg.source.bpmEnable = 0;
        hwaParamCfg.source.bpmPhase = 0;

        hwaParamCfg.dest.dstAddr = DPU_UDOPPROC_BANK_1 + rngGateCfg->dopFftCfg[idx].dstAddrOffset * sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstAcnt = (cfg->staticCfg.numDopplerBins) - 1; //this is samples - 1
        hwaParamCfg.dest.dstAIdx = numAntCol * sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx32ImRe_t) * rngGateCfg->dopFftCfg[idx].dstBidx;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstConjugate = 0; //no conjugate
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSkipInit = skipOutputDopplerBins; //Doppler zero skipped - CLUTTER RMOVAL

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumDopplerBins;

        /* scaling is disabled in all stages */
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = obj->butterflyScalingDoppler;


        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0; //disabled
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

        if(rngGateCfg->dopFftCfg[idx].scale == 0)
        {
            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_SCALAR_MULT;
        }
        else
        {
            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        }
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
    }

    if(cfg->staticCfg.angleDimension == 2)
    {
        /* Elevation FFT  and output only zero bin (zero elevation)      */
        /* Sum antenna symbols across rows                               */
        /*****************************************************************/
        /******************* Configure Elevation FFT *********************/
        /*****************************************************************/
        memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));

        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg.dmaTriggerSrc = 0;

        hwaParamCfg.source.srcAddr = (uint16_t) DPU_UDOPPROC_BANK_1;
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.source.srcAcnt = numAntRow - 1;
        hwaParamCfg.source.srcBcnt = (numAntCol * numOutputDopplerBins) - 1;
        hwaParamCfg.source.srcAIdx = numAntCol * numOutputDopplerBins * sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcBIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcConjugate = 0; //no conjugate
        hwaParamCfg.source.srcScale = 0;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAntRow);
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0; //N/A
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0; //NA
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //Antenna summation
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;

        hwaParamCfg.dest.dstAddr =  (uint16_t) DPU_UDOPPROC_BANK_0;
        hwaParamCfg.dest.dstAcnt = 1 - 1; //Save only bin zero (sum of antenna symbols)
        hwaParamCfg.dest.dstAIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.dest.dstConjugate = 0;  //no conjugate
        hwaParamCfg.dest.dstScale = 8;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;

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

    /***************************************************************/
    /******************* Configure Azimuth FFT/Beamforming *********/
    /***************************************************************/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    if(cfg->staticCfg.angleDimension == 2)
    {
        hwaParamCfg.source.srcAddr = (uint16_t) (DPU_UDOPPROC_BANK_0);
    }
    else
    {
        hwaParamCfg.source.srcAddr = (uint16_t) (DPU_UDOPPROC_BANK_1);
    }
    hwaParamCfg.source.srcAcnt = numAntCol - 1;
    hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBIdx = numAntCol * sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBcnt = (numOutputDopplerBins) - 1;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 0;

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER;

    if (obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
    {
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.azimuthFftSize);
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0; //No window
        hwaParamCfg.accelModeArgs.fftMode.windowStart = 0;
        hwaParamCfg.accelModeArgs.fftMode.winSymm = 0;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = obj->butterflyScalingAzimuth;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    }
    else
    {
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;
    }

    hwaParamCfg.dest.dstAddr = (uint16_t) (DPU_UDOPPROC_BANK_2);
    if (obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
    {
        hwaParamCfg.dest.dstAcnt = numAccAzimBins - 1;    //This is temporary filled, will be configured on the fly
        hwaParamCfg.dest.dstAIdx = sizeof(uint32_t);
        hwaParamCfg.dest.dstBIdx = numAccAzimBins * sizeof(uint32_t);   //This is temporary filled, will be configured on the fly
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstConjugate = 0; //no conjugate
        hwaParamCfg.dest.dstScale = 8;
    }
    else
    {
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
        hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
        hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
        hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation
        hwaParamCfg.dest.dstConjugate = 0; //no conjugate
        hwaParamCfg.dest.dstScale = 8;
    }

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
    obj->hwaAzimuthSpectrumParamSetIdx = (uint8_t) paramsetIdx;
    obj->hwaAzimuthSpectrumParamSetCfg = hwaParamCfg; //save the whole configuration structure
    paramsetIdx++;


    /***************************************************************/
    /******************* Accumulate AZIMUTH bins           *********/
    /***************************************************************/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.source.srcAddr = (uint16_t) (DPU_UDOPPROC_BANK_2);
    if (obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
    {
        hwaParamCfg.source.srcAcnt = numAccAzimBins - 1; //This will be programmed on the fly
        hwaParamCfg.source.srcAIdx = sizeof(uint32_t);
        hwaParamCfg.source.srcBIdx = numAccAzimBins * sizeof(uint32_t); //This will be programmed on the fly
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
    }
    else
    {
        hwaParamCfg.source.srcAcnt = 1 - 1;
        hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcBIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    }
    hwaParamCfg.source.srcBcnt = (numOutputDopplerBins) - 1;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 0;

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    if (obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
    {
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    }
    else
    {
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
    }
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;

    hwaParamCfg.dest.dstAddr =  (uint16_t) (DPU_UDOPPROC_BANK_3);
    hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
    hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
    hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation

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
    obj->hwaAzimuthBinAccParamSetIdx = (uint8_t) paramsetIdx;
    obj->hwaAzimuthBinAccParamSetCfg = hwaParamCfg; //save the whole configuration structure
    paramsetIdx++;

    /***************************************************************/
    /*************** Accumulate RANGE bins        ******************/
    /***************************************************************/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.source.srcAddr = (uint16_t) (DPU_UDOPPROC_BANK_3);
    hwaParamCfg.source.srcAcnt = 2 - 1;
    hwaParamCfg.source.srcAIdx = numOutputDopplerBins*sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBcnt = (numOutputDopplerBins) - 1;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 0;

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;

    hwaParamCfg.dest.dstAddr =  (uint16_t) (DPU_UDOPPROC_BANK_2);
    hwaParamCfg.dest.dstAcnt = 4095;    //HWA user guide recommendation
    hwaParamCfg.dest.dstAIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstBIdx = 8;       //HWA user guide recommendation
    hwaParamCfg.dest.dstRealComplex = 0;//HWA user guide recommendation
    hwaParamCfg.dest.dstWidth = 1;      //HWA user guide recommendation

    hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
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

    /*************************************************************************/
    /** Copy accumulated spectrum from M2 to M3 for next range accumulation **/
    /*************************************************************************/
    memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
    hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg.dmaTriggerSrc = 0;

    hwaParamCfg.source.srcAddr = (uint16_t) (DPU_UDOPPROC_BANK_2);
    hwaParamCfg.source.srcAcnt = numOutputDopplerBins - 1;
    hwaParamCfg.source.srcAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg.source.srcBIdx = 0;
    hwaParamCfg.source.srcBcnt = 1 - 1;
    hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.source.srcSign = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg.source.srcConjugate = 0; //no conjugate
    hwaParamCfg.source.srcScale = 0;

    hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    hwaParamCfg.dest.dstAddr =  (uint16_t) (DPU_UDOPPROC_BANK_3 + numOutputDopplerBins*sizeof(cmplx32ImRe_t));
    hwaParamCfg.dest.dstAcnt = numOutputDopplerBins - 1;
    hwaParamCfg.dest.dstAIdx =sizeof(cmplx32ImRe_t);
    hwaParamCfg.dest.dstBIdx = 0;
    hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
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

    retVal = HWA_getDMAChanIndex(obj->hwaHandle,
                                  cfg->hwRes.edmaCfg.edmaChainOut.channel,
                                  &destChan);
    if (retVal != 0)
    {
     goto exit;
    }
    /* Now enable interrupt */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChan;
    paramISRConfig.cpu.callbackArg = NULL;
    retVal = HWA_enableParamSetInterrupt(obj->hwaHandle, paramsetIdx, &paramISRConfig);
    if (retVal != 0)
    {
     goto exit;
    }

    obj->hwaParamStopIdx = paramsetIdx;

#if 0//ToDo introduce params save/load approach for power saving mode...
    /* Save HWA Params */
    if (obj->hwaParamsSaveLoc.sizeBytes < (obj->hwaParamStopIdx - obj->hwaParamStartIdx + 1)*HWA_NUM_REG_PER_PARAM_SET*sizeof(uint32_t))
    {
        retVal = DPU_UDOPPROC_EHWA_PARAM_SAVE_LOC_SIZE;
        goto exit;
    }
    retVal = HWA_saveParams(obj->hwaHandle,
                            obj->hwaParamStartIdx,
                            obj->hwaParamStopIdx - obj->hwaParamStartIdx + 1,
                            obj->hwaParamsSaveLoc.data);
    if (retVal != 0)
    {
     goto exit;
    }
#endif

exit:
    return(retVal);
 }

/**
 *  @b Description
 *  @n
 *  EDMA configuration. This EDMA channel perfoms dummy copy, and for N range bins, (N iterations), it loops back the
 *  trigger event (N-1) iterations, and the last iteration (Nth iteration) it triggers the output EDMA
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_UDOP_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
int32_t uDopProc_configEdma_loopBack(DPU_uDopProc_Obj      *obj,
                                         DPU_uDopProc_Config   *cfg)
{

    int32_t                 errorCode = SystemP_SUCCESS;
    EDMA_Handle             handle = obj->edmaHandle;
    int32_t                 idx;
    EDMACCPaRAMEntry        shadowParam;
    uint32_t                numShadowSets;
    uint32_t                shadowStartIdx;
    uint8_t                 numRangeAccumBins = obj->numRangeAccumBins;
    int32_t                 edmaReturn;
    uint32_t                dmaChId;
    uint32_t                tcc;
    bool isFinalTransferInterruptEnabled;
    bool isIntermediateTransferInterruptEnabled;
    bool isFinalChainingEnabled;
    bool isIntermediateChainingEnabled;
    uint32_t                baseAddr, regionId;

    baseAddr = EDMA_getBaseAddr(handle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(handle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaChId = cfg->hwRes.edmaCfg.edmaChainOut.channel;
    edmaReturn = DPEDMA_configDummyChannel(handle,
                                           EDMA_CHANNEL_TYPE_DMA,
                                           &dmaChId,
                                           &dmaChId,
                                           &dmaChId);
    if (edmaReturn != SystemP_SUCCESS)
    {
        errorCode = DPU_UDOPPROC_EEDMA_CONFIG_FAILED;
        goto exit;
    }


    numShadowSets = 2;
    if (numRangeAccumBins > 1)
    {
        shadowStartIdx = 0;
    }
    else
    {
        shadowStartIdx = 1;
    }

    /* Program Shadow Param Sets */
    for (idx = shadowStartIdx; idx < numShadowSets; idx++)
    {
        memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
        shadowParam.srcAddr = DPEDMA_local2Global((uint32_t) &obj->dummySrc);
        shadowParam.destAddr = DPEDMA_local2Global((uint32_t) &obj->dummyDst);
        shadowParam.aCnt = 4;
        shadowParam.bCnt = 1;
        if(idx==0)
        {
            shadowParam.cCnt = numRangeAccumBins - 1;
        }
        else
        {
            shadowParam.cCnt = 1;
        }
        shadowParam.bCntReload = shadowParam.bCnt;

        shadowParam.srcBIdx = 0;
        shadowParam.destBIdx = 0;

        shadowParam.srcCIdx = 0;
        shadowParam.destCIdx = 0;

        if(idx==0)
        {
            tcc = cfg->hwRes.edmaCfg.edmaIn.channel;
        }
        else
        {
            tcc = cfg->hwRes.edmaCfg.edmaMicroDopOut.channel;
        }
        isFinalTransferInterruptEnabled = false;
        isIntermediateTransferInterruptEnabled = false;
        isFinalChainingEnabled = true;
        isIntermediateChainingEnabled = true;

        shadowParam.opt =
            (EDMA_OPT_TCCHEN_MASK |
             ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
             ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK) |
             (((uint32_t)isFinalTransferInterruptEnabled) << EDMA_TPCC_OPT_TCINTEN_SHIFT) |
             (((uint32_t)isIntermediateTransferInterruptEnabled) << EDMA_TPCC_OPT_ITCINTEN_SHIFT) |
             (((uint32_t)isFinalChainingEnabled) << EDMA_TPCC_OPT_TCCHEN_SHIFT) |
             (((uint32_t)isIntermediateChainingEnabled) << EDMA_TPCC_OPT_ITCCHEN_SHIFT)
            );

        EDMASetPaRAM(baseAddr,
                     cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[idx],
                     &shadowParam);
    }

    /**********************************************/
    /* Link physical channel and param sets       */
    /**********************************************/
    if (numRangeAccumBins > 1)
    {
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                            cfg->hwRes.edmaCfg.edmaChainOut.channel,
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[0])) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[0],
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[1])) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[1],
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[0])) != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else
    {
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                            cfg->hwRes.edmaCfg.edmaChainOut.channel,
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[1])) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(handle,
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[1],
                                            cfg->hwRes.edmaCfg.edmaChainOut.ShadowPramId[1])) != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    /********************************/
    /* Bring in the first param set */
    /********************************/
    edmaReturn = EDMAEnableTransferRegion(baseAddr, regionId, cfg->hwRes.edmaCfg.edmaChainOut.channel, EDMA_TRIG_MODE_MANUAL);

    /********************************/
    /* Enable Event in EER register */
    /********************************/
    edmaReturn = EDMAEnableTransferRegion(baseAddr, regionId, cfg->hwRes.edmaCfg.edmaChainOut.channel, EDMA_TRIG_MODE_EVENT);

    if (edmaReturn != TRUE)
    {
       errorCode = DPU_UDOPPROC_EEDMA_CONFIG_FAILED;
       goto exit;
    }


exit:
    return errorCode;
}




/**
 *  @b Description
 *  @n
 *  EDMA configuration.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_UDOP_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static inline int32_t uDopProc_configEdma
(
    DPU_uDopProc_Obj      *obj,
    DPU_uDopProc_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    cmplx16ImRe_t       *radarCubeBase = (cmplx16ImRe_t *)cfg->hwRes.radarCube.data;
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;

    bool isTransferCompletionEnabled;
    bool isIntermediateTransferInterruptEnabled;
    Edma_EventCallback transferCompletionCallbackFxn;
    void* transferCompletionCallbackFxnArg;
    uint8_t numRangeAccumBins = obj->numRangeAccumBins;

    if(obj == NULL)
    {
        retVal = DPU_UDOPPROC_EINVAL;
        goto exit;
    }

    /*****************************************************************************************/
    /**************                     PROGRAM DMA INPUT                    *****************/
    /*****************************************************************************************/
    /******************************************************************************************
    *  PROGRAM DMA channel  to RESET accumulated micro Doppler in HWA
    ******************************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled        = true;

    obj->zeroValue.real = 0;
    obj->zeroValue.imag = 0;
    syncABCfg.srcAddress  = DPEDMA_local2Global((uint32_t)(&obj->zeroValue));
    syncABCfg.destAddress = (uint32_t) obj->hwaMemBankAddr[3] + cfg->staticCfg.numDopplerBins * sizeof(cmplx32ImRe_t);
    syncABCfg.aCount      = sizeof(cmplx32ImRe_t);
    syncABCfg.bCount      = cfg->staticCfg.numDopplerBins;
    syncABCfg.cCount      = 1;
    syncABCfg.srcBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstBIdx     = sizeof(cmplx32ImRe_t);
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaResetIn,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL, //transferCompletionCallbackFxnArg
                                 NULL);//interrupt object

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************
    *  PROGRAM DMA channel to transfer data from radar cube matrix per range bin
    ******************************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]); //This will be updated per frame with the minor motion
    if (cfg->staticCfg.isRxChGainPhaseCompensationEnabled)
    {
        syncABCfg.destAddress = (uint32_t)(obj->hwaMemBankAddr[1]);
    }
    else
    {
        syncABCfg.destAddress = (uint32_t)(obj->hwaMemBankAddr[0]);
    }
    syncABCfg.aCount      = sizeof(cmplx16ImRe_t);
    syncABCfg.bCount      = cfg->staticCfg.numDopplerChirps * cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas;
    syncABCfg.cCount      = numRangeAccumBins;
    syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sizeof(cmplx16ImRe_t);
    syncABCfg.srcCIdx     = sizeof(cmplx16ImRe_t);
    syncABCfg.dstBIdx     = sizeof(cmplx16ImRe_t);
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 false, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL, //transferCompletionCallbackFxnArg
                                 NULL);//interrupt object

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************
    *  PROGRAM DMA Hot Signature
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSig,
                                             obj->hwaHandle,
                                             obj->hwaDmaTriggerSourceChan,
                                             false);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************/
    /**************                      PROGRAM DMA OUTPUT                   *****************/
    /******************************************************************************************/

    /******************************************************************************************
    *  PROGRAM DMA output chaining
    ******************************************************************************************/
    uDopProc_configEdma_loopBack(obj, cfg);


    /******************************************************************************************
    *  PROGRAM DMA  output of micro Doppler
    ******************************************************************************************/
    chainingCfg.chainingChan = cfg->hwRes.edmaCfg.edmaMicroDopOut.channel;
    chainingCfg.isIntermediateChainingEnabled = false;
    chainingCfg.isFinalChainingEnabled         = false;

    isIntermediateTransferInterruptEnabled = true;
    isTransferCompletionEnabled = true;
    transferCompletionCallbackFxn = uDopProc_edmaDoneIsrCallback;
    transferCompletionCallbackFxnArg = (void *)&obj->edmaDoneSemaHandle;;

    syncABCfg.srcAddress  = (uint32_t)obj->hwaMemBankAddr[3] + cfg->staticCfg.numDopplerBins * sizeof(cmplx32ImRe_t) + sizeof(int32_t);
    syncABCfg.destAddress = DPEDMA_local2Global((uint32_t)(cfg->hwRes.uDopplerHwaOutput));
    syncABCfg.aCount      = sizeof(int32_t);
    syncABCfg.bCount      = cfg->staticCfg.numDopplerBins;
    syncABCfg.cCount      = 2;//Ping and Pong
    syncABCfg.srcBIdx     = sizeof(DPU_uDopProc_HwaSumOutput);
    syncABCfg.dstBIdx     = sizeof(uint32_t);
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstCIdx     = cfg->staticCfg.numDopplerBins * sizeof(uint32_t);

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaMicroDopOut,
                                &chainingCfg,
                                &syncABCfg,
                                false, //isEventTriggered
                                isIntermediateTransferInterruptEnabled,
                                isTransferCompletionEnabled,
                                transferCompletionCallbackFxn,
                                transferCompletionCallbackFxnArg,
                                cfg->hwRes.edmaCfg.intrObj);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

exit:
    return(retVal);
} 

/**
  *  @b Description
  *  @n
  *   uDopProc_FeatExtraction
  *
  *  @param[in]  obj    Micro Doppler DPU object
  *  @param[out] outParams  Output result
  *  @param[in]  startSrcIndex doppler index (signed value) of the centroid, (value coming from the tracker)
  *  @param[in]  targetOutIdx index in the processed target in the frame
  *
  *  \ingroup    DPU_UDOPPROC_INTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t uDopProc_FeatExtraction(DPU_uDopProc_Obj *obj,
                                    DPU_uDopProc_OutParams   *outParams,
                                    int32_t          startSrcIndex,
                                    uint32_t         targetOutIdx)
{
    uint32_t i;
    uint32_t numDopBins = obj->numDopplerBins;
    uint32_t numDopBinsMask = numDopBins -1; //NOTE: We assume number of Doppler bins is power of 2
    uint32_t pingPongInd = obj->uDopplerHwaOutputPinPongInd;
    int32_t  retVal = 0;
    float sample, sampleMax, sampleMin;
    int32_t srcIdx;
    int32_t dstIdx = numDopBins/2;
    float scale;

    uint32_t *uDopSrc = &obj->uDopplerHwaOutput[numDopBins * pingPongInd];
    float *uDopDst = &outParams->uDopplerOutput[numDopBins * targetOutIdx];

    FeatExtract_featOutput *featureOut = &outParams->uDopplerFeatures[targetOutIdx];

    /* Make zero Doppler the average of the neighbors */
    uDopSrc[0] = (uint32_t) (0.5 * ((float) uDopSrc[1] + (float) uDopSrc[numDopBins-1]) + 0.5);

    /* copy from uint32 to float buffer and circular shift */
    dstIdx = numDopBins/2;
    if(startSrcIndex < 0)
    {
        srcIdx = startSrcIndex + numDopBins;
    }
    else
    {
        srcIdx = startSrcIndex;
    }

    if (obj->normalizedSpectrum)
    {
        /* Normalized spectrum */
        sampleMax = 0;
        sampleMin = FLT_MAX;
        for (i=0; i < numDopBins; i++)
        {
            sample = (float) uDopSrc[srcIdx];
            uDopDst[dstIdx] = sample;
            if (sample > sampleMax)
            {
                sampleMax = sample;
            }
            if (sample < sampleMin)
            {
                sampleMin = sample;
            }
            srcIdx = (srcIdx+1) & numDopBinsMask;
            dstIdx = (dstIdx+1) & numDopBinsMask;
        }

        scale = (sampleMax - sampleMin);
        if (scale > 0)
        {
            scale = 1./scale;
        }
        if (obj->magnitudeSquared)
        {
            scale = scale * scale;
            for (i = 0; i < numDopBins; i++)
            {
                uDopDst[i] = (uDopDst[i]-sampleMin) * (uDopDst[i]-sampleMin) * scale;
            }
        }
        else
        {
            for (i = 0; i < numDopBins; i++)
            {
                uDopDst[i] = (uDopDst[i]-sampleMin) * scale;
            }
        }
    }
    else
    {
        /* Non-normalized spectrum */
        if (obj->magnitudeSquared)
        {
            for (i=0; i < numDopBins; i++)
            {
                uDopDst[dstIdx] = (float) uDopSrc[srcIdx] * (float) uDopSrc[srcIdx];
                srcIdx = (srcIdx+1) & numDopBinsMask;
                dstIdx = (dstIdx+1) & numDopBinsMask;
            }
        }
        else
        {
            for (i=0; i < numDopBins; i++)
            {
                uDopDst[dstIdx] = (float) uDopSrc[srcIdx];
                srcIdx = (srcIdx+1) & numDopBinsMask;
                dstIdx = (dstIdx+1) & numDopBinsMask;
            }
        }
    }

    /* Feature extraction */
    featExtract_compute(obj->featExtHandle, uDopDst, featureOut);

    obj->uDopplerHwaOutputPinPongInd = pingPongInd ^ 0x1;

    return retVal;
}

/**
  *  @b Description
  *  @n
  *   uDopProc_ConfigTriggerHwa
  *
  *  @param[in]  ToDo Add API params
  *  @param[in]
  *
  *  \ingroup    DPU_UDOPPROC_INTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t uDopProc_ConfigTriggerHwa
(
    DPU_uDopProc_Obj         *obj,
    DPIF_RadarCube           *radarCube,
    DPIF_DetMatrix           *detMatrix,
    DPU_uDopProc_TrackerData *trackerData,
    DPU_uDopProc_OutParams   *outParams,
    uint32_t                 idx,
    int32_t                  *dopCentroid
)
{
    int32_t             retVal = 0;
    float oneOverRangeStep = obj->oneOverRangeStep;
    float oneOverDopplerStep = obj->oneOverDopplerStep;
    float targetSize = obj->targetSize;
    int32_t numRangeAccumBins = obj->numRangeAccumBins;
    int32_t numRangeBins = obj->numRangeBins;
    int32_t maxNumAzimAccumBins = obj->maxNumAzimAccumBins;
    uint32_t *radarCubeData = (uint32_t *) (radarCube->data);

    uint32_t twiddleStepHalf = obj->twiddleStepHalf;
    uint32_t log2TwiddleStep = obj->log2TwiddleStep;

    HWA_CommonConfig    hwaCommonConfig;
    uint32_t srcBuffAddr;
    float oneOverRange;
    float x, y, z, range;
    float vx, vy, vz, doppler;
    float temp;
    int32_t rangeCentreIdx, rangeStartIdx;
    int32_t azimuthCenterIdx,  numAzimAccumBins, azimuthStartIdx, azimuthTargetHalfWidthIdx;
    int32_t hwaTwidincr;

    x = trackerData->tList[idx].posX;
    y = trackerData->tList[idx].posY;
    z = trackerData->tList[idx].posZ;
    range = sqrtf(x*x + y*y + z*z); //ToDo For now assume no elevation
    oneOverRange = 1./range;

    if (obj->circShiftAroundCentroid)
    {
        vx = trackerData->tList[idx].velX;
        vy = trackerData->tList[idx].velY;
        vz = trackerData->tList[idx].velZ;
        doppler = ((x*vx) + (y*vy) + (z*vz)) * oneOverRange;
        if (doppler > 0)
        {
            *dopCentroid = (int32_t) (doppler * oneOverDopplerStep + 0.5);
        }
        else
        {
            *dopCentroid = (int32_t) (doppler * oneOverDopplerStep - 0.5);
        }
    }
    else
    {
        *dopCentroid = 0;
    }

    rangeCentreIdx = (int32_t) (range * oneOverRangeStep + 0.5);
    rangeStartIdx = rangeCentreIdx - (int32_t) (numRangeAccumBins-1)/2;
    if (rangeStartIdx < 1)
    {
        rangeStartIdx = 1;  //ToDo check if there is programmable number of left range bins to skip
    }
    if ((rangeStartIdx + numRangeAccumBins) > (numRangeBins - 1))
    {
        rangeStartIdx = (numRangeBins - 1) - numRangeAccumBins;  //ToDo check if there is programmable number of right range bins to skip
    }


    if(obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
    {
        //Azimuth FFT target center index
        if (x > 0)
        {
            azimuthCenterIdx =  (int32_t) (16384. * 0.5 * x * oneOverRange + 0.5);
        }
        else
        {
            azimuthCenterIdx =  (int32_t) (16384. * 0.5 * x * oneOverRange - 0.5);
        }

        //Number of azimuth accumulations:
        temp = 1. - x*x*oneOverRange*oneOverRange;
        if (temp < 0)
        {
            temp = 0;
        }
        else
        {
            temp = sqrtf(temp);
        }
        azimuthTargetHalfWidthIdx = (int32_t) (16384. * 0.5 * temp * targetSize * 0.5 * oneOverRange + 0.5);
        numAzimAccumBins = (2*azimuthTargetHalfWidthIdx + twiddleStepHalf) >> log2TwiddleStep;
        if (numAzimAccumBins > maxNumAzimAccumBins)
        {
            numAzimAccumBins = maxNumAzimAccumBins;
        }
        else if (numAzimAccumBins <= 0)
        {
            numAzimAccumBins = 1;
        }

        //Frequency shifter in HWA
        azimuthStartIdx = azimuthCenterIdx - azimuthTargetHalfWidthIdx;
        hwaTwidincr = azimuthStartIdx ;
    }
    else
    {
        if (x > 0)
        {
            hwaTwidincr =  (int32_t) (16384. * 0.5 * x * oneOverRange + 0.5);
        }
        else
        {
            hwaTwidincr =  (int32_t) (16384. * 0.5 * x * oneOverRange - 0.5);
        }
    }
    if (hwaTwidincr < 0)
    {
        hwaTwidincr = hwaTwidincr + 16384;
    }

    //Program EDMA source address of the starting range index (rangeStartIdx) into radar cube
    srcBuffAddr = (uint32_t) (&radarCubeData[rangeStartIdx]);
    retVal = DPEDMA_setSourceAddress(obj->edmaHandle,
                                     obj->edmaIn.channel,
                                     srcBuffAddr);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle,0);
    if (retVal != 0)
    {
        goto exit;
    }

    if(obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
    {
        obj->hwaAzimuthSpectrumParamSetCfg.dest.dstAcnt = numAzimAccumBins - 1;
        obj->hwaAzimuthSpectrumParamSetCfg.dest.dstBIdx = numAzimAccumBins * sizeof(uint32_t);
    }
    obj->hwaAzimuthSpectrumParamSetCfg.complexMultiply.cmpMulArgs.twidIncrement = hwaTwidincr;
    //Program azimuth FFT HWA param set:  dstAcnt = numAzimAccumBins, frequency shifter (hwaTwidincr)
    retVal = HWA_configParamSet(obj->hwaHandle,
                                obj->hwaAzimuthSpectrumParamSetIdx,
                                &obj->hwaAzimuthSpectrumParamSetCfg, NULL);
    if (retVal != 0)
    {
        goto exit;
    }
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                          obj->hwaAzimuthSpectrumParamSetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
      goto exit;
    }

    if(obj->azimuthProcessingApproach == DPU_UDOPPROC_AZIMUTH_FFT_APPROACH)
    {
        //Program azimuth bin accumulation param set:
        obj->hwaAzimuthBinAccParamSetCfg.source.srcAcnt = numAzimAccumBins - 1;
        obj->hwaAzimuthBinAccParamSetCfg.source.srcBIdx = numAzimAccumBins * sizeof(uint32_t);
        retVal = HWA_configParamSet(obj->hwaHandle,
                                    obj->hwaAzimuthBinAccParamSetIdx,
                                    &obj->hwaAzimuthBinAccParamSetCfg, NULL);
        if (retVal != 0)
        {
            goto exit;
        }
        retVal = HWA_disableParamSetInterrupt(obj->hwaHandle,
                                              obj->hwaAzimuthBinAccParamSetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
          goto exit;
        }
    }

    // Configure Common Registers
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFTSUMDIV;

    hwaCommonConfig.numLoops      = obj->hwaNumLoops;
    hwaCommonConfig.paramStartIdx = obj->hwaParamStartIdx;
    hwaCommonConfig.paramStopIdx  = obj->hwaParamStopIdx;
    hwaCommonConfig.fftConfig.fftSumDiv = 0;

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

    /* Trigger EDMA */
    retVal  = DPEDMA_startTransfer(obj->edmaHandle, obj->edmaResetIn.channel);
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

exit:
    return retVal;
}

/*===========================================================
 *                    Micro Doppler DPU  External APIs
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
 *  \ingroup    DPU_UDOPPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
DPU_uDopProc_Handle DPU_uDopProc_init
(
    DPU_uDopProc_InitParams *initCfg,
    int32_t                    *errCode
)
{
    DPU_uDopProc_Obj  *obj = NULL;
    HWA_MemInfo             hwaMemInfo;
    uint32_t                i;
    int32_t                 status = SystemP_SUCCESS;

    *errCode       = 0;
    
    if((initCfg == NULL) || (initCfg->hwaHandle == NULL))
    {
        *errCode = DPU_UDOPPROC_EINVAL;
        goto exit;
    }    

    /* Allocate memory */
    obj = (DPU_uDopProc_Obj *) &guDopProcHeapMem;
    if(obj == NULL)
    {
        *errCode = DPU_UDOPPROC_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)obj, 0U, sizeof(DPU_uDopProc_Obj));
    
    //printf("UDOP DPU: (DPU_uDopProc_Obj *) 0x%08x\n", (uint32_t) obj);

    /* Save init config params */
    obj->hwaHandle   = initCfg->hwaHandle;

    /* Create DPU semaphores */
    status = SemaphoreP_constructBinary(&obj->edmaDoneSemaHandle, 0);
    if(SystemP_SUCCESS != status)
    {
        *errCode = DPU_UDOPPROC_ESEMA;
        goto exit;
    }


    /* Populate HWA base addresses and offsets. This is done only once, at init time.*/
    *errCode =  HWA_getHWAMemInfo(obj->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {       
        goto exit;
    }
    
    /* check if we have enough memory banks*/
    if(hwaMemInfo.numBanks < DPU_UDOPPROC_NUM_HWA_MEMBANKS)
    {    
        *errCode = DPU_UDOPPROC_EHWARES;
        goto exit;
    }
    
    for (i = 0; i < DPU_UDOPPROC_NUM_HWA_MEMBANKS; i++)
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
   return ((DPU_uDopProc_Handle)obj);
}

/**
  *  @b Description
  *  @n
  *   UDOP DPU configuration
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    DPU_UDOPPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_uDopProc_config
(
    DPU_uDopProc_Handle    handle,
    DPU_uDopProc_Config    *cfg
)
{
    DPU_uDopProc_Obj    *obj;
    int32_t             retVal = 0;
	uint32_t twiddleStep;
	FeatExtract_moduleConfig featExtCfg;

    obj = (DPU_uDopProc_Obj *)handle;
    if(obj == NULL)
    {
        retVal = DPU_UDOPPROC_EINVAL;
        goto exit;
    }
    
#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle
      )
    {
        retVal = DPU_UDOPPROC_EINVAL;
        goto exit;
    }

    /* Check if radar cube format is supported by DPU*/
    if(cfg->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_6)
    {
        retVal = DPU_UDOPPROC_ECUBEFORMAT;
        goto exit;
    }

    /* Check if radar cube column fits into one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_UDOPPROC_EEXCEEDHWAMEM;
        goto exit;
    }
#endif

    /* EDMA parameters needed to trigger first EDMA transfer*/
    obj->edmaHandle  = cfg->hwRes.edmaCfg.edmaHandle;
    memcpy((void*)(&obj->edmaIn), (void *)(&cfg->hwRes.edmaCfg.edmaIn), sizeof(DPEDMA_ChanCfg));
    memcpy((void*)(&obj->edmaResetIn), (void *)(&cfg->hwRes.edmaCfg.edmaResetIn), sizeof(DPEDMA_ChanCfg));

    obj->oneOverRangeStep = 1./cfg->staticCfg.rangeStep;
    obj->oneOverDopplerStep = 1./cfg->staticCfg.dopplerStep;
    obj->targetSize = cfg->staticCfg.cliCfg.targetSize;
    obj->numAzimuthBins = cfg->staticCfg.azimuthFftSize;
    obj->numRangeBins = cfg->staticCfg.numRangeBins;
    obj->maxNumAzimAccumBins = cfg->staticCfg.maxNumAzimAccumBins;
    obj->numDopplerBins = cfg->staticCfg.numDopplerBins;
    obj->uDopplerHwaOutput = cfg->hwRes.uDopplerHwaOutput;
    obj->azimuthProcessingApproach = cfg->staticCfg.cliCfg.genApproach;
    obj->magnitudeSquared  = cfg->staticCfg.cliCfg.magnitudeSquared;
    obj->circShiftAroundCentroid = cfg->staticCfg.cliCfg.circShiftAroundCentroid;
    obj->normalizedSpectrum = cfg->staticCfg.cliCfg.normalizedSpectrum;
    obj->hwaParamsSaveLoc = cfg->hwRes.hwaCfg.hwaParamsSaveLoc; //Copy structure

    obj->maxNumTracks = cfg->staticCfg.maxNumTracks;
    obj->microDopplerClassifierCliCfg =cfg->staticCfg.microDopplerClassifierCliCfg;

    /* For shifting azimuth FFT */
    twiddleStep = 16384 / obj->numAzimuthBins;
    obj->log2TwiddleStep = mathUtils_ceilLog2(twiddleStep);
    obj->twiddleStepHalf = twiddleStep / 2;
    
    obj->numRangeAccumBins = (uint8_t) ((obj->targetSize * obj->oneOverRangeStep) + 0.5);
    if ( !(obj->numRangeAccumBins & 0x1))
    {
        /* Make it odd number. Accumulation span: [-(obj->numRangeAccumBins-1)/2 : +(obj->numRangeAccumBins-1)/2] + targetRangeIndex */
        obj->numRangeAccumBins++;
    }

    /*HWA parameters needed for the HWA common configuration*/
    obj->hwaNumLoops      = obj->numRangeAccumBins;
    obj->hwaParamStartIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;    
    obj->uDopplerHwaOutputPinPongInd = 0;


    /* Set configuration for Micro-doppler Feature Extraction */
    if (cfg->isFirstTimeCfg)
    {
        featExtCfg.freqStep = cfg->staticCfg.dopplerStep;
        featExtCfg.numFreqBins = cfg->staticCfg.numDopplerBins;
        featExtCfg.scratchBuffer = (float *) cfg->hwRes.scratchBuf.data;
        featExtCfg.scratchBufferSizeInBytes = cfg->hwRes.scratchBuf.sizeBytes;
        featExtCfg.pwrThre.low = cfg->staticCfg.cliCfg.interceptThrLowFreq;
        featExtCfg.pwrThre.up = cfg->staticCfg.cliCfg.interceptThrUpFreq;
        featExtCfg.specShiftMode = cfg->staticCfg.cliCfg.specShiftMode;
        /* Configure Feature Extraction from the Micro-doppler spectrum */
        obj->featExtHandle = featExtract_create(&featExtCfg, &retVal);
        if (retVal != 0)
        {
            goto exit;
        }
    }

    if (obj->microDopplerClassifierCliCfg.enabled && cfg->isFirstTimeCfg)
    {
        Classifier_moduleConfig classifierCfg;
        classifierCfg.num_classes = CLASSIFIER_NUM_CLASSES;
        classifierCfg.num_features = CLASSIFIER_NUM_FEATURES;
        classifierCfg.num_frames = CLASSIFIER_NUM_FRAMES;
        classifierCfg.scratchBuffer = cfg->hwRes.scratchBuf.data;
        classifierCfg.scratchBufferSizeInBytes = cfg->hwRes.scratchBuf.sizeBytes;

        /* Allocated memory for feature circular buffer per track ID */
        obj->featureObj = cfg->hwRes.featureObj;

        /* Linearized feature buffer, input to classifier */
        obj->featureLinBuf = (float *) cfg->hwRes.scratchBuf2.data;
        if (cfg->hwRes.scratchBuf2.sizeBytes < (CLASSIFIER_NUM_FRAMES * CLASSIFIER_NUM_FEATURES * sizeof(float)))
        {
            retVal = DPU_UDOPPROC_ENOMEM;
            goto exit;
        }


        /* Initialize classifier */
        obj->classifierHandle = classifier_create(&classifierCfg, &retVal);
        if (retVal != 0)
        {
            goto exit;
        }
        {
            obj->featureSelectionInd[0]=0;
            obj->featureSelectionInd[1]=1;
            obj->featureSelectionInd[2]=2;
            obj->featureSelectionInd[3]=5;
        }
    }


    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }
#if 0
    //ToDo This is shared with DoA DPU, no need to do it here
    if(cfg->staticCfg.isRxChGainPhaseCompensationEnabled)
    {
        /* HWA window configuration */
        retVal = HWA_configRam(obj->hwaHandle,
                               HWA_RAM_TYPE_INTERNAL_RAM,
                               (uint8_t *)cfg->staticCfg.compRxChanCfg.rxChPhaseComp,
                               cfg->staticCfg.numRxAntennas * cfg->staticCfg.numTxAntennas *sizeof(cmplx32ImRe_t), //size in bytes
                               0);
        if (retVal != 0)
        {
            goto exit;
        }
    }
#endif
    /*******************************/
    /**  Configure HWA            **/
    /*******************************/
    /*Compute source DMA channels that will be programmed in both HWA and EDMA.   
      The DMA channels are set to be equal to the paramSetIdx used by HWA*/
    obj->hwaDmaTriggerSourceChan = cfg->hwRes.hwaCfg.dmaTrigSrcChan;
    retVal = uDopProc_configHwa(obj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /*******************************/
    /**  Configure EDMA           **/
    /*******************************/
    retVal = uDopProc_configEdma(obj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
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
int32_t DPU_uDopProc_GetNumUsedHwaParamSets
(
    DPU_uDopProc_Handle    handle,
    uint8_t *numUsedHwaParamSets
)
{
    DPU_uDopProc_Obj *obj;
    int32_t retVal = 0;

    obj = (DPU_uDopProc_Obj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_UDOPPROC_EINVAL;
        goto exit;
    }
    *numUsedHwaParamSets = (uint8_t) (obj->hwaParamStopIdx - obj->hwaParamStartIdx + 1);
exit:
    return retVal;
}

/**
*  @b Description
*  @n Computes the average (element by element) of two arrays. It is used to interpolate
*     features for the previous frame if the features were not taken as reliable (when the number
*     of point-cloud points was below threshold)
*
*  @param[in]   f1       Array 1
*  @param[in]   f2       Array 2
*  @param[in]   length   length of the arrays
*  @param[out]  fAvg     Array average of two input arrays
*
*  @retval
*      none
*/
void uDopProc_featureAverage(float *f1,
                    float *f2,
                    uint32_t length,
                    float *fAvg)
{
    uint32_t i;
    for (i = 0; i < length; i++)
    {
        fAvg[i] = 0.5 * (f1[i] + f2[i]);
    }
}

/**
*  @b Description
*  @n Returns the index of the previous frame in the circular feature buffer
*
*  @param[in]   frmInd  index of the current frame
*
*  @retval
*      frmIndPrev   index of the previous frame
*/
int32_t uDopProc_getPrevFrmNum(int32_t frmInd)
{
    int32_t frmIndPrev = frmInd - 1;
    if (frmIndPrev < 0)
    {
        frmIndPrev += CLASSIFIER_NUM_FRAMES;
    }
    return frmIndPrev;
}

/**
*  @b Description
*  @n Returns the index of the next frame in the circular feature buffer
*
*  @param[in]   frmInd  index of the current frame
*
*  @retval
*      frmIndNext   index of the next frame
*/
static int32_t uDopProc_getNextFrmNum(int32_t frmInd)
{
    int32_t frmIndNext = frmInd + 1;
    if (frmIndNext == CLASSIFIER_NUM_FRAMES)
    {
        frmIndNext = 0;
    }
    return frmIndNext;
}


/**
*  @b Description
*  @n The function updates feature object. It operates on a per-track ID basis, and stores
*  the selected features from the input feature set into the feature circular buffer if
*  certain conditions are met. Please refer to the accompanying flow chart for further details.
*
*  @param[inout] obj                 pointer to feature object
*  @param[in]    featureSet          pointer to feature set (currently 6 features)
*  @param[in]    frmInd              current frame index, (modulo CLASSIFIER_NUM_FRAMES)
*  @param[in]    featureSelectInd    array with indices of used features in the featureSet
*  @param[in]    numPoints           number of point-cloud points associated with the track ID
*  @param[in]    minNumPoints        minimum number of points required for the feature set to be placed in the circular buffer
*
*  @retval
*      none
*/
void uDopProc_updateTrackIdFeatures(DPU_uDopProc_FeatureObj *obj,
                           FeatExtract_featOutput *featureSet,
                           int32_t frmInd,
                           uint8_t *featureSelectInd,
                           uint32_t numPoints,
                           uint32_t minNumPoints)
{
    int32_t frmInd1, frmInd2;
    uint32_t i;
    float *featSrc, *featDst;

    obj->numPresent -= obj->presentDelayLine[frmInd];
    obj->numMisses -= obj->missDelayLine[frmInd];
    obj->numCumMisses -= obj->cumMissDelayLine[frmInd];
    if (featureSet != NULL)
    {
        /* FEATURES PRESENT */
        obj->presentDelayLine[frmInd] = 1;
        obj->numPresent++;

        if (numPoints >= minNumPoints)
        {
            obj->missDelayLine[frmInd] = 0;
            obj->cumMissDelayLine[frmInd] = 0;
            /* Copy selected features */
            featDst = (float *) obj->featureDelayLine[frmInd].features;
            featSrc = (float *) featureSet;
            for(i=0; i<CLASSIFIER_NUM_FEATURES; i++)
            {
                featDst[i] = featSrc[featureSelectInd[i]];
            }

            /* If there was a missing feature extraction in previous frame and it was present */
            frmInd1 = uDopProc_getPrevFrmNum(frmInd);
            if ((obj->missDelayLine[frmInd1]) && obj->presentDelayLine[frmInd1])
            {
                /* There was a miss in previous, average features from current and from two frames before */
                frmInd2 = uDopProc_getPrevFrmNum(frmInd1);
                if (!obj->missDelayLine[frmInd2] && obj->presentDelayLine[frmInd2])
                {
                    /* Calculate features in previous frame as interpolation between the current frame and the frame before previous frame */
                    uDopProc_featureAverage((float *)&obj->featureDelayLine[frmInd],   //frame(n)
                                   (float *)&obj->featureDelayLine[frmInd2],  //frame(n-2)
                                   CLASSIFIER_NUM_FEATURES,
                                   (float *)&obj->featureDelayLine[frmInd1]); //frame(n-1)
                    obj->missDelayLine[frmInd1] = 0;
                    obj->numMisses--;
                }
            }
        }
        else
        {
            obj->missDelayLine[frmInd] = 1;
            obj->numMisses++;
            obj->cumMissDelayLine[frmInd] = 1;
            obj->numCumMisses++;
        }
    }
    else
    {
        /* Tracker ID was not present, features not provided */
        obj->presentDelayLine[frmInd] = 0;

        obj->missDelayLine[frmInd] = 1;
        obj->numMisses++;
        obj->cumMissDelayLine[frmInd] = 1;
        obj->numCumMisses++;
    }
}

/**
*  @b Description
*  @n The function copies features from circular buffer to linear buffer.
*
*  @param[out] linBuf   pointer to linear buffer
*  @param[in]  circBuf  pointer to circular buffer
*  @param[in]  curInd   current index, index to the newest element in the buffer
*
*  @retval
*      none
*/
void uDopProc_featuresCircularToLinear(float *linBuf, float *circBuf, uint32_t curInd)
{
    uint32_t copyInd;
    if (curInd == (CLASSIFIER_NUM_FRAMES-1))
    {
        memcpy(linBuf, circBuf, CLASSIFIER_NUM_FRAMES * CLASSIFIER_NUM_FEATURES * sizeof(float));
        return;
    }

    copyInd = (curInd + 1) * CLASSIFIER_NUM_FEATURES;
    memcpy(linBuf,
           &circBuf[copyInd],
           (CLASSIFIER_NUM_FRAMES * CLASSIFIER_NUM_FEATURES - copyInd) * sizeof(float));
    memcpy(&linBuf[CLASSIFIER_NUM_FRAMES * CLASSIFIER_NUM_FEATURES - copyInd],
           circBuf,
           copyInd * sizeof(float));
}

/**
*  @b Description
*  @n The function counts the number of major motion point-cloud points associated with the specified track ID.
*
*  @param[in]  trackId   track ID
*  @param[in]  trackerData  pointer to tracker output data, that includes the target index array
*
*  @retval
*      numPoints    number of point-cloud points associated with the specified track ID
*/
uint32_t uDopProc_countPointCloudPoints(uint32_t trackId, DPU_uDopProc_TrackerData *trackerData)
{
    uint32_t numPoints = 0;
    uint32_t i;

    for (i = 0; i < trackerData->numIndicesMajorMotion; i++)
    {
        if ((uint32_t) trackerData->tIndex[i] == trackId)
        {
            numPoints++;
        }
    }
    return numPoints;
}

/**
*  @b Description
*  @n The function checks if the specified track ID is present, and if yes it returns its index position in the list.
*
*  @param[in]  trackId   track ID
*  @param[in]  trackerData  pointer to tracker output data. It includes the target index array.
*
*  @retval
*      true track ID is present in the list
*  @retval
*      false track ID is not present in the list
*/
bool uDopProc_isTrackIdPresent(uint32_t trackId, DPU_uDopProc_TrackerData *trackerData, uint32_t *trackIndInFrame)
{
    bool isPresent = false;
    uint32_t i;
    for(i = 0; i < trackerData->numTargets; i++)
    {
        if (trackId == trackerData->tList[i].tid)
        {
            isPresent = true;
            *trackIndInFrame = i;
            break;
        }
    }
    return isPresent;
}

/**
*  @b Description
*  @n UDOP DPU process function. Computes the range/azimuth detection matrix
*
*  @param[in]   handle        DPU handle
*  @param[in]   radarCubeSrc  Structure descriptor of the input radar cube
*  @param[in]   detMatrix     Pointer to output detection matrix.
*  @param[out]  outParams     Output parameters.
*
*  \ingroup    DPU_UDOPPROC_EXTERNAL_FUNCTION
*
*  @retval
*      Success     =0
*  @retval
*      Error      !=0
*/
int32_t DPU_uDopProc_process
(
    DPU_uDopProc_Handle      handle,
    DPIF_RadarCube           *radarCube,
    DPIF_DetMatrix           *detMatrix,
    DPU_uDopProc_TrackerData *trackerData,
    DPU_uDopProc_OutParams   *outParams
)
{
    bool                status;
    volatile uint32_t   startTime;
    uint32_t inIdx, outIdx;
    uint32_t numTargets;
    int32_t dopCentroidList[TRACKER_MAX_NUM_TR];
    int32_t dopCentroid;

    DPU_uDopProc_Obj *obj;
    int32_t             retVal = 0;

    obj = (DPU_uDopProc_Obj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_UDOPPROC_EINVAL;
        goto exit;
    }
    /* Set inProgress state */
    obj->inProgress = true;

#if 0
    /* Load HWA param sets */
    retVal = HWA_loadParams(obj->hwaHandle,
                            obj->hwaParamStartIdx,
                            obj->hwaParamStopIdx - obj->hwaParamStartIdx + 1,
                            obj->hwaParamsSaveLoc.data);
#endif
    startTime = Cycleprofiler_getTimeStamp();

    numTargets = trackerData->numTargets;
    outParams->numTargets = numTargets;
    outParams->numDopplerBins = obj->numDopplerBins;
    if (outParams->numTargets > 0)
    {
        inIdx = 0;
        retVal = uDopProc_ConfigTriggerHwa(obj,
                                            radarCube,
                                            detMatrix,
                                            trackerData,
                                            outParams,
                                            inIdx,
                                            &dopCentroid);
        dopCentroidList[inIdx] = dopCentroid;
        inIdx++;

        for (outIdx = 0; outIdx < numTargets; outIdx++)
        {
            /**********************************************/
            /* WAIT FOR EDMA DONE INTERRUPT               */
            /**********************************************/
            status = SemaphoreP_pend(&obj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);
            if (status != SystemP_SUCCESS)
            {
                retVal = DPU_UDOPPROC_ESEMASTATUS;
                goto exit;
            }

            /* Disable the HWA */
            retVal = HWA_enable(obj->hwaHandle, 0);
            if (retVal != 0)
            {
                goto exit;
            }


            if (inIdx < numTargets)
            {
                retVal = uDopProc_ConfigTriggerHwa(obj,
                                                    radarCube,
                                                    detMatrix,
                                                    trackerData,
                                                    outParams,
                                                    inIdx,
                                                    &dopCentroid);
                dopCentroidList[inIdx] = dopCentroid;
                inIdx++;
            }

            /* Extract 6 features from row micro Doppler array */
            retVal = uDopProc_FeatExtraction(obj,
                                                 outParams,
                                                 dopCentroidList[outIdx],
                                                 outIdx);
        }
    }
    outParams->stats.featureExtractTime_cpuCycles = Cycleprofiler_getTimeStamp() - startTime;

    /*  Fill features of the targets into corresponding delay lines and call Classifier */
    outParams->numClassifiedTargets = 0;
    if (obj->microDopplerClassifierCliCfg.enabled)
    {
        uint32_t trackId;
        uint32_t trackIndInFrame;
        uint32_t classifierStartTime = Cycleprofiler_getTimeStamp();
        int32_t wrInd;
        uint32_t numPoints;

        wrInd = obj->frameCntrModClassifierNumFrames;

        /* Scan all possible (allocated) Track IDs */
        for (trackId=0; trackId < obj->maxNumTracks; trackId++)
        {
            numPoints = uDopProc_countPointCloudPoints(trackId, trackerData);
            if(uDopProc_isTrackIdPresent(trackId, trackerData, &trackIndInFrame))
            {
                //Track ID is present in the current frame
                uDopProc_updateTrackIdFeatures(&obj->featureObj[trackId],
                                      &outParams->uDopplerFeatures[trackIndInFrame], //Six features detected in the frame
                                      wrInd,
                                      obj->featureSelectionInd,
                                      numPoints,
                                      obj->microDopplerClassifierCliCfg.minNumPntsPerTrack);
            }
            else
            {
                //Track ID is not present in the current frame
                uDopProc_updateTrackIdFeatures(&obj->featureObj[trackId],
                                      NULL,
                                      wrInd,
                                      obj->featureSelectionInd,
                                      numPoints,
                                      obj->microDopplerClassifierCliCfg.minNumPntsPerTrack);
            }
            if ((obj->featureObj[trackId].numPresent == CLASSIFIER_NUM_FRAMES) &&
                (obj->featureObj[trackId].numMisses == 0) &&
                (obj->featureObj[trackId].numCumMisses <= obj->microDopplerClassifierCliCfg.missTotFrmThre))
            {
                /* Copy features from circular array to linear array */
                uDopProc_featuresCircularToLinear(obj->featureLinBuf,
                                         (float *) obj->featureObj[trackId].featureDelayLine[0].features,
                                         wrInd);
                /* CNN Classifier */
                classifier_predict(obj->classifierHandle,
                                   obj->featureLinBuf,
                                   &obj->featureObj[trackId].predictDelayLine[wrInd][0]);
            }
            else
            {
                obj->featureObj[trackId].predictDelayLine[wrInd][0] = 0.5;
                obj->featureObj[trackId].predictDelayLine[wrInd][1] = 0.5;
            }
        }

        /* Send out to host only along with tracker output, trackIds are in the same order as in tracker TLV */
        outParams->numClassifiedTargets = trackerData->numTargets;
        for (outIdx = 0; outIdx < trackerData->numTargets; outIdx++)
        {
            trackId = trackerData->tList[outIdx].tid;
            outParams->classifierOutput.predictions[outIdx].prediction[0] = (uint8_t) round(128 * obj->featureObj[trackId].predictDelayLine[wrInd][0]);
            outParams->classifierOutput.predictions[outIdx].prediction[1] = (uint8_t) round(128 * obj->featureObj[trackId].predictDelayLine[wrInd][1]);
        }

        wrInd = uDopProc_getNextFrmNum(wrInd);
        obj->frameCntrModClassifierNumFrames = wrInd;
        outParams->stats.classifierTime_cpuCycles = (Cycleprofiler_getTimeStamp() - classifierStartTime);
    }

    outParams->stats.numProcess++;

exit:
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
  *  \ingroup    DPU_UDOPPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_uDopProc_deinit(DPU_uDopProc_Handle handle)
{
    int32_t     retVal = 0;
    
    /* Free memory */
    if(handle == NULL)
    {
        retVal = DPU_UDOPPROC_EINVAL;
    }
    
    return retVal;
}



