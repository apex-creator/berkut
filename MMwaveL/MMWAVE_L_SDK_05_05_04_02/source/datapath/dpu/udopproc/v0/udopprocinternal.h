/**
 *   @file  udopproc.h
 *
 *   @brief
 *      Implements Data path Micro Doppler processing functionality.
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
#ifndef UDOPPROC_INTERNAL_H
#define UDOPPROC_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */
#include <drivers/hwa.h>

/* DPIF Components Include Files */
#include <datapath/dpif/dpif_detmatrix.h>
#include <datapath/dpif/dpif_radarcube.h>
#include <datapath/dpif/dp_error.h>

#include <datapath/dpu/udopproc/v0/udopproc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  dopplerProc DPU internal data Object
 *
 * @details
 *  The structure is used to hold dopplerProc internal data object
 *
 *  \ingroup DPU_DOPPLERPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_hwaLoopCfg_t
{
    /*! @brief  HWA number of loops */
    uint16_t hwaNumLoops;

    /*! @brief  HWA start paramset index */
    uint8_t  hwaParamStartIdx;

    /*! @brief  HWA stop paramset index */
    uint8_t  hwaParamStopIdx;
} DPU_uDopProc_hwaLoopCfg;

/**
 * @brief
 *  dopplerProc DPU internal data Object
 *
 * @details
 *  The structure is used to hold dopplerProc internal data object
 *
 *  \ingroup DPU_DOPPLERPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_Obj_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
    
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;
    uint32_t    edmaInstanceId;

    /*! @brief  EDMA resets acummulate micro doppler in HWA memory */
    DPEDMA_ChanCfg edmaResetIn;

    /*! @brief  EDMA to transfer data from radar cube to HWA memory */
    DPEDMA_ChanCfg edmaIn;

    /*! @brief  EDMA configuration for data output from HWA - Detection matrix */
    DPEDMA_ChanCfg edmaDetMatOut;

    /*! @brief  EDMA configuration for data in */
    DPEDMA_ChanCfg edmaInterLoopIn;

    /*! @brief   Dummy location */
    uint32_t dummySrc;
    /*! @brief   Dummy location  */
    uint32_t dummyDst;

    /*! @brief EDMA Done semaphore Handle */
    SemaphoreP_Object  edmaDoneSemaHandle;
    
    /*! @brief Flag to indicate if DPU is in processing state */
    bool inProgress;

    /*! @brief  DMA trigger source channel for Ping param set */
    uint8_t hwaDmaTriggerSourceChan;

    /*! @brief  HWA number of loops */
    uint16_t hwaNumLoops;
    
    /*! @brief  HWA start paramset index */
    uint8_t  hwaParamStartIdx;
    
    /*! @brief  HWA stop paramset index */
    uint8_t  hwaParamStopIdx;
    
    /*! @brief  HWA memory bank addresses */
    uint32_t hwaMemBankAddr[DPU_UDOPPROC_NUM_HWA_MEMBANKS];

    /*! @brief   Azimuth Spectrum ParamSet index */
    uint8_t hwaAzimuthSpectrumParamSetIdx;
    /*! @brief   Azimuth Spectrum ParamSet configuration */
    HWA_ParamConfig hwaAzimuthSpectrumParamSetCfg;

    /*! @brief   Azimuth bin accumulation ParamSet index */
    uint8_t hwaAzimuthBinAccParamSetIdx;
    /*! @brief   Azimuth bin accumulation  ParamSet configuration */
    HWA_ParamConfig hwaAzimuthBinAccParamSetCfg;

    /*! @brief   Field with value equal to zero to be copied to HWA memory to reset accumulated micro doppler */
    cmplx32ImRe_t zeroValue;

    /*! @brief   Number of range bins around the centroid over which the micro doppler is accumulated */
    uint8_t     numRangeAccumBins;

    /*! @brief   RangeStep reciprocal value to convert range (in meters) to range FFT index */
    float oneOverRangeStep;

    /*! @brief   RangeStep reciprocal value to convert Doppler (in meters/sec) to Doppler FFT index */
    float oneOverDopplerStep;

    /*! @brief   Target width in meters used  for micro-doppler accumulation range/azimuth area size */
    float targetSize;

    /*! @brief   Azimuth FFT size */
    uint16_t numAzimuthBins;

    /*! @brief   Number of range bins */
    uint16_t numRangeBins;

    /*! @brief   Number of range bins */
    uint16_t numDopplerBins;

    /*! @brief   Maximum number of azimuth bins in micro-doppler accumulation */
    uint16_t maxNumAzimAccumBins;

    /*! @brief  Output buffer (ping/pong) to store HWA micro Doppler data */
    uint32_t *uDopplerHwaOutput;

    /*! @brief  Output buffer index */
    uint32_t uDopplerHwaOutputPinPongInd;

    /*! @brief   Azimuth processing approach: 0-FFT,  1-Beam Forming, single beam*/
    uint8_t     azimuthProcessingApproach;

    /**
     * @brief Output option 0-magnitude, 1-magnitude squared
     */
    uint8_t magnitudeSquared;
    /**
     * @brief Output option 0-disabled, 1-enabled
     */
    uint8_t circShiftAroundCentroid;
    /**
     * @brief Normalized output between [0 1]
     */
    uint8_t normalizedSpectrum;

    /**
     * @brief Butterfly scaling in azimuth FFT
     */
    uint8_t butterflyScalingAzimuth;
    /**
     * @brief Butterfly scaling in Doppler FFT
     */
    uint8_t butterflyScalingDoppler;

    /**
     * @brief Twiddle increment half step size for calcualting twiddle increment in HWA to shift azimuth spectrum 
     */
    uint32_t twiddleStepHalf;
    /**
     * @brief Log2 of Twiddle increment half step size
     */
    uint32_t log2TwiddleStep;

    /*! @brief HWA Params save location
    */
    DPU_uDopProc_HwaParamSaveLoc hwaParamsSaveLoc;

    /*! @brief Micro-doppler feature extraction handle
    */
    FeatExtract_Handle featExtHandle;

    /*! @brief Collected features across frames per track ID
    */
    DPU_uDopProc_FeatureObj *featureObj;

    /*! @brief Linearized features set, inpuyt to classifier
    */
    float *featureLinBuf;

    uint8_t featureSelectionInd[sizeof(FeatExtract_featOutput)/sizeof(float)];

    int32_t frameCntrModClassifierNumFrames;

    classifier_Handle classifierHandle;

    /*! @brief  Maximum number of tracks by tracker
    */
    uint16_t maxNumTracks;

    /*! @brief  Classifier CLI configuration */
    DPU_uDopClassifierCliCfg microDopplerClassifierCliCfg;

}DPU_uDopProc_Obj;


#ifdef __cplusplus
}
#endif

#endif
