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
#ifndef UDOPPROC_HWA_H
#define UDOPPROC_HWA_H

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

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dp_error.h>
#include <datapath/dpu/udopproc/v0/udopproccommon.h>
#include <alg/featExtract/spectrumBased/featExtract.h>
#include <alg/classifier/targetClassifier/classifier.h>
#include <datapath/dpu/doaproc/v0/doaproc.h> //ToDo Remove this and create structure DPU_DoaProc_HWA_Option_Cfg in common file e.g. antenna_geometry.h


#ifdef __cplusplus
extern "C" {
#endif

#define DP_ERRNO_UDOPPLER_PROC_BASE                 (MMWAVE_ERRNO_DPU_BASE -700) //ToDo move this to dp_error.h

/** @addtogroup DPU_UDOPPROC_ERROR_CODE
 *  Base error code for the dopplerProc DPU is defined in the
 *  \include datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_UDOPPROC_EINVAL                  (DP_ERRNO_UDOPPLER_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_UDOPPROC_ENOMEM                  (DP_ERRNO_UDOPPLER_PROC_BASE-2)

/**
 * @brief   Error Code: DPU is in progress
 */
#define DPU_UDOPPROC_EINPROGRESS             (DP_ERRNO_UDOPPLER_PROC_BASE-3)

/**
 * @brief   Error Code: Out of HWA resources
 */
#define DPU_UDOPPROC_EHWARES                 (DP_ERRNO_UDOPPLER_PROC_BASE-4)

/**
 * @brief   Error Code: Semaphore creation failed
 */
#define DPU_UDOPPROC_ESEMA                   (DP_ERRNO_UDOPPLER_PROC_BASE-5)

/**
 * @brief   Error Code: Bad semaphore status 
 */
#define DPU_UDOPPROC_ESEMASTATUS             (DP_ERRNO_UDOPPLER_PROC_BASE-6)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size 
 */
#define DPU_UDOPPROC_EEXCEEDHWAMEM           (DP_ERRNO_UDOPPLER_PROC_BASE-7)

/**
 * @brief   Error Code: Unsupported radar cube format 
 */
#define DPU_UDOPPROC_ECUBEFORMAT             (DP_ERRNO_UDOPPLER_PROC_BASE-8)

/**
 * @brief   Error Code: Unsupported detection matrix format 
 */
#define DPU_UDOPPROC_EDETMFORMAT             (DP_ERRNO_UDOPPLER_PROC_BASE-9)

/**
 * @brief   Error Code: Insufficient detection matrix size
 */
#define DPU_UDOPPROC_EDETMSIZE               (DP_ERRNO_UDOPPLER_PROC_BASE-10)

/**
 * @brief   Error Code: Wrong window size
 */
#define DPU_UDOPPROC_EWINDSIZE               (DP_ERRNO_UDOPPLER_PROC_BASE-11)

/**
 * @brief   Error Code: Insufficient memory to save HWA param sets
 */
#define DPU_UDOPPROC_EHWA_PARAM_SAVE_LOC_SIZE (DP_ERRNO_UDOPPLER_PROC_BASE-12)

/**
 * @brief   Error Code: EDMA Configuration failed
 */
#define DPU_UDOPPROC_EEDMA_CONFIG_FAILED (DP_ERRNO_UDOPPLER_PROC_BASE-13)

/**
@}
*/

/**
 * @brief   Maximum number of HWA paramsets used by DPU.
 */
#define DPU_UDOPPROC_MAX_NUM_HWA_PARAMSET  6

/**
 * @brief   Number of HWA memory banks needed
 */
#define DPU_UDOPPROC_NUM_HWA_MEMBANKS  4 

/**
 * @brief   Disables first butterfly stage scaling
 */
#define DPU_UDOPPROC_FIRST_SCALING_DISABLED ((uint8_t)0U)

/**
 * @brief   Enables first butterfly stage scaling
 */
#define DPU_UDOPPROC_FIRST_SCALING_ENABLED ((uint8_t)1U)

/**
 * @brief   Range loop type controlled by HWA
 */
#define DPU_UDOPPROC_RANGE_LOOP_HWA_CONTROL (0)
/**
 * @brief   Range loop type controlled by CPU
 */
#define DPU_UDOPPROC_RANGE_LOOP_CPU_CONTROL (1)


/**
 * @brief   Maximum number of tracks
 */
#define  DPU_UDOPPROC_MAX_NUM_TRACKS         TRACKER_MAX_NUM_TR


#define DPU_UDOPPROC_AZIMUTH_FFT_APPROACH               0
#define DPU_UDOPPROC_AZIMUTH_BEAM_FORMING_APPROACH      1

/* Maximum number of azimuth accumulation bins */
#define DPU_UDOPPROC_MAX_NUM_AZIMUTH_ACCUM_BINS         15

/* Constant layer definitions */
#define CLASSIFIER_NUM_FRAMES           30
#define CLASSIFIER_NUM_FEATURES         4
#define CLASSIFIER_NUM_CLASSES          2

typedef struct FeatExtract_reducedFeatOutput_t
{
    float features[CLASSIFIER_NUM_FEATURES];
} FeatExtract_reducedFeatOutput;

typedef struct DPU_uDopProc_FeatureObj_t
{
    FeatExtract_reducedFeatOutput featureDelayLine[CLASSIFIER_NUM_FRAMES];
    float predictDelayLine[CLASSIFIER_NUM_FRAMES][CLASSIFIER_NUM_CLASSES];
    uint8_t presentDelayLine[CLASSIFIER_NUM_FRAMES];
    uint8_t missDelayLine[CLASSIFIER_NUM_FRAMES];
    uint8_t cumMissDelayLine[CLASSIFIER_NUM_FRAMES];
    uint8_t numPresent;
    uint8_t numMisses;
    uint8_t numCumMisses;
} DPU_uDopProc_FeatureObj;

/*!
 *  @brief   Handle for classifier
 */
typedef void*  classifier_Handle;

/**
* @brief
*  Source and destination addresses for EDMA input configuration. 
*
* @details
*  The structure is one of the input arguments of the DPU's proces function.
*
*  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
*/
typedef struct DPU_uDopProc_RadarCubeChunkEdmaCfg_t
{
    /*! @brief     Address in L3 of the chunk of data in radar cube matrix to be copied to HWA  */
    uint32_t srcAddress;

    /*! @brief     Destination address in HWA of the copied chunk */
    uint32_t dstAddress;

    /*! @brief     Packed EDMA values for A count (LSB 16-bit position) and B count (MSB 16-bit position */
    uint32_t Bcnt_Acnt;
} DPU_uDopProc_RadarCubeChunkEdmaCfg;

/*!
 *  @brief   Handle for Doppler Processing DPU.
 */
typedef void*  DPU_uDopProc_Handle;

/**
 * @brief
 *  dopplerProc DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations.
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_InitCfg_t
{
    HWA_Handle  hwaHandle;
    
}DPU_uDopProc_InitParams;

/**
 * @brief
 *  Structure for the HWA Params save location
 *
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_HwaParamSaveLoc_t
{
    /*! @brief  Pointer to save location for HWA PARAM sets */
    void        *data;

    /*! @brief  Size of the save location for HWA PARAM sets in Bytes*/
    uint32_t    sizeBytes;
} DPU_uDopProc_HwaParamSaveLoc;

/**
 * @brief
 *  Structure for
 *
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_scratch_t
{
    /*! @brief  Pointer to scratch */
    void        *data;

    /*! @brief  Size of scratch in Bytes */
    uint32_t    sizeBytes;
} DPU_uDopProc_scratch;

/**
 * @brief
 *  dopplerProc DPU HWA configuration parameters
 *
 * @details
 *  The structure is used to hold the HWA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_HwaCfg_t
{
    /*! @brief Indicates if HWA should enable butterfly scaling (divide by 2) of the 
         first radix-2 stage. Depending on the window definition, 
         user may want to skip the first stage scaling in order to avoid signal degradation.\n
         Options are:\n
         Disable first stage scaling: firstStageScaling = @ref DPU_UDOPPROC_FIRST_SCALING_DISABLED \n
         Enable first stage scaling: firstStageScaling = @ref DPU_UDOPPROC_FIRST_SCALING_ENABLED \n
         Note: All other butterfly stages have the scaling enabled. 
         This option applies only for the first stage.\n
    */
    uint8_t     firstStageScaling;

    /*! @brief Number of HWA paramsets reserved for the Doppler DPU. 
         The number of HWA paramsets required by this DPU is a function of the number of TX antennas 
         used in the configuration:\n 
         numParamSets = 2 x (Number of TX antennas) + 2\n
         The DPU will use numParamSets consecutively, starting from paramSetStartIdx.\n
    */     
    //uint8_t     numParamSets;
    
    /*! @brief HWA paramset Start index.  
         Application has to ensure that paramSetStartIdx is such that \n
        [paramSetStartIdx, paramSetStartIdx + 1, ... (paramSetStartIdx + numParamSets - 1)] \n
        is a valid set of HWA paramsets.\n
    */
    uint32_t    paramSetStartIdx;

    /*! @brief HWA Params save location
    */
    DPU_uDopProc_HwaParamSaveLoc hwaParamsSaveLoc;

    /*! @brief HWA param set dma trigger source channel
    */
    uint8_t     dmaTrigSrcChan;
}DPU_uDopProc_HwaCfg;

/*!
 *  @brief    Maximum peaks filled by HWA statistics block
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef volatile struct DPU_uDopProc_HwaMaxOutput_t
{
    uint32_t   maxInd;      /*!< @brief Maximum peak index position */
    uint32_t   peak;        /*!< @brief Maximum peak value */
}  DPU_uDopProc_HwaMaxOutput;

/*!
 *  @brief    Sum filled by HWA statistics block
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef volatile struct DPU_uDopProc_HwaSumOutput_t
{
    int32_t   sumIm;        /*!< @brief Sum imaginary */
    int32_t   sumRe;        /*!< @brief Sum real */
}  DPU_uDopProc_HwaSumOutput;

/**
 * @brief
 *  dopplerProc DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_EdmaCfg_t
{
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;
    uint32_t    edmaInstanceId;
    
    /*! @brief  EDMA configuration for Input data (Radar cube -> HWA memory). */
    DPEDMA_ChanCfg edmaIn;

    /*! @brief  EDMA configuration for the memory reset. */
    DPEDMA_ChanCfg edmaResetIn;

    /*! @brief  EDMA configuration for hot signature. */
    DPEDMA_ChanCfg edmaHotSig;
    
    /*! @brief  EDMA configuration for chaining back and final out */
    DPEDMA_3LinkChanCfg edmaChainOut;  //ToDo replace this with DPEDMA_2LinkChanCfg once defined in dpedma

    /*! @brief  EDMA configuration for data output from HWA - Detection matrix */
    DPEDMA_ChanCfg edmaMicroDopOut;

    /*! @brief  EDMA interrupt object. */
    Edma_IntrObject   *intrObj;

}DPU_uDopProc_EdmaCfg;

/**
 * @brief
 *  Doppler DPU HW configuration parameters
 *
 * @details
 *  The structure is used to hold the  HW configuration parameters
 *  for the Doppler DPU
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_HW_Resources_t
{
    /*! @brief  EDMA configuration */
    DPU_uDopProc_EdmaCfg edmaCfg;
    
    /*! @brief  HWA configuration */
    DPU_uDopProc_HwaCfg  hwaCfg;
    
    /*! @brief  Radar Cube */
    DPIF_RadarCube radarCube;

    /*! @brief  Detection matrix */
    DPIF_DetMatrix detMatrix;

    /*! @brief  Output buffer (ping/pong) to store HWA micro Doppler data */
    uint32_t *uDopplerHwaOutput;

    /*! @brief  Scratch buffer for feature extraction and classifier */
    DPU_uDopProc_scratch scratchBuf;

    /*! @brief HWA Doppler FFT Parmas set configuration (performing Doppler FFT and antenna mapping)
    */
    DPU_DoaProc_HWA_Option_Cfg doaRngGateCfg;

    /*! @brief Collected features across frames per track ID
    */
    DPU_uDopProc_FeatureObj *featureObj;

    /*! @brief  Scratch buffer for classifier, for linearized input feature set */
    DPU_uDopProc_scratch scratchBuf2;

}DPU_uDopProc_HW_Resources;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 *
 *  \ingroup    DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_uDopProc_compRxChannelBiasCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q20 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    cmplx32ReIm_t rxChPhaseComp[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPU_uDopProc_compRxChannelBiasCfg;

/**
 * @brief
 *  Micro Doppler Processing CLI Configuration
 *
 * @details
 */
typedef struct DPU_uDopProcCliCfg_t
{
    /**
     * @brief Target size in meters
     */
    float targetSize;

    /**
     * @brief  Lower frequency power interception (in percentage)
     */
    float interceptThrLowFreq;

    /**
     * @brief  Upper frequency power interception (in percentage)
     */
    float interceptThrUpFreq;

    /**
     * @brief  Additional micro doppler spectrum shift by the feature extractor, based on the mean/median estimate,
     *         options are: 0 (no shift), 1 (shift around mean), 2 (shift around median)
     */
    uint8_t specShiftMode;

    /**
     * @brief Enabled Micro Doppler processing
     */
    uint8_t enabled;

    /**
     * @brief Generation approach: 0-Azimuth FFT + mean around fixed target size, 1-Azimuth beamforming+centroid only
     */
    uint8_t genApproach;

    /**
     * @brief Output option 0-magnitude, 1-magnitude squared
     */
    uint8_t magnitudeSquared;

    /**
     * @brief micro doppler FFT output shift option: 0-circulate FFT spectrum around 0 Hz (equivalent to matlab fftshift),
     *                                               1-circulate FFT spectrum around target's radial doppler centroid
     */
    uint8_t circShiftAroundCentroid;

    /**
     * @brief Normalized output between [0 1]
     */
    uint8_t normalizedSpectrum;

} DPU_uDopProcCliCfg;

/**
 * @brief
 *  Micro Doppler Classifier CLI Configuration
 *
 * @details
 */
typedef struct DPU_uDopClassifierCliCfg_t
{
    /**
     * @brief Enabled Classifier processing on target
     */
    uint8_t enabled;

    /**
     * @brief Maximum number of allowed missed frames in the observation window specified for classification
     */
    uint8_t missTotFrmThre;

    /**
     * @brief Minimum number of associated point-cloud points with the track ID for classifier to be called
     */
    uint8_t minNumPntsPerTrack;


} DPU_uDopClassifierCliCfg;

/**
 * @brief
 *  Doppler DPU static configuration parameters
 *
 * @details
 *  The structure is used to hold the static configuration parameters
 *  for the Doppler DPU. The following conditions must be satisfied:
 *
 *    @verbatim
      numTxAntennas * numRxAntennas * numDopplerChirps * sizeof(cmplx16ImRe_t) <= 16 KB (one HWA memory bank)
      numTxAntennas * numRxAntennas * numDopplerBins * sizeof(uint16_t) <= 16 KB (one HWA memory bank)
      @endverbatim
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;
    
    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;
    
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
    
    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;
    
    /*! @brief  Number of Doppler chirps. */
    uint16_t    numDopplerChirps;
    
    /*! @brief  Number of Doppler bins */
    uint16_t    numDopplerBins;
    
    /*! @brief  Azimuth FFT size */
    uint16_t    azimuthFftSize;

    /*! @brief  elevation FFT size */
    uint16_t    elevationFftSize;

    /*! @brief  Number of chirps per frame for Minor Motion Detection */
    uint16_t    numMinorMotionChirpsPerFrame;

    /*! @brief  Log2 of number of Doppler bins */
    uint8_t     log2NumDopplerBins;
    
    /*! @brief     1 - 1D: 6x1 azimuth antenna array 2 - 2D: one lower row: 4 antennas, upper row 2 antennas */
    uint8_t     angleDimension;

    /*! @brief     Detection matrix format 0 - Linear 32-bit integer, 1 - 16-bit log2 magnitude in Q11 format */
    bool        isDetMatrixLogScale;

    /*! @brief   Static Clutter Removal Cfg */
    bool        isStaticClutterRemovalEnabled;

    /*! @brief   Range Bias and rx channel gain/phase compensation enable flag */
    bool        isRxChGainPhaseCompensationEnabled;

    /*! @brief   Range Bias and rx channel gain/phase compensation configuration */
    DPU_uDopProc_compRxChannelBiasCfg compRxChanCfg;

    /*! @brief  Range conversion factor for FFT range index to meters */
    float       rangeStep;

    /*! @brief  Doppler conversion factor for Doppler FFT index to m/s */
    float       dopplerStep;

    /*! @brief   Maximum number of azimuth bins in micro-doppler accumulation */
    uint8_t maxNumAzimAccumBins;

    /*! @brief   CLI Configuration */
    DPU_uDopProcCliCfg cliCfg;

    /*! @brief  Number of virtual antenna rows */
    uint16_t    numAntRow;

    /*! @brief  Number of virtual antenna columns */
    uint16_t    numAntCol;

    /*! @brief  Maximum number of tracks by tracker */
    uint16_t maxNumTracks;

    /*! @brief  Classifier CLI configuration */
    DPU_uDopClassifierCliCfg microDopplerClassifierCliCfg;

}DPU_uDopProc_StaticConfig;

/**
 * @brief
 *  dopplerProc DPU configuration parameters
 *
 * @details
 *  The structure is used to hold the configuration parameters
 *  for the Doppler Processing removal DPU
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_Config_t
{
    /*! @brief HW resources. */
    DPU_uDopProc_HW_Resources  hwRes;
    
    /*! @brief Static configuration. */
    DPU_uDopProc_StaticConfig  staticCfg;
    
    /*! @brief Flag indicating first time configuration call */
    bool isFirstTimeCfg;

}DPU_uDopProc_Config;


/**
 * @brief
 * Structure holds the target coordinates
 *
 */
typedef struct DPU_uDopProc_tList_t
{
    /*! @brief   tracking ID */
    uint32_t     tid;
    /*! @brief   Detected target X coordinate, in m */
    float        posX;
    /*! @brief   Detected target Y coordinate, in m */
    float        posY;
     /*! @brief   Detected target Z coordinate, in m */
    float        posZ;
    /*! @brief   velocity, in m/s */
    float        velX;
    /*! @brief   velocity, in m/s */
    float        velY;
    /*! @brief   velocity, in m/s */
    float        velZ;
} DPU_uDopProc_tList;

typedef struct DPU_uDopProc_TrackerData_t
{
    /*! @brief   Number of tracks */
    uint32_t numTargets;

    /*! @brief   Number of detected points in point cloud */
    uint32_t numIndices;

    /*! @brief   Number of major motion detected points in point cloud */
    uint32_t numIndicesMajorMotion;

    /*! @brief   Number of minor motion detected points in point cloud */
    uint32_t numIndicesMinorMotion;

    /*! @brief   tracking list */
    DPU_uDopProc_tList tList[DPU_UDOPPROC_MAX_NUM_TRACKS];

    /*! @brief   index list */
    uint8_t *tIndex;

} DPU_uDopProc_TrackerData;

typedef struct DPU_uDopProc_classifierPrediction_t
{
    uint8_t prediction[CLASSIFIER_NUM_CLASSES];
} DPU_uDopProc_classifierPrediction;


typedef struct DPU_uDopProc_classifierOutput_t
{
    DPU_uDopProc_classifierPrediction predictions[DPU_UDOPPROC_MAX_NUM_TRACKS];
} DPU_uDopProc_classifierOutput;

/**
 * @brief
 *  DPU processing output parameters
 *
 * @details
 *  The structure is used to hold the output parameters DPU processing
 *
 *  \ingroup DPU_UDOPPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_uDopProc_OutParams_t
{
    /*! @brief DPU statistics */
    DPU_uDopProc_Stats  stats;

    /*! @brief micro doppler arrays */
    float       *uDopplerOutput;

    /*! @brief Number of micro doppler buffers */
    uint32_t     numDopplerBins;

    /*! @brief Number of targets, i.e. number of micro doppler buffers */
    uint32_t     numTargets;

    /*! @brief micro doppler feature array */
   FeatExtract_featOutput *uDopplerFeatures;

   /*! @brief Number of classified Targets */
   uint32_t     numClassifiedTargets;

   DPU_uDopProc_classifierOutput classifierOutput;
}DPU_uDopProc_OutParams;


DPU_uDopProc_Handle DPU_uDopProc_init(DPU_uDopProc_InitParams *initCfg, int32_t* errCode);
int32_t DPU_uDopProc_process(DPU_uDopProc_Handle handle,
                            DPIF_RadarCube *radarCube,
                            DPIF_DetMatrix *detMatrix,
                            DPU_uDopProc_TrackerData *trackerData,
                            DPU_uDopProc_OutParams *outParams);
int32_t DPU_uDopProc_deinit(DPU_uDopProc_Handle handle);
int32_t DPU_uDopProc_config(DPU_uDopProc_Handle handle, DPU_uDopProc_Config *cfg);
int32_t DPU_uDopProc_GetNumUsedHwaParamSets(DPU_uDopProc_Handle handle, uint8_t *numUsedHwaParamSets);



#ifdef __cplusplus
}
#endif

#endif
