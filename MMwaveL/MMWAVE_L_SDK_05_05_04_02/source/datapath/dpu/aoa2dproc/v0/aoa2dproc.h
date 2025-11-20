/**
 *   @file  aoa2dproc.h
 *
 *   @brief
 *      Implements Data path AoA2D processing functionality.
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
#ifndef AOA2DPROC_HWA_H
#define AOA2DPROC_HWA_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */
#include <drivers/hwa.h>
#include <common/syscommon.h>

/* DPIF Components Include Files */
#include <datapath/dpif/dpif_detmatrix.h>
#include <datapath/dpif/dpif_radarcube.h>
#include <datapath/dpif/dpif_pointcloud.h>

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dp_error.h>
#include <datapath/dpu/aoa2dproc/v0/aoa2dproccommon.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_AOA2DPROC_ERROR_CODE
 *  Base error code for the aoa2dProc DPU is defined in the 
 *  \include datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_AOA2DPROC_EINVAL                  (DP_ERRNO_AOA2D_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_AOA2DPROC_ENOMEM                  (DP_ERRNO_AOA2D_PROC_BASE-2)

/**
 * @brief   Error Code: DPU is in progress
 */
#define DPU_AOA2DPROC_EINPROGRESS             (DP_ERRNO_AOA2D_PROC_BASE-3)

/**
 * @brief   Error Code: Out of HWA resources
 */
#define DPU_AOA2DPROC_EHWARES                 (DP_ERRNO_AOA2D_PROC_BASE-4)

/**
 * @brief   Error Code: Semaphore creation failed
 */
#define DPU_AOA2DPROC_ESEMA                   (DP_ERRNO_AOA2D_PROC_BASE-5)

/**
 * @brief   Error Code: Bad semaphore status 
 */
#define DPU_AOA2DPROC_ESEMASTATUS             (DP_ERRNO_AOA2D_PROC_BASE-6)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size 
 */
#define DPU_AOA2DPROC_EEXCEEDHWAMEM           (DP_ERRNO_AOA2D_PROC_BASE-7)

/**
 * @brief   Error Code: Unsupported radar cube format 
 */
#define DPU_AOA2DPROC_ECUBEFORMAT             (DP_ERRNO_AOA2D_PROC_BASE-8)

/**
 * @brief   Error Code: Unsupported detection matrix format 
 */
#define DPU_AOA2DPROC_EDETMFORMAT             (DP_ERRNO_AOA2D_PROC_BASE-9)

/**
 * @brief   Error Code: Insufficient detection matrix size
 */
#define DPU_AOA2DPROC_EDETMSIZE               (DP_ERRNO_AOA2D_PROC_BASE-10)

/**
 * @brief   Error Code: Wrong window size
 */
#define DPU_AOA2DPROC_EWINDSIZE               (DP_ERRNO_AOA2D_PROC_BASE-11)

/**
 * @brief   Error Code: Not supported angle dim 1 (no elevation) and
 */
#define DPU_AOA2DPROC_E_NOTSUPPORTED_ANGLE_DIM_ONE_AND_CPU_LOOP_CTRL (DP_ERRNO_AOA2D_PROC_BASE-12)

/**
 * @brief   Error Code: Doppler FFT size exceeded the limit. Maximum Doppler index is stored in array of type uint8_t.
 */
#define DPU_AOA2DPROC_E_EXCEEDED_MAX_NUM_DOPPLER_BINS (DP_ERRNO_AOA2D_PROC_BASE-13)

/**
@}
*/

/**
 * @brief   Number of registers per HWA param set
 */
#define DPU_AOA2DPROC_HWA_NUM_REGS_PER_PARAM_SET 8

/**
 * @brief   Maximum number of HWA paramsets used by DPU.
 */
#define DPU_AOA2DPROC_MAX_NUM_HWA_PARAMSET  6

/**
 * @brief   Number of HWA memory banks needed
 */
#define DPU_AOA2DPROC_NUM_HWA_MEMBANKS  4 

/**
 * @brief   Disables first butterfly stage scaling
 */
#define DPU_AOA2DPROC_FIRST_SCALING_DISABLED ((uint8_t)0U)

/**
 * @brief   Enables first butterfly stage scaling
 */
#define DPU_AOA2DPROC_FIRST_SCALING_ENABLED ((uint8_t)1U)

/**
 * @brief   scale for side-lobe threshold
 */
#define DPU_AOA2DPROC_SHIFT_Q8  8
#define DPU_AOA2DPROC_ONE_Q8    (1<<DPU_CFARPROCHWA_SHIFT_Q8)

/*!
 *  @brief   Handle for Aoa2d Processing DPU.
 */
typedef void*  DPU_Aoa2dProc_Handle;

/**
 * @brief
 *  dopplerProc DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations.
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_InitCfg_t
{
    HWA_Handle  hwaHandle;
    
}DPU_Aoa2dProc_InitParams;

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
typedef struct DPU_Aoa2dProc_HWA_Doppler_Fft_Cfg_t
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
} DPU_Aoa2dProc_HWA_Doppler_Fft_Cfg;

/**
 * @brief    Maximum nuber of Doppler FFT params
 *
 */
#define DPU_AOA2D_PROC_MAX_NUM_DOP_FFFT_PARAMS 5

/**
 * @brief    Maximum number of antenna elements in 2D virtual antenna array (maximum value of the product numRows*numCol)
 *
 */
#define DPU_AOA2D_PROC_MAX_2D_ANT_ARRAY_ELEMENTS 16

/**
 * @brief
 *
 */
typedef struct DPU_Aoa2dProc_HWA_Option_Cfg_t
{
    uint8_t numDopFftParams;
    DPU_Aoa2dProc_HWA_Doppler_Fft_Cfg dopFftCfg[DPU_AOA2D_PROC_MAX_NUM_DOP_FFFT_PARAMS];
} DPU_Aoa2dProc_HWA_Option_Cfg;


/**
 * @brief
 *  dopplerProc DPU HWA configuration parameters
 *
 * @details
 *  The structure is used to hold the HWA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_FFTWindowCfg_t
{
    /*! @brief Indicates if HWA window is symmetric or non-symmetric.
        Use HWA macro definitions for symmetric/non-symmetric.
    */
    uint8_t     winSym;

    /*!  @brief Doppler FFT window size in bytes. 
         This is the number of coefficients to be programmed in the HWA for the windowing
         functionality. The size is a function of numDopplerChirps as follows:\n
         If non-symmetric window is selected: windowSize = numDopplerChirps * sizeof(int32_t) \n
         If symmetric window is selected and numDopplerChirps is even:
         windowSize = numDopplerChirps * sizeof(int32_t) / 2 \n
         If symmetric window is selected and numDopplerChirps is odd:
         windowSize = (numDopplerChirps + 1) * sizeof(int32_t) / 2        
    */
    uint32_t    windowSize;

    /*! @brief Pointer to Doppler FFT window coefficients. */
    int32_t     *window;

    /*! @brief HWA window RAM offset in number of samples. */
    uint32_t    winRamOffset;
    
} DPU_Aoa2dProc_FFTWindowCfg;


/**
 * @brief
 *  dopplerProc DPU HWA configuration parameters
 *
 * @details
 *  The structure is used to hold the HWA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_HwaCfg_t
{
    DPU_Aoa2dProc_FFTWindowCfg dopplerWindow;
    DPU_Aoa2dProc_FFTWindowCfg angleWindow;

    /*! @brief Indicates if HWA should enable butterfly scaling (divide by 2) of the
         first radix-2 stage. Depending on the window definition,
         user may want to skip the first stage scaling in order to avoid signal degradation.\n
         Options are:\n
         Disable first stage scaling: firstStageScaling = @ref DPU_AOA2DPROC_FIRST_SCALING_DISABLED \n
         Enable first stage scaling: firstStageScaling = @ref DPU_AOA2DPROC_FIRST_SCALING_ENABLED \n
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
    uint8_t     numParamSets;
    
    /*! @brief HWA paramset Start index.  
         Application has to ensure that paramSetStartIdx is such that \n
        [paramSetStartIdx, paramSetStartIdx + 1, ... (paramSetStartIdx + numParamSets - 1)] \n
        is a valid set of HWA paramsets.\n
    */
    uint32_t    paramSetStartIdx;

    /*! @brief HWA input memory bank address
    */
    uint32_t    hwaMemInpAddr;

    /*! @brief HWA output memory bank address
    */
    uint32_t    hwaMemOutAddr;

    /*! @brief HWA Doppler FFT Parmas set configuration (performing Doppler FFT and antenna mapping)
    */
    DPU_Aoa2dProc_HWA_Option_Cfg aoa2dRngGateCfg;

    /*! @brief HWA param set dma trigger source channel
    */
    uint8_t     dmaTrigSrcChan;
}DPU_Aoa2dProc_HwaCfg;

/*!
 *  @brief    Maximum peaks filled by HWA statistics block
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef volatile struct DPU_Aoa2dProc_HwaMaxOutput_t
{
    uint32_t   maxInd;      /*!< @brief Maximum peak index position */
    uint32_t   peak;        /*!< @brief Maximum peak value */
}  DPU_Aoa2dProc_HwaMaxOutput;


/**
 * @brief
 *  dopplerProc DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_EdmaCfg_t
{
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;

    /*! @brief  EDMA Instance id */
    uint32_t    edmaInstanceId;
    
    /*! @brief  EDMA configuration for Input data (Radar cube -> HWA memory). */
    DPEDMA_ChanCfg edmaIn;

    /*! @brief  EDMA configuration for hot signature. */
    DPEDMA_ChanCfg edmaHotSig;
    
    /*! @brief  EDMA interrupt object. */
    Edma_IntrObject   *intrObj;

}DPU_Aoa2dProc_EdmaCfg;

/**
 * @brief
 *  Aoa2d DPU HW configuration parameters
 *
 * @details
 *  The structure is used to hold the  HW configuration parameters
 *  for the Doppler DPU
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_HW_Resources_t
{
    /*! @brief  EDMA configuration */
    DPU_Aoa2dProc_EdmaCfg edmaCfg;
    
    /*! @brief  HWA configuration */
    DPU_Aoa2dProc_HwaCfg  hwaCfg;
    
    /*! @brief  Radar Cube */
    DPIF_RadarCube radarCube;
    
    /*! @brief  Detection matrix */
    DPIF_DetMatrix detMatrix;

}DPU_Aoa2dProc_HW_Resources;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 *
 *  \ingroup    DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_Aoa2dProc_compRxChannelBiasCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q20 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    cmplx32ReIm_t rxChPhaseComp[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPU_Aoa2dProc_compRxChannelBiasCfg;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 *
 *  \ingroup    DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_Aoa2dProc_compRxChannelBiasFloatCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q20 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    float rxChPhaseComp[2 * SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPU_Aoa2dProc_compRxChannelBiasFloatCfg;

/**
 * @brief
 *  Field of view - AoA2d Configuration
 *
 * @details
 *  The structure contains the field of view - DoA configuration
 *
 */
typedef struct DPU_Aoa2dProc_AoaFovCfg_t
{
    /*! @brief    minimum azimuth angle (in degrees) exported to Host*/
    float        minAzimuthDeg;

    /*! @brief    maximum azimuth angle (in degrees) exported to Host*/
    float        maxAzimuthDeg;

    /*! @brief    minimum elevation angle (in degrees) exported to Host*/
    float        minElevationDeg;

    /*! @brief    maximum elevation angle (in degrees) exported to Host*/
    float        maxElevationDeg;
} DPU_Aoa2dProc_AoaFovCfg;

/*!
 *  @brief    Holds number of samples to be skipped in detection process from
 *            left and right side of the dimension for range, azimuth and
 *            elevation dimension
 *
 */
typedef struct DPU_Aoa2dProc_detectionCfg_t
{
    /*! @brief    number of samples to be skipped from left in range dimension */
    int16_t skipLeftRange;
    /*! @brief    number of samples to be skipped from right in range dimension */
    int16_t skipRightRange;
    /*! @brief    number of samples to be skipped from left in azimuth dimension */
    int16_t skipLeftAzim;
    /*! @brief    number of samples to be skipped from right in azimuth dimension */
    int16_t skipRightAzim;
    /*! @brief    number of samples to be skipped from left in elevation dimension */
    int16_t skipLeftElev;
    /*! @brief    number of samples to be skipped from right in elevation dimension */
    int16_t skipRightElev;
} DPU_Aoa2dProc_detectionCfg;

/**
 * @brief
 *  Compression parameters
 *
 * @details
 *  The structure is used to hold compression parameters.
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_Aoa2dProc_compressCfg_t
{

    uint8_t     compressionFactor;

    /* number of complex samples to compress */
    uint32_t numComplexElements;

} DPU_Aoa2dProc_compressCfg;

/**
 * @brief
 *  Doppler DPU static configuration parameters
 *
 * @details
 *  The structure is used to hold the static configuration parameters
 *  for the AOA2D DPU.
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_StaticConfig_t
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

    /*! @brief  Number of real chirps per frame */
    uint16_t    numBpmDopplerChirps;

    /*! @brief  BPM Doppler FFT size - same as number of real chirps per frame */
    uint16_t    bpmDopplerFftSize;

    /*! @brief  Azimuth FFT size */
    uint16_t    azimuthFftSize;

    /*! @brief  elevation FFT size */
    uint16_t    elevationFftSize;

    /*! @brief  Lambda over antenna spacing in x-direction */
    float       lambdaOverDistX;

    /*! @brief  Lambda over antenna spacing in z-direction */
    float       lambdaOverDistZ;

    /*! @brief  Number of chirps per frame for Minor Motion Detection */
    uint16_t    numMinorMotionChirpsPerFrame;

    /*! @brief  Number of frames Minor Motion Detection spreads over */
    uint16_t    numFrmPerMinorMotProc;

    /*! @brief  Log2 of number of Doppler bins */
    uint8_t     log2NumDopplerBins;
    
    /*! @brief  log2 of BPM Doppler FFT size */
    uint16_t    log2BpmDopplerFftSize;

    /*! @brief     1 - 1D: 6x1 azimuth antenna array 2 - 2D: one lower row: 4 antennas, upper row 2 antennas */
    uint8_t     angleDimension;

    /*! @brief     Detection matrix format 0 - Linear 32-bit integer, 1 - 16-bit log2 magnitude in Q11 format */
    bool        isDetMatrixLogScale;

    /*! @brief   Static Clutter Removal Cfg */
    bool        isStaticClutterRemovalEnabled;

    /*! @brief   Range Bias and rx channel gain/phase compensation configuration */
    DPU_Aoa2dProc_compRxChannelBiasCfg compRxChanCfg;

    /*! @brief  Number of virtual antenna rows */
    uint16_t    numAntRow;

    /*! @brief  Number of virtual antenna columns */
    uint16_t    numAntCol;

    /*! @brief  field of view cfg */
    DPU_Aoa2dProc_AoaFovCfg *fovAoaCfg;

    /*! @brief  Side lobe threshold linear scale in Q8 format */
    int16_t     sideLobeThresholdScaleQ8;

    /*! @brief  Interpolation in azimuth direction */
    bool        enableInterpAzimuthDom;

    /*! @brief  compression enable/disable flag for radar cube data */
    bool isCompressionEnabled;

    /*! @brief  compression parameters for radar cube data */
    DPU_Aoa2dProc_compressCfg compressCfg;
    
}DPU_Aoa2dProc_StaticConfig;

/**
 * @brief
 *  aoa2dProc DPU configuration parameters
 *
 * @details
 *  The structure is used to hold the configuration parameters
 *  for the AOA2D DPU
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_Config_t
{
    /*! @brief HW resources. */
    DPU_Aoa2dProc_HW_Resources  hwRes;
    
    /*! @brief Static configuration. */
    DPU_Aoa2dProc_StaticConfig  staticCfg;
    
}DPU_Aoa2dProc_Config;


/**
 * @brief
 *  DPU processing output parameters
 *
 * @details
 *  The structure is used to hold the output parameters DPU processing
 *
 *  \ingroup DPU_AOA2DPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_OutParams_t
{
    /*! @brief      Number of detected points */
    uint32_t numDetectedPoints;

    /*! @brief      point-cloud output list */
    DPIF_PointCloudCartesianExt     *detObjOut;

    /*! @brief      point-cloud output list size */
    uint32_t detObjOutMaxSize;

    /*! @brief DPU statistics */
    DPU_Aoa2dProc_Stats  stats;

}DPU_Aoa2dProc_OutParams;


DPU_Aoa2dProc_Handle DPU_Aoa2dProc_init(DPU_Aoa2dProc_InitParams *initCfg, int32_t* errCode);
int32_t DPU_Aoa2dProc_process(DPU_Aoa2dProc_Handle    handle,
                              DPIF_RadarCube *radarCubeSrc,
                              DPIF_CFARRngDopDetListElement *detRngDopList,
                                uint32_t numDetPoints,
                                DPIF_DetectedRangeGates *detectedRangeGates, //ToDo this should be scratch memory
                                DPIF_DetMatrix *detMatrix,
                                DPU_Aoa2dProc_OutParams *outParams);

int32_t DPU_Aoa2dProc_deinit(DPU_Aoa2dProc_Handle handle);
int32_t DPU_Aoa2dProc_config(DPU_Aoa2dProc_Handle handle, DPU_Aoa2dProc_Config *cfg);
int32_t DPU_Aoa2dProc_GetNumUsedHwaParamSets( DPU_Aoa2dProc_Handle handle, uint8_t *numUsedHwaParamSets);

#if 0
int32_t DPU_Aoa2dProcHWA_control
(
   DPU_AoAProcHWA_Handle handle,
   DPU_AoAProcHWA_Cmd cmd,
   void *arg,
   uint32_t argSize
);
#endif


#ifdef __cplusplus
}
#endif

#endif
