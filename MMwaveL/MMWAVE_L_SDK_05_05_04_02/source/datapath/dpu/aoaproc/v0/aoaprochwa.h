/*
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

/**
 *   @file  aoaprochwa.h
 *
 *   @brief
 *      Implements Datapath Angle of Arrival processing functionality using the HWA.
 *      Based on the RF parameters, rangeFFT output and detected points
 *      aoaProc configures hardware accelerator FFT engine in FFT mode, does recalculation
 *      for detected points doppler FFT, doppler phase compensation, rearranges the data
 *      in memory for elevation-azimuth FFT and finally outputs the elevation and azimuth indices.

 *      It also configures data input and output EDMA channels to bring data in and out of 
 *      AoA Processing memory during Doppler FFT recalculation and AoA calculation.
 *
 *      The process is triggered in the inter-frame processing time after 2D-FFT and detected points.
 *
 *      After HWA processing is done, it generates interrupt to AoA DPU, at the same time triggers 
 *      EDMA data output channel to copy out elevation-azimuth indices. EDMA interrupt done interrupt is 
 *      triggered by EDMA hardware after the copy is completed.
 *
 */
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef AOAPROCHWA_H_
#define AOAPROCHWA_H_

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
#include <datapath/dpif/dpif_anglemat.h>
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <datapath/dpu/aoaproc/v0/aoaproc_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_AOAPROC_ERROR_CODE
 *  Base error code for the dopplerProc DPU is defined in the
 *  \include datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_AOAPROCHWA_EINVAL                  (DP_ERRNO_AOA_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_AOAPROCHWA_ENOMEM                  (DP_ERRNO_AOA_PROC_BASE-2)

/**
 * @brief   Error Code: DPU is in progress
 */
#define DPU_AOAPROCHWA_EINPROGRESS             (DP_ERRNO_AOA_PROC_BASE-3)

/**
 * @brief   Error Code: Out of HWA resources
 */
#define DPU_AOAPROCHWA_EHWARES                 (DP_ERRNO_AOA_PROC_BASE-4)

/**
 * @brief   Error Code: Semaphore creation failed
 */
#define DPU_AOAPROCHWA_ESEMA                   (DP_ERRNO_AOA_PROC_BASE-5)

/**
 * @brief   Error Code: Bad semaphore status 
 */
#define DPU_AOAPROCHWA_ESEMASTATUS             (DP_ERRNO_AOA_PROC_BASE-6)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size 
 */
#define DPU_AOAPROCHWA_EEXCEEDHWAMEM           (DP_ERRNO_AOA_PROC_BASE-7)

/**
 * @brief   Error Code: Unsupported radar cube format 
 */
#define DPU_AOAPROCHWA_ECUBEFORMAT             (DP_ERRNO_AOA_PROC_BASE-8)

/**
 * @brief   Error Code: Unsupported detection matrix format 
 */
#define DPU_AOAPROCHWA_EDETMFORMAT             (DP_ERRNO_AOA_PROC_BASE-9)

/**
 * @brief   Error Code: Insufficient detection matrix size
 */
#define DPU_AOAPROCHWA_EDETMSIZE               (DP_ERRNO_AOA_PROC_BASE-10)

/**
 * @brief   Error Code: Wrong window size
 */
#define DPU_AOAPROCHWA_EWINDSIZE               (DP_ERRNO_AOA_PROC_BASE-11)
/**
@}
*/

/**
 * @brief   Maximum number of HWA paramsets used by DPU.
 *          Computed as follows:\n
 * (1 for Doppler-FFT)*2(ping/pong) + (2 for Doppler-Compensation for Tx1 and Tx2)*2(ping/pong) + (3- for azimuth, elevation FFT, finding the maxima)*2(ping/pong) = 12
 */
#define DPU_AOAPROCHWA_MAX_NUM_HWA_PARAMSET 12

/**
 * @brief   Number of HWA paramsets used by DPU for 2D FFT + Doppler compensation
 *          Computed as follows:\n
 * (1 for Doppler FFT and 2 for Doppler Compensation - for 1st Tx and 2nd Tx)*2(ping/pong) = 6
 */
#define DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT 6

/**
 * @brief   Number of HWA paramsets used by DPU for 2D FFT if numTx antennas = 1 (Doppler compensation is not required)
 *          Computed as follows:\n
 * (1 for Doppler FFT)*2(ping/pong) = 2
 */
#define DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT_ONLY 2

/**
 * @brief   Number of HWA paramsets used by DPU for AoA
 *          Computed as follows:\n
 * (1 paramset for azimuth FFT + 1 paramset for elevation FFT + 1-paramset to find the maxima)*2(ping/pong) = 6
 */
#define DPU_AOAPROCHWA_NUM_HWA_PARAMSET_AOA 6

/**
 * @brief   Number of HWA memory banks needed
 */
#define DPU_AOAPROCHWA_NUM_HWA_MEMBANKS  4

/*!
 *  @brief   Handle for AoA Processing DPU.
 */
typedef void*  DPU_AoAProcHWA_Handle;

/**
 * @brief
 *  AoAProc DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations.
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_InitCfg_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
    
}DPU_AoAProcHWA_InitParams;

/**
 * @brief
 *  AoAProc DPU HWA configuration parameters
 *
 * @details
 *  The structure is used to hold the HWA configuration parameters
 *  for the AoA Processing DPU
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_HwaCfg_t
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

    /*!  @brief Vector Multiplication RAM size in bytes. 
        This is the number of coefficients to be programmed in the HWA for the Vector Multiplication
        functionality. 
    */
    uint32_t    vectorRamSize;

    /*! @brief Pointer to Vector Multiplication RAM coefficients. */
    cmplx32ImRe_t     *vectorRam;

    /*! @brief HWA vector multiplication RAM offset in number of samples. */
    uint32_t    vecRamOffset;
    
    /*! @brief HWA paramset Start index.  
         Application has to ensure that paramSetStartIdx is such that \n
        [paramSetStartIdx, paramSetStartIdx + 1, ... (paramSetStartIdx + numParamSets - 1)] \n
        is a valid set of HWA paramsets.\n
    */
    uint32_t    paramSetStartIdx;
    
    /*! @brief HWA param set dma trigger source channel */
    uint8_t     hwaDmaTriggerDoppPing;

    /*! @brief HWA param set dma trigger source channel */
    uint8_t     hwaDmaTriggerDoppPong;

    /*! @brief HWA param set dma trigger source channel */
    uint8_t     hwaDmaTriggerAoAPing;

    /*! @brief HWA param set dma trigger source channel */
    uint8_t     hwaDmaTriggerAoAPong;

    /*! @brief HWA NumLoops for AoA calculation */
    uint16_t     *AoAhwaNumLoops;

    /*! @brief Flag to check Doppler compensations is enabled or not */
    uint8_t      *isDopplerPhaseCompensationEnabled;

}DPU_AoAProcHWA_HwaCfg;

/**
 * @brief
 *  AoAProc DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the AoA Processing DPU
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_EdmaCfg_t
{
    /*! @brief  EDMA driver handle of TPCC B*/
    EDMA_Handle edmaHandle;
    
    /*! @brief  EDMA configuration for Input data (Radar FFT cube -> HWA memory). */
    DPU_AoAProc_Edma edmaIn_rangeFFT;

    /*! @brief  EDMA configuration for Input data (Radar Angle Matrix -> HWA memory). */
    DPU_AoAProc_Edma edmaIn_dopplerFFT;

    /*! @brief  EDMA configuration for hot signature before Doppler FFT. */
    DPU_AoAProc_Edma edmaHotSigDoppler;

    /*! @brief  EDMA configuration for hot signature before AoA FFT. */
    DPU_AoAProc_Edma edmaHotSigAoA;

    /*! @brief  EDMA configuration for hot signature before AoA FFT. */
    DPU_AoAProc_Edma edmaOut_maxAoA;
    
}DPU_AoAProcHWA_EdmaCfg;

/**
 * @brief
 *  AoA DPU HW configuration parameters
 *
 * @details
 *  The structure is used to hold the HW configuration parameters
 *  for the AoA DPU
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_HW_Resources_t
{
    /*! @brief  EDMA configuration */
    DPU_AoAProcHWA_EdmaCfg edmaCfg;
    
    /*! @brief  HWA configuration */
    DPU_AoAProcHWA_HwaCfg  hwaCfg;
    
    /*! @brief  Radar Cube */
    DPIF_RadarCube radar_1D_FFT_Cube;

    /*! @brief  Angle Matrix on which Elevation-Azimuth FFT is to be applied*/
    DPIF_Anglemat angleMat;

    /*! @brief  Matrix where the azimuth-elevation FFT max values is stored (this matrix should be in M4 for ease of computation*/
    uint32_t *maxVal_elev_azim;

    /*! @brief     EDMA interrupt object */
    /* NOTE: Application needs to provide address of the EDMA interrupt object.
     * This needs to be done as there might be multiple subframes configured
     * and each subframe needs EDMA interrupt to be registered.
     */
    Edma_IntrObject *intrObj;
}DPU_AoAProcHWA_HW_Resources;

/**
 * @brief
 *  AoA DPU HW configuration parameters
 *
 * @details
 *  The structure is used to hold the HW configuration parameters
 *  for the AoA DPU
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_maxDetMatrix_Resources_t
{
    /*! @brief  Range Gates count */
    uint16_t     *rangeGatesCount;    

    /*! @brief  Array to store range index for which Doppler FFT is to be applied */
    uint16_t    *range_idx_arr;

    /*! @brief  // Array to store the number of MAX POINTS for a range index for which AoA is to be applied */
    uint16_t    *angle_idx_doppler_count_arr;

    /*! @brief  Array to store Doppler index corresponding to range index for which Angle FFT is to be applied */
    int16_t    *doppler_idx_arr;//Pointer to array of size [MAX_NUM_RANGE_BINS][MAX_NUM_POINTS];

    /*! @brief  Antenna array to store elements before applying elevation-azimuth FFT */
    cmplx32ImRe_t     *antenna_array;

    /*! @brief  Number of max points under consideration */
    uint8_t                max_num_points;

    /*! @brief  Number of antenna elements under consideration */
    uint8_t                antenna_array_elements;
}DPU_AoAProcHWA_maxDetMatrix_Resources;

/**
 * @brief
 *  AoA DPU static configuration parameters
 *
 * @details
 *  The structure is used to hold the static configuration parameters
 *  for the AoA DPU. The following conditions must be satisfied:
 *
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_StaticConfig_t
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
    
    /*! @brief  Log2 of number of Doppler bins */
    uint8_t     log2NumDopplerBins;

    /*! @brief  Flag to set Doppler FFT to 16-bit(1-True)/32-bit(0-False) */
    uint8_t     doppFFT_is16b;

    /*! @brief  Azimuth FFT Size */
    uint8_t     azimuthFFTSize;

    /*! @brief  Log2 of number of Azimuth FFT bins */
    uint8_t     log2NumAzimuthBins;

    /*! @brief  Log2 of number of Elevation FFT bins */
    uint8_t     log2NumElevationBins;

    /*! @brief  Elevation FFT Size */
    uint8_t     elevationFFTSize;  

    /*! @brief  Num of antenna in azimuth dimension*/
    uint8_t     numCols_Antenna;

    /*! @brief  Num of antenna in elevation dimension*/
    uint8_t     numRows_Antenna;  

}DPU_AoAProcHWA_StaticConfig;

/**
 * @brief
 *  AoAProc DPU configuration parameters
 *
 * @details
 *  The structure is used to hold the configuration parameters
 *  for the AoA Processing removal DPU
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_Config_t
{
    /*! @brief HW resources. */
    DPU_AoAProcHWA_HW_Resources  hwRes;
    
    /*! @brief Static configuration. */
    DPU_AoAProcHWA_StaticConfig  staticCfg;

    /*! @brief Configuration for maxDetMatrix*/
    DPU_AoAProcHWA_maxDetMatrix_Resources maxDetmatrix_res;
    
}DPU_AoAProcHWA_Config;


/**
 * @brief
 *  DPU processing output parameters
 *
 * @details
 *  The structure is used to hold the output parameters DPU processing
 *
 *  \ingroup DPU_AoAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcHWA_OutParams_t
{
    /*! @brief DPU statistics */
    DPU_AoAProc_stats  stats;
}DPU_AoAProcHWA_OutParams;

DPU_AoAProcHWA_Handle DPU_AoAProcHWA_init(DPU_AoAProcHWA_InitParams *initCfg, int32_t* errCode);
int32_t DPU_AoAProcHWA_Dopplerprocess(DPU_AoAProcHWA_Handle handle, DPU_AoAProcHWA_OutParams *outParams);
int32_t DPU_AoAProcHWA_AoAprocess(DPU_AoAProcHWA_Handle handle, DPU_AoAProcHWA_OutParams *outParams);
int32_t DPU_AoAProcHWA_deinit(DPU_AoAProcHWA_Handle handle);
int32_t DPU_AoAProcHWA_config(DPU_AoAProcHWA_Handle handle, DPU_AoAProcHWA_Config *cfg);

/**
 *  @b Description
 *  @n
 *      The function returns number of allocated HWA PARAM Sets.
 *
 *  @pre    DPU_CFARProcHWA_config() has been called
 *
 *  @param[in]  handle           CFARCAProcHWA DPU handle
 *
 *  @param[out]  numUsedHwaParamSets           Number of allocated HWA PARAM Sets
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcHWA_GetNumUsedHwaParamSets
(
    DPU_AoAProcHWA_Handle   handle,
    uint8_t *numUsedHwaParamSets
);

#ifdef __cplusplus
}
#endif

#endif