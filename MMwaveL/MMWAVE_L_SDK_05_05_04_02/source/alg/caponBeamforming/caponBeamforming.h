/**
 *   @file  caponbeamforming.h
 *
 *   @brief
 *      Implements 1D Capon Beamforming functionality.
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
#ifndef CAPONBEAMFORMING_H
#define CAPONBEAMFORMING_H

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
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dp_error.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_CAPONBEAMFORMING_ERROR_CODE
 *  Base error code for the caponbeamforming DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define CAPONBEAMFORMINGHWA_EINVAL                  (DP_ERRNO_CAPONBEAMFORMING_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define CAPONBEAMFORMINGHWA_ENOMEM                  (DP_ERRNO_CAPONBEAMFORMING_BASE-2)

/**
 * @brief   Error Code: Out of HWA resources
 */
#define CAPONBEAMFORMINGHWA_EHWARES                 (DP_ERRNO_CAPONBEAMFORMING_BASE-3)

/**
 * @brief   Error Code: Semaphore creation failed
 */
#define CAPONBEAMFORMINGHWA_ESEMA                   (DP_ERRNO_CAPONBEAMFORMING_BASE-4)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size 
 */
#define CAPONBEAMFORMINGHWA_EEXCEEDHWAMEM           (DP_ERRNO_CAPONBEAMFORMING_BASE-5)

/**
 * @brief   Error Code: HWA Correlation Matrix and Capon Spectrum paramset indices do not conflict  
 */
#define CAPONBEAMFORMINGHWA_HWAPARAMSETSCONFLICT    (DP_ERRNO_CAPONBEAMFORMING_BASE-5)

/**
 * @brief   Number of HWA memory banks needed
 */
#define CAPONBEAMFORMINGHWA_NUM_HWA_MEMBANKS  4

         
/*!
 *  @brief   Handle for Capon Beamforming DPU.
 */
typedef void*  CaponBeamformingHWA_Handle;

/**
 * @brief
 *  Capon Beamforming DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations.
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_InitCfg_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
    
} CaponBeamformingHWA_InitParams;

/**
 * @brief
 *  Capon Beamforming DPU HWA configuration parameters
 *
 * @details
 *  The structure is used to hold the HWA configuration parameters
 *  for the Capon Beamforming DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_HwaCfg_t
{
    /*! @brief  PaRAM set start index for correlation matrix computation. */
    uint32_t    corrMatParamSetIdx;

    /*! @brief  PaRAM set start index for capon spectrum computation. */
    uint32_t    caponSpectrumParamSetStartIdx;
} CaponBeamformingHWA_HwaCfg;

/**
 * @brief
 *  Capon Beamforming DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Capon Beamforming DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamforming_Edma_t
{
    /*! @brief  EDMA channel for the Correlation Matrix. */
    DPEDMA_ChanCfg  correlationMatrixChannel;
    /*! @brief  EDMA channel for the Capon Spectrum. */
    DPEDMA_ChanCfg  caponSpectrumChannel;    
}CaponBeamforming_Edma;

/**
 * @brief
 *  Capon Beamforming DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Capon Beamforming DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_EdmaCfg_t
{
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;
    
    /*! @brief  EDMA configuration for Inputting data to the HWA */
    CaponBeamforming_Edma edmaIn;

    /*! @brief  EDMA configuration for Output data from the HWA */
    CaponBeamforming_Edma edmaOut;
    
    /*! @brief  EDMA configuration for hot signature. */
    CaponBeamforming_Edma edmaHotSig;
} CaponBeamformingHWA_EdmaCfg;

/**
 * @brief
 *  Capon Beamforming HW configuration parameters
 *
 * @details
 *  The structure is used to hold the HW configuration parameters
 *  for Capon Beamforming
 *
 *  \ingroup CAPON_BEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_HW_Resources_t
{
    /*! @brief  EDMA configuration */
    CaponBeamformingHWA_EdmaCfg edmaCfg;
    
    /*! @brief  HWA configuration */
    CaponBeamformingHWA_HwaCfg  hwaCfg;
    
    /*! @brief  Input Data */
    int16_t *caponBeamformingInputData;

    /*! @brief  Capon Spectrum*/
    int32_t *caponSpectrum;

    /*! @brief     EDMA interrupt object for the EDMA completion interrupt after 
                   transfer of Correlation Matrix from the HWA to the M4 */ 
    /* NOTE: Application needs to provide address of the EDMA interrupt object.
     * This needs to be done as there might be multiple subframes configured
     * and each subframe needs EDMA interrupt to be registered.
     */
    Edma_IntrObject *intrObjCorrelationMatrix;

    /*! @brief     EDMA interrupt object for the EDMA completion interrupt after 
                   transfer of Capon Spectrum from the HWA to L3 */  
    /* NOTE: Application needs to provide address of the EDMA interrupt object.
     * This needs to be done as there might be multiple subframes configured
     * and each subframe needs EDMA interrupt to be registered.
     */
    Edma_IntrObject *intrObjCaponSpectrum;
} CaponBeamformingHWA_HW_Resources;

/**
 * @brief
 *  Capon Beamforming DPU static configuration parameters
 *
 * @details
 *  The structure is used to hold the static configuration parameters
 *  for the Capon Beamforming DPU. The following conditions must be satisfied:
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;
    
    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;
    
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
    
    /*! @brief  Number of range bins */
    uint16_t    numSnapshots;

    /*! @brief  Physical address of the HWA Internal RAM  */
    uint32_t    internalRAMAddress;

    /*! @brief  The number of angles to sample in the Capon Spectrum  */
    uint8_t     numAnglesToSample;
    
} CaponBeamformingHWA_StaticConfig;

/**
 * @brief
 *  Capon Beamforming DPU configuration parameters
 *
 * @details
 *  The structure is used to hold the configuration parameters
 *  for the Capon Beamforming removal DPU
 *
 *  \ingroup DPU_CAPONBEAMFORMING_EXTERNAL_DATA_STRUCTURE
 */
typedef struct CaponBeamformingHWA_Config_t
{
    /*! @brief HW resources. */
    CaponBeamformingHWA_HW_Resources  hwRes;
    
    /*! @brief Static configuration. */
    CaponBeamformingHWA_StaticConfig  staticCfg;
    
}CaponBeamformingHWA_Config;


CaponBeamformingHWA_Handle CaponBeamformingHWA_init(CaponBeamformingHWA_InitParams *initCfg, int32_t* errCode);
int32_t CaponBeamformingHWA_process(CaponBeamformingHWA_Handle handle, CaponBeamformingHWA_Config *cfg);
int32_t CaponBeamformingHWA_deinit(CaponBeamformingHWA_Handle handle, CaponBeamformingHWA_Config *cfg);
int32_t CaponBeamformingHWA_config(CaponBeamformingHWA_Handle handle, CaponBeamformingHWA_Config *cfg);

#ifdef __cplusplus
}
#endif

#endif
