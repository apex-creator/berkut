/**
 *   @file  aoa2dproc.h
 *
 *   @brief
 *      Implements Data path DoA processing functionality.
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
#ifndef AOA2DPROC_INTERNAL_H
#define AOA2DPROC_INTERNAL_H

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
#include <datapath/dpu/aoa2dproc/v0/aoa2dproc.h>
#include <datapath/dpif/dpif_hwa.h>


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief
 *  HWA loop parameters
 *
 *  \ingroup DPU_AOA2DPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_hwaAngleProcOutAddr_t
{
    /*! @brief  Pointer to Azimuth FFT Outputs in HWA */
    int32_t *azimFftOutPtr;

    /*! @brief  Pointer to elevation FFT stats output in HWA (list of peaks and corresponding elevation indices */
    DPIF_HWA_STATS_MaxOutput *azimSpectElevIndPtr;

    /*! @brief  Pointer to maximum azimuth speak in a spectrum in HWA */
    DPIF_HWA_STATS_MaxOutput *azimMaxPeakAzimIndPtr;

    /*! @brief  Pointer to list of peak values and positions in azimuth spectrum in HWA */
    DPIF_HWA_CFAR_DetOutput  *azimPeakList;
} DPU_Aoa2dProc_hwaAngleProcOutAddr;

/**
 * @brief
 *  HWA param start index and number of params
 */
typedef struct DPU_Aoa2dProc_HwaParamCfg_t
{
    /*! @brief   HWA param start index */
    uint16_t    startIdx;

    /*! @brief   number of HWA params */
    uint16_t    numParams;
} DPU_Aoa2dProc_HwaParamCfg;



/**
 * @brief
 *  dopplerProc DPU internal data Object
 *
 * @details
 *  The structure is used to hold dopplerProc internal data object
 *
 *  \ingroup DPU_DOPPLERPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_Aoa2dProc_Obj_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
    
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;
    uint32_t    edmaInstanceId;

    /*! @brief  EDMA configuration for Input data (Radar cube -> HWA memory). */
    DPEDMA_ChanCfg edmaIn;


    /*! @brief HWA Processing Done semaphore Handle */
    SemaphoreP_Object  hwaDoneSemaHandle;

    /*! @brief EDMA Done semaphore Handle */
    SemaphoreP_Object  edmaDoneSemaHandle;
    
    /*! @brief Flag to indicate if DPU is in processing state */
    bool inProgress;

    /*! @brief  DMA trigger source channel */
    uint8_t dmaTrigSrcChan;

    /*! @brief  HWA memory bank addresses */
    uint32_t hwaMemBankAddr[DPU_AOA2DPROC_NUM_HWA_MEMBANKS];

    /*! @brief  Summation division shift for Doppler FFT non-coherent integration */
    uint8_t dopFftSumDiv;

    /*! @brief   Range loop type: 0 - HWA internal loop, 1- Loop controlled by CPU */
    uint16_t    aoa2dRangeLoopType;

    /*! @brief   Doppler computation HWA param start index, and number of used params */
    DPU_Aoa2dProc_HwaParamCfg dopplerHwaComp;

    /*! @brief   Angle computation HWA param start index, and number of used params */
    DPU_Aoa2dProc_HwaParamCfg angleHwaComp;

    /*! @brief  Radar Cube */
    DPIF_RadarCube radarCube;

    /*! @brief HWA Doppler FFT Parmas set configuration (performing Doppler FFT and antenna mapping)
    */
    DPU_Aoa2dProc_HWA_Option_Cfg aoa2dRngGateCfg;

    /*! @brief Saved DPU configuration,ToDo remove other duplicate variables
    */
    DPU_Aoa2dProc_Config config;

    /*! @brief HWA memory addresses of the angle processing output
    */
    DPU_Aoa2dProc_hwaAngleProcOutAddr hwaAngleProcOut;

    /*! @brief Number of samples to be skipped in detection process
    */
    DPU_Aoa2dProc_detectionCfg detectionCfg;

    /*! @brief Location of the decompression HWA Param sset
    */
    uint32_t *decompressHwaParamSetPtr;

    /*! @brief Two copies of decompression param set, [0] for even range gate index, [1] for odd range gate index
    */
    uint32_t decompressHwaParams[2][DPU_AOA2DPROC_HWA_NUM_REGS_PER_PARAM_SET];

    uint32_t  decompressHwaParamSetIdx;
    
}DPU_Aoa2dProc_Obj;


#ifdef __cplusplus
}
#endif

#endif
