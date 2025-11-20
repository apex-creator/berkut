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
 *   @file  aoaprochwa_internal.h
 *
 *   @brief
 *      AoAProcHWA internal definitions.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef DPU_AoAPROC_INTERNAL_H
#define DPU_AoAPROC_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmwave SDK include files */
#include <datapath/dpu/aoaproc/v0/aoaproc_common.h>
#include <datapath/dpu/aoaproc/v0/aoaprochwa.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  AoAProcHWA DPU Object
 *
 * @details
 *  The structure is used to hold AoAProcHWA internal data object
 *
 *  \ingroup DPU_AoAPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct aoaProcHWAObj_t
{
    DPU_AoAProcHWA_InitParams     initParms;

    /*! @brief      EDMA Handle */
    HWA_Handle              hwaHandle;

    /*! @brief      EDMA Handle of TPCC B*/
    EDMA_Handle             edmaHandle;

    /*! @brief     aoaProc HWA configuration */
    DPU_AoAProcHWA_HwaCfg   hwaCfg;

    /*! @brief     aoaProc EDMA configuration before Doppler FFT is applied*/
    DPU_AoAProc_Edma        edmaIn_rangeFFT;

    /*! @brief     aoaProc EDMA configuration before AoA FFT's are applied*/
    DPU_AoAProc_Edma        edmaIn_dopplerFFT;

    /*! @brief     aoaProc EDMA configuration  AoA FFT's are applied*/
    DPU_AoAProc_Edma        edmaOut_maxAoA;

    /*! @brief     EDMA done semaphore */
    SemaphoreP_Object       edmaDoneSemaHandle;

    /*! @brief     HWA Processing Done (Doppler) semaphore Handle */
    SemaphoreP_Object       hwaDoneSemaHandleDopp;

    /*! @brief     HWA Processing Done (AoA) semaphore Handle */
    SemaphoreP_Object       hwaDoneSemaHandleAoA;

    /*! @brief     HWA Paramset Done semaphore Handle for ping/pong in Doppler use case*/
    SemaphoreP_Object       hwaSemaHandleParamset;

    /*! @brief      Pointer to Radar Cube buffer */
    cmplx16ImRe_t           *radarCubebuf;

    /*! @brief      Pointer to Angle Matrix buffer */
    cmplx16ImRe_t           *angleMatBuf;

    /*! @brief      HWA Memory address */
    uint32_t                hwaMemBankAddr[SOC_HWA_NUM_MEM_BANKS];

    /*! @brief     AoAProc DPU is in processing state */
    bool                    inProgress;

    /*! @brief     Total number of AoAProc DPU processing */
    uint32_t                numProcess;

    /*! @brief  DMA trigger source channel (before Doppler FFT) for Ping param set */
    uint8_t                 hwaDmaTriggerDoppPing;
    
    /*! @brief  DMA trigger source channel (before Doppler FFT) for Pong param set */
    uint8_t                 hwaDmaTriggerDoppPong;   

    /*! @brief  DMA trigger source channel (before AoA FFT) for Ping param set */
    uint8_t                 hwaDmaTriggerAoAPing;
    
    /*! @brief  DMA trigger source channel (before AoA FFT) for Pong param set */
    uint8_t                 hwaDmaTriggerAoAPong;     
            
    /*! @brief  HWA number of loops for Doppler FFT and compensation*/
    uint16_t                hwaDoppNumLoops;

    /*! @brief  HWA number (1-basically if the no. is odd) of loops for Doppler FFT and compensation*/
    uint16_t                hwaDoppNumLoops1;

    /*! @brief  HWA number of loops for AoA and max of Elevation-Azimuth FFT*/
    uint16_t                *hwaAoANumLoops;

    /*! @brief  HWA start paramset index - Doppler FFT and compensation*/
    uint8_t                 hwaParamStartIdxDopp;
    
    /*! @brief  HWA stop paramset index - Doppler FFT and compensation*/
    uint8_t                 hwaParamStopIdxDopp;

    /*! @brief  HWA start paramset index - AoA Calculation*/
    uint8_t                 hwaParamStartIdxAoA;
    
    /*! @brief  HWA stop paramset index - AoA Calculation*/
    uint8_t                 hwaParamStopIdxAoA;

    /*! @brief  Doppler FFT to be run on those range gates which have a maximum in first 50-points*/
    uint16_t                 *rangeGatesCount;

    /*! @brief  Array to store range index for which Doppler FFT is to be applied */
    uint16_t                *range_idx_arr;

    /*! @brief  Array to store Doppler index corresponding to range index for which Angle FFT is to be applied */
    uint16_t               *angle_idx_doppler_count_arr;

    /*! @brief  Array to store Doppler index corresponding to range index for which Angle FFT is to be applied */
    int16_t                *doppler_idx_arr;//Pointer to array of size [MAX_NUM_RANGE_BINS][MAX_NUM_POINTS];

    /*! @brief  Antenna array to store elements before applying elevation-azimuth FFT */
    cmplx32ImRe_t          *antenna_array;

    /*! @brief  Number of transmit antennas */
    uint8_t                numTxAntennas;

    /*! @brief  Flag to set Doppler FFT to 16-bit(1-True)/32-bit(0-False) */
    uint8_t                doppFFT_is16b;

    /*! @brief  Number of virtual antennas */
    uint8_t                numVirtualAntennas;

    /*! @brief  Number of max points under consideration */
    uint8_t                max_num_points;

    /*! @brief  Number of max points under consideration */
    uint8_t                antenna_array_elements;

    /*! @brief  Flag to check Doppler compensations is enabled or not */
    uint8_t                *isDopplerPhaseCompensationEnabled;
}aoaProcHWAObj;

#ifdef __cplusplus
}
#endif

#endif
