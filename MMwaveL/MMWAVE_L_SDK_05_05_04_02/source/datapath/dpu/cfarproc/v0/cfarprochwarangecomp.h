/**
 *   @file  cfarprochwarangecomp.h
 *
 *   @brief
 *      Implements range compensation for CFAR Processing with HWA.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2024-2025 Texas Instruments, Inc.
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


#ifndef CFAR_PROC_HWA_RANGE__COMP_H
#define CFAR_PROC_HWA_RANGE__COMP_H
/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <datapath/dpif/dpif_pointcloud.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RANGE_FFT_ANTI_ALIAS_SAFETY_FACTOR 0.9

/*!
 *  @brief    Holds the parameters that describe the SNR compensation over range and angle for point detection
 *
 *  \ingroup DPU_CFARPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct CFAR_DET_HWA_RangeComp_Config_t
{
    /*! @brief  1-enabled 0-disabled */
    uint8_t enabled;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float detectionRangeIdx;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float detectionSNR;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float minCompRange;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float maxCompRange;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float minCompAngle1;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float maxCompAngle1;

    /*! @brief  Range Index for distance corresponding to the SNR in detectionSNR */
    float minCompAngle2;

    /*! @brief  SNR corresponding to the distance in detectionRange */
    float maxCompAngle2;

    /*! @brief  SNR difference between first and second compensation angles */
    float snrDropfromAngle1ToAngle2;


} CFAR_DET_HWA_RangeComp_Config;

CFAR_DET_HWA_RangeComp_Config rangeCompCfg;

/**
 *  @b Description
 *  @n
 *      Function to compute whether candidate point has enough SNR to become a point in the point cloud.
 *
 *  @param[in]  inputRange                    The range of the point detected
 *  @param[in]  secondaryCompSNRDrop          Subtraction factor to be included if necessary
 * 
 *  @param[out]  detectionSNR                 The SNR needed to allocate a point at this range.
 *
 *  \ingroup    DPU_CFARPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
float CFAR_DET_HWA_RangeComp_SNRNeededToKeepPoint(float inputRange, float secondaryCompSNRDrop);

#ifdef __cplusplus
}
#endif

#endif
