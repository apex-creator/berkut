/**
 *   @file  cfarprochwarangecomp.c
 *
 *   @brief
 *      Implements range compensation for CFAR Processing with HWA.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014-2025 Texas Instruments, Inc.
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


#include <datapath/dpu/cfarproc/v0/cfarprochwarangecomp.h>

float gMaxSatYVal = 0;
float gMinSatYVal = 0;

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
float CFAR_DET_HWA_RangeComp_SNRNeededToKeepPoint(float inputRangeIdx, float secondaryCompSNRDrop)
{
    /* If there are a min/maxCompensatedRange set in the CLI, use them */
    if (rangeCompCfg.minCompRange > -1)
    {

        /* Populate the min and max y vals on the first instance, then we don't need to recompute */
        if (gMaxSatYVal == 0)
        {
            gMaxSatYVal = rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / rangeCompCfg.minCompRange);
        }
        if (gMinSatYVal == 0)
        {
            gMinSatYVal = rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / rangeCompCfg.maxCompRange);
        }

        /* See if the point meets the range-specific threshold */
        if (inputRangeIdx < rangeCompCfg.minCompRange)
        {
            return gMaxSatYVal - secondaryCompSNRDrop;
        }
        else if (inputRangeIdx >= rangeCompCfg.minCompRange && inputRangeIdx < rangeCompCfg.maxCompRange)
        {
            /* A lookup table could be added and populated at startup to accelerate computation if needed instead of recomputing per-point. */
            return rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / inputRangeIdx) - secondaryCompSNRDrop;
        }
        else
        {
            return gMinSatYVal - secondaryCompSNRDrop;
        }
    }

    /* If no min/maxCompensatedRanges are set, then just use the single function (unsaturated). */
    else
    {
        return rangeCompCfg.detectionSNR + 40 * log10f(rangeCompCfg.detectionRangeIdx / inputRangeIdx) - secondaryCompSNRDrop;
    }
}
