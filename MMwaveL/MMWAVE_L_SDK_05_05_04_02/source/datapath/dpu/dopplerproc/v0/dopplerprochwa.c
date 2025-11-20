/**
 *   @file  dopplerprochwa.c
 *
 *   @brief
 *      Implements Data path Doppler processing Unit using HWA.
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK driver/common Include Files */
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/edma.h>
#include <drivers/esm.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include <drivers/hwa.h>
#include <kernel/dpl/CycleCounterP.h>

/* Data Path Include files */
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <datapath/dpu/dopplerproc/v0/dopplerprochwa.h>
#include <datapath/dpu/dopplerproc/v0/dopplerproc_internal.h>

/* Utils */
#include <utils/mathutils/mathutils.h>

/* User defined heap memory and handle */
#define DOPPLERPROCHWA_HEAP_MEM_SIZE  (sizeof(dopplerProcHWAObj))

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

/* HWA ping/pong buffers offset */
#define DPU_DOPPLERPROCHWA_SRC_PING_OFFSET   dopplerProcObj->hwaMemBankAddr[0]
#define DPU_DOPPLERPROCHWA_DST_PING_OFFSET   dopplerProcObj->hwaMemBankAddr[1]
#define DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET   dopplerProcObj->hwaMemBankAddr[2]
#define DPU_DOPPLERPROCHWA_DST_PONG_OFFSET   dopplerProcObj->hwaMemBankAddr[3]

static uint8_t gDopplerProcHeapMem[DOPPLERPROCHWA_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/*===========================================================
 *                    Internal Functions
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function.
 *  \ingroup    DPU_DOPPLERPROC_INTERNAL_FUNCTION
 */
void dopplerProcHWADoneIsrCallback(void *arg)
{   
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA completion call back function.
 *  \ingroup    DPU_DOPPLERPROC_INTERNAL_FUNCTION
 */
void dopplerProcHWA_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *  Doppler DPU EDMA configuration.
 *  This implementation of doppler processing involves Ping/Pong 
 *  Mechanism, hence there are two sets of EDMA transfer.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOPPLERPROC_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static int32_t dopplerProcHWA_configEdma
(
    dopplerProcHWAObj      *dopplerProcObj,
    DPU_DopplerProcHWA_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    cmplx16ImRe_t       *radar_1DFFT_base = (cmplx16ImRe_t *)cfg->hwRes.radar_1D_FFT_Cube.data;
    uint32_t            *detMatrixBase = (uint32_t *)cfg->hwRes.detMatrix.data;
    int16_t             sampleLenInBytes = sizeof(cmplx16ImRe_t);
    uint32_t            sizeOfMatrixTransfer = cfg->staticCfg.numDopplerBins*2;//As 2-range bins are processed per ping/pong
    uint32_t            sizeOfMatrixTransferBytes;
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;
	int16_t             outputSampLenInBytes;

    if(dopplerProcObj == NULL)
    {
        retVal = DPU_DOPPLERPROCHWA_EINVAL;
        goto exit;
    }
    if (cfg->staticCfg.isDetMatrixLogScale)
    {
        outputSampLenInBytes = sizeof(uint16_t);
    }
    else
    {
        outputSampLenInBytes = sizeof(uint32_t);
    }
    sizeOfMatrixTransferBytes = sizeOfMatrixTransfer * outputSampLenInBytes;
    
    /*********************************PROGRAM DMA'S FOR PING**************************************/
        
    /***************************************************************************
     *  EDMA OUT (ping)
     *  PROGRAM DMA channel to transfer detection matrix from accelerator output
     *  buffer (ping) to HWA_SS RAM
     **************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn.ping.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = false;

    if (cfg->staticCfg.isDetMatrixLogScale)
    {
        syncABCfg.srcAddress  = (uint32_t)(dopplerProcObj->hwaMemBankAddr[1]);
        syncABCfg.srcBIdx     = sizeof(uint16_t);
    }
    else
    {
        syncABCfg.srcAddress  = (uint32_t)(dopplerProcObj->hwaMemBankAddr[0] + sizeof(int32_t));
        syncABCfg.srcBIdx     = sizeof(DPU_DopplerProcHWA_HwaSumOutput);
    }
    syncABCfg.destAddress = (uint32_t)SOC_virtToPhy(&detMatrixBase[0]);
    syncABCfg.aCount      = outputSampLenInBytes;
    syncABCfg.bCount      = sizeOfMatrixTransfer;
    syncABCfg.cCount      = cfg->staticCfg.numRangeBins / 4U;//factor of 4 due to 2-range bins in ping/pong
    syncABCfg.dstBIdx     = outputSampLenInBytes;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstCIdx     = sizeOfMatrixTransferBytes * 2U; 

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaOutDetectionMatrix.ping,
                                &chainingCfg,
                                &syncABCfg,
                                true, //isEventTriggered
                                false,//isIntermediateTransferInterruptEnabled
                                false,//isTransferCompletionEnabled
                                NULL, //transferCompletionCallbackFxn
                                NULL,//transferCompletionCallbackFxnArg
                                NULL);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /*****************************************************************************************
    * EDMA INPUT (ping)
    * PROGRAM DMA channel  to transfer data from Radar cube to accelerator input buffer (ping)*
    ******************************************************************************************/    
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.ping.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)(&radar_1DFFT_base[0]);
    if (!cfg->staticCfg.isCompressionEnabled)
    {
        syncABCfg.destAddress = (uint32_t)(dopplerProcObj->hwaMemBankAddr[0]);
        syncABCfg.aCount      = 2 * sampleLenInBytes;
        syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numDopplerChirps *  cfg->staticCfg.numTxAntennas;
        syncABCfg.cCount      = cfg->staticCfg.numRangeBins / 4;//factor of 4 due to 2-range bins per ping/pong
        syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sampleLenInBytes;
        syncABCfg.dstBIdx     = 2 * sampleLenInBytes;
        syncABCfg.srcCIdx     = 4 * sampleLenInBytes;//factor of 4 due to 2-range bins per ping/pong
        syncABCfg.dstCIdx     = 0;
    }
    else
    {
        syncABCfg.destAddress = (uint32_t)(dopplerProcObj->hwaMemBankAddr[1]);
        if (cfg->staticCfg.compressCfg.compressionFactor == 2)
        {
            syncABCfg.aCount      = sizeof(uint32_t);
            syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins/2 * sizeof(uint32_t);
            syncABCfg.dstBIdx     = sizeof(uint32_t);
            syncABCfg.srcCIdx     = 2 * sizeof(uint32_t);// to next ping
        }
        else if(cfg->staticCfg.compressCfg.compressionFactor == 4)
        {
            syncABCfg.aCount      = sizeof(uint16_t);
            syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins/2 * sizeof(uint16_t);
            syncABCfg.dstBIdx     = sizeof(uint16_t);
            syncABCfg.srcCIdx     = 2 * sizeof(uint16_t);// to next ping
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
        syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numDopplerChirps *  cfg->staticCfg.numTxAntennas;
        syncABCfg.cCount      = cfg->staticCfg.numRangeBins / 4;//factor of 4 due to numberOfRnageBins/2 chunks per ping/pong
        syncABCfg.dstCIdx     = 0;
    }

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn.ping,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL,//transferCompletionCallbackFxnArg
                                 NULL);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }
    
    /******************************************************************************************
    *  EDMA SIGNATURE (ping)
    *  PROGRAM DMA channel to trigger HWA for processing of input (ping)
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSig.ping,
                                             dopplerProcObj->hwaHandle,
                                             dopplerProcObj->hwaDmaTriggerSourcePing,
                                             false);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************PROGRAM DMA'S FOR PONG***********************************/

    /*****************************************************************************
     *  EDMA OUT (pong)
     *  PROGRAM DMA channel  to transfer detection matrix from accelerator output
     *  buffer (pong) to HWA_SS
     *****************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn.pong.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = false;

    /* Transfer parameters are the same as ping, except for src/dst addresses */
    if (cfg->staticCfg.isDetMatrixLogScale)
    {
        syncABCfg.srcAddress  = (uint32_t)(dopplerProcObj->hwaMemBankAddr[3]);
        syncABCfg.srcBIdx     = sizeof(uint16_t);
    }
    else
    {
        syncABCfg.srcAddress  = (uint32_t)(dopplerProcObj->hwaMemBankAddr[2] + sizeof(int32_t));
        syncABCfg.srcBIdx     = sizeof(DPU_DopplerProcHWA_HwaSumOutput);
    }
    syncABCfg.destAddress = (uint32_t)SOC_virtToPhy(&detMatrixBase[0]) + sizeOfMatrixTransferBytes;
    syncABCfg.aCount      = outputSampLenInBytes;
    syncABCfg.bCount      = sizeOfMatrixTransfer;
    syncABCfg.cCount      = cfg->staticCfg.numRangeBins / 4U;//factor of 4 due to 2-range bins per ping/pong
    syncABCfg.dstBIdx     = outputSampLenInBytes;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstCIdx     = sizeOfMatrixTransferBytes * 2U;
    
    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaOutDetectionMatrix.pong,
                                &chainingCfg,
                                &syncABCfg,
                                true, //isEventTriggered
                                false,//isIntermediateTransferInterruptEnabled
                                true, //isTransferCompletionEnabled
                                dopplerProcHWA_edmaDoneIsrCallback, //transferCompletionCallbackFxn
                                (void *)&dopplerProcObj->edmaDoneSemaHandle,//transferCompletionCallbackFxnArg
                                cfg->hwRes.intrObj); 
    
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************
     *  EDMA INPUT (pong)
     *  PROGRAM DMA channel  to transfer data from HWA_SS to accelerator input buffer (pong)
     ******************************************************************************************/ 
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.pong.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    if (!cfg->staticCfg.isCompressionEnabled)
    {
        syncABCfg.srcAddress  = (uint32_t)(&radar_1DFFT_base[2]);
        syncABCfg.destAddress = (uint32_t)(dopplerProcObj->hwaMemBankAddr[2]);
        syncABCfg.aCount      = 2 * sampleLenInBytes;
        syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numDopplerChirps *  cfg->staticCfg.numTxAntennas;
        syncABCfg.cCount      = cfg->staticCfg.numRangeBins / 4;//factor of 4 due to 2-range bins per ping/pong
        syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sampleLenInBytes;
        syncABCfg.dstBIdx     = 2 * sampleLenInBytes;
        syncABCfg.srcCIdx     = 4 * sampleLenInBytes;//factor of 4 due to 2-range bins per ping/pong
        syncABCfg.dstCIdx     = 0;
    }
    else
    {
        syncABCfg.destAddress = (uint32_t)(dopplerProcObj->hwaMemBankAddr[3]);
        if (cfg->staticCfg.compressCfg.compressionFactor == 2)
        {
            syncABCfg.srcAddress  = (uint32_t)(&radar_1DFFT_base[0]) + sizeof(uint32_t);
            syncABCfg.aCount      = sizeof(uint32_t);
            syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins/2 * sizeof(uint32_t);
            syncABCfg.dstBIdx     = sizeof(uint32_t);
            syncABCfg.srcCIdx     = 2 * sizeof(uint32_t);// to next ping
        }
        else if(cfg->staticCfg.compressCfg.compressionFactor == 4)
        {
            syncABCfg.srcAddress  = (uint32_t)(&radar_1DFFT_base[0]) + sizeof(uint16_t);
            syncABCfg.aCount      = sizeof(uint16_t);
            syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins/2 * sizeof(uint16_t);
            syncABCfg.dstBIdx     = sizeof(uint16_t);
            syncABCfg.srcCIdx     = 2 * sizeof(uint16_t);// to next ping
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
        syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numDopplerChirps *  cfg->staticCfg.numTxAntennas;
        syncABCfg.cCount      = cfg->staticCfg.numRangeBins / 4;//factor of 4 due to numberOfRnageBins/2 chunks per ping/pong
        syncABCfg.dstCIdx     = 0;
    }

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn.pong,
                                 &chainingCfg,
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL,//transferCompletionCallbackFxnArg
                                 NULL);
    
    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************************************************************************
    *  EDMA SIGNATURE (pong)
    *  PROGRAM DMA channel to trigger HWA for processing of input (pong)
    ******************************************************************************************/  
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSig.pong,
                                             dopplerProcObj->hwaHandle,
                                             dopplerProcObj->hwaDmaTriggerSourcePong,
                                             false);

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
 *      Configures HWA for Doppler processing where range-doppler cube and heatmap are calculated.
 *
 *  @param[in] dopplerProcobj    - DPU obj
 *  @param[in] cfg               - DPU configuration
 *
 *  \ingroup    DPU_DOPPLERPROC_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
 
 static int32_t dopplerProcHWA_ConfigHWA
 (
    dopplerProcHWAObj     *dopplerProcObj,
    DPU_DopplerProcHWA_Config    *cfg
 )
 {
    HWA_ParamConfig         hwaParamCfg[DPU_DOPPLERPROCHWA_MAX_NUM_HWA_PARAMSET];
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;
    uint8_t                 destChan;
    uint8_t                 errCode=0;

    uint32_t decompressIdx;
    uint32_t dopplerFftRngBin1Idx;
    uint32_t dopplerFftRngBin2Idx;
    uint32_t sumAbsRngBin1Idx;
    uint32_t sumAbsRngBin2Idx;
    uint32_t log2MagIdx;

    if (cfg->staticCfg.isCompressionEnabled)
    {
        decompressIdx = paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
        /****** Ping paramset of Doppler FFT for 1st Range bin (by EDMA in) ******/
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg[paramsetIdx].dmaTriggerSrc = dopplerProcObj->hwaDmaTriggerSourcePing;
        hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET); // MEM BANK M1
        hwaParamCfg[paramsetIdx].source.srcAcnt = 1 - 1;
        hwaParamCfg[paramsetIdx].source.srcBcnt = (cfg->staticCfg.numVirtualAntennas * cfg->staticCfg.numDopplerChirps) - 1;
        hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        if (cfg->staticCfg.compressCfg.compressionFactor  == 2)
        {
            hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t);
            hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint32_t);
            hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
            hwaParamCfg[paramsetIdx].source.srcScale = 0;
        }
        else if (cfg->staticCfg.compressCfg.compressionFactor == 4)
        {
            hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint16_t);
            hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint16_t);
            hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
            hwaParamCfg[paramsetIdx].source.srcScale = 8;
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
        hwaParamCfg[paramsetIdx].source.srcShift = 0;
        hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0;

        hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
        hwaParamCfg[paramsetIdx].dest.dstAcnt = cfg->staticCfg.compressCfg.numComplexElements - 1;; //this is samples - 1
        hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
        hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg[paramsetIdx].dest.dstBIdx = cfg->staticCfg.compressCfg.numComplexElements * sizeof(cmplx16ImRe_t);
        hwaParamCfg[paramsetIdx].dest.dstScale = 0;

        hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
        hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;

        hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.compressDecompress = HWA_CMP_DCMP_DECOMPRESS;
        hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_ENABLE;  // Enable dither to suppress quantization spurs
        hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
        hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.method = HWA_COMPRESS_METHOD_BFP;
        hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
        if (cfg->staticCfg.compressCfg.compressionFactor  == 2)
        {
            hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.BFPMantissaBW = 7;
            hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4;
        }
        else if (cfg->staticCfg.compressCfg.compressionFactor == 4)
        {
            hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.BFPMantissaBW = 3;
            hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4;
        }
        else
        {
            retVal = SystemP_INVALID_PARAM;
            goto exit;
        }
        retVal = HWA_configParamSet(dopplerProcObj->hwaHandle,
                                    cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle,
                                              cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }
        ++paramsetIdx;
    }

    dopplerFftRngBin1Idx = paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    /****** Ping paramset of Doppler FFT for 1st Range bin (by EDMA in) ******/ 
    if (!cfg->staticCfg.isCompressionEnabled)
    {
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg[paramsetIdx].dmaTriggerSrc = dopplerProcObj->hwaDmaTriggerSourcePing;
    }
    else
    {
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    }
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 

    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
    hwaParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1; //size in samples - 1

    hwaParamCfg[paramsetIdx].source.srcAIdx = 2 * cfg->staticCfg.numVirtualAntennas * sizeof(cmplx16ImRe_t); 
    hwaParamCfg[paramsetIdx].source.srcBcnt = cfg->staticCfg.numVirtualAntennas - 1; 
    hwaParamCfg[paramsetIdx].source.srcBIdx = 2 * sizeof(cmplx16ImRe_t); 
    hwaParamCfg[paramsetIdx].source.srcShift = 0; 
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; 
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0; 
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0; 
    hwaParamCfg[paramsetIdx].source.srcScale = 8;
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET); // MEM BANK M1
    hwaParamCfg[paramsetIdx].dest.dstAcnt = cfg->staticCfg.numDopplerBins - 1; //this is samples - 1
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    if(cfg->staticCfg.doppFFT_is16b)
    {
        hwaParamCfg[paramsetIdx].dest.dstAIdx = cfg->staticCfg.numVirtualAntennas * sizeof(cmplx16ImRe_t);
        hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg[paramsetIdx].dest.dstScale = 0;
    }
    else
    {
        hwaParamCfg[paramsetIdx].dest.dstAIdx = cfg->staticCfg.numVirtualAntennas * sizeof(cmplx32ImRe_t);
        hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(cmplx32ImRe_t);
        hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg[paramsetIdx].dest.dstScale = 8;
    }
    
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumDopplerBins;

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0;

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    if(hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize>2)
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 1;
    else
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.winRamOffset; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.winSym; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    /****** Ping paramset of Doppler FFT for 2nd Range bin ******/ 
    ++paramsetIdx;
    dopplerFftRngBin2Idx = paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[dopplerFftRngBin1Idx];
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; 
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET)+0x4;// MEM BANK M0 + 0x4
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET)+cfg->staticCfg.numDopplerBins*cfg->staticCfg.numVirtualAntennas*hwaParamCfg[paramsetIdx].dest.dstBIdx;// MEM BANK M1 + 0xC00
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    /****** Ping paramset of Sum Abs of 1st Range bin ******/ 
    ++paramsetIdx;
    sumAbsRngBin1Idx = paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; 
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 

    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET); // MEM BANK M1
    hwaParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.numVirtualAntennas - 1; //size in samples - 1

    if(cfg->staticCfg.doppFFT_is16b)
    {
        hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(cmplx16ImRe_t); 
        hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg[paramsetIdx].source.srcBIdx = cfg->staticCfg.numVirtualAntennas * sizeof(cmplx16ImRe_t); 
        hwaParamCfg[paramsetIdx].source.srcScale = 8;//Fix this to 8 to achieve near bit-matching with MATLAB 
    }
    else
    {
        hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(cmplx32ImRe_t); 
        hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg[paramsetIdx].source.srcBIdx = cfg->staticCfg.numVirtualAntennas * sizeof(cmplx32ImRe_t);
        hwaParamCfg[paramsetIdx].source.srcScale = 0;//Fix this to 0 to achieve near bit-matching with MATLAB
    }
    
    hwaParamCfg[paramsetIdx].source.srcBcnt = cfg->staticCfg.numDopplerBins - 1; 
    hwaParamCfg[paramsetIdx].source.srcShift = 0; 
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0; 
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0; 

    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET); // address is relative to start of MEM0
    hwaParamCfg[paramsetIdx].dest.dstAcnt = 1 - 1; //Fixed for sum statistics
    hwaParamCfg[paramsetIdx].dest.dstAIdx = 8;//Fixed for sum statistics
    hwaParamCfg[paramsetIdx].dest.dstBIdx = 8;//Fixed for sum statistics
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //Fixed for sum statistics
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;//Fixed for sum statistics
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;
    hwaParamCfg[paramsetIdx].dest.dstScale = 8;//Fixed for sum statistics
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;// Enable magnitude operation only
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;//Put sum statistics output in HWA memory

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    /****** Ping paramset of Sum Abs of 2nd Range bin ******/ 
    ++paramsetIdx;
    sumAbsRngBin2Idx = paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[sumAbsRngBin1Idx];
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET)+cfg->staticCfg.numDopplerBins*cfg->staticCfg.numVirtualAntennas*hwaParamCfg[paramsetIdx].source.srcAIdx; // MEM BANK M1 + 0xC00
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET)+cfg->staticCfg.numDopplerBins*sizeof(cmplx32ImRe_t); // MEM BANK M0 + 0x300 
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    if (!cfg->staticCfg.isDetMatrixLogScale)
    {
        retVal = HWA_getDMAChanIndex(dopplerProcObj->hwaHandle,
                                     cfg->hwRes.edmaCfg.edmaOutDetectionMatrix.ping.channel,
                                     &destChan);
        if (retVal != 0)
        {
            goto exit;
        }
        /* enable the DMA hookup to this paramset so that data gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = destChan;

        errCode = HWA_enableParamSetInterrupt(dopplerProcObj->hwaHandle, cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }

    if (cfg->staticCfg.isDetMatrixLogScale)
    {
        /****** Ping paramset log2 magnitude ******/
        ++paramsetIdx;
        log2MagIdx = paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));

        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
        hwaParamCfg[paramsetIdx].dmaTriggerSrc = 0;
        hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;

        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
        hwaParamCfg[paramsetIdx].source.srcAcnt = 2*(cfg->staticCfg.numDopplerChirps) - 1;
        hwaParamCfg[paramsetIdx].source.srcBcnt = 1 - 1;
        hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(DPU_DopplerProcHWA_HwaSumOutput);
        hwaParamCfg[paramsetIdx].source.srcBIdx = 0;
        hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        hwaParamCfg[paramsetIdx].source.srcScale = 0;
        hwaParamCfg[paramsetIdx].source.srcShift = 0;
        hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0;
        hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET); // MEM BANK M0
        hwaParamCfg[paramsetIdx].dest.dstAcnt = 2*(cfg->staticCfg.numDopplerChirps) - 1;
        hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL; //complex data
        hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint16_t);
        hwaParamCfg[paramsetIdx].dest.dstBIdx = 0;
        hwaParamCfg[paramsetIdx].dest.dstScale = 0;

        hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
        hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;

        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

        retVal = HWA_configParamSet(dopplerProcObj->hwaHandle,
                                    cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle,
                                              cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = HWA_getDMAChanIndex(dopplerProcObj->hwaHandle,
                                     cfg->hwRes.edmaCfg.edmaOutDetectionMatrix.ping.channel,
                                     &destChan);
        if (retVal != 0)
        {
            goto exit;
        }
        /* enable the DMA hookup to this paramset so that data gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = destChan;

        errCode = HWA_enableParamSetInterrupt(dopplerProcObj->hwaHandle, cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }
    
    /****** Pong paramset of Doppler FFT for 1st Range bin (EDMA In) ******/ 
    if (cfg->staticCfg.isCompressionEnabled)
    {
        ++paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
        hwaParamCfg[paramsetIdx] = hwaParamCfg[decompressIdx];
        hwaParamCfg[paramsetIdx].dmaTriggerSrc = dopplerProcObj->hwaDmaTriggerSourcePong;
        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET);// MEM BANK M3
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET);// MEM BANK M2
        retVal = HWA_configParamSet(dopplerProcObj->hwaHandle,
                                    cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle,
                                              cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }

    }
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[dopplerFftRngBin1Idx];
    if (!cfg->staticCfg.isCompressionEnabled)
    {
        hwaParamCfg[paramsetIdx].dmaTriggerSrc = dopplerProcObj->hwaDmaTriggerSourcePong;
    }
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET);// MEM BANK M2
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET);// MEM BANK M3
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    /****** Pong paramset of Doppler FFT for 2nd Range bin ******/ 
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[dopplerFftRngBin2Idx];
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET)+0x4;// MEM BANK M2 + 0x4
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET)+cfg->staticCfg.numDopplerBins*cfg->staticCfg.numVirtualAntennas*hwaParamCfg[paramsetIdx].dest.dstBIdx;// MEM BANK M3 + 0xC00
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    /****** Pong paramset of Sum Abs of 1st Range bin ******/ 
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[sumAbsRngBin1Idx];
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET); // MEM BANK M3
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET); // MEM BANK M2 
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    /****** Pong paramset of Sum Abs of 2nd Range bin ******/ 
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[sumAbsRngBin2Idx];
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET)+cfg->staticCfg.numDopplerBins*cfg->staticCfg.numVirtualAntennas*hwaParamCfg[paramsetIdx].source.srcAIdx; // MEM BANK M3 + 0xC00
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET) + cfg->staticCfg.numDopplerBins*sizeof(cmplx32ImRe_t); // MEM BANK M2 + 0x300 
    retVal = HWA_configParamSet(dopplerProcObj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    if (!cfg->staticCfg.isDetMatrixLogScale)
    {
        retVal = HWA_getDMAChanIndex(dopplerProcObj->hwaHandle,
                                     cfg->hwRes.edmaCfg.edmaOutDetectionMatrix.pong.channel,
                                     &destChan);
        if (retVal != 0)
        {
            goto exit;
        }
        /* enable the DMA hookup to this paramset so that data gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = destChan;

        errCode = HWA_enableParamSetInterrupt(dopplerProcObj->hwaHandle, cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }

    if (cfg->staticCfg.isDetMatrixLogScale)
    {
        /****** Pong paramset log2 magnitude ******/
        ++paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
        hwaParamCfg[paramsetIdx] = hwaParamCfg[log2MagIdx];


        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET);
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET);

        retVal = HWA_configParamSet(dopplerProcObj->hwaHandle,
                                    cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(dopplerProcObj->hwaHandle,
                                              cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                              HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = HWA_getDMAChanIndex(dopplerProcObj->hwaHandle,
                                     cfg->hwRes.edmaCfg.edmaOutDetectionMatrix.pong.channel,
                                     &destChan);
        if (retVal != 0)
        {
            goto exit;
        }
        /* enable the DMA hookup to this paramset so that data gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = destChan;

        errCode = HWA_enableParamSetInterrupt(dopplerProcObj->hwaHandle, cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }

    dopplerProcObj->hwaParamStopIdx = cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx;
exit:
    return(errCode);
 }

/*===========================================================
 *                    Doppler Proc External APIs
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
 *  \ingroup    DPU_DOPPLERPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
DPU_DopplerProcHWA_Handle DPU_DopplerProcHWA_init
(
    DPU_DopplerProcHWA_InitParams *initCfg,
    int32_t                    *errCode
)
{   
    dopplerProcHWAObj  *dopplerProcObj = NULL;
    //SemaphoreP_Params       semParams;
    HWA_MemInfo             hwaMemInfo;
    uint32_t                i;
    int32_t             status = SystemP_SUCCESS;

    *errCode       = 0;
    
    if((initCfg == NULL) || (initCfg->hwaHandle == NULL))
    {
        *errCode = DPU_DOPPLERPROCHWA_EINVAL;
        goto exit;
    }    

    /* Allocate Memory for DopplerProc */
    dopplerProcObj = (dopplerProcHWAObj*)&gDopplerProcHeapMem;
    if(dopplerProcObj == NULL)
    {
        *errCode = DPU_DOPPLERPROCHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)dopplerProcObj, 0U, sizeof(dopplerProcHWAObj));
    
    /* Save init config params */
    dopplerProcObj->hwaHandle   = initCfg->hwaHandle;

    /* Populate HWA base addresses and offsets. This is done only once, at init time.*/
    *errCode =  HWA_getHWAMemInfo(dopplerProcObj->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {       
        goto exit;
    }

    /* check if we have enough memory banks*/
    if(hwaMemInfo.numBanks < DPU_DOPPLERPROCHWA_NUM_HWA_MEMBANKS)
    {    
        *errCode = DPU_DOPPLERPROCHWA_EHWARES;
        goto exit;
    }
    
    for (i = 0; i < DPU_DOPPLERPROCHWA_NUM_HWA_MEMBANKS; i++)
    {
        dopplerProcObj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress + i * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    status = SemaphoreP_constructBinary(&dopplerProcObj->edmaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_DOPPLERPROCHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done */
    status = SemaphoreP_constructBinary(&dopplerProcObj->hwaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_DOPPLERPROCHWA_ESEMA;
        goto exit;
    }

exit:    

    if(*errCode < 0)
    {
        dopplerProcObj = (DPU_DopplerProcHWA_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((DPU_DopplerProcHWA_Handle)dopplerProcObj);
}

/**
  *  @b Description
  *  @n
  *   Doppler DPU configuration 
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    DPU_DOPPLERPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_DopplerProcHWA_config
(
    DPU_DopplerProcHWA_Handle    handle,
    DPU_DopplerProcHWA_Config    *cfg
)
{
    dopplerProcHWAObj   *dopplerProcObj;
    int32_t                  retVal = 0;
    uint16_t                 expectedWinSamples;
    uint32_t                 sizeElementInBytes;

    dopplerProcObj = (dopplerProcHWAObj *)handle;
    if(dopplerProcObj == NULL)
    {
        retVal = DPU_DOPPLERPROCHWA_EINVAL;
        goto exit;
    }
    
#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle ||
       !cfg->hwRes.hwaCfg.window || 
       !cfg->hwRes.radar_1D_FFT_Cube.data ||
       !cfg->hwRes.detMatrix.data
      )
    {
        retVal = DPU_DOPPLERPROCHWA_EINVAL;
        goto exit;
    }

    /* Check if radar cube format is supported by DPU*/
    if(cfg->hwRes.radar_1D_FFT_Cube.datafmt != DPIF_RADARCUBE_FORMAT_6)
    {
        retVal = DPU_DOPPLERPROCHWA_ECUBEFORMAT;
        goto exit;
    }

    /* Check if detection matrix format is supported by DPU*/
    if(cfg->hwRes.detMatrix.datafmt != DPIF_DETMATRIX_FORMAT_1)
    {
        retVal = DPU_DOPPLERPROCHWA_EDETMFORMAT;
        goto exit;
    }
    
    /* Check if radar cube column fits into one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_DOPPLERPROCHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if abs value of log2 of 2D FFT fits in one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerBins * sizeof(uint16_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_DOPPLERPROCHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if number of range bins is even*/
    if((cfg->staticCfg.numRangeBins % 2) != 0)
    {
        retVal = DPU_DOPPLERPROCHWA_EINVAL;
        goto exit;
    }
    
    if (cfg->staticCfg.isDetMatrixLogScale)
    {
        sizeElementInBytes = sizeof(uint16_t);
    }
    else
    {
        sizeElementInBytes = sizeof(uint32_t);
    }
    /* Check if detection matrix size is sufficient*/
    if(cfg->hwRes.detMatrix.dataSize < (cfg->staticCfg.numRangeBins *
                                        cfg->staticCfg.numDopplerBins * sizeElementInBytes ))
    {
        retVal = DPU_DOPPLERPROCHWA_EDETMSIZE;
        goto exit;
    }

    /* Check window Size */
    if(cfg->hwRes.hwaCfg.winSym == HWA_FFT_WINDOW_NONSYMMETRIC)
    {
        expectedWinSamples = cfg->staticCfg.numDopplerChirps;
    }
    else
    {
        /* odd samples have to be rounded up per HWA */
        expectedWinSamples = (cfg->staticCfg.numDopplerChirps + 1) / 2;
    }

    if (cfg->hwRes.hwaCfg.windowSize != expectedWinSamples * sizeof(int32_t)) 
    {
            retVal = DPU_DOPPLERPROCHWA_EWINDSIZE;
            goto exit;
    }        
#endif

    /* Save necessary parameters to DPU object that will be used during Process time */
    /* EDMA parameters needed to trigger first EDMA transfer*/
    dopplerProcObj->edmaHandle  = cfg->hwRes.edmaCfg.edmaHandle;
    memcpy((void*)(&dopplerProcObj->edmaIn), (void *)(&cfg->hwRes.edmaCfg.edmaIn), sizeof(DPU_DopplerProc_Edma));
    
    /*HWA parameters needed for the 2D FFT HWA common configuration*/
    dopplerProcObj->hwaNumLoops      = cfg->staticCfg.numRangeBins / 4U; // As two range bins of data are taken per ping/pong
    dopplerProcObj->hwaParamStartIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;    
    
    /* Disable the HWA */
    retVal = HWA_enable(dopplerProcObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    /* HWA window configuration */
    retVal = HWA_configRam(dopplerProcObj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)cfg->hwRes.hwaCfg.window,
                           cfg->hwRes.hwaCfg.windowSize, //size in bytes
                           cfg->hwRes.hwaCfg.winRamOffset * sizeof(int32_t)); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    /*******************************/
    /**       Configure HWA       **/
    /*******************************/
    /*Compute source DMA channels that will be programmed in both HWA and EDMA.   
      The DMA channels are set to be equal to the paramSetIdx used by HWA*/
    /* Ping DMA channel (Ping uses the first paramset)*/  
    dopplerProcObj->hwaDmaTriggerSourcePing = cfg->hwRes.hwaCfg.dmaTrigSrcPingChan;
    /* Pong DMA channel (multiplication by 2 to compensate for range bins*/
    dopplerProcObj->hwaDmaTriggerSourcePong = cfg->hwRes.hwaCfg.dmaTrigSrcPongChan;
    retVal = dopplerProcHWA_ConfigHWA(dopplerProcObj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }
                    
    /*********************************/
    /**        Configure EDMA      **/
    /*******************************/    
    retVal = dopplerProcHWA_configEdma(dopplerProcObj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}

/**
  *  @b Description
  *  @n Doppler DPU process function. 
  *   
  *  @param[in]   handle     DPU handle.
  *  @param[out]  outParams  Output parameters.
  *
  *  \ingroup    DPU_DOPPLERPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success     =0
  *  @retval
  *      Error      !=0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_DopplerProcHWA_process
(
    DPU_DopplerProcHWA_Handle    handle,
    DPU_DopplerProcHWA_OutParams *outParams
)
{
    //volatile uint32_t   startTime;
    dopplerProcHWAObj   *dopplerProcObj;
    int32_t             retVal = 0;
    HWA_CommonConfig    hwaCommonConfig;
    uint32_t            baseAddr, regionId;

    dopplerProcObj = (dopplerProcHWAObj *)handle;
    if (dopplerProcObj == NULL)
    {
        retVal = DPU_DOPPLERPROCHWA_EINVAL;
        dopplerProcObj->inProgress = false;
        goto exit;
    }
    /* Set inProgress state */
    dopplerProcObj->inProgress = true;

    baseAddr = EDMA_getBaseAddr(dopplerProcObj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(dopplerProcObj->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);
    
    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(dopplerProcObj->hwaHandle,
                                    dopplerProcHWADoneIsrCallback,
                                    (void*)&dopplerProcObj->hwaDoneSemaHandle);
    if (retVal != 0)
    {
        goto exit;
    }
    
    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
        HWA_COMMONCONFIG_MASK_FFTSUMDIV;

    hwaCommonConfig.numLoops      = dopplerProcObj->hwaNumLoops;     
    hwaCommonConfig.paramStartIdx = dopplerProcObj->hwaParamStartIdx;
    hwaCommonConfig.paramStopIdx  = dopplerProcObj->hwaParamStopIdx; 
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    hwaCommonConfig.fftConfig.fftSumDiv = 0;
    
    retVal = HWA_configCommon(dopplerProcObj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(dopplerProcObj->hwaHandle,1); 
    if (retVal != 0)
    {
        goto exit;
    }

    EDMAEnableTransferRegion(baseAddr, regionId, dopplerProcObj->edmaIn.ping.channel, EDMA_TRIG_MODE_MANUAL);
    EDMAEnableTransferRegion(baseAddr, regionId, dopplerProcObj->edmaIn.pong.channel, EDMA_TRIG_MODE_MANUAL);
    
    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    SemaphoreP_pend(&dopplerProcObj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

    HWA_disableDoneInterrupt(dopplerProcObj->hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(dopplerProcObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************/
    /* WAIT FOR EDMA DONE INTERRUPT               */
    /**********************************************/
    SemaphoreP_pend(&dopplerProcObj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

exit:
    return retVal;
}

/**
  *  @b Description
  *  @n
  *  Doppler DPU deinit 
  *
  *  @param[in]   handle   DPU handle.
  *
  *  \ingroup    DPU_DOPPLERPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_DOPPLERPROC_ERROR_CODE
  */
int32_t DPU_DopplerProcHWA_deinit(DPU_DopplerProcHWA_Handle handle)
{
    dopplerProcHWAObj     *dopplerProcObj;
    int32_t             retVal = 0;

    /* Sanity Check */
    dopplerProcObj = (dopplerProcHWAObj *)handle;
    if(dopplerProcObj == NULL)
    {
        retVal = DPU_DOPPLERPROCHWA_EINVAL;
        goto exit;
    }

    /* Delete Semaphores */
    SemaphoreP_destruct(&dopplerProcObj->edmaDoneSemaHandle);
    SemaphoreP_destruct(&dopplerProcObj->hwaDoneSemaHandle);

exit:

    return (retVal);
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
int32_t DPU_DopplerProcHWA_GetNumUsedHwaParamSets
(
    DPU_DopplerProcHWA_Handle    handle,
    uint8_t *numUsedHwaParamSets
)
{
    dopplerProcHWAObj *obj;
    int32_t retVal = 0;

    obj = (dopplerProcHWAObj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_DOPPLERPROCHWA_EINVAL;
        goto exit;
    }
    *numUsedHwaParamSets = (uint8_t) obj->hwaParamStopIdx - obj->hwaParamStartIdx + 1;
exit:
    return retVal;
}