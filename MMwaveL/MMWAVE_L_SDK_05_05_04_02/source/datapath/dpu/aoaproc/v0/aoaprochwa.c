/**
 *   @file  aoaprochwa.c
 *
 *   @brief
 *      Implements Data path AoA processing Unit using HWA.
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
#include <drivers/hwa.h>
#include <drivers/soc.h>

/* Data Path Include files */
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <datapath/dpu/aoaproc/v0/aoaprochwa.h>
#include <datapath/dpu/aoaproc/v0/aoaproc_internal.h>

/* Utils */
#include <utils/mathutils/mathutils.h>

/* User defined heap memory and handle */
#define AOAPROCHWA_HEAP_MEM_SIZE  (sizeof(aoaProcHWAObj))

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

#define DEBUG_SINGLE_STEP_HWA 0

#define AOA_AZIMUTH_HWA_PARAMSET_RELATIVE_IDX                    0
#define AOA_ELEVATION_HWA_PARAMSET_RELATIVE_IDX                  1
#define AOA_MAX_ELEVATION_AZIMUTH_HWA_PARAMSET_RELATIVE_IDX      2

#define DOPPLER_FFT_HWA_PARAMSET_RELATIVE_IDX                    0
#define DOPPLER_COMPENSATION_TX1_HWA_PARAMSET_RELATIVE_IDX       1
#define DOPPLER_COMPENSATION_TX2_HWA_PARAMSET_RELATIVE_IDX       2

#define ANT_2TX_3RX 0 //set to 1 here for NEARRANGE,LONGRANGE and LOWPOWER DEMO
#define ANT_2TX_2RX 1 //set to 1 here for K2O DEMO
#define ANT_1TX_2RX 0
#define ANT_1TX_2RX_ELEV 0
#define ANT_2TX_2RX_AZIM 0
#define LINEAR_ARRAY 0 //This needs to be set to 1 if linear array (along elevation/azimuth) is to be exercised
#define ANT_AOP_2TX_3RX 0

/* HWA ping/pong buffers offset */
#define DPU_DOPPLERPROCHWA_SRC_PING_OFFSET   aoaProcObj->hwaMemBankAddr[0]
#define DPU_AOAPROCHWA_SRC_PING_OFFSET       DPU_DOPPLERPROCHWA_SRC_PING_OFFSET
#define DPU_DOPPLERPROCHWA_DST_PING_OFFSET   aoaProcObj->hwaMemBankAddr[1]
#define DPU_AOAPROCHWA_DST_PING_OFFSET       DPU_DOPPLERPROCHWA_DST_PING_OFFSET
#define DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET   aoaProcObj->hwaMemBankAddr[2]
#define DPU_AOAPROCHWA_SRC_PONG_OFFSET       DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET
#define DPU_DOPPLERPROCHWA_DST_PONG_OFFSET   aoaProcObj->hwaMemBankAddr[3]
#define DPU_AOAPROCHWA_DST_PONG_OFFSET       DPU_DOPPLERPROCHWA_DST_PONG_OFFSET

// Heap memory allocation
static uint8_t gAoAProcHeapMem[AOAPROCHWA_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

uint8_t num_points_count; // num_points_count variable to place the data in angleMatrix for AoA processing

void copyData_after_2DFFT(aoaProcHWAObj *aoaProcObj, int16_t loop_count)
{   
    uint8_t curr_range_bin, curr_doppler_index_count, bit_width_length, i, curr_doppler_index;
    uint32_t hwa_op_addr, jump_address;

    curr_doppler_index_count = aoaProcObj->angle_idx_doppler_count_arr[loop_count];

    if(loop_count==0)
    {
        num_points_count = 0;
    }
    else
    {
        num_points_count += aoaProcObj->angle_idx_doppler_count_arr[loop_count-1];
    }

    //Based on ping/pong memory path to be serviced and if Doppler compensation is to be applied or not (based on if both Tx is in use or not)
    if(aoaProcObj->isDopplerPhaseCompensationEnabled[0])
    {
        if(loop_count%2==0)
        {   
            if(aoaProcObj->numTxAntennas>1)
                hwa_op_addr = CSL_APP_HWA_DMA0_RAM_BANK0_BASE;
            else
                hwa_op_addr = CSL_APP_HWA_DMA0_RAM_BANK1_BASE;
        }
        else
        {   
            if(aoaProcObj->numTxAntennas>1)
                hwa_op_addr = CSL_APP_HWA_DMA0_RAM_BANK2_BASE;
            else
                hwa_op_addr = CSL_APP_HWA_DMA0_RAM_BANK3_BASE;
        }
    }
    else
    {
        if(loop_count%2==0)
        {   
            hwa_op_addr = CSL_APP_HWA_DMA0_RAM_BANK1_BASE;
        }
        else
        {   
            hwa_op_addr = CSL_APP_HWA_DMA0_RAM_BANK3_BASE;
        }
    }

    curr_range_bin = aoaProcObj->range_idx_arr[loop_count];// stored as 16bit though 8-bit is sufficient for storage
    
    if(aoaProcObj->doppFFT_is16b)
    {   
        cmplx16ImRe_t *antenna_array = (cmplx16ImRe_t*)aoaProcObj->antenna_array;
        cmplx16ImRe_t *angleMatrixbase = (cmplx16ImRe_t*)aoaProcObj->angleMatBuf;
        bit_width_length = sizeof(cmplx16ImRe_t);
        for(i=0;i<curr_doppler_index_count;++i)
        {   
            curr_doppler_index = aoaProcObj->doppler_idx_arr[curr_range_bin*aoaProcObj->max_num_points+i];

            // jump address - based on which, antenna elements corresponding to range and doppler index is picked up 
            jump_address = hwa_op_addr + curr_doppler_index*aoaProcObj->numVirtualAntennas*bit_width_length;

            cmplx16ImRe_t *actual_address = (cmplx16ImRe_t *)jump_address;
            #if ANT_2TX_3RX
            // Based on Antenna array pattern of xWRx432 - virtual antenna array pattern below
            /* 
               0 2 5 0
               1 4 3 6                      
            */
            antenna_array[0].imag = 0;
            antenna_array[0].real = 0;
            antenna_array[1].imag = -actual_address[1].imag;
            antenna_array[1].real = -actual_address[1].real;
            antenna_array[2] = actual_address[4];
            antenna_array[3].imag = 0;
            antenna_array[3].real = 0;
            antenna_array[4] = actual_address[0];
            antenna_array[5].imag = -actual_address[3].imag;
            antenna_array[5].real = -actual_address[3].real;
            antenna_array[6] = actual_address[2];
            antenna_array[7].imag = -actual_address[5].imag;
            antenna_array[7].real = -actual_address[5].real;
            #elif ANT_2TX_2RX
            // Based on Antenna array pattern of xWRx422 - virtual antenna array pattern below
            /* 
               0 2 4
               1 3 0                      
            */
            antenna_array[0].imag = 0;
            antenna_array[0].real = 0;
            antenna_array[1].imag = -actual_address[1].imag;
            antenna_array[1].real = -actual_address[1].real;
            antenna_array[2] = actual_address[3];
            antenna_array[3] = actual_address[0];
            antenna_array[4].imag = -actual_address[2].imag;
            antenna_array[4].real = -actual_address[2].real;
            antenna_array[5].imag = 0;
            antenna_array[5].real = 0;
            #elif ANT_1TX_2RX
            // Based on Antenna array pattern of xWRx422 - virtual antenna array pattern below
            /* 
               0 2 
               1 0                      
            */
            antenna_array[0].imag = 0;
            antenna_array[0].real = 0;
            antenna_array[1].imag = -actual_address[1].imag;
            antenna_array[1].real = -actual_address[1].real;
            antenna_array[2] = actual_address[0];
            antenna_array[3].imag = 0;
            antenna_array[3].real = 0;
            #elif ANT_1TX_2RX_ELEV
            // Based on simulated Antenna array pattern of xWRx421 - virtual antenna array pattern below
            /* 
               2  
               4                      
            */
            antenna_array[0].imag = -actual_address[1].imag;
            antenna_array[0].real = -actual_address[1].real;
            antenna_array[1].imag = -actual_address[3].imag;;
            antenna_array[1].real = -actual_address[3].real;
            #elif ANT_2TX_2RX_AZIM
            // Based on linear array pattern of xwRx432 - virtual antenna array pattern below
            /* 
                1 -4 3 -6                    
            */
            antenna_array[0] = actual_address[0];
            antenna_array[1].imag = -actual_address[3].imag;
            antenna_array[1].real = -actual_address[3].real;
            antenna_array[2] = actual_address[2];
            antenna_array[3].imag = -actual_address[5].imag;
            antenna_array[3].real = -actual_address[5].real;
            #elif ANT_AOP_2TX_3RX
            // Based on Antenna array pattern of xWRx432 AOP - virtual antenna array pattern below
            /* 
               2 1 5 4
               0 3 0 6                      
            */
            antenna_array[0].imag = actual_address[1].imag;
            antenna_array[0].real = actual_address[1].real;
            antenna_array[1] = actual_address[0];
            antenna_array[2] = actual_address[4];
            antenna_array[3].imag = actual_address[3].imag;
            antenna_array[3].real = actual_address[3].real;
            antenna_array[4].imag = 0;
            antenna_array[4].real = 0;
            antenna_array[5] = actual_address[2];
            antenna_array[6].imag = 0;
            antenna_array[6].real = 0;
            antenna_array[7].imag = actual_address[5].imag;
            antenna_array[7].real = actual_address[5].real;
            #endif

            //Copy out this data to HWA_SS RAM for further Azimuth-Elevation FFT
            memcpy((void *)&angleMatrixbase[(num_points_count+i)*aoaProcObj->antenna_array_elements], (void *)&antenna_array[0], aoaProcObj->antenna_array_elements*bit_width_length);
        }
    }
    else
    {   
        cmplx32ImRe_t *antenna_array = (cmplx32ImRe_t*)aoaProcObj->antenna_array;
        cmplx32ImRe_t *angleMatrixbase = (cmplx32ImRe_t *)aoaProcObj->angleMatBuf;
        bit_width_length = sizeof(cmplx32ImRe_t);
        for(i=0;i<curr_doppler_index_count;++i)
        {   
            curr_doppler_index = aoaProcObj->doppler_idx_arr[curr_range_bin*aoaProcObj->max_num_points+i];

            // jump address - based on which, antenna elements corresponding to range and doppler index is picked up 
            jump_address = hwa_op_addr + curr_doppler_index*aoaProcObj->numVirtualAntennas*bit_width_length;

            cmplx32ImRe_t *actual_address = (cmplx32ImRe_t *)jump_address;
            #if ANT_2TX_3RX
            // Based on Antenna array pattern of xWRx432 - virtual antenna array pattern below
            /* 
               0 2 5 0
               1 4 3 6                      
            */
            antenna_array[0].imag = 0;
            antenna_array[0].real = 0;
            antenna_array[1].imag = -actual_address[1].imag;
            antenna_array[1].real = -actual_address[1].real;
            antenna_array[2] = actual_address[4];
            antenna_array[3].imag = 0;
            antenna_array[3].real = 0;
            antenna_array[4] = actual_address[0];
            antenna_array[5].imag = -actual_address[3].imag;
            antenna_array[5].real = -actual_address[3].real;
            antenna_array[6] = actual_address[2];
            antenna_array[7].imag = -actual_address[5].imag;
            antenna_array[7].real = -actual_address[5].real;
            #elif ANT_2TX_2RX
            // Based on Antenna array pattern of xWRx422 - virtual antenna array pattern below
            /* 
               0 2 4
               1 3 0                      
            */
            antenna_array[0].imag = 0;
            antenna_array[0].real = 0;
            antenna_array[1].imag = -actual_address[1].imag;
            antenna_array[1].real = -actual_address[1].real;
            antenna_array[2] = actual_address[3];
            antenna_array[3] = actual_address[0];
            antenna_array[4].imag = -actual_address[2].imag;
            antenna_array[4].real = -actual_address[2].real;
            antenna_array[5].imag = 0;
            antenna_array[5].real = 0;
            #elif ANT_1TX_2RX
            // Based on Antenna array pattern of xWRx422 - virtual antenna array pattern below
            /* 
               0 2 
               1 0                      
            */
            antenna_array[0].imag = 0;
            antenna_array[0].real = 0;
            antenna_array[1].imag = -actual_address[1].imag;
            antenna_array[1].real = -actual_address[1].real;
            antenna_array[2] = actual_address[0];
            antenna_array[3].imag = 0;
            antenna_array[3].real = 0;
            #elif ANT_1TX_2RX_ELEV
            // Based on simulated Antenna array pattern of xWRx421 - virtual antenna array pattern below
            /* 
               2  
               4                      
            */
            antenna_array[0].imag = -actual_address[1].imag;
            antenna_array[0].real = -actual_address[1].real;
            antenna_array[1].imag = -actual_address[2].imag;;
            antenna_array[1].real = -actual_address[2].real;
            #elif ANT_2TX_2RX_AZIM
            // Based on linear array pattern of xwRx432 - virtual antenna array pattern below
            /* 
                1 -4 3 -6                    
            */
            antenna_array[0] = actual_address[0];
            antenna_array[1].imag = -actual_address[3].imag;
            antenna_array[1].real = -actual_address[3].real;
            antenna_array[2] = actual_address[2];
            antenna_array[3].imag = -actual_address[5].imag;
            antenna_array[3].real = -actual_address[5].real;
            #elif ANT_AOP_2TX_3RX
            // Based on Antenna array pattern of xWRx432 AOP - virtual antenna array pattern below
            /* 
               2 1 5 4
               0 3 0 6                      
            */
            antenna_array[0].imag = actual_address[1].imag;
            antenna_array[0].real = actual_address[1].real;
            antenna_array[1] = actual_address[0];
            antenna_array[2] = actual_address[4];
            antenna_array[3].imag = actual_address[3].imag;
            antenna_array[3].real = actual_address[3].real;
            antenna_array[4].imag = 0;
            antenna_array[4].real = 0;
            antenna_array[5] = actual_address[2];
            antenna_array[6].imag = 0;
            antenna_array[6].real = 0;
            antenna_array[7].imag = actual_address[5].imag;
            antenna_array[7].real = actual_address[5].real;
            #endif

            //Copy out this data to HWA_SS RAM for further Azimuth-Elevation FFT
            memcpy((void *)&angleMatrixbase[(num_points_count+i)*aoaProcObj->antenna_array_elements], (void *)&antenna_array[0], aoaProcObj->antenna_array_elements*bit_width_length);
        }
    }
}

/*===========================================================
 *                    Internal Functions
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function of Doppler.
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 */
void aoaProcHWADoneIsrCallback_Doppler(void *arg)
{   
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function of AoA.
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 */
void aoaProcHWADoneIsrCallback_AoA(void *arg)
{   
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}


/**
 *  @b Description
 *  @n
 *      EDMA completion call back function of EDMA out of max index of AoA.
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 */
void aoaProcHWA_edmaDoneIsrCallback(Edma_IntrHandle intrHandle, void *arg)
{   
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      Configures HWA ISR callback to copy out the data before AoA.
 *
 *  @param[in] paramset            - DPU obj
 *  @param[in] cfg (arg)           - DPU configuration
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
void HWA_ParamSetISR_Callback(uint32_t paramSet, void *arg)
{   
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *  AoA DPU Doppler EDMA configuration.
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
static int32_t aoaProcHWA_configEdma_Doppler
(
    aoaProcHWAObj      *aoaProcObj,
    DPU_AoAProcHWA_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    cmplx16ImRe_t       *radarCubeBase = (cmplx16ImRe_t *)cfg->hwRes.radar_1D_FFT_Cube.data;
    int16_t             sampleLenInBytes = sizeof(cmplx16ImRe_t);
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;

    if(aoaProcObj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    
    /*********************************PROGRAM DMA'S FOR PING**************************************/

    /*****************************************************************************************
    *PROGRAM DMA channel  to transfer data from Radar cube to accelerator input buffer (ping)*
    ******************************************************************************************/    
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSigDoppler.ping.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[cfg->maxDetmatrix_res.range_idx_arr[0]]);
    syncABCfg.destAddress = (uint32_t)(aoaProcObj->hwaMemBankAddr[0]);
    syncABCfg.aCount      = sampleLenInBytes;
    syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numDopplerChirps *  cfg->staticCfg.numTxAntennas;
    syncABCfg.cCount      = (cfg->maxDetmatrix_res.rangeGatesCount[0]/2) + (cfg->maxDetmatrix_res.rangeGatesCount[0]%2);//factor of 2 due to ping/pong
    syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sampleLenInBytes;
    syncABCfg.dstBIdx     = sampleLenInBytes;
    syncABCfg.srcCIdx     = 4*sampleLenInBytes;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn_rangeFFT.ping,
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
    *  PROGRAM DMA channel to trigger HWA for processing of input (ping)
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSigDoppler.ping,
                                             aoaProcObj->hwaHandle,
                                             aoaProcObj->hwaDmaTriggerDoppPing,
                                             false);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /******************************PROGRAM DMA'S FOR PONG***********************************/

    /******************************************************************************************
     *  PROGRAM DMA channel  to transfer data from HWA_SS to accelerator input buffer (pong)
     ******************************************************************************************/ 
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSigDoppler.pong.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[cfg->maxDetmatrix_res.range_idx_arr[1]]);
    syncABCfg.destAddress = (uint32_t)(aoaProcObj->hwaMemBankAddr[2]);
    syncABCfg.aCount      = sampleLenInBytes;
    syncABCfg.bCount      = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numDopplerChirps *  cfg->staticCfg.numTxAntennas;
    syncABCfg.cCount      = (cfg->maxDetmatrix_res.rangeGatesCount[0]/2);//factor of 2 due to ping/pong
    syncABCfg.srcBIdx     = cfg->staticCfg.numRangeBins * sampleLenInBytes;
    syncABCfg.dstBIdx     = sampleLenInBytes;
    syncABCfg.srcCIdx     = 4*sampleLenInBytes;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn_rangeFFT.pong,
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
    *  PROGRAM DMA channel to trigger HWA for processing of input (pong)
    ******************************************************************************************/  
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSigDoppler.pong,
                                             aoaProcObj->hwaHandle,
                                             aoaProcObj->hwaDmaTriggerDoppPong,
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
 *  AoA DPU Doppler EDMA configuration.
 *  This implementation of AoA processing involves Ping/Pong 
 * (including finding the maximum value in elevation-Azimuth heatmap)
 *  Mechanism, hence there are two sets of EDMA transfer.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOPPLERPROC_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static int32_t aoaProcHWA_configEdma_AoA
(
    aoaProcHWAObj      *aoaProcObj,
    DPU_AoAProcHWA_Config   *cfg
)
{
    int32_t             retVal = SystemP_SUCCESS;
    cmplx32ImRe_t       *angleMatrixbase = (cmplx32ImRe_t *)cfg->hwRes.angleMat.data;//TODO: generalize here
    int16_t             sampleLenInBytes;
    if(cfg->staticCfg.doppFFT_is16b)
        sampleLenInBytes = sizeof(cmplx16ImRe_t);
    else
        sampleLenInBytes = sizeof(cmplx32ImRe_t);
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;

    if(aoaProcObj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    
    /*********************************PROGRAM DMA'S FOR PING**************************************/

    /*******************************************************************************************
     *PROGRAM DMA channel to transfer data from Angle Matrix to accelerator input buffer (ping)*
    ********************************************************************************************/    
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSigAoA.ping.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)(&angleMatrixbase[0]);
    syncABCfg.destAddress = (uint32_t)(aoaProcObj->hwaMemBankAddr[0]);
    syncABCfg.aCount      = sampleLenInBytes*aoaProcObj->antenna_array_elements;
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = (aoaProcObj->max_num_points/2) + (aoaProcObj->max_num_points%2);//factor of 2 due to ping/pong
    syncABCfg.srcBIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.srcCIdx     = 2*sampleLenInBytes*aoaProcObj->antenna_array_elements;//factor of 2 due to ping/pong
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn_dopplerFFT.ping,
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
    *  PROGRAM DMA channel to trigger HWA for processing of input (ping)
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSigAoA.ping,
                                             aoaProcObj->hwaHandle,
                                             aoaProcObj->hwaDmaTriggerAoAPing,
                                             false);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /***************************************************************************
     *  PROGRAM DMA channel  to transfer max value from accelerator output
     *  buffer (ping) to M4 memory
     **************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn_dopplerFFT.ping.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = false;
#if(!LINEAR_ARRAY)
    syncABCfg.srcAddress  = (uint32_t)aoaProcObj->hwaMemBankAddr[1];
#else
    syncABCfg.srcAddress  = (uint32_t)aoaProcObj->hwaMemBankAddr[0];
#endif
    syncABCfg.destAddress = (uint32_t)SOC_virtToPhy(&cfg->hwRes.maxVal_elev_azim[0]);
    syncABCfg.aCount      = sizeof(uint32_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = (aoaProcObj->max_num_points/2) + (aoaProcObj->max_num_points%2);//factor of 2 due to ping/pong
    syncABCfg.srcBIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstCIdx     = 2*sizeof(uint32_t);    

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaOut_maxAoA.ping,
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

    /******************************PROGRAM DMA'S FOR PONG***********************************/

    /*******************************************************************************************
     *PROGRAM DMA channel to transfer data from Angle Matrix to accelerator input buffer (pong)*
    ********************************************************************************************/    
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSigAoA.pong.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    syncABCfg.srcAddress  = (uint32_t)(&angleMatrixbase[aoaProcObj->antenna_array_elements]);
    syncABCfg.destAddress = (uint32_t)(aoaProcObj->hwaMemBankAddr[2]);
    syncABCfg.aCount      = sampleLenInBytes*aoaProcObj->antenna_array_elements;
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = (aoaProcObj->max_num_points/2);//factor of 2 due to ping/pong
    syncABCfg.srcBIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.srcCIdx     = 2*sampleLenInBytes*aoaProcObj->antenna_array_elements;
    syncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn_dopplerFFT.pong,
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
    *  PROGRAM DMA channel to trigger HWA for processing of input (pong)
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSigAoA.pong,
                                             aoaProcObj->hwaHandle,
                                             aoaProcObj->hwaDmaTriggerAoAPong,
                                             false);

    if (retVal != SystemP_SUCCESS)
    {
        goto exit;
    }

    /***************************************************************************
     *  PROGRAM DMA channel  to transfer max value from accelerator output
     *  buffer (pong) to M4 memory
     **************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn_dopplerFFT.pong.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = false;
#if(!LINEAR_ARRAY)
    syncABCfg.srcAddress  = (uint32_t)(aoaProcObj->hwaMemBankAddr[3]);
#else
    syncABCfg.srcAddress  = (uint32_t)(aoaProcObj->hwaMemBankAddr[2]);
#endif
    syncABCfg.destAddress = (uint32_t)SOC_virtToPhy(&cfg->hwRes.maxVal_elev_azim[1]);
    syncABCfg.aCount      = sizeof(uint32_t);
    syncABCfg.bCount      = 1;
    syncABCfg.cCount      = (aoaProcObj->max_num_points/2)+(aoaProcObj->max_num_points%2);//factor of 2 due to ping/pong
    syncABCfg.srcBIdx     = 0;
    syncABCfg.dstBIdx     = 0;
    syncABCfg.srcCIdx     = 0;
    syncABCfg.dstCIdx     = 2*sizeof(uint32_t);

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaOut_maxAoA.pong,
                                &chainingCfg,
                                &syncABCfg,
                                true, //isEventTriggered
                                false,//isIntermediateTransferInterruptEnabled
                                true,//isTransferCompletionEnabled
                                aoaProcHWA_edmaDoneIsrCallback, //transferCompletionCallbackFxn
                                (void *)&aoaProcObj->edmaDoneSemaHandle,//transferCompletionCallbackFxnArg
                                cfg->hwRes.intrObj);

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
 *      Configures HWA for Doppler processing where before Angle FFT.
 *
 *  @param[in] aoaProcobj    - DPU obj
 *  @param[in] cfg           - DPU configuration
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
 
 static int32_t aoaProcHWA_ConfigHWA_Doppler
 (
    aoaProcHWAObj     *aoaProcObj,
    DPU_AoAProcHWA_Config    *cfg
 )
 {
    HWA_ParamConfig         hwaParamCfg[DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT];
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;
    uint8_t                 errCode=0;

    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    /****** Ping paramset of Doppler FFT for 1-Range bin (by EDMA in) ******/ 
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; 
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = aoaProcObj->hwaDmaTriggerDoppPing;
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
    hwaParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1; //size in samples - 1
    hwaParamCfg[paramsetIdx].source.srcAIdx = cfg->staticCfg.numVirtualAntennas * sizeof(cmplx16ImRe_t); 
    hwaParamCfg[paramsetIdx].source.srcBcnt = cfg->staticCfg.numVirtualAntennas - 1; 
    hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(cmplx16ImRe_t); 
    hwaParamCfg[paramsetIdx].source.srcShift = 0; 
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; 
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].source.srcScale = 8;
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0; 
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0; 

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
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.winRamOffset; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.winSym; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                          paramsetIdx + aoaProcObj->hwaParamStartIdxDopp,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
    
    // Paramset ISR enabling if numTx used is only 1 and there is no Doppler compensation needed
    if(cfg->staticCfg.numTxAntennas==1)
    {
        /* enable the CPU hookup to this paramset so that data for angle-FFT gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU;
        paramISRConfig.cpu.callbackFn = HWA_ParamSetISR_Callback;
        paramISRConfig.cpu.callbackArg = (void*)&(aoaProcObj->hwaSemaHandleParamset);
        errCode = HWA_enableParamSetInterrupt(aoaProcObj->hwaHandle, paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }

    // To do Doppler compensation only if num Tx antennas > 1
    if(cfg->staticCfg.numTxAntennas>1)
    {
        /****** Ping paramset of Doppler Compensation (for Tx1) ******/ 
        ++paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
        if(DEBUG_SINGLE_STEP_HWA)
            hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
        else
            hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; 
        hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;

        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET);
        hwaParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.numDopplerBins - 1; //size in samples - 1
        hwaParamCfg[paramsetIdx].source.srcBcnt = cfg->staticCfg.numVirtualAntennas/2 - 1; 
        if(cfg->staticCfg.doppFFT_is16b)
        {
            hwaParamCfg[paramsetIdx].source.srcAIdx = cfg->staticCfg.numVirtualAntennas * sizeof(cmplx16ImRe_t);
            hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(cmplx16ImRe_t);
            hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; 
            hwaParamCfg[paramsetIdx].source.srcScale = 8;
        }
        else
        {
            hwaParamCfg[paramsetIdx].source.srcAIdx = cfg->staticCfg.numVirtualAntennas * sizeof(cmplx32ImRe_t);
            hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(cmplx32ImRe_t);
            hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT; 
            hwaParamCfg[paramsetIdx].source.srcScale = 0;
        }
        hwaParamCfg[paramsetIdx].source.srcShift = 0; 
        hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
        hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data 
        hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 
        hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
        
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET);
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

        hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                    paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, 
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                            paramsetIdx + aoaProcObj->hwaParamStartIdxDopp,
                                            HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }

        /****** Ping paramset of Doppler Compensation (for Tx2) ******/ 
        ++paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
        hwaParamCfg[paramsetIdx] = hwaParamCfg[DOPPLER_COMPENSATION_TX1_HWA_PARAMSET_RELATIVE_IDX];
        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PING_OFFSET)+hwaParamCfg[paramsetIdx].source.srcBIdx*cfg->staticCfg.numRxAntennas;
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PING_OFFSET)+hwaParamCfg[paramsetIdx].source.srcBIdx*cfg->staticCfg.numRxAntennas;
        hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT;
        retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                    paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, 
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }
        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                             paramsetIdx + aoaProcObj->hwaParamStartIdxDopp,
                                            HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }

        /* enable the CPU hookup to this paramset so that data for angle-FFT gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU;
        paramISRConfig.cpu.callbackFn = HWA_ParamSetISR_Callback;
        paramISRConfig.cpu.callbackArg = (void*)&(aoaProcObj->hwaSemaHandleParamset);
        errCode = HWA_enableParamSetInterrupt(aoaProcObj->hwaHandle, paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }

    /****** Pong paramset of Doppler FFT for 1-Range bin (by EDMA in) ******/ 
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[DOPPLER_FFT_HWA_PARAMSET_RELATIVE_IDX]; 
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = aoaProcObj->hwaDmaTriggerDoppPong;
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET);
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET);
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                        paramsetIdx + aoaProcObj->hwaParamStartIdxDopp,
                                        HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }

    // Paramset ISR enabling if numTx used is only 1 and there is no Doppler compensation needed
    if(cfg->staticCfg.numTxAntennas==1)
    {
        /* enable the CPU hookup to this paramset so that data for angle-FFT gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU;
        paramISRConfig.cpu.callbackFn = HWA_ParamSetISR_Callback;
        paramISRConfig.cpu.callbackArg = (void*)&(aoaProcObj->hwaSemaHandleParamset);
        errCode = HWA_enableParamSetInterrupt(aoaProcObj->hwaHandle, paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }

    // To do Doppler compensation only if num Tx antennas > 1
    if(cfg->staticCfg.numTxAntennas>1)
    {
        /****** Pong paramset of Doppler Compensation (for Tx1) ******/ 
        ++paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
        hwaParamCfg[paramsetIdx] = hwaParamCfg[DOPPLER_COMPENSATION_TX1_HWA_PARAMSET_RELATIVE_IDX];
        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET);
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET);
        retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                    paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, 
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/
        retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                            paramsetIdx + aoaProcObj->hwaParamStartIdxDopp,
                                            HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }

        /****** Pong paramset of Doppler Compensation (for Tx2) ******/ 
        ++paramsetIdx;
        memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
        hwaParamCfg[paramsetIdx] = hwaParamCfg[DOPPLER_COMPENSATION_TX2_HWA_PARAMSET_RELATIVE_IDX];
        hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_DST_PONG_OFFSET)+hwaParamCfg[paramsetIdx].source.srcBIdx*cfg->staticCfg.numRxAntennas;
        hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCHWA_SRC_PONG_OFFSET)+hwaParamCfg[paramsetIdx].source.srcBIdx*cfg->staticCfg.numRxAntennas;
        retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                    paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, 
                                    &hwaParamCfg[paramsetIdx], NULL);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Make sure DMA interrupt/trigger is disabled for this paramset*/ 
        retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                            paramsetIdx + aoaProcObj->hwaParamStartIdxDopp,
                                            HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
            goto exit;
        }

        /* enable the CPU hookup to this paramset so that data for angle-FFT gets copied out */
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU;
        paramISRConfig.cpu.callbackFn = HWA_ParamSetISR_Callback;
        paramISRConfig.cpu.callbackArg = (void*)&(aoaProcObj->hwaSemaHandleParamset);
        errCode = HWA_enableParamSetInterrupt(aoaProcObj->hwaHandle, paramsetIdx + aoaProcObj->hwaParamStartIdxDopp, &paramISRConfig);
        if (errCode != 0)
        {
            goto exit;
        }
    }

exit:
    return(errCode);
 }

 /**
 *  @b Description
 *  @n
 *      Configures HWA for AoA processing where Azimuth-Elevation FFT is done.
 *
 *  @param[in] aoaProcobj    - DPU obj
 *  @param[in] cfg           - DPU configuration
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */
 
 static int32_t aoaProcHWA_ConfigHWA_AoA
 (
    aoaProcHWAObj     *aoaProcObj,
    DPU_AoAProcHWA_Config    *cfg
 )
 {
    HWA_ParamConfig         hwaParamCfg[DPU_AOAPROCHWA_NUM_HWA_PARAMSET_AOA];
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;
    int32_t                 retVal = 0U;
    uint8_t                 destChan;
    uint8_t                 errCode = 0;

    /****** Ping paramset of Azimuth FFT (by EDMA in) ******/ 
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; 
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = aoaProcObj->hwaDmaTriggerAoAPing; 
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
    hwaParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.numCols_Antenna-1; //Based on antenna array arrangement
    hwaParamCfg[paramsetIdx].source.srcBcnt = cfg->staticCfg.numRows_Antenna - 1;//Based on antenna array arrangement
    if(cfg->staticCfg.doppFFT_is16b)
    {
        hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(cmplx16ImRe_t);  
        hwaParamCfg[paramsetIdx].source.srcBIdx = cfg->staticCfg.numCols_Antenna*sizeof(cmplx16ImRe_t);
        hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
        hwaParamCfg[paramsetIdx].source.srcScale = 8;
    }
    else
    {
        hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(cmplx32ImRe_t);  
        hwaParamCfg[paramsetIdx].source.srcBIdx = cfg->staticCfg.numCols_Antenna*sizeof(cmplx32ImRe_t);
        hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        hwaParamCfg[paramsetIdx].source.srcScale = 0;
    }
    hwaParamCfg[paramsetIdx].source.srcShift = 0; 
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate

    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PING_OFFSET); // MEM BANK M1
    hwaParamCfg[paramsetIdx].dest.dstAcnt = cfg->staticCfg.azimuthFFTSize-1; //this is samples - 1
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg[paramsetIdx].dest.dstBIdx = cfg->staticCfg.azimuthFFTSize*sizeof(cmplx32ImRe_t);
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg[paramsetIdx].dest.dstScale = 8;
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumAzimuthBins;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                          paramsetIdx + aoaProcObj->hwaParamStartIdxAoA,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
#if(!LINEAR_ARRAY)
    /****** Ping paramset of Elevation FFT ******/ 
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    if(DEBUG_SINGLE_STEP_HWA)
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    else
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;  
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 

    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PING_OFFSET); // MEM BANK M1
    hwaParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.numRows_Antenna-1; //Based on antenna array arrangement
    hwaParamCfg[paramsetIdx].source.srcBcnt = cfg->staticCfg.azimuthFFTSize - 1;//Based on azimuth FFT calculated
    hwaParamCfg[paramsetIdx].source.srcAIdx = cfg->staticCfg.azimuthFFTSize*sizeof(cmplx32ImRe_t);
    hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg[paramsetIdx].source.srcScale = 0;// Fixed to bitmatch with MATLAB model
    hwaParamCfg[paramsetIdx].source.srcShift = 0; 
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 

    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
    hwaParamCfg[paramsetIdx].dest.dstAcnt = cfg->staticCfg.elevationFFTSize-1; //this is samples - 1
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].dest.dstAIdx = cfg->staticCfg.azimuthFFTSize*sizeof(cmplx32ImRe_t);
    hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(cmplx32ImRe_t);
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg[paramsetIdx].dest.dstScale = 8;
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumElevationBins;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                          paramsetIdx + aoaProcObj->hwaParamStartIdxAoA,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
#endif
    /****** Ping paramset of Abs-Max of Elevation-Azimuth heatmap ******/ 
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    if(DEBUG_SINGLE_STEP_HWA)
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_SOFTWARE;
    else
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; 
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 
#if(LINEAR_ARRAY)
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PING_OFFSET); // MEM BANK M1
#else
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
#endif
    hwaParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.elevationFFTSize*cfg->staticCfg.azimuthFFTSize - 1; //size in samples - 1
    hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(cmplx32ImRe_t); 
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
    hwaParamCfg[paramsetIdx].source.srcBIdx = 0;
    hwaParamCfg[paramsetIdx].source.srcScale = 0;
    hwaParamCfg[paramsetIdx].source.srcBcnt = 1 - 1; //Run through the elevation-azimuth map once
    hwaParamCfg[paramsetIdx].source.srcShift = 0; 
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
#if(LINEAR_ARRAY)
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PING_OFFSET); // MEM BANK M0
#else
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PING_OFFSET); // MEM BANK M1
#endif
    hwaParamCfg[paramsetIdx].dest.dstAcnt = 1 - 1; //Fixed for max statistics
    hwaParamCfg[paramsetIdx].dest.dstAIdx = 8;//Fixed for max statistics
    hwaParamCfg[paramsetIdx].dest.dstBIdx = 8;//Fixed for max statistics
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //Fixed for max statistics
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;//Fixed for max statistics
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED;//Fixed for max statistics
    hwaParamCfg[paramsetIdx].dest.dstScale = 8;//Fixed for max statistics
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;// Enable magnitude operation only
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;//Put max statistics output in HWA memory

    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }
    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                          paramsetIdx + aoaProcObj->hwaParamStartIdxAoA,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
    retVal = HWA_getDMAChanIndex(aoaProcObj->hwaHandle,
                                 cfg->hwRes.edmaCfg.edmaOut_maxAoA.ping.channel,
                                 &destChan);
    if (retVal != 0)
    {
        goto exit;
    }
    /* enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChan;
    errCode = HWA_enableParamSetInterrupt(aoaProcObj->hwaHandle, paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, &paramISRConfig);
    if (errCode != 0)
    {
        goto exit;
    }

    /****** Pong paramset of Azimuth FFT (by EDMA in) ******/
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[AOA_AZIMUTH_HWA_PARAMSET_RELATIVE_IDX]; 
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PONG_OFFSET);// MEM BANK M2
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PONG_OFFSET);// MEM BANK M3
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = aoaProcObj->hwaDmaTriggerAoAPong;
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                          paramsetIdx + aoaProcObj->hwaParamStartIdxAoA,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
#if(!LINEAR_ARRAY)
    /****** Pong paramset of Elevation FFT ******/  
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaParamCfg[paramsetIdx] = hwaParamCfg[AOA_ELEVATION_HWA_PARAMSET_RELATIVE_IDX];
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PONG_OFFSET);// MEM BANK M3
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PONG_OFFSET);// MEM BANK M2
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                          paramsetIdx + aoaProcObj->hwaParamStartIdxAoA,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
#endif

    /****** Pong paramset of Abs-Max of Elevation-Azimuth heatmap ******/ 
    ++paramsetIdx;
    memset((void*) &hwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
#if(!LINEAR_ARRAY)
    hwaParamCfg[paramsetIdx] = hwaParamCfg[AOA_MAX_ELEVATION_AZIMUTH_HWA_PARAMSET_RELATIVE_IDX];
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PONG_OFFSET); // MEM BANK M2
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PONG_OFFSET); // MEM BANK M3 
#else
    hwaParamCfg[paramsetIdx] = hwaParamCfg[AOA_ELEVATION_HWA_PARAMSET_RELATIVE_IDX];
    hwaParamCfg[paramsetIdx].source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_DST_PONG_OFFSET); // MEM BANK M3
    hwaParamCfg[paramsetIdx].dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(DPU_AOAPROCHWA_SRC_PONG_OFFSET); // MEM BANK M2
#endif
    retVal = HWA_configParamSet(aoaProcObj->hwaHandle, 
                                paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, 
                                &hwaParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
        goto exit;
    }
    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(aoaProcObj->hwaHandle, 
                                          paramsetIdx + aoaProcObj->hwaParamStartIdxAoA,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
        goto exit;
    }
    retVal = HWA_getDMAChanIndex(aoaProcObj->hwaHandle,
                                 cfg->hwRes.edmaCfg.edmaOut_maxAoA.pong.channel,
                                 &destChan);
    if (retVal != 0)
    {
        goto exit;
    }
    /* enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChan;
    errCode = HWA_enableParamSetInterrupt(aoaProcObj->hwaHandle, paramsetIdx + aoaProcObj->hwaParamStartIdxAoA, &paramISRConfig);
    if (errCode != 0)
    {
        goto exit;
    }

exit:
    return(errCode);
 }

 /*===========================================================
 *                    Doppler Proc External APIs
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      AoAProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]   initCfg Pointer to initial configuration parameters
 *  @param[out]  errCode Pointer to errCode generates by the API
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
DPU_AoAProcHWA_Handle DPU_AoAProcHWA_init
(
    DPU_AoAProcHWA_InitParams *initCfg,
    int32_t                    *errCode
)
{
    aoaProcHWAObj  *aoaProcObj = NULL;
    //SemaphoreP_Params       semParams;
    HWA_MemInfo             hwaMemInfo;
    uint32_t                i;
    int32_t             status = SystemP_SUCCESS;

    *errCode       = 0;
    
    if((initCfg == NULL) || (initCfg->hwaHandle == NULL))
    {
        *errCode = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }    

    /* Allocate Memory for DopplerProc */
    aoaProcObj = (aoaProcHWAObj*)&gAoAProcHeapMem;
    if(aoaProcObj == NULL)
    {
        *errCode = DPU_AOAPROCHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)aoaProcObj, 0U, sizeof(aoaProcHWAObj));
    
    /* Save init config params */
    aoaProcObj->hwaHandle   = initCfg->hwaHandle;

    /* Populate HWA base addresses and offsets. This is done only once, at init time.*/
    *errCode =  HWA_getHWAMemInfo(aoaProcObj->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {       
        goto exit;
    }

    /* check if we have enough memory banks*/
    if(hwaMemInfo.numBanks < DPU_AOAPROCHWA_NUM_HWA_MEMBANKS)
    {    
        *errCode = DPU_AOAPROCHWA_EHWARES;
        goto exit;
    }
    
    for (i = 0; i < DPU_AOAPROCHWA_NUM_HWA_MEMBANKS; i++)
    {
        aoaProcObj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress + i * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    status = SemaphoreP_constructBinary(&aoaProcObj->edmaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_AOAPROCHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done Doppler*/
    status = SemaphoreP_constructBinary(&aoaProcObj->hwaDoneSemaHandleDopp, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_AOAPROCHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done AoA*/
    status = SemaphoreP_constructBinary(&aoaProcObj->hwaDoneSemaHandleAoA, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_AOAPROCHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA paramset done on 1st three and last three paramsets*/
    status = SemaphoreP_constructBinary(&aoaProcObj->hwaSemaHandleParamset, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_AOAPROCHWA_ESEMA;
        goto exit;
    }

exit:    

    if(*errCode < 0)
    {
        aoaProcObj = (DPU_AoAProcHWA_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((DPU_AoAProcHWA_Handle)aoaProcObj);
}

/**
  *  @b Description
  *  @n
  *   AOA DPU configuration 
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_AOAPROC_ERROR_CODE
  */
int32_t DPU_AoAProcHWA_config
(
    DPU_AoAProcHWA_Handle    handle,
    DPU_AoAProcHWA_Config    *cfg
)
{
    aoaProcHWAObj            *aoaProcObj;
    int32_t                  retVal = 0;
    uint16_t                 expectedWinSamples;


    aoaProcObj = (aoaProcHWAObj *)handle;
    if(aoaProcObj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    
#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle ||
       !cfg->hwRes.hwaCfg.window ||
       !cfg->hwRes.radar_1D_FFT_Cube.data
      )
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }

    /* Check if radar cube format is supported by DPU*/
    if(cfg->hwRes.radar_1D_FFT_Cube.datafmt != DPIF_RADARCUBE_FORMAT_6)
    {
        retVal = DPU_AOAPROCHWA_ECUBEFORMAT;
        goto exit;
    }
    
    /* Check if radar cube column fits into one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_AOAPROCHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if abs value of log2 of 2D FFT fits in one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerBins * sizeof(uint16_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_AOAPROCHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if number of range bins is even*/
    if((cfg->staticCfg.numRangeBins % 2) != 0)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
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
            retVal = DPU_AOAPROCHWA_EWINDSIZE;
            goto exit;
    }        
#endif

    /* Save necessary parameters to DPU object that will be used during Process time */
    /* EDMA parameters needed to trigger first EDMA transfer*/
    aoaProcObj->edmaHandle  = cfg->hwRes.edmaCfg.edmaHandle;
    memcpy((void*)(&aoaProcObj->edmaIn_rangeFFT), (void *)(&cfg->hwRes.edmaCfg.edmaIn_rangeFFT), sizeof(DPU_AoAProc_Edma));
    memcpy((void*)(&aoaProcObj->edmaIn_dopplerFFT), (void *)(&cfg->hwRes.edmaCfg.edmaIn_dopplerFFT), sizeof(DPU_AoAProc_Edma));
    memcpy((void*)(&aoaProcObj->edmaOut_maxAoA), (void *)(&cfg->hwRes.edmaCfg.edmaOut_maxAoA), sizeof(DPU_AoAProc_Edma));
    aoaProcObj->numTxAntennas = cfg->staticCfg.numTxAntennas;
    aoaProcObj->numVirtualAntennas = cfg->staticCfg.numVirtualAntennas;
    aoaProcObj->doppFFT_is16b = cfg->staticCfg.doppFFT_is16b;
    aoaProcObj->max_num_points = cfg->maxDetmatrix_res.max_num_points;
    aoaProcObj->antenna_array_elements = cfg->maxDetmatrix_res.antenna_array_elements;
    
    /*HWA parameters needed for the 2D FFT HWA common configuration*/
    aoaProcObj->hwaParamStartIdxDopp = cfg->hwRes.hwaCfg.paramSetStartIdx;   
    aoaProcObj->hwaParamStopIdxDopp  = cfg->hwRes.hwaCfg.paramSetStartIdx + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT - 1;
    aoaProcObj->hwaAoANumLoops  = cfg->hwRes.hwaCfg.AoAhwaNumLoops;//aoaProcObj->max_num_points/2+aoaProcObj->max_num_points%2; // As ping/pong is adopted
    aoaProcObj->isDopplerPhaseCompensationEnabled = cfg->hwRes.hwaCfg.isDopplerPhaseCompensationEnabled;
    aoaProcObj->hwaParamStartIdxAoA = aoaProcObj->hwaParamStopIdxDopp+1;//cfg->hwRes.hwaCfg.paramSetStartIdx + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT;
    #if(!LINEAR_ARRAY)    
        aoaProcObj->hwaParamStopIdxAoA  = aoaProcObj->hwaParamStopIdxDopp+DPU_AOAPROCHWA_NUM_HWA_PARAMSET_AOA;//cfg->hwRes.hwaCfg.paramSetStartIdx + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_AOA - 1;
    #else
        aoaProcObj->hwaParamStopIdxAoA  = aoaProcObj->hwaParamStopIdxDopp+DPU_AOAPROCHWA_NUM_HWA_PARAMSET_AOA-2;
    #endif
    /* Parameters from the max of detection Matrix based on which AoA is done*/
    aoaProcObj->rangeGatesCount = cfg->maxDetmatrix_res.rangeGatesCount;
    aoaProcObj->range_idx_arr = cfg->maxDetmatrix_res.range_idx_arr;
    aoaProcObj->radarCubebuf = (cmplx16ImRe_t *)cfg->hwRes.radar_1D_FFT_Cube.data;
    aoaProcObj->doppler_idx_arr = cfg->maxDetmatrix_res.doppler_idx_arr;
    aoaProcObj->angle_idx_doppler_count_arr = cfg->maxDetmatrix_res.angle_idx_doppler_count_arr;

    /* Pointer to array which rearranges the data and stores the data for AoA processing */
    aoaProcObj->antenna_array = cfg->maxDetmatrix_res.antenna_array;
    aoaProcObj->angleMatBuf = (cmplx16ImRe_t *)cfg->hwRes.angleMat.data;

    /* Disable the HWA */
    retVal = HWA_enable(aoaProcObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }

    /* HWA vector multiplication RAM configuration */
    retVal = HWA_configRam(aoaProcObj->hwaHandle,
                           HWA_RAM_TYPE_INTERNAL_RAM,
                           (uint8_t *)cfg->hwRes.hwaCfg.vectorRam,
                           cfg->hwRes.hwaCfg.vectorRamSize, //size in bytes
                           cfg->hwRes.hwaCfg.vecRamOffset * sizeof(int32_t));
    
    /* HWA window configuration */
    retVal = HWA_configRam(aoaProcObj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)cfg->hwRes.hwaCfg.window,
                           cfg->hwRes.hwaCfg.windowSize, //size in bytes
                           cfg->hwRes.hwaCfg.winRamOffset * sizeof(int32_t)); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    /*******************************/
    /*Configure HWA for Doppler/AoA*/
    /*******************************/
    /*Compute source DMA channels that will be programmed in both HWA and EDMA.   
      The DMA channels are set to be equal to the paramSetIdx used by HWA*/
    /* Ping DMA channel (Ping uses the first paramset)*/  
    aoaProcObj->hwaDmaTriggerDoppPing = cfg->hwRes.hwaCfg.hwaDmaTriggerDoppPing;
    /* Pong DMA channel (Opposite path to Ping for Doppler)*/
    if(cfg->staticCfg.numTxAntennas>1)
        aoaProcObj->hwaDmaTriggerDoppPong = cfg->hwRes.hwaCfg.hwaDmaTriggerDoppPong;
    else
        aoaProcObj->hwaDmaTriggerDoppPong = cfg->hwRes.hwaCfg.hwaDmaTriggerDoppPong;
    /* Ping DMA channel (Ping uses the first paramset for AoA)*/
    aoaProcObj->hwaDmaTriggerAoAPing = cfg->hwRes.hwaCfg.hwaDmaTriggerAoAPing;
    /* Pong DMA channel (Opposite path to Ping)*/
    aoaProcObj->hwaDmaTriggerAoAPong = cfg->hwRes.hwaCfg.hwaDmaTriggerAoAPong;
    
    retVal = aoaProcHWA_ConfigHWA_Doppler(aoaProcObj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }
    retVal = aoaProcHWA_ConfigHWA_AoA(aoaProcObj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }
                    
    /*********************************/
    /** Configure EDMA for Doppler **/
    /*******************************/    
    retVal = aoaProcHWA_configEdma_Doppler(aoaProcObj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }
                    
    /*********************************/
    /**   Configure EDMA for AoA   **/
    /*******************************/    
    retVal = aoaProcHWA_configEdma_AoA(aoaProcObj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}

/**
  *  @b Description
  *  @n AOA DPU process function. 
  *   
  *  @param[in]   handle     DPU handle.
  *  @param[out]  outParams  Output parameters.
  *
  *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success     =0
  *  @retval
  *      Error      !=0 @ref DPU_AOAPROC_ERROR_CODE
  */
int32_t DPU_AoAProcHWA_Dopplerprocess
(
    DPU_AoAProcHWA_Handle    handle,
    DPU_AoAProcHWA_OutParams *outParams
)
{
    //volatile uint32_t   startTime;
    aoaProcHWAObj       *aoaProcObj;
    int32_t             retVal = 0;
    int16_t             index;
    //bool                status;
    HWA_CommonConfig    hwaCommonConfig;
    uint32_t            baseAddr, regionId, srcAddress;

    aoaProcObj = (aoaProcHWAObj *)handle;
    if (aoaProcObj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    /* Set inProgress state */
    aoaProcObj->inProgress = true;

    //startTime = Cycleprofiler_getTimeStamp();

    baseAddr = EDMA_getBaseAddr(aoaProcObj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(aoaProcObj->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    cmplx16ImRe_t *radarCubeBase = (cmplx16ImRe_t *)aoaProcObj->radarCubebuf;
    
    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(aoaProcObj->hwaHandle,
                                    aoaProcHWADoneIsrCallback_Doppler,
                                    (void*)&aoaProcObj->hwaDoneSemaHandleDopp);
    if (retVal != 0)
    {
        goto exit;
    }

    // Getting the num of times the HWA has to run based on range gates which has max num of points
    aoaProcObj->hwaDoppNumLoops  = aoaProcObj->rangeGatesCount[0]/2; // As ping/pong approach is adopted
    aoaProcObj->hwaDoppNumLoops1 = aoaProcObj->rangeGatesCount[0]%2; // In case odd number of rangeGates are detected

    EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_rangeFFT.ping.channel, EDMACC_PARAM_ENTRY_CCNT, (uint32_t) (aoaProcObj->rangeGatesCount[0]/2+aoaProcObj->rangeGatesCount[0]%2));    
    EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_rangeFFT.pong.channel, EDMACC_PARAM_ENTRY_CCNT, (uint32_t) (aoaProcObj->rangeGatesCount[0]/2));
    if(aoaProcObj->isDopplerPhaseCompensationEnabled[0])
    {
        EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_rangeFFT.ping.channel, EDMACC_PARAM_ENTRY_SRC, (uint32_t)(&radarCubeBase[aoaProcObj->range_idx_arr[0]]));    
        EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_rangeFFT.pong.channel, EDMACC_PARAM_ENTRY_SRC, (uint32_t)(&radarCubeBase[aoaProcObj->range_idx_arr[1]]));
    }

    if(aoaProcObj->rangeGatesCount[0]>1)
    {
        if(aoaProcObj->numTxAntennas>1)    
            aoaProcObj->hwaParamStopIdxDopp  = aoaProcObj->hwaParamStartIdxDopp + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT - 1;
        else
            aoaProcObj->hwaParamStopIdxDopp  = aoaProcObj->hwaParamStartIdxDopp + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT_ONLY - 1;
    }
    else
    {
        if(aoaProcObj->numTxAntennas>1)   
            aoaProcObj->hwaParamStopIdxDopp  = aoaProcObj->hwaParamStartIdxDopp + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT/2 - 1;
        else
            aoaProcObj->hwaParamStopIdxDopp  = aoaProcObj->hwaParamStartIdxDopp + DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT_ONLY/2 - 1; 
    }
    
    uint32_t *setSoftwareTrigger = (uint32_t *)0x55010008;
    
    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    if(aoaProcObj->hwaDoppNumLoops>0)
    {
        memset((void*) &hwaCommonConfig, 0, sizeof(HWA_CommonConfig));

        /* Config Common Registers */
        hwaCommonConfig.configMask =
            HWA_COMMONCONFIG_MASK_NUMLOOPS |
            HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
            HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
            HWA_COMMONCONFIG_MASK_FFT1DENABLE |
            HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;

        hwaCommonConfig.numLoops      = aoaProcObj->hwaDoppNumLoops;     
        hwaCommonConfig.paramStartIdx = aoaProcObj->hwaParamStartIdxDopp;
        hwaCommonConfig.paramStopIdx  = aoaProcObj->hwaParamStopIdxDopp; 
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
        hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
        
        retVal = HWA_configCommon(aoaProcObj->hwaHandle, &hwaCommonConfig);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Enable the HWA */
        retVal = HWA_enable(aoaProcObj->hwaHandle,1); 
        if (retVal != 0)
        {
            goto exit;
        }

        for(index = 0; index < (aoaProcObj->hwaDoppNumLoops*2); index++)
        {
            if(index % 2 == 0)
            {
                EDMAEnableTransferRegion(baseAddr, regionId, aoaProcObj->edmaIn_rangeFFT.ping.channel, EDMA_TRIG_MODE_MANUAL);
                if(DEBUG_SINGLE_STEP_HWA)
                {
                    *setSoftwareTrigger = 0x1;
                    *setSoftwareTrigger = 0x1;
                }
                srcAddress = (uint32_t)(&radarCubeBase[aoaProcObj->range_idx_arr[index+1]]);
                EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_rangeFFT.pong.channel, EDMACC_PARAM_ENTRY_SRC, (uint32_t) ((void *)(srcAddress)));
                
            }
            else
            {   
                EDMAEnableTransferRegion(baseAddr, regionId, aoaProcObj->edmaIn_rangeFFT.pong.channel, EDMA_TRIG_MODE_MANUAL);
                if(DEBUG_SINGLE_STEP_HWA)
                {
                    *setSoftwareTrigger = 0x1;
                    *setSoftwareTrigger = 0x1;
                }
                srcAddress = (uint32_t)(&radarCubeBase[aoaProcObj->range_idx_arr[index+1]]);
                EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_rangeFFT.ping.channel, EDMACC_PARAM_ENTRY_SRC, (uint32_t) ((void *)(srcAddress)));
            }

            SemaphoreP_pend((void *)&aoaProcObj->hwaSemaHandleParamset, SystemP_WAIT_FOREVER);

            copyData_after_2DFFT(aoaProcObj, index);
        }
    
        /**********************************************/
        /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
        /**********************************************/
        SemaphoreP_pend(&aoaProcObj->hwaDoneSemaHandleDopp, SystemP_WAIT_FOREVER);
    }
    /*************************************************************************/
    /*       This if condition is for the case of odd no of range gates     */
    /***********************************************************************/
    if(aoaProcObj->hwaDoppNumLoops1==1)
    {
        /* Disable the HWA */
        retVal = HWA_enable(aoaProcObj->hwaHandle, 0); 
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = HWA_enableDoneInterrupt(aoaProcObj->hwaHandle,
                                    aoaProcHWADoneIsrCallback_Doppler,
                                    (void*)&aoaProcObj->hwaDoneSemaHandleDopp);
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
            HWA_COMMONCONFIG_MASK_PARAMSTOPIDX;

        hwaCommonConfig.numLoops      = aoaProcObj->hwaDoppNumLoops1;     
        hwaCommonConfig.paramStartIdx = aoaProcObj->hwaParamStartIdxDopp;
        if(aoaProcObj->numTxAntennas>1)
            hwaCommonConfig.paramStopIdx  = aoaProcObj->hwaParamStartIdxDopp+DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT/2-1;// Do the ping part of it for AoA calculation 
        else
            hwaCommonConfig.paramStopIdx  = aoaProcObj->hwaParamStartIdxDopp+DPU_AOAPROCHWA_NUM_HWA_PARAMSET_DOPPLER_FFT_ONLY/2-1;// Do the ping part of it for AoA calculation 
        
        retVal = HWA_configCommon(aoaProcObj->hwaHandle, &hwaCommonConfig);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Enable the HWA */
        retVal = HWA_enable(aoaProcObj->hwaHandle,1); 
        if (retVal != 0)
        {
            goto exit;
        }

        EDMAEnableTransferRegion(baseAddr, regionId, aoaProcObj->edmaIn_rangeFFT.ping.channel, EDMA_TRIG_MODE_MANUAL);
        if(DEBUG_SINGLE_STEP_HWA)
        {
            // HWA_setSoftwareTrigger(aoaProcObj->hwaHandle);
            // HWA_setSoftwareTrigger(aoaProcObj->hwaHandle);
            *setSoftwareTrigger = 0x1;
            *setSoftwareTrigger = 0x1;
        }
        SemaphoreP_pend((void *)&aoaProcObj->hwaSemaHandleParamset, SystemP_WAIT_FOREVER);
        
        copyData_after_2DFFT(aoaProcObj, aoaProcObj->hwaDoppNumLoops*2);

        /**********************************************/
        /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
        /**********************************************/
        SemaphoreP_pend(&aoaProcObj->hwaDoneSemaHandleDopp, SystemP_WAIT_FOREVER);

    }

    HWA_disableDoneInterrupt(aoaProcObj->hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(aoaProcObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    if (aoaProcObj != NULL)
    {
        aoaProcObj->inProgress = false;
    }    
    
    return retVal;
}

/**
  *  @b Description
  *  @n AOA DPU process function. 
  *   
  *  @param[in]   handle     DPU handle.
  *  @param[out]  outParams  Output parameters.
  *
  *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success     =0
  *  @retval
  *      Error      !=0 @ref DPU_AOAPROC_ERROR_CODE
  */
int32_t DPU_AoAProcHWA_AoAprocess
(
    DPU_AoAProcHWA_Handle    handle,
    DPU_AoAProcHWA_OutParams *outParams
)
{
    //volatile uint32_t   startTime;
    aoaProcHWAObj       *aoaProcObj;
    int32_t             retVal = 0;
    //bool                status;
    HWA_CommonConfig    hwaCommonConfig;
    uint32_t            baseAddr, regionId;

    aoaProcObj = (aoaProcHWAObj *)handle;
    if (aoaProcObj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    /* Set inProgress state */
    aoaProcObj->inProgress = true;

    //startTime = Cycleprofiler_getTimeStamp();

    baseAddr = EDMA_getBaseAddr(aoaProcObj->edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(aoaProcObj->edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_dopplerFFT.ping.channel, EDMACC_PARAM_ENTRY_CCNT, (uint32_t) (aoaProcObj->hwaAoANumLoops[0]));    
    EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaIn_dopplerFFT.pong.channel, EDMACC_PARAM_ENTRY_CCNT, (uint32_t) (aoaProcObj->hwaAoANumLoops[0]));
    EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaOut_maxAoA.ping.channel, EDMACC_PARAM_ENTRY_CCNT, (uint32_t) (aoaProcObj->hwaAoANumLoops[0]));
    EDMADmaSetPaRAMEntry(baseAddr, aoaProcObj->edmaOut_maxAoA.pong.channel, EDMACC_PARAM_ENTRY_CCNT, (uint32_t) (aoaProcObj->hwaAoANumLoops[0]));

    uint32_t *setSoftwareTrigger = (uint32_t *)0x55010008;
    
    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(aoaProcObj->hwaHandle,
                                    aoaProcHWADoneIsrCallback_AoA,
                                    (void*)&aoaProcObj->hwaDoneSemaHandleAoA);
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
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;

    hwaCommonConfig.numLoops      = aoaProcObj->hwaAoANumLoops[0];     
    hwaCommonConfig.paramStartIdx = aoaProcObj->hwaParamStartIdxAoA;
    hwaCommonConfig.paramStopIdx  = aoaProcObj->hwaParamStopIdxAoA; 
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    
    retVal = HWA_configCommon(aoaProcObj->hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(aoaProcObj->hwaHandle,1); 
    if (retVal != 0)
    {
        goto exit;
    }

    EDMAEnableTransferRegion(baseAddr, regionId, aoaProcObj->edmaIn_dopplerFFT.ping.channel, EDMA_TRIG_MODE_MANUAL);
    EDMAEnableTransferRegion(baseAddr, regionId, aoaProcObj->edmaIn_dopplerFFT.pong.channel, EDMA_TRIG_MODE_MANUAL);

    if(DEBUG_SINGLE_STEP_HWA)
    {
        uint8_t i;
        for(i=0;i<aoaProcObj->hwaAoANumLoops[0];++i)
        {
            // HWA_setSoftwareTrigger(aoaProcObj->hwaHandle);
            // HWA_setSoftwareTrigger(aoaProcObj->hwaHandle);
            // HWA_setSoftwareTrigger(aoaProcObj->hwaHandle);
            // HWA_setSoftwareTrigger(aoaProcObj->hwaHandle);
            #if(!LINEAR_ARRAY)
                *setSoftwareTrigger = 0x1;
                *setSoftwareTrigger = 0x1;
                *setSoftwareTrigger = 0x1;
                *setSoftwareTrigger = 0x1;
            #else
                *setSoftwareTrigger = 0x1;
                *setSoftwareTrigger = 0x1;
            #endif
        }
    }
    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    SemaphoreP_pend(&aoaProcObj->hwaDoneSemaHandleAoA, SystemP_WAIT_FOREVER);

    HWA_disableDoneInterrupt(aoaProcObj->hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(aoaProcObj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************/
    /* WAIT FOR EDMA DONE INTERRUPT               */
    /**********************************************/
    SemaphoreP_pend(&aoaProcObj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

exit:
    if (aoaProcObj != NULL)
    {
        aoaProcObj->inProgress = false;
    }    
    
    return retVal;
}


/**
  *  @b Description
  *  @n
  *  AoA DPU deinit 
  *
  *  @param[in]   handle   DPU handle.
  *
  *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_AOAPROC_ERROR_CODE
  */
int32_t DPU_AoAProcHWA_deinit(DPU_AoAProcHWA_Handle handle)
{
    aoaProcHWAObj     *aoaProcObj;
    int32_t             retVal = 0;

    /* Sanity Check */
    aoaProcObj = (aoaProcHWAObj *)handle;
    if(aoaProcObj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }

    /* Delete Semaphores */
    SemaphoreP_destruct(&aoaProcObj->edmaDoneSemaHandle);
    SemaphoreP_destruct(&aoaProcObj->hwaDoneSemaHandleDopp);
    SemaphoreP_destruct(&aoaProcObj->hwaDoneSemaHandleAoA);

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
int32_t DPU_AoAProcHWA_GetNumUsedHwaParamSets
(
    DPU_AoAProcHWA_Handle    handle,
    uint8_t *numUsedHwaParamSets
)
{
    aoaProcHWAObj *obj;
    int32_t retVal = 0;

    obj = (aoaProcHWAObj *)handle;
    if (obj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    *numUsedHwaParamSets = (uint8_t) (obj->hwaParamStopIdxDopp - obj->hwaParamStartIdxDopp + 1 +  obj->hwaParamStopIdxAoA- obj->hwaParamStartIdxAoA + 1);
exit:
    return retVal;
}
