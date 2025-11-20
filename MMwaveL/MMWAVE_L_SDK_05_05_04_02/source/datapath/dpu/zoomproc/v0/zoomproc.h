/*
 *  
 *  NOTE:
 *      (C) Copyright 2018 - 2023 Texas Instruments, Inc.
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
 *   @file  zoomproc.h
 *
 *   @brief
 *      Implements zoom FFT processing functionality using HWA.
 *      Zoom FFT processing is done by HWA hardware. Based on the RF parameters and range FFT output, 
 *      zoomProc configures hardware accelerator FFT engine in slow DFT mode accordingly. 
 *      It also configures data input and output EDMA channels to bring data in and out of 
 *      zoom Processing memory.
 *
 *      The process is triggered by the peak location availability based on 1D FFT.
 *
 *      After HWA processing is done, it generates interrupt to zoomProc DPU, at the same time triggers EDMA data
 *      output channel to copy zoom FFT results. EDMA interrupt done interrupt is triggered by EDMA hardware
 *      after the copy is completed.
 */
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef ZOOMPROC_H_
#define ZOOMPROC_H_

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* MCU Plus SDK Drivers include files */
#include <drivers/hwa.h>

/* mmWave SDK Data Path Include Files */
#include <datapath/dpif/dpif_adcdata.h>
#include <datapath/dpif/dpif_radarcube.h>
#include <datapath/dpif/dp_error.h>
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>

#ifdef __cplusplus
extern "C" {
#endif
 
/*! Number of HWA parameter sets */
#define DPU_ZOOMPROC_NUM_HWA_PARAM_SETS             1U

/** @addtogroup DPU_ZOOMPROC_ERROR_CODE
 *  Base error code for the zoomProc DPU is defined in the
 *  \include datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_ZOOMPROC_EINVAL                  (DP_ERRNO_ZOOM_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_ZOOMPROC_ENOMEM                  (DP_ERRNO_ZOOM_PROC_BASE-2)

/**
 * @brief   Error Code: Internal error
 */
#define DPU_ZOOMPROC_EINTERNAL               (DP_ERRNO_ZOOM_PROC_BASE-3)

/**
 * @brief   Error Code: Not implemented
 */
#define DPU_ZOOMPROC_ENOTIMPL                (DP_ERRNO_ZOOM_PROC_BASE-4)

/**
 * @brief   Error Code: In Progress
 */
#define DPU_ZOOMPROC_EINPROGRESS             (DP_ERRNO_ZOOM_PROC_BASE-5)

/**
 * @brief   Error Code: Invalid control command
 */
#define DPU_ZOOMPROC_ECMD                    (DP_ERRNO_ZOOM_PROC_BASE-6)

/**
 * @brief   Error Code: Semaphore error
 */
#define DPU_ZOOMPROC_ESEMA                   (DP_ERRNO_ZOOM_PROC_BASE-7)

/**
@}
*/

/**
 * @brief   DPU Operating Modes
 */
typedef uint32_t DPU_ZoomProc_InputMode;
#define DPU_ZoomProc_InputMode_MAPPED              (uint32_t) 0U
#define DPU_ZoomProc_InputMode_ISOLATED            (uint32_t) 1U  
#define DPU_ZoomProc_InputMode_HWA_INTERNAL_MEM    (uint32_t) 2U

/**
 * @brief
 *  ZoomProc HWA configuration
 *
 * @details
 *  The structure is used to hold the HWA configuration needed for Range FFT
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_HwaConfig_t
{
    /*! @brief     HWA paramset Start index */
    uint8_t         paramSetStartIdx;

    /*! @brief     Number of HWA param sets must be @ref DPU_ZOOMPROC_NUM_HWA_PARAM_SETS */
    uint8_t         numParamSet;

    /*! @brief     Data Input Mode, to configure EDMA in for the adc input */
    DPU_ZoomProc_InputMode      dataInputMode;

    /*! @brief     ZoomProc HWA data input paramset dma trigger source channel */
    uint8_t        dmaTrigSrcChan;

}DPU_ZoomProc_HwaConfig;

/**
 * @brief
 *  ZoomProc EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Zoom FFT
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_EDMAInputConfig_t
{
    /*! @brief     EDMA configuration for zoomProc data Input
                    This is needed only in @ref DPU_ZoomProc_InputMode_ISOLATED
     */
    DPEDMA_ChanCfg        dataIn;

    /*! @brief     EDMA configuration for zoomProc data Input Signature */
    DPEDMA_ChanCfg        dataInSignature;
}DPU_ZoomProc_EDMAInputConfig;

/**
 * @brief
 *  ZoomProc EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Zoom FFT
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_EDMAOutputConfig_t
{
    /*! @brief     EDMA configuration for zoomProc data Out.
                    It must be a HWACC triggered EDMA channel.
     */
    DPEDMA_ChanCfg        dataOutZoom;

}DPU_ZoomProc_EDMAOutputConfig;


/**
 * @brief
 *  ZoomProc hardware resources
 *
 * @details
 *  The structure is used to hold the hardware resources needed for Zoom FFT
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_HW_Resources_t
{
    /*! @brief     EDMA Handle */
    EDMA_Handle         edmaHandle;

    /*! @brief     HWA configuration */
    DPU_ZoomProc_HwaConfig      hwaCfg;

    /*! @brief     EDMA configuration for zoomProc data Input */
    DPU_ZoomProc_EDMAInputConfig edmaInCfg;

    /*! @brief     EDMA configuration for zoomProc data Output */ 
    DPU_ZoomProc_EDMAOutputConfig edmaOutCfg;

    /*! @brief     EDMA interrupt object */
    Edma_IntrObject *intrObj;

    /*! @brief      Pointer to DFT output buffer */
    cmplx32ImRe_t   *zoomDftOut;

    /*! @brief      DFT output buffer size */
    uint32_t        zoomDftOutSize;

    /*! @brief     Zoom in frequencies as range bin locations based on 1D FFT resolution*/
    uint16_t        *zoomInRbin;

}DPU_ZoomProc_HW_Resources;

/**
 * @brief
 *  ZoomProc static configuration
 *
 * @details
 *  The structure is used to hold the static configuraiton used by zoomProc
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  Enable for offline ADC test mode */
    bool        isTestMode;

    /* Number of ADC samples */
    //uint16_t    numAdcSamples;

    /*! @brief  Range FFT frequency bin resolution */
    float       freqRes;

    /*! @brief  Chirp ramp end time/duration - in sec */
    float       zoomRngFactor; // 3.0e8 * chirpRampTime/(2 * chirpBandwidth)

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief  Number of zoom DFT bins */
    uint16_t    numDftBins;

    /*! @brief  zoom FFT size */
    uint16_t    zoomFftSize;

    /*! @brief  number of samples to zoom on either side of peak freq of interest */
    uint16_t    zoomSamplesOneSide;

    /*! @brief  interpolation factor for zoom in */
    uint16_t    interpFactor;

    /*! @brief  1D FFT peak bin location */
    uint16_t    *peakLoc;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief      ADCBuf buffer interface */
    DPIF_ADCBufData     ADCBufData;

    /*! @brief  Low power mode 0-disabled, 1-enabled, 2-test mode (power stays on, system coftware components reset) */
     uint8_t    lowPowerMode;
}DPU_ZoomProc_StaticConfig;

/**
 * @brief
 *  Range FFT configuration
 *
 * @details
 *  The structure is used to hold the configuration needed for Range FFT
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_Config_t
{
    /*! @brief     zoomProc hardware resources */
    DPU_ZoomProc_HW_Resources   hwRes;

    /*! @brief     zoomProc static configuration */
    DPU_ZoomProc_StaticConfig   staticCfg;

}DPU_ZoomProc_Config;

/**
 * @brief
 *  zoomProc output parameters populated during zoomProc Processing time
 *
 * @details
 *  The structure is used to hold the output parameters for zoomProc
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_InitParams_t
{
    /*! @brief     HWA Handle */
    HWA_Handle          hwaHandle;
}DPU_ZoomProc_InitParams;

/**
 * @brief
 *  zoomProc output parameters populated during zoomProc Processing time
 *
 * @details
 *  The structure is used to hold the output parameters for zoomProc
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_ZoomProc_OutParams_t
{
    /*! @brief     zoomProc stats */
    float  zoomRangeMeas;

    /*! @brief     End of Chirp indication for rangeProcHWA */
    bool      endOfChirp;
}DPU_ZoomProc_OutParams;

/**
 * @brief
 *  zoomProc DPU Handle
 *
 *  \ingroup DPU_ZOOMPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef void* DPU_ZoomProc_Handle ;

/**
 * @brief
 *  ZoomProc DPU Object
 *
 * @details
 *  The structure is used to hold ZoomProc internal data object
 *
 *  \ingroup DPU_ZOOMPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct ZoomProcObj_t
{
    DPU_ZoomProc_InitParams     initParms;

    /*! @brief     EDMA Handle */
    EDMA_Handle                 edmaHandle;

    /*! @brief      ZoomProc static configuration */
    DPU_ZoomProc_StaticConfig    params;

    /*! @brief     HWA configuration */
    DPU_ZoomProc_HwaConfig      hwaCfg;

    bool                    isTestMode;

    /*! @brief     ZoomProc HWA data input paramset trigger */
    uint8_t                 dataInTrigger;

    /*! @brief     ZoomProc HWA data output paramset trigger */
    uint8_t                 dataOutTrigger;

    /*! @brief     EDMA done semaphore */
    SemaphoreP_Object       edmaDoneSemaHandle;

    /*! @brief     HWA Processing Done semaphore Handle */
    SemaphoreP_Object       hwaDoneSemaHandle;

    /*! @brief      HWA Memory address */
    uint32_t                hwaMemBankAddr[SOC_HWA_NUM_MEM_BANKS];

    /*! @brief      Pointer to ADC buffer */
    cmplx16ImRe_t           *ADCdataBuf;

    /*! @brief      Pointer to Radar Cube buffer */
    cmplx32ImRe_t           *zoomDftOut;

    /*! @brief     Zoom in frequencies as range bin locations based on 1D FFT resolution*/
    uint16_t                *zoomInRbin;
    
    /*! @brief     DMA data out event channel */
    uint8_t                 dataOutZoomChan;

    /*! @brief     zoomProc DPU is in processing state */
    bool                    inProgress;

    /*! @brief     Total number of zoomProc DPU processing */
    uint32_t                numProcess;

    /*! @brief     Total number of data output EDMA done interrupt */
    uint32_t                numEdmaDataOutCnt;

}ZoomProcObj;

/*================================================================
               zoomProc DPU exposed APIs            
 ================================================================*/
DPU_ZoomProc_Handle DPU_ZoomProc_init
(
    DPU_ZoomProc_InitParams     *initParams,
    int32_t*                        errCode
);

int32_t DPU_ZoomProc_config
(
    DPU_ZoomProc_Handle     handle,
    DPU_ZoomProc_Config*    zoomCfg
);

int32_t DPU_ZoomProc_process
(
    DPU_ZoomProc_Handle     handle,
    DPU_ZoomProc_OutParams* outParams
);

int32_t DPU_ZoomProc_deinit
(
    DPU_ZoomProc_Handle     handle
);

#ifdef __cplusplus
}
#endif

#endif
