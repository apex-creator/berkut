/*
  *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *
 */

/*
 *  ======== hwa.c ========
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hwa.h>
#include <drivers/hwa/v0/soc/hwa_soc.h>
#include <drivers/hw_include/hw_types.h>

#define HWA_PARAM_CHECK

#define HWA_GET_DRIVER_STRUCT(handle) \
{\
     ptrHWADriver = (HWA_Object *)handle;\
}

//TODO: The below global variables come from hwa cfg xdt template
//uint32_t         gHwaConfigNum;
//HWA_Attrs        gHwaAttrs[2];
//HWA_Object       gHwaObject[2];
//HWA_Object      *gHwaObjectPtr[2];

HWA_InterruptCtx HwaParamsetIntr[SOC_HWA_NUM_PARAM_SETS];

/* HWA dma destination channel index to EDMA channel id mapping */
uint8_t  gHwaEDMAChanMapping [32] =
{
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ0,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ1,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ2,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ3,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ4,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ5,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ6,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ7,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ8,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ9,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ10,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ11,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ12,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ13,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ14,
    EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ15
};

/* INTERNAL FUNCTIONS */

/* PROTOTYPES */
static int32_t HWA_getDriverAccess(HWA_Object *ptrHWADriver, bool checkConfigProgress, bool checkParamProgress, uint32_t paramsetIdx);
static void HWA_releaseDriverAccess(HWA_Object *ptrHWADriver, bool configProgress, bool paramProgress, uint32_t paramsetIdx);
static int32_t HWA_validateParamSetConfig(HWA_Object *ptrHWADriver, HWA_ParamConfig *paramConfig);
static void HWA_deleteInstance(HWA_Object *ptrHWADriver);
static void HWA_paramDoneISR(void *arg);
static void HWA_allParamDoneISR(void *arg);

static inline void HWA_doReset(DSSHWACCRegs  *ctrlBaseAddr);

/** @addtogroup HWA_DRIVER_INTERNAL_FUNCTION
 @{ */
/*!
 *  @brief  Function to validate the state of the driver and gain access.
 *
 *  @param  ptrHWADriver         Internal HWA driver object that will be checked for sanity
 *
 *  @param  checkConfigProgress  flag to indicate whether to check driver's configInProgress bit
 *
 *  @param  checkParamProgress   flag to indicate whether to check driver's paramSetMapInProgress bitmap
 *
 *  @param  paramsetIdx          should be valid if checkParamProgress is set to true.
 *
 *  @return 0 if driver access is allowed; error code if access is denied.
 *
 */
static int32_t HWA_getDriverAccess(HWA_Object *ptrHWADriver, bool checkConfigProgress, bool checkParamProgress, uint32_t paramsetIdx)
{
    int32_t retCode = 0;
    uintptr_t           key;

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check state of driver */
    if ((ptrHWADriver == NULL)  || (ptrHWADriver->refCnt == 0U))
    {
        retCode = HWA_ENOINIT;
    }
    else
    {
        /* check config first */
        if (checkConfigProgress==true)
        {
            if (ptrHWADriver->configInProgress == 1U)
            {
                retCode = HWA_EINUSE;
            }
            else
            {
                ptrHWADriver->configInProgress = 1U;
            }
        }
        /* next check paramset */
        else if (checkParamProgress==true) /* GAP COMMENT ID: 1 START END*/
        {
            /*first check for valid index */
            if (paramsetIdx>=ptrHWADriver->hwAttrs->numHwaParamSets) /*GAP COMMENT ID:2 START*/
            {
                retCode = HWA_EINVAL;
            }/*GAP COMMENT ID:2 END*/
            /* next check for inProgress bit */
            else if ((ptrHWADriver->paramSetMapInProgress & ((uint16_t)1U <<paramsetIdx))==1U)
            {
                retCode = HWA_EINUSE;
            }
            else
            {
                /* no error */
                retCode = 0;
                ptrHWADriver->paramSetMapInProgress =
                    ptrHWADriver->paramSetMapInProgress | ((uint16_t)1U <<paramsetIdx);

            }
        }
        else /*GAP COMMENT ID:3 START*/
        {
            /* no error */
            retCode = 0;
        }/*GAP COMMENT ID:3 END*/
    }

    /* Restore the interrupts: */
    HwiP_restore(key);

    return retCode;
}

/*!
 *  @brief  Function to release access to the driver.
 *
 *  @param  ptrHWADriver         Internal HWA driver object that will be checked for sanity
 *
 *  @param  configProgress  flag to indicate whether to release driver's configInProgress bit
 *
 *  @param  paramProgress   flag to indicate whether to release driver's paramSetMapInProgress bitmap
 *
 *  @param  paramsetIdx          should be valid if paramProgress is set to true.
 *
 *  @return none
 *
 */
static void HWA_releaseDriverAccess(HWA_Object *ptrHWADriver, bool configProgress, bool paramProgress, uint32_t paramsetIdx)
{
    uintptr_t           key;

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check config first */
    if (configProgress==true)
    {
        ptrHWADriver->configInProgress = 0;
    }
    /* next check paramset */
    if (paramProgress==true)
    {
        ptrHWADriver->paramSetMapInProgress=
            (ptrHWADriver->paramSetMapInProgress & (uint32_t)(~(1U <<paramsetIdx)));
    }

    /* Restore the interrupts: */
    HwiP_restore(key);

    return;
}


/*!
 *  @brief  Function to validate the user passed paramConfig. This function gets compiled out
 *          if HWA_PARAM_CHECK is disabled.
 *
 *  @param  ptrHWADriver         Internal HWA driver object that will be checked for sanity
 *
 *  @param  paramConfig          user supplied paramSet Config
 *
 *  @return 0 if valid paramSet config if provided else a valid error code.
 *
 */
static int32_t HWA_validateParamSetConfig(HWA_Object *ptrHWADriver, HWA_ParamConfig *paramConfig)
{
    int32_t retCode = 0;
#ifdef HWA_PARAM_CHECK
    if (paramConfig == NULL)
    {
        /* invalid config */
        retCode = HWA_EINVAL;
    }
    else if (
        (paramConfig->triggerMode > HWA_TRIG_MODE_DMA) ||
        (paramConfig->dmaTriggerSrc > (ptrHWADriver->hwAttrs->numHwaParamSets - 1U)) ||
        (paramConfig->accelMode > HWA_ACCELMODE_NONE) ||
        (paramConfig->source.srcAcnt >= (1U << 12)) ||
        (paramConfig->source.srcBcnt >= (1U << 12)) ||
        (paramConfig->source.srcShift >= (1U << 12)) ||
        (paramConfig->source.srcCircShiftWrap >= (1U << 4)) ||
        (paramConfig->source.srcRealComplex > HWA_SAMPLES_FORMAT_REAL) ||
        (paramConfig->source.srcWidth > HWA_SAMPLES_WIDTH_32BIT) ||
        (paramConfig->source.srcSign > HWA_SAMPLES_SIGNED) ||
        (paramConfig->source.srcConjugate > HWA_FEATURE_BIT_ENABLE) ||
        (paramConfig->source.srcScale > 8U) ||
        (paramConfig->source.bpmEnable > HWA_FEATURE_BIT_ENABLE) ||
        (paramConfig->source.bpmPhase >= (1U << 4)) ||
        (paramConfig->dest.dstAcnt >= (1U << 12)) ||
        (paramConfig->dest.dstRealComplex > HWA_SAMPLES_FORMAT_REAL) ||
        (paramConfig->dest.dstWidth > HWA_SAMPLES_WIDTH_32BIT) ||
        (paramConfig->dest.dstSign > HWA_SAMPLES_SIGNED) ||
        (paramConfig->dest.dstConjugate > HWA_FEATURE_BIT_ENABLE) ||
        (paramConfig->dest.dstScale > 8U) ||
        (paramConfig->dest.dstSkipInit >= (1U << 10)) ||
        ((paramConfig->accelMode != HWA_ACCELMODE_NONE) && (paramConfig->source.srcAddr == paramConfig->dest.dstAddr))
        )
    {

        /* invalid config params */
        retCode = HWA_EINVAL;
    }
    else if ((paramConfig->accelMode == HWA_ACCELMODE_FFT) &&
        (  /* if FFT mode, then fftEn should be checked for correct values */
        (paramConfig->accelModeArgs.fftMode.fftEn > HWA_FEATURE_BIT_ENABLE) ||
            ( /* if fftEn is set to Enable, then fftSize should be checked for correct values */
            (paramConfig->accelModeArgs.fftMode.fftEn == HWA_FEATURE_BIT_ENABLE) &&
                (
                (paramConfig->accelModeArgs.fftMode.fftSize < 1U) ||
                    (paramConfig->accelModeArgs.fftMode.fftSize > 10U)
                    )
                ) ||
                (paramConfig->accelModeArgs.fftMode.butterflyScaling >= (1U << 10)) ||
            (paramConfig->accelModeArgs.fftMode.interfZeroOutEn > HWA_FEATURE_BIT_ENABLE) ||
            (paramConfig->accelModeArgs.fftMode.windowEn > HWA_FEATURE_BIT_ENABLE) ||
            (paramConfig->accelModeArgs.fftMode.windowStart >= (1U << 10)) ||
            (paramConfig->accelModeArgs.fftMode.winSymm > HWA_FFT_WINDOW_SYMMETRIC) ||
            (paramConfig->accelModeArgs.fftMode.winInterpolateMode > HWA_FFT_WINDOW_INTERPOLATE_MODE_2K) ||
            (paramConfig->accelModeArgs.fftMode.magLogEn > HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED) ||
            (paramConfig->accelModeArgs.fftMode.fftOutMode > HWA_FFT_MODE_OUTPUT_SUM_STATS)
            )
        )
    {
        /* invalid config params */
        retCode = HWA_EINVAL;
    }
    else if (
        (paramConfig->preProcCfg.dcEstResetMode > HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET_ZEROLPCONT) ||
        (paramConfig->preProcCfg.dcSubEnable > HWA_FEATURE_BIT_ENABLE) ||
        (paramConfig->preProcCfg.dcSubSelect > HWA_DCSUB_SELECT_DCEST) ||
        (paramConfig->preProcCfg.interfLocalize.thresholdEnable > HWA_INTERFERENCE_ENABLE) ||
        (paramConfig->preProcCfg.interfLocalize.thresholdMode > HWA_INTERFERENCE_THRESH_MODE_MAG_AND_MAGDIFF) ||
        (paramConfig->preProcCfg.interfLocalize.thresholdSelect > HWA_INTERFERENCE_THRESH_SELECT_BUILTIN) ||
        (paramConfig->preProcCfg.interfStat.resetMode > HWA_INTERFERENCE_STATS_RESET_PER_FRAME)
        /* for fft stitching, first paramset, using the winInterpolateMode, but the cmultMode is set to disable,
            only second paramset is set to stitching mode */
        //((paramConfig->complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_DISABLE) &&
        //    (paramConfig->complexMultiply.modeCfg.FFTstitching.winInterpolateMode > HWA_FFT_WINDOW_INTERPOLATE_MODE_4K))
        )
    {
        /* invalid config */
        retCode = HWA_EINVAL;
    }
    else if (paramConfig->accelMode == HWA_ACCELMODE_CFAR) 
    {
       if( (paramConfig->accelModeArgs.cfarMode.numGuardCells >= (1U << 3)) ||
            (paramConfig->accelModeArgs.cfarMode.nAvgDivFactor > 8U) ||
            (paramConfig->accelModeArgs.cfarMode.nAvgMode > HWA_NOISE_AVG_MODE_CFAR_OS) ||
            (paramConfig->accelModeArgs.cfarMode.operMode > HWA_CFAR_OPER_MODE_LOG_INPUT_COMPLEX_LINEARCFAR) ||
            (paramConfig->accelModeArgs.cfarMode.outputMode > HWA_CFAR_OUTPUT_MODE_I_PEAK_IDX_Q_CUT) ||
            (paramConfig->accelModeArgs.cfarMode.peakGroupEn > HWA_FEATURE_BIT_ENABLE) ||
            (paramConfig->accelModeArgs.cfarMode.cyclicModeEn > HWA_FEATURE_BIT_ENABLE)
        )
        {
            /* invalid config params */
        retCode = HWA_EINVAL_PARAMSET_CFARMODE_GENERALCONFIG;
        }

        if ((paramConfig->accelModeArgs.cfarMode.nAvgMode == HWA_NOISE_AVG_MODE_CFAR_OS) &&
        (
            (paramConfig->accelModeArgs.cfarMode.cfarOsKvalue > 63U) ||
            (paramConfig->accelModeArgs.cfarMode.cfarOsEdgeKScaleEn > HWA_FEATURE_BIT_ENABLE) ||
            ((paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 0U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 4U)
                && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 6U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 8U)
                && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 12U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 16U)
                && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 24U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 32U)
            ) ||
            ((paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 0U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 4U)
                && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 6U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 8U)
                && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 12U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 16U)
                && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 24U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight != 32U)
            ) ||
            (
                ((paramConfig->accelModeArgs.cfarMode.operMode != HWA_CFAR_OPER_MODE_LOG_INPUT_REAL) &&
                 (paramConfig->accelModeArgs.cfarMode.operMode != HWA_CFAR_OPER_MODE_LOG_INPUT_COMPLEX))
            )
        )
        )
        {
            /* invalid config params */
            retCode = HWA_EINVAL_PARAMSET_CFARMODE_OSCONFIG;
        }
    }
    else if (paramConfig->accelMode==HWA_ACCELMODE_COMPRESS)
	{
		if (ptrHWADriver->hwAttrs->isCompressionEnginePresent == false)/*GAP COMMENT ID:6 START*/
		{
			/* not present on device. */
			retCode = HWA_ENOTSUPP;

		}/*GAP COMMENT ID:6 END*/
		else if (
				(paramConfig->accelModeArgs.compressMode.compressDecompress > HWA_CMP_DCMP_DECOMPRESS ) ||
                ((paramConfig->accelModeArgs.compressMode.method != HWA_COMPRESS_METHOD_BFP ) &&
                 (paramConfig->accelModeArgs.compressMode.method != HWA_COMPRESS_METHOD_EGE)) ||
                (paramConfig->accelModeArgs.compressMode.ditherEnable>HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.compressMode.passSelect!=HWA_COMPRESS_PATHSELECT_BOTHPASSES) ||
                (paramConfig->accelModeArgs.compressMode.headerEnable>HWA_FEATURE_BIT_ENABLE) ||
				(paramConfig->accelModeArgs.compressMode.scaleFactorBW > 15U) ||
                (paramConfig->accelModeArgs.compressMode.BFPMantissaBW > ( 1U << 5U) ) ||
				(paramConfig->accelModeArgs.compressMode.EGEKarrayLength > 3U)
        )
    {
        /* invalid config params */
        retCode = HWA_EINVAL;
    }
        else
        {
            /*MISRA-C*/
        }
    }
    else
    {
        retCode = 0;
    }
#endif
    return retCode;
}


/*!
 *  @brief  Function to de-initialize HWA specified by the passed driver object. No driver
 *  functions should be invoked after this call.
 *
 *  @pre    HWA_open() has been called
 *
 *  @param  ptrHWADriver         Internal HWA driver object
 *
 *  @return none
 *
 *  @sa     HWA_close()
 */
static void HWA_deleteInstance(HWA_Object *ptrHWADriver)
{
    uint32_t            index  = ptrHWADriver->instanceNum;

    /* un-register the interrupts */
    HwiP_destruct(&ptrHWADriver->hwiHandleParamSet);
    HwiP_destruct(&ptrHWADriver->hwiHandleDone);

    /* free the allocated memory */
   if (ptrHWADriver->interruptCtxParamSet != NULL) {
        ptrHWADriver->interruptCtxParamSet = NULL;
    }

    gHwaObjectPtr[index]=NULL; /* reset the driver's cached handle too */
}


/*!
 *  @brief  Function to handle each paramset completion ISR
 *
 *  @pre    HWA_open() has been called
 *
 *  @param  arg         Internal HWA driver object
 *
 *  @return none
 *
 *  @sa     HWA_enableParamSetInterrupt()
 */
static void HWA_paramDoneISR(void *arg)
{
    HWA_Object          *ptrHWADriver = NULL;
    uint32_t   paramDoneFlag;
    uint32_t            loopCnt;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(arg); /* this fills the ptrHWADriver */

    if ((ptrHWADriver != NULL) && (ptrHWADriver->refCnt > 0U))/*GAP COMMENT ID:7 START END */
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

        /* read the interrupt flag register */
        paramDoneFlag = CSL_FEXTR_64(ctrlBaseAddr->HWACCREG14, 31U, 0U);
        /* clear the interrupt flag by writing 1 to it */
        CSL_FINSR_64(ctrlBaseAddr->HWACCREG15, 31U, 0U, paramDoneFlag);

        /* now process the interrupt */
        for (loopCnt = 0U; (paramDoneFlag != 0U) && (loopCnt < ptrHWADriver->hwAttrs->numHwaParamSets); loopCnt++)/*GAP COMMENT ID:8 START END */
        {
            if ((paramDoneFlag & ((uint32_t)1U << loopCnt)) != 0U)/*GAP COMMENT ID:4 START END */
            {
                HWA_InterruptCtx *interruptCtx = &ptrHWADriver->interruptCtxParamSet[loopCnt];
                if (interruptCtx->callbackFn != NULL)/*GAP COMMENT ID:20 START END */
                {
                    (interruptCtx->callbackFn)(loopCnt, interruptCtx->callbackArg);
                }
                paramDoneFlag &= ~((uint32_t)1U << loopCnt);
            }
        }
    }
    else/*GAP COMMENT ID:22 START */
    {
        /* Throw fatal error as driver is not in valid state */
        DebugP_assert(0U);
    }/*GAP COMMENT ID:22 END */
}


/*!
 *  @brief  Function to handle completion ISR after all paramset are executed in the background thread.
 *
 *  @pre    HWA_open() has been called
 *
 *  @param  arg         Internal HWA driver object
 *
 *  @return none
 *
 *  @sa     HWA_enableDoneInterrupt()
 */
static void HWA_allParamDoneISR(void *arg)
{
    HWA_Object          *ptrHWADriver = NULL;

    HWA_GET_DRIVER_STRUCT(arg); /* this fills the ptrHWADriver */

    if ((ptrHWADriver != NULL) && (ptrHWADriver->refCnt > 0U))/*GAP COMMENT ID:9 START END*/
    {
        /* now process the interrupt */
        HWA_DoneInterruptCtx *interruptCtx = &ptrHWADriver->interruptCtxDone;
        if (interruptCtx->callbackFn != NULL)/*GAP COMMENT ID:10  START END*/
        {
            (interruptCtx->callbackFn)(interruptCtx->callbackArg);
        }
    }
    else/*GAP COMMENT ID:11  START*/
    {
        /* Fatal error as driver is not in valid state */
        DebugP_assert(0U);
    }/*GAP COMMENT ID:11  END*/
}



/*!
 *  @brief  Function to perform the hardware reset of HWA peripheral
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 *  @sa     HWA_close(), HWA_open(), HWA_reset()
 */
static inline void HWA_doReset(DSSHWACCRegs  *ctrlBaseAddr)
{
    CSL_FINSR(ctrlBaseAddr->HWACCREG1, 8U, 6U, 0x7U);
    CSL_FINSR(ctrlBaseAddr->HWACCREG1, 8U, 6U, 0x0U);

    return;
}



/* EXTERNAL FUNCTIONS */


/*!
 *  @brief  Function to initialize the HWA module
 *
 *  @pre    This function must be called once per system and before
 *          any other HWA driver APIs. It resets the HWA H/W instances in the system.
 *
 */
void HWA_init(void)
{
    return;
}

void HWA_deinit(void)
{
    return;
}

/*!
 *  @brief  Function to initialize HWA specified by the
 *  particular index value.
 *
 *  @pre    HWA_init() has been called
 *
 *  @param  index         HWA instance number
 *  @param  hwaCfg        pointer to HWA configuration, if set to NULL, default values will be used.
 *  @param  errCode       [out] valid errorCode if NULL handle returned.
 *
 *  @return A HWA_Handle upon success. NULL if an error occurs.
 *
 *  @sa     HWA_init()
 *  @sa     HWA_close()
 */
extern HWA_Handle HWA_open(uint32_t index, HWA_OpenConfig * hwaCfg, int32_t* errCode)

{
    HWA_Handle          handle = NULL;
    uint32_t            memReqSize = 0U;
    uintptr_t           key;
    int32_t             retCode = 0;
    HwiP_Params         hwiPrms;

    if(index >= gHwaConfigNum)
    {
        retCode = HWA_EOUTOFRANGE;
    }
    else /* start of if valid index */
    {

        /* Disable preemption while opening the driver */
        key = HwiP_disable();

        /*
         * check if driver is already init
         */
        if (gHwaObjectPtr[index] == NULL)
        {
            if (&gHwaObject[index] == NULL)/*GAP COMMENT ID:12  START*/
            {
                /* Error: Out of memory */
                DebugP_log("Debug: HWA Driver (%d) Out of memory (requested size: %d)\n",index,(uint32_t)sizeof(HWA_Object));
                retCode = HWA_EOUTOFMEM;
            }/*GAP COMMENT ID:12  END*/
            else
            {
                /* Allocate memory for the driver and initialize it */
                gHwaObjectPtr[index] = &gHwaObject[index];
                memset ((void *)gHwaObjectPtr[index], 0U, sizeof(HWA_Object));
                gHwaObjectPtr[index]->hwAttrs = &gHwaAttrs[index];
                gHwaObjectPtr[index]->instanceNum = index;

                /* initialize internal driver struture memory */
                /* malloc interruptCtrx */
                memReqSize = (uint32_t)(sizeof(HWA_InterruptCtx) * gHwaObjectPtr[index]->hwAttrs->numHwaParamSets);
                gHwaObjectPtr[index]->interruptCtxParamSet = &HwaParamsetIntr[0];
                memset ((void *)gHwaObjectPtr[index]->interruptCtxParamSet, 0U, memReqSize);
                if (gHwaObjectPtr[index]->interruptCtxParamSet == NULL)/*GAP COMMENT ID:13  START*/
                {
                    /* Error: Out of memory */
                    DebugP_log("Debug: HWA Driver (%d) Out of memory for interruptCtxParamSet(requested size: %d)\n",
                                  index,memReqSize);
                    retCode = HWA_EOUTOFMEM;
                }/*GAP COMMENT ID:13  END*/
                else
                {
                    memset(gHwaObjectPtr[index]->interruptCtxParamSet,0U,memReqSize);
                    /* dont set the handle yet as we dont know if all the mem allocs are going to succeed */
                }
            }
        }

        /*
         * now do config if we got the driver memory allocated and primed
         */
        if ((retCode==0) && (gHwaObjectPtr[index] != NULL) && (gHwaObjectPtr[index]->refCnt==0U))/*GAP COMMENT ID:14  START END*/
        {
            DSSHWACCRegs *ctrlBaseAddr = (DSSHWACCRegs *)gHwaObjectPtr[index]->hwAttrs->ctrlBaseAddr;
            DSSHWACCPARAMRegs *paramBaseAddr = (DSSHWACCPARAMRegs *)(gHwaObjectPtr[index]->hwAttrs->paramBaseAddr);

            /* Register interrupt handlers */
            /* Each Paramset done interrupt */
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args            = gHwaObjectPtr[index];
            hwiPrms.callback        = &HWA_paramDoneISR;
            hwiPrms.intNum          = gHwaObjectPtr[index]->hwAttrs->intNumParamSet;
            if(hwaCfg == NULL)
            {
                hwiPrms.priority    = HWA_PARAMSETDONE_INTERRUPT1_PRIORITY;
            }
            else
            {
                hwiPrms.priority    = hwaCfg->interruptPriority.paramsetDone1;
            }
            hwiPrms.isPulse         = true;
            /* Register interrupts */
            retCode = HwiP_construct(&gHwaObjectPtr[index]->hwiHandleParamSet, &hwiPrms);
            if(SystemP_SUCCESS != retCode)/*GAP COMMENT ID:17  START */
            {
                DebugP_log("Error Could not register ISR !!!\n");
            }/*GAP COMMENT ID:17  END */

            /* All programmed Paramsets HWA done interrupt */
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args            = gHwaObjectPtr[index];
            hwiPrms.callback        = &HWA_allParamDoneISR;
            hwiPrms.intNum          = gHwaObjectPtr[index]->hwAttrs->intNumDone;
            if(hwaCfg == NULL)/*GAP COMMENT ID: 15_1  START END*/
            {
                hwiPrms.priority    = HWA_DONE_INTERRUPT_PRIORITY;
            }
            else/*GAP COMMENT ID:16_1   START*/
            {
                hwiPrms.priority    = hwaCfg->interruptPriority.backgroundDone;
            }/*GAP COMMENT ID:16_1  END*/
            hwiPrms.isPulse         = true;
            /* Register interrupts */
            retCode = HwiP_construct(&gHwaObjectPtr[index]->hwiHandleDone, &hwiPrms);
            if(SystemP_SUCCESS != retCode)/*GAP COMMENT ID:17_1  START*/
            {
                DebugP_log("Error Could not register ISR !!!\n");
            }/*GAP COMMENT ID:17_1  END*/

            {
                //uint32_t * dbgAckCtl1Ptr;
                //CSL_dss_ctrlRegs        *dssCtrlRegsPtr;

                /* unlock the HWA registers */
                //ctrlBaseAddr->LOCK0_KICK0 = 0x01234567U;
                //ctrlBaseAddr->LOCK0_KICK1 = 0xFEDCBA8U;


                /* set DBG_ACK_CTL1_DSS_HWA in DSS_CTRL:DBG_ACK_CTL1*/
                //dbgAckCtl1Ptr = (uint32_t *) (gHwaObjectPtr[index]->hwAttrs->dssBaseAddr + 0x58C);
                //*dbgAckCtl1Ptr |= 0x10000000;   /* set bit 28- 30 (DBG_ACK_CTL1_DSS_HWA) to 1*/
                //dssCtrlRegsPtr  = (CSL_dss_ctrlRegs *) gHwaObjectPtr[index]->hwAttrs->dssBaseAddr;
                //CSL_FINSR(dssCtrlRegsPtr->DBG_ACK_CTL1,30U,28U, 1U);

                /*disable accelerator and clock*/
                ctrlBaseAddr->HWACCREG1 = 0x0U;

                /*now reset the hwa*/
                HWA_doReset(ctrlBaseAddr);
                /*leave hw_acc disabled but enable the clock*/
                ctrlBaseAddr->HWACCREG1 = 0x38U;

                /* 1 bit dynamic clock  */

                CSL_FINSR(ctrlBaseAddr->HWACCREG1,
                          HWACCREG1_ACCDYNCLKEN_BIT_END,
                          HWACCREG1_ACCDYNCLKEN_BIT_START,
                          0x1U);


                /*TODO: check if needed for real silicon*/
                /* reset paramset */
                 memset((void *)paramBaseAddr, 0U, sizeof(DSSHWACCPARAMRegs)*gHwaObjectPtr[index]->hwAttrs->numHwaParamSets);
                 /* clear the PARAMDONESTAT */
                 ctrlBaseAddr->HWACCREG15 = 0xFFFFFFFFU;
                 /* clear the ACC_TRIGGER_IN_STAT*/
                 CSL_FINSR(ctrlBaseAddr->HWACCREG12,
                          HWACCREG12_ACC_TRIGGER_IN_CLR_BIT_END,
                          HWACCREG12_ACC_TRIGGER_IN_CLR_BIT_START,
                          0x1U);
                 /* clear the FFTCLIP register */
                 CSL_FINSR(ctrlBaseAddr->FFTCLIP,
                          FFTCLIP_CLRFFTCLIPSTAT_BIT_END,
                          FFTCLIP_CLRFFTCLIPSTAT_BIT_START,
                          0x1U);

                 /* Initialize HWA peripheral memories */
                 CSL_FINSR(ctrlBaseAddr->MEMINIT, 7U, 0U, 0xFFU);

                 /* clear other clip registers*/
                 CSL_FINSR(ctrlBaseAddr->CLR_MISC_CLIP,
                          CLR_MISC_CLIP_CLR_MISC_CLIP_BIT_END,
                          CLR_MISC_CLIP_CLR_MISC_CLIP_BIT_START,
                          0x1U);
            }
        }

        /*
         * check before returning
         */
        if (retCode == 0)/*GAP COMMENT ID:18  START END*/
        {
            /* increment reference count */
            gHwaObjectPtr[index]->refCnt++;
            /* Setup the return handle: */
            handle = (void *)gHwaObjectPtr[index];
        }
        else/*GAP COMMENT ID:19  START */
        {
            handle = NULL;
            /* errCode is already set */
            HWA_deleteInstance(gHwaObjectPtr[index]);
        }/*GAP COMMENT ID:19  END */

        /* Restore the interrupts: */
        HwiP_restore(key);

    }/* end of if valid index */

    /* return */
    if (errCode!=NULL)
    {
        *errCode = retCode;
    }
    return handle;
}


/*!
 *  @brief  Function to close a HWA peripheral specified by the HWA handle
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle      A HWA_Handle returned from HWA_open()
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_close(HWA_Handle handle)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    uintptr_t           key;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* Disable preemption while opening the driver */
    key = HwiP_disable();

    if (ptrHWADriver!=NULL)
    {
        if ((ptrHWADriver->configInProgress == 0U) && (ptrHWADriver->paramSetMapInProgress == 0U))
        {
            /* decrement the refCount and if this is the last reference, delete the instance */
            if (ptrHWADriver->refCnt>0U)
            {
                ptrHWADriver->refCnt--;
            }
            if (ptrHWADriver->refCnt==0U)
            {
                //uint32_t *mssToprcmRegPtr;
                ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
                /* disable the HWA state machine */
                CSL_FINSR(ctrlBaseAddr->HWACCREG1, 2U, 0U, 0x0U);
                /* do hardware reset */
                HWA_doReset(ctrlBaseAddr);
                /* what else to reset??, lock the register  */
                //ctrlBaseAddr->LOCK0_KICK0 = 0x0;
                //ctrlBaseAddr->LOCK0_KICK1 = 0x0;

                /* unlock the param mem */
                //mssToprcmRegPtr = (uint32_t *)0x02141008U;  //TPR:MSS_TOPRCM:LOCK0_KICK0 physical address
                //*mssToprcmRegPtr = 0x0;
                //mssToprcmRegPtr = (uint32_t *)0x0214100CU;  //TPR:MSS_TOPRCM:LOCK1_KICK1 physical address
                //*mssToprcmRegPtr = 0x0;
              //  mssToprcmRegPtr = (uint32_t *)0x02140044;  //TPR:MSS_TOPRCM:SYS_CLK_DIV_VAL
              //  *mssToprcmRegPtr = 0x111;


                /* delete the instance */
                HWA_deleteInstance(ptrHWADriver);
            }
        }
        else
        {
            /* return inuse code */
            retCode = HWA_EINUSE;
        }
    }
    else
    {
        /* return invalid code */
        retCode = HWA_EINVAL;
    }
    /* Restore the interrupts */
    HwiP_restore(key);

    return retCode;
}


/*!
 *  @brief  Function to reset the internal state machine of the HWA
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */

int32_t HWA_reset(HWA_Handle handle)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        HWA_doReset(ctrlBaseAddr);
        HWA_releaseDriverAccess(ptrHWADriver,(bool)true,(bool)false,0);
    }

    /* return */
    return retCode;
}



/*!
 *  @brief  Function to set the common HWA configuration parameters
 *          needed for the next operations/iterations/paramsets of HWA
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  commonConfig    HWA Common Config Parameters
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_configCommon(HWA_Handle handle, HWA_CommonConfig *commonConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCRegs        *ctrlBaseAddr;
    uint8_t             ii;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (commonConfig == NULL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else if (
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_NUMLOOPS) != 0U) && (commonConfig->numLoops > 4095U)) ||
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_PARAMSTARTIDX) != 0U) && (commonConfig->paramStartIdx > (SOC_HWA_NUM_PARAM_SETS - 1U))) ||
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_PARAMSTOPIDX)!=0U) && (commonConfig->paramStopIdx > (SOC_HWA_NUM_PARAM_SETS - 1U))) ||
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_FFT1DENABLE)!=0U) && (commonConfig->fftConfig.fft1DEnable > HWA_FEATURE_BIT_ENABLE)) ||
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_BPMRATE)!=0U) && (commonConfig->fftConfig.bpmRate > (1U << 10U))) ||
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE)!=0U) && (commonConfig->fftConfig.twidDitherEnable > HWA_FEATURE_BIT_ENABLE)) ||
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_LFSRSEED)!=0U) && (commonConfig->fftConfig.lfsrSeed > (1U << 29U))) ||
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_FFTSUMDIV)!=0U) && (commonConfig->fftConfig.fftSumDiv > (1U << 5U)))
            )

        {
            /* invalid config params */
            retCode = HWA_EINVAL;
        }
        else if (
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFARTHRESHOLDSCALE)!=0U) && (commonConfig->cfarConfig.cfarThresholdScale > (1U << 18U)))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_CFAR;
        }
        else if (
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFAR_DET_THR)!=0U) && (commonConfig->cfarDetThresConfig.cfarDetthreshold > (1U << 24U)))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_CFAR_DET_THR;
        }
        
        else if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) != 0U)
        {
            int idx;
			if (ptrHWADriver->hwAttrs->isCompressionEnginePresent == false)/*GAP COMMENT ID:21  START */
			{
				retCode = HWA_ENOTSUPP;
			}/*GAP COMMENT ID:21  END */
			else
			{
            for (idx = 0; idx < HWA_CMP_K_ARR_LEN; idx++)
            {
	                if (commonConfig->compressMode.EGEKparam[idx] > 31U)
                {
					/* invalid config params */
                    retCode = HWA_EINVAL; break;
                }
            }
        }
        }
        else if (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_I_CMULT_SCALE)!=0U) &&
                 ((commonConfig->scalarMult.i_cmult_scale[0] > (1U << 21U)) || (commonConfig->scalarMult.i_cmult_scale[1] > (1U << 21U)) || (commonConfig->scalarMult.i_cmult_scale[2] > (1U << 21U)) ||
                  (commonConfig->scalarMult.i_cmult_scale[3] > (1U << 21U)) || (commonConfig->scalarMult.i_cmult_scale[4] > (1U << 21U)) || (commonConfig->scalarMult.i_cmult_scale[5] > (1U << 21U))))                                                                          
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_COMPLEXMULT;
        }
        else if (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_Q_CMULT_SCALE) != 0U) &&
                 ((commonConfig->scalarMult.q_cmult_scale[0] > (1U << 21U)) || (commonConfig->scalarMult.q_cmult_scale[1] > (1U << 21U)) || (commonConfig->scalarMult.q_cmult_scale[2] > (1U << 21U)) ||
                  (commonConfig->scalarMult.q_cmult_scale[3] > (1U << 21U)) || (commonConfig->scalarMult.q_cmult_scale[4] > (1U << 21U)) || (commonConfig->scalarMult.q_cmult_scale[5] > (1U << 21U))))                                                                          
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_COMPLEXMULT;
        }
        else if (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT)!= 0U)&&
                 ((commonConfig->dcEstimateConfig.scale > (1U << 9U)) || (commonConfig->dcEstimateConfig.shift > 14U)))
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_DCEST;
        }
        else if (
            (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAG)!=0U) || ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAGDIFF)!=0U)) && 
            ((commonConfig->interfConfig.sumMagScale > 255U) || (commonConfig->interfConfig.sumMagShift > 13) || (commonConfig->interfConfig.sumMagDiffScale > 255U) || (commonConfig->interfConfig.sumMagDiffShift > 13))/*GAP COMMENT ID: 23 START*/
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_INTERFERENCE;
        }/*GAP COMMENT ID: 23 END*/
        else
        {
            /*MISRA-C*/
        }

        if (retCode == 0)
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_NUMLOOPS) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG16, 11U, 0U, commonConfig->numLoops);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_PARAMSTARTIDX) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG16, 16U, 12U, commonConfig->paramStartIdx);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_PARAMSTOPIDX) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG16, 21U, 17U, commonConfig->paramStopIdx);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_FFT1DENABLE) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG1, 10U, 10U, commonConfig->fftConfig.fft1DEnable);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_BPMRATE) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG7, 9U, 0U, commonConfig->fftConfig.bpmRate);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_BPMPATTERN) != 0U)
            {
                ctrlBaseAddr->HWACCREG6 = commonConfig->fftConfig.bpmPattern[0];
                ctrlBaseAddr->HWACCREG5 = commonConfig->fftConfig.bpmPattern[1];
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG7, 16U, 16U, commonConfig->fftConfig.twidDitherEnable);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_LFSRSEED) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG11, 28U, 0U, commonConfig->fftConfig.lfsrSeed);
                /* Pulse (set and reset)single register-bit REG_LFSRLOAD */
                CSL_FINSR(ctrlBaseAddr->HWACCREG11, 31U, 31U, 1U);
                CSL_FINSR(ctrlBaseAddr->HWACCREG11, 31U, 31U, 0U);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_FFTSUMDIV) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG8, 28U, 24U, commonConfig->fftConfig.fftSumDiv);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFARTHRESHOLDSCALE) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG13, 17U, 0U, commonConfig->cfarConfig.cfarThresholdScale);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFAR_DET_THR) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->CFAR_DET_THR, 23U, 0U, commonConfig->cfarDetThresConfig.cfarDetthreshold);
            }
            /* set DCEST scale and shift */
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->DCEST_SCALE, 8U, 0U, commonConfig->dcEstimateConfig.scale);
                CSL_FINSR(ctrlBaseAddr->DCEST_SHIFT, 3U, 0U, commonConfig->dcEstimateConfig.shift);
            }
            /* if DC SUB is enabled, and DCSUB_SELECT is set to 0, set the pramgrammed DC values used in subtraction */
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_DCSUB_SWVAL) != 0U)
            {
                for (ii = 0; ii < 6U; ii++)
                {
                    CSL_FINSR(ctrlBaseAddr->DCEST_I_SW[ii], 23U, 0U, (uint32_t)commonConfig->dcSubtractConfig.swIVal[ii]);
                    CSL_FINSR(ctrlBaseAddr->DCEST_Q_SW[ii], 23U, 0U, (uint32_t)commonConfig->dcSubtractConfig.swQVal[ii]);
                }
            }

            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_I_CMULT_SCALE) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE1I, 20U, 0U, commonConfig->scalarMult.i_cmult_scale[0]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE2I, 20U, 0U, commonConfig->scalarMult.i_cmult_scale[1]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE3I, 20U, 0U, commonConfig->scalarMult.i_cmult_scale[2]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE4I, 20U, 0U, commonConfig->scalarMult.i_cmult_scale[3]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE5I, 20U, 0U, commonConfig->scalarMult.i_cmult_scale[4]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE6I, 20U, 0U, commonConfig->scalarMult.i_cmult_scale[5]);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_Q_CMULT_SCALE) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE1Q, 20U, 0U, commonConfig->scalarMult.q_cmult_scale[0]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE2Q, 20U, 0U, commonConfig->scalarMult.q_cmult_scale[1]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE3Q, 20U, 0U, commonConfig->scalarMult.q_cmult_scale[2]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE4Q, 20U, 0U, commonConfig->scalarMult.q_cmult_scale[3]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE5Q, 20U, 0U, commonConfig->scalarMult.q_cmult_scale[4]);
                CSL_FINSR(ctrlBaseAddr->CMULTSCALE6Q, 20U, 0U, commonConfig->scalarMult.q_cmult_scale[5]);
            }
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123, 4U, 0U,commonConfig->compressMode.EGEKparam[0]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123,12U, 8U,commonConfig->compressMode.EGEKparam[1]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123,20U,16U,commonConfig->compressMode.EGEKparam[2]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123,28U,24U,commonConfig->compressMode.EGEKparam[3]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567,  4U,  0U, commonConfig->compressMode.EGEKparam[4]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567, 12U,  8U, commonConfig->compressMode.EGEKparam[5]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567, 20U, 16U, commonConfig->compressMode.EGEKparam[6]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567, 28U, 24U, commonConfig->compressMode.EGEKparam[7]);
            }

            /* set the interference threshold for magnitude */
            if((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFMAG_THRESHOLD) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->INTF_MAGTHRESH1_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagSw[0]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGTHRESH2_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagSw[1]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGTHRESH3_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagSw[2]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGTHRESH4_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagSw[3]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGTHRESH5_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagSw[4]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGTHRESH6_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagSw[5]);
            }

            /* set the interference threshold for matnitude diff */
            if((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFMAGDIFF_THRESHOLD) != 0U)
            {
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFFTHRESH1_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagDiffSw[0]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFFTHRESH2_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagDiffSw[1]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFFTHRESH3_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagDiffSw[2]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFFTHRESH4_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagDiffSw[3]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFFTHRESH5_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagDiffSw[4]);
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFFTHRESH6_SW, 23U, 0U, commonConfig->interfConfig.thresholdMagDiffSw[5]);
            }


            /* set the interference sum magn unsigned scaler */
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAG) != 0U)
            {
                /* magSum scale */
                CSL_FINSR(ctrlBaseAddr->INTF_MAG_SCALE, 7U, 0U, commonConfig->interfConfig.sumMagScale);
                /* magSum shift*/
                CSL_FINSR(ctrlBaseAddr->INTF_MAG_SHIFT, 3U, 0U, (uint32_t)commonConfig->interfConfig.sumMagShift);
            }

            /* set the interference sum magn diff unsigned scaler */
            if ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAGDIFF) != 0U)
            {
                /* magDiffSum scale */
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFF_SCALE, 7U, 0U, commonConfig->interfConfig.sumMagDiffScale);
                /* magDiffSum shift*/
                CSL_FINSR(ctrlBaseAddr->INTF_MAGDIFF_SHIFT, 3U, 0U, (uint32_t)commonConfig->interfConfig.sumMagDiffShift);
            }

        }
        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}

DSSHWACCRegs *HWA_getCommonCtrlAddr(HWA_Handle handle)
{
    int32_t             retCode = 0;
    DSSHWACCRegs       *ctrlBaseAddr = NULL;
    HWA_Object         *ptrHWADriver;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if(retCode==0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)(ptrHWADriver->hwAttrs->ctrlBaseAddr);
    }
    HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);

    return (ctrlBaseAddr);
}

DSSHWACCPARAMRegs *HWA_getParamSetAddr(HWA_Handle handle, uint8_t paramsetIdx)
{
    int32_t             retCode = 0;
    DSSHWACCPARAMRegs  *paramBaseAddr = NULL;
    HWA_Object         *ptrHWADriver;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) false, (bool) true, paramsetIdx);
    if(retCode==0)
    {
        paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr + (paramsetIdx * sizeof(DSSHWACCPARAMRegs)));
    }
    HWA_releaseDriverAccess(ptrHWADriver, (bool) false, (bool) true, paramsetIdx);

    return (paramBaseAddr);
}

/*!
 *  @brief  Function to set the HWA configuration parameters for a given paramSet
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  paramsetIdx     A valid paramSet index for which the paramConfig is provided.
 *
 *  @param  paramConfig     HWA ParamSet Config Parameters
 *
 *  @param  dmaConfig       [out] This parameter is set by the driver with values that user
 *                                should use to program the source trigger DMA. user should provide
 *                                a valid buffer here if the triggerMode is set to DMA in paramConfig
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_configParamSet(HWA_Handle handle, uint8_t paramsetIdx, HWA_ParamConfig *paramConfig, HWA_SrcDMAConfig *dmaConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    uint32_t *paramBaseAddr;
    DSSHWACCRegs *ctrlBaseAddr;
    DSSHWACCPARAMRegs paramReg = { 0 };

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, false, true, paramsetIdx);
    if (retCode == 0)
    {
        /* validate the paramConfig */
        retCode = HWA_validateParamSetConfig(ptrHWADriver, paramConfig);
        if (retCode == 0)
        {
            /* valid driver access and valid paramConfig */
            paramBaseAddr = (uint32_t *)(ptrHWADriver->hwAttrs->paramBaseAddr + (paramsetIdx * sizeof(DSSHWACCPARAMRegs)));

            /* Get paramset interrupt settings which will be preserved */
            paramReg.PARAMn_0 = CSL_FEXTR(*paramBaseAddr, 12U, 7U) << 7U;

            /* All register start with value 0 except PARAMn_0 */
            paramReg.PARAMn_0 |= CSL_FMKR(2U, 0U, paramConfig->triggerMode);
            if (paramConfig->triggerMode == HWA_TRIG_MODE_DMA)
            {
                paramReg.PARAMn_0 |= CSL_FMKR(6U, 3U, paramConfig->dmaTriggerSrc);
                /* provide the user with the DMA programming config to facilitate the DMA triggering of HWA */
                if (dmaConfig != NULL)
                {
                    ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

                    dmaConfig->destAddr = (uint32_t)(&ctrlBaseAddr->HWACCREG2);
                    dmaConfig->srcAddr = (uint32_t)(&ctrlBaseAddr->SIGDMACHxDONE[paramConfig->dmaTriggerSrc]);
                    dmaConfig->aCnt = sizeof(uint32_t);
                    dmaConfig->bCnt = 1;
                    dmaConfig->cCnt = 1;
                }
            }

            paramReg.PARAMn_0 |= CSL_FMKR(23U, 21U, paramConfig->accelMode);
            paramReg.PARAMn_1 |= CSL_FMKR(15U, 0U, paramConfig->source.srcAddr);
            paramReg.PARAMn_2 |= CSL_FMKR(11U, 0U, paramConfig->source.srcAcnt);
            paramReg.PARAMn_3 |= CSL_FMKR(15U, 0U, (uint32_t)paramConfig->source.srcAIdx);
            paramReg.PARAMn_5 |= CSL_FMKR(11U, 0U, paramConfig->source.srcBcnt);
            paramReg.PARAMn_4 |= CSL_FMKR(15U, 0U, (uint32_t)paramConfig->source.srcBIdx);
            paramReg.PARAMn_7 |= CSL_FMKR(11U, 0U, paramConfig->source.srcShift);
            paramReg.PARAMn_7 |= CSL_FMKR(31U, 28U, paramConfig->source.srcCircShiftWrap);
            paramReg.PARAMn_2 |= CSL_FMKR(12U, 12U, paramConfig->source.srcRealComplex);
            paramReg.PARAMn_2 |= CSL_FMKR(13U, 13U, paramConfig->source.srcWidth);
            paramReg.PARAMn_2 |= CSL_FMKR(14U, 14U, paramConfig->source.srcSign);
            paramReg.PARAMn_2 |= CSL_FMKR(15U, 15U, paramConfig->source.srcConjugate);
            paramReg.PARAMn_5 |= CSL_FMKR(15U, 12U, paramConfig->source.srcScale);
            paramReg.PARAMn_0 |= CSL_FMKR(13U, 13U, paramConfig->source.bpmEnable);
            paramReg.PARAMn_6 |= CSL_FMKR(11U, 8U, paramConfig->source.bpmPhase);

            paramReg.PARAMn_1 |= CSL_FMKR(31U, 16U, paramConfig->dest.dstAddr);
            paramReg.PARAMn_2 |= CSL_FMKR(27U, 16U, paramConfig->dest.dstAcnt);
            paramReg.PARAMn_3 |= CSL_FMKR(31U, 16U, (uint32_t)paramConfig->dest.dstAIdx);
            paramReg.PARAMn_4 |= CSL_FMKR(31U, 16U, (uint32_t)paramConfig->dest.dstBIdx);
            paramReg.PARAMn_2 |= CSL_FMKR(28U, 28U, paramConfig->dest.dstRealComplex);
            paramReg.PARAMn_2 |= CSL_FMKR(29U, 29U, paramConfig->dest.dstWidth);
            paramReg.PARAMn_2 |= CSL_FMKR(30U, 30U, paramConfig->dest.dstSign);
            paramReg.PARAMn_2 |= CSL_FMKR(31U, 31U, paramConfig->dest.dstConjugate);
            paramReg.PARAMn_5 |= CSL_FMKR(19U, 16U, paramConfig->dest.dstScale);
            paramReg.PARAMn_5 |= CSL_FMKR(29U, 20U, paramConfig->dest.dstSkipInit);

            if (paramConfig->accelMode == HWA_ACCELMODE_FFT)
            {
                paramReg.PARAMn_0 |= CSL_FMKR(14U, 14U, paramConfig->accelModeArgs.fftMode.fftEn);
                paramReg.PARAMn_6 |= CSL_FMKR(5U, 2U, paramConfig->accelModeArgs.fftMode.fftSize);
                paramReg.PARAMn_6 |= CSL_FMKR(31U, 22U, paramConfig->accelModeArgs.fftMode.butterflyScaling);
                paramReg.PARAMn_6 |= CSL_FMKR(7U, 7U, paramConfig->accelModeArgs.fftMode.interfZeroOutEn);
                paramReg.PARAMn_0 |= CSL_FMKR(15U, 15U, paramConfig->accelModeArgs.fftMode.windowEn);
                paramReg.PARAMn_6 |= CSL_FMKR(21U, 12U, paramConfig->accelModeArgs.fftMode.windowStart);
                paramReg.PARAMn_6 |= CSL_FMKR(6U, 6U, paramConfig->accelModeArgs.fftMode.winSymm);
                paramReg.PARAMn_7 |= CSL_FMKR(27U, 26U, paramConfig->accelModeArgs.fftMode.winInterpolateMode);
                paramReg.PARAMn_0 |= CSL_FMKR(17U, 16U, paramConfig->accelModeArgs.fftMode.magLogEn);
                paramReg.PARAMn_0 |= CSL_FMKR(25U, 24U, paramConfig->accelModeArgs.fftMode.fftOutMode);
                paramReg.PARAMn_0 |= CSL_FMKR(27U, 26U, paramConfig->preProcCfg.interfLocalize.thresholdSelect);
                paramReg.PARAMn_0 |= CSL_FMKR(29U, 28U, paramConfig->preProcCfg.interfStat.resetMode);
                paramReg.PARAMn_6 |= CSL_FMKR(1U, 0U, paramConfig->preProcCfg.interfLocalize.thresholdMode);
                paramReg.PARAMn_6 |= CSL_FMKR(7U, 7U, paramConfig->preProcCfg.interfLocalize.thresholdEnable);
                if (paramConfig->accelModeArgs.fftMode.fftOutMode != HWA_FFT_MODE_OUTPUT_DEFAULT)
                {
                    /* reset the destination parameters if the output is statistics */ 
                    paramReg.PARAMn_2 |= CSL_FMKR(27U, 16U, 4095U);
                    paramReg.PARAMn_3 |= CSL_FMKR(31U, 16U, 8U);
                    paramReg.PARAMn_4 |= CSL_FMKR(31U, 16U, 8U);
                    paramReg.PARAMn_2 |= CSL_FMKR(28U, 28U, 0U);
                    paramReg.PARAMn_2 |= CSL_FMKR(29U, 29U, 1U);
                }
            }

            if (paramConfig->accelMode == HWA_ACCELMODE_CFAR)
            {
                paramReg.PARAMn_7 |= CSL_FMKR(25U, 20U, paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft);
                paramReg.PARAMn_7 |= CSL_FMKR(19U, 14U, paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight);
                paramReg.PARAMn_6 |= CSL_FMKR(14U, 12U, paramConfig->accelModeArgs.cfarMode.numGuardCells);
                paramReg.PARAMn_6 |= CSL_FMKR(18U, 15U, paramConfig->accelModeArgs.cfarMode.nAvgDivFactor);
                paramReg.PARAMn_7 |= CSL_FMKR(13U, 12U, paramConfig->accelModeArgs.cfarMode.nAvgMode);
                switch (paramConfig->accelModeArgs.cfarMode.operMode)
                {
                case HWA_CFAR_OPER_MODE_LOG_INPUT_REAL:           //0xC     // CFAR_LOG_MODE=1, CFAR_INP_MODE=1, CFAR_ABS_MODE=dont care    1100
                    paramReg.PARAMn_0 |= CSL_FMKR(19U, 16U, 0xCU);
                    break;
                case HWA_CFAR_OPER_MODE_LOG_INPUT_COMPLEX:        //0x7     // CFAR_LOG_MODE=1, CFAR_INP_MODE=0, CFAR_ABS_MODE=3            0111
                    paramReg.PARAMn_0 |= CSL_FMKR(19U, 16U, 0x7U);
                    break;
                case HWA_CFAR_OPER_MODE_MAG_INPUT_REAL:           //0x8     // CFAR_LOG_MODE=0, CFAR_INP_MODE=1, CFAR_ABS_MODE=dont care    1000
                    paramReg.PARAMn_0 |= CSL_FMKR(19U, 16U, 0x8U);
                    break;
                case HWA_CFAR_OPER_MODE_MAG_INPUT_COMPLEX:        //0x2     // CFAR_LOG_MODE=0, CFAR_INP_MODE=0, CFAR_ABS_MODE=2            0010
                    paramReg.PARAMn_0 |= CSL_FMKR(19U, 16U, 0x2U);
                    break;
                case HWA_CFAR_OPER_MODE_MAG_SQR_INPUT_REAL:       //0x8     // CFAR_LOG_MODE=0, CFAR_INP_MODE=1, CFAR_ABS_MODE=dont care    1000
                    paramReg.PARAMn_0 |= CSL_FMKR(19U, 16U, 0x8U);
                    break;
                case HWA_CFAR_OPER_MODE_MAG_SQR_INPUT_COMPLEX:    //0x0     // CFAR_LOG_MODE=0, CFAR_INP_MODE=0, CFAR_ABS_MODE=0            0000
                    paramReg.PARAMn_0 |= CSL_FMKR(19U, 16U, 0x0U);
                    break; 
                case HWA_CFAR_OPER_MODE_LOG_INPUT_COMPLEX_LINEARCFAR:  //0x // CFAR_LOG_MODE=0, CFAR_INP_MODE=0, CFAR_ABS_MODE=3            0011
                    paramReg.PARAMn_0 |= CSL_FMKR(19U, 16U, 0x3U);
                    break; 
                default:/*GAP COMMENT ID: 24 START*/
                    break;/*GAP COMMENT ID: 24 END*/
                }
                paramReg.PARAMn_7 |= CSL_FMKR(27U, 26U, paramConfig->accelModeArgs.cfarMode.outputMode);
                paramReg.PARAMn_0 |= CSL_FMKR(15U, 15U, paramConfig->accelModeArgs.cfarMode.peakGroupEn);
                paramReg.PARAMn_0 |= CSL_FMKR(20U, 20U, paramConfig->accelModeArgs.cfarMode.cyclicModeEn);
                paramReg.PARAMn_6 |= CSL_FMKR(5U, 0U, paramConfig->accelModeArgs.cfarMode.cfarOsKvalue);
                paramReg.PARAMn_6 |= CSL_FMKR(6U, 6U, paramConfig->accelModeArgs.cfarMode.cfarOsEdgeKScaleEn);
            }

            if (paramConfig->accelMode == HWA_ACCELMODE_COMPRESS)
            {
				if (ptrHWADriver->hwAttrs->isCompressionEnginePresent == true)/*GAP COMMENT ID: 26 START END*/
				{
					paramReg.PARAMn_6 |= CSL_FMKR(3U,3U,paramConfig->accelModeArgs.compressMode.compressDecompress);
					paramReg.PARAMn_6 |= CSL_FMKR(6U,4U,paramConfig->accelModeArgs.compressMode.method);
					paramReg.PARAMn_6 |= CSL_FMKR(2U,2U,paramConfig->accelModeArgs.compressMode.ditherEnable);
					paramReg.PARAMn_6 |= CSL_FMKR(22U,21U,paramConfig->accelModeArgs.compressMode.passSelect);
					paramReg.PARAMn_6 |= CSL_FMKR(20U,20U,paramConfig->accelModeArgs.compressMode.headerEnable);
					paramReg.PARAMn_6 |= CSL_FMKR(19U, 16U, paramConfig->accelModeArgs.compressMode.scaleFactorBW);
					paramReg.PARAMn_6 |= CSL_FMKR(10U, 7U, paramConfig->accelModeArgs.compressMode.EGEKarrayLength);

                    paramReg.PARAMn_6   |= CSL_FMKR(31U, 27U, paramConfig->accelModeArgs.compressMode.scaleFactor);
					paramReg.PARAMn_6   |= CSL_FMKR(15U, 11U, paramConfig->accelModeArgs.compressMode.BFPMantissaBW);
				}
				else/*GAP COMMENT ID: 27 START*/
				{
					retCode = HWA_EINVAL;
				}/*GAP COMMENT ID: 27 START END*/
            }

            if ((paramConfig->accelMode == HWA_ACCELMODE_FFT) || (paramConfig->accelMode == HWA_ACCELMODE_NONE))
            {
                if (paramConfig->complexMultiply.mode <= HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT_2)
                {
                    paramReg.PARAMn_0 |= CSL_FMKR(20U, 18U, paramConfig->complexMultiply.mode);

                    if (paramConfig->complexMultiply.mode == HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER)
                    {
                        paramReg.PARAMn_7 |= CSL_FMKR(25U, 12U, paramConfig->complexMultiply.cmpMulArgs.twidIncrement);
                    }
                    if (paramConfig->complexMultiply.mode == HWA_COMPLEX_MULTIPLY_MODE_SLOW_DFT)
                    {
                        paramReg.PARAMn_7 |= CSL_FMKR(25U, 12U, paramConfig->complexMultiply.cmpMulArgs.dft.startFreq);
                        paramReg.PARAMn_6 |= CSL_FMKR(5U, 2U, paramConfig->complexMultiply.cmpMulArgs.dft.freqIncrement);
                    }
                    if (paramConfig->complexMultiply.mode == HWA_COMPLEX_MULTIPLY_MODE_FFT_STITCHING)
                    {
                        paramReg.PARAMn_7 |= CSL_FMKR(13U, 12U, paramConfig->complexMultiply.cmpMulArgs.twidFactorPattern);
                    }
                    if (paramConfig->complexMultiply.mode == HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT)
                    {
                        paramReg.PARAMn_7 |= CSL_FMKR(25U, 12U, paramConfig->complexMultiply.cmpMulArgs.twidIncrement);
                    }
                    if (paramConfig->complexMultiply.mode == HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT_2)
                    {
                        paramReg.PARAMn_7 |= CSL_FMKR(25U, 12U, paramConfig->complexMultiply.cmpMulArgs.twidIncrement);
                    }
                }

                if (paramConfig->preProcCfg.dcEstResetMode <= HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET_ZEROLPCONT)/*GAP COMMENT ID: 28 START END*/
                {
                    paramReg.PARAMn_0 |= CSL_FMKR(31U, 30U, paramConfig->preProcCfg.dcEstResetMode);
                    paramReg.PARAMn_5 |= CSL_FMKR(31U, 31U, paramConfig->preProcCfg.dcSubEnable);
                    paramReg.PARAMn_5 |= CSL_FMKR(30U, 30U, paramConfig->preProcCfg.dcSubSelect);       
                }
            }

            /* All register is constructed, write the value to hardware */
            //memcpy((void *)paramBaseAddr, (void *)&paramReg, sizeof(DSSHWACCPARAMRegs)); /*TODO: Debug memcpy at asembly level to understand the design level limitation in doing this reg write */
            
            HW_WR_REG32(paramBaseAddr,(uint32_t) paramReg.PARAMn_0);
            HW_WR_REG32((paramBaseAddr + 0x1U),(uint32_t) paramReg.PARAMn_1);
            HW_WR_REG32((paramBaseAddr + 0x2U),(uint32_t) paramReg.PARAMn_2);
            HW_WR_REG32((paramBaseAddr + 0x3U),(uint32_t) paramReg.PARAMn_3);
            HW_WR_REG32((paramBaseAddr + 0x4U),(uint32_t) paramReg.PARAMn_4);
            HW_WR_REG32((paramBaseAddr + 0x5U),(uint32_t) paramReg.PARAMn_5);
            HW_WR_REG32((paramBaseAddr + 0x6U),(uint32_t) paramReg.PARAMn_6);
            HW_WR_REG32((paramBaseAddr + 0x7U),(uint32_t) paramReg.PARAMn_7);
        }

        HWA_releaseDriverAccess(ptrHWADriver, false, true, paramsetIdx);
    }

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to get the config to program the DMA for a given DMA Trigger channel.
 *          Application should use the returned config to program the DMA so that it can then
 *          in turn trigger the paramset. Application should make sure that the channel provided
 *          here in dmaTriggerSrc should match the \ref HWA_ParamConfig_t::dmaTriggerSrc passed
 *          to HWA_configParamSet()
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  dmaTriggerSrc   Same as \ref HWA_ParamConfig_t::dmaTriggerSrc of the paramset for
 *                          whom this DMA is getting configured
 *
 *  @param  dmaConfig       [out]This parameter is set by the driver with values that user
 *                               should use to program the source trigger DMA. user should provide
 *                               a valid buffer here if the triggerMode is set to DMA in paramConfig
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getDMAconfig(HWA_Handle handle, uint8_t dmaTriggerSrc, HWA_SrcDMAConfig *dmaConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCRegs *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if ((dmaTriggerSrc > (ptrHWADriver->hwAttrs->numDmaChannels -1U)) || (dmaConfig == NULL))
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            dmaConfig->destAddr = (uint32_t)(&ctrlBaseAddr->HWACCREG2);
            /* Read signatures from local core memory instead of HWA registers */
            dmaConfig->srcAddr = (uint32_t) &ctrlBaseAddr->SIGDMACHxDONE[dmaTriggerSrc];
            dmaConfig->aCnt = sizeof(uint32_t);
            dmaConfig->bCnt = 1U;
            dmaConfig->cCnt = 1U;
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    return(retCode);
}

/*!
 *  @brief  Function to get HWA processing Memory information including address,
 *          size and number of banks.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  memInfo         Pointer to save HWA processing memory information
 *
  *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getHWAMemInfo(HWA_Handle handle, HWA_MemInfo *memInfo)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (memInfo == NULL)
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            memInfo->baseAddress    = ptrHWADriver->hwAttrs->accelMemBaseAddr;
            memInfo->numBanks       = SOC_HWA_NUM_MEM_BANKS;
            memInfo->bankSize       = ptrHWADriver->hwAttrs->accelMemSize / memInfo->numBanks;
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    return(retCode);
}



/*!
 *  @brief  Function to get the dma destination index with a given EDMA channel number
 *          This function assumes the EDMA channel number is from the first EDMA instance.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  edmaChanId      EDMA channell id
 *
 *  @param  hwaDestChan     Pointer to save destination channel index
 *
 *  @return     =0          Success
 *              <0          Error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getDMAChanIndex(HWA_Handle handle, uint8_t edmaChanId, uint8_t *hwaDestChan)
{
    HWA_Object      *ptrHWADriver = NULL;
    int32_t         retCode = 0;
    uint8_t         index;
    bool            foundChan = false;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (hwaDestChan == NULL)
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            for(index = 0; index < ptrHWADriver->hwAttrs->numDmaChannels; index ++)
            {
                if(edmaChanId == gHwaEDMAChanMapping[index])
                {
                    foundChan = true;
                    *hwaDestChan = index;
                    break;
                }
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    if(foundChan == false)
    {
        retCode = HWA_EINVAL;
    }
    return(retCode);
}

/*!
 *  @brief  Function to get the edma EDMA channel number from a given HWA paramset destination channel.
 *          This function assumes the EDMA channel number is from the first EDMA instance.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  hwaDMAdestChan  Destination channle id set in a paramset
 *
 *  @return     >=0         Upon success, EDMA channel number
 *              <0          Error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getEDMAChanId(HWA_Handle handle, uint8_t hwaDMAdestChan)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (hwaDMAdestChan > (ptrHWADriver->hwAttrs->numDmaChannels-1U))
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            retCode = gHwaEDMAChanMapping[hwaDMAdestChan];
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    return(retCode);
}

/*!
*  @brief  Function to set the HWA RAM : HWA_RAM_TYPE_WINDOW_RAM, HWA_RAM_TYPE_VECTORMULTIPLY_RAM, HWA_RAM_TYPE_LUT_FREQ_DEROTATE_RAM or HWA_RAM_TYPE_SHUFFLE_RAM
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  ramType         Use defines \ref HWA_RAM_TYPE
*
*  @param  data            data pointer that needs to be copied to RAM
*
*  @param  dataSize        Size of data to be copied in size of bytes
*
*  @param  startIdx        start index (in terms of bytes) within RAM where data needs to be copied
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_configRam(HWA_Handle handle, uint8_t ramType, uint8_t *data, uint32_t dataSize, uint32_t startIdx)
{
    HWA_Object            *ptrHWADriver = NULL;
    int32_t               retCode = 0;
    DSSHWACCRegs *ctrlBaseAddr;
    uint32_t *ramBaseAddr;
    uint32_t *window;
    uint32_t i=0;

    window = (uint32_t*)data;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (data == NULL)
        {
            /* invalid data */
            retCode = HWA_EINVAL;
        }
        else if ((ramType != HWA_RAM_TYPE_WINDOW_RAM) && (ramType != HWA_RAM_TYPE_INTERNAL_RAM))
        {
            /* invalid config params */
            retCode = HWA_EINVAL;
        }
        else if ((dataSize == 0U) || (dataSize > HWA_RAM_WINDOW_SIZE_IN_BYTES))
        {
            /* invalid data size */
            retCode = HWA_EINVAL;
        }
        else if ((startIdx >= HWA_RAM_WINDOW_SIZE_IN_BYTES) || ((startIdx + dataSize) > HWA_RAM_WINDOW_SIZE_IN_BYTES))
        {
            /* invalid data size */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            ramBaseAddr = (uint32_t*)(ptrHWADriver->hwAttrs->ramBaseAddr + startIdx);
            /* select the right RAM */
            if (ramType == HWA_RAM_TYPE_INTERNAL_RAM)
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG7, 24U, 24U, 1U);
            }
            else
            {
                CSL_FINSR(ctrlBaseAddr->HWACCREG7, 24U, 24U, 0U);
            }
            /* copy the RAM contents */
            for(i=0; i<(dataSize/sizeof(uint32_t)); i++)
            {
                HW_WR_REG32(ramBaseAddr, *window);
                ramBaseAddr = ramBaseAddr + 0x1U;
                window = window + 0x1U;
            }

            //memcpy((void *)&ramBaseAddr[startIdx],(void *)data, dataSize);
            /* reset the RAM selection - this is needed to avoid paramset corruption */
            CSL_FINSR(ctrlBaseAddr->HWACCREG7, 24U, 24U, 0U);
        }

        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}



/*!
*  @brief  Function to enable the CPU and/or DMA interrupt after a paramSet completion.
*          The CPU interrupt for every paramset completion may not be supported on all
*          devices - see \ref HWA_Attrs::isConcurrentAccessAllowed
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  paramsetIdx     A valid paramSet index for which the intrConfig is provided.
*
*  @param  intrConfig      HWA Interrupt Config Parameters
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_enableParamSetInterrupt(HWA_Handle handle, uint8_t paramsetIdx, HWA_InterruptConfig *intrConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCPARAMRegs *paramBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, false, true, paramsetIdx);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (intrConfig == NULL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else if (((intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_DMA)!=0U) &&
            (intrConfig->dma.dstChannel > (ptrHWADriver->hwAttrs->numDmaChannels - 1U)))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else if (((intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU)!=0U) &&
            (ptrHWADriver->hwAttrs->isConcurrentAccessAllowed == false))/*GAP COMMENT ID:29 START*/
        {
            /* This version of HWA IP doesn't allow concurrent read of HWACCREGx registers when HWA is active.
             * While enabling of this CPU interrupt is fine, it is the handling of this interrupt in ISR that
             * would require read of HWACCREG4 register when HWA is active which is prohibited.
             */
            retCode = HWA_ENOTSUPP;
        }/*GAP COMMENT ID:29 END*/
        else
#endif
        {
            paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr + (paramsetIdx * sizeof(DSSHWACCPARAMRegs)));
            if ((intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU) != 0U)
            {
                /* handle this flag for IPs/devices where concurrent read access of common registers are allowed
                   while HWA is executing paramsets */
                uintptr_t           key;

                /* Disable preemption while setting the registers; since these are accessed via ISR as well */
                key = HwiP_disable();

                /* save the interrupt context */
               // ptrHWADriver->interruptCtxParamSet[paramsetIdx].bIsEnabled = true;
                ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackFn = intrConfig->cpu.callbackFn;
                ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackArg = intrConfig->cpu.callbackArg;

                /* enable the interrupt to CPU in H/W */
                CSL_FINSR(paramBaseAddr->PARAMn_0, 7U, 7U, 1U);

                /* Restore the interrupts */
                HwiP_restore(key);
            }
            if ((intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_DMA) != 0U)
            {
                /* set the destination channel number */
                CSL_FINSR(paramBaseAddr->PARAMn_0, 12U, 9U, intrConfig->dma.dstChannel);
                /* enable the interrupt to DMA */
                CSL_FINSR(paramBaseAddr->PARAMn_0, 8U, 8U, 1U);
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, false, true, paramsetIdx);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to enable the CPU interrupt after all programmed paramSets have been completed in the background or ALT thread.
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  callbackFn      user defined callback function to be called when this interrupt is generated
*
*  @param  callbackArg     user defined callback arg for the callback function
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_enableDoneInterrupt(HWA_Handle handle, HWA_Done_IntHandlerFuncPTR callbackFn, void * callbackArg)
{
    HWA_Object          *ptrHWADriver = NULL;
    uintptr_t           key;
    int32_t             retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check state of driver */
    if ((ptrHWADriver == NULL) || (ptrHWADriver->refCnt == 0U))
    {
        retCode = HWA_ENOINIT;
    }
    else
    {
        /* save the interrupt context */
        //ptrHWADriver->interruptCtxDone.bIsEnabled = true;
        ptrHWADriver->interruptCtxDone.callbackFn = callbackFn;
        ptrHWADriver->interruptCtxDone.callbackArg = callbackArg;

        /* enable the interrupt to CPU in H/W */
        HwiP_enableInt(ptrHWADriver->hwAttrs->intNumDone);

        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* Restore the interrupts: */
    HwiP_restore(key);

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to disable the CPU and/or DMA interrupt after a paramSet completion.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle              A HWA_Handle returned from HWA_open()
 *
 *  @param  paramsetIdx         A valid paramSet index for which the interrupt is to be disabled
 *
 *  @param  interruptTypeFlag   Flag to indicate if CPU and/or DMA interrupts are to be disabled
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_disableParamSetInterrupt(HWA_Handle handle, uint8_t paramsetIdx, uint8_t interruptTypeFlag)
{
    HWA_Object          *ptrHWADriver = NULL;
    uintptr_t           key;
    int32_t             retCode = 0;
    DSSHWACCPARAMRegs *paramBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, false, true, paramsetIdx);
    if (retCode == 0)
    {
        paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr + (paramsetIdx * sizeof(DSSHWACCPARAMRegs)));

        if ((interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU) != 0U)
        {

            /* Disable preemption while setting the interrupt context */
            key = HwiP_disable();

            /* update the interrupt context */
            //ptrHWADriver->interruptCtxParamSet[paramsetIdx].bIsEnabled = false;
            ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackFn = NULL;
            ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackArg = NULL;

            /* disable the interrupt to CPU in H/W */
            CSL_FINSR(paramBaseAddr->PARAMn_0, 7U, 7U, 0U);

            /* Restore the interrupts */
            HwiP_restore(key);
        }
        if ((interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_DMA) != 0U)
        {
            /* disable the interrupt to DMA */
            CSL_FINSR(paramBaseAddr->PARAMn_0, 8U, 8U, 0U);
        }
        HWA_releaseDriverAccess(ptrHWADriver, false, true, paramsetIdx);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to disable the CPU interrupt after all programmed paramSets have been completed.
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_disableDoneInterrupt(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    uintptr_t                key;
    int32_t                  retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check state of driver */
    if ((ptrHWADriver == NULL) || (ptrHWADriver->refCnt == 0U))
    {
        retCode = HWA_ENOINIT;
    }
    else
    {
        /* save the interrupt context */
        //ptrHWADriver->interruptCtxDone.bIsEnabled = false;
        ptrHWADriver->interruptCtxDone.callbackFn = NULL;
        ptrHWADriver->interruptCtxDone.callbackArg = NULL;

        /* enable the interrupt to CPU in H/W */
        HwiP_disableInt(ptrHWADriver->hwAttrs->intNumDone);
    }

    /* Restore the interrupts */
    HwiP_restore(key);

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to enable the state machine of the HWA. This should be called after
 *          paramset and RAM have been programmed
 *
 *  @pre    HWA_open() HWA_ConfigCommon() HWA_ConfigParamSet HWA_ConfigRam has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  flagEnDis       Enable/Disable Flag: 0-disable, 1-enable
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_enable(HWA_Handle handle, uint8_t flagEnDis)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        /* set the ACCENABLE bits to 0x7 */
        if (flagEnDis == 1U)
        {
            CSL_FINSR(ctrlBaseAddr->HWACCREG1, 5U, 3U, 0x7U);
            CSL_FINSR(ctrlBaseAddr->HWACCREG1, 2U, 0U, 0x7U);
        }
        /* set the ACCENABLE bits to 0x0 */
        else
        {
            CSL_FINSR(ctrlBaseAddr->HWACCREG1, 2U, 0U, 0x0U);
            CSL_FINSR(ctrlBaseAddr->HWACCREG1, 5U, 3U, 0x0U);
        }
        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to manually trigger the execution of the state machine via software,
*           the software trigger through either FW2HWA_TRIG_0 or FW2HWA_TRIG_1 register
*           if the trigger is set to HWA_TRIG_MODE_SOFTWARE from DSP, trigger is done through FW2HWA_TRIG_0
*           if the trigger is set to HWA_TRIG_MODE_M4CONTROL from M4, trigger is done through FW2HWA_TRIG_1
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_setSoftwareTrigger(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        /* set the CM42ACCTRIG bits */
        CSL_FINSR(ctrlBaseAddr->HWACCREG3, 0U, 0U, 1U);

        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function for software to reset the DC accumulators or interference statistics accumulators
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*  @param  accumulatortype  see maros \ref HWA_ACCUMULATORREG_TYPE, takes value either HWA_ACCUMULATORREG_TYPE_DC
*                           or HWA_ACCUMULATORREG_TYPE_INTERF
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_softwareResetAccumulators(HWA_Handle handle, uint8_t accumulatortype)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        if (accumulatortype == HWA_ACCUMULATORREG_TYPE_DC)
        {
            CSL_FINSR(ctrlBaseAddr->DCEST_RESET_SW,
                      DCEST_RESET_SW_DCEST_RESET_SW_BIT_END,
                      DCEST_RESET_SW_DCEST_RESET_SW_BIT_START,
                      1U);
        }
        else
        {
            CSL_FINSR(ctrlBaseAddr->INTF_STATS_RESET_SW,
                      INTF_STATS_RESET_SW_INTF_STATS_RESET_SW_BIT_END,
                      INTF_STATS_RESET_SW_INTF_STATS_RESET_SW_BIT_START,
                      1U);
        }

        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to manually trigger the execution of the state machine waiting on DMA via software
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  idx            DMA channel number for whom software should simulate the trigger
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_setDMA2ACCManualTrig(HWA_Handle handle, uint8_t idx)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        /* set the DMA2ACCTRIG bits to DMA Channel */
        CSL_FINSR(ctrlBaseAddr->HWACCREG2, 15U, 0U, (1U << idx));

        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to set the source address for one paramset
*
*  @pre    HWA_open() has been called.
*
*  @param  handle             A HWA_Handle returned from HWA_open()
*
*  @param  paramparamsetIdxIdx           the paramset index
*
*  @param  sourceAddress      source address
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_setSourceAddress(HWA_Handle handle, uint16_t paramsetIdx, uint32_t sourceAddress)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCPARAMRegs *paramBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) false, (bool) true, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (paramsetIdx > SOC_HWA_NUM_PARAM_SETS)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr + (paramsetIdx * sizeof(DSSHWACCPARAMRegs)));
            CSL_FINSR(paramBaseAddr->PARAMn_1, PARAMn_1_SRCADDR_BIT_END, PARAMn_1_SRCADDR_BIT_START, sourceAddress);
        }

        HWA_releaseDriverAccess(ptrHWADriver, (bool) false, (bool) true, 0);
    }
    return (retCode);

}


/*!
 *  @brief  Function to read the 4 sets of 'MAX' statistics register
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  pStats          pointer to a memory of type HWA_Stats where value of all the
 *                          Max and statistics Registers would be copied
 *
 *  @param  numIter         number of iterations to read. Value 1-4 should be provided.
 *                          User is expected to provide enough space for the pStats to hold 'numIter' worth of HWA_Stats
 *                          Ex: HWA_Stats appHWAStats[3]; HWA_readStatsReg(appHWAhandle,appHWAStats,3);
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_readStatsReg(HWA_Handle handle, HWA_Stats *pStats, uint8_t numIter)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((pStats == NULL) || (numIter > 4U) || (numIter == 0U))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            uint8_t i = 0;
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read the MAX and SUM registers*/
            for (i = 0; i < numIter; i++)
            {
                pStats[i].maxValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->HWACCSTATSREG[i].MAXnVALUE, 23U, 0U);
                pStats[i].maxIndex = (uint16_t)CSL_FEXTR(ctrlBaseAddr->HWACCSTATSREG[i].MAXnINDEX, 11U, 0U);
                pStats[i].iSumLSB = (uint32_t)CSL_FEXTR_64(ctrlBaseAddr->HWACCSTATSREG[i].ISUMnLSB, 31U, 0U);
                pStats[i].iSumMSB = (uint8_t)CSL_FEXTR(ctrlBaseAddr->HWACCSTATSREG[i].ISUMnMSB, 3U, 0U);
                pStats[i].qSumLSB = (uint32_t)CSL_FEXTR_64(ctrlBaseAddr->HWACCSTATSREG[i].QSUMnLSB, 31U, 0U);
                pStats[i].qSumMSB = (uint8_t)CSL_FEXTR(ctrlBaseAddr->HWACCSTATSREG[i].QSUMnMSB, 3U, 0U);
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}



/*!
 *  @brief  Function to read the PEAKCNT register
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  pbuf            pointer to a memory where value of the
 *                          PEAKCNT Registers would be copied
 *
 *  @param  size            size (in bytes) of the pbuf register provided.
 *                          It should be atleast 2 bytes.
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_readCFARPeakCountReg(HWA_Handle handle, uint8_t *pbuf, uint8_t size)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;
    uint16_t                *peakCnt;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((size < 2U) || (pbuf == NULL))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            peakCnt = (uint16_t*)pbuf;
            /* read the FFTPEAKCNT register */
            *peakCnt = (uint16_t)CSL_FEXTR(ctrlBaseAddr->FFTPEAKCNT, 11U, 0U);
        }
        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}


/*!
 *  @brief  Function to read the debug registers (paramcurr, loopcou, acc_trig_in_stat)
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  pStats          pointer to a memory of type HWA_debugStats where value of the
 *                          RDSTATUS and HWACCREG12 Registers would be copied
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_readDebugReg(HWA_Handle handle, HWA_DebugStats *pStats)
{
    HWA_Object              *ptrHWADriver = NULL;
   int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (pStats == NULL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read the Debug registers*/
            pStats->currentParamSet = (uint8_t)CSL_FEXTR(ctrlBaseAddr->RDSTATUS, 4U, 0U);
            pStats->currentLoopCount = (uint16_t)CSL_FEXTR(ctrlBaseAddr->RDSTATUS, 16U, 5U);
            pStats->dmaTrigStatus = (uint16_t)CSL_FEXTR(ctrlBaseAddr->HWACCREG12, 18U, 3U);
            pStats->dfePingPongStatus = (uint8_t)CSL_FEXTR(ctrlBaseAddr->HWACCREG12, 2U, 2U);
            pStats->swTrigStatus = (uint8_t)CSL_FEXTR(ctrlBaseAddr->HWACCREG12, 1U, 1U);
        }
        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to clear the debug registers (acc_trig_in_clr)
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_clearDebugReg(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, true, false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        /* clear the Debug Reg */
        CSL_FINSR(ctrlBaseAddr->HWACCREG12, 24U, 24U, 1U);

        HWA_releaseDriverAccess(ptrHWADriver, true, false, 0);
    }

    /* return */
    return retCode;
}
