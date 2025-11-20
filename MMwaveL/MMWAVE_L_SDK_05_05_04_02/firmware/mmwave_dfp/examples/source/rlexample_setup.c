/*!*****************************************************************************
 * @file rlexample_setup.c
 *
 * @brief mmWave DFP Low Example Setup Data and Functions
 *
 * @b Description @n
 * This file contains setup data and functions required for running mmWave DFP Low
 * examples.
 *
 * @note Global setup variables should be updated according the device/ application
 * environment
 *
 *
 * @note
 * <B> © Copyright 2022, Texas Instruments Incorporated – www.ti.com </B>
 * @n
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * @n
 * <ul>
 *   <li> Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 * @n
 *   <li> Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 * @n
 *   <li> Neither the name of Texas Instruments Incorporated nor the names of
 *        its contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 * </ul>
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS
 * IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
 * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     15Nov2022   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */

#include <mmwavelink/mmwavelink.h>
#include <mmwavelink/include/rl_device.h>
#include <examples/include/rlexample.h>

/* Library headers */
#include <stdarg.h>

/* Application Specific Headers */
#include <platform/xwrlx432/syscore/include/sys_nonos.h>
#include <platform/xwrlx432/syscore/include/sys_nvic.h>
#include <platform/xwrlx432/syscore/include/sys_syscore.h>
#include <platform/xwrlx432/syscore/include/sys_logger.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
/*!
 * @brief  Example full calibration data
 */
T_RL_API_FECSS_FACT_CAL_DATA z_RlExampleFactCalData =
    {
        .h_CalResValidity = 0x0027U,
        .h_CalResValRed = 0x0027U,
        .w_ApllCapCodes = 0x00010113U,
        .h_SynthCapCodes = 0x0001U,
        .h_IfaTrimCodes = 0x6E19U,
        .h_PdFactCalTemp = 0x0025U,
        .c_PdCalCodes = {
            0x14U,
            0x00U,
            0x40U,
            0x02U,
            0x40U,
            0x40U,
            0x40U,
            0x40U,
            0x40U,
            0x42U,
            0x06U,
            0x02U,
            0x44U,
            0x42U,
            0x02U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0xC9U,
            0x06U,
            0x98U,
            0x06U,
            0x4DU,
            0x07U,
            0xBFU,
            0x06U,
            0x7AU,
            0x06U,
            0xACU,
            0x06U,
            0xDCU,
            0x06U,
            0x71U,
            0x06U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
        },
};

/** @brief Example Tx and Rx only calibration data */
T_RL_API_FECSS_RXTX_CAL_DATA z_RlExampleTxRxCalData =
    {

};

/** @brief Expected DFP Version */
T_RL_API_DFP_FW_VER_GET_RSP z_RlExampleDfpVersion =
    {
        .z_FecssLibVersion =
            {
                .c_BuildVerNum = M_FE_FECSSLIB_VERSION_BUILD,
                .c_Date = M_FE_FECSSLIB_VERSION_DAY,
                .c_GenVerNum = M_FE_FECSSLIB_VERSION_GEN,
                .c_MajorVerNum = M_FE_FECSSLIB_VERSION_MAJOR,
                .c_MinorVerNum = M_FE_FECSSLIB_VERSION_MINOR,
                .c_Month = M_FE_FECSSLIB_VERSION_MONTH,
                .c_Year = M_FE_FECSSLIB_VERSION_YEAR,
            },

        .z_MmwlLibVersion =
            {
                .c_BuildVerNum = M_RL_MMWAVELINK_VERSION_BUILD,
                .c_Date = M_RL_MMWAVELINK_VERSION_DAY,
                .c_GenVerNum = M_RL_MMWAVELINK_VERSION_GEN,
                .c_MajorVerNum = M_RL_MMWAVELINK_VERSION_MAJOR,
                .c_MinorVerNum = M_RL_MMWAVELINK_VERSION_MINOR,
                .c_Month = M_RL_MMWAVELINK_VERSION_MONTH,
                .c_Year = M_RL_MMWAVELINK_VERSION_YEAR,
            },

        .z_RfsRomVersion =
            {
                .c_BuildVerNum = 0U,
                .c_MinorVerNum = 10U,
                .c_MajorVerNum = 0U,
                .c_GenVerNum = 7U,
                .c_Date = 11U,
                .c_Month = 11U,
                .c_Year = 23U,
            },

        .z_RfsPatchVersion =
            {
                .c_BuildVerNum = 0U,
                .c_MinorVerNum = 10U,
                .c_MajorVerNum = 0U,
                .c_GenVerNum = 7U,
                .c_Date = 11U,
                .c_Month = 11U,
                .c_Year = 23U,
            },
};

UINT32 w_RlExampleRfsDebugData[64U / sizeof(UINT32)];

T_DFP_CLIENT_CB_DATA z_RlExampleLinkClientCbData =
{
    .c_PlatformType = M_DFP_DEVICE_PLATFORM_TYPE,
    .c_DeviceType = 0U,
    .c_ApiErrorCheckDis = M_FALSE,
    .h_ApiRespTimeoutMs = (UINT16)20U,
    .c_ApiDbglogEn = 1U,

    /*! OSI related initialization */
    .z_OsiCb =
        {
            .z_Mutex =
                {
                    .p_OsiMutexCreate = &sys_lockObjCreate,
                    .p_OsiMutexLock = &sys_lockObjLock,
                    .p_OsiMutexUnLock = &sys_lockObjUnLock,
                    .p_OsiMutexDelete = &sys_lockObjDelete},

            .z_Semp =
                {
                    .p_OsiSemCreate = &sys_syncObjCreate,
                    .p_OsiSemWait = &sys_syncObjWait,
                    .p_OsiSemSignal = &sys_syncObjSignal,
                    .p_OsiSemDelete = &sys_syncObjDelete},

            .z_Queue =
                {
                    .p_OsiSpawn = M_NULL_PTR},
        },

    /*! Platform related initialization */
    .z_PltfCb =
        {
            .p_Delay = &sys_delayP1us,
            .p_TimeStamp = &sys_getSysTickCount_ind,
            .w_TimeStampCounterMask = M_SYS_SYSTICK_CONTER_VAL_MASK,
            .p_RegisterIsr = &sys_registerMailboxIsr,
            .p_DeRegisterIsr = &sys_deRegisterIsr,
            .p_MaskIsr = &sys_maskIsr,
            .p_UnMaskIsr = &sys_unMaskIsr,
            .p_EnterCriticalRegion = &sys_enterCriticalReg,
            .p_ExitCriticalRegion = &sys_exitCriticalReg},

    /*! Debug related initialization */
    .z_DbgCb =
        {
            .p_RfsdbgData = w_RlExampleRfsDebugData,
#if (0U == M_DFP_DISABLE_LOGGING)
            .p_Print = sys_loggerPrintf,
#endif
        },

    /*! Communication Interface related initialization */
    .z_ComIfCb =
        {
            .p_ComIfOpen = M_NULL,
            .p_ComIfRead = M_NULL,
            .p_ComIfWrite = M_NULL,
            .p_ComIfClose = M_NULL
        }
};

volatile UINT32 w_RlExampleFrameOffsetCount = 0U;
volatile UINT32 w_RlExampleMonitorDoneCount = 0U;

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief memcmp function
 *
 * @b Description @n
 * This function compares source data with reference date and returns number of
 * matching bytes. The function is used in the DFP examples to compare the test results
 * with expected data.
 *
 * @param[in]   p_srcData   : Pointer to source data
 * @param[in]   p_refData   : Pointer to reference data
 * @param[in]   w_numBytes  : number of bytes in the data
 *
 * @returns     w_numMatch  : Number of matching bytes
 *
 *******************************************************************************
 */
UINT32 rlExample_memcmp(const UINT8 *p_srcData, const UINT8 *p_refData, UINT32 w_numBytes)
{
    UINT32 w_numMatch = 0U;

    while ((w_numMatch < w_numBytes) && (*p_srcData == *p_refData))
    {
        w_numMatch++;
        p_srcData++;
        p_refData++;
    }

    return w_numMatch;
}

/*!*****************************************************************************
 * @brief Blocking delay function
 *
 * @b Description @n
 * @param[in]   w_delayUs   : Delay in micro-seconds
 *
 *******************************************************************************
 */
void rlExample_delayUs(UINT32 w_delayUs)
{
    sys_delayP1us(w_delayUs * 10U);
}

UINT8 rlExample_getCalibrationTempBinIndex(SINT32 xw_avgAnaTemp)
{
    UINT8 c_tempBinIndex;
    if (xw_avgAnaTemp < 0)
    {
        c_tempBinIndex = M_RL_FECSS_CAL_TEMP_INDEX_LOW;
    }
    else if (xw_avgAnaTemp < 85)
    {
        c_tempBinIndex = M_RL_FECSS_CAL_TEMP_INDEX_MID;
    }
    else
    {
        c_tempBinIndex = M_RL_FECSS_CAL_TEMP_INDEX_HIGH;
    }

    return c_tempBinIndex;
}

SINT16 rlExample_runtimeCalHandler(SINT16 xh_currentTemp, SINT16 xh_previousCalTemp,
                                   UINT16 h_calEnableMask, UINT8 c_deviceIndex, UINT8 c_logEnable)
{
    SINT16 xh_newCalTemp = xh_previousCalTemp;
    T_RETURNTYPE xw_errorCode;

    UINT8 c_currentCalTempBin = rlExample_getCalibrationTempBinIndex(xh_currentTemp);
    UINT8 c_previousCalTempBin = rlExample_getCalibrationTempBinIndex(xh_previousCalTemp);

    if ((M_RL_EXAMPLE_ABS(xh_currentTemp - xh_previousCalTemp) >= 40) ||
        (c_currentCalTempBin != c_previousCalTempBin))
    {
        UINT16 h_runCalMask = (UINT16)0xCE & h_calEnableMask;

        T_RL_API_FECSS_RF_RUN_CAL_CMD z_fecssRunCalCtrl =
            {
                .h_CalCtrlBitMask = h_runCalMask, /**< Enable all runtime calibrations */
                .c_TempBinIndex = c_currentCalTempBin,
            };
        T_RL_API_FECSS_RF_RUN_CAL_RSP z_fecssRunCalRsp = {0};

        xw_errorCode = rl_fecssRfRuntimeCal(c_deviceIndex, &z_fecssRunCalCtrl, &z_fecssRunCalRsp);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_runtimeCalHandler: rl_fecssRfRuntimeCal failed with %d\n",
                            xw_errorCode);
        }
        else if (z_fecssRunCalRsp.h_CalRunStatus != z_fecssRunCalCtrl.h_CalCtrlBitMask)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_runtimeCalHandler: rl_fecssRfRuntimeCal calibration run status %x\n",
                            z_fecssRunCalRsp.h_CalRunStatus);
            xw_errorCode = M_DFP_RET_CODE_FATAL_ERROR;
        }

        xh_newCalTemp = xh_currentTemp;
    }

    return xh_newCalTemp;
}

void rlExample_frameOffsetTimerInterrupt(void)
{
    w_RlExampleFrameOffsetCount++;
}

void rlExample_monitorDoneInterrupt(void)
{
    w_RlExampleMonitorDoneCount++;
}


void rlExample_configureInterrupt(UINT8 c_control, UINT8 c_irqNum, void (*p_handler)(void))
{

    if (M_DISABLE != c_control)
    {
        sys_nvicVectTableSet(c_irqNum, (UINT32)p_handler);
        sys_nvicClrPendInt(c_irqNum);
        sys_nvicSetIntPriority(c_irqNum, M_SYS_EXCEPTION_PRIORITY_1);
        sys_nvicEnableInt(c_irqNum);
    }
    else
    {
        sys_nvicDisableInt(c_irqNum);
    }
}

/*
 * END OF rlexample_setup.c FILE
 */
