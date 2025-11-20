/*!*****************************************************************************
 * @file example_functions.h
 *
 * @brief mmWave DFP Low Example Functions
 *
 * @b Description @n
 * This file includes declarations for mmWaveLink example functions
 *
 * @warning Application developer / User shall review and update the build time
 * MACROs and rebuild the libraries as needed.
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
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     12Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef EXAMPLE_FUNCTIONS_H
#define EXAMPLE_FUNCTIONS_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @addtogroup MMWAVE_DFP_EXAMPLES mmWave DFP Example Functions
 * @{
 */


/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */

/*!
 * @brief Get the absolute of a signed number (static cast)
 * @param[in]   xw_val  : Signed number (SINT32)
 * @returns     Unsigned (UINT32) absolute value of xw_val
 */
#define M_RL_EXAMPLE_ABS(xw_val)   (((SINT32)(xw_val) >= 0) ? (UINT32)(xw_val) : (UINT32)(-((SINT32)(xw_val))))


/*!*****************************************************************************
 * @brief Example log function
 *
 * @b Description @n
 * Application log function
 * @param[in]   c_enable            : Enable log prints [0: Disable]
 * @param[in]   p_format            : (const char *) printf like  format string,
 *                                      Max log size 125 Bytes
 * @param[in]   variable_arguments  :arguments to the format string
 *
 *******************************************************************************
 */
extern T_RETURNTYPE sys_loggerPrintf(const char *p_format, ...);
#define rlExample_logger(c_enable, p_format, ...)  {if ((UINT8)0U != (c_enable)){(void)sys_loggerPrintf(p_format, ##__VA_ARGS__);}}
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

extern T_DFP_CLIENT_CB_DATA         z_RlExampleLinkClientCbData;
extern T_RL_API_FECSS_FACT_CAL_DATA z_RlExampleFactCalData;
extern T_RL_API_FECSS_RXTX_CAL_DATA z_RlExampleTxRxCalData;
extern T_RL_API_DFP_FW_VER_GET_RSP  z_RlExampleDfpVersion;

extern volatile UINT32 w_RlExampleFrameOffsetCount;
extern volatile UINT32 w_RlExampleMonitorDoneCount;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/** @name mmWave DFP Example Helper Functions @{ */
extern UINT32 rlExample_memcmp(const UINT8* p_srcData, const UINT8* p_refData, UINT32 w_numBytes);
extern void rlExample_delayUs(UINT32 w_delayUs);
extern void rlExample_configureInterrupt(UINT8 c_control, UINT8 c_irqNum, void (*p_handler)(void));
extern void rlExample_frameOffsetTimerInterrupt(void);
extern void rlExample_monitorDoneInterrupt(void);
/** @} */

/** @name Example sub-sequences @{ */
extern T_RETURNTYPE rlExample_setupChirpProfile(UINT8 c_deviceType, UINT8 c_deviceIndex, UINT8 c_logEnable,
        UINT8 c_numFrames);

extern T_RETURNTYPE rlExample_setupCalibrationData(
                UINT8                               c_deviceType,
                UINT8                               c_deviceIndex,
                UINT8                               c_calDataSelect,
                UINT8                               c_logEnable,
                const T_RL_API_FECSS_RXTX_CAL_DATA* p_txRxCalData,
                const T_RL_API_FECSS_FACT_CAL_DATA* p_fullCalData);

extern SINT16 rlExample_runtimeCalHandler(SINT16 xh_currentTemp, SINT16 xh_previousCalTemp,
        UINT16 h_calEnableMask, UINT8 c_deviceIndex, UINT8 c_logEnable);

extern UINT8  rlExample_getCalibrationTempBinIndex(SINT32 xw_avgAnaTemp);

extern T_RETURNTYPE rl_Example_applyCalibrationDataOnTempChange(UINT8 c_deviceType, UINT8 c_deviceIndex,
                UINT8 c_logEnable, SINT16* p_calTemp);
/** @} */

/** @name mmWave DFP Examples @{ */
extern T_RETURNTYPE rlExample_coldBootLoop(
                UINT8                               c_deviceType,
                UINT8                               c_calDataSelect,
                UINT16                              h_numLoops,
                const T_DFP_CLIENT_CB_DATA*         p_refLinkCbData,
                const T_RL_API_DFP_FW_VER_GET_RSP*  p_expectedDfpVersion,
                const T_RL_API_FECSS_RXTX_CAL_DATA* p_txRxCalData,
                const T_RL_API_FECSS_FACT_CAL_DATA* p_fullCalData);

extern T_RETURNTYPE rlExample_warmBootLoop(
                UINT8                               c_deviceType,
                UINT8                               c_calDataSelect,
                UINT16                              h_numLoops,
                const T_DFP_CLIENT_CB_DATA*         p_refLinkCbData,
                const T_RL_API_FECSS_RXTX_CAL_DATA* p_txRxCalData,
                const T_RL_API_FECSS_FACT_CAL_DATA* p_fullCalData);

extern T_RETURNTYPE rlExample_liveMonsLoop(UINT8 c_deviceType, UINT16 h_numLoops);

extern T_RETURNTYPE rlExample_gpadcMeasLoop(UINT8 c_deviceType, UINT16 h_numLoops);

extern T_RETURNTYPE rlExample_tempMeasLoop(UINT8 c_deviceType, UINT16 h_numLoops);

extern T_RETURNTYPE rlExample_perChirpLutLoop(UINT8 c_deviceType, UINT8 c_deviceIndex,
                UINT8 c_logEnable, UINT16 h_numLoops);

extern T_RETURNTYPE rlExample_infFrameRfStatusGetLoop(UINT8 c_deviceType, UINT16 h_numLoops);

extern T_RETURNTYPE rlExample_factoryCalibLoop(
                UINT8                               c_deviceType,
                UINT16                              h_numLoops,
                const T_DFP_CLIENT_CB_DATA*         p_refLinkCbData);

extern T_RETURNTYPE rlExample_factoryCalibOlpcLoop(
                UINT8                               c_deviceType,
                UINT16                              h_numLoops,
                const T_DFP_CLIENT_CB_DATA*         p_refLinkCbData);

extern T_RETURNTYPE rlExample_txClpcCalibLoop(UINT8 c_deviceType, UINT16 h_numLoops);

extern T_RETURNTYPE rlExample_infFrameWithMonitorsLoop(UINT8 c_deviceType, UINT16 h_numLoops);
/** @} */


#include <platform/xwrlx432/syscore/include/sys_main.h>

/*!
 * @}
 */
#ifdef __cplusplus
}
#endif


#endif /* example_functions.h */
