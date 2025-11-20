/*!*****************************************************************************
 * @file rlexample_txclpccal.c
 *
 * @brief mmWave DFP Low Tx CLPC Calibration Example
 *
 * @b Description @n
 * This example demonstrates tx clpc calibration configuration sequence for
 * mmWaveLow devices
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
 * 0.1     19Mar2023   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */

#include <mmwavelink/mmwavelink.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>
#include <mmwavelink/include/rl_monitor.h>

#include <examples/include/rlexample.h>


#define M_RL_EXAMPLE_NUM_TX_CLPC_BINS           (9U)


typedef struct
{
    UINT8   c_TxCalTempBinIndex;
    UINT8   c_Reserved;
    SINT16  xh_LastCalTemp;
    T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP z_txRuntimeCalLut[M_RL_EXAMPLE_NUM_TX_CLPC_BINS];
} M_RL_EXAMPLE_TX_CLPC_DATA;

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief Run all calibrations with Tx runtime CLPC
 *
 * @b Description @n
 * This function run Tx CLPC and other calibrations based on temperature delta
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_txClpcRunCalibrations(M_RL_EXAMPLE_TX_CLPC_DATA* p_txClpcLut,
                                        UINT8 c_devIndex,
                                        UINT8 c_deviceType,
                                        UINT8 c_logEnable)
{
    T_RETURNTYPE xw_errorCode  = M_DFP_RET_CODE_OK;

    /** -# Measure current average temperature using @ref rl_fecssTempMeasTrig */
    T_RL_API_FECSS_TEMP_MEAS_RSP z_fecssTempRes;
    xw_errorCode = rl_fecssTempMeasTrig(c_devIndex, &z_fecssTempRes);

    SINT16 xh_currentTemp = (z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_RX0] +
                    z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_TX0] +
                    z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_PM]) / 3;

    /** -# Run runtime calibration for Synth, PD, LODIST and Rx */
    p_txClpcLut->xh_LastCalTemp  = rlExample_runtimeCalHandler(xh_currentTemp ,
            p_txClpcLut->xh_LastCalTemp, 0x4A, c_devIndex, c_logEnable);

    /** -# Compute current Tx Calibration temperature bin = (Temperature + 40) / 20 */
    UINT8 c_currentCalTempBin;
    if (xh_currentTemp <= -40)
    {
        c_currentCalTempBin = 0U;
    }
    else if (xh_currentTemp < 140)
    {
        c_currentCalTempBin = (UINT8)((xh_currentTemp + 40) / 20);
    }
    else
    {
        c_currentCalTempBin = 8U;
    }

    /** -# @b IF Current Tx bin != Previous Tx Bin <OL type="a"> */
    if( c_currentCalTempBin != p_txClpcLut->c_TxCalTempBinIndex)
    {
        /** <LI> Setup for Tx Runtime CLPC calibration */
        p_txClpcLut->c_TxCalTempBinIndex = c_currentCalTempBin;

        T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD z_fecssTxClpcCalCtrl =
        {
            .xc_OverrideCalTemp     = 0,
            .c_CalTxBackOffSel[0]   = 0U,     /**< 0 db Backoff */
            .c_CalTxBackOffSel[1]   = 0U,     /**< 0 db Backoff */
            .h_CalRfFreq            = M_RL_SENS_CHIRP_RFFREQ_LR_57G,   /**< Starting frequency */
            .xh_CalRfSlope          = (SINT16)0x4D,
            .c_TxPwrCalTxEnaMask[0] = 0b11,
            .c_TxPwrCalTxEnaMask[1] = 0b01,
        };

        /** <LI> @b IF CLPC LUT data is valid for current temp bin <OL type="i"> */
        if(p_txClpcLut->z_txRuntimeCalLut[c_currentCalTempBin].c_CalRunStatus != 0U)
        {
            /** <LI> Set calibration mode to Override */
            z_fecssTxClpcCalCtrl.c_CalMode = 1U;
            /** <LI> Copy Tx Stage codes from results to config */
            z_fecssTxClpcCalCtrl.h_TxStgOverrideCodes[0U] = p_txClpcLut->z_txRuntimeCalLut[c_currentCalTempBin].h_TxStgCodes[0U];
            z_fecssTxClpcCalCtrl.h_TxStgOverrideCodes[1U] = p_txClpcLut->z_txRuntimeCalLut[c_currentCalTempBin].h_TxStgCodes[1U];
            z_fecssTxClpcCalCtrl.h_TxStgOverrideCodes[2U] = p_txClpcLut->z_txRuntimeCalLut[c_currentCalTempBin].h_TxStgCodes[2U];
            z_fecssTxClpcCalCtrl.h_TxStgOverrideCodes[3U] = p_txClpcLut->z_txRuntimeCalLut[c_currentCalTempBin].h_TxStgCodes[3U];
        } /** </OL> */

        /** <LI> Runtime Tx CLPC Calibration API */
        xw_errorCode = rl_fecssRlRuntimeTxClpcCal(c_devIndex, &z_fecssTxClpcCalCtrl,
                            &(p_txClpcLut->z_txRuntimeCalLut[c_currentCalTempBin]));
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_txClpcRunCalibrations: rl_fecssRlRuntimeTxClpcCal \
                    failed with %d in iteration %d\n", xw_errorCode);
        }
    } /** </OL> */
    return xw_errorCode;
}

/*!*****************************************************************************
 * @brief Tx CLPC Calibration Example
 *
 * @b Description @n
 *
 * @param[in]   c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   h_numLoops      : Number of test loops
 * @param[out]  xw_errorCode    : Error-code in case of failure
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 * <tr> <th> @b Design-Ids <td>
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_txClpcCalibLoop(UINT8 c_deviceType, UINT16 h_numLoops)
{
    T_RETURNTYPE xw_errorCode  = M_DFP_RET_CODE_OK;
    UINT8 c_devIndex = 0U;
    UINT8 c_logEnable = M_ENABLE;
    UINT16 h_loopIndex = 0U;

    /** -# Create @ref M_RL_EXAMPLE_TX_CLPC_DATA to store Tx CLPC results */
    M_RL_EXAMPLE_TX_CLPC_DATA z_txClpcLut = {0};

    /** -# Set last calibration temperature, and calibration temperature bin indices to invalid
     * values to force the cal on first run */
    z_txClpcLut.xh_LastCalTemp      = -1000;
    z_txClpcLut.c_TxCalTempBinIndex = 255;

    /** -# Setup temperature measurement for all 4 temperature sensors using @ref rl_fecssTempMeasCfg */
    T_RL_API_FECSS_TEMP_MEAS_CMD z_fecssTempCfg =
    {
        .h_TempCtrlBitMask = 0x311U,    /**< Enable all temperature sensors */
    };
    xw_errorCode = rl_fecssTempMeasCfg(c_devIndex, &z_fecssTempCfg);

    /** -# Setup Chirp profile */
    UINT8 c_numFrames = 2U;
    xw_errorCode = rlExample_setupChirpProfile(c_deviceType, c_devIndex, c_logEnable, c_numFrames);
    if(M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_txClpcCalibLoop: rlExample_setupChirpProfile failed with %d\n",
                xw_errorCode);
    }

    /** -# @b FOR required number of iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Handle calibrations in Tx CLPC mode */
        xw_errorCode = rlExample_txClpcRunCalibrations(&z_txClpcLut, c_devIndex, c_deviceType, c_logEnable);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_txClpcCalibLoop: rlExample_txClpcRunCalibrations failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
        }

        /** <LI> Trigger frames twice */
        for (UINT8 c_triggerIndex = 0U; c_triggerIndex < 2U; c_triggerIndex++)
        {
            T_RL_API_SENSOR_START_CMD z_sensStart =
            {
                .c_FrameTrigMode = M_RL_SENS_FRAME_SW_TRIG,
                .c_ChirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS,
                .c_FrameLivMonEn = 0x0U,
                .w_FrameTrigTimerVal = 0U
            };
            xw_errorCode = rl_sensSensorStart(c_devIndex, &z_sensStart);

            rlExample_delayUs(3000U * c_numFrames);

            T_RL_API_SENSOR_STATUS_RSP z_sensStatus;
            xw_errorCode += rl_sensStatusGet(c_devIndex, &z_sensStatus);
            if(M_DFP_RET_CODE_OK != xw_errorCode)
            {
                rlExample_logger(c_logEnable, "ERROR: rlExample_txClpcCalibLoop: sensor start failed with %d in iteration %d\n",
                        xw_errorCode, h_loopIndex);
                break;
            }
            else if (z_sensStatus.c_FrameStartStopStatus != 1U)
            {
                rlExample_logger(c_logEnable, "ERROR: rlExample_txClpcCalibLoop: incorrect sensor status %d in iteration %d\n",
                        z_sensStatus.c_FrameStartStopStatus, h_loopIndex);
                break;
            }
        }

        h_loopIndex++;
    } /** </OL> */

    return xw_errorCode;
}

/*
 * END OF rlexample_txclpccal.c FILE
 */