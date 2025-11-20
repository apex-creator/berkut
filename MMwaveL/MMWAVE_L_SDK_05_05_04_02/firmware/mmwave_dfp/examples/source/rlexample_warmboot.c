/*!*****************************************************************************
 * @file rlexample_warmboot.c
 *
 * @brief mmWave DFP Low Warm Boot Example
 *
 * @b Description @n
 * This  example demonstrates warm boot API configuration sequence for
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
 * 0.1     15Nov2022   TI      NA                  Initial Version
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

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief Rf power on setup after Warm boot
 *
 * @b Description @n
 * This function enabled APLL and RF components after warm boot
 *
 * @param[in]   c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   c_deviceIndex   : mmWave DFP Device index
 * @param[in]   c_logEnable     : Enable logger print [0: Disable]
 * @param[in]   c_calDataSelect : Calibration data select [ 0: Calibrate After Boot, 1: Restore full calibration data, 2: Restore only Tx-Rx calibration data]
 * @param[in]   c_bootMode      : FECSS boot mode [ @ref M_RL_FECSS_PWR_ON_MODE_COLD, @ref M_RL_FECSS_PWR_ON_MODE_WARM]
 * @param[in]   p_txRxCalData   : Pointer to Tx-Rx calibration data (Reference: @ref z_RlExampleTxRxCalData)
 * @param[in]   p_fullCalData   : Pointer to full calibration data (Reference: @ref z_RlExampleFactCalData)
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_warmBootSetup(
                UINT8                               c_deviceType,
                UINT8                               c_deviceIndex,
                UINT8                               c_logEnable,
                UINT8                               c_calDataSelect,
                UINT8                               c_bootMode,
                const T_RL_API_FECSS_RXTX_CAL_DATA* p_txRxCalData,
                const T_RL_API_FECSS_FACT_CAL_DATA* p_fullCalData)
{
    T_RETURNTYPE    xw_errorCode = M_DFP_RET_CODE_OK;

    /* Dummy Do-While Loop */
    do
    {
    /** -# Power on FECSS with correct boot mode using @ref rl_fecssDevPwrOn API */
    T_RL_API_FECSS_DEV_PWR_ON_CMD z_fecssDevOnApiData =
    {
        .h_XtalClkFreq      = M_RL_FECSS_XTAL_CLK_FREQ_40M,     /**< 40 MHz Crystal */
        .c_ClkSourceSel     = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS running on 80MHz fast clock */
        .c_PowerMode        = c_bootMode,
        .c_FecBootCfg       = 0U,                               /**< Enable boot-time self-tests */
        .c_ChirpTimerResol  = M_RL_FECSS_CHIRP_TIMER_RES_11,    /**< Frequency and time in high resolution */
    };

    xw_errorCode = rl_fecssDevPwrOn(c_deviceIndex, &z_fecssDevOnApiData);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootSetup: rl_fecssDevPwrOn failed with %d\n",
            xw_errorCode);
        break;
    }

    /** -# Configure RF front end enables with @ref rl_fecssRfPwrOnOff */
    T_RL_API_FECSS_RF_PWR_CFG_CMD z_rfsRfpwrOn =
    {
        .h_RxChCtrlBitMask = (1U << M_RL_MAX_RX_CHANNELS) - 1U,
        .h_TxChCtrlBitMask = (1U << M_RL_MAX_TX_CHANNELS) - 1U,
        .c_MiscCtrl        = (1U << M_RL_RF_MISC_CTRL_RDIF_CLK),
    };

    xw_errorCode = rl_fecssRfPwrOnOff(c_deviceIndex, &z_rfsRfpwrOn);

    if(M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootSetup: rl_fecssRfPwrOnOff power on failed with %d\n",
                xw_errorCode);
            break;
    }

    /** <LI> Enable APLL using @ref rl_fecssDevClockCtrl API */
    T_RL_API_FECSS_DEV_CLK_CTRL_CMD z_fecssClkOn =
    {
        .c_DevClkCtrl   = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS core on fast clock */
        .c_FtClkCtrl    = M_RL_FECSS_FT_CLK_SRC_XTAL,       /**< Frame Timer on XTAL clock */
        .c_ApllClkCtrl  = M_RL_FECSS_APLL_CTRL_ON_CAL       /**< Calibrate and turn-on the APLL */
    };
    xw_errorCode = rl_fecssDevClockCtrl(c_deviceIndex, &z_fecssClkOn);
    if(M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_fecssDevClockCtrl APLL on failed with %d\n",
                xw_errorCode);
        break;
    }

    /** <LI> @b IF Cold Boot: Setup Calibration data and temperature measurement */
    if (M_RL_FECSS_PWR_ON_MODE_COLD == c_bootMode)
    {
        xw_errorCode = rlExample_setupCalibrationData(c_deviceType, c_deviceIndex, c_calDataSelect,
            c_logEnable, p_txRxCalData, p_fullCalData);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootSetup: rlExample_setupCalibrationData failed with %d\n",
                    xw_errorCode);
            break;
        }

        T_RL_API_FECSS_TEMP_MEAS_CMD z_fecssTempCfg =
        {
            .h_TempCtrlBitMask = 0x311U,    /**< Enable all temperature sensors */
        };

        xw_errorCode = rl_fecssTempMeasCfg(c_deviceIndex, &z_fecssTempCfg);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootSetup: rl_fecssTempMeasCfg failed with %d\n",
                    xw_errorCode);
            break;
        }
    }

    } while (M_FALSE);

    return xw_errorCode;
}


/*!*****************************************************************************
 * @brief Apply calibration data based on temperature change
 *
 * @b Description @n
 * This function measures temperature and applies the correct set of calibration data based
 * on change in junction temperature
 *
 * @param[in]       c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]       c_deviceIndex   : mmWave DFP Device index
 * @param[in]       c_logEnable     : Enable logger print [0: Disable]
 * @param[in]       c_bootMode      : Calibration temperature
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_Example_applyCalibrationDataOnTempChange(UINT8 c_deviceType, UINT8 c_deviceIndex,
                UINT8 c_logEnable, SINT16* p_calTemp)
{
    T_RETURNTYPE    xw_errorCode = M_DFP_RET_CODE_OK;

    T_RL_API_FECSS_TEMP_MEAS_RSP z_fecssTempRes;
    xw_errorCode += rl_fecssTempMeasTrig(c_deviceIndex, &z_fecssTempRes);

    if(M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_applyCalibrationData: temperature measurement failed with %d\n",
                xw_errorCode);
    }
    else
    {
        /** -# Compute average analog temperature using Tx, Rx and PM temperature sensors */
        SINT16 xh_avgAnaTemp = (z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_RX0] +
                    z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_TX0] +
                    z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_PM]) / 3;

        /** -# @b IF |Current_Temperature - last_Cal_Temperature| > Threshold @b THEN trigger runtime calibration */
        *p_calTemp = rlExample_runtimeCalHandler(xh_avgAnaTemp, *p_calTemp, 0xCE, c_deviceIndex, c_logEnable);
    }

    return xw_errorCode;
}





/*!*****************************************************************************
 * @brief Warm boot example
 *
 * @b Description @n
 * This example demonstrates initialization and frame trigger in warm boot mode along with
 * calibration restore/ run. FECSS data memories are retained in warm boot hence monitoring
 * and calibration configurations are retained. However hardware register configurations are lost
 * and must be reconfigured.
 *
 * @param[in]   c_deviceType            : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   c_calDataSelect         : Calibration data select [ 0: Calibrate After Boot, 1: Restore full calibration data, 2: Restore only Tx-Rx calibration data]
 * @param[in]   p_refLinkCbData         : Pointer to reference mmWaveLink initialization callback data (Reference: @ref z_RlExampleLinkClientCbData)
 * @param[in]   p_txRxCalData           : Pointer to Tx-Rx calibration data (Reference: @ref z_RlExampleTxRxCalData)
 * @param[in]   p_fullCalData           : Pointer to full calibration data (Reference: @ref z_RlExampleFactCalData)
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Assumptions @n
 * -# Timing and frequencies are configured for 40MHz XTAL
 * -# @ref rlExample_delayUs and @ref rlExample_logger are provided by the application
 *
 * @b Documentation @n
 * Following diagram describes the overall flow
 *
 * @startuml
 *  (*) -[#blue]->[Cold Boot] rl_mmWaveLinkInit
 * --> "Set PreviousCalTemp = -1000"
 *  partition "rlExample_warmBootSetup" #technology {
 *     --> ==BootEntry==
 *     IF "If First Boot"  Then
 *         -[#blue]->[Yes] "rl_fecssDevPwrOn Cold Boot" as powerOnCold
 *     ELSE
 *         --> "rl_fecssDevPwrOn Warm Boot" as powerOnWarm
 *     ENDIF
 *
 *     powerOnCold -[#blue]->[Cold Boot] "rl_fecssRfPwrOnOff: Power On" as rl_fecssRfPwrOn
 *     powerOnWarm --> "rl_fecssRfPwrOnOff: Power On" as rl_fecssRfPwrOn
 *
 *     --> "rl_fecssDevClockCtrl: APLL ON" as rl_fecssDevClockCtrlOn
 *
 *     IF "If First Boot" Then
 *         -[#blue]->[Yes] rlExample_setupCalibrationData
 *         -[#blue]-> rl_fecssTempMeasCfg
 *         --> ===SetupExit=====
 *     ELSE
 *         -->[No] ===SetupExit=====
 *     ENDIF
 *  }
 *
 *  partition "rlExample_setupChirpProfile" #aquamarine {
 *     --> rl_sensChirpProfComnCfg
 *     --> rl_sensChirpProfTimeCfg
 *     --> rl_sensFrameCfg
 *  }
 *
 *  partition "rlExample_warmBootApplyCalibrationData #strategy {
 *     --> rl_fecssTempMeasTrig
 *     IF "abs(CurrentTemperature - PreviousCalTemp) > Threshold" THEN
 *         -->[Yes] "Compute calibration temperature bin index"
 *         --> rl_fecssRfRuntimeCal
 *         --> Set PreviousCalTemp = CurrentTemperature
 *         --> ==CalExit==
 *     ELSE
 *         -->[NO] ==CalExit==
 *  }
 *
 *  partition "Trigger Frames" #application {
 *      --> rl_sensSensorStart
 *      --> Wait for frames to finish
 *      --> rl_sensStatusGet
 *      -up->[loop] rl_sensSensorStart
 *  }
 *
 *  partition "Device Power Off" #implementation {
 *      rl_sensStatusGet -left-> "rl_fecssRfPwrOnOff: Power Off" as rl_fecssRfPwrOffWarm
 *      -up-> "rl_fecssDevClockCtrl: APLL Off" as rl_fecssDevClockCtrlOff
 *      -up-> "rl_fecssDevPwrOff Retain" as devPwrOffWarm
 *  }
 *
 *  devPwrOffWarm -up->[loop] ==BootEntry==
 *
 *  rl_fecssDevClockCtrlOff -right[#blue]->[Exit] "rl_fecssDevPwrOff Don't Retain" as devPwrOffCold
 *  --> rl_mmWaveLinkDeInit
 *  --> (*)
 * @enduml
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_warmBootLoop(
                UINT8                               c_deviceType,
                UINT8                               c_calDataSelect,
                UINT16                              h_numLoops,
                const T_DFP_CLIENT_CB_DATA*         p_refLinkCbData,
                const T_RL_API_FECSS_RXTX_CAL_DATA* p_txRxCalData,
                const T_RL_API_FECSS_FACT_CAL_DATA* p_fullCalData)
{
    T_RETURNTYPE    xw_errorCode    = M_DFP_RET_CODE_OK;
    UINT8           c_logEnable     = M_ENABLE;
    UINT16          h_loopIndex     = 0U;
    # if defined(_MODE_INSTR_)
    UINT8           c_numFrames     = 1U;
    # else
    UINT8           c_numFrames     = 2U;
    # endif

    /** -# Setup device index and device map for SoC platform */
    UINT32          w_deviceMap     = M_DFP_DEVICE_MAP_DEVICE_0;
    UINT8           c_deviceIndex   = 0U;

    /** -# @b SET Last_Calibration_Temperature = -1000 */
    SINT16 xh_lastCalTemp = -1000;

    /** -# Initialize mmWaveLink using @ref rl_mmWaveLinkInit API
     *      - Update @ref T_DFP_CLIENT_CB_DATA according to the selected device
     */
    T_DFP_CLIENT_CB_DATA    z_mmWaveLinkClientCbData = *p_refLinkCbData;
    z_mmWaveLinkClientCbData.c_DeviceType = c_deviceType;

    xw_errorCode = rl_mmWaveLinkInit(w_deviceMap, z_mmWaveLinkClientCbData);
    if(xw_errorCode != M_DFP_RET_CODE_OK)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: rl_mmWaveLinkInit failed with %d in iteration %d\n",
                xw_errorCode, h_loopIndex);
    }

    /** -# @b FOR 10 Iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Power on FECSS and setup RF */
        # if defined(_MODE_INSTR_)
        UINT8 c_bootMode = M_RL_FECSS_PWR_ON_MODE_WARM;
        # else
        UINT8 c_bootMode = (h_loopIndex == 0U) ? M_RL_FECSS_PWR_ON_MODE_COLD : M_RL_FECSS_PWR_ON_MODE_WARM;
        # endif
        xw_errorCode = rlExample_warmBootSetup(c_deviceType, c_deviceIndex, c_logEnable,
            c_calDataSelect, c_bootMode, p_txRxCalData, p_fullCalData);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: rlExample_warmBootSetup failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Configure profile and frames */
        xw_errorCode = rlExample_setupChirpProfile(c_deviceType, c_deviceIndex, c_logEnable, c_numFrames);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: rlExample_setupChirpProfile failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Apply calibration data based on temperature change */
        xw_errorCode = rl_Example_applyCalibrationDataOnTempChange(c_deviceType, c_deviceIndex, c_logEnable,
            &xh_lastCalTemp);
            if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: rl_Example_applyCalibrationDataOnTempChange failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Trigger frames twice */
        for (UINT8 c_triggerIndex = 0U; c_triggerIndex < 2U; c_triggerIndex++)
        {
            T_RL_API_SENSOR_START_CMD z_fecssSensStart =
            {
                .c_FrameTrigMode        = M_RL_SENS_FRAME_SW_TRIG,
                .c_ChirpStartSigLbEn    = M_RL_SENS_CT_START_SIG_LB_DIS,
                .c_FrameLivMonEn        = 0x0U,
                .w_FrameTrigTimerVal    = 0U
            };
            xw_errorCode = rl_sensSensorStart(c_deviceIndex, &z_fecssSensStart);

            rlExample_delayUs(4000U * c_numFrames);

            T_RL_API_SENSOR_STATUS_RSP z_fecssSensStatus;
            xw_errorCode += rl_sensStatusGet(c_deviceIndex, &z_fecssSensStatus);

            if(M_DFP_RET_CODE_OK != xw_errorCode)
            {
                rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: sensor start failed with %d in iteration %d\n",
                        xw_errorCode, h_loopIndex);
                break;
            }
            else if (z_fecssSensStatus.c_FrameStartStopStatus != 1U)
            {
                rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: incorrect sensor status %d in iteration %d\n",
                        z_fecssSensStatus.c_FrameStartStopStatus, h_loopIndex);
                break;
            }
        }
        if(M_DFP_RET_CODE_OK != xw_errorCode){break;}

        /** <LI> Turn-off RF enables using @ref rl_fecssRfPwrOnOff API */
        T_RL_API_FECSS_RF_PWR_CFG_CMD z_rfsRfpwrOff = {0};
        xw_errorCode = rl_fecssRfPwrOnOff(c_deviceIndex, &z_rfsRfpwrOff);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: rl_fecssRfPwrOnOff power off failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Turn-off APLL using @ref rl_fecssDevClockCtrl API */
                T_RL_API_FECSS_DEV_CLK_CTRL_CMD z_fecssClkOff =
        {
            .c_DevClkCtrl   = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS core on fast clock */
            .c_FtClkCtrl    = M_RL_FECSS_FT_CLK_SRC_XTAL,       /**< Frame Timer on XTAL clock */
            .c_ApllClkCtrl  = M_RL_FECSS_APLL_CTRL_OFF           /**< Calibrate and turn-on the APLL */
        };
        xw_errorCode = rl_fecssDevClockCtrl(c_deviceIndex, &z_fecssClkOff);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: rl_fecssDevClockCtrl APLL off failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Turn-off FECSS using @ref rl_fecssDevPwrOff API */
        T_RL_API_FECSS_DEV_PWR_OFF_CMD z_fecssDevOff = {0};
        # if defined(_MODE_INSTR_)
        z_fecssDevOff.c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_ON;
        # else
        z_fecssDevOff.c_RetentionMode = (h_loopIndex < (h_numLoops - 1U)) ?
            M_RL_FECSS_PWR_DOWN_RET_ON : M_RL_FECSS_PWR_DOWN_RET_OFF;
        # endif
        xw_errorCode = rl_fecssDevPwrOff(c_deviceIndex, &z_fecssDevOff);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_warmBootLoop: rl_fecssDevPwrOff failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }

        /** <LI> Wait for 10ms */
        rlExample_delayUs(10U * 1000U);

        h_loopIndex++;
    } /** </OL> */


    /** -# Deinit mmWaveLink using @ref rl_mmWaveLinkDeInit */
    xw_errorCode += rl_mmWaveLinkDeInit(w_deviceMap);

    return xw_errorCode;
}

/*
 * END OF rlexample_coldboot.c FILE
 */


