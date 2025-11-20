/*!*****************************************************************************
 * @file rlexample_coldboot.c
 *
 * @brief mmWave DFP Low Cold Boot Example
 *
 * @b Description @n
 * This  example demonstrates cold boot API configuration sequence for
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
 * @brief Apply calibration data for Cold Boot Example
 *
 * @b Description @n
 * This function measures temperature and applies the correct set of calibration data
 *
 * @param[in]   c_deviceType            : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   c_deviceIndex           : mmWave DFP Device index
 * @param[in]   c_logEnable             : Enable logger print [0: Disable]
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_applyCalibrationData(UINT8 c_deviceType, UINT8 c_deviceIndex, UINT8 c_logEnable)
{
    T_RETURNTYPE    xw_errorCode;

    /** <LI> Measure current average analog temperature using @ref rl_fecssTempMeasCfg and @ref rl_fecssTempMeasTrig APIs */
    T_RL_API_FECSS_TEMP_MEAS_CMD z_fecssTempCfg =
    {
        .h_TempCtrlBitMask = 0x311U,    /**< Enable all temperature sensors */
    };

    T_RL_API_FECSS_TEMP_MEAS_RSP z_fecssTempRes;

    xw_errorCode = rl_fecssTempMeasCfg(c_deviceIndex, &z_fecssTempCfg);
    xw_errorCode += rl_fecssTempMeasTrig(c_deviceIndex, &z_fecssTempRes);

    if(M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_applyCalibrationData: temperature measurement failed with %d\n",
                xw_errorCode);
    }
    else
    {
        SINT16 xh_avgAnaTemp = (z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_RX0] +
                    z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_TX0] +
                    z_fecssTempRes.xh_TempValue[M_RL_FECSS_TEMP_SENS_PM]) / 3;

        /** <LI> Apply correct calibration data using @ref rl_fecssRfRuntimeCal API */
        (void)rlExample_runtimeCalHandler(xh_avgAnaTemp, -1000, 0xCE, c_deviceIndex, c_logEnable);
    }

    return xw_errorCode;
}

/*!*****************************************************************************
 * @brief Setup chirp profile for Cold Boot Example
 *
 * @b Description @n
 * This function configures chirp and frame parameters for cold boot example.
 *
 * @param[in]   c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   c_deviceIndex   : mmWave DFP Device index
 * @param[in]   c_logEnable     : Enable logger print [0: Disable]
 * @param[in]   c_numFrames     : Number of frames
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_setupChirpProfile(UINT8 c_deviceType, UINT8 c_deviceIndex, UINT8 c_logEnable,
    UINT8 c_numFrames)
{
    T_RETURNTYPE    xw_errorCode = M_DFP_RET_CODE_OK;

    /** -# Select RF slope to cover entire bandwidth*/
    UINT32 w_rampEndTimeUs  = 64U;
    SINT32  xw_slope = (M_DFP_DEVICETYPE_6432 == c_deviceType) ?
        (((M_RL_SENS_CHIRP_RFFREQ_HR_64G - M_RL_SENS_CHIRP_RFFREQ_HR_57G) * 64U) / (w_rampEndTimeUs * 400U)) :
        (((M_RL_SENS_CHIRP_RFFREQ_HR_81G - M_RL_SENS_CHIRP_RFFREQ_HR_76G) * 64U) / (w_rampEndTimeUs * 400U));
    /** -# Setup chirp common configurations using @ref rl_sensChirpProfComnCfg */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG z_sensChirpProfComnCfg =
    {
        .c_DigOutputSampRate    = M_RL_SENS_DIG_OUT_SAMP_RATE_MAX_12P5M,        /**< Sampling rate: 12.5 MSPS */
        .c_DigOutputBitsSel     = M_RL_SENS_DIG_OUT_12BITS_4LSB_ROUND,          /**< Data format: 12-bit with round 4 LSBSs */
        .c_DfeFirSel            = M_RL_SENS_DFE_FIR_LONG_FILT,                  /**< Long FIR filter */
        .c_VcoMultiChipMode     = M_RL_SENS_VCO_MULT_CHIP_SINGLE,               /**< Single chip device */
        .h_NumOfAdcSamples      = 512U,                                         /**< Number of samples: 512 */
        .c_ChirpTxMimoPatSel    = M_RL_SENS_TX_MIMO_PATRN_DIS,                  /**< Disable Tx MIMO pattern */
        .c_MiscSettings         = 0U,                                           /**< HPF FINIT, CRD ena, PA .blank dis */
        .c_HpfFastInitDuration  = 15U,                                          /**< HPF fast init duration: 1.5us */
        .h_CrdNSlopeMag         = 0xA00U,                                       /**< Default CRD Slope: 0xA00 */
        .h_ChirpRampEndTime     = (UINT16)(w_rampEndTimeUs * 50U),              /**< 64us high res */
        .c_ChirpRxHpfSel        = M_RL_SENS_RX_HPF_SEL_350KHZ                   /**< HPF cutoff frequency: 350 KHz */
    };
    xw_errorCode = rl_sensChirpProfComnCfg(c_deviceIndex, &z_sensChirpProfComnCfg);

    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensChirpProfComnCfg failed with error code %d\n",
                xw_errorCode);
    }

    /** -# Setup chirp time configurations using @ref rl_sensChirpProfTimeCfg */
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG z_sensChirpProfTimeCfg =
    {
        .h_ChirpIdleTime        = 5U * 50U,                                             /**< Chirp idle time: 5us in high res */
        .h_ChirpAdcStartTime    = ((UINT16)30U << M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET),/**< Skip samples: 30 */
        .xh_ChirpTxStartTime    = 0,                                                    /**< Tx start time: 0us */
        .xh_ChirpRfFreqSlope    = xw_slope,                                             /**< RF Slope */
        .w_ChirpRfFreqStart     = M_RL_SENS_CHIRP_RFFREQ_HR_76G,                        /**< Lowest supported frequency */
        .h_ChirpTxEnSel         = 0x3U,                                                 /**< Enable both Tx channels */
        .h_ChirpTxBpmEnSel      = 0x0U,                                                 /**< No BPM enable in chirp */
    };
    xw_errorCode = rl_sensChirpProfTimeCfg(c_deviceIndex, &z_sensChirpProfTimeCfg);

    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensChirpProfTimeCfg failed with error code %d\n",
                xw_errorCode);
    }

    /**
     * -# Setup frame configurations using @ref rl_sensFrameCfg
     *      - BurstIdleTime: 150us
     *      - FrameIdleTime: 500us
     *      - TotalChirpsInBurst = ChirpAccumulation * ChirpsPerBurst
     *      - BurstPeriod = ChirpPeriod * TotalChirpsInBurst + BurstIdleTime (High res)
     *      - FramePeriod = BurstPeriod * TotalBurstsInFrame + FrameIdleTime (XTAL clock)
     */
    T_RL_API_SENS_FRAME_CFG z_sensFrameCfg =
    {
        .h_NumOfChirpsInBurst   = 10U,              /**< Number of Chirps: 10 */
        .c_NumOfChirpsAccum     = 0U,               /**< Chirp accumulation: 0 */
        .h_NumOfBurstsInFrame   = 2U,               /**< Number of Bursts: 2 */
        .h_NumOfFrames          = c_numFrames,      /**< Number of frames: 2 */
    };

    /* Compute burst and frame period in high resolution time units */
    UINT32 w_numChirpsPerBurst = (UINT32)z_sensFrameCfg.h_NumOfChirpsInBurst *
        (z_sensFrameCfg.c_NumOfChirpsAccum == 0U ? 1U : z_sensFrameCfg.c_NumOfChirpsAccum);

    z_sensFrameCfg.w_BurstPeriodicity = (w_numChirpsPerBurst * ((UINT32)z_sensChirpProfTimeCfg.h_ChirpIdleTime +
            (UINT32)z_sensChirpProfComnCfg.h_ChirpRampEndTime)) + (1000U * 50U);

    z_sensFrameCfg.w_FramePeriodicity = ((UINT32)z_sensFrameCfg.h_NumOfBurstsInFrame * z_sensFrameCfg.w_BurstPeriodicity) +
        (1000U * 50U);
    /* COnvert frame periodicity to XTAL period units */
    z_sensFrameCfg.w_FramePeriodicity *= 40U;
    z_sensFrameCfg.w_FramePeriodicity /= 50U;

    xw_errorCode = rl_sensFrameCfg(c_deviceIndex, &z_sensFrameCfg);

    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensFrameCfg failed with error code %d\n",
                xw_errorCode);
    }

    /** -# Check the configuration using @ref rl_sensChirpProfComnCfgGet,
     * @ref rl_sensChirpProfTimeCfgGet and @ref rl_sensFrameCfgGet APIs */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG   z_sensChirpProfComnGet;
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG   z_sensChirpProfTimeGet;
    T_RL_API_SENS_FRAME_CFG             z_sensFrameGet;

    xw_errorCode = rl_sensChirpProfComnCfgGet(c_deviceIndex, &z_sensChirpProfComnGet);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensChirpProfComnCfgGet failed with error code %d\n",
                xw_errorCode);
    }
    else if (sizeof(z_sensChirpProfComnGet) != rlExample_memcmp((UINT8*)&z_sensChirpProfComnGet, (UINT8*)&z_sensChirpProfComnCfg, sizeof(z_sensChirpProfComnGet)))
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensChirpProfComnCfgGet didn't match with configured data\n");
    }

    xw_errorCode = rl_sensChirpProfTimeCfgGet(c_deviceIndex, &z_sensChirpProfTimeGet);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensChirpProfTimeCfgGet failed with error code %d\n",
                xw_errorCode);
    }
    else if (sizeof(z_sensChirpProfTimeGet) != rlExample_memcmp((UINT8*)&z_sensChirpProfTimeGet, (UINT8*)&z_sensChirpProfTimeCfg, sizeof(z_sensChirpProfTimeGet)))
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensChirpProfTimeCfgGet didn't match with configured data\n");
    }

    xw_errorCode = rl_sensFrameCfgGet(c_deviceIndex, &z_sensFrameGet);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensFrameCfgGet failed with error code %d\n",
                xw_errorCode);
    }
    else if (sizeof(z_sensFrameGet) != rlExample_memcmp((UINT8*)&z_sensFrameCfg, (UINT8*)&z_sensFrameGet, sizeof(z_sensFrameGet)))
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensFrameCfgGet didn't match with configured data\n");
    }

    return xw_errorCode;
}

/*!*****************************************************************************
 * @brief Calibration data set
 *
 * @b Description @n
 * This function sets up the calibration data in cold boot example for the mmWave Low DFP
 *
 * @param[in]   c_deviceType            : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   c_deviceIndex           : mmWave DFP Device index
 * @param[in]   c_calDataSelect         : Calibration data select [ 0: Calibrate After Boot, 1: Restore full calibration data, 2: Restore only Tx-Rx calibration data]
 * @param[in]   c_logEnable             : Enable logger print [0: Disable]
 * @param[in]   p_txRxCalData           : Pointer to Tx-Rx calibration data
 * @param[in]   p_fullCalData           : Pointer to full calibration data
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_setupCalibrationData(
                UINT8                               c_deviceType,
                UINT8                               c_deviceIndex,
                UINT8                               c_calDataSelect,
                UINT8                               c_logEnable,
                const T_RL_API_FECSS_RXTX_CAL_DATA* p_txRxCalData,
                const T_RL_API_FECSS_FACT_CAL_DATA* p_fullCalData)
{
    T_RETURNTYPE    xw_errorCode = M_DFP_RET_CODE_OK;

    /** -# @b IF c_calDataSelect == 0: Run factory calibration using @ref rl_fecssRfFactoryCal API <OL type="a"> */
    if ((UINT8)0U == c_calDataSelect)
    {
        /** <LI> Calculate mid-band frequency of the functional chirp and select calibration slope */
        UINT16 h_calFreqCode = (M_DFP_DEVICETYPE_6432 == c_deviceType) ?
            ((M_RL_SENS_CHIRP_RFFREQ_LR_57G + M_RL_SENS_CHIRP_RFFREQ_LR_64G) / 2U) :
            ((M_RL_SENS_CHIRP_RFFREQ_LR_76G + M_RL_SENS_CHIRP_RFFREQ_LR_81G) / 2U);

        UINT16 xh_calSlopeCode = (M_DFP_DEVICETYPE_6432 == c_deviceType) ? (UINT16)0x4DU : (UINT16)0x3A;

        UINT8  c_rxGainSel = 38U;

        /** <LI> Setup factory calibration API configuration @ref T_RL_API_FECSS_RF_FACT_CAL_CMD */
        T_RL_API_FECSS_RF_FACT_CAL_CMD z_fecssFactCalCtrl =
        {
            .h_CalCtrlBitMask       = 0xCE,                 /**< Enable Synth, PD, LODIST, Rx and Tx calibrations */
            .c_MiscCalCtrl          = 0x0U,                 /**< Run all calibration without override */
            .h_CalRfFreq            = h_calFreqCode,        /**< Lowest frequency (Low Resolution) */
            .xh_CalRfSlope          = xh_calSlopeCode,      /**< Recommended calibration slope */
            .c_CalRxGainSel         = c_rxGainSel,          /**< Rx gain 38dB Gain */
            .c_CalTxBackOffSel[M_RL_TX_CHANNEL_0] = 0U,     /**< Tx0: 0dB backoff from target power */
            .c_CalTxBackOffSel[M_RL_TX_CHANNEL_1] = 10U,    /**< Tx1: 5dB backoff from target power */
            .c_TxPwrCalTxEnaMask[M_RL_TX_CHANNEL_0] = 0x1U, /**< Enable only Tx0 for Tx0 calibration */
            .c_TxPwrCalTxEnaMask[M_RL_TX_CHANNEL_1] = 0x3U, /**< Enable Tx0 and Tx1 for Tx1 calibration */
        };

        T_RL_API_FECSS_RF_FACT_CAL_RSP z_fecssFactCalRsp;

        /** <LI> Run factory calibration using @ref rl_fecssRfFactoryCal API */
        xw_errorCode = rl_fecssRfFactoryCal(c_deviceIndex, &z_fecssFactCalCtrl, &z_fecssFactCalRsp);

        /** <LI> Check calibration status */
        if ((M_DFP_RET_CODE_OK == xw_errorCode) && ((UINT8)0xCE != z_fecssFactCalRsp.h_CalRunStatus))
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_setupCalibrationData: Factory calibrations failed! Run Status: %d\n",
                z_fecssFactCalRsp.h_CalRunStatus);
            xw_errorCode = M_DFP_RET_CODE_FATAL_ERROR;
        }
        else if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_setupCalibrationData: rl_fecssRfFactoryCal failed with error code %d\n",
                xw_errorCode);
        }
    } /** </OL> */

    /** -# @b ELSE-IF c_calDataSelect == 1: Restore full calibration data using @ref rl_fecssRfFactoryCalDataSet API <OL type="a"> */
    else if ((UINT8)1U == c_calDataSelect)
    {
        xw_errorCode = rl_fecssRfFactoryCalDataSet(c_deviceIndex, p_fullCalData);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_setupCalibrationData: rl_fecssRfFactoryCalDataSet failed with error code %d\n",
                xw_errorCode);
        }
    } /** </OL> */

    /** -# @b ELSE: Restore only Tx-Rx calibration data using @ref rl_fecssRfRxTxCalDataSet API <OL type="a"> */
    else
    {
        xw_errorCode = rl_fecssRfRxTxCalDataSet(c_deviceIndex, p_txRxCalData);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_setupCalibrationData: rl_fecssRfRxTxCalDataSet failed with error code %d\n",
                xw_errorCode);
        }
    } /** </OL> */

    return xw_errorCode;
}

/*!*****************************************************************************
 * @brief Cold boot example
 *
 * @b Description @n
 * This example demonstrates initialization and frame trigger in cold boot mode along with
 * calibration restore/ run. Cold boot resets the FECSS data memories hence all the FECSS configuration
 * APIs must be triggered every time FECSS is enabled. This mode could be useful to save leakage power
 * due to memory retention during long idle time.
 *
 * @param[in]   h_numLoops              : Number of test loops
 * @param[in]   c_deviceType            : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   c_calDataSelect         : Calibration data select [ 0: Calibrate After Boot, 1: Restore full calibration data, 2: Restore only Tx-Rx calibration data]
 * @param[in]   p_refLinkCbData         : Pointer to reference mmWaveLink initialization callback data (Reference: @ref z_RlExampleLinkClientCbData)
 * @param[in]   p_expectedDfpVersion    : Pointer to expected DFP version (pass @ref M_NULL_PTR to disable the check) (Reference: @ref z_RlExampleDfpVersion)
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
 * (*) --> rl_mmWaveLinkInit
 *
 * partition "Device Power Up" #technology {
 *     rl_mmWaveLinkInit -> rl_fecssDevPwrOn
 *
 *     if "p_expectedDfpVersion" then
 *         ->[True] rl_mmWaveDfpVerGet
 *         --> Check DFP Version
 *         --> "rl_fecssRfPwrOnOff: Power On" as rl_fecssRfPwrOn
 *     else
 *         -->[False] rl_fecssRfPwrOn
 *     endif
 *
 *     --> "rl_fecssDevClockCtrl: APLL ON" as rl_fecssDevClockCtrlOn
 * }
 *
 * partition "Setup Profile and Calibrate Device" #strategy {
 *     -right-> rlExample_setupChirpProfile
 *     -up-> rlExample_setupCalibrationData
 *     -up-> rlExample_applyCalibrationData
 * }
 *
 * partition "Frame Trigger" #application {
 *     -right-> rl_sensSensorStart
 *     -up-> Wait for frames to finish
 *     -up-> rl_sensStatusGet
 *     -->[loop] rl_sensSensorStart
 * }
 *
 * partition "Device Power Off" #implementation {
 *     rl_sensStatusGet -left-> "rl_fecssRfPwrOnOff: Power Off" as rl_fecssRfPwrOff
 *     -up-> "rl_fecssDevClockCtrl: APLL Off" as rl_fecssDevClockCtrlOff
 *     -up-> rl_fecssDevPwrOff
 * }
 *
 * rl_fecssDevPwrOff -left->[loop] rl_fecssDevPwrOn
 * rl_fecssDevPwrOff -right->[Exit] rl_mmWaveLinkDeInit
 * --> (*)
 *
 * @enduml
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_coldBootLoop(
                UINT8                               c_deviceType,
                UINT8                               c_calDataSelect,
                UINT16                              h_numLoops,
                const T_DFP_CLIENT_CB_DATA*         p_refLinkCbData,
                const T_RL_API_DFP_FW_VER_GET_RSP*  p_expectedDfpVersion,
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


    /** -# Initialize mmWaveLink using @ref rl_mmWaveLinkInit API
     *      - Update @ref T_DFP_CLIENT_CB_DATA according to the selected device
     */
    T_DFP_CLIENT_CB_DATA    z_mmWaveLinkClientCbData = *p_refLinkCbData;
    z_mmWaveLinkClientCbData.c_DeviceType = c_deviceType;

    xw_errorCode = rl_mmWaveLinkInit(w_deviceMap, z_mmWaveLinkClientCbData);
    if(xw_errorCode != M_DFP_RET_CODE_OK)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_mmWaveLinkInit failed with %d in iteration %d\n",
                xw_errorCode, h_loopIndex);
    }

    /** -# @b FOR 10 Iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {

        /** <LI> Power-on FECSS (Warm Boot) using @ref rl_fecssDevPwrOn */
        T_RL_API_FECSS_DEV_PWR_ON_CMD z_fecssDevOnApiData =
        {
            .h_XtalClkFreq      = M_RL_FECSS_XTAL_CLK_FREQ_40M,     /**< 40 MHz Crystal */
            .c_ClkSourceSel     = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS running on 80MHz fast clock */
            # if defined(_MODE_INSTR_)
            .c_PowerMode        = M_RL_FECSS_PWR_ON_MODE_WARM,      /**< Warm boot mode */
            # else
            .c_PowerMode        = M_RL_FECSS_PWR_ON_MODE_COLD,      /**< Cold boot mode */
            # endif
            .c_FecBootCfg       = 0U,                               /**< Enable boot-time self-tests */
            .c_ChirpTimerResol  = M_RL_FECSS_CHIRP_TIMER_RES_11,    /**< Frequency and time in high resolution */
        };

        xw_errorCode = rl_fecssDevPwrOn(c_deviceIndex, &z_fecssDevOnApiData);

        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_fecssDevPwrOn failed with %d in iteration %d\n",
                xw_errorCode, h_loopIndex);
            break;
        }

        /** <LI> @b IF p_expectedDfpVersion != M_NULL_PTR, Check DFP version number
         *      with expected version using @ref rl_mmWaveDfpVerGet */
        if (M_NULL_PTR != p_expectedDfpVersion)
        {
            T_RL_API_DFP_FW_VER_GET_RSP z_dfpVersion;
            xw_errorCode = rl_mmWaveDfpVerGet(c_deviceIndex, &z_dfpVersion);
            if (M_DFP_RET_CODE_OK != xw_errorCode)
            {
                rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_mmWaveDfpVerGet failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
            }
            if (sizeof(T_RL_API_DFP_FW_VER_GET_RSP) != rlExample_memcmp((UINT8*)&z_dfpVersion, (UINT8*)p_expectedDfpVersion, sizeof(T_RL_API_DFP_FW_VER_GET_RSP)))
            {
                rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_mmWaveDfpVerGet returned incorrect version in iteration %d\n",
                    h_loopIndex);
                break;
            }
        }

        /** <LI> Configure RF front end enables with @ref rl_fecssRfPwrOnOff */
        T_RL_API_FECSS_RF_PWR_CFG_CMD z_rfsRfpwrOn =
        {
            .h_RxChCtrlBitMask = (1U << M_RL_MAX_RX_CHANNELS) - 1U,
            .h_TxChCtrlBitMask = (1U << M_RL_MAX_TX_CHANNELS) - 1U,
            .c_MiscCtrl        = (1U << M_RL_RF_MISC_CTRL_RDIF_CLK),
        };

        xw_errorCode = rl_fecssRfPwrOnOff(c_deviceIndex, &z_rfsRfpwrOn);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_fecssRfPwrOnOff power on failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Setup Chirp profile */
        xw_errorCode = rlExample_setupChirpProfile(c_deviceType, c_deviceIndex, c_logEnable, c_numFrames);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rlExample_setupChirpProfile failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
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
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_fecssDevClockCtrl APLL on failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /**
         * <LI> Setup calibration data @n
         *      @b NOTE: This step can be performed before APLL enable if calibration data is restored to save power
         */
        xw_errorCode = rlExample_setupCalibrationData(c_deviceType, c_deviceIndex, c_calDataSelect,
            c_logEnable, p_txRxCalData, p_fullCalData);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rlExample_setupCalibrationData failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }

        /** <LI> Apply calibration data */
        xw_errorCode = rlExample_applyCalibrationData(c_deviceType, c_deviceIndex, c_logEnable);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rlExample_applyCalibrationData failed with %d in iteration %d\n",
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
                rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: sensor start failed with %d in iteration %d\n",
                        xw_errorCode, h_loopIndex);
                break;
            }
            else if (z_fecssSensStatus.c_FrameStartStopStatus != 1U)
            {
                rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: incorrect sensor status %d in iteration %d\n",
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
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_fecssRfPwrOnOff power off failed with %d in iteration %d\n",
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
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_fecssDevClockCtrl APLL off failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Turn-off FECSS using @ref rl_fecssDevPwrOff API */
        T_RL_API_FECSS_DEV_PWR_OFF_CMD z_fecssDevOff =
        {
            # if defined(_MODE_INSTR_)
            .c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_ON,          /**< FECSS memory retention on */
            # else
            .c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_OFF,         /**< FECSS memory retention off */
            # endif
        };
        xw_errorCode = rl_fecssDevPwrOff(c_deviceIndex, &z_fecssDevOff);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_coldBootLoop: rl_fecssDevPwrOff failed with %d in iteration %d\n",
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


