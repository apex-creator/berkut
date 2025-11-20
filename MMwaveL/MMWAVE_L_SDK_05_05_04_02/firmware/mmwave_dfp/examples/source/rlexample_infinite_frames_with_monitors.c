/*!*****************************************************************************
 * @file rlexample_infiniteframerfstatus.c
 *
 * @brief mmWave DFP Low Infinite Frame Trigger with RF Status Get Example
 *
 * @b Description @n
 * This example demonstrates infinite frame trig with rf status get configuration
 * sequence for mmWaveLow devices
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
 * 0.1     16Mar2023   TI      NA                  Initial Version
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

#include <fecsslib/rfscripter/include/rfs_driver.h>

#include <examples/include/rlexample.h>

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

T_RETURNTYPE rlExample_configureAllMonitors(UINT8 c_deviceType, UINT8 c_deviceIndex, UINT8 c_logEnable)
{
    T_RETURNTYPE xw_errorCode;
    T_RETURNTYPE xw_returnCode = M_DFP_RET_CODE_OK;

    /** -# Configure PLL control voltage monitor using @ref rl_monPllCtrlVoltCfg */
    T_RL_API_MON_PLL_CTRL_VOLT_CFG z_monPllCtrCfg =
    {
        .c_PllMonEnaMask = 0x3U,
    };
    xw_errorCode = rl_monPllCtrlVoltCfg(c_deviceIndex, &z_monPllCtrCfg);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_configureAllMonitors: rl_monPllCtrlVoltCfg failed with error code %d\n",
                xw_errorCode);
    }
    xw_returnCode += xw_errorCode;

    UINT16 h_monFrequencyCode = (M_DFP_DEVICETYPE_6432 == c_deviceType) ?
        ((M_RL_SENS_CHIRP_RFFREQ_LR_57G + M_RL_SENS_CHIRP_RFFREQ_LR_64G) / 2U) :
        ((M_RL_SENS_CHIRP_RFFREQ_LR_76G + M_RL_SENS_CHIRP_RFFREQ_LR_81G) / 2U);

    UINT16 xh_monSlopeCode = (M_DFP_DEVICETYPE_6432 == c_deviceType) ? (UINT16)0x4DU : (UINT16)0x3A;

    /**
     * -# Configure Tx dependent monitors for all Tx channel:
     *  - Tx-Rx loopback monitor using @ref rl_monTxNRxLbCfg
     *  - Tx power monitor using @ref rl_monTxNPowerCfg
     *  - Tx Ball-Break monitor using @ref rl_monTxNBbCfg
     *  - Tx DC-BIST monitor @ref rl_monTxNDcSigCfg
     * */
    T_RL_API_MON_TXN_RX_LB_CFG z_monTxNRxLbCfg =
    {
        .c_MonEnaCtrl       = 0x7U,
        .xh_RfFreqSlope     = xh_monSlopeCode,
        .h_RfFreqStart      = h_monFrequencyCode,
    };

    T_RL_API_MON_TXN_POWER_CFG z_monTxNPwrCfg =
    {
        .xh_RfFreqSlope     = xh_monSlopeCode,
        .h_RfFreqStart      = h_monFrequencyCode,
    };

    T_RL_API_MON_TXN_BB_CFG z_monTxNBbCfg =
    {
        .xh_RfFreqSlope     = xh_monSlopeCode,
        .h_RfFreqStart      = h_monFrequencyCode,
    };

    T_RL_API_MON_TXN_DCSIG_CFG z_monTxNDcSigCfg =
    {
        .xh_RfFreqSlope     = xh_monSlopeCode,
        .h_RfFreqStart      = h_monFrequencyCode,
    };
    for (UINT8 c_txIndex = 0U; c_txIndex < M_RL_MAX_TX_CHANNELS; c_txIndex++)
    {
        z_monTxNRxLbCfg.c_TxIndSel = c_txIndex;
        xw_errorCode = rl_monTxNRxLbCfg(c_deviceIndex, &z_monTxNRxLbCfg);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_configureAllMonitors: rl_monTxNRxLbCfg failed with error code %d for Tx index %d\n",
                    xw_errorCode, c_txIndex);
        }
        xw_returnCode += xw_errorCode;

        z_monTxNPwrCfg.c_TxIndSel = c_txIndex;
        xw_errorCode = rl_monTxNPowerCfg(c_deviceIndex, &z_monTxNPwrCfg);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_configureAllMonitors: rl_monTxNPowerCfg failed with error code %d for Tx index %d\n",
                    xw_errorCode, c_txIndex);
        }
        xw_returnCode += xw_errorCode;

        z_monTxNBbCfg.c_TxIndSel = c_txIndex;
        xw_errorCode = rl_monTxNBbCfg(c_deviceIndex, &z_monTxNBbCfg);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_configureAllMonitors: rl_monTxNBbCfg failed with error code %d for Tx index %d\n",
                    xw_errorCode, c_txIndex);
        }
        xw_returnCode += xw_errorCode;

        z_monTxNDcSigCfg.c_TxIndSel = c_txIndex;
        xw_errorCode = rl_monTxNDcSigCfg(c_deviceIndex, &z_monTxNDcSigCfg);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_configureAllMonitors: rl_monTxNDcSigCfg failed with error code %d for Tx index %d\n",
                    xw_errorCode, c_txIndex);
        }
        xw_returnCode += xw_errorCode;
    }

    /** -# Configure Rx-HPF DC signal and PM Clock DC Signal monitors using @ref rl_monRxHpfDcSigCfg and @ref rl_monPmClkDcSigCfg respectively */
    T_RL_API_MON_RX_HPF_DCSIG_CFG z_monRxDcSigCfg =
    {
        .h_RfFreqStart  = h_monFrequencyCode,
        .c_MonEnaCtrl   = 0x3U,
        .c_RxHpfSel     = M_RL_SENS_RX_HPF_SEL_350KHZ,
    };
    xw_errorCode = rl_monRxHpfDcSigCfg(c_deviceIndex, &z_monRxDcSigCfg);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_configureAllMonitors: rl_monRxHpfDcSigCfg failed with error code %d\n",
            xw_errorCode);
    }
    xw_returnCode += xw_errorCode;

    T_RL_API_MON_PMCLK_DCSIG_CFG z_monPmLoDcSigCfg =
    {
        .h_RfFreqStart = h_monFrequencyCode,
    };
    xw_errorCode = rl_monPmClkDcSigCfg(c_deviceIndex, &z_monPmLoDcSigCfg);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_configureAllMonitors: rl_monPmClkDcSigCfg failed with error code %d\n",
            xw_errorCode);
    }
    xw_returnCode += xw_errorCode;

    return xw_returnCode;
}


/*!*****************************************************************************
 * @brief Setup chirp profile for infinite frame trigger example
 *
 * @b Description @n
 * This function configures chirp and frame parameters for infinite frame trigger example
 *
 * @param[in]   c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   c_deviceIndex   : mmWave DFP Device index
 * @param[in]   c_logEnable     : Enable logger print [0: Disable]
 * @param[in]   c_numFrames     : Number of frames
 * @param[in]   c_skipFTConfig  : Skip frame timer configuration for subsequent frame configuration
 *
 * @returns     xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_setupChirpProfileForFecssSleep(UINT8 c_deviceType, UINT8 c_deviceIndex, UINT8 c_logEnable,
    UINT8 c_numFrames, UINT8 c_skipFTConfig)
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
        # if defined(_MODE_INSTR_)
        .h_NumOfBurstsInFrame   = 1U,               /**< Number of Bursts: 1 */
        # else
        .h_NumOfBurstsInFrame   = 2U,               /**< Number of Bursts: 2 */
        # endif
        .h_NumOfFrames          = c_numFrames,      /**< Number of frames: 2 */
        .c_MiscSetupMask        = c_skipFTConfig,
    };

    /* Compute burst and frame period in high resolution time units */
    UINT32 w_numChirpsPerBurst = (UINT32)z_sensFrameCfg.h_NumOfChirpsInBurst *
        (z_sensFrameCfg.c_NumOfChirpsAccum == 0U ? 1U : z_sensFrameCfg.c_NumOfChirpsAccum);

    z_sensFrameCfg.w_BurstPeriodicity = (w_numChirpsPerBurst * ((UINT32)z_sensChirpProfTimeCfg.h_ChirpIdleTime +
            (UINT32)z_sensChirpProfComnCfg.h_ChirpRampEndTime)) + (800U * 50U);

    /* Frame Idle Time: 100ms */
    z_sensFrameCfg.w_FramePeriodicity = 100U * 1000U * 40U;

    /** -# Configure frame event timer 0 to interrupt Application 20ms before the next frame starts to wakeup and configure FECSS in time */
    z_sensFrameCfg.w_FrameEvent0TimeCfg   = z_sensFrameCfg.w_FramePeriodicity - (40U * 20U * 1000U);

    xw_errorCode = rl_sensFrameCfg(c_deviceIndex, &z_sensFrameCfg);

    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_setupChirpProfile: rl_sensFrameCfg failed with error code %d\n",
                xw_errorCode);
    }

    return xw_errorCode;
}

T_RETURNTYPE rlExample_infiniteFrameMonTrigger(UINT16 h_loopIndex, UINT8 c_deviceIndex, UINT8 c_logEnable)
{
    T_RETURNTYPE xw_errorCode = M_DFP_RET_CODE_OK;

    UINT64 l_monEnableMasks[] = {0x607ULL, 0x6060000ULL, 0x1e00000000ULL};
    UINT16 h_monSetIndex = h_loopIndex % 3U;

    T_RL_API_MON_ENABLE_TRIG z_monEnable =
    {
        .w_MonitorEnable[0U]= (UINT32)(l_monEnableMasks[h_monSetIndex] & 0xFFFFFFFFU),
        .w_MonitorEnable[1U]= l_monEnableMasks[h_monSetIndex] >> 32U,
        .w_FaultInjEnable   = 0U,
    };
    xw_errorCode = rl_monEnableTrig(c_deviceIndex, &z_monEnable);
    if(M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_infiniteFrameMonTrigger: rl_monEnableTrig failed with %d\n",
                xw_errorCode);
    }

    /** -# Wait for monitor done interrupt */
    while (w_RlExampleMonitorDoneCount <= (UINT32)h_loopIndex){}

    return xw_errorCode;
}

/*!*****************************************************************************
 * @brief Infinite Frame Trigger with RF Status Get Example
 *
 * @b Description @n
 *
 * @param[in]   c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   h_numLoops      : Number of test loops
 * @param[out]  xw_errorCode    : Error-code in case of failure
 *
 * @b Assumptions @n
 * -# FECSS is on and calibration data is applied correctly
 * -# APLL is on
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
T_RETURNTYPE rlExample_infFrameWithMonitorsLoop(UINT8 c_deviceType, UINT16 h_numLoops)
{
    T_RETURNTYPE xw_errorCode  = M_DFP_RET_CODE_OK;
    UINT8 c_devIndex = 0U;
    UINT8 c_logEnable = M_ENABLE;
    UINT16 h_loopIndex = 0U;
    SINT16 xh_calTemp;

    /** -# Define volatile pointer to FECSS frame end counter @n
     *  Alternatively, one can enable Frame-Done interrupt to the AppSS and wait for at least 100us
     * after that to turn off FECSS
    */
    REG32* p_fecssFrameCounter = (REG32*)&(M_FE_RFS_SENS_STATUS_START_ADDRESS->w_FrameFreeRunCount);

    /** -# Configure all monitors */
    xw_errorCode = rlExample_configureAllMonitors(c_deviceType, c_devIndex, c_logEnable);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rlExample_configureAllMonitors failed with error code %d\n",
                xw_errorCode);
    }


    /** -# Configure chirp with infinite frames */
    xw_errorCode = rlExample_setupChirpProfileForFecssSleep(c_deviceType, c_devIndex, c_logEnable, 0U, M_DISABLE);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rlExample_setupChirpProfile failed with error code %d\n",
                xw_errorCode);
    }

    w_RlExampleFrameOffsetCount = 0U;
    w_RlExampleMonitorDoneCount = 0U;

    /** -# Enable frame offset interrupt */
    rlExample_configureInterrupt(M_ENABLE, 36U, rlExample_frameOffsetTimerInterrupt);

    /** -# Enable monitoring done interrupt */
    rlExample_configureInterrupt(M_ENABLE, 4U, rlExample_monitorDoneInterrupt);

    /** -# Start infinite frames using @ref rl_sensSensorStart */
    T_RL_API_SENSOR_START_CMD z_sensStart =
    {
        .c_FrameTrigMode        = M_RL_SENS_FRAME_SW_TRIG,
        .c_ChirpStartSigLbEn    = M_RL_SENS_CT_START_SIG_LB_DIS,
        .c_FrameLivMonEn        = 0x0U,
        .w_FrameTrigTimerVal    = 0U
    };
    xw_errorCode = rl_sensSensorStart(c_devIndex, &z_sensStart);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rl_sensSensorStart failed with error code %d\n",
                xw_errorCode);
    }


    /** -# @b FOR required number of iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Wait for current frame to finish using Sensor Status from FECSS */
        while (*p_fecssFrameCounter <= (UINT32)h_loopIndex){}

        /** <LI> Trigger monitors and wait for monitor done interrupt */
        xw_errorCode = rlExample_infiniteFrameMonTrigger(h_loopIndex, c_devIndex, c_logEnable);

        /** <LI> Turn-off FECSS using @ref rl_fecssDevPwrOff API */
        T_RL_API_FECSS_DEV_PWR_OFF_CMD z_fecssDevOff = {
            .c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_ON,
        };

        /** <LI> Power off APLL using @ref rl_fecssDevClockCtrl API */
        T_RL_API_FECSS_DEV_CLK_CTRL_CMD z_fecssClkOff =
        {
            .c_DevClkCtrl   = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS core on fast clock */
            .c_FtClkCtrl    = M_RL_FECSS_FT_CLK_SRC_XTAL,       /**< Frame Timer on XTAL clock */
            .c_ApllClkCtrl  = M_RL_FECSS_APLL_CTRL_OFF          /**< Turn-off the APLL */
        };
        xw_errorCode = rl_fecssDevClockCtrl(c_devIndex, &z_fecssClkOff);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rl_fecssDevClockCtrl APLL on failed with %d\n",
                    xw_errorCode);
            break;
        }

        xw_errorCode = rl_fecssDevPwrOff(c_devIndex, &z_fecssDevOff);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rl_fecssDevPwrOff failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }

        /** <LI> Wait for Frame timer offset interrupt */
        while (w_RlExampleFrameOffsetCount <= (UINT32)h_loopIndex){}

        /** <LI> Turn-on FECSS in warm boot mode */
        T_RL_API_FECSS_DEV_PWR_ON_CMD z_fecssDevOnApiData =
        {
            .h_XtalClkFreq      = M_RL_FECSS_XTAL_CLK_FREQ_40M,     /**< 40 MHz Crystal */
            .c_ClkSourceSel     = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS running on 80MHz fast clock */
            .c_PowerMode        = M_RL_FECSS_PWR_ON_MODE_WARM,
            .c_FecBootCfg       = 0U,                               /**< Enable boot-time self-tests */
            .c_ChirpTimerResol  = M_RL_FECSS_CHIRP_TIMER_RES_11,    /**< Frequency and time in high resolution */
        };

        xw_errorCode = rl_fecssDevPwrOn(c_devIndex, &z_fecssDevOnApiData);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rl_fecssDevPwrOn failed with %d\n",
                xw_errorCode);
            break;
        }

        /** <LI> Enable APLL using @ref rl_fecssDevClockCtrl API */
        T_RL_API_FECSS_DEV_CLK_CTRL_CMD z_fecssClkOn =
        {
            .c_DevClkCtrl   = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS core on fast clock */
            .c_FtClkCtrl    = M_RL_FECSS_FT_CLK_SRC_XTAL,       /**< Frame Timer on XTAL clock */
            .c_ApllClkCtrl  = M_RL_FECSS_APLL_CTRL_ON           /**< Turn-on the APLL */
        };
        xw_errorCode = rl_fecssDevClockCtrl(c_devIndex, &z_fecssClkOn);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rl_fecssDevClockCtrl APLL on failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** <LI> Force run, runtime calibration */
        xh_calTemp = -1000;
        xw_errorCode = rl_Example_applyCalibrationDataOnTempChange(c_deviceType, c_devIndex, c_logEnable, &xh_calTemp);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rl_fecssDevClockCtrl APLL on failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** -# Configure chirp with infinite frames */
        xw_errorCode = rlExample_setupChirpProfileForFecssSleep(c_deviceType, c_devIndex, c_logEnable, h_numLoops, M_ENABLE);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rlExample_setupChirpProfile failed with error code %d\n",
                    xw_errorCode);
        }

        h_loopIndex++;
    } /** </OL> */

    /** -# Stop the sensor using @ref rl_sensSensorStop */
    T_RL_API_SENSOR_STOP_CMD z_sensStop =
    {
        .c_FrameStopMode = M_RL_SENS_STOP_FRAME
    };
    xw_errorCode = rl_sensSensorStop(c_devIndex, &z_sensStop);

    /** -# Wait for at least 1 frame cycle to finish to allow completion of ongoing frame */
    rlExample_delayUs(4000U);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameWithMonitorsLoop: rl_sensSensorStop failed with error code %d\n",
                xw_errorCode);
    }

    /** -# Disable frame offset interrupt */
    rlExample_configureInterrupt(M_DISABLE, 36U, rlExample_frameOffsetTimerInterrupt);

    /** -# Disable monitoring done interrupt */
    rlExample_configureInterrupt(M_DISABLE, 4U, rlExample_monitorDoneInterrupt);


    return xw_errorCode;
}

/*
 * END OF rlexample_infiniteframerfstatus.c FILE
 */