/*!*****************************************************************************
 * @file rlexample_perchirplut.c
 *
 * @brief mmWave DFP Low Per-Chirp LUT Example
 *
 * @b Description @n
 * This example demonstrates per-chirp lut configuration sequence for
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
 * 0.1     15Mar2023   TI      NA                  Initial Version
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
#include <platform/xwrlx432/syscore/include/sys_main.h>

#include <examples/include/rlexample.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
/*!
 * @brief  Sensor Perchirp LUT, total 72 bytes used, 4 values per params
 */
typedef struct
{
    UINT32 w_StartFreqHighRes[4U];  /* LUT address 0 */
    SINT16 xh_ChirpSlope[4U];       /* LUT address 16 */
    UINT16 h_ChirpIdleTime[4U];     /* LUT address 24 */
    UINT16 h_ChirpAdcStartTime[4U]; /* LUT address 32 */
    SINT16 xh_ChirpTxStartTime[4U]; /* LUT address 40 */
    UINT8 c_ChirpTxEn[4U];          /* LUT address 48 */
    UINT8 c_ChirpBpmEn[4U];         /* LUT address 52 */
} T_SENS_PER_CHIRP_LUT;

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief Per-Chirp LUT Example
 *
 * @b Description @n
 * This example demonstrates per chirp LUT configuration and frame trigger
 *
 * @b Assumptions @n
 * - FECSS is powered on and configured in High-High resolution
 * - APLL is on and calibration data is applied correctly using factory cand runtime calibration APIs
 *
 * @param[in]   c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   h_numLoops      : Number of test loops
 * @param[out]  xw_errorCode    : Error-code in case of failure
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rlExample_perChirpLutLoop(UINT8 c_deviceType, UINT8 c_deviceIndex,
    UINT8 c_logEnable, UINT16 h_numLoops)
{
    T_RETURNTYPE xw_errorCode  = M_DFP_RET_CODE_OK;
    UINT16 h_loopIndex = 0U;
    UINT8 c_numFrames = 2U;

    /** -# Sensor per chirp LUT configuration at address 0x21880000 */
    T_SENS_PER_CHIRP_LUT *p_sensPerChirpLUT = \
        (T_SENS_PER_CHIRP_LUT*)M_RL_SENS_PER_CHIRP_LUT_HW_START_ADDRESS;

    /** -# Configure start frequencies in high resolution based on device type */
    if (M_DFP_DEVICETYPE_1432 == c_deviceType)
    {
        p_sensPerChirpLUT->w_StartFreqHighRes[0U]    = M_RL_SENS_CHIRP_RFFREQ_HR_76G;
        p_sensPerChirpLUT->w_StartFreqHighRes[1U]    = M_RL_SENS_CHIRP_RFFREQ_HR_81G;
        p_sensPerChirpLUT->w_StartFreqHighRes[2U]    = M_RL_SENS_CHIRP_RFFREQ_HR_78G;
        p_sensPerChirpLUT->w_StartFreqHighRes[3U]    = M_RL_SENS_CHIRP_RFFREQ_HR_80G;
    }
    else
    {
        p_sensPerChirpLUT->w_StartFreqHighRes[0U]    = M_RL_SENS_CHIRP_RFFREQ_HR_57G;
        p_sensPerChirpLUT->w_StartFreqHighRes[1U]    = M_RL_SENS_CHIRP_RFFREQ_HR_64G;
        p_sensPerChirpLUT->w_StartFreqHighRes[2U]    = M_RL_SENS_CHIRP_RFFREQ_HR_59G;
        p_sensPerChirpLUT->w_StartFreqHighRes[3U]    = M_RL_SENS_CHIRP_RFFREQ_HR_61G;
    }

    /** -# Configure chirp slopes */
    p_sensPerChirpLUT->xh_ChirpSlope[0U]    = 1000;             /*!< High positive slope */
    p_sensPerChirpLUT->xh_ChirpSlope[1U]    = -1000;            /*!< High negative slope */
    p_sensPerChirpLUT->xh_ChirpSlope[2U]    = 35;               /*!< Low positive slope */
    p_sensPerChirpLUT->xh_ChirpSlope[3U]    = -35;              /*!< Low negative slope */

    /** -# TX start time in high resolution */
    p_sensPerChirpLUT->xh_ChirpTxStartTime[0U]  = -1 * 50;      /*!< -1us, min idle time: 7us */
    p_sensPerChirpLUT->xh_ChirpTxStartTime[1U]  = 3 * 50;       /*!< 3us, min idle time: 3.1us */
    p_sensPerChirpLUT->xh_ChirpTxStartTime[2U]  = 10 * 50;      /*!< 10us, min idle time: 3.1us */
    p_sensPerChirpLUT->xh_ChirpTxStartTime[3U]  = -4 * 50;      /*!< -4us, min idle time: 10us */


    /** -# Configure chirp idle times
     *  @b NOTE: Check mmWaveLink documentation for the timing information. max(6us - TX_START_TIME, 3.1us)
     */
    p_sensPerChirpLUT->h_ChirpIdleTime[0U]  = 10U * 50U;                                  /*!< 6us */
    p_sensPerChirpLUT->h_ChirpIdleTime[1U]  = 5U * 50U;                                /*!< 10us */
    p_sensPerChirpLUT->h_ChirpIdleTime[2U]  = 4U * 50U;                                 /*!< 6us */
    p_sensPerChirpLUT->h_ChirpIdleTime[3U]  = 15U * 50U;                                /*!< 12us */

    /** -# Configure ADC skip samples */
    p_sensPerChirpLUT->h_ChirpAdcStartTime[0U] = ((UINT16)30U << M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET);
    p_sensPerChirpLUT->h_ChirpAdcStartTime[1U] = ((UINT16)63U << M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET);
    p_sensPerChirpLUT->h_ChirpAdcStartTime[2U] = ((UINT16)0U << M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET);
    p_sensPerChirpLUT->h_ChirpAdcStartTime[3U] = ((UINT16)63U << M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET);


    /** -# Configure Tx Enable LUT */
    p_sensPerChirpLUT->c_ChirpTxEn[0U]  = 0x23;
    p_sensPerChirpLUT->c_ChirpTxEn[1U]  = 0x31;

    /** -# Configure BPM LUT */
    p_sensPerChirpLUT->c_ChirpBpmEn[0U] = 0x12;
    p_sensPerChirpLUT->c_ChirpBpmEn[1U] = 0x30;

    T_RL_API_SENS_PER_CHIRP_CFG z_sensPerChirpCfg ={
        .h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_START]     = 4U,
        .h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_SLOPE]     = 4U,
        .h_ParamArrayLen[M_RL_SENS_PER_CHIRP_IDLE_TIME]      = 4U,
        .h_ParamArrayLen[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = 4U,
        .h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_START_TIME]  = 4U,
        .h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_ENABLE]      = 4U,
        .h_ParamArrayLen[M_RL_SENS_PER_CHIRP_BPM_ENABLE]     = 4U,

        .h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_START]     = 1U,
        .h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_SLOPE]     = 1U,
        .h_ParamRptCount[M_RL_SENS_PER_CHIRP_IDLE_TIME]      = 1U,
        .h_ParamRptCount[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = 1U,
        .h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_START_TIME]  = 1U,
        .h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_ENABLE]      = 1U,
        .h_ParamRptCount[M_RL_SENS_PER_CHIRP_BPM_ENABLE]     = 1U
    };

    T_RL_API_SENS_PER_CHIRP_CTRL z_sensPerChirpCtrl = {
        .h_PerChirpParamCtrl                                      = M_RL_SENS_PER_CHIRP_CTRL_MAX,
        .h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_START]     =
            ((UINT32)(&p_sensPerChirpLUT->w_StartFreqHighRes[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK),
        .h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_SLOPE]     =
            ((UINT32)(&p_sensPerChirpLUT->xh_ChirpSlope[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK),
        .h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_IDLE_TIME]      =
            ((UINT32)(&p_sensPerChirpLUT->h_ChirpIdleTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK),
        .h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_ADC_START_TIME] =
            ((UINT32)(&p_sensPerChirpLUT->h_ChirpAdcStartTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK),
        .h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_START_TIME]  =
            ((UINT32)(&p_sensPerChirpLUT->xh_ChirpTxStartTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK),
        .h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_ENABLE]      =
            ((UINT32)(&p_sensPerChirpLUT->c_ChirpTxEn[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK),
        .h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_BPM_ENABLE]     =
            ((UINT32)(&p_sensPerChirpLUT->c_ChirpBpmEn[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK)
    };

    do
    {
        /** -# Setup basic chirp profile */
        xw_errorCode = rlExample_setupChirpProfile(c_deviceType, c_deviceIndex, c_logEnable, c_numFrames);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_perChirpLutLoop: profile config failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** -# Configure per chirp LUT lengths and repetition counts using rl_sensPerChirpCfg */
        xw_errorCode = rl_sensPerChirpCfg(c_deviceIndex, &z_sensPerChirpCfg);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_perChirpLutLoop: rl_sensPerChirpCfg failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** -# Check configuration with get APIs */
        T_RL_API_SENS_PER_CHIRP_CFG z_sensPerChirpCfgGet;
        xw_errorCode = rl_sensPerChirpCfgGet(c_deviceIndex, &z_sensPerChirpCfgGet);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_perChirpLutLoop: rl_sensPerChirpCfgGet failed with %d\n",
                    xw_errorCode);
            break;
        }
        else if (sizeof(z_sensPerChirpCfgGet) != rlExample_memcmp((UINT8*)&z_sensPerChirpCfgGet, (UINT8*)&z_sensPerChirpCfg, sizeof(z_sensPerChirpCfgGet)))
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_perChirpLutLoop: rl_sensPerChirpCfgGet didn't match\n");
            break;
        }
    } while (M_FALSE);

    /** -# @b FOR required number of iterations: <OL type="a"> */
    while ((h_loopIndex < 2U*h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Enable/Disable per-chirp control using rl_sensPerChirpCtrl API based loop index */
        z_sensPerChirpCtrl.h_PerChirpParamCtrl = ((h_loopIndex & 1U) == 0U) ? M_RL_SENS_PER_CHIRP_CTRL_MAX : 0U;

        xw_errorCode = rl_sensPerChirpCtrl(c_deviceIndex, &z_sensPerChirpCtrl);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_perChirpLutLoop: rl_sensPerChirpCtrl failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** <LI> Verify per chirp control using rl_sensPerChirpCtrlGet API */
        T_RL_API_SENS_PER_CHIRP_CTRL z_sensPerChirpCtrlGet;
        xw_errorCode = rl_sensPerChirpCtrlGet(c_deviceIndex, &z_sensPerChirpCtrlGet);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_perChirpLutLoop: rl_sensPerChirpCtrlGet failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }
        else if (sizeof(z_sensPerChirpCtrlGet) != rlExample_memcmp((UINT8*)&z_sensPerChirpCtrlGet, (UINT8*)&z_sensPerChirpCtrl, sizeof(z_sensPerChirpCtrlGet)))
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_perChirpLutLoop: rl_sensPerChirpCtrlGet doesn't match in iteration %d\n",
                    h_loopIndex);
            break;
        }

        /** <LI> Trigger frame and wait for completion */
        T_RL_API_SENSOR_START_CMD z_fecssSensStart =
        {
            .c_FrameTrigMode        = M_RL_SENS_FRAME_SW_TRIG,
            .c_ChirpStartSigLbEn    = M_RL_SENS_CT_START_SIG_LB_DIS,
            .c_FrameLivMonEn        = 0x0U,
            .w_FrameTrigTimerVal    = 0U
        };
        xw_errorCode = rl_sensSensorStart(c_deviceIndex, &z_fecssSensStart);
        rlExample_delayUs(4000U * c_numFrames);
        h_loopIndex++;
    } /** </OL> */

    return xw_errorCode;
}

/*
 * END OF rlexample_perchirplut.c FILE
 */