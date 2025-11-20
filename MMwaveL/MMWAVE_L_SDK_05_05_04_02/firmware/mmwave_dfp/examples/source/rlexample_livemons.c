/*!*****************************************************************************
 * @file rlexample_livemons.c
 *
 * @brief mmWave DFP Low Live Mons Example
 *
 * @b Description @n
 * This example demonstrates live monitors configuration sequence for
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

#include <examples/include/rlexample.h>

/*!
 * @brief Structure to store the per-chirp Rx saturation results
 * - This structure can store tha data for at max 20 chirps/ frame
 */
typedef struct
{
    T_RL_API_MON_LIVE_RX_SATURATION_RESULT z_RxSatResult[20];
} T_SENS_PER_CHIRP_RX_SAT_RESULT_LUT;

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief Live Monitors Example
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
T_RETURNTYPE rlExample_liveMonsLoop(UINT8 c_deviceType, UINT16 h_numLoops)
{
    T_RETURNTYPE xw_errorCode  = M_DFP_RET_CODE_OK;
    UINT8 c_devIndex = 0U;
    UINT8 c_logEnable = M_ENABLE;
    UINT16 h_loopIndex = 0U;
    UINT8 c_numFrames = 2U;
    UINT8 c_synthMonStatusRes;
    UINT8 c_rxSatMonStatusRes;

    do
    {
        /** -# Setup sensor with 2 frames, 2 Bursts/Frame, 10 Chirps/ Bursts */
        xw_errorCode = rlExample_setupChirpProfile(c_deviceType, c_devIndex, c_logEnable, c_numFrames);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: rlExample_setupChirpProfile failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** -# Configure synth live monitor using @ref rl_monLiveSynthFreqCfg */
        T_RL_API_MON_LIVE_SYNTH_FREQ_CFG z_monLiveSynthCfg =
        {
            .xc_ChirpMonStartTime    = 20,      /*!< 4us */
            .h_FreqErrThreshold      = 600U     /*!< ~10MHz */
        };

        xw_errorCode = rl_monLiveSynthFreqCfg(c_devIndex, &z_monLiveSynthCfg);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: rl_monLiveSynthFreqCfg failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** -# Configure Rx saturation live monitor using @ref rl_monLiveRxSaturationCfg */
        T_RL_API_MON_LIVE_RX_SATURATION_CFG z_monLiveRxSatCfg =
        {
            .h_PerChirpRamStartAdd   = ((UINT16)0x1000U & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK)
        };

        xw_errorCode = rl_monLiveRxSaturationCfg(c_devIndex, &z_monLiveRxSatCfg);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: rl_monLiveRxSaturationCfg failed with %d\n",
                    xw_errorCode);
            break;
        }

        /** -# Configure GPADC live monitor using @ref rl_monLiveRxSaturationCfg */
        T_RL_FE_LIVE_GPADC_CTM_MON_INSTR_DATA_STRUCT *p_gpadcInstLut = \
            M_RL_FE_MON_GPADC_INSTR_RAM_START_ADDRESS;
        T_RL_FE_LIVE_GPADC_CTM_MON_RESULT_DATA_STRUCT *p_gpadcResLut = \
            M_RL_FE_MON_GPADC_RESULT_RAM_START_ADDRESS;

        /* Extern signal 6 buffered */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[0].w_ConfigVal = 0x00200000U;
        /* enable | offset | type | skip | collect | param */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[0].w_ParamVal = ((1U << 31) | (0U << 27) | (1U << 24) | \
            (10U << 16) | (4U << 8)| 124U) ;
        /* Extern signal 5 buffered */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[1].w_ConfigVal = 0x00010000U;
        /* enable | offset | type | skip | collect | param */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[1].w_ParamVal = ((1U << 31) | (0U << 27) | (1U << 24) | \
            (10U << 16) | (4U << 8) | 124U) ;
        /* Extern signal 6 non-buffered */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[2].w_ConfigVal = 0x00008000U;
        /* enable | offset | type | skip | collect | param */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[2].w_ParamVal = ((1U << 31) | (0U << 27) | (2U << 24) | \
            (10U << 16) | (4U << 8) | 124U) ;
        /* Extern signal 5 non-buffered */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[3].w_ConfigVal = 0x000004000U;
        /* enable | offset | type | skip | collect | param */
        p_gpadcInstLut->z_MonLiveGpadcCtmInstr[3].w_ParamVal = ((1U << 31) | (0U << 27) | (2U << 24) | \
            (10U << 16) | (4U << 8) | 124U) ;

        T_RL_API_MON_LIVE_GPADC_CTM_CFG z_monLiveGpadcCfg =
        {
            .c_GpadcInstrStartOffset    = 0U,
            .c_NumOfGpadcInstr          = 4U,
        };
        xw_errorCode = rl_monLiveGpadcCtmCfg(c_devIndex, &z_monLiveGpadcCfg);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: rl_monLiveRxSaturationCfg failed with %d\n",
                    xw_errorCode);
            break;
        }

    } while (M_FALSE);

    /** -# @b FOR required number of iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Trigger frames */
        T_RL_API_SENSOR_START_CMD z_sensStart =
        {
            .c_FrameTrigMode = M_RL_SENS_FRAME_SW_TRIG,
            .c_ChirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS,
            .c_FrameLivMonEn = 0x3U,
            .w_FrameTrigTimerVal = 0U
        };
        xw_errorCode = rl_sensSensorStart(c_devIndex, &z_sensStart);

        rlExample_delayUs(4000U * c_numFrames);

        /** <LI> Check sensor status */
        T_RL_API_SENSOR_STATUS_RSP z_sensStatus;
        xw_errorCode += rl_sensStatusGet(c_devIndex, &z_sensStatus);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: sensor start failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }
        else if (z_sensStatus.c_FrameStartStopStatus != 1U)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: incorrect sensor status %d in iteration %d\n",
                    z_sensStatus.c_FrameStartStopStatus, h_loopIndex);
            break;
        }

        /** <LI> Check monitor status */
        c_synthMonStatusRes = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS->z_MonLiveSynthFreqResult.c_MonStatus;
        if(c_synthMonStatusRes == 0U)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: Synth monitor status failed %d in iteration %d\n",
                    c_synthMonStatusRes, h_loopIndex);
        }
        T_SENS_PER_CHIRP_RX_SAT_RESULT_LUT *p_perChirpRxSat = \
            (T_SENS_PER_CHIRP_RX_SAT_RESULT_LUT *)(M_RL_SENS_PER_CHIRP_LUT_HW_START_ADDRESS + 0x1000U); /* 4k offset */

        for(UINT8 c_chirpIndex = 0U; c_chirpIndex < 20U; c_chirpIndex++)
        {
            T_RL_API_MON_LIVE_RX_SATURATION_RESULT z_rxSatChirp = p_perChirpRxSat->z_RxSatResult[c_chirpIndex];
            for(UINT8 c_rxIndex = 0U; c_rxIndex < M_RL_MAX_RX_CHANNELS; c_rxIndex++)
            {
                if(z_rxSatChirp.c_ChirpMonData[c_rxIndex] != 0U)
                {
                    rlExample_logger(c_logEnable, "ERROR: rlExample_liveMonsLoop: Rx Sat Chirp Mon data failed with %d in chirp %d & iteration %d\n",
                            z_rxSatChirp.c_ChirpMonData[c_rxIndex], c_chirpIndex, h_loopIndex);
                }
            }
        }

        h_loopIndex++;
    } /** </OL> */
    return xw_errorCode;
}

/*
 * END OF rlexample_livemons.c FILE
 */