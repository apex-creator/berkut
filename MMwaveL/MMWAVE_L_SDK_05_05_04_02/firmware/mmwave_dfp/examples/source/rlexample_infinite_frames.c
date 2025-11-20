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

#include <examples/include/rlexample.h>

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief Infinite Frame Trigger with RF Status Get Example
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
T_RETURNTYPE rlExample_infFrameRfStatusGetLoop(UINT8 c_deviceType, UINT16 h_numLoops)
{
    T_RETURNTYPE xw_errorCode  = M_DFP_RET_CODE_OK;
    UINT8 c_devIndex = 0U;
    UINT8 c_logEnable = M_ENABLE;
    UINT16 h_loopIndex = 0U;

    /** -# Configure chirp with infinite frame */
    xw_errorCode = rlExample_setupChirpProfile(c_deviceType, c_devIndex, c_logEnable, 0U);
    if (M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameRfStatusGetLoop: rlExample_setupChirpProfile failed with error code %d\n",
                xw_errorCode);
    }

    /** -# @b FOR required number of iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Start infinite frames using @ref rl_sensSensorStart */
        T_RL_API_SENSOR_START_CMD z_sensStart =
        {
            .c_FrameTrigMode = M_RL_SENS_FRAME_SW_TRIG,
            .c_ChirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS,
            .c_FrameLivMonEn = 0x0U,
            .w_FrameTrigTimerVal = 0U
        };
        xw_errorCode = rl_sensSensorStart(c_devIndex, &z_sensStart);

        /** <LI> Wait for 10ms */
        rlExample_delayUs(10000U);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameRfStatusGetLoop: rl_sensSensorStart failed with error code %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
        }

        /** <LI> Stop the sensor using @ref rl_sensSensorStop */
        T_RL_API_SENSOR_STOP_CMD z_sensStop =
        {
            .c_FrameStopMode = M_RL_SENS_STOP_FRAME,
        };
        xw_errorCode = rl_sensSensorStop(c_devIndex, &z_sensStop);

        /** <LI> Wait for at least 1 frame cycle to finish to allow completion of ongoing frame */
        rlExample_delayUs(4000U);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameRfStatusGetLoop: rl_sensSensorStop failed with error code %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }

        /** <LI> Check sensor status using @ref rl_sensStatusGet */
        T_RL_API_SENSOR_STATUS_RSP z_sensStatus;
        xw_errorCode += rl_sensStatusGet(c_devIndex, &z_sensStatus);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameRfStatusGetLoop: sensor start failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }
        else if (z_sensStatus.c_FrameStartStopStatus == 3U)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_infFrameRfStatusGetLoop: incorrect sensor status %d in iteration %d\n",
                    z_sensStatus.c_FrameStartStopStatus, h_loopIndex);
            break;
        }

        h_loopIndex++;
    } /** </OL> */

    /** Stop the sensor using @ref rl_sensSensorStop */
    T_RL_API_SENSOR_STOP_CMD z_sensStop =
    {
        .c_FrameStopMode = M_RL_SENS_FORCE_STOP_FRAME,
    };
    xw_errorCode = rl_sensSensorStop(c_devIndex, &z_sensStop);


    return xw_errorCode;
}

/*
 * END OF rlexample_infiniteframerfstatus.c FILE
 */