/*!*****************************************************************************
 * @file rlexample_tempmeas.c
 *
 * @brief mmWave DFP Low TEMP Measurement Example
 *
 * @b Description @n
 * This example demonstrates temp measurement configuration sequence for
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

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief Temp Measurement Example
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
T_RETURNTYPE rlExample_tempMeasLoop(UINT8 c_deviceType, UINT16 h_numLoops)
{
    /** -# T_RETURNTYPE type is SINT32 value */
    T_RETURNTYPE xw_errorCode  = M_DFP_RET_CODE_OK;
    UINT8 c_devIndex = 0U;
    UINT8 c_logEnable = M_ENABLE;
    UINT16 h_loopIndex = 0U;
    UINT16 h_tempStsMaskRes;

    /** -# Configure temperature measurement using @ref rl_fecssTempMeasCfg */
    T_RL_API_FECSS_TEMP_MEAS_CMD z_fecssTempCfg =
    {
        .h_TempCtrlBitMask = 0x311U
    };

    xw_errorCode = rl_fecssTempMeasCfg(c_devIndex, &z_fecssTempCfg);
    if(M_DFP_RET_CODE_OK != xw_errorCode)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_tempMeasLoop: rl_fecssTempMeasCfg failed with error code %d\n",
                xw_errorCode, h_loopIndex);
    }

    /** -# @b FOR required number of iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Measure temperature using @ref rl_fecssTempMeasTrig */
        T_RL_API_FECSS_TEMP_MEAS_RSP z_fecssTempRes;
        xw_errorCode = rl_fecssTempMeasTrig(c_devIndex, &z_fecssTempRes);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_tempMeasLoop: rl_fecssTempMeasTrig failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
            break;
        }
        /** <LI> Check Temp measurement status */
        h_tempStsMaskRes = z_fecssTempRes.h_TempStsBitMask;
        if(h_tempStsMaskRes != 0x311U)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_tempMeasLoop: Temp Measurement Status failed %d in iteration %d\n",
                    h_tempStsMaskRes, h_loopIndex);
        }

        h_loopIndex++;
    } /** </OL> */

    return xw_errorCode;
}

/*
 * END OF rlexample_tempmeas.c FILE
 */