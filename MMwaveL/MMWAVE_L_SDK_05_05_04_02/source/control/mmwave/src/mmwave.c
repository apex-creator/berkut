/**
 *   @file  mmwave.c
 *
 *   @brief
 *      The file implements the mmWave control module
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2021 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
/* Uncomment below to enable debugp traces*/
#define DebugP_LOG_ENABLED 1 
  
#include <stdint.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr.h>

#include <control/mmwave/mmwave.h>
#include <control/mmwave/include/mmwave_internal.h>
#include <kernel/dpl/HwiP.h>

/**
 * @brief   The RF Monitors IRQ send by FECSS Core
 */
#define FEC_MON_EVENT_IRQ 4

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

/**
 * @brief   This the global variable which track all the information which
 * is required by the mmWave control module.
 */
MMWave_MCB   gMMWave_MCB;


/**************************************************************************
 ************************* Control Functions ******************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to encode the error code. This encoded error
 *      will be returned back to the application.
 *
 *  @param[in]  errorLevel
 *      Error level to be encoded
 *  @param[in]  mmWaveError
 *      Error code for the mmWave
 *  @param[in]  subSysError
 *      Subsystem error code
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Encoded error code
 */
int32_t MMWave_encodeError
(
    MMWave_ErrorLevel   errorLevel,
    int32_t             mmWaveError,
    int32_t             subSysError
)
{
    int32_t     encodedErrorCode;
    uint32_t    tmpEncodedErrorCode;

    /* Determine the error level */
    if (errorLevel == MMWave_ErrorLevel_SUCCESS)
    {
        /* No error: */
        encodedErrorCode = 0;
    }
    else
    {
        /* Warning or Error: */
        tmpEncodedErrorCode = CSL_FMKR (31U, 16U, (uint32_t)mmWaveError);
        tmpEncodedErrorCode = tmpEncodedErrorCode | CSL_FMKR (15U, 2U,  (uint32_t)subSysError);
        tmpEncodedErrorCode = tmpEncodedErrorCode | CSL_FMKR (1U,  0U,  (uint32_t)errorLevel);

        /* Convert the error code into an integer. */
        encodedErrorCode = (int32_t)tmpEncodedErrorCode;
    }
    return encodedErrorCode;
}

/**
 *  @b Description
 *  @n
 *      Application can use the API to decode the error. The mmWave API
 *      can return errors because of either the mmWave or the underlying
 *      mmWave Link API.
 *
 *  @param[in]  errCode
 *      Encoded error code retreived from the mmWave API. This is the error
 *      code which will be decoded.
 *  @param[out] errorLevel
 *      Error level populated by the API.
 *  @param[out] mmWaveError
 *      Error code from the mmWave module. This could be set to 0
 *      to indicate that the mmWave module did not return an error.
 *  @param[out] subSysError
 *      Subsystem error code. This could be set to 0 to indicate that there
 *      was no error at the subsystem level.
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void MMWave_decodeError
(
    int32_t             errCode,
    MMWave_ErrorLevel*  errorLevel,
    int16_t*            mmWaveError,
    int16_t*            subSysError
)
{
    uint16_t    tmpSubSysError;

    /* Get the error level: */
    *errorLevel = (MMWave_ErrorLevel)CSL_FEXTR ((uint32_t)errCode, 1U, 0U);

    /* Determine the error level */
    if (*errorLevel == MMWave_ErrorLevel_SUCCESS)
    {
        /* No error: */
        *mmWaveError = 0;
        *subSysError = 0;
    }
    else
    {
        /* Warning or Error: */
        *mmWaveError = (int16_t)CSL_FEXTR ((uint32_t)errCode, 31U, 16U);

        /* Get the subsystem error: Handle the sign extension correctly. */
        tmpSubSysError = CSL_FEXTR ((uint32_t)errCode, 15U, 2U) << 2;
        tmpSubSysError = tmpSubSysError >> 2;
        *subSysError   = (int16_t)tmpSubSysError;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which given the error code will return
 *      the corresponding error level.
 *
 *  @param[in]  errCode
 *      Encoded error code retreived from the mmWave API. This is the error
 *      code which will be decoded.
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Decoded Error Level
 */
MMWave_ErrorLevel MMWave_decodeErrorLevel (int32_t errCode)
{
    return (MMWave_ErrorLevel)CSL_FEXTR ((uint32_t)errCode, 1U, 0U);
}


/**
 *  @b Description
 *  @n
 *      The function is used to execute the mmWave control module and
 *      this should only be called in the context of an executing task
 *      The function blocks and process mmWave link messages.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_execute (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Pending on the semaphore: Waiting for events to be received */
    SemaphoreP_pend (&(ptrMMWaveMCB->linkSemHandle), SystemP_WAIT_FOREVER);
    DebugP_logInfo("MMWave_execute: Semaphore posted. Will execute MMWave_executeLink\n");

    /* Execute and process all the link messages: */
    retVal = MMWave_executeLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to execute the link; error code is already setup */
        goto exit;
    }


exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave.
 *      This is only applicable in the full configuration mode.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in]  ptrControlCfg
 *      Pointer to the control configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_config (MMWave_Handle mmWaveHandle, MMWave_CtrlCfg* ptrControlCfg, int32_t* errCode)
{
    MMWave_MCB* ptrMMWaveMCB;
    int32_t     retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrControlCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/

    /* Full Configuration Mode: Ensure that the application has opened the mmWave module. */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED) == 0U)
    {
        /* Error: Invalid usage the module should be synchronized before it can be started. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Configure the link: */
    if (MMWave_configLink (ptrMMWaveMCB, ptrControlCfg, errCode) < 0)
    {
        /* Error: Unable to configure the link; error code is already setup. */
        goto exit;
    }

    /* The module has been configured successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_CONFIGURED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to trigger 
 *      temperature measurements
 *  @param[in]  ptrTempStats
 *      Pointer to the temperature measurements demo struct
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *  @pre
 *      MMWave_config
 *
 *  @retval
 *      Void
 */
void MMWave_getTemperatureReport (MMWave_temperatureStats* ptrTempStats)
{
    int32_t retVal = 0;
    T_RL_API_FECSS_TEMP_MEAS_RSP  tempResp;

    retVal = rl_fecssTempMeasTrig(M_DFP_DEVICE_INDEX_0, &tempResp);
    
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Temp measurement failed */
        DebugP_logError("rlTempMeasTrig error %d\n",retVal);
        DebugP_assert(0);
    }
    else
    {
        ptrTempStats->tempStatus = tempResp.h_TempStsBitMask;
        
        /*TODO: Index to be configurable */
        ptrTempStats->tempValue[0] = tempResp.xh_TempValue[0]; //Rx temp
        ptrTempStats->tempValue[1] = tempResp.xh_TempValue[4]; //Tx temp
        ptrTempStats->tempValue[2] = tempResp.xh_TempValue[8]; //PM temp
        ptrTempStats->tempValue[3] = tempResp.xh_TempValue[9]; //DIG temp
    }

}

/**
 *  @b Description
 *  @n
 *      The function enables the GPADC pin selected.
 *  @param[in]  pinsEnabled
 *      Select the GPADC pins to enable
 *      Bit 0: GPADC 1
 *      Bit 1: GPADC 2
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @retval
 *      int32_t
 *      0 - Success
 *      <0- Failure
 */

int32_t MMWave_enableGPADC(uint8_t pinsEnabled)
{
    T_RL_API_FECSS_GPADC_MEAS_CMD gpadccfg;
    uint8_t chan_ctrl;
    uint8_t num_meas;
    uint8_t skip_sampls;
    uint8_t sig_type;
    uint8_t offset;
    uint8_t enable;
    int32_t retVal = 0;

  
    if((pinsEnabled & GPADCPIN1_ENABLE) == GPADCPIN1_ENABLE)
    {
        /*GPADC Pin 1 Configuration*/
        chan_ctrl=124;          
        num_meas=4;
        skip_sampls=10;
        sig_type=2;
        offset=0;
        enable=1;
        
        /*Setting the Local Structure with configuration values of External GPADC Signal 1 non-buffered*/
        gpadccfg.w_ParamVal[0]=(chan_ctrl) |(num_meas << 8U ) | (skip_sampls << 16U ) | (sig_type << 24U)| (offset << 27U)|(enable << 31U);
        gpadccfg.w_ConfigVal[0]=0x00004000;  
    }
    if((pinsEnabled & GPADCPIN2_ENABLE) == GPADCPIN2_ENABLE)
    {
        /*GPADC Pin 2 Configuration*/
        chan_ctrl=124;
        num_meas=4;
        skip_sampls=10;
        sig_type=2;
        offset=0;
        enable=1;

        /*Setting the Local Structure with configuration values of External GPADC Signal 2 non-buffered*/
        gpadccfg.w_ParamVal[1]=(chan_ctrl) |(num_meas << 8U ) | (skip_sampls << 16U ) | (sig_type << 24U)| (offset << 27U)|(enable << 31U);
        gpadccfg.w_ConfigVal[1]=0x00008000;   
    }

    /* Call FECSS GAPDC Meas CFG API  */
    retVal= rl_fecssGpadcMeasCfg(0,&gpadccfg);              

 return retVal;
    
}

/**
 *  @b Description
 *  @n
 *      The function triggers measurement on configured GPADC pins and returns the measured value.
 *  @param[out]  GpadcPin1Vol
 *      pointer to the variable storing the measurement value of ADC data from External GPADC Signal 1.
 *  @param[out]  GpadcPin2Vol
 *      pointer to the variable storing the measurement value of ADC data from External GPADC Signal 2.
 * 
 *      Note: 
 *      1) Max Voltage that can be measured is 1.8V.
 *      2) The user has to ensure that addresses to the variables storing the measurement values of the enabled pins
 *      are passed. If not used/configured pass "NULL".
 *      3) If the values read is "-1" it indicates that GPADC is not configured for measurement or measurement failed.
 * 
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_enableGPADC
 *
 *  @retval
 *      Void
 */
void MMWave_readGPADC(float *GpadcPin1Vol , float *GpadcPin2Vol)
{
    T_RL_API_FECSS_GPADC_MEAS_RSP gpadcrsp;
    int32_t retVal = 0;
    uint8_t status;
   
    
    /* Call FECSS GAPDC Meas Trig API  */
    retVal= rl_fecssGpadcMeasTrig(0,&gpadcrsp);

    if ((retVal != 0))
    {
        /* Error: GPADC measurement failed */
        DebugP_log("error %d\n",retVal);
    }
    else
    {           
                /* Checking which GPADC pins are enabled and Passed */
                status= gpadcrsp.c_GpadcChRunStatus;
                
                /* Checking if pointer storing GPADC PIN 1 Measurement value is not NULL */
                if(GpadcPin1Vol!=NULL)
                {
                    /* Checking if GPADCPIN1 is Configured and passed */
                    if ((status & GPADCPIN1_ENABLE) == GPADCPIN1_ENABLE)
                    {
                        
                        *GpadcPin1Vol=(float)(gpadcrsp.c_GpadcMeasVal[0]*(1.8/256));
                    }
                    else
                    {
                        /* Error in Measuring value */
                        *GpadcPin1Vol=(float)-1;
                    }
                }

                /* Checking if pointer storing GPADC PIN 2 Measurement value is not NULL */
                 if(GpadcPin2Vol!=NULL)
                {
                    /* Checking if GPADCPIN2 is Configured and passed */
                    if ((status & GPADCPIN2_ENABLE) == GPADCPIN2_ENABLE)
                    {
                        *GpadcPin2Vol=(float)(gpadcrsp.c_GpadcMeasVal[1]*(1.8/256));
                    }
                    else
                    {
                        /* Error in Measuring value */
                        *GpadcPin2Vol=(float)-1;
                    }
                }
                

    }
    
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the mmWave control module after the
 *      configuration has been applied.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *  @pre
 *      MMWave_config (Only in full configuration mode)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_start (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, const MMWave_StrtCfg* ptrStrtcfg, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal =1;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrCalibrationCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    /* Full Configuration Mode: Ensure that the application has configured the mmWave module
     * Only then can we start the module. */
    if (((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_CONFIGURED)   == 0U))
    {
        /* Error: Invalid usage the module should be synchronized before it can be started. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Sanity Check: Ensure that the module has not already been started */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == MMWAVE_STATUS_STARTED)
    {
        /* Error: Invalid usage the module should be stopped before it can be started again. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Start the mmWave link: */
    retVal = MMWave_startLink (ptrMMWaveMCB, ptrStrtcfg, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to start the link; error code is already setup */
        goto exit;
    }

    /* The module has been started successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_STARTED;

    exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the mmWave control module after the
 *      configuration has been applied.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_start
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_stop (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    MMWave_MCB*         ptrMMWaveMCB;
    int32_t             retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == 0U)
    {
        /* Error: Invalid usage the module should be started before it can be stopped. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Stop the mmWave link */
    retVal = MMWave_stopLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to stop the link; error code is setup already */
        goto exit;
    }

    /* The module has been stopped successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status & (~(uint32_t)MMWAVE_STATUS_STARTED);

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the RF monitors
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in] monEnable
 *      The RF monitors to be enabled
 * | Value   | Definition  |
 * |---------|-----------  |
 * | 0      | Monitor Disabled   |
 * | 1      | Monitor Enabled    |
 * Bit Field Definition:
 * | BitField   | Definition  |
 * |---------|-----------  |
 * | Bit[0]  | PLL_CTRL_VOLT_MON ON/OFF control    |
 * | Bit[1]  | TX0_RX_LB_MON ON/OFF control   |
 * | Bit[2]  | TX1_RX_LB_MON ON/OFF control   |
 * | Bit[3]  | TX2_RX_LB_MON ON/OFF control (Reserved)   |
 * | Bit[4]  | TX3_RX_LB_MON ON/OFF control (Reserved)   |
 * | Bit[5]  | TX0_POWER_MON ON/OFF control   |
 * | Bit[6]  | TX1_POWER_MON ON/OFF control   |
 * | Bit[7]  | TX2_POWER_MON ON/OFF control (Reserved)  |
 * | Bit[8]  | TX3_POWER_MON ON/OFF control (Reserved)  |
 * | Bit[9]  | TX0_BALL_BREAK_MON ON/OFF control   |
 * | Bit[10] | TX1_BALL_BREAK_MON ON/OFF control   |
 * | Bit[11] | TX2_BALL_BREAK_MON ON/OFF control (Reserved)  |
 * | Bit[12] | TX3_BALL_BREAK_MON ON/OFF control (Reserved)  |
 * | Bit[13] | TX0_INTRNAL_DC_SIG_MON ON/OFF control   |
 * | Bit[14] | TX1_INTRNAL_DC_SIG_MON ON/OFF control   |
 * | Bit[15] | TX2_INTRNAL_DC_SIG_MON ON/OFF control (Reserved)  |
 * | Bit[16] | TX3_INTRNAL_DC_SIG_MON ON/OFF control (Reserved)  |
 * | Bit[17] | RX_HPF_INTRNAL_DC_SIG_MON ON/OFF control   |
 * | Bit[18] | PM_CLK_INTRNAL_DC_SIG_MON ON/OFF control   |
 * | Bit[19] | DFE_BISTFFT_DIAGNOSTIC ON/OFF control   |
 * | Bit[20] | FECSS_STATIC_REG_READBACK_MON ON/OFF control   |
 * | Bit[31:21]  | Reserved   |
 *  @param[in] isrHandler
 *      Interrupt handler for Monitor Interrupt (IRQ 4)
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_monitorsCfg (MMWave_Handle mmWaveHandle, uint64_t monEnable, T_DFP_PLT_ISR_HANDLER isrHandler)
{
    int32_t retVal = -1;
    MMWave_MCB* ptrMMWaveMCB;
    HwiP_Object HwiObj;
    HwiP_Params HwiParams;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    ptrMMWaveMCB->mmwaveMonitorCfg.w_MonitorEnable[0] = (uint64_t)(monEnable & 0x00000000FFFFFFFF);
    ptrMMWaveMCB->mmwaveMonitorCfg.w_MonitorEnable[1] = (uint64_t)(monEnable>>32U);
    
    /* Configure and enable the FEC_INTR2 (IRQ4) for monitor event*/
    HwiP_Params_init(&HwiParams);
    HwiParams.intNum = 16 + FEC_MON_EVENT_IRQ;
    HwiParams.callback = (HwiP_FxnCallback) isrHandler;
    HwiParams.isPulse = 0;
    retVal = HwiP_construct(&HwiObj, &HwiParams);
    if(0u == retVal)
    {
        HwiP_enableInt((uint32_t)FEC_MON_EVENT_IRQ);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to Enable the RF monitors
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_monitorsCfg
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_enableMonitors (MMWave_Handle mmWaveHandle)
{
    int32_t retVal = -1;
    MMWave_MCB* ptrMMWaveMCB;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Enable the monitors as passed in cfg*/
    retVal = MMWaveMon_enable(ptrMMWaveMCB);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get status of RF monitors
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_getMonitorStatus
 *
 *  @retval
 *      Success -   1 (in bit corresponding to the monitor)
 *  @retval
 *      Failed  -   0 (in bit corresponding to the monitor)
 */
int64_t MMWave_getMonitorStatus (MMWave_Handle mmWaveHandle)
{
    int64_t retVal = 0;
    MMWave_MCB* ptrMMWaveMCB;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Get results of Monitors enabled*/
    retVal = MMWaveMon_getStatus(ptrMMWaveMCB);

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to open the mmWave module.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *  @param[in] ptrCalibrationData
 *      This is the pointer to the calibration data which needs to be passed
 *      to the RadarSS to bypass calibration and restore it to a previously saved
 *      value. Set this to NULL and the mmWave module will request the RadarSS to
 *      perform the calibration. Note this structure consists of multiple sub-calibration
 *      structures.
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_open
(
    MMWave_Handle               mmWaveHandle,
    const MMWave_OpenCfg*       ptrOpenCfg,
    int32_t*                    errCode
)
{
    int32_t         retVal;
    MMWave_MCB*     ptrMMWaveMCB;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    if (ptrOpenCfg->fecRDIFCtrlCmd.c_RdifEnable == M_RL_FECSS_RDIF_ENA)
    {
        retVal = rl_fecssDevRdifCtrl(M_DFP_DEVICE_INDEX_0, &ptrOpenCfg->fecRDIFCtrlCmd);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
            retVal = MINUS_ONE;
            goto exit;
        }
    }

    /* Sanity Check: Validate the arguments */
    if ((ptrMMWaveMCB == NULL) || (ptrOpenCfg == NULL))
    {
        /* Error: Invalid arguments. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/

    /* Sanity Check: Ensure that the module is not already open */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED) == MMWAVE_STATUS_OPENED)
    {
        /* Error: Invalid usage the module should be closed before it can be opened */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Copy over the configuration: */
    memcpy ((void *)&ptrMMWaveMCB->openCfg, (const void*)ptrOpenCfg, sizeof(MMWave_OpenCfg));

    /* Open the mmWave Link: */
    retVal = MMWave_openLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: mmWave Open failed; error code is already setup  */
        goto exit;
    }

    /* The module has been opened successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_OPENED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to close the mmWave module.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_close (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    int32_t         retVal;
    MMWave_MCB*     ptrMMWaveMCB;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid arguments. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Sanity Check: Ensure that the module is not already closed */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED) == 0U)
    {
        /* Error: Invalid usage the module should be opened before it can be closed */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* The module has been closed successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status & (~(uint32_t)(MMWAVE_STATUS_OPENED | MMWAVE_STATUS_CONFIGURED));

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}




/**
 *  @b Description
 *  @n
 *      The function is used to initialize the mmWave control module. This
 *      needs to be invoked before any other control API is invoked.
 *
 *  @param[in]  ptrCtrlInitCfg
 *      Pointer to the control init configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the control module
 *  @retval
 *      Error   -   NULL
 */
MMWave_Handle MMWave_init (MMWave_InitCfg* ptrCtrlInitCfg, int32_t* errCode)
{
    MMWave_MCB*         ptrMMWaveMCB = NULL;
    int32_t             retVal;
    int32_t             status;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if (ptrCtrlInitCfg == NULL)
    {
        /* Error: Invalid argument detected */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Initialize the memory block */
    memset ((void *)&gMMWave_MCB, 0, sizeof(MMWave_MCB));

    /* Populate the devicemap based on the device type*/
    gMMWave_MCB.deviceMap = M_DFP_DEVICE_MAP_DEVICE_0;

    /* Populate the control MCB */
    memcpy ((void *)&gMMWave_MCB.initCfg, (void *)ptrCtrlInitCfg, sizeof(MMWave_InitCfg));


    status = SemaphoreP_constructMutex(&gMMWave_MCB.cfgSemHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&(gMMWave_MCB.linkSemHandle), 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Initialize the mmWave Link module: */
    retVal = MMWave_initLink (&gMMWave_MCB, errCode);
    if (retVal < 0)
    {
        /* Error: Failure to open the mmWave Link; error code is already setup. */
        goto exit;
    }

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        ptrMMWaveMCB = &gMMWave_MCB;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        ptrMMWaveMCB = NULL;
    }
    return (MMWave_Handle)ptrMMWaveMCB;
}

int32_t MMWave_factoryCalibConfig (
    MMWave_Handle    mmWaveHandle,
    MMWave_calibCfg* ptrfactoryCalibCfg, 
    int32_t*         errCode)
{
    MMWave_MCB*         ptrMMWaveMCB = NULL;
    int32_t             retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Sanity Check: Validate the arguments */
    if ((ptrfactoryCalibCfg == NULL) || (ptrMMWaveMCB == NULL))
    {
        /* Error: Invalid argument detected */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Populate the control MCB */
    memcpy ((void *)&gMMWave_MCB.factoryCalCfg, (void *)ptrfactoryCalibCfg, sizeof(MMWave_calibCfg));

    /* If Restore factory calibration data option is selected */
    if(ptrMMWaveMCB->factoryCalCfg.ptrFactoryCalibData != NULL)
    {
        retVal = rl_fecssRfRxTxCalDataSet(M_DFP_DEVICE_INDEX_0, ptrMMWaveMCB->factoryCalCfg.ptrFactoryCalibData);
        if (retVal < 0)
        {
            /* Error: Unable to start the link; error code is already setup */
            goto exit;
        }
    }

    /* Run factory calibration if enabled */
    if (TRUE == ptrMMWaveMCB->factoryCalCfg.isFactoryCalEnabled)
    {
        /* If ATE calibration data is not EFUSED, send the ATE calibration data fetched from flash */
        if(!ptrMMWaveMCB->factoryCalCfg.isATECalibEfused)
        {
            retVal = rl_fecssRfFactoryCalDataSet(M_DFP_DEVICE_INDEX_0, (T_RL_API_FECSS_FACT_CAL_DATA *)(ptrMMWaveMCB->factoryCalCfg.ptrAteCalibration));
            if (retVal < 0)
            {
                /* Error: Unable to start the link; error code is already setup */
                goto exit;
            }
        }

        retVal = rl_fecssRfFactoryCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalCmd, &ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalRsp);
        if((retVal != M_DFP_RET_CODE_OK) || 
           (ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalRsp.h_CalRunStatus != ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask) || 
           (ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalRsp.h_CalResStatus != 
           ( ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask | RFS_FACTORY_CAL_VALIDITY_STATUS)))
           /* Checking Result Status with RFS_FACTORY_CAL_VALIDITY_STATUS (0x2F) is required because, irrespective of their
              * enablement in factory Cal, status of the following calibrations are shown based on ATE calibrations
              * | bits [0]    | APLL calibration validity status.
              * | bits [1]    | VCO calibration validity status
              * | bits [2]    | PD calibration validity status
              * | bits [3]    | LODIST calibration validity status
              * | bits [5]    | RX IFA calibration validity status
              */

        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ERFSBOOTCAL, retVal);
            retVal = MINUS_ONE;
            goto exit;
        }
    }

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize and shutdown the mmWave module
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deinit (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Deinitialize the link: */
    retVal = MMWave_deinitLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Link deinitialization failed. Error code is already setup */
        goto exit;
    }

    /* SOC was deinitialized: Delete the semaphore handle */
    SemaphoreP_destruct(&(ptrMMWaveMCB->linkSemHandle));

    /* Delete the configuration semaphore handle (if available) */
    SemaphoreP_destruct(&(ptrMMWaveMCB->cfgSemHandle));

    /* Reset the memory: */
    memset ((void *)ptrMMWaveMCB, 0, sizeof(MMWave_MCB));

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to synchronize the execution of the control
 *      module between the execution domains. Applications are recommended
 *      to invoke the function until the "sync" is acheived. Failure to
 *      do so will result in the mmWave module misbehaving.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Synchronized    -   1
 *  @retval
 *      Unsynchronized  -   0
 *  @retval
 *      Error           -   <0
 */
int32_t MMWave_sync (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    int32_t     retVal = MINUS_ONE;
    MMWave_MCB*     ptrMMWaveMCB;
    
    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }
    
    /* YES: Set the synchronization flag appropriately */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | (MMWAVE_STATUS_SYNCHRONIZED);
    retVal = 0;
    
    exit:
    return retVal;
}

