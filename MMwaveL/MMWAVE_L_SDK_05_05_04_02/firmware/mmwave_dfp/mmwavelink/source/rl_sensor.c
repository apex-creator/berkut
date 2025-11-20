/*!*****************************************************************************
 * @file rl_sensor.c
 *
 * @brief mmWaveLink sensor module API configuration functions
 *
 * @b Description @n
 * This file provides mmWaveLink sensor module API function calls, application
 * shall call these APIs to configure the sensor and generate waveforms.
 * Refer API documentation for more details.
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
 * @addtogroup MMWL_SENSOR_API mmWaveLink Sensor Control API functions
 * @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     07July2021  TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <mmwavelink/mmwavelink.h>
#include <mmwavelink/include/rl_sensor.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Sensor Driver static data structure
 * @{
 */

/*! @} */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */
/****************************mmWaveLink Static Functions*************************/
/*!
 * @name mmWaveLink Library Sensor module static function calls
 * @{
 */

/*! @} */


/******************************mmWaveLink APIs*********************************/

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Profile common configuration API function
 *
 * @b Description @n
 * This API function configures the FMCW radar chirp profile common parameters like
 * sample rate, num of samples, ramp end time, TX/RX gain etc.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_SENS_CHIRP_PROF_COMN_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-332, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PROF_COMN_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensChirpProfComnCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensChirpProfComnCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor chirp profile common config API  */
            xw_return = fe_sensChirpProfComnCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensChirpProfComnCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink GET API Sensor Profile common configuration API function
 *
 * @b Description @n
 * This API function get the configuration of the FMCW radar chirp profile common parameters like
 * sample rate, num of samples, ramp end time, TX/RX gain etc.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_SENS_CHIRP_PROF_COMN_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-333, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PROF_COMN_CFG_GET
 *
 *******************************************************************************
 */

T_RETURNTYPE rl_sensChirpProfComnCfgGet(UINT8 c_devIndex, \
                    T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensChirpProfComnCfgGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor chirp profile common config get API  */
            xw_return = fe_sensChirpProfComnCfgGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensChirpProfComnCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}
/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Profile timing configuration API function
 *
 * @b Description @n
 * This API function configures the FMCW radar chirp profile timing parameters like
 * idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and BPM enable.
 * These parameters are common for all chirps in a frame.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_SENS_CHIRP_PROF_TIME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * @note This API is not required to be issued if @ref rl_sensPerChirpCfg is used to
 * configure the chirp parameters.
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-334, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PROF_TIME_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensChirpProfTimeCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensChirpProfTimeCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor chirp profile time config API  */
            xw_return = fe_sensChirpProfTimeCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensChirpProfTimeCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Profile timing configuration Get API function
 *
 * @b Description @n
 * This API function get the configuration of the FMCW radar chirp profile timing parameters like
 * idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and BPM enable.
 * These parameters are common for all chirps in a frame.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_SENS_CHIRP_PROF_TIME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * @note This API is not required to be issued if @ref rl_sensPerChirpCfg is used to
 * configure the chirp parameters.
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-335, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PROF_TIME_CFG_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensChirpProfTimeCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensChirpProfTimeCfgGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor chirp profile time config get API  */
            xw_return = fe_sensChirpProfTimeCfgGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensChirpProfTimeCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Per chirp timing param configuration API function
 *
 * @b Description @n
 * This API function configures the FMCW radar per chirp timing parameters like
 * idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and BPM enable.
 * This API allows user to vary or introduce dithers for each chirp timing parameters.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_SENS_PER_CHIRP_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * @note The @ref rl_sensChirpProfTimeCfg is not required to be issued when this per chirp
 * API is used.
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-336, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PER_CHIRP_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensPerChirpCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CFG *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensPerChirpCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor per chirp param config API  */
            xw_return = fe_sensPerChirpCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensPerChirpCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Per chirp timing param configuration Get API function
 *
 * @b Description @n
 * This API function get the configuration of the FMCW radar per chirp timing parameters like
 * idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and BPM enable.
 * This API allows user to vary or introduce dithers for each chirp timing parameters.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_SENS_PER_CHIRP_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * @note The @ref rl_sensChirpProfTimeCfg is not required to be issued when this per chirp
 * API is used.
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-337, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PER_CHIRP_CFG_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensPerChirpCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CFG *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensPerChirpCfgGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor per chirp param config get API  */
            xw_return = fe_sensPerChirpCfgGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensPerChirpCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Per chirp timing param control API function
 *
 * @b Description @n
 * This API function controls the FMCW radar per chirp timing parameters like
 * idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and BPM enable.
 * This API allows user to vary or introduce dithers for each chirp timing parameters.
 *
 * @b Assumption: The per chirp cfg is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_SENS_PER_CHIRP_CTRL
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * @note The @ref rl_sensChirpProfTimeCfg is not required to be issued when this per chirp
 * API is used.
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-338, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PER_CHIRP_CTRL
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensPerChirpCtrl(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CTRL *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensPerChirpCtrl command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor per chirp param control API  */
            xw_return = fe_sensPerChirpCtrl(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensPerChirpCtrl API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Per chirp timing param control Get API function
 *
 * @b Description @n
 * This API function get the control parametrs like the FMCW radar per chirp timing parameters like
 * idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and BPM enable.
 * This API allows user to vary or introduce dithers for each chirp timing parameters.
 *
 * @b Assumption: The per chirp cfg is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_SENS_PER_CHIRP_CTRL
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * @note The @ref rl_sensChirpProfTimeCfg is not required to be issued when this per chirp
 * API is used.
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-339, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_PER_CHIRP_CTRL_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensPerChirpCtrlGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CTRL *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensPerChirpCtrlGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor per chirp param control API  */
            xw_return = fe_sensPerChirpCtrlGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensPerChirpCtrlGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Frame configuration API function
 *
 * @b Description @n
 * This API function configures the FMCW radar chirp, burst and frame configuration
 * parameters like num of chirps, num of chirp accumulation, Burst period, num of bursts
 * Frame period, num of frames and frame event configurations.
 *
 * @b Assumption: The chirp profile common & time cfg or perchirp cfg are done
 * before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_SENS_FRAME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-340, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_FRAME_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensFrameCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_FRAME_CFG *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensFrameCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor Frame config API  */
            xw_return = fe_sensFrameCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensFrameCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Frame configuration Get API function
 *
 * @b Description @n
 * This API function get the configuration of the FMCW radar chirp, burst and frame configuration
 * parameters like num of chirps, num of chirp accumulation, Burst period, num of bursts
 * Frame period, num of frames and frame event configurations.
 *
 * @b Assumption: The chirp profile common & time cfg or perchirp cfg are done
 * before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_SENS_FRAME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-341, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_FRAME_CFG_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensFrameCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_FRAME_CFG *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensFrameCfgGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor Frame config API  */
            xw_return = fe_sensFrameCfgGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensFrameCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Start configuration API function
 *
 * @b Description @n
 * This API function configures and starts the FMCW radar burst and frames. The
 * configuration parameters like frame trigger mode, frame start signal LB enable and
 * frame start timer value.
 *
 * @b Assumption: The frame cfg is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_SENSOR_START_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-342, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_START
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensSensorStart(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_START_CMD *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensorStart command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor start API  */
            xw_return = fe_sensorStart(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensSensorStart API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Stop API function
 *
 * @b Description @n
 * This API function stops the FMCW radar frames and CW mode.
 * The configuration parameter Frame Stop Mode is used to select the stop mode.
 *
 * @b Assumption: The frame start is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_SENSOR_STOP_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-343, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_STOP
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensSensorStop(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_STOP_CMD *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensorStop command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor stop API  */
            xw_return = fe_sensorStop(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensSensorStop API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Status Get API function
 *
 * @b Description @n
 * This API function reads the status of Radar Sensor frame and chrip timers. \n
 * This API provides chirp and frame timer chirp, burst and frame counts. \n
 *
 * @note This API can be used to read the status of the sensor before issuing
 * successive sensor start or stop APIs.
 *
 * @b Assumption: The sensor is configured before issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_SENSOR_STATUS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-344, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_STATUS_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensStatusGet(UINT8 c_devIndex, \
                T_RL_API_SENSOR_STATUS_RSP *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_sensorStatusGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS sensor Status get API  */
            xw_return = fe_sensorStatusGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensStatusGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor dynamic power save disable function
 *
 * @b Description @n
 * This function disables the various sensor inter-chirp and inter-burst dynamic
 * power save options, they are enabled by default.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_SENS_DYN_PWR_SAVE_DIS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-345, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_DYN_PS_DIS
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensDynPwrSaveDis(UINT8 c_devIndex, \
                const T_RL_API_SENS_DYN_PWR_SAVE_DIS *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdSensDynPwrSaveDis command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdSensDynPwrSaveDis(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensDynPwrSaveDis API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink Get API Sensor dynamic power save disable function
 *
 * @b Description @n
 * This function Get config of the various sensor inter-chirp and inter-burst dynamic
 * power save options, they are enabled by default.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_SENS_DYN_PWR_SAVE_DIS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-346, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_DYN_PS_DIS_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensDynPwrSaveStsGet(UINT8 c_devIndex, \
                T_RL_API_SENS_DYN_PWR_SAVE_DIS *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdSensDynPwrSaveStsGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdSensDynPwrSaveStsGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensDynPwrSaveStsGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}
/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Loopback configuration function
 *
 * @b Description @n
 * This function configures the Analog and Digital loop back settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_SENS_DYN_PWR_SAVE_DIS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-347, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_LB_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensLoopBackCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_LOOPBACK_CFG *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdSensLbCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdSensLbCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensLoopBackCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Sensor Loopback Enable function
 *
 * @b Description @n
 * This function Enables the Analog and Digital loop back settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_SENS_LOOPBACK_ENA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-348, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_SENS_LB_ENA
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_sensLoopBackEna(UINT8 c_devIndex, \
                const T_RL_API_SENS_LOOPBACK_ENA *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdSensLbEna command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdSensLbEna(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_sensLoopBackEna API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/* End of group */
/*! @} */

/*
 * END OF rl_sensor.c
 */
