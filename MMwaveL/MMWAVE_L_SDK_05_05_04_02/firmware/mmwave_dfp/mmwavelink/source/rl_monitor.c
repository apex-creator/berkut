/*!*****************************************************************************
 * @file rl_monitor.c
 *
 * @brief mmWaveLink Monitor module API configuration functions
 *
 * @b Description @n
 * This file provides mmWaveLink Monitor module API function calls, application
 * shall call these APIs to configure the Monitor and to generate reports.
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
 * @addtogroup MMWL_MONITOR_API mmWaveLink Monitor Control API functions
 * @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     19Sep2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <mmwavelink/mmwavelink.h>
#include <mmwavelink/include/rl_monitor.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Monitor Driver static data structure
 * @{
 */

/*! @} */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */
/****************************mmWaveLink Static Functions*************************/
/*!
 * @name mmWaveLink Library Monitor module static function calls
 * @{
 */

/*! @} */


/******************************mmWaveLink APIs*********************************/

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor Enable and Trigger API function
 *
 * @b Description @n
 * This API function enables and triggers the monitors. All the enabled monitors will be triggered
 * and result will be stored in a predefined IPC/HW RAM with fixed address location,
 * upon completion of all enabled monitors, the FECSS RFS core will send a IRQ event \
 * FEC_INTR2 (IRQ[4]) to APPSS (application core). The application is responsible for
 * reading out all the results from the predefined address. \n
 * The fault injection can be done to each of these enabled monitors in this API.
 *
 * @b Assumption: 1. The FECSS RF ON is done before issuing this API. \n
 * 2. Application shall call this API in interframe interval. \n
 * 3. Apllication shall read all monitor results before next frame starts. \n
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_MON_ENABLE_TRIG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-319, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_ENABLE_TRIG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monEnableTrig(UINT8 c_devIndex, \
                const T_RL_API_MON_ENABLE_TRIG *p_apiCmdData)
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
        *   - call rfs_cmdMonEnableTrig command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS monitor enable and trigger API  */
            xw_return = rfs_cmdMonEnableTrig(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monEnableTrig API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/*!*****************************************************************************
 * @brief mmWaveLink API Monitor Debug PD Power Meas API function
 *
 * @b Description @n
 * This API function measures the PD power under certain test condition.
 *
 * @b Assumption: 1. The FECSS RF ON is done before issuing this API. \n
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_MON_DBG_PD_MEAS_CMD
 * @param[out] p_apiResData  - pointer to command data @ref T_RL_API_MON_DBG_PD_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-320, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_DBG_PD_PWR_MEAS
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monDbgPdMeas(UINT8 c_devIndex, \
                const T_RL_API_MON_DBG_PD_MEAS_CMD *p_apiCmdData, \
                T_RL_API_MON_DBG_PD_MEAS_RSP *p_apiResData)
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
        *   - call rfs_cmdDbgPdMeas command
        * */
        if ((M_NULL_PTR == p_apiCmdData) || (M_NULL_PTR == p_apiResData))
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS monitor Debug PD meas  */
            xw_return = rfs_cmdDbgPdMeas(c_devIndex, p_apiCmdData, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monDbgPdMeas API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor Debug TX Power Meas API function
 *
 * @b Description @n
 * This API function measures the TX power under certain test condition.
 *
 * @b Assumption: 1. The FECSS RF ON is done before issuing this API. \n
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_MON_DBG_TXPWR_MEAS_CMD
 * @param[out] p_apiResData  - pointer to command data @ref T_RL_API_MON_DBG_TXPWR_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-321, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_DBG_TX_PWR_MEAS
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monDbgTxPwrMeas(UINT8 c_devIndex, \
                const T_RL_API_MON_DBG_TXPWR_MEAS_CMD *p_apiCmdData, \
                T_RL_API_MON_DBG_TXPWR_MEAS_RSP *p_apiResData)
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
        *   - call rfs_cmdDbgTxPwrMeas command
        * */
        if ((M_NULL_PTR == p_apiCmdData) || (M_NULL_PTR == p_apiResData))
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS monitor Debug PD meas  */
            xw_return = rfs_cmdDbgTxPwrMeas(c_devIndex, p_apiCmdData, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monDbgTxPwrMeas API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor Live Synth Freq configuration function
 *
 * @b Description @n
 * This function configures the Live Synth Freq monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_LIVE_SYNTH_FREQ_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-322, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_LIVE_SYNTH_FREQ_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monLiveSynthFreqCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_LIVE_SYNTH_FREQ_CFG *p_apiCmdData)
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
        *   - call rfs_cmdLiveSynthFreqCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdLiveSynthFreqCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monLiveSynthFreqCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor Live Rx Saturation configuration function
 *
 * @b Description @n
 * This function configures the Live Rx Saturation monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_LIVE_RX_SATURATION_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-323, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_LIVE_RX_SAT_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monLiveRxSaturationCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_LIVE_RX_SATURATION_CFG *p_apiCmdData)
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
        *   - call rfs_cmdLiveRxSaturCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdLiveRxSaturCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monLiveRxSaturationCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor Live GPADC CTM configuration function
 *
 * @b Description @n
 * This function configures the Live GPADC CTM monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_LIVE_GPADC_CTM_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-324, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_LIVE_GPADC_CTM_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monLiveGpadcCtmCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_LIVE_GPADC_CTM_CFG *p_apiCmdData)
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
        *   - call rfs_cmdLiveGpadcCtmCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdLiveGpadcCtmCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monLiveGpadcCtmCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor PLL control Voltage configuration function
 *
 * @b Description @n
 * This function configures the PLL control Voltage monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_PLL_CTRL_VOLT_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-325, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_PLL_CTRL_VOLT_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monPllCtrlVoltCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_PLL_CTRL_VOLT_CFG *p_apiCmdData)
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
        *   - call rfs_cmdPllCtrlVltMonCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdPllCtrlVltMonCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monPllCtrlVoltCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor TXn RX Loopback configuration function
 *
 * @b Description @n
 * This function configures the Txn RX loopback monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_TXN_RX_LB_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-326, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_TXn_RX_LB_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monTxNRxLbCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_RX_LB_CFG *p_apiCmdData)
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
        *   - call rfs_cmdTxNRxLbMonCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdTxNRxLbMonCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monTxNRxLbCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor TXn Power configuration function
 *
 * @b Description @n
 * This function configures the Txn Power monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_TXN_POWER_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-327, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_TXn_PWR_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monTxNPowerCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_POWER_CFG *p_apiCmdData)
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
        *   - call rfs_cmdTxNPwrMonCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdTxNPwrMonCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monTxNPowerCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor TXn Ball Break configuration function
 *
 * @b Description @n
 * This function configures the TXn Ball Break monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_TXN_BB_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-328, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_TXn_BB_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monTxNBbCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_BB_CFG *p_apiCmdData)
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
        *   - call rfs_cmdTxNBbMonCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdTxNBbMonCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monTxNBbCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor TXn Internal DC Signal configuration function
 *
 * @b Description @n
 * This function configures the TXn Internal DC Signal monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_TXN_DCSIG_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-329, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_TXn_DCSIG_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monTxNDcSigCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_DCSIG_CFG *p_apiCmdData)
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
        *   - call rfs_cmdTxNDcSigMonCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdTxNDcSigMonCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monTxNDcSigCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor RX HPF and DC Signals configuration function
 *
 * @b Description @n
 * This function configures the RX HPF and DC Signals monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_RX_HPF_DCSIG_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-330, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_RX_HPF_DCSIG_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monRxHpfDcSigCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_RX_HPF_DCSIG_CFG *p_apiCmdData)
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
        *   - call rfs_cmdRxHpfDcSigMonCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdRxHpfDcSigMonCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monRxHpfDcSigCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API Monitor PM LO CLK DC Signal configuration function
 *
 * @b Description @n
 * This function configures the PM LO CLK DC Signal monitor settings.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to Command data @ref T_RL_API_MON_PMCLK_DCSIG_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-331, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MON_PMCLK_DCSIG_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_monPmClkDcSigCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_PMCLK_DCSIG_CFG *p_apiCmdData)
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
        *   - call rfs_cmdPmClkMonCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            xw_return = rfs_cmdPmClkMonCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_monPmClkDcSigCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/* End of group */
/*! @} */

/*
 * END OF rl_monitor.c
 */
