/*!*****************************************************************************
 * @file rfs_command.c
 *
 * @brief FECSSLib RF Frontend Scripter (RFS) command API functions.
 *
 * @b Description @n
 * This file provides FECSS library RFS command API function services to FECSSLib and
 * mmWaveLink for IPC with RFS.
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
 * @addtogroup FECSSLIB_RFS_CMD_MODULE RFS Command API functions
 *  @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     15Mar2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <fecsslib/fecsslib.h>
#include <fecsslib/common/fe_registers.h>
#include <fecsslib/rfscripter/include/rfs_driver.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/****************************FECSSLib Static Functions*************************/


/******************************FECSSLib RFS command APIs***********************/
/*!*****************************************************************************
 * @brief FECSS RFS FW version Get API
 *
 * @b Description @n
 * This function API reads RFS FW version data structure from RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[out] p_fwVerData  - pointer to RFS FW version data @ref T_FE_API_RFS_FW_VER_GET_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-349, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdFwVerGet_rom(UINT8 c_devIndex, T_FE_API_RFS_FW_VER_GET_RSP* p_fwVerData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsFwVerPtr = M_FE_RFS_FW_VER_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_FE_API_RFS_FW_VER_GET_RSP)) / (UINT16)4U;

    /*! -# Read RFS FW version from RFS IPC memory */
    M_MEM_READ(c_devIndex, p_fwVerData, p_rfsFwVerPtr, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdFwVerGet API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS CPU status Get API
 *
 * @b Description @n
 * This function API reads RFS CPU status data structure from RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[out] p_cpuStsData - pointer to RFS CPU status data @ref T_RL_API_RFS_FAULT_STS_GET_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-350, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdCpuStsGet_rom(UINT8 c_devIndex, T_RL_API_RFS_FAULT_STS_GET_RSP* p_cpuStsData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsCpuStsPtr = M_FE_RFS_CPU_STS_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_RFS_FAULT_STS_GET_RSP)) / (UINT16)4U;

    /*! -# Read RFS CPU status from RFS IPC memory */
    M_MEM_READ(c_devIndex, p_cpuStsData, p_rfsCpuStsPtr, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdCpuStsGet API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Factory Calibration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Factory Calibration Data structure to RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_calData     - pointer to RFS Cal data @ref T_RL_API_FECSS_FACT_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-462, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdFactCalDataSet_rom(UINT8 c_devIndex, \
    const T_RL_API_FECSS_FACT_CAL_DATA* p_calData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsCalDataPtr = M_FE_RFS_FACT_CAL_DATA_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_FECSS_FACT_CAL_DATA)) / (UINT16)4U;

    /*! -# Write RFS Calibration Data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsCalDataPtr, p_calData, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdFactCalDataSet_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Factory Calibration Data Get API
 *
 * @b Description @n
 * This function API reads RFS Factory Calibration Data structure from RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[out] p_calData    - pointer to RFS Cal data @ref T_RL_API_FECSS_FACT_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-351, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdFactCalDataGet_rom(UINT8 c_devIndex, \
    T_RL_API_FECSS_FACT_CAL_DATA* p_calData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsCalDataPtr = M_FE_RFS_FACT_CAL_DATA_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_FECSS_FACT_CAL_DATA)) / (UINT16)4U;

    /*! -# Read RFS Calibration Data from RFS IPC memory */
    M_MEM_READ(c_devIndex, p_calData, p_rfsCalDataPtr, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdFactCalDataGet_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS RX-TX Calibration Data Set API
 *
 * @b Description @n
 * This function API writes RFS RX-TX Calibration Data structure to RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_calData     - pointer to RFS Cal data @ref T_RL_API_FECSS_RXTX_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-463, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRxTxCalDataSet_rom(UINT8 c_devIndex, \
    const T_RL_API_FECSS_RXTX_CAL_DATA* p_calData)
{
    T_RETURNTYPE xw_return;
    T_RETURNTYPE xw_return1;
    UINT32* p_rfsCalDataPtr = M_FE_RFS_RXTX_CAL_DATA_START_ADDRESS;
    UINT32* p_rfsCalValidityPtr = M_FE_RFS_FACT_CAL_DATA_START_ADDRESS;
    UINT32  w_calValidity;
    UINT32  w_rxTxValidity;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_FECSS_RXTX_CAL_DATA)) / (UINT16)4U;

    /*! -# Write RFS Calibration Data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsCalDataPtr, p_calData, h_numOfWords, xw_return);

    /*! -# Update calibration validity and redundant validity in IPC memory */
    M_MEM32_READ(c_devIndex, p_rfsCalValidityPtr, w_calValidity, xw_return1);
    xw_return += xw_return1;
    w_rxTxValidity = ((UINT32)p_calData->c_CalRxTxResValidity & (UINT32)0x3U) << (UINT32)6U;
    w_calValidity |= w_rxTxValidity | (w_rxTxValidity << (UINT32)16U);
    M_MEM32_WRITE(c_devIndex, p_rfsCalValidityPtr, w_calValidity, xw_return1);
    xw_return += xw_return1;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdRxTxCalDataSet_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS RX-TX Calibration Data Get API
 *
 * @b Description @n
 * This function API reads RFS RX-TX Calibration Data structure from RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[out] p_calData    - pointer to RFS Cal data @ref T_RL_API_FECSS_RXTX_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-352, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRxTxCalDataGet_rom(UINT8 c_devIndex, \
    T_RL_API_FECSS_RXTX_CAL_DATA* p_calData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsCalDataPtr = M_FE_RFS_RXTX_CAL_DATA_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_FECSS_RXTX_CAL_DATA)) / (UINT16)4U;

    /*! -# Read RFS Calibration Data from RFS IPC memory */
    M_MEM_READ(c_devIndex, p_calData, p_rfsCalDataPtr, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdRxTxCalDataGet_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS RF power ON/OFF API
 *
 * @b Description @n
 * This API triggeres the RFS RF block power ON/OFF command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the configuration.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_RF_PWR_CFG_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-353, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_RF_PWR_ONOFF
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRfPwrOnOff_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RF_PWR_CFG_CMD *p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_RF_PWR_ONOFF;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_RF_PWR_CFG_CMD);
    /*! -# Update RFS MB response size - empty */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure, NULL in case there is no data */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS RF Factory Calibration API
 *
 * @b Description @n
 * This API triggers the RFS RF boot calibration command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the calibration.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_RF_FACT_CAL_CMD
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_FECSS_RF_FACT_CAL_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-354, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_FACT_CAL
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRfFactCal_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RF_FACT_CAL_CMD *p_cmdData, \
                            T_RL_API_FECSS_RF_FACT_CAL_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_FACT_CAL;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_RF_FACT_CAL_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_FECSS_RF_FACT_CAL_RSP);
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RF Tx Runtime CLPC Calibration API
 *
 * @b Description @n
 * This API triggeres the RFS TX CLPC calibration command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the calibration.
 *
 * @b Assumptions: @n
 * - The FECSS RF ON is done before issuing this API.
 * - The APLL should be on before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-355, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RUNTIME_TX_CLPC_CAL
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRfTxRuntimeClpcCal_rom(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD *p_cmdData, \
                T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_TX_RUNTIME_CLPC_CAL;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP);
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS APLL control API
 *
 * @b Description @n
 * This API used to control the APLL clock source dynamically based on application low power mode,
 * the command data is sent through MB and response is read once RFS
 * is done with the calibration.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_CLK_CTRL_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-356, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_APLL_CLK_CTRL
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdApllClkCtrl_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_APLL_CLK_CTRL;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_DEV_CLK_CTRL_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS RF Runtime Calibration (calibrate) API
 *
 * @b Description @n
 * This API triggeres the RFS RF runtime calibration command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the calibration.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_RF_RUN_CAL_CMD
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_FECSS_RF_RUN_CAL_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-357, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_RUNTIME_CAL
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRfCalibrate_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RF_RUN_CAL_CMD *p_cmdData, \
                            T_RL_API_FECSS_RF_RUN_CAL_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_RUNTIME_CAL;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_RF_RUN_CAL_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_FECSS_RF_RUN_CAL_RSP);
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS RF clock BW config API
 *
 * @b Description @n
 * This API triggeres the RFS RF clock cfg command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the calibration.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_CLK_BW_CFG_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-358, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_RF_CLOCK_BW
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRfClockBwCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_CLK_BW_CFG_CMD *p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_RF_CLOCK_BW;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_CLK_BW_CFG_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS GPADC configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS GPADC configuration Data structure to RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to RFS GPADC CFG data @ref T_RL_API_FECSS_GPADC_MEAS_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-359, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdGpadcCfg_rom(UINT8 c_devIndex, const T_RL_API_FECSS_GPADC_MEAS_CMD* p_cmdData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsGpadcDataPtr = M_FE_RFS_GPADC_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_FECSS_GPADC_MEAS_CMD)) / (UINT16)4U;

    /*! -# Write RFS GPADC configuration data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsGpadcDataPtr, p_cmdData, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdGpadcCfg API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS GPADC measurement trigger API
 *
 * @b Description @n
 * This API triggeres the RFS GPADC trigger command, the RFS perfroms the measurement
 * based on the configuration data @ref T_RL_API_FECSS_GPADC_MEAS_CMD, the command trigger is
 * sent through MB and response is read once RFS is done with the measurement.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_FECSS_GPADC_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-360, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_GPADC_TRIG
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdGpadcTrig_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_GPADC_MEAS_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_GPADC_TRIG;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)0U;
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_FECSS_GPADC_MEAS_RSP);
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)M_NULL_PTR;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Temperature configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Temp configuration Data structure to RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to RFS GPADC CFG data @ref T_RL_API_FECSS_TEMP_MEAS_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-361, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdTempCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_TEMP_MEAS_CMD *p_cmdData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsTempDataPtr = M_FE_RFS_TEMP_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_FECSS_TEMP_MEAS_CMD)) / (UINT16)4U;

    /*! -# Write RFS GPADC configuration data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsTempDataPtr, p_cmdData, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdTempCfg API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Temperature measurement trigger API
 *
 * @b Description @n
 * This API triggeres the RFS Temp trigger command, the RFS perfroms the measurement
 * based on the configuration data @ref T_RL_API_FECSS_TEMP_MEAS_CMD, the command trigger is
 * sent through MB and response is read once RFS is done with the measurement.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_FECSS_TEMP_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-362, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_TEMP_TRIG
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdTempTrig_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_TEMP_MEAS_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_TEMP_TRIG;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)0U;
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_FECSS_TEMP_MEAS_RSP);
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)M_NULL_PTR;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS RDIF configuration and control API
 *
 * @b Description @n
 * This API triggeres the RFS RDIF control command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the configuration.
 *
 * @b Assumption: The FECSS power up and common Profile cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_RDIF_CTRL_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-363, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_RDIF_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdFecssRdifCtrl_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RDIF_CTRL_CMD *p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_RDIF_CFG;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_RDIF_CTRL_CMD);
    /*! -# Update RFS MB response size - empty */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure, NULL in case there is no data */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}


/*!*****************************************************************************
 * @brief FECSS RFS RF status Get API
 *
 * @b Description @n
 * This API function Get the config of the various RF analog control in RFS.
 *
 * @b Assumption: The FECSS powerup and RF ONOFF API is issued before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData  - pointer to command data @ref T_RL_API_FECSS_RF_STS_GET_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-364, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_RF_STATUS_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRfStatusGet_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_RF_STS_GET_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_RF_STATUS_GET;
    /*! -# Update RFS MB command size - empty */
    z_cmdResExcData.h_CmdSize = (UINT16)0U;
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_FECSS_RF_STS_GET_RSP);
    /*! -# Update pointer to command data structure - NULL since no data */
    z_cmdResExcData.p_CmdDataPtr = M_NULL_PTR;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Debug configuration function
 *
 * @b Description @n
 * This API triggeres the RFS Debug cfg command, the command data is sent through
 * MB.
 *
 * @b Assumption: This has to be first API issued to RFS.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_RFS_DBG_CFG_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-365, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_DBG_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdDebugCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RFS_DBG_CFG_CMD *p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_DBG_CFG;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_FECSS_RFS_DBG_CFG_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)M_NULL;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Chirp Profile configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Chirp Profile configuration Data structure to RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to RFS Chirp Profile CFG data
 *                            @ref T_FE_API_RFS_CHIRP_PROFILE_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-366, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdChirpProfCfg_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_CHIRP_PROFILE_CMD* p_cmdData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsChirpProfDataPtr = M_FE_RFS_PROF_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_FE_API_RFS_CHIRP_PROFILE_CMD)) / (UINT16)4U;

    /*! -# Write RFS Chirp Profile configuration data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsChirpProfDataPtr, p_cmdData, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdChirpProfCfg API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Chirp Profile configuration Data GET API
 *
 * @b Description @n
 * This function API Reads RFS Chirp Profile configuration Data structure to RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[out] p_resData    - pointer to RFS Chirp Profile CFG data
 *                            @ref T_FE_API_RFS_CHIRP_PROFILE_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-367, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdChirpProfCfgGet_rom(UINT8 c_devIndex, \
                            T_FE_API_RFS_CHIRP_PROFILE_CMD* p_resData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsChirpProfDataPtr = M_FE_RFS_PROF_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_FE_API_RFS_CHIRP_PROFILE_CMD)) / (UINT16)4U;

    /*! -# Read RFS Chirp Profile configuration data to RFS IPC memory */
    M_MEM_READ(c_devIndex, p_resData, p_rfsChirpProfDataPtr, h_numOfWords, xw_return);

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdChirpProfCfgGet API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Live Monitor configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Live Monitor configuration Data structure to RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to RFS Live Mon sensor start CFG data
 *                            @ref T_FE_API_RFS_LIVE_MON_SENS_START_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-368, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdSensStartCfg_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_LIVE_MON_SENS_START_CMD* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsSensStartDataPtr = (UINT32*)M_FE_RFS_SENS_START_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_FE_API_RFS_LIVE_MON_SENS_START_CMD)) / (UINT16)4U;

    /*! -# Clear the RFS sensor status Params */
    xw_return = rfs_sensStatusClear(c_devIndex);

    /*! -# Write RFS Sensor Start configuration data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsSensStartDataPtr, p_cmdData, h_numOfWords, xw_return1);
    xw_return += xw_return1;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdChirpProfCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS CW sensor start API
 *
 * @b Description @n
 * This API triggeres the RFS CW sensor start command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the configuration.
 *
 * @b Assumption: The FECSS power up and RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_FE_API_RFS_CW_SENSOR_START_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-369, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_CW_SENS_START
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdCwSensorStart_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_CW_SENSOR_START_CMD* p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_CW_SENS_START;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_FE_API_RFS_CW_SENSOR_START_CMD);
    /*! -# Update RFS MB response size - empty */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure, NULL in case there is no data */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS CW sensor stop API
 *
 * @b Description @n
 * This API stops the RFS CW tone generation and resets the RFS state machine in case of
 * forced frame stop, the command data is sent through MB and response is read once RFS
 * is done with the configuration.
 *
 * @b Assumption: The sensor start is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_FE_API_RFS_CW_SENSOR_STOP_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-370, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_CW_SENS_STOP
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdCwSensorStop_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_CW_SENSOR_STOP_CMD* p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_CW_SENS_STOP;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_FE_API_RFS_CW_SENSOR_STOP_CMD);
    /*! -# Update RFS MB response size - empty */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure, NULL in case there is no data */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS sensor dynamic power save disable API
 *
 * @b Description @n
 * This API function disables the various sensor inter-chirp and inter-burst dynamic
 * power save options in RFS, they are enabled by default.
 *
 * @b Assumption: The FECSS powerup is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENS_DYN_PWR_SAVE_DIS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-371, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_SENS_DYN_PS_DIS
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdSensDynPwrSaveDis_rom(UINT8 c_devIndex, \
                            const T_RL_API_SENS_DYN_PWR_SAVE_DIS* p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_SENS_DYN_PS_DIS;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_SENS_DYN_PWR_SAVE_DIS);
    /*! -# Update RFS MB response size - empty */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure, NULL in case there is no data */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS sensor dynamic power save disable Get API
 *
 * @b Description @n
 * This API function Get the config of the various sensor inter-chirp and inter-burst dynamic
 * power save options in RFS, they are enabled by default.
 *
 * @b Assumption: The FECSS powerup is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData  - pointer to command data @ref T_RL_API_SENS_DYN_PWR_SAVE_DIS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-464, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_SENS_DYN_PS_DIS_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdSensDynPwrSaveStsGet_rom(UINT8 c_devIndex, \
                            T_RL_API_SENS_DYN_PWR_SAVE_DIS* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_SENS_DYN_PS_DIS_GET;
    /*! -# Update RFS MB command size - empty */
    z_cmdResExcData.h_CmdSize = (UINT16)0U;
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_SENS_DYN_PWR_SAVE_DIS);
    /*! -# Update pointer to command data structure - NULL since no data */
    z_cmdResExcData.p_CmdDataPtr = M_NULL_PTR;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Sensor Loopback Configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Sensor Loopback Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to RFS loopback CFG data @ref T_RL_API_SENS_LOOPBACK_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-372, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdSensLbCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_SENS_LOOPBACK_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return;
    UINT32* p_rfsSensLbStructPtr = M_FE_RFS_LB_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_SENS_LOOPBACK_CFG)) / (UINT16)4U;

    if ((p_cmdData->c_LbFreqSel < M_RL_SENS_LB_FREQ_MIN) || \
            (p_cmdData->c_LbFreqSel > M_RL_SENS_LB_FREQ_MAX))
    {
        /*! -# Return error M_DFP_RET_CODE_SENS_INVAL_LB_FREQ if freq is invalid */
        xw_return = M_DFP_RET_CODE_SENS_INVAL_LB_FREQ;
    }
    else
    {
        /*! -# Write RFS Chirp Profile configuration data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsSensLbStructPtr, p_cmdData, h_numOfWords, xw_return);
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdSensLbCfg API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS sensor loopback enable API
 *
 * @b Description @n
 * This API function enables the RFS sensor loopbacks as per the configuration API.
 *
 * @b Assumption: The FECSS powerup is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENS_LOOPBACK_ENA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-373, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_ID_LB_ENA
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdSensLbEna_rom(UINT8 c_devIndex, \
                            const T_RL_API_SENS_LOOPBACK_ENA* p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_ID_LB_ENA;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_SENS_LOOPBACK_ENA);
    /*! -# Update RFS MB response size - empty */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure, NULL in case there is no data */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor enable and trigger API
 *
 * @b Description @n
 * This API function enables the RFS monitor as per the configuration API.
 *
 * @b Assumption: The FECSS powerup is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_MON_ENABLE_TRIG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-374, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_MON_ENABLE_TRIGGER
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdMonEnableTrig_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_ENABLE_TRIG* p_cmdData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_MON_ENABLE_TRIGGER;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_MON_ENABLE_TRIG);
    /*! -# Update RFS MB response size - empty */
    z_cmdResExcData.h_ResSize = (UINT16)0U;
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure, NULL in case there is no data */
    z_cmdResExcData.p_ResDataPtr = M_NULL_PTR;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Debug PD Power Measurement API
 *
 * @b Description @n
 * This API triggeres the RFS Debug PD power meas command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the calibration.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_MON_DBG_PD_MEAS_CMD
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_MON_DBG_PD_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-375, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_MON_DBG_PD_MEAS
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdDbgPdMeas_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_DBG_PD_MEAS_CMD *p_cmdData, \
                            T_RL_API_MON_DBG_PD_MEAS_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_MON_DBG_PD_MEAS;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_MON_DBG_PD_MEAS_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_MON_DBG_PD_MEAS_RSP);
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}


/*!*****************************************************************************
 * @brief FECSS RFS Debug TX Power Measurement API
 *
 * @b Description @n
 * This API triggeres the RFS Debug TX power meas command based on the command
 * configuration, the command data is sent through MB and response is read once RFS
 * is done with the calibration.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_MON_DBG_TXPWR_MEAS_CMD
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_MON_DBG_TXPWR_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-376, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_FE_RFS_CMD_MON_DBG_TXPWR_MEAS
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdDbgTxPwrMeas_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_DBG_TXPWR_MEAS_CMD *p_cmdData, \
                            T_RL_API_MON_DBG_TXPWR_MEAS_RSP* p_resData)
{
    T_RETURNTYPE xw_return;
    T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData;

    /*! -# Update RFS MB command ID */
    z_cmdResExcData.h_CommandId = M_FE_RFS_CMD_MON_DBG_TXPWR_MEAS;
    /*! -# Update RFS MB command size */
    z_cmdResExcData.h_CmdSize = (UINT16)sizeof(T_RL_API_MON_DBG_TXPWR_MEAS_CMD);
    /*! -# Update RFS MB response size */
    z_cmdResExcData.h_ResSize = (UINT16)sizeof(T_RL_API_MON_DBG_TXPWR_MEAS_RSP);
    /*! -# Update pointer to command data structure */
    z_cmdResExcData.p_CmdDataPtr = (const void*)p_cmdData;
    /*! -# Update pointer to response data structure */
    z_cmdResExcData.p_ResDataPtr = (void*)p_resData;

    /*! -# Execute the command and wait for response
        *     - Lock the mutex context for c_devIndex to avoid multiple API calls from
        *       different thread
        *     - Use Semaphore signal to wait for response from c_devIndex
        */
    xw_return = rfs_cmdRespExecute(c_devIndex, z_cmdResExcData);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor Live Synth Freq Configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Monitor Live Synth freq Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to to Command data @ref T_RL_API_MON_LIVE_SYNTH_FREQ_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-377, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdLiveSynthFreqCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_LIVE_SYNTH_FREQ_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsSynthMonStructPtr = M_FE_RFS_SYNTH_MON_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_LIVE_SYNTH_FREQ_CFG)) / (UINT16)4U;
    UINT8  c_monCfgIpcStatus;
    UINT8  c_monCfgStatus;
    T_FE_API_RFS_LIVE_MON_SENS_START_CMD *p_sensLiveCfg = M_FE_RFS_SENS_START_CFG_START_ADDRESS;

    /*! -# Perfrom error check before writing to IPC */
    if ((p_cmdData->xc_ChirpMonStartTime < M_RL_MON_LIVE_SYNTH_FREQ_START_TIME_MIN) ||
        (p_cmdData->xc_ChirpMonStartTime > M_RL_MON_LIVE_SYNTH_FREQ_START_TIME_MAX))
    {
        /*! -# Return error M_DFP_RET_CODE_MON_INVAL_SYNTH_START if start time is invalid */
        xw_return = M_DFP_RET_CODE_MON_INVAL_SYNTH_START;
    }
    else if ((p_cmdData->h_FreqErrThreshold < M_RL_MON_LIVE_SYNTH_FREQ_ERR_THRESH_MIN) || \
        (p_cmdData->h_FreqErrThreshold > M_RL_MON_LIVE_SYNTH_FREQ_ERR_THRESH_MAX))
    {
        /*! -# Return error M_DFP_RET_CODE_MON_INVAL_SYNTH_THRSH if start time is invalid */
        xw_return = M_DFP_RET_CODE_MON_INVAL_SYNTH_THRSH;
    }
    else
    {
        /*! -# Write RFS command data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsSynthMonStructPtr, p_cmdData, h_numOfWords, xw_return);

        /*!
        * -# Read p_sensLiveCfg->c_RfsLiveMonCfgStatus value from IPC to update the mon cfg status
        *    - RFS uses this status before performing the monitor.
        * -# Write the update status to p_sensLiveCfg->c_RfsLiveMonCfgStatus
        */
        M_MEM8_READ(c_devIndex, &p_sensLiveCfg->c_RfsLiveMonCfgStatus, \
            c_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
        c_monCfgStatus = (UINT8)1U << M_RL_SENS_LIVE_MON_SYNTH_FREQ;
        c_monCfgIpcStatus |= c_monCfgStatus;
        M_MEM8_WRITE(c_devIndex, &p_sensLiveCfg->c_RfsLiveMonCfgStatus, \
                                    c_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdLiveSynthFreqCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor Live RX Saturation Configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Monitor Live RX Saturation Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to to Command data @ref T_RL_API_MON_LIVE_RX_SATURATION_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-378, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdLiveRxSaturCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_LIVE_RX_SATURATION_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsRxSatMonStructPtr = M_FE_RFS_RX_SAT_MON_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_LIVE_RX_SATURATION_CFG)) / (UINT16)4U;
    UINT8  c_monCfgIpcStatus,
           c_monCfgStatus;
    T_FE_API_RFS_LIVE_MON_SENS_START_CMD *p_sensLiveCfg = M_FE_RFS_SENS_START_CFG_START_ADDRESS;

    /*! -# Perfrom error check before writing to IPC */
    if (((UINT16)0U != (p_cmdData->h_PerChirpRamStartAdd & \
        M_RL_SENS_PER_CHIRP_LUT_4BYTE_ADD_MASK)) || \
        (p_cmdData->h_PerChirpRamStartAdd > M_RL_SENS_PER_CHIRP_LUT_STRT_ADDR_MAX))
    {
        /*! -# Return M_DFP_RET_CODE_MON_INVAL_PERCHRP_ADDR error code */
        xw_return = M_DFP_RET_CODE_MON_INVAL_PERCHRP_ADDR;
    }
    else
    {
        /*! -# Write command data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsRxSatMonStructPtr, p_cmdData, h_numOfWords, xw_return);

        /*!
        * -# Read p_sensLiveCfg->c_RfsLiveMonCfgStatus value from IPC to update the mon cfg status
        *    - RFS uses this status before performing the monitor.
        * -# Write the update status to p_sensLiveCfg->c_RfsLiveMonCfgStatus
        */
        M_MEM8_READ(c_devIndex, &p_sensLiveCfg->c_RfsLiveMonCfgStatus,
            c_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
        c_monCfgStatus = (UINT32)1U << M_RL_SENS_LIVE_MON_RX_SATURATION;
        c_monCfgIpcStatus |= c_monCfgStatus;
        M_MEM8_WRITE(c_devIndex, &p_sensLiveCfg->c_RfsLiveMonCfgStatus,
                                    c_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdLiveRxSaturCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor Live GPADC CTM Configuration Data Set API
 *
 * @b Description @n
 * This function API writes RFS Monitor Live GPADC CTM Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to to Command data @ref T_RL_API_MON_LIVE_GPADC_CTM_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-379, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdLiveGpadcCtmCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_LIVE_GPADC_CTM_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsGpadcCtmMonStructPtr = M_FE_RFS_GPADC_CTM_MON_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_LIVE_GPADC_CTM_CFG)) / (UINT16)4U;
    UINT8  c_monCfgIpcStatus,
           c_monCfgStatus;
    T_FE_API_RFS_LIVE_MON_SENS_START_CMD *p_sensLiveCfg = M_FE_RFS_SENS_START_CFG_START_ADDRESS;


    /*! -# Perfrom error check before writing to IPC */
    if ((p_cmdData->c_NumOfGpadcInstr < M_RL_MON_LIVE_GPADC_CTM_NUM_INSTR_MIN) || \
        (p_cmdData->c_NumOfGpadcInstr > M_RL_MON_LIVE_GPADC_CTM_NUM_INSTR_MAX))
    {
        /*! -# Return error M_DFP_RET_CODE_MON_INVAL_GPADC_INST if start time is invalid */
        xw_return = M_DFP_RET_CODE_MON_INVAL_GPADC_INST;
    }
    else
    {
        /*! -# Write command data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsGpadcCtmMonStructPtr, p_cmdData, h_numOfWords, xw_return);

        /*!
        * -# Read p_sensLiveCfg->c_RfsLiveMonCfgStatus value from IPC to update the mon cfg status
        *    - RFS uses this status before performing the monitor.
        * -# Write the update status to p_sensLiveCfg->c_RfsLiveMonCfgStatus
        */
        M_MEM8_READ(c_devIndex, &p_sensLiveCfg->c_RfsLiveMonCfgStatus, \
            c_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
        c_monCfgStatus = (UINT8)1U << M_RL_SENS_LIVE_MON_GPADC_CTM;
        c_monCfgIpcStatus |= c_monCfgStatus;
        M_MEM8_WRITE(c_devIndex, &p_sensLiveCfg->c_RfsLiveMonCfgStatus, \
                                    c_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdLiveGpadcCtmCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor PLL control Voltage configuration function
 *
 * @b Description @n
 * This function API writes RFS Monitor PLL control Voltage Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to Command data @ref T_RL_API_MON_PLL_CTRL_VOLT_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-380, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdPllCtrlVltMonCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_PLL_CTRL_VOLT_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsMonStructPtr = M_FE_RFS_PLL_CTRLV_MON_CFG_START_ADDRESS;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_PLL_CTRL_VOLT_CFG)) / (UINT16)4U;
    UINT32 w_monCfgIpcStatus;
    UINT32 w_monCfgStatus;
    UINT32 w_monIndex;
    UINT32* p_monStatusAddress;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    /*! -# Write Command data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsMonStructPtr, p_cmdData, h_numOfWords, xw_return);

    /*!
     * -# Read p_sensStatus->w_RfsMonCfgStatus value from IPC to update the mon cfg status
     *    - RFS uses this status before performing the monitor.
     * -# Write the update status to p_sensStatus->w_RfsMonCfgStatus
     */
    w_monIndex = M_RL_MON_PLL_CTRL_VOLT;
    p_monStatusAddress = (UINT32*)&p_sensStatus->w_RfsMonCfgStatus[0U];
    if (w_monIndex > (UINT32)32U)
    {
        w_monIndex -= (UINT32)32U;
        p_monStatusAddress++;
    }

    M_MEM32_READ(c_devIndex, p_monStatusAddress, w_monCfgIpcStatus, xw_return1);
    xw_return += xw_return1;
    w_monCfgStatus = (UINT32)1U << w_monIndex;
    w_monCfgIpcStatus |= w_monCfgStatus;
    M_MEM32_WRITE(c_devIndex, p_monStatusAddress, \
                                (UINT32)w_monCfgIpcStatus, xw_return1);
    xw_return += xw_return1;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdPllCtrlVltMonCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor TXn RX LB configuration function
 *
 * @b Description @n
 * This function API writes RFS Monitor TXn RX LB Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to Command data @ref T_RL_API_MON_TXN_RX_LB_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-381, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdTxNRxLbMonCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_TXN_RX_LB_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsMonStructPtr = M_NULL_PTR;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_TXN_RX_LB_CFG)) / (UINT16)4U;
    UINT32 w_monCfgIpcStatus,
           w_monCfgStatus;
    UINT32 w_monIndex;
    UINT32* p_monStatusAddress;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    if (p_cmdData->c_TxIndSel >= M_DFP_MAX_TX_CHANNELS)
    {
        /*! -# Return error M_DFP_RET_CODE_MON_INVAL_TX_IND if Tx index is invalid */
        xw_return = M_DFP_RET_CODE_MON_INVAL_TX_IND;
    } //TBD add more error checks
    else
    {
        p_rfsMonStructPtr = \
            M_FE_RFS_TXn_RX_LB_MON_CFG_START_ADDRESS(p_cmdData->c_TxIndSel);
        /*! -# Write Command data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsMonStructPtr, p_cmdData, h_numOfWords, xw_return);

        /*!
        * -# Read p_sensStatus->w_RfsMonCfgStatus value from IPC to update the mon cfg status
        *    - RFS uses this status before performing the monitor.
        *  -# Write updated status to p_sensStatus->w_RfsMonCfgStatus
        */
        w_monIndex = (M_RL_MON_TX0_RX_LB + (UINT32)p_cmdData->c_TxIndSel);
        p_monStatusAddress = (UINT32*)&p_sensStatus->w_RfsMonCfgStatus[0U];
        if (w_monIndex > (UINT32)32U)
        {
            w_monIndex -= (UINT32)32U;
            p_monStatusAddress++;
        }

        M_MEM32_READ(c_devIndex, p_monStatusAddress, w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
        w_monCfgStatus = (UINT32)1U << w_monIndex;
        w_monCfgIpcStatus |= w_monCfgStatus;
        M_MEM32_WRITE(c_devIndex, p_monStatusAddress, \
                                    (UINT32)w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdTxNRxLbMonCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor TXn Power configuration function
 *
 * @b Description @n
 * This function API writes RFS Monitor TXn Power Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to Command data @ref T_RL_API_MON_TXN_POWER_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-382, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdTxNPwrMonCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_TXN_POWER_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsMonStructPtr = M_NULL_PTR;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_TXN_POWER_CFG)) / (UINT16)4U;
    UINT32 w_monCfgIpcStatus,
           w_monCfgStatus;
    UINT32 w_monIndex;
    UINT32* p_monStatusAddress;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    if (p_cmdData->c_TxIndSel >= M_DFP_MAX_TX_CHANNELS)
    {
        /*! -# Return error M_DFP_RET_CODE_MON_INVAL_TX_IND if Tx index is invalid */
        xw_return = M_DFP_RET_CODE_MON_INVAL_TX_IND;
    } //TBD add more error checks
    else
    {
        p_rfsMonStructPtr = \
            M_FE_RFS_TXn_PWR_MON_CFG_START_ADDRESS(p_cmdData->c_TxIndSel);
        /*! -# Write Command data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsMonStructPtr, p_cmdData, h_numOfWords, xw_return);

        /*!
        * -# Read p_sensStatus->w_RfsMonCfgStatus value from IPC to update the mon cfg status
        *    - RFS uses this status before performing the monitor.
        *  -# Write updated status to p_sensStatus->w_RfsMonCfgStatus
        */
        w_monIndex = (M_RL_MON_TX0_POWER + (UINT32)p_cmdData->c_TxIndSel);
        p_monStatusAddress = (UINT32*)&p_sensStatus->w_RfsMonCfgStatus[0U];
        if (w_monIndex > (UINT32)32U)
        {
            w_monIndex -= (UINT32)32U;
            p_monStatusAddress++;
        }

        M_MEM32_READ(c_devIndex, p_monStatusAddress, w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
        w_monCfgStatus = (UINT32)1U << w_monIndex;
        w_monCfgIpcStatus |= w_monCfgStatus;
        M_MEM32_WRITE(c_devIndex, p_monStatusAddress, \
                                    (UINT32)w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdTxNPwrMonCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor TXn Ballbreak configuration function
 *
 * @b Description @n
 * This function API writes RFS Monitor TXn Ballbreak Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to Command data @ref T_RL_API_MON_TXN_BB_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-383, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdTxNBbMonCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_TXN_BB_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsMonStructPtr = M_NULL_PTR;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_TXN_BB_CFG)) / (UINT16)4U;
    UINT32 w_monCfgIpcStatus,
           w_monCfgStatus;
    UINT32 w_monIndex;
    UINT32* p_monStatusAddress;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    if (p_cmdData->c_TxIndSel >= M_DFP_MAX_TX_CHANNELS)
    {
        /*! -# Return error M_DFP_RET_CODE_MON_INVAL_TX_IND if Tx index is invalid */
        xw_return = M_DFP_RET_CODE_MON_INVAL_TX_IND;
    } //TBD add more error checks
    else
    {
        p_rfsMonStructPtr = \
            M_FE_RFS_TXn_BB_MON_CFG_START_ADDRESS(p_cmdData->c_TxIndSel);
        /*! -# Write Command data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsMonStructPtr, p_cmdData, h_numOfWords, xw_return);

        /*!
        * -# Read p_sensStatus->w_RfsMonCfgStatus value from IPC to update the mon cfg status
        *    - RFS uses this status before performing the monitor.
        *  -# Write updated status to p_sensStatus->w_RfsMonCfgStatus
        */
        w_monIndex = (M_RL_MON_TX0_BB + (UINT32)p_cmdData->c_TxIndSel);
        p_monStatusAddress = (UINT32*)&p_sensStatus->w_RfsMonCfgStatus[0U];
        if (w_monIndex > (UINT32)32U)
        {
            w_monIndex -= (UINT32)32U;
            p_monStatusAddress++;
        }

        M_MEM32_READ(c_devIndex, p_monStatusAddress, w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
        w_monCfgStatus = (UINT32)1U << w_monIndex;
        w_monCfgIpcStatus |= w_monCfgStatus;
        M_MEM32_WRITE(c_devIndex, p_monStatusAddress, \
                                    (UINT32)w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdTxNBbMonCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor TXn DC Signal configuration function
 *
 * @b Description @n
 * This function API writes RFS Monitor TXn DC Signal Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to Command data @ref T_RL_API_MON_TXN_DCSIG_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-384, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdTxNDcSigMonCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_TXN_DCSIG_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsMonStructPtr = M_NULL_PTR;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_TXN_DCSIG_CFG)) / (UINT16)4U;
    UINT32 w_monCfgIpcStatus,
           w_monCfgStatus;
    UINT32 w_monIndex;
    UINT32* p_monStatusAddress;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    if (p_cmdData->c_TxIndSel >= M_DFP_MAX_TX_CHANNELS)
    {
        /*! -# Return error M_DFP_RET_CODE_MON_INVAL_TX_IND if Tx index is invalid */
        xw_return = M_DFP_RET_CODE_MON_INVAL_TX_IND;
    } //TBD add more error checks
    else
    {
        p_rfsMonStructPtr = \
            M_FE_RFS_TXn_DCSIG_MON_CFG_START_ADDRESS(p_cmdData->c_TxIndSel);
        /*! -# Write Command data to RFS IPC memory */
        M_MEM_WRITE(c_devIndex, p_rfsMonStructPtr, p_cmdData, h_numOfWords, xw_return);

        /*!
        * -# Read p_sensStatus->w_RfsMonCfgStatus value from IPC to update the mon cfg status
        *    - RFS uses this status before performing the monitor.
        *  -# Write updated status to p_sensStatus->w_RfsMonCfgStatus
        */
        w_monIndex = (M_RL_MON_TX0_INTRNAL_DC_SIG + (UINT32)p_cmdData->c_TxIndSel);
        p_monStatusAddress = (UINT32*)&p_sensStatus->w_RfsMonCfgStatus[0U];
        if (w_monIndex > (UINT32)32U)
        {
            w_monIndex -= (UINT32)32U;
            p_monStatusAddress++;
        }

        M_MEM32_READ(c_devIndex, p_monStatusAddress, w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
        w_monCfgStatus = (UINT32)1U << w_monIndex;
        w_monCfgIpcStatus |= w_monCfgStatus;
        M_MEM32_WRITE(c_devIndex, p_monStatusAddress, \
                                    (UINT32)w_monCfgIpcStatus, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdTxNDcSigMonCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor RX HPF and DC Signal configuration function
 *
 * @b Description @n
 * This function API writes RFS Monitor RX HPF and DC Signal Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to Command data @ref T_RL_API_MON_RX_HPF_DCSIG_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-385, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRxHpfDcSigMonCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_RX_HPF_DCSIG_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsMonStructPtr = M_NULL_PTR;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_RX_HPF_DCSIG_CFG)) / (UINT16)4U;
    UINT32 w_monCfgIpcStatus,
           w_monCfgStatus;
    UINT32 w_monIndex;
    UINT32* p_monStatusAddress;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    p_rfsMonStructPtr = M_FE_RFS_RX_HPF_DCSIG_MON_CFG_START_ADDRESS;
    /*! -# Write Command data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsMonStructPtr, p_cmdData, h_numOfWords, xw_return);

    /*!
    * -# Read p_sensStatus->w_RfsMonCfgStatus value from IPC to update the mon cfg status
    *    - RFS uses this status before performing the monitor.
    *  -# Write updated status to p_sensStatus->w_RfsMonCfgStatus
    */
    w_monIndex = M_RL_MON_RX_HPF_INTRNAL_DC_SIG;
    p_monStatusAddress = (UINT32*)&p_sensStatus->w_RfsMonCfgStatus[0U];
    if (w_monIndex > (UINT32)32U)
    {
        w_monIndex -= (UINT32)32U;
        p_monStatusAddress++;
    }

    M_MEM32_READ(c_devIndex, p_monStatusAddress, w_monCfgIpcStatus, xw_return1);
    xw_return += xw_return1;
    w_monCfgStatus = (UINT32)1U << w_monIndex;
    w_monCfgIpcStatus |= w_monCfgStatus;
    M_MEM32_WRITE(c_devIndex, p_monStatusAddress, \
                                (UINT32)w_monCfgIpcStatus, xw_return1);
    xw_return += xw_return1;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdRxHpfDcSigMonCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Monitor PM LO CLK DC Signal configuration function
 *
 * @b Description @n
 * This function API writes RFS Monitor PM LO CLK DC Signal Configuration Data structure to
 * RFS-FECSSLib IPC RAM.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_cmdData     - pointer to Command data @ref T_RL_API_MON_PMCLK_DCSIG_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-386, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdPmClkMonCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_PMCLK_DCSIG_CFG* p_cmdData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_rfsMonStructPtr = M_NULL_PTR;
    UINT16 h_numOfWords = (UINT16)(sizeof(T_RL_API_MON_PMCLK_DCSIG_CFG)) / (UINT16)4U;
    UINT32 w_monCfgIpcStatus,
           w_monCfgStatus;
    UINT32 w_monIndex;
    UINT32* p_monStatusAddress;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    p_rfsMonStructPtr = M_FE_RFS_PM_CLK_DCSIG_MON_CFG_START_ADDRESS;
    /*! -# Write Command data to RFS IPC memory */
    M_MEM_WRITE(c_devIndex, p_rfsMonStructPtr, p_cmdData, h_numOfWords, xw_return);

    /*!
    * -# Read p_sensStatus->w_RfsMonCfgStatus value from IPC to update the mon cfg status
    *    - RFS uses this status before performing the monitor.
    *  -# Write updated status to p_sensStatus->w_RfsMonCfgStatus
    */
    w_monIndex = M_RL_MON_PM_CLK_INTRNAL_DC_SIG;
    p_monStatusAddress = (UINT32*)&p_sensStatus->w_RfsMonCfgStatus[0U];

    if (w_monIndex > (UINT32)32U)
    {
        w_monIndex -= (UINT32)32U;
        p_monStatusAddress++;
    }

    M_MEM32_READ(c_devIndex, p_monStatusAddress, w_monCfgIpcStatus, xw_return1);
    xw_return += xw_return1;
    w_monCfgStatus = (UINT32)1U << w_monIndex;
    w_monCfgIpcStatus |= w_monCfgStatus;
    M_MEM32_WRITE(c_devIndex, p_monStatusAddress, \
                                (UINT32)w_monCfgIpcStatus, xw_return1);
    xw_return += xw_return1;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdPmClkMonCfg_rom API:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF fe_driver.c
 */
