/*!*****************************************************************************
 * @file rl_device.c
 *
 * @brief mmWaveLink device module API configuration and interface functions
 *
 * @b Description @n
 * This file provides mmWaveLink device module API function calls, application
 * shall call these APIs to initialize the interface call backs, powerup FECSS,
 * powerup RF front end etc. Refer API documentation for more details.
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
 * @addtogroup MMWL_DEVICE_API mmWaveLink Device Control API functions
 * @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     18Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <mmwavelink/mmwavelink.h>
#include <mmwavelink/include/rl_device.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Device Driver static data structure
 * @{
 */
/*!
 * @brief  FECSSLib Driver Data, holds all interface and callback info locally.
 * FECSSLib drivers can be ROMed in TI device, in that case static library in ROM can not
 * hold any data symbols, the address for data symbols are allocated during linking of
 * application with FECSSLib and mmwavelink libraries. The z_FeDriverData is delcared in
 * mmWaveLink Library which is always executes on RAM and memory allocation can be done
 * during linking with application, the address of z_FeDriverData is stored in AON8 register
 * during rl_mmWaveLinkInit.  The FECSSLib ROM always gets the z_FeDriverData hander address
 * from fixed AON8 register.
 */
static T_FE_DRIVER_DATA z_FeDriverData;

#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_FE)
/*!
 * @brief  This Global Variable holds the address of z_FeDriverData in case of
 * front end device
 */
REG32 w_FeDrvDataAdd;
#endif

/*! @} */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */
/****************************mmWaveLink Static Functions*************************/
/*!
 * @name mmWaveLink Library Device module static function calls
 * @{
 */
static T_RETURNTYPE rl_initErrorCheck(T_DFP_CLIENT_CB_DATA z_clientCbData);
/*! @} */

/*!*****************************************************************************
 * @brief mmWaveLink init function error check
 *
 * @b Description @n
 * This is a static function performs the error check for mmWaveLink init frame
 * work input parameters.
 *
 * @b Assumption:
 *
 * @param[in] z_clientCbData - Client context data @ref T_DFP_CLIENT_CB_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-295, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
static T_RETURNTYPE rl_initErrorCheck(T_DFP_CLIENT_CB_DATA z_clientCbData)
{
    /*! -# T_RETURNTYPE type is SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# If z_clientCbData platform, device and api error chk status are wrong then
     *     - return M_DFP_RET_CODE_INVALID_INPUT
     */
    if ((z_clientCbData.c_PlatformType > (UINT8)M_DFP_DEVICE_PLATFORM_FE) || \
        (z_clientCbData.c_DeviceType > M_DFP_DEVICETYPE_6131) || \
        (z_clientCbData.c_ApiErrorCheckDis > M_TRUE))
    {
        xw_return = M_DFP_RET_CODE_INVALID_INPUT;
    }
    /*! -# else If OSI Mutex callback functions are NULL then
     *    - return M_DFP_RET_CODE_INTERFACE_CB_NULL
     */
    else if ((z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexCreate == M_NULL_PTR) || \
        (z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexLock == M_NULL_PTR) || \
        (z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexUnLock == M_NULL_PTR) || \
        (z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexDelete == M_NULL_PTR))
    {
        xw_return = M_DFP_RET_CODE_INTERFACE_CB_NULL;
    }
    /*! -# else If OSI Semaphore callback functions are NULL then
     *    - return M_DFP_RET_CODE_INTERFACE_CB_NULL
     */
    else if ((z_clientCbData.z_OsiCb.z_Semp.p_OsiSemCreate == M_NULL_PTR) || \
        (z_clientCbData.z_OsiCb.z_Semp.p_OsiSemWait == M_NULL_PTR) || \
        (z_clientCbData.z_OsiCb.z_Semp.p_OsiSemSignal == M_NULL_PTR) || \
        (z_clientCbData.z_OsiCb.z_Semp.p_OsiSemDelete == M_NULL_PTR))
    {
        xw_return = M_DFP_RET_CODE_INTERFACE_CB_NULL;
    }
    /*! -# else If Platform callback functions are NULL then
     *    - return M_DFP_RET_CODE_INTERFACE_CB_NULL
     */
    else if ((z_clientCbData.z_PltfCb.p_Delay == M_NULL_PTR) || \
        (z_clientCbData.z_PltfCb.p_TimeStamp == M_NULL_PTR) || \
        (z_clientCbData.z_PltfCb.p_RegisterIsr == M_NULL_PTR) || \
        (z_clientCbData.z_PltfCb.p_DeRegisterIsr == M_NULL_PTR))
    {
        xw_return = M_DFP_RET_CODE_INTERFACE_CB_NULL;
    }
    /*! -# else If Platform callback functions are NULL then
     *    - return M_DFP_RET_CODE_INTERFACE_CB_NULL
     */
    else if ((z_clientCbData.z_PltfCb.p_MaskIsr == M_NULL_PTR) || \
        (z_clientCbData.z_PltfCb.p_UnMaskIsr == M_NULL_PTR) || \
        (z_clientCbData.z_PltfCb.p_EnterCriticalRegion == M_NULL_PTR) || \
        (z_clientCbData.z_PltfCb.p_ExitCriticalRegion == M_NULL_PTR))
    {
        xw_return = M_DFP_RET_CODE_INTERFACE_CB_NULL;
    }
    /*! -# If Device is M_DFP_DEVICE_PLATFORM_FE and Comm IF callback functions are NULL then
     *    - return M_DFP_RET_CODE_INTERFACE_CB_NULL
     */
    else if (((UINT8)M_DFP_DEVICE_PLATFORM_FE == z_clientCbData.c_PlatformType) && \
        ((z_clientCbData.z_ComIfCb.p_ComIfOpen == M_NULL_PTR) || \
        (z_clientCbData.z_ComIfCb.p_ComIfRead == M_NULL_PTR) || \
        (z_clientCbData.z_ComIfCb.p_ComIfWrite == M_NULL_PTR) || \
        (z_clientCbData.z_ComIfCb.p_ComIfClose == M_NULL_PTR)))
    {
        xw_return = M_DFP_RET_CODE_INTERFACE_CB_NULL;
    }
    /*! -# If RFS debug data buffer is NULL then
     *    - return M_DFP_RET_CODE_RFS_DBG_BUF_CB_NULL
     */
    else if (z_clientCbData.z_DbgCb.p_RfsdbgData == M_NULL_PTR)
    {
        xw_return = M_DFP_RET_CODE_RFS_DBG_BUF_CB_NULL;
    }
    /*! -# else return M_DFP_RET_CODE_OK */
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# Return the status xw_return */
    return xw_return;
}

/******************************mmWaveLink APIs*********************************/

/*!*****************************************************************************
 * @brief mmWaveLink interface init API function
 *
 * @b Description @n
 * This function initializes all the interface call back functions and
 * client context data structures for mmWaveLink and FECSS Library.
 *
 * @b Assumption: In case of multiple cascade front end (FE) devices, this function shall be
 * called once for all devices to initialize the mmWaveLink SW framework, or in case of multiple
 * calls the call back client context data shall be identical across device threads.
 *
 * @param[in] w_deviceMap    - Device Map
 * @param[in] z_clientCbData - API input data structure, mmWaveLink client call back
 *                             @ref T_DFP_CLIENT_CB_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-293, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MMWL_INIT
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_mmWaveLinkInit(UINT32 w_deviceMap, T_DFP_CLIENT_CB_DATA z_clientCbData)
{
    /*! -# T_RETURNTYPE type is SINT32 value */
    T_RETURNTYPE xw_return;
    /*! -# Get ptr address holds the z_FeDriverData address */
    REG32* p_regPtr = M_FE_DRV_DATA_ADDRESS_PTR;

    /*!
     *  -# If w_deviceMap = 0 or greater than M_DFP_MAX_DEV_MAP_MASK then
     *     - return M_DFP_RET_CODE_INVALID_DEVICE
     *  -# else
     *     - Perform Error checks for client call back data in rl_initErrorCheck()
     */
    if (((UINT32)0U == w_deviceMap) || (w_deviceMap > M_DFP_MAX_DEV_MAP_MASK))
    {
        xw_return = M_DFP_RET_CODE_INVALID_DEVICE;
    }
    else
    {
       xw_return = rl_initErrorCheck(z_clientCbData);
    }

    /*!
     * -# If there is no error then call fe_fecssLibInit() to initialize FECSSLib interface
     * call back functions.
     */
    if (xw_return == M_DFP_RET_CODE_OK)
    {
        /*! -# Store z_FeDriverData address in ptr register */
        *p_regPtr = (REG32)(&z_FeDriverData);

        /*! -# call fe_fecssLibInit() */
        xw_return = fe_fecssLibInit(w_deviceMap, z_clientCbData);
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_mmWaveLinkInit API:%x:%d\n", w_deviceMap, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink interface Deinit API function
 *
 * @b Description @n
 * This function un initializes all the interface call back functions and
 * client context data structure for mmWaveLink and FECSS Library.
 *
 * @b Assumption: This function shall be called once for a device after calling
 * rl_fecssDevPwrOff API.
 *
 * @param[in] w_deviceMap    - Device Map
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-294, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_MMWL_DEINIT
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_mmWaveLinkDeInit(UINT32 w_deviceMap)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*!
     * -# If w_deviceMap = 0 or greater than M_DFP_MAX_DEV_MAP_MASK then
     *   - return M_DFP_RET_CODE_INVALID_DEVICE
     * -# else
     *   - Un initialize FECSS Interface callback data structure in fe_fecssLibDeInit()
     * */
    if (((UINT32)0U == w_deviceMap) || (w_deviceMap > M_DFP_MAX_DEV_MAP_MASK))
    {
        xw_return = M_DFP_RET_CODE_INVALID_DEVICE;
    }
    else
    {
        xw_return = fe_fecssLibDeInit(w_deviceMap);
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_mmWaveLinkDeInit API:%x:%d\n", w_deviceMap, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS device power ON API function
 *
 * @b Description @n
 * This API function configures the FECSS device power ON and clock settings.
 * The RFS M3 core is powered up part of this API.
 *
 * @b Assumption: The mmWavelink Init is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_PWR_ON_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-296, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_DEV_PWR_ON
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssDevPwrOn(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_PWR_ON_CMD *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    UINT32 w_devMap;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();

    /*! -# Validate device mmwaveLink status  */
    w_devMap = ((UINT32)1U << c_devIndex);

    /*!
    * -# If c_devIndex is greater than or equal to M_DFP_MAX_DEVICES then
    *   - return M_DFP_RET_CODE_INVALID_DEVICE
    * */
    if (c_devIndex >= M_DFP_MAX_DEVICES)
    {
        xw_return = M_DFP_RET_CODE_INVALID_DEVICE;
    }
    /*!
    * -# The FECSS mmwavelink should be in init state now for selected device
    *    - Return M_DFP_RET_CODE_MMWL_INIT_NOT_DONE if not initialized
    *    - Return M_DFP_RET_CODE_OK if device is powered up
    */
    else if ((w_devMap & p_feDrvData->w_ActiveDevInit) != w_devMap)
    {
        xw_return = M_DFP_RET_CODE_MMWL_INIT_NOT_DONE;
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call fe_fecssDevPwrOn command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS device power ON API */
            xw_return = fe_fecssDevPwrOn(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssDevPwrOn API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS device power OFF API function
 *
 * @b Description @n
 * This API function configures the FECSS device in power OFF state.
 * The RFS M3 core is powered down part of this API.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_PWR_OFF_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-297, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_DEV_PWR_OFF
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssDevPwrOff(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_PWR_OFF_CMD *p_apiCmdData)
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
        *   - call fe_fecssDevPwrOn command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS device power OFF API */
            xw_return = fe_fecssDevPwrOff(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssDevPwrOff API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS RF power ON/OFF API function
 *
 * @b Description @n
 * This API function configures the FECSS RF block power ON/OFF. The application
 * shall use this API to power up/down the RF blocks before and after the usage.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_RF_PWR_CFG_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-299, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RF_PWR_ONOFF
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfPwrOnOff(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RF_PWR_CFG_CMD *p_apiCmdData)
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
        *   - call rfs_rfsRfPwrOnOff command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RF power ON/OFF API  */
            xw_return = rfs_cmdRfPwrOnOff(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfPwrOnOff API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API DFP version get API function
 *
 * @b Description @n
 * This API function gets the mmWave DFP mmWaveLink, FECSSLib and RFS component version
 * number.
 *
 * @b Assumption: The FECSS is poweredup while issuing this API
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData   - pointer to Response data @ref T_RL_API_DFP_FW_VER_GET_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-298, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_DFP_VER_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_mmWaveDfpVerGet(UINT8 c_devIndex, \
                T_RL_API_DFP_FW_VER_GET_RSP *p_apiResData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    T_FE_API_RFS_FW_VER_GET_RSP z_rfsFwVerData = {0};

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command and response pointers are NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdFwVerGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Get RFS ROM and Patch version */
            xw_return = rfs_cmdFwVerGet(c_devIndex, &z_rfsFwVerData);
        }
    }

    /*!
     * -# If RFS version get is successful then update the response data structure
     */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Copy RFS ROM version */
        p_apiResData->z_RfsRomVersion   = z_rfsFwVerData.z_RomVersion;
        /*! -# Copy RFS Patch version */
        p_apiResData->z_RfsPatchVersion = z_rfsFwVerData.z_PatchVersion;
        /*! -# Copy mmWaveLink version */
        p_apiResData->z_MmwlLibVersion.c_GenVerNum = M_RL_MMWAVELINK_VERSION_GEN;
        p_apiResData->z_MmwlLibVersion.c_MajorVerNum = M_RL_MMWAVELINK_VERSION_MAJOR;
        p_apiResData->z_MmwlLibVersion.c_MinorVerNum = M_RL_MMWAVELINK_VERSION_MINOR;
        p_apiResData->z_MmwlLibVersion.c_BuildVerNum = M_RL_MMWAVELINK_VERSION_BUILD;
        p_apiResData->z_MmwlLibVersion.c_Date = M_RL_MMWAVELINK_VERSION_DAY;
        p_apiResData->z_MmwlLibVersion.c_Month = M_RL_MMWAVELINK_VERSION_MONTH;
        p_apiResData->z_MmwlLibVersion.c_Year = M_RL_MMWAVELINK_VERSION_YEAR;
        /*! -# Copy FecssLib version */
        p_apiResData->z_FecssLibVersion.c_GenVerNum = M_FE_FECSSLIB_VERSION_GEN;
        p_apiResData->z_FecssLibVersion.c_MajorVerNum = M_FE_FECSSLIB_VERSION_MAJOR;
        p_apiResData->z_FecssLibVersion.c_MinorVerNum = M_FE_FECSSLIB_VERSION_MINOR;
        p_apiResData->z_FecssLibVersion.c_BuildVerNum = M_FE_FECSSLIB_VERSION_BUILD;
        p_apiResData->z_FecssLibVersion.c_Date = M_FE_FECSSLIB_VERSION_DAY;
        p_apiResData->z_FecssLibVersion.c_Month = M_FE_FECSSLIB_VERSION_MONTH;
        p_apiResData->z_FecssLibVersion.c_Year = M_FE_FECSSLIB_VERSION_YEAR;
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_mmWaveDfpVerGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS device clock control API function
 *
 * @b Description @n
 * This API function is used to gate (FECSS Sleep) and configure the FECSS device and
 * frame time clock source dynamically based on application low power mode.
 *
 * @note This API can be used to enter FECSS fast sleep by gating the entire FECSS system clock.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_CLK_CTRL_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-300, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_DEV_CLK_CTRL
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssDevClockCtrl(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_apiCmdData)
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
        *   - call fe_fecssDevClockCtrl command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS Device clock control API  */
            xw_return = fe_fecssDevClockCtrl(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssDevClockCtrl API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS device RDIF control API function
 *
 * @b Description @n
 * This API function is used to configure and control the FECSS device RDIF block, The Application
 * can use this API to collect the radar Data over CMOS interface pins using this API.
 *
 * @b Assumption: The FECSS common Profile config is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_RDIF_CTRL_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-301, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RDIF_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssDevRdifCtrl(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RDIF_CTRL_CMD *p_apiCmdData)
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
        *   - call rfs_cmdFecssRdifCtrl command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call RFS Rdif control API  */
            xw_return = rfs_cmdFecssRdifCtrl(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssDevRdifCtrl API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS RF Factory calibration API function
 *
 * @b Description @n
 * This API function configures and triggers the FECSS RF boot calibrations. The application
 * shall use this API to perform one time RF boot calibration (factory calibration) before
 * storing the calibration data in non volatile memory.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_RF_FACT_CAL_CMD
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_FECSS_RF_FACT_CAL_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-302, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RF_FACTORY_CAL
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfFactoryCal(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RF_FACT_CAL_CMD *p_apiCmdData, \
                T_RL_API_FECSS_RF_FACT_CAL_RSP* p_apiResData)
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
        *   - call rfs_cmdRfFactCal command
        * */
        if ((M_NULL_PTR == p_apiCmdData) || (M_NULL_PTR == p_apiResData))
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RF factory calibration API  */
            xw_return = rfs_cmdRfFactCal(c_devIndex, p_apiCmdData, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfFactoryCal API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS runtime Tx CLPC calibration API function
 *
 * @b Description @n
 * mmWave FECSS RF TX CLPC calibration configuration and trigger SET API.
 * This API shall be used to configure and trigger TX CLPC calibrations.
 * @note
 * This is a special advance API to improve the accuracy of TX power in higher Back Off conditions.
 * Using this API user can build calibrated TX bias code LUT externally and feed
 * them to device in run time in various temperature bins.
 *
 * @b Assumptions: @n
 * - The FECSS RF ON is done before issuing this API.
 * - The APLL should be on before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-304, MMWAVE_DFP_REQ-471
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
T_RETURNTYPE rl_fecssRlRuntimeTxClpcCal(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD *p_apiCmdData, \
                T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP* p_apiResData)
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
        *   - call rfs_cmdRfTxRuntimeClpcCal command
        * */
        if ((M_NULL_PTR == p_apiCmdData) || (M_NULL_PTR == p_apiResData))
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RF factory calibration API  */
            xw_return = rfs_cmdRfTxRuntimeClpcCal(c_devIndex, p_apiCmdData, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRlRuntimeTxClpcCal API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS RF Runtime calibration (calibrate) API function
 *
 * @b Description @n
 * This API function configures and triggers the FECSS RF runtime calibrations. The application
 * shall use this API to perform runtime RF calibration based on change in device temperature
 * or time, typically this API shall be called before sensor start after issuing chirp profile
 * config API.
 *
 * @b Assumption: The sensor chirp profile configuration is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_RF_RUN_CAL_CMD
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_FECSS_RF_RUN_CAL_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-303, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RF_RUN_CAL
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfRuntimeCal(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RF_RUN_CAL_CMD *p_apiCmdData, \
                T_RL_API_FECSS_RF_RUN_CAL_RSP* p_apiResData)
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
        *   - call rfs_cmdRfCalibrate command
        * */
        if ((M_NULL_PTR == p_apiCmdData) || (M_NULL_PTR == p_apiResData))
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RF calibrate (runtime) API  */
            xw_return = rfs_cmdRfCalibrate(c_devIndex, p_apiCmdData, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfRuntimeCal API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS RF clock BW configuration API function
 *
 * @b Description @n
 * This API function configures the FECSS RF APLL and SYNTH clock bandwidth. The application
 * shall use this API to change the BW based on extrnal factors. Typically this API shall
 * be called before run time calibration trigger API.
 *
 * @b Assumption: The sensor RF ON-OFF and calibration data restore is done before
 * issuing this API.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_apiCmdData  - pointer to response data @ref T_RL_API_FECSS_CLK_BW_CFG_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-305, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RF_CLKBW_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfClockBwCfg(UINT8 c_devIndex, const T_RL_API_FECSS_CLK_BW_CFG_CMD *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command pointer is NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdRfClockBwCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RF clock BW cfg API  */
            xw_return = rfs_cmdRfClockBwCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfClockBwCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS GPADC measurement configuration API function
 *
 * @b Description @n
 * This API function configures the FECSS GPADC in RFS IPC. The application
 * shall use this API to configure the internal or external GPADC signals. \n
 * This is a configuration only API, this configuration is retained in FECSS RAM during
 * deep sleep / warm reset.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_GPADC_MEAS_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-306, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_GPADC_MEAS_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssGpadcMeasCfg(UINT8 c_devIndex, \
                const T_RL_API_FECSS_GPADC_MEAS_CMD *p_apiCmdData)
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
        *   - call rfs_cmdGpadcCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS GAPDC CFG API  */
            xw_return = rfs_cmdGpadcCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssGpadcMeasCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS GPADC measurement Trigger API function
 *
 * @b Description @n
 * This API function triggeres the FECSS GPADC measurement in RFS. The application
 * shall use this API to measure the internal or external GPADC signals. \n
 * This is a trigger only API, the configuration is done using @ref rl_fecssGpadcMeasCfg API.
 *
 * @b Assumption: The FECSS GPADC config is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData   - pointer to response data @ref T_RL_API_FECSS_GPADC_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-307, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_GPADC_MEAS_TRIG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssGpadcMeasTrig(UINT8 c_devIndex, \
                T_RL_API_FECSS_GPADC_MEAS_RSP *p_apiResData)
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
        *   - call rfs_cmdGpadcTrig command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS GAPDC Trigger API  */
            xw_return = rfs_cmdGpadcTrig(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssGpadcMeasTrig API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/*!*****************************************************************************
 * @brief mmWaveLink API FECSS Temperature measurement configuration API function
 *
 * @b Description @n
 * This API function configures the FECSS Temp sensors in RFS IPC. The application
 * shall use this API to configure the temperature sensors. \n
 * This is a configuration only API, this configuration is retained in FECSS RAM during
 * deep sleep / warm reset.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to command data @ref T_RL_API_FECSS_TEMP_MEAS_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-308, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_TEMP_MEAS_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssTempMeasCfg(UINT8 c_devIndex, \
                const T_RL_API_FECSS_TEMP_MEAS_CMD *p_apiCmdData)
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
        *   - call rfs_cmdTempCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS Temp CFG API  */
            xw_return = rfs_cmdTempCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssTempMeasCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS Temperature measurement Trigger API function
 *
 * @b Description @n
 * This API function triggeres the FECSS Temp measurement in RFS. The application
 * shall use this API to measure the RFS temperature sensors. \n
 * This is a trigger only API, the configuration is done using @ref rl_fecssTempMeasCfg API.
 *
 * @b Assumption: The FECSS Temp config is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_FECSS_TEMP_MEAS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-309, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_TEMP_MEAS_TRIG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssTempMeasTrig(UINT8 c_devIndex, \
                T_RL_API_FECSS_TEMP_MEAS_RSP *p_apiResData)
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
        *   - call rfs_cmdTempTrig command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS Temp Trigger API  */
            xw_return = rfs_cmdTempTrig(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssTempMeasTrig API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API RF factory calibration data GET
 *
 * @b Description @n
 * This API function gets the RF factory Calibration data from FEC. Application can store the
 * calibration data in Flash or external memory using this API.
 *
 * @b Assumption: The sensor RF ON-OFF, factory calibrations shall be done before
 * issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_FECSS_FACT_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-310, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_FACTORY_CAL_DATA_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfFactoryCalDataGet(UINT8 c_devIndex, \
    T_RL_API_FECSS_FACT_CAL_DATA *p_apiResData)
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
        *   - call rfs_cmdFactCalDataGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS rf cal data GET API  */
            xw_return = rfs_cmdFactCalDataGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfFactoryCalDataGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API RF factory calibration data SET
 *
 * @b Description @n
 * This API function sets the RF factory Calibration data to FEC. Application can restore the
 * calibration data to FEC using this API.
 *
 * @b Assumption: The sensor RF ON-OFF shall be done before issuing this API.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_apiCmdData  - pointer to response data @ref T_RL_API_FECSS_FACT_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-311, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_FACTORY_CAL_DATA_SET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfFactoryCalDataSet(UINT8 c_devIndex, \
    const T_RL_API_FECSS_FACT_CAL_DATA *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command pointer is NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdFactCalDataSet command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RF Cal data Set API  */
            xw_return = rfs_cmdFactCalDataSet(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfFactoryCalDataSet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS Device status GET
 *
 * @b Description @n
 * This API function Retrieves the fecss device status.
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData   - pointer to response data @ref T_RL_API_FECSS_DEV_STS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-312, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_DEV_STS_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssDevStatusGet(UINT8 c_devIndex, \
                T_RL_API_FECSS_DEV_STS_RSP *p_apiResData)
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
        *   - call fe_fecssDevStatusGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS Dev Status GET API  */
            xw_return = fe_fecssDevStatusGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssDevStatusGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API RF status GET
 *
 * @b Description @n
 * This API function Retrieves the RF status.
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData   - pointer to response data @ref T_RL_API_FECSS_RF_STS_GET_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-313, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RF_STS_GET
 *
 *******************************************************************************
 */

T_RETURNTYPE rl_fecssRfStatusGet(UINT8 c_devIndex, \
                T_RL_API_FECSS_RF_STS_GET_RSP *p_apiResData)
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
        *   - call rfs_cmdRfStatusGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS rf status GET API  */
            xw_return = rfs_cmdRfStatusGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfStatusGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/*!*****************************************************************************
 * @brief mmWaveLink API FECSS Device RFS Fault status GET
 *
 * @b Description @n
 * This API function Retrieves the fecss device RFS CM3 Fault status.
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData   - pointer to response data @ref T_RL_API_RFS_FAULT_STS_GET_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-314, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_FAULT_STS_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfFaultStatusGet(UINT8 c_devIndex, \
                T_RL_API_RFS_FAULT_STS_GET_RSP *p_apiResData)
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
        *   - call fe_fecssRfsFaultStatusGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS Dev Status GET API  */
            xw_return = fe_fecssRfsFaultStatusGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfFaultStatusGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API GET DIEID
 *
 * @b Description @n
 * This API function Retrievs DIE ID. @ref T_RETURNTYPE rl_fecssDieIdGet
 API.
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData   - pointer to response data @ref T_RL_API_SENSOR_DIEID_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-315, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_DIEID_GET
 *
 *******************************************************************************
 */

T_RETURNTYPE rl_fecssDieIdGet(UINT8 c_devIndex, \
                T_RL_API_SENSOR_DIEID_RSP *p_apiResData)
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
        *   - call fe_fecssDieIdGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS DIE Id GET API  */
            xw_return = fe_fecssDieIdGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssDieIdGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API RF RX-TX calibration data GET
 *
 * @b Description @n
 * This API function gets the RF RX-TX Calibration data from FEC. Application can store the
 * calibration data in Flash or external memory using this API.
 *
 * @b Assumption: The sensor RF ON-OFF, factory calibrations shall be done before
 * issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_apiResData  - pointer to response data @ref T_RL_API_FECSS_RXTX_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-316, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RXTX_CAL_DATA_GET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfRxTxCalDataGet(UINT8 c_devIndex, \
    T_RL_API_FECSS_RXTX_CAL_DATA *p_apiResData)
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
        *   - call rfs_cmdRxTxCalDataGet command
        * */
        if (M_NULL_PTR == p_apiResData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS rf cal data GET API  */
            xw_return = rfs_cmdRxTxCalDataGet(c_devIndex, p_apiResData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfRxTXCalDataGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API RF RX-TX calibration data SET
 *
 * @b Description @n
 * This API function sets the RF RX-TX Calibration data to FEC. Application can restore the
 * calibration data to FEC using this API.
 *
 * @b Assumption: The sensor RF ON-OFF shall be done before issuing this API.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_apiCmdData  - pointer to response data @ref T_RL_API_FECSS_RXTX_CAL_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-317, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RXTX_CAL_DATA_SET
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfRxTxCalDataSet(UINT8 c_devIndex, \
    const T_RL_API_FECSS_RXTX_CAL_DATA *p_apiCmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Validate device status  */
    xw_return = fe_validateDevice(c_devIndex);

    /*! -# If device is valid then execute the API  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        * -# If input command pointer is NULL then
        *   - return M_DFP_RET_CODE_NULL_PTR_ERROR
        * -# else
        *   - call rfs_cmdRxTXCalDataSet command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RF Cal data Set API  */
            xw_return = rfs_cmdRxTxCalDataSet(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfRxTxCalDataSet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief mmWaveLink API FECSS RFS Debug configuration API function
 *
 * @b Description @n
 * This API function configures the FECSS RFS Debug logger memory. \n
 * This API enables the MPU for selected debug memory and logs the RFS internal
 * debug data.
 *
 * @b Assumption: The FECSS Power ON is done before issuing this API.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[in] p_apiCmdData   - pointer to response data @ref T_RL_API_FECSS_RFS_DBG_CFG_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-318, MMWAVE_DFP_REQ-471
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 * Command ID: @ref M_RL_API_ID_FECSS_RFS_DBG_CFG
 *
 *******************************************************************************
 */
T_RETURNTYPE rl_fecssRfsDbgCfg(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RFS_DBG_CFG_CMD *p_apiCmdData)
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
        *   - call rfs_cmdDebugCfg command
        * */
        if (M_NULL_PTR == p_apiCmdData)
        {
            xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
        }
        else
        {
            /*! -# Call FECSS RFS Debug Cfg API  */
            xw_return = rfs_cmdDebugCfg(c_devIndex, p_apiCmdData);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("MMWL rl_fecssRfsDbgCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF rl_device.c
 */
