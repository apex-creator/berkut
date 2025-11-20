/*!*****************************************************************************
 * @file fe_driver.c
 *
 * @brief FECSSLib Device Module interface driver PATCH functions
 *
 * @b Description @n
 * This file provides PATCH functions for FECSS library Device Module interface driver
 * function service, application framework init and handling of driver data structure.
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
 * @addtogroup FECSSLIB_DEVICE_DRV_PATCH Device Interface and Driver API Patch functions
 *  @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     14Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <fecsslib/fecsslib.h>
#include <fecsslib/device/include/fe_driver.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/****************************FECSSLib Static Functions*************************/
/*!
 * @name FECSS Library Device module Interface Driver static patch function calls
 * @{
 */
static T_RETURNTYPE fe_driverDeInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
/*! @} */

/*!*****************************************************************************
 * @brief FECSS driver Deinit driver function
 *
 * @b Description @n
 * This driver API function uninitializes all the interface call back functions for each device
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] p_feDrvData   - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
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
static T_RETURNTYPE fe_driverDeInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData)
{
    T_RETURNTYPE xw_return;

    /*! -# UnInitialize OSI callback objects */
    /*!   - Un Initialize OSI semaphore objects */
    xw_return = p_feDrvData->z_ClientCbData.z_OsiCb.z_Semp.p_OsiSemDelete(\
                    p_feDrvData->z_DevData[c_devIndex].p_SemHdl);
    p_feDrvData->z_DevData[c_devIndex].p_SemHdl = M_NULL_PTR;

    /*! -# Un Initialize OSI mutex objects */
    xw_return += p_feDrvData->z_ClientCbData.z_OsiCb.z_Mutex.p_OsiMutexDelete(\
                    p_feDrvData->z_DevData[c_devIndex].p_MutexHdl);
    p_feDrvData->z_DevData[c_devIndex].p_MutexHdl = M_NULL_PTR;

    /*! -# UnInitialize platform RFS MB response interrupt */
    xw_return += p_feDrvData->z_ClientCbData.z_PltfCb.p_DeRegisterIsr(\
        p_feDrvData->z_DevData[c_devIndex].p_PltHwiHdl[M_FE_INTR0_RFS_MB_RSP]);
    p_feDrvData->z_DevData[c_devIndex].p_PltHwiHdl[M_FE_INTR0_RFS_MB_RSP] = M_NULL_PTR;

#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_FE)
    /*!
     * -# Initialize Comm IF callback objects, this is done only for
     * M_DFP_DEVICE_PLATFORM_FE devices
     */
    if (M_DFP_DEVICE_PLATFORM_FE == p_feDrvData->z_ClientCbData.c_PlatformType)
    {
        xw_return += p_feDrvData->z_ClientCbData.z_ComIfCb.p_ComIfClose(\
                p_feDrvData->z_DevData[c_devIndex].p_ComIfHdl);
    }
    p_feDrvData->z_DevData[c_devIndex].p_ComIfHdl = M_NULL_PTR;
#endif

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG2("FECCLib fe_driverDeInit:%d:%d \n", c_devIndex, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/******************************FECSSLib Driver PATCH APIs****************************/

/*!*****************************************************************************
 * @brief FECSS Library interface Deinit API function
 *
 * @b Description @n
 * This driver API function uninitializes all the interface call back functions and
 * client context data structure.
 *
 * @b Assumption: This function shall be called once for a device after calling
 * fe_fecssDevPwrOff API.
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
T_RETURNTYPE fe_fecssLibDeInit_patch(UINT32 w_deviceMap)
{
    T_RETURNTYPE xw_return;
    UINT8 c_devIndex;
    UINT32 w_locDevMapIn,
           w_devMap;

    /*! -# Get fecss driver handle global structure pointer */
    T_FE_DRIVER_DATA *p_feDrvData = fe_DrvGetHandle();

    /*!
     * -# If any w_device is not yet initialized or num of devices more than supported
     * M_DFP_MAX_DEV_MAP_MASK then
     *   - return M_DFP_RET_CODE_INVALID_DEVICE
     */
    if ((w_deviceMap & p_feDrvData->w_ActiveDevInit) == w_deviceMap)
    {
        xw_return = M_DFP_RET_CODE_INVALID_DEVICE;
    }
    else
    {
        /*!
         * -# Else The Uninitialize all interface driver handlers
         *   - clear the w_ActiveDevInit for enabled devices
         */
        p_feDrvData->w_ActiveDevInit  &= ~(w_deviceMap);
        w_locDevMapIn = w_deviceMap;

        /*! -# The FECSS power down API should have been called before DeInit, ASSERT if not done */
        M_DFP_ASSERT((p_feDrvData->w_ActiveDevInit == p_feDrvData->w_ActiveDevPwrUp), \
            xw_return, M_DFP_RET_CODE_FATAL_ERROR);

        if (M_DFP_RET_CODE_OK == xw_return)
        {
            /*! -# UnInitialize all FECSS callback objects for all devices in a loop */
            for (c_devIndex = (UINT8)0U; c_devIndex < M_DFP_MAX_DEVICES; c_devIndex++)
            {
                w_devMap = ((UINT32)1U << c_devIndex);
                /*!
                 * -# If device is enabled then
                 *   - Uninitialize all the interface drivers
                 */
                if ((w_devMap & w_locDevMapIn) != (UINT32)0U)
                {
                    xw_return = fe_driverDeInit(c_devIndex, p_feDrvData);
                }
                w_locDevMapIn &= ~w_devMap;

                /*! -# Break if all enabled devices are deinitialized and if any error */
                if (((UINT32)0U == w_locDevMapIn) || (M_DFP_RET_CODE_OK != xw_return))
                {
                    break;
                }
            }
        }
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG2("FECCLib fe_fecssLibDeInit_patch API:%x:%d \n", w_deviceMap, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF fe_driver.c
 */
