/*!*****************************************************************************
 * @file fe_driver.c
 *
 * @brief FECSSLib Device Module interface driver functions
 *
 * @b Description @n
 * This file provides FECSS library Device Module HW interface driver function services,
 * application framework init and handling of driver data structure.
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
 * @addtogroup FECSSLIB_DEVICE_DRV_MODULE Device Interface and Driver API functions
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
/*!
 * @name FECSS Library Device module Interface Driver static function calls
 * @{
 */
static T_RETURNTYPE fe_driverInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
static T_RETURNTYPE fe_pltDrvInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
static T_RETURNTYPE fe_osiDrvInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
static T_RETURNTYPE fe_driverDeInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_FE)
static T_RETURNTYPE fe_comIfDrvInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
#endif
/*! @} */

/*!*****************************************************************************
 * @brief FECSS OSI driver init function
 *
 * @b Description @n
 * This function initializes all the OSI interface call back functions for each device
 *
 * @b Assumption: The callback function NULL pointer check is done outside this function
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData  - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-406, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
static T_RETURNTYPE fe_osiDrvInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData)
{
    T_RETURNTYPE xw_return;
    T_DFP_OSI_SEM_HDL p_osiHdl = M_NULL_PTR;
    /*! -# Init semaphore and Mutex strings */
    UINT8 c_semStr[9U] = "DevSem_";
    UINT8 c_mutStr[9U] = "DevMut_";

    /*!
     * -# Append Device index to c_semStr & c_mutStr string and
     *   - Append End of string string
     */
    c_semStr[7U] = (UINT8)0x30 + (UINT8)c_devIndex;
    c_mutStr[7U] = c_semStr[7U];
    c_semStr[8U] = (UINT8)('\0');
    c_mutStr[8U] = c_semStr[8U];

    /*! -# Initialize OSI Semaphore object and update return value */
    xw_return = p_feDrvData->z_ClientCbData.z_OsiCb.z_Semp.p_OsiSemCreate(&p_osiHdl, \
                        &c_semStr[0U]);

    /*!
     * -# Update Semaphore handler pointer to created object if no error
     *   - Initialize OSI Mutex object
     */
    if ((xw_return == M_DFP_OSI_RET_CODE_OK) && (M_NULL_PTR != p_osiHdl))
    {
        p_feDrvData->z_DevData[c_devIndex].p_SemHdl = p_osiHdl;
        xw_return = p_feDrvData->z_ClientCbData.z_OsiCb.z_Mutex.p_OsiMutexCreate(&p_osiHdl, \
                        &c_mutStr[0U]);
    }
    /*!
     * -# else in case of error Clear semaphore handler pointer
     *   - update xw_return value */
    else
    {
        p_feDrvData->z_DevData[c_devIndex].p_SemHdl = M_NULL_PTR;
        xw_return = M_DFP_RET_CODE_RADAR_OSIF_ERROR;
    }

    /*! -# If any error Clear Mutex handler pointer and update xw_return value */
    if ((xw_return != M_DFP_OSI_RET_CODE_OK) || (p_osiHdl == M_NULL_PTR))
    {
        p_feDrvData->z_DevData[c_devIndex].p_MutexHdl = M_NULL_PTR;
        xw_return = M_DFP_RET_CODE_RADAR_OSIF_ERROR;
    }
    /*!
     * -# else update Mutex handler pointer to created object
     *   - update xw_return value
     */
    else
    {
        p_feDrvData->z_DevData[c_devIndex].p_MutexHdl = (T_DFP_OSI_MUTEX_HDL)p_osiHdl;
        xw_return = M_DFP_RET_CODE_OK;
    }

    M_DFP_LOG_INFO_ARG2("FECCLib fe_osiDrvInit:%d:%d \n", c_devIndex, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Platform driver init function
 *
 * @b Description @n
 * This function initializes all the platform interface call back functions for each device
 *
 * @b Assumption: The callback function NULL pointer check is done outside this function
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-407, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
static T_RETURNTYPE fe_pltDrvInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData)
{
    T_RETURNTYPE xw_return;
    T_DFP_PLT_HWI_HDL p_hwiHdl = M_NULL_PTR;
    /*! Device FE MB ISR registration  */
    UINT8 c_irqType = M_DFP_FE_DEV_INTR0_RFS_MB_RSP(c_devIndex);
    T_DFP_PLT_ISR_HANDLER p_isrHandler = &rfs_cmdRespDev0IntHandler;

    /*! -# Register RFS MB interrupt handler here */
    xw_return = p_feDrvData->z_ClientCbData.z_PltfCb.p_RegisterIsr(c_devIndex, \
        p_isrHandler, (void*)&c_irqType, &p_hwiHdl);

    /*!
     * -# Update platform handler pointer to created HWi handler if no error
     *   - Initialize platform HWI handler
     */
    if ((xw_return == M_DFP_OSI_RET_CODE_OK) && (M_NULL_PTR != p_hwiHdl))
    {
        p_feDrvData->z_DevData[c_devIndex].p_PltHwiHdl[M_FE_INTR0_RFS_MB_RSP] = p_hwiHdl;
    }
    /*!
     * -# else in case of error Clear semaphore handler pointer
     *   - update xw_return value */
    else
    {
        p_feDrvData->z_DevData[c_devIndex].p_PltHwiHdl[M_FE_INTR0_RFS_MB_RSP] = M_NULL_PTR;
        xw_return = M_DFP_RET_CODE_RADAR_PLTIF_ERROR;
    }

    /*!
     * -# Print logger Debug info
     */
    M_DFP_LOG_INFO_ARG2("FECCLib fe_pltDrvInit:%d:%d \n", c_devIndex, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_FE)
/*!*****************************************************************************
 * @brief FECSS Comm interface driver init function
 *
 * @b Description @n
 * This function initializes all the communication interface call back functions for each device
 *
 * @b Assumption: The callback function NULL pointer check is done outside this function
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData  - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-408, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
static T_RETURNTYPE fe_comIfDrvInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData)
{
    T_RETURNTYPE xw_return;
    T_DFP_COMIF_HDL p_comIfHdl;

    /*! -# Register all communication interface handler here  */
    /*! -# Open communication interface handle */
    p_comIfHdl = p_feDrvData->z_ClientCbData.z_ComIfCb.p_ComIfOpen(c_devIndex, 0U);

    /*! -# If any error update the xw_return value */
    /*! -# Print logger Debug info */
    if (M_NULL_PTR == p_comIfHdl)
    {
        xw_return = M_DFP_RET_CODE_RADAR_CIF_ERROR;
        p_feDrvData->z_DevData[c_devIndex].p_ComIfHdl = M_NULL_PTR;
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
        p_feDrvData->z_DevData[c_devIndex].p_ComIfHdl = p_comIfHdl;
    }

    M_DFP_LOG_INFO_ARG2("FECCLib fe_comIfDrvInit:%d:%d \n", c_devIndex, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}
#endif

/*!*****************************************************************************
 * @brief FECSS driver init function
 *
 * @b Description @n
 * This function initializes all the interface call back functions for each device
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-409, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
static T_RETURNTYPE fe_driverInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData)
{
    T_RETURNTYPE xw_return;

    /*! -# Initialize OSI callback objects */
    xw_return = fe_osiDrvInit(c_devIndex, p_feDrvData);

    /*! -# Initialize Platform callback objects */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        xw_return = fe_pltDrvInit(c_devIndex, p_feDrvData);
    }

#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_FE)
    /*!
     * -# Initialize Comm IF callback objects, this is done only for
     * M_DFP_DEVICE_PLATFORM_FE devices
     */
    if ((M_DFP_DEVICE_PLATFORM_FE == p_feDrvData->z_ClientCbData.c_PlatformType) && \
        (M_DFP_RET_CODE_OK == xw_return))
    {
        xw_return = fe_comIfDrvInit(c_devIndex, p_feDrvData);
    }
#endif

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG2("FECCLib fe_driverInit:%d:%d \n", c_devIndex, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS driver Deinit driver function
 *
 * @b Description @n
 * This driver API function uninitializes all the interface call back functions for each device
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData  - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-410, MMWAVE_DFP_REQ-472
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

/******************************FECSSLib Driver APIs****************************/

#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
/*!*****************************************************************************
 * @brief FECSS HW memory memcopy function
 *
 * @b Description @n
 * This driver function supports memory clear in FECSS HW.
 * The normal memclr provided by compiler shall be avoided to copy FECSS Memory.
 *
 * @b Assumption: The MPU settings are configured for FECSS memory as per recommendation.
 *
 * @param[in, out] p_srcAdd    - Destination Address
 * @param[in] h_numOfWords      - Num of words (4 bytes), max 65536 words
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-411, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_hwMemclr(UINT32 * const p_srcAdd, UINT16 h_numOfWords)
{
    UINT16 h_index;
    REG32* p_data;
    T_RETURNTYPE xw_return;

    if (M_NULL_PTR == p_srcAdd)
    {
        /*! -# Return M_DFP_RET_CODE_NULL_PTR_ERROR if pointer is NULL */
        xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
    }
    else if ((UINT16)0U == h_numOfWords)
    {
        /*! -# Return M_DFP_RET_CODE_INVALID_INPUT if num of words is zero */
        xw_return = M_DFP_RET_CODE_INVALID_INPUT;
    }
    else
    {
        /*! -# else Return M_DFP_RET_CODE_OK */
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*!
     * -# Clear memory in a for loop for h_numOfWords if xw_return status is M_DFP_RET_CODE_OK
     */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        p_data = (REG32*)p_srcAdd;

        for (h_index = (UINT16)0U; h_index < h_numOfWords; h_index++)
        {
            *p_data = 0U;
            p_data++;
        }
    }

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS HW memory memcopy function
 *
 * @b Description @n
 * This driver function supports memory copy from source to destination in FECSS HW.
 * The normal memcopy provided by compiler shall be avoided to copy FECSS Memory.
 *
 * @b Assumption: The MPU settings are configured for FECSS memory as per recommendation.
 *
 * @param[in, out] p_destAdd    - Destination Address
 * @param[in] p_sourceAdd       - Source Address
 * @param[in] h_numOfWords      - Num of words (4 bytes), max 65536 words
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-412, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_hwMemcopy(UINT32 * const p_destAdd, const UINT32 * const p_sourceAdd, UINT16 h_numOfWords)
{
    UINT16       h_index;
    T_RETURNTYPE xw_return;
    UINT32 w_memClear = (UINT32)0U,
           w_memClrMask = (UINT32)0xFFFFFFFFU;
    UINT32 w_destAddLoc = (UINT32)p_destAdd,
           w_sourceAddLoc = (UINT32)p_sourceAdd;
    UINT32 w_destOffset,
           w_srcOffset;

    if (M_NULL_PTR == p_destAdd)
    {
        /*! -# Return M_DFP_RET_CODE_NULL_PTR_ERROR if pointer is NULL */
        xw_return = M_DFP_RET_CODE_NULL_PTR_ERROR;
    }
    else if ((UINT16)0U == h_numOfWords)
    {
        /*! -# Return M_DFP_RET_CODE_INVALID_INPUT if num of words is zero */
        xw_return = M_DFP_RET_CODE_INVALID_INPUT;
    }
    else
    {
        /*! -# else Return M_DFP_RET_CODE_OK */
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# if p_sourceAdd 0 then the request is memclear */
    if (M_NULL_PTR == p_sourceAdd)
    {
        w_sourceAddLoc = (UINT32)(&w_memClear);
        w_memClrMask = (UINT32)0U;
    }

    /*!
     * -# Copy Data from source memory to destination memory in for loop
     * for h_numOfWords if xw_return status is M_DFP_RET_CODE_OK
     */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        for (h_index = (UINT16)0U; h_index < h_numOfWords; h_index++)
        {
            /*! -# Increment source address */
            w_destOffset = (UINT32)h_index * (UINT32)4U;
            /*! -# Increment destination address */
            w_srcOffset  = w_destOffset & w_memClrMask;
            /*! -# Copy Data from source memory to destination memory */
            *((REG32*)(w_destAddLoc + w_destOffset)) = \
                    *((REG32*)(w_sourceAddLoc + w_srcOffset));
        }
    }

    /*! -# Return xw_return value */
    return xw_return;
}
#else
/*!*****************************************************************************
 * @brief FECSS HW memory read function for FE devices
 *
 * @b Description @n
 * This driver function supports memory read from FECSS FE HW source memory to SW destination over
 * SPI communication interface. The FE HW supports SPI interface to read the conetnt from sensor.
 *
 * @b Assumption: The callback functions are registered during mmWaveLink interface
 *
 * @param[in]  c_devInd         - Device Index for FE devices
 * @param[in, out] p_destAdd    - Destination SW Address
 * @param[in] p_sourceAdd       - Source HW Address
 * @param[in] h_numOfWords      - Num of words (4 bytes), max 65536 words
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-413, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_comIfRead(UINT8 c_devInd, UINT32 *p_destAdd, \
                    const UINT32 *p_sourceAdd, UINT16 h_numOfWords)
{
    /*! -# Get fecss driver handle global structure pointer */
    T_FE_DRIVER_DATA *p_feDrvData = fe_DrvGetHandle();
    T_DFP_COMIF_HDL p_fd = p_feDrvData->z_DevData[c_devInd].p_ComIfHdl;
    UINT32 w_numBytes = (UINT32)h_numOfWords * (UINT32)4U;
    T_RETURNTYPE xw_return;

    xw_return = p_feDrvData->z_ClientCbData.z_ComIfCb.p_ComIfRead(p_fd, (UINT8 *)p_destAdd, \
        (const UINT8 *)p_sourceAdd, w_numBytes);

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS HW memory write function for FE devices
 *
 * @b Description @n
 * This driver function supports memory write from FECSS FE HW destination memory from SW
 * buffer over SPI communication interface. The FE HW supports SPI interface to read the
 * conetnt from sensor.
 *
 * @b Assumption: The callback functions are registered during mmWaveLink interface
 *
 * @param[in]  c_devInd         - Device Index for FE devices
 * @param[in, out] p_destAdd    - Destination HW Address
 * @param[in] p_sourceAdd       - Source SW Address
 * @param[in] h_numOfWords      - Num of words (4 bytes), max 65536 words
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-414, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_comIfWrite(UINT8 c_devInd, UINT32 *p_destAdd, \
                    const UINT32 *p_sourceAdd, UINT16 h_numOfWords)
{
    /*! -# Get fecss driver handle global structure pointer */
    T_FE_DRIVER_DATA *p_feDrvData = fe_DrvGetHandle();
    T_DFP_COMIF_HDL p_fd = p_feDrvData->z_DevData[c_devInd].p_ComIfHdl;
    UINT32 w_numBytes = (UINT32)h_numOfWords * (UINT32)4U;
    T_RETURNTYPE xw_return;

    xw_return = p_feDrvData->z_ClientCbData.z_ComIfCb.p_ComIfWrite(p_fd, (UINT8 *)p_destAdd, \
        (const UINT8 *)p_sourceAdd, w_numBytes);

    return xw_return;
}
#endif

/*!*****************************************************************************
 * @brief FECSS driver handle get
 *
 * @b Description @n
 * This function returns the FECSS driver handler pointer
 *
 * @b Assumption: This function always returns non NULL pointer
 *
 * @retval T_FE_DRIVER_DATA* - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-415, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_FE_DRIVER_DATA* fe_DrvGetHandle(void)
{
    /*! -# Get ptr address holds the z_FeDriverData address */
    REG32* p_regPtr = M_FE_DRV_DATA_ADDRESS_PTR;
    /*! -# Read z_FeDriverData address from ptr register */
    T_FE_DRIVER_DATA* p_feDrvData = (T_FE_DRIVER_DATA*)(*p_regPtr);

    /*! -# return z_FeDriverData pointer/handle */
    return (p_feDrvData);
}

/*!*****************************************************************************
 * @brief DFP FECSS logger function driver pointer get
 *
 * @b Description @n
 * This function returns the DFP FECSS logger function driver pointer
 *
 * @b Assumption: This function is called only if logger is enabled by application
 *                with callback function initialized
 *
 * @retval T_DFP_PRINT_FUNCP - pointer to logger function @ref T_DFP_PRINT_FUNCP
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-416, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_DFP_PRINT_FUNCP fe_getDbgLogFptr(void)
{
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();

    /*! -# return logger function pointer */
    return (p_feDrvData->z_ClientCbData.z_DbgCb.p_Print);
}

/*!*****************************************************************************
 * @brief FECSS Device validation function
 *
 * @b Description @n
 * This driver function validates device ID passed to API againest active device power ON
 *
 * @b Assumption: None
 *
 * @param[in]  c_devIndex         - Device Index for FE devices
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-417, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_validateDevice(UINT8 c_devIndex)
{
    /*! -# Get fecss driver handle global structure pointer */
    T_FE_DRIVER_DATA *p_feDrvData = fe_DrvGetHandle();
    T_RETURNTYPE xw_return;
    UINT32 w_devMap;

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        w_devMap = ((UINT32)1U << c_devIndex);

        /*!
        * -# The FECSS should be in power up state now for selected device
        *    - Return M_DFP_RET_CODE_DEVICE_NOT_POWERED if not powered up
        *    - Return M_DFP_RET_CODE_OK if device is powered up
        */
        if ((w_devMap & p_feDrvData->w_ActiveDevPwrUp) != w_devMap)
        {
            xw_return = M_DFP_RET_CODE_DEVICE_NOT_POWERED;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS interface init API function
 *
 * @b Description @n
 * This driver API function initializes all the interface call back functions and
 * client context data structure.
 *
 * @b Assumption:
 * -# This function shall be called once for all devices to init
 * FECSS SW framework or the call back client context data shall be identical
 * across device threads.
 * -# The callback function NULL pointer check is done outside this function.
 *
 * @param[in] w_deviceMap    - Device Map
 * @param[in] z_clientCbData - API input data structure, FECSS client call back
 *                             @ref T_DFP_CLIENT_CB_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-418, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssLibInit_rom(UINT32 w_deviceMap, T_DFP_CLIENT_CB_DATA z_clientCbData)
{
    T_RETURNTYPE xw_return;
    UINT8 c_devIndex;
    UINT32 w_locDevMapIn,
           w_devMap;
    UINT32 w_cbDataSize;
    UINT8 c_ind;
    UINT32 *p_destMem = M_NULL_PTR,
           *p_srcMem = M_NULL_PTR;

    /*! -# Get fecss driver handle global structure pointer */
    T_FE_DRIVER_DATA *p_feDrvData = fe_DrvGetHandle();

    /*!
     * -# If w_deviceMap = 0 or p_feDrvData is NULL then
     *   - return M_DFP_RET_CODE_INVALID_INPUT
     */
    if (M_NULL_PTR == p_feDrvData)
    {
        xw_return = M_DFP_RET_CODE_INVALID_INPUT;
    }
    /*!
     * -# If any w_device is already initialized
     *   - return M_DFP_RET_CODE_INVALID_DEVICE
     */
    else if ((w_deviceMap & p_feDrvData->w_ActiveDevInit) != (UINT32)0U)
    {
        xw_return = M_DFP_RET_CODE_INVALID_DEVICE;
    }
    else
    {
        /*!
         * -# The z_clientCbData error checks are expected to be done in mmWaveLink layer
         *   - copy z_clientCbData to p_feDrvData->z_ClientCbData
         *   - clear the w_ActiveDevInit for enabled devices
         */
        /*! -# Avoid compiler memcpy function */
        p_destMem = (UINT32*)&p_feDrvData->z_ClientCbData;
        p_srcMem = (UINT32*)&z_clientCbData;
        w_cbDataSize = ((UINT32)sizeof(T_DFP_CLIENT_CB_DATA)) / (UINT32)4U;
        for (c_ind = 0U; c_ind < (UINT8)w_cbDataSize; c_ind++)
        {
            *p_destMem = *p_srcMem;
            p_srcMem++;
            p_destMem++;
        }
        p_feDrvData->w_ActiveDevInit  &= ~(w_deviceMap);
        w_locDevMapIn = w_deviceMap;

        /*! -# The FECSS should be in power down state now */
        M_DFP_ASSERT((p_feDrvData->w_ActiveDevInit == p_feDrvData->w_ActiveDevPwrUp), \
            xw_return, M_DFP_RET_CODE_FATAL_ERROR);

        if (xw_return == M_DFP_RET_CODE_OK)
        {
            /*! -# Initialize all FECSS callback objects for all devices */
            for (c_devIndex = (UINT8)0U; c_devIndex < M_DFP_MAX_DEVICES; c_devIndex++)
            {
                w_devMap = ((UINT32)1U << c_devIndex);
                if ((w_devMap & w_locDevMapIn) != (UINT32)0U)
                {
                    xw_return = fe_driverInit(c_devIndex, p_feDrvData);
                }
                w_locDevMapIn &= ~w_devMap;

                if ((xw_return != M_DFP_RET_CODE_OK) || ((UINT32)0U == w_locDevMapIn))
                {
                    break;
                }
            }
        }

        /*! -# If no error in initialization then set the w_ActiveDevInit */
        if (M_DFP_RET_CODE_OK == xw_return)
        {
            p_feDrvData->w_ActiveDevInit |= w_deviceMap;
        }
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG2("FECCLib fe_fecssLibInit_rom API:%x:%d \n", w_deviceMap, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Library interface Deinit API function
 *
 * @b Description @n
 * This driver API function uninitializes all the interface call back functions and
 * client context data structure.
 *
 * @b Assumption: This function shall be called once for a device after calling
 * fe_fecssDevPwrOff api.
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
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-419, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssLibDeInit_rom(UINT32 w_deviceMap)
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
    if ((w_deviceMap & p_feDrvData->w_ActiveDevInit) != w_deviceMap)
    {
        xw_return = M_DFP_RET_CODE_INVALID_DEVICE;
    }
    else
    {
        /*!
         * -# Else The Uninitialize all interface driver handlers
         *   - clear the w_ActiveDevInit for enabled devices
         *   - clear the w_ActiveDevPwrUp for enabled devices
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
    M_DFP_LOG_INFO_ARG2("FECCLib fe_fecssLibDeInit_rom API:%x:%d \n", w_deviceMap, xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Open function
 *
 * @b Description @n
 * This driver API function does power up of FECSS subsystem, the FECSS clock
 * will be set to user defined clock
 *
 * @b Assumption: This function shall be called first in the sequence to power-up
 * the FECSS before configuring other settings related to FECSS.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] c_clkSrcSel   - FECSS clock source select
 * @param[in] c_powerMode   - FECSS power ON mode
 * @param[in, out] p_feDrvData  - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-420, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssOpen(UINT8 c_devIndex, UINT8 c_clkSrcSel, \
        UINT8 c_powerMode, const T_FE_DRIVER_DATA* p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    REG32 *p_regFecPdCtrl = M_FE_TOPRCM_FEC_PD_CTRL_REG_ADDRESS;
    UINT32 w_regValue,
           w_regCmpSts = (UINT32)0U;
    T_FECRCM_REGS *p_structFecRcm = M_FE_FECSS_RCM_REG_START_ADDRESS;
    REG32 *p_regFecRcm;
    REG32 *p_regFecPdSts = M_FE_TOPRCM_FEC_PD_STATUS_REG_ADDRESS;
    UINT32 w_timeout = (UINT32)0U,
           w_timeOutPeriod;

    /*! -# Ungate FECSS clock before powerup */
    xw_return = fe_fecssClockGate(c_devIndex, c_clkSrcSel);

    /*! -# Power up the FECSS PD
     *    - During powerup entire FECSS and M3 go through system reset
     */
    w_regValue = M_FE_TOPRCM_FEC_PD_CTRL_PWR_UP_VAL;
    M_REG32_SWRITE(c_devIndex, p_regFecPdCtrl, w_regValue, w_regCmpSts, xw_return1);
    xw_return += xw_return1;

    /*!
    * -# Wait for FECSS powerup completion, it takes ~300us
    *    - Poll for update till API timeout
    *    - Use 2us delay in loop
    */
    w_timeOutPeriod = \
        (UINT32)p_feDrvData->z_ClientCbData.h_ApiRespTimeoutMs * (UINT32)1000U;
    do
    {
        /*! -# 2us delay */
        xw_return += p_feDrvData->z_ClientCbData.z_PltfCb.p_Delay(20U);
        w_timeout += (UINT32)2U;

        /*! -# Read FEC PD status */
        M_REG32_READ(c_devIndex, p_regFecPdSts, w_regValue, xw_return1);
        xw_return += xw_return1;
    } while ((M_DFP_RET_CODE_OK == xw_return) && \
        (M_FE_TOPRCM_FEC_PD_STATUS_PWR_UP != (w_regValue & M_FE_TOPRCM_FEC_PD_ON_STATUS_MASK)) && \
        (w_timeout < w_timeOutPeriod));

    /*! -# If there is a API time out return error */
    if (w_timeout >= w_timeOutPeriod)
    {
        xw_return = M_DFP_RET_CODE_FEC_POWERUP_TIMEOUT;
    }

    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
        *  -# Ungate FECSS IP register space clock
        *     - Ungate chirp timer and DFE
        *     - Gate ANA, GPADC, RTI (RFS) space
        */
        p_regFecRcm = (REG32 *)(&p_structFecRcm->r_Ipcfgclkgate0.b32_Reg);
        w_regValue = M_FE_FECSS_RCM_SPACE_CLK_UNGATE_VALUE;
        M_REG32_SWRITE(c_devIndex, p_regFecRcm, w_regValue, w_regCmpSts, xw_return);

        /*!
        * -# Set FECSS clock as per c_clkSrcSel
        *    - Fast clock DIG_PLL or APLL (160MHz) / DIV (2) = 80MHz
        *    - XTAL clock is 40MHz (typical) / DIV 1 = 40MHz
        */
       xw_return1 = fe_fecssClockSwitch(c_devIndex, c_clkSrcSel);
       xw_return += xw_return1;

        /*!
         * -# Perform FECSS RFS data RAM memory init in case of cold boot
         *   - ECC control is done by APPSS / SDL
         *   - Mem init Bank 1 (DATA RAM)
         *   - Mem init Chirp RAM in parallel
         */
        if ((M_RL_FECSS_PWR_ON_MODE_COLD == c_powerMode) && \
                (M_DFP_RET_CODE_OK == xw_return))
        {
            /*! -# FECSS Mem init function call
             *     - Wait till max API time out
             */
            xw_return = fe_fecssMemInit(c_devIndex, p_feDrvData);
        }

    }

    /*! -# Perform register readback error check */
    M_DFP_ASSERT(((UINT32)0U == w_regCmpSts), xw_return1, M_DFP_RET_CODE_REG_READBACK_ERROR);
    xw_return += xw_return1;

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Close function
 *
 * @b Description @n
 * This driver API function does power down of FECSS subsystem, the FECSS clock
 * will be gated after the power down
 *
 * @b Assumption: This function shall be called last in the sequence to power-down
 * the FECSS after configuring other settings related to FECSS.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] c_powerMode   - FECSS power ON mode
 * @param[in, out] p_feDrvData  - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-421, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssClose(UINT8 c_devIndex, UINT8 c_powerMode, \
                const T_FE_DRIVER_DATA* p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    REG32 *p_regFecPdCtrl = M_FE_TOPRCM_FEC_PD_CTRL_REG_ADDRESS;
    REG32 *p_regFecMemSt = M_FE_TOPRCM_FEC_PD_CLUST1_RET_REG_ADDRESS;
    UINT32 w_regValue,
           w_regCmpSts = (UINT32)0U;
    REG32 *p_regFecPdSts = M_FE_TOPRCM_FEC_PD_STATUS_REG_ADDRESS;
    UINT32 w_timeout = (UINT32)0U,
           w_timeOutPeriod;

    /*! -# Gate FECSS clock before power down */
    xw_return = fe_fecssClockGate(c_devIndex, M_RL_FECSS_DEV_CLK_SRC_OFF);


    /*! -# FECSS memory setting before entering deep sleep as per c_powerMode */
    if (M_RL_FECSS_PWR_DOWN_RET_ON == c_powerMode)
    {
        /*! -# Retain memory for FECC RAM */
        w_regValue = M_FE_TOPRCM_FEC_PD_CLUST1_RAM_STATE_ON_VAL;
    }
    else
    {
        /*! -# Switch OFF memory for FECC RAM */
        w_regValue = M_FE_TOPRCM_FEC_PD_CLUST1_RAM_STATE_OFF_VAL;
    }
    M_REG32_SWRITE(c_devIndex, p_regFecMemSt, w_regValue, w_regCmpSts, xw_return1);
    xw_return += xw_return1;

    /*! -# Power down the FECSS PD, this power down the FECSS except memory */
    w_regValue = M_FE_TOPRCM_FEC_PD_CTRL_PWR_DOWN_VAL;
    M_REG32_SWRITE(c_devIndex, p_regFecPdCtrl, w_regValue, w_regCmpSts, xw_return1);
    xw_return += xw_return1;

    /*!
    * -# Wait for FECSS powerdown completion, it takes ~300us
    *    - Poll for update till API timeout
    *    - Use 2us delay in loop
    */
    w_timeOutPeriod = \
        (UINT32)p_feDrvData->z_ClientCbData.h_ApiRespTimeoutMs * (UINT32)1000U;
    do
    {
        /*! -# 2us delay */
        xw_return += p_feDrvData->z_ClientCbData.z_PltfCb.p_Delay(20U);
        w_timeout += (UINT32)2U;

        /*! -# Read FEC PD status */
        M_REG32_READ(c_devIndex, p_regFecPdSts, w_regValue, xw_return1);
        xw_return += xw_return1;
    } while ((M_DFP_RET_CODE_OK == xw_return) && \
        (M_FE_TOPRCM_FEC_PD_STATUS_PWR_DOWN != \
        (w_regValue & M_FE_TOPRCM_FEC_PD_OFF_STATUS_MASK)) && \
        (w_timeout < w_timeOutPeriod));

    /*! -# If there is a API time out return error */
    if (w_timeout >= w_timeOutPeriod)
    {
        xw_return = M_DFP_RET_CODE_FEC_PWRDOWN_TIMEOUT;
    }

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS MemInit function
 *
 * @b Description @n
 * This driver function does meminit for all FECSS chirp ram and Data RAM memory.
 * 16kB DATA RAM and 8kB Chirp RAM..
 *
 * @b Assumption: This function shall be called after the FECSS power up.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-422, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssMemInit(UINT8 c_devIndex, const T_FE_DRIVER_DATA* p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FECCTRL_REGS *p_structFecCtrl = M_FE_FECSS_CTRL_REG_START_ADDRESS;
    REG32 *p_regFecDataMemSel = (REG32*)(&p_structFecCtrl->r_FecssMemInitSliceSel.b32_Reg);
    REG32 *p_regFecDataMemInit = (REG32*)(&p_structFecCtrl->r_FecssRamMemInit.b32_Reg);
    REG32 *p_regFecChirpMemInit = (REG32*)(&p_structFecCtrl->r_FecssTimingEngineMemInit.b32_Reg);
    REG32 *p_regFecDataMemInitDone = (REG32*)(&p_structFecCtrl->r_FecssRamMemDone.b32_Reg);
    REG32 *p_regFecChirpMemInitDone = \
                    (REG32*)(&p_structFecCtrl->r_FecssTimingEngineMemDone.b32_Reg);

    UINT32 w_regValue;
    UINT32 w_timeout = (UINT32)0U,
           w_timeOutPeriod,
           w_dataMemDone,
           w_chirpMemDone;

    /*! Clear the done status before triggering the meminit */
    M_REG32_WRITE(c_devIndex, p_regFecChirpMemInitDone, M_FE_RFS_RAM_MEM_INIT, xw_return);
    M_REG32_WRITE(c_devIndex, p_regFecDataMemInitDone, M_FE_RFS_RAM_MEM_INIT, xw_return1);
    xw_return += xw_return1;

    /*!
    * -# Perform FECSS RFS data RAM memory init in case of cold boot
    *   - ECC is enabled by default in safety device, disabled in non safety device by RBL
    *   - Select the Bank 1 (DATA RAM) only
    *   - Mem init Bank 1 (DATA RAM)
    *   - Mem init Chirp RAM in parallel
    */
    /*! -# FEC Bank1 Data RAM and Chirp RAM meminit
    *    - Trigger meminit
    *    - Wait for done status
    *    - Read memory status
    */
    w_regValue = M_FE_RFS_RAM_MEM_INIT_BANK1_SEL;
    M_REG32_WRITE(c_devIndex, p_regFecDataMemSel, w_regValue, xw_return1);

    w_regValue = M_FE_RFS_RAM_MEM_INIT;
    M_REG32_WRITE(c_devIndex, p_regFecDataMemInit, w_regValue, xw_return1);
    xw_return += xw_return1;
    M_REG32_WRITE(c_devIndex, p_regFecChirpMemInit, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*!
    * -# Wait for meminit done
    *    - Poll for update til API timeout
    *    - Use 2us delay in loop
    */
    w_timeOutPeriod = \
        (UINT32)p_feDrvData->z_ClientCbData.h_ApiRespTimeoutMs * (UINT32)1000U;
    do
    {
        /*! -# 2us delay */
        xw_return += p_feDrvData->z_ClientCbData.z_PltfCb.p_Delay(20U);
        w_timeout += (UINT32)2U;
        /*! -# Read meminit done status for chirp and Data Ram */
        M_REG32_READ(c_devIndex, p_regFecChirpMemInitDone, w_chirpMemDone, xw_return1);
        xw_return += xw_return1;
        w_regValue = w_chirpMemDone;
        M_REG32_READ(c_devIndex, p_regFecDataMemInitDone, w_dataMemDone, xw_return1);
        w_regValue &= w_dataMemDone;
        xw_return += xw_return1;
    } while (((UINT8)0U == (UINT8)(w_regValue & M_FE_RFS_RAM_MEM_INIT)) && \
            (w_timeout < w_timeOutPeriod) && (M_DFP_RET_CODE_OK == xw_return));

    /*! -# If there is a API time out return error */
    if (w_timeout >= w_timeOutPeriod)
    {
        xw_return = M_DFP_RET_CODE_MEM_INIT_TIMEOUT;
    }

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Clock switch function
 *
 * @b Description @n
 * This driver function does clock switch for FECSS module based on input selection
 * XTAL or Fast clock source.
 *
 * @b Assumption: This function shall be called after the FECSS power up.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] c_clkSrcSel   - FECSS clock source select
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-423, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssClockSwitch(UINT8 c_devIndex, UINT8 c_clkSrcSel)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32 w_divValue,
           w_srcValue;
    T_FECRCM_REGS *p_structFecRcm = M_FE_FECSS_RCM_REG_START_ADDRESS;
    REG32 *p_regFecRcm;
    UINT32 w_regCmpSts = (UINT32)0U;

    /*!
    * -# Set FECSS clock as per c_clkSrcSel
    *    - Fast clock DIG_PLL or APLL (160MHz) / DIV (2) = 80MHz
    *    - XTAL clock is 40MHz (typical) / DIV 1 = 40MHz
    */
    p_regFecRcm = (REG32 *)(&p_structFecRcm->r_FecSysClkctl.b32_Reg);
    if (M_RL_FECSS_DEV_CLK_SRC_FCLK == c_clkSrcSel)
    {
        /*!
        * -# Set Fast clock 80MHz (with DIV by 2)
        *    - Set divider setting = 1 (DIV by 2)
        *    - Set clock source = 3
        */
        w_divValue = M_FE_FECSS_CLK_SRC_FAST_DIV_VALUE;
        w_srcValue = M_FE_FECSS_CLK_SRC_FAST_SEL_VALUE;
        /*! -# Configure the divider first - switching from low to high */
        M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regFecRcm, (UINT16)w_divValue, \
            U_FECRCM_FEC_SYS_CLKCTL, b12_Divr, xw_return);

        /*! -# Configure the source value  */
        M_REG_STRUCT_FIELD_SWRITE(c_devIndex, p_regFecRcm, (UINT16)w_srcValue, \
            U_FECRCM_FEC_SYS_CLKCTL, b12_Srcsel, w_regCmpSts, xw_return1);
        xw_return += xw_return1;
    }
    else
    {
        /*!
        * -# Set XTAL clock 40MHz
        *    - Set divider setting = 0 (DIV by 1)
        *    - Set clock source = 0
        */
        w_divValue = M_FE_FECSS_CLK_SRC_XTAL_DIV_VALUE;
        w_srcValue = M_FE_FECSS_CLK_SRC_XTAL_SEL_VALUE;
        /*! -# Configure the source value first - switching from high to low */
        M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regFecRcm, (UINT16)w_srcValue, \
            U_FECRCM_FEC_SYS_CLKCTL, b12_Srcsel, xw_return);

        /*! -# Configure the divider value */
        M_REG_STRUCT_FIELD_SWRITE(c_devIndex, p_regFecRcm, (UINT16)w_divValue, \
            U_FECRCM_FEC_SYS_CLKCTL, b12_Divr, w_regCmpSts, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Perform register readback error check */
    M_DFP_ASSERT(((UINT32)0U == w_regCmpSts), xw_return1, M_DFP_RET_CODE_REG_READBACK_ERROR);
    xw_return += xw_return1;

    return xw_return;
}


/*!*****************************************************************************
 * @brief FECSS Clock Gate (enter sleep) function
 *
 * @b Description @n
 * This driver function performs FECSS clock gate (entering sleep) or ungate based on
 * input.
 *
 * @b Assumption: This function shall be called after the FECSS power up.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] c_clkSrcSel   - FECSS clock source select
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-424, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssClockGate(UINT8 c_devIndex, UINT8 c_clkSrcSel)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    REG32 *p_regFecClkGate = M_FE_APPSS_FEC_CLK_GATE_REG_ADDRESS;
    UINT32 w_regValue,
           w_regCmpSts = (UINT32)0U;

    /*! -# Read FECSS clock config in APPSS control  */
    M_REG32_READ(c_devIndex, p_regFecClkGate, w_regValue, xw_return);

    /*! -# Gate FECSS clock in APPSS control if clock source is OFF */
    if (M_RL_FECSS_DEV_CLK_SRC_OFF == c_clkSrcSel)
    {
        w_regValue |= (M_FE_APPSS_FEC_SYS_CLK_GATE_VALUE);
    }
    else
    {
        w_regValue &= ~(M_FE_APPSS_FEC_SYS_CLK_GATE_VALUE);
    }
    M_REG32_SWRITE(c_devIndex, p_regFecClkGate, w_regValue, w_regCmpSts, xw_return1);
    xw_return += xw_return1;

    /*! -# Perform register readback error check */
    M_DFP_ASSERT(((UINT32)0U == w_regCmpSts), xw_return1, M_DFP_RET_CODE_REG_READBACK_ERROR);
    xw_return += xw_return1;

    return xw_return;
}


/*!*****************************************************************************
 * @brief FECSS FT/Frame Timer Clock Gate function
 *
 * @b Description @n
 * This driver function performs FECSS FRC module clock gate or ungate based on
 * input.
 *
 * @b Assumption: This function shall be called after the FECSS power up.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] c_clkSrcSel   - FECSS FT clock source select
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-425, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssFtClockGate(UINT8 c_devIndex, UINT8 c_clkSrcSel)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    REG32 *p_regFtClkGate = M_FE_TOPRCM_FEC_FT_CLK_GATE_REG_ADDRESS;
    UINT32 w_regValue = (UINT32)0U,
           w_regCmpSts = (UINT32)0U;

    if (M_RL_FECSS_FT_CLK_SRC_XTAL == c_clkSrcSel)
    {
        /*! -# Un Gate the clock  */
        w_regValue &= ~(M_FE_TOPRCM_FEC_FT_CLK_GATE_VALUE);
    }
    else
    {
        /*! -# Gate the clock  */
        w_regValue |= M_FE_TOPRCM_FEC_FT_CLK_GATE_VALUE;
    }
    M_REG32_SWRITE(c_devIndex, p_regFtClkGate, w_regValue, w_regCmpSts, xw_return);

    /*! -# Perform register readback error check */
    M_DFP_ASSERT(((UINT32)0U == w_regCmpSts), xw_return1, M_DFP_RET_CODE_REG_READBACK_ERROR);
    xw_return += xw_return1;

    return xw_return;
}

/*!*****************************************************************************
 * @brief Get device clock Source and , ft clock source
 *
 * @b Description @n
 * This driver function Get device clock Source and ft clock source
 *
 * @b Assumption: This function shall be called after the FECSS power up.
 *
 * @param[in] c_devIndex     - Device Index
 * @param[out] p_devClkSrc   - FECSS Dev clock source select
 * @param[out] p_ftClkSrc    - FECSS FT clock source select
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-426, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDevclkSrcFtClkStsGet(UINT8 c_devIndex, \
                    UINT8 *p_devClkSrc, UINT8 *p_ftClkSrc)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32 w_regValue;
    REG32 *p_regFtClkGate = M_FE_TOPRCM_FEC_FT_CLK_GATE_REG_ADDRESS;
    T_FECRCM_REGS *p_structFecRcm = M_FE_FECSS_RCM_REG_START_ADDRESS;
    REG32 *p_regFecRcm;

    /*!
    * -# FECSS system clock source status
    *    - Fast clock M_FE_FECSS_CLK_SRC_FAST_SEL_VALUE
    *    - XTAL clock M_FE_FECSS_CLK_SRC_XTAL_SEL_VALUE
    */
    p_regFecRcm = (REG32 *)(&p_structFecRcm->r_FecSysClkctl.b32_Reg);
    M_REG_STRUCT_FIELD_READ(c_devIndex, p_regFecRcm, w_regValue, \
            U_FECRCM_FEC_SYS_CLKCTL, b12_Srcsel, xw_return);
    if (M_FE_FECSS_CLK_SRC_FAST_SEL_VALUE == w_regValue)
    {
        *p_devClkSrc = M_RL_FECSS_DEV_CLK_SRC_FCLK;
    }
    else
    {
        *p_devClkSrc = M_RL_FECSS_DEV_CLK_SRC_XTAL;
    }

    /*!
    * -# FECSS FT clock gate status
    *    - FT clock Gate M_FE_TOPRCM_FEC_FT_CLK_GATE_VALUE
    *    - FT clock ON
    */
    M_REG32_READ(c_devIndex, p_regFtClkGate, w_regValue, xw_return1);
    xw_return += xw_return1;
    if (M_FE_TOPRCM_FEC_FT_CLK_GATE_VALUE == \
            (w_regValue & M_FE_TOPRCM_FEC_FT_CLK_GATE_VALUE))
    {
        *p_ftClkSrc   =  M_RL_FECSS_FT_CLK_SRC_OFF;
    }
    else
    {
        *p_ftClkSrc   =  M_RL_FECSS_FT_CLK_SRC_XTAL;
    }

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Die ID Get driver function
 *
 * @b Description @n
 * This driver function Gets the device Die ID.
 *
 * @b Assumption: This function shall be called after the FECSS power up.
 *
 * @param[in] c_devIndex   - Device Index
 * @param[out] p_resData   - FECSS Dev Die ID info @ref T_RL_API_SENSOR_DIEID_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-427, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDieIdDataGet(UINT8 c_devIndex, \
                            T_RL_API_SENSOR_DIEID_RSP *p_resData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    /*! -# Reading DIEID0 to DIEID3 values from M_FE_TOP_DIEID0_REG_ADDR  */
    REG32 *p_dieId = M_FE_TOP_DIEID0_REG_ADDR;
    UINT32 w_regValue;

    /*! -# Read DIEID0 and update p_resData->w_DieIdData[0] */
    M_REG32_READ(c_devIndex, p_dieId, w_regValue, xw_return);
    p_resData->w_DieIdData[0] = w_regValue;

    /*! -# Read DIEID1 and update p_resData->w_DieIdData[1] */
    p_dieId++;
    M_REG32_READ(c_devIndex, p_dieId, w_regValue, xw_return1);
    p_resData->w_DieIdData[1] = w_regValue;
    xw_return += xw_return1;

    /*! -# Read DIEID2 and update p_resData->w_DieIdData[2] */
    p_dieId++;
    M_REG32_READ(c_devIndex, p_dieId, w_regValue, xw_return1);
    p_resData->w_DieIdData[2] = w_regValue;
    xw_return += xw_return1;

    /*! -# Read DIEID3 and update p_resData->w_DieIdData[3] */
    p_dieId++;
    M_REG32_READ(c_devIndex, p_dieId, w_regValue, xw_return1);
    p_resData->w_DieIdData[3] = w_regValue;
    xw_return += xw_return1;

    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF fe_driver.c
 */
