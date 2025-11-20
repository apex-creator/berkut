/*!*****************************************************************************
 * @file fe_device.h
 *
 * @brief FECSSLib Device Module header file.
 *
 * @b Description @n
 * This FECSS library Device Module header file defines FECSS Device control APIs,
 * data structure, MACROs.
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
 * 0.1     14Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef FE_DEVICE_H
#define FE_DEVICE_H

/*!*****************************************************************************
 \brief FECSSLib Library Device Module Driver API details.

 \section fecsslib_Device_sec FECSSLib Device Module
FECSSLib Device Module is responsible for providing driver APIs for FECSS device control, Lib interface and Register Interface. Below block diagram shows the individual functional blocks of the Device Module.

@image html fecsslib_device.png "FECSSLib Device Module"

 \subsection fecsslib_device_api_subsec FECSSLib Device driver API functions and Data Structures

The following FECSSLib device APIs provides FECSS Device control for application, refer function definition for details on data structure.
  - \ref fe_fecssLibInit             - ROM/PATCH function
  - \ref fe_fecssLibDeInit           - ROM/PATCH function
  - \ref fe_fecssDevPwrOn            - ROM/PATCH function
  - \ref fe_fecssDevPwrOff           - ROM/PATCH function
  - \ref fe_fecssDevClockCtrl        - ROM/PATCH function
  - \ref fe_fecssDevStatusGet        - ROM/PATCH function
  - \ref fe_fecssDieIdGet            - ROM/PATCH function
  - \ref fe_fecssRfsFaultStatusGet   - ROM/PATCH function
.

Link to Parent Directory: \ref FECSSLIB_DEVICE_API

  @addtogroup FECSSLIB_DEVICE_API FECSSLib Device Control driver API functions
  @{
 @brief Below sections provides information about FECSSLib Library Device module driver API functions and data structure and MACROs
********************************************************************************
*/

/*!
 * @defgroup FECSSLIB_DEVICE_DRV_MODULE Device Interface and Driver API functions
 * @brief FECSSLib Device Interface and driver API functions and Data Structures
 * @ingroup FECSSLIB_DEVICE_API
 */

/*!
 * @defgroup FECSSLIB_DEVICE_REGIF_MODULE Device Register Interface Driver functions
 * @brief FECSSLib Device Register Interface driver functions and Data Structures
 * @ingroup FECSSLIB_DEVICE_API
 */

/*!
 * @defgroup FECSSLIB_DEVICE_DRV_PATCH Device Interface and Driver API Patch functions
 * @brief FECSSLib Device Interface and driver API Patch functions and Data Structures
 * @ingroup FECSSLIB_DEVICE_API
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Device module MACROs
 * @{
 */

/*!
 * @name FECSS Platform specific MACROs
 * @{
 */
/*!
 * @brief ARM DSB instruction, in FE device this is not applicable
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_ARM_DSB()                     __asm("  DSB")
#else
#define M_ARM_DSB()
#endif

/*!
 * @brief ARM ISB instruction, in FE device this is not applicable
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_ARM_ISB()                     __asm("  ISB")
#else
#define M_ARM_DSB()
#endif

/*! @} */

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Device module Type defines
 * @{
 */


/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/*!
 * @name FECSS Library Device module API functions
 * @{
 */
M_LIB_EXPORT T_RETURNTYPE fe_fecssDevPwrOn_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_DEV_PWR_ON_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_fecssDevPwrOff_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_DEV_PWR_OFF_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_fecssDevClockCtrl_rom(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_fecssDevClockCtrlCheck_rom(UINT8 c_disableErrorCheck, \
                const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_fecssDevStatusGet_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_DEV_STS_RSP *p_rspData);
M_LIB_EXPORT T_RETURNTYPE fe_fecssDieIdGet_rom(UINT8 c_devIndex, \
                            T_RL_API_SENSOR_DIEID_RSP *p_resData);
M_LIB_EXPORT T_RETURNTYPE fe_fecssRfsFaultStatusGet_rom(UINT8 c_devIndex, \
                            T_RL_API_RFS_FAULT_STS_GET_RSP *p_rspData);
/*! @} */

M_LIB_EXPORT T_RETURNTYPE fe_fecssDevStatusGet_patch(UINT8 c_devIndex, \
                            T_RL_API_FECSS_DEV_STS_RSP *p_rspData);
#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF fe_device.h
 */


