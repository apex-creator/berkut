/*!*****************************************************************************
 * @file pfe_device.c
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
 * @addtogroup FECSSLIB_DEVICE_PATCH Device Interface and Driver API Patch functions
 *  @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     27Oct2023   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <fecsslib/fecsslib.h>
#include <fecsslib/common/fe_registers.h>
#include <fecsslib/rfscripter/include/rfs_driver.h>
#include <fecsslib/sensor/include/fe_sensdriver.h>


/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */


/*!*****************************************************************************
 * @brief FECSS device status GET
 *
 * @b Description @n
 * This FECSS API driver function Get the FECSS Device Status
 *
 * @b Assumption: The FECSS Init is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_rspData   - pointer to response data @ref T_RL_API_FECSS_DEV_STS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-403
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDevStatusGet_patch(UINT8 c_devIndex, \
                            T_RL_API_FECSS_DEV_STS_RSP *p_rspData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return, xw_return1;
    T_FE_RFS_BOOT_STS   z_rfsBootSts = {0};
    T_FE_RFS_BOOT_INFO  z_rfsBootInfo = {0};
    T_RL_API_RFS_FAULT_STS_GET_RSP z_rfsCpuFaultSts;

    (void)fe_hwMemclr((UINT32*)&z_rfsCpuFaultSts, (UINT16)(sizeof(T_RL_API_RFS_FAULT_STS_GET_RSP)/sizeof(UINT32)));

    /*! -# Get RFS CPU status  */
    xw_return = rfs_cmdCpuStsGet(c_devIndex, &z_rfsCpuFaultSts);
    p_rspData->h_RfsFwState     = z_rfsCpuFaultSts.h_RfsFwState;
    p_rspData->c_PatchAndSilEna = z_rfsCpuFaultSts.c_PatchAndSilEna;
    p_rspData->c_RfAndSensType  = z_rfsCpuFaultSts.c_RfAndSensType;

    /*! -# Get RFS Boot self test status  */
    xw_return1 = rfs_rfsBootStatusGet(c_devIndex, &z_rfsBootSts);
    xw_return += xw_return1;
    p_rspData->c_RfsBootSelfTest = z_rfsBootSts.c_RfsBootSelfTest;

    /*! -# Get RFS Boot info status
     *     - Xtal Freq
     *     - Power Mode
     */
    xw_return1 = rfs_rfsBootInfoGet(c_devIndex, &z_rfsBootInfo);
    xw_return += xw_return1;
    p_rspData->h_XtalClkFreq = z_rfsBootInfo.h_ClkFreq;
    p_rspData->c_PowerModeState =  \
            (z_rfsBootInfo.c_PwrModeAndXtalType & (UINT8)0x0FU);

    /*! -# Get CT resolution */
    xw_return1 = sens_getCtResolution(c_devIndex, &p_rspData->c_ChirpTimerResType);
    xw_return += xw_return1;

    /*! Get Clock Status */
    xw_return1 = fe_fecssDevclkSrcFtClkStsGet(c_devIndex, \
                    &p_rspData->c_DevClkSrcState, \
                    &p_rspData->c_FtClkState);
    xw_return += xw_return1;

    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF pfe_device.c
 */
