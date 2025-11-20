/*!*****************************************************************************
 * @file rfs_driver.c
 *
 * @brief FECSSLib RF Frontend Scripter (RFS) control driver functions.
 *
 * @b Description @n
 * This file provides FECSS library RFS control driver functions for powerup,
 * power down RFS core, Mailbox message handling.
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
 * @addtogroup FECSSLIB_RFS_DRV_MODULE RFS control Driver API functions
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
/*!
 * @name FECSS Library RFS control Driver static function calls
 * @{
 */
static T_RETURNTYPE rfs_rfsMbInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
static T_RETURNTYPE rfs_rfsMbDeInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData);
/*! @} */

/*!*****************************************************************************
 * @brief FECSS RFS Mailbox Init function
 *
 * @b Description @n
 * This function initializes the RFS - FECSSLib Mailbox for IPC, the command-response
 * protocol is used to communicate with RFS. The after sending the command, FECSSLib
 * will wait for Response interrupt.
 *
 * @b Assumption: The response interrupt registration is done in fe_pltDrvInit() function
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData   - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-387, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
static T_RETURNTYPE rfs_rfsMbInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    REG32 *p_regCmd = M_FE_RFS_IPC_CMD_TRIG_REG_ADDRESS;
    REG32 *p_regRsp = M_FE_RFS_IPC_RES_TRIG_REG_ADDRESS;
    REG32 *p_ipcBusy = M_FE_RFS_IPC_BUSY_STS_REG_ADDRESS;
    UINT32 w_regValue;
    UINT32 w_regCmpSts = (UINT32)0U;

    /*!
     * -# Initialize the RFS MB
     *   - MB memory 0x21200000, 128 bytes Mem init
     *   - Clear all IPC status register
     *   - MB response interrupt handler registration is done in fe_pltDrvInit() function
     */
    /*! -# MB mem init to 0 , num of words = 128 bytes / 4 */
    M_MEM_WRITE(c_devIndex, M_FE_RFS_MB_START_ADDRESS, (const UINT32*)M_NULL_PTR, 32U, xw_return);

    /*! -# Clear RFS IPC CMD trigger register status */
    M_REG32_READ(c_devIndex, p_regCmd, w_regValue, xw_return1);
    xw_return += xw_return1;
    w_regValue &= ~(M_FE_RFS_MB_CMDRSP_STATUS_MASK);
    M_REG32_SWRITE(c_devIndex, p_regCmd, w_regValue, w_regCmpSts, xw_return1);
    xw_return += xw_return1;

    /*! -# Clear RFS IPC RSP trigger register status */
    M_REG32_READ(c_devIndex, p_regRsp, w_regValue, xw_return1);
    xw_return += xw_return1;
    w_regValue &= ~(M_FE_RFS_MB_CMDRSP_STATUS_MASK);
    M_REG32_SWRITE(c_devIndex, p_regRsp, w_regValue, w_regCmpSts, xw_return1);
    xw_return += xw_return1;

    /*! -# Clear RFS IPC BUSY status register */
    w_regValue = (UINT32)0x1U;
    M_REG32_WRITE(c_devIndex, p_ipcBusy, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Perform register readback error check */
    M_DFP_ASSERT(((UINT32)0U == w_regCmpSts), xw_return1, M_DFP_RET_CODE_REG_READBACK_ERROR);
    xw_return += xw_return1;

    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Update MB init status pass */
        p_feDrvData->z_RfsData[c_devIndex].c_MbStatus = M_FE_RFS_MB_STS_PASS;
    }
    else
    {
        /*! -# Update MB init status uninit */
        p_feDrvData->z_RfsData[c_devIndex].c_MbStatus = M_FE_RFS_MB_STS_IPC_FAIL;
    }

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Mailbox DeInit function
 *
 * @b Description @n
 * This function Un-initializes the RFS - FECSSLib Mailbox.
 *
 * @b Assumption: The response interrupt registration is done in fe_pltDrvInit() function
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
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-388, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
static T_RETURNTYPE rfs_rfsMbDeInit(UINT8 c_devIndex, T_FE_DRIVER_DATA *p_feDrvData)
{
    T_RETURNTYPE xw_return = M_DFP_RET_CODE_OK;

    /*!
     * -# UnInitialize the RFS MB
     *   - Update MB init status
     */

    /*! -# Update MB init status */
    p_feDrvData->z_RfsData[c_devIndex].c_MbStatus = M_FE_RFS_MB_STS_UNINIT;

    /*! -# Return xw_return value */
    return xw_return;
}

/******************************FECSSLib RFS Driver APIs****************************/
/*!*****************************************************************************
 * @brief FECSS RFS Open function
 *
 * @b Description @n
 * This function does RFS M3 core power up. The boot info seetings are passed to
 * RFS core through dedicated shared ram location, once boot is complete boot status
 * is read from dedicated shared ram location and returned to mmWaveLink API layer.
 *
 * @b Assumption: The FECSS poweredup and clock un gated before calling this function
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] z_rfsBootInfo - RFS boot info data @ref T_FE_RFS_BOOT_INFO
 * @param[out] p_rfsBootStatus - pointer to RFS boot status data @ref T_FE_RFS_BOOT_STS
 * @param[in, out] p_feDrvData   - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-389, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsOpen(UINT8 c_devIndex, const T_FE_RFS_BOOT_INFO z_rfsBootInfo, \
        T_FE_RFS_BOOT_STS* p_rfsBootStatus, T_FE_DRIVER_DATA* p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_bootInfoPtr = M_FE_RFS_BOOT_INFO_START_ADDRESS;
    UINT32* p_bootStsPtr  = M_FE_RFS_BOOT_STS_START_ADDRESS;
    UINT32* p_cpuStsPtr   = M_FE_RFS_CPU_STS_START_ADDRESS;
    UINT16 h_numOfWords;
    UINT32 w_timeout = (UINT32)0U,
           w_timeOutPeriod;
    UINT32 w_ts1,
           w_ts2,
           w_tsDiff,
           w_regValue;
    UINT32 w_regCmpSts = (UINT32)0U;
    REG32 *p_regFecPdRst = M_FE_TOPRCM_FEC_PD_CM3_RESET_REG_ADDRESS;
    T_FECCTRL_REGS *p_structFecCtrl = M_FE_FECSS_CTRL_REG_START_ADDRESS;
    REG32 *p_regFecCtrl;

    /*! -# Init Mailbox */
    xw_return = rfs_rfsMbInit(c_devIndex, p_feDrvData);

    /*! -# Copy Boot info if MB init is successful */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Copy Bootinfo data to RFS - FECSSLib shared Memory location
         * M_FE_RFS_BOOT_INFO_START_ADDRESS
         */
        h_numOfWords = (UINT16)(sizeof(T_FE_RFS_BOOT_INFO)) / (UINT16)4U;
        M_MEM_WRITE(c_devIndex, p_bootInfoPtr, &z_rfsBootInfo, h_numOfWords, xw_return);

        /*!
         * -# Clear the RFS CPU State to 0 (halt) before unhalt
         * -# Read 1st 32 bit value in T_RL_API_RFS_CPU_STS_GET_RSP struct from RFS, the 1st 2byte
         *  is h_FwState
         */
        M_REG32_READ(c_devIndex, p_cpuStsPtr, w_regValue, xw_return1);
        xw_return += xw_return1;
        /*! -# Clear 1st 2 bytes - halt state */
        w_regValue &= (UINT32)0xFFFF0000U;
        /*! -# Write new h_FwState to RFS */
        M_REG32_WRITE(c_devIndex, p_cpuStsPtr, w_regValue, xw_return1);
        xw_return += xw_return1;

        /*!
         * -# Clear the RFS Boot State to 0 (Uninit) before unhalt
         * -# Read 1st 32 bit value in T_FE_RFS_BOOT_STS struct from RFS, the 1st 1byte
         *  is c_BootStatus
         */
        M_REG32_READ(c_devIndex, p_bootStsPtr, w_regValue, xw_return1);
        xw_return += xw_return1;
        /*! -# Clear 1st byte - Uninit */
        w_regValue &= (UINT32)0xFFFFFF00U;
        /*! -# Write new h_FwState to RFS */
        M_REG32_WRITE(c_devIndex, p_bootStsPtr, w_regValue, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Unhalt RFS M3 core if boot info copy is successful */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
         * -# RFS M3 core shared ram clock should not be gated in ES1.0
         *    - by default FEC clock is enabled to shared ram in reset
         *    - this can be disabled by application in ES2.0
         * -# RFS M3 core pre unhalt settings
         *    - Disabled sleep hold feature by default in reset
         */

        /*!
         * -# RFS M3 core FW unhalt setting
         *    - Unhalt the CM3 FW by writing value 0xF5A36A17 to CM3_CPU_HALT_HANDSHAKE register
         */
        w_regValue = M_FE_RFS_CM3_FW_UNHALT_VAL;
        p_regFecCtrl = &p_structFecCtrl->r_Cm3CpuHaltHandshake.b32_Reg;
        M_REG32_SWRITE(c_devIndex, p_regFecCtrl, w_regValue, w_regCmpSts, xw_return);

        /*! -# unhalt RFS M3 core - use TOPRCM register, this takes ~120us */
        w_regValue = M_FE_TOPRCM_FEC_PD_CM3_UNHALT_VAL;
        M_REG32_SWRITE(c_devIndex, p_regFecPdRst, w_regValue, w_regCmpSts, xw_return1);
        xw_return += xw_return1;

        /*! -# Perform register readback error check */
        M_DFP_ASSERT(((UINT32)0U == w_regCmpSts), xw_return1, M_DFP_RET_CODE_REG_READBACK_ERROR);
        xw_return += xw_return1;
    }

    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Profile RFS unhalt time duration - ts1 */
        w_ts1 = p_feDrvData->z_ClientCbData.z_PltfCb.p_TimeStamp();

        /*!
         * -# Wait for M3 boot status update
         *    - Poll for update til API timeout
         *    - Use 1us delay in loop
         */
        w_timeOutPeriod = (UINT32)p_feDrvData->z_ClientCbData.h_ApiRespTimeoutMs * (UINT32)1000U;
        do
        {
            /*! -# 10us delay */
            xw_return = p_feDrvData->z_ClientCbData.z_PltfCb.p_Delay(100U);
            w_timeout += (UINT32)10U;
            /*! -# 1st byte in T_FE_RFS_BOOT_STS is c_BootStatus */
            M_REG32_READ(c_devIndex, p_bootStsPtr, w_regValue, xw_return1);
            xw_return += xw_return1;
        } while ((M_FE_RFS_BOOT_STS_UNINIT == (UINT8)(w_regValue & (UINT32)0x000000FFU)) && \
                (w_timeout < w_timeOutPeriod) && (M_DFP_RET_CODE_OK == xw_return));

        /*! -# Profile RFS unhalt time duration - ts2 */
        w_ts2 = p_feDrvData->z_ClientCbData.z_PltfCb.p_TimeStamp();

        /*! -# Update RFS boot time */
        w_tsDiff = w_ts1 - w_ts2;
        p_feDrvData->z_RfsData[c_devIndex].w_BootTime = (w_tsDiff & \
                p_feDrvData->z_ClientCbData.z_PltfCb.w_TimeStampCounterMask);

        /*! If there is a time out record error M_DFP_RET_CODE_RFS_BOOT_TIMEOUT */
        if (w_timeout >= w_timeOutPeriod)
        {
            xw_return = M_DFP_RET_CODE_RFS_BOOT_TIMEOUT;
        }
    }

    /*! -# Copy Boot status data to p_rfsBootStatus address */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
         * -# Copy Boot status from shared ram M_FE_RFS_BOOT_STS_START_ADDRESS to p_rfsBootStatus
         */
        h_numOfWords = (UINT16)(sizeof(T_FE_RFS_BOOT_STS)) / (UINT16)4U;
        M_MEM_READ(c_devIndex, p_rfsBootStatus, p_bootStsPtr, h_numOfWords, xw_return);
    }

    /*! -# Copy Boot status data to p_rfsBootStatus address */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Update RFS boot status */
        p_feDrvData->z_RfsData[c_devIndex].c_BootStatus = p_rfsBootStatus->c_BootStatus;

        /*! -# If boot status is not passed return failure */
        if (M_FE_RFS_BOOT_STS_PASS != p_rfsBootStatus->c_BootStatus)
        {
            xw_return = M_DFP_RET_CODE_RFS_BOOT_ERROR;
        }
    }
    else
    {
        /*! -# Update RFS boot status as M_FE_RFS_BOOT_STS_FAIL */
        p_feDrvData->z_RfsData[c_devIndex].c_BootStatus = M_FE_RFS_BOOT_STS_FAIL;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_rfsOpen:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Close function
 *
 * @b Description @n
 * This function does RFS M3 core power down and brought into reset state.
 *
 * @b Assumption: This function is called while entring deep sleep only.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_feDrvData   - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-390, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsClose(UINT8 c_devIndex, T_FE_DRIVER_DATA* p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    REG32 *p_regFecPdRst = M_FE_TOPRCM_FEC_PD_CM3_RESET_REG_ADDRESS;
    UINT32 w_regValue;

    /*! -# Deinit RFS MB */
    xw_return = rfs_rfsMbDeInit(c_devIndex, p_feDrvData);

    /*!
     * -# Set FECSS CM3 PD reset state in manual mode
     *    - Reset M3 core when entering deep sleep
     *    - Keep M3 core in reset/halt state during FECSS power down (default sate in powerup)
     */
    w_regValue = M_FE_TOPRCM_FEC_PD_CM3_RESET_VAL;
    M_REG32_WRITE(c_devIndex, p_regFecPdRst, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Update RFS init status */
    p_feDrvData->z_RfsData[c_devIndex].c_BootStatus = M_FE_RFS_BOOT_STS_UNINIT;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_rfsClose:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Command Set function
 *
 * @b Description @n
 * This function sends command to RFS M3 core, the packet formation and command trigger
 * is done by this function.
 *
 * @b Assumption: The command data size is always multiple for 4.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in] z_rfsMbData   - RFS MB command data structure @ref T_FE_RFS_CMD_RES_DATA
 * @param[in, out] p_feDrvData  - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-391, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsCmdSet(UINT8 c_devIndex, T_FE_RFS_CMD_RES_DATA z_rfsMbData, \
                    T_FE_DRIVER_DATA* p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32 w_cmdIdSize;
    UINT32* p_mbAddress = M_FE_RFS_MB_START_ADDRESS;
    UINT16 h_numOfWords;
    REG32 *p_regCmd = M_FE_RFS_IPC_CMD_TRIG_REG_ADDRESS;
    REG32 *p_ipcBusy = M_FE_RFS_IPC_BUSY_STS_REG_ADDRESS;
    UINT32 w_regValue;
    REG32 *p_regRsp = M_FE_RFS_IPC_RES_TRIG_REG_ADDRESS;

    /*! -# Read MB busy flag and return M_DFP_RET_CODE_RFS_MB_BUSY_ERROR if MB is already busy */
    M_REG32_READ(c_devIndex, p_ipcBusy, w_regValue, xw_return);
    if ((w_regValue & (UINT32)0x1U) != (UINT32)0U)
    {
        xw_return = M_DFP_RET_CODE_RFS_MB_BUSY_ERROR;
    }
    /*! -# Check if size of command API is > M_FE_RFS_MB_SIZE */
    else if (z_rfsMbData.h_CmdResDataSize > M_FE_RFS_MB_SIZE)
    {
        xw_return = M_DFP_RET_CODE_RFS_API_SIZE_ERR;
    }
    else
    {
        /*! -# No error */
    }

    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Clear c_RfsRespSemaSigFail flag */
        p_feDrvData->z_RfsData[c_devIndex].c_RfsRespSemaSigFail = 0U;

        /*! -# pack Commnad ID and size as 32bit value */
        w_cmdIdSize = ((UINT32)z_rfsMbData.h_CommandId) | \
                            ((UINT32)z_rfsMbData.h_CmdResDataSize << (UINT32)16U);

        /*! -# Write Commnad ID and size to MB */
        M_MEM32_WRITE(c_devIndex, p_mbAddress, w_cmdIdSize, xw_return);

        /*! -# Write Command data to MB
         *  -# IF size is zero indicates there is no command data for command
         */
        if ((UINT16)0U != z_rfsMbData.h_CmdResDataSize)
        {
            p_mbAddress++;
            h_numOfWords = z_rfsMbData.h_CmdResDataSize / (UINT16)4U;
        M_MEM_WRITE(c_devIndex, p_mbAddress, z_rfsMbData.p_CmdResDataPtr, \
                h_numOfWords, xw_return1);
            xw_return += xw_return1;
        }

        /*! -# Clear RFS response status before triggerring new command  */
        M_REG32_READ(c_devIndex, p_regRsp, w_regValue, xw_return1);
        xw_return += xw_return1;
        w_regValue &= ~(M_FE_RFS_MB_CMDRSP_STATUS_MASK);
        M_REG32_WRITE(c_devIndex, p_regRsp, w_regValue, xw_return1);
        xw_return += xw_return1;

        /*! -# Trigger command to RFS  */
        M_REG32_READ(c_devIndex, p_regCmd, w_regValue, xw_return1);
        xw_return += xw_return1;
        w_regValue &= ~(M_FE_RFS_MB_CMDRSP_STATUS_MASK);
        /*! -# Update CMD status and trigger value M_FE_RFS_MB_WRITE_CMDRSP_TRIG */
        w_regValue |= M_FE_RFS_MB_WRITE_CMDRSP_TRIG;
        M_REG32_WRITE(c_devIndex, p_regCmd, w_regValue, xw_return1);
        xw_return += xw_return1;

        /*! -# Update z_RfsData.h_MbLastCmd */
        p_feDrvData->z_RfsData[c_devIndex].h_MbLastCmd = z_rfsMbData.h_CommandId;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_rfsCmdSet:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Response Get function
 *
 * @b Description @n
 * This function reads response from RFS for a command, this function extracts the packet
 * and copies response data to SW buffer.
 *
 * @b Assumption: The Response data size is always multiple for 4.
 *
 * @param[in] c_devIndex    - Device Index
 * @param[in, out] p_rfsMbData  - Pointer to RFS MB response data structure
 *                                @ref T_FE_RFS_CMD_RES_DATA
 * @param[in, out] p_feDrvData  - pointer to FECSS driver handler @ref T_FE_DRIVER_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-392, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsResGet(UINT8 c_devIndex, T_FE_RFS_CMD_RES_DATA* p_rfsMbData, \
                T_FE_DRIVER_DATA* p_feDrvData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32 w_cmdIdSize;
    UINT32* p_mbAddress = M_FE_RFS_MB_START_ADDRESS;
    UINT32* p_dbgAddress = M_FE_RFS_DBG_START_ADDRESS;
    UINT16 h_numOfWords;
    /*! -# Pointer to response debug data structure */
    UINT32 w_dbgDataAddress = (UINT32)(p_feDrvData->z_ClientCbData.z_DbgCb.p_RfsdbgData);
    REG32 *p_regCmd = M_FE_RFS_IPC_CMD_TRIG_REG_ADDRESS;
    REG32 *p_ipcBusy = M_FE_RFS_IPC_BUSY_STS_REG_ADDRESS;
    UINT32 w_regValue;
#if (0U == M_DFP_DISABLE_LOGGING)
    UINT8 c_ind;
#endif

    /*! -# Read MB response ready flag and return M_DFP_RET_CODE_RFS_MB_RESP_ERROR
     *  if MB is empty (not busy)
     */
    M_REG32_READ(c_devIndex, p_ipcBusy, w_regValue, xw_return);
    if ((w_regValue & (UINT32)0x1U) != (UINT32)1U)
    {
        xw_return = M_DFP_RET_CODE_RFS_MB_RESP_ERROR;
    }

    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Read Commnad ID and size from MB */
        M_MEM32_READ(c_devIndex, p_mbAddress, w_cmdIdSize, xw_return);

        /*! -# Extract Commnad ID and size */
        p_rfsMbData->h_CommandId = (UINT16)(w_cmdIdSize & (UINT32)0x0000FFFFU);
        p_rfsMbData->h_CmdResDataSize = (UINT16)((w_cmdIdSize >> (UINT32)16U) & (UINT32)0x0000FFFFU);

        /*! -# Check if size of command API is > M_FE_RFS_MB_SIZE */
        if (p_rfsMbData->h_CmdResDataSize > M_FE_RFS_MB_SIZE)
        {
            xw_return += M_DFP_RET_CODE_RFS_API_SIZE_ERR;
        }

        /*! -# Read Response data from MB
         *  -# IF size is zero indicates there is no response data for command
         */
        if ((UINT16)0U != p_rfsMbData->h_CmdResDataSize)
        {
            p_mbAddress++;
            h_numOfWords = p_rfsMbData->h_CmdResDataSize / (UINT16)4U;
            M_MEM_READ(c_devIndex, p_rfsMbData->p_CmdResDataPtr, p_mbAddress, \
                        h_numOfWords, xw_return1);
            xw_return += xw_return1;
        }

        /*! -# Update z_RfsData.h_MbLastRes  */
        p_feDrvData->z_RfsData[c_devIndex].h_MbLastRes = p_rfsMbData->h_CommandId;
    }

    /*! -# Read debug data after successful response read */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Read size of debug data from MB */
        M_MEM32_READ(c_devIndex, p_dbgAddress, w_cmdIdSize, xw_return);

        /*! -# Extract Size in Debug data */
        h_numOfWords = (UINT16)((w_cmdIdSize) & (UINT32)0x000000FFU);
        /*! -# Check if size of debug data is > M_FE_RFS_DBG_DATA_SIZE - 1 */
        if (h_numOfWords > (UINT16)(M_FE_RFS_DBG_DATA_SIZE - (UINT16)1U))
        {
            xw_return += M_DFP_RET_CODE_RFS_API_SIZE_ERR;
        }

        /*! -# Compute the RFS debug data buffer offset for c_devIndex */
        w_dbgDataAddress += ((UINT32)c_devIndex * (UINT32)M_FE_RFS_DBG_DATA_SIZE);

        /*! -# Read Debug data from MB including header */
        h_numOfWords = h_numOfWords / (UINT16)4U;
        M_MEM_READ(c_devIndex, ((UINT32*)w_dbgDataAddress), p_dbgAddress, \
            (h_numOfWords + (UINT16)1U), xw_return1);
        xw_return += xw_return1;

        /*! -# Clear RFS IPC command status register  */
        M_REG32_READ(c_devIndex, p_regCmd, w_regValue, xw_return1);
        xw_return += xw_return1;
        w_regValue &= ~(M_FE_RFS_MB_CMDRSP_STATUS_MASK);
        /*! -# Update CMD status and trigger value M_FE_RFS_MB_WRITE_CMDRSP_TRIG */
        w_regValue |= M_FE_RFS_MB_READ_CMDRSP_RECV;
        M_REG32_WRITE(c_devIndex, p_regCmd, w_regValue, xw_return1);
        xw_return += xw_return1;
    }

    /*! -# Clear RFS IPC BUSY status register */
    w_regValue = (UINT32)0x1U;
    M_REG32_WRITE(c_devIndex, p_ipcBusy, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_rfsResGet:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS clock configuration function
 *
 * @b Description @n
 * This function updates the RFS M3 clock frequency, RFS may use this info to
 * compute the delay function.
 *
 * @b Assumption: The RFS is in idle state, when clock switch is done.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] h_clkFreq   - RFS clock Frequency
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-393, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsClkCfg(UINT8 c_devIndex, UINT16 h_clkFreq)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32* p_bootInfoPtr = M_FE_RFS_BOOT_INFO_START_ADDRESS;
    UINT32 w_regValue;

    /*!
     * -# Read 1st 32 bit value in T_FE_RFS_BOOT_INFO struct from RFS, the 1st 2byte
     *  is h_ClkFreq
     */
    M_REG32_READ(c_devIndex, p_bootInfoPtr, w_regValue, xw_return);
    /*! -# Clear 1st 2 bytes */
    w_regValue &= (UINT32)0xFFFF0000U;
    /*! -# Append new h_clkFreq to 1st 2 bytes */
    w_regValue |= (UINT32)h_clkFreq;
    /*! -# Write new h_clkFreq to RFS */
    M_REG32_WRITE(c_devIndex, p_bootInfoPtr, w_regValue, xw_return1)
    xw_return += xw_return1;

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_rfsClkCfg:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS Mailbox response interrupt handler for FE device 0
 *
 * @b Description @n
 * This interrupt handler is invoked by device 0 RFS when reponse is ready for a command.
 *
 * @b Assumption: The command is triggered to RFS M3 core by M4/Host. In case of FE device type
 *                a GPIO is used to generate response interrupt to Host.
 *
 *
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-394, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
void rfs_cmdRespDev0IntHandler(void)
{
    /*! -# Get FE driver handle */
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    T_DFP_OSI_SEM_HDL p_semHdl;
    T_RETURNTYPE xw_return;
    UINT8 c_devIndex = M_DFP_DEVICE_INDEX_0;

    /*!
        * -# Signal semaphore once RFS response is received
        *   - Get Semaphore Handle for c_devIndex
        *   - Signal semaphore
        */
    p_semHdl  = p_feDrvData->z_DevData[c_devIndex].p_SemHdl;
    xw_return = p_feDrvData->z_ClientCbData.z_OsiCb.z_Semp.p_OsiSemSignal(p_semHdl);
    /*! if return failure record the failure */
    if (M_DFP_RET_CODE_OK != xw_return)
    {
        p_feDrvData->z_RfsData[c_devIndex].c_RfsRespSemaSigFail = (UINT8)1U;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1("FECCLib rfs_cmdRespDev0IntHandler:%d \n", xw_return);
}

/*!*****************************************************************************
 * @brief FECSS RFS Command Response Execute
 *
 * @b Description @n
 * This function API triggers RFS command and waits for response. The command data is
 * sent to RFS through MB and response is read from MB once RFS interrupts FECSSLib upon
 * availability of response.
 *
 * @b Assumption: The RFS M3 core is un halted.
 *
 * @param[in] c_devIndex       - Device Index
 * @param[in] z_cmdResExcData  - Command-Response info @ref T_FE_RFS_CMD_RES_EXC_DATA
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-395, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_cmdRespExecute(UINT8 c_devIndex, T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FE_RFS_CMD_RES_DATA z_rfsMbCmdData;
    T_FE_RFS_CMD_RES_DATA z_rfsMbResData = {0};
    /*! -# Get FE driver handle */
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    T_FE_API_RFS_ERROR_RSP* p_rfsErrorRes;
    T_DFP_OSI_MUTEX_HDL p_mutexHdl;
    T_DFP_OSI_SEM_HDL p_semHdl;
    UINT32 w_timeOutMs;
    T_FE_API_RFS_ERROR_RSP z_rfsErrorRes = {0};

    /*! -# Lock the Mutex for c_devIndex forever */
    p_mutexHdl = p_feDrvData->z_DevData[c_devIndex].p_MutexHdl;
    xw_return = p_feDrvData->z_ClientCbData.z_OsiCb.z_Mutex.p_OsiMutexLock(p_mutexHdl, \
                M_DFP_OSI_WAIT_FOREVER);

    /*! -# If mutex lock returns error then update error code M_DFP_RET_CODE_RADAR_OSIF_ERROR */
    if (M_DFP_RET_CODE_OK != xw_return)
    {
        xw_return = M_DFP_RET_CODE_RADAR_OSIF_ERROR;
    }

    /*! -# If Mutex lock is successful then trigger command to RFS */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Update RFS MB command ID */
        z_rfsMbCmdData.h_CommandId = z_cmdResExcData.h_CommandId;
        /*! -# Update RFS MB command size */
        z_rfsMbCmdData.h_CmdResDataSize = z_cmdResExcData.h_CmdSize;
        /*! -# Update pointer to command data structure */
        z_rfsMbCmdData.p_CmdResDataPtr = (void*)z_cmdResExcData.p_CmdDataPtr;

        /*! -# Trigger command to RFS */
        xw_return = rfs_rfsCmdSet(c_devIndex, z_rfsMbCmdData, p_feDrvData);
    }

    /*! -# If Command trigger is successful wait for the response */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
         * -# Wait for RFS response using semaphore signal/wait
         *   - Get Semaphore Handle for c_devIndex
         *   - get API timeout from interface data
         *   - Pend semaphore, signal is done in rfs_cmdRespDevXIntHandler
         *      (rfs_cmdRespDev0IntHandler is the response handler for device 0 / SOC type )
         */
        p_semHdl    = p_feDrvData->z_DevData[c_devIndex].p_SemHdl;
        w_timeOutMs = (UINT32)p_feDrvData->z_ClientCbData.h_ApiRespTimeoutMs;
        xw_return = p_feDrvData->z_ClientCbData.z_OsiCb.z_Semp.p_OsiSemWait(p_semHdl, \
                        (w_timeOutMs * (UINT32)1000U));

        /*! -# If semaphore wait returns error then return response time out */
        if (M_DFP_RET_CODE_OK != xw_return)
        {
            xw_return = M_DFP_RET_CODE_RFS_RESP_TIMEOUT;
        }
        /*! if any error in semaphore signal then update error code */
        else
        {
            if ((UINT8)1U == p_feDrvData->z_RfsData[c_devIndex].c_RfsRespSemaSigFail)
            {
                xw_return = M_DFP_RET_CODE_RADAR_OSIF_ERROR;
            }
        }
    }

    /*! -# If semaphore wait is successful then read the response */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Update Response data struct address to store the response */
        z_rfsMbResData.p_CmdResDataPtr = z_cmdResExcData.p_ResDataPtr;
        /*! -# If Response is NULL to parent API then populate error response ptr */
        if (M_NULL_PTR == z_rfsMbResData.p_CmdResDataPtr)
        {
             z_rfsMbResData.p_CmdResDataPtr = &z_rfsErrorRes;
        }
        /*! -# Read Response data from RFS */
        xw_return = rfs_rfsResGet(c_devIndex, &z_rfsMbResData, p_feDrvData);
    }

    /*! -# Perform error check if successfully read response  */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# If response command ID is error response type M_FE_RFS_CMD_ID_ERROR_RESP
         *     then update the error code in xw_return value
         */
        if (M_FE_RFS_CMD_ID_ERROR_RESP == z_rfsMbResData.h_CommandId)
        {
            p_rfsErrorRes = (T_FE_API_RFS_ERROR_RSP*)z_rfsMbResData.p_CmdResDataPtr;
            xw_return = p_rfsErrorRes->xw_CmdErrorCode;
            /*! -# Print logger Debug info */
            M_DFP_LOG_INFO_ARG1(" FECCLib RFS Error Response:%d \n", xw_return);
        }
        else if ((z_rfsMbCmdData.h_CommandId != z_rfsMbResData.h_CommandId) || \
            (z_rfsMbResData.h_CmdResDataSize != z_cmdResExcData.h_ResSize))
        {
            /*! -# Command / response ID and size error  */
            xw_return = M_DFP_RET_CODE_RFS_PROTOCOL_ERROR;
            /*! -# Print logger Debug info */
            M_DFP_LOG_INFO_ARG1(" FECCLib RFS Protocol Error:%d \n", xw_return);
        }
        else
        {
            /*! -# Success  */
        }
    }

    /*! -# Release the Mutex for c_devIndex */
    xw_return1 = p_feDrvData->z_ClientCbData.z_OsiCb.z_Mutex.p_OsiMutexUnLock(p_mutexHdl);

    /*! -# If mutex lock returns error then update error code M_DFP_RET_CODE_RADAR_OSIF_ERROR */
    if (M_DFP_RET_CODE_OK != xw_return1)
    {
        xw_return = M_DFP_RET_CODE_RADAR_OSIF_ERROR;
    }

    /*! -# Print logger Debug info */
    M_DFP_LOG_INFO_ARG1(" FECCLib rfs_cmdRespExecute:%d \n", xw_return);

    /*! -# Return xw_return value */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS SensorStatus Clear
 *
 * @b Description @n
 * This function driver clears the RFS sensor status start of sensor start.
 *
 * @b Assumption: The RFS M3 core is un halted and called in sensor start.
 *
 * @param[in] c_devIndex       - Device Index
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-396, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_sensStatusClear(UINT8 c_devIndex)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FE_API_RFS_SENSOR_STATUS *p_sensStatus = M_FE_RFS_SENS_STATUS_START_ADDRESS;

    /*!
     * -# Clear all the sensor status counts in IPC
     *   - p_sensStatus->w_BurstStartCount
     *   - p_sensStatus->w_BurstEndCount
     *   - p_sensStatus->w_FrameStartCount
     *   - p_sensStatus->w_FrameEndCount
     */
    M_MEM32_WRITE(c_devIndex, &p_sensStatus->w_BurstStartCount, (UINT32)0U, xw_return);
    M_MEM32_WRITE(c_devIndex, &p_sensStatus->w_BurstEndCount, (UINT32)0U, xw_return1);
    xw_return += xw_return1;
    M_MEM32_WRITE(c_devIndex, &p_sensStatus->w_FrameStartCount, (UINT32)0U, xw_return1);
    xw_return += xw_return1;
    M_MEM32_WRITE(c_devIndex, &p_sensStatus->w_FrameEndCount, (UINT32)0U, xw_return1);
    xw_return += xw_return1;

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS boot status get
 *
 * @b Description @n
 * This function driver get rfs boot status
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex       - Device Index
 * @param[out] p_rfsBootSts    - The boot status structure @ref T_FE_RFS_BOOT_STS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-397, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsBootStatusGet(UINT8 c_devIndex, T_FE_RFS_BOOT_STS *p_rfsBootSts)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    M_MEM_READ(c_devIndex, p_rfsBootSts, M_FE_RFS_BOOT_STS_START_ADDRESS, \
                     (sizeof(T_FE_RFS_BOOT_STS) / 4U), xw_return);
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS RFS boot info get
 *
 * @b Description @n
 * This function driver get rfs boot info
 *
 * @b Assumption:
 *
 * @param[in] c_devIndex       - Device Index
 * @param[out] p_rfsBootInfo   - The boot info structure @ref T_FE_RFS_BOOT_INFO
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-398, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsBootInfoGet(UINT8 c_devIndex, T_FE_RFS_BOOT_INFO *p_rfsBootInfo)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    M_MEM_READ(c_devIndex, p_rfsBootInfo, M_FE_RFS_BOOT_INFO_START_ADDRESS, \
                     (sizeof(T_FE_RFS_BOOT_INFO) / 4U), xw_return);

    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF fe_driver.c
 */
