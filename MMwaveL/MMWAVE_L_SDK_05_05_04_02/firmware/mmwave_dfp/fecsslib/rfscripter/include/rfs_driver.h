/*!*****************************************************************************
 * @file rfs_driver.h
 *
 * @brief FECSSLib RFS driver header file.
 *
 * @b Description @n
 * This FECSS library RFS driver header file defines RFS driver data structure,
 * MACROs and functions.
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
 * @addtogroup FECSSLIB_RFS_DRV_MODULE RFS control Driver API functions
 *  @{
 *
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
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef FE_RFS_DRIVER_H
#define FE_RFS_DRIVER_H

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
 * @name FECSS Library RFS Driver module MACROs
 * @{
 */
/*!
 * @brief FECSS RFS MB start address, FEC cluster1 - Bank1 DATA (retained)
 */
#define M_FE_RFS_IPC_MEM_START_ADDRESS      \
            ((T_FE_RFS_IPC_DATA_STRUCTURE*)0x21200000U)

/*!
 * @brief FECSS RFS MB start address
 */
#define M_FE_RFS_MB_START_ADDRESS           \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->w_IpcMailbox[0]))
/*!
 * @brief FECSS RFS Debug Data start address
 */
#define M_FE_RFS_DBG_START_ADDRESS          \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->w_RfsDebugData[0]))
/*!
 * @brief FECSS RFS Boot Info address
 */
#define M_FE_RFS_BOOT_INFO_START_ADDRESS    \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsBootInfo))
/*!
 * @brief FECSS RFS Boot Status address
 */
#define M_FE_RFS_BOOT_STS_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsBootSts))
/*!
 * @brief FECSS RFS Fw version address
 */
#define M_FE_RFS_FW_VER_START_ADDRESS       \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsFwVerRsp))
/*!
 * @brief FECSS RFS CPU status address
 */
#define M_FE_RFS_CPU_STS_START_ADDRESS      \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsCpuFaultSts))
/*!
 * @brief FECSS RFS GPADC config address
 */
#define M_FE_RFS_GPADC_CFG_START_ADDRESS    \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_GpadcCfgCmd))
/*!
 * @brief FECSS RFS Temperature config address
 */
#define M_FE_RFS_TEMP_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_TempMeasCfgCmd))
/*!
 * @brief FECSS RFS sensor profile config address
 */
#define M_FE_RFS_PROF_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_ProfileCfgCmd))

/*!
 * @brief FECSS RFS Live Mon sensor start config address
 */
#define M_FE_RFS_SENS_START_CFG_START_ADDRESS     \
            (&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsLiveMonSensStartCmd))

/*!
 * @brief FECSS RFS sensor loopback config address
 */
#define M_FE_RFS_LB_CFG_START_ADDRESS       \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfLbCfgCmd))

/*!
 * @brief FECSS RFS Sensor Status address
 */
#define M_FE_RFS_SENS_STATUS_START_ADDRESS     \
            (&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsSensorStsRsp))

/*!
 * @brief FECSS RFS Factory calibration data address
 */
#define M_FE_RFS_FACT_CAL_DATA_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsCalData))

/*!
 * @brief FECSS RFS RX-TX calibration data address
 */
#define M_FE_RFS_RXTX_CAL_DATA_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsCalData.z_RxTxCalData))

/*!
 * @brief FECSS RFS Synth Monitor Cfg data address
 */
#define M_FE_RFS_SYNTH_MON_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonLiveSynthFreqCmd))

/*!
 * @brief FECSS RFS Rx Saturation Cfg data address
 */
#define M_FE_RFS_RX_SAT_MON_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonLiveRxSatCmd))

/*!
 * @brief FECSS RFS GPADC CTM Monitor Cfg data address
 */
#define M_FE_RFS_GPADC_CTM_MON_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonLiveGpadcCtmCmd))

/*!
 * @brief FECSS RFS PLL control voltage Monitor Cfg data address
 */
#define M_FE_RFS_PLL_CTRLV_MON_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonPllCtrlVoltCmd))

/*!
 * @brief FECSS RFS TXn RX LB Monitor Cfg data address
 */
#define M_FE_RFS_TXn_RX_LB_MON_CFG_START_ADDRESS(x)     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonTxNRxLbCmd[x]))

/*!
 * @brief FECSS RFS TXn Power Monitor Cfg data address
 */
#define M_FE_RFS_TXn_PWR_MON_CFG_START_ADDRESS(x)     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonTxNPwrCmd[x]))

/*!
 * @brief FECSS RFS TXn BB Monitor Cfg data address
 */
#define M_FE_RFS_TXn_BB_MON_CFG_START_ADDRESS(x)     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonTxNBbCmd[x]))

/*!
 * @brief FECSS RFS TXn DC Signal  Monitor Cfg data address
 */
#define M_FE_RFS_TXn_DCSIG_MON_CFG_START_ADDRESS(x)     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonTxNDcSigCmd[x]))

/*!
 * @brief FECSS RFS RX HPF DC Signal Monitor Cfg data address
 */
#define M_FE_RFS_RX_HPF_DCSIG_MON_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonRxHpfDcSigCmd))

/*!
 * @brief FECSS RFS PL LO CLK DC signal Monitor Cfg data address
 */
#define M_FE_RFS_PM_CLK_DCSIG_MON_CFG_START_ADDRESS     \
            ((UINT32*)&(M_FE_RFS_IPC_MEM_START_ADDRESS->z_RfsMonPmClkDcSigCmd))

/*!
 * @brief FECSSLib(M4) - RFS(M3) IPC command trigger register address
 */
#define M_FE_RFS_IPC_CMD_TRIG_REG_ADDRESS     ((REG32*)0x56060028U)

/*!
 * @brief FECSSLib(M4) - RFS(M3) IPC response trigger register address
 */
#define M_FE_RFS_IPC_RES_TRIG_REG_ADDRESS     \
                        ((REG32*)&(M_FE_FECSS_CTRL_REG_START_ADDRESS->r_FecssIpcRfs.b32_Reg))
/*!
 * @brief FECSSLib(M4) - RFS(M3) IPC Busy status register address
 */
#define M_FE_RFS_IPC_BUSY_STS_REG_ADDRESS     \
                        ((REG32*)&(M_FE_FECSS_CTRL_REG_START_ADDRESS->r_FecssIpcBusyInt0.b32_Reg))

/*!
 * @brief FECSS RFS MB Header Size
 */
#define M_FE_RFS_MB_HDR_SIZE                ((UINT16)4U)
/*!
 * @brief FECSS RFS MB Size
 */
#define M_FE_RFS_MB_SIZE                    ((UINT16)124U)
/*!
 * @brief FECSS RFS DBG Header Size
 */
#define M_FE_RFS_DBG_HDR_SIZE               ((UINT16)4U)
/*!
 * @brief FECSS RFS MB Size
 */
#define M_FE_RFS_DBG_DATA_SIZE              ((UINT16)64U)

/*!
 * @brief FECSS RFS MB Status Init Fail
 */
#define M_FE_RFS_MB_STS_UNINIT              ((UINT8)0U)
/*!
 * @brief FECSS RFS MB Status pass/Init done
 */
#define M_FE_RFS_MB_STS_PASS                ((UINT8)0xAU)
/*!
 * @brief FECSS RFS MB Status IPC fail
 */
#define M_FE_RFS_MB_STS_IPC_FAIL            ((UINT8)0x5U)

/*!
 * @brief FECSS RFS Boot Status Halt
 */
#define M_FE_RFS_BOOT_STS_UNINIT            ((UINT8)0U)
/*!
 * @brief FECSS RFS Boot Status Pass
 */
#define M_FE_RFS_BOOT_STS_PASS              ((UINT8)0xAU)
/*!
 * @brief FECSS RFS Boot Status Fail
 */
#define M_FE_RFS_BOOT_STS_FAIL              ((UINT8)0x5U)

/*!
 * @brief FECSS RFS MB Write Done and cmd/resp triggered
 */
#define M_FE_RFS_MB_WRITE_CMDRSP_TRIG       ((UINT32)0xA1U)
/*!
 * @brief FECSS RFS MB Read Done and cmd/resp received
 */
#define M_FE_RFS_MB_READ_CMDRSP_RECV        ((UINT32)0x50U)
/*!
 * @brief FECSS RFS MB and cmd/resp status
 */
#define M_FE_RFS_MB_CMDRSP_STATUS_MASK      ((UINT32)0xFFU)

/*!
 * @brief FECSS RFS CM3 FW unhat value
 */
#define M_FE_RFS_CM3_FW_UNHALT_VAL          ((UINT32)0xF5A36A17U)

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library RFS Driver module Type defines
 * @{
 */

/*!
 * @brief FECSS RFS Command/Response Data structure
 */
typedef struct
{
    /*!
     * @brief  RFS MB Command ID
     */
    UINT16 h_CommandId;

    /*!
     * @brief  RFS MB Data structure size in bytes
     */
    UINT16 h_CmdResDataSize;

    /*!
     * @brief  The pointer to RFS MB Data structure
     */
    void *p_CmdResDataPtr;

} T_FE_RFS_CMD_RES_DATA;

/*!
 * @brief FECSS RFS Command and Response Execute Data structure
 */
typedef struct
{
    /*!
     * @brief  RFS MB Command ID
     */
    UINT16 h_CommandId;

    /*!
     * @brief  RFS Command size in bytes
     */
    UINT16 h_CmdSize;

    /*!
     * @brief  RFS Response size in bytes
     */
    UINT16 h_ResSize;

    /*!
     * @brief  The pointer to command Data structure
     */
    const void *p_CmdDataPtr;

    /*!
     * @brief  The pointer to response Data structure
     */
    void *p_ResDataPtr;

} T_FE_RFS_CMD_RES_EXC_DATA;

/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */
T_RETURNTYPE rfs_rfsOpen(UINT8 c_devIndex, const T_FE_RFS_BOOT_INFO z_rfsBootInfo, \
        T_FE_RFS_BOOT_STS* p_rfsBootStatus, T_FE_DRIVER_DATA* p_feDrvData);
T_RETURNTYPE rfs_rfsClose(UINT8 c_devIndex, T_FE_DRIVER_DATA* p_feDrvData);
T_RETURNTYPE rfs_rfsCmdSet(UINT8 c_devIndex, T_FE_RFS_CMD_RES_DATA z_rfsMbData, \
        T_FE_DRIVER_DATA* p_feDrvData);
T_RETURNTYPE rfs_rfsResGet(UINT8 c_devIndex, T_FE_RFS_CMD_RES_DATA* p_rfsMbData, \
        T_FE_DRIVER_DATA* p_feDrvData);
T_RETURNTYPE rfs_rfsClkCfg(UINT8 c_devIndex, UINT16 h_clkFreq);
T_RETURNTYPE rfs_cmdRespExecute(UINT8 c_devIndex, T_FE_RFS_CMD_RES_EXC_DATA z_cmdResExcData);
void rfs_cmdRespDev0IntHandler(void);
T_RETURNTYPE rfs_sensStatusClear(UINT8 c_devIndex);
T_RETURNTYPE rfs_rfsBootStatusGet(UINT8 c_devIndex, T_FE_RFS_BOOT_STS *p_rfsBootSts);
T_RETURNTYPE rfs_rfsBootInfoGet(UINT8 c_devIndex, T_FE_RFS_BOOT_INFO *p_rfsBootInfo);

#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF rfs_driver.h
 */


