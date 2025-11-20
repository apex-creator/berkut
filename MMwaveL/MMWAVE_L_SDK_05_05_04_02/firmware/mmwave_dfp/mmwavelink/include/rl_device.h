/*!*****************************************************************************
 * @file rl_device.h
 *
 * @brief mmWaveLink Device Module header file
 *
 * @b Description @n
 * This is a mmWaveLink Device Module API header file, provides information
 * about FECSS Device control API functions and Data Structures.
 *
 * @warning Application developer / User shall review Device APIs data structure,
 * API functions and handle the Errors.
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
 * 0.1     12Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef MMWL_DEVICE_H
#define MMWL_DEVICE_H

/*!*****************************************************************************
 \brief mmWaveLink Library Device Module API details.

 \section mmwl_Device_sec mmWaveLink Device Module
mmWaveLink Device Module is responsible for providing APIs for FECSS device control. Below block diagram shows the individual functional blocks of the Device Module.

@image html mmwl_device.png "mmWaveLinkLib Device Module"

 \subsection mmwl_device_api_subsec mmWaveLink Device API functions and Data Structures

The following mmWaveLink APIs provides FECSS Device control for application, refer function definition for details on data structure.
  - \ref rl_mmWaveLinkInit
  - \ref rl_mmWaveLinkDeInit
  - \ref rl_fecssDevPwrOn
  - \ref rl_fecssDevPwrOff
  - \ref rl_mmWaveDfpVerGet
  - \ref rl_fecssRfPwrOnOff
  - \ref rl_fecssRfFactoryCal
  - \ref rl_fecssRfRuntimeCal
  - \ref rl_fecssRfClockBwCfg
  - \ref rl_fecssGpadcMeasCfg
  - \ref rl_fecssGpadcMeasTrig
  - \ref rl_fecssTempMeasCfg
  - \ref rl_fecssTempMeasTrig
  - \ref rl_fecssRfFactoryCalDataGet
  - \ref rl_fecssRfFactoryCalDataSet
  - \ref rl_fecssDevClockCtrl
  - \ref rl_fecssDevRdifCtrl
  - \ref rl_fecssDevStatusGet
  - \ref rl_fecssRfStatusGet
  - \ref rl_fecssRfFaultStatusGet
  - \ref rl_fecssDieIdGet
  - \ref rl_fecssRfRxTxCalDataGet
  - \ref rl_fecssRfRxTxCalDataSet
  - \ref rl_fecssRfsDbgCfg

Link to Parent Directory: \ref MMWL_DEVICE_API
.

 @addtogroup MMWL_DEVICE_API mmWaveLink Device Control API functions
 @{
 @brief Below sections provides information about mmWaveLink Library Device module API functions and data structure and MACROs
********************************************************************************
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
 * @name mmWaveLink Library Device Module API command IDs
 * @{
 */
/*!
 * @brief mmWaveLink API Error response command ID
 */
#define M_RL_API_ID_ERROR_RESP                  ((UINT16)0x0000U)

/*!
 * @brief mmWaveLink Interface Init command ID
 */
#define M_RL_API_ID_MMWL_INIT                   ((UINT16)0x0001U)

/*!
 * @brief mmWaveLink Interface DeInit command ID
 */
#define M_RL_API_ID_MMWL_DEINIT                 ((UINT16)0x0002U)

/*!
 * @brief mmWaveLink FECSS DEV PWR ON/OFF command ID
 */
#define M_RL_API_ID_FECSS_DEV_PWR_ON            ((UINT16)0x0003U)

/*!
 * @brief mmWaveLink FECSS DEV PWR ON/OFF command ID
 */
#define M_RL_API_ID_FECSS_DEV_PWR_OFF           ((UINT16)0x0004U)

/*!
 * @brief mmWaveLink DFP Version GET command ID
 */
#define M_RL_API_ID_DFP_VER_GET                 ((UINT16)0x0005U)

/*!
 * @brief mmWaveLink FECSS RF PWR ON/OFF command ID
 */
#define M_RL_API_ID_FECSS_RF_PWR_ONOFF          ((UINT16)0x0006U)

/*!
 * @brief mmWaveLink FECSS Factory cal trigger command ID
 */
#define M_RL_API_ID_FECSS_RF_FACTORY_CAL        ((UINT16)0x0007U)

/*!
 * @brief mmWaveLink FECSS RF run time cal trigger command ID
 */
#define M_RL_API_ID_FECSS_RF_RUN_CAL            ((UINT16)0x0008U)

/*!
 * @brief mmWaveLink FECSS RF clock Cfg (clk BW) command ID (reserved)
 */
#define M_RL_API_ID_FECSS_RF_CLKBW_CFG          ((UINT16)0x0009U)

/*!
 * @brief mmWaveLink FECSS GPADC Measurement config command ID
 */
#define M_RL_API_ID_FECSS_GPADC_MEAS_CFG        ((UINT16)0x000AU)

/*!
 * @brief mmWaveLink FECSS GPADC Trigger command ID
 */
#define M_RL_API_ID_FECSS_GPADC_MEAS_TRIG       ((UINT16)0x000BU)

/*!
 * @brief mmWaveLink FECSS Temperature Measurement config command ID
 */
#define M_RL_API_ID_FECSS_TEMP_MEAS_CFG         ((UINT16)0x000CU)

/*!
 * @brief mmWaveLink FECSS Temperature Measurement trigger command ID
 */
#define M_RL_API_ID_FECSS_TEMP_MEAS_TRIG        ((UINT16)0x000DU)

/*!
 * @brief mmWaveLink FECSS factory Cal data Get command ID
 */
#define M_RL_API_ID_FECSS_FACTORY_CAL_DATA_GET  ((UINT16)0x000EU)

/*!
 * @brief mmWaveLink FECSS factory Cal data Set command ID
 */
#define M_RL_API_ID_FECSS_FACTORY_CAL_DATA_SET  ((UINT16)0x000FU)

/*!
 * @brief mmWaveLink FECSS Device Clock Cfg command ID
 */
#define M_RL_API_ID_FECSS_DEV_CLK_CTRL          ((UINT16)0x0010U)

/*!
 * @brief mmWaveLink FECSS RDIF Cfg command ID
 */
#define M_RL_API_ID_FECSS_RDIF_CFG              ((UINT16)0x0011U)

/*!
 * @brief mmWaveLink FECSS Device Status Get command ID
 */
#define M_RL_API_ID_FECSS_DEV_STS_GET           ((UINT16)0x0012U)

/*!
 * @brief mmWaveLink FECSS RF Status Get command ID
 */
#define M_RL_API_ID_FECSS_RF_STS_GET            ((UINT16)0x0013U)

/*!
 * @brief mmWaveLink Reserved command ID
 */
#define M_RL_API_ID_FECSS_FAULT_STS_GET         ((UINT16)0x0014U)

/*!
 * @brief mmWaveLink FECSS Device Die ID Get command ID
 */
#define M_RL_API_ID_FECSS_DIEID_GET             ((UINT16)0x0015U)

/*!
 * @brief mmWaveLink FECSS RX-TX Cal data Get command ID
 */
#define M_RL_API_ID_FECSS_RXTX_CAL_DATA_GET     ((UINT16)0x0016U)

/*!
 * @brief mmWaveLink FECSS RX-TX Cal data Set command ID
 */
#define M_RL_API_ID_FECSS_RXTX_CAL_DATA_SET     ((UINT16)0x0017U)

/*!
 * @brief mmWaveLink FECSS RFS Debug Cfg command ID
 */
#define M_RL_API_ID_FECSS_RFS_DBG_CFG           ((UINT16)0x0018U)

/*!
 * @brief mmWaveLink FECSS Runtime Tx CLPC calibration command ID
 */
#define M_RL_API_ID_FECSS_RUNTIME_TX_CLPC_CAL   ((UINT16)0x0019U)

/*! @} */

/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name mmWaveLink Library Device Module Type defines
 * @{
 */


/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Device Module global Variables
 * @{
 */

/*! @} */
/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Device module API function calls
 * @{
 */
M_LIB_EXPORT T_RETURNTYPE rl_fecssDevPwrOn(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_PWR_ON_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssDevPwrOff(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_PWR_OFF_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_mmWaveDfpVerGet(UINT8 c_devIndex, \
                T_RL_API_DFP_FW_VER_GET_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfPwrOnOff(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RF_PWR_CFG_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfFactoryCal(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RF_FACT_CAL_CMD *p_apiCmdData, \
                T_RL_API_FECSS_RF_FACT_CAL_RSP* p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfRuntimeCal(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RF_RUN_CAL_CMD *p_apiCmdData, \
                T_RL_API_FECSS_RF_RUN_CAL_RSP* p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfClockBwCfg(UINT8 c_devIndex, \
                const T_RL_API_FECSS_CLK_BW_CFG_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssGpadcMeasCfg(UINT8 c_devIndex, \
                const T_RL_API_FECSS_GPADC_MEAS_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssGpadcMeasTrig(UINT8 c_devIndex, \
                T_RL_API_FECSS_GPADC_MEAS_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssTempMeasCfg(UINT8 c_devIndex, \
                const T_RL_API_FECSS_TEMP_MEAS_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssTempMeasTrig(UINT8 c_devIndex, \
                T_RL_API_FECSS_TEMP_MEAS_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfFactoryCalDataGet(UINT8 c_devIndex, \
                T_RL_API_FECSS_FACT_CAL_DATA *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfFactoryCalDataSet(UINT8 c_devIndex, \
                const T_RL_API_FECSS_FACT_CAL_DATA *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssDevClockCtrl(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssDevRdifCtrl(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RDIF_CTRL_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssDevStatusGet(UINT8 c_devIndex, \
                T_RL_API_FECSS_DEV_STS_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfStatusGet(UINT8 c_devIndex, \
                T_RL_API_FECSS_RF_STS_GET_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfFaultStatusGet(UINT8 c_devIndex, \
                T_RL_API_RFS_FAULT_STS_GET_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssDieIdGet(UINT8 c_devIndex, \
                T_RL_API_SENSOR_DIEID_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfRxTxCalDataGet(UINT8 c_devIndex, \
                T_RL_API_FECSS_RXTX_CAL_DATA *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfRxTxCalDataSet(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RXTX_CAL_DATA *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRfsDbgCfg(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RFS_DBG_CFG_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_fecssRlRuntimeTxClpcCal(UINT8 c_devIndex, \
                const T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD *p_apiCmdData, \
                T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP* p_apiResData);
/*! @} */


#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF rl_device.h
 */


