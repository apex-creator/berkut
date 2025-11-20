/*!*****************************************************************************
 * @file rl_monitor.h
 *
 * @brief mmWaveLink Monitor Module header file
 *
 * @b Description @n
 * This is a mmWaveLink Monitor Module API header file, provides information
 * about FECSS Monitor control API functions and Data Structures.
 *
 * @warning Application developer / User shall review Monitor APIs data structure,
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
 * 0.1     19Sep2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef MMWL_MONITOR_H
#define MMWL_MONITOR_H

/*!*****************************************************************************
 \brief mmWaveLink Library Monitor Module API details.

 \section mmwl_Monitor_sec mmWaveLink Monitor Module
mmWaveLink Monitor Module is responsible for providing APIs for FECSS Monitor control. Below block diagram shows the individual functional blocks of the Monitor Module.

@image html mmwl_monitor.png "mmWaveLinkLib Monitor Module"

 \subsection mmwl_monitor_api_subsec mmWaveLink Monitor API functions and Data Structures

The following mmWaveLink APIs provides FECSS Monitor control for application, refer function definition for details on data structure.
  - \ref rl_monEnableTrig
  - \ref rl_monDbgPdMeas
  - \ref rl_monDbgTxPwrMeas
  - \ref rl_monLiveSynthFreqCfg
  - \ref rl_monLiveRxSaturationCfg
  - \ref rl_monLiveGpadcCtmCfg
  - \ref rl_monPllCtrlVoltCfg
  - \ref rl_monTxNRxLbCfg
  - \ref rl_monTxNPowerCfg
  - \ref rl_monTxNBbCfg
  - \ref rl_monTxNDcSigCfg
  - \ref rl_monRxHpfDcSigCfg
  - \ref rl_monPmClkDcSigCfg
.
Link to Parent Directory: \ref MMWL_MONITOR_API
Note: Monitor API features are not supported in this release, reserved for future use only.

 @addtogroup MMWL_MONITOR_API mmWaveLink Monitor Control API functions
 @{
 @brief Below sections provides information about mmWaveLink Library Monitor module API functions and data structure and MACROs
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
 * @name mmWaveLink Library Monitor Module API command IDs
 * @{
 */
/*!
 * @brief mmWaveLink Monitor enable & trigger command ID
 */
#define M_RL_API_ID_MON_ENABLE_TRIG             ((UINT16)0x0080U)

/*!
 * @brief mmWaveLink Monitor Debug PD power meas command ID
 */
#define M_RL_API_ID_MON_DBG_PD_PWR_MEAS         ((UINT16)0x0081U)

/*!
 * @brief mmWaveLink Monitor Debug TX power meas command ID
 */
#define M_RL_API_ID_MON_DBG_TX_PWR_MEAS         ((UINT16)0x0082U)

/*!
 * @brief mmWaveLink Monitor Live Synth Freq cfg command ID
 */
#define M_RL_API_ID_MON_LIVE_SYNTH_FREQ_CFG   	((UINT16)0x0090U)

/*!
 * @brief mmWaveLink Monitor Live Rx Saturation cfg command ID
 */
#define M_RL_API_ID_MON_LIVE_RX_SAT_CFG   		  ((UINT16)0x0091U)

/*!
 * @brief mmWaveLink Monitor Live Synth Freq cfg command ID
 */
#define M_RL_API_ID_MON_LIVE_GPADC_CTM_CFG   	  ((UINT16)0x0092U)

/*!
 * @brief mmWaveLink Monitor PLL controll volt cfg command ID
 */
#define M_RL_API_ID_MON_PLL_CTRL_VOLT_CFG   	  ((UINT16)0x0093U)

/*!
 * @brief mmWaveLink Monitor TXn RX loopback cfg command ID
 */
#define M_RL_API_ID_MON_TXn_RX_LB_CFG   			  ((UINT16)0x0094U)

/*!
 * @brief mmWaveLink Monitor TXn power cfg command ID
 */
#define M_RL_API_ID_MON_TXn_PWR_CFG   		      ((UINT16)0x0095U)

/*!
 * @brief mmWaveLink Monitor TXn ballbreak cfg command ID
 */
#define M_RL_API_ID_MON_TXn_BB_CFG   		        ((UINT16)0x0096U)

/*!
 * @brief mmWaveLink Monitor TXn Internal DC signal cfg command ID
 */
#define M_RL_API_ID_MON_TXn_DCSIG_CFG           ((UINT16)0x0097U)

/*!
 * @brief mmWaveLink Monitor RX HPF & Internal DC signal cfg command ID
 */
#define M_RL_API_ID_MON_RX_HPF_DCSIG_CFG        ((UINT16)0x0098U)

/*!
 * @brief mmWaveLink Monitor PM LO CLK Internal DC signal cfg command ID
 */
#define M_RL_API_ID_MON_PMCLK_DCSIG_CFG         ((UINT16)0x0099U)

/*! @} */

/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name mmWaveLink Library Monitor Module Type defines
 * @{
 */


/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Monitor Module global Variables
 * @{
 */

/*! @} */
/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Monitor module API function calls
 * @{
 */
M_LIB_EXPORT T_RETURNTYPE rl_monEnableTrig(UINT8 c_devIndex, \
                const T_RL_API_MON_ENABLE_TRIG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monDbgPdMeas(UINT8 c_devIndex, \
                const T_RL_API_MON_DBG_PD_MEAS_CMD *p_apiCmdData, \
                T_RL_API_MON_DBG_PD_MEAS_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_monDbgTxPwrMeas(UINT8 c_devIndex, \
                const T_RL_API_MON_DBG_TXPWR_MEAS_CMD *p_apiCmdData, \
                T_RL_API_MON_DBG_TXPWR_MEAS_RSP *p_apiResData);

M_LIB_EXPORT T_RETURNTYPE rl_monLiveSynthFreqCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_LIVE_SYNTH_FREQ_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monLiveRxSaturationCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_LIVE_RX_SATURATION_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monLiveGpadcCtmCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_LIVE_GPADC_CTM_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monPllCtrlVoltCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_PLL_CTRL_VOLT_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monTxNRxLbCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_RX_LB_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monTxNPowerCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_POWER_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monTxNBbCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_BB_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monTxNDcSigCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_DCSIG_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monRxHpfDcSigCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_RX_HPF_DCSIG_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_monPmClkDcSigCfg(UINT8 c_devIndex, \
                const T_RL_API_MON_PMCLK_DCSIG_CFG *p_apiCmdData);

/*! @} */


#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF rl_monitor.h
 */


