/*!*****************************************************************************
 * @file rl_sensor.h
 *
 * @brief mmWaveLink Sensor Module header file
 *
 * @b Description @n
 * This is a mmWaveLink Sensor Module API header file, provides information
 * about FECSS Sensor control API functions and Data Structures.
 *
 * @warning Application developer / User shall review Sensor APIs data structure,
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
 * 0.1     02Jun2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef MMWL_SENSOR_H
#define MMWL_SENSOR_H

/*!*****************************************************************************
 \brief mmWaveLink Library Sensor Module API details.

 \section mmwl_Sensor_sec mmWaveLink Sensor Module
mmWaveLink Sensor Module is responsible for providing APIs for FECSS Sensor control. Below block diagram shows the individual functional blocks of the Sensor Module.

@image html mmwl_sensor.png "mmWaveLinkLib Sensor Module"

 \subsection mmwl_sensor_api_subsec mmWaveLink Sensor API functions and Data Structures

The following mmWaveLink APIs provides FECSS Sensor control for application, refer function definition for details on data structure.
  - \ref rl_sensChirpProfComnCfg
  - \ref rl_sensChirpProfComnCfgGet
  - \ref rl_sensChirpProfTimeCfg
  - \ref rl_sensChirpProfTimeCfgGet
  - \ref rl_sensPerChirpCfg
  - \ref rl_sensPerChirpCfgGet
  - \ref rl_sensPerChirpCtrl
  - \ref rl_sensPerChirpCtrlGet
  - \ref rl_sensFrameCfg
  - \ref rl_sensFrameCfgGet
  - \ref rl_sensSensorStart
  - \ref rl_sensSensorStop
  - \ref rl_sensStatusGet
  - \ref rl_sensDynPwrSaveDis
  - \ref rl_sensDynPwrSaveStsGet
  - \ref rl_sensLoopBackCfg
  - \ref rl_sensLoopBackEna


Link to Parent Directory: \ref MMWL_SENSOR_API
.

 @addtogroup MMWL_SENSOR_API mmWaveLink Sensor Control API functions
 @{
 @brief Below sections provides information about mmWaveLink Library Sensor module API functions and data structure and MACROs
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
 * @name mmWaveLink Library Sensor Module API command IDs
 * @{
 */
/*!
 * @brief mmWaveLink sensor profile common cfg command ID
 */
#define M_RL_API_ID_SENS_PROF_COMN_CFG            ((UINT16)0x0040U)

/*!
 * @brief mmWaveLink sensor profile common cfg Get command ID
 */
#define M_RL_API_ID_SENS_PROF_COMN_CFG_GET        ((UINT16)0x0041U)

/*!
 * @brief mmWaveLink sensor profile time cfg command ID
 */
#define M_RL_API_ID_SENS_PROF_TIME_CFG            ((UINT16)0x0042U)

/*!
 * @brief mmWaveLink sensor profile time cfg Get command ID
 */
#define M_RL_API_ID_SENS_PROF_TIME_CFG_GET        ((UINT16)0x0043U)

/*!
 * @brief mmWaveLink sensor per chirp cfg command ID
 */
#define M_RL_API_ID_SENS_PER_CHIRP_CFG            ((UINT16)0x0044U)

/*!
 * @brief mmWaveLink sensor per chirp cfg Get command ID
 */
#define M_RL_API_ID_SENS_PER_CHIRP_CFG_GET        ((UINT16)0x0045U)

/*!
 * @brief mmWaveLink sensor per chirp control command ID
 */
#define M_RL_API_ID_SENS_PER_CHIRP_CTRL           ((UINT16)0x0046U)

/*!
 * @brief mmWaveLink sensor per chirp control Get command ID
 */
#define M_RL_API_ID_SENS_PER_CHIRP_CTRL_GET       ((UINT16)0x0047U)

/*!
 * @brief mmWaveLink sensor frame config command ID
 */
#define M_RL_API_ID_SENS_FRAME_CFG                ((UINT16)0x0048U)

/*!
 * @brief mmWaveLink sensor frame config Get command ID
 */
#define M_RL_API_ID_SENS_FRAME_CFG_GET            ((UINT16)0x0049U)

/*!
 * @brief mmWaveLink sensor start command ID
 */
#define M_RL_API_ID_SENS_START                    ((UINT16)0x004AU)

/*!
 * @brief mmWaveLink sensor stop command ID
 */
#define M_RL_API_ID_SENS_STOP                     ((UINT16)0x004BU)

/*!
 * @brief mmWaveLink sensor status Get command ID
 */
#define M_RL_API_ID_SENS_STATUS_GET               ((UINT16)0x004CU)

/*!
 * @brief mmWaveLink sensor dynamic power save disable command ID
 */
#define M_RL_API_ID_SENS_DYN_PS_DIS               ((UINT16)0x004DU)

/*!
 * @brief mmWaveLink sensor dynamic power save disable Get command ID
 */
#define M_RL_API_ID_SENS_DYN_PS_DIS_GET           ((UINT16)0x004EU)

/*!
 * @brief mmWaveLink sensor loopback config command ID
 */
#define M_RL_API_ID_SENS_LB_CFG                   ((UINT16)0x004FU)

/*!
 * @brief mmWaveLink sensor loopback control command ID
 */
#define M_RL_API_ID_SENS_LB_ENA                   ((UINT16)0x0050U)

/*! @} */

/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name mmWaveLink Library Sensor Module Type defines
 * @{
 */


/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Sensor Module global Variables
 * @{
 */

/*! @} */
/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Sensor module API function calls
 * @{
 */
M_LIB_EXPORT T_RETURNTYPE rl_sensChirpProfComnCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensChirpProfComnCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_sensChirpProfTimeCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensChirpProfTimeCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_sensPerChirpCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensPerChirpCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CFG *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_sensPerChirpCtrl(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CTRL *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensPerChirpCtrlGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CTRL *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_sensFrameCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_FRAME_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensFrameCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_FRAME_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensSensorStart(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_START_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensSensorStop(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_STOP_CMD *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensStatusGet(UINT8 c_devIndex, \
                T_RL_API_SENSOR_STATUS_RSP *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_sensDynPwrSaveDis(UINT8 c_devIndex, \
                const T_RL_API_SENS_DYN_PWR_SAVE_DIS *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensDynPwrSaveStsGet(UINT8 c_devIndex, \
                T_RL_API_SENS_DYN_PWR_SAVE_DIS *p_apiResData);
M_LIB_EXPORT T_RETURNTYPE rl_sensLoopBackCfg(UINT8 c_devIndex, \
                const T_RL_API_SENS_LOOPBACK_CFG *p_apiCmdData);
M_LIB_EXPORT T_RETURNTYPE rl_sensLoopBackEna(UINT8 c_devIndex, \
                const T_RL_API_SENS_LOOPBACK_ENA *p_apiCmdData);

/*! @} */


#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF rl_sensor.h
 */


