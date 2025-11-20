/*!*****************************************************************************
 * @file fe_sensor.h
 *
 * @brief FECSSLib Sensor Module header file.
 *
 * @b Description @n
 * This FECSS library Sensor Module header file defines FECSS Sensor control APIs,
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
 * 0.1     09July2021  TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef FE_SENSOR_H
#define FE_SENSOR_H

/*!*****************************************************************************
 \brief FECSSLib Library Sensor Module Driver API details.

 \section fecsslib_Sensor_sec FECSSLib Sensor Module
FECSSLib Sensor Module is responsible for providing driver APIs for FECSS sensor control, configuration and frame start / stop. Below block diagram shows the individual functional
blocks of the Sensor Module.

@image html fecsslib_sensor.png "FECSSLib Sensor Module"

 \subsection fecsslib_sensor_api_subsec FECSSLib Sensor driver API functions and Data Structures

The following FECSSLib sensor APIs provides FECSS Sensor control for application, refer function definition for details on data structure.
  - \ref fe_sensChirpProfComnCfg     - ROM/PATCH function
  - \ref fe_sensChirpProfComnCfgGet  - ROM/PATCH function
  - \ref fe_sensChirpProfTimeCfg     - ROM/PATCH function
  - \ref fe_sensChirpProfTimeCfgGet  - ROM/PATCH function
  - \ref fe_sensPerChirpCfg          - ROM/PATCH function
  - \ref fe_sensPerChirpCfgGet       - ROM/PATCH function
  - \ref fe_sensPerChirpCtrl         - ROM/PATCH function
  - \ref fe_sensPerChirpCtrlGet      - ROM/PATCH function
  - \ref fe_sensFrameCfg             - ROM/PATCH function
  - \ref fe_sensFrameCfgGet          - ROM/PATCH function
  - \ref fe_sensorStart              - ROM/PATCH function
  - \ref fe_sensorStop               - ROM/PATCH function
  - \ref fe_sensorStatusGet          - ROM/PATCH function
.

Link to Parent Directory: \ref FECSSLIB_SENSOR_API

   @addtogroup FECSSLIB_SENSOR_API FECSSLib Sensor Control driver API functions
    @{
 @brief Below sections provides information about FECSSLib Library Sensor module driver API functions and data structure and MACROs
********************************************************************************
*/

/*!
 * @defgroup FECSSLIB_SENSOR_DRV_MODULE Sensor device Driver API functions
 * @brief FECSSLib Sensor device driver API functions and Data Structures
 * @ingroup FECSSLIB_SENSOR_API
 */

/*!
 * @defgroup FECSSLIB_SENSOR_DRV_PATCH Sensor Driver API Patch functions
 * @brief FECSSLib Sensor Device driver API Patch functions and Data Structures
 * @ingroup FECSSLIB_SENSOR_API
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
 * @name FECSS Library Sensor module MACROs
 * @{
 */

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Sensor module Type defines
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
 * @name FECSS Library Sensor module API functions
 * @{
 */
M_LIB_EXPORT T_RETURNTYPE fe_sensChirpProfComnCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_sensChirpProfComnCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_resData);
M_LIB_EXPORT T_RETURNTYPE fe_sensChirpProfTimeCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_sensChirpProfTimeCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_resData);
M_LIB_EXPORT T_RETURNTYPE fe_sensPerChirpCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_sensPerChirpCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CFG *p_resData);
M_LIB_EXPORT T_RETURNTYPE fe_sensPerChirpCtrl_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CTRL *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_sensPerChirpCtrlGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CTRL *p_resData);
M_LIB_EXPORT T_RETURNTYPE fe_sensFrameCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_FRAME_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_sensFrameCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_FRAME_CFG *p_resData);
M_LIB_EXPORT T_RETURNTYPE fe_sensorStart_rom(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_START_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_sensorStop_rom(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_STOP_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE fe_sensorStatusGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENSOR_STATUS_RSP *p_resData);

/*! @} */
#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF fe_sensor.h
 */


