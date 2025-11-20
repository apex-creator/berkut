/*!*****************************************************************************
 * @file fecsslib.h
 *
 * @brief FECSSLIB (FE) application interface header file.
 *
 * @b Description @n
 * This is a fecsslib (fe) application interface header file, provides information
 * about application interface, integration data structure and callback functions.
 * mmWaveLink has internal dependency on this library and the interface is done with
 * mmWaveLink Library, the application is not required to interface with fecsslib
 * directly, all the DFP APIs are interfaced at mmWaveLink layer using mmWaveLink interface.
 *
 * @warning DFP or Application developer shall review and update the fecsslib
 * integration client call back data structure and shall call fe_fecssLibInit() function.
 * Refer mmwave-dfp-low/common/dfp_users.h file for build time MACROs.
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
#ifndef FECSSLIB_H
#define FECSSLIB_H

/*!*****************************************************************************
 \brief FECSSLib is the middle driver layer in mmWave DFP, which provides interface to FECSS from mmwaveLink and allows mmwaveLink to invoke low level driver function API calls based on mmWaveLink API calls.

 \section fecsslib_interface_sec FECSSLib Interface and Porting Details
The interface frame work is common to both mmWaveLink and FECSSLib. The mmwaveLink internally calls FECSSLib interface functions and user is not required to perform the same. \n
The FECSSLib interface with mmWaveLink is performed using below functions.
\ref fe_fecssLibInit API function    - ROM/PATCH function
\ref fe_fecssLibDeInit API function  - ROM/PATCH function

\ref T_DFP_CLIENT_CB_DATA client call back data structure used in the interface.

Refer below section for more details on mmWave DFP interface frame work data structures and MACROs.
 - \ref MMWAVE_DFP_INTERFACE
.
 \note Refer \ref mmwl_interface_sec for mmwaveLink interface and porting details.

 \section fecsslib_api_sec FECSSLib driver functions API details
FECSSLib driver API functions provides seamless driver interface to mmWave Sensor FECSS Front End. FECSSLib provides low level driver functions to configure/control FECSS hardware and returns the status to caller. mmWaveLink APIs calls these drivers to get service from FECSS. \n
The FECSSLib drivers are divided into three major driver modules based on functionality as below, refer individual module section for more details on driver APIs.
-# \ref fecsslib_Device_sec : The Device module consist of below major driver APIs
  - FECSSLib Framework Init / DeInit
  - FECSS Device Power ON/OFF
  - FECSSLib Version Get
  - FECSS Digital monitors ?
-# \ref fecsslib_Sensor_sec : The Sensor module consist of below major driver APIs
  - Chirp profile Configuration
  - Chirp Power Save Control
  - Per Chirp Configuration
  - Frame / Burst Configuration
  - Sensor Start / Stop
  - CW Mode
-# \ref fecsslib_rfs_sec : The Monitor module consist of below major driver APIs, these monitors are available only in Functional Safety RTM releases.
  - FECSS RF Power ON/OFF
  - Front End Clock Configuration
  - FECSS Analog Monitors
  - FECSS RFS Digital monitors ?
  - FECSS Calibrations
  - Device Temperature Measurement
  - Device GPADC Signal Measurement
  - Test Source and RF Loopback Options

Refer below section for more details
 - \ref FECSSLIB_DRV_API

 \section fecsslib_build_sec FECSSLib Library Build Options
The FECSSLib library can be built using make files and batch files provided by this package. The 'build' directory in 'mmwave-dfp-low/fecsslib' parent directory includes 'fe_build.bat' batch file and make files for the library generation for FECSSLib componets. \n
The 'fe_setenv.bat' file in 'build' directory shall be updated to select below paths before building the library:
 - Path to TI arm compiler CG_TOOLS version '20.2.3.LTS'
 - Path to make.exe file version 'gmake 4.1'
.
The below commands can be used to perform the build, run this from 'build' directory. Refer build help for more details on build options.
 - fe_build.bat help
 - fe_build.bat romlib_m4_release 6432
 - fe_build.bat patchlib_m4_debug 1432
 .
Refer \ref FECSSLIB_ROM_MAP for more details on ROM/RAM, PATCH FECSSLib library.

Link to Parent Directory: \ref FECSSLIB_DRV_API

 @addtogroup FECSSLIB_DRV_API FECSSLib Driver API functions
 @{
 @brief Below sections provides information about FECSS Library driver API functions, data structure and MACROs.
********************************************************************************
*/

/*!
 * @defgroup FECSSLIB_DEVICE_API FECSSLib Device Control driver API functions
 * @brief FECSSLib Device Control driver API functions and Data Structures
 * @ingroup FECSSLIB_DRV_API
 */

/*!
 * @defgroup FECSSLIB_SENSOR_API FECSSLib Sensor Control driver API functions
 * @brief FECSSLib Sensor Control driver API functions and Data Structures
 * @ingroup FECSSLIB_DRV_API
 */

/*!
 * @defgroup FECSSLIB_RFS_API FECSSLib RFS Control driver API functions
 * @brief FECSSLib RFS Control driver API functions and Data Structures
 * @ingroup FECSSLIB_DRV_API
 */

/*!
 * @defgroup FECSSLIB_ROM_MAP FECSSLib ROM/PATCH functions mapping
 * @brief FECSSLib ROM/PATCH mapping for driver APIs
 * @ingroup FECSSLIB_DRV_API
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <common/dfp_datatypes.h>
#include <common/dfp_users.h>
#include <common/dfp_device.h>
#include <common/dfp_sensor.h>
#include <common/dfp_monitor.h>
#include <fecsslib/device/include/fe_driver.h>
#include <common/dfp_trace.h>
#include <fecsslib/rfscripter/include/rfs_command.h>
#include <fecsslib/device/include/fe_device.h>
#include <fecsslib/sensor/include/fe_sensor.h>
#include <fecsslib/rommap.h>

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Framework Interface MACROs
 * @{
 */

/*!
 * @brief FECSS Library Version
 */
#define M_FE_FECSSLIB_VERSION              "3.1.8.1.29.10.23"
#define M_FE_FECSSLIB_VERSION_GEN          (3U)
#define M_FE_FECSSLIB_VERSION_MAJOR        (1U)
#define M_FE_FECSSLIB_VERSION_MINOR        (8U)
#define M_FE_FECSSLIB_VERSION_BUILD        (1U)
#define M_FE_FECSSLIB_VERSION_DAY          (29U)
#define M_FE_FECSSLIB_VERSION_MONTH        (10U)
#define M_FE_FECSSLIB_VERSION_YEAR         (23U)

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Framework Interface Type defines
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
 * @name FECSS Library API Interface ROM function calls
 * @{
 */
M_LIB_EXPORT T_RETURNTYPE fe_fecssLibInit_rom(UINT32 w_deviceMap, \
                T_DFP_CLIENT_CB_DATA z_clientCbData);
M_LIB_EXPORT T_RETURNTYPE fe_fecssLibDeInit_rom(UINT32 w_deviceMap);

/*! @} */

#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF mmwavelink.h
 */


