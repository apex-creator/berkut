/*!*****************************************************************************
 * @file mmwavelink.h
 *
 * @brief mmWaveLink (RadarLink) application interface header file
 *
 * @b Description @n
 * This is a mmWaveLink application interface header file, provides information
 * about application interface, integration data structure and callback functions.
 *
 * @warning Application developer / User shall review and update the mmWaveLink
 * integration client call back data structure and shall call rl_mmWaveLinkInit()
 * function. Refer mmwave-dfp-low/common/dfp_users.h file for build time MACROs.
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
#ifndef MMWAVELINK_H
#define MMWAVELINK_H

/*!*****************************************************************************
 \brief mmWaveLink Library Interface and API details.

 \section mmwl_interface_sec mmWaveLink Application Interface and Porting Details
mmWaveLink is the top most layer in mmWave DFP, which provides interface to application and allows application to invoke mmWaveLink SW API calls. Below block diagram shows high level interface layer unit blocks, the mmWaveLink interface framework ensures reliable SW integration between application and mmWaveLink Library.

@image html mmwavelink.png "mmWaveLink 3.x Library Block Diagram"

The mmwaveLink framework uses \ref T_DFP_CLIENT_CB_DATA client call back data structure to get service from application platform in terms of callback functions to application. These callbacks are grouped as different structures such as OS interface callbacks, Communication interface callbacks, Processor/core interface callbacks, Debug interface callbacks and Device platform details. Application needs to define these callbacks and initialize the client call back data structure.

Once above data structure is populated, the application shall call \ref rl_mmWaveLinkInit API function to initialize the mmwaveLink SW layer. This is the first function call to mmWave DFP to initialize the DFP SW. The \ref rl_mmWaveLinkDeInit API function call shall be used to uninitialize the mmWaveLink.

Refer below section for more details on mmWave DFP interface frame work data structures and MACROs.
 - \ref MMWAVE_DFP_INTERFACE
.

 \subsection mmwl_porting_subsec mmWaveLink Application Porting Guide
The porting of the mmWaveLink library framework to any new application platform is based on few simple steps. This guide takes you through this process step by step. Please follow the instructions carefully to avoid any problems during this process and to enable efficient and proper
work with the device. \n
Please notice that all modifications and porting adjustments of the callback functions should be made in the application only and the mmWave DFP API functions should not be modified. The changes in the application file will ensure smoothly transaction to new versions of the mmWave DFP libraries in the future!

### Step1: Update Build time MACROs
The build time MACROs are settings shall be configured during the library build time, in case user wants to update these settings, it can be updated directly in dfp_users.h file or it can be updated part of build time option in 'dfp_build.bat'.

The default settings are as below and which supports xWRL6432/xWRL1432 TI low power mmWave radar SOC with single device support and Debug log function is disabled.

 @code
 #define M_RL_DISABLE_LOGGING                (1U)
 #define M_RL_DEVICE_PLATFORM_TYPE           (M_DFP_DEVICE_PLATFORM_SOC)
 #define M_RL_DEVICE_MAX_DEVICES             (1U)
 #define M_DFP_BIG_ENDIAN                    (0U)
 @endcode

Refer to DFP Interface Build time MACROs in \ref MMWAVE_DFP_INTERFACE for more details on all setting options. \n

If any change in build time MACROs then build the library using dfp_build.bat file and link the latest library in application.

Refer below modules for mmWave DFP related standard data types and MACROs
- \ref MMWAVE_DFP_DATA : mmWave DFP standard data types
- \ref MMWAVE_DFP_DEBUG : mmWave DFP Debug function MACROs
.

### Step2: mmWaveLink Initialization
The application shall initialize the mmWaveLink SW framework before invoking the mmWaveLink APIs for a device. The \ref rl_mmWaveLinkInit function shall be used to initialize the mmwaveLink, the following are the passing parameters for the function.
-# w_deviceMap: The device map mask of type \ref UINT32 for mmwaveLink SW framework initialization, for single SOC type device, this value shall be 0x00000001.
-# z_clientCbData: The client callback data structure of type \ref T_DFP_CLIENT_CB_DATA, this data structure holds all application platform related callback function pointers and other device related information. \n
.
The typical declaration of the function parameters as below.
 @code
   UINT32 w_deviceMap = (UINT32)M_DFP_DEVICE_MAP_DEVICE_0;
   T_DFP_CLIENT_CB_DATA z_clientCbData = {0};
   // z_clientCbData parameter initialization (step 3 to step 7)
   rl_mmWaveLinkInit(w_deviceMap, z_clientCbData);
 @endcode

### Step3: Device Platform Initialization
Initialize Device platform related info in client callback data structure as below
 @code
   z_clientCbData.c_PlatformType     = M_DFP_DEVICE_PLATFORM_TYPE;
   z_clientCbData.c_DeviceType       = M_DFP_DEVICETYPE_6432;
   z_clientCbData.c_ApiErrorCheckEn  = M_ENABLE;
   z_clientCbData.h_ApiRespTimeoutMs = (UINT16)10U;
 @endcode

Refer \ref T_DFP_CLIENT_CB_DATA for more details.

### Step4: OS Interface Callback Initialization
The mmWaveLink and FECSSLib drivers can work in both OS and NonOS environment. If Application prefers to use operating system, it needs to implement basic OS routines such as tasks, mutex and Semaphore. In Case of Non-OS environment application needs to implement equivalent form of task (interrupt routine), mutex & semaphore. \n
Initialize application OS related info in client callback data structure as below
 @code
    z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexCreate = &Plt_osiLockObjCreate;
    z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexLock   = &Plt_osiLockObjLock;
    z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexUnLock = &Plt_osiLockObjUnlock;
    z_clientCbData.z_OsiCb.z_Mutex.p_OsiMutexDelete = &Plt_osiLockObjDelete;

    z_clientCbData.z_OsiCb.z_Semp.p_OsiSemCreate = &Plt_osiSyncObjCreate;
    z_clientCbData.z_OsiCb.z_Semp.p_OsiSemWait   = &Plt_osiSyncObjWait;
    z_clientCbData.z_OsiCb.z_Semp.p_OsiSemSignal = &Plt_osiSyncObjSignal;
    z_clientCbData.z_OsiCb.z_Semp.p_OsiSemDelete = &Plt_osiSyncObjDelete;

    z_clientCbData.z_OsiCb.z_Queue.p_OsiSpawn = &Plt_osiSpawn;
 @endcode

Refer \ref T_DFP_OSI_CB_DATA for more details.

### Step5: Processor Platform Interface Callback Initialization
The mmWaveLink and FECSSLib drivers need callback service from application processor platform to service interrupts, delay for wait, time stamp for profiling and perform tasks in atomic region. \n
Initialize application Platform related info in client callback data structure as below
 @code
    z_clientCbData.z_PltfCb.p_Delay               = &Plt_coreDelay;
    z_clientCbData.z_PltfCb.p_TimeStamp           = &Plt_coreTimerCount;
    z_clientCbData.z_PltfCb.p_RegisterIsr         = &Plt_coreRegisterIsr;
    z_clientCbData.z_PltfCb.p_DeRegisterIsr       = &Plt_coreDeRegisterIsr;
    z_clientCbData.z_PltfCb.p_MaskIsr             = &Plt_coreMaskIsr;
    z_clientCbData.z_PltfCb.p_UnMaskIsr           = &Plt_coreUnMaskIsr;
    z_clientCbData.z_PltfCb.p_EnterCriticalRegion = &Plt_coreEnterCriticalRegion;
    z_clientCbData.z_PltfCb.p_ExitCriticalRegion  = &Plt_coreExitCriticalRegion;
 @endcode

Refer \ref T_DFP_PLATFORM_CB_DATA for more details.

### Step6: Application Debug Interface Callback Initialization
The mmWaveLink and FECSSLib drivers can print debug info if M_RL_DISABLE_LOGGING = 0 in debug build option, DFP need print callback service from application to print the logs. In release build option debug prints are not enabled. Refer module \ref MMWAVE_DFP_DEBUG for more info on debug print functions. \n
Initialize application Platform related info in client callback data structure as below
 @code
    z_clientCbData.z_DbgCb.p_Print  = &Plt_dbgPrint;
 @endcode

Refer \ref T_DFP_DBG_PRINT_CB_DATA for more details.

### Step7: Communication Interface Callback Initialization (Device Dependent)
The SPI communication interface is needed only in \ref M_DFP_DEVICE_PLATFORM_FE type of mmWave sensor (xWRL6131), in \ref M_DFP_DEVICE_PLATFORM_SOC type (xWRL6432/xWRL1432), these callback drivers can be initialized to NULL.  \n
Initialize communication Interface related info in client callback data structure as below
 @code
    z_clientCbData.z_ComIfCb.p_ComIfOpen   = &Plt_spiOpen;
    z_clientCbData.z_ComIfCb.p_ComIfRead   = &Plt_spiRead;
    z_clientCbData.z_ComIfCb.p_ComIfWrite  = &Plt_spiWrite;
    z_clientCbData.z_ComIfCb.p_ComIfClose  = &Plt_spiClose;
 @endcode

Refer \ref T_DFP_COMIF_CB_DATA for more details.

 \section mmwl_api_sec mmWaveLink API details
mmWaveLink API functions provides seamless API interface to mmWave Sensor FECSS Front End. mmWaveLink Library is a high level software API layer which interface with FECSS Library, FECSSLib provides low level driver functions to configure/control FECSS hardware. mmWaveLink API internally calls FECSSLib drivers to perform the API job and returns the status to application. \n
The mmWaveLink APIs are divided into three major API modules based on functionality as below, refer individual module section for more details on APIs.
-# \ref mmwl_Device_sec : The Device module consist of below major mmWaveLink APIs
  - mmWaveLink Framework Init / DeInit
  - FECSS Device and RF Power ON/OFF
  - DFP Version Get
  - Front End Clock Configuration
  - Calibrations
  - Device Temperature Measurement
  - Device GPADC Signal Measurement
-# \ref mmwl_Sensor_sec : The Sensor module consist of below major mmWaveLink APIs
  - Chirp profile Configuration
  - Chirp Power Save Control
  - Per Chirp Configuration
  - Frame / Burst Configuration
  - Sensor Start / Stop
  - CW Mode
  - Test Source and RF Loopback Options
-# \ref mmwl_Monitor_sec : The Monitor module consist of below major mmWaveLink APIs, these monitors are available only in Functional Safety RTM releases.
  - Clock Monitor
  - APLL & SYNTH Monitor
  - IF Filter Monitor
  - RX Gain Phase Monitor
  - TX Gain Phase Monitor
  - TX Power Monitor
  - TX Ballbreak Monitor
  - RX Saturation Live monitor
  - Synth Frequency Monitor
  - Internal Signal DCBIST Monitor
  - FECSS Digital monitors
.
Refer below section for more details
 - \ref MMWL_DEVICE_API
 .

 \subsection mmwl_Notes_subsec General Guidelines and Notes
 This section captures all the general guidelines and notes related to mmWaveLink APIs
  \note 1: Please refer latest mmWave DFP 3.x release notes for all known issues and supported
          APIs. \n
  \note 2: All reserved bits/bytes in API configuration fields in commands shall be programmed with value zero. The functionality of radar device is not guaranteed if reserved bytes are not zero. \n
  \note 3: All reserved bits/bytes in API message response shall be masked off or discarded. \n

 \section mmwl_build_sec mmWaveLink Library Build Options
The mmWaveLink library can be built using make files and batch files provided by this package. The 'build' directory in 'mmwave-dfp-low/mmwavelink' parent directory includes 'rl_build.bat' batch file and make files for the library generation for mmwaveLink componets. \n
The 'rl_setenv.bat' file in 'build' directory shall be updated to select below paths before building the library:
 - Path to TI arm compiler CG_TOOLS version '20.2.3.LTS'
 - Path to make.exe file version 'gmake 4.1'
.
The below commands can be used to perform the build, run this from 'build' directory. Refer build help for more details on build options.
 - rl_build.bat help
 - rl_build.bat lib_m4_release 6432
 - rl_build.bat lib_m4_debug 1432
 .

Link to Parent Directory: \ref MMWAVELINK_API

 @addtogroup MMWAVELINK_API mmWaveLink API functions
 @{
 @brief Below sections provides information about mmWaveLink Library API functions, data structure and MACROs.
********************************************************************************
*/

/*!
 * @defgroup MMWL_DEVICE_API mmWaveLink Device Control API functions
 * @brief mmWaveLink Device Control API functions and Data Structures
 * @ingroup MMWAVELINK_API
 */

/*!
 * @defgroup MMWL_SENSOR_API mmWaveLink Sensor Control API functions
 * @brief mmWaveLink Sensor Control API functions and Data Structures
 * @ingroup MMWAVELINK_API
 */

/*!
 * @defgroup MMWL_MONITOR_API mmWaveLink Monitor Control API functions
 * @brief mmWaveLink Monitor Control API functions and Data Structures
 * @ingroup MMWAVELINK_API
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <fecsslib/fecsslib.h>

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Framework Interface MACROs
 * @{
 */

/*!
 * @brief mmWaveLink Library Version
 */
#define M_RL_MMWAVELINK_VERSION              "3.2.3.0.09.02.24"
#define M_RL_MMWAVELINK_VERSION_GEN          (3U)
#define M_RL_MMWAVELINK_VERSION_MAJOR        (2U)
#define M_RL_MMWAVELINK_VERSION_MINOR        (3U)
#define M_RL_MMWAVELINK_VERSION_BUILD        (0U)
#define M_RL_MMWAVELINK_VERSION_DAY          (9U)
#define M_RL_MMWAVELINK_VERSION_MONTH        (2U)
#define M_RL_MMWAVELINK_VERSION_YEAR         (24U)

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name mmWaveLink Library Framework Interface Type defines
 * @{
 */


/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Framework Interface global Variables
 * @{
 */

/*! @} */
/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library API Interface function calls
 * @{
 */
M_LIB_EXPORT T_RETURNTYPE rl_mmWaveLinkInit(UINT32 w_deviceMap, \
                    T_DFP_CLIENT_CB_DATA z_clientCbData);
M_LIB_EXPORT T_RETURNTYPE rl_mmWaveLinkDeInit(UINT32 w_deviceMap);

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


