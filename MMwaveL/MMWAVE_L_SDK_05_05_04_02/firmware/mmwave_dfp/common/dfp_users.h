/*!*****************************************************************************
 * @file dfp_users.h
 *
 * @brief DFP users application interface header file
 *
 * @b Description @n
 * This is a mmWaveLink/FECSS user application interface header file, provides information
 * about build time MACROs, user can modify these settings and rebuild the mmWaveLink
 * and FECSS libraries. Refer below header files for more details on interface
 *  - # mmwave-dfp-low/mmwavelink/mmwavelink.h - mmwavelink interface file
 *  - # mmwave-dfp-low/fecsslib/fecsslib.h - FECSSLIB interface file
 *
 * @warning Application developer / User shall review and update the build time
 * MACROs and rebuild the libraries as needed.
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
#ifndef DFP_USERS_H
#define DFP_USERS_H

/*!*****************************************************************************
 @mainpage mmWave DFP 3.x (Device Firmware Package)

 \section dfp_intro_sec Introduction

mmWave DFP 3.x is a Device Firmware Package for TI’s low end platform mmWave RF transceiver devices. mmWave DFP is collection of foundational API software, firmware, drivers, and ROM components associated with mmWave devices. mmWave DFP provides seamless control and configuration of Radar Frontend Controller Sub System (FECSS) in real-time either from an external host device in case of Front End (FE) mmWave sensors or from integrated M4 APPSS core in case of SOC. \n

The mmWave DFP is responsible to provide API function calls to all RF front end related functionalities and configurations; using these APIs application can calibrate the RF device, generate radar waveforms and get functional safety monitoring results from FECSS. These APIs enables the RF transceiver to be self-contained and capable of adapting itself to dynamic conditions, such as temperature changes, functional safety capable and minimize the RF related knowledge/dependencies and algoritms run part of application software. \n

TI low cost and low power, low end mmWave radar products are highly-integrated 60GHz and 77GHz CMOS millimeter wave devices for Automotive and Industrial applications. The device integrated with all of the RF and Analog functionality, including VCO, PLL, PA, LNA, Mixer and ADC for multiple TX/RX channels into a single chip in a low power digital architecture. \n
 -# The xWRL6432 is a 60GHz mmwave radar-on-a-chip (SOC) device, which includes 3 receive channels
and 2 transmit channels and additionally an Cortex M4F and hardware FFT accelerator. \n
 -# The xWRL1432 is a 77GHz mmwave radar-on-a-chip (SOC) device, which includes 3 receive channels
and 2 transmit channels and additionally an Cortex M4F and hardware FFT accelerator. \n
.
The mmWave DFP 3.x consist of 3 major components, these components closely associated with the functionality of underlying device sensor front-end hardware IPs and implementation. \n
 -# FECSS Library (FECSSLib)
 -# RF Scripter (RFS) firmware
 -# mmWaveLink Library
| Component       | Description |
| --------------- | ---------- |
| FECSS Library   | The FECSS open-source ROM/RAM drivers library is responsible for management and operation of the Radar sensor frontend functions and waveform generation in the device. The mWaveLink APIs internally call these FECSSLib driver APIs to configure the FECSS  |
| RFS             | The ROM/patch binary is responsible for executing pre defined TI device specific RF analog sequences, the dedicated RF scripter M3 core runs these sequence based on commands from FECSS library or mmWaveLink. The patch file is loaded in the device patch RAM during boot which is used to implement bug fixes and minor feature additions in RF Scriptor ROM  |
| mmWaveLink      | This is an open-source, OS and device agnostic API functions for user control  and configuration of the RF analog FECSS in the device.  The mmwaveLink is the user communication API to the FECSS. The mmWaveLink layer can be integrated with SDK(SoC)/External Host (Frontend) |

## mmWave DFP Block Diagram
 @image html dfp_arch_3x.png "mmWave DFP Architecture Block Diagram"

## Supported mmWave Sensor Devices
| Device        | RF Frequency |
| ------------- | ----------   |
| xWRL6432 SOC  | 60GHz        |
| xWRL1432 SOC  | 77GHz        |

 \section dfp_components_sec DFP Components

 \subsection dfp_fecssLib_subsec FECSS Library

TI low end mmWave radar device includes a radar front end or FECSS (Radar Frontend ControllerSub System), which  is responsible for configuring the RF/Analog and digital front-end for sensor functionality for waveform generation, calibration and functional safety monitoring based on driver API calls. FECSSLib is responsible to provide these driver API calls to mmWaveLink. \n

TI FECSS is a open subsystem whose digital blocks can be configurable using FECSSLib drivers. FECSS has a closed RF Front End Scripter M3 core (RFS), which is responsible to run Analog Front End (AFE) sequences based on commands from FECSS library. mmWaveLink APIs calls FECSSLib drivers to configure the FECSS.

### FECSS Library Block Diagram
 @image html FecssLib.png "mmWave FECSSLib Block Diagram"

### FECSS Library ROM/Patch Options
The FECSS drivers are open source software, TI is providing option either to use ROM/patch library driver functions or RAM functions in library generation build options. In ES1.0 mmwave silicon, the FECSSLib ROM is not available and user has to link 'fecss_rom_m4.lib' by default and it is a static RAM library. Refer \ref fecsslib_build_sec for more details on ROM and patch library build options.

### FECSS Library Driver APIs
The FECSS library provides multiple low level driver APIs to configure/control the FECSS. Refer below section to know more about FECSSLib driver modules and APIs.
 - \ref fecsslib_api_sec
.
Refer below sections for more details on FECSSLib
 - \ref fecsslib_interface_sec
 - \ref fecsslib_build_sec
.
 \subsection dfp_rfs_subsec RFS Firmware
RF Front End Scripter (RFS) is a closed subsystem, which has cortex M3 core responsible to run set of analog and digital configuration sequences and algoritms related to analog power domains, clocks, runtime sensor configurations, calibrations and functional analog safety monitors. \n
RFS has a command interface mailbox for IPC, a standard TI internal protocol is used to communicate with RFS from FECSSLib. The RFS firmware algorithms are functional safety compliance TI proprietary libraries and TI provides binary image part of this DFP package. The 'rfsfirmware' folder in DFP package contains this binary image.

### RFS Block Diagram
 @image html RFS.png "mmWave RF Front End Scripter Block Diagram"

### RFS - FECSSLib IPC
A mailbox IPC interface is used to communicate between RFS and FECSSLib. The FECSS library provides multiple low level driver APIs to construct and invoke RFS commands. Refer below section to know more about RFS commands and protocol details.
 - \ref fecsslib_rfs_sec
.
 \subsection dfp_mmwl_subsec mmWaveLink

mmWaveLink (also called RadarLink) Library is a top most layer in DFP, which provides software abstraction API function calls to FECSS. mmWaveLink 3.x APIs are a simple wrapper API functions, which calls FECSSLib driver api functions to service the API called by application. \n

Highlights of mmWaveLink Library Framework:
 - A link between application and FECSS.
 - Provides low level APIs to configure the FECSS and handles the communication with the front
   end.
 - Platform and OS independent which means it can be ported into any processor which provides
   communication interface such as SPI/Register and basic OS/non-OS routines. The mmWaveLink framework can also run in single and multi threaded environment.
 - Can be integrated into TI mmWave SDK or customer application and runs on APPSS cortex M4F   processor in xWR6432 and xWR1432 SOC devices.
 - It can run on any external HOST processor which controls TI mmWave Front End (FE) devices (xWR6131 FE device).

### mmWaveLink Library Block Diagram
 @image html mmwavelink.png "mmWaveLink 3.x Library Block Diagram"

### mmWaveLink APIs
The mmWaveLink library provides multiple high level SW APIs to configure/control the FECSS. Refer below section to know more about mmWaveLink API modules and SW APIs.
 - \ref mmwl_api_sec
.
Refer below section for more details on mmWaveLink Integration and APIs
 - \ref mmwl_interface_sec
 - \ref mmwl_build_sec
.
 \section dfp_interface_sec DFP Interface and Porting
mmWaveLink, which is the top most layer of mmWave DFP provides interface to application. The interface frame work is common to both mmWaveLink and FECSSLib. The mmwaveLink internally interface with FECSSLib and user is not required to perform the same. \n
The application interface with mmWaveLink is performed using below functions, refer \ref mmwl_interface_sec for more details on interface and porting of mmWaveLink.
- \ref rl_mmWaveLinkInit API function
- \ref rl_mmWaveLinkDeInit API function
- \ref T_DFP_CLIENT_CB_DATA client call back data structure
.
Refer below section for more details on mmWave DFP interface frame work data structures and MACROs.
 - \ref MMWAVE_DFP_INTERFACE
.
 \note Refer \ref fecsslib_interface_sec for internal FECSSLib interface details. This is for information purpose only, user is not required to perform this job as it is done part of DFP.

 \section dfp_distr_sec DFP Deployment Model
The mmWave DFP 3.x can be deployed in following models depends on mmWave sensor device type.
 - Model 1: Standalone single chip sensor Model
 - Model 2: Front end sensor Model
 - Model 3: Single chip with Host control sensor Model

 ### Model 1:
 @image html Singlechip.png "Model 1: Standalone single chip sensor Model"

 ### Model 2:
 @image html frontend.png "Model 2: Front end sensor Model"

 ### Model 3:
 @image html Singlechip-HW-trigger.png "Model 3: Single chip with Host control sensor Model"

 \section dev_ref_clk Device Reference Clock
 The few sensor API parameters are highly dependent on device reference XTAL clock and APLL root
 clock frequency. The xWRL6432/1432 devices support following XTAL sources. The Variable Name
 XTAL_FREQ, APLL_FREQ and DIG_PLL_FREQ are being used in various API parameters 1LSB computation.

 |   | XTAL_FREQ (MHz) | APLL_FREQ (Analog) (MHz) | DIG_PLL_FREQ (Digital) (MHz) |
 |---|-----------------|--------------------------|------------------------------|
 | 1 | 25.0            | 400.0                    | 160.0                        |
 | 2 | 26.0            | 403.0                    | 161.2                        |
 | 3 | 38.4            | 403.2                    | 161.28                       |
 | 4 | 40.0            | 400.0                    | 160.0                        |

 \section mmWaveLink API Documentation
  [mmWaveLink 3.x API Documentation](@ref MMWAVE_LINK_DOC)

 \section dfp_folder_sec "mmwave-dfp-low" Folder Structure

 The mmwave-dfp-low directory structure is as below:
 <PRE>
 V <span style="color: black"><B>mmwave-dfp-low</B> . . . . . . . . .(Parent Directory)</span>
   > <span style="color: red">bin . . . . . . . . . . . . . (output bin folder)</span>
   V <span style="color: blue">build . . . . . . . . . . . . (Build directory for DFP</span>)
     <span style="color: green"><B>dfp_build.bat . . . . . . . . (Script to build libraries)</B></span>
   V <span style="color: blue">common . . . . . . . . . . . .(Common directory for DFP</span>)
     <span style="color: green"><B>dfp_datatypes.h . . . . . . . (DFP datatypes)</B></span>
     <span style="color: green"><B>dfp_trace.h . . . . . . . . . (DFP debug trace)</B></span>
     <span style="color: green"><B>dfp_users.h . . . . . . . . . (DFP Interface)</B></span>
   V <span style="color: blue">docs/doxygen  . . . . . . . . (Documentation)</span>
     > <span style="color: blue">images</span>
     > <span style="color: blue">scripts</span>
     > <span style="color: blue">source</span>
     <span style="color: green"><B>dfp_doxygen.bat</B> . . . . . . . (Script to generate documentation)</span>
   > <span style="color: blue">examples . . . . . . . . . . .(DFP Examples)</span>
   V <span style="color: blue"><B>fecsslib</B> . . . . . . . . . . .(FECSSLib source directory)</span>
     > <span style="color: red">bin </span>
     V <span style="color: blue">build </span>
       <span style="color: green"><B>fe_build.bat</B></span>
     > <span style="color: blue">ccs </span>
     > <span style="color: blue">common </span>
     > <span style="color: blue">device. . . . . . . . . . . (Module directory)</span>
     > <span style="color: blue">docs </span>
     > <span style="color: blue">lib. . . . . . . . . . . . .(Library directory)</span>
     > <span style="color: blue">rfscripter </span>
     > <span style="color: blue">sensor </span>
     <span style="color: green"><B>fecsslib.h</B> . . . . . . . . . .(FECSSLib Interface)</span>
     <span style="color: green"><B>rommap.h</B> . . . . . . . . . . .(FECSSLib ROM/PATCH map)</span>
   V <span style="color: blue"><B>mmwavelink</B> . . . . . . . . . .(mmWaveLink source directory)</span>
     > <span style="color: red">bin </span>
     V <span style="color: blue">build </span>
       <span style="color: green"><B>rl_build.bat</B></span>
     > <span style="color: blue">ccs </span>
     > <span style="color: blue">docs </span>
     > <span style="color: blue">include . . . . . . . . . . (Include directory)</span>
     > <span style="color: blue">lib. . . . . . . . . . . . .(Library directory)</span>
     > <span style="color: blue">source . . . . . . . . . . .(Source directory)</span>
     > <span style="color: blue">test . . . . . . . . . . . .(Test directory)</span>
     <span style="color: green"><B>mmwavelink.h</B> . . . . . . . . .(mmWaveLink Interface)</span>

Each source module directory follow below structure:
V <span style="color: black"><B>module</B> . . . . . . . . . . . . (Module Directory)</span>
  V <span style="color: blue">include . . . . . . . . . . . .(Include directory)</span>
    <span style="color: green">module_file_A.h</span>
    <span style="color: green">module_file_B.h</span>
  V <span style="color: blue">source . . . . . . . . . . . . (Source directory)</span>
    <span style="color: green">module_file_A.c</span>
    <span style="color: green">module_file_B.c</span>
  V <span style="color: blue">test . . . . . . . . . . . . . (Test directory)</span>
    V <span style="color: blue">include </span>
      <span style="color: green">module_file_A.h</span>
      <span style="color: green">module_file_B.h</span>
    V <span style="color: blue">source </span>
      <span style="color: green">module_file_A.c</span>
      <span style="color: green">module_file_B.c</span>
 </PRE>

 \section dfp_build_sec Build Options
The mmWave DFP mmWaveLink and FECSSLib libraries can be built using make files and batch files provided by this package. The 'build' directory in 'mmwave-dfp-low' parent directory includes 'dfp_build.bat' batch file and make files for the library generation for both mmwaveLink and FECSSLib componets. \n
The below commands can be used to perform the build, run this from 'build' directory. Refer build help for more details on build options.
 ```shell
 dfp_build.bat help
 dfp_build.bat dfp_romlib_m4_release Lx432
 dfp_build.bat dfp_romlib_m4_debug Lx432
 ```

refer \ref mmwl_build_sec and \ref fecsslib_build_sec for more details on individual library build options.

 \section dfp_example_sec DFP Examples
 The DFP examples are the simple reference functions which demonstrates sequence of mmWaveLink APIs to be issued in
 a particular scenario. These examples are @b NOT standalone must be integrated in an application environment.
 Please refer following section for more details about the examples:
 @ref MMWAVE_DFP_EXAMPLES

 @addtogroup MMWAVE_DFP_INTERFACE mmWave DFP Interface Framework
 @{
 @brief Below sections provides information about mmWave DFP mmWaveLink and FECSS Library interface Framework data structures, MACROs and type defines.
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
 * @name DFP Interface Build time MACROs
 * @brief User needs to set these MACROs and rebuild this library or these MACROs can be
 * defined as a compiler option without amending the source code.
 * @{
 */

/*!
 * @brief Export Macro for windows DLL creation
 */
#if defined(WIN32) || defined(WIN32_) || defined(_MSC_VER)
#define M_LIB_EXPORT __declspec(dllexport)
#else
#define M_LIB_EXPORT
#endif

/*!
 * @brief DFP BUILD TIME MACRO to enable/disable logging.
 * To enable logging set this MACRO to '0' and set proper function pointer
 * z_DbgCb.p_Print in T_DFP_CLIENT_CB_DATA data structure during rl_mmWaveLinkInit() call. \n
 * By default this MACRO set to 1, all test logs are not supported by default
 */
#ifndef M_DFP_DISABLE_LOGGING
#define M_DFP_DISABLE_LOGGING               (1U)
#endif

#define M_DFP_DEVICE_PLATFORM_SOC           (1U)
#define M_DFP_DEVICE_PLATFORM_FE            (2U)

/*!
 * @brief DFP BUILD TIME MACRO to select Device platform.
 * - # value 1 builds mmWave DFP for SOC platform (xWRL6432/1432) - M_DFP_DEVICE_PLATFORM_SOC
 * - # value 2 builds mmWave DFP for FE platform (xWRL6131) - M_DFP_DEVICE_PLATFORM_FE \n
 * . By default MACRO set to M_DFP_DEVICE_PLATFORM_SOC type
 */
#ifndef M_DFP_DEVICE_PLATFORM_TYPE
#define M_DFP_DEVICE_PLATFORM_TYPE          (M_DFP_DEVICE_PLATFORM_SOC)
#endif

/*!
 * @brief : mmWave DFP BUILD TIME MACRO to select number of connected
 * mmWave sensor devices. This is applicable mostly in case of Front End cascade devices. \n
 * In case of cascade FE setup, configuring multiple devices may be time consuming. So to save the
 * configuration time Host needs to follow below steps:
 * -# set this MACRO to the number of cascade chips
 * -# Create multiple tasks for each of the devices.
 * -# Invoke same config API from each task but with matching device-index. \n
 *.
 * Set to '1' if number of device is one or device is SOC type. \n
 * Set to 'N' if number of cascade devices are N and device is FE type. \n
 * By default MACRO set to 1 for M_DFP_DEVICE_PLATFORM_SOC type
 */
#ifndef M_DFP_MAX_DEVICES
#define M_DFP_MAX_DEVICES                     ((UINT8)1U)    /* Max Devices */
#endif

/*!
 * @brief : mmWave DFP BUILD TIME MACRO to select number of connected
 * mmWave sensor devices. Set max device map mask as per M_DFP_MAX_DEVICES setting
 * - Mask 0x1 for M_DFP_MAX_DEVICES = 1
 * - Mask 0x3 for M_DFP_MAX_DEVICES = 2
 * - Mask 0x7 for M_DFP_MAX_DEVICES = 3
 * - Mask 0xF for M_DFP_MAX_DEVICES = 4
 * . This MACRO is automatically set based on M_DFP_MAX_DEVICES
 */
#define M_DFP_MAX_DEV_MAP_MASK                (((UINT32)2^((UINT32)M_DFP_MAX_DEVICES)) - (UINT32)1U)

/*!
 * @brief : mmWave DFP BUILD TIME MACRO to select Platform endianess,
 * The mmWave DFP drivers by default is enabled for Little Endian host. Support for
 * Big Endian is provided as compile time option using a Pre-processor Macro M_DFP_BIG_ENDIAN.
 * The RFS M3 core is a little endian core, the FECSSLib swaps the RFS command data structure
 * before sending command to RFS. \n
 * By default MACRO set to 0 for little endian platforms
 */
#ifndef M_DFP_BIG_ENDIAN
#define M_DFP_BIG_ENDIAN                      (0U)    /* Endian type */
#endif

/*! @} */

/*!
 * @name DFP Interface MACROs
 * @{
 */

/*!
 * @brief : mmWaveLink FECSS device map bits, Maximum 4 parallel devices
 * are supported in cascade mode in this mmWaveLink, this can be extended if
 * needed.
 * - # Device id 0 : Bit 0
 * - # Device id 1 : Bit 1
 * - # Device id 2 : Bit 2
 * - # Device id 3 : Bit 3
 */
#define M_DFP_DEVICE_MAP_DEVICE_0             ((UINT32)1U)    /* Device Id 0 */
#define M_DFP_DEVICE_MAP_DEVICE_1             ((UINT32)2U)    /* Device Id 1 */
#define M_DFP_DEVICE_MAP_DEVICE_2             ((UINT32)4U)    /* Device Id 2 */
#define M_DFP_DEVICE_MAP_DEVICE_3             ((UINT32)8U)    /* Device Id 3 */

/*!
 * @brief : mmWaveLink FECSS device index, Maximum 4 parallel devices
 * are supported in cascade mode in this mmWaveLink, this can be extended if
 * needed.
 * - # Device id 0
 * - # Device id 1
 * - # Device id 2
 * - # Device id 3
 */
#define M_DFP_DEVICE_INDEX_0                  ((UINT8)0U)    /* Device Id 0 */
#define M_DFP_DEVICE_INDEX_1                  ((UINT8)1U)    /* Device Id 1 */
#define M_DFP_DEVICE_INDEX_2                  ((UINT8)2U)    /* Device Id 2 */
#define M_DFP_DEVICE_INDEX_3                  ((UINT8)3U)    /* Device Id 3 */

/*!
 * @brief
 * mmWave Device Types:
 * - # xWR6432 : 0
 * - # xWR1432 : 1
 * - # xWR6131 : 2
 */
#define M_DFP_DEVICETYPE_6432                 ((UINT8)0x0U)  /* Device 6432 */
#define M_DFP_DEVICETYPE_1432                 ((UINT8)0x1U)  /* Device 1432 */
#define M_DFP_DEVICETYPE_6131                 ((UINT8)0x2U)  /* Device 6131 */

/*!
 * @brief FECSS DFP Max RX channels supported. \n
 * In xWRL6432 and xWRL1432 devices only 3 RX channels are supported.
 */
#define M_DFP_MAX_RX_CHANNELS                 ((UINT8)4U)

/*!
 * @brief FECSS DFP Max TX channels supported. \n
 * In xWRL6432 and xWRL1432 devices only 2 TX channels are supported.
 */
#define M_DFP_MAX_TX_CHANNELS                 ((UINT8)4U)

/*!
 * @brief FECSS DFP Max Cal Temp Bins. \n
 */
#define M_DFP_MAX_CAL_TEMP_BINS               ((UINT8)3U)
/*!
 * @brief FECSS FE MB interrupt mapping for devices
 */
#define M_DFP_FE_DEV_INTR0_RFS_MB_RSP(c_devInd)     (c_devInd)

/*! @} */

/*!
 * @name mmWaveLink API Error Codes
 * @{
 */

/*!
 * @name DFP FECSSLib and mmWaveLink Integration Error Codes.
 * - The error code range -1 to -999 used by mmWaveLink and FECSSLib
 * - The error code range -1000 to -1999 used by RFS subsystem
 * - Range: -1 to -100   : General error codes
 * - Range: -101 to -300 : Device Module error codes
 * - Range: -301 to -500 : Sensor Module error codes
 * - Range: -501 to -700 : Monitor Module error codes
 * - Range: -701 to -999 : Reserved
 * @{
 */
#define M_DFP_RET_CODE_OK                       ((SINT32)0) /*!< no-error */
#define M_DFP_OSI_RET_CODE_OK                   (M_DFP_RET_CODE_OK)
#define M_DFP_CIF_RET_CODE_OK                   (M_DFP_RET_CODE_OK)
#define M_DFP_RET_CODE_INVALID_INPUT            ((SINT32)-1) /*!< Invalid input - General */
#define M_DFP_RET_CODE_RESP_TIMEOUT             ((SINT32)-2) /*!< API Response timeout */
#define M_DFP_RET_CODE_FATAL_ERROR              ((SINT32)-3) /*!< Fatal error in DFP */
#define M_DFP_RET_CODE_RADAR_OSIF_ERROR         ((SINT32)-4) /*!< OS Interface failure */
#define M_DFP_RET_CODE_INVALID_API              ((SINT32)-5) /*!< Invalid API call */
#define M_DFP_RET_CODE_NULL_PTR_ERROR           ((SINT32)-6) /*!< Null pointer error */
#define M_DFP_RET_CODE_INTERFACE_CB_NULL        ((SINT32)-7) /*!< Interface callback is NULL */
#define M_DFP_RET_CODE_INVALID_DEVICE           ((SINT32)-8) /*!< Invalid Device map/Id  */
#define M_DFP_RET_CODE_RADAR_PLTIF_ERROR        ((SINT32)-9) /*!< Platform Interface failure */
#define M_DFP_RET_CODE_RADAR_CIF_ERROR          ((SINT32)-10) /*!< Comm IF Interface failure */
#define M_DFP_RET_CODE_MMWL_INIT_NOT_DONE       ((SINT32)-11) /*!< mmWaveLink Init not done */
#define M_DFP_RET_CODE_RFS_DBG_BUF_CB_NULL      ((SINT32)-12) /*!< RFS debug buffer is NULL  */

/*! Device Module Error codes (Includes RfScripter error codes) */
#define M_DFP_RET_CODE_RFS_BOOT_ERROR           ((SINT32)-101) /*!< RFS boot failure */
#define M_DFP_RET_CODE_RFS_MB_INIT_ERROR        ((SINT32)-102) /*!< RFS MB Init failure */
#define M_DFP_RET_CODE_RFS_MB_BUSY_ERROR        ((SINT32)-103) /*!< RFS MB busy while sending cmd */
#define M_DFP_RET_CODE_RFS_MB_RESP_ERROR        ((SINT32)-104) /*!< RFS MB empty while reading res*/
#define M_DFP_RET_CODE_RFS_PROTOCOL_ERROR       ((SINT32)-105) /*!< RFS Protocol error */
#define M_DFP_RET_CODE_RFS_API_SIZE_ERR         ((SINT32)-106) /*!< RFS cmd/res API size error */
#define M_DFP_RET_CODE_DEVICE_NOT_POWERED       ((SINT32)-107) /*!< FECSS device is not powered up */
#define M_DFP_RET_CODE_REG_READBACK_ERROR       ((SINT32)-108) /*!< Register readback error  */
#define M_DFP_RET_CODE_RFS_RESP_TIMEOUT         ((SINT32)-109) /*!< RFS Response timeout */
#define M_DFP_RET_CODE_RFS_CMDID_MISMATCH       ((SINT32)-110) /*!< RFS cmd id mismatch */
#define M_DFP_RET_CODE_MEM_INIT_TIMEOUT         ((SINT32)-111) /*!< Device Mem init failure */
#define M_DFP_RET_CODE_RFS_BOOT_TIMEOUT         ((SINT32)-112) /*!< RFS boot timeout error */
#define M_DFP_RET_CODE_INVAL_DEV_PWR_MODE       ((SINT32)-113) /*!< Invalid Device power mode */
#define M_DFP_RET_CODE_INVAL_DEV_CLK_SRC        ((SINT32)-114) /*!< Invalid Device clock source */
#define M_DFP_RET_CODE_INVAL_FT_CLK_SRC         ((SINT32)-115) /*!< Invalid FT clock source */
#define M_DFP_RET_CODE_INVAL_XTAL_CLK_SRC       ((SINT32)-116) /*!< Invalid XTAL clock */
#define M_DFP_RET_CODE_INVAL_CT_RESOL           ((SINT32)-117) /*!< Invalid CT resolution */
#define M_DFP_RET_CODE_FEC_POWERUP_TIMEOUT      ((SINT32)-118) /*!< FECSS powerup timeout */
#define M_DFP_RET_CODE_FEC_PWRDOWN_TIMEOUT      ((SINT32)-119) /*!< FECSS power down timeout */
#define M_DFP_RET_CODE_INVAL_APLL_CLK_CTRL      ((SINT32)-120) /*!< Invalid APLL clock ctrl */


/*! Sensor Module Error codes */
#define M_DFP_RET_CODE_SENS_INVAL_SAMP_RATE     ((SINT32)-301) /*!< Invalid sample rate */
#define M_DFP_RET_CODE_SENS_INVAL_DIG_BITS      ((SINT32)-302) /*!< Invalid Digout bits */
#define M_DFP_RET_CODE_SENS_INVAL_DFEFIR_SEL    ((SINT32)-303) /*!< Invalid DFE FIR select */
#define M_DFP_RET_CODE_SENS_INVAL_VCO_MC_SEL    ((SINT32)-304) /*!< Invalid VCO Multichip select */
#define M_DFP_RET_CODE_SENS_INVAL_NUMADC_SAMP   ((SINT32)-305) /*!< Invalid num ADC samples */
#define M_DFP_RET_CODE_SENS_INVAL_TX_MIMO       ((SINT32)-306) /*!< Invalid TX MIMO pattern */
#define M_DFP_RET_CODE_SENS_INVAL_RAMPEND       ((SINT32)-307) /*!< Invalid rampend time */
#define M_DFP_RET_CODE_SENS_INVAL_HPF_SEL       ((SINT32)-308) /*!< Invalid HPF select */
#define M_DFP_RET_CODE_SENS_INVAL_RX_GAIN       ((SINT32)-309) /*!< Invalid RX gain select */
#define M_DFP_RET_CODE_SENS_INVAL_RF_GAIN       ((SINT32)-310) /*!< Invalid RF gain select */
#define M_DFP_RET_CODE_SENS_INVAL_TX_BO         ((SINT32)-311) /*!< Invalid TX Back off */
#define M_DFP_RET_CODE_SENS_INVAL_IDLE_TIME     ((SINT32)-312) /*!< Invalid idle time */
#define M_DFP_RET_CODE_SENS_INVAL_ADCFRAC_TIME  ((SINT32)-313) /*!< Invalid adc fract time */
#define M_DFP_RET_CODE_SENS_INVAL_ADCSKIP_SAMP  ((SINT32)-314) /*!< Invalid adc skip samples */
#define M_DFP_RET_CODE_SENS_INVAL_TXSTART_TIME  ((SINT32)-315) /*!< Invalid TX start time */
#define M_DFP_RET_CODE_SENS_INVAL_RF_SLOPE      ((SINT32)-316) /*!< Invalid RF slope */
#define M_DFP_RET_CODE_SENS_INVAL_RF_FREQ       ((SINT32)-317) /*!< Invalid RF Frequency */
#define M_DFP_RET_CODE_SENS_INVAL_TX_EN         ((SINT32)-318) /*!< Invalid TX enable */
#define M_DFP_RET_CODE_SENS_INVAL_TXBPM_EN      ((SINT32)-319) /*!< Invalid TX BPM enable */
#define M_DFP_RET_CODE_SENS_INVAL_NUM_CHIRPS    ((SINT32)-320) /*!< Invalid Num of chirps */
#define M_DFP_RET_CODE_SENS_INVAL_NUM_ACCUM     ((SINT32)-321) /*!< Invalid Num of accumulation */
#define M_DFP_RET_CODE_SENS_INVAL_BURST_PERD    ((SINT32)-322) /*!< Invalid Burst period */
#define M_DFP_RET_CODE_SENS_INVAL_NUM_BURST     ((SINT32)-323) /*!< Invalid Num of bursts */
#define M_DFP_RET_CODE_SENS_INVAL_FRAME_PERD    ((SINT32)-324) /*!< Invalid Frame period */
#define M_DFP_RET_CODE_SENS_INVAL_NUM_FRAME     ((SINT32)-325) /*!< Invalid num of frames */
#define M_DFP_RET_CODE_SENS_INVAL_FRAME_TRIG    ((SINT32)-326) /*!< Invalid frame trigger mode */
#define M_DFP_RET_CODE_SENS_INVAL_CT_START_LB   ((SINT32)-327) /*!< Invalid frame signal LB enable */
#define M_DFP_RET_CODE_SENS_INVAL_FRAME_STOP    ((SINT32)-328) /*!< Invalid frame stop */
#define M_DFP_RET_CODE_SENS_INVAL_PERCHRP_LEN   ((SINT32)-329) /*!< Invalid Per chirp array length */
#define M_DFP_RET_CODE_SENS_INVAL_PERCHRP_RPCNT ((SINT32)-330) /*!< Invalid per chirp repeat count */
#define M_DFP_RET_CODE_SENS_INVAL_PERCHRP_CTRL  ((SINT32)-331) /*!< Invalid per chirp control */
#define M_DFP_RET_CODE_SENS_INVAL_PERCHRP_ADDR  ((SINT32)-332) /*!< Invalid per chirp address */
#define M_DFP_RET_CODE_SENS_INVAL_HPF_FI_TIME   ((SINT32)-333) /*!< Invalid HPF Fast Init TIme */
#define M_DFP_RET_CODE_SENS_INVAL_CRD_NSLOPE    ((SINT32)-334) /*!< Invalid CRD Nslope */
#define M_DFP_RET_CODE_SENS_INVAL_LIVE_MON_CFG  ((SINT32)-335) /*!< Inavlid Live Mon cfg */
#define M_DFP_RET_CODE_SENS_INVAL_LB_FREQ       ((SINT32)-336) /*!< Inavlid LB freq */


/*! Monitor Module Error codes */
#define M_DFP_RET_CODE_MON_INVAL_SYNTH_START    ((SINT32)-501) /*!< Invalid SYNTH mon start time */
#define M_DFP_RET_CODE_MON_INVAL_SYNTH_THRSH    ((SINT32)-502) /*!< Invalid SYNTH mon threshold */
#define M_DFP_RET_CODE_MON_INVAL_PERCHRP_ADDR   ((SINT32)-503) /*!< Invalid Rx Sat mon result add */
#define M_DFP_RET_CODE_MON_INVAL_GPADC_INST     ((SINT32)-504) /*!< Invalid GPADC num instructions */
#define M_DFP_RET_CODE_MON_INVAL_TX_IND         ((SINT32)-505) /*!< Invalid TX index select */
/*! @} */

/*!
 * @name RFS Error Codes
 * @{
 */
#define M_DFP_RET_CODE_RFS_INVALID_CMDID                   ((SINT32)-1000)
#define M_DFP_RET_CODE_RFS_INVALID_MSG_LEN                 ((SINT32)-1002)
#define M_DFP_RET_CODE_RFS_INVALID_RX_MASK                 ((SINT32)-1003)
#define M_DFP_RET_CODE_RFS_INVALID_TX_MASK                 ((SINT32)-1004)
#define M_DFP_RET_CODE_RFS_INVALID_APLL_CTRL               ((SINT32)-1005)
#define M_DFP_RET_CODE_RFS_INVALID_RF_CTRL_OVERRIDE_MASK   ((SINT32)-1006)
#define M_DFP_RET_CODE_RFS_INVALID_CAL_TEMP_BIN            ((SINT32)-1007)
#define M_DFP_RET_CODE_RFS_INVALID_CAL_FREQ                ((SINT32)-1008)
#define M_DFP_RET_CODE_RFS_INVALID_CAL_FREQ_SLOPE          ((SINT32)-1009)
#define M_DFP_RET_CODE_RFS_INVALID_CAL_EN_MASK             ((SINT32)-1010)
#define M_DFP_RET_CODE_RFS_INVALID_CHIRP_PS_DIS_MASK       ((SINT32)-1011)
#define M_DFP_RET_CODE_RFS_INVALID_BURST_PS_DIS_MASK       ((SINT32)-1012)
#define M_DFP_RET_CODE_RFS_INVALID_RFS_SENS_START_MODE     ((SINT32)-1013)
#define M_DFP_RET_CODE_RFS_INVALID_RFS_SENS_STOP_MODE      ((SINT32)-1014)
#define M_DFP_RET_CODE_RFS_INVALID_SENS_LB_CTRL            ((SINT32)-1015)
#define M_DFP_RET_CODE_RFS_INVALID_SENS_LB_ENA             ((SINT32)-1016)
#define M_DFP_RET_CODE_RFS_GPADC_BUSY                      ((SINT32)-1017)
#define M_DFP_RET_CODE_RFS_INVALID_RDIF_ENA_CTRL           ((SINT32)-1018)
#define M_DFP_RET_CODE_RFS_INVALID_RDIF_SAMP_COUNT         ((SINT32)-1019)
#define M_DFP_RET_CODE_RFS_INVALID_ORBIT_RECORD            ((SINT32)-1020)
#define M_DFP_RET_CODE_RFS_APLL_ON_FAILED                  ((SINT32)-1021)
#define M_DFP_RET_CODE_RFS_APLL_OFF_IN_CAL                 ((SINT32)-1022)
#define M_DFP_RET_CODE_RFS_INVALID_TESTPATRN_ENA_CTRL      ((SINT32)-1023)
#define M_DFP_RET_CODE_RFS_INVALID_TEMP_MEAS_CHANNEL       ((SINT32)-1024)
#define M_DFP_RET_CODE_RFS_LOOPBACK_ACTIVE                 ((SINT32)-1025)
#define M_DFP_RET_CODE_RFS_GPADC_IFM_CONV_ERROR            ((SINT32)-1026)
#define M_DFP_RET_CODE_RFS_GPADC_IFM_INVAL_SAMPLS          ((SINT32)-1027)
#define M_DFP_RET_CODE_RFS_GPADC_IFM_INVAL_BUF_INDX        ((SINT32)-1028)
#define M_DFP_RET_CODE_RFS_INVALID_MON_EN_MASK             ((SINT32)-1029)
#define M_DFP_RET_CODE_RFS_INVALID_MON_FAULT_MASK          ((SINT32)-1030)
#define M_DFP_RET_CODE_RFS_MON_MANAGER_BUSY                ((SINT32)-1031)
#define M_DFP_RET_CODE_RFS_INVALID_DEBUG_DATA_ADDRESS      ((SINT32)-1032)
#define M_DFP_RET_CODE_RFS_INVALID_TX_INDEX                ((SINT32)-1033)
#define M_DFP_RET_CODE_RFS_INVALID_PD_INDEX                ((SINT32)-1034)
#define M_DFP_RET_CODE_RFS_INVALID_SUM_SAMPLES             ((SINT32)-1035)
#define M_DFP_RET_CODE_RFS_INVALID_PD_LNA_INDEX            ((SINT32)-1036)
#define M_DFP_RET_CODE_RFS_GPADC_CTM_MEM_INIT_TIMEOUT      ((SINT32)-1037)
#define M_DFP_RET_CODE_RFS_INVALID_BOOT_CAL_DATA           ((SINT32)-1038)
/*! @} */
/*! @} */

/*!
 * @brief mmWaveLink Constants for OSI delay in milliseconds
 */
#define M_DFP_OSI_WAIT_FOREVER         ((UINT32)0xFFFFFFFFU)
#define M_DFP_OSI_WAIT_SECOND          ((UINT32)0x000F4240U)
#define M_DFP_OSI_NO_WAIT              ((UINT32)0U)

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name DFP Interface Type defines
 * @{
 */

/*!
* @brief Communication Interface Handle
*/
typedef void* T_DFP_COMIF_HDL;

/*!
 * @brief Platform dependent ISR HWI Handle
 */
typedef void* T_DFP_PLT_HWI_HDL;

/*!
 * @brief OS Message Queue Object Handle
 */
typedef void* T_DFP_OSI_MSGQ_HDL;

/*!
 * @brief OS Semaphore Object Handle
 */
typedef void* T_DFP_OSI_SEM_HDL;

/*!
 * @brief OS Mutex Object Handle
 */
typedef void* T_DFP_OSI_MUTEX_HDL;

/*!
 * @brief DFP API/drivers return type, signed integer
 */
typedef SINT32 T_RETURNTYPE;

/*!*****************************************************************************
* @brief Interrupt ISR Handler callback function pointer type
*
* @b Description @n
* This is a callback function can be used to register interrupt ISR in application
* platform. The ISR is defined and registered in mWave DFP, it will have all the info
* regarding source of the interrupt.
*
*
* -----------------------------------------------------------------------------
* <B> Traceability Information </B> @n
*
* <table>
* <tr> <th> @b Requirement-Ids <td>
*
* <tr> <th> @b Architecture-Ids <td>
*
* <tr> <th> @b Design-Ids <td>
*
* </table>
*
* -----------------------------------------------------------------------------
*
* @b Documentation @n
*
*******************************************************************************
*/
typedef void (*T_DFP_PLT_ISR_HANDLER)(void);

/*!*****************************************************************************
* @brief logger Print callback function pointer type
*
* @b Description @n
* This is a callback function can be used to Print input message as per the
* format in the input arguments.
*
* @param[in] p_format  - Format of message and arguments to be printed
* @param[in] ...     - Multiple input arguments to be printed
*
* @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
* -----------------------------------------------------------------------------
* <B> Traceability Information </B> @n
*
* <table>
* <tr> <th> @b Requirement-Ids <td>
*
* <tr> <th> @b Architecture-Ids <td>
*
* <tr> <th> @b Design-Ids <td>
*
* </table>
*
* -----------------------------------------------------------------------------
*
* @b Documentation @n
*
*******************************************************************************
*/
typedef T_RETURNTYPE (*T_DFP_PRINT_FUNCP)(const UINT8 format[], ...);


/*!*****************************************************************************
* @brief Spawn Task callback function pointer type
*
* @b Description @n
* This is a callback function can be used to execute a task in a different context
*
* @param[in] c_deviceIndex  - Device Index
* @param[in] p_value     - ISR input parameter values
*
* -----------------------------------------------------------------------------
* <B> Traceability Information </B> @n
*
* <table>
* <tr> <th> @b Requirement-Ids <td>
*
* <tr> <th> @b Architecture-Ids <td>
*
* <tr> <th> @b Design-Ids <td>
*
* </table>
*
* -----------------------------------------------------------------------------
*
* @b Documentation @n
*
*******************************************************************************
*/
typedef void (*T_DFP_OSI_SPAWN_HANDLER)(UINT8 c_deviceIndex, const void* pValue);

/*! @} */

/*!
 * @name DFP Interface Data Structures
 * @{
 */
/*!
 * @brief mmWaveLink Communication interface(SPI, MailBox, UART etc) callback functions
 * data structure
 */
typedef struct
{
    /*!*****************************************************************************
    * @brief Open Communication interface
    *
    * @b Description @n
    * This is a callback function can be used to open the communication interface
    *
    * @param[in]   c_deviceIndex - Device index
    * @param[in]   w_flags  - Flags to configure the interface
    *
    * @return      T_DFP_COMIF_HDL - Handle to access the communication interface
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Architecture-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_DFP_COMIF_HDL (*p_ComIfOpen)(UINT8 c_deviceIndex, UINT32 w_flags);

    /*!*****************************************************************************
    * @brief Read Data from Communication interface
    *
    * @b Description @n
    * This is a callback function can be used to read from communication interface
    *
    * @param[in] p_fd - Handle to access the communication interface
    * @param[out] p_destAdd - Address of Buffer to store data from communication interface
    * @param[in] p_srcAdd - Address of HW memory read using communication interface
    * @param[in] w_len - Read size in bytes
    *
    * @return SINT32 Length of received data, < 0 is error code
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    SINT32 (*p_ComIfRead)(T_DFP_COMIF_HDL p_fd, UINT8 *p_destAdd, \
                    const UINT8 *p_srcAdd, UINT32 w_len);

    /*!*****************************************************************************
    * @brief Write Data to Communication interface
    *
    * @b Description @n
    * This is a callback function can be used to write over communication interface
    *
    * @param[in] p_fd - Handle to access the communication interface
    * @param[in] p_destAdd - Address of destination HW memory to store data using communication
    *                       interface
    * @param[in] p_srcAdd - Address of SW buffer containing data to write over communication
    *                       interface
    * @param[in] w_len - Write size in bytes
    *
    * @return SINT32 Length of received data, < 0 is error code
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    SINT32 (*p_ComIfWrite)(T_DFP_COMIF_HDL p_fd, UINT8 *p_destAdd, \
                    const UINT8 *p_srcAdd, UINT32 w_len);

    /*!*****************************************************************************
    * @brief Close the Communication interface
    *
    * @b Description @n
    * This is a callback function can be used to close the communication interface
    *
    * @param[in] p_fd - Handle to access the communication interface
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_ComIfClose)(T_DFP_COMIF_HDL p_fd);

}T_DFP_COMIF_CB_DATA;

/*!
 * @brief mmWaveLink platform dependent callback functions data structure
 */
typedef struct
{
    /*!*****************************************************************************
    * @brief Delay function
    *
    * @b Description @n
    * This is a callback function can be used to perform delay function, takes input
    * in us.
    *
    * @param[in] w_delayInP1Us - Delay in 0.1 us
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_Delay)(UINT32 w_delayInP1Us);

    /*!*****************************************************************************
    * @brief Time Stamp function
    *
    * @b Description @n
    * This is a callback function can be used to get systick time stamp
    *
    * @return UINT32 - Return Time stamp
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    * Systick is a decrementing 24bit counter.
    *
    *******************************************************************************
    */
    UINT32 (*p_TimeStamp)(void);

    /*!*****************************************************************************
    * @brief Register Interrupt Handler
    *
    * @b Description @n
    * This is a callback function can be used to register ISR routine on applicaton
    * platform. This ISR routine can be utilized by mmWaveLink for execution FECSS sensor
    * related APIs.
    *
    * @param[in] c_deviceIndex - Device Index to identify source of Interrupt
    * @param[in] p_isrHandler - Interrupt Handler routine
    * @param[in] p_value   - To Pass any additional data
    * @param[out] p_hwiHdl - Pointer to HWI ISR handler
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Architecture-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_RegisterIsr)(UINT8 c_deviceIndex, T_DFP_PLT_ISR_HANDLER p_isrHandler, \
                    void* p_value, T_DFP_PLT_HWI_HDL* p_hwiHdl);

    /*!*****************************************************************************
    * @brief DeRegister Interrupt Handler
    *
    * @b Description @n
    * This is a callback function can be used to deregister ISR routine on applicaton
    * platform.
    *
    * @param[in] p_hwiHdl - Pointer to HWI ISR handler
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Architecture-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_DeRegisterIsr)(T_DFP_PLT_HWI_HDL p_hwiHdl);

    /*!*****************************************************************************
    * @brief Mask Interrupt Handler
    *
    * @b Description @n
    * This is a callback function can be used to Mask ISR routine on applicaton
    * platform.
    *
    * @param[in] p_hwiHdl - Pointer to HWI ISR handler
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Architecture-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_MaskIsr)(T_DFP_PLT_HWI_HDL p_hwiHdl);

    /*!*****************************************************************************
    * @brief UnMask Interrupt Handler
    *
    * @b Description @n
    * This is a callback function can be used to UnMask ISR routine on applicaton
    * platform.
    *
    * @param[in] p_hwiHdl - Pointer to HWI ISR handler
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Architecture-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_UnMaskIsr)(T_DFP_PLT_HWI_HDL p_hwiHdl);

    /*!*****************************************************************************
    * @brief Enter Critical Region
    *
    * @b Description @n
    * This is a callback function can be used to eneter critical region on applicaton
    * platform.
    *
    * @return UINT64 - Return interrupt mask key
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Architecture-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    UINT64 (*p_EnterCriticalRegion)(void);

    /*!*****************************************************************************
    * @brief Exit Critical Region
    *
    * @b Description @n
    * This is a callback function can be used to exit critical region on applicaton
    * platform.
    *
    * @param[in] l_key - Key to restore the interupt masking
    *
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Architecture-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    void (*p_ExitCriticalRegion)(UINT64 l_key);

    /*!
    * @brief The time stamp counter mask for p_TimeStamp function
    */
    UINT32 w_TimeStampCounterMask;

}T_DFP_PLATFORM_CB_DATA;

/*!
* @brief mmWaveLink debug callback function data structure
*/
typedef struct
{
    /*!
    * @brief mmWaveLink print function pointer, logger is disabled if pointer is NULL
    */
    T_DFP_PRINT_FUNCP p_Print;

    /*!
    * @brief Pointer to mmWave DFP RFS Debug Data buffer. In case of multiple sensor
    * devices, the offset of 64 bytes will be added by mmWave DFP while storing the data.
    * The application shall allocate 64 bytes per sensor device. \n
    * RFS core pumps 64 bytes debug data in response to each API, the 64 byte data
    * is copied to this buffer address in response to each API. The Application can
    * print or dump this data and share this with TI in case of any debug.
    * The RFS debug data is in compressed format and will be parsed by TI internal
    * scripts.
    */
    UINT32* p_RfsdbgData;

}T_DFP_DBG_PRINT_CB_DATA;

/*!
 * @brief OS mutex callback functions data structure
 */
typedef struct
{
    /*!*****************************************************************************
    * @brief Create Mutex Object
    *
    * @b Description @n
    * This is a callback function can be used to create mutex object
    *
    * @param[out] p_mutexHdl - Pointer to Mutex object
    * @param[in] p_name      - Name to associate with Mutex Object
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiMutexCreate)(T_DFP_OSI_MUTEX_HDL* p_mutexHdl, UINT8* p_name);

    /*!*****************************************************************************
    * @brief Lock Mutex Object
    *
    * @b Description @n
    * This is a callback function can be used to lock mutex object.
    * Any non-zero return value will be treated as error and mmWaveLink
    * will return its own error code (M_DFP_RET_CODE_RADAR_OSIF_ERROR).
    *
    * @param[in] p_mutexHdl - Pointer to Mutex object
    * @param[in] w_timeoutUs  - Lock request timeout in us
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiMutexLock)(T_DFP_OSI_MUTEX_HDL p_mutexHdl, UINT32 w_timeoutUs);

    /*!*****************************************************************************
    * @brief Unlock Mutex Object
    *
    * @b Description @n
    * This is a callback function can be used to unlock mutex object
    * Any non-zero return value will be treated as error and mmWaveLink
    * will return its own error code (M_DFP_RET_CODE_RADAR_OSIF_ERROR).
    *
    * @param[in] p_mutexHdl - Pointer to Mutex object
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiMutexUnLock)(T_DFP_OSI_MUTEX_HDL p_mutexHdl);

    /*!*****************************************************************************
    * @brief Delete Mutex Object
    *
    * @b Description @n
    * This is a callback function can be used to delete mutex object
    *
    * @param[in] p_mutexHdl - Pointer to Mutex object
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiMutexDelete)(T_DFP_OSI_MUTEX_HDL p_mutexHdl);

}T_DFP_OSI_MUTEX_CB_DATA;

/*!
* @brief OS semaphore callback functions data structure
*/
typedef struct
{
    /*!*****************************************************************************
    * @brief Create Semaphore Object
    *
    * @b Description @n
    * This is a callback function can be used to create Semaphore object
    *
    * @param[out] p_semHdl - Pointer to Semaphore object
    * @param[in] p_name      - Name to associate with Semaphore Object
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiSemCreate)(T_DFP_OSI_SEM_HDL* p_semHdl, UINT8* p_name);

    /*!*****************************************************************************
    * @brief Wait for Semaphore
    *
    * @b Description @n
    * This is a callback function can be used to wait for Object Signal
    *
    * @param[in] p_semHdl  - Pointer to Semaphore object
    * @param[in] w_timeoutUs - Maximum Time to wait for Semaphore in us
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiSemWait)(T_DFP_OSI_SEM_HDL p_semHdl, UINT32 w_timeoutUs);

    /*!*****************************************************************************
    * @brief Release / Signal Semaphore
    *
    * @b Description @n
    * This is a callback function can be used to release Semaphore object
    *
    * @param[in] p_semHdl  - Pointer to Semaphore object
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiSemSignal)(T_DFP_OSI_SEM_HDL p_semHdl);

    /*!*****************************************************************************
    * @brief Delete Semaphore
    *
    * @b Description @n
    * This is a callback function can be used to delete Semaphore object
    *
    * @param[in] p_semHdl  - Pointer to Semaphore object
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiSemDelete)(T_DFP_OSI_SEM_HDL p_semHdl);

}T_DFP_OSI_SEMP_CB_DATA;

/*!
* @brief OS message queue/Spawn callback functions data structure
*/
typedef struct
{
    /*!*****************************************************************************
    * @brief Spawn a task in a different context
    *
    * @b Description @n
    * This is a callback function can be used to Spawn a task in a different context.
    * This is reserved for future use.
    *
    * @param[in] p_entry - Pointer to Entry Function
    * @param[in] p_value - Pointer to data passed to function
    * @param[in] w_flags - Flag to indicate preference
    *
    * @return T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
    * -----------------------------------------------------------------------------
    * <B> Traceability Information </B> @n
    *
    * <table>
    * <tr> <th> @b Requirement-Ids <td>
    *
    * <tr> <th> @b Design-Ids <td>
    *
    * </table>
    *
    * -----------------------------------------------------------------------------
    *
    * @b Documentation @n
    *
    *******************************************************************************
    */
    T_RETURNTYPE (*p_OsiSpawn)(T_DFP_OSI_SPAWN_HANDLER p_entry, const void* p_value, \
                UINT32 w_flags);

}T_DFP_OSI_MSGQ_CB_DATA;

/*!
 * @brief OS services callback functions data structure
 */
typedef struct
{
    /*!
     * @brief Mutex callback functions data structure
     */
    T_DFP_OSI_MUTEX_CB_DATA z_Mutex;
    /*!
    * @brief Semaphore callback functions data structure
    */
    T_DFP_OSI_SEMP_CB_DATA z_Semp;
    /*!
    * @brief OS message queue/Spawn callback functions.
    */
    T_DFP_OSI_MSGQ_CB_DATA z_Queue;

}T_DFP_OSI_CB_DATA;

/*!
 * @brief DFP FECSS and mmWaveLink client callback data structure
 */
typedef struct
{
    /*!
     * @brief  Device platform type holds the value of @ref M_DFP_DEVICE_PLATFORM_TYPE
     * - M_DFP_DEVICE_PLATFORM_SOC  : For mmWaveLink running on the AppSS
     * - M_DFP_DEVICE_PLATFORM_FE   : For mmWaveLink running on external HOST
     */
    UINT8                   c_PlatformType;

    /*!
     * @brief  TI mmWave Device type holds the value of @ref M_DFP_DEVICETYPE_6432 or
     * @ref M_DFP_DEVICETYPE_1432
     */
    UINT8                   c_DeviceType;

    /*!
     * @brief  mmWaveLink API error check Disable contol, the API error checks
     * are done in mmWaveLink or FECSS layer will be disabled for APIs.
     * This feature can be used to reduce the API delay in production SW on non safety devices.
     * On Safety devices it is recommended to to enable check always.
     */
    UINT8                   c_ApiErrorCheckDis;

    /*!
     * @brief  mmWaveLink API debug logger enable contol, The debug logger data print can be
     * disabled based on this control. This control will help to reduce the API latency in RFS.
     */
    UINT8                   c_ApiDbglogEn;

    /*!
     * @brief  mmWaveLink API response wait timeout in Milliseconds \n
     * mmWaveLink API returns an error if the API fails to execute or generate response in the specified time period.
     * Valid Range: 0 to 65535ms, the typical wait is 10ms
     * 1 LSB = 1ms
     */
    UINT16                  h_ApiRespTimeoutMs;

    /*!
     * @brief  Reserved for future use
     */
    UINT16                  h_Reserved1;

    /*!
     * @brief  Operating System Callbacks
     */
    T_DFP_OSI_CB_DATA       z_OsiCb;

    /*!
     * @brief  Platform related service Callbacks
     */
    T_DFP_PLATFORM_CB_DATA  z_PltfCb;

    /*!
     * @brief  Debug Callback, required to receive Debug information
     */
    T_DFP_DBG_PRINT_CB_DATA z_DbgCb;

    /*!
     * @brief  Comunication Interface Callbacks, Reserved on SOC devices
     */
    T_DFP_COMIF_CB_DATA     z_ComIfCb;

}T_DFP_CLIENT_CB_DATA;

/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

#ifdef __cplusplus
}
#endif

#endif
/*
 * END OF mmwavelink.h
 */


