@echo off
REM/*!**************************************************************************
REM * @file dfp_build.bat
REM *
REM * @brief Wrapper script for the DFP Libraries build makefile
REM *
REM * @note
REM * © Copyright 2022, Texas Instruments Incorporated – www.ti.com
REM *
REM * Redistribution and use in source and binary forms, with or without
REM * modification, are permitted provided that the following conditions are met:
REM * * Redistributions of source code must retain the above copyright
REM *   notice, this list of conditions and the following disclaimer.
REM * * Redistributions in binary form must reproduce the above copyright
REM *   notice, this list of conditions and the following disclaimer in the
REM *   documentation and/or other materials provided with the distribution.
REM * * Neither the name of Texas Instruments Incorporated nor the names of
REM *   its contributors may be used to endorse or promote products derived
REM *   from this software without specific prior written permission.
REM *
REM * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS
REM * IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
REM * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
REM * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
REM * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
REM * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
REM * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
REM * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
REM * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
REM * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
REM * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
REM */
REM
REM * Revision History   :
REM *---------------------------------------------------------------------------
REM * Version Date        Author  Ticket No           Description
REM *---------------------------------------------------------------------------
REM * 0.1     27Jan2021   TI      NA                  Initial Version
REM * 0.2     24May2023   Prisha  MMWAVE_DFP_LOW-430  Instrumented Build for DFP
REM */
@echo.

SETLOCAL

REM/* Clear environment path to avoid conflicting tools */
SET PATH_BACKUP=%PATH%
SET PATH=""

SET BIN_DIR=..\bin\
if not exist %BIN_DIR% mkdir %BIN_DIR%

SET LOGFILE=%BIN_DIR%\dfp_build.log

SET UTILS= ..\..\tools\cgtools\utils
SET WORKING_DIR=%CD%
SET FECSS_BUILD_PATH= ..\fecsslib\build
SET MMWL_BUILD_PATH= ..\mmwavelink\build

REM log file
@echo Build: %1 %2 %3 %date% %time% | %UTILS%\tee %LOGFILE% 2>&1
SET DEVICE=%2
SET TARGET=%3
REM/*!**************************************************************************
REM * Command Line Arguments and Supported Options:
REM * =============================================
REM *  BUILD_NAME  : mmWaveLInk & FECSS Library Build name
REM *  DEVICE      : Lx432
REM *  OPTION      : rebuild
REM *  OTHER       : <; separated list of MACROS>
REM ****************************************************************************
REM */

REM /* DFP build option */
IF "%1" == "dfp_romlib_m4_release"     (    SET FECSLIB_MODE=romlib_m4_release
                                            SET LINK_MODE=lib_m4_release
                                            GOTO :dfp_build_function )

IF "%1" == "dfp_romlib_m4_debug"       (    SET FECSLIB_MODE=romlib_m4_debug
                                            SET LINK_MODE=lib_m4_debug
                                            GOTO :dfp_build_function )

IF "%1" == "dfp_patchlib_m4_release"   (    SET FECSLIB_MODE=patchlib_m4_release
                                            SET LINK_MODE=lib_m4_release
                                            GOTO :dfp_build_function )

IF "%1" == "dfp_patchlib_m4_debug"     (    SET FECSLIB_MODE=patchlib_m4_debug
                                            SET LINK_MODE=lib_m4_debug
                                            GOTO :dfp_build_function )

IF "%1" == "dfp_ramlib_m4_release"     (    SET FECSLIB_MODE=ramlib_m4_release
                                            SET LINK_MODE=lib_m4_release
                                            GOTO :dfp_build_function )

IF "%1" == "dfp_ramlib_m4_debug"       (    SET FECSLIB_MODE=ramlib_m4_debug
                                            SET LINK_MODE=lib_m4_debug
                                            GOTO :dfp_build_function )

IF "%1" == "dfp_ramlib_m4_instr"       (    SET FECSLIB_MODE=ramlib_m4_instr
                                            SET LINK_MODE=lib_m4_instr
                                            GOTO :dfp_build_function )

GOTO help


:dfp_build_function
REM /* Build FECSSLib library */
cd %FECSS_BUILD_PATH%
call fe_build.bat %FECSLIB_MODE% %DEVICE% %TARGET% || exit 1
cd %WORKING_DIR%

@echo FECSSLib build is Done!! 2>&1  | %UTILS%\tee -a %LOGFILE% || exit 1

REM /* Build mmWaveLink library */
cd %MMWL_BUILD_PATH%
call rl_build.bat %LINK_MODE% %DEVICE% %TARGET% || exit 1
cd %WORKING_DIR%

@echo mmWaveLink build is Done!! 2>&1  | %UTILS%\tee -a %LOGFILE% || exit 1

GOTO exit


:help
@ECHO.
@ECHO "dfp_build.bat Usage: ./dfp_build.bat [build] [device] [USER_MACROS]"
@ECHO.
@ECHO.
@ECHO Supported Devices (mandatory): Lx432
@ECHO =============================
@ECHO.
@ECHO Build Options (mandatory)
@ECHO =========================
@ECHO      dfp_romlib_m4_release   : DFP, FECSSLib ROM and mmWaveLink Release Build
@ECHO                              - FECSS Static Library Release Build for ROM functions.
@ECHO                                TI ROM is not available in ES1.0, application
@ECHO                                can place this libarary in any RAM section.
@ECHO                                Build Directory: fecsslib/bin/FECSSLIB-M4-ROM-RELEASE
@ECHO                              - mmWaveLink Static Library Release Build for RAM functions.
@ECHO                                Build Directory: mmwavelink/bin/MMWL-M4-RELEASE
@ECHO      dfp_romlib_m4_debug     : DFP, FECSSLib ROM and mmWaveLink Debug Build
@ECHO                              - FECSS Static Library Debug Build for ROM functions.
@ECHO                                The Debug print is supported in this build.
@ECHO                                Build Directory: fecsslib/bin/FECSSLIB-M4-ROM-DEBUG
@ECHO                              - mmWaveLink Static Library Debug Build for RAM functions.
@ECHO                                The Debug print is supported in this build.
@ECHO                                Build Directory: mmwavelink/bin/MMWL-M4-DEBUG
@ECHO      dfp_patchlib_m4_release : : DFP, FECSSLib PATCH and mmWaveLink Release Build
@ECHO                              - FECSS Static Library Release Build for PATCH functions.
@ECHO                                TI ROM is not available in ES1.0, this build option
@ECHO                                is reserved.
@ECHO                                Build Directory: fecsslib/bin/FECSSLIB-M4-PATCH-RELEASE
@ECHO                              - mmWaveLink Static Library Release Build for RAM functions.
@ECHO                                Build Directory: mmwavelink/bin/MMWL-M4-RELEASE
@ECHO      dfp_patchlib_m4_debug   : DFP, FECSSLib ROM and mmWaveLink Debug Build
@ECHO                              - FECSS Static Library Debug Build for PATCH functions.
@ECHO                                The Debug print is supported in this build. this build option
@ECHO                                is reserved.
@ECHO                                Build Directory: fecsslib/bin/FECSSLIB-M4-PATCH-DEBUG
@ECHO                              - mmWaveLink Static Library Debug Build for RAM functions.
@ECHO                                The Debug print is supported in this build.
@ECHO                                Build Directory: mmwavelink/bin/MMWL-M4-DEBUG
@ECHO      dfp_ramlib_m4_release   : DFP, FECSSLib ROM+PATCH and mmWaveLink RAM Release Build
@ECHO                              - FECSS Static Library Release Build for all functions.
@ECHO                                This build option is available on all silicon versions at the
@ECHO                                at the expense of memory.
@ECHO                                Build Directory: fecsslib/bin/FECSSLIB-M4-RAM-RELEASE
@ECHO                              - mmWaveLink Static Library Release Build for RAM functions.
@ECHO                                Build Directory: mmwavelink/bin/MMWL-M4-RELEASE
@ECHO      dfp_ramlib_m4_debug     : DFP, FECSSLib ROM+Patch and mmWaveLink Debug Build
@ECHO                              - FECSS Static Library Debug Build for RAM functions.
@ECHO                                The Debug print is supported in this build. this build option
@ECHO                                is reserved.
@ECHO                                Build Directory: fecsslib/bin/FECSSLIB-M4-RAM-DEBUG
@ECHO                              - mmWaveLink Static Library Debug Build for RAM functions.
@ECHO                                The Debug print is supported in this build.
@ECHO                                Build Directory: mmwavelink/bin/MMWL-M4-DEBUG
@ECHO      dfp_ramlib_m4_instr     : DFP, FECSSLib ROM+Patch functions instrumented build
@ECHO                              - This mode is useful of testing and dynamic analysis report generation
@ECHO.
@ECHO Examples:
@ECHO ==========
@ECHO .\dfp_build.bat dfp_romlib_m4_release Lx432
@ECHO .\dfp_build.bat dfp_romlib_m4_debug Lx432
@ECHO.

:exit

REM/* Restore path for xcopy */
SET PATH=%PATH_BACKUP%

@echo. 2>&1  | %UTILS%\tee -a %LOGFILE%

REM/* Restore environment path */
ENDLOCAL

REM/*
REM * END OF dfp_build.bat
REM */
