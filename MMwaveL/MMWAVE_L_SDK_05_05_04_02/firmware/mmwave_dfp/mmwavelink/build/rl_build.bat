@echo off
REM/*!**************************************************************************
REM * @file rl_build.bat
REM *
REM * @brief Wrapper script for the MMWAVELINK firmware build makefile
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
REM * 0.1     20Jan2021   TI      NA                  Initial Version
REM * 0.1     24May2023   Prisha  MMWAVE_DFP_LOW-430  Instrumented Build for DFP
REM */
@echo.

SETLOCAL

REM/* Clear environment path to avoid conflicting tools */
SET PATH_BACKUP=%PATH%
SET PATH=""

REM call user defined environment batch file
call rl_setenv.bat

SET BIN_DIR=..\bin\
if not exist %BIN_DIR% mkdir %BIN_DIR%

SET LOGFILE=%BIN_DIR%\rl_build.log

SET UTILS= %MMWAVE_DFP_UTILS_DIR_PATH%

REM log file
@echo Build: %1 %2 %3 %date% %time% > %LOGFILE% 2>&1

REM/*!**************************************************************************
REM * Command Line Arguments and Supported Options:
REM * =============================================
REM *  BUILD_NAME  : Build directory name
REM *  MODE        : RELEASE, DEBUG (logging enable)
REM *  DEVICE      : 6432, 1432, 6131
REM *  PLATFORM    : M4
REM *  TARGET      : ROM
REM *  OTHER       : <; separated list of MACROS>
REM ****************************************************************************
REM */

if not "%3" == "clean" (echo > .failed.tmp)

IF "%1" == "lib_m4_release"     GOTO lib_m4_release
IF "%1" == "lib_m4_debug"       GOTO lib_m4_debug
IF "%1" == "lib_m4_instr"       GOTO lib_m4_instr

GOTO help

:lib_m4_release
SET BUILD_NAME=MMWL_M4_RELEASE
SET DEVICE=%2
SET MACRO_FLAGS=%4
%UTILS%\make -f .\make\rl_top.mak BUILD_NAME=%BUILD_NAME% PLATFORM=M4 TARGET=ROM MODE=RELEASE ^
    DEVICE=%DEVICE% MACRO_FLAGS=%MACRO_FLAGS% %3 >> %LOGFILE% 2>&1
if exist .failed.tmp (del .failed.tmp && GOTO exit_error)
GOTO exit

:lib_m4_debug
SET BUILD_NAME=MMWL_M4_DEBUG
SET DEVICE=%2
%UTILS%\make -f .\make\rl_top.mak BUILD_NAME=%BUILD_NAME% PLATFORM=M4 TARGET=ROM MODE=DEBUG ^
    DEVICE=%DEVICE% MACRO_FLAGS=%4 %3 >> %LOGFILE% 2>&1
if exist .failed.tmp (del .failed.tmp && GOTO exit_error)
GOTO exit

:lib_m4_instr
SET BUILD_NAME=MMWL_M4_INSTR
SET DEVICE=%2
%UTILS%\make -f .\make\rl_instr_top.mak BUILD_NAME=%BUILD_NAME% PLATFORM=M4 TARGET=ROM MODE=INSTR ^
    DEVICE=%DEVICE% MACRO_FLAGS=%4 %3 >> %LOGFILE% 2>&1
if exist .failed.tmp (del .failed.tmp && GOTO exit_error)
GOTO exit

:help
@ECHO.
@ECHO "rl_build.bat Usage: ./rl_build.bat [build] [device] [option] [USER_MACROS]"
@ECHO.
@ECHO.
@ECHO Supported Devices (mandatory): 6432, 1432 (Lx432)
@ECHO =============================
@ECHO.
@ECHO Build Options (mandatory)
@ECHO =========================
@ECHO       lib_m4_release   : mmWaveLink Static Library Release Build for RAM functions.
@ECHO                             Build Directory: MMWL_M4_RELEASE
@ECHO       lib_m4_debug     : mmWaveLink Static Library Debug Build for RAM functions.
@ECHO                             The Debug print is supported in this build.
@ECHO                             Build Directory: MMWL_M4_DEBUG
@ECHO       lib_m4_instr     : mmWaveLink Static Library Instrumented Build for RAM functions.
@ECHO                             Build Directory: MMWL_M4_INSTR
@ECHO.
@ECHO Targets (Optional)
@ECHO ==================
@ECHO       build         : (Default) Build all targets
@ECHO       clean         : Clean all files
@ECHO       rebuild       : Clean and build all targets
@ECHO.
@ECHO.
@ECHO Examples:
@ECHO ==========
@ECHO .\rl_build.bat lib_m4_release Lx432
@ECHO .\rl_build.bat lib_m4_release Lx432 clean
@ECHO .\rl_build.bat lib_m4_release Lx432 rebuild
@ECHO.

:exit_error
REM/* Restore path for xcopy */
SET PATH=%PATH_BACKUP%

@echo. >> %LOGFILE% 2>&1 | type %LOGFILE%

REM/* Restore environment path */
ENDLOCAL

exit 1

:exit

REM/* Restore path for xcopy */
SET PATH=%PATH_BACKUP%

@echo. >> %LOGFILE% 2>&1 | type %LOGFILE%

REM/* Restore environment path */
ENDLOCAL

REM/*
REM * END OF rl_build.bat
REM */