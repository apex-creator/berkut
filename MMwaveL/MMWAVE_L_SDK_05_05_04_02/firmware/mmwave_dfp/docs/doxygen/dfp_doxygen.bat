@echo off
::/*!**************************************************************************
:: * @file dfp_doxygen.bat
:: *
:: * @brief Build the doxygen documentation for the mmWave DFP
:: *
:: * @note
:: * © Copyright 2022, Texas Instruments Incorporated – www.ti.com
:: *
:: * Redistribution and use in source and binary forms, with or without
:: * modification, are permitted provided that the following conditions are met:
:: * * Redistributions of source code must retain the above copyright
:: *   notice, this list of conditions and the following disclaimer.
:: * * Redistributions in binary form must reproduce the above copyright
:: *   notice, this list of conditions and the following disclaimer in the
:: *   documentation and/or other materials provided with the distribution.
:: * * Neither the name of Texas Instruments Incorporated nor the names of
:: *   its contributors may be used to endorse or promote products derived
:: *   from this software without specific prior written permission.
:: *
:: * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS
:: * IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
:: * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
:: * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
:: * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
:: * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
:: * LIMITED TO, PROCU::ENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
:: * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
:: * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
:: * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
:: * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
:: */
::
:: * Revision History   :
:: *---------------------------------------------------------------------------
:: * Version Date        Author  Defect No           Description
:: *---------------------------------------------------------------------------
:: * 0.1     27Jan2021   TI      NA                  Initial Version
:: */
@echo.

::/* Start local environment */
SETLOCAL

::/****************************************************************************
:: * Setup
:: ****************************************************************************
:: */

::/*!--------------------------------------------------------------------------
:: * Required Paths
:: *---------------------------------------------------------------------------
:: */

::/* Doxygen path */
SET DOXYGEN="C:\Program Files\doxygen\bin\doxygen.exe"

::/* Graphviz path */
SET DOT_PATH="C:\Program Files\Graphviz\bin\"

::/*!--------------------------------------------------------------------------
:: * Optional Paths
:: *---------------------------------------------------------------------------
:: */

::/* PlantUML for beautiful graphs, @note Must be in POSIX-style without "" */
SET ENV_PLANTUML_JAR_PATH=../../../../tools/cgtools/utils/plantuml.jar

::/* Java to run plantUML from CCS */
for /f %%i in ('dir /ad /b "c:\ti" ^| findstr /x "ccs[0-9][0-9][0-9][0-9]" ^| sort') do set LATEST_CCS=%%i
set CCS_INSTALL_DIR=C:\ti\%LATEST_CCS%\ccs
SET JAVA_PATH=%CCS_INSTALL_DIR%"\eclipse\jre\bin"

::/* HHC exe file path */
SET ENV_HHC_EXE="C:\Programs\HTML_Help_Workshop\hhc.exe"

::/* 7-Zip exe file path */
SET ZIP="C:\Program Files\7-Zip\7z.exe"

::/*!--------------------------------------------------------------------------
:: * Pre-Run Setup
:: *---------------------------------------------------------------------------
:: */

::/* Create run directory for doxygen to avoid clutter in docs directory */
If not exist ".\run\" mkdir .\run\

::/* Include dot and java in the local path variable; Doxygen calls them directly */
SET PATH=%DOT_PATH%;%JAVA_PATH%;%PATH%

IF "clean"=="%1"    GOTO clean
IF "run"=="%1"      GOTO run
GOTO help

::/****************************************************************************
:: * Run Doxygen
:: ****************************************************************************
:: */
:run

::/* Default settings */
SET ENV_DOXY_GENERATE_HTML_HELP=NO
SET ENV_DOXY_GENERATE_LATEX=NO
SET COMPRESS=NO

::/* Flags start from %2 */
SHIFT
SET RUN_FLAGS=
:run_flags_extract_loop
if "%1" == "" GOTO post_run_flags_extract_loop
SET RUN_FLAGS=%RUN_FLAGS% %1
SHIFT
GOTO run_flags_extract_loop
:post_run_flags_extract_loop

for %%a in (%RUN_FLAGS%) do (
    if "%%a" == "-h" SET ENV_DOXY_GENERATE_HTML_HELP=YES
    if "%%a" == "-l" SET ENV_DOXY_GENERATE_LATEX=YES
    if "%%a" == "-c" SET COMPRESS=YES
)

::/* Run Doxygen */
cd .\run\
%DOXYGEN% -d extcmd ..\source\Doxyfile
cd ../

if "YES"== "%COMPRESS%" (
    call %ZIP% a -tzip ..\mmwave_dfp_low_api_documentation.zip .\html\*
)

GOTO exitHandler


::/****************************************************************************
:: * Clean directories
:: ****************************************************************************
:: */
:clean

@RMDIR /S /Q .\html > nul
@RMDIR /S /Q .\run  > nul
@RMDIR /S /Q .\latex > nul
GOTO exitHandler


::/****************************************************************************
:: * Help
:: ****************************************************************************
:: */

:help
ECHO.
ECHO "dfp_doxygen.bat Usage: .\dfp_doxygen.bat [ACTION] [OPTIONS]"
ECHO.
ECHO Examples:
ECHO =========
ECHO .\dfp_doxygen.bat run
ECHO .\dfp_doxygen.bat clean
ECHO.
ECHO RUN:
ECHO =====
ECHO Create doxygen documentation
ECHO Options:
ECHO    -h  : Generate HTML help output
ECHO    -l  : Generate LaTex output
ECHO    -c  : Generate ZIP of HTML output
ECHO.

:exitHandler
ECHO ========== DONE %~n0%~x0 %1 ==========
ECHO.


ENDLOCAL


::/*
:: * END OF dfp_doxygen.bat
:: */
