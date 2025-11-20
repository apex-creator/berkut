@echo off
REM/*!**************************************************************************
REM * @file rl_setenv.bat
REM *
REM * @brief Set up tools and build environment variables for MMWAVELINK library generation
REM *
REM * @note
REM * © Copyright 2022, Texas Instruments Incorporated © www.ti.com
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
REM * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
REM * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
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
REM * 0.1     22Jan2021   TI      NA                  Initial Version
REM */
@echo.

@REM Path to <CG tools> folder, this path needs to be updated as per user tools directory
set MMWAVE_DFP_CGTOOLS_DIR_PATH=..\..\..\tools\cgtools\arm\armllvm_2.1.3.LTS
@REM Path to <make file utils> folder, this path needs to be updated as per user tools directory
set MMWAVE_DFP_UTILS_DIR_PATH=..\..\..\tools\cgtools\utils

REM/*
REM * END OF rl_setenv.bat
REM */

