#/*!****************************************************************************
# * @file rl_instr_top.mak
# *
# * @brief Top level wrapper fo makefile setup to build firmware for mmWave low
# *        power devices when files are instrumented for code coverage
# *
# * @note
# * © Copyright 2022, Texas Instruments Incorporated – www.ti.com
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# * * Redistributions of source code must retain the above copyright
# *   notice, this list of conditions and the following disclaimer.
# * * Redistributions in binary form must reproduce the above copyright
# *   notice, this list of conditions and the following disclaimer in the
# *   documentation and/or other materials provided with the distribution.
# * * Neither the name of Texas Instruments Incorporated nor the names of
# *   its contributors may be used to endorse or promote products derived
# *   from this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS
# * IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
# * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

#/******************************************************************************
# * Revision History   :
# *-----------------------------------------------------------------------------
# * Version Date        Author  Ticket No           Description
# *-----------------------------------------------------------------------------
# * 0.1     24May2023   Prisha  MMWAVE_DFP_LOW-430  Instrumented Build for DFP
# */

#/******************************************************************************
# * Warning:
# * ========
# * * This file is designed and tested only to work with gmake 4.1 higher
# * * White spaces are VERY IMPORTANT in makefiles, do not allow unwanted spaces
# *   to creep onto the beginning of blank lines, or at end of a line.
# *   Indent with spaces than tabs
# ******************************************************************************
# */

#/******************************************************************************
# * Command Line Arguments and Supported Options:
# * =============================================
# * | Argument    | Options |
# * | =========== | ======= |
# * | BUILD_NAME  | Build directory name |
# * | PLATFORM    | M4
# * | TARGET      | ROM, PATCH  |
# * | MODE        | INSTR |
# * | DEVICE      | 6432, 6131, 1432 |
# * | OTHER       | <; separated list of MACROS> |
# *
# ******************************************************************************
# */

#/*!
# * The makefile includes following make files:
# * @see rl_setup.mak : Tools and directory paths, Compiler and Linker Flags
# * @see rl_files     : Source files selection based on the TARGET and MODE
# * @see rl_targets   : Build target rules
# *
# * @note: Please do not change the order of included files as most of the
# *        variables are set as expanded variables
# */

include .\make\rl_setup.mak

#/* Add code coverage flags */

CODE_COVERAGE_FLAGS := -fmcdc -fprofile-instr-generate -fcoverage-mapping -fprofile-list=../../platform/xwrlx432/test/temp/instrumented_files.list

include .\make\rl_files.mak
include .\make\rl_targets.mak

#/*
# * END OF rl_top.mak
# */
