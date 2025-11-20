#/*!****************************************************************************
# * @file rl_files.mak
# *
# * @brief Defines the list of input source files for all build configurations
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
# * 0.1     21Jan2021   TI      NA                  Initial Version
# */

# /*****************************************************************************
# Warning:
# ========
#   White space is VERY IMPORTANT in makefiles, do not allow unwanted spaces
#   to creep onto the beginning of blank lines, or at end of a line.
#   Indent with spaces than tabs
# ******************************************************************************
# */

#/******************************************************************************
# * List of Module Directories for All Targets
# ******************************************************************************
# */

#/* Include Source Directory */
MMWL_WORK_INC_DIR      := $(FW_DIR)\..

#/* Source Directories */
MMWL_MODULE_SRC_DIR       := $(FW_DIR)\source

#/******************************************************************************
# * List of Source Files for Selected Modules
# ******************************************************************************
# */

#/*-----------------------------------------------------------------------------
# * Select required source files
# *-----------------------------------------------------------------------------
# */

#/* Get list of source file names */
CORE_C_SRC          := rl_device.c \
                       rl_sensor.c \
                       rl_monitor.c
CORE_ASM_SRC        :=

#/* List of object and dependency files */
CORE_C_OBJ          := $(CORE_C_SRC:.c=.o)
CORE_C_DEP          := $(CORE_C_SRC:.c=.dep)

CORE_ASM_OBJ        := $(CORE_ASM_SRC:.s=.o)

#/* List of all object files required by build */
ALL_OBJ             := $(CORE_C_OBJ) $(CORE_ASM_OBJ)

#/******************************************************************************
# * Directory Include Paths for Compiler and Make
# ******************************************************************************
# */

MMWL_COMP_DIR_INC    := $(foreach MODULE_PATH,$(MMWL_WORK_INC_DIR), -I $(MODULE_PATH))
MMWL_VPATH           := $(MMWL_WORK_INC_DIR) $(MMWL_MODULE_SRC_DIR)

#/*
# * END OF rl_files.mak
# */
