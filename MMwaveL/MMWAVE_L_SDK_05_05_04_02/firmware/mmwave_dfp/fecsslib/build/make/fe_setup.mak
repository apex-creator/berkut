#/*!****************************************************************************
# * @file fe_setup.mak
# *
# * @brief Defines the tools and directory paths, compiler and linker flags,
# *        information about the memory models, etc.
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

# * Revision History   :
# *-----------------------------------------------------------------------------
# * Version Date        Author  Ticket No           Description
# *-----------------------------------------------------------------------------
# * 0.1     21Jan2021   TI      NA                  Initial Version
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
# * Exit if DEVICE is not supported
# ******************************************************************************
# */

SUPPORTED_DEVICES := Lx432
SUPPORTED_SOC_DEVICES := Lx432

ifneq ($(DEVICE),$(filter $(DEVICE),$(SUPPORTED_DEVICES)))
	$(error Unsupported Device Selected: $(DEVICE))
endif

#/******************************************************************************
# * Tools Paths
# ******************************************************************************
# */

TOOL_DIR		:= $(MMWAVE_DFP_CGTOOLS_DIR_PATH)\bin

#/* Compile and Build Tools Paths */
CC              := $(TOOL_DIR)\tiarmclang
LNK             := $(TOOL_DIR)\tiarmclang
HEX             := $(TOOL_DIR)\tiarmhex
DIS             := $(TOOL_DIR)\tiarmdis
OFD             := $(TOOL_DIR)\tiarmofd
ARCH            := $(TOOL_DIR)\tiarmar

#/******************************************************************************
# * Source and Target Directories
# ******************************************************************************
# */
override FW_DIR     := ..

#/* Miscellaneous Input Directories */
MAKE_DIR            := .\make

#/* Output Directories */
BUILD_DIR           := $(FW_DIR)\bin\$(BUILD_NAME)_$(DEVICE)
LIST_DIR            := $(BUILD_DIR)\lst
OBJ_DIR             := $(BUILD_DIR)\obj
DEP_DIR             := $(BUILD_DIR)\dep
OUT_DIR             := $(BUILD_DIR)\out
FE_LIB_DIR          := $(FW_DIR)\lib\xWR$(DEVICE)

#/******************************************************************************
# * Compiler Flags and Include Paths
# ******************************************************************************
# */

#/* Optimization Level Flags */
OPTMFLAGS           := -Os

#/* Platform related Flags, default support only M4 now */
ifeq ($(PLATFORM), M4)
	MODEFLAGS           := -mcpu=cortex-m4
else
	MODEFLAGS           := -mcpu=cortex-m4
endif

VFPFLAGS            := -mfloat-abi=hard -mfpu=fpv4-sp-d16
ENDIANFLAGS         := -mlittle-endian
COMMONFLAGS         := -g -fdiagnostics-show-option -ffunction-sections -fshort-enums \
						 -finline-limit=0

#/* Build Defines for the source code */
BUILD_OTHER_FLAGS   := $(foreach FLAG,$(subst *, ,$(MACRO_FLAGS)), $(FLAG))
BUILD_FLAGS         := _DEVICE_$(DEVICE)_ _TARGET_$(TARGET)_ _MODE_$(MODE)_ _PLATFORM_$(PLATFORM)_ $(BUILD_OTHER_FLAGS)
#/* Build Define - device type FE (not SOC) */
ifneq ($(DEVICE),$(filter $(DEVICE),$(SUPPORTED_SOC_DEVICES)))
	BUILD_FLAGS += M_DFP_DEVICE_PLATFORM_TYPE=2
else
#/* Build Define - device type SOC */
	BUILD_FLAGS += M_DFP_DEVICE_PLATFORM_TYPE=1
endif
#/* Build Define - Debug print enable */
ifeq ($(MODE),DEBUG)
	BUILD_FLAGS   += M_DFP_DISABLE_LOGGING=0
else
#/* Build Define - Debug print disable */
	BUILD_FLAGS   += M_DFP_DISABLE_LOGGING=1
endif
COMPILER_BUILD_FLAGS    = $(foreach FLAG,$(BUILD_FLAGS),-D $(FLAG))
LINKER_BUILD_FLAGS    = $(foreach FLAG,$(BUILD_FLAGS),-Xlinker --define=$(FLAG))

COMMONFLAGS_COMPILER := -c -mno-unaligned-access -ferror-limit=20 \
						-Wno-ignored-optimization-argument

#/*-----------------------------------------------------------------------------
# * Final Flags and Options, for ROM code creation
# *-----------------------------------------------------------------------------
# */
COMPILER_FLAGS   := $(MODEFLAGS) $(VFPFLAGS) $(ENDIANFLAGS) $(COMMONFLAGS) \
					$(COMMONFLAGS_COMPILER) $(OPTMFLAGS_O3)

COMPILE_OPTIONS  = $(COMPILER_FLAGS) $(COMPILER_BUILD_FLAGS)

#/* ARM ARCHIVE LIB Flags */
ARFLAGS    := -r

#/******************************************************************************
# * Miscellaneous Variables
# ******************************************************************************
# */

#/* Default number of processes in parallel mode */
ifeq ($(NPROC),)
	NPROC := 4
endif

#/* Done Message for Major Targets */
DONE_MSG    = @echo === DONE MAKING [ $@ ] =========================

# ------------------------------------------------------------------------------
# VPATH : Search Path for All Prerequisites
# ------------------------------------------------------------------------------
VPATH = $(FECSS_VPATH) $(LIST_DIR) $(OBJ_DIR) $(DEP_DIR) $(OUT_DIR)

#/*
# * END OF fe_setup.mak
# */
