#/*!****************************************************************************
# * @file rl_targets.mak
# *
# * @brief Defines the target rules for all build options
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
# * Target File Names
# * =================
# *
# * | Identifier      | Description                                            |
# * | --------------- | ------------------------------------------------------ |
# * | TARGET_NAME     | Stem of primary target filenames                       |
# * | OUT_LIB_TARGET  | .lib File path                                         |
# * | BUILD_TARGETS   | List of all top level targets required by build        |
# *
# ******************************************************************************
# */

#/* Stem of primary target filenames */
ifeq ($(PLATFORM),M4)
	TARGET_NAME := $(OUT_DIR)\mmwavelink_m4
endif

#/* Primary Target Paths */
ifeq ($(MODE),DEBUG)
	OUT_LIB_TARGET  := $(TARGET_NAME)_debug.lib
else ifeq ($(MODE),INSTR)
	OUT_LIB_TARGET  := $(TARGET_NAME)_instr.lib
else
	OUT_LIB_TARGET  := $(TARGET_NAME).lib
endif

#/*-----------------------------------------------------------------------------
# * Build Targets Selection
# *-----------------------------------------------------------------------------
# */

#/* Build option independent list of targets */
BUILD_TARGETS := $(OUT_LIB_TARGET)

#/******************************************************************************
# * Top Level Wrapper Targets
# ******************************************************************************
# */

#/* Keep this on top of all targets */
default: rebuild

#/* Sequential build wrapper targets */
build: build_dir $(BUILD_TARGETS)
	@echo Built Targets: $^
	@if exist .failed.tmp           (del /Q /S .failed.tmp > nul)
	$(DONE_MSG)

rebuild: clean build
	$(DONE_MSG)

#/******************************************************************************
# * Directory Manipulation Targets
# ******************************************************************************
# */

build_dir:
	@if not exist $(OUT_DIR)        (mkdir $(OUT_DIR))
	@if not exist $(LIST_DIR)       (mkdir $(LIST_DIR))
	@if not exist $(OBJ_DIR)        (mkdir $(OBJ_DIR))
	@if not exist $(DEP_DIR)        (mkdir $(DEP_DIR))
	@if not exist $(RL_LIB_DIR)     (mkdir $(RL_LIB_DIR))

clean: cleanIntermediates cleanBinaries
	@if exist $(BUILD_DIR)      (del /Q $(BUILD_DIR)\*)
	@if exist $(BUILD_DIR)      (rmdir /Q /S $(BUILD_DIR))
	$(DONE_MSG)

cleanIntermediates:
	@if exist $(LIST_DIR)       (del /Q /S $(LIST_DIR)\* > nul)
	@if exist $(LIST_DIR)       (rmdir /Q /S $(LIST_DIR))
	@if exist $(OBJ_DIR)        (del /Q /S $(OBJ_DIR)\* > nul)
	@if exist $(OBJ_DIR)        (rmdir /Q /S $(OBJ_DIR))
	@if exist $(DEP_DIR)         (del /Q /S $(DEP_DIR)\* > nul)
	@if exist $(DEP_DIR)         (rmdir /Q /S $(DEP_DIR))

cleanBinaries :
	@if exist $(OUT_DIR)        (del /Q /S $(OUT_DIR)\* > nul)
	@if exist $(OUT_DIR)        (rmdir /Q /S $(OUT_DIR))

#/******************************************************************************
# * Utility Targets
# ******************************************************************************
# */


#/******************************************************************************
# * Object Files Targets
# ******************************************************************************
# */
#/* Include dependency files in make
-include $(addprefix $(DEP_DIR)\,$(CORE_C_DEP))

#/* Compile C targets; and create create dependency files */
$(CORE_C_OBJ) : %.o : %.c
	@$(CC) $(CODE_COVERAGE_FLAGS) -MT $@ -MMD -MF $(DEP_DIR)\$*.dep $(MMWL_COMP_DIR_INC) $(COMPILE_OPTIONS) $(subst \,/, $<) -o $(OBJ_DIR)\$@
	@$(CC) -S $(MMWL_COMP_DIR_INC) $(COMPILE_OPTIONS) $(subst \,/, $<) -o $(OBJ_DIR)/$*.S
	@echo [$(@:.o=)]

#/*
# * Compile ASM targets
# * @note -Wno-unused-command-line-argument is used. Be careful while adding new compile arguments
# */
$(CORE_ASM_OBJ) : %.o : %.s
	@$(CC) $(MMWL_COMP_DIR_INC) $(COMPILE_OPTIONS) -Wno-unused-command-line-argument $(subst \,/, $<) -o $(OBJ_DIR)\$@
	@echo [$(@:.o=)]

#/******************************************************************************
# * Static Library Target
# ******************************************************************************
# */

#/*!
# * ARM archive library generation
# */
$(OUT_LIB_TARGET) : $(ALL_OBJ)
#   Create library file
	@del /Q /S $(OUT_DIR)\* > nul
	@$(ARCH) $(ARFLAGS) $@ $(OBJ_DIR)/*.o
	@copy /Y $(OUT_DIR)\*.lib $(RL_LIB_DIR)\. >nul
	@echo !!!!!!!!! Done ARM Archive Lib Generation !!!!!!!!

#/******************************************************************************
# * Miscellaneous Tasks/ Setup
# ******************************************************************************
# */

#/* List of phony targets */
.PHONY: build cleanBinaries clean cleanIntermediates build_dir default

#/* Makefile debug print target */
print_mk_vars:
	@echo CURR_DIR      : $(CURDIR)
	@echo CORE_C_SRC    : $(CORE_C_SRC)
	@echo CORE_ASM_SRC  : $(CORE_ASM_SRC)
	@echo ALL_OBJ       : $(ALL_OBJ)
	@echo TARGET_NAME   : $(TARGET_NAME)
	@echo TOP_MAK       : $(CURDIR)/make/fe$(TOP_MAK_NAME)_top.mak

#/*
# * END OF fe_targets.mak
# */
