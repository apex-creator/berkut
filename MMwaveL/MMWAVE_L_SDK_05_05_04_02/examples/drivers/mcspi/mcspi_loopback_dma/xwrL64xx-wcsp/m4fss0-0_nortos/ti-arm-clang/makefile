#
# Auto generated makefile
#

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak

CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)

CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang
STRIP=$(CG_TOOL_ROOT)/bin/tiarmstrip
OBJCOPY=$(CG_TOOL_ROOT)/bin/tiarmobjcopy

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=mcspi_loopback_dma_wcsp.$(PROFILE).out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_BIN_NAME:=mcspi_loopback_dma_wcsp.$(PROFILE).appimage
BOOTIMAGE_BIN_TEMP:=mcspi_loopback_dma_wcsp.$(PROFILE)_temp.appimage
BOOTIMAGE_RPRC_NAME_TMP:=mcspi_loopback_dma_wcsp.$(PROFILE)_rprc.bin
AUTH_BOOTIMAGE_PATH=$(abspath .)
AUTH_BOOTIMAGE_BIN_NAME:=mcspi_loopback_dma_auth_wcsp.$(PROFILE).appimage
BSS_RPRC_BIN:=$(MMW_DFP_PATH)/rfsfirmware/xWRL6432_WCSP/mmwave_rfs_patch_rprc.bin

FILES_common := \
	mcspi_loopback_dma.c \
	main.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_common = \
	.. \
	../../.. \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MCU_PLUS_SDK_PATH}/source \
	-Igenerated \

DEFINES_common := \
	-DSOC_XWRL64XX \

CFLAGS_common := \
	-mcpu=cortex-m4 \
	-mfloat-abi=hard \
	-mno-unaligned-access \
	-mthumb \
	-Wall \
	-Werror \
	-g \
	-Wno-gnu-variable-sized-type-not-at-end \
	-Wno-unused-function \

CFLAGS_cpp_common := \
	-Wno-c99-designator \
	-Wno-extern-c-compat \
	-Wno-c++11-narrowing \
	-Wno-reorder-init-list \
	-Wno-deprecated-register \
	-Wno-writable-strings \
	-Wno-enum-compare \
	-Wno-reserved-user-defined-literal \
	-Wno-unused-const-variable \
	-x c++ \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	-Os \

LNK_FILES_common = \
	linker.cmd \

LIBS_PATH_common = \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/board/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lnortos.xwrL64xx.m4f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.xwrL64xx.m4f.ti-arm-clang.${ConfigName}.lib \
	-lboard.xwrL64xx.m4f.ti-arm-clang.${ConfigName}.lib \
	-llibc.a \

LFLAGS_common = \
	-Wl,--ram_model \
	-Wl,--reread_libs \

LIBS_NAME = \
	nortos.xwrL64xx.m4f.ti-arm-clang.${ConfigName}.lib \
	drivers.xwrL64xx.m4f.ti-arm-clang.${ConfigName}.lib \
	board.xwrL64xx.m4f.ti-arm-clang.${ConfigName}.lib \
	libc.a \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${CG_TOOL_ROOT}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
LNK_FILES := $(LNK_FILES_common) $(LNK_FILES_$(PROFILE))

OBJDIR := obj/$(PROFILE)/
OBJS := $(FILES:%.c=%.obj)
OBJS += $(ASMFILES:%.S=%.obj)
DEPS := $(FILES:%.c=%.d)

vpath %.obj $(OBJDIR)
vpath %.c $(FILES_PATH)
vpath %.S $(FILES_PATH)
vpath %.lib $(LIBS_PATH_NAME)
vpath %.a $(LIBS_PATH_NAME)

$(OBJDIR)/%.obj %.obj: %.c
	@echo  Compiling: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $(LIBNAME): $<
	$(CC) -c $(CFLAGS) -o $(OBJDIR)/$@ $<

ifeq ($(OS),Windows_NT)
all: $(AUTH_BOOTIMAGE_BIN_NAME) $(BOOTIMAGE_BIN_NAME)
else
all: $(BOOTIMAGE_BIN_NAME)
endif

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h

$(OUTNAME): syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
	@echo  .
	@echo  Linking: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $@ ...
	$(LNK) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES)
	@echo  Linking: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $@ Done !!!
	@echo  .

clean:
	@echo  Cleaning: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_BIN_NAME)
	$(RM) $(AUTH_BOOTIMAGE_BIN_NAME)
	$(RM) $(INTEGRITY_BIN)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: xwrL64xx:m4fss0-0:nortos:ti-arm-clang mcspi_loopback_dma ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
endif
	$(RMDIR) generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@


.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

syscfg: ../example.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context m4fss0-0 --part Default --package FCCSP --output generated/ ../example.syscfg

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device XWRL64XX --context m4fss0-0 --part Default --package FCCSP --output generated/  ../example.syscfg

#
# Generation of boot image which can be loaded by ROM Boot Loader (RBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif

BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt
AUTH_BOOTIMAGE_TEMP_OUT_FILE=temp_auth_stdout_$(PROFILE).txt

INTEGRITY_GEN=$(MCU_PLUS_SDK_PATH)/tools/auth_tool/Integrity_generator$(EXE_EXT)
SALT_PATH=$(MCU_PLUS_SDK_PATH)/tools/auth_tool/encsalt.txt
INFO_PATH=$(MCU_PLUS_SDK_PATH)/tools/auth_tool/encinfo.txt
SYMM_KEY_PATH=$(MCU_PLUS_SDK_PATH)/tools/auth_tool/encsymm.txt
INTEGRITY_BIN=mcspi_loopback_dma_integrity.bin

OUTRPRC_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/out2rprc/out2rprc.exe
MULTI_CORE_IMAGE_GEN=$(MCU_PLUS_SDK_PATH)/tools/boot/multicoreImageGen/MulticoreImageGen$(EXE_EXT)
CRC_MULTI_CORE_IMAGE_GEN=$(MCU_PLUS_SDK_PATH)/tools/boot/crcMulticoreImageGen/crcMulticoreimage$(EXE_EXT)
APPEND_CRC=$(MCU_PLUS_SDK_PATH)/tools/boot/appendBinCrc/appendBinCrc$(EXE_EXT)
ifeq ($(OS),Windows_NT)
  OUTRPRC_CMD=$(OUTRPRC_PATH)
else
  OUTRPRC_CMD=mono $(OUTRPRC_PATH)
endif


BOOT_MODE=0
BOOT_VECTOR_ADDRESS=0x400000
SH_MEM_CONFIG=0
APPS_FLAGS=0x35510000
RFS_FLAGS=0xb5510000
INTEGRITY_FLAGS=0xab130000

ifeq ($(OS),Windows_NT)
$(AUTH_BOOTIMAGE_BIN_NAME): $(OUTNAME)
	@echo  Boot image: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $(AUTH_BOOTIMAGE_PATH)/$@ ...
ifneq ($(OS),Windows_NT)
	$(CHMOD) a+x $(MULTI_CORE_IMAGE_GEN)
	$(CHMOD) a+x $(CRC_MULTI_CORE_IMAGE_GEN)
	$(CHMOD) a+x $(APPEND_CRC)
	$(CHMOD) a+x $(INTEGRITY_GEN)
endif
	$(OUTRPRC_CMD) $(OUTNAME) $(BOOTIMAGE_RPRC_NAME_TMP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	
	$(INTEGRITY_GEN) $(SYMM_KEY_PATH) $(SALT_PATH) $(INFO_PATH) $(INTEGRITY_BIN) app $(BOOTIMAGE_RPRC_NAME_TMP) rfs $(BSS_RPRC_BIN)
	$(MULTI_CORE_IMAGE_GEN) LE $(BOOT_MODE) $(BOOT_VECTOR_ADDRESS) $(SH_MEM_CONFIG) $(AUTH_BOOTIMAGE_BIN_NAME) $(INTEGRITY_FLAGS) $(INTEGRITY_BIN) $(APPS_FLAGS) $(BOOTIMAGE_RPRC_NAME_TMP) $(RFS_FLAGS) $(BSS_RPRC_BIN) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(CRC_MULTI_CORE_IMAGE_GEN) $(AUTH_BOOTIMAGE_BIN_NAME) $(BOOTIMAGE_BIN_TEMP) >> $(AUTH_BOOTIMAGE_TEMP_OUT_FILE)
	$(APPEND_CRC) $(AUTH_BOOTIMAGE_BIN_NAME) >> $(AUTH_BOOTIMAGE_TEMP_OUT_FILE)
	@echo  Boot image: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $(AUTH_BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .
	$(RM) $(AUTH_BOOTIMAGE_TEMP_OUT_FILE)
	$(RM) $(INTEGRITY_BIN)
endif

$(BOOTIMAGE_BIN_NAME): $(OUTNAME)
	@echo  Boot image: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ ...
ifneq ($(OS),Windows_NT)
	$(CHMOD) a+x $(MULTI_CORE_IMAGE_GEN)
	$(CHMOD) a+x $(CRC_MULTI_CORE_IMAGE_GEN)
	$(CHMOD) a+x $(APPEND_CRC)
endif
	$(OUTRPRC_CMD) $(OUTNAME) $(BOOTIMAGE_RPRC_NAME_TMP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) LE $(BOOT_MODE) $(BOOT_VECTOR_ADDRESS) $(SH_MEM_CONFIG) $(BOOTIMAGE_BIN_NAME) $(APPS_FLAGS) $(BOOTIMAGE_RPRC_NAME_TMP) $(RFS_FLAGS) $(BSS_RPRC_BIN) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(CRC_MULTI_CORE_IMAGE_GEN) $(BOOTIMAGE_BIN_NAME) $(BOOTIMAGE_BIN_TEMP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(APPEND_CRC) $(BOOTIMAGE_BIN_NAME) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	$(RM) $(BOOTIMAGE_BIN_TEMP)
	$(RM) $(BOOTIMAGE_RPRC_NAME_TMP)
	@echo  Boot image: xwrL64xx:m4fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .


-include $(addprefix $(OBJDIR)/, $(DEPS))
