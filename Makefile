CYGWIN=nodosfilewarning

# setup executables
TOOLCHAIN_PATH ?= /usr/local/gcc-arm-none-eabi-5_4-2016q3/bin
TOOLCHAIN_PREFIX ?= arm-none-eabi-
CC_PREFIX ?= $(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)
CC = $(CC_PREFIX)gcc
OBJCOPY		 = $(CC_PREFIX)objcopy
OBJDUMP		 = $(CC_PREFIX)objdump
SIZE		 = $(CC_PREFIX)size
CO_FLASH     = /cygdrive/c/CooCox/CoIDE/bin/coflash.exe

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=/usr/share/openocd/scripts/board

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=extra/stm32f4-openocd.cfg

# indicate which platform we're building for
TARGET ?= STM32F4_DISC1
TARGETS = STM32F4_DISC1

# check for valid target
ifeq ($(TARGET),$(filter $(TARGET),$(TARGETS)),)
$(error Invalid Target '$(TARGET)'. Valid targets are $(TARGETS))
endif

USER_OPTIONS ?=

# directories
ROOT         := .
SRC_DIR		 = $(ROOT)
OBJ_DIR	     = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/bin
COOS_DIR     = $(SRC_DIR)/CoOS
TARGET_HEX   = $(BIN_DIR)/$(TARGET).hex
TARGET_ELF   = $(BIN_DIR)/$(TARGET).elf
TARGET_DIS   = $(BIN_DIR)/$(TARGET).dis
TARGET_MAP   = $(BIN_DIR)/$(TARGET).map

COMMON_CFLAGS = -ffunction-sections \
                -fdata-sections \
				-g \
				-Wall \
				-Wextra

OPTIMISE_FLAGS = -O2


INCLUDE_DIRS = \
			$(SRC_DIR) \
			$(SRC_DIR)/drivers \
			$(SRC_DIR)/timers \
			$(SRC_DIR)/tasks \
			$(COOS_DIR) \
			$(COOS_DIR)/portable \
			$(COOS_DIR)/kernel

ifeq ($(TARGET),STM32F4_DISC1)

INCLUDE_DIRS := $(INCLUDE_DIRS) \
				$(SRC_DIR)/cmsis_boot/startup \
				$(SRC_DIR)/cmsis_boot \
				$(SRC_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc \
				$(SRC_DIR)/cmsis


CPU_FLAGS = -mcpu=cortex-m4 -mthumb
CPU_DEFINES = -DSTM32F4XX

# use hardware floating point
FPU_FLAGS = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
FPU_DEFINES = -D__FPU_USED

LINK_SCRIPT = $(ROOT)/link.ld
CO_FLASH_PROCESSOR_TYPE = STM32F407VG

CMSIS_BOOT_SRC = $(SRC_DIR)/cmsis_boot/startup/startup_stm32f4xx.c \
                 $(SRC_DIR)/cmsis_boot/*.c

STD_PERIPHERAL_LIB_SRC = $(SRC_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/*.c

TARGET_SRC = $(CMSIS_BOOT_SRC) \
             $(STD_PERIPHERAL_LIB_SRC)

endif

#LTO_FLAGS	 = -flto -fuse-linker-plugin

CFLAGS = $(COMMON_CFLAGS) \
			$(OPTIMISE_FLAGS) \
			-D'__TARGET__="$(TARGET)"' \
			-DUSE_STDPERIPH_DRIVER \
			$(addprefix -I,$(INCLUDE_DIRS)) \
			$(CPU_FLAGS) \
			$(CPU_DEFINES) \
			$(FPU_FLAGS) \
			$(FPU_DEFINES) \
			$(addprefix -D,$(PLATFORM_FLAGS)) \
			$(addprefix -D,$(USER_OPTIONS)) \
			$(LTO_FLAGS) \
			-MMD

COMMON_LDFLAGS = -g \
                 -Wall \
                 $(OPTIMISE_FLAGS) \
                 -Wl,--gc-sections \
                 -nostartfiles \
                 -lm
		         --specs=nano.specs \
		         -lc \
		         -lnosys

LDFLAGS = $(CPU_FLAGS) \
          $(FPU_FLAGS) \
          $(COMMON_LDFLAGS) \
          -Wl,-Map=$(TARGET_MAP) \
          $(LTO_FLAGS) \
          -T$(LINK_SCRIPT)

# now specify source files
COMMON_SRC = \
	$(SRC_DIR)/main.c \
	$(SRC_DIR)/pulser.c \
	$(SRC_DIR)/injector_output.c \
	$(SRC_DIR)/timers/*.c \
	$(SRC_DIR)/drivers/serial.c \
	$(SRC_DIR)/drivers/uart.c \
	$(SRC_DIR)/drivers/usart.c \
	$(SRC_DIR)/tasks/*.c

COOS_SRC = $(COOS_DIR)/kernel/*.c \
           $(COOS_DIR)/portable/*.c

COOS_SRC_NO_LTO = $(wildcard $(COOS_DIR)/portable/GCC/*.c)


SRC_FILES = $(COMMON_SRC) \
            $(TARGET_SRC) \
            $(STM32_SRC) \
            $(COOS_SRC)

SRC_FILES_NO_LTO = $(COOS_SRC_NO_LTO) \
				$(SRC_DIR)/syscalls/*.c


# add .c files to object list
OBJS = $(patsubst %.c,%.o,$(wildcard $(SRC_FILES)))
# add .S files to object list
OBJS := $(patsubst %.S,%.o,$(OBJS))
OBJS := $(patsubst %.s,%.o,$(OBJS))
# prepend obj directory to object list
OBJS := $(OBJS:%=$(OBJ_DIR)/%)

OBJS_NO_LTO = $(patsubst %.c,%.o,$(wildcard $(SRC_FILES_NO_LTO)))
# prepend obj directory to object list
OBJS_NO_LTO := $(OBJS_NO_LTO:%=$(OBJ_DIR)/no_lto/%)


TARGET_DEPENDENCIES = $(patsubst %.o,%.d,$(OBJS)) $(patsubst %.o,%.d,$(OBJS_NO_LTO))


all: $(TARGET_HEX)

program: $(TARGET_HEX)
	openocd -f $(OPENOCD_PROC_FILE) -c "stm32f4_flash $<" -c shutdown

flash: all
	$(CO_FLASH) program $(CO_FLASH_PROCESSOR_TYPE) $(TARGET_ELF) --adapter-name=ST-Link

dump: all
	$(OBJDUMP) -D $(TARGET_ELF) > $(TARGET_DIS)

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJS) $(OBJS_NO_LTO)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(OBJ_DIR)/no_lto/%.o: CFLAGS_NO_LTO=$(filter-out -flto, $(CFLAGS))
$(OBJ_DIR)/no_lto/%.o : %.c
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS_NO_LTO)

$(OBJ_DIR)/%.o : %.c
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS)

$(OBJ_DIR)/%.o: %.s
	echo $<
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS)

$(OBJ_DIR)/%.o: %.S
	echo $<
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS)


.PHONY:	clean


clean:
	rm -rf $(OBJS)
	rm -rf $(TARGET_ELF)
	rm -rf $(TARGET_HEX)

-include $(TARGET_DEPENDENCIES)

