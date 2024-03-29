PROJECT_NAME := ble_app_uart_s110_pca10028

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

ifndef NORDIC_SDK_PATH
    NORDIC_SDK_PATH := /opt/nrf51_sdk_10.0.0
endif

TEMPLATE_PATH = $(NORDIC_SDK_PATH)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO :=
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
./src/main.c \
./src/timer_issue.c \
./src/app_timer.c \
$(NORDIC_SDK_PATH)/components/ble/ble_advertising/ble_advertising.c \
$(NORDIC_SDK_PATH)/components/ble/common/ble_advdata.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/delay/nrf_delay.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/pstorage/pstorage.c \
$(NORDIC_SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
$(NORDIC_SDK_PATH)/components/toolchain/system_nrf51.c \


#assembly files common to all targets
ASM_SOURCE_FILES  = gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS  = -I./config
INC_PATHS += -I./include
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/ble_advertising
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/common
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/device
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/delay
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/hal
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/pstorage
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/pstorage/config
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/timer
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/trace
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/util
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/softdevice/s110/headers
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/toolchain
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/toolchain/gcc

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS110
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu11
CFLAGS += -Werror
CFLAGS += -Wall -fomit-frame-pointer -O3 -flto -fno-math-errno
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-strict-aliasing

# CFLAGS += -DDEBUG
# INC_PATHS += -I$(NORDIC_SDK_PATH)/examples/bsp

LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0 -flto
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections -ggdb
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS110
#ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD

#default target - first one defined
default: clean nrf51822_s110

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51822_s110

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51822_s110
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51822_s110: OUTPUT_FILENAME := nrf51822_s110
nrf51822_s110: LINKER_SCRIPT=ble_app_uart_gcc_nrf51.ld
nrf51822_s110: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex:
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

merge:
	## Merge softdevice and bootloader:
	mergehex -m $(NORDIC_SDK_PATH)/components/softdevice/s110/hex/s110_nrf51_8.0.0_softdevice.hex \
				bootloader/_build/nrf51822_s110_bootloader.hex \
			-o $(OUTPUT_BINARY_DIRECTORY)/DFU_A.hex

	## Merge softdevice+bootloader and user application
	mergehex -m $(OUTPUT_BINARY_DIRECTORY)/DFU_A.hex \
				$(OUTPUT_BINARY_DIRECTORY)/nrf51822_s110.hex \
			-o _build/DFU_B.hex

	## Merge softdevice+bootloader+application and bootloader settings file
	mergehex -m $(OUTPUT_BINARY_DIRECTORY)/DFU_B.hex \
				resources/bootloader_settings.hex \
			-o $(OUTPUT_BINARY_DIRECTORY)/merged_diver_dongle.hex

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --sectorerase
	nrfjprog --reset

## Flash softdevice
flash_softdevice:
	@echo Flashing: s110_nrf51_8.0.0_softdevice.hex
	nrfjprog --program $(NORDIC_SDK_PATH)/components/softdevice/s110/hex/s110_nrf51_8.0.0_softdevice.hex -f nrf51 --chiperase
	nrfjprog --reset
