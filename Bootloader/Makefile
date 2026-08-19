PROJECT_ROOT := .
DIMA_WINDOWS_NATIVE := $(filter Windows_NT,$(OS))
ifeq ($(DIMA_WINDOWS_NATIVE),)
$(error Windows-native GNU Make is required (OS=$(OS)); WSL/Linux builds are disabled)
endif

DIMA_USERPROFILE_POSIX := $(subst \,/,$(USERPROFILE))
DIMA_PLATFORMIO_PYTHON := $(DIMA_USERPROFILE_POSIX)/.platformio/penv/Scripts/python.exe

TARGET ?= mcuboot
BUILD_DIR ?= build/mcuboot
KEY_FILE ?= .keys/development-ecdsa-p256.pem
KEY_ID_STAMP ?= $(BUILD_DIR)/.key-identity
KEY_ID_TOOL ?= tools/update_key_identity.py
PYTHON ?= $(if $(wildcard $(DIMA_PLATFORMIO_PYTHON)),$(DIMA_PLATFORMIO_PYTHON),python.exe)
IMGTOOL_PYTHONPATH ?= build/host-python
BUILD_PROGRESS_TOOL ?= tools/build_progress.py
DIMA_PROGRESS_STATE ?=
KEY_IDENTITY_CHECKED_BY_PARENT ?= 0
DIMA_PROGRESS_VERBOSE_FLAG = $(if $(filter 1,$(V)),--verbose,)
DIMA_PROGRESS_NO_COLOR_FLAG = $(if $(strip $(NO_COLOR)),--no-color,)
DIMA_PROGRESS_RUN = $(PYTHON) $(BUILD_PROGRESS_TOOL) run \
	--state "$(DIMA_PROGRESS_STATE)" --plan-token DIMA_PROGRESS_STEP_V1 \
	$(DIMA_PROGRESS_VERBOSE_FLAG) $(DIMA_PROGRESS_NO_COLOR_FLAG)

PREFIX ?= arm-none-eabi-
ifdef GCC_PATH
CC := $(GCC_PATH)/$(PREFIX)gcc
AS := $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP := $(GCC_PATH)/$(PREFIX)objcopy
SZ := $(GCC_PATH)/$(PREFIX)size
else
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc -x assembler-with-cpp
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size
endif

DIMA_REAL_CC := $(CC)
DIMA_REAL_AS := $(AS)
DIMA_REAL_CP := $(CP)
DIMA_REAL_SZ := $(SZ)

ifneq ($(strip $(DIMA_PROGRESS_STATE)),)
override CC = $(DIMA_PROGRESS_RUN) --kind cc --target "$@" --source "$<" -- $(DIMA_REAL_CC)
override AS = $(DIMA_PROGRESS_RUN) --kind as --target "$@" --source "$<" -- $(DIMA_REAL_AS)
override CP = $(DIMA_PROGRESS_RUN) --kind objcopy --target "$@" --source "$<" -- $(DIMA_REAL_CP)
override SZ = $(DIMA_PROGRESS_RUN) --kind size --target "$@" --source "$<" -- $(DIMA_REAL_SZ)
.SILENT:
endif

MCUBOOT_ROOT := Middlewares/Third_Party/MCUboot
IMGTOOL ?= $(MCUBOOT_ROOT)/scripts/imgtool.py
PUBLIC_KEY_C := $(BUILD_DIR)/generated/ecdsa_pub_key.c

C_SOURCES := \
Bootloader/Src/main.c \
Bootloader/Src/boot_compat.c \
Boards/H743/Src/boot_diagnostics_store.c \
Dima/platform/stm32h7/cache.c \
Dima/platform/stm32h7/flash_bank1.c \
Bootloader/Src/boot_platform.c \
Bootloader/Src/boot_primary.c \
Bootloader/Src/boot_watchdog.c \
Bootloader/Src/boot_usb.c \
Bootloader/Src/flash_map_backend.c \
Bootloader/Src/keys.c \
Bootloader/Src/stm32h7xx_it.c \
Bootloader/Src/usbd_desc.c \
USB_DEVICE/Target/usbd_conf.c \
Core/Src/system_stm32h7xx.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/bootutil_area.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/bootutil_find_key.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/bootutil_img_hash.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/bootutil_img_security_cnt.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/bootutil_loader.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/bootutil_misc.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/bootutil_public.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/caps.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/fault_injection_hardening.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/image_ecdsa.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/image_validate.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/loader.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/swap_misc.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/swap_scratch.c \
$(MCUBOOT_ROOT)/boot/bootutil/src/tlv.c \
$(MCUBOOT_ROOT)/boot/boot_serial/src/boot_serial.c \
$(MCUBOOT_ROOT)/boot/boot_serial/src/zcbor_bulk.c \
$(MCUBOOT_ROOT)/boot/zcbor/src/zcbor_common.c \
$(MCUBOOT_ROOT)/boot/zcbor/src/zcbor_decode.c \
$(MCUBOOT_ROOT)/boot/zcbor/src/zcbor_encode.c \
$(MCUBOOT_ROOT)/ext/tinycrypt/lib/source/ecc.c \
$(MCUBOOT_ROOT)/ext/tinycrypt/lib/source/ecc_dsa.c \
$(MCUBOOT_ROOT)/ext/tinycrypt/lib/source/sha256.c \
$(MCUBOOT_ROOT)/ext/tinycrypt/lib/source/utils.c \
$(MCUBOOT_ROOT)/ext/mbedtls-asn1/src/asn1parse.c \
$(MCUBOOT_ROOT)/ext/mbedtls-asn1/src/platform_util.c \
$(PUBLIC_KEY_C)

ASM_SOURCES := startup_stm32h743xx.s

MCU := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
C_DEFS := -DUSE_HAL_DRIVER -DSTM32H743xx
C_INCLUDES := \
-I. \
-IBoards/H743/Inc \
-IDima/platform/stm32h7 \
-IBootloader/Inc \
-ICore/Inc \
-IUSB_DEVICE/Target \
-IDrivers/STM32H7xx_HAL_Driver/Inc \
-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
-IDrivers/CMSIS/Include \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-I$(MCUBOOT_ROOT)/boot/bootutil/include \
-I$(MCUBOOT_ROOT)/boot/bootutil/src \
-I$(MCUBOOT_ROOT)/boot/boot_serial/include \
-I$(MCUBOOT_ROOT)/boot/boot_serial/src \
-I$(MCUBOOT_ROOT)/boot/zcbor/include \
-I$(MCUBOOT_ROOT)/ext/tinycrypt/lib/include \
-I$(MCUBOOT_ROOT)/ext/mbedtls-asn1/include

CFLAGS := $(MCU) $(C_DEFS) $(C_INCLUDES) -std=gnu11 -Os -g3 -Wall -Wextra -Werror \
          -ffunction-sections -fdata-sections -MMD -MP
ASFLAGS := $(MCU) -Os -g3 -Wall -ffunction-sections -fdata-sections
LDSCRIPT := Bootloader/Linker/STM32H743_MCUBOOT.ld
LDFLAGS := $(MCU) -specs=nano.specs -T$(LDSCRIPT) -lc -lm -lnosys \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

OBJECTS := $(addprefix $(BUILD_DIR)/obj/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/obj/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

ifeq ($(KEY_IDENTITY_CHECKED_BY_PARENT),1)
KEY_ID_FORCE_PREREQUISITE :=
else
KEY_IDENTITY_STATUS := $(shell $(PYTHON) "$(KEY_ID_TOOL)" \
	--key "$(KEY_FILE)" --stamp "$(KEY_ID_STAMP)" --status)
KEY_IDENTITY_WILL_CHANGE := $(if $(filter current,$(KEY_IDENTITY_STATUS)),0,1)
KEY_ID_FORCE_PREREQUISITE := $(if $(filter 1,$(KEY_IDENTITY_WILL_CHANGE)),FORCE_KEY_IDENTITY_CHECK,)
endif
PUBLIC_KEY_FORCE_PREREQUISITE := $(if $(filter 1,$(KEY_IDENTITY_WILL_CHANGE)),FORCE_KEY_IDENTITY_CHECK,)

.PHONY: all clean FORCE_KEY_IDENTITY_CHECK
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

FORCE_KEY_IDENTITY_CHECK:

ifeq ($(KEY_IDENTITY_CHECKED_BY_PARENT),1)
# The parent makes this stamp before entering the real sub-Make.  During a
# dry-run it intentionally does not create the file, so use an empty rule here
# instead of planning the same KEY action a second time.
$(KEY_ID_STAMP): ;
else
$(KEY_ID_STAMP): $(KEY_ID_FORCE_PREREQUISITE) | $(KEY_ID_TOOL)
	$(DIMA_PROGRESS_RUN) --label KEY --target "$@" -- \
		$(PYTHON) "$(KEY_ID_TOOL)" --key "$(KEY_FILE)" --stamp "$@"
endif

$(PUBLIC_KEY_C): $(PUBLIC_KEY_FORCE_PREREQUISITE) $(KEY_ID_STAMP) $(IMGTOOL)
	@mkdir -p $(dir $@)
	$(DIMA_PROGRESS_RUN) --label PUBKEY --target "$@" -- \
		env PYTHONPATH=$(IMGTOOL_PYTHONPATH) $(PYTHON) \
		"$(IMGTOOL)" getpub -k "$(KEY_FILE)" -o "$@"

$(BUILD_DIR)/obj/ecdsa_pub_key.o: $(PUBLIC_KEY_C) | $(BUILD_DIR)/obj
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/obj/%.o: %.c Bootloader/Makefile | $(BUILD_DIR)/obj
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/obj/%.o: %.s Bootloader/Makefile | $(BUILD_DIR)/obj
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Bootloader/Makefile $(LDSCRIPT)
	@set -eu; \
		tmp="$@.tmp.$$$$"; \
		cleanup() { rm -f "$$tmp"; }; \
		trap cleanup EXIT HUP INT TERM; \
		$(CC) $(OBJECTS) $(LDFLAGS) -o "$$tmp"; \
		mv -f "$$tmp" "$@"; \
		trap - EXIT HUP INT TERM
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary -S $< $@

$(BUILD_DIR)/obj:
	@mkdir -p $@

clean:
	$(RM) -r $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/obj/*.d)
