# User-owned build overlay for the CubeMX-generated application Makefile.

# The application always runs from the MCUboot primary slot.  Keep this an
# override so neither a regenerated Makefile nor a command-line typo can move
# the vector table away from 0x08040400.
override LDSCRIPT := Linker/STM32H743VITx_MCUBOOT_APP.ld

BOARD_SD_INIT_AT_BOOT ?= 0
ifneq ($(BOARD_SD_INIT_AT_BOOT),0)
ifneq ($(BOARD_SD_INIT_AT_BOOT),1)
$(error BOARD_SD_INIT_AT_BOOT must be 0 or 1)
endif
endif

APP_HELLO_WORLD_ENABLED ?= 1
ifneq ($(APP_HELLO_WORLD_ENABLED),0)
ifneq ($(APP_HELLO_WORLD_ENABLED),1)
$(error APP_HELLO_WORLD_ENABLED must be 0 or 1)
endif
endif

PYTHON ?= python3
APP_HELLO_WORLD_INTERVAL_MS ?= 1000
APP_HELLO_WORLD_INTERVAL_RAW := $(value APP_HELLO_WORLD_INTERVAL_MS)
unexport APP_HELLO_WORLD_INTERVAL_MS
export APP_HELLO_WORLD_INTERVAL_RAW
APP_HELLO_WORLD_INTERVAL_NON_DIGITS := $(subst 0,,$(subst 1,,$(subst 2,,$(subst 3,,$(subst 4,,$(subst 5,,$(subst 6,,$(subst 7,,$(subst 8,,$(subst 9,,$(APP_HELLO_WORLD_INTERVAL_RAW)))))))))))
ifneq ($(words $(APP_HELLO_WORLD_INTERVAL_RAW)),1)
$(error APP_HELLO_WORLD_INTERVAL_MS must be in 1..2147483647)
endif
ifneq ($(strip $(APP_HELLO_WORLD_INTERVAL_NON_DIGITS)),)
$(error APP_HELLO_WORLD_INTERVAL_MS must be in 1..2147483647)
endif
strip_interval_leading_zeros = $(if $(filter 0%,$(1)),$(call strip_interval_leading_zeros,$(patsubst 0%,%,$(1))),$(1))
APP_HELLO_WORLD_INTERVAL_CANONICAL := $(call strip_interval_leading_zeros,$(APP_HELLO_WORLD_INTERVAL_RAW))
ifeq ($(APP_HELLO_WORLD_INTERVAL_CANONICAL),)
$(error APP_HELLO_WORLD_INTERVAL_MS must be in 1..2147483647)
endif
APP_HELLO_WORLD_INTERVAL_VALIDATED := $(shell APP_HELLO_WORLD_INTERVAL_RAW=$(APP_HELLO_WORLD_INTERVAL_CANONICAL) $(PYTHON) tools/validate_hello_world_interval.py)
ifeq ($(strip $(APP_HELLO_WORLD_INTERVAL_VALIDATED)),)
$(error APP_HELLO_WORLD_INTERVAL_MS must be in 1..2147483647)
endif

C_DEFS += -DH743_APPLICATION_IMAGE \
	-DBOARD_SD_INIT_AT_BOOT=$(BOARD_SD_INIT_AT_BOOT) \
	-DAPP_HELLO_WORLD_ENABLED=$(APP_HELLO_WORLD_ENABLED) \
	-DAPP_HELLO_WORLD_INTERVAL_MS=$(APP_HELLO_WORLD_INTERVAL_VALIDATED)
PROJECT_HEADER_DIRS := \
	Dima/application \
	Dima/product \
	Dima/modules \
	Dima/platform \
	Dima/middleware \
	Dima/messages \
	Dima/lib \
	Dima/adapters \
	Boards/H743/Inc
C_INCLUDES += -I. \
	$(addprefix -I,$(PROJECT_HEADER_DIRS)) \
	-I$(BUILD_DIR)/generated_include \
	-I$(BUILD_DIR)/generated/parameters

PARAMETER_GENERATOR := tools/parameters/generate_parameters.py
PARAMETER_GENERATOR_DEPS := $(wildcard tools/parameters/*.py tools/parameters/dima_params/*.py)
PARAMETER_DEFINITIONS := \
	Dima/middleware/parameters/definitions/commander_params.c \
	Dima/middleware/parameters/definitions/rover_control_params.c \
	Dima/middleware/parameters/definitions/rover_differential_params.c \
	Dima/middleware/parameters/definitions/rc_params.c
PARAMETER_GENERATED_DIR := $(BUILD_DIR)/generated/parameters
PARAMETER_INCLUDE_DIR := $(BUILD_DIR)/generated_include
PARAMETER_GENERATED_STAMP := $(PARAMETER_GENERATED_DIR)/.generated
PARAMETER_GENERATED_OUTPUTS := \
	$(PARAMETER_GENERATED_DIR)/parameters.xml \
	$(PARAMETER_GENERATED_DIR)/parameters.json \
	$(PARAMETER_GENERATED_DIR)/px4_parameters.hpp \
	$(PARAMETER_GENERATED_DIR)/parameter_metadata.c \
	$(PARAMETER_INCLUDE_DIR)/px4_platform_common/param.h \
	$(PARAMETER_INCLUDE_DIR)/px4_platform_common/param_macros.h \
	$(PARAMETER_INCLUDE_DIR)/px4_platform_common/module_params.h \
	$(PARAMETER_INCLUDE_DIR)/parameters/px4_parameters.hpp

$(PARAMETER_GENERATED_STAMP): $(PARAMETER_GENERATOR_DEPS) $(PARAMETER_DEFINITIONS)
	$(PYTHON) $(PARAMETER_GENERATOR) \
		$(foreach source,$(PARAMETER_DEFINITIONS),--source $(source)) \
		--output $(PARAMETER_GENERATED_DIR) \
		--include-output $(PARAMETER_INCLUDE_DIR)
	@touch $@

$(PARAMETER_GENERATED_OUTPUTS): $(PARAMETER_GENERATED_STAMP)
	@test -f $@

.PHONY: parameter-generated
parameter-generated: $(PARAMETER_GENERATED_OUTPUTS)

# newlib-nano lazily allocates its three standard FILE objects on the first
# setvbuf/printf call.  This application has no runtime heap, so link the full
# newlib variant whose _reent embeds stdin/stdout/stderr statically.  Keep the
# generated root Makefile and the independent MCUboot build untouched.
override LDFLAGS := $(filter-out -specs=nano.specs,$(LDFLAGS))

# CubeMX may restore heap_4.c when regenerating its Makefile even though the
# kernel has dynamic allocation disabled.  Remove both the regenerated source
# and its current flat-object name before preserving the CubeMX object set.
FREERTOS_DYNAMIC_HEAP_SOURCE := Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
FREERTOS_DYNAMIC_HEAP_OBJECT := $(BUILD_DIR)/heap_4.o
override C_SOURCES := $(filter-out $(FREERTOS_DYNAMIC_HEAP_SOURCE),$(C_SOURCES))

# The generated ELF prerequisite list is expanded before this overlay is read.
# Make that already-frozen heap target inert as well as removing it from the
# effective source/object variables and final link command.
$(FREERTOS_DYNAMIC_HEAP_OBJECT):
	@mkdir -p $(@D)
	@touch $@

# Project-owned sources are explicit.  Object names retain their source path,
# so sources with identical basenames in different Dima modules cannot collide.
CUBEMX_OBJECTS := $(filter-out $(FREERTOS_DYNAMIC_HEAP_OBJECT),$(OBJECTS))
override OBJECTS := $(filter-out $(FREERTOS_DYNAMIC_HEAP_OBJECT),$(CUBEMX_OBJECTS))
PROJECT_C_SOURCES ?= \
	$(PARAMETER_GENERATED_DIR)/parameter_metadata.c \
	Dima/adapters/mcuboot/mcuboot_app.c \
	Boards/H743/Src/board_init.c \
	Boards/H743/Src/motor_pwm.c \
	Dima/adapters/usb_console/usb_console.c \
	Dima/platform/freertos/libc/cpp_runtime.c \
	Dima/platform/freertos/libc/no_heap.c \
	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_5.c
PROJECT_CXX_SOURCES ?= \
	Dima/application/app_bootstrap.cpp \
	Dima/platform/freertos/dima_heap.cpp \
	Dima/platform/freertos/hrt.cpp \
	Dima/platform/freertos/parameter_flash.cpp \
	Dima/platform/freertos/sbus_uart_backend.cpp \
	Dima/product/rover/ApplicationContext.cpp \
	Dima/product/rover/LogService.cpp \
	Dima/product/rover/ParameterService.cpp \
	Dima/application/app_main.cpp \
	Dima/lib/motor/speed_to_pwm.cpp \
	Dima/lib/rover_control/rover_control.cpp \
	Dima/lib/rc/sbus.cpp \
	Dima/modules/rc/SbusRc.cpp \
	Dima/modules/rc/RCUpdate.cpp \
	Dima/modules/rc/ManualControl.cpp \
	Dima/modules/safety/Commander.cpp \
	Dima/modules/hello_world/hello_world.cpp \
	Dima/modules/boot_health/boot_health.cpp \
	Dima/middleware/lifecycle/module_manager.cpp \
	Dima/platform/freertos/platform_time.cpp \
	Dima/middleware/work_queue/WorkQueue.cpp \
	Dima/middleware/uorb/uORB.cpp \
	Dima/middleware/parameters/param.cpp \
	Dima/middleware/parameters/autosave.cpp \
	Dima/lib/tinybson/tinybson.cpp \
	Dima/middleware/parameters/flashparams/flashparams.cpp \
	Dima/messages/app_heartbeat.cpp \
	Dima/messages/parameter_update.cpp \
	Dima/messages/input_rc.cpp \
	Dima/messages/rc_channels.cpp \
	Dima/messages/manual_control_setpoint.cpp \
	Dima/messages/manual_control_switches.cpp \
	Dima/messages/action_request.cpp \
	Dima/messages/vehicle_status.cpp \
	Dima/messages/vehicle_control_mode.cpp \
	Dima/messages/actuator_armed.cpp \
	Dima/middleware/events/events.cpp \
	Dima/middleware/perf/perf_counter.cpp \
	Dima/middleware/logging/logging.cpp

PROJECT_C_OBJECTS := $(addprefix $(BUILD_DIR)/,$(PROJECT_C_SOURCES:.c=.o))
PROJECT_CXX_OBJECTS := $(addprefix $(BUILD_DIR)/,$(PROJECT_CXX_SOURCES:.cpp=.o))
PROJECT_OBJECTS := $(PROJECT_C_OBJECTS) $(PROJECT_CXX_OBJECTS)
$(PROJECT_OBJECTS): | $(PARAMETER_GENERATED_STAMP)
override OBJECTS += $(PROJECT_OBJECTS)

ifneq ($(strip $(CUBEMX_OBJECTS)),)
$(CUBEMX_OBJECTS): GNUmakefile make/project.mk
endif

ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++
else
CXX = $(PREFIX)g++
endif

CXXFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CXXFLAGS += -std=gnu++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit
ifeq ($(DEBUG), 1)
CXXFLAGS += -g -gdwarf-2
endif
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

ifneq ($(strip $(PROJECT_C_OBJECTS)),)
$(PROJECT_C_OBJECTS): $(BUILD_DIR)/%.o: %.c GNUmakefile Makefile make/project.mk
	@mkdir -p $(@D)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(@:.o=.lst) $< -o $@
endif

ifneq ($(strip $(PROJECT_CXX_OBJECTS)),)
$(PROJECT_CXX_OBJECTS): $(BUILD_DIR)/%.o: %.cpp GNUmakefile Makefile make/project.mk
	@mkdir -p $(@D)
	$(CXX) -c $(CXXFLAGS) $< -o $@
endif

# Add project objects as prerequisites without replacing CubeMX's ELF recipe.
# That recipe intentionally links $(CC) $(OBJECTS), keeping GCC as the final
# link driver and avoiding implicit C++ runtime-library dependencies.
$(BUILD_DIR)/$(TARGET).elf: $(PROJECT_OBJECTS) $(LDSCRIPT) GNUmakefile make/project.mk

# MCUboot image metadata. Override release values at build time, for example:
#   make IMAGE_VERSION=1.2.3+45 KEY_FILE=/secure/path/production.pem
IMAGE_VERSION ?= 0.1.0+0
DEVELOPMENT_KEY = .keys/development-ecdsa-p256.pem
KEY_FILE ?= $(DEVELOPMENT_KEY)
MCUBOOT_ROOT = Middlewares/Third_Party/MCUboot
HOST_TOOLS_CACHE_ROOT ?= $(HOME)/.cache/dima-rover/host-tools
HOST_PYTHON_DIR = $(HOST_TOOLS_CACHE_ROOT)/host-python
HOST_TOOLS_STAMP = $(HOST_TOOLS_CACHE_ROOT)/.host-tools-installed
IMGTOOL = $(MCUBOOT_ROOT)/scripts/imgtool.py
MCUBOOT_BUILD_DIR = $(BUILD_DIR)/mcuboot
SIGNED_BIN = $(BUILD_DIR)/$(TARGET)_signed.bin
SIGNED_HEX = $(BUILD_DIR)/$(TARGET)_signed.hex
FACTORY_HEX = $(BUILD_DIR)/$(TARGET)_factory.hex
IMAGE_VERSION_STAMP = $(BUILD_DIR)/.image-version-$(subst /,_,$(IMAGE_VERSION))
KEY_ID_STAMP ?= $(BUILD_DIR)/.key-identity
KEY_ID_TOOL = tools/update_key_identity.py
MCUMGR ?= mcumgr
MCUMGR_PORT ?=
UPLOAD_WAIT_SECONDS ?= 60
UPLOAD_IMAGE ?= $(SIGNED_BIN)
UPLOAD_TOOL = tools/mcumgr_upload.py

.PHONY: firmware mcuboot host-tools verify dima_rover upload \
        FORCE_MCUBOOT_BUILD FORCE_KEY_IDENTITY_CHECK
firmware: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex \
          $(BUILD_DIR)/$(TARGET).bin $(SIGNED_BIN) \
          $(MCUBOOT_BUILD_DIR)/mcuboot.hex $(FACTORY_HEX)

$(HOST_TOOLS_STAMP): $(MCUBOOT_ROOT)/scripts/requirements.txt
	@set -eu; \
		mkdir -p "$(HOST_TOOLS_CACHE_ROOT)"; \
		tmp="$(HOST_PYTHON_DIR).tmp.$$$$"; \
		old="$(HOST_PYTHON_DIR).old.$$$$"; \
		rm -rf "$$tmp" "$$old"; \
		cleanup() { \
			status=$$?; \
			trap - EXIT HUP INT TERM; \
			rm -rf "$$tmp"; \
			if test -e "$$old"; then \
				if test -e "$(HOST_PYTHON_DIR)"; then \
					rm -rf "$$old"; \
				else \
					mv "$$old" "$(HOST_PYTHON_DIR)"; \
				fi; \
			fi; \
			exit "$$status"; \
		}; \
		trap cleanup EXIT HUP INT TERM; \
		$(PYTHON) -m pip install --disable-pip-version-check \
			--target "$$tmp" -r "$<"; \
		if test -e "$(HOST_PYTHON_DIR)"; then \
			mv "$(HOST_PYTHON_DIR)" "$$old"; \
		fi; \
		if ! mv "$$tmp" "$(HOST_PYTHON_DIR)"; then \
			exit 1; \
		fi; \
		if ! touch "$@"; then \
			rm -rf "$(HOST_PYTHON_DIR)"; \
			exit 1; \
		fi; \
		rm -rf "$$old"; \
		trap - EXIT HUP INT TERM

host-tools: $(HOST_TOOLS_STAMP)

$(DEVELOPMENT_KEY): | $(HOST_TOOLS_STAMP)
	@mkdir -p $(dir $@)
	PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) keygen \
		-k "$@" -t ecdsa-p256
	@chmod 600 $@

FORCE_MCUBOOT_BUILD:

FORCE_KEY_IDENTITY_CHECK:

$(KEY_ID_STAMP): FORCE_KEY_IDENTITY_CHECK $(KEY_ID_TOOL) | $(BUILD_DIR)
	$(PYTHON) "$(KEY_ID_TOOL)" --key "$(KEY_FILE)" --stamp "$@"

ifeq ($(KEY_FILE),$(DEVELOPMENT_KEY))
$(KEY_ID_STAMP): | $(DEVELOPMENT_KEY)
endif

$(MCUBOOT_BUILD_DIR)/mcuboot.hex: FORCE_MCUBOOT_BUILD \
                                      $(KEY_ID_STAMP) $(HOST_TOOLS_STAMP)
	$(MAKE) -f Bootloader/Makefile \
		GCC_PATH=$(GCC_PATH) BUILD_DIR=$(MCUBOOT_BUILD_DIR) \
		KEY_FILE="$(KEY_FILE)" KEY_ID_STAMP="$(KEY_ID_STAMP)" \
		IMGTOOL_PYTHONPATH=$(HOST_PYTHON_DIR) all

$(IMAGE_VERSION_STAMP): | $(BUILD_DIR)
	@rm -f $(BUILD_DIR)/.image-version-*
	@touch $@

$(SIGNED_BIN): $(BUILD_DIR)/$(TARGET).bin $(KEY_ID_STAMP) \
               $(HOST_TOOLS_STAMP) $(IMGTOOL) \
               GNUmakefile make/project.mk $(IMAGE_VERSION_STAMP) | $(BUILD_DIR)
	PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) sign \
		-k "$(KEY_FILE)" --align 32 --max-align 32 \
		-v $(IMAGE_VERSION) -H 0x400 --pad-header -S 0xC0000 \
		$< $@

$(SIGNED_HEX): $(SIGNED_BIN) | $(BUILD_DIR)
	$(CP) -I binary -O ihex --change-addresses 0x08040000 $< $@

$(FACTORY_HEX): $(MCUBOOT_BUILD_DIR)/mcuboot.hex $(SIGNED_HEX) \
                tools/merge_hex.py $(HOST_TOOLS_STAMP) | $(BUILD_DIR)
	PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) tools/merge_hex.py \
		--output $@ $(MCUBOOT_BUILD_DIR)/mcuboot.hex $(SIGNED_HEX)

mcuboot: $(MCUBOOT_BUILD_DIR)/mcuboot.hex


verify: firmware
	PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) verify \
		-k "$(KEY_FILE)" $(SIGNED_BIN)
	PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) tools/verify_mcuboot_images.py \
		--app-elf $(BUILD_DIR)/$(TARGET).elf \
		--boot-elf $(MCUBOOT_BUILD_DIR)/mcuboot.elf \
		--signed $(SIGNED_BIN) --factory $(FACTORY_HEX) \
		--nm $(if $(GCC_PATH),$(GCC_PATH)/$(PREFIX)nm,$(PREFIX)nm)

# Product-facing entry points.  `upload` depends on `dima_rover`, so both
# `make upload` and the requested `make dima_rover upload` build and verify the
# exact signed image before touching the board.
dima_rover: verify

upload: dima_rover $(UPLOAD_TOOL)
	PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(UPLOAD_TOOL) \
		--image "$(UPLOAD_IMAGE)" --imgtool "$(IMGTOOL)" \
		--mcumgr "$(MCUMGR)" \
		$(if $(strip $(MCUMGR_PORT)),--port "$(MCUMGR_PORT)",) \
		--wait-seconds "$(UPLOAD_WAIT_SECONDS)"

-include $(PROJECT_OBJECTS:.o=.d)
