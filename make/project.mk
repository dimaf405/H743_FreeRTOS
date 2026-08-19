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

# GCC assembler listings for C sources add tens of megabytes of text I/O to a
# clean Windows build.  Keep them opt-in for low-level inspection.
DIMA_LISTINGS ?= 0
ifneq ($(filter-out 0 1,$(DIMA_LISTINGS)),)
$(error DIMA_LISTINGS must be 0 or 1)
endif
DIMA_COMMA := ,
DIMA_CUBEMX_LISTING_OPTIONS = -Wa$(DIMA_COMMA)-a$(DIMA_COMMA)-ad$(DIMA_COMMA)-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))
DIMA_PROJECT_LISTING_OPTIONS = -Wa$(DIMA_COMMA)-a$(DIMA_COMMA)-ad$(DIMA_COMMA)-alms=$(@:.o=.lst)
DIMA_CUBEMX_LISTING_FLAG = $(if $(filter 1,$(DIMA_LISTINGS)),$(DIMA_CUBEMX_LISTING_OPTIONS),)
DIMA_PROJECT_LISTING_FLAG = $(if $(filter 1,$(DIMA_LISTINGS)),$(DIMA_PROJECT_LISTING_OPTIONS),)

PYTHON ?= python3
BUILD_PROGRESS_TOOL ?= tools/build_progress.py
DIMA_PROGRESS_STATE ?=
DIMA_PROGRESS_VERBOSE_FLAG = $(if $(filter 1,$(V)),--verbose,)
DIMA_PROGRESS_NO_COLOR_FLAG = $(if $(strip $(NO_COLOR)),--no-color,)
DIMA_PROGRESS_RUN = $(PYTHON) $(BUILD_PROGRESS_TOOL) run \
	--state "$(DIMA_PROGRESS_STATE)" --plan-token DIMA_PROGRESS_STEP_V1 \
	$(DIMA_PROGRESS_VERBOSE_FLAG) $(DIMA_PROGRESS_NO_COLOR_FLAG)
C_DEFS += -DH743_APPLICATION_IMAGE \
	-DBOARD_SD_INIT_AT_BOOT=$(BOARD_SD_INIT_AT_BOOT)
DIMA_COMMON_HEADER_DIRS := \
	Dima \
	Dima/application \
	Dima/rover \
	Dima/modules \
	Dima/middleware \
	Dima/messages \
	Dima/lib \
	Dima/adapters
# CubeMX/board objects retain the generated include surface. Project-owned
# objects below use target-private include sets and never inherit this list.
C_INCLUDES += -I. \
	$(addprefix -I,$(DIMA_COMMON_HEADER_DIRS)) \
	-IBoards/H743/Inc \
	-I$(BUILD_DIR)/generated_include \
	-I$(BUILD_DIR)/generated/parameters \
	-I$(BUILD_DIR)/generated/serial \
	-I$(BUILD_DIR)/generated/component_metadata \
	-I$(BUILD_DIR)/generated/mavlink

MAVLINK_GENERATED_DIR := $(BUILD_DIR)/generated/mavlink
DIMA_GENERATED_INCLUDES := \
	-I$(BUILD_DIR)/generated_include \
	-I$(BUILD_DIR)/generated/parameters \
	-I$(BUILD_DIR)/generated/serial \
	-I$(BUILD_DIR)/generated/component_metadata \
	-I$(MAVLINK_GENERATED_DIR)
DIMA_COMMON_INCLUDES := \
	$(addprefix -I,$(DIMA_COMMON_HEADER_DIRS)) \
	$(DIMA_GENERATED_INCLUDES)
DIMA_FREERTOS_INCLUDES := \
	-IDima \
	-IDima/platform/freertos \
	-IMiddlewares/Third_Party/FreeRTOS/Source/include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
DIMA_STM32_INCLUDES := \
	-IDima \
	-IDima/platform/stm32h7 \
	$(DIMA_GENERATED_INCLUDES) \
	-IBoards/H743/Inc \
	-ICore/Inc \
	-IUSB_DEVICE/App \
	-IUSB_DEVICE/Target \
	-IDrivers/STM32H7xx_HAL_Driver/Inc \
	-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
	-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
	-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
	-IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
	-IDrivers/CMSIS/Include
DIMA_BOARD_INCLUDES := \
	$(DIMA_COMMON_INCLUDES) \
	$(DIMA_FREERTOS_INCLUDES) \
	$(DIMA_STM32_INCLUDES)

PARAMETER_GENERATOR := tools/parameters/generate_parameters.py
ARCHITECTURE_CHECK_TOOL := tools/check_architecture.py
APPLICATION_ELF_CHECK_TOOL := tools/verify_application_elf.py
COMPILE_COMMANDS_TOOL := tools/generate_compile_commands.py
MAVLINK_GENERATOR := tools/mavlink/build_trimmed_dialect.py
MAVLINK_BOOTSTRAP := tools/mavlink/bootstrap_pymavlink.py
MAVLINK_LOCK := tools/mavlink/mavlink.lock.json
MAVLINK_XML_DIR := tools/mavlink/message_definitions
MAVLINK_XML_INPUTS := \
	$(MAVLINK_XML_DIR)/common.xml \
	$(MAVLINK_XML_DIR)/standard.xml \
	$(MAVLINK_XML_DIR)/minimal.xml
MAVLINK_GENERATED_STAMP := $(MAVLINK_GENERATED_DIR)/.generated.json
MAVLINK_LIBRARY_HEADER := $(MAVLINK_GENERATED_DIR)/dima/mavlink.h
MAVLINK_GENERATOR_DEPS := \
	$(MAVLINK_GENERATOR) $(MAVLINK_BOOTSTRAP) $(MAVLINK_LOCK) \
	$(MAVLINK_XML_INPUTS)
COMPILE_COMMANDS_OUTPUT ?= compile_commands.json
PARAMETER_GENERATOR_DEPS := $(wildcard tools/parameters/*.py tools/parameters/dima_params/*.py)
SERIAL_CONFIG_GENERATOR := tools/serial/generate_config.py
SERIAL_PORT_MANIFEST := Boards/H743/serial_ports.json
SERIAL_GENERATED_DIR := $(BUILD_DIR)/generated/serial
SERIAL_GENERATED_STAMP := $(SERIAL_GENERATED_DIR)/.generated
SERIAL_BAUD_PARAMETERS := $(SERIAL_GENERATED_DIR)/serial_baud_params.c
SERIAL_CONFIG_PARAMETERS := $(SERIAL_GENERATED_DIR)/serial_config_params.c
SERIAL_CONFIG_HEADER := $(SERIAL_GENERATED_DIR)/board_serial_config.hpp
SERIAL_GENERATED_OUTPUTS := \
	$(SERIAL_BAUD_PARAMETERS) \
	$(SERIAL_CONFIG_PARAMETERS) \
	$(SERIAL_CONFIG_HEADER)
PARAMETER_DEFINITIONS := \
	Dima/middleware/parameters/definitions/commander_params.c \
	Dima/middleware/parameters/definitions/rover_control_params.c \
	Dima/middleware/parameters/definitions/rover_differential_params.c \
	Dima/middleware/parameters/definitions/rover_actuator_params.c \
	Dima/middleware/parameters/definitions/rc_params.c \
	Dima/middleware/parameters/definitions/qgc_compat_params.c \
	$(SERIAL_CONFIG_PARAMETERS) \
	$(SERIAL_BAUD_PARAMETERS)
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
PARAMETER_METADATA_GENERATOR := tools/mavlink/generate_parameter_metadata.py
PARAMETER_METADATA_DIR := $(BUILD_DIR)/generated/component_metadata
PARAMETER_METADATA_STAMP := $(PARAMETER_METADATA_DIR)/.generated.json
PARAMETER_METADATA_HEADER := $(PARAMETER_METADATA_DIR)/parameter_metadata_files.hpp
PARAMETER_METADATA_OUTPUTS := \
	$(PARAMETER_METADATA_DIR)/component_general.json \
	$(PARAMETER_METADATA_DIR)/component_general.json.xz \
	$(PARAMETER_METADATA_DIR)/parameters.json \
	$(PARAMETER_METADATA_DIR)/parameters.json.xz \
	$(PARAMETER_METADATA_HEADER)
$(SERIAL_GENERATED_STAMP): make/project.mk $(SERIAL_CONFIG_GENERATOR) $(SERIAL_PORT_MANIFEST)
	$(DIMA_PROGRESS_RUN) --label SERIAL --target "$@" \
		--display "$(SERIAL_GENERATED_DIR)" -- \
		$(PYTHON) $(SERIAL_CONFIG_GENERATOR) \
			--manifest $(SERIAL_PORT_MANIFEST) \
			--output $(SERIAL_GENERATED_DIR)
	@touch "$@"

$(SERIAL_GENERATED_OUTPUTS): | $(SERIAL_GENERATED_STAMP)
	@test -f $@

$(PARAMETER_GENERATED_STAMP): make/project.mk $(PARAMETER_GENERATOR_DEPS) \
		$(SERIAL_GENERATED_OUTPUTS) $(PARAMETER_DEFINITIONS)
	$(DIMA_PROGRESS_RUN) --label PARAM --target "$@" \
		--display "$(PARAMETER_GENERATED_DIR)" -- \
		$(PYTHON) $(PARAMETER_GENERATOR) \
		$(foreach source,$(PARAMETER_DEFINITIONS),--source $(source)) \
		--stable-tail-source Dima/middleware/parameters/definitions/qgc_compat_params.c \
		--stable-tail-source $(SERIAL_BAUD_PARAMETERS) \
		--output $(PARAMETER_GENERATED_DIR) \
		--include-output $(PARAMETER_INCLUDE_DIR)
	@touch "$@"

$(PARAMETER_GENERATED_OUTPUTS): | $(PARAMETER_GENERATED_STAMP)
	@test -f $@

.PHONY: parameter-generated
parameter-generated: $(PARAMETER_GENERATED_OUTPUTS)

$(PARAMETER_METADATA_STAMP): make/project.mk $(PARAMETER_METADATA_GENERATOR) \
		$(PARAMETER_GENERATED_DIR)/parameters.json
	$(DIMA_PROGRESS_RUN) --label META --target "$@" \
		--display "$(PARAMETER_METADATA_DIR)" -- \
		$(PYTHON) $(PARAMETER_METADATA_GENERATOR) \
			--parameters $(PARAMETER_GENERATED_DIR)/parameters.json \
			--output $(PARAMETER_METADATA_DIR)
	@touch "$@"

$(PARAMETER_METADATA_OUTPUTS): | $(PARAMETER_METADATA_STAMP)
	@test -f $@

.PHONY: parameter-metadata
parameter-metadata: $(PARAMETER_METADATA_OUTPUTS)

$(MAVLINK_GENERATED_STAMP): make/project.mk $(MAVLINK_GENERATOR_DEPS)
	$(DIMA_PROGRESS_RUN) --label MAVLINK --target "$@" \
		--display "$(MAVLINK_GENERATED_DIR)" -- \
		env PYTHONDONTWRITEBYTECODE=1 $(PYTHON) $(MAVLINK_GENERATOR) \
			--xml-dir $(MAVLINK_XML_DIR) \
			--output-dir $(MAVLINK_GENERATED_DIR) \
			--lock $(MAVLINK_LOCK) \
			--cache-root "$(HOST_TOOLS_CACHE_ROOT)" \
			$(if $(strip $(PYMAVLINK_ROOT)),--pymavlink-root "$(PYMAVLINK_ROOT)",)
	@touch "$@"

$(MAVLINK_LIBRARY_HEADER): | $(MAVLINK_GENERATED_STAMP)
	@test -f "$@"

.PHONY: mavlink
mavlink: $(MAVLINK_LIBRARY_HEADER)

# newlib-nano lazily allocates its three standard FILE objects on the first
# setvbuf/printf call.  Keep standard-stream setup independent of the shared
# platform heap by linking the full newlib variant whose _reent embeds
# stdin/stdout/stderr statically.  Keep the generated root Makefile and the
# independent MCUboot build untouched.
override LDFLAGS := $(filter-out -specs=nano.specs,$(LDFLAGS))

# CubeMX may restore heap_4.c when regenerating its Makefile.  The platform owns
# one explicit heap_5 region, so remove both the regenerated heap_4 source and
# its current flat-object name before preserving the CubeMX object set.
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
DIMA_COMMON_C_SOURCES := \
	$(PARAMETER_GENERATED_DIR)/parameter_metadata.c
DIMA_FREERTOS_C_SOURCES := \
	Dima/platform/freertos/libc/cpp_runtime.c \
	Dima/platform/freertos/libc/no_heap.c \
	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_5.c
DIMA_STM32_C_SOURCES := \
	Dima/platform/stm32h7/cache.c \
	Dima/platform/stm32h7/early_memory.c \
	Dima/platform/stm32h7/flash_bank1.c
DIMA_BOARD_C_SOURCES := \
	Boards/H743/Src/board_init.c \
	Boards/H743/Src/boot_diagnostics.c \
	Boards/H743/Src/motor_pwm.c
PROJECT_C_SOURCES := \
	$(DIMA_COMMON_C_SOURCES) \
	$(DIMA_FREERTOS_C_SOURCES) \
	$(DIMA_STM32_C_SOURCES) \
	$(DIMA_BOARD_C_SOURCES)

DIMA_COMMON_CXX_SOURCES := \
	Dima/platform/api/Platform.cpp \
	Dima/adapters/usb_console/UsbConsole.cpp \
	Dima/application/app_bootstrap.cpp \
	Dima/rover/ApplicationContext.cpp \
	Dima/rover/control/RoverDifferential.cpp \
	Dima/rover/modes/ManualMode.cpp \
	Dima/lib/timesync/Timesync.cpp \
	Dima/modules/logging/LogService.cpp \
	Dima/modules/mavlink/HeartbeatPacer.cpp \
	Dima/modules/mavlink/MavlinkCommands.cpp \
	Dima/modules/mavlink/MavlinkIdentity.cpp \
	Dima/modules/mavlink/MavlinkMetadataFtp.cpp \
	Dima/modules/mavlink/MavlinkMission.cpp \
	Dima/modules/mavlink/MavlinkParameters.cpp \
	Dima/modules/mavlink/MavlinkService.cpp \
	Dima/modules/mavlink/MavlinkTimesync.cpp \
	Dima/modules/motor/MotorOutput.cpp \
	Dima/modules/parameters/ParameterService.cpp \
	Dima/modules/serial/SerialConfig.cpp \
	Dima/application/app_main.cpp \
	Dima/lib/rover/DifferentialDrive.cpp \
	Dima/lib/rc/sbus.cpp \
	Dima/modules/rc/SbusRc.cpp \
	Dima/modules/rc/RCUpdate.cpp \
	Dima/modules/safety/Commander.cpp \
	Dima/modules/boot_health/BootHealthService.cpp \
	Dima/middleware/lifecycle/module_manager.cpp \
	Dima/middleware/work_queue/WorkQueue.cpp \
	Dima/middleware/uorb/uORB.cpp \
	Dima/middleware/parameters/ParameterJournal.cpp \
	Dima/middleware/parameters/param.cpp \
	Dima/middleware/parameters/autosave.cpp \
	Dima/lib/tinybson/tinybson.cpp \
	Dima/middleware/parameters/flashparams/flashparams.cpp \
	Dima/modules/rc/RcManualInput.cpp \
	Dima/messages/parameter_update.cpp \
	Dima/messages/input_rc.cpp \
	Dima/messages/rc_channels.cpp \
	Dima/messages/manual_control_setpoint.cpp \
	Dima/messages/manual_control_switches.cpp \
	Dima/messages/action_request.cpp \
	Dima/messages/vehicle_status.cpp \
	Dima/messages/vehicle_control_mode.cpp \
	Dima/messages/actuator_armed.cpp \
	Dima/messages/actuator_motors.cpp \
	Dima/messages/rover_motion_request.cpp \
	Dima/messages/actuator_output_status.cpp \
	Dima/messages/vehicle_command.cpp \
	Dima/messages/vehicle_command_ack.cpp \
	Dima/messages/mavlink_log.cpp \
	Dima/middleware/events/events.cpp \
	Dima/middleware/perf/perf_counter.cpp \
	Dima/middleware/logging/logging.cpp
DIMA_FREERTOS_CXX_SOURCES := \
	Dima/platform/freertos/Backend.cpp
DIMA_STM32_CXX_SOURCES := \
	Dima/platform/stm32h7/ActuatorPwm.cpp \
	Dima/platform/stm32h7/BootControl.cpp \
	Dima/platform/stm32h7/Clock.cpp \
	Dima/platform/stm32h7/DmaMemory.cpp \
	Dima/platform/stm32h7/FlashDevice.cpp \
	Dima/platform/stm32h7/HardwareUid.cpp \
	Dima/platform/stm32h7/Watchdog.cpp \
	Dima/platform/stm32h7/SbusUart.cpp \
	Dima/platform/stm32h7/SensorInterrupts.cpp \
	Dima/platform/stm32h7/UsbCdcTransport.cpp
DIMA_BOARD_CXX_SOURCES := \
	Boards/H743/Src/platform_composition.cpp
PROJECT_CXX_SOURCES := \
	$(DIMA_COMMON_CXX_SOURCES) \
	$(DIMA_FREERTOS_CXX_SOURCES) \
	$(DIMA_STM32_CXX_SOURCES) \
	$(DIMA_BOARD_CXX_SOURCES)

PROJECT_C_OBJECTS := $(addprefix $(BUILD_DIR)/,$(PROJECT_C_SOURCES:.c=.o))
PROJECT_CXX_OBJECTS := $(addprefix $(BUILD_DIR)/,$(PROJECT_CXX_SOURCES:.cpp=.o))
DIMA_COMMON_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_COMMON_C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_COMMON_CXX_SOURCES:.cpp=.o))
DIMA_FREERTOS_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_FREERTOS_C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_FREERTOS_CXX_SOURCES:.cpp=.o))
DIMA_STM32_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_STM32_C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_STM32_CXX_SOURCES:.cpp=.o))
DIMA_BOARD_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_BOARD_C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_BOARD_CXX_SOURCES:.cpp=.o))
PROJECT_OBJECTS := $(PROJECT_C_OBJECTS) $(PROJECT_CXX_OBJECTS)
$(PROJECT_OBJECTS): | $(PARAMETER_GENERATED_STAMP) $(PARAMETER_METADATA_STAMP) \
	$(MAVLINK_GENERATED_STAMP) check-architecture
override OBJECTS += $(PROJECT_OBJECTS)

$(DIMA_COMMON_OBJECTS): DIMA_PRIVATE_DEFS := $(DIMA_PRODUCT_DEFS)
$(DIMA_COMMON_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_COMMON_INCLUDES)
$(DIMA_FREERTOS_OBJECTS): DIMA_PRIVATE_DEFS :=
$(DIMA_FREERTOS_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_FREERTOS_INCLUDES)
$(DIMA_STM32_OBJECTS): DIMA_PRIVATE_DEFS := $(C_DEFS)
$(DIMA_STM32_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_STM32_INCLUDES)
$(DIMA_BOARD_OBJECTS): DIMA_PRIVATE_DEFS := $(C_DEFS)
$(DIMA_BOARD_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_BOARD_INCLUDES)

ifneq ($(strip $(CUBEMX_OBJECTS)),)
$(CUBEMX_OBJECTS): GNUmakefile make/project.mk | check-architecture
endif

DIMA_REAL_CC := $(CC)
DIMA_REAL_AS := $(AS)
DIMA_REAL_CP := $(CP)
DIMA_REAL_SZ := $(SZ)

ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++
else
CXX = $(PREFIX)g++
endif
DIMA_REAL_CXX := $(CXX)

ifneq ($(strip $(DIMA_PROGRESS_STATE)),)
override CC = $(DIMA_PROGRESS_RUN) --kind cc --target "$@" --source "$<" -- $(DIMA_REAL_CC)
override AS = $(DIMA_PROGRESS_RUN) --kind as --target "$@" --source "$<" -- $(DIMA_REAL_AS)
override CXX = $(DIMA_PROGRESS_RUN) --kind cxx --target "$@" --source "$<" -- $(DIMA_REAL_CXX)
override CP = $(DIMA_PROGRESS_RUN) --kind objcopy --target "$@" --source "$<" -- $(DIMA_REAL_CP)
override SZ = $(DIMA_PROGRESS_RUN) --kind size --target "$@" --source "$<" -- $(DIMA_REAL_SZ)
.SILENT:
endif

DIMA_PROJECT_CFLAGS = $(MCU) $(DIMA_PRIVATE_DEFS) $(DIMA_PRIVATE_INCLUDES) \
	$(OPT) -Wall -Werror -fdata-sections -ffunction-sections
DIMA_PROJECT_CXXFLAGS = $(DIMA_PROJECT_CFLAGS) \
	-std=gnu++17 -fno-exceptions -fno-rtti \
	-fno-threadsafe-statics -fno-use-cxa-atexit
ifeq ($(DEBUG), 1)
DIMA_PROJECT_CFLAGS += -g -gdwarf-2
endif
DIMA_PROJECT_CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Treat warnings from the CubeMX/HAL application object set as build failures.
# Third-party configuration-specific unused arguments are annotated at source;
# do not hide future diagnostics with broad -Wno-* flags.
override CFLAGS += -Werror

ifneq ($(strip $(PROJECT_C_OBJECTS)),)
$(PROJECT_C_OBJECTS): $(BUILD_DIR)/%.o: %.c GNUmakefile Makefile make/project.mk
	@mkdir -p $(@D)
	$(CC) -c $(DIMA_PROJECT_CFLAGS) $(DIMA_PROJECT_LISTING_FLAG) $< -o $@
endif

ifneq ($(strip $(PROJECT_CXX_OBJECTS)),)
$(PROJECT_CXX_OBJECTS): $(BUILD_DIR)/%.o: %.cpp GNUmakefile Makefile make/project.mk
	@mkdir -p $(@D)
	$(CXX) -c $(DIMA_PROJECT_CXXFLAGS) $< -o $@
endif

# Generated MAVLink C library headers (c_library_v2) trigger packed-member
# and alignment warnings, suppressed upstream by PX4 the same way.
$(BUILD_DIR)/Dima/modules/mavlink/%.o: DIMA_PROJECT_CXXFLAGS += \
	-Wno-cast-align -Wno-address-of-packed-member

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
HOST_TOOLS_CACHE_ROOT ?= $(shell $(PYTHON) -c "import pathlib; print((pathlib.Path.home() / '.cache' / 'dima-rover' / 'host-tools').as_posix())")
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
# A forced no-op recipe looks updated to `make -n` even when the real helper
# preserves the stamp mtime.  Resolve that condition read-only while parsing
# so the dry-run and real dependency graph agree.
KEY_IDENTITY_STATUS := $(shell $(PYTHON) "$(KEY_ID_TOOL)" \
	--key "$(KEY_FILE)" --stamp "$(KEY_ID_STAMP)" --status)
KEY_IDENTITY_WILL_CHANGE := $(if $(filter current,$(KEY_IDENTITY_STATUS)),0,1)
KEY_ID_FORCE_PREREQUISITE := $(if $(filter 1,$(KEY_IDENTITY_WILL_CHANGE)),FORCE_KEY_IDENTITY_CHECK,)
MCUMGR ?= mcumgr
MCUMGR_PORT ?=
MCUMGR_BAUD ?= 921600
MCUMGR_MTU ?= 512
MCUMGR_MAX_WINDOW ?= 1
UPLOAD_WAIT_SECONDS ?= 60
UPLOAD_CONFIRM_WAIT_SECONDS ?= 8
UPLOAD_FORCE ?= 0
UPLOAD_VERIFY_CONFIRM ?= 1
UPLOAD_IMAGE ?= $(SIGNED_BIN)
UPLOAD_VERIFY_STAMP = $(BUILD_DIR)/.upload-image-verified
UPLOAD_TOOL = tools/mcumgr_upload.py
MCUMGR_BOOTSTRAP_TOOL = tools/bootstrap_mcumgr.py
UPLOAD_FORCE_FLAG = $(if $(filter 1,$(UPLOAD_FORCE)),--force,)
UPLOAD_CONFIRM_FLAG = $(if $(filter 0,$(UPLOAD_VERIFY_CONFIRM)),--skip-confirm-verification,)

ifneq ($(filter-out 0 1,$(UPLOAD_FORCE)),)
$(error UPLOAD_FORCE must be 0 or 1)
endif
ifneq ($(filter-out 0 1,$(UPLOAD_VERIFY_CONFIRM)),)
$(error UPLOAD_VERIFY_CONFIRM must be 0 or 1)
endif

.PHONY: check-architecture app-check intellisense mavlink firmware mcuboot host-tools verify dima_rover \
		upload-ready upload-preflight upload install-hooks \
		__dima_clean_progress __dima_prepare_generated __dima_summary \
		FORCE_MCUBOOT_BUILD FORCE_KEY_IDENTITY_CHECK

# Stabilize content-addressed/generated outputs before the public wrapper takes
# its dry-run snapshot.  A generator may run because an input timestamp changed
# yet preserve every output timestamp when the content is unchanged; planning
# before that decision overestimates the downstream compile/link actions.
__dima_prepare_generated: $(SERIAL_GENERATED_OUTPUTS) \
		$(PARAMETER_GENERATED_OUTPUTS) $(PARAMETER_METADATA_OUTPUTS) \
		$(MAVLINK_LIBRARY_HEADER)
	@:

check-architecture: $(ARCHITECTURE_CHECK_TOOL)
	$(DIMA_PROGRESS_RUN) --label ARCH --target "$@" -- \
		$(PYTHON) $(ARCHITECTURE_CHECK_TOOL)

# Install git hooks so that architecture check runs automatically on commit.
GIT_HOOKS_DIR := $(shell git rev-parse --show-toplevel 2>/dev/null)/.git/hooks
install-hooks:
	@cp hooks/pre-commit $(GIT_HOOKS_DIR)/pre-commit && \
	chmod +x $(GIT_HOOKS_DIR)/pre-commit && \
	echo "Installed pre-commit hook → $(GIT_HOOKS_DIR)/pre-commit"

# Fast application-only gate for local iterations.  It deliberately excludes
# image signing, MCUboot and Factory HEX generation; `verify` remains the full
# incremental image gate.
app-check: check-architecture $(BUILD_DIR)/$(TARGET).elf
	$(DIMA_PROGRESS_RUN) --label ELF --target "$(BUILD_DIR)/$(TARGET).elf" -- \
		$(PYTHON) $(APPLICATION_ELF_CHECK_TOOL) \
		--elf $(BUILD_DIR)/$(TARGET).elf

intellisense: $(COMPILE_COMMANDS_TOOL)
	$(DIMA_PROGRESS_RUN) --label COMPDB --target "$(COMPILE_COMMANDS_OUTPUT)" -- \
		$(PYTHON) $(COMPILE_COMMANDS_TOOL) \
		--output "$(COMPILE_COMMANDS_OUTPUT)" \
		$(if $(strip $(GCC_PATH)),--gcc-path "$(GCC_PATH)",) \
		--make-variable "BOARD_SD_INIT_AT_BOOT=$(BOARD_SD_INIT_AT_BOOT)" \
		--make-variable "DEBUG=$(DEBUG)"

firmware: mavlink check-architecture \
          $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex \
          $(BUILD_DIR)/$(TARGET).bin $(SIGNED_BIN) \
          $(MCUBOOT_BUILD_DIR)/mcuboot.hex $(FACTORY_HEX)

clean: __dima_clean_progress

__dima_clean_progress:
	$(DIMA_PROGRESS_RUN) --label CLEAN --target "$(BUILD_DIR)" \
		--display "$(BUILD_DIR)" -- rm -fR "$(BUILD_DIR)"

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
		if test "$(V)" = "1"; then \
			printf '%s\n' '+ $(PYTHON) -m pip install --disable-pip-version-check --target $(HOST_PYTHON_DIR) -r $<'; \
		fi; \
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
		trap - EXIT HUP INT TERM; \
		$(DIMA_PROGRESS_RUN) --label HOST --target "$@" \
			--display "$(HOST_PYTHON_DIR)" --quiet-command -- true

host-tools: $(HOST_TOOLS_STAMP)

$(DEVELOPMENT_KEY): | $(HOST_TOOLS_STAMP)
	@mkdir -p $(dir $@)
	$(DIMA_PROGRESS_RUN) --label KEY --target "$@" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) \
		keygen -k "$@" -t ecdsa-p256
	@chmod 600 $@

FORCE_MCUBOOT_BUILD:

FORCE_KEY_IDENTITY_CHECK:

$(KEY_ID_STAMP): $(KEY_ID_FORCE_PREREQUISITE) | $(KEY_ID_TOOL) $(BUILD_DIR)
	$(DIMA_PROGRESS_RUN) --label KEY --target "$@" -- \
		$(PYTHON) "$(KEY_ID_TOOL)" --key "$(KEY_FILE)" --stamp "$@"

ifeq ($(KEY_FILE),$(DEVELOPMENT_KEY))
$(KEY_ID_STAMP): | $(DEVELOPMENT_KEY)
endif

$(MCUBOOT_BUILD_DIR)/mcuboot.hex: FORCE_MCUBOOT_BUILD \
                                      $(KEY_ID_STAMP) $(HOST_TOOLS_STAMP)
	$(MAKE) -f Bootloader/Makefile \
		GCC_PATH=$(GCC_PATH) BUILD_DIR=$(MCUBOOT_BUILD_DIR) \
		KEY_FILE="$(KEY_FILE)" KEY_ID_STAMP="$(KEY_ID_STAMP)" \
		IMGTOOL_PYTHONPATH=$(HOST_PYTHON_DIR) PYTHON="$(PYTHON)" \
		DIMA_PROGRESS_STATE="$(DIMA_PROGRESS_STATE)" \
		KEY_IDENTITY_CHECKED_BY_PARENT=1 \
		KEY_IDENTITY_WILL_CHANGE=$(KEY_IDENTITY_WILL_CHANGE) \
		V="$(V)" NO_COLOR="$(NO_COLOR)" all

$(IMAGE_VERSION_STAMP): | $(BUILD_DIR)
	@rm -f $(BUILD_DIR)/.image-version-*
	@touch $@

$(SIGNED_BIN): $(BUILD_DIR)/$(TARGET).bin $(KEY_ID_STAMP) \
               $(HOST_TOOLS_STAMP) $(IMGTOOL) \
               GNUmakefile make/project.mk $(IMAGE_VERSION_STAMP) | $(BUILD_DIR)
	$(DIMA_PROGRESS_RUN) --label SIGN --target "$@" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) \
		sign -k "$(KEY_FILE)" --align 32 --max-align 32 \
		-v $(IMAGE_VERSION) -H 0x400 --pad-header -S 0xC0000 $< $@

# Routine MCUboot OTA only consumes the signed application.  Cache its two
# direct gates so an unchanged image does not rebuild MCUboot/Factory HEX or
# rerun the full release-layout verification on every upload.
$(UPLOAD_VERIFY_STAMP): $(BUILD_DIR)/$(TARGET).elf $(SIGNED_BIN) \
                        $(APPLICATION_ELF_CHECK_TOOL) $(IMGTOOL) | $(BUILD_DIR)
	$(DIMA_PROGRESS_RUN) --label ELF --target "$@" \
		--display "$(BUILD_DIR)/$(TARGET).elf" -- \
		$(PYTHON) $(APPLICATION_ELF_CHECK_TOOL) \
		--elf $(BUILD_DIR)/$(TARGET).elf
	$(DIMA_PROGRESS_RUN) --label VERIFY --target "$@" \
		--display "$(SIGNED_BIN)" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) \
		verify -k "$(KEY_FILE)" $(SIGNED_BIN)
	@touch "$@"

ifeq ($(UPLOAD_IMAGE),$(SIGNED_BIN))
upload-ready: $(UPLOAD_VERIFY_STAMP)
else
# An external package has no trustworthy relationship to the local ELF stamp.
# Verify the exact file on every invocation before the uploader may consume it.
upload-ready: $(UPLOAD_IMAGE) $(HOST_TOOLS_STAMP) $(IMGTOOL)
	$(DIMA_PROGRESS_RUN) --label VERIFY --target "$(UPLOAD_IMAGE)" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) \
		verify -k "$(KEY_FILE)" "$(UPLOAD_IMAGE)"
endif

$(SIGNED_HEX): $(SIGNED_BIN) | $(BUILD_DIR)
	$(CP) -I binary -O ihex --change-addresses 0x08040000 $< $@

$(FACTORY_HEX): $(MCUBOOT_BUILD_DIR)/mcuboot.hex $(SIGNED_HEX) \
                tools/merge_hex.py $(HOST_TOOLS_STAMP) | $(BUILD_DIR)
	$(DIMA_PROGRESS_RUN) --label MERGE --target "$@" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) tools/merge_hex.py \
		--output $@ $(MCUBOOT_BUILD_DIR)/mcuboot.hex $(SIGNED_HEX)

mcuboot: $(MCUBOOT_BUILD_DIR)/mcuboot.hex


verify: check-architecture firmware
	$(DIMA_PROGRESS_RUN) --label ELF --target "$(BUILD_DIR)/$(TARGET).elf" -- \
		$(PYTHON) $(APPLICATION_ELF_CHECK_TOOL) \
		--elf $(BUILD_DIR)/$(TARGET).elf
	$(DIMA_PROGRESS_RUN) --label VERIFY --target "$(SIGNED_BIN)" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) \
		verify -k "$(KEY_FILE)" $(SIGNED_BIN)
	$(DIMA_PROGRESS_RUN) --label VERIFY --target "$(FACTORY_HEX)" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) tools/verify_mcuboot_images.py \
		--app-elf $(BUILD_DIR)/$(TARGET).elf \
		--boot-elf $(MCUBOOT_BUILD_DIR)/mcuboot.elf \
		--signed $(SIGNED_BIN) --factory $(FACTORY_HEX) \
		--nm $(if $(GCC_PATH),$(GCC_PATH)/$(PREFIX)nm,$(PREFIX)nm)

# `make dima_rover` remains the full release-image gate.  When upload is also
# requested, the long-standing `make dima_rover upload` spelling is normalized
# to the cached application-only OTA gate instead of rebuilding Factory HEX.
ifneq ($(filter upload,$(MAKECMDGOALS)),)
dima_rover: upload-ready
else
dima_rover: check-architecture verify
endif

upload-preflight: $(UPLOAD_TOOL) $(MCUMGR_BOOTSTRAP_TOOL) $(MAVLINK_BOOTSTRAP) $(MAVLINK_LOCK)
	@env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(UPLOAD_TOOL) \
		--preflight-only --mcumgr "$(MCUMGR)" \
		--tools-cache "$(HOST_TOOLS_CACHE_ROOT)" \
		$(if $(strip $(MCUMGR_PORT)),--port "$(MCUMGR_PORT)",) \
		--baud "$(MCUMGR_BAUD)" --mtu "$(MCUMGR_MTU)" \
		--wait-seconds "$(UPLOAD_WAIT_SECONDS)"

upload: dima_rover $(UPLOAD_TOOL) $(MCUMGR_BOOTSTRAP_TOOL) $(MAVLINK_BOOTSTRAP) $(MAVLINK_LOCK)
	$(DIMA_PROGRESS_RUN) --label UPLOAD --target "$(UPLOAD_IMAGE)" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(UPLOAD_TOOL) \
		--image "$(UPLOAD_IMAGE)" --imgtool "$(IMGTOOL)" \
		--mcumgr "$(MCUMGR)" --tools-cache "$(HOST_TOOLS_CACHE_ROOT)" \
		$(if $(strip $(MCUMGR_PORT)),--port "$(MCUMGR_PORT)",) \
		--baud "$(MCUMGR_BAUD)" --mtu "$(MCUMGR_MTU)" \
		--max-window "$(MCUMGR_MAX_WINDOW)" \
		--wait-seconds "$(UPLOAD_WAIT_SECONDS)" \
		--confirm-wait-seconds "$(UPLOAD_CONFIRM_WAIT_SECONDS)" \
		$(UPLOAD_FORCE_FLAG) $(UPLOAD_CONFIRM_FLAG)

__dima_summary:
	$(PYTHON) $(BUILD_PROGRESS_TOOL) summary \
		--goals "$(DIMA_SUMMARY_GOALS)" --version "$(IMAGE_VERSION)" \
		--app-elf "$(BUILD_DIR)/$(TARGET).elf" \
		--boot-bin "$(MCUBOOT_BUILD_DIR)/mcuboot.bin" \
		--signed "$(SIGNED_BIN)" --factory "$(FACTORY_HEX)" \
		--layout "Boards/H743/Inc/boot_layout.h" \
		--size-tool "$(DIMA_REAL_SZ)" \
		$(DIMA_PROGRESS_NO_COLOR_FLAG)

-include $(PROJECT_OBJECTS:.o=.d)
