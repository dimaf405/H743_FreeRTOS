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
	Dima/middleware/parameters/definitions/rc_input_mapping_params.c \
	Dima/middleware/parameters/definitions/rc_calibration_1_9_params.c \
	Dima/middleware/parameters/definitions/rc_calibration_10_18_params.c \
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
	Dima/platform/stm32h7/memory/cache.c \
	Dima/platform/stm32h7/memory/early_memory.c \
	Dima/platform/stm32h7/flash/flash_bank1.c
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
	Dima/platform/api/Execution.cpp \
	Dima/platform/api/Flash.cpp \
	Dima/platform/api/Memory.cpp \
	Dima/platform/api/Services.cpp \
	Dima/platform/api/Synchronization.cpp \
	Dima/adapters/usb_console/UsbConsole.cpp \
	Dima/application/app_bootstrap.cpp \
	Dima/rover/ApplicationContext.cpp \
	Dima/rover/control/RoverDifferential.cpp \
	Dima/rover/modes/ManualMode.cpp \
	Dima/lib/timesync/Timesync.cpp \
	Dima/modules/logging/LogService.cpp \
	Dima/modules/mavlink/HeartbeatPacer.cpp \
	Dima/modules/mavlink/MavlinkChannelState.cpp \
	Dima/modules/mavlink/MavlinkCommands.cpp \
	Dima/modules/mavlink/MavlinkIdentity.cpp \
	Dima/modules/mavlink/MavlinkMetadataFtp.cpp \
	Dima/modules/mavlink/MavlinkMission.cpp \
	Dima/modules/mavlink/MavlinkParameterExt.cpp \
	Dima/modules/mavlink/MavlinkParameters.cpp \
	Dima/modules/mavlink/MavlinkRcStream.cpp \
	Dima/modules/mavlink/MavlinkService.cpp \
	Dima/modules/mavlink/MavlinkSystemMessages.cpp \
	Dima/modules/mavlink/MavlinkTimesync.cpp \
	Dima/modules/motor/MotorOutput.cpp \
	Dima/modules/motor/MotorOutputFrames.cpp \
	Dima/modules/motor/MotorOutputParameters.cpp \
	Dima/modules/motor/MotorOutputSafety.cpp \
	Dima/modules/parameters/ParameterService.cpp \
	Dima/modules/parameters/SerialParameterMigration.cpp \
	Dima/modules/serial/SerialConfig.cpp \
	Dima/application/app_main.cpp \
	Dima/lib/rover/DifferentialDrive.cpp \
	Dima/lib/rc/sbus.cpp \
	Dima/modules/rc/SbusRc.cpp \
	Dima/modules/rc/RCUpdate.cpp \
	Dima/modules/safety/Commander.cpp \
	Dima/modules/safety/CommanderActions.cpp \
	Dima/modules/safety/CommanderCommands.cpp \
	Dima/modules/safety/CommanderSafety.cpp \
	Dima/modules/boot_health/BootHealthService.cpp \
	Dima/middleware/lifecycle/module_manager.cpp \
	Dima/middleware/work_queue/WorkQueue.cpp \
	Dima/middleware/uorb/uORB.cpp \
	Dima/middleware/parameters/ParameterJournal.cpp \
	Dima/middleware/parameters/param.cpp \
	Dima/middleware/parameters/param_storage.cpp \
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
	Dima/platform/freertos/Backend.cpp \
	Dima/platform/freertos/HeapOperators.cpp
DIMA_STM32_CXX_SOURCES := \
	Dima/platform/stm32h7/io/ActuatorPwm.cpp \
	Dima/platform/stm32h7/system/BootControl.cpp \
	Dima/platform/stm32h7/system/Clock.cpp \
	Dima/platform/stm32h7/memory/DmaMemory.cpp \
	Dima/platform/stm32h7/flash/FlashDevice.cpp \
	Dima/platform/stm32h7/system/HardwareUid.cpp \
	Dima/platform/stm32h7/system/Watchdog.cpp \
	Dima/platform/stm32h7/serial/SbusUart.cpp \
	Dima/platform/stm32h7/serial/SbusUartHal.cpp \
	Dima/platform/stm32h7/io/SensorInterrupts.cpp \
	Dima/platform/stm32h7/io/UsbCdcTransport.cpp
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

include make/release.mk


-include $(PROJECT_OBJECTS:.o=.d)
