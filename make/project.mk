# User-owned build overlay for the CubeMX-generated application Makefile.
# 本文件是项目源码与生成合同的权威构建闭包；不得把派生头、参数表或消息表
# 复制回源码树手工维护。

# The application always runs from the MCUboot primary slot.  Keep this an
# override so neither a regenerated Makefile nor a command-line typo can move
# the vector table away from 0x08040400.
# Primary slot 从 0x08040000 开始，前 0x400 字节属于 MCUboot header，故应用
# 向量必须固定在 0x08040400；这里用 override 阻止外部参数静默改址。
override LDSCRIPT := Linker/STM32H743VITx_MCUBOOT_APP.ld

# Application builds default to size optimization while retaining DWARF for
# post-build inspection.  The independently-built MCUboot image does not
# include this overlay and keeps its existing optimization policy.
ifeq ($(DIMA_BUILD_PROFILE),release)
override OPT := -Os
else
override OPT := -Og
endif
override DEBUG := 1

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
# 普通 .data/.bss 位于 D2 SRAM1/2；该宏保证 SystemInit 在启动汇编复制、
# 清零这些段之前打开对应 SRAM 时钟，不能仅依赖链接地址完成迁移。
# SDMMC_DATATIMEOUT=0xffffffff 是外设数据计数器上限，若误作 HAL 毫秒等待约为
# 49.7 天；产品合同把所有 SD 同步调用的单次阻塞限制在 500 ms。
DIMA_PRODUCT_DEFS := -DH743_APPLICATION_IMAGE -DDATA_IN_D2_SRAM \
	-DDIMA_SD_BLOCKING_TIMEOUT_MS=500U
C_DEFS += $(DIMA_PRODUCT_DEFS)
MAVLINK_GENERATED_DIR := $(BUILD_DIR)/generated/mavlink
DRONECAN_GENERATED_DIR := $(BUILD_DIR)/generated/dronecan
FIRMWARE_IDENTITY_GENERATED_DIR := $(BUILD_DIR)/generated/firmware_identity
SENSOR_DEVICE_GENERATED_DIR := $(BUILD_DIR)/generated/sensor_devices
# 所有生成物只存在于 build/generated*；源码侧 JSON/msg/lock 是权威输入，
# C/C++ 消费者只能包含生成合同，不能再建立名称、ID 或 source 列表副本。
DIMA_PLATFORM_INCLUDES := -IDima/platform
DIMA_STM32H7_PLATFORM_INCLUDES := -IDima/platform/stm32h7
DIMA_FREERTOS_CONFIG_INCLUDES := -IDima/platform/freertos
DIMA_MIDDLEWARE_INCLUDES := -IDima/middleware
DIMA_PARAMETER_MIDDLEWARE_INCLUDES := -IDima/middleware/parameters
DIMA_MODULE_INCLUDES := -IDima/modules
DIMA_SENSOR_MODULE_INCLUDES := -IDima/modules/sensors
DIMA_LIB_INCLUDES := -IDima/lib
DIMA_PROTOCOL_INCLUDES := -IDima/lib/protocols
DIMA_LIB_SENSOR_INCLUDES := -IDima/lib/sensors
DIMA_ADAPTER_INCLUDES := -IDima/adapters
DIMA_ROVER_INCLUDES := -IDima/rover
DIMA_APPLICATION_INCLUDES := -IDima/application
DIMA_GPS_DRIVER_INCLUDES := -IDima/drivers/gps
DIMA_IMU_DRIVER_INCLUDES := -IDima/drivers/imu
DIMA_MAG_DRIVER_INCLUDES := -IDima/drivers/magnetometer
DIMA_RC_DRIVER_INCLUDES := -IDima/drivers/rc
DIMA_BOARD_HEADER_INCLUDES := -IBoards/H743/Inc
DIMA_PARAMETER_GENERATED_INCLUDES := \
	-I$(BUILD_DIR)/generated_include

DIMA_MESSAGE_GENERATED_INCLUDES := \
	-I$(BUILD_DIR)/generated/messages \
	-I$(BUILD_DIR)/generated
DIMA_UM982_GENERATED_INCLUDES := -I$(BUILD_DIR)/generated/gps
DIMA_COMPONENT_GENERATED_INCLUDES := \
	-I$(BUILD_DIR)/generated/component_metadata
# mavgen 产物是未修改的第三方头；作为 system include 只隔离上游
# inline codec 的编译器警告，项目自身代码仍保持 -Werror。
DIMA_MAVLINK_GENERATED_INCLUDES := -isystem $(MAVLINK_GENERATED_DIR)
DIMA_FIRMWARE_IDENTITY_GENERATED_INCLUDES := \
	-I$(FIRMWARE_IDENTITY_GENERATED_DIR)
DIMA_SENSOR_DEVICE_GENERATED_INCLUDES := -I$(SENSOR_DEVICE_GENERATED_DIR)

# CubeMX-owned sources only consume board declarations and the two platform
# roots used by Core interrupt/early-memory hooks. Generated application
# contracts remain target-private to project-owned objects below.
# CubeMX/HAL 翻译单元只获得板级声明；应用生成合同通过 target-private include
# 精确授予消费者，借编译边界阻止跨层“顺手 include”。
C_INCLUDES += \
	$(DIMA_BOARD_HEADER_INCLUDES) \
	$(DIMA_PLATFORM_INCLUDES) \
	$(DIMA_STM32H7_PLATFORM_INCLUDES)

DIMA_DRONECAN_CONTRACT_INCLUDES := \
	-I$(DRONECAN_GENERATED_DIR)
DIMA_DRONECAN_INCLUDES := \
	-IMiddlewares/Third_Party/libcanard \
	-I$(DRONECAN_GENERATED_DIR)/include \
	$(DIMA_DRONECAN_CONTRACT_INCLUDES)
DIMA_FATFS_INCLUDES := \
	-IMiddlewares/Third_Party/FatFs/src
DIMA_NANOPRINTF_INCLUDES := \
	-IMiddlewares/Third_Party/nanoprintf
DIMA_FREERTOS_RUNTIME_INCLUDES := \
	$(DIMA_FREERTOS_CONFIG_INCLUDES) \
	-IMiddlewares/Third_Party/FreeRTOS/Source/include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
DIMA_FREERTOS_INCLUDES := \
	$(DIMA_PLATFORM_INCLUDES) \
	$(DIMA_FREERTOS_RUNTIME_INCLUDES)
DIMA_STM32_INCLUDES := \
	$(DIMA_PLATFORM_INCLUDES) \
	$(DIMA_STM32H7_PLATFORM_INCLUDES) \
	$(DIMA_BOARD_HEADER_INCLUDES) \
	-ICore/Inc \
	-IDrivers/STM32H7xx_HAL_Driver/Inc \
	-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
	-IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
	-IDrivers/CMSIS/Include
DIMA_STM32_USB_INCLUDES := \
	-IUSB_DEVICE/App \
	-IUSB_DEVICE/Target \
	-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
	-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
DIMA_BOARD_INCLUDES := $(DIMA_STM32_INCLUDES)

PARAMETER_GENERATOR := tools/parameters/generate_parameters.py
SOURCE_MANIFEST_TOOL := tools/generation/source_manifest.py
PARAMETER_UPSTREAM_ROOT := tools/upstream/parameter_yaml_20260827
PARAMETER_SOURCE_MANIFEST := $(PARAMETER_UPSTREAM_ROOT)/SOURCE_MANIFEST.json
PARAMETER_UPSTREAM_DEPS := \
	$(wildcard $(PARAMETER_UPSTREAM_ROOT)/Tools/*.py) \
	$(wildcard $(PARAMETER_UPSTREAM_ROOT)/Tools/module_config/*.py) \
	$(wildcard $(PARAMETER_UPSTREAM_ROOT)/src/lib/parameters/*.py) \
	$(wildcard $(PARAMETER_UPSTREAM_ROOT)/src/lib/parameters/px4params/*.py) \
	$(wildcard $(PARAMETER_UPSTREAM_ROOT)/src/lib/parameters/templates/*) \
	$(PARAMETER_UPSTREAM_ROOT)/src/lib/mixer_module/output_functions.yaml \
	$(PARAMETER_UPSTREAM_ROOT)/validation/module_schema.yaml
FIRMWARE_IDENTITY_GENERATOR := \
	tools/firmware/generate_identity_contract.py
FIRMWARE_IDENTITY_MANIFEST := Boards/H743/firmware_identity.json
FIRMWARE_IDENTITY_GIT_COMMIT := $(shell git rev-parse HEAD)
FIRMWARE_IDENTITY_GENERATED_STAMP := \
	$(FIRMWARE_IDENTITY_GENERATED_DIR)/.generated-$(FIRMWARE_IDENTITY_GIT_COMMIT)
FIRMWARE_IDENTITY_HEADER := \
	$(FIRMWARE_IDENTITY_GENERATED_DIR)/FirmwareIdentityContract.hpp
FIRMWARE_IDENTITY_UPLOAD_CONTRACT := \
	$(FIRMWARE_IDENTITY_GENERATED_DIR)/firmware_identity_contract.json
FIRMWARE_IDENTITY_GENERATED_OUTPUTS := \
	$(FIRMWARE_IDENTITY_HEADER) \
	$(FIRMWARE_IDENTITY_UPLOAD_CONTRACT)
SENSOR_DEVICE_GENERATOR := tools/sensors/generate_device_contract.py
SENSOR_DEVICE_MANIFEST := Boards/H743/sensor_devices.json
SENSOR_DEVICE_GENERATED_STAMP := $(SENSOR_DEVICE_GENERATED_DIR)/.generated
SENSOR_DEVICE_CONTRACT_HEADER := \
	$(SENSOR_DEVICE_GENERATED_DIR)/SensorDeviceContract.hpp
SENSOR_DEVICE_CATALOG := $(SENSOR_DEVICE_GENERATED_DIR)/sensor_devices.json
SENSOR_DEVICE_GENERATED_OUTPUTS := \
	$(SENSOR_DEVICE_CONTRACT_HEADER) \
	$(SENSOR_DEVICE_CATALOG)
DRONECAN_CONTRACT_GENERATOR := tools/dronecan/generate_contract.py
# manifest 与第一方翻译单元由生成工具在当前 Dima 树中唯一发现；目录移动不再要求
# 同步修改 Make 和架构门禁，重名或缺失则由工具明确拒绝。
DRONECAN_CONTRACT_MANIFEST := $(strip $(shell \
	$(PYTHON) $(DRONECAN_CONTRACT_GENERATOR) --find-manifest Dima))
DIMA_DRONECAN_RUNTIME_SOURCES := $(strip $(shell \
	$(PYTHON) $(DRONECAN_CONTRACT_GENERATOR) --print-runtime-sources Dima))
ifeq ($(DRONECAN_CONTRACT_MANIFEST),)
$(error unable to discover the unique DroneCAN manifest)
endif
ifeq ($(DIMA_DRONECAN_RUNTIME_SOURCES),)
$(error unable to discover DroneCAN runtime sources)
endif
DRONECAN_GENERATED_MAKEFILE := \
	$(DRONECAN_GENERATED_DIR)/dronecan_sources.mk
DRONECAN_DSDL_INPUTS := $(shell $(PYTHON) $(DRONECAN_CONTRACT_GENERATOR) \
	--manifest $(DRONECAN_CONTRACT_MANIFEST) --print-inputs)
DRONECAN_GENERATOR_DEPS := $(DRONECAN_CONTRACT_GENERATOR) \
	$(DRONECAN_CONTRACT_MANIFEST) $(DRONECAN_DSDL_INPUTS)
ARCHITECTURE_CHECK_TOOL := tools/check_architecture.py
APPLICATION_ELF_CHECK_TOOL := tools/verify_application_elf.py
COMPILE_COMMANDS_TOOL := tools/generate_compile_commands.py
MAVLINK_BOOTSTRAP := tools/mavlink/bootstrap_pymavlink.py
MAVLINK_LOCK := tools/mavlink/mavlink.lock.json
MAVLINK_XML_DIR := tools/mavlink/message_definitions
MAVLINK_DIALECT := $(MAVLINK_XML_DIR)/dima.xml
MAVLINK_UPSTREAM_XML_INPUTS := \
	$(MAVLINK_XML_DIR)/common.xml \
	$(MAVLINK_XML_DIR)/standard.xml \
	$(MAVLINK_XML_DIR)/minimal.xml
MAVLINK_RUNTIME_POLICY := Dima/modules/mavlink/mavlink_runtime.yaml
MAVLINK_RUNTIME_GENERATOR := tools/mavlink/generate_runtime_contract.py
MAVLINK_GENERATED_WORK_DIR := $(MAVLINK_GENERATED_DIR).work
MAVLINK_VERIFY_WORK_DIR := $(MAVLINK_GENERATED_DIR).verify.work
MAVLINK_GENERATED_STAMP := $(MAVLINK_GENERATED_DIR)/.generated.json
MAVLINK_LIBRARY_HEADER := $(MAVLINK_GENERATED_DIR)/dima/mavlink.h
MAVLINK_STREAM_CONTRACT_HEADER := \
	$(MAVLINK_GENERATED_DIR)/mavlink_stream_contract.hpp
MAVLINK_GENERATED_OUTPUTS := \
	$(MAVLINK_LIBRARY_HEADER) \
	$(MAVLINK_STREAM_CONTRACT_HEADER)
MAVLINK_GENERATION_DEPS := \
	$(MAVLINK_BOOTSTRAP) $(MAVLINK_LOCK) $(MAVLINK_DIALECT) \
	$(MAVLINK_UPSTREAM_XML_INPUTS) $(MAVLINK_RUNTIME_POLICY) \
	$(MAVLINK_RUNTIME_GENERATOR)
# dima.xml 是唯一 wire 根，Make 直接调用锁定 mavgen.py；YAML 只派生
# request/interval/dispatch 合同，不参与 ID、CRC、payload 或编解码。
COMPILE_COMMANDS_OUTPUT ?= compile_commands.json
PARAMETER_GENERATOR_DEPS := \
	$(PARAMETER_GENERATOR) $(SOURCE_MANIFEST_TOOL) \
	$(PARAMETER_SOURCE_MANIFEST) $(PARAMETER_UPSTREAM_DEPS)
UM982_CONTRACT_GENERATOR := tools/gps/generate_um982_contract.py
UM982_CONTRACT_MANIFEST := Dima/drivers/gps/um982/um982_messages.json
UM982_GENERATED_DIR := $(BUILD_DIR)/generated/gps
UM982_GENERATED_STAMP := $(UM982_GENERATED_DIR)/.generated
UM982_CONTRACT_HEADER := $(UM982_GENERATED_DIR)/Um982MessageContract.hpp
MESSAGE_GENERATOR := tools/uorb/generate_messages.py
MESSAGE_SCHEMA_DIR := Dima/messages/schemas
MESSAGE_SCHEMAS := $(sort $(wildcard $(MESSAGE_SCHEMA_DIR)/*.msg))
UORB_UPSTREAM_ROOT := tools/upstream/uorb_v1_17
UORB_SOURCE_MANIFEST := $(UORB_UPSTREAM_ROOT)/SOURCE_MANIFEST.json
UORB_UPSTREAM_DEPS := \
	$(wildcard $(UORB_UPSTREAM_ROOT)/Tools/msg/*.py) \
	$(wildcard $(UORB_UPSTREAM_ROOT)/Tools/msg/templates/uorb/*)
MESSAGE_GENERATED_DIR := $(BUILD_DIR)/generated/uORB
MESSAGE_COMPAT_GENERATED_DIR := $(BUILD_DIR)/generated/messages
MESSAGE_GENERATED_MAKEFILE := $(MESSAGE_GENERATED_DIR)/uorb_sources.mk
MESSAGE_GENERATOR_DEPS := \
	$(MESSAGE_GENERATOR) $(SOURCE_MANIFEST_TOOL) \
	$(UORB_SOURCE_MANIFEST) $(UORB_UPSTREAM_DEPS)
# Topic 头、源文件、ID、hash、JSON 与注册目录全部由 PX4 v1.17 原工具派生；
# Make 只包含生成的 source fragment，不维护消息或 producer/consumer 名单。
$(DRONECAN_GENERATED_MAKEFILE): make/project.mk $(DRONECAN_GENERATOR_DEPS) | \
		$(HOST_TOOLS_STAMP)
	$(DIMA_PROGRESS_RUN) --label DRONECAN_GEN --target "$@" \
		--display "$(DRONECAN_GENERATED_DIR)" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(DRONECAN_CONTRACT_GENERATOR) \
			--manifest $(DRONECAN_CONTRACT_MANIFEST) \
			--output $(DRONECAN_GENERATED_DIR)
	@touch "$@"

include $(DRONECAN_GENERATED_MAKEFILE)

$(DIMA_DRONECAN_GENERATED_OUTPUTS): | $(DRONECAN_GENERATED_MAKEFILE)
	@test -f $@

.PHONY: dronecan-generated dronecan-generated-verify
dronecan-generated: $(DIMA_DRONECAN_GENERATED_OUTPUTS)

dronecan-generated-verify: $(DIMA_DRONECAN_GENERATED_OUTPUTS)
	$(DIMA_PROGRESS_RUN) --label DRONECAN_GEN --target "$@" \
		--display "verify $(DRONECAN_GENERATED_DIR)" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(DRONECAN_CONTRACT_GENERATOR) \
			--manifest $(DRONECAN_CONTRACT_MANIFEST) \
			--output $(DRONECAN_GENERATED_DIR) --verify

PARAMETER_DEFINITION_DIR := Dima/middleware/parameters/definitions
PARAMETER_YAML_DEFINITIONS := \
	$(sort $(wildcard $(PARAMETER_DEFINITION_DIR)/module_*.yaml)) \
	$(DIMA_DRONECAN_PARAMETER_YAML)
PARAMETER_GENERATED_DIR := $(BUILD_DIR)/generated/parameters
PARAMETER_INCLUDE_DIR := $(BUILD_DIR)/generated_include
PARAMETER_GENERATED_STAMP := $(PARAMETER_GENERATED_DIR)/.generated
PARAMETER_GENERATED_OUTPUTS := \
	$(PARAMETER_GENERATED_DIR)/module_params.c \
	$(PARAMETER_GENERATED_DIR)/parameters.xml \
	$(PARAMETER_GENERATED_DIR)/parameters.json \
	$(PARAMETER_GENERATED_DIR)/px4_parameters.hpp \
	$(PARAMETER_GENERATED_DIR)/parameter_contract.hpp \
	$(PARAMETER_INCLUDE_DIR)/px4_platform_common/param.h \
	$(PARAMETER_INCLUDE_DIR)/parameters/px4_parameters.hpp \
	$(PARAMETER_INCLUDE_DIR)/parameters/parameter_contract.hpp
PARAMETER_METADATA_GENERATOR := tools/mavlink/generate_parameter_metadata.py
PARAMETER_METADATA_DIR := $(BUILD_DIR)/generated/component_metadata
PARAMETER_METADATA_STAMP := $(PARAMETER_METADATA_DIR)/.generated.json
PARAMETER_METADATA_HEADER := $(PARAMETER_METADATA_DIR)/parameter_metadata_files.hpp
PARAMETER_METADATA_OUTPUTS := \
	$(PARAMETER_METADATA_DIR)/component_general.json \
	$(PARAMETER_METADATA_DIR)/component_general.json.xz \
	$(PARAMETER_METADATA_DIR)/parameters.json \
	$(PARAMETER_METADATA_DIR)/parameters.json.xz \
	$(PARAMETER_METADATA_DIR)/actuators.json \
	$(PARAMETER_METADATA_DIR)/actuators.json.xz \
	$(PARAMETER_METADATA_HEADER)
$(FIRMWARE_IDENTITY_GENERATED_STAMP): make/project.mk \
		$(FIRMWARE_IDENTITY_GENERATOR) $(FIRMWARE_IDENTITY_MANIFEST)
	$(DIMA_PROGRESS_RUN) --label FW_ID --target "$@" \
		--display "$(FIRMWARE_IDENTITY_GENERATED_DIR)" -- \
		$(PYTHON) $(FIRMWARE_IDENTITY_GENERATOR) \
			--manifest $(FIRMWARE_IDENTITY_MANIFEST) \
			--output $(FIRMWARE_IDENTITY_GENERATED_DIR) \
			--git-commit $(FIRMWARE_IDENTITY_GIT_COMMIT)
	@touch "$@"

$(FIRMWARE_IDENTITY_GENERATED_OUTPUTS): | \
		$(FIRMWARE_IDENTITY_GENERATED_STAMP)
	@test -f "$@"

.PHONY: firmware-identity-generated firmware-identity-generated-verify
firmware-identity-generated: $(FIRMWARE_IDENTITY_GENERATED_OUTPUTS)

firmware-identity-generated-verify: $(FIRMWARE_IDENTITY_GENERATED_OUTPUTS)
	$(DIMA_PROGRESS_RUN) --label FW_ID_VERIFY --target "$@" -- \
		$(PYTHON) $(FIRMWARE_IDENTITY_GENERATOR) \
			--manifest $(FIRMWARE_IDENTITY_MANIFEST) \
			--output $(FIRMWARE_IDENTITY_GENERATED_DIR) \
			--git-commit $(FIRMWARE_IDENTITY_GIT_COMMIT) --verify

$(SENSOR_DEVICE_GENERATED_STAMP): make/project.mk \
		$(SENSOR_DEVICE_GENERATOR) $(SENSOR_DEVICE_MANIFEST)
	$(DIMA_PROGRESS_RUN) --label SENSOR_ID --target "$@" \
		--display "$(SENSOR_DEVICE_GENERATED_DIR)" -- \
		$(PYTHON) $(SENSOR_DEVICE_GENERATOR) \
			--manifest $(SENSOR_DEVICE_MANIFEST) \
			--output $(SENSOR_DEVICE_GENERATED_DIR)
	@touch "$@"

$(SENSOR_DEVICE_GENERATED_OUTPUTS): | $(SENSOR_DEVICE_GENERATED_STAMP)
	@test -f "$@"

.PHONY: sensor-device-generated sensor-device-generated-verify
sensor-device-generated: $(SENSOR_DEVICE_GENERATED_OUTPUTS)

sensor-device-generated-verify: $(SENSOR_DEVICE_GENERATED_OUTPUTS)
	$(DIMA_PROGRESS_RUN) --label SENS_ID_VFY --target "$@" -- \
		$(PYTHON) $(SENSOR_DEVICE_GENERATOR) \
			--manifest $(SENSOR_DEVICE_MANIFEST) \
			--output $(SENSOR_DEVICE_GENERATED_DIR) --verify

$(UM982_GENERATED_STAMP): make/project.mk $(UM982_CONTRACT_GENERATOR) \
		$(UM982_CONTRACT_MANIFEST)
	$(DIMA_PROGRESS_RUN) --label GPS_CONTRACT --target "$@" \
		--display "$(UM982_GENERATED_DIR)" -- \
		$(PYTHON) $(UM982_CONTRACT_GENERATOR) \
			--manifest $(UM982_CONTRACT_MANIFEST) \
			--output $(UM982_GENERATED_DIR)
	@touch "$@"

$(UM982_CONTRACT_HEADER): | $(UM982_GENERATED_STAMP)
	@test -f $@

$(MESSAGE_GENERATED_MAKEFILE): make/project.mk $(MESSAGE_GENERATOR_DEPS) \
		$(MESSAGE_SCHEMAS) | $(HOST_TOOLS_STAMP)
	$(DIMA_PROGRESS_RUN) --label UORB_GEN --target "$@" \
		--display "$(MESSAGE_GENERATED_DIR)" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(MESSAGE_GENERATOR) \
			--schemas $(MESSAGE_SCHEMA_DIR) \
			--upstream-root $(UORB_UPSTREAM_ROOT) \
			--output $(MESSAGE_GENERATED_DIR) \
			--compat-output $(MESSAGE_COMPAT_GENERATED_DIR)

include $(MESSAGE_GENERATED_MAKEFILE)

MESSAGE_GENERATED_STAMP := $(DIMA_UORB_GENERATED_STAMP)
MESSAGE_GENERATED_OUTPUTS := $(DIMA_UORB_GENERATED_OUTPUTS)

$(MESSAGE_GENERATED_OUTPUTS): | $(MESSAGE_GENERATED_MAKEFILE)
	@test -f $@

.PHONY: uorb-generated uorb-generated-verify
uorb-generated: $(MESSAGE_GENERATED_OUTPUTS)
	$(DIMA_PROGRESS_RUN) --label UORB_SOURCE --target "$@" \
		--display "$(UORB_SOURCE_MANIFEST)" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			$(PYTHON) $(SOURCE_MANIFEST_TOOL) \
			--root $(UORB_UPSTREAM_ROOT) \
			--output $(UORB_SOURCE_MANIFEST) \
			--project PX4-Autopilot \
			--commit d6f12ad1c4f70ad3230afd7d86e971421e02fef4 \
			--verify

uorb-generated-verify: $(MESSAGE_GENERATED_OUTPUTS)
	$(DIMA_PROGRESS_RUN) --label UORB_VERIFY --target "$@" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(MESSAGE_GENERATOR) \
			--schemas $(MESSAGE_SCHEMA_DIR) \
			--upstream-root $(UORB_UPSTREAM_ROOT) \
			--output $(MESSAGE_GENERATED_DIR) \
			--compat-output $(MESSAGE_COMPAT_GENERATED_DIR) --verify

$(PARAMETER_GENERATED_STAMP): make/project.mk $(PARAMETER_GENERATOR_DEPS) \
		$(PARAMETER_YAML_DEFINITIONS) | $(HOST_TOOLS_STAMP)
	$(DIMA_PROGRESS_RUN) --label PARAM --target "$@" \
		--display "$(PARAMETER_GENERATED_DIR)" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(PARAMETER_GENERATOR) \
		$(foreach definition,$(PARAMETER_YAML_DEFINITIONS),--yaml "$(definition)") \
		--upstream-root $(PARAMETER_UPSTREAM_ROOT) \
		--output $(PARAMETER_GENERATED_DIR) \
		--include-output $(PARAMETER_INCLUDE_DIR)
	@touch "$@"
# 源码 YAML 与 DroneCAN YAML 只进入 PX4 官方链；Dima 参数合同随后仅从
# 官方 XML/JSON/Header 派生，因此类型、默认值、索引和 volatile 语义没有第二解释器。

$(PARAMETER_GENERATED_OUTPUTS): | $(PARAMETER_GENERATED_STAMP)
	@test -f $@

.PHONY: parameter-generated
parameter-generated: $(PARAMETER_GENERATED_OUTPUTS)
	$(DIMA_PROGRESS_RUN) --label PARAM_SOURCE --target "$@" \
		--display "$(PARAMETER_SOURCE_MANIFEST)" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			$(PYTHON) $(SOURCE_MANIFEST_TOOL) \
			--root $(PARAMETER_UPSTREAM_ROOT) \
			--output $(PARAMETER_SOURCE_MANIFEST) \
			--project PX4-Autopilot \
			--commit 1f6b6f61f8f42eaab0269c16a442cb580f954d7c \
			--verify

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

.PHONY: parameter-metadata parameter-metadata-verify
parameter-metadata: $(PARAMETER_METADATA_OUTPUTS)

parameter-metadata-verify: $(PARAMETER_METADATA_OUTPUTS) \
		$(PARAMETER_METADATA_STAMP)
	$(DIMA_PROGRESS_RUN) --label META_VERIFY --target "$@" -- \
		$(PYTHON) $(PARAMETER_METADATA_GENERATOR) \
			--parameters $(PARAMETER_GENERATED_DIR)/parameters.json \
			--output $(PARAMETER_METADATA_DIR) --verify

# host 工具不固定 lxml；显式关闭 XSD，改由 XML 解析、来源 hash、include
# 闭包和生成符号门禁提供跨主机一致的失败边界。
$(MAVLINK_GENERATED_STAMP): make/project.mk $(MAVLINK_GENERATION_DEPS) | \
		$(HOST_TOOLS_STAMP)
	@set -eu; \
		rm -rf "$(MAVLINK_GENERATED_WORK_DIR)"; \
		pymavlink_root="$$(env PYTHONDONTWRITEBYTECODE=1 \
			$(PYTHON) $(MAVLINK_BOOTSTRAP) --lock $(MAVLINK_LOCK) \
			--cache-root "$(HOST_TOOLS_CACHE_ROOT)" \
			$(if $(strip $(PYMAVLINK_ROOT)),--source-root "$(PYMAVLINK_ROOT)",))"; \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONHASHSEED=0 \
			$(PYTHON) "$$pymavlink_root/tools/mavgen.py" \
			--lang C --wire-protocol 2.0 --no-validate \
			--output "$(MAVLINK_GENERATED_WORK_DIR)" \
			$(MAVLINK_DIALECT); \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(MAVLINK_RUNTIME_GENERATOR) \
			--generated-dir "$(MAVLINK_GENERATED_WORK_DIR)" \
			--output-dir "$(MAVLINK_GENERATED_DIR)" \
			--dialect $(MAVLINK_DIALECT) \
			$(foreach xml,$(MAVLINK_UPSTREAM_XML_INPUTS),--upstream-xml "$(xml)") \
			--policy $(MAVLINK_RUNTIME_POLICY) --lock $(MAVLINK_LOCK)
# .generated.json 由薄策略生成器在 mavgen 完成后写入并原子安装；
# 任一阶段失败都不会用半棵新头文件覆盖已安装树。

$(MAVLINK_GENERATED_OUTPUTS): | $(MAVLINK_GENERATED_STAMP)
	@test -f "$@"

.PHONY: mavlink mavlink-generated mavlink-generated-verify
mavlink: mavlink-generated
mavlink-generated: $(MAVLINK_GENERATED_OUTPUTS)

mavlink-generated-verify: $(MAVLINK_GENERATED_OUTPUTS) | $(HOST_TOOLS_STAMP)
	@set -eu; \
		rm -rf "$(MAVLINK_VERIFY_WORK_DIR)"; \
		pymavlink_root="$$(env PYTHONDONTWRITEBYTECODE=1 \
			$(PYTHON) $(MAVLINK_BOOTSTRAP) --lock $(MAVLINK_LOCK) \
			--cache-root "$(HOST_TOOLS_CACHE_ROOT)" \
			$(if $(strip $(PYMAVLINK_ROOT)),--source-root "$(PYMAVLINK_ROOT)",))"; \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONHASHSEED=0 \
			$(PYTHON) "$$pymavlink_root/tools/mavgen.py" \
			--lang C --wire-protocol 2.0 --no-validate \
			--output "$(MAVLINK_VERIFY_WORK_DIR)" \
			$(MAVLINK_DIALECT); \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(MAVLINK_RUNTIME_GENERATOR) \
			--generated-dir "$(MAVLINK_VERIFY_WORK_DIR)" \
			--output-dir "$(MAVLINK_GENERATED_DIR)" \
			--dialect $(MAVLINK_DIALECT) \
			$(foreach xml,$(MAVLINK_UPSTREAM_XML_INPUTS),--upstream-xml "$(xml)") \
			--policy $(MAVLINK_RUNTIME_POLICY) --lock $(MAVLINK_LOCK) --verify

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
DIMA_DRONECAN_C_SOURCES := \
	Middlewares/Third_Party/libcanard/canard.c \
	$(DIMA_DRONECAN_GENERATED_C_SOURCES)
DIMA_FATFS_FREERTOS_C_SOURCES := \
	Middlewares/Third_Party/FatFs/src/ff.c
DIMA_FATFS_BOARD_C_SOURCES := \
	Boards/H743/Src/fatfs_diskio.c
DIMA_FREERTOS_C_SOURCES := \
	Dima/platform/freertos/libc/cpp_runtime.c \
	Dima/platform/freertos/libc/no_heap.c \
	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_5.c \
	$(DIMA_FATFS_FREERTOS_C_SOURCES)
DIMA_STM32_C_SOURCES := \
	Dima/platform/stm32h7/memory/cache.c \
	Dima/platform/stm32h7/memory/early_memory.c \
	Dima/platform/stm32h7/flash/flash_bank1.c
DIMA_BOARD_C_SOURCES := \
	Boards/H743/Src/board_init.c \
	Boards/H743/Src/boot_diagnostics.c \
	Boards/H743/Src/motor_pwm.c \
	$(DIMA_FATFS_BOARD_C_SOURCES)
PROJECT_C_SOURCES := \
	$(DIMA_DRONECAN_C_SOURCES) \
	$(DIMA_FREERTOS_C_SOURCES) \
	$(DIMA_STM32_C_SOURCES) \
	$(DIMA_BOARD_C_SOURCES)

DIMA_COMMON_CXX_SOURCES := \
	Dima/platform/common/Execution.cpp \
	Dima/platform/common/Flash.cpp \
	Dima/platform/common/Memory.cpp \
	Dima/platform/common/Services.cpp \
	Dima/platform/common/Synchronization.cpp \
	Dima/adapters/usb_console/UsbConsole.cpp \
	Dima/application/app_bootstrap.cpp \
	Dima/rover/ApplicationContext.cpp \
	Dima/rover/control/RoverDifferential.cpp \
	Dima/rover/modes/ManualMode.cpp \
	Dima/lib/timesync/Timesync.cpp \
	Dima/lib/sensors/SensorRotation.cpp \
	Dima/lib/sensors/validation/DataValidator.cpp \
	Dima/lib/sensors/validation/SensorValidityAlgorithms.cpp \
	Dima/drivers/gps/um982/Um982Protocol.cpp \
	Dima/drivers/gps/um982/Um982Gps.cpp \
	Dima/drivers/gps/um982/Um982GpsBaud.cpp \
	Dima/drivers/gps/um982/Um982GpsLogging.cpp \
	Dima/drivers/gps/um982/Um982GpsValidation.cpp \
	Dima/drivers/imu/icm42688p/ICM42688P.cpp \
	Dima/drivers/imu/icm42688p/ICM42688PFifo.cpp \
	Dima/modules/sensors/imu/VehicleImu.cpp \
	Dima/lib/sensors/calibration/SensorCalibrationAlgorithms.cpp \
	Dima/modules/sensors/magnetometer/VehicleMagnetometer.cpp \
	Dima/modules/sensors/calibration/SensorCalibration.cpp \
	Dima/modules/sensors/calibration/SensorCalibrationGyro.cpp \
	Dima/modules/sensors/calibration/SensorCalibrationAccel.cpp \
	Dima/modules/sensors/calibration/SensorCalibrationMag.cpp \
	Dima/modules/sensors/calibration/SensorCalibrationParameters.cpp \
	Dima/modules/logging/LogService.cpp \
	Dima/modules/mavlink/HeartbeatPacer.cpp \
	Dima/adapters/mavlink/MavlinkChannelState.cpp \
	Dima/modules/mavlink/MavlinkCommands.cpp \
	Dima/modules/mavlink/MavlinkIdentity.cpp \
	Dima/modules/mavlink/MavlinkMetadataFtp.cpp \
	Dima/modules/mavlink/MavlinkMission.cpp \
	Dima/modules/mavlink/MavlinkParameterExt.cpp \
	Dima/modules/mavlink/MavlinkParameters.cpp \
	Dima/modules/mavlink/MavlinkRcStream.cpp \
	Dima/modules/mavlink/MavlinkSensorStreams.cpp \
	Dima/modules/mavlink/MavlinkService.cpp \
	Dima/modules/mavlink/MavlinkSystemMessages.cpp \
	Dima/modules/mavlink/MavlinkTimesync.cpp \
	Dima/modules/motor/MotorOutput.cpp \
	Dima/modules/motor/MotorOutputFrames.cpp \
	Dima/modules/motor/MotorOutputParameters.cpp \
	Dima/modules/motor/MotorOutputSafety.cpp \
	Dima/modules/parameters/ParameterService.cpp \
	Dima/modules/parameters/ParameterServicePersistence.cpp \
	Dima/modules/parameters/ParameterServiceSdMirror.cpp \
	Dima/modules/parameters/ParameterSnapshotCodec.cpp \
	Dima/modules/serial/SerialConfig.cpp \
	Dima/application/app_main.cpp \
	Dima/lib/rover/DifferentialDrive.cpp \
	Dima/drivers/rc/sbus/SbusProtocol.cpp \
	Dima/drivers/rc/sbus/SbusRc.cpp \
	Dima/modules/rc/RCUpdate.cpp \
	Dima/modules/safety/Commander.cpp \
	Dima/modules/safety/CommanderActions.cpp \
	Dima/modules/safety/CommanderCommands.cpp \
	Dima/modules/safety/CommanderSafety.cpp \
	Dima/modules/boot_health/BootHealthService.cpp \
	Dima/middleware/maintenance/RuntimeMaintenanceCoordinator.cpp \
	Dima/middleware/lifecycle/module_manager.cpp \
	Dima/middleware/work_queue/WorkQueue.cpp \
	Dima/middleware/uORB/uORB.cpp \
	Dima/middleware/parameters/param.cpp \
	Dima/middleware/parameters/param_storage.cpp \
	Dima/middleware/parameters/autosave.cpp \
	Dima/lib/format/Format.cpp \
	Dima/lib/tinybson/tinybson.cpp \
	Dima/middleware/parameters/flashparams/flashparams.cpp \
	Dima/middleware/parameters/FileStorage.cpp \
	Dima/middleware/parameters/flashfs.cpp \
	Dima/modules/rc/RcManualInput.cpp \
	$(DIMA_UORB_GENERATED_SOURCES) \
	Dima/middleware/events/events.cpp \
	Dima/middleware/perf/perf_counter.cpp \
	Dima/middleware/logging/logging.cpp
DIMA_DRONECAN_COMMON_CXX_SOURCES := \
	$(DIMA_DRONECAN_RUNTIME_SOURCES)
DIMA_FATFS_FREERTOS_CXX_SOURCES := \
	Dima/platform/freertos/storage/FatFsParameterFileStore.cpp
DIMA_FREERTOS_CXX_SOURCES := \
	Dima/platform/freertos/Backend.cpp \
	Dima/platform/freertos/HeapOperators.cpp \
	$(DIMA_FATFS_FREERTOS_CXX_SOURCES)
DIMA_STM32_CXX_SOURCES := \
	Dima/platform/stm32h7/pwm/ActuatorPwm.cpp \
	Dima/platform/stm32h7/system/BootControl.cpp \
	Dima/platform/stm32h7/system/Clock.cpp \
	Dima/platform/stm32h7/memory/DmaMemory.cpp \
	Dima/platform/stm32h7/flash/FlashDevice.cpp \
	Dima/platform/stm32h7/system/HardwareUid.cpp \
	Dima/platform/stm32h7/system/Watchdog.cpp \
	Dima/platform/stm32h7/serial/SerialPorts.cpp \
	Dima/platform/stm32h7/serial/UartResources.cpp \
	Dima/platform/stm32h7/serial/UartIrqRouter.cpp \
	Dima/platform/stm32h7/serial/UartTimestampedRxEndpoint.cpp \
	Dima/platform/stm32h7/serial/UartDuplexDmaEndpoint.cpp \
	Dima/platform/stm32h7/can/Fdcan1.cpp \
	Dima/platform/stm32h7/spi/Spi4.cpp \
	Dima/platform/stm32h7/interrupts/SensorInterrupts.cpp \
	Dima/platform/stm32h7/usb/UsbCdcTransport.cpp
DIMA_BOARD_CXX_SOURCES := \
	Boards/H743/Src/platform_composition.cpp
PROJECT_CXX_SOURCES := \
	$(DIMA_COMMON_CXX_SOURCES) \
	$(DIMA_DRONECAN_COMMON_CXX_SOURCES) \
	$(DIMA_FREERTOS_CXX_SOURCES) \
	$(DIMA_STM32_CXX_SOURCES) \
	$(DIMA_BOARD_CXX_SOURCES)

PROJECT_C_OBJECTS := $(addprefix $(BUILD_DIR)/,$(PROJECT_C_SOURCES:.c=.o))
PROJECT_CXX_OBJECTS := $(addprefix $(BUILD_DIR)/,$(PROJECT_CXX_SOURCES:.cpp=.o))
DIMA_DRONECAN_C_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_DRONECAN_C_SOURCES:.c=.o))
DIMA_FREERTOS_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_FREERTOS_C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_FREERTOS_CXX_SOURCES:.cpp=.o))
DIMA_STM32_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_STM32_C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_STM32_CXX_SOURCES:.cpp=.o))
DIMA_BOARD_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_BOARD_C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_BOARD_CXX_SOURCES:.cpp=.o))
DIMA_FATFS_FREERTOS_CXX_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_FATFS_FREERTOS_CXX_SOURCES:.cpp=.o))
DIMA_FORMAT_CXX_OBJECT := \
	$(BUILD_DIR)/Dima/lib/format/Format.o
DIMA_FATFS_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_FATFS_FREERTOS_C_SOURCES:.c=.o)) \
	$(DIMA_FATFS_FREERTOS_CXX_OBJECTS) \
	$(addprefix $(BUILD_DIR)/,$(DIMA_FATFS_BOARD_C_SOURCES:.c=.o))
PROJECT_OBJECTS := $(PROJECT_C_OBJECTS) $(PROJECT_CXX_OBJECTS)
DIMA_DRONECAN_COMMON_CXX_OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_DRONECAN_COMMON_CXX_SOURCES:.cpp=.o))
DIMA_PLATFORM_COMMON_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/platform/common/%,$(PROJECT_OBJECTS))
DIMA_ADAPTER_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/adapters/%,$(PROJECT_OBJECTS))
DIMA_USB_ADAPTER_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/adapters/usb_console/%,$(PROJECT_OBJECTS))
DIMA_MAVLINK_ADAPTER_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/adapters/mavlink/%,$(PROJECT_OBJECTS))
DIMA_APPLICATION_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/application/%,$(PROJECT_OBJECTS))
DIMA_APP_MAIN_OBJECT := $(BUILD_DIR)/Dima/application/app_main.o
DIMA_APPLICATION_CONTEXT_OBJECT := \
	$(BUILD_DIR)/Dima/rover/ApplicationContext.o
DIMA_COMPOSITION_OBJECTS := \
	$(DIMA_APP_MAIN_OBJECT) $(DIMA_APPLICATION_CONTEXT_OBJECT)
DIMA_ROVER_OBJECTS := \
	$(filter-out $(DIMA_COMPOSITION_OBJECTS), \
		$(filter $(BUILD_DIR)/Dima/rover/%,$(PROJECT_OBJECTS)))
DIMA_LIB_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/lib/%,$(PROJECT_OBJECTS))
DIMA_DRONECAN_LIB_OBJECTS := \
	$(filter $(DIMA_LIB_OBJECTS),$(DIMA_DRONECAN_COMMON_CXX_OBJECTS))
DIMA_TIMESYNC_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/lib/timesync/%,$(PROJECT_OBJECTS))
DIMA_DRIVER_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/drivers/%,$(PROJECT_OBJECTS))
DIMA_GPS_DRIVER_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/drivers/gps/%,$(PROJECT_OBJECTS))
DIMA_MAG_DRIVER_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/drivers/magnetometer/%,$(PROJECT_OBJECTS))
DIMA_RC_DRIVER_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/drivers/rc/%,$(PROJECT_OBJECTS))
DIMA_MODULE_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/modules/%,$(PROJECT_OBJECTS))
DIMA_SENSOR_MODULE_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/modules/sensors/%,$(PROJECT_OBJECTS))
DIMA_PARAMETER_MODULE_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/modules/parameters/%,$(PROJECT_OBJECTS))
DIMA_MAVLINK_MODULE_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/modules/mavlink/%,$(PROJECT_OBJECTS))
DIMA_MAVLINK_COMPONENT_METADATA_OBJECTS := \
	$(BUILD_DIR)/Dima/modules/mavlink/MavlinkService.o \
	$(BUILD_DIR)/Dima/modules/mavlink/MavlinkSystemMessages.o
DIMA_COMMANDER_CALIBRATION_OBJECT := \
	$(BUILD_DIR)/Dima/modules/safety/CommanderCommands.o
DIMA_SERIAL_CONFIG_OBJECT := \
	$(BUILD_DIR)/Dima/modules/serial/SerialConfig.o
DIMA_MIDDLEWARE_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/middleware/%,$(PROJECT_OBJECTS))
DIMA_UORB_RUNTIME_OBJECT := \
	$(BUILD_DIR)/Dima/middleware/uORB/uORB.o
DIMA_PARAMETER_MIDDLEWARE_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/middleware/parameters/%,$(PROJECT_OBJECTS))
DIMA_LOG_EVENT_MIDDLEWARE_OBJECTS := \
	$(filter $(BUILD_DIR)/Dima/middleware/logging/% \
		$(BUILD_DIR)/Dima/middleware/events/%,$(PROJECT_OBJECTS))
DIMA_MESSAGE_GENERATED_OBJECT := \
	$(addprefix $(BUILD_DIR)/,$(DIMA_UORB_GENERATED_SOURCES:.cpp=.o))
DIMA_STM32_USB_TRANSPORT_OBJECT := \
	$(BUILD_DIR)/Dima/platform/stm32h7/usb/UsbCdcTransport.o
DIMA_BOARD_BOOT_DIAGNOSTICS_OBJECT := \
	$(BUILD_DIR)/Boards/H743/Src/boot_diagnostics.o
DIMA_BOARD_COMPOSITION_OBJECT := \
	$(BUILD_DIR)/Boards/H743/Src/platform_composition.o
DIMA_CUBEMX_APPLICATION_BRIDGE_OBJECT := $(BUILD_DIR)/main.o
DIMA_CUBEMX_USB_CONSOLE_BRIDGE_OBJECT := $(BUILD_DIR)/usbd_cdc_if.o
$(PROJECT_OBJECTS): | $(PARAMETER_GENERATED_STAMP) $(PARAMETER_METADATA_STAMP) \
	$(MESSAGE_GENERATED_STAMP) $(MAVLINK_GENERATED_STAMP) \
	$(DIMA_DRONECAN_GENERATED_STAMP) \
	$(FIRMWARE_IDENTITY_GENERATED_STAMP) $(SENSOR_DEVICE_GENERATED_STAMP) \
	$(UM982_GENERATED_STAMP)
# 项目对象只等待实际编译所需的生成合同；这些 order-only 依赖避免 stamp 的
# mtime 更新触发无意义重编译，生成输出的真实内容依赖仍由各自规则维护。
override OBJECTS += $(PROJECT_OBJECTS)

# Object files share one Application build directory.  Keep a single profile
# stamp so switching release <-> debug invalidates every CubeMX and project
# object instead of allowing a mixed-profile ELF.
# release/debug 共用对象目录，profile stamp 切换时删除另一模式标记，禁止把
# -Os 与 -Og 对象混链进同一个 ELF。
DIMA_BUILD_PROFILE_STAMP := $(BUILD_DIR)/.application-profile-$(DIMA_BUILD_PROFILE)
DIMA_OTHER_BUILD_PROFILE := $(if $(filter release,$(DIMA_BUILD_PROFILE)),debug,release)
DIMA_OTHER_BUILD_PROFILE_STAMP := $(BUILD_DIR)/.application-profile-$(DIMA_OTHER_BUILD_PROFILE)
$(DIMA_BUILD_PROFILE_STAMP): | $(BUILD_DIR)
	@rm -f "$(DIMA_OTHER_BUILD_PROFILE_STAMP)"
	@touch "$@"

$(OBJECTS): $(DIMA_BUILD_PROFILE_STAMP)

$(PROJECT_OBJECTS): DIMA_PRIVATE_DEFS := $(DIMA_PRODUCT_DEFS)
# 下列 target-private include 是架构隔离的一部分：每组对象只看到职责所需的
# 平台、协议和生成合同，不把全仓库 include 根暴露为隐式依赖。
$(PROJECT_OBJECTS): DIMA_PRIVATE_INCLUDES := \
	$(DIMA_FIRMWARE_IDENTITY_GENERATED_INCLUDES) \
	$(DIMA_SENSOR_DEVICE_GENERATED_INCLUDES)

$(DIMA_PLATFORM_COMMON_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_PLATFORM_INCLUDES)
$(DIMA_ADAPTER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_ADAPTER_INCLUDES)
$(DIMA_USB_ADAPTER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_PLATFORM_INCLUDES)
$(DIMA_MAVLINK_ADAPTER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_MAVLINK_GENERATED_INCLUDES)

$(DIMA_APPLICATION_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_PLATFORM_INCLUDES)
$(DIMA_ROVER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_ROVER_INCLUDES) $(DIMA_LIB_INCLUDES) \
	$(DIMA_MIDDLEWARE_INCLUDES) $(DIMA_PLATFORM_INCLUDES) \
	$(DIMA_MESSAGE_GENERATED_INCLUDES) \
	$(DIMA_PARAMETER_GENERATED_INCLUDES)
$(DIMA_COMPOSITION_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_ROVER_INCLUDES) $(DIMA_LIB_INCLUDES) \
	$(DIMA_MIDDLEWARE_INCLUDES) \
	$(DIMA_DRONECAN_CONTRACT_INCLUDES) \
	$(DIMA_MESSAGE_GENERATED_INCLUDES) \
	$(DIMA_PARAMETER_GENERATED_INCLUDES) \
	$(DIMA_GPS_DRIVER_INCLUDES) $(DIMA_IMU_DRIVER_INCLUDES) \
	$(DIMA_MAG_DRIVER_INCLUDES) $(DIMA_RC_DRIVER_INCLUDES) \
	$(DIMA_MODULE_INCLUDES) $(DIMA_SENSOR_MODULE_INCLUDES) \
	$(DIMA_PROTOCOL_INCLUDES) $(DIMA_LIB_SENSOR_INCLUDES) \
	$(DIMA_ADAPTER_INCLUDES) \
	$(DIMA_MAVLINK_GENERATED_INCLUDES)
$(DIMA_APPLICATION_CONTEXT_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_PLATFORM_INCLUDES)

$(DIMA_LIB_OBJECTS): DIMA_PRIVATE_INCLUDES += $(DIMA_LIB_INCLUDES)
$(DIMA_TIMESYNC_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_PLATFORM_INCLUDES)
$(DIMA_DRONECAN_LIB_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_PLATFORM_INCLUDES) $(DIMA_DRONECAN_INCLUDES)

$(DIMA_DRIVER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_MIDDLEWARE_INCLUDES) $(DIMA_PLATFORM_INCLUDES) \
	$(DIMA_MESSAGE_GENERATED_INCLUDES) \
	$(DIMA_PARAMETER_GENERATED_INCLUDES)
# GPS 对象集合由 PROJECT_OBJECTS 的路径闭包推导，串口配置对象复用上方单一变量；
# 直接等待生成头可防止并行编译在合同文件可见前启动，也不引入手写消费者清单。
$(DIMA_GPS_DRIVER_OBJECTS) $(DIMA_SERIAL_CONFIG_OBJECT): | \
	$(UM982_CONTRACT_HEADER)
$(DIMA_GPS_DRIVER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_LIB_INCLUDES) $(DIMA_LIB_SENSOR_INCLUDES) \
	$(DIMA_PROTOCOL_INCLUDES) $(DIMA_UM982_GENERATED_INCLUDES)
$(DIMA_MAG_DRIVER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_LIB_INCLUDES) $(DIMA_LIB_SENSOR_INCLUDES) \
	$(DIMA_PROTOCOL_INCLUDES) $(DIMA_DRONECAN_INCLUDES)
$(DIMA_RC_DRIVER_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_LIB_INCLUDES) $(DIMA_PROTOCOL_INCLUDES)

$(DIMA_MODULE_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_MODULE_INCLUDES) $(DIMA_LIB_INCLUDES) \
	$(DIMA_MIDDLEWARE_INCLUDES) $(DIMA_PLATFORM_INCLUDES) \
	$(DIMA_MESSAGE_GENERATED_INCLUDES) \
	$(DIMA_PARAMETER_GENERATED_INCLUDES)
$(DIMA_SENSOR_MODULE_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_SENSOR_MODULE_INCLUDES) $(DIMA_LIB_SENSOR_INCLUDES)
$(DIMA_COMMANDER_CALIBRATION_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_LIB_SENSOR_INCLUDES)
$(DIMA_PARAMETER_MODULE_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_PARAMETER_MIDDLEWARE_INCLUDES)
$(DIMA_MAVLINK_MODULE_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_ADAPTER_INCLUDES) $(DIMA_MAVLINK_GENERATED_INCLUDES)
$(DIMA_MAVLINK_COMPONENT_METADATA_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_COMPONENT_GENERATED_INCLUDES)
$(DIMA_SERIAL_CONFIG_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_UM982_GENERATED_INCLUDES)

$(DIMA_MIDDLEWARE_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_MIDDLEWARE_INCLUDES) $(DIMA_PLATFORM_INCLUDES)
# uORB Runtime 需要读取官方 uORBTopics.hpp 聚合目录；只对该对象
# 开放生成 include 根，不把消息合同扩散到其他中间件。
$(DIMA_UORB_RUNTIME_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_MESSAGE_GENERATED_INCLUDES)
$(DIMA_PARAMETER_MIDDLEWARE_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_LIB_INCLUDES) $(DIMA_MESSAGE_GENERATED_INCLUDES) \
	$(DIMA_PARAMETER_GENERATED_INCLUDES)
$(DIMA_LOG_EVENT_MIDDLEWARE_OBJECTS): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_LIB_INCLUDES) $(DIMA_MESSAGE_GENERATED_INCLUDES)
$(DIMA_MESSAGE_GENERATED_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_MESSAGE_GENERATED_INCLUDES) $(DIMA_MIDDLEWARE_INCLUDES) \
	$(DIMA_PLATFORM_INCLUDES) $(DIMA_PARAMETER_GENERATED_INCLUDES)

$(DIMA_DRONECAN_C_OBJECTS): DIMA_PRIVATE_DEFS := $(DIMA_PRODUCT_DEFS)
$(DIMA_DRONECAN_C_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_DRONECAN_INCLUDES)
$(DIMA_FORMAT_CXX_OBJECT): DIMA_PRIVATE_INCLUDES += $(DIMA_NANOPRINTF_INCLUDES)
$(DIMA_FREERTOS_OBJECTS): DIMA_PRIVATE_DEFS :=
$(DIMA_FREERTOS_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_FREERTOS_INCLUDES)
$(DIMA_STM32_OBJECTS): DIMA_PRIVATE_DEFS := $(C_DEFS)
$(DIMA_STM32_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_STM32_INCLUDES)
$(DIMA_STM32_USB_TRANSPORT_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_STM32_USB_INCLUDES)
$(DIMA_BOARD_OBJECTS): DIMA_PRIVATE_DEFS := $(C_DEFS)
$(DIMA_BOARD_OBJECTS): DIMA_PRIVATE_INCLUDES := $(DIMA_BOARD_INCLUDES)
$(DIMA_BOARD_BOOT_DIAGNOSTICS_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_FREERTOS_RUNTIME_INCLUDES)
$(DIMA_BOARD_COMPOSITION_OBJECT): DIMA_PRIVATE_INCLUDES += \
	$(DIMA_ADAPTER_INCLUDES)
$(DIMA_FATFS_OBJECTS): DIMA_PRIVATE_INCLUDES += $(DIMA_FATFS_INCLUDES)

# Core main is the sole CubeMX-owned C-to-application ABI bridge. Keep the
# application include root private to that object instead of exposing it to
# every CubeMX and vendor translation unit through C_INCLUDES.
# main.o 是 CubeMX C 世界到应用 C ABI 的唯一入口；USB 回调同理只给
# usbd_cdc_if.o 暴露 adapter，避免厂商代码反向依赖应用层。
$(DIMA_CUBEMX_APPLICATION_BRIDGE_OBJECT): C_INCLUDES += \
	$(DIMA_APPLICATION_INCLUDES)
$(DIMA_CUBEMX_USB_CONSOLE_BRIDGE_OBJECT): C_INCLUDES += \
	$(DIMA_ADAPTER_INCLUDES)

ifneq ($(strip $(CUBEMX_OBJECTS)),)
$(CUBEMX_OBJECTS): GNUmakefile make/project.mk
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
# 仍由 GCC C driver 最终链接，并通过显式对象闭包避免隐式拉入 C++ runtime；
# 本层只追加前置对象，不复制或分叉 CubeMX 的链接命令。
$(BUILD_DIR)/$(TARGET).elf: $(PROJECT_OBJECTS) $(LDSCRIPT) GNUmakefile make/project.mk

include make/release.mk


-include $(PROJECT_OBJECTS:.o=.d)
