# MCUboot image metadata. Override release values at build time, for example:
#   make IMAGE_VERSION=1.2.3+45 KEY_FILE=/secure/path/production.pem
# IMAGE_VERSION 来自权威 firmware_identity.json；KEY_FILE 的内容身份另存 stamp，
# 因而同一路径换钥匙也会使签名、MCUboot 与验证链失效重建。
IMAGE_VERSION ?= $(shell $(PYTHON) $(FIRMWARE_IDENTITY_GENERATOR) \
	--manifest $(FIRMWARE_IDENTITY_MANIFEST) --print-image-version)
ifeq ($(strip $(IMAGE_VERSION)),)
$(error IMAGE_VERSION could not be generated from $(FIRMWARE_IDENTITY_MANIFEST))
endif
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
# 解析期只读 status 解决 dry-run 的“伪更新”问题：只有密钥内容身份变化才注入
# FORCE 前置条件，计划图与真实执行图保持一致。
KEY_IDENTITY_STATUS := $(shell $(PYTHON) "$(KEY_ID_TOOL)" \
	--key "$(KEY_FILE)" --stamp "$(KEY_ID_STAMP)" --status)
KEY_IDENTITY_WILL_CHANGE := $(if $(filter current,$(KEY_IDENTITY_STATUS)),0,1)
KEY_ID_FORCE_PREREQUISITE := $(if $(filter 1,$(KEY_IDENTITY_WILL_CHANGE)),FORCE_KEY_IDENTITY_CHECK,)
ARCHITECTURE_CACHE_TOOL = tools/architecture_cache.py
ARCHITECTURE_VERIFY_STAMP = $(BUILD_DIR)/.architecture-verified
# Full application/release goals keep their historical live gate.  Upload is
# deliberately excluded when present so `make dima_rover upload` remains the
# application-only OTA spelling; a changed architecture input still invalidates
# the content identity and runs the complete checker before any rebuild.
# architecture stamp 是对受控源码/manifest/生成器输入的内容寻址证明；上传可复用
# 当前 stamp，但任一输入哈希漂移都会先完整重跑架构门禁。
DIMA_ARCHITECTURE_FULL_GOALS := check-architecture app-check firmware verify dima_rover
DIMA_ARCHITECTURE_FORCE_GOALS := $(if $(filter upload,$(MAKECMDGOALS)),\
	$(filter check-architecture,$(MAKECMDGOALS)),\
	$(filter $(DIMA_ARCHITECTURE_FULL_GOALS),$(MAKECMDGOALS)))
DIMA_ARCHITECTURE_CACHE_GOALS := architecture-ready upload upload-ready
# Avoid hashing the architecture surface in generated-output preparation,
# summaries, preflight, and full goals that will force the checker anyway.
ARCHITECTURE_IDENTITY_STATUS := $(if $(DIMA_ARCHITECTURE_FORCE_GOALS),stale,\
	$(if $(filter $(DIMA_ARCHITECTURE_CACHE_GOALS),$(MAKECMDGOALS)),\
		$(shell $(PYTHON) "$(ARCHITECTURE_CACHE_TOOL)" \
			--stamp "$(ARCHITECTURE_VERIFY_STAMP)" --status),stale))
ARCHITECTURE_IDENTITY_WILL_CHANGE := $(if \
	$(and $(filter current,$(ARCHITECTURE_IDENTITY_STATUS)),\
	      $(if $(DIMA_ARCHITECTURE_FORCE_GOALS),,1)),0,1)
ARCHITECTURE_FORCE_PREREQUISITE := $(if \
	$(filter 1,$(ARCHITECTURE_IDENTITY_WILL_CHANGE)),\
	FORCE_ARCHITECTURE_CHECK,)
MCUMGR ?= mcumgr
MCUMGR_PORT ?=
MCUMGR_BAUD ?= 921600
MCUMGR_MTU ?= 512
MCUMGR_MAX_WINDOW ?= 1
UPLOAD_WAIT_SECONDS ?= 60
UPLOAD_FORCE ?= 0
UPLOAD_IMAGE ?= $(SIGNED_BIN)
UPLOAD_VERIFY_STAMP = $(BUILD_DIR)/.upload-image-verified
UPLOAD_TOOL = tools/mcumgr_upload.py
MCUMGR_BOOTSTRAP_TOOL = tools/bootstrap_mcumgr.py
UPLOAD_FORCE_FLAG = $(if $(filter 1,$(UPLOAD_FORCE)),--force,)

ifneq ($(filter-out 0 1,$(UPLOAD_FORCE)),)
$(error UPLOAD_FORCE must be 0 or 1)
endif
.PHONY: architecture-ready check-architecture app-check intellisense mavlink firmware mcuboot host-tools verify dima_rover \
		upload-ready upload-preflight upload \
		__dima_clean_progress __dima_prepare_make_includes \
		__dima_prepare_generated __dima_summary \
		FORCE_ARCHITECTURE_CHECK FORCE_MCUBOOT_BUILD FORCE_KEY_IDENTITY_CHECK

# Stabilize content-addressed/generated outputs before the public wrapper takes
# its dry-run snapshot.  A generator may run because an input timestamp changed
# yet preserve every output timestamp when the content is unchanged; planning
# before that decision overestimates the downstream compile/link actions.
# 先收敛 DroneCAN、UM982、uORB、参数、Metadata 与 MAVLink 生成闭包，
# 再截取 dry-run；生成内容未变时保留 mtime，避免虚假的全量重编译计划。
__dima_prepare_make_includes: $(DRONECAN_GENERATED_MAKEFILE)
	@:

__dima_prepare_generated: $(DIMA_DRONECAN_GENERATED_OUTPUTS) \
		$(UM982_CONTRACT_HEADER) \
		$(MESSAGE_GENERATED_OUTPUTS) $(PARAMETER_GENERATED_OUTPUTS) \
		$(PARAMETER_METADATA_OUTPUTS) \
		$(MAVLINK_LIBRARY_HEADER)
	@:

FORCE_ARCHITECTURE_CHECK:

$(ARCHITECTURE_VERIFY_STAMP): $(ARCHITECTURE_FORCE_PREREQUISITE) \
		$(ARCHITECTURE_CHECK_TOOL) $(ARCHITECTURE_CACHE_TOOL) | \
		parameter-metadata-verify $(BUILD_DIR)
	$(DIMA_PROGRESS_RUN) --label ARCH --target "$@" \
		--display "check-architecture" -- \
		env PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 \
			PYTHONPATH="$(HOST_PYTHON_DIR)" \
			$(PYTHON) $(ARCHITECTURE_CHECK_TOOL) --stamp "$@"

architecture-ready: $(ARCHITECTURE_VERIFY_STAMP)
	@:

check-architecture: architecture-ready
	@:

# Fast application-only gate for local iterations.  It deliberately excludes
# image signing, MCUboot and Factory HEX generation; `verify` remains the full
# incremental image gate.
# app-check 仅证明 Application ELF；firmware/verify 才覆盖签名、MCUboot 与
# Factory HEX。两者的证据层级不可互换。
app-check: check-architecture $(BUILD_DIR)/$(TARGET).elf
	$(DIMA_PROGRESS_RUN) --label ELF --target "$(BUILD_DIR)/$(TARGET).elf" -- \
		$(PYTHON) $(APPLICATION_ELF_CHECK_TOOL) \
		--elf $(BUILD_DIR)/$(TARGET).elf

intellisense: $(COMPILE_COMMANDS_TOOL)
	$(DIMA_PROGRESS_RUN) --label COMPDB --target "$(COMPILE_COMMANDS_OUTPUT)" -- \
		$(PYTHON) $(COMPILE_COMMANDS_TOOL) \
		--output "$(COMPILE_COMMANDS_OUTPUT)" \
		$(if $(strip $(GCC_PATH)),--gcc-path "$(GCC_PATH)",) \
		--make-variable "DIMA_BUILD_PROFILE=$(DIMA_BUILD_PROFILE)" \
		--make-variable "DEBUG=$(DEBUG)"

firmware: mavlink check-architecture \
          $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex \
          $(BUILD_DIR)/$(TARGET).bin $(SIGNED_BIN) \
          $(MCUBOOT_BUILD_DIR)/mcuboot.hex $(FACTORY_HEX)

clean: __dima_clean_progress

__dima_clean_progress:
	$(DIMA_PROGRESS_RUN) --label CLEAN --target "$(BUILD_DIR)" \
		--display "$(BUILD_DIR)" -- rm -fR "$(BUILD_DIR)"

GENERATION_HOST_REQUIREMENTS := tools/generation/requirements-windows.txt

# 生成器依赖使用固定归档哈希并禁用隐式依赖解析，避免 PyPI 最新版本改变
# 参数、uORB 或 Metadata 产物。二进制 wheel 与正式 PlatformIO Python 3.11 x64
# 环境绑定，其他主机只负责发起 Windows 原生构建。
$(HOST_TOOLS_STAMP): $(MCUBOOT_ROOT)/scripts/requirements.txt \
		$(GENERATION_HOST_REQUIREMENTS) make/release.mk
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
		$(PYTHON) -m pip install --disable-pip-version-check \
			--upgrade --no-deps --require-hashes \
			--only-binary=PyYAML,MarkupSafe \
			--target "$$tmp" -r "$(GENERATION_HOST_REQUIREMENTS)"; \
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

# -H 0x400 为 MCUboot 头，-S 0xC0000 为完整 Primary slot 上限；签名包仍从
# slot 基址 0x08040000 写入，向量位于头后的 0x08040400。
$(SIGNED_BIN): $(BUILD_DIR)/$(TARGET).bin $(KEY_ID_STAMP) \
               $(HOST_TOOLS_STAMP) $(IMGTOOL) \
               GNUmakefile make/project.mk make/release.mk \
               $(IMAGE_VERSION_STAMP) | $(BUILD_DIR)
	$(DIMA_PROGRESS_RUN) --label SIGN --target "$@" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(IMGTOOL) \
		sign -k "$(KEY_FILE)" --align 32 --max-align 32 \
		-v $(IMAGE_VERSION) -H 0x400 --pad-header -S 0xC0000 $< $@

# Routine MCUboot OTA only consumes the signed application.  Cache its two
# direct gates so an unchanged image does not rebuild MCUboot/Factory HEX or
# rerun the full release-layout verification on every upload.
# 默认 OTA 只消费 signed Application；缓存的是本地 ELF 布局与签名验证结果，
# 不是板端成功。上传工具仍执行 MCUboot TEST、reset 与应用身份闭环。
$(UPLOAD_VERIFY_STAMP): $(BUILD_DIR)/$(TARGET).elf $(SIGNED_BIN) \
                        $(ARCHITECTURE_VERIFY_STAMP) \
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

# signed BIN 的地址域从 Primary slot 基址开始，绝不能按 0x08000000 写入。
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
# 单独 dima_rover 是完整发布镜像门禁；与 upload 联用时仅归一化构建范围，
# 不放宽签名与上传协议验证。
ifneq ($(filter upload,$(MAKECMDGOALS)),)
dima_rover: upload-ready
else
dima_rover: check-architecture verify
endif

upload-preflight: $(UPLOAD_TOOL) $(MCUMGR_BOOTSTRAP_TOOL) $(MAVLINK_BOOTSTRAP) \
		$(MAVLINK_LOCK) $(FIRMWARE_IDENTITY_UPLOAD_CONTRACT)
	@env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(UPLOAD_TOOL) \
		--preflight-only --mcumgr "$(MCUMGR)" \
		--identity-contract "$(FIRMWARE_IDENTITY_UPLOAD_CONTRACT)" \
		--tools-cache "$(HOST_TOOLS_CACHE_ROOT)" \
		$(if $(strip $(MCUMGR_PORT)),--port "$(MCUMGR_PORT)",) \
		--baud "$(MCUMGR_BAUD)" --mtu "$(MCUMGR_MTU)" \
		--wait-seconds "$(UPLOAD_WAIT_SECONDS)"

upload: dima_rover $(UPLOAD_TOOL) $(MCUMGR_BOOTSTRAP_TOOL) $(MAVLINK_BOOTSTRAP) \
		$(MAVLINK_LOCK) $(FIRMWARE_IDENTITY_UPLOAD_CONTRACT)
	$(DIMA_PROGRESS_RUN) --label UPLOAD --target "$(UPLOAD_IMAGE)" -- \
		env PYTHONPATH=$(HOST_PYTHON_DIR) $(PYTHON) $(UPLOAD_TOOL) \
		--image "$(UPLOAD_IMAGE)" --imgtool "$(IMGTOOL)" \
		--identity-contract "$(FIRMWARE_IDENTITY_UPLOAD_CONTRACT)" \
		--mcumgr "$(MCUMGR)" --tools-cache "$(HOST_TOOLS_CACHE_ROOT)" \
		$(if $(strip $(MCUMGR_PORT)),--port "$(MCUMGR_PORT)",) \
		--baud "$(MCUMGR_BAUD)" --mtu "$(MCUMGR_MTU)" \
		--max-window "$(MCUMGR_MAX_WINDOW)" \
		--wait-seconds "$(UPLOAD_WAIT_SECONDS)" \
		$(UPLOAD_FORCE_FLAG)

__dima_summary:
	$(PYTHON) $(BUILD_PROGRESS_TOOL) summary \
		--goals "$(DIMA_SUMMARY_GOALS)" --version "$(IMAGE_VERSION)" \
		--app-elf "$(BUILD_DIR)/$(TARGET).elf" \
		--boot-bin "$(MCUBOOT_BUILD_DIR)/mcuboot.bin" \
		--signed "$(SIGNED_BIN)" --factory "$(FACTORY_HEX)" \
		--layout "Boards/H743/Inc/boot_layout.h" \
		--size-tool "$(DIMA_REAL_SZ)" \
		$(DIMA_PROGRESS_NO_COLOR_FLAG)
