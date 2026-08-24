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

check-architecture: $(ARCHITECTURE_CHECK_TOOL) parameter-metadata-verify
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
               GNUmakefile make/project.mk make/release.mk \
               $(IMAGE_VERSION_STAMP) | $(BUILD_DIR)
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
