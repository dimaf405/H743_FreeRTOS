# Stable user-owned entry point.  STM32CubeMX may regenerate Makefile; project
# sources, C++ support and release-image rules stay in make/project.mk.

DIMA_WINDOWS_NATIVE := $(filter Windows_NT,$(OS))
ifeq ($(DIMA_WINDOWS_NATIVE),)
$(error Windows-native GNU Make is required (OS=$(OS)); WSL/Linux builds are disabled)
endif

DIMA_USERPROFILE_POSIX := $(subst \,/,$(USERPROFILE))
DIMA_PLATFORMIO_PYTHON := $(DIMA_USERPROFILE_POSIX)/.platformio/penv/Scripts/python.exe
PYTHON ?= $(if $(wildcard $(DIMA_PLATFORMIO_PYTHON)),$(DIMA_PLATFORMIO_PYTHON),python.exe)
HOST_TOOLS_CACHE_ROOT ?= $(shell $(PYTHON) -c "import pathlib; print((pathlib.Path.home() / '.cache' / 'dima-rover' / 'host-tools').as_posix())")
DIMA_ARM_GCC_BOOTSTRAP := tools/bootstrap_arm_gcc.py

DIMA_BUILD_INTERNAL ?= 0

ifeq ($(DIMA_BUILD_INTERNAL),1)

.DEFAULT_GOAL := firmware

include Makefile
include make/project.mk

else

# Public invocations first enumerate the exact work with an output-synchronized
# dry-run, then execute the same goals through one recursive Make so its
# jobserver and command-line variables remain authoritative.
DIMA_REQUESTED_GOALS := $(if $(strip $(MAKECMDGOALS)),$(MAKECMDGOALS),firmware)
DIMA_SUMMARY_GOALS := $(filter firmware mcuboot verify dima_rover upload,$(DIMA_REQUESTED_GOALS))
DIMA_SHORT_MAKEFLAGS := $(filter-out --% %=%,$(firstword $(MAKEFLAGS)))
DIMA_DRY_RUN := $(findstring n,$(DIMA_SHORT_MAKEFLAGS))
DIMA_NO_COLOR_FLAG := $(if $(strip $(NO_COLOR)),--no-color,)
DIMA_OUTPUT_SYNC_FLAG := $(if $(findstring output-sync,$(.FEATURES)),--output-sync=target,)
DIMA_TOOLCHAIN_GOALS := $(filter app-check firmware mcuboot verify dima_rover upload intellisense,$(DIMA_REQUESTED_GOALS))

.DEFAULT_GOAL := __dima_dispatch
.PHONY: __dima_dispatch $(DIMA_REQUESTED_GOALS)

$(DIMA_REQUESTED_GOALS): __dima_dispatch
	@:

ifneq ($(DIMA_DRY_RUN),)

__dima_dispatch:
	+@$(MAKE) --no-print-directory -f GNUmakefile \
		DIMA_BUILD_INTERNAL=1 \
		DIMA_PROGRESS_STATE=/tmp/dima-progress-dry-run-not-used \
		$(DIMA_REQUESTED_GOALS)

else

__dima_dispatch:
	+@set -eu; \
		if test -n "$(filter upload,$(DIMA_REQUESTED_GOALS))"; then \
			$(MAKE) --no-print-directory -f GNUmakefile \
				DIMA_BUILD_INTERNAL=1 upload-preflight; \
		fi; \
		toolchain_path="$(GCC_PATH)"; \
		if test -n "$(DIMA_TOOLCHAIN_GOALS)" && test -z "$$toolchain_path"; then \
			toolchain_path=$$($(PYTHON) $(DIMA_ARM_GCC_BOOTSTRAP) \
				--cache-root "$(HOST_TOOLS_CACHE_ROOT)" --quiet-cache); \
		fi; \
		if test -n "$$toolchain_path"; then \
			printf '[TOOLCHAIN] Build\n  Arm GCC    : %s\n\n' "$$toolchain_path"; \
		fi; \
		progress_dir=$$($(PYTHON) -c "import pathlib,tempfile; print(pathlib.Path(tempfile.mkdtemp(prefix='dima-build-progress.')).as_posix())"); \
		cleanup() { \
			status=$$?; \
			trap - EXIT HUP INT TERM; \
			if test -n "$$progress_dir"; then \
				$(PYTHON) -c "import shutil,sys; shutil.rmtree(sys.argv[1], ignore_errors=True)" "$$progress_dir"; \
			fi; \
			exit "$$status"; \
		}; \
		trap cleanup EXIT HUP INT TERM; \
		plan="$$progress_dir/plan.txt"; \
		state="$$progress_dir/state.json"; \
		$(MAKE) --no-print-directory -f GNUmakefile -n $(DIMA_OUTPUT_SYNC_FLAG) \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE="$$state" \
			GCC_PATH="$$toolchain_path" \
			$(DIMA_REQUESTED_GOALS) >"$$plan"; \
		$(PYTHON) tools/build_progress.py prepare \
			--plan "$$plan" --state "$$state" \
			--goals "$(DIMA_REQUESTED_GOALS)" $(DIMA_NO_COLOR_FLAG); \
		$(MAKE) --no-print-directory -f GNUmakefile \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE="$$state" \
			GCC_PATH="$$toolchain_path" \
			$(DIMA_REQUESTED_GOALS); \
		$(PYTHON) tools/build_progress.py finish \
			--state "$$state" $(DIMA_NO_COLOR_FLAG); \
		if test -n "$(strip $(DIMA_SUMMARY_GOALS))"; then \
			$(MAKE) --no-print-directory -s -f GNUmakefile \
				DIMA_BUILD_INTERNAL=1 \
				GCC_PATH="$$toolchain_path" \
				DIMA_SUMMARY_GOALS="$(DIMA_REQUESTED_GOALS)" \
				__dima_summary; \
		fi; \
		trap - EXIT HUP INT TERM; \
		$(PYTHON) -c "import shutil,sys; shutil.rmtree(sys.argv[1], ignore_errors=True)" "$$progress_dir"

endif
endif
