# Stable user-owned entry point.  STM32CubeMX may regenerate Makefile; project
# sources, C++ support and release-image rules stay in make/project.mk.

DIMA_WINDOWS_MAKE_HOST := $(strip $(or \
	$(findstring cygwin,$(MAKE_HOST)), \
	$(findstring mingw,$(MAKE_HOST)), \
	$(findstring msys,$(MAKE_HOST))))
ifeq ($(DIMA_WINDOWS_MAKE_HOST),)
$(error Windows-native GNU Make is required (MAKE_HOST=$(MAKE_HOST)); WSL/Linux builds are disabled)
endif

DIMA_BUILD_INTERNAL ?= 0

ifeq ($(DIMA_BUILD_INTERNAL),1)

.DEFAULT_GOAL := firmware

include Makefile
include make/project.mk

else

# Public invocations first enumerate the exact work with an output-synchronized
# dry-run, then execute the same goals through one recursive Make so its
# jobserver and command-line variables remain authoritative.
PYTHON ?= python3
DIMA_REQUESTED_GOALS := $(if $(strip $(MAKECMDGOALS)),$(MAKECMDGOALS),firmware)
DIMA_SUMMARY_GOALS := $(filter firmware mcuboot verify dima_rover upload,$(DIMA_REQUESTED_GOALS))
DIMA_SHORT_MAKEFLAGS := $(filter-out --% %=%,$(firstword $(MAKEFLAGS)))
DIMA_DRY_RUN := $(findstring n,$(DIMA_SHORT_MAKEFLAGS))
DIMA_NO_COLOR_FLAG := $(if $(strip $(NO_COLOR)),--no-color,)

.DEFAULT_GOAL := __dima_dispatch
.PHONY: __dima_dispatch $(DIMA_REQUESTED_GOALS)

$(DIMA_REQUESTED_GOALS): __dima_dispatch ;

ifneq ($(DIMA_DRY_RUN),)

__dima_dispatch:
	+@$(MAKE) --no-print-directory -f GNUmakefile \
		DIMA_BUILD_INTERNAL=1 \
		DIMA_PROGRESS_STATE=/tmp/dima-progress-dry-run-not-used \
		$(DIMA_REQUESTED_GOALS)

else

__dima_dispatch:
	+@set -eu; \
		progress_dir=$$(mktemp -d "$${TMPDIR:-/tmp}/dima-build-progress.XXXXXX"); \
		cleanup() { \
			status=$$?; \
			trap - EXIT HUP INT TERM; \
			if test -n "$$progress_dir" && test -d "$$progress_dir"; then \
				rm -rf -- "$$progress_dir"; \
			fi; \
			exit "$$status"; \
		}; \
		trap cleanup EXIT HUP INT TERM; \
		plan="$$progress_dir/plan.txt"; \
		state="$$progress_dir/state.json"; \
		$(MAKE) --no-print-directory -f GNUmakefile -n --output-sync=target \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE="$$state" \
			$(DIMA_REQUESTED_GOALS) >"$$plan"; \
		$(PYTHON) tools/build_progress.py prepare \
			--plan "$$plan" --state "$$state" \
			--goals "$(DIMA_REQUESTED_GOALS)" $(DIMA_NO_COLOR_FLAG); \
		$(MAKE) --no-print-directory -f GNUmakefile \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE="$$state" \
			$(DIMA_REQUESTED_GOALS); \
		$(PYTHON) tools/build_progress.py finish \
			--state "$$state" $(DIMA_NO_COLOR_FLAG); \
		if test -n "$(strip $(DIMA_SUMMARY_GOALS))"; then \
			$(MAKE) --no-print-directory -s -f GNUmakefile \
				DIMA_BUILD_INTERNAL=1 \
				DIMA_SUMMARY_GOALS="$(DIMA_REQUESTED_GOALS)" \
				__dima_summary; \
		fi; \
		trap - EXIT HUP INT TERM; \
		rm -rf -- "$$progress_dir"

endif
endif
