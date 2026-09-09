# Stable user-owned entry point.  STM32CubeMX may regenerate Makefile; project
# sources and C++ support stay in make/project.mk; signing, MCUboot and upload
# rules stay in make/release.mk.
# 这是用户维护的唯一构建入口：CubeMX 可重生成 Makefile，但项目源码闭包、
# 生成合同与发布链分别固定在 project.mk/release.mk，避免生成器覆盖产品规则。

include make/host.mk

# CubeMX 的 Makefile 会赋值 BUILD_DIR；只对未显式指定的目录覆盖默认值，
# 使 Linux/Windows 的对象、依赖和生成物各自独立，同时保留用户的 BUILD_DIR 入口。
ifneq ($(origin BUILD_DIR),command line)
override BUILD_DIR := $(if $(strip $(BUILD_DIR)),$(BUILD_DIR),$(DIMA_DEFAULT_BUILD_DIR))
endif
export BUILD_DIR
DIMA_ARM_GCC_BOOTSTRAP := tools/bootstrap_arm_gcc.py
DIMA_DEFAULT_JOBS ?= $(shell $(PYTHON) tools/build_progress.py jobs)
DIMA_CCACHE ?= auto
DIMA_BUILD_TRACE ?= 0
export DIMA_BUILD_TRACE
ifeq ($(strip $(DIMA_DEFAULT_JOBS)),)
$(error DIMA_DEFAULT_JOBS must be a positive job count)
endif

DIMA_BUILD_INTERNAL ?= 0
DIMA_BUILD_PROFILE ?= release
DIMA_VALID_BUILD_PROFILES := release debug
ifneq ($(words $(DIMA_BUILD_PROFILE)),1)
$(error DIMA_BUILD_PROFILE must be release or debug)
endif
ifeq ($(filter $(DIMA_BUILD_PROFILE),$(DIMA_VALID_BUILD_PROFILES)),)
$(error DIMA_BUILD_PROFILE must be release or debug)
endif

ifeq ($(DIMA_BUILD_INTERNAL),1)

.DEFAULT_GOAL := firmware

include Makefile
include make/project.mk

else

# Release-oriented public invocations first enumerate the exact work with an
# output-synchronized dry-run.  The upload-only exception below executes one
# recursive dependency graph because its protocol stages already report
# progress and an extra planning pass is measurable overhead.
# 公共目标先用 dry-run 固化真实依赖图，再按该图显示进度并执行一次；上传是唯一
# 例外，因为 mcumgr 已有阶段进度，且正常只复用已签名 Application。
DIMA_REQUESTED_GOALS := $(if $(strip $(MAKECMDGOALS)),$(MAKECMDGOALS),firmware)
DIMA_SUMMARY_GOALS := $(if $(filter upload,$(DIMA_REQUESTED_GOALS)),,$(filter firmware mcuboot verify dima_rover,$(DIMA_REQUESTED_GOALS)))
DIMA_SHORT_MAKEFLAGS := $(filter-out --% %=%,$(firstword $(MAKEFLAGS)))
DIMA_DRY_RUN := $(findstring n,$(DIMA_SHORT_MAKEFLAGS))
DIMA_NO_COLOR_FLAG := $(if $(strip $(NO_COLOR)),--no-color,)
DIMA_OUTPUT_SYNC_FLAG := $(if $(findstring output-sync,$(.FEATURES)),--output-sync=target,)
DIMA_TOOLCHAIN_GOALS := $(filter app-check firmware mcuboot verify dima_rover upload upload-ready intellisense,$(DIMA_REQUESTED_GOALS))
DIMA_PARALLEL_FLAG := $(if $(filter -j% --jobs%,$(MAKEFLAGS)),,-j$(DIMA_DEFAULT_JOBS))
DIMA_STABILIZE_GENERATED_GOALS := $(filter app-check firmware verify \
	dima_rover upload upload-ready intellisense check-architecture \
	parameter-metadata-verify,$(DIMA_REQUESTED_GOALS))
DIMA_UPLOAD_FLOW_GOALS := $(filter upload upload-ready upload-preflight,$(DIMA_REQUESTED_GOALS))
DIMA_FAST_UPLOAD_ALLOWED_GOALS := dima_rover upload upload-ready upload-preflight
DIMA_FAST_UPLOAD_EXTRA_GOALS := $(filter-out $(DIMA_FAST_UPLOAD_ALLOWED_GOALS),$(DIMA_REQUESTED_GOALS))
# The legacy `dima_rover upload` spelling is fast only when upload is actually
# present.  Any additional verification/build goal keeps the planned release
# dispatcher below.
DIMA_FAST_UPLOAD_DISPATCH := $(and \
	$(DIMA_UPLOAD_FLOW_GOALS),\
	$(if $(DIMA_FAST_UPLOAD_EXTRA_GOALS),,1),\
	$(if $(filter dima_rover,$(DIMA_REQUESTED_GOALS)),\
		$(filter upload,$(DIMA_REQUESTED_GOALS)),1))

# 同一会话计时覆盖主机准备到上传结束；不通过 Python 包装递归 Make，保留
# GNU Make jobserver。逐对象计时仅在 TRACE=1 时启用，日常 OTA 不增加编译包装进程。
define DIMA_START_SESSION
DIMA_BUILD_SESSION=$$($(PYTHON) tools/build_progress.py session-start \
	--build-dir "$(BUILD_DIR)" --cache-root "$(HOST_TOOLS_CACHE_ROOT)" \
	--ccache "$(if $(DIMA_TOOLCHAIN_GOALS),$(DIMA_CCACHE),off)" --jobs="$(DIMA_PARALLEL_FLAG)"); \
export DIMA_BUILD_SESSION; \
IFS= read -r DIMA_CCACHE_EXECUTABLE < "$$DIMA_BUILD_SESSION/ccache-path"; \
export DIMA_CCACHE_EXECUTABLE; \
export CCACHE_DIR="$(HOST_TOOLS_CACHE_ROOT)/compiler-cache"; \
export CCACHE_BASEDIR="$(CURDIR)" CCACHE_MAXSIZE=2G CCACHE_COMPILERCHECK=content; \
export CCACHE_SLOPPINESS= CCACHE_IGNOREHEADERS= CCACHE_IGNOREOPTIONS=; \
export CCACHE_STATSLOG="$$DIMA_BUILD_SESSION/ccache.log"; \
progress_dir=; \
cleanup() { \
	status=$$?; trap - EXIT HUP INT TERM; \
	$(PYTHON) tools/build_progress.py session-finish \
		--session "$$DIMA_BUILD_SESSION" --exit-code "$$status" || true; \
	if test -n "$$progress_dir"; then \
		$(PYTHON) -c "import shutil,sys; shutil.rmtree(sys.argv[1], ignore_errors=True)" "$$progress_dir"; \
	fi; \
	exit "$$status"; \
}; \
trap cleanup EXIT; \
trap 'exit 129' HUP; trap 'exit 130' INT; trap 'exit 143' TERM;
endef

.DEFAULT_GOAL := __dima_dispatch
.PHONY: __dima_dispatch $(DIMA_REQUESTED_GOALS)

$(DIMA_REQUESTED_GOALS): __dima_dispatch
	@:

ifneq ($(DIMA_DRY_RUN),)

__dima_dispatch:
	+@$(MAKE) $(DIMA_PARALLEL_FLAG) --no-print-directory -f GNUmakefile \
		DIMA_BUILD_INTERNAL=1 \
		DIMA_BUILD_PROFILE="$(DIMA_BUILD_PROFILE)" \
		DIMA_PROGRESS_STATE= \
		$(DIMA_REQUESTED_GOALS)

else

ifneq ($(DIMA_FAST_UPLOAD_DISPATCH),)

# Upload already has protocol-level stages and normally reuses a signed image.
# Prepare the shared firmware identity before dependency scanning, then execute
# the upload graph once. Any stale source, generated contract, key, ELF or
# signature still rebuilds before the uploader can start.
# 先在独立 Make 阶段收敛身份头，防止本轮读取到旧 mtime 后漏编译消费者。
# 快速 OTA 保留这个准备阶段，仍省略进度计划和 Factory/MCUboot 非必要重建；签名、哈希、
# Application ELF、Secondary/pending/reset/identity 等安全证明仍由依赖链保留。
__dima_dispatch:
	+@set -eu; \
		$(DIMA_START_SESSION) \
		toolchain_path="$(GCC_PATH)"; \
		if test -n "$(DIMA_TOOLCHAIN_GOALS)" && test -z "$$toolchain_path"; then \
			toolchain_path=$$($(PYTHON) $(DIMA_ARM_GCC_BOOTSTRAP) \
				--cache-root "$(HOST_TOOLS_CACHE_ROOT)" --quiet-cache); \
		fi; \
		printf '[FAST OTA] Dependency checks\n  Progress plan: skipped\n\n'; \
		if test -n "$$toolchain_path"; then \
			printf '[TOOLCHAIN] Build\n  Arm GCC    : %s\n\n' "$$toolchain_path"; \
		fi; \
		$(MAKE) $(DIMA_PARALLEL_FLAG) --no-print-directory -s -f GNUmakefile \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE= \
			DIMA_BUILD_PROFILE="$(DIMA_BUILD_PROFILE)" \
			GCC_PATH="$$toolchain_path" firmware-identity-generated; \
		$(MAKE) $(DIMA_PARALLEL_FLAG) --no-print-directory -s -f GNUmakefile \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE= \
			DIMA_BUILD_PROFILE="$(DIMA_BUILD_PROFILE)" \
			GCC_PATH="$$toolchain_path" \
			$(DIMA_REQUESTED_GOALS)

else

__dima_dispatch:
	+@set -eu; \
		$(DIMA_START_SESSION) \
		toolchain_path="$(GCC_PATH)"; \
		if test -n "$(DIMA_TOOLCHAIN_GOALS)" && test -z "$$toolchain_path"; then \
			toolchain_path=$$($(PYTHON) $(DIMA_ARM_GCC_BOOTSTRAP) \
				--cache-root "$(HOST_TOOLS_CACHE_ROOT)" --quiet-cache); \
		fi; \
		if test -n "$$toolchain_path"; then \
			printf '[TOOLCHAIN] Build\n  Arm GCC    : %s\n\n' "$$toolchain_path"; \
		fi; \
		progress_dir=$$($(PYTHON) -c "import pathlib,tempfile; print(pathlib.Path(tempfile.mkdtemp(prefix='dima-build-progress.')).as_posix())"); \
		generated_prepare_goal=__dima_prepare_make_includes; \
		if test -n "$(DIMA_STABILIZE_GENERATED_GOALS)"; then \
			generated_prepare_goal=__dima_prepare_generated; \
		fi; \
		$(MAKE) $(DIMA_PARALLEL_FLAG) --no-print-directory -s -f GNUmakefile \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE= \
			DIMA_BUILD_PROFILE="$(DIMA_BUILD_PROFILE)" \
			GCC_PATH="$$toolchain_path" \
			"$$generated_prepare_goal"; \
		plan="$$progress_dir/plan.txt"; \
		state="$$progress_dir/state.json"; \
		$(MAKE) $(DIMA_PARALLEL_FLAG) --no-print-directory -f GNUmakefile -n $(DIMA_OUTPUT_SYNC_FLAG) \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE="$$state" \
			DIMA_BUILD_PROFILE="$(DIMA_BUILD_PROFILE)" \
			GCC_PATH="$$toolchain_path" \
			$(DIMA_REQUESTED_GOALS) >"$$plan"; \
		$(PYTHON) tools/build_progress.py prepare \
			--plan "$$plan" --state "$$state" \
			--goals "$(DIMA_REQUESTED_GOALS)" $(DIMA_NO_COLOR_FLAG); \
		$(MAKE) $(DIMA_PARALLEL_FLAG) --no-print-directory -f GNUmakefile \
			DIMA_BUILD_INTERNAL=1 DIMA_PROGRESS_STATE="$$state" \
			DIMA_BUILD_PROFILE="$(DIMA_BUILD_PROFILE)" \
			GCC_PATH="$$toolchain_path" \
			$(DIMA_REQUESTED_GOALS); \
		$(PYTHON) tools/build_progress.py finish \
			--state "$$state" $(DIMA_NO_COLOR_FLAG); \
		if test -n "$(strip $(DIMA_SUMMARY_GOALS))"; then \
			$(MAKE) --no-print-directory -s -f GNUmakefile \
				DIMA_BUILD_INTERNAL=1 \
				DIMA_BUILD_PROFILE="$(DIMA_BUILD_PROFILE)" \
				GCC_PATH="$$toolchain_path" \
				DIMA_SUMMARY_GOALS="$(DIMA_REQUESTED_GOALS)" \
				__dima_summary; \
		fi

endif
endif
endif
