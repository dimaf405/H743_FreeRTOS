# 按运行 Make 的主机选择原生 Python，WSL 属于 Linux，不切换到 Windows 进程。
# MAKE_HOST 来自 Make 自身，避免 WSL 继承 OS=Windows_NT 时误选 python.exe。
ifneq ($(findstring linux,$(MAKE_HOST)),)
PYTHON ?= python3
DIMA_DEFAULT_BUILD_DIR := build-linux
else
DIMA_USERPROFILE_POSIX := $(subst \,/,$(USERPROFILE))
DIMA_PLATFORMIO_PYTHON := $(DIMA_USERPROFILE_POSIX)/.platformio/penv/Scripts/python.exe
PYTHON ?= $(if $(wildcard $(DIMA_PLATFORMIO_PYTHON)),$(DIMA_PLATFORMIO_PYTHON),python.exe)
DIMA_DEFAULT_BUILD_DIR := build
endif

# 缓存目录始终由本机 Python 计算，Linux Make 的规则目标不能出现 Windows 盘符冒号。
HOST_TOOLS_CACHE_ROOT ?= $(shell $(PYTHON) -c "import pathlib; print((pathlib.Path.home() / '.cache' / 'dima-rover' / 'host-tools').as_posix())")
# project.mk 的生成规则在解析时就展开前置条件，必须提前定义 stamp，
# 否则首次 Linux 构建会在安装 Cerberus 等正式依赖之前启动生成器。
HOST_PYTHON_DIR = $(HOST_TOOLS_CACHE_ROOT)/host-python
HOST_TOOLS_STAMP = $(HOST_TOOLS_CACHE_ROOT)/.host-tools-installed
