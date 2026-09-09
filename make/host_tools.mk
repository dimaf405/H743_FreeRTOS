# 安装配方、依赖内容和 Python ABI 由 host.mk 生成目录身份；相同内容的
# checkout 不重装工具，缺失/失败仍走同一有锁、原子安装入口。
GENERATION_HOST_REQUIREMENTS := tools/generation/requirements-host.txt
$(HOST_TOOLS_STAMP): | $(MCUBOOT_ROOT)/scripts/requirements.txt \
		$(GENERATION_HOST_REQUIREMENTS) tools/generation/host_tools.py
	$(DIMA_PROGRESS_RUN) --label HOST --target "$@" \
		--display "$(HOST_PYTHON_DIR)" -- \
		$(PYTHON) tools/generation/host_tools.py --output "$(HOST_PYTHON_DIR)" \
			--signing "$(MCUBOOT_ROOT)/scripts/requirements.txt" \
			--generation "$(GENERATION_HOST_REQUIREMENTS)"
