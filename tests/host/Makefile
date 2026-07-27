# Invoke this file with `make -C <repo-root> -f tests/host/Makefile ...`.
# All paths intentionally remain root-relative: Make prerequisites split on
# whitespace, so embedding an absolute repository path is not portable.
CXX ?= g++
CC ?= gcc
ROOT := .
BUILD_DIR := build/host-tests
OBJECT_DIR := $(BUILD_DIR)/obj
TEST_BIN := $(BUILD_DIR)/host_tests
FREERTOS_BACKEND_BUILD_DIR := build/host-freertos-backend-tests
FREERTOS_BACKEND_TEST_BIN := $(FREERTOS_BACKEND_BUILD_DIR)/host_tests
BOARD_INIT_TEST_BUILD_DIR := build/host-board-init-tests
BOARD_INIT_DEFAULT_TEST_BIN := $(BOARD_INIT_TEST_BUILD_DIR)/default/board_init_test
BOARD_INIT_SD_TEST_BIN := $(BOARD_INIT_TEST_BUILD_DIR)/sd-enabled/board_init_test
APP_BOOTSTRAP_TEST_BUILD_DIR := build/host-app-bootstrap-tests
APP_BOOTSTRAP_TEST_BIN := $(APP_BOOTSTRAP_TEST_BUILD_DIR)/app_bootstrap_test
HELLO_WORLD_TEST_BUILD_DIR := build/host-hello-world-tests
HELLO_WORLD_TEST_BIN := $(HELLO_WORLD_TEST_BUILD_DIR)/host_tests
BOOT_HEALTH_TEST_BUILD_DIR := build/host-boot-health-tests
BOOT_HEALTH_TEST_BIN := $(BOOT_HEALTH_TEST_BUILD_DIR)/host_tests
USB_CONSOLE_TEST_BUILD_DIR := build/host-usb-console-tests
USB_CONSOLE_TEST_BIN := $(USB_CONSOLE_TEST_BUILD_DIR)/host_tests
USB_CONSOLE_FREERTOS_BUILD_DIR := build/host-usb-console-freertos-tests
USB_CONSOLE_FREERTOS_TEST_BIN := $(USB_CONSOLE_FREERTOS_BUILD_DIR)/host_tests
NO_HEAP_TEST_BUILD_DIR := build/host-no-heap-tests
NO_HEAP_TEST_BIN := $(NO_HEAP_TEST_BUILD_DIR)/no_heap_test
USB_CDC_GLUE_TEST_BUILD_DIR := build/host-usb-cdc-glue-tests
USB_CDC_GLUE_TEST_BIN := $(USB_CDC_GLUE_TEST_BUILD_DIR)/usb_cdc_glue_test
SPEED_TO_PWM_TEST_BUILD_DIR := build/host-speed-to-pwm-tests
SPEED_TO_PWM_TEST_BIN := $(SPEED_TO_PWM_TEST_BUILD_DIR)/host_tests

TEST_CPP_SOURCES := \
	tests/host/boot_health_test.cpp \
	tests/host/hello_world_test.cpp \
	tests/host/module_manager_test.cpp \
	tests/host/messaging_test.cpp \
	tests/host/usb_console_test.cpp \
	tests/host/work_queue_test.cpp \
	tests/host/test_main.cpp

# Deliberately portable host implementation list.  Do not replace this with a
# find/glob: HAL/FreeRTOS board sources must never enter a host test build.
# If an implementation is intentionally merged into another listed file,
# update this list in the same change so the dependency graph stays explicit.
HOST_PRODUCTION_SOURCES ?= \
	App/runtime/lifecycle/module_manager.cpp \
	App/runtime/scheduling/work_queue.cpp \
	App/runtime/scheduling/scheduled_work_item.cpp \
	App/adapters/usb_console/usb_console.c \
	App/features/hello_world/hello_world.cpp \
	App/features/boot_health/boot_health.cpp

HOST_CPP_SOURCES := $(filter %.cpp,$(HOST_PRODUCTION_SOURCES))
HOST_C_SOURCES := $(filter %.c,$(HOST_PRODUCTION_SOURCES))
CPP_SOURCES := $(HOST_CPP_SOURCES) $(TEST_CPP_SOURCES)
C_SOURCES := $(HOST_C_SOURCES)
CPP_OBJECTS := $(patsubst %.cpp,$(OBJECT_DIR)/%.o,$(filter %.cpp,$(CPP_SOURCES)))
C_OBJECTS := $(patsubst %.c,$(OBJECT_DIR)/%.o,$(C_SOURCES))
OBJECTS := $(CPP_OBJECTS) $(C_OBJECTS)
DEPFILES := $(OBJECTS:.o=.d)

CPPFLAGS += -I$(ROOT) -DAPP_HOST_TEST=1 -DAPP_HELLO_WORLD_INTERVAL_MS=1000
CXXFLAGS += -std=c++17 -Wall -Wextra -Werror -pedantic -MMD -MP
CFLAGS += -std=c17 -Wall -Wextra -Werror -pedantic -MMD -MP
SANITIZE ?= 1

ifeq ($(SANITIZE),1)
SANITIZER_FLAGS := -fsanitize=address,undefined -fno-omit-frame-pointer
CXXFLAGS += $(SANITIZER_FLAGS)
CFLAGS += $(SANITIZER_FLAGS)
LDFLAGS += $(SANITIZER_FLAGS)
endif

.PHONY: all clean app-bootstrap-test hello-world-test boot-health-test regenerated-config-test board-init-test freertos-backend-test usb-console-test usb-console-freertos-backend-test usb-cdc-glue-test no-heap-test speed-to-pwm-test negative-messaging negative-usb-set-backend-seam negative-usb-backend-type-seam negative-usb-tx-result-type-seam negative-hello-dependencies-seam negative-hello-run-for-test-seam negative-boot-dependencies-seam negative-boot-run-for-test-seam

all: $(HOST_PRODUCTION_SOURCES) $(TEST_BIN)

$(TEST_BIN): $(OBJECTS) | $(BUILD_DIR)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@

$(OBJECT_DIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(OBJECT_DIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

freertos-backend-test:
	$(MAKE) -f tests/host/Makefile \
		BUILD_DIR=$(FREERTOS_BACKEND_BUILD_DIR) clean
	$(MAKE) -f tests/host/Makefile \
		BUILD_DIR=$(FREERTOS_BACKEND_BUILD_DIR) \
		CPPFLAGS='-I. -DAPP_FREERTOS_WORK_QUEUE_TEST_SEAM=1 -Itests/host -Itests/host/freertos_shim' \
		TEST_CPP_SOURCES='tests/host/freertos_work_queue_backend_test.cpp tests/host/test_main.cpp' \
		HOST_PRODUCTION_SOURCES='App/runtime/scheduling/work_queue.cpp App/runtime/scheduling/scheduled_work_item.cpp App/runtime/scheduling/freertos_work_queue.cpp' \
		all
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
		UBSAN_OPTIONS=halt_on_error=1 $(FREERTOS_BACKEND_TEST_BIN)

# USB console has an isolated portable semantic suite because the ordinary
# aggregate remains intentionally RED until the HelloWorld stage exists.
usb-console-test:
	$(MAKE) -f tests/host/Makefile BUILD_DIR=$(USB_CONSOLE_TEST_BUILD_DIR) clean
	$(MAKE) -f tests/host/Makefile \
		BUILD_DIR=$(USB_CONSOLE_TEST_BUILD_DIR) \
		TEST_CPP_SOURCES='tests/host/usb_console_test.cpp tests/host/test_main.cpp' \
		HOST_PRODUCTION_SOURCES='App/adapters/usb_console/usb_console.c' \
		all
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
	UBSAN_OPTIONS=halt_on_error=1 $(USB_CONSOLE_TEST_BIN)

# This target compiles usb_console.c without APP_HOST_TEST.  Its FreeRTOS and
# USB shims exercise the actual static-semaphore/scheduler/ISR branch.
usb-console-freertos-backend-test:
	$(MAKE) -f tests/host/Makefile BUILD_DIR=$(USB_CONSOLE_FREERTOS_BUILD_DIR) clean
	$(MAKE) -f tests/host/Makefile \
		BUILD_DIR=$(USB_CONSOLE_FREERTOS_BUILD_DIR) \
		CPPFLAGS='-Itests/host/usb_console_freertos_shim -I. -DAPP_USB_CONSOLE_FREERTOS_TEST_SEAM=1' \
		TEST_CPP_SOURCES='tests/host/usb_console_freertos_backend_test.cpp tests/host/test_main.cpp' \
		HOST_PRODUCTION_SOURCES='App/adapters/usb_console/usb_console.c' \
		all
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
	UBSAN_OPTIONS=halt_on_error=1 $(USB_CONSOLE_FREERTOS_TEST_BIN)

no-heap-test:
	rm -rf $(NO_HEAP_TEST_BUILD_DIR)
	mkdir -p $(NO_HEAP_TEST_BUILD_DIR)
	$(CC) -std=c17 -Wall -Wextra -Werror -pedantic $(SANITIZER_FLAGS) \
		tests/host/no_heap_test.c App/runtime/libc/no_heap.c \
		-o $(NO_HEAP_TEST_BIN)
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(NO_HEAP_TEST_BIN)

speed-to-pwm-test:
	$(MAKE) -f tests/host/Makefile BUILD_DIR=$(SPEED_TO_PWM_TEST_BUILD_DIR) clean
	$(MAKE) -f tests/host/Makefile \
		BUILD_DIR=$(SPEED_TO_PWM_TEST_BUILD_DIR) \
		CPPFLAGS='-I.' \
		TEST_CPP_SOURCES='tests/host/speed_to_pwm_test.cpp tests/host/test_main.cpp' \
		HOST_PRODUCTION_SOURCES='App/domain/motor/speed_to_pwm.cpp' \
		all
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
	UBSAN_OPTIONS=halt_on_error=1 $(SPEED_TO_PWM_TEST_BIN)

usb-cdc-glue-test:
	rm -rf $(USB_CDC_GLUE_TEST_BUILD_DIR)
	mkdir -p $(USB_CDC_GLUE_TEST_BUILD_DIR)
	$(CC) -std=c17 -Wall -Wextra -Werror -pedantic $(SANITIZER_FLAGS) \
		-Itests/host/usbd_cdc_if_shim -IUSB_DEVICE/App -I. \
		tests/host/usbd_cdc_if_test.c USB_DEVICE/App/usbd_cdc_if.c \
		-o $(USB_CDC_GLUE_TEST_BIN)
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(USB_CDC_GLUE_TEST_BIN)

# app_bootstrap.cpp has a dedicated shim test.  The legacy USB/MCUboot body in
# app_main.cpp intentionally stays out of this target and the portable suite.
app-bootstrap-test:
	rm -rf $(APP_BOOTSTRAP_TEST_BUILD_DIR)
	mkdir -p $(APP_BOOTSTRAP_TEST_BUILD_DIR)
	$(CXX) -std=c++17 -Wall -Wextra -Werror -pedantic $(SANITIZER_FLAGS) \
		-I. -Itests/host/freertos_shim \
		tests/host/app_bootstrap_test.cpp App/application/app_bootstrap.cpp \
		-o $(APP_BOOTSTRAP_TEST_BIN)
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(APP_BOOTSTRAP_TEST_BIN) existing-task
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(APP_BOOTSTRAP_TEST_BIN) no-tasks
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(APP_BOOTSTRAP_TEST_BIN) other-task
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(APP_BOOTSTRAP_TEST_BIN) failure-retry

hello-world-test:
	$(MAKE) -f tests/host/Makefile BUILD_DIR=$(HELLO_WORLD_TEST_BUILD_DIR) clean
	$(MAKE) -f tests/host/Makefile \
		BUILD_DIR=$(HELLO_WORLD_TEST_BUILD_DIR) \
		CPPFLAGS='-I. -DAPP_HOST_TEST=1 -DAPP_HELLO_WORLD_INTERVAL_MS=1000' \
		TEST_CPP_SOURCES='tests/host/hello_world_test.cpp tests/host/test_main.cpp' \
		HOST_PRODUCTION_SOURCES='App/runtime/scheduling/work_queue.cpp App/runtime/scheduling/scheduled_work_item.cpp App/features/hello_world/hello_world.cpp' \
		all
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
	UBSAN_OPTIONS=halt_on_error=1 $(HELLO_WORLD_TEST_BIN)

boot-health-test:
	$(MAKE) -f tests/host/Makefile BUILD_DIR=$(BOOT_HEALTH_TEST_BUILD_DIR) clean
	$(MAKE) -f tests/host/Makefile \
		BUILD_DIR=$(BOOT_HEALTH_TEST_BUILD_DIR) \
		CPPFLAGS='-I. -DAPP_HOST_TEST=1' \
		TEST_CPP_SOURCES='tests/host/boot_health_test.cpp tests/host/test_main.cpp' \
		HOST_PRODUCTION_SOURCES='App/runtime/scheduling/work_queue.cpp App/runtime/scheduling/scheduled_work_item.cpp App/features/boot_health/boot_health.cpp' \
		all
	@if nm -u $(BOOT_HEALTH_TEST_BUILD_DIR)/obj/App/features/boot_health/boot_health.o | \
		grep -Eq '(^|[[:space:]])(printf|puts|fflush|fwrite|_write)$$'; then \
		echo 'BOOT HEALTH OUTPUT DEPENDENCY ERROR' >&2; exit 1; \
	fi
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
	UBSAN_OPTIONS=halt_on_error=1 $(BOOT_HEALTH_TEST_BIN)

regenerated-config-test:
	python3 tests/host/check_regenerated_freertos_config.py

# board_init.c is tested separately with fake CubeMX headers/functions.  It is
# intentionally absent from HOST_PRODUCTION_SOURCES so HAL board code can never
# leak into the ordinary portable host binary.
board-init-test:
	rm -rf $(BOARD_INIT_TEST_BUILD_DIR)
	mkdir -p $(dir $(BOARD_INIT_DEFAULT_TEST_BIN)) $(dir $(BOARD_INIT_SD_TEST_BIN))
	$(CC) -std=c17 -Wall -Wextra -Werror -pedantic $(SANITIZER_FLAGS) \
		-Itests/host/board_init_shim -I. \
		-DEXPECT_BOARD_SD_INIT_AT_BOOT=0 \
		tests/host/board_init_test.c Boards/H743/Src/board_init.c \
		-o $(BOARD_INIT_DEFAULT_TEST_BIN)
	$(CC) -std=c17 -Wall -Wextra -Werror -pedantic $(SANITIZER_FLAGS) \
		-Itests/host/board_init_shim -I. \
		-DBOARD_SD_INIT_AT_BOOT=1 -DEXPECT_BOARD_SD_INIT_AT_BOOT=1 \
		tests/host/board_init_test.c Boards/H743/Src/board_init.c \
		-o $(BOARD_INIT_SD_TEST_BIN)
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(BOARD_INIT_DEFAULT_TEST_BIN)
	ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
		$(BOARD_INIT_SD_TEST_BIN)

negative-messaging:
	bash tests/host/run_negative_compile.sh tests/host/probes/messaging_nontrivial_topic_must_fail.cpp 'Topic requires trivially-copyable data' 'Topic.*NonTriviallyCopyableMessage|NonTriviallyCopyableMessage.*Topic'

negative-usb-set-backend-seam:
	bash tests/host/run_negative_compile.sh tests/host/probes/usb_test_set_backend_hidden_without_host_macro.cpp 'not declared|has not been declared|does not name a type|is not a member of|no member named' 'usb_console_test_set_backend'

negative-usb-backend-type-seam:
	bash tests/host/run_negative_compile.sh tests/host/probes/usb_test_backend_type_hidden_without_host_macro.cpp 'not declared|has not been declared|does not name a type|is not a member of|no member named' 'usb_console_test_backend_t'

negative-usb-tx-result-type-seam:
	bash tests/host/run_negative_compile.sh tests/host/probes/usb_tx_result_type_hidden_without_host_macro.cpp 'not declared|has not been declared|does not name a type|is not a member of|no member named' 'usb_console_tx_result_t'

negative-hello-dependencies-seam:
	bash tests/host/run_negative_compile.sh tests/host/probes/hello_host_dependencies_hidden_without_host_macro.cpp 'not declared|has not been declared|does not name a type|is not a member of|no member named' 'HostDependencies'

negative-hello-run-for-test-seam:
	bash tests/host/run_negative_compile.sh tests/host/probes/hello_run_for_test_hidden_without_host_macro.cpp 'not declared|has not been declared|does not name a type|is not a member of|no member named' 'RunForTest'

negative-boot-dependencies-seam:
	bash tests/host/run_negative_compile.sh tests/host/probes/boot_host_dependencies_hidden_without_host_macro.cpp 'not declared|has not been declared|does not name a type|is not a member of|no member named' 'HostDependencies'

negative-boot-run-for-test-seam:
	bash tests/host/run_negative_compile.sh tests/host/probes/boot_run_for_test_hidden_without_host_macro.cpp 'not declared|has not been declared|does not name a type|is not a member of|no member named' 'RunForTest'

-include $(DEPFILES)
