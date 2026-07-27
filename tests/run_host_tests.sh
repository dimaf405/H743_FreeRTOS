#!/usr/bin/env bash
# One deterministic entry point.  It deliberately runs both checks so a RED
# baseline reports every currently missing architecture boundary in one pass.
set -u -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATUS=0

BUILD_OK=0
if make -C "$ROOT" -f tests/host/Makefile all; then
    BUILD_OK=1
else
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-messaging; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-usb-set-backend-seam; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-usb-backend-type-seam; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-usb-tx-result-type-seam; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile usb-console-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile usb-console-freertos-backend-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile usb-cdc-glue-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile no-heap-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile speed-to-pwm-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile hello-world-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile boot-health-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-hello-dependencies-seam; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-hello-run-for-test-seam; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-boot-dependencies-seam; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile negative-boot-run-for-test-seam; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile freertos-backend-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile app-bootstrap-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile regenerated-config-test; then
    STATUS=1
fi

if ! make -C "$ROOT" -f tests/host/Makefile board-init-test; then
    STATUS=1
fi

if [[ $BUILD_OK -eq 1 && -x "$ROOT/build/host-tests/host_tests" ]]; then
    if ! "$ROOT/build/host-tests/host_tests"; then
        STATUS=1
    fi
fi

if ! python3 "$ROOT/tests/structure/check_architecture.py"; then
    STATUS=1
fi

exit "$STATUS"
