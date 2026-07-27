#!/usr/bin/env bash
# Expect a probe to fail for the requested contract reason.  A missing public
# architecture header remains a clear RED failure; once it exists the probe's
# own static assertion or hidden-symbol error is required.
set -u -o pipefail

ROOT="."
PROBE="$1"
EXPECTED_PATTERN="$2"
CONTEXT_PATTERN="${3:-}"
LOG="$ROOT/build/host-tests/$(basename "$PROBE").log"
mkdir -p "$ROOT/build/host-tests"

set +e
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I"$ROOT" -UAPP_HOST_TEST \
    -c "$PROBE" -o "$ROOT/build/host-tests/$(basename "$PROBE").o" >"$LOG" 2>&1
RESULT=$?
set -e

if [[ $RESULT -eq 0 ]]; then
    echo "NEGATIVE PROBE ERROR: $PROBE unexpectedly compiled" >&2
    exit 1
fi

if grep -Eiq "$EXPECTED_PATTERN" "$LOG" &&
   { [[ -z "$CONTEXT_PATTERN" ]] || grep -Eiq "$CONTEXT_PATTERN" "$LOG"; }; then
    echo "NEGATIVE PROBE PASS: $PROBE failed for the required contract" >&2
    exit 0
fi

if grep -Eq 'No such file or directory' "$LOG"; then
    echo "NEGATIVE PROBE RED: required production header is not present for $PROBE" >&2
    exit 1
fi

echo "NEGATIVE PROBE ERROR: $PROBE failed, but not for the required contract" >&2
sed -n '1,80p' "$LOG" >&2
exit 1
