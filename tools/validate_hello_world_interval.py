#!/usr/bin/env python3
"""Validate the unexpanded HelloWorld interval passed through the environment."""

from __future__ import annotations

import os
import re
import sys


ENVIRONMENT_VARIABLE = "APP_HELLO_WORLD_INTERVAL_RAW"
MAXIMUM_INTERVAL = "4294967"


def main() -> int:
    raw_value = os.environ.get(ENVIRONMENT_VARIABLE, "")
    if re.fullmatch(r"[0-9]+", raw_value, flags=re.ASCII) is None:
        return 1

    significant_digits = raw_value.lstrip("0")
    if not significant_digits:
        return 1
    if len(significant_digits) > len(MAXIMUM_INTERVAL):
        return 1
    if (len(significant_digits) == len(MAXIMUM_INTERVAL)
            and significant_digits > MAXIMUM_INTERVAL):
        return 1

    sys.stdout.write(significant_digits)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
