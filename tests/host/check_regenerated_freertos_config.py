#!/usr/bin/env python3
"""Compile app bootstrap against a CubeMX-regenerated FreeRTOSConfig fixture."""

from __future__ import annotations

import os
import pathlib
import re
import shutil
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[2]
PRODUCTION_CONFIG = ROOT / "Core/Inc/FreeRTOSConfig.h"
BASELINE_CONFIG = ROOT / "tests/host/fixtures/FreeRTOSConfig.cubemx-regenerated.h"
BUILD_DIR = ROOT / "build/host-regenerated-config-test"
GENERATED_CONFIG = BUILD_DIR / "FreeRTOSConfig.h"
TEST_BIN = BUILD_DIR / "regenerated_config_test"


def defines_user_body(source: str) -> str:
    match = re.search(
        r"/\*\s*USER CODE BEGIN Defines\s*\*/(.*?)"
        r"/\*\s*USER CODE END Defines\s*\*/",
        source,
        flags=re.DOTALL,
    )
    if match is None:
        raise RuntimeError("FreeRTOSConfig.h is missing the persisted Defines block")
    return match.group(1)


def inject_defines_user_body(baseline: str, user_body: str) -> str:
    replacement = (
        "/* USER CODE BEGIN Defines */"
        f"{user_body}"
        "/* USER CODE END Defines */"
    )
    generated, count = re.subn(
        r"/\*\s*USER CODE BEGIN Defines\s*\*/.*?"
        r"/\*\s*USER CODE END Defines\s*\*/",
        lambda _: replacement,
        baseline,
        count=1,
        flags=re.DOTALL,
    )
    if count != 1:
        raise RuntimeError("CubeMX baseline fixture has no unique Defines block")
    return generated


def main() -> int:
    production = PRODUCTION_CONFIG.read_text(encoding="utf-8")
    baseline = BASELINE_CONFIG.read_text(encoding="utf-8")
    generated = inject_defines_user_body(baseline, defines_user_body(production))

    shutil.rmtree(BUILD_DIR, ignore_errors=True)
    BUILD_DIR.mkdir(parents=True)
    GENERATED_CONFIG.write_text(generated, encoding="utf-8")

    compiler = os.environ.get("CXX", "g++")
    compile_run = subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{BUILD_DIR}",
            f"-I{ROOT}",
            f"-I{ROOT / 'tests/host/regenerated_config_shim'}",
            str(ROOT / "App/application/app_bootstrap.cpp"),
            str(ROOT / "tests/host/regenerated_config_support.cpp"),
            "-o",
            str(TEST_BIN),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if compile_run.returncode != 0:
        print("regenerated FreeRTOSConfig compile/link gate FAILED", file=sys.stderr)
        print(compile_run.stdout, file=sys.stderr, end="")
        return 1

    run = subprocess.run(
        [str(TEST_BIN)], cwd=ROOT, check=False,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    if run.returncode != 0:
        print("regenerated FreeRTOSConfig runtime gate FAILED", file=sys.stderr)
        print(run.stdout, file=sys.stderr, end="")
        return 1

    print("regenerated FreeRTOSConfig compile/link gate passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
