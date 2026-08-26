#!/usr/bin/env python3
"""从真实 GNU Make recipes 生成 Windows 主机原生 compilation database。"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from collections import Counter
from collections.abc import Iterator, Sequence

from architecture.build_closure import BuildClosureError, load_build_closure

SOURCE_SUFFIXES = frozenset({".c", ".cc", ".cpp", ".cxx", ".s"})
COMPILER_PATTERN = re.compile(r"arm-none-eabi-(gcc|g\+\+)(?:\.exe)?$")


class DatabaseError(RuntimeError):
    """Make 计划无法无损转换为可信 compilation database。"""


def logical_lines(text: str) -> Iterator[str]:
    """合并 Make recipe 续行但不改变 shell 引号语义。"""
    parts: list[str] = []
    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        if line.endswith("\\"):
            parts.append(line[:-1].strip())
            continue
        parts.append(line.strip())
        yield " ".join(part for part in parts if part)
        parts.clear()
    if parts:
        yield " ".join(part for part in parts if part)


def compiler_kind(token: str) -> str | None:
    name = pathlib.PurePosixPath(token.replace("\\", "/")).name
    match = COMPILER_PATTERN.fullmatch(name)
    return match.group(1) if match is not None else None


def host_compiler(gcc_path: pathlib.Path, kind: str) -> str:
    suffix = ".exe" if os.name == "nt" else ""
    executable = gcc_path / f"arm-none-eabi-{kind}{suffix}"
    if not executable.is_file():
        raise DatabaseError(f"compiler does not exist: {executable}")
    return str(executable.resolve())


def find_gcc_path(value: str | None) -> pathlib.Path:
    if value:
        candidate = pathlib.Path(value.replace("\\", "/")).expanduser()
    else:
        executable = shutil.which("arm-none-eabi-gcc")
        if executable is None:
            raise DatabaseError(
                "GCC_PATH is empty and arm-none-eabi-gcc is not on PATH"
            )
        candidate = pathlib.Path(executable).resolve().parent

    suffix = ".exe" if os.name == "nt" else ""
    compiler = candidate / f"arm-none-eabi-gcc{suffix}"
    if not compiler.is_file():
        raise DatabaseError(f"Arm GCC bin directory is invalid: {candidate}")
    return candidate.resolve()


def make_plan(
    root: pathlib.Path,
    make_program: str,
    gcc_path: pathlib.Path,
    make_variables: Sequence[str],
) -> str:
    # Windows Store execution alias 可能触发 WinError 1920；dry-run 直接复用当前解释器。
    python = pathlib.Path(sys.executable).as_posix()
    command = [
        make_program,
        "--no-print-directory",
        "-B",
        "-n",
        "-f",
        "GNUmakefile",
        "OS=Windows_NT",
        "DIMA_BUILD_INTERNAL=1",
        "DIMA_PROGRESS_STATE=",
        f"PYTHON={python}",
        f"GCC_PATH={gcc_path.as_posix()}",
        *make_variables,
        "firmware",
    ]
    environment = os.environ.copy()
    # 生成器通常由 Make 调用；清除外层 dry-run/jobserver/递归状态，避免污染只读子 Make。
    for name in ("MAKEFLAGS", "MFLAGS", "MAKELEVEL"):
        environment.pop(name, None)

    try:
        completed = subprocess.run(
            command,
            cwd=root,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            encoding="utf-8",
            errors="replace",
        )
    except OSError as error:
        raise DatabaseError(f"cannot execute {make_program!r}: {error}") from error
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise DatabaseError(
            f"Make dry-run failed with exit code {completed.returncode}:\n{detail}"
        )
    return completed.stdout


def source_and_output(tokens: Sequence[str]) -> tuple[str, str | None] | None:
    source: str | None = None
    output: str | None = None
    for index, token in enumerate(tokens):
        if token == "-o" and index + 1 < len(tokens):
            output = tokens[index + 1]
        if token.startswith("-"):
            continue
        if pathlib.PurePosixPath(token.replace("\\", "/")).suffix.lower() in SOURCE_SUFFIXES:
            source = token
    if source is None:
        return None
    return source, output


def absolute_path(root: pathlib.Path, value: str) -> pathlib.Path:
    path = pathlib.Path(value)
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def parse_commands(
    plan: str, root: pathlib.Path, gcc_path: pathlib.Path
) -> list[dict[str, object]]:
    """只保留 Arm 编译命令并替换为主机可执行编译器，拒绝丢失 source/output。"""
    entries: list[dict[str, object]] = []
    arm_compile_commands = 0
    for line in logical_lines(plan):
        try:
            tokens = shlex.split(line, posix=True)
        except ValueError as error:
            raise DatabaseError(f"cannot parse Make command: {line}") from error
        if not tokens or "-c" not in tokens:
            continue
        kind = compiler_kind(tokens[0])
        if kind is None:
            continue
        arm_compile_commands += 1
        paths = source_and_output(tokens)
        if paths is None:
            raise DatabaseError(
                f"Arm compile command has no recognized source file: {line}"
            )
        source_value, output_value = paths
        if output_value is None:
            raise DatabaseError(f"Arm compile command has no output: {line}")
        source = absolute_path(root, source_value)
        try:
            relative_source = source.relative_to(root)
        except ValueError as error:
            raise DatabaseError(f"source is outside the workspace: {source}") from error
        if not source.is_file() and relative_source.parts[:1] != ("build",):
            raise DatabaseError(f"Make references a missing source: {relative_source}")

        arguments = list(tokens)
        arguments[0] = host_compiler(gcc_path, kind)
        entry: dict[str, object] = {
            "directory": str(root),
            "file": str(source),
            "arguments": arguments,
        }
        entry["output"] = str(absolute_path(root, output_value))
        entries.append(entry)

    if not entries:
        raise DatabaseError("Make dry-run contained no compile commands")
    if len(entries) != arm_compile_commands:
        raise DatabaseError(
            "not every Arm compile command was preserved in the database"
        )
    return entries


def relative_file(entry: dict[str, object], root: pathlib.Path) -> str:
    path = pathlib.Path(str(entry["file"]))
    return path.relative_to(root).as_posix()


def include_paths(entry: dict[str, object]) -> set[str]:
    return set(include_path_list(entry))


def include_path_list(entry: dict[str, object]) -> list[str]:
    arguments = entry["arguments"]
    assert isinstance(arguments, list)
    includes: list[str] = []
    for index, value in enumerate(arguments):
        token = str(value)
        if token == "-I" and index + 1 < len(arguments):
            includes.append(str(arguments[index + 1]).replace("\\", "/"))
        elif token.startswith("-I") and len(token) > 2:
            includes.append(token[2:].replace("\\", "/"))
    return includes


def validate_closure(
    entries: Sequence[dict[str, object]],
    root: pathlib.Path,
    make_program: str,
) -> None:
    """将数据库 source/output 双向比对求值 Make 闭包，并拒绝重复 include 与对象碰撞。"""
    pairs: list[tuple[str, str]] = []
    outputs: list[str] = []
    for entry in entries:
        output = pathlib.Path(str(entry["output"]))
        try:
            relative_output = output.relative_to(root).as_posix()
        except ValueError as error:
            raise DatabaseError(
                f"compile output is outside the workspace: {output}"
            ) from error
        pair = (relative_file(entry, root), relative_output)
        pairs.append(pair)
        outputs.append(relative_output)

        includes = include_path_list(entry)
        duplicate_includes = sorted(
            include for include, count in Counter(includes).items() if count > 1
        )
        if duplicate_includes:
            raise DatabaseError(
                f"duplicate include roots for {pair[0]}: "
                f"{', '.join(duplicate_includes)}"
            )

    duplicate_pairs = sorted(
        pair for pair, count in Counter(pairs).items() if count > 1
    )
    if duplicate_pairs:
        raise DatabaseError(
            f"duplicate source/output compile contexts: {duplicate_pairs}"
        )
    duplicate_outputs = sorted(
        output for output, count in Counter(outputs).items() if count > 1
    )
    if duplicate_outputs:
        raise DatabaseError(f"duplicate compile outputs: {duplicate_outputs}")

    try:
        closure = load_build_closure(root, make_program)
    except BuildClosureError as error:
        raise DatabaseError(f"cannot verify Make build closure: {error}") from error
    actual = set(pairs)
    expected = set(closure.source_outputs)
    if actual != expected or len(pairs) != len(closure.units):
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise DatabaseError(
            "compilation database differs from the evaluated Make closure: "
            f"missing={missing}, unexpected={unexpected}"
        )


def validate_contexts(entries: Sequence[dict[str, object]], root: pathlib.Path) -> None:
    """抽查关键层的必需/禁止 include 根，证明 target-private 上下文没有泄漏。"""
    by_source: dict[str, list[dict[str, object]]] = {}
    for entry in entries:
        by_source.setdefault(relative_file(entry, root), []).append(entry)

    contexts = {
        "Dima/application/app_main.cpp": (
            {
                "Dima/rover",
                "Dima/drivers/gps",
                "Dima/modules",
                "Dima/middleware",
                "Dima/lib",
                "Dima/platform",
                "build/generated_include",
                "build/generated/messages",
            },
            {
                "Dima",
                "Middlewares/Third_Party/FreeRTOS/Source/include",
                "Core/Inc",
            },
        ),
        "Dima/platform/freertos/Backend.cpp": (
            {
                "Dima/platform",
                "Dima/platform/freertos",
                "Middlewares/Third_Party/FreeRTOS/Source/include",
            },
            {"Dima", "Core/Inc", "Drivers/STM32H7xx_HAL_Driver/Inc"},
        ),
        "Dima/platform/stm32h7/flash/FlashDevice.cpp": (
            {
                "Dima/platform",
                "Dima/platform/stm32h7",
                "Core/Inc",
                "Drivers/STM32H7xx_HAL_Driver/Inc",
            },
            {"Dima", "Middlewares/Third_Party/FreeRTOS/Source/include"},
        ),
        "Boards/H743/Src/platform_composition.cpp": (
            {
                "Dima/adapters",
                "Dima/platform",
                "Dima/platform/stm32h7",
                "Drivers/STM32H7xx_HAL_Driver/Inc",
            },
            {"Dima", "Middlewares/Third_Party/FreeRTOS/Source/include"},
        ),
        "Boards/H743/Src/boot_diagnostics.c": (
            {
                "Dima/platform",
                "Dima/platform/freertos",
                "Middlewares/Third_Party/FreeRTOS/Source/include",
            },
            {"Dima"},
        ),
        "USB_DEVICE/App/usbd_cdc_if.c": (
            {"Dima/adapters", "Dima/platform", "Dima/platform/stm32h7"},
            {"Dima", "Dima/modules", "build/generated/messages"},
        ),
        "build/generated/messages/uorb_topics.cpp": (
            {
                "Dima/middleware",
                "Dima/platform",
                "build/generated_include",
                "build/generated/messages",
            },
            {
                "Dima",
                "Core/Inc",
                "Middlewares/Third_Party/FreeRTOS/Source/include",
            },
        ),
    }
    for source, (required, forbidden) in contexts.items():
        candidates = by_source.get(source)
        if not candidates:
            raise DatabaseError(f"compilation database does not cover {source}")
        includes = include_paths(candidates[0])
        missing = sorted(required - includes)
        unexpected = sorted(forbidden & includes)
        if missing or unexpected:
            details: list[str] = []
            if missing:
                details.append("missing " + ", ".join(missing))
            if unexpected:
                details.append("unexpected " + ", ".join(unexpected))
            raise DatabaseError(f"invalid compile context for {source}: {'; '.join(details)}")


def write_database(path: pathlib.Path, entries: Sequence[dict[str, object]]) -> None:
    """同目录写临时 JSON、fsync 后原子替换，避免 clangd 读取半写数据库。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=path.name + ".",
            suffix=".tmp",
            dir=path.parent,
            delete=False,
        ) as output:
            temporary_name = output.name
            json.dump(entries, output, indent=2, ensure_ascii=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, path)
    finally:
        if temporary_name is not None:
            pathlib.Path(temporary_name).unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path, default="compile_commands.json")
    parser.add_argument("--gcc-path")
    parser.add_argument("--make", default=os.environ.get("MAKE", "make"))
    parser.add_argument(
        "--make-variable",
        action="append",
        default=[],
        metavar="NAME=VALUE",
    )
    arguments = parser.parse_args()

    root = pathlib.Path(__file__).resolve().parent.parent
    output = arguments.output
    if not output.is_absolute():
        output = root / output
    output = output.resolve()
    gcc_path = find_gcc_path(arguments.gcc_path)
    plan = make_plan(root, arguments.make, gcc_path, arguments.make_variable)
    entries = parse_commands(plan, root, gcc_path)
    validate_closure(entries, root, arguments.make)
    validate_contexts(entries, root)
    write_database(output, entries)

    unique_sources = len({str(entry["file"]) for entry in entries})
    print(
        f"Generated {len(entries)} compile commands for {unique_sources} sources:\n"
        f"  {output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DatabaseError as error:
        print(f"compile database generation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
