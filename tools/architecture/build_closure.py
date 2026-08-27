"""从 Windows 原生 GNU Make 数据库求值真实 Application/MCUboot 构建闭包。"""

from __future__ import annotations

import functools
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import Literal


VARIABLE_RE = re.compile(
    r"^(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*"
    r"(?P<operator>:=|\+=|\?=|=)\s*(?P<value>.*)$"
)
TARGET_VARIABLE_RE = re.compile(
    r"^(?P<target>[^#:\s][^:]*)\s*:\s*"
    r"(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*"
    r"(?P<operator>:=|\+=|\?=|=)\s*(?P<value>.*)$"
)
VARIABLE_REFERENCE_RE = re.compile(
    r"\$\(([A-Za-z_][A-Za-z0-9_]*)\)|"
    r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}"
)


class BuildClosureError(RuntimeError):
    """The evaluated Make database is missing or internally inconsistent."""


@dataclass(frozen=True)
class CompileUnit:
    owner: Literal["application", "bootloader"]
    source: str
    output: str
    includes: tuple[str, ...]
    project_owned: bool


@dataclass(frozen=True)
class BuildClosure:
    units: tuple[CompileUnit, ...]
    parameter_generator_inputs: frozenset[str]

    @property
    def sources(self) -> frozenset[str]:
        return frozenset(unit.source for unit in self.units)

    @property
    def source_outputs(self) -> frozenset[tuple[str, str]]:
        return frozenset((unit.source, unit.output) for unit in self.units)

@dataclass(frozen=True)
class _MakeDatabase:
    variables: dict[str, str]
    target_variables: dict[tuple[str, str], str]

    def words(self, name: str) -> tuple[str, ...]:
        value = _expand_variables(self.variables.get(name, ""), self.variables)
        try:
            return tuple(shlex.split(value, posix=True))
        except ValueError as error:
            raise BuildClosureError(
                f"cannot parse evaluated Make variable {name}: {value}"
            ) from error

    def target_value(self, target: str, name: str) -> str | None:
        value = self.target_variables.get((target, name))
        if value is None:
            return None
        return _expand_variables(value, self.variables)


def _normalize_path(value: str) -> str:
    normalized = pathlib.PurePosixPath(value.replace("\\", "/")).as_posix()
    while normalized.startswith("./"):
        normalized = normalized[2:]
    return normalized


def _expand_variables(
    value: str,
    variables: dict[str, str],
    active: tuple[str, ...] = (),
) -> str:
    """递归展开简单 Make 变量；遇到循环引用立即报错，避免得到截断闭包。"""
    def replace(match: re.Match[str]) -> str:
        name = match.group(1) or match.group(2)
        if name in active:
            raise BuildClosureError(
                f"recursive Make variable reference while expanding {name}"
            )
        nested = variables.get(name, "")
        return _expand_variables(nested, variables, (*active, name))

    return VARIABLE_REFERENCE_RE.sub(replace, value)


def _parse_make_database(text: str) -> _MakeDatabase:
    variables: dict[str, str] = {}
    target_variables: dict[tuple[str, str], str] = {}
    for line in text.splitlines():
        target_match = TARGET_VARIABLE_RE.match(line)
        if target_match is not None:
            target = _normalize_path(target_match.group("target").strip())
            name = target_match.group("name")
            target_variables[(target, name)] = target_match.group("value")
            continue
        match = VARIABLE_RE.match(line)
        if match is not None:
            variables[match.group("name")] = match.group("value")
    return _MakeDatabase(variables, target_variables)


def _make_program(explicit: str | None) -> str:
    if os.name != "nt":
        raise BuildClosureError(
            "build closure evaluation requires Windows-native Python and Make"
        )
    candidate = explicit or os.environ.get("MAKE") or "make"
    resolved = shutil.which(candidate)
    if resolved is None:
        raise BuildClosureError(f"cannot find GNU Make executable {candidate!r}")
    return str(pathlib.Path(resolved).resolve())


def _run_make_database(
    root: pathlib.Path,
    make_program: str,
    arguments: tuple[str, ...],
) -> _MakeDatabase:
    """只读执行 ``make -pn``，固定 Windows 上下文并清除父 Make 递归环境。"""
    command = [
        make_program,
        "--no-print-directory",
        "-pn",
        "OS=Windows_NT",
        "DIMA_PROGRESS_STATE=",
        # Resolving a Windows Store execution alias can raise WinError 1920.
        # The already-running interpreter path is sufficient for Make's
        # read-only database expansion and avoids dereferencing that alias.
        f"PYTHON={pathlib.Path(sys.executable).as_posix()}",
        *arguments,
    ]
    environment = os.environ.copy()
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
        raise BuildClosureError(
            f"cannot evaluate Make build closure: {error}"
        ) from error
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise BuildClosureError(
            f"Make database failed with exit code {completed.returncode}:\n{detail}"
        )
    return _parse_make_database(completed.stdout)


def _includes(value: str) -> tuple[str, ...]:
    """解析 Make 已求值的 -I 参数，兼容分离/紧邻写法并拒绝悬空的 -I。"""
    try:
        tokens = shlex.split(value, posix=True)
    except ValueError as error:
        raise BuildClosureError(f"cannot parse include context: {value}") from error
    includes: list[str] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if token == "-I":
            index += 1
            if index >= len(tokens):
                raise BuildClosureError("include context ends with a bare -I")
            includes.append(_normalize_path(tokens[index]))
        elif token.startswith("-I") and len(token) > 2:
            includes.append(_normalize_path(token[2:]))
        index += 1
    return tuple(includes)


def _object_path(build_dir: str, source: str, keep_directories: bool) -> str:
    """复现 Make 的对象路径规则；第一方源码保留目录以消除同名 basename 冲突。"""
    source_path = pathlib.PurePosixPath(source)
    relative = source_path.with_suffix(".o")
    if not keep_directories:
        relative = pathlib.PurePosixPath(relative.name)
    return _normalize_path(f"{build_dir}/{relative.as_posix()}")


def _validate_units(
    owner: str,
    units: tuple[CompileUnit, ...],
    evaluated_objects: tuple[str, ...],
) -> None:
    """双向核对 source->object 映射，拒绝碰撞、重复、缺失或闭包外对象。"""
    by_output: dict[str, list[str]] = {}
    for unit in units:
        by_output.setdefault(unit.output, []).append(unit.source)
    collisions = {
        output: sources for output, sources in by_output.items()
        if len(sources) != 1
    }
    if collisions:
        details = "; ".join(
            f"{output} <- {', '.join(sources)}"
            for output, sources in sorted(collisions.items())
        )
        raise BuildClosureError(f"{owner} object basename collision: {details}")

    expected = [unit.output for unit in units]
    actual = [_normalize_path(output) for output in evaluated_objects]
    if len(actual) != len(set(actual)):
        raise BuildClosureError(f"{owner} OBJECTS contains duplicate outputs")
    if set(actual) != set(expected):
        missing = sorted(set(expected) - set(actual))
        unexpected = sorted(set(actual) - set(expected))
        raise BuildClosureError(
            f"{owner} OBJECTS differs from its source closure: "
            f"missing={missing}, unexpected={unexpected}"
        )


def _application_units(database: _MakeDatabase) -> tuple[CompileUnit, ...]:
    """从应用 Make 数据库派生真实编译单元、对象路径及有效 include 闭包。"""
    build_dir = _normalize_path(database.words("BUILD_DIR")[0])
    cubemx_sources = tuple(
        _normalize_path(source) for source in database.words("C_SOURCES")
    )
    assembly_sources = tuple(
        _normalize_path(source) for source in database.words("ASM_SOURCES")
    )
    project_sources = tuple(
        _normalize_path(source)
        for name in ("PROJECT_C_SOURCES", "PROJECT_CXX_SOURCES")
        for source in database.words(name)
    )
    global_includes = _includes(
        _expand_variables(database.variables.get("C_INCLUDES", ""),
                          database.variables)
    )

    units: list[CompileUnit] = []
    for source in (*cubemx_sources, *assembly_sources):
        output = _object_path(build_dir, source, keep_directories=False)
        target_context = database.target_value(output, "C_INCLUDES")
        extra_includes = _includes(target_context) if target_context else ()
        units.append(CompileUnit(
            owner="application",
            source=source,
            output=output,
            includes=(*global_includes, *extra_includes),
            project_owned=False,
        ))

    for source in project_sources:
        output = _object_path(build_dir, source, keep_directories=True)
        target_context = database.target_value(output, "DIMA_PRIVATE_INCLUDES")
        units.append(CompileUnit(
            owner="application",
            source=source,
            output=output,
            includes=_includes(target_context or ""),
            project_owned=True,
        ))

    result = tuple(units)
    _validate_units("Application", result, database.words("OBJECTS"))
    project_outputs = {
        _normalize_path(output) for output in database.words("PROJECT_OBJECTS")
    }
    expected_project_outputs = {
        unit.output for unit in result if unit.project_owned
    }
    if project_outputs != expected_project_outputs:
        raise BuildClosureError(
            "Application PROJECT_OBJECTS differs from PROJECT_C/CXX_SOURCES"
        )
    return result


def _bootloader_units(database: _MakeDatabase) -> tuple[CompileUnit, ...]:
    """从 MCUboot Make 数据库派生编译单元，并核对 OBJECTS 与源码闭包完全一致。"""
    build_dir = _normalize_path(database.words("BUILD_DIR")[0])
    sources = tuple(
        _normalize_path(source)
        for name in ("C_SOURCES", "ASM_SOURCES")
        for source in database.words(name)
    )
    global_includes = _includes(
        _expand_variables(database.variables.get("C_INCLUDES", ""),
                          database.variables)
    )
    units: list[CompileUnit] = []
    for source in sources:
        output = _normalize_path(
            f"{build_dir}/obj/{pathlib.PurePosixPath(source).stem}.o"
        )
        target_context = database.target_value(output, "C_INCLUDES")
        units.append(CompileUnit(
            owner="bootloader",
            source=source,
            output=output,
            includes=(
                _includes(target_context)
                if target_context is not None
                else global_includes
            ),
            project_owned=False,
        ))
    result = tuple(units)
    _validate_units("MCUboot", result, database.words("OBJECTS"))
    return result


def _evaluate_build_closure(
    root: pathlib.Path,
    make_program: str,
) -> BuildClosure:
    """分别求值应用与 MCUboot 目标，再合并编译单元及参数生成器权威输入。"""
    application = _run_make_database(
        root,
        make_program,
        ("-f", "GNUmakefile", "DIMA_BUILD_INTERNAL=1", "app-check"),
    )
    bootloader = _run_make_database(
        root,
        make_program,
        (
            "-f", "Bootloader/Makefile",
            "KEY_IDENTITY_CHECKED_BY_PARENT=1",
            "KEY_IDENTITY_WILL_CHANGE=0",
            "all",
        ),
    )
    parameter_inputs = frozenset(
        _normalize_path(source)
        for source in application.words("PARAMETER_YAML_DEFINITIONS")
    )
    return BuildClosure(
        units=(*_application_units(application), *_bootloader_units(bootloader)),
        parameter_generator_inputs=parameter_inputs,
    )


@functools.cache
def _cached_build_closure(
    root: pathlib.Path,
    make_program: str,
) -> tuple[BuildClosure | None, str | None]:
    try:
        return _evaluate_build_closure(root, make_program), None
    except BuildClosureError as error:
        return None, str(error)


def load_build_closure(
    root: pathlib.Path,
    make_program: str | None = None,
) -> BuildClosure:
    """Return one cached, evaluated build closure for the repository."""
    resolved_root = root.resolve()
    closure, error = _cached_build_closure(
        resolved_root, _make_program(make_program)
    )
    if closure is None:
        raise BuildClosureError(error or "unknown build closure failure")
    return closure
