#!/usr/bin/env python3
"""Dima 架构门禁共享的路径、分层规则、源码扫描与 Make 解析辅助函数。"""

from __future__ import annotations

import functools
import pathlib
import re
from collections.abc import Iterable


ROOT = pathlib.Path(__file__).resolve().parents[2]
SOURCE_SUFFIXES = {".c", ".h", ".cpp", ".hpp"}
C_COMMENT_RE = re.compile(
    r'("(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\')|'
    r'(/\*.*?\*/|//[^\r\n]*)',
    re.DOTALL,
)
PROTECTED_ROOTS = (
    "Dima/application",
    "Dima/rover",
    "Dima/drivers",
    "Dima/modules",
    "Dima/middleware",
    "Dima/messages",
    "Dima/lib",
    "Dima/adapters",
    "Dima/platform/api",
    "Dima/platform/common",
)
# 这些目录保存从 PX4 v1.17 裁剪出的算法闭包。它们仍属于产品源码并继续接受
# 依赖方向、硬件所有权和生成闭包检查；这里只集中定义上游源码身份，供必须保留
# PX4 原生 include/namespace 形式的规则复用，不能扩大到整个 Dima/lib。
PX4_UPSTREAM_ALGORITHM_ROOTS = (
    "Dima/lib/ekf2",
    "Dima/lib/matrix",
    "Dima/lib/mathlib",
    "Dima/lib/geo",
    "Dima/lib/lat_lon_alt",
    "Dima/lib/world_magnetic_model",
)
# 上游算法只能借用这两个结构化兼容根。参数、生命周期、WorkQueue、uORB Core
# 等其余 middleware 属于产品运行时，不得因“兼容 PX4”而被算法层反向引用。
PX4_MIDDLEWARE_COMPAT_ROOTS = (
    "Dima/middleware/px4_platform_common",
    "Dima/middleware/lib/mathlib",
)
FREERTOS_ROOT = "Dima/platform/freertos"
STM32_ROOT = "Dima/platform/stm32h7"
COMMON_INCLUDE_ROOTS = (
    "Dima/application",
    "Dima/rover",
    "Dima/drivers/gps",
    "Dima/drivers/imu",
    "Dima/drivers/magnetometer",
    "Dima/drivers/rc",
    "Dima/modules",
    "Dima/modules/sensors",
    "Dima/middleware",
    "Dima/middleware/parameters",
    "Dima/messages",
    "Dima/lib",
    "Dima/lib/protocols",
    "Dima/lib/sensors",
    "Dima/adapters",
    "Dima/platform",
    "Dima/platform/stm32h7",
)
LAYER_ROOTS = (
    ("platform/freertos", ROOT / "Dima/platform/freertos"),
    ("platform/stm32h7", ROOT / "Dima/platform/stm32h7"),
    ("platform/common", ROOT / "Dima/platform/common"),
    ("platform/api", ROOT / "Dima/platform/api"),
    ("application", ROOT / "Dima/application"),
    ("rover", ROOT / "Dima/rover"),
    ("drivers", ROOT / "Dima/drivers"),
    ("modules", ROOT / "Dima/modules"),
    ("middleware", ROOT / "Dima/middleware"),
    ("messages", ROOT / "Dima/messages"),
    ("lib", ROOT / "Dima/lib"),
    ("adapters", ROOT / "Dima/adapters"),
)
ALLOWED_LAYER_DEPENDENCIES = {
    # PX4 算法闭包只能依赖同层 lib、薄 middleware 兼容头和平台 API；即使其
    # 上游源码风格被保留，也不能反向访问 drivers/modules/rover/application。
    "px4-upstream-algorithm": {"lib", "middleware", "platform/api"},
    "platform/freertos": {"platform/freertos", "platform/api"},
    "platform/stm32h7": {"platform/stm32h7", "platform/api"},
    "platform/common": {"platform/common", "platform/api"},
    "platform/api": {"platform/api"},
    "lib": {"lib", "platform/api"},
    "middleware": {"middleware", "lib", "platform/api"},
    "messages": {"messages", "middleware", "lib", "platform/api"},
    "modules": {
        "modules", "messages", "middleware", "lib", "adapters",
        "platform/api",
    },
    "drivers": {
        "drivers", "messages", "middleware", "lib", "platform/api",
    },
    "adapters": {"adapters", "platform/api"},
    "rover": {
        "rover", "drivers", "modules", "messages", "middleware", "lib",
        "adapters", "platform/api",
    },
    "application": {
        "application", "rover", "drivers", "modules", "messages",
        "middleware", "lib", "adapters", "platform/api",
    },
}

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')
PROTECTED_API_RE = re.compile(
    r"\b(?:xTask\w*|vTask\w*|uxTask\w*|xSemaphore\w*|vSemaphore\w*|"
    r"task(?:ENTER|EXIT|YIELD)\w*|portMAX_DELAY|TickType_t|TaskHandle_t|"
    r"SemaphoreHandle_t|StaticTask_t|StaticSemaphore_t|BaseType_t|"
    r"UBaseType_t|osKernel\w*|HAL_\w+|__HAL_\w+|LL_\w+|NVIC_\w+|"
    r"__disable_irq|__enable_irq|__get_\w+|irqstate_t|"
    r"enter_critical_section|leave_critical_section)\b|\bSCB(?:->|_)\w+"
)
FREERTOS_VENDOR_API_RE = re.compile(
    r"\b(?:HAL_\w+|__HAL_\w+|LL_\w+|NVIC_\w+|SystemCoreClock)\b|"
    r"\b(?:SCB|RCC|FLASH|MPU)(?:->|_)\w+"
)
RTOS_API_RE = re.compile(
    r"\b(?:xTask\w*|vTask\w*|uxTask\w*|xSemaphore\w*|vSemaphore\w*|"
    r"task(?:ENTER|EXIT|YIELD)\w*|portMAX_DELAY|TickType_t|TaskHandle_t|"
    r"SemaphoreHandle_t|StaticTask_t|StaticSemaphore_t|BaseType_t|"
    r"UBaseType_t|osKernel\w*)\b"
)
CACHE_OPERATION_RE = re.compile(
    r"\bSCB_(?:Enable|Disable|Clean|Invalidate|CleanInvalidate)"
    r"\w*Cache\w*\s*\("
)
HAL_DMA_RE = re.compile(r"\bHAL_\w*DMA\w*\s*\(")
HAL_FLASH_RE = re.compile(r"\b(?:HAL_FLASH\w*|__HAL_FLASH\w*)\s*\(")
NONZERO_PWM_PULSE_RE = re.compile(
    r"^TIM(?:5|8)\.Pulse-PWM.*=(?!0$)(.+)$"
)

BOARD_HEADERS = {
    "board_bus_resources.h",
    "board_init.h",
    "boot_diagnostics.h",
    "boot_diagnostics_store.h",
    "boot_layout.h",
    "dima_boot_request.h",
    "motor_pwm.h",
    "platform_composition.h",
}
CORE_GENERATED_HEADERS = {
    "dma.h",
    "fdcan.h",
    "gpio.h",
    "i2c.h",
    "main.h",
    "sdmmc.h",
    "spi.h",
    "tim.h",
    "usart.h",
}
FATFS_HEADERS = {
    "diskio.h",
    "ff.h",
    "ffconf.h",
    "integer.h",
}
MAKE_CONTRACT_PATHS = (
    ROOT / "make/project.mk",
    ROOT / "make/release.mk",
)


class Violation:
    def __init__(self, path: pathlib.Path, line: int, rule: str,
                 message: str) -> None:
        self.path = path
        self.line = line
        self.rule = rule
        self.message = message

    def render(self) -> str:
        relative = self.path.relative_to(ROOT).as_posix()
        return f"{relative}:{self.line}: {self.rule} {self.message}"


def strip_c_comments(text: str) -> str:
    """删除 C/C++ 注释，但保留字符串内容和换行数，保证违规行号仍可追溯。"""
    def replace(match: re.Match[str]) -> str:
        if match.group(1) is not None:
            return match.group(1)
        return "\n" * match.group(2).count("\n")

    return C_COMMENT_RE.sub(replace, text)


def strip_cpp_structure(text: str) -> list[str]:
    """删除注释与字面量，并保持输入输出逐行对应，供结构正则避免误匹配文本。"""
    literal_re = re.compile(
        r'"(?:\\.|[^"\\\r\n])*"|\'(?:\\.|[^\'\\\r\n])*\''
    )
    text = literal_re.sub('""', text)
    text = re.sub(
        r"/\*.*?\*/",
        lambda match: "\n" * match.group(0).count("\n"),
        text,
        flags=re.DOTALL,
    )
    output: list[str] = []
    for line in text.splitlines():
        output.append(re.sub(r"//.*$", "", line))
    return output


@functools.lru_cache(maxsize=None)
def source_files_under(relative: str) -> tuple[pathlib.Path, ...]:
    """缓存一个产品目录下的第一方 C/C++ 文件集合，排除原样 vendored 代码。"""
    base = ROOT / relative
    if not base.exists():
        return ()
    return tuple(
        sorted(
            path
            for path in base.rglob("*")
            if path.is_file()
            and path.suffix.lower() in SOURCE_SUFFIXES
            and not is_vendored(path)
        )
    )


def sources_under(relative_roots: Iterable[str]) -> list[pathlib.Path]:
    files = {
        path
        for relative in relative_roots
        for path in source_files_under(relative)
    }
    return sorted(files)


def is_vendored(path: pathlib.Path) -> bool:
    """True for upstream-generated sources kept verbatim (c_library_v2)."""
    try:
        relative = path.relative_to(ROOT)
    except ValueError:
        return False
    return "c_library_v2" in relative.parts


def is_px4_upstream_algorithm(path: pathlib.Path) -> bool:
    """按六个受控目录识别 PX4 算法源码，不把其他 Dima/lib 文件一并豁免。"""
    try:
        relative = path.resolve().relative_to(ROOT)
    except ValueError:
        return False
    return any(
        relative.is_relative_to(pathlib.Path(root))
        for root in PX4_UPSTREAM_ALGORITHM_ROOTS
    )


def first_party_sources() -> list[pathlib.Path]:
    roots = ("Dima", "Boards", "Core", "Bootloader", "USB_DEVICE")
    return sources_under(roots)


def fatfs_include(include: str) -> bool:
    lowered = include.lower()
    basename = pathlib.PurePosixPath(lowered).name
    return (
        basename in FATFS_HEADERS
        or lowered.startswith("middlewares/third_party/fatfs/")
    )


def low_level_include(include: str) -> bool:
    """按头文件身份识别 RTOS/HAL/板级依赖，不依赖调用方写出的具体相对路径。"""
    lowered = include.lower()
    basename = pathlib.PurePosixPath(lowered).name
    return (
        "freertos" in lowered
        or "nuttx" in lowered
        or "cmsis" in lowered
        or "stm32" in lowered
        or lowered.startswith(("core/", "boards/", "usb_device/",
                               "middlewares/"))
        or lowered.startswith(("platform/freertos/",
                               "platform/stm32h7/"))
        or fatfs_include(include)
        or basename in BOARD_HEADERS
        or basename in CORE_GENERATED_HEADERS
        or basename.startswith(("usb_device", "usbd_", "core_cm"))
    )


def vendor_include(include: str) -> bool:
    lowered = include.lower()
    basename = pathlib.PurePosixPath(lowered).name
    return (
        "stm32" in lowered
        or "cmsis" in lowered
        or lowered.startswith(("core/", "boards/", "usb_device/",
                               "drivers/"))
        or lowered.startswith("platform/stm32h7/")
        or basename in BOARD_HEADERS
        or basename in CORE_GENERATED_HEADERS
        or basename.startswith(("usb_device", "usbd_", "core_cm"))
    )


def freertos_include(include: str) -> bool:
    lowered = include.lower()
    basename = pathlib.PurePosixPath(lowered).name
    return (
        "freertos" in lowered
        or lowered.startswith("platform/freertos/")
        or basename in {
            "event_groups.h", "portable.h", "queue.h", "semphr.h",
            "stream_buffer.h", "task.h", "timers.h",
        }
        or basename.startswith("cmsis_os")
    )


def protected_layer(path: pathlib.Path) -> str | None:
    for name, root in LAYER_ROOTS:
        if path.is_relative_to(root):
            return name
    return None


def is_px4_middleware_compat(path: pathlib.Path) -> bool:
    relative = path.resolve().relative_to(ROOT).as_posix()
    return any(
        relative == root or relative.startswith(root + "/")
        for root in PX4_MIDDLEWARE_COMPAT_ROOTS
    )


def resolve_common_include(source: pathlib.Path,
                           include: str) -> pathlib.Path | None:
    candidates = [source.parent / include]
    candidates.extend(ROOT / root / include for root in COMMON_INCLUDE_ROOTS)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None

def effective_variable_blocks(
        lines: list[str], name: str) -> list[list[tuple[int, str]]]:
    """按 Make 的覆盖、追加和条件赋值顺序计算变量的有效定义块。"""
    start_re = re.compile(
        rf"^\s*(?:(?:override|export|private)\s+)*"
        rf"{re.escape(name)}\s*(?P<operator>:=|\+=|\?=|=)"
    )
    effective: list[list[tuple[int, str]]] = []
    defined = False
    for index, line in enumerate(lines):
        match = start_re.match(line)
        if match is None:
            continue
        block = [(index + 1, line)]
        while block[-1][1].rstrip().endswith("\\"):
            next_index = index + len(block)
            if next_index >= len(lines):
                break
            block.append((next_index + 1, lines[next_index]))
        operator = match.group("operator")
        if operator == "+=":
            effective.append(block)
            defined = True
        elif operator == "?=":
            if not defined:
                effective = [block]
                defined = True
        else:
            effective = [block]
            defined = True
    return effective


MAKE_VARIABLE_REFERENCE_RE = re.compile(
    r"\$\(([A-Za-z_][A-Za-z0-9_]*)\)|"
    r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}"
)


def transitive_variable_block(lines: list[str],
                              name: str) -> list[tuple[int, str]]:
    """递归展开 Make 变量引用，返回源码清单真正可达的定义块并阻止重复遍历。"""
    output: list[tuple[int, str]] = []
    visited: set[str] = set()

    def visit(variable: str) -> None:
        if variable in visited:
            return
        visited.add(variable)
        for block in effective_variable_blocks(lines, variable):
            output.extend(block)
            for _line_number, text in block:
                for match in MAKE_VARIABLE_REFERENCE_RE.finditer(text):
                    dependency = match.group(1) or match.group(2)
                    visit(dependency)

    visit(name)
    return output


def line_for(text: str, needle: str) -> int:
    offset = text.find(needle)
    return 1 if offset < 0 else text.count("\n", 0, offset) + 1


@functools.lru_cache(maxsize=None)
def repository_files_named(filename: str) -> tuple[pathlib.Path, ...]:
    """按文件身份搜索源码树；目录位置不属于内容合同。"""
    matches: list[pathlib.Path] = []
    skipped_roots = {".git", ".keys", ".vscode", "build"}
    for child in ROOT.iterdir():
        if child.name in skipped_roots or child.name.startswith("build-"):
            continue
        if child.is_file():
            if child.name == filename:
                matches.append(child)
            continue
        if child.is_dir():
            matches.extend(path for path in child.rglob(filename) if path.is_file())
    return tuple(sorted(set(matches)))


def resolve_architecture_source(
        requested: pathlib.Path,
        identity_literals: Iterable[str] = (),
) -> pathlib.Path | None:
    """优先使用当前路径；移动后按唯一文件名和内容身份重新定位。"""
    if requested.is_file():
        return requested
    candidates = repository_files_named(requested.name)
    literals = tuple(identity_literals)
    if literals:
        matching: list[pathlib.Path] = []
        for candidate in candidates:
            try:
                text = candidate.read_text(encoding="utf-8")
            except (OSError, UnicodeError):
                continue
            if all(literal in text for literal in literals):
                matching.append(candidate)
        if len(matching) == 1:
            return matching[0]
    return candidates[0] if len(candidates) == 1 else None


def require_literals(path: pathlib.Path,
                     requirements: Iterable[tuple[str, str, str]],
                     violations: list[Violation]) -> None:
    """按唯一文件名和内容身份定位 owner，再核对合同字面量而不锁死原目录位置。"""
    required = tuple(requirements)
    resolved = resolve_architecture_source(
        path, (literal for literal, _rule, _message in required)
    )
    if resolved is None:
        violations.append(Violation(
            path, 1, "R040",
            "required architecture source identity is missing or ambiguous",
        ))
        return
    text = resolved.read_text(encoding="utf-8")
    for literal, rule, message in required:
        if literal not in text:
            violations.append(Violation(resolved, 1, rule, message))


def owner_texts(
        paths: Iterable[pathlib.Path]) -> tuple[tuple[pathlib.Path, str], ...]:
    """读取显式 owner 白名单中当前存在的文件；白名单本身仍是安全边界。"""
    return tuple(
        (path, path.read_text(encoding="utf-8"))
        for path in paths
        if path.is_file()
    )


def find_literal_owner(
        paths: Iterable[pathlib.Path],
        literal: str) -> tuple[pathlib.Path, str] | None:
    """在 owner 白名单内寻找合同字面量，不允许白名单外文件取得该能力。"""
    for path, text in owner_texts(paths):
        if literal in text:
            return path, text
    return None


def require_literals_in_owners(
        paths: tuple[pathlib.Path, ...],
        requirements: Iterable[tuple[str, str, str]],
        violations: list[Violation]) -> None:
    """要求每项合同至少由一个白名单 owner 实现，允许同一职责在白名单内拆文件。"""
    existing = owner_texts(paths)
    if not existing:
        require_literals(paths[0], requirements, violations)
        return
    missing = tuple(
        (literal, rule, message)
        for literal, rule, message in requirements
        if not any(literal in text for _path, text in existing)
    )
    if missing:
        require_literals(existing[0][0], missing, violations)


def require_make_source_paths(
        path: pathlib.Path,
        requirements: Iterable[tuple[str, str, str]],
        violations: list[Violation]) -> None:
    """要求源码从 PROJECT_C/CXX_SOURCES 的传递变量闭包可达，不能只散落在文件中。"""
    if not path.is_file():
        require_literals(path, requirements, violations)
        return
    lines = path.read_text(encoding="utf-8").splitlines()
    reachable = transitive_variable_block(lines, "PROJECT_C_SOURCES")
    reachable.extend(
        transitive_variable_block(lines, "PROJECT_CXX_SOURCES")
    )
    for source, rule, message in requirements:
        source_pattern = re.compile(
            rf"(?<![A-Za-z0-9_.-]){re.escape(source)}(?=\s|\\|$)"
        )
        if not any(source_pattern.search(text) for _line, text in reachable):
            violations.append(Violation(path, 1, rule, message))
