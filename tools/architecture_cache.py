#!/usr/bin/env python3
"""计算架构门禁全部输入的内容身份。

架构检查覆盖面很广，不应在未变化镜像的每次上传前重复运行。本模块把受管源码、配置、
目录存在性和符号链接目标纳入指纹，使 Make 仅在输入完全相同时复用成功 stamp；release
目标仍强制重跑，因此缓存只是一项增量构建优化，不能替代最终门禁。
"""

from __future__ import annotations

import argparse
import hashlib
import os
import pathlib
import tempfile


ROOT = pathlib.Path(__file__).resolve().parents[1]
IDENTITY_VERSION = "dima-architecture-input-v1"
INPUT_ROOTS = (
    "Dima",
    "Boards/H743",
    "Core",
    "USB_DEVICE",
    "Bootloader",
    "tests",
    "Linker",
    "make",
    "tools/architecture",
    "tools/dronecan",
    "tools/mavlink",
    "tools/parameters",
    "tools/serial",
    "tools/uorb",
    "Middlewares/Third_Party/dronecan_dsdl/dsdl",
)
EXPLICIT_INPUTS = (
    "GNUmakefile",
    "Makefile",
    "H743_FreeRTOS.ioc",
    "tools/check_architecture.py",
    # 这两个缺失路径仍由生成边界/调试出口合同显式禁止，创建后必须使 stamp 失效。
    "tools/generate_actuators_metadata.py",
    "tools/validate_hello_world_interval.py",
)
IGNORED_DIRECTORY_NAMES = frozenset({
    ".git",
    "__pycache__",
    ".mypy_cache",
    ".pytest_cache",
})


def _tree_entries(root: pathlib.Path, relative_root: str) -> list[pathlib.Path]:
    """稳定枚举文件、目录和符号链接；空目录/README-only 语义也必须进入指纹。"""
    base = root / relative_root
    if not base.exists() and not base.is_symlink():
        return [base]
    if base.is_file() or base.is_symlink():
        return [base]

    entries: list[pathlib.Path] = []
    for current_name, directory_names, file_names in os.walk(
            base, followlinks=False):
        current = pathlib.Path(current_name)
        entries.append(current)
        directory_names[:] = sorted(
            (
                name for name in directory_names
                if name not in IGNORED_DIRECTORY_NAMES
            ),
            key=str.casefold,
        )
        for directory_name in directory_names:
            directory = current / directory_name
            if directory.is_symlink():
                entries.append(directory)
        for file_name in sorted(file_names, key=str.casefold):
            # 目录 owner 门禁会区分空目录、README-only 与真实实现，因此不能按
            # 后缀过滤；任何实现文件出现都必须改变架构输入身份。
            entries.append(current / file_name)
    return entries


def architecture_inputs(root: pathlib.Path = ROOT) -> tuple[pathlib.Path, ...]:
    canonical_root = root.resolve()
    entries: set[pathlib.Path] = set()
    for relative_root in INPUT_ROOTS:
        entries.update(_tree_entries(canonical_root, relative_root))
    entries.update(canonical_root / relative for relative in EXPLICIT_INPUTS)
    return tuple(sorted(
        entries,
        key=lambda path: path.relative_to(canonical_root).as_posix().casefold(),
    ))


def architecture_identity(root: pathlib.Path = ROOT) -> bytes:
    """按相对路径、条目类型和原始内容计算稳定 SHA-256；缺失路径也进入合同。"""
    canonical_root = root.resolve()
    digest = hashlib.sha256()
    digest.update((IDENTITY_VERSION + "\0").encode("ascii"))
    entry_count = 0
    for path in architecture_inputs(canonical_root):
        relative = path.relative_to(canonical_root).as_posix()
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        if path.is_symlink():
            digest.update(b"L\0")
            digest.update(os.readlink(path).encode("utf-8"))
        elif path.is_file():
            digest.update(b"F\0")
            with path.open("rb") as source:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    digest.update(chunk)
        elif path.is_dir():
            digest.update(b"D\0")
        else:
            # 显式缺失路径也写入 M 标记；以后创建该路径必须使旧 stamp 失效。
            digest.update(b"M\0")
        digest.update(b"\0")
        entry_count += 1

    return (
        f"{IDENTITY_VERSION}\n"
        f"{canonical_root.as_posix()}\n"
        f"{entry_count}\n"
        f"{digest.hexdigest()}\n"
    ).encode("utf-8")


def stamp_is_current(
        stamp: pathlib.Path, root: pathlib.Path = ROOT) -> bool:
    try:
        return stamp.read_bytes() == architecture_identity(root)
    except FileNotFoundError:
        return False


def invalidate_stamp(stamp: pathlib.Path) -> None:
    stamp.unlink(missing_ok=True)


def update_stamp(
        stamp: pathlib.Path, root: pathlib.Path = ROOT,
        identity: bytes | None = None) -> bool:
    """同目录写临时文件、fsync 后原子替换，避免中断留下可误复用的半写 stamp。"""
    if identity is None:
        identity = architecture_identity(root)
    try:
        if stamp.read_bytes() == identity:
            return False
    except FileNotFoundError:
        pass

    stamp.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=stamp.parent, prefix=f".{stamp.name}.tmp."
    )
    temporary = pathlib.Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(identity)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, stamp)
    finally:
        temporary.unlink(missing_ok=True)
    return True


def main() -> int:
    """CLI 只查询 stamp 新旧；成功 stamp 只能由完整架构检查原子写入。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--stamp", required=True, type=pathlib.Path)
    parser.add_argument("--root", type=pathlib.Path, default=ROOT)
    parser.add_argument(
        "--status",
        action="store_true",
        help="print current or stale without modifying the stamp",
    )
    arguments = parser.parse_args()
    if arguments.status:
        print(
            "current"
            if stamp_is_current(arguments.stamp, arguments.root)
            else "stale"
        )
        return 0
    parser.error(
        "--status is required; only check_architecture.py may write a "
        "verified stamp"
    )


if __name__ == "__main__":
    raise SystemExit(main())
