#!/usr/bin/env python3
"""从受版本控制的 DSDL 源码树生成 H743 DroneCAN v0 协议合同。

DSDL 文件是消息类型的唯一权威输入；C codec 始终由固定版本的上游
dronecan_dsdlc 生成。本脚本只补充 Dima 节点所需的运行策略，不处理产品参数，
也不维护消息 ID、签名或线布局清单。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Iterable, Optional


sys.dont_write_bytecode = True


class GenerationError(RuntimeError):
    """可确定复现且带可操作原因的 DroneCAN 生成失败。"""


IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
DSDL_TYPE_RE = re.compile(
    r"^[a-z][a-z0-9_]*(?:\.[A-Za-z][A-Za-z0-9_]*)+$"
)
ROOT_DSDL_RE = re.compile(r"^(?P<id>[0-9]+)\.(?P<name>[A-Za-z][A-Za-z0-9_]*)\.uavcan$")

# DSDL 编译器和 Python 依赖属于工具链身份，而不是产品参数或消息清单。
# 固定散列保留可复现下载；DSDL 类型集合则始终从源码树自动发现。
PINNED_TOOLCHAIN: dict[str, Any] = {
    "generator": {
        "repository": "https://github.com/DroneCAN/dronecan_dsdlc",
        "commit": "431170fa4bfe2212b516b8f33bdc796267907f1c",
        "archive": {
            "url": "https://codeload.github.com/DroneCAN/dronecan_dsdlc/tar.gz/431170fa4bfe2212b516b8f33bdc796267907f1c",
            "size": 15289,
            "sha256": "fc427b3630d18c7149a3161739914b1095c10d5d4e3d65a4684e2325e8e55d71",
        },
    },
    "python_packages": [
        {
            "name": "dronecan",
            "version": "1.0.27",
            "source_commit": "08cda37aaf2958657399606653b99ceb5a6beae0",
            "url": "https://files.pythonhosted.org/packages/48/9f/fc77f73e9adb04ed1178083fc7d78dffa8f2fb98490a335fa3dcfdeb4402/dronecan-1.0.27-py3-none-any.whl",
            "size": 161959,
            "sha256": "206f70e44b74b85653acf53851df0f373dd4fe7f63db96df8cbb1e564112afeb",
        },
        {
            "name": "empy",
            "version": "3.3.4",
            "url": "https://files.pythonhosted.org/packages/3b/95/88ed47cb7da88569a78b7d6fb9420298df7e99997810c844a924d96d3c08/empy-3.3.4.tar.gz",
            "size": 62857,
            "sha256": "73ac49785b601479df4ea18a7c79bc1304a8a7c34c02b9472cf1206ae88f01b3",
        },
        {
            "name": "pexpect",
            "version": "4.9.0",
            "url": "https://files.pythonhosted.org/packages/9e/c3/059298687310d527a58bb01f3b1965787ee3b40dce76752eda8b44e9a2c5/pexpect-4.9.0-py2.py3-none-any.whl",
            "size": 63772,
            "sha256": "7236d1e080e4936be2dc3e326cec0af72acf9212a7e1d060210e70a47e253523",
        },
        {
            "name": "ptyprocess",
            "version": "0.7.0",
            "url": "https://files.pythonhosted.org/packages/22/a6/858897256d0deac81a172289110f31629fc4cee19b6f01283303e18c8db3/ptyprocess-0.7.0-py2.py3-none-any.whl",
            "size": 13993,
            "sha256": "4b41f3967fce3af57cc7e94b888626c18bf37a083e3651ca8feeb66d492fef35",
        },
    ],
}

# 以下值是 Dima DroneCAN 节点的运行策略，不参与参数定义。能从 DSDL
# 生成头取得的 ID、签名、分片长度和超时仍由编译器产物决定。
DIMA_ALLOCATION_POLICY: dict[str, Any] = {
    "max_node_id": 125,
    "unique_id_bytes": 16,
    "transfer_priority": 30,
    "storage": {
        "token": "dna0",
        "magic": "DNAV",
        "footer": "DEND",
        "format_version": 1,
        "states": {
            "empty": 0,
            "occupied_without_uid": 1,
            "known_uid": 2,
        },
    },
    "discovery": {
        "poll_interval_ms": 170,
        "response_timeout_ms": 500,
        "max_get_node_info_attempts": 5,
    },
    "persistence_retry_ms": 1000,
    "error_log_interval_ms": 10000,
}

DIMA_DEVICE_ID_POLICY: dict[str, Any] = {
    "bus_type": {"value": 3, "shift": 0},
    "bus": {"value": 0, "shift": 3},
    "address": {"shift": 8},
    "devtype": {"value": 136, "shift": 16},
}


def file_sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    """对排序紧凑 ASCII JSON 求散列，作为依赖缓存的内容身份。"""
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def directory_sha256(root: pathlib.Path) -> str:
    """把相对路径和每个文件散列共同纳入目录闭包，避免同内容换路径被误复用。"""
    digest = hashlib.sha256()
    paths = (entry for entry in root.rglob("*") if entry.is_file())
    for path in sorted(
        paths, key=lambda entry: entry.relative_to(root).as_posix()
    ):
        relative = path.relative_to(root).as_posix()
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_sha256(path).encode("ascii"))
        digest.update(b"\0")
    return digest.hexdigest()


def repository_root(path: pathlib.Path) -> pathlib.Path:
    for candidate in (path.resolve(), *path.resolve().parents):
        if (candidate / "GNUmakefile").is_file():
            return candidate
    raise GenerationError(f"cannot find repository root above {path}")


def discover_runtime_sources(search_root: pathlib.Path) -> tuple[pathlib.Path, ...]:
    """由源码命名约定发现第一方 DroneCAN 翻译单元，避免维护路径清单。"""
    root = repository_root(search_root)
    resolved_search_root = search_root.resolve()
    matches = tuple(sorted(
        (
            path for path in resolved_search_root.rglob("DroneCan*.cpp")
            if path.is_file()
        ),
        key=lambda path: path.relative_to(resolved_search_root).as_posix(),
    ))
    if not matches:
        raise GenerationError(
            f"no DroneCan*.cpp runtime sources found below {search_root.resolve()}"
        )
    for path in matches:
        if not path.is_relative_to(root):
            raise GenerationError(f"runtime source is outside repository: {path}")
    return matches


def snake_to_pascal(value: str) -> str:
    return "".join(part[:1].upper() + part[1:] for part in value.split("_"))


def camel_to_snake(value: str) -> str:
    """把 DSDL 类型尾名稳定转换为生成枚举名，不维护消息角色清单。"""
    words = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", value)
    return words.lower()


def dsdl_macro_prefix(type_name: str) -> str:
    return re.sub(r"[^A-Za-z0-9]", "_", type_name).upper()


def dsdl_header_name(type_name: str) -> str:
    return f"{type_name}.h"


def discover_contract(
    namespace_root: pathlib.Path,
) -> tuple[dict[str, Any], pathlib.Path]:
    """从 DSDL 源码树发现完整类型闭包和根消息，不保留额外 JSON 合同。"""
    namespace_root = namespace_root.resolve()
    root = repository_root(namespace_root)
    if not namespace_root.is_dir() or not namespace_root.is_relative_to(root):
        raise GenerationError(
            f"DroneCAN DSDL root is unavailable or outside repository: {namespace_root}"
        )

    namespace_name = namespace_root.name
    if IDENTIFIER_RE.fullmatch(namespace_name) is None:
        raise GenerationError(f"invalid DSDL namespace name: {namespace_name!r}")

    files: list[dict[str, Any]] = []
    root_roles: set[str] = set()
    type_names: set[str] = set()
    # WindowsPath 默认按大小写折叠比较，POSIX Path 则逐字节比较；显式使用
    # 相对 POSIX 路径排序，确保两端发现的 DSDL 有序闭包完全相同。
    dsdl_paths = sorted(
        namespace_root.rglob("*.uavcan"),
        key=lambda path: path.relative_to(namespace_root).as_posix(),
    )
    for path in dsdl_paths:
        relative = path.relative_to(namespace_root)
        root_match = ROOT_DSDL_RE.fullmatch(path.name)
        type_leaf = root_match.group("name") if root_match else path.stem
        type_name = ".".join((namespace_name, *relative.parts[:-1], type_leaf))
        if DSDL_TYPE_RE.fullmatch(type_name) is None or type_name in type_names:
            raise GenerationError(f"invalid or duplicate DSDL type: {type_name}")
        type_names.add(type_name)
        item: dict[str, Any] = {
            "path": relative.as_posix(),
            "sha256": file_sha256(path),
            "type": type_name,
        }
        if root_match:
            role = camel_to_snake(type_leaf)
            if role in root_roles:
                raise GenerationError(f"duplicate generated DSDL role: {role}")
            root_roles.add(role)
            is_service = any(
                line.strip() == "---"
                for line in path.read_text(encoding="utf-8").splitlines()
            )
            item.update({
                "role": role,
                "owner": camel_to_snake(relative.parts[0]),
                "transfers": ["request", "response"]
                    if is_service else ["broadcast"],
            })
        files.append(item)

    if not files or "allocation" not in root_roles:
        raise GenerationError(
            "DSDL tree must contain root types including dynamic Node ID Allocation"
        )
    namespace_relative = namespace_root.relative_to(root).as_posix()
    contract = {
        "contract": "dima_dronecan_v0",
        "toolchain": PINNED_TOOLCHAIN,
        "dsdl": {
            "namespace_root": namespace_relative,
            "files": files,
        },
        "device_id": DIMA_DEVICE_ID_POLICY,
        "allocation": DIMA_ALLOCATION_POLICY,
    }
    return contract, root


def dsdl_input_paths(contract: dict[str, Any]) -> list[str]:
    namespace = pathlib.PurePosixPath(contract["dsdl"]["namespace_root"])
    return [
        namespace.joinpath(*pathlib.PurePosixPath(item["path"]).parts).as_posix()
        for item in contract["dsdl"]["files"]
    ]


def download_verified(
    url: str,
    destination: pathlib.Path,
    expected_size: int,
    expected_sha256: str,
    label: str,
) -> None:
    """有界重试下载 pin 归档，并同时验证字节数与 SHA-256 后才允许进入缓存。"""
    last_error: Optional[BaseException] = None
    for attempt in range(5):
        digest = hashlib.sha256()
        downloaded = 0
        request = urllib.request.Request(
            url, headers={"User-Agent": "dima-rover-dronecan-generator/1"}
        )
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                with destination.open("wb") as output:
                    while True:
                        chunk = response.read(1024 * 1024)
                        if not chunk:
                            break
                        output.write(chunk)
                        digest.update(chunk)
                        downloaded += len(chunk)
        except (OSError, urllib.error.URLError) as error:
            destination.unlink(missing_ok=True)
            last_error = error
            if attempt + 1 < 5:
                time.sleep(0.25 * (attempt + 1))
                continue
            break
        actual_sha256 = digest.hexdigest()
        if downloaded != expected_size or actual_sha256 != expected_sha256:
            destination.unlink(missing_ok=True)
            raise GenerationError(
                f"pinned {label} archive mismatch: expected {expected_size} bytes "
                f"and {expected_sha256}, got {downloaded} bytes and {actual_sha256}"
            )
        return
    raise GenerationError(
        f"unable to download pinned {label} after 5 attempts: {last_error}"
    ) from last_error


def safe_extract_tar(archive: pathlib.Path, destination: pathlib.Path) -> pathlib.Path:
    """拒绝链接及目录穿越成员，并要求归档只有一个源码根。"""
    resolved_destination = destination.resolve()
    try:
        with tarfile.open(archive, "r:gz") as package:
            members = package.getmembers()
            for member in members:
                if member.issym() or member.islnk():
                    raise GenerationError(f"archive contains a link: {member.name}")
                resolved_member = (destination / member.name).resolve()
                if (
                    resolved_member != resolved_destination
                    and resolved_destination not in resolved_member.parents
                ):
                    raise GenerationError(f"unsafe archive path: {member.name}")
            package.extractall(destination)
    except (OSError, tarfile.TarError) as error:
        raise GenerationError(f"unable to extract pinned archive: {error}") from error
    roots = [path for path in destination.iterdir() if path.is_dir()]
    if len(roots) != 1:
        raise GenerationError("pinned archive must contain one source root")
    return roots[0]


def install_directory(candidate: pathlib.Path, destination: pathlib.Path) -> None:
    attempts = 5 if os.name == "nt" else 1
    for attempt in range(attempts):
        try:
            if destination.exists():
                shutil.rmtree(destination)
            os.replace(candidate, destination)
            return
        except PermissionError as error:
            if attempt + 1 == attempts:
                raise GenerationError(
                    f"unable to install generated directory {destination}: {error}"
                ) from error
            time.sleep(0.05 * (attempt + 1))


def provision_compiler(contract: dict[str, Any], cache_root: pathlib.Path) -> pathlib.Path:
    """按 commit/归档散列/源码树散列复用 DSDL 编译器，否则在临时目录重建。"""
    generator = contract["toolchain"]["generator"]
    commit = generator["commit"]
    destination = cache_root / "dronecan_dsdlc" / commit / "source"
    script = destination / "dronecan_dsdlc.py"
    marker = destination.parent / ".source.json"
    if script.is_file() and marker.is_file():
        try:
            installed = json.loads(marker.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            installed = None
        if (isinstance(installed, dict) and
                installed.get("commit") == commit and
                installed.get("archive_sha256") ==
                    generator["archive"]["sha256"] and
                installed.get("source_sha256") ==
                    directory_sha256(destination)):
            return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".dronecan-dsdlc-", dir=destination.parent
    ) as temporary_name:
        temporary = pathlib.Path(temporary_name)
        archive_path = temporary / "dronecan_dsdlc.tar.gz"
        unpacked = temporary / "unpacked"
        unpacked.mkdir()
        archive = generator["archive"]
        download_verified(
            archive["url"], archive_path, archive["size"], archive["sha256"],
            "dronecan_dsdlc",
        )
        source = safe_extract_tar(archive_path, unpacked)
        for required in (
            "dronecan_dsdlc.py",
            "dronecan_dsdlc_helpers.py",
            "dronecan_dsdlc_tester.py",
            "templates/msg.h.em",
            "templates/msg.c.em",
            "templates/service.h.em",
        ):
            if not (source / required).is_file():
                raise GenerationError(f"pinned compiler lacks {required}")
        candidate = temporary / "source"
        shutil.copytree(source, candidate)
        install_directory(candidate, destination)
        source_identity = {
            "archive_sha256": generator["archive"]["sha256"],
            "commit": commit,
            "source_sha256": directory_sha256(destination),
        }
        with marker.open("w", encoding="utf-8", newline="\n") as output:
            output.write(
                json.dumps(source_identity, indent=2, sort_keys=True) + "\n"
            )
    if not script.is_file():
        raise GenerationError("cached dronecan_dsdlc failed final validation")
    return destination


def python_probe(site_packages: pathlib.Path, packages: list[dict[str, Any]]) -> bool:
    expected = {package["name"]: package["version"] for package in packages}
    code = (
        "import json,sys; "
        "sys.path.insert(0, sys.argv[1]); "
        "import dronecan,em,pexpect; "
        "from importlib.metadata import version; "
        "print(json.dumps({name:version(name) for name in "
        "('dronecan','empy','pexpect','ptyprocess')},sort_keys=True))"
    )
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    completed = subprocess.run(
        [sys.executable, "-S", "-c", code, str(site_packages)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        encoding="utf-8",
        errors="replace",
        env=environment,
    )
    if completed.returncode != 0:
        return False
    try:
        actual = json.loads(completed.stdout)
    except json.JSONDecodeError:
        return False
    return actual == expected


def provision_python_packages(
    contract: dict[str, Any], cache_root: pathlib.Path
) -> pathlib.Path:
    """按完整依赖清单内容寻址安装 Python 包，并用 import probe 验证缓存可用。"""
    packages = contract["toolchain"]["python_packages"]
    identity = canonical_sha256(packages)
    destination = (
        cache_root / "dronecan-python" / identity[:16] / "site-packages"
    )
    marker = destination.parent / ".installation.json"
    if destination.is_dir() and marker.is_file():
        try:
            installed = json.loads(marker.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            installed = None
        if installed == {"identity": identity, "packages": packages} and python_probe(destination, packages):
            return destination

    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".dronecan-python-", dir=destination.parent
    ) as temporary_name:
        temporary = pathlib.Path(temporary_name)
        archives = temporary / "archives"
        candidate = temporary / "site-packages"
        archives.mkdir()
        candidate.mkdir()
        requirement_lines: list[str] = []
        for package in packages:
            filename = pathlib.PurePosixPath(
                urllib.parse.urlparse(package["url"]).path
            ).name
            if not filename:
                raise GenerationError(f"package URL has no filename: {package['url']}")
            archive_path = archives / filename
            download_verified(
                package["url"], archive_path, package["size"],
                package["sha256"], package["name"],
            )
            requirement_lines.append(
                f"{package['name']} @ {archive_path.resolve().as_uri()} "
                f"--hash=sha256:{package['sha256']}"
            )
        requirements = temporary / "requirements.txt"
        with requirements.open("w", encoding="utf-8", newline="\n") as output:
            output.write("\n".join(requirement_lines) + "\n")
        command = [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--disable-pip-version-check",
            "--no-input",
            "--no-user",
            "--no-index",
            "--no-deps",
            "--require-hashes",
            "--no-build-isolation",
            "--no-compile",
            "--target",
            str(candidate),
            "--requirement",
            str(requirements),
        ]
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            encoding="utf-8",
            errors="replace",
        )
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip()
            raise GenerationError(f"unable to install pinned DSDL Python tools: {detail}")
        if not python_probe(candidate, packages):
            raise GenerationError("pinned DSDL Python tools failed import/version validation")
        install_directory(candidate, destination)
        with marker.open("w", encoding="utf-8", newline="\n") as output:
            output.write(json.dumps(
                {"identity": identity, "packages": packages},
                indent=2,
                sort_keys=True,
            ) + "\n")
    if not python_probe(destination, packages):
        raise GenerationError("cached DSDL Python tools failed final validation")
    return destination


def normalize_generated_files(root: pathlib.Path) -> None:
    for path in (entry for entry in root.rglob("*") if entry.is_file()):
        content = path.read_bytes()
        normalized = content.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
        if normalized != content:
            path.write_bytes(normalized)


def run_dsdl_compiler(
    contract: dict[str, Any],
    root: pathlib.Path,
    output: pathlib.Path,
    compiler_root: pathlib.Path,
    site_packages: pathlib.Path,
) -> None:
    """只把自动发现的根 DSDL 类型传给固定上游工具，单 job 保证确定性。"""
    roots = [item for item in contract["dsdl"]["files"] if "role" in item]
    namespace = root.joinpath(
        *pathlib.PurePosixPath(contract["dsdl"]["namespace_root"]).parts
    )
    command = [
        sys.executable,
        "-S",
        str(compiler_root / "dronecan_dsdlc.py"),
        "--output",
        str(output),
        "--jobs",
        "1",
    ]
    for item in roots:
        command.extend(("--build", item["type"]))
    command.append(str(namespace))
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(site_packages)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONHASHSEED"] = "0"
    completed = subprocess.run(
        command,
        cwd=compiler_root,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        encoding="utf-8",
        errors="replace",
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise GenerationError(f"pinned dronecan_dsdlc failed: {detail}")
    if not (output / "include" / "dronecan_msgs.h").is_file():
        raise GenerationError("pinned dronecan_dsdlc did not emit dronecan_msgs.h")
    normalize_generated_files(output)


def macro_integer(header: pathlib.Path, macro: str) -> int:
    text = header.read_text(encoding="utf-8")
    match = re.search(
        rf"^\s*#define\s+{re.escape(macro)}\s+(.+?)\s*$", text, re.MULTILINE
    )
    if match is None:
        raise GenerationError(f"generated header lacks {macro}: {header}")
    value = match.group(1).strip()
    while value.startswith("(") and value.endswith(")"):
        value = value[1:-1].strip()
    value = re.sub(r"(?:ULL|LLU|UL|LU|LL|U|L)$", "", value, flags=re.IGNORECASE)
    try:
        return int(value, 0)
    except ValueError as error:
        raise GenerationError(f"cannot parse generated macro {macro}={value}") from error


def role_descriptors(
    contract: dict[str, Any], output: pathlib.Path
) -> list[dict[str, Any]]:
    """从生成头读取上游计算的 ID/签名，并绑定自动发现的类型语义。"""
    descriptors: list[dict[str, Any]] = []
    for item in contract["dsdl"]["files"]:
        if "role" not in item:
            continue
        prefix = dsdl_macro_prefix(item["type"])
        header = output / "include" / dsdl_header_name(item["type"])
        if not header.is_file():
            raise GenerationError(f"generated DSDL header is missing: {header.name}")
        descriptors.append({
            "role": item["role"],
            "owner": item["owner"],
            "transfers": item["transfers"],
            "type": item["type"],
            "macro": prefix,
            "id": macro_integer(header, f"{prefix}_ID"),
            "signature": macro_integer(header, f"{prefix}_SIGNATURE"),
        })
    return descriptors


def cpp_hex(value: int, digits: int) -> str:
    return f"0x{value:0{digits}X}"


def ascii_u32(value: str) -> int:
    return int.from_bytes(value.encode("ascii"), "little")


def render_protocol_contract(
    contract: dict[str, Any], descriptors: list[dict[str, Any]], output: pathlib.Path
) -> str:
    """从 descriptors 生成运行期订阅表，源码不得另写 DroneCAN 消息 ID 或签名。"""
    role_entries = [snake_to_pascal(item["role"]) for item in descriptors]
    owner_entries = sorted({
        snake_to_pascal(item["owner"]) for item in descriptors
    })
    subscription_rows: list[str] = []
    for item in descriptors:
        for transfer in item["transfers"]:
            subscription_rows.append(
                "    {SubscriptionOwner::%s, TransferKind::%s, "
                "MessageRole::%s, %dU, %sULL}," % (
                    snake_to_pascal(item["owner"]),
                    snake_to_pascal(transfer),
                    snake_to_pascal(item["role"]),
                    item["id"],
                    cpp_hex(item["signature"], 16),
                )
            )

    allocation_descriptor = next(
        item for item in descriptors if item["role"] == "allocation"
    )
    allocation_header = (
        output / "include" / dsdl_header_name(allocation_descriptor["type"])
    )
    prefix = allocation_descriptor["macro"]
    followup_timeout_ms = macro_integer(
        allocation_header, f"{prefix}_FOLLOWUP_TIMEOUT_MS"
    )
    request_fragment_bytes = macro_integer(
        allocation_header, f"{prefix}_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST"
    )
    any_node_id = macro_integer(allocation_header, f"{prefix}_ANY_NODE_ID")
    allocation = contract["allocation"]
    storage = allocation["storage"]
    states = storage["states"]
    unique_id_bytes = allocation["unique_id_bytes"]
    max_node_id = allocation["max_node_id"]
    header_bytes = 8
    entry_bytes = unique_id_bytes + 1
    records_bytes = max_node_id * entry_bytes
    footer_offset = header_bytes + records_bytes
    image_bytes = footer_offset + 4
    if image_bytes > 0xFFFF:
        raise GenerationError("allocation storage image does not fit uint16 size")

    device = contract["device_id"]
    bus_type = device["bus_type"]
    bus = device["bus"]
    address = device["address"]
    devtype = device["devtype"]
    example_node = max_node_id
    example_device_id = (
        (bus_type["value"] << bus_type["shift"])
        | (bus["value"] << bus["shift"])
        | (example_node << address["shift"])
        | (devtype["value"] << devtype["shift"])
    )
    token_values = ", ".join(f"{ord(character)}U" for character in storage["token"])

    return "\n".join([
        "#pragma once",
        "",
        "// Generated from the pinned DSDL source closure. DO NOT EDIT.",
        "// This header exposes no libcanard or generated DSDL structure ABI.",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace dima::protocols::dronecan::generated {",
        "",
        "enum class SubscriptionOwner : std::uint8_t {",
        *[f"    {entry}," for entry in owner_entries],
        "};",
        "",
        "enum class TransferKind : std::uint8_t {",
        "    Broadcast,",
        "    Request,",
        "    Response,",
        "};",
        "",
        "enum class MessageRole : std::uint8_t {",
        *[f"    {entry}," for entry in role_entries],
        "};",
        "",
        "struct SubscriptionDescriptor {",
        "    SubscriptionOwner owner;",
        "    TransferKind transfer_kind;",
        "    MessageRole role;",
        "    std::uint16_t data_type_id;",
        "    std::uint64_t signature;",
        "};",
        "",
        "inline constexpr SubscriptionDescriptor kSubscriptions[]{",
        *subscription_rows,
        "};",
        "inline constexpr std::size_t kSubscriptionCount =",
        "    sizeof(kSubscriptions) / sizeof(kSubscriptions[0]);",
        "",
        "constexpr const SubscriptionDescriptor *find_subscription(",
        "    SubscriptionOwner owner, TransferKind transfer_kind,",
        "    std::uint16_t data_type_id) noexcept",
        "{",
        "    for (const auto &subscription : kSubscriptions) {",
        "        if (subscription.owner == owner &&",
        "            subscription.transfer_kind == transfer_kind &&",
        "            subscription.data_type_id == data_type_id) {",
        "            return &subscription;",
        "        }",
        "    }",
        "    return nullptr;",
        "}",
        "",
        "constexpr const SubscriptionDescriptor *find_subscription(",
        "    SubscriptionOwner owner, TransferKind transfer_kind,",
        "    MessageRole role) noexcept",
        "{",
        "    for (const auto &subscription : kSubscriptions) {",
        "        if (subscription.owner == owner &&",
        "            subscription.transfer_kind == transfer_kind &&",
        "            subscription.role == role) {",
        "            return &subscription;",
        "        }",
        "    }",
        "    return nullptr;",
        "}",
        "",
        f"inline constexpr std::size_t kUniqueIdBytes = {unique_id_bytes}U;",
        f"inline constexpr std::uint8_t kMaximumNodeId = {max_node_id}U;",
        f"inline constexpr std::uint8_t kAnyNodeId = {any_node_id}U;",
        f"inline constexpr std::uint8_t kAllocationTransferPriority = {allocation['transfer_priority']}U;",
        f"inline constexpr std::size_t kAllocationRequestFragmentBytes = {request_fragment_bytes}U;",
        f"inline constexpr std::uint64_t kAllocationFollowupTimeoutUs = {followup_timeout_ms}000ULL;",
        f"inline constexpr std::uint64_t kDiscoveryPollIntervalUs = {allocation['discovery']['poll_interval_ms']}000ULL;",
        f"inline constexpr std::uint64_t kDiscoveryResponseTimeoutUs = {allocation['discovery']['response_timeout_ms']}000ULL;",
        f"inline constexpr std::uint8_t kDiscoveryMaximumAttempts = {allocation['discovery']['max_get_node_info_attempts']}U;",
        f"inline constexpr std::uint64_t kPersistenceRetryUs = {allocation['persistence_retry_ms']}000ULL;",
        f"inline constexpr std::uint64_t kErrorLogIntervalUs = {allocation['error_log_interval_ms']}000ULL;",
        "",
        f"inline constexpr std::uint8_t kAllocationStorageToken[]{{{token_values}}};",
        f"inline constexpr std::uint32_t kAllocationStorageMagic = {cpp_hex(ascii_u32(storage['magic']), 8)}U;",
        f"inline constexpr std::uint32_t kAllocationStorageFooter = {cpp_hex(ascii_u32(storage['footer']), 8)}U;",
        f"inline constexpr std::uint16_t kAllocationStorageVersion = {storage['format_version']}U;",
        f"inline constexpr std::uint8_t kAllocationStateEmpty = {states['empty']}U;",
        f"inline constexpr std::uint8_t kAllocationStateOccupiedWithoutUid = {states['occupied_without_uid']}U;",
        f"inline constexpr std::uint8_t kAllocationStateKnownUid = {states['known_uid']}U;",
        f"inline constexpr std::size_t kAllocationStorageHeaderBytes = {header_bytes}U;",
        f"inline constexpr std::size_t kAllocationStorageEntryBytes = {entry_bytes}U;",
        f"inline constexpr std::size_t kAllocationStorageFooterOffset = {footer_offset}U;",
        f"inline constexpr std::size_t kAllocationStorageImageBytes = {image_bytes}U;",
        "",
        f"inline constexpr std::uint32_t kDeviceBusTypeUavcan = {bus_type['value']}U;",
        f"inline constexpr std::uint32_t kMagnetometerDeviceTypeUavcan = {devtype['value']}U;",
        "constexpr std::uint32_t make_magnetometer_device_id(",
        "    std::uint8_t source_node_id) noexcept",
        "{",
        f"    return (kDeviceBusTypeUavcan << {bus_type['shift']}U) |",
        f"           ({bus['value']}U << {bus['shift']}U) |",
        f"           (static_cast<std::uint32_t>(source_node_id) << {address['shift']}U) |",
        f"           (kMagnetometerDeviceTypeUavcan << {devtype['shift']}U);",
        "}",
        f"static_assert(make_magnetometer_device_id({example_node}U) == {cpp_hex(example_device_id, 8)}U);",
        "",
        "} // namespace dima::protocols::dronecan::generated",
        "",
    ])


def make_list(name: str, values: Iterable[str]) -> list[str]:
    items = list(values)
    if not items:
        return [f"{name} :="]
    lines = [f"{name} := \\"]
    for index, value in enumerate(items):
        suffix = " \\" if index + 1 < len(items) else ""
        lines.append(f"\t{value}{suffix}")
    return lines


def tree_hashes(root: pathlib.Path, exclude: Iterable[str] = ()) -> dict[str, str]:
    excluded = set(exclude)
    return {
        path.relative_to(root).as_posix(): file_sha256(path)
        for path in sorted(
            (entry for entry in root.rglob("*") if entry.is_file()),
            key=lambda entry: entry.relative_to(root).as_posix(),
        )
        if path.relative_to(root).as_posix() not in excluded
    }


def render_make_fragment(output_relative: str, generated: pathlib.Path) -> str:
    sources = [
        f"{output_relative}/{path.relative_to(generated).as_posix()}"
        for path in sorted(
            (generated / "src").glob("*.c"),
            key=lambda entry: entry.relative_to(generated).as_posix(),
        )
    ]
    outputs = [
        f"{output_relative}/{path.relative_to(generated).as_posix()}"
        for path in sorted(
            (entry for entry in generated.rglob("*") if entry.is_file()),
            key=lambda entry: entry.relative_to(generated).as_posix(),
        )
    ]
    lines = [
        "# Generated from the DSDL source closure. DO NOT EDIT.",
        *make_list("DIMA_DRONECAN_GENERATED_C_SOURCES", sources),
        f"DIMA_DRONECAN_PROTOCOL_HEADER := {output_relative}/DroneCanContract.hpp",
        *make_list("DIMA_DRONECAN_GENERATED_OUTPUTS", sorted(set(outputs))),
        "",
    ]
    return "\n".join(lines)


def write_text(path: pathlib.Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(content)


def build_generated_tree(
    contract: dict[str, Any],
    root: pathlib.Path,
    generated: pathlib.Path,
    logical_output: pathlib.Path,
    cache_root: pathlib.Path,
) -> None:
    """在候选目录构建 codec、协议合同和 Make 片段。"""
    compiler_root = provision_compiler(contract, cache_root)
    site_packages = provision_python_packages(contract, cache_root)
    run_dsdl_compiler(
        contract, root, generated, compiler_root, site_packages
    )
    descriptors = role_descriptors(contract, generated)
    write_text(
        generated / "DroneCanContract.hpp",
        render_protocol_contract(contract, descriptors, generated),
    )
    try:
        output_relative = logical_output.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as error:
        raise GenerationError("DroneCAN generated output must remain inside the repository") from error
    make_path = generated / "dronecan_sources.mk"
    write_text(make_path, render_make_fragment(output_relative, generated))


def generated_tree_current(candidate: pathlib.Path, destination: pathlib.Path) -> bool:
    return destination.is_dir() and tree_hashes(candidate) == tree_hashes(destination)


def generate(
    contract: dict[str, Any],
    root: pathlib.Path,
    output: pathlib.Path,
    cache_root: pathlib.Path,
) -> None:
    """临时目录完整生成后按目录散列比较并整体安装，避免暴露半代产物。"""
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".dronecan-generate-", dir=output.parent) as name:
        temporary = pathlib.Path(name)
        candidate = temporary / "dronecan"
        candidate.mkdir()
        build_generated_tree(
            contract, root, candidate, output, cache_root
        )
        if generated_tree_current(candidate, output):
            print(f"DroneCAN contract already current: {output}")
            return
        install_directory(candidate, output)
    print(
        f"Generated {len([item for item in contract['dsdl']['files'] if 'role' in item])} "
        f"DroneCAN root types into {output}"
    )


def verify_generated(
    contract: dict[str, Any],
    root: pathlib.Path,
    output: pathlib.Path,
    cache_root: pathlib.Path,
) -> None:
    """临时重跑锁定生成链并逐文件比较，不保存第二份 JSON 清单。"""
    if not output.is_dir():
        raise GenerationError(f"generated DroneCAN output is unavailable: {output}")
    # 校验候选树只存在于临时目录；DSDL、固定工具链和生成逻辑共同决定
    # 期望结果，避免把派生文件散列再固化成一份独立状态。
    with tempfile.TemporaryDirectory(
        prefix=".dronecan-verify-", dir=output.parent
    ) as name:
        candidate = pathlib.Path(name) / "dronecan"
        candidate.mkdir()
        build_generated_tree(
            contract, root, candidate, output, cache_root
        )
        expected_hashes = tree_hashes(candidate)
        actual_hashes = tree_hashes(output)
    if actual_hashes != expected_hashes:
        missing = sorted(set(expected_hashes) - set(actual_hashes))
        unexpected = sorted(set(actual_hashes) - set(expected_hashes))
        changed = sorted(
            path for path in set(expected_hashes) & set(actual_hashes)
            if expected_hashes[path] != actual_hashes[path]
        )
        raise GenerationError(
            "generated DroneCAN output differs from regenerated DSDL output: "
            f"missing={missing}, unexpected={unexpected}, changed={changed}"
        )
    print(f"DroneCAN generated contract verified: {output}")


def parse_arguments() -> argparse.Namespace:
    cache_override = os.environ.get("DIMA_HOST_TOOLS_CACHE")
    if cache_override:
        default_cache_root = pathlib.Path(cache_override)
    elif os.name == "nt" and os.environ.get("USERPROFILE"):
        default_cache_root = (
            pathlib.Path(os.environ["USERPROFILE"])
            / ".dima-rover"
            / "host-tools"
        )
    else:
        default_cache_root = (
            pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools"
        )
    parser = argparse.ArgumentParser()
    input_mode = parser.add_mutually_exclusive_group(required=True)
    input_mode.add_argument("--dsdl-root", type=pathlib.Path)
    input_mode.add_argument("--print-runtime-sources", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument(
        "--cache-root",
        type=pathlib.Path,
        default=default_cache_root,
    )
    parser.add_argument("--print-inputs", action="store_true")
    parser.add_argument("--verify", action="store_true")
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    if arguments.print_runtime_sources is not None:
        if (arguments.output is not None or arguments.print_inputs or
                arguments.verify):
            raise GenerationError(
                "--print-runtime-sources cannot be combined with generation modes"
            )
        root = repository_root(arguments.print_runtime_sources)
        sources = discover_runtime_sources(arguments.print_runtime_sources)
        print(" ".join(path.relative_to(root).as_posix() for path in sources))
        return 0
    if arguments.dsdl_root is None:
        raise GenerationError("--dsdl-root is required for generation")
    contract, root = discover_contract(arguments.dsdl_root)
    if arguments.print_inputs:
        if arguments.output is not None or arguments.verify:
            raise GenerationError("--print-inputs cannot be combined with output modes")
        print(" ".join(dsdl_input_paths(contract)))
        return 0
    if arguments.output is None:
        raise GenerationError("--output is required unless --print-inputs is used")
    output = arguments.output
    if not output.is_absolute():
        output = root / output
    if arguments.verify:
        verify_generated(
            contract,
            root,
            output.resolve(),
            arguments.cache_root.expanduser().resolve(),
        )
    else:
        generate(
            contract,
            root,
            output,
            arguments.cache_root.expanduser().resolve(),
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except GenerationError as error:
        print(f"DroneCAN generation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
