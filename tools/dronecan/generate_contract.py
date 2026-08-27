#!/usr/bin/env python3
"""从单一 manifest 生成完整 H743 DroneCAN v0 合同。

manifest 独占 DSDL 根、参数目录、订阅 role、PX4 device-ID 布局和动态分配策略；C codec
始终由固定版本上游 dronecan_dsdlc 生成。本脚本不得嵌入消息 ID、签名或线布局。
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
PARAMETER_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,15}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
DSDL_TYPE_RE = re.compile(
    r"^[a-z][a-z0-9_]*(?:\.[A-Za-z][A-Za-z0-9_]*)+$"
)
TRANSFER_KINDS = ("broadcast", "request", "response")
OWNERS = ("node", "magnetometer")


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
    for path in sorted(entry for entry in root.rglob("*") if entry.is_file()):
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


def discover_manifest(search_root: pathlib.Path) -> tuple[pathlib.Path, pathlib.Path]:
    """按唯一文件身份发现权威 manifest，不把其目录写死在构建脚本中。"""
    root = repository_root(search_root)
    resolved_search_root = search_root.resolve()
    matches = sorted(
        path for path in resolved_search_root.rglob("dronecan_contract.json")
        if path.is_file()
    )
    if len(matches) != 1:
        relative_matches = [
            path.relative_to(root).as_posix()
            for path in matches if path.is_relative_to(root)
        ]
        raise GenerationError(
            "expected exactly one dronecan_contract.json below "
            f"{resolved_search_root}, found {relative_matches}"
        )
    return matches[0], root


def discover_runtime_sources(search_root: pathlib.Path) -> tuple[pathlib.Path, ...]:
    """由源码命名约定发现第一方 DroneCAN 翻译单元，避免维护路径清单。"""
    root = repository_root(search_root)
    matches = tuple(sorted(
        path for path in search_root.resolve().rglob("DroneCan*.cpp")
        if path.is_file()
    ))
    if not matches:
        raise GenerationError(
            f"no DroneCan*.cpp runtime sources found below {search_root.resolve()}"
        )
    for path in matches:
        if not path.is_relative_to(root):
            raise GenerationError(f"runtime source is outside repository: {path}")
    return matches


def require_dict(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise GenerationError(f"{field} must be an object")
    return value


def require_list(value: Any, field: str) -> list[Any]:
    if not isinstance(value, list):
        raise GenerationError(f"{field} must be a list")
    return value


def require_int(value: Any, field: str, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise GenerationError(f"{field} must be an integer >= {minimum}")
    return value


def require_string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise GenerationError(f"{field} must be a non-empty string")
    return value


def require_sha256(value: Any, field: str) -> str:
    text = require_string(value, field)
    if SHA256_RE.fullmatch(text) is None:
        raise GenerationError(f"{field} must be a lowercase SHA-256")
    return text


def require_commit(value: Any, field: str) -> str:
    text = require_string(value, field)
    if COMMIT_RE.fullmatch(text) is None:
        raise GenerationError(f"{field} must be a lowercase Git commit")
    return text


def snake_to_pascal(value: str) -> str:
    return "".join(part[:1].upper() + part[1:] for part in value.split("_"))


def dsdl_macro_prefix(type_name: str) -> str:
    return re.sub(r"[^A-Za-z0-9]", "_", type_name).upper()


def dsdl_header_name(type_name: str) -> str:
    return f"{type_name}.h"


def parse_manifest(path: pathlib.Path) -> tuple[dict[str, Any], pathlib.Path]:
    """完整校验权威 manifest、固定上游、依赖散列、DSDL role 和参数 schema。"""
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise GenerationError(f"unable to read DroneCAN manifest: {error}") from error

    if manifest.get("format_version") != 1:
        raise GenerationError("unsupported DroneCAN manifest format")
    if manifest.get("contract") != "px4_v1_17_dronecan_v0_rm3100":
        raise GenerationError("unsupported DroneCAN contract identity")
    root = repository_root(path.parent)

    upstream = require_dict(manifest.get("upstream"), "upstream")
    for name in ("px4", "dsdl", "generator"):
        package = require_dict(upstream.get(name), f"upstream.{name}")
        require_string(package.get("repository"), f"upstream.{name}.repository")
        require_commit(package.get("commit"), f"upstream.{name}.commit")
    generator = upstream["generator"]
    archive = require_dict(generator.get("archive"), "upstream.generator.archive")
    require_string(archive.get("url"), "upstream.generator.archive.url")
    require_int(archive.get("size"), "upstream.generator.archive.size", 1)
    require_sha256(archive.get("sha256"), "upstream.generator.archive.sha256")

    packages = require_list(upstream.get("python_packages"), "python_packages")
    if [package.get("name") for package in packages] != [
        "dronecan", "empy", "pexpect", "ptyprocess"
    ]:
        raise GenerationError(
            "python_packages must pin dronecan, empy, pexpect, and ptyprocess in order"
        )
    for index, raw_package in enumerate(packages):
        package = require_dict(raw_package, f"python_packages[{index}]")
        name = require_string(package.get("name"), f"python_packages[{index}].name")
        if IDENTIFIER_RE.fullmatch(name) is None:
            raise GenerationError(f"invalid Python package name {name!r}")
        require_string(package.get("version"), f"python_packages[{index}].version")
        require_string(package.get("url"), f"python_packages[{index}].url")
        require_int(package.get("size"), f"python_packages[{index}].size", 1)
        require_sha256(package.get("sha256"), f"python_packages[{index}].sha256")
        if "source_commit" in package:
            require_commit(
                package["source_commit"],
                f"python_packages[{index}].source_commit",
            )

    dsdl = require_dict(manifest.get("dsdl"), "dsdl")
    namespace_relative = pathlib.PurePosixPath(
        require_string(dsdl.get("namespace_root"), "dsdl.namespace_root")
    )
    if namespace_relative.is_absolute() or ".." in namespace_relative.parts:
        raise GenerationError("dsdl.namespace_root must remain inside the repository")
    namespace_root = root.joinpath(*namespace_relative.parts)
    files = require_list(dsdl.get("files"), "dsdl.files")
    if not files:
        raise GenerationError("dsdl.files must not be empty")
    paths: set[str] = set()
    types: set[str] = set()
    roles: set[str] = set()
    for index, raw_file in enumerate(files):
        item = require_dict(raw_file, f"dsdl.files[{index}]")
        type_name = require_string(item.get("type"), f"dsdl.files[{index}].type")
        if DSDL_TYPE_RE.fullmatch(type_name) is None or type_name in types:
            raise GenerationError(f"invalid or duplicate DSDL type {type_name!r}")
        types.add(type_name)
        relative_text = require_string(item.get("path"), f"dsdl.files[{index}].path")
        relative = pathlib.PurePosixPath(relative_text)
        if relative.is_absolute() or ".." in relative.parts or relative.suffix != ".uavcan":
            raise GenerationError(f"invalid DSDL path {relative_text!r}")
        if relative_text in paths:
            raise GenerationError(f"duplicate DSDL path {relative_text!r}")
        paths.add(relative_text)
        require_sha256(item.get("sha256"), f"dsdl.files[{index}].sha256")
        role = item.get("role")
        if role is None:
            if any(key in item for key in ("owner", "transfers", "allow_anonymous")):
                raise GenerationError(f"dependency DSDL {type_name} has subscription fields")
            continue
        role = require_string(role, f"dsdl.files[{index}].role")
        if IDENTIFIER_RE.fullmatch(role) is None or role in roles:
            raise GenerationError(f"invalid or duplicate DSDL role {role!r}")
        roles.add(role)
        if item.get("owner") not in OWNERS:
            raise GenerationError(f"invalid subscription owner for {type_name}")
        transfers = require_list(item.get("transfers"), f"{type_name}.transfers")
        if not transfers or any(kind not in TRANSFER_KINDS for kind in transfers):
            raise GenerationError(f"invalid transfer list for {type_name}")
        if len(transfers) != len(set(transfers)):
            raise GenerationError(f"duplicate transfer kind for {type_name}")
        if type(item.get("allow_anonymous")) is not bool:
            raise GenerationError(f"allow_anonymous must be boolean for {type_name}")
    # 角色集合完全由 manifest 声明；生成器只要求动态分配的语义锚点存在，
    # 其余 MessageRole/订阅表均从当前集合生成，不能在脚本中复制消息清单。
    if "allocation" not in roles:
        raise GenerationError("DSDL roles must contain the allocation contract")
    actual_dsdl = {
        path.relative_to(namespace_root).as_posix()
        for path in namespace_root.rglob("*.uavcan")
        if path.is_file()
    }
    if actual_dsdl != paths:
        raise GenerationError(
            "vendored DSDL tree differs from manifest: "
            f"missing={sorted(paths - actual_dsdl)}, "
            f"unexpected={sorted(actual_dsdl - paths)}"
        )
    for item in files:
        source = namespace_root.joinpath(*pathlib.PurePosixPath(item["path"]).parts)
        actual_hash = file_sha256(source)
        if actual_hash != item["sha256"]:
            raise GenerationError(
                f"DSDL hash mismatch for {item['path']}: "
                f"expected {item['sha256']}, got {actual_hash}"
            )

    parameters = require_list(manifest.get("parameters"), "parameters")
    parameter_names: set[str] = set()
    parameter_roles: set[str] = set()
    for index, raw_parameter in enumerate(parameters):
        parameter = require_dict(raw_parameter, f"parameters[{index}]")
        name = require_string(parameter.get("name"), f"parameters[{index}].name")
        if PARAMETER_RE.fullmatch(name) is None or name in parameter_names:
            raise GenerationError(f"invalid or duplicate parameter {name!r}")
        parameter_names.add(name)
        role = require_string(parameter.get("role"), f"{name}.role")
        if IDENTIFIER_RE.fullmatch(role) is None or role in parameter_roles:
            raise GenerationError(f"invalid or duplicate parameter role {role!r}")
        parameter_roles.add(role)
        if parameter.get("type") not in ("INT32", "FLOAT"):
            raise GenerationError(f"unsupported type for {name}")
        default = parameter.get("default")
        if parameter["type"] == "INT32" and type(default) is not int:
            raise GenerationError(f"{name} default must be an integer")
        if parameter["type"] == "FLOAT" and not isinstance(default, (int, float)):
            raise GenerationError(f"{name} default must be numeric")
        require_string(parameter.get("group"), f"{name}.group")
        description = require_list(parameter.get("description"), f"{name}.description")
        if not description or any(not isinstance(line, str) or not line for line in description):
            raise GenerationError(f"{name} description must contain text lines")
        values = parameter.get("values", [])
        if not isinstance(values, list):
            raise GenerationError(f"{name}.values must be a list")
        seen_values: set[int] = set()
        for value in values:
            if not isinstance(value, dict) or type(value.get("value")) is not int:
                raise GenerationError(f"{name} has an invalid value entry")
            if value["value"] in seen_values or not isinstance(value.get("name"), str):
                raise GenerationError(f"{name} has a duplicate or unnamed value")
            seen_values.add(value["value"])
            if "symbol" in value and IDENTIFIER_RE.fullmatch(str(value["symbol"])) is None:
                raise GenerationError(f"{name} has an invalid mode symbol")
    if not parameter_names:
        raise GenerationError("parameters must not be empty")

    device_id = require_dict(manifest.get("px4_device_id"), "px4_device_id")
    expected_device_id = {
        "layout": "DeviceStructure",
        "bus_type": {"name": "UAVCAN", "value": 3, "shift": 0},
        "bus": {"value": 0, "shift": 3},
        "address": {"source": "source_node_id", "shift": 8},
        "devtype": {
            "name": "DRV_MAG_DEVTYPE_UAVCAN",
            "value": 0x88,
            "shift": 16,
        },
        "sensor_id_policy": "ignored",
    }
    if device_id != expected_device_id:
        raise GenerationError(
            "magnetometer device ID must match the fixed PX4 UAVCAN "
            "DeviceStructure layout"
        )
    for field in ("bus_type", "bus", "address", "devtype"):
        entry = require_dict(device_id.get(field), f"px4_device_id.{field}")
        require_int(entry.get("shift"), f"px4_device_id.{field}.shift")
        if field != "address":
            require_int(entry.get("value"), f"px4_device_id.{field}.value")
    if device_id["address"].get("source") != "source_node_id":
        raise GenerationError("PX4 device address must come from source_node_id")

    allocation = require_dict(manifest.get("allocation"), "allocation")
    max_node_id = require_int(allocation.get("max_node_id"), "allocation.max_node_id", 1)
    if max_node_id != 125 or allocation.get("reserved_node_ids") != [126, 127]:
        raise GenerationError("automatic allocation must reserve Node IDs 126 and 127")
    if require_int(allocation.get("unique_id_bytes"), "allocation.unique_id_bytes", 1) != 16:
        raise GenerationError("DroneCAN v0 unique IDs must contain 16 bytes")
    transfer_priority = require_int(
        allocation.get("transfer_priority"), "allocation.transfer_priority"
    )
    if transfer_priority > 31:
        raise GenerationError("DroneCAN transfer priority must fit five bits")
    storage = require_dict(allocation.get("storage"), "allocation.storage")
    for field, length in (("token", 4), ("magic", 4), ("footer", 4)):
        value = require_string(storage.get(field), f"allocation.storage.{field}")
        if len(value) != length or not value.isascii():
            raise GenerationError(f"allocation.storage.{field} must be four ASCII bytes")
    require_int(storage.get("format_version"), "allocation.storage.format_version", 1)
    states = require_dict(storage.get("states"), "allocation.storage.states")
    if states != {"empty": 0, "occupied_without_uid": 1, "known_uid": 2}:
        raise GenerationError("unsupported allocation storage states")
    discovery = require_dict(allocation.get("discovery"), "allocation.discovery")
    for field in ("poll_interval_ms", "response_timeout_ms", "max_get_node_info_attempts"):
        require_int(discovery.get(field), f"allocation.discovery.{field}", 1)
    require_int(allocation.get("persistence_retry_ms"), "allocation.persistence_retry_ms", 1)
    require_int(allocation.get("error_log_interval_ms"), "allocation.error_log_interval_ms", 1)
    return manifest, root


def dsdl_input_paths(manifest: dict[str, Any]) -> list[str]:
    namespace = pathlib.PurePosixPath(manifest["dsdl"]["namespace_root"])
    return [
        namespace.joinpath(*pathlib.PurePosixPath(item["path"]).parts).as_posix()
        for item in manifest["dsdl"]["files"]
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


def provision_compiler(manifest: dict[str, Any], cache_root: pathlib.Path) -> pathlib.Path:
    """按 commit/归档散列/源码树散列复用 DSDL 编译器，否则在临时目录重建。"""
    generator = manifest["upstream"]["generator"]
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
    manifest: dict[str, Any], cache_root: pathlib.Path
) -> pathlib.Path:
    """按完整依赖清单内容寻址安装 Python 包，并用 import probe 验证缓存可用。"""
    packages = manifest["upstream"]["python_packages"]
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
    manifest: dict[str, Any],
    root: pathlib.Path,
    output: pathlib.Path,
    compiler_root: pathlib.Path,
    site_packages: pathlib.Path,
) -> None:
    """只把 manifest 标记 role 的 DSDL 类型传给固定上游工具，单 job 保证确定性。"""
    roots = [item for item in manifest["dsdl"]["files"] if "role" in item]
    namespace = root.joinpath(
        *pathlib.PurePosixPath(manifest["dsdl"]["namespace_root"]).parts
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
    manifest: dict[str, Any], output: pathlib.Path
) -> list[dict[str, Any]]:
    """从生成头读取上游计算的 ID/签名，再与 manifest role/owner/transfer 绑定。"""
    descriptors: list[dict[str, Any]] = []
    for item in manifest["dsdl"]["files"]:
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
            "allow_anonymous": item["allow_anonymous"],
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
    manifest: dict[str, Any], descriptors: list[dict[str, Any]], output: pathlib.Path
) -> str:
    """从 descriptors 生成运行期订阅表，源码不得另写 DroneCAN 消息 ID 或签名。"""
    role_entries = [snake_to_pascal(item["role"]) for item in descriptors]
    subscription_rows: list[str] = []
    for item in descriptors:
        for transfer in item["transfers"]:
            subscription_rows.append(
                "    {SubscriptionOwner::%s, TransferKind::%s, "
                "MessageRole::%s, %dU, %sULL, %s}," % (
                    snake_to_pascal(item["owner"]),
                    snake_to_pascal(transfer),
                    snake_to_pascal(item["role"]),
                    item["id"],
                    cpp_hex(item["signature"], 16),
                    "true" if item["allow_anonymous"] else "false",
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
    allocation = manifest["allocation"]
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

    device = manifest["px4_device_id"]
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
        "// Generated from dronecan_contract.json and pinned DSDL. DO NOT EDIT.",
        "// This header exposes no libcanard or generated DSDL structure ABI.",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace dima::protocols::dronecan::generated {",
        "",
        "enum class SubscriptionOwner : std::uint8_t {",
        "    Node,",
        "    Magnetometer,",
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
        "    bool allow_anonymous;",
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


def parameter_by_role(manifest: dict[str, Any], role: str) -> dict[str, Any]:
    """按 manifest 语义角色取参数，避免生成器再复制参数名称。"""
    matches = [
        parameter for parameter in manifest["parameters"]
        if parameter["role"] == role
    ]
    if len(matches) != 1:
        raise GenerationError(f"parameter role {role!r} must resolve exactly once")
    return matches[0]


def render_parameter_contract(manifest: dict[str, Any]) -> str:
    """从 manifest 生成参数 handle、模式与支持波特率合同，不复制参数定义。"""
    parameters = manifest["parameters"]
    enable = parameter_by_role(manifest, "mode")
    modes = {
        value["symbol"]: value["value"]
        for value in enable["values"]
        if "symbol" in value
    }
    if set(modes) != {"disabled", "manual", "automatic"}:
        raise GenerationError("mode parameter must define disabled/manual/automatic")
    bitrate = parameter_by_role(manifest, "bitrate")
    bitrates = ", ".join(f"{value['value']}U" for value in bitrate["values"])
    local_node = parameter_by_role(manifest, "local_node_id")
    magnetic_node = parameter_by_role(manifest, "magnetometer_node_id")
    handles = [f"    px4::params::{parameter['name']}," for parameter in parameters]
    role_handles = [
        "inline constexpr px4::params kParameter%s = px4::params::%s;" % (
            snake_to_pascal(parameter["role"]), parameter["name"]
        )
        for parameter in parameters
    ]
    return "\n".join([
        "#pragma once",
        "",
        "// Generated DroneCAN parameter binding. DO NOT EDIT.",
        "#include <parameters/param.h>",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace dima::protocols::dronecan::generated {",
        "",
        "inline constexpr px4::params kParameterHandles[]{",
        *handles,
        "};",
        "inline constexpr std::size_t kParameterHandleCount =",
        "    sizeof(kParameterHandles) / sizeof(kParameterHandles[0]);",
        "",
        *role_handles,
        "",
        f"inline constexpr std::int32_t kModeDisabled = {modes['disabled']};",
        f"inline constexpr std::int32_t kModeManual = {modes['manual']};",
        f"inline constexpr std::int32_t kModeAutomatic = {modes['automatic']};",
        f"inline constexpr std::uint32_t kSupportedBitrates[]{{{bitrates}}};",
        f"inline constexpr std::int32_t kMinimumLocalNodeId = {local_node['min']};",
        f"inline constexpr std::int32_t kMaximumLocalNodeId = {local_node['max']};",
        f"inline constexpr std::int32_t kMinimumMagnetometerNodeId = {magnetic_node['min']};",
        f"inline constexpr std::int32_t kMaximumMagnetometerNodeId = {magnetic_node['max']};",
        "",
        "constexpr bool mode_supported(std::int32_t mode) noexcept",
        "{",
        "    return mode == kModeDisabled || mode == kModeManual ||",
        "           mode == kModeAutomatic;",
        "}",
        "",
        "constexpr bool automatic_allocation_enabled(std::int32_t mode) noexcept",
        "{",
        "    return mode == kModeAutomatic;",
        "}",
        "",
        "constexpr bool bitrate_supported(std::uint32_t bitrate) noexcept",
        "{",
        "    for (const std::uint32_t candidate : kSupportedBitrates) {",
        "        if (candidate == bitrate) return true;",
        "    }",
        "    return false;",
        "}",
        "",
        "} // namespace dima::protocols::dronecan::generated",
        "",
    ])


def render_parameters(manifest: dict[str, Any]) -> str:
    """把 manifest 参数投影为 PX4 module YAML，供官方参数链统一处理。"""
    # YAML 仅在正式生成阶段加载；Make 的 manifest/source 发现路径保持纯标准库，
    # 这样首次 bootstrap host-tools 前也能构造依赖图。
    import yaml

    groups: dict[str, dict[str, Any]] = {}
    for parameter in manifest["parameters"]:
        description = {"short": parameter["description"][0]}
        if len(parameter["description"]) > 1:
            description["long"] = "\n\n".join(parameter["description"][1:])
        definition: dict[str, Any] = {
            "description": description,
            "type": (
                "enum" if parameter.get("values")
                else "int32" if parameter["type"] == "INT32"
                else "float"
            ),
            "default": parameter["default"],
        }
        for field in ("min", "max", "unit"):
            if field in parameter:
                definition[field] = parameter[field]
        if parameter.get("values"):
            definition["values"] = {
                value["value"]: value["name"]
                for value in parameter["values"]
            }
        groups.setdefault(parameter["group"], {})[parameter["name"]] = definition

    document = {
        "module_name": "dronecan",
        "parameters": [
            {"group": group, "definitions": definitions}
            for group, definitions in groups.items()
        ],
    }
    return yaml.safe_dump(
        document, sort_keys=False, allow_unicode=True, default_flow_style=False
    )


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
        for path in sorted(entry for entry in root.rglob("*") if entry.is_file())
        if path.relative_to(root).as_posix() not in excluded
    }


def render_make_fragment(output_relative: str, generated: pathlib.Path) -> str:
    sources = [
        f"{output_relative}/{path.relative_to(generated).as_posix()}"
        for path in sorted((generated / "src").glob("*.c"))
    ]
    outputs = [
        f"{output_relative}/{path.relative_to(generated).as_posix()}"
        for path in sorted(entry for entry in generated.rglob("*") if entry.is_file())
    ]
    outputs.append(f"{output_relative}/.generated.json")
    lines = [
        "# Generated from dronecan_contract.json. DO NOT EDIT.",
        *make_list("DIMA_DRONECAN_GENERATED_C_SOURCES", sources),
        f"DIMA_DRONECAN_PARAMETER_YAML := {output_relative}/module_dronecan.yaml",
        f"DIMA_DRONECAN_PROTOCOL_HEADER := {output_relative}/DroneCanContract.hpp",
        f"DIMA_DRONECAN_PARAMETER_HEADER := {output_relative}/DroneCanParameterContract.hpp",
        f"DIMA_DRONECAN_GENERATED_STAMP := {output_relative}/.generated.json",
        *make_list("DIMA_DRONECAN_GENERATED_OUTPUTS", sorted(set(outputs))),
        "",
    ]
    return "\n".join(lines)


def write_text(path: pathlib.Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(content)


def build_generated_tree(
    manifest: dict[str, Any],
    manifest_path: pathlib.Path,
    root: pathlib.Path,
    generated: pathlib.Path,
    logical_output: pathlib.Path,
    cache_root: pathlib.Path,
) -> None:
    """在候选目录构建 codec、协议/参数合同、Make 片段和完整输出散列目录。"""
    compiler_root = provision_compiler(manifest, cache_root)
    site_packages = provision_python_packages(manifest, cache_root)
    run_dsdl_compiler(
        manifest, root, generated, compiler_root, site_packages
    )
    descriptors = role_descriptors(manifest, generated)
    write_text(
        generated / "DroneCanContract.hpp",
        render_protocol_contract(manifest, descriptors, generated),
    )
    write_text(
        generated / "DroneCanParameterContract.hpp",
        render_parameter_contract(manifest),
    )
    write_text(generated / "module_dronecan.yaml", render_parameters(manifest))
    try:
        output_relative = logical_output.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as error:
        raise GenerationError("DroneCAN generated output must remain inside the repository") from error
    make_path = generated / "dronecan_sources.mk"
    write_text(make_path, render_make_fragment(output_relative, generated))

    catalog = {
        "contract": manifest["contract"],
        "format_version": manifest["format_version"],
        "manifest_sha256": file_sha256(manifest_path),
        "upstream": manifest["upstream"],
        "dsdl": [
            {
                "path": item["path"],
                "sha256": item["sha256"],
                "type": item["type"],
            }
            for item in manifest["dsdl"]["files"]
        ],
        "messages": [
            {
                "allow_anonymous": item["allow_anonymous"],
                "id": item["id"],
                "owner": item["owner"],
                "role": item["role"],
                "signature": f"0x{item['signature']:016X}",
                "transfers": item["transfers"],
                "type": item["type"],
            }
            for item in descriptors
        ],
        "parameters": [parameter["name"] for parameter in manifest["parameters"]],
        "output_sha256": tree_hashes(generated),
    }
    write_text(
        generated / ".generated.json",
        json.dumps(catalog, indent=2, sort_keys=True) + "\n",
    )


def generated_tree_current(candidate: pathlib.Path, destination: pathlib.Path) -> bool:
    return destination.is_dir() and tree_hashes(candidate) == tree_hashes(destination)


def generate(
    manifest: dict[str, Any],
    manifest_path: pathlib.Path,
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
            manifest, manifest_path, root, candidate, output, cache_root
        )
        if generated_tree_current(candidate, output):
            print(f"DroneCAN contract already current: {output}")
            return
        install_directory(candidate, output)
    print(
        f"Generated {len([item for item in manifest['dsdl']['files'] if 'role' in item])} "
        f"DroneCAN root types and {len(manifest['parameters'])} parameters into {output}"
    )


def verify_generated(
    manifest: dict[str, Any], manifest_path: pathlib.Path, output: pathlib.Path
) -> None:
    """验证 manifest pin、上游身份与每个派生文件散列，verify 模式不写文件。"""
    catalog_path = output / ".generated.json"
    try:
        catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise GenerationError(f"unable to read generated DroneCAN catalog: {error}") from error
    if (
        catalog.get("contract") != manifest["contract"]
        or catalog.get("manifest_sha256") != file_sha256(manifest_path)
        or catalog.get("upstream") != manifest["upstream"]
    ):
        raise GenerationError("generated DroneCAN catalog does not match the manifest")
    expected_hashes = catalog.get("output_sha256")
    if not isinstance(expected_hashes, dict):
        raise GenerationError("generated DroneCAN catalog lacks output hashes")
    actual_hashes = tree_hashes(output, exclude=(".generated.json",))
    if actual_hashes != expected_hashes:
        missing = sorted(set(expected_hashes) - set(actual_hashes))
        unexpected = sorted(set(actual_hashes) - set(expected_hashes))
        changed = sorted(
            path for path in set(expected_hashes) & set(actual_hashes)
            if expected_hashes[path] != actual_hashes[path]
        )
        raise GenerationError(
            "generated DroneCAN output differs from its catalog: "
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
    input_mode.add_argument("--manifest", type=pathlib.Path)
    input_mode.add_argument("--find-manifest", type=pathlib.Path)
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
    if arguments.find_manifest is not None:
        if (arguments.output is not None or arguments.print_inputs or
                arguments.verify):
            raise GenerationError(
                "--find-manifest cannot be combined with generation modes"
            )
        manifest_path, root = discover_manifest(arguments.find_manifest)
        print(manifest_path.relative_to(root).as_posix())
        return 0
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
    if arguments.manifest is None:
        raise GenerationError("--manifest is required for generation")
    manifest_path = arguments.manifest.resolve()
    manifest, root = parse_manifest(manifest_path)
    if arguments.print_inputs:
        if arguments.output is not None or arguments.verify:
            raise GenerationError("--print-inputs cannot be combined with output modes")
        print(" ".join(dsdl_input_paths(manifest)))
        return 0
    if arguments.output is None:
        raise GenerationError("--output is required unless --print-inputs is used")
    output = arguments.output
    if not output.is_absolute():
        output = root / output
    if arguments.verify:
        verify_generated(manifest, manifest_path, output.resolve())
    else:
        generate(
            manifest,
            manifest_path,
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
