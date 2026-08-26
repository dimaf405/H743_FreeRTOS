#!/usr/bin/env python3
"""从单一固件身份 manifest 生成 firmware 与上传工具共享的版本合同。"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import re
import sys


class ContractError(RuntimeError):
    """The authoritative firmware identity manifest is invalid."""


FIRMWARE_TYPE_VALUES = {
    "dev": 0,
    "alpha": 64,
    "beta": 128,
    "rc": 192,
    "official": 255,
}
IMAGE_VERSION_PATTERN = re.compile(
    r"^(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\."
    r"(?:0|[1-9][0-9]*)(?:\+(?:0|[1-9][0-9]*))?$"
)
GIT_COMMIT_PATTERN = re.compile(r"^[0-9a-fA-F]{40}(?:[0-9a-fA-F]{24})?$")


def load_manifest(path: pathlib.Path) -> dict:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f"unable to load firmware identity manifest: {error}") from error
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise ContractError("firmware identity schema_version must be 1")
    return manifest


def require_uint(value: object, field: str, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError(f"{field} must be an integer")
    if value < 0 or value > maximum:
        raise ContractError(f"{field} must be in range 0..{maximum}")
    return value


def manifest_values(manifest: dict) -> tuple[str, str, int, int, int, int, str, str]:
    product = manifest.get("product")
    if not isinstance(product, str) or not product.strip():
        raise ContractError("product must be a non-empty string")
    board_version = require_uint(manifest.get("board_version"), "board_version", 0xFFFFFFFF)

    mcuboot = manifest.get("mcuboot")
    if not isinstance(mcuboot, dict):
        raise ContractError("mcuboot must be an object")
    image_version = mcuboot.get("image_version")
    if not isinstance(image_version, str) or not IMAGE_VERSION_PATTERN.fullmatch(image_version):
        raise ContractError("mcuboot.image_version must be an imgtool semantic version")

    mavlink = manifest.get("mavlink")
    if not isinstance(mavlink, dict):
        raise ContractError("mavlink must be an object")
    version = mavlink.get("flight_version")
    if not isinstance(version, dict):
        raise ContractError("mavlink.flight_version must be an object")
    major = require_uint(version.get("major"), "flight_version.major", 255)
    minor = require_uint(version.get("minor"), "flight_version.minor", 255)
    patch = require_uint(version.get("patch"), "flight_version.patch", 255)
    version_type = version.get("type")
    if version_type not in FIRMWARE_TYPE_VALUES:
        raise ContractError(
            "flight_version.type must be one of "
            + ", ".join(sorted(FIRMWARE_TYPE_VALUES))
        )

    compatibility = mavlink.get("px4_compatibility")
    if not isinstance(compatibility, dict):
        raise ContractError("mavlink.px4_compatibility must be an object")
    compatibility_version = compatibility.get("version")
    compatibility_commit = compatibility.get("commit")
    expected_version = f"{major}.{minor}.{patch}"
    if compatibility_version != expected_version:
        raise ContractError(
            "PX4 compatibility version must match the advertised flight version"
        )
    if not isinstance(compatibility_commit, str) or not GIT_COMMIT_PATTERN.fullmatch(
        compatibility_commit
    ):
        raise ContractError("PX4 compatibility commit must be a full Git object ID")
    return (
        product.strip(),
        image_version,
        board_version,
        major,
        minor,
        patch,
        version_type,
        compatibility_commit.lower(),
    )


def make_contract(manifest: dict, git_commit: str) -> dict:
    """编码 MAVLink flight version，并从实际 Git 对象 ID 推导 custom version。"""
    if not GIT_COMMIT_PATTERN.fullmatch(git_commit):
        raise ContractError("--git-commit must be a full SHA-1 or SHA-256 object ID")
    git_commit = git_commit.lower()
    (
        product,
        image_version,
        board_version,
        major,
        minor,
        patch,
        version_type,
        compatibility_commit,
    ) = manifest_values(manifest)
    type_value = FIRMWARE_TYPE_VALUES[version_type]
    flight_sw_version = (
        (major << 24) | (minor << 16) | (patch << 8) | type_value
    )
    git_hash_prefix = git_commit[:16]
    # PX4 在小端目标上 memcpy uint64 hash；QGC 再反转 8 个线字节还原前 16 个十六进制字符。
    custom_version = list(bytes.fromhex(git_hash_prefix)[::-1])
    return {
        "schema_version": 1,
        "product": product,
        "board_version": board_version,
        "mcuboot_image_version": image_version,
        "mavlink": {
            "flight_version": {
                "major": major,
                "minor": minor,
                "patch": patch,
                "type": version_type,
                "type_value": type_value,
                "encoded": flight_sw_version,
            },
            "flight_custom_version": custom_version,
            "git_commit": git_commit,
            "git_hash_prefix": git_hash_prefix,
            "px4_compatibility_commit": compatibility_commit,
        },
    }


def render_header(contract: dict) -> bytes:
    mavlink = contract["mavlink"]
    version = mavlink["flight_version"]
    custom = ", ".join(f"0x{value:02x}U" for value in mavlink["flight_custom_version"])
    content = f"""// Generated by tools/firmware/generate_identity_contract.py. Do not edit.
#pragma once

#include <cstdint>

namespace dima::generated::firmware_identity {{

inline constexpr std::uint8_t kFlightVersionMajor = {version['major']}U;
inline constexpr std::uint8_t kFlightVersionMinor = {version['minor']}U;
inline constexpr std::uint8_t kFlightVersionPatch = {version['patch']}U;
inline constexpr std::uint8_t kFlightVersionType = {version['type_value']}U;
inline constexpr std::uint32_t kFlightSoftwareVersion = 0x{version['encoded']:08x}U;
inline constexpr std::uint32_t kBoardVersion = {contract['board_version']}U;
inline constexpr std::uint8_t kFlightCustomVersion[8]{{{custom}}};
inline constexpr char kGitCommit[] = "{mavlink['git_commit']}";
inline constexpr char kPx4CompatibilityCommit[] =
    "{mavlink['px4_compatibility_commit']}";

}} // namespace dima::generated::firmware_identity
"""
    return content.encode("utf-8")


def render_json(contract: dict) -> bytes:
    return (json.dumps(contract, indent=2, sort_keys=True) + "\n").encode("utf-8")


def write_if_changed(path: pathlib.Path, content: bytes) -> None:
    if path.is_file() and path.read_bytes() == content:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_bytes(content)
    temporary.replace(path)


def output_files(output: pathlib.Path, contract: dict) -> dict[pathlib.Path, bytes]:
    return {
        output / "FirmwareIdentityContract.hpp": render_header(contract),
        output / "firmware_identity_contract.json": render_json(contract),
    }


def generate(output: pathlib.Path, contract: dict, verify: bool) -> None:
    """生成/验证 C++ 与 JSON 两种同源表示，verify 模式绝不改写文件。"""
    files = output_files(output, contract)
    if verify:
        mismatches = [
            str(path)
            for path, expected in files.items()
            if not path.is_file() or path.read_bytes() != expected
        ]
        if mismatches:
            raise ContractError(
                "generated firmware identity is stale: " + ", ".join(mismatches)
            )
        print(f"verified generated firmware identity in {output}")
        return
    for path, content in files.items():
        write_if_changed(path, content)
    digest = hashlib.sha256(files[output / "FirmwareIdentityContract.hpp"]).hexdigest()
    print(
        "generated firmware identity "
        f"version=0x{contract['mavlink']['flight_version']['encoded']:08x} "
        f"git={contract['mavlink']['git_hash_prefix']} header_sha256={digest}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--git-commit")
    parser.add_argument("--print-image-version", action="store_true")
    parser.add_argument("--verify", action="store_true")
    arguments = parser.parse_args()
    manifest = load_manifest(arguments.manifest)
    if arguments.print_image_version:
        print(manifest_values(manifest)[1])
        return 0
    if arguments.output is None or arguments.git_commit is None:
        parser.error("--output and --git-commit are required for generation")
    contract = make_contract(manifest, arguments.git_commit)
    generate(arguments.output, contract, arguments.verify)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ContractError as error:
        print(f"firmware identity generation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
