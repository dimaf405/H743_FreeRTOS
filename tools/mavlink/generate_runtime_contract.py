#!/usr/bin/env python3
"""Validate the MAVLink runtime policy and derive its C++ dispatch contract.

Wire definitions are intentionally outside this generator: Make invokes the
pinned upstream ``mavgen.py`` first, and this script only consumes the symbols
that mavgen emitted from ``dima.xml``. It never calculates message IDs, CRCs,
field layouts, payload sizes, encoders or decoders.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import shutil
import sys
import time
from typing import Any

import yaml


sys.dont_write_bytecode = True

ROOT = pathlib.Path(__file__).resolve().parents[2]
MESSAGE_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")
HANDLER_NAME = re.compile(r"^[A-Z][A-Za-z0-9]*$")
MESSAGE_HEADER = re.compile(r"^mavlink_msg_([a-z0-9_]+)\.h$")
POLICY_KEYS = frozenset({"version", "outbound", "inbound"})
OUTBOUND_KEYS = frozenset({
    "message",
    "handler",
    "requestable",
    "scheduler",
    "tx_stage",
    "default_interval_us",
    "interval_configurable",
})
INBOUND_KEYS = frozenset({"message", "handler"})


class ContractError(RuntimeError):
    """A deterministic runtime-policy or generated-closure failure."""


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def repository_path(path: pathlib.Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def write_utf8(path: pathlib.Path, content: str) -> None:
    """显式固定 UTF-8/LF，同时兼容项目仍可使用的 Python 3.9。"""
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(content)


def load_json(path: pathlib.Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f"unable to read JSON {path}: {error}") from error
    if not isinstance(document, dict):
        raise ContractError(f"JSON root must be an object: {path}")
    return document


def load_policy(path: pathlib.Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ContractError(f"unable to read runtime policy {path}: {error}") from error
    if not isinstance(document, dict) or document.get("version") != 1:
        raise ContractError("MAVLink runtime policy must use mapping schema version 1")
    unknown = set(document) - POLICY_KEYS
    if unknown:
        labels = sorted(repr(key) for key in unknown)
        raise ContractError(f"MAVLink runtime policy has unknown keys: {labels}")
    return document


def reject_unknown_keys(
    entry: dict[str, Any], allowed: frozenset[str], field: str
) -> None:
    """策略键必须显式受支持，避免拼写错误静默退化为可选字段默认值。"""
    unknown = set(entry) - allowed
    if unknown:
        labels = sorted(repr(key) for key in unknown)
        raise ContractError(f"{field} has unknown keys: {labels}")


def validate_locked_sources(lock: dict[str, Any], xml_paths: list[pathlib.Path]) -> None:
    """Verify the pinned upstream XML files without interpreting wire fields."""
    mavlink = lock.get("mavlink")
    if not isinstance(mavlink, dict):
        raise ContractError("MAVLink lock lacks the mavlink source object")
    expected = mavlink.get("xml_sha256")
    if not isinstance(expected, dict) or not expected:
        raise ContractError("MAVLink lock lacks upstream XML hashes")
    by_name = {path.name: path for path in xml_paths}
    if set(by_name) != set(expected):
        raise ContractError(
            "declared upstream XML closure differs from the lock: "
            f"declared={sorted(by_name)} locked={sorted(expected)}"
        )
    for name, expected_hash in expected.items():
        path = by_name[name]
        if not path.is_file():
            raise ContractError(f"missing pinned MAVLink XML: {path}")
        actual_hash = sha256(path)
        if actual_hash != expected_hash:
            raise ContractError(
                f"{name} SHA-256 mismatch: expected {expected_hash}, got {actual_hash}"
            )


def discover_generated_messages(generated: pathlib.Path) -> set[str]:
    """Discover message symbols from untouched mavgen headers.

    只从 mavgen 产物发现名称并验证宏存在；数值 ID 由生成头在
    C++ 编译期提供，Python 不解析、复制或重新编码它。
    """
    root_header = generated / "dima" / "mavlink.h"
    if not root_header.is_file():
        raise ContractError(f"mavgen output is missing the root dialect header: {root_header}")

    messages: set[str] = set()
    for path in sorted(generated.rglob("mavlink_msg_*.h")):
        match = MESSAGE_HEADER.fullmatch(path.name)
        if match is None:
            continue
        name = match.group(1).upper()
        macro = f"MAVLINK_MSG_ID_{name}"
        try:
            text = path.read_text(encoding="utf-8")
        except OSError as error:
            raise ContractError(f"unable to read generated header {path}: {error}") from error
        if re.search(rf"^#define\s+{re.escape(macro)}\s+", text, re.MULTILINE) is None:
            raise ContractError(f"generated header lacks {macro}: {path}")
        messages.add(name)
    if not messages:
        raise ContractError("mavgen output contains no message headers")
    return messages


def required_bool(entry: dict[str, Any], key: str, field: str) -> bool:
    value = entry.get(key)
    if not isinstance(value, bool):
        raise ContractError(f"{field}.{key} must be boolean")
    return value


def required_name(
    entry: dict[str, Any], key: str, field: str, pattern: re.Pattern[str]
) -> str:
    value = entry.get(key)
    if not isinstance(value, str) or pattern.fullmatch(value) is None:
        raise ContractError(f"{field}.{key} has an invalid symbol")
    return value


def normalize_outbound(
    policy: dict[str, Any], available: set[str]
) -> list[dict[str, Any]]:
    """把发送策略收敛为严格调度合同，所有消息符号必须来自 mavgen 产物。"""
    entries = policy.get("outbound")
    if not isinstance(entries, list) or not entries:
        raise ContractError("runtime policy outbound must be a non-empty list")

    normalized: list[dict[str, Any]] = []
    messages: set[str] = set()
    handlers: set[str] = set()
    heartbeat_count = 0
    for index, raw in enumerate(entries):
        field = f"outbound[{index}]"
        if not isinstance(raw, dict):
            raise ContractError(f"{field} must be a mapping")
        reject_unknown_keys(raw, OUTBOUND_KEYS, field)
        message = required_name(raw, "message", field, MESSAGE_NAME)
        handler = required_name(raw, "handler", field, HANDLER_NAME)
        if message not in available:
            raise ContractError(f"{field}.message is absent from dima.xml output: {message}")
        if message in messages:
            raise ContractError(f"duplicate outbound message: {message}")
        if handler in handlers:
            raise ContractError(f"duplicate outbound handler: {handler}")

        requestable = required_bool(raw, "requestable", field)
        configurable = raw.get("interval_configurable", False)
        if not isinstance(configurable, bool):
            raise ContractError(f"{field}.interval_configurable must be boolean")
        scheduler = raw.get("scheduler")
        if scheduler not in {"none", "heartbeat", "service"}:
            raise ContractError(f"{field}.scheduler is invalid")
        tx_stage = raw.get("tx_stage", "none")
        if tx_stage not in {"none", "pre_metadata", "post_metadata"}:
            raise ContractError(f"{field}.tx_stage is invalid")
        if scheduler == "service" and tx_stage == "none":
            raise ContractError(f"{field}.tx_stage is required for service scheduling")
        if scheduler != "service" and tx_stage != "none":
            raise ContractError(f"{field}.tx_stage requires service scheduling")

        default_interval = raw.get("default_interval_us", -1)
        if scheduler == "none":
            if default_interval != -1 or configurable:
                raise ContractError(f"{field} cannot configure an unscheduled interval")
        elif (
            isinstance(default_interval, bool)
            or not isinstance(default_interval, int)
            or default_interval <= 0
            or default_interval > 0x7FFFFFFF
        ):
            raise ContractError(f"{field}.default_interval_us is invalid")
        if scheduler == "heartbeat":
            heartbeat_count += 1
            if handler != "Heartbeat" or configurable:
                raise ContractError("heartbeat scheduler requires fixed Heartbeat handler")

        messages.add(message)
        handlers.add(handler)
        normalized.append(
            {
                "message": message,
                "handler": handler,
                "requestable": requestable,
                "scheduler": scheduler,
                "tx_stage": tx_stage,
                "default_interval_us": default_interval,
                "interval_configurable": configurable,
            }
        )
    if heartbeat_count != 1:
        raise ContractError("runtime policy must contain exactly one heartbeat scheduler")
    return normalized


def normalize_inbound(
    policy: dict[str, Any], available: set[str]
) -> list[dict[str, str]]:
    """校验接收消息与 handler 映射，不读取或复制任何 wire 数值常量。"""
    entries = policy.get("inbound")
    if not isinstance(entries, list) or not entries:
        raise ContractError("runtime policy inbound must be a non-empty list")
    normalized: list[dict[str, str]] = []
    messages: set[str] = set()
    for index, raw in enumerate(entries):
        field = f"inbound[{index}]"
        if not isinstance(raw, dict):
            raise ContractError(f"{field} must be a mapping")
        reject_unknown_keys(raw, INBOUND_KEYS, field)
        message = required_name(raw, "message", field, MESSAGE_NAME)
        handler = required_name(raw, "handler", field, HANDLER_NAME)
        if message not in available:
            raise ContractError(f"{field}.message is absent from dima.xml output: {message}")
        if message in messages:
            raise ContractError(f"duplicate inbound message: {message}")
        messages.add(message)
        normalized.append({"message": message, "handler": handler})
    return normalized


def render_contract(
    outbound: list[dict[str, Any]], inbound: list[dict[str, str]]
) -> str:
    handlers = ",\n".join(f"    {entry['handler']}" for entry in outbound)
    inbound_handlers = list(dict.fromkeys(entry["handler"] for entry in inbound))
    inbound_handler_text = ",\n".join(f"    {name}" for name in inbound_handlers)
    scheduler_names = {"none": "None", "heartbeat": "Heartbeat", "service": "Service"}
    stage_names = {"none": "None", "pre_metadata": "PreMetadata", "post_metadata": "PostMetadata"}

    descriptors = ",\n".join(
        "    {"
        f"MAVLINK_MSG_ID_{entry['message']}, "
        f"MessageHandler::{entry['handler']}, "
        f"Scheduler::{scheduler_names[entry['scheduler']]}, "
        f"TxStage::{stage_names[entry['tx_stage']]}, "
        f"{entry['default_interval_us']}, "
        f"{'true' if entry['requestable'] else 'false'}, "
        f"{'true' if entry['interval_configurable'] else 'false'}"
        "}"
        for entry in outbound
    )
    inbound_descriptors = ",\n".join(
        "    {"
        f"MAVLINK_MSG_ID_{entry['message']}, "
        f"InboundHandler::{entry['handler']}"
        "}"
        for entry in inbound
    )
    service_count = sum(entry["scheduler"] == "service" for entry in outbound)

    return f"""// Generated from dima.xml and mavlink_runtime.yaml. DO NOT EDIT.
#pragma once

#include <cstddef>
#include <cstdint>

#ifndef MAVLINK_MSG_ID_HEARTBEAT
#error "include MavlinkBridge.h before mavlink_stream_contract.hpp"
#endif

namespace dima::generated::mavlink_streams {{

enum class MessageHandler : std::uint8_t {{
{handlers}
}};

enum class InboundHandler : std::uint8_t {{
{inbound_handler_text}
}};

enum class Scheduler : std::uint8_t {{
    None,
    Heartbeat,
    Service,
}};

enum class TxStage : std::uint8_t {{
    None,
    PreMetadata,
    PostMetadata,
}};

struct MessageContract {{
    std::uint32_t message_id;
    MessageHandler handler;
    Scheduler scheduler;
    TxStage tx_stage;
    std::int32_t default_interval_us;
    bool requestable;
    bool interval_configurable;
}};

struct InboundMessageContract {{
    std::uint32_t message_id;
    InboundHandler handler;
}};

// ID 始终引用 mavgen 宏，运行策略不复制 wire 常量。
inline constexpr MessageContract kMessages[]{{
{descriptors}
}};
inline constexpr std::size_t kMessageCount =
    sizeof(kMessages) / sizeof(kMessages[0]);
inline constexpr std::size_t kServiceStreamCount = {service_count}U;
inline constexpr std::size_t kInvalidServiceIndex = kServiceStreamCount;

inline constexpr InboundMessageContract kInboundMessages[]{{
{inbound_descriptors}
}};
inline constexpr std::size_t kInboundMessageCount =
    sizeof(kInboundMessages) / sizeof(kInboundMessages[0]);

constexpr const MessageContract *find_message(std::uint32_t message_id) noexcept
{{
    for (const MessageContract &message : kMessages) {{
        if (message.message_id == message_id) return &message;
    }}
    return nullptr;
}}

constexpr const MessageContract *find_handler(MessageHandler handler) noexcept
{{
    for (const MessageContract &message : kMessages) {{
        if (message.handler == handler) return &message;
    }}
    return nullptr;
}}

constexpr const InboundMessageContract *find_inbound_message(
    std::uint32_t message_id) noexcept
{{
    for (const InboundMessageContract &message : kInboundMessages) {{
        if (message.message_id == message_id) return &message;
    }}
    return nullptr;
}}

constexpr std::size_t service_index(MessageHandler handler) noexcept
{{
    std::size_t index = 0U;
    for (const MessageContract &message : kMessages) {{
        if (message.scheduler == Scheduler::Service) {{
            if (message.handler == handler) return index;
            ++index;
        }}
    }}
    return kInvalidServiceIndex;
}}

constexpr std::int32_t default_interval_us(MessageHandler handler) noexcept
{{
    const MessageContract *message = find_handler(handler);
    return message != nullptr ? message->default_interval_us : -1;
}}

}} // namespace dima::generated::mavlink_streams
"""


def tree_hashes(root: pathlib.Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): sha256(path)
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.name != ".generated.json"
    }


def write_manifest(
    generated: pathlib.Path,
    dialect: pathlib.Path,
    upstream_xml: list[pathlib.Path],
    policy: pathlib.Path,
    lock_path: pathlib.Path,
    lock: dict[str, Any],
    available: set[str],
    outbound: list[dict[str, Any]],
    inbound: list[dict[str, str]],
) -> None:
    inputs = [
        dialect,
        *upstream_xml,
        policy,
        lock_path,
        pathlib.Path(__file__),
        ROOT / "tools/mavlink/bootstrap_pymavlink.py",
    ]
    document = {
        "format": 1,
        "dialect": "dima",
        "mavlink_commit": lock["mavlink"]["commit"],
        "pymavlink_commit": lock["pymavlink"]["commit"],
        "pymavlink_version": lock["pymavlink"]["version"],
        "dialect_message_count": len(available),
        "runtime_outbound_count": len(outbound),
        "runtime_inbound_count": len(inbound),
        "inputs": {repository_path(path): sha256(path) for path in inputs},
        "outputs": tree_hashes(generated),
    }
    write_utf8(
        generated / ".generated.json",
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )


def verify_tree(candidate: pathlib.Path, expected: pathlib.Path) -> None:
    if not expected.is_dir():
        raise ContractError(f"generated MAVLink output is unavailable: {expected}")
    candidate_hashes = {
        path.relative_to(candidate).as_posix(): sha256(path)
        for path in sorted(candidate.rglob("*"))
        if path.is_file()
    }
    expected_hashes = {
        path.relative_to(expected).as_posix(): sha256(path)
        for path in sorted(expected.rglob("*"))
        if path.is_file()
    }
    if candidate_hashes != expected_hashes:
        missing = sorted(set(candidate_hashes) - set(expected_hashes))
        extra = sorted(set(expected_hashes) - set(candidate_hashes))
        changed = sorted(
            path
            for path in set(candidate_hashes) & set(expected_hashes)
            if candidate_hashes[path] != expected_hashes[path]
        )
        raise ContractError(
            "MAVLink regeneration differs from installed output: "
            f"missing={missing} extra={extra} changed={changed}"
        )


def replace_with_retry(source: pathlib.Path, destination: pathlib.Path) -> None:
    attempts = 6 if os.name == "nt" else 1
    for attempt in range(attempts):
        try:
            os.replace(source, destination)
            return
        except PermissionError as error:
            if attempt + 1 == attempts:
                raise ContractError(
                    f"unable to replace {destination} after {attempts} attempts: {error}"
                ) from error
            time.sleep(0.05 * (attempt + 1))


def install_tree(staging: pathlib.Path, destination: pathlib.Path) -> None:
    """原子替换整棵生成树，失败时恢复旧版，避免混合 wire 头。"""
    destination.parent.mkdir(parents=True, exist_ok=True)
    backup = destination.with_name(destination.name + ".previous")
    if backup.exists():
        shutil.rmtree(backup)
    had_destination = destination.exists()
    if had_destination:
        replace_with_retry(destination, backup)
    try:
        replace_with_retry(staging, destination)
    except Exception:
        if had_destination and backup.exists() and not destination.exists():
            replace_with_retry(backup, destination)
        raise
    if backup.exists():
        shutil.rmtree(backup)


def generate(arguments: argparse.Namespace) -> None:
    """在 mavgen 临时树追加薄合同，再整树验证或原子安装。"""
    staging = arguments.generated_dir.resolve()
    if not staging.is_dir():
        raise ContractError(f"mavgen staging directory does not exist: {staging}")
    dialect = arguments.dialect.resolve()
    policy_path = arguments.policy.resolve()
    lock_path = arguments.lock.resolve()
    upstream_xml = [path.resolve() for path in arguments.upstream_xml]

    lock = load_json(lock_path)
    validate_locked_sources(lock, upstream_xml)
    available = discover_generated_messages(staging)
    policy = load_policy(policy_path)
    outbound = normalize_outbound(policy, available)
    inbound = normalize_inbound(policy, available)

    contract_path = staging / "mavlink_stream_contract.hpp"
    write_utf8(contract_path, render_contract(outbound, inbound))
    write_manifest(
        staging,
        dialect,
        upstream_xml,
        policy_path,
        lock_path,
        lock,
        available,
        outbound,
        inbound,
    )

    if arguments.verify:
        verify_tree(staging, arguments.output_dir.resolve())
        shutil.rmtree(staging)
        print(f"MAVLink generation verification passed: {arguments.output_dir.resolve()}")
    else:
        install_tree(staging, arguments.output_dir.resolve())
        print(
            f"generated {len(available)}-message dima dialect and "
            f"{len(outbound)}/{len(inbound)} runtime routes into "
            f"{arguments.output_dir.resolve()}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--generated-dir", type=pathlib.Path, required=True)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    parser.add_argument("--dialect", type=pathlib.Path, required=True)
    parser.add_argument("--upstream-xml", type=pathlib.Path, action="append", required=True)
    parser.add_argument("--policy", type=pathlib.Path, required=True)
    parser.add_argument("--lock", type=pathlib.Path, required=True)
    parser.add_argument("--verify", action="store_true")
    return parser.parse_args()


def main() -> int:
    try:
        generate(parse_args())
    except ContractError as error:
        print(f"MAVLink runtime generation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
