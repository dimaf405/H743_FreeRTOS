#!/usr/bin/env python3
"""生成可复现的 Dima MAVLink v2 dialect 与运行时消息合同。

消息、枚举、命令和流调度均以 ``mavlink.lock.json`` 为权威输入；本文件只负责
校验、裁剪和生成，不能再维护第二份消息列表或 ID 映射。
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True

from bootstrap_pymavlink import (
    BootstrapError,
    file_sha256,
    load_lock,
    provision_pymavlink,
)


IDENTIFIER_PATTERN = re.compile(r"^[A-Z][A-Za-z0-9]*$")


class GenerationError(RuntimeError):
    """A deterministic dialect generation or validation failure."""


def install_generated_tree(source: pathlib.Path,
                           destination: pathlib.Path) -> None:
    """原子安装生成树，并对 Windows 杀毒/索引造成的短暂句柄占用做有界重试。"""
    attempts = 5 if os.name == "nt" else 1
    for attempt in range(attempts):
        try:
            os.replace(source, destination)
            return
        except PermissionError as error:
            if attempt + 1 >= attempts:
                raise GenerationError(
                    "unable to install generated MAVLink tree after "
                    f"{attempts} attempts: {error}"
                ) from error
            time.sleep(0.05 * (attempt + 1))


def extract(
    source_path: pathlib.Path,
    messages_from_common: set[str],
    enums_from_common_full: set[str],
    mav_cmd_entries: set[str],
) -> tuple[list[ET.Element], list[ET.Element]]:
    """按 lock 传入的集合从 pinned common.xml 提取消息和枚举。

    ``MAV_CMD`` 只复制合同列出的 entry；其余完整枚举按名称复制。调用方传入
    集合而非模块常量，确保 XML 裁剪与运行时合同共用同一权威来源。
    """
    root = ET.parse(source_path).getroot()
    messages = [
        copy.deepcopy(message)
        for message in root.findall("./messages/message")
        if message.get("name") in messages_from_common
    ]

    enums: list[ET.Element] = []
    for enum in root.findall("./enums/enum"):
        name = enum.get("name")
        if name in enums_from_common_full:
            enums.append(copy.deepcopy(enum))
        elif name == "MAV_CMD":
            filtered = ET.Element("enum", enum.attrib)
            description = enum.find("description")
            if description is not None:
                filtered.append(copy.deepcopy(description))
            for entry in enum.findall("entry"):
                if entry.get("name") in mav_cmd_entries:
                    filtered.append(copy.deepcopy(entry))
            enums.append(filtered)
    return messages, enums


def indent(element: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = pad + "  "
        for child in element:
            indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = pad + "  "
        element[-1].tail = pad


def validate_inputs(xml_dir: pathlib.Path, lock: dict) -> None:
    """在解析前核对每个上游 XML 的 SHA-256，拒绝静默版本漂移。"""
    expected = lock["mavlink"]["xml_sha256"]
    for filename, expected_sha in expected.items():
        path = xml_dir / filename
        if not path.is_file():
            raise GenerationError(f"missing pinned MAVLink XML: {path}")
        actual_sha = file_sha256(path)
        if actual_sha != expected_sha:
            raise GenerationError(
                f"{filename} SHA-256 mismatch: expected {expected_sha}, got {actual_sha}"
            )


def source_message_names(path: pathlib.Path) -> set[str]:
    return {
        message.get("name", "")
        for message in ET.parse(path).getroot().findall("./messages/message")
    }


def required_string_set(mapping: dict, key: str) -> set[str]:
    """读取非空且无重复的字符串数组，同时消除后续查找的顺序依赖。"""
    values = mapping.get(key)
    if not isinstance(values, list) or not values:
        raise GenerationError(f"dialect.{key} must be a non-empty array")
    if any(not isinstance(value, str) or not value for value in values):
        raise GenerationError(f"dialect.{key} entries must be non-empty strings")
    result = set(values)
    if len(result) != len(values):
        raise GenerationError(f"dialect.{key} contains duplicate entries")
    return result


def source_message_ids(paths: tuple[pathlib.Path, ...]) -> dict[str, int]:
    """从 pinned XML 解析 name -> ID，并拒绝跨 dialect 的同名异号。"""
    identifiers: dict[str, int] = {}
    for path in paths:
        for message in ET.parse(path).getroot().findall("./messages/message"):
            name = message.get("name")
            identifier = message.get("id")
            if not name or identifier is None:
                continue
            value = int(identifier)
            previous = identifiers.get(name)
            if previous is not None and previous != value:
                raise GenerationError(f"message {name} has conflicting IDs")
            identifiers[name] = value
    return identifiers


def validate_runtime_contract(
    lock: dict, complete: set[str], message_ids: dict[str, int]
) -> list[dict]:
    """把 lock 中的消息流声明规范化为唯一、可生成的运行时描述符。

    scheduler 决定谁产生周期消息，tx_stage 决定 Service 流在 metadata 前后
    的发送阶段；无调度器的消息不得携带周期配置。这里校验的是合同闭包，
    C++ 侧只消费生成结果，不再手写 request/interval/dispatch 清单。
    """
    runtime = lock.get("runtime")
    if not isinstance(runtime, dict):
        raise GenerationError("runtime must be an object")
    messages = runtime.get("messages")
    if not isinstance(messages, list) or not messages:
        raise GenerationError("runtime.messages must be a non-empty array")

    normalized: list[dict] = []
    names: set[str] = set()
    handlers: set[str] = set()
    heartbeat_count = 0
    for index, entry in enumerate(messages):
        field = f"runtime.messages[{index}]"
        if not isinstance(entry, dict):
            raise GenerationError(f"{field} must be an object")
        name = entry.get("name")
        handler = entry.get("handler")
        requestable = entry.get("requestable")
        scheduler = entry.get("scheduler")
        tx_stage = entry.get("tx_stage", "none")
        if not isinstance(name, str) or name not in complete:
            raise GenerationError(f"{field}.name is not in the generated dialect")
        if name in names:
            raise GenerationError(f"duplicate runtime message {name}")
        if not isinstance(handler, str) or not IDENTIFIER_PATTERN.fullmatch(handler):
            raise GenerationError(f"{field}.handler is not a C++ identifier")
        if handler in handlers:
            raise GenerationError(f"duplicate runtime handler {handler}")
        if not isinstance(requestable, bool):
            raise GenerationError(f"{field}.requestable must be boolean")
        if scheduler not in {"none", "heartbeat", "service"}:
            raise GenerationError(f"{field}.scheduler is invalid")
        if tx_stage not in {"none", "pre_metadata", "post_metadata"}:
            raise GenerationError(f"{field}.tx_stage is invalid")
        if scheduler == "service" and tx_stage == "none":
            raise GenerationError(f"{field}.tx_stage is required for service streams")
        if scheduler != "service" and tx_stage != "none":
            raise GenerationError(f"{field}.tx_stage requires the service scheduler")
        if name not in message_ids:
            raise GenerationError(f"unable to resolve MAVLink message ID for {name}")

        default_interval = entry.get("default_interval_us", -1)
        configurable = entry.get("interval_configurable", False)
        if not isinstance(configurable, bool):
            raise GenerationError(f"{field}.interval_configurable must be boolean")
        # -1 只表示“没有周期调度”；真正的周期必须是正数且可装入 int32_t，
        # 以便生成头与 MAV_CMD_SET_MESSAGE_INTERVAL 使用同一线协议范围。
        if scheduler == "none":
            if default_interval != -1 or configurable:
                raise GenerationError(
                    f"{field} cannot configure an interval without a scheduler"
                )
        else:
            if (
                isinstance(default_interval, bool)
                or not isinstance(default_interval, int)
                or default_interval <= 0
                or default_interval > 0x7FFFFFFF
            ):
                raise GenerationError(f"{field}.default_interval_us is invalid")
        if scheduler == "heartbeat":
            heartbeat_count += 1
            if handler != "Heartbeat" or configurable:
                raise GenerationError(
                    "the heartbeat scheduler requires fixed Heartbeat handler"
                )

        names.add(name)
        handlers.add(handler)
        normalized.append(
            {
                "name": name,
                "message_id": message_ids[name],
                "handler": handler,
                "requestable": requestable,
                "scheduler": scheduler,
                "tx_stage": tx_stage,
                "default_interval_us": default_interval,
                "interval_configurable": configurable,
            }
        )
    if heartbeat_count != 1:
        raise GenerationError("runtime contract must contain exactly one heartbeat")
    return normalized


def validate_inbound_contract(
    lock: dict, complete: set[str], message_ids: dict[str, int]
) -> list[dict]:
    """验证由 lock 声明的接收消息 -> 业务 handler 路由。"""
    runtime = lock["runtime"]
    inbound = runtime.get("inbound")
    if not isinstance(inbound, list) or not inbound:
        raise GenerationError("runtime.inbound must be a non-empty array")

    normalized: list[dict] = []
    names: set[str] = set()
    for index, entry in enumerate(inbound):
        field = f"runtime.inbound[{index}]"
        if not isinstance(entry, dict):
            raise GenerationError(f"{field} must be an object")
        name = entry.get("name")
        handler = entry.get("handler")
        if not isinstance(name, str) or name not in complete:
            raise GenerationError(f"{field}.name is not in the generated dialect")
        if name in names:
            raise GenerationError(f"duplicate inbound message {name}")
        if not isinstance(handler, str) or not IDENTIFIER_PATTERN.fullmatch(handler):
            raise GenerationError(f"{field}.handler is not a C++ identifier")
        if name not in message_ids:
            raise GenerationError(f"unable to resolve MAVLink message ID for {name}")
        names.add(name)
        normalized.append(
            {"name": name, "message_id": message_ids[name], "handler": handler}
        )
    return normalized


def validate_allowlist(
    xml_dir: pathlib.Path, lock: dict
) -> tuple[list[dict], list[dict]]:
    """验证完整 dialect 集合、禁止消息和运行时流合同彼此闭合。"""
    dialect = lock.get("dialect")
    if not isinstance(dialect, dict):
        raise GenerationError("dialect must be an object")
    messages_from_common = required_string_set(dialect, "messages_from_common")
    inherited_messages = required_string_set(dialect, "inherited_messages")
    required_string_set(dialect, "enums_from_common_full")
    required_string_set(dialect, "mav_cmd_entries")
    common_names = source_message_names(xml_dir / "common.xml")
    missing = messages_from_common - common_names
    if missing:
        raise GenerationError(f"messages missing from common.xml: {sorted(missing)}")

    inherited = (
        source_message_names(xml_dir / "standard.xml")
        | source_message_names(xml_dir / "minimal.xml")
    )
    if inherited != inherited_messages:
        raise GenerationError(
            "pinned standard/minimal inherited message set changed: "
            f"expected {sorted(inherited_messages)}, got {sorted(inherited)}"
        )

    complete = messages_from_common | inherited
    expected_count = lock["dialect"]["message_count"]
    if len(complete) != expected_count:
        raise GenerationError(
            f"dialect has {len(complete)} messages, expected {expected_count}"
        )
    forbidden = set(lock["dialect"]["forbidden_messages"])
    present_forbidden = complete & forbidden
    if present_forbidden:
        raise GenerationError(
            f"forbidden messages entered dialect: {sorted(present_forbidden)}"
        )
    message_ids = source_message_ids(
        (
            xml_dir / "common.xml",
            xml_dir / "standard.xml",
            xml_dir / "minimal.xml",
        )
    )
    return (
        validate_runtime_contract(lock, complete, message_ids),
        validate_inbound_contract(lock, complete, message_ids),
    )


def write_dialect(xml_dir: pathlib.Path, output_path: pathlib.Path, lock: dict) -> None:
    """生成仅 include standard.xml 的 dima.xml，并固定消息输出顺序与换行。"""
    dialect = lock["dialect"]
    messages, enums = extract(
        xml_dir / "common.xml",
        set(dialect["messages_from_common"]),
        set(dialect["enums_from_common_full"]),
        set(dialect["mav_cmd_entries"]),
    )
    root = ET.Element("mavlink")
    ET.SubElement(root, "include").text = "standard.xml"
    ET.SubElement(root, "version").text = "0"
    ET.SubElement(root, "dialect").text = "0"
    enums_node = ET.SubElement(root, "enums")
    for enum in enums:
        enums_node.append(enum)
    messages_node = ET.SubElement(root, "messages")
    for message in sorted(messages, key=lambda item: int(item.get("id", "0"))):
        messages_node.append(message)
    indent(root)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    content = output_path.read_bytes()
    output_path.write_bytes(content.replace(b"\r\n", b"\n").replace(b"\r", b"\n"))


def hash_tree(root: pathlib.Path) -> dict[str, str]:
    """按相对路径排序记录生成树内容哈希，供 verify/缓存判断真实漂移。"""
    hashes: dict[str, str] = {}
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        hashes[path.relative_to(root).as_posix()] = file_sha256(path)
    return hashes


def normalize_xml_hashes(
    generated: pathlib.Path,
    dialect_path: pathlib.Path,
    xml_dir: pathlib.Path,
) -> None:
    """用源 XML 的 SHA-256 截断值替换依赖 Python 版本的 mavgen 哈希。"""
    sources = {
        "dima": dialect_path,
        "standard": xml_dir / "standard.xml",
        "minimal": xml_dir / "minimal.xml",
    }
    for dialect, source in sources.items():
        stable_hash = int.from_bytes(
            hashlib.sha256(source.read_bytes()).digest()[:8], "big"
        ) & ((1 << 63) - 1)
        replacements = (
            (
                generated / dialect / f"{dialect}.h",
                rb"(#define MAVLINK_" + dialect.upper().encode()
                + rb"_XML_HASH )-?[0-9]+",
            ),
            (
                generated / dialect / "mavlink.h",
                rb"(#define MAVLINK_PRIMARY_XML_HASH )-?[0-9]+",
            ),
        )
        for path, pattern in replacements:
            original = path.read_bytes()
            updated, count = re.subn(
                pattern, rb"\g<1>" + str(stable_hash).encode(), original, count=1
            )
            if count != 1:
                raise GenerationError(f"unable to normalize XML hash in {path}")
            path.write_bytes(updated)


def normalize_generated_line_endings(generated: pathlib.Path) -> None:
    """统一为 LF，避免 Windows/Unix 主机差异污染生成合同哈希。"""
    for path in (item for item in generated.rglob("*") if item.is_file()):
        content = path.read_bytes()
        normalized = content.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
        if normalized != content:
            path.write_bytes(normalized)


def write_runtime_contract(
    generated: pathlib.Path,
    runtime_messages: list[dict],
    inbound_messages: list[dict],
) -> None:
    """生成 C++ 编译期消息表、调度阶段和 handler 索引辅助函数。

    表项完全来自已验证 lock；``service_index`` 只对 Service 周期流计数，因此
    数组槽位与 MavlinkService 的运行时 interval 状态一一对应。
    """
    handlers = ",\n".join(
        f"    {message['handler']}" for message in runtime_messages
    )
    scheduler_names = {
        "none": "None",
        "heartbeat": "Heartbeat",
        "service": "Service",
    }
    tx_stage_names = {
        "none": "None",
        "pre_metadata": "PreMetadata",
        "post_metadata": "PostMetadata",
    }
    descriptors = []
    for message in runtime_messages:
        descriptors.append(
            "    {"
            f"{message['message_id']}U, MessageHandler::{message['handler']}, "
            f"Scheduler::{scheduler_names[message['scheduler']]}, "
            f"TxStage::{tx_stage_names[message['tx_stage']]}, "
            f"{message['default_interval_us']}, "
            f"{'true' if message['requestable'] else 'false'}, "
            f"{'true' if message['interval_configurable'] else 'false'}"
            "}"
        )
    descriptor_text = ",\n".join(descriptors)
    inbound_handlers = list(dict.fromkeys(
        message["handler"] for message in inbound_messages
    ))
    inbound_handler_text = ",\n".join(
        f"    {handler}" for handler in inbound_handlers
    )
    inbound_descriptor_text = ",\n".join(
        "    {"
        f"{message['message_id']}U, InboundHandler::{message['handler']}"
        "}"
        for message in inbound_messages
    )
    service_count = sum(
        message["scheduler"] == "service" for message in runtime_messages
    )
    content = f"""// Generated by tools/mavlink/build_trimmed_dialect.py. Do not edit.
#pragma once

#include <cstddef>
#include <cstdint>

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

inline constexpr MessageContract kMessages[]{{
{descriptor_text}
}};
inline constexpr std::size_t kMessageCount =
    sizeof(kMessages) / sizeof(kMessages[0]);
inline constexpr std::size_t kServiceStreamCount = {service_count}U;
inline constexpr std::size_t kInvalidServiceIndex = kServiceStreamCount;

inline constexpr InboundMessageContract kInboundMessages[]{{
{inbound_descriptor_text}
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
    (generated / "mavlink_stream_contract.hpp").write_bytes(
        content.encode("utf-8")
    )


def generate(arguments: argparse.Namespace) -> None:
    """在隔离 work 目录完成校验、mavgen、规范化和原子发布。"""
    lock = load_lock(arguments.lock)
    validate_inputs(arguments.xml_dir, lock)
    runtime_messages, inbound_messages = validate_allowlist(arguments.xml_dir, lock)

    pymavlink_root = provision_pymavlink(
        arguments.cache_root.expanduser(), arguments.lock, arguments.pymavlink_root
    )
    sys.path.insert(0, str(pymavlink_root.parent))
    try:
        import pymavlink
        from pymavlink.generator import mavgen
    except ImportError as error:
        raise GenerationError(f"unable to import pinned pymavlink: {error}") from error
    imported_root = pathlib.Path(pymavlink.__file__).resolve().parent
    if imported_root != pymavlink_root.resolve() or pymavlink.__version__ != lock["pymavlink"]["version"]:
        raise GenerationError(
            "imported pymavlink does not match the pinned cache: "
            f"loaded {pymavlink.__version__} from {imported_root}"
        )

    # 所有中间产物先进入同级 work 目录；只有完整生成、规范化和 manifest
    # 写入都成功后才替换目标树，失败不会留下“半新半旧”的头文件集合。
    output_dir = arguments.output_dir
    work_dir = output_dir.with_name(output_dir.name + ".work")
    if work_dir.exists():
        shutil.rmtree(work_dir)
    definitions = work_dir / "definitions"
    generated = work_dir / "generated"
    definitions.mkdir(parents=True)
    generated.mkdir()
    for filename in ("minimal.xml", "standard.xml"):
        shutil.copyfile(arguments.xml_dir / filename, definitions / filename)
    dialect_path = definitions / "dima.xml"
    write_dialect(arguments.xml_dir, dialect_path, lock)

    class Options:
        language = "C"
        wire_protocol = "2.0"
        output = str(generated)
        validate = False
        error_limit = 200
        strict_types = False
        strict_units = False

    if not mavgen.mavgen(Options(), [str(dialect_path)]):
        raise GenerationError("pinned mavgen reported failure")
    required_header = generated / "dima" / "mavlink.h"
    if not required_header.is_file():
        raise GenerationError(f"mavgen did not create {required_header}")

    normalize_generated_line_endings(generated)
    normalize_xml_hashes(generated, dialect_path, arguments.xml_dir)
    shutil.copyfile(dialect_path, generated / "dima.xml")
    write_runtime_contract(generated, runtime_messages, inbound_messages)
    manifest = {
        "dialect": "dima",
        "message_count": lock["dialect"]["message_count"],
        "mavlink_commit": lock["mavlink"]["commit"],
        "pymavlink_commit": lock["pymavlink"]["commit"],
        "pymavlink_version": lock["pymavlink"]["version"],
        "xml_sha256": lock["mavlink"]["xml_sha256"],
        "runtime_messages": runtime_messages,
        "runtime_inbound": inbound_messages,
        "output_sha256": hash_tree(generated),
    }
    (generated / ".generated.json").write_bytes(
        (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    )

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    if output_dir.exists():
        shutil.rmtree(output_dir)
    install_generated_tree(generated, output_dir)
    shutil.rmtree(work_dir)
    print(
        f"generated {lock['dialect']['message_count']}-message Dima MAVLink "
        f"library with pymavlink {lock['pymavlink']['version']} into {output_dir}"
    )


def main() -> int:
    script_dir = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml-dir", type=pathlib.Path, required=True)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    parser.add_argument(
        "--lock", type=pathlib.Path, default=script_dir / "mavlink.lock.json"
    )
    parser.add_argument(
        "--cache-root",
        type=pathlib.Path,
        default=pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools",
    )
    parser.add_argument("--pymavlink-root", type=pathlib.Path)
    arguments = parser.parse_args()
    generate(arguments)
    return 0


if __name__ == "__main__":
    if os.environ.get("PYTHONHASHSEED") != "0":
        environment = os.environ.copy()
        environment["PYTHONHASHSEED"] = "0"
        completed = subprocess.run(
            [sys.executable, __file__, *sys.argv[1:]], env=environment, check=False
        )
        raise SystemExit(completed.returncode)
    try:
        raise SystemExit(main())
    except (BootstrapError, GenerationError) as error:
        print(f"MAVLink generation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
