#!/usr/bin/env python3
"""从 UM982 manifest 生成 PX4 派生的 NMEA 10 Hz 产品合同。"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import tempfile
from pathlib import Path
from typing import TypedDict


EXPECTED_UPSTREAM = {
    "repository": "PX4/PX4-GPSDrivers",
    "commit": "0b9695881bd1e8f830ab4538ab3acc0050019eba",
    "source": "src/nmea.cpp",
    "lines": "1053-1091",
}
EXPECTED_RECEIVER_MANUAL = {
    "title": "Unicore Reference Commands Manual For N4 High Precision Products",
    "version": "V2 EN R1.15",
    "date": "2026-06",
    "sections": ["4.2", "7.5.9", "7.5.99", "7.5.101", "8.3", "8.4"],
}
EXPECTED_RATE_POLICY = {
    "message_rate_hz": 10,
    "target_baudrate": 460800,
}
EXPECTED_MESSAGES_SHA256 = (
    "7b429858dbbf2cb9adb9619339ee3b2398204d7e6a29edd232f5f9d1c1458ac7"
)
NAME_RE = re.compile(r"^[A-Z][A-Z0-9]*$")
PERIOD_RE = re.compile(r"^(?:0\.[0-9]*[1-9][0-9]*|[1-9][0-9]*\.[0-9]+)$")
ROLE_CPP = {
    "gga": "Gga",
    "agrica": "Agrica",
    "heading": "Heading",
    "gst": "Gst",
    "gsa": "Gsa",
    "rmc": "Rmc",
}


class MessageContractEntry(TypedDict):
    """manifest 中单条消息的已校验结构；aliases 只能由生成合同消费。"""

    log_name: str
    command_name: str
    period_s: str
    role: str
    aliases: list[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def load_contract(
        path: Path) -> tuple[list[MessageContractEntry], dict[str, int]]:
    """校验固定上游、消息清单内容散列、10 Hz 周期和 uint8 位图容量。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    if (data.get("format_version") != 2 or
            data.get("contract") != "px4_um982_nmea_10hz" or
            data.get("upstream") != EXPECTED_UPSTREAM or
            data.get("receiver_manual") != EXPECTED_RECEIVER_MANUAL or
            data.get("rate_policy") != EXPECTED_RATE_POLICY):
        raise RuntimeError("unsupported UM982 contract identity")

    messages = data.get("messages")
    if not isinstance(messages, list):
        raise RuntimeError("UM982 messages must be a list")
    canonical = json.dumps(
        messages, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("ascii")
    if hashlib.sha256(canonical).hexdigest() != EXPECTED_MESSAGES_SHA256:
        raise RuntimeError("UM982 messages differ from the fixed 10 Hz contract")

    seen_logs: set[str] = set()
    seen_names: set[str] = set()
    seen_roles: set[str] = set()
    for index, message in enumerate(messages):
        if not isinstance(message, dict) or set(message) != {
                "log_name", "command_name", "period_s", "role", "aliases"}:
            raise RuntimeError(f"invalid UM982 message entry {index}")
        log_name = message["log_name"]
        command_name = message["command_name"]
        period_s = message["period_s"]
        role = message["role"]
        aliases = message["aliases"]
        if (not isinstance(log_name, str) or NAME_RE.fullmatch(log_name) is None or
                not isinstance(command_name, str) or
                NAME_RE.fullmatch(command_name) is None or
                not isinstance(period_s, str) or
                PERIOD_RE.fullmatch(period_s) is None or
                not isinstance(role, str) or role not in ROLE_CPP or
                not isinstance(aliases, list)):
            raise RuntimeError(f"invalid UM982 message fields at entry {index}")
        if log_name in seen_logs:
            raise RuntimeError(f"duplicate UM982 log name {log_name}")
        if log_name in seen_names:
            raise RuntimeError(f"duplicate UM982 accepted name {log_name}")
        if role in seen_roles:
            raise RuntimeError(f"duplicate UM982 message role {role}")
        for alias in aliases:
            if (not isinstance(alias, str) or NAME_RE.fullmatch(alias) is None or
                    alias in seen_names or alias == log_name):
                raise RuntimeError(
                    f"invalid UM982 alias {alias!r} at entry {index}")
            seen_names.add(alias)
        if period_s != "0.1":
            raise RuntimeError(f"UM982 message {log_name} is not configured at 10 Hz")
        seen_logs.add(log_name)
        seen_names.add(log_name)
        seen_roles.add(role)

    if not messages or len(messages) > 8:
        raise RuntimeError("UM982 message count must fit the uint8 log mask")
    if seen_roles != set(ROLE_CPP):
        raise RuntimeError("UM982 message roles are incomplete")
    return messages, data["rate_policy"]


def render(messages: list[MessageContractEntry],
           rate_policy: dict[str, int]) -> str:
    """把权威消息清单和目标波特率渲染成只读 C++ 数组，运行期不得另列命令名。"""
    rows = []
    aliases = []
    for index, message in enumerate(messages):
        log_name = json.dumps(message["log_name"])
        command_name = json.dumps(message["command_name"])
        period_s = json.dumps(message["period_s"])
        role = ROLE_CPP[message["role"]]
        rows.append(
            f"    {{{log_name}, {command_name}, {period_s}, "
            f"{message['period_s']}F, MessageRole::{role}}},"
        )
        for alias in message["aliases"]:
            aliases.append(
                f"    {{{index}U, {json.dumps(alias)}}},"
            )
    return "\n".join([
        "/****************************************************************************",
        " * Generated from the PX4-derived UM982 10 Hz product policy and the",
        " * Unicore N4 V2 EN R1.15 command syntax. Compatibility names are RX-only.",
        " * DO NOT EDIT.",
        " * PX4-GPSDrivers 0b9695881bd1e8f830ab4538ab3acc0050019eba",
        " * src/nmea.cpp:1053-1091",
        " ****************************************************************************/",
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <cstring>",
        "",
        "namespace dima::protocols::um982::generated {",
        "",
        f"inline constexpr std::uint32_t kTargetBaudrate = "
        f"{rate_policy['target_baudrate']}U;",
        "",
        "enum class MessageRole : std::uint8_t {",
        "    Gga,",
        "    Agrica,",
        "    Heading,",
        "    Gst,",
        "    Gsa,",
        "    Rmc,",
        "};",
        "",
        "struct MessageContract {",
        "    const char *log_name;",
        "    const char *command_name;",
        "    const char *period_s;",
        "    float expected_period_s;",
        "    MessageRole role;",
        "};",
        "",
        "struct MessageAlias {",
        "    std::size_t contract_index;",
        "    const char *name;",
        "};",
        "",
        "inline constexpr MessageContract kMessageContracts[]{",
        *rows,
        "};",
        "inline constexpr std::size_t kMessageContractCount =",
        "    sizeof(kMessageContracts) / sizeof(kMessageContracts[0]);",
        "static_assert(kMessageContractCount > 0U && kMessageContractCount <= 8U);",
        "",
        "inline constexpr MessageAlias kMessageAliases[]{",
        *aliases,
        "};",
        "inline constexpr std::size_t kMessageAliasCount =",
        "    sizeof(kMessageAliases) / sizeof(kMessageAliases[0]);",
        "",
        "inline bool message_name_matches(std::size_t contract_index,",
        "                                 const char *name) noexcept",
        "{",
        "    if (contract_index >= kMessageContractCount || name == nullptr) {",
        "        return false;",
        "    }",
        "    if (std::strcmp(kMessageContracts[contract_index].log_name, name) == 0) {",
        "        return true;",
        "    }",
        "    for (const auto &alias : kMessageAliases) {",
        "        if (alias.contract_index == contract_index &&",
        "            std::strcmp(alias.name, name) == 0) {",
        "            return true;",
        "        }",
        "    }",
        "    return false;",
        "}",
        "",
        "inline std::size_t find_message_contract(const char *name) noexcept",
        "{",
        "    for (std::size_t index = 0U; index < kMessageContractCount; ++index) {",
        "        if (message_name_matches(index, name)) {",
        "            return index;",
        "        }",
        "    }",
        "    return kMessageContractCount;",
        "}",
        "",
        "inline std::size_t find_message_role(MessageRole role) noexcept",
        "{",
        "    for (std::size_t index = 0U; index < kMessageContractCount; ++index) {",
        "        if (kMessageContracts[index].role == role) {",
        "            return index;",
        "        }",
        "    }",
        "    return kMessageContractCount;",
        "}",
        "",
        "} // namespace dima::protocols::um982::generated",
        "",
    ])


def write_if_changed(path: Path, content: str) -> None:
    """同目录临时文件后原子替换，内容未变化时保持时间戳稳定。"""
    if path.is_file() and path.read_text(encoding="utf-8") == content:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = ""
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", newline="\n",
                dir=path.parent, delete=False) as temporary:
            temporary.write(content)
            temporary_name = temporary.name
        os.replace(temporary_name, path)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def main() -> int:
    args = parse_args()
    messages, rate_policy = load_contract(args.manifest)
    destination = args.output / "Um982MessageContract.hpp"
    write_if_changed(destination, render(messages, rate_policy))
    print(f"Generated {len(messages)} UM982 message contracts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
