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


EXPECTED_UPSTREAM = {
    "repository": "PX4/PX4-GPSDrivers",
    "commit": "0b9695881bd1e8f830ab4538ab3acc0050019eba",
    "source": "src/nmea.cpp",
    "lines": "1053-1091",
}
EXPECTED_RATE_POLICY = {
    "message_rate_hz": 10,
    "target_baudrate": 460800,
}
EXPECTED_MESSAGES_SHA256 = (
    "bcdffa293c27be1d17c20fb9b43e24b1890df84665a6c281bb5eb0a270261519"
)
NAME_RE = re.compile(r"^[A-Z][A-Z0-9]*$")
PERIOD_RE = re.compile(r"^(?:0\.[0-9]*[1-9][0-9]*|[1-9][0-9]*\.[0-9]+)$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def load_contract(
        path: Path) -> tuple[list[dict[str, str]], dict[str, int]]:
    """校验固定上游、消息清单内容散列、10 Hz 周期和 uint8 位图容量。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    if (data.get("format_version") != 1 or
            data.get("contract") != "px4_um982_nmea_10hz" or
            data.get("upstream") != EXPECTED_UPSTREAM or
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
    for index, message in enumerate(messages):
        if not isinstance(message, dict) or set(message) != {
                "log_name", "command_name", "period_s"}:
            raise RuntimeError(f"invalid UM982 message entry {index}")
        log_name = message["log_name"]
        command_name = message["command_name"]
        period_s = message["period_s"]
        if (not isinstance(log_name, str) or NAME_RE.fullmatch(log_name) is None or
                not isinstance(command_name, str) or
                NAME_RE.fullmatch(command_name) is None or
                not isinstance(period_s, str) or
                PERIOD_RE.fullmatch(period_s) is None):
            raise RuntimeError(f"invalid UM982 message fields at entry {index}")
        if log_name in seen_logs:
            raise RuntimeError(f"duplicate UM982 log name {log_name}")
        if period_s != "0.1":
            raise RuntimeError(f"UM982 message {log_name} is not configured at 10 Hz")
        seen_logs.add(log_name)

    if not messages or len(messages) > 8:
        raise RuntimeError("UM982 message count must fit the uint8 log mask")
    return messages, data["rate_policy"]


def render(messages: list[dict[str, str]], rate_policy: dict[str, int]) -> str:
    """把权威消息清单和目标波特率渲染成只读 C++ 数组，运行期不得另列命令名。"""
    rows = []
    for message in messages:
        log_name = json.dumps(message["log_name"])
        command_name = json.dumps(message["command_name"])
        period_s = json.dumps(message["period_s"])
        rows.append(
            f"    {{{log_name}, {command_name}, {period_s}, "
            f"{message['period_s']}F}},"
        )
    return "\n".join([
        "/****************************************************************************",
        " * Generated from PX4 UM982 NMEA names with the 10 Hz product policy.",
        " * DO NOT EDIT.",
        " * PX4-GPSDrivers 0b9695881bd1e8f830ab4538ab3acc0050019eba",
        " * src/nmea.cpp:1053-1091",
        " ****************************************************************************/",
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace dima::protocols::um982::generated {",
        "",
        f"inline constexpr std::uint32_t kTargetBaudrate = "
        f"{rate_policy['target_baudrate']}U;",
        "",
        "struct MessageContract {",
        "    const char *log_name;",
        "    const char *command_name;",
        "    const char *period_s;",
        "    float expected_period_s;",
        "};",
        "",
        "inline constexpr MessageContract kMessageContracts[]{",
        *rows,
        "};",
        "inline constexpr std::size_t kMessageContractCount =",
        "    sizeof(kMessageContracts) / sizeof(kMessageContracts[0]);",
        "static_assert(kMessageContractCount > 0U && kMessageContractCount <= 8U);",
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
