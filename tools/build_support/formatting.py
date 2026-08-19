"""Stable progress colors and child-command output formatting."""

from __future__ import annotations

import os
import re
import sys
from typing import Sequence

from .models import Step

def color_enabled(*, disabled: bool, stream: object) -> bool:
    if disabled or os.environ.get("NO_COLOR"):
        return False
    if os.environ.get("CI") or os.environ.get("TERM", "").lower() == "dumb":
        return False
    return bool(getattr(stream, "isatty", lambda: False)())


def colored(text: str, code: str, *, enabled: bool) -> str:
    if not enabled:
        return text
    return f"\033[{code}m{text}\033[0m"


def report_progress_error(message: str, *, no_color: bool) -> None:
    enabled = color_enabled(disabled=no_color, stream=sys.stderr)
    heading = colored("PROGRESS ERROR", "1;31", enabled=enabled)
    print(f"{heading}: {message}", file=sys.stderr, flush=True)

def child_exit_code(returncode: int) -> int:
    if returncode >= 0:
        return returncode
    return 128 + abs(returncode)


def decoded_child_output(output: bytes) -> str:
    return output.decode("utf-8", errors="replace").replace("\r\n", "\n").replace(
        "\r", "\n"
    )


def emit_child_output(output: bytes, stream: object) -> None:
    text = decoded_child_output(output).rstrip("\n")
    if text:
        print(text, file=stream, flush=True)


def size_details(lines: Sequence[str]) -> list[str] | None:
    for line in lines:
        match = re.fullmatch(
            r"\s*([0-9]+)\s+([0-9]+)\s+([0-9]+)\s+([0-9]+)\s+"
            r"[0-9a-fA-F]+\s+.+",
            line,
        )
        if match is None:
            continue
        text_size, data_size, bss_size, total_size = (
            int(value) for value in match.groups()
        )
        return [
            f"text={text_size:,}  data={data_size:,}  bss={bss_size:,}  "
            f"total={total_size:,} bytes"
        ]
    return None


def formatted_child_output(step: Step, output: bytes) -> list[str]:
    lines = [line.strip() for line in decoded_child_output(output).splitlines()]
    lines = [line for line in lines if line]
    if not lines:
        return []

    if step.label == "SIZE":
        details = size_details(lines)
        return details if details is not None else lines

    if step.label == "SIGN":
        return [line for line in lines if line != "image.py: sign the payload"]

    if step.label == "ARCH":
        match = re.fullmatch(r"architecture check: (PASS|FAIL) \((.+)\)", lines[0])
        if match is not None:
            return [f"{match.group(1)} - {match.group(2)}", *lines[1:]]
        return lines

    if step.label == "PARAM":
        match = re.fullmatch(r"generated ([0-9]+) parameters in .+", lines[0])
        if match is not None:
            return [f"{int(match.group(1)):,} parameters generated", *lines[1:]]
        return lines

    if step.label == "VERIFY":
        ignored = {
            "Image was correctly validated",
            "MCUboot image verification passed",
        }
        details: list[str] = []
        for line in lines:
            if line in ignored:
                continue
            key, separator, value = line.partition(":")
            if separator and key and value.strip():
                details.append(f"{key.strip().lower():<16}: {value.strip()}")
            else:
                details.append(line)
        return details

    return lines

