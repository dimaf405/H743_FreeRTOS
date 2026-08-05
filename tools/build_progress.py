#!/usr/bin/env python3
"""Exact PX4/Ninja-style progress reporting for the GNU Make build."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
from dataclasses import dataclass
import json
import os
import pathlib
import re
import shlex
import struct
import subprocess
import sys
from typing import Iterator, Sequence

if os.name == "nt":
    import msvcrt
else:
    import fcntl


PLAN_TOKEN = "DIMA_PROGRESS_STEP_V1"
STATE_VERSION = 1
PROGRESS_ERROR_EXIT = 125
CAPTURED_OUTPUT_LABELS = frozenset({"ARCH", "PARAM", "SIZE", "SIGN", "VERIFY"})


class ProgressError(RuntimeError):
    """The dry-run plan and the real build no longer describe the same work."""


@dataclass(frozen=True)
class Step:
    label: str
    target: str
    display: str

    @property
    def identity(self) -> str:
        return f"{self.label}:{self.target}"


def normalized_path(value: str) -> str:
    path = value.replace("\\", "/")
    while path.startswith("./"):
        path = path[2:]
    if not path:
        raise ProgressError("a progress step has an empty target")
    return path


def derive_step(
    *, kind: str | None, label: str | None, target: str, source: str | None,
    display: str | None
) -> Step:
    target = normalized_path(target)
    kind = (kind or "").lower()

    if label:
        action = label.upper()
    elif kind in {"cc", "cxx", "as"}:
        if target.lower().endswith(".o"):
            action = {"cc": "CC", "cxx": "CXX", "as": "AS"}[kind]
        else:
            action = "LD"
    elif kind == "objcopy":
        suffix = pathlib.PurePosixPath(target).suffix.lower()
        action = {".hex": "HEX", ".bin": "BIN"}.get(suffix, "OBJCOPY")
    elif kind == "size":
        action = "SIZE"
    else:
        raise ProgressError(f"cannot infer the action for target {target!r}")

    if not re.fullmatch(r"[A-Z][A-Z0-9_]{0,11}", action):
        raise ProgressError(f"invalid progress label: {action!r}")

    if display:
        shown = normalized_path(display)
    elif action in {"CC", "CXX", "AS"} and source:
        shown = normalized_path(source)
    else:
        shown = target
    return Step(action, target, shown)


def option_value(tokens: Sequence[str], name: str) -> str | None:
    for index, token in enumerate(tokens):
        if token == name:
            if index + 1 >= len(tokens):
                raise ProgressError(f"{name} is missing its value")
            return tokens[index + 1]
        prefix = name + "="
        if token.startswith(prefix):
            return token[len(prefix):]
    return None


def logical_plan_lines(text: str) -> Iterator[str]:
    parts: list[str] = []
    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        if line.endswith("\\"):
            parts.append(line[:-1].strip())
            continue
        parts.append(line.strip())
        yield " ".join(part for part in parts if part)
        parts.clear()
    if parts:
        yield " ".join(part for part in parts if part)


def step_from_plan_line(line: str) -> Step | None:
    if PLAN_TOKEN not in line:
        return None

    # Everything required to identify a step appears before the command's
    # ``--`` separator.  Continuations have already been joined, and starting
    # at this helper avoids unrelated ``--`` tokens in compound shell recipes.
    token_position = line.index(PLAN_TOKEN)
    tool_position = line.rfind("build_progress.py", 0, token_position)
    if tool_position < 0:
        raise ProgressError(f"progress marker has no build_progress.py command: {line}")
    arguments = line[tool_position + len("build_progress.py"):]
    prefix = arguments.split(" -- ", 1)[0]
    try:
        tokens = shlex.split(prefix)
    except ValueError as error:
        raise ProgressError(f"cannot parse dry-run progress marker: {line}") from error

    if option_value(tokens, "--plan-token") != PLAN_TOKEN:
        raise ProgressError(f"malformed dry-run progress marker: {line}")
    target = option_value(tokens, "--target")
    if target is None:
        raise ProgressError(f"dry-run progress marker has no target: {line}")
    return derive_step(
        kind=option_value(tokens, "--kind"),
        label=option_value(tokens, "--label"),
        target=target,
        source=option_value(tokens, "--source"),
        display=option_value(tokens, "--display"),
    )


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


def write_state(path: pathlib.Path, state: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as output:
        json.dump(state, output, sort_keys=True)
        output.write("\n")
        output.flush()
        os.fsync(output.fileno())


@contextmanager
def locked_state(path: pathlib.Path) -> Iterator[dict[str, object]]:
    try:
        state_file = path.open("r+", encoding="utf-8")
    except OSError as error:
        raise ProgressError(f"cannot open progress state {path}: {error}") from error

    with state_file:
        state_file.seek(0)
        try:
            if os.name == "nt":
                msvcrt.locking(state_file.fileno(), msvcrt.LK_LOCK, 1)
            else:
                fcntl.flock(state_file.fileno(), fcntl.LOCK_EX)
        except OSError as error:
            raise ProgressError(f"cannot lock progress state {path}: {error}") from error
        try:
            state_file.seek(0)
            state = json.load(state_file)
            if state.get("version") != STATE_VERSION:
                raise ProgressError(f"unsupported progress state in {path}")
            yield state
            state_file.seek(0)
            json.dump(state, state_file, sort_keys=True)
            state_file.write("\n")
            state_file.truncate()
            state_file.flush()
        finally:
            state_file.seek(0)
            if os.name == "nt":
                msvcrt.locking(state_file.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(state_file.fileno(), fcntl.LOCK_UN)


def prepare(arguments: argparse.Namespace) -> int:
    plan_path = pathlib.Path(arguments.plan)
    state_path = pathlib.Path(arguments.state)
    try:
        plan = plan_path.read_text(encoding="utf-8", errors="replace")
        lines = logical_plan_lines(plan)
        steps = [step for line in lines if (step := step_from_plan_line(line))]
        remaining = Counter(step.identity for step in steps)
        descriptions = {
            step.identity: {"label": step.label, "display": step.display}
            for step in steps
        }
        state: dict[str, object] = {
            "version": STATE_VERSION,
            "goals": arguments.goals,
            "total": len(steps),
            "completed": 0,
            "remaining": dict(remaining),
            "running": {},
            "descriptions": descriptions,
        }
        write_state(state_path, state)
    except (OSError, ProgressError) as error:
        report_progress_error(str(error), no_color=arguments.no_color)
        return 2
    return 0


def reserve_step(path: pathlib.Path, step: Step) -> None:
    with locked_state(path) as state:
        remaining = state["remaining"]
        running = state["running"]
        assert isinstance(remaining, dict)
        assert isinstance(running, dict)
        available = int(remaining.get(step.identity, 0))
        if available <= 0:
            raise ProgressError(
                f"unplanned action {step.label} {step.display}; the build graph changed"
            )
        remaining[step.identity] = available - 1
        running[step.identity] = int(running.get(step.identity, 0)) + 1


def finish_step(
    path: pathlib.Path,
    step: Step,
    *,
    no_color: bool,
    detail_lines: Sequence[str] = (),
) -> None:
    with locked_state(path) as state:
        running = state["running"]
        assert isinstance(running, dict)
        active = int(running.get(step.identity, 0))
        if active <= 0:
            raise ProgressError(f"action {step.identity} was not reserved")
        if active == 1:
            running.pop(step.identity, None)
        else:
            running[step.identity] = active - 1

        completed = int(state["completed"]) + 1
        total = int(state["total"])
        if completed > total:
            raise ProgressError(f"completed {completed} actions from a {total}-action plan")
        state["completed"] = completed

        width = max(1, len(str(total)))
        counter = f"[{completed:>{width}}/{total}]"
        enabled = color_enabled(disabled=no_color, stream=sys.stdout)
        counter = colored(counter, "1;32", enabled=enabled)
        label = colored(f"{step.label:<8}", "1;36", enabled=enabled)
        print(f"{counter} {label} {step.display}", flush=True)
        detail_indent = " " * (2 * width + 13)
        for detail in detail_lines:
            print(f"{detail_indent}{detail}", flush=True)


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


def run_step(arguments: argparse.Namespace) -> int:
    command = list(arguments.command)
    if command and command[0] == "--":
        command.pop(0)
    if not command:
        report_progress_error("a progress step has no command", no_color=arguments.no_color)
        return PROGRESS_ERROR_EXIT

    try:
        step = derive_step(
            kind=arguments.kind,
            label=arguments.label,
            target=arguments.target,
            source=arguments.source,
            display=arguments.display,
        )
    except ProgressError as error:
        report_progress_error(str(error), no_color=arguments.no_color)
        return PROGRESS_ERROR_EXIT

    state_path = pathlib.Path(arguments.state) if arguments.state else None
    if state_path:
        try:
            reserve_step(state_path, step)
        except ProgressError as error:
            report_progress_error(str(error), no_color=arguments.no_color)
            return PROGRESS_ERROR_EXIT

    if arguments.verbose and not arguments.quiet_command:
        print("+ " + shlex.join(command), flush=True)

    capture_output = (
        state_path is not None and step.label in CAPTURED_OUTPUT_LABELS
    )
    captured_stdout = b""
    captured_stderr = b""
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE if capture_output else None,
            stderr=subprocess.PIPE if capture_output else None,
        )
        returncode = child_exit_code(completed.returncode)
        if capture_output:
            captured_stdout = completed.stdout or b""
            captured_stderr = completed.stderr or b""
    except FileNotFoundError as error:
        print(f"{error.filename}: command not found", file=sys.stderr, flush=True)
        returncode = 127
    except KeyboardInterrupt:
        returncode = 130

    if returncode != 0:
        emit_child_output(captured_stdout, sys.stdout)
        emit_child_output(captured_stderr, sys.stderr)
        enabled = color_enabled(disabled=arguments.no_color, stream=sys.stderr)
        failed = colored("[FAILED]", "1;31", enabled=enabled)
        label = colored(f"{step.label:<8}", "1;31", enabled=enabled)
        print(f"{failed} {label} {step.display}", file=sys.stderr, flush=True)
        return returncode

    if state_path:
        try:
            detail_lines = formatted_child_output(
                step, captured_stdout + captured_stderr
            )
            finish_step(
                state_path,
                step,
                no_color=arguments.no_color,
                detail_lines=detail_lines,
            )
        except ProgressError as error:
            report_progress_error(str(error), no_color=arguments.no_color)
            return PROGRESS_ERROR_EXIT
    return 0


def finish(arguments: argparse.Namespace) -> int:
    state_path = pathlib.Path(arguments.state)
    try:
        with locked_state(state_path) as state:
            total = int(state["total"])
            completed = int(state["completed"])
            remaining = state["remaining"]
            running = state["running"]
            descriptions = state["descriptions"]
            assert isinstance(remaining, dict)
            assert isinstance(running, dict)
            assert isinstance(descriptions, dict)
            unclaimed = sum(int(value) for value in remaining.values())
            active = sum(int(value) for value in running.values())
            if completed != total or unclaimed or active:
                pending: list[str] = []
                for identity, value in sorted(remaining.items()):
                    count = int(value)
                    if count <= 0:
                        continue
                    description = descriptions.get(identity, {})
                    if isinstance(description, dict):
                        label = str(description.get("label", "ACTION"))
                        display = str(description.get("display", identity))
                    else:
                        label = "ACTION"
                        display = identity
                    suffix = f" x{count}" if count > 1 else ""
                    pending.append(f"{label} {display}{suffix}")
                pending_detail = (
                    f"; unclaimed: {', '.join(pending)}" if pending else ""
                )
                raise ProgressError(
                    f"completed {completed}/{total} actions "
                    f"({unclaimed} unclaimed, {active} still running)"
                    f"{pending_detail}"
                )
            if total == 0:
                print("No work to do.", flush=True)
    except ProgressError as error:
        report_progress_error(str(error), no_color=arguments.no_color)
        return 2
    return 0


def integer_macro(header: str, name: str) -> int:
    match = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+(0x[0-9a-fA-F]+|[0-9]+)(?:U|L)*\s*$",
        header,
        flags=re.MULTILINE,
    )
    if not match:
        raise ProgressError(f"cannot read {name} from the boot layout")
    return int(match.group(1), 0)


def artifact_line(label: str, path: pathlib.Path, address: int | None = None) -> str:
    size = path.stat().st_size
    location = f" @ 0x{address:08x}" if address is not None else ""
    return f"  {label:<10} {normalized_path(str(path))} ({size} bytes{location})"


# Section-name → memory-region classification for the MCUboot-managed H743 app.
_FLASH_SECTIONS = frozenset({
    ".text", ".rodata", ".ARM.extab", ".ARM.exidx",
    ".preinit_array", ".init_array", ".fini_array",
    ".eh_frame", ".glue_7", ".glue_7t",
})
_FLASH_SECTION_PREFIXES = (".text.", ".rodata.")
_DTCM_SECTIONS = frozenset({".dima_ramfunc", ".data", ".bss"})
_DTCM_SECTION_PREFIXES = (".data.", ".bss.")


def _section_belongs(name: str, exact: frozenset[str], prefixes: tuple[str, ...]) -> bool:
    if name in exact:
        return True
    return any(name.startswith(p) for p in prefixes)


def _parse_elf_sections(
    elf_path: pathlib.Path,
) -> list[tuple[str, int, int]] | None:
    """Parse ELF section headers with pure Python (no external tools).

    Returns a list of ``(name, vma, size)`` tuples, or *None* on failure.
    """
    try:
        data = elf_path.read_bytes()
    except OSError:
        return None
    if len(data) < 16 or data[:4] != b"\x7fELF":
        return None

    ei_class = data[4]  # 1 = 32-bit, 2 = 64-bit
    ei_data = data[5]   # 1 = little-endian, 2 = big-endian
    endian = "<" if ei_data == 1 else ">"

    try:
        if ei_class == 1:  # 32-bit
            # Layout at offset 32: e_shoff(4), e_flags(4), e_ehsize(2),
            #   e_phentsize(2), e_phnum(2), e_shentsize(2), e_shnum(2),
            #   e_shstrndx(2)
            e_shoff = struct.unpack_from(f"{endian}I", data, 32)[0]
            e_shentsize, e_shnum, e_shstrndx = struct.unpack_from(
                f"{endian}HHH", data, 46
            )
        elif ei_class == 2:  # 64-bit
            # Layout at offset 40: e_shoff(8), e_flags(4), e_ehsize(2),
            #   e_phentsize(2), e_phnum(2), e_shentsize(2), e_shnum(2),
            #   e_shstrndx(2)
            e_shoff = struct.unpack_from(f"{endian}Q", data, 40)[0]
            e_shentsize, e_shnum, e_shstrndx = struct.unpack_from(
                f"{endian}HHH", data, 58
            )
        else:
            return None
    except struct.error:
        return None

    min_shentsize = 40 if ei_class == 1 else 64
    if e_shoff == 0 or e_shnum == 0 or e_shentsize < min_shentsize:
        return None

    def _read_shdr(index: int) -> tuple[int, int, int, int, int] | None:
        """Return (sh_name, sh_addr, sh_size, sh_flags, sh_offset)."""
        off = e_shoff + index * e_shentsize
        if off + e_shentsize > len(data):
            return None
        if ei_class == 1:
            # ELF32 Shdr: name(4) type(4) flags(4) addr(4) offset(4) size(4) ...
            sh_name, _, sh_flags, sh_addr, sh_offset, sh_size = struct.unpack_from(
                f"{endian}IIIIII", data, off
            )[:6]
        else:
            # ELF64 Shdr: name(4) type(4) flags(8) addr(8) offset(8) size(8) ...
            sh_name, _, sh_flags, sh_addr, sh_offset, sh_size = struct.unpack_from(
                f"{endian}IIQQQQ", data, off
            )[:6]
        return sh_name, sh_addr, sh_size, sh_flags, sh_offset

    # Read the section-header string table.
    strtab_info = _read_shdr(e_shstrndx)
    if strtab_info is None:
        return None
    strtab_file_off = strtab_info[4]  # sh_offset

    def _section_name(name_offset: int) -> str:
        start = strtab_file_off + name_offset
        end = data.index(b"\x00", start) if start < len(data) else start
        return data[start:end].decode("utf-8", errors="replace")

    sections: list[tuple[str, int, int]] = []
    for i in range(e_shnum):
        info = _read_shdr(i)
        if info is None:
            continue
        sh_name_idx, sh_addr, sh_size, _, _ = info
        name = _section_name(sh_name_idx)
        sections.append((name, sh_addr, sh_size))
    return sections


def parse_elf_memory_usage(
    elf_path: pathlib.Path,
    *,
    objdump: str = "",
) -> dict[str, int] | None:
    """Return per-region byte counts parsed from the ELF section headers.

    Keys: ``flash``, ``dtcm``, ``ram_d1``, ``ram_d2``, ``ram_dma``, ``ram_d3``.
    Returns ``None`` when the ELF cannot be parsed.
    """
    sections = _parse_elf_sections(elf_path)
    if sections is None:
        return None

    counts: dict[str, int] = {
        "flash": 0, "dtcm": 0,
        "ram_d1": 0, "ram_d2": 0, "ram_dma": 0, "ram_d3": 0,
    }
    for name, vma, size in sections:
        if size == 0:
            continue
        # Flash-resident sections (code + constants + init data load image).
        if _section_belongs(name, _FLASH_SECTIONS, _FLASH_SECTION_PREFIXES):
            counts["flash"] += size
            continue
        # DTCM sections (.dima_ramfunc, .data, .bss).
        if _section_belongs(name, _DTCM_SECTIONS, _DTCM_SECTION_PREFIXES):
            counts["dtcm"] += size
            continue
        # Classify remaining sections by VMA address range.
        if 0x08000000 <= vma < 0x08200000:
            # Flash-mapped (e.g. .isr_vector, .ARM).
            counts["flash"] += size
        elif 0x20000000 <= vma < 0x20020000:
            # DTCM (e.g. ._user_heap_stack).
            counts["dtcm"] += size
        elif 0x24000000 <= vma < 0x24080000:
            counts["ram_d1"] += size
        elif 0x30000000 <= vma < 0x30040000:
            counts["ram_d2"] += size
        elif 0x30040000 <= vma < 0x30048000:
            counts["ram_dma"] += size
        elif 0x38000000 <= vma < 0x38010000:
            counts["ram_d3"] += size
    return counts


def _fmt_bytes(value: int) -> str:
    if value >= 1024 * 1024:
        precision = 1 if value < 1024 * 1024 * 10 else 2
        return f"{value / (1024 * 1024):.{precision}f} MiB"
    if value >= 1024:
        return f"{value / 1024:.1f} KiB"
    return f"{value} B"


def _fmt_pct(used: int, total: int) -> str:
    if total <= 0:
        return "  n/a"
    return f"{used / total * 100:5.1f}%"


def print_memory_summary(
    counts: dict[str, int],
    *,
    layout_header: str,
    enabled: bool,
) -> None:
    """Print a compact Flash / RAM usage table."""
    try:
        layout = pathlib.Path(layout_header).read_text(encoding="utf-8")
        flash_total = integer_macro(layout, "H743_PRIMARY_SLOT_SIZE") - integer_macro(layout, "H743_MCUBOOT_HEADER_SIZE")
    except ProgressError:
        flash_total = 0

    # Linker-script capacities (from the MEMORY block).
    dtcm_total = 128 * 1024
    ram_d1_total = 512 * 1024
    ram_d2_total = 256 * 1024
    ram_dma_total = 32 * 1024
    ram_d3_total = 64 * 1024

    flash_used = counts.get("flash", 0)
    dtcm_used = counts.get("dtcm", 0)
    ram_d1_used = counts.get("ram_d1", 0)
    ram_d2_used = counts.get("ram_d2", 0)
    ram_dma_used = counts.get("ram_dma", 0)
    ram_d3_used = counts.get("ram_d3", 0)
    ram_total = ram_d1_used + ram_d2_used + ram_dma_used + ram_d3_used
    ram_capacity = dtcm_total + ram_d1_total + ram_d2_total + ram_dma_total + ram_d3_total

    heading = colored("Memory usage", "1;36", enabled=enabled)
    print(f"  {heading}")

    # Flash line.
    flash_bar = _bar(flash_used, flash_total, enabled=enabled)
    print(
        f"    {'Flash':<10} {flash_used:>7,} / {flash_total:>7,} B"
        f"  ({_fmt_bytes(flash_used)} / {_fmt_bytes(flash_total)})"
        f"  {_fmt_pct(flash_used, flash_total)}  {flash_bar}"
    )

    # DTCM line (code-executable RAM + .data + .bss).
    dtcm_bar = _bar(dtcm_used, dtcm_total, enabled=enabled)
    print(
        f"    {'DTCM':<10} {dtcm_used:>7,} / {dtcm_total:>7,} B"
        f"  ({_fmt_bytes(dtcm_used)} / {_fmt_bytes(dtcm_total)})"
        f"  {_fmt_pct(dtcm_used, dtcm_total)}  {dtcm_bar}"
    )

    # Combined SRAM line.
    sram_used = ram_d1_used + ram_d2_used + ram_dma_used + ram_d3_used
    sram_capacity = ram_d1_total + ram_d2_total + ram_dma_total + ram_d3_total
    sram_bar = _bar(sram_used, sram_capacity, enabled=enabled)
    print(
        f"    {'SRAM':<10} {sram_used:>7,} / {sram_capacity:>7,} B"
        f"  ({_fmt_bytes(sram_used)} / {_fmt_bytes(sram_capacity)})"
        f"  {_fmt_pct(sram_used, sram_capacity)}  {sram_bar}"
    )
    # Sub-lines for each SRAM region (only if non-zero).
    for label, used, capacity in [
        ("  D1 heap", ram_d1_used, ram_d1_total),
        ("  D2 SRAM", ram_d2_used, ram_d2_total),
        ("  DMA",     ram_dma_used, ram_dma_total),
        ("  D3 diag", ram_d3_used, ram_d3_total),
    ]:
        if used > 0:
            print(
                f"    {label:<10} {used:>7,} / {capacity:>7,} B"
                f"  ({_fmt_bytes(used)} / {_fmt_bytes(capacity)})"
                f"  {_fmt_pct(used, capacity)}"
            )

    # Total RAM (DTCM + all SRAM).
    total_used = flash_used + dtcm_used + sram_used
    total_cap = flash_total + dtcm_total + sram_capacity
    print(
        f"    {'-' * 10} {'-' * 7}   {'-' * 7}"
    )
    print(
        f"    {'Total':<10} {total_used:>7,} / {total_cap:>7,} B"
        f"  ({_fmt_bytes(total_used)} / {_fmt_bytes(total_cap)})"
        f"  {_fmt_pct(total_used, total_cap)}"
    )


def _bar(used: int, total: int, *, enabled: bool, width: int = 20) -> str:
    if total <= 0:
        return ""
    filled = max(0, min(width, round(used / total * width)))
    empty = width - filled
    pct = used / total
    if pct >= 0.9:
        color = "1;31"  # red
    elif pct >= 0.7:
        color = "1;33"  # yellow
    else:
        color = "1;32"  # green
    # Keep redirected and native Windows GBK output encodable. ANSI color can
    # still decorate the bar, but the glyphs themselves remain portable ASCII.
    bar_text = f"[{'#' * filled}{'.' * empty}]"
    return colored(bar_text, color, enabled=enabled)


def summary(arguments: argparse.Namespace) -> int:
    try:
        layout = pathlib.Path(arguments.layout).read_text(encoding="utf-8")
        boot_address = integer_macro(layout, "H743_MCUBOOT_BASE")
        app_address = integer_macro(layout, "H743_PRIMARY_SLOT_BASE")
        header_size = integer_macro(layout, "H743_MCUBOOT_HEADER_SIZE")
        vector_address = app_address + header_size

        enabled = color_enabled(disabled=arguments.no_color, stream=sys.stdout)
        print(colored("Build complete", "1;32", enabled=enabled))
        print(f"  {'Target:':<10} {arguments.goals}")
        print(f"  {'Version:':<10} {arguments.version}")

        app_elf = pathlib.Path(arguments.app_elf)
        boot_bin = pathlib.Path(arguments.boot_bin)
        signed = pathlib.Path(arguments.signed)
        factory = pathlib.Path(arguments.factory)
        if app_elf.is_file():
            print(f"  {'App ELF:':<10} {normalized_path(str(app_elf))}")
        if boot_bin.is_file():
            print(artifact_line("MCUboot:", boot_bin, boot_address))
        if signed.is_file():
            print(artifact_line("Signed:", signed, app_address))
            print(f"  {'Vector:':<10} 0x{vector_address:08x}")
        if factory.is_file():
            print(artifact_line("Factory:", factory))

        # Flash & RAM usage from the application ELF (pure-Python parser).
        if app_elf.is_file():
            counts = parse_elf_memory_usage(app_elf)
            if counts is not None:
                print()
                print_memory_summary(
                    counts,
                    layout_header=arguments.layout,
                    enabled=enabled,
                )
    except (OSError, ProgressError) as error:
        report_progress_error(str(error), no_color=arguments.no_color)
        return 2
    return 0


def parser() -> argparse.ArgumentParser:
    main_parser = argparse.ArgumentParser()
    subparsers = main_parser.add_subparsers(dest="operation", required=True)

    prepare_parser = subparsers.add_parser("prepare")
    prepare_parser.add_argument("--plan", required=True)
    prepare_parser.add_argument("--state", required=True)
    prepare_parser.add_argument("--goals", required=True)
    prepare_parser.add_argument("--no-color", action="store_true")
    prepare_parser.set_defaults(handler=prepare)

    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--state", default="")
    run_parser.add_argument("--plan-token", default=PLAN_TOKEN)
    run_parser.add_argument("--kind")
    run_parser.add_argument("--label")
    run_parser.add_argument("--target", required=True)
    run_parser.add_argument("--source")
    run_parser.add_argument("--display")
    run_parser.add_argument("--verbose", action="store_true")
    run_parser.add_argument("--quiet-command", action="store_true")
    run_parser.add_argument("--no-color", action="store_true")
    run_parser.add_argument("command", nargs=argparse.REMAINDER)
    run_parser.set_defaults(handler=run_step)

    finish_parser = subparsers.add_parser("finish")
    finish_parser.add_argument("--state", required=True)
    finish_parser.add_argument("--no-color", action="store_true")
    finish_parser.set_defaults(handler=finish)

    summary_parser = subparsers.add_parser("summary")
    summary_parser.add_argument("--goals", required=True)
    summary_parser.add_argument("--version", required=True)
    summary_parser.add_argument("--app-elf", required=True)
    summary_parser.add_argument("--boot-bin", required=True)
    summary_parser.add_argument("--signed", required=True)
    summary_parser.add_argument("--factory", required=True)
    summary_parser.add_argument("--layout", required=True)
    summary_parser.add_argument("--objdump", default="")
    summary_parser.add_argument("--size-tool", default="")
    summary_parser.add_argument("--no-color", action="store_true")
    summary_parser.set_defaults(handler=summary)

    return main_parser


def main() -> int:
    arguments = parser().parse_args()
    return int(arguments.handler(arguments))


if __name__ == "__main__":
    raise SystemExit(main())
