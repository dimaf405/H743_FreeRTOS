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


def finish_step(path: pathlib.Path, step: Step, *, no_color: bool) -> None:
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


def child_exit_code(returncode: int) -> int:
    if returncode >= 0:
        return returncode
    return 128 + abs(returncode)


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

    try:
        completed = subprocess.run(command, check=False)
        returncode = child_exit_code(completed.returncode)
    except FileNotFoundError as error:
        print(f"{error.filename}: command not found", file=sys.stderr, flush=True)
        returncode = 127
    except KeyboardInterrupt:
        returncode = 130

    if returncode != 0:
        enabled = color_enabled(disabled=arguments.no_color, stream=sys.stderr)
        failed = colored("[FAILED]", "1;31", enabled=enabled)
        label = colored(f"{step.label:<8}", "1;31", enabled=enabled)
        print(f"{failed} {label} {step.display}", file=sys.stderr, flush=True)
        return returncode

    if state_path:
        try:
            finish_step(state_path, step, no_color=arguments.no_color)
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
    summary_parser.add_argument("--no-color", action="store_true")
    summary_parser.set_defaults(handler=summary)

    return main_parser


def main() -> int:
    arguments = parser().parse_args()
    return int(arguments.handler(arguments))


if __name__ == "__main__":
    raise SystemExit(main())
