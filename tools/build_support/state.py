"""Locked progress-state lifecycle for parallel GNU Make actions."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
import json
import os
import pathlib
import sys
from typing import Iterator, Sequence

if os.name == "nt":
    import msvcrt
else:
    import fcntl

from .formatting import color_enabled, colored, report_progress_error
from .models import ProgressError, STATE_VERSION, Step
from .plan import logical_plan_lines, step_from_plan_line

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

