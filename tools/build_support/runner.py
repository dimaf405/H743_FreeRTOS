"""Execution wrapper for one planned build action."""

from __future__ import annotations

import argparse
import pathlib
import shlex
import subprocess
import sys

from .formatting import (
    child_exit_code,
    color_enabled,
    colored,
    emit_child_output,
    formatted_child_output,
    report_progress_error,
)
from .models import CAPTURED_OUTPUT_LABELS, PROGRESS_ERROR_EXIT, ProgressError
from .plan import derive_step
from .state import finish_step, reserve_step

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
