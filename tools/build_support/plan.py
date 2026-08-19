"""Dry-run Make plan parsing and progress-step derivation."""

from __future__ import annotations

import pathlib
import re
import shlex
from typing import Iterator, Sequence

from .models import PLAN_TOKEN, ProgressError, Step

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

