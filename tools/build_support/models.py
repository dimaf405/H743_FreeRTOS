"""Shared progress-plan constants and immutable step model."""

from __future__ import annotations

from dataclasses import dataclass

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

