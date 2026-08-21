"""Endpoint error classification shared by discovery and wait loops."""

from __future__ import annotations

import re


PORT_BUSY_RE = re.compile(
    r"access is denied|permission denied|resource busy|being used by another process|"
    r"device or resource busy|访问被拒绝",
    re.IGNORECASE,
)


def port_busy(output: str) -> bool:
    return PORT_BUSY_RE.search(output) is not None
