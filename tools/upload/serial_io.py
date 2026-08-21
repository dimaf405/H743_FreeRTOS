"""Backend-neutral binary serial exchange dispatch."""

from __future__ import annotations

from .models import SerialBackend, SerialSequenceResult, SerialWriteStage
from .serial_posix import (
    posix_serial_exchange_bytes,
    posix_serial_sequence_bytes,
)
from .serial_windows import (
    windows_serial_exchange_bytes,
    windows_serial_sequence_bytes,
)


def serial_exchange_bytes(
    serial_backend: SerialBackend,
    port: str,
    request: bytes,
    read_seconds: float,
) -> tuple[bool, bytes, str]:
    if serial_backend == SerialBackend.WINDOWS_COM:
        return windows_serial_exchange_bytes(port, request, read_seconds)
    return posix_serial_exchange_bytes(port, request, read_seconds)


def serial_sequence_bytes(
    serial_backend: SerialBackend,
    port: str,
    stages: tuple[SerialWriteStage, ...],
    read_seconds: float,
) -> SerialSequenceResult:
    if serial_backend == SerialBackend.WINDOWS_COM:
        return windows_serial_sequence_bytes(port, stages, read_seconds)
    return posix_serial_sequence_bytes(port, stages, read_seconds)
