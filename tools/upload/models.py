"""上传流程共享的串口合同、不可变数据模型、阶段计时和输出解码。"""

from __future__ import annotations

import dataclasses
import enum
import locale
import time
from typing import TextIO

DEFAULT_USB_CDC_BAUD = 921600
DEFAULT_SERIAL_MTU = 512
DEFAULT_MAX_WINDOW = 1

_STAGE_STARTED = time.monotonic()
_STAGE_PREVIOUS = _STAGE_STARTED



class UploadError(RuntimeError):
    """可直接据错误文本处理的一条命令上传失败。"""


class HostPlatform(enum.Enum):
    WINDOWS_NATIVE = "windows-native"
    WSL = "wsl"
    POSIX = "posix"


class SerialBackend(enum.Enum):
    WINDOWS_COM = "windows-com"
    POSIX_TTY = "posix-tty"


@dataclasses.dataclass(frozen=True)
class McumgrRuntime:
    executable: str
    host_platform: HostPlatform
    serial_backend: SerialBackend
    executable_is_windows: bool


@dataclasses.dataclass
class ImageState:
    image: int
    slot: int
    version: str = ""
    flags: set[str] = dataclasses.field(default_factory=set)
    digest: str = ""


@dataclasses.dataclass(frozen=True)
class SerialWriteStage:
    payload: bytes
    delay_seconds: float = 0.0
    reset_buffers: bool = False


@dataclasses.dataclass(frozen=True)
class SerialSequenceResult:
    attempted: bool
    completed_stages: int
    disconnected: bool
    output: bytes
    error: str = ""


@dataclasses.dataclass(frozen=True)
class ApplicationIdentity:
    uid: int
    flight_sw_version: int
    board_version: int

    def summary(self) -> str:
        return (
            "Dima Rover MAVLink "
            f"version=0x{self.flight_sw_version:08x} "
            f"board={self.board_version} uid=0x{self.uid:016x}"
        )

def reset_stage_timing() -> None:
    global _STAGE_STARTED, _STAGE_PREVIOUS
    now = time.monotonic()
    _STAGE_STARTED = now
    _STAGE_PREVIOUS = now


def stage(
        name: str, message: str, *, stream: TextIO | None = None) -> None:
    """用单调时钟同时报告总耗时和相邻阶段耗时，便于定位枚举/传输卡点。"""
    global _STAGE_PREVIOUS
    now = time.monotonic()
    elapsed = now - _STAGE_STARTED
    delta = now - _STAGE_PREVIOUS
    _STAGE_PREVIOUS = now
    print(
        f"[{name}] t={elapsed:.2f}s delta={delta:.2f}s {message}",
        file=stream,
        flush=True,
    )


def decode_output(output: bytes) -> str:
    """先按 UTF-8 解码工具输出，失败时使用主机编码并替换坏字节，永不隐藏原错误。"""
    output = output.replace(b"\x00", b"")
    try:
        return output.decode("utf-8").strip()
    except UnicodeDecodeError:
        return output.decode(
            locale.getpreferredencoding(False), errors="replace"
        ).strip()

