"""Shared upload constants, data models, and output helpers."""

from __future__ import annotations

import dataclasses
import enum
import locale

DEFAULT_USB_CDC_BAUD = 921600
DEFAULT_SERIAL_MTU = 512
DEFAULT_MAX_WINDOW = 1
DEFAULT_CONFIRM_WAIT_SECONDS = 8



class UploadError(RuntimeError):
    """An actionable one-command upload failure."""


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

def stage(name: str, message: str) -> None:
    print(f"[{name}] {message}", flush=True)


def decode_output(output: bytes) -> str:
    output = output.replace(b"\x00", b"")
    try:
        return output.decode("utf-8").strip()
    except UnicodeDecodeError:
        return output.decode(
            locale.getpreferredencoding(False), errors="replace"
        ).strip()

