"""Host platform detection and pinned mcumgr runtime resolution."""

from __future__ import annotations

import os
import pathlib
import re
import shutil

from bootstrap_mcumgr import BootstrapError, ensure_mcumgr

from .models import (
    HostPlatform,
    McumgrRuntime,
    SerialBackend,
    UploadError,
)

def is_wsl() -> bool:
    try:
        return "microsoft" in pathlib.Path("/proc/sys/kernel/osrelease").read_text(
            encoding="utf-8"
        ).lower()
    except OSError:
        return False


def detect_host_platform() -> HostPlatform:
    if os.name == "nt":
        return HostPlatform.WINDOWS_NATIVE
    if is_wsl():
        return HostPlatform.WSL
    return HostPlatform.POSIX

def resolve_mcumgr(
    requested: str, explicit_port: str | None, tools_cache: pathlib.Path
) -> McumgrRuntime:
    host_platform = detect_host_platform()
    explicit_windows_port = explicit_port is not None and re.fullmatch(
        r"COM[0-9]+", explicit_port, re.IGNORECASE
    ) is not None
    if host_platform == HostPlatform.WINDOWS_NATIVE:
        if explicit_port is not None and not explicit_windows_port:
            raise UploadError(
                "HOST_PREFLIGHT: native Windows requires a COM port, got "
                f"{explicit_port}"
            )
        serial_backend = SerialBackend.WINDOWS_COM
    elif host_platform == HostPlatform.WSL:
        serial_backend = (
            SerialBackend.WINDOWS_COM
            if explicit_port is None or explicit_windows_port
            else SerialBackend.POSIX_TTY
        )
    else:
        if explicit_windows_port:
            raise UploadError(
                "HOST_PREFLIGHT: a COM port requires native Windows or WSL interop"
            )
        serial_backend = SerialBackend.POSIX_TTY

    def runtime_for(path: pathlib.Path, windows_executable: bool) -> McumgrRuntime:
        if (
            serial_backend == SerialBackend.WINDOWS_COM
            and host_platform == HostPlatform.WSL
            and not windows_executable
        ):
            raise UploadError(
                "HOST_PREFLIGHT: a Windows COM port requires mcumgr.exe under WSL"
            )
        if (
            serial_backend == SerialBackend.POSIX_TTY
            and windows_executable
        ):
            raise UploadError(
                "HOST_PREFLIGHT: a POSIX tty cannot be used with Windows mcumgr.exe"
            )
        return McumgrRuntime(
            executable=str(path.resolve()),
            host_platform=host_platform,
            serial_backend=serial_backend,
            executable_is_windows=windows_executable,
        )

    direct = pathlib.Path(requested).expanduser()
    if direct.is_file():
        windows_executable = (
            host_platform == HostPlatform.WINDOWS_NATIVE
            or direct.suffix.casefold() == ".exe"
        )
        return runtime_for(direct, windows_executable)

    default_mcumgr = requested in {"mcumgr", "mcumgr.exe"}
    if default_mcumgr:
        # Always use the pinned Dima build here.  Besides keeping the executable
        # and serial port on one OS, it removes Apache newtmgr's UART-only 20 ms
        # delay from the USB CDC fast path. An arbitrary PATH executable can
        # silently restore the approximately 2 KiB/s behavior.
        try:
            bootstrapped = ensure_mcumgr(
                tools_cache,
                target_windows=serial_backend == SerialBackend.WINDOWS_COM,
                announce_cache_hit=False,
            )
        except BootstrapError as error:
            raise UploadError(str(error)) from error
        return runtime_for(
            bootstrapped,
            serial_backend == SerialBackend.WINDOWS_COM,
        )

    resolved = shutil.which(requested)
    if resolved is not None:
        resolved_path = pathlib.Path(resolved)
        windows_executable = (
            host_platform == HostPlatform.WINDOWS_NATIVE
            or resolved_path.suffix.casefold() == ".exe"
        )
        return runtime_for(resolved_path, windows_executable)

    raise UploadError(
        f"the requested mcumgr executable was not found: {requested}"
    )

