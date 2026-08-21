"""Preflight discovery for Dima application and MCUboot endpoints."""

from __future__ import annotations

import pathlib
import time

from .endpoint_errors import port_busy
from .mavlink import MavlinkCodec
from .mcumgr import try_image_list
from .models import (
    HostPlatform,
    McumgrRuntime,
    SerialBackend,
    UploadError,
)
from .recovery_request import try_application_identify
from .serial_discovery import serial_ports


def endpoint_preflight(
    runtime: McumgrRuntime,
    codec: MavlinkCodec,
    explicit_port: str | None,
    wait_seconds: int,
    baud: int,
    mtu: int,
) -> tuple[str, str]:
    host_label = {
        HostPlatform.WINDOWS_NATIVE: "Windows native",
        HostPlatform.WSL: "WSL",
        HostPlatform.POSIX: "POSIX",
    }[runtime.host_platform]
    transport_label = {
        SerialBackend.WINDOWS_COM: "Windows COM",
        SerialBackend.POSIX_TTY: "POSIX tty",
    }[runtime.serial_backend]
    print("[PREFLIGHT] USB upload", flush=True)
    print(f"  Host       : {host_label}", flush=True)
    print(f"  Transport  : {transport_label}", flush=True)
    print(
        f"  mcumgr     : {pathlib.Path(runtime.executable).name}",
        flush=True,
    )
    print(f"  Scan limit : {wait_seconds}s", flush=True)
    deadline = time.monotonic() + wait_seconds
    last_error = "no serial ports detected"
    probe_after: dict[str, float] = {}
    while time.monotonic() < deadline:
        ports = [explicit_port] if explicit_port else serial_ports(runtime)
        matches: list[tuple[str, str]] = []
        now = time.monotonic()
        for port in ports:
            if time.monotonic() >= deadline:
                break
            if port is None or now < probe_after.get(port, 0.0):
                continue
            probe_after[port] = now + 1.0
            success, output = try_image_list(
                runtime.executable, port, baud, mtu
            )
            if success:
                matches.append((port, "MCUboot Recovery"))
                continue
            if output:
                last_error = f"{port}: {output}"
            if explicit_port and port_busy(output):
                raise UploadError(
                    f"PORT_BUSY: {port} is already open by another process ({output})"
                )

            _, identity, identify_output = try_application_identify(
                codec, runtime.serial_backend, port, deadline
            )
            if identity is not None:
                matches.append((port, identity.summary()))
                continue
            if identify_output:
                last_error = f"{port}: {identify_output[-240:]}"
            if explicit_port and port_busy(identify_output):
                raise UploadError(
                    f"PORT_BUSY: {port} is already open by another process "
                    f"({identify_output})"
                )

        if len(matches) > 1:
            raise UploadError(
                "HOST_PREFLIGHT: multiple Dima protocol endpoints were identified: "
                + ", ".join(port for port, _ in matches)
                + "; set MCUMGR_PORT to the intended application port"
            )
        if matches:
            port, endpoint = matches[0]
            print(f"  Device     : {port} ({endpoint})", flush=True)
            print("  Status     : ready\n", flush=True)
            return port, endpoint
        time.sleep(0.25)

    raise UploadError(
        f"HOST_PREFLIGHT: no compatible endpoint was found within "
        f"{wait_seconds}s ({last_error})"
    )
