"""只读发现 Dima Application/MCUboot endpoint 的上传前置检查。"""

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
    stage,
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
    """在期限内分别用 SMP image list 和 MAVLink 身份探测，且只接受唯一匹配设备。"""
    host_label = {
        HostPlatform.WINDOWS_NATIVE: "Windows native",
        HostPlatform.WSL: "WSL",
        HostPlatform.POSIX: "POSIX",
    }[runtime.host_platform]
    transport_label = {
        SerialBackend.WINDOWS_COM: "Windows COM",
        SerialBackend.POSIX_TTY: "POSIX tty",
    }[runtime.serial_backend]
    stage("PREFLIGHT", "probing USB application and MCUboot endpoints")
    print("USB upload", flush=True)
    print(f"  Host       : {host_label}", flush=True)
    print(f"  Transport  : {transport_label}", flush=True)
    print(
        f"  mcumgr     : {pathlib.Path(runtime.executable).name}",
        flush=True,
    )
    print(f"  Scan limit : {wait_seconds}s", flush=True)
    deadline = time.monotonic() + wait_seconds
    last_error = "no serial ports detected"
    last_busy_error = ""
    # 每个端口最多 1 Hz 探测，避免反复打开同一 COM 干扰 USB 重枚举或其他工具。
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
            if port_busy(output):
                last_busy_error = f"{port}: {output}"
                if explicit_port:
                    raise UploadError(
                        f"PORT_BUSY: {port} is already open by another process "
                        f"({output})"
                    )

            _, identity, identify_output = try_application_identify(
                codec,
                runtime.serial_backend,
                port,
                deadline,
                require_target_build=False,
            )
            if identity is not None:
                matches.append((port, identity.summary()))
                continue
            if identify_output:
                last_error = f"{port}: {identify_output[-240:]}"
            if port_busy(identify_output):
                last_busy_error = f"{port}: {identify_output}"
                if explicit_port:
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
            stage("PREFLIGHT_READY", f"{port} identified as {endpoint}")
            return port, endpoint
        time.sleep(0.25)

    # 自动发现会在整个窗口内等待串口释放；期限结束后优先保留排他占用根因，
    # 避免最后一次 deadline expired 覆盖真正可处理的错误。
    if last_busy_error:
        raise UploadError(
            "PORT_BUSY: automatic USB discovery could not open a detected "
            f"serial endpoint ({last_busy_error})"
        )
    raise UploadError(
        f"HOST_PREFLIGHT: no compatible endpoint was found within "
        f"{wait_seconds}s ({last_error})"
    )
