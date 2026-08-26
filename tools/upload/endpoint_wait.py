"""等待 MCUboot Recovery 与 Application USB 重枚举，并保持物理设备身份连续。"""

from __future__ import annotations

import time

from .endpoint_errors import port_busy
from .recovery_request import (
    request_explicit_port_recovery,
    request_application_recovery,
    try_application_identify,
)
from .mavlink import MavlinkCodec
from .mcumgr import try_image_list
from .models import (
    ApplicationIdentity,
    McumgrRuntime,
    UploadError,
    stage,
)
from .serial_discovery import (
    port_binding,
    ports_matching_binding,
    serial_ports,
)

def wait_for_recovery(
    runtime: McumgrRuntime,
    codec: MavlinkCodec,
    explicit_port: str | None,
    wait_seconds: int,
    baud: int,
    mtu: int,
) -> tuple[str, str, ApplicationIdentity | None, str]:
    """识别应用、请求 Recovery，并仅在同一物理 USB binding 上接受 MCUboot。"""
    stage(
        "PORT_OPEN",
        "scanning for a Dima Rover application or MCUboot Recovery "
        f"for up to {wait_seconds}s",
    )
    deadline = time.monotonic() + wait_seconds
    last_error = "no serial ports detected"
    reboot_requested = False
    incompatible_response_seen = False
    application_identity: ApplicationIdentity | None = None
    application_binding: str | None = None
    probe_after: dict[str, float] = {}
    while time.monotonic() < deadline:
        # 选中应用后只扫描同一物理 USB binding；若退回所有串口，重启窗口中新插入的
        # 另一块板可能被误烧录。
        if reboot_requested:
            ports = (
                ports_matching_binding(runtime, application_binding)
                if application_binding is not None
                else []
            )
        elif explicit_port:
            ports = [explicit_port]
        else:
            ports = serial_ports(runtime)

        recovery_matches: list[tuple[str, str]] = []
        application_matches: list[tuple[str, ApplicationIdentity, str]] = []
        now = time.monotonic()
        for port in ports:
            if time.monotonic() >= deadline:
                break
            if now < probe_after.get(port, 0.0):
                continue
            probe_after[port] = now + 1.0
            success, output = try_image_list(
                runtime.executable, port, baud, mtu
            )
            if success:
                recovery_matches.append((port, output))
                continue
            if output:
                last_error = f"{port}: {output}"
            if explicit_port and not reboot_requested and port_busy(output):
                raise UploadError(
                    f"PORT_BUSY: {port} is already open by another process ({output})"
                )

            if reboot_requested:
                continue

            if explicit_port:
                application_binding = port_binding(runtime, port)
                if application_binding is None:
                    raise UploadError(
                        f"DEVICE_BINDING: unable to bind {port} to a physical USB device"
                    )
                reboot_output = request_explicit_port_recovery(
                    runtime.serial_backend, port
                )
                print(reboot_output)
                stage(
                    "REBOOT_REQUEST",
                    "operator-selected port received the PX4 MAVLink reboot "
                    "sequence; waiting for MCUboot on "
                    "the same physical USB device",
                )
                reboot_requested = True
                probe_after.clear()
                last_error = (
                    f"{port}: PX4 explicit-port reboot sent; "
                    "Recovery not enumerated yet"
                )
                break

            sent, identity, identify_output = try_application_identify(
                codec, runtime.serial_backend, port, deadline
            )
            if identity is None:
                if sent and identify_output:
                    incompatible_response_seen = True
                    last_error = f"{port}: {identify_output[-240:]}"
                elif identify_output:
                    last_error = f"{port}: {identify_output[-240:]}"
                if (
                    explicit_port
                    and not reboot_requested
                    and port_busy(identify_output)
                ):
                    raise UploadError(
                        f"PORT_BUSY: {port} is already open by another process "
                        f"({identify_output})"
                    )
                continue

            application_matches.append((port, identity, identify_output))

        matched_ports = [port for port, _ in recovery_matches]
        matched_ports.extend(port for port, _, _ in application_matches)
        if len(matched_ports) > 1:
            raise UploadError(
                "multiple Dima protocol endpoints were identified: "
                + ", ".join(matched_ports)
                + "; set MCUMGR_PORT to the intended application port"
            )

        if recovery_matches:
            port, output = recovery_matches[0]
            recovery_binding = application_binding or port_binding(runtime, port)
            if recovery_binding is None:
                raise UploadError(
                    f"DEVICE_BINDING: unable to bind {port} to a physical USB device"
                )
            stage("SMP_LIST", f"connected to MCUboot Recovery on {port}")
            if output:
                print(output)
            return port, output, application_identity, recovery_binding

        if application_matches:
            port, application_identity, _ = application_matches[0]
            application_binding = port_binding(runtime, port)
            if application_binding is None:
                raise UploadError(
                    f"DEVICE_BINDING: unable to bind {port} to a physical USB device"
                )

            stage(
                "APP_IDENTIFY",
                f"identified {application_identity.summary()} on {port}",
            )
            reboot_acknowledged, reboot_output = request_application_recovery(
                codec, runtime.serial_backend, port
            )
            if reboot_output:
                print(reboot_output)
            if reboot_acknowledged:
                stage(
                    "REBOOT_REQUEST",
                    "application accepted Recovery reboot; waiting for MCUboot",
                )
            else:
                stage(
                    "REBOOT_REQUEST",
                    "automatic reboot sequence sent; ACK was not required; "
                    "waiting for MCUboot on the same physical USB device",
                )
            reboot_requested = True
            probe_after.clear()
            last_error = f"{port}: application reboot requested; Recovery not enumerated yet"
        time.sleep(0.25)

    if not reboot_requested:
        if incompatible_response_seen:
            last_error += (
                "; the selected endpoint did not return the current Dima MAVLink "
                "identity; its application "
                "runtime may be incompatible or unresponsive, so the uploader "
                "will not issue a blind reboot"
            )
        else:
            last_error += "; no compatible application identification response was received"
    raise UploadError(
        f"USB_REENUM: MCUboot Recovery was not reached within "
        f"{wait_seconds}s ({last_error}); Recovery must match the selected "
        "application USB identity"
    )


def wait_for_application(
    runtime: McumgrRuntime,
    codec: MavlinkCodec,
    preferred_port: str | None,
    wait_seconds: int,
    expected_identity: ApplicationIdentity | None = None,
    expected_binding: str | None = None,
) -> tuple[str, ApplicationIdentity, str]:
    """复位后要求应用重新出现，并核对 UID 与原物理 binding，防止错认另一块板。"""
    stage(
        "APPLICATION_REENUM",
        f"waiting up to {wait_seconds}s for the application",
    )
    deadline = time.monotonic() + wait_seconds
    last_error = "no serial ports detected"
    probe_after: dict[str, float] = {}
    while time.monotonic() < deadline:
        ports = (
            ports_matching_binding(runtime, expected_binding)
            if expected_binding is not None
            else ([preferred_port] if preferred_port else serial_ports(runtime))
        )
        matches: list[tuple[str, ApplicationIdentity, str]] = []
        now = time.monotonic()
        for port in ports:
            if time.monotonic() >= deadline:
                break
            if now < probe_after.get(port, 0.0):
                continue
            probe_after[port] = now + 1.0
            sent, identity, output = try_application_identify(
                codec, runtime.serial_backend, port, deadline
            )
            if identity is not None:
                if (
                    expected_identity is not None
                    and identity.uid != expected_identity.uid
                ):
                    last_error = (
                        f"{port}: Dima UID changed from "
                        f"0x{expected_identity.uid:016x} to {identity.uid!r}"
                    )
                    continue
                binding = expected_binding or port_binding(runtime, port)
                if binding is None:
                    last_error = f"{port}: unable to bind application USB identity"
                    continue
                matches.append((port, identity, binding))
            elif output:
                last_error = f"{port}: {output[-240:]}"
            elif not sent:
                last_error = f"{port}: application identify could not be sent"
        if len(matches) > 1:
            raise UploadError(
                "APPLICATION_REENUM: multiple Dima applications were identified: "
                + ", ".join(port for port, _, _ in matches)
            )
        if matches:
            port, identity, binding = matches[0]
            stage(
                "APPLICATION_RUNNING",
                f"{identity.summary()} identified on {port}",
            )
            return port, identity, binding
        time.sleep(0.25)
    raise UploadError(
        f"APPLICATION_REENUM: application was not reached within "
        f"{wait_seconds}s ({last_error})"
    )
