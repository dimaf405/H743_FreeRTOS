"""Recovery endpoint discovery and USB re-enumeration orchestration."""

from __future__ import annotations

import pathlib
import re
import time

from .application_recovery import (
    request_application_recovery,
    request_px4_bootstrap_recovery,
    try_application_identify,
)
from .mavlink import MavlinkCodec
from .mcumgr import try_image_list
from .models import (
    ApplicationIdentity,
    HostPlatform,
    McumgrRuntime,
    SerialBackend,
    UploadError,
    stage,
)
from .serial_discovery import (
    port_binding,
    ports_matching_binding,
    serial_ports,
)

PORT_BUSY_RE = re.compile(
    r"access is denied|permission denied|resource busy|being used by another process|"
    r"device or resource busy|访问被拒绝",
    re.IGNORECASE,
)

def port_busy(output: str) -> bool:
    return PORT_BUSY_RE.search(output) is not None


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


def wait_for_recovery(
    runtime: McumgrRuntime,
    codec: MavlinkCodec,
    explicit_port: str | None,
    wait_seconds: int,
    baud: int,
    mtu: int,
) -> tuple[str, str, ApplicationIdentity | None, str]:
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
        # Once an application is selected, only ports bound to the same physical
        # USB identity may become Recovery. Falling back to every serial port
        # could flash another board that enumerated during this reboot window.
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
                reboot_output = request_px4_bootstrap_recovery(
                    runtime.serial_backend, port
                )
                print(reboot_output)
                stage(
                    "REBOOT_REQUEST",
                    "operator-selected port received the complete PX4 "
                    "MAVLink/NSH reboot sequence; waiting for MCUboot on "
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
                codec, runtime.serial_backend, port, application_identity
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
                "; the selected endpoint returned neither the Dima legacy console "
                "identity nor the current Dima MAVLink identity; its application "
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
                    and expected_identity.uid is not None
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


def wait_for_recovery_endpoint(
    runtime: McumgrRuntime,
    binding: str,
    wait_seconds: int,
    baud: int,
    mtu: int,
) -> tuple[str, str]:
    stage(
        "RECOVERY_REENUM",
        f"waiting up to {wait_seconds}s for MCUboot Recovery",
    )
    deadline = time.monotonic() + wait_seconds
    last_error = "no serial ports detected"
    probe_after: dict[str, float] = {}
    while time.monotonic() < deadline:
        ports = ports_matching_binding(runtime, binding)
        matches: list[tuple[str, str]] = []
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
                matches.append((port, output))
            elif output:
                last_error = f"{port}: {output}"
        if len(matches) > 1:
            raise UploadError(
                "RECOVERY_REENUM: multiple MCUboot endpoints were identified: "
                + ", ".join(port for port, _ in matches)
            )
        if matches:
            port, output = matches[0]
            stage("SMP_LIST", f"connected to MCUboot Recovery on {port}")
            if output:
                print(output)
            return port, output
        time.sleep(0.25)
    raise UploadError(
        f"RECOVERY_REENUM: MCUboot Recovery was not reached within "
        f"{wait_seconds}s ({last_error})"
    )
