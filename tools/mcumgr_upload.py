#!/usr/bin/env python3
"""Upload, test, and boot the current signed image through MCUboot USB CDC."""

from __future__ import annotations

import argparse
import base64
import dataclasses
import enum
import glob
import json
import os
import pathlib
import queue
import re
import select
import shlex
import shutil
import subprocess
import sys
import threading
import time

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - only POSIX hosts use these modules.
    termios = None
    tty = None

from bootstrap_mcumgr import BootstrapError, ensure_mcumgr


DEFAULT_USB_CDC_BAUD = 921600
DEFAULT_SERIAL_MTU = 512
DEFAULT_MAX_WINDOW = 1
DEFAULT_CONFIRM_WAIT_SECONDS = 8
APP_IDENTIFY_TOKEN = "DIMA_ROVER_APP_V1"
APP_REBOOT_ACK = "DIMA_REBOOTING_BOOTLOADER"
APP_REBOOT_DENIED = "DIMA_REBOOT_DENIED_ARMED"
APP_IDENTIFY_REQUEST = b"\r\r\rdima identify\n"
APP_REBOOT_REQUEST = b"\r\r\rreboot -b\n"
IMAGE_SHA256_TLV = 0x10
MCUMGR_ERROR_RE = re.compile(
    r"^Error:[ \t]*([+-]?[0-9]+)[ \t]*\r?$", re.MULTILINE
)
MCUMGR_IMAGES_RE = re.compile(r"^Images:[ \t]*\r?$", re.MULTILINE)
PORT_BUSY_RE = re.compile(
    r"access is denied|permission denied|resource busy|being used by another process|"
    r"device or resource busy|访问被拒绝",
    re.IGNORECASE,
)


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


def stage(name: str, message: str) -> None:
    print(f"[{name}] {message}", flush=True)


def decode_output(output: bytes) -> str:
    return output.replace(b"\x00", b"").decode("utf-8", errors="replace").strip()


def powershell_output(script: str) -> str:
    executable = shutil.which("powershell.exe") or shutil.which("pwsh.exe")
    if executable is None:
        return ""
    try:
        completed = subprocess.run(
            [
                executable,
                "-NoLogo",
                "-NoProfile",
                "-NonInteractive",
                "-Command",
                "[Console]::OutputEncoding=[System.Text.Encoding]::UTF8;" + script,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=10,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return ""
    return decode_output(completed.stdout)


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


def windows_serial_ports() -> list[str]:
    output = powershell_output(
        "[System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object {$_}"
    )
    ports = set(re.findall(r"\bCOM[0-9]+\b", output, re.IGNORECASE))
    return sorted(ports, key=lambda port: int(port[3:]))


def posix_serial_ports() -> list[str]:
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    candidates = (
        by_id
        + sorted(glob.glob("/dev/ttyACM*"))
        + sorted(glob.glob("/dev/ttyUSB*"))
    )
    physical_ports: dict[str, str] = {}
    for candidate in candidates:
        path = pathlib.Path(candidate)
        try:
            physical_port = str(path.resolve(strict=True))
        except OSError:
            continue
        # by-id aliases are listed first and remain the stable displayed path.
        physical_ports.setdefault(physical_port, candidate)
    return sorted(physical_ports.values(), key=str.casefold)


def windows_serial_exchange(
    port: str, request: bytes, read_seconds: float
) -> tuple[bool, str]:
    """Write to a Windows COM port from WSL without requiring pyserial."""
    if re.fullmatch(r"COM[0-9]+", port, re.IGNORECASE) is None:
        return False, f"invalid Windows COM port: {port}"
    executable = shutil.which("powershell.exe") or shutil.which("pwsh.exe")
    if executable is None:
        return False, "PowerShell is unavailable for Windows serial access"

    encoded_request = base64.b64encode(request).decode("ascii")
    read_milliseconds = max(0, int(read_seconds * 1000.0))
    script = (
        "$ErrorActionPreference='Stop';"
        f"$serial=[System.IO.Ports.SerialPort]::new('{port.upper()}',115200,"
        "[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One);"
        "$serial.Encoding=[System.Text.Encoding]::ASCII;"
        "$serial.ReadTimeout=50;$serial.WriteTimeout=500;"
        "$serial.DtrEnable=$false;$serial.RtsEnable=$false;"
        "$written=$false;$builder=New-Object System.Text.StringBuilder;"
        "try{"
        "$serial.Open();$serial.DiscardInBuffer();"
        f"$payload=[Convert]::FromBase64String('{encoded_request}');"
        "$serial.BaseStream.Write($payload,0,$payload.Length);"
        "$serial.BaseStream.Flush();$written=$true;"
        f"$deadline=[DateTime]::UtcNow.AddMilliseconds({read_milliseconds});"
        "while([DateTime]::UtcNow -lt $deadline){"
        "try{$chunk=$serial.ReadExisting();"
        "if($chunk){[void]$builder.Append($chunk)}}"
        "catch{if(-not $written){throw};break};"
        "Start-Sleep -Milliseconds 10}"
        "}catch{if(-not $written){"
        "[Console]::Error.Write($_.Exception.Message);exit 2}}"
        "finally{try{if($serial.IsOpen){$serial.Close()}}catch{};"
        "$serial.Dispose()};"
        "[Console]::Out.Write($builder.ToString());"
        "if(-not $written){exit 2}"
    )
    try:
        completed = subprocess.run(
            [
                executable,
                "-NoLogo",
                "-NoProfile",
                "-NonInteractive",
                "-Command",
                "[Console]::OutputEncoding=[System.Text.Encoding]::UTF8;" + script,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=max(5.0, read_seconds + 3.0),
            check=False,
        )
    except subprocess.TimeoutExpired:
        return False, "Windows serial exchange timed out"

    output = decode_output(completed.stdout)
    if completed.returncode != 0:
        details = decode_output(completed.stderr or completed.stdout)
        return False, details or "Windows serial exchange failed"
    return True, output


def posix_serial_exchange(
    port: str, request: bytes, read_seconds: float
) -> tuple[bool, str]:
    """Perform a small raw serial exchange using only the Python standard library."""
    if termios is None or tty is None:
        return False, "POSIX serial support is unavailable"

    try:
        descriptor = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        return False, f"cannot open {port}: {error}"

    previous_attributes = None
    received = bytearray()
    sent = False
    try:
        previous_attributes = termios.tcgetattr(descriptor)
        tty.setraw(descriptor, when=termios.TCSANOW)
        attributes = termios.tcgetattr(descriptor)
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
        termios.tcflush(descriptor, termios.TCIOFLUSH)

        write_deadline = time.monotonic() + 0.5
        offset = 0
        while offset < len(request) and time.monotonic() < write_deadline:
            remaining = max(0.0, write_deadline - time.monotonic())
            _, writable, _ = select.select([], [descriptor], [], remaining)
            if not writable:
                break
            try:
                offset += os.write(descriptor, request[offset:])
            except BlockingIOError:
                continue
        sent = offset == len(request)

        read_deadline = time.monotonic() + read_seconds
        while sent and time.monotonic() < read_deadline:
            remaining = max(0.0, read_deadline - time.monotonic())
            try:
                readable, _, _ = select.select([descriptor], [], [], remaining)
            except (OSError, ValueError):
                break
            if not readable:
                break
            try:
                chunk = os.read(descriptor, 4096)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                break
            received.extend(chunk)
    except (OSError, termios.error) as error:
        return False, f"serial exchange on {port} failed: {error}"
    finally:
        if previous_attributes is not None:
            try:
                termios.tcsetattr(descriptor, termios.TCSANOW, previous_attributes)
            except (OSError, termios.error):
                pass
        os.close(descriptor)

    return sent, decode_output(bytes(received))


def serial_exchange(
    serial_backend: SerialBackend,
    port: str,
    request: bytes,
    read_seconds: float,
) -> tuple[bool, str]:
    if serial_backend == SerialBackend.WINDOWS_COM:
        return windows_serial_exchange(port, request, read_seconds)
    return posix_serial_exchange(port, request, read_seconds)


def try_application_identify(
    serial_backend: SerialBackend, port: str
) -> tuple[bool, bool, str]:
    sent, output = serial_exchange(
        serial_backend, port, APP_IDENTIFY_REQUEST, read_seconds=0.45
    )
    return sent, sent and APP_IDENTIFY_TOKEN in output, output


def request_application_recovery(
    serial_backend: SerialBackend, port: str
) -> str:
    sent, output = serial_exchange(
        serial_backend, port, APP_REBOOT_REQUEST, read_seconds=0.45
    )
    if not sent:
        raise UploadError(
            f"REBOOT_REQUEST: unable to send reboot -b to {port}: {output}"
        )
    if APP_REBOOT_DENIED in output:
        raise UploadError(
            "REBOOT_REQUEST: the application refused bootloader reboot because it is armed"
        )
    return output


def image_hash(imgtool: pathlib.Path, image: pathlib.Path) -> str:
    if not image.is_file():
        raise UploadError(f"signed upload image does not exist: {image}")
    if not imgtool.is_file():
        raise UploadError(f"imgtool does not exist: {imgtool}")

    completed = subprocess.run(
        [sys.executable, str(imgtool), "dumpinfo", "--format", "json", str(image)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if completed.returncode != 0:
        details = decode_output(completed.stderr or completed.stdout)
        raise UploadError(f"unable to inspect signed image: {details}")

    try:
        metadata = json.loads(completed.stdout)
        tlvs = metadata["tlv_area"]["tlvs"]
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        raise UploadError("imgtool returned malformed signed-image metadata") from error

    if not isinstance(tlvs, list):
        raise UploadError("imgtool returned malformed signed-image TLVs")
    sha_tlvs = [
        tlv
        for tlv in tlvs
        if isinstance(tlv, dict) and tlv.get("type") == IMAGE_SHA256_TLV
    ]
    if len(sha_tlvs) != 1:
        raise UploadError(
            f"signed image must contain exactly one SHA-256 TLV; found {len(sha_tlvs)}"
        )

    sha_tlv = sha_tlvs[0]
    digest = sha_tlv.get("data")
    if sha_tlv.get("len") != 32 or not isinstance(digest, str):
        raise UploadError("signed-image SHA-256 TLV is not 32 bytes")
    if re.fullmatch(r"[0-9a-fA-F]{64}", digest) is None:
        raise UploadError("signed-image SHA-256 TLV is not 64 hexadecimal characters")
    return digest.lower()


def wsl_windows_path(path: pathlib.Path) -> str:
    completed = subprocess.run(
        ["wslpath", "-w", str(path.resolve())],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    converted = decode_output(completed.stdout)
    if completed.returncode != 0 or not converted:
        raise UploadError(f"cannot convert image path for Windows mcumgr: {path}")
    return converted


def mcumgr_image_path(runtime: McumgrRuntime, path: pathlib.Path) -> str:
    if (
        runtime.host_platform == HostPlatform.WSL
        and runtime.executable_is_windows
    ):
        return wsl_windows_path(path)
    return str(path.resolve())


def mcumgr_command(
    executable: str,
    port: str,
    *arguments: str,
    baud: int = DEFAULT_USB_CDC_BAUD,
    mtu: int = DEFAULT_SERIAL_MTU,
) -> list[str]:
    return [
        executable,
        "--conntype",
        "serial",
        "--connstring",
        f"dev={port},baud={baud},mtu={mtu}",
        *arguments,
    ]


def protocol_error(output: str) -> str | None:
    for match in MCUMGR_ERROR_RE.finditer(output):
        if int(match.group(1)) != 0:
            return match.group(0)
    return None


def try_image_list(
    executable: str, port: str, baud: int, mtu: int
) -> tuple[bool, str]:
    command = mcumgr_command(
        executable,
        port,
        "--timeout",
        "0.5",
        "--tries",
        "1",
        "image",
        "list",
        baud=baud,
        mtu=mtu,
    )
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=1.5,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return False, "image list timed out"
    except OSError as error:
        return False, f"unable to start mcumgr: {error}"
    output = decode_output(completed.stdout)
    success = (
        completed.returncode == 0
        and protocol_error(output) is None
        and MCUMGR_IMAGES_RE.search(output) is not None
    )
    return success, output


def serial_ports(runtime: McumgrRuntime) -> list[str]:
    if runtime.serial_backend == SerialBackend.WINDOWS_COM:
        return windows_serial_ports()
    return posix_serial_ports()


def port_busy(output: str) -> bool:
    return PORT_BUSY_RE.search(output) is not None


def endpoint_preflight(
    runtime: McumgrRuntime,
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

            sent, identified, identify_output = try_application_identify(
                runtime.serial_backend, port
            )
            if identified:
                matches.append((port, "Dima Rover application"))
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
    explicit_port: str | None,
    wait_seconds: int,
    baud: int,
    mtu: int,
) -> tuple[str, str]:
    stage(
        "PORT_OPEN",
        "scanning for a Dima Rover application or MCUboot Recovery "
        f"for up to {wait_seconds}s",
    )
    deadline = time.monotonic() + wait_seconds
    last_error = "no serial ports detected"
    reboot_requested = False
    incompatible_response_seen = False
    probe_after: dict[str, float] = {}
    while time.monotonic() < deadline:
        # MCUMGR_PORT selects the application initially.  After reboot the OS
        # may assign a different COM/tty name, so return to full serial scanning.
        if explicit_port and not reboot_requested:
            ports = [explicit_port]
        else:
            ports = serial_ports(runtime)

        recovery_matches: list[tuple[str, str]] = []
        application_matches: list[tuple[str, str]] = []
        now = time.monotonic()
        for port in ports:
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

            sent, identified, identify_output = try_application_identify(
                runtime.serial_backend, port
            )
            if not identified:
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

            application_matches.append((port, identify_output))

        matched_ports = [port for port, _ in recovery_matches]
        matched_ports.extend(port for port, _ in application_matches)
        if len(matched_ports) > 1:
            raise UploadError(
                "multiple Dima protocol endpoints were identified: "
                + ", ".join(matched_ports)
                + "; set MCUMGR_PORT to the intended application port"
            )

        if recovery_matches:
            port, output = recovery_matches[0]
            stage("SMP_LIST", f"connected to MCUboot Recovery on {port}")
            if output:
                print(output)
            return port, output

        if application_matches:
            port, _ = application_matches[0]

            stage("APP_IDENTIFY", f"identified Dima Rover application on {port}")
            reboot_output = request_application_recovery(
                runtime.serial_backend, port
            )
            if reboot_output:
                print(reboot_output)
            if APP_REBOOT_ACK not in reboot_output:
                stage(
                    "REBOOT_REQUEST",
                    "reboot -b sent; waiting for USB re-enumeration",
                )
            else:
                stage(
                    "REBOOT_REQUEST",
                    "application accepted reboot -b; waiting for MCUboot",
                )
            reboot_requested = True
            probe_after.clear()
            last_error = f"{port}: application reboot requested; Recovery not enumerated yet"
        time.sleep(0.25)

    if not reboot_requested:
        if incompatible_response_seen:
            last_error += (
                "; the connected firmware does not support DIMA_ROVER_APP_V1; "
                "install the newly built Factory HEX once through an existing "
                "factory/debug programmer"
            )
        else:
            last_error += "; no compatible application identification response was received"
    raise UploadError(
        f"USB_REENUM: MCUboot Recovery was not reached within "
        f"{wait_seconds}s ({last_error})"
    )


def wait_for_application(
    runtime: McumgrRuntime,
    preferred_port: str | None,
    wait_seconds: int,
) -> str:
    stage(
        "APPLICATION_REENUM",
        f"waiting up to {wait_seconds}s for the application",
    )
    deadline = time.monotonic() + wait_seconds
    last_error = "no serial ports detected"
    probe_after: dict[str, float] = {}
    while time.monotonic() < deadline:
        ports = serial_ports(runtime)
        if preferred_port in ports:
            ports.remove(preferred_port)
            ports.insert(0, preferred_port)
        matches: list[str] = []
        now = time.monotonic()
        for port in ports:
            if now < probe_after.get(port, 0.0):
                continue
            probe_after[port] = now + 1.0
            sent, identified, output = try_application_identify(
                runtime.serial_backend, port
            )
            if identified:
                matches.append(port)
            elif output:
                last_error = f"{port}: {output[-240:]}"
            elif not sent:
                last_error = f"{port}: application identify could not be sent"
        if len(matches) > 1:
            raise UploadError(
                "APPLICATION_REENUM: multiple Dima applications were identified: "
                + ", ".join(matches)
            )
        if matches:
            stage("APPLICATION_RUNNING", f"application identified on {matches[0]}")
            return matches[0]
        time.sleep(0.25)
    raise UploadError(
        f"APPLICATION_REENUM: application was not reached within "
        f"{wait_seconds}s ({last_error})"
    )


def parse_image_states(output: str) -> list[ImageState]:
    states: list[ImageState] = []
    current: ImageState | None = None
    for raw_line in output.splitlines():
        line = raw_line.strip()
        header = re.fullmatch(r"image=([0-9]+) slot=([0-9]+)", line)
        if header is not None:
            current = ImageState(
                image=int(header.group(1)),
                slot=int(header.group(2)),
            )
            states.append(current)
            continue
        if current is None:
            continue
        if line.startswith("version:"):
            current.version = line.partition(":")[2].strip()
        elif line.startswith("flags:"):
            current.flags = set(line.partition(":")[2].strip().split())
        elif line.startswith("hash:"):
            digest = line.partition(":")[2].strip().casefold()
            if re.fullmatch(r"[0-9a-f]{64}", digest) is not None:
                current.digest = digest
    return states


def has_active_confirmed_image(output: str, digest: str) -> bool:
    return any(
        state.digest == digest
        and "active" in state.flags
        and "confirmed" in state.flags
        for state in parse_image_states(output)
    )


def has_secondary_image(output: str, digest: str) -> bool:
    return any(
        state.slot == 1 and state.digest == digest
        for state in parse_image_states(output)
    )


def has_pending_secondary_image(output: str, digest: str) -> bool:
    return any(
        state.slot == 1
        and state.digest == digest
        and "pending" in state.flags
        for state in parse_image_states(output)
    )


def run_mcumgr(
    executable: str,
    port: str,
    *arguments: str,
    expect_images: bool = False,
    timeout_seconds: int = 300,
    baud: int = DEFAULT_USB_CDC_BAUD,
    mtu: int = DEFAULT_SERIAL_MTU,
    measure_bytes: int | None = None,
) -> str:
    command = mcumgr_command(executable, port, *arguments, baud=baud, mtu=mtu)
    print("+ " + shlex.join(command), flush=True)
    started = time.monotonic()
    chunks: list[bytes] = []
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
    except OSError as error:
        raise UploadError(f"unable to start mcumgr: {error}") from error

    assert process.stdout is not None
    output_queue: queue.Queue[bytes | None] = queue.Queue()
    reader_errors: list[OSError] = []

    def read_output() -> None:
        try:
            while True:
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk:
                    break
                output_queue.put(chunk)
        except OSError as error:
            reader_errors.append(error)
        finally:
            output_queue.put(None)

    reader = threading.Thread(
        target=read_output,
        name="mcumgr-output",
        daemon=True,
    )
    reader.start()
    deadline = started + timeout_seconds
    timed_out = False
    reader_done = False
    last_byte = b"\n"
    while not reader_done:
        if not timed_out and time.monotonic() >= deadline:
            timed_out = True
            if process.poll() is None:
                process.kill()
        try:
            chunk = output_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        if chunk is None:
            reader_done = True
            continue
        chunks.append(chunk)
        last_byte = chunk[-1:]
        output_buffer = getattr(sys.stdout, "buffer", None)
        if output_buffer is not None:
            output_buffer.write(chunk)
            output_buffer.flush()
        else:
            sys.stdout.write(chunk.decode("utf-8", errors="replace"))
            sys.stdout.flush()

    process.wait()
    reader.join(timeout=1.0)
    if chunks and last_byte != b"\n":
        print()

    if timed_out:
        raise UploadError("mcumgr command timed out: " + " ".join(arguments))
    if reader_errors:
        raise UploadError(f"unable to read mcumgr output: {reader_errors[0]}")

    output = decode_output(b"".join(chunks))
    if process.returncode != 0:
        raise UploadError(
            f"mcumgr command failed with exit status {process.returncode}: "
            + " ".join(arguments)
        )
    device_error = protocol_error(output)
    if device_error is not None:
        raise UploadError(
            f"MCUboot rejected {' '.join(arguments)} ({device_error})"
        )
    if expect_images and MCUMGR_IMAGES_RE.search(output) is None:
        raise UploadError("mcumgr image list returned no Images section")
    if measure_bytes is not None:
        elapsed = time.monotonic() - started
        rate = measure_bytes / max(elapsed, 0.001) / 1024.0
        print(
            f"Transferred {measure_bytes} bytes in {elapsed:.2f}s "
            f"({rate:.2f} KiB/s)",
            flush=True,
        )
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=pathlib.Path)
    parser.add_argument("--imgtool", type=pathlib.Path)
    parser.add_argument("--mcumgr", default="mcumgr")
    parser.add_argument(
        "--tools-cache",
        type=pathlib.Path,
        default=pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools",
    )
    parser.add_argument("--port")
    parser.add_argument("--wait-seconds", type=int, default=60)
    parser.add_argument("--baud", type=int, default=DEFAULT_USB_CDC_BAUD)
    parser.add_argument("--mtu", type=int, default=DEFAULT_SERIAL_MTU)
    parser.add_argument("--max-window", type=int, default=DEFAULT_MAX_WINDOW)
    parser.add_argument(
        "--confirm-wait-seconds",
        type=int,
        default=DEFAULT_CONFIRM_WAIT_SECONDS,
    )
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--skip-confirm-verification", action="store_true")
    arguments = parser.parse_args()

    if arguments.wait_seconds <= 0:
        parser.error("--wait-seconds must be positive")
    if arguments.baud <= 0:
        parser.error("--baud must be positive")
    if arguments.mtu < 128 or arguments.mtu > 512:
        parser.error("--mtu must be between 128 and 512 for this MCUboot build")
    if arguments.max_window < 1 or arguments.max_window > 3:
        parser.error("--max-window must be between 1 and 3 for the 2048-byte RX ring")
    if arguments.confirm_wait_seconds <= 0:
        parser.error("--confirm-wait-seconds must be positive")

    runtime = resolve_mcumgr(
        arguments.mcumgr, arguments.port, arguments.tools_cache.expanduser()
    )
    if arguments.preflight_only:
        endpoint_preflight(
            runtime,
            arguments.port,
            arguments.wait_seconds,
            arguments.baud,
            arguments.mtu,
        )
        return 0

    if arguments.image is None or arguments.imgtool is None:
        parser.error("--image and --imgtool are required unless --preflight-only is used")

    image = arguments.image.resolve()
    digest = image_hash(arguments.imgtool.resolve(), image)
    stage("SIGN_VERIFY", f"signed image hash={digest}")
    port, initial_list = wait_for_recovery(
        runtime,
        arguments.port,
        arguments.wait_seconds,
        arguments.baud,
        arguments.mtu,
    )
    if has_active_confirmed_image(initial_list, digest) and not arguments.force:
        stage(
            "ALREADY_INSTALLED",
            "the signed image is already active and confirmed; use --force to rewrite it",
        )
        stage("RESET", "returning from MCUboot Recovery to the application")
        run_mcumgr(
            runtime.executable,
            port,
            "reset",
            baud=arguments.baud,
            mtu=arguments.mtu,
        )
        wait_for_application(runtime, port, arguments.wait_seconds)
        stage("COMPLETE", "firmware is already installed and the application is running")
        return 0

    upload_image = mcumgr_image_path(runtime, image)

    # The bootloader receives serial frames in a 2048-byte ring. Stop-and-wait
    # remains the production default until a larger window passes sustained
    # real-board acceptance.
    stage(
        "UPLOAD_SECONDARY",
        f"uploading {image.stat().st_size} bytes with mtu={arguments.mtu} "
        f"window={arguments.max_window}",
    )
    run_mcumgr(
        runtime.executable,
        port,
        "image",
        "upload",
        "-n",
        "2",
        "--maxwinsize",
        str(arguments.max_window),
        upload_image,
        timeout_seconds=900,
        baud=arguments.baud,
        mtu=arguments.mtu,
        measure_bytes=image.stat().st_size,
    )
    stage("VERIFY_SECONDARY", "reading MCUboot image state")
    secondary_list = run_mcumgr(
        runtime.executable,
        port,
        "image",
        "list",
        expect_images=True,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    if not has_secondary_image(secondary_list, digest):
        raise UploadError(
            "SECONDARY_HASH_MISMATCH: uploaded image hash was not found in slot 1"
        )
    stage("TEST", f"marking {digest} as the test image")
    run_mcumgr(
        runtime.executable,
        port,
        "image",
        "test",
        digest,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    stage("VERIFY_SECONDARY", "verifying pending/test state")
    pending_list = run_mcumgr(
        runtime.executable,
        port,
        "image",
        "list",
        expect_images=True,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    if not has_pending_secondary_image(pending_list, digest):
        raise UploadError(
            "PENDING_STATE_MISMATCH: uploaded image was not pending in slot 1"
        )
    stage("RESET", "resetting into the test image")
    run_mcumgr(
        runtime.executable,
        port,
        "reset",
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    application_port = wait_for_application(
        runtime, port, arguments.wait_seconds
    )
    if arguments.skip_confirm_verification:
        stage(
            "COMPLETE",
            "upload and swap completed; application confirmation was not probed",
        )
        return 0

    stage(
        "HEALTH_CONFIRM",
        f"waiting {arguments.confirm_wait_seconds}s for application health confirmation",
    )
    time.sleep(arguments.confirm_wait_seconds)
    confirm_port, confirm_list = wait_for_recovery(
        runtime,
        application_port,
        arguments.wait_seconds,
        arguments.baud,
        arguments.mtu,
    )
    confirmed = has_active_confirmed_image(confirm_list, digest)
    stage("RESET", "returning from confirmation probe to the application")
    run_mcumgr(
        runtime.executable,
        confirm_port,
        "reset",
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    wait_for_application(runtime, confirm_port, arguments.wait_seconds)
    if not confirmed:
        raise UploadError(
            "IMAGE_NOT_CONFIRMED: the uploaded image did not become active and confirmed"
        )
    stage("COMPLETE", "upload, swap, health confirmation, and application restart passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except UploadError as error:
        print(f"upload failed: {error}", file=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("upload cancelled", file=sys.stderr)
        raise SystemExit(130)
