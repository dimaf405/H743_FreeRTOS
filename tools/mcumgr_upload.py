#!/usr/bin/env python3
"""Upload, test, and boot the current signed image through MCUboot USB CDC."""

from __future__ import annotations

import argparse
import base64
import binascii
import dataclasses
import enum
import errno
import glob
import hashlib
import json
import locale
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

try:
    import winreg
except ImportError:  # pragma: no cover - only native Windows provides winreg.
    winreg = None

from bootstrap_mcumgr import BootstrapError, ensure_mcumgr


DEFAULT_USB_CDC_BAUD = 921600
DEFAULT_SERIAL_MTU = 512
DEFAULT_MAX_WINDOW = 1
DEFAULT_CONFIRM_WAIT_SECONDS = 8
LEGACY_APP_IDENTIFY_TOKEN = "DIMA_ROVER_APP_V1"
LEGACY_APP_REBOOT_ACK = "DIMA_REBOOTING_BOOTLOADER"
LEGACY_APP_REBOOT_DENIED = "DIMA_REBOOT_DENIED_ARMED"
LEGACY_APP_IDENTIFY_REQUEST = b"\r\r\rdima identify\n"
LEGACY_APP_REBOOT_REQUEST = b"\r\r\rreboot -b\n"
# PX4-Autopilot Tools/px4_uploader.py sends fixed MAVLink v1 COMMAND_LONG
# frames followed by the NSH reboot command. Both frames carry
# MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(param1=3): broadcast 0/0 first, then the
# conventional autopilot target 1/0. The exact MAVLink+NSH sequence is used
# only when the operator explicitly selected a serial port; automatic discovery
# still identifies the Dima application before issuing a Commander-arbitrated
# MAVLink reboot.
PX4_MAVLINK_REBOOT_BROADCAST = bytes.fromhex(
    "fe2145ff004c00004040000000000000000000000000"
    "000000000000000000000000f600000000cc37"
)
PX4_MAVLINK_REBOOT_TARGETED = bytes.fromhex(
    "fe2172ff004c00004040000000000000000000000000"
    "000000000000000000000000f600010000536b"
)
PX4_NSH_INIT = b"\r\r\r"
PX4_NSH_REBOOT_BOOTLOADER = b"reboot -b\n"
PX4_REBOOT_ATTEMPTS = 3
PX4_REBOOT_SETTLE_SECONDS = 0.35
PX4_BOOTSTRAP_MAVLINK_SETTLE_SECONDS = 0.1
PX4_BOOTSTRAP_NSH_INIT_SETTLE_SECONDS = 0.05
PX4_BOOTSTRAP_NSH_REBOOT_SETTLE_SECONDS = 0.2
MAVLINK_GCS_SYSTEM_ID = 255
MAVLINK_GCS_COMPONENT_ID = 190
MAVLINK_APP_SYSTEM_ID = 1
MAVLINK_APP_COMPONENT_ID = 1
DIMA_FLIGHT_SW_VERSION = 0x00010000
DIMA_BOARD_VERSION = 1
# Runtime-file digests from the size/SHA-256-pinned pymavlink 2.4.47 archive.
PINNED_PYMAVLINK_RUNTIME_SHA256 = {
    "__init__.py": "d902c5d877504a9098956ddf5c4a6321ba8e432815a78b279af4d8b79c939a5f",
    "dialects/__init__.py": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "dialects/v20/__init__.py": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "dialects/v20/common.py": "11761aba1f8eafcaceaebf02bb81a5836c823d83da28bcb405fc30ade11b02d7",
}
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
    protocol: str
    uid: int | None = None
    flight_sw_version: int | None = None
    board_version: int | None = None

    def summary(self) -> str:
        if self.protocol == "mavlink":
            assert self.uid is not None
            assert self.flight_sw_version is not None
            assert self.board_version is not None
            return (
                "Dima Rover MAVLink "
                f"version=0x{self.flight_sw_version:08x} "
                f"board={self.board_version} uid=0x{self.uid:016x}"
            )
        return "Dima Rover legacy console"


class MavlinkCodec:
    """Pinned pymavlink codec without mavutil or a pyserial dependency."""

    def __init__(self, dialect: object) -> None:
        self._dialect = dialect
        self._encoder = dialect.MAVLink(  # type: ignore[attr-defined]
            None,
            srcSystem=MAVLINK_GCS_SYSTEM_ID,
            srcComponent=MAVLINK_GCS_COMPONENT_ID,
        )
        parameter_bytewise_capability = getattr(
            dialect, "MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE", None
        )
        if parameter_bytewise_capability is None:
            parameter_bytewise_capability = (
                dialect.MAV_PROTOCOL_CAPABILITY_PARAM_UNION  # type: ignore[attr-defined]
            )
        self._expected_capabilities = (
            dialect.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT  # type: ignore[attr-defined]
            | dialect.MAV_PROTOCOL_CAPABILITY_COMMAND_INT  # type: ignore[attr-defined]
            | parameter_bytewise_capability
            | dialect.MAV_PROTOCOL_CAPABILITY_MAVLINK2  # type: ignore[attr-defined]
        )

    def command_long(self, command: int, param1: float) -> bytes:
        message = self._encoder.command_long_encode(
            MAVLINK_APP_SYSTEM_ID,
            MAVLINK_APP_COMPONENT_ID,
            command,
            0,
            param1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        frame = message.pack(self._encoder, force_mavlink1=False)
        self._encoder.seq = (self._encoder.seq + 1) & 0xFF
        return frame

    def identify_request(self) -> bytes:
        request_message = self._dialect.MAV_CMD_REQUEST_MESSAGE
        return self.command_long(request_message, 0.0) + self.command_long(
            request_message, float(self._dialect.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        )

    def parse(self, payload: bytes) -> list[object]:
        parser = self._dialect.MAVLink(None)
        parser.robust_parsing = True
        return list(parser.parse_buffer(payload) or [])

    @staticmethod
    def _source_is_application(message: object) -> bool:
        return (
            message.get_srcSystem() == MAVLINK_APP_SYSTEM_ID
            and message.get_srcComponent() == MAVLINK_APP_COMPONENT_ID
        )

    def application_identity(self, payload: bytes) -> ApplicationIdentity | None:
        heartbeat = None
        version = None
        for message in self.parse(payload):
            if not self._source_is_application(message):
                continue
            if message.get_type() == "HEARTBEAT":
                heartbeat = message
            elif message.get_type() == "AUTOPILOT_VERSION":
                version = message
        if heartbeat is None or version is None:
            return None
        if (
            heartbeat.type != self._dialect.MAV_TYPE_GROUND_ROVER
            or heartbeat.autopilot != self._dialect.MAV_AUTOPILOT_PX4
            or int(version.flight_sw_version) != DIMA_FLIGHT_SW_VERSION
            or int(version.board_version) != DIMA_BOARD_VERSION
            or (
                int(version.capabilities) & self._expected_capabilities
            ) != self._expected_capabilities
            or int(version.uid) == 0
        ):
            return None
        return ApplicationIdentity(
            protocol="mavlink",
            uid=int(version.uid),
            flight_sw_version=int(version.flight_sw_version),
            board_version=int(version.board_version),
        )

    def reboot_ack(self, payload: bytes) -> tuple[bool | None, str]:
        command = self._dialect.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        for message in self.parse(payload):
            if (
                message.get_type() != "COMMAND_ACK"
                or not self._source_is_application(message)
                or int(message.command) != command
            ):
                continue
            result = int(message.result)
            mode = int(message.result_param2)
            # Current firmware directs the ACK to the PX4 frame's 255/0
            # source and reports mode 3 in result_param2.  The already deployed
            # Phase-3 firmware left the ACK target and result_param2 at zero;
            # accept that legacy encoding on this point-to-point USB link.
            if int(message.target_system) not in (0, MAVLINK_GCS_SYSTEM_ID):
                continue
            if int(message.target_component) not in (
                0,
                MAVLINK_GCS_COMPONENT_ID,
            ):
                continue
            if result == self._dialect.MAV_RESULT_ACCEPTED and mode in (0, 3):
                suffix = "legacy mode field" if mode == 0 else "mode 3"
                return True, f"MAVLink reboot ACK accepted ({suffix})"
            return False, f"MAVLink reboot was rejected (result={result}, mode={mode})"
        return None, "no matching MAVLink reboot ACK was received"


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


def resolve_mavlink_codec(tools_cache: pathlib.Path) -> MavlinkCodec:
    tools_directory = pathlib.Path(__file__).resolve().parent / "mavlink"
    bootstrap_directory = str(tools_directory)
    if bootstrap_directory not in sys.path:
        sys.path.insert(0, bootstrap_directory)
    try:
        from bootstrap_pymavlink import (  # type: ignore[import-not-found]
            BootstrapError as MavlinkBootstrapError,
            provision_pymavlink,
        )
    except ImportError as error:
        raise UploadError(
            "the pinned MAVLink bootstrap is unavailable under tools/mavlink"
        ) from error

    lock_path = tools_directory / "mavlink.lock.json"
    try:
        package_root = provision_pymavlink(tools_cache, lock_path)
    except MavlinkBootstrapError as error:
        raise UploadError(str(error)) from error

    for relative, expected_digest in PINNED_PYMAVLINK_RUNTIME_SHA256.items():
        runtime_file = package_root / relative
        try:
            actual_digest = hashlib.sha256(runtime_file.read_bytes()).hexdigest()
        except OSError as error:
            raise UploadError(
                f"unable to verify cached pymavlink runtime {runtime_file}: {error}"
            ) from error
        if actual_digest != expected_digest:
            raise UploadError(
                f"cached pymavlink runtime integrity check failed: {runtime_file}"
            )

    package_parent = str(package_root.parent)
    if package_parent not in sys.path:
        sys.path.insert(0, package_parent)
    try:
        import pymavlink
        from pymavlink.dialects.v20 import common
    except ImportError as error:
        raise UploadError(f"unable to import pinned pymavlink: {error}") from error

    imported_root = pathlib.Path(pymavlink.__file__).resolve().parent
    if imported_root != package_root.resolve():
        raise UploadError(
            "the imported pymavlink package did not come from the pinned tools cache"
        )
    return MavlinkCodec(common)


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


def powershell_output(script: str, timeout_seconds: float = 3.0) -> str:
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
            timeout=timeout_seconds,
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


def windows_registry_present_ports() -> set[str]:
    if winreg is None:
        return set()
    ports: set[str] = set()
    try:
        with winreg.OpenKey(
            winreg.HKEY_LOCAL_MACHINE,
            r"HARDWARE\DEVICEMAP\SERIALCOMM",
        ) as serial_key:
            index = 0
            while True:
                try:
                    _, value, _ = winreg.EnumValue(serial_key, index)
                except OSError:
                    break
                index += 1
                port = str(value).upper()
                if re.fullmatch(r"COM[0-9]+", port) is not None:
                    ports.add(port)
    except OSError:
        return set()
    return ports


def windows_instance_present(instance_id: str) -> bool | None:
    """Use Configuration Manager to reject phantom Windows devnodes."""
    if os.name != "nt":
        return None
    try:
        import ctypes

        device_instance = ctypes.c_ulong()
        locate = ctypes.WinDLL("cfgmgr32").CM_Locate_DevNodeW
        locate.argtypes = [
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.c_wchar_p,
            ctypes.c_ulong,
        ]
        locate.restype = ctypes.c_ulong
        # CM_LOCATE_DEVNODE_NORMAL excludes historical phantom instances.
        return locate(ctypes.byref(device_instance), instance_id, 0) == 0
    except (AttributeError, OSError):
        return None


def windows_registry_port_binding_candidates() -> dict[str, dict[str, str]]:
    if winreg is None:
        return {}
    present_ports = windows_registry_present_ports()
    if not present_ports:
        return {}

    enum_root_path = r"SYSTEM\CurrentControlSet\Enum"
    candidates: dict[str, dict[str, str]] = {}
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, enum_root_path) as enum_root:
            enumerator_count = winreg.QueryInfoKey(enum_root)[0]
            for enumerator_index in range(enumerator_count):
                enumerator = winreg.EnumKey(enum_root, enumerator_index)
                with winreg.OpenKey(enum_root, enumerator) as enumerator_key:
                    device_count = winreg.QueryInfoKey(enumerator_key)[0]
                    for device_index in range(device_count):
                        device = winreg.EnumKey(enumerator_key, device_index)
                        with winreg.OpenKey(enumerator_key, device) as device_key:
                            instance_count = winreg.QueryInfoKey(device_key)[0]
                            for instance_index in range(instance_count):
                                instance = winreg.EnumKey(device_key, instance_index)
                                instance_path = (
                                    f"{enum_root_path}\\{enumerator}\\{device}\\{instance}"
                                )
                                try:
                                    with winreg.OpenKey(
                                        winreg.HKEY_LOCAL_MACHINE, instance_path
                                    ) as instance_key:
                                        with winreg.OpenKey(
                                            instance_key, r"Device Parameters"
                                        ) as parameters_key:
                                            port = str(
                                                winreg.QueryValueEx(
                                                    parameters_key, "PortName"
                                                )[0]
                                            ).upper()
                                        if port not in present_ports:
                                            continue
                                        pnp_instance = (
                                            f"{enumerator}\\{device}\\{instance}"
                                        ).casefold()
                                        if windows_instance_present(pnp_instance) is not True:
                                            continue
                                        try:
                                            container = str(
                                                winreg.QueryValueEx(
                                                    instance_key, "ContainerID"
                                                )[0]
                                            ).strip("{} ").casefold()
                                        except OSError:
                                            container = ""
                                except OSError:
                                    continue

                                if container and container != (
                                    "00000000-0000-0000-0000-000000000000"
                                ):
                                    binding = f"windows-container:{container}"
                                else:
                                    binding = f"windows-instance:{pnp_instance}"
                                candidates.setdefault(port, {})[
                                    pnp_instance
                                ] = binding
    except OSError:
        return {}
    return candidates


def windows_cim_port_instances() -> dict[str, str]:
    script = (
        "@(Get-CimInstance Win32_SerialPort | "
        "Select-Object DeviceID,PNPDeviceID) | ConvertTo-Json -Compress"
    )
    output = powershell_output(script)
    if not output:
        return {}
    try:
        records = json.loads(output)
    except json.JSONDecodeError:
        return {}
    if isinstance(records, dict):
        records = [records]
    if not isinstance(records, list):
        return {}

    instances: dict[str, str] = {}
    for record in records:
        if not isinstance(record, dict):
            continue
        port = str(record.get("DeviceID", "")).upper()
        if re.fullmatch(r"COM[0-9]+", port) is None:
            continue
        instance = str(record.get("PNPDeviceID", "")).strip().casefold()
        if instance:
            instances[port] = instance
    return instances


def windows_serial_ports() -> list[str]:
    registry_ports = windows_registry_present_ports()
    if registry_ports:
        return sorted(registry_ports, key=lambda port: int(port[3:]))
    output = powershell_output(
        "[System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object {$_}"
    )
    ports = set(re.findall(r"\bCOM[0-9]+\b", output, re.IGNORECASE))
    return sorted(ports, key=lambda port: int(port[3:]))


def windows_port_bindings() -> dict[str, str]:
    candidates = windows_registry_port_binding_candidates()
    bindings: dict[str, str] = {}
    for port, port_candidates in candidates.items():
        unique_bindings = set(port_candidates.values())
        if len(unique_bindings) == 1:
            bindings[port] = next(iter(unique_bindings))
    present_ports = windows_registry_present_ports()
    if present_ports and bindings.keys() >= present_ports:
        return bindings

    # WSL has no winreg, while native Windows reaches this path only for ports
    # missing or ambiguous in Enum.  Merge per port instead of allowing one
    # successful registry entry to suppress fallback data for every other COM.
    for port, instance in windows_cim_port_instances().items():
        if port in bindings:
            continue
        exact = candidates.get(port, {}).get(instance)
        bindings[port] = exact or f"windows-instance:{instance}"
    return bindings


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


def posix_port_binding(port: str) -> str | None:
    try:
        tty_name = pathlib.Path(port).resolve(strict=True).name
        device = (pathlib.Path("/sys/class/tty") / tty_name / "device").resolve(
            strict=True
        )
    except OSError:
        return None

    for candidate in (device, *device.parents):
        vendor_path = candidate / "idVendor"
        if not vendor_path.is_file():
            continue
        try:
            vendor = vendor_path.read_text(encoding="ascii").strip().casefold()
            serial_path = candidate / "serial"
            if serial_path.is_file():
                serial = serial_path.read_text(encoding="utf-8").strip().casefold()
                if serial:
                    return f"usb-serial:{vendor}:{serial}"
            bus = (candidate / "busnum").read_text(encoding="ascii").strip()
            devpath = (candidate / "devpath").read_text(encoding="ascii").strip()
            return f"usb-path:{bus}:{devpath}"
        except OSError:
            return None
    return None


def port_binding(runtime: McumgrRuntime, port: str) -> str | None:
    if runtime.serial_backend == SerialBackend.WINDOWS_COM:
        return windows_port_bindings().get(port.upper())
    return posix_port_binding(port)


def ports_matching_binding(runtime: McumgrRuntime, binding: str) -> list[str]:
    if runtime.serial_backend == SerialBackend.WINDOWS_COM:
        bindings = windows_port_bindings()
        return sorted(
            (port for port, candidate in bindings.items() if candidate == binding),
            key=lambda port: int(port[3:]),
        )
    return [
        port for port in serial_ports(runtime) if posix_port_binding(port) == binding
    ]


def windows_serial_exchange_bytes(
    port: str, request: bytes, read_seconds: float
) -> tuple[bool, bytes, str]:
    """Binary-safe Windows COM exchange without requiring pyserial."""
    if re.fullmatch(r"COM[0-9]+", port, re.IGNORECASE) is None:
        return False, b"", f"invalid Windows COM port: {port}"
    executable = shutil.which("powershell.exe") or shutil.which("pwsh.exe")
    if executable is None:
        return False, b"", "PowerShell is unavailable for Windows serial access"

    encoded_request = base64.b64encode(request).decode("ascii")
    read_milliseconds = max(0, int(read_seconds * 1000.0))
    script = (
        "$ErrorActionPreference='Stop';"
        f"$serial=[System.IO.Ports.SerialPort]::new('{port.upper()}',115200,"
        "[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One);"
        "$serial.ReadTimeout=50;$serial.WriteTimeout=500;"
        "$serial.DtrEnable=$false;$serial.RtsEnable=$false;"
        "$written=$false;$buffer=New-Object byte[] 4096;"
        "$received=New-Object System.IO.MemoryStream;"
        "try{"
        "$serial.Open();$serial.DiscardInBuffer();"
        f"$payload=[Convert]::FromBase64String('{encoded_request}');"
        "if($payload.Length -gt 0){$serial.Write($payload,0,$payload.Length)};"
        "$written=$true;"
        f"$deadline=[DateTime]::UtcNow.AddMilliseconds({read_milliseconds});"
        "while([DateTime]::UtcNow -lt $deadline){"
        "try{$available=$serial.BytesToRead;"
        "if($available -gt 0){"
        "$count=$serial.Read($buffer,0,[Math]::Min($buffer.Length,$available));"
        "if($count -gt 0){$received.Write($buffer,0,$count)}}}"
        "catch{if(-not $written){throw};break};"
        "Start-Sleep -Milliseconds 10}"
        "}catch{if(-not $written){"
        "[Console]::Error.Write($_.Exception.Message);exit 2}}"
        "finally{try{if($serial.IsOpen){$serial.Close()}}catch{};"
        "$serial.Dispose()};"
        "$encoded=[Convert]::ToBase64String($received.ToArray());"
        "$received.Dispose();[Console]::Out.Write($encoded);"
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
            timeout=max(2.0, read_seconds + 1.0),
            check=False,
        )
    except subprocess.TimeoutExpired:
        return False, b"", "Windows serial exchange timed out"

    if completed.returncode != 0:
        details = decode_output(completed.stderr or completed.stdout)
        return False, b"", details or "Windows serial exchange failed"
    try:
        output = base64.b64decode(completed.stdout.strip(), validate=True)
    except (binascii.Error, ValueError) as error:
        return False, b"", f"Windows serial exchange returned invalid Base64: {error}"
    return True, output, ""


def posix_serial_exchange_bytes(
    port: str, request: bytes, read_seconds: float
) -> tuple[bool, bytes, str]:
    """Perform a small raw serial exchange using only the Python standard library."""
    if termios is None or tty is None:
        return False, b"", "POSIX serial support is unavailable"

    try:
        descriptor = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        return False, b"", f"cannot open {port}: {error}"

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
        return False, b"", f"serial exchange on {port} failed: {error}"
    finally:
        if previous_attributes is not None:
            try:
                termios.tcsetattr(descriptor, termios.TCSANOW, previous_attributes)
            except (OSError, termios.error):
                pass
        os.close(descriptor)

    if not sent:
        return False, bytes(received), f"serial write to {port} did not complete"
    return True, bytes(received), ""


def serial_exchange_bytes(
    serial_backend: SerialBackend,
    port: str,
    request: bytes,
    read_seconds: float,
) -> tuple[bool, bytes, str]:
    if serial_backend == SerialBackend.WINDOWS_COM:
        return windows_serial_exchange_bytes(port, request, read_seconds)
    return posix_serial_exchange_bytes(port, request, read_seconds)


def serial_exchange(
    serial_backend: SerialBackend,
    port: str,
    request: bytes,
    read_seconds: float,
) -> tuple[bool, str]:
    sent, output, error = serial_exchange_bytes(
        serial_backend, port, request, read_seconds
    )
    return sent, decode_output(output) if sent else error


def px4_mavlink_reboot_stages() -> tuple[SerialWriteStage, ...]:
    """PX4-style automatic reboot writes, kept on one open serial session."""
    stages: list[SerialWriteStage] = []
    for attempt in range(PX4_REBOOT_ATTEMPTS):
        stages.extend(
            (
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_BROADCAST,
                    # PX4 clears buffers before every attempt because it never
                    # inspects application replies.  Dima clears only once so
                    # a late DENIED ACK cannot be discarded between retries.
                    reset_buffers=attempt == 0,
                ),
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_TARGETED,
                    delay_seconds=PX4_REBOOT_SETTLE_SECONDS,
                ),
            )
        )
    return tuple(stages)


def px4_bootstrap_reboot_stages() -> tuple[SerialWriteStage, ...]:
    """Exact PX4 MAVLink+NSH bootloader kick for an operator-selected port."""
    stages: list[SerialWriteStage] = []
    for _ in range(PX4_REBOOT_ATTEMPTS):
        stages.extend(
            (
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_BROADCAST,
                    reset_buffers=True,
                ),
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_TARGETED,
                    delay_seconds=PX4_BOOTSTRAP_MAVLINK_SETTLE_SECONDS,
                ),
                SerialWriteStage(
                    PX4_NSH_INIT,
                    delay_seconds=PX4_BOOTSTRAP_NSH_INIT_SETTLE_SECONDS,
                ),
                SerialWriteStage(
                    PX4_NSH_REBOOT_BOOTLOADER,
                    delay_seconds=PX4_BOOTSTRAP_NSH_REBOOT_SETTLE_SECONDS,
                ),
            )
        )
    return tuple(stages)


def windows_serial_sequence_bytes(
    port: str,
    stages: tuple[SerialWriteStage, ...],
    read_seconds: float,
) -> SerialSequenceResult:
    """Run all staged writes through one PowerShell SerialPort instance."""
    if re.fullmatch(r"COM[0-9]+", port, re.IGNORECASE) is None:
        return SerialSequenceResult(
            False, 0, False, b"", f"invalid Windows COM port: {port}"
        )
    executable = shutil.which("powershell.exe") or shutil.which("pwsh.exe")
    if executable is None:
        return SerialSequenceResult(
            False,
            0,
            False,
            b"",
            "PowerShell is unavailable for Windows serial access",
        )

    plan = [
        {
            "payload": base64.b64encode(stage.payload).decode("ascii"),
            "delay_ms": max(0, int(round(stage.delay_seconds * 1000.0))),
            "reset": stage.reset_buffers,
        }
        for stage in stages
    ]
    encoded_plan = base64.b64encode(
        json.dumps({"stages": plan}, separators=(",", ":")).encode("utf-8")
    ).decode("ascii")
    read_milliseconds = max(0, int(round(read_seconds * 1000.0)))
    script = (
        "$ErrorActionPreference='Stop';"
        f"$serial=[System.IO.Ports.SerialPort]::new('{port.upper()}',115200,"
        "[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One);"
        "$serial.ReadTimeout=50;$serial.WriteTimeout=500;"
        "$serial.DtrEnable=$false;$serial.RtsEnable=$false;"
        "$buffer=New-Object byte[] 4096;"
        "$received=New-Object System.IO.MemoryStream;"
        "$attempted=$false;$completed=0;$disconnected=$false;$failure='';"
        f"$planJson=[Text.Encoding]::UTF8.GetString([Convert]::FromBase64String('{encoded_plan}'));"
        "$plan=(ConvertFrom-Json -InputObject $planJson).stages;"
        "function ReadAvailable{while($serial.IsOpen){"
        "$available=$serial.BytesToRead;if($available -le 0){break};"
        "$count=$serial.Read($buffer,0,[Math]::Min($buffer.Length,$available));"
        "if($count -gt 0){$received.Write($buffer,0,$count)}}};"
        "try{$serial.Open();foreach($stage in $plan){"
        "if([bool]$stage.reset){$serial.DiscardInBuffer();"
        "$serial.DiscardOutBuffer()};"
        "$payload=[Convert]::FromBase64String([string]$stage.payload);"
        "$attempted=$true;"
        "if($payload.Length -gt 0){$serial.Write($payload,0,$payload.Length);"
        "$serial.BaseStream.Flush()};"
        "$completed+=1;"
        "$delay=[int]$stage.delay_ms;"
        "if($delay -gt 0){Start-Sleep -Milliseconds $delay};"
        "ReadAvailable};"
        f"$deadline=[DateTime]::UtcNow.AddMilliseconds({read_milliseconds});"
        "while([DateTime]::UtcNow -lt $deadline){ReadAvailable;"
        "Start-Sleep -Milliseconds 10}"
        "}catch{$failure=$_.Exception.Message;"
        "if($attempted){try{Start-Sleep -Milliseconds 50;"
        "$present=@([System.IO.Ports.SerialPort]::GetPortNames()|"
        "ForEach-Object{$_.ToUpperInvariant()});"
        f"$disconnected=-not ($present -contains '{port.upper()}')"
        "}catch{$disconnected=$false}}}"
        "finally{try{ReadAvailable}catch{};"
        "try{if($serial.IsOpen){$serial.Close()}}catch{};$serial.Dispose()};"
        "$result=[ordered]@{attempted=$attempted;completed=$completed;"
        "disconnected=$disconnected;"
        "output=[Convert]::ToBase64String($received.ToArray());error=$failure};"
        "$received.Dispose();$result|ConvertTo-Json -Compress"
    )
    timeout_seconds = (
        sum(stage.delay_seconds for stage in stages) + read_seconds + 5.0
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
            timeout=max(3.0, timeout_seconds),
            check=False,
        )
    except subprocess.TimeoutExpired:
        return SerialSequenceResult(
            False, 0, False, b"", "Windows serial sequence timed out"
        )

    if completed.returncode != 0:
        details = decode_output(completed.stderr or completed.stdout)
        return SerialSequenceResult(
            False, 0, False, b"", details or "Windows serial sequence failed"
        )
    try:
        result = json.loads(decode_output(completed.stdout))
        output = base64.b64decode(str(result["output"]), validate=True)
        return SerialSequenceResult(
            bool(result["attempted"]),
            int(result["completed"]),
            bool(result["disconnected"]),
            output,
            str(result["error"]),
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        return SerialSequenceResult(
            False,
            0,
            False,
            b"",
            f"Windows serial sequence returned malformed JSON: {error}",
        )


def posix_serial_sequence_bytes(
    port: str,
    stages: tuple[SerialWriteStage, ...],
    read_seconds: float,
) -> SerialSequenceResult:
    """Run all staged writes through one raw POSIX descriptor."""
    if termios is None or tty is None:
        return SerialSequenceResult(
            False, 0, False, b"", "POSIX serial support is unavailable"
        )
    try:
        descriptor = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        return SerialSequenceResult(
            False, 0, False, b"", f"cannot open {port}: {error}"
        )

    previous_attributes = None
    received = bytearray()
    attempted = False
    completed_stages = 0
    disconnected = False
    failure = ""

    def collect_for(duration: float) -> None:
        deadline = time.monotonic() + max(0.0, duration)
        while True:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select([descriptor], [], [], remaining)
            if not readable:
                return
            chunk = os.read(descriptor, 4096)
            if not chunk:
                return
            received.extend(chunk)
            if time.monotonic() >= deadline:
                return

    try:
        previous_attributes = termios.tcgetattr(descriptor)
        tty.setraw(descriptor, when=termios.TCSANOW)
        attributes = termios.tcgetattr(descriptor)
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)

        for stage in stages:
            if stage.reset_buffers:
                termios.tcflush(descriptor, termios.TCIOFLUSH)

            attempted = True
            write_deadline = time.monotonic() + 0.5
            offset = 0
            while offset < len(stage.payload) and time.monotonic() < write_deadline:
                remaining = max(0.0, write_deadline - time.monotonic())
                _, writable, _ = select.select([], [descriptor], [], remaining)
                if not writable:
                    break
                try:
                    offset += os.write(descriptor, stage.payload[offset:])
                except BlockingIOError:
                    continue
            if offset != len(stage.payload):
                failure = f"serial write to {port} did not complete"
                break
            completed_stages += 1
            collect_for(stage.delay_seconds)

        if not failure:
            collect_for(read_seconds)
    except (OSError, ValueError, termios.error) as error:
        failure = f"serial sequence on {port} ended: {error}"
        disconnected = attempted and getattr(error, "errno", None) in {
            errno.ENODEV,
            errno.ENXIO,
            errno.EIO,
        }
    finally:
        if previous_attributes is not None:
            try:
                termios.tcsetattr(descriptor, termios.TCSANOW, previous_attributes)
            except (OSError, termios.error):
                pass
        try:
            os.close(descriptor)
        except OSError:
            pass

    return SerialSequenceResult(
        attempted,
        completed_stages,
        disconnected,
        bytes(received),
        failure,
    )


def serial_sequence_bytes(
    serial_backend: SerialBackend,
    port: str,
    stages: tuple[SerialWriteStage, ...],
    read_seconds: float,
) -> SerialSequenceResult:
    if serial_backend == SerialBackend.WINDOWS_COM:
        return windows_serial_sequence_bytes(port, stages, read_seconds)
    return posix_serial_sequence_bytes(port, stages, read_seconds)


def try_application_identify(
    codec: MavlinkCodec,
    serial_backend: SerialBackend,
    port: str,
    deadline: float | None = None,
) -> tuple[bool, ApplicationIdentity | None, str]:
    # Keep one upgrade bridge for boards still running the pre-MAVLink console.
    remaining = None if deadline is None else deadline - time.monotonic()
    if remaining is not None and remaining <= 0:
        return False, None, "application probe deadline expired"
    legacy_read_seconds = min(0.45, remaining) if remaining is not None else 0.45
    sent, output = serial_exchange(
        serial_backend,
        port,
        LEGACY_APP_IDENTIFY_REQUEST,
        read_seconds=legacy_read_seconds,
    )
    if sent and LEGACY_APP_IDENTIFY_TOKEN in output:
        return sent, ApplicationIdentity(protocol="legacy"), output

    remaining = None if deadline is None else deadline - time.monotonic()
    if remaining is not None and remaining <= 0:
        return sent, None, output or "application probe deadline expired"
    mavlink_read_seconds = min(0.8, remaining) if remaining is not None else 0.8
    mavlink_sent, binary_output, mavlink_error = serial_exchange_bytes(
        serial_backend,
        port,
        codec.identify_request(),
        read_seconds=mavlink_read_seconds,
    )
    if mavlink_sent:
        identity = codec.application_identity(binary_output)
        if identity is not None:
            return True, identity, identity.summary()
        mavlink_error = "no matching Dima Rover MAVLink identity was received"
    details = output or mavlink_error
    return sent or mavlink_sent, None, details


def request_application_recovery(
    codec: MavlinkCodec,
    serial_backend: SerialBackend,
    port: str,
    identity: ApplicationIdentity,
) -> tuple[bool, str]:
    if identity.protocol == "legacy":
        sent, output = serial_exchange(
            serial_backend, port, LEGACY_APP_REBOOT_REQUEST, read_seconds=0.45
        )
        if not sent:
            raise UploadError(
                f"REBOOT_REQUEST: unable to send reboot -b to {port}: {output}"
            )
        if LEGACY_APP_REBOOT_DENIED in output:
            raise UploadError(
                "REBOOT_REQUEST: the application refused bootloader reboot because it is armed"
            )
        if LEGACY_APP_REBOOT_ACK in output:
            return True, "legacy reboot ACK accepted"
        return False, output or "legacy reboot -b sent without an ACK"

    sequence = serial_sequence_bytes(
        serial_backend,
        port,
        px4_mavlink_reboot_stages(),
        read_seconds=0.25,
    )
    if (
        not sequence.attempted
        or (
            sequence.completed_stages < 2
            and not sequence.disconnected
        )
    ):
        raise UploadError(
            f"REBOOT_REQUEST: unable to send PX4 MAVLink reboot sequence to "
            f"{port}: {sequence.error or 'no targeted reboot frame completed'}"
        )
    accepted, details = codec.reboot_ack(sequence.output)
    if accepted is False:
        raise UploadError(f"REBOOT_REQUEST: {details}")
    sequence_details = (
        f"PX4 MAVLink reboot writes={sequence.completed_stages}/"
        f"{PX4_REBOOT_ATTEMPTS * 2}"
    )
    if sequence.disconnected:
        sequence_details += "; application USB port disappeared during reboot"
    elif sequence.error:
        sequence_details += f"; serial sequence ended with {sequence.error}"
    if accepted is True:
        return True, f"{details}; {sequence_details}"
    return False, f"{details}; {sequence_details}"


def request_px4_bootstrap_recovery(
    serial_backend: SerialBackend,
    port: str,
) -> str:
    sequence = serial_sequence_bytes(
        serial_backend,
        port,
        px4_bootstrap_reboot_stages(),
        read_seconds=0.0,
    )
    if (
        not sequence.attempted
        or (
            sequence.completed_stages < 2
            and not sequence.disconnected
        )
    ):
        raise UploadError(
            f"REBOOT_REQUEST: unable to send the PX4 MAVLink/NSH reboot "
            f"sequence to {port}: "
            f"{sequence.error or 'no targeted reboot frame completed'}"
        )
    details = (
        f"PX4 explicit-port reboot writes={sequence.completed_stages}/"
        f"{PX4_REBOOT_ATTEMPTS * 4}"
    )
    if sequence.disconnected:
        details += "; application USB port disappeared during reboot"
    elif sequence.error:
        details += f"; serial sequence ended with {sequence.error}"
    return details


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
        state.slot == 0
        and state.digest == digest
        and "active" in state.flags
        and "confirmed" in state.flags
        for state in parse_image_states(output)
    )


def has_unconfirmed_active_image(output: str) -> bool:
    return any(
        state.slot == 0
        and "active" in state.flags
        and "confirmed" not in state.flags
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
        raise UploadError("mcumgr response returned no Images section")
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

    tools_cache = arguments.tools_cache.expanduser()
    runtime = resolve_mcumgr(arguments.mcumgr, arguments.port, tools_cache)
    codec = resolve_mavlink_codec(tools_cache)
    if arguments.preflight_only:
        endpoint_preflight(
            runtime,
            codec,
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
    port, initial_list, initial_identity, initial_binding = wait_for_recovery(
        runtime,
        codec,
        arguments.port,
        arguments.wait_seconds,
        arguments.baud,
        arguments.mtu,
    )
    already_active_confirmed = has_active_confirmed_image(initial_list, digest)
    if has_unconfirmed_active_image(initial_list):
        raise UploadError(
            "ACTIVE_IMAGE_UNCONFIRMED: refusing to overwrite Secondary while "
            "the current test image still depends on it for rollback"
        )
    if already_active_confirmed and not arguments.force:
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
        wait_for_application(
            runtime,
            codec,
            port,
            arguments.wait_seconds,
            initial_identity,
            initial_binding,
        )
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
    # An explicitly forced rewrite of the active hash is ambiguous to MCUboot's
    # hash lookup because Primary matches first.  Only that uncommon case needs
    # a separate Secondary proof before setting pending.
    if already_active_confirmed:
        stage("VERIFY_SECONDARY", "verifying the forced Secondary rewrite")
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
                "SECONDARY_HASH_MISMATCH: the forced image was not found in slot 1"
            )
    stage("TEST", f"marking {digest} as the test image")
    pending_list = run_mcumgr(
        runtime.executable,
        port,
        "image",
        "test",
        digest,
        expect_images=True,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    stage("VERIFY_SECONDARY", "verifying hash and pending state from TEST response")
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
    application_port, application_identity, application_binding = wait_for_application(
        runtime,
        codec,
        port,
        arguments.wait_seconds,
        initial_identity,
        initial_binding,
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
    reboot_acknowledged, reboot_output = request_application_recovery(
        codec,
        runtime.serial_backend,
        application_port,
        application_identity,
    )
    if reboot_output:
        print(reboot_output)
    if not reboot_acknowledged:
        stage(
            "REBOOT_REQUEST",
            "automatic reboot sequence sent; ACK was not required; waiting "
            "for MCUboot on the same physical USB device",
        )
    confirm_port, confirm_list = wait_for_recovery_endpoint(
        runtime,
        application_binding,
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
    wait_for_application(
        runtime,
        codec,
        confirm_port,
        arguments.wait_seconds,
        application_identity,
        application_binding,
    )
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
