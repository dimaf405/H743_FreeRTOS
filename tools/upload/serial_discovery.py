"""Windows, WSL, and POSIX serial-port discovery and identity binding."""

from __future__ import annotations

import glob
import json
import os
import pathlib
import re
import shutil
import subprocess

try:
    import winreg
except ImportError:  # pragma: no cover - only native Windows provides winreg.
    winreg = None

from .models import McumgrRuntime, SerialBackend, decode_output

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

def serial_ports(runtime: McumgrRuntime) -> list[str]:
    if runtime.serial_backend == SerialBackend.WINDOWS_COM:
        return windows_serial_ports()
    return posix_serial_ports()

