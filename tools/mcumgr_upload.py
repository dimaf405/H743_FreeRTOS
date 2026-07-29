#!/usr/bin/env python3
"""Upload, test, and boot the current signed image through MCUboot USB CDC."""

from __future__ import annotations

import argparse
import glob
import json
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
import time


USB_VID = "0483"
USB_PID = "5740"
USB_PRODUCT = "H743 MCUboot Recovery"
IMAGE_SHA256_TLV = 0x10
MCUMGR_ERROR_RE = re.compile(
    r"^Error:[ \t]*([+-]?[0-9]+)[ \t]*\r?$", re.MULTILINE
)
MCUMGR_IMAGES_RE = re.compile(r"^Images:[ \t]*\r?$", re.MULTILINE)


class UploadError(RuntimeError):
    """An actionable one-command upload failure."""


def is_wsl() -> bool:
    try:
        return "microsoft" in pathlib.Path("/proc/sys/kernel/osrelease").read_text(
            encoding="utf-8"
        ).lower()
    except OSError:
        return False


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


def windows_mcumgr_from_path() -> str | None:
    windows_path = powershell_output(
        "$paths=@();"
        "$command=Get-Command mcumgr.exe -ErrorAction SilentlyContinue;"
        "if($command){$paths+=$command.Source};"
        "$paths+=(Join-Path $env:USERPROFILE 'go\\bin\\mcumgr.exe');"
        "$paths | Where-Object {Test-Path $_} | Select-Object -First 1"
    )
    if not windows_path:
        return None
    completed = subprocess.run(
        ["wslpath", "-u", windows_path.splitlines()[0].strip()],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    linux_path = decode_output(completed.stdout)
    return linux_path if completed.returncode == 0 and linux_path else None


def resolve_mcumgr(requested: str, explicit_port: str | None) -> tuple[str, bool]:
    direct = pathlib.Path(requested).expanduser()
    if direct.is_file():
        resolved = str(direct.resolve())
        return resolved, is_wsl() and resolved.lower().endswith(".exe")

    use_windows_port = explicit_port is None or explicit_port.upper().startswith("COM")
    if is_wsl() and use_windows_port and requested in {"mcumgr", "mcumgr.exe"}:
        windows_mcumgr = windows_mcumgr_from_path()
        if windows_mcumgr is not None:
            return windows_mcumgr, True

    resolved = shutil.which(requested)
    if resolved is not None:
        return resolved, is_wsl() and resolved.lower().endswith(".exe")

    raise UploadError(
        "mcumgr was not found; install Apache mcumgr or set "
        "MCUMGR=/absolute/path/to/mcumgr"
    )


def windows_recovery_ports() -> list[str]:
    output = powershell_output(
        "Get-CimInstance Win32_PnPEntity | "
        f"Where-Object {{$_.ConfigManagerErrorCode -eq 0 "
        f"-and $_.PNPDeviceID -match 'VID_{USB_VID}&PID_{USB_PID}' "
        "-and $_.Name -match '\\(COM[0-9]+\\)'}} | "
        "ForEach-Object {if($_.Name -match '\\((COM[0-9]+)\\)'){$Matches[1]}}"
    )
    return sorted(set(re.findall(r"\bCOM[0-9]+\b", output)), key=str.casefold)


def is_posix_recovery_port(port: pathlib.Path) -> bool:
    """Validate the USB identity behind a tty instead of probing every ACM port."""
    try:
        tty_name = port.resolve(strict=True).name
        device = (pathlib.Path("/sys/class/tty") / tty_name / "device").resolve(
            strict=True
        )
    except OSError:
        return False

    for parent in (device, *device.parents):
        vendor_path = parent / "idVendor"
        product_id_path = parent / "idProduct"
        product_path = parent / "product"
        if not vendor_path.is_file() or not product_id_path.is_file():
            continue
        try:
            vendor = vendor_path.read_text(encoding="ascii").strip()
            product_id = product_id_path.read_text(encoding="ascii").strip()
            product = product_path.read_text(encoding="utf-8").strip()
        except OSError:
            return False
        return (
            vendor.casefold() == USB_VID.casefold()
            and product_id.casefold() == USB_PID.casefold()
            and product.casefold() == USB_PRODUCT.casefold()
        )
    return False


def posix_recovery_ports() -> list[str]:
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    candidates = by_id + sorted(glob.glob("/dev/ttyACM*"))
    physical_ports: dict[str, str] = {}
    for candidate in candidates:
        path = pathlib.Path(candidate)
        if not is_posix_recovery_port(path):
            continue
        try:
            physical_port = str(path.resolve(strict=True))
        except OSError:
            continue
        # by-id aliases are listed first and remain the stable displayed path.
        physical_ports.setdefault(physical_port, candidate)
    return sorted(physical_ports.values(), key=str.casefold)


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


def windows_path(path: pathlib.Path) -> str:
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


def mcumgr_command(executable: str, port: str, *arguments: str) -> list[str]:
    return [
        executable,
        "--conntype",
        "serial",
        "--connstring",
        f"dev={port},baud=115200",
        *arguments,
    ]


def protocol_error(output: str) -> str | None:
    for match in MCUMGR_ERROR_RE.finditer(output):
        if int(match.group(1)) != 0:
            return match.group(0)
    return None


def try_image_list(executable: str, port: str) -> tuple[bool, str]:
    command = mcumgr_command(
        executable, port, "--timeout", "0.5", "--tries", "1", "image", "list"
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
    output = decode_output(completed.stdout)
    success = (
        completed.returncode == 0
        and protocol_error(output) is None
        and MCUMGR_IMAGES_RE.search(output) is not None
    )
    return success, output


def wait_for_recovery(
    executable: str, windows_backend: bool, explicit_port: str | None, wait_seconds: int
) -> str:
    print(
        f"Waiting up to {wait_seconds}s for H743 MCUboot Recovery; "
        "keep BOOT0 low and press RESET now...",
        flush=True,
    )
    deadline = time.monotonic() + wait_seconds
    last_error = "no matching USB CDC port detected"
    while time.monotonic() < deadline:
        if explicit_port:
            ports = [explicit_port]
        elif windows_backend:
            ports = windows_recovery_ports()
        else:
            ports = posix_recovery_ports()

        if len(ports) > 1:
            raise UploadError(
                "multiple matching USB devices were detected: "
                + ", ".join(ports)
                + "; set MCUMGR_PORT to the intended port"
            )

        for port in ports:
            success, output = try_image_list(executable, port)
            if success:
                print(f"Connected to MCUboot recovery on {port}")
                if output:
                    print(output)
                return port
            if output:
                last_error = f"{port}: {output}"
        time.sleep(0.25)

    raise UploadError(
        f"MCUboot recovery was not reached within {wait_seconds}s ({last_error})"
    )


def run_mcumgr(
    executable: str, port: str, *arguments: str, expect_images: bool = False
) -> str:
    command = mcumgr_command(executable, port, *arguments)
    print("+ " + shlex.join(command), flush=True)
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=300,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        partial_output = decode_output(error.output or b"")
        if partial_output:
            print(partial_output)
        raise UploadError("mcumgr command timed out: " + " ".join(arguments)) from error
    output = decode_output(completed.stdout)
    if output:
        print(output)
    if completed.returncode != 0:
        raise UploadError(
            f"mcumgr command failed with exit status {completed.returncode}: "
            + " ".join(arguments)
        )
    device_error = protocol_error(output)
    if device_error is not None:
        raise UploadError(
            f"MCUboot rejected {' '.join(arguments)} ({device_error})"
        )
    if expect_images and MCUMGR_IMAGES_RE.search(output) is None:
        raise UploadError("mcumgr image list returned no Images section")
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, type=pathlib.Path)
    parser.add_argument("--imgtool", required=True, type=pathlib.Path)
    parser.add_argument("--mcumgr", default="mcumgr")
    parser.add_argument("--port")
    parser.add_argument("--wait-seconds", type=int, default=60)
    arguments = parser.parse_args()

    if arguments.wait_seconds <= 0:
        parser.error("--wait-seconds must be positive")

    image = arguments.image.resolve()
    digest = image_hash(arguments.imgtool.resolve(), image)
    executable, windows_backend = resolve_mcumgr(arguments.mcumgr, arguments.port)
    port = wait_for_recovery(
        executable, windows_backend, arguments.port, arguments.wait_seconds
    )
    upload_image = windows_path(image) if windows_backend else str(image)

    run_mcumgr(executable, port, "image", "upload", "-n", "2", upload_image)
    run_mcumgr(executable, port, "image", "list", expect_images=True)
    run_mcumgr(executable, port, "image", "test", digest)
    run_mcumgr(executable, port, "image", "list", expect_images=True)
    run_mcumgr(executable, port, "reset")
    print("Upload complete; keep power stable while MCUboot swaps the image.")
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
