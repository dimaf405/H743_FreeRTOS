#!/usr/bin/env python3
"""Upload, test, and boot the current signed image through MCUboot USB CDC."""

from __future__ import annotations

import argparse
import base64
import glob
import json
import os
import pathlib
import re
import select
import shlex
import shutil
import subprocess
import sys
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
DEFAULT_MAX_WINDOW = 2
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


def resolve_mcumgr(
    requested: str, explicit_port: str | None, tools_cache: pathlib.Path
) -> tuple[str, bool]:
    direct = pathlib.Path(requested).expanduser()
    if direct.is_file():
        resolved = str(direct.resolve())
        return resolved, is_wsl() and resolved.lower().endswith(".exe")

    use_windows_port = explicit_port is None or explicit_port.upper().startswith("COM")
    default_mcumgr = requested in {"mcumgr", "mcumgr.exe"}
    if is_wsl() and use_windows_port and default_mcumgr:
        # Always use the pinned Dima build here.  Besides keeping the executable
        # and COM port on Windows, it removes Apache newtmgr's UART-only 20 ms
        # delay from the USB CDC fast path.  An arbitrary PATH executable can
        # silently restore the approximately 2 KiB/s behavior.
        try:
            bootstrapped = ensure_mcumgr(tools_cache, target_windows=True)
        except BootstrapError as error:
            raise UploadError(str(error)) from error
        return str(bootstrapped.resolve()), True

    resolved = shutil.which(requested)
    if resolved is not None:
        return resolved, is_wsl() and resolved.lower().endswith(".exe")

    if default_mcumgr:
        try:
            bootstrapped = ensure_mcumgr(tools_cache, target_windows=False)
        except BootstrapError as error:
            raise UploadError(str(error)) from error
        resolved_bootstrap = str(bootstrapped.resolve())
        return resolved_bootstrap, False

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
    windows_backend: bool, port: str, request: bytes, read_seconds: float
) -> tuple[bool, str]:
    if windows_backend:
        return windows_serial_exchange(port, request, read_seconds)
    return posix_serial_exchange(port, request, read_seconds)


def try_application_identify(
    windows_backend: bool, port: str
) -> tuple[bool, bool, str]:
    sent, output = serial_exchange(
        windows_backend, port, APP_IDENTIFY_REQUEST, read_seconds=0.45
    )
    return sent, sent and APP_IDENTIFY_TOKEN in output, output


def request_application_recovery(windows_backend: bool, port: str) -> str:
    sent, output = serial_exchange(
        windows_backend, port, APP_REBOOT_REQUEST, read_seconds=0.45
    )
    if not sent:
        raise UploadError(f"unable to send reboot -b to {port}: {output}")
    if APP_REBOOT_DENIED in output:
        raise UploadError("the application refused bootloader reboot because it is armed")
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
    output = decode_output(completed.stdout)
    success = (
        completed.returncode == 0
        and protocol_error(output) is None
        and MCUMGR_IMAGES_RE.search(output) is not None
    )
    return success, output


def wait_for_recovery(
    executable: str,
    windows_backend: bool,
    explicit_port: str | None,
    wait_seconds: int,
    baud: int,
    mtu: int,
) -> str:
    print(
        "Scanning serial ports for a Dima Rover application or MCUboot Recovery "
        f"for up to {wait_seconds}s...",
        flush=True,
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
        elif windows_backend:
            ports = windows_serial_ports()
        else:
            ports = posix_serial_ports()

        recovery_matches: list[tuple[str, str]] = []
        application_matches: list[tuple[str, str]] = []
        now = time.monotonic()
        for port in ports:
            if now < probe_after.get(port, 0.0):
                continue
            probe_after[port] = now + 1.0
            success, output = try_image_list(executable, port, baud, mtu)
            if success:
                recovery_matches.append((port, output))
                continue
            if output:
                last_error = f"{port}: {output}"

            if reboot_requested:
                continue

            sent, identified, identify_output = try_application_identify(
                windows_backend, port
            )
            if not identified:
                if sent and identify_output:
                    incompatible_response_seen = True
                    last_error = f"{port}: {identify_output[-240:]}"
                elif identify_output:
                    last_error = f"{port}: {identify_output[-240:]}"
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
            print(f"Connected to MCUboot recovery on {port}")
            if output:
                print(output)
            return port

        if application_matches:
            port, _ = application_matches[0]

            print(f"Identified Dima Rover application on {port}", flush=True)
            reboot_output = request_application_recovery(windows_backend, port)
            if reboot_output:
                print(reboot_output)
            if APP_REBOOT_ACK not in reboot_output:
                print(
                    "reboot -b was sent; waiting for USB re-enumeration...",
                    flush=True,
                )
            else:
                print("Application accepted reboot -b; waiting for MCUboot...", flush=True)
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
        f"MCUboot recovery was not reached within {wait_seconds}s ({last_error})"
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
    deadline = started + timeout_seconds
    timed_out = False
    last_byte = b"\n"
    while process.poll() is None:
        if time.monotonic() >= deadline:
            timed_out = True
            process.kill()
            break
        ready, _, _ = select.select([process.stdout], [], [], 0.1)
        if not ready:
            continue
        chunk = os.read(process.stdout.fileno(), 4096)
        if not chunk:
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
    remainder = process.stdout.read()
    if remainder:
        chunks.append(remainder)
        last_byte = remainder[-1:]
        output_buffer = getattr(sys.stdout, "buffer", None)
        if output_buffer is not None:
            output_buffer.write(remainder)
            output_buffer.flush()
        else:
            sys.stdout.write(remainder.decode("utf-8", errors="replace"))
            sys.stdout.flush()
    if chunks and last_byte != b"\n":
        print()

    if timed_out:
        raise UploadError("mcumgr command timed out: " + " ".join(arguments))

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
    parser.add_argument("--image", required=True, type=pathlib.Path)
    parser.add_argument("--imgtool", required=True, type=pathlib.Path)
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
    arguments = parser.parse_args()

    if arguments.wait_seconds <= 0:
        parser.error("--wait-seconds must be positive")
    if arguments.baud <= 0:
        parser.error("--baud must be positive")
    if arguments.mtu < 128 or arguments.mtu > 512:
        parser.error("--mtu must be between 128 and 512 for this MCUboot build")
    if arguments.max_window < 1 or arguments.max_window > 3:
        parser.error("--max-window must be between 1 and 3 for the 2048-byte RX ring")

    image = arguments.image.resolve()
    digest = image_hash(arguments.imgtool.resolve(), image)
    executable, windows_backend = resolve_mcumgr(
        arguments.mcumgr, arguments.port, arguments.tools_cache.expanduser()
    )
    port = wait_for_recovery(
        executable,
        windows_backend,
        arguments.port,
        arguments.wait_seconds,
        arguments.baud,
        arguments.mtu,
    )
    upload_image = windows_path(image) if windows_backend else str(image)

    # The currently deployed bootloader receives serial frames in a 2048-byte
    # ring.  Two encoded 512-byte-MTU requests fit with ample margin; larger
    # windows remain opt-in until they pass a real-board acceptance run.
    run_mcumgr(
        executable,
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
    run_mcumgr(
        executable,
        port,
        "image",
        "list",
        expect_images=True,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    run_mcumgr(
        executable,
        port,
        "image",
        "test",
        digest,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    run_mcumgr(
        executable,
        port,
        "image",
        "list",
        expect_images=True,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    run_mcumgr(
        executable,
        port,
        "reset",
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
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
