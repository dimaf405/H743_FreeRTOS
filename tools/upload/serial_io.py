"""Binary-safe Windows and POSIX serial exchange primitives."""

from __future__ import annotations

import base64
import binascii
import errno
import json
import os
import re
import select
import shutil
import subprocess
import time

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - only POSIX hosts use these modules.
    termios = None
    tty = None

from .models import (
    SerialBackend,
    SerialSequenceResult,
    SerialWriteStage,
    decode_output,
)

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

