"""Binary-safe Windows COM serial exchange primitives."""

from __future__ import annotations

import base64
import binascii
import json
import re
import shutil
import subprocess

from .models import SerialSequenceResult, SerialWriteStage, decode_output


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
        "$planJson=[Text.Encoding]::UTF8.GetString("
        f"[Convert]::FromBase64String('{encoded_plan}'));"
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
