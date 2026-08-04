#!/usr/bin/env python3
"""Read and decode persistent boot diagnostics through STM32 ROM USB DFU."""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib


FLASH_BASE = 0x08020000
FLASH_SIZE = 0x00020000
RECORD_SIZE = 256
RECORD_MAGIC = 0x44424652
RECORD_VERSION = 1
RECORD_COMMIT = 0x434D4954
DIAGNOSTICS_SIZE = 192
DIAGNOSTICS_MAGIC = 0x44424447
DIAGNOSTICS_VERSION = 1
DIAGNOSTICS_CAPTURE_VALID = 0x43505452

DIAGNOSTIC_FIELDS = (
    "magic", "version", "size", "boot_count", "reset_flags", "stage",
    "detail", "failure_kind", "capture_valid", "previous_stage",
    "previous_failure_kind", "previous_pc", "previous_cfsr",
    "exception_return", "stacked_r0", "stacked_r1", "stacked_r2",
    "stacked_r3", "stacked_r12", "stacked_lr", "stacked_pc",
    "stacked_xpsr", "msp", "psp", "primask", "basepri", "faultmask",
    "control", "cfsr", "hfsr", "dfsr", "afsr", "mmfar", "bfar",
    "icsr", "shcsr", "system_core_clock", "system_d2_clock",
    "rcc_cfgr", "rcc_d1cfgr", "rcc_d2cfgr", "systick_ctrl",
    "systick_load", "systick_value", "tim2_psc", "tim2_arr",
    "tim2_cnt", "tim2_sr",
)

STAGES = {
    0x0100: "SYSTEM_INIT", 0x0200: "MAIN_ENTER", 0x0210: "HAL_INIT",
    0x0220: "SYSTEM_CLOCK", 0x0230: "PERIPHERAL_CLOCK",
    0x0301: "GPIO", 0x0302: "DMA", 0x0303: "FDCAN1",
    0x0304: "I2C2", 0x0305: "SPI4", 0x0306: "UART4",
    0x0307: "UART5", 0x0308: "UART7", 0x0309: "UART8",
    0x030A: "USART1", 0x030B: "USART2", 0x030C: "USART3",
    0x030D: "USART6", 0x030E: "TIM5", 0x030F: "TIM8",
    0x03FF: "BOARD_READY", 0x0400: "PLATFORM_EARLY",
    0x0410: "HEAP_INIT", 0x0420: "HRT_INIT", 0x04FF: "PLATFORM_READY",
    0x0500: "KERNEL_INIT", 0x0510: "APP_TASK_CREATE",
    0x0520: "SCHEDULER_START", 0x0600: "APP_TASK_ENTER",
    0x0610: "USB_INIT", 0x061F: "USB_READY",
    0x0620: "WORK_QUEUE_INIT", 0x0630: "UORB_INIT",
    0x0640: "PARAMETER_INIT", 0x0650: "MODULE_REGISTER",
    0x06FF: "APPLICATION_INITIALIZED", 0x0700: "BOOT_HEALTH_START",
    0x0710: "HELLO_START", 0x0720: "PARAMETER_START",
    0x0730: "LOG_START", 0x0740: "COMMANDER_START",
    0x0750: "RC_START", 0x07FF: "APPLICATION_RUNNING",
    0x0F00: "APPLICATION_FAILED",
}

FAILURES = {
    0: "NONE", 1: "NMI", 2: "HARDFAULT", 3: "MEMMANAGE",
    4: "BUSFAULT", 5: "USAGEFAULT", 6: "ERROR_HANDLER",
    7: "FREERTOS_ASSERT", 8: "STACK_OVERFLOW",
}


def find_cubeprogrammer(explicit: str | None) -> pathlib.Path:
    candidates: list[pathlib.Path] = []
    if explicit:
        candidates.append(pathlib.Path(explicit))
    configured = os.environ.get("CUBEPROGRAMMER_CLI")
    if configured:
        candidates.append(pathlib.Path(configured))
    discovered = shutil.which("STM32_Programmer_CLI.exe") or shutil.which(
        "STM32_Programmer_CLI"
    )
    if discovered:
        candidates.append(pathlib.Path(discovered))

    suffix = pathlib.Path(
        "Program Files/STMicroelectronics/STM32Cube/"
        "STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"
    )
    if os.name == "nt":
        for drive in "CDEFG":
            candidates.append(pathlib.Path(f"{drive}:/") / suffix)
    else:
        for drive in "cdefg":
            candidates.append(pathlib.Path(f"/mnt/{drive}") / suffix)

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise RuntimeError(
        "STM32_Programmer_CLI was not found; set CUBEPROGRAMMER_CLI"
    )


def run_cli(cli: pathlib.Path, arguments: list[str]) -> str:
    completed = subprocess.run(
        [str(cli), *arguments], check=False, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    output = completed.stdout.decode(errors="replace")
    if completed.returncode != 0 or "Error:" in output:
        raise RuntimeError(output.strip())
    return output


def detect_port(cli: pathlib.Path) -> str:
    output = run_cli(cli, ["-l", "usb"])
    ports = re.findall(r"Device Index\s*:\s*(USB\d+)", output)
    if len(ports) != 1:
        raise RuntimeError(
            f"expected exactly one STM32 DFU device, found {len(ports)}"
        )
    return ports[0]


def cli_path(path: pathlib.Path) -> str:
    if os.name == "nt":
        return str(path)
    return subprocess.check_output(
        ["wslpath", "-w", str(path)], text=True
    ).strip()


def upload_flash(cli: pathlib.Path, port: str) -> bytes:
    with tempfile.TemporaryDirectory(prefix="dima-boot-diag-") as directory:
        dump = pathlib.Path(directory) / "boot-diagnostics.bin"
        run_cli(
            cli,
            ["-c", f"port={port}", "-u", hex(FLASH_BASE),
             str(FLASH_SIZE), cli_path(dump)],
        )
        data = dump.read_bytes()
    if len(data) != FLASH_SIZE:
        raise RuntimeError(
            f"DFU returned {len(data)} bytes, expected {FLASH_SIZE}"
        )
    return data


def valid_records(data: bytes) -> list[tuple[int, dict[str, int]]]:
    if len(data) != FLASH_SIZE:
        raise RuntimeError(
            f"diagnostic dump is {len(data)} bytes, expected {FLASH_SIZE}"
        )
    records: list[tuple[int, dict[str, int]]] = []
    for offset in range(0, len(data), RECORD_SIZE):
        record = data[offset:offset + RECORD_SIZE]
        magic, version, size, sequence = struct.unpack_from("<IIII", record)
        commit = struct.unpack_from("<I", record, RECORD_SIZE - 4)[0]
        stored_crc = struct.unpack_from(
            "<I", record, 16 + DIAGNOSTICS_SIZE
        )[0]
        actual_crc = zlib.crc32(
            record[:16 + DIAGNOSTICS_SIZE]
        ) & 0xFFFFFFFF
        if (
            magic != RECORD_MAGIC or version != RECORD_VERSION
            or size != RECORD_SIZE or commit != RECORD_COMMIT
            or stored_crc != actual_crc
        ):
            continue
        values = struct.unpack_from(
            "<" + "I" * len(DIAGNOSTIC_FIELDS), record, 16
        )
        diagnostics = dict(zip(DIAGNOSTIC_FIELDS, values))
        if (
            diagnostics["magic"] != DIAGNOSTICS_MAGIC
            or diagnostics["version"] != DIAGNOSTICS_VERSION
            or diagnostics["size"] != DIAGNOSTICS_SIZE
            or diagnostics["capture_valid"] != DIAGNOSTICS_CAPTURE_VALID
            or diagnostics["failure_kind"] == 0
        ):
            continue
        records.append((sequence, diagnostics))
    return records


def hex_value(value: int) -> str:
    return f"0x{value:08X}"


def print_record(sequence: int, record: dict[str, int]) -> None:
    stage = record["stage"]
    failure = record["failure_kind"]
    print(f"record_sequence: {sequence}")
    print(f"boot_count: {record['boot_count']}")
    print(f"stage: {hex_value(stage)} ({STAGES.get(stage, 'UNKNOWN')})")
    print(f"detail: {hex_value(record['detail'])} ({record['detail']})")
    print(f"failure: {failure} ({FAILURES.get(failure, 'UNKNOWN')})")
    for name in (
        "reset_flags", "stacked_pc", "stacked_lr", "stacked_xpsr",
        "cfsr", "hfsr", "mmfar", "bfar", "msp", "psp",
        "system_core_clock", "system_d2_clock", "rcc_cfgr",
        "rcc_d1cfgr", "rcc_d2cfgr", "systick_ctrl", "systick_load",
        "systick_value", "tim2_psc", "tim2_arr", "tim2_cnt", "tim2_sr",
    ):
        print(f"{name}: {hex_value(record[name])}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cubeprogrammer")
    parser.add_argument("--port")
    parser.add_argument("--dump", type=pathlib.Path)
    args = parser.parse_args()

    if args.dump:
        data = args.dump.read_bytes()
    else:
        cli = find_cubeprogrammer(args.cubeprogrammer)
        port = args.port or detect_port(cli)
        print(f"DFU port: {port}")
        data = upload_flash(cli, port)

    records = valid_records(data)
    if not records:
        print("No valid persistent boot diagnostics record found.")
        return 2
    sequence, record = max(records, key=lambda item: item[0])
    print_record(sequence, record)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
