#!/usr/bin/env python3
"""Structural verification for the H743 MCUboot build artifacts."""

import argparse
import pathlib
import struct
import subprocess

from intelhex import IntelHex


BOOT_BASE = 0x08000000
BOOT_SIZE = 0x00020000
BOOT_DIAGNOSTICS_BASE = 0x08020000
BOOT_DIAGNOSTICS_SIZE = 0x00020000
PRIMARY_BASE = 0x08040000
SLOT_SIZE = 0x000C0000
HEADER_SIZE = 0x400
APP_VECTOR = PRIMARY_BASE + HEADER_SIZE
IMAGE_MAGIC = 0x96F3B83D
# scratch swap: 6 sectors * 3 status writes * 32 bytes, four aligned
# trailer fields, and one 32-byte-aligned magic field.
SLOT_TRAILER_SIZE = (6 * 3 * 32) + (4 * 32) + 32


def symbol_address(nm: str, elf: pathlib.Path, symbol: str) -> int:
    output = subprocess.check_output([nm, "-n", str(elf)], text=True)
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 3 and fields[-1] == symbol:
            return int(fields[0], 16)
    raise RuntimeError(f"{symbol} not found in {elf}")


def verify_no_undefined_symbols(nm: str, elf: pathlib.Path) -> None:
    output = subprocess.check_output([nm, "-u", str(elf)], text=True)
    unresolved = [line.strip() for line in output.splitlines() if line.strip()]
    if unresolved:
        raise RuntimeError(
            f"{elf} has unresolved symbols: {', '.join(unresolved)}"
        )


def verify_required_symbols(
        nm: str, elf: pathlib.Path, required: set[str]) -> None:
    output = subprocess.check_output(
        [nm, "-g", "--defined-only", str(elf)], text=True,
    )
    names = {
        fields[-1]
        for line in output.splitlines()
        if len(fields := line.split()) >= 3
    }
    missing = sorted(required - names)
    if missing:
        raise RuntimeError(f"{elf} is missing required symbols: {missing}")


def verify_signed_image(filename: pathlib.Path) -> bytes:
    data = filename.read_bytes()
    if len(data) > SLOT_SIZE - SLOT_TRAILER_SIZE:
        raise RuntimeError("signed update image overlaps the MCUboot swap trailer")
    if len(data) < HEADER_SIZE + 8:
        raise RuntimeError("signed update image is truncated")

    fields = struct.unpack_from("<IIHHIIBBHII", data, 0)
    magic, _, header_size, protected_tlv_size, image_size = fields[:5]
    if magic != IMAGE_MAGIC:
        raise RuntimeError(f"bad MCUboot magic: 0x{magic:08x}")
    if header_size != HEADER_SIZE:
        raise RuntimeError(f"unexpected header size: 0x{header_size:x}")

    stack_pointer, reset_handler = struct.unpack_from("<II", data, HEADER_SIZE)
    if not (0x20000000 <= stack_pointer <= 0x20020000):
        raise RuntimeError(f"invalid application MSP: 0x{stack_pointer:08x}")
    if (reset_handler & 1) == 0 or not (APP_VECTOR <= (reset_handler & ~1) < PRIMARY_BASE + SLOT_SIZE):
        raise RuntimeError(f"invalid application reset vector: 0x{reset_handler:08x}")

    tlv_offset = header_size + image_size + protected_tlv_size
    if tlv_offset + 4 > len(data):
        raise RuntimeError("MCUboot TLV area is missing")
    tlv_magic, tlv_total = struct.unpack_from("<HH", data, tlv_offset)
    if tlv_magic != 0x6907 or tlv_offset + tlv_total > len(data):
        raise RuntimeError("MCUboot TLV area is malformed")

    types = set()
    cursor = tlv_offset + 4
    while cursor < tlv_offset + tlv_total:
        tlv_type, length = struct.unpack_from("<HH", data, cursor)
        cursor += 4 + length
        types.add(tlv_type)
    required = {0x01, 0x10, 0x22}  # key hash, SHA-256, ECDSA signature
    if not required.issubset(types):
        raise RuntimeError(f"required signed-image TLVs missing: {required - types}")

    return data


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--app-elf", type=pathlib.Path, required=True)
    parser.add_argument("--boot-elf", type=pathlib.Path, required=True)
    parser.add_argument("--signed", type=pathlib.Path, required=True)
    parser.add_argument("--factory", type=pathlib.Path, required=True)
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    args = parser.parse_args()

    app_vector = symbol_address(args.nm, args.app_elf, "g_pfnVectors")
    boot_vector = symbol_address(args.nm, args.boot_elf, "g_pfnVectors")
    if app_vector != APP_VECTOR:
        raise RuntimeError(f"application vector is 0x{app_vector:08x}, expected 0x{APP_VECTOR:08x}")
    if boot_vector != BOOT_BASE:
        raise RuntimeError(f"MCUboot vector is 0x{boot_vector:08x}, expected 0x{BOOT_BASE:08x}")

    verify_no_undefined_symbols(args.nm, args.app_elf)
    verify_no_undefined_symbols(args.nm, args.boot_elf)
    verify_required_symbols(args.nm, args.app_elf, {"app_main_task"})
    verify_required_symbols(
        args.nm, args.boot_elf,
        {"boot_watchdog_prepare", "boot_watchdog_feed"},
    )

    boot_bin = args.boot_elf.with_suffix(".bin").read_bytes()
    if len(boot_bin) > BOOT_SIZE:
        raise RuntimeError("MCUboot binary exceeds its 128 KiB code sector")
    signed = verify_signed_image(args.signed)

    factory = IntelHex(str(args.factory))
    boot_hex = IntelHex(str(args.boot_elf.with_suffix(".hex")))
    boot_from_hex = bytes(factory.tobinarray(start=BOOT_BASE,
                                             end=BOOT_BASE + BOOT_SIZE - 1))
    expected_boot = bytes(boot_hex.tobinarray(start=BOOT_BASE,
                                              end=BOOT_BASE + BOOT_SIZE - 1))
    app_from_hex = bytes(factory.tobinarray(start=PRIMARY_BASE,
                                            end=PRIMARY_BASE + len(signed) - 1))
    if boot_from_hex != expected_boot:
        raise RuntimeError("factory HEX bootloader content mismatch")
    if factory.start_addr != boot_hex.start_addr:
        raise RuntimeError("factory HEX does not preserve the MCUboot start address")
    if app_from_hex != signed:
        raise RuntimeError("factory HEX signed application content mismatch")
    diagnostics_end = BOOT_DIAGNOSTICS_BASE + BOOT_DIAGNOSTICS_SIZE
    for start, end in boot_hex.segments():
        if start < diagnostics_end and end > BOOT_DIAGNOSTICS_BASE:
            raise RuntimeError("MCUboot HEX overlaps the boot diagnostics sector")
    for start, end in factory.segments():
        in_boot = BOOT_BASE <= start and end <= BOOT_BASE + BOOT_SIZE
        in_primary = PRIMARY_BASE <= start and end <= PRIMARY_BASE + SLOT_SIZE
        if not (in_boot or in_primary):
            raise RuntimeError(
                f"factory HEX contains data outside boot/primary partitions: "
                f"0x{start:08x}-0x{end - 1:08x}"
            )
    if factory.maxaddr() >= 0x1FF00000:
        raise RuntimeError("factory HEX must not contain system-memory or option-byte data")

    print("MCUboot image verification passed")
    print(f"  bootloader: {len(boot_bin)} bytes @ 0x{BOOT_BASE:08x}")
    print(
        f"  boot diagnostics: 0x{BOOT_DIAGNOSTICS_BASE:08x}-"
        f"0x{diagnostics_end - 1:08x}"
    )
    print(f"  signed app: {len(signed)} bytes @ 0x{PRIMARY_BASE:08x}")
    print(f"  app vector: 0x{app_vector:08x}")
    print("  application/MCUboot unresolved symbols: none")
    print("  MCUboot watchdog prepare/feed chain: linked")


if __name__ == "__main__":
    main()
