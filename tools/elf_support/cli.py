"""Stable command-line H743 application ELF acceptance workflow."""

from __future__ import annotations

import argparse
import pathlib
import sys
from collections.abc import Iterable

from .layout import (
    APP_VECTOR,
    DMA_BASE,
    DMA_SIZE,
    INIT_ARRAY_ALLOWLIST,
    verify_handler_symbols,
    verify_initialization_arrays,
    verify_memory_layout,
)
from .reader import Elf32, ElfVerificationError
from .symbols import (
    verify_actuator_symbols,
    verify_forbidden_symbols,
    verify_lifecycle_symbols,
)

def verify(elf_path: pathlib.Path) -> None:
    elf = Elf32(elf_path)
    verify_handler_symbols(elf)
    verify_initialization_arrays(elf)
    verify_memory_layout(elf)
    verify_lifecycle_symbols(elf)
    verify_actuator_symbols(elf)
    verify_forbidden_symbols(elf)
    print("application ELF lifecycle verification passed")
    print(f"  vector: 0x{APP_VECTOR:08x}")
    print(f"  init array: {len(INIT_ARRAY_ALLOWLIST)} allowed entries")
    print(f"  DMA region: 0x{DMA_BASE:08x}, {DMA_SIZE} bytes maximum")
    print("  six-channel safety-gated PWM chain: linked")
    print("  SBUS/Commander/IWDG health chain: linked")


def parse_args(arguments: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify H743 application ELF architecture contracts",
    )
    parser.add_argument("--elf", required=True, type=pathlib.Path)
    return parser.parse_args(arguments)


def main(arguments: Iterable[str] | None = None) -> int:
    args = parse_args(arguments)
    try:
        verify(args.elf)
    except ElfVerificationError as error:
        print(f"application ELF lifecycle verification failed: {error}",
              file=sys.stderr)
        return 1
    return 0

