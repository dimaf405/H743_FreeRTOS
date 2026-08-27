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
    verify_memory_layout,
    verify_vector_symbol,
)
from .reader import Elf32, ElfVerificationError

def verify(elf_path: pathlib.Path) -> None:
    elf = Elf32(elf_path)
    verify_vector_symbol(elf)
    verify_memory_layout(elf)
    print("application ELF layout verification passed")
    print(f"  vector: 0x{APP_VECTOR:08x}")
    print(f"  DMA region: 0x{DMA_BASE:08x}, {DMA_SIZE} bytes maximum")


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
        print(f"application ELF layout verification failed: {error}",
              file=sys.stderr)
        return 1
    return 0
