#!/usr/bin/env python3
"""Merge non-overlapping Intel HEX images without touching option bytes."""

import argparse

from intelhex import IntelHex


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("inputs", nargs="+")
    args = parser.parse_args()

    merged = IntelHex()
    factory_start_addr = None
    for index, filename in enumerate(args.inputs):
        image = IntelHex(filename)
        if index == 0:
            # The first image is the bootloader.  Intel HEX start-address
            # records are execution metadata rather than bytes in flash, so
            # application Reset_Handler metadata must not conflict with it.
            factory_start_addr = image.start_addr
        image.start_addr = None
        merged.merge(image, overlap="error")

    merged.start_addr = factory_start_addr
    merged.write_hex_file(args.output, byte_count=32)


if __name__ == "__main__":
    main()
