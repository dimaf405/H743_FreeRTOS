"""ELF memory accounting and final build artifact summary."""

from __future__ import annotations

import argparse
import pathlib
import re
import struct
import sys

from .formatting import color_enabled, colored, report_progress_error
from .models import ProgressError
from .plan import normalized_path

def integer_macro(header: str, name: str) -> int:
    match = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+(0x[0-9a-fA-F]+|[0-9]+)(?:U|L)*\s*$",
        header,
        flags=re.MULTILINE,
    )
    if not match:
        raise ProgressError(f"cannot read {name} from the boot layout")
    return int(match.group(1), 0)


def artifact_line(label: str, path: pathlib.Path, address: int | None = None) -> str:
    size = path.stat().st_size
    location = f" @ 0x{address:08x}" if address is not None else ""
    return f"  {label:<10} {normalized_path(str(path))} ({size} bytes{location})"


# Section-name → memory-region classification for the MCUboot-managed H743 app.
_FLASH_SECTIONS = frozenset({
    ".text", ".rodata", ".ARM.extab", ".ARM.exidx",
    ".preinit_array", ".init_array", ".fini_array",
    ".eh_frame", ".glue_7", ".glue_7t",
})
_FLASH_SECTION_PREFIXES = (".text.", ".rodata.")
_DTCM_SECTIONS = frozenset({".dima_ramfunc", ".data", ".bss"})
_DTCM_SECTION_PREFIXES = (".data.", ".bss.")


def _section_belongs(name: str, exact: frozenset[str], prefixes: tuple[str, ...]) -> bool:
    if name in exact:
        return True
    return any(name.startswith(p) for p in prefixes)


def _parse_elf_sections(
    elf_path: pathlib.Path,
) -> list[tuple[str, int, int]] | None:
    """Parse ELF section headers with pure Python (no external tools).

    Returns a list of ``(name, vma, size)`` tuples, or *None* on failure.
    """
    try:
        data = elf_path.read_bytes()
    except OSError:
        return None
    if len(data) < 16 or data[:4] != b"\x7fELF":
        return None

    ei_class = data[4]  # 1 = 32-bit, 2 = 64-bit
    ei_data = data[5]   # 1 = little-endian, 2 = big-endian
    endian = "<" if ei_data == 1 else ">"

    try:
        if ei_class == 1:  # 32-bit
            # Layout at offset 32: e_shoff(4), e_flags(4), e_ehsize(2),
            #   e_phentsize(2), e_phnum(2), e_shentsize(2), e_shnum(2),
            #   e_shstrndx(2)
            e_shoff = struct.unpack_from(f"{endian}I", data, 32)[0]
            e_shentsize, e_shnum, e_shstrndx = struct.unpack_from(
                f"{endian}HHH", data, 46
            )
        elif ei_class == 2:  # 64-bit
            # Layout at offset 40: e_shoff(8), e_flags(4), e_ehsize(2),
            #   e_phentsize(2), e_phnum(2), e_shentsize(2), e_shnum(2),
            #   e_shstrndx(2)
            e_shoff = struct.unpack_from(f"{endian}Q", data, 40)[0]
            e_shentsize, e_shnum, e_shstrndx = struct.unpack_from(
                f"{endian}HHH", data, 58
            )
        else:
            return None
    except struct.error:
        return None

    min_shentsize = 40 if ei_class == 1 else 64
    if e_shoff == 0 or e_shnum == 0 or e_shentsize < min_shentsize:
        return None

    def _read_shdr(index: int) -> tuple[int, int, int, int, int] | None:
        """Return (sh_name, sh_addr, sh_size, sh_flags, sh_offset)."""
        off = e_shoff + index * e_shentsize
        if off + e_shentsize > len(data):
            return None
        if ei_class == 1:
            # ELF32 Shdr: name(4) type(4) flags(4) addr(4) offset(4) size(4) ...
            sh_name, _, sh_flags, sh_addr, sh_offset, sh_size = struct.unpack_from(
                f"{endian}IIIIII", data, off
            )[:6]
        else:
            # ELF64 Shdr: name(4) type(4) flags(8) addr(8) offset(8) size(8) ...
            sh_name, _, sh_flags, sh_addr, sh_offset, sh_size = struct.unpack_from(
                f"{endian}IIQQQQ", data, off
            )[:6]
        return sh_name, sh_addr, sh_size, sh_flags, sh_offset

    # Read the section-header string table.
    strtab_info = _read_shdr(e_shstrndx)
    if strtab_info is None:
        return None
    strtab_file_off = strtab_info[4]  # sh_offset

    def _section_name(name_offset: int) -> str:
        start = strtab_file_off + name_offset
        end = data.index(b"\x00", start) if start < len(data) else start
        return data[start:end].decode("utf-8", errors="replace")

    sections: list[tuple[str, int, int]] = []
    for i in range(e_shnum):
        info = _read_shdr(i)
        if info is None:
            continue
        sh_name_idx, sh_addr, sh_size, _, _ = info
        name = _section_name(sh_name_idx)
        sections.append((name, sh_addr, sh_size))
    return sections


def parse_elf_memory_usage(
    elf_path: pathlib.Path,
    *,
    objdump: str = "",
) -> dict[str, int] | None:
    """Return per-region byte counts parsed from the ELF section headers.

    Keys: ``flash``, ``dtcm``, ``ram_d1``, ``ram_d2``, ``ram_dma``, ``ram_d3``.
    Returns ``None`` when the ELF cannot be parsed.
    """
    sections = _parse_elf_sections(elf_path)
    if sections is None:
        return None

    counts: dict[str, int] = {
        "flash": 0, "dtcm": 0,
        "ram_d1": 0, "ram_d2": 0, "ram_dma": 0, "ram_d3": 0,
    }
    for name, vma, size in sections:
        if size == 0:
            continue
        # Flash-resident sections (code + constants + init data load image).
        if _section_belongs(name, _FLASH_SECTIONS, _FLASH_SECTION_PREFIXES):
            counts["flash"] += size
            continue
        # DTCM sections (.dima_ramfunc, .data, .bss).
        if _section_belongs(name, _DTCM_SECTIONS, _DTCM_SECTION_PREFIXES):
            counts["dtcm"] += size
            continue
        # Classify remaining sections by VMA address range.
        if 0x08000000 <= vma < 0x08200000:
            # Flash-mapped (e.g. .isr_vector, .ARM).
            counts["flash"] += size
        elif 0x20000000 <= vma < 0x20020000:
            # DTCM (e.g. ._user_heap_stack).
            counts["dtcm"] += size
        elif 0x24000000 <= vma < 0x24080000:
            counts["ram_d1"] += size
        elif 0x30000000 <= vma < 0x30040000:
            counts["ram_d2"] += size
        elif 0x30040000 <= vma < 0x30048000:
            counts["ram_dma"] += size
        elif 0x38000000 <= vma < 0x38010000:
            counts["ram_d3"] += size
    return counts


def _fmt_bytes(value: int) -> str:
    if value >= 1024 * 1024:
        precision = 1 if value < 1024 * 1024 * 10 else 2
        return f"{value / (1024 * 1024):.{precision}f} MiB"
    if value >= 1024:
        return f"{value / 1024:.1f} KiB"
    return f"{value} B"


def _fmt_pct(used: int, total: int) -> str:
    if total <= 0:
        return "  n/a"
    return f"{used / total * 100:5.1f}%"


def print_memory_summary(
    counts: dict[str, int],
    *,
    layout_header: str,
    enabled: bool,
) -> None:
    """Print a compact Flash / RAM usage table."""
    try:
        layout = pathlib.Path(layout_header).read_text(encoding="utf-8")
        flash_total = integer_macro(layout, "H743_PRIMARY_SLOT_SIZE") - integer_macro(layout, "H743_MCUBOOT_HEADER_SIZE")
    except ProgressError:
        flash_total = 0

    # Linker-script capacities (from the MEMORY block).
    dtcm_total = 128 * 1024
    ram_d1_total = 512 * 1024
    ram_d2_total = 256 * 1024
    ram_dma_total = 32 * 1024
    ram_d3_total = 64 * 1024

    flash_used = counts.get("flash", 0)
    dtcm_used = counts.get("dtcm", 0)
    ram_d1_used = counts.get("ram_d1", 0)
    ram_d2_used = counts.get("ram_d2", 0)
    ram_dma_used = counts.get("ram_dma", 0)
    ram_d3_used = counts.get("ram_d3", 0)

    heading = colored("Memory usage", "1;36", enabled=enabled)
    print(f"  {heading}")

    # Flash line.
    flash_bar = _bar(flash_used, flash_total, enabled=enabled)
    print(
        f"    {'Flash':<10} {flash_used:>7,} / {flash_total:>7,} B"
        f"  ({_fmt_bytes(flash_used)} / {_fmt_bytes(flash_total)})"
        f"  {_fmt_pct(flash_used, flash_total)}  {flash_bar}"
    )

    # DTCM line (code-executable RAM + .data + .bss).
    dtcm_bar = _bar(dtcm_used, dtcm_total, enabled=enabled)
    print(
        f"    {'DTCM':<10} {dtcm_used:>7,} / {dtcm_total:>7,} B"
        f"  ({_fmt_bytes(dtcm_used)} / {_fmt_bytes(dtcm_total)})"
        f"  {_fmt_pct(dtcm_used, dtcm_total)}  {dtcm_bar}"
    )

    # Combined SRAM line.
    sram_used = ram_d1_used + ram_d2_used + ram_dma_used + ram_d3_used
    sram_capacity = ram_d1_total + ram_d2_total + ram_dma_total + ram_d3_total
    sram_bar = _bar(sram_used, sram_capacity, enabled=enabled)
    print(
        f"    {'SRAM':<10} {sram_used:>7,} / {sram_capacity:>7,} B"
        f"  ({_fmt_bytes(sram_used)} / {_fmt_bytes(sram_capacity)})"
        f"  {_fmt_pct(sram_used, sram_capacity)}  {sram_bar}"
    )
    # Sub-lines for each SRAM region (only if non-zero).
    for label, used, capacity in [
        ("  D1 heap", ram_d1_used, ram_d1_total),
        ("  D2 SRAM", ram_d2_used, ram_d2_total),
        ("  DMA",     ram_dma_used, ram_dma_total),
        ("  D3 diag", ram_d3_used, ram_d3_total),
    ]:
        if used > 0:
            print(
                f"    {label:<10} {used:>7,} / {capacity:>7,} B"
                f"  ({_fmt_bytes(used)} / {_fmt_bytes(capacity)})"
                f"  {_fmt_pct(used, capacity)}"
            )

    # Total RAM (DTCM + all SRAM).
    total_used = flash_used + dtcm_used + sram_used
    total_cap = flash_total + dtcm_total + sram_capacity
    print(
        f"    {'-' * 10} {'-' * 7}   {'-' * 7}"
    )
    print(
        f"    {'Total':<10} {total_used:>7,} / {total_cap:>7,} B"
        f"  ({_fmt_bytes(total_used)} / {_fmt_bytes(total_cap)})"
        f"  {_fmt_pct(total_used, total_cap)}"
    )


def _bar(used: int, total: int, *, enabled: bool, width: int = 20) -> str:
    if total <= 0:
        return ""
    filled = max(0, min(width, round(used / total * width)))
    empty = width - filled
    pct = used / total
    if pct >= 0.9:
        color = "1;31"  # red
    elif pct >= 0.7:
        color = "1;33"  # yellow
    else:
        color = "1;32"  # green
    # Keep redirected and native Windows GBK output encodable. ANSI color can
    # still decorate the bar, but the glyphs themselves remain portable ASCII.
    bar_text = f"[{'#' * filled}{'.' * empty}]"
    return colored(bar_text, color, enabled=enabled)


def summary(arguments: argparse.Namespace) -> int:
    try:
        layout = pathlib.Path(arguments.layout).read_text(encoding="utf-8")
        boot_address = integer_macro(layout, "H743_MCUBOOT_BASE")
        app_address = integer_macro(layout, "H743_PRIMARY_SLOT_BASE")
        header_size = integer_macro(layout, "H743_MCUBOOT_HEADER_SIZE")
        vector_address = app_address + header_size

        enabled = color_enabled(disabled=arguments.no_color, stream=sys.stdout)
        print(colored("Build complete", "1;32", enabled=enabled))
        print(f"  {'Target:':<10} {arguments.goals}")
        print(f"  {'Version:':<10} {arguments.version}")

        app_elf = pathlib.Path(arguments.app_elf)
        boot_bin = pathlib.Path(arguments.boot_bin)
        signed = pathlib.Path(arguments.signed)
        factory = pathlib.Path(arguments.factory)
        if app_elf.is_file():
            print(f"  {'App ELF:':<10} {normalized_path(str(app_elf))}")
        if boot_bin.is_file():
            print(artifact_line("MCUboot:", boot_bin, boot_address))
        if signed.is_file():
            print(artifact_line("Signed:", signed, app_address))
            print(f"  {'Vector:':<10} 0x{vector_address:08x}")
        if factory.is_file():
            print(artifact_line("Factory:", factory))

        # Flash & RAM usage from the application ELF (pure-Python parser).
        if app_elf.is_file():
            counts = parse_elf_memory_usage(app_elf)
            if counts is not None:
                print()
                print_memory_summary(
                    counts,
                    layout_header=arguments.layout,
                    enabled=enabled,
                )
    except (OSError, ProgressError) as error:
        report_progress_error(str(error), no_color=arguments.no_color)
        return 2
    return 0
