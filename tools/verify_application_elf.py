#!/usr/bin/env python3
"""Verify H743 application ELF lifecycle and hardware ownership contracts."""

from __future__ import annotations

import argparse
import dataclasses
import pathlib
import struct
import sys
from collections.abc import Callable, Iterable


ELF_MAGIC = b"\x7fELF"
ELFCLASS32 = 1
ELFDATA2LSB = 1
ET_EXEC = 2
EM_ARM = 40
SHT_SYMTAB = 2
SHT_NOBITS = 8
SHN_UNDEF = 0
SHN_XINDEX = 0xFFFF
STB_GLOBAL = 1
STB_WEAK = 2
STT_NOTYPE = 0
STT_OBJECT = 1
STT_FUNC = 2

APP_VECTOR = 0x08040400
DTCM_BASE = 0x20000000
DTCM_SIZE = 128 * 1024
RAM_D1_BASE = 0x24000000
RAM_D1_SIZE = 512 * 1024
RAM_D2_BASE = 0x30000000
RAM_D2_SIZE = 256 * 1024
DMA_BASE = 0x30040000
DMA_SIZE = 32 * 1024
D3_DIAGNOSTICS_BASE = 0x38000000
D3_DIAGNOSTICS_SIZE = 512
PLATFORM_HEAP_SIZE = 256 * 1024
TASK_POOL_BASE = RAM_D1_BASE + PLATFORM_HEAP_SIZE
TASK_POOL_MAX_SIZE = 48 * 1024

INIT_ARRAY_ALLOWLIST = {
    "register_fini",
    "frame_dummy",
    "_GLOBAL__sub_I__ZN4dima10parameters6detail16transaction_lockEv",
    "_GLOBAL__sub_I___orb_action_request",
    "_GLOBAL__sub_I___orb_actuator_armed",
    "_GLOBAL__sub_I___orb_input_rc",
    "_GLOBAL__sub_I___orb_manual_control_setpoint",
    "_GLOBAL__sub_I___orb_manual_control_switches",
    "_GLOBAL__sub_I___orb_parameter_update",
    "_GLOBAL__sub_I___orb_rc_channels",
    "_GLOBAL__sub_I___orb_vehicle_control_mode",
    "_GLOBAL__sub_I___orb_vehicle_status",
}
FINI_ARRAY_ALLOWLIST = {"__do_global_dtors_aux"}

FORBIDDEN_APPLICATION_SYMBOLS = {
    "__orb_app_heartbeat",
    "dima_boot_diagnostics_capture_pending",
    "dima_boot_diagnostics_store_enable",
    "dima_boot_diagnostics_store_pending",
}
FORBIDDEN_APPLICATION_FRAGMENTS = {
    "HelloWorld",
    "app_heartbeat",
}
FORBIDDEN_ACTUATOR_SYMBOLS = {
    "HAL_TIM_PWM_Start",
    "HAL_TIM_PWM_Stop",
    "HAL_TIMEx_PWMN_Start",
    "HAL_TIMEx_PWMN_Stop",
    "board_motor_pwm_start",
    "board_motor_pwm_started",
    "board_motor_pwm_stop",
    "board_motor_pwm_write",
}
FORBIDDEN_ACTUATOR_FRAGMENTS = {
    "ActuatorOutput",
    "FunctionMotors",
    "Mixer",
    "MixingOutput",
    "RoverDifferential",
}


class ElfVerificationError(RuntimeError):
    """An ELF file violates a required application contract."""


@dataclasses.dataclass(frozen=True)
class Section:
    index: int
    name: str
    section_type: int
    address: int
    offset: int
    size: int
    link: int
    entry_size: int


@dataclasses.dataclass(frozen=True)
class Symbol:
    name: str
    value: int
    size: int
    binding: int
    symbol_type: int
    section_index: int

    @property
    def defined(self) -> bool:
        return self.section_index != SHN_UNDEF

    @property
    def normalized_address(self) -> int:
        return self.value & ~1


class Elf32:
    """Minimal bounds-checked ELF32 little-endian reader."""

    def __init__(self, path: pathlib.Path) -> None:
        self.path = path
        try:
            self.data = path.read_bytes()
        except OSError as error:
            raise ElfVerificationError(
                f"cannot read '{path}': {error}"
            ) from error
        self.sections: list[Section] = []
        self.symbols: list[Symbol] = []
        self._parse()

    def _slice(self, offset: int, size: int, description: str) -> bytes:
        if offset < 0 or size < 0 or offset + size > len(self.data):
            raise ElfVerificationError(
                f"{description} extends outside the ELF file"
            )
        return self.data[offset:offset + size]

    @staticmethod
    def _cstring(table: bytes, offset: int, description: str) -> str:
        if offset < 0 or offset >= len(table):
            raise ElfVerificationError(
                f"{description} string offset is outside its string table"
            )
        end = table.find(b"\0", offset)
        if end < 0:
            raise ElfVerificationError(
                f"{description} is not terminated in its string table"
            )
        return table[offset:end].decode("utf-8", errors="replace")

    def _parse(self) -> None:
        header_format = "<16sHHIIIIIHHHHHH"
        header_size = struct.calcsize(header_format)
        if len(self.data) < header_size:
            raise ElfVerificationError("ELF header is truncated")
        fields = struct.unpack_from(header_format, self.data, 0)
        identification = fields[0]
        if identification[:4] != ELF_MAGIC:
            raise ElfVerificationError("file does not have an ELF signature")
        if identification[4] != ELFCLASS32:
            raise ElfVerificationError("application must be ELF32")
        if identification[5] != ELFDATA2LSB:
            raise ElfVerificationError("application ELF must be little-endian")
        if identification[6] != 1:
            raise ElfVerificationError("unsupported ELF identification version")

        elf_type, machine, version = fields[1:4]
        if elf_type != ET_EXEC:
            raise ElfVerificationError("application ELF must be ET_EXEC")
        if machine != EM_ARM:
            raise ElfVerificationError("application ELF is not for ARM")
        if version != 1:
            raise ElfVerificationError("unsupported ELF header version")

        section_offset = fields[6]
        section_entry_size = fields[11]
        section_count = fields[12]
        section_names_index = fields[13]
        expected_section_size = struct.calcsize("<IIIIIIIIII")
        if section_count == 0 or section_names_index == SHN_XINDEX:
            raise ElfVerificationError(
                "extended ELF section indexes are not supported"
            )
        if section_entry_size < expected_section_size:
            raise ElfVerificationError("ELF section header size is invalid")
        if section_names_index >= section_count:
            raise ElfVerificationError("ELF section-name table index is invalid")
        self._slice(
            section_offset, section_entry_size * section_count,
            "section header table",
        )

        raw_sections: list[tuple[int, ...]] = []
        for index in range(section_count):
            offset = section_offset + index * section_entry_size
            raw_sections.append(struct.unpack_from(
                "<IIIIIIIIII", self.data, offset,
            ))

        names_header = raw_sections[section_names_index]
        names = self._slice(
            names_header[4], names_header[5], "section-name string table",
        )
        for index, raw in enumerate(raw_sections):
            name = self._cstring(names, raw[0], f"section {index} name")
            self.sections.append(Section(
                index=index,
                name=name,
                section_type=raw[1],
                address=raw[3],
                offset=raw[4],
                size=raw[5],
                link=raw[6],
                entry_size=raw[9],
            ))
        self._parse_symbols()

    def _parse_symbols(self) -> None:
        symbol_sections = [
            section for section in self.sections
            if section.section_type == SHT_SYMTAB
        ]
        if not symbol_sections:
            raise ElfVerificationError(
                "ELF has no static symbol table; lifecycle gates cannot run"
            )
        symbol_size = struct.calcsize("<IIIBBH")
        for section in symbol_sections:
            if section.link >= len(self.sections):
                raise ElfVerificationError(
                    f"symbol table '{section.name}' has an invalid string table"
                )
            entry_size = section.entry_size or symbol_size
            if entry_size < symbol_size or section.size % entry_size != 0:
                raise ElfVerificationError(
                    f"symbol table '{section.name}' has invalid entries"
                )
            strings_section = self.sections[section.link]
            strings = self.section_data(strings_section)
            table = self.section_data(section)
            for offset in range(0, len(table), entry_size):
                name_offset, value, size, info, _, section_index = \
                    struct.unpack_from("<IIIBBH", table, offset)
                name = self._cstring(
                    strings, name_offset,
                    f"symbol {offset // entry_size} name",
                )
                self.symbols.append(Symbol(
                    name=name,
                    value=value,
                    size=size,
                    binding=info >> 4,
                    symbol_type=info & 0x0F,
                    section_index=section_index,
                ))

    def section_data(self, section: Section) -> bytes:
        if section.section_type == SHT_NOBITS:
            raise ElfVerificationError(
                f"section '{section.name}' has no file-backed contents"
            )
        return self._slice(
            section.offset, section.size, f"section '{section.name}'",
        )

    def section(self, name: str, required: bool = True) -> Section | None:
        matches = [section for section in self.sections if section.name == name]
        if not matches:
            if required:
                raise ElfVerificationError(f"required section '{name}' is missing")
            return None
        if len(matches) != 1:
            raise ElfVerificationError(f"section '{name}' is duplicated")
        return matches[0]

    def symbol(self, name: str) -> Symbol:
        matches = [
            symbol for symbol in self.symbols
            if symbol.name == name and symbol.defined
        ]
        if not matches:
            raise ElfVerificationError(f"required symbol '{name}' is missing")
        if len(matches) != 1:
            raise ElfVerificationError(f"symbol '{name}' is duplicated")
        return matches[0]

    def symbols_matching(
            self, predicate: Callable[[Symbol], bool]) -> list[Symbol]:
        return [
            symbol for symbol in self.symbols
            if symbol.defined and predicate(symbol)
        ]


def range_contains(base: int, size: int, address: int, length: int) -> bool:
    return length > 0 and base <= address and address + length <= base + size


def verify_section(
        elf: Elf32, name: str, address: int, maximum_size: int,
        exact_size: int | None = None) -> Section:
    section = elf.section(name)
    assert section is not None
    if section.address != address:
        raise ElfVerificationError(
            f"section '{name}' starts at 0x{section.address:08x}, "
            f"expected 0x{address:08x}"
        )
    if section.size == 0:
        raise ElfVerificationError(f"section '{name}' is empty")
    if section.size > maximum_size:
        raise ElfVerificationError(
            f"section '{name}' uses {section.size} bytes, limit is "
            f"{maximum_size} bytes"
        )
    if exact_size is not None and section.size != exact_size:
        raise ElfVerificationError(
            f"section '{name}' uses {section.size} bytes, expected exactly "
            f"{exact_size} bytes"
        )
    return section


def verify_handler_symbols(elf: Elf32) -> None:
    vector = elf.symbol("g_pfnVectors")
    if vector.value != APP_VECTOR:
        raise ElfVerificationError(
            f"g_pfnVectors is 0x{vector.value:08x}, expected "
            f"0x{APP_VECTOR:08x}"
        )

    default_handler = elf.symbol("Default_Handler")
    for name in ("SysTick_Handler", "TIM2_IRQHandler"):
        handler = elf.symbol(name)
        if handler.binding != STB_GLOBAL or handler.symbol_type != STT_FUNC:
            raise ElfVerificationError(
                f"{name} must be a strong global function"
            )
        if handler.normalized_address == default_handler.normalized_address:
            raise ElfVerificationError(f"{name} still resolves to Default_Handler")

    tim12_vector = elf.symbol("TIM8_BRK_TIM12_IRQHandler")
    if tim12_vector.binding != STB_WEAK:
        raise ElfVerificationError(
            "TIM8_BRK_TIM12_IRQHandler must remain a weak startup vector"
        )
    if tim12_vector.normalized_address != default_handler.normalized_address:
        raise ElfVerificationError(
            "TIM8_BRK_TIM12_IRQHandler unexpectedly owns a handler"
        )
    if elf.symbols_matching(lambda symbol: symbol.name == "htim12"):
        raise ElfVerificationError("htim12 is linked into the application")


def array_entry_names(
        elf: Elf32, section_name: str,
        allowlist: set[str], required: bool = True) -> list[str]:
    section = elf.section(section_name, required=required)
    if section is None:
        return []
    if section.size % 4 != 0:
        raise ElfVerificationError(
            f"section '{section_name}' is not an array of 32-bit pointers"
        )
    if section.size == 0:
        if allowlist:
            raise ElfVerificationError(
                f"section '{section_name}' is empty; expected "
                f"{len(allowlist)} entries"
            )
        return []
    data = elf.section_data(section)
    pointers = struct.unpack(f"<{section.size // 4}I", data)
    if len(pointers) != len(allowlist):
        raise ElfVerificationError(
            f"section '{section_name}' has {len(pointers)} entries; "
            f"allowlist has {len(allowlist)}"
        )

    resolved: list[str] = []
    for index, pointer in enumerate(pointers):
        normalized = pointer & ~1
        candidates = {
            symbol.name for symbol in elf.symbols
            if symbol.defined and symbol.name and
            symbol.symbol_type in {STT_NOTYPE, STT_FUNC} and
            symbol.normalized_address == normalized
        }
        allowed = candidates & allowlist
        if len(allowed) != 1:
            detail = ", ".join(sorted(candidates)) or "no symbol"
            raise ElfVerificationError(
                f"{section_name}[{index}] at 0x{pointer:08x} does not "
                f"resolve uniquely to the allowlist ({detail})"
            )
        resolved.append(next(iter(allowed)))
    if set(resolved) != allowlist or len(set(resolved)) != len(resolved):
        missing = sorted(allowlist - set(resolved))
        raise ElfVerificationError(
            f"section '{section_name}' has duplicate or missing entries: "
            f"missing={missing}"
        )
    return resolved


def verify_initialization_arrays(elf: Elf32) -> None:
    preinit = elf.section(".preinit_array", required=False)
    if preinit is not None and preinit.size != 0:
        raise ElfVerificationError(".preinit_array must remain empty")
    array_entry_names(elf, ".init_array", INIT_ARRAY_ALLOWLIST)
    array_entry_names(elf, ".fini_array", FINI_ARRAY_ALLOWLIST)


def verify_memory_layout(elf: Elf32) -> None:
    vector = elf.section(".isr_vector")
    assert vector is not None
    if vector.address != APP_VECTOR or vector.size == 0:
        raise ElfVerificationError(
            ".isr_vector is empty or does not start at 0x08040400"
        )
    verify_section(elf, ".dima_ramfunc", DTCM_BASE, DTCM_SIZE)
    dma = verify_section(elf, ".dima_dma", DMA_BASE, DMA_SIZE)
    verify_section(
        elf, ".dima_heap", RAM_D1_BASE, PLATFORM_HEAP_SIZE,
        exact_size=PLATFORM_HEAP_SIZE,
    )
    verify_section(
        elf, ".dima_task_pool", TASK_POOL_BASE, TASK_POOL_MAX_SIZE,
    )
    verify_section(
        elf, ".dima_boot_diag", D3_DIAGNOSTICS_BASE,
        D3_DIAGNOSTICS_SIZE,
    )

    receive_ring = unique_fragment_symbol(elf, "g_receive_ring")
    if receive_ring.symbol_type != STT_OBJECT or receive_ring.size == 0:
        raise ElfVerificationError("g_receive_ring is not a sized object")
    receive_section = symbol_section(elf, receive_ring)
    if receive_section.name != ".bss":
        raise ElfVerificationError(
            f"g_receive_ring is in '{receive_section.name}', expected CPU .bss"
        )
    cpu_regions = (
        (DTCM_BASE, DTCM_SIZE),
        (RAM_D1_BASE, RAM_D1_SIZE),
        (RAM_D2_BASE, RAM_D2_SIZE),
    )
    if not any(range_contains(base, size, receive_ring.value,
                              receive_ring.size)
               for base, size in cpu_regions):
        raise ElfVerificationError("g_receive_ring is outside CPU RAM")
    if range_contains(DMA_BASE, DMA_SIZE, receive_ring.value,
                      receive_ring.size):
        raise ElfVerificationError("g_receive_ring overlaps DMA-owned SRAM")

    dma_buffer = unique_fragment_symbol(elf, "g_dma_buffer")
    if (dma_buffer.symbol_type != STT_OBJECT or dma_buffer.size != 64 or
            not range_contains(DMA_BASE, DMA_SIZE, dma_buffer.value,
                               dma_buffer.size)):
        raise ElfVerificationError(
            "g_dma_buffer must be a 64-byte object in DMA-owned SRAM"
        )
    dma_buffer_section = symbol_section(elf, dma_buffer)
    if dma_buffer_section.index != dma.index:
        raise ElfVerificationError("g_dma_buffer is not in .dima_dma")


def symbol_section(elf: Elf32, symbol: Symbol) -> Section:
    if symbol.section_index >= len(elf.sections):
        raise ElfVerificationError(
            f"symbol '{symbol.name}' has an unsupported section index"
        )
    return elf.sections[symbol.section_index]


def unique_fragment_symbol(elf: Elf32, fragment: str) -> Symbol:
    matches = elf.symbols_matching(lambda symbol: fragment in symbol.name)
    if not matches:
        raise ElfVerificationError(
            f"required symbol containing '{fragment}' is missing"
        )
    if len(matches) != 1:
        names = ", ".join(symbol.name for symbol in matches)
        raise ElfVerificationError(
            f"symbol fragment '{fragment}' is ambiguous: {names}"
        )
    return matches[0]


def require_symbol_match(
        elf: Elf32, description: str,
        predicate: Callable[[Symbol], bool]) -> None:
    matches = elf.symbols_matching(predicate)
    if not matches:
        raise ElfVerificationError(
            f"required lifecycle symbol is missing: {description}"
        )


def verify_lifecycle_symbols(elf: Elf32) -> None:
    for name in (
        "param_shutdown",
        "_ZN3px419work_queue_shutdownEv",
        "_ZN4uORB15lifecycle_epochEv",
        "_ZN4uORB8shutdownEv",
    ):
        elf.symbol(name)
    unique_fragment_symbol(elf, "g_owner_task")
    require_symbol_match(
        elf, "FreeRTOS Backend::destroy(TaskHandle)",
        lambda symbol: "7Backend7destroyE" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    require_symbol_match(
        elf, "ParameterJournal::shutdown()",
        lambda symbol: "16ParameterJournal8shutdownEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    require_symbol_match(
        elf, "ParameterService::shutdown()",
        lambda symbol: "16ParameterService8shutdownEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )


def verify_forbidden_symbols(elf: Elf32) -> None:
    forbidden_names = FORBIDDEN_APPLICATION_SYMBOLS | FORBIDDEN_ACTUATOR_SYMBOLS
    present_names = sorted({
        symbol.name for symbol in elf.symbols
        if symbol.defined and symbol.name in forbidden_names
    })
    forbidden_fragments = (
        FORBIDDEN_APPLICATION_FRAGMENTS | FORBIDDEN_ACTUATOR_FRAGMENTS
    )
    present_fragments = sorted({
        fragment
        for symbol in elf.symbols
        if symbol.defined
        for fragment in forbidden_fragments
        if fragment in symbol.name
    })
    if present_names or present_fragments:
        raise ElfVerificationError(
            "forbidden Application symbols are linked: "
            f"names={present_names}, fragments={present_fragments}"
        )


def verify(elf_path: pathlib.Path) -> None:
    elf = Elf32(elf_path)
    verify_handler_symbols(elf)
    verify_initialization_arrays(elf)
    verify_memory_layout(elf)
    verify_lifecycle_symbols(elf)
    verify_forbidden_symbols(elf)
    print("application ELF lifecycle verification passed")
    print(f"  vector: 0x{APP_VECTOR:08x}")
    print(f"  init array: {len(INIT_ARRAY_ALLOWLIST)} allowed entries")
    print(f"  DMA region: 0x{DMA_BASE:08x}, {DMA_SIZE} bytes maximum")
    print("  actuator consumers: absent")


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


if __name__ == "__main__":
    raise SystemExit(main())
