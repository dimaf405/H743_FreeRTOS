"""Minimal bounds-checked ELF32 little-endian reader."""

from __future__ import annotations

import dataclasses
import pathlib
import struct
from collections.abc import Callable

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

