"""FlashFS 扫描、CRC、Flash-word 前进与持久化恢复不变量门禁。"""

from __future__ import annotations

import re

from architecture.common import (
    ROOT,
    Violation,
    line_for,
    strip_cpp_structure,
)


def _cpp_block_body(code: str, opening: int) -> str | None:
    if opening < 0 or opening >= len(code) or code[opening] != "{":
        return None
    depth = 0
    for index in range(opening, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return code[opening + 1:index]
    return None


def _cpp_function_body(code: str, signature: str) -> str | None:
    """Return one balanced, literal-free C++ function body."""
    matches = list(re.finditer(re.escape(signature), code))
    if len(matches) != 1:
        return None
    opening = code.find("{", matches[0].end())
    declaration_end = code.find(";", matches[0].end())
    if 0 <= declaration_end < opening:
        return None
    return _cpp_block_body(code, opening)


def _brace_depth_at(code: str, offset: int) -> int:
    return code[:offset].count("{") - code[:offset].count("}")


def scan_flashfs_contract(violations: list[Violation]) -> None:
    """解析去字面量后的函数体，验证损坏头不会驱动 size 跳转或越界扫描。"""
    flashfs_path = ROOT / "Dima/middleware/parameters/flashfs.cpp"
    if not flashfs_path.is_file():
        return

    flashfs_text = flashfs_path.read_text(encoding="utf-8")
    flashfs_code = "\n".join(strip_cpp_structure(flashfs_text))
    conditional_directive = re.search(
        r"(?m)^\s*#\s*(?:if|ifdef|ifndef|elif|else|endif)\b",
        flashfs_code,
    )
    if conditional_directive is not None:
        violations.append(Violation(
            flashfs_path,
            line_for(flashfs_text, conditional_directive.group(0).strip()),
            "R234",
            "FlashFS persistence invariants may not be hidden behind "
            "conditional preprocessing",
        ))

    header_guard = re.compile(
        r"if\s*\(\s*!\s*header_crc_valid\s*\(\s*hdr\s*\)\s*\)"
        r"\s*\{(?P<body>[^{}]*)\}",
        re.DOTALL,
    )
    magic_guard = re.compile(
        r"if\s*\(\s*hdr\s*\.\s*magic\s*!=\s*kMagicValid\s*\)"
    )
    size_access = re.compile(r"\bhdr\s*\.\s*size\b")
    header_call = re.compile(
        r"\bheader_crc_valid\s*\(\s*hdr\s*\)"
    )
    word_advance = re.compile(
        r"\boffset\s*\+=\s*kFlashWordBytes\s*;"
    )
    offset_mutation = re.compile(
        r"\boffset\s*(?:\+\+|--|[+\-*/%]?=)"
    )
    nested_control = re.compile(
        r"\b(?:if|for|while|do|switch|goto|return|break|throw)\b|\?"
    )
    scan_functions = (
        "int FlashFS::scan() noexcept",
        "int FlashFS::find_entry_locked(",
    )
    for signature in scan_functions:
        body = _cpp_function_body(flashfs_code, signature)
        guard_match = header_guard.search(body or "")
        magic_match = magic_guard.search(body or "")
        size_match = size_access.search(body or "")
        guard_body = (
            guard_match.group("body") if guard_match is not None else ""
        )
        word_advances = word_advance.findall(guard_body)
        word_advance_match = word_advance.search(guard_body)
        continues = list(re.finditer(r"\bcontinue\s*;", guard_body))
        same_depth = (
            body is not None and guard_match is not None and
            magic_match is not None and size_match is not None and
            len({
                _brace_depth_at(body, magic_match.start()),
                _brace_depth_at(body, guard_match.start()),
                _brace_depth_at(body, size_match.start()),
            }) == 1
        )
        valid_guard = (
            same_depth and
            magic_match.end() <= guard_match.start() < size_match.start() and
            len(header_call.findall(body)) == 1 and
            len(word_advances) == 1 and word_advance_match is not None and
            len(offset_mutation.findall(guard_body)) == 1 and
            len(continues) == 1 and
            word_advance_match.end() <= continues[0].start() and
            nested_control.search(guard_body) is None
        )
        if not valid_guard:
            violations.append(Violation(
                flashfs_path,
                line_for(flashfs_text, signature),
                "R234",
                "FlashFS must unconditionally validate the header CRC "
                "after magic and before size, then advance exactly one "
                "flashword",
            ))

    scan_body = _cpp_function_body(
        flashfs_code, "int FlashFS::scan() noexcept"
    )
    layout_commits = list(re.finditer(
        r"write_offset_\s*=\s*std::min\s*\(\s*"
        r"align_flashword\s*\(\s*high_water\s*\)\s*,\s*"
        r"partition_size_\s*\)\s*;",
        scan_body or "",
    ))
    scan_layout_valid = (
        scan_body is not None and len(layout_commits) == 1 and
        len(re.findall(
            r"\bwrite_offset_\s*(?:\+\+|--|[+\-*/%]?=)", scan_body
        )) == 1 and
        _brace_depth_at(scan_body, layout_commits[0].start()) == 0 and
        "reset_layout(" not in scan_body
    )
    if not scan_layout_valid:
        violations.append(Violation(
            flashfs_path,
            line_for(flashfs_text, "int FlashFS::scan()"),
            "R234",
            "FlashFS scan must commit its candidate high-water only after "
            "the complete partition scan",
        ))

    continue_body = _cpp_function_body(
        flashfs_code, "int FlashFS::continue_operation() noexcept"
    )
    header_phase = re.search(
        r"if\s*\(\s*operation_\s*==\s*Operation::ProgramHeader\s*\)",
        continue_body or "",
    )
    header_opening = (
        continue_body.find("{", header_phase.end())
        if continue_body is not None and header_phase is not None else -1
    )
    header_phase_body = _cpp_block_body(
        continue_body or "", header_opening
    )
    reservations = list(re.finditer(
        r"write_offset_\s*=\s*operation_entry_offset_\s*\+\s*"
        r"operation_total_size_\s*;",
        header_phase_body or "",
    ))
    header_programs = list(re.finditer(
        r"partition_\s*\.\s*program\s*\(",
        header_phase_body or "",
    ))
    reservation_valid = (
        header_phase_body is not None and len(reservations) == 1 and
        len(header_programs) == 1 and
        reservations[0].end() <= header_programs[0].start() and
        _brace_depth_at(header_phase_body, reservations[0].start()) == 0 and
        _brace_depth_at(header_phase_body, header_programs[0].start()) == 0 and
        len(re.findall(
            r"\bwrite_offset_\s*(?:\+\+|--|[+\-*/%]?=)",
            header_phase_body,
        )) == 1 and
        re.search(r"\bscan\s*\(", header_phase_body) is None
    )
    if not reservation_valid:
        violations.append(Violation(
            flashfs_path,
            line_for(flashfs_text, "Operation::ProgramHeader"),
            "R234",
            "FlashFS must reserve the complete record before attempting "
            "header programming and may not rescan away that reservation",
        ))

    begin_body = _cpp_function_body(
        flashfs_code, "int FlashFS::begin_write_entry("
    )
    payload_assignments = list(re.finditer(
        r"operation_header_\s*\.\s*crc\s*=\s*"
        r"crc\s*\^\s*UINT32_MAX\s*;",
        begin_body or "",
    ))
    header_assignments = list(re.finditer(
        r"operation_header_\s*\.\s*header_checksum\s*=\s*"
        r"header_crc\s*\(\s*operation_header_\s*\)\s*;",
        begin_body or "",
    ))
    program_header = list(re.finditer(
        r"operation_\s*=\s*Operation::ProgramHeader\s*;",
        begin_body or "",
    ))
    write_order_valid = (
        begin_body is not None and len(payload_assignments) == 1 and
        len(header_assignments) == 1 and len(program_header) == 1 and
        len(re.findall(
            r"operation_header_\s*\.\s*header_checksum",
            begin_body,
        )) == 1 and
        payload_assignments[0].end() <= header_assignments[0].start() <
        program_header[0].start() and
        _brace_depth_at(begin_body, payload_assignments[0].start()) == 0 and
        _brace_depth_at(begin_body, header_assignments[0].start()) == 0 and
        _brace_depth_at(begin_body, program_header[0].start()) == 0
    )
    if not write_order_valid:
        violations.append(Violation(
            flashfs_path,
            line_for(flashfs_text, "begin_write_entry("),
            "R234",
            "FlashFS must unconditionally store the recomputed header CRC "
            "after payload CRC finalization and before programming",
        ))

    validator_body = _cpp_function_body(
        flashfs_code, "bool FlashFS::header_crc_valid("
    )
    validator_valid = validator_body is not None and re.fullmatch(
        r"\s*return\s+header\s*\.\s*header_checksum\s*==\s*"
        r"header_crc\s*\(\s*header\s*\)\s*;\s*",
        validator_body,
    ) is not None
    crc_body = _cpp_function_body(
        flashfs_code, "std::uint32_t FlashFS::header_crc("
    )
    covered_fields = ("magic", "crc", "size", "token", "flag")
    field_updates = (
        len(re.findall(
            rf"\bcrc\s*=\s*crc32_update\s*\(\s*crc\s*,\s*"
            rf"reinterpret_cast\s*<\s*const\s+std::uint8_t\s*\*\s*>"
            rf"\s*\(\s*&\s*header\s*\.\s*{field}\s*\)\s*,\s*"
            rf"sizeof\s*\(\s*header\s*\.\s*{field}\s*\)\s*\)\s*;",
            crc_body or "",
            re.DOTALL,
        )) == 1
        for field in covered_fields
    )
    crc_mutations = re.findall(
        r"\bcrc\s*(?:\+\+|--|[+\-*/%^&|]?=)|(?:\+\+|--)\s*crc\b",
        crc_body or "",
    )
    crc_coverage_valid = (
        crc_body is not None and
        len(re.findall(r"\bcrc32_update\s*\(", crc_body)) == 5 and
        re.search(
            r"std::uint32_t\s+crc\s*=\s*UINT32_MAX\s*;", crc_body
        ) is not None and
        all(field_updates) and len(crc_mutations) == 6 and
        "header_checksum" not in crc_body and "reserved" not in crc_body and
        re.search(
            r"return\s+crc\s*\^\s*UINT32_MAX\s*;", crc_body
        ) is not None
    )
    if not validator_valid or not crc_coverage_valid:
        violations.append(Violation(
            flashfs_path,
            line_for(flashfs_text, "header_crc("),
            "R234",
            "FlashFS header CRC must cover magic/payload CRC/size/token/"
            "flag exactly and compare against its independent field",
        ))
