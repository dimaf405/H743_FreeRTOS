#!/usr/bin/env python3
"""从小型确定性 .msg schema 生成 Dima uORB C++ 合同。

schema 只支持本固件实际使用的定宽标量、定长数组、整数常量、一个外部 C 类型和一个
生成别名；ORB_DEFINE 仍是唯一 metadata 构造入口，保持现有链接段与运行期 ABI。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path


NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")
MEMBER_RE = re.compile(r"^[a-z][a-z0-9_]*$")
CONSTANT_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
DECLARATION_RE = re.compile(
    r"^(?P<type>[a-z][a-z0-9]*)"
    r"(?:\[(?P<array>[A-Za-z_][A-Za-z0-9_]*|[0-9]+)\])?"
    r"\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)"
    r"(?:\s*=\s*(?P<value>[A-Za-z_][A-Za-z0-9_]*|-?[0-9]+))?$"
)
INTEGER_RE = re.compile(r"^-?[0-9]+$")
HEADER_RE = re.compile(r"^[A-Za-z0-9_./-]+\.(?:h|hpp)$")


@dataclass(frozen=True)
class Primitive:
    cpp: str
    size: int
    alignment: int
    unsigned: bool = False


PRIMITIVES = {
    "bool": Primitive("bool", 1, 1),
    "char": Primitive("char", 1, 1),
    "int8": Primitive("std::int8_t", 1, 1),
    "uint8": Primitive("std::uint8_t", 1, 1, True),
    "int16": Primitive("std::int16_t", 2, 2),
    "uint16": Primitive("std::uint16_t", 2, 2, True),
    "int32": Primitive("std::int32_t", 4, 4),
    "uint32": Primitive("std::uint32_t", 4, 4, True),
    "int64": Primitive("std::int64_t", 8, 8),
    "uint64": Primitive("std::uint64_t", 8, 8, True),
    "float32": Primitive("float", 4, 4),
    "float64": Primitive("double", 8, 8),
}
INTEGER_LIMITS = {
    "int8": (-(1 << 7), (1 << 7) - 1),
    "uint8": (0, (1 << 8) - 1),
    "int16": (-(1 << 15), (1 << 15) - 1),
    "uint16": (0, (1 << 16) - 1),
    "int32": (-(1 << 31), (1 << 31) - 1),
    "uint32": (0, (1 << 32) - 1),
    "int64": (-(1 << 63), (1 << 63) - 1),
    "uint64": (0, (1 << 64) - 1),
}


@dataclass(frozen=True)
class Constant:
    primitive: str
    name: str
    expression: str
    comment: str


@dataclass(frozen=True)
class Field:
    primitive: str
    name: str
    array: str | None
    comment: str


@dataclass
class Message:
    path: Path
    topic: str
    queue_size: int | None = None
    kind: str = "struct"
    external_header: str | None = None
    underlying_type: str | None = None
    abi_size: int | None = None
    abi_alignment: int | None = None
    constants: list[Constant] | None = None
    fields: list[Field] | None = None

    def __post_init__(self) -> None:
        self.constants = [] if self.constants is None else self.constants
        self.fields = [] if self.fields is None else self.fields

    @property
    def public_type(self) -> str:
        return f"{self.topic}_s"

    @property
    def metadata_type(self) -> str:
        return self.underlying_type or self.public_type


@dataclass(frozen=True)
class FieldLayout:
    name: str
    offset: int


@dataclass(frozen=True)
class Layout:
    size: int
    alignment: int
    fields: tuple[FieldLayout, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--schemas", required=True, type=Path)
    parser.add_argument("--lock", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--print-lock", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if args.print_lock:
        if args.output is not None or args.verify_only:
            parser.error("--print-lock cannot be combined with output options")
    elif args.lock is None:
        parser.error("--lock is required unless --print-lock is used")
    elif not args.verify_only and args.output is None:
        parser.error("--output is required unless --verify-only is used")
    return args


def split_comment(line: str) -> tuple[str, str]:
    declaration, separator, comment = line.partition("#")
    return declaration.strip(), comment.strip() if separator else ""


def parse_positive(value: str, path: Path, line: int, label: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as error:
        raise RuntimeError(f"{path}:{line}: invalid {label}") from error
    if parsed <= 0:
        raise RuntimeError(f"{path}:{line}: {label} must be positive")
    return parsed


def parse_directive(message: Message, line: str, line_number: int) -> None:
    tokens = line.split()
    directive = tokens[0]
    if directive == "@queue" and len(tokens) == 2:
        if message.queue_size is not None:
            raise RuntimeError(
                f"{message.path}:{line_number}: duplicate @queue"
            )
        queue_size = parse_positive(
            tokens[1], message.path, line_number, "queue size"
        )
        if queue_size > 0xFF:
            raise RuntimeError(
                f"{message.path}:{line_number}: queue size exceeds metadata"
            )
        message.queue_size = queue_size
        return
    if directive in {"@external", "@alias"} and len(tokens) == 3:
        if message.kind != "struct" or message.fields or message.constants:
            raise RuntimeError(
                f"{message.path}:{line_number}: type directive must precede fields"
            )
        header, cpp_type = tokens[1], tokens[2]
        if (HEADER_RE.fullmatch(header) is None or header.startswith("/") or
                ".." in Path(header).parts or
                re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", cpp_type) is None):
            raise RuntimeError(
                f"{message.path}:{line_number}: invalid external type"
            )
        message.kind = directive[1:]
        message.external_header = header
        message.underlying_type = cpp_type
        return
    if directive == "@abi" and len(tokens) == 3:
        if message.abi_size is not None or message.abi_alignment is not None:
            raise RuntimeError(
                f"{message.path}:{line_number}: duplicate @abi"
            )
        message.abi_size = parse_positive(
            tokens[1], message.path, line_number, "ABI size"
        )
        message.abi_alignment = parse_positive(
            tokens[2], message.path, line_number, "ABI alignment"
        )
        return
    raise RuntimeError(
        f"{message.path}:{line_number}: unsupported directive {line!r}"
    )


def parse_schema(path: Path) -> Message:
    """解析一个权威 schema，拒绝未知指令、非定宽字段和超出类型范围的常量。"""
    topic = path.stem
    if NAME_RE.fullmatch(topic) is None:
        raise RuntimeError(f"{path}: invalid topic name")
    message = Message(path=path, topic=topic)
    seen_names: set[str] = set()
    for line_number, raw_line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), 1):
        line, comment = split_comment(raw_line)
        if not line:
            continue
        if line.startswith("@"):
            parse_directive(message, line, line_number)
            continue
        if message.kind != "struct":
            raise RuntimeError(
                f"{path}:{line_number}: external topics cannot declare fields"
            )
        match = DECLARATION_RE.fullmatch(line)
        if match is None or match.group("type") not in PRIMITIVES:
            raise RuntimeError(f"{path}:{line_number}: invalid declaration")
        primitive = match.group("type")
        array = match.group("array")
        name = match.group("name")
        value = match.group("value")
        if name in seen_names:
            raise RuntimeError(f"{path}:{line_number}: duplicate name {name}")
        seen_names.add(name)
        if value is not None:
            if (array is not None or CONSTANT_RE.fullmatch(name) is None or
                    primitive not in INTEGER_LIMITS):
                raise RuntimeError(f"{path}:{line_number}: invalid constant")
            message.constants.append(Constant(primitive, name, value, comment))
        else:
            if MEMBER_RE.fullmatch(name) is None:
                raise RuntimeError(f"{path}:{line_number}: invalid field name")
            message.fields.append(Field(primitive, name, array, comment))

    if message.queue_size is None:
        raise RuntimeError(f"{path}: missing @queue")
    if message.kind == "struct":
        if not message.fields:
            raise RuntimeError(f"{path}: struct topic has no fields")
        if message.abi_size is not None or message.abi_alignment is not None:
            raise RuntimeError(f"{path}: @abi is only valid for external types")
    elif message.kind == "external":
        if message.underlying_type != message.public_type:
            raise RuntimeError(
                f"{path}: external topic type must match {message.public_type}"
            )
        if message.abi_size is None or message.abi_alignment is None:
            raise RuntimeError(f"{path}: external topic requires @abi")
        if (message.abi_alignment & (message.abi_alignment - 1)) != 0 or \
                message.abi_size % message.abi_alignment != 0:
            raise RuntimeError(f"{path}: invalid external ABI layout")
    elif message.kind == "alias":
        if message.abi_size is not None or message.abi_alignment is not None:
            raise RuntimeError(f"{path}: alias ABI is inherited from its target")
    return message


def load_messages(schema_dir: Path) -> list[Message]:
    if not schema_dir.is_dir():
        raise RuntimeError(f"schema directory does not exist: {schema_dir}")
    messages = [parse_schema(path) for path in sorted(schema_dir.glob("*.msg"))]
    if not messages:
        raise RuntimeError("no uORB schemas found")
    topics = [message.topic for message in messages]
    if len(topics) != len(set(topics)):
        raise RuntimeError("duplicate uORB topic")
    return messages


def resolve_integer(expression: str, values: dict[str, int]) -> int:
    if INTEGER_RE.fullmatch(expression):
        return int(expression, 10)
    if expression not in values:
        raise RuntimeError(f"unresolved integer constant {expression}")
    return values[expression]


def align_up(value: int, alignment: int) -> int:
    """按 ``(value + alignment - 1) // alignment * alignment`` 向上对齐。"""
    return (value + alignment - 1) // alignment * alignment


def struct_layout(message: Message) -> Layout:
    """复现目标 ABI 的字段 padding、结构对齐与尾部 padding，生成可锁定布局。"""
    constant_values: dict[str, int] = {}
    for constant in message.constants:
        value = resolve_integer(
            constant.expression, constant_values
        )
        minimum, maximum = INTEGER_LIMITS[constant.primitive]
        if value < minimum or value > maximum:
            raise RuntimeError(
                f"{message.path}: constant {constant.name} is out of range"
            )
        constant_values[constant.name] = value
    offset = 0
    maximum_alignment = 1
    fields: list[FieldLayout] = []
    for field in message.fields:
        primitive = PRIMITIVES[field.primitive]
        count = 1 if field.array is None else resolve_integer(
            field.array, constant_values
        )
        if count <= 0:
            raise RuntimeError(f"{message.path}: array {field.name} is empty")
        offset = align_up(offset, primitive.alignment)
        fields.append(FieldLayout(field.name, offset))
        offset += primitive.size * count
        maximum_alignment = max(maximum_alignment, primitive.alignment)
    return Layout(
        size=align_up(offset, maximum_alignment),
        alignment=maximum_alignment,
        fields=tuple(fields),
    )


def resolve_layouts(messages: list[Message]) -> dict[str, Layout]:
    by_type = {message.public_type: message for message in messages}
    layouts: dict[str, Layout] = {}
    for message in messages:
        if message.kind == "struct":
            layouts[message.public_type] = struct_layout(message)
        elif message.kind == "external":
            assert message.abi_size is not None
            assert message.abi_alignment is not None
            layouts[message.public_type] = Layout(
                message.abi_size, message.abi_alignment, ()
            )
    unresolved = [message for message in messages if message.kind == "alias"]
    while unresolved:
        progress = False
        for message in unresolved[:]:
            target = message.underlying_type
            if target in layouts:
                target_message = by_type[target]
                expected_header = f"{target_message.topic}.hpp"
                if message.external_header != expected_header:
                    raise RuntimeError(
                        f"{message.path}: alias target must include "
                        f"{expected_header}"
                    )
                layouts[message.public_type] = layouts[target]
                unresolved.remove(message)
                progress = True
            elif target not in by_type:
                raise RuntimeError(
                    f"{message.path}: alias target {target} is not generated"
                )
        if not progress:
            raise RuntimeError("cyclic uORB type aliases")
    return layouts


def canonical_contract(message: Message, layout: Layout) -> dict[str, object]:
    return {
        "topic": message.topic,
        "cpp_type": message.public_type,
        "metadata_type": message.metadata_type,
        "queue_size": message.queue_size,
        "kind": message.kind,
        "external_header": message.external_header,
        "constants": [
            {
                "type": constant.primitive,
                "name": constant.name,
                "value": constant.expression,
            }
            for constant in message.constants
        ],
        "fields": [
            {
                "type": field.primitive,
                "name": field.name,
                "array": field.array,
            }
            for field in message.fields
        ],
        "size": layout.size,
        "alignment": layout.alignment,
        "offsets": [
            {"name": field.name, "offset": field.offset}
            for field in layout.fields
        ],
    }


def lock_payload(messages: list[Message], layouts: dict[str, Layout]) -> dict:
    topics = []
    for message in messages:
        contract = canonical_contract(message, layouts[message.public_type])
        encoded = json.dumps(
            contract, ensure_ascii=True, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
        topics.append({
            "name": message.topic,
            "cpp_type": message.public_type,
            "metadata_type": message.metadata_type,
            "queue_size": message.queue_size,
            "size": layouts[message.public_type].size,
            "alignment": layouts[message.public_type].alignment,
            "abi_sha256": hashlib.sha256(encoded).hexdigest(),
        })
    return {
        "format_version": 1,
        "maximum_instances": 4,
        "topic_count": len(topics),
        "topics": topics,
    }


def verify_lock(path: Path, expected: dict) -> None:
    """要求 ABI lock 与全部 schema 的 canonical contract 完全一致，禁止静默漂移。"""
    try:
        actual = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"cannot read uORB ABI lock {path}: {error}") from error
    if actual != expected:
        raise RuntimeError(
            "uORB ABI/catalog lock differs from the message schemas; "
            "review the ABI change and update the lock explicitly"
        )


def cpp_constant(constant: Constant) -> str:
    primitive = PRIMITIVES[constant.primitive]
    expression = constant.expression
    if INTEGER_RE.fullmatch(expression) and primitive.unsigned:
        expression += "U"
    return expression


def comment_suffix(comment: str) -> str:
    return f" // {comment}" if comment else ""


def generate_header(message: Message, layout: Layout) -> str:
    lines = [
        "/* SPDX-License-Identifier: BSD-3-Clause */",
        f"/* Generated from Dima/messages/schemas/{message.path.name}. */",
        "#pragma once",
        "",
    ]
    if message.external_header is not None:
        lines.append(f'#include "{message.external_header}"')
    lines.extend([
        '#include "uorb/uORB.hpp"',
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
    ])
    if message.kind == "struct":
        lines.append(f"struct {message.public_type} {{")
        for constant in message.constants:
            primitive = PRIMITIVES[constant.primitive]
            lines.append(
                f"    static constexpr {primitive.cpp} {constant.name} = "
                f"{cpp_constant(constant)};{comment_suffix(constant.comment)}"
            )
        if message.constants and message.fields:
            lines.append("")
        for field in message.fields:
            primitive = PRIMITIVES[field.primitive]
            array = f"[{field.array}]" if field.array is not None else ""
            lines.append(
                f"    {primitive.cpp} {field.name}{array};"
                f"{comment_suffix(field.comment)}"
            )
        lines.extend(["};", ""])
    elif message.kind == "alias":
        lines.extend([
            f"using {message.public_type} = {message.underlying_type};",
            "",
        ])
    lines.extend([
        f"static_assert(sizeof({message.public_type}) == {layout.size}U);",
        f"static_assert(alignof({message.public_type}) == {layout.alignment}U);",
    ])
    for field in layout.fields:
        lines.append(
            f"static_assert(offsetof({message.public_type}, {field.name}) == "
            f"{field.offset}U);"
        )
    lines.extend(["", f"ORB_DECLARE({message.topic});", ""])
    return "\n".join(lines)


def generate_catalog(messages: list[Message]) -> str:
    lines = [
        "/* SPDX-License-Identifier: BSD-3-Clause */",
        "/* Generated uORB metadata catalog. DO NOT EDIT. */",
    ]
    for message in messages:
        lines.append(f'#include "{message.topic}.hpp"')
    lines.append("")
    for message in messages:
        lines.append(
            f"ORB_DEFINE({message.topic}, {message.metadata_type}, "
            f"{message.queue_size}U);"
        )
    lines.append("")
    return "\n".join(lines)


def write_if_changed(path: Path, text: str) -> None:
    encoded = text.encode("utf-8")
    try:
        if path.read_bytes() == encoded:
            return
    except FileNotFoundError:
        pass
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=path.name + ".", suffix=".tmp",
            dir=path.parent, delete=False,
        ) as temporary:
            temporary.write(encoded)
            temporary_name = temporary.name
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)


def main() -> int:
    args = parse_args()
    messages = load_messages(args.schemas)
    layouts = resolve_layouts(messages)
    expected_lock = lock_payload(messages, layouts)
    if args.print_lock:
        print(json.dumps(expected_lock, indent=2, ensure_ascii=False))
        return 0
    assert args.lock is not None
    verify_lock(args.lock, expected_lock)
    if args.verify_only:
        print(f"verified {len(messages)} locked uORB message schemas")
        return 0
    assert args.output is not None
    for message in messages:
        header = args.output / f"{message.topic}.hpp"
        write_if_changed(header, generate_header(
            message, layouts[message.public_type]
        ))
    write_if_changed(args.output / "uorb_topics.cpp", generate_catalog(messages))
    write_if_changed(
        args.output / "uorb_catalog.json",
        json.dumps(expected_lock, indent=2, ensure_ascii=False) + "\n",
    )
    print(f"generated {len(messages)} locked uORB message contracts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
