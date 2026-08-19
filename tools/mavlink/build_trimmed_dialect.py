#!/usr/bin/env python3
"""Generate the deterministic 24-message Dima MAVLink v2 dialect."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True

from bootstrap_pymavlink import (
    BootstrapError,
    file_sha256,
    load_lock,
    provision_pymavlink,
)


MESSAGES_FROM_COMMON = {
    "PING",
    "PARAM_REQUEST_READ",
    "PARAM_REQUEST_LIST",
    "PARAM_VALUE",
    "PARAM_SET",
    "COMMAND_LONG",
    "COMMAND_ACK",
    "COMMAND_INT",
    "FILE_TRANSFER_PROTOCOL",
    "MISSION_REQUEST_LIST",
    "MISSION_COUNT",
    "MISSION_CLEAR_ALL",
    "MISSION_ACK",
    "RC_CHANNELS",
    "TIMESYNC",
    "STATUSTEXT",
    "PARAM_EXT_REQUEST_READ",
    "PARAM_EXT_VALUE",
    "COMPONENT_INFORMATION",
    "COMPONENT_METADATA",
}

INHERITED_MESSAGES = {
    "AUTOPILOT_VERSION",
    "GLOBAL_POSITION_INT",
    "HEARTBEAT",
    "PROTOCOL_VERSION",
}

ENUMS_FROM_COMMON_FULL = {
    "MAV_RESULT",
    "MAV_SEVERITY",
    "MAV_PARAM_TYPE",
    "MAV_FRAME",
    "MAV_MISSION_RESULT",
    "MAV_MISSION_TYPE",
}

MAV_CMD_ENTRIES = {
    "MAV_CMD_PREFLIGHT_CALIBRATION",
    "MAV_CMD_DO_SET_MODE",
    "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
    "MAV_CMD_COMPONENT_ARM_DISARM",
    "MAV_CMD_REQUEST_MESSAGE",
    "MAV_CMD_SET_MESSAGE_INTERVAL",
    "MAV_CMD_GET_MESSAGE_INTERVAL",
    "MAV_CMD_REQUEST_PROTOCOL_VERSION",
    "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES",
}


class GenerationError(RuntimeError):
    """A deterministic dialect generation or validation failure."""


def install_generated_tree(source: pathlib.Path,
                           destination: pathlib.Path) -> None:
    """Atomically install a generated tree, tolerating short Windows locks."""
    attempts = 5 if os.name == "nt" else 1
    for attempt in range(attempts):
        try:
            os.replace(source, destination)
            return
        except PermissionError as error:
            if attempt + 1 >= attempts:
                raise GenerationError(
                    "unable to install generated MAVLink tree after "
                    f"{attempts} attempts: {error}"
                ) from error
            time.sleep(0.05 * (attempt + 1))


def extract(source_path: pathlib.Path) -> tuple[list[ET.Element], list[ET.Element]]:
    root = ET.parse(source_path).getroot()
    messages = [
        copy.deepcopy(message)
        for message in root.findall("./messages/message")
        if message.get("name") in MESSAGES_FROM_COMMON
    ]

    enums: list[ET.Element] = []
    for enum in root.findall("./enums/enum"):
        name = enum.get("name")
        if name in ENUMS_FROM_COMMON_FULL:
            enums.append(copy.deepcopy(enum))
        elif name == "MAV_CMD":
            filtered = ET.Element("enum", enum.attrib)
            description = enum.find("description")
            if description is not None:
                filtered.append(copy.deepcopy(description))
            for entry in enum.findall("entry"):
                if entry.get("name") in MAV_CMD_ENTRIES:
                    filtered.append(copy.deepcopy(entry))
            enums.append(filtered)
    return messages, enums


def indent(element: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = pad + "  "
        for child in element:
            indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = pad + "  "
        element[-1].tail = pad


def validate_inputs(xml_dir: pathlib.Path, lock: dict) -> None:
    expected = lock["mavlink"]["xml_sha256"]
    for filename, expected_sha in expected.items():
        path = xml_dir / filename
        if not path.is_file():
            raise GenerationError(f"missing pinned MAVLink XML: {path}")
        actual_sha = file_sha256(path)
        if actual_sha != expected_sha:
            raise GenerationError(
                f"{filename} SHA-256 mismatch: expected {expected_sha}, got {actual_sha}"
            )


def source_message_names(path: pathlib.Path) -> set[str]:
    return {
        message.get("name", "")
        for message in ET.parse(path).getroot().findall("./messages/message")
    }


def validate_allowlist(xml_dir: pathlib.Path, lock: dict) -> None:
    common_names = source_message_names(xml_dir / "common.xml")
    missing = MESSAGES_FROM_COMMON - common_names
    if missing:
        raise GenerationError(f"messages missing from common.xml: {sorted(missing)}")

    inherited = (
        source_message_names(xml_dir / "standard.xml")
        | source_message_names(xml_dir / "minimal.xml")
    )
    if inherited != INHERITED_MESSAGES:
        raise GenerationError(
            "pinned standard/minimal inherited message set changed: "
            f"expected {sorted(INHERITED_MESSAGES)}, got {sorted(inherited)}"
        )

    complete = MESSAGES_FROM_COMMON | inherited
    expected_count = lock["dialect"]["message_count"]
    if len(complete) != expected_count:
        raise GenerationError(
            f"dialect has {len(complete)} messages, expected {expected_count}"
        )
    forbidden = set(lock["dialect"]["forbidden_messages"])
    present_forbidden = complete & forbidden
    if present_forbidden:
        raise GenerationError(
            f"forbidden messages entered dialect: {sorted(present_forbidden)}"
        )


def write_dialect(xml_dir: pathlib.Path, output_path: pathlib.Path) -> None:
    messages, enums = extract(xml_dir / "common.xml")
    root = ET.Element("mavlink")
    ET.SubElement(root, "include").text = "standard.xml"
    ET.SubElement(root, "version").text = "0"
    ET.SubElement(root, "dialect").text = "0"
    enums_node = ET.SubElement(root, "enums")
    for enum in enums:
        enums_node.append(enum)
    messages_node = ET.SubElement(root, "messages")
    for message in sorted(messages, key=lambda item: int(item.get("id", "0"))):
        messages_node.append(message)
    indent(root)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    content = output_path.read_bytes()
    output_path.write_bytes(content.replace(b"\r\n", b"\n").replace(b"\r", b"\n"))


def hash_tree(root: pathlib.Path) -> dict[str, str]:
    hashes: dict[str, str] = {}
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        hashes[path.relative_to(root).as_posix()] = file_sha256(path)
    return hashes


def normalize_xml_hashes(
    generated: pathlib.Path,
    dialect_path: pathlib.Path,
    xml_dir: pathlib.Path,
) -> None:
    """Replace Python-version-dependent mavgen XML hashes with stable values."""
    sources = {
        "dima": dialect_path,
        "standard": xml_dir / "standard.xml",
        "minimal": xml_dir / "minimal.xml",
    }
    for dialect, source in sources.items():
        stable_hash = int.from_bytes(
            hashlib.sha256(source.read_bytes()).digest()[:8], "big"
        ) & ((1 << 63) - 1)
        replacements = (
            (
                generated / dialect / f"{dialect}.h",
                rb"(#define MAVLINK_" + dialect.upper().encode()
                + rb"_XML_HASH )-?[0-9]+",
            ),
            (
                generated / dialect / "mavlink.h",
                rb"(#define MAVLINK_PRIMARY_XML_HASH )-?[0-9]+",
            ),
        )
        for path, pattern in replacements:
            original = path.read_bytes()
            updated, count = re.subn(
                pattern, rb"\g<1>" + str(stable_hash).encode(), original, count=1
            )
            if count != 1:
                raise GenerationError(f"unable to normalize XML hash in {path}")
            path.write_bytes(updated)


def normalize_generated_line_endings(generated: pathlib.Path) -> None:
    for path in (item for item in generated.rglob("*") if item.is_file()):
        content = path.read_bytes()
        normalized = content.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
        if normalized != content:
            path.write_bytes(normalized)


def generate(arguments: argparse.Namespace) -> None:
    lock = load_lock(arguments.lock)
    validate_inputs(arguments.xml_dir, lock)
    validate_allowlist(arguments.xml_dir, lock)

    pymavlink_root = provision_pymavlink(
        arguments.cache_root.expanduser(), arguments.lock, arguments.pymavlink_root
    )
    sys.path.insert(0, str(pymavlink_root.parent))
    try:
        import pymavlink
        from pymavlink.generator import mavgen
    except ImportError as error:
        raise GenerationError(f"unable to import pinned pymavlink: {error}") from error
    imported_root = pathlib.Path(pymavlink.__file__).resolve().parent
    if imported_root != pymavlink_root.resolve() or pymavlink.__version__ != lock["pymavlink"]["version"]:
        raise GenerationError(
            "imported pymavlink does not match the pinned cache: "
            f"loaded {pymavlink.__version__} from {imported_root}"
        )

    output_dir = arguments.output_dir
    work_dir = output_dir.with_name(output_dir.name + ".work")
    if work_dir.exists():
        shutil.rmtree(work_dir)
    definitions = work_dir / "definitions"
    generated = work_dir / "generated"
    definitions.mkdir(parents=True)
    generated.mkdir()
    for filename in ("minimal.xml", "standard.xml"):
        shutil.copyfile(arguments.xml_dir / filename, definitions / filename)
    dialect_path = definitions / "dima.xml"
    write_dialect(arguments.xml_dir, dialect_path)

    class Options:
        language = "C"
        wire_protocol = "2.0"
        output = str(generated)
        validate = False
        error_limit = 200
        strict_types = False
        strict_units = False

    if not mavgen.mavgen(Options(), [str(dialect_path)]):
        raise GenerationError("pinned mavgen reported failure")
    required_header = generated / "dima" / "mavlink.h"
    if not required_header.is_file():
        raise GenerationError(f"mavgen did not create {required_header}")

    normalize_generated_line_endings(generated)
    normalize_xml_hashes(generated, dialect_path, arguments.xml_dir)
    shutil.copyfile(dialect_path, generated / "dima.xml")
    manifest = {
        "dialect": "dima",
        "message_count": lock["dialect"]["message_count"],
        "mavlink_commit": lock["mavlink"]["commit"],
        "pymavlink_commit": lock["pymavlink"]["commit"],
        "pymavlink_version": lock["pymavlink"]["version"],
        "xml_sha256": lock["mavlink"]["xml_sha256"],
        "output_sha256": hash_tree(generated),
    }
    (generated / ".generated.json").write_bytes(
        (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    )

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    if output_dir.exists():
        shutil.rmtree(output_dir)
    install_generated_tree(generated, output_dir)
    shutil.rmtree(work_dir)
    print(
        f"generated {lock['dialect']['message_count']}-message Dima MAVLink "
        f"library with pymavlink {lock['pymavlink']['version']} into {output_dir}"
    )


def main() -> int:
    script_dir = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml-dir", type=pathlib.Path, required=True)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    parser.add_argument(
        "--lock", type=pathlib.Path, default=script_dir / "mavlink.lock.json"
    )
    parser.add_argument(
        "--cache-root",
        type=pathlib.Path,
        default=pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools",
    )
    parser.add_argument("--pymavlink-root", type=pathlib.Path)
    arguments = parser.parse_args()
    generate(arguments)
    return 0


if __name__ == "__main__":
    if os.environ.get("PYTHONHASHSEED") != "0":
        environment = os.environ.copy()
        environment["PYTHONHASHSEED"] = "0"
        completed = subprocess.run(
            [sys.executable, __file__, *sys.argv[1:]], env=environment, check=False
        )
        raise SystemExit(completed.returncode)
    try:
        raise SystemExit(main())
    except (BootstrapError, GenerationError) as error:
        print(f"MAVLink generation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
