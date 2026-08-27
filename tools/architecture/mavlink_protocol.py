"""MAVLink native dialect, pinned sources and generated-closure gate."""

from __future__ import annotations

import hashlib
import json
import pathlib
import re
import xml.etree.ElementTree as ET

import yaml

from architecture.common import ROOT, Violation, line_for, sources_under


PINNED_MAVLINK_COMMIT = "33af200d25ec6f0925b49b1ba82bbf1294ea5f72"
PINNED_PYMAVLINK_COMMIT = "fcaa2c7d25e3169dc66155929c338487941555e9"
PINNED_PYMAVLINK_VERSION = "2.4.47"
WIRE_DEFINITION_NAMES = {"dima.xml", "common.xml", "standard.xml", "minimal.xml"}


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: pathlib.Path) -> dict:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise TypeError("root is not an object")
    return document


def _load_policy(path: pathlib.Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("version") != 1:
        raise TypeError("runtime policy is not schema version 1")
    outbound = document.get("outbound")
    inbound = document.get("inbound")
    if not isinstance(outbound, list) or not outbound:
        raise TypeError("outbound is not a non-empty list")
    if not isinstance(inbound, list) or not inbound:
        raise TypeError("inbound is not a non-empty list")
    for label, entries in (("outbound", outbound), ("inbound", inbound)):
        names = [entry.get("message") for entry in entries if isinstance(entry, dict)]
        if (
            len(names) != len(entries)
            or any(not isinstance(name, str) or not name for name in names)
            or len(names) != len(set(names))
        ):
            raise TypeError(f"{label} message symbols are invalid or duplicated")
    return document


def _message_names(root: ET.Element) -> set[str]:
    return {
        message.attrib["name"]
        for message in root.findall("./messages/message")
        if "name" in message.attrib
    }


def _scan_dialect_root(
    xml_root: pathlib.Path, violations: list[Violation]
) -> None:
    dialect_path = xml_root / "dima.xml"
    actual_xml = {path.name for path in xml_root.glob("*.xml") if path.is_file()}
    if actual_xml != WIRE_DEFINITION_NAMES:
        violations.append(Violation(
            xml_root, 1, "R334",
            "MAVLink wire inputs must be exactly dima/common/standard/minimal XML",
        ))

    try:
        dialect = ET.parse(dialect_path).getroot()
        includes = [
            (element.text or "").strip()
            for element in dialect.findall("./include")
        ]
        if includes != ["common.xml"]:
            raise ValueError("dima.xml must include only common.xml")
        if dialect.findtext("./version") != "3":
            raise ValueError("dima.xml must declare MAVLink version 3")

        upstream_names: set[str] = set()
        for name in ("common.xml", "standard.xml", "minimal.xml"):
            upstream_names |= _message_names(ET.parse(xml_root / name).getroot())
        duplicate_product_names = _message_names(dialect) & upstream_names
        if duplicate_product_names:
            raise ValueError(
                "product dialect duplicates upstream messages: "
                f"{sorted(duplicate_product_names)}"
            )
    except (OSError, ET.ParseError, ValueError) as error:
        violations.append(Violation(
            dialect_path, 1, "R334", f"invalid dima.xml wire root: {error}",
        ))


def _scan_make_wiring(violations: list[Violation]) -> None:
    path = ROOT / "make/project.mk"
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        violations.append(Violation(path, 1, "R334", f"cannot read Make wiring: {error}"))
        return

    required = (
        "MAVLINK_DIALECT := $(MAVLINK_XML_DIR)/dima.xml",
        "MAVLINK_RUNTIME_POLICY := Dima/modules/mavlink/mavlink_runtime.yaml",
        "MAVLINK_RUNTIME_GENERATOR := tools/mavlink/generate_runtime_contract.py",
        '"$$pymavlink_root/tools/mavgen.py"',
        "--lang C --wire-protocol 2.0",
        "--no-validate",
        "--policy $(MAVLINK_RUNTIME_POLICY)",
        "mavlink-generated-verify:",
    )
    for literal in required:
        if literal not in text:
            violations.append(Violation(
                path, 1, "R334", f"native MAVLink build wiring is missing: {literal}",
            ))

    forbidden = re.search(
        r"build_trimmed_dialect|normalize_xml_hashes|write_dialect\s*\(", text
    )
    if forbidden:
        violations.append(Violation(
            path, line_for(text, forbidden.group(0)), "R334",
            "Make must not trim or rewrite the mavgen wire output",
        ))


def _scan_generated_closure(
    lock_path: pathlib.Path,
    lock: dict,
    policy_path: pathlib.Path,
    policy: dict,
    violations: list[Violation],
) -> None:
    generated_root = ROOT / "build/generated/mavlink"
    catalog_path = generated_root / ".generated.json"
    try:
        catalog = _load_json(catalog_path)
    except (OSError, TypeError, json.JSONDecodeError) as error:
        violations.append(Violation(
            catalog_path, 1, "R334", f"cannot read generated MAVLink catalog: {error}",
        ))
        return

    expected_inputs = {
        ROOT / "tools/mavlink/message_definitions/dima.xml",
        ROOT / "tools/mavlink/message_definitions/common.xml",
        ROOT / "tools/mavlink/message_definitions/standard.xml",
        ROOT / "tools/mavlink/message_definitions/minimal.xml",
        policy_path,
        lock_path,
        ROOT / "tools/mavlink/generate_runtime_contract.py",
        ROOT / "tools/mavlink/bootstrap_pymavlink.py",
    }
    manifest_inputs = catalog.get("inputs")
    expected_input_hashes = {
        path.relative_to(ROOT).as_posix(): _sha256(path) for path in expected_inputs
    }
    if manifest_inputs != expected_input_hashes:
        violations.append(Violation(
            catalog_path, 1, "R334",
            "generated MAVLink input hash closure is stale or incomplete",
        ))

    outbound = policy["outbound"]
    inbound = policy["inbound"]
    identity_matches = (
        catalog.get("dialect") == "dima"
        and catalog.get("mavlink_commit") == lock["mavlink"]["commit"]
        and catalog.get("pymavlink_commit") == lock["pymavlink"]["commit"]
        and catalog.get("pymavlink_version") == lock["pymavlink"]["version"]
        and catalog.get("runtime_outbound_count") == len(outbound)
        and catalog.get("runtime_inbound_count") == len(inbound)
        and isinstance(catalog.get("dialect_message_count"), int)
        and catalog["dialect_message_count"] > 0
    )
    if not identity_matches:
        violations.append(Violation(
            catalog_path, 1, "R334",
            "generated MAVLink identity/counts differ from source inputs",
        ))

    expected_hashes = catalog.get("outputs")
    if not isinstance(expected_hashes, dict) or not expected_hashes:
        violations.append(Violation(
            catalog_path, 1, "R334",
            "generated MAVLink catalog lacks its output hash closure",
        ))
        return
    actual_paths = {
        path.relative_to(generated_root).as_posix(): path
        for path in generated_root.rglob("*")
        if path.is_file() and path != catalog_path
    }
    if set(actual_paths) != set(expected_hashes):
        violations.append(Violation(
            catalog_path, 1, "R334",
            "generated MAVLink file closure differs from its catalog",
        ))
        return
    changed = [
        relative for relative, path in actual_paths.items()
        if _sha256(path) != expected_hashes[relative]
    ]
    if changed:
        violations.append(Violation(
            generated_root / changed[0], 1, "R334",
            f"generated MAVLink output hash mismatch: {changed}",
        ))

    contract_path = generated_root / "mavlink_stream_contract.hpp"
    try:
        contract = contract_path.read_text(encoding="utf-8")
    except OSError as error:
        violations.append(Violation(
            contract_path, 1, "R334", f"cannot read runtime contract: {error}",
        ))
        return
    if "MAVLINK_MSG_ID_" not in contract:
        violations.append(Violation(
            contract_path, 1, "R334",
            "runtime contract must reference IDs from mavgen macros",
        ))
    copied_numeric_id = re.search(
        r"\{\s*[0-9]+U?\s*,\s*(?:MessageHandler|InboundHandler)::", contract
    )
    if copied_numeric_id:
        violations.append(Violation(
            contract_path, line_for(contract, copied_numeric_id.group(0)), "R334",
            "runtime contract must not copy numeric MAVLink IDs",
        ))


def _scan_handwritten_wire_code(violations: list[Violation]) -> None:
    legacy_generator = ROOT / "tools/mavlink/build_trimmed_dialect.py"
    if legacy_generator.exists():
        violations.append(Violation(
            legacy_generator, 1, "R334",
            "local MAVLink trimming/codec generator must be retired",
        ))

    forbidden_definition = re.compile(
        r"^\s*#\s*define\s+MAVLINK_MSG_ID_[A-Z0-9_]+\s+[0-9]+|"
        r"\bMAVLINK_MESSAGE_CRCS\b",
        re.MULTILINE,
    )
    for path in sources_under(("Dima", "Boards")):
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        match = forbidden_definition.search(text)
        if match:
            violations.append(Violation(
                path, line_for(text, match.group(0)), "R334",
                "first-party source must not define MAVLink wire IDs or CRC tables",
            ))


def scan_mavlink_protocol_contract(violations: list[Violation]) -> None:
    """核对 dima.xml + 原始 mavgen.py + YAML 策略的单向生成闭包。"""
    lock_path = ROOT / "tools/mavlink/mavlink.lock.json"
    policy_path = ROOT / "Dima/modules/mavlink/mavlink_runtime.yaml"
    try:
        lock = _load_json(lock_path)
        policy = _load_policy(policy_path)
        mavlink = lock["mavlink"]
        pymavlink = lock["pymavlink"]
        xml_hashes = mavlink["xml_sha256"]
        archive = pymavlink["archive"]
        if (
            lock.get("format") != 1
            or set(lock) != {"format", "mavlink", "pymavlink"}
            or mavlink.get("commit") != PINNED_MAVLINK_COMMIT
            or pymavlink.get("commit") != PINNED_PYMAVLINK_COMMIT
            or pymavlink.get("version") != PINNED_PYMAVLINK_VERSION
            or not re.fullmatch(r"[0-9a-f]{64}", pymavlink.get("source_tree_sha256", ""))
            or not isinstance(xml_hashes, dict)
            or set(xml_hashes) != {"common.xml", "standard.xml", "minimal.xml"}
            or not re.fullmatch(r"[0-9a-f]{64}", archive.get("sha256", ""))
        ):
            raise ValueError("source pins or lock shape differ from the native contract")
    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
        yaml.YAMLError,
    ) as error:
        violations.append(Violation(
            lock_path, 1, "R334", f"invalid MAVLink source lock/policy: {error}",
        ))
        return

    xml_root = ROOT / "tools/mavlink/message_definitions"
    for name, expected_hash in xml_hashes.items():
        path = xml_root / name
        if (
            not isinstance(expected_hash, str)
            or not path.is_file()
            or _sha256(path) != expected_hash
        ):
            violations.append(Violation(
                path, 1, "R334",
                "upstream MAVLink XML is missing or differs from its pin",
            ))

    _scan_dialect_root(xml_root, violations)
    _scan_make_wiring(violations)
    _scan_generated_closure(lock_path, lock, policy_path, policy, violations)
    _scan_handwritten_wire_code(violations)
