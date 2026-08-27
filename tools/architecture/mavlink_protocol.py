"""MAVLink XML/lock 输入与生成目录一致性门禁。"""

from __future__ import annotations

import hashlib
import json
import pathlib

from architecture.common import ROOT, Violation


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: pathlib.Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _catalog_entries_match(
    declared: list[dict], generated: object,
) -> bool:
    """生成项可以增加 ID/规范化默认值，但必须完整保留 lock 的权威字段。"""
    if not isinstance(generated, list) or len(generated) != len(declared):
        return False
    for source, output in zip(declared, generated):
        if not isinstance(output, dict):
            return False
        if any(output.get(key) != value for key, value in source.items()):
            return False
    return True


def scan_mavlink_protocol_contract(violations: list[Violation]) -> None:
    """只核对 XML、lock、生成 catalog 和逐文件哈希，不冻结业务消费者。"""
    lock_path = ROOT / "tools/mavlink/mavlink.lock.json"
    try:
        lock = _load_json(lock_path)
        mavlink = lock["mavlink"]
        pymavlink = lock["pymavlink"]
        dialect = lock["dialect"]
        runtime = lock["runtime"]
        messages_from_common = dialect["messages_from_common"]
        inherited_messages = dialect["inherited_messages"]
        messages = [*messages_from_common, *inherited_messages]
        forbidden = dialect["forbidden_messages"]
        runtime_messages = runtime["messages"]
        inbound_messages = runtime["inbound"]
        runtime_names = [entry["name"] for entry in runtime_messages]
        inbound_names = [entry["name"] for entry in inbound_messages]
        xml_hashes = mavlink["xml_sha256"]
        if (not isinstance(dialect["message_count"], int) or
                dialect["message_count"] != len(messages) or
                len(messages) != len(set(messages)) or
                not isinstance(forbidden, list) or
                len(forbidden) != len(set(forbidden)) or
                set(messages) & set(forbidden) or
                len(runtime_names) != len(set(runtime_names)) or
                len(inbound_names) != len(set(inbound_names)) or
                not set(runtime_names + inbound_names).issubset(messages) or
                not isinstance(xml_hashes, dict) or not xml_hashes):
            raise ValueError("MAVLink lock lists are internally inconsistent")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        violations.append(Violation(
            lock_path, 1, "R334", f"invalid MAVLink lock contract: {error}",
        ))
        return

    xml_root = ROOT / "tools/mavlink/message_definitions"
    for name, expected_hash in xml_hashes.items():
        xml_path = xml_root / name
        if (not isinstance(name, str) or
                not isinstance(expected_hash, str) or
                not xml_path.is_file() or _sha256(xml_path) != expected_hash):
            violations.append(Violation(
                xml_path, 1, "R334",
                "MAVLink XML input is missing or differs from its lock hash",
            ))

    generated_root = ROOT / "build/generated/mavlink"
    catalog_path = generated_root / ".generated.json"
    try:
        catalog = _load_json(catalog_path)
    except (OSError, json.JSONDecodeError) as error:
        violations.append(Violation(
            catalog_path, 1, "R334",
            f"cannot read generated MAVLink catalog: {error}",
        ))
        return

    catalog_matches_lock = (
        catalog.get("dialect") == dialect.get("name") and
        catalog.get("message_count") == dialect.get("message_count") and
        catalog.get("mavlink_commit") == mavlink.get("commit") and
        catalog.get("pymavlink_commit") == pymavlink.get("commit") and
        catalog.get("pymavlink_version") == pymavlink.get("version") and
        catalog.get("xml_sha256") == xml_hashes and
        _catalog_entries_match(runtime_messages,
                               catalog.get("runtime_messages")) and
        _catalog_entries_match(inbound_messages,
                               catalog.get("runtime_inbound"))
    )
    if not catalog_matches_lock:
        violations.append(Violation(
            catalog_path, 1, "R334",
            "generated MAVLink catalog differs from mavlink.lock.json",
        ))

    expected_hashes = catalog.get("output_sha256")
    if not isinstance(expected_hashes, dict):
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
