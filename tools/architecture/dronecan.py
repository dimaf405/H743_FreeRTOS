"""生成 DroneCAN/DSDL 合同、PX4 动态节点 ID 与设备身份布局门禁。"""

from __future__ import annotations

import hashlib
import json
import pathlib
import re

from architecture.common import (
    ROOT,
    Violation,
    line_for,
    repository_files_named,
    require_literals,
)


EXPECTED_PX4_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
EXPECTED_DSDL_COMMIT = "993be80a62ec957c01fb41115b83663959a49f46"
EXPECTED_GENERATOR_COMMIT = "431170fa4bfe2212b516b8f33bdc796267907f1c"


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: pathlib.Path, violations: list[Violation], label: str):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        violations.append(Violation(
            path, 1, "R349", f"cannot read {label}: {error}",
        ))
        return None


def _unique_dima_file(
    filename: str,
    violations: list[Violation],
    label: str,
) -> pathlib.Path | None:
    """按唯一文件身份定位 owner；目录位置不参与 DroneCAN 合同。"""
    dima_root = ROOT / "Dima"
    matches = tuple(
        path for path in repository_files_named(filename)
        if path.is_relative_to(dima_root)
    )
    if len(matches) == 1:
        return matches[0]
    violations.append(Violation(
        dima_root, 1, "R349",
        f"expected one {label} named {filename}, found {len(matches)}",
    ))
    return None


def _scan_generated_catalog(
    manifest_path: pathlib.Path,
    manifest: dict,
    violations: list[Violation],
) -> None:
    """比对 manifest pin、输出文件闭包和逐文件 SHA-256，拒绝陈旧或手改产物。"""
    generated = ROOT / "build/generated/dronecan"
    catalog_path = generated / ".generated.json"
    catalog = _load_json(catalog_path, violations, "generated DroneCAN catalog")
    if catalog is None:
        return
    if (catalog.get("contract") != manifest.get("contract") or
            catalog.get("manifest_sha256") != _sha256(manifest_path) or
            catalog.get("upstream") != manifest.get("upstream")):
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN catalog is stale or uses different pins",
        ))
    expected_hashes = catalog.get("output_sha256")
    if not isinstance(expected_hashes, dict):
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN catalog lacks its output hash closure",
        ))
        return
    actual_paths = {
        path.relative_to(generated).as_posix(): path
        for path in generated.rglob("*")
        if path.is_file() and path != catalog_path
    }
    if set(actual_paths) != set(expected_hashes):
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN file closure differs from its catalog",
        ))
        return
    changed = [
        relative for relative, path in actual_paths.items()
        if _sha256(path) != expected_hashes[relative]
    ]
    if changed:
        violations.append(Violation(
            generated / changed[0], 1, "R349",
            f"generated DroneCAN output hash mismatch: {changed}",
        ))
    messages = catalog.get("messages")
    roles = {
        message.get("role") for message in messages
        if isinstance(message, dict)
    } if isinstance(messages, list) else set()
    manifest_files = manifest.get("dsdl", {}).get("files", [])
    manifest_roles = {
        item.get("role") for item in manifest_files
        if isinstance(item, dict) and "role" in item
    } if isinstance(manifest_files, list) else set()
    if roles != manifest_roles:
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated subscription roles differ from the manifest",
        ))
    parameters = catalog.get("parameters")
    manifest_parameters = manifest.get("parameters", [])
    manifest_parameter_names = [
        parameter.get("name") for parameter in manifest_parameters
        if isinstance(parameter, dict)
    ] if isinstance(manifest_parameters, list) else []
    if not isinstance(parameters, list) or parameters != manifest_parameter_names:
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN parameter binding differs from the manifest",
        ))


def scan_dronecan_contract(violations: list[Violation]) -> None:
    """核对唯一 manifest、生成 role/参数集合、DNA 状态机与设备身份合同。"""
    manifest_path = _unique_dima_file(
        "dronecan_contract.json", violations, "DroneCAN manifest"
    )
    if manifest_path is None:
        return
    manifest = _load_json(manifest_path, violations, "DroneCAN manifest")
    if manifest is None:
        return
    upstream = manifest.get("upstream", {})
    commits = {
        "px4": EXPECTED_PX4_COMMIT,
        "dsdl": EXPECTED_DSDL_COMMIT,
        "generator": EXPECTED_GENERATOR_COMMIT,
    }
    for owner, expected in commits.items():
        actual = upstream.get(owner, {}).get("commit")
        if actual != expected:
            violations.append(Violation(
                manifest_path, 1, "R349",
                f"DroneCAN {owner} pin must remain {expected}",
            ))

    dsdl = manifest.get("dsdl", {})
    namespace = ROOT / str(dsdl.get("namespace_root", ""))
    files = dsdl.get("files")
    if not isinstance(files, list):
        files = []
    roles = {
        item.get("role") for item in files
        if isinstance(item, dict) and "role" in item
    }
    if not roles or "allocation" not in roles or len(roles) != sum(
        1 for item in files if isinstance(item, dict) and "role" in item
    ):
        violations.append(Violation(
            manifest_path, 1, "R349",
            "DroneCAN roles must be unique and contain the allocation role",
        ))
    listed_paths: set[pathlib.Path] = set()
    for item in files:
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            continue
        source = namespace / item["path"]
        listed_paths.add(source)
        if not source.is_file() or _sha256(source) != item.get("sha256"):
            violations.append(Violation(
                source, 1, "R349",
                "vendored DSDL input is missing or differs from its pin",
            ))
    actual_paths = {
        path for path in namespace.rglob("*.uavcan") if path.is_file()
    } if namespace.is_dir() else set()
    if actual_paths != listed_paths:
        violations.append(Violation(
            namespace, 1, "R349",
            "vendored DSDL inputs must be exactly the manifest closure",
        ))

    parameters = manifest.get("parameters")
    parameter_names = [
        parameter.get("name") for parameter in parameters
        if isinstance(parameter, dict)
    ] if isinstance(parameters, list) else []
    parameter_roles = [
        parameter.get("role") for parameter in parameters
        if isinstance(parameter, dict)
    ] if isinstance(parameters, list) else []
    if (not parameter_names or len(parameter_names) != len(set(parameter_names)) or
            len(parameter_roles) != len(parameter_names) or
            any(not isinstance(role, str) or not role for role in parameter_roles) or
            len(parameter_roles) != len(set(parameter_roles))):
        violations.append(Violation(
            manifest_path, 1, "R349",
            "DroneCAN parameter names and roles must be non-empty and unique",
        ))
    enable = next(
        (parameter for parameter in parameters
         if isinstance(parameter, dict)
         and parameter.get("role") == "mode"),
        {},
    ) if isinstance(parameters, list) else {}
    symbols = {
        value.get("symbol"): value.get("value")
        for value in enable.get("values", [])
        if isinstance(value, dict) and "symbol" in value
    }
    if enable.get("default") != 2 or symbols != {
            "disabled": 0, "manual": 1, "automatic": 2}:
        violations.append(Violation(
            manifest_path, 1, "R349",
            "UAVCAN1 mode 2 must enable automatic Node ID allocation",
        ))
    device = manifest.get("px4_device_id", {})
    if (device.get("layout") != "DeviceStructure" or
            device.get("bus_type", {}).get("value") != 3 or
            device.get("bus", {}).get("value") != 0 or
            device.get("address", {}).get("source") != "source_node_id" or
            device.get("devtype", {}).get("value") != 0x88 or
            device.get("sensor_id_policy") != "ignored"):
        violations.append(Violation(
            manifest_path, 1, "R349",
            "magnetometer device ID must match PX4 UAVCAN DeviceStructure",
        ))

    project_path = ROOT / "make/project.mk"
    require_literals(
        project_path,
        (
            ("DRONECAN_CONTRACT_GENERATOR := tools/dronecan/generate_contract.py",
             "R349", "build must invoke the DroneCAN generator"),
            ("--print-runtime-sources", "R349",
             "first-party DroneCAN sources must be discovered by the generator"),
            ("include $(DRONECAN_GENERATED_MAKEFILE)", "R349",
             "build must consume the generated source catalog"),
            ("$(DIMA_DRONECAN_GENERATED_C_SOURCES)", "R349",
             "DSDL C sources must come from the generated catalog"),
            ("$(DIMA_DRONECAN_PARAMETER_SOURCE)", "R349",
             "DroneCAN parameters must come from the generated catalog"),
            ("$(sort $(wildcard $(PARAMETER_DEFINITION_DIR)/*.c))", "R349",
             "parameter input discovery must not be a handwritten list"),
        ),
        violations,
    )
    project_text = project_path.read_text(encoding="utf-8")
    forbidden_make = re.search(
        r"Middlewares/Third_Party/dronecan_dsdl/(?:include|src)|"
        r"uavcan\.(?:protocol|equipment)\.[^\s\\]+\.[ch]",
        project_text,
    )
    if forbidden_make:
        violations.append(Violation(
            project_path, line_for(project_text, forbidden_make.group(0)),
            "R349", "Make must not handwrite DroneCAN message files",
        ))

    configuration_path = _unique_dima_file(
        "DroneCanMag2Configuration.cpp", violations,
        "DroneCAN configuration owner",
    )
    if configuration_path is None:
        return
    configuration = configuration_path.read_text(encoding="utf-8")
    if re.search(r"constexpr\s+px4::params\s+kParameters", configuration):
        violations.append(Violation(
            configuration_path, 1, "R349",
            "DroneCAN consumers must use the generated parameter binding",
        ))
    require_literals(
        configuration_path,
        (
            ("contract::kParameterHandles", "R349",
             "parameter consumers must use the generated handle list"),
            ("automatic_allocation_enabled(enable)", "R349",
             "mode 2 must start the generated DNA contract"),
            ("allocation_storage_token()", "R349",
             "DNA mappings must use the generated persistent token"),
            ("anonymous nodes require UAVCAN1_ENABLE=2", "R349",
             "manual mode must explain why anonymous nodes stay offline"),
        ),
        violations,
    )

    node_path = _unique_dima_file(
        "DroneCanDynamicNodeId.cpp", violations,
        "DroneCAN dynamic node-ID owner",
    )
    if node_path is None:
        return
    require_literals(
        node_path,
        (
            ("uavcan_protocol_dynamic_node_id_Allocation_decode", "R349",
             "DNA requests must use the generated decoder"),
            ("uavcan_protocol_dynamic_node_id_Allocation_encode", "R349",
             "DNA responses must use the generated encoder"),
            ("generated::kAllocationFollowupTimeoutUs", "R349",
             "DNA follow-up timeout must come from generated DSDL"),
            ("PendingCommitKind::AllocatorIdentity", "R349",
             "allocator identity must be persisted before serving requests"),
            ("send_get_node_info_request", "R349",
             "NodeStatus discovery must retrieve static-node unique IDs"),
            ("uavcan_protocol_GetNodeInfoRequest_encode", "R349",
             "node discovery requests must use the generated service codec"),
        ),
        violations,
    )
    runtime_sources = tuple(sorted(
        path for path in (ROOT / "Dima").rglob("DroneCan*.cpp")
        if path.is_file()
    ))
    for source in runtime_sources:
        text = source.read_text(encoding="utf-8")
        handwritten_descriptor = re.search(
            r"\bUAVCAN_[A-Z0-9_]+_(?:ID|SIGNATURE)\b", text
        )
        if handwritten_descriptor:
            violations.append(Violation(
                source,
                line_for(text, handwritten_descriptor.group(0)),
                "R349",
                "first-party consumers must use generated subscriptions, "
                "not message ID/signature lists",
            ))

    mag_header = _unique_dima_file(
        "DroneCanMag2.hpp", violations, "DroneCAN magnetometer owner"
    )
    if mag_header is None:
        return
    require_literals(
        mag_header,
        (("make_magnetometer_device_id(source_node_id)", "R349",
          "magnetometer device ID must use the generated PX4 helper"),),
        violations,
    )
    mag_text = mag_header.read_text(encoding="utf-8")
    if "0x00430000" in mag_text or re.search(
            r"make_device_id\([^)]*sensor_id", mag_text):
        violations.append(Violation(
            mag_header, 1, "R349",
            "legacy custom device-ID packing must remain absent",
        ))

    parameter_root = ROOT / "Dima/middleware/parameters/definitions"
    for parameter_source in sorted(parameter_root.glob("*.c")):
        sensor_text = parameter_source.read_text(encoding="utf-8")
        manual_parameter = re.search(
            r"PARAM_DEFINE_(?:INT32|FLOAT)\s*\(\s*"
            r"(?:UAVCAN1_|MAG1_CAN_NODE)", sensor_text,
        )
        if manual_parameter:
            violations.append(Violation(
                parameter_source,
                line_for(sensor_text, manual_parameter.group(0)), "R349",
                "DroneCAN parameters must come from the generated manifest",
            ))

    _scan_generated_catalog(manifest_path, manifest, violations)
