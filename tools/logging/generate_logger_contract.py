#!/usr/bin/env python3
"""从产品 Logger YAML、生成参数与 uORB 目录生成唯一的 SD ULog 合同。"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import sys
import tempfile
from dataclasses import dataclass
from typing import Any

try:
    import yaml
except ImportError as error:  # pragma: no cover - 由 host-tools 前置门禁负责安装
    raise RuntimeError("PyYAML is required by the Logger contract generator") from error


EXPECTED_PRODUCT = "h743_sd_ulog"
EXPECTED_UPSTREAM_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
EXPECTED_ULOG_SOURCE = "src/modules/logger/messages.h"
EXPECTED_ULOG_SHA256 = (
    "a6b3c9a52533785ff7a50960cb5f314dfdc6946ece2276a7da03ee2a8a46b4ee"
)
EXPECTED_MODES = {
    "ArmedSessions": 0,
    "BootUntilFirstPostArmDisarm": 1,
    "BootUntilShutdown": 2,
    "FirstArmUntilShutdown": 3,
}
EXPECTED_PROFILES = {
    "GeneralRover": 0,
    "Ekf2Replay": 1,
    "RoverSystemIdentification": 2,
}
EXPECTED_PARAMETER_NAMES = {
    "mode": "SDLOG_MODE",
    "profile": "SDLOG_PROFILE",
    "directories": "SDLOG_DIRS_MAX",
}
EXPECTED_SIDECAR_FIELDS = [
    ("magic", 0, "bytes4"),
    ("version", 4, "uint8"),
    ("record_length", 5, "uint8"),
    ("flags", 6, "uint8"),
    ("reserved", 7, "uint8"),
    ("record_generation", 8, "uint32"),
    ("session_sequence", 12, "uint64"),
    ("start_monotonic_us", 20, "uint64"),
    ("boot_utc_us", 28, "uint64"),
    ("start_utc_us", 36, "uint64"),
    ("hardware_uid", 44, "uint64"),
    ("file_size", 52, "uint32"),
    ("file_crc32", 56, "uint32"),
    ("record_crc32", 60, "uint32"),
]
EXPECTED_SIDECAR_FLAGS = {
    "UtcValid": 0,
    "Closed": 1,
    "Recovered": 2,
    "Corrupt": 3,
}
DISPOSITIONS = {
    "record": "Record",
    "text": "Text",
    "parameter_trigger": "ParameterTrigger",
    "unsupported_alias": "UnsupportedAlias",
    "duplicate": "Duplicate",
    "policy_excluded": "PolicyExcluded",
}
TOPIC_ENUM_RE = re.compile(r"^\s*([a-z][a-z0-9_]*)\s*=\s*([0-9]+),\s*$")
TOPICS_LINE_RE = re.compile(r"^\s*#\s*TOPICS\s+(.+?)\s*$", re.MULTILINE)
QUEUE_LENGTH_RE = re.compile(
    r"^\s*uint8\s+ORB_QUEUE_LENGTH\s*=\s*([0-9]+)\s*(?:#.*)?$",
    re.MULTILINE,
)


class ContractError(RuntimeError):
    """权威输入缺失、互相矛盾或生成物陈旧。"""


@dataclass(frozen=True)
class Topic:
    """uORB 聚合目录中的稳定 Topic 名称和连续索引。"""

    name: str
    index: int


def sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def sha256_file(path: pathlib.Path) -> str:
    try:
        return sha256_bytes(path.read_bytes())
    except OSError as error:
        raise ContractError(f"cannot read authoritative input {path}: {error}") from error


def load_yaml(path: pathlib.Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ContractError(f"cannot load Logger policy {path}: {error}") from error
    if not isinstance(document, dict):
        raise ContractError("Logger policy root must be a mapping")
    return document


def load_json(path: pathlib.Path, label: str) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f"cannot load {label} {path}: {error}") from error
    if not isinstance(document, dict):
        raise ContractError(f"{label} root must be an object")
    return document


def camel_to_snake(name: str) -> str:
    first = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", first).lower()


def schema_topics(schema: pathlib.Path, content: str) -> list[str]:
    match = TOPICS_LINE_RE.search(content)
    if match is None:
        return [camel_to_snake(schema.stem)]
    topics = match.group(1).split()
    if not topics or any(re.fullmatch(r"[a-z][a-z0-9_]*", item) is None
                         for item in topics):
        raise ContractError(f"invalid # TOPICS declaration in {schema}")
    if len(topics) != len(set(topics)):
        raise ContractError(f"duplicate # TOPICS alias in {schema}")
    return topics


def load_schema_catalog(
    directory: pathlib.Path,
) -> tuple[dict[str, str], dict[str, str], dict[str, int]]:
    """全集扫描 `.msg`，返回 Topic->schema 和每个 schema 的内容散列。"""
    if not directory.is_dir():
        raise ContractError(f"uORB schema directory does not exist: {directory}")
    owners: dict[str, str] = {}
    hashes: dict[str, str] = {}
    queues: dict[str, int] = {}
    schemas = sorted(directory.glob("*.msg"), key=lambda item: item.name)
    if not schemas:
        raise ContractError("uORB schema directory is empty")
    for schema in schemas:
        try:
            raw = schema.read_bytes()
            content = raw.decode("utf-8")
        except (OSError, UnicodeDecodeError) as error:
            raise ContractError(f"cannot read uORB schema {schema}: {error}") from error
        hashes[schema.name] = sha256_bytes(raw)
        queue_matches = QUEUE_LENGTH_RE.findall(content)
        if len(queue_matches) > 1:
            raise ContractError(f"multiple ORB_QUEUE_LENGTH values in {schema}")
        queues[schema.name] = int(queue_matches[0]) if queue_matches else 1
        for topic in schema_topics(schema, content):
            if topic in owners:
                raise ContractError(
                    f"uORB topic {topic} is declared by both "
                    f"{owners[topic]} and {schema.name}"
                )
            owners[topic] = schema.name

    return owners, hashes, queues


def parse_uorb_topics(path: pathlib.Path) -> list[Topic]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ContractError(f"cannot read generated uORB catalogue {path}: {error}") from error
    topics = [Topic(match.group(1), int(match.group(2)))
              for line in lines if (match := TOPIC_ENUM_RE.match(line))]
    if not topics:
        raise ContractError("generated uORB catalogue contains no Topic enum")
    for expected, topic in enumerate(topics):
        if topic.index != expected:
            raise ContractError(
                f"uORB Topic IDs must be contiguous: {topic.name} is {topic.index}, "
                f"expected {expected}"
            )
    if len({topic.name for topic in topics}) != len(topics):
        raise ContractError("generated uORB catalogue contains duplicate Topic names")
    return topics


def require_exact_mapping(value: object, expected: dict[str, int], label: str) -> None:
    if value != expected:
        raise ContractError(f"{label} must be exactly {expected}")


def validate_upstream(
    policy: dict[str, Any], source: pathlib.Path, manifest_path: pathlib.Path
) -> bytes:
    reference = policy.get("px4_ulog_wire")
    expected_reference = {
        "project": "PX4-Autopilot",
        "commit": EXPECTED_UPSTREAM_COMMIT,
        "source": EXPECTED_ULOG_SOURCE,
        "sha256": EXPECTED_ULOG_SHA256,
    }
    if reference != expected_reference:
        raise ContractError("PX4 ULog wire reference is not the locked product revision")
    raw = source.read_bytes()
    if sha256_bytes(raw) != EXPECTED_ULOG_SHA256:
        raise ContractError("PX4 messages.h differs from its locked SHA-256")
    manifest = load_json(manifest_path, "PX4 source manifest")
    if (manifest.get("format") != 1 or
            manifest.get("project") != "PX4-Autopilot" or
            manifest.get("commit") != EXPECTED_UPSTREAM_COMMIT):
        raise ContractError("PX4 source manifest identity is not locked")
    files = manifest.get("files")
    if (not isinstance(files, dict) or
            files.get(EXPECTED_ULOG_SOURCE) != EXPECTED_ULOG_SHA256):
        raise ContractError("PX4 source manifest does not own the locked messages.h")
    return raw


def validate_parameter_source(document: dict[str, Any]) -> set[str]:
    """确认 Logger YAML 独占且只定义产品公开的三项参数。"""
    groups = document.get("parameters")
    if document.get("module_name") != "Logger" or not isinstance(groups, list):
        raise ContractError("module_logger.yaml must describe the Logger module")
    if len(groups) != 1 or not isinstance(groups[0], dict):
        raise ContractError("module_logger.yaml must contain one Logger parameter group")
    group = groups[0]
    definitions = group.get("definitions")
    if group.get("group") != "Logger" or not isinstance(definitions, dict):
        raise ContractError("module_logger.yaml must own one Logger definitions mapping")
    if any(not isinstance(name, str) or not isinstance(value, dict)
           for name, value in definitions.items()):
        raise ContractError("module_logger.yaml contains a malformed parameter definition")

    names = set(definitions)
    expected = set(EXPECTED_PARAMETER_NAMES.values())
    if names != expected:
        # 名称合同在权威 YAML 与生成目录之间双向核对，防止删除项被换名后仍以
        # “一个枚举、一个位掩码、一个整数”的形状绕过产品参数面约束。
        raise ContractError(
            "module_logger.yaml parameter names must be exactly "
            f"{sorted(expected)}; got {sorted(names)}"
        )
    return names


def parameter_catalog(parameters: dict[str, Any]) -> tuple[list[dict[str, Any]], int]:
    catalogue = parameters.get("parameters")
    if not isinstance(catalogue, list):
        raise ContractError("generated parameter JSON has no parameters array")
    names: set[str] = set()
    logger_parameters: list[dict[str, Any]] = []
    for item in catalogue:
        if not isinstance(item, dict) or not isinstance(item.get("name"), str):
            raise ContractError("generated parameter JSON contains a malformed entry")
        name = item["name"]
        if name in names:
            raise ContractError(f"duplicate generated parameter {name}")
        names.add(name)
        if name.startswith("SDLOG_"):
            logger_parameters.append(item)
    if len(logger_parameters) != 3:
        raise ContractError(
            "generated parameter catalogue must contain exactly three SDLOG entries"
        )
    return logger_parameters, len(catalogue)


def integer_contract(
    parameter: dict[str, Any], label: str,
) -> dict[str, Any]:
    name = parameter["name"]
    if parameter.get("type") != "Int32" or parameter.get("rebootRequired") is not True:
        raise ContractError(f"{name} must be a reboot-required Int32 parameter")
    minimum = parameter.get("min")
    maximum = parameter.get("max")
    default = parameter.get("default")
    if any(isinstance(value, bool) or not isinstance(value, int)
           for value in (minimum, maximum, default)):
        raise ContractError(f"{name} has a non-integer {label} contract")
    if minimum > maximum or default < minimum or default > maximum:
        raise ContractError(f"{name} has an invalid {label} range/default")
    return {
        "name": name,
        "minimum": minimum,
        "maximum": maximum,
        "default": default,
    }


def validate_parameters(
    document: dict[str, Any], modes: dict[str, int], profiles: dict[str, int],
    source_names: set[str],
) -> tuple[dict[str, dict[str, Any]], int]:
    logger_parameters, count = parameter_catalog(document)
    generated_by_name = {item["name"]: item for item in logger_parameters}
    if set(generated_by_name) != source_names:
        raise ContractError(
            "generated SDLOG parameters differ from module_logger.yaml: "
            f"generated={sorted(generated_by_name)}, source={sorted(source_names)}"
        )
    mode_parameter = generated_by_name[EXPECTED_PARAMETER_NAMES["mode"]]
    profile_parameter = generated_by_name[EXPECTED_PARAMETER_NAMES["profile"]]
    directory_parameter = generated_by_name[EXPECTED_PARAMETER_NAMES["directories"]]
    if ("values" not in mode_parameter or "bitmask" in mode_parameter or
            "bitmask" not in profile_parameter or "values" in profile_parameter or
            "values" in directory_parameter or "bitmask" in directory_parameter):
        raise ContractError(
            "SDLOG_MODE, SDLOG_PROFILE and SDLOG_DIRS_MAX must remain "
            "an enum, bitmask and integer respectively"
        )
    contracts = {
        "mode": integer_contract(mode_parameter, "mode"),
        "profile": integer_contract(profile_parameter, "profile"),
        "directories": integer_contract(directory_parameter, "directory"),
    }
    if (contracts["mode"]["minimum"], contracts["mode"]["maximum"],
            contracts["mode"]["default"]) != (0, 3, 0):
        raise ContractError("SDLOG mode contract must be range 0..3 with default 0")
    if (contracts["profile"]["minimum"], contracts["profile"]["maximum"],
            contracts["profile"]["default"]) != (0, 7, 1):
        raise ContractError("SDLOG profile contract must be range 0..7 with default 1")
    if (contracts["directories"]["minimum"],
            contracts["directories"]["maximum"],
            contracts["directories"]["default"]) != (1, 999, 50):
        raise ContractError("SDLOG directory contract must be range 1..999 with default 50")
    mode_values = mode_parameter.get("values")
    if (not isinstance(mode_values, list) or
            {item.get("value") for item in mode_values if isinstance(item, dict)} !=
            set(modes.values())):
        raise ContractError("SDLOG_MODE selectable values must be exactly 0..3")
    profile_bits = profile_parameter.get("bitmask")
    if (not isinstance(profile_bits, list) or
            {item.get("index") for item in profile_bits if isinstance(item, dict)} !=
            set(profiles.values())):
        raise ContractError("SDLOG_PROFILE bit positions must be exactly 0..2")
    return contracts, count


def validate_sidecar(policy: dict[str, Any]) -> dict[str, Any]:
    sidecar = policy.get("sidecar")
    if not isinstance(sidecar, dict):
        raise ContractError("sidecar must be a mapping")
    if (sidecar.get("magic") != "DLOG" or sidecar.get("version") != 1 or
            sidecar.get("record_size") != 64 or
            sidecar.get("maximum_records") != 3):
        raise ContractError("sidecar identity must be DLOG v1 with three 64-byte records")
    require_exact_mapping(sidecar.get("flags"), EXPECTED_SIDECAR_FLAGS,
                          "sidecar.flags")
    fields = sidecar.get("fields")
    if not isinstance(fields, list):
        raise ContractError("sidecar.fields must be a list")
    parsed: list[tuple[str, int, str]] = []
    for field in fields:
        if not isinstance(field, dict) or set(field) != {"name", "offset", "type"}:
            raise ContractError("each sidecar field must contain name, offset and type")
        parsed.append((field["name"], field["offset"], field["type"]))
    if parsed != EXPECTED_SIDECAR_FIELDS:
        raise ContractError("sidecar field layout differs from the fixed 64-byte v1 codec")
    return sidecar


def validate_topic_policy(
    policy: dict[str, Any], topics: list[Topic], schema_owners: dict[str, str],
    schema_queues: dict[str, int],
) -> dict[str, dict[str, Any]]:
    raw = policy.get("topics")
    if not isinstance(raw, dict):
        raise ContractError("topics must be a mapping")
    generated_names = {topic.name for topic in topics}
    policy_names = set(raw)
    schema_names = set(schema_owners)
    if generated_names != schema_names:
        raise ContractError(
            "generated uORB catalogue and .msg directory differ: "
            f"generated_only={sorted(generated_names - schema_names)}, "
            f"schema_only={sorted(schema_names - generated_names)}"
        )
    if generated_names != policy_names:
        raise ContractError(
            "Logger policy does not classify the complete uORB catalogue: "
            f"unclassified={sorted(generated_names - policy_names)}, "
            f"unknown={sorted(policy_names - generated_names)}"
        )

    utc_sources: list[str] = []
    replay_callbacks: list[str] = []
    text_sources: list[str] = []
    parameter_triggers: list[str] = []
    queue_contract_schemas: dict[str, str] = {}
    normalized: dict[str, dict[str, Any]] = {}
    for name, entry in raw.items():
        if not isinstance(name, str) or not isinstance(entry, dict):
            raise ContractError(f"invalid Logger Topic entry {name!r}")
        disposition = entry.get("disposition")
        if disposition not in DISPOSITIONS:
            raise ContractError(f"unknown disposition for {name}: {disposition!r}")
        allowed_keys = {"disposition"}
        profiles: dict[str, object] = {}
        if disposition == "record":
            allowed_keys |= {"profiles", "flush_on_stop", "utc_source",
                             "replay_callback", "queue_length"}
            profiles_value = entry.get("profiles")
            if not isinstance(profiles_value, dict) or not profiles_value:
                raise ContractError(f"record Topic {name} must select at least one profile")
            profiles = profiles_value
            for profile, rate in profiles.items():
                if profile not in EXPECTED_PROFILES:
                    raise ContractError(f"Topic {name} uses unknown profile {profile}")
                if rate != "source" and (
                        isinstance(rate, bool) or not isinstance(rate, int) or
                        rate <= 0 or rate > 1000 or 1_000_000 % rate != 0):
                    raise ContractError(
                        f"Topic {name} rate must be source or an integral-us Hz value"
                    )
            for boolean_key in ("flush_on_stop", "utc_source", "replay_callback"):
                if boolean_key in entry and not isinstance(entry[boolean_key], bool):
                    raise ContractError(f"Topic {name}.{boolean_key} must be boolean")
            if entry.get("utc_source", False):
                utc_sources.append(name)
            if entry.get("replay_callback", False):
                replay_callbacks.append(name)
            if "queue_length" in entry:
                queue_length = entry["queue_length"]
                schema = schema_owners[name]
                if (isinstance(queue_length, bool) or
                        not isinstance(queue_length, int) or
                        queue_length <= 1 or queue_length > 255 or
                        queue_length & (queue_length - 1) != 0):
                    raise ContractError(
                        f"Topic {name}.queue_length must be a power of two in 2..255"
                    )
                if schema in queue_contract_schemas:
                    raise ContractError(
                        f"schema {schema} queue is repeated by "
                        f"{queue_contract_schemas[schema]} and {name}"
                    )
                if schema_queues.get(schema) != queue_length:
                    raise ContractError(
                        f"{schema} ORB_QUEUE_LENGTH must be {queue_length}, "
                        f"got {schema_queues.get(schema)}"
                    )
                queue_contract_schemas[schema] = name
        elif disposition == "duplicate":
            allowed_keys.add("duplicate_of")
            target = entry.get("duplicate_of")
            if not isinstance(target, str) or target == name:
                raise ContractError(f"duplicate Topic {name} needs a distinct target")
        elif disposition == "text":
            text_sources.append(name)
        elif disposition == "parameter_trigger":
            parameter_triggers.append(name)
        if set(entry) - allowed_keys:
            raise ContractError(
                f"Topic {name} has fields invalid for {disposition}: "
                f"{sorted(set(entry) - allowed_keys)}"
            )
        normalized[name] = {
            "disposition": disposition,
            "profiles": profiles,
            "flush_on_stop": entry.get("flush_on_stop", False),
            "utc_source": entry.get("utc_source", False),
            "replay_callback": entry.get("replay_callback", False),
            "schema": schema_owners[name],
        }

    for name, entry in raw.items():
        if entry.get("disposition") == "duplicate":
            target = entry["duplicate_of"]
            if target not in raw or raw[target].get("disposition") != "record":
                raise ContractError(f"duplicate Topic {name} target is not a record Topic")
    if utc_sources != ["vehicle_gps_position"]:
        raise ContractError("vehicle_gps_position must be the single UTC source")
    if set(replay_callbacks) != {"vehicle_imu", "ekf2_timestamps"}:
        raise ContractError("replay callbacks must be vehicle_imu and ekf2_timestamps")
    if text_sources != ["mavlink_log"]:
        raise ContractError("mavlink_log must be the single text source")
    if parameter_triggers != ["parameter_update"]:
        raise ContractError("parameter_update must be the single parameter trigger")
    if len(queue_contract_schemas) != 4:
        raise ContractError("Logger replay policy must define exactly four queue contracts")
    return normalized


def merged_sampling(entry: dict[str, Any], profile_mask: int) -> tuple[str, int]:
    if entry["disposition"] != "record" or profile_mask == 0:
        return "Excluded", 0
    selected: list[object] = []
    for profile, rate in entry["profiles"].items():
        bit = EXPECTED_PROFILES[profile]
        if profile_mask & (1 << bit):
            selected.append(rate)
    if not selected:
        return "Excluded", 0
    if "source" in selected:
        return "SourceRate", 0
    frequency = max(int(rate) for rate in selected)
    return "FixedRate", 1_000_000 // frequency


def render_logger_header(
    topics: list[Topic], entries: dict[str, dict[str, Any]],
    parameter_contracts: dict[str, dict[str, Any]],
) -> bytes:
    mode_rows = [f"    {name} = {value}," for name, value in EXPECTED_MODES.items()]
    profile_rows = [
        f"inline constexpr std::uint8_t k{name}Profile = "
        f"static_cast<std::uint8_t>(1U << {bit}U);"
        for name, bit in EXPECTED_PROFILES.items()
    ]
    topic_rows: list[str] = []
    sampling_indices: dict[tuple[str, int], int] = {}
    assertions: list[str] = []
    utc_index = 0
    text_index = 0
    parameter_trigger_index = 0
    replay_indices: list[int] = []
    for topic in topics:
        entry = entries[topic.name]
        policies = []
        for mask in range(8):
            # 合并规则不变；相同 kind/interval 只存一次，Topic 保留逐 Profile 索引。
            # 运行期仍是有界查表，不引入动态合并、除法或采样周期量化。
            sampling = merged_sampling(entry, mask)
            index = sampling_indices.setdefault(sampling, len(sampling_indices))
            policies.append(f"{index}U")
        topic_rows.append(
            "    {TopicDisposition::%s, {{%s}}, %s}," % (
                DISPOSITIONS[entry["disposition"]],
                ", ".join(policies),
                "true" if entry["flush_on_stop"] else "false",
            )
        )
        assertions.append(
            f"static_assert(static_cast<std::size_t>(ORB_ID::{topic.name}) == "
            f"{topic.index}U);"
        )
        if entry["utc_source"]:
            utc_index = topic.index
        if entry["disposition"] == "text":
            text_index = topic.index
        if entry["disposition"] == "parameter_trigger":
            parameter_trigger_index = topic.index
        if entry["replay_callback"]:
            replay_indices.append(topic.index)

    if len(sampling_indices) > 256:
        raise ContractError("Logger sampling catalogue exceeds uint8 index capacity")
    sampling_rows = [
        f"    {{SamplingKind::{kind}, {interval}U}},"
        for kind, interval in sampling_indices
    ]

    parameter_rows = []
    for symbol, role in (
        ("kModeParameter", "mode"),
        ("kProfileParameter", "profile"),
        ("kDirectoriesParameter", "directories"),
    ):
        contract = parameter_contracts[role]
        parameter_rows.append(
            f"inline constexpr IntegerParameterContract {symbol}{{"
            f"dima::params::{contract['name']}, {contract['default']}, "
            f"{contract['minimum']}, {contract['maximum']}}};"
        )

    replay_rows = ", ".join(f"{index}U" for index in replay_indices)
    lines = [
        "// Generated by tools/logging/generate_logger_contract.py. DO NOT EDIT.",
        "#pragma once",
        "",
        "#include <parameters/param.h>",
        "#include <uORB/topics/uORBTopics.hpp>",
        "",
        "#include <array>",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace dima::modules::logging::generated {",
        "",
        "enum class LogMode : std::int32_t {",
        *mode_rows,
        "};",
        "",
        *profile_rows,
        "inline constexpr std::uint8_t kProfileMask = 0x07U;",
        "",
        "struct IntegerParameterContract {",
        "    dima::params parameter;",
        "    std::int32_t default_value;",
        "    std::int32_t minimum;",
        "    std::int32_t maximum;",
        "};",
        "",
        *parameter_rows,
        "",
        "enum class TopicDisposition : std::uint8_t {",
        "    Record,",
        "    Text,",
        "    ParameterTrigger,",
        "    UnsupportedAlias,",
        "    Duplicate,",
        "    PolicyExcluded,",
        "};",
        "",
        "enum class SamplingKind : std::uint8_t { Excluded, FixedRate, SourceRate };",
        "",
        "struct SamplingPolicy {",
        "    SamplingKind kind;",
        "    std::uint32_t interval_us;",
        "};",
        "",
        "// 只共享值相同的采样策略；每个 Topic 的八种 Profile 选择保持独立。",
        f"inline constexpr std::array<SamplingPolicy, {len(sampling_indices)}U> kSamplingPolicies{{{{",
        *sampling_rows,
        "}};",
        "",
        "// 名称/ID 直接使用 uORB metadata；回放回调使用下方生成的索引表。",
        "struct TopicPolicy {",
        "    TopicDisposition disposition;",
        "    std::array<std::uint8_t, 8U> sampling_by_profile;",
        "    bool flush_on_stop;",
        "};",
        "",
        "inline constexpr std::array<TopicPolicy, ORB_TOPICS_COUNT> kTopicPolicies{{",
        *topic_rows,
        "}};",
        "static_assert(kTopicPolicies.size() == ORB_TOPICS_COUNT);",
        *assertions,
        "",
        f"inline constexpr std::size_t kUtcSourceTopicIndex = {utc_index}U;",
        f"inline constexpr std::size_t kTextTopicIndex = {text_index}U;",
        "inline constexpr std::size_t kParameterTriggerTopicIndex = "
        f"{parameter_trigger_index}U;",
        "inline constexpr std::array<std::size_t, 2U> kReplayCallbackTopicIndices{{",
        f"    {replay_rows}",
        "}};",
        "",
        "inline constexpr const TopicPolicy &topic_policy(std::size_t index) noexcept",
        "{",
        "    return kTopicPolicies[index];",
        "}",
        "",
        "inline constexpr SamplingPolicy sampling_policy(",
        "    std::size_t index, std::uint8_t profile) noexcept",
        "{",
        "    return profile <= kProfileMask",
        "               ? kSamplingPolicies[kTopicPolicies[index].sampling_by_profile[profile]]",
        "               : SamplingPolicy{SamplingKind::Excluded, 0U};",
        "}",
        "",
        "} // namespace dima::modules::logging::generated",
        "",
    ]
    return "\n".join(lines).encode("utf-8")


def render_sidecar_header(sidecar: dict[str, Any]) -> bytes:
    offsets = {field["name"]: field["offset"] for field in sidecar["fields"]}
    known_flag_mask = sum(1 << bit for bit in sidecar["flags"].values())
    flag_rows = [
        f"inline constexpr std::uint8_t k{symbol}Flag = "
        f"static_cast<std::uint8_t>(1U << {bit}U);"
        for symbol, bit in EXPECTED_SIDECAR_FLAGS.items()
    ]
    lines = [
        "// Generated by tools/logging/generate_logger_contract.py. DO NOT EDIT.",
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <cstring>",
        "#include <limits>",
        "",
        "namespace dima::modules::logging::generated::sidecar {",
        "",
        f"inline constexpr std::size_t kRecordSize = {sidecar['record_size']}U;",
        f"inline constexpr std::size_t kMaximumRecords = {sidecar['maximum_records']}U;",
        f"inline constexpr std::uint8_t kVersion = {sidecar['version']}U;",
        "inline constexpr std::uint8_t kMagic[4]{'D', 'L', 'O', 'G'};",
        *flag_rows,
        f"inline constexpr std::uint8_t kKnownFlags = 0x{known_flag_mask:02x}U;",
        "inline constexpr std::uint64_t kMinimumUtcUs = 1577836800000000ULL;",
        "",
        "struct Metadata {",
        "    std::uint32_t record_generation{0U};",
        "    std::uint8_t flags{0U};",
        "    std::uint64_t session_sequence{0U};",
        "    std::uint64_t start_monotonic_us{0U};",
        "    std::uint64_t boot_utc_us{0U};",
        "    std::uint64_t start_utc_us{0U};",
        "    std::uint64_t hardware_uid{0U};",
        "    std::uint32_t file_size{0U};",
        "    std::uint32_t file_crc32{0U};",
        "};",
        "",
        "inline constexpr std::uint32_t kCrc32Initial = 0xffffffffU;",
        "inline constexpr std::uint32_t crc32_finish(std::uint32_t state) noexcept",
        "{",
        "    return state ^ 0xffffffffU;",
        "}",
        "",
        "inline std::uint32_t crc32_update(std::uint32_t state,",
        "                                  const std::uint8_t *data,",
        "                                  std::size_t size) noexcept",
        "{",
        "    if (data == nullptr && size != 0U) {",
        "        return state;",
        "    }",
        "    for (std::size_t index = 0U; index < size; ++index) {",
        "        state ^= data[index];",
        "        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {",
        "            const std::uint32_t mask =",
        "                static_cast<std::uint32_t>(0U - (state & 1U));",
        "            state = (state >> 1U) ^ (0xedb88320U & mask);",
        "        }",
        "    }",
        "    return state;",
        "}",
        "",
        "inline void write_u32(std::uint8_t *destination, std::uint32_t value) noexcept",
        "{",
        "    for (std::uint8_t byte = 0U; byte < 4U; ++byte) {",
        "        destination[byte] = static_cast<std::uint8_t>(value >> (8U * byte));",
        "    }",
        "}",
        "",
        "inline void write_u64(std::uint8_t *destination, std::uint64_t value) noexcept",
        "{",
        "    for (std::uint8_t byte = 0U; byte < 8U; ++byte) {",
        "        destination[byte] = static_cast<std::uint8_t>(value >> (8U * byte));",
        "    }",
        "}",
        "",
        "inline std::uint32_t read_u32(const std::uint8_t *source) noexcept",
        "{",
        "    std::uint32_t value = 0U;",
        "    for (std::uint8_t byte = 0U; byte < 4U; ++byte) {",
        "        value |= static_cast<std::uint32_t>(source[byte]) << (8U * byte);",
        "    }",
        "    return value;",
        "}",
        "",
        "inline std::uint64_t read_u64(const std::uint8_t *source) noexcept",
        "{",
        "    std::uint64_t value = 0U;",
        "    for (std::uint8_t byte = 0U; byte < 8U; ++byte) {",
        "        value |= static_cast<std::uint64_t>(source[byte]) << (8U * byte);",
        "    }",
        "    return value;",
        "}",
        "",
        "inline void encode(const Metadata &metadata, std::uint8_t *record) noexcept",
        "{",
        "    std::memset(record, 0, kRecordSize);",
        f"    std::memcpy(record + {offsets['magic']}U, kMagic, sizeof(kMagic));",
        f"    record[{offsets['version']}U] = kVersion;",
        f"    record[{offsets['record_length']}U] = static_cast<std::uint8_t>(kRecordSize);",
        f"    record[{offsets['flags']}U] = metadata.flags;",
        f"    write_u32(record + {offsets['record_generation']}U, metadata.record_generation);",
        f"    write_u64(record + {offsets['session_sequence']}U, metadata.session_sequence);",
        f"    write_u64(record + {offsets['start_monotonic_us']}U, metadata.start_monotonic_us);",
        f"    write_u64(record + {offsets['boot_utc_us']}U, metadata.boot_utc_us);",
        f"    write_u64(record + {offsets['start_utc_us']}U, metadata.start_utc_us);",
        f"    write_u64(record + {offsets['hardware_uid']}U, metadata.hardware_uid);",
        f"    write_u32(record + {offsets['file_size']}U, metadata.file_size);",
        f"    write_u32(record + {offsets['file_crc32']}U, metadata.file_crc32);",
        "    const std::uint32_t checksum = crc32_finish(",
        f"        crc32_update(kCrc32Initial, record, {offsets['record_crc32']}U));",
        f"    write_u32(record + {offsets['record_crc32']}U, checksum);",
        "}",
        "",
        "inline bool decode(const std::uint8_t *record, Metadata &metadata) noexcept",
        "{",
        "    if (record == nullptr ||",
        f"        std::memcmp(record + {offsets['magic']}U, kMagic, sizeof(kMagic)) != 0 ||",
        f"        record[{offsets['version']}U] != kVersion ||",
        f"        record[{offsets['record_length']}U] != kRecordSize ||",
        f"        record[{offsets['reserved']}U] != 0U) {{",
        "        return false;",
        "    }",
        "    const std::uint32_t expected = crc32_finish(",
        f"        crc32_update(kCrc32Initial, record, {offsets['record_crc32']}U));",
        f"    if (read_u32(record + {offsets['record_crc32']}U) != expected) {{",
        "        return false;",
        "    }",
        f"    metadata.flags = record[{offsets['flags']}U];",
        f"    metadata.record_generation = read_u32(record + {offsets['record_generation']}U);",
        f"    metadata.session_sequence = read_u64(record + {offsets['session_sequence']}U);",
        f"    metadata.start_monotonic_us = read_u64(record + {offsets['start_monotonic_us']}U);",
        f"    metadata.boot_utc_us = read_u64(record + {offsets['boot_utc_us']}U);",
        f"    metadata.start_utc_us = read_u64(record + {offsets['start_utc_us']}U);",
        f"    metadata.hardware_uid = read_u64(record + {offsets['hardware_uid']}U);",
        f"    metadata.file_size = read_u32(record + {offsets['file_size']}U);",
        f"    metadata.file_crc32 = read_u32(record + {offsets['file_crc32']}U);",
        "    if (metadata.record_generation == 0U ||",
        "        metadata.record_generation > kMaximumRecords ||",
        "        metadata.session_sequence == 0U ||",
        "        (metadata.flags & static_cast<std::uint8_t>(~kKnownFlags)) != 0U) {",
        "        return false;",
        "    }",
        "    const bool utc_valid = (metadata.flags & kUtcValidFlag) != 0U;",
        "    if (!utc_valid) {",
        "        return metadata.boot_utc_us == 0U && metadata.start_utc_us == 0U;",
        "    }",
        "    // UTC 映射必须满足 start=boot+monotonic，且能无损装入 MAVLink 秒字段。",
        "    if (metadata.boot_utc_us >",
        "        std::numeric_limits<std::uint64_t>::max() -",
        "            metadata.start_monotonic_us) {",
        "        return false;",
        "    }",
        "    const std::uint64_t calculated_start =",
        "        metadata.boot_utc_us + metadata.start_monotonic_us;",
        "    return calculated_start == metadata.start_utc_us &&",
        "           calculated_start >= kMinimumUtcUs &&",
        "           calculated_start / 1000000ULL <=",
        "               std::numeric_limits<std::uint32_t>::max();",
        "}",
        "",
        "} // namespace dima::modules::logging::generated::sidecar",
        "",
    ]
    return "\n".join(lines).encode("utf-8")


def canonical_input_hashes(
    arguments: argparse.Namespace, schema_hashes: dict[str, str]
) -> dict[str, str]:
    inputs = {
        "logger_topics.yaml": sha256_file(arguments.policy),
        "module_logger.yaml": sha256_file(arguments.parameter_source),
        "parameters.json": sha256_file(arguments.parameters),
        "uORBTopics.hpp": sha256_file(arguments.uorb_topics),
        "px4_source_manifest.json": sha256_file(arguments.source_manifest),
        "px4_messages.h": sha256_file(arguments.ulog_source),
    }
    for name, digest in sorted(schema_hashes.items()):
        inputs[f"schemas/{name}"] = digest
    return inputs


def write_if_changed(path: pathlib.Path, content: bytes) -> None:
    if path.is_file() and path.read_bytes() == content:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = ""
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=path.parent, delete=False
        ) as temporary:
            temporary.write(content)
            temporary_name = temporary.name
        os.replace(temporary_name, path)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def generate(arguments: argparse.Namespace) -> None:
    policy = load_yaml(arguments.policy)
    if policy.get("schema_version") != 1 or policy.get("product") != EXPECTED_PRODUCT:
        raise ContractError("unsupported Logger policy identity")
    require_exact_mapping(policy.get("modes"), EXPECTED_MODES, "modes")
    require_exact_mapping(policy.get("profiles"), EXPECTED_PROFILES, "profiles")
    sidecar = validate_sidecar(policy)
    parameter_source_names = validate_parameter_source(
        load_yaml(arguments.parameter_source)
    )
    schema_owners, schema_hashes, schema_queues = load_schema_catalog(
        arguments.schemas
    )
    topics = parse_uorb_topics(arguments.uorb_topics)
    entries = validate_topic_policy(
        policy, topics, schema_owners, schema_queues
    )
    ulog_header = validate_upstream(
        policy, arguments.ulog_source, arguments.source_manifest
    )
    parameter_contracts, parameter_count = validate_parameters(
        load_json(arguments.parameters, "generated parameter JSON"),
        EXPECTED_MODES,
        EXPECTED_PROFILES,
        parameter_source_names,
    )

    logger_header = render_logger_header(topics, entries, parameter_contracts)
    sidecar_header = render_sidecar_header(sidecar)
    outputs: dict[str, bytes] = {
        "logger_contract.hpp": logger_header,
        "logger_sidecar.hpp": sidecar_header,
        "ulog_messages.hpp": ulog_header,
    }
    manifest = {
        "format": 1,
        "product": EXPECTED_PRODUCT,
        "parameter_count": parameter_count,
        "topic_count": len(topics),
        "inputs": canonical_input_hashes(arguments, schema_hashes),
        "outputs": {name: sha256_bytes(content)
                    for name, content in sorted(outputs.items())},
        "px4_ulog_wire": policy["px4_ulog_wire"],
        "topics": [
            {
                "index": topic.index,
                "name": topic.name,
                "schema": entries[topic.name]["schema"],
                "disposition": entries[topic.name]["disposition"],
                "sampling": [
                    {"profile": mask, "kind": merged_sampling(entries[topic.name], mask)[0],
                     "interval_us": merged_sampling(entries[topic.name], mask)[1]}
                    for mask in range(8)
                ],
            }
            for topic in topics
        ],
    }
    outputs[".generated.json"] = (
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")

    arguments.output.mkdir(parents=True, exist_ok=True)
    if arguments.verify:
        stale = [
            str(arguments.output / name)
            for name, expected in outputs.items()
            if not (arguments.output / name).is_file()
            or (arguments.output / name).read_bytes() != expected
        ]
        if stale:
            raise ContractError("generated Logger contract is stale: " + ", ".join(stale))
        print(
            f"verified Logger contract: {len(topics)} Topics, "
            f"{parameter_count} parameters, 8 Profile combinations"
        )
        return
    for name, content in outputs.items():
        write_if_changed(arguments.output / name, content)
    print(
        f"generated Logger contract: {len(topics)} Topics, "
        f"{parameter_count} parameters, 8 Profile combinations"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=pathlib.Path, required=True)
    parser.add_argument("--schemas", type=pathlib.Path, required=True)
    parser.add_argument("--uorb-topics", type=pathlib.Path, required=True)
    parser.add_argument("--parameters", type=pathlib.Path, required=True)
    parser.add_argument("--parameter-source", type=pathlib.Path, required=True)
    parser.add_argument("--ulog-source", type=pathlib.Path, required=True)
    parser.add_argument("--source-manifest", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--verify", action="store_true")
    return parser.parse_args()


def main() -> int:
    try:
        generate(parse_args())
    except (ContractError, OSError, RuntimeError) as error:
        print(f"Logger contract generation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
