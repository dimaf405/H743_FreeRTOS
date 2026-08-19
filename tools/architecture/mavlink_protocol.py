"""MAVLink identity, Metadata, FTP, RC stream, and policy checks."""

from __future__ import annotations

import json
import pathlib
import re

from architecture.common import (
    ROOT,
    Violation,
    line_for,
    require_literals,
    MAKE_CONTRACT_PATHS,
    find_literal_owner,
    owner_texts,
    require_literals_in_owners,
)


def scan_mavlink_protocol_contract(
        violations: list[Violation],
        mavlink_parameters_path: pathlib.Path,
        mavlink_parameters_text: str) -> None:
    mavlink_service_owners = (
        ROOT / "Dima/modules/mavlink/MavlinkService.cpp",
        ROOT / "Dima/modules/mavlink/MavlinkRcStream.cpp",
        ROOT / "Dima/modules/mavlink/MavlinkSystemMessages.cpp",
    )
    mavlink_parameter_owners = (
        mavlink_parameters_path,
        ROOT / "Dima/modules/mavlink/MavlinkParameterExt.cpp",
    )
    commander_owners = (
        ROOT / "Dima/modules/safety/Commander.cpp",
        ROOT / "Dima/modules/safety/CommanderSafety.cpp",
        ROOT / "Dima/modules/safety/CommanderActions.cpp",
        ROOT / "Dima/modules/safety/CommanderCommands.cpp",
    )
    require_literals(
        ROOT / "Dima/messages/vehicle_command.hpp",
        (("std::uint8_t  source_system;", "R332",
          "vehicle_command must retain the MAVLink source system"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkIdentity.cpp",
        (
            ("MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT", "R333",
             "MAVLink parameter capability is missing"),
            ("MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE", "R333",
             "MAVLink bytewise parameter encoding is missing"),
            ("MAV_PROTOCOL_CAPABILITY_FTP", "R333",
             "read-only parameter Metadata FTP capability is missing"),
            ("MAV_PROTOCOL_CAPABILITY_COMMAND_INT", "R333",
             "COMMAND_INT capability is missing"),
            ("MAV_PROTOCOL_CAPABILITY_MAVLINK2", "R333",
             "MAVLink2 capability is missing"),
        ),
        violations,
    )
    identity_path = ROOT / "Dima/modules/mavlink/MavlinkIdentity.cpp"
    identity_text = identity_path.read_text(encoding="utf-8")
    capability_tokens = set(re.findall(
        r"MAV_PROTOCOL_CAPABILITY_[A-Z0-9_]+", identity_text,
    ))
    expected_capabilities = {
        "MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT",
        "MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE",
        "MAV_PROTOCOL_CAPABILITY_FTP",
        "MAV_PROTOCOL_CAPABILITY_COMMAND_INT",
        "MAV_PROTOCOL_CAPABILITY_MAVLINK2",
    }
    if capability_tokens != expected_capabilities:
        violations.append(Violation(
            identity_path, 1, "R333",
            "MAVLink capability set differs from implemented surface: "
            f"{sorted(capability_tokens)}",
        ))
    require_literals_in_owners(
        MAKE_CONTRACT_PATHS,
        (
            ("MAVLINK_GENERATED_DIR := $(BUILD_DIR)/generated/mavlink", "R334",
             "MAVLink generation must stay under build/generated"),
            ("PARAMETER_METADATA_GENERATOR := tools/mavlink/"
             "generate_parameter_metadata.py", "R334",
             "parameter Metadata generator is missing"),
            ("PARAMETER_METADATA_DIR := $(BUILD_DIR)/generated/"
             "component_metadata", "R334",
             "parameter Metadata outputs must stay under build/generated"),
            ("$(PARAMETER_METADATA_STAMP)", "R334",
             "firmware objects must depend on generated parameter Metadata"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/mavlink/build_trimmed_dialect.py",
        (
            ("install_generated_tree(generated, output_dir)", "R334",
             "Windows MAVLink generation must tolerate short directory locks"),
            ('"FILE_TRANSFER_PROTOCOL"', "R337",
             "Metadata FTP message is absent from the dialect"),
            ('"COMPONENT_METADATA"', "R337",
             "modern Component Metadata message is absent"),
            ('"COMPONENT_INFORMATION"', "R337",
             "deprecated Component Information fallback is absent"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/mavlink/generate_parameter_metadata.py",
        (
            ('GENERAL_URI = "mftp://etc/extras/'
             'component_general.json.xz"', "R337",
             "General Metadata URI changed"),
            ('PARAMETER_URI = "mftp://etc/extras/parameters.json.xz"',
             "R337", "Parameter Metadata URI changed"),
            ('INTERNAL_PARAMETERS = {"RC_PORT_CONFIG", "DIMA_SER_VER"}',
             "R337", "internal parameters entered public Metadata"),
            ('"type": 1', "R337",
             "General Metadata must advertise parameters only"),
            ('"version": 1', "R337",
             "QGC parameter Metadata version must remain one"),
            ("general_crc = mavlink_crc32(general_json)", "R337",
             "General Metadata CRC must follow PX4 uncompressed semantics"),
            ("parameter_crc = mavlink_crc32(parameter_xz)", "R337",
             "Parameter Metadata CRC must cover the served XZ file"),
            ("validate_parameter(parameter, index)", "R337",
             "QGC parameter object validation is missing"),
            ("lzma.decompress(parameter_xz) != parameter_json", "R337",
             "Parameter Metadata XZ round-trip validation is missing"),
            ("lzma.decompress(general_xz) != general_json", "R337",
             "General Metadata XZ round-trip validation is missing"),
        ),
        violations,
    )
    metadata_generator_text = (
        ROOT / "tools/mavlink/generate_parameter_metadata.py"
    ).read_text(encoding="utf-8")
    for forbidden in (
            "COMP_METADATA_TYPE_ACTUATORS", "actuators.json",
            "events.json", '"type": 4', '"type": 5'):
        if forbidden in metadata_generator_text:
            violations.append(Violation(
                ROOT / "tools/mavlink/generate_parameter_metadata.py",
                line_for(metadata_generator_text, forbidden), "R337",
                "only Parameter Metadata may be generated",
            ))
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkService.hpp",
        (
            ("SubscriptionData<input_rc_s>", "R335",
             "MAVLink must subscribe to raw input_rc for QGC calibration"),
            ("kRcChannelsIntervalUs = 200000ULL", "R336",
             "QGC raw RC stream must remain fixed at 5 Hz"),
            ("bool rc_stream_active_{false};", "R336",
             "QGC raw RC stream must retain loss/recovery edge state"),
            ("MavlinkMetadataFtp metadata_ftp_", "R337",
             "MavlinkService must own the read-only Metadata FTP state"),
            ("bool transport_was_ready_{false};", "R337",
             "physical USB readiness must own FTP session lifetime"),
            ("bool send_component_metadata()", "R337",
             "modern Component Metadata response is missing"),
            ("bool send_component_information()", "R337",
             "deprecated Component Information fallback is missing"),
        ),
        violations,
    )
    require_literals_in_owners(
        mavlink_service_owners,
        (
            ("mavlink_msg_rc_channels_encode", "R335",
             "MAVLink must stream raw RC_CHANNELS"),
            ("if (!rc_sample_streamable(now)) {", "R336",
             "missing or stale raw RC must stop the QGC stream"),
            ("channels.chancount = channel_count;", "R336",
             "RC_CHANNELS must carry the real valid channel count"),
            ("case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:", "R337",
             "FILE_TRANSFER_PROTOCOL dispatch is missing"),
            ("metadata_ftp_.handle_message(&msg, hrt_absolute_time())", "R337",
             "FTP requests must retain Runtime time and source routing"),
            ("if (!metadata_ftp_.service(now))", "R337",
             "pending FTP must block lower-priority parameter/log traffic"),
            ("case MAVLINK_MSG_ID_COMPONENT_METADATA:", "R337",
             "MAV_CMD_REQUEST_MESSAGE cannot serve COMPONENT_METADATA"),
            ("case MAVLINK_MSG_ID_COMPONENT_INFORMATION:", "R337",
             "MAV_CMD_REQUEST_MESSAGE cannot serve fallback information"),
            ("metadata_ftp_.reset();", "R337",
             "Runtime/link loss must clear FTP state"),
            ("if (!transport_ready && transport_was_ready_)", "R337",
             "physical USB falling edge must clear FTP state"),
            ("discard_rx();", "R337",
             "USB falling edge must discard stale Console input"),
            ("reset_parser_state();", "R337",
             "USB falling edge must reset the MAVLink parser"),
        ),
        violations,
    )
    mavlink_service_path = ROOT / "Dima/modules/mavlink/MavlinkService.cpp"
    mavlink_service_owner_texts = owner_texts(mavlink_service_owners)
    if sum(text.count("metadata_ftp_.reset();")
           for _path, text in mavlink_service_owner_texts) < 2:
        violations.append(Violation(
            mavlink_service_path, 1, "R337",
            "FTP state must reset on Runtime reset and USB disconnect",
        ))
    for forbidden in ("rc_invalid_sent_", "channels.chancount = 0"):
        owner = find_literal_owner(mavlink_service_owners, forbidden)
        if owner is not None:
            owner_path, owner_text = owner
            violations.append(Violation(
                owner_path,
                line_for(owner_text, forbidden),
                "R336",
                "RC loss must stop RC_CHANNELS instead of sending a zero-count "
                "snapshot",
            ))
    for forbidden in ("latest_input_rc_.rc_failsafe",
                      "latest_input_rc_.rc_lost"):
        owner = find_literal_owner(mavlink_service_owners, forbidden)
        if owner is not None:
            owner_path, owner_text = owner
            violations.append(Violation(
                owner_path,
                line_for(owner_text, forbidden),
                "R336",
                "RC failsafe/lost flags must gate control, not hide a fresh "
                "raw RC_CHANNELS sample from QGC",
            ))
    ftp_path = ROOT / "Dima/modules/mavlink/MavlinkMetadataFtp.hpp"
    require_literals(
        ftp_path,
        (
            ("kMaxDataLength == 239U", "R337",
             "MAVLink FTP data capacity changed"),
            ("sizeof(PayloadHeader) == 12U", "R337",
             "MAVLink FTP header must match PX4/QGC"),
            ("sizeof(Payload) ==", "R337",
             "MAVLink FTP full payload contract is missing"),
            ("kCmdOpenFileRO", "R337", "OpenFileRO support is missing"),
            ("kCmdReadFile", "R337", "ReadFile hole repair is missing"),
            ("kCmdBurstReadFile", "R337", "BurstReadFile support is missing"),
            ("kCmdResetSessions", "R337", "ResetSessions support is missing"),
            ("kCmdTerminateSession", "R337",
             "TerminateSession support is missing"),
            ("kMaxTxRetries = 4U", "R337",
             "FTP transient TX retries must remain bounded"),
            ("kSessionTimeoutUs = 10000000ULL", "R337",
             "stale FTP sessions need a ten-second timeout"),
            ("ErrorCode reset_sessions()", "R337",
             "ResetSessions must retain PX4 reset-all semantics"),
        ),
        violations,
    )
    ftp_implementation_path = (
        ROOT / "Dima/modules/mavlink/MavlinkMetadataFtp.cpp"
    )
    require_literals(
        ftp_implementation_path,
        (
            ("reply.header.burst_complete = burst ? 1U : 0U", "R337",
             "single-packet bursts must explicitly complete"),
            ("path_length == request.header.size", "R337",
             "virtual Metadata paths must use exact length matching"),
            ("session_owned_by(key)", "R337",
             "FTP sessions must remain bound to the requester"),
            ("response.target_system = key.source_system", "R337",
             "FTP replies must be directed back to the request source"),
            ("if (error == EAGAIN &&", "R337",
             "only definitely unsent FTP frames may be actively retried"),
            ("reply_.valid && same_request", "R337",
             "duplicate FTP requests must replay an identical response"),
            ("if (reply_.pending)", "R337",
             "an unsent FTP response must not be overwritten"),
            ("request_fingerprint(request.payload", "R337",
             "duplicate FTP keys must cover the complete request payload"),
        ),
        violations,
    )
    ftp_text = ftp_implementation_path.read_text(encoding="utf-8")
    handled_ftp_opcodes = set(re.findall(
        r"case\s+(kCmd[A-Za-z0-9_]+)\s*:", ftp_text
    ))
    expected_ftp_opcodes = {
        "kCmdNone", "kCmdTerminateSession", "kCmdResetSessions",
        "kCmdOpenFileRO", "kCmdReadFile", "kCmdBurstReadFile",
    }
    if handled_ftp_opcodes != expected_ftp_opcodes:
        violations.append(Violation(
            ftp_implementation_path, 1, "R337",
            "read-only FTP opcode surface changed: "
            f"{sorted(handled_ftp_opcodes)}",
        ))
    require_literals(
        ROOT / "Dima/adapters/usb_console/UsbConsole.cpp",
        (("kTxCapacity = 280U", "R337",
          "USB TX staging cannot fit a maximum MAVLink2 FTP frame"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/HeartbeatPacer.hpp",
        (
            ("kPx4CustomModeManual = 1UL << 16", "R336",
             "PX4 Manual custom_mode encoding changed"),
            ("kPx4CustomModeTermination = 10UL << 16", "R336",
             "PX4 Termination custom_mode encoding changed"),
            ("SubscriptionData<vehicle_control_mode_s>", "R336",
             "HEARTBEAT must project the published control mode"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/HeartbeatPacer.cpp",
        (("MAV_MODE_FLAG_MANUAL_INPUT_ENABLED", "R336",
          "Manual HEARTBEAT base_mode flag is missing"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkIdentity.hpp",
        (("MAV_AUTOPILOT_VALUE = 12", "R336",
          "stock QGC compatibility requires MAV_AUTOPILOT_PX4"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/middleware/parameters/definitions/rc_input_mapping_params.c",
        (
            ("PARAM_DEFINE_INT32(RC_MAP_ROLL, 0);", "R336",
             "Rover Roll mapping must default unassigned"),
            ("PARAM_DEFINE_INT32(RC_MAP_PITCH, 0);", "R336",
             "Rover Pitch mapping must default unassigned"),
            ("PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 1);", "R336",
             "Rover Throttle mapping must default to channel 1"),
            ("PARAM_DEFINE_INT32(RC_MAP_YAW, 2);", "R336",
             "Rover Yaw mapping must default to channel 2"),
            ("Dima has no selectable flight modes", "R336",
             "RC_MAP_FLTMODE must remain an explicit disabled placeholder"),
            ("/** Arm 开关触发阈值；负值表示反向比较。", "R336",
             "Arm switch polarity metadata is missing"),
        ),
        violations,
    )
    require_literals_in_owners(
        mavlink_parameter_owners,
        (
            ("kQgcFixedInt32Parameters", "R336",
             "fixed QGC INT32 registry is missing"),
            ("is_qgc_fixed_parameter(name)", "R336",
             "QGC compatibility parameters must be active before LIST"),
            ("return value == fixed->value;", "R336",
             "unsupported QGC INT32 writes must be rejected"),
            ("std::strcmp(name, \"RC_MAP_FLTMODE\") == 0", "R336",
             "disabled flight-mode mapping compatibility guard is missing"),
            ("return mapping == 0;", "R336",
             "flight-mode mapping writes must remain disabled"),
            ("std::strcmp(name, \"RC_PORT_CONFIG\") == 0", "R336",
             "legacy RC_PORT_CONFIG must remain write-protected"),
            ("return protocol == 0 || protocol == 2;", "R336",
             "RC_INPUT_PROTO writes must be limited to Disabled or SBUS"),
            ("serial_baud_parameter(name)", "R336",
             "SERIALx baud parameters must be active before LIST"),
            ("serial_function_parameter(name)", "R336",
             "SERIALx function parameters must be active before LIST"),
            ("supported_serial_baud(baudrate)", "R336",
             "serial baud writes must be limited to implemented rates"),
            ("serial_function_write_allowed(name, function)", "R336",
             "serial function writes must preserve single RC ownership"),
            ("is_internal_parameter(name)", "R336",
             "migration-only parameters must be hidden from named requests"),
            ("param_foreach(&MavlinkParameters::append_used_parameter",
             "R331", "PARAM_REQUEST_LIST must snapshot the used set"),
            ("msg.param_count = count;", "R331",
             "LIST PARAM_VALUE frames must use the frozen count"),
            ("msg.param_index = index;", "R331",
             "LIST PARAM_VALUE frames must use the frozen index"),
            ("_send_all_snapshot[requested_index]", "R331",
             "indexed PARAM_REQUEST_READ must use the latest LIST snapshot"),
            ("const int snapshot_index = parameter_snapshot_index(param);",
             "R331", "READ/SET replies must reuse frozen LIST metadata"),
            ("void MavlinkParameters::stop_parameter_stream() noexcept", "R331",
             "LIST completion must stop only the send cursor"),
            ("void MavlinkParameters::clear_parameter_snapshot() noexcept", "R331",
             "Runtime and hash reset must invalidate the LIST snapshot"),
        ),
        violations,
    )
    for forbidden in (
            "FixedFloatParameter", "kQgcFixedFloatParameters",
            "RTL_RETURN_ALT", "RTL_DESCEND_ALT", "RTL_LAND_DELAY",
            "COM_FLTMODE1", "COM_FLTMODE2", "COM_FLTMODE3",
            "COM_FLTMODE4", "COM_FLTMODE5", "COM_FLTMODE6"):
        owner = find_literal_owner(mavlink_parameter_owners, forbidden)
        if owner is not None:
            owner_path, owner_text = owner
            violations.append(Violation(
                owner_path,
                line_for(owner_text, forbidden), "R336",
                f"unimplemented mode parameter path remains: {forbidden}",
            ))
    require_literals_in_owners(
        mavlink_service_owners,
        (
            ("param_find(\"MAV_SYS_ID\")", "R336",
             "fixed MAVLink system ID must have a runtime consumer"),
            ("system_id == 1", "R336",
             "MAVLink system ID must remain fixed at one"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/rc/RcManualInput.cpp",
        (
            ("parameter_update_subscription_.update()", "R336",
             "RC switch mapping changes must be observed"),
            ("reset_switch_baseline();", "R336",
             "RC switch configuration changes must establish a safe baseline"),
            ("action_request_s::ACTION_ARM", "R336",
             "two-position Arm ON edge is missing"),
            ("action_request_s::ACTION_DISARM", "R336",
             "two-position Arm OFF edge is missing"),
        ),
        violations,
    )
    rc_manual_path = ROOT / "Dima/modules/rc/RcManualInput.cpp"
    rc_manual_text = rc_manual_path.read_text(encoding="utf-8")
    for forbidden in ("COM_FLTMODE", "SOURCE_RC_MODE_SLOT",
                      "ACTION_SWITCH_MODE", "flight_modes_"):
        if forbidden in rc_manual_text:
            violations.append(Violation(
                rc_manual_path, line_for(rc_manual_text, forbidden), "R336",
                "unimplemented RC flight-mode action path remains",
            ))
    require_literals_in_owners(
        commander_owners,
        (
            ("case vehicle_command_s::NAV_CMD_PREFLIGHT_CALIBRATION:",
             "R335", "Commander must arbitrate RC calibration"),
            ("vehicle_status_.rc_calibration_in_progress = true;", "R335",
             "Commander must publish RC calibration state"),
            ("param_find(\"NAV_RCL_ACT\")", "R336",
             "RC-loss action compatibility parameter has no consumer"),
            ("param_find(\"NAV_DLL_ACT\")", "R336",
             "data-link-loss compatibility parameter has no consumer"),
            ("kRcLossActionDisarm", "R336",
             "RC loss must remain fixed to Disarm"),
            ("kDataLinkLossActionDisabled", "R336",
             "GCS loss must remain disabled"),
        ),
        violations,
    )
    for forbidden in ("ACTION_SWITCH_MODE", "SOURCE_RC_MODE_SLOT",
                      "switch_mode("):
        owner = find_literal_owner(commander_owners, forbidden)
        if owner is not None:
            owner_path, owner_text = owner
            violations.append(Violation(
                owner_path, line_for(owner_text, forbidden), "R336",
                "Commander must not claim an unimplemented selectable mode",
            ))
    require_literals(
        ROOT / "Dima/modules/rc/RCUpdate.cpp",
        (("assign(Mapping::Flaps, rc_channels_s::FUNCTION_FLAPS);", "R335",
          "RC_MAP_FLAPS must have a production mapping consumer"),),
        violations,
    )
    rc_update_path = ROOT / "Dima/modules/rc/RCUpdate.cpp"
    rc_update_text = rc_update_path.read_text(encoding="utf-8")
    for forbidden in ("RC_MAP_FLTMODE", "Mapping::FlightMode", "mode_slot()"):
        if forbidden in rc_update_text:
            violations.append(Violation(
                rc_update_path, line_for(rc_update_text, forbidden), "R336",
                "disabled mode compatibility parameter entered RC runtime",
            ))

    lock_path = ROOT / "tools/mavlink/mavlink.lock.json"
    try:
        lock = json.loads(lock_path.read_text(encoding="utf-8"))
        if lock["dialect"]["message_count"] != 24:
            raise ValueError("message_count is not 24")
        forbidden = set(lock["dialect"]["forbidden_messages"])
        if forbidden != {"COMPONENT_INFORMATION_BASIC"}:
            raise ValueError("forbidden message set changed")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        violations.append(Violation(
            lock_path, 1, "R334", f"invalid MAVLink lock contract: {error}",
        ))
