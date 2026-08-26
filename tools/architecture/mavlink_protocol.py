"""MAVLink 身份、生成 Metadata、只读 FTP、RC/传感器流与速率策略门禁。"""

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
    """核对公开能力与真实处理面一致，并约束消息/metadata 必须来自生成合同。"""
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
    sensor_calibration_root = ROOT / "Dima/modules/sensors/calibration"
    sensor_calibration_owners = tuple(sorted(
        (*sensor_calibration_root.glob("SensorCalibration*.cpp"),
         *sensor_calibration_root.glob("SensorCalibration*.hpp"))
    ))
    require_literals(
        ROOT / "Dima/messages/schemas/vehicle_command.msg",
        (("uint8 source_system", "R332",
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
            ("parameter-metadata-verify", "R337",
             "formal builds do not verify generated Metadata stamps"),
            ("--output $(PARAMETER_METADATA_DIR) --verify", "R337",
             "Metadata stamp verification is not invoked"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/mavlink/build_trimmed_dialect.py",
        (
            ("install_generated_tree(generated, output_dir)", "R334",
             "Windows MAVLink generation must tolerate short directory locks"),
            ("source_message_ids(", "R334",
             "MAVLink IDs must be resolved from the pinned XML inputs"),
            ("validate_runtime_contract(", "R344",
             "generated MAVLink stream contract validation is missing"),
            ("validate_inbound_contract(", "R337",
             "generated MAVLink inbound routing validation is missing"),
            ("write_runtime_contract(", "R344",
             "generated MAVLink runtime contract writer is missing"),
            ('generated / "mavlink_stream_contract.hpp"', "R344",
             "generated MAVLink runtime contract header is missing"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkService.hpp",
        (
            ("SubscriptionData<input_rc_s>", "R335",
             "MAVLink must subscribe to raw input_rc for QGC calibration"),
            ('#include "mavlink_stream_contract.hpp"', "R344",
             "MAVLink service must consume the generated runtime contract"),
            ("dima::generated::mavlink_streams::kServiceStreamCount", "R344",
             "MAVLink periodic state must be sized by the generated contract"),
            ("configured_streams_", "R344",
             "MAVLink periodic streams must share generated descriptor state"),
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
            ("SubscriptionData<sensor_accel_s>", "R344",
             "MAVLink SYS_STATUS must consume the raw accelerometer topic"),
            ("SubscriptionData<sensor_gyro_s>", "R344",
             "MAVLink SYS_STATUS must consume the raw gyroscope topic"),
            ("SubscriptionData<sensor_mag_s>", "R344",
             "PX4 SCALED_IMU must consume the raw magnetometer topic"),
        ),
        violations,
    )
    rate_policy_path = (
        ROOT / "Dima/modules/mavlink/MavlinkStreamRatePolicy.hpp"
    )
    if rate_policy_path.exists():
        violations.append(Violation(
            rate_policy_path, 1, "R344",
            "a global telemetry ceiling is not PX4 stream architecture; "
            "keep per-stream defaults and intervals",
        ))
    gps_stream_path = ROOT / "Dima/modules/mavlink/MavlinkSensorStreams.cpp"
    require_literals(
        gps_stream_path,
        (
            ("mavlink_msg_gps_raw_int_encode", "R338",
             "QGC GPS telemetry must publish GPS_RAW_INT"),
            ("GPS_FIX_TYPE_NO_GPS", "R338",
             "stale GPS telemetry must distinguish NO_GPS from NO_FIX"),
            ("gps_healthy_ ? gps.satellites_used : UINT8_MAX", "R338",
             "stale GPS telemetry must mark satellite count unavailable"),
            ("gps_sample_valid && gps_status_valid", "R343",
             "GPS health must require one matched receiver/status verdict"),
            ("PX4 calibration_count is a calibration-change counter", "R344",
             "identity calibration must remain a healthy IMU state"),
            ("if (!imu_streamable_", "R344",
             "HIGHRES_IMU availability must follow vehicle_imu, not health bits"),
            ("const bool imu_updated = imu_streamable_", "R344",
             "SCALED_IMU availability must follow vehicle_imu updates"),
            ("latest_vehicle_imu_.delta_velocity_dt != 0U", "R344",
             "MAVLink IMU health must require a valid integration interval"),
            ("latest_vehicle_imu_status_.timestamp", "R344",
             "MAVLink IMU health must consume vehicle_imu_status freshness"),
            ("const bool mag_streamable =", "R344",
             "HIGHRES_IMU must retain the latest vehicle_magnetometer value"),
            ("const bool raw_mag_streamable =", "R344",
             "SCALED_IMU must retain the latest raw sensor_mag value"),
            ("mavlink_msg_scaled_imu_encode", "R344",
             "PX4 raw-sensor SCALED_IMU encoder is missing"),
        ),
        violations,
    )
    gps_stream_text = gps_stream_path.read_text(encoding="utf-8")
    stream_function_bounds = (
        ("bool MavlinkService::send_highres_imu",
         "bool MavlinkService::send_scaled_imu"),
        ("bool MavlinkService::send_scaled_imu",
         "bool MavlinkService::send_gps_raw_int"),
    )
    for function, next_function in stream_function_bounds:
        start = gps_stream_text.find(function)
        end = gps_stream_text.find(next_function, start + 1)
        if start < 0 or end <= start:
            continue
        function_body = gps_stream_text[start:end]
        for forbidden in ("mag_healthy_", "raw_mag_healthy", "fresh("):
            if forbidden in function_body:
                offset = start + function_body.find(forbidden)
                violations.append(Violation(
                    gps_stream_path,
                    gps_stream_text.count("\n", 0, offset) + 1,
                    "R344",
                    "PX4 IMU streams must expose the latest valid mag sample; "
                    "freshness belongs only to SYS_STATUS health",
                ))
    for forbidden in (
            "latest_vehicle_imu_.accel_calibration_count != 0U",
            "latest_vehicle_imu_.gyro_calibration_count != 0U"):
        if forbidden in gps_stream_text:
            violations.append(Violation(
                gps_stream_path, line_for(gps_stream_text, forbidden), "R344",
                "an uncalibrated PX4 identity IMU must not be marked unhealthy",
            ))
    highres_temperature_token = "HIGHRES_IMU_UPDATED_TEMPERATURE"
    if highres_temperature_token in gps_stream_text:
        violations.append(Violation(
            gps_stream_path,
            line_for(gps_stream_text, highres_temperature_token), "R344",
            "HIGHRES_IMU temperature requires vehicle_air_data, not IMU status",
        ))
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkCommands.cpp",
        (
            ("command == MAV_CMD_SET_MESSAGE_INTERVAL", "R344",
             "PX4/QGC SET_MESSAGE_INTERVAL handling is missing"),
            ("command == MAV_CMD_GET_MESSAGE_INTERVAL", "R344",
             "PX4/QGC GET_MESSAGE_INTERVAL handling is missing"),
            ("message_id == MAVLINK_MSG_ID_MESSAGE_INTERVAL", "R344",
             "REQUEST_MESSAGE must support MESSAGE_INTERVAL queries"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkService.cpp",
        (
            ("mavlink_msg_message_interval_encode", "R344",
             "MESSAGE_INTERVAL response encoding is missing"),
            ("stream_contract::find_message(message_id)", "R344",
             "MAVLink request and interval lookup must use the generated contract"),
            ("!contract->interval_configurable", "R344",
             "fixed generated streams must reject interval changes"),
            ("interval_us < -0.00001F", "R344",
             "negative PX4 message intervals must stop a stream"),
            ("std::max(1.0, rounded_interval)", "R344",
             "positive PX4 message intervals must retain the requested value"),
            ("stream_contract::service_index(contract->handler)", "R344",
             "configured interval state must use the generated service index"),
            ("send_contract_message(", "R344",
             "one-shot requests must dispatch through generated handlers"),
            ("stream_contract::find_inbound_message(msg.msgid)", "R344",
             "MAVLink inbound consumer routing must use the generated contract"),
        ),
        violations,
    )
    mavlink_service_text = (
        ROOT / "Dima/modules/mavlink/MavlinkService.cpp"
    ).read_text(encoding="utf-8")
    sensor_stream_text = gps_stream_path.read_text(encoding="utf-8")
    for forbidden_path, forbidden_text, forbidden in (
            (ROOT / "Dima/modules/mavlink/MavlinkService.cpp",
             mavlink_service_text, "normalize_periodic_interval"),
            (gps_stream_path, sensor_stream_text, "requested_due("),
            (ROOT / "Dima/modules/mavlink/MavlinkService.hpp",
             (ROOT / "Dima/modules/mavlink/MavlinkService.hpp").read_text(
                 encoding="utf-8"), "MavlinkStreamRatePolicy")):
        if forbidden in forbidden_text:
            violations.append(Violation(
                forbidden_path, line_for(forbidden_text, forbidden), "R344",
                "PX4 one-shot requests and per-stream rates must not use a "
                "shared telemetry ceiling",
            ))
    gps_driver_path = ROOT / "Dima/drivers/gps/um982/Um982Gps.cpp"
    require_literals(
        gps_driver_path,
        (
            ("publish_receiver_status(arrival_us);", "R338",
             "a detected receiver without fresh GGA must remain visible"),
            ("output.fix_type = sensor_gps_s::FIX_TYPE_NONE;", "R338",
             "receiver-only telemetry must publish NO_FIX"),
            ("output.latitude_deg = unavailable_double;", "R338",
             "receiver-only telemetry must not invent a position"),
            ("receiver_status_ != ReceiverStatus::Offline", "R338",
             "GPS offline errors must be emitted only on a state edge"),
            ("validation_fault_active_", "R343",
              "GPS validation errors must remain edge-triggered"),
            ("kValidationReportIntervalUs", "R343",
             "GPS validation errors must remain rate limited"),
            ("GPS invalid structure=", "R343",
             "GPS final-sample validation reason must remain visible"),
            ("publish_solution_status(output, solution);", "R343",
             "valid GPS samples must publish PX4 estimator status"),
        ),
        violations,
    )
    gps_configuration_path = (
        ROOT / "Dima/drivers/gps/um982/Um982GpsLogging.cpp"
    )
    require_literals(
        gps_configuration_path,
        (("configuration_retry_after_us_ == 0U", "R338",
          "GPS configuration errors must remain deduplicated"),),
        violations,
    )
    gps_log_owners = (
        gps_driver_path,
        gps_configuration_path,
        ROOT / "Dima/drivers/gps/um982/Um982GpsBaud.cpp",
        ROOT / "Dima/drivers/gps/um982/Um982GpsValidation.cpp",
        gps_stream_path,
    )
    for forbidden in (
            "UM982 probing", "UM982 detected", "GPS1 assigned",
            "UM982 configuration valid", "UM982 configuration saved",
            "GPS detected", "GPS data timeout", "Sensor status: GPS",
            "GPS validation recovered", "GPS checks passed",
            "GPS validation failed", "GPS proto crc=", "GPS uart S"):
        owner = find_literal_owner(gps_log_owners, forbidden)
        if owner is not None:
            owner_path, owner_text = owner
            violations.append(Violation(
                owner_path, line_for(owner_text, forbidden), "R338",
                "GPS progress and health logs must not reintroduce chatter",
            ))

    imu_driver_path = (
        ROOT / "Dima/drivers/imu/icm42688p/ICM42688P.cpp"
    )
    require_literals(
        imu_driver_path,
        (
            ("if (restart_fault_active_)", "R344",
             "repeated IMU restart diagnostics must remain edge-triggered"),
            ("healthy_publications_after_fault_", "R344",
             "IMU restart logging must require sustained recovery"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/sensors/imu/VehicleImu.cpp",
        (
            ("validation_fault_active_", "R344",
             "IMU stream validation errors must remain edge-triggered"),
            ("candidate.accel.enabled = accel_device_id_ != 0U", "R344",
             "an uncalibrated accelerometer must use enabled identity correction"),
            ("candidate.gyro.enabled = gyro_device_id_ != 0U", "R344",
             "an uncalibrated gyroscope must use enabled identity correction"),
            ("saved IMU calibration unavailable; using identity calibration", "R344",
             "a stale saved calibration must not suppress vehicle_imu"),
        ),
        violations,
    )
    imu_log_owners = (
        imu_driver_path,
        ROOT / "Dima/drivers/imu/icm42688p/ICM42688PFifo.cpp",
        ROOT / "Dima/modules/sensors/imu/VehicleImu.cpp",
        gps_stream_path,
    )
    for forbidden in (
            "ICM42688P detected", "high-resolution FIFO ready",
            "IMU detected", "IMU data timeout", "IMU validation recovered",
            "IMU path int1="):
        owner = find_literal_owner(imu_log_owners, forbidden)
        if owner is not None:
            owner_path, owner_text = owner
            violations.append(Violation(
                owner_path, line_for(owner_text, forbidden), "R344",
                "IMU progress and periodic health logs must not chatter",
            ))
    require_literals_in_owners(
        mavlink_service_owners,
        (
            ("mavlink_msg_rc_channels_encode", "R335",
             "MAVLink must stream raw RC_CHANNELS"),
            ("rc_stream_active_ = rc_sample_streamable(now) &&", "R336",
             "missing or stale raw RC must stop the QGC stream"),
            ("channels.chancount = channel_count;", "R336",
             "RC_CHANNELS must carry the real valid channel count"),
            ("case stream_contract::InboundHandler::MetadataFtp:", "R337",
             "FILE_TRANSFER_PROTOCOL dispatch is missing"),
            ("metadata_ftp_.handle_message(&msg, hrt_absolute_time())", "R337",
             "FTP requests must retain Runtime time and source routing"),
            ("if (!metadata_ftp_.service(now))", "R337",
             "pending FTP must block lower-priority parameter/log traffic"),
            ("case stream_contract::MessageHandler::ComponentMetadata:", "R337",
             "MAV_CMD_REQUEST_MESSAGE cannot serve COMPONENT_METADATA"),
            ("case stream_contract::MessageHandler::ComponentInformation:", "R337",
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
            ("prepare_parameter_catalogue()", "R336",
             "the generated public parameter catalogue must be active before LIST"),
            ("contract::kMavlinkPublicParameters", "R336",
             "MAVLink parameter visibility must use generated handles"),
            ("fixed_parameter_constraint(param)", "R336",
             "fixed parameter writes must use generated constraints"),
            ("return value == fixed->int32_value;", "R336",
             "generated fixed INT32 writes must be rejected"),
            ("return wire_value == fixed->float_value;", "R336",
             "generated fixed FLOAT writes must be rejected"),
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
            ("param_foreach(&MavlinkParameters::append_used_parameter",
             "R331", "PARAM_REQUEST_LIST must snapshot the used set"),
            ("snapshot_contains_qgc_required_parameters()", "R331",
             "LIST snapshots must contain every generated QGC setup Fact"),
            ("param read failed, index %u retained", "R331",
             "LIST must retain a failed parameter index instead of creating a hole"),
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
            ("param_handle(px4::params::MAV_SYS_ID)", "R336",
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
             "R335", "Commander must arbitrate every calibration command"),
            ("sensor_calibration_request_publication_.publish(", "R335",
             "Commander must dispatch sensor calibration to its worker"),
            ("vehicle_status_.rc_calibration_in_progress = true;", "R335",
             "Commander must publish RC calibration state"),
            ("param_handle(px4::params::NAV_RCL_ACT)", "R336",
             "RC-loss action compatibility parameter has no consumer"),
            ("param_handle(px4::params::NAV_DLL_ACT)", "R336",
             "data-link-loss compatibility parameter has no consumer"),
            ("kRcLossActionDisarm", "R336",
             "RC loss must remain fixed to Disarm"),
            ("kDataLinkLossActionDisabled", "R336",
             "GCS loss must remain disabled"),
        ),
        violations,
    )
    require_literals_in_owners(
        sensor_calibration_owners,
        (
            ("ORB_ID(sensor_calibration_request)", "R335",
             "sensor calibration worker must consume Commander's generated "
             "internal request"),
            ("calibration_request_subscription_.copy(&request)", "R335",
             "sensor calibration worker must drain Commander's internal "
             "request queue"),
        ),
        violations,
    )
    vehicle_command_subscription = re.compile(
        r"uORB::Subscription[A-Za-z0-9_]*"
        r"(?:<[^;{}]+>)?\s+[A-Za-z_][A-Za-z0-9_]*\s*"
        r"\{[^{}]*ORB_ID\(vehicle_command\)",
        re.DOTALL,
    )
    command_subscribers: list[tuple[pathlib.Path, str, re.Match[str]]] = []
    for pattern in ("*.hpp", "*.h", "*.cpp", "*.cc", "*.cxx"):
        for path in sorted((ROOT / "Dima").rglob(pattern)):
            text = path.read_text(encoding="utf-8")
            command_subscribers.extend(
                (path, text, match)
                for match in vehicle_command_subscription.finditer(text)
            )
    commander_header = ROOT / "Dima/modules/safety/Commander.hpp"
    if (len(command_subscribers) != 1 or
            command_subscribers[0][0] != commander_header):
        if not command_subscribers:
            violations.append(Violation(
                commander_header, 1, "R335",
                "Commander must be the sole vehicle_command subscriber",
            ))
        for path, text, match in command_subscribers:
            if path != commander_header or len(command_subscribers) != 1:
                violations.append(Violation(
                    path, line_for(text, match.group(0)), "R335",
                    "vehicle_command may only be subscribed by Commander; "
                    "workers must consume generated internal requests",
                ))
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
        (
            ("assign(Mapping::Flaps, rc_channels_s::FUNCTION_FLAPS);", "R335",
             "RC_MAP_FLAPS must have a production mapping consumer"),
            ("contract::kRcCalibrationParameters", "R336",
             "RC calibration handles must come from the generated contract"),
            ("contract::kRcMappingParameters", "R336",
             "RC mapping handles must come from the generated contract"),
        ),
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
    if "param_find(\"" in rc_update_text:
        violations.append(Violation(
            rc_update_path, line_for(rc_update_text, "param_find(\""), "R336",
            "RC runtime parameter handles must use generated enums/contracts",
        ))

    lock_path = ROOT / "tools/mavlink/mavlink.lock.json"
    try:
        lock = json.loads(lock_path.read_text(encoding="utf-8"))
        dialect = lock["dialect"]
        messages = (
            dialect["messages_from_common"] +
            dialect["inherited_messages"]
        )
        if (not isinstance(dialect["message_count"], int) or
                dialect["message_count"] != len(messages) or
                len(set(messages)) != len(messages)):
            raise ValueError("dialect message count/list is inconsistent")
        forbidden = dialect["forbidden_messages"]
        if (not isinstance(forbidden, list) or not forbidden or
                len(set(forbidden)) != len(forbidden) or
                set(messages) & set(forbidden)):
            raise ValueError("forbidden message contract is inconsistent")
        runtime = lock["runtime"]
        runtime_names = [entry["name"] for entry in runtime["messages"]]
        inbound_names = [entry["name"] for entry in runtime["inbound"]]
        if (len(set(runtime_names)) != len(runtime_names) or
                len(set(inbound_names)) != len(inbound_names) or
                not set(runtime_names + inbound_names).issubset(messages)):
            raise ValueError("runtime message routing is inconsistent")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        violations.append(Violation(
            lock_path, 1, "R334", f"invalid MAVLink lock contract: {error}",
        ))
