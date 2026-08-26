"""Runtime 生命周期、故障所有权、单调时钟与链接布局安全门禁。"""

from __future__ import annotations

import re

from architecture.common import (
    ROOT,
    Violation,
    first_party_sources,
    line_for,
    require_literals,
    MAKE_CONTRACT_PATHS,
    owner_texts,
)


def scan_runtime_contracts(violations: list[Violation]) -> None:
    """以精确源码合同核对模块启停、参数绑定、看门狗、维护与 Safe-Off 生命周期。"""
    requirements = {
        ROOT / "Dima/middleware/parameters/param.h": (
            ("constexpr Param() noexcept", "R050",
             "Param construction must remain side-effect free"),
            ("bool bind()", "R051", "Param bind contract is missing"),
            ("param_set_used(handle());", "R052",
             "Param bind must register parameter use"),
            ("bool param_shutdown(void)", "R053",
             "Parameter core shutdown declaration is missing"),
        ),
        ROOT / "Dima/middleware/parameters/param.cpp": (
            ("bool param_shutdown(void) noexcept", "R054",
             "Parameter core shutdown implementation is missing"),
            ("g_active.reset();", "R055",
             "Parameter shutdown must invalidate the used cache"),
            ("g_unsaved.reset();", "R056",
             "Parameter shutdown must invalidate the unsaved cache"),
        ),
        ROOT / "Dima/middleware/uorb/uORB.cpp": (
            ("uint64_t g_lifecycle_epoch", "R057",
             "uORB lifecycle epoch storage is missing"),
            ("++g_lifecycle_epoch;", "R058",
             "uORB initialize must advance the lifecycle epoch"),
            ("void shutdown() noexcept", "R059",
             "uORB Runtime shutdown is missing"),
            ("if (newest == 0U)", "R061",
             "uORB must reject empty generation-zero slots"),
            ("generation == 0U || generation > newest", "R062",
             "uORB queued subscriptions must recover from stale generations"),
        ),
        ROOT / "Dima/middleware/uorb/uORB.hpp": (
            ("synchronize_epoch()", "R060",
             "uORB endpoint epoch synchronization is missing"),
        ),
        ROOT / "Dima/middleware/work_queue/WorkQueue.cpp": (
            ("dima::platform::SignalHandle signal", "R063",
             "WorkQueue globals must not own destructed Signal objects"),
            ("g_owner_task", "R064",
             "WorkQueue Runtime owner tracking is missing"),
            ("bool work_queue_shutdown() noexcept", "R065",
             "WorkQueue shutdown contract is missing"),
            ("tasks.destroy(queue.task)", "R066",
             "WorkQueue shutdown must synchronously destroy workers"),
            ("const wq_config_t lp_default{\"wq:lp_default\", 2U, 4096U, false};",
             "R228", "MAVLink/Parameter/Log WorkQueue stack regressed below 4 KiB"),
        ),
        ROOT / "Dima/platform/api/TaskRuntime.hpp": (
            ("virtual bool destroy(TaskHandle handle) noexcept", "R067",
             "TaskRuntime destroy must report failure"),
        ),
        ROOT / "Dima/platform/freertos/Backend.cpp": (
            ("native == xTaskGetCurrentTaskHandle()", "R068",
             "TaskRuntime must reject deleting the current task"),
            ("kTaskStackPoolBytes = 48U * 1024U", "R229",
             "fixed task stack pool must retain the 48 KiB budget"),
        ),
        ROOT / "Dima/rover/ApplicationContext.cpp": (
            ("bool ApplicationContext::shutdown() noexcept", "R069",
             "Application Runtime shutdown is missing"),
            ("if (!services_.console.shutdown())", "R070",
             "Application Runtime must release the Console frontend"),
            ("if (!px4::work_queue_shutdown())", "R071",
             "Application Runtime must release WorkQueue"),
            ("sbus_rc_.state() ==", "R217",
             "Application Runtime ignores SBUS UART restore failure"),
            ("serial_config_.state() ==", "R196",
             "Application Runtime ignores serial backend reset failure"),
            ("Board serial configuration invalid; RC input inhibited", "R196",
             "invalid serial parameters must preserve USB/QGC recovery"),
        ),
        ROOT / "Dima/platform/stm32h7/serial/UartTimestampedRxEndpoint.cpp": (
            ("kDmaBufferSize = 64U", "R072",
             "SBUS DMA buffer size contract changed"),
            ("kReceiveRingCapacity = 256U", "R073",
             "SBUS CPU Ring capacity contract changed"),
            ("g_receive_ring", "R074",
             "SBUS CPU-only handoff Ring is missing"),
            ("reset_receive_epoch()", "R075",
             "SBUS receive epoch reset is missing"),
            ("restore_normal_uart", "R182",
             "SBUS release must restore the pre-takeover UART state"),
            ("receive_error_flags_", "R183",
             "SBUS UART error detail is missing"),
            ("reset_configuration", "R196",
             "Runtime serial configuration reset is missing"),
            ("normal_advanced_init_", "R193",
             "SBUS must preserve the normal UART advanced configuration"),
            ("normal_fifo_mode_", "R199",
             "SBUS must preserve the normal UART FIFO mode"),
            ("normal_tx_fifo_threshold_", "R200",
             "SBUS must preserve the normal UART TX FIFO threshold"),
            ("normal_rx_fifo_threshold_", "R201",
             "SBUS must preserve the normal UART RX FIFO threshold"),
            ("normal_rx_pin_", "R202",
             "SBUS must preserve the pre-takeover RX GPIO state"),
            ("restore_rx_pin(normal_rx_pin_)", "R203",
             "SBUS release must restore the pre-takeover RX GPIO state"),
            ("bool stop() noexcept override", "R194",
             "SBUS stop must report UART restoration failure"),
            ("if (restored) {", "R218",
             "SBUS restore failure discards the retry context"),
        ),
        ROOT / "Dima/platform/stm32h7/serial/UartResources.cpp": (
            ("DIMA_STM32_SERIAL_PORT_LIST", "R196",
             "STM32 serial mapping must come from the board generator"),
            ("UART_ADVFEATURE_RXINV_ENABLE", "R192",
             "generic UART line configuration must support RX inversion"),
            ("UART_WORDLENGTH_9B", "R205",
             "generic UART line configuration must support 8-bit parity"),
        ),
        ROOT / "Dima/platform/stm32h7/serial/UartIrqRouter.cpp": (
            ("UART5_IRQHandler", "R196",
             "UART5 must have an IRQ handler when exposed as SERIAL5"),
        ),
        ROOT / "Dima/platform/stm32h7/serial/SerialPorts.cpp": (
            ("configure_line", "R196",
             "normal serial baud configuration backend is missing"),
        ),
        ROOT / "Dima/middleware/parameters/definitions/rc_input_mapping_params.c": (
            ("* @value 0 Disabled", "R195",
             "RC_INPUT_PROTO disabled value is missing"),
            ("PARAM_DEFINE_INT32(RC_INPUT_PROTO, 2);", "R196",
             "SBUS must remain the default RC input protocol"),
        ),
        ROOT / "Dima/modules/serial/SerialConfig.hpp": (
            ("DIMA_BOARD_SERIAL_PARAMETER_LIST", "R196",
             "SerialConfig parameter members must come from the board table"),
            ("rc_input_port() const", "R196",
             "serial function owner must resolve the RC port"),
        ),
        ROOT / "Dima/modules/serial/SerialConfig.cpp": (
            ("serial_baud_supported", "R196",
             "normal baud values must use the generated whitelist"),
            ("serial_function_supported", "R196",
             "serial function values must use the generated whitelist"),
            ("generated::kTargetBaudrate", "R235",
             "GPS baudrate must come from the generated UM982 contract"),
            ("rc_owner_count > 1U", "R196",
             "multiple RC serial owners must fail closed"),
            ("backend_.configure_line", "R196",
             "SerialConfig must apply normal 8N1 settings"),
        ),
        ROOT / "Dima/drivers/magnetometer/dronecan_mag2/"
               "DroneCanMag2Configuration.cpp": (
            ("armed_.begin_maintenance()", "R235",
             "DroneCAN reconfiguration must atomically block arming"),
            ("armed_.end_maintenance()", "R235",
             "DroneCAN reconfiguration must release its arming interlock"),
            ("same_transport_configuration(", "R235",
             "DroneCAN restart decisions must remain transport-only"),
        ),
        ROOT / "Dima/drivers/magnetometer/dronecan_mag2/DroneCanMag2.cpp": (
            ("DroneCAN mag not detected", "R235",
             "DroneCAN source diagnostics must remain bounded and visible"),
            ("DroneCAN CAN ov=", "R235",
             "CAN overrun/error/recovery diagnostics must use one key record"),
            ("DroneCAN DNA storage error=", "R349",
             "dynamic allocation persistence failures must remain visible"),
            ("sensor_mag_publication_.publish(sample)", "R235",
             "the DroneCAN driver must publish the raw sensor_mag topic"),
        ),
        ROOT / "Dima/modules/sensors/magnetometer/"
               "VehicleMagnetometer.cpp": (
            ("correction_for_device(", "R235",
             "vehicle_magnetometer must select identity or matching calibration"),
            ("return Calibration{};", "R235",
             "missing or mismatched magnetometer calibration must use identity"),
            ("vehicle_magnetometer_publication_.publish(output)", "R235",
             "the independent frontend must publish vehicle_magnetometer"),
            ("SensorCalibration already owns the arming interlock", "R235",
             "calibration parameter application must not reacquire its interlock"),
            ("calibration_parameter_update_applied(", "R235",
             "magnetometer calibration must acknowledge parameter generations"),
            ("mag_calibration_matches(", "R235",
             "magnetometer calibration must verify the applied correction"),
        ),
        ROOT / "Dima/modules/sensors/calibration/"
               "SensorCalibrationParameters.cpp": (
            ("vehicle_magnetometer_frontend_.", "R235",
             "sensor calibration must handshake with VehicleMagnetometer"),
            ("parameter_snapshot_.type == Type::Mag", "R235",
             "magnetometer rollback must wait for the sensor frontend"),
            ("Keep the same arming interlock across rollback", "R235",
             "calibration rollback must retain the arming interlock"),
        ),
        ROOT / "Dima/modules/sensors/calibration/SensorCalibration.hpp": (
            ("kMagOrientationSamples = 25U", "R235",
             "PX4 magnetometer orientation detection must remain bounded"),
            ("kMagSideSamples = 40U", "R235",
             "PX4 six-side magnetometer calibration needs 40 points per side"),
            ("kMagSideSamples * 6U", "R235",
             "PX4 magnetometer calibration must retain 240 total points"),
            ("kMagSideCollectionUs = 7000000ULL", "R235",
             "PX4 magnetometer side collection window must remain seven seconds"),
            ("kMagMinimumRotationRad = 0.5", "R235",
             "PX4 magnetometer calibration must require real rotation"),
            ("float x[kMagMinimumSamples]", "R235",
             "magnetometer calibration must retain accepted points for spacing checks"),
            ("kMagMinimumScale = 0.1", "R235",
             "magnetometer calibration scale must match PX4 parameter metadata"),
            ("kMagMaximumScale = 3.0", "R235",
             "magnetometer calibration scale must match the sensor frontend"),
        ),
        ROOT / "Dima/modules/sensors/calibration/SensorCalibrationMag.cpp": (
            ("[cal] %s orientation detected", "R235",
             "QGC magnetometer orientation transition token is missing"),
            ("[cal] %s side done, rotate to a different side", "R235",
             "QGC magnetometer side-complete token is missing"),
            ("minimum_distance", "R235",
             "PX4 magnetometer duplicate-point rejection is missing"),
            ("static_cast<double>(sensor_gyro_.x) * dt_s", "R235",
             "magnetometer rotation must integrate signed gyro motion"),
            ("std::fabs(mag_rotation_integral_[0])", "R235",
             "magnetometer rotation must test absolute net angle"),
            ("mag_side_samples_ < kMagSideSamples", "R235",
             "magnetometer sides must not complete before 40 accepted points"),
        ),
        ROOT / "Dima/modules/sensors/calibration/SensorCalibration.cpp": (
            ("px4::wq_configurations::lp_default", "R235",
             "QGC calibration must run outside realtime work queues so its "
             "STATUSTEXT protocol can be formatted"),
            ("PX4_INFO_RAW(\"[cal] calibration started: 2 %s\"", "R235",
             "QGC calibration start token must bypass normal log filtering"),
            ("PX4_INFO_RAW(\"[cal] pending: back front left right up down\"",
             "R235", "QGC six-side pending token is missing"),
            ("px4_log_raw(_PX4_LOG_LEVEL_ERROR", "R235",
             "QGC calibration terminal failures must bypass log filtering"),
        ),
        ROOT / "Dima/drivers/gps/um982/Um982GpsBaud.cpp": (
            ("9600U, 19200U, 38400U", "R235",
             "GPS auto baud must retain the official UM982 probe range"),
            ("230400U, 460800U, 921600U", "R235",
             "GPS auto baud must retain all high-rate UM982 candidates"),
            ("last_valid_data_arrival_us_", "R235",
             "GPS baud detection must use a valid received data frame"),
        ),
        ROOT / "Dima/drivers/gps/um982/Um982Gps.hpp": (
            ("kAuxiliaryFreshnessUs = 500000U", "R235",
             "ten-hertz UM982 auxiliary messages need a bounded jitter margin"),
            ("kValidationReportIntervalUs = 30000000ULL", "R235",
             "GPS validation QGC errors must remain rate limited"),
        ),
        ROOT / "Dima/drivers/gps/um982/Um982GpsLogging.cpp": (
            ("Um982MessageContract.hpp", "R235",
             "UM982 output configuration must use its generated contract"),
            ("generated::kMessageContracts", "R235",
             "UM982 output configuration bypasses its generated message list"),
            ("entry.expected_period_s", "R235",
             "UM982 output validation ignores generated PX4 periods"),
            ("UNLOG COM%u %s", "R235",
             "UM982 output removal must target the detected receiver COM"),
            ("%s COM%u %s", "R235",
             "UM982 output configuration must target the detected receiver COM"),
            ("entry.command_name", "R235",
             "UM982 output names must come from the generated contract"),
            ("entry.period_s", "R235",
             "UM982 output periods must come from the generated contract"),
            ("CONFIG COM%u %lu 8 N 1", "R235",
             "UM982 target baud must configure the detected receiver COM"),
            ("UNILOGLIST", "R235",
             "UM982 output changes must compare the active log list"),
            ("SAVECONFIG", "R235",
             "UM982 changed configuration must support one-time persistence"),
            ("armed_.begin_maintenance()", "R235",
             "UM982 receiver writes must atomically block arming"),
            ("armed_.end_maintenance()", "R235",
             "UM982 receiver writes must release the arming interlock"),
        ),
        ROOT / "Dima/drivers/gps/um982/Um982Protocol.cpp": (
            ("Um982MessageContract.hpp", "R235",
             "UM982 protocol must use its generated message contract"),
            ("parse_px4_gst_float", "R235",
             "UM982 GST empty-field handling differs from fixed PX4"),
            ("generated::kMessageContracts", "R235",
             "UM982 UNILOGLIST parsing bypasses the generated contract"),
        ),
        ROOT / "tools/gps/generate_um982_contract.py": (
            ("EXPECTED_MESSAGES_SHA256", "R235",
             "UM982 generator must pin the PX4-derived 10 Hz product contract"),
            ("0b9695881bd1e8f830ab4538ab3acc0050019eba", "R235",
             "UM982 generator lost its fixed PX4 provenance"),
            ("const char *command_name;", "R235",
             "UM982 generated commands must retain a runtime COM slot"),
            ("const char *period_s;", "R235",
             "UM982 generated periods must remain exact command text"),
            ("kTargetBaudrate", "R235",
             "UM982 generator must emit the fixed product baudrate"),
        ),
        ROOT / "make/project.mk": (
            ("UM982_CONTRACT_GENERATOR := tools/gps/generate_um982_contract.py",
             "R235", "UM982 message contract generator is not in the build"),
            ("UM982_CONTRACT_MANIFEST := Dima/drivers/gps/um982/um982_messages.json",
             "R235", "UM982 message manifest is not in the build"),
            ("$(UM982_GENERATED_STAMP) architecture-ready", "R235",
             "project objects can compile before the UM982 contract exists"),
        ),
        ROOT / "Dima/platform/stm32h7/serial/"
               "UartDuplexDmaEndpoint.cpp": (
            ("kErrorRecoveryNotificationIntervalUs", "R235",
             "GPS wrong-baud UART recovery must remain rate limited"),
            ("receive_fault_, true", "R235",
             "UART errors must leave one bounded recovery pending"),
        ),
        ROOT / "Dima/drivers/rc/sbus/SbusRc.cpp": (
            ("configuration.baudrate = 100000U", "R204",
             "SBUS baud rate must remain 100000 bit/s"),
            ("configuration.parity = dima::platform::SerialParity::Even",
             "R207", "SBUS must use even parity"),
            ("configuration.stop_bits = dima::platform::SerialStopBits::Two",
             "R206", "SBUS must use two stop bits"),
            ("configuration.tx_enabled = false", "R208",
             "SBUS UART must remain RX-only"),
            ("configuration.rx_inverted = true", "R192",
             "SBUS must request RX inversion"),
            ("configuration.rx_pull = dima::platform::SerialRxPull::Down",
             "R181", "inverted SBUS must request a pull-down"),
            ("if (protocol == 0)", "R209",
             "disabled RC protocol must remain a normal lifecycle state"),
            ("schedule_signal_timeout()", "R210",
             "SBUS signal loss logging must follow the RC timeout"),
            ("signal lost last_frame_us=", "R211",
             "SBUS signal loss state log is missing"),
            ("SBUS release failed; UART normal configuration not restored",
             "R219", "SBUS module does not surface UART restore failure"),
            ("serial_assignments_.rc_input_port()", "R196",
             "SBUS port must come from SERIALx_FUNCTION ownership"),
            ("dima::board::serial_port(port)", "R196",
             "SBUS must reject ports absent from the board manifest"),
            ("consecutive_healthy_frames_", "R220",
             "SBUS lock does not count consecutive healthy frames"),
            ("signal_locked_ = true;", "R220",
             "SBUS lock transition is missing"),
        ),
        ROOT / "Dima/drivers/rc/sbus/SbusRc.hpp": (
            ("kRequiredLockFrames = 3U", "R220",
             "SBUS must require three consecutive healthy frames"),
        ),
        ROOT / "Dima/modules/rc/RCUpdate.hpp": (
            ("kRecoveryStableUs = 100000ULL", "R221",
             "RC recovery must remain stable for 100 ms"),
        ),
        ROOT / "Dima/modules/rc/RcManualInput.hpp": (
            ("kSwitchDebounceUs = 200000ULL", "R222",
             "RC Arm/Kill switches must debounce for 200 ms"),
            ("kRequiredStableSamples = 2U", "R222",
             "RC Arm/Kill debounce must require repeated samples"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

    for path in (
            ROOT / "Dima/drivers/gps/um982/Um982Protocol.cpp",
            ROOT / "Dima/drivers/gps/um982/Um982GpsLogging.cpp"):
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        for forbidden in ("kRequiredLogs[", "kRequiredCommands[",
                          "command_body"):
            if forbidden in text:
                violations.append(Violation(
                    path, line_for(text, forbidden), "R235",
                    "hand-written UM982 message lists must remain removed",
                ))

    mag_driver_owners = (
        ROOT / "Dima/drivers/magnetometer/dronecan_mag2/DroneCanMag2.hpp",
        ROOT / "Dima/drivers/magnetometer/dronecan_mag2/DroneCanMag2.cpp",
        ROOT / "Dima/drivers/magnetometer/dronecan_mag2/"
               "DroneCanMag2Configuration.cpp",
    )
    for mag_driver_path, mag_driver_text in owner_texts(mag_driver_owners):
        for forbidden in ("CAL_MAG0_", "SENS_MAG_RATE",
                          "vehicle_magnetometer"):
            if forbidden in mag_driver_text:
                violations.append(Violation(
                    mag_driver_path,
                    line_for(mag_driver_text, forbidden), "R235",
                    "the DroneCAN driver must publish only raw sensor_mag; "
                    "calibration belongs to VehicleMagnetometer",
                ))

    sensor_frontends = (
        ROOT / "Dima/modules/sensors/imu/VehicleImu.cpp",
        ROOT / "Dima/modules/sensors/magnetometer/VehicleMagnetometer.cpp",
    )
    for frontend_path, frontend_text in owner_texts(sensor_frontends):
        for forbidden in ("armed_.begin_maintenance()",
                          "armed_.end_maintenance()"):
            if forbidden in frontend_text:
                violations.append(Violation(
                    frontend_path, line_for(frontend_text, forbidden), "R235",
                    "PX4 sensor frontends must apply calibration while disarmed "
                    "without reacquiring SensorCalibration's interlock",
                ))

    calibration_parameters_path = (
        ROOT / "Dima/modules/sensors/calibration/"
               "SensorCalibrationParameters.cpp")
    if calibration_parameters_path.is_file():
        calibration_parameters_text = calibration_parameters_path.read_text(
            encoding="utf-8")
        for function, next_function in (
                ("void SensorCalibration::begin_wait_for_apply",
                 "void SensorCalibration::process_wait_for_apply"),
                ("bool SensorCalibration::begin_rollback",
                 "void SensorCalibration::finish_rollback")):
            start = calibration_parameters_text.find(function)
            end = calibration_parameters_text.find(next_function, start + 1)
            if start >= 0 and end > start:
                function_body = calibration_parameters_text[start:end]
                if "release_interlock();" in function_body:
                    release_offset = start + function_body.find(
                        "release_interlock();")
                    violations.append(Violation(
                        calibration_parameters_path,
                        calibration_parameters_text.count(
                            "\n", 0, release_offset) + 1,
                        "R235",
                        "sensor calibration must retain its arming interlock "
                        "until apply or rollback is acknowledged",
                    ))

    calibration_protocol_paths = (
        ROOT / "Dima/modules/sensors/calibration/SensorCalibration.cpp",
        ROOT / "Dima/modules/sensors/calibration/SensorCalibrationAccel.cpp",
        ROOT / "Dima/modules/sensors/calibration/SensorCalibrationMag.cpp",
        ROOT / "Dima/modules/sensors/calibration/"
               "SensorCalibrationParameters.cpp",
    )
    filtered_calibration_log = re.compile(
        r"PX4_(?:INFO|WARN|ERR|DEBUG)\s*\(\s*\"\[cal\]"
    )
    for calibration_path, calibration_text in owner_texts(
            calibration_protocol_paths):
        for match in filtered_calibration_log.finditer(calibration_text):
            violations.append(Violation(
                calibration_path,
                calibration_text.count("\n", 0, match.start()) + 1,
                "R235",
                "QGC [cal] protocol tokens must use the unfiltered raw log path",
            ))

    calibration_owner_path = (
        ROOT / "Dima/modules/sensors/calibration/SensorCalibration.cpp")
    if calibration_owner_path.is_file():
        calibration_owner_text = calibration_owner_path.read_text(
            encoding="utf-8")
        if "px4::wq_configurations::sensors" in calibration_owner_text:
            violations.append(Violation(
                calibration_owner_path,
                line_for(calibration_owner_text,
                         "px4::wq_configurations::sensors"),
                "R235",
                "QGC calibration logging cannot run on the realtime sensor "
                "work queue",
            ))

    mag_calibration_path = (
        ROOT / "Dima/modules/sensors/calibration/SensorCalibrationMag.cpp")
    if mag_calibration_path.is_file():
        mag_calibration_text = mag_calibration_path.read_text(encoding="utf-8")
        for forbidden in (
                "kMagSampleTimeoutUs",
                "std::fabs(static_cast<double>(sensor_gyro_.x)) * dt_s"):
            if forbidden in mag_calibration_text:
                violations.append(Violation(
                    mag_calibration_path,
                    line_for(mag_calibration_text, forbidden),
                    "R235",
                    "magnetometer calibration diverges from PX4 signed-angle "
                    "and seven-second side collection semantics",
                ))

    for path in first_party_sources():
        text = path.read_text(encoding="utf-8")
        if "DIMA_SBUS_INV" in text:
            violations.append(Violation(
                path, line_for(text, "DIMA_SBUS_INV"), "R197",
                "manual SBUS inversion control must remain removed",
            ))

    sbus_backend_owners = (
        ROOT / "Dima/platform/stm32h7/serial/UartTimestampedRxEndpoint.cpp",
        ROOT / "Dima/platform/stm32h7/serial/UartResources.cpp",
        ROOT / "Dima/platform/stm32h7/serial/UartIrqRouter.cpp",
        ROOT / "Dima/platform/stm32h7/serial/SerialPorts.cpp",
    )
    for sbus_backend, text in owner_texts(sbus_backend_owners):
        for token in ("PX4_INFO", "PX4_WARN", "PX4_ERR", "PX4_DEBUG",
                      "write_module(", "printf(", "console_.write"):
            if token in text:
                violations.append(Violation(
                    sbus_backend, line_for(text, token), "R198",
                    "SBUS ISR/backend path performs formatted USB logging",
                ))

    param_header = ROOT / "Dima/middleware/parameters/param.h"
    if param_header.is_file():
        text = param_header.read_text(encoding="utf-8")
        constructor = re.search(
            r"constexpr\s+Param\(\)\s+noexcept\s*\{(.*?)\n\s*\}"
            r"\s*bool\s+bind\(\)", text, re.DOTALL,
        )
        if constructor is None or any(
                token in constructor.group(1)
                for token in ("ParamTraits<T, p>::get", "param_set_used")):
            violations.append(Violation(
                param_header, line_for(text, "constexpr Param() noexcept"),
                "R076", "Param constructor touches the Parameter Core",
            ))


def scan_fault_ownership(violations: list[Violation]) -> None:
    """限制 panic/复位/故障写入只由诊断和平台组合根拥有，防止模块越权停机。"""
    allowed = {
        "Boards/H743/Inc/boot_diagnostics_store.h",
        "Boards/H743/Src/boot_diagnostics_store.c",
        "Bootloader/Src/main.c",
    }
    for path in first_party_sources():
        relative = path.relative_to(ROOT).as_posix()
        if relative in allowed:
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if ("boot_diagnostics_store.h" in line or
                    "dima_boot_diagnostics_store_" in line):
                violations.append(Violation(
                    path, line_number, "R080",
                    "Application-side code depends on diagnostics Flash store",
                ))

    require_literals(
        ROOT / "Boards/H743/Src/boot_diagnostics.c",
        (
            ("DIMA_BOOT_DIAGNOSTICS_CAPTURE_VALID", "R081",
             "Fault capture valid marker is missing"),
            ("__DMB();", "R082", "Fault capture DMB is missing"),
            ("__DSB();", "R083", "Fault capture DSB is missing"),
            ("__ISB();", "R084", "Fault capture ISB is missing"),
            ("NVIC_SystemReset();", "R085",
             "Fault capture must reset immediately"),
        ), violations,
    )
    require_literals(
        ROOT / "Bootloader/Makefile",
        (("Boards/H743/Src/boot_diagnostics_store.c", "R086",
          "MCUboot must own diagnostics Flash persistence"),),
        violations,
    )

    for project_make, text in owner_texts(MAKE_CONTRACT_PATHS):
        if "Boards/H743/Src/boot_diagnostics_store.c" in text:
            violations.append(Violation(
                project_make,
                line_for(text, "Boards/H743/Src/boot_diagnostics_store.c"),
                "R087", "Application build links diagnostics Flash store",
            ))


def scan_clock_contract(violations: list[Violation]) -> None:
    """要求所有运行期超时依赖单调 HRT，并拒绝绕过平台时钟的 HAL tick 所有者。"""
    requirements = {
        ROOT / "Core/Inc/stm32h7xx_hal_conf.h": (
            ("#define HSE_VALUE    (8000000UL)", "R090",
             "HSE contract must remain 8 MHz"),
        ),
        ROOT / "Core/Src/main.c": (
            ("RCC_OscInitStruct.PLL.PLLM = 2;", "R091",
             "PLL1 M divider changed"),
            ("RCC_OscInitStruct.PLL.PLLN = 240;", "R092",
             "PLL1 N multiplier changed"),
            ("RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;", "R093",
             "CPU clock divider changed"),
            ("RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;", "R094",
             "HCLK divider changed"),
            ("RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;", "R095",
             "APB1 divider changed"),
        ),
        ROOT / "Dima/platform/stm32h7/system/Clock.cpp": (
            ("kExpectedTimerInputClockHz = 240000000U", "R096",
             "TIM2 input clock contract must remain 240 MHz"),
            ("kTimerFrequencyHz = 1000000U", "R097",
             "TIM2 HRT must remain 1 MHz"),
            ("pclk * 2U", "R098",
             "TIM2 APB prescaler doubling rule is missing"),
        ),
        ROOT / "Core/Src/stm32h7xx_it.c": (
            ("void SysTick_Handler(void)", "R099",
             "strong shared SysTick handler is missing"),
            ("HAL_IncTick();", "R100",
             "SysTick no longer advances the HAL tick"),
            ("xPortSysTickHandler();", "R101",
             "SysTick no longer advances FreeRTOS"),
            ("void TIM2_IRQHandler(void)", "R102",
             "strong TIM2 HRT handler is missing"),
            ("dima_hrt_overflow_isr();", "R103",
             "TIM2 handler no longer advances HRT overflow"),
        ),
        ROOT / "Dima/platform/freertos/FreeRTOSConfig.h": (
            ("#define configUSE_TICKLESS_IDLE                  0", "R104",
             "tickless idle must remain disabled"),
        ),
        ROOT / "H743_FreeRTOS.ioc": (
            ("RCC.HSE_VALUE=8000000", "R105", "CubeMX HSE is not 8 MHz"),
            ("RCC.SYSCLKFreq_VALUE=480000000", "R106",
             "CubeMX SYSCLK is not 480 MHz"),
            ("RCC.HCLKFreq_Value=240000000", "R107",
             "CubeMX HCLK is not 240 MHz"),
            ("RCC.APB1Freq_Value=120000000", "R108",
             "CubeMX APB1 is not 120 MHz"),
            ("RCC.Tim2OutputFreq_Value=240000000", "R109",
             "CubeMX TIM2 input is not 240 MHz"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

    clock_contract_owners = owner_texts((
        ROOT / "Core/Src/main.c",
        ROOT / "Core/Src/stm32h7xx_it.c",
        ROOT / "Dima/platform/stm32h7/system/Clock.cpp",
        *MAKE_CONTRACT_PATHS,
    ))
    for path, text in clock_contract_owners:
        for line_number, line in enumerate(text.splitlines(), 1):
            if "TIM12" in line or "htim12" in line:
                violations.append(Violation(
                    path, line_number, "R110",
                    "TIM12 must remain released from the system timebase",
                ))

def scan_linker_contract(violations: list[Violation]) -> None:
    """核对应用向量、诊断区、DMA/heap/orb 段及 MCUboot 槽位的单一布局合同。"""
    require_literals(
        ROOT / "Linker/STM32H743VITx_MCUBOOT_APP.ld",
        (
            ("FLASH (rx)         : ORIGIN = 0x08040400", "R130",
             "Application FLASH origin changed"),
            (".dima_ramfunc", "R131", "RAM function section is missing"),
            (".dima_dma ORIGIN(RAM_DMA)", "R132",
             "DMA section ownership is missing"),
            (".dima_heap (NOLOAD)", "R133", "platform heap section is missing"),
            (".dima_task_pool (NOLOAD)", "R134",
             "task pool section is missing"),
            (".dima_boot_diag (NOLOAD)", "R135",
             "D3 diagnostics section is missing"),
            ("ASSERT(ADDR(.dima_dma) == 0x30040000", "R136",
             "DMA address assertion is missing"),
            ("ASSERT(ADDR(.dima_boot_diag) == 0x38000000", "R137",
             "D3 diagnostics address assertion is missing"),
        ),
        violations,
    )
