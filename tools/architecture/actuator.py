"""主动执行器所有权、物理 PWM Safe-Off 与 Rover 输出链架构门禁。"""

from __future__ import annotations

import re

from architecture.common import (
    ROOT,
    Violation,
    first_party_sources,
    line_for,
    require_literals,
    require_make_source_paths,
    strip_c_comments,
)


def scan_active_actuator_contract(violations: list[Violation]) -> None:
    """限制 HAL/板级 PWM 调用、差速控制器和六通道能力只能出现在显式所有者中。"""
    allowed_motor_calls = {
        "Boards/H743/Inc/motor_pwm.h",
        "Boards/H743/Src/motor_pwm.c",
        "Dima/platform/stm32h7/pwm/ActuatorPwm.cpp",
    }
    allowed_rover_differential_owners = {
        "Dima/rover/ApplicationContext.cpp",
        "Dima/rover/ApplicationContext.hpp",
        "Dima/rover/control/RoverDifferential.cpp",
        "Dima/rover/control/RoverDifferential.hpp",
    }
    allowed_actuator_pwm_owners = {
        "Dima/platform/api/ActuatorPwmLimits.h",
        "Dima/platform/api/ActuatorPwm.hpp",
        "Dima/platform/api/Services.hpp",
        "Dima/platform/stm32h7/pwm/ActuatorPwm.cpp",
        "Dima/platform/stm32h7/HardwareServices.hpp",
        "Dima/modules/boot_health/BootHealthService.cpp",
        "Dima/modules/motor/MotorOutput.cpp",
        "Dima/modules/motor/MotorOutputFrames.cpp",
        "Dima/modules/motor/MotorOutput.hpp",
        "Dima/modules/motor/MotorOutputParameters.cpp",
        "Dima/modules/safety/CommanderSafety.cpp",
    }
    for path in first_party_sources():
        relative = path.relative_to(ROOT).as_posix()
        is_dima_source = relative.startswith("Dima/")
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            board_call = re.search(
                r"\bboard_motor_pwm_(start|stop|write|started)\s*\(", line,
            )
            board_call_allowed = relative in allowed_motor_calls or (
                relative == "Boards/H743/Src/board_init.c" and
                board_call is not None and board_call.group(1) == "stop"
            )
            if board_call is not None and not board_call_allowed:
                violations.append(Violation(
                    path, line_number, "R120",
                    "board motor PWM escaped the board safe-off or STM32 "
                    "capability owners",
                ))
            if (relative != "Boards/H743/Src/motor_pwm.c" and
                    re.search(
                        r"\bHAL_TIM(?:Ex)?_PWMN?_(?:Start|Stop)\s*\(", line,
                    )):
                violations.append(Violation(
                    path, line_number, "R121",
                    "PWM start/stop is outside the board motor backend",
                ))
            if (is_dima_source and re.search(
                    r"\b(?:Mixer|MixingOutput|FunctionMotors|ActuatorOutput)\b",
                    line)):
                violations.append(Violation(
                    path, line_number, "R122",
                    "an unauthorized actuator consumer is present",
                ))
            if (is_dima_source and
                    "RoverDifferential" in line and
                    relative not in allowed_rover_differential_owners):
                violations.append(Violation(
                    path, line_number, "R122",
                    "RoverDifferential escaped the Rover control boundary",
                ))
            if (is_dima_source and
                    "ActuatorPwm" in line and
                    relative not in allowed_actuator_pwm_owners):
                violations.append(Violation(
                    path, line_number, "R126",
                    "six-channel PWM capability escaped its platform and "
                    "Rover output owners",
                ))

    requirements = {
        ROOT / "Boards/H743/Src/board_init.c": (
            ("board_motor_pwm_stop() != BOARD_MOTOR_PWM_APPLIED", "R161",
             "board initialization must establish motor PWM safe-off"),
        ),
        ROOT / "Boards/H743/Src/platform_composition.cpp": (
            ("&stm32h7::actuator_pwm(),", "R162",
             "platform composition does not inject actuator PWM"),
            ("stm32h7::independent_watchdog(),", "R183",
             "platform composition does not inject the IWDG capability"),
        ),
        ROOT / "Dima/platform/api/Boot.hpp": (
            ("class IndependentWatchdog", "R183",
             "platform API is missing the narrow IWDG capability"),
        ),
        ROOT / "Dima/platform/api/Services.hpp": (
            ("IndependentWatchdog &watchdog;", "R183",
             "platform Services do not expose the IWDG capability"),
        ),
        ROOT / "Dima/platform/common/Flash.cpp": (
            ("if (flash_busy_ || maintenance_busy_)", "R183",
             "Runtime maintenance does not block Arm requests"),
            ("bool ArmedFlashCoordinator::begin_maintenance()", "R183",
             "Runtime maintenance has no long-lived arming interlock"),
        ),
        ROOT / "Dima/rover/ApplicationContext.cpp": (
            ("boot_health_(services.boot_control, services.clock, "
             "maintenance_)", "R163",
             "BootHealth is not bound to the Runtime maintenance contract"),
            ("module_manager_.start(motor_output_)", "R164",
             "Application Runtime does not start MotorOutput"),
            ("module_manager_.stop(motor_output_)", "R165",
             "Application Runtime does not stop MotorOutput"),
            ("motor_output_.safe_off_confirmed()", "R166",
             "MotorOutput lifecycle does not verify physical safe-off"),
        ),
        ROOT / "Dima/rover/ApplicationContext.hpp": (
            ("dima::modules::motor::MotorOutput motor_output_;", "R167",
             "ApplicationContext does not own MotorOutput"),
        ),
        ROOT / "Dima/rover/control/RoverDifferential.hpp": (
            ("#include \"rover/DifferentialDrive.hpp\"", "R215",
             "RoverDifferential does not consume the pure Rover library"),
            ("dima::lib::rover::DifferentialDrive drive_{};", "R216",
             "DifferentialDrive escaped its pure algorithm namespace"),
        ),
        ROOT / "Dima/modules/motor/MotorOutput.hpp": (
            ("px4::params::COM_ACT_LOSS_T", "R168",
             "MotorOutput is missing the actuator command timeout"),
            ("hard_safe_inhibit_observed_", "R180",
             "MotorOutput is missing the asynchronous hard-safe latch"),
        ),
        ROOT / "Dima/modules/motor/MotorOutputSafety.cpp": (
            ("timestamp > safety_.actuator_armed.timestamp", "R175",
             "positive safety recovery does not require a newer snapshot"),
            ("now_us - actuator_motors_.timestamp <= timeout_us", "R169",
             "actuator publication time is not bounded"),
            ("now_us - actuator_motors_.timestamp_sample <= timeout_us", "R170",
             "actuator sample time is not bounded"),
            ("hard_safe_inhibit_observed_ || !parameters_valid_", "R180",
             "asynchronous Kill/Failsafe does not inhibit neutral output"),
        ),
        ROOT / "Dima/modules/motor/MotorOutputFrames.cpp": (
            ("pwm_->stop()", "R171",
             "MotorOutput has no backend safe-off path"),
            ("frame.pulse_us[channel] = config.center_us", "R180",
             "disarmed-neutral output must use each channel center"),
        ),
        ROOT / "Dima/modules/motor/MotorOutput.cpp": (
            ("build_neutral_frame(frame)", "R180",
             "MotorOutput has no disarmed-neutral frame path"),
        ),
        ROOT / "Dima/modules/safety/CommanderSafety.cpp": (
            ("actuator_output_ready_for_arming(now)", "R181",
             "Commander pre-arm does not require output readiness"),
            ("actuator_output_recovered_disarmed(now)", "R181",
             "Commander output-failsafe recovery can self-lock"),
            ("dima::platform::kActuatorPwmMinimumPulseUs", "R233",
             "Commander minimum pulse bypasses the shared envelope"),
            ("dima::platform::kActuatorPwmMaximumPulseUs", "R233",
             "Commander maximum pulse bypasses the shared envelope"),
        ),
        ROOT / "Dima/modules/safety/CommanderActions.cpp": (
            ("Kill engaged; Rover requires a new Arm action", "R181",
             "Kill must disarm and require a new Arm edge"),
        ),
        ROOT / "Dima/lib/rover/DifferentialDrive.cpp": (
            ("-1.0F / config_.thrust_asymmetry", "R182",
             "reverse asymmetry feasible domain is missing"),
            ("config_.steering_throttle_mix, lower_motor_limit", "R182",
             "axis priority does not use the asymmetric motor domain"),
        ),
        ROOT / "Dima/modules/boot_health/BootHealthService.cpp": (
            ("update_output_health(now_us)", "R176",
             "BootHealth does not require output Topic progress"),
            ("maintenance_.boot_health_update(", "R183",
             "BootHealth does not own maintenance approval"),
            ("confirmation_state_safe()", "R177",
             "BootHealth does not reset its window for unsafe vehicle state"),
            ("actuator_output_status_s::STATE_HARD_SAFE_OFF", "R178",
             "BootHealth does not recognize hard-safe-off output"),
            ("actuator_output_status_s::STATE_DISARMED_NEUTRAL", "R178",
             "BootHealth does not recognize disarmed-neutral output"),
            ("output.active_output_mask == 0U", "R179",
             "BootHealth does not validate a zero-channel hard-safe frame"),
            ("output_frame_valid(output)", "R179",
             "BootHealth does not validate active/neutral PWM frames"),
            ("health_generation_", "R183",
             "BootHealth does not advance a Runtime health generation"),
            ("dima::platform::kActuatorPwmMinimumPulseUs", "R233",
             "BootHealth minimum pulse bypasses the shared envelope"),
            ("dima::platform::kActuatorPwmMaximumPulseUs", "R233",
             "BootHealth maximum pulse bypasses the shared envelope"),
        ),
        ROOT / "Dima/application/app_main.cpp": (
            ("kWatchdogTimeoutMs = 2048U", "R183",
             "application IWDG timeout changed"),
            ("kWatchdogCheckIntervalMs = 100U", "R183",
             "application IWDG health cadence changed"),
            ("application.watchdog_feed_allowed", "R183",
             "appMain does not gate IWDG feeds on Runtime health"),
            ("services.watchdog.feed();", "R183",
             "appMain is missing the IWDG feed"),
        ),
        ROOT / "Dima/platform/stm32h7/system/Watchdog.cpp": (
            ("kPrescalerDiv32 = 3U", "R183",
             "STM32 IWDG prescaler contract changed"),
            ("IWDG1->KR = kStartKey;\n        IWDG1->KR = kWriteAccessKey;",
             "R183", "IWDG configuration waits for LSI before starting it"),
            ("DBGMCU_APB4FZ1_DBG_IWDG1", "R183",
             "IWDG must freeze while the debugger is halted"),
        ),
        ROOT / "Bootloader/Inc/mcuboot_config/mcuboot_config.h": (
            ("MCUBOOT_WATCHDOG_FEED()", "R183",
             "MCUboot long loops have no watchdog feed hook"),
            ("boot_watchdog_feed()", "R183",
             "MCUboot watchdog hook is not connected"),
        ),
        ROOT / "Bootloader/Src/main.c": (
            ("boot_watchdog_prepare();", "R183",
             "MCUboot does not extend a watchdog carried across reset"),
            ("dima_boot_diagnostics_mark_application_bridge()", "R183",
             "MCUboot does not preserve reset flags across its bridge reset"),
        ),
        ROOT / "Boards/H743/Src/boot_diagnostics_store.c": (
            ("DIMA_BOOT_DETAIL_APPLICATION_BRIDGE", "R183",
             "MCUboot diagnostics owner cannot mark its bridge reset"),
            ("seed_application_bridge_record(record);", "R183",
             "MCUboot cold boot depends on an Application-initialized D3 record"),
            ("record->reset_flags = reset_flags;", "R183",
             "cold-start bridge seeding does not preserve reset flags"),
            ("record->magic = DIMA_BOOT_DIAGNOSTICS_MAGIC;", "R183",
             "cold-start bridge seeding does not publish a valid D3 record"),
        ),
        ROOT / "Bootloader/Makefile": (
            ("Bootloader/Src/boot_watchdog.c", "R183",
             "MCUboot build does not link the watchdog bridge"),
        ),
        ROOT / "Boards/H743/Src/boot_diagnostics.c": (
            ("DIMA_BOOT_DETAIL_APPLICATION_BRIDGE", "R183",
             "application bridge can overwrite the original reset cause"),
            ("previous_reset_flags", "R183",
             "startup diagnostics do not retain the original reset flags"),
        ),
        ROOT / "Dima/middleware/parameters/definitions/commander_params.c": (
            ("PARAM_DEFINE_FLOAT(COM_ACT_LOSS_T, 0.10f);", "R172",
             "COM_ACT_LOSS_T definition or default changed"),
            ("* @min 0.02", "R173",
             "COM_ACT_LOSS_T minimum changed"),
            ("* @max 1.00", "R174",
             "COM_ACT_LOSS_T maximum changed"),
        ),
        ROOT / "Dima/platform/api/ActuatorPwmLimits.h": (
            ("#define DIMA_ACTUATOR_PWM_MIN_PULSE_US 500U", "R233",
             "product PWM minimum must remain 500 us"),
            ("#define DIMA_ACTUATOR_PWM_MAX_PULSE_US 2500U", "R233",
             "product PWM maximum must remain 2500 us"),
        ),
        ROOT / "Dima/platform/api/ActuatorPwm.hpp": (
            ("kActuatorPwmMinimumPulseUs", "R233",
             "C++ actuator API is missing the shared minimum pulse"),
            ("kActuatorPwmMaximumPulseUs", "R233",
             "C++ actuator API is missing the shared maximum pulse"),
        ),
        ROOT / "Boards/H743/Src/motor_pwm.c": (
            ("DIMA_ACTUATOR_PWM_MAX_PULSE_US <= MOTOR_PWM_PERIOD_TICKS",
             "R233", "board PWM envelope exceeds the timer period"),
            ("pulse_us[index] < DIMA_ACTUATOR_PWM_MIN_PULSE_US", "R233",
             "board PWM backend does not enforce the product minimum"),
            ("pulse_us[index] > DIMA_ACTUATOR_PWM_MAX_PULSE_US", "R233",
             "board PWM backend does not enforce the product maximum"),
        ),
        ROOT / "Dima/modules/motor/MotorOutputParameters.cpp": (
            ("dima::platform::kActuatorPwmMinimumPulseUs", "R233",
             "PWM parameter consumer bypasses the shared minimum"),
            ("dima::platform::kActuatorPwmMaximumPulseUs", "R233",
             "PWM parameter consumer bypasses the shared maximum"),
            ("ParameterIssue::EndpointsSwapped", "R233",
             "reversed PWM endpoints are not swapped at runtime"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

    motor_parameters_path = (
        ROOT / "Dima/modules/motor/MotorOutputParameters.cpp"
    )
    motor_parameters_text = motor_parameters_path.read_text(encoding="utf-8")
    motor_parameters_code = strip_c_comments(motor_parameters_text)
    reverse_domain = re.compile(
        r"raw\[channel\]\[Reversed\]\s*!=\s*0\s*&&\s*"
        r"raw\[channel\]\[Reversed\]\s*!=\s*1\s*\)\s*\{.*?"
        r"report_parameter_issue\(ParameterIssue::InvalidReversed,.*?"
        r"candidate\.channels\[channel\]\s*=\s*ChannelConfig\{\}\s*;.*?"
        r"continue\s*;\s*\}",
        re.DOTALL,
    )
    reverse_assignment = re.compile(
        r"config\.reversed\s*=\s*raw\[channel\]\[Reversed\]\s*==\s*1\s*;"
    )
    if (reverse_domain.search(motor_parameters_code) is None or
            reverse_assignment.search(motor_parameters_code) is None):
        violations.append(Violation(
            motor_parameters_path, 1, "R233",
            "PWM reverse must reject outside 0/1 before consuming value 1",
        ))

    motor_frames_path = ROOT / "Dima/modules/motor/MotorOutputFrames.cpp"
    motor_frames_text = motor_frames_path.read_text(encoding="utf-8")
    motor_frames_code = strip_c_comments(motor_frames_text)
    reverse_consumption = re.compile(
        r"if\s*\(\s*config\.reversed\s*\)\s*\{\s*"
        r"command\s*=\s*-command\s*;\s*\}",
        re.DOTALL,
    )
    if reverse_consumption.search(motor_frames_code) is None:
        violations.append(Violation(
            motor_frames_path, 1, "R233",
            "validated PWM reverse is not consumed by frame generation",
        ))

    actuator_parameters = (
        ROOT / "Dima/middleware/parameters/definitions/rover_actuator_params.c"
    )
    if actuator_parameters.is_file():
        text = actuator_parameters.read_text(encoding="utf-8")
        fields = (
            ("MIN", "minimum", 1000),
            ("CENT", "center", 1500),
            ("MAX", "maximum", 2000),
        )
        for channel in range(1, 7):
            for suffix, label, default in fields:
                definition = (
                    f"PARAM_DEFINE_INT32(PWM_S{channel}_{suffix}, "
                    f"{default});"
                )
                pattern = re.compile(
                    rf"/\*\*\s*\n\s*\* PWM S{channel} {label} pulse\s*\n"
                    rf"(?P<metadata>.*?)\*/\s*{re.escape(definition)}",
                    re.DOTALL,
                )
                match = pattern.search(text)
                if match is None:
                    violations.append(Violation(
                        actuator_parameters,
                        line_for(text, definition),
                        "R233",
                        f"PWM_S{channel}_{suffix} metadata/default changed",
                    ))
                    continue
                metadata = match.group("metadata")
                minimum_ok = re.search(
                    r"(?m)^\s*\*\s+@min\s+500\s*$", metadata
                ) is not None
                maximum_ok = re.search(
                    r"(?m)^\s*\*\s+@max\s+2500\s*$", metadata
                ) is not None
                if not minimum_ok or not maximum_ok:
                    violations.append(Violation(
                        actuator_parameters,
                        line_for(text, match.group(0)),
                        "R233",
                        f"PWM_S{channel}_{suffix} metadata is not 500..2500 us",
                    ))
            reverse_definition = (
                f"PARAM_DEFINE_INT32(PWM_S{channel}_REV, 0);"
            )
            reverse_pattern = re.compile(
                rf"/\*\*\s*\n\s*\* PWM S{channel} reverse\s*\n"
                rf"(?P<metadata>.*?)\*/\s*{re.escape(reverse_definition)}",
                re.DOTALL,
            )
            reverse_match = reverse_pattern.search(text)
            reverse_metadata = (
                reverse_match.group("metadata")
                if reverse_match is not None else ""
            )
            reverse_directives = (
                r"(?m)^\s*\*\s+@value\s+0\s+Normal\s*$",
                r"(?m)^\s*\*\s+@value\s+1\s+Reversed\s*$",
                r"(?m)^\s*\*\s+@min\s+0\s*$",
                r"(?m)^\s*\*\s+@max\s+1\s*$",
            )
            if (reverse_match is None or any(
                    re.search(pattern, reverse_metadata) is None
                    for pattern in reverse_directives)):
                violations.append(Violation(
                    actuator_parameters,
                    line_for(text, reverse_definition),
                    "R233",
                    f"PWM_S{channel}_REV metadata/default is not strict 0/1",
                ))

    legacy_envelope_paths = (
        ROOT / "Dima/modules/motor/MotorOutputParameters.cpp",
        ROOT / "Dima/modules/motor/MotorOutputFrames.cpp",
        ROOT / "Dima/modules/safety/CommanderSafety.cpp",
        ROOT / "Dima/modules/boot_health/BootHealthService.cpp",
        actuator_parameters,
    )
    for path in legacy_envelope_paths:
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        for match in re.finditer(r"\b(?:800|2200)U?\b", text):
            violations.append(Violation(
                path, line_for(text, match.group(0)), "R233",
                "legacy 800..2200 actuator PWM envelope remains",
            ))
    require_make_source_paths(
        ROOT / "make/project.mk",
        (("Dima/platform/stm32h7/system/Watchdog.cpp", "R183",
          "Application build does not link the STM32 IWDG backend"),),
        violations,
    )

    watchdog_feed_pattern = re.compile(
        r"\b(?:[A-Za-z_]\w*(?:\.|->))*watchdog_?(?:\.|->)feed\s*\("
    )
    watchdog_capability_access = re.compile(r"(?:\.|->)watchdog\b")
    watchdog_feed_calls = []
    for path in first_party_sources():
        text = path.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        relative = path.relative_to(ROOT).as_posix()
        if (relative != "Dima/application/app_main.cpp" and
                watchdog_capability_access.search(code)):
            violations.append(Violation(
                path,
                line_for(text, watchdog_capability_access.search(code).group(0)),
                "R183",
                "application watchdog capability may only be accessed by "
                "appMain",
            ))
        watchdog_feed_calls.extend(
            (path, match.start())
            for match in watchdog_feed_pattern.finditer(code)
        )
    watchdog_feed_owners = [
        path.relative_to(ROOT).as_posix()
        for path, _offset in watchdog_feed_calls
    ]
    if (len(watchdog_feed_calls) != 1 or
            watchdog_feed_owners != ["Dima/application/app_main.cpp"]):
        violations.append(Violation(
            ROOT / "Dima/application/app_main.cpp", 1, "R183",
            "appMain must be the unique application IWDG feed owner: "
            f"{watchdog_feed_owners}",
        ))

    app_main_path = ROOT / "Dima/application/app_main.cpp"
    if app_main_path.is_file():
        app_main_text = app_main_path.read_text(encoding="utf-8")
        app_main_code = strip_c_comments(app_main_text)
        guarded_feed = re.compile(
            r"if\s*\(\s*!application\.watchdog_feed_allowed\(.*?\)\s*\)"
            r"\s*\{\s*continue;\s*\}\s*"
            r"services\.watchdog\.feed\(\);\s*"
            r"application\.watchdog_feed_completed\(\);\s*"
            r"last_health_generation\s*=\s*health_generation\s*;",
            re.DOTALL,
        )
        guarded_match = guarded_feed.search(app_main_code)
        feed_matches = list(watchdog_feed_pattern.finditer(app_main_code))
        feed_is_guarded = (
            guarded_match is not None and len(feed_matches) == 1 and
            guarded_match.start() <= feed_matches[0].start() <
            guarded_match.end()
        )
        if not feed_is_guarded:
            violations.append(Violation(
                app_main_path,
                line_for(app_main_text, "services.watchdog.feed();"),
                "R183",
                "appMain feed must follow a new health generation and precede "
                "maintenance acknowledgement",
            ))

    parameter_service_path = (
        ROOT / "Dima/modules/parameters/ParameterServicePersistence.cpp"
    )
    if parameter_service_path.is_file():
        parameter_service_text = parameter_service_path.read_text(
            encoding="utf-8"
        )
        parameter_service_code = strip_c_comments(parameter_service_text)
        begin_start = parameter_service_code.find(
            "int ParameterService::begin_persistence("
        )
        begin_end = parameter_service_code.find(
            "bool ParameterService::record_maintenance_progress()",
            begin_start,
        )
        begin_body = (
            parameter_service_code[begin_start:begin_end]
            if begin_start >= 0 and begin_end > begin_start else ""
        )
        interlock_offset = begin_body.find("armed_flash_.begin_maintenance()")
        request_offset = begin_body.find("maintenance_.request(")
        if (interlock_offset < 0 or request_offset < 0 or
                interlock_offset > request_offset):
            violations.append(Violation(
                parameter_service_path,
                line_for(parameter_service_text, "begin_persistence("),
                "R183",
                "maintenance must acquire the Arm interlock before requesting "
                "a ticket",
            ))
