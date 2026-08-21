"""Active actuator and physical PWM safety contract checks."""

from __future__ import annotations

import re

from architecture.common import (
    ROOT,
    NONZERO_PWM_PULSE_RE,
    Violation,
    first_party_sources,
    line_for,
    require_literals,
    require_make_source_paths,
    strip_c_comments,
    strip_cpp_structure,
)


def _cpp_block_body(code: str, opening: int) -> str | None:
    if opening < 0 or opening >= len(code) or code[opening] != "{":
        return None
    depth = 0
    for index in range(opening, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return code[opening + 1:index]
    return None


def _cpp_function_body(code: str, signature: str) -> str | None:
    """Return one balanced, literal-free C++ function body."""
    matches = list(re.finditer(re.escape(signature), code))
    if len(matches) != 1:
        return None
    opening = code.find("{", matches[0].end())
    declaration_end = code.find(";", matches[0].end())
    if 0 <= declaration_end < opening:
        return None
    return _cpp_block_body(code, opening)


def _brace_depth_at(code: str, offset: int) -> int:
    return code[:offset].count("{") - code[:offset].count("}")


def scan_active_actuator_contract(violations: list[Violation]) -> None:
    allowed_motor_calls = {
        "Boards/H743/Inc/motor_pwm.h",
        "Boards/H743/Src/motor_pwm.c",
        "Dima/platform/stm32h7/io/ActuatorPwm.cpp",
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
        "Dima/platform/stm32h7/io/ActuatorPwm.cpp",
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
        ROOT / "Dima/platform/api/Flash.cpp": (
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
        ROOT / "Dima/modules/parameters/ParameterService.cpp"
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

    flashfs_path = ROOT / "Dima/middleware/parameters/flashfs.cpp"
    if flashfs_path.is_file():
        flashfs_text = flashfs_path.read_text(encoding="utf-8")
        flashfs_code = "\n".join(strip_cpp_structure(flashfs_text))
        conditional_directive = re.search(
            r"(?m)^\s*#\s*(?:if|ifdef|ifndef|elif|else|endif)\b",
            flashfs_code,
        )
        if conditional_directive is not None:
            violations.append(Violation(
                flashfs_path,
                line_for(flashfs_text, conditional_directive.group(0).strip()),
                "R234",
                "FlashFS persistence invariants may not be hidden behind "
                "conditional preprocessing",
            ))

        header_guard = re.compile(
            r"if\s*\(\s*!\s*header_crc_valid\s*\(\s*hdr\s*\)\s*\)"
            r"\s*\{(?P<body>[^{}]*)\}",
            re.DOTALL,
        )
        magic_guard = re.compile(
            r"if\s*\(\s*hdr\s*\.\s*magic\s*!=\s*kMagicValid\s*\)"
        )
        size_access = re.compile(r"\bhdr\s*\.\s*size\b")
        header_call = re.compile(
            r"\bheader_crc_valid\s*\(\s*hdr\s*\)"
        )
        word_advance = re.compile(
            r"\boffset\s*\+=\s*kFlashWordBytes\s*;"
        )
        offset_mutation = re.compile(
            r"\boffset\s*(?:\+\+|--|[+\-*/%]?=)"
        )
        nested_control = re.compile(
            r"\b(?:if|for|while|do|switch|goto|return|break|throw)\b|\?"
        )
        scan_functions = (
            "int FlashFS::scan() noexcept",
            "int FlashFS::find_entry_locked(",
        )
        for signature in scan_functions:
            body = _cpp_function_body(flashfs_code, signature)
            guard_match = header_guard.search(body or "")
            magic_match = magic_guard.search(body or "")
            size_match = size_access.search(body or "")
            guard_body = (
                guard_match.group("body") if guard_match is not None else ""
            )
            word_advances = word_advance.findall(guard_body)
            word_advance_match = word_advance.search(guard_body)
            continues = list(re.finditer(r"\bcontinue\s*;", guard_body))
            same_depth = (
                body is not None and guard_match is not None and
                magic_match is not None and size_match is not None and
                len({
                    _brace_depth_at(body, magic_match.start()),
                    _brace_depth_at(body, guard_match.start()),
                    _brace_depth_at(body, size_match.start()),
                }) == 1
            )
            valid_guard = (
                same_depth and
                magic_match.end() <= guard_match.start() < size_match.start() and
                len(header_call.findall(body)) == 1 and
                len(word_advances) == 1 and word_advance_match is not None and
                len(offset_mutation.findall(guard_body)) == 1 and
                len(continues) == 1 and
                word_advance_match.end() <= continues[0].start() and
                nested_control.search(guard_body) is None
            )
            if not valid_guard:
                violations.append(Violation(
                    flashfs_path,
                    line_for(flashfs_text, signature),
                    "R234",
                    "FlashFS must unconditionally validate the header CRC "
                    "after magic and before size, then advance exactly one "
                    "flashword",
                ))

        scan_body = _cpp_function_body(
            flashfs_code, "int FlashFS::scan() noexcept"
        )
        layout_commits = list(re.finditer(
            r"write_offset_\s*=\s*std::min\s*\(\s*"
            r"align_flashword\s*\(\s*high_water\s*\)\s*,\s*"
            r"partition_size_\s*\)\s*;",
            scan_body or "",
        ))
        scan_layout_valid = (
            scan_body is not None and len(layout_commits) == 1 and
            len(re.findall(
                r"\bwrite_offset_\s*(?:\+\+|--|[+\-*/%]?=)", scan_body
            )) == 1 and
            _brace_depth_at(scan_body, layout_commits[0].start()) == 0 and
            "reset_layout(" not in scan_body
        )
        if not scan_layout_valid:
            violations.append(Violation(
                flashfs_path,
                line_for(flashfs_text, "int FlashFS::scan()"),
                "R234",
                "FlashFS scan must commit its candidate high-water only after "
                "the complete partition scan",
            ))

        continue_body = _cpp_function_body(
            flashfs_code, "int FlashFS::continue_operation() noexcept"
        )
        header_phase = re.search(
            r"if\s*\(\s*operation_\s*==\s*Operation::ProgramHeader\s*\)",
            continue_body or "",
        )
        header_opening = (
            continue_body.find("{", header_phase.end())
            if continue_body is not None and header_phase is not None else -1
        )
        header_phase_body = _cpp_block_body(
            continue_body or "", header_opening
        )
        reservations = list(re.finditer(
            r"write_offset_\s*=\s*operation_entry_offset_\s*\+\s*"
            r"operation_total_size_\s*;",
            header_phase_body or "",
        ))
        header_programs = list(re.finditer(
            r"partition_\s*\.\s*program\s*\(",
            header_phase_body or "",
        ))
        reservation_valid = (
            header_phase_body is not None and len(reservations) == 1 and
            len(header_programs) == 1 and
            reservations[0].end() <= header_programs[0].start() and
            _brace_depth_at(header_phase_body, reservations[0].start()) == 0 and
            _brace_depth_at(header_phase_body, header_programs[0].start()) == 0 and
            len(re.findall(
                r"\bwrite_offset_\s*(?:\+\+|--|[+\-*/%]?=)",
                header_phase_body,
            )) == 1 and
            re.search(r"\bscan\s*\(", header_phase_body) is None
        )
        if not reservation_valid:
            violations.append(Violation(
                flashfs_path,
                line_for(flashfs_text, "Operation::ProgramHeader"),
                "R234",
                "FlashFS must reserve the complete record before attempting "
                "header programming and may not rescan away that reservation",
            ))

        begin_body = _cpp_function_body(
            flashfs_code, "int FlashFS::begin_write_entry("
        )
        payload_assignments = list(re.finditer(
            r"operation_header_\s*\.\s*crc\s*=\s*"
            r"crc\s*\^\s*UINT32_MAX\s*;",
            begin_body or "",
        ))
        header_assignments = list(re.finditer(
            r"operation_header_\s*\.\s*header_checksum\s*=\s*"
            r"header_crc\s*\(\s*operation_header_\s*\)\s*;",
            begin_body or "",
        ))
        program_header = list(re.finditer(
            r"operation_\s*=\s*Operation::ProgramHeader\s*;",
            begin_body or "",
        ))
        write_order_valid = (
            begin_body is not None and len(payload_assignments) == 1 and
            len(header_assignments) == 1 and len(program_header) == 1 and
            len(re.findall(
                r"operation_header_\s*\.\s*header_checksum",
                begin_body,
            )) == 1 and
            payload_assignments[0].end() <= header_assignments[0].start() <
            program_header[0].start() and
            _brace_depth_at(begin_body, payload_assignments[0].start()) == 0 and
            _brace_depth_at(begin_body, header_assignments[0].start()) == 0 and
            _brace_depth_at(begin_body, program_header[0].start()) == 0
        )
        if not write_order_valid:
            violations.append(Violation(
                flashfs_path,
                line_for(flashfs_text, "begin_write_entry("),
                "R234",
                "FlashFS must unconditionally store the recomputed header CRC "
                "after payload CRC finalization and before programming",
            ))

        validator_body = _cpp_function_body(
            flashfs_code, "bool FlashFS::header_crc_valid("
        )
        validator_valid = validator_body is not None and re.fullmatch(
            r"\s*return\s+header\s*\.\s*header_checksum\s*==\s*"
            r"header_crc\s*\(\s*header\s*\)\s*;\s*",
            validator_body,
        ) is not None
        crc_body = _cpp_function_body(
            flashfs_code, "std::uint32_t FlashFS::header_crc("
        )
        covered_fields = ("magic", "crc", "size", "token", "flag")
        field_updates = (
            len(re.findall(
                rf"\bcrc\s*=\s*crc32_update\s*\(\s*crc\s*,\s*"
                rf"reinterpret_cast\s*<\s*const\s+std::uint8_t\s*\*\s*>"
                rf"\s*\(\s*&\s*header\s*\.\s*{field}\s*\)\s*,\s*"
                rf"sizeof\s*\(\s*header\s*\.\s*{field}\s*\)\s*\)\s*;",
                crc_body or "",
                re.DOTALL,
            )) == 1
            for field in covered_fields
        )
        crc_mutations = re.findall(
            r"\bcrc\s*(?:\+\+|--|[+\-*/%^&|]?=)|(?:\+\+|--)\s*crc\b",
            crc_body or "",
        )
        crc_coverage_valid = (
            crc_body is not None and
            len(re.findall(r"\bcrc32_update\s*\(", crc_body)) == 5 and
            re.search(
                r"std::uint32_t\s+crc\s*=\s*UINT32_MAX\s*;", crc_body
            ) is not None and
            all(field_updates) and len(crc_mutations) == 6 and
            "header_checksum" not in crc_body and "reserved" not in crc_body and
            re.search(
                r"return\s+crc\s*\^\s*UINT32_MAX\s*;", crc_body
            ) is not None
        )
        if not validator_valid or not crc_coverage_valid:
            violations.append(Violation(
                flashfs_path,
                line_for(flashfs_text, "header_crc("),
                "R234",
                "FlashFS header CRC must cover magic/payload CRC/size/token/"
                "flag exactly and compare against its independent field",
            ))

    timer_source = ROOT / "Core/Src/tim.c"
    require_literals(
        timer_source,
        (
            ("sConfigOC.Pulse = 0;", "R123",
             "CubeMX PWM compare defaults must remain zero"),
            ("sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;", "R127",
             "TIM5 must remain a reset-mode slave"),
            ("sSlaveConfig.InputTrigger = TIM_TS_ITR3;", "R128",
             "TIM5 must remain connected to TIM8 TRGO"),
            ("sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;", "R129",
             "TIM8 must publish its update event as TRGO"),
            ("sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;", "R138",
             "TIM8 complementary outputs must map zero compare to low"),
        ),
        violations,
    )
    if timer_source.is_file():
        text = timer_source.read_text(encoding="utf-8")
        for match in re.finditer(r"sConfigOC\.Pulse\s*=\s*([^;]+);", text):
            if match.group(1).strip() not in {"0", "0U", "0UL"}:
                violations.append(Violation(
                    timer_source, line_for(text, match.group(0)), "R124",
                    "CubeMX timer init writes a non-zero compare",
                ))

    ioc = ROOT / "H743_FreeRTOS.ioc"
    require_literals(
        ioc,
        (
            ("TIM5.TIM_SlaveMode=TIM_SLAVEMODE_RESET", "R139",
             "CubeMX TIM5 reset-slave contract is missing"),
            ("VP_TIM5_VS_ClockSourceITR.Mode=TriggerSource_ITR3", "R157",
             "CubeMX TIM5 ITR3 virtual connection is missing"),
            ("TIM8.OCNPolarity_2=TIM_OCNPOLARITY_LOW", "R158",
             "CubeMX TIM8 CH2N polarity is not safe at zero compare"),
            ("TIM8.OCNPolarity_3=TIM_OCNPOLARITY_LOW", "R159",
             "CubeMX TIM8 CH3N polarity is not safe at zero compare"),
            ("TIM8.TIM_MasterOutputTrigger=TIM_TRGO_UPDATE", "R160",
             "CubeMX TIM8 update TRGO contract is missing"),
        ),
        violations,
    )
    if ioc.is_file():
        for line_number, line in enumerate(
                ioc.read_text(encoding="utf-8").splitlines(), 1):
            if NONZERO_PWM_PULSE_RE.match(line):
                violations.append(Violation(
                    ioc, line_number, "R125",
                    "CubeMX PWM pulse default must remain zero",
                ))
