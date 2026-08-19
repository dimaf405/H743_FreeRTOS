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
    MAKE_CONTRACT_PATHS,
    require_literals_in_owners,
)


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
        "Dima/platform/api/ActuatorPwm.hpp",
        "Dima/platform/api/Services.hpp",
        "Dima/platform/stm32h7/io/ActuatorPwm.cpp",
        "Dima/platform/stm32h7/HardwareServices.hpp",
        "Dima/modules/motor/MotorOutput.cpp",
        "Dima/modules/motor/MotorOutputFrames.cpp",
        "Dima/modules/motor/MotorOutput.hpp",
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
        ROOT / "Dima/rover/ApplicationContext.cpp": (
            ("boot_health_.bind_motor_output(motor_output_);", "R163",
             "BootHealth is not bound to MotorOutput"),
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
            ("motor_output_->state()", "R176",
             "BootHealth does not monitor MotorOutput lifecycle"),
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
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)
    require_literals_in_owners(
        MAKE_CONTRACT_PATHS,
        (("Dima/platform/stm32h7/system/Watchdog.cpp", "R183",
          "Application build does not link the STM32 IWDG backend"),),
        violations,
    )

    watchdog_feed_owners = []
    for path in first_party_sources():
        text = path.read_text(encoding="utf-8")
        if "services.watchdog.feed();" in text:
            watchdog_feed_owners.append(path.relative_to(ROOT).as_posix())
    if watchdog_feed_owners != ["Dima/application/app_main.cpp"]:
        violations.append(Violation(
            ROOT / "Dima/application/app_main.cpp", 1, "R183",
            "appMain must be the unique application IWDG feed owner: "
            f"{watchdog_feed_owners}",
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
