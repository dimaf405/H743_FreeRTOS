"""Repository layout, Rover root, debug, and message contract checks."""

from __future__ import annotations

import pathlib
import re

from architecture.common import (
    ROOT,
    Violation,
    sources_under,
    is_vendored,
    line_for,
    require_literals,
    require_make_source_paths,
    MAKE_CONTRACT_PATHS,
    owner_texts,
)


def scan_rover_root_contract(violations: list[Violation]) -> None:
    rover_root = ROOT / "Dima/rover"
    legacy_root = ROOT / "Dima/modules/rover"
    if not rover_root.is_dir():
        violations.append(Violation(
            rover_root, 1, "R040", "the unique Rover product root is missing",
        ))
    if legacy_root.exists():
        violations.append(Violation(
            legacy_root, 1, "R041",
            "legacy Dima/modules/rover must not exist",
        ))

    ambiguous_paths = (
        "Dima/modules/rc/ManualControl.cpp",
        "Dima/modules/rc/ManualControl.hpp",
        "Dima/modules/manual_control",
        "Dima/rover/control/ManualMotionAdapter.cpp",
        "Dima/rover/control/ManualMotionAdapter.hpp",
        "Dima/rover/modes/manual",
        "Dima/rover/control/MotorOutput.cpp",
        "Dima/rover/control/MotorOutput.hpp",
        "Dima/rover/control/DifferentialDrive.cpp",
        "Dima/rover/control/DifferentialDrive.hpp",
        "Dima/lib/rover_control/rover_control.cpp",
        "Dima/lib/rover_control/rover_control.hpp",
        "Dima/lib/motor/speed_to_pwm.cpp",
        "Dima/lib/motor/speed_to_pwm.hpp",
    )
    for relative in ambiguous_paths:
        path = ROOT / relative
        if path.exists():
            violations.append(Violation(
                path, 1, "R212",
                "file or directory remains in a retired ambiguous path",
            ))

    required_paths = (
        "Dima/modules/rc/RcManualInput.cpp",
        "Dima/modules/rc/RcManualInput.hpp",
        "Dima/modules/motor/MotorOutput.cpp",
        "Dima/modules/motor/MotorOutput.hpp",
        "Dima/rover/modes/ManualMode.cpp",
        "Dima/rover/modes/ManualMode.hpp",
        "Dima/lib/rover/DifferentialDrive.cpp",
        "Dima/lib/rover/DifferentialDrive.hpp",
    )
    for relative in required_paths:
        path = ROOT / relative
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R212",
                "required source is missing from its unambiguous owner path",
            ))

    roots = ("Dima", "Boards", "Core", "Bootloader", "USB_DEVICE",
             "make", "docs")
    candidates = sources_under(roots)
    for root in roots:
        base = ROOT / root
        if not base.exists():
            continue
        candidates.extend(
            path for path in base.rglob("*.md") if path.is_file()
        )
        candidates.extend(
            path for path in base.rglob("*.mk") if path.is_file()
        )
    candidates.extend((ROOT / "Makefile", ROOT / "GNUmakefile"))
    for path in sorted(set(candidates)):
        if not path.is_file():
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            normalized = line.replace("\\", "/")
            if "Dima/modules/rover" in normalized:
                violations.append(Violation(
                    path, line_number, "R042",
                    "references the removed Dima/modules/rover path",
                ))
            ambiguous_references = (
                "Dima/modules/rc/ManualControl",
                "Dima/modules/manual_control/",
                "Dima/rover/control/ManualMotionAdapter",
                "Dima/rover/modes/manual/",
                "Dima/rover/control/MotorOutput",
                "Dima/rover/control/DifferentialDrive",
                "Dima/lib/rover_control/",
                "Dima/lib/motor/speed_to_pwm",
            )
            if any(token in normalized for token in ambiguous_references):
                violations.append(Violation(
                    path, line_number, "R213",
                    "references a retired ambiguous ownership path",
                ))

    runtime_tokens = (
        "ModuleBase", "ScheduledWorkItem", "uORB::", "px4::Param",
        "param_find", "param_get", "param_set",
    )
    for path in sources_under(("Dima/lib/rover",)):
        text = path.read_text(encoding="utf-8")
        for token in runtime_tokens:
            if token in text:
                violations.append(Violation(
                    path, line_for(text, token), "R214",
                    "Rover algorithm library owns runtime or middleware state",
                ))


def scan_repository_layout(violations: list[Violation]) -> None:
    expected_ioc = ROOT / "H743_FreeRTOS.ioc"
    ioc_files = list(ROOT.glob("*.ioc"))
    skipped_roots = {
        ".git", ".keys", ".vscode", "Drivers", "Middlewares", "build",
    }
    for child in ROOT.iterdir():
        if (not child.is_dir() or child.name in skipped_roots or
                child.name.startswith("build-")):
            continue
        ioc_files.extend(child.rglob("*.ioc"))
    for path in sorted(ioc_files):
        if path != expected_ioc:
            violations.append(Violation(
                path, 1, "R220",
                "secondary CubeMX project makes the authoritative .ioc "
                "ambiguous",
            ))

    retired_files = (
        ROOT / "newlib_lock_glue.c",
        ROOT / "stm32_lock.h",
        ROOT / "Dima/platform/stm32h7/Backend.hpp",
        ROOT / "Dima/modules/boot_health/boot_health.cpp",
        ROOT / "Dima/modules/boot_health/boot_health.hpp",
        ROOT / "Dima/modules/parameters/SerialMigrationSchema.hpp",
        ROOT / "Dima/modules/parameters/SerialParameterMigration.cpp",
        ROOT / "Middlewares/Third_Party/FatFs/src/diskio.c",
        ROOT / "Middlewares/Third_Party/FatFs/src/syscall.c",
    )
    for path in retired_files:
        if path.exists():
            violations.append(Violation(
                path, 1, "R221",
                "retired or misleading source path has returned",
            ))

    required_storage_paths = (
        "Boards/H743/Src/fatfs_diskio.c",
        "Dima/middleware/maintenance/RuntimeMaintenanceCoordinator.cpp",
        "Dima/middleware/parameters/FileStorage.cpp",
        "Dima/middleware/parameters/flashfs.cpp",
        "Dima/modules/parameters/ParameterService.cpp",
        "Dima/platform/api/ParameterFileStore.hpp",
        "Dima/platform/freertos/storage/FatFsParameterFileStore.cpp",
    )
    for relative in required_storage_paths:
        path = ROOT / relative
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R230",
                "required parameter storage owner is missing",
            ))

    require_make_source_paths(
        ROOT / "make/project.mk",
        tuple(
            (relative, "R231",
             "parameter storage owner is absent from the build manifest")
            for relative in required_storage_paths
            if pathlib.PurePosixPath(relative).suffix in {".c", ".cpp"}
        ),
        violations,
    )
    project_mk_text = (ROOT / "make/project.mk").read_text(encoding="utf-8")
    if "BOARD_SD_INIT_AT_BOOT" in project_mk_text:
        violations.append(Violation(
            ROOT / "make/project.mk", 1, "R231",
            "mandatory SD support cannot be compiled out",
        ))
    board_init_path = ROOT / "Boards/H743/Src/board_init.c"
    board_init_text = board_init_path.read_text(encoding="utf-8")
    if "MX_SDMMC1_SD_Init" in board_init_text:
        violations.append(Violation(
            board_init_path, line_for(board_init_text, "MX_SDMMC1_SD_Init"),
            "R230", "SD initialization must remain non-fatal and retryable",
        ))
    require_literals(
        ROOT / "Boards/H743/Src/fatfs_diskio.c",
        (
            ("SD_Reinitialize", "R230",
             "software SD reinitialization is missing"),
            ("HAL_SD_DeInit(&hsd1)", "R230",
             "SD removal must reset the HAL instance before retry"),
            ("HAL_SD_Init(&hsd1)", "R230",
             "SD insertion must reinitialize the HAL instance"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/platform/freertos/storage/FatFsParameterFileStore.cpp",
        (
            ("disk_status(0)", "R230",
             "mounted SD media must be probed in software"),
            ("invalidate_mount()", "R230",
             "FatFs media failures must invalidate the mount"),
            ("continue_write()", "R230",
             "FatFs parameter writes are not chunked"),
            ("continue_verify()", "R230",
             "FatFs parameter verification is not chunked"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/parameters/ParameterService.cpp",
        (
            ("kSnapshotMagic", "R230",
             "parameter mirror snapshot identity is missing"),
            ("storage_generation_", "R230",
             "Flash/SD snapshot ordering is missing"),
            ("payload_crc", "R230",
             "equal-generation split-brain detection is missing"),
            ("file_storage_poll(available)", "R230",
             "periodic software SD probing is missing"),
            ("resume_after_storage_available()", "R230",
             "SD reinsertion cannot resume ENOSPC autosave"),
            ("sd_mirror_required_", "R230",
             "independent SD mirror retry is missing"),
            ("begin_persistence(", "R230",
             "runtime persistence is not maintenance-gated"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/middleware/parameters/flashfs.cpp",
        (
            ("word_erased", "R230",
             "FlashFS scan treats a blank magic as end-of-media"),
            ("Operation::VerifyPayload", "R230",
             "FlashFS writes are not incrementally verified"),
            ("Operation::Commit", "R230",
             "FlashFS has no final commit step"),
        ),
        violations,
    )

    dima_root = ROOT / "Dima"
    for directory in sorted(
            path for path in dima_root.rglob("*") if path.is_dir()):
        if is_vendored(directory):
            continue
        files = sorted(path for path in directory.rglob("*") if path.is_file())
        if not files:
            violations.append(Violation(
                directory, 1, "R222",
                "empty directory does not describe an implemented owner",
            ))
        elif all(path.name == "README.md" for path in files):
            violations.append(Violation(
                files[0], 1, "R223",
                "README-only source directory implies an implementation "
                "that does not exist",
            ))

    by_basename: dict[str, list[pathlib.Path]] = {}
    for path in sources_under(("Dima",)):
        if is_vendored(path):
            continue
        by_basename.setdefault(path.name.lower(), []).append(path)
    for paths in by_basename.values():
        if len(paths) < 2:
            continue
        for path in paths:
            violations.append(Violation(
                path, 1, "R224",
                "duplicate Dima source basename makes short includes "
                "ambiguous",
            ))

    make_owners = owner_texts(MAKE_CONTRACT_PATHS)
    if make_owners:
        dima_sources = sorted(
            path for path in dima_root.rglob("*")
            if path.is_file() and path.suffix.lower() in {".c", ".cpp"}
        )
        for path in dima_sources:
            relative = path.relative_to(ROOT).as_posix()
            if not any(relative in text for _owner, text in make_owners):
                violations.append(Violation(
                    path, 1, "R225",
                    "Dima translation unit is absent from make/project.mk",
                ))
        for owner_path, text in make_owners:
            listed = set(re.findall(
                r"Dima/[A-Za-z0-9_./-]+\.(?:cpp|c)\b", text,
            ))
            for relative in sorted(listed):
                path = ROOT / relative
                if not path.is_file():
                    violations.append(Violation(
                        owner_path, line_for(text, relative), "R226",
                        f"build manifest references missing source {relative}",
                    ))

    require_literals(
        ROOT / "Core/Inc/FreeRTOSConfig.h",
        (("#include \"../../Dima/platform/freertos/FreeRTOSConfig.h\"",
          "R227", "CubeMX FreeRTOSConfig shim no longer forwards to the "
          "single Dima configuration"),),
        violations,
    )
    require_literals(
        ROOT / "Makefile",
        (("ifneq ($(DIMA_BUILD_INTERNAL),1)", "R232",
          "CubeMX Makefile can be invoked outside the GNUmakefile overlay"),),
        violations,
    )


def scan_debug_console_contract(violations: list[Violation]) -> None:
    legacy_paths = (
        ROOT / "Dima/modules/hello_world",
        ROOT / "Dima/messages/app_heartbeat.cpp",
        ROOT / "Dima/messages/app_heartbeat.hpp",
        ROOT / "tools/validate_hello_world_interval.py",
    )
    for path in legacy_paths:
        if path.exists():
            violations.append(Violation(
                path, 1, "R043",
                "HelloWorld and app_heartbeat must remain removed",
            ))

    for make_path, text in owner_texts(MAKE_CONTRACT_PATHS):
        for token in ("APP_HELLO_WORLD", "app_heartbeat", "hello_world"):
            if token in text:
                violations.append(Violation(
                    make_path, line_for(text, token), "R044",
                    f"legacy debug example token '{token}' is still built",
                ))

    require_literals(
        ROOT / "Dima/rover/ApplicationContext.cpp",
        (
            ("dima::events::reset();", "R045",
             "Application Runtime must reset the Event Ring"),
            ("dima::logging::reset();", "R046",
             "Application Runtime must reset the Log Ring"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/modules/logging/LogService.cpp",
        (
            ("dima::events::pop(event)", "R047",
             "USB debug logger must consume structured events"),
            ("kMaxEventsPerRun", "R048",
             "structured event logging must remain bounded"),
            ("Structured logging ready", "R049",
             "structured logger startup record is missing"),
            ("set_structured_sink(nullptr, &LogService::structured_sink)",
             "R049B",
             "structured logger must register the mavlink_log sink"),
            ("mavlink_log_publication_.publish(record)", "R049C",
             "structured logger must publish mavlink_log records"),
            ("enqueue_sbus_data(hrt_absolute_time())", "R189",
             "USB debug logger does not service SBUS data"),
            ("kSbus.data_period_ms", "R190",
             "SBUS USB data output is not rate limited by DebugConfig"),
            ("Source::Sbus, Level::Debug, \"sbus\"", "R191",
             "SBUS data does not use the PX4-style source log path"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/modules/logging/LogService.hpp",
        (
            ("uORB::SubscriptionData<input_rc_s>", "R188",
             "USB debug logger must observe decoded SBUS data through uORB"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/middleware/logging/debug_config.hpp",
        (
            ("kUsbMinimumLevel = Level::Debug", "R184",
             "default USB debug level changed"),
            ("kIcm42688{Level::Off, false, 100U}", "R186",
             "future ICM42688 debug output must default off"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/middleware/logging/logging.cpp",
        (
            ("if (level == Level::Off || (!raw && !config::enabled(source, level)))", "R187",
             "module logs are not filtered before formatting"),
        ), violations,
    )


def scan_phase5_message_contracts(violations: list[Violation]) -> None:
    requirements = {
        ROOT / "Dima/messages/actuator_motors.hpp": (
            ("MESSAGE_VERSION = 0U", "R140",
             "actuator_motors version contract changed"),
            ("NUM_CONTROLS = 12U", "R141",
             "actuator_motors must retain 12 public controls"),
            ("std::uint16_t reversible_flags", "R142",
             "actuator_motors reversible flags are missing"),
            ("float control[NUM_CONTROLS]", "R143",
             "actuator_motors control array is missing"),
        ),
        ROOT / "Dima/messages/rover_motion_request.hpp": (
            ("SOURCE_MANUAL = 0U", "R144",
             "Manual motion source contract changed"),
            ("SOURCE_NAVIGATION = 1U", "R145",
             "Navigation motion source reservation changed"),
            ("MODE_NORMALIZED_AXES = 0U", "R146",
             "normalized two-axis mode contract changed"),
            ("MODE_SPEED_YAW_RATE = 1U", "R147",
             "navigation speed/yaw-rate mode reservation changed"),
            ("float normalized_longitudinal", "R148",
             "longitudinal motion axis is missing"),
            ("float normalized_steering", "R149",
             "steering motion axis is missing"),
        ),
        ROOT / "Dima/messages/actuator_output_status.hpp": (
            ("NUM_OUTPUTS = 6U", "R150",
             "actuator output status must remain six-channel"),
            ("STATE_HARD_SAFE_OFF = 1U", "R151",
             "hard-safe-off output state is missing"),
            ("STATE_DISARMED_NEUTRAL = 5U", "R151",
             "disarmed-neutral output state is missing"),
            ("configured_output_mask", "R151",
             "configured output mask is missing"),
            ("right_output_mask", "R151",
             "right motor output mask is missing"),
            ("left_output_mask", "R151",
             "left motor output mask is missing"),
            ("STATE_FAULT = 4U", "R152",
             "fault output state is missing"),
            ("std::uint16_t pwm_us[NUM_OUTPUTS]", "R153",
             "per-channel applied PWM status is missing"),
        ),
        ROOT / "Dima/messages/actuator_motors.cpp": (
            ("ORB_DEFINE(actuator_motors, actuator_motors_s, 1U)", "R154",
             "actuator_motors must remain a latest-value Topic"),
        ),
        ROOT / "Dima/messages/rover_motion_request.cpp": (
            ("ORB_DEFINE(rover_motion_request, rover_motion_request_s, 8U)",
             "R155", "motion request queue depth must remain eight"),
        ),
        ROOT / "Dima/messages/actuator_output_status.cpp": (
            ("ORB_DEFINE(actuator_output_status, actuator_output_status_s, 8U)",
             "R156", "output status queue depth must remain eight"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)
