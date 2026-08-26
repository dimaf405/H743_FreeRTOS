"""仓库布局、唯一 Rover 组合根、调试出口与消息 schema/生成物边界门禁。"""

from __future__ import annotations

import json
import pathlib

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
from architecture.build_closure import BuildClosureError, load_build_closure

def scan_rover_root_contract(violations: list[Violation]) -> None:
    """要求产品实现位于唯一 Dima/rover 根，并拒绝已废弃或语义歧义的旧路径。"""
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
    """核对有效 owner、非空目录和 Make 源码闭包，不固定文档或工程文件位置。"""

    required_storage_paths = (
        "Boards/H743/Src/fatfs_diskio.c",
        "Dima/middleware/maintenance/RuntimeMaintenanceCoordinator.cpp",
        "Dima/middleware/parameters/FileStorage.cpp",
        "Dima/middleware/parameters/flashfs.cpp",
        "Dima/modules/parameters/ParameterSnapshotCodec.cpp",
        "Dima/modules/parameters/ParameterSnapshotCodec.hpp",
        "Dima/modules/parameters/ParameterService.cpp",
        "Dima/modules/parameters/ParameterServicePersistence.cpp",
        "Dima/modules/parameters/ParameterServiceSdMirror.cpp",
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
        ROOT / "Dima/modules/parameters/ParameterSnapshotCodec.cpp",
        (
            ("kSnapshotMagic", "R230",
             "parameter mirror snapshot identity is missing"),
            ("payload_crc", "R230",
             "equal-generation split-brain detection is missing"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/parameters/ParameterServicePersistence.cpp",
        (
            ("storage_generation_", "R230",
             "Flash/SD snapshot ordering is missing"),
            ("begin_persistence(", "R230",
             "runtime persistence is not maintenance-gated"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/parameters/ParameterServiceSdMirror.cpp",
        (
            ("file_storage_poll(available)", "R230",
             "periodic software SD probing is missing"),
            ("resume_after_storage_available()", "R230",
             "SD reinsertion cannot resume ENOSPC autosave"),
            ("sd_mirror_required_", "R230",
             "independent SD mirror retry is missing"),
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

    project_make = ROOT / "make/project.mk"
    if project_make.is_file():
        make_text = project_make.read_text(encoding="utf-8")
        try:
            closure = load_build_closure(ROOT)
        except BuildClosureError as error:
            violations.append(Violation(
                project_make, 1, "R225",
                f"cannot evaluate the real Make build closure: {error}",
            ))
        else:
            compiled_sources = closure.sources
            parameter_sources = closure.parameter_generator_inputs
            first_party_sources = sorted(
                path
                for source_root in (dima_root, ROOT / "Boards/H743")
                for path in source_root.rglob("*")
                if path.is_file()
                and path.suffix.lower() in {".c", ".cpp"}
            )
            for path in first_party_sources:
                relative = path.relative_to(ROOT).as_posix()
                is_parameter_input = path.is_relative_to(
                    ROOT / "Dima/middleware/parameters/definitions"
                )
                expected_sources = (
                    parameter_sources if is_parameter_input
                    else compiled_sources
                )
                if relative not in expected_sources:
                    violations.append(Violation(
                        path, 1, "R225",
                        "first-party source is outside its compiled or "
                        "generator input closure",
                    ))

            for relative in sorted(compiled_sources | parameter_sources):
                if not relative.startswith(("Dima/", "Boards/H743/")):
                    continue
                source_path = ROOT / relative
                if source_path.is_file():
                    continue
                boot_make = ROOT / "Bootloader/Makefile"
                owner = project_make
                owner_text = make_text
                if relative not in owner_text and boot_make.is_file():
                    owner = boot_make
                    owner_text = boot_make.read_text(encoding="utf-8")
                violations.append(Violation(
                    owner, line_for(owner_text, relative), "R226",
                    f"evaluated build closure references missing {relative}",
                ))

    require_literals(
        ROOT / "Core/Inc/FreeRTOSConfig.h",
        (("#include \"freertos/FreeRTOSConfig.h\"",
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
    """核对调试示例已退出生产闭包，并保证结构化日志与 SBUS 输出仍有唯一出口。"""
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
    """验证 uORB schema 是唯一权威输入，派生头/源及 ABI lock 只能由生成器产生。"""
    requirements = {
        ROOT / "Dima/messages/schemas/actuator_motors.msg": (
            ("uint32 MESSAGE_VERSION = 0", "R140",
             "actuator_motors version contract changed"),
            ("uint8 NUM_CONTROLS = 12", "R141",
             "actuator_motors must retain 12 public controls"),
            ("uint16 reversible_flags", "R142",
             "actuator_motors reversible flags are missing"),
            ("float32[NUM_CONTROLS] control", "R143",
             "actuator_motors control array is missing"),
            ("@queue 1", "R154",
             "actuator_motors must remain a latest-value Topic"),
        ),
        ROOT / "Dima/messages/schemas/rover_motion_request.msg": (
            ("uint8 SOURCE_MANUAL = 0", "R144",
             "Manual motion source contract changed"),
            ("uint8 SOURCE_NAVIGATION = 1", "R145",
             "Navigation motion source reservation changed"),
            ("uint8 MODE_NORMALIZED_AXES = 0", "R146",
             "normalized two-axis mode contract changed"),
            ("uint8 MODE_SPEED_YAW_RATE = 1", "R147",
             "navigation speed/yaw-rate mode reservation changed"),
            ("float32 normalized_longitudinal", "R148",
             "longitudinal motion axis is missing"),
            ("float32 normalized_steering", "R149",
             "steering motion axis is missing"),
            ("@queue 8", "R155",
             "motion request queue depth must remain eight"),
        ),
        ROOT / "Dima/messages/schemas/actuator_output_status.msg": (
            ("uint8 NUM_OUTPUTS = 6", "R150",
             "actuator output status must remain six-channel"),
            ("uint8 STATE_HARD_SAFE_OFF = 1", "R151",
             "hard-safe-off output state is missing"),
            ("uint8 STATE_DISARMED_NEUTRAL = 5", "R151",
             "disarmed-neutral output state is missing"),
            ("configured_output_mask", "R151",
             "configured output mask is missing"),
            ("right_output_mask", "R151",
             "right motor output mask is missing"),
            ("left_output_mask", "R151",
             "left motor output mask is missing"),
            ("uint8 STATE_FAULT = 4", "R152",
             "fault output state is missing"),
            ("uint16[NUM_OUTPUTS] pwm_us", "R153",
             "per-channel applied PWM status is missing"),
            ("@queue 8", "R156",
             "output status queue depth must remain eight"),
        ),
        ROOT / "Dima/messages/schemas/estimator_gps_status.msg": (
            ("bool checks_passed", "R343",
             "PX4 GPS eligibility result is missing"),
            ("bool check_fail_spoofed_gps", "R343",
             "PX4 GPS spoofing failure bit is missing"),
            ("bool check_fail_max_horz_drift", "R343",
             "unsupported PX4 GPS drift bit must remain explicit"),
            ("@queue 1", "R343",
             "estimator_gps_status must remain latest-value"),
        ),
        ROOT / "Dima/messages/schemas/vehicle_imu_status.msg": (
            ("uint32[3] accel_clipping", "R344",
             "PX4 cumulative accelerometer clipping is missing"),
            ("float32 delta_angle_coning_metric", "R344",
             "PX4 coning diagnostic is missing"),
            ("float32[3] var_gyro", "R344",
             "PX4 gyroscope variance is missing"),
            ("@queue 1", "R344",
             "vehicle_imu_status must remain latest-value"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

    schema_dir = ROOT / "Dima/messages/schemas"
    schema_names = sorted(path.stem for path in schema_dir.glob("*.msg"))

    lock_path = ROOT / "Dima/messages/uorb_abi.lock.json"
    try:
        lock = json.loads(lock_path.read_text(encoding="utf-8"))
        locked_topics = lock["topics"]
        locked_names = sorted(topic["name"] for topic in locked_topics)
        # schema 目录是 Topic 集合的权威来源；ABI lock 只锁定这个派生集合及
        # 实例上限，新增 schema 时不再同步维护第二份固定数量常量。
        if (lock.get("format_version") != 1 or
                lock.get("maximum_instances") != 4 or
                lock.get("topic_count") != len(locked_topics) or
                len(locked_topics) != len(schema_names) or
                locked_names != schema_names):
            violations.append(Violation(
                lock_path, 1, "R339",
                "uORB ABI lock does not match the schema-derived "
                "four-instance topic set",
            ))
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        violations.append(Violation(
            lock_path, 1, "R339", "uORB ABI lock is missing or invalid",
        ))

    for legacy in sorted((ROOT / "Dima/messages").glob("*.[ch]pp")):
        violations.append(Violation(
            legacy, 1, "R340",
            "hand-written uORB Topic contract bypasses schema generation",
        ))

    require_literals(
        ROOT / "make/project.mk",
        (
            ("MESSAGE_GENERATOR := tools/uorb/generate_messages.py", "R341",
             "uORB schema generator is not part of the build contract"),
            ("MESSAGE_ABI_LOCK := Dima/messages/uorb_abi.lock.json", "R341",
             "uORB ABI lock is not part of the build contract"),
            ("$(MESSAGE_GENERATED_SOURCE)", "R341",
             "generated uORB metadata source is not built"),
        ),
        violations,
    )
