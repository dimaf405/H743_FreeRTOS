"""Lifecycle, actuator, and forbidden application symbol contracts."""

from __future__ import annotations

from collections.abc import Callable

from .reader import (
    STT_FUNC,
    Elf32,
    ElfVerificationError,
    Section,
    Symbol,
)

FORBIDDEN_APPLICATION_SYMBOLS = {
    "__orb_app_heartbeat",
    "dima_boot_diagnostics_capture_pending",
    "dima_boot_diagnostics_store_enable",
    "dima_boot_diagnostics_store_pending",
}
FORBIDDEN_APPLICATION_FRAGMENTS = {
    "HelloWorld",
    "app_heartbeat",
}
REQUIRED_ACTUATOR_SYMBOLS = {
    "HAL_TIM_PWM_Start",
    "HAL_TIM_PWM_Stop",
    "HAL_TIMEx_PWMN_Start",
    "HAL_TIMEx_PWMN_Stop",
    "board_motor_pwm_start",
    "board_motor_pwm_started",
    "board_motor_pwm_stop",
    "board_motor_pwm_write",
}
FORBIDDEN_ACTUATOR_FRAGMENTS = {
    "ActuatorOutput",
    "FunctionMotors",
    "Mixer",
    "MixingOutput",
}

def symbol_section(elf: Elf32, symbol: Symbol) -> Section:
    if symbol.section_index >= len(elf.sections):
        raise ElfVerificationError(
            f"symbol '{symbol.name}' has an unsupported section index"
        )
    return elf.sections[symbol.section_index]


def unique_fragment_symbol(elf: Elf32, fragment: str) -> Symbol:
    matches = elf.symbols_matching(lambda symbol: fragment in symbol.name)
    if not matches:
        raise ElfVerificationError(
            f"required symbol containing '{fragment}' is missing"
        )
    if len(matches) != 1:
        names = ", ".join(symbol.name for symbol in matches)
        raise ElfVerificationError(
            f"symbol fragment '{fragment}' is ambiguous: {names}"
        )
    return matches[0]


def require_symbol_match(
        elf: Elf32, description: str,
        predicate: Callable[[Symbol], bool]) -> None:
    matches = elf.symbols_matching(predicate)
    if not matches:
        raise ElfVerificationError(
            f"required lifecycle symbol is missing: {description}"
        )


def verify_lifecycle_symbols(elf: Elf32) -> None:
    for name in (
        "param_shutdown",
        "_ZN3px419work_queue_shutdownEv",
        "_ZN4uORB15lifecycle_epochEv",
        "_ZN4uORB8shutdownEv",
    ):
        elf.symbol(name)
    unique_fragment_symbol(elf, "g_owner_task")
    require_symbol_match(
        elf, "FreeRTOS Backend::destroy(TaskHandle)",
        lambda symbol: "7Backend7destroyE" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    require_symbol_match(
        elf, "FlashFS::initialize()",
        lambda symbol: "7FlashFS10initializeEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    require_symbol_match(
        elf, "ParameterService::shutdown()",
        lambda symbol: "16ParameterService8shutdownEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    require_symbol_match(
        elf, "ManualMode::start()",
        lambda symbol: "ManualMode5startEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    require_symbol_match(
        elf, "RoverDifferential::start()",
        lambda symbol: "RoverDifferential5startEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    for description, fragment in (
            ("SbusRc::start()", "SbusRc5startEv"),
            ("RCUpdate::start()", "RCUpdate5startEv"),
            ("RcManualInput::start()", "RcManualInput5startEv"),
            ("Commander::start()", "Commander5startEv"),
            ("DifferentialDrive::update()", "DifferentialDrive6updateE"),
            ("BootHealthService::health_generation()",
             "BootHealthService17health_generationEv"),
            ("ApplicationContext::watchdog_feed_allowed()",
             "ApplicationContext21watchdog_feed_allowedE"),
            ("Stm32IndependentWatchdog::start()",
             "Stm32IndependentWatchdog5startE"),
            ("Stm32IndependentWatchdog::feed()",
             "Stm32IndependentWatchdog4feedEv")):
        require_symbol_match(
            elf, description,
            lambda symbol, fragment=fragment:
                fragment in symbol.name and symbol.symbol_type == STT_FUNC,
        )
    app_main = elf.symbol("app_main_task")
    if app_main.symbol_type != STT_FUNC:
        raise ElfVerificationError("app_main_task is not a function")


def verify_actuator_symbols(elf: Elf32) -> None:
    for name in sorted(REQUIRED_ACTUATOR_SYMBOLS):
        symbol = elf.symbol(name)
        if symbol.symbol_type != STT_FUNC:
            raise ElfVerificationError(
                f"required actuator symbol '{name}' is not a function"
            )
    require_symbol_match(
        elf, "MotorOutput::start()",
        lambda symbol: "MotorOutput5startEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )
    require_symbol_match(
        elf, "MotorOutput::force_safe_off()",
        lambda symbol: "MotorOutput14force_safe_offEv" in symbol.name and
        symbol.symbol_type == STT_FUNC,
    )


def verify_forbidden_symbols(elf: Elf32) -> None:
    present_names = sorted({
        symbol.name for symbol in elf.symbols
        if symbol.defined and symbol.name in FORBIDDEN_APPLICATION_SYMBOLS
    })
    forbidden_fragments = (
        FORBIDDEN_APPLICATION_FRAGMENTS | FORBIDDEN_ACTUATOR_FRAGMENTS
    )
    present_fragments = sorted({
        fragment
        for symbol in elf.symbols
        if symbol.defined
        for fragment in forbidden_fragments
        if fragment in symbol.name
    })
    if present_names or present_fragments:
        raise ElfVerificationError(
            "forbidden Application symbols are linked: "
            f"names={present_names}, fragments={present_fragments}"
        )

