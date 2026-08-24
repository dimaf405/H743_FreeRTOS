"""H743 section layout, interrupt, and initialization-array contracts."""

from __future__ import annotations

import struct

from .reader import (
    SHT_NOBITS,
    STB_GLOBAL,
    STB_WEAK,
    STT_FUNC,
    STT_NOTYPE,
    STT_OBJECT,
    Elf32,
    ElfVerificationError,
    Section,
)
from .symbols import symbol_section, unique_fragment_symbol

APP_VECTOR = 0x08040400
DTCM_BASE = 0x20000000
DTCM_SIZE = 128 * 1024
RAM_D1_BASE = 0x24000000
RAM_D1_SIZE = 512 * 1024
RAM_D2_BASE = 0x30000000
RAM_D2_SIZE = 256 * 1024
DMA_BASE = 0x30040000
DMA_SIZE = 32 * 1024
D3_DIAGNOSTICS_BASE = 0x38000000
D3_DIAGNOSTICS_SIZE = 512
PLATFORM_HEAP_SIZE = 256 * 1024
TASK_POOL_BASE = RAM_D1_BASE + PLATFORM_HEAP_SIZE
TASK_POOL_MAX_SIZE = 48 * 1024
ZERO_INITIALIZED_STATE_SIZES = {
    "g_counters": 64 * 72,
    "g_backend_state": 4228,
    "g_file_store_state": 1128,
}

# MAVLink module added 4 entries: orb_mavlink_log, orb_vehicle_command,
# orb_vehicle_command_ack, and LogService::mavlink_log_publication_.
INIT_ARRAY_ALLOWLIST = {
    "register_fini",
    "frame_dummy",
    "_GLOBAL__sub_I__ZN4dima10parameters8internal18g_runtime_defaultsE",
    "_GLOBAL__sub_I___orb_action_request",
    "_GLOBAL__sub_I___orb_actuator_armed",
    "_GLOBAL__sub_I___orb_actuator_motors",
    "_GLOBAL__sub_I___orb_actuator_output_status",
    "_GLOBAL__sub_I___orb_input_rc",
    "_GLOBAL__sub_I___orb_manual_control_setpoint",
    "_GLOBAL__sub_I___orb_manual_control_switches",
    "_GLOBAL__sub_I___orb_parameter_update",
    "_GLOBAL__sub_I___orb_rc_channels",
    "_GLOBAL__sub_I___orb_rover_motion_request",
    "_GLOBAL__sub_I___orb_vehicle_control_mode",
    "_GLOBAL__sub_I___orb_vehicle_status",
    # --- MAVLink module (uORB topics + LogService static) ---
    "_GLOBAL__sub_I___orb_mavlink_log",
    "_GLOBAL__sub_I___orb_vehicle_command",
    "_GLOBAL__sub_I___orb_vehicle_command_ack",
    "_GLOBAL__sub_I__ZN4dima7modules7logging10LogService24mavlink_log_publication_E",
}
FINI_ARRAY_ALLOWLIST = {
    "__do_global_dtors_aux",
    # --- MAVLink module (LogService static destructor) ---
    "_GLOBAL__sub_D__ZN4dima7modules7logging10LogService24mavlink_log_publication_E",
}

def range_contains(base: int, size: int, address: int, length: int) -> bool:
    return length > 0 and base <= address and address + length <= base + size


def verify_section(
        elf: Elf32, name: str, address: int, maximum_size: int,
        exact_size: int | None = None) -> Section:
    section = elf.section(name)
    assert section is not None
    if section.address != address:
        raise ElfVerificationError(
            f"section '{name}' starts at 0x{section.address:08x}, "
            f"expected 0x{address:08x}"
        )
    if section.size == 0:
        raise ElfVerificationError(f"section '{name}' is empty")
    if section.size > maximum_size:
        raise ElfVerificationError(
            f"section '{name}' uses {section.size} bytes, limit is "
            f"{maximum_size} bytes"
        )
    if exact_size is not None and section.size != exact_size:
        raise ElfVerificationError(
            f"section '{name}' uses {section.size} bytes, expected exactly "
            f"{exact_size} bytes"
        )
    return section


def verify_handler_symbols(elf: Elf32) -> None:
    vector = elf.symbol("g_pfnVectors")
    if vector.value != APP_VECTOR:
        raise ElfVerificationError(
            f"g_pfnVectors is 0x{vector.value:08x}, expected "
            f"0x{APP_VECTOR:08x}"
        )

    default_handler = elf.symbol("Default_Handler")
    for name in ("SysTick_Handler", "TIM2_IRQHandler"):
        handler = elf.symbol(name)
        if handler.binding != STB_GLOBAL or handler.symbol_type != STT_FUNC:
            raise ElfVerificationError(
                f"{name} must be a strong global function"
            )
        if handler.normalized_address == default_handler.normalized_address:
            raise ElfVerificationError(f"{name} still resolves to Default_Handler")

    tim12_vector = elf.symbol("TIM8_BRK_TIM12_IRQHandler")
    if tim12_vector.binding != STB_WEAK:
        raise ElfVerificationError(
            "TIM8_BRK_TIM12_IRQHandler must remain a weak startup vector"
        )
    if tim12_vector.normalized_address != default_handler.normalized_address:
        raise ElfVerificationError(
            "TIM8_BRK_TIM12_IRQHandler unexpectedly owns a handler"
        )
    if elf.symbols_matching(lambda symbol: symbol.name == "htim12"):
        raise ElfVerificationError("htim12 is linked into the application")


def array_entry_names(
        elf: Elf32, section_name: str,
        allowlist: set[str], required: bool = True) -> list[str]:
    section = elf.section(section_name, required=required)
    if section is None:
        return []
    if section.size % 4 != 0:
        raise ElfVerificationError(
            f"section '{section_name}' is not an array of 32-bit pointers"
        )
    if section.size == 0:
        if allowlist:
            raise ElfVerificationError(
                f"section '{section_name}' is empty; expected "
                f"{len(allowlist)} entries"
            )
        return []
    data = elf.section_data(section)
    pointers = struct.unpack(f"<{section.size // 4}I", data)
    if len(pointers) != len(allowlist):
        raise ElfVerificationError(
            f"section '{section_name}' has {len(pointers)} entries; "
            f"allowlist has {len(allowlist)}"
        )

    resolved: list[str] = []
    for index, pointer in enumerate(pointers):
        normalized = pointer & ~1
        candidates = {
            symbol.name for symbol in elf.symbols
            if symbol.defined and symbol.name and
            symbol.symbol_type in {STT_NOTYPE, STT_FUNC} and
            symbol.normalized_address == normalized
        }
        allowed = candidates & allowlist
        if len(allowed) != 1:
            detail = ", ".join(sorted(candidates)) or "no symbol"
            raise ElfVerificationError(
                f"{section_name}[{index}] at 0x{pointer:08x} does not "
                f"resolve uniquely to the allowlist ({detail})"
            )
        resolved.append(next(iter(allowed)))
    if set(resolved) != allowlist or len(set(resolved)) != len(resolved):
        missing = sorted(allowlist - set(resolved))
        raise ElfVerificationError(
            f"section '{section_name}' has duplicate or missing entries: "
            f"missing={missing}"
        )
    return resolved


def verify_initialization_arrays(elf: Elf32) -> None:
    preinit = elf.section(".preinit_array", required=False)
    if preinit is not None and preinit.size != 0:
        raise ElfVerificationError(".preinit_array must remain empty")
    array_entry_names(elf, ".init_array", INIT_ARRAY_ALLOWLIST)
    array_entry_names(elf, ".fini_array", FINI_ARRAY_ALLOWLIST)


def verify_memory_layout(elf: Elf32) -> None:
    vector = elf.section(".isr_vector")
    assert vector is not None
    if vector.address != APP_VECTOR or vector.size == 0:
        raise ElfVerificationError(
            ".isr_vector is empty or does not start at 0x08040400"
        )
    verify_section(elf, ".dima_ramfunc", DTCM_BASE, DTCM_SIZE)
    dma = verify_section(elf, ".dima_dma", DMA_BASE, DMA_SIZE)
    verify_section(
        elf, ".dima_heap", RAM_D1_BASE, PLATFORM_HEAP_SIZE,
        exact_size=PLATFORM_HEAP_SIZE,
    )
    verify_section(
        elf, ".dima_task_pool", TASK_POOL_BASE, TASK_POOL_MAX_SIZE,
    )
    verify_section(
        elf, ".dima_boot_diag", D3_DIAGNOSTICS_BASE,
        D3_DIAGNOSTICS_SIZE,
    )

    receive_ring = unique_fragment_symbol(elf, "g_receive_ring")
    if receive_ring.symbol_type != STT_OBJECT or receive_ring.size == 0:
        raise ElfVerificationError("g_receive_ring is not a sized object")
    receive_section = symbol_section(elf, receive_ring)
    if receive_section.name != ".bss":
        raise ElfVerificationError(
            f"g_receive_ring is in '{receive_section.name}', expected CPU .bss"
        )
    cpu_regions = (
        (DTCM_BASE, DTCM_SIZE),
        (RAM_D1_BASE, RAM_D1_SIZE),
        (RAM_D2_BASE, RAM_D2_SIZE),
    )
    if not any(range_contains(base, size, receive_ring.value,
                              receive_ring.size)
               for base, size in cpu_regions):
        raise ElfVerificationError("g_receive_ring is outside CPU RAM")
    if range_contains(DMA_BASE, DMA_SIZE, receive_ring.value,
                      receive_ring.size):
        raise ElfVerificationError("g_receive_ring overlaps DMA-owned SRAM")

    dma_buffer = unique_fragment_symbol(elf, "g_dma_buffer")
    if (dma_buffer.symbol_type != STT_OBJECT or dma_buffer.size != 64 or
            not range_contains(DMA_BASE, DMA_SIZE, dma_buffer.value,
                               dma_buffer.size)):
        raise ElfVerificationError(
            "g_dma_buffer must be a 64-byte object in DMA-owned SRAM"
        )
    dma_buffer_section = symbol_section(elf, dma_buffer)
    if dma_buffer_section.index != dma.index:
        raise ElfVerificationError("g_dma_buffer is not in .dima_dma")

    for fragment, expected_size in ZERO_INITIALIZED_STATE_SIZES.items():
        state = unique_fragment_symbol(elf, fragment)
        state_section = symbol_section(elf, state)
        if state.symbol_type != STT_OBJECT or state.size != expected_size:
            raise ElfVerificationError(
                f"{fragment} must remain a {expected_size}-byte object"
            )
        if (state_section.name != ".bss" or
                state_section.section_type != SHT_NOBITS):
            raise ElfVerificationError(
                f"{fragment} is not in zero-initialized .bss"
            )
