"""H743 Application ELF 的通用段地址、容量与链接边界验证。"""

from __future__ import annotations

from .reader import (
    SHT_NOBITS,
    Elf32,
    ElfVerificationError,
    Section,
)

APP_VECTOR = 0x08040400
APP_FLASH_SIZE = 0x000BF000
DTCM_BASE = 0x20000000
DTCM_SIZE = 128 * 1024
DTCM_STATIC_LIMIT = 64 * 1024
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


def range_contains(base: int, size: int, address: int, length: int) -> bool:
    """要求非空区间 ``[address,address+length)`` 完整落在目标内存区间。"""
    return length > 0 and base <= address and address + length <= base + size


def verify_section(
    elf: Elf32,
    name: str,
    address: int,
    maximum_size: int,
    exact_size: int | None = None,
) -> Section:
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


def verify_vector_symbol(elf: Elf32) -> None:
    """向量表符号必须与应用槽位起点一致。"""
    vector = elf.symbol("g_pfnVectors")
    if vector.value != APP_VECTOR:
        raise ElfVerificationError(
            f"g_pfnVectors is 0x{vector.value:08x}, expected "
            f"0x{APP_VECTOR:08x}"
        )


def verify_memory_layout(elf: Elf32) -> None:
    """核对 Application Flash、DTCM、D1/D2/D3、DMA 与启动边界。"""
    vector = elf.section(".isr_vector")
    assert vector is not None
    if vector.address != APP_VECTOR or vector.size == 0:
        raise ElfVerificationError(
            ".isr_vector is empty or does not start at 0x08040400"
        )
    ramfunc = verify_section(elf, ".dima_ramfunc", DTCM_BASE, DTCM_SIZE)
    data = verify_section(elf, ".data", RAM_D2_BASE, RAM_D2_SIZE)
    bss = elf.section(".bss")
    assert bss is not None
    # 普通初始化/零初始化数据必须完整位于 D2 SRAM1/2，DMA SRAM3 由独立段管理。
    if (bss.section_type != SHT_NOBITS or bss.size == 0 or
            not range_contains(RAM_D2_BASE, RAM_D2_SIZE,
                               bss.address, bss.size) or
            bss.address < data.address + data.size):
        raise ElfVerificationError(
            ".bss must be a non-empty D2 SRAM section following .data"
        )
    user_stack = elf.section("._user_heap_stack")
    assert user_stack is not None
    # DTCM 静态低地址占用不得超过 64 KiB，以保留 MSP 向下增长空间。
    if (not range_contains(DTCM_BASE, DTCM_SIZE,
                           user_stack.address, user_stack.size) or
            user_stack.address + user_stack.size >
            DTCM_BASE + DTCM_STATIC_LIMIT):
        raise ElfVerificationError(
            "DTCM static footprint exceeds the 64 KiB budget"
        )
    expected_boundaries = {
        "_sramfunc": ramfunc.address,
        "_eramfunc": ramfunc.address + ramfunc.size,
        "_sdata": data.address,
        "_edata": data.address + data.size,
        "_sbss": bss.address,
        "_ebss": bss.address + bss.size,
    }
    # 启动复制/清零只能依赖这些链接边界，逐项核对脚本和启动汇编的一致性。
    for symbol_name, expected_value in expected_boundaries.items():
        actual_value = elf.symbol(symbol_name).value
        if actual_value != expected_value:
            raise ElfVerificationError(
                f"{symbol_name} is 0x{actual_value:08x}, expected "
                f"0x{expected_value:08x}"
            )

    verify_section(elf, ".dima_dma", DMA_BASE, DMA_SIZE)
    verify_section(
        elf, ".dima_heap", RAM_D1_BASE, PLATFORM_HEAP_SIZE,
        exact_size=PLATFORM_HEAP_SIZE,
    )
    task_pool = verify_section(
        elf, ".dima_task_pool", TASK_POOL_BASE, TASK_POOL_MAX_SIZE,
    )
    if not range_contains(RAM_D1_BASE, RAM_D1_SIZE,
                          task_pool.address, task_pool.size):
        raise ElfVerificationError("task pool exceeds D1 SRAM capacity")
    verify_section(
        elf, ".dima_boot_diag", D3_DIAGNOSTICS_BASE,
        D3_DIAGNOSTICS_SIZE,
    )
