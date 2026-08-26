#include "stm32h7/HardwareServices.hpp"

#include "flash_fault.h"
#include "memory/cache.h"
#include "boot_layout.h"

#include <algorithm>
#include <cstring>

#include "stm32h7xx_hal.h"

namespace dima::platform::stm32h7 {
namespace {

constexpr std::size_t kFlashWordBytes = H743_FLASH_WRITE_SIZE;
constexpr std::uintptr_t kStorageBegin = H743_STORAGE_BASE;
constexpr std::size_t kStorageBytes = H743_STORAGE_SIZE;
constexpr std::uintptr_t kStorageEnd = kStorageBegin + kStorageBytes;
constexpr std::uint32_t kEccFlags = FLASH_FLAG_SNECCERR_BANK2 |
                                    FLASH_FLAG_DBECCERR_BANK2;

volatile bool g_safe_read_active{false};
volatile bool g_safe_read_faulted{false};
volatile std::uintptr_t g_safe_read_resume{0U};
volatile std::uintptr_t g_safe_read_address{0U};

/* Bank2 擦除态或 ECC 损坏字可能在普通 load 上触发精确 BusFault。该受控读窗口
 * 仅覆盖参数分区：异常处理器可把堆栈 PC 改到 read_complete，将硬故障转换为
 * false；其他地址/其他 BusFault 仍走正常致命诊断。O0/noinline 固定恢复标签语义。 */
__attribute__((noinline, optimize("O0")))
bool safe_read_u32(std::uintptr_t address, std::uint32_t &value) noexcept
{
    __HAL_FLASH_CLEAR_FLAG_BANK2(kEccFlags);
    g_safe_read_address = address;
    g_safe_read_faulted = false;
    g_safe_read_resume =
        reinterpret_cast<std::uintptr_t>(&&read_complete) &
        ~std::uintptr_t{1U};
    g_safe_read_active = true;
    __DSB();
    value = *reinterpret_cast<volatile const std::uint32_t *>(address);
read_complete:
    __DSB();
    g_safe_read_active = false;
    if (g_safe_read_faulted) {
        return false;
    }
    if (__HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_SNECCERR_BANK2) ||
        __HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_DBECCERR_BANK2)) {
        __HAL_FLASH_CLEAR_FLAG_BANK2(kEccFlags);
        return false;
    }
    return true;
}

bool safe_read_bytes(std::uintptr_t address, void *destination,
                     std::size_t length) noexcept
{
    if ((destination == nullptr && length != 0U) ||
        address < kStorageBegin || address > kStorageEnd ||
        length > kStorageEnd - address) {
        return false;
    }

    /* 非对齐范围按 32-bit 安全读取后切片；每次只把已通过 ECC/BusFault 检查的
     * 字节复制给调用方，地址加法同时检查上溢与分区边界。 */
    auto *output = static_cast<std::uint8_t *>(destination);
    while (length > 0U) {
        const std::uintptr_t word_address =
            address & ~std::uintptr_t{sizeof(std::uint32_t) - 1U};
        const std::size_t byte_offset =
            static_cast<std::size_t>(address - word_address);
        std::uint32_t word = 0U;
        if (!safe_read_u32(word_address, word)) {
            return false;
        }
        const std::size_t copy_length =
            std::min(sizeof(word) - byte_offset, length);
        const auto *word_bytes =
            reinterpret_cast<const std::uint8_t *>(&word);
        std::memcpy(output, word_bytes + byte_offset, copy_length);
        address += copy_length;
        output += copy_length;
        length -= copy_length;
    }
    return true;
}

class ParameterFlashPartition final : public FlashPartition {
public:
    std::uintptr_t base() const noexcept override { return kStorageBegin; }
    std::size_t size() const noexcept override { return kStorageBytes; }
    std::size_t program_size() const noexcept override
    {
        return kFlashWordBytes;
    }

    bool read(std::size_t offset, void *destination,
              std::size_t length) noexcept override
    {
        if ((destination == nullptr && length != 0U) ||
            offset > kStorageBytes || length > kStorageBytes - offset) {
            return false;
        }
        return length == 0U ||
               safe_read_bytes(kStorageBegin + offset, destination, length);
    }

    bool program(std::size_t offset, const void *source,
                 std::size_t length) noexcept override
    {
        /* H743 Flash 只能按 32-byte flashword 编程；偏移和长度必须整齐，且每个
         * flashword 写后立即失效 cache、受控回读并逐字节核对。 */
        if (source == nullptr || length == 0U ||
            (offset % kFlashWordBytes) != 0U ||
            (length % kFlashWordBytes) != 0U ||
            offset > kStorageBytes || length > kStorageBytes - offset) {
            return false;
        }
        if (HAL_FLASH_Unlock() != HAL_OK) {
            return false;
        }
        __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2 |
                                     FLASH_FLAG_EOP_BANK2);

        alignas(kFlashWordBytes) std::uint8_t flashword[kFlashWordBytes]{};
        const auto *input = static_cast<const std::uint8_t *>(source);
        bool programmed = true;
        for (std::size_t written = 0U; written < length;
             written += kFlashWordBytes) {
            std::memcpy(flashword, input + written, sizeof(flashword));
            const std::uintptr_t address = kStorageBegin + offset + written;
            if (HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_FLASHWORD, address,
                    static_cast<std::uint32_t>(
                        reinterpret_cast<std::uintptr_t>(flashword))) !=
                HAL_OK) {
                programmed = false;
                break;
            }
            dima_stm32_cache_invalidate_range(
                reinterpret_cast<const void *>(address), kFlashWordBytes);
            alignas(4) std::uint8_t verify[kFlashWordBytes]{};
            if (!safe_read_bytes(address, verify, sizeof(verify)) ||
                std::memcmp(verify, flashword, sizeof(verify)) != 0) {
                programmed = false;
                break;
            }
        }
        (void)HAL_FLASH_Lock();
        return programmed;
    }

    bool erase() noexcept override
    {
        /* 参数分区独占 Bank2 sector 7。HAL 返回成功后仍遍历全部 flashword 验证
         * 0xff，ECC/BusFault 或任一非擦除字节都会使擦除失败。 */
        if (HAL_FLASH_Unlock() != HAL_OK) {
            return false;
        }
        __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2 |
                                     FLASH_FLAG_EOP_BANK2);
        FLASH_EraseInitTypeDef erase{};
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.Banks = FLASH_BANK_2;
        erase.Sector = FLASH_SECTOR_7;
        erase.NbSectors = 1U;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        std::uint32_t sector_error = UINT32_MAX;
        const HAL_StatusTypeDef result =
            HAL_FLASHEx_Erase(&erase, &sector_error);
        (void)HAL_FLASH_Lock();
        if (result != HAL_OK || sector_error != UINT32_MAX) {
            return false;
        }

        dima_stm32_cache_invalidate_range(
            reinterpret_cast<const void *>(kStorageBegin), kStorageBytes);
        alignas(4) std::uint8_t word[kFlashWordBytes]{};
        for (std::size_t offset = 0U; offset < kStorageBytes;
             offset += kFlashWordBytes) {
            if (!safe_read_bytes(kStorageBegin + offset, word,
                                 sizeof(word))) {
                return false;
            }
            for (std::uint8_t byte : word) {
                if (byte != UINT8_MAX) {
                    return false;
                }
            }
        }
        return true;
    }
};

} // namespace

bool flash_initialize() noexcept
{
    /* 显式使能 BusFault，受控 ECC 读才能由专用恢复钩子识别；未命中的故障继续
     * 进入启动诊断，不能被全局吞掉。 */
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    __DSB();
    __ISB();
    return true;
}

FlashPartition &parameter_partition() noexcept
{
    static ParameterFlashPartition instance;
    return instance;
}

} // namespace dima::platform::stm32h7

extern "C" int dima_flash_busfault_recover(std::uint32_t *stacked_frame)
{
    using namespace dima::platform::stm32h7;
    if (!g_safe_read_active || stacked_frame == nullptr ||
        g_safe_read_address < kStorageBegin ||
        g_safe_read_address >= kStorageEnd ||
        (FLASH->SR2 & FLASH_SR_DBECCERR) == 0U) {
        return 0;
    }

    /* Cortex-M 异常栈第 6 个 word 是返回 PC。仅在 active+Bank2 DBECC 条件全部
     * 成立时改写为受控恢复标签，并清粘滞状态；返回 1 表示已消费本次 BusFault。 */
    g_safe_read_active = false;
    g_safe_read_faulted = true;
    __HAL_FLASH_CLEAR_FLAG_BANK2(kEccFlags);
    SCB->CFSR = SCB_CFSR_BUSFAULTSR_Msk;
    stacked_frame[6] = static_cast<std::uint32_t>(g_safe_read_resume);
    __DSB();
    __ISB();
    return 1;
}
