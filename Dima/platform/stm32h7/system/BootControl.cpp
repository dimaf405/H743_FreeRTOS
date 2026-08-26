#include "stm32h7/HardwareServices.hpp"

#include "boot_layout.h"
#include "dima_boot_request.h"
#include "flash/flash_bank1.h"

#include <cstring>

#include "stm32h7xx.h"

namespace dima::platform::stm32h7 {
namespace {

constexpr std::size_t kMagicSize = 16U;
constexpr std::uintptr_t kImageOkAddress =
    H743_PRIMARY_SLOT_BASE + H743_PRIMARY_SLOT_SIZE - 64U;
constexpr std::uintptr_t kMagicAddress =
    H743_PRIMARY_SLOT_BASE + H743_PRIMARY_SLOT_SIZE - kMagicSize;
constexpr std::uint8_t kMagic[kMagicSize] = {
    0x20U, 0x00U, 0x2dU, 0xe1U, 0x5dU, 0x29U, 0x41U, 0x0bU,
    0x8dU, 0x77U, 0x67U, 0x9cU, 0x11U, 0x0fU, 0x1fU, 0x8aU,
};

class Stm32BootControl final : public BootControl {
public:
    Stm32BootControl(FlashTransactionManager &transactions,
                     ArmedFlashCoordinator &armed_flash) noexcept
        : transactions_(transactions), armed_flash_(armed_flash)
    {
    }

    BootConfirmResult confirm_running_image() noexcept override
    {
        /* 仅应用侧确认当前 Primary 中已带 MCUboot TEST magic 的镜像。先用 no-wait
         * 取得全局 Flash 事务与未 armed 写租约；资源忙时返回 Deferred，不阻塞
         * 控制链。非测试镜像不写 image_ok，已确认镜像保持幂等。 */
        FlashTransaction transaction{transactions_, Timeout::no_wait()};
        if (!transaction) {
            return BootConfirmResult::Deferred;
        }

        const auto *magic =
            reinterpret_cast<const std::uint8_t *>(kMagicAddress);
        const auto *image_ok =
            reinterpret_cast<const std::uint8_t *>(kImageOkAddress);
        if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
            return BootConfirmResult::NotTestImage;
        }
        if (*image_ok == 0x01U) {
            return BootConfirmResult::AlreadyConfirmed;
        }
        if (*image_ok != UINT8_MAX) {
            return BootConfirmResult::FlashError;
        }

        FlashWriteLease write{armed_flash_};
        if (!write) {
            return BootConfirmResult::Deferred;
        }
        /* image_ok 位于 trailer 所属 flashword 首字节。Flash 只能把 1 写成 0，
         * 因此要求原值为 0xff，写入 0x01 后立即回读；其余字节保持 0xff。 */
        alignas(H743_FLASH_WRITE_SIZE)
            std::uint8_t flashword[H743_FLASH_WRITE_SIZE];
        std::memset(flashword, UINT8_MAX, sizeof(flashword));
        flashword[0] = 0x01U;
        if (!dima_stm32_flash_bank1_program(
                static_cast<std::uint32_t>(kImageOkAddress), flashword,
                sizeof(flashword)) ||
            *image_ok != 0x01U) {
            return BootConfirmResult::FlashError;
        }
        return BootConfirmResult::Ok;
    }

    [[noreturn]] void reboot() noexcept override
    {
        NVIC_SystemReset();
        for (;;) {
            __NOP();
        }
    }

    [[noreturn]] void reboot_to_recovery() noexcept override
    {
        /* 先写跨复位 recovery request，再触发系统复位；若请求写入失败仍复位，
         * MCUboot 将按现有 trailer/镜像状态选择启动路径。 */
        (void)dima_boot_request_set_recovery();
        NVIC_SystemReset();
        for (;;) {
            __NOP();
        }
    }

private:
    FlashTransactionManager &transactions_;
    ArmedFlashCoordinator &armed_flash_;
};

} // namespace

BootControl &boot_control(FlashTransactionManager &transactions,
                          ArmedFlashCoordinator &armed_flash) noexcept
{
    static Stm32BootControl instance{transactions, armed_flash};
    return instance;
}

} // namespace dima::platform::stm32h7
