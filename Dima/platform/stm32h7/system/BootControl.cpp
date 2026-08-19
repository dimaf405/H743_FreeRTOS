#include "platform/stm32h7/HardwareServices.hpp"

#include "boot_layout.h"
#include "dima_boot_request.h"
#include "platform/stm32h7/flash/flash_bank1.h"

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
