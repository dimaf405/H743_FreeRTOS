#include "HardwareServices.hpp"

#include "usb_device.h"
#include "usbd_cdc_if.h"

namespace dima::platform::stm32h7 {
namespace {

class UsbCdcTransport final : public ConsoleTransport {
public:
    bool initialize() noexcept override
    {
        if (!initialized_) {
            MX_USB_DEVICE_Init();
            initialized_ = true;
        }
        return initialized_;
    }

    void service() noexcept override
    {
        if (initialized_) {
            (void)CDC_RearmRx_FS();
        }
    }

    bool ready() const noexcept override
    {
        return initialized_ && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
               hUsbDeviceFS.pClassData != nullptr;
    }

    ConsoleTransmitResult transmit(const std::uint8_t *data,
                                    std::size_t length) noexcept override
    {
        const std::uint8_t result = CDC_Transmit_FS(
            const_cast<std::uint8_t *>(data),
            static_cast<std::uint16_t>(length));
        if (result == USBD_OK) {
            return ConsoleTransmitResult::Accepted;
        }
        return result == USBD_BUSY ? ConsoleTransmitResult::Busy
                                   : ConsoleTransmitResult::Failed;
    }

private:
    bool initialized_{false};
};

} // namespace

ConsoleTransport &usb_console_transport() noexcept
{
    static UsbCdcTransport instance;
    return instance;
}

} // namespace dima::platform::stm32h7
