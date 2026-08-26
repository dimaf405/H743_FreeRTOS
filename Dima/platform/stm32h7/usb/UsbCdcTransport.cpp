#include "stm32h7/HardwareServices.hpp"

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
        /* CDC 接收端点可能在主机重连/短包后需要重新 arm；维护调用保持幂等，
         * 不在 ISR 中执行 USB 协议栈恢复。 */
        if (initialized_) {
            (void)CDC_RearmRx_FS();
        }
    }

    bool ready() const noexcept override
    {
        /* CONFIGURED 仍不足够，pClassData 必须已建立；否则 transmit 可能访问尚未
         * 绑定的 CDC 类实例。 */
        return initialized_ && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
               hUsbDeviceFS.pClassData != nullptr;
    }

    ConsoleTransmitResult transmit(const std::uint8_t *data,
                                    std::size_t length) noexcept override
    {
        /* USBD_BUSY 是正常背压，由上层队列重试；只有其他返回值才记为发送失败。 */
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
