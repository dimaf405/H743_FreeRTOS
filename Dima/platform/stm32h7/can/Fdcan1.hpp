#pragma once

#include "api/Can.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::platform::stm32h7 {

struct Fdcan1State;

/* FDCAN1 的唯一拥有者：ISR 只写 SPSC ring 和错误位，任务上下文负责读取、发送
 * 与 bus-off 恢复。对象只搬运 CAN 帧，不解析任何上层协议。 */
class Fdcan1 final : public dima::platform::CanTransport {
public:
    explicit Fdcan1(Fdcan1State &state) noexcept;

    bool start(const dima::platform::CanConfiguration &configuration)
        noexcept override;
    void stop() noexcept override;
    bool service() noexcept override;
    bool running() const noexcept override;

    std::size_t receive(dima::platform::CanFrame *frames,
                        std::size_t capacity) noexcept override;
    dima::platform::CanTransmitResult transmit(
        const dima::platform::CanFrame &frame) noexcept override;
    dima::platform::CanStats stats() const noexcept override;

    void handle_rx_fifo0_irq() noexcept;
    void handle_error_irq(std::uint32_t flags) noexcept;
    void handle_hal_error_irq(std::uint32_t error) noexcept;

private:
    static bool valid_configuration(
        const dima::platform::CanConfiguration &configuration) noexcept;
    bool configure_and_start() noexcept;
    void reset_ring() noexcept;
    bool push_from_isr(const dima::platform::CanFrame &frame) noexcept;

    Fdcan1State *state_;
};

Fdcan1 &fdcan1() noexcept;

} // namespace dima::platform::stm32h7
