#pragma once

#include "PlatformTypes.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::platform {

enum class SpiMode : std::uint8_t {
    Mode0 = 0U,
    Mode1,
    Mode2,
    Mode3,
};

struct SpiConfiguration {
    /* frequency_hz 是允许的最大 SCK；后端选择不超过它的最近离散分频。 */
    std::uint32_t frequency_hz{0U};
    SpiMode mode{SpiMode::Mode0};
};

enum class SpiTransferResult : std::uint8_t {
    Idle = 0U,
    Pending,
    Complete,
    Error,
};

enum SpiError : std::uint32_t {
    SpiErrorNone = 0U,
    SpiErrorInvalidArgument = 1U << 0U,
    SpiErrorNotReady = 1U << 1U,
    SpiErrorBusy = 1U << 2U,
    SpiErrorTimeout = 1U << 3U,
    SpiErrorPeripheral = 1U << 4U,
    SpiErrorDma = 1U << 5U,
    SpiErrorAborted = 1U << 6U,
};

struct SpiStats {
    std::uint32_t blocking_transfers{0U};
    std::uint32_t dma_starts{0U};
    std::uint32_t dma_completions{0U};
    std::uint32_t errors{0U};
    std::uint32_t aborts{0U};
    std::uint32_t last_error_flags{SpiErrorNone};
    std::uint32_t kernel_frequency_hz{0U};
    std::uint32_t requested_frequency_hz{0U};
    std::uint32_t configured_frequency_hz{0U};
};

/**
 * 板级绑定的 SPI 端点：组合根选择控制器和片选，调用方只拥有协议 mode、时钟
 * 上限和本次事务数据。一个端点同一时刻只允许一笔轮询或 DMA 事务。
 */
class SpiDevice {
public:
    virtual ~SpiDevice() = default;

    virtual bool configure(const SpiConfiguration &configuration) noexcept = 0;
    virtual void shutdown() noexcept = 0;

    /** 面向短寄存器访问的轮询事务；timeout 到期必须释放片选并返回失败。 */
    virtual bool transfer(const std::uint8_t *transmit,
                          std::uint8_t *receive, std::size_t length,
                          Timeout timeout) noexcept = 0;

    /** 启动一笔非阻塞全双工 DMA；缓冲区所有权保持到 finish/abort。 */
    virtual bool start_transfer(const std::uint8_t *transmit,
                                std::uint8_t *receive,
                                std::size_t length,
                                IsrCallback completion) noexcept = 0;

    /**
     * 在任务上下文完成 DMA 交接。Complete/Error 只返回一次，随后状态回到 Idle
     * 并释放 bounce buffer；调用者不能把中断通知本身当成缓冲区已归还。
     */
    virtual SpiTransferResult finish_transfer() noexcept = 0;
    virtual bool abort_transfer() noexcept = 0;
    virtual bool busy() const noexcept = 0;
    virtual std::size_t maximum_dma_transfer_size() const noexcept = 0;
    virtual SpiStats stats() const noexcept = 0;
};

} // namespace dima::platform
