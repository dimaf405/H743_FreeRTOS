#include "MavlinkService.hpp"

#include "api/Time.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace dima::modules::mavlink {

bool MavlinkEndpoint::enqueue_frame(const std::uint8_t *data, std::size_t size, bool reboot_ack) noexcept
{
    if (data == nullptr || size == 0U || size > MAVLINK_MAX_PACKET_LEN) { errno = EINVAL; return false; }
    if (tx_count_ >= kTxQueueCapacity || (tx_class_ != TxClass::Reply && !background_space())) {
        errno = EAGAIN;
        return false;
    }
    // 编码完成后只保存固定缓冲副本；FIFO 保持该 channel 的最终线序，不重排已编码帧。
    auto &frame = tx_queue_[(tx_head_ + tx_count_) % kTxQueueCapacity];
    std::memcpy(frame.bytes, data, size);
    frame.length = size;
    frame.kind = tx_class_;
    frame.reboot_ack = reboot_ack;
    ++tx_count_;
    return true;
}

bool MavlinkEndpoint::background_space() const noexcept
{
    return transport_.ready() && (channel_ == MAVLINK_COMM_0
        ? tx_count_ < kTxQueueCapacity / 2U && usb_timeout_remaining() != 0U
        : tx_count_ == 0U);
}

std::uint32_t MavlinkEndpoint::usb_timeout_remaining() const noexcept
{
    const auto now = hrt_absolute_time();
    // 向下取整避免每次写入向上舍入累加等待；不足 1 ms 留给下一轮。
    return now < usb_deadline_us_ ? static_cast<std::uint32_t>((usb_deadline_us_ - now) / 1000ULL) : 0U;
}

void MavlinkEndpoint::flush_tx() noexcept
{
    if (!transport_.ready() || tx_count_ == 0U) return;
    std::size_t count = 1U;
    std::size_t length = tx_queue_[tx_head_].length;
    const std::uint8_t *data = tx_queue_[tx_head_].bytes;
    const bool usb = channel_ == MAVLINK_COMM_0;
    if (usb) {
        if (usb_timeout_remaining() == 0U) return;
        std::memcpy(shared_.usb_batch, data, length);
        while (count < tx_count_) {
            const auto &frame = tx_queue_[(tx_head_ + count) % kTxQueueCapacity];
            if (length + frame.length > sizeof(shared_.usb_batch)) break;
            std::memcpy(shared_.usb_batch + length, frame.bytes, frame.length);
            length += frame.length;
            ++count;
        }
        data = shared_.usb_batch;
    } else if (transport_.tx_free_bytes() < length) {
        return;
    }
    if (transport_.write(data, length, usb ? usb_timeout_remaining() : 0U) != static_cast<int>(length)) return;
    for (std::size_t i = 0U; i < count; ++i) {
        const auto &frame = tx_queue_[tx_head_];
        if (frame.kind != TxClass::Stream) transaction_bytes_ += frame.length;
        if (frame.reboot_ack) wait_reboot_completion_ = true;
        tx_head_ = (tx_head_ + 1U) % kTxQueueCapacity;
        --tx_count_;
    }
}

bool MavlinkEndpoint::tx_drained() noexcept
{
    return tx_count_ == 0U && transport_.tx_idle();
}

std::uint64_t MavlinkEndpoint::drain_timeout_us() const noexcept
{
    if (channel_ == MAVLINK_COMM_0) return kRebootDeadlineUs;
    // 8N1 每字节 10 bit；包括已提交的最大 staging 和待加入 ACK，整数向上取整。
    std::uint64_t bytes = 512U + MAVLINK_MAX_PACKET_LEN;
    for (std::size_t i = 0U; i < tx_count_; ++i) bytes += tx_queue_[(tx_head_ + i) % kTxQueueCapacity].length;
    const auto baud = std::max<std::uint32_t>(1U, transport_.baudrate());
    return (bytes * 10000000ULL + baud - 1U) / baud + 200000ULL;
}

std::int32_t MavlinkEndpoint::default_interval(
    const dima::generated::mavlink_streams::MessageContract &contract) const noexcept
{
    return channel_ == MAVLINK_COMM_0 ? contract.default_interval_us : contract.normal_interval_us;
}

void MavlinkEndpoint::update_rate_mult(std::uint64_t now) noexcept
{
    if (channel_ == MAVLINK_COMM_0) return;
    if (rate_window_us_ == 0U) { rate_window_us_ = now; return; }
    if (now - rate_window_us_ < 1000000ULL) return;
    const float elapsed = static_cast<float>(now - rate_window_us_) * 1e-6F;
    std::int32_t configured = 0;
    (void)param_get(rate_handle_, &configured);
    const float baud = static_cast<float>(transport_.baudrate());
    const float budget = configured > 0 ? std::min(static_cast<float>(configured), baud / 10.0F) : baud / 20.0F;
    float demand = 0.0F;
    std::size_t index = 0U;
    for (const auto &contract : dima::generated::mavlink_streams::kMessages) {
        if (contract.scheduler != dima::generated::mavlink_streams::Scheduler::Service) continue;
        const auto interval = configured_streams_[index++].interval_us;
        if (interval > 0) demand += static_cast<float>(contract.wire_size) * 1e6F / static_cast<float>(interval);
    }
    // 对照 PX4 update_rate_mult：(预算 - 非周期事务实测速率) / 周期需求，限于 [0.05,1]。
    // 额外按真实排队受阻比例降速；仅改变发送时刻，不覆盖每条链路请求的原始间隔。
    const float available = std::max(0.0F, budget - static_cast<float>(transaction_bytes_) / elapsed);
    float multiplier = demand > 0.0F ? std::clamp(available / demand, 0.05F, 1.0F) : 1.0F;
    // 同周期到期造成的短暂排队是正常现象；仅超过 75% 调度持续受阻时降速。
    if (stream_attempts_ != 0U && stream_blocked_ > stream_attempts_ * 3U / 4U) {
        multiplier = std::min(multiplier, rate_multiplier_ * 0.8F);
    } else {
        multiplier = std::min(multiplier, rate_multiplier_ + 0.1F);
    }
    rate_multiplier_ = std::clamp(multiplier, 0.05F, 1.0F);
    rate_window_us_ = now;
    transaction_bytes_ = stream_attempts_ = stream_blocked_ = 0U;
}

} // namespace dima::modules::mavlink
