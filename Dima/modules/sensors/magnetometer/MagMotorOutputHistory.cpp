#include "MagMotorOutputHistory.hpp"

#include <algorithm>
#include <cmath>

namespace dima::modules::sensors {

void MagMotorOutputHistory::reset() noexcept
{
    next_ = count_ = 0U;
    have_sequence_ = false;
}

void MagMotorOutputHistory::update() noexcept
{
    for (unsigned n = 0U; n < actuator_output_status_s::ORB_QUEUE_LENGTH && subscription_.update(); ++n) {
        const auto &status = subscription_.get();
        const std::uint32_t expected = sequence_ == UINT32_MAX ? 1U : sequence_ + 1U;
        // 丢帧可能跳过停波/故障；清历史，不能把缺失窗口假定成一直有输出。
        if (have_sequence_ && status.sequence != expected) reset();
        sequence_ = status.sequence;
        have_sequence_ = true;
        const bool known = status.backend_ready && status.timestamp_output != 0U &&
            status.timestamp_output <= status.timestamp &&
            std::isfinite(status.applied_right) && std::isfinite(status.applied_left) &&
            std::fabs(status.applied_right) <= 1.0F && std::fabs(status.applied_left) <= 1.0F &&
            ((status.state == actuator_output_status_s::STATE_ACTIVE && status.command_valid) ||
             status.state == actuator_output_status_s::STATE_DISARMED_NEUTRAL ||
             (status.safe_off && status.state != actuator_output_status_s::STATE_FAULT &&
              status.state != actuator_output_status_s::STATE_RETRY));
        const std::uint64_t timestamp = known ? status.timestamp_output : status.timestamp;
        if (timestamp == 0U) { reset(); continue; }
        if (count_ != 0U && timestamp < samples_[(next_ + 31U) % 32U].timestamp) reset();
        samples_[next_] = {timestamp, status.applied_right, status.applied_left, known};
        next_ = (next_ + 1U) % 32U;
        count_ = std::min(count_ + 1U, std::size_t{32U});
    }
}

bool MagMotorOutputHistory::forward_output(std::uint64_t sample_time, float &longitudinal) const noexcept
{
    // 只选磁样本之前最近的输出，最多相差 100 ms；遇到故障/反向记录立即
    // 拒绝，不越过否定证据回找更旧的正向输出。两轮均非负才属于学习域。
    for (std::size_t age = 0U; age < count_; ++age) {
        const auto &sample = samples_[(next_ + 31U - age) % 32U];
        if (sample.timestamp > sample_time) continue;
        if (!sample.valid || sample_time - sample.timestamp > 100000ULL ||
            sample.right < 0.0F || sample.left < 0.0F) return false;
        longitudinal = 0.5F * (sample.right + sample.left);
        return true;
    }
    return false;
}

} // namespace dima::modules::sensors
