/****************************************************************************
 * PX4-Autopilot v1.17.0 DataValidator strategy adapted for Dima.
 * Upstream: src/modules/sensors/data_validator
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "DataValidator.hpp"

#include <cmath>

namespace dima::lib::sensors::validation {

DataValidator::DataValidator(std::uint32_t timeout_us,
                             std::uint32_t equal_value_threshold) noexcept
    : timeout_us_(timeout_us),
      equal_value_threshold_(equal_value_threshold)
{
}

bool DataValidator::put(std::uint64_t timestamp,
                        const float (&values)[3],
                        std::uint32_t error_count) noexcept
{
    /* 同一 DMA 批次内多帧可合法共享到达时间；只拒绝 0 或向后跳变，不要求严格
     * 单调递增。 */
    if (timestamp == 0U ||
        (time_last_us_ != 0U && timestamp < time_last_us_)) {
        reject(StreamFailureTimestamp);
        return false;
    }
    for (float value : values) {
        if (!std::isfinite(value)) {
            reject(StreamFailureInvalidValue);
            return false;
        }
    }

    /* 底层累计 error_count 增量直接加入 density；无新增错误的每个有效样本令
     * density 衰减 1，形成按“样本数”而非墙钟时间的滑动健康近似。 */
    if (error_count > error_count_) {
        increase_error_density(error_count - error_count_);
    } else if (error_density_ > 0U) {
        --error_density_;
    }
    error_count_ = error_count;

    if (time_last_us_ != 0U && vectors_equal(values_, values)) {
        if (equal_value_count_ != UINT32_MAX) {
            ++equal_value_count_;
        }
    } else {
        equal_value_count_ = 0U;
    }

    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        values_[axis] = values[axis];
    }
    time_last_us_ = timestamp;
    input_failure_mask_ = StreamFailureNone;
    if (event_count_ != UINT64_MAX) {
        ++event_count_;
    }
    return true;
}

void DataValidator::reject(std::uint32_t failure_mask,
                           std::uint32_t error_increment) noexcept
{
    input_failure_mask_ = failure_mask == StreamFailureNone
                              ? StreamFailureInvalidValue
                              : failure_mask;
    if (error_increment != 0U) {
        increase_error_density(error_increment);
    }
    if (event_count_ != UINT64_MAX) {
        ++event_count_;
    }
}

StreamValidity DataValidator::evaluate(std::uint64_t now_us) const noexcept
{
    std::uint32_t failures = input_failure_mask_;
    if (time_last_us_ == 0U) {
        failures |= StreamFailureNoData;
    } else if (now_us < time_last_us_) {
        failures |= StreamFailureTimestamp;
    } else if (now_us - time_last_us_ > timeout_us_) {
        failures |= StreamFailureTimeout;
    }
    /* 三轴连续相等（每轴差 <1e-6）超过阈值判 stale；等于阈值仍允许，下一样本
     * 才置位，保持与 PX4 策略的边界一致。 */
    if (equal_value_count_ > equal_value_threshold_) {
        failures |= StreamFailureStaleData;
    }
    if (error_count_ > kNoReturnErrorCount) {
        failures |= StreamFailureHighErrorCount;
    }
    if (error_density_ >= kErrorDensityWindow) {
        failures |= StreamFailureHighErrorDensity;
    }

    float confidence = 0.0F;
    if (failures == StreamFailureNone) {
        /* 无硬失败时 confidence=1-density/100；density 达 100 已先置硬失败，
         * 因此健康返回值保持 (0,1]。 */
        confidence = 1.0F - static_cast<float>(error_density_) /
                                 static_cast<float>(kErrorDensityWindow);
    }
    return {failures, confidence};
}

void DataValidator::reset() noexcept
{
    time_last_us_ = 0U;
    event_count_ = 0U;
    error_count_ = 0U;
    error_density_ = 0U;
    equal_value_count_ = 0U;
    input_failure_mask_ = StreamFailureNone;
    values_[0] = 0.0F;
    values_[1] = 0.0F;
    values_[2] = 0.0F;
}

bool DataValidator::vectors_equal(const float (&left)[3],
                                  const float (&right)[3]) noexcept
{
    constexpr float kEqualEpsilon = 1.0e-6F;
    return std::fabs(left[0] - right[0]) < kEqualEpsilon &&
           std::fabs(left[1] - right[1]) < kEqualEpsilon &&
           std::fabs(left[2] - right[2]) < kEqualEpsilon;
}

void DataValidator::increase_error_density(std::uint32_t increment) noexcept
{
    /* 饱和在 window+1，既防加法回绕，也保留“已越过阈值”的稳定状态。 */
    constexpr std::uint32_t kSaturation = kErrorDensityWindow + 1U;
    if (increment >= kSaturation ||
        error_density_ >= kSaturation - increment) {
        error_density_ = kSaturation;
    } else {
        error_density_ += increment;
    }
}

} // namespace dima::lib::sensors::validation
