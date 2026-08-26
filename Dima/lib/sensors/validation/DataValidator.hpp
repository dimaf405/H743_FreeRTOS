/****************************************************************************
 * PX4-Autopilot v1.17.0 DataValidator strategy adapted for Dima.
 * Upstream: src/modules/sensors/data_validator
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::lib::sensors::validation {

inline constexpr std::uint32_t StreamFailureNone = 0U;
inline constexpr std::uint32_t StreamFailureNoData = 1U << 0U;
inline constexpr std::uint32_t StreamFailureStaleData = 1U << 1U;
inline constexpr std::uint32_t StreamFailureTimeout = 1U << 2U;
inline constexpr std::uint32_t StreamFailureHighErrorCount = 1U << 3U;
inline constexpr std::uint32_t StreamFailureHighErrorDensity = 1U << 4U;
inline constexpr std::uint32_t StreamFailureInvalidValue = 1U << 5U;
inline constexpr std::uint32_t StreamFailureTimestamp = 1U << 6U;

struct StreamValidity {
    std::uint32_t failure_mask{StreamFailureNoData};
    float confidence{0.0F};

    constexpr bool healthy() const noexcept
    {
        return failure_mask == StreamFailureNone && confidence > 0.0F;
    }
};

class DataValidator final {
public:
    /* timeout/equal threshold 判断数据时效与卡值；error_density 是短期健康量，
     * error_count 是底层累计计数。二者语义不可互换。 */
    static constexpr std::uint32_t kDefaultTimeoutUs = 40000U;
    static constexpr std::uint32_t kDefaultEqualValueThreshold = 100U;
    static constexpr std::uint32_t kErrorDensityWindow = 100U;
    static constexpr std::uint32_t kNoReturnErrorCount = 10000U;

    explicit DataValidator(
        std::uint32_t timeout_us = kDefaultTimeoutUs,
        std::uint32_t equal_value_threshold =
            kDefaultEqualValueThreshold) noexcept;

    bool put(std::uint64_t timestamp, const float (&values)[3],
             std::uint32_t error_count) noexcept;
    void reject(std::uint32_t failure_mask,
                std::uint32_t error_increment = 1U) noexcept;
    StreamValidity evaluate(std::uint64_t now_us) const noexcept;
    void reset() noexcept;

    constexpr std::uint32_t error_count() const noexcept
    {
        return error_count_;
    }

    constexpr std::uint32_t error_density() const noexcept
    {
        return error_density_;
    }

private:
    static bool vectors_equal(const float (&left)[3],
                              const float (&right)[3]) noexcept;
    void increase_error_density(std::uint32_t increment) noexcept;

    std::uint32_t timeout_us_{kDefaultTimeoutUs};
    std::uint32_t equal_value_threshold_{kDefaultEqualValueThreshold};
    std::uint64_t time_last_us_{0U};
    std::uint64_t event_count_{0U};
    std::uint32_t error_count_{0U};
    std::uint32_t error_density_{0U};
    std::uint32_t equal_value_count_{0U};
    std::uint32_t input_failure_mask_{StreamFailureNone};
    float values_[3]{};
};

} // namespace dima::lib::sensors::validation
