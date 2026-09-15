#pragma once

#include "actuator_output_status.hpp"
#include "uORB/SubscriptionData.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::sensors {

// 磁样本与已应用输出的有界时间对齐；学习端和前端共用同一判据。
class MagMotorOutputHistory final {
public:
    void reset() noexcept;
    void update() noexcept;
    bool forward_output(std::uint64_t sample_time, float &longitudinal) const noexcept;

private:
    struct Sample { std::uint64_t timestamp{}; float right{}, left{}; bool valid{}; };
    uORB::SubscriptionData<actuator_output_status_s> subscription_{ORB_ID(actuator_output_status)};
    Sample samples_[32]{};
    std::size_t next_{}, count_{};
    std::uint32_t sequence_{};
    bool have_sequence_{};
};

} // namespace dima::modules::sensors
