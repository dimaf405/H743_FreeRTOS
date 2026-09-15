#include "VehicleMagnetometer.hpp"

#include <cmath>

namespace dima::modules::sensors {

void VehicleMagnetometer::apply_throttle_compensation(
    float (&gauss)[3], std::uint64_t sample_timestamp) noexcept
{
    px4::AtomicTransaction atomic;
    const auto &c = active_configuration_;
    if (c.throttle_device_id <= 0 || c.throttle_generation <= 0 ||
        static_cast<std::uint32_t>(c.throttle_device_id) != active_device_id_ ||
        !active_correction_.saved) return;
    // 即使校正更新仍在等待 Disarmed，核心已原子清 ID：逐样本核对标识/代次，
    // 避免参数通知排队的短窗口继续套用旧补偿。这里只读，不从传感器写参数。
    std::int32_t id{}, generation{};
    if (param_get(mot_id_.handle(), &id) != 0 || param_get(mot_generation_.handle(), &generation) != 0 ||
        id != c.throttle_device_id || generation != c.throttle_generation) return;
    float u{};
    if (!motor_history_.forward_output(sample_timestamp, u)) return;
    for (float coefficient : c.throttle_coefficient) if (!std::isfinite(coefficient)) return;
    // K 定义在最终机体系、高斯/单位实际输出；仅两轮均非负的学习域应用 B-K*u。
    // 原地转向、倒退、未来/过期或未确认输出均不套用直线回归模型。
    for (unsigned axis = 0U; axis < 3U; ++axis)
        gauss[axis] -= c.throttle_coefficient[axis] * u;
}

bool VehicleMagnetometer::throttle_compensation_matches(
    std::uint32_t instance, std::int32_t device_id, std::int32_t generation,
    const float (&coefficient)[3]) const noexcept
{
    px4::AtomicTransaction atomic;
    if (!calibration_parameter_update_applied(instance) ||
        active_configuration_.throttle_device_id != device_id ||
        active_configuration_.throttle_generation != generation) return false;
    for (unsigned i = 0U; i < 3U; ++i)
        if (active_configuration_.throttle_coefficient[i] != coefficient[i]) return false;
    return true;
}

} // namespace dima::modules::sensors
