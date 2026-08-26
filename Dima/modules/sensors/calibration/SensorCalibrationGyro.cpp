#include "SensorCalibration.hpp"

#include <algorithm>

namespace dima::modules::sensors {

void SensorCalibration::process_gyro(std::uint64_t now) noexcept
{
    // 只消费严格递增且来自启动时锁定 device_id 的新样本；超过 500 ms 无数据
    // 失败，重复 uORB 样本不重复计数。
    if (!fresh(now, sensor_gyro_.timestamp, kSensorFreshnessUs)) {
        if (now - started_us_ > kSensorFreshnessUs) fail("gyro data timeout");
        return;
    }
    const std::uint64_t sample_time = sensor_gyro_.timestamp_sample != 0U
        ? sensor_gyro_.timestamp_sample : sensor_gyro_.timestamp;
    if (sample_time <= last_sample_us_) return;
    last_sample_us_ = sample_time;
    if (sensor_gyro_.device_id != device_id_ ||
        !algorithms::finite(sensor_gyro_.x, sensor_gyro_.y,
                           sensor_gyro_.z)) {
        fail("gyro identity or sample invalid");
        return;
    }
    // 瞬时角速度模长 >0.35 rad/s 清空窗口，要求重新收集连续静止样本。
    if (algorithms::norm(sensor_gyro_.x, sensor_gyro_.y,
                         sensor_gyro_.z) > 0.35) {
        gyro_stats_.reset();
        return;
    }
    gyro_stats_.add(sensor_gyro_.x, sensor_gyro_.y, sensor_gyro_.z);
    update_progress(static_cast<std::uint8_t>(std::min<std::uint32_t>(
        90U, gyro_stats_.count() * 90U / kGyroSamples)), now);
    if (gyro_stats_.count() < kGyroSamples) return;

    // 150 个样本后每轴样本方差需 <=4e-4 (rad/s)^2，均值模长 <=0.35 rad/s；
    // 合格均值就是零偏 offset，陀螺校准不引入 scale。
    const auto variance = gyro_stats_.variance();
    const auto mean = gyro_stats_.mean();
    if (variance.x > 4.0e-4 || variance.y > 4.0e-4 ||
        variance.z > 4.0e-4 ||
        algorithms::norm(mean.x, mean.y, mean.z) > 0.35) {
        gyro_stats_.reset();
        return;
    }
    if (!commit_gyro(mean, device_id_)) {
        fail("unable to commit gyro parameters");
        return;
    }
    begin_wait_for_apply(now);
}

} // namespace dima::modules::sensors
