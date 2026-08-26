#define MODULE_NAME "sensor_cal"
#include "SensorCalibration.hpp"

#include "logging/logging.hpp"
#include "SensorRotation.hpp"

#include <cmath>

namespace dima::modules::sensors {
namespace {

constexpr const char *kAccelSideNames[6]{
    "back", "front", "left", "right", "up", "down"};

template<typename Side>
void report_pending_sides(const Side (&sides)[6]) noexcept
{
    PX4_INFO_RAW("[cal] pending:%s%s%s%s%s%s",
                 sides[0].done ? "" : " back",
                 sides[1].done ? "" : " front",
                 sides[2].done ? "" : " left",
                 sides[3].done ? "" : " right",
                 sides[4].done ? "" : " up",
                 sides[5].done ? "" : " down");
}

} // namespace

int SensorCalibration::classify_accel_side(
    const sensor_accel_s &sample) const noexcept
{
    if (!algorithms::finite(sample.x, sample.y, sample.z)) return -1;
    const double values[3]{sample.x, sample.y, sample.z};
    // 静止时 |a| 应在 8.0..11.5 m/s^2，主轴绝对分量至少占模长 88%；返回
    // 2*axis+(negative?1:0)，对应 back/front/left/right/up/down 六个方向。
    const double magnitude = algorithms::norm(values[0], values[1],
                                               values[2]);
    if (magnitude < 8.0 || magnitude > 11.5) return -1;
    std::size_t dominant = 0U;
    for (std::size_t axis = 1U; axis < 3U; ++axis) {
        if (std::fabs(values[axis]) > std::fabs(values[dominant])) {
            dominant = axis;
        }
    }
    if (std::fabs(values[dominant]) < magnitude * 0.88) return -1;
    return static_cast<int>(dominant * 2U +
                            (values[dominant] < 0.0 ? 1U : 0U));
}

void SensorCalibration::reset_accel_candidate() noexcept
{
    accel_candidate_ = -1;
    accel_candidate_stats_.reset();
}

void SensorCalibration::process_accel(std::uint64_t now) noexcept
{
    if (now - started_us_ > kAccelTimeoutUs) {
        fail("accelerometer six-side timeout");
        return;
    }
    if (!fresh(now, sensor_accel_.timestamp, kSensorFreshnessUs) ||
        !fresh(now, sensor_gyro_.timestamp, kSensorFreshnessUs)) {
        if (now - started_us_ > kSensorFreshnessUs) {
            fail("accelerometer or gyro data timeout");
        }
        return;
    }
    const std::uint64_t sample_time = sensor_accel_.timestamp_sample != 0U
        ? sensor_accel_.timestamp_sample : sensor_accel_.timestamp;
    if (sample_time <= last_sample_us_) return;
    last_sample_us_ = sample_time;
    if (sensor_accel_.device_id != device_id_) {
        fail("accelerometer identity changed");
        return;
    }
    // 陀螺模长超过 0.15 rad/s 说明车辆未静止，丢弃当前候选面的累计样本。
    if (algorithms::norm(sensor_gyro_.x, sensor_gyro_.y,
                         sensor_gyro_.z) > 0.15) {
        reset_accel_candidate();
        return;
    }
    // 原始 accel 先按 SENS_BOARD_ROT 转到机体系再分类；拟合输出仍由算法结合
    // rotation matrix 转回传感器校准参数语义。
    const dima::lib::sensors::Vector3 body_accel =
        dima::lib::sensors::rotate(
            board_rotation_matrix_,
            {sensor_accel_.x, sensor_accel_.y, sensor_accel_.z});
    sensor_accel_s body_sample = sensor_accel_;
    body_sample.x = body_accel.x;
    body_sample.y = body_accel.y;
    body_sample.z = body_accel.z;
    const int side = classify_accel_side(body_sample);
    if (side < 0 || accel_sides_[side].done) {
        reset_accel_candidate();
        return;
    }
    if (accel_candidate_ != side) {
        reset_accel_candidate();
        accel_candidate_ = side;
        PX4_INFO_RAW("[cal] %s orientation detected", kAccelSideNames[side]);
        PX4_INFO_RAW("[cal] %s orientation detected", kAccelSideNames[side]);
    }
    accel_candidate_stats_.add(body_accel.x, body_accel.y, body_accel.z);
    if (accel_candidate_stats_.count() < kAccelSideSamples) return;

    // 每面 75 个样本的三轴样本方差都必须 <=0.09 (m/s^2)^2，否则视为晃动。
    const auto variance = accel_candidate_stats_.variance();
    if (variance.x > 0.09 || variance.y > 0.09 || variance.z > 0.09) {
        reset_accel_candidate();
        return;
    }
    accel_sides_[side].stats = accel_candidate_stats_;
    accel_sides_[side].done = true;
    PX4_INFO_RAW("[cal] %s side done, rotate to a different side",
                 kAccelSideNames[side]);
    PX4_INFO_RAW("[cal] %s side done, rotate to a different side",
                 kAccelSideNames[side]);
    std::uint32_t done = 0U;
    for (const auto &entry : accel_sides_) done += entry.done ? 1U : 0U;
    update_progress(static_cast<std::uint8_t>(done * 90U / 6U), now);
    reset_accel_candidate();
    if (done != 6U) {
        report_pending_sides(accel_sides_);
        PX4_INFO_RAW("[cal] hold vehicle still on a pending side");
        return;
    }

    // 六面均值送入固定内存六面拟合，求 offset 与 diagonal scale；任何设备
    // 身份变化、尺度越界或奇异解都会在提交前失败。
    algorithms::Vector3d measurements[6]{};
    for (std::size_t side_index = 0U; side_index < 6U; ++side_index) {
        measurements[side_index] = accel_sides_[side_index].stats.mean();
    }
    algorithms::Vector3d offset{};
    algorithms::Vector3d scale{};
    if (!algorithms::fit_accel_six_side(
            measurements, board_rotation_matrix_, offset, scale)) {
        fail("accelerometer fit outside limits");
        return;
    }
    if (!commit_accel(offset, scale, device_id_)) {
        fail("unable to commit accelerometer parameters");
        return;
    }
    begin_wait_for_apply(now);
}

} // namespace dima::modules::sensors
