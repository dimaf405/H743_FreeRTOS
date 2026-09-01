#include "Ekf2.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace dima::modules::ekf2 {

std::int16_t Ekf2::relative_timestamp(
    std::uint64_t timestamp_us, std::uint64_t reference_us) noexcept
{
    // EKF2 timestamps 的线单位是 0.1 ms；32767 是保留的 invalid。超出
    // int16 可表达窗口时显式返回 invalid，不能让强制窄化回绕成伪造新样本。
    const std::int64_t relative =
        static_cast<std::int64_t>(timestamp_us / 100U) -
        static_cast<std::int64_t>(reference_us / 100U);
    if (relative < std::numeric_limits<std::int16_t>::min() ||
        relative >= ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID) {
        return ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
    }
    return static_cast<std::int16_t>(relative);
}

bool Ekf2::update_gps() noexcept
{
    sensor_gps_s gps{};
    if (!gps_sub_.copy(&gps)) {
        return false;
    }

    // PX4 v1.17 的这一入口要求完整 NED 速度；若 UM982 当前只剩位置解，保留
    // 原始 GPS topic 供诊断，但不把不完整向量送入 EKF 观测模型。
    if (!gps.vel_ned_valid) {
        return false;
    }

    const matrix::Vector3f velocity{
        gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s};
    const float altitude_amsl = static_cast<float>(gps.altitude_msl_m);
    const float altitude_ellipsoid =
        static_cast<float>(gps.altitude_ellipsoid_m);
    const float pdop =
        std::sqrt(gps.hdop * gps.hdop + gps.vdop * gps.vdop);

    // UM982 已发布 body_yaw=wrap_pi(raw+pi-GPS_YAW_OFFSET)，同时保留
    // heading_offset。Core 的 GNSS yaw 观测预测使用 yaw+yaw_offset 恢复阵列
    // 方向，而 reset 使用已补偿 yaw；此处绝不再次减安装偏置。
    gnssSample sample{};
    sample.time_us = gps.timestamp;
    sample.lat = gps.latitude_deg;
    sample.lon = gps.longitude_deg;
    sample.alt = altitude_amsl;
    sample.vel = velocity;
    sample.hacc = gps.eph;
    sample.vacc = gps.epv;
    sample.sacc = gps.s_variance_m_s;
    sample.fix_type = gps.fix_type;
    sample.nsats = gps.satellites_used;
    sample.pdop = pdop;
    sample.yaw = gps.heading;
    sample.yaw_acc = gps.heading_accuracy;
    sample.yaw_offset = gps.heading_offset;
    sample.spoofed =
        gps.spoofing_state == sensor_gps_s::SPOOFING_STATE_DETECTED;
    ekf_.setGpsData(sample);

    const float geoid_height = altitude_ellipsoid - altitude_amsl;
    if (std::isfinite(geoid_height)) {
        if (last_geoid_height_update_us_ == 0U) {
            geoid_height_lpf_.reset(geoid_height);
            last_geoid_height_update_us_ = sample.time_us;
        } else if (sample.time_us > last_geoid_height_update_us_) {
            const float dt = static_cast<float>(
                sample.time_us - last_geoid_height_update_us_) * 1.0e-6F;
            geoid_height_lpf_.setParameters(
                dt, kGeoidHeightTimeConstantS);
            geoid_height_lpf_.update(geoid_height);
            last_geoid_height_update_us_ = sample.time_us;
        }
    }
    return true;
}

bool Ekf2::update_magnetometer(ekf2_timestamps_s &timestamps) noexcept
{
    vehicle_magnetometer_s magnetometer{};
    if (!magnetometer_sub_.copy(&magnetometer)) {
        return false;
    }

    bool reset = false;
    if (magnetometer.device_id != mag_device_id_ ||
        magnetometer.calibration_count != mag_calibration_count_) {
        reset = true;
        mag_device_id_ = magnetometer.device_id;
        mag_calibration_count_ = magnetometer.calibration_count;
        mag_cal_ = {};
    }

    // Gauss、FRD 和 timestamp_sample 均已由 VehicleMagnetometer 标准化；reset
    // 告诉 Core 静态校准/设备已变，旧 hard-iron bias 不可跨模型延用。
    const magSample sample{
        magnetometer.timestamp_sample,
        matrix::Vector3f{magnetometer.magnetometer_ga}, reset};
    ekf_.setMagData(sample);
    timestamps.vehicle_magnetometer_timestamp_rel = relative_timestamp(
        magnetometer.timestamp, timestamps.timestamp);
    return true;
}

void Ekf2::update_system_flags(std::uint64_t sample_time_us) noexcept
{
    const bool periodic_due =
        !system_flags_sent_ || last_system_flags_sent_us_ == 0U ||
        sample_time_us < last_system_flags_sent_us_ ||
        sample_time_us - last_system_flags_sent_us_ >= kPeriodicStatusUs;
    if (!periodic_due) {
        return;
    }

    const bool status_fresh =
        have_vehicle_status_ && vehicle_status_.timestamp != 0U &&
        sample_time_us >= vehicle_status_.timestamp &&
        sample_time_us - vehicle_status_.timestamp <= 3000000ULL;
    const bool disarmed =
        status_fresh &&
        vehicle_status_.arming_state ==
            vehicle_status_s::ARMING_STATE_DISARMED;

    systemFlagUpdate flags{};
    flags.time_us = sample_time_us;
    // Rover 永远不是 fixed-wing/in-air。没有 wheel/land-detector 时只把“新鲜且
    // disarmed”视为 at_rest/constant_pos；状态陈旧或已武装一律保守为未知运动，
    // 避免 GNSS 漂移检查和 bias 学习借虚假的静止条件通过。
    flags.in_air = false;
    flags.is_fixed_wing = false;
    flags.at_rest = disarmed;
    flags.constant_pos = disarmed;
    flags.gnd_effect = false;
    ekf_.setSystemFlagData(flags);
    system_flags_sent_ = true;
    last_system_flags_sent_us_ = sample_time_us;
}

} // namespace dima::modules::ekf2
