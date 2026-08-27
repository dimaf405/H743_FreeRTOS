/****************************************************************************
 * Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * PX4-Autopilot v1.17.0 HIGHRES_IMU and SCALED_IMU stream architecture
 * adapted for the Dima single-IMU/single-magnetometer product.
 * Upstream: src/modules/mavlink/streams/{HIGHRES_IMU,SCALED_IMU}.hpp
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace dima::modules::mavlink {
namespace {

bool fresh(std::uint64_t now, std::uint64_t timestamp,
           std::uint64_t maximum_age) noexcept
{
    return timestamp != 0U && timestamp <= now &&
           now - timestamp <= maximum_age;
}

bool finite3(const float values[3]) noexcept
{
    return std::isfinite(values[0]) && std::isfinite(values[1]) &&
           std::isfinite(values[2]);
}

std::uint16_t saturating_u16(double value,
                              std::uint16_t invalid) noexcept
{
    // UINT16_MAX 保留给“未知”，有效值最多饱和到 UINT16_MAX-1。
    if (!std::isfinite(value) || value < 0.0) return invalid;
    return static_cast<std::uint16_t>(
        std::min(value, static_cast<double>(UINT16_MAX - 1U)));
}

std::int16_t scaled_i16(float value, double multiplier) noexcept
{
    const double scaled = static_cast<double>(value) * multiplier;
    if (!std::isfinite(scaled)) return 0;
    // PX4 的 SCALED_IMU 浮点转 int16 向零截断；转换前先饱和，避免越界强转产生 UB。
    const double truncated = std::trunc(scaled);
    return static_cast<std::int16_t>(std::max(
        static_cast<double>(INT16_MIN),
        std::min(truncated, static_cast<double>(INT16_MAX))));
}

std::int16_t temperature_cdeg(float temperature) noexcept
{
    if (!std::isfinite(temperature)) return 0;
    const double rounded = std::round(static_cast<double>(temperature) *
                                      100.0);
    const std::int16_t scaled = static_cast<std::int16_t>(std::max(
        static_cast<double>(INT16_MIN),
        std::min(rounded, static_cast<double>(INT16_MAX))));
    // MAVLink 用 0 表示温度不可用；真实 0 degC 编码为 1，即 0.01 degC。
    return scaled == 0 ? 1 : scaled;
}

std::uint32_t saturating_u32(double value) noexcept
{
    if (!std::isfinite(value) || value < 0.0) return 0U;
    return static_cast<std::uint32_t>(
        std::min(value, static_cast<double>(UINT32_MAX)));
}

std::int32_t saturating_i32(double value) noexcept
{
    if (!std::isfinite(value)) return 0;
    return static_cast<std::int32_t>(std::max(
        static_cast<double>(INT32_MIN),
        std::min(value, static_cast<double>(INT32_MAX))));
}

std::uint16_t heading_cdeg(float radians) noexcept
{
    if (!std::isfinite(radians)) {
        return 0U;
    }
    constexpr double kRadiansToDegrees =
        180.0 / 3.1415926535897932384626433832795;
    double degrees = std::fmod(
        static_cast<double>(radians) * kRadiansToDegrees, 360.0);
    if (degrees < 0.0) degrees += 360.0;
    // GPS_RAW_INT.yaw 以厘度编码，0 表示不可用，因此正北 0 度必须编码为 36000。
    std::uint16_t result = static_cast<std::uint16_t>(
        std::min(std::round(degrees * 100.0), 35999.0));
    return result == 0U ? 36000U : result;
}

std::uint16_t course_cdeg(float radians) noexcept
{
    // COG 的未知哨兵为 UINT16_MAX，正北可以合法编码为 0，与 yaw 字段语义不同。
    if (!std::isfinite(radians)) return UINT16_MAX;
    constexpr double kRadiansToDegrees =
        180.0 / 3.1415926535897932384626433832795;
    double degrees = std::fmod(
        static_cast<double>(radians) * kRadiansToDegrees, 360.0);
    if (degrees < 0.0) degrees += 360.0;
    return static_cast<std::uint16_t>(
        std::min(std::round(degrees * 100.0), 35999.0));
}

std::uint16_t clamp_error_count(std::uint64_t value) noexcept
{
    return static_cast<std::uint16_t>(
        std::min<std::uint64_t>(value, UINT16_MAX));
}

} // namespace

bool MavlinkService::stream_due(std::uint64_t now, std::uint64_t last_tx,
                                std::int32_t interval_us) noexcept
{
    // 负间隔表示禁用；时钟回退时立即允许一次发送，以重建新的单调节拍基线。
    return interval_us >= 0 &&
           (last_tx == 0U || now < last_tx ||
            now - last_tx >= static_cast<std::uint64_t>(interval_us));
}

void MavlinkService::reset_sensor_streams() noexcept
{
    latest_sensor_accel_ = sensor_accel_s{};
    latest_sensor_gyro_ = sensor_gyro_s{};
    latest_sensor_mag_ = sensor_mag_s{};
    latest_vehicle_imu_ = vehicle_imu_s{};
    latest_vehicle_imu_status_ = vehicle_imu_status_s{};
    latest_vehicle_magnetometer_ = vehicle_magnetometer_s{};
    latest_vehicle_gps_ = sensor_gps_s{};
    latest_estimator_gps_status_ = estimator_gps_status_s{};
    reset_sensor_link_state();
    accel_seen_ = false;
    gyro_seen_ = false;
    mag_seen_ = false;
    gps_seen_ = false;
    mag_health_known_ = false;
    imu_streamable_ = false;
    imu_healthy_ = false;
    mag_healthy_ = false;
    gps_healthy_ = false;
}

void MavlinkService::reset_sensor_link_state() noexcept
{
    reset_configured_streams();
    last_highres_imu_timestamp_us_ = 0U;
    last_highres_mag_timestamp_us_ = 0U;
    last_scaled_imu_timestamp_us_ = 0U;
    last_scaled_mag_timestamp_us_ = 0U;
}

void MavlinkService::update_sensor_topics() noexcept
{
    if (sensor_accel_subscription_.update()) {
        latest_sensor_accel_ = sensor_accel_subscription_.get();
    }
    if (sensor_gyro_subscription_.update()) {
        latest_sensor_gyro_ = sensor_gyro_subscription_.get();
    }
    if (sensor_mag_subscription_.update()) {
        latest_sensor_mag_ = sensor_mag_subscription_.get();
    }
    if (vehicle_imu_subscription_.update()) {
        latest_vehicle_imu_ = vehicle_imu_subscription_.get();
    }
    if (vehicle_imu_status_subscription_.update()) {
        latest_vehicle_imu_status_ =
            vehicle_imu_status_subscription_.get();
    }
    if (vehicle_magnetometer_subscription_.update()) {
        latest_vehicle_magnetometer_ =
            vehicle_magnetometer_subscription_.get();
    }
    if (vehicle_gps_subscription_.update()) {
        latest_vehicle_gps_ = vehicle_gps_subscription_.get();
    }
    if (estimator_gps_status_subscription_.update()) {
        latest_estimator_gps_status_ =
            estimator_gps_status_subscription_.get();
    }

    const bool accel_sample_present = latest_sensor_accel_.device_id != 0U &&
        latest_sensor_accel_.timestamp != 0U;
    const bool gyro_sample_present = latest_sensor_gyro_.device_id != 0U &&
        latest_sensor_gyro_.timestamp != 0U;
    const bool imu_sample_valid =
        latest_vehicle_imu_.timestamp != 0U &&
        latest_vehicle_imu_.timestamp_sample != 0U &&
        latest_vehicle_imu_.accel_device_id != 0U &&
        latest_vehicle_imu_.gyro_device_id != 0U &&
        /* PX4 calibration_count is a calibration-change counter.
         * 零表示启用 identity calibration，不能据此把 IMU 判为不健康。 */
        latest_vehicle_imu_.delta_angle_dt != 0U &&
        latest_vehicle_imu_.delta_velocity_dt != 0U &&
        finite3(latest_vehicle_imu_.delta_angle) &&
        finite3(latest_vehicle_imu_.delta_velocity);
    const bool mag_sample_valid =
        latest_vehicle_magnetometer_.device_id != 0U &&
        latest_vehicle_magnetometer_.timestamp != 0U &&
        finite3(latest_vehicle_magnetometer_.magnetometer_ga);
    /* 接收机检测只证明 transport/data 链路，不证明已经获得定位解；室内或
     * 尚未收敛时，已连接的 UM982 仍须以 NO_FIX 和未知位置保持可见。 */
    const bool gps_sample_valid = latest_vehicle_gps_.device_id != 0U &&
        latest_vehicle_gps_.timestamp != 0U;
    const bool imu_status_valid =
        latest_vehicle_imu_status_.accel_device_id != 0U &&
        latest_vehicle_imu_status_.gyro_device_id != 0U &&
        latest_vehicle_imu_status_.accel_device_id ==
            latest_sensor_accel_.device_id &&
        latest_vehicle_imu_status_.gyro_device_id ==
            latest_sensor_gyro_.device_id &&
        latest_vehicle_imu_.accel_device_id ==
            latest_vehicle_imu_status_.accel_device_id &&
        latest_vehicle_imu_.gyro_device_id ==
            latest_vehicle_imu_status_.gyro_device_id;
    const bool gps_status_valid =
        latest_estimator_gps_status_.timestamp != 0U &&
        latest_estimator_gps_status_.timestamp_sample != 0U &&
        latest_estimator_gps_status_.timestamp ==
            latest_vehicle_gps_.timestamp &&
        latest_estimator_gps_status_.timestamp_sample ==
            latest_vehicle_gps_.timestamp_sample;

    // seen 是“本次启动曾检测到”的存在性锁存；healthy 是结合设备 ID、状态 Topic
    // 同源关系和新鲜度计算的当前健康值，两者不得混为一谈。
    accel_seen_ = accel_seen_ || accel_sample_present;
    gyro_seen_ = gyro_seen_ || gyro_sample_present;
    mag_seen_ = mag_seen_ || mag_sample_valid;
    gps_seen_ = gps_seen_ || gps_sample_valid;
    imu_streamable_ = imu_sample_valid;

    /* 高优先级传感器任务可能在进入本函数后、Topic 复制完成前发布新样本。
     * 所有复制完成后再统一采样健康时钟，避免把新 IMU/磁力计/GPS 样本
     * 短暂误判为“来自未来”。 */
    const std::uint64_t health_now = hrt_absolute_time();
    const bool imu_now = imu_sample_valid && imu_status_valid &&
        fresh(health_now, latest_vehicle_imu_.timestamp,
              kImuFreshnessUs) &&
        fresh(health_now, latest_vehicle_imu_status_.timestamp,
              kImuStatusFreshnessUs);
    const bool mag_now = mag_sample_valid && fresh(
        health_now, latest_vehicle_magnetometer_.timestamp, kMagFreshnessUs);
    const bool gps_now = gps_sample_valid && gps_status_valid &&
        fresh(health_now, latest_vehicle_gps_.timestamp, kGpsFreshnessUs) &&
        fresh(health_now, latest_estimator_gps_status_.timestamp,
              kGpsStatusFreshnessUs);
    if (mag_seen_ && (!mag_health_known_ || mag_now != mag_healthy_)) {
        if (mag_now) {
            PX4_INFO("Magnetometer detected device_id=%lu",
                     static_cast<unsigned long>(
                         latest_vehicle_magnetometer_.device_id));
        } else if (mag_health_known_) {
            PX4_WARN("Magnetometer data timeout");
        }
        mag_health_known_ = true;
    }
    imu_healthy_ = imu_now;
    mag_healthy_ = mag_now;
    gps_healthy_ = gps_now;
}

void MavlinkService::report_sensor_link_summary() noexcept
{
    if (!mag_seen_) {
        PX4_WARN("Sensor status: magnetometer not detected");
    } else {
        PX4_INFO("Sensor status: magnetometer %s device_id=%lu",
                 mag_healthy_ ? "healthy" : "stale",
                 static_cast<unsigned long>(
                     latest_vehicle_magnetometer_.device_id));
    }
}

bool MavlinkService::send_highres_imu(std::uint64_t) noexcept
{
    // HIGHRES_IMU 由选中的 vehicle_imu 驱动；仅磁力计更新不能伪造新的 IMU 时间戳。
    if (!imu_streamable_ || latest_vehicle_imu_.timestamp ==
        last_highres_imu_timestamp_us_) {
        return false;
    }

    mavlink_highres_imu_t imu{};
    imu.time_usec = latest_vehicle_imu_.timestamp_sample;
    imu.id = 0U;

    // vehicle_imu 保存积分量：角速度=delta_angle/dt，加速度=delta_velocity/dt；
    // dt 从微秒转换为秒。
    const float angle_dt = static_cast<float>(
        latest_vehicle_imu_.delta_angle_dt) * 1.0e-6F;
    const float velocity_dt = static_cast<float>(
        latest_vehicle_imu_.delta_velocity_dt) * 1.0e-6F;
    imu.xacc = latest_vehicle_imu_.delta_velocity[0] / velocity_dt;
    imu.yacc = latest_vehicle_imu_.delta_velocity[1] / velocity_dt;
    imu.zacc = latest_vehicle_imu_.delta_velocity[2] / velocity_dt;
    imu.xgyro = latest_vehicle_imu_.delta_angle[0] / angle_dt;
    imu.ygyro = latest_vehicle_imu_.delta_angle[1] / angle_dt;
    imu.zgyro = latest_vehicle_imu_.delta_angle[2] / angle_dt;
    imu.fields_updated |= HIGHRES_IMU_UPDATED_XACC |
        HIGHRES_IMU_UPDATED_YACC | HIGHRES_IMU_UPDATED_ZACC |
        HIGHRES_IMU_UPDATED_XGYRO | HIGHRES_IMU_UPDATED_YGYRO |
        HIGHRES_IMU_UPDATED_ZGYRO;
    // PX4 HIGHRES_IMU 的温度来自 vehicle_air_data；本产品无该前端，故保持不可用，
    // 实际 IMU 温度通过 SCALED_IMU 提供。
    const bool mag_streamable =
        latest_vehicle_magnetometer_.device_id != 0U &&
        latest_vehicle_magnetometer_.timestamp != 0U &&
        finite3(latest_vehicle_magnetometer_.magnetometer_ga);
    // 只要磁力计曾发布有效数值就携带最新值；新鲜度只影响 SYS_STATUS.health，
    // 不能从 HIGHRES_IMU 擦除仍有诊断价值的最后原始样本。
    if (mag_streamable) {
        imu.xmag = latest_vehicle_magnetometer_.magnetometer_ga[0];
        imu.ymag = latest_vehicle_magnetometer_.magnetometer_ga[1];
        imu.zmag = latest_vehicle_magnetometer_.magnetometer_ga[2];
        if (latest_vehicle_magnetometer_.timestamp !=
            last_highres_mag_timestamp_us_) {
            imu.fields_updated |= HIGHRES_IMU_UPDATED_XMAG |
                HIGHRES_IMU_UPDATED_YMAG | HIGHRES_IMU_UPDATED_ZMAG;
        }
    }

    mavlink_message_t message{};
    mavlink_msg_highres_imu_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                   &message, &imu);
    const bool sent = send_message(message);
    if (sent) {
        last_highres_imu_timestamp_us_ = latest_vehicle_imu_.timestamp;
        if (mag_streamable) {
            last_highres_mag_timestamp_us_ =
                latest_vehicle_magnetometer_.timestamp;
        }
    }
    return sent;
}

bool MavlinkService::send_scaled_imu(std::uint64_t) noexcept
{
    const bool raw_mag_streamable = latest_sensor_mag_.device_id != 0U &&
        latest_sensor_mag_.timestamp != 0U &&
        std::isfinite(latest_sensor_mag_.x) &&
        std::isfinite(latest_sensor_mag_.y) &&
        std::isfinite(latest_sensor_mag_.z);
    const bool imu_updated = imu_streamable_ && latest_vehicle_imu_.timestamp !=
        last_scaled_imu_timestamp_us_;
    const bool mag_updated = raw_mag_streamable &&
        latest_sensor_mag_.timestamp !=
        last_scaled_mag_timestamp_us_;
    if (!imu_updated && !mag_updated) return false;

    mavlink_scaled_imu_t imu{};
    float temperature = std::numeric_limits<float>::quiet_NaN();
    const bool imu_status_matches =
        latest_vehicle_imu_status_.accel_device_id ==
            latest_vehicle_imu_.accel_device_id &&
        latest_vehicle_imu_status_.gyro_device_id ==
            latest_vehicle_imu_.gyro_device_id;
    if (imu_streamable_) {
        const float angle_dt = static_cast<float>(
            latest_vehicle_imu_.delta_angle_dt) * 1.0e-6F;
        const float velocity_dt = static_cast<float>(
            latest_vehicle_imu_.delta_velocity_dt) * 1.0e-6F;
        if (angle_dt > 0.0F && velocity_dt > 0.0F) {
            // SCALED_IMU 加速度单位为 mg：m/s^2 * 1000 / 标准重力；
            // 角速度单位为 mrad/s：rad/s * 1000。
            constexpr double kMilligravityPerMeterPerSecondSquared =
                1000.0 / 9.80665;
            imu.xacc = scaled_i16(
                latest_vehicle_imu_.delta_velocity[0] / velocity_dt,
                kMilligravityPerMeterPerSecondSquared);
            imu.yacc = scaled_i16(
                latest_vehicle_imu_.delta_velocity[1] / velocity_dt,
                kMilligravityPerMeterPerSecondSquared);
            imu.zacc = scaled_i16(
                latest_vehicle_imu_.delta_velocity[2] / velocity_dt,
                kMilligravityPerMeterPerSecondSquared);
            imu.xgyro = scaled_i16(
                latest_vehicle_imu_.delta_angle[0] / angle_dt, 1000.0);
            imu.ygyro = scaled_i16(
                latest_vehicle_imu_.delta_angle[1] / angle_dt, 1000.0);
            imu.zgyro = scaled_i16(
                latest_vehicle_imu_.delta_angle[2] / angle_dt, 1000.0);
            imu.time_boot_ms = static_cast<std::uint32_t>(
                latest_vehicle_imu_.timestamp / 1000ULL);
        }
        if (imu_status_matches && std::isfinite(
                latest_vehicle_imu_status_.temperature_accel)) {
            temperature = latest_vehicle_imu_status_.temperature_accel;
        } else if (imu_status_matches && std::isfinite(
                       latest_vehicle_imu_status_.temperature_gyro)) {
            temperature = latest_vehicle_imu_status_.temperature_gyro;
        }
    }

    // SCALED_IMU 是原始传感器流，磁场单位由 gauss * 1000 转为 mgauss；
    // 与 SYS_STATUS 的当前健康判定独立，保留最后一次可诊断原始值。
    if (raw_mag_streamable) {
        if (imu.time_boot_ms == 0U) {
            imu.time_boot_ms = static_cast<std::uint32_t>(
                latest_sensor_mag_.timestamp / 1000ULL);
        }
        imu.xmag = scaled_i16(latest_sensor_mag_.x, 1000.0);
        imu.ymag = scaled_i16(latest_sensor_mag_.y, 1000.0);
        imu.zmag = scaled_i16(latest_sensor_mag_.z, 1000.0);
        if (!std::isfinite(temperature) &&
            std::isfinite(latest_sensor_mag_.temperature)) {
            temperature = latest_sensor_mag_.temperature;
        }
    }
    imu.temperature = temperature_cdeg(temperature);

    mavlink_message_t message{};
    mavlink_msg_scaled_imu_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                  &message, &imu);
    const bool sent = send_message(message);
    if (sent) {
        if (imu_streamable_) {
            last_scaled_imu_timestamp_us_ = latest_vehicle_imu_.timestamp;
        }
        if (raw_mag_streamable) {
            last_scaled_mag_timestamp_us_ = latest_sensor_mag_.timestamp;
        }
    }
    return sent;
}

bool MavlinkService::send_gps_raw_int(std::uint64_t now) noexcept
{
    if (!gps_seen_) return false;
    const auto &gps = latest_vehicle_gps_;
    mavlink_gps_raw_int_t raw{};
    raw.time_usec = gps_healthy_
        ? (gps.time_utc_usec != 0U ? gps.time_utc_usec
                                   : gps.timestamp_sample)
        : now;
    raw.fix_type = gps_healthy_ ? gps.fix_type : GPS_FIX_TYPE_NO_GPS;
    // 线协议单位：经纬度 1e-7 deg，高度 mm，DOP/速度 1e-2，精度 mm，
    // heading accuracy 为 1e-5 deg。链路不健康时使用各字段规定的未知哨兵。
    raw.lat = gps_healthy_ ? saturating_i32(gps.latitude_deg * 1.0e7) : 0;
    raw.lon = gps_healthy_ ? saturating_i32(gps.longitude_deg * 1.0e7) : 0;
    raw.alt = gps_healthy_ ? saturating_i32(gps.altitude_msl_m * 1000.0) : 0;
    raw.eph = gps_healthy_ ? saturating_u16(
        static_cast<double>(gps.hdop) * 100.0, UINT16_MAX) : UINT16_MAX;
    raw.epv = gps_healthy_ ? saturating_u16(
        static_cast<double>(gps.vdop) * 100.0, UINT16_MAX) : UINT16_MAX;
    raw.vel = gps_healthy_ ? saturating_u16(
        static_cast<double>(gps.vel_m_s) * 100.0, UINT16_MAX) : UINT16_MAX;
    raw.cog = gps_healthy_ ? course_cdeg(gps.cog_rad) : UINT16_MAX;
    raw.satellites_visible = gps_healthy_ ? gps.satellites_used : UINT8_MAX;
    raw.alt_ellipsoid = gps_healthy_ ? saturating_i32(
        gps.altitude_ellipsoid_m * 1000.0) : 0;
    raw.h_acc = gps_healthy_ ? saturating_u32(
        static_cast<double>(gps.eph) * 1000.0) : 0U;
    raw.v_acc = gps_healthy_ ? saturating_u32(
        static_cast<double>(gps.epv) * 1000.0) : 0U;
    raw.vel_acc = gps_healthy_ ? saturating_u32(
        static_cast<double>(gps.s_variance_m_s) * 1000.0) : 0U;
    raw.hdg_acc = gps_healthy_ && std::isfinite(gps.heading_accuracy)
        ? saturating_u32(static_cast<double>(gps.heading_accuracy) *
                         (180.0 / 3.1415926535897932384626433832795) *
                         100000.0)
        : 0U;
    raw.yaw = gps_healthy_ ? heading_cdeg(gps.heading) : 0U;

    mavlink_message_t message{};
    mavlink_msg_gps_raw_int_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                   &message, &raw);
    return send_message(message);
}

bool MavlinkService::send_system_status(std::uint64_t) noexcept
{
    const std::uint32_t gyro = MAV_SYS_STATUS_SENSOR_3D_GYRO;
    const std::uint32_t accel = MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    const std::uint32_t mag = MAV_SYS_STATUS_SENSOR_3D_MAG;
    const std::uint32_t gps = MAV_SYS_STATUS_SENSOR_GPS;
    mavlink_sys_status_t status{};
    // present/enabled 表示启动后检测过该设备；health 只表示当前配对关系与新鲜度健康。
    if (gyro_seen_) {
        status.onboard_control_sensors_present |= gyro;
        status.onboard_control_sensors_enabled |= gyro;
    }
    if (accel_seen_) {
        status.onboard_control_sensors_present |= accel;
        status.onboard_control_sensors_enabled |= accel;
    }
    if (mag_seen_) {
        status.onboard_control_sensors_present |= mag;
        status.onboard_control_sensors_enabled |= mag;
    }
    if (gps_seen_) {
        status.onboard_control_sensors_present |= gps;
        status.onboard_control_sensors_enabled |= gps;
    }
    if (imu_healthy_) status.onboard_control_sensors_health |= gyro | accel;
    if (mag_healthy_) status.onboard_control_sensors_health |= mag;
    if (gps_healthy_) status.onboard_control_sensors_health |= gps;
    status.voltage_battery = UINT16_MAX;
    status.current_battery = -1;
    status.battery_remaining = -1;
    status.errors_count1 = clamp_error_count(
        static_cast<std::uint64_t>(latest_sensor_accel_.error_count) +
        latest_sensor_gyro_.error_count);
    status.errors_count2 = clamp_error_count(latest_sensor_mag_.error_count);
    status.errors_count3 = clamp_error_count(latest_vehicle_gps_.system_error);

    mavlink_message_t message{};
    mavlink_msg_sys_status_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                  &message, &status);
    return send_message(message);
}

} // namespace dima::modules::mavlink
