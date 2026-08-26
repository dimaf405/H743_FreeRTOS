/****************************************************************************
 * PX4-Autopilot v1.17.0 GNSS/IMU validity checks adapted for Dima.
 * Upstream: EKF GnssChecks, commander accel/gyro checks and VehicleIMU
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "SensorValidityAlgorithms.hpp"

#include <cmath>

namespace dima::lib::sensors::validation {
namespace {

bool optional_nonnegative(float value) noexcept
{
    return !std::isfinite(value) || value >= 0.0F;
}

} // namespace

bool known_fix_type(std::uint8_t fix_type) noexcept
{
    return (fix_type >= 1U && fix_type <= 6U) || fix_type == 8U;
}

GpsStructureResult validate_gps_structure(const GpsSample &sample) noexcept
{
    /* 结构检查验证字段自洽与单位范围，不等价于“导航解算健康”。NO_FIX/仅时间
     * 状态允许位置类字段为 NaN；fix>=2 后才要求位置、高度、卫星数和 HDOP。 */
    std::uint32_t failures = GpsFailureNone;
    if (sample.device_id == 0U) {
        failures |= GpsFailureDeviceId;
    }
    if (sample.timestamp == 0U || sample.timestamp_sample == 0U ||
        sample.timestamp < sample.timestamp_sample) {
        failures |= GpsFailureTimestamp;
    }
    if (!known_fix_type(sample.fix_type)) {
        failures |= GpsFailureFix;
    }

    const bool has_fix = sample.fix_type >= 2U &&
                         known_fix_type(sample.fix_type);
    if (has_fix) {
        if (!std::isfinite(sample.latitude_deg) ||
            !std::isfinite(sample.longitude_deg) ||
            sample.latitude_deg < -90.0 || sample.latitude_deg > 90.0 ||
            sample.longitude_deg < -180.0 ||
            sample.longitude_deg > 180.0) {
            failures |= GpsFailurePosition;
        }
        if (!std::isfinite(sample.altitude_msl_m) ||
            !std::isfinite(sample.altitude_ellipsoid_m)) {
            failures |= GpsFailureAltitude;
        }
        if (sample.satellites_used == 0U) {
            failures |= GpsFailureSatellites;
        }
        if (!std::isfinite(sample.hdop) || sample.hdop < 0.0F) {
            failures |= GpsFailureDop;
        }
    }

    if (!optional_nonnegative(sample.hdop) ||
        !optional_nonnegative(sample.vdop)) {
        failures |= GpsFailureDop;
    }
    if (!optional_nonnegative(sample.eph) ||
        !optional_nonnegative(sample.epv) ||
        !optional_nonnegative(sample.speed_accuracy_m_s)) {
        failures |= GpsFailureAccuracy;
    }
    if (std::isfinite(sample.velocity_m_s) && sample.velocity_m_s < 0.0F) {
        failures |= GpsFailureVelocity;
    }
    if (sample.velocity_ned_valid &&
        (!std::isfinite(sample.velocity_m_s) ||
         !std::isfinite(sample.velocity_n_m_s) ||
         !std::isfinite(sample.velocity_e_m_s) ||
         !std::isfinite(sample.velocity_d_m_s))) {
        failures |= GpsFailureVelocity;
    }
    if (std::isfinite(sample.course_rad) &&
        (sample.course_rad < -kPi || sample.course_rad > kPi)) {
        failures |= GpsFailureCourse;
    }

    if (std::isfinite(sample.heading_rad)) {
        if (sample.heading_rad < -kPi || sample.heading_rad > kPi ||
            !std::isfinite(sample.heading_accuracy_rad) ||
            sample.heading_accuracy_rad <= 0.0F ||
            sample.heading_accuracy_rad > 2.0F * kPi) {
            failures |= GpsFailureHeading;
        }
    } else if (std::isfinite(sample.heading_accuracy_rad)) {
        failures |= GpsFailureHeading;
    }
    if (std::isfinite(sample.heading_offset_rad) &&
        (sample.heading_offset_rad < -kPi ||
         sample.heading_offset_rad > kPi)) {
        failures |= GpsFailureHeading;
    }
    return {failures};
}

GpsSolutionStatus GpsSolutionChecker::update(
    const GpsSample &sample, std::uint64_t now_us) noexcept
{
    /* 健康驻留时间从“最近一次失败”起算；时钟倒退先 reset，不能用无符号下溢
     * 伪造已经满足 10 s/1 s。 */
    if (time_last_update_us_ != 0U && now_us < time_last_update_us_) {
        reset();
    }
    time_last_update_us_ = now_us;
    if (time_last_failure_us_ == 0U) {
        time_last_failure_us_ = now_us;
    }

    GpsSolutionStatus status = initial_checks_passed_
                                   ? simplified_checks(sample)
                                   : initial_checks(sample);
    if (!no_failures(status)) {
        time_last_failure_us_ = now_us;
        passed_ = false;
    } else {
        const std::uint64_t dwell = initial_checks_passed_
                                        ? kRecoveryHealthTimeUs
                                        : kInitialHealthTimeUs;
        passed_ = now_us >= time_last_failure_us_ &&
                  now_us - time_last_failure_us_ >= dwell;
        if (passed_ && !initial_checks_passed_) {
            initial_checks_passed_ = true;
        }
    }
    status.checks_passed = passed_;
    return status;
}

void GpsSolutionChecker::reset() noexcept
{
    time_last_failure_us_ = 0U;
    time_last_update_us_ = 0U;
    initial_checks_passed_ = false;
    passed_ = false;
}

bool GpsSolutionChecker::spoofing_failure(
    const GpsSample &sample) noexcept
{
    return sample.spoofing_state == kSpoofingDetected ||
           sample.authentication_state == kAuthenticationError;
}

float GpsSolutionChecker::pdop(const GpsSample &sample) noexcept
{
    if (!std::isfinite(sample.hdop) || !std::isfinite(sample.vdop) ||
        sample.hdop < 0.0F || sample.vdop < 0.0F) {
        return std::numeric_limits<float>::infinity();
    }
    /* PDOP = sqrt(HDOP² + VDOP²)，任一缺失/负值返回 +inf 使严格检查失败。 */
    return std::sqrt(sample.hdop * sample.hdop +
                     sample.vdop * sample.vdop);
}

GpsSolutionStatus GpsSolutionChecker::initial_checks(
    const GpsSample &sample) noexcept
{
    GpsSolutionStatus status{};
    /* 初始门限：3D fix、>=6 星、PDOP<=2.5、eph<=3 m、epv<=5 m、
     * 速度精度<=0.5 m/s，且无 spoof/authentication 明确失败。 */
    status.check_fail_fix = sample.fix_type < 3U;
    status.check_fail_min_sat_count = sample.satellites_used < 6U;
    status.check_fail_max_pdop = pdop(sample) > 2.5F;
    status.check_fail_max_horz_err =
        !std::isfinite(sample.eph) || sample.eph > 3.0F;
    status.check_fail_max_vert_err =
        !std::isfinite(sample.epv) || sample.epv > 5.0F;
    status.check_fail_max_spd_err =
        !std::isfinite(sample.speed_accuracy_m_s) ||
        sample.speed_accuracy_m_s > 0.5F;
    status.check_fail_spoofed = spoofing_failure(sample);
    return status;
}

GpsSolutionStatus GpsSolutionChecker::simplified_checks(
    const GpsSample &sample) noexcept
{
    GpsSolutionStatus status{};
    /* 已通过初始检查后采用恢复门限：仍要求 3D fix/无欺骗，但位置误差放宽到
     * 50 m、速度精度到 10 m/s，避免短时精度抖动造成长时间不可用。 */
    status.check_fail_fix = sample.fix_type < 3U;
    status.check_fail_max_horz_err =
        !std::isfinite(sample.eph) || sample.eph > 50.0F;
    status.check_fail_max_vert_err =
        !std::isfinite(sample.epv) || sample.epv > 50.0F;
    status.check_fail_max_spd_err =
        !std::isfinite(sample.speed_accuracy_m_s) ||
        sample.speed_accuracy_m_s > 10.0F;
    status.check_fail_spoofed = spoofing_failure(sample);
    return status;
}

bool GpsSolutionChecker::no_failures(
    const GpsSolutionStatus &status) noexcept
{
    return !status.check_fail_fix &&
           !status.check_fail_min_sat_count &&
           !status.check_fail_max_pdop &&
           !status.check_fail_max_horz_err &&
           !status.check_fail_max_vert_err &&
           !status.check_fail_max_spd_err &&
           !status.check_fail_spoofed;
}

ImuValidationResult validate_imu_sample(
    const ImuSample &sample, std::uint32_t expected_device_id) noexcept
{
    // IMU 结构门禁只验证来源、时间因果、有限值、温度和批量样本数；error_count
    // 与 clip_counter 是累计诊断量，健康趋势由 DataValidator/VehicleImu 另行判断。
    std::uint32_t failures = ImuFailureNone;
    if (expected_device_id == 0U || sample.device_id != expected_device_id) {
        failures |= ImuFailureDeviceId;
    }
    if (sample.timestamp == 0U || sample.timestamp_sample == 0U ||
        sample.timestamp < sample.timestamp_sample) {
        failures |= ImuFailureTimestamp;
    }
    if (!std::isfinite(sample.value[0]) ||
        !std::isfinite(sample.value[1]) ||
        !std::isfinite(sample.value[2])) {
        failures |= ImuFailureValue;
    }
    if (!std::isfinite(sample.temperature_c)) {
        failures |= ImuFailureTemperature;
    }
    if (sample.samples == 0U || sample.samples > 10U) {
        failures |= ImuFailureSampleCount;
    }
    return {failures};
}

} // namespace dima::lib::sensors::validation
