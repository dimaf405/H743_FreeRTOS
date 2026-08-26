/****************************************************************************
 * PX4-Autopilot v1.17.0 GNSS/IMU validity checks adapted for Dima.
 * Upstream: EKF GnssChecks, commander accel/gyro checks and VehicleIMU
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cstdint>
#include <limits>

namespace dima::lib::sensors::validation {

inline constexpr float kPi = 3.14159265358979323846F;
inline constexpr std::uint8_t kSpoofingUnknown = 0U;
inline constexpr std::uint8_t kSpoofingDetected = 3U;
inline constexpr std::uint8_t kAuthenticationUnknown = 0U;
inline constexpr std::uint8_t kAuthenticationError = 2U;

inline constexpr std::uint32_t GpsFailureNone = 0U;
inline constexpr std::uint32_t GpsFailureDeviceId = 1U << 0U;
inline constexpr std::uint32_t GpsFailureTimestamp = 1U << 1U;
inline constexpr std::uint32_t GpsFailureFix = 1U << 2U;
inline constexpr std::uint32_t GpsFailurePosition = 1U << 3U;
inline constexpr std::uint32_t GpsFailureAltitude = 1U << 4U;
inline constexpr std::uint32_t GpsFailureDop = 1U << 5U;
inline constexpr std::uint32_t GpsFailureAccuracy = 1U << 6U;
inline constexpr std::uint32_t GpsFailureVelocity = 1U << 7U;
inline constexpr std::uint32_t GpsFailureCourse = 1U << 8U;
inline constexpr std::uint32_t GpsFailureHeading = 1U << 9U;
inline constexpr std::uint32_t GpsFailureSatellites = 1U << 10U;

struct GpsSample {
    /* NaN 表示协议未提供的可选量；一旦相应 valid/fix 语义要求该字段，校验器才
     * 将 NaN 判错。角度单位由字段名区分 deg/rad，速度 m/s，高度 m。 */
    std::uint64_t timestamp{0U};
    std::uint64_t timestamp_sample{0U};
    std::uint32_t device_id{0U};
    double latitude_deg{std::numeric_limits<double>::quiet_NaN()};
    double longitude_deg{std::numeric_limits<double>::quiet_NaN()};
    double altitude_msl_m{std::numeric_limits<double>::quiet_NaN()};
    double altitude_ellipsoid_m{std::numeric_limits<double>::quiet_NaN()};
    float eph{std::numeric_limits<float>::quiet_NaN()};
    float epv{std::numeric_limits<float>::quiet_NaN()};
    float hdop{std::numeric_limits<float>::quiet_NaN()};
    float vdop{std::numeric_limits<float>::quiet_NaN()};
    float speed_accuracy_m_s{std::numeric_limits<float>::quiet_NaN()};
    float velocity_m_s{std::numeric_limits<float>::quiet_NaN()};
    float velocity_n_m_s{std::numeric_limits<float>::quiet_NaN()};
    float velocity_e_m_s{std::numeric_limits<float>::quiet_NaN()};
    float velocity_d_m_s{std::numeric_limits<float>::quiet_NaN()};
    float course_rad{std::numeric_limits<float>::quiet_NaN()};
    float heading_rad{std::numeric_limits<float>::quiet_NaN()};
    float heading_offset_rad{std::numeric_limits<float>::quiet_NaN()};
    float heading_accuracy_rad{std::numeric_limits<float>::quiet_NaN()};
    std::uint8_t fix_type{0U};
    std::uint8_t satellites_used{0U};
    std::uint8_t spoofing_state{kSpoofingUnknown};
    std::uint8_t authentication_state{kAuthenticationUnknown};
    bool velocity_ned_valid{false};
};

struct GpsStructureResult {
    std::uint32_t failure_mask{GpsFailureNone};

    constexpr bool valid() const noexcept
    {
        return failure_mask == GpsFailureNone;
    }
};

bool known_fix_type(std::uint8_t fix_type) noexcept;
GpsStructureResult validate_gps_structure(const GpsSample &sample) noexcept;

struct GpsSolutionStatus {
    bool checks_passed{false};
    bool check_fail_fix{false};
    bool check_fail_min_sat_count{false};
    bool check_fail_max_pdop{false};
    bool check_fail_max_horz_err{false};
    bool check_fail_max_vert_err{false};
    bool check_fail_max_spd_err{false};
    bool check_fail_spoofed{false};
};

class GpsSolutionChecker final {
public:
    /* 首次健康需连续 10 s 严格检查；曾健康后的恢复需连续 1 s 简化检查，形成
     * 启动可信度与短时掉星恢复之间的滞回。 */
    static constexpr std::uint64_t kInitialHealthTimeUs = 10000000ULL;
    static constexpr std::uint64_t kRecoveryHealthTimeUs = 1000000ULL;

    GpsSolutionStatus update(const GpsSample &sample,
                             std::uint64_t now_us) noexcept;
    void reset() noexcept;

private:
    static bool spoofing_failure(const GpsSample &sample) noexcept;
    static float pdop(const GpsSample &sample) noexcept;
    static GpsSolutionStatus initial_checks(const GpsSample &sample) noexcept;
    static GpsSolutionStatus simplified_checks(
        const GpsSample &sample) noexcept;
    static bool no_failures(const GpsSolutionStatus &status) noexcept;

    std::uint64_t time_last_failure_us_{0U};
    std::uint64_t time_last_update_us_{0U};
    bool initial_checks_passed_{false};
    bool passed_{false};
};

inline constexpr std::uint32_t ImuFailureNone = 0U;
inline constexpr std::uint32_t ImuFailureDeviceId = 1U << 0U;
inline constexpr std::uint32_t ImuFailureTimestamp = 1U << 1U;
inline constexpr std::uint32_t ImuFailureValue = 1U << 2U;
inline constexpr std::uint32_t ImuFailureTemperature = 1U << 3U;
inline constexpr std::uint32_t ImuFailureSampleCount = 1U << 4U;

struct ImuSample {
    std::uint64_t timestamp{0U};
    std::uint64_t timestamp_sample{0U};
    std::uint32_t device_id{0U};
    float value[3]{};
    float temperature_c{std::numeric_limits<float>::quiet_NaN()};
    std::uint32_t error_count{0U};
    std::uint8_t clip_counter[3]{};
    std::uint8_t samples{0U};
};

struct ImuValidationResult {
    std::uint32_t failure_mask{ImuFailureNone};

    constexpr bool valid() const noexcept
    {
        return failure_mask == ImuFailureNone;
    }
};

ImuValidationResult validate_imu_sample(
    const ImuSample &sample, std::uint32_t expected_device_id) noexcept;

} // namespace dima::lib::sensors::validation
