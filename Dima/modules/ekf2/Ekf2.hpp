/****************************************************************************
 * PX4-Autopilot v1.17.0 EKF2 single-instance adapter for Dima Rover.
 * Core source: src/modules/ekf2 @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "ekf.h"

#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "uORB/Publication.hpp"
#include "uORB/uORB.hpp"
#include "work_queue/WorkQueue.hpp"

#include "ekf2_timestamps.hpp"
#include "estimator_aid_src_fake_hgt.hpp"
#include "estimator_aid_src_fake_pos.hpp"
#include "estimator_aid_src_gnss_hgt.hpp"
#include "estimator_aid_src_gnss_pos.hpp"
#include "estimator_aid_src_gnss_vel.hpp"
#include "estimator_aid_src_gnss_yaw.hpp"
#include "estimator_aid_src_gravity.hpp"
#include "estimator_aid_src_mag.hpp"
#include "estimator_event_flags.hpp"
#include "estimator_gps_status.hpp"
#include "estimator_sensor_bias.hpp"
#include "estimator_status.hpp"
#include "estimator_status_flags.hpp"
#include "parameter_update.hpp"
#include "sensor_gps.hpp"
#include "vehicle_attitude.hpp"
#include "vehicle_global_position.hpp"
#include "vehicle_imu.hpp"
#include "vehicle_local_position.hpp"
#include "vehicle_magnetometer.hpp"
#include "vehicle_odometry.hpp"
#include "vehicle_status.hpp"
#include "yaw_estimator_status.hpp"

#include "AlphaFilter.hpp"

#include <cstdint>

namespace dima::modules::ekf2 {

// N1 只有这一份 EKF2：固定消费 vehicle_imu[0]、vehicle_magnetometer[0] 与
// UM982 vehicle_gps_position[0]，并直接发布主 vehicle_* 估计结果。类中没有
// Selector、实例数组或运行时装卸开关，估计健康也不会反向门控 Manual/PWM。
class Ekf2 final : public dima::middleware::lifecycle::ModuleBase,
                   public px4::ScheduledWorkItem {
public:
    Ekf2() noexcept;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

private:
    static constexpr std::uint32_t kBackupScheduleUs = 100000U;
    static constexpr std::uint64_t kPeriodicStatusUs = 1000000ULL;
    static constexpr std::uint64_t kParameterCommitRetryUs = 1000000ULL;
    static constexpr float kGeoidHeightTimeConstantS = 10.0F;

    struct InFlightCalibration {
        std::uint64_t last_us{0U};
        std::uint64_t total_time_us{0U};
        matrix::Vector3f bias{};
        bool cal_available{false};
    };

    enum class MagDeclinationCommitState : std::uint8_t {
        Idle = 0U,
        Pending,
        Running,
        Succeeded,
        Failed,
    };

    class MagDeclinationCommitWorkItem final
        : public px4::ScheduledWorkItem {
    public:
        explicit MagDeclinationCommitWorkItem(Ekf2 &owner) noexcept;

    private:
        void Run() override;
        Ekf2 &owner_;
    };

    void Run() override;

    bool schedule_backup() noexcept;
    bool load_parameters(bool initial) noexcept;
    bool advertise_outputs() noexcept;
    bool validate_imu(const vehicle_imu_s &imu) const noexcept;
    void update_imu_identity(const vehicle_imu_s &imu) noexcept;
    bool update_gps() noexcept;
    bool update_magnetometer(ekf2_timestamps_s &timestamps) noexcept;
    void update_system_flags(std::uint64_t sample_time_us) noexcept;
    static std::int16_t relative_timestamp(std::uint64_t timestamp_us,
                                           std::uint64_t reference_us) noexcept;

    void publish_attitude(std::uint64_t sample_time_us) noexcept;
    void publish_local_position(std::uint64_t sample_time_us) noexcept;
    void publish_global_position(std::uint64_t sample_time_us) noexcept;
    void publish_odometry(std::uint64_t sample_time_us,
                          const imuSample &imu_sample) noexcept;
    void publish_aid_sources(std::uint64_t now_us) noexcept;
    void publish_event_flags(std::uint64_t now_us) noexcept;
    void publish_gps_status(std::uint64_t now_us) noexcept;
    void publish_sensor_bias(std::uint64_t now_us) noexcept;
    void publish_status(std::uint64_t now_us) noexcept;
    void publish_status_flags(std::uint64_t now_us) noexcept;
    void publish_yaw_estimator_status(std::uint64_t now_us) noexcept;

    void update_calibration(std::uint64_t timestamp,
                            InFlightCalibration &cal,
                            const matrix::Vector3f &bias,
                            const matrix::Vector3f &bias_variance,
                            float bias_limit, bool bias_valid,
                            bool learning_valid) noexcept;
    void update_bias_stability(std::uint64_t timestamp) noexcept;
    void update_mag_declination(std::uint64_t now_us) noexcept;
    void run_mag_declination_commit() noexcept;
    void reset_mag_declination_commit() noexcept;

    template<typename T>
    void publish_aid_source(std::uint64_t now_us, const T &source,
                            std::uint64_t &last_sample,
                            uORB::Publication<T> &publication) noexcept
    {
        if (source.timestamp_sample == 0U ||
            source.timestamp_sample <= last_sample) {
            return;
        }
        // estimator_instance 固定为 0；观测、创新、方差与拒绝状态原样来自
        // PX4 EKF Core，适配层只补发布时钟和单实例身份。
        T output{source};
        output.estimator_instance = 0U;
        output.timestamp = now_us;
        (void)publication.publish(output);
        last_sample = source.timestamp_sample;
    }

    Ekf ekf_{};
    MagDeclinationCommitWorkItem mag_declination_commit_worker_;

    uORB::SubscriptionCallbackWorkItem vehicle_imu_sub_;
    uORB::Subscription gps_sub_{ORB_ID(vehicle_gps_position)};
    uORB::Subscription magnetometer_sub_{ORB_ID(vehicle_magnetometer)};
    uORB::Subscription vehicle_status_sub_{ORB_ID(vehicle_status)};
    uORB::Subscription parameter_update_sub_{ORB_ID(parameter_update)};

    uORB::Publication<vehicle_attitude_s> attitude_pub_{
        ORB_ID(vehicle_attitude)};
    uORB::Publication<vehicle_local_position_s> local_position_pub_{
        ORB_ID(vehicle_local_position)};
    uORB::Publication<vehicle_global_position_s> global_position_pub_{
        ORB_ID(vehicle_global_position)};
    uORB::Publication<vehicle_odometry_s> odometry_pub_{
        ORB_ID(vehicle_odometry)};
    uORB::Publication<estimator_event_flags_s> event_flags_pub_{
        ORB_ID(estimator_event_flags)};
    uORB::Publication<estimator_gps_status_s> gps_status_pub_{
        ORB_ID(estimator_gps_status)};
    uORB::Publication<estimator_sensor_bias_s> sensor_bias_pub_{
        ORB_ID(estimator_sensor_bias)};
    uORB::Publication<estimator_status_s> status_pub_{
        ORB_ID(estimator_status)};
    uORB::Publication<estimator_status_flags_s> status_flags_pub_{
        ORB_ID(estimator_status_flags)};
    uORB::Publication<yaw_estimator_status_s> yaw_estimator_status_pub_{
        ORB_ID(yaw_estimator_status)};
    uORB::Publication<ekf2_timestamps_s> timestamps_pub_{
        ORB_ID(ekf2_timestamps)};

    uORB::Publication<estimator_aid_source1d_s> aid_gnss_hgt_pub_{
        ORB_ID(estimator_aid_src_gnss_hgt)};
    uORB::Publication<estimator_aid_source1d_s> aid_gnss_yaw_pub_{
        ORB_ID(estimator_aid_src_gnss_yaw)};
    uORB::Publication<estimator_aid_source1d_s> aid_fake_hgt_pub_{
        ORB_ID(estimator_aid_src_fake_hgt)};
    uORB::Publication<estimator_aid_source2d_s> aid_gnss_pos_pub_{
        ORB_ID(estimator_aid_src_gnss_pos)};
    uORB::Publication<estimator_aid_source2d_s> aid_fake_pos_pub_{
        ORB_ID(estimator_aid_src_fake_pos)};
    uORB::Publication<estimator_aid_source3d_s> aid_gnss_vel_pub_{
        ORB_ID(estimator_aid_src_gnss_vel)};
    uORB::Publication<estimator_aid_source3d_s> aid_gravity_pub_{
        ORB_ID(estimator_aid_src_gravity)};
    uORB::Publication<estimator_aid_source3d_s> aid_mag_pub_{
        ORB_ID(estimator_aid_src_mag)};

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    vehicle_status_s vehicle_status_{};
    estimator_event_flags_s event_flags_{};
    AlphaFilter<float> geoid_height_lpf_{};
    InFlightCalibration accel_cal_{};
    InFlightCalibration gyro_cal_{};
    InFlightCalibration mag_cal_{};
    matrix::Vector3f last_accel_bias_published_{};
    matrix::Vector3f last_gyro_bias_published_{};
    matrix::Vector3f last_mag_bias_published_{};

    std::uint64_t integrated_time_us_{0U};
    std::uint64_t start_time_us_{0U};
    std::int64_t last_time_slip_us_{0};
    std::uint64_t last_geoid_height_update_us_{0U};
    std::uint64_t last_sensor_bias_published_{0U};
    std::uint64_t last_event_flags_published_{0U};
    std::uint64_t last_status_flags_published_{0U};
    std::uint64_t last_gps_status_sample_{0U};
    std::uint64_t last_system_flags_sent_us_{0U};
    std::uint64_t mag_declination_retry_after_us_{0U};

    std::uint64_t last_aid_gnss_hgt_sample_{0U};
    std::uint64_t last_aid_gnss_yaw_sample_{0U};
    std::uint64_t last_aid_fake_hgt_sample_{0U};
    std::uint64_t last_aid_gnss_pos_sample_{0U};
    std::uint64_t last_aid_fake_pos_sample_{0U};
    std::uint64_t last_aid_gnss_vel_sample_{0U};
    std::uint64_t last_aid_gravity_sample_{0U};
    std::uint64_t last_aid_mag_sample_{0U};

    std::uint64_t filter_control_status_{0U};
    std::uint32_t filter_fault_status_{0U};
    std::uint32_t innovation_fault_status_{0U};
    std::uint32_t control_status_changes_{0U};
    std::uint32_t fault_status_changes_{0U};
    std::uint32_t innovation_status_changes_{0U};
    std::uint32_t information_event_changes_{0U};

    std::uint32_t accel_device_id_{0U};
    std::uint32_t gyro_device_id_{0U};
    std::uint32_t mag_device_id_{0U};
    std::uint8_t accel_calibration_count_{0U};
    std::uint8_t gyro_calibration_count_{0U};
    std::uint8_t mag_calibration_count_{0U};
    float mag_declination_commit_value_deg_{0.0F};
    std::uint8_t mag_declination_commit_state_{
        static_cast<std::uint8_t>(MagDeclinationCommitState::Idle)};
    bool have_vehicle_status_{false};
    bool system_flags_sent_{false};
    bool mag_declination_saved_{false};
    bool imu_rejection_active_{false};
};

} // namespace dima::modules::ekf2
