/****************************************************************************
 * PX4-Autopilot v1.17.0 VehicleIMU single-device front end adapted for Dima.
 * Upstream: src/modules/sensors/vehicle_imu
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "VehicleImuAlgorithms.hpp"

#include "validation/DataValidator.hpp"
#include "validation/SensorValidityAlgorithms.hpp"
#include "lifecycle/module_base.hpp"
#include "parameter_update.hpp"
#include "parameters/param.h"
#include "api/Flash.hpp"
#include "sensor_accel.hpp"
#include "sensor_gyro.hpp"
#include "uORB/Publication.hpp"
#include "uORB/uORB.hpp"
#include "vehicle_imu.hpp"
#include "vehicle_imu_status.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::sensors {

// 单设备 VehicleIMU 前端：校准/旋转原始 accel+gyro，验证数据流，执行梯形与
// coning 补偿积分，并按 IMU_INTEG_RATE 发布 vehicle_imu。ICM42688P 的 8 kHz
// ODR/FIFO 属于驱动层，本模块不会改硬件采样率。
class VehicleImu final : public dima::middleware::lifecycle::ModuleBase,
                         public px4::ScheduledWorkItem {
public:
    struct Stats {
        std::uint32_t accel_updates{0U};
        std::uint32_t gyro_updates{0U};
        std::uint32_t accel_device_selections{0U};
        std::uint32_t gyro_device_selections{0U};
        std::uint32_t timestamp_gaps{0U};
        std::uint32_t calibration_rejections{0U};
        std::uint32_t parameter_update_failures{0U};
        std::uint32_t configurations_staged{0U};
        std::uint32_t configurations_applied{0U};
        std::uint32_t clipping_warnings{0U};
        std::uint32_t validation_rejections{0U};
        std::uint32_t status_publications{0U};
        std::uint32_t status_publication_failures{0U};
        std::uint32_t publications{0U};
        std::uint32_t publication_failures{0U};
    };

    explicit VehicleImu(
        dima::platform::ArmedFlashCoordinator &armed) noexcept;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;
    const Stats &stats() const noexcept { return stats_; }
    bool calibration_parameter_update_applied(
        std::uint32_t required_instance) const noexcept;
    bool accel_calibration_matches(
        std::int32_t configured_device_id,
        const float (&values)[6]) const noexcept;
    bool gyro_calibration_matches(
        std::int32_t configured_device_id,
        const float (&values)[3]) const noexcept;

private:
    // gyro callback 是主触发，20 ms 备份调度覆盖丢回调/参数更新。每轮最多处理
    // 8 组更新，单样本间隔 >20 ms 重置积分，流超过 200 ms 判不健康。
    static constexpr std::uint32_t kBackupScheduleUs = 20000U;
    static constexpr std::uint32_t kMaximumSampleGapUs = 20000U;
    static constexpr std::uint32_t kStreamTimeoutUs = 200000U;
    static constexpr std::uint64_t kStatusPublishIntervalUs = 1000000ULL;
    static constexpr std::uint64_t kStatusMinimumIntervalUs = 100000ULL;
    static constexpr std::size_t kMaximumUpdatesPerRun = 8U;

    using Vector3 = vehicle_imu_algorithms::Vector3;
    using Configuration = vehicle_imu_algorithms::Configuration;

    enum class ConfigurationReadResult : std::uint8_t {
        Valid = 0U,
        Invalid,
        ReadError,
    };

    struct StatusMoments {
        // Welford 在线矩：m2 最终除以 count-1 得三轴样本方差。
        Vector3 mean{};
        Vector3 m2{};
        std::uint32_t count{0U};
    };

    void Run() override;
    bool bind_parameters() noexcept;
    void invalidate_parameters() noexcept;
    bool refresh_parameter_cache() noexcept;
    ConfigurationReadResult read_configuration(
        bool refresh, Configuration &configuration) noexcept;
    void process_parameter_update(std::uint32_t instance) noexcept;
    void service_pending_configuration() noexcept;
    void apply_configuration(const Configuration &configuration) noexcept;
    void mark_parameter_update_applied(std::uint32_t instance) noexcept;
    void clear_pending_configuration() noexcept;
    Vector3 correct_accel(const sensor_accel_s &sample) const noexcept;
    Vector3 correct_gyro(const sensor_gyro_s &sample) const noexcept;
    bool select_accel_device(std::uint32_t device_id) noexcept;
    bool select_gyro_device(std::uint32_t device_id) noexcept;
    bool process_accel(const sensor_accel_s &sample) noexcept;
    bool process_gyro(const sensor_gyro_s &sample) noexcept;
    bool publish_if_ready() noexcept;
    void accumulate_accel_status(const sensor_accel_s &sample,
                                 const Vector3 &value) noexcept;
    void accumulate_gyro_status(const sensor_gyro_s &sample,
                                const Vector3 &value) noexcept;
    void publish_status(std::uint64_t now_us, bool force = false) noexcept;
    void reset_status_window() noexcept;
    void update_health_state(std::uint64_t now_us) noexcept;
    void reset_integrators(bool reset_last_samples) noexcept;
    void fail_module(const char *reason) noexcept;

    // 参数更新先 stage，只有 disarmed 才由前端应用；SensorCalibration 已持有
    // 全局 interlock 时本模块不能再次获取该锁。
    dima::platform::ArmedFlashCoordinator &armed_;
    // 与 PX4 SensorCalibration::set_device_id() 一致，设备身份来自首次有效 raw
    // topic 样本，而不是由组合根预先注入。单设备产品锁定后拒绝静默切换。
    std::uint32_t accel_device_id_{0U};
    std::uint32_t gyro_device_id_{0U};
    uORB::Subscription accel_sub_{ORB_ID(sensor_accel)};
    uORB::SubscriptionCallbackWorkItem gyro_sub_{ORB_ID(sensor_gyro), *this};
    uORB::SubscriptionCallbackWorkItem parameter_update_sub_{
        ORB_ID(parameter_update), *this};
    uORB::Publication<vehicle_imu_s> vehicle_imu_pub_{ORB_ID(vehicle_imu)};
    uORB::Publication<vehicle_imu_status_s> vehicle_imu_status_pub_{
        ORB_ID(vehicle_imu_status)};

    dima::ParamInt<dima::params::SENS_BOARD_ROT> board_rotation_{};
    dima::ParamInt<dima::params::IMU_INTEG_RATE> integration_rate_{};
    dima::ParamInt<dima::params::SENS_IMU_CLPNOTI>
        clipping_notifications_{};
    dima::ParamInt<dima::params::CAL_ACC0_ID> accel_id_{};
    dima::ParamFloat<dima::params::CAL_ACC0_XOFF> accel_x_offset_{};
    dima::ParamFloat<dima::params::CAL_ACC0_YOFF> accel_y_offset_{};
    dima::ParamFloat<dima::params::CAL_ACC0_ZOFF> accel_z_offset_{};
    dima::ParamFloat<dima::params::CAL_ACC0_XSCALE> accel_x_scale_{};
    dima::ParamFloat<dima::params::CAL_ACC0_YSCALE> accel_y_scale_{};
    dima::ParamFloat<dima::params::CAL_ACC0_ZSCALE> accel_z_scale_{};
    dima::ParamInt<dima::params::CAL_GYRO0_ID> gyro_id_{};
    dima::ParamFloat<dima::params::CAL_GYRO0_XOFF> gyro_x_offset_{};
    dima::ParamFloat<dima::params::CAL_GYRO0_YOFF> gyro_y_offset_{};
    dima::ParamFloat<dima::params::CAL_GYRO0_ZOFF> gyro_z_offset_{};

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    // active/pending 分离保证 invalid 或 armed 期间参数变化不会半应用；应用后
    // 通过原子 instance 发布给 lp_default 校准协调器完成跨 WorkQueue 握手。
    Configuration active_configuration_{};
    Configuration pending_configuration_{};
    std::uint32_t pending_configuration_instance_{0U};
    std::uint32_t applied_parameter_update_instance_{0U};
    std::uint64_t last_status_publish_us_{0U};
    bool configuration_pending_{false};
    bool applied_parameter_update_valid_{false};
    bool validation_fault_active_{false};
    bool clipping_fault_active_{false};
    bool status_dirty_{false};

    // accel/gyro 分别维护结构错误+驱动 error_count 的流健康，任一不健康都停止
    // vehicle_imu 发布并重置积分，但仍继续消费样本以允许恢复。
    dima::lib::sensors::validation::DataValidator accel_validator_{
        kStreamTimeoutUs};
    dima::lib::sensors::validation::DataValidator gyro_validator_{
        kStreamTimeoutUs};
    std::uint32_t accel_validation_error_count_{0U};
    std::uint32_t gyro_validation_error_count_{0U};
    std::uint32_t latest_accel_error_count_{0U};
    std::uint32_t latest_gyro_error_count_{0U};

    StatusMoments accel_status_moments_{};
    StatusMoments gyro_status_moments_{};
    Vector3 previous_status_accel_{};
    Vector3 previous_status_gyro_{};
    bool have_previous_status_accel_{false};
    bool have_previous_status_gyro_{false};
    std::uint64_t accel_status_first_us_{0U};
    std::uint64_t accel_status_last_us_{0U};
    std::uint64_t gyro_status_first_us_{0U};
    std::uint64_t gyro_status_last_us_{0U};
    std::uint32_t accel_status_updates_{0U};
    std::uint32_t gyro_status_updates_{0U};
    std::uint32_t accel_status_raw_samples_{0U};
    std::uint32_t gyro_status_raw_samples_{0U};
    std::uint8_t accel_status_first_samples_{0U};
    std::uint8_t gyro_status_first_samples_{0U};
    float accel_temperature_sum_{0.0F};
    float gyro_temperature_sum_{0.0F};
    std::uint32_t accel_temperature_count_{0U};
    std::uint32_t gyro_temperature_count_{0U};
    std::uint32_t accel_clipping_total_[3]{};
    std::uint32_t gyro_clipping_total_[3]{};
    float accel_vibration_metric_{0.0F};
    float gyro_vibration_metric_{0.0F};
    float coning_metric_accumulator_{0.0F};
    float coning_metric_time_s_{0.0F};

    // accel_integral 单位 m/s（delta velocity），gyro_integral/coning_correction
    // 单位 rad（delta angle），对应 dt 字段单位 us。
    Vector3 accel_integral_{};
    Vector3 gyro_integral_{};
    Vector3 coning_correction_{};
    Vector3 last_accel_{};
    Vector3 last_gyro_{};
    Vector3 last_delta_angle_{};
    Vector3 last_angle_integral_{};
    std::uint64_t last_accel_timestamp_us_{0U};
    std::uint64_t last_gyro_timestamp_us_{0U};
    std::uint32_t accel_integral_dt_us_{0U};
    std::uint32_t gyro_integral_dt_us_{0U};
    std::uint32_t latest_accel_update_us_{0U};
    std::uint32_t latest_gyro_update_us_{0U};
    std::uint8_t accel_clipping_{0U};
    std::uint8_t gyro_clipping_{0U};
    Stats stats_{};
};

} // namespace dima::modules::sensors
