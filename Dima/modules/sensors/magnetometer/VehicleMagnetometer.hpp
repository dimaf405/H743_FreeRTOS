/****************************************************************************
 * PX4-Autopilot v1.17.0 VehicleMagnetometer single-device frontend adapted
 * for Dima.
 * Upstream: src/modules/sensors/vehicle_magnetometer and
 *           src/lib/sensor_calibration/Magnetometer.cpp
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "sensors/SensorRotation.hpp"
#include "lifecycle/module_base.hpp"
#include "parameter_update.hpp"
#include "parameters/param.h"
#include "api/Flash.hpp"
#include "sensor_mag.hpp"
#include "uorb/Publication.hpp"
#include "uorb/uORB.hpp"
#include "vehicle_magnetometer.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::sensors {

// 单设备磁力计前端：选择与 device_id 匹配的保存校准，执行 offset/diagonal
// scale/旋转，按 SENS_MAG_RATE 对原始 gauss 样本做窗口平均并发布。远端 CAN
// 采样率仍由驱动/设备决定，本参数只限制 vehicle_magnetometer 输出频率。
class VehicleMagnetometer final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    struct Stats {
        std::uint32_t raw_updates{0U};
        std::uint32_t invalid_samples{0U};
        std::uint32_t parameter_update_failures{0U};
        std::uint32_t calibration_changes{0U};
        std::uint32_t publications{0U};
        std::uint32_t publication_failures{0U};
    };

    explicit VehicleMagnetometer(
        dima::platform::ArmedFlashCoordinator &armed) noexcept;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;
    const Stats &stats() const noexcept { return stats_; }
    bool calibration_parameter_update_applied(
        std::uint32_t required_instance) const noexcept;
    bool mag_calibration_matches(
        std::int32_t configured_device_id,
        const float (&values)[6]) const noexcept;

private:
    // sensor_mag 回调主触发，50 ms 备份调度处理参数；每轮最多四个样本，避免
    // sensors 队列被磁场突发长期占用。
    static constexpr std::uint32_t kBackupScheduleUs = 50000U;
    static constexpr std::size_t kMaximumUpdatesPerRun = 4U;

    struct Calibration {
        // offset/scale 定义在原始传感器轴，先执行 (raw-offset)*scale，再按
        // CAL_MAG0_ROT 转机体系。saved=false 表示 identity correction。
        std::int32_t configured_device_id{0};
        std::int32_t rotation{0};
        float offset[3]{};
        float scale[3]{1.0F, 1.0F, 1.0F};
        bool saved{false};
    };

    struct Configuration {
        float publication_rate_hz{15.0F};
        Calibration calibration{};
    };

    void Run() override;
    bool bind_parameters() noexcept;
    void invalidate_parameters() noexcept;
    bool refresh_parameter_cache() noexcept;
    bool read_configuration(bool refresh,
                            Configuration &configuration) noexcept;
    void process_parameter_update(std::uint32_t instance) noexcept;
    void service_pending_configuration() noexcept;
    void apply_configuration(const Configuration &configuration) noexcept;
    void mark_parameter_update_applied(std::uint32_t instance) noexcept;
    void clear_pending_configuration() noexcept;
    void configure_device(std::uint32_t device_id) noexcept;
    bool process_sample(const sensor_mag_s &sample) noexcept;
    void reset_accumulator(bool reset_last_publication) noexcept;
    void fail_module(const char *reason) noexcept;

    static bool valid_saved_calibration(
        const Calibration &calibration) noexcept;
    static bool same_configuration(const Configuration &left,
                                   const Configuration &right) noexcept;
    static bool same_correction(const Calibration &left,
                                const Calibration &right) noexcept;
    static Calibration correction_for_device(
        const Configuration &configuration,
        std::uint32_t device_id) noexcept;

    dima::platform::ArmedFlashCoordinator &armed_;
    uORB::SubscriptionCallbackWorkItem sensor_mag_subscription_{
        ORB_ID(sensor_mag), *this};
    uORB::SubscriptionCallbackWorkItem parameter_update_subscription_{
        ORB_ID(parameter_update), *this};
    uORB::Publication<vehicle_magnetometer_s>
        vehicle_magnetometer_publication_{ORB_ID(vehicle_magnetometer)};

    px4::ParamFloat<px4::params::SENS_MAG_RATE> publication_rate_{};
    px4::ParamInt<px4::params::CAL_MAG0_ID> calibration_id_{};
    px4::ParamInt<px4::params::CAL_MAG0_ROT> calibration_rotation_{};
    px4::ParamFloat<px4::params::CAL_MAG0_XOFF> x_offset_{};
    px4::ParamFloat<px4::params::CAL_MAG0_YOFF> y_offset_{};
    px4::ParamFloat<px4::params::CAL_MAG0_ZOFF> z_offset_{};
    px4::ParamFloat<px4::params::CAL_MAG0_XSCALE> x_scale_{};
    px4::ParamFloat<px4::params::CAL_MAG0_YSCALE> y_scale_{};
    px4::ParamFloat<px4::params::CAL_MAG0_ZSCALE> z_scale_{};

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    // 运行期参数更新先 staged，仅在 disarmed 应用并通过原子 instance 与校准
    // 协调器握手；active_correction_ 是针对当前 device_id 实际选中的校正。
    Configuration active_configuration_{};
    Configuration pending_configuration_{};
    Calibration active_correction_{};
    float rotation_matrix_[9]{1.0F, 0.0F, 0.0F,
                              0.0F, 1.0F, 0.0F,
                              0.0F, 0.0F, 1.0F};
    // 输出窗口累加 gauss 与 sample timestamp，发布其算术均值；double 降低
    // 多样本累加误差，最终消息仍按 float ABI 输出。
    double sum_ga_[3]{};
    std::uint64_t timestamp_sample_sum_us_{0U};
    std::uint64_t last_sample_timestamp_us_{0U};
    std::uint64_t last_publication_timestamp_us_{0U};
    std::uint32_t publication_interval_us_{0U};
    std::uint32_t active_device_id_{0U};
    std::uint32_t pending_configuration_instance_{0U};
    std::uint32_t applied_parameter_update_instance_{0U};
    std::uint32_t sample_count_{0U};
    std::uint8_t calibration_count_{0U};
    bool configuration_pending_{false};
    bool applied_parameter_update_valid_{false};
    bool invalid_saved_calibration_reported_{false};
    Stats stats_{};
};

} // namespace dima::modules::sensors
