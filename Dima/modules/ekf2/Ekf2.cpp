#include "Ekf2.hpp"

#include "api/Time.hpp"
#include "logging/logging.hpp"

#include <cmath>

namespace dima::modules::ekf2 {

Ekf2::Ekf2() noexcept
    : px4::ScheduledWorkItem("ekf2", px4::wq_configurations::estimator),
      mag_declination_commit_worker_(*this),
      vehicle_imu_sub_(ORB_ID(vehicle_imu), *this)
{
}

Ekf2::MagDeclinationCommitWorkItem::MagDeclinationCommitWorkItem(
    Ekf2 &owner) noexcept
    : px4::ScheduledWorkItem("ekf2_param_commit",
                             px4::wq_configurations::lp_default),
      owner_(owner)
{
}

void Ekf2::MagDeclinationCommitWorkItem::Run()
{
    owner_.run_mag_declination_commit();
}

bool Ekf2::advertise_outputs() noexcept
{
    // 单实例发布端在回调注册前一次性建好。uORB 的 Topic ID、布局和 alias
    // 全部来自 .msg 正式生成物；这里仅声明本模块真实拥有的发布端。
    return attitude_pub_.advertise() && local_position_pub_.advertise() &&
           global_position_pub_.advertise() && odometry_pub_.advertise() &&
           event_flags_pub_.advertise() && gps_status_pub_.advertise() &&
           sensor_bias_pub_.advertise() && status_pub_.advertise() &&
           status_flags_pub_.advertise() &&
           yaw_estimator_status_pub_.advertise() &&
           timestamps_pub_.advertise() && aid_gnss_hgt_pub_.advertise() &&
           aid_gnss_yaw_pub_.advertise() && aid_fake_hgt_pub_.advertise() &&
           aid_gnss_pos_pub_.advertise() && aid_fake_pos_pub_.advertise() &&
           aid_gnss_vel_pub_.advertise() && aid_gravity_pub_.advertise() &&
           aid_mag_pub_.advertise();
}

bool Ekf2::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }

    reset_mag_declination_commit();
    // 参数和全部 RingBuffer 都在 appMain 的非实时启动上下文完成；只有闭包
    // 完整后才开放 estimator WorkQueue 并注册 IMU callback，热路径因此不会
    // 触发 new/delete。启动失败只把本模块置 Error，组合根仍保留 Manual。
    if (!mag_declination_commit_worker_.ScheduleEnable() ||
        !load_parameters(true) ||
        !ekf_.initialiseBuffers(hrt_absolute_time()) ||
        !advertise_outputs() || !ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        mag_declination_commit_worker_.ScheduleCancelAndDrain();
        ekf_.releaseBuffers();
        PX4_ERR("EKF2 startup closure failed; Manual remains available");
        return false;
    }

    integrated_time_us_ = 0U;
    start_time_us_ = 0U;
    last_time_slip_us_ = 0;
    last_geoid_height_update_us_ = 0U;
    last_sensor_bias_published_ = 0U;
    last_event_flags_published_ = 0U;
    last_status_flags_published_ = 0U;
    last_gps_status_sample_ = 0U;
    last_system_flags_sent_us_ = 0U;
    mag_declination_retry_after_us_ = 0U;
    accel_cal_ = {};
    gyro_cal_ = {};
    mag_cal_ = {};
    event_flags_ = {};
    have_vehicle_status_ = false;
    system_flags_sent_ = false;
    mag_declination_saved_ = false;
    imu_rejection_active_ = false;
    filter_control_status_ = 0U;
    filter_fault_status_ = 0U;
    innovation_fault_status_ = 0U;
    control_status_changes_ = 0U;
    fault_status_changes_ = 0U;
    innovation_status_changes_ = 0U;
    information_event_changes_ = 0U;

    if (!vehicle_imu_sub_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        mag_declination_commit_worker_.ScheduleCancelAndDrain();
        ekf_.releaseBuffers();
        PX4_ERR("vehicle_imu callback registration failed");
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleNow()) {
        // callback 注册成功但首次调度失败时，不能把一个永远不运行的实例报告为
        // Running；完整撤销 producer 回调和两个 WorkItem，再由组合根按降级
        // 合同保留 Manual，而不是留下假健康 EKF2。
        vehicle_imu_sub_.unregisterCallback();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        mag_declination_commit_worker_.ScheduleCancelAndDrain();
        ekf_.releaseBuffers();
        PX4_ERR("EKF2 initial scheduling failed; Manual remains available");
        return false;
    }
    return true;
}

void Ekf2::stop()
{
    // 产品运行期没有 EKF2 装卸入口；该 stop 只服务完整 Application shutdown，
    // 先撤 callback 再 drain WorkQueue，绝不影响 Commander、Manual 或 PWM。
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    vehicle_imu_sub_.unregisterCallback();
    ScheduleCancelAndDrain();
    // estimator producer 排空后才允许 lp_default consumer 退出，固定请求快照
    // 在完整 shutdown 期间始终有效。
    mag_declination_commit_worker_.ScheduleCancelAndDrain();
    reset_mag_declination_commit();
    ekf_.releaseBuffers();
}

dima::middleware::lifecycle::ModuleState Ekf2::state() const
{
    return state_;
}

bool Ekf2::schedule_backup() noexcept
{
    if (ScheduleDelayed(kBackupScheduleUs)) {
        return true;
    }

    // callback 只能补偿“有下一帧 IMU”的情况；若 100 ms 备份调度本身失败却仍
    // 报告 Running，无输入时模块会永久静默。正常 shutdown 已先置 Stopped，
    // 因此这里只把真实的运行期调度故障收敛为 Error，并撤掉 callback 防止后续
    // 高频发布反复唤醒一个已经不能维持调度合同的实例。Manual 安全链保持独立。
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        vehicle_imu_sub_.unregisterCallback();
        ScheduleCancelAndDrain();
        PX4_ERR("EKF2 backup scheduling failed; Manual remains available");
    }
    return false;
}

bool Ekf2::validate_imu(const vehicle_imu_s &imu) const noexcept
{
    if (imu.timestamp == 0U || imu.timestamp_sample == 0U ||
        imu.timestamp_sample > imu.timestamp || imu.accel_device_id == 0U ||
        imu.gyro_device_id == 0U || imu.delta_angle_dt == 0U ||
        imu.delta_velocity_dt == 0U) {
        return false;
    }
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        if (!std::isfinite(imu.delta_angle[axis]) ||
            !std::isfinite(imu.delta_velocity[axis])) {
            return false;
        }
    }
    return true;
}

void Ekf2::update_imu_identity(const vehicle_imu_s &imu) noexcept
{
    if (accel_device_id_ == 0U || gyro_device_id_ == 0U) {
        accel_device_id_ = imu.accel_device_id;
        gyro_device_id_ = imu.gyro_device_id;
        accel_calibration_count_ = imu.accel_calibration_count;
        gyro_calibration_count_ = imu.gyro_calibration_count;
        return;
    }

    // 设备或前端静态校准变化后，旧的 in-run bias 已不再对应同一测量模型；
    // 与 PX4 VehicleIMU 路径一致，分别清零相关 bias 与稳定计时。
    if (imu.accel_device_id != accel_device_id_ ||
        imu.accel_calibration_count != accel_calibration_count_) {
        accel_device_id_ = imu.accel_device_id;
        accel_calibration_count_ = imu.accel_calibration_count;
        ekf_.resetAccelBias();
        accel_cal_ = {};
    }
    if (imu.gyro_device_id != gyro_device_id_ ||
        imu.gyro_calibration_count != gyro_calibration_count_) {
        gyro_device_id_ = imu.gyro_device_id;
        gyro_calibration_count_ = imu.gyro_calibration_count;
        ekf_.resetGyroBias();
        gyro_cal_ = {};
    }
}

void Ekf2::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s parameter_update{};
    if (parameter_update_sub_.copy(&parameter_update) &&
        !load_parameters(false)) {
        // 参数刷新采用候选副本，失败时旧参数仍完整生效；不允许半更新 EKF。
        PX4_WARN("EKF2 parameter refresh rejected; keeping previous values");
    }

    vehicle_status_s status{};
    if (vehicle_status_sub_.copy(&status)) {
        vehicle_status_ = status;
        have_vehicle_status_ = true;
        system_flags_sent_ = false;
    }

    vehicle_imu_s imu{};
    if (!vehicle_imu_sub_.copy(&imu)) {
        (void)schedule_backup();
        return;
    }
    if (!validate_imu(imu)) {
        if (!imu_rejection_active_) {
            PX4_WARN("EKF2 rejected invalid vehicle_imu sample");
        }
        imu_rejection_active_ = true;
        (void)schedule_backup();
        return;
    }
    imu_rejection_active_ = false;
    update_imu_identity(imu);

    imuSample imu_sample{};
    imu_sample.time_us = imu.timestamp_sample;
    imu_sample.delta_ang_dt =
        static_cast<float>(imu.delta_angle_dt) * 1.0e-6F;
    imu_sample.delta_ang = matrix::Vector3f{imu.delta_angle};
    imu_sample.delta_vel_dt =
        static_cast<float>(imu.delta_velocity_dt) * 1.0e-6F;
    imu_sample.delta_vel = matrix::Vector3f{imu.delta_velocity};
    imu_sample.delta_vel_clipping[0] =
        (imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X) != 0U;
    imu_sample.delta_vel_clipping[1] =
        (imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y) != 0U;
    imu_sample.delta_vel_clipping[2] =
        (imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z) != 0U;

    ekf_.setIMUData(imu_sample);
    publish_attitude(imu_sample.time_us);

    ekf2_timestamps_s timestamps{};
    timestamps.timestamp = imu_sample.time_us;
    timestamps.airspeed_timestamp_rel =
        ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
    timestamps.airspeed_validated_timestamp_rel =
        ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
    timestamps.distance_sensor_timestamp_rel =
        ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
    timestamps.optical_flow_timestamp_rel =
        ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
    timestamps.vehicle_air_data_timestamp_rel =
        ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
    timestamps.vehicle_magnetometer_timestamp_rel =
        ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
    timestamps.visual_odometry_timestamp_rel =
        ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;

    (void)update_gps();
    (void)update_magnetometer(timestamps);
    update_system_flags(imu_sample.time_us);

    if (start_time_us_ == 0U || imu_sample.time_us < start_time_us_) {
        start_time_us_ = imu_sample.time_us;
        integrated_time_us_ = 0U;
        last_time_slip_us_ = 0;
    } else {
        integrated_time_us_ += imu.delta_angle_dt;
        last_time_slip_us_ =
            static_cast<std::int64_t>(imu_sample.time_us - start_time_us_) -
            static_cast<std::int64_t>(integrated_time_us_);
    }

    if (ekf_.update()) {
        const std::uint64_t now_us = hrt_absolute_time();
        publish_local_position(imu_sample.time_us);
        publish_odometry(imu_sample.time_us, imu_sample);
        publish_global_position(imu_sample.time_us);
        publish_event_flags(now_us);
        publish_status(now_us);
        publish_status_flags(now_us);
        publish_aid_sources(now_us);
        publish_gps_status(now_us);
        publish_yaw_estimator_status(now_us);
        publish_sensor_bias(now_us);
        // 稳定标志按 PX4 规则在发布后更新，下一帧才可见；这样消费者只会拿到
        // 已连续满足完整 10 s 的 bias，而不是当前帧刚越过阈值的候选。
        update_bias_stability(imu_sample.time_us);
        update_mag_declination(now_us);
    }

    (void)timestamps_pub_.publish(timestamps);
    (void)schedule_backup();
}

} // namespace dima::modules::ekf2
