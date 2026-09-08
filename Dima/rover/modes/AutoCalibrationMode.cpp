#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include "api/Time.hpp"
#include "logging/logging.hpp"
#include "rover/RoverModeContract.hpp"
#include "sensors/SensorRotation.hpp"
#include <uORB/topics/auto_calibration_status_labels.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

AutoCalibrationMode::AutoCalibrationMode(dima::platform::ArmedFlashCoordinator &armed,
    dima::modules::sensors::VehicleMagnetometer &mag,
    dima::modules::sensors::VehicleImu &imu,
    dima::rover::control::RoverDifferential &drive, AutoMode &navigation) noexcept
    : px4::ScheduledWorkItem("auto_cal", px4::wq_configurations::lp_default),
      armed_(armed), mag_frontend_(mag), imu_frontend_(imu), drive_(drive), navigation_(navigation), transaction_(armed)
{
}

AutoCalibrationMode::~AutoCalibrationMode() { stop(); }

bool AutoCalibrationMode::start()
{
    if (module_state_ == dima::middleware::lifecycle::ModuleState::Running) return true;
    if (transaction_.active() || !ScheduleEnable() || !status_pub_.advertise() ||
        !request_pub_.advertise() || !motion_pub_.advertise()) return false;
    status_ = {};
    selected_ = false;
    previously_armed_ = false;
    last_run_ = 0U;
    module_state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleOnInterval(kIntervalUs, 0U)) {
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    return true;
}

void AutoCalibrationMode::stop()
{
    if (module_state_ == dima::middleware::lifecycle::ModuleState::Stopped) return;
    ScheduleCancelAndDrain();
    const auto now = hrt_absolute_time();
    if (status_.active) request(auto_calibration_request_s::REQUEST_EXIT, now);
    // 停机不能跨线程偷偷放弃未完成事务；保持 Error/Arm 互锁，组合根据此拒绝
    // 把部分停机报告为成功。运行期取消走 Run 内完整的回滚/保存状态机。
    module_state_ = transaction_.active() ? dima::middleware::lifecycle::ModuleState::Error
                                        : dima::middleware::lifecycle::ModuleState::Stopped;
    status_.motion_allowed = false;
    pending_termination_ = true;
    (void)publish(now);
}

bool AutoCalibrationMode::fresh(std::uint64_t timestamp, std::uint64_t now,
                                 std::uint64_t limit) noexcept
{
    return timestamp != 0U && timestamp <= now && now - timestamp <= limit;
}

void AutoCalibrationMode::update_inputs() noexcept
{
    // 每个 Topic 最多 drain 8 个队列元素，优先使用最新测量，固定上界避免低优先级
    // 协调器因高频 IMU 持续到达而永久占用 WorkQueue。
    const auto latest = [](auto &subscription) {
        for (unsigned n = 0U; n < 8U && subscription.update(); ++n) {}
    };
    latest(vehicle_status_sub_); latest(control_sub_); latest(armed_sub_);
    latest(gps_sub_); latest(rtk_sub_); latest(imu_sub_); latest(imu_status_sub_); latest(attitude_sub_);
    latest(raw_mag_sub_); latest(mag_sub_); latest(motors_sub_); latest(level_sub_);
    latest(control_feedback_sub_); latest(position_sub_); latest(odometry_sub_);
    latest(bias_sub_); latest(flags_sub_); latest(yaw_aid_sub_); latest(mag_aid_sub_);
    const auto sample = raw_mag_sub_.get().timestamp_sample;
    if (sample > last_alignment_mag_sample_) {
        last_alignment_mag_sample_ = sample;
        const auto attitude_sample = attitude_sub_.get().timestamp_sample;
        const auto skew = sample > attitude_sample ? sample - attitude_sample : attitude_sample - sample;
        if (skew <= 50000ULL) matched_mag_sample_ = sample;
    }
}

bool AutoCalibrationMode::read_config() noexcept
{
    const auto read = [](dima::params parameter, auto &value) {
        const auto handle = param_handle(parameter);
        param_set_used(handle);
        return param_get(handle, &value) == 0;
    };
    px4::AtomicTransaction transaction;
    Config next{};
    const bool loaded = read(dima::params::RO_CAL_THR_MAX, next.throttle) &&
        read(dima::params::RO_CAL_TURN_MAX, next.steering) &&
        read(dima::params::RO_CAL_RADIUS, next.radius) && read(dima::params::RO_CAL_STOP_D, next.stop_distance) &&
        read(dima::params::RO_SPEED_LIM, next.entry_cruise) && read(dima::params::RO_CAL_VMAX, next.fallback_speed) &&
        read(dima::params::MOT_THR_MAX, next.motor_maximum) &&
        read(dima::params::RD_WHEEL_TRACK, next.track) &&
        read(dima::params::SENS_BOARD_X_OFF, next.board_offset[0]) &&
        read(dima::params::SENS_BOARD_Y_OFF, next.board_offset[1]) &&
        read(dima::params::SENS_BOARD_Z_OFF, next.board_offset[2]) &&
        read(dima::params::EKF2_GPS_CTRL, next.gps_control) &&
        read(dima::params::CAL_MAG0_ID, next.mag_id) && read(dima::params::CAL_MAG0_ROT, next.mag_rotation) &&
        read(dima::params::CAL_MAG0_XOFF, next.mag_offset[0]) &&
        read(dima::params::CAL_MAG0_YOFF, next.mag_offset[1]) &&
        read(dima::params::CAL_MAG0_ZOFF, next.mag_offset[2]) &&
        read(dima::params::CAL_MAG0_XSCALE, next.mag_scale[0]) &&
        read(dima::params::CAL_MAG0_YSCALE, next.mag_scale[1]) &&
        read(dima::params::CAL_MAG0_ZSCALE, next.mag_scale[2]) && read(dima::params::SENS_MAG_RATE, next.mag_rate);
    // 静态 Level 不依赖运动配置。先完整读取并冻结，运动阶段再检查轮距、
    // 激励与停车距离，避免未配置底盘把独立可做的水平校准一并挡住。
    if (!loaded) return false;
    config_ = next;
    return true;
}

bool AutoCalibrationMode::safety_fresh(std::uint64_t now) const noexcept
{
    const auto &status = vehicle_status_sub_.get();
    const auto &control = control_sub_.get();
    const auto &armed = armed_sub_.get();
    return fresh(status.timestamp, now, 750000ULL) && status.timestamp == control.timestamp &&
        status.timestamp == armed.timestamp && !armed.kill && !armed.termination && !armed.lockdown &&
        !status.failsafe && !status.rc_calibration_in_progress &&
        status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER &&
        armed.armed == control.flag_armed && armed.armed == armed_.armed();
}

bool AutoCalibrationMode::imu_quality(std::uint64_t now) const noexcept
{
    const auto &imu = imu_sub_.get();
    const auto &att = attitude_sub_.get();
    double norm_squared = 0.0;
    for (float q : att.q) norm_squared += static_cast<double>(q) * q;
    const float tilt_cosine = 1.0F - 2.0F * (att.q[1] * att.q[1] + att.q[2] * att.q[2]);
    return fresh(imu.timestamp_sample, now, 100000ULL) && fresh(att.timestamp_sample, now, 100000ULL) &&
        std::isfinite(norm_squared) && std::fabs(norm_squared - 1.0) < 0.01 &&
        ((status_.completed_stages & Status::STAGE_LEVEL) == 0U || tilt_cosine > 0.9659F) &&
        imu.accel_device_id != 0U && (imu_device_id_ == 0U || imu.accel_device_id == imu_device_id_) &&
        imu.gyro_device_id != 0U && imu.delta_angle_dt > 0U &&
        imu.delta_angle_clipping == 0U && imu.delta_velocity_clipping == 0U &&
        std::isfinite(yaw_rate()) && std::isfinite(att.q[0]) && std::isfinite(att.q[1]) &&
        std::isfinite(att.q[2]) && std::isfinite(att.q[3]);
}

bool AutoCalibrationMode::rtk_quality(std::uint64_t now) const noexcept
{
    const auto &gps = gps_sub_.get();
    const auto &rtk = rtk_sub_.get();
    return fresh(gps.timestamp_sample, now, 300000ULL) && fresh(rtk.timestamp_sample, now, 300000ULL) &&
        fresh(rtk_progress_time_, now, 300000ULL) &&
        gps.device_id != 0U && gps.device_id == rtk.device_id &&
        (gps_device_id_ == 0U || gps.device_id == gps_device_id_) &&
        gps.fix_type == sensor_gps_s::FIX_TYPE_RTK_FIXED && rtk.solution_computed && rtk.integer_fixed &&
        rtk.gps_week != 0U && rtk.velocity_aligned &&
        std::isfinite(rtk.array_heading_rad) && std::isfinite(rtk.baseline_m) &&
        rtk.baseline_m >= 0.1F && rtk.baseline_m <= 10.0F &&
        std::isfinite(rtk.heading_accuracy_rad) && rtk.heading_accuracy_rad > 0.0F &&
        rtk.heading_accuracy_rad <= kRadians && std::isfinite(ground_speed()) &&
        std::isfinite(rtk.speed_accuracy_m_s) && rtk.speed_accuracy_m_s > 0.0F &&
        rtk.speed_accuracy_m_s <= 0.1F && std::isfinite(gps.eph) && gps.eph <= 0.15F && gps.eph > 0.0F &&
        std::isfinite(gps.latitude_deg) && std::fabs(gps.latitude_deg) < 85.0 &&
        std::isfinite(gps.longitude_deg) && std::fabs(gps.longitude_deg) <= 180.0;
}

bool AutoCalibrationMode::rtk_yaw_fused(std::uint64_t now) const noexcept
{
    const auto &flags = flags_sub_.get();
    const auto &aid = yaw_aid_sub_.get();
    const auto &q = attitude_sub_.get().q;
    const float yaw = std::atan2(2.0F * (q[0] * q[3] + q[1] * q[2]), 1.0F - 2.0F * (q[2] * q[2] + q[3] * q[3]));
    const float expected = math::wrap_pi(rtk_sub_.get().array_heading_rad - rtk_sub_.get().configured_yaw_offset_rad);
    return rtk_quality(now) && rtk_sub_.get().baseline_consistent &&
        imu_quality(now) && flags.cs_yaw_align && flags.cs_tilt_align && !flags.cs_mag_hdg && !flags.cs_mag_3d && std::isfinite(yaw) &&
        std::fabs(math::wrap_pi(yaw - expected)) < 5.0F * kRadians &&
        fresh(flags.timestamp, now, 1500000ULL) && flags.cs_gnss_yaw && !flags.cs_gnss_yaw_fault &&
        fresh(aid.timestamp, now, 1500000ULL) && fresh(aid.time_last_fuse, now, 500000ULL) &&
        aid.fused && !aid.innovation_rejected && std::isfinite(aid.test_ratio) && aid.test_ratio < 1.0F;
}

bool AutoCalibrationMode::dynamics_pending() const noexcept
{
    return !bootstrap_applied_ && status_.maximum_speed_m_s > 0.0F &&
        (status_.completed_stages & Status::STAGE_SPEED) == 0U;
}

bool AutoCalibrationMode::mag_path_ready(std::uint64_t now) const noexcept
{
    const auto &raw = raw_mag_sub_.get();
    const auto &mag = mag_sub_.get();
    if (!status_.magnetometer_present || config_.mag_id < 0 ||
        !std::isfinite(config_.mag_rate) || config_.mag_rate < 10.0F || config_.mag_rate > 200.0F ||
        !dima::lib::sensors::valid_rotation(config_.mag_rotation) || raw.device_id == 0U ||
        raw.device_id != mag_device_id_ || (config_.mag_id > 0 && static_cast<std::uint32_t>(config_.mag_id) != raw.device_id) ||
        !fresh(raw.timestamp_sample, now, 300000ULL) || !fresh(mag.timestamp_sample, now, 300000ULL) ||
        !fresh(matched_mag_sample_, now, 300000ULL) ||
        mag.device_id != raw.device_id || !std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.z) ||
        std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z) > 2.0F) return false;
    for (float scale : config_.mag_scale) if (!std::isfinite(scale) || scale < 0.1F || scale > 3.0F) return false;
    for (float offset : config_.mag_offset) if (!std::isfinite(offset)) return false;
    for (float value : mag.magnetometer_ga) if (!std::isfinite(value)) return false;
    // fresh vehicle_magnetometer 证明前端最近确实消费并发布过数据；只靠 raw
    // device 存在不足以允许一次“仅磁校准”的自动运动。匹配时间按新 raw 样本
    // 单独记录，不能每轮用已经消费的旧 raw 对最新姿态重新判错时。1..200 Hz 的正常运行
    // 合同不变，本校准要求至少 10 Hz，避免低速前端必然无法满足连续融合门禁。
    return true;
}

float AutoCalibrationMode::ground_speed() const noexcept
{
    return std::hypot(rtk_sub_.get().velocity_north_m_s, rtk_sub_.get().velocity_east_m_s);
}

float AutoCalibrationMode::yaw_rate() const noexcept
{
    const auto &imu = imu_sub_.get();
    return imu.delta_angle_dt == 0U ? std::numeric_limits<float>::quiet_NaN()
        : imu.delta_angle[2] / (1.0e-6F * imu.delta_angle_dt);
}

bool AutoCalibrationMode::stopped() const noexcept
{
    return ground_speed() < 0.08F && std::fabs(yaw_rate()) < 0.05F;
}

bool AutoCalibrationMode::is_motion_state() const noexcept
{
    switch (status_.state) {
    case Status::STATE_STRAIGHT_OUT: case Status::STATE_TURN_AROUND: case Status::STATE_STRAIGHT_BACK:
    case Status::STATE_TURN_CW: case Status::STATE_TURN_CCW:
    case Status::STATE_STOP_DISARM_FIRST: case Status::STATE_STOP_DISARM_SECOND: return true;
    case Status::STATE_IDENTIFY_SPEED: case Status::STATE_IDENTIFY_RATE:
    case Status::STATE_STOP_IDENTIFICATION: case Status::STATE_VALIDATE_SPEED:
    case Status::STATE_VALIDATE_RATE: case Status::STATE_VALIDATE_HEADING:
    case Status::STATE_VALIDATE_PATH: case Status::STATE_STOP_VALIDATION: return true;
    case Status::STATE_PROFILE_FORWARD: case Status::STATE_PROFILE_REVERSE:
    case Status::STATE_PROFILE_RATE_CW: case Status::STATE_PROFILE_RATE_CCW:
    case Status::STATE_PROFILE_FULL: case Status::STATE_STOP_PROFILE:
    case Status::STATE_VALIDATE_REVERSE: return true;
    case Status::STATE_VALIDATE_DRIVING: return true;
    default: return false;
    }
}

bool AutoCalibrationMode::motion_envelope(std::uint64_t now) const noexcept
{
    return safety_fresh(now) && imu_quality(now) && rtk_quality(now) &&
        motion_configuration_valid() && fence_result(now).can_stop &&
        (!tuning_started_ || now - arm_started_ <= 250000ULL || tuning_feedback(now)) &&
        linear_acceleration_ok_ && angular_acceleration_ok_ &&
        ((status_.completed_stages & Status::STAGE_RTK) == 0U || rtk_yaw_fused(now)) &&
        ((status_.completed_stages & Status::STAGE_RTK) == 0U || dynamics_pending() || tuning_started_ || mag_path_ready(now)) &&
        std::fabs(rtk_sub_.get().baseline_m - status_.rtk_baseline_m) <= std::fmax(0.02F, 0.05F * status_.rtk_baseline_m) &&
        ground_speed() <= fence_.speed_limit_m_s && std::fabs(yaw_rate()) <= 0.6F &&
        now >= motion_started_ && now - motion_started_ < 180000000ULL;
}

bool AutoCalibrationMode::new_heading_epoch() noexcept
{
    const auto &rtk = rtk_sub_.get();
    const std::uint64_t epoch = static_cast<std::uint64_t>(rtk.gps_week) * 604800000ULL + rtk.gps_milliseconds;
    if (epoch <= last_epoch_) return false;
    last_epoch_ = epoch;
    return true;
}

void AutoCalibrationMode::update_acceleration(std::uint64_t now) noexcept
{
    const auto &rtk = rtk_sub_.get();
    const auto heading_epoch = static_cast<std::uint64_t>(rtk.gps_week) * 604800000ULL + rtk.gps_milliseconds;
    if (heading_epoch > heading_progress_epoch_) { heading_progress_epoch_ = heading_epoch; rtk_progress_time_ = now; }
    const auto epoch = static_cast<std::uint64_t>(rtk.velocity_gps_week) * 604800000ULL + rtk.velocity_gps_milliseconds;
    if (rtk.velocity_aligned && epoch != acceleration_epoch_) {
        // 使用 GNSS 测量历元而非串口到达间隔，a=delta(v)/delta(t)。固定上限
        // 3 m/s^2 独立于电机归一化斜率，重复历元只维持原判定。
        if (acceleration_epoch_ != 0U && epoch > acceleration_epoch_ && epoch - acceleration_epoch_ <= 500U) {
            const float dt = 0.001F * (epoch - acceleration_epoch_);
            const float north = (rtk.velocity_north_m_s - previous_velocity_[0]) / dt;
            const float east = (rtk.velocity_east_m_s - previous_velocity_[1]) / dt;
            const float acceleration = std::hypot(north, east);
            linear_acceleration_ok_ = std::isfinite(acceleration) && acceleration <= 3.0F;
            if (linear_acceleration_ok_) {
                const float alpha = dt / (0.5F + dt);
                filtered_acceleration_[0] += alpha * (north - filtered_acceleration_[0]);
                filtered_acceleration_[1] += alpha * (east - filtered_acceleration_[1]);
            }
        } else linear_acceleration_ok_ = acceleration_epoch_ == 0U;
        acceleration_epoch_ = epoch;
        previous_velocity_[0] = rtk.velocity_north_m_s;
        previous_velocity_[1] = rtk.velocity_east_m_s;
    }
    const auto timestamp = imu_sub_.get().timestamp_sample;
    if (timestamp != angular_acceleration_sample_ && timestamp != 0U) {
        if (angular_acceleration_sample_ != 0U && timestamp > angular_acceleration_sample_ &&
            timestamp - angular_acceleration_sample_ <= 100000ULL) {
            const float dt = 1.0e-6F * (timestamp - angular_acceleration_sample_);
            const float acceleration = (yaw_rate() - previous_yaw_rate_) / dt;
            angular_acceleration_ok_ = std::isfinite(acceleration) && std::fabs(acceleration) <= 3.0F;
            if (angular_acceleration_ok_) filtered_angular_acceleration_ += dt / (0.5F + dt) * (acceleration - filtered_angular_acceleration_);
        } else angular_acceleration_ok_ = angular_acceleration_sample_ == 0U;
        angular_acceleration_sample_ = timestamp;
        previous_yaw_rate_ = yaw_rate();
    }
}

void AutoCalibrationMode::transition(std::uint8_t next, std::uint64_t now) noexcept
{
    status_.state = next;
    state_started_ = now;
    stable_since_ = 0U;
    steady_since_ = 0U;
    first_circle_at_ = 0U;
    turn_started_ = false;
    turn_braking_ = false;
    steady_level_ = 3U;
    last_report_ = 0U;
    PX4_INFO("[autocal] %s", dima::generated::uorb_labels::auto_calibration_status_state_name(next));
}

void AutoCalibrationMode::request(std::uint8_t action, std::uint64_t now) noexcept
{
    auto_calibration_request_s request{};
    request.timestamp = now;
    request.session_id = status_.session_id;
    request.sequence = ++sequence_;
    request.parameter_set_count = expected_set_count_;
    request.request = action;
    // 先发布当前 session/阶段，再发布请求；Commander 处理请求时再取一次状态，
    // 防止新的会话请求撞上上一拍的空闲状态而丢失唯一 Level 启动动作。
    status_.timestamp = now;
    status_.sequence = sequence_;
    (void)status_pub_.publish(status_);
    (void)request_pub_.publish(request);
}

void AutoCalibrationMode::begin(std::uint64_t now) noexcept
{
    status_ = {};
    if (++session_id_ == 0U) ++session_id_;
    status_.session_id = session_id_;
    status_.active = true;
    status_.result = Status::RESULT_RUNNING;
    session_started_ = now;
    last_resume_request_ = 0U;
    longitudinal_ = steering_ = 0.0F;
    pending_termination_ = cancel_requested_ = bootstrap_applied_ = mag_ready_ = false;
    tuning_started_ = exercise_running_ = validation_passed_ = false;
    imu_bias_attempted_ = imu_bias_accel_ = imu_bias_gyro_ = imu_bias_finalizing_ = false;
    runtime_cohort_ = motor_candidate_changed_ = false;
    motor_reprofiled_ = false;
    tuning_reference_ = 0U;
    physical_speed_ = physical_rate_ = 0.0F;
    turn_resume_state_ = Status::STATE_STRAIGHT_BACK;
    baseline_count_ = 0U;
    last_epoch_ = last_mag_sample_ = last_bias_sample_ = mag_stable_since_ = 0U;
    last_fit_velocity_epoch_ = 0U;
    gps_device_id_ = imu_device_id_ = mag_device_id_ = 0U;
    speed_fit_ = {};
    for (unsigned i = 0U; i < 2U; ++i) {
        heading_mean_[i] = {}; yaw_fit_[i].reset(); bias_fit_[i].reset(); bootstrap_fit_[i].reset(); coverage_[i] = 0U;
    }
    {
        px4::AtomicTransaction atomic;
        config_valid_ = read_config();
        expected_set_count_ = param_set_count();
    }
    // 圆心只能在本会话入口抓取一次。后续定位恢复、Level、Disarm 或 RTK
    // 重锁都不能把当前坐标替换成新圆心；入口无定位则本会话仅做静态项。
    capture_fence(now);
    transition(Status::STATE_PREFLIGHT_CHECK, now);
    if (!config_valid_) terminate(Status::FAILURE_PARAMETER, false, now);
}

void AutoCalibrationMode::terminate(std::uint8_t reason, bool cancelled, std::uint64_t now) noexcept
{
    if (!status_.active || pending_termination_) return;
    pending_termination_ = true;
    cancel_requested_ = cancelled;
    status_.failure_reason = reason;
    status_.motion_allowed = false;
    status_.awaiting_arm = false;
    status_.session_authorized = false;
    longitudinal_ = steering_ = 0.0F;
    request(auto_calibration_request_s::REQUEST_EXIT, now);
    PX4_WARN("[autocal] stopping: %s", dima::generated::uorb_labels::auto_calibration_status_failure_name(reason));
}

void AutoCalibrationMode::finish(std::uint64_t now) noexcept
{
    const std::uint32_t required = Status::STAGE_LEVEL | Status::STAGE_RTK | Status::STAGE_SPEED | Status::STAGE_YAW |
        Status::STAGE_INNER_GAINS | Status::STAGE_HEADING_GAIN | Status::STAGE_PATH_GAIN;
    const bool complete = status_.unavailable_stages == 0U && (status_.completed_stages & required) == required &&
        (!status_.magnetometer_present || (status_.completed_stages & Status::STAGE_MAG) != 0U);
    status_.result = cancel_requested_ ? Status::RESULT_CANCELLED : complete && status_.failure_reason == 0U
        ? Status::RESULT_SUCCESS : status_.completed_stages != 0U ? Status::RESULT_PARTIAL : Status::RESULT_FAILED;
    status_.active = status_.motion_allowed = status_.awaiting_arm = false;
    status_.session_authorized = false;
    // 未最终保存的关联证据在回滚/退出后不再代表当前 RAM 配置；终态只展示
    // 已保存阶段，避免“validated RAM / awaiting checks”在会话结束后继续误导。
    status_.provisional_validated_stages = 0U;
    if (tuning_started_ && !transaction_.active() && read_tuning_config()) {
        // 终态显示当前实际参数，回滚后不得继续把候选值显示为已保存值。
        status_.speed_p = tuning_config_.inner[0]; status_.speed_i = tuning_config_.inner[1];
        status_.yaw_rate_p = tuning_config_.inner[2]; status_.yaw_rate_i = tuning_config_.inner[3];
        status_.heading_p = tuning_config_.heading_p;
        status_.lookahead_gain = tuning_config_.pursuit.lookahead_gain;
        px4::AtomicTransaction atomic;
        (void)param_get(param_handle(dima::params::RO_MAX_THR_SPEED), &status_.maximum_speed_m_s);
        (void)param_get(param_handle(dima::params::RO_YAW_RATE_CORR), &status_.yaw_rate_correction);
    }
    status_.progress = complete ? 100U : status_.progress;
    request(auto_calibration_request_s::REQUEST_EXIT, now);
    PX4_INFO("[autocal] %s: %s", dima::generated::uorb_labels::auto_calibration_status_result_name(status_.result),
        dima::generated::uorb_labels::auto_calibration_status_failure_name(status_.failure_reason));
}

void AutoCalibrationMode::Run()
{
    if (module_state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    update_inputs();
    const auto now = hrt_absolute_time();
    update_acceleration(now);
    const bool selected = dima::middleware::rover::mode_contract::auto_calibration(vehicle_status_sub_.get().nav_state);
    if (selected && !selected_ && !status_.active) begin(now);
    if (status_.active) update_fence(now);
    if (!selected && selected_ && status_.active) {
        const auto &armed = armed_sub_.get();
        // latest_disarming_reason 没有自己的时间戳，Disarmed 期间切模式不能
        // 用历史故障原因覆盖本次用户取消；只有新 Disarm 边沿才读取该字段。
        const bool fault = vehicle_status_sub_.get().failsafe || armed.kill || armed.termination || armed.lockdown ||
            (previously_armed_ && !armed.armed && vehicle_status_sub_.get().latest_disarming_reason == vehicle_status_s::ARM_DISARM_REASON_FAILURE_DETECTOR) ||
            (last_run_ != 0U && now >= last_run_ && now - last_run_ > 200000ULL);
        terminate(fault ? Status::FAILURE_CONTROL_LOSS : Status::FAILURE_OPERATOR_CANCEL, !fault, now);
    }
    selected_ = selected;
    if (status_.active && fresh(raw_mag_sub_.get().timestamp, now, 1000000ULL) && raw_mag_sub_.get().device_id != 0U) {
        // 本会话曾出现的设备只能从 absent 单向变为 present；晚启动设备不能
        // 被误记为跳过。已锁定设备若更换，则终止当前会话并保留已完成阶段。
        status_.magnetometer_present = true;
        if (mag_device_id_ == 0U) mag_device_id_ = raw_mag_sub_.get().device_id;
        else if (mag_device_id_ != raw_mag_sub_.get().device_id) terminate(Status::FAILURE_SENSOR_STALE, false, now);
    }
    if (status_.active && !pending_termination_) {
        // 采样/等待人工 Arm 时冻结整份参数代次；并发参数写入立即终止采样。
        // Level 自己拥有事务，其他阶段由 CalibrationParameters 精确核对代次。
        if ((!transaction_.active() || transaction_.phase() == CalibrationParameters::Phase::Provisional) && status_.state != Status::STATE_LEVEL_HOLD &&
            param_set_count() != expected_set_count_) terminate(Status::FAILURE_PARAMETER, false, now);
        else if (last_run_ != 0U && (now < last_run_ || (armed_.armed() && now - last_run_ > 100000ULL)))
            terminate(Status::FAILURE_CONTROL_LOSS, false, now);
        else if (armed_.armed() && is_motion_state() && !motion_envelope(now))
            terminate(!fence_result(now).can_stop ? Status::FAILURE_FENCE_BOUNDARY : Status::FAILURE_MOTION_ENVELOPE, false, now);
        else if (!safety_fresh(now)) terminate(Status::FAILURE_CONTROL_LOSS, false, now);
        else if (now - session_started_ > 600000000ULL) terminate(Status::FAILURE_TIMEOUT, false, now);
        else step(now);
    }
    if (status_.active && pending_termination_ && !armed_.armed()) {
        if (level_sub_.get().active) request(auto_calibration_request_s::REQUEST_CANCEL_LEVEL, now);
        if (transaction_.active()) {
            if (status_.state != Status::STATE_RESTORE_MAG) transaction_.cancel(now);
            transaction_.poll(transaction_frontend_confirmed(now), status_.state == Status::STATE_RESTORE_MAG, now);
            if (transaction_.phase() == CalibrationParameters::Phase::Failed) {
                bootstrap_applied_ = false;
                status_.gains_provisional = false;
            }
        } else if (bootstrap_applied_) {
            if (begin_mag_transaction(now, true)) transition(Status::STATE_RESTORE_MAG, now);
            else status_.failure_reason = Status::FAILURE_PARAMETER;
        } else if (!level_sub_.get().active) finish(now);
        if (status_.state == Status::STATE_RESTORE_MAG && transaction_.phase() == CalibrationParameters::Phase::Done) {
            bootstrap_applied_ = false;
        }
    }
    last_run_ = now;
    previously_armed_ = armed_sub_.get().armed;
    if (transaction_.phase() == CalibrationParameters::Phase::Fault && status_.result != Status::RESULT_FAILED) {
        status_.result = Status::RESULT_FAILED;
        status_.motion_allowed = status_.awaiting_arm = false;
        PX4_ERR("[autocal] rollback unconfirmed; disarm/reset");
    }
    service_motion_authorization(now);
    report_status(now);
    if (!publish(now) || !ScheduleOnInterval(kIntervalUs)) {
        terminate(Status::FAILURE_CONTROL_LOSS, false, now);
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
    }
}

bool AutoCalibrationMode::publish(std::uint64_t now) noexcept
{
    status_.timestamp = now;
    status_.timestamp_sample = now;
    status_.sequence = ++sequence_;
    status_.mag_bootstrap_active = bootstrap_applied_;
    status_.ready_for_entry = !selected_ && !status_.active && !transaction_.active() &&
        !level_sub_.get().active && module_state_ == dima::middleware::lifecycle::ModuleState::Running;
    status_.motion_allowed = status_.active && !pending_termination_ && selected_ && is_motion_state() &&
        armed_.armed() && motion_envelope(now) && module_state_ == dima::middleware::lifecycle::ModuleState::Running;
    rover_motion_request_s motion{};
    motion.timestamp = now;
    motion.timestamp_sample = imu_sub_.get().timestamp_sample;
    motion.sequence = sequence_;
    motion.source = rover_motion_request_s::SOURCE_CALIBRATION;
    motion.mode = status_.closed_loop ? rover_motion_request_s::MODE_SPEED_YAW_RATE : rover_motion_request_s::MODE_NORMALIZED_AXES;
    motion.valid = status_.motion_allowed;
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    motion.speed_m_s = motion.valid && status_.closed_loop ? physical_speed_ : unavailable;
    motion.yaw_rate_rad_s = motion.valid && status_.closed_loop ? physical_rate_ : unavailable;
    motion.normalized_longitudinal = motion.valid && !status_.closed_loop ? longitudinal_ : unavailable;
    motion.normalized_steering = motion.valid && !status_.closed_loop ? steering_ : unavailable;
    return status_pub_.publish(status_) && motion_pub_.publish(motion);
}

} // namespace dima::rover::modes
