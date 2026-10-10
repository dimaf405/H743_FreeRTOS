#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include "control/RoverDifferential.hpp"      // 前端确认需完整类型（事务机路由）
#include "magnetometer/VehicleMagnetometer.hpp"
#include "api/Time.hpp"
#include "logging/logging.hpp"
#include "parameters/param.h"
#include "rover/RoverModeContract.hpp"
#include "rover/RoverControl.hpp"
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
    if (transaction_.active() || transaction_.session_active() || !ScheduleEnable() || !status_pub_.advertise() ||
        !request_pub_.advertise() || !motion_pub_.advertise()) return false;
    status_ = {};
    request_pending_ = 0U;
    status_.mag_interference_pct = -1.0F;
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
    if (status_.active) (void)armed_.set_calibration_output_inhibited(true);
    const auto now = hrt_absolute_time();
    if (status_.active) request_pending_ = auto_calibration_request_s::REQUEST_EXIT;
    // 停机不能跨线程偷偷放弃未完成事务；保持 Error/Arm 互锁，组合根据此拒绝
    // 把部分停机报告为成功。运行期取消走 Run 内完整的回滚/保存状态机。
    module_state_ = (transaction_.active() || transaction_.session_active()) ? dima::middleware::lifecycle::ModuleState::Error
                                        : dima::middleware::lifecycle::ModuleState::Stopped;
    status_.motion_allowed = false;
    pending_termination_ = true;
    (void)publish(now);
}

// fresh()/safety_fresh()/imu_quality()/rtk_quality()/rtk_yaw_fused()/stopped()/
// ground_speed()/yaw_rate()/motion_quality_failure()/
// 公共质量与停止判据位于 AutoCalibrationGates.cpp；
// apply_transaction()/poll_active_transaction()/transaction_settled()/
// transaction_frontend_confirmed()/abort_rollback_complete()/handle_group_abort()/
// on_transaction_provisional()/on_transaction_finished() 迁入
// AutoCalibrationTransactions.cpp；step_finalize()/enter_finalize()/
// session_failure_requires_rollback()/terminate()/finish() 迁入
// AutoCalibrationFinalize.cpp；组调度器与 after_gain_validated() 迁入
// AutoCalibrationGroupScheduler.cpp；publish()/report_status() 迁入
// AutoCalibrationDiagnostics.cpp。跳转点合同见 Dima/rover/modes/README.md。

void AutoCalibrationMode::update_inputs() noexcept
{
    // 每个 Topic 最多 drain 8 个队列元素，优先使用最新测量，固定上界避免低优先级
    // 协调器因高频 IMU 持续到达而永久占用 WorkQueue。
    const auto latest = [](auto &subscription) {
        for (unsigned n = 0U; n < 8U && subscription.update(); ++n) {}
    };
    latest(vehicle_status_sub_); latest(control_sub_); latest(armed_sub_);
    latest(gps_sub_); latest(rtk_sub_); latest(imu_sub_); latest(attitude_sub_);
    latest(raw_mag_sub_); latest(mag_sub_); latest(output_sub_); latest(level_sub_);
    latest(control_feedback_sub_); latest(position_sub_); latest(odometry_sub_);
    latest(bias_sub_); latest(flags_sub_); latest(yaw_aid_sub_); latest(mag_aid_sub_);
    motor_history_.update();
    const auto sample = raw_mag_sub_.get().timestamp_sample;
    if (sample > last_alignment_mag_sample_) {
        last_alignment_mag_sample_ = sample;
        const auto attitude_sample = attitude_sub_.get().timestamp_sample;
        const auto skew = sample > attitude_sample ? sample - attitude_sample : attitude_sample - sample;
        if (skew <= 50000ULL) matched_mag_sample_ = sample;
    }
    // 位置静止滑窗：GNSS速度解在急减速瞬态后振铃~1s（±0.35m/s，实车ULG证实
    // 轮端归零、航向稳定时仍在摆），而GNSS定位不振铃。用≥0.5s窗口内的定位
    // 位移<4cm作为独立静止证据，供停稳门在速度解振铃期间使用。
    const auto &gps = gps_sub_.get();
    if (gps.timestamp_sample > quiet_window_at_ + 500000ULL ||
        (quiet_window_at_ == 0U && gps.timestamp_sample != 0U)) {
        if (quiet_window_at_ != 0U && gps.timestamp_sample > quiet_window_at_) {
            float north{}, east{};
            math::displacement(gps.latitude_deg, gps.longitude_deg,
                quiet_window_lat_, quiet_window_lon_, north, east);
            position_quiet_ = std::hypot(north, east) < 0.04F;
        }
        quiet_window_lat_ = gps.latitude_deg;
        quiet_window_lon_ = gps.longitude_deg;
        quiet_window_at_ = gps.timestamp_sample;
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
    // Level 只依赖板级偏置；运动/磁参数在进入动态阶段时由
    // motion_configuration_valid()/mag_path_ready() 单独判定，不能因为
    // 缺少一项运动配置就阻断独立的静态校准。
    bool level_loaded = true;
    level_loaded &= read(dima::params::SENS_BOARD_X_OFF, next.board_offset[0]);
    level_loaded &= read(dima::params::SENS_BOARD_Y_OFF, next.board_offset[1]);
    level_loaded &= read(dima::params::SENS_BOARD_Z_OFF, next.board_offset[2]);

    bool motion_loaded = true;
    motion_loaded &= read(dima::params::RO_CAL_RADIUS, next.radius);
    motion_loaded &= read(dima::params::RO_CAL_DIST, next.straight_distance);
    motion_loaded &= read(dima::params::RO_CAL_XTE_MAX, next.path_error);
    motion_loaded &= read(dima::params::RO_DECEL_LIM, next.deceleration);
    motion_loaded &= read(dima::params::RO_SPEED_LIM, next.entry_cruise);
    motion_loaded &= read(dima::params::MOT_THR_MAX, next.motor_maximum);
    motion_loaded &= read(dima::params::MOT_REV_DELAY, next.motor_reversal_delay_s);
    motion_loaded &= read(dima::params::MOT_SLEW_RATE, next.motor_slew_rate);
    motion_loaded &= read(dima::params::RD_WHEEL_TRACK, next.track);
    motion_loaded &= read(dima::params::EKF2_GPS_CTRL, next.gps_control);
    motion_loaded &= read(dima::params::CAL_MAG0_ID, next.mag_id);
    motion_loaded &= read(dima::params::CAL_MAG0_ROT, next.mag_rotation);
    motion_loaded &= read(dima::params::CAL_MAG0_XOFF, next.mag_offset[0]);
    motion_loaded &= read(dima::params::CAL_MAG0_YOFF, next.mag_offset[1]);
    motion_loaded &= read(dima::params::CAL_MAG0_ZOFF, next.mag_offset[2]);
    motion_loaded &= read(dima::params::CAL_MAG0_XSCALE, next.mag_scale[0]);
    motion_loaded &= read(dima::params::CAL_MAG0_YSCALE, next.mag_scale[1]);
    motion_loaded &= read(dima::params::CAL_MAG0_ZSCALE, next.mag_scale[2]);
    motion_loaded &= read(dima::params::SENS_MAG_RATE, next.mag_rate);
    if (!level_loaded) return false;
    if (!motion_loaded) PX4_WARN("[autocal] motion configuration incomplete; static Level remains available");
    config_ = next;
    return true;
}

bool AutoCalibrationMode::dynamics_pending() const noexcept
{
    return !bootstrap_applied_ && status_.maximum_speed_m_s > 0.0F &&
        (status_.provisional_validated_stages & Status::STAGE_SPEED) == 0U;
}

bool AutoCalibrationMode::is_motion_state() const noexcept
{
    // 顶层阶段 + 子状态共同决定是否处于运动子状态；事务/停波/等待子状态
    // 一律不发布运动意图。status_.state 单独不再足以判断。
    if (!status_.active) return false;
    switch (status_.state) {
    case Status::STATE_STRAIGHT: case Status::STATE_TURN: case Status::STATE_PROFILE:
    case Status::STATE_IDENTIFICATION: case Status::STATE_VALIDATION: case Status::STATE_NAVIGATION:
        return session_.substate == PhaseSubstate::Running ||
            session_.substate == PhaseSubstate::TurnAround ||
            session_.substate == PhaseSubstate::Return ||
            session_.substate == PhaseSubstate::Braking;
    case Status::STATE_MAGNETIC:
        // 磁阶段本身是决策态；仅磁校准后的回场对准子状态发布运动意图，
        // Evaluate/WaitStop 期间照常互锁。
        return session_.substate == PhaseSubstate::TurnAround ||
            session_.substate == PhaseSubstate::Return;
    default: return false;
    }
}

bool AutoCalibrationMode::new_heading_epoch() noexcept
{
    const auto &rtk = rtk_sub_.get();
    const std::uint64_t epoch = static_cast<std::uint64_t>(rtk.gps_week) * 604800000ULL + rtk.gps_milliseconds;
    if (epoch <= last_epoch_) return false;
    last_epoch_ = epoch;
    return true;
}

void AutoCalibrationMode::enter_phase(std::uint8_t target, std::uint64_t now, PhaseSubstate initial) noexcept
{
    // 阶段只保存在status_.state，常量来自正式消息生成物，不再维护内部镜像。
    // 阶段切换统一走此入口；事务身份与子状态仍由各自字段表达。
    status_.state = target;
    // 阶段默认围栏语义：直线族走廊 / 圆形围栏；子状态与几何 helper 可再细化。
    switch (target) {
    case Status::STATE_BASELINE: case Status::STATE_STRAIGHT:
        status_.straight_active = true; break;
    case Status::STATE_TURN:
        status_.straight_active = false; break;
    default: break;
    }
    PX4_INFO("[autocal] %s", dima::generated::uorb_labels::auto_calibration_status_state_name(target));
    // 已完成准备的阶段直接进入目标子状态，避免空 Entry 再次锁住刚获准的输出。
    // 此处不授予运动许可；Running 必须继承 start_motion() 已获得的许可。
    enter_substate(initial, now);
}

void AutoCalibrationMode::enter_substate(PhaseSubstate substate, std::uint64_t now) noexcept
{
    session_.substate = substate;
    session_.substate_started = now;
    // 静态采集、停车与提交统一先锁 PWM；实际 Armed 不随内部阶段变化。
    if (!is_motion_state()) (void)armed_.set_calibration_output_inhibited(true);
    stable_since_ = 0U;
    steady_since_ = 0U;
    first_circle_at_ = 0U;
    turn_started_ = false;
    stop_turn_rate_request();
    status_.braking_full_output = false;
    drive_envelope_since_ = 0U;
    last_report_ = 0U;
    status_.excitation_phase = Status::EXCITATION_NONE;
    status_.excitation_samples = 0U;
    status_.excitation_target = status_.excitation_remaining_s = 0.0F;
    // WaitStop 保留本轮 SETTLE，供执行层在真实停波后接收模型；恢复运动才撤销。
    if (substate != PhaseSubstate::Braking && substate != PhaseSubstate::WaitStop)
        status_.braking_phase = Status::BRAKING_ACCELERATE;
}

void AutoCalibrationMode::reset_session_dynamics() noexcept
{
    // 每次入场清空运动、动力学和 RTK 拟合，不继承上一轮证据。
    // 杆臂测量同为会话证据：新会话重新测，不沿用旧值。
    // turn_productive_at_ 是停滞窗锚点（绝对时间戳）：同开机连续会话若继承
    // 上一场残留值，新会话 TURN 入口的 75s 窗口已过期，+1 方向被整段跳过
    // （2026-09-30 实车：16:18 后不开机连续重试全部单向旋转）。
    // turn_started_/first_circle_at_ 由换向块自愈；turn_direction/
    // turn_integral_ 由 TURN 授权分支复位——此处只补缺失的锚点。
    measured_lever_m_ = 0.0F;
    measured_lever_valid_ = false;
    reset_rotation_lever_window();
    turn_productive_at_ = 0U;
    longitudinal_ = steering_ = 0.0F;
    // 已确认能行驶的输入供本轮跨阶段返程复用，不能在进入Return时清掉快照。
    straight_start_floor_ = return_open_loop_input_ = 0.0F;
    return_yaw_offset_ = 0.0F;
    return_motion_ = ReturnMotion::Align;
    return_started_ = 0U;
    return_axis_rad_ = NAN;
    stop_turn_rate_request();
    tuning_reference_ = 0U;
    physical_speed_ = physical_rate_ = 0.0F;
    baseline_count_ = 0U;
    last_epoch_ = 0U;
    last_fit_velocity_epoch_ = 0U;
    endpoint_reported_ = false;
    for (unsigned i = 0U; i < 2U; ++i) { heading_mean_[i] = {}; yaw_fit_[i].reset(); }
}

void AutoCalibrationMode::reset_session_magnetic() noexcept
{
    // begin() 复位大块拆分（磁/IMU 偏置面）；语义与原内联序列一致。
    bootstrap_applied_ = mag_ready_ = false;
    imu_bias_attempted_ = imu_bias_finalizing_ = false;
    last_mag_sample_ = last_bias_sample_ = mag_stable_since_ = 0U;
    last_alignment_mag_sample_ = matched_mag_sample_ = 0U;
    mag_device_id_ = 0U;
    mag_mot_fit_[0] = mag_mot_fit_[1] = {};
    motor_history_.reset();
    status_.mag_interference_pct = -1.0F;
    last_mag_mot_sample_ = 0U;
    mag_mot_reported_ = false;
    for (unsigned i = 0U; i < 2U; ++i) { bias_fit_[i].reset(); bootstrap_fit_[i].reset(); coverage_[i] = 0U; }
}

void AutoCalibrationMode::begin(std::uint64_t now) noexcept
{
    (void)armed_.set_calibration_output_inhibited(true);
    status_ = {};
    request_pending_ = 0U;
    session_ = SessionController{};
    for (auto &line : motion_failure_text_) line[0] = '\0';
    if (++session_id_ == 0U) ++session_id_;
    status_.session_id = session_id_;
    status_.active = true;
    status_.result = Status::RESULT_RUNNING;
    helper_failure_reason_ = Status::FAILURE_NONE;
    imu_relock_started_ = 0U;
    const bool session_ready = transaction_.begin_session();
    // 每次进入均从 Preflight/Level 全量重跑；不继承上轮进度，结果只认本轮保存证据。
    // 本模式只采前进/CW/CCW 稳态；短时反向制动不能证明 Manual 共用
    // 电机整形的完整正反域（制动观测的有界反向输出不是倒退校准）。
    session_started_ = now;
    motion_quality_bad_since_ = 0U;
    level_snapshot_valid_ = false;
    pending_termination_ = cancel_requested_ = false;
    tuning_started_ = exercise_running_ = validation_passed_ = false;
    // 回场意图与回场方向都是本会话证据：失败会话可能在回场中途终止，
    // 残留 pending 会把下一会话的 Preflight 误导入 TurnAround，残留的
    // outbound 轴会让新会话跳过基线首次授权方向捕获。
    profile_homing_pending_ = profile_homing_aligning_ = profile_homing_returned_ = false;
    outbound_axis_valid_ = false;
    inner_speed_ok_ = false;
    runtime_cohort_ = false;
    nav_ = {};
    reset_session_dynamics();
    reset_session_magnetic();
    {
        px4::AtomicTransaction atomic;
        config_valid_ = read_config();
        expected_set_count_ = param_set_count();
        // 原值已由会话事务捕获；同一原子读取中的config用于有效性检查，不再存第二份数组。
        level_snapshot_valid_ = transaction_.capture(dima::params::SENS_BOARD_X_OFF) &&
            transaction_.capture(dima::params::SENS_BOARD_Y_OFF) &&
            std::isfinite(config_.board_offset[0]) && std::isfinite(config_.board_offset[1]);
    }
    // 圆心只能在本会话入口抓取一次。后续定位恢复、Level、Disarm 或 RTK
    // 重锁都不能把当前坐标替换成新圆心；入口无定位则本会话仅做静态项。
    capture_fence(now);
    PX4_INFO_RAW("[autocal] session=%lu begin\n", static_cast<unsigned long>(status_.session_id));
    enter_phase(Status::STATE_PREFLIGHT, now, PhaseSubstate::Evaluate);
    if (!session_ready || !config_valid_) terminate(Status::FAILURE_PARAMETER, false, now);
}

void AutoCalibrationMode::step_wait_arm(std::uint64_t now) noexcept
{
    // 所有运动面共用清零、就绪检查和授权；独立速度实验先定向，再进入Running。
    longitudinal_ = steering_ = physical_speed_ = physical_rate_ = 0.0F;
    const bool baseline = status_.state == Status::STATE_BASELINE;
    const bool speed_model = status_.state == Status::STATE_STRAIGHT &&
        (status_.provisional_validated_stages & Status::STAGE_RTK) != 0U;
    const bool turn = status_.state == Status::STATE_TURN;
    const bool profile = status_.state == Status::STATE_PROFILE;
    const bool identifying = status_.state == Status::STATE_IDENTIFICATION;
    const bool validating = status_.state == Status::STATE_VALIDATION || status_.state == Status::STATE_NAVIGATION;
    if (!baseline && !speed_model && !turn && !profile && !identifying && !validating) {
        terminate(Status::FAILURE_PREFLIGHT, false, now);
        return;
    }
    status_.straight_active = baseline || speed_model || (profile && profile_motion_ == 0U) ||
        (identifying && exercise_ < 2U) || (validating && status_.gain_group == Status::GAIN_INNER && exercise_ < 3U);
    const bool quality = baseline ? imu_quality(now) && rtk_quality(now) : rtk_yaw_fused(now);
    if (turn && quality) {
        // 旋转还服务磁采样；无速度模型时只有磁实验可继续，不能凭空启动后续整定。
        const bool no_motion_model = status_.maximum_speed_m_s <= 0.0F && !status_.magnetometer_present;
        if (no_motion_model || (!dynamics_pending() && !mag_path_ready(now))) {
            if (!no_motion_model && bootstrap_applied_) {
                terminate(Status::FAILURE_SENSOR_STALE, false, now);
                return;
            }
            if (no_motion_model) status_.skipped_stages |= Status::STAGE_MAG | Status::STAGE_MAG_MOT;
            else if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = Status::FAILURE_SENSOR_STALE;
            // 两类无工作出口共用速度记账与后续调度；磁bootstrap故障仍走会话终止。
            if (no_motion_model || (status_.provisional_validated_stages & Status::STAGE_SPEED) == 0U)
                status_.unavailable_stages |= Status::STAGE_SPEED;
            start_profile_chain(now);
            return;
        }
    }
    const auto fence = fence_result(now);
    if (speed_model && !braking_model_ready()) {
        terminate(Status::FAILURE_DECELERATION, false, now);
        return;
    }
    if (!quality || !stopped() || !fence.can_stop ||
        ((!baseline && !speed_model && !turn) && !tuning_estimator_valid(now)) ||
        (validating && !gain_frontend_confirmed())) return;
    if (speed_model) {
        // return_axis是冻结的“起点→远端”地理方位；执行层目标使用RTK阵列坐标，
        // 因此加上已确认的安装偏置。不能把返程车头方向直接当作下一段去程。
        turn_heading_ = math::wrap_pi(return_axis_rad_ + rtk_sub_.get().configured_yaw_offset_rad);
        if (!std::isfinite(turn_heading_)) {
            terminate(Status::FAILURE_RTK_INCONSISTENT, false, now);
            return;
        }
    }
    // 同代候选已应用仍可能无法运行；只在将要启动试验时判定，恢复参数不经此门。
    if (validating && !gain_controller_ready()) {
        fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
        return;
    }
    if (profile) {
        // 参考点仅首次有效时冻结；具体空间检查必须在开放运动前完成。
        if (tuning_reference_ == 0U) {
            const auto &position = position_sub_.get();
            tuning_reference_ = position.ref_timestamp;
            tuning_xy_reset_ = position.xy_reset_counter; tuning_vxy_reset_ = position.vxy_reset_counter;
            tuning_yaw_reset_ = position.heading_reset_counter; tuning_odom_reset_ = odometry_sub_.get().reset_counter;
        }
        // 上方已用本运动面的几何检查围栏；仅补充直线长度/旋转杆臂约束，不再重算投影。
        const float lever = profile_motion_ == 0U ? 0.0F : sensor_lever_arm();
        if (profile_motion_ == 0U) leg_distance_ = config_.straight_distance;
        const bool space = profile_motion_ == 0U ? std::isfinite(leg_distance_) && leg_distance_ >= 1.0F
            : std::isfinite(lever) && fence.working_radius_m - fence.distance_m > 0.5F + 2.0F * lever;
        if (!space) {
            fail_tuning(Status::FAILURE_FENCE_SPACE, now);
            return;
        }
    }
    status_.awaiting_arm = true;
    if (!start_motion(now)) return;
    status_.awaiting_arm = false;
    arm_started_ = now;
    leg_heading_ = rtk_sub_.get().array_heading_rad;
    exercise_heading_ = body_yaw();
    if (baseline) {
        session_.straight_outward = true;
        if (!outbound_axis_valid_) {
            // 首次去程授权方向 = 会话初始行驶方向（阵列坐标）；磁后回场
            // 对准以此为目標，PROFILE 前进面沿原方向起跑。
            outbound_axis_rad_ = leg_heading_;
            outbound_axis_valid_ = true;
        }
    }
    else if (speed_model) {
        // 独立起跑不清已确认的制动/RTK，也不重设实验入口的截止时间。
        session_.straight_outward = true;
        leg_distance_ = config_.straight_distance;
    } else if (turn) {
        if (status_.maximum_speed_m_s <= 0.0F) {
            if ((status_.provisional_validated_stages & Status::STAGE_SPEED) == 0U)
                status_.unavailable_stages |= Status::STAGE_SPEED;
            PX4_WARN("[autocal] speed unobservable; magnetic acquisition only");
        }
        turn_integral_ = 0.0F;
        last_turn_heading_ = rtk_sub_.get().array_heading_rad;
        mag_stable_since_ = last_bias_sample_ = last_mag_sample_ = 0U;
        for (unsigned i = 0U; i < 2U; ++i) { bias_fit_[i].reset(); bootstrap_fit_[i].reset(); coverage_[i] = 0U; }
        reset_rotation_lever_window();
        session_.turn_direction = 1;
    } else if (profile) {
        profile_started_ = profile_braking_ = false;
        response_stop_started_ = 0U;
        profile_level_ = 0U;
    } else {
        exercise_started_ = tuning_sample_ = 0U;
        exercise_running_ = false;
        if (validating && !start_validation(now)) {
            fail_tuning(take_helper_failure(Status::FAILURE_GAIN_VALIDATION), now);
            return;
        }
    }
    if (baseline) enter_phase(Status::STATE_STRAIGHT, now, PhaseSubstate::Running);
    else if (speed_model) {
        PX4_INFO_RAW("[autocal] speed model aligning outbound heading %.1fdeg\n",
            static_cast<double>(turn_heading_ / kRadians));
        enter_substate(PhaseSubstate::TurnAround, now);
    } else enter_substate(PhaseSubstate::Running, now);
}

std::uint8_t AutoCalibrationMode::take_helper_failure(std::uint8_t fallback) noexcept
{
    const std::uint8_t reason = helper_failure_reason_;
    helper_failure_reason_ = Status::FAILURE_NONE;
    return reason != Status::FAILURE_NONE ? reason : fallback;
}

bool AutoCalibrationMode::fail_flag(std::uint8_t reason) noexcept
{
    // bool helper 仅用于“准备失败”函数；原因仍由调度器统一消费，不在阶段文件跳转状态。
    helper_failure_reason_ = reason;
    return false;
}

StepResult AutoCalibrationMode::fail_step(std::uint8_t reason) noexcept
{
    // 阶段 helper 只登记原因并返回结果；停车、回滚和后续组调度统一由调度器处理。
    helper_failure_reason_ = reason;
    return StepResult::Failed;
}

StepResult AutoCalibrationMode::abort_step(std::uint8_t reason) noexcept
{
    // 传感器/围栏/控制链等会话级故障走 Abort，由统一安全出口终止，不在 helper 内跳转状态。
    helper_failure_reason_ = reason;
    return StepResult::Abort;
}

bool AutoCalibrationMode::dispatch_step_result(StepResult step, std::uint64_t now) noexcept
{
    // 阶段只提交动作意图；共用机动/停车跳转在此执行一次，不在各阶段复制。
    switch (step) {
    case StepResult::WantTurn: enter_substate(PhaseSubstate::TurnAround, now); return true;
    case StepResult::WantReturn:
        // 先快照原控制模式与已观测油门，再切返程子状态；各实验只提交返程意图。
        return_prepare(now);
        if (status_.state == Status::STATE_STRAIGHT) session_.straight_outward = false;
        enter_substate(PhaseSubstate::Return, now);
        return true;
    case StepResult::WantBrake: enter_substate(PhaseSubstate::Braking, now); return true;
    case StepResult::WantStop: enter_substate(PhaseSubstate::WaitStop, now); return true;
    default: break;
    }
    if (step != StepResult::Failed && step != StepResult::Abort) return false;
    const std::uint8_t reason = take_helper_failure(step == StepResult::Abort
        ? Status::FAILURE_MOTION_ENVELOPE : Status::FAILURE_GAIN_VALIDATION);
    if (step == StepResult::Abort)
        terminate(reason, false, now);
    else
        fail_tuning(reason, now);
    return true;
}

bool AutoCalibrationMode::wait_stop_settled(std::uint64_t now) noexcept
{
    // WaitStop 公共门——“失败→停车→回滚→下一组”样板中各阶段 step_* 近同构
    // 的统一入口：清运动意图 → 15 s 停车预算 → 真实停波 + 后端停波证据 →
    // 组失败回滚处置。返回 true 表示本周期可以通过，调用方继续各自收尾；
    // 差异化语义（恢复直行/建事务/cohort 推进等）保留在各调用点。
    longitudinal_ = steering_ = physical_speed_ = physical_rate_ = 0.0F;
    if (now - session_.substate_started > 15000000ULL) {
        terminate(Status::FAILURE_TIMEOUT, false, now);
        return false;
    }
    // 停车阶段保持 Armed；必须确认车辆已停且 PWM 已物理停波后才推进。
    // 正式制动已按前向测速/轮端归零完成停稳窗，交接继续使用同一判据，
    // 不在这里重新附加陀螺角速度门。静态采集/掉头等仍需原stopped语义。
    const bool stopped_now = status_.braking_phase == Status::BRAKING_SETTLE
        ? braking_stop_confirmed(now) : stopped();
    if (!stopped_now || !maintenance_ready()) return false;
    if (session_.abort_reason != Status::FAILURE_NONE) { handle_group_abort(now); return false; }
    return true;
}

void AutoCalibrationMode::step_magnetic(std::uint64_t now) noexcept
{
    // 回场意图一旦置起，磁决策不再运行：start_profile_chain 可能来自事务
    // 完成回调，本周期仍处于 Evaluate，若不拦截会再次走完磁评估并因
    // tuning_started_ 已置而直接误入 FINALIZE。
    if (session_.substate != PhaseSubstate::Evaluate || profile_homing_pending_) return;
    if (!status_.magnetometer_present) {
        status_.skipped_stages |= Status::STAGE_MAG | Status::STAGE_MAG_MOT;
        start_profile_chain(now);
        return;
    }
    std::uint8_t reason = Status::FAILURE_SENSOR_STALE;
    if (mag_path_ready(now)) {
        mag_ready_ = finish_mag(false);
        // 已收敛候选和首次 bootstrap 共用应用入口；bootstrap 仍只允许一次，
        // 保留原始快照，第二轮采集继续沿用已有人工 Arm 授权。
        if (mag_ready_ || (!bootstrap_applied_ && finish_mag(true))) {
            if (!begin_mag_transaction(now, false)) terminate(Status::FAILURE_PARAMETER, false, now);
            return;
        }
        reason = Status::FAILURE_MAG_DISTURBED;
        PX4_WARN("[autocal] magnetic calibration did not converge");
    } else {
        status_.unavailable_stages |= Status::STAGE_MAG;
        PX4_WARN("[autocal] magnetic path unavailable: %s", config_.mag_id <= 0
            ? "detected device is not bound to CAL_MAG0_ID" : "device/configuration/sample check failed");
    }
    if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = reason;
    // 两类失败共用恢复路径：有 bootstrap 先确认回滚，否则直接进入后续整定。
    if (bootstrap_applied_) {
        if (!begin_mag_transaction(now, true)) terminate(Status::FAILURE_PARAMETER, false, now);
    } else start_profile_chain(now);
}

void AutoCalibrationMode::continue_profile_entry(std::uint64_t now) noexcept
{
    // RUNTIME 入场即整定链入口，依赖由调度器表声明：速度/角速度模型未闭合
    // 或制动模型缺失时不发起注定失败的响应采集，链上组按“有因跳过”记录并收尾。
    if (!group_entry_available(kGroupRowRuntime)) {
        skip_tuning_chain();
        enter_finalize(now, false);
        return;
    }
    const auto result = profile_prepare();
    if (dispatch_step_result(result, now)) return;
    if (result == StepResult::Advance) enter_phase(Status::STATE_PROFILE, now, PhaseSubstate::WaitArm);
    else fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
}

void AutoCalibrationMode::start_profile_chain(std::uint64_t now) noexcept
{
    // 原 start_tuning：整定链入口。Level 之后的全部组都从这条链经过。
    if (pending_termination_ || tuning_started_) { enter_finalize(now, false); return; }
    tuning_started_ = true;
    if ((status_.provisional_validated_stages & Status::STAGE_RTK) == 0U || !read_tuning_config()) {
        fail_tuning(Status::FAILURE_MOTION_UNAVAILABLE, now);
        return;
    }
    // 磁/转向组结束后先回场对准：本函数只在 Evaluate 置起回场意图，不直接
    // 进入运动子状态——静止决策态携带的停波锁必须由 step() 顶部的回场挂钩
    // 经统一输出授权（start_motion）逐段释放后才能启动 Return/TurnAround。
    // 车已在入场点时 Return 即时 Arrived，不产生多余运动；整定链体由
    // resume_profile_chain 执行。
    profile_homing_pending_ = true;
    profile_homing_aligning_ = false;
    profile_homing_returned_ = false;
}

void AutoCalibrationMode::resume_profile_chain(std::uint64_t now) noexcept
{
    // 回场对准完成后的原 start_tuning 后半：IMU bias 观察与整定链入场。
    // 正常 MIN/EXPO/ASYM 可能使旧线性 FF 不可观；RTK 成功后仍能独立采集
    // 真实响应，不能以“FF 已保存”为后置整形/运行参数辨识的循环依赖。
    if (begin_imu_bias(now)) return;
    if (transaction_.active()) { terminate(Status::FAILURE_PARAMETER, false, now); return; }
    status_.unavailable_stages |= Status::STAGE_IMU_BIAS;
    continue_profile_entry(now);
}

void AutoCalibrationMode::step_profile(std::uint64_t now) noexcept
{
    switch (session_.substate) {
    case PhaseSubstate::Running: {
        const StepResult step = profile_run(now);
        (void)dispatch_step_result(step, now);
        break;
    }
    case PhaseSubstate::WaitStop: {
        // 平台序列推进（原 STOP_PROFILE）：下一运动面或候选计算+Runtime 事务。
        ++profile_motion_;
        if (profile_motion_ < 3U) {
            // 不再用置信下界阻断下一运动面；完整数据由最终实际拟合检验。
            enter_substate(PhaseSubstate::WaitArm, now);
        } else {
            profile_motion_deadline_ = 0U;
            if (!calculate_runtime_candidates(now)) {
                fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
                break;
            }
            const bool applied = begin_runtime_transaction(now);
            if (applied || transaction_.active()) {
                // 平台序列结束后只推进本次事务，不再重入 WaitStop 增加序号。
                enter_substate(PhaseSubstate::Evaluate, now);
            } else fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
        }
        break;
    }
    default: break;
    }
}

void AutoCalibrationMode::step_identification(std::uint64_t now) noexcept
{
    switch (session_.substate) {
    case PhaseSubstate::Running: {
        const StepResult step = identification_run(now);
        (void)dispatch_step_result(step, now);
        break;
    }
    case PhaseSubstate::WaitStop: {
        if (exercise_ == 2U) { enter_substate(PhaseSubstate::WaitArm, now); break; }
        // 速度/角速度两面辨识完成：计算公式法 PI 候选并建立 Gains 事务。
        if (!calculate_inner_gains() || !begin_gain_transaction(now, Status::GAIN_INNER)) {
            fail_tuning(take_helper_failure(Status::FAILURE_IDENTIFICATION), now);
            break;
        }
        enter_substate(PhaseSubstate::Evaluate, now);
        break;
    }
    default: break;
    }
}

void AutoCalibrationMode::step_validation(std::uint64_t now) noexcept
{
    switch (session_.substate) {
    case PhaseSubstate::Running: {
        // 内环、航向、转驱与路径共用启动/停车/事务流程，实验算法按组选择。
        StepResult step;
        switch (status_.gain_group) {
        case Status::GAIN_INNER: step = validation_inner(now, exercise_ >= 3U); break;
        case Status::GAIN_HEADING: step = validation_heading(now); break;
        case Status::GAIN_PATH: step = validation_path(now); break;
        default: step = validation_driving(now); break;
        }
        (void)dispatch_step_result(step, now);
        break;
    }
    case PhaseSubstate::WaitStop: {
        if (status_.state == Status::STATE_NAVIGATION) {
            if (status_.gain_group == Status::GAIN_NAVIGATION) {
                if (validation_passed_) advance_cohort_validation(now);
                else fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
            } else {
                const StepResult step = finish_path_validation(now);
                if (dispatch_step_result(step, now)) break;
                if (step == StepResult::Advance)
                    enter_substate(PhaseSubstate::Evaluate, now);
            }
            break;
        }
        // 失败已由统一组失败链停车/回滚；不再按固定百分比猜测新PID。
        // 第 3 段后重新检查运动条件再进入角速度组；第 9 段已完成两轴验证。
        if (status_.gain_group == Status::GAIN_INNER && exercise_ == 3U) {
            inner_speed_ok_ = true;
            enter_substate(PhaseSubstate::WaitArm, now);
        } else if (validation_passed_) advance_cohort_validation(now);
        else fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
        break;
    }
    default: break;
    }
}

void AutoCalibrationMode::advance_cohort_validation(std::uint64_t now) noexcept
{
    // 内环通过只记录 RAM 验证证据，绝不提前开放 autosave。新整形/FF/运行
    // 参数必须与后续航向和真实路径一起验证，失败恢复这一整组最初旧值。
    if (!runtime_cohort_ || !validation_passed_) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    if (status_.gain_group == Status::GAIN_INNER) {
        // 此处已通过角速度验证；速度侧也通过才登记完整内环证据。
        // 后续准入、结算与回滚统一使用此位，不再维护第二份完成标志。
        if (inner_speed_ok_)
            status_.provisional_validated_stages |= Status::STAGE_INNER_GAINS;
        status_.progress = 85U;
        // 外环至少慢于已验证内环，且在转向退出边界仍能产生超过测量死区的
        // 角速度。两项不兼容就拒绝，不能调小死区来伪造可用的 Heading P。
        const float tau = std::max({identification_results_[2].model.time_constant_s, identification_results_[3].model.time_constant_s, 0.1F});
        const float test_angle = 30.0F * kRadians; // Heading采用既有30度阶跃，与用户转驱角独立。
        status_.heading_p = std::min(1.0F / (3.0F * tau), tuning_config_.rate_limit / test_angle);
        if (!std::isfinite(status_.heading_p) || status_.heading_p <= 0.0F || status_.heading_p > 100.0F ||
            !prepare_navigation_candidate() ||
            !begin_gain_transaction(now, Status::GAIN_HEADING)) {
            fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
            return;
        }
        enter_substate(PhaseSubstate::Evaluate, now);
    } else if (status_.gain_group == Status::GAIN_HEADING) {
        if ((status_.provisional_validated_stages & Status::STAGE_INNER_GAINS) == 0U) {
            // Heading 控制器消费完整内环候选；内环不完整时不能把 Heading
            // 或后续 Path 当成独立组继续验证。
            status_.unavailable_stages |= Status::STAGE_HEADING_GAIN | Status::STAGE_PATH_GAIN | Status::STAGE_NAV_STRATEGY;
            enter_finalize(now, false);
            return;
        }
        status_.provisional_validated_stages |= Status::STAGE_HEADING_GAIN;
        status_.progress = 92U;
        if (!read_tuning_config()) { fail_tuning(Status::FAILURE_PARAMETER, now); return; }
        if (!begin_gain_transaction(now, Status::GAIN_NAVIGATION)) {
            fail_tuning(Status::FAILURE_PARAMETER, now);
            return;
        }
        enter_phase(Status::STATE_NAVIGATION, now, PhaseSubstate::Evaluate);
    } else if (status_.gain_group == Status::GAIN_NAVIGATION) {
        if ((status_.provisional_validated_stages & Status::STAGE_HEADING_GAIN) == 0U) {
            status_.unavailable_stages |= Status::STAGE_PATH_GAIN | Status::STAGE_NAV_STRATEGY;
            enter_finalize(now, false);
            return;
        }
        status_.navigation_observed_fields |= Status::NAV_FIELD_TRANSITIONS;
        status_.provisional_validated_stages |= Status::STAGE_NAV_STRATEGY;
        if (!start_path_validation(now)) {
            fail_tuning(take_helper_failure(Status::FAILURE_PATH_UNOBSERVABLE), now);
            return;
        }
        enter_substate(PhaseSubstate::Evaluate, now);
    }
    // GAIN_PATH：由 NAVIGATION.WaitStop 的 finish_path_validation 推进。
}

void AutoCalibrationMode::apply_straight_envelope(std::uint64_t now) noexcept
{
    // 这里只约束真实轮端输出包络；全局速度上限统一使用用户巡航速度。
    // 制动观测单独允许有界反向，正常直行只给正向动力；到顶持续无运动判为失去驱动。
    longitudinal_ = std::clamp(longitudinal_, 0.0F, config_.motor_maximum);
    steering_ = std::clamp(steering_, -1.0F, 1.0F);
    // 复用原8秒窗口：速度实验冻结的工作点若失去前行能力也必须退出，不能
    // 因未到满输出而原地带载等总超时。基础RTK尚无偏置时保留原地速判据。
    const bool speed_model = status_.state == Status::STATE_STRAIGHT &&
        (status_.provisional_validated_stages & Status::STAGE_RTK) != 0U;
    const auto &rtk = rtk_sub_.get();
    const float speed = speed_model ? dima::lib::rover::measure_body_speed(rtk.velocity_north_m_s,
        rtk.velocity_east_m_s, rtk.array_heading_rad - rtk.configured_yaw_offset_rad, 0.0F).speed_m_s : ground_speed();
    const bool moving = speed >= kStoppedSpeedMps;
    const bool held = speed_model && status_.excitation_phase == Status::EXCITATION_COLLECT && longitudinal_ > 0.0F;
    if (!moving && (held || longitudinal_ >= 0.995F * config_.motor_maximum)) {
        if (drive_envelope_since_ == 0U) drive_envelope_since_ = now;
        if (now - drive_envelope_since_ >= 8000000ULL) {
            PX4_WARN("[autocal] no forward response at held input; check MOT_THR_MIN/mechanics/battery");
            terminate(Status::FAILURE_DRIVE_ENVELOPE, false, now);
        }
    } else drive_envelope_since_ = 0U;
}

void AutoCalibrationMode::step_preflight(std::uint64_t now) noexcept
{
    switch (session_.substate) {
    case PhaseSubstate::Evaluate:
        // 静态准备复用Evaluate；Level请求发出后仍进入Running等待该次结果。
        // 入场后允许提前 Arm；Level 只等待物理停波证明，不改变 Armed 状态。
        if (now - session_.substate_started > 30000000ULL) { terminate(Status::FAILURE_PREFLIGHT, false, now); break; }
        // 存储预检（2026-09-30 实车教训）：会话成果只在 FINALIZE 落盘一次，
        // 分区在 SD 拔出期间写满后回收链锁存挂起时，整场结果都会丢失。
        // 无可用存储直接拒绝开跑，不烧一场实验再发现存不进去。
        {
            param_storage_status_s storage{};
            if (param_storage_get_status(&storage) != 0 || !storage.autosave_enabled) {
                PX4_WARN("[autocal] storage unavailable: insert SD card and power-cycle, then retry");
                terminate(Status::FAILURE_STORAGE, false, now);
                break;
            }
            if (storage.free_bytes == 0U && storage.enospc_failures != 0U)
                PX4_WARN("[autocal] parameter partition full: SD card must stay inserted for the recovery save");
        }
        if (!maintenance_ready()) break;
        if (!imu_quality(now) || level_sub_.get().active) break;
        if (!level_snapshot_valid_) { terminate(Status::FAILURE_PARAMETER, false, now); break; }
        if (mag_device_id_ == 0U && fresh(raw_mag_sub_.get().timestamp, now, 1000000ULL))
            mag_device_id_ = raw_mag_sub_.get().device_id;
        status_.magnetometer_present = status_.magnetometer_present || config_.mag_id != 0 ||
            (mag_device_id_ != 0U && fresh(raw_mag_sub_.get().timestamp, now, 1000000ULL));
        // Level 事务：请求/结果都在 PREFLIGHT 阶段处理。
        session_.kind = TransactionKind::Level;
        level_request_time_ = now;
        request_pending_ = auto_calibration_request_s::REQUEST_LEVEL;
        enter_substate(PhaseSubstate::Running, now);
        break;
    case PhaseSubstate::Running: {
        const auto &level = level_sub_.get();
        if (now - session_.substate_started > 45000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); break; }
        if (level.request_timestamp != level_request_time_) break;
        if (!level.active && level.result == sensor_calibration_status_s::RESULT_SUCCESS) {
            // 两项板级校正最多附带一次磁补偿 ID 失效，仍严格核对自有写入计数。
            if (level.parameter_start_count != expected_set_count_ || level.parameter_owned_changes > 3U ||
                level.parameter_set_count != level.parameter_start_count + level.parameter_owned_changes) {
                terminate(Status::FAILURE_PARAMETER, false, now); break;
            }
            expected_set_count_ = level.parameter_set_count;
            session_.kind = TransactionKind::None;
            // Level worker 已完成前端确认；这里仅记录 RAM 候选并继续。持久化由
            // FINALIZE 的统一 session_save() 负责，不单独保存一次。
            px4::AtomicTransaction atomic;
            if (param_set_count() != expected_set_count_ || !read_config()) {
                terminate(Status::FAILURE_PARAMETER, false, now); break;
            }
            status_.level_roll_offset_deg = config_.board_offset[0];
            status_.level_pitch_offset_deg = config_.board_offset[1];
            status_.provisional_validated_stages |= Status::STAGE_LEVEL;
            if (!status_.fence_center_valid || !motion_configuration_valid()) {
                status_.unavailable_stages |= Status::STAGE_DECELERATION | Status::STAGE_RTK |
                    Status::STAGE_SPEED | Status::STAGE_YAW;
                status_.failure_reason = !status_.fence_center_valid ? Status::FAILURE_ENTRY_POSITION
                    : Status::FAILURE_MOTION_UNAVAILABLE;
                enter_finalize(now, false);
            } else enter_phase(Status::STATE_BASELINE, now, PhaseSubstate::Running);
        } else if (!level.active && level.result != sensor_calibration_status_s::RESULT_RUNNING) {
            // Level 失败不是可跳过组：会话级回滚收尾（板级快照由 session 回滚还原）。
            status_.failure_reason = Status::FAILURE_NOT_STATIONARY;
            // 保留 Level 身份，由 FINALIZE 接收 worker 自有恢复计数后再切换事务。
            enter_finalize(now, true);
        }
        break;
    }
    default: break;
    }
}

void AutoCalibrationMode::step_straight(std::uint64_t now) noexcept
{
    switch (session_.substate) {
    case PhaseSubstate::Braking: {
        // 内联制动观测：模式保持零纵向，公共执行层ACTIVE制动（不是倒退校准）。
        const StepResult step = braking_step(now);
        if (step == StepResult::Abort) {
            terminate(take_helper_failure(Status::FAILURE_DECELERATION), false, now);
            break;
        }
        if (step == StepResult::Failed) {
            // 制动观测失败是组级测量结果：标记 DECELERATION 不可用后继续
            // 独立组——原地旋转的 TURN/磁覆盖不需要平移停车模型，不终止会话。
            status_.unavailable_stages |= Status::STAGE_DECELERATION;
            if (status_.failure_reason == Status::FAILURE_NONE)
                status_.failure_reason = Status::FAILURE_DECELERATION;
            PX4_WARN("[autocal] braking group unavailable; continuing turns for magnetic coverage");
            session_.turn_round = 1U;
            enter_phase(Status::STATE_TURN, now, PhaseSubstate::WaitArm);
            break;
        }
        if (step == StepResult::Advance) {
            session_.stop_intent = StopIntent::Braking;
            enter_substate(PhaseSubstate::WaitStop, now);
        }
        break;
    }
    case PhaseSubstate::WaitStop: {
        switch (session_.stop_intent) {
        case StopIntent::Braking:
            // 两轮观测及参数提交后的恢复共用一次模型确认。观测轮数与已验证位
            // 决定是否提交减速度，不再用两个停车意图重复编码同一组进度。
            if (status_.braking_observations != 0U && !drive_.calibration_braking_model_applied(
                    status_.session_id, status_.braking_model_generation, status_.braking_deceleration_m_s2)) break;
            if (status_.braking_observations >= 2U &&
                (status_.provisional_validated_stages & Status::STAGE_DECELERATION) == 0U) {
                if (begin_deceleration_transaction(now)) enter_substate(PhaseSubstate::Evaluate, now);
                else terminate(Status::FAILURE_PARAMETER, false, now);
            } else if (start_motion(now)) {
                session_.stop_intent = StopIntent::None;
                // 正式制动和参数确认不改变运动角色；返程绝不能恢复旧Running直推路径。
                if (!session_.straight_outward) return_motion_ = ReturnMotion::Align;
                enter_substate(session_.straight_outward ? PhaseSubstate::Running : PhaseSubstate::Return, now);
            }
            break;
        case StopIntent::RtkCommit:
            if (begin_rtk_transaction(now)) {
                enter_substate(PhaseSubstate::Evaluate, now);
            } else terminate(Status::FAILURE_PARAMETER, false, now);
            break;
        default:
            if ((status_.provisional_validated_stages & Status::STAGE_RTK) != 0U) {
                // 动态速度候选已形成并返场停稳，才进入后续角速度模型/磁覆盖。
                // 速度不可观测是 SPEED 组的测量结果，不是会话故障：标记不可用
                // 后继续 TURN 的磁覆盖（"仅磁校准"语义）；yaw 辨识依赖速度，
                // 由 turn 阶段自身的 maximum_speed 门自然落入不可用。
                if (!std::isfinite(status_.maximum_speed_m_s) || status_.maximum_speed_m_s <= 0.0F) {
                    status_.unavailable_stages |= Status::STAGE_SPEED;
                    if (status_.failure_reason == Status::FAILURE_NONE)
                        status_.failure_reason = Status::FAILURE_DYNAMICS_UNOBSERVABLE;
                    PX4_WARN("[autocal] speed unobservable; continuing turns for magnetic coverage");
                }
                session_.turn_round = 1U;
                enter_phase(Status::STATE_TURN, now, PhaseSubstate::WaitArm);
            } else enter_substate(PhaseSubstate::Evaluate, now);
            break;
        }
        break;
    }
    default: break;
    }
}

void AutoCalibrationMode::step_turn(std::uint64_t now) noexcept
{
    switch (session_.substate) {
    case PhaseSubstate::Running: {
        // CW/CCW 切换在 helper 内部（session_.turn_direction）；双向完成才 Advance。
        const StepResult step = turn_spin(now);
        if (dispatch_step_result(step, now)) break;
        if (step == StepResult::Advance) enter_substate(PhaseSubstate::WaitStop, now);
        break;
    }
    case PhaseSubstate::WaitStop: {
        enter_substate(PhaseSubstate::Evaluate, now);
        break;
    }
    case PhaseSubstate::Evaluate: {
        if (session_.turn_round == 1U) {
            if (!bootstrap_applied_ && (status_.provisional_validated_stages & Status::STAGE_SPEED) == 0U && finish_dynamics()) {
                if (!begin_dynamics_transaction(now)) terminate(Status::FAILURE_PARAMETER, false, now);
            } else {
                // TURN 的目标角速度不可观测只废 yaw-rate 侧及其增益依赖；
                // MAG 采集使用开环旋转和 RTK/EKF 姿态覆盖，仍由磁决策表继续。
                if ((status_.provisional_validated_stages & Status::STAGE_SPEED) == 0U) {
                    // 动力学面未闭合：SPEED 证据缺席同 YAW 一并记入不可用，
                    // 速度模型可观测时才归因为机械不对称。
                    status_.unavailable_stages |= Status::STAGE_SPEED;
                    if (status_.maximum_speed_m_s > 0.0F)
                        status_.failure_reason = Status::FAILURE_MECHANICAL_ASYMMETRY;
                }
                status_.unavailable_stages |= Status::STAGE_YAW;
                enter_phase(Status::STATE_MAGNETIC, now, PhaseSubstate::Evaluate);
            }
        } else enter_phase(Status::STATE_MAGNETIC, now, PhaseSubstate::Evaluate);
        break;
    }
    default: break;
    }
}

void AutoCalibrationMode::step(std::uint64_t now) noexcept
{
    // 顶层阶段 switch：11 个状态之外一律 PREFLIGHT 失败（状态区已压平，
    // 旧细粒度状态不再存在）。
    status_.awaiting_arm = false;
    // 磁校准/转向结束后的回场挂钩：整定链启动前先回入场点并对准初始
    // 行驶方向，PROFILE 前进面从入场点沿原方向起跑 15m 直线走廊（与
    // 制动/RTK 航段同几何），不再从旋转落点的圆围栏内直接起跑。回场与
    // 对准都是真实运动面：静止决策态携带的停波锁必须先经统一授权出口
    // start_motion() 释放，未获授权时保持 Evaluate 等待，不发无效运动帧
    // （否则 Return/TurnAround 全程 motion_allowed=0，只会耗尽 90s 截止）。
    if (profile_homing_pending_ && session_.substate == PhaseSubstate::Evaluate) {
        if (profile_homing_aligning_) {
            // 对准完成回到 Evaluate 会重锁停波；后端停波确认需要约百毫秒。
            // 静态链（IMU 偏置事务）必须等确认后才发起——begin_imu_bias 的
            // 一次性尝试闸在读到未确认状态时会被立即烧掉，IMU 永久不可用。
            if (!maintenance_ready()) return;
            profile_homing_pending_ = false;
            profile_homing_aligning_ = false;
            resume_profile_chain(now);
            return;
        }
        if (!outbound_axis_valid_) {
            profile_homing_pending_ = false;
            resume_profile_chain(now);
            return;
        }
        status_.awaiting_arm = true;
        if (!start_motion(now)) return;
        status_.awaiting_arm = false;
        if (!profile_homing_returned_) {
            // 先 Return 驶回入场点（车已在入场点时即时 Arrived）；返场完成
            // 回到 Evaluate 后再由本挂钩进入对准，两段都携带已释放的输出锁。
            profile_homing_returned_ = true;
            return_prepare(now);
            session_.resume_substate = PhaseSubstate::Evaluate;
            enter_substate(PhaseSubstate::Return, now);
            return;
        }
        profile_homing_aligning_ = true;
        turn_heading_ = outbound_axis_rad_;
        session_.resume_substate = PhaseSubstate::Evaluate;
        enter_substate(PhaseSubstate::TurnAround, now);
        return;
    }
    const bool speed_model = status_.state == Status::STATE_STRAIGHT &&
        (status_.provisional_validated_stages & Status::STAGE_RTK) != 0U;
    // 入口至成功返场总预算180秒，包含首次就绪等待；没有换场补跑分支。
    if (speed_model && (now < exercise_started_ || now - exercise_started_ > 180000000ULL)) {
        PX4_WARN("[autocal] speed model incomplete: samples=%lu",
            static_cast<unsigned long>(exercise_running_ ? identifier_.sample_count() : 0U));
        // 预算耗尽是 SPEED 组结果：先停车，经 WaitStop 默认意图落入上面的
        // "速度不可观测→继续磁覆盖"路径，不终止会话。
        status_.unavailable_stages |= Status::STAGE_SPEED;
        if (status_.failure_reason == Status::FAILURE_NONE)
            status_.failure_reason = Status::FAILURE_DYNAMICS_UNOBSERVABLE;
        longitudinal_ = steering_ = 0.0F;
        session_.stop_intent = StopIntent::None;
        enter_substate(PhaseSubstate::WaitStop, now);
        return;
    }
    // 各实验共享停车/回滚入口；FINALIZE的停波/保存由step_finalize独占，不走运动检查。
    if (session_.substate == PhaseSubstate::WaitStop && status_.state != Status::STATE_FINALIZE &&
        !wait_stop_settled(now)) return;
    // Evaluate只轮询一笔事务，完成动作由事务路由统一执行；磁bootstrap第二轮
    // 保持Provisional时才落到磁阶段的refine/restore决策，不能重复推进其他阶段。
    if (session_.substate == PhaseSubstate::Evaluate) {
        if (status_.state == Status::STATE_NAVIGATION && session_.kind != TransactionKind::Gains) {
            terminate(Status::FAILURE_PREFLIGHT, false, now);
            return;
        }
        // 磁bootstrap事务会跨第二轮Turn保持Provisional；Turn先结束采集并回到
        // Magnetic再处理该事务，不能把仍持有的事务误当作Turn正在等待的事务。
        const bool transaction_owner = status_.state != Status::STATE_TURN || session_.kind == TransactionKind::Dynamics;
        if (transaction_owner && session_.kind != TransactionKind::None && session_.kind != TransactionKind::Level) {
            poll_active_transaction(now);
            if (status_.state != Status::STATE_MAGNETIC || session_.substate != PhaseSubstate::Evaluate ||
                session_.kind != TransactionKind::Magnetic ||
                transaction_.phase() != CalibrationParameters::Phase::Provisional || !bootstrap_applied_) return;
        }
    }
    if (session_.substate == PhaseSubstate::WaitArm) {
        step_wait_arm(now);
        return;
    }
    // 直行、掉头、返场共用结果分发与直线包络，动作结束后的推进才按角色区分。
    const bool turning = session_.substate == PhaseSubstate::TurnAround;
    const bool straight = status_.state == Status::STATE_STRAIGHT && session_.substate == PhaseSubstate::Running;
    if (straight || turning || session_.substate == PhaseSubstate::Return) {
        const StepResult result = straight ? (speed_model ? speed_model_leg(now) : straight_leg(now))
            : turning ? straight_turnaround(now) : return_step(now);
        if (dispatch_step_result(result, now)) return;
        if (result != StepResult::Busy && result != StepResult::Advance) return;
        if (status_.state == Status::STATE_STRAIGHT) {
            apply_straight_envelope(now);
            if (pending_termination_ || status_.state != Status::STATE_STRAIGHT) return;
        }
        if (result != StepResult::Advance) return;
        if (status_.state != Status::STATE_STRAIGHT) enter_substate(session_.resume_substate, now);
        else if (speed_model) {
            if (turning) {
                // 只有同会话、同机动标识的完成反馈才走到这里；执行层已确认
                // 到位、停稳与轮端归零。直行交接窗从此刻开始，总实验期限不重置。
                arm_started_ = now;
                PX4_INFO_RAW("[autocal] speed model heading aligned; starting identification\n");
                enter_substate(PhaseSubstate::Running, now);
            } else {
                // 模型成功后的Return只走停波确认；不能再次掉头或重启采样。
                session_.stop_intent = StopIntent::None;
                enter_substate(PhaseSubstate::WaitStop, now);
            }
        } else if (straight) enter_substate(PhaseSubstate::TurnAround, now);
        else if (turning) {
            // 保留现有执行层掉头；完成后只进入通用Return，冻结一次返程几何/期限。
            session_.straight_outward = false;
            (void)dispatch_step_result(StepResult::WantReturn, now);
        } else {
            if (!braking_model_ready()) { terminate(Status::FAILURE_DECELERATION, false, now); return; }
            if (!finish_rtk()) { terminate(Status::FAILURE_RTK_INCONSISTENT, false, now); return; }
            session_.stop_intent = StopIntent::RtkCommit;
            enter_substate(PhaseSubstate::WaitStop, now);
        }
        return;
    }
    // 内环/航向/导航验证共用反馈入口。控制链首帧可能正在复位，沿用已有
    // 250ms启动交接窗；返程恢复Running也按本次子状态起点计算，不继承旧计时。
    // 窗口外反馈无效须退出，不能无限等待或凭上一批合格样本完成本次验证。
    if (session_.substate == PhaseSubstate::Running &&
        (status_.state == Status::STATE_VALIDATION || status_.state == Status::STATE_NAVIGATION) &&
        !tuning_feedback(now)) {
        physical_speed_ = physical_rate_ = 0.0F;
        if (now - session_.substate_started > 250000ULL) fail_tuning(Status::FAILURE_SENSOR_STALE, now);
        return;
    }
    switch (status_.state) {
    case Status::STATE_PREFLIGHT: step_preflight(now); break;
    case Status::STATE_BASELINE:
        // 基线采集只有一个运行出口，直接消费结果，删除单case转发状态机。
        if (session_.substate == PhaseSubstate::Running) {
            const StepResult result = baseline_collect(now);
            if (!dispatch_step_result(result, now) && result == StepResult::Advance)
                enter_substate(PhaseSubstate::WaitArm, now);
        }
        break;
    case Status::STATE_STRAIGHT: step_straight(now); break;
    case Status::STATE_TURN: step_turn(now); break;
    case Status::STATE_MAGNETIC: step_magnetic(now); break;
    case Status::STATE_PROFILE: step_profile(now); break;
    case Status::STATE_IDENTIFICATION: step_identification(now); break;
    case Status::STATE_VALIDATION:
    case Status::STATE_NAVIGATION: step_validation(now); break;
    case Status::STATE_FINALIZE: step_finalize(now); break;
    default: terminate(Status::FAILURE_PREFLIGHT, false, now); break;
    }
}

void AutoCalibrationMode::Run()
{
    if (module_state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    update_inputs();
    const auto now = hrt_absolute_time();
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
    if (selected && status_.active && !pending_termination_ && previously_armed_ &&
        !armed_sub_.get().armed) {
        const auto &armed = armed_sub_.get();
        const auto &vehicle = vehicle_status_sub_.get();
        // Commander 已经完成真实 Disarm 时，协调器只记录责任层并进入既有
        // 收尾。Failure Detector/Failsafe/Kill 是全局安全终止；普通人工
        // Disarm 是用户取消，二者都不能在 External1 内继续发布运动请求。
        const bool global_safety = vehicle.failsafe || armed.kill || armed.termination ||
            armed.lockdown || vehicle.latest_disarming_reason ==
            vehicle_status_s::ARM_DISARM_REASON_FAILURE_DETECTOR;
        // FINALIZE 握手豁免：enter_finalize 发出的 REQUEST_EXIT 由 Commander
        // 只解除 Armed、不切模式，本下降沿是完成握手而非取消/控制链事件——
        // 跳过边沿处置，finalize 继续“等 Disarmed → 保存/回滚 → finish()
        // （active=false）→ 终态 EXIT → Commander 切回 Manual → 模式未选中但
        // 已不 active → 不触发取消”的链条，保存决策不被翻成回滚。
        // 真 Kill/Failsafe/Termination/Lockdown 仍按全局安全终止。
        if (status_.state == Status::STATE_FINALIZE && !vehicle.failsafe &&
            !armed.kill && !armed.termination && !armed.lockdown) {
            PX4_INFO("[autocal] finalize disarm handshake accepted");
        } else {
            terminate(global_safety ? Status::FAILURE_CONTROL_LOSS : Status::FAILURE_OPERATOR_CANCEL,
                      !global_safety, now);
        }
    }
    // 授权只镜像Commander的Armed状态，并先于静态worker/事务处理；它不能独立授予动力。
    if (status_.active && !pending_termination_ && selected_ && status_.state != Status::STATE_FINALIZE &&
        armed_.armed() && safety_fresh(now)) status_.session_authorized = true;
    if (!status_.active && !armed_.armed() && !transaction_.active() && !level_sub_.get().active)
        (void)armed_.set_calibration_output_inhibited(false);
    if (status_.active && fresh(raw_mag_sub_.get().timestamp, now, 1000000ULL) && raw_mag_sub_.get().device_id != 0U) {
        // 本会话曾出现的设备只能从 absent 单向变为 present；晚启动设备不能
        // 被误记为跳过。已锁定设备若更换，则终止当前会话并保留已完成阶段。
        status_.magnetometer_present = true;
        if (mag_device_id_ == 0U) mag_device_id_ = raw_mag_sub_.get().device_id;
        else if (mag_device_id_ != raw_mag_sub_.get().device_id) terminate(Status::FAILURE_SENSOR_STALE, false, now);
    }
    if (status_.active && !pending_termination_) {
        // 采样/等待人工 Arm 时冻结整份参数代次；并发参数写入立即终止采样。
        // Level 事务由外部 worker 持有；其他阶段由 CalibrationParameters 精确核对代次。
        if (session_.kind != TransactionKind::Level &&
            (!transaction_.active() || transaction_.phase() == CalibrationParameters::Phase::Provisional) &&
            param_set_count() != expected_set_count_)
            terminate(Status::FAILURE_PARAMETER, false, now);
        else if (last_run_ != 0U && (now < last_run_ || (armed_.armed() && !armed_.calibration_output_inhibited() && now - last_run_ > 100000ULL)))
            terminate(Status::FAILURE_CONTROL_LOSS, false, now);
        else if (!safety_fresh(now))
            terminate(Status::FAILURE_CONTROL_LOSS, false, now);
        else if (armed_.armed() && is_motion_state() && !fence_result(now).can_stop) {
            capture_motion_failure(session_.substate == PhaseSubstate::Return ||
                (status_.state == Status::STATE_STRAIGHT && !session_.straight_outward)
                ? "return boundary" : "fence", now);
            terminate(Status::FAILURE_FENCE_BOUNDARY, false, now);
        }
        else if ((armed_.armed() && (is_motion_state() ||
                    (status_.state == Status::STATE_STRAIGHT && session_.substate == PhaseSubstate::WaitArm &&
                     status_.session_authorized)) &&
                  motion_quality_failure(now)) ||
                 (session_.substate == PhaseSubstate::WaitArm &&
                  (!imu_quality(now) || !fresh(gps_sub_.get().timestamp_sample, now, 300000ULL) ||
                   !fresh(rtk_sub_.get().timestamp_sample, now, 300000ULL) ||
                   (tuning_reference_ != 0U && !tuning_estimator_valid(now))))) {
            // 已授权的速度实验WaitArm与运动阶段共用质量判定和300ms确认窗，GNSS航向
            // 未融合不能因Topic仍新鲜而永久Busy；正常等待首次人工授权语义保留。
            if (motion_quality_bad_since_ == 0U) motion_quality_bad_since_ = now;
            else if (now - motion_quality_bad_since_ >= 300000ULL) {
                capture_motion_failure("quality", now);
                terminate(session_.substate == PhaseSubstate::WaitArm
                    ? (tuning_reference_ != 0U && !tuning_estimator_valid(now)
                        ? Status::FAILURE_ESTIMATOR : Status::FAILURE_SENSOR_STALE)
                    : Status::FAILURE_MOTION_ENVELOPE, false, now);
            }
            else step(now);
        } else {
            motion_quality_bad_since_ = 0U;
            step(now);
        }
    }
    if (status_.active && pending_termination_ && !armed_.armed()) {
        // 所有取消/故障统一在 FINALIZE 处理 worker、候选回滚和存储，避免重复收尾。
        if (status_.state != Status::STATE_FINALIZE)
            enter_finalize(now, cancel_requested_ || session_failure_requires_rollback(status_.failure_reason));
        else step(now);
    }
    last_run_ = now;
    previously_armed_ = armed_sub_.get().armed;
    if (transaction_.phase() == CalibrationParameters::Phase::Fault && status_.result != Status::RESULT_FAILED) {
        status_.result = Status::RESULT_FAILED;
        status_.motion_allowed = status_.awaiting_arm = false;
        PX4_ERR("[autocal] rollback unconfirmed; disarm/reset");
    }
    // 保存可能同步占用低优先级队列；发布使用实际完成时刻，不能重报写入前的旧时间。
    const auto published_at = hrt_absolute_time();
    report_status(published_at);
    if (!publish(published_at) || !ScheduleOnInterval(kIntervalUs)) {
        terminate(Status::FAILURE_CONTROL_LOSS, false, now);
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        (void)publish(hrt_absolute_time()); // 故障停止调度前尽力送出撤销后的状态与EXIT。
    }
}

bool AutoCalibrationMode::maintenance_ready() const noexcept
{
    // 不能用“命令为零”代替后端停波确认；该锁保持到维护/写入完全结束。
    return armed_.calibration_output_stopped();
}

bool AutoCalibrationMode::start_motion(std::uint64_t now) noexcept
{
    // 人工 Arm 与运动许可分离。消费者和 Commander 投影均已收敛后才释放停波锁；
    // 锁释放与 Flash/maintenance 共用原子门，写入期间绝不恢复 PWM。
    const auto &control = control_sub_.get();
    const bool projection = status_.closed_loop
        ? dima::middleware::rover::mode_contract::calibration_closed_loop_projection(control)
        : dima::middleware::rover::mode_contract::calibration_open_loop_projection(control);
    return status_.session_authorized && armed_.armed() && safety_fresh(now) && projection &&
        !vehicle_status_sub_.get().calibration_enabled && !level_sub_.get().active &&
        armed_.set_calibration_output_inhibited(false);
}

} // namespace dima::rover::modes


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::rover::modes {

dima::middleware::lifecycle::ModuleState AutoCalibrationMode::state() const
{ return module_state_; }

} // namespace dima::rover::modes
