#define MODULE_NAME "rover_auto_mode"

#include "AutoMode.hpp"

#include "api/Time.hpp"
#include "events/events.hpp"

#include <cerrno>
#include <cfloat>
#include <cmath>
#include <limits>

namespace dima::rover::modes {
namespace {

constexpr std::uint32_t kEventParameterBindingFailure = 0x52415501U;
constexpr std::uint32_t kEventParameterInvalid = 0x52415502U;
constexpr std::uint32_t kEventPublishFailure = 0x52415503U;
constexpr std::uint32_t kEventScheduleFailure = 0x52415504U;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegreesToRadians = kPi / 180.0F;
constexpr float kUnavailable = std::numeric_limits<float>::quiet_NaN();

float angular_parameter_to_radians(float value) noexcept
{
    // PX4 的 RO_YAW_* 参数以 deg、deg/s 或 deg/s^2 暴露；控制核统一使用
    // rad、rad/s、rad/s^2。负值是“-1 禁用”哨兵，不能乘角度换算系数。
    return value < 0.0F ? value : value * kDegreesToRadians;
}

float vector_bearing(const dima::lib::rover::Position2f &from,
                     const dima::lib::rover::Position2f &to) noexcept
{
    return std::atan2(to.east_m - from.east_m,
                      to.north_m - from.north_m);
}

float vector_length(const dima::lib::rover::Position2f &from,
                    const dima::lib::rover::Position2f &to) noexcept
{
    return std::hypot(to.north_m - from.north_m,
                      to.east_m - from.east_m);
}

} // namespace

AutoMode::AutoMode(
    dima::modules::mission::MissionService &mission_service) noexcept
    : px4::ScheduledWorkItem("rover_auto_mode", px4::wq_configurations::nav),
      mission_service_(mission_service)
{
}

AutoMode::~AutoMode()
{
    stop();
}

bool AutoMode::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    reset_runtime_state();
    if (!bind_parameters()) {
        enter_error(kEventParameterBindingFailure);
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    // AUTO 外环固定 50 Hz。首帧立即运行，用无效请求和 readiness 状态覆盖
    // 重启前的队列内容，随后由间隔调度维持确定的控制周期。
    if (!ScheduleOnInterval(kRunIntervalUs, 0U)) {
        enter_error(kEventScheduleFailure);
        return false;
    }
    return true;
}

void AutoMode::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();

    dima::modules::mission::MissionStatus mission{};
    (void)mission_service_.status(mission);
    (void)publish_invalid(hrt_absolute_time(), mission,
                          rover_navigation_status_s::FAILURE_NOT_AUTO,
                          false, false);
    invalidate_parameter_bindings();
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState AutoMode::state() const
{
    return state_;
}

void AutoMode::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    update_subscriptions();
    const std::uint64_t now = hrt_absolute_time();
    const std::uint64_t previous_run = last_run_us_;
    const bool clock_regressed = previous_run != 0U && now < previous_run;
    const bool control_gap = previous_run != 0U && now >= previous_run &&
                             now - previous_run > kMaximumControlDtUs;
    last_run_us_ = now;

    apply_pending_parameters(now);

    dima::modules::mission::MissionStatus mission{};
    const bool mission_status_available =
        refresh_mission_snapshot(now, mission);
    const bool mission_plan_available =
        mission_status_available && refresh_mission_plan(mission);
    const bool safety_fresh = safety_snapshot_fresh(now);
    const bool estimator_is_healthy = estimator_healthy(now);
    // Armed 期间参数更新只标记为 pending，当前已验证快照继续服务正在执行的
    // Mission，避免半套重配造成控制跳变；但 pending 代尚未经过 Disarmed 原子
    // 应用与完整校验，因此不得把它当作“可启动新任务”。这只锁闭 Mission Start
    // readiness，不会令已有 AUTO 请求失效或把车辆误切入 Hold。
    bool ready_for_auto = mission_status_available &&
        mission_plan_available && mission.loaded &&
        mission.committed && mission.count > 0U &&
        !mission.mutation_in_progress && parameters_valid_ &&
        !parameter_update_pending_ &&
        safety_fresh && estimator_is_healthy;

    const bool estimator_time_regressed =
        (have_local_position_ &&
         ((last_estimator_timestamp_us_ != 0U &&
           local_position_.timestamp < last_estimator_timestamp_us_) ||
          (last_estimator_sample_us_ != 0U &&
           local_position_.timestamp_sample < last_estimator_sample_us_))) ||
        (have_odometry_ &&
         ((last_odometry_timestamp_us_ != 0U &&
           vehicle_odometry_.timestamp < last_odometry_timestamp_us_) ||
          (last_odometry_sample_us_ != 0U &&
           vehicle_odometry_.timestamp_sample <
               last_odometry_sample_us_)));
    if (have_local_position_) {
        last_estimator_timestamp_us_ = local_position_.timestamp;
        last_estimator_sample_us_ = local_position_.timestamp_sample;
    }
    if (have_odometry_) {
        last_odometry_timestamp_us_ = vehicle_odometry_.timestamp;
        last_odometry_sample_us_ = vehicle_odometry_.timestamp_sample;
    }

    if (clock_regressed || estimator_time_regressed) {
        // 单调时间或估计样本回退会破坏 slew/integral 的 dt 假设；本周期先发
        // 无效帧并清空全部控制状态，下一份正常样本重新建立基线。
        ready_for_auto = false;
        reset_control_state();
        have_estimator_baseline_ = false;
        if (!publish_invalid(
                now, mission,
                rover_navigation_status_s::FAILURE_TIME_REGRESSION,
                ready_for_auto, estimator_is_healthy)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (!parameters_valid_) {
        reset_control_state();
        if (!publish_invalid(
                now, mission,
                rover_navigation_status_s::FAILURE_PARAMETER_INVALID,
                false, estimator_is_healthy)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (!mission_status_available || !mission_plan_available ||
        !mission.loaded ||
        !mission.committed || mission.count == 0U ||
        mission.current >= mission.count) {
        reset_control_state();
        if (!publish_invalid(
                now, mission,
                rover_navigation_status_s::FAILURE_MISSION_UNAVAILABLE,
                false, estimator_is_healthy)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (!estimator_is_healthy) {
        // 无效姿态/位置/速度、dead-reckoning 或失鲜都禁止沿用上一控制量。
        // 时间戳格式正确但年龄超限时单独报告 STALE，便于 Commander/QGC
        // 区分“数据停止更新”和“估计器明确拒绝用于控制”。
        const bool timestamp_shape_valid = have_local_position_ &&
            have_odometry_ &&
            local_position_.timestamp != 0U &&
            local_position_.timestamp_sample != 0U &&
            local_position_.timestamp_sample <= local_position_.timestamp &&
            local_position_.timestamp <= now;
        const bool odometry_timestamp_shape_valid = timestamp_shape_valid &&
            vehicle_odometry_.timestamp != 0U &&
            vehicle_odometry_.timestamp_sample != 0U &&
            vehicle_odometry_.timestamp_sample <=
                vehicle_odometry_.timestamp &&
            vehicle_odometry_.timestamp <= now;
        const bool stale = odometry_timestamp_shape_valid &&
            (now - local_position_.timestamp > kEstimatorTimeoutUs ||
             now - local_position_.timestamp_sample > kEstimatorTimeoutUs ||
             now - vehicle_odometry_.timestamp > kEstimatorTimeoutUs ||
             now - vehicle_odometry_.timestamp_sample >
                 kEstimatorTimeoutUs);
        reset_control_state();
        have_estimator_baseline_ = false;
        if (!publish_invalid(
                now, mission,
                stale ? rover_navigation_status_s::FAILURE_ESTIMATOR_STALE
                      : rover_navigation_status_s::FAILURE_ESTIMATOR_UNHEALTHY,
                false, false)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (control_gap || estimator_reset_detected()) {
        // reset counter、NED 原点或控制周期间隔变化后严格空一帧，清空
        // Heading slew/转向状态，并要求下一周期以当前车位重建航段起点。
        ready_for_auto = false;
        reset_control_state();
        if (!publish_invalid(
                now, mission,
                control_gap
                    ? rover_navigation_status_s::FAILURE_ESTIMATOR_STALE
                    : rover_navigation_status_s::FAILURE_ESTIMATOR_RESET,
                ready_for_auto, true)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (!safety_fresh ||
        (!auto_projection(vehicle_status_.nav_state, true))) {
        // Mission execution 状态只由 Commander 改变。导航线程若在 Commander
        // 发布新 nav_state 前自行 suspend，会把刚接受的 Mission Start 竞态撤销。
        auto_was_active_ = false;
        reset_control_state();
        if (!publish_invalid(now, mission,
                             rover_navigation_status_s::FAILURE_NOT_AUTO,
                             ready_for_auto, true)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    // Commander 切换 nav_state 后，只接受切换时间戳之后新生成的请求；等号也
    // 拒绝，避免一个旧周期与新安全投影恰好共享时间戳。
    if (vehicle_status_.nav_state_timestamp == 0U ||
        now <= vehicle_status_.nav_state_timestamp) {
        reset_control_state();
        if (!publish_invalid(now, mission,
                             rover_navigation_status_s::FAILURE_NOT_AUTO,
                             false, true)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (vehicle_status_.nav_state ==
        vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
        auto_was_active_ = false;
        reset_control_state();
        GuidanceOutput hold{};
        hold.control_state = rover_navigation_status_s::CONTROL_HOLD;
        hold.waypoint_state =
            mission.execution_state ==
                    dima::modules::mission::MissionExecutionState::Complete
                ? rover_navigation_status_s::WAYPOINT_MISSION_COMPLETE
                : rover_navigation_status_s::WAYPOINT_ACTIVE;
        hold.speed_setpoint_m_s = 0.0F;
        hold.yaw_rate_setpoint_rad_s = 0.0F;
        hold.valid = true;
        if (!publish_cycle(now, mission, hold,
                           rover_navigation_status_s::FAILURE_NONE,
                           ready_for_auto, true, true)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (mission.execution_state ==
        dima::modules::mission::MissionExecutionState::Complete) {
        // Commander 尚未消费完成状态的过渡周期继续发布物理量零请求，禁止
        // 最终航点完成后复用最后一条非零控制量。
        auto_was_active_ = false;
        reset_control_state();
        GuidanceOutput complete{};
        complete.control_state = rover_navigation_status_s::CONTROL_HOLD;
        complete.waypoint_state =
            rover_navigation_status_s::WAYPOINT_MISSION_COMPLETE;
        complete.speed_setpoint_m_s = 0.0F;
        complete.yaw_rate_setpoint_rad_s = 0.0F;
        complete.valid = true;
        if (!publish_cycle(now, mission, complete,
                           rover_navigation_status_s::FAILURE_NONE,
                           ready_for_auto, true, true)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    if (mission.execution_state !=
        dima::modules::mission::MissionExecutionState::Active) {
        reset_control_state();
        if (!publish_invalid(
                now, mission,
                rover_navigation_status_s::FAILURE_MISSION_UNAVAILABLE,
                ready_for_auto, true)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    const float dt_s = previous_run == 0U
                           ? static_cast<float>(kRunIntervalUs) * 1.0e-6F
                           : static_cast<float>(now - previous_run) * 1.0e-6F;
    if (!auto_was_active_) {
        // 模式首次激活从当前位置建立首段；HeadingController 也从实测 yaw
        // 初始化，避免 Manual 最后一帧直接变成 AUTO 的航向阶跃。
        reset_control_state();
        auto_was_active_ = true;
    }

    GuidanceOutput guidance = run_guidance(mission, dt_s);
    if (!guidance.valid || !finite(guidance.speed_setpoint_m_s) ||
        !finite(guidance.yaw_rate_setpoint_rad_s)) {
        reset_control_state();
        if (!publish_invalid(
                now, mission,
                rover_navigation_status_s::FAILURE_OUTPUT_NONFINITE,
                false, true)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    // run_guidance 可能已原子推进任务索引或完成任务；刷新快照后再发布，保证
    // status、MISSION_CURRENT 的代际/索引与本周期 waypoint_state 一致。
    dima::modules::mission::MissionStatus published_mission = mission;
    (void)refresh_mission_snapshot(now, published_mission);
    if (!publish_cycle(now, published_mission, guidance,
                       rover_navigation_status_s::FAILURE_NONE,
                       ready_for_auto, true, true)) {
        enter_error(kEventPublishFailure);
    }
}

bool AutoMode::bind_parameters() noexcept
{
    const bool bound = lookahead_gain_.bind() && lookahead_min_.bind() &&
        lookahead_max_.bind() && acceptance_radius_.bind() && yaw_p_.bind() &&
        yaw_rate_limit_.bind() && turn_to_drive_.bind() &&
        drive_to_turn_.bind() && maximum_speed_.bind() &&
        speed_limit_.bind() && deceleration_limit_.bind() &&
        jerk_limit_.bind() && speed_reduction_.bind() &&
        speed_threshold_.bind() && speed_p_.bind() && speed_i_.bind() &&
        acceleration_limit_.bind() && yaw_rate_p_.bind() &&
        yaw_rate_i_.bind() && yaw_rate_correction_.bind() &&
        yaw_acceleration_.bind() && yaw_deceleration_.bind() &&
        yaw_rate_threshold_.bind() && wheel_track_.bind();
    if (!bound) {
        invalidate_parameter_bindings();
        parameters_valid_ = false;
        return false;
    }

    // 参数句柄存在是模块可运行的结构合同；车辆增益仍为零时只保持
    // parameters_valid=false，使 QGC 可在 Disarmed 下标定后解锁 AUTO。
    (void)apply_parameter_snapshot();
    return true;
}

void AutoMode::invalidate_parameter_bindings() noexcept
{
    lookahead_gain_.invalidate();
    lookahead_min_.invalidate();
    lookahead_max_.invalidate();
    acceptance_radius_.invalidate();
    yaw_p_.invalidate();
    yaw_rate_limit_.invalidate();
    turn_to_drive_.invalidate();
    drive_to_turn_.invalidate();
    maximum_speed_.invalidate();
    speed_limit_.invalidate();
    deceleration_limit_.invalidate();
    jerk_limit_.invalidate();
    speed_reduction_.invalidate();
    speed_threshold_.invalidate();
    speed_p_.invalidate();
    speed_i_.invalidate();
    acceleration_limit_.invalidate();
    yaw_rate_p_.invalidate();
    yaw_rate_i_.invalidate();
    yaw_rate_correction_.invalidate();
    yaw_acceleration_.invalidate();
    yaw_deceleration_.invalidate();
    yaw_rate_threshold_.invalidate();
    wheel_track_.invalidate();
}

bool AutoMode::apply_parameter_snapshot() noexcept
{
    if (!lookahead_gain_.bound() || !lookahead_min_.bound() ||
        !lookahead_max_.bound() || !acceptance_radius_.bound() ||
        !yaw_p_.bound() || !yaw_rate_limit_.bound() ||
        !turn_to_drive_.bound() || !drive_to_turn_.bound() ||
        !maximum_speed_.bound() || !speed_limit_.bound() ||
        !deceleration_limit_.bound() || !jerk_limit_.bound() ||
        !speed_reduction_.bound() || !speed_threshold_.bound() ||
        !speed_p_.bound() || !speed_i_.bound() ||
        !acceleration_limit_.bound() || !yaw_rate_p_.bound() ||
        !yaw_rate_i_.bound() || !yaw_rate_correction_.bound() ||
        !yaw_acceleration_.bound() || !yaw_deceleration_.bound() ||
        !yaw_rate_threshold_.bound() || !wheel_track_.bound()) {
        parameters_valid_ = false;
        return false;
    }

    Config candidate{};
    float yaw_rate_limit_deg_s{};
    float yaw_acceleration_deg_s2{};
    float yaw_deceleration_deg_s2{};
    float yaw_rate_threshold_deg_s{};
    bool loaded = false;
    {
        // 四环与路径参数必须来自同一 Parameter epoch，禁止把一次 QGC 批量
        // 写入的前半组旧值和后半组新值拼成不可复现的控制器配置。
        px4::AtomicTransaction transaction;
        loaded =
            param_get(lookahead_gain_.handle(),
                      &candidate.pure_pursuit.lookahead_gain) == 0 &&
            param_get(lookahead_min_.handle(),
                      &candidate.pure_pursuit.lookahead_min_m) == 0 &&
            param_get(lookahead_max_.handle(),
                      &candidate.pure_pursuit.lookahead_max_m) == 0 &&
            param_get(acceptance_radius_.handle(),
                      &candidate.default_acceptance_radius_m) == 0 &&
            param_get(yaw_p_.handle(),
                      &candidate.heading.proportional_gain) == 0 &&
            param_get(yaw_rate_limit_.handle(),
                      &yaw_rate_limit_deg_s) == 0 &&
            param_get(turn_to_drive_.handle(),
                      &candidate.driving.turn_to_drive_yaw_error_rad) == 0 &&
            param_get(drive_to_turn_.handle(),
                      &candidate.driving.drive_to_turn_yaw_error_rad) == 0 &&
            param_get(speed_threshold_.handle(),
                      &candidate.driving.stopped_speed_threshold_m_s) == 0 &&
            param_get(maximum_speed_.handle(),
                      &candidate.speed_inner.speed_at_full_throttle_m_s) == 0 &&
            param_get(speed_limit_.handle(),
                      &candidate.cruise_speed_m_s) == 0 &&
            param_get(speed_p_.handle(),
                      &candidate.speed_inner.proportional_gain) == 0 &&
            param_get(speed_i_.handle(),
                      &candidate.speed_inner.integral_gain) == 0 &&
            param_get(acceleration_limit_.handle(),
                      &candidate.speed_inner.acceleration_limit_m_s2) == 0 &&
            param_get(deceleration_limit_.handle(),
                      &candidate.speed_inner.deceleration_limit_m_s2) == 0 &&
            param_get(speed_threshold_.handle(),
                      &candidate.speed_inner.measurement_threshold_m_s) == 0 &&
            param_get(yaw_rate_p_.handle(),
                      &candidate.yaw_rate_inner.proportional_gain) == 0 &&
            param_get(yaw_rate_i_.handle(),
                      &candidate.yaw_rate_inner.integral_gain) == 0 &&
            param_get(yaw_rate_correction_.handle(),
                      &candidate.yaw_rate_inner.yaw_rate_correction) == 0 &&
            param_get(wheel_track_.handle(),
                      &candidate.yaw_rate_inner.wheel_track_m) == 0 &&
            param_get(yaw_acceleration_.handle(),
                      &yaw_acceleration_deg_s2) == 0 &&
            param_get(yaw_deceleration_.handle(),
                      &yaw_deceleration_deg_s2) == 0 &&
            param_get(yaw_rate_threshold_.handle(),
                      &yaw_rate_threshold_deg_s) == 0 &&
            param_get(jerk_limit_.handle(),
                      &candidate.jerk_limit_m_s3) == 0 &&
            param_get(deceleration_limit_.handle(),
                      &candidate.deceleration_limit_m_s2) == 0 &&
            param_get(speed_reduction_.handle(),
                      &candidate.speed_reduction_gain) == 0;
    }

    candidate.heading.yaw_rate_limit_rad_s =
        angular_parameter_to_radians(yaw_rate_limit_deg_s);
    candidate.yaw_rate_inner.speed_at_full_throttle_m_s =
        candidate.speed_inner.speed_at_full_throttle_m_s;
    candidate.yaw_rate_inner.yaw_rate_limit_rad_s =
        candidate.heading.yaw_rate_limit_rad_s;
    candidate.yaw_rate_inner.yaw_acceleration_limit_rad_s2 =
        angular_parameter_to_radians(yaw_acceleration_deg_s2);
    candidate.yaw_rate_inner.yaw_deceleration_limit_rad_s2 =
        angular_parameter_to_radians(yaw_deceleration_deg_s2);
    candidate.yaw_rate_inner.measurement_threshold_rad_s =
        angular_parameter_to_radians(yaw_rate_threshold_deg_s);

    dima::lib::rover::SpeedController speed_validator{};
    dima::lib::rover::YawRateController yaw_rate_validator{};
    if (!loaded || !valid_config(candidate) ||
        !speed_validator.configure(candidate.speed_inner) ||
        !yaw_rate_validator.configure(candidate.yaw_rate_inner) ||
        !pure_pursuit_.configure(candidate.pure_pursuit) ||
        !heading_controller_.configure(candidate.heading) ||
        !driving_state_.configure(candidate.driving)) {
        parameters_valid_ = false;
        reset_control_state();
        return false;
    }

    lookahead_gain_.set(candidate.pure_pursuit.lookahead_gain);
    lookahead_min_.set(candidate.pure_pursuit.lookahead_min_m);
    lookahead_max_.set(candidate.pure_pursuit.lookahead_max_m);
    acceptance_radius_.set(candidate.default_acceptance_radius_m);
    yaw_p_.set(candidate.heading.proportional_gain);
    yaw_rate_limit_.set(yaw_rate_limit_deg_s);
    turn_to_drive_.set(candidate.driving.turn_to_drive_yaw_error_rad);
    drive_to_turn_.set(candidate.driving.drive_to_turn_yaw_error_rad);
    maximum_speed_.set(candidate.speed_inner.speed_at_full_throttle_m_s);
    speed_limit_.set(candidate.cruise_speed_m_s);
    deceleration_limit_.set(candidate.deceleration_limit_m_s2);
    jerk_limit_.set(candidate.jerk_limit_m_s3);
    speed_reduction_.set(candidate.speed_reduction_gain);
    speed_threshold_.set(candidate.speed_inner.measurement_threshold_m_s);
    speed_p_.set(candidate.speed_inner.proportional_gain);
    speed_i_.set(candidate.speed_inner.integral_gain);
    acceleration_limit_.set(candidate.speed_inner.acceleration_limit_m_s2);
    yaw_rate_p_.set(candidate.yaw_rate_inner.proportional_gain);
    yaw_rate_i_.set(candidate.yaw_rate_inner.integral_gain);
    yaw_rate_correction_.set(candidate.yaw_rate_inner.yaw_rate_correction);
    yaw_acceleration_.set(yaw_acceleration_deg_s2);
    yaw_deceleration_.set(yaw_deceleration_deg_s2);
    yaw_rate_threshold_.set(yaw_rate_threshold_deg_s);
    wheel_track_.set(candidate.yaw_rate_inner.wheel_track_m);

    config_ = candidate;
    parameters_valid_ = true;
    reset_control_state();
    return true;
}

void AutoMode::apply_pending_parameters(std::uint64_t now) noexcept
{
    if (!parameter_update_pending_ || !safety_snapshot_fresh(now) ||
        control_mode_.flag_armed ||
        vehicle_status_.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        return;
    }

    parameter_update_pending_ = false;
    if (!apply_parameter_snapshot()) {
        (void)dima::events::report(kEventParameterInvalid,
                                   dima::events::Severity::Error);
    }
}

void AutoMode::update_subscriptions() noexcept
{
    if (parameter_update_subscription_.update()) {
        parameter_update_pending_ = true;
    }
    if (local_position_subscription_.update()) {
        local_position_ = local_position_subscription_.get();
        have_local_position_ = true;
    }
    if (vehicle_odometry_subscription_.update()) {
        vehicle_odometry_ = vehicle_odometry_subscription_.get();
        have_odometry_ = true;
    }
    if (control_mode_subscription_.update()) {
        control_mode_ = control_mode_subscription_.get();
        have_control_mode_ = true;
    }
    if (vehicle_status_subscription_.update()) {
        vehicle_status_ = vehicle_status_subscription_.get();
        have_vehicle_status_ = true;
    }
}

bool AutoMode::safety_snapshot_fresh(std::uint64_t now) const noexcept
{
    return have_control_mode_ && have_vehicle_status_ &&
        control_mode_.timestamp != 0U && vehicle_status_.timestamp != 0U &&
        // Commander 对同一安全代际使用同一个发布时间；拒绝跨代拼接的
        // control/status，即使两者字段暂时看起来一致也不能解锁 AUTO readiness。
        control_mode_.timestamp == vehicle_status_.timestamp &&
        control_mode_.timestamp <= now && vehicle_status_.timestamp <= now &&
        now - control_mode_.timestamp <= kSafetyTopicTimeoutUs &&
        now - vehicle_status_.timestamp <= kSafetyTopicTimeoutUs &&
        control_mode_.source_id == vehicle_status_.nav_state &&
        control_mode_.flag_armed ==
            (vehicle_status_.arming_state ==
             vehicle_status_s::ARMING_STATE_ARMED);
}

bool AutoMode::auto_projection(std::uint8_t nav_state,
                               bool require_armed) const noexcept
{
    const bool supported_state =
        nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
    const bool exact_flags = control_mode_.flag_control_auto_enabled &&
        control_mode_.flag_control_position_enabled &&
        control_mode_.flag_control_velocity_enabled &&
        control_mode_.flag_control_attitude_enabled &&
        control_mode_.flag_control_rates_enabled &&
        !control_mode_.flag_control_manual_enabled &&
        !control_mode_.flag_control_offboard_enabled &&
        !control_mode_.flag_control_termination_enabled &&
        !control_mode_.flag_control_altitude_enabled &&
        !control_mode_.flag_control_climb_rate_enabled &&
        !control_mode_.flag_control_acceleration_enabled &&
        !control_mode_.flag_control_allocation_enabled &&
        !control_mode_.flag_multicopter_position_control_enabled;
    const bool armed = control_mode_.flag_armed &&
        vehicle_status_.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
    return supported_state && vehicle_status_.nav_state == nav_state &&
        control_mode_.source_id == nav_state && exact_flags &&
        (!require_armed || armed);
}

bool AutoMode::estimator_healthy(std::uint64_t now) const noexcept
{
    if (!have_local_position_ || !have_odometry_ ||
        local_position_.timestamp == 0U ||
        local_position_.timestamp_sample == 0U ||
        local_position_.timestamp_sample > local_position_.timestamp ||
        local_position_.timestamp > now ||
        now - local_position_.timestamp > kEstimatorTimeoutUs ||
        now - local_position_.timestamp_sample > kEstimatorTimeoutUs ||
        !local_position_.xy_valid || !local_position_.v_xy_valid ||
        !local_position_.heading_good_for_control ||
        !local_position_.xy_global || local_position_.dead_reckoning ||
        local_position_.ref_timestamp == 0U ||
        local_position_.ref_timestamp > local_position_.timestamp ||
        vehicle_odometry_.timestamp == 0U ||
        vehicle_odometry_.timestamp_sample == 0U ||
        vehicle_odometry_.timestamp_sample > vehicle_odometry_.timestamp ||
        vehicle_odometry_.timestamp > now ||
        now - vehicle_odometry_.timestamp > kEstimatorTimeoutUs ||
        vehicle_odometry_.timestamp_sample > now ||
        now - vehicle_odometry_.timestamp_sample > kEstimatorTimeoutUs) {
        return false;
    }

    // Pure Pursuit/Heading P 使用 local position，YawRate PI 使用 odometry.z；
    // AutoMode 同时复核两者，使 Commander 能把任一估计输入失鲜统一降级 Hold。
    return finite(local_position_.x) && finite(local_position_.y) &&
        finite(local_position_.vx) && finite(local_position_.vy) &&
        finite(local_position_.heading) &&
        finite(vehicle_odometry_.angular_velocity[2]) &&
        std::isfinite(local_position_.ref_lat) &&
        std::isfinite(local_position_.ref_lon) &&
        local_position_.ref_lat >= -90.0 &&
        local_position_.ref_lat <= 90.0 &&
        local_position_.ref_lon >= -180.0 &&
        local_position_.ref_lon <= 180.0;
}

bool AutoMode::estimator_reset_detected() noexcept
{
    const bool changed = !have_estimator_baseline_ ||
        xy_reset_counter_ != local_position_.xy_reset_counter ||
        velocity_reset_counter_ != local_position_.vxy_reset_counter ||
        heading_reset_counter_ != local_position_.heading_reset_counter ||
        odometry_reset_counter_ != vehicle_odometry_.reset_counter ||
        projection_reference_timestamp_ != local_position_.ref_timestamp;

    xy_reset_counter_ = local_position_.xy_reset_counter;
    velocity_reset_counter_ = local_position_.vxy_reset_counter;
    heading_reset_counter_ = local_position_.heading_reset_counter;
    odometry_reset_counter_ = vehicle_odometry_.reset_counter;
    projection_reference_timestamp_ = local_position_.ref_timestamp;
    have_estimator_baseline_ = true;
    return changed;
}

bool AutoMode::refresh_mission_snapshot(
    std::uint64_t now,
    dima::modules::mission::MissionStatus &status) noexcept
{
    dima::modules::mission::MissionStatus fresh{};
    const int result = mission_service_.status(fresh);
    if (result == 0) {
        status = fresh;

        // 导航只缓存一份已经完整提交且结构自洽的任务状态。staging/clear
        // 事务期间即使旧 active 仍在 RAM，也不能把它当成可启动的新快照。
        const bool valid_snapshot = fresh.loaded && fresh.committed &&
            !fresh.mutation_in_progress && fresh.mission_id != 0U &&
            fresh.count > 0U &&
            fresh.count <= dima::modules::mission::kMissionCapacity &&
            fresh.current < fresh.count &&
            fresh.execution_state !=
                dima::modules::mission::MissionExecutionState::NoMission;
        if (!valid_snapshot) {
            cached_mission_status_ = fresh;
            have_mission_status_ = false;
            last_mission_status_us_ = 0U;
            mission_plan_ = {};
            mission_plan_valid_ = false;
            segment_valid_ = false;
            return false;
        }

        cached_mission_status_ = fresh;
        last_mission_status_us_ = now;
        have_mission_status_ = true;
        return true;
    }

    // MissionService 使用 no-wait 互斥，QGC 回读或 storage 提交可能让单个
    // 20 ms 周期遇到 -EAGAIN。只在 200 ms 内复用“同 mission_id、同 count、
    // 同 current”的冻结计划，避免一次正常锁竞争把正在运行的 AUTO 推入 Hold。
    const bool cache_fresh = result == -EAGAIN && have_mission_status_ &&
        last_mission_status_us_ != 0U && now >= last_mission_status_us_ &&
        now - last_mission_status_us_ <= kMissionStatusCacheUs;
    const bool same_plan = cache_fresh && mission_plan_valid_ &&
        cached_mission_status_.mission_id == mission_plan_.mission_id &&
        cached_mission_status_.count == mission_plan_.count &&
        cached_mission_status_.current == mission_plan_.current;
    if (same_plan) {
        status = cached_mission_status_;
        return true;
    }

    if (have_mission_status_) {
        status = cached_mission_status_;
    }
    have_mission_status_ = false;
    last_mission_status_us_ = 0U;
    mission_plan_valid_ = false;
    segment_valid_ = false;
    return false;
}

bool AutoMode::refresh_mission_plan(
    const dima::modules::mission::MissionStatus &status) noexcept
{
    if (!status.loaded || !status.committed || status.mutation_in_progress ||
        status.mission_id == 0U || status.count == 0U ||
        status.count > dima::modules::mission::kMissionCapacity ||
        status.current >= status.count) {
        mission_plan_ = {};
        mission_plan_valid_ = false;
        segment_valid_ = false;
        return false;
    }

    if (mission_plan_valid_ &&
        mission_plan_.mission_id == status.mission_id &&
        mission_plan_.count == status.count) {
        // 航点推进只改变 current，不改变任务内容；同步本地索引即可，禁止在
        // 50 Hz 外环中为每个航点重复复制完整 64 项数组并争抢 Mission mutex。
        mission_plan_.current = status.current;
        return true;
    }

    dima::modules::mission::MissionPlan candidate{};
    if (mission_service_.active_plan(candidate) != 0 ||
        candidate.mission_id != status.mission_id ||
        candidate.count != status.count ||
        candidate.current != status.current ||
        candidate.count == 0U ||
        candidate.count > dima::modules::mission::kMissionCapacity) {
        mission_plan_valid_ = false;
        segment_valid_ = false;
        return false;
    }

    for (std::uint16_t sequence = 0U; sequence < candidate.count;
         ++sequence) {
        if (candidate.items[sequence].sequence != sequence) {
            mission_plan_valid_ = false;
            segment_valid_ = false;
            return false;
        }
    }

    // mission_id/count 是任务内容代际；只有代际变化时替换整份冻结快照。
    // 接受新计划同时清空 Heading slew 与转向状态，防止旧航段控制量串入新任务。
    mission_plan_ = candidate;
    mission_plan_valid_ = true;
    reset_control_state();
    return true;
}

bool AutoMode::prepare_segment(
    const dima::modules::mission::MissionStatus &status,
    bool rebuild_from_vehicle) noexcept
{
    if (!status.committed || status.mission_id == 0U ||
        status.count == 0U || status.current >= status.count ||
        status.execution_state !=
            dima::modules::mission::MissionExecutionState::Active ||
        !mission_plan_valid_ ||
        mission_plan_.mission_id != status.mission_id ||
        mission_plan_.count != status.count ||
        mission_plan_.current != status.current ||
        !estimator_healthy(hrt_absolute_time())) {
        return false;
    }

    if (!projection_.isInitialized() ||
        projection_.getProjectionReferenceTimestamp() !=
            local_position_.ref_timestamp) {
        projection_.initReference(local_position_.ref_lat,
                                  local_position_.ref_lon,
                                  local_position_.ref_timestamp);
    }

    const auto &target_item = mission_plan_.items[status.current];
    dima::lib::rover::Position2f candidate_target{};
    if (target_item.sequence != status.current ||
        !project_item(target_item, candidate_target)) {
        return false;
    }

    dima::lib::rover::Position2f candidate_start{};
    if (rebuild_from_vehicle || status.current == 0U) {
        // 首段或 EKF reset 后，路径起点必须是当前车位。继续使用旧全球前一
        // 航点会在新 NED 原点中制造一条跨 reset 的虚假横向误差。
        candidate_start = {local_position_.x, local_position_.y};
    } else {
        const auto &previous_item =
            mission_plan_.items[status.current - 1U];
        if (previous_item.sequence != status.current - 1U ||
            !project_item(previous_item, candidate_start)) {
            return false;
        }
    }

    const float acceptance_radius =
        finite(target_item.acceptance_radius_m) &&
                target_item.acceptance_radius_m > 0.0F
            ? target_item.acceptance_radius_m
            : config_.default_acceptance_radius_m;
    if (!finite(acceptance_radius) || acceptance_radius <= 0.0F) {
        return false;
    }

    const bool final_waypoint = status.current + 1U >= status.count;
    float arrival_speed = 0.0F;
    if (!final_waypoint) {
        const auto &next_item = mission_plan_.items[status.current + 1U];
        dima::lib::rover::Position2f next_position{};
        if (next_item.sequence != status.current + 1U ||
            !project_item(next_item, next_position)) {
            return false;
        }

        const float incoming_length =
            vector_length(candidate_start, candidate_target);
        const float outgoing_length =
            vector_length(candidate_target, next_position);
        if (incoming_length > FLT_EPSILON && outgoing_length > FLT_EPSILON) {
            const float turn_angle = std::fabs(wrap_pi(
                vector_bearing(candidate_target, next_position) -
                vector_bearing(candidate_start, candidate_target)));
            if (!finite(turn_angle)) {
                return false;
            }

            // 大转角的到达速度固定为零；小转角按 RO_SPEED_RED 计算非零
            // 通过速度。该值在进入航段时冻结，避免逐周期任务锁竞争和抖动。
            if (turn_angle <=
                config_.driving.drive_to_turn_yaw_error_rad) {
                arrival_speed = dima::lib::rover::
                    reduce_speed_for_heading_error(
                        config_.cruise_speed_m_s, turn_angle,
                        config_.speed_inner.speed_at_full_throttle_m_s,
                        config_.speed_reduction_gain);
                if (!finite(arrival_speed)) {
                    return false;
                }
                arrival_speed = std::fmax(arrival_speed, 0.0F);
            }
        }
    }

    segment_start_ = candidate_start;
    segment_target_ = candidate_target;
    segment_acceptance_radius_m_ = acceptance_radius;
    segment_arrival_speed_m_s_ = arrival_speed;
    segment_final_waypoint_ = final_waypoint;
    segment_mission_id_ = status.mission_id;
    segment_sequence_ = status.current;
    segment_valid_ = true;
    rebuild_segment_from_vehicle_ = false;
    waypoint_advance_pending_ = false;
    mission_complete_pending_ = false;
    return true;
}

bool AutoMode::project_item(
    const dima::modules::mission::MissionItem &item,
    dima::lib::rover::Position2f &position) const noexcept
{
    if (!projection_.isInitialized()) {
        return false;
    }
    const double latitude = static_cast<double>(item.latitude_e7) * 1.0e-7;
    const double longitude = static_cast<double>(item.longitude_e7) * 1.0e-7;
    if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
        latitude < -90.0 || latitude > 90.0 || longitude < -180.0 ||
        longitude > 180.0) {
        return false;
    }

    projection_.project(latitude, longitude, position.north_m,
                        position.east_m);
    return finite(position.north_m) && finite(position.east_m);
}

AutoMode::GuidanceOutput AutoMode::run_guidance(
    const dima::modules::mission::MissionStatus &mission_status,
    float dt_s) noexcept
{
    GuidanceOutput output{};
    if (!finite(dt_s) || dt_s <= 0.0F ||
        dt_s > static_cast<float>(kMaximumControlDtUs) * 1.0e-6F ||
        !segment_valid_ || segment_mission_id_ != mission_status.mission_id ||
        segment_sequence_ != mission_status.current) {
        if (!prepare_segment(mission_status,
                             rebuild_segment_from_vehicle_)) {
            return output;
        }
    }

    const auto measured_speed = dima::lib::rover::measure_body_speed(
        local_position_.vx, local_position_.vy, local_position_.heading,
        config_.driving.stopped_speed_threshold_m_s);
    if (!measured_speed.valid) {
        return output;
    }

    const dima::lib::rover::Position2f vehicle_position{
        local_position_.x, local_position_.y};
    // Pure Pursuit 的 L=clamp(k*|ground_speed|, Lmin, Lmax) 使用 NED
    // 水平地速模长；它与 Speed PI 的 sign(v_body_x)*hypot(v_body_x,v_body_y)
    // 不是同一个量。若复用带前向符号的测量，车辆纯横向滑动时 sign 为 0，
    // 会把实际非零地速误报为零并错误缩短前视距离。
    const float ground_speed_m_s = std::hypot(
        measured_speed.forward_m_s, measured_speed.lateral_m_s);
    if (!finite(ground_speed_m_s)) {
        return output;
    }
    const auto pursuit = pure_pursuit_.update(
        segment_start_, segment_target_, vehicle_position,
        ground_speed_m_s);
    if (!pursuit.valid) {
        return output;
    }

    const float guidance_heading_error = wrap_pi(
        pursuit.target_bearing_rad - local_position_.heading);
    if (!finite(guidance_heading_error)) {
        return output;
    }

    const auto heading = heading_controller_.update(
        pursuit.target_bearing_rad, local_position_.heading, dt_s);
    if (!heading.valid) {
        return output;
    }

    const auto speed_plan = dima::lib::rover::plan_waypoint_speed(
        pursuit.distance_to_waypoint_m, segment_acceptance_radius_m_,
        config_.cruise_speed_m_s, segment_arrival_speed_m_s_,
        config_.cruise_speed_m_s, config_.jerk_limit_m_s3,
        config_.deceleration_limit_m_s2);
    if (!speed_plan.valid) {
        return output;
    }

    float requested_speed = dima::lib::rover::
        reduce_speed_for_heading_error(
            speed_plan.speed_setpoint_m_s, guidance_heading_error,
            config_.speed_inner.speed_at_full_throttle_m_s,
            config_.speed_reduction_gain);
    const float state_speed_setpoint =
        driving_state_.state() == dima::lib::rover::DrivingState::Driving
            ? requested_speed
            : 0.0F;
    const auto driving = driving_state_.update(
        guidance_heading_error, state_speed_setpoint,
        measured_speed.speed_m_s);
    if (!driving.valid) {
        return output;
    }

    output.distance_to_waypoint_m = pursuit.distance_to_waypoint_m;
    output.crosstrack_error_m = pursuit.crosstrack_error_m;
    output.lookahead_distance_m = pursuit.lookahead_distance_m;
    // 状态迁移和 QGC 状态显示使用未经过 slew 的路径航向误差，保证侧后方
    // 航点在第一周期就进入停车确认；Heading P 内部仍用受限误差平滑 yaw-rate。
    output.heading_error_rad = guidance_heading_error;
    output.speed_setpoint_m_s =
        driving.translation_enabled ? requested_speed : 0.0F;
    output.yaw_rate_setpoint_rad_s =
        driving.heading_control_enabled
            ? heading.yaw_rate_setpoint_rad_s
            : 0.0F;
    output.control_state = control_state(driving.state);
    output.waypoint_state = speed_plan.waypoint_inside_acceptance
                                ? rover_navigation_status_s::
                                      WAYPOINT_INSIDE_ACCEPTANCE
                                : rover_navigation_status_s::WAYPOINT_ACTIVE;
    output.valid = finite(output.speed_setpoint_m_s) &&
        finite(output.yaw_rate_setpoint_rad_s) &&
        finite(output.heading_error_rad);
    if (!output.valid) {
        return output;
    }

    const bool stopped = std::fabs(measured_speed.speed_m_s) <=
                         config_.driving.stopped_speed_threshold_m_s;
    if (segment_final_waypoint_) {
        if (speed_plan.waypoint_inside_acceptance) {
            // 最终航点一旦进入 acceptance radius 就锁存停车意图。
            // GNSS 在车辆减速期间抖出半径不得重新给速度，否则会在
            // 到达边界反复起停；完成事件仍必须等实测速度收敛。
            mission_complete_pending_ = true;
        }
        if (!mission_complete_pending_) {
            return output;
        }

        output.speed_setpoint_m_s = 0.0F;
        // 最终航点进入 acceptance 后的目标是完整停车，不再追踪
        // Pure Pursuit 的瞬时航向。减速确认期同时置零 yaw-rate，
        // 防止纵向速度已很低时又进入非预期原地转向。
        output.yaw_rate_setpoint_rad_s = 0.0F;
        if (!stopped) {
            return output;
        }

        // 进入半径与实测已停均成立后才提交完成；Mission mutex
        // 短暂忙时保留锁存，继续零输出并重试，不丢失最终到达事件。
        output.yaw_rate_setpoint_rad_s = 0.0F;
        output.waypoint_state =
            rover_navigation_status_s::WAYPOINT_INSIDE_ACCEPTANCE;
        const int completed = mission_service_.complete_execution(
            mission_status.mission_id, mission_status.current);
        if (completed == 0) {
            cached_mission_status_ = mission_status;
            cached_mission_status_.execution_state =
                dima::modules::mission::MissionExecutionState::Complete;
            mission_plan_.current = mission_status.current;
            output.yaw_rate_setpoint_rad_s = 0.0F;
            output.control_state = rover_navigation_status_s::CONTROL_HOLD;
            output.waypoint_state =
                rover_navigation_status_s::WAYPOINT_MISSION_COMPLETE;
            // 完成事件可能被 Commander 与新的 Mission Start 在下一次 50 Hz
            // 周期前连续消费；当周期即清 Heading slew/转向/航段状态并标记退出，
            // 即使同内容单航点任务的 mission_id/current 都不变也会全新起步。
            auto_was_active_ = false;
            reset_control_state();
        } else if (completed != -EAGAIN) {
            output.valid = false;
        }
        return output;
    }

    if (speed_plan.waypoint_inside_acceptance) {
        // 停车型中间航点也在首次进圈时锁存；后续即使 GNSS
        // 抖出半径，仍等设定/实测速度归零再推进。带速通过航点
        // 则会在本周期直接尝试 advance。
        waypoint_advance_pending_ = true;
    }
    if (!waypoint_advance_pending_) {
        return output;
    }

    const bool stop_required = segment_arrival_speed_m_s_ <=
                               config_.driving.stopped_speed_threshold_m_s;
    if (stop_required) {
        output.speed_setpoint_m_s = 0.0F;
        // 大转角中间航点必须先完整停车再切换到下一航段。
        // 在 advance 成功前同时压住 yaw-rate，确保“停车确认→
        // 新航段 StoppingForTurn→SpotTurning”之间没有一帧旧转向量。
        output.yaw_rate_setpoint_rad_s = 0.0F;
    }
    if (stop_required && !stopped) {
        return output;
    }

    // 带速通过航点可能在一次 -EAGAIN 后立即驶出 acceptance；锁存“已经满足
    // 到达条件”，后续周期继续推进同一 mission_id/current，成功前不丢事件。
    output.waypoint_state =
        rover_navigation_status_s::WAYPOINT_INSIDE_ACCEPTANCE;
    const int advanced = mission_service_.advance_current(
        mission_status.mission_id, mission_status.current);
    if (advanced == 0) {
        waypoint_advance_pending_ = false;
        cached_mission_status_ = mission_status;
        ++cached_mission_status_.current;
        mission_plan_.current = cached_mission_status_.current;
        output.waypoint_state = rover_navigation_status_s::WAYPOINT_REACHED;
        segment_valid_ = false;
        // 正常航点推进使用“已到达航点 -> 下一航点”的全球航段；只有 EKF
        // reset 才以当前车位重建，避免小角度通过时把路径起点随车漂移。
        rebuild_segment_from_vehicle_ = false;
    } else if (advanced != -EAGAIN) {
        output.valid = false;
    }
    return output;
}

bool AutoMode::publish_cycle(
    std::uint64_t now,
    const dima::modules::mission::MissionStatus &mission,
    const GuidanceOutput &guidance, std::uint8_t failure_reason,
    bool ready_for_auto, bool estimator_is_healthy,
    bool request_valid) noexcept
{
    const bool finite_request = guidance.valid &&
        finite(guidance.speed_setpoint_m_s) &&
        finite(guidance.yaw_rate_setpoint_rad_s);
    const bool valid = request_valid && finite_request;

    rover_motion_request_s request{};
    request.timestamp = now;
    // 请求的 sample 时间必须覆盖外环 local position 与内环 yaw-rate 的最旧
    // 输入；下游再与自己的快照取 min，任何一路旧样本都不能被较新发布时刻掩盖。
    request.timestamp_sample =
        have_local_position_ && have_odometry_
            ? (local_position_.timestamp_sample <
                       vehicle_odometry_.timestamp_sample
                   ? local_position_.timestamp_sample
                   : vehicle_odometry_.timestamp_sample)
            : 0U;
    request_sequence_ = request_sequence_ == UINT32_MAX
                            ? 1U
                            : request_sequence_ + 1U;
    request.sequence = request_sequence_;
    request.valid = valid;
    request.source = rover_motion_request_s::SOURCE_NAVIGATION;
    request.mode = rover_motion_request_s::MODE_SPEED_YAW_RATE;
    request.normalized_longitudinal = kUnavailable;
    request.normalized_steering = kUnavailable;
    request.speed_m_s = valid ? guidance.speed_setpoint_m_s : kUnavailable;
    request.yaw_rate_rad_s =
        valid ? guidance.yaw_rate_setpoint_rad_s : kUnavailable;

    rover_navigation_status_s status{};
    status.timestamp = now;
    status.timestamp_sample = request.timestamp_sample;
    status.mission_generation = mission.mission_id;
    status.mission_current = mission.current;
    status.mission_count = mission.count;
    status.control_state = valid
                               ? guidance.control_state
                               : rover_navigation_status_s::CONTROL_INACTIVE;
    status.failure_reason = failure_reason;
    status.waypoint_state = guidance.waypoint_state;
    status.request_valid = valid;
    status.ready_for_auto = ready_for_auto;
    status.mission_committed = mission.loaded && mission.committed &&
                               mission.count > 0U;
    status.parameters_valid = parameters_valid_;
    status.estimator_healthy = estimator_is_healthy;
    status.distance_to_waypoint_m =
        guidance.valid && finite(guidance.distance_to_waypoint_m)
            ? guidance.distance_to_waypoint_m
            : kUnavailable;
    status.crosstrack_error_m =
        guidance.valid && finite(guidance.crosstrack_error_m)
            ? guidance.crosstrack_error_m
            : kUnavailable;
    status.lookahead_distance_m =
        guidance.valid && finite(guidance.lookahead_distance_m)
            ? guidance.lookahead_distance_m
            : kUnavailable;
    status.heading_error_rad =
        guidance.valid && finite(guidance.heading_error_rad)
            ? guidance.heading_error_rad
            : kUnavailable;
    status.speed_setpoint_m_s =
        valid ? guidance.speed_setpoint_m_s : kUnavailable;
    status.yaw_rate_setpoint_rad_s =
        valid ? guidance.yaw_rate_setpoint_rad_s : kUnavailable;

    // 失效周期先发布原因、再发布显式无效请求：Commander 因而能在下游停波状态
    // 到达前看到同代导航证据并切入 Hold。有效周期采用相同固定顺序，避免正负
    // 路径形成不同的跨 WorkQueue 竞态；两次 publish 任一失败仍由调用者报错。
    const bool status_published = navigation_status_publication_.publish(status);
    const bool request_published = motion_request_publication_.publish(request);
    return request_published && status_published;
}

bool AutoMode::publish_invalid(
    std::uint64_t now,
    const dima::modules::mission::MissionStatus &mission,
    std::uint8_t failure_reason, bool ready_for_auto,
    bool estimator_is_healthy) noexcept
{
    GuidanceOutput invalid{};
    invalid.distance_to_waypoint_m = kUnavailable;
    invalid.crosstrack_error_m = kUnavailable;
    invalid.lookahead_distance_m = kUnavailable;
    invalid.heading_error_rad = kUnavailable;
    invalid.speed_setpoint_m_s = kUnavailable;
    invalid.yaw_rate_setpoint_rad_s = kUnavailable;
    invalid.valid = false;
    return publish_cycle(now, mission, invalid, failure_reason,
                         ready_for_auto, estimator_is_healthy, false);
}

void AutoMode::reset_control_state() noexcept
{
    heading_controller_.reset();
    driving_state_.reset();
    segment_start_ = {};
    segment_target_ = {};
    segment_acceptance_radius_m_ = 0.0F;
    segment_arrival_speed_m_s_ = 0.0F;
    segment_mission_id_ = 0U;
    segment_sequence_ = 0U;
    segment_valid_ = false;
    segment_final_waypoint_ = false;
    waypoint_advance_pending_ = false;
    mission_complete_pending_ = false;
    rebuild_segment_from_vehicle_ = true;
}

void AutoMode::reset_runtime_state() noexcept
{
    reset_control_state();
    config_ = {};
    projection_ = {};
    mission_plan_ = {};
    cached_mission_status_ = {};
    local_position_ = {};
    vehicle_odometry_ = {};
    control_mode_ = {};
    vehicle_status_ = {};
    last_run_us_ = 0U;
    last_estimator_timestamp_us_ = 0U;
    last_estimator_sample_us_ = 0U;
    last_odometry_timestamp_us_ = 0U;
    last_odometry_sample_us_ = 0U;
    last_mission_status_us_ = 0U;
    projection_reference_timestamp_ = 0U;
    request_sequence_ = 0U;
    xy_reset_counter_ = 0U;
    velocity_reset_counter_ = 0U;
    heading_reset_counter_ = 0U;
    odometry_reset_counter_ = 0U;
    have_local_position_ = false;
    have_odometry_ = false;
    have_control_mode_ = false;
    have_vehicle_status_ = false;
    have_estimator_baseline_ = false;
    mission_plan_valid_ = false;
    have_mission_status_ = false;
    parameters_valid_ = false;
    parameter_update_pending_ = false;
    auto_was_active_ = false;
}

void AutoMode::enter_error(std::uint32_t event_id) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    ScheduleCancelAndDrain();
    reset_control_state();
    (void)dima::events::report(event_id, dima::events::Severity::Error);
}

bool AutoMode::finite(float value) noexcept
{
    return std::isfinite(value);
}

bool AutoMode::valid_config(const Config &config) noexcept
{
    const auto &pp = config.pure_pursuit;
    const auto &heading = config.heading;
    const auto &driving = config.driving;
    const auto &speed = config.speed_inner;
    const auto &yaw_rate = config.yaw_rate_inner;

    return finite(pp.lookahead_gain) && pp.lookahead_gain > 0.0F &&
        finite(pp.lookahead_min_m) && pp.lookahead_min_m > 0.0F &&
        finite(pp.lookahead_max_m) &&
        pp.lookahead_max_m >= pp.lookahead_min_m &&
        finite(config.default_acceptance_radius_m) &&
        config.default_acceptance_radius_m > 0.0F &&
        finite(config.cruise_speed_m_s) && config.cruise_speed_m_s > 0.0F &&
        config.cruise_speed_m_s <= speed.speed_at_full_throttle_m_s &&
        finite(config.jerk_limit_m_s3) && config.jerk_limit_m_s3 > 0.0F &&
        finite(config.deceleration_limit_m_s2) &&
        config.deceleration_limit_m_s2 > 0.0F &&
        finite(config.speed_reduction_gain) &&
        config.speed_reduction_gain >= -1.0F &&
        finite(heading.proportional_gain) &&
        heading.proportional_gain > 0.0F &&
        finite(heading.yaw_rate_limit_rad_s) &&
        heading.yaw_rate_limit_rad_s > 0.0F &&
        finite(driving.turn_to_drive_yaw_error_rad) &&
        driving.turn_to_drive_yaw_error_rad > 0.0F &&
        finite(driving.drive_to_turn_yaw_error_rad) &&
        driving.drive_to_turn_yaw_error_rad >
            driving.turn_to_drive_yaw_error_rad &&
        driving.drive_to_turn_yaw_error_rad <= kPi &&
        finite(driving.stopped_speed_threshold_m_s) &&
        driving.stopped_speed_threshold_m_s >= 0.0F &&
        // 停车死区必须严格小于实际巡航速度，否则正常移动会被误判为静止，
        // StoppingForTurn 可能在车辆仍前进时放行左右轮反转。
        driving.stopped_speed_threshold_m_s < config.cruise_speed_m_s &&
        finite(speed.proportional_gain) && speed.proportional_gain >= 0.0F &&
        finite(speed.integral_gain) && speed.integral_gain >= 0.0F &&
        speed.proportional_gain + speed.integral_gain > 0.0F &&
        finite(speed.speed_at_full_throttle_m_s) &&
        speed.speed_at_full_throttle_m_s > 0.0F &&
        finite(speed.acceleration_limit_m_s2) &&
        speed.acceleration_limit_m_s2 > 0.0F &&
        finite(speed.deceleration_limit_m_s2) &&
        speed.deceleration_limit_m_s2 > 0.0F &&
        finite(speed.measurement_threshold_m_s) &&
        speed.measurement_threshold_m_s >= 0.0F &&
        finite(yaw_rate.proportional_gain) &&
        yaw_rate.proportional_gain >= 0.0F &&
        finite(yaw_rate.integral_gain) && yaw_rate.integral_gain >= 0.0F &&
        yaw_rate.proportional_gain + yaw_rate.integral_gain > 0.0F &&
        finite(yaw_rate.yaw_rate_correction) &&
        yaw_rate.yaw_rate_correction > 0.0F &&
        finite(yaw_rate.wheel_track_m) && yaw_rate.wheel_track_m > 0.0F &&
        finite(yaw_rate.speed_at_full_throttle_m_s) &&
        yaw_rate.speed_at_full_throttle_m_s > 0.0F &&
        finite(yaw_rate.yaw_rate_limit_rad_s) &&
        yaw_rate.yaw_rate_limit_rad_s > 0.0F &&
        finite(yaw_rate.yaw_acceleration_limit_rad_s2) &&
        yaw_rate.yaw_acceleration_limit_rad_s2 > 0.0F &&
        finite(yaw_rate.yaw_deceleration_limit_rad_s2) &&
        yaw_rate.yaw_deceleration_limit_rad_s2 > 0.0F &&
        finite(yaw_rate.measurement_threshold_rad_s) &&
        yaw_rate.measurement_threshold_rad_s >= 0.0F &&
        // yaw-rate 死区覆盖整个可用范围时，Heading P 虽有输出，内环却永远
        // 解释为零；该配置不具备原地转向闭环能力，必须保持 AUTO 锁闭。
        yaw_rate.measurement_threshold_rad_s <
            yaw_rate.yaw_rate_limit_rad_s &&
        // SpotTurning 的退出条件是 |yaw_error|<RD_TRANS_TRN_DRV；在该边界处
        // Heading P 仍必须产生严格大于 RO_YAW_RATE_TH 的目标。否则 YawRate PI
        // 会先把目标归零并清空 slew/积分，车辆停在滞回边界外而永久无法回到
        // Driving。此处按 rad/s < (rad/s/rad)*rad 校验完整单位闭包。
        yaw_rate.measurement_threshold_rad_s <
            heading.proportional_gain *
                driving.turn_to_drive_yaw_error_rad;
}

float AutoMode::wrap_pi(float angle) noexcept
{
    if (!finite(angle)) {
        return kUnavailable;
    }
    float wrapped = std::fmod(angle + kPi, 2.0F * kPi);
    if (wrapped < 0.0F) {
        wrapped += 2.0F * kPi;
    }
    return wrapped - kPi;
}

std::uint8_t AutoMode::control_state(
    dima::lib::rover::DrivingState state) noexcept
{
    switch (state) {
    case dima::lib::rover::DrivingState::Driving:
        return rover_navigation_status_s::CONTROL_DRIVING;
    case dima::lib::rover::DrivingState::StoppingForTurn:
        return rover_navigation_status_s::CONTROL_STOPPING_FOR_TURN;
    case dima::lib::rover::DrivingState::SpotTurning:
        return rover_navigation_status_s::CONTROL_SPOT_TURNING;
    }
    return rover_navigation_status_s::CONTROL_INACTIVE;
}

} // namespace dima::rover::modes
