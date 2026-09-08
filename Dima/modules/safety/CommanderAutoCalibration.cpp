#define MODULE_NAME "commander"
#include "Commander.hpp"
#include "logging/logging.hpp"
#include "api/Time.hpp"

namespace dima::modules::safety {

void Commander::revoke_auto_calibration() noexcept
{
    // 授权既不持久化，也不从协调器状态重建；外部负向动作在已 Disarmed 的
    // 参数窗口同样清除待续行请求，防止稍后参数确认触发一次意外重新 Arm。
    authorized_calibration_session_ = 0U;
    pending_calibration_arm_ = {};
}

bool Commander::auto_calibration_fresh(std::uint64_t now) const noexcept
{
    return auto_calibration_status_.timestamp != 0U && auto_calibration_status_.timestamp <= now &&
        now - auto_calibration_status_.timestamp <= 200000ULL;
}

bool Commander::start_auto_calibration(std::uint64_t now) noexcept
{
    // 必须以新的显式模式动作在 Disarmed 进入；默认预留槽/启动恢复不会调用。
    // 选择模式不隐式 Arm；首次 awaiting_arm 才接受人工授权，后续正常续行
    // 仍须持有该 session grant 并通过同一套 preflight，不能自行重建授权。
    if (actuator_armed_.armed || !auto_calibration_fresh(now) ||
        !auto_calibration_status_.ready_for_entry || auto_calibration_status_.active ||
        !parameters_valid_ || actuator_armed_.kill || termination_latched_ || vehicle_status_.failsafe ||
        vehicle_status_.rc_calibration_in_progress || vehicle_status_.calibration_enabled || maintenance_.in_progress()) {
        PX4_WARN("Auto calibration rejected: disarmed readiness required");
        return false;
    }
    // 进入静态会话不是 Arm：无 RC/输出后端时仍可 Level；真正运动继续由
    // preflight_checks_pass 唯一核对 RC、Neutral 和完整安全条件。
    revoke_auto_calibration();
    return change_navigation_state(vehicle_status_s::NAVIGATION_STATE_EXTERNAL1, now);
}

bool Commander::process_auto_calibration(std::uint64_t now) noexcept
{
    for (unsigned i = 0U; i < 8U && auto_calibration_sub_.update(); ++i)
        auto_calibration_status_ = auto_calibration_sub_.get();
    bool changed = false;
    auto_calibration_request_s request{};
    for (unsigned i = 0U; i < 4U && auto_calibration_request_sub_.copy(&request); ++i) {
        for (unsigned j = 0U; j < 8U && auto_calibration_sub_.update(); ++j)
            auto_calibration_status_ = auto_calibration_sub_.get();
        now = hrt_absolute_time();
        if (!auto_calibration_fresh(now) || request.session_id == 0U ||
            request.session_id != auto_calibration_status_.session_id || request.timestamp == 0U ||
            request.timestamp > now || now - request.timestamp > 200000ULL) continue;
        if (request.request == auto_calibration_request_s::REQUEST_EXIT) {
            if (vehicle_status_.nav_state == vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
                changed = disarm(vehicle_status_s::ARM_DISARM_REASON_COMMAND_INTERNAL, now) == TransitionResult::Changed || changed;
        } else if (request.request == auto_calibration_request_s::REQUEST_STAGE_DISARM) {
            const auto state = auto_calibration_status_.state;
            if (auto_calibration_status_.active &&
                (state == auto_calibration_status_s::STATE_STOP_DISARM_FIRST || state == auto_calibration_status_s::STATE_STOP_DISARM_SECOND ||
                 state == auto_calibration_status_s::STATE_STOP_IDENTIFICATION || state == auto_calibration_status_s::STATE_STOP_VALIDATION ||
                 state == auto_calibration_status_s::STATE_STOP_PROFILE))
                changed = disarm(vehicle_status_s::ARM_DISARM_REASON_COMMAND_INTERNAL, now, true) == TransitionResult::Changed || changed;
        } else if (request.request == auto_calibration_request_s::REQUEST_STAGE_ARM) {
            // 此处只登记请求；本轮所有 RC/MAVLink 负向动作和安全检查完成后才
            // 处理续行，不能在已排队的用户 Disarm 之前抢先发布 Armed 快照。
            if (request.session_id == authorized_calibration_session_)
                pending_calibration_arm_ = request;
        } else if (request.request == auto_calibration_request_s::REQUEST_LEVEL) {
            if (actuator_armed_.armed || vehicle_status_.nav_state != vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 ||
                vehicle_status_.calibration_enabled || !auto_calibration_status_.active ||
                auto_calibration_status_.state != auto_calibration_status_s::STATE_LEVEL_HOLD) continue;
            sensor_calibration_request_s worker{};
            worker.timestamp = request.timestamp;
            worker.request = sensor_calibration_request_s::REQUEST_LEVEL;
            worker.feedback_owner = sensor_calibration_request_s::FEEDBACK_AUTO;
            worker.parameter_set_count = request.parameter_set_count;
            if (sensor_calibration_request_publication_.publish(worker)) {
                vehicle_status_.calibration_enabled = true;
                sensor_calibration_dispatch_time_ = now;
                auto_level_request_timestamp_ = request.timestamp;
                changed = true;
            }
        } else if (request.request == auto_calibration_request_s::REQUEST_CANCEL_LEVEL &&
                   auto_level_request_timestamp_ != 0U && sensor_calibration_status_.active &&
                   sensor_calibration_status_.request_timestamp == auto_level_request_timestamp_) {
            sensor_calibration_request_s worker{};
            worker.timestamp = now;
            worker.request = sensor_calibration_request_s::REQUEST_CANCEL;
            worker.feedback_owner = sensor_calibration_request_s::FEEDBACK_AUTO;
            (void)sensor_calibration_request_publication_.publish(worker);
        }
    }
    return changed;
}

bool Commander::resume_auto_calibration(std::uint64_t now) noexcept
{
    const auto request = pending_calibration_arm_;
    pending_calibration_arm_ = {};
    if (request.session_id == 0U || request.session_id != authorized_calibration_session_ ||
        request.timestamp == 0U || request.timestamp > now || now - request.timestamp > 200000ULL ||
        request.session_id != auto_calibration_status_.session_id ||
        vehicle_status_.nav_state != vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 ||
        !auto_calibration_fresh(now) || !auto_calibration_status_.active ||
        !auto_calibration_status_.awaiting_arm || actuator_armed_.armed) return false;
    if (param_set_count() != request.parameter_set_count) return false;
    // 和首次人工 Arm 完全相同的 preflight/Flash 原子门，不支持 force Arm。
    return arm(vehicle_status_s::ARM_DISARM_REASON_COMMAND_INTERNAL, now, true) == TransitionResult::Changed;
}

bool Commander::evaluate_auto_calibration(std::uint64_t now) noexcept
{
    // 进入/Arm 的跨队列投影给一个固定短窗口；物理输出仍由下游完整同拍快照
    // 与请求 TTL 门控。窗口结束后协调器丢失或否定许可立即 Disarm。
    const bool entering = now >= vehicle_status_.nav_state_timestamp && now - vehicle_status_.nav_state_timestamp <= 250000ULL;
    const bool arming = actuator_armed_.armed && now >= vehicle_status_.armed_time && now - vehicle_status_.armed_time <= 250000ULL;
    if (entering || arming) return false;
    const bool healthy = auto_calibration_fresh(now) && auto_calibration_status_.active &&
        auto_calibration_status_.result == auto_calibration_status_s::RESULT_RUNNING &&
        (!actuator_armed_.armed || auto_calibration_status_.motion_allowed);
    if (healthy) return false;
    return disarm(vehicle_status_s::ARM_DISARM_REASON_FAILURE_DETECTOR, now) == TransitionResult::Changed;
}

} // namespace dima::modules::safety
