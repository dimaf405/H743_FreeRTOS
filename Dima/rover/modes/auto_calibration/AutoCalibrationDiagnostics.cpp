#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include "logging/logging.hpp"
#include "format/Format.hpp"
#include <uORB/topics/auto_calibration_status_labels.hpp>

#include <algorithm>
#include <cstring>
#include <limits>

namespace dima::rover::modes {

void AutoCalibrationMode::capture_motion_failure(const char *cause, std::uint64_t now) noexcept
{
    // 只在失败确认点复制证据，不新增周期采样；回滚、停转或传感器恢复后仍
    // 报同一现场。保持既有判据，三个短行可由现有 STATUSTEXT 传输。
    if (motion_failure_text_[0][0] != '\0') return;
    const auto &rtk = rtk_sub_.get();
    const auto &gps = gps_sub_.get();
    const bool rotating = session_.substate == PhaseSubstate::TurnAround ||
        status_.state == Status::STATE_TURN ||
        (session_.substate == PhaseSubstate::Return && return_motion_ == ReturnMotion::Align);
    const auto age_ms = [now](std::uint64_t timestamp) {
        // -1=无样本/未来时间；9999=至少约10秒，防止失鲜时间撑满日志正文。
        return timestamp != 0U && timestamp <= now
            ? static_cast<double>(std::min<std::uint64_t>((now - timestamp) / 1000ULL, 9999ULL)) : -1.0;
    };
    dima::format::format_to(motion_failure_text_[0], sizeof(motion_failure_text_[0]),
        "[autocal] fault=%s state=%s turn=%u yaw=%.3f target=%.3f input=%.3f",
        cause, dima::generated::uorb_labels::auto_calibration_status_state_name(status_.state),
        rotating ? 1U : 0U, static_cast<double>(yaw_rate()),
        static_cast<double>(turn_rate_target_), static_cast<double>(steering_));
    dima::format::format_to(motion_failure_text_[1], sizeof(motion_failure_text_[1]),
        "[autocal] quality imu=%u rtk=%u fused=%d base=%.3f/%.3f hacc=%.2fdeg eph=%.3fm",
        imu_quality(now) ? 1U : 0U, rtk_quality(now) ? 1U : 0U,
        (status_.provisional_validated_stages & Status::STAGE_RTK) != 0U ? (rtk_yaw_fused(now) ? 1 : 0) : -1,
        static_cast<double>(rtk.baseline_m), static_cast<double>(status_.rtk_baseline_m),
        static_cast<double>(rtk.heading_accuracy_rad / kRadians), static_cast<double>(gps.eph));
    // 到点复核复用首因快照槽位，保存判定时的实测距离/容差/速度，退出后不重算。
    if (std::strcmp(cause, "arrival") == 0) {
        dima::format::format_to(motion_failure_text_[1], sizeof(motion_failure_text_[1]),
            "[autocal] return motion=%u distance=%.3f limit=%.3f speed=%.3f",
            static_cast<unsigned>(return_motion_), static_cast<double>(distance_to_start()),
            static_cast<double>(0.5F + fence_.origin_error_m + gps.eph), static_cast<double>(ground_speed()));
    }
    dima::format::format_to(motion_failure_text_[2], sizeof(motion_failure_text_[2]),
        "[autocal] RTK fix=%u sol=%u int=%u vel=%u gps/heading_age=%.0f/%.0fms",
        static_cast<unsigned>(gps.fix_type), rtk.solution_computed ? 1U : 0U,
        rtk.integer_fixed ? 1U : 0U, rtk.velocity_aligned ? 1U : 0U,
        age_ms(gps.timestamp_sample), age_ms(rtk.timestamp_sample));
}

void AutoCalibrationMode::report_motion_failure() const noexcept
{
    for (const auto &line : motion_failure_text_)
        if (line[0] != '\0') px4_log_raw(_PX4_LOG_LEVEL_WARN, "%s\n", line);
}

bool AutoCalibrationMode::publish(std::uint64_t now) noexcept
{
    status_.arming_allowed = status_.active && !pending_termination_ && status_.state != Status::STATE_FINALIZE && status_.result == Status::RESULT_RUNNING;
    status_.motion_inhibited = armed_.calibration_output_inhibited();
    status_.timestamp = now;
    status_.timestamp_sample = now;
    status_.sequence = ++sequence_;
    status_.mag_bootstrap_active = bootstrap_applied_;
    status_.ready_for_entry = !selected_ && !status_.active && !transaction_.active() &&
        !level_sub_.get().active && module_state_ == dima::middleware::lifecycle::ModuleState::Running;
    // motion_allowed 表示当前阶段确实要发布运动请求，不在发布函数中再次
    // 把 RTK/IMU/围栏瞬时判定变成无效帧。实验条件失败由 Run() 的结果检查
    // 统一进入明确终态，避免模式层周期性清零电机。
    status_.motion_allowed = status_.active && !pending_termination_ && selected_ && is_motion_state() &&
        armed_.armed() && !status_.motion_inhibited && module_state_ == dima::middleware::lifecycle::ModuleState::Running;
    rover_motion_request_s motion{};
    motion.timestamp = now;
    // 开环请求本身不依赖 IMU sample；闭环才把估计器样本传给控制器。
    motion.timestamp_sample = status_.closed_loop ? imu_sub_.get().timestamp_sample : now;
    motion.sequence = sequence_;
    motion.source = rover_motion_request_s::SOURCE_CALIBRATION;
    motion.valid = status_.motion_allowed;
    const bool heading_goal = motion.valid && session_.substate == PhaseSubstate::TurnAround;
    motion.mode = heading_goal ? rover_motion_request_s::MODE_HEADING_TARGET :
        status_.closed_loop ? rover_motion_request_s::MODE_SPEED_YAW_RATE : rover_motion_request_s::MODE_NORMALIZED_AXES;
    if (heading_goal) {
        // 每拍只重发同一目标和标识；控制侧独占转角累计、输入生成、撤力及完成判定。
        motion.heading_request_timestamp = session_.substate_started;
        motion.heading_target_rad = turn_heading_;
        motion.heading_direction = 1;
    }
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    motion.speed_m_s = motion.valid && !heading_goal && status_.closed_loop ? physical_speed_ : unavailable;
    motion.yaw_rate_rad_s = motion.valid && !heading_goal && status_.closed_loop ? physical_rate_ : unavailable;
    motion.normalized_longitudinal = motion.valid && !heading_goal && !status_.closed_loop ? longitudinal_ : unavailable;
    motion.normalized_steering = motion.valid && !heading_goal && !status_.closed_loop ? steering_ : unavailable;
    // 每拍只发布一次完整状态；请求紧随同一快照，Commander不会读到上一会话状态。
    if (!status_pub_.publish(status_) || !motion_pub_.publish(motion)) return false;
    if (request_pending_ == 0U) return true;
    auto_calibration_request_s request{};
    request.timestamp = request_pending_ == auto_calibration_request_s::REQUEST_LEVEL ? level_request_time_ : now;
    request.session_id = status_.session_id;
    request.sequence = sequence_;
    request.parameter_set_count = expected_set_count_;
    request.request = request_pending_;
    if (!request_pub_.publish(request)) return false;
    request_pending_ = 0U;
    return true;
}

void AutoCalibrationMode::report_status(std::uint64_t now) noexcept
{
    // 退出 External1 或会话结束后停止周期文本；终态仍保留在状态 Topic，
    // finish() 单次报告结果。回滚仍由调度器继续收尾，停止日志不等于解除
    // 停波或参数事务互锁。
    if (!selected_ || !status_.active || status_.session_id == 0U ||
        (last_report_ != 0U && now >= last_report_ && now - last_report_ < 5000000ULL)) return;
    last_report_ = now;
    using namespace dima::generated::uorb_labels;
    if (pending_termination_ || status_.result != Status::RESULT_RUNNING) {
        // 失败收尾优先重报首因，不让十余条运行快照挤掉关键文本；保持原5秒
        // 周期，不把等待停波/保存期间的 RUNNING 描述成仍在执行实验。
        px4_log_raw(_PX4_LOG_LEVEL_WARN, "[autocal] session=%lu ended: %s; Arm inhibited\n",
            static_cast<unsigned long>(status_.session_id), auto_calibration_status_failure_name(status_.failure_reason));
        report_motion_failure();
        if (transaction_.phase() == CalibrationParameters::Phase::Fault)
            px4_log_raw(_PX4_LOG_LEVEL_ERROR, "[autocal] rollback unconfirmed; interlock latched; reset required\n");
        return;
    }
    const bool braking_substate = session_.substate == PhaseSubstate::Braking;
    // 周期快照走非实时 RAW 路径，不受普通日志级别过滤。
    PX4_INFO_RAW("[autocal] session=%lu state=%s progress=%u%%\n",
        static_cast<unsigned long>(status_.session_id), auto_calibration_status_state_name(status_.state),
        static_cast<unsigned>(status_.progress));
    PX4_INFO_RAW("[autocal] stages saved=0x%lx unavailable=0x%lx skipped=0x%lx\n",
        static_cast<unsigned long>(status_.completed_stages), static_cast<unsigned long>(status_.unavailable_stages),
        static_cast<unsigned long>(status_.skipped_stages));
    PX4_INFO_RAW("[autocal] fence %s %s %.2f/%.2fm margin %.2fm brake model %.2fm\n",
        status_.straight_active ? "straight" : "circle",
        auto_calibration_status_fence_name(status_.fence_state), static_cast<double>(status_.fence_distance_m),
        static_cast<double>(status_.straight_active ? status_.straight_distance_m : status_.fence_radius_m), static_cast<double>(status_.fence_margin_m),
        static_cast<double>(status_.fence_stop_distance_m));
    PX4_INFO_RAW("[autocal] speed cap %.2fm/s motor cap %.2f\n",
        static_cast<double>(status_.session_speed_limit_m_s), static_cast<double>(status_.session_motor_limit));
    PX4_INFO_RAW("[autocal] braking observations=%u decel=%.3fm/s2 distance=%.2fm time=%.2fs ram_model=%u\n",
        static_cast<unsigned>(status_.braking_observations), static_cast<double>(status_.braking_deceleration_m_s2),
        static_cast<double>(status_.braking_stop_distance_m), static_cast<double>(status_.braking_stop_time_s),
        braking_model_ready() ? 1U : 0U);
    if (status_.braking_full_speed_m_s > 0.0F)
        PX4_INFO_RAW("[autocal] measured full-output ground speed %.2fm/s\n", static_cast<double>(status_.braking_full_speed_m_s));
    if (status_.active && !pending_termination_ && braking_substate)
        report_braking_probe(now);
    if (status_.active && !pending_termination_ && status_.excitation_phase != Status::EXCITATION_NONE)
        PX4_INFO_RAW("[autocal] excitation=%s level=%u target=%.3f samples=%lu remaining=%.1fs\n",
            auto_calibration_status_excitation_name(status_.excitation_phase), static_cast<unsigned>(status_.excitation_level),
            static_cast<double>(status_.excitation_target), static_cast<unsigned long>(status_.excitation_samples),
            static_cast<double>(status_.excitation_remaining_s));
    // 周期快照可补齐重连后缺失的一次性证据；不重复发 endpoint reached 事件。
    PX4_INFO_RAW("[autocal] evidence endpoint=%u mag_pct=%.1f mag_comp_saved=%u mag_present=%u\n",
        endpoint_reported_ ? 1U : 0U, static_cast<double>(status_.mag_interference_pct),
        (status_.completed_stages & Status::STAGE_MAG_MOT) != 0U ? 1U : 0U, status_.magnetometer_present ? 1U : 0U);
    if (status_.active && !pending_termination_) {
        // 提示不再等待 RTK 完成；Armed 和运动许可分别报告，避免用户误以为拒绝解锁。
        PX4_INFO_RAW(!status_.session_authorized
            ? "[autocal] Arm ONCE now; motion waits for readiness\n"
            : "[autocal] session authorized; Manual/Disarm cancels\n");
        if (armed_.calibration_output_inhibited())
            PX4_INFO_RAW("[autocal] motor output inhibited; collecting/committing\n");
    }
    if (status_.mag_bootstrap_active || status_.gains_provisional)
        px4_log_raw(_PX4_LOG_LEVEL_WARN, "[autocal] provisional RAM values; NOT saved\n");
    if (status_.provisional_validated_stages != 0U)
        PX4_INFO_RAW("[autocal] validated RAM=0x%lx; final save pending\n",
            static_cast<unsigned long>(status_.provisional_validated_stages));
}

} // namespace dima::rover::modes
