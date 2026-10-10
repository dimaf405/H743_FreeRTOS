#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include "logging/logging.hpp"
#include <uORB/topics/auto_calibration_status_labels.hpp>

namespace dima::rover::modes {

// —— FINALIZE 族（从 AutoCalibrationMode.cpp 原样迁入）——————————————————
// step_finalize/enter_finalize/session_failure_requires_rollback/terminate/
// finish 的决策锁、握手链条与预算语义不变；合同正文见
// Dima/rover/modes/README.md。

void AutoCalibrationMode::step_finalize(std::uint64_t now) noexcept
{
    // 候选、取消和失败共用收尾：停波/Disarm → 组回滚 → 按需恢复会话 RAM → 一次保存。
    if (!status_.active) return;
    if (armed_.armed()) return; // 入口EXIT已请求解除Armed，此处只等待Commander确认。
    const auto &output = output_sub_.get();
    if (!maintenance_ready() && (output.state == actuator_output_status_s::STATE_FAULT ||
        !fresh(output.timestamp, now, 750000ULL) || now - session_.substate_started > 15000000ULL)) {
        // 后端明确故障/停波确认失败时不写参数、不恢复输出；沿用停车确认窗并发布锁存失败。
        if (level_sub_.get().active) request_pending_ = auto_calibration_request_s::REQUEST_CANCEL_LEVEL;
        transaction_.fault();
        status_.failure_reason = Status::FAILURE_CONTROL_LOSS;
        finish(now);
        return;
    }
    if (!maintenance_ready()) return;
    if (level_sub_.get().active) {
        request_pending_ = auto_calibration_request_s::REQUEST_CANCEL_LEVEL;
        return;
    }
    const auto &level = level_sub_.get();
    // 失败/取消允许累计一次应用与一次恢复：成功最多3次变化，恢复总计最多6次。
    if (session_.kind == TransactionKind::Level && level.request_timestamp == level_request_time_ &&
        level.parameter_start_count == expected_set_count_ && level.parameter_owned_changes <= 6U &&
        level.parameter_set_count == expected_set_count_ + level.parameter_owned_changes)
        expected_set_count_ = level.parameter_set_count;
    if (transaction_.phase() == CalibrationParameters::Phase::Fault) {
        status_.failure_reason = Status::FAILURE_PARAMETER;
        finish(now); // 故障锁仍保留，不能再授权运动或后台写回。
        return;
    }
    if (!abort_rollback_complete(now)) return;
    const bool rollback = session_.force_rollback || cancel_requested_ ||
        session_failure_requires_rollback(status_.failure_reason);
    session_.kind = TransactionKind::None;
    if (rollback) {
        status_.provisional_validated_stages = 0U;
        status_.gains_provisional = false;
    }
    if (transaction_.session_active()) {
        if (rollback && !transaction_.session_rollback(expected_set_count_))
            status_.failure_reason = Status::FAILURE_PARAMETER;
        else if (!transaction_.session_save(expected_set_count_, !rollback)) {
            if (transaction_.phase() != CalibrationParameters::Phase::Fault) return;
            // 显性报错（2026-09-30 实车教训）：此前落盘失败只在状态枚举里
            // 体现，操作者看不到"成果没存上"。分区满时 SD 卡必须在场，收尾
            // 的直存才会走 镜像→擦除→重建 自愈。
            PX4_WARN("[autocal] final save FAILED: results not persisted (partition full or SD missing)");
            status_.failure_reason = Status::FAILURE_STORAGE;
        }
    }
    finish(now); // active=false，下一调度周期不会再次保存；存储失败保留 Fault 互锁。
}

void AutoCalibrationMode::enter_finalize(std::uint64_t now, bool force_rollback) noexcept
{
    // 统一 FINALIZE 入口：撤销运动许可与等待标志，决策留给 Prepare。
    // EXIT 握手链条（环环相扣，握手期内任何一环都不得切模式/判取消）：
    //   ① 此处发 REQUEST_EXIT（active 仍为 true）→ Commander 只解除 Armed、
    //     不切模式（见 CommanderAutoCalibration.cpp 的 EXIT 分支）；
    //   ② Run() 的 armed 下降沿在 FINALIZE 内豁免（非 failsafe/kill/termination/
    //     lockdown），不按取消/控制链终止；
    //   ③ step_finalize 等 !armed 后执行保存/回滚 → finish()（active=false）再发
    //     终态 EXIT → Commander 解除 Armed 并切回 Manual；
    //   ④ 模式未选中但 active 已为 false，“未选中”不再触发 OPERATOR_CANCEL，
    //     已做的保存决策不会被翻成回滚。
    session_.force_rollback = session_.force_rollback || force_rollback;
    if (status_.state != Status::STATE_FINALIZE) {
        status_.motion_allowed = false;
        status_.awaiting_arm = false;
        status_.session_authorized = false;
        longitudinal_ = steering_ = 0.0F;
        session_.stop_intent = StopIntent::None;
        // FINALIZE直接使用静态WaitStop，内部停波/保存仍由step_finalize唯一推进。
        status_.braking_phase = Status::BRAKING_ACCELERATE;
        enter_phase(Status::STATE_FINALIZE, now, PhaseSubstate::WaitStop);
        if (!pending_termination_) request_pending_ = auto_calibration_request_s::REQUEST_EXIT;
        return;
    }
}

bool AutoCalibrationMode::session_failure_requires_rollback(std::uint8_t reason) const noexcept
{
    switch (reason) {
    case Status::FAILURE_CONTROL_LOSS:
    case Status::FAILURE_SENSOR_STALE:
    case Status::FAILURE_PARAMETER:
    case Status::FAILURE_FRONTEND_CONFIRMATION:
    case Status::FAILURE_STORAGE:
    case Status::FAILURE_FENCE_BOUNDARY:
    case Status::FAILURE_FENCE_SPACE:
    case Status::FAILURE_ESTIMATOR:
    case Status::FAILURE_RTK_INCONSISTENT:
        return true;
    default:
        return false; // 实验组失败允许保存其他已确认组，结果为 PARTIAL。
    }
}

void AutoCalibrationMode::terminate(std::uint8_t reason, bool cancelled, std::uint64_t now) noexcept
{
    if (!status_.active) return;
    if (pending_termination_) {
        // 负向事件只锁存，统一在保存前处理；保留首因也不能忽略后来发生的硬故障。
        cancel_requested_ = cancel_requested_ || cancelled;
        session_.force_rollback = session_.force_rollback || session_failure_requires_rollback(reason);
        if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = reason;
        return;
    }
    (void)armed_.set_calibration_output_inhibited(true);
    pending_termination_ = true;
    cancel_requested_ = cancelled;
    status_.failure_reason = reason;
    status_.motion_allowed = false;
    status_.awaiting_arm = false;
    status_.session_authorized = false;
    if (session_.substate == PhaseSubstate::Braking) report_braking_probe(now);
    longitudinal_ = steering_ = 0.0F;
    if (status_.excitation_phase != Status::EXCITATION_NONE) status_.excitation_phase = Status::EXCITATION_BRAKE;
    status_.excitation_target = status_.excitation_remaining_s = 0.0F;
    request_pending_ = auto_calibration_request_s::REQUEST_EXIT;
    px4_log_raw(_PX4_LOG_LEVEL_WARN, "[autocal] session ending: %s\n",
        dima::generated::uorb_labels::auto_calibration_status_failure_name(reason));
    report_motion_failure();
    // 决策与持久化统一推迟到 FINALIZE：Run() 的终止收尾先取消活动事务/
    // 恢复 bootstrap，再 enter_finalize()。
}

void AutoCalibrationMode::finish(std::uint64_t now) noexcept
{
    // 终态发布（FINALIZE Publish）：保存/回滚已由 step_finalize 决策执行完毕；
    // 这里只按已完成证据与失败原因计算 RESULT，并撤销全部授权与运动许可。
    // completed 的唯一写入边界是最终持久化成功；取消、回滚或存储失败不留完成位。
    status_.completed_stages = transaction_.session_committed()
        ? status_.provisional_validated_stages & ~status_.unavailable_stages & ~status_.skipped_stages : 0U;
    status_.provisional_validated_stages = 0U;
    if (status_.magnetometer_present && (status_.completed_stages & Status::STAGE_MAG_MOT) == 0U)
        status_.unavailable_stages |= Status::STAGE_MAG_MOT;
    if (!status_.magnetometer_present) status_.skipped_stages |= Status::STAGE_MAG | Status::STAGE_MAG_MOT;
    // IMU offset 是可选项，未完成单独报告，不否决整体成功；动力学与运行组仍需本轮证据。
    // 导航组在场地不支持时可明确 skipped，不能用未知状态冒充完成。
    const std::uint32_t core_required = Status::STAGE_LEVEL | Status::STAGE_DECELERATION | Status::STAGE_RTK |
        Status::STAGE_SPEED | Status::STAGE_YAW | Status::STAGE_INNER_GAINS |
        Status::STAGE_RUNTIME |
        (status_.magnetometer_present ? Status::STAGE_MAG | Status::STAGE_MAG_MOT : 0U);
    const std::uint32_t navigation_required = Status::STAGE_HEADING_GAIN | Status::STAGE_PATH_GAIN |
        Status::STAGE_NAV_STRATEGY;
    const std::uint32_t satisfied = status_.completed_stages;
    const std::uint32_t unresolved = status_.unavailable_stages;
    const bool core_complete = (satisfied & core_required) == core_required;
    // 可选导航逐位覆盖：某一项跳过不能使其余未知或失败项也被视为通过。
    const bool navigation_complete = ((satisfied | status_.skipped_stages) & navigation_required) == navigation_required;
    const bool complete = transaction_.session_committed() && core_complete && navigation_complete &&
        (unresolved & (core_required | navigation_required)) == 0U;
    // 回滚、放弃或未保存任何成果都没有完成证据；部分成果须保存成功才报 PARTIAL。
    const bool session_fault = session_failure_requires_rollback(status_.failure_reason);
    // 取消过程中发生硬故障也必须先报FAILED，不能先发布CANCELLED再由Run改写。
    status_.result = session_fault ? Status::RESULT_FAILED
        : cancel_requested_ ? Status::RESULT_CANCELLED
        : complete && status_.failure_reason == 0U ? Status::RESULT_SUCCESS
        : satisfied == 0U ? Status::RESULT_FAILED : Status::RESULT_PARTIAL;
    status_.active = status_.motion_allowed = status_.awaiting_arm = false;
    status_.session_authorized = false;
    status_.excitation_phase = Status::EXCITATION_NONE;
    status_.excitation_target = status_.excitation_remaining_s = 0.0F;
    // 未最终保存的关联证据在回滚/退出后不再代表当前 RAM 配置；终态只展示
    // 已保存阶段，避免“validated RAM / awaiting checks”在会话结束后继续误导。
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
    request_pending_ = auto_calibration_request_s::REQUEST_EXIT;
    PX4_INFO_RAW("[autocal] session=%lu %s: %s\n", static_cast<unsigned long>(status_.session_id),
        dima::generated::uorb_labels::auto_calibration_status_result_name(status_.result),
        dima::generated::uorb_labels::auto_calibration_status_failure_name(status_.failure_reason));
    report_motion_failure();
}

} // namespace dima::rover::modes
