#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include "control/RoverDifferential.hpp"      // 前端确认需完整类型（事务机路由）
#include "magnetometer/VehicleMagnetometer.hpp"
#include "logging/logging.hpp"

namespace dima::rover::modes {

// —— 统一事务机（从 AutoCalibrationMode.cpp 原样迁入）———————————————————
// apply_transaction/poll_active_transaction/transaction_settled/
// transaction_frontend_confirmed/abort_rollback_complete/handle_group_abort/
// on_transaction_provisional/on_transaction_finished 的推进逻辑、kind 路由与
// 确认判据不变；合同正文见 Dima/rover/modes/README.md。

bool AutoCalibrationMode::apply_transaction(TransactionKind kind, std::uint64_t now,
    bool provisional, std::uint32_t stages) noexcept
{
    // 候选已准备好后，先登记身份再写入：apply可能部分失败并进入Rollback，
    // 不能等它返回成功才登记，否则回滚会按旧kind确认错误的前端。
    session_.kind = kind;
    session_.transaction_stages = stages; // 仅Dynamics需要区分减速度与速度/转向模型。
    const bool applied = transaction_.phase() == CalibrationParameters::Phase::Provisional
        ? transaction_.apply_revisions(now, expected_set_count_, provisional)
        : transaction_.apply(now, expected_set_count_, provisional);
    // 未取得维护锁且尚未写入时不留下虚假活动身份；已进入回滚/故障则保留它。
    if (!applied && !transaction_.active()) session_.kind = TransactionKind::None;
    return applied;
}

void AutoCalibrationMode::poll_active_transaction(std::uint64_t now) noexcept
{
    // 直接读取事务对象的 Phase，不再维护调度侧的第二份状态镜像。
    // 公共 Applying/Provisional/Rollback/
    // Done/Failed/Fault 推进在此统一完成（事务只在 RAM 收尾，无 Flash 写）；
    // kind 差异只体现为三个钩子——
    // transaction_frontend_confirmed()（前端确认路由）、transaction_settled()
    // （确认判据与稳定窗）、on_transaction_provisional()/on_transaction_finished()
    // （Provisional/终态差异化收尾）。Level 与会话级保存/回滚不经此机。
    // 唯一调用点已排除None/Level；不再维护第二份事务种类白名单。
    const bool applied = transaction_frontend_confirmed(now);
    transaction_.poll(applied, transaction_settled(now, applied), now);
    switch (transaction_.phase()) {
    case CalibrationParameters::Phase::Provisional:
        expected_set_count_ = transaction_.set_count_snapshot();
        on_transaction_provisional(now, applied);
        break;
    case CalibrationParameters::Phase::Done:
    case CalibrationParameters::Phase::Failed:
    case CalibrationParameters::Phase::Fault:
        expected_set_count_ = transaction_.set_count_snapshot();
        on_transaction_finished(now);
        break;
    default:   // Applying/Rollback 由 poll 推进；Idle 无完成事件
        break;
    }
}

bool AutoCalibrationMode::transaction_settled(std::uint64_t now, const bool applied) noexcept
{
    // RTK/磁偏置保留融合/残差观测窗；其余参数确认不额外等待，IMU另有残差窗。
    const bool rollback = transaction_.rolling_back();
    bool live{};
    switch (session_.kind) {
    case TransactionKind::Rtk:
        live = applied && (rollback || (rtk_yaw_fused(now) &&
            yaw_aid_sub_.get().time_last_fuse > transaction_.applied_at()));
        break;
    case TransactionKind::MagneticMotor:
    case TransactionKind::Dynamics:
    case TransactionKind::Gains:
    case TransactionKind::Runtime:
        // 这些事务只确认消费者已使用同代同值；补偿系数空等2秒无新证据。
        // Runtime后必经IDENTIFICATION.WaitArm，由那里统一确认停车、融合与
        // 围栏后才放行；事务和准备函数不抢先以瞬时未就绪否决整组。
        // Restore只恢复参数，不启动试验，无需绕过一套额外的运动稳定窗。
        return applied;
    case TransactionKind::Imu:
        return rollback || (maintenance_ready() && applied &&
            (imu_bias_finalizing_ ? imu_bias_residual_valid(now) : imu_quality(now)));
    case TransactionKind::Magnetic:
        live = applied && (rollback || mag_residual(now) < 0.08F);
        break;
    default:
        return false;
    }
    if (live) { if (stable_since_ == 0U) stable_since_ = now; } else stable_since_ = 0U;
    return stable_since_ != 0U && now - stable_since_ >= 2000000ULL;
}

bool AutoCalibrationMode::transaction_frontend_confirmed(std::uint64_t now) const noexcept
{
    if (!transaction_.generation_valid()) return false;
    // 前端确认按 kind/stages 路由，不再读 status_.state。
    if (session_.kind == TransactionKind::Dynamics) {
        if ((session_.transaction_stages & Status::STAGE_DECELERATION) != 0U &&
            (session_.transaction_stages & Status::STAGE_SPEED) == 0U)
            return drive_.calibration_deceleration_applied(transaction_.generation(),
                transaction_.expected_float(dima::params::RO_DECEL_LIM));
        return drive_.calibration_parameters_applied(transaction_.generation(),
            transaction_.expected_float(dima::params::RO_MAX_THR_SPEED),
            transaction_.expected_float(dima::params::RO_YAW_RATE_CORR));
    }
    if (session_.kind == TransactionKind::Imu) return imu_bias_confirmed(now);
    if (session_.kind == TransactionKind::Runtime) return runtime_frontend_confirmed();
    if (session_.kind == TransactionKind::Gains)
        return gain_frontend_confirmed();
    if (session_.kind == TransactionKind::Rtk) {
        const auto &rtk = rtk_sub_.get();
        const float offset = dima::lib::rover::calibration::wrap_pi(
            transaction_.expected_float(dima::params::GPS_YAW_OFFSET) * kRadians);
        return rtk.parameter_update_instance == transaction_.generation() &&
            fresh(rtk.timestamp_sample, now, 300000ULL) && rtk.timestamp_sample > transaction_.applied_at() &&
            rtk.configured_baseline_m == transaction_.expected_float(dima::params::GPS_YAW_BASELINE) &&
            std::fabs(dima::lib::rover::calibration::wrap_pi(rtk.configured_yaw_offset_rad - offset)) < 1.0e-6F;
    }
    // 磁基础校正与其补偿失效/回滚同代确认；不能只匹配 offset 就释放保存锁。
    const auto &mag = mag_sub_.get();
    const float coefficient[3]{transaction_.expected_float(dima::params::CAL_MAG_MOT_KX),
        transaction_.expected_float(dima::params::CAL_MAG_MOT_KY),
        transaction_.expected_float(dima::params::CAL_MAG_MOT_KZ)};
    const bool compensation_applied = mag_frontend_.throttle_compensation_matches(transaction_.generation(),
        transaction_.expected_int(dima::params::CAL_MAG_MOT_ID),
        transaction_.expected_int(dima::params::CAL_MAG_MOT_GEN), coefficient) &&
        fresh(mag.timestamp_sample, now, 300000ULL) &&
        mag.timestamp_sample > transaction_.applied_at();
    if (session_.kind == TransactionKind::MagneticMotor) return compensation_applied;
    const std::int32_t configured_id = config_.mag_id; // 六面校准已绑定设备；本轮只改 offset。
    const float expected[6]{transaction_.expected_float(dima::params::CAL_MAG0_XOFF),
        transaction_.expected_float(dima::params::CAL_MAG0_YOFF),
        transaction_.expected_float(dima::params::CAL_MAG0_ZOFF),
        config_.mag_scale[0], config_.mag_scale[1], config_.mag_scale[2]};
    return compensation_applied &&
        mag_frontend_.mag_calibration_matches(transaction_.generation(), configured_id, expected);
}

bool AutoCalibrationMode::abort_rollback_complete(std::uint64_t now) noexcept
{
    // 组失败后的 provisional 回滚：取消并在停波/前端确认合同内推进。
    if (transaction_.active()) {
        transaction_.cancel(now);
        transaction_.poll(transaction_frontend_confirmed(now), true, now);
        if (transaction_.active()) return false;
    }
    if (transaction_.phase() == CalibrationParameters::Phase::Failed) {
        // 回滚写入也是本会话自有修改，先同步计数再进入下一组或 FINALIZE。
        expected_set_count_ = transaction_.set_count_snapshot();
        if (runtime_cohort_) {
            // Runtime/PI/导航共用一份事务，整组恢复后此前局部验证证据全部失效。
            const auto cohort = Status::STAGE_RUNTIME | Status::STAGE_INNER_GAINS |
                Status::STAGE_HEADING_GAIN | Status::STAGE_NAV_STRATEGY | Status::STAGE_PATH_GAIN;
            status_.provisional_validated_stages &= ~cohort;
            status_.unavailable_stages |= cohort;
            runtime_cohort_ = false;
            inner_speed_ok_ = false;
            status_.gains_provisional = false;
        }
    }
    return true;
}

void AutoCalibrationMode::handle_group_abort(std::uint64_t now) noexcept
{
    // 组级机动失败处置：回滚确认后由组调度器从失败行继续向后找独立组；
    // 无可独立继续的组时进入统一 FINALIZE/终态（保存/回滚决策按原因）。
    if (!abort_rollback_complete(now)) return;
    // 唯一调用方已确认abort_reason非NONE；原因本身就是待处理标记。
    const std::uint8_t reason = session_.abort_reason;
    const auto group_bit = session_.abort_group;
    session_.abort_reason = Status::FAILURE_NONE;
    session_.abort_group = 0U;
    if (advance_group_scheduler(group_row(group_bit), now)) return;
    terminate(reason, false, now);
}

void AutoCalibrationMode::on_transaction_provisional(std::uint64_t now, const bool applied) noexcept
{
    // Provisional 确认后的 kind 差异化推进（公共标记与代次快照已由事务机完成）。
    switch (session_.kind) {
    case TransactionKind::Gains: {
        if (status_.gain_group == Status::GAIN_PATH && nav_.active && nav_.pass == NavigationPass::Restore) {
            status_.navigation_observed_fields |= nav_.evidence;
            if ((nav_.evidence & Status::NAV_FIELD_LOOKAHEAD) != 0U)
                status_.provisional_validated_stages |= Status::STAGE_PATH_GAIN;
            else status_.unavailable_stages |= Status::STAGE_PATH_GAIN;
            const auto required = Status::NAV_FIELD_JERK | Status::NAV_FIELD_SPEED_REDUCTION | Status::NAV_FIELD_TRANSITIONS;
            if ((status_.navigation_observed_fields & required) == required)
                status_.provisional_validated_stages |= Status::STAGE_NAV_STRATEGY;
            else status_.unavailable_stages |= Status::STAGE_NAV_STRATEGY;
            PX4_INFO_RAW("[autocal] navigation tuning confirmed fields=0x%lx; unconfirmed values restored\n",
                static_cast<unsigned long>(nav_.evidence));
            nav_.active = false;
            status_.navigation_tuning_item = Status::NAV_TUNE_NONE;
            if (!transaction_.finalize_provisional(now, expected_set_count_))
                terminate(Status::FAILURE_PARAMETER, false, now);
            break;
        }
        status_.gains_provisional = true;
        status_.closed_loop = true;
        // 每个新候选从本组起点验证；验证失败由原组失败链停车/回滚。
        exercise_ = 0U;
        // INNER/HEADING 的验证属于 VALIDATION 阶段；PATH/NAVIGATION 属于
        // NAVIGATION 阶段。从 IDENTIFICATION 确认 INNER 时在这里切换阶段。
        const std::uint8_t target = (status_.gain_group == Status::GAIN_NAVIGATION ||
            status_.gain_group == Status::GAIN_PATH)
            ? Status::STATE_NAVIGATION : Status::STATE_VALIDATION;
        if (status_.state != target) enter_phase(target, now, PhaseSubstate::WaitArm);
        else enter_substate(PhaseSubstate::WaitArm, now);
        break;
    }
    case TransactionKind::Imu:
        // 残差重锁窗：Provisional 残差 3 s 稳定才 finalize；30 s 截止取消回滚。
        if (imu_relock_started_ == 0U) imu_relock_started_ = now;
        // 本拍前端确认已由事务入口取得，残差检查不再重复读取同一组参数。
        if (applied && imu_bias_residual_valid(now)) { if (stable_since_ == 0U) stable_since_ = now; }
        else stable_since_ = 0U;
        if (stable_since_ != 0U && now - stable_since_ >= 3000000ULL) {
            if (!transaction_.finalize_provisional(now, expected_set_count_)) transaction_.cancel(now);
            else imu_bias_finalizing_ = true;
            imu_relock_started_ = now;
        } else if (now - imu_relock_started_ > 30000000ULL) {
            transaction_.cancel(now);
            imu_relock_started_ = now;
        }
        break;
    case TransactionKind::Magnetic:
        // bootstrap 只应用 RAM 并保持 provisional：2 s 稳定确认后进入第二轮
        // TURN 学残差；已学习时本周期由磁决策表（refine/restore）接管。
        if (!bootstrap_applied_ && !transaction_.rolling_back() && applied &&
            stable_since_ != 0U && now - stable_since_ >= 2000000ULL) {
            bootstrap_applied_ = true;
            for (unsigned axis = 0U; axis < 3U; ++axis) bootstrap_offset_[axis] = candidate_mag_[axis];
            session_.turn_round = 2U;
            enter_phase(Status::STATE_TURN, now, PhaseSubstate::WaitArm);
        }
        break;
    case TransactionKind::Runtime:
        status_.gains_provisional = true;
        // 运行候选已确认，在此完成辨识准备并直接等待运动就绪。
        if (begin_identification()) {
            enter_phase(Status::STATE_IDENTIFICATION, now, PhaseSubstate::WaitArm);
        } else {
            fail_tuning(take_helper_failure(Status::FAILURE_PROFILE_UNOBSERVABLE), now);
        }
        break;
    default:
        break;   // Rtk/Dynamics：停波静态事务不经过 Provisional（RAM-only 收尾）。
    }
}

void AutoCalibrationMode::on_transaction_finished(std::uint64_t now) noexcept
{
    // Done/Failed/Fault统一在此收尾；阶段调度器不再重复读取终态执行第二套动作。
    if (transaction_.phase() == CalibrationParameters::Phase::Failed)
        (void)abort_rollback_complete(now);
    switch (session_.kind) {
    case TransactionKind::Rtk:
    case TransactionKind::Dynamics: {
        const bool rtk = session_.kind == TransactionKind::Rtk;
        const bool deceleration = (session_.transaction_stages & Status::STAGE_DECELERATION) != 0U;
        session_.kind = TransactionKind::None;
        if (transaction_.phase() != CalibrationParameters::Phase::Done) {
            // frontend confirmation 的确认者按 kind 路由（Rtk=接收机回读、
            // Dynamics=驱动控制器回显）。失败时打出事务相位与代次有效性，
            // 与 RoverDifferential 的应用门限频打印配合定位具体卡点。
            PX4_WARN("[autocal] %s transaction unconfirmed: phase=%d generation_valid=%u",
                rtk ? "rtk" : "dynamics",
                static_cast<int>(transaction_.phase()),
                transaction_.generation_valid() ? 1U : 0U);
            terminate(Status::FAILURE_FRONTEND_CONFIRMATION, false, now);
        } else if (rtk) {
            status_.provisional_validated_stages |= Status::STAGE_RTK;
            status_.progress = 55U;
            // 基础制动/RTK返场后启动独立动态辨识；不再维护三档覆盖或静态斜率统计。
            exercise_running_ = false;
            // RTK事务结束时会话已获授权；预算从实验入口开始，包含首次就绪等待。
            exercise_started_ = now;
            last_fit_velocity_epoch_ = 0U;
            session_.stop_intent = StopIntent::None;
            status_.maximum_speed_m_s = 0.0F;
            PX4_INFO_RAW("[autocal] speed model begin: independent ramp/response identification\n");
            enter_phase(Status::STATE_STRAIGHT, now, PhaseSubstate::WaitArm);
        } else if (deceleration) {
            if ((status_.provisional_validated_stages & Status::STAGE_DECELERATION) == 0U) {
                status_.provisional_validated_stages |= Status::STAGE_DECELERATION;
                PX4_INFO("[autocal] braking deceleration validated in RAM: %.3fm/s2; resume straight runs",
                    static_cast<double>(status_.braking_deceleration_m_s2));
            }
            session_.stop_intent = StopIntent::Braking;
            enter_substate(PhaseSubstate::WaitStop, now);
        } else {
            if (status_.failure_reason == Status::FAILURE_MECHANICAL_ASYMMETRY)
                status_.failure_reason = Status::FAILURE_NONE;
            status_.provisional_validated_stages |= Status::STAGE_SPEED | Status::STAGE_YAW;
            status_.progress = 80U;
            enter_phase(Status::STATE_MAGNETIC, now, PhaseSubstate::Evaluate);
        }
        break;
    }
    case TransactionKind::Gains:
        if (transaction_.phase() == CalibrationParameters::Phase::Done) {
            status_.gains_provisional = false;
            session_.kind = TransactionKind::None;
            after_gain_validated(now);
        } else if (transaction_.phase() == CalibrationParameters::Phase::Failed) {
            status_.gains_provisional = false;
            session_.kind = TransactionKind::None;
            status_.unavailable_stages |=
                (Status::STAGE_INNER_GAINS | Status::STAGE_HEADING_GAIN | Status::STAGE_PATH_GAIN) &
                ~status_.provisional_validated_stages;
            if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = Status::FAILURE_GAIN_VALIDATION;
            enter_finalize(now, false);
        } else terminate(Status::FAILURE_PARAMETER, false, now);
        break;
    case TransactionKind::Imu:
        if (transaction_.phase() == CalibrationParameters::Phase::Fault) {
            terminate(Status::FAILURE_PARAMETER, false, now);
            break;
        }
        if (transaction_.phase() == CalibrationParameters::Phase::Done)
            status_.provisional_validated_stages |= Status::STAGE_IMU_BIAS;
        else status_.unavailable_stages |= Status::STAGE_IMU_BIAS;
        // 新 IMU 校准代重新建立后才启动响应采集；不重设全球圆心，不叠加旧 bias。
        session_.kind = TransactionKind::None;
        continue_profile_entry(now);
        break;
    case TransactionKind::Magnetic:
    case TransactionKind::MagneticMotor:
        if (transaction_.phase() == CalibrationParameters::Phase::Done) {
            if (session_.kind == TransactionKind::MagneticMotor) {
                status_.provisional_validated_stages |= Status::STAGE_MAG_MOT;
                status_.unavailable_stages &= ~Status::STAGE_MAG_MOT;
                session_.kind = TransactionKind::None;
                PX4_INFO("[autocal] mag throttle compensation validated in RAM; save pending");
                start_profile_chain(now);
            } else {
                bootstrap_applied_ = false;
                status_.provisional_validated_stages |= Status::STAGE_MAG;
                session_.kind = TransactionKind::None;
                // 基础磁校准 RAM 确认后才启动独立补偿事务；不可观测明确保留未完成位。
                if (!commit_mag_throttle(now)) start_profile_chain(now);
            }
            break;
        }
        if (transaction_.phase() == CalibrationParameters::Phase::Failed) {
            if (session_.kind == TransactionKind::MagneticMotor) {
                // 补偿组已确认回滚，基础磁校准仍有效；记录未完成并继续其他整定。
                status_.unavailable_stages |= Status::STAGE_MAG_MOT;
                session_.kind = TransactionKind::None;
                PX4_WARN("[autocal] mag throttle commit rolled back; compensation incomplete");
                start_profile_chain(now);
                break;
            }
            session_.kind = TransactionKind::None;
            if (bootstrap_applied_) {
                bootstrap_applied_ = false;
                start_profile_chain(now);
                break;
            }
            terminate(Status::FAILURE_FRONTEND_CONFIRMATION, false, now);
            break;
        }
        terminate(Status::FAILURE_PARAMETER, false, now);
        break;
    case TransactionKind::Runtime:
        if (transaction_.phase() == CalibrationParameters::Phase::Failed) {
            status_.gains_provisional = false;
            session_.kind = TransactionKind::None;
            fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
        } else if (transaction_.phase() == CalibrationParameters::Phase::Fault) {
            terminate(Status::FAILURE_PARAMETER, false, now);
        }
        // Committed 不可达：Runtime cohort 保持 provisional，终态保存统一进 FINALIZE。
        break;
    default:
        break;
    }
}

} // namespace dima::rover::modes
