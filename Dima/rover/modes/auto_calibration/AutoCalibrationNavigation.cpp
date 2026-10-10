#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

bool AutoCalibrationMode::prepare_navigation_candidate() noexcept
{
    // 转驱阈值沿用用户配置；共同DrivingStateMachine负责正值/滞回/π域。
    // 不再每次乘0.8、按噪声造阈值或附加0.5/0.75 rad上限。
    return tuning_driving_.configure(tuning_config_.driving);
}

StepResult AutoCalibrationMode::validation_driving(std::uint64_t now) noexcept
{
    const auto &f = control_feedback_sub_.get();
    // state_started_ 在状态压平后不再是阶段时间戳：与其余阶段 helper 一致，
    // 先用子状态起点重锚为本次转驱验收的窗口起点。
    if (state_started_ < session_.substate_started) state_started_ = now;
    // 截止依据：两轮转驱验收最坏 ~80 s（每轮建速稳定 + 一次转驱相位受
    // navigation_phase 的 20 s 截止 + 原位停车确认），60 s 会在合法的第二轮
    // 中途误报超时；放宽到 90 s，仍在 Run() 的 180 s 运动硬窗之内。
    if (now - state_started_ > 90000000ULL ||
        f.parameter_update_instance != transaction_.generation()) {
        physical_speed_ = physical_rate_ = 0.0F;
        if (now - arm_started_ > 250000ULL) {
            return fail_step(Status::FAILURE_GAIN_VALIDATION);
        }
        return StepResult::Busy;
    }
    // 饱和标志只是输出到限，不单独否决转驱。实际建速、停车及状态切换
    // 仍由本实验验证；执行保护继续由公共控制链承担。
    const float speed = 0.5F * tuning_config_.speed_limit;
    const float error = math::wrap_pi(exercise_heading_ - body_yaw());
    if (navigation_phase_ == 2U) {
        physical_speed_ = physical_rate_ = 0.0F;
        if (!stopped()) return StepResult::Busy;
        if (++exercise_ == 2U) {
            validation_passed_ = navigation_directions_ == 3U;
            return StepResult::WantStop;   // 原位停波后由调度器推进 cohort
        }
        navigation_phase_ = 0U;
        navigation_phase_started_ = now;
        exercise_heading_ = body_yaw();
        tuning_driving_.reset(); tuning_heading_.reset();
        stable_since_ = 0U;
        return StepResult::Busy;
    }
    const float state_speed = tuning_driving_.state() == dima::lib::rover::DrivingState::Driving ? speed : 0.0F;
    const auto before = tuning_driving_.state();
    const auto driving = tuning_driving_.update(error, state_speed, f.speed_m_s);
    // 航向设定限速使用实际dt，不把10ms调度按20ms计算成双倍转向变化率。
    const float dt = control_interval_s(now);
    const auto heading = tuning_heading_.update(exercise_heading_, body_yaw(), dt);
    if (!driving.valid || !heading.valid) {
        return fail_step(Status::FAILURE_GAIN_VALIDATION);
    }
    physical_speed_ = driving.translation_enabled ? speed : 0.0F;
    physical_rate_ = driving.heading_control_enabled ? heading.yaw_rate_setpoint_rad_s : 0.0F;
    if (navigation_phase_ == 0U) {
        const bool steady = f.speed_raw_m_s >= 0.8F * speed && std::fabs(error) < 0.5F *
            tuning_config_.driving.turn_to_drive_yaw_error_rad && feedback_unmasked(true);
        if (steady) { if (stable_since_ == 0U) stable_since_ = now; } else stable_since_ = 0U;
        if (stable_since_ != 0U && now - stable_since_ >= 1000000ULL) {
            // 在真实前进中跨过进入阈值；不是在停车航点上假装验证 Driving→Stop。
            const float sign = exercise_ == 0U ? 1.0F : -1.0F;
            exercise_heading_ = math::wrap_pi(body_yaw() + sign * 1.2F * tuning_config_.driving.drive_to_turn_yaw_error_rad);
            navigation_phase_ = 1U;
            navigation_phase_started_ = now;
        }
    } else {
        if (before == dima::lib::rover::DrivingState::Driving &&
            driving.state == dima::lib::rover::DrivingState::StoppingForTurn) {
            if (f.speed_raw_m_s < 0.5F * speed) {
                return fail_step(Status::FAILURE_GAIN_VALIDATION);
            }
            navigation_directions_ |= static_cast<std::uint8_t>(1U << exercise_);
        }
        if (before == dima::lib::rover::DrivingState::StoppingForTurn &&
            driving.state == dima::lib::rover::DrivingState::SpotTurning &&
            std::hypot(f.forward_raw_m_s, f.lateral_raw_m_s) > tuning_config_.speed_threshold) {
            return fail_step(Status::FAILURE_GAIN_VALIDATION);
        }
        if (before == dima::lib::rover::DrivingState::SpotTurning && driving.state == dima::lib::rover::DrivingState::Driving) {
            navigation_phase_ = 2U;
            physical_speed_ = physical_rate_ = 0.0F;
        }
        if (now - navigation_phase_started_ > 20000000ULL) {
            return fail_step(Status::FAILURE_GAIN_VALIDATION);
        }
    }
    return StepResult::Busy;
}

bool AutoCalibrationMode::start_path_validation(std::uint64_t now) noexcept
{
    if (!read_tuning_config() || !std::isfinite(config_.path_error) || config_.path_error <= 0.0F)
        return fail_flag(Status::FAILURE_PARAMETER);
    nav_ = {};
    nav_.active = true;
    nav_.item = Status::NAV_TUNE_LOOKAHEAD;
    // 在既有转驱滞回区间内选择试验角，避免默认5度小角的反解病态放大。
    // 这是实验几何输入，不改公共转驱门限；最终还验证其最大允许转角。
    nav_.turn_angle = 0.5F * (tuning_config_.driving.turn_to_drive_yaw_error_rad +
        tuning_config_.driving.drive_to_turn_yaw_error_rad);
    nav_.original[0] = nav_.accepted[0] = tuning_config_.pursuit.lookahead_gain;
    nav_.original[1] = nav_.accepted[1] = tuning_config_.jerk;
    nav_.original[2] = nav_.accepted[2] = tuning_config_.speed_reduction;
    nav_.maximum_speed = std::min(status_.observed_forward_speed_m_s, fence_.speed_limit_m_s);
    // 试验速度=巡航速度 min(实测前进能力, RO_SPEED_LIM 巡航上限)（2026-09-30 用户
    // 确认）。lookahead/jerk 候选只需恒定速度做公平比较，速度能力由减速比项的
    // [low_speed, maximum_speed] 探测覆盖；原半速是上游推荐工况，对停车距离
    // ~0.1m 的弱动力车无物理必要，且 jerk 差异的可观测性随速度放大(≈v·a/jerk)。
    // 低速车巡航速度落入现有控制死区时仍回退最低实测平台，不改死区。
    nav_.nominal_speed = nav_.maximum_speed;
    nav_.low_speed = nav_.maximum_speed;
    for (unsigned i = 0U; i < math::MotorResponseProfile::kLevels; ++i) {
        const auto *p = response_speed_.plateau(0U, i);
        if (p != nullptr && p->count() != 0U && p->raw_speed.mean() > tuning_config_.speed_threshold)
            nav_.low_speed = std::min(nav_.low_speed, static_cast<float>(p->raw_speed.mean()));
    }
    if (nav_.nominal_speed <= tuning_config_.speed_threshold) nav_.nominal_speed = nav_.low_speed;
    nav_.low_speed = std::min(nav_.low_speed, nav_.nominal_speed);
    if (!std::isfinite(nav_.nominal_speed) || nav_.nominal_speed <= tuning_config_.speed_threshold ||
        !prepare_path(now)) return fail_flag(Status::FAILURE_PATH_UNOBSERVABLE);
    if (nav_.accepted[1] <= 0.0F) {
        const float a = tuning_config_.deceleration, acceleration = tuning_config_.acceleration;
        if (!std::isfinite(a) || a <= 0.0F || !std::isfinite(acceleration) || acceleration <= 0.0F)
            return fail_flag(Status::FAILURE_PARAMETER);
        // 从完整航段扣除建速距离及已验证闭环到90%目标的模型时间v*lambda*ln(10)。
        // 剩余距离才用于PX4停车公式反解；它只是临时种子，绝不是jerk的物理下界。
        const auto seed_at = [&](float v) {
            const float distance = nav_.side - 2.0F * std::min(tuning_config_.acceptance, config_.path_error) -
                v * v / (2.0F * acceleration) - v * loop_time_[0] * std::log(10.0F);
            const float remaining = distance - v * v / (2.0F * a);
            return remaining > 0.0F ? 2.0F * a * v / remaining : NAN;
        };
        // 场地装不下巡航速度的试验几何时明确失败：静默降速采到的增益
        // 不代表实际工况（2026-09-30 用户确认删除该回退）。
        const float seed = seed_at(nav_.nominal_speed);
        if (!std::isfinite(seed) || seed <= 0.0F || seed > 100.0F)
            return fail_flag(Status::FAILURE_FENCE_SPACE);
        nav_.accepted[1] = seed;
    }
    status_.gain_group = Status::GAIN_PATH;
    return start_navigation_item(now);
}

bool AutoCalibrationMode::start_navigation_item(std::uint64_t now) noexcept
{
    nav_.pass = NavigationPass::Baseline;
    nav_.baseline = {}; nav_.best = {}; nav_.result = {};
    nav_.best_arrival = 0.0F;
    nav_.candidate_changed = false;
    nav_.speed_bracketed = false;
    nav_.speed_probe = nav_.item == Status::NAV_TUNE_SPEED_REDUCTION;
    nav_.trial_speed = nav_.nominal_speed;
    nav_.candidate = nav_.accepted[nav_.item - 1U];
    if (nav_.item == Status::NAV_TUNE_LOOKAHEAD) {
        nav_.lower = std::max(0.1F, tuning_config_.pursuit.lookahead_min_m / nav_.trial_speed);
        // 比整个直线段还长的前视圆只会直接指向终点，不提供增益辨识信息。
        const float effective_length = nav_.side - 2.0F * std::min(tuning_config_.acceptance, config_.path_error);
        nav_.upper = std::min(100.0F, std::min(tuning_config_.pursuit.lookahead_max_m, effective_length) / nav_.trial_speed);
    } else if (nav_.item == Status::NAV_TUNE_JERK) {
        // jerk=a/T 的行为区间用实测动力学锚定：T∈[0.1,20]s 覆盖“近瞬时”到
        // “平缓”的加速度过渡（约2.3个数量级）。固定[0.01,100]跨4个数量级，
        // 黄金分割按0.01分辨率要跑满~21次方形路径——2026-09-30 全链仿真：
        // PATH 阶段 146min 中 jerk 项独占 78min，区间与车辆物理无关联是主因。
        const float measured_accel = std::isfinite(tuning_config_.acceleration) ? tuning_config_.acceleration : 0.0F;
        const float measured_decel = std::isfinite(tuning_config_.deceleration) ? tuning_config_.deceleration : 0.0F;
        const float accel_ref = std::max(measured_accel, measured_decel);
        if (accel_ref > 0.0F) {
            nav_.lower = std::max(0.01F, 0.05F * accel_ref);
            nav_.upper = std::min(100.0F, 10.0F * accel_ref);
        } else {
            nav_.lower = std::min(0.01F, nav_.accepted[1]);
            nav_.upper = 100.0F; // 与权威RO_JERK_LIM现有合法上限一致。
        }
        // 既有原值/停车公式种子必须可被搜索：显式配置不被锚定边界吞掉。
        if (std::isfinite(nav_.accepted[1]) && nav_.accepted[1] > 0.0F) {
            nav_.lower = std::min(nav_.lower, nav_.accepted[1]);
            nav_.upper = std::max(nav_.upper, std::min(100.0F, nav_.accepted[1]));
        }
    } else {
        if (tuning_config_.driving.turn_to_drive_yaw_error_rad <= 0.0F ||
            tuning_config_.driving.turn_to_drive_yaw_error_rad > tuning_config_.driving.drive_to_turn_yaw_error_rad)
            return false;
        nav_.lower = std::max(nav_.low_speed, std::nextafter(tuning_config_.speed_threshold, INFINITY));
        nav_.upper = nav_.maximum_speed;
        nav_.candidate = nav_.nominal_speed;
    }
    // 分辨率=相对值：搜索区间收缩到初始区间的 10% 即止（2026-09-30 用户确认
    // 本义——"放宽为相对值"）。绝对步长在宽区间上把黄金分割推满 ~20 次试验；
    // 10% 相对精度对路径行为差异已充分且与车辆/区间宽度无关，速度二分同理。
    const float resolution = 0.10F * (nav_.upper - nav_.lower);
    nav_.search_ready = nav_.search.reset(nav_.lower, nav_.upper, resolution, nav_.speed_probe);
    if (nav_.item == Status::NAV_TUNE_LOOKAHEAD && (!std::isfinite(nav_.candidate) || nav_.candidate < 0.1F || nav_.candidate > 100.0F)) {
        nav_.candidate = 1.0F; nav_.pass = NavigationPass::Reference;
    }
    nav_.best_value = nav_.candidate;
    if (nav_.speed_probe && !prepare_path(now)) return false;
    return apply_navigation_trial(now) == StepResult::Advance;
}

StepResult AutoCalibrationMode::apply_navigation_trial(std::uint64_t now) noexcept
{
    // 停车子状态是唯一调用点；每次从上一已确认配置出发，只修订当前候选。
    // 独立测速临时禁用降速系数，避免把公式自己的输出反拟合成能力。
    status_.lookahead_gain = nav_.accepted[0];
    tuning_config_.jerk = nav_.accepted[1];
    tuning_config_.speed_reduction = nav_.accepted[2];
    nav_.trial_speed = nav_.nominal_speed;
    if (nav_.pass != NavigationPass::Restore) {
        if (nav_.item == Status::NAV_TUNE_LOOKAHEAD) status_.lookahead_gain = nav_.candidate;
        else if (nav_.item == Status::NAV_TUNE_JERK) tuning_config_.jerk = nav_.candidate;
        else if (nav_.speed_probe) {
            tuning_config_.speed_reduction = -1.0F;
            nav_.trial_speed = nav_.candidate;
        } else {
            tuning_config_.speed_reduction = nav_.candidate;
            nav_.trial_speed = nav_.maximum_speed;
        }
        if (nav_.pass == NavigationPass::LowSpeed) nav_.trial_speed = nav_.low_speed;
    }
    status_.navigation_tuning_item = nav_.item;
    status_.navigation_candidate = nav_.candidate;
    status_.navigation_speed_probe = nav_.speed_probe;
    status_.closed_loop = false;
    validation_passed_ = false;
    if (!revise_runtime_candidates() || !apply_transaction(TransactionKind::Gains, now, true))
        return fail_step(Status::FAILURE_PARAMETER);
    // 候选事务已建立；与其他阶段共用Advance，由调用者进入Evaluate等待确认。
    return StepResult::Advance;
}

StepResult AutoCalibrationMode::end_navigation_item(std::uint64_t now, bool confirmed) noexcept
{
    if (nav_.item == Status::NAV_TUNE_SPEED_REDUCTION && nav_.speed_probe) {
        if (!confirmed || nav_.best_arrival <= tuning_config_.speed_threshold)
            return finish_navigation_tuning(now);
        nav_.turn_speeds[nav_.direction] = nav_.best_arrival;
        nav_.turn_bounded[nav_.direction] = nav_.speed_bracketed;
        if (nav_.direction == 0U) {
            nav_.direction = 1U;
            if (!start_navigation_item(now)) return fail_step(Status::FAILURE_PARAMETER);
            return StepResult::Advance;
        }
        const float angle = nav_.turn_angle;
        // 同一k须使两向速度上限都不超过独立实测能力，因此取两个不等式下界的最大值。
        if (!nav_.turn_bounded[0] && !nav_.turn_bounded[1]) {
            PX4_INFO_RAW("[autocal] turn speed reached cruise cap; reduction not identifiable\n");
            return finish_navigation_tuning(now);
        }
        // 成功到巡航/观测上限只说明能力>=上限，不是转弯极限；删失方向不参与等式反解。
        const float left = nav_.turn_bounded[0] ? kPi / angle * (1.0F - nav_.turn_speeds[0] / status_.maximum_speed_m_s) : 0.0F;
        const float right = nav_.turn_bounded[1] ? kPi / angle * (1.0F - nav_.turn_speeds[1] / status_.maximum_speed_m_s) : 0.0F;
        nav_.candidate = std::max({0.0F, left, right});
        if (!std::isfinite(nav_.candidate) || nav_.candidate < 0.0F || nav_.candidate > 100.0F)
            return finish_navigation_tuning(now);
        nav_.direction = 0U; nav_.speed_probe = false;
        nav_.pass = NavigationPass::Confirm;
        return apply_navigation_trial(now);
    }
    if (confirmed) {
        nav_.accepted[nav_.item - 1U] = nav_.candidate;
        const auto bit = nav_.item == Status::NAV_TUNE_LOOKAHEAD ? Status::NAV_FIELD_LOOKAHEAD :
            nav_.item == Status::NAV_TUNE_JERK ? Status::NAV_FIELD_JERK : Status::NAV_FIELD_SPEED_REDUCTION;
        nav_.evidence |= bit;
    }
    if (nav_.item == Status::NAV_TUNE_LOOKAHEAD) {
        if (!confirmed && (nav_.accepted[0] < 0.1F || nav_.accepted[0] > 100.0F)) return finish_navigation_tuning(now);
        nav_.item = Status::NAV_TUNE_JERK;
    } else if (nav_.item == Status::NAV_TUNE_JERK) {
        // 种子未获实测确认不得带入后续能力标定或最终保存。
        if (!confirmed && nav_.original[1] <= 0.0F) return finish_navigation_tuning(now);
        nav_.item = Status::NAV_TUNE_SPEED_REDUCTION; nav_.direction = 0U;
    } else {
        // 三项分别有证据后，用最终配置重跑方形和两个连续转弯方向。
        nav_.pass = NavigationPass::Final; nav_.direction = 0U;
        return apply_navigation_trial(now);
    }
    if (!start_navigation_item(now)) return finish_navigation_tuning(now);
    return StepResult::Advance;
}

StepResult AutoCalibrationMode::finish_navigation_tuning(std::uint64_t now) noexcept
{
    // 未确认参数回到会话原值；已确认项保留。恢复也必须经过真实前端代次确认。
    const std::uint32_t bits[3]{Status::NAV_FIELD_LOOKAHEAD, Status::NAV_FIELD_JERK, Status::NAV_FIELD_SPEED_REDUCTION};
    for (unsigned i = 0U; i < 3U; ++i)
        if ((nav_.evidence & bits[i]) == 0U) nav_.accepted[i] = nav_.original[i];
    nav_.pass = NavigationPass::Restore;
    return apply_navigation_trial(now);
}

StepResult AutoCalibrationMode::finish_path_validation(std::uint64_t now) noexcept
{
    const auto result = nav_.result;
    PX4_INFO_RAW("[autocal] nav item=%u candidate=%.3f result=%s xte=%.3f speed_rms=%.3f jerk_rms=%.3f\n",
        unsigned(nav_.item), double(nav_.candidate), result.valid ? "pass" : nav_.incomplete ? "unobservable" : nav_.failed ? "performance" : "unobservable",
        double(nav_.peak_error), double(status_.navigation_speed_rms_m_s), double(status_.navigation_jerk_rms_m_s3));
    if (nav_.pass == NavigationPass::Final) {
        if (!result.valid) { nav_.evidence = 0U; return finish_navigation_tuning(now); }
        if (++nav_.direction < 5U) return apply_navigation_trial(now);
        return finish_navigation_tuning(now);
    }
    if (nav_.pass == NavigationPass::LowSpeed) {
        return end_navigation_item(now, result.valid);
    }
    if (nav_.pass == NavigationPass::Fallback) {
        // 回退赢家也要按名义工况复跑一次；失败不能再次回到同一基线形成重试环。
        if (result.valid && nav_.item == Status::NAV_TUNE_LOOKAHEAD && nav_.low_speed < nav_.nominal_speed) {
            nav_.pass = NavigationPass::LowSpeed;
            return apply_navigation_trial(now);
        }
        return end_navigation_item(now, result.valid);
    }
    if (nav_.pass == NavigationPass::Confirm) {
        if (!nav_.speed_probe && nav_.item == Status::NAV_TUNE_SPEED_REDUCTION) {
            if (!result.valid) return finish_navigation_tuning(now);
            if (nav_.direction == 0U) { nav_.direction = 1U; return apply_navigation_trial(now); }
            return end_navigation_item(now, true);
        }
        // 选中新值必须再次优于原基线；不一致时保留已验证的原值，而非无界复跑。
        const bool improved = !nav_.candidate_changed || math::better_trial(result, nav_.baseline);
        if (!result.valid || !improved) {
            if (nav_.candidate_changed && nav_.baseline.valid && !nav_.speed_probe) {
                nav_.candidate = nav_.accepted[nav_.item - 1U];
                nav_.pass = NavigationPass::Fallback;
                return apply_navigation_trial(now);
            }
            return end_navigation_item(now, false);
        }
        if (nav_.item == Status::NAV_TUNE_LOOKAHEAD && nav_.low_speed < nav_.nominal_speed) {
            nav_.pass = NavigationPass::LowSpeed;
            return apply_navigation_trial(now);
        }
        if (nav_.speed_probe) nav_.best_arrival = nav_.measured_arrival;
        return end_navigation_item(now, true);
    }
    if (result.valid && (nav_.speed_probe ? (!nav_.best.valid || nav_.candidate > nav_.best_value) :
        math::better_trial(result, nav_.best))) {
        nav_.best = result; nav_.best_value = nav_.candidate; nav_.best_arrival = nav_.measured_arrival;
    }
    if (nav_.speed_probe && !result.valid && nav_.turn_failed && !nav_.incomplete && nav_.arrival_observed)
        nav_.speed_bracketed = true;
    if (nav_.pass == NavigationPass::UpperBound) {
        if (result.valid) { // 真正跑过上界，禁止把巡航限速误当车辆转弯能力。
            nav_.speed_bracketed = false; // 即使较低速度曾失败，上界通过也不能声称已找到单调能力边界。
            nav_.best_value = nav_.candidate; nav_.best = result; nav_.best_arrival = nav_.measured_arrival;
            nav_.candidate_changed = false; nav_.pass = NavigationPass::Confirm;
            return apply_navigation_trial(now);
        }
        nav_.search_ready = nav_.speed_bracketed && nav_.search.reset(
            nav_.best.valid ? nav_.best_value : nav_.lower,
            nav_.baseline.valid ? nav_.upper : nav_.nominal_speed, 0.01F, true);
    }
    // 观测缺失、几何遮蔽不是物理不合格，不能据此收缩黄金分割/速度二分区间。
    if ((nav_.pass == NavigationPass::Search || nav_.pass == NavigationPass::UpperBound) &&
        !result.valid && (!nav_.failed || nav_.incomplete)) nav_.search_ready = false;
    if (nav_.pass == NavigationPass::Baseline) {
        nav_.baseline = result;
        if (nav_.speed_probe) {
            if (!nav_.incomplete && (result.valid || nav_.speed_bracketed) && nav_.candidate < nav_.upper) {
                nav_.pass = NavigationPass::UpperBound; nav_.candidate = nav_.upper;
                return apply_navigation_trial(now);
            }
            nav_.search_ready = !nav_.incomplete && (result.valid || nav_.speed_bracketed) && nav_.search.reset(
                result.valid ? nav_.candidate : nav_.lower, result.valid ? nav_.upper : nav_.candidate, 0.01F, true);
        }
        if (nav_.item == Status::NAV_TUNE_LOOKAHEAD && nav_.candidate != 1.0F && nav_.lower <= 1.0F && nav_.upper >= 1.0F) {
            nav_.candidate = 1.0F; nav_.pass = NavigationPass::Reference;
            return apply_navigation_trial(now);
        }
    }
    if (nav_.search_ready && (nav_.pass != NavigationPass::Search || nav_.search.observe(result))) {
        nav_.pass = NavigationPass::Search;
        nav_.candidate = nav_.search.candidate();
        return apply_navigation_trial(now);
    }
    if (!nav_.best.valid) return end_navigation_item(now, false);
    nav_.candidate = nav_.best_value;
    nav_.candidate_changed = !nav_.speed_probe && nav_.candidate != nav_.accepted[nav_.item - 1U];
    nav_.pass = NavigationPass::Confirm;
    return apply_navigation_trial(now);
}

} // namespace dima::rover::modes
