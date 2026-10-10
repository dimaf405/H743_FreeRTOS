#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include "sensors/SensorRotation.hpp"
#include "world_magnetic_model/geo_mag_declination.h"
#include "matrix/math.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::rover::modes {
namespace {

bool predicted_field(const vehicle_attitude_s &attitude, const sensor_gps_s &gps,
                     matrix::Vector3f &body) noexcept
{
    const matrix::Quatf q(attitude.q);
    if (!q.isAllFinite() || std::fabs(q.norm() - 1.0F) > 0.01F) return false;
    constexpr float radians = 0.01745329251994329577F;
    const matrix::Eulerf tilt(q);
    // 调用方先证明 GNSS yaw 实际融合且 mag_hdg/mag_3d 均关闭。使用 EKF
    // 已按 GNSS delay 对齐并传播至 attitude 时间的姿态，避免把接收机到达
    // 时刻的旧阵列 heading 当成当前时刻的 yaw。磁场不能参与该 yaw 闭环。
    if (std::fabs(tilt.phi()) > 15.0F * radians || std::fabs(tilt.theta()) > 15.0F * radians) return false;
    const float declination = get_mag_declination_degrees(gps.latitude_deg, gps.longitude_deg) * radians;
    const float inclination = get_mag_inclination_degrees(gps.latitude_deg, gps.longitude_deg) * radians;
    const float strength = get_mag_strength_gauss(gps.latitude_deg, gps.longitude_deg);
    if (!std::isfinite(strength) || strength < 0.1F || strength > 0.8F) return false;
    const matrix::Vector3f earth(strength * std::cos(inclination) * std::cos(declination),
        strength * std::cos(inclination) * std::sin(declination), strength * std::sin(inclination));
    body = matrix::Dcmf(q).transpose() * earth;
    return body.isAllFinite();
}

unsigned sectors(std::uint32_t mask) noexcept
{
    unsigned count = 0U;
    for (unsigned bit = 0U; bit < 24U; ++bit) if ((mask & (1UL << bit)) != 0U) ++count;
    return count;
}

} // namespace

// —— MAGNETIC 阶段 helper ———————————————————————————————————————————
// 磁路径只产出观测与候选：TURN 机动内的 bootstrap 覆盖/EKF 残差采样、
// 磁-油门逐轴回归样本、覆盖度/一致性门与残差判据。提交/恢复一律经
// begin_mag_transaction/commit_mag_throttle 建立事务候选，事务身份由调度器
// 的 TransactionKind::Magnetic / MagneticMotor 维护，本文件不做状态跳转。

bool AutoCalibrationMode::mag_scale_valid() const noexcept
{
    // 准入、采样和补偿共用同一scale合法域，不维护第二份配置缓存。
    for (float scale : config_.mag_scale) if (!std::isfinite(scale) || scale < 0.1F || scale > 3.0F) return false;
    return true;
}

bool AutoCalibrationMode::mag_rotation(float (&rotation)[9]) const noexcept
{
    if (!mag_scale_valid()) return false;
    const dima::lib::sensors::Vector3 fine{config_.board_offset[0], config_.board_offset[1], config_.board_offset[2]};
    return dima::lib::sensors::make_board_rotation_matrix(config_.mag_rotation, fine, rotation);
}

bool AutoCalibrationMode::mag_path_ready(std::uint64_t now) const noexcept
{
    // 纯检查：磁设备身份、有限配置、1..200 Hz 合同内 ≥10 Hz 前端与
    // raw/前端样本匹配均在新鲜窗内，才允许“仅磁校准”目的的运动。
    const auto &raw = raw_mag_sub_.get();
    const auto &mag = mag_sub_.get();
    // 前端只对已绑定设备应用offset/scale；ID=0的探测数据不能冒充候选已生效。
    if (!status_.magnetometer_present || config_.mag_id <= 0 ||
        !std::isfinite(config_.mag_rate) || config_.mag_rate < 10.0F || config_.mag_rate > 200.0F ||
        !dima::lib::sensors::valid_rotation(config_.mag_rotation) || raw.device_id == 0U ||
        raw.device_id != mag_device_id_ || static_cast<std::uint32_t>(config_.mag_id) != raw.device_id ||
        !fresh(raw.timestamp_sample, now, 300000ULL) || !fresh(mag.timestamp_sample, now, 300000ULL) ||
        !fresh(matched_mag_sample_, now, 300000ULL) ||
        mag.device_id != raw.device_id || !std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.z) ||
        std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z) > 2.0F) return false;
    for (float offset : config_.mag_offset) if (!std::isfinite(offset)) return false;
    for (float value : mag.magnetometer_ga) if (!std::isfinite(value)) return false;
    // fresh vehicle_magnetometer 证明前端最近确实消费并发布过数据；只靠 raw
    // device 存在不足以允许一次“仅磁校准”的自动运动。匹配时间按新 raw 样本
    // 单独记录，不能每轮用已经消费的旧 raw 对最新姿态重新判错时。1..200 Hz 的正常运行
    // 合同不变，本校准要求至少 10 Hz，避免低速前端必然无法满足连续融合门禁。
    return mag_scale_valid(); // 准入不消费矩阵，不提前执行采样阶段的三角运算。
}

void AutoCalibrationMode::collect_mag(std::uint64_t now, unsigned direction) noexcept
{
    if (!status_.magnetometer_present || direction >= 2U) return;
    const auto &raw = raw_mag_sub_.get();
    const auto &rtk = rtk_sub_.get();
    const bool identity_ok = config_.mag_id > 0 && raw.device_id == mag_device_id_ &&
        static_cast<std::uint32_t>(config_.mag_id) == raw.device_id;
    // source_ok 只检查设备身份/新鲜度/数值域。rtk_yaw_fused 曾放在这里作
    // 为姿态质量代理——旋转中 aid.test_ratio 会随速率持续超标（16°/s 下
    // 5 Hz RTK yaw 的 innovation 长期 >1），source_ok 持久 false → 10 s
    // 稳定窗永远走不满 → bias_fit 集不满 5 份 → 每方向白转满 45 s 延伸帽
    // （实车 6 圈根因）。bias 学习的正确门禁是下方 valid 块的 EKF 磁融合
    // 状态（cs_mag/!fault/!disturbed），不需要 GNSS yaw 融合健康；姿态由
    // 会话级 imu_quality 与 mag_rotation/predicted_field 的有限值检查兜底。
    const bool source_ok = identity_ok && fresh(raw.timestamp_sample, now, 200000ULL) &&
        std::isfinite(raw.x) && std::isfinite(raw.y) && std::isfinite(raw.z) &&
        std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z) <= 2.0F;
    if (!source_ok) {
        // 旋转中 GNSS yaw 融合的 aid.test_ratio 会随速率抖动，瞬时 false 不能
        // 清掉整段 10 s 稳定窗：只在持续 250 ms 不可用后才重置（实测 45 s 全程
        // 抖动会让 bias_fit 永远集不满 5 份，每个方向白转满延伸窗上限）。
        if (mag_source_bad_since_ == 0U) mag_source_bad_since_ = now;
        else if (now - mag_source_bad_since_ >= 250000ULL) mag_stable_since_ = 0U;
        return;
    }
    mag_source_bad_since_ = 0U;
    const auto attitude_sample = attitude_sub_.get().timestamp_sample;
    const auto skew = raw.timestamp_sample > attitude_sample ? raw.timestamp_sample - attitude_sample : attitude_sample - raw.timestamp_sample;
    // 仅匹配相差 <=50 ms 的磁场/已传播姿态样本；分别 fresh 不能证明同一姿态。
    float rotation[9]{};
    if (!mag_rotation(rotation)) return;

    if (raw.timestamp_sample > last_mag_sample_) {
        last_mag_sample_ = raw.timestamp_sample;
        matrix::Vector3f field{};
        if (skew <= 50000ULL && predicted_field(attitude_sub_.get(), gps_sub_.get(), field)) {
            // 覆盖度描述实际采到的方向，与 bootstrap 的偏置幅值门禁分开；
            // 已有有效大偏置时，正常 EKF 残差学习仍可独立完成。
            const float phase = dima::lib::rover::calibration::wrap_pi(rtk.array_heading_rad) + kPi;
            const unsigned sector = std::min(23U, static_cast<unsigned>(24.0F * phase / (2.0F * kPi)));
            coverage_[direction] |= 1UL << sector;
            const float sensor[3]{raw.x, raw.y, raw.z};
            double candidate[3]{};
            for (unsigned axis = 0U; axis < 3U; ++axis) {
                // known-yaw bootstrap：offset=raw-S^-1*R_sensor_to_body^T*B_body。
                // WMM 给出场强/倾角先验，属于有界初始化，不是全三轴软铁标定。
                const float expected = rotation[axis] * field(0) + rotation[3U + axis] * field(1) + rotation[6U + axis] * field(2);
                candidate[axis] = sensor[axis] - expected / config_.mag_scale[axis];
            }
            if (std::hypot(std::hypot(candidate[0], candidate[1]), candidate[2]) <= 1.0) {
                bootstrap_fit_[direction].add(candidate[0], candidate[1], candidate[2]);
            }
        }
    }

    const auto &flags = flags_sub_.get();
    const auto &bias = bias_sub_.get();
    const auto &aid = mag_aid_sub_.get();
    // source_ok已统一确认配置ID与本轮设备身份，此处只检查对应设备的新鲜EKF偏置。
    // 不要求 bias.mag_bias_stable（2026-09-30 用户确认）：Ekf2Bias 的 stable 判据
    // 含 max<100×min 的各向同性比，为飞行器三轴姿态激励设计；地面车 yaw-only
    // 旋转的垂直轴 bias 物理不可观测（实测 Z 方差 8.5e-4 vs X/Y 1.1e-6，比值
    // 770:1），stable 永不置真，磁校准被结构性堵死。改用逐轴绝对方差上限
    // （与 Ekf2Bias kMaximumVariance 同值）+ 模式自有质量门（10s 稳定窗、
    // 修正幅值、finish_mag 样本方差与双向一致性）把关。
    bool valid = flags.cs_mag && !flags.cs_mag_hdg && !flags.cs_mag_3d && !flags.cs_mag_fault && !flags.cs_mag_field_disturbed &&
        fresh(aid.timestamp, now, 1500000ULL) && fresh(aid.time_last_fuse, now, 500000ULL) && aid.fused && !aid.innovation_rejected &&
        bias.mag_device_id == raw.device_id && fresh(bias.timestamp, now, 1500000ULL) &&
        bias.timestamp_sample > arm_started_ && bias.mag_bias_valid;
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        valid = valid && std::isfinite(bias.mag_bias[axis]) && std::isfinite(bias.mag_bias_variance[axis]) &&
            bias.mag_bias_variance[axis] >= 0.0F && bias.mag_bias_variance[axis] < 1.0e-3F;
    }
    if (!valid) { mag_stable_since_ = 0U; return; }
    if (mag_stable_since_ == 0U) mag_stable_since_ = now;
    // EKF 的 stable 可能沿用历史累计；此处还需要本会话连续 10 s，再按唯一
    // bias 时间戳采样。Armed 期间只写固定 RAM 累计器。
    if (now - mag_stable_since_ < 10000000ULL || bias.timestamp <= last_bias_sample_) return;
    last_bias_sample_ = bias.timestamp;
    double candidate[3]{};
    double correction_squared = 0.0;
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        const float correction = (rotation[axis] * bias.mag_bias[0] + rotation[3U + axis] * bias.mag_bias[1] +
            rotation[6U + axis] * bias.mag_bias[2]) / config_.mag_scale[axis];
        correction_squared += correction * correction;
        candidate[axis] = (bootstrap_applied_ ? bootstrap_offset_[axis] : config_.mag_offset[axis]) + correction;
    }
    if (correction_squared <= 0.09) bias_fit_[direction].add(candidate[0], candidate[1], candidate[2]);
}

bool AutoCalibrationMode::finish_mag(bool bootstrap) noexcept
{
    // 纯算法：双向覆盖度 + 样本量 + 方向一致门；候选写入 candidate_mag_。
    if (sectors(coverage_[0]) < 20U || sectors(coverage_[1]) < 20U) return false;
    const Stats3 *stats = bootstrap ? bootstrap_fit_ : bias_fit_;
    const unsigned required = bootstrap ? 60U : 5U;
    if (stats[0].count() < required || stats[1].count() < required) return false;
    const auto a = stats[0].mean(), b = stats[1].mean();
    const auto va = stats[0].variance(), vb = stats[1].variance();
    const double av[3]{a.x, a.y, a.z}, bv[3]{b.x, b.y, b.z};
    const double var_a[3]{va.x, va.y, va.z}, var_b[3]{vb.x, vb.y, vb.z};
    double difference_squared = 0.0;
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        if (!std::isfinite(av[axis]) || !std::isfinite(bv[axis]) ||
            var_a[axis] > 0.0004 || var_b[axis] > 0.0004) return false;
        difference_squared += (av[axis] - bv[axis]) * (av[axis] - bv[axis]);
        candidate_mag_[axis] = static_cast<float>(0.5 * (av[axis] + bv[axis]));
    }
    // CW/CCW 的电流磁干扰不同；固定偏置无法解释的方向差异必须拒绝。
    return difference_squared <= 0.0009;
}

float AutoCalibrationMode::mag_residual(std::uint64_t now) const noexcept
{
    const auto &mag = mag_sub_.get();
    matrix::Vector3f expected{};
    if (!fresh(mag.timestamp_sample, now, 300000ULL) || mag.device_id != mag_device_id_ ||
        !stopped() || !rtk_yaw_fused(now) || !predicted_field(attitude_sub_.get(), gps_sub_.get(), expected))
        return std::numeric_limits<float>::infinity();
    // 提交后的验证只在确认停车时使用窗口均值；窗口内姿态不变，不再把同一
    // 前端样本与每个新姿态比较时间差并反复清空稳定计时。
    const matrix::Vector3f measured(mag.magnetometer_ga);
    return measured.isAllFinite() ? (measured - expected).norm() : std::numeric_limits<float>::infinity();
}

void AutoCalibrationMode::sample_mag_throttle(std::uint64_t now) noexcept
{
    // 只学未补偿的原始传感器值，避免旧 K 反馈进入下一代。去/返程单独截距，
    // 掉头引起的环境磁场方向变化不应成为油门斜率。
    const auto &raw = raw_mag_sub_.get();
    if (!status_.magnetometer_present || mag_device_id_ == 0U || raw.device_id != mag_device_id_ ||
        !fresh(raw.timestamp_sample, now, 200000ULL) || raw.timestamp_sample > raw.timestamp ||
        raw.timestamp_sample <= last_mag_mot_sample_ || matched_mag_sample_ != raw.timestamp_sample ||
        ground_speed() < 0.1F || std::fabs(yaw_rate()) > 0.05F ||
        std::fabs(dima::lib::rover::calibration::wrap_pi(
            leg_heading_ - rtk_sub_.get().array_heading_rad)) > 3.0F * kRadians) return;
    const matrix::Vector3f field(raw.x, raw.y, raw.z);
    const matrix::Quatf attitude(attitude_sub_.get().q);
    if (!field.isAllFinite() || field.norm() > 2.0F || !attitude.isAllFinite() ||
        std::fabs(attitude.norm() - 1.0F) > 0.01F) return;
    const matrix::Eulerf tilt(attitude);
    if (std::fabs(tilt.phi()) > 15.0F * kRadians || std::fabs(tilt.theta()) > 15.0F * kRadians) return;
    float u{};
    if (!motor_history_.forward_output(raw.timestamp_sample, u) || u < 0.3F * config_.motor_maximum) return;
    // 仅observe_straight调用：去程Running与返程Return的有效直行分别进入两桶，
    // 通用返场的重对准/停车不参与磁-油门回归。
    const bool returning_leg = !session_.straight_outward;
    auto &fit = mag_mot_fit_[returning_leg ? 1U : 0U];
    if (fit.count == 0U) { fit.roll = tilt.phi(); fit.pitch = tilt.theta(); }
    if (std::fabs(tilt.phi() - fit.roll) > 2.0F * kRadians ||
        std::fabs(tilt.theta() - fit.pitch) > 2.0F * kRadians || fit.count == UINT32_MAX) return;
    last_mag_mot_sample_ = raw.timestamp_sample;
    ++fit.count;
    const double delta_u = u - fit.mean_u;
    fit.mean_u += delta_u / fit.count;
    fit.variance_u += delta_u * (u - fit.mean_u);
    fit.minimum_u = std::min(fit.minimum_u, u);
    fit.maximum_u = std::max(fit.maximum_u, u);
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        const double delta_b = field(axis) - fit.mean_b[axis];
        fit.mean_b[axis] += delta_b / fit.count;
        fit.covariance[axis] += delta_u * (field(axis) - fit.mean_b[axis]);
        fit.variance_b[axis] += delta_b * (field(axis) - fit.mean_b[axis]);
    }
}

bool AutoCalibrationMode::commit_mag_throttle(std::uint64_t now) noexcept
{
    // 事务操作：磁-油门补偿候选（只读观测参数写入）。不可观测=组如实失败
    //（STAGE_MAG_MOT 保持未完成），不产出比例法候选。事务身份
    //（TransactionKind::MagneticMotor）与推进由调度器维护，这里不做跳转。
    if (mag_mot_reported_) return false;
    mag_mot_reported_ = true;
    status_.unavailable_stages |= Status::STAGE_MAG_MOT;
    const auto &out = mag_mot_fit_[0];
    const auto &back = mag_mot_fit_[1];
    const double n = static_cast<double>(out.count) + back.count;
    const double variance_u = out.variance_u + back.variance_u;
    const double sum_uu = variance_u + out.count * out.mean_u * out.mean_u +
        back.count * back.mean_u * back.mean_u;
    const float span = std::max(out.maximum_u - out.minimum_u, back.maximum_u - back.minimum_u);
    float rotation[9]{};
    bool usable = mag_rotation(rotation) && mag_device_id_ != 0U && mag_device_id_ <= static_cast<std::uint32_t>(INT32_MAX) &&
        out.count >= 30U && back.count >= 30U && n >= 100.0 &&
        span >= 0.15F * config_.motor_maximum && variance_u > 1.0e-4 * sum_uu;
    float sensor_slope[3]{};
    double residual_squared = 0.0;
    for (unsigned axis = 0U; axis < 3U && usable; ++axis) {
        // 共用斜率 K=Σ段 cov(u,B)/Σ段 var(u)，各段截距独立；静态偏置变化
        // 只改变截距。残差先用最终 scale 换到校正后的 gauss 域再判 0.08 G。
        const double covariance = out.covariance[axis] + back.covariance[axis];
        const double k = covariance / variance_u;
        usable = std::isfinite(k) && std::fabs(k) <= 1.0;
        sensor_slope[axis] = static_cast<float>(k) * config_.mag_scale[axis];
        residual_squared += std::max(0.0, out.variance_b[axis] + back.variance_b[axis] -
            covariance * covariance / variance_u) * config_.mag_scale[axis] * config_.mag_scale[axis];
    }
    usable = usable && residual_squared <= 0.08 * 0.08 * n;
    float coefficient[3]{};
    for (unsigned axis = 0U; axis < 3U; ++axis)
        coefficient[axis] = rotation[axis * 3U] * sensor_slope[0] +
            rotation[axis * 3U + 1U] * sensor_slope[1] + rotation[axis * 3U + 2U] * sensor_slope[2];
    const float reference = get_mag_strength_gauss(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg);
    usable = usable && std::isfinite(reference) && reference >= 0.1F && reference <= 0.8F;
    if (!usable) {
        PX4_INFO("[autocal] mag throttle interference unobservable; compensation incomplete");
        return false;
    }
    // 干扰率是完整矢量模长相对当地 WMM 场强，允许超过 100%，未知用 -1。
    status_.mag_interference_pct = 100.0F * std::hypot(std::hypot(coefficient[0], coefficient[1]),
        coefficient[2]) * config_.motor_maximum / reference;
    PX4_INFO("[autocal] mag throttle %.0f%% interference; awaiting parameter commit",
        static_cast<double>(status_.mag_interference_pct));
    if (status_.mag_interference_pct > 30.0F)
        PX4_WARN("[autocal] magnetic interference above 30%%; check hardware placement");
    px4::AtomicTransaction atomic;
    std::int32_t generation{};
    if (param_get(param_handle(dima::params::CAL_MAG_MOT_GEN), &generation) != 0 ||
        generation < 0 || generation == INT32_MAX) {
        PX4_WARN("[autocal] mag throttle generation unavailable/exhausted; compensation incomplete");
        return false;
    }
    // 停车/Disarmed、前端代次确认、显式保存和回滚沿用同一有限事务，不允许
    // 五次裸写成功就宣称持久化完成。GEN 最后发布，整组通知在锁退出时合并。
    const bool prepared = transaction_.prepare() &&
        transaction_.add_float(dima::params::CAL_MAG_MOT_KX, coefficient[0]) &&
        transaction_.add_float(dima::params::CAL_MAG_MOT_KY, coefficient[1]) &&
        transaction_.add_float(dima::params::CAL_MAG_MOT_KZ, coefficient[2]) &&
        transaction_.add_int(dima::params::CAL_MAG_MOT_ID, static_cast<std::int32_t>(mag_device_id_)) &&
        transaction_.add_int(dima::params::CAL_MAG_MOT_GEN, generation + 1);
    if (prepared && apply_transaction(TransactionKind::MagneticMotor, now)) return true;
    if (transaction_.active()) return true; // 进入回滚/故障确认，不覆盖仍持有互锁的事务。
    PX4_WARN("[autocal] mag throttle transaction unavailable; compensation incomplete");
    return false;
}

} // namespace dima::rover::modes
