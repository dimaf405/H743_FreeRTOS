#include "Ekf2.hpp"

#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace dima::modules::ekf2 {
namespace {

template<dima::params Parameter>
bool load_float(float &destination) noexcept
{
    // 名称、索引与类型来自参数生成枚举；临时 typed handle 只读取真实消费者，
    // 不在 EKF2 再维护字符串参数表或第二份默认值。
    dima::ParamFloat<Parameter> value{};
    if (!value.bind() || !std::isfinite(value.get())) {
        return false;
    }
    destination = value.get();
    return true;
}

template<dima::params Parameter>
bool load_int(std::int32_t &destination) noexcept
{
    dima::ParamInt<Parameter> value{};
    if (!value.bind()) {
        return false;
    }
    destination = value.get();
    return true;
}

bool valid_mag_type(std::int32_t value) noexcept
{
    return value == MagFuseType::AUTO || value == MagFuseType::HEADING ||
           value == MagFuseType::NONE || value == MagFuseType::INIT;
}

void apply_candidate(::parameters &destination,
                     const ::parameters &source) noexcept
{
    // Core parameters 含 PX4 编译期 const 限值，整个 struct 没有赋值运算符。
    // 候选已经完整读取并通过验证，以下纯赋值阶段不会失败；只复制本适配器实际
    // 消费的可变字段，const 限值继续由 PX4 common.h 唯一定义。
    destination.ekf2_predict_us = source.ekf2_predict_us;
    destination.ekf2_delay_max = source.ekf2_delay_max;
    destination.ekf2_angerr_init = source.ekf2_angerr_init;
    destination.ekf2_hdg_gate = source.ekf2_hdg_gate;
    destination.ekf2_head_noise = source.ekf2_head_noise;
    destination.ekf2_noaid_noise = source.ekf2_noaid_noise;
    destination.ekf2_noaid_tout = source.ekf2_noaid_tout;
    destination.ekf2_hgt_ref = source.ekf2_hgt_ref;
    destination.ekf2_imu_ctrl = source.ekf2_imu_ctrl;
    destination.ekf2_gyr_noise = source.ekf2_gyr_noise;
    destination.ekf2_acc_noise = source.ekf2_acc_noise;
    destination.imu_pos_body = source.imu_pos_body;
    destination.ekf2_vel_lim = source.ekf2_vel_lim;

    destination.ekf2_abias_init = source.ekf2_abias_init;
    destination.ekf2_acc_b_noise = source.ekf2_acc_b_noise;
    destination.ekf2_abl_lim = source.ekf2_abl_lim;
    destination.ekf2_abl_acclim = source.ekf2_abl_acclim;
    destination.ekf2_abl_gyrlim = source.ekf2_abl_gyrlim;
    destination.ekf2_abl_tau = source.ekf2_abl_tau;
    destination.ekf2_gbias_init = source.ekf2_gbias_init;
    destination.ekf2_gyr_b_noise = source.ekf2_gyr_b_noise;
    destination.ekf2_gyr_b_lim = source.ekf2_gyr_b_lim;

    destination.ekf2_gps_ctrl = source.ekf2_gps_ctrl;
    destination.ekf2_gps_mode = source.ekf2_gps_mode;
    destination.ekf2_gps_delay = source.ekf2_gps_delay;
    destination.ekf2_gps_p_noise = source.ekf2_gps_p_noise;
    destination.ekf2_gps_p_gate = source.ekf2_gps_p_gate;
    destination.ekf2_gps_v_gate = source.ekf2_gps_v_gate;
    destination.ekf2_gps_v_noise = source.ekf2_gps_v_noise;
    destination.gps_pos_body = source.gps_pos_body;
    destination.ekf2_gps_check = source.ekf2_gps_check;
    destination.ekf2_req_eph = source.ekf2_req_eph;
    destination.ekf2_req_epv = source.ekf2_req_epv;
    destination.ekf2_req_sacc = source.ekf2_req_sacc;
    destination.ekf2_req_nsats = source.ekf2_req_nsats;
    destination.ekf2_req_pdop = source.ekf2_req_pdop;
    destination.ekf2_req_hdrift = source.ekf2_req_hdrift;
    destination.ekf2_req_vdrift = source.ekf2_req_vdrift;
    destination.ekf2_req_fix = source.ekf2_req_fix;
    destination.ekf2_grav_noise = source.ekf2_grav_noise;
    destination.ekf2_mag_type = source.ekf2_mag_type;
    destination.ekf2_mag_delay = source.ekf2_mag_delay;
    destination.ekf2_mag_gate = source.ekf2_mag_gate;
    destination.ekf2_mag_noise = source.ekf2_mag_noise;
    destination.ekf2_mag_b_noise = source.ekf2_mag_b_noise;
    destination.ekf2_mag_e_noise = source.ekf2_mag_e_noise;
    destination.ekf2_decl_type = source.ekf2_decl_type;
    destination.ekf2_mag_acclim = source.ekf2_mag_acclim;
    destination.ekf2_mag_check = source.ekf2_mag_check;
    destination.ekf2_mag_chk_str = source.ekf2_mag_chk_str;
    destination.ekf2_mag_chk_inc = source.ekf2_mag_chk_inc;
    destination.ekf2_synt_mag_z = source.ekf2_synt_mag_z;
    destination.ekf2_mag_decl = source.ekf2_mag_decl;
    destination.position_sensor_ref = source.position_sensor_ref;
}

} // namespace

bool Ekf2::load_parameters(bool initial) noexcept
{
    ::parameters *const active = ekf_.getParamHandle();
    if (active == nullptr) {
        return false;
    }
    const ::parameters previous{*active};
    ::parameters candidate{previous};
    float tau_velocity = 0.0F;
    float tau_position = 0.0F;
    float gps_health_seconds = 0.0F;

    bool valid = true;
    valid = load_int<dima::params::EKF2_PREDICT_US>(
                candidate.ekf2_predict_us) && valid;
    valid = load_float<dima::params::EKF2_DELAY_MAX>(
                candidate.ekf2_delay_max) && valid;
    valid = load_float<dima::params::EKF2_ANGERR_INIT>(
                candidate.ekf2_angerr_init) && valid;
    valid = load_float<dima::params::EKF2_HDG_GATE>(
                candidate.ekf2_hdg_gate) && valid;
    valid = load_float<dima::params::EKF2_HEAD_NOISE>(
                candidate.ekf2_head_noise) && valid;
    valid = load_float<dima::params::EKF2_NOAID_NOISE>(
                candidate.ekf2_noaid_noise) && valid;
    valid = load_int<dima::params::EKF2_NOAID_TOUT>(
                candidate.ekf2_noaid_tout) && valid;
    valid = load_int<dima::params::EKF2_HGT_REF>(
                candidate.ekf2_hgt_ref) && valid;
    valid = load_int<dima::params::EKF2_IMU_CTRL>(
                candidate.ekf2_imu_ctrl) && valid;
    valid = load_float<dima::params::EKF2_GYR_NOISE>(
                candidate.ekf2_gyr_noise) && valid;
    valid = load_float<dima::params::EKF2_ACC_NOISE>(
                candidate.ekf2_acc_noise) && valid;
    valid = load_float<dima::params::EKF2_IMU_POS_X>(
                candidate.imu_pos_body(0)) && valid;
    valid = load_float<dima::params::EKF2_IMU_POS_Y>(
                candidate.imu_pos_body(1)) && valid;
    valid = load_float<dima::params::EKF2_IMU_POS_Z>(
                candidate.imu_pos_body(2)) && valid;
    valid = load_float<dima::params::EKF2_TAU_VEL>(tau_velocity) && valid;
    valid = load_float<dima::params::EKF2_TAU_POS>(tau_position) && valid;
    valid = load_float<dima::params::EKF2_VEL_LIM>(
                candidate.ekf2_vel_lim) && valid;

    valid = load_float<dima::params::EKF2_ABIAS_INIT>(
                candidate.ekf2_abias_init) && valid;
    valid = load_float<dima::params::EKF2_ACC_B_NOISE>(
                candidate.ekf2_acc_b_noise) && valid;
    valid = load_float<dima::params::EKF2_ABL_LIM>(
                candidate.ekf2_abl_lim) && valid;
    valid = load_float<dima::params::EKF2_ABL_ACCLIM>(
                candidate.ekf2_abl_acclim) && valid;
    valid = load_float<dima::params::EKF2_ABL_GYRLIM>(
                candidate.ekf2_abl_gyrlim) && valid;
    valid = load_float<dima::params::EKF2_ABL_TAU>(
                candidate.ekf2_abl_tau) && valid;
    valid = load_float<dima::params::EKF2_GBIAS_INIT>(
                candidate.ekf2_gbias_init) && valid;
    valid = load_float<dima::params::EKF2_GYR_B_NOISE>(
                candidate.ekf2_gyr_b_noise) && valid;
    valid = load_float<dima::params::EKF2_GYR_B_LIM>(
                candidate.ekf2_gyr_b_lim) && valid;

    valid = load_int<dima::params::EKF2_GPS_CTRL>(
                candidate.ekf2_gps_ctrl) && valid;
    valid = load_int<dima::params::EKF2_GPS_MODE>(
                candidate.ekf2_gps_mode) && valid;
    valid = load_float<dima::params::EKF2_GPS_DELAY>(
                candidate.ekf2_gps_delay) && valid;
    valid = load_float<dima::params::EKF2_GPS_P_NOISE>(
                candidate.ekf2_gps_p_noise) && valid;
    valid = load_float<dima::params::EKF2_GPS_P_GATE>(
                candidate.ekf2_gps_p_gate) && valid;
    valid = load_float<dima::params::EKF2_GPS_V_GATE>(
                candidate.ekf2_gps_v_gate) && valid;
    valid = load_float<dima::params::EKF2_GPS_V_NOISE>(
                candidate.ekf2_gps_v_noise) && valid;
    valid = load_float<dima::params::EKF2_GPS_POS_X>(
                candidate.gps_pos_body(0)) && valid;
    valid = load_float<dima::params::EKF2_GPS_POS_Y>(
                candidate.gps_pos_body(1)) && valid;
    valid = load_float<dima::params::EKF2_GPS_POS_Z>(
                candidate.gps_pos_body(2)) && valid;
    valid = load_int<dima::params::EKF2_GPS_CHECK>(
                candidate.ekf2_gps_check) && valid;
    valid = load_float<dima::params::EKF2_REQ_EPH>(
                candidate.ekf2_req_eph) && valid;
    valid = load_float<dima::params::EKF2_REQ_EPV>(
                candidate.ekf2_req_epv) && valid;
    valid = load_float<dima::params::EKF2_REQ_SACC>(
                candidate.ekf2_req_sacc) && valid;
    valid = load_int<dima::params::EKF2_REQ_NSATS>(
                candidate.ekf2_req_nsats) && valid;
    valid = load_float<dima::params::EKF2_REQ_PDOP>(
                candidate.ekf2_req_pdop) && valid;
    valid = load_float<dima::params::EKF2_REQ_HDRIFT>(
                candidate.ekf2_req_hdrift) && valid;
    valid = load_float<dima::params::EKF2_REQ_VDRIFT>(
                candidate.ekf2_req_vdrift) && valid;
    valid = load_int<dima::params::EKF2_REQ_FIX>(
                candidate.ekf2_req_fix) && valid;
    valid = load_float<dima::params::EKF2_REQ_GPS_H>(
                gps_health_seconds) && valid;
    valid = load_float<dima::params::EKF2_GRAV_NOISE>(
                candidate.ekf2_grav_noise) && valid;

    valid = load_int<dima::params::EKF2_MAG_TYPE>(
                candidate.ekf2_mag_type) && valid;
    valid = load_float<dima::params::EKF2_MAG_DELAY>(
                candidate.ekf2_mag_delay) && valid;
    valid = load_float<dima::params::EKF2_MAG_GATE>(
                candidate.ekf2_mag_gate) && valid;
    valid = load_float<dima::params::EKF2_MAG_NOISE>(
                candidate.ekf2_mag_noise) && valid;
    valid = load_float<dima::params::EKF2_MAG_B_NOISE>(
                candidate.ekf2_mag_b_noise) && valid;
    valid = load_float<dima::params::EKF2_MAG_E_NOISE>(
                candidate.ekf2_mag_e_noise) && valid;
    valid = load_int<dima::params::EKF2_DECL_TYPE>(
                candidate.ekf2_decl_type) && valid;
    valid = load_float<dima::params::EKF2_MAG_ACCLIM>(
                candidate.ekf2_mag_acclim) && valid;
    valid = load_int<dima::params::EKF2_MAG_CHECK>(
                candidate.ekf2_mag_check) && valid;
    valid = load_float<dima::params::EKF2_MAG_CHK_STR>(
                candidate.ekf2_mag_chk_str) && valid;
    valid = load_float<dima::params::EKF2_MAG_CHK_INC>(
                candidate.ekf2_mag_chk_inc) && valid;
    valid = load_int<dima::params::EKF2_SYNT_MAG_Z>(
                candidate.ekf2_synt_mag_z) && valid;
    valid = load_float<dima::params::EKF2_MAG_DECL>(
                candidate.ekf2_mag_decl) && valid;

    // N1 没有 Baro/Range/EV 高度源；接受其他 EKF2_HGT_REF 会制造公开参数与
    // 编译闭包冲突，因此拒绝整次候选更新，而不是静默运行一个不存在的源。
    valid = valid &&
            candidate.ekf2_hgt_ref ==
                static_cast<std::int32_t>(HeightSensor::GNSS) &&
            candidate.ekf2_predict_us >= 1000 &&
            candidate.ekf2_predict_us <= 20000 &&
            candidate.ekf2_imu_ctrl >= 0 && candidate.ekf2_imu_ctrl <= 7 &&
             candidate.ekf2_gps_ctrl >= 0 && candidate.ekf2_gps_ctrl <= 15 &&
             candidate.ekf2_gps_mode >= 0 && candidate.ekf2_gps_mode <= 1 &&
             gps_health_seconds >= 0.0F &&
             valid_mag_type(candidate.ekf2_mag_type);
    if (!valid) {
        return false;
    }

    // PX4 VerifyParams 保证 delay_max 不小于任一观测延迟。这里只修正候选内存
    // 值，不回写参数，也不新增持久化动作。
    candidate.ekf2_delay_max = std::max(
        candidate.ekf2_delay_max,
        std::max(candidate.ekf2_gps_delay, candidate.ekf2_mag_delay));
    candidate.position_sensor_ref =
        static_cast<std::int32_t>(PositionSensor::GNSS);

    if (!initial) {
        // YAML 标注 reboot_required 的字段不能在已经分配的延迟缓冲上半应用；
        // 运行期只采纳可在线调参项，以下值继续使用本次启动快照。
        // PREDICT_US 与 DELAY_MAX 共同决定 RingBuffer 容量，任一在线变化都会使
        // 已分配长度和 PX4 容量公式失配，因此两者必须作为同一重启边界处理。
        candidate.ekf2_predict_us = previous.ekf2_predict_us;
        candidate.ekf2_delay_max = previous.ekf2_delay_max;
        candidate.ekf2_angerr_init = previous.ekf2_angerr_init;
        candidate.ekf2_hgt_ref = previous.ekf2_hgt_ref;
        candidate.ekf2_abias_init = previous.ekf2_abias_init;
        candidate.ekf2_gbias_init = previous.ekf2_gbias_init;
        candidate.ekf2_gps_delay = previous.ekf2_gps_delay;
        candidate.ekf2_mag_type = previous.ekf2_mag_type;
        candidate.ekf2_mag_delay = previous.ekf2_mag_delay;
        candidate.ekf2_decl_type = previous.ekf2_decl_type;
    }

    apply_candidate(*active, candidate);
    ekf_.output_predictor().set_imu_offset(candidate.imu_pos_body);
    ekf_.output_predictor().set_pos_correction_tc(tau_position);
    ekf_.output_predictor().set_vel_correction_tc(tau_velocity);
    if (initial) {
        constexpr double kMicrosecondsPerSecond = 1000000.0;
        const double health_us =
            static_cast<double>(gps_health_seconds) *
            kMicrosecondsPerSecond;
        // UINT32_MAX 不能先转 float：该值会舍入为 2^32，随后窄化越界。使用
        // double 比较并在转换前饱和，保持 PX4 微秒合同且不制造回绕健康时间。
        const std::uint32_t health_time_us =
            health_us >=
                    static_cast<double>(
                        std::numeric_limits<std::uint32_t>::max())
                ? std::numeric_limits<std::uint32_t>::max()
                : static_cast<std::uint32_t>(health_us);
        ekf_.set_min_required_gps_health_time(
            health_time_us);
    }
    ekf_.updateParameters();
    return true;
}

} // namespace dima::modules::ekf2
