#include "Ekf2.hpp"

#include "api/Time.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::modules::ekf2 {

void Ekf2::publish_aid_sources(std::uint64_t now_us) noexcept
{
    // N1 只发布实际编译进 Core 的观测源。Topic alias 由 .msg 生成器提供，
    // 此处不维护第二份消息名、ID 或发布者清单。
    publish_aid_source(now_us, ekf_.aid_src_fake_hgt(),
                       last_aid_fake_hgt_sample_, aid_fake_hgt_pub_);
    publish_aid_source(now_us, ekf_.aid_src_fake_pos(),
                       last_aid_fake_pos_sample_, aid_fake_pos_pub_);
    publish_aid_source(now_us, ekf_.aid_src_gnss_hgt(),
                       last_aid_gnss_hgt_sample_, aid_gnss_hgt_pub_);
    publish_aid_source(now_us, ekf_.aid_src_gnss_pos(),
                       last_aid_gnss_pos_sample_, aid_gnss_pos_pub_);
    publish_aid_source(now_us, ekf_.aid_src_gnss_vel(),
                       last_aid_gnss_vel_sample_, aid_gnss_vel_pub_);
    publish_aid_source(now_us, ekf_.aid_src_gnss_yaw(),
                       last_aid_gnss_yaw_sample_, aid_gnss_yaw_pub_);
    publish_aid_source(now_us, ekf_.aid_src_mag(),
                       last_aid_mag_sample_, aid_mag_pub_);
    publish_aid_source(now_us, ekf_.aid_src_gravity(),
                       last_aid_gravity_sample_, aid_gravity_pub_);
}

void Ekf2::publish_event_flags(std::uint64_t now_us) noexcept
{
    const std::uint32_t information_events =
        ekf_.information_event_status().value;
    if (information_events != 0U) {
        ++information_event_changes_;
        event_flags_ = {};
        event_flags_.timestamp_sample = ekf_.time_delayed_us();
        event_flags_.information_event_changes =
            information_event_changes_;
        const auto &flags = ekf_.information_event_flags();
        event_flags_.gps_checks_passed = flags.gps_checks_passed;
        event_flags_.reset_vel_to_gps = flags.reset_vel_to_gps;
        event_flags_.reset_vel_to_flow = flags.reset_vel_to_flow;
        event_flags_.reset_vel_to_vision = flags.reset_vel_to_vision;
        event_flags_.reset_vel_to_zero = flags.reset_vel_to_zero;
        event_flags_.reset_pos_to_last_known =
            flags.reset_pos_to_last_known;
        event_flags_.reset_pos_to_gps = flags.reset_pos_to_gps;
        event_flags_.reset_pos_to_vision = flags.reset_pos_to_vision;
        event_flags_.starting_gps_fusion = flags.starting_gps_fusion;
        event_flags_.starting_vision_pos_fusion =
            flags.starting_vision_pos_fusion;
        event_flags_.starting_vision_vel_fusion =
            flags.starting_vision_vel_fusion;
        event_flags_.starting_vision_yaw_fusion =
            flags.starting_vision_yaw_fusion;
        event_flags_.yaw_aligned_to_imu_gps =
            flags.yaw_aligned_to_imu_gps;
        event_flags_.reset_hgt_to_baro = flags.reset_hgt_to_baro;
        event_flags_.reset_hgt_to_gps = flags.reset_hgt_to_gps;
        event_flags_.reset_hgt_to_rng = flags.reset_hgt_to_rng;
        event_flags_.reset_hgt_to_ev = flags.reset_hgt_to_ev;
        event_flags_.timestamp = now_us;
        (void)event_flags_pub_.publish(event_flags_);
        last_event_flags_published_ = now_us;
        ekf_.clear_information_events();
        return;
    }

    // 事件 Topic 在无新事件时仍以 1 Hz 重发最后快照，使后加入的消费者能取得
    // 当前计数；仅更新时间戳，绝不把历史事件再次计为新事件。
    if (last_event_flags_published_ != 0U &&
        (now_us < last_event_flags_published_ ||
         now_us - last_event_flags_published_ >= kPeriodicStatusUs)) {
        event_flags_.timestamp = now_us;
        (void)event_flags_pub_.publish(event_flags_);
        last_event_flags_published_ = now_us;
    }
}

void Ekf2::publish_gps_status(std::uint64_t now_us) noexcept
{
    const std::uint64_t sample_time_us =
        ekf_.get_gps_sample_delayed().time_us;
    if (sample_time_us == 0U || sample_time_us == last_gps_status_sample_) {
        return;
    }

    estimator_gps_status_s status{};
    status.timestamp_sample = sample_time_us;
    status.position_drift_rate_horizontal_m_s =
        ekf_.gps_horizontal_position_drift_rate_m_s();
    status.position_drift_rate_vertical_m_s =
        ekf_.gps_vertical_position_drift_rate_m_s();
    status.filtered_horizontal_speed_m_s =
        ekf_.gps_filtered_horizontal_velocity_m_s();
    status.checks_passed = ekf_.gps_checks_passed();
    const auto &fail = ekf_.gps_check_fail_status_flags();
    status.check_fail_gps_fix = fail.fix;
    status.check_fail_min_sat_count = fail.nsats;
    status.check_fail_max_pdop = fail.pdop;
    status.check_fail_max_horz_err = fail.hacc;
    status.check_fail_max_vert_err = fail.vacc;
    status.check_fail_max_spd_err = fail.sacc;
    status.check_fail_max_horz_drift = fail.hdrift;
    status.check_fail_max_vert_drift = fail.vdrift;
    status.check_fail_max_horz_spd_err = fail.hspeed;
    status.check_fail_max_vert_spd_err = fail.vspeed;
    status.check_fail_spoofed_gps = fail.spoofed;
    status.timestamp = now_us;
    (void)gps_status_pub_.publish(status);
    last_gps_status_sample_ = sample_time_us;
}

void Ekf2::publish_status(std::uint64_t now_us) noexcept
{
    estimator_status_s status{};
    status.timestamp_sample = ekf_.time_delayed_us();
    ekf_.getOutputTrackingError().copyTo(status.output_tracking_error);

    const ::parameters *const params = ekf_.getParamHandle();
    if (params != nullptr) {
        // PX4 的 EKF2_GPS_CHECK 位 0 从 nsats 开始，status 位 0 固定为 fix，
        // 因而掩码需左移一位并始终保留 fix 位。
        const std::uint16_t enabled_checks = static_cast<std::uint16_t>(
            (static_cast<std::uint32_t>(params->ekf2_gps_check) << 1U) |
            1U);
        status.gps_check_fail_flags = static_cast<std::uint16_t>(
            ekf_.gps_check_fail_status().value & enabled_checks);
    }

    status.control_mode_flags = ekf_.control_status().value;
    status.filter_fault_flags = ekf_.fault_status().value;

    const float velocity_xy_ratio =
        ekf_.getHorizontalVelocityInnovationTestRatio();
    const float velocity_z_ratio =
        ekf_.getVerticalVelocityInnovationTestRatio();
    if (std::isfinite(velocity_xy_ratio) &&
        std::isfinite(velocity_z_ratio)) {
        status.vel_test_ratio =
            std::max(velocity_xy_ratio, velocity_z_ratio);
    } else if (std::isfinite(velocity_xy_ratio)) {
        status.vel_test_ratio = velocity_xy_ratio;
    } else if (std::isfinite(velocity_z_ratio)) {
        status.vel_test_ratio = velocity_z_ratio;
    } else {
        status.vel_test_ratio =
            std::numeric_limits<float>::quiet_NaN();
    }

    status.hdg_test_ratio = ekf_.getHeadingInnovationTestRatio();
    status.pos_test_ratio =
        ekf_.getHorizontalPositionInnovationTestRatio();
    status.hgt_test_ratio =
        ekf_.getVerticalPositionInnovationTestRatio();
    status.tas_test_ratio = ekf_.getAirspeedInnovationTestRatio();
    status.hagl_test_ratio =
        ekf_.getHeightAboveGroundInnovationTestRatio();
    status.beta_test_ratio =
        ekf_.getSyntheticSideslipInnovationTestRatio();
    ekf_.get_ekf_lpos_accuracy(&status.pos_horiz_accuracy,
                               &status.pos_vert_accuracy);
    status.solution_status_flags = ekf_.get_ekf_soln_status();

    const auto &reset_count = ekf_.state_reset_status().reset_count;
    status.reset_count_vel_ne = reset_count.velNE;
    status.reset_count_vel_d = reset_count.velD;
    status.reset_count_pos_ne = reset_count.posNE;
    status.reset_count_pod_d = reset_count.posD;
    status.reset_count_quat = reset_count.quat;
    status.time_slip = static_cast<float>(last_time_slip_us_) * 1.0e-6F;

    constexpr float kMinimumPreflightTestRatio = 0.5F;
    status.pre_flt_fail_innov_heading =
        kMinimumPreflightTestRatio < status.hdg_test_ratio;
    status.pre_flt_fail_innov_height =
        kMinimumPreflightTestRatio < status.hgt_test_ratio;
    status.pre_flt_fail_innov_pos_horiz =
        kMinimumPreflightTestRatio < status.pos_test_ratio;
    status.pre_flt_fail_innov_vel_horiz =
        kMinimumPreflightTestRatio < velocity_xy_ratio;
    status.pre_flt_fail_innov_vel_vert =
        kMinimumPreflightTestRatio < velocity_z_ratio;
    status.pre_flt_fail_mag_field_disturbed =
        ekf_.control_status_flags().mag_field_disturbed;

    status.accel_device_id = accel_device_id_;
    status.gyro_device_id = gyro_device_id_;
    status.mag_device_id = mag_device_id_;
    ekf_.get_mag_checks(status.mag_inclination_deg,
                        status.mag_inclination_ref_deg,
                        status.mag_strength_gs,
                        status.mag_strength_ref_gs);
    status.timestamp = now_us;
    (void)status_pub_.publish(status);
}

void Ekf2::publish_status_flags(std::uint64_t now_us) noexcept
{
    bool publish = last_status_flags_published_ == 0U ||
                   now_us < last_status_flags_published_ ||
                   now_us - last_status_flags_published_ >=
                       kPeriodicStatusUs;

    if (ekf_.control_status().value != filter_control_status_) {
        filter_control_status_ = ekf_.control_status().value;
        ++control_status_changes_;
        publish = true;
    }
    if (ekf_.fault_status().value != filter_fault_status_) {
        filter_fault_status_ = ekf_.fault_status().value;
        ++fault_status_changes_;
        publish = true;
    }
    if (ekf_.innov_check_fail_status().value != innovation_fault_status_) {
        innovation_fault_status_ = ekf_.innov_check_fail_status().value;
        ++innovation_status_changes_;
        publish = true;
    }
    if (!publish) {
        return;
    }

    estimator_status_flags_s status{};
    status.timestamp_sample = ekf_.time_delayed_us();
    status.control_status_changes = control_status_changes_;
    const auto &control = ekf_.control_status_flags();
    status.cs_tilt_align = control.tilt_align;
    status.cs_yaw_align = control.yaw_align;
    status.cs_gnss_pos = control.gnss_pos;
    status.cs_opt_flow = control.opt_flow;
    status.cs_mag_hdg = control.mag_hdg;
    status.cs_mag_3d = control.mag_3D;
    status.cs_mag_dec = control.mag_dec;
    status.cs_in_air = control.in_air;
    // Wind 状态已由唯一 derivation.py 入口裁掉，保留 PX4 消息 ABI 字段但恒为 false。
    status.cs_wind = false;
    status.cs_baro_hgt = control.baro_hgt;
    status.cs_rng_hgt = control.rng_hgt;
    status.cs_gps_hgt = control.gps_hgt;
    status.cs_ev_pos = control.ev_pos;
    status.cs_ev_yaw = control.ev_yaw;
    status.cs_ev_hgt = control.ev_hgt;
    status.cs_fuse_beta = control.fuse_beta;
    status.cs_mag_field_disturbed = control.mag_field_disturbed;
    status.cs_fixed_wing = control.fixed_wing;
    status.cs_mag_fault = control.mag_fault;
    status.cs_fuse_aspd = control.fuse_aspd;
    status.cs_gnd_effect = control.gnd_effect;
    status.cs_rng_stuck = control.rng_stuck;
    status.cs_gnss_yaw = control.gnss_yaw;
    status.cs_mag_aligned_in_flight = control.mag_aligned_in_flight;
    status.cs_ev_vel = control.ev_vel;
    status.cs_synthetic_mag_z = control.synthetic_mag_z;
    status.cs_vehicle_at_rest = control.vehicle_at_rest;
    status.cs_gnss_yaw_fault = control.gnss_yaw_fault;
    status.cs_rng_fault = control.rng_fault;
    status.cs_inertial_dead_reckoning = control.inertial_dead_reckoning;
    status.cs_wind_dead_reckoning = false;
    status.cs_rng_kin_consistent = control.rng_kin_consistent;
    status.cs_fake_pos = control.fake_pos;
    status.cs_fake_hgt = control.fake_hgt;
    status.cs_gravity_vector = control.gravity_vector;
    status.cs_mag = control.mag;
    status.cs_ev_yaw_fault = control.ev_yaw_fault;
    status.cs_mag_heading_consistent = control.mag_heading_consistent;
    status.cs_aux_gpos = control.aux_gpos;
    status.cs_rng_terrain = control.rng_terrain;
    status.cs_opt_flow_terrain = control.opt_flow_terrain;
    status.cs_valid_fake_pos = control.valid_fake_pos;
    status.cs_constant_pos = control.constant_pos;
    status.cs_baro_fault = control.baro_fault;
    status.cs_gnss_vel = control.gnss_vel;
    status.cs_gnss_fault = control.gnss_fault;
    status.cs_yaw_manual = control.yaw_manual;
    status.cs_gnss_hgt_fault = control.gnss_hgt_fault;

    status.fault_status_changes = fault_status_changes_;
    const auto &fault = ekf_.fault_status_flags();
    status.fs_bad_mag_x = fault.bad_mag_x;
    status.fs_bad_mag_y = fault.bad_mag_y;
    status.fs_bad_mag_z = fault.bad_mag_z;
    status.fs_bad_hdg = fault.bad_hdg;
    status.fs_bad_mag_decl = fault.bad_mag_decl;
    status.fs_bad_airspeed = fault.bad_airspeed;
    status.fs_bad_sideslip = fault.bad_sideslip;
    status.fs_bad_optflow_x = fault.bad_optflow_X;
    status.fs_bad_optflow_y = fault.bad_optflow_Y;
    status.fs_bad_acc_vertical = fault.bad_acc_vertical;
    status.fs_bad_acc_clipping = fault.bad_acc_clipping;

    status.innovation_fault_status_changes = innovation_status_changes_;
    const auto &innovation = ekf_.innov_check_fail_status_flags();
    status.reject_hor_vel = innovation.reject_hor_vel;
    status.reject_ver_vel = innovation.reject_ver_vel;
    status.reject_hor_pos = innovation.reject_hor_pos;
    status.reject_ver_pos = innovation.reject_ver_pos;
    status.reject_yaw = innovation.reject_yaw;
    status.reject_airspeed = innovation.reject_airspeed;
    status.reject_sideslip = innovation.reject_sideslip;
    status.reject_hagl = innovation.reject_hagl;
    status.reject_optflow_x = innovation.reject_optflow_X;
    status.reject_optflow_y = innovation.reject_optflow_Y;
    status.timestamp = now_us;
    (void)status_flags_pub_.publish(status);
    last_status_flags_published_ = now_us;
}

void Ekf2::publish_yaw_estimator_status(std::uint64_t now_us) noexcept
{
    static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) ==
                      N_MODELS_EKFGSF,
                  "yaw_estimator_status_s::yaw size mismatch");
    yaw_estimator_status_s status{};
    if (!ekf_.getDataEKFGSF(&status.yaw_composite, &status.yaw_variance,
                            status.yaw, status.innov_vn,
                            status.innov_ve, status.weight)) {
        return;
    }
    status.yaw_composite_valid =
        ekf_.isYawEmergencyEstimateAvailable();
    status.timestamp_sample = ekf_.time_delayed_us();
    status.timestamp = now_us;
    (void)yaw_estimator_status_pub_.publish(status);
}

} // namespace dima::modules::ekf2
