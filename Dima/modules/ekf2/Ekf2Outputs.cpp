#include "Ekf2.hpp"

#include "api/Time.hpp"

#include <cmath>
#include <limits>

namespace dima::modules::ekf2 {

void Ekf2::publish_attitude(std::uint64_t sample_time_us) noexcept
{
    if (!ekf_.attitude_valid()) {
        return;
    }

    vehicle_attitude_s attitude{};
    attitude.timestamp_sample = sample_time_us;
    ekf_.getQuaternion().copyTo(attitude.q);
    ekf_.get_quat_reset(attitude.delta_q_reset,
                        &attitude.quat_reset_counter);
    attitude.timestamp = hrt_absolute_time();
    (void)attitude_pub_.publish(attitude);
}

void Ekf2::publish_local_position(std::uint64_t sample_time_us) noexcept
{
    vehicle_local_position_s local_position{};
    local_position.timestamp_sample = sample_time_us;

    // EKF 输出统一采用 NED：x/y/z 分别为北/东/下，速度和加速度沿用同一坐标系。
    // 这里保持 PX4 的 predictor 输出，不在 Wrapper 中叠加 Rover 专属坐标变换。
    const matrix::Vector3f position{ekf_.getPosition()};
    local_position.x = position(0);
    local_position.y = position(1);
    local_position.z = position(2);

    const matrix::Vector3f velocity{ekf_.getVelocity()};
    local_position.vx = velocity(0);
    local_position.vy = velocity(1);
    local_position.vz = velocity(2);
    local_position.z_deriv = ekf_.getVerticalPositionDerivative();

    const matrix::Vector3f acceleration{ekf_.getVelocityDerivative()};
    ekf_.resetVelocityDerivativeAccumulation();
    local_position.ax = acceleration(0);
    local_position.ay = acceleration(1);
    local_position.az = acceleration(2);

    local_position.xy_valid = ekf_.isLocalHorizontalPositionValid();
    local_position.v_xy_valid = ekf_.isLocalHorizontalPositionValid();
    // 与 PX4 v1.17 一致，垂直位置或垂直速度任一仍有效时，两类消费者均可继续
    // 使用同一 NED 输出；真正的降级状态同时由 dead_reckoning/status 发布。
    local_position.z_valid = ekf_.isLocalVerticalPositionValid() ||
                             ekf_.isLocalVerticalVelocityValid();
    local_position.v_z_valid = ekf_.isLocalVerticalVelocityValid() ||
                               ekf_.isLocalVerticalPositionValid();

    if (ekf_.global_origin_valid()) {
        local_position.ref_timestamp =
            ekf_.global_origin().getProjectionReferenceTimestamp();
        local_position.ref_lat =
            ekf_.global_origin().getProjectionReferenceLat();
        local_position.ref_lon =
            ekf_.global_origin().getProjectionReferenceLon();
        local_position.ref_alt = ekf_.getEkfGlobalOriginAltitude();
        local_position.xy_global = true;
        local_position.z_global = true;
    } else {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        local_position.ref_timestamp = 0U;
        local_position.ref_lat =
            std::numeric_limits<double>::quiet_NaN();
        local_position.ref_lon =
            std::numeric_limits<double>::quiet_NaN();
        local_position.ref_alt = nan;
        local_position.xy_global = false;
        local_position.z_global = false;
    }

    matrix::Quatf delta_q_reset{};
    ekf_.get_quat_reset(&delta_q_reset(0),
                        &local_position.heading_reset_counter);
    local_position.heading = matrix::Eulerf(ekf_.getQuaternion()).psi();
    local_position.unaided_heading = ekf_.getUnaidedYaw();
    local_position.heading_var = ekf_.getYawVar();
    local_position.delta_heading = matrix::Eulerf(delta_q_reset).psi();
    local_position.heading_good_for_control =
        ekf_.isYawFinalAlignComplete();
    local_position.tilt_var = ekf_.getTiltVariance();

    ekf_.get_ekf_lpos_accuracy(&local_position.eph, &local_position.epv);
    ekf_.get_ekf_vel_accuracy(&local_position.evh, &local_position.evv);
    ekf_.get_posD_reset(&local_position.delta_z,
                        &local_position.z_reset_counter);
    ekf_.get_velD_reset(&local_position.delta_vz,
                        &local_position.vz_reset_counter);
    ekf_.get_posNE_reset(local_position.delta_xy,
                         &local_position.xy_reset_counter);
    ekf_.get_velNE_reset(local_position.delta_vxy,
                         &local_position.vxy_reset_counter);

    // N1 已从状态和协方差方程中物理移除 Wind；因此这里只允许惯性失约束形成
    // dead-reckoning，不能把一个不存在的风状态重新映射进产品健康语义。
    local_position.dead_reckoning =
        ekf_.control_status_flags().inertial_dead_reckoning;

    ekf_.get_ekf_ctrl_limits(&local_position.vxy_max,
                             &local_position.vz_max,
                             &local_position.hagl_min,
                             &local_position.hagl_max_z,
                             &local_position.hagl_max_xy);
    const float infinity = std::numeric_limits<float>::infinity();
    if (!std::isfinite(local_position.vxy_max)) {
        local_position.vxy_max = infinity;
    }
    if (!std::isfinite(local_position.vz_max)) {
        local_position.vz_max = infinity;
    }
    if (!std::isfinite(local_position.hagl_min)) {
        local_position.hagl_min = infinity;
    }
    if (!std::isfinite(local_position.hagl_max_z)) {
        local_position.hagl_max_z = infinity;
    }
    if (!std::isfinite(local_position.hagl_max_xy)) {
        local_position.hagl_max_xy = infinity;
    }

    local_position.timestamp = hrt_absolute_time();
    (void)local_position_pub_.publish(local_position);
}

void Ekf2::publish_global_position(std::uint64_t sample_time_us) noexcept
{
    if (!ekf_.global_origin_valid() ||
        !ekf_.control_status_flags().yaw_align) {
        return;
    }

    vehicle_global_position_s global_position{};
    global_position.timestamp_sample = sample_time_us;
    const LatLonAlt lla = ekf_.getLatLonAlt();
    global_position.lat = lla.latitude_deg();
    global_position.lon = lla.longitude_deg();
    global_position.lat_lon_valid =
        ekf_.isGlobalHorizontalPositionValid();
    global_position.alt = lla.altitude();
    global_position.alt_valid = ekf_.isGlobalVerticalPositionValid();

    // Core 内部高度为 AMSL；MAVLink GLOBAL_POSITION_INT 还需要椭球高。滤波的
    // geoid_height=ellipsoid-AMSL 来自同一 GNSS 样本，避免混用不同时间基准。
    global_position.alt_ellipsoid =
        global_position.alt + geoid_height_lpf_.getState();

    float delta_down = 0.0F;
    ekf_.get_posD_reset(&delta_down,
                        &global_position.alt_reset_counter);
    global_position.delta_alt = -delta_down;
    float delta_ne[2]{};
    ekf_.get_posNE_reset(delta_ne,
                         &global_position.lat_lon_reset_counter);
    ekf_.get_ekf_gpos_accuracy(&global_position.eph,
                               &global_position.epv);
    global_position.dead_reckoning =
        ekf_.control_status_flags().inertial_dead_reckoning;
    global_position.timestamp = hrt_absolute_time();
    (void)global_position_pub_.publish(global_position);
}

void Ekf2::publish_odometry(std::uint64_t sample_time_us,
                            const imuSample &imu_sample) noexcept
{
    vehicle_odometry_s odometry{};
    odometry.timestamp_sample = imu_sample.time_us;
    odometry.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
    ekf_.getPosition().copyTo(odometry.position);
    ekf_.getQuaternion().copyTo(odometry.q);
    odometry.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
    ekf_.getVelocity().copyTo(odometry.velocity);
    ekf_.getAngularVelocityAndResetAccumulator().copyTo(
        odometry.angular_velocity);
    ekf_.getVelocityVariance().copyTo(odometry.velocity_variance);
    ekf_.getPositionVariance().copyTo(odometry.position_variance);
    ekf_.getRotVarBody().copyTo(odometry.orientation_variance);

    // PX4 的 odometry reset_counter 是各状态重置次数之和；uint8 自然回绕只用于
    // 提醒消费者发生过不连续，不承载累计安全计数。
    odometry.reset_counter = static_cast<std::uint8_t>(
        ekf_.get_quat_reset_count() + ekf_.get_velNE_reset_count() +
        ekf_.get_velD_reset_count() + ekf_.get_posNE_reset_count() +
        ekf_.get_posD_reset_count());
    odometry.quality = 0;
    odometry.timestamp = hrt_absolute_time();
    (void)sample_time_us;
    (void)odometry_pub_.publish(odometry);
}

} // namespace dima::modules::ekf2
