#pragma once

#include "mission/MissionService.hpp"
#include "rover/PurePursuit.hpp"
#include "rover/RoverControl.hpp"

#include "parameter_update.hpp"
#include "rover_motion_request.hpp"
#include "rover_navigation_status.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_local_position.hpp"
#include "vehicle_odometry.hpp"
#include "vehicle_status.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "uORB/Publication.hpp"
#include "uORB/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include "geo/geo.h"

#include <cstdint>

namespace dima::rover::modes {

/**
 * 50 Hz Rover AUTO 外环。
 *
 * Pure Pursuit 产生 yaw setpoint，Heading P 产生 yaw-rate setpoint；速度与
 * yaw-rate 的 PI 内环由 100 Hz RoverDifferential 执行。本模块只发布物理量
 * RoverMotionRequest，禁止直接访问 DifferentialDrive、MotorOutput 或 PWM。
 */
class AutoMode final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    explicit AutoMode(
        dima::modules::mission::MissionService &mission_service) noexcept;
    ~AutoMode() override;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

private:
    static constexpr std::uint32_t kRunIntervalUs = 20000U;
    static constexpr std::uint64_t kMaximumControlDtUs = 50000ULL;
    static constexpr std::uint64_t kSafetyTopicTimeoutUs = 750000ULL;
    static constexpr std::uint64_t kEstimatorTimeoutUs = 200000ULL;
    static constexpr std::uint64_t kMissionStatusCacheUs = 200000ULL;

    struct Config {
        dima::lib::rover::PurePursuitConfig pure_pursuit;
        dima::lib::rover::HeadingControlConfig heading;
        dima::lib::rover::DrivingStateConfig driving;
        dima::lib::rover::SpeedControlConfig speed_inner;
        dima::lib::rover::YawRateControlConfig yaw_rate_inner;
        float default_acceptance_radius_m{0.0F};
        float cruise_speed_m_s{0.0F};
        float jerk_limit_m_s3{0.0F};
        float deceleration_limit_m_s2{0.0F};
        float speed_reduction_gain{0.0F};
    };

    struct GuidanceOutput {
        float distance_to_waypoint_m{0.0F};
        float crosstrack_error_m{0.0F};
        float lookahead_distance_m{0.0F};
        float heading_error_rad{0.0F};
        float speed_setpoint_m_s{0.0F};
        float yaw_rate_setpoint_rad_s{0.0F};
        std::uint8_t control_state{
            rover_navigation_status_s::CONTROL_INACTIVE};
        std::uint8_t waypoint_state{
            rover_navigation_status_s::WAYPOINT_ACTIVE};
        bool valid{false};
    };

    void Run() override;
    bool bind_parameters() noexcept;
    void invalidate_parameter_bindings() noexcept;
    bool apply_parameter_snapshot() noexcept;
    void apply_pending_parameters(std::uint64_t now) noexcept;
    void update_subscriptions() noexcept;
    bool safety_snapshot_fresh(std::uint64_t now) const noexcept;
    bool auto_projection(std::uint8_t nav_state,
                         bool require_armed) const noexcept;
    bool estimator_healthy(std::uint64_t now) const noexcept;
    bool estimator_reset_detected() noexcept;
    bool refresh_mission_snapshot(
        std::uint64_t now,
        dima::modules::mission::MissionStatus &status) noexcept;
    bool refresh_mission_plan(
        const dima::modules::mission::MissionStatus &status) noexcept;
    bool prepare_segment(const dima::modules::mission::MissionStatus &status,
                         bool rebuild_from_vehicle) noexcept;
    bool project_item(const dima::modules::mission::MissionItem &item,
                      dima::lib::rover::Position2f &position) const noexcept;
    GuidanceOutput run_guidance(
        const dima::modules::mission::MissionStatus &mission_status,
        float dt_s) noexcept;
    bool publish_cycle(std::uint64_t now,
                       const dima::modules::mission::MissionStatus &mission,
                       const GuidanceOutput &guidance,
                       std::uint8_t failure_reason,
                       bool ready_for_auto,
                       bool estimator_is_healthy,
                       bool request_valid) noexcept;
    bool publish_invalid(std::uint64_t now,
                         const dima::modules::mission::MissionStatus &mission,
                         std::uint8_t failure_reason,
                         bool ready_for_auto,
                         bool estimator_is_healthy) noexcept;
    void reset_control_state() noexcept;
    void reset_runtime_state() noexcept;
    void enter_error(std::uint32_t event_id) noexcept;

    static bool finite(float value) noexcept;
    static bool valid_config(const Config &config) noexcept;
    static float wrap_pi(float angle) noexcept;
    static std::uint8_t control_state(
        dima::lib::rover::DrivingState state) noexcept;

    dima::modules::mission::MissionService &mission_service_;
    uORB::SubscriptionData<parameter_update_s> parameter_update_subscription_{
        ORB_ID(parameter_update)};
    uORB::SubscriptionData<vehicle_local_position_s>
        local_position_subscription_{ORB_ID(vehicle_local_position)};
    uORB::SubscriptionData<vehicle_odometry_s>
        vehicle_odometry_subscription_{ORB_ID(vehicle_odometry)};
    uORB::SubscriptionData<vehicle_control_mode_s>
        control_mode_subscription_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_subscription_{
        ORB_ID(vehicle_status)};
    uORB::Publication<rover_motion_request_s> motion_request_publication_{
        ORB_ID(rover_motion_request)};
    uORB::Publication<rover_navigation_status_s> navigation_status_publication_{
        ORB_ID(rover_navigation_status)};

    dima::ParamFloat<dima::params::PP_LOOKAHD_GAIN> lookahead_gain_{};
    dima::ParamFloat<dima::params::PP_LOOKAHD_MIN> lookahead_min_{};
    dima::ParamFloat<dima::params::PP_LOOKAHD_MAX> lookahead_max_{};
    dima::ParamFloat<dima::params::NAV_ACC_RAD> acceptance_radius_{};
    dima::ParamFloat<dima::params::RO_YAW_P> yaw_p_{};
    dima::ParamFloat<dima::params::RO_YAW_RATE_LIM> yaw_rate_limit_{};
    dima::ParamFloat<dima::params::RD_TRANS_TRN_DRV> turn_to_drive_{};
    dima::ParamFloat<dima::params::RD_TRANS_DRV_TRN> drive_to_turn_{};
    dima::ParamFloat<dima::params::RO_MAX_THR_SPEED> maximum_speed_{};
    dima::ParamFloat<dima::params::RO_SPEED_LIM> speed_limit_{};
    dima::ParamFloat<dima::params::RO_DECEL_LIM> deceleration_limit_{};
    dima::ParamFloat<dima::params::RO_JERK_LIM> jerk_limit_{};
    dima::ParamFloat<dima::params::RO_SPEED_RED> speed_reduction_{};
    dima::ParamFloat<dima::params::RO_SPEED_TH> speed_threshold_{};
    dima::ParamFloat<dima::params::RO_SPEED_P> speed_p_{};
    dima::ParamFloat<dima::params::RO_SPEED_I> speed_i_{};
    dima::ParamFloat<dima::params::RO_ACCEL_LIM> acceleration_limit_{};
    dima::ParamFloat<dima::params::RO_YAW_RATE_P> yaw_rate_p_{};
    dima::ParamFloat<dima::params::RO_YAW_RATE_I> yaw_rate_i_{};
    dima::ParamFloat<dima::params::RO_YAW_RATE_CORR> yaw_rate_correction_{};
    dima::ParamFloat<dima::params::RO_YAW_ACCEL_LIM> yaw_acceleration_{};
    dima::ParamFloat<dima::params::RO_YAW_DECEL_LIM> yaw_deceleration_{};
    dima::ParamFloat<dima::params::RO_YAW_RATE_TH> yaw_rate_threshold_{};
    dima::ParamFloat<dima::params::RD_WHEEL_TRACK> wheel_track_{};

    dima::lib::rover::PurePursuit pure_pursuit_{};
    dima::lib::rover::HeadingController heading_controller_{};
    dima::lib::rover::DrivingStateMachine driving_state_{};
    Config config_{};
    MapProjection projection_{};
    dima::modules::mission::MissionPlan mission_plan_{};
    dima::modules::mission::MissionStatus cached_mission_status_{};
    dima::lib::rover::Position2f segment_start_{};
    dima::lib::rover::Position2f segment_target_{};
    float segment_acceptance_radius_m_{0.0F};
    float segment_arrival_speed_m_s_{0.0F};
    vehicle_local_position_s local_position_{};
    vehicle_odometry_s vehicle_odometry_{};
    vehicle_control_mode_s control_mode_{};
    vehicle_status_s vehicle_status_{};
    std::uint64_t last_run_us_{0U};
    std::uint64_t last_estimator_timestamp_us_{0U};
    std::uint64_t last_estimator_sample_us_{0U};
    std::uint64_t last_odometry_timestamp_us_{0U};
    std::uint64_t last_odometry_sample_us_{0U};
    std::uint64_t last_mission_status_us_{0U};
    std::uint64_t projection_reference_timestamp_{0U};
    std::uint32_t segment_mission_id_{0U};
    std::uint32_t request_sequence_{0U};
    std::uint16_t segment_sequence_{0U};
    std::uint8_t xy_reset_counter_{0U};
    std::uint8_t velocity_reset_counter_{0U};
    std::uint8_t heading_reset_counter_{0U};
    std::uint8_t odometry_reset_counter_{0U};
    bool have_local_position_{false};
    bool have_odometry_{false};
    bool have_control_mode_{false};
    bool have_vehicle_status_{false};
    bool have_estimator_baseline_{false};
    bool segment_valid_{false};
    bool segment_final_waypoint_{false};
    bool waypoint_advance_pending_{false};
    bool mission_complete_pending_{false};
    bool rebuild_segment_from_vehicle_{true};
    bool mission_plan_valid_{false};
    bool have_mission_status_{false};
    bool parameters_valid_{false};
    bool parameter_update_pending_{false};
    bool auto_was_active_{false};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::rover::modes
