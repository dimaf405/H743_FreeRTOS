#pragma once

#include "CalibrationParameters.hpp"
#include "rover/CalibrationMath.hpp"
#include "rover/CalibrationFence.hpp"
#include "rover/CalibrationIdentification.hpp"
#include "rover/CalibrationResponse.hpp"
#include "rover/DifferentialDrive.hpp"
#include "rover/SegmentGuidance.hpp"
#include "calibration/SensorCalibrationAlgorithms.hpp"
#include "lifecycle/module_base.hpp"
#include "work_queue/ScheduledWorkItem.hpp"
#include "uORB/Publication.hpp"
#include "uORB/SubscriptionData.hpp"
#include "auto_calibration_request.hpp"
#include "auto_calibration_status.hpp"
#include "actuator_armed.hpp"
#include "actuator_motors.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"
#include "vehicle_attitude.hpp"
#include "vehicle_imu.hpp"
#include "vehicle_imu_status.hpp"
#include "vehicle_magnetometer.hpp"
#include "sensor_mag.hpp"
#include "sensor_gps.hpp"
#include "sensor_calibration_status.hpp"
#include "rover_motion_request.hpp"
#include "rover_control_status.hpp"
#include "vehicle_local_position.hpp"
#include "vehicle_odometry.hpp"
#include "rtk_heading_status.hpp"
#include "estimator_sensor_bias.hpp"
#include "estimator_status_flags.hpp"
#include "estimator_aid_src_gnss_yaw.hpp"
#include "estimator_aid_src_mag.hpp"

#include <cstdint>

namespace dima::modules::sensors { class VehicleMagnetometer; class VehicleImu; }
namespace dima::rover::control { class RoverDifferential; }

namespace dima::rover::modes {

class AutoMode;

// 组合校准只发布运动意图和 Commander 请求。低优先级队列负责参数/文本及
// 固定内存拟合；100 Hz 差速层和 PWM 后端独立执行失鲜与限幅保护。
class AutoCalibrationMode final : public dima::middleware::lifecycle::ModuleBase,
                                  public px4::ScheduledWorkItem {
public:
    AutoCalibrationMode(dima::platform::ArmedFlashCoordinator &armed,
        dima::modules::sensors::VehicleMagnetometer &mag,
        dima::modules::sensors::VehicleImu &imu,
        dima::rover::control::RoverDifferential &drive, AutoMode &navigation) noexcept;
    ~AutoCalibrationMode() override;
    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override { return module_state_; }

private:
    using Status = auto_calibration_status_s;
    using Stats3 = dima::lib::sensors::calibration::RunningStats3;
    static constexpr std::uint32_t kIntervalUs = 20000U;
    static constexpr float kPi = 3.14159265358979323846F;
    static constexpr float kRadians = kPi / 180.0F;
    // 直线期望长度改为内部规划尺度，保持原默认 12 m；实际长度仍按固定圆的
    // 剩余工作空间缩短，不能将它当作安全半径或绕过各阶段/会话时间预算。
    static constexpr float kPreferredStraightDistanceM = 12.0F;
    struct Config {
        float throttle{}, steering{}, track{}, radius{}, stop_distance{};
        float entry_cruise{}, fallback_speed{}, motor_maximum{};
        float board_offset[3]{};
        std::int32_t mag_rotation{}, mag_id{}, gps_control{};
        float mag_offset[3]{}, mag_scale[3]{};
        float mag_rate{};
    };
    struct TuningConfig {
        float inner[4]{};
        float speed_limit{}, speed_threshold{}, rate_limit{}, rate_threshold{};
        float acceleration{}, deceleration{}, rate_acceleration{}, rate_deceleration{};
        float jerk{}, speed_reduction{}, heading_p{}, acceptance{};
        dima::lib::rover::PurePursuitConfig pursuit{};
        dima::lib::rover::DrivingStateConfig driving{};
    };
    void Run() override;
    void update_inputs() noexcept;
    bool read_config() noexcept;
    bool motion_configuration_valid() const noexcept;
    void capture_fence(std::uint64_t now) noexcept;
    void update_fence(std::uint64_t now) noexcept;
    dima::lib::rover::calibration::CircleFenceResult fence_result(std::uint64_t now) const noexcept;
    bool prepare_straight(std::uint64_t now) noexcept;
    void report_status(std::uint64_t now) noexcept;
    void service_motion_authorization(std::uint64_t now) noexcept;
    bool read_tuning_config() noexcept;
    void start_tuning(std::uint64_t now) noexcept;
    bool begin_imu_bias(std::uint64_t now) noexcept;
    bool step_imu_bias(std::uint64_t now) noexcept;
    bool imu_bias_confirmed(std::uint64_t now) const noexcept;
    bool imu_bias_residual_valid(std::uint64_t now) const noexcept;
    void start_response_profile(std::uint64_t now) noexcept;
    bool step_response_profile(std::uint64_t now) noexcept;
    void run_response_profile(std::uint64_t now) noexcept;
    void finish_response_window(bool rate) noexcept;
    void reset_response_tail() noexcept;
    void finish_slew_window(bool rate) noexcept;
    bool slew_evidence_valid() const noexcept;
    bool select_slew_candidate(std::uint64_t now) noexcept;
    bool finish_slew_trial(std::uint64_t now) noexcept;
    bool calculate_runtime_candidates(std::uint64_t now) noexcept;
    bool begin_runtime_transaction(std::uint64_t now) noexcept;
    void begin_identification(std::uint64_t now) noexcept;
    bool runtime_frontend_confirmed() const noexcept;
    bool revise_runtime_candidates() noexcept;
    void advance_cohort_validation(std::uint64_t now) noexcept;
    void start_path_trials(std::uint64_t now) noexcept;
    bool prepare_navigation_candidate() noexcept;
    void validate_driving(std::uint64_t now) noexcept;
    bool apply_path_candidate(std::uint64_t now, unsigned trial) noexcept;
    bool feedback_unmasked() const noexcept;
    bool step_tuning(std::uint64_t now) noexcept;
    bool reset_identification() noexcept;
    void identify_speed(std::uint64_t now) noexcept;
    void identify_rate(std::uint64_t now) noexcept;
    bool calculate_inner_gains() noexcept;
    bool tuning_feedback(std::uint64_t now) const noexcept;
    bool tuning_estimator_valid(std::uint64_t now) const noexcept;
    bool take_tuning_sample(std::uint64_t now, bool rate) noexcept;
    void start_validation(std::uint64_t now) noexcept;
    void validate_inner(std::uint64_t now, bool rate) noexcept;
    void validate_heading(std::uint64_t now) noexcept;
    void validate_path(std::uint64_t now) noexcept;
    bool prepare_path(std::uint64_t now) noexcept;
    bool begin_gain_transaction(std::uint64_t now, std::uint8_t group) noexcept;
    void poll_gain_transaction(std::uint64_t now) noexcept;
    bool gain_frontend_confirmed() const noexcept;
    void after_gain_saved(std::uint64_t now) noexcept;
    void advance_path_trial(std::uint64_t now) noexcept;
    void fail_tuning(std::uint8_t reason, std::uint64_t now) noexcept;
    void end_validation_motion(std::uint64_t now) noexcept;
    float body_yaw() const noexcept;
    bool safety_fresh(std::uint64_t now) const noexcept;
    bool rtk_quality(std::uint64_t now) const noexcept;
    bool rtk_yaw_fused(std::uint64_t now) const noexcept;
    bool mag_path_ready(std::uint64_t now) const noexcept;
    bool dynamics_pending() const noexcept;
    bool imu_quality(std::uint64_t now) const noexcept;
    bool motion_envelope(std::uint64_t now) const noexcept;
    bool is_motion_state() const noexcept;
    bool stopped() const noexcept;
    float ground_speed() const noexcept;
    float yaw_rate() const noexcept;
    bool new_heading_epoch() noexcept;
    void update_acceleration(std::uint64_t now) noexcept;
    void begin(std::uint64_t now) noexcept;
    void step(std::uint64_t now) noexcept;
    void transition(std::uint8_t next, std::uint64_t now) noexcept;
    void terminate(std::uint8_t reason, bool cancelled, std::uint64_t now) noexcept;
    void finish(std::uint64_t now) noexcept;
    void request(std::uint8_t action, std::uint64_t now) noexcept;
    bool publish(std::uint64_t now) noexcept;
    void collect_baseline(std::uint64_t now) noexcept;
    void run_straight(std::uint64_t now, bool returning) noexcept;
    void run_turn(std::uint64_t now, int direction) noexcept;
    bool finish_rtk() noexcept;
    bool finish_dynamics() noexcept;
    void collect_mag(std::uint64_t now, unsigned direction) noexcept;
    bool finish_mag(bool bootstrap) noexcept;
    void finish_movement(std::uint64_t now) noexcept;
    float mag_residual(std::uint64_t now) const noexcept;
    bool begin_rtk_transaction(std::uint64_t now) noexcept;
    bool begin_dynamics_transaction(std::uint64_t now) noexcept;
    bool begin_mag_transaction(std::uint64_t now, bool restore) noexcept;
    void poll_transaction(std::uint64_t now) noexcept;
    bool transaction_frontend_confirmed(std::uint64_t now) const noexcept;
    static bool fresh(std::uint64_t timestamp, std::uint64_t now, std::uint64_t limit) noexcept;

    dima::platform::ArmedFlashCoordinator &armed_;
    dima::modules::sensors::VehicleMagnetometer &mag_frontend_;
    dima::modules::sensors::VehicleImu &imu_frontend_;
    dima::rover::control::RoverDifferential &drive_;
    AutoMode &navigation_;
    CalibrationParameters transaction_;
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub_{ORB_ID(vehicle_status)};
    uORB::SubscriptionData<vehicle_control_mode_s> control_sub_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<actuator_armed_s> armed_sub_{ORB_ID(actuator_armed)};
    uORB::SubscriptionData<sensor_gps_s> gps_sub_{ORB_ID(sensor_gps)};
    uORB::SubscriptionData<rtk_heading_status_s> rtk_sub_{ORB_ID(rtk_heading_status)};
    uORB::SubscriptionData<vehicle_imu_s> imu_sub_{ORB_ID(vehicle_imu)};
    uORB::SubscriptionData<vehicle_imu_status_s> imu_status_sub_{ORB_ID(vehicle_imu_status)};
    uORB::SubscriptionData<vehicle_attitude_s> attitude_sub_{ORB_ID(vehicle_attitude)};
    uORB::SubscriptionData<sensor_mag_s> raw_mag_sub_{ORB_ID(sensor_mag)};
    uORB::SubscriptionData<vehicle_magnetometer_s> mag_sub_{ORB_ID(vehicle_magnetometer)};
    uORB::SubscriptionData<actuator_motors_s> motors_sub_{ORB_ID(actuator_motors)};
    uORB::SubscriptionData<rover_control_status_s> control_feedback_sub_{ORB_ID(rover_control_status)};
    uORB::SubscriptionData<vehicle_local_position_s> position_sub_{ORB_ID(vehicle_local_position)};
    uORB::SubscriptionData<vehicle_odometry_s> odometry_sub_{ORB_ID(vehicle_odometry)};
    uORB::SubscriptionData<sensor_calibration_status_s> level_sub_{ORB_ID(sensor_calibration_status)};
    uORB::SubscriptionData<estimator_sensor_bias_s> bias_sub_{ORB_ID(estimator_sensor_bias)};
    uORB::SubscriptionData<estimator_status_flags_s> flags_sub_{ORB_ID(estimator_status_flags)};
    uORB::SubscriptionData<estimator_aid_source1d_s> yaw_aid_sub_{ORB_ID(estimator_aid_src_gnss_yaw)};
    uORB::SubscriptionData<estimator_aid_source3d_s> mag_aid_sub_{ORB_ID(estimator_aid_src_mag)};
    uORB::Publication<auto_calibration_status_s> status_pub_{ORB_ID(auto_calibration_status)};
    uORB::Publication<auto_calibration_request_s> request_pub_{ORB_ID(auto_calibration_request)};
    uORB::Publication<rover_motion_request_s> motion_pub_{ORB_ID(rover_motion_request)};

    Config config_{};
    dima::lib::rover::calibration::CircleFence fence_{};
    Status status_{};
    TuningConfig tuning_config_{};
    dima::lib::rover::DifferentialDriveConfig tuning_motor_{};
    dima::lib::rover::calibration::MotorResponseProfile response_speed_{}, response_rate_{}, response_full_{};
    dima::lib::rover::calibration::ResponseStatistics noise_speed_{}, noise_rate_{}, response_tail_{};
    struct ResponseSample { std::uint64_t timestamp{}; float value{}; bool usable{}; bool motor_slew{}; };
    ResponseSample response_samples_[256]{};
    std::size_t response_sample_count_{};
    float response_rate_lower_[4]{}, response_noise_[2]{}, response_initial_{};
    float profile_forward_gain_{}, profile_input_ceiling_{}, profile_rate_gain_{};
    float response_settle_s_{};
    float runtime_ff_speed_{}, runtime_ff_yaw_{};
    std::uint64_t response_phase_started_{}, response_last_sample_{}, response_stop_started_{};
    std::uint64_t response_tail_timestamp_{}, response_profile_deadline_{};
    std::uint64_t tuning_reference_{};
    std::uint8_t profile_motion_{}, profile_level_{}, tuning_xy_reset_{}, tuning_vxy_reset_{}, tuning_yaw_reset_{}, tuning_odom_reset_{};
    bool profile_started_{}, profile_braking_{}, profile_noise_ready_{}, runtime_cohort_{}, motor_candidate_changed_{};
    bool runtime_fully_observed_{}, path_gain_sensitive_{};
    bool motor_reprofiled_{};
    dima::lib::rover::calibration::MotorResponseCandidate motor_profile_reference_{};
    bool motor_profile_verified_{}, profile_speed_only_{}, slew_probe_eligible_{}, slew_reprofile_pending_{};
    bool motor_slew_verified_{}, motor_slew_changed_{};
    enum class SlewTrial : std::uint8_t { Baseline, Candidate, Restore, Complete };
    struct SlewEvidence {
        // 普通正向/反向/满输出各七个窗口；只存比较指标，不再复制原始时序。
        float target[21]{}, input[21]{}, error[21]{};
        float active_min[2]{}, active_max[2]{};
        float peak_overshoot{}, noise_ratio{}, noise{};
        std::uint32_t observed_mask{}, failed_mask{};
        std::uint8_t active_directions{};
    };
    SlewEvidence slew_evidence_{}, slew_baseline_{};
    SlewTrial slew_trial_{SlewTrial::Baseline};
    float slew_original_{}, slew_input_ceiling_[2]{};
    dima::lib::rover::calibration::FirstOrderDelayIdentifier identifiers_[4]{};
    dima::lib::rover::calibration::FirstOrderModel identified_models_[4]{};
    dima::lib::rover::calibration::StepResponseValidator validator_{};
    dima::lib::rover::PurePursuit tuning_pursuit_{};
    dima::lib::rover::HeadingController tuning_heading_{};
    dima::lib::rover::DrivingStateMachine tuning_driving_{};
    dima::lib::rover::Position2f path_points_[5]{};
    dima::lib::rover::Position2f path_entry_{};
    bool path_entry_captured_{};
    Stats3 path_errors_{};
    float gains_[4]{}, loop_time_[2]{}, physical_speed_{}, physical_rate_{};
    float tuning_noise_[2]{};
    float imu_bias_scale_[3]{}, imu_bias_old_norm_[2]{}, imu_bias_noise_[2]{};
    bool imu_bias_accel_{}, imu_bias_gyro_{}, imu_bias_finalizing_{}, imu_bias_attempted_{};
    float exercise_heading_{}, exercise_start_yaw_{}, exercise_duration_{};
    float identification_input_origin_{}, identification_output_origin_{};
    float identification_applied_origin_{};
    double shaping_uu_[4]{}, shaping_up_[4]{}, shaping_pp_[4]{};
    float best_path_error_{};
    struct PathCandidate { float gain{}, jerk{}, reduction{}; };
    PathCandidate path_candidates_[3]{};
    unsigned best_path_trial_{};
    float path_duration_s_[3]{};
    std::uint32_t path_sample_counts_[3]{};
    std::uint32_t path_observed_fields_[3]{}, path_jerk_samples_{}, path_reduction_samples_{};
    std::uint64_t navigation_phase_started_{};
    std::uint64_t path_observation_sample_{}, path_measured_started_{};
    std::uint8_t navigation_phase_{}, navigation_directions_{};
    float path_scores_[3]{}, path_yaw_rate_previous_{};
    std::uint64_t tuning_sample_{}, exercise_started_{}, path_sample_{}, path_leg_started_{};
    std::uint8_t exercise_{}, path_trial_{}, path_index_{};
    std::uint8_t path_count_{4U};
    std::uint8_t turn_resume_state_{Status::STATE_STRAIGHT_BACK};
    std::uint32_t path_observable_samples_{}, path_crossings_{};
    std::uint32_t validation_ramp_samples_{};
    std::uint8_t runtime_observed_mask_{};
    std::uint64_t validation_setpoint_time_{};
    float validation_previous_setpoint_{};
    bool validation_previous_unmasked_{};
    bool exercise_running_{}, validation_passed_{}, path_arrived_{}, tuning_started_{};
    dima::lib::rover::calibration::CircularMean heading_mean_[2]{};
    dima::lib::rover::calibration::SpeedFit speed_fit_{};
    Stats3 yaw_fit_[2]{};
    Stats3 bias_fit_[2]{};
    Stats3 bootstrap_fit_[2]{};
    float baseline_samples_[120]{};
    std::size_t baseline_count_{};
    double leg_lat_{}, leg_lon_{};
    float leg_distance_{};
    float leg_heading_{}, turn_heading_{}, last_turn_heading_{}, turn_integral_{}, turn_remaining_{};
    float longitudinal_{}, steering_{};
    float candidate_mag_[3]{};
    float bootstrap_offset_[3]{};
    std::uint32_t mag_device_id_{}, gps_device_id_{}, imu_device_id_{};
    std::uint32_t coverage_[2]{};
    std::uint64_t last_epoch_{}, last_mag_sample_{}, last_bias_sample_{};
    std::uint64_t acceleration_epoch_{}, angular_acceleration_sample_{};
    std::uint64_t heading_progress_epoch_{}, last_fit_velocity_epoch_{};
    std::uint64_t last_alignment_mag_sample_{}, matched_mag_sample_{};
    std::uint64_t rtk_progress_time_{}, steady_since_{};
    std::uint64_t first_circle_at_{};
    std::uint64_t last_report_{};
    std::uint64_t last_resume_request_{};
    unsigned steady_level_{3U};
    float previous_velocity_[2]{}, previous_yaw_rate_{};
    float filtered_acceleration_[2]{}, filtered_angular_acceleration_{};
    bool linear_acceleration_ok_{true}, angular_acceleration_ok_{true};
    std::uint64_t last_run_{}, state_started_{}, session_started_{}, motion_started_{};
    std::uint64_t stable_since_{}, mag_stable_since_{}, level_request_time_{}, arm_started_{};
    std::uint32_t sequence_{}, session_id_{};
    std::uint32_t expected_set_count_{};
    std::uint8_t transaction_stage_{};
    bool selected_{}, pending_termination_{}, cancel_requested_{}, bootstrap_applied_{}, mag_ready_{};
    bool config_valid_{};
    bool previously_armed_{}, turn_started_{}, turn_braking_{};
    dima::middleware::lifecycle::ModuleState module_state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::rover::modes
