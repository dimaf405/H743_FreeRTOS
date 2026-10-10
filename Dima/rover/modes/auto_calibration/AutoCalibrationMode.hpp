#pragma once

#include "CalibrationParameters.hpp"
#include "magnetometer/MagMotorOutputHistory.hpp"
#include "rover/CalibrationMath.hpp"
#include "rover/CalibrationFence.hpp"
#include "rover/CalibrationBraking.hpp"
#include "rover/CalibrationIdentification.hpp"
#include "rover/CalibrationResponse.hpp"
#include "rover/SegmentGuidance.hpp"
#include "calibration/SensorCalibrationAlgorithms.hpp"
#include "lifecycle/module_base.hpp"
#include "work_queue/ScheduledWorkItem.hpp"
#include "uORB/Publication.hpp"
#include "uORB/SubscriptionData.hpp"
#include "auto_calibration_request.hpp"
#include "auto_calibration_status.hpp"
#include "actuator_armed.hpp"
#include "actuator_output_status.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"
#include "vehicle_attitude.hpp"
#include "vehicle_imu.hpp"
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

// 调度合同权威正文（状态压平、唯一跳转点、事务机、组调度器、阶段 helper
// 约定、Arm/停波/600s/180s/300ms/0.65/0.90/yaw0.6/围栏 can_stop/provisional
// 不越权/FAILURE_STORAGE 不降级/前端代次确认等安全不变量）见
// Dima/rover/modes/README.md「AutoCalibrationMode 调度合同」一节；
// 本头文件只保留声明与定义，不承载合同注释。

enum class PhaseSubstate : std::uint8_t {
    WaitArm,
    Running,
    TurnAround,
    Return,
    Braking,
    WaitStop,
    Evaluate,
};

enum class TransactionKind : std::uint8_t {
    None, Level, Rtk, Dynamics, Magnetic, MagneticMotor, Imu, Runtime,
    Gains
};

enum class StepResult : std::uint8_t {
    Busy,
    Advance,
    WantTurn,
    WantBrake,
    WantReturn,
    WantStop,
    Failed,
    Abort,
};

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
    dima::middleware::lifecycle::ModuleState state() const override;

private:
    using Status = auto_calibration_status_s;
    using Stats3 = dima::lib::sensors::calibration::RunningStats3;
    static constexpr std::uint32_t kIntervalUs = 10000U;
    static constexpr float kStoppedSpeedMps = 0.08F; // 已有运动/停止边界，统一单位为m/s。
    static constexpr float kPi = 3.14159265358979323846F;
    static constexpr float kRadians = kPi / 180.0F;
    struct Config {
        float track{}, radius{}, straight_distance{}, deceleration{}, path_error{};
        float entry_cruise{}, motor_maximum{}, motor_reversal_delay_s{}, motor_slew_rate{};
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

    enum class StopIntent : std::uint8_t {
        None, Braking, RtkCommit
    };

    struct SessionController {
        PhaseSubstate substate{PhaseSubstate::Evaluate};
        TransactionKind kind{TransactionKind::None};
        std::uint32_t transaction_stages{};
        std::uint64_t substate_started{};
        StopIntent stop_intent{StopIntent::None};
        PhaseSubstate resume_substate{PhaseSubstate::Running};
        bool straight_outward{true};
        std::int8_t turn_direction{1};
        std::uint8_t turn_round{1};
        bool force_rollback{};
        std::uint8_t abort_reason{};
        std::uint8_t abort_group{};
    };

    void Run() override;
    void update_inputs() noexcept;

    void begin(std::uint64_t now) noexcept;
    void reset_session_dynamics() noexcept;
    void reset_session_magnetic() noexcept;
    void step_wait_arm(std::uint64_t now) noexcept;
    void step(std::uint64_t now) noexcept;
    void enter_phase(std::uint8_t target, std::uint64_t now,
                     PhaseSubstate initial) noexcept;
    void enter_substate(PhaseSubstate substate, std::uint64_t now) noexcept;
    void step_preflight(std::uint64_t now) noexcept;
    void step_straight(std::uint64_t now) noexcept;
    void step_turn(std::uint64_t now) noexcept;
    void step_magnetic(std::uint64_t now) noexcept;
    void step_profile(std::uint64_t now) noexcept;
    void step_identification(std::uint64_t now) noexcept;
    void step_validation(std::uint64_t now) noexcept;
    void step_finalize(std::uint64_t now) noexcept;
    void apply_straight_envelope(std::uint64_t now) noexcept;
    bool dispatch_step_result(StepResult step, std::uint64_t now) noexcept;
    bool apply_transaction(TransactionKind kind, std::uint64_t now, bool provisional = false,
        std::uint32_t stages = 0U) noexcept;
    void poll_active_transaction(std::uint64_t now) noexcept;
    bool transaction_frontend_confirmed(std::uint64_t now) const noexcept;
    bool transaction_settled(std::uint64_t now, bool applied) noexcept;
    void on_transaction_provisional(std::uint64_t now, bool applied) noexcept;
    void on_transaction_finished(std::uint64_t now) noexcept;
    bool wait_stop_settled(std::uint64_t now) noexcept;
    void handle_group_abort(std::uint64_t now) noexcept;
    bool abort_rollback_complete(std::uint64_t now) noexcept;
    void start_profile_chain(std::uint64_t now) noexcept;

    void resume_profile_chain(std::uint64_t now) noexcept;
    void continue_profile_entry(std::uint64_t now) noexcept;
    std::uint8_t take_helper_failure(std::uint8_t fallback) noexcept;
    void advance_cohort_validation(std::uint64_t now) noexcept;
    void after_gain_validated(std::uint64_t now) noexcept;
    void enter_finalize(std::uint64_t now, bool force_rollback) noexcept;
    void terminate(std::uint8_t reason, bool cancelled, std::uint64_t now) noexcept;
    bool session_failure_requires_rollback(std::uint8_t reason) const noexcept;
    void finish(std::uint64_t now) noexcept;
    bool publish(std::uint64_t now) noexcept;
    void report_status(std::uint64_t now) noexcept;
    void capture_motion_failure(const char *cause, std::uint64_t now) noexcept;
    void report_motion_failure() const noexcept;
    bool maintenance_ready() const noexcept;
    bool start_motion(std::uint64_t now) noexcept;
    bool is_motion_state() const noexcept;
    bool fail_flag(std::uint8_t reason) noexcept;
    StepResult fail_step(std::uint8_t reason) noexcept;
    StepResult abort_step(std::uint8_t reason) noexcept;
    static std::uint32_t group_dependents(std::uint8_t bit) noexcept;
    bool group_entry_available(unsigned row) const noexcept;
    void skip_group(unsigned row, const char *cause) noexcept;
    void skip_tuning_chain() noexcept;
    bool advance_group_scheduler(unsigned row, std::uint64_t now) noexcept;
    void fail_tuning(std::uint8_t reason, std::uint64_t now) noexcept;

    static bool fresh(std::uint64_t timestamp, std::uint64_t now, std::uint64_t limit) noexcept;
    bool read_config() noexcept;
    bool safety_fresh(std::uint64_t now) const noexcept;
    bool imu_quality(std::uint64_t now) const noexcept;
    bool rtk_quality(std::uint64_t now) const noexcept;
    bool rtk_yaw_fused(std::uint64_t now) const noexcept;
    bool dynamics_pending() const noexcept;
    bool motion_quality_failure(std::uint64_t now) const noexcept;
    bool stopped() const noexcept;
    float ground_speed() const noexcept;
    std::uint64_t velocity_epoch_us() const noexcept;
    float control_interval_s(std::uint64_t now) const noexcept;
    float yaw_rate() const noexcept;
    bool new_heading_epoch() noexcept;

    bool motion_configuration_valid() const noexcept;
    void capture_fence(std::uint64_t now) noexcept;
    void update_fence(std::uint64_t now) noexcept;
    dima::lib::rover::calibration::CircleFenceResult fence_result(std::uint64_t now) const noexcept;
    bool prepare_straight(std::uint64_t now) noexcept;
    float sensor_lever_arm() const noexcept;
    void reset_rotation_lever_window() noexcept;
    void finalize_rotation_lever(int direction) noexcept;
    StepResult baseline_collect(std::uint64_t now) noexcept;
    StepResult straight_leg(std::uint64_t now) noexcept;
    StepResult observe_straight(std::uint64_t now, bool full_output_trial) noexcept;
    StepResult speed_model_leg(std::uint64_t now) noexcept;
    StepResult straight_turnaround(std::uint64_t now) noexcept;
    bool finish_rtk() noexcept;
    StepResult request_turn_rate(std::uint64_t now, float target_rate) noexcept;
    void stop_turn_rate_request() noexcept;
    bool finish_dynamics() noexcept;
    StepResult turn_spin(std::uint64_t now) noexcept;
    float braking_deceleration() const noexcept;
    float braking_distance(float speed) const noexcept;
    bool braking_model_ready() const noexcept;
    bool braking_stop_confirmed(std::uint64_t now) const noexcept;
    bool braking_output_zero(std::uint64_t now) const noexcept;
    bool begin_braking_observation(std::uint64_t now) noexcept;
    StepResult braking_step(std::uint64_t now) noexcept;
    bool complete_braking_observation(std::uint64_t now) noexcept;
    void fail_braking_group(std::uint64_t now) noexcept;
    void report_braking_probe(std::uint64_t now) const noexcept;
    void request_measured_braking(std::uint64_t now, std::uint64_t epoch) noexcept;
    bool begin_deceleration_transaction(std::uint64_t now) noexcept;
    bool begin_rtk_transaction(std::uint64_t now) noexcept;
    bool begin_dynamics_transaction(std::uint64_t now) noexcept;
    float distance_to_start() const noexcept;
    float heading_to_start() const noexcept;
    float straight_speed_limit(float remaining) const noexcept;
    void return_prepare(std::uint64_t now) noexcept;
    StepResult return_step(std::uint64_t now) noexcept;
    bool mag_path_ready(std::uint64_t now) const noexcept;
    bool mag_scale_valid() const noexcept;
    bool mag_rotation(float (&rotation)[9]) const noexcept;
    void collect_mag(std::uint64_t now, unsigned direction) noexcept;
    bool finish_mag(bool bootstrap) noexcept;
    float mag_residual(std::uint64_t now) const noexcept;
    void sample_mag_throttle(std::uint64_t now) noexcept;
    bool commit_mag_throttle(std::uint64_t now) noexcept;
    bool begin_mag_transaction(std::uint64_t now, bool restore) noexcept;
    bool begin_imu_bias(std::uint64_t now) noexcept;
    bool imu_bias_confirmed(std::uint64_t now) const noexcept;
    bool imu_bias_residual_valid(std::uint64_t now) const noexcept;
    bool read_tuning_config() noexcept;
    bool feedback_unmasked(bool tracking_valid_at_limit = false) const noexcept;
    StepResult profile_prepare() noexcept;
    StepResult profile_run(std::uint64_t now) noexcept;
    void begin_profile_window(std::uint64_t now, bool rate) noexcept;
    void record_response_sample(std::uint64_t timestamp, float value, bool usable) noexcept;
    void reset_response_tail() noexcept;
    void finish_response_window(bool rate) noexcept;
    bool calculate_runtime_candidates(std::uint64_t now) noexcept;
    bool begin_runtime_transaction(std::uint64_t now) noexcept;
    bool runtime_frontend_confirmed() const noexcept;
    bool revise_runtime_candidates() noexcept;
    bool begin_identification() noexcept;
    StepResult identification_run(std::uint64_t now) noexcept;
    bool calculate_inner_gains() noexcept;
    float body_yaw() const noexcept;
    bool tuning_estimator_valid(std::uint64_t now) const noexcept;
    bool tuning_feedback(std::uint64_t now) const noexcept;
    bool take_tuning_sample(std::uint64_t now, bool rate) noexcept;
    bool begin_gain_transaction(std::uint64_t now, std::uint8_t group) noexcept;
    bool gain_frontend_confirmed() const noexcept;
    bool gain_controller_ready() const noexcept;
    bool start_validation(std::uint64_t now) noexcept;
    StepResult validation_inner(std::uint64_t now, bool rate) noexcept;
    StepResult validation_heading(std::uint64_t now) noexcept;
    bool prepare_navigation_candidate() noexcept;
    StepResult validation_driving(std::uint64_t now) noexcept;
    bool start_path_validation(std::uint64_t now) noexcept;
    StepResult validation_path(std::uint64_t now) noexcept;
    StepResult finish_path_validation(std::uint64_t now) noexcept;
    bool prepare_path(std::uint64_t now) noexcept;
    bool start_navigation_item(std::uint64_t now) noexcept;
    StepResult apply_navigation_trial(std::uint64_t now) noexcept;
    StepResult end_navigation_item(std::uint64_t now, bool confirmed) noexcept;
    StepResult finish_navigation_tuning(std::uint64_t now) noexcept;
    bool reset_navigation_trial(std::uint64_t now) noexcept;
    void observe_navigation(const dima::lib::rover::SegmentGuidanceOutput &guidance,
        std::uint64_t now) noexcept;


    dima::platform::ArmedFlashCoordinator &armed_;
    dima::modules::sensors::VehicleMagnetometer &mag_frontend_;
    dima::modules::sensors::VehicleImu &imu_frontend_;
    dima::rover::control::RoverDifferential &drive_;
    AutoMode &navigation_;
    CalibrationParameters transaction_;
    dima::modules::sensors::MagMotorOutputHistory motor_history_{};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub_{ORB_ID(vehicle_status)};
    uORB::SubscriptionData<vehicle_control_mode_s> control_sub_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<actuator_armed_s> armed_sub_{ORB_ID(actuator_armed)};
    uORB::SubscriptionData<sensor_gps_s> gps_sub_{ORB_ID(sensor_gps)};
    uORB::SubscriptionData<rtk_heading_status_s> rtk_sub_{ORB_ID(rtk_heading_status)};
    uORB::SubscriptionData<vehicle_imu_s> imu_sub_{ORB_ID(vehicle_imu)};
    uORB::SubscriptionData<vehicle_attitude_s> attitude_sub_{ORB_ID(vehicle_attitude)};
    uORB::SubscriptionData<sensor_mag_s> raw_mag_sub_{ORB_ID(sensor_mag)};
    uORB::SubscriptionData<vehicle_magnetometer_s> mag_sub_{ORB_ID(vehicle_magnetometer)};
    uORB::SubscriptionData<actuator_output_status_s> output_sub_{ORB_ID(actuator_output_status)};
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
    SessionController session_{};
    // 只锁存首个运动失败点，停车后的数据不能覆盖根因；每行小于日志正文容量。
    char motion_failure_text_[3][120]{};
    TuningConfig tuning_config_{};
    float tuning_motor_slew_{}, tuning_arm_ramp_{};
    dima::lib::rover::calibration::MotorResponseProfile response_speed_{}, response_rate_{};
    dima::lib::rover::calibration::ResponseStatistics response_tail_{};
    struct ResponseSample { std::uint64_t timestamp{}; float value{}; bool usable{}; };
    ResponseSample response_samples_[256]{};
    std::size_t response_sample_count_{};
    float response_rate_sum_[4]{}, response_rate_duration_[4]{}, response_initial_{};
    float profile_input_floor_{}, profile_input_ceiling_{};
    std::uint64_t response_stop_started_{};
    std::uint64_t response_tail_timestamp_{}, profile_motion_deadline_{}, profile_phase_deadline_{};
    std::uint64_t profile_stable_since_{}, response_sample_interval_{};
    std::uint64_t tuning_reference_{};
    std::uint8_t profile_motion_{}, profile_level_{}, tuning_xy_reset_{}, tuning_vxy_reset_{}, tuning_yaw_reset_{}, tuning_odom_reset_{};
    bool profile_started_{}, profile_braking_{}, runtime_cohort_{};
    bool runtime_fully_observed_{};
    // 四段依次采样，共用一套RLS工作区；每段只保存拟合结果，统一验证后才应用。
    dima::lib::rover::calibration::FirstOrderDelayIdentifier identifier_{};
    dima::lib::rover::calibration::IdentificationResult identification_results_[4]{};
    dima::lib::rover::calibration::StepResponseValidator validator_{};
    dima::lib::rover::PurePursuit tuning_pursuit_{};
    dima::lib::rover::HeadingController tuning_heading_{};
    dima::lib::rover::DrivingStateMachine tuning_driving_{};
    // 以下只表达实验角色，不承担导航运动状态；运动始终由update_segment推进。
    enum class NavigationPass : std::uint8_t { Baseline, Reference, UpperBound, Search, Confirm, Fallback, LowSpeed, Final, Restore };
    struct NavigationTuning {
        std::uint8_t item{}, direction{};
        NavigationPass pass{NavigationPass::Baseline};
        bool active{}, speed_probe{}, search_ready{}, failed{}, observable{}, unmasked{}, incomplete{};
        bool arrival_observed{}, candidate_changed{}, braking_window{}, cruise_observed{};
        bool turn_window{}, turn_completed{}, turn_failed{}, turn_stop_allowed{}, speed_bracketed{}, turn_bounded[2]{};
        float original[3]{}, accepted[3]{};
        float candidate{}, best_value{}, trial_speed{}, nominal_speed{}, maximum_speed{}, low_speed{};
        float lower{}, upper{}, side{}, heading{}, turn_angle{}, arrival_speed{}, turn_speeds[2]{};
        float measured_arrival{}, best_arrival{}, peak_error{}, peak_speed_target{}, peak_rate_target{};
        float previous_acceleration{}, previous_rate{}, previous_setpoint{}, leg_timeout_s{};
        double duration{}, error_squared{}, speed_squared{}, rate_squared{}, jerk_squared{}, jerk_duration{}, yaw_accel_squared{};
        double turn_duration{}, turn_speed_squared{};
        std::uint64_t sample{}, acceleration_epoch{}, started{};
        std::uint32_t samples{}, brake_samples{}, braking_segments{}, evidence{}, leg_sequence{};
        dima::lib::rover::Position2f center{};
        dima::lib::rover::calibration::TrialScore baseline{}, best{}, result{};
        dima::lib::rover::calibration::BoundedParameterSearch search{};
    } nav_{};
    dima::lib::rover::Position2f path_points_[5]{};
    dima::lib::rover::Position2f path_entry_{};
    bool path_entry_captured_{};
    float gains_[4]{}, loop_time_[2]{}, physical_speed_{}, physical_rate_{};
    float imu_bias_scale_[3]{};
    bool imu_bias_finalizing_{}, imu_bias_attempted_{};
    std::uint64_t imu_relock_started_{};
    float exercise_heading_{}, exercise_start_yaw_{}, exercise_duration_{};
    float identification_input_origin_{}, identification_output_origin_{};
    std::uint64_t navigation_phase_started_{};
    std::uint8_t navigation_phase_{}, navigation_directions_{};
    std::uint64_t tuning_sample_{}, exercise_started_{};
    std::uint8_t exercise_{}, path_index_{};
    std::uint8_t path_count_{4U};
    bool exercise_running_{}, validation_passed_{}, path_arrived_{}, tuning_started_{};
    bool inner_speed_ok_{};
    struct GroupStep { std::uint8_t bit; std::uint32_t depends; };
    static constexpr std::uint8_t kGroupBitInner = 6U;
    static constexpr std::uint8_t kGroupBitHeading = 7U;
    static constexpr std::uint8_t kGroupBitPath = 8U;
    static constexpr std::uint8_t kGroupBitNav = 12U;
    static constexpr std::uint8_t kGroupBitImuBias = 9U;
    static constexpr GroupStep kGroupSteps[] = {
        {2U, Status::STAGE_DECELERATION},
        {3U, 0U},
        {kGroupBitInner, Status::STAGE_SPEED | Status::STAGE_YAW},
        {kGroupBitHeading, Status::STAGE_INNER_GAINS},
        {kGroupBitPath, Status::STAGE_HEADING_GAIN | Status::STAGE_DECELERATION},
        {11U, Status::STAGE_SPEED | Status::STAGE_YAW | Status::STAGE_DECELERATION},
        {kGroupBitNav, Status::STAGE_PATH_GAIN},
        {4U, Status::STAGE_YAW},
        {13U, Status::STAGE_MAG},
        {kGroupBitImuBias, 0U},
    };
    static constexpr unsigned kGroupStepCount =
        sizeof(kGroupSteps) / sizeof(kGroupSteps[0]);
    static constexpr unsigned kGroupRowInner = 2U;
    static constexpr unsigned kGroupRowRuntime = 5U;
    static constexpr unsigned kGroupRowNav = 6U;
    static constexpr std::uint32_t group_stage_mask(std::uint8_t bit) noexcept
    {
        return 1U << bit;
    }
    static constexpr unsigned group_row(std::uint8_t bit) noexcept
    {
        for (unsigned row = 0U; row < kGroupStepCount; ++row)
            if (kGroupSteps[row].bit == bit) return row;
        return kGroupStepCount;
    }
    dima::lib::rover::calibration::CircularMean heading_mean_[2]{};
    Stats3 yaw_fit_[2]{};
    Stats3 bias_fit_[2]{};
    Stats3 bootstrap_fit_[2]{};
    float baseline_samples_[120]{};
    std::size_t baseline_count_{};
    float leg_distance_{};
    float leg_heading_{}, turn_heading_{}, last_turn_heading_{}, turn_integral_{};
    // 慢转向自适应窗口：最后一次可辨识旋转（≥0.03 rad/s 且方向一致）时刻。
    std::uint64_t turn_productive_at_{};
    float longitudinal_{}, steering_{};
    float straight_start_floor_{};
    float return_yaw_offset_{};
    bool turn_rate_request_active_{};
    bool turn_rate_closed_loop_{}, turn_excitation_locked_{};
    float turn_excitation_input_{};
    std::uint64_t turn_excitation_observed_since_{};
    float turn_rate_target_{};
    std::uint64_t turn_rate_opposed_since_{};
    float turn_rate_opposed_heading_{};
    double braking_lat_{}, braking_lon_{};
    float braking_initial_speed_{}, braking_stopped_speed_{}, braking_stopped_distance_{};
    float braking_observed_speed_{};
    float braking_integral_distance_{}, braking_previous_speed_{};
    float braking_reverse_impulse_{}, braking_squared_impulse_{}, braking_minimum_speed_{};
    float braking_output_delay_s_{};
    float braking_budget_s_{};
    std::uint64_t braking_last_epoch_{}, braking_command_at_{}, braking_start_epoch_{}, braking_stopped_epoch_{};
    std::uint64_t braking_settled_since_{}, braking_impulse_time_{};
    std::uint64_t braking_reverse_at_{};
    bool braking_drift_reported_{}; // 制动摆动一次性诊断；观测投影仍用入口冻结方向
    // 磁/转向结束后的回场对准：整定链前进面从入场点沿初始方向起跑；
    // Return/对准两段均由 step() 顶部挂钩经 start_motion 授权后启动。
    bool profile_homing_pending_{};
    bool profile_homing_aligning_{};
    bool profile_homing_returned_{};
    float outbound_axis_rad_{};
    bool outbound_axis_valid_{};
    // 天线杆臂自动测量（2026-09-30）：原地旋转圆半径；参数和只作未测得回退。
    float measured_lever_m_{};
    bool measured_lever_valid_{};
    double rotation_lever_n_sum_{}, rotation_lever_e_sum_{};
    double rotation_lever_nn_{}, rotation_lever_ee_{};
    unsigned rotation_lever_count_{};
    double rotation_lever_lat_{}, rotation_lever_lon_{};
    // 磁源不可用确认计时：瞬时融合抖动不清稳定窗，持续 250ms 才重置。
    std::uint64_t mag_source_bad_since_{};
    // 免受 GNSS 速度解急停振铃（±0.35m/s 持续~1s）阻塞停稳门。
    double quiet_window_lat_{}, quiet_window_lon_{};
    // 位置法入口定位滞后补偿（v0×定位样本滞后秒数，沿冻结方向）。
    float braking_entry_lag_distance_{};
    // 位置静止滑窗（update_inputs 维护）：≥0.5s 定位位移<4cm 为独立静止证据，
    std::uint64_t quiet_window_at_{};
    bool position_quiet_{};
    float braking_forward_north_{}, braking_forward_east_{};
    // 替代原行驶布尔量：停车过程与已经命中到达区不能退回普通行驶。
    enum class ReturnMotion : std::uint8_t { Align, Drive, Brake, Arrived };
    ReturnMotion return_motion_{ReturnMotion::Align};
    bool return_closed_loop_{};
    float return_open_loop_input_{};
    float return_axis_rad_{};
    std::uint64_t return_started_{};
    std::uint64_t drive_envelope_since_{};
    float candidate_mag_[3]{};
    float bootstrap_offset_[3]{};
    std::uint32_t mag_device_id_{};
    std::uint32_t coverage_[2]{};
    std::uint64_t last_epoch_{}, last_mag_sample_{}, last_bias_sample_{};
    std::uint64_t last_fit_velocity_epoch_{};
    std::uint64_t last_alignment_mag_sample_{}, matched_mag_sample_{};
    std::uint64_t steady_since_{};
    std::uint64_t first_circle_at_{};
    std::uint64_t last_report_{};
    std::uint64_t last_run_{}, state_started_{}, session_started_{};
    std::uint64_t stable_since_{}, mag_stable_since_{}, level_request_time_{}, arm_started_{};
    std::uint64_t motion_quality_bad_since_{};
    bool level_snapshot_valid_{};
    std::uint32_t sequence_{}, session_id_{};
    std::uint8_t request_pending_{}; // 本拍动作意图，随唯一状态发布后发送并清空。
    std::uint32_t expected_set_count_{};
    struct MagThrottleFit {
        double mean_u{}, variance_u{}, mean_b[3]{}, covariance[3]{}, variance_b[3]{};
        float minimum_u{1.0F}, maximum_u{}, roll{}, pitch{};
        std::uint32_t count{};
    };
    MagThrottleFit mag_mot_fit_[2]{};
    std::uint64_t last_mag_mot_sample_{};
    bool mag_mot_reported_{false};
    bool endpoint_reported_{false};
    std::uint8_t helper_failure_reason_{};
    bool selected_{}, pending_termination_{}, cancel_requested_{}, bootstrap_applied_{}, mag_ready_{};
    bool config_valid_{};
    bool previously_armed_{}, turn_started_{};
    dima::middleware::lifecycle::ModuleState module_state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::rover::modes
