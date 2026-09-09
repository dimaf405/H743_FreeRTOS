#pragma once

#include "ArxRls.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::lib::rover::calibration {

enum class CalibrationAlgorithmFailure : std::uint8_t {
    None,
    InvalidConfiguration,
    InvalidSample,
    DuplicateTimestamp,
    TimeRegression,
    SamplePeriodMismatch,
    InsufficientSamples,
    InsufficientExcitation,
    ModelRejected,
    PiRejected,
    Overshoot,
    SteadyStateError,
    SustainedOscillation,
    SustainedSaturation,
};

struct IdentificationConfig {
    // 速度辨识默认按 10 Hz 唯一样本工作；yaw-rate 调用方改为 0.02 s。
    float sample_period_s{0.1F};
    float sample_period_tolerance_s{0.025F};
    float forgetting_time_constant_s{60.0F};
    std::uint32_t minimum_samples{60U};
    float maximum_absolute_input{1.0F};
    float maximum_absolute_output{2.0F};
    float minimum_input_variance{1.0e-4F};
    float minimum_output_variance{1.0e-4F};
    // 输出测量噪声方差下界，单位是 output_delta 单位的平方。
    float output_noise_variance{1.0e-6F};
    float maximum_normalized_rms_residual{0.25F};
    float maximum_relative_coefficient_stddev{0.50F};
    float coefficient_scale_floor{1.0e-3F};
    float minimum_pole{0.01F};
    float maximum_pole{0.995F};
    float minimum_dc_gain{1.0e-3F};
    float maximum_dc_gain{1000.0F};
    float minimum_time_constant_s{0.02F};
    float maximum_time_constant_s{30.0F};
};

struct PiDesignLimits {
    // 实际 lambda 还会被提高到 max(3*tau, 5*delay, 1 s)。
    float requested_closed_loop_time_s{1.0F};
    float output_headroom{0.10F};
    float maximum_error{1.0F};
    float integration_horizon_s{1.0F};
    float maximum_proportional_gain{100.0F};
    float maximum_integral_gain{100.0F};
};

struct FirstOrderModel {
    bool valid{false};
    std::uint8_t delay_samples{0U};
    std::uint32_t sample_count{0U};
    float pole{0.0F};
    float input_coefficient{0.0F};
    float dc_gain{0.0F};
    float time_constant_s{0.0F};
    float delay_s{0.0F};
    float pole_variance{0.0F};
    float input_coefficient_variance{0.0F};
    float maximum_relative_coefficient_stddev{0.0F};
    float normalized_rms_residual{0.0F};
    float input_variance{0.0F};
    float output_variance{0.0F};
};

struct ParallelPiCandidate {
    bool valid{false};
    bool headroom_limited{false};
    float proportional_gain{0.0F};
    float integral_gain{0.0F};
    float integral_time_s{0.0F};
    float closed_loop_time_s{0.0F};
    float worst_case_feedback_demand{0.0F};
};

struct IdentificationResult {
    CalibrationAlgorithmFailure failure{CalibrationAlgorithmFailure::InvalidConfiguration};
    FirstOrderModel model{};
    ParallelPiCandidate pi{};

    bool valid() const noexcept;
};

// 固定六模型 delay bank；调用方必须输入已经去工作点的 delta input/output。
class FirstOrderDelayIdentifier final {
public:
    static constexpr std::size_t kDelayBankSize{6U};

    bool configure(const IdentificationConfig &config) noexcept;
    void reset() noexcept;
    bool add_sample(std::uint64_t timestamp_us, float input_delta,
                    float output_delta) noexcept;
    IdentificationResult fit(const PiDesignLimits &limits) const noexcept;

    CalibrationAlgorithmFailure sample_failure() const noexcept;
    std::uint32_t sample_count() const noexcept;

private:
    IdentificationConfig config_{};
    ArxRls<1U, 0U, 0U> delay_0_{};
    ArxRls<1U, 0U, 1U> delay_1_{};
    ArxRls<1U, 0U, 2U> delay_2_{};
    ArxRls<1U, 0U, 3U> delay_3_{};
    ArxRls<1U, 0U, 4U> delay_4_{};
    ArxRls<1U, 0U, 5U> delay_5_{};
    double residual_squared_[kDelayBankSize]{};
    std::uint32_t residual_count_[kDelayBankSize]{};
    double input_mean_{0.0};
    double input_m2_{0.0};
    double output_mean_{0.0};
    double output_m2_{0.0};
    std::uint64_t last_timestamp_us_{0U};
    std::uint32_t sample_count_{0U};
    CalibrationAlgorithmFailure sample_failure_{CalibrationAlgorithmFailure::InvalidConfiguration};
    bool configured_{false};
};

struct StepValidationConfig {
    float sample_period_s{0.1F};
    float sample_period_tolerance_s{0.025F};
    float initial_output{0.0F};
    float target_output{1.0F};
    float noise{0.01F};
    // 最终稳态均值允许的绝对误差，单位与 measurement 相同；必须小于阶跃幅值。
    // 该值不参与逐样本振荡判据，避免用执行机构死区掩盖真实的持续振荡。
    float absolute_steady_tolerance{0.0F};
    std::uint32_t minimum_samples{30U};
    std::uint8_t steady_window_samples{10U};
    std::uint8_t maximum_error_crossings{4U};
    float maximum_continuous_saturation_s{1.0F};
};

struct StepValidationResult {
    CalibrationAlgorithmFailure failure{CalibrationAlgorithmFailure::InvalidConfiguration};
    std::uint32_t sample_count{0U};
    std::uint8_t error_crossings{0U};
    float maximum_overshoot_ratio{0.0F};
    float steady_state_error{0.0F};
    float allowed_steady_state_error{0.0F};
    float maximum_continuous_saturation_s{0.0F};

    bool valid() const noexcept;
};

class StepResponseValidator final {
public:
    static constexpr std::size_t kMaximumSteadyWindowSamples{64U};

    bool reset(const StepValidationConfig &config) noexcept;
    bool add_sample(std::uint64_t timestamp_us, float measurement,
                    bool saturated) noexcept;
    StepValidationResult result() const noexcept;

    CalibrationAlgorithmFailure sample_failure() const noexcept;

private:
    StepValidationConfig config_{};
    float steady_measurements_[kMaximumSteadyWindowSamples]{};
    bool steady_saturation_[kMaximumSteadyWindowSamples]{};
    std::size_t steady_next_{0U};
    std::size_t steady_count_{0U};
    std::size_t steady_saturation_count_{0U};
    std::uint64_t last_timestamp_us_{0U};
    std::uint64_t saturation_started_us_{0U};
    std::uint64_t maximum_saturation_us_{0U};
    std::uint32_t sample_count_{0U};
    std::uint8_t error_crossings_{0U};
    std::int8_t last_error_sign_{0};
    float maximum_overshoot_ratio_{0.0F};
    CalibrationAlgorithmFailure sample_failure_{CalibrationAlgorithmFailure::InvalidConfiguration};
    bool configured_{false};
};

} // namespace dima::lib::rover::calibration
