#include "CalibrationIdentification.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::lib::rover::calibration {
namespace {

constexpr float kMicrosecondsToSeconds = 1.0e-6F;
constexpr float kMaximumOvershootRatio = 0.20F;
constexpr float kSteadyErrorRatio = 0.10F;
constexpr float kNoiseMultiplier = 3.0F;
constexpr float kMinimumDivisor = 1.0e-9F;

bool finite(float value) noexcept
{
    return std::isfinite(value);
}

bool identification_config_valid(const IdentificationConfig &config) noexcept
{
    return finite(config.sample_period_s) && config.sample_period_s > 0.0F &&
        config.sample_period_s <= 1.0F && finite(config.sample_period_tolerance_s) &&
        config.sample_period_tolerance_s >= 0.0F &&
        config.sample_period_tolerance_s < config.sample_period_s &&
        finite(config.forgetting_time_constant_s) &&
        config.forgetting_time_constant_s > config.sample_period_s &&
        config.minimum_samples >= 16U && finite(config.maximum_absolute_input) &&
        config.maximum_absolute_input > 0.0F && finite(config.maximum_absolute_output) &&
        config.maximum_absolute_output > 0.0F && finite(config.minimum_input_variance) &&
        config.minimum_input_variance > 0.0F && finite(config.minimum_output_variance) &&
        config.minimum_output_variance > 0.0F && finite(config.output_noise_variance) &&
        config.output_noise_variance >= 0.0F &&
        finite(config.maximum_normalized_rms_residual) &&
        config.maximum_normalized_rms_residual > 0.0F &&
        finite(config.maximum_relative_coefficient_stddev) &&
        config.maximum_relative_coefficient_stddev > 0.0F &&
        finite(config.coefficient_scale_floor) && config.coefficient_scale_floor > 0.0F &&
        finite(config.minimum_pole) && finite(config.maximum_pole) &&
        config.minimum_pole > 0.0F && config.maximum_pole < 1.0F &&
        config.minimum_pole < config.maximum_pole && finite(config.minimum_dc_gain) &&
        finite(config.maximum_dc_gain) && config.minimum_dc_gain > 0.0F &&
        config.minimum_dc_gain < config.maximum_dc_gain &&
        finite(config.minimum_time_constant_s) && finite(config.maximum_time_constant_s) &&
        config.minimum_time_constant_s > 0.0F &&
        config.minimum_time_constant_s < config.maximum_time_constant_s;
}

bool pi_limits_valid(const PiDesignLimits &limits) noexcept
{
    return finite(limits.requested_closed_loop_time_s) &&
        limits.requested_closed_loop_time_s > 0.0F && finite(limits.output_headroom) &&
        limits.output_headroom > 0.0F && limits.output_headroom <= 1.0F &&
        finite(limits.maximum_error) && limits.maximum_error > 0.0F &&
        finite(limits.integration_horizon_s) && limits.integration_horizon_s > 0.0F &&
        finite(limits.maximum_proportional_gain) &&
        limits.maximum_proportional_gain > 0.0F && finite(limits.maximum_integral_gain) &&
        limits.maximum_integral_gain > 0.0F;
}

bool sample_time_valid(std::uint64_t timestamp_us, std::uint64_t last_timestamp_us,
                       float expected_period_s, float tolerance_s,
                       CalibrationAlgorithmFailure &failure) noexcept
{
    if (timestamp_us == 0U) {
        failure = CalibrationAlgorithmFailure::InvalidSample;
        return false;
    }
    if (last_timestamp_us == 0U) return true;
    if (timestamp_us == last_timestamp_us) {
        failure = CalibrationAlgorithmFailure::DuplicateTimestamp;
        return false;
    }
    if (timestamp_us < last_timestamp_us) {
        failure = CalibrationAlgorithmFailure::TimeRegression;
        return false;
    }
    const float period_s = static_cast<float>(timestamp_us - last_timestamp_us) *
        kMicrosecondsToSeconds;
    if (!finite(period_s) || std::fabs(period_s - expected_period_s) > tolerance_s) {
        failure = CalibrationAlgorithmFailure::SamplePeriodMismatch;
        return false;
    }
    return true;
}

template<std::size_t Delay>
void reset_bank(ArxRls<1U, 0U, Delay> &bank,
                const IdentificationConfig &config) noexcept
{
    bank.reset();
    bank.setForgettingFactor(config.forgetting_time_constant_s,
                             config.sample_period_s);
}

template<std::size_t Delay>
bool update_bank(ArxRls<1U, 0U, Delay> &bank, std::size_t index,
                 float input, float output, std::uint32_t sample_count,
                 const IdentificationConfig &config,
                 double (&residual_squared)[FirstOrderDelayIdentifier::kDelayBankSize],
                 std::uint32_t (&residual_count)[FirstOrderDelayIdentifier::kDelayBankSize]) noexcept
{
    bank.update(input, output);
    const auto &coefficients = bank.getCoefficients();
    const auto variances = bank.getVariances();
    if (!finite(coefficients(0)) || !finite(coefficients(1)) ||
        !finite(variances(0)) || !finite(variances(1)) ||
        !finite(bank.getInnovation())) return false;

    // RLS 初始大协方差会产生很大的暂态 innovation；残差门禁只累计后半段，
    // 但系数与协方差仍从第一份唯一样本开始更新。
    const std::uint32_t warmup = std::max<std::uint32_t>(
        static_cast<std::uint32_t>(Delay + 3U), config.minimum_samples / 2U);
    if (sample_count > warmup) {
        const double innovation = bank.getInnovation();
        residual_squared[index] += innovation * innovation;
        ++residual_count[index];
    }
    return true;
}

bool evaluate_model(const matrix::Vector<float, 2U> &coefficients,
                    const matrix::Vector<float, 2U> &variances,
                    std::size_t delay, std::size_t index,
                    const IdentificationConfig &config, std::uint32_t sample_count,
                    float input_variance, float output_variance,
                    const double (&residual_squared)[FirstOrderDelayIdentifier::kDelayBankSize],
                    const std::uint32_t (&residual_count)[FirstOrderDelayIdentifier::kDelayBankSize],
                    FirstOrderModel &model) noexcept
{
    // 六个延迟模型共用同一评估实体；调用方已检查残差样本数并按原顺序
    // 取得系数/方差。仅把 Delay 作为数据传入，不改公式、精度或短路门禁。
    const float a = coefficients(0);
    const float b = coefficients(1);
    const float pole = -a;
    if (!finite(a) || !finite(b) || !finite(variances(0)) || !finite(variances(1)) ||
        variances(0) < 0.0F || variances(1) < 0.0F || pole < config.minimum_pole ||
        pole > config.maximum_pole) return false;

    const float denominator = 1.0F + a;
    if (std::fabs(denominator) <= kMinimumDivisor) return false;
    const float dc_gain = b / denominator;
    const float time_constant_s = -config.sample_period_s / std::log(pole);
    if (!finite(dc_gain) || dc_gain < config.minimum_dc_gain ||
        dc_gain > config.maximum_dc_gain || !finite(time_constant_s) ||
        time_constant_s < config.minimum_time_constant_s ||
        time_constant_s > config.maximum_time_constant_s) return false;

    // ArxRls::getVariances() 返回的是 RLS 逆信息矩阵 P 的对角项，不是已经
    // 带输出量纲的参数协方差。先以扣除两个模型自由度的 innovation 残差
    // 方差估计输出噪声，再与调用方给出的测量噪声下界取大，最终 covariance=P*sigma^2。
    const double residual_variance = residual_squared[index] /
        static_cast<double>(residual_count[index] - 2U);
    const double output_noise_variance = std::max(
        residual_variance, static_cast<double>(config.output_noise_variance));
    const double pole_variance = static_cast<double>(variances(0)) *
        output_noise_variance;
    const double input_coefficient_variance =
        static_cast<double>(variances(1)) * output_noise_variance;
    if (!std::isfinite(residual_variance) || residual_variance < 0.0 ||
        !std::isfinite(pole_variance) || pole_variance < 0.0 ||
        !std::isfinite(input_coefficient_variance) ||
        input_coefficient_variance < 0.0) return false;

    const float relative_a = static_cast<float>(std::sqrt(pole_variance)) /
        std::max(std::fabs(a), config.coefficient_scale_floor);
    const float relative_b = static_cast<float>(
        std::sqrt(input_coefficient_variance)) /
        std::max(std::fabs(b), config.coefficient_scale_floor);
    const float maximum_relative = std::max(relative_a, relative_b);
    if (!finite(maximum_relative) ||
        maximum_relative > config.maximum_relative_coefficient_stddev) return false;

    const float rms_residual = static_cast<float>(std::sqrt(
        residual_squared[index] / static_cast<double>(residual_count[index])));
    const float normalized_residual = rms_residual /
        std::max(std::sqrt(output_variance),
                 std::sqrt(config.minimum_output_variance));
    if (!finite(normalized_residual) ||
        normalized_residual > config.maximum_normalized_rms_residual) return false;

    model.valid = true;
    model.delay_samples = static_cast<std::uint8_t>(delay);
    model.sample_count = sample_count;
    model.pole = pole;
    model.input_coefficient = b;
    model.dc_gain = dc_gain;
    model.time_constant_s = time_constant_s;
    model.delay_s = static_cast<float>(delay) * config.sample_period_s;
    model.pole_variance = static_cast<float>(pole_variance);
    model.input_coefficient_variance = static_cast<float>(
        input_coefficient_variance);
    model.maximum_relative_coefficient_stddev = maximum_relative;
    model.normalized_rms_residual = normalized_residual;
    model.input_variance = input_variance;
    model.output_variance = output_variance;
    return true;
}

template<std::size_t Delay>
bool evaluate_bank(const ArxRls<1U, 0U, Delay> &bank, std::size_t index,
                   const IdentificationConfig &config, std::uint32_t sample_count,
                   float input_variance, float output_variance,
                   const double (&residual_squared)[FirstOrderDelayIdentifier::kDelayBankSize],
                   const std::uint32_t (&residual_count)[FirstOrderDelayIdentifier::kDelayBankSize],
                   FirstOrderModel &model) noexcept
{
    // 一阶模型有 pole/input coefficient 两个自由参数；N<=2 时残差方差
    // 没有正自由度，不能生成表面有限的系数置信度。保留先判定、再读模型的顺序。
    if (residual_count[index] <= 2U ||
        residual_count[index] < config.minimum_samples / 4U) return false;
    const auto &coefficients = bank.getCoefficients();
    const auto variances = bank.getVariances();
    return evaluate_model(coefficients, variances, Delay, index, config, sample_count,
                          input_variance, output_variance, residual_squared,
                          residual_count, model);
}

bool step_config_valid(const StepValidationConfig &config) noexcept
{
    if (!finite(config.sample_period_s) || config.sample_period_s <= 0.0F ||
        config.sample_period_s > 1.0F || !finite(config.sample_period_tolerance_s) ||
        config.sample_period_tolerance_s < 0.0F ||
        config.sample_period_tolerance_s >= config.sample_period_s ||
        !finite(config.initial_output) || !finite(config.target_output) ||
        !finite(config.noise) || config.noise < 0.0F ||
        !finite(config.absolute_steady_tolerance) ||
        config.absolute_steady_tolerance < 0.0F ||
        config.minimum_samples < 2U || config.steady_window_samples < 2U ||
        config.steady_window_samples > StepResponseValidator::kMaximumSteadyWindowSamples ||
        config.minimum_samples < config.steady_window_samples ||
        !finite(config.maximum_continuous_saturation_s) ||
        config.maximum_continuous_saturation_s < config.sample_period_s) return false;
    const float step = std::fabs(config.target_output - config.initial_output);
    return finite(step) && config.absolute_steady_tolerance < step &&
        step > std::max(kNoiseMultiplier * config.noise,
                        std::numeric_limits<float>::epsilon());
}

} // namespace

bool FirstOrderDelayIdentifier::configure(const IdentificationConfig &config) noexcept
{
    configured_ = identification_config_valid(config);
    config_ = configured_ ? config : IdentificationConfig{};
    reset();
    return configured_;
}

void FirstOrderDelayIdentifier::reset() noexcept
{
    reset_bank(delay_0_, config_);
    reset_bank(delay_1_, config_);
    reset_bank(delay_2_, config_);
    reset_bank(delay_3_, config_);
    reset_bank(delay_4_, config_);
    reset_bank(delay_5_, config_);
    for (std::size_t index = 0U; index < kDelayBankSize; ++index) {
        residual_squared_[index] = 0.0;
        residual_count_[index] = 0U;
    }
    input_mean_ = input_m2_ = output_mean_ = output_m2_ = 0.0;
    last_timestamp_us_ = 0U;
    sample_count_ = 0U;
    sample_failure_ = configured_ ? CalibrationAlgorithmFailure::None
                                  : CalibrationAlgorithmFailure::InvalidConfiguration;
}

bool FirstOrderDelayIdentifier::add_sample(std::uint64_t timestamp_us,
                                           float input_delta,
                                           float output_delta) noexcept
{
    if (!configured_ || sample_failure_ != CalibrationAlgorithmFailure::None) return false;
    if (!finite(input_delta) || !finite(output_delta) ||
        std::fabs(input_delta) > config_.maximum_absolute_input ||
        std::fabs(output_delta) > config_.maximum_absolute_output) {
        sample_failure_ = CalibrationAlgorithmFailure::InvalidSample;
        return false;
    }
    if (!sample_time_valid(timestamp_us, last_timestamp_us_, config_.sample_period_s,
                           config_.sample_period_tolerance_s, sample_failure_)) return false;
    last_timestamp_us_ = timestamp_us;

    ++sample_count_;
    const double count = static_cast<double>(sample_count_);
    const double input_difference = static_cast<double>(input_delta) - input_mean_;
    input_mean_ += input_difference / count;
    input_m2_ += input_difference * (static_cast<double>(input_delta) - input_mean_);
    const double output_difference = static_cast<double>(output_delta) - output_mean_;
    output_mean_ += output_difference / count;
    output_m2_ += output_difference * (static_cast<double>(output_delta) - output_mean_);

    const bool updated =
        update_bank(delay_0_, 0U, input_delta, output_delta, sample_count_, config_,
                    residual_squared_, residual_count_) &&
        update_bank(delay_1_, 1U, input_delta, output_delta, sample_count_, config_,
                    residual_squared_, residual_count_) &&
        update_bank(delay_2_, 2U, input_delta, output_delta, sample_count_, config_,
                    residual_squared_, residual_count_) &&
        update_bank(delay_3_, 3U, input_delta, output_delta, sample_count_, config_,
                    residual_squared_, residual_count_) &&
        update_bank(delay_4_, 4U, input_delta, output_delta, sample_count_, config_,
                    residual_squared_, residual_count_) &&
        update_bank(delay_5_, 5U, input_delta, output_delta, sample_count_, config_,
                    residual_squared_, residual_count_);
    if (!updated) sample_failure_ = CalibrationAlgorithmFailure::ModelRejected;
    return updated;
}

IdentificationResult FirstOrderDelayIdentifier::fit(
    const PiDesignLimits &limits) const noexcept
{
    IdentificationResult result{};
    if (!configured_) return result;
    if (sample_failure_ != CalibrationAlgorithmFailure::None) {
        result.failure = sample_failure_;
        return result;
    }
    if (sample_count_ < config_.minimum_samples || sample_count_ < 2U) {
        result.failure = CalibrationAlgorithmFailure::InsufficientSamples;
        return result;
    }
    const float input_variance = static_cast<float>(
        input_m2_ / static_cast<double>(sample_count_ - 1U));
    const float output_variance = static_cast<float>(
        output_m2_ / static_cast<double>(sample_count_ - 1U));
    if (!finite(input_variance) || !finite(output_variance) ||
        input_variance < config_.minimum_input_variance ||
        output_variance < config_.minimum_output_variance) {
        result.failure = CalibrationAlgorithmFailure::InsufficientExcitation;
        return result;
    }

    FirstOrderModel candidate{};
    FirstOrderModel best{};
    float best_residual = std::numeric_limits<float>::infinity();
    const auto consider = [&](bool valid) {
        if (valid && candidate.normalized_rms_residual < best_residual) {
            best = candidate;
            best_residual = candidate.normalized_rms_residual;
        }
        candidate = {};
    };
    consider(evaluate_bank(delay_0_, 0U, config_, sample_count_, input_variance,
                           output_variance, residual_squared_, residual_count_, candidate));
    consider(evaluate_bank(delay_1_, 1U, config_, sample_count_, input_variance,
                           output_variance, residual_squared_, residual_count_, candidate));
    consider(evaluate_bank(delay_2_, 2U, config_, sample_count_, input_variance,
                           output_variance, residual_squared_, residual_count_, candidate));
    consider(evaluate_bank(delay_3_, 3U, config_, sample_count_, input_variance,
                           output_variance, residual_squared_, residual_count_, candidate));
    consider(evaluate_bank(delay_4_, 4U, config_, sample_count_, input_variance,
                           output_variance, residual_squared_, residual_count_, candidate));
    consider(evaluate_bank(delay_5_, 5U, config_, sample_count_, input_variance,
                           output_variance, residual_squared_, residual_count_, candidate));
    if (!best.valid) {
        result.failure = CalibrationAlgorithmFailure::ModelRejected;
        return result;
    }
    result.model = best;

    if (!pi_limits_valid(limits)) {
        result.failure = CalibrationAlgorithmFailure::PiRejected;
        return result;
    }
    // 一阶带延迟模型 G(s)=K*exp(-theta*s)/(tau*s+1) 的保守 IMC PI：
    // Kp=tau/[K(lambda+theta)]，Ti=tau+theta/2，Ki=Kp/Ti。
    // lambda 至少取 3*tau、5*theta 和 1 s，避免把 aircraft 的快环带宽套给 Rover。
    const float lambda = std::max({limits.requested_closed_loop_time_s,
                                   3.0F * best.time_constant_s,
                                   5.0F * best.delay_s, 1.0F});
    const float integral_time = best.time_constant_s + 0.5F * best.delay_s;
    float proportional = best.time_constant_s /
        (best.dc_gain * (lambda + best.delay_s));
    float integral = proportional / integral_time;
    if (!finite(lambda) || !finite(integral_time) || integral_time <= 0.0F ||
        !finite(proportional) || !finite(integral) || proportional <= 0.0F ||
        integral <= 0.0F) {
        result.failure = CalibrationAlgorithmFailure::PiRejected;
        return result;
    }

    // 同比例缩放 P/I，保留 Ti；在 maximum_error 持续 integration_horizon
    // 的保守情况下，反馈项仍不得吃完调用方预留的归一化输出余量。
    const float raw_demand = limits.maximum_error *
        (proportional + integral * limits.integration_horizon_s);
    float scale = 1.0F;
    if (raw_demand > limits.output_headroom)
        scale = std::min(scale, limits.output_headroom / raw_demand);
    if (proportional > limits.maximum_proportional_gain)
        scale = std::min(scale, limits.maximum_proportional_gain / proportional);
    if (integral > limits.maximum_integral_gain)
        scale = std::min(scale, limits.maximum_integral_gain / integral);
    if (!finite(scale) || scale <= 0.0F) {
        result.failure = CalibrationAlgorithmFailure::PiRejected;
        return result;
    }
    proportional *= scale;
    integral *= scale;
    const float bounded_demand = limits.maximum_error *
        (proportional + integral * limits.integration_horizon_s);
    if (!finite(proportional) || !finite(integral) ||
        proportional <= 0.0F || integral <= 0.0F ||
        bounded_demand > limits.output_headroom + 1.0e-6F) {
        result.failure = CalibrationAlgorithmFailure::PiRejected;
        return result;
    }

    result.pi.valid = true;
    result.pi.headroom_limited = scale < 1.0F;
    result.pi.proportional_gain = proportional;
    result.pi.integral_gain = integral;
    result.pi.integral_time_s = integral_time;
    result.pi.closed_loop_time_s = lambda;
    result.pi.worst_case_feedback_demand = bounded_demand;
    result.failure = CalibrationAlgorithmFailure::None;
    return result;
}

bool StepResponseValidator::reset(const StepValidationConfig &config) noexcept
{
    configured_ = step_config_valid(config);
    config_ = configured_ ? config : StepValidationConfig{};
    for (std::size_t index = 0U; index < kMaximumSteadyWindowSamples; ++index) {
        steady_measurements_[index] = 0.0F;
        steady_saturation_[index] = false;
    }
    steady_next_ = steady_count_ = steady_saturation_count_ = 0U;
    last_timestamp_us_ = saturation_started_us_ = maximum_saturation_us_ = 0U;
    sample_count_ = 0U;
    error_crossings_ = 0U;
    last_error_sign_ = 0;
    maximum_overshoot_ratio_ = 0.0F;
    sample_failure_ = configured_ ? CalibrationAlgorithmFailure::None
                                  : CalibrationAlgorithmFailure::InvalidConfiguration;
    return configured_;
}

bool StepResponseValidator::add_sample(std::uint64_t timestamp_us,
                                       float measurement,
                                       bool saturated) noexcept
{
    if (!configured_ || sample_failure_ != CalibrationAlgorithmFailure::None) return false;
    if (!finite(measurement)) {
        sample_failure_ = CalibrationAlgorithmFailure::InvalidSample;
        return false;
    }
    if (!sample_time_valid(timestamp_us, last_timestamp_us_, config_.sample_period_s,
                           config_.sample_period_tolerance_s, sample_failure_)) return false;
    last_timestamp_us_ = timestamp_us;
    ++sample_count_;

    const float signed_step = config_.target_output - config_.initial_output;
    const float step = std::fabs(signed_step);
    const float direction = signed_step > 0.0F ? 1.0F : -1.0F;
    const float progress = direction * (measurement - config_.initial_output);
    const float overshoot = std::max(0.0F, (progress - step) / step);
    maximum_overshoot_ratio_ = std::max(maximum_overshoot_ratio_, overshoot);
    if (maximum_overshoot_ratio_ > kMaximumOvershootRatio) {
        sample_failure_ = CalibrationAlgorithmFailure::Overshoot;
        return false;
    }

    const float allowed_error = std::max(kSteadyErrorRatio * step,
                                         kNoiseMultiplier * config_.noise);
    const float error = config_.target_output - measurement;
    if (std::fabs(error) > allowed_error) {
        const std::int8_t sign = error > 0.0F ? 1 : -1;
        if (last_error_sign_ != 0 && sign != last_error_sign_) {
            if (error_crossings_ < std::numeric_limits<std::uint8_t>::max())
                ++error_crossings_;
            if (error_crossings_ > config_.maximum_error_crossings) {
                sample_failure_ = CalibrationAlgorithmFailure::SustainedOscillation;
                return false;
            }
        }
        last_error_sign_ = sign;
    }

    if (saturated) {
        if (saturation_started_us_ == 0U) saturation_started_us_ = timestamp_us;
        const std::uint64_t duration_us = timestamp_us - saturation_started_us_;
        maximum_saturation_us_ = std::max(maximum_saturation_us_, duration_us);
        const double limit_us = static_cast<double>(config_.maximum_continuous_saturation_s) *
            1000000.0;
        if (static_cast<double>(duration_us) >= limit_us) {
            sample_failure_ = CalibrationAlgorithmFailure::SustainedSaturation;
            return false;
        }
    } else {
        saturation_started_us_ = 0U;
    }

    const std::size_t window = config_.steady_window_samples;
    if (steady_count_ == window && steady_saturation_[steady_next_])
        --steady_saturation_count_;
    steady_measurements_[steady_next_] = measurement;
    steady_saturation_[steady_next_] = saturated;
    if (saturated) ++steady_saturation_count_;
    steady_next_ = (steady_next_ + 1U) % window;
    if (steady_count_ < window) ++steady_count_;
    return true;
}

StepValidationResult StepResponseValidator::result() const noexcept
{
    StepValidationResult output{};
    output.sample_count = sample_count_;
    output.error_crossings = error_crossings_;
    output.maximum_overshoot_ratio = maximum_overshoot_ratio_;
    output.maximum_continuous_saturation_s =
        static_cast<float>(maximum_saturation_us_) * kMicrosecondsToSeconds;
    const float step = std::fabs(config_.target_output - config_.initial_output);
    // 最终稳态允许误差取比例门槛、三倍测量噪声和调用方给出的物理死区三者最大值；
    // 绝对死区只在最终窗口判定使用，不能伪装成 noise 去放宽逐样本振荡检测。
    output.allowed_steady_state_error = std::max(
        {kSteadyErrorRatio * step, kNoiseMultiplier * config_.noise,
         config_.absolute_steady_tolerance});
    if (!configured_) return output;
    if (sample_failure_ != CalibrationAlgorithmFailure::None) {
        output.failure = sample_failure_;
        return output;
    }
    if (sample_count_ < config_.minimum_samples ||
        steady_count_ < config_.steady_window_samples) {
        output.failure = CalibrationAlgorithmFailure::InsufficientSamples;
        return output;
    }
    // 整个最终窗口都顶在输出边界，即使均值碰巧接近目标也不算可信闭环。
    if (steady_saturation_count_ == steady_count_) {
        output.failure = CalibrationAlgorithmFailure::SustainedSaturation;
        return output;
    }

    double sum = 0.0;
    for (std::size_t index = 0U; index < steady_count_; ++index)
        sum += steady_measurements_[index];
    const float mean = static_cast<float>(sum / static_cast<double>(steady_count_));
    output.steady_state_error = std::fabs(config_.target_output - mean);
    if (!finite(mean) || !finite(output.steady_state_error) ||
        output.steady_state_error > output.allowed_steady_state_error) {
        output.failure = CalibrationAlgorithmFailure::SteadyStateError;
        return output;
    }
    output.failure = CalibrationAlgorithmFailure::None;
    return output;
}

} // namespace dima::lib::rover::calibration
