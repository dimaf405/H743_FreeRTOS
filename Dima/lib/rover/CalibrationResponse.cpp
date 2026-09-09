#include "CalibrationResponse.hpp"

#include <algorithm>
#include <cmath>

namespace dima::lib::rover::calibration {

bool MotorResponseProfile::replace_direction(std::size_t direction, const MotorResponseProfile &source,
    std::size_t source_direction) noexcept
{
    if (direction >= kDirections || source_direction >= kDirections ||
        failure_ != ResponseFailure::None || source.failure_ != ResponseFailure::None) return false;
    // 协调器仅在同 session/设备/参数代下，用独立满输出扫描替换前向曲线。
    // 后向数据仍保留；合并本身不放宽完整覆盖或拟合门禁。
    for (std::size_t i = 0U; i < kLevels; ++i) plateaus_[direction][i] = source.plateaus_[source_direction][i];
    last_timestamp_us_ = std::max(last_timestamp_us_, source.last_timestamp_us_);
    return true;
}

bool MotorResponseProfile::reset_plateau(std::size_t direction, std::size_t level) noexcept
{
    if (direction >= kDirections || level >= kLevels || failure_ != ResponseFailure::None) return false;
    // 连续稳态被打断时只清当前档；全局去重时间保留，不能藉清窗重复喂旧样本。
    plateaus_[direction][level] = {};
    return true;
}

namespace {

constexpr std::uint32_t kMinimumPlateauSamples{10U};
constexpr double kConfidenceMultiplier{3.0};
constexpr double kMinimumDivisor{1.0e-12};

float unavailable() noexcept
{
    return std::numeric_limits<float>::quiet_NaN();
}

bool valid_noise(float noise) noexcept
{
    // 真实测量精度必须由调用方给出；零方差的重复量化样本不能自己创造精度。
    return std::isfinite(noise) && noise > 0.0F;
}

bool valid_timestamp(std::uint64_t timestamp, std::uint64_t previous,
                      ResponseFailure &failure) noexcept
{
    if (timestamp == 0U) failure = ResponseFailure::InvalidSample;
    else if (timestamp == previous) failure = ResponseFailure::DuplicateTimestamp;
    else if (timestamp < previous) failure = ResponseFailure::TimeRegression;
    return failure == ResponseFailure::None;
}

ResponseEstimate rejected(ResponseFailure failure) noexcept
{
    return {unavailable(), failure};
}

ResponseEstimate estimate(double value) noexcept
{
    const float result = static_cast<float>(value);
    return std::isfinite(result) ? ResponseEstimate{result, ResponseFailure::None}
                                 : rejected(ResponseFailure::UncertainResponse);
}

bool plateau_valid(const ResponsePlateau &plateau) noexcept
{
    const auto count = plateau.count();
    return count >= kMinimumPlateauSamples &&
        plateau.pre_input.count() == count && plateau.applied_input.count() == count &&
        plateau.pre_input.failure() == ResponseFailure::None &&
        plateau.applied_input.failure() == ResponseFailure::None &&
        plateau.raw_speed.failure() == ResponseFailure::None &&
        plateau.raw_speed.duration_s() >= 1.0F &&
        std::isfinite(plateau.pre_input.variance()) &&
        std::isfinite(plateau.applied_input.variance()) &&
        std::isfinite(plateau.raw_speed.variance());
}

double response_sigma(const ResponsePlateau &plateau, float noise) noexcept
{
    return std::max(static_cast<double>(noise),
                    std::sqrt(plateau.raw_speed.variance()));
}

bool input_stable(const ResponsePlateau &plateau) noexcept
{
    // 这里只检查平台内稳定性，不要求 pre==applied；正常 MIN/EXPO/ASYM 映射
    // 正是待辨识对象。保护器是否遮蔽平台仍由调用方按真实原因位剔除。
    return plateau.pre_input.standard_deviation() <=
               std::max(0.002, 0.02 * plateau.pre_input.mean()) &&
        plateau.applied_input.standard_deviation() <=
               std::max(0.002, 0.02 * plateau.applied_input.mean());
}

struct CurveFit {
    double gain{};
    double expo{};
    double expo_variance{};
    bool valid{};
};

CurveFit fit_curve(const MotorResponseProfile &profile, std::size_t direction,
                    double minimum, double minimum_uncertainty,
                    const MotorResponseDesign &design) noexcept
{
    CurveFit result{};
    const double range = static_cast<double>(design.motor_max) - minimum;
    const auto &endpoint = *profile.plateau(direction, MotorResponseProfile::kLevels - 1U);
    const double endpoint_r = (endpoint.applied_input.mean() - minimum) / range;
    if (range <= kMinimumDivisor || endpoint_r <= kMinimumDivisor) return result;
    const double gain_seed = endpoint.raw_speed.mean() / endpoint_r;
    double xx{}, xq{}, qq{}, xy{}, qy{};
    double weights[MotorResponseProfile::kLevels]{};
    double normalized[MotorResponseProfile::kLevels]{};
    unsigned count{};
    for (std::size_t level = 0U; level < MotorResponseProfile::kLevels; ++level) {
        const auto &plateau = *profile.plateau(direction, level);
        const double r = (plateau.applied_input.mean() - minimum) / range;
        const double speed = plateau.raw_speed.mean();
        if (r <= 0.0 || speed <= 5.0 * design.measurement_noise) continue;
        if (r > 1.0 || !std::isfinite(r)) return result;
        const double sigma = response_sigma(plateau, design.measurement_noise);
        const double input_variance = (plateau.applied_input.variance() +
            minimum_uncertainty * minimum_uncertainty) / (range * range);
        // 将实际输入/起动括界的不确定性也计入权重；不能以精确命令值掩盖未知
        // 的 MIN。系数协方差保留真实测量噪声，即使拟合残差恰好为零也不归零。
        const double variance = sigma * sigma + 4.0 * gain_seed * gain_seed * input_variance;
        if (!std::isfinite(variance) || variance <= 0.0) return result;
        const double weight = 1.0 / variance;
        const double r2 = r * r;
        normalized[level] = r;
        weights[level] = weight;
        xx += weight * r2;
        xq += weight * r2 * r;
        qq += weight * r2 * r2;
        xy += weight * r * speed;
        qy += weight * r2 * speed;
        ++count;
    }
    const double determinant = xx * qq - xq * xq;
    if (count < 4U || !std::isfinite(determinant) || xx <= 0.0 || qq <= 0.0 ||
        determinant <= 1.0e-4 * xx * qq) return result;

    // 生产 expo_inverse 对应的物理模型是 v=a*r+b*r^2，而非直接对 u 加平方。
    // K=a+b，expo=b/K；两自由度同时拟合，95% 输出点不被冒充为精确满输出点。
    const double a = (xy * qq - qy * xq) / determinant;
    const double b = (qy * xx - xy * xq) / determinant;
    const double gain = a + b;
    if (!std::isfinite(gain) || gain <= 5.0 * design.measurement_noise) return result;
    const double expo = b / gain;
    if (!std::isfinite(expo) || expo < -1.0 || expo > 1.0) return result;
    double weighted_rss{}, rss{};
    for (std::size_t level = 0U; level < MotorResponseProfile::kLevels; ++level) {
        if (weights[level] == 0.0) continue;
        const double r = normalized[level];
        const double error = profile.plateau(direction, level)->raw_speed.mean() - a * r - b * r * r;
        rss += error * error;
        weighted_rss += weights[level] * error * error;
    }
    const double variance_scale = std::max(1.0, weighted_rss / static_cast<double>(count - 2U));
    const double variance = variance_scale * (b * b * qq + a * a * xx + 2.0 * a * b * xq) /
        (determinant * gain * gain * gain * gain);
    if (!std::isfinite(variance) || variance <= 0.0 || variance > 0.01 ||
        !std::isfinite(rss) || std::sqrt(rss / count) > 0.10 * gain) return result;
    result = {gain, expo, variance, true};
    return result;
}

bool fit_shared_expo(const MotorResponseProfile &profile, std::size_t direction,
                      double minimum, double minimum_uncertainty,
                      double expo, double expo_variance, float noise,
                      float motor_max, double &gain, double &gain_variance) noexcept
{
    const double range = motor_max - minimum;
    double ff{}, fv{}, rss{};
    double common_minimum_variance{}, common_expo_variance{};
    unsigned count{};
    for (std::size_t level = 0U; level < MotorResponseProfile::kLevels; ++level) {
        const auto &plateau = *profile.plateau(direction, level);
        const double r = (plateau.applied_input.mean() - minimum) / range;
        if (r <= 0.0 || plateau.raw_speed.mean() <= 5.0 * noise) continue;
        const double shape = (1.0 - expo) * r + expo * r * r;
        if (!std::isfinite(shape) || shape <= 0.0) return false;
        const double sigma = response_sigma(plateau, noise);
        const double weight = 1.0 / (sigma * sigma);
        ff += weight * shape * shape;
        fv += weight * shape * plateau.raw_speed.mean();
        const double minimum_sensitivity = ((1.0 - expo) + 2.0 * expo * r) *
            (1.0 - r) / (shape * range);
        const double expo_sensitivity = (r * r - r) / shape;
        common_minimum_variance = std::max(common_minimum_variance,
            minimum_sensitivity * minimum_sensitivity * minimum_uncertainty * minimum_uncertainty);
        common_expo_variance = std::max(common_expo_variance,
            expo_sensitivity * expo_sensitivity * expo_variance);
        ++count;
    }
    if (count < 4U || !std::isfinite(ff) || ff <= kMinimumDivisor) return false;
    gain = fv / ff;
    if (!std::isfinite(gain) || gain <= 5.0 * noise) return false;
    for (std::size_t level = 0U; level < MotorResponseProfile::kLevels; ++level) {
        const auto &plateau = *profile.plateau(direction, level);
        const double r = (plateau.applied_input.mean() - minimum) / range;
        if (r <= 0.0 || plateau.raw_speed.mean() <= 5.0 * noise) continue;
        const double error = plateau.raw_speed.mean() - gain * ((1.0 - expo) * r + expo * r * r);
        const double sigma = response_sigma(plateau, noise);
        if (std::fabs(error) > std::max(3.0 * sigma, 0.10 * plateau.raw_speed.mean())) return false;
        rss += error * error / (sigma * sigma);
    }
    // MIN/EXPO 的不确定性是所有平台共享的误差，不能靠增加平台数除小；
    // 以各平台最大灵敏度另加共同误差项，避免 ASYM 得到虚假的高置信度。
    gain_variance = std::max(1.0, rss / (count - 1U)) / ff +
        gain * gain * (common_minimum_variance + common_expo_variance);
    return std::isfinite(gain_variance) && gain_variance > 0.0;
}

} // namespace

bool ResponseStatistics::add(std::uint64_t timestamp_us, float value) noexcept
{
    if (failure_ != ResponseFailure::None) return false;
    if (!std::isfinite(value)) failure_ = ResponseFailure::InvalidSample;
    if (count_ == UINT32_MAX) failure_ = ResponseFailure::CapacityExceeded;
    if (failure_ != ResponseFailure::None || !valid_timestamp(timestamp_us, last_timestamp_us_, failure_)) return false;
    if (count_ == 0U) {
        first_timestamp_us_ = timestamp_us;
        maximum_ = value;
    }
    last_timestamp_us_ = timestamp_us;
    maximum_ = std::max(maximum_, value);
    ++count_;
    const double difference = value - mean_;
    mean_ += difference / count_;
    m2_ += difference * (value - mean_);
    return true;
}

void ResponseStatistics::reset() noexcept { *this = {}; }

double ResponseStatistics::mean() const noexcept
{
    return count_ != 0U && failure_ == ResponseFailure::None ? mean_ : unavailable();
}

double ResponseStatistics::variance() const noexcept
{
    return count_ >= 2U && failure_ == ResponseFailure::None
        ? std::max(0.0, m2_) / (count_ - 1U) : unavailable();
}

float ResponseStatistics::standard_deviation() const noexcept
{
    return static_cast<float>(std::sqrt(variance()));
}

float ResponseStatistics::maximum() const noexcept
{
    return count_ != 0U && failure_ == ResponseFailure::None ? maximum_ : unavailable();
}

float ResponseStatistics::duration_s() const noexcept
{
    return count_ >= 2U && failure_ == ResponseFailure::None
        ? static_cast<float>(last_timestamp_us_ - first_timestamp_us_) * 1.0e-6F : 0.0F;
}

bool TransientSlope::add(std::uint64_t timestamp_us, float response) noexcept
{
    if (failure_ != ResponseFailure::None) return false;
    if (!std::isfinite(response)) failure_ = ResponseFailure::InvalidSample;
    if (count_ == UINT32_MAX) failure_ = ResponseFailure::CapacityExceeded;
    if (failure_ != ResponseFailure::None || !valid_timestamp(timestamp_us, last_timestamp_us_, failure_)) return false;
    if (count_ == 0U) {
        first_timestamp_us_ = timestamp_us;
        minimum_ = maximum_ = response;
    }
    const double seconds = static_cast<double>(timestamp_us - first_timestamp_us_) * 1.0e-6;
    last_timestamp_us_ = timestamp_us;
    minimum_ = std::min(minimum_, response);
    maximum_ = std::max(maximum_, response);
    ++count_;
    const double time_difference = seconds - mean_time_;
    const double response_difference = response - mean_response_;
    mean_time_ += time_difference / count_;
    mean_response_ += response_difference / count_;
    time_m2_ += time_difference * (seconds - mean_time_);
    response_m2_ += response_difference * (response - mean_response_);
    time_response_m2_ += time_difference * (response - mean_response_);
    return true;
}

void TransientSlope::reset() noexcept { *this = {}; }

float TransientSlope::signed_rate() const noexcept
{
    if (failure_ != ResponseFailure::None || count_ < 2U || time_m2_ <= kMinimumDivisor) return unavailable();
    const float rate = static_cast<float>(time_response_m2_ / time_m2_);
    return std::isfinite(rate) ? rate : unavailable();
}

ResponseEstimate TransientSlope::lower_confidence_rate(float measurement_noise) const noexcept
{
    if (failure_ != ResponseFailure::None) return rejected(failure_);
    if (!valid_noise(measurement_noise)) return rejected(ResponseFailure::InvalidConfiguration);
    if (count_ < 8U) return rejected(ResponseFailure::InsufficientSamples);
    const double span = static_cast<double>(maximum_) - minimum_;
    if (span <= 5.0 * measurement_noise || time_m2_ <= kMinimumDivisor)
        return rejected(ResponseFailure::InsufficientExcitation);
    const double slope = time_response_m2_ / time_m2_;
    const double rss = std::max(0.0, response_m2_ - time_response_m2_ * time_response_m2_ / time_m2_);
    // a_LCB=|a|-3*sqrt(max(RSS/(N-2),sigma_sensor^2)/sum((t-mean(t))^2))。
    // 零残差不等于传感器无噪声；显著弯曲/往复响应不能冒充稳定加速度斜率。
    const double variance = std::max(rss / (count_ - 2U),
        static_cast<double>(measurement_noise) * measurement_noise);
    const double lower = std::fabs(slope) - kConfidenceMultiplier * std::sqrt(variance / time_m2_);
    if (!std::isfinite(lower) || lower <= 0.0 || std::sqrt(rss / count_) > 0.20 * span)
        return rejected(ResponseFailure::UncertainResponse);
    return estimate(lower);
}

bool MotorResponseProfile::add(std::size_t direction, std::size_t level,
                               std::uint64_t timestamp_us, float pre_input,
                               float applied_input, float raw_speed) noexcept
{
    if (failure_ != ResponseFailure::None) return false;
    if (direction >= kDirections || level >= kLevels) failure_ = ResponseFailure::CapacityExceeded;
    else if (!std::isfinite(pre_input) || !std::isfinite(applied_input) || !std::isfinite(raw_speed) ||
             std::fabs(pre_input) > 1.0F || std::fabs(applied_input) > 1.0F)
        failure_ = ResponseFailure::InvalidSample;
    const float sign = direction == kForward ? 1.0F : -1.0F;
    if (failure_ == ResponseFailure::None && (sign * pre_input < 0.0F || sign * applied_input < 0.0F))
        failure_ = ResponseFailure::DirectionMismatch;
    if (failure_ != ResponseFailure::None || !valid_timestamp(timestamp_us, last_timestamp_us_, failure_)) return false;
    last_timestamp_us_ = timestamp_us;
    auto &plateau = plateaus_[direction][level];
    const bool accepted = plateau.pre_input.add(timestamp_us, sign * pre_input) &&
        plateau.applied_input.add(timestamp_us, sign * applied_input) &&
        plateau.raw_speed.add(timestamp_us, sign * raw_speed);
    if (!accepted) failure_ = ResponseFailure::InvalidSample;
    return accepted;
}

void MotorResponseProfile::reset() noexcept { *this = {}; }

const ResponsePlateau *MotorResponseProfile::plateau(std::size_t direction,
                                                     std::size_t level) const noexcept
{
    return direction < kDirections && level < kLevels ? &plateaus_[direction][level] : nullptr;
}

ResponseEstimate MotorResponseProfile::speed_lower_bound(std::size_t direction,
                                                          float measurement_noise) const noexcept
{
    if (failure_ != ResponseFailure::None) return rejected(failure_);
    if (direction >= kDirections || !valid_noise(measurement_noise)) return rejected(ResponseFailure::InvalidConfiguration);
    double best{};
    for (const auto &plateau : plateaus_[direction]) {
        if (!plateau_valid(plateau) || !input_stable(plateau)) continue;
        const double lower = plateau.raw_speed.mean() - kConfidenceMultiplier * response_sigma(plateau, measurement_noise);
        best = std::max(best, lower);
    }
    return best > 0.0 ? estimate(best) : rejected(ResponseFailure::InsufficientExcitation);
}

ResponseEstimate MotorResponseProfile::gain_lower_bound(std::size_t direction,
                                                         float measurement_noise) const noexcept
{
    if (failure_ != ResponseFailure::None) return rejected(failure_);
    if (direction >= kDirections || !valid_noise(measurement_noise))
        return rejected(ResponseFailure::InvalidConfiguration);
    double minimum = std::numeric_limits<double>::infinity();
    unsigned levels{};
    for (const auto &plateau : plateaus_[direction]) {
        if (!plateau_valid(plateau) || !input_stable(plateau)) continue;
        // 生产调用只需要整形前命令坐标；ARX和全局电机曲线仍各自使用真实
        // applied输入，不因删除未使用的坐标选项而改变物理辨识链。
        const auto &input = plateau.pre_input;
        const double sigma = response_sigma(plateau, measurement_noise);
        const double lower = plateau.raw_speed.mean() - kConfidenceMultiplier * sigma;
        if (lower <= 2.0 * measurement_noise || input.mean() <= kMinimumDivisor) continue;
        const double upper_input = input.mean() + kConfidenceMultiplier * input.standard_deviation();
        minimum = std::min(minimum, lower / upper_input);
        ++levels;
    }
    return levels >= 3U && minimum > 0.0 ? estimate(minimum)
                                        : rejected(ResponseFailure::InsufficientExcitation);
}

ResponseFailure MotorResponseProfile::global_coverage(float motor_max,
                                                       float measurement_noise) const noexcept
{
    if (failure_ != ResponseFailure::None) return failure_;
    if (!valid_noise(measurement_noise) || !std::isfinite(motor_max) || motor_max < 0.05F || motor_max > 1.0F)
        return ResponseFailure::InvalidConfiguration;
    for (std::size_t direction = 0U; direction < kDirections; ++direction) {
        double previous_input = -1.0, previous_speed{}, previous_sigma{};
        bool middle{};
        for (std::size_t level = 0U; level < kLevels; ++level) {
            const auto &plateau = plateaus_[direction][level];
            if (!plateau_valid(plateau)) return ResponseFailure::InsufficientSamples;
            if (!input_stable(plateau)) return ResponseFailure::UncertainResponse;
            const double applied = plateau.applied_input.mean();
            const double speed = plateau.raw_speed.mean();
            const double sigma = response_sigma(plateau, measurement_noise);
            if (plateau.applied_input.maximum() > motor_max || applied < 0.0 || speed < -3.0 * sigma)
                return ResponseFailure::DirectionMismatch;
            if (sigma > std::max(3.0 * measurement_noise, 0.10 * std::fabs(speed))) return ResponseFailure::UncertainResponse;
            if (level != 0U) {
                if (applied <= previous_input || applied - previous_input > 0.30 * motor_max)
                    return ResponseFailure::IncompleteCoverage;
                if (speed + 3.0 * sigma < previous_speed - 3.0 * previous_sigma)
                    return ResponseFailure::NonmonotonicResponse;
            }
            if (level == 0U && applied > 0.15 * motor_max) return ResponseFailure::IncompleteCoverage;
            if (level == kLevels - 1U && (applied < 0.95 * motor_max || speed <= 5.0 * sigma))
                return ResponseFailure::IncompleteCoverage;
            middle = middle || (applied >= 0.30 * motor_max && applied <= 0.70 * motor_max);
            previous_input = applied;
            previous_speed = speed;
            previous_sigma = sigma;
        }
        if (!middle) return ResponseFailure::IncompleteCoverage;
    }
    return ResponseFailure::None;
}

MotorResponseCandidate design_motor_response(const MotorResponseProfile &profile,
                                              const MotorResponseDesign &design) noexcept
{
    MotorResponseCandidate result{};
    if (!std::isfinite(design.current_min_output) || design.current_min_output < 0.0F ||
        design.current_min_output >= design.motor_max || !std::isfinite(design.current_expo) ||
        design.current_expo < -1.0F || design.current_expo > 1.0F ||
        !std::isfinite(design.current_asymmetry) || design.current_asymmetry < 1.0F || design.current_asymmetry > 10.0F) {
        result.failure = result.minimum_failure = result.expo_failure = result.asymmetry_failure = ResponseFailure::InvalidConfiguration;
        return result;
    }
    result.failure = profile.global_coverage(design.motor_max, design.measurement_noise);
    if (result.failure != ResponseFailure::None) {
        result.minimum_failure = result.expo_failure = result.asymmetry_failure = result.failure;
        return result;
    }
    result.global_coverage = true;
    double lower{}, upper = design.motor_max;
    for (std::size_t direction = 0U; direction < MotorResponseProfile::kDirections; ++direction) {
        const ResponsePlateau *last_stopped = nullptr;
        bool bracketed{};
        for (std::size_t level = 0U; level < MotorResponseProfile::kLevels; ++level) {
            const auto &plateau = *profile.plateau(direction, level);
            const double sigma = response_sigma(plateau, design.measurement_noise);
            if (std::fabs(plateau.raw_speed.mean()) <= sigma && plateau.applied_input.mean() > 0.0) {
                last_stopped = &plateau;
                continue;
            }
            if (plateau.raw_speed.mean() <= 5.0 * sigma) continue;
            if (last_stopped == nullptr) break;
            const double low = std::max(0.0, last_stopped->applied_input.mean() - 3.0 * last_stopped->applied_input.standard_deviation());
            const double high = std::min(static_cast<double>(design.motor_max), plateau.applied_input.mean() + 3.0 * plateau.applied_input.standard_deviation());
            if (high - low > 0.10 * design.motor_max) break;
            lower = std::max(lower, low);
            upper = std::min(upper, high);
            bracketed = true;
            break;
        }
        if (!bracketed) {
            result.failure = result.minimum_failure = result.expo_failure = result.asymmetry_failure = ResponseFailure::MinimumUnobservable;
            return result;
        }
    }
    // 公共 MIN 必须由两方向相交的“不动/能动”括界支持；仅有零输出静止点，
    // 或两个方向需要互不相容的起动阈值，都不产生默认零 MIN 候选。
    if (lower >= upper) {
        result.failure = result.minimum_failure = result.expo_failure = result.asymmetry_failure = ResponseFailure::InconsistentDirections;
        return result;
    }
    const double minimum = 0.5 * (lower + upper);
    result.min_output = static_cast<float>(minimum);
    result.minimum_lower_bound = static_cast<float>(lower);
    result.minimum_upper_bound = static_cast<float>(upper);
    result.identifiable_mask = MotorResponseCandidate::kMinimum;
    result.minimum_failure = ResponseFailure::None;
    const double minimum_uncertainty = 0.5 * (upper - lower);
    const auto forward = fit_curve(profile, MotorResponseProfile::kForward, minimum, minimum_uncertainty, design);
    const auto reverse = fit_curve(profile, MotorResponseProfile::kReverse, minimum, minimum_uncertainty, design);
    if (!forward.valid || !reverse.valid ||
        std::fabs(forward.expo - reverse.expo) > std::max(0.10, 3.0 * std::sqrt(forward.expo_variance + reverse.expo_variance))) {
        result.failure = result.expo_failure = result.asymmetry_failure = ResponseFailure::ModelRejected;
        return result;
    }
    const double expo = (forward.expo / forward.expo_variance + reverse.expo / reverse.expo_variance) /
        (1.0 / forward.expo_variance + 1.0 / reverse.expo_variance);
    const double expo_variance = std::max(forward.expo_variance, reverse.expo_variance);
    double gains[2]{}, variances[2]{};
    for (std::size_t direction = 0U; direction < MotorResponseProfile::kDirections; ++direction) {
        if (!fit_shared_expo(profile, direction, minimum, minimum_uncertainty, expo,
                expo_variance, design.measurement_noise,
                design.motor_max, gains[direction], variances[direction])) {
            result.failure = result.expo_failure = result.asymmetry_failure = ResponseFailure::InconsistentDirections;
            return result;
        }
    }
    result.expo = static_cast<float>(expo);
    result.identifiable_mask |= MotorResponseCandidate::kExpo;
    result.expo_failure = ResponseFailure::None;
    const double asymmetry = gains[0] / gains[1];
    const double relative_sigma = std::sqrt(variances[0] / (gains[0] * gains[0]) + variances[1] / (gains[1] * gains[1]));
    // 当前公共模型只允许加强倒车（ASYM>=1）；反向更强或区间内比例不恒定时
    // 不把候选钳成 1 冒充成功。实际保存还须由调用方验证全局 Manual 影响。
    if (!std::isfinite(asymmetry) || asymmetry < 1.0 || asymmetry > 10.0 ||
        !std::isfinite(relative_sigma) || 3.0 * relative_sigma > 0.20) {
        result.failure = result.asymmetry_failure = ResponseFailure::ModelRejected;
        return result;
    }
    result.asymmetry = static_cast<float>(asymmetry);
    result.identifiable_mask |= MotorResponseCandidate::kAsymmetry;
    result.failure = result.asymmetry_failure = ResponseFailure::None;
    return result;
}

} // namespace dima::lib::rover::calibration
