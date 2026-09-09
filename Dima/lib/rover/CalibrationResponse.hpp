#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace dima::lib::rover::calibration {

enum class ResponseFailure : std::uint8_t {
    None,
    InvalidConfiguration,
    InvalidSample,
    DuplicateTimestamp,
    TimeRegression,
    CapacityExceeded,
    InsufficientSamples,
    InsufficientExcitation,
    UncertainResponse,
    DirectionMismatch,
    IncompleteCoverage,
    NonmonotonicResponse,
    InconsistentDirections,
    MinimumUnobservable,
    ModelRejected,
};

struct ResponseEstimate {
    float value{std::numeric_limits<float>::quiet_NaN()};
    ResponseFailure failure{ResponseFailure::InsufficientSamples};
    bool valid() const noexcept { return failure == ResponseFailure::None; }
};

// 单变量 Welford；保留真实零方差，不人为补噪声。调用方负责保证停车/稳态、
// 传感器设备和参数代次一致，重复或回退时间会锁存失败直到显式 reset。
class ResponseStatistics final {
public:
    bool add(std::uint64_t timestamp_us, float value) noexcept;
    void reset() noexcept;
    std::uint32_t count() const noexcept { return count_; }
    double mean() const noexcept;
    double variance() const noexcept;
    float standard_deviation() const noexcept;
    float maximum() const noexcept;
    float duration_s() const noexcept;
    ResponseFailure failure() const noexcept { return failure_; }

private:
    double mean_{};
    double m2_{};
    float maximum_{};
    std::uint64_t first_timestamp_us_{};
    std::uint64_t last_timestamp_us_{};
    std::uint32_t count_{};
    ResponseFailure failure_{ResponseFailure::None};
};

// 调用方只喂非零升/降阶跃的单调响应区间，不把最终零命令的自由停车当成
// RO_DECEL_LIM 生效证据。固定中心矩避免绝对时间戳使最小二乘发生相消。
class TransientSlope final {
public:
    bool add(std::uint64_t timestamp_us, float response) noexcept;
    void reset() noexcept;
    std::uint32_t count() const noexcept { return count_; }
    ResponseFailure failure() const noexcept { return failure_; }
    float signed_rate() const noexcept;
    ResponseEstimate lower_confidence_rate(float measurement_noise) const noexcept;

private:
    double mean_time_{};
    double mean_response_{};
    double time_m2_{};
    double response_m2_{};
    double time_response_m2_{};
    float minimum_{};
    float maximum_{};
    std::uint64_t first_timestamp_us_{};
    std::uint64_t last_timestamp_us_{};
    std::uint32_t count_{};
    ResponseFailure failure_{ResponseFailure::None};
};

struct ResponsePlateau {
    // 三项都按 direction 归一化：reverse 的负命令/负车速乘 -1；raw_speed
    // 不取绝对值，真实反向响应和零速噪声仍保留符号，不伪装成正响应。
    ResponseStatistics pre_input{};
    ResponseStatistics applied_input{};
    ResponseStatistics raw_speed{};
    std::uint32_t count() const noexcept { return raw_speed.count(); }
};

class MotorResponseProfile final {
public:
    static constexpr std::size_t kDirections{2U};
    static constexpr std::size_t kLevels{6U};
    static constexpr std::size_t kForward{0U};
    static constexpr std::size_t kReverse{1U};

    bool add(std::size_t direction, std::size_t level,
             std::uint64_t timestamp_us, float pre_input,
             float applied_input, float raw_speed) noexcept;
    void reset() noexcept;
    const ResponsePlateau *plateau(std::size_t direction,
                                   std::size_t level) const noexcept;
    bool replace_direction(std::size_t direction, const MotorResponseProfile &source,
                           std::size_t source_direction) noexcept;
    bool reset_plateau(std::size_t direction, std::size_t level) noexcept;
    ResponseFailure failure() const noexcept { return failure_; }

    // 速度取已测平台的最大保守响应；增益取至少三档有效平台比值的最小下界。
    // 增益下界不是“整条曲线线性”的证明，不能单独授权 RO_MAX_THR_SPEED 保存。
    ResponseEstimate speed_lower_bound(std::size_t direction,
                                       float measurement_noise) const noexcept;
    ResponseEstimate gain_lower_bound(std::size_t direction,
                                      float measurement_noise) const noexcept;
    ResponseFailure global_coverage(float motor_max,
                                    float measurement_noise) const noexcept;

private:
    ResponsePlateau plateaus_[kDirections][kLevels]{};
    std::uint64_t last_timestamp_us_{};
    ResponseFailure failure_{ResponseFailure::None};
};

struct MotorResponseDesign {
    float motor_max{};
    float measurement_noise{};
    float current_min_output{};
    float current_expo{};
    float current_asymmetry{1.0F};
};

struct MotorResponseCandidate {
    static constexpr std::uint8_t kMinimum{1U};
    static constexpr std::uint8_t kExpo{2U};
    static constexpr std::uint8_t kAsymmetry{4U};
    static constexpr std::uint8_t kAll{kMinimum | kExpo | kAsymmetry};

    float min_output{std::numeric_limits<float>::quiet_NaN()};
    float minimum_lower_bound{std::numeric_limits<float>::quiet_NaN()};
    float minimum_upper_bound{std::numeric_limits<float>::quiet_NaN()};
    float expo{std::numeric_limits<float>::quiet_NaN()};
    float asymmetry{std::numeric_limits<float>::quiet_NaN()};
    std::uint8_t identifiable_mask{};
    bool global_coverage{};
    ResponseFailure failure{ResponseFailure::InsufficientSamples};
    ResponseFailure minimum_failure{ResponseFailure::InsufficientSamples};
    ResponseFailure expo_failure{ResponseFailure::InsufficientSamples};
    ResponseFailure asymmetry_failure{ResponseFailure::InsufficientSamples};
    bool complete() const noexcept
    {
        return failure == ResponseFailure::None && global_coverage &&
               identifiable_mask == kAll;
    }
};

// 仅设计 MIN/EXPO/ASYM 候选，不触碰参数、输出、MOT_MAX、Arm ramp、换向延时。
// 满覆盖也是必要而非充分条件；调用方仍须实际验证关联 FF/PI 和全部运行范围。
// MOT_SLEW_RATE 没有静态曲线候选，必须由真实限制器激活证据另行比较。
MotorResponseCandidate design_motor_response(const MotorResponseProfile &profile,
                                              const MotorResponseDesign &design) noexcept;

} // namespace dima::lib::rover::calibration
