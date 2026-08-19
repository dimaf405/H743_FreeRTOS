#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

constexpr std::size_t kActuatorPwmChannelCount = 6U;

enum class ActuatorPwmResult : std::uint8_t {
    Applied,
    Retry,
    Fault,
};

struct ActuatorPwmFrame {
    std::uint16_t pulse_us[kActuatorPwmChannelCount]{};
    std::uint8_t enabled_mask{0U};
};

class ActuatorPwm {
public:
    virtual ~ActuatorPwm() = default;
    virtual ActuatorPwmResult start() noexcept = 0;
    virtual ActuatorPwmResult stop() noexcept = 0;
    virtual ActuatorPwmResult write(const ActuatorPwmFrame &frame) noexcept = 0;
    virtual bool started() const noexcept = 0;
};

} // namespace dima::platform
