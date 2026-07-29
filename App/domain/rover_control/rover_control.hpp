#pragma once

#include <stdint.h>

namespace app::domain::rover_control {

enum class RoverControlMode : uint8_t {
    Stop,
    SpeedAndYawRate,
    SpeedAndHeading,
};

enum class RoverControlStatus : uint8_t {
    Ok,
    Stopped,
    InvalidParams,
    InvalidCommand,
    InvalidState,
    InvalidDt,
};

struct PidParams {
    float proportional;
    float integral;
    float derivative;
    float feedforward;
    float integral_limit;
    float output_limit;
};

struct RoverControlParams {
    PidParams yaw_rate_pid;
    PidParams speed_pid;
    float heading_p;
    float max_yaw_rate_rad_s;
    float max_yaw_accel_rad_s2;
    float max_yaw_decel_rad_s2;
    float max_speed_m_s;
    float max_accel_m_s2;
    float max_decel_m_s2;
    float spot_turn_enter_rad;
    float spot_turn_exit_rad;
    float state_timeout_s;
};

struct RoverVehicleState {
    uint64_t timestamp_us;
    float yaw_rad;
    float yaw_rate_rad_s;
    float body_speed_m_s;
    bool yaw_valid;
    bool yaw_rate_valid;
    bool speed_valid;
};

struct RoverControlCommand {
    RoverControlMode mode;
    float speed_m_s;
    float yaw_rate_rad_s;
    float heading_rad;
};

struct RoverControlOutput {
    float requested_speed_m_s;
    float requested_yaw_rate_rad_s;
    float normalized_throttle;
    float normalized_steering;
    float left_normalized;
    float right_normalized;
    bool spot_turn_active;
    bool throttle_limited;
    bool steering_limited;
    bool valid;
    RoverControlStatus status;
};

class RoverControllerChain {
public:
    bool configure(const RoverControlParams &params) noexcept;
    RoverControlOutput update(const RoverControlCommand &command,
                              const RoverVehicleState &state,
                              uint64_t now_us,
                              float dt_s) noexcept;
    void reset() noexcept;
    bool configured() const noexcept { return configured_; }

private:
    struct PidState {
        float integral{0.0F};
        float previous_error{0.0F};
        bool initialized{false};
    };

    static bool finite(float value) noexcept;
    static float clamp(float value, float lower, float upper) noexcept;
    static float wrap_pi(float angle) noexcept;
    static float slew(float current, float target, float accel, float decel,
                      float dt_s) noexcept;
    static bool valid_pid(const PidParams &params) noexcept;
    static RoverControlOutput zero_output(RoverControlStatus status) noexcept;
    static float update_pid(const PidParams &params, PidState &state,
                            float setpoint, float measurement, float dt_s,
                            bool freeze_integrator, bool &limited) noexcept;
    void reset_state() noexcept;

    RoverControlParams params_{};
    PidState yaw_rate_state_{};
    PidState speed_state_{};
    float limited_yaw_rate_{0.0F};
    float limited_speed_{0.0F};
    bool configured_{false};
    bool spot_turn_active_{false};
    bool steering_limited_previous_{false};
    bool throttle_limited_previous_{false};
};

} // namespace app::domain::rover_control
