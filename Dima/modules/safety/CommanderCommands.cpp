/****************************************************************************
 * PX4-Autopilot v1.17.0 Commander Rover subset adapted to the Dima platform.
 ****************************************************************************/
#define MODULE_NAME "commander"
#include "Commander.hpp"

#include "logging/logging.hpp"

#include <cstddef>
#include <cmath>
#include <cstring>

namespace dima::modules::safety {
namespace {

constexpr std::uint8_t kMavAutopilotSystemId = 1U;
constexpr std::uint8_t kMavAutopilotComponentId = 1U;

float command_parameter(std::uint32_t raw) noexcept
{
    float value = 0.0F;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

} // namespace

bool Commander::handle_vehicle_command(std::uint64_t now) noexcept
{
    vehicle_command_s cmd{};
    bool state_changed = false;
    while (vehicle_command_subscription_.copy(&cmd)) {
        if ((cmd.target_system != 0U &&
             cmd.target_system != kMavAutopilotSystemId) ||
            (cmd.target_component != 0U &&
             cmd.target_component != kMavAutopilotComponentId)) {
            continue;
        }
        std::uint8_t result = vehicle_command_ack_s::RESULT_UNSUPPORTED;

        switch (cmd.command) {
        case vehicle_command_s::NAV_CMD_COMPONENT_ARM_DISARM: {
            float param1 = 0.0F;
            std::memcpy(&param1, &cmd.param1_raw, sizeof(param1));
            int action = -1;
            if (std::isfinite(param1) && param1 >= -0.5F && param1 <= 1.5F) {
                action = static_cast<int>(std::lround(param1));
            }
            if (action != 0 && action != 1) {
                result = vehicle_command_ack_s::RESULT_UNSUPPORTED;
            } else {
                const std::uint8_t reason = cmd.from_external
                    ? vehicle_status_s::ARM_DISARM_REASON_COMMAND_EXTERNAL
                    : vehicle_status_s::ARM_DISARM_REASON_COMMAND_INTERNAL;
                const TransitionResult transition = action == 1
                    ? arm(reason, now) : disarm(reason);
                state_changed = transition == TransitionResult::Changed ||
                                state_changed;
                result = transition == TransitionResult::Denied
                    ? vehicle_command_ack_s::RESULT_TEMPORARILY_REJECTED
                    : vehicle_command_ack_s::RESULT_ACCEPTED;
            }
            break;
        }

        case vehicle_command_s::NAV_CMD_PREFLIGHT_CALIBRATION: {
            const float parameters[7]{
                command_parameter(cmd.param1_raw),
                command_parameter(cmd.param2_raw),
                command_parameter(cmd.param3_raw),
                command_parameter(cmd.param4_raw),
                command_parameter(cmd.param5_raw),
                command_parameter(cmd.param6_raw),
                command_parameter(cmd.param7_raw),
            };
            bool all_zero = true;
            bool rc_start = true;
            for (std::size_t index = 0U; index < 7U; ++index) {
                const bool expected_start_value = index == 3U
                    ? parameters[index] == 1.0F
                    : parameters[index] == 0.0F;
                all_zero = all_zero && std::isfinite(parameters[index]) &&
                           parameters[index] == 0.0F;
                rc_start = rc_start && std::isfinite(parameters[index]) &&
                           expected_start_value;
            }

            if (actuator_armed_.armed) {
                result =
                    vehicle_command_ack_s::RESULT_TEMPORARILY_REJECTED;
            } else if (rc_start) {
                if (!vehicle_status_.rc_calibration_in_progress) {
                    vehicle_status_.rc_calibration_in_progress = true;
                    state_changed = true;
                    PX4_INFO("Calibration: Disabling RC control actions");
                }
                result = vehicle_command_ack_s::RESULT_ACCEPTED;
            } else if (all_zero) {
                if (vehicle_status_.rc_calibration_in_progress) {
                    vehicle_status_.rc_calibration_in_progress = false;
                    state_changed = true;
                    PX4_INFO("Calibration: Restoring RC control actions");
                }
                result = vehicle_command_ack_s::RESULT_ACCEPTED;
            } else {
                result = vehicle_command_ack_s::RESULT_UNSUPPORTED;
            }
            break;
        }

        case vehicle_command_s::NAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {
            /* Only permit reboot when disarmed. */
            if (actuator_armed_.armed) {
                result = vehicle_command_ack_s::RESULT_DENIED;
            } else {
                float param1 = 0.0F;
                std::memcpy(&param1, &cmd.param1_raw, sizeof(param1));
                const int mode = static_cast<int>(param1);
                if (mode == 0) {
                    /* Idempotent no-op request. */
                    result = vehicle_command_ack_s::RESULT_ACCEPTED;
                } else if (mode == 1 || mode == 3) {
                    /* 1 = normal reset, 3 = MCUboot Recovery. The actual
                     * reset is deferred to MavlinkService so the ACK is
                     * delivered over USB first. */
                    result = vehicle_command_ack_s::RESULT_ACCEPTED;
                    answer_command(cmd, result, now,
                                   static_cast<std::uint32_t>(mode));
                    PX4_INFO("Reboot (mode %d) permitted by Commander", mode);
                    continue;   /* skip the normal ACK below */
                } else {
                    result = vehicle_command_ack_s::RESULT_UNSUPPORTED;
                }
            }
            break;
        }

        case vehicle_command_s::NAV_CMD_REQUEST_MESSAGE:
            /* Handled by MavlinkService directly via its own subscription. */
            result = vehicle_command_ack_s::RESULT_UNSUPPORTED;
            break;

        default:
            PX4_WARN("Commander: unsupported command %u", cmd.command);
            result = vehicle_command_ack_s::RESULT_UNSUPPORTED;
            break;
        }

        answer_command(cmd, result, now);
    }
    return state_changed;
}

void Commander::answer_command(const vehicle_command_s &command,
                               std::uint8_t result, std::uint64_t now,
                               std::uint32_t result_param2) noexcept
{
    vehicle_command_ack_s ack{};
    ack.timestamp = now;
    ack.result_param2 = result_param2;
    ack.command = command.command;
    ack.result = result;
    ack.from_external = command.from_external;
    ack.target_system = command.source_system;
    ack.target_component = command.source_component;
    (void)vehicle_command_ack_publication_.publish(ack);
}

} // namespace dima::modules::safety
