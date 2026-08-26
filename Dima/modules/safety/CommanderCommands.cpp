/****************************************************************************
 * PX4-Autopilot v1.17.0 Commander Rover subset adapted to the Dima platform.
 ****************************************************************************/
#define MODULE_NAME "commander"
#include "Commander.hpp"

#include "calibration/SensorCalibrationAlgorithms.hpp"
#include "logging/logging.hpp"

#include <cstddef>
#include <cmath>
#include <cstring>

namespace dima::modules::safety {
namespace {

namespace calibration = dima::lib::sensors::calibration;

constexpr std::uint8_t kMavAutopilotSystemId = 1U;
constexpr std::uint8_t kMavAutopilotComponentId = 1U;

float command_parameter(std::uint32_t raw) noexcept
{
    // vehicle_command 保存 MAVLink float 的原始位模式；memcpy 避免别名规则未定义行为。
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
        // 0 表示广播；非本系统/组件的定向命令必须静默跳过，不能替其他组件 ACK。
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
            const calibration::PreflightCalibrationRequest request =
                calibration::classify_preflight_calibration_request(
                    parameters);
            const bool worker_status_fresh =
                sensor_calibration_status_.timestamp != 0U &&
                sensor_calibration_status_.timestamp <= now &&
                now - sensor_calibration_status_.timestamp <=
                    kSensorCalibrationStatusTimeoutUs;

            if (request ==
                    calibration::PreflightCalibrationRequest::Radio) {
                if (actuator_armed_.armed ||
                    vehicle_status_.calibration_enabled) {
                    result = vehicle_command_ack_s::
                        RESULT_TEMPORARILY_REJECTED;
                    break;
                }
                if (!vehicle_status_.rc_calibration_in_progress) {
                    vehicle_status_.rc_calibration_in_progress = true;
                    state_changed = true;
                    PX4_INFO("Calibration: Disabling RC control actions");
                }
                result = vehicle_command_ack_s::RESULT_ACCEPTED;
                break;
            }

            if (request ==
                    calibration::PreflightCalibrationRequest::Cancel) {
                if (vehicle_status_.rc_calibration_in_progress) {
                    vehicle_status_.rc_calibration_in_progress = false;
                    state_changed = true;
                    PX4_INFO("Calibration: Restoring RC control actions");
                }
                if (vehicle_status_.calibration_enabled) {
                    if (!worker_status_fresh) {
                        result = vehicle_command_ack_s::
                            RESULT_TEMPORARILY_REJECTED;
                    } else {
                        sensor_calibration_request_s worker_request{};
                        worker_request.timestamp = now;
                        worker_request.request =
                            sensor_calibration_request_s::REQUEST_CANCEL;
                        result =
                            sensor_calibration_request_publication_.publish(
                                worker_request)
                            ? vehicle_command_ack_s::RESULT_ACCEPTED
                            : vehicle_command_ack_s::RESULT_FAILED;
                    }
                } else {
                    // PX4 treats an idle all-zero calibration command as an
                    // accepted idempotent end/cancel request.
                    result = vehicle_command_ack_s::RESULT_ACCEPTED;
                }
                break;
            }

            std::uint8_t worker_request_type =
                sensor_calibration_request_s::REQUEST_NONE;
            if (request ==
                    calibration::PreflightCalibrationRequest::Gyro) {
                worker_request_type =
                    sensor_calibration_request_s::REQUEST_GYRO;
            } else if (request == calibration::
                           PreflightCalibrationRequest::Magnetometer) {
                worker_request_type =
                    sensor_calibration_request_s::REQUEST_MAG;
            } else if (request == calibration::
                           PreflightCalibrationRequest::Accelerometer) {
                worker_request_type =
                    sensor_calibration_request_s::REQUEST_ACCEL;
            }

            if (worker_request_type ==
                    sensor_calibration_request_s::REQUEST_NONE) {
                result = vehicle_command_ack_s::RESULT_UNSUPPORTED;
                break;
            }

            if (actuator_armed_.armed ||
                vehicle_status_.rc_calibration_in_progress ||
                vehicle_status_.calibration_enabled ||
                !worker_status_fresh) {
                result = vehicle_command_ack_s::
                    RESULT_TEMPORARILY_REJECTED;
                break;
            }

            sensor_calibration_request_s worker_request{};
            worker_request.timestamp = now;
            worker_request.request = worker_request_type;

            // PX4 Commander reserves the calibration state before starting
            // its low-priority worker. Keep the same ordering across Dima's
            // generated internal Topic so a worker callback cannot race an
            // apparently idle Commander state.
            vehicle_status_.calibration_enabled = true;
            sensor_calibration_dispatch_time_ = now;
            if (!sensor_calibration_request_publication_.publish(
                    worker_request)) {
                vehicle_status_.calibration_enabled = false;
                sensor_calibration_dispatch_time_ = 0U;
                result = vehicle_command_ack_s::RESULT_FAILED;
                break;
            }

            state_changed = true;
            result = vehicle_command_ack_s::RESULT_ACCEPTED;
            break;
        }

        case vehicle_command_s::NAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {
            // 仅 Disarmed 允许重启，避免 Armed 车辆因远端命令突然失去控制。
            if (actuator_armed_.armed) {
                result = vehicle_command_ack_s::RESULT_DENIED;
            } else {
                float param1 = 0.0F;
                std::memcpy(&param1, &cmd.param1_raw, sizeof(param1));
                const int mode = static_cast<int>(param1);
                if (mode == 0) {
                    // mode=0 是幂等空操作，按已接受返回但不触发复位。
                    result = vehicle_command_ack_s::RESULT_ACCEPTED;
                } else if (mode == 1 || mode == 3) {
                    // 1=普通复位，3=MCUboot Recovery。这里只把 mode 放入 ACK；
                    // MavlinkService 等 USB ACK 发送完成后才执行真实复位。
                    result = vehicle_command_ack_s::RESULT_ACCEPTED;
                    answer_command(cmd, result, now,
                                   static_cast<std::uint32_t>(mode));
                    PX4_INFO("Reboot (mode %d) permitted by Commander", mode);
                    continue;   // 已发送带复位 mode 的 ACK，跳过下方普通 ACK。
                } else {
                    result = vehicle_command_ack_s::RESULT_UNSUPPORTED;
                }
            }
            break;
        }

        case vehicle_command_s::NAV_CMD_REQUEST_MESSAGE:
            // MavlinkService 通过自己的订阅处理请求，Commander 不抢占消息所有权。
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
