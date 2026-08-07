#pragma once
/*
 * MAVLink command reception — ported from PX4-Autopilot v1.17.0
 * src/modules/mavlink/mavlink_receiver.cpp (commit d6f12ad):
 * evaluate_target_ok / acknowledge / handle_message_command_long /
 * handle_message_command_both / handle_request_message_command.
 *
 * Architecture is unchanged from PX4: the receiver does framing-level
 * target filtering and protocol translation, publishes vehicle_command
 * for Commander arbitration, and publishes vehicle_command_ack for
 * locally served requests. MavlinkService turns the acks back into
 * MAVLink COMMAND_ACK frames.
 *
 * Dima adaptations:
 *   - COMMAND_INT is not accepted (not in the allowlist).
 *   - Stream-interval commands are answered Unsupported until the
 *     periodic stream set gains interval control.
 *   - Autotune / failure injection / logging commands are omitted.
 *   - handle_request_message_command resolves via a caller callback
 *     instead of the PX4 stream list.
 */

#include "vehicle_command.hpp"
#include "vehicle_command_ack.hpp"
#include "lib/mavlink/mavlink_bridge.h"
#include "logging/logging.hpp"
#include "platform/api/Time.hpp"
#include "uorb/Publication.hpp"

#include <cmath>
#include <cstring>

namespace dima::modules::mavlink {

class MavlinkCommands {
public:
    /**
     * Resolve MAV_CMD_REQUEST_MESSAGE: send the requested message once.
     * Returns the MAV_RESULT used for the ack.
     */
    using RequestMessageFn = std::uint8_t (*)(void *ctx,
                                              std::uint16_t message_id);

    MavlinkCommands(RequestMessageFn request_message,
                    void *request_ctx) noexcept
        : request_message_(request_message), request_ctx_(request_ctx)
    {
    }

    void handle_message(const mavlink_message_t *msg) noexcept
    {
        switch (msg->msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_message_command_long(msg);
            break;

        default:
            break;
        }
    }

private:
    bool evaluate_target_ok(std::uint16_t command, std::uint8_t target_system,
                            std::uint8_t target_component) const noexcept
    {
        /* evaluate if this system should accept this command */
        bool target_ok = false;

        switch (command) {

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        case MAV_CMD_REQUEST_PROTOCOL_VERSION:
            /* broadcast and ignore component */
            target_ok = (target_system == 0) ||
                        (target_system == MAVLINK_SYSTEM_ID);
            break;

        default:
            target_ok = (target_system == MAVLINK_SYSTEM_ID) &&
                        ((target_component == MAVLINK_COMPONENT_ID) ||
                         (target_component == MAV_COMP_ID_ALL));
            break;
        }

        return target_ok;
    }

    void acknowledge(std::uint8_t sysid, std::uint8_t compid,
                     std::uint16_t command, std::uint8_t result,
                     std::uint8_t progress = 0) noexcept
    {
        vehicle_command_ack_s command_ack{};

        command_ack.timestamp = hrt_absolute_time();
        command_ack.command = command;
        command_ack.result = result;
        command_ack.target_system = sysid;
        command_ack.target_component = compid;
        command_ack.result_param2 = progress;

        _cmd_ack_pub.publish(command_ack);
    }

    std::uint8_t handle_request_message_command(std::uint16_t message_id) noexcept
    {
        bool message_sent = false;

        if (request_message_ != nullptr) {
            message_sent = request_message_(request_ctx_, message_id);
        }

        return (message_sent ? vehicle_command_ack_s::RESULT_ACCEPTED :
                vehicle_command_ack_s::RESULT_UNSUPPORTED);
    }

    void handle_message_command_long(const mavlink_message_t *msg) noexcept
    {
        /* command */
        mavlink_command_long_t cmd_mavlink;
        mavlink_msg_command_long_decode(msg, &cmd_mavlink);

        vehicle_command_s vcmd{};

        vcmd.timestamp = hrt_absolute_time();

        const float before_int32_max = std::nextafter((float)INT32_MAX, 0.0f);
        const float after_int32_max =
            std::nextafter((float)INT32_MAX, (float)INFINITY);

        if (cmd_mavlink.param5 >= before_int32_max &&
            cmd_mavlink.param5 <= after_int32_max &&
            cmd_mavlink.param6 >= before_int32_max &&
            cmd_mavlink.param6 <= after_int32_max) {
            // This looks suspiciously like INT32_MAX was sent in a COMMAND_LONG
            // instead of a COMMAND_INT.
            PX4_ERR("param5/param6 invalid of command %u", cmd_mavlink.command);
            acknowledge(msg->sysid, msg->compid, cmd_mavlink.command,
                        vehicle_command_ack_s::RESULT_DENIED);
            return;
        }

        /* Copy the content of mavlink_command_long_t into vehicle_command_s,
         * keeping the raw float bit patterns (Dima contract). */
        std::memcpy(&vcmd.param1_raw, &cmd_mavlink.param1, sizeof(float));
        std::memcpy(&vcmd.param2_raw, &cmd_mavlink.param2, sizeof(float));
        std::memcpy(&vcmd.param3_raw, &cmd_mavlink.param3, sizeof(float));
        std::memcpy(&vcmd.param4_raw, &cmd_mavlink.param4, sizeof(float));
        std::memcpy(&vcmd.param5_raw, &cmd_mavlink.param5, sizeof(float));
        std::memcpy(&vcmd.param6_raw, &cmd_mavlink.param6, sizeof(float));
        std::memcpy(&vcmd.param7_raw, &cmd_mavlink.param7, sizeof(float));
        vcmd.command = cmd_mavlink.command;
        vcmd.target_system = cmd_mavlink.target_system;
        vcmd.target_component = cmd_mavlink.target_component;
        vcmd.source_component = msg->compid;
        vcmd.confirmation = cmd_mavlink.confirmation;
        vcmd.from_external = true;

        handle_message_command_both(msg, cmd_mavlink, vcmd);
    }

    void handle_message_command_both(const mavlink_message_t *msg,
                                     const mavlink_command_long_t &cmd_mavlink,
                                     const vehicle_command_s &vehicle_command) noexcept
    {
        bool target_ok = evaluate_target_ok(cmd_mavlink.command,
                                            cmd_mavlink.target_system,
                                            cmd_mavlink.target_component);
        bool send_ack = true;
        std::uint8_t result = vehicle_command_ack_s::RESULT_ACCEPTED;
        std::uint8_t progress = 0;

        if (!target_ok) {
            PX4_INFO("Ignore command %d from %d/%d to %d/%d",
                     cmd_mavlink.command, msg->sysid, msg->compid,
                     cmd_mavlink.target_system, cmd_mavlink.target_component);
            return;
        }

        // First we handle legacy support requests which were used before we had
        // the generic MAV_CMD_REQUEST_MESSAGE.
        if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
            result = handle_request_message_command(MAVLINK_MSG_ID_AUTOPILOT_VERSION);

        } else if (cmd_mavlink.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
            result = handle_request_message_command(MAVLINK_MSG_ID_PROTOCOL_VERSION);

        } else if (cmd_mavlink.command == MAV_CMD_SET_MESSAGE_INTERVAL ||
                   cmd_mavlink.command == MAV_CMD_GET_MESSAGE_INTERVAL) {
            /* Stream interval control is not implemented yet. */
            result = vehicle_command_ack_s::RESULT_UNSUPPORTED;

        } else if (cmd_mavlink.command == MAV_CMD_REQUEST_MESSAGE) {

            std::uint16_t message_id = (std::uint16_t)std::lroundf(
                cmd_mavlink.param1);
            result = handle_request_message_command(message_id);

        } else {
            send_ack = false;

            if (msg->sysid == MAVLINK_SYSTEM_ID &&
                msg->compid == MAVLINK_COMPONENT_ID) {
                PX4_WARN("ignoring CMD with same SYS/COMP (%d/%d) ID",
                         MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID);
                return;
            }

            /* Commander performs the sole safety arbitration and acks. */
            _cmd_pub.publish(vehicle_command);
        }

        if (send_ack) {
            acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, result,
                        progress);
        }
    }

    RequestMessageFn request_message_{nullptr};
    void *request_ctx_{nullptr};
    uORB::Publication<vehicle_command_s> _cmd_pub{ORB_ID(vehicle_command)};
    uORB::Publication<vehicle_command_ack_s> _cmd_ack_pub{
        ORB_ID(vehicle_command_ack)};
};

}  // namespace dima::modules::mavlink
