#pragma once
/*
 * MAVLink command reception — ported from PX4-Autopilot v1.17.0
 * src/modules/mavlink/mavlink_receiver.cpp (commit d6f12ad):
 * evaluate_target_ok / acknowledge / handle_message_command_long /
 * handle_message_command_int / handle_message_command_both /
 * handle_request_message_command.
 *
 * Architecture is unchanged from PX4: the receiver does framing-level
 * target filtering and protocol translation, publishes vehicle_command
 * for Commander arbitration, and publishes vehicle_command_ack for
 * locally served requests. MavlinkService turns the acks back into
 * MAVLink COMMAND_ACK frames.
 *
 * Dima adaptations:
 *   - COMMAND_INT is accepted: handle_message_command_int is ported
 *     1:1 from upstream and shares handle_message_command_both with
 *     the COMMAND_LONG path (Phase-6 dialect extension).
 *   - Stream-interval commands are answered Unsupported until the
 *     periodic stream set gains interval control.
 *   - Autotune / failure injection / logging commands are omitted.
 *   - handle_request_message_command resolves via a caller callback
 *     instead of the PX4 stream list.
 *
 * 结果码语义矩阵（对 QGC 的可观测行为）：
 *   UNSUPPORTED — 命令不在能力清单（Commander default 分支 /
 *     REQUEST_MESSAGE 未知消息 id / SET·GET_MESSAGE_INTERVAL）。
 *   DENIED — 安全策略拒绝（armed 时 reboot 由 Commander 裁决；
 *     LONG 路径 param5/6、INT 路径 x/y 误用编码拒绝）。
 *   target 不匹配 — 静默丢弃（协议规定不对非本机命令应答）。
 */

#include "vehicle_command.hpp"
#include "vehicle_command_ack.hpp"
#include "lib/mavlink/mavlink_bridge.h"
#include "uorb/Publication.hpp"

#include <cstdint>

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
                    void *request_ctx) noexcept;

    void handle_message(const mavlink_message_t *msg) noexcept;

private:
    bool evaluate_target_ok(std::uint16_t command, std::uint8_t target_system,
                            std::uint8_t target_component) const noexcept;

    void acknowledge(std::uint8_t sysid, std::uint8_t compid,
                     std::uint16_t command, std::uint8_t result,
                     std::uint8_t progress = 0) noexcept;

    std::uint8_t handle_request_message_command(
        std::uint16_t message_id) noexcept;

    void handle_message_command_long(const mavlink_message_t *msg) noexcept;

    /* 移植自 PX4 v1.17.0 mavlink_receiver.cpp handle_message_command_int
     * （与 handle_message_command_long 对偶）：NAN 误发为 int 检查
     * （0x7ff80000，对应 LONG 路径的 INT32_MAX 误用检查）、x/y 按
     * ×1e-7 缩放填 param5/6、INT32_MAX 表示未用→NAN、z 填 param7；
     * confirmation 固定 false（COMMAND_INT 无该字段）。 */
    void handle_message_command_int(const mavlink_message_t *msg) noexcept;

    void handle_message_command_both(const mavlink_message_t *msg,
                                     std::uint16_t command,
                                     std::uint8_t target_system,
                                     std::uint8_t target_component,
                                     float param1,
                                     const vehicle_command_s &vehicle_command)
        noexcept;

    RequestMessageFn request_message_{nullptr};
    void *request_ctx_{nullptr};
    uORB::Publication<vehicle_command_s> _cmd_pub{ORB_ID(vehicle_command)};
    uORB::Publication<vehicle_command_ack_s> _cmd_ack_pub{
        ORB_ID(vehicle_command_ack)};
};

}  // namespace dima::modules::mavlink
