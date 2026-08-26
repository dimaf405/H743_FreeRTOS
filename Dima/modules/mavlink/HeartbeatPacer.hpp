#pragma once
/*
 * HEARTBEAT periodic pacer + AUTOPILOT_VERSION on-demand responder.
 *
 * QGC expects HEARTBEAT at ~1 Hz to keep the link alive; the
 * AUTOPILOT_VERSION message is sent once at connect time and again
 * in response to MAV_CMD_REQUEST_MESSAGE(148).
 *
 * This is a pure message-filling helper — the owning MavlinkService
 * decides when to call tick() and how to transmit the resulting frames.
 *
 * 1 Hz 仅是链路存活节拍，不代表传感器或控制循环频率。base_mode、
 * custom_mode 与 system_status 必须来自 Commander 发布状态，不能自行推测。
 */

#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"
#include "mavlink/MavlinkBridge.h"
#include "mavlink_stream_contract.hpp"
#include "uorb/SubscriptionData.hpp"

#include "MavlinkIdentity.hpp"

#include <cstdint>

namespace dima::modules::mavlink {

class HeartbeatPacer {
public:
    static constexpr std::uint64_t kIntervalUs =
        static_cast<std::uint64_t>(
            dima::generated::mavlink_streams::default_interval_us(
                dima::generated::mavlink_streams::
                    MessageHandler::Heartbeat));
    static_assert(kIntervalUs > 0U,
                  "generated HEARTBEAT interval must be enabled");
    // PX4 custom_mode 的 main_mode 位于高 16 bit：Manual=1，Termination=10。
    static constexpr std::uint32_t kPx4CustomModeManual = 1UL << 16;
    static constexpr std::uint32_t kPx4CustomModeTermination = 10UL << 16;

    explicit HeartbeatPacer(MavlinkIdentity &identity) noexcept;

    /**
     * Periodic tick: packs a HEARTBEAT message if the interval elapsed.
     * 只有真正打包成功才推进 last_send_us_；发送失败由 owner reset 后重试。
     *
     * @param now_us    Current monotonic time in microseconds.
     * @param[out] msg  Filled HEARTBEAT message when true is returned.
     * @return          true if a message was packed (send it).
     */
    bool tick(std::uint64_t now_us, mavlink_message_t &msg) noexcept;

    void pack_now(std::uint64_t now_us, mavlink_message_t &msg) noexcept;

    /**
     * Pack an AUTOPILOT_VERSION message (on request or at connect).
     * 版本、能力和 UID 均来自 MavlinkIdentity 的已配置快照。
     */
    void pack_autopilot_version(mavlink_message_t &msg) const noexcept;

    void reset() noexcept;

private:
    /**
     * Project vehicle_status_s into HEARTBEAT base_mode/system_status
     * via MavlinkIdentity.
     * 仅投影现有 Topic，不反向改变 Commander 状态。
     */
    void refresh_state_from_orb() noexcept;

    MavlinkIdentity &identity_;
    uORB::SubscriptionData<vehicle_status_s> status_subscription_{
        ORB_ID(vehicle_status)};
    uORB::SubscriptionData<vehicle_control_mode_s> control_mode_subscription_{
        ORB_ID(vehicle_control_mode)};
    std::uint64_t last_send_us_{0};
    std::uint32_t custom_mode_{kPx4CustomModeManual};
};

}  // namespace dima::modules::mavlink
