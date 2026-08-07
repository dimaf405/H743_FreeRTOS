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
 */

#include "vehicle_status.hpp"
#include "lib/mavlink/mavlink_bridge.h"
#include "uorb/SubscriptionData.hpp"

#include "MavlinkIdentity.hpp"

#include <cstdint>
#include <cstring>

namespace dima::modules::mavlink {

class HeartbeatPacer {
public:
    static constexpr std::uint64_t kIntervalUs = 1000000ULL;  /* 1 Hz */

    explicit HeartbeatPacer(MavlinkIdentity &identity) noexcept
        : identity_(identity)
    {
    }

    /**
     * Periodic tick: packs a HEARTBEAT message if the interval elapsed.
     *
     * @param now_us    Current monotonic time in microseconds.
     * @param[out] msg  Filled HEARTBEAT message when true is returned.
     * @return          true if a message was packed (send it).
     */
    bool tick(std::uint64_t now_us, mavlink_message_t &msg) noexcept
    {
        if (now_us - last_send_us_ < kIntervalUs) {
            return false;
        }
        last_send_us_ = now_us;

        refresh_state_from_orb();

        mavlink_heartbeat_t heartbeat{};
        heartbeat.custom_mode = 0;
        heartbeat.type = MavlinkIdentity::MAV_TYPE_VALUE;
        heartbeat.autopilot = MavlinkIdentity::MAV_AUTOPILOT_VALUE;
        heartbeat.base_mode = identity_.base_mode();
        heartbeat.system_status = identity_.system_status();
        heartbeat.mavlink_version = MAVLINK_VERSION;

        mavlink_msg_heartbeat_encode(MavlinkIdentity::SYSTEM_ID,
                                     MavlinkIdentity::COMPONENT_ID,
                                     &msg, &heartbeat);
        return true;
    }

    /**
     * Pack an AUTOPILOT_VERSION message (on request or at connect).
     */
    void pack_autopilot_version(mavlink_message_t &msg) const noexcept
    {
        mavlink_autopilot_version_t version{};
        version.capabilities = identity_.capabilities();
        version.flight_sw_version = identity_.flight_sw_version();
        version.middleware_sw_version = 0;
        version.os_sw_version = 0;
        version.board_version = identity_.board_version();
        identity_.get_flight_custom_version(version.flight_custom_version);
        identity_.get_middleware_custom_version(
            version.middleware_custom_version);
        identity_.get_os_custom_version(version.os_custom_version);
        version.vendor_id = identity_.vendor_id();
        version.product_id = identity_.product_id();
        version.uid = identity_.uid();
        std::memset(version.uid2, 0, sizeof(version.uid2));

        mavlink_msg_autopilot_version_encode(MavlinkIdentity::SYSTEM_ID,
                                             MavlinkIdentity::COMPONENT_ID,
                                             &msg, &version);
    }

    /** Force the next tick() to fire immediately (e.g. after reboot ack). */
    void kick() noexcept { last_send_us_ = 0; }

private:
    /**
     * Project vehicle_status_s into HEARTBEAT base_mode/system_status
     * via MavlinkIdentity.
     */
    void refresh_state_from_orb() noexcept
    {
        if (status_subscription_.update()) {
            const vehicle_status_s &s = status_subscription_.get();

            std::uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            if (s.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
            }

            std::uint8_t system_status;
            if (s.failsafe) {
                system_status = MAV_STATE_CRITICAL;
            } else if (s.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                system_status = MAV_STATE_ACTIVE;
            } else {
                system_status = MAV_STATE_STANDBY;
            }

            identity_.set_state(base_mode, system_status);
        }
    }

    MavlinkIdentity &identity_;
    uORB::SubscriptionData<vehicle_status_s> status_subscription_{
        ORB_ID(vehicle_status)};
    std::uint64_t last_send_us_{0};
};

}  // namespace dima::modules::mavlink
