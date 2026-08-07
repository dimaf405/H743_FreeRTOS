#pragma once
/*
 * MAVLink TIMESYNC handler — ported from PX4-Autopilot v1.17.0
 * (src/modules/mavlink/mavlink_timesync.cpp, commit d6f12ad).
 *
 * Semantics (unchanged from upstream):
 *   - tc1 == 0: message originates from the remote system — timestamp
 *     it (tc1 = local hrt time in NANOSECONDS) and return it.
 *   - tc1 > 0 : message originates from this system — feed the
 *     round-trip into the Timesync convergence filter.
 *
 * Dima adaptations: the SYSTEM_TIME clock-set leg is omitted (this
 * platform has no realtime clock), and the Mavlink& reference is
 * replaced by a caller-supplied transmit callback.
 */

#include "lib/mavlink/mavlink_bridge.h"
#include "lib/timesync/Timesync.hpp"
#include "platform/api/Time.hpp"

#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkTimesync {
public:
    /** Transmit callback: finalise + send the prepared mavlink_message_t. */
    using SendFn = void (*)(void *ctx, mavlink_message_t &msg);

    MavlinkTimesync(SendFn send, void *send_ctx) noexcept
        : send_(send), send_ctx_(send_ctx)
    {
    }

    /**
     * Handle an incoming TIMESYNC message (PX4 semantics).
     */
    void handle_message(const mavlink_message_t *msg) noexcept
    {
        mavlink_timesync_t tsync{};
        mavlink_msg_timesync_decode(msg, &tsync);

        const std::uint64_t now = hrt_absolute_time();

        if (tsync.tc1 == 0) {
            /* Message originating from remote system, timestamp and return it */
            mavlink_timesync_t rsync{};
            rsync.tc1 = static_cast<std::int64_t>(now * 1000ULL);
            rsync.ts1 = tsync.ts1;

            mavlink_message_t reply{};
            mavlink_msg_timesync_encode(1, 1, &reply, &rsync);
            if (send_ != nullptr) {
                send_(send_ctx_, reply);
            }

        } else if (tsync.tc1 > 0) {
            /* Message originating from this system, compute time offset */
            timesync_.update(now, tsync.tc1, tsync.ts1);
        }
    }

    /**
     * Convert remote timestamp to local hrt time (usec).
     * Uses synchronised time if available, monotonic boot time otherwise.
     */
    std::uint64_t sync_stamp(std::uint64_t usec) noexcept
    {
        return timesync_.sync_stamp(usec);
    }

    bool converged() const noexcept { return timesync_.sync_converged(); }

private:
    dima::lib::timesync::Timesync timesync_{};
    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
};

}  // namespace dima::modules::mavlink
