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

#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkTimesync {
public:
    /** Transmit callback: finalise + send the prepared mavlink_message_t. */
    using SendFn = void (*)(void *ctx, mavlink_message_t &msg);

    MavlinkTimesync(SendFn send, void *send_ctx) noexcept;

    /**
     * Handle an incoming TIMESYNC message (PX4 semantics).
     */
    void handle_message(const mavlink_message_t *msg) noexcept;

    /**
     * Convert remote timestamp to local hrt time (usec).
     * Uses synchronised time if available, monotonic boot time otherwise.
     */
    std::uint64_t sync_stamp(std::uint64_t usec) noexcept;

    bool converged() const noexcept;

    void reset() noexcept;

private:
    dima::lib::timesync::Timesync timesync_{};
    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
};

}  // namespace dima::modules::mavlink
