#include "MavlinkTimesync.hpp"

#include "api/Time.hpp"

namespace dima::modules::mavlink {

MavlinkTimesync::MavlinkTimesync(SendFn send, void *send_ctx) noexcept
    : send_(send), send_ctx_(send_ctx)
{
}

void MavlinkTimesync::handle_message(const mavlink_message_t *msg) noexcept
{
    mavlink_timesync_t tsync{};
    mavlink_msg_timesync_decode(msg, &tsync);

    const std::uint64_t now = hrt_absolute_time();

    if (tsync.tc1 == 0) {
        // 远端发起：MAVLink TIMESYNC 字段单位为纳秒，本机 HRT 为微秒，故乘 1000。
        mavlink_timesync_t rsync{};
        rsync.tc1 = static_cast<std::int64_t>(now * 1000ULL);
        rsync.ts1 = tsync.ts1;

        mavlink_message_t reply{};
        mavlink_msg_timesync_encode(1, 1, &reply, &rsync);
        if (send_ != nullptr) {
            send_(send_ctx_, reply);
        }

    } else if (tsync.tc1 > 0) {
        // 本机发起的返回包：利用发送时间、远端接收时间和当前接收时间估计偏移及 RTT。
        timesync_.update(now, tsync.tc1, tsync.ts1);
    }
}

std::uint64_t MavlinkTimesync::sync_stamp(std::uint64_t usec) noexcept
{
    return timesync_.sync_stamp(usec);
}

bool MavlinkTimesync::converged() const noexcept
{
    return timesync_.sync_converged();
}

void MavlinkTimesync::reset() noexcept
{
    timesync_.reset_filter();
}

} // namespace dima::modules::mavlink
