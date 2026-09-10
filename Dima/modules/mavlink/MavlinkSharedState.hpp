#pragma once

#include "MavlinkLogHandler.hpp"
#include "vehicle_command.hpp"
#include "vehicle_command_ack.hpp"
#include "uORB/Publication.hpp"
#include "uORB/uORB.hpp"

namespace dima::modules::mavlink {

// Commander 与文件 reader 是车辆级资源。链路只隔离协议状态，不复制业务执行者。
class MavlinkSharedState {
public:
    using AckSink = bool (*)(void *, std::uint8_t, std::uint32_t,
                             const vehicle_command_ack_s &) noexcept;
    std::uint8_t submit(std::uint8_t channel, std::uint32_t epoch,
                        const vehicle_command_s &command) noexcept;
    void service_acks(std::uint64_t now, AckSink sink, void *context) noexcept;
    void reset_link(std::uint8_t channel, std::uint32_t epoch) noexcept;
    void reset() noexcept;
    bool reboot_pending() const noexcept;
    void mark_reboot_pending() noexcept;
    void cancel_reboot() noexcept;

    MavlinkLogLease log_lease{};
    std::uint8_t usb_batch[dima::platform::Console::kWriteCapacity]{};
private:
    uORB::Publication<vehicle_command_s> command_publication_{ORB_ID(vehicle_command)};
    uORB::Subscription ack_subscription_{ORB_ID(vehicle_command_ack)};
    vehicle_command_ack_s pending_ack_{};
    std::uint64_t command_started_us_{0U};
    std::uint32_t command_{0U};
    std::uint32_t epoch_{0U};
    std::uint8_t channel_{0U};
    std::uint8_t source_system_{0U};
    std::uint8_t source_component_{0U};
    bool command_pending_{false};
    bool ack_pending_{false};
    bool owner_valid_{false};
    bool reboot_pending_{false};
};

} // namespace dima::modules::mavlink
