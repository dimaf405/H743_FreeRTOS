#pragma once
/*
 * MavlinkService — the single RX/TX owner of the USB CDC transport.
 *
 * Architecture follows PX4 src/modules/mavlink/: protocol framing from
 * the official generated C library (c_library_v2, trimmed dialect),
 * RX dispatch through receiver-style handlers (MavlinkCommands,
 * MavlinkParameters, MavlinkMetadataFtp, MavlinkTimesync), and a
 * fixed-priority TX path.
 *
 * Responsibilities:
 *   - RX: mavlink_parse_char() on the Console byte stream, dispatch.
 *   - TX priority: COMMAND_ACK / HEARTBEAT -> Parameter stream / FTP
 *     -> STATUSTEXT log conversion.
 *   - Deferred reboot: after the reboot COMMAND_ACK has been written
 *     to USB, perform normal reset or MCUboot Recovery entry.
 *
 * Runs on the low-priority WorkQueue with fixed buffers; no dynamic
 * allocation, no blocking USB writes in the hot path.
 */

#include "mavlink_log.hpp"
#include "vehicle_command.hpp"
#include "vehicle_command_ack.hpp"
#include "lifecycle/module_base.hpp"
#include "lib/mavlink/mavlink_bridge.h"
#include "platform/api/Platform.hpp"
#include "uorb/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include "HeartbeatPacer.hpp"
#include "MavlinkCommands.hpp"
#include "MavlinkIdentity.hpp"
#include "MavlinkMetadataFtp.hpp"
#include "MavlinkParameters.hpp"
#include "MavlinkTimesync.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkService final : public dima::middleware::lifecycle::ModuleBase,
                             public px4::ScheduledWorkItem {
public:
    MavlinkService(dima::platform::Console &console,
                   dima::platform::BootControl &boot_control) noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

private:
    static constexpr std::uint32_t kRunIntervalUs = 10000U;   /* 100 Hz */
    static constexpr std::size_t kRxBatchBytes = 256U;
    static constexpr std::size_t kMaxStatusTextPerRun = 2U;
    static constexpr std::uint32_t kTxTimeoutMs = 5U;
    static constexpr std::uint32_t kRebootAckTimeoutMs = 250U;

    /* Callback trampolines for the protocol handlers. */
    static bool send_frame(void *ctx, mavlink_message_t &msg) noexcept;
    static void send_frame_void(void *ctx, mavlink_message_t &msg) noexcept;
    static std::uint8_t request_message(void *ctx,
                                        std::uint16_t message_id) noexcept;

    void Run() override;
    void drain_rx() noexcept;
    void dispatch(const mavlink_message_t &msg) noexcept;
    void handle_ping(const mavlink_message_t &msg) noexcept;
    void record_reboot_mode(const mavlink_message_t &msg) noexcept;
    void process_command_acks() noexcept;
    void stream_statustext() noexcept;
    bool send_message(mavlink_message_t &msg,
                      std::uint32_t timeout_ms = kTxTimeoutMs) noexcept;
    void send_autopilot_version() noexcept;
    void send_protocol_version() noexcept;
    [[noreturn]] void perform_reboot() noexcept;

    dima::platform::Console &console_;
    dima::platform::BootControl &boot_control_;

    mavlink_message_t parse_message_{};
    mavlink_status_t parse_status_{};

    MavlinkIdentity identity_{};
    HeartbeatPacer heartbeat_pacer_{identity_};
    MavlinkParameters parameters_{&MavlinkService::send_frame, this};
    MavlinkTimesync timesync_{&MavlinkService::send_frame_void, this};
    MavlinkCommands commands_{&MavlinkService::request_message, this};
    MavlinkMetadataFtp ftp_{&MavlinkService::send_frame_void, this};

    uORB::SubscriptionData<vehicle_command_ack_s>
        command_ack_subscription_{ORB_ID(vehicle_command_ack)};
    uORB::SubscriptionData<mavlink_log_s>
        mavlink_log_subscription_{ORB_ID(mavlink_log)};

    std::uint8_t rx_buffer_[kRxBatchBytes]{};
    std::uint8_t tx_buffer_[MAVLINK_MAX_PACKET_LEN]{};
    std::uint16_t statustext_id_{0U};
    /* 0 = none, 1 = normal reset, 3 = MCUboot Recovery. */
    int reboot_mode_pending_{0};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

}  // namespace dima::modules::mavlink
