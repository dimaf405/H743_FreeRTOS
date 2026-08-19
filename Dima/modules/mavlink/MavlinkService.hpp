#pragma once
/*
 * MavlinkService — the single RX/TX owner of the USB CDC transport.
 *
 * Architecture follows PX4 src/modules/mavlink/: protocol framing from
 * the official generated C library (c_library_v2, trimmed dialect),
 * RX dispatch through receiver-style handlers (MavlinkCommands,
 * MavlinkParameters, MavlinkMission, MavlinkTimesync), and a
 * fixed-priority TX path.
 *
 * Responsibilities:
 *   - RX: mavlink_parse_char() on the Console byte stream, dispatch.
 *   - TX priority: COMMAND_ACK / HEARTBEAT -> raw RC stream -> one pending
 *     read-only Metadata FTP response -> Parameter stream -> STATUSTEXT.
 *   - Deferred reboot: prefer delivering COMMAND_ACK, then perform normal
 *     reset or MCUboot Recovery no later than the fixed deadline.
 *
 * Runs on the low-priority WorkQueue with fixed buffers; no dynamic
 * allocation and only bounded USB writes in the hot path.
 */

#include "input_rc.hpp"
#include "mavlink_log.hpp"
#include "parameter_update.hpp"
#include "vehicle_command.hpp"
#include "vehicle_command_ack.hpp"
#include "lifecycle/module_base.hpp"
#include "lib/mavlink/mavlink_bridge.h"
#include "platform/api/Platform.hpp"
#include "parameters/param.h"
#include "uorb/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include "HeartbeatPacer.hpp"
#include "MavlinkCommands.hpp"
#include "MavlinkIdentity.hpp"
#include "MavlinkMission.hpp"
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
    static constexpr std::uint64_t kRebootDeadlineUs = 400000ULL;
    static constexpr std::uint64_t kRcChannelsIntervalUs = 200000ULL; /* 5 Hz */
    /* COMMAND_ACK 重试槽上限（首次发送失败后最多再试 4 次）。 */
    static constexpr std::uint8_t kMaxAckRetries = 4U;

    /* Callback trampolines for the protocol handlers. */
    static bool send_frame(void *ctx, mavlink_message_t &msg) noexcept;
    static void send_frame_void(void *ctx, mavlink_message_t &msg) noexcept;
    static std::uint8_t request_message(void *ctx,
                                        std::uint16_t message_id) noexcept;

    void Run() override;
    void reset_runtime_state() noexcept;
    void reset_parser_state() noexcept;
    void discard_rx() noexcept;
    void drain_rx() noexcept;
    void dispatch(const mavlink_message_t &msg) noexcept;
    void handle_ping(const mavlink_message_t &msg) noexcept;
    void process_command_acks() noexcept;
    void send_command_ack(const mavlink_command_ack_t &ack,
                          bool reboot_ack) noexcept;
    void flush_pending_ack() noexcept;
    void maybe_perform_reboot(std::uint64_t now) noexcept;
    bool refresh_protocol_parameters() noexcept;
    void update_rc_input() noexcept;
    void stream_rc_channels(std::uint64_t now) noexcept;
    bool rc_sample_streamable(std::uint64_t now) const noexcept;
    bool send_rc_channels(std::uint64_t now) noexcept;
    void stream_statustext() noexcept;
    bool send_message(mavlink_message_t &msg,
                      std::uint32_t timeout_ms = kTxTimeoutMs) noexcept;
    bool send_autopilot_version() noexcept;
    bool send_protocol_version() noexcept;
    bool send_component_metadata() noexcept;
    bool send_component_information() noexcept;
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
    MavlinkMission mission_{&MavlinkService::send_frame_void, this};
    MavlinkMetadataFtp metadata_ftp_{&MavlinkService::send_frame, this};

    uORB::SubscriptionData<vehicle_command_ack_s>
        command_ack_subscription_{ORB_ID(vehicle_command_ack)};
    uORB::SubscriptionData<mavlink_log_s>
        mavlink_log_subscription_{ORB_ID(mavlink_log)};
    uORB::SubscriptionData<input_rc_s>
        input_rc_subscription_{ORB_ID(input_rc)};
    uORB::SubscriptionData<parameter_update_s>
        parameter_update_subscription_{ORB_ID(parameter_update)};

    std::uint8_t rx_buffer_[kRxBatchBytes]{};
    std::uint8_t tx_buffer_[MAVLINK_MAX_PACKET_LEN]{};
    /* COMMAND_ACK 发送失败单重试槽；pending 时不继续消费 uORB ACK，
     * 因此其深度 4 队列继续保持 FIFO。 */
    mavlink_command_ack_t pending_ack_{};
    bool pending_ack_valid_{false};
    bool pending_ack_is_reboot_{false};
    std::uint8_t ack_retry_{0U};

    std::uint16_t statustext_id_{0U};
    input_rc_s latest_input_rc_{};
    param_t rc_loss_timeout_handle_{PARAM_INVALID};
    param_t mav_system_id_handle_{PARAM_INVALID};
    float rc_loss_timeout_s_{0.0F};
    std::uint64_t last_rc_channels_tx_us_{0U};
    bool was_link_ready_{false};
    bool transport_was_ready_{false};
    bool have_input_rc_{false};
    bool rc_stream_active_{false};
    bool rc_loss_timeout_valid_{false};
    /* 0 = none, 1 = normal reset, 3 = MCUboot Recovery. */
    int reboot_mode_pending_{0};
    std::uint64_t reboot_deadline_us_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

}  // namespace dima::modules::mavlink
