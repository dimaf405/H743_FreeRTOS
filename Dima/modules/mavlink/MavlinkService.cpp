#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"

#include "logging/logging.hpp"
#include "platform/api/Time.hpp"

#include <cstring>

namespace dima::modules::mavlink {

MavlinkService::MavlinkService(
    dima::platform::Console &console,
    dima::platform::BootControl &boot_control) noexcept
    : px4::ScheduledWorkItem("mavlink", px4::wq_configurations::lp_default),
      console_(console), boot_control_(boot_control)
{
}

bool MavlinkService::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    /* Firmware version 0.1.0; UID and board version from platform API. */
    identity_.configure(
        MavlinkIdentity::encode_version(0, 1, 0, 0),
        dima::platform::board_version(),
        dima::platform::board_hardware_uid());
    identity_.set_state(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MAV_STATE_BOOT);
    std::memset(&parse_status_, 0, sizeof(parse_status_));

    /* Metadata file table is attached once the XZ arrays are linked. */
    ftp_.init(nullptr, 0U);
    statustext_id_ = 0U;
    reboot_mode_pending_ = 0;

    if (!ScheduleOnInterval(kRunIntervalUs, kRunIntervalUs)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    PX4_INFO("MAVLink service owns USB transport");
    return true;
}

void MavlinkService::stop() noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    reboot_mode_pending_ = 0;
}

dima::middleware::lifecycle::ModuleState MavlinkService::state()
    const noexcept
{
    return state_;
}

void MavlinkService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    console_.service();

    /* RX dispatch (responses are sent inline during handling). */
    drain_rx();

    /* TX priority: ACK -> Heartbeat -> Parameter stream / FTP -> STATUSTEXT. */
    process_command_acks();

    mavlink_message_t message{};
    if (heartbeat_pacer_.tick(hrt_absolute_time(), message)) {
        (void)send_message(message);
    }

    parameters_.send();
    stream_statustext();
}

/* ── RX path ─────────────────────────────────────────────────────── */

void MavlinkService::drain_rx() noexcept
{
    const std::size_t count = console_.read(rx_buffer_, sizeof(rx_buffer_));
    for (std::size_t i = 0U; i < count; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer_[i],
                               &parse_message_, &parse_status_)) {
            dispatch(parse_message_);
        }
    }
}

void MavlinkService::dispatch(const mavlink_message_t &msg) noexcept
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    case MAVLINK_MSG_ID_PARAM_SET:
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
        parameters_.handle_message(&msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        record_reboot_mode(msg);
        commands_.handle_message(&msg);
        break;

    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        ftp_.handle_message(&msg);
        break;

    case MAVLINK_MSG_ID_TIMESYNC:
        timesync_.handle_message(&msg);
        break;

    case MAVLINK_MSG_ID_PING:
        handle_ping(msg);
        break;

    default:
        /* GCS heartbeat and everything outside the allowlist: ignore. */
        break;
    }
}

void MavlinkService::handle_ping(const mavlink_message_t &msg) noexcept
{
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(&msg, &ping);

    /* Respond to pings addressed to us or broadcast. */
    if (ping.target_system != 0 &&
        ping.target_system != MAVLINK_SYSTEM_ID) {
        return;
    }

    mavlink_ping_t reply{};
    reply.time_usec = ping.time_usec;
    reply.seq = ping.seq;
    reply.target_system = msg.sysid;
    reply.target_component = msg.compid;

    mavlink_message_t frame{};
    mavlink_msg_ping_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                            &frame, &reply);
    (void)send_message(frame);
}

void MavlinkService::record_reboot_mode(const mavlink_message_t &msg) noexcept
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    if (cmd.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) {
        const int mode = static_cast<int>(cmd.param1);
        /* Remember the mode so the deferred reset uses the right
         * BootControl entry point once the ACK has been delivered. */
        reboot_mode_pending_ = (mode == 1 || mode == 3) ? mode : 0;
    }
}

/* ── TX path ─────────────────────────────────────────────────────── */

bool MavlinkService::send_message(mavlink_message_t &msg,
                                  std::uint32_t timeout_ms) noexcept
{
    const std::uint16_t length =
        mavlink_msg_to_send_buffer(tx_buffer_, &msg);
    const int written = console_.write(tx_buffer_, length, timeout_ms);
    return written == static_cast<int>(length);
}

bool MavlinkService::send_frame(void *ctx, mavlink_message_t &msg) noexcept
{
    if (ctx == nullptr) {
        return false;
    }
    return static_cast<MavlinkService *>(ctx)->send_message(msg);
}

void MavlinkService::send_frame_void(void *ctx,
                                     mavlink_message_t &msg) noexcept
{
    if (ctx != nullptr) {
        (void)static_cast<MavlinkService *>(ctx)->send_message(msg);
    }
}

std::uint8_t MavlinkService::request_message(void *ctx,
                                             std::uint16_t message_id) noexcept
{
    if (ctx == nullptr) {
        return vehicle_command_ack_s::RESULT_UNSUPPORTED;
    }
    auto &self = *static_cast<MavlinkService *>(ctx);

    switch (message_id) {
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        self.send_autopilot_version();
        return vehicle_command_ack_s::RESULT_ACCEPTED;

    case MAVLINK_MSG_ID_PROTOCOL_VERSION:
        self.send_protocol_version();
        return vehicle_command_ack_s::RESULT_ACCEPTED;

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_message_t heartbeat{};
        if (self.heartbeat_pacer_.tick(hrt_absolute_time(), heartbeat)) {
            (void)self.send_message(heartbeat);
        }
        return vehicle_command_ack_s::RESULT_ACCEPTED;
    }

    default:
        return vehicle_command_ack_s::RESULT_UNSUPPORTED;
    }
}

void MavlinkService::send_autopilot_version() noexcept
{
    mavlink_message_t message{};
    heartbeat_pacer_.pack_autopilot_version(message);
    (void)send_message(message);
}

void MavlinkService::send_protocol_version() noexcept
{
    mavlink_protocol_version_t version{};
    version.version = 200;      /* MAVLink v2.0 */
    version.min_version = 100;
    version.max_version = 200;
    std::memset(version.spec_version_hash, 0, sizeof(version.spec_version_hash));
    std::memset(version.library_version_hash, 0,
                sizeof(version.library_version_hash));

    mavlink_message_t message{};
    mavlink_msg_protocol_version_encode(MAVLINK_SYSTEM_ID,
                                        MAVLINK_COMPONENT_ID,
                                        &message, &version);
    (void)send_message(message);
}

void MavlinkService::process_command_acks() noexcept
{
    while (command_ack_subscription_.update()) {
        const vehicle_command_ack_s &ack = command_ack_subscription_.get();

        const bool is_reboot =
            ack.command ==
                vehicle_command_s::NAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN &&
            ack.result == vehicle_command_ack_s::RESULT_ACCEPTED &&
            reboot_mode_pending_ != 0;

        mavlink_command_ack_t command_ack{};
        command_ack.command = ack.command;
        command_ack.result = ack.result;
        command_ack.progress = 0;
        command_ack.target_system = ack.target_system;
        command_ack.target_component = ack.target_component;

        mavlink_message_t message{};
        mavlink_msg_command_ack_encode(MAVLINK_SYSTEM_ID,
                                       MAVLINK_COMPONENT_ID,
                                       &message, &command_ack);

        /* Console::write waits for completion, so the reboot ACK reaches
         * the host before USB disappears during the reset. */
        (void)send_message(message, is_reboot ? kRebootAckTimeoutMs
                                              : kTxTimeoutMs);

        if (is_reboot) {
            perform_reboot();
        }
    }
}

/* ── STATUSTEXT stream (ported from streams/STATUSTEXT.hpp) ──────── */

void MavlinkService::stream_statustext() noexcept
{
    /* Keep records queued while USB is disconnected. */
    if (!console_.ready()) {
        return;
    }

    std::size_t sent_records = 0U;

    while (mavlink_log_subscription_.update() &&
           sent_records < kMaxStatusTextPerRun) {
        const mavlink_log_s &mavlink_log = mavlink_log_subscription_.get();

        /* don't send stale messages */
        if (hrt_elapsed_time(&mavlink_log.timestamp) >= 5000000ULL) {
            continue;
        }

        mavlink_statustext_t msg{};
        const char *text = mavlink_log.text;
        constexpr unsigned max_chunk_size = sizeof(msg.text);
        msg.severity = mavlink_log.severity;
        msg.chunk_seq = 0;
        msg.id = statustext_id_++;
        unsigned text_size;
        bool send_ok = true;

        while ((text_size = std::strlen(text)) > 0) {
            unsigned chunk_size = text_size < max_chunk_size
                                      ? text_size : max_chunk_size;

            if (chunk_size < max_chunk_size) {
                std::memcpy(&msg.text[0], &text[0], chunk_size);
                /* pad with zeros */
                std::memset(&msg.text[0] + chunk_size, 0,
                            max_chunk_size - chunk_size);

            } else {
                std::memcpy(&msg.text[0], &text[0], chunk_size);
            }

            mavlink_message_t frame{};
            mavlink_msg_statustext_encode(MAVLINK_SYSTEM_ID,
                                          MAVLINK_COMPONENT_ID,
                                          &frame, &msg);
            if (!send_message(frame)) {
                send_ok = false;
                break;
            }

            if (text_size <= max_chunk_size) {
                break;

            } else {
                text += max_chunk_size;
            }

            msg.chunk_seq += 1;
        }

        if (!send_ok) {
            break;
        }
        ++sent_records;
    }
}

/* ── Deferred reboot ─────────────────────────────────────────────── */

void MavlinkService::perform_reboot() noexcept
{
    PX4_INFO("Executing deferred reboot (mode %d)", reboot_mode_pending_);
    if (reboot_mode_pending_ == 3) {
        boot_control_.reboot_to_recovery();
    } else {
        boot_control_.reboot();
    }
    for (;;) {
        /* Unreachable — reboot() is [[noreturn]]. */
    }
}

}  // namespace dima::modules::mavlink
