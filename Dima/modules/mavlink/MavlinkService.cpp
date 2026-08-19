#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"
#include "platform/api/BoardIdentity.hpp"

#include "logging/logging.hpp"
#include "parameter_metadata_files.hpp"
#include "platform/api/Time.hpp"

#include <cerrno>
#include <cstring>

namespace dima::modules::mavlink {
namespace {

namespace metadata = dima::generated::parameter_metadata;

static_assert(metadata::kGeneralFileSize <= UINT32_MAX);
static_assert(metadata::kParameterFileSize <= UINT32_MAX);

constexpr MavlinkMetadataFtp::VirtualFile kMetadataFiles[]{
    {metadata::kGeneralPath, metadata::kGeneralFile,
     static_cast<std::uint32_t>(metadata::kGeneralFileSize)},
    {metadata::kParameterPath, metadata::kParameterFile,
     static_cast<std::uint32_t>(metadata::kParameterFileSize)},
};

} // namespace

MavlinkService::MavlinkService(
    dima::platform::Console &console,
    dima::platform::BootControl &boot_control) noexcept
    : px4::ScheduledWorkItem("mavlink", px4::wq_configurations::lp_default),
      console_(console), boot_control_(boot_control)
{
    metadata_ftp_.init(
        kMetadataFiles,
        static_cast<std::uint8_t>(sizeof(kMetadataFiles) /
                                  sizeof(kMetadataFiles[0])));
}

bool MavlinkService::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    reset_runtime_state();
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
    rc_loss_timeout_handle_ = param_find("COM_RC_LOSS_T");
    mav_system_id_handle_ = param_find("MAV_SYS_ID");
    if (rc_loss_timeout_handle_ == PARAM_INVALID ||
        mav_system_id_handle_ == PARAM_INVALID ||
        !refresh_protocol_parameters()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        reset_runtime_state();
        PX4_ERR("MAVLink protocol parameters unavailable");
        return false;
    }

    if (!ScheduleOnInterval(kRunIntervalUs, kRunIntervalUs)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        reset_runtime_state();
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
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState MavlinkService::state()
    const noexcept
{
    return state_;
}

void MavlinkService::reset_runtime_state() noexcept
{
    reset_parser_state();
    identity_.set_state(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MAV_STATE_BOOT);
    heartbeat_pacer_.reset();
    parameters_.reset();
    timesync_.reset();
    metadata_ftp_.reset();
    pending_ack_ = mavlink_command_ack_t{};
    pending_ack_valid_ = false;
    pending_ack_is_reboot_ = false;
    ack_retry_ = 0U;
    statustext_id_ = 0U;
    was_link_ready_ = false;
    reboot_mode_pending_ = 0;
    reboot_deadline_us_ = 0U;
    latest_input_rc_ = input_rc_s{};
    rc_loss_timeout_handle_ = PARAM_INVALID;
    mav_system_id_handle_ = PARAM_INVALID;
    rc_loss_timeout_s_ = 0.0F;
    last_rc_channels_tx_us_ = 0U;
    have_input_rc_ = false;
    rc_stream_active_ = false;
    rc_loss_timeout_valid_ = false;
    transport_was_ready_ = false;
}

void MavlinkService::reset_parser_state() noexcept
{
    *mavlink_get_channel_status(MAVLINK_COMM_0) = mavlink_status_t{};
    *mavlink_get_channel_buffer(MAVLINK_COMM_0) = mavlink_message_t{};
    parse_message_ = mavlink_message_t{};
    parse_status_ = mavlink_status_t{};
    std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
    std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
}

void MavlinkService::discard_rx() noexcept
{
    while (console_.available() != 0U) {
        if (console_.read(rx_buffer_, sizeof(rx_buffer_)) == 0U) {
            break;
        }
    }
    std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

void MavlinkService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    console_.service();
    const bool transport_ready = console_.ready();
    if (!transport_ready && transport_was_ready_) {
        /* Track the physical USB edge independently from first-frame TX.
         * Metadata requests can arrive while HEARTBEAT/AUTOPILOT_VERSION are
         * still retrying, so was_link_ready_ is not a sufficient session
         * lifetime boundary. */
        discard_rx();
        reset_parser_state();
        metadata_ftp_.reset();
    }
    transport_was_ready_ = transport_ready;
    if (!transport_ready) {
        was_link_ready_ = false;
    }
    update_rc_input();
    if (parameter_update_subscription_.update() &&
        !refresh_protocol_parameters()) {
        PX4_ERR("MAVLink protocol parameters invalid");
    }
    flush_pending_ack();
    std::uint64_t now = hrt_absolute_time();
    maybe_perform_reboot(now);
    if (pending_ack_valid_ || reboot_mode_pending_ != 0) {
        return;
    }

    /* RX handlers only freeze protocol replies; the fixed-priority TX path
     * below owns their transmission. Never consume stale USB bytes while the
     * physical link is down. */
    if (transport_ready) {
        drain_rx();
    }

    /* TX priority: ACK -> Heartbeat -> RC -> Metadata FTP -> Params -> Log. */
    process_command_acks();
    now = hrt_absolute_time();
    maybe_perform_reboot(now);
    if (pending_ack_valid_ || reboot_mode_pending_ != 0) {
        return;
    }

    const bool link_ready = transport_ready;
    if (link_ready && !was_link_ready_) {
        mavlink_message_t heartbeat{};
        heartbeat_pacer_.pack_now(now, heartbeat);
        const bool heartbeat_sent = send_message(heartbeat);
        if (!heartbeat_sent) {
            heartbeat_pacer_.reset();
        }
        const bool version_sent = send_autopilot_version();
        if (heartbeat_sent && version_sent) {
            was_link_ready_ = true;
            last_rc_channels_tx_us_ = 0U;
            rc_stream_active_ = false;
            PX4_INFO("MAVLink USB link ready");
        }
    }

    if (!link_ready || !was_link_ready_) {
        return;
    }

    mavlink_message_t message{};
    if (heartbeat_pacer_.tick(now, message)) {
        if (!send_message(message)) {
            heartbeat_pacer_.reset();
        }
    }
    stream_rc_channels(now);
    if (!metadata_ftp_.service(now)) {
        return;
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
        PX4_INFO("PARAM_REQUEST_LIST from sys=%u comp=%u", msg.sysid, msg.compid);
        parameters_.handle_message(&msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    case MAVLINK_MSG_ID_PARAM_SET:
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
        parameters_.handle_message(&msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
    case MAVLINK_MSG_ID_COMMAND_INT:
        commands_.handle_message(&msg);
        break;

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        mission_.handle_message(&msg);
        break;

    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        metadata_ftp_.handle_message(&msg, hrt_absolute_time());
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
        return self.send_autopilot_version()
            ? vehicle_command_ack_s::RESULT_ACCEPTED
            : vehicle_command_ack_s::RESULT_TEMPORARILY_REJECTED;

    case MAVLINK_MSG_ID_PROTOCOL_VERSION:
        return self.send_protocol_version()
            ? vehicle_command_ack_s::RESULT_ACCEPTED
            : vehicle_command_ack_s::RESULT_TEMPORARILY_REJECTED;

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_message_t heartbeat{};
        self.heartbeat_pacer_.pack_now(hrt_absolute_time(), heartbeat);
        if (self.send_message(heartbeat)) {
            return vehicle_command_ack_s::RESULT_ACCEPTED;
        }
        self.heartbeat_pacer_.reset();
        return vehicle_command_ack_s::RESULT_TEMPORARILY_REJECTED;
    }

    case MAVLINK_MSG_ID_COMPONENT_METADATA:
        return self.send_component_metadata()
            ? vehicle_command_ack_s::RESULT_ACCEPTED
            : vehicle_command_ack_s::RESULT_TEMPORARILY_REJECTED;

    case MAVLINK_MSG_ID_COMPONENT_INFORMATION:
        return self.send_component_information()
            ? vehicle_command_ack_s::RESULT_ACCEPTED
            : vehicle_command_ack_s::RESULT_TEMPORARILY_REJECTED;

    default:
        return vehicle_command_ack_s::RESULT_UNSUPPORTED;
    }
}

void MavlinkService::process_command_acks() noexcept
{
    if (pending_ack_valid_ || reboot_mode_pending_ != 0) {
        return;
    }

    while (command_ack_subscription_.update()) {
        const vehicle_command_ack_s ack = command_ack_subscription_.get();
        if (!ack.from_external) {
            continue;
        }

        mavlink_command_ack_t command_ack{};
        command_ack.command = ack.command;
        command_ack.result = ack.result;
        command_ack.progress = 0;
        command_ack.result_param2 = static_cast<std::int32_t>(ack.result_param2);
        command_ack.target_system = ack.target_system;
        command_ack.target_component = ack.target_component;

        const bool is_reboot =
            ack.command ==
                vehicle_command_s::NAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN &&
            ack.result == vehicle_command_ack_s::RESULT_ACCEPTED &&
            (ack.result_param2 == 1U || ack.result_param2 == 3U);
        if (is_reboot) {
            reboot_mode_pending_ = static_cast<int>(ack.result_param2);
            reboot_deadline_us_ = hrt_absolute_time() + kRebootDeadlineUs;
        }
        send_command_ack(command_ack, is_reboot);
        break;
    }
}

void MavlinkService::send_command_ack(
    const mavlink_command_ack_t &ack, bool reboot_ack) noexcept
{
    mavlink_message_t message{};
    mavlink_msg_command_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                   &message, &ack);

    if (send_message(message)) {
        if (reboot_ack) {
            perform_reboot();
        }
        return;
    }

    const int error = errno;
    if (error == EAGAIN || error == ETIMEDOUT) {
        pending_ack_ = ack;
        pending_ack_valid_ = true;
        pending_ack_is_reboot_ = reboot_ack;
        ack_retry_ = 0U;
    } else {
        PX4_ERR("COMMAND_ACK tx errno %d, dropped", error);
    }
}

void MavlinkService::flush_pending_ack() noexcept
{
    if (!pending_ack_valid_) {
        return;
    }

    mavlink_message_t message{};
    mavlink_msg_command_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                   &message, &pending_ack_);

    if (send_message(message)) {
        const bool reboot_ack = pending_ack_is_reboot_;
        pending_ack_valid_ = false;
        pending_ack_is_reboot_ = false;
        ack_retry_ = 0U;
        if (reboot_ack) {
            perform_reboot();
        }
        return;
    }

    const int error = errno;
    if (error == EAGAIN || error == ETIMEDOUT) {
        ++ack_retry_;
        if (ack_retry_ >= kMaxAckRetries) {
            PX4_ERR("COMMAND_ACK retries exhausted, dropped");
            pending_ack_valid_ = false;
            pending_ack_is_reboot_ = false;
            ack_retry_ = 0U;
        }
    } else {
        PX4_ERR("COMMAND_ACK tx errno %d, dropped", error);
        pending_ack_valid_ = false;
        pending_ack_is_reboot_ = false;
        ack_retry_ = 0U;
    }
}

void MavlinkService::maybe_perform_reboot(std::uint64_t now) noexcept
{
    if (reboot_mode_pending_ != 0 && reboot_deadline_us_ != 0U &&
        now >= reboot_deadline_us_) {
        pending_ack_valid_ = false;
        pending_ack_is_reboot_ = false;
        ack_retry_ = 0U;
        PX4_ERR("Reboot ACK deadline expired; executing approved reboot");
        perform_reboot();
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

    while (sent_records < kMaxStatusTextPerRun &&
           mavlink_log_subscription_.update()) {
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
