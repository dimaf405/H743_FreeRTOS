#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"
#include "FirmwareIdentityContract.hpp"
#include "api/BoardIdentity.hpp"

#include "logging/logging.hpp"
#include "parameter_metadata_files.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>

namespace dima::modules::mavlink {
namespace stream_contract = dima::generated::mavlink_streams;
namespace {

namespace metadata = dima::generated::parameter_metadata;

static_assert(metadata::kGeneralFileSize <= UINT32_MAX);
static_assert(metadata::kParameterFileSize <= UINT32_MAX);
static_assert(metadata::kActuatorFileSize <= UINT32_MAX);
static_assert(
    HeartbeatPacer::kIntervalUs == static_cast<std::uint64_t>(
        stream_contract::default_interval_us(
            stream_contract::MessageHandler::Heartbeat)),
    "Heartbeat pacer must match the generated MAVLink stream contract");

constexpr MavlinkMetadataFtp::VirtualFile kMetadataFiles[]{
    {metadata::kGeneralPath, metadata::kGeneralFile,
     static_cast<std::uint32_t>(metadata::kGeneralFileSize)},
    {metadata::kParameterPath, metadata::kParameterFile,
     static_cast<std::uint32_t>(metadata::kParameterFileSize)},
    {metadata::kActuatorPath, metadata::kActuatorFile,
     static_cast<std::uint32_t>(metadata::kActuatorFileSize)},
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

    // PX4/QGC compatibility identity is generated independently of MCUboot's
    // product image version; board version and UID remain hardware identity.
    identity_.configure(
        dima::generated::firmware_identity::kFlightSoftwareVersion,
        dima::platform::board_version(),
        dima::platform::board_hardware_uid());
    identity_.set_state(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MAV_STATE_BOOT);
    // 协议参数使用生成枚举句柄；MAVLink 层不维护参数名或目录副本。
    rc_loss_timeout_handle_ = param_handle(px4::params::COM_RC_LOSS_T);
    mav_system_id_handle_ = param_handle(px4::params::MAV_SYS_ID);
    if (rc_loss_timeout_handle_ == PARAM_INVALID ||
        mav_system_id_handle_ == PARAM_INVALID ||
        !refresh_protocol_parameters() ||
        !parameters_.prepare_parameter_catalogue()) {
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
    reset_sensor_streams();
    rc_loss_timeout_handle_ = PARAM_INVALID;
    mav_system_id_handle_ = PARAM_INVALID;
    rc_loss_timeout_s_ = 0.0F;
    have_input_rc_ = false;
    rc_stream_active_ = false;
    rc_loss_timeout_valid_ = false;
    transport_was_ready_ = false;
}

void MavlinkService::reset_parser_state() noexcept
{
    // 官方 MAVLink C 库还维护 channel 全局状态；本地解析器和 channel 状态必须一起清零。
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
        // USB 物理断开边沿独立于“首帧发送成功”：握手重试期间也可能已收到 FTP 请求，
        // 因此断开时必须同时清解析器、参数快照、FTP 会话和各流节拍。
        discard_rx();
        reset_parser_state();
        parameters_.reset();
        metadata_ftp_.reset();
        reset_sensor_link_state();
    }
    transport_was_ready_ = transport_ready;
    if (!transport_ready) {
        was_link_ready_ = false;
    }
    update_rc_input();
    update_sensor_topics();
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

    // RX 处理器只冻结回复状态，真实写出仍由固定优先级 TX 路径统一拥有；物理链路
    // 未就绪时不消费残留字节，避免上一 USB 会话的数据进入新会话。
    if (transport_ready) {
        drain_rx();
    }

    /* TX priority: ACK -> Heartbeat -> RC -> Metadata FTP -> Sensors -> Params -> Log. */
    process_command_acks();
    now = hrt_absolute_time();
    maybe_perform_reboot(now);
    // ACK 重试和批准重启拥有最高优先级；存在时本轮不发送低优先级数据。
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
            last_highres_imu_timestamp_us_ = 0U;
            last_highres_mag_timestamp_us_ = 0U;
            last_scaled_imu_timestamp_us_ = 0U;
            last_scaled_mag_timestamp_us_ = 0U;
            rc_stream_active_ = false;
            PX4_INFO("MAVLink USB link ready");
            report_sensor_link_summary();
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
    stream_configured_messages(
        now, dima::generated::mavlink_streams::TxStage::PreMetadata);
    if (!metadata_ftp_.service(now)) {
        return;
    }
    stream_configured_messages(
        now, dima::generated::mavlink_streams::TxStage::PostMetadata);
    parameters_.send();
    stream_statustext();
}

    // 接收路径：官方解析器完成帧校验后，才按生成方言允许的消息分派。

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
    // 接收路由由 mavlink.lock.json 生成；源码只实现 handler 行为，不维护
    // msgid/consumer 的第二份 switch 清单。
    const stream_contract::InboundMessageContract *inbound =
        stream_contract::find_inbound_message(msg.msgid);
    if (inbound == nullptr) {
        // GCS HEARTBEAT 及当前消费集合之外的消息静默忽略。
        return;
    }

    switch (inbound->handler) {
    case stream_contract::InboundHandler::ParameterList:
        PX4_INFO("PARAM_REQUEST_LIST from sys=%u comp=%u", msg.sysid, msg.compid);
        [[fallthrough]];
    case stream_contract::InboundHandler::Parameters:
        parameters_.handle_message(&msg);
        break;

    case stream_contract::InboundHandler::Commands:
        commands_.handle_message(&msg);
        break;

    case stream_contract::InboundHandler::Mission:
        mission_.handle_message(&msg);
        break;

    case stream_contract::InboundHandler::MetadataFtp:
        metadata_ftp_.handle_message(&msg, hrt_absolute_time());
        break;

    case stream_contract::InboundHandler::Timesync:
        timesync_.handle_message(&msg);
        break;

    case stream_contract::InboundHandler::Ping:
        handle_ping(msg);
        break;
    }
}

void MavlinkService::handle_ping(const mavlink_message_t &msg) noexcept
{
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(&msg, &ping);

    // 只回应本系统或广播 PING，并把 target 定向回原发送者。
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
    // 只有整帧字节全部写入才算成功；短写保留给上层按各自策略重试。
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

void MavlinkService::reset_configured_streams() noexcept
{
    std::size_t index = 0U;
    for (const stream_contract::MessageContract &contract :
         stream_contract::kMessages) {
        if (contract.scheduler != stream_contract::Scheduler::Service) {
            continue;
        }
        if (index < configured_streams_.size()) {
            configured_streams_[index].interval_us =
                contract.default_interval_us;
            configured_streams_[index].last_tx_us = 0U;
        }
        ++index;
    }
    rc_stream_active_ = false;
}

bool MavlinkService::send_contract_message(
    stream_contract::MessageHandler handler, std::uint64_t now,
    bool refresh_topics) noexcept
{
    switch (handler) {
    case stream_contract::MessageHandler::Heartbeat: {
        mavlink_message_t heartbeat{};
        heartbeat_pacer_.pack_now(now, heartbeat);
        if (send_message(heartbeat)) return true;
        heartbeat_pacer_.reset();
        return false;
    }
    case stream_contract::MessageHandler::AutopilotVersion:
        return send_autopilot_version();
    case stream_contract::MessageHandler::ProtocolVersion:
        return send_protocol_version();
    case stream_contract::MessageHandler::ComponentMetadata:
        return send_component_metadata();
    case stream_contract::MessageHandler::ComponentInformation:
        return send_component_information();
    case stream_contract::MessageHandler::RcChannels:
        update_rc_input();
        rc_stream_active_ = rc_sample_streamable(now) &&
                            send_rc_channels(now);
        return rc_stream_active_;
    case stream_contract::MessageHandler::HighresImu:
        if (refresh_topics) update_sensor_topics();
        return send_highres_imu(now);
    case stream_contract::MessageHandler::ScaledImu:
        if (refresh_topics) update_sensor_topics();
        return send_scaled_imu(now);
    case stream_contract::MessageHandler::GpsRawInt:
        if (refresh_topics) update_sensor_topics();
        return send_gps_raw_int(now);
    case stream_contract::MessageHandler::SystemStatus:
        if (refresh_topics) update_sensor_topics();
        return send_system_status(now);
    }
    return false;
}

void MavlinkService::stream_configured_messages(
    std::uint64_t now, stream_contract::TxStage stage) noexcept
{
    std::size_t index = 0U;
    for (const stream_contract::MessageContract &contract :
         stream_contract::kMessages) {
        if (contract.scheduler != stream_contract::Scheduler::Service) {
            continue;
        }
        if (index >= configured_streams_.size()) return;
        ConfiguredStreamState &state = configured_streams_[index++];
        if (contract.tx_stage != stage) continue;
        if (state.interval_us < 0) {
            if (contract.handler ==
                stream_contract::MessageHandler::RcChannels) {
                rc_stream_active_ = false;
            }
            continue;
        }
        if (stream_due(now, state.last_tx_us, state.interval_us) &&
            send_contract_message(contract.handler, now, false)) {
            state.last_tx_us = now;
        }
    }
}

std::uint8_t MavlinkService::request_message(void *ctx,
                                             std::uint16_t message_id) noexcept
{
    if (ctx == nullptr) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
    }
    auto &self = *static_cast<MavlinkService *>(ctx);
    const stream_contract::MessageContract *contract =
        stream_contract::find_message(message_id);
    if (contract == nullptr || !contract->requestable) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
    }
    return self.send_contract_message(
               contract->handler, hrt_absolute_time(), true)
        ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
        : vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
}

std::uint8_t MavlinkService::set_message_interval(
    void *ctx, std::uint16_t message_id, float interval_us,
    float param3, float param4, float param7) noexcept
{
    if (ctx == nullptr || !std::isfinite(interval_us)) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
    }
    const auto unsupported_nonzero = [](float value) {
        return !std::isfinite(value) || std::lround(value) != 0L;
    };
    if (unsupported_nonzero(param3) || unsupported_nonzero(param4) ||
        unsupported_nonzero(param7)) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
    }

    auto &self = *static_cast<MavlinkService *>(ctx);
    const stream_contract::MessageContract *contract =
        stream_contract::find_message(message_id);
    if (contract == nullptr || !contract->interval_configurable ||
        contract->scheduler != stream_contract::Scheduler::Service) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
    }
    const std::size_t index = stream_contract::service_index(contract->handler);
    if (index >= self.configured_streams_.size()) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
    }

    // MAV_CMD_SET_MESSAGE_INTERVAL：负值禁用，0 恢复产品默认值，正值按微秒四舍五入，
    // HEARTBEAT 不允许关闭；param3/4/7 当前未实现，非零时明确拒绝。
    std::int32_t selected_interval = contract->default_interval_us;
    if (interval_us < -0.00001F) {
        selected_interval = -1;
    } else if (interval_us > 0.00001F) {
        const double rounded_interval = std::round(
            static_cast<double>(interval_us));
        if (rounded_interval > static_cast<double>(
                std::numeric_limits<std::int32_t>::max())) {
            return vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
        }
        selected_interval = static_cast<std::int32_t>(
            std::max(1.0, rounded_interval));
    }

    self.configured_streams_[index].interval_us = selected_interval;
    if (contract->handler == stream_contract::MessageHandler::RcChannels &&
        selected_interval < 0) {
        self.rc_stream_active_ = false;
    }
    return vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
}

std::uint8_t MavlinkService::get_message_interval(
    void *ctx, std::uint16_t message_id) noexcept
{
    if (ctx == nullptr) return vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
    auto &self = *static_cast<MavlinkService *>(ctx);
    std::int32_t interval_us = -1;
    const stream_contract::MessageContract *contract =
        stream_contract::find_message(message_id);
    if (contract != nullptr) {
        interval_us = contract->default_interval_us;
        if (contract->scheduler == stream_contract::Scheduler::Service) {
            const std::size_t index =
                stream_contract::service_index(contract->handler);
            if (index < self.configured_streams_.size()) {
                interval_us = self.configured_streams_[index].interval_us;
            }
        }
    }

    mavlink_message_interval_t report{};
    report.message_id = message_id;
    report.interval_us = interval_us;
    mavlink_message_t message{};
    mavlink_msg_message_interval_encode(MAVLINK_SYSTEM_ID,
                                        MAVLINK_COMPONENT_ID,
                                        &message, &report);
    return self.send_message(message)
        ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
        : vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
}

void MavlinkService::process_command_acks() noexcept
{
    // pending 单槽未清空时不继续消费深度 4 的 uORB FIFO，保持 Commander ACK 顺序。
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
        // 按 PX4 原生 ACK 投影 result_param1/progress 与 result_param2，
        // 不丢弃生成消息恢复的进度和附加结果字段。
        command_ack.progress = ack.result_param1;
        command_ack.result_param2 = ack.result_param2;
        command_ack.target_system = ack.target_system;
        command_ack.target_component = ack.target_component;

        const bool is_reboot =
            ack.command ==
                vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN &&
            ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED &&
            (ack.result_param2 == 1U || ack.result_param2 == 3U);
        if (is_reboot) {
            // 重启 mode 只接受 Commander 在 ACK.result_param2 中批准的 1/3。
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
        // 重启 ACK 整帧写入成功后立即复位；无需再等待低优先级流。
        if (reboot_ack) {
            perform_reboot();
        }
        return;
    }

    const int error = errno;
    if (error == EAGAIN || error == ETIMEDOUT) {
        // 仅暂态拥塞/超时进入有界重试槽，其他错误直接丢弃并记录。
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
    // ACK 无法在 400 ms 内送达时仍执行已批准重启，避免链路拥塞永久卡住 Recovery。
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
    // USB 断开时不消费日志 Topic，尽量保留记录给下一次连接。
    if (!console_.ready()) {
        return;
    }

    std::size_t sent_records = 0U;

    while (sent_records < kMaxStatusTextPerRun &&
           mavlink_log_subscription_.update()) {
        const mavlink_log_s &mavlink_log = mavlink_log_subscription_.get();

        // 超过 5 s 的文本已失去操作时效，丢弃而不占用当前 USB 带宽。
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
                // 最后一片不足 50 字节时补零，满足 MAVLink 定长文本字段语义。
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
    // mode=3 进入 MCUboot Recovery，其余已批准 mode 走普通平台复位。
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
