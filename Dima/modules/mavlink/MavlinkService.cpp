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

constexpr std::uint64_t kRebootSaveTimeoutUs = 10000000ULL;
constexpr std::uint64_t kRebootSafetyTimeoutUs = 750000ULL;

bool parameters_saved_for_reboot() noexcept
{
    // 调用者持有参数事务锁；只读取生成目录与 RAM 脏标记，绝不在 MAVLink
    // 工作队列调用存储后端。volatile 值无需落盘，校准暂存期则必须等待结束。
    if (!param_is_ready() || param_storage_paused()) return false;
    for (unsigned index = 0U; index < param_count(); ++index) {
        const param_t parameter = param_for_index(index);
        if (parameter == PARAM_INVALID ||
            (!param_is_volatile(parameter) && param_value_unsaved(parameter))) {
            return false;
        }
    }
    return true;
}

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

MavlinkEndpoint::MavlinkEndpoint(
    MavlinkTransport &transport, MavlinkSharedState &shared, std::uint8_t channel,
    dima::platform::BootControl &boot_control,
    dima::modules::mission::MissionService &mission_service,
    dima::platform::LogFileStore &log_files) noexcept
    : channel_(channel), shared_(shared), transport_(transport), boot_control_(boot_control),
      mission_(mission_service, &MavlinkEndpoint::send_frame, this, channel),
      log_handler_(log_files, shared.log_lease, &MavlinkEndpoint::send_log_batch, this, channel)
{
    metadata_ftp_.init(kMetadataFiles, static_cast<std::uint8_t>(
        sizeof(kMetadataFiles) / sizeof(kMetadataFiles[0])));
}

bool MavlinkEndpoint::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) return true;
    reset_runtime_state();
    identity_.configure(dima::generated::firmware_identity::kFlightSoftwareVersion,
        dima::platform::board_version(), dima::platform::board_hardware_uid());
    rc_loss_timeout_handle_ = param_handle(dima::params::COM_RC_LOSS_T);
    mav_system_id_handle_ = param_handle(dima::params::MAV_SYS_ID);
    rate_handle_ = param_handle(dima::params::MAV_0_RATE);
    if (rc_loss_timeout_handle_ == PARAM_INVALID || mav_system_id_handle_ == PARAM_INVALID ||
        rate_handle_ == PARAM_INVALID || !refresh_protocol_parameters() ||
        !parameters_.prepare_parameter_catalogue() || !log_handler_.start()) {
        log_handler_.stop();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void MavlinkEndpoint::stop() noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    log_handler_.stop();
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState MavlinkEndpoint::state() const noexcept
{
    return state_;
}

void MavlinkEndpoint::reset_runtime_state() noexcept
{
    reset_link();
    identity_.set_state(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MAV_STATE_BOOT);
    mission_.reset();
    reboot_mode_pending_ = 0;
    reboot_deadline_us_ = 0U;
    reboot_save_deadline_us_ = 0U;
    reboot_ack_completed_ = false;
    reboot_save_requested_ = false;
    reboot_tx_completion_baseline_ = reboot_tx_error_baseline_ = 0U;
    wait_reboot_completion_ = false;
    latest_input_rc_ = {};
    reset_sensor_streams();
    rc_loss_timeout_handle_ = mav_system_id_handle_ = rate_handle_ = PARAM_INVALID;
    rc_loss_timeout_s_ = 0.0F;
    have_input_rc_ = rc_loss_timeout_valid_ = false;
}

void MavlinkEndpoint::reset_link() noexcept
{
    // 连接代次失效时旧 ACK 的完成证据也失效，不能在重新连线后执行旧重启请求。
    if (reboot_mode_pending_ != 0) {
        reboot_mode_pending_ = 0;
        reboot_deadline_us_ = reboot_save_deadline_us_ = 0U;
        reboot_ack_completed_ = reboot_save_requested_ = false;
        shared_.cancel_reboot();
        PX4_WARN("Link reset; reboot cancelled");
    }
    // 连接代次同时保护 Commander ACK 路由；只撤销本链路的 Mission token/日志租约。
    // storage 复位异步执行，LP worker 不等待文件关闭，也不触碰另一条链路的 channel。
    shared_.reset_link(channel_, epoch_++);
    reset_parser_state();
    parameters_.reset();
    mission_.reset_link();
    log_handler_.reset_link();
    metadata_ftp_.reset();
    timesync_.reset();
    heartbeat_pacer_.reset();
    reset_sensor_link_state();
    tx_head_ = tx_count_ = 0U;
    wait_reboot_completion_ = false;
    tx_class_ = TxClass::Reply;
    statustext_id_ = 0U;
    was_link_ready_ = transport_was_ready_ = false;
    rate_window_us_ = 0U;
    transaction_bytes_ = stream_attempts_ = stream_blocked_ = 0U;
    rate_multiplier_ = 1.0F;
}

void MavlinkEndpoint::reset_parser_state() noexcept
{
    *mavlink_get_channel_status(channel_) = mavlink_status_t{};
    *mavlink_get_channel_buffer(channel_) = mavlink_message_t{};
    parse_message_ = {};
    parse_status_ = {};
    rx_position_ = rx_size_ = 0U;
    rx_message_pending_ = false;
}

void MavlinkEndpoint::discard_rx() noexcept
{
    // RX 每轮有界；故障后的其余字节由官方 parser 重新同步帧头。
    (void)transport_.read(rx_buffer_, sizeof(rx_buffer_));
    rx_position_ = rx_size_ = 0U;
    rx_message_pending_ = false;
}

void MavlinkEndpoint::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    usb_deadline_us_ = hrt_absolute_time() + kTxTimeoutMs * 1000ULL;
    transport_.service();
    const bool ready = transport_.ready();
    const auto errors = transport_.error_generation();
    if (!ready && transport_was_ready_) {
        // 真实端口关闭（停止/换档/quiesce）才是链路事件：撤销半帧并全量复位
        // 协议会话，等价于 PX4 实例失去端口。
        discard_rx();
        reset_link();
    }
    /* 字节级错误（丢字节/framing/溢出）只作为统计计数保留，与 PX4 一致：
     * 错帧由 parser 校验失败自行丢弃并按帧头重同步，固件层不得再 discard
     * 接收缓冲——那会把同批有效字节一并丢弃，线路噪声下会吞掉 QGC 的
     * 参数请求并造成会话反复掉线；完整通过 CRC 的帧不可能跨越丢字节拼成。 */
    error_generation_ = errors;
    transport_was_ready_ = ready;
    flush_tx();
    maybe_perform_reboot(hrt_absolute_time());
    if (reboot_mode_pending_ != 0 || shared_.reboot_pending() || !ready) {
        mission_.update(hrt_absolute_time(), false);
        // 保存等待可能跨越多次心跳周期；继续报告连接与取消原因，但不再接收
        // 新的参数/命令。ACK 的发送完成证据独立锁存，不被后续心跳覆盖。
        if (ready && (reboot_mode_pending_ == 0 || reboot_ack_completed_)) {
            mavlink_message_t heartbeat{};
            if (background_space() && heartbeat_pacer_.tick(hrt_absolute_time(), heartbeat)) {
                if (!send_message(heartbeat)) heartbeat_pacer_.reset();
            }
            if (background_space()) stream_statustext();
        }
        return;
    }

    update_rc_input();
    update_sensor_topics();
    if (parameter_update_subscription_.update() && !refresh_protocol_parameters()) {
        PX4_ERR("MAVLink protocol parameters invalid");
    }
    tx_class_ = TxClass::Reply;
    drain_rx();
    flush_tx();
    const auto now = hrt_absolute_time();
    update_rate_mult(now);
    if (!was_link_ready_ && tx_count_ == 0U) {
        mavlink_message_t heartbeat{};
        heartbeat_pacer_.pack_now(now, heartbeat);
        if (send_message(heartbeat) && send_autopilot_version()) {
            // UART 配置成功只表示端点可用，不表示无线端/GCS 已连接。传感器
            // 摘要是非错误 Info，由服务层在本 Runtime 首次链路就绪时输出一次，
            // 不随链路重建重复；健康变化走边沿消息，持续健康走 SYS_STATUS。
            was_link_ready_ = true;
        } else {
            heartbeat_pacer_.reset();
        }
    }
    if (!was_link_ready_) return;
    mavlink_message_t heartbeat{};
    if (tx_count_ < kTxQueueCapacity && heartbeat_pacer_.tick(now, heartbeat)) {
        if (!send_message(heartbeat)) heartbeat_pacer_.reset();
    }
    // ACK/心跳先入队；已编码帧保持 FIFO 顺序，后台只允许预排一帧。
    if (tx_count_ == 0U) mission_.update(now, true);
    flush_tx();
    tx_class_ = TxClass::Bulk;
    if (background_space()) stream_available_modes();
    tx_class_ = TxClass::Stream;
    stream_configured_messages(now, stream_contract::TxStage::PreMetadata);
    tx_class_ = TxClass::Bulk;
    if (background_space()) (void)metadata_ftp_.service(now);
    tx_class_ = TxClass::Stream;
    stream_configured_messages(now, stream_contract::TxStage::PostMetadata);
    flush_tx();
    tx_class_ = TxClass::Bulk;
    if (background_space()) log_handler_.send(channel_ == MAVLINK_COMM_0 ? 16U : 1U);
    if (background_space()) parameters_.send();
    // 一条诊断记录可能含多个片，给整条记录预留回复队列容量。
    if (tx_count_ == 0U && transport_.tx_free_bytes() != 0U) {
        tx_class_ = TxClass::Reply;
        stream_statustext();
    }
    flush_tx();
    tx_class_ = TxClass::Reply;
}

void MavlinkEndpoint::drain_rx() noexcept
{
    // 解析完成后若暂时无回应容量，保留消息和余下字节。恢复时只 dispatch 一次，
    // 参数原子写入及 Commander 发布不会因 EAGAIN 重做。空队列可容纳端口迁移回显。
    std::size_t processed = 0U;
    while (processed < kRxBatchBytes) {
        if (rx_message_pending_) {
            if (tx_count_ != 0U) return;
            if (!dispatch(parse_message_)) return;
            rx_message_pending_ = false;
            if (tx_count_ != 0U) return;
        }
        if (rx_position_ == rx_size_) {
            rx_size_ = transport_.read(rx_buffer_, sizeof(rx_buffer_));
            rx_position_ = 0U;
            if (rx_size_ == 0U) return;
        }
        ++processed;
        rx_message_pending_ = mavlink_parse_char(channel_, rx_buffer_[rx_position_++],
            &parse_message_, &parse_status_) != 0;
    }
}

bool MavlinkEndpoint::dispatch(const mavlink_message_t &msg) noexcept
{
    // 接收路由由 mavlink.lock.json 生成；源码只实现 handler 行为，不维护
    // msgid/consumer 的第二份 switch 清单。
    const stream_contract::InboundMessageContract *inbound =
        stream_contract::find_inbound_message(msg.msgid);
    if (inbound == nullptr) {
        // GCS HEARTBEAT 及当前消费集合之外的消息静默忽略。
        return true;
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

    case stream_contract::InboundHandler::LogTransfer:
        return log_handler_.handle_message(msg);

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
    return true;
}

void MavlinkEndpoint::handle_ping(const mavlink_message_t &msg) noexcept
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
    mavlink_msg_ping_encode_chan(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, channel_,
                            &frame, &reply);
    (void)send_message(frame);
}

/* ── TX path ─────────────────────────────────────────────────────── */

bool MavlinkEndpoint::send_message(mavlink_message_t &msg, std::uint32_t) noexcept
{
    const auto length = mavlink_msg_to_send_buffer(tx_buffer_, &msg);
    return enqueue_frame(tx_buffer_, length);
}

bool MavlinkEndpoint::send_frame(void *ctx, mavlink_message_t &msg) noexcept
{
    return ctx != nullptr && static_cast<MavlinkEndpoint *>(ctx)->send_message(msg);
}

bool MavlinkEndpoint::send_log_batch(void *ctx, const std::uint8_t *data, std::size_t length) noexcept
{
    if (ctx == nullptr) return false;
    auto &self = *static_cast<MavlinkEndpoint *>(ctx);
    if (self.channel_ != MAVLINK_COMM_0) return self.enqueue_frame(data, length);
    // USB 仍保留 16 帧完整日志聚合；它与其他写共享本轮 5 ms 总等待预算。
    if (self.tx_count_ != 0U || self.usb_timeout_remaining() == 0U) { errno = EAGAIN; return false; }
    const bool sent = self.transport_.write(data, length, self.usb_timeout_remaining()) == static_cast<int>(length);
    if (sent) self.transaction_bytes_ += length;
    return sent;
}

void MavlinkEndpoint::send_frame_void(void *ctx, mavlink_message_t &msg) noexcept
{
    if (ctx != nullptr) (void)static_cast<MavlinkEndpoint *>(ctx)->send_message(msg);
}

void MavlinkEndpoint::reset_configured_streams() noexcept
{
    std::size_t index = 0U;
    for (const stream_contract::MessageContract &contract :
         stream_contract::kMessages) {
        if (contract.scheduler != stream_contract::Scheduler::Service) {
            continue;
        }
        if (index < configured_streams_.size()) {
            configured_streams_[index].interval_us =
                default_interval(contract);
            configured_streams_[index].last_tx_us = 0U;
        }
        ++index;
    }
    rc_stream_active_ = false;
    available_modes_next_ = available_modes_end_ = 0U;
    last_current_mode_ = {};
    have_current_mode_tx_ = false;
}

bool MavlinkEndpoint::send_contract_message(
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
    case stream_contract::MessageHandler::AvailableModes:
        // 全量/单项索引只由 request_message() 冻结，不能经无参周期入口发送。
        return false;
    case stream_contract::MessageHandler::CurrentMode:
        return send_current_mode();
    case stream_contract::MessageHandler::ComponentMetadata:
        return send_component_metadata();
    case stream_contract::MessageHandler::ComponentInformation:
        return send_component_information();
    case stream_contract::MessageHandler::StorageInformation:
        // 容量查询可进入 FatFs，只能由 request_message() 投递到
        // wq:storage；周期调度和 MAVLink owner 不得直接调用。
        return false;
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
    case stream_contract::MessageHandler::Attitude:
        if (refresh_topics) update_sensor_topics();
        return send_attitude(now);
    case stream_contract::MessageHandler::LocalPositionNed:
        if (refresh_topics) update_sensor_topics();
        return send_local_position_ned(now);
    case stream_contract::MessageHandler::GlobalPositionInt:
        if (refresh_topics) update_sensor_topics();
        return send_global_position_int(now);
    case stream_contract::MessageHandler::EstimatorStatus:
        if (refresh_topics) update_sensor_topics();
        return send_estimator_status(now);
    }
    return false;
}

void MavlinkEndpoint::stream_configured_messages(
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
        // CURRENT_MODE 使用官方 0.5 Hz 默认节拍，同时在当前/意图模式变化时
        // 立即发送；SET_MESSAGE_INTERVAL 停流仍由上面的负间隔门禁统一处理。
        const bool mode_changed = contract.handler ==
            stream_contract::MessageHandler::CurrentMode && current_mode_changed();
        // PX4 按倍率延长实际周期，保存的间隔仍用于 GET 原样回报。
        const auto effective = static_cast<std::int32_t>(std::min<double>(
            std::numeric_limits<std::int32_t>::max(), state.interval_us / rate_multiplier_));
        if (!(stream_due(now, state.last_tx_us, effective) || mode_changed)) continue;
        ++stream_attempts_;
        if (!background_space()) { ++stream_blocked_; continue; }
        if (send_contract_message(contract.handler, now, false)) {
            state.last_tx_us = now;
            // UART 空闲时立即异步提交，避免同轮多个到期流仅因软件槽位互相阻塞。
            if (channel_ != MAVLINK_COMM_0) flush_tx();
        }
    }
}

std::uint8_t MavlinkEndpoint::request_message(void *ctx,
                                             std::uint16_t message_id,
                                             float param2, float param3,
                                             float param4, float param5,
                                             float param6, float param7) noexcept
{
    if (ctx == nullptr) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
    }
    auto &self = *static_cast<MavlinkEndpoint *>(ctx);
    const stream_contract::MessageContract *contract =
        stream_contract::find_message(message_id);
    if (contract == nullptr || !contract->requestable) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
    }
    if (contract->handler == stream_contract::MessageHandler::AvailableModes) {
        return self.request_available_modes(param2);
    }
    if (contract->handler ==
        stream_contract::MessageHandler::StorageInformation) {
        // PX4 v1.17 的 STORAGE_INFORMATION request_message 对 param2 做 roundf，
        // 并且只接受 0=全部或 1=第一块。其他参数按上游语义忽略。
        if (!std::isfinite(param2)) {
            return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
        }
        const long requested_storage = std::lround(param2);
        if (requested_storage < 0L || requested_storage > 1L) {
            return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
        }
        return self.log_handler_.request_storage_information(
                   static_cast<std::uint8_t>(requested_storage))
            ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
            : vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
    }

    (void)param3;
    (void)param4;
    (void)param5;
    (void)param6;
    (void)param7;
    return self.send_contract_message(
               contract->handler, hrt_absolute_time(), true)
        ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
        : vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
}

std::uint8_t MavlinkEndpoint::set_message_interval(
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

    auto &self = *static_cast<MavlinkEndpoint *>(ctx);
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
    std::int32_t selected_interval = self.default_interval(*contract);
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

std::uint8_t MavlinkEndpoint::get_message_interval(
    void *ctx, std::uint16_t message_id) noexcept
{
    if (ctx == nullptr) return vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
    auto &self = *static_cast<MavlinkEndpoint *>(ctx);
    std::int32_t interval_us = -1;
    const stream_contract::MessageContract *contract =
        stream_contract::find_message(message_id);
    if (contract != nullptr) {
        interval_us = self.default_interval(*contract);
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
    mavlink_msg_message_interval_encode_chan(MAVLINK_SYSTEM_ID,
                                        MAVLINK_COMPONENT_ID, self.channel_,
                                        &message, &report);
    return self.send_message(message)
        ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
        : vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
}

std::uint8_t MavlinkEndpoint::dispatch_command(void *context, const vehicle_command_s &command) noexcept
{
    auto &self = *static_cast<MavlinkEndpoint *>(context);
    return self.shared_.submit(self.channel_, self.epoch_, command);
}

void MavlinkEndpoint::acknowledge_local(void *context, const vehicle_command_ack_s &ack) noexcept
{
    auto &self = *static_cast<MavlinkEndpoint *>(context);
    (void)self.queue_command_ack(ack, self.epoch_);
}

bool MavlinkEndpoint::queue_command_ack(const vehicle_command_ack_s &ack, std::uint32_t epoch) noexcept
{
    if (epoch != epoch_) return true;
    if (tx_count_ >= kTxQueueCapacity) return false;
    mavlink_command_ack_t reply{};
    reply.command = ack.command;
    reply.result = ack.result;
    reply.progress = ack.result_param1;
    reply.result_param2 = ack.result_param2;
    reply.target_system = ack.target_system;
    reply.target_component = ack.target_component;
    const bool reboot = ack.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN &&
        ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED &&
        (ack.result_param2 == 1U || (channel_ == MAVLINK_COMM_0 && ack.result_param2 == 3U));
    if (reboot) {
        const auto now = hrt_absolute_time();
        reboot_mode_pending_ = static_cast<int>(ack.result_param2);
        reboot_deadline_us_ = now + drain_timeout_us();
        reboot_save_deadline_us_ = now + kRebootSaveTimeoutUs;
        reboot_tx_error_baseline_ = transport_.tx_error_generation();
        wait_reboot_completion_ = reboot_ack_completed_ = reboot_save_requested_ = false;
        shared_.mark_reboot_pending();
    }
    send_command_ack(reply, reboot);
    return true;
}

void MavlinkEndpoint::send_command_ack(const mavlink_command_ack_t &ack, bool reboot_ack) noexcept
{
    mavlink_message_t message{};
    mavlink_msg_command_ack_encode_chan(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, channel_, &message, &ack);
    const auto length = mavlink_msg_to_send_buffer(tx_buffer_, &message);
    const auto previous = tx_class_;
    tx_class_ = TxClass::Reply;
    (void)enqueue_frame(tx_buffer_, length, reboot_ack);
    tx_class_ = previous;
}

void MavlinkEndpoint::maybe_perform_reboot(std::uint64_t now) noexcept
{
    if (reboot_mode_pending_ == 0) return;
    if (!transport_.ready()) {
        cancel_reboot("Reboot link unavailable");
        return;
    }
    // 发送失败代数包含恢复/stop 中的主动中止；接收噪声计数不参与此判据。
    // 即使 abort 已把 HAL 状态恢复为空闲，也不能把截断的 ACK 当成已送达。
    if (transport_.tx_error_generation() != reboot_tx_error_baseline_) {
        cancel_reboot("Reboot TX failed");
        return;
    }
    // Commander 批准后可能等待数秒保存；持续复查新鲜的 Disarmed 状态，
    // 不能在等待期间被 RC 解锁后沿用旧批准复位。
    (void)vehicle_status_subscription_.update();
    const auto &status = vehicle_status_subscription_.get();
    const auto safety_now = hrt_absolute_time();
    if (status.timestamp == 0U || safety_now < status.timestamp ||
        safety_now - status.timestamp > kRebootSafetyTimeoutUs ||
        status.arming_state != vehicle_status_s::ARMING_STATE_DISARMED) {
        cancel_reboot("Reboot requires fresh disarmed state");
        return;
    }
    if (!reboot_ack_completed_) {
        if (now >= reboot_deadline_us_) {
            cancel_reboot("Reboot ACK completion timeout");
            return;
        }
        // 正常完成代数必须跨过 ACK 提交前基线，且整个队列已排空。后续存储
        // 等待独立计时，不能把原有 ACK 超时直接延长到存储超时。
        reboot_ack_completed_ = wait_reboot_completion_ && tx_drained() &&
            transport_.tx_completion_generation() != reboot_tx_completion_baseline_;
    }

    bool saved = false;
    {
        px4::AtomicTransaction transaction;
        saved = parameters_saved_for_reboot();
        // 最终检查到复位之间保持参数锁，防止另一个任务在“已保存”检查后
        // 写入新值。存储 worker 完成当前快照且无更新后才会清除脏标记。
        if (saved && reboot_ack_completed_) perform_reboot();
        if (!saved && !reboot_save_requested_) {
            reboot_save_requested_ = true;
            // 仅唤醒既有 autosave；Flash/SD 与保存失败重试仍由 storage 执行。
            param_notify_changes();
        }
    }
    if (!saved && now >= reboot_save_deadline_us_) {
        cancel_reboot("Parameter save incomplete or failed");
    }
}

void MavlinkEndpoint::cancel_reboot(const char *reason) noexcept
{
    // 所有失败路径撤销本次请求并恢复两条链路服务，绝不以复位代替保存/发送失败。
    reboot_mode_pending_ = 0;
    reboot_deadline_us_ = reboot_save_deadline_us_ = 0U;
    wait_reboot_completion_ = reboot_ack_completed_ = reboot_save_requested_ = false;
    shared_.cancel_reboot();
    reset_link();
    PX4_ERR("%s; reboot cancelled", reason);
}

/* ── STATUSTEXT stream (ported from streams/STATUSTEXT.hpp) ──────── */

void MavlinkEndpoint::stream_statustext() noexcept
{
    // USB 断开时不消费日志 Topic，尽量保留记录给下一次连接。
    if (!transport_.ready()) {
        return;
    }

    std::size_t sent_records = 0U;

    while (sent_records < kMaxStatusTextPerRun &&
           tx_count_ <= kTxQueueCapacity - 4U &&
           mavlink_log_subscription_.update()) {
        const mavlink_log_s &mavlink_log = mavlink_log_subscription_.get();

        // 超过 5 s 的文本已失去操作时效，丢弃而不占用当前 USB 带宽。
        if (hrt_elapsed_time(&mavlink_log.timestamp) >= 5000000ULL) {
            continue;
        }

        mavlink_statustext_t msg{};
        const char *text = mavlink_log.text;
        constexpr unsigned max_chunk_size = sizeof(msg.text);
        const std::size_t total_text_size = std::strlen(text);
        msg.severity = mavlink_log.severity;
        msg.chunk_seq = 0;
        // common.xml：id=0 明确表示单片，可直接发送恰好 50 字符的短文本。
        // 长文本必须使用非零 id，计数回绕也不能把首片误报为独立消息。
        if (total_text_size > max_chunk_size) {
            msg.id = statustext_id_++;
            if (msg.id == 0U) msg.id = statustext_id_++;
        }
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
            mavlink_msg_statustext_encode_chan(MAVLINK_SYSTEM_ID,
                                          MAVLINK_COMPONENT_ID, channel_,
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

        if (send_ok && total_text_size > max_chunk_size &&
            total_text_size % max_chunk_size == 0U) {
            // 长文本整 50 倍数时追加同 id 的 NUL 终止片。QGC 5.1.3 会把
            // 完全空片误判为缺片，因此仅在传输末尾添加一个无害空格再 NUL；
            // 不修改原日志、不掩盖发送失败，也不增加本轮日志记录预算。
            std::memset(msg.text, 0, sizeof(msg.text));
            msg.text[0] = ' ';
            msg.chunk_seq += 1;
            mavlink_message_t frame{};
            mavlink_msg_statustext_encode_chan(MAVLINK_SYSTEM_ID,
                                          MAVLINK_COMPONENT_ID, channel_,
                                          &frame, &msg);
            send_ok = send_message(frame);
        }

        if (!send_ok) {
            break;
        }
        ++sent_records;
    }
}

/* ── Deferred reboot ─────────────────────────────────────────────── */

void MavlinkEndpoint::perform_reboot() noexcept
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
