#define MODULE_NAME "dronecan_mag2"
#include "DroneCanMag2.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"

#include <canard.h>
#include <dronecan_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::drivers::magnetometer {
namespace {

using dima::protocols::dronecan::DroneCanNode;
using dima::protocols::dronecan::TransferIdTracker;

std::uint32_t transport_error_count(
    const dima::platform::CanStats &stats) noexcept
{
    // sensor_mag.error_count 汇总底层可归因错误：RX 溢出、RX/TX 错误、bus-off
    // 和恢复失败；普通协议过滤、重复或非目标源不冒充硬件传输错误。
    return stats.receive_overruns + stats.receive_errors +
           stats.transmit_errors + stats.bus_off_events +
           stats.recovery_failures;
}

} // namespace

DroneCanMag2::DroneCanMag2(
    dima::platform::CanTransport &transport,
    dima::platform::ArmedFlashCoordinator &armed,
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        &maintenance,
    dima::parameters::FlashFS &allocation_storage) noexcept
    : px4::ScheduledWorkItem("dronecan_mag2", px4::wq_configurations::io),
      transport_(transport), protocol_node_(transport), armed_(armed),
      maintenance_(maintenance), allocation_storage_(allocation_storage)
{
}

bool DroneCanMag2::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    // 参数句柄来自统一 YAML 生成目录；先绑定完整快照并 advertise，再让
    // WorkQueue 应用，避免模块进入 Running 后才发现参数或 uORB 资源不可用。
    if (!bind_parameters() || !load_configuration(pending_configuration_) ||
        !sensor_mag_publication_.advertise()) {
        invalidate_parameters();
        ++stats_.parameter_failures;
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }

    startup_configuration_pending_ = true;
    configuration_pending_ = false;
    reconfigure_phase_ = ReconfigurePhase::Idle;
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleNow()) {
        startup_configuration_pending_ = false;
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    return true;
}

void DroneCanMag2::stop()
{
    // 先停止调度并释放维护票据，再关闭 DroneCAN/transport；所有在线绑定与
    // 一次性日志门限随生命周期清零，下一次 start 是全新会话。
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    cancel_maintenance();
    stop_protocol();
    invalidate_parameters();
    stats_ = Stats{};
    next_allocation_error_log_us_ = 0U;
    manual_mode_warning_reported_ = false;
    protocol_start_log_reported_ = false;
    allocation_ready_log_reported_ = false;
    anonymous_request_log_reported_ = false;
    source_absence_log_reported_ = false;
    source_detection_log_reported_ = false;
    startup_configuration_pending_ = false;
    configuration_pending_ = false;
    reconfigure_phase_ = ReconfigurePhase::Idle;
}

dima::middleware::lifecycle::ModuleState DroneCanMag2::state() const
{
    return state_;
}

void DroneCanMag2::reset_source_state(std::uint64_t now) noexcept
{
    // transport 配置变化使 transfer-ID 会话和源时间域全部失效；配置了固定
    // magnetic_node_id 时预先绑定该源，0 则允许首个合法节点完成绑定。
    magnetic_transfer_ids_.reset();
    start_time_us_ = now;
    last_magnetic_time_us_ = 0U;
    active_device_id_ = 0U;
    active_source_node_id_ = configuration_.magnetic_node_id;
    source_online_ = false;
    source_timeout_reported_ = false;
}

void DroneCanMag2::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    const std::uint64_t now = hrt_absolute_time();

    // parameter_update 只触发重读权威参数值；变化值放入 pending，实际 CAN
    // 重启由 process_reconfiguration 在 disarmed maintenance 窗口内完成。
    parameter_update_s update{};
    if (parameter_update_subscription_.copy(&update)) {
        Configuration next{};
        if (!update_parameters() || !load_configuration(next)) {
            ++stats_.parameter_failures;
            PX4_ERR("DroneCAN parameter update rejected");
        } else if (startup_configuration_pending_) {
            pending_configuration_ = next;
        } else if (same_configuration(next, configuration_)) {
            configuration_pending_ = false;
            cancel_maintenance();
        } else {
            pending_configuration_ = next;
            configuration_pending_ = true;
            next_reconfigure_retry_us_ = now;
        }
    }

    process_reconfiguration(now);

    // service 失败关闭当前协议会话并以 100 ms 重试，模块任务仍 Running，
    // 使临时 bus-off/transport 故障具备自恢复路径。
    if (configuration_.enabled) {
        if (protocol_started_) {
            protocol_node_.set_health_warning(source_timeout_reported_);
            if (!protocol_node_.service(now)) {
                synchronize_protocol_stats();
                protocol_started_ = false;
                next_transport_retry_us_ = now + kTransportRetryUs;
            }
            if (protocol_started_) {
                process_periodic(now);
                synchronize_protocol_stats();
            }
        } else if (now >= next_transport_retry_us_ && !start_protocol(now)) {
            next_transport_retry_us_ = now + kTransportRetryUs;
        }
    }

    const std::uint32_t delay = configuration_.enabled
                                    ? kPollIntervalUs
                                    : kDisabledPollIntervalUs;
    if (!ScheduleDelayed(delay)) {
        stop_protocol();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
    }
}

void DroneCanMag2::process_periodic(std::uint64_t now) noexcept
{
    // 首帧前从协议启动时间计 500 ms，首帧后从最近磁场样本计时。每次离线边沿
    // 都累计 source_timeouts；恢复样本只清 timeout 边沿状态，使后续离线可再次
    // 计数。详细 not-detected/timeout 告警每个协议配置会话只打印一次，避免反复
    // 掉线时占满日志链路，重配或协议重启后才重新开放该详细告警。
    const std::uint64_t reference = last_magnetic_time_us_ != 0U
                                        ? last_magnetic_time_us_
                                        : start_time_us_;
    if (!source_timeout_reported_ && now >= reference &&
        now - reference >= kSourceTimeoutUs) {
        source_online_ = false;
        source_timeout_reported_ = true;
        ++stats_.source_timeouts;
        if (source_absence_log_reported_) return;
        source_absence_log_reported_ = true;
        const auto can = transport_.stats();
        if (last_magnetic_time_us_ == 0U) {
            PX4_WARN("DroneCAN mag not detected node=%u can_rx=%lu accepted=%lu reject=%lu decode=%lu",
                     configuration_.magnetic_node_id,
                     static_cast<unsigned long>(can.received_frames),
                     static_cast<unsigned long>(stats_.accepted_transfers),
                     static_cast<unsigned long>(stats_.rejected_sources),
                     static_cast<unsigned long>(stats_.decode_errors));
        } else {
            PX4_WARN("DroneCAN mag timeout node=%u can_rx=%lu accepted=%lu reject=%lu decode=%lu",
                     active_source_node_id_,
                     static_cast<unsigned long>(can.received_frames),
                     static_cast<unsigned long>(stats_.accepted_transfers),
                     static_cast<unsigned long>(stats_.rejected_sources),
                     static_cast<unsigned long>(stats_.decode_errors));
        }
        if (transport_error_count(can) != 0U || can.last_error_flags != 0U) {
            PX4_WARN("DroneCAN CAN ov=%lu rx=%lu tx=%lu bo=%lu rec=%lu fl=0x%lx",
                     static_cast<unsigned long>(can.receive_overruns),
                     static_cast<unsigned long>(can.receive_errors),
                     static_cast<unsigned long>(can.transmit_errors),
                     static_cast<unsigned long>(can.bus_off_events),
                     static_cast<unsigned long>(can.recovery_failures),
                     static_cast<unsigned long>(can.last_error_flags));
        }
    }
}

bool DroneCanMag2::should_accept_broadcast(
    std::uint64_t &signature, std::uint16_t data_type_id,
    std::uint8_t source_node_id) const noexcept
{
    // DSDL data type ID/signature 只从生成订阅表取得；同时在 libcanard 分配
    // payload 前过滤匿名源和非活动节点，降低固定内存压力。
    const auto *const descriptor =
        dima::protocols::dronecan::generated::find_subscription(
            dima::protocols::dronecan::generated::
                SubscriptionOwner::Equipment,
            dima::protocols::dronecan::generated::TransferKind::Broadcast,
            data_type_id);
    if (descriptor != nullptr &&
        source_node_id != CANARD_BROADCAST_NODE_ID &&
        (active_source_node_id_ == 0U ||
         source_node_id == active_source_node_id_)) {
        signature = descriptor->signature;
        return true;
    }
    return false;
}

void DroneCanMag2::synchronize_protocol_stats() noexcept
{
    // DroneCanNode 每次 start 清零内部统计，本层保存上次快照并只累加差值，
    // 从而跨 transport 自动重启保留模块生命周期累计值。
    const auto &current = protocol_node_.stats();
    stats_.protocol_errors +=
        current.protocol_errors - protocol_stats_snapshot_.protocol_errors;
    stats_.node_status_transfers += current.node_status_transfers -
        protocol_stats_snapshot_.node_status_transfers;
    stats_.node_info_responses += current.node_info_responses -
        protocol_stats_snapshot_.node_info_responses;
    stats_.transport_failures += current.transport_failures -
        protocol_stats_snapshot_.transport_failures;
    stats_.allocation_requests += current.allocation_requests -
        protocol_stats_snapshot_.allocation_requests;
    stats_.allocation_successes += current.allocation_successes -
        protocol_stats_snapshot_.allocation_successes;
    stats_.allocation_malformed += current.allocation_malformed -
        protocol_stats_snapshot_.allocation_malformed;
    stats_.allocation_timeouts += current.allocation_timeouts -
        protocol_stats_snapshot_.allocation_timeouts;
    stats_.allocation_storage_failures +=
        current.allocation_storage_failures -
        protocol_stats_snapshot_.allocation_storage_failures;
    stats_.discovered_nodes += current.discovered_nodes -
        protocol_stats_snapshot_.discovered_nodes;
    protocol_stats_snapshot_ = current;
}

void DroneCanMag2::handle_allocation_event(
    const DroneCanNode::AllocationEvent &event) noexcept
{
    // 正常里程碑各自按业务门限记录；高频 malformed/timeout/storage/conflict
    // 统一限频，避免坏节点占满日志链路。
    const bool error_event =
        event.kind == DroneCanNode::AllocationEventKind::MalformedRequest ||
        event.kind == DroneCanNode::AllocationEventKind::FollowupTimeout ||
        event.kind == DroneCanNode::AllocationEventKind::AllocationExhausted ||
        event.kind == DroneCanNode::AllocationEventKind::StorageFailure ||
        event.kind == DroneCanNode::AllocationEventKind::NodeConflict;
    if (error_event) {
        const std::uint64_t now = hrt_absolute_time();
        if (now < next_allocation_error_log_us_) return;
        next_allocation_error_log_us_ =
            now + dima::protocols::dronecan::generated::
                      kErrorLogIntervalUs;
    }
    switch (event.kind) {
    case DroneCanNode::AllocationEventKind::ServerReady:
        if (allocation_ready_log_reported_) break;
        allocation_ready_log_reported_ = true;
        PX4_INFO("DroneCAN DNA server ready node=%u bitrate=%lu mode=automatic",
                 event.node_id,
                 static_cast<unsigned long>(configuration_.bitrate));
        break;
    case DroneCanNode::AllocationEventKind::FirstAnonymousRequest:
        if (anonymous_request_log_reported_) break;
        anonymous_request_log_reported_ = true;
        PX4_INFO("DroneCAN DNA request preferred=%u uid=%08lx",
                 event.preferred_node_id,
                 static_cast<unsigned long>(
                     event.unique_id_fingerprint));
        break;
    case DroneCanNode::AllocationEventKind::AllocationSucceeded:
        PX4_INFO("DroneCAN DNA allocated node=%u uid=%08lx",
                 event.node_id,
                 static_cast<unsigned long>(
                     event.unique_id_fingerprint));
        break;
    case DroneCanNode::AllocationEventKind::MalformedRequest:
        PX4_WARN("DroneCAN DNA malformed request error=%ld",
                 static_cast<long>(event.error));
        break;
    case DroneCanNode::AllocationEventKind::FollowupTimeout:
        PX4_WARN("DroneCAN DNA followup timeout");
        break;
    case DroneCanNode::AllocationEventKind::AllocationExhausted:
        PX4_ERR("DroneCAN DNA allocation exhausted");
        break;
    case DroneCanNode::AllocationEventKind::StorageFailure:
        PX4_ERR("DroneCAN DNA storage error=%ld node=%u",
                static_cast<long>(event.error), event.node_id);
        break;
    case DroneCanNode::AllocationEventKind::NodeConflict:
        PX4_ERR("DroneCAN DNA node conflict node=%u uid=%08lx error=%ld",
                event.node_id,
                static_cast<unsigned long>(
                    event.unique_id_fingerprint),
                static_cast<long>(event.error));
        break;
    }
}

void DroneCanMag2::handle_magnetic_field(
    DroneCanNode::Transfer &transfer_view) noexcept
{
    auto *const transfer = static_cast<CanardRxTransfer *>(
        transfer_view.native_handle());
    if (transfer == nullptr) {
        ++stats_.protocol_errors;
        return;
    }
    const std::uint16_t data_type_id = transfer_view.data_type_id;
    const std::uint8_t transfer_type = transfer_view.transfer_type;
    const std::uint8_t source_node_id = transfer_view.source_node_id;
    const std::uint8_t transfer_id = transfer_view.transfer_id;
    const std::uint64_t timestamp_us = transfer_view.timestamp_us;
    const auto *const descriptor =
        dima::protocols::dronecan::generated::find_subscription(
            dima::protocols::dronecan::generated::
                SubscriptionOwner::Equipment,
            dima::protocols::dronecan::generated::TransferKind::Broadcast,
            data_type_id);
    if (descriptor == nullptr) {
        ++stats_.protocol_errors;
        return;
    }
    if (active_source_node_id_ != 0U &&
        source_node_id != active_source_node_id_) {
        ++stats_.rejected_sources;
        return;
    }
    // 解码前先以 (data type, transfer type, source) 的模 32 transfer-ID 会话
    // 拒绝重复和乱序旧帧，避免同一样本多次发布或时间倒退。
    const auto disposition = magnetic_transfer_ids_.observe(
        TransferIdTracker::Key{
            data_type_id, transfer_type, source_node_id},
        transfer_id, timestamp_us);
    if (disposition == TransferIdTracker::Disposition::Duplicate) {
        ++stats_.duplicate_transfers;
        return;
    }
    if (disposition == TransferIdTracker::Disposition::Stale) {
        ++stats_.stale_transfers;
        return;
    }

    // 兼容 MagneticFieldStrength2（带 sensor_id）和旧 MagneticFieldStrength；
    // 两者的 magnetic_field_ga 与 sensor_mag x/y/z 均为 gauss，不在后端换算。
    std::uint8_t sensor_id = 0U;
    float magnetic_field_ga[3]{};
    bool decode_failed = false;
    if (descriptor->role == dima::protocols::dronecan::generated::
                                MessageRole::MagneticFieldStrength2) {
        uavcan_equipment_ahrs_MagneticFieldStrength2 message{};
        decode_failed = uavcan_equipment_ahrs_MagneticFieldStrength2_decode(
            transfer, &message);
        sensor_id = message.sensor_id;
        std::copy_n(message.magnetic_field_ga, 3U, magnetic_field_ga);
    } else if (descriptor->role ==
               dima::protocols::dronecan::generated::
                   MessageRole::MagneticFieldStrength) {
        uavcan_equipment_ahrs_MagneticFieldStrength message{};
        decode_failed = uavcan_equipment_ahrs_MagneticFieldStrength_decode(
            transfer, &message);
        std::copy_n(message.magnetic_field_ga, 3U, magnetic_field_ga);
    } else {
        ++stats_.protocol_errors;
        return;
    }
    if (decode_failed) {
        ++stats_.decode_errors;
        return;
    }
    for (float component : magnetic_field_ga) {
        if (!std::isfinite(component)) {
            ++stats_.decode_errors;
            return;
        }
    }

    // 首个合法源锁定生成的 device_id；后续不同源即使数据可解码也拒绝，避免
    // 运行中校准对象静默切换。重配/协议重启才允许重新选择。
    const std::uint32_t candidate_device_id =
        make_device_id(source_node_id);
    if (active_device_id_ != 0U &&
        active_device_id_ != candidate_device_id) {
        ++stats_.rejected_sources;
        return;
    }
    const bool newly_online = !source_online_;
    active_source_node_id_ = source_node_id;
    active_device_id_ = candidate_device_id;
    last_magnetic_time_us_ = timestamp_us;
    source_online_ = true;
    source_timeout_reported_ = false;
    ++stats_.accepted_transfers;
    if (newly_online && !source_detection_log_reported_) {
        source_detection_log_reported_ = true;
        const auto can = transport_.stats();
        PX4_INFO("DroneCAN mag detected node=%u sensor=%u device_id=0x%08lx can_rx=%lu accepted=%lu",
                 source_node_id, sensor_id,
                 static_cast<unsigned long>(active_device_id_),
                 static_cast<unsigned long>(can.received_frames),
                 static_cast<unsigned long>(stats_.accepted_transfers));
    }

    const auto transport_stats = transport_.stats();
    // timestamp 是本地发布时刻，timestamp_sample 保留 CAN 接收时刻；远端消息
    // 不含温度，因此用 NaN 明确表示不可用而不是伪造 0 degC。
    sensor_mag_s sample{};
    sample.timestamp = hrt_absolute_time();
    sample.timestamp_sample = timestamp_us;
    sample.device_id = active_device_id_;
    sample.x = magnetic_field_ga[0];
    sample.y = magnetic_field_ga[1];
    sample.z = magnetic_field_ga[2];
    sample.temperature = std::numeric_limits<float>::quiet_NaN();
    sample.error_count = transport_error_count(transport_stats);
    if (sensor_mag_publication_.publish(sample)) {
        ++stats_.publications;
    } else {
        ++stats_.publication_failures;
    }
}

} // namespace dima::drivers::magnetometer
