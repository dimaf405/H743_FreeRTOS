#include "DroneCanNode.hpp"

#include <canard.h>
#include <dronecan_msgs.h>

#include <algorithm>
#include <cstring>

namespace dima::protocols::dronecan {
namespace {

static_assert(sizeof(CanardInstance) <= 64U,
              "DroneCAN instance storage contract is too small");
static_assert(alignof(CanardInstance) <= 8U,
              "DroneCAN instance alignment contract is too small");

CanardInstance *instance(void *storage) noexcept
{
    // 协议公开头文件只保留对齐字节数组；具体 libcanard 类型被限制在实现文件内。
    return static_cast<CanardInstance *>(storage);
}

bool transfer_kind(std::int32_t native,
                   generated::TransferKind &kind) noexcept
{
    switch (static_cast<CanardTransferType>(native)) {
    case CanardTransferTypeBroadcast:
        kind = generated::TransferKind::Broadcast;
        return true;
    case CanardTransferTypeRequest:
        kind = generated::TransferKind::Request;
        return true;
    case CanardTransferTypeResponse:
        kind = generated::TransferKind::Response;
        return true;
    default:
        return false;
    }
}

} // namespace

DroneCanNode::DroneCanNode(dima::platform::CanTransport &transport) noexcept
    : transport_(transport)
{
}

bool DroneCanNode::start(const Configuration &configuration,
                         const Callbacks &callbacks,
                         std::uint64_t now_us) noexcept
{
    // start 是幂等重建而非增量修改：先释放旧 transport/持久化事务，再验证
    // 完整配置，防止新旧节点 ID、回调或内存池状态交叉。
    stop();
    stats_ = {};
    if (configuration.bitrate == 0U || configuration.local_node_id == 0U ||
        configuration.local_node_id > generated::kMaximumNodeId ||
        configuration.identity.name == nullptr ||
        callbacks.accept_broadcast == nullptr ||
        callbacks.receive_broadcast == nullptr ||
        (configuration.automatic_allocation &&
         (configuration.allocation_storage.load == nullptr ||
          configuration.allocation_storage.begin_save == nullptr ||
          configuration.allocation_storage.continue_save == nullptr ||
          configuration.allocation_storage.cancel_save == nullptr))) {
        ++stats_.protocol_errors;
        return false;
    }

    configuration_ = configuration;
    callbacks_ = callbacks;
    auto *const canard = instance(instance_storage_);
    const auto receive_callback = +[](CanardInstance *canard_instance,
                                      CanardRxTransfer *transfer) {
        if (canard_instance == nullptr || transfer == nullptr) return;
        auto *const self = static_cast<DroneCanNode *>(
            canardGetUserReference(canard_instance));
        if (self != nullptr) self->receive(transfer);
    };
    const auto accept_callback = +[](
        const CanardInstance *canard_instance, std::uint64_t *signature,
        std::uint16_t data_type_id, CanardTransferType transfer_type,
        std::uint8_t source_node_id) -> bool {
        if (canard_instance == nullptr) return false;
        const auto *const self = static_cast<const DroneCanNode *>(
            canardGetUserReference(canard_instance));
        return self != nullptr && self->accept(
            signature, data_type_id, static_cast<std::int32_t>(transfer_type),
            source_node_id);
    };

    // libcanard 的所有会话分配都来自对象内 3072 B pool；清零后初始化回调，
    // user reference 是从 C ABI 回到当前 C++ 节点的唯一入口。
    std::memset(canard, 0, sizeof(*canard));
    std::memset(memory_pool_, 0, sizeof(memory_pool_));
    canardInit(canard, memory_pool_, sizeof(memory_pool_), receive_callback,
               accept_callback, this);
    canardSetLocalNodeID(canard, configuration.local_node_id);
    automatic_allocation_ = configuration.automatic_allocation;
    if (automatic_allocation_ && !initialize_allocation(now_us)) {
        ++stats_.allocation_storage_failures;
        callbacks_ = {};
        automatic_allocation_ = false;
        return false;
    }

    // DroneCAN v0 只使用 29-bit Extended Classic CAN 数据帧；具体 DSDL 签名/
    // data type ID 由生成合同在 libcanard accept 回调内继续过滤。
    dima::platform::CanConfiguration transport_configuration{};
    transport_configuration.bitrate = configuration.bitrate;
    transport_configuration.acceptance.filter.enabled = true;
    transport_configuration.acceptance.filter.identifier_type =
        dima::platform::CanIdentifierType::Extended;
    transport_configuration.acceptance.filter.type =
        dima::platform::CanFilterType::Mask;
    if (!transport_.start(transport_configuration)) {
        ++stats_.transport_failures;
        reset_allocation();
        callbacks_ = {};
        return false;
    }

    start_time_us_ = now_us;
    next_node_status_us_ = now_us;
    node_status_transfer_id_ = 0U;
    health_warning_ = false;
    running_ = true;
    if (automatic_allocation_ && allocation_ready_) {
        emit_allocation_event(AllocationEventKind::ServerReady,
                              configuration_.local_node_id, 0U,
                              configuration_.identity.unique_id, 0);
    }
    return true;
}

void DroneCanNode::stop() noexcept
{
    reset_allocation();
    if (running_ || transport_.running()) {
        transport_.stop();
    }
    running_ = false;
    callbacks_ = {};
}

bool DroneCanNode::service(std::uint64_t now_us) noexcept
{
    if (!running_) return false;
    if (!transport_.service()) {
        ++stats_.transport_failures;
        reset_allocation();
        running_ = false;
        return false;
    }

    // RX 必须先进入 libcanard，分配状态机才可消费本轮请求；随后 NodeStatus
    // 入队，最后统一冲刷 TX，Busy 时保留队首供下轮重试。
    process_rx();
    service_allocation(now_us);
    if (now_us >= next_node_status_us_) {
        send_node_status(now_us);
        canardCleanupStaleTransfers(instance(instance_storage_), now_us);
        next_node_status_us_ = now_us + kNodeStatusIntervalUs;
    }
    process_tx();
    return true;
}

void DroneCanNode::process_rx() noexcept
{
    // 每批最多搬运 16 帧，但循环排空底层 ring。非 Extended/Data 或 DLC 越界
    // 在边界处拒绝，不能把损坏帧交给 libcanard 解码。
    dima::platform::CanFrame frames[16]{};
    for (;;) {
        const std::size_t count = transport_.receive(frames, 16U);
        if (count == 0U) break;
        for (std::size_t index = 0U; index < count; ++index) {
            if (frames[index].identifier_type !=
                    dima::platform::CanIdentifierType::Extended ||
                frames[index].frame_type !=
                    dima::platform::CanFrameType::Data ||
                frames[index].data_length > sizeof(frames[index].data)) {
                ++stats_.protocol_errors;
                continue;
            }
            CanardCANFrame frame{};
            frame.id = frames[index].identifier | CANARD_CAN_FRAME_EFF;
            frame.data_len = frames[index].data_length;
            std::memcpy(frame.data, frames[index].data, frame.data_len);
            if (canardHandleRxFrame(instance(instance_storage_), &frame,
                                    frames[index].timestamp_us) < 0) {
                ++stats_.protocol_errors;
            }
        }
    }
}

void DroneCanNode::process_tx() noexcept
{
    for (;;) {
        CanardCANFrame *const queued =
            canardPeekTxQueue(instance(instance_storage_));
        if (queued == nullptr) break;
        dima::platform::CanFrame frame{};
        frame.identifier = queued->id & CANARD_CAN_EXT_ID_MASK;
        frame.data_length = queued->data_len;
        frame.identifier_type = dima::platform::CanIdentifierType::Extended;
        frame.frame_type = dima::platform::CanFrameType::Data;
        std::memcpy(frame.data, queued->data, frame.data_length);
        const auto result = transport_.transmit(frame);
        // Busy 表示硬件邮箱暂不可用：不能 pop，否则会静默丢帧；Error 才丢弃
        // 当前队首并计数，避免一个永久失败帧阻塞整个发送队列。
        if (result == dima::platform::CanTransmitResult::Busy) break;
        canardPopTxQueue(instance(instance_storage_));
        if (result == dima::platform::CanTransmitResult::Error) {
            ++stats_.transport_failures;
        }
    }
}

void DroneCanNode::send_node_status(std::uint64_t now_us) noexcept
{
    // uptime_sec=(now-start)/1e6；磁力计源超时仅把节点 health 降为 WARNING，
    // 节点仍保持 OPERATIONAL，从而让总线诊断与 GetNodeInfo 继续可见。
    uavcan_protocol_NodeStatus status{};
    status.uptime_sec = static_cast<std::uint32_t>(
        (now_us - start_time_us_) / 1000000ULL);
    status.health = health_warning_
                        ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING
                        : UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    status.sub_mode = 0U;
    status.vendor_specific_status_code = health_warning_ ? 1U : 0U;
    std::uint8_t payload[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE]{};
    const std::uint32_t length =
        uavcan_protocol_NodeStatus_encode(&status, payload);
    const auto *const descriptor = generated::find_subscription(
        generated::SubscriptionOwner::Node,
        generated::TransferKind::Broadcast,
        generated::MessageRole::NodeStatus);
    if (descriptor == nullptr) {
        ++stats_.protocol_errors;
        return;
    }
    const std::int16_t result = canardBroadcast(
        instance(instance_storage_), descriptor->signature,
        descriptor->data_type_id, &node_status_transfer_id_,
        CANARD_TRANSFER_PRIORITY_LOW, payload,
        static_cast<std::uint16_t>(length));
    if (result > 0) ++stats_.node_status_transfers;
    else if (result < 0) ++stats_.protocol_errors;
}

bool DroneCanNode::accept(std::uint64_t *signature,
                          std::uint16_t data_type_id,
                          std::int32_t transfer_type,
                          std::uint8_t source_node_id) const noexcept
{
    if (signature == nullptr) return false;
    generated::TransferKind kind{};
    if (!transfer_kind(transfer_type, kind)) return false;
    // 先查询生成的节点内部订阅（NodeInfo/NodeStatus/Allocation），再把未知业务
    // 广播交给设备回调。首方代码不手写 DSDL ID 或 signature。
    const auto *const internal = generated::find_subscription(
        generated::SubscriptionOwner::Node, kind, data_type_id);
    if (internal != nullptr) {
        const bool unicast_source =
            source_node_id != CANARD_BROADCAST_NODE_ID &&
            source_node_id <= generated::kMaximumNodeId;
        bool accepted = false;
        switch (internal->role) {
        case generated::MessageRole::NodeInfo:
            accepted = kind == generated::TransferKind::Request
                           ? unicast_source
                           : automatic_allocation_ && unicast_source;
            break;
        case generated::MessageRole::NodeStatus:
            accepted = automatic_allocation_ && unicast_source &&
                       source_node_id != configuration_.local_node_id;
            break;
        case generated::MessageRole::Allocation:
            accepted = automatic_allocation_;
            break;
        default:
            break;
        }
        if (accepted) {
            *signature = internal->signature;
            return true;
        }
    }
    return kind == generated::TransferKind::Broadcast &&
           callbacks_.accept_broadcast != nullptr &&
           callbacks_.accept_broadcast(callbacks_.context, *signature,
                                       data_type_id, source_node_id);
}

void DroneCanNode::receive(void *native_transfer) noexcept
{
    auto &transfer = *static_cast<CanardRxTransfer *>(native_transfer);
    generated::TransferKind kind{};
    bool internal_handled = false;
    if (transfer_kind(static_cast<std::int32_t>(transfer.transfer_type), kind)) {
        const auto *const internal = generated::find_subscription(
            generated::SubscriptionOwner::Node, kind,
            transfer.data_type_id);
        if (internal != nullptr) {
            internal_handled = true;
            if (internal->role == generated::MessageRole::NodeInfo &&
                kind == generated::TransferKind::Request) {
                respond_node_info(&transfer);
                return;
            }
            if (internal->role == generated::MessageRole::Allocation) {
                handle_allocation(&transfer);
            } else if (internal->role ==
                           generated::MessageRole::NodeStatus) {
                handle_node_status(&transfer);
            } else if (internal->role == generated::MessageRole::NodeInfo &&
                       kind == generated::TransferKind::Response) {
                handle_node_info_response(&transfer);
            }
        }
    }

    // 节点核心消息由内部状态机拥有；只有未内部处理的广播才下发设备驱动。
    if (!internal_handled &&
        transfer.transfer_type == CanardTransferTypeBroadcast &&
        callbacks_.receive_broadcast != nullptr) {
        Transfer view{};
        view.timestamp_us = transfer.timestamp_usec;
        view.data_type_id = transfer.data_type_id;
        view.source_node_id = transfer.source_node_id;
        view.transfer_id = transfer.transfer_id;
        view.transfer_type = transfer.transfer_type;
        view.native_handle_ = &transfer;
        // view 只在本次回调内有效；返回后统一释放 libcanard payload。
        // 消费者必须同步解码，不能保存 native_handle 或底层 payload 指针。
        callbacks_.receive_broadcast(callbacks_.context, view);
    }
    canardReleaseRxTransferPayload(instance(instance_storage_), &transfer);
}

void DroneCanNode::respond_node_info(void *native_transfer) noexcept
{
    auto &transfer = *static_cast<CanardRxTransfer *>(native_transfer);
    const std::uint8_t source_node_id = transfer.source_node_id;
    const std::uint8_t transfer_id = transfer.transfer_id;
    const std::uint8_t priority = transfer.priority;

    uavcan_protocol_GetNodeInfoResponse response{};
    const std::uint64_t uptime_us = transfer.timestamp_usec >= start_time_us_
                                        ? transfer.timestamp_usec - start_time_us_
                                        : 0U;
    response.status.uptime_sec = static_cast<std::uint32_t>(
        uptime_us / 1000000ULL);
    response.status.health = health_warning_
                                 ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING
                                 : UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    response.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    response.status.sub_mode = 0U;
    response.status.vendor_specific_status_code = health_warning_ ? 1U : 0U;
    response.software_version.major = configuration_.identity.software_major;
    response.software_version.minor = configuration_.identity.software_minor;
    response.software_version.optional_field_flags = 0U;
    response.hardware_version.major = configuration_.identity.hardware_major;
    response.hardware_version.minor = configuration_.identity.hardware_minor;
    std::copy_n(configuration_.identity.unique_id,
                sizeof(response.hardware_version.unique_id),
                response.hardware_version.unique_id);
    // DSDL name 是定长容量+显式长度，过长节点名安全截断且不要求 NUL。
    const std::size_t name_length = std::min<std::size_t>(
        std::strlen(configuration_.identity.name),
        sizeof(response.name.data));
    response.name.len = static_cast<std::uint8_t>(name_length);
    std::memcpy(response.name.data, configuration_.identity.name, name_length);

    std::uint8_t payload[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE]{};
    const std::uint32_t length =
        uavcan_protocol_GetNodeInfoResponse_encode(&response, payload);
    // 响应编码完成后先释放请求 payload，再让 libcanard 分配 TX 内存，降低固定
    // 3 KiB pool 的瞬时峰值；响应沿用请求 transfer-ID 和 priority。
    canardReleaseRxTransferPayload(instance(instance_storage_), &transfer);
    const auto *const descriptor = generated::find_subscription(
        generated::SubscriptionOwner::Node,
        generated::TransferKind::Request,
        generated::MessageRole::NodeInfo);
    if (descriptor == nullptr) {
        ++stats_.protocol_errors;
        return;
    }
    std::uint8_t response_transfer_id = transfer_id;
    const std::int16_t result = canardRequestOrRespond(
        instance(instance_storage_), source_node_id,
        descriptor->signature, descriptor->data_type_id,
        &response_transfer_id, priority,
        CanardResponse, payload, static_cast<std::uint16_t>(length));
    if (result > 0) ++stats_.node_info_responses;
    else if (result < 0) ++stats_.protocol_errors;
}

} // namespace dima::protocols::dronecan
