/****************************************************************************
 * Centralized DroneCAN v0 allocation behavior follows PX4 v1.17.0's pinned
 * libuavcan implementation (MIT): allocation_request_manager.hpp,
 * node_id_selector.hpp, centralized/server.hpp, and node_discoverer.hpp.
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 ****************************************************************************/

#include "DroneCanNode.hpp"

#include <canard.h>
#include <dronecan_msgs.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace dima::protocols::dronecan {
namespace {

// 协议持久化镜像 ABI 固定为 little-endian，不能依赖 MCU 原生对齐或结构体 padding。
CanardInstance *instance(void *storage) noexcept
{
    return static_cast<CanardInstance *>(storage);
}

void write_u16(std::uint8_t *destination, std::uint16_t value) noexcept
{
    destination[0] = static_cast<std::uint8_t>(value);
    destination[1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32(std::uint8_t *destination, std::uint32_t value) noexcept
{
    destination[0] = static_cast<std::uint8_t>(value);
    destination[1] = static_cast<std::uint8_t>(value >> 8U);
    destination[2] = static_cast<std::uint8_t>(value >> 16U);
    destination[3] = static_cast<std::uint8_t>(value >> 24U);
}

std::uint16_t read_u16(const std::uint8_t *source) noexcept
{
    return static_cast<std::uint16_t>(source[0]) |
           (static_cast<std::uint16_t>(source[1]) << 8U);
}

std::uint32_t read_u32(const std::uint8_t *source) noexcept
{
    return static_cast<std::uint32_t>(source[0]) |
           (static_cast<std::uint32_t>(source[1]) << 8U) |
           (static_cast<std::uint32_t>(source[2]) << 16U) |
           (static_cast<std::uint32_t>(source[3]) << 24U);
}

} // namespace

std::uint32_t DroneCanNode::unique_id_fingerprint(
    const std::uint8_t *unique_id) noexcept
{
    if (unique_id == nullptr) return 0U;
    // 日志只公开 32-bit FNV-1a 指纹：hash=(hash XOR byte)*16777619，
    // 不把完整 128-bit 硬件 UID 写入遥测日志。
    std::uint32_t hash = 2166136261U;
    for (std::size_t index = 0U; index < generated::kUniqueIdBytes;
         ++index) {
        hash ^= unique_id[index];
        hash *= 16777619U;
    }
    return hash;
}

void DroneCanNode::emit_allocation_event(
    AllocationEventKind kind, std::uint8_t node_id,
    std::uint8_t preferred_node_id, const std::uint8_t *unique_id,
    std::int32_t error) noexcept
{
    if (callbacks_.allocation_event == nullptr) return;
    AllocationEvent event{};
    event.kind = kind;
    event.node_id = node_id;
    event.preferred_node_id = preferred_node_id;
    event.unique_id_fingerprint = unique_id_fingerprint(unique_id);
    event.error = error;
    callbacks_.allocation_event(callbacks_.context, event);
}

void DroneCanNode::emit_bounded_allocation_error(
    AllocationEventKind kind, std::uint64_t now_us,
    std::uint8_t node_id, std::int32_t error) noexcept
{
    // 协议/存储噪声按生成合同给出的间隔限频，正常成功事件不受此门限影响。
    if (now_us < next_allocation_error_event_us_) return;
    next_allocation_error_event_us_ =
        now_us + generated::kErrorLogIntervalUs;
    emit_allocation_event(kind, node_id, 0U, nullptr, error);
}

void DroneCanNode::reset_allocation() noexcept
{
    // stop/restart 时先取消可能仍引用 allocation_storage_image_ 的异步保存，再清除
    // RAM 索引、分片前缀、待响应和发现状态，防止后端回调访问重用后的缓冲。
    if (allocation_storage_active_ &&
        configuration_.allocation_storage.cancel_save != nullptr) {
        configuration_.allocation_storage.cancel_save(
            configuration_.allocation_storage.context);
    }
    std::memset(get_node_info_transfer_ids_, 0,
                sizeof(get_node_info_transfer_ids_));
    std::memset(allocation_prefix_, 0, sizeof(allocation_prefix_));
    std::memset(node_states_, 0, sizeof(node_states_));
    std::memset(node_unique_ids_, 0, sizeof(node_unique_ids_));
    std::memset(discovery_attempts_, 0, sizeof(discovery_attempts_));
    std::memset(discovery_uptime_, 0, sizeof(discovery_uptime_));
    std::memset(observed_nodes_, 0, sizeof(observed_nodes_));
    std::memset(discovery_pending_, 0, sizeof(discovery_pending_));
    std::memset(pending_request_unique_id_, 0,
                sizeof(pending_request_unique_id_));
    std::memset(pending_discovery_unique_id_, 0,
                sizeof(pending_discovery_unique_id_));
    std::memset(allocation_storage_image_, 0,
                sizeof(allocation_storage_image_));
    allocation_prefix_length_ = 0U;
    pending_request_preferred_node_id_ = 0U;
    pending_discovery_node_id_ = 0U;
    discovery_query_node_id_ = 0U;
    discovery_query_deadline_us_ = 0U;
    next_discovery_poll_us_ = 0U;
    last_allocation_message_us_ = 0U;
    storage_retry_after_us_ = 0U;
    next_allocation_error_event_us_ = 0U;
    last_allocation_storage_error_ = 0;
    pending_commit_ = {};
    pending_allocation_response_ = {};
    allocation_transfer_id_ = 0U;
    automatic_allocation_ = false;
    allocation_ready_ = false;
    allocation_storage_active_ = false;
    allocation_first_request_reported_ = false;
    pending_request_valid_ = false;
    pending_discovery_valid_ = false;
}

bool DroneCanNode::load_allocation_image() noexcept
{
    std::memset(allocation_storage_image_, 0,
                sizeof(allocation_storage_image_));
    const int result = configuration_.allocation_storage.load(
        configuration_.allocation_storage.context,
        allocation_storage_image_, sizeof(allocation_storage_image_));
    // -ENOENT 表示首次启动的合法空表；其他错误或 ABI magic/version/size/footer
    // 不匹配均 fail-closed，不能带着不可信分配表启动服务器。
    if (result == -ENOENT) {
        return true;
    }
    if (result != 0) {
        last_allocation_storage_error_ = result;
        return false;
    }
    if (read_u32(&allocation_storage_image_[0]) !=
            generated::kAllocationStorageMagic ||
        read_u16(&allocation_storage_image_[4]) !=
            generated::kAllocationStorageVersion ||
        read_u16(&allocation_storage_image_[6]) !=
            generated::kAllocationStorageImageBytes ||
        read_u32(&allocation_storage_image_[
            generated::kAllocationStorageFooterOffset]) !=
            generated::kAllocationStorageFooter) {
        last_allocation_storage_error_ = -EINVAL;
        return false;
    }

    // 每个条目只允许 Empty/OccupiedWithoutUid/KnownUid；KnownUid 还要求全表
    // 唯一，避免同一硬件身份被持久化到两个节点 ID。
    for (std::uint16_t node_id = 1U;
         node_id <= generated::kMaximumNodeId; ++node_id) {
        const std::size_t offset =
            generated::kAllocationStorageHeaderBytes +
            (static_cast<std::size_t>(node_id) - 1U) *
                generated::kAllocationStorageEntryBytes;
        const std::uint8_t state = allocation_storage_image_[offset];
        if (state != generated::kAllocationStateEmpty &&
            state != generated::kAllocationStateOccupiedWithoutUid &&
            state != generated::kAllocationStateKnownUid) {
            last_allocation_storage_error_ = -EINVAL;
            return false;
        }
        node_states_[node_id] = state;
        if (state == generated::kAllocationStateKnownUid) {
            std::copy_n(&allocation_storage_image_[offset + 1U],
                        generated::kUniqueIdBytes,
                        node_unique_ids_[node_id]);
            for (std::uint16_t previous = 1U; previous < node_id;
                 ++previous) {
                if (node_states_[previous] ==
                        generated::kAllocationStateKnownUid &&
                    std::memcmp(node_unique_ids_[previous],
                                node_unique_ids_[node_id],
                                generated::kUniqueIdBytes) == 0) {
                    last_allocation_storage_error_ = -EINVAL;
                    return false;
                }
            }
        }
    }
    return true;
}

bool DroneCanNode::initialize_allocation(std::uint64_t now_us) noexcept
{
    // 分配服务器自身也必须先占用并持久化 local_node_id。若同一 UID 已绑定其他
    // ID，或本地 ID 已被不同 UID 占用，直接报告冲突并拒绝启动。
    if (!load_allocation_image()) {
        emit_allocation_event(AllocationEventKind::StorageFailure, 0U, 0U,
                              nullptr, last_allocation_storage_error_);
        return false;
    }
    observed_nodes_[configuration_.local_node_id] = true;
    const std::uint8_t stored_node = node_id_for_unique_id(
        configuration_.identity.unique_id);
    if (stored_node != 0U &&
        stored_node != configuration_.local_node_id) {
        emit_allocation_event(AllocationEventKind::NodeConflict,
                              stored_node, configuration_.local_node_id,
                              configuration_.identity.unique_id, -EINVAL);
        return false;
    }
    if (node_states_[configuration_.local_node_id] ==
            generated::kAllocationStateKnownUid &&
        std::memcmp(node_unique_ids_[configuration_.local_node_id],
                    configuration_.identity.unique_id,
                    generated::kUniqueIdBytes) != 0) {
        emit_allocation_event(AllocationEventKind::NodeConflict,
                              configuration_.local_node_id,
                              configuration_.local_node_id,
                              configuration_.identity.unique_id, -EINVAL);
        return false;
    }
    if (stored_node == configuration_.local_node_id) {
        allocation_ready_ = true;
        return true;
    }
    return stage_commit(PendingCommitKind::AllocatorIdentity,
                        configuration_.local_node_id,
                        generated::kAllocationStateKnownUid,
                        configuration_.identity.unique_id, now_us);
}

void DroneCanNode::encode_allocation_image() noexcept
{
    // 生成完整快照时把 pending_commit 覆盖到已提交表上，但尚不修改 node_states_；
    // footer 最后编码，后端再负责其事务/校验，形成“保存成功后发布”的语义。
    std::memset(allocation_storage_image_, 0,
                sizeof(allocation_storage_image_));
    write_u32(&allocation_storage_image_[0],
              generated::kAllocationStorageMagic);
    write_u16(&allocation_storage_image_[4],
              generated::kAllocationStorageVersion);
    write_u16(&allocation_storage_image_[6],
              static_cast<std::uint16_t>(
                  generated::kAllocationStorageImageBytes));
    for (std::uint16_t node_id = 1U;
         node_id <= generated::kMaximumNodeId; ++node_id) {
        std::uint8_t state = node_states_[node_id];
        const std::uint8_t *unique_id = node_unique_ids_[node_id];
        if (pending_commit_.kind != PendingCommitKind::None &&
            pending_commit_.node_id == node_id) {
            state = pending_commit_.state;
            unique_id = pending_commit_.unique_id;
        }
        const std::size_t offset =
            generated::kAllocationStorageHeaderBytes +
            (static_cast<std::size_t>(node_id) - 1U) *
                generated::kAllocationStorageEntryBytes;
        allocation_storage_image_[offset] = state;
        if (state == generated::kAllocationStateKnownUid) {
            std::copy_n(unique_id, generated::kUniqueIdBytes,
                        &allocation_storage_image_[offset + 1U]);
        }
    }
    write_u32(&allocation_storage_image_[
                  generated::kAllocationStorageFooterOffset],
              generated::kAllocationStorageFooter);
}

bool DroneCanNode::stage_commit(PendingCommitKind kind,
                                std::uint8_t node_id,
                                std::uint8_t state,
                                const std::uint8_t *unique_id,
                                std::uint64_t now_us) noexcept
{
    // 同时最多一个持久化事务。stage 只构造 pending 镜像并标记可保存，真正 RAM
    // 状态和网络响应都要等 service_allocation_storage 确认保存成功。
    if (pending_commit_.kind != PendingCommitKind::None ||
        node_id == 0U || node_id > generated::kMaximumNodeId ||
        (state == generated::kAllocationStateKnownUid &&
         unique_id == nullptr)) {
        return false;
    }
    pending_commit_ = {};
    pending_commit_.kind = kind;
    pending_commit_.node_id = node_id;
    pending_commit_.state = state;
    if (state == generated::kAllocationStateKnownUid) {
        std::copy_n(unique_id, generated::kUniqueIdBytes,
                    pending_commit_.unique_id);
    }
    encode_allocation_image();
    storage_retry_after_us_ = now_us;
    return true;
}

std::uint8_t DroneCanNode::node_id_for_unique_id(
    const std::uint8_t *unique_id) const noexcept
{
    if (unique_id == nullptr) return 0U;
    if (pending_commit_.kind != PendingCommitKind::None &&
        pending_commit_.state == generated::kAllocationStateKnownUid &&
        std::memcmp(pending_commit_.unique_id, unique_id,
                    generated::kUniqueIdBytes) == 0) {
        return pending_commit_.node_id;
    }
    for (std::uint16_t node_id = 1U;
         node_id <= generated::kMaximumNodeId; ++node_id) {
        if (node_states_[node_id] ==
                generated::kAllocationStateKnownUid &&
            std::memcmp(node_unique_ids_[node_id], unique_id,
                        generated::kUniqueIdBytes) == 0) {
            return static_cast<std::uint8_t>(node_id);
        }
    }
    return 0U;
}

bool DroneCanNode::node_id_occupied(std::uint8_t node_id) const noexcept
{
    if (node_id == 0U || node_id > generated::kMaximumNodeId) return true;
    return node_id == configuration_.local_node_id ||
           node_states_[node_id] != generated::kAllocationStateEmpty ||
           observed_nodes_[node_id] ||
           (pending_commit_.kind != PendingCommitKind::None &&
            pending_commit_.node_id == node_id);
}

std::uint8_t DroneCanNode::select_node_id(std::uint8_t preferred) const noexcept
{
    // 有效 preferred 从该 ID 向上查找，再向下回绕；无效 preferred 从最大 ID
    // 向下选择。local/stored/observed/pending 任一占用都不可复用。
    const std::uint16_t first =
        preferred >= 1U && preferred <= generated::kMaximumNodeId
            ? preferred
            : generated::kMaximumNodeId;
    for (std::uint16_t candidate = first;
         candidate <= generated::kMaximumNodeId; ++candidate) {
        if (!node_id_occupied(static_cast<std::uint8_t>(candidate))) {
            return static_cast<std::uint8_t>(candidate);
        }
    }
    for (std::uint16_t candidate = first; candidate > 0U; --candidate) {
        if (!node_id_occupied(static_cast<std::uint8_t>(candidate))) {
            return static_cast<std::uint8_t>(candidate);
        }
    }
    return 0U;
}

void DroneCanNode::queue_allocation_response(
    std::uint8_t node_id, const std::uint8_t *unique_id,
    std::size_t unique_id_length) noexcept
{
    if (unique_id == nullptr ||
        unique_id_length > generated::kUniqueIdBytes) {
        ++stats_.protocol_errors;
        return;
    }
    pending_allocation_response_ = {};
    pending_allocation_response_.valid = true;
    pending_allocation_response_.node_id = node_id;
    pending_allocation_response_.unique_id_length =
        static_cast<std::uint8_t>(unique_id_length);
    std::copy_n(unique_id, unique_id_length,
                pending_allocation_response_.unique_id);
}

void DroneCanNode::complete_allocation_request(
    const std::uint8_t *unique_id, std::uint8_t preferred_node_id,
    std::uint64_t now_us) noexcept
{
    // 已知 UID 直接复用原 ID；新 UID 必须先 stage 持久化。忙时只缓存一个后续
    // 请求，避免固定内存服务器被匿名节点请求洪泛拖入无界队列。
    if (pending_commit_.kind != PendingCommitKind::None) {
        if (pending_commit_.kind == PendingCommitKind::Allocation &&
            std::memcmp(pending_commit_.unique_id, unique_id,
                        generated::kUniqueIdBytes) == 0) {
            return;
        }
        if (!pending_request_valid_) {
            std::copy_n(unique_id, generated::kUniqueIdBytes,
                        pending_request_unique_id_);
            pending_request_preferred_node_id_ = preferred_node_id;
            pending_request_valid_ = true;
        }
        return;
    }
    const std::uint8_t existing = node_id_for_unique_id(unique_id);
    if (existing != 0U) {
        queue_allocation_response(existing, unique_id,
                                  generated::kUniqueIdBytes);
        return;
    }
    if (!allocation_ready_) {
        if (!pending_request_valid_) {
            std::copy_n(unique_id, generated::kUniqueIdBytes,
                        pending_request_unique_id_);
            pending_request_preferred_node_id_ = preferred_node_id;
            pending_request_valid_ = true;
        }
        return;
    }
    const std::uint8_t allocated = select_node_id(preferred_node_id);
    if (allocated == 0U) {
        emit_bounded_allocation_error(
            AllocationEventKind::AllocationExhausted, now_us, 0U, -ENOSPC);
        return;
    }
    if (!stage_commit(PendingCommitKind::Allocation, allocated,
                      generated::kAllocationStateKnownUid,
                      unique_id, now_us)) {
        if (!pending_request_valid_) {
            std::copy_n(unique_id, generated::kUniqueIdBytes,
                        pending_request_unique_id_);
            pending_request_preferred_node_id_ = preferred_node_id;
            pending_request_valid_ = true;
        }
    }
}

void DroneCanNode::handle_allocation(void *native_transfer) noexcept
{
    // DroneCAN v0 动态分配请求必须来自匿名节点；服务器串行处理一个分配/
    // 持久化/最终响应链，进行中的请求不会被另一匿名流覆盖。
    auto &transfer = *static_cast<CanardRxTransfer *>(native_transfer);
    if (transfer.source_node_id != CANARD_BROADCAST_NODE_ID) {
        return;
    }
    if (pending_commit_.kind == PendingCommitKind::Allocation ||
        pending_request_valid_ ||
        (pending_allocation_response_.valid &&
         pending_allocation_response_.node_id != 0U)) {
        return;
    }
    uavcan_protocol_dynamic_node_id_Allocation message{};
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(
            &transfer, &message)) {
        ++stats_.allocation_malformed;
        emit_bounded_allocation_error(
            AllocationEventKind::MalformedRequest,
            transfer.timestamp_usec, 0U, -EINVAL);
        return;
    }
    const std::uint64_t timestamp_us = transfer.timestamp_usec;
    if (allocation_prefix_length_ != 0U &&
        timestamp_us > last_allocation_message_us_ &&
        timestamp_us - last_allocation_message_us_ >
            generated::kAllocationFollowupTimeoutUs) {
        allocation_prefix_length_ = 0U;
        pending_allocation_response_ = {};
        ++stats_.allocation_timeouts;
        emit_bounded_allocation_error(
            AllocationEventKind::FollowupTimeout, timestamp_us, 0U,
            -ETIMEDOUT);
    }

    // Classic CAN 的匿名请求每帧最多携带 6 B UID，但后续片实际长度由节点根据
    // 分配器已确认的累计前缀决定，并不要求固定为 6+6+4。更重要的是，节点看到
    // 其他 Allocation 消息后会按协议放弃当前 follow-up 并重发首片；所以 first
    // 是合法的重新同步边界，即使服务器正等待后续片也必须清空旧前缀后接收。
    const std::uint8_t length = message.unique_id.len;
    if (length == 0U ||
        length > generated::kAllocationRequestFragmentBytes) {
        ++stats_.allocation_malformed;
        emit_bounded_allocation_error(
            AllocationEventKind::MalformedRequest, timestamp_us, 0U,
            -EINVAL);
        return;
    }

    if (message.first_part_of_unique_id) {
        // 未发送的 node_id=0 前缀确认属于旧分片链；首片重启时必须一起撤销，
        // 否则节点可能先收到旧 ACK，再按错误偏移继续发送 UID。
        std::memset(allocation_prefix_, 0, sizeof(allocation_prefix_));
        allocation_prefix_length_ = 0U;
        pending_allocation_response_ = {};
    } else if (allocation_prefix_length_ == 0U) {
        // 丢包或另一匿名节点的后续片在本分配器没有对应前缀时无法归属；这是协议
        // 竞争下的可恢复事件，静默丢弃并等待下一次 first，不能误报 CAN 故障。
        return;
    }

    if (length > generated::kUniqueIdBytes - allocation_prefix_length_) {
        allocation_prefix_length_ = 0U;
        pending_allocation_response_ = {};
        ++stats_.allocation_malformed;
        emit_bounded_allocation_error(
            AllocationEventKind::MalformedRequest, timestamp_us, 0U,
            -EINVAL);
        return;
    }
    std::copy_n(message.unique_id.data, length,
                &allocation_prefix_[allocation_prefix_length_]);
    allocation_prefix_length_ = static_cast<std::uint8_t>(
        allocation_prefix_length_ + length);
    last_allocation_message_us_ = timestamp_us;
    ++stats_.allocation_requests;
    if (!allocation_first_request_reported_) {
        allocation_first_request_reported_ = true;
        emit_allocation_event(
            AllocationEventKind::FirstAnonymousRequest, 0U,
            message.node_id, allocation_prefix_, 0);
    }
    // 非最终分片用 node_id=0 回显已确认前缀，驱动匿名节点继续发送下一段；
    // 完整 UID 才进入“查旧绑定/选择 ID/持久化”流程。
    if (allocation_prefix_length_ == generated::kUniqueIdBytes) {
        std::uint8_t unique_id[generated::kUniqueIdBytes]{};
        std::copy_n(allocation_prefix_, generated::kUniqueIdBytes,
                    unique_id);
        allocation_prefix_length_ = 0U;
        complete_allocation_request(unique_id, message.node_id,
                                    timestamp_us);
    } else {
        queue_allocation_response(0U, allocation_prefix_,
                                  allocation_prefix_length_);
    }
}

void DroneCanNode::handle_node_status(void *native_transfer) noexcept
{
    // 观察到持久化表之外的静态节点时启动 GetNodeInfo 发现；若 uptime 下降说明
    // 节点重启，清零之前的查询尝试，允许重新获取身份。
    auto &transfer = *static_cast<CanardRxTransfer *>(native_transfer);
    const std::uint8_t node_id = transfer.source_node_id;
    if (node_id == 0U || node_id > generated::kMaximumNodeId ||
        node_id == configuration_.local_node_id) {
        return;
    }
    uavcan_protocol_NodeStatus status{};
    if (uavcan_protocol_NodeStatus_decode(&transfer, &status)) {
        ++stats_.protocol_errors;
        return;
    }
    observed_nodes_[node_id] = true;
    if (node_states_[node_id] != generated::kAllocationStateEmpty) {
        discovery_pending_[node_id] = false;
        return;
    }
    if (discovery_pending_[node_id] &&
        status.uptime_sec < discovery_uptime_[node_id]) {
        discovery_attempts_[node_id] = 0U;
    }
    discovery_uptime_[node_id] = status.uptime_sec;
    discovery_pending_[node_id] = true;
}

void DroneCanNode::handle_node_info_response(void *native_transfer) noexcept
{
    // 同一 UID 已绑定其他 node-ID 属于冲突；当前 ID 仍标记为“占用但 UID 未知”，
    // 绝不把它重新分配。无冲突响应则待持久化为 KnownUid。
    auto &transfer = *static_cast<CanardRxTransfer *>(native_transfer);
    const std::uint8_t node_id = transfer.source_node_id;
    if (node_id == 0U || node_id != discovery_query_node_id_) return;
    uavcan_protocol_GetNodeInfoResponse response{};
    if (uavcan_protocol_GetNodeInfoResponse_decode(&transfer, &response)) {
        ++stats_.protocol_errors;
        return;
    }
    discovery_query_node_id_ = 0U;
    discovery_query_deadline_us_ = 0U;
    discovery_pending_[node_id] = false;
    const std::uint8_t existing = node_id_for_unique_id(
        response.hardware_version.unique_id);
    if (existing != 0U && existing != node_id) {
        emit_bounded_allocation_error(
            AllocationEventKind::NodeConflict, transfer.timestamp_usec,
            node_id, -EEXIST);
        const bool staged =
            pending_commit_.kind == PendingCommitKind::None &&
            stage_commit(PendingCommitKind::DiscoveredNode, node_id,
                         generated::kAllocationStateOccupiedWithoutUid,
                         nullptr, transfer.timestamp_usec);
        if (!staged) {
            discovery_pending_[node_id] = true;
        }
        return;
    }
    pending_discovery_node_id_ = node_id;
    std::copy_n(response.hardware_version.unique_id,
                generated::kUniqueIdBytes,
                pending_discovery_unique_id_);
    pending_discovery_valid_ = true;
}

bool DroneCanNode::send_get_node_info_request(
    std::uint8_t node_id) noexcept
{
    const auto *const descriptor = generated::find_subscription(
        generated::SubscriptionOwner::Protocol,
        generated::TransferKind::Request,
        generated::MessageRole::GetNodeInfo);
    if (descriptor == nullptr || node_id == 0U ||
        node_id > generated::kMaximumNodeId) {
        ++stats_.protocol_errors;
        return false;
    }
    uavcan_protocol_GetNodeInfoRequest request{};
    std::uint8_t payload[1]{};
    const std::uint32_t length =
        uavcan_protocol_GetNodeInfoRequest_encode(&request, payload);
    const std::int16_t result = canardRequestOrRespond(
        instance(instance_storage_), node_id, descriptor->signature,
        descriptor->data_type_id,
        &get_node_info_transfer_ids_[node_id],
        generated::kAllocationTransferPriority, CanardRequest,
        payload, static_cast<std::uint16_t>(length));
    if (result > 0) return true;
    if (result < 0) ++stats_.protocol_errors;
    return false;
}

void DroneCanNode::apply_completed_commit(std::uint64_t) noexcept
{
    // 这是 pending 状态对运行时索引的唯一发布点：仅后端确认保存成功后调用。
    // Allocation 类型随后排队最终响应，保证断电后不会遗失已经对外承诺的绑定。
    const PendingCommit completed = pending_commit_;
    node_states_[completed.node_id] = completed.state;
    std::memset(node_unique_ids_[completed.node_id], 0,
                generated::kUniqueIdBytes);
    if (completed.state == generated::kAllocationStateKnownUid) {
        std::copy_n(completed.unique_id, generated::kUniqueIdBytes,
                    node_unique_ids_[completed.node_id]);
    }
    observed_nodes_[completed.node_id] = true;
    pending_commit_ = {};
    if (completed.kind == PendingCommitKind::AllocatorIdentity) {
        allocation_ready_ = true;
        emit_allocation_event(AllocationEventKind::ServerReady,
                              configuration_.local_node_id, 0U,
                              configuration_.identity.unique_id, 0);
    } else if (completed.kind == PendingCommitKind::Allocation) {
        queue_allocation_response(completed.node_id,
                                  completed.unique_id,
                                  generated::kUniqueIdBytes);
    } else if (completed.kind == PendingCommitKind::DiscoveredNode) {
        ++stats_.discovered_nodes;
    }
}

void DroneCanNode::service_allocation_storage(
    std::uint64_t now_us) noexcept
{
    if (pending_commit_.kind == PendingCommitKind::None ||
        now_us < storage_retry_after_us_) {
        return;
    }
    // begin_save/continue_save 是非阻塞两阶段接口；-EAGAIN/-EBUSY 保持事务等待，
    // 0 表示当前阶段成功。失败先 cancel，再按生成合同的周期重试同一镜像。
    int result = 0;
    if (!allocation_storage_active_) {
        result = configuration_.allocation_storage.begin_save(
            configuration_.allocation_storage.context,
            allocation_storage_image_, sizeof(allocation_storage_image_));
        if (result == 0) {
            allocation_storage_active_ = true;
            return;
        }
    } else {
        result = configuration_.allocation_storage.continue_save(
            configuration_.allocation_storage.context);
        if (result == -EAGAIN || result == -EBUSY) return;
        if (result == 0) {
            allocation_storage_active_ = false;
            apply_completed_commit(now_us);
            return;
        }
        configuration_.allocation_storage.cancel_save(
            configuration_.allocation_storage.context);
        allocation_storage_active_ = false;
    }
    if (result == -EAGAIN || result == -EBUSY) return;
    ++stats_.allocation_storage_failures;
    last_allocation_storage_error_ =
        result != 0 ? result : -EIO;
    emit_bounded_allocation_error(
        AllocationEventKind::StorageFailure, now_us,
        pending_commit_.node_id, last_allocation_storage_error_);
    storage_retry_after_us_ = now_us + generated::kPersistenceRetryUs;
}

void DroneCanNode::service_pending_allocation_request(
    std::uint64_t now_us) noexcept
{
    if (!pending_request_valid_ || !allocation_ready_ ||
        pending_commit_.kind != PendingCommitKind::None) {
        return;
    }
    std::uint8_t unique_id[generated::kUniqueIdBytes]{};
    std::copy_n(pending_request_unique_id_, generated::kUniqueIdBytes,
                unique_id);
    const std::uint8_t preferred =
        pending_request_preferred_node_id_;
    pending_request_valid_ = false;
    complete_allocation_request(unique_id, preferred, now_us);
}

void DroneCanNode::service_discovery(std::uint64_t now_us) noexcept
{
    // 分配服务器只串行发出一个 GetNodeInfo 请求。超时累计到最大次数后，将该
    // node-ID 持久化为 OccupiedWithoutUid，宁可少分配也不碰撞在线静态节点。
    if (!allocation_ready_) return;
    if (pending_discovery_valid_ &&
        pending_commit_.kind == PendingCommitKind::None &&
        !pending_request_valid_) {
        if (stage_commit(PendingCommitKind::DiscoveredNode,
                         pending_discovery_node_id_,
                         generated::kAllocationStateKnownUid,
                         pending_discovery_unique_id_, now_us)) {
            pending_discovery_valid_ = false;
        }
        return;
    }
    if (discovery_query_node_id_ != 0U) {
        if (now_us < discovery_query_deadline_us_) return;
        const std::uint8_t node_id = discovery_query_node_id_;
        discovery_query_node_id_ = 0U;
        discovery_query_deadline_us_ = 0U;
        if (discovery_attempts_[node_id] < UINT8_MAX) {
            ++discovery_attempts_[node_id];
        }
        next_discovery_poll_us_ = now_us;
    }
    if (pending_commit_.kind != PendingCommitKind::None ||
        pending_request_valid_ || pending_discovery_valid_ ||
        now_us < next_discovery_poll_us_) {
        return;
    }
    for (std::uint16_t candidate = 1U;
         candidate <= generated::kMaximumNodeId; ++candidate) {
        const std::uint8_t node_id = static_cast<std::uint8_t>(candidate);
        if (!discovery_pending_[node_id]) continue;
        if (node_states_[node_id] != generated::kAllocationStateEmpty) {
            discovery_pending_[node_id] = false;
            continue;
        }
        if (discovery_attempts_[node_id] >=
            generated::kDiscoveryMaximumAttempts) {
            if (stage_commit(PendingCommitKind::DiscoveredNode, node_id,
                             generated::kAllocationStateOccupiedWithoutUid,
                             nullptr, now_us)) {
                discovery_pending_[node_id] = false;
            }
            return;
        }
        if (send_get_node_info_request(node_id)) {
            discovery_query_node_id_ = node_id;
            discovery_query_deadline_us_ =
                now_us + generated::kDiscoveryResponseTimeoutUs;
        }
        next_discovery_poll_us_ =
            now_us + generated::kDiscoveryPollIntervalUs;
        return;
    }
}

void DroneCanNode::service_allocation_response(
    std::uint64_t now_us) noexcept
{
    if (!pending_allocation_response_.valid) return;
    const auto *const descriptor = generated::find_subscription(
        generated::SubscriptionOwner::Protocol,
        generated::TransferKind::Broadcast,
        generated::MessageRole::Allocation);
    if (descriptor == nullptr) {
        ++stats_.protocol_errors;
        pending_allocation_response_ = {};
        return;
    }
    uavcan_protocol_dynamic_node_id_Allocation response{};
    // node_id=0 是分片前缀确认；非零是已持久化完成的最终分配结果。只有底层
    // 成功入队后才清 pending，并为最终结果累计 success/发送事件。
    response.node_id = pending_allocation_response_.node_id;
    response.first_part_of_unique_id = false;
    response.unique_id.len =
        pending_allocation_response_.unique_id_length;
    std::copy_n(pending_allocation_response_.unique_id,
                response.unique_id.len, response.unique_id.data);
    std::uint8_t payload[
        UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE]{};
    const std::uint32_t length =
        uavcan_protocol_dynamic_node_id_Allocation_encode(
            &response, payload);
    const std::int16_t result = canardBroadcast(
        instance(instance_storage_), descriptor->signature,
        descriptor->data_type_id, &allocation_transfer_id_,
        generated::kAllocationTransferPriority, payload,
        static_cast<std::uint16_t>(length));
    if (result <= 0) {
        if (result < 0) ++stats_.protocol_errors;
        (void)now_us;
        return;
    }
    if (pending_allocation_response_.node_id != 0U) {
        ++stats_.allocation_successes;
        emit_allocation_event(
            AllocationEventKind::AllocationSucceeded,
            pending_allocation_response_.node_id, 0U,
            pending_allocation_response_.unique_id, 0);
    }
    pending_allocation_response_ = {};
}

void DroneCanNode::service_allocation(std::uint64_t now_us) noexcept
{
    // 固定顺序维持因果关系：先完成持久化，再处理排队请求/发现，最后发送响应。
    if (!automatic_allocation_) return;
    service_allocation_storage(now_us);
    service_pending_allocation_request(now_us);
    service_discovery(now_us);
    service_allocation_response(now_us);
}

} // namespace dima::protocols::dronecan
