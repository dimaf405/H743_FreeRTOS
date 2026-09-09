#include "TransferIdTracker.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::protocols::dronecan {

TransferIdTracker::Disposition TransferIdTracker::observe(const Key &key, std::uint8_t transfer_id,
                         std::uint64_t timestamp_us) noexcept
{
    // DroneCAN v0 transfer-ID 只有 5 bit，先钳位到 [0,31]。超过 libcanard
    // 2 s 会话超时后，任何 ID 都可作为新会话首帧接受。
    transfer_id &= kTransferIdMask;
    State *state = find(key);
    if (state == nullptr) {
        state = allocate(key);
        accept(*state, transfer_id, timestamp_us);
        return Disposition::Accept;
    }

    if (timestamp_us < state->timestamp_us) {
        return Disposition::Stale;
    }
    if (timestamp_us - state->timestamp_us > kTransferTimeoutUs) {
        accept(*state, transfer_id, timestamp_us);
        return Disposition::Accept;
    }

    // 模 32 前向距离：distance=(new+32-old)&31。0 是重复；1..15 是前进；
    // 16..31 落在较远半环，视为乱序到达的旧传输。
    const std::uint8_t distance = static_cast<std::uint8_t>(
        (static_cast<std::uint32_t>(transfer_id) + 32U -
         state->transfer_id) & kTransferIdMask);
    if (distance == 0U) {
        return Disposition::Duplicate;
    }
    // The nearer half of the modulo-32 space is forward progress; the
    // farther half is an old transfer arriving out of order.
    if (distance > kMaximumForwardDistance) {
        return Disposition::Stale;
    }

    accept(*state, transfer_id, timestamp_us);
    return Disposition::Accept;
}

void TransferIdTracker::reset() noexcept
{ states_ = {}; }

bool TransferIdTracker::same_key(const Key &lhs, const Key &rhs) noexcept
{
    return lhs.data_type_id == rhs.data_type_id &&
           lhs.transfer_type == rhs.transfer_type &&
           lhs.source_node_id == rhs.source_node_id;
}

TransferIdTracker::State * TransferIdTracker::find(const Key &key) noexcept
{
    for (auto &state : states_) {
        if (state.valid && same_key(state.key, key)) {
            return &state;
        }
    }
    return nullptr;
}

TransferIdTracker::State * TransferIdTracker::allocate(const Key &key) noexcept
{
    // 优先空槽；无空槽时按 timestamp 淘汰最旧状态。容量不足只降低跨键
    // 去重窗口，不会拒绝一个此前未跟踪的合法消息流。
    State *selected = &states_[0];
    for (auto &state : states_) {
        if (!state.valid) {
            selected = &state;
            break;
        }
        if (state.timestamp_us < selected->timestamp_us) {
            selected = &state;
        }
    }
    *selected = State{};
    selected->key = key;
    selected->valid = true;
    return selected;
}

void TransferIdTracker::accept(State &state, std::uint8_t transfer_id,
                       std::uint64_t timestamp_us) noexcept
{
    state.transfer_id = transfer_id & kTransferIdMask;
    state.timestamp_us = timestamp_us;
}

} // namespace dima::protocols::dronecan
