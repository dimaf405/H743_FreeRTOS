#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dima::protocols::dronecan {

// Protocol-side guard for the vendored libcanard single-frame duplicate path.
// 为 vendored libcanard 的单帧路径补充业务侧去重。键由数据类型、传输类型和
// 源节点组成；固定 4 槽，满时淘汰最久未更新项，不使用动态内存。
class TransferIdTracker final {
public:
    enum class Disposition : std::uint8_t {
        Accept,
        Duplicate,
        Stale,
    };

    struct Key {
        std::uint16_t data_type_id{0U};
        std::uint8_t transfer_type{0U};
        std::uint8_t source_node_id{0U};
    };

    // Matches libcanard's Classic CAN TRANSFER_TIMEOUT_USEC contract.
    static constexpr std::uint64_t kTransferTimeoutUs{2000000ULL};

    Disposition observe(const Key &key, std::uint8_t transfer_id,
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

    void reset() noexcept { states_ = {}; }

private:
    static constexpr std::size_t kStateCapacity{4U};
    static constexpr std::uint8_t kTransferIdMask{31U};
    static constexpr std::uint8_t kMaximumForwardDistance{15U};

    struct State {
        Key key{};
        std::uint64_t timestamp_us{0U};
        std::uint8_t transfer_id{0U};
        bool valid{false};
    };

    static bool same_key(const Key &lhs, const Key &rhs) noexcept
    {
        return lhs.data_type_id == rhs.data_type_id &&
               lhs.transfer_type == rhs.transfer_type &&
               lhs.source_node_id == rhs.source_node_id;
    }

    State *find(const Key &key) noexcept
    {
        for (auto &state : states_) {
            if (state.valid && same_key(state.key, key)) {
                return &state;
            }
        }
        return nullptr;
    }

    State *allocate(const Key &key) noexcept
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

    static void accept(State &state, std::uint8_t transfer_id,
                       std::uint64_t timestamp_us) noexcept
    {
        state.transfer_id = transfer_id & kTransferIdMask;
        state.timestamp_us = timestamp_us;
    }

    std::array<State, kStateCapacity> states_{};
};

} // namespace dima::protocols::dronecan
