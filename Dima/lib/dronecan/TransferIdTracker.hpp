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
                         std::uint64_t timestamp_us) noexcept;

    void reset() noexcept;

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

    static bool same_key(const Key &lhs, const Key &rhs) noexcept;

    State *find(const Key &key) noexcept;

    State *allocate(const Key &key) noexcept;

    static void accept(State &state, std::uint8_t transfer_id,
                       std::uint64_t timestamp_us) noexcept;

    std::array<State, kStateCapacity> states_{};
};

} // namespace dima::protocols::dronecan
