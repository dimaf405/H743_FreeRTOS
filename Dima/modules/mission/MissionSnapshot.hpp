#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::modules::mission {
struct MissionPlan;

namespace snapshot {

struct Identity {
    std::uint64_t generation{0U};
    std::uint32_t crc{0U};
};

// 文件封装只承载存储代次和持久入口；航点载荷仍由正式 MAVLink codec 生成。
constexpr std::size_t kOverhead = 24U;

int encode(const MissionPlan &plan, std::uint64_t generation,
           std::uint8_t *destination, std::size_t capacity,
           std::size_t &size, std::uint32_t &mission_id,
           Identity &identity) noexcept;
int decode(const std::uint8_t *data, std::size_t size,
           MissionPlan &plan, Identity &identity) noexcept;
bool same_identity(const Identity &left, const Identity &right) noexcept;

} // namespace snapshot
} // namespace dima::modules::mission
