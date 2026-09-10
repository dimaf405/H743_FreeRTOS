#include "MissionSnapshot.hpp"

#include "MissionCodec.hpp"
#include "parameters/Crc32.hpp"

#include <cerrno>

namespace dima::modules::mission::snapshot {
namespace {

constexpr std::uint32_t kSnapshotMagic = 0x32534d44U; // DMS2
constexpr std::size_t kHeaderSize = kOverhead - sizeof(std::uint32_t);

void put_u32(std::uint8_t *data, std::uint32_t value) noexcept
{
    // 文件固定小端，与编译器 padding 和主机 ABI 无关。
    for (unsigned index = 0U; index < 4U; ++index) {
        data[index] = static_cast<std::uint8_t>(value >> (index * 8U));
    }
}

std::uint32_t get_u32(const std::uint8_t *data) noexcept
{
    std::uint32_t value = 0U;
    for (unsigned index = 0U; index < 4U; ++index) {
        value |= static_cast<std::uint32_t>(data[index]) << (index * 8U);
    }
    return value;
}

void put_generation(std::uint8_t *data, std::uint64_t value) noexcept
{
    put_u32(data, static_cast<std::uint32_t>(value));
    put_u32(data + 4U, static_cast<std::uint32_t>(value >> 32U));
}

std::uint64_t get_generation(const std::uint8_t *data) noexcept
{
    return static_cast<std::uint64_t>(get_u32(data)) |
           (static_cast<std::uint64_t>(get_u32(data + 4U)) << 32U);
}

} // namespace

bool same_identity(const Identity &left, const Identity &right) noexcept
{
    return left.generation != 0U && left.generation == right.generation &&
           left.crc == right.crc;
}

int encode(const MissionPlan &plan, std::uint64_t generation,
           std::uint8_t *destination, std::size_t capacity,
           std::size_t &size, std::uint32_t &mission_id,
           Identity &identity) noexcept
{
    size = 0U;
    identity = {};
    if (destination == nullptr || generation == 0U ||
        capacity < codec::kFileCapacity + kOverhead ||
        (plan.count == 0U ? plan.current != 0U : plan.current >= plan.count)) {
        return -EINVAL;
    }
    std::size_t payload_size = 0U;
    const int result = codec::encode(
        plan, destination + kHeaderSize, capacity - kOverhead,
        payload_size, mission_id);
    if (result != 0) {
        return result;
    }

    // CRC 覆盖版本、代次、持久 current、长度及完整标准 MAVLink 载荷。
    // current 与 AUTO 的运行进度分离，不改变原有任务内容 ID 的计算。
    put_u32(destination, kSnapshotMagic);
    put_generation(destination + 4U, generation);
    put_u32(destination + 12U, plan.current);
    put_u32(destination + 16U, static_cast<std::uint32_t>(payload_size));
    const std::size_t protected_size = kHeaderSize + payload_size;
    identity = {generation, dima::parameters::crc32(destination, protected_size)};
    put_u32(destination + protected_size, identity.crc);
    size = protected_size + sizeof(std::uint32_t);
    return 0;
}

int decode(const std::uint8_t *data, std::size_t size,
           MissionPlan &plan, Identity &identity) noexcept
{
    plan = {};
    identity = {};
    if (data == nullptr || size <= kOverhead ||
        size > codec::kFileCapacity + kOverhead ||
        get_u32(data) != kSnapshotMagic ||
        get_generation(data + 4U) == 0U ||
        get_u32(data + 16U) != size - kOverhead) {
        return -EBADMSG;
    }
    const std::uint32_t crc = get_u32(data + size - sizeof(std::uint32_t));
    if (dima::parameters::crc32(data, size - sizeof(std::uint32_t)) != crc) {
        return -EBADMSG;
    }
    const int result = codec::decode(data + kHeaderSize, size - kOverhead, plan);
    const std::uint32_t current = get_u32(data + 12U);
    if (result != 0 || (plan.count == 0U ? current != 0U : current >= plan.count)) {
        plan = {};
        return result != 0 ? result : -EBADMSG;
    }
    plan.current = static_cast<std::uint16_t>(current);
    identity = {get_generation(data + 4U), crc};
    return 0;
}

} // namespace dima::modules::mission::snapshot
