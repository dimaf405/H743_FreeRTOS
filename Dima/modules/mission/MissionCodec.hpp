#pragma once

#include "MissionRepository.hpp"
#include "mavlink/MavlinkBridge.h"

#include <cstddef>
#include <cstdint>

namespace dima::modules::mission::codec {

// 64 个 MISSION_ITEM_INT 与一个 MISSION_COUNT 均使用 MAVLink2 无签名标准帧；
// 文件内 frame 已规范化为 GLOBAL_INT/GLOBAL_RELATIVE_ALT_INT。
constexpr std::size_t kFileCapacity =
    MAVLINK_MSG_ID_MISSION_COUNT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES +
    kMissionCapacity *
        (MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN +
         MAVLINK_NUM_NON_PAYLOAD_BYTES);

int encode(const MissionPlan &plan, std::uint8_t *destination,
           std::size_t capacity, std::size_t &output_size,
           std::uint32_t &mission_id) noexcept;

int decode(const std::uint8_t *data, std::size_t size,
           MissionPlan &plan) noexcept;

} // namespace dima::modules::mission::codec
