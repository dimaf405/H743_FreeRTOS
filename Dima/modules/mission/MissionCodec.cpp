#include "MissionCodec.hpp"

#include <cerrno>
#include <cfloat>
#include <cmath>
#include <cstring>

namespace dima::modules::mission::codec {
namespace {

constexpr std::uint32_t kFnvOffsetBasis = 2166136261U;
constexpr std::uint32_t kFnvPrime = 16777619U;

bool canonical_frame(std::uint8_t frame) noexcept
{
    // 文件格式故意只接受规范化的 _INT frame。QGC 在 MISSION_ITEM_INT 中携带
    // GLOBAL 别名的兼容处理位于协议接收边界，落盘后不得保留多种等价编码，
    // 否则同一任务会产生不同 mission ID，破坏 PX4 Mission State 的内容标识。
    return frame == MAV_FRAME_GLOBAL_INT ||
           frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
}

bool valid_item(const MissionItem &item) noexcept
{
    constexpr std::int32_t latitude_max_e7 = 900000000;
    constexpr std::int32_t longitude_max_e7 = 1800000000;
    return item.sequence < kMissionCapacity && canonical_frame(item.frame) &&
           item.latitude_e7 >= -latitude_max_e7 &&
           item.latitude_e7 <= latitude_max_e7 &&
           item.longitude_e7 >= -longitude_max_e7 &&
           item.longitude_e7 <= longitude_max_e7 &&
           std::isfinite(item.altitude_m) &&
           std::isfinite(item.acceptance_radius_m) &&
           item.acceptance_radius_m >= 0.0F;
}

void hash_bytes(std::uint32_t &hash, const std::uint8_t *data,
                std::size_t size) noexcept
{
    for (std::size_t index = 0U; index < size; ++index) {
        hash ^= data[index];
        hash *= kFnvPrime;
    }
}

struct EncodedStream {
    std::uint8_t *destination{nullptr};
    std::size_t capacity{0U};
    std::size_t size{0U};
    std::uint32_t hash{kFnvOffsetBasis};
    bool write{false};
};

bool append_message(const mavlink_message_t &message,
                    EncodedStream &stream) noexcept
{
    std::uint8_t frame[MAVLINK_MAX_PACKET_LEN]{};
    const std::uint16_t length =
        mavlink_msg_to_send_buffer(frame, &message);
    if (length == 0U || stream.size > stream.capacity ||
        static_cast<std::size_t>(length) > stream.capacity - stream.size) {
        return false;
    }
    hash_bytes(stream.hash, frame, length);
    if (stream.write) {
        std::memcpy(stream.destination + stream.size, frame, length);
    }
    stream.size += length;
    return true;
}

int encode_stream(const MissionPlan &plan, std::uint32_t opaque_id,
                  EncodedStream &stream) noexcept
{
    if (plan.count > kMissionCapacity ||
        (stream.write && stream.destination == nullptr)) {
        return -EINVAL;
    }
    for (std::uint16_t sequence = 0U; sequence < plan.count; ++sequence) {
        if (plan.items[sequence].sequence != sequence ||
            !valid_item(plan.items[sequence])) {
            return -EINVAL;
        }
    }

    // 使用独立 status 固定 MAVLink2 帧序号从 0 开始；持久格式不借用 USB
    // channel 的 parser/sequence 状态，因而相同任务始终生成相同字节流。
    mavlink_status_t tx_status{};
    mavlink_mission_count_t count{};
    count.count = plan.count;
    count.target_system = 0U;
    count.target_component = 0U;
    count.mission_type = MAV_MISSION_TYPE_MISSION;
    count.opaque_id = opaque_id;
    mavlink_message_t message{};
    (void)mavlink_msg_mission_count_encode_status(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
        &tx_status, &message, &count);
    if (!append_message(message, stream)) {
        return -ENOSPC;
    }

    for (std::uint16_t sequence = 0U; sequence < plan.count; ++sequence) {
        const MissionItem &source = plan.items[sequence];
        mavlink_mission_item_int_t item{};
        item.param1 = 0.0F;
        item.param2 = source.acceptance_radius_m;
        item.param3 = 0.0F;
        item.param4 = NAN;
        item.x = source.latitude_e7;
        item.y = source.longitude_e7;
        item.z = source.altitude_m;
        item.seq = sequence;
        item.command = MAV_CMD_NAV_WAYPOINT;
        item.target_system = 0U;
        item.target_component = 0U;
        item.frame = source.frame;
        item.current = sequence == 0U ? 1U : 0U;
        item.autocontinue = 1U;
        item.mission_type = MAV_MISSION_TYPE_MISSION;
        message = {};
        (void)mavlink_msg_mission_item_int_encode_status(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
            &tx_status, &message, &item);
        if (!append_message(message, stream)) {
            return -ENOSPC;
        }
    }
    return 0;
}

int compute_mission_id(const MissionPlan &plan,
                       std::uint32_t &mission_id) noexcept
{
    if (plan.count == 0U) {
        mission_id = 0U;
        return 0;
    }
    EncodedStream canonical{nullptr, kFileCapacity, 0U,
                            kFnvOffsetBasis, false};
    const int result = encode_stream(plan, 0U, canonical);
    if (result != 0) {
        return result;
    }
    // MAVLink opaque_id 以 0 表示“不支持/无任务”，因此非空任务即使哈希碰巧为
    // 0 也提升为 1，保持 AUTO 门控和 QGC plan-id 语义明确。
    mission_id = canonical.hash == 0U ? 1U : canonical.hash;
    return 0;
}

bool default_waypoint_fields(
    const mavlink_mission_item_int_t &item) noexcept
{
    // 首版只允许可选 acceptance radius。dwell/pass radius/指定 yaw 均必须保持
    // 协议默认值；这里再次验证落盘内容，损坏或旧版本扩展不会被静默忽略。
    return std::isfinite(item.param1) &&
           std::fabs(item.param1) <= FLT_EPSILON &&
           std::isfinite(item.param2) && item.param2 >= 0.0F &&
           std::isfinite(item.param3) &&
           std::fabs(item.param3) <= FLT_EPSILON &&
           std::isnan(item.param4) && item.autocontinue == 1U;
}

} // namespace

int encode(const MissionPlan &plan, std::uint8_t *destination,
           std::size_t capacity, std::size_t &output_size,
           std::uint32_t &mission_id) noexcept
{
    output_size = 0U;
    mission_id = 0U;
    if (destination == nullptr || capacity < kFileCapacity) {
        return -ENOSPC;
    }
    const int id_result = compute_mission_id(plan, mission_id);
    if (id_result != 0) {
        return id_result;
    }
    EncodedStream stream{destination, capacity, 0U,
                         kFnvOffsetBasis, true};
    const int encoded = encode_stream(plan, mission_id, stream);
    if (encoded != 0) {
        return encoded;
    }
    output_size = stream.size;
    return 0;
}

int decode(const std::uint8_t *data, std::size_t size,
           MissionPlan &plan) noexcept
{
    plan = {};
    if (data == nullptr || size == 0U || size > kFileCapacity) {
        return -EINVAL;
    }

    mavlink_message_t parser_message{};
    mavlink_status_t parser_status{};
    mavlink_message_t decoded_message{};
    mavlink_status_t decoded_status{};
    std::uint16_t frame_index = 0U;
    std::uint16_t expected_count = 0U;
    std::uint32_t stored_id = 0U;
    bool expecting_frame_start = true;

    for (std::size_t offset = 0U; offset < size; ++offset) {
        if (expecting_frame_start && data[offset] != MAVLINK_STX) {
            return -EBADMSG;
        }
        expecting_frame_start = false;
        const std::uint8_t framing = mavlink_frame_char_buffer(
            &parser_message, &parser_status, data[offset],
            &decoded_message, &decoded_status);
        if (framing == MAVLINK_FRAMING_BAD_CRC ||
            framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
            return -EBADMSG;
        }
        if (framing != MAVLINK_FRAMING_OK) {
            continue;
        }
        expecting_frame_start = true;

        if (frame_index == 0U) {
            if (decoded_message.msgid != MAVLINK_MSG_ID_MISSION_COUNT) {
                return -EBADMSG;
            }
            mavlink_mission_count_t count{};
            mavlink_msg_mission_count_decode(&decoded_message, &count);
            if (count.target_system != 0U ||
                count.target_component != 0U ||
                count.mission_type != MAV_MISSION_TYPE_MISSION ||
                count.count > kMissionCapacity) {
                return -EINVAL;
            }
            expected_count = count.count;
            stored_id = count.opaque_id;
            plan.count = expected_count;
        } else {
            if (frame_index > expected_count ||
                decoded_message.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT) {
                return -EBADMSG;
            }
            mavlink_mission_item_int_t item{};
            mavlink_msg_mission_item_int_decode(&decoded_message, &item);
            const std::uint16_t sequence =
                static_cast<std::uint16_t>(frame_index - 1U);
            if (item.seq != sequence ||
                item.command != MAV_CMD_NAV_WAYPOINT ||
                item.target_system != 0U || item.target_component != 0U ||
                item.mission_type != MAV_MISSION_TYPE_MISSION ||
                !canonical_frame(item.frame) ||
                item.current != (sequence == 0U ? 1U : 0U) ||
                !default_waypoint_fields(item)) {
                return -EINVAL;
            }
            MissionItem restored{};
            restored.sequence = sequence;
            restored.latitude_e7 = item.x;
            restored.longitude_e7 = item.y;
            restored.altitude_m = item.z;
            restored.acceptance_radius_m = item.param2;
            restored.frame = item.frame;
            if (!valid_item(restored)) {
                return -EINVAL;
            }
            plan.items[sequence] = restored;
        }
        ++frame_index;
    }

    if (!expecting_frame_start ||
        frame_index != static_cast<std::uint16_t>(expected_count + 1U)) {
        return -EBADMSG;
    }
    plan.current = 0U;
    plan.mission_id = stored_id;

    std::uint32_t computed_id{};
    const int id_result = compute_mission_id(plan, computed_id);
    if (id_result != 0 || computed_id != stored_id) {
        plan = {};
        return id_result != 0 ? id_result : -EBADMSG;
    }
    return 0;
}

} // namespace dima::modules::mission::codec
