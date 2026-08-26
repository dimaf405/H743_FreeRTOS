#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"

#include "parameter_metadata_files.hpp"
#include "api/Time.hpp"

#include <cstring>

namespace dima::modules::mavlink {
namespace {

namespace metadata = dima::generated::parameter_metadata;

} // namespace

bool MavlinkService::send_autopilot_version() noexcept
{
    mavlink_message_t message{};
    heartbeat_pacer_.pack_autopilot_version(message);
    return send_message(message);
}

bool MavlinkService::send_protocol_version() noexcept
{
    // version 字段单位为 100：100=MAVLink1，200=MAVLink2。当前未生成规范库与
    // MAVLink C 库各自的 8-byte hash，因此两个 hash 数组必须明确置零。
    mavlink_protocol_version_t version{};
    version.version = 200;
    version.min_version = 100;
    version.max_version = 200;
    std::memset(version.spec_version_hash, 0, sizeof(version.spec_version_hash));
    std::memset(version.library_version_hash, 0,
                sizeof(version.library_version_hash));

    mavlink_message_t message{};
    mavlink_msg_protocol_version_encode(MAVLINK_SYSTEM_ID,
                                        MAVLINK_COMPONENT_ID,
                                        &message, &version);
    return send_message(message);
}

bool MavlinkService::send_component_metadata() noexcept
{
    // URI 与 CRC 来自参数 metadata 生成物；二者必须同代，QGC 才能安全复用缓存。
    mavlink_component_metadata_t metadata_message{};
    metadata_message.time_boot_ms = static_cast<std::uint32_t>(
        hrt_absolute_time() / 1000ULL);
    metadata_message.file_crc = metadata::kGeneralCrc;
    std::strncpy(metadata_message.uri, metadata::kGeneralUri,
                 sizeof(metadata_message.uri) - 1U);

    mavlink_message_t message{};
    mavlink_msg_component_metadata_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
        &message, &metadata_message);
    return send_message(message);
}

bool MavlinkService::send_component_information() noexcept
{
    // COMPONENT_INFORMATION 提供兼容旧 QGC 的同一 general metadata 入口。
    mavlink_component_information_t information{};
    information.time_boot_ms = static_cast<std::uint32_t>(
        hrt_absolute_time() / 1000ULL);
    information.general_metadata_file_crc = metadata::kGeneralCrc;
    std::strncpy(information.general_metadata_uri, metadata::kGeneralUri,
                 sizeof(information.general_metadata_uri) - 1U);

    mavlink_message_t message{};
    mavlink_msg_component_information_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
        &message, &information);
    return send_message(message);
}

} // namespace dima::modules::mavlink
