#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"

#include "parameter_metadata_files.hpp"
#include "platform/api/Time.hpp"

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
