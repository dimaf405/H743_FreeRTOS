#include "MavlinkIdentity.hpp"

#include "lib/mavlink/mavlink_bridge.h"

#include <cstring>

namespace dima::modules::mavlink {

void MavlinkIdentity::configure(uint32_t version, uint32_t board_version,
                                uint64_t uid, uint16_t vendor_id,
                                uint16_t product_id)
{
    flight_sw_version_ = version;
    board_version_ = board_version;
    uid_ = uid;
    vendor_id_ = vendor_id;
    product_id_ = product_id;
}

void MavlinkIdentity::set_state(uint8_t base_mode,
                                uint8_t system_status)
{
    base_mode_ = base_mode;
    system_status_ = system_status;
}

uint64_t MavlinkIdentity::capabilities() const noexcept
{
    return static_cast<std::uint64_t>(MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT)
         | static_cast<std::uint64_t>(MAV_PROTOCOL_CAPABILITY_FTP)
         | static_cast<std::uint64_t>(MAV_PROTOCOL_CAPABILITY_COMMAND_INT)
         | static_cast<std::uint64_t>(
               MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
         | static_cast<std::uint64_t>(MAV_PROTOCOL_CAPABILITY_MAVLINK2);
}

void MavlinkIdentity::get_flight_custom_version(
    uint8_t out[8]) const noexcept
{
    std::memset(out, 0, 8);
}

void MavlinkIdentity::get_middleware_custom_version(
    uint8_t out[8]) const noexcept
{
    std::memset(out, 0, 8);
}

void MavlinkIdentity::get_os_custom_version(uint8_t out[8]) const noexcept
{
    std::memset(out, 0, 8);
}

} // namespace dima::modules::mavlink
