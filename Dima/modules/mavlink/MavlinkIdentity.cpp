#include "MavlinkIdentity.hpp"

#include "FirmwareIdentityContract.hpp"
#include "mavlink/MavlinkBridge.h"

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
    // 只声明本固件实际实现的参数 float/逐字节编码、FTP、COMMAND_INT 和 MAVLink2；
    // 未实现的任务上传、参数 EXT 写入等能力不得为迎合 QGC 而虚报。
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
    std::memcpy(out,
                dima::generated::firmware_identity::kFlightCustomVersion,
                sizeof(dima::generated::firmware_identity::
                           kFlightCustomVersion));
}

void MavlinkIdentity::get_middleware_custom_version(
    uint8_t out[8]) const noexcept
{
    // 当前飞控与中间件由同一仓库、同一镜像生成，故二者共享生成的 8-byte
    // commit 身份；若未来拆分版本源，应由身份生成合同分别提供，而非在此手写。
    std::memcpy(out,
                dima::generated::firmware_identity::kFlightCustomVersion,
                sizeof(dima::generated::firmware_identity::
                           kFlightCustomVersion));
}

void MavlinkIdentity::get_os_custom_version(uint8_t out[8]) const noexcept
{
    // FreeRTOS/STM32 中间件尚无独立的 8-byte 构建身份合同，按 MAVLink 语义填零。
    std::memset(out, 0, 8);
}

} // namespace dima::modules::mavlink
