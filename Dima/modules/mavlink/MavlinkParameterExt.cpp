#define MODULE_NAME "mavlink"
#include "MavlinkParameters.hpp"

#include <cstdio>
#include <cstring>

namespace dima::modules::mavlink {
namespace {

constexpr std::uint8_t kMavParamExtTypeFloat = 9U;
constexpr std::uint8_t kMavParamExtTypeInt32 = 6U;

} // namespace

void MavlinkParameters::handle_param_ext_request_read(
    const mavlink_message_t *msg) noexcept
{
    mavlink_param_ext_request_read_t req;
    mavlink_msg_param_ext_request_read_decode(msg, &req);

    if (req.target_system != MAVLINK_SYSTEM_ID ||
        (req.target_component != MAVLINK_COMPONENT_ID &&
         req.target_component != MAV_COMP_ID_ALL)) {
        return;
    }

    if (req.param_index >= 0) {
        send_param_ext_not_found(req.param_id, 0);
        return;
    }

    char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
    std::strncpy(name, req.param_id,
                 MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

    param_t param = param_find_no_notification(name);
    if (param == PARAM_INVALID) {
        send_param_ext_not_found(
            req.param_id,
            _send_all_count > 0U
                ? static_cast<std::uint16_t>(_send_all_count)
                : param_count_used());
        return;
    }

    char value_str[128];
    std::memset(value_str, 0, sizeof(value_str));
    uint8_t ext_type;

    if (param_type(param) == PARAM_TYPE_INT32) {
        int32_t value;
        if (param_get(param, &value) != 0) {
            return;
        }
        std::snprintf(value_str, sizeof(value_str), "%d",
                      static_cast<int>(value));
        ext_type = kMavParamExtTypeInt32;
    } else {
        float value;
        if (param_get(param, &value) != 0) {
            return;
        }
        std::snprintf(value_str, sizeof(value_str), "%.9g",
                      static_cast<double>(value));
        ext_type = kMavParamExtTypeFloat;
    }

    unsigned reply_count = param_count_used();
    int reply_index = param_get_used_index(param);
    const int snapshot_index = parameter_snapshot_index(param);
    if (snapshot_index >= 0) {
        reply_count = _send_all_count;
        reply_index = snapshot_index;
    }

    mavlink_param_ext_value_t reply{};
    reply.param_count = static_cast<std::uint16_t>(reply_count);
    reply.param_index = static_cast<std::uint16_t>(reply_index);
    reply.param_type = ext_type;
    std::strncpy(reply.param_id, param_name(param),
                 MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_LEN);
    std::strncpy(reply.param_value, value_str,
                 MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN);

    mavlink_message_t packet{};
    mavlink_msg_param_ext_value_encode(MAVLINK_SYSTEM_ID,
                                       MAVLINK_COMPONENT_ID,
                                       &packet, &reply);
    if (send_ != nullptr) {
        send_(send_ctx_, packet);
    }
}

void MavlinkParameters::send_param_ext_not_found(
    const char param_id[16], uint16_t count) noexcept
{
    mavlink_param_ext_value_t reply{};
    reply.param_count = count;
    reply.param_index = 0xFFFF;
    reply.param_type = 0;
    std::memcpy(reply.param_id, param_id,
                MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_LEN);

    mavlink_message_t packet{};
    mavlink_msg_param_ext_value_encode(MAVLINK_SYSTEM_ID,
                                       MAVLINK_COMPONENT_ID,
                                       &packet, &reply);
    if (send_ != nullptr) {
        send_(send_ctx_, packet);
    }
}

} // namespace dima::modules::mavlink
