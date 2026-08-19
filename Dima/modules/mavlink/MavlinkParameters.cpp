#define MODULE_NAME "mavlink"
#include "MavlinkParameters.hpp"

#include "board_serial_config.hpp"
#include "logging/logging.hpp"

#include <cstdio>
#include <cstring>

namespace dima::modules::mavlink {
namespace {

constexpr std::uint8_t kMavParamExtTypeFloat = 9U;
constexpr std::uint8_t kMavParamExtTypeInt32 = 6U;

} // namespace

const MavlinkParameters::FixedInt32Parameter
    MavlinkParameters::kQgcFixedInt32Parameters[]{
        {"SYS_AUTOSTART", 50000},
        {"SYS_AUTOCONFIG", 0},
        {"MAV_SYS_ID", 1},
        {"CAL_GYRO0_ID", 0},
        {"CAL_ACC0_ID", 0},
        {"CAL_MAG0_ID", 0},
        {"CAL_MAG1_ID", 0},
        {"CAL_MAG2_ID", 0},
        {"NAV_RCL_ACT", 6},
        {"NAV_DLL_ACT", 0},
        {"COM_LOW_BAT_ACT", 0},
    };

MavlinkParameters::MavlinkParameters(SendFn send, void *send_ctx) noexcept
    : send_(send), send_ctx_(send_ctx)
{
}

void MavlinkParameters::reset() noexcept
{
    clear_parameter_snapshot();
    _param_update_time = 0U;
    _param_update_index = 0;
    _qgc_setup_parameters_marked = false;
}

unsigned MavlinkParameters::get_size() const noexcept
{
    return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void MavlinkParameters::handle_message(
    const mavlink_message_t *msg) noexcept
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        /* request all parameters */
        mavlink_param_request_list_t req_list;
        mavlink_msg_param_request_list_decode(msg, &req_list);

        if (req_list.target_system == MAVLINK_SYSTEM_ID &&
            (req_list.target_component == MAVLINK_COMPONENT_ID ||
             req_list.target_component == MAV_COMP_ID_ALL)) {
            mark_qgc_setup_parameters_used();
            /* No hash check on this platform: stream from index 0.
             * A restart skips straight to the list (PX4 semantics
             * for repeated requests). */
            snapshot_parameter_stream();
            PX4_INFO("Starting param stream: %u params", _send_all_count);
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET: {
        /* set parameter */
        mavlink_param_set_t set;
        mavlink_msg_param_set_decode(msg, &set);

        if (set.target_system == MAVLINK_SYSTEM_ID &&
            (set.target_component == MAVLINK_COMPONENT_ID ||
             set.target_component == MAV_COMP_ID_ALL)) {

            /* local name buffer to enforce null-terminated string */
            char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
            std::strncpy(name, set.param_id,
                         MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
            /* enforce null termination */
            name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

            /* Whatever the value is, we're being told to stop sending */
            if (std::strncmp(name, "_HASH_CHECK", sizeof(name)) == 0) {
                clear_parameter_snapshot();
                /* No other action taken, return */
                return;
            }
            if (is_internal_parameter(name)) {
                return;
            }

            /* attempt to find parameter, set and send it */
            param_t param = param_find_no_notification(name);

            if (param == PARAM_INVALID) {
                PX4_ERR("unknown param: %s", name);

            } else if (!((param_type(param) == PARAM_TYPE_INT32 &&
                           set.param_type == MAV_PARAM_TYPE_INT32) ||
                         (param_type(param) == PARAM_TYPE_FLOAT &&
                           set.param_type == MAV_PARAM_TYPE_REAL32))) {
                PX4_ERR("param types mismatch param: %s", name);

            } else if (!write_value_allowed(param, set.param_value)) {
                PX4_ERR("unsupported param value: %s", name);
                (void)send_param(param);

            } else {
                /* According to the mavlink spec we should always
                 * acknowledge a write operation. */
                param_set(param, &(set.param_value));
                send_param(param);
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        /* request one parameter */
        mavlink_param_request_read_t req_read;
        mavlink_msg_param_request_read_decode(msg, &req_read);

        if (req_read.target_system == MAVLINK_SYSTEM_ID &&
            (req_read.target_component == MAVLINK_COMPONENT_ID ||
             req_read.target_component == MAV_COMP_ID_ALL)) {

            /* when no index is given, loop through string ids and compare them */
            if (req_read.param_index < 0) {
                /* local name buffer to enforce null-terminated string */
                char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
                std::strncpy(name, req_read.param_id,
                             MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
                /* enforce null termination */
                name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
                /* attempt to find parameter and send it */
                if (!is_internal_parameter(name)) {
                    send_param(param_find_no_notification(name));
                }

            } else {
                /* when index is >= 0, send this parameter again */
                const unsigned requested_index =
                    static_cast<unsigned>(req_read.param_index);
                int ret = 1;
                if (_send_all_count > 0U) {
                    if (requested_index < _send_all_count) {
                        ret = send_param(
                            _send_all_snapshot[requested_index],
                            static_cast<std::uint16_t>(_send_all_count),
                            static_cast<std::uint16_t>(requested_index));
                    }
                } else {
                    ret = send_param(param_for_used_index(requested_index));
                }

                if (ret == 1) {
                    PX4_ERR("unknown param ID: %i", req_read.param_index);

                } else if (ret == 2) {
                    PX4_ERR("failed loading param from storage ID: %i",
                            req_read.param_index);
                }
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
        handle_param_ext_request_read(msg);
        break;

    default:
        break;
    }
}

void MavlinkParameters::send() noexcept
{
    int max_num_to_send = 20;
    int i = 0;

    /* Send while burst is not exceeded and still something to send */
    while ((i++ < max_num_to_send) && send_params()) {}
}

const MavlinkParameters::FixedInt32Parameter *
MavlinkParameters::fixed_int32_parameter(const char *name) noexcept
{
    if (name == nullptr) {
        return nullptr;
    }
    for (const FixedInt32Parameter &parameter :
         kQgcFixedInt32Parameters) {
        if (std::strcmp(name, parameter.name) == 0) {
            return &parameter;
        }
    }
    return nullptr;
}

bool MavlinkParameters::is_qgc_fixed_parameter(const char *name) noexcept
{
    return fixed_int32_parameter(name) != nullptr;
}

bool MavlinkParameters::is_internal_parameter(const char *name) noexcept
{
    return name != nullptr &&
        (std::strcmp(name, "RC_PORT_CONFIG") == 0 ||
         std::strcmp(name, "DIMA_SER_VER") == 0);
}

bool MavlinkParameters::is_serial_baud_parameter(
    const char *name) noexcept
{
    return dima::board::serial_baud_parameter(name);
}

bool MavlinkParameters::supported_serial_baud(std::int32_t value) noexcept
{
    return value >= 0 && dima::board::serial_baud_supported(
        static_cast<std::uint32_t>(value));
}

bool MavlinkParameters::serial_function_write_allowed(
    const char *name, std::int32_t value) noexcept
{
    if (!dima::board::serial_function_supported(value)) {
        return false;
    }
    if (value == dima::board::kSerialFunctionDisabled) {
        return true;
    }
    for (const dima::board::SerialPortDescriptor &descriptor :
         dima::board::kSerialPorts) {
        if (std::strcmp(name, descriptor.function_parameter_name) == 0) {
            continue;
        }
        const param_t other = param_find_no_notification(
            descriptor.function_parameter_name);
        std::int32_t other_value{};
        if (other == PARAM_INVALID || param_get(other, &other_value) != 0 ||
            other_value == dima::board::kSerialFunctionRcInput) {
            return false;
        }
    }
    return true;
}

void MavlinkParameters::mark_qgc_setup_parameters_used() noexcept
{
    if (_qgc_setup_parameters_marked) {
        return;
    }

    for (unsigned index = 0U; index < param_count(); ++index) {
        const param_t param = param_for_index(index);
        const char *const name = param_name(param);
        if (name != nullptr &&
            ((std::strncmp(name, "RC", 2U) == 0 &&
              std::strcmp(name, "RC_PORT_CONFIG") != 0) ||
             dima::board::serial_baud_parameter(name) ||
             dima::board::serial_function_parameter(name) ||
             std::strcmp(name, "COM_RC_IN_MODE") == 0 ||
             is_qgc_fixed_parameter(name))) {
            param_set_used(param);
        }
    }
    _qgc_setup_parameters_marked = true;
}

void MavlinkParameters::append_used_parameter(
    void *context, param_t param) noexcept
{
    if (context == nullptr) {
        return;
    }
    auto &self = *static_cast<MavlinkParameters *>(context);
    if (self._send_all_count < px4::param_info_count) {
        self._send_all_snapshot[self._send_all_count++] = param;
    }
}

void MavlinkParameters::stop_parameter_stream() noexcept
{
    _send_all_index = -1;
}

void MavlinkParameters::clear_parameter_snapshot() noexcept
{
    stop_parameter_stream();
    _send_all_count = 0U;
}

void MavlinkParameters::snapshot_parameter_stream() noexcept
{
    static_assert(px4::param_info_count <= 0xFFFFU,
                  "MAVLink parameter count exceeds uint16_t");
    _send_all_count = 0U;
    param_foreach(&MavlinkParameters::append_used_parameter, this,
                  false, true);
    _send_all_index = _send_all_count == 0U ? -1 : 0;
}

int MavlinkParameters::parameter_snapshot_index(param_t param) const noexcept
{
    for (unsigned index = 0U; index < _send_all_count; ++index) {
        if (_send_all_snapshot[index] == param) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool MavlinkParameters::write_value_allowed(param_t param,
                                            float wire_value) noexcept
{
    const char *const name = param_name(param);
    if (name != nullptr && std::strcmp(name, "COM_RC_IN_MODE") == 0) {
        std::int32_t mode = 0;
        std::memcpy(&mode, &wire_value, sizeof(mode));
        return mode == 0;
    }
    if (name != nullptr && std::strcmp(name, "RC_MAP_FLTMODE") == 0) {
        std::int32_t mapping = 0;
        std::memcpy(&mapping, &wire_value, sizeof(mapping));
        return mapping == 0;
    }
    if (name != nullptr && std::strcmp(name, "RC_PORT_CONFIG") == 0) {
        return false;
    }
    if (name != nullptr && std::strcmp(name, "RC_INPUT_PROTO") == 0) {
        std::int32_t protocol = 0;
        std::memcpy(&protocol, &wire_value, sizeof(protocol));
        return protocol == 0 || protocol == 2;
    }
    if (is_serial_baud_parameter(name)) {
        std::int32_t baudrate = 0;
        std::memcpy(&baudrate, &wire_value, sizeof(baudrate));
        return supported_serial_baud(baudrate);
    }
    if (dima::board::serial_function_parameter(name)) {
        std::int32_t function = 0;
        std::memcpy(&function, &wire_value, sizeof(function));
        return serial_function_write_allowed(name, function);
    }
    if (name != nullptr && std::strcmp(name, "DIMA_SER_VER") == 0) {
        return false;
    }
    if (const FixedInt32Parameter *const fixed =
            fixed_int32_parameter(name)) {
        std::int32_t value = 0;
        std::memcpy(&value, &wire_value, sizeof(value));
        return value == fixed->value;
    }
    return true;
}

bool MavlinkParameters::send_params() noexcept
{
    if (send_one()) {
        return true;

    } else if (send_untransmitted()) {
        return true;
    }

    return false;
}

bool MavlinkParameters::send_untransmitted() noexcept
{
    bool sent_one = false;

    if (_parameter_update_sub.update()) {
        const parameter_update_s &pupdate = _parameter_update_sub.get();

        /* Schedule an update if not already the case */
        if (_param_update_time == 0) {
            _param_update_time = pupdate.timestamp;
            _param_update_index = 0;
        }
    }

    if ((_param_update_time != 0) &&
        ((_param_update_time + 5 * 1000) < hrt_absolute_time())) {

        param_t param = 0;

        /* send out all changed values */
        do {
            /* skip over all parameters which are not invalid and not used */
            do {
                param = param_for_index(_param_update_index);
                ++_param_update_index;
            } while (param != PARAM_INVALID && !param_used(param));

            /* send parameters which are untransmitted */
            if ((param != PARAM_INVALID) && param_value_unsaved(param)) {
                int ret = send_param(param);
                sent_one = true;

                if (ret != 0) {
                    break;
                }
            }
        } while (_param_update_index < (int)param_count());

        /* Flag work as done once all params have been sent */
        if (_param_update_index >= (int)param_count()) {
            _param_update_time = 0;
        }
    }

    return sent_one;
}

bool MavlinkParameters::send_one() noexcept
{
    if (_send_all_index >= 0) {
        if (static_cast<unsigned>(_send_all_index) >= _send_all_count) {
            stop_parameter_stream();
            return false;
        }

        const unsigned snapshot_index =
            static_cast<unsigned>(_send_all_index);
        const param_t param = _send_all_snapshot[snapshot_index];
        const int ret = send_param(
            param, static_cast<std::uint16_t>(_send_all_count),
            static_cast<std::uint16_t>(snapshot_index));

        if (ret == 1) {
            /* 传输失败：保留快照 index，下一轮 Run() 重发。 */
            return false;
        }

        ++_send_all_index;
        if (ret == 2) {
            /* 读取失败（非传输失败）：快照 index 已推进，直接
             * 跳过该参数继续后续发送。 */
            PX4_ERR("param read failed, index %u skipped", snapshot_index);
        }

        if (static_cast<unsigned>(_send_all_index) >= _send_all_count) {
            stop_parameter_stream();
            return false;
        }
        return true;
    }

    return false;
}

int MavlinkParameters::send_param(param_t param) noexcept
{
    const int snapshot_index = parameter_snapshot_index(param);
    if (snapshot_index >= 0) {
        return send_param(
            param, static_cast<std::uint16_t>(_send_all_count),
            static_cast<std::uint16_t>(snapshot_index));
    }
    return send_param(
        param, static_cast<std::uint16_t>(param_count_used()),
        static_cast<std::uint16_t>(param_get_used_index(param)));
}

int MavlinkParameters::send_param(param_t param, std::uint16_t count,
                                  std::uint16_t index) noexcept
{
    if (param == PARAM_INVALID) {
        return 1;
    }

    mavlink_param_value_t msg{};

    /*
     * get param value, since MAVLink encodes float and int params in the same
     * space during transmission, copy param onto float val_buf
     */
    if (param_type(param) == PARAM_TYPE_INT32) {
        int32_t param_value;

        if (param_get(param, &param_value) != 0) {
            return 2;
        }

        std::memcpy(&msg.param_value, &param_value, sizeof(param_value));

    } else {
        float param_value;

        if (param_get(param, &param_value) != 0) {
            return 2;
        }

        msg.param_value = param_value;
    }

    msg.param_count = count;
    msg.param_index = index;

    /*
     * The MAVLink spec does not require the string to be NUL-terminated if it
     * has length 16. In this case the receiving end needs to terminate it
     * when copying it.
     */
    std::strncpy(msg.param_id, param_name(param),
                 MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

    /* query parameter type */
    param_type_t type = param_type(param);

    /*
     * Map onboard parameter type to MAVLink type,
     * endianess matches (both little endian)
     */
    if (type == PARAM_TYPE_INT32) {
        msg.param_type = MAVLINK_TYPE_INT32_T;

    } else if (type == PARAM_TYPE_FLOAT) {
        msg.param_type = MAVLINK_TYPE_FLOAT;

    } else {
        msg.param_type = MAVLINK_TYPE_FLOAT;
    }

    mavlink_message_t packet{};
    mavlink_msg_param_value_encode(MAVLINK_SYSTEM_ID,
                                   MAVLINK_COMPONENT_ID, &packet, &msg);
    if (send_ != nullptr && send_(send_ctx_, packet)) {
        return 0;
    }
    return 1;
}

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

    /* Only support lookup by name (param_index == -1). */
    if (req.param_index >= 0) {
        send_param_ext_not_found(req.param_id, 0);
        return;
    }

    char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
    std::strncpy(name, req.param_id,
                 MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

    if (is_internal_parameter(name)) {
        send_param_ext_not_found(
            req.param_id,
            _send_all_count > 0U
                ? static_cast<std::uint16_t>(_send_all_count)
                : param_count_used());
        return;
    }

    param_t param = param_find_no_notification(name);
    if (param == PARAM_INVALID) {
        send_param_ext_not_found(
            req.param_id,
            _send_all_count > 0U
                ? static_cast<std::uint16_t>(_send_all_count)
                : param_count_used());
        return;
    }

    /* Format value as string (float preserves precision). */
    char value_str[128];
    std::memset(value_str, 0, sizeof(value_str));
    uint8_t ext_type;

    if (param_type(param) == PARAM_TYPE_INT32) {
        int32_t v;
        if (param_get(param, &v) != 0) {
            return;
        }
        std::snprintf(value_str, sizeof(value_str), "%d",
                      static_cast<int>(v));
        ext_type = kMavParamExtTypeInt32;
    } else {
        float v;
        if (param_get(param, &v) != 0) {
            return;
        }
        std::snprintf(value_str, sizeof(value_str), "%.9g",
                      static_cast<double>(v));
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
