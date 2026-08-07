#pragma once
/*
 * MAVLink Classic Parameter Protocol handler — ported from
 * PX4-Autopilot v1.17.0 src/modules/mavlink/mavlink_parameters.cpp
 * (commit d6f12ad), class MavlinkParametersManager.
 *
 * Dima adaptations:
 *   - The Mavlink& channel reference is replaced by a transmit
 *     callback; TX buffer accounting is delegated to the caller.
 *   - UAVCAN parameter forwarding is removed (no UAVCAN on board).
 *   - PARAM_HASH / _HASH_CHECK is omitted: Dima's param core has no
 *     param_hash_check(); _send_all_index starts streaming from 0,
 *     which QGC accepts.
 *   - send_untransmitted() (parameter_update echo of unsaved values)
 *     is kept in simplified form without radio-status throttling.
 *   - The first_send QGC-compatibility param_find() list is omitted:
 *     those parameters do not exist on this platform.
 */

#include "parameter_update.hpp"
#include "lib/mavlink/mavlink_bridge.h"
#include "logging/logging.hpp"
#include "parameters/param.h"
#include "platform/api/Time.hpp"
#include "uorb/SubscriptionData.hpp"

#include <cstring>
#include <cstdio>

/* MAV_PARAM_EXT_TYPE constants (MAVLink param extension protocol). */
static constexpr uint8_t MAV_PARAM_EXT_TYPE_FLOAT  = 9;
static constexpr uint8_t MAV_PARAM_EXT_TYPE_INT32  = 6;

namespace dima::modules::mavlink {

class MavlinkParameters {
public:
    /** Transmit callback: finalise + send the prepared mavlink_message_t.
     *  Returns false when the transport could not accept the frame. */
    using SendFn = bool (*)(void *ctx, mavlink_message_t &msg);

    MavlinkParameters(SendFn send, void *send_ctx) noexcept
        : send_(send), send_ctx_(send_ctx)
    {
    }

    unsigned get_size() const noexcept
    {
        return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

    void handle_message(const mavlink_message_t *msg) noexcept
    {
        switch (msg->msgid) {
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            /* request all parameters */
            mavlink_param_request_list_t req_list;
            mavlink_msg_param_request_list_decode(msg, &req_list);

            if (req_list.target_system == MAVLINK_SYSTEM_ID &&
                (req_list.target_component == MAVLINK_COMPONENT_ID ||
                 req_list.target_component == MAV_COMP_ID_ALL)) {
                /* No hash check on this platform: stream from index 0.
                 * A restart skips straight to the list (PX4 semantics
                 * for repeated requests). */
                _send_all_index = 0;
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
                    _send_all_index = -1;
                    /* No other action taken, return */
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
                    send_param(param_find_no_notification(name));

                } else {
                    /* when index is >= 0, send this parameter again */
                    int ret = send_param(
                        param_for_used_index(req_read.param_index));

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

    /**
     * Periodic TX entry point — streams pending parameter values.
     * USB link: try to send up to 20 at once (PX4 semantics).
     */
    void send() noexcept
    {
        int max_num_to_send = 20;

        int i = 0;

        /* Send while burst is not exceeded and still something to send */
        while ((i++ < max_num_to_send) && send_params()) {}
    }

private:
    bool send_params() noexcept
    {
        if (send_one()) {
            return true;

        } else if (send_untransmitted()) {
            return true;
        }

        return false;
    }

    bool send_untransmitted() noexcept
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

    bool send_one() noexcept
    {
        if (_send_all_index >= 0) {
            /* send all parameters if requested */

            /* look for the first parameter which is used */
            param_t p;

            do {
                /* walk through all parameters, including unused ones */
                p = param_for_index(_send_all_index);
                _send_all_index++;
            } while (p != PARAM_INVALID && !param_used(p));

            if (p != PARAM_INVALID) {
                send_param(p);
            }

            if ((p == PARAM_INVALID) ||
                (_send_all_index >= (int)param_count())) {
                _send_all_index = -1;
                return false;

            } else {
                return true;
            }
        }

        return false;
    }

    int send_param(param_t param) noexcept
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

        msg.param_count = param_count_used();
        msg.param_index = param_get_used_index(param);

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

    void handle_param_ext_request_read(const mavlink_message_t *msg) noexcept
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

        param_t param = param_find_no_notification(name);
        if (param == PARAM_INVALID) {
            send_param_ext_not_found(req.param_id,
                                     param_count_used());
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
            ext_type = MAV_PARAM_EXT_TYPE_INT32;
        } else {
            float v;
            if (param_get(param, &v) != 0) {
                return;
            }
            std::snprintf(value_str, sizeof(value_str), "%.9g",
                          static_cast<double>(v));
            ext_type = MAV_PARAM_EXT_TYPE_FLOAT;
        }

        mavlink_param_ext_value_t reply{};
        reply.param_count = param_count_used();
        reply.param_index = static_cast<uint16_t>(param_get_used_index(param));
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

    void send_param_ext_not_found(const char param_id[16],
                                  uint16_t count) noexcept
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

    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
    int _send_all_index{-1};
    hrt_abstime _param_update_time{0};
    int _param_update_index{0};
    uORB::SubscriptionData<parameter_update_s> _parameter_update_sub{
        ORB_ID(parameter_update)};
};

}  // namespace dima::modules::mavlink
