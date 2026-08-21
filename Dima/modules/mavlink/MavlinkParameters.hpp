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
 *   - Before the first list request, real RC parameters and the fixed rover
 *     identity are explicitly marked used so QGC cannot race the later
 *     RCUpdate startup or fail its Radio-page Airframe prerequisite.
 */

#include "parameter_update.hpp"
#include "lib/mavlink/mavlink_bridge.h"
#include "parameters/param.h"
#include "parameters/QgcCompatibility.hpp"
#include "platform/api/Time.hpp"
#include "uorb/SubscriptionData.hpp"

#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkParameters {
public:
    /** Transmit callback: finalise + send the prepared mavlink_message_t.
     *  Returns false when the transport could not accept the frame. */
    using SendFn = bool (*)(void *ctx, mavlink_message_t &msg);

    MavlinkParameters(SendFn send, void *send_ctx) noexcept;

    void reset() noexcept;

    unsigned get_size() const noexcept;

    void handle_message(const mavlink_message_t *msg) noexcept;

    /**
     * Periodic TX entry point — streams pending parameter values.
     * USB link: try to send up to 20 at once (PX4 semantics).
     */
    void send() noexcept;

private:
    using FixedInt32Parameter =
        dima::parameters::QgcFixedInt32Parameter;

    static const FixedInt32Parameter *fixed_int32_parameter(
        const char *name) noexcept;

    static bool is_qgc_fixed_parameter(const char *name) noexcept;

    static bool is_serial_baud_parameter(const char *name) noexcept;

    static bool supported_serial_baud(std::int32_t value) noexcept;

    static bool serial_function_write_allowed(
        const char *name, std::int32_t value) noexcept;

    bool write_and_acknowledge_parameter(
        param_t param, float wire_value) noexcept;

    bool set_serial_function(
        param_t param, const char *name, std::int32_t value) noexcept;

    void mark_qgc_setup_parameters_used() noexcept;

    static void append_used_parameter(void *context,
                                      param_t param) noexcept;

    void stop_parameter_stream() noexcept;

    void clear_parameter_snapshot() noexcept;

    void snapshot_parameter_stream() noexcept;

    int parameter_snapshot_index(param_t param) const noexcept;

    static bool write_value_allowed(param_t param,
                                    float wire_value) noexcept;

    bool send_params() noexcept;

    bool send_untransmitted() noexcept;

    /* 发送下一个参数：send_param 返回 1（传输失败）时保留 index
     * 待下一轮重发；返回 2（param_get 读取失败）时推进并跳过，
     * 防止持续读取失败把参数流永久卡在同一 index。 */
    bool send_one() noexcept;

    int send_param(param_t param) noexcept;

    int send_param(param_t param, std::uint16_t count,
                   std::uint16_t index) noexcept;

    void handle_param_ext_request_read(
        const mavlink_message_t *msg) noexcept;

    void send_param_ext_not_found(const char param_id[16],
                                  uint16_t count) noexcept;

    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
    param_t _send_all_snapshot[px4::param_info_count]{};
    int _send_all_index{-1};
    unsigned _send_all_count{0U};
    hrt_abstime _param_update_time{0};
    int _param_update_index{0};
    bool _qgc_setup_parameters_marked{false};
    uORB::SubscriptionData<parameter_update_s> _parameter_update_sub{
        ORB_ID(parameter_update)};
};

}  // namespace dima::modules::mavlink
