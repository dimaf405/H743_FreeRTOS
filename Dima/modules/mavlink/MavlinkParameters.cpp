#define MODULE_NAME "mavlink"
#include "MavlinkParameters.hpp"

#include "SerialContract.hpp"
#include "logging/logging.hpp"

#include <cmath>
#include <cstring>

namespace dima::modules::mavlink {

MavlinkParameters::MavlinkParameters(SendFn send, void *send_ctx) noexcept
    : send_(send), send_ctx_(send_ctx)
{
}

void MavlinkParameters::reset() noexcept
{
    clear_parameter_snapshot();
    _param_update_time = 0U;
    _param_update_index = 0;
    _last_read_failure_index = -1;
    _catalogue_reported = false;
}

bool MavlinkParameters::prepare_parameter_catalogue() noexcept
{
    namespace contract = dima::generated::parameters;
    // MAVLink 直接遍历 PX4 官方目录的连续 handle；只校验生成总数，不再生成或
    // 维护第二份公开参数数组。
    static_assert(contract::kParameterCount ==
                      sizeof(px4::parameters) / sizeof(px4::parameters[0]),
                  "official parameter catalogues must have the same size");

    if (!param_is_ready() || param_count() !=
            contract::kParameterCount) {
        PX4_ERR("parameter catalogue unavailable: core=%u generated=%u",
                param_count(),
                static_cast<unsigned>(contract::kParameterCount));
        return false;
    }

    for (param_t handle = 0U; handle < contract::kParameterCount; ++handle) {
        param_set_used(handle);
    }
    for (param_t handle = 0U; handle < contract::kParameterCount; ++handle) {
        if (!param_used(handle)) {
            PX4_ERR("parameter catalogue activation failed: %s",
                    param_name(handle));
            return false;
        }
    }
    if (param_count_used() != contract::kParameterCount) {
        PX4_ERR("parameter catalogue count mismatch: active=%u generated=%u",
                param_count_used(),
                static_cast<unsigned>(contract::kParameterCount));
        return false;
    }

    if (!_catalogue_reported) {
        PX4_INFO("MAVLink parameter catalogue ready: %u params",
                 static_cast<unsigned>(contract::kParameterCount));
        _catalogue_reported = true;
    }
    return true;
}

void MavlinkParameters::handle_message(
    const mavlink_message_t *msg) noexcept
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        // 每次 LIST 都重新冻结目录；重复请求按 PX4 语义从索引 0 重新开始。
        mavlink_param_request_list_t req_list;
        mavlink_msg_param_request_list_decode(msg, &req_list);

        if (req_list.target_system == MAVLINK_SYSTEM_ID &&
            (req_list.target_component == MAVLINK_COMPONENT_ID ||
             req_list.target_component == MAV_COMP_ID_ALL)) {
            // 本平台不宣称 PARAM_HASH，不能用 _HASH_CHECK 提前结束；直接从索引 0 发送。
            if (snapshot_parameter_stream()) {
                PX4_INFO("Starting param stream: %u params", _send_all_count);
            } else {
                PX4_ERR("PARAM_REQUEST_LIST rejected: catalogue incomplete");
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET: {
        // 参数写入先做目标、类型和产品约束校验；成功后必须回显实际生效值作为 ACK。
        mavlink_param_set_t set;
        mavlink_msg_param_set_decode(msg, &set);

        if (set.target_system == MAVLINK_SYSTEM_ID &&
            (set.target_component == MAVLINK_COMPONENT_ID ||
             set.target_component == MAV_COMP_ID_ALL)) {

            // MAVLink 的 16 字节 param_id 可无 NUL，复制到本地后显式补终止符。
            char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
            std::strncpy(name, set.param_id,
                         MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
            /* enforce null termination */
            name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

            // 未发布匹配散列就不能接受 QGC/PX4 的缓存确认，否则会把普通参数列表错误截断。
            if (std::strncmp(name, "_HASH_CHECK", sizeof(name)) == 0) {
                PX4_WARN("Ignoring unadvertised PARAM_HASH acknowledgement");
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

            } else if (!write_and_acknowledge_parameter(
                           param, set.param_value)) {
                PX4_ERR("param write failed: %s", name);
                (void)send_param(param);
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        // 单参数既支持按名称，也支持按本次冻结目录的索引重传。
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

    // 单轮最多 20 帧，给 Commander、传感器和 ACK 留出同一 USB WorkQueue 带宽。
    while ((i++ < max_num_to_send) && send_params()) {}
}

const MavlinkParameters::FixedParameterConstraint *
MavlinkParameters::fixed_parameter_constraint(param_t param) noexcept
{
    namespace contract = dima::generated::parameters;
    for (const FixedParameterConstraint &constraint :
         contract::kFixedParameterConstraints) {
        if (param_handle(constraint.parameter) == param) {
            return &constraint;
        }
    }
    return nullptr;
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
    return dima::board::serial_function_parameter(name) &&
           dima::board::serial_function_supported(value);
}

bool MavlinkParameters::write_and_acknowledge_parameter(
    param_t param, float wire_value) noexcept
{
    const char *const name = param_name(param);
    if (dima::board::serial_function_parameter(name)) {
        std::int32_t function = 0;
        std::memcpy(&function, &wire_value, sizeof(function));
        return set_serial_function(param, name, function);
    }

    int result = 0;
    if (param_type(param) == PARAM_TYPE_INT32) {
        std::int32_t value = 0;
        std::memcpy(&value, &wire_value, sizeof(value));
        result = param_set(param, &value);
    } else {
        result = param_set(param, &wire_value);
    }
    if (result != 0) {
        return false;
    }

    // Classic 参数协议以回显 PARAM_VALUE 作为写入 ACK，且应回显存储层实际值。
    (void)send_param(param);
    return true;
}

bool MavlinkParameters::set_serial_function(
    param_t param, const char *name, std::int32_t value) noexcept
{
    param_t owners[sizeof(dima::board::kSerialPorts) /
                   sizeof(dima::board::kSerialPorts[0])]{};
    unsigned owner_count = 0U;
    {
        // QGC 常先在新端口启用独占功能，再关闭旧端口。整个交接放进参数原子事务：
        // 读者只能看到旧状态或完成后的新状态，不能看到两个 RC/GPS owner。
        px4::AtomicTransaction transaction;
        if (!serial_function_write_allowed(name, value)) {
            return false;
        }
        if (value == dima::board::kSerialFunctionDisabled) {
            if (param_set(param, &value) != 0) {
                return false;
            }
        } else {
            std::int32_t previous_target = 0;
            if (param_get(param, &previous_target) != 0) {
                return false;
            }

            for (const dima::board::SerialPortDescriptor &descriptor :
                 dima::board::kSerialPorts) {
                if (std::strcmp(
                        name, descriptor.function_parameter_name) == 0) {
                    continue;
                }
                const param_t other = param_find_no_notification(
                    descriptor.function_parameter_name);
                std::int32_t other_value = 0;
                if (other == PARAM_INVALID ||
                    param_get(other, &other_value) != 0 ||
                    !dima::board::serial_function_supported(other_value)) {
                    return false;
                }
                if (value != dima::board::kSerialFunctionDisabled &&
                    other_value == value) {
                    owners[owner_count++] = other;
                }
            }

            const bool target_changed = previous_target != value;
            if (target_changed &&
                param_set_no_notification(param, &value) != 0) {
                return false;
            }

            const std::int32_t disabled =
                dima::board::kSerialFunctionDisabled;
            unsigned cleared_count = 0U;
            for (; cleared_count < owner_count; ++cleared_count) {
                if (param_set_no_notification(
                        owners[cleared_count], &disabled) != 0) {
                    bool rollback_ok = true;
                    if (target_changed && param_set_no_notification(
                            param, &previous_target) != 0) {
                        rollback_ok = false;
                    }
                    for (unsigned rollback = 0U;
                         rollback < cleared_count; ++rollback) {
                        if (param_set_no_notification(
                                owners[rollback], &value) != 0) {
                            rollback_ok = false;
                        }
                    }
                    if (!rollback_ok) {
                        // 即使回滚不完整也必须通知消费者重新校验，禁止继续使用陈旧缓存。
                        param_notify_changes();
                        PX4_ERR("serial RC owner rollback failed");
                    }
                    return false;
                }
            }

            if (target_changed || owner_count > 0U) {
                param_notify_changes();
            }
        }
    }

    (void)send_param(param);
    for (unsigned index = 0U; index < owner_count; ++index) {
        (void)send_param(owners[index]);
    }
    return true;
}

void MavlinkParameters::append_used_parameter(
    void *context, param_t param) noexcept
{
    if (context == nullptr) {
        return;
    }
    auto &self = *static_cast<MavlinkParameters *>(context);
    if (self._send_all_count <
            dima::generated::parameters::kParameterCount) {
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

bool MavlinkParameters::snapshot_parameter_stream() noexcept
{
    namespace contract = dima::generated::parameters;
    static_assert(contract::kParameterCount <= 0xFFFFU,
                  "MAVLink parameter count exceeds uint16_t");
    if (!prepare_parameter_catalogue()) {
        clear_parameter_snapshot();
        return false;
    }
    // 每次 LIST 冻结 handle/count/index；之后才激活的参数必须等下一次 LIST，
    // 保证 QGC 的缺失索引重试不会解析到另一个 handle。
    _send_all_count = 0U;
    param_foreach(&MavlinkParameters::append_used_parameter, this,
                  false, true);
    // 完整数量闭包直接覆盖 PX4 官方目录，不再维护第二份 QGC 必需参数名单。
    if (_send_all_count != contract::kParameterCount) {
        PX4_ERR("parameter snapshot incomplete: snapshot=%u generated=%u",
                _send_all_count,
                static_cast<unsigned>(contract::kParameterCount));
        clear_parameter_snapshot();
        return false;
    }
    _send_all_index = _send_all_count == 0U ? -1 : 0;
    _last_read_failure_index = -1;
    return _send_all_index >= 0;
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
    // PX4/APM 把 min/max/enum metadata 作为 GCS 指引，不作为通用协议写门；这里拒绝
    // 非有限 float、生成合同的固定值及串口结构约束，输出专属校验与禁武装仍归消费者。
    if (param_type(param) == PARAM_TYPE_FLOAT &&
        !std::isfinite(wire_value)) {
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
    if (const FixedParameterConstraint *const fixed =
            fixed_parameter_constraint(param)) {
        if (fixed->type == dima::generated::parameters::
                FixedParameterType::Int32) {
            std::int32_t value = 0;
            std::memcpy(&value, &wire_value, sizeof(value));
            return value == fixed->int32_value;
        }
        return wire_value == fixed->float_value;
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

        // 首次更新后延迟 5 ms 合并同一参数事务产生的多个通知，避免重复回显半套状态。
        if (_param_update_time == 0) {
            _param_update_time = pupdate.timestamp;
            _param_update_index = 0;
        }
    }

    if ((_param_update_time != 0) &&
        ((_param_update_time + 5 * 1000) < hrt_absolute_time())) {

        param_t param = 0;

        // 只回显 used 且尚未持久化的参数；完整扫描结束后才清除本轮更新时间。
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
            // 传输暂不可用：保留索引，下一次 Run 重试同一 PARAM_VALUE。
            return false;
        }

        if (ret == 2) {
            // 取值失败也不能制造 count/index 空洞；任一声明索引缺失都会让 QGC 把整个
            // VehicleComponent 判为 missing，即使所有校准 Fact 实际已经到达。
            if (_last_read_failure_index !=
                    static_cast<int>(snapshot_index)) {
                PX4_ERR("param read failed, index %u retained",
                        snapshot_index);
                _last_read_failure_index = static_cast<int>(snapshot_index);
            }
            return false;
        }

        _last_read_failure_index = -1;
        ++_send_all_index;

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

    // Classic PARAM_VALUE 共用 float 载荷槽；INT32 按四字节位模式复制，而非数值转 float，
    // 与 MAV_PARAM_TYPE_INT32 的 bytewise encoding 合同一致。
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

    // 规范允许恰好 16 字节的 param_id 不含 NUL，接收方必须按定长字段处理。
    std::strncpy(msg.param_id, param_name(param),
                 MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

    // 参数类型来自生成的参数 core 描述符。
    param_type_t type = param_type(param);

    // 板端与 MAVLink 均为小端，可直接保持 INT32/float 的四字节编码。
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

} // namespace dima::modules::mavlink
