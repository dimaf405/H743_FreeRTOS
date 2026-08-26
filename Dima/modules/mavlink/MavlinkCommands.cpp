#define MODULE_NAME "mavlink"
#include "MavlinkCommands.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"

#include <cmath>
#include <cstring>

namespace dima::modules::mavlink {
namespace {

float command_parameter(std::uint32_t raw) noexcept
{
    // vehicle_command 以 uint32 保存 COMMAND_* 浮点参数的原始 IEEE-754 位模式；
    // 必须用 memcpy 还原，数值 static_cast 会把位模式本身当作整数再转换。
    float value = 0.0F;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

bool message_id_from_float(float value, std::uint16_t &message_id) noexcept
{
    // MAV_CMD 把 message id 放在 float 参数中：先拒绝 NaN/Inf 和协议范围外值，
    // 再按 MAVLink 常用整数参数语义四舍五入，并在写窄类型前二次核对边界。
    if (!std::isfinite(value) || value < 0.0F ||
        value > static_cast<float>(UINT16_MAX)) return false;
    const long rounded = std::lround(value);
    if (rounded < 0L || rounded > static_cast<long>(UINT16_MAX)) return false;
    message_id = static_cast<std::uint16_t>(rounded);
    return true;
}

} // namespace

MavlinkCommands::MavlinkCommands(RequestMessageFn request_message,
                                 SetMessageIntervalFn set_message_interval,
                                 GetMessageIntervalFn get_message_interval,
                                 void *callback_ctx) noexcept
    : request_message_(request_message),
      set_message_interval_(set_message_interval),
      get_message_interval_(get_message_interval),
      callback_ctx_(callback_ctx)
{
}

void MavlinkCommands::handle_message(const mavlink_message_t *msg) noexcept
{
    // 上层生成的 inbound 路由只会把声明过的 COMMAND 消息交给本对象；这里仍按
    // wire 类型分流，避免 COMMAND_LONG 与 COMMAND_INT 的坐标编码规则相互污染。
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_message_command_long(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_INT:
        handle_message_command_int(msg);
        break;

    default:
        break;
    }
}

bool MavlinkCommands::evaluate_target_ok(
    std::uint16_t command, std::uint8_t target_system,
    std::uint8_t target_component) const noexcept
{
    // 能力/协议版本查询允许广播且忽略 component，便于地面站发现飞控；其余命令
    // 必须精确命中本 system，并只接受本 component 或 MAV_COMP_ID_ALL。
    bool target_ok = false;

    switch (command) {

    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
    case MAV_CMD_REQUEST_PROTOCOL_VERSION:
        /* broadcast and ignore component */
        target_ok = (target_system == 0) ||
                    (target_system == MAVLINK_SYSTEM_ID);
        break;

    default:
        target_ok = (target_system == MAVLINK_SYSTEM_ID) &&
                    ((target_component == MAVLINK_COMPONENT_ID) ||
                     (target_component == MAV_COMP_ID_ALL));
        break;
    }

    return target_ok;
}

void MavlinkCommands::acknowledge(std::uint8_t sysid, std::uint8_t compid,
                                  std::uint16_t command, std::uint8_t result,
                                  std::uint8_t result_param2) noexcept
{
    // ACK 先发布到 uORB，由 MAVLink 输出侧统一编码；target 指回原始发送端，
    // from_external 保留跨安全边界的来源属性。
    vehicle_command_ack_s command_ack{};

    command_ack.timestamp = hrt_absolute_time();
    command_ack.command = command;
    command_ack.result = result;
    command_ack.from_external = true;
    command_ack.target_system = sysid;
    command_ack.target_component = compid;
    command_ack.result_param2 = result_param2;

    _cmd_ack_pub.publish(command_ack);
}

std::uint8_t MavlinkCommands::handle_request_message_command(
    std::uint16_t message_id) noexcept
{
    // 可请求消息集合与速率上限由生成合同及 MavlinkService 持有，本层只转交，
    // 避免在命令解析器中再维护一份 message-id 清单。
    if (request_message_ != nullptr) {
        return request_message_(callback_ctx_, message_id);
    }
    return vehicle_command_ack_s::RESULT_UNSUPPORTED;
}

void MavlinkCommands::handle_message_command_long(
    const mavlink_message_t *msg) noexcept
{
    mavlink_command_long_t cmd_mavlink;
    mavlink_msg_command_long_decode(msg, &cmd_mavlink);

    vehicle_command_s vcmd{};
    vcmd.timestamp = hrt_absolute_time();

    const float before_int32_max = std::nextafter((float)INT32_MAX, 0.0f);
    const float after_int32_max =
        std::nextafter((float)INT32_MAX, (float)INFINITY);

    // COMMAND_LONG 的 param5/6 本应直接是浮点坐标。部分发送端误把 COMMAND_INT
    // 的 INT32_MAX 哨兵塞入 float；由于 2^31-1 会在 float 中舍入，使用相邻
    // 可表示值包住该点，并且仅在 x/y 两项同时命中时拒绝这类错误编码。
    if (cmd_mavlink.param5 >= before_int32_max &&
        cmd_mavlink.param5 <= after_int32_max &&
        cmd_mavlink.param6 >= before_int32_max &&
        cmd_mavlink.param6 <= after_int32_max) {
        PX4_ERR("param5/param6 invalid of command %u", cmd_mavlink.command);
        acknowledge(msg->sysid, msg->compid, cmd_mavlink.command,
                    vehicle_command_ack_s::RESULT_DENIED);
        return;
    }

    /* Dima 的 vehicle_command 合同保存 param1..7 的 IEEE-754 原始位模式，使 NaN
     * 哨兵和 -0 等值不在 MAVLink -> Commander 边界被数值转换规范化。 */
    std::memcpy(&vcmd.param1_raw, &cmd_mavlink.param1, sizeof(float));
    std::memcpy(&vcmd.param2_raw, &cmd_mavlink.param2, sizeof(float));
    std::memcpy(&vcmd.param3_raw, &cmd_mavlink.param3, sizeof(float));
    std::memcpy(&vcmd.param4_raw, &cmd_mavlink.param4, sizeof(float));
    std::memcpy(&vcmd.param5_raw, &cmd_mavlink.param5, sizeof(float));
    std::memcpy(&vcmd.param6_raw, &cmd_mavlink.param6, sizeof(float));
    std::memcpy(&vcmd.param7_raw, &cmd_mavlink.param7, sizeof(float));
    vcmd.command = cmd_mavlink.command;
    vcmd.target_system = cmd_mavlink.target_system;
    vcmd.target_component = cmd_mavlink.target_component;
    vcmd.source_system = msg->sysid;
    vcmd.source_component = msg->compid;
    vcmd.confirmation = cmd_mavlink.confirmation;
    vcmd.from_external = true;

    handle_message_command_both(msg, cmd_mavlink.command,
                                cmd_mavlink.target_system,
                                cmd_mavlink.target_component,
                                cmd_mavlink.param1, vcmd);
}

void MavlinkCommands::handle_message_command_int(
    const mavlink_message_t *msg) noexcept
{
    mavlink_command_int_t cmd_mavlink;
    mavlink_msg_command_int_decode(msg, &cmd_mavlink);

    vehicle_command_s vcmd{};
    vcmd.timestamp = hrt_absolute_time();

    // 0x7ff80000 是常见 quiet-NaN 的 float 位模式；若它被错误地按 int32 写入
    // COMMAND_INT 的 x/y，两项会同时出现该整数值，必须拒绝而非当作经纬度。
    if (cmd_mavlink.x == 0x7ff80000 && cmd_mavlink.y == 0x7ff80000) {
        PX4_ERR("x/y invalid of command %u", cmd_mavlink.command);
        acknowledge(msg->sysid, msg->compid, cmd_mavlink.command,
                    vehicle_command_ack_s::RESULT_DENIED);
        return;
    }

    // param1..4、z 原本就是 float，继续保留其原始位模式；x/y 在下方按
    // COMMAND_INT 的定点坐标合同转换为内部 float。
    std::memcpy(&vcmd.param1_raw, &cmd_mavlink.param1, sizeof(float));
    std::memcpy(&vcmd.param2_raw, &cmd_mavlink.param2, sizeof(float));
    std::memcpy(&vcmd.param3_raw, &cmd_mavlink.param3, sizeof(float));
    std::memcpy(&vcmd.param4_raw, &cmd_mavlink.param4, sizeof(float));

    float param5;
    float param6;
    if (cmd_mavlink.x == INT32_MAX && cmd_mavlink.y == INT32_MAX) {
        // MAVLink 以 x=y=INT32_MAX 表示“不提供位置”；内部继续用 NaN 表达缺省，
        // Commander 因而不会把哨兵整数误当作 214.7483647 度。
        param5 = NAN;
        param6 = NAN;
    } else {
        // COMMAND_INT 的 x/y 单位为 1e-7 度：degrees=int32/1e7。先用 double 做
        // 除法以减少中间舍入，最后再按 Dima 的 float 位模式合同收窄。
        param5 = static_cast<float>(
            static_cast<double>(cmd_mavlink.x) / 1e7);
        param6 = static_cast<float>(
            static_cast<double>(cmd_mavlink.y) / 1e7);
    }
    std::memcpy(&vcmd.param5_raw, &param5, sizeof(float));
    std::memcpy(&vcmd.param6_raw, &param6, sizeof(float));
    std::memcpy(&vcmd.param7_raw, &cmd_mavlink.z, sizeof(float));

    vcmd.command = cmd_mavlink.command;
    vcmd.target_system = cmd_mavlink.target_system;
    vcmd.target_component = cmd_mavlink.target_component;
    vcmd.source_system = msg->sysid;
    vcmd.source_component = msg->compid;
    vcmd.confirmation = false;
    vcmd.from_external = true;

    handle_message_command_both(msg, cmd_mavlink.command,
                                cmd_mavlink.target_system,
                                cmd_mavlink.target_component,
                                cmd_mavlink.param1, vcmd);
}

void MavlinkCommands::handle_message_command_both(
    const mavlink_message_t *msg, std::uint16_t command,
    std::uint8_t target_system, std::uint8_t target_component, float param1,
    const vehicle_command_s &vehicle_command) noexcept
{
    // 目标过滤必须先于任何响应或 uORB 发布；发给其他飞控的广播链路流量应完全
    // 静默，不能产生误导发送端的 ACK。
    bool target_ok = evaluate_target_ok(command, target_system,
                                        target_component);
    bool send_ack = true;
    std::uint8_t result = vehicle_command_ack_s::RESULT_ACCEPTED;

    if (!target_ok) {
        PX4_INFO("Ignore command %d from %d/%d to %d/%d",
                 command, msg->sysid, msg->compid,
                 target_system, target_component);
        return;
    }

    // 旧版能力/协议查询先映射到通用 request callback，与 REQUEST_MESSAGE 共用
    // 生成的可发送消息合同，不保留第二套路由表。
    if (command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
        result = handle_request_message_command(
            MAVLINK_MSG_ID_AUTOPILOT_VERSION);

    } else if (command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
        result = handle_request_message_command(MAVLINK_MSG_ID_PROTOCOL_VERSION);

    } else if (command == MAV_CMD_SET_MESSAGE_INTERVAL) {
        // param1 是目标 message id；param2/3/4/7 的原始位模式分别承载 interval
        // 及扩展参数，具体限频和支持性由生成 stream 合同的回调判定。
        std::uint16_t message_id = 0U;
        if (!message_id_from_float(param1, message_id) ||
            set_message_interval_ == nullptr) {
            result = vehicle_command_ack_s::RESULT_FAILED;
        } else {
            result = set_message_interval_(
                callback_ctx_, message_id,
                command_parameter(vehicle_command.param2_raw),
                command_parameter(vehicle_command.param3_raw),
                command_parameter(vehicle_command.param4_raw),
                command_parameter(vehicle_command.param7_raw));
        }

    } else if (command == MAV_CMD_GET_MESSAGE_INTERVAL) {
        std::uint16_t message_id = 0U;
        if (!message_id_from_float(param1, message_id) ||
            get_message_interval_ == nullptr) {
            result = vehicle_command_ack_s::RESULT_FAILED;
        } else {
            result = get_message_interval_(callback_ctx_, message_id);
        }

    } else if (command == MAV_CMD_REQUEST_MESSAGE) {
        std::uint16_t message_id = 0U;
        if (!message_id_from_float(param1, message_id)) {
            result = vehicle_command_ack_s::RESULT_FAILED;
        } else if (message_id == MAVLINK_MSG_ID_MESSAGE_INTERVAL) {
            // 请求 MESSAGE_INTERVAL 时，param2 才是要查询的实际 message id；
            // 查询回调会先发送响应消息，再由本函数发送命令 ACK。
            std::uint16_t requested_message_id = 0U;
            const float requested = command_parameter(
                vehicle_command.param2_raw);
            if (!message_id_from_float(requested, requested_message_id) ||
                get_message_interval_ == nullptr) {
                result = vehicle_command_ack_s::RESULT_FAILED;
            } else {
                result = get_message_interval_(callback_ctx_,
                                               requested_message_id);
            }
        } else {
            result = handle_request_message_command(message_id);
        }

    } else {
        // 普通飞控命令进入 Commander 做唯一安全仲裁和唯一 ACK；本层若抢先确认，
        // 会把“已接收”误报成“已安全执行”。本机同 sys/comp 的回环命令也不转发。
        send_ack = false;

        if (msg->sysid == MAVLINK_SYSTEM_ID &&
            msg->compid == MAVLINK_COMPONENT_ID) {
            PX4_WARN("ignoring CMD with same SYS/COMP (%d/%d) ID",
                     MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID);
            return;
        }

        _cmd_pub.publish(vehicle_command);
    }

    if (send_ack) {
        acknowledge(msg->sysid, msg->compid, command, result);
    }
}

} // namespace dima::modules::mavlink
