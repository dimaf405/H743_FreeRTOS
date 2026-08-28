#pragma once

#include <cstdint>
#include <cstring>

namespace dima::lib::serial {

enum class SerialParameterKind : std::uint8_t {
    None,
    Baud,
    Function,
};

struct SerialParameterIdentity {
    std::uint32_t port{0U};
    SerialParameterKind kind{SerialParameterKind::None};
};

// SERIALx_FUNCTION 的数值语义是运行时协议合同；参数条目、默认值和 QGC
// 枚举文本仍只在 module_serial.yaml 中定义并由 PX4 工具生成。
inline constexpr std::int32_t kSerialFunctionDisabled = 0;
inline constexpr std::int32_t kSerialFunctionSbus = 1;
inline constexpr std::int32_t kSerialFunctionGps = 2;

inline constexpr bool serial_function_supported(
    std::int32_t function) noexcept
{
    return function >= kSerialFunctionDisabled &&
           function <= kSerialFunctionGps;
}

inline SerialParameterIdentity identify_serial_parameter(
    const char *name) noexcept
{
    constexpr char kPrefix[] = "SERIAL";
    if (name == nullptr || std::strncmp(name, kPrefix, sizeof(kPrefix) - 1U) != 0) {
        return {};
    }

    const char *cursor = name + sizeof(kPrefix) - 1U;
    if (*cursor < '1' || *cursor > '9') {
        return {};
    }

    // 从 PX4 生成的参数注册表按命名规则发现端口，不在源码维护 SERIAL1/2/... 清单。
    std::uint32_t port = 0U;
    do {
        const std::uint32_t digit = static_cast<std::uint32_t>(*cursor - '0');
        if (port > (UINT32_MAX - digit) / 10U) {
            return {};
        }
        port = port * 10U + digit;
        ++cursor;
    } while (*cursor >= '0' && *cursor <= '9');

    if (std::strcmp(cursor, "_BAUD") == 0) {
        return {port, SerialParameterKind::Baud};
    }
    if (std::strcmp(cursor, "_FUNCTION") == 0) {
        return {port, SerialParameterKind::Function};
    }
    return {};
}

inline bool serial_baud_parameter(const char *name) noexcept
{
    return identify_serial_parameter(name).kind == SerialParameterKind::Baud;
}

inline bool serial_function_parameter(const char *name) noexcept
{
    return identify_serial_parameter(name).kind ==
           SerialParameterKind::Function;
}

/**
 * 参数服务选出的只读产品串口所有权视图。
 * 消费者只能读取已经验证并提交的 RC/GPS 端口，不能各自重新解析参数形成冲突所有者。
 */
class SerialPortAssignments {
public:
    virtual ~SerialPortAssignments() = default;
    virtual std::int32_t rc_input_port() const noexcept = 0;
    virtual std::int32_t gps_port() const noexcept = 0;
    virtual std::uint32_t gps_target_baudrate() const noexcept = 0;
};

} // namespace dima::lib::serial
