#include "SerialPortAssignments.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::lib::serial {

bool serial_function_supported(
    std::int32_t function) noexcept
{
    return function >= kSerialFunctionDisabled &&
           function <= kSerialFunctionGps;
}

SerialParameterIdentity identify_serial_parameter(
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

bool serial_baud_parameter(const char *name) noexcept
{
    return identify_serial_parameter(name).kind == SerialParameterKind::Baud;
}

bool serial_function_parameter(const char *name) noexcept
{
    return identify_serial_parameter(name).kind ==
           SerialParameterKind::Function;
}

} // namespace dima::lib::serial
