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

bool serial_function_supported(
    std::int32_t function) noexcept;

SerialParameterIdentity identify_serial_parameter(
    const char *name) noexcept;

bool serial_baud_parameter(const char *name) noexcept;

bool serial_function_parameter(const char *name) noexcept;

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
