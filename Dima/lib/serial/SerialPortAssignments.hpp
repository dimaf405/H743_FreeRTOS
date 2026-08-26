#pragma once

#include <cstdint>

namespace dima::lib::serial {

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
