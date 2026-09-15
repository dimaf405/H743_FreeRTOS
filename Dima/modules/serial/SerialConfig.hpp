#pragma once

#include "serial/SerialPortAssignments.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "api/Serial.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::serial {

/**
 * 从 PX4 生成的参数注册表发现 SERIALx 参数，并解析为唯一的 SBUS/GPS/MAVLink
 * 端口所有权。参数条目只由 module_serial.yaml 定义，本类不维护参数名或端口
 * 参数清单。
 *
 * 串口映射合同（对齐 ArduPilot/PX4）：SERIALx_BAUD/FUNCTION 标记
 * reboot_required，写入时只做参数层原子迁移与校验，实际端口接管、波特率
 * 应用与 Auto 扫描全部发生在启动时；本模块不提供运行期热重配。
 */
class SerialConfig final : public dima::middleware::lifecycle::ModuleBase,
                           public dima::lib::serial::SerialPortAssignments {
public:
    explicit SerialConfig(dima::platform::SerialPorts &backend) noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

    std::int32_t telemetry_port() const noexcept override;
    std::uint32_t telemetry_baudrate() const noexcept override;
    std::int32_t rc_input_port() const noexcept override;
    std::int32_t gps_port() const noexcept override;
    std::uint32_t gps_target_baudrate() const noexcept override;

private:
    // 本板最大物理编号为 UART8；SERIAL5 不存在但编号槽不能压缩，否则 USART6
    // 会被错误映射为 SERIAL5。槽位只承载运行时状态，不代表存在 UART5 参数或硬件。
    static constexpr std::size_t kPortCount = 8U;

    struct ParameterBinding {
        param_t baud{PARAM_INVALID};
        param_t function{PARAM_INVALID};
    };

    struct Configuration {
        std::uint32_t baudrate[kPortCount]{};
        std::int32_t telemetry_port{0};
        std::uint32_t telemetry_baudrate{0U};
        bool telemetry_valid{true};
        std::int32_t rc_input_port{0};
        std::int32_t gps_port{0};
        std::uint32_t gps_target_baudrate{0U};
    };

    bool bind_parameters() noexcept;
    void invalidate_parameters() noexcept;
    bool read_configuration(Configuration &configuration) const noexcept;
    bool apply_baudrates(const std::uint32_t *baudrates) noexcept;
    void commit_configuration(const Configuration &configuration) noexcept;

    dima::platform::SerialPorts &backend_;

    ParameterBinding serial_parameters_[kPortCount]{};

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    std::int32_t telemetry_port_{0};
    std::uint32_t telemetry_baudrate_{0U};
    std::int32_t rc_input_port_{0};
    std::int32_t gps_port_{0};
    std::uint32_t gps_target_baudrate_{0U};
};

} // namespace dima::modules::serial
