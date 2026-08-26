#pragma once

#include "SerialContract.hpp"
#include "serial/SerialPortAssignments.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "api/Serial.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::serial {

/**
 * 将生成的板级串口参数合同解析为唯一的 RC/GPS 端口所有权，并原子切换后端波特率。
 * 参数成员由 DIMA_BOARD_SERIAL_PARAMETER_LIST 展开，禁止在此类中手写端口清单。
 */
class SerialConfig final : public dima::middleware::lifecycle::ModuleBase,
                           public dima::lib::serial::SerialPortAssignments {
public:
    explicit SerialConfig(dima::platform::SerialPorts &backend) noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;
    bool pending_configuration_valid() const noexcept;
    bool reconfigure() noexcept;

    std::int32_t rc_input_port() const noexcept override;
    std::int32_t gps_port() const noexcept override;
    std::uint32_t gps_target_baudrate() const noexcept override;
    std::uint64_t configuration_signature() const noexcept;

private:
    static constexpr std::size_t kPortCount =
        sizeof(dima::board::kSerialPorts) /
        sizeof(dima::board::kSerialPorts[0]);

    struct Configuration {
        std::uint32_t baudrate[kPortCount]{};
        std::int32_t rc_input_port{0};
        std::int32_t gps_port{0};
        std::uint32_t gps_target_baudrate{0U};
        std::int32_t gps_protocol{0};
    };

    bool bind_parameters() noexcept;
    void invalidate_parameters() noexcept;
    bool read_configuration(Configuration &configuration) const noexcept;
    bool apply_baudrates(const std::uint32_t *baudrates) noexcept;
    void commit_configuration(const Configuration &configuration) noexcept;

    dima::platform::SerialPorts &backend_;

    px4::ParamInt<px4::params::DIMA_PRIMARY_GPS_PORT_PARAMETER>
        gps1_config_{};
    px4::ParamInt<px4::params::GPS_1_PROTOCOL> gps1_protocol_{};

#define DIMA_DECLARE_SERIAL_PARAMETERS(index, baud, function) \
    px4::ParamInt<px4::params::baud> serial##index##_baud_{}; \
    px4::ParamInt<px4::params::function> serial##index##_function_{};
    // 参数声明与板级 serial_ports.json 的生成结果保持同源。
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_DECLARE_SERIAL_PARAMETERS)
#undef DIMA_DECLARE_SERIAL_PARAMETERS

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    std::int32_t rc_input_port_{0};
    std::int32_t gps_port_{0};
    std::uint32_t gps_target_baudrate_{0U};
    std::uint32_t applied_baudrates_[kPortCount]{};
};

} // namespace dima::modules::serial
