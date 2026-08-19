#pragma once

#include "board_serial_config.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/module_params.h"
#include "platform/api/Platform.hpp"

#include <cstdint>

namespace dima::modules::serial {

class SerialConfig final : public dima::middleware::lifecycle::ModuleBase,
                           public ModuleParams {
public:
    explicit SerialConfig(dima::platform::SerialPorts &backend) noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

    std::int32_t rc_input_port() const noexcept;

private:
    bool bind_parameters() noexcept;
    void invalidate_parameters() noexcept;

    dima::platform::SerialPorts &backend_;

#define DIMA_DECLARE_SERIAL_PARAMETERS(index, baud, function) \
    px4::ParamInt<px4::params::baud> serial##index##_baud_{}; \
    px4::ParamInt<px4::params::function> serial##index##_function_{};
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_DECLARE_SERIAL_PARAMETERS)
#undef DIMA_DECLARE_SERIAL_PARAMETERS

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    std::int32_t rc_input_port_{0};
};

} // namespace dima::modules::serial
