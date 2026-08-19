#define MODULE_NAME "serial"
#include "SerialConfig.hpp"

#include "logging/logging.hpp"

namespace dima::modules::serial {

SerialConfig::SerialConfig(dima::platform::SerialPorts &backend) noexcept
    : ModuleParams(nullptr), backend_(backend)
{
}

bool SerialConfig::bind_parameters() noexcept
{
    bool bound = true;
#define DIMA_BIND_SERIAL_PARAMETERS(index, baud, function) \
    bound = serial##index##_baud_.bind() && \
            serial##index##_function_.bind() && bound;
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_BIND_SERIAL_PARAMETERS)
#undef DIMA_BIND_SERIAL_PARAMETERS
    return bound;
}

void SerialConfig::invalidate_parameters() noexcept
{
#define DIMA_INVALIDATE_SERIAL_PARAMETERS(index, baud, function) \
    serial##index##_baud_.invalidate(); \
    serial##index##_function_.invalidate();
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_INVALIDATE_SERIAL_PARAMETERS)
#undef DIMA_INVALIDATE_SERIAL_PARAMETERS
}

bool SerialConfig::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }

    rc_input_port_ = 0;
    if (!bind_parameters()) {
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("board serial parameters unavailable");
        return false;
    }

    bool configuration_valid = true;
    unsigned rc_owner_count = 0U;
#define DIMA_VALIDATE_SERIAL_PARAMETERS(index, baud, function) \
    do { \
        const std::int32_t baud_value = serial##index##_baud_.get(); \
        const std::int32_t function_value = serial##index##_function_.get(); \
        if (baud_value < 0 || \
            !dima::board::serial_baud_supported( \
                static_cast<std::uint32_t>(baud_value)) || \
            !dima::board::serial_function_supported(function_value)) { \
            configuration_valid = false; \
        } \
        if (function_value == dima::board::kSerialFunctionRcInput) { \
            ++rc_owner_count; \
            rc_input_port_ = index; \
        } \
    } while (false);
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_VALIDATE_SERIAL_PARAMETERS)
#undef DIMA_VALIDATE_SERIAL_PARAMETERS

    if (!configuration_valid || rc_owner_count > 1U) {
        rc_input_port_ = 0;
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("invalid serial configuration rc_owners=%u", rc_owner_count);
        return false;
    }

    bool configured = true;
#define DIMA_APPLY_SERIAL_BAUD(index, baud, function) \
    configured = backend_.configure_normal_baud( \
        index, static_cast<std::uint32_t>(serial##index##_baud_.get())) && \
        configured;
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_APPLY_SERIAL_BAUD)
#undef DIMA_APPLY_SERIAL_BAUD

    if (!configured) {
        rc_input_port_ = 0;
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("normal serial configuration failed");
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    PX4_INFO("configured SERIAL1..SERIAL8 rc_port=%ld",
             static_cast<long>(rc_input_port_));
    return true;
}

void SerialConfig::stop() noexcept
{
    invalidate_parameters();
    rc_input_port_ = 0;
    state_ = backend_.reset_normal_configuration()
                 ? dima::middleware::lifecycle::ModuleState::Stopped
                 : dima::middleware::lifecycle::ModuleState::Error;
}

dima::middleware::lifecycle::ModuleState SerialConfig::state() const noexcept
{
    return state_;
}

std::int32_t SerialConfig::rc_input_port() const noexcept
{
    return state_ == dima::middleware::lifecycle::ModuleState::Running
               ? rc_input_port_
               : 0;
}

} // namespace dima::modules::serial
