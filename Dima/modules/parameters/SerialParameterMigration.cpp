#define MODULE_NAME "param"
#include "ParameterService.hpp"
#include "SerialMigrationSchema.hpp"

#include "board_serial_config.hpp"
#include "logging/logging.hpp"

namespace dima::modules::parameters {

bool ParameterService::migrate_serial_schema_v1() noexcept
{
    unsigned rc_owner_count = 0U;
    for (std::size_t index = 0U; index < kSchema1SerialCount; ++index) {
        const std::int32_t baud = stored_schema1_baud_[index];
        const std::int32_t function = stored_schema1_function_[index];
        if (baud < 0 || !dima::board::serial_baud_supported(
                static_cast<std::uint32_t>(baud)) ||
            !dima::board::serial_function_supported(function)) {
            PX4_ERR("invalid serial schema v1 value index=%u baud=%ld function=%ld",
                    static_cast<unsigned>(index + 1U),
                    static_cast<long>(baud), static_cast<long>(function));
            return false;
        }
        if (function == dima::board::kSerialFunctionRcInput) {
            ++rc_owner_count;
        }
    }
    if (rc_owner_count > 1U) {
        PX4_ERR("invalid serial schema v1 rc_owners=%u", rc_owner_count);
        return false;
    }

    for (std::size_t index = 0U; index < kSchema1SerialCount; ++index) {
        const std::int32_t direct_serial = kSchema1ToDirectSerial[index];
        const dima::board::SerialPortDescriptor *const descriptor =
            dima::board::serial_port(direct_serial);
        if (descriptor == nullptr) return false;

        const param_t baud = param_find_no_notification(
            descriptor->parameter_name);
        const param_t function = param_find_no_notification(
            descriptor->function_parameter_name);
        if (baud == PARAM_INVALID || function == PARAM_INVALID ||
            param_set_no_notification(
                baud, &stored_schema1_baud_[index]) != 0 ||
            param_set_no_notification(
                function, &stored_schema1_function_[index]) != 0) {
            return false;
        }
    }
    return true;
}

bool ParameterService::migrate_serial_configuration(
    bool existing_storage) noexcept
{
    const std::int32_t current_version = static_cast<std::int32_t>(
        dima::board::kSerialSchemaVersion);
    if (stored_serial_schema_version_ < 0 ||
        stored_serial_schema_version_ > current_version) {
        PX4_ERR("unsupported serial schema=%ld current=%ld",
                static_cast<long>(stored_serial_schema_version_),
                static_cast<long>(current_version));
        return false;
    }
    if (stored_serial_schema_version_ == current_version) {
        return true;
    }

    const auto store_current_version = [current_version]() noexcept {
        const param_t version = param_find_no_notification("DIMA_SER_VER");
        return version != PARAM_INVALID &&
               param_set_no_notification(version, &current_version) == 0;
    };

    if (stored_serial_schema_version_ == 1) {
        if (!migrate_serial_schema_v1() || !store_current_version()) {
            return false;
        }
        PX4_INFO("serial config migrated schema=1->%ld by physical UART",
                 static_cast<long>(current_version));
        return true;
    }

    std::int32_t selected_port = existing_storage
        ? dima::board::migrate_legacy_rc_port(1)
        : dima::board::kDefaultRcPort;
    if (stored_legacy_rc_port_present_) {
        selected_port = dima::board::migrate_legacy_rc_port(
            stored_legacy_rc_port_);
        if (selected_port != 0 &&
            !dima::board::serial_port_supported(selected_port)) {
            selected_port = 0;
        }
    }
    for (const dima::board::SerialPortDescriptor &descriptor :
         dima::board::kSerialPorts) {
        const param_t function = param_find_no_notification(
            descriptor.function_parameter_name);
        const std::int32_t value =
            descriptor.port_id == selected_port
                ? dima::board::kSerialFunctionRcInput
                : dima::board::kSerialFunctionDisabled;
        if (function == PARAM_INVALID ||
            param_set_no_notification(function, &value) != 0) {
            return false;
        }

        const std::size_t serial_index =
            static_cast<std::size_t>(descriptor.port_id - 1);
        if (stored_legacy_baud_present_[serial_index]) {
            const std::int32_t baud_value = stored_legacy_baud_[serial_index];
            if (baud_value < 0 || !dima::board::serial_baud_supported(
                    static_cast<std::uint32_t>(baud_value))) {
                PX4_WARN("discarding invalid legacy SERIAL%ld baud=%ld",
                         static_cast<long>(descriptor.port_id),
                         static_cast<long>(baud_value));
            } else {
                const param_t baud = param_find_no_notification(
                    descriptor.parameter_name);
                if (baud == PARAM_INVALID ||
                    param_set_no_notification(baud, &baud_value) != 0) {
                    return false;
                }
            }
        }
    }

    if (!store_current_version()) {
        return false;
    }

    PX4_INFO("serial config migrated legacy_port=%ld selected_serial=%ld",
             static_cast<long>(stored_legacy_rc_port_present_
                                   ? stored_legacy_rc_port_
                                   : -1),
             static_cast<long>(selected_port));
    return true;
}

} // namespace dima::modules::parameters
