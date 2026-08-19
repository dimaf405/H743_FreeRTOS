#define MODULE_NAME "param"
#include "ParameterService.hpp"

#include "board_serial_config.hpp"
#include "logging/logging.hpp"
#include "parameters/flashparams/flashparams.h"
#include "platform/api/Time.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace dima::modules::parameters {
namespace {

constexpr const char *kQgcJournalIgnoredParameters[]{
    "SYS_AUTOSTART",
    "SYS_AUTOCONFIG",
    "MAV_SYS_ID",
    "CAL_GYRO0_ID",
    "CAL_ACC0_ID",
    "CAL_MAG0_ID",
    "CAL_MAG1_ID",
    "CAL_MAG2_ID",
    "NAV_RCL_ACT",
    "NAV_DLL_ACT",
    "COM_LOW_BAT_ACT",
};

constexpr std::size_t kSchema1SerialCount = 7U;
constexpr std::int32_t kSchema1BaudDefaults[kSchema1SerialCount]{
    0, 115200, 0, 115200, 0, 57600, 921600,
};
constexpr std::int32_t kSchema1FunctionDefaults[kSchema1SerialCount]{
    0, 0, 1, 0, 0, 0, 0,
};
constexpr std::int32_t kSchema1ToDirectSerial[kSchema1SerialCount]{
    2, 4, 6, 8, 3, 7, 1,
};

struct FilteredLoadContext {
    param_storage_visitor_t visitor;
    void *visitor_context;
    bool migrate_schema1_serial;
};

struct SerialStorageScanContext {
    std::int32_t schema_version{0};
    bool rc_port_present{false};
    std::int32_t rc_port{0};
    bool baud_present[8]{};
    std::int32_t baud[8]{};
    std::int32_t schema1_baud[kSchema1SerialCount]{};
    std::int32_t schema1_function[kSchema1SerialCount]{};

    SerialStorageScanContext() noexcept
    {
        std::copy_n(kSchema1BaudDefaults, kSchema1SerialCount,
                    schema1_baud);
        std::copy_n(kSchema1FunctionDefaults, kSchema1SerialCount,
                    schema1_function);
    }
};

int scan_schema1_serial_parameter(const char *name, param_type_t type,
                                  const void *value,
                                  SerialStorageScanContext &scan) noexcept
{
    for (std::size_t index = 0U; index < kSchema1SerialCount; ++index) {
        char baud_name[] = "SERIAL1_BAUD";
        char function_name[] = "SERIAL1_FUNCTION";
        baud_name[6] = static_cast<char>('1' + index);
        function_name[6] = static_cast<char>('1' + index);
        if (std::strcmp(name, baud_name) == 0) {
            if (type != PARAM_TYPE_INT32) return -EINVAL;
            std::memcpy(&scan.schema1_baud[index], value,
                        sizeof(scan.schema1_baud[index]));
            return 1;
        }
        if (std::strcmp(name, function_name) == 0) {
            if (type != PARAM_TYPE_INT32) return -EINVAL;
            std::memcpy(&scan.schema1_function[index], value,
                        sizeof(scan.schema1_function[index]));
            return 1;
        }
    }
    return 0;
}

bool is_qgc_compatibility_parameter(const char *name) noexcept
{
    if (name == nullptr) {
        return false;
    }
    for (const char *const fixed_name : kQgcJournalIgnoredParameters) {
        if (std::strcmp(name, fixed_name) == 0) {
            return true;
        }
    }
    return false;
}

bool is_disabled_mode_compatibility_parameter(const char *name) noexcept
{
    return name != nullptr && std::strcmp(name, "RC_MAP_FLTMODE") == 0;
}

int load_mutable_parameter(const char *name, param_type_t type,
                           const void *value, void *context) noexcept
{
    if (context == nullptr) {
        return -EINVAL;
    }
    auto &filtered = *static_cast<FilteredLoadContext *>(context);
    if (is_qgc_compatibility_parameter(name) ||
        is_disabled_mode_compatibility_parameter(name) ||
        std::strcmp(name, "RC_PORT_CONFIG") == 0) {
        return 0;
    }
    if (filtered.migrate_schema1_serial &&
        (dima::board::serial_baud_parameter(name) ||
         dima::board::serial_function_parameter(name))) {
        return 0;
    }

    return filtered.visitor(name, type, value, filtered.visitor_context);
}

int scan_serial_storage(const char *name, param_type_t type,
                        const void *value, void *context) noexcept
{
    if (name == nullptr || value == nullptr || context == nullptr) {
        return -EINVAL;
    }
    auto &scan = *static_cast<SerialStorageScanContext *>(context);
    if (std::strcmp(name, "DIMA_SER_VER") == 0) {
        if (type != PARAM_TYPE_INT32) {
            return -EINVAL;
        }
        std::memcpy(&scan.schema_version, value,
                    sizeof(scan.schema_version));
    } else if (std::strcmp(name, "RC_PORT_CONFIG") == 0) {
        if (type != PARAM_TYPE_INT32) {
            return -EINVAL;
        }
        std::memcpy(&scan.rc_port, value, sizeof(scan.rc_port));
        scan.rc_port_present = true;
    } else {
        const int schema1_serial =
            scan_schema1_serial_parameter(name, type, value, scan);
        if (schema1_serial < 0) return schema1_serial;
        const std::int32_t serial =
            dima::board::legacy_serial_for_baud_parameter(name);
        if (serial > 0) {
            if (type != PARAM_TYPE_INT32) {
                return -EINVAL;
            }
            std::memcpy(&scan.baud[serial - 1], value,
                        sizeof(scan.baud[serial - 1]));
            scan.baud_present[serial - 1] = true;
        }
    }
    return 0;
}

} // namespace

const param_storage_backend_s ParameterService::storage_backend_{
    &ParameterService::storage_load,
    &ParameterService::storage_save,
    &ParameterService::storage_erase,
    &ParameterService::storage_status,
};

ParameterService::ParameterService(
    dima::parameters::ParameterJournal &journal,
    dima::platform::ArmedFlashCoordinator &armed_flash,
    dima::platform::Synchronization &synchronization,
    dima::platform::CriticalSection &critical) noexcept
    : ScheduledWorkItem("param", px4::wq_configurations::lp_default),
      journal_(journal), armed_flash_(armed_flash),
      synchronization_(synchronization), critical_(critical),
      autosave_(armed_flash)
{
}

void ParameterService::lock_params(void *context) noexcept
{
    if (context != nullptr) {
        auto &self = *static_cast<ParameterService *>(context);
        (void)self.param_mutex_.lock();
    }
}

void ParameterService::unlock_params(void *context) noexcept
{
    if (context != nullptr) {
        static_cast<ParameterService *>(context)->param_mutex_.unlock();
    }
}

void ParameterService::notify_params(const parameter_update_s *source,
                                     void *context) noexcept
{
    if (source == nullptr || context == nullptr) {
        return;
    }
    auto &self = *static_cast<ParameterService *>(context);
    parameter_update_s update = *source;
    update.timestamp = hrt_absolute_time();

    dima::platform::CriticalGuard guard{self.critical_};
    self.pending_update_ = update;
    self.update_pending_ = true;
    if (!self.loading_) {
        self.autosave_request_pending_ = true;
    }
}

int ParameterService::storage_save(param_storage_enumerator_t enumerate,
                                   void *enumerate_context,
                                   void *backend_context) noexcept
{
    if (enumerate == nullptr || backend_context == nullptr) {
        return -EINVAL;
    }
    auto &self = *static_cast<ParameterService *>(backend_context);
    if (!self.flash_write_allowed()) {
        return -EPERM;
    }

    dima::platform::MutexGuard lock{self.storage_mutex_};
    if (!lock) {
        return -EDEADLK;
    }
    std::size_t payload_size{};
    const int encoded = flashparams_encode_buffer(
        self.payload_, sizeof(self.payload_), enumerate, enumerate_context,
        &payload_size);
    if (encoded != 0) {
        return encoded;
    }
    std::uint32_t sequence{};
    return self.journal_.append(self.payload_, payload_size, &sequence);
}

int ParameterService::storage_load(param_storage_visitor_t visitor,
                                   void *visitor_context,
                                   void *backend_context) noexcept
{
    if (visitor == nullptr || backend_context == nullptr) {
        return -EINVAL;
    }
    auto &self = *static_cast<ParameterService *>(backend_context);
    dima::platform::MutexGuard lock{self.storage_mutex_};
    if (!lock) {
        return -EDEADLK;
    }
    self.stored_serial_schema_version_ = 0;
    self.stored_legacy_rc_port_present_ = false;
    self.stored_legacy_rc_port_ = 0;
    std::memset(self.stored_legacy_baud_present_, 0,
                sizeof(self.stored_legacy_baud_present_));
    std::memset(self.stored_legacy_baud_, 0,
                sizeof(self.stored_legacy_baud_));
    std::size_t payload_size{};
    std::uint32_t sequence{};
    const int loaded = self.journal_.load(
        self.payload_, sizeof(self.payload_), &payload_size, &sequence);
    if (loaded != 0) {
        return loaded;
    }

    SerialStorageScanContext scan{};
    const int scanned = flashparams_decode_buffer(
        self.payload_, payload_size, scan_serial_storage, &scan);
    if (scanned != 0) {
        return scanned;
    }
    self.stored_serial_schema_version_ = scan.schema_version;
    self.stored_legacy_rc_port_present_ = scan.rc_port_present;
    self.stored_legacy_rc_port_ = scan.rc_port;
    std::memcpy(self.stored_legacy_baud_present_, scan.baud_present,
                sizeof(self.stored_legacy_baud_present_));
    std::memcpy(self.stored_legacy_baud_, scan.baud,
                sizeof(self.stored_legacy_baud_));
    std::memcpy(self.stored_schema1_baud_, scan.schema1_baud,
                sizeof(self.stored_schema1_baud_));
    std::memcpy(self.stored_schema1_function_, scan.schema1_function,
                sizeof(self.stored_schema1_function_));

    FilteredLoadContext filtered{
        visitor, visitor_context, scan.schema_version == 1};
    return flashparams_decode_buffer(self.payload_, payload_size,
                                     load_mutable_parameter, &filtered);
}

int ParameterService::storage_erase(void *backend_context) noexcept
{
    if (backend_context == nullptr) {
        return -EINVAL;
    }
    auto &self = *static_cast<ParameterService *>(backend_context);
    if (!self.flash_write_allowed()) {
        return -EPERM;
    }
    dima::platform::MutexGuard lock{self.storage_mutex_};
    return lock ? self.journal_.erase() : -EDEADLK;
}

int ParameterService::storage_status(param_storage_status_s *output,
                                     void *backend_context) noexcept
{
    if (output == nullptr || backend_context == nullptr) {
        return -EINVAL;
    }
    auto &self = *static_cast<ParameterService *>(backend_context);
    const dima::parameters::ParameterJournalStatus status =
        self.journal_.status();
    output->sequence = status.valid_sequence;
    output->used_bytes = static_cast<std::uint32_t>(status.used_bytes);
    output->free_bytes = static_cast<std::uint32_t>(status.free_bytes);
    output->crc_failures = status.crc_failures;
    output->write_failures = status.write_failures;
    output->enospc_failures = status.enospc_failures;
    output->last_save_timestamp = self.autosave_.lastAutosave();
    output->autosave_enabled = self.autosave_.enabled();
    return 0;
}

bool ParameterService::flash_write_allowed() const noexcept
{
    return !armed_flash_.armed();
}

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

bool ParameterService::init() noexcept
{
    if (initialized_) {
        return true;
    }

    reset_runtime_state();

    const auto fail_init = [this]() noexcept {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        (void)shutdown();
        return false;
    };

    if (!param_mutex_.initialize(synchronization_)) {
        return fail_init();
    }
    if (!storage_mutex_.initialize(synchronization_)) {
        return fail_init();
    }
    if (!journal_.initialize()) {
        return fail_init();
    }

    param_register_lock_callbacks(&ParameterService::lock_params,
                                  &ParameterService::unlock_params, this);
    param_register_notify_callback(&ParameterService::notify_params, this);
    if (param_register_storage_backend(&storage_backend_, this) != 0) {
        return fail_init();
    }

    param_init();
    if (!param_is_ready()) {
        return fail_init();
    }

    loading_ = true;
    const int loaded = param_load_default();
    loading_ = false;
    if (loaded != 0 && loaded != -ENOENT) {
        PX4_ERR("load failed: %d", loaded);
    }
    if ((loaded == 0 || loaded == -ENOENT) &&
        !migrate_serial_configuration(loaded == 0)) {
        return fail_init();
    }
    initialized_ = true;
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    return true;
}

bool ParameterService::shutdown() noexcept
{
    if (dima::platform::in_interrupt_context()) {
        return false;
    }

    stop();
    if (!param_shutdown()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    if (!journal_.shutdown()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    storage_mutex_.reset();
    param_mutex_.reset();
    initialized_ = false;
    reset_runtime_state();
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    return true;
}

bool ParameterService::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!initialized_ || !ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    autosave_.enable(true);
    if (!autosave_.enabled()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }
    param_register_notify_callback(&ParameterService::notify_params, this);
    for (param_t parameter = 0U; parameter < param_count(); ++parameter) {
        if (param_value_unsaved(parameter)) {
            param_notify_changes();
            break;
        }
    }
    if (!ScheduleOnInterval(kPollUs)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        param_register_notify_callback(nullptr, nullptr);
        autosave_.stop();
        ScheduleCancelAndDrain();
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void ParameterService::stop() noexcept
{
    param_register_notify_callback(nullptr, nullptr);
    autosave_.stop();
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    dima::platform::CriticalGuard guard{critical_};
    pending_update_ = {};
    update_pending_ = false;
    autosave_request_pending_ = false;
    loading_ = false;
}

dima::middleware::lifecycle::ModuleState ParameterService::state() const noexcept
{
    return state_;
}

void ParameterService::reset_runtime_state() noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    std::memset(payload_, 0, sizeof(payload_));
    pending_update_ = {};
    loading_ = false;
    update_pending_ = false;
    autosave_request_pending_ = false;
    stored_serial_schema_version_ = 0;
    stored_legacy_rc_port_present_ = false;
    stored_legacy_rc_port_ = 0;
    std::memset(stored_legacy_baud_present_, 0,
                sizeof(stored_legacy_baud_present_));
    std::memset(stored_legacy_baud_, 0, sizeof(stored_legacy_baud_));
    std::memset(stored_schema1_baud_, 0, sizeof(stored_schema1_baud_));
    std::memset(stored_schema1_function_, 0,
                sizeof(stored_schema1_function_));
}

void ParameterService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    parameter_update_s update{};
    bool publish_update = false;
    bool request_autosave = false;
    {
        dima::platform::CriticalGuard guard{critical_};
        if (update_pending_) {
            update = pending_update_;
            update_pending_ = false;
            publish_update = true;
        }
        request_autosave = autosave_request_pending_;
        autosave_request_pending_ = false;
    }
    if (publish_update) {
        (void)parameter_update_pub_.publish(update);
    }
    if (request_autosave) {
        autosave_.request();
    }
}

} // namespace dima::modules::parameters
