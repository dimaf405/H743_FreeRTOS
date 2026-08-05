#define MODULE_NAME "param"
#include "ParameterService.hpp"

#include "logging/logging.hpp"
#include "parameters/flashparams/flashparams.h"
#include "platform/api/Time.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace dima::modules::parameters {
namespace {

constexpr char kIdentifyResponse[] = "DIMA_ROVER_APP_V1\n";
constexpr char kRebootResponse[] = "DIMA_REBOOTING_BOOTLOADER\n";
constexpr char kRebootDeniedResponse[] = "DIMA_REBOOT_DENIED_ARMED\n";

bool match_prefix(const char *name, const char *pattern) noexcept
{
    if (pattern == nullptr || *pattern == '\0') {
        return true;
    }
    const std::size_t length = std::strlen(pattern);
    const std::size_t prefix =
        pattern[length - 1U] == '*' ? length - 1U : length;
    return std::strncmp(name, pattern, prefix) == 0 &&
           (prefix != length || name[prefix] == '\0');
}

void print_param(param_t parameter) noexcept
{
    if (param_type(parameter) == PARAM_TYPE_FLOAT) {
        float value{};
        (void)param_get(parameter, &value);
        PX4_INFO_RAW("%s %.9g\n", param_name(parameter),
                     static_cast<double>(value));
    } else {
        std::int32_t value{};
        (void)param_get(parameter, &value);
        PX4_INFO_RAW("%s %ld\n", param_name(parameter),
                     static_cast<long>(value));
    }
}

bool parse_value(param_t parameter, const char *text,
                 param_value_u &value) noexcept
{
    if (text == nullptr) {
        return false;
    }
    char *end{};
    errno = 0;
    if (param_type(parameter) == PARAM_TYPE_FLOAT) {
        value.f = std::strtof(text, &end);
        return errno == 0 && end != text && *end == '\0' &&
               std::isfinite(value.f);
    }
    const long parsed = std::strtol(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed < INT32_MIN ||
        parsed > INT32_MAX) {
        return false;
    }
    value.i = static_cast<std::int32_t>(parsed);
    return true;
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
    dima::platform::Console &console,
    dima::platform::BootControl &boot_control,
    dima::platform::ArmedFlashCoordinator &armed_flash,
    dima::platform::Synchronization &synchronization,
    dima::platform::CriticalSection &critical) noexcept
    : ScheduledWorkItem("param", px4::wq_configurations::lp_default),
      journal_(journal), console_(console), boot_control_(boot_control),
      armed_flash_(armed_flash), synchronization_(synchronization),
      critical_(critical), autosave_(armed_flash)
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
    std::size_t payload_size{};
    std::uint32_t sequence{};
    const int loaded = self.journal_.load(
        self.payload_, sizeof(self.payload_), &payload_size, &sequence);
    return loaded == 0
               ? flashparams_decode_buffer(self.payload_, payload_size,
                                           visitor, visitor_context)
               : loaded;
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
    line_length_ = 0U;
    discard_line_ = false;
    ignore_lf_ = false;
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
    std::memset(line_, 0, sizeof(line_));
    line_length_ = 0U;
    loading_ = false;
    update_pending_ = false;
    autosave_request_pending_ = false;
    discard_line_ = false;
    ignore_lf_ = false;
}

void ParameterService::write_control_response(const char *response) noexcept
{
    if (response != nullptr) {
        (void)console_.write(
            reinterpret_cast<const std::uint8_t *>(response),
            std::strlen(response), 250U);
    }
}

void ParameterService::reboot_to_bootloader() noexcept
{
    if (!flash_write_allowed()) {
        write_control_response(kRebootDeniedResponse);
        PX4_ERR("bootloader reboot is not allowed while armed");
        return;
    }
    /* Console::write waits for the IN completion so the acknowledgement reaches
     * the host before USB disappears during reset. */
    write_control_response(kRebootResponse);
    boot_control_.reboot_to_recovery();
}

void ParameterService::execute_command(char *line) noexcept
{
    char *save{};
    char *command = ::strtok_r(line, " \t", &save);
    if (command == nullptr) {
        return;
    }

    if (std::strcmp(command, "dima") == 0) {
        char *operation = ::strtok_r(nullptr, " \t", &save);
        char *extra = ::strtok_r(nullptr, " \t", &save);
        if (operation != nullptr &&
            std::strcmp(operation, "identify") == 0 && extra == nullptr) {
            write_control_response(kIdentifyResponse);
        } else {
            PX4_ERR("usage: dima identify");
        }
        return;
    }

    if (std::strcmp(command, "reboot") == 0) {
        char *operation = ::strtok_r(nullptr, " \t", &save);
        char *extra = ::strtok_r(nullptr, " \t", &save);
        if (operation != nullptr && std::strcmp(operation, "-b") == 0 &&
            extra == nullptr) {
            reboot_to_bootloader();
        } else {
            PX4_ERR("usage: reboot -b");
        }
        return;
    }

    if (std::strcmp(command, "param") != 0) {
        PX4_ERR("unknown command");
        return;
    }
    char *operation = ::strtok_r(nullptr, " \t", &save);
    if (operation == nullptr) {
        PX4_ERR("missing subcommand");
        return;
    }

    if (std::strcmp(operation, "show") == 0) {
        char *pattern = ::strtok_r(nullptr, " \t", &save);
        const bool changed_only =
            pattern != nullptr && std::strcmp(pattern, "-c") == 0;
        if (pattern != nullptr &&
            (std::strcmp(pattern, "-a") == 0 || changed_only)) {
            pattern = ::strtok_r(nullptr, " \t", &save);
        }
        for (param_t parameter = 0U; parameter < param_count(); ++parameter) {
            if ((!changed_only || !param_value_is_default(parameter)) &&
                match_prefix(param_name(parameter), pattern)) {
                print_param(parameter);
            }
        }
    } else if (std::strcmp(operation, "get") == 0) {
        const param_t parameter =
            param_find(::strtok_r(nullptr, " \t", &save));
        if (parameter == PARAM_INVALID) {
            PX4_ERR("parameter not found");
        } else {
            print_param(parameter);
        }
    } else if (std::strcmp(operation, "set") == 0) {
        char *name = ::strtok_r(nullptr, " \t", &save);
        char *text = ::strtok_r(nullptr, " \t", &save);
        char *extra = ::strtok_r(nullptr, " \t", &save);
        const param_t parameter = param_find(name);
        param_value_u value{};
        if (parameter == PARAM_INVALID || extra != nullptr ||
            !parse_value(parameter, text, value)) {
            PX4_ERR("invalid parameter or value");
        } else {
            void *const source = param_type(parameter) == PARAM_TYPE_FLOAT
                                     ? static_cast<void *>(&value.f)
                                     : static_cast<void *>(&value.i);
            if (param_set(parameter, source) != 0) {
                PX4_ERR("set failed");
            } else {
                print_param(parameter);
            }
        }
    } else if (std::strcmp(operation, "save") == 0) {
        if (!flash_write_allowed()) {
            PX4_ERR("flash write is not allowed");
            return;
        }
        const int result = autosave_.saveNow(true);
        if (result == 0) {
            PX4_INFO_RAW("saved\n");
        } else {
            PX4_ERR("save failed: %d", result);
        }
    } else if (std::strcmp(operation, "reset") == 0) {
        bool any = false;
        for (char *name = ::strtok_r(nullptr, " \t", &save); name != nullptr;
             name = ::strtok_r(nullptr, " \t", &save)) {
            const param_t parameter = param_find(name);
            if (parameter == PARAM_INVALID) {
                PX4_ERR("parameter not found: %s", name);
            } else {
                (void)param_reset(parameter);
                any = true;
            }
        }
        if (!any) {
            PX4_ERR("missing parameter");
        }
    } else if (std::strcmp(operation, "reset_all") == 0) {
        param_reset_all();
    } else if (std::strcmp(operation, "status") == 0) {
        param_storage_status_s status{};
        const int status_result = param_storage_get_status(&status);
        const char *autosave_state = !autosave_.enabled()
                                         ? "disabled"
                                         : (autosave_.pending() ? "pending"
                                                                : "idle");
        PX4_INFO("count=%u used=%u storage=%d seq=%lu used_bytes=%lu free_bytes=%lu crc=%lu write=%lu enospc=%lu last_save=%llu autosave=%s",
                 param_count(), param_count_used(), status_result,
                 static_cast<unsigned long>(status.sequence),
                 static_cast<unsigned long>(status.used_bytes),
                 static_cast<unsigned long>(status.free_bytes),
                 static_cast<unsigned long>(status.crc_failures),
                 static_cast<unsigned long>(status.write_failures),
                 static_cast<unsigned long>(status.enospc_failures),
                 static_cast<unsigned long long>(status.last_save_timestamp),
                 autosave_state);
    } else if (std::strcmp(operation, "storage") == 0) {
        char *erase = ::strtok_r(nullptr, " \t", &save);
        char *confirm = ::strtok_r(nullptr, " \t", &save);
        if (erase == nullptr || std::strcmp(erase, "erase") != 0 ||
            confirm == nullptr || std::strcmp(confirm, "CONFIRM") != 0) {
            PX4_ERR("usage: param storage erase CONFIRM");
        } else if (!flash_write_allowed()) {
            PX4_ERR("flash write is not allowed");
        } else {
            PX4_INFO_RAW(
                "WARNING: erasing parameter storage is not power-fail safe\n");
            const int erased = param_storage_erase();
            if (erased != 0) {
                PX4_ERR("erase failed: %d", erased);
            } else {
                autosave_.enable(true);
                const int saved = autosave_.saveNow(true);
                if (saved != 0) {
                    PX4_ERR("rewrite failed: %d", saved);
                }
            }
        }
    } else {
        PX4_ERR("unknown subcommand: %s", operation);
    }
}

bool ParameterService::consume_line() noexcept
{
    std::uint8_t byte{};
    while (console_.read_byte(byte)) {
        if (ignore_lf_ && byte == '\n') {
            ignore_lf_ = false;
            continue;
        }
        ignore_lf_ = false;
        if (byte == '\r' || byte == '\n') {
            if (byte == '\r') {
                ignore_lf_ = true;
            }
            if (discard_line_) {
                discard_line_ = false;
                line_length_ = 0U;
                PX4_ERR("command line too long");
                return true;
            }
            if (line_length_ == 0U) {
                continue;
            }
            line_[line_length_] = '\0';
            execute_command(line_);
            line_length_ = 0U;
            return true;
        }
        if (byte == '\b' || byte == 0x7FU) {
            if (!discard_line_ && line_length_ > 0U) {
                --line_length_;
            }
            continue;
        }
        if (discard_line_) {
            continue;
        }
        if (line_length_ >= kLineCapacity) {
            discard_line_ = true;
            continue;
        }
        line_[line_length_++] = static_cast<char>(byte);
    }
    return false;
}

void ParameterService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    console_.service();

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
    (void)consume_line();
}

} // namespace dima::modules::parameters
