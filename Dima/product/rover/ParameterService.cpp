#define MODULE_NAME "param"
#include "ParameterService.hpp"

#include "usb_console/usb_console.h"
#include "logging/logging.hpp"
#include "parameters/param.h"
#include "parameters/flashparams/flashparams.h"
#include "parameter_update.hpp"
#include "uorb/uORB.hpp"
#include "freertos/hrt.hpp"
#include "freertos/parameter_flash.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
}

namespace dima::product::rover {
namespace {
constexpr uint32_t kPollUs = 10000U;
constexpr size_t kLineCapacity = 192U;
constexpr size_t kPayloadCapacity = px4::parameter_storage_max_bytes;
StaticSemaphore_t g_param_mutex_storage{};
SemaphoreHandle_t g_param_mutex{};
StaticSemaphore_t g_storage_mutex_storage{};
SemaphoreHandle_t g_storage_mutex{};
uORB::Publication<parameter_update_s> g_parameter_update_pub{ORB_ID(parameter_update)};
alignas(32) uint8_t g_payload[kPayloadCapacity]{};
ParamAutosave *g_autosave{};
bool g_loading{};
parameter_update_s g_pending_update{};
bool g_update_pending{};
bool g_autosave_request_pending{};
char g_line[kLineCapacity + 1U]{};
size_t g_line_length{};
bool g_discard_line{};
bool g_ignore_lf{};
FlashWriteAllowedHook g_write_allowed_hook{};

void lock_params(void *) noexcept { if (g_param_mutex) { (void)xSemaphoreTakeRecursive(g_param_mutex, portMAX_DELAY); } }
void unlock_params(void *) noexcept { if (g_param_mutex) { (void)xSemaphoreGiveRecursive(g_param_mutex); } }

class StorageLock {
public:
    StorageLock() noexcept
        : locked_(g_storage_mutex != nullptr
                  && xSemaphoreTakeRecursive(g_storage_mutex, portMAX_DELAY) == pdTRUE) {}
    ~StorageLock() { if (locked_) { (void)xSemaphoreGiveRecursive(g_storage_mutex); } }
    explicit operator bool() const noexcept { return locked_; }
private:
    bool locked_;
};

void notify_params(const parameter_update_s *source, void *) noexcept
{
    if (source == nullptr) { return; }
    parameter_update_s update = *source;
    update.timestamp = hrt_absolute_time();

    taskENTER_CRITICAL();
    // Core 回调已按生成顺序串行，直接保留最后到达的快照可自然跨 uint32 回绕。
    g_pending_update = update;
    g_update_pending = true;
    if (!g_loading) {
        g_autosave_request_pending = true;
    }
    taskEXIT_CRITICAL();
}

int storage_save(param_storage_enumerator_t enumerate, void *enumerate_context, void *) noexcept
{
    if (!enumerate || !flash_write_allowed()) { return -EPERM; }
    StorageLock lock;
    if (!lock) { return -EDEADLK; }
    size_t payload_size{};
    const int encode_ret = flashparams_encode_buffer(g_payload, sizeof(g_payload),
                                                      enumerate, enumerate_context,
                                                      &payload_size);
    if (encode_ret != 0) { return encode_ret; }
    uint32_t sequence{};
    return dima::platform::parameter_flash_append(g_payload, payload_size, &sequence);
}

int storage_load(param_storage_visitor_t visitor, void *visitor_context, void *) noexcept
{
    if (!visitor) { return -EINVAL; }
    StorageLock lock;
    if (!lock) { return -EDEADLK; }
    size_t payload_size{};
    uint32_t sequence{};
    const int load_ret = dima::platform::parameter_flash_load(
        g_payload, sizeof(g_payload), &payload_size, &sequence);
    if (load_ret != 0) { return load_ret; }
    return flashparams_decode_buffer(g_payload, payload_size, visitor, visitor_context);
}

int storage_erase(void *) noexcept { if (!flash_write_allowed()) { return -EPERM; } StorageLock lock; return lock ? dima::platform::parameter_flash_erase() : -EDEADLK; }
int storage_status(param_storage_status_s *out, void *) noexcept
{
    if (!out) { return -EINVAL; }
    const auto status = dima::platform::parameter_flash_status();
    out->sequence = status.valid_sequence;
    out->used_bytes = static_cast<uint32_t>(status.used_bytes);
    out->free_bytes = static_cast<uint32_t>(status.free_bytes);
    out->crc_failures = status.crc_failures;
    out->write_failures = status.write_failures;
    out->enospc_failures = status.enospc_failures;
    out->last_save_timestamp = g_autosave ? g_autosave->lastAutosave() : 0U;
    out->autosave_enabled = g_autosave && g_autosave->enabled();
    return 0;
}
const param_storage_backend_s g_storage_backend{storage_load, storage_save, storage_erase, storage_status};

bool autosave_write_allowed() { return flash_write_allowed(); }

bool match_prefix(const char *name, const char *pattern)
{
    if (!pattern || !*pattern) { return true; }
    const size_t length = std::strlen(pattern);
    const size_t prefix = pattern[length - 1U] == '*' ? length - 1U : length;
    return std::strncmp(name, pattern, prefix) == 0 && (prefix != length || name[prefix] == '\0');
}

void print_param(param_t param)
{
    if (param_type(param) == PARAM_TYPE_FLOAT) { float value{}; (void)param_get(param, &value); PX4_INFO_RAW("%s %.9g\n", param_name(param), static_cast<double>(value)); }
    else { int32_t value{}; (void)param_get(param, &value); PX4_INFO_RAW("%s %ld\n", param_name(param), static_cast<long>(value)); }
}

bool parse_value(param_t param, const char *text, param_value_u &value)
{
    char *end{};
    errno = 0;
    if (param_type(param) == PARAM_TYPE_FLOAT) {
        value.f = std::strtof(text, &end);
        return errno == 0 && end != text && *end == '\0' && std::isfinite(value.f);
    }
    const long parsed = std::strtol(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed < INT32_MIN || parsed > INT32_MAX) { return false; }
    value.i = static_cast<int32_t>(parsed); return true;
}

void execute_command(char *line)
{
    char *save{}; char *cmd = ::strtok_r(line, " \t", &save);
    if (!cmd || std::strcmp(cmd, "param") != 0) { PX4_ERR("unknown command"); return; }
    char *op = ::strtok_r(nullptr, " \t", &save);
    if (!op) { PX4_ERR("missing subcommand"); return; }
    if (std::strcmp(op, "show") == 0) {
        char *arg = ::strtok_r(nullptr, " \t", &save); bool changed_only = arg && std::strcmp(arg, "-c") == 0;
        if (arg && (std::strcmp(arg, "-a") == 0 || changed_only)) { arg = ::strtok_r(nullptr, " \t", &save); }
        for (param_t p = 0; p < param_count(); ++p) { if ((!changed_only || !param_value_is_default(p)) && match_prefix(param_name(p), arg)) { print_param(p); } }
    } else if (std::strcmp(op, "get") == 0) {
        char *name = ::strtok_r(nullptr, " \t", &save); const param_t p = param_find(name); if (p == PARAM_INVALID) { PX4_ERR("parameter not found"); } else { print_param(p); }
    } else if (std::strcmp(op, "set") == 0) {
        char *name = ::strtok_r(nullptr, " \t", &save);
        char *text = ::strtok_r(nullptr, " \t", &save);
        char *extra = ::strtok_r(nullptr, " \t", &save);
        const param_t p = param_find(name);
        param_value_u value{};
        if (p == PARAM_INVALID || !text || extra != nullptr || !parse_value(p, text, value)) { PX4_ERR("invalid parameter or value"); }
        else if (param_set(p, param_type(p) == PARAM_TYPE_FLOAT ? static_cast<void *>(&value.f) : static_cast<void *>(&value.i)) != 0) { PX4_ERR("set failed"); } else { print_param(p); }
    } else if (std::strcmp(op, "save") == 0) {
        if (!flash_write_allowed()) { PX4_ERR("flash write is not allowed"); return; }
        const int ret = g_autosave ? g_autosave->saveNow(true) : param_save_default(true); if (ret == 0) { PX4_INFO_RAW("saved\n"); } else { PX4_ERR("save failed: %d", ret); }
    } else if (std::strcmp(op, "reset") == 0) {
        bool any = false; for (char *name = ::strtok_r(nullptr, " \t", &save); name; name = ::strtok_r(nullptr, " \t", &save)) { const param_t p = param_find(name); if (p == PARAM_INVALID) { PX4_ERR("parameter not found: %s", name); } else { (void)param_reset(p); any = true; } } if (!any) { PX4_ERR("missing parameter"); }
    } else if (std::strcmp(op, "reset_all") == 0) { param_reset_all();
    } else if (std::strcmp(op, "status") == 0) {
        param_storage_status_s status{};
        const int status_ret = param_storage_get_status(&status);
        const char *autosave_state = !g_autosave || !g_autosave->enabled() ? "disabled" :
                                     g_autosave->pending() ? "pending" : "idle";
        PX4_INFO("count=%u used=%u storage=%d seq=%lu used_bytes=%lu free_bytes=%lu crc=%lu write=%lu enospc=%lu last_save=%llu autosave=%s",
                 param_count(), param_count_used(), status_ret,
                 static_cast<unsigned long>(status.sequence),
                 static_cast<unsigned long>(status.used_bytes),
                 static_cast<unsigned long>(status.free_bytes),
                 static_cast<unsigned long>(status.crc_failures),
                 static_cast<unsigned long>(status.write_failures),
                 static_cast<unsigned long>(status.enospc_failures),
                 static_cast<unsigned long long>(status.last_save_timestamp),
                 autosave_state);
    } else if (std::strcmp(op, "storage") == 0) {
        char *erase = ::strtok_r(nullptr, " \t", &save); char *confirm = ::strtok_r(nullptr, " \t", &save);
        if (!erase || std::strcmp(erase, "erase") != 0 || !confirm || std::strcmp(confirm, "CONFIRM") != 0) { PX4_ERR("usage: param storage erase CONFIRM"); }
        else if (!flash_write_allowed()) { PX4_ERR("flash write is not allowed"); }
        else {
            PX4_INFO_RAW("WARNING: erasing parameter storage is not power-fail safe\n");
            const int ret = param_storage_erase();
            if (ret == 0) {
                if (g_autosave) { g_autosave->enable(true); }
                const int save_ret = g_autosave ? g_autosave->saveNow(true) : param_save_default(true);
                if (save_ret != 0) { PX4_ERR("rewrite failed: %d", save_ret); }
            } else { PX4_ERR("erase failed: %d", ret); }
        }
    } else { PX4_ERR("unknown subcommand: %s", op); }
}

bool consume_line()
{
    uint8_t byte{};
    while (usb_console_read_byte(&byte)) {
        if (g_ignore_lf && byte == '\n') { g_ignore_lf = false; continue; } g_ignore_lf = false;
        if (byte == '\r' || byte == '\n') {
            if (byte == '\r') { g_ignore_lf = true; }
            if (g_discard_line) { g_discard_line = false; g_line_length = 0U; PX4_ERR("command line too long"); return true; }
            if (g_line_length == 0U) { continue; }
            g_line[g_line_length] = '\0'; execute_command(g_line); g_line_length = 0U; return true;
        }
        if (byte == '\b' || byte == 0x7FU) { if (!g_discard_line && g_line_length > 0U) { --g_line_length; } continue; }
        if (g_discard_line) { continue; }
        if (g_line_length >= kLineCapacity) { g_discard_line = true; continue; }
        g_line[g_line_length++] = static_cast<char>(byte);
    }
    return false;
}
} // namespace

bool flash_write_allowed() noexcept
{
    taskENTER_CRITICAL();
    const FlashWriteAllowedHook hook = g_write_allowed_hook;
    taskEXIT_CRITICAL();
    return !hook || hook();
}
void set_flash_write_allowed_hook(FlashWriteAllowedHook hook) noexcept
{
    taskENTER_CRITICAL();
    g_write_allowed_hook = hook;
    taskEXIT_CRITICAL();
}

ParameterService::ParameterService() noexcept : ScheduledWorkItem("param", px4::wq_configurations::lp_default) {}
bool ParameterService::init() noexcept
{
    if (initialized_) { return true; }
    g_param_mutex = xSemaphoreCreateRecursiveMutexStatic(&g_param_mutex_storage);
    g_storage_mutex = xSemaphoreCreateRecursiveMutexStatic(&g_storage_mutex_storage);
    if (!g_param_mutex || !g_storage_mutex || !dima::platform::parameter_flash_init()) { return false; }
    param_register_lock_callbacks(lock_params, unlock_params, nullptr);
    g_autosave = &autosave_;
    autosave_.setWriteAllowedCallback(autosave_write_allowed);
    param_register_notify_callback(notify_params, nullptr);
    if (param_register_storage_backend(&g_storage_backend, nullptr) != 0) { return false; }
    param_init();
    if (!param_is_ready()) {
        param_register_notify_callback(nullptr, nullptr);
        g_autosave = nullptr;
        return false;
    }
    g_loading = true;
    const int load_ret = param_load_default();
    g_loading = false;
    if (load_ret != 0 && load_ret != -ENOENT) { PX4_ERR("load failed: %d", load_ret); }
    initialized_ = true; return true;
}
bool ParameterService::start() noexcept
{
    if (!initialized_ || started_) { return initialized_ && started_; }
    autosave_.enable(true);
    param_register_notify_callback(notify_params, nullptr);
    for (param_t param = 0U; param < param_count(); ++param) {
        if (param_value_unsaved(param)) {
            param_notify_changes();
            break;
        }
    }
    started_ = ScheduleOnInterval(kPollUs);
    if (!started_) {
        param_register_notify_callback(nullptr, nullptr);
        autosave_.stop();
    }
    return started_;
}

void ParameterService::stop() noexcept
{
    param_register_notify_callback(nullptr, nullptr);
    autosave_.stop();
    taskENTER_CRITICAL();
    g_update_pending = false;
    g_autosave_request_pending = false;
    taskEXIT_CRITICAL();
    if (started_) {
        ScheduleClear();
        started_ = false;
    }
}

void ParameterService::Run()
{
    usb_console_service();
    parameter_update_s update{};
    bool publish_update = false;
    bool request_autosave = false;
    taskENTER_CRITICAL();
    if (g_update_pending) {
        update = g_pending_update;
        g_update_pending = false;
        publish_update = true;
    }
    request_autosave = g_autosave_request_pending;
    g_autosave_request_pending = false;
    taskEXIT_CRITICAL();

    if (publish_update) {
        (void)g_parameter_update_pub.publish(update);
    }
    if (request_autosave && g_autosave != nullptr) {
        g_autosave->request();
    }
    (void)consume_line();
}

} // namespace dima::product::rover
