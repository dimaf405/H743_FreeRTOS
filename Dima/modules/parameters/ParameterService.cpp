#define MODULE_NAME "param"
#include "ParameterService.hpp"

#include "logging/logging.hpp"
#include "parameters/FileStorage.hpp"
#include "api/Time.hpp"

#include <cerrno>
#include <cstring>

namespace dima::modules::parameters {

ParameterService::ParameterService(
    dima::parameters::FlashFS &flashfs,
    dima::platform::ParameterFileStore &parameter_files,
    dima::platform::ArmedFlashCoordinator &armed_flash,
    dima::platform::Synchronization &synchronization,
    dima::platform::CriticalSection &critical,
    dima::middleware::maintenance::
        RuntimeMaintenanceCoordinator &maintenance) noexcept
    : ScheduledWorkItem("param", px4::wq_configurations::lp_default),
      flashfs_(flashfs), parameter_files_(parameter_files),
      armed_flash_(armed_flash),
      synchronization_(synchronization), critical_(critical),
      maintenance_(maintenance),
      autosave_(armed_flash, &ParameterService::cancel_async_save, this)
{
}

void ParameterService::cancel_async_save(void *context) noexcept
{
    if (context == nullptr) {
        return;
    }
    auto &self = *static_cast<ParameterService *>(context);
    dima::platform::MutexGuard lock{self.storage_mutex_};
    if (lock) {
        self.cancel_persistence();
    }
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

    // notify 可能来自不同任务：临界区只覆盖固定 pending 快照和标志，不在其中
    // 发布 uORB 或请求保存。loading 期间的逐参数写入不触发 autosave。
    dima::platform::CriticalGuard guard{self.critical_};
    self.pending_update_ = update;
    self.update_pending_ = true;
    if (!self.loading_) {
        self.autosave_request_pending_ = true;
    }
}

bool ParameterService::parameters_unsaved() const noexcept
{
    // 遍历的是生成参数 registry 的句柄范围，不维护另一份参数名列表。
    for (param_t parameter = 0U; parameter < param_count(); ++parameter) {
        if (param_value_unsaved(parameter)) {
            return true;
        }
    }
    return false;
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

    // 初始化顺序：两把 mutex -> FlashFS -> param 回调 -> FatFs -> storage backend
    // -> param_init/load。失败统一 shutdown，绝不格式化/擦除不可用 Flash。
    if (!param_mutex_.initialize(synchronization_)) {
        return fail_init();
    }
    if (!storage_mutex_.initialize(synchronization_)) {
        return fail_init();
    }
    flashfs_ready_ = flashfs_.initialize();
    if (!flashfs_ready_) {
        PX4_ERR("param: FlashFS unavailable; preserving storage contents");
    }

    param_register_lock_callbacks(&ParameterService::lock_params,
                                  &ParameterService::unlock_params, this);
    param_register_notify_callback(&ParameterService::notify_params, this);

    if (dima::file_storage_initialize(
            parameter_files_, synchronization_, sd_available_) != 0) {
        return fail_init();
    }
    last_sd_poll_us_ = hrt_absolute_time();
    if (sd_available_) {
        PX4_INFO("param: SD mirror mounted");
    } else {
        PX4_WARN("param: SD absent; FlashFS remains active");
    }
    if (param_register_storage_backend(&storage_backend_, this) != 0) {
        return fail_init();
    }

    param_init();
    if (!param_is_ready()) {
        return fail_init();
    }

    // load 期间禁止 autosave 回响；Flash/SD 都不存在是合法首次启动，其他错误只
    // 记录并保持生成默认值。若从 SD 选中更新代，则通知后台重建 Flash。
    loading_ = true;
    const int loaded = param_load_default();
    loading_ = false;
    if (loaded != 0 && loaded != -ENOENT) {
        PX4_ERR("load failed: %d", loaded);
    }
    if ((loaded == 0 || loaded == -ENOENT) &&
        flash_resync_required_) {
        param_notify_changes();
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

    autosave_.enable();
    if (!autosave_.enabled()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }
    param_register_notify_callback(&ParameterService::notify_params, this);
    if (parameters_unsaved()) {
        param_notify_changes();
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
    // 先撤销 notify 和 autosave，再排空 WorkQueue，最后在 storage_mutex 下取消
    // 介质事务；防止回调在 payload_ 清理后继续推进。
    param_register_notify_callback(nullptr, nullptr);
    autosave_.stop();
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    {
        dima::platform::MutexGuard lock{storage_mutex_};
        if (lock) {
            cancel_persistence();
        }
    }
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
    std::memset(comparison_payload_, 0, sizeof(comparison_payload_));
    pending_update_ = {};
    loading_ = false;
    update_pending_ = false;
    autosave_request_pending_ = false;
    flashfs_ready_ = false;
    sd_available_ = false;
    flash_resync_required_ = false;
    sd_mirror_required_ = false;
    generation_committed_ = false;
    storage_generation_ = 0U;
    persistence_generation_ = 0U;
    maintenance_progress_ = 0U;
    persistence_size_ = 0U;
    flash_result_ = 0;
    sd_result_ = 0;
    maintenance_ticket_ = 0U;
    persistence_kind_ = PersistenceKind::None;
    persistence_phase_ = PersistencePhase::Idle;
    last_sd_poll_us_ = 0U;
    last_sd_mirror_attempt_us_ = 0U;
}

void ParameterService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    poll_sd_card();

    // 在短临界区内取走最新一次 coalesced parameter_update/autosave 请求；实际
    // uORB 发布和持久化都在临界区外执行，避免扩大 IRQ 屏蔽窗口。
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
    if (flash_resync_required_ && !autosave_.pending()) {
        autosave_.request();
    }
    service_sd_mirror();
}

} // namespace dima::modules::parameters
