#define MODULE_NAME "param"
#include "ParameterService.hpp"

#include "logging/logging.hpp"
#include "parameters/FileStorage.hpp"
#include "api/Time.hpp"

#include <cerrno>

namespace dima::modules::parameters {

void ParameterService::service_sd_mirror() noexcept
{
    // storage_mutex 串行化 autosave 与后台镜像。镜像每轮只推进一个异步阶段；
    // 有 autosave、无卡、武装或其他持久化事务时保持 pending，不抢占主保存。
    dima::platform::MutexGuard lock{storage_mutex_};
    if (!lock) {
        return;
    }

    if (persistence_kind_ == PersistenceKind::SdMirror) {
        const int result = advance_persistence();
        if (result == 0) {
            PX4_INFO("param: SD mirror synchronized generation=%lu",
                     static_cast<unsigned long>(storage_generation_));
        } else if (result != -EAGAIN && result != -EPERM) {
            PX4_WARN("param: SD mirror retry failed: %d", result);
        }
        return;
    }
    if (persistence_kind_ != PersistenceKind::None ||
        !sd_mirror_required_ || !sd_available_ || autosave_.pending()) {
        return;
    }
    if (!flash_write_allowed()) {
        return;
    }

    const std::uint64_t now = hrt_absolute_time();
    if (now < sd_mirror_ready_after_us_) {
        return;
    }
    // 失败镜像最多每 3 s 重试一次，避免卡拔出/介质故障形成低优先级忙循环。
    if (last_sd_mirror_attempt_us_ != 0U &&
        now >= last_sd_mirror_attempt_us_ &&
        now - last_sd_mirror_attempt_us_ < kSdPollIntervalUs) {
        return;
    }
    last_sd_mirror_attempt_us_ = now;
    const int result = begin_sd_mirror();
    if (result != 0 && result != -EAGAIN && result != -EBUSY) {
        PX4_WARN("param: unable to start SD mirror: %d", result);
    }
}

void ParameterService::poll_sd_card() noexcept
{
    // armed 时不探测/挂载介质；disarmed 下 3 s 轮询。卡重新出现且参数未保存时
    // 请求 autosave，卡移除只降级 SD 副本，FlashFS 仍是运行主存储。
    const std::uint64_t now = hrt_absolute_time();
    if (armed_flash_.armed() ||
        (last_sd_poll_us_ != 0U && now >= last_sd_poll_us_ &&
         now - last_sd_poll_us_ < kSdPollIntervalUs)) {
        return;
    }
    last_sd_poll_us_ = now;

    bool available = false;
    const int result = dima::file_storage_poll(available);
    if (result == -EDEADLK || result == -EBUSY) {
        return;
    }
    if (available && autosave_.enabled() && parameters_unsaved()) {
        autosave_.request();
    }
    if (available == sd_available_) {
        return;
    }
    sd_available_ = available;
    if (available) {
        PX4_INFO("param: SD card mounted; synchronizing parameters");
        sd_mirror_ready_after_us_ =
            now > UINT64_MAX - kSdMountSettleUs
                ? UINT64_MAX
                : now + kSdMountSettleUs;
        last_sd_mirror_attempt_us_ = 0U;
        sd_mirror_required_ = flashfs_ready_ && storage_generation_ != 0U;
        (void)autosave_.resume_after_storage_available();
    } else {
        sd_mirror_ready_after_us_ = 0U;
        sd_mirror_required_ = flashfs_ready_ && storage_generation_ != 0U;
        PX4_WARN("param: SD card removed; FlashFS remains active");
    }
}

} // namespace dima::modules::parameters
