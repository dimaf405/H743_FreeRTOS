#define MODULE_NAME "param"
#include "ParameterService.hpp"
#include "ParameterSnapshotCodec.hpp"

#include "parameters/FileStorage.hpp"
#include "api/Time.hpp"

#include <cerrno>
#include <cstring>

namespace dima::modules::parameters {
namespace {

bool sd_media_failure(int result) noexcept
{
    // 这些错误表示当前 FatFs/块设备会话已经不可信：统一转为 unavailable，必须
    // 经过下一轮 initialize/unmount/mount，不能在旧 FATFS 对象上继续镜像。
    return result == -ENODEV || result == -EIO || result == -ENXIO ||
           result == -EBADF || result == -ETIMEDOUT;
}

} // namespace

const param_storage_backend_s ParameterService::storage_backend_{
    &ParameterService::storage_load,
    &ParameterService::storage_save,
    &ParameterService::storage_status,
};

int ParameterService::encode_snapshot(
    param_storage_enumerator_t enumerate, void *enumerate_context,
    std::uint8_t *destination, std::uint32_t generation,
    std::size_t &snapshot_size) noexcept
{
    return snapshot_codec::encode(
        enumerate, enumerate_context, destination, kPayloadCapacity,
        generation, snapshot_size);
}

// 参数保存与 Arm 原子互斥，但不再申请逐阶段的 BootHealth/喂狗维护票据。
// 和 PX4 的后台保存一样，RAM 参数锁只覆盖编码，介质 I/O 可被高优先级任务抢占。
namespace {
class ParameterSaveLease final {
public:
    explicit ParameterSaveLease(dima::platform::ArmedFlashCoordinator &armed) noexcept
        : armed_(armed), acquired_(armed.begin_maintenance())
    {
    }
    ~ParameterSaveLease()
    {
        if (acquired_) armed_.end_maintenance();
    }
    explicit operator bool() const noexcept { return acquired_; }
    ParameterSaveLease(const ParameterSaveLease &) = delete;
    ParameterSaveLease &operator=(const ParameterSaveLease &) = delete;

private:
    dima::platform::ArmedFlashCoordinator &armed_;
    bool acquired_;
};
} // namespace

int ParameterService::write_flash_snapshot(std::size_t size) noexcept
{
    int result = flashfs_.begin_write_entry(
        dima::parameters::FLASH_TOKEN_PARAMS, payload_, size);
    if (result != 0) return result;

    // -EAGAIN 表示本笔事务已推进；连续完成 header/payload/commit，不再每步等 10 ms。
    // -EBUSY 表示资源尚未取得，必须退出重试，不能在 storage 队列内忙等其他 owner。
    do {
        result = flashfs_.continue_operation();
    } while (result == -EAGAIN);
    if (result != 0) flashfs_.cancel_operation();
    return result;
}

int ParameterService::write_sd_snapshot(std::size_t size) noexcept
{
    if (!sd_available_) return -ENODEV;
    if (hrt_absolute_time() < sd_mirror_ready_after_us_) return -EAGAIN;

    constexpr auto domain = dima::platform::AtomicFileDomain::Parameters;
    int result = dima::file_storage_begin_save(domain, payload_, size);
    if (result == -ESTALE) {
        // 换卡后只重新发现一次有效 primary/backup/tmp，防止覆盖唯一恢复副本。
        snapshot_codec::SnapshotInfo previous_info{};
        std::size_t previous_size{};
        result = dima::file_storage_load(
            domain, comparison_payload_, sizeof(comparison_payload_), previous_size,
            &snapshot_codec::validate, &previous_info);
        if (result != -EBUSY && result != -EDEADLK && result != -EAGAIN &&
            !sd_media_failure(result)) {
            result = dima::file_storage_begin_save(domain, payload_, size);
        }
    }
    if (result == 0) {
        // 沿用原子文件的写入、同步、一次回读和主备轮换；低优先级任务连续推进，
        // 底层每次 I/O 仍有超时。仅推进自己已经取得的事务，不取消其他文件域。
        do {
            result = dima::file_storage_continue_save(domain);
        } while (result == -EAGAIN);
        if (result != 0) dima::file_storage_cancel_save(domain);
    }
    if (sd_media_failure(result)) sd_available_ = false;
    return result;
}

int ParameterService::storage_save(param_storage_enumerator_t enumerate,
                                   void *enumerate_context,
                                   void *backend_context) noexcept
{
    if (enumerate == nullptr || backend_context == nullptr) return -EINVAL;
    auto &self = *static_cast<ParameterService *>(backend_context);
    // 对齐 PX4 param_save_default(false)：存储忙时立即返回，交给 autosave 限频重试。
    dima::platform::MutexGuard lock{self.storage_mutex_, dima::platform::Timeout::no_wait()};
    if (!lock) return -EBUSY;
    ParameterSaveLease lease{self.armed_flash_};
    if (!lease) return -EPERM;
    if (self.storage_generation_ == UINT32_MAX) return -EOVERFLOW;

    const std::uint32_t generation = self.storage_generation_ + 1U;
    std::size_t snapshot_size{};
    const int encoded = self.encode_snapshot(
        enumerate, enumerate_context, self.payload_, generation, snapshot_size);
    if (encoded != 0) return encoded;

    // 一次编码后连续完成主副本和可用 SD 备份，保存过程中不持有参数锁。
    int flash_result = self.flashfs_ready_
        ? self.write_flash_snapshot(snapshot_size) : -ENODEV;
    const int sd_result = self.write_sd_snapshot(snapshot_size);
    if (flash_result == 0 || sd_result == 0) self.storage_generation_ = generation;

    // Flash 满/损坏时，只有 SD 已提交同代完整快照才允许擦除重建；保留掉电恢复边界。
    if (self.flashfs_ready_ && (flash_result == -ENOSPC || flash_result == -EIO) &&
        sd_result == 0) {
        flash_result = self.flashfs_.begin_erase_all(dima::parameters::FLASH_TOKEN_PARAMS);
        if (flash_result == 0) {
            flash_result = self.flashfs_.continue_operation();
            if (flash_result == 0) {
                flash_result = self.write_flash_snapshot(snapshot_size);
            } else {
                self.flashfs_.cancel_operation();
            }
        }
    }
    self.sd_mirror_required_ = flash_result == 0 && sd_result != 0;
    self.flash_resync_required_ = self.flashfs_ready_ && sd_result == 0 && flash_result != 0;

    // 主副本成功即可确认保存；SD 失败留给后台镜像。保存期间是否有新参数，
    // 由 Parameter Core 的写入计数判断，不再重新编码整份快照进行逐字节比较。
    return self.flashfs_ready_ ? flash_result : sd_result;
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
    const auto read_flash = [&self](std::uint8_t *destination,
                                    snapshot_codec::SnapshotInfo &info) noexcept {
        if (!self.flashfs_ready_) {
            return -ENODEV;
        }
        const int loaded = self.flashfs_.read_entry(
            dima::parameters::FLASH_TOKEN_PARAMS,
            destination, kPayloadCapacity);
        return loaded < 0
                   ? loaded
                   : snapshot_codec::validate(
                         destination, static_cast<std::size_t>(loaded), &info);
    };

    snapshot_codec::SnapshotInfo flash_info{};
    const int flash_result = read_flash(self.payload_, flash_info);

    snapshot_codec::SnapshotInfo sd_info{};
    std::size_t sd_size{};
    const int sd_result = self.sd_available_
        ? dima::file_storage_load(
              dima::platform::AtomicFileDomain::Parameters,
              self.comparison_payload_, sizeof(self.comparison_payload_),
              sd_size,
              &snapshot_codec::validate, &sd_info)
        : -ENODEV;
    if (sd_media_failure(sd_result)) {
        self.sd_available_ = false;
    }

    // Flash 与 SD 都通过完整 codec 验证后，选择 generation 更高者；同代以 Flash
    // 为主。另一介质缺失/CRC 或 generation 不同会设置后台 resync/mirror 标志。
    snapshot_codec::SnapshotInfo selected{};
    const bool loaded_from_sd =
        sd_result == 0 &&
        (flash_result != 0 || sd_info.generation > flash_info.generation);
    const std::uint8_t *selected_payload = self.payload_;
    if (loaded_from_sd) {
        selected = sd_info;
        selected_payload = self.comparison_payload_;
    } else if (flash_result == 0) {
        selected = flash_info;
    } else {
        if (flash_result == -ENOENT &&
            (sd_result == -ENOENT || sd_result == -ENODEV)) {
            return -ENOENT;
        }
        return flash_result != -ENOENT && flash_result != -ENODEV
                   ? flash_result
                   : sd_result;
    }

    self.storage_generation_ = selected.generation;
    const bool media_match =
        flash_result == 0 && sd_result == 0 &&
        flash_info.generation == sd_info.generation &&
        flash_info.payload_size == sd_info.payload_size &&
        flash_info.payload_crc == sd_info.payload_crc;
    self.flash_resync_required_ =
        loaded_from_sd && self.flashfs_ready_ && !media_match;
    self.sd_mirror_required_ =
        !loaded_from_sd && flash_result == 0 && !media_match;
    if (selected.payload_size == 0U) {
        return -ENOENT;
    }

    // 只解码可变参数层；生成固定参数不会被存储镜像覆盖。
    return snapshot_codec::decode_mutable(
        selected_payload + kSnapshotHeaderBytes, selected.payload_size,
        visitor, visitor_context);
}

int ParameterService::storage_status(param_storage_status_s *output,
                                     void *backend_context) noexcept
{
    if (output == nullptr || backend_context == nullptr) {
        return -EINVAL;
    }
    auto &self = *static_cast<ParameterService *>(backend_context);
    dima::platform::MutexGuard lock{self.storage_mutex_};
    if (!lock) {
        return -EDEADLK;
    }
    const dima::parameters::FlashFSStatus status = self.flashfs_.status();
    std::memset(output, 0, sizeof(*output));
    output->used_bytes = status.used_bytes;
    output->free_bytes = status.free_bytes;
    output->crc_failures = status.crc_failures;
    output->write_failures = status.write_failures;
    output->sequence = self.storage_generation_;
    output->last_save_timestamp = self.autosave_.lastAutosave();
    output->autosave_enabled = self.autosave_.enabled();
    return 0;
}

int ParameterService::save_sd_mirror() noexcept
{
    ParameterSaveLease lease{armed_flash_};
    if (!lease) return -EPERM;

    // 镜像只复制已经提交且通过 CRC 的 Flash 快照，不把当前 RAM 候选写成已保存代。
    const int loaded = flashfs_.read_entry(
        dima::parameters::FLASH_TOKEN_PARAMS, payload_, sizeof(payload_));
    if (loaded < 0) return loaded;
    snapshot_codec::SnapshotInfo info{};
    const int valid = snapshot_codec::validate(
        payload_, static_cast<std::size_t>(loaded), &info);
    if (valid != 0 || info.generation != storage_generation_) {
        return valid != 0 ? valid : -EAGAIN;
    }
    const int result = write_sd_snapshot(static_cast<std::size_t>(loaded));
    sd_mirror_required_ = result != 0;
    return result;
}

} // namespace dima::modules::parameters
