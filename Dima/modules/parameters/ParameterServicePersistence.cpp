#define MODULE_NAME "param"
#include "ParameterService.hpp"
#include "ParameterSnapshotCodec.hpp"

#include "parameters/FileStorage.hpp"
#include "api/Time.hpp"

#include <cerrno>
#include <cstring>

namespace dima::modules::parameters {

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

int ParameterService::begin_persistence(PersistenceKind kind,
                                        std::size_t snapshot_size,
                                        std::uint32_t generation) noexcept
{
    if (kind == PersistenceKind::None ||
        persistence_kind_ != PersistenceKind::None ||
        persistence_phase_ != PersistencePhase::Idle ||
        snapshot_size < kSnapshotHeaderBytes ||
        snapshot_size > sizeof(payload_) || generation == 0U) {
        return -EINVAL;
    }

    // 每个 Save/SD mirror 都先取得“未武装 Flash”排他锁，再申请 Runtime maintenance
    // ticket；两者任一失败都不进入介质写状态机。
    if (!armed_flash_.begin_maintenance()) {
        return -EPERM;
    }
    maintenance_interlock_acquired_ = true;

    const auto ticket = maintenance_.request(hrt_absolute_time());
    if (ticket == 0U) {
        armed_flash_.end_maintenance();
        maintenance_interlock_acquired_ = false;
        return -EBUSY;
    }

    persistence_kind_ = kind;
    persistence_phase_ = PersistencePhase::WaitForApproval;
    persistence_size_ = snapshot_size;
    persistence_generation_ = generation;
    maintenance_ticket_ = ticket;
    maintenance_progress_ = 0U;
    // SdMirror 复制的是已提交 generation，不可再次推进 storage_generation；普通
    // Save 则在 Flash 或 SD 首个成功提交时只推进一次。
    generation_committed_ = kind == PersistenceKind::SdMirror;
    flash_result_ = flashfs_ready_ ? -EINPROGRESS : -ENODEV;
    sd_result_ = sd_available_ ? -EINPROGRESS : -ENODEV;
    return 0;
}

bool ParameterService::record_maintenance_progress() noexcept
{
    // 每个非阻塞阶段前进都增加进度并续租；uint32 回绕或协调器拒绝立即取消
    // 两个介质操作并释放 interlock。
    ++maintenance_progress_;
    if (maintenance_progress_ == 0U) {
        cancel_persistence();
        return false;
    }
    if (!maintenance_.report_progress(
            maintenance_ticket_, maintenance_progress_,
            hrt_absolute_time())) {
        cancel_persistence();
        return false;
    }
    return true;
}

void ParameterService::mark_generation_committed() noexcept
{
    if (!generation_committed_) {
        storage_generation_ = persistence_generation_;
        generation_committed_ = true;
    }
}

int ParameterService::advance_persistence() noexcept
{
    // 状态机不阻塞：WaitApproval -> Flash write -> SD write；若 Flash 满/损坏且
    // SD 已成功，则 Erase Flash -> Rewrite。每轮至多推进一个实际介质步骤。
    if (persistence_kind_ == PersistenceKind::None ||
        persistence_phase_ == PersistencePhase::Idle) {
        return -EINVAL;
    }

    const auto permit = maintenance_.permit(
        maintenance_ticket_, hrt_absolute_time());
    if (permit == dima::middleware::maintenance::
                      RuntimeMaintenanceCoordinator::Permit::Waiting) {
        return -EAGAIN;
    }
    if (permit == dima::middleware::maintenance::
                      RuntimeMaintenanceCoordinator::Permit::Denied) {
        cancel_persistence();
        return -EPERM;
    }
    int result = 0;
    switch (persistence_phase_) {
    case PersistencePhase::WaitForApproval:
        persistence_phase_ =
            persistence_kind_ == PersistenceKind::SdMirror
                ? PersistencePhase::BeginSdWrite
                : (flashfs_ready_ ? PersistencePhase::BeginFlashWrite
                                  : PersistencePhase::BeginSdWrite);
        return record_maintenance_progress() ? -EAGAIN : -EPERM;

    case PersistencePhase::BeginFlashWrite:
        // Flash 是首选主副本；启动失败仍继续尝试 SD，使无 Flash 时保留可恢复路径。
        result = flashfs_.begin_write_entry(
            dima::parameters::FLASH_TOKEN_PARAMS, payload_,
            persistence_size_);
        if (result == -EBUSY) {
            return -EAGAIN;
        }
        if (result == 0) {
            persistence_phase_ = PersistencePhase::ContinueFlashWrite;
        } else {
            flash_result_ = result;
            persistence_phase_ = PersistencePhase::BeginSdWrite;
        }
        return record_maintenance_progress() ? -EAGAIN : -EPERM;

    case PersistencePhase::ContinueFlashWrite:
        result = flashfs_.continue_operation();
        if (result == -EBUSY) {
            return -EAGAIN;
        }
        if (result == -EAGAIN) {
            return record_maintenance_progress() ? -EAGAIN : -EPERM;
        }
        flash_result_ = result;
        if (result == 0) {
            mark_generation_committed();
        }
        persistence_phase_ = PersistencePhase::BeginSdWrite;
        return record_maintenance_progress() ? -EAGAIN : -EPERM;

    case PersistencePhase::BeginSdWrite:
        if (!sd_available_) {
            sd_result_ = -ENODEV;
            return finish_persistence();
        }
        result = dima::file_storage_begin_save(payload_, persistence_size_);
        if (result == 0) {
            persistence_phase_ = PersistencePhase::ContinueSdWrite;
            return record_maintenance_progress() ? -EAGAIN : -EPERM;
        }
        sd_result_ = result;
        if (result == -ENODEV || result == -EIO || result == -ENXIO ||
            result == -EBADF) {
            sd_available_ = false;
        }
        return finish_persistence();

    case PersistencePhase::ContinueSdWrite:
        result = dima::file_storage_continue_save();
        if (result == -EAGAIN) {
            return record_maintenance_progress() ? -EAGAIN : -EPERM;
        }
        sd_result_ = result;
        if (result == 0 &&
            persistence_kind_ != PersistenceKind::SdMirror) {
            mark_generation_committed();
        } else if (result == -ENODEV || result == -EIO ||
                   result == -ENXIO || result == -EBADF) {
            sd_available_ = false;
        }
        // 只有 SD 已完整提交同代快照后，才允许擦除已满或损坏的 Flash；
        // 即使中途掉电，也必须始终保留至少一份可恢复副本。
        if (persistence_kind_ != PersistenceKind::SdMirror &&
            (flash_result_ == -ENOSPC || flash_result_ == -EIO) &&
            sd_result_ == 0 && flashfs_ready_) {
            persistence_phase_ = PersistencePhase::BeginFlashErase;
            return record_maintenance_progress() ? -EAGAIN : -EPERM;
        }
        return finish_persistence();

    case PersistencePhase::BeginFlashErase:
        result = flashfs_.begin_erase_all(
            dima::parameters::FLASH_TOKEN_PARAMS);
        if (result == -EBUSY) {
            return -EAGAIN;
        }
        if (result != 0) {
            flash_result_ = result;
            return finish_persistence();
        }
        persistence_phase_ = PersistencePhase::ContinueFlashErase;
        return record_maintenance_progress() ? -EAGAIN : -EPERM;

    case PersistencePhase::ContinueFlashErase:
        result = flashfs_.continue_operation();
        if (result == -EBUSY) {
            return -EAGAIN;
        }
        if (result == -EAGAIN) {
            return record_maintenance_progress() ? -EAGAIN : -EPERM;
        }
        if (result != 0) {
            flash_result_ = result;
            return finish_persistence();
        }
        persistence_phase_ = PersistencePhase::BeginFlashRewrite;
        return record_maintenance_progress() ? -EAGAIN : -EPERM;

    case PersistencePhase::BeginFlashRewrite:
        result = flashfs_.begin_write_entry(
            dima::parameters::FLASH_TOKEN_PARAMS, payload_,
            persistence_size_);
        if (result == -EBUSY) {
            return -EAGAIN;
        }
        if (result != 0) {
            flash_result_ = result;
            return finish_persistence();
        }
        persistence_phase_ = PersistencePhase::ContinueFlashRewrite;
        return record_maintenance_progress() ? -EAGAIN : -EPERM;

    case PersistencePhase::ContinueFlashRewrite:
        result = flashfs_.continue_operation();
        if (result == -EBUSY) {
            return -EAGAIN;
        }
        if (result == -EAGAIN) {
            return record_maintenance_progress() ? -EAGAIN : -EPERM;
        }
        flash_result_ = result;
        if (result == 0) {
            mark_generation_committed();
        }
        return finish_persistence();

    case PersistencePhase::Idle:
    default:
        return -EINVAL;
    }
}

int ParameterService::finish_persistence() noexcept
{
    // 普通 Save 的返回值优先反映可用 Flash；Flash 失败但 SD 成功时设置
    // flash_resync_required，后续 autosave 重建 Flash。镜像失败则保留 mirror flag。
    const PersistenceKind kind = persistence_kind_;
    int result = 0;
    if (kind == PersistenceKind::SdMirror) {
        result = sd_result_;
        sd_mirror_required_ = result != 0;
    } else {
        result = flashfs_ready_ ? flash_result_ : sd_result_;
        if (flash_result_ == 0) {
            sd_mirror_required_ = sd_result_ != 0;
        } else if (sd_result_ == 0) {
            sd_mirror_required_ = false;
        }
        flash_resync_required_ = flashfs_ready_ && sd_result_ == 0 &&
                                 flash_result_ != 0;
    }

    if (maintenance_interlock_acquired_) {
        armed_flash_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
    maintenance_.complete(maintenance_ticket_);
    reset_persistence_state();
    return result;
}

void ParameterService::cancel_persistence() noexcept
{
    // 取消必须同时撤销 FlashFS 与 FatFs 临时事务，再释放 interlock/ticket；
    // 任一后端不得继续引用 payload_。
    flashfs_.cancel_operation();
    dima::file_storage_cancel_save();
    if (maintenance_interlock_acquired_) {
        armed_flash_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
    maintenance_.cancel(maintenance_ticket_);
    reset_persistence_state();
}

void ParameterService::reset_persistence_state() noexcept
{
    maintenance_ticket_ = 0U;
    maintenance_progress_ = 0U;
    persistence_generation_ = 0U;
    persistence_size_ = 0U;
    generation_committed_ = false;
    maintenance_interlock_acquired_ = false;
    flash_result_ = 0;
    sd_result_ = 0;
    persistence_kind_ = PersistenceKind::None;
    persistence_phase_ = PersistencePhase::Idle;
}

int ParameterService::storage_save(param_storage_enumerator_t enumerate,
                                   void *enumerate_context,
                                   void *backend_context) noexcept
{
    if (enumerate == nullptr || backend_context == nullptr) {
        return -EINVAL;
    }
    auto &self = *static_cast<ParameterService *>(backend_context);
    dima::platform::MutexGuard lock{self.storage_mutex_};
    if (!lock) {
        return -EDEADLK;
    }
    if (self.persistence_kind_ == PersistenceKind::None &&
        !self.flash_write_allowed()) {
        return -EPERM;
    }
    if (self.persistence_kind_ == PersistenceKind::SdMirror) {
        const int mirror = self.advance_persistence();
        if (mirror == -EAGAIN) {
            return mirror;
        }
    } else if (self.persistence_kind_ != PersistenceKind::None &&
               self.persistence_kind_ != PersistenceKind::Save) {
        return -EBUSY;
    }

    // param 核心用 -EAGAIN 驱动异步保存：首次编码 generation+1 并启动事务，
    // 后续调用只推进状态机。generation 到 UINT32_MAX 时 fail-closed 不回绕。
    if (self.persistence_kind_ == PersistenceKind::None) {
        if (self.storage_generation_ == UINT32_MAX) {
            return -EOVERFLOW;
        }
        const std::uint32_t generation = self.storage_generation_ + 1U;
        std::size_t snapshot_size{};
        const int encoded = self.encode_snapshot(
            enumerate, enumerate_context, self.payload_, generation,
            snapshot_size);
        if (encoded != 0) {
            return encoded;
        }
        const int begun = self.begin_persistence(
            PersistenceKind::Save, snapshot_size, generation);
        return begun == 0 ? -EAGAIN : begun;
    }

    const int persisted = self.advance_persistence();
    if (persisted != 0) {
        return persisted;
    }

    if (self.storage_generation_ == UINT32_MAX) {
        return -EOVERFLOW;
    }
    std::size_t comparison_size{};
    const int encoded = self.encode_snapshot(
        enumerate, enumerate_context, self.comparison_payload_,
        self.storage_generation_ + 1U, comparison_size);
    if (encoded != 0) {
        return encoded;
    }

    // 保存完成后重新枚举当前参数并与已提交 payload 比较；若保存期间又有变化，
    // 返回 -ESTALE 让 autosave 再保存一代，而不谎称最新状态已落盘。
    if (snapshot_codec::payload_matches(
            self.payload_, self.comparison_payload_, comparison_size)) {
        return 0;
    }

    return -ESTALE;
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
    if (self.persistence_kind_ != PersistenceKind::None) {
        return -EBUSY;
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
              self.comparison_payload_, sizeof(self.comparison_payload_),
              sd_size,
              &snapshot_codec::validate, &sd_info)
        : -ENODEV;
    if (sd_result == -ENODEV || sd_result == -EIO ||
        sd_result == -ENXIO || sd_result == -EBADF) {
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

bool ParameterService::flash_write_allowed() const noexcept
{
    // 无论写片内 Flash 还是 SD 镜像，整个持久化事务都禁止在 armed 状态开始。
    return !armed_flash_.armed();
}

int ParameterService::begin_sd_mirror() noexcept
{
    // SD mirror 只能复制经过 FlashFS 重新读取、CRC 验证且 generation 等于当前
    // 已提交代数的快照，不能直接使用可能已被下一次编码改写的旧 RAM 缓冲。
    if (!sd_mirror_required_ || !sd_available_ || !flashfs_ready_ ||
        !flash_write_allowed() ||
        storage_generation_ == 0U ||
        persistence_kind_ != PersistenceKind::None) {
        return -EAGAIN;
    }

    const int loaded = flashfs_.read_entry(
        dima::parameters::FLASH_TOKEN_PARAMS, payload_, sizeof(payload_));
    if (loaded < 0) {
        return loaded;
    }
    snapshot_codec::SnapshotInfo info{};
    const int valid = snapshot_codec::validate(
        payload_, static_cast<std::size_t>(loaded), &info);
    if (valid != 0 || info.generation != storage_generation_) {
        return valid != 0 ? valid : -EAGAIN;
    }
    return begin_persistence(PersistenceKind::SdMirror,
                             static_cast<std::size_t>(loaded),
                             info.generation);
}

} // namespace dima::modules::parameters
