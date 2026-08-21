#define MODULE_NAME "param"
#include "ParameterService.hpp"

#include "logging/logging.hpp"
#include "parameters/Crc32.hpp"
#include "parameters/FileStorage.hpp"
#include "parameters/QgcCompatibility.hpp"
#include "parameters/flashparams/flashparams.h"
#include "platform/api/Time.hpp"

#include <cerrno>
#include <cstring>

namespace dima::modules::parameters {
namespace {

struct FilteredLoadContext {
    param_storage_visitor_t visitor;
    void *visitor_context;
};

struct SnapshotHeader {
    std::uint32_t magic;
    std::uint32_t format;
    std::uint32_t generation;
    std::uint32_t payload_size;
    std::uint32_t payload_crc;
};
static_assert(sizeof(SnapshotHeader) == 20U);

struct SnapshotInfo {
    std::uint32_t generation{0U};
    std::uint32_t payload_crc{0U};
    std::size_t payload_size{0U};
};

constexpr std::uint32_t kSnapshotMagic = 0x5041524DU;
constexpr std::uint32_t kSnapshotFormat = 1U;

int inspect_snapshot(const std::uint8_t *data, std::size_t size,
                     SnapshotInfo &info) noexcept
{
    if (data == nullptr || size < sizeof(SnapshotHeader)) {
        return -EILSEQ;
    }
    SnapshotHeader header{};
    std::memcpy(&header, data, sizeof(header));
    if (header.magic != kSnapshotMagic || header.format != kSnapshotFormat ||
        header.generation == 0U ||
        header.payload_size != size - sizeof(SnapshotHeader) ||
        header.payload_size > px4::parameter_storage_max_bytes) {
        return -EILSEQ;
    }
    const auto *payload = data + sizeof(SnapshotHeader);
    if (dima::parameters::crc32(payload, header.payload_size) !=
        header.payload_crc) {
        return -EILSEQ;
    }
    info.generation = header.generation;
    info.payload_crc = header.payload_crc;
    info.payload_size = header.payload_size;
    return 0;
}

int validate_snapshot(const std::uint8_t *data, std::size_t size,
                      void *context) noexcept
{
    SnapshotInfo info{};
    const int result = inspect_snapshot(data, size, info);
    if (result != 0) {
        return result;
    }

    if (info.payload_size == 0U) {
        if (context != nullptr) {
            *static_cast<SnapshotInfo *>(context) = info;
        }
        return 0;
    }

    const auto validate_parameter = [](const char *name, param_type_t type,
                                       const void *value,
                                       void *) noexcept {
        if (name == nullptr || value == nullptr) {
            return -EINVAL;
        }
        const param_t parameter = param_find_no_notification(name);
        if (parameter == PARAM_INVALID) {
            return -ENOENT;
        }
        return param_type(parameter) == type ? 0 : -EINVAL;
    };
    const int decoded = flashparams_decode_buffer(
        data + sizeof(SnapshotHeader), info.payload_size,
        validate_parameter, nullptr);
    if (decoded != 0) {
        return decoded;
    }
    if (context != nullptr) {
        *static_cast<SnapshotInfo *>(context) = info;
    }
    return 0;
}

bool is_qgc_compatibility_parameter(const char *name) noexcept
{
    return dima::parameters::qgc_fixed_int32_parameter(name) != nullptr;
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
        is_disabled_mode_compatibility_parameter(name)) {
        return 0;
    }

    return filtered.visitor(name, type, value, filtered.visitor_context);
}

} // namespace

const param_storage_backend_s ParameterService::storage_backend_{
    &ParameterService::storage_load,
    &ParameterService::storage_save,
    &ParameterService::storage_status,
};

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

    dima::platform::CriticalGuard guard{self.critical_};
    self.pending_update_ = update;
    self.update_pending_ = true;
    if (!self.loading_) {
        self.autosave_request_pending_ = true;
    }
}

int ParameterService::encode_snapshot(
    param_storage_enumerator_t enumerate, void *enumerate_context,
    std::uint8_t *destination, std::uint32_t generation,
    std::size_t &snapshot_size) noexcept
{
    if (enumerate == nullptr || destination == nullptr || generation == 0U) {
        return -EINVAL;
    }

    std::size_t payload_size{};
    const int encoded = flashparams_encode_buffer(
        destination + kSnapshotHeaderBytes,
        kPayloadCapacity - kSnapshotHeaderBytes,
        enumerate, enumerate_context, &payload_size);
    if (encoded != 0) {
        return encoded;
    }

    const SnapshotHeader header{
        kSnapshotMagic,
        kSnapshotFormat,
        generation,
        static_cast<std::uint32_t>(payload_size),
        dima::parameters::crc32(
            destination + kSnapshotHeaderBytes, payload_size),
    };
    std::memcpy(destination, &header, sizeof(header));
    snapshot_size = kSnapshotHeaderBytes + payload_size;
    return 0;
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
    generation_committed_ = kind == PersistenceKind::SdMirror;
    flash_result_ = flashfs_ready_ ? -EINPROGRESS : -ENODEV;
    sd_result_ = sd_available_ ? -EINPROGRESS : -ENODEV;
    return 0;
}

bool ParameterService::record_maintenance_progress() noexcept
{
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
        if (persistence_kind_ != PersistenceKind::SdMirror &&
            (flash_result_ == -ENOSPC || flash_result_ == -EIO) &&
            sd_result_ == 0 && flashfs_ready_) {
            persistence_phase_ = PersistencePhase::BeginFlashErase;
            return record_maintenance_progress() ? -EAGAIN : -EPERM;
        }
        return finish_persistence();

    case PersistencePhase::BeginFlashErase:
        result = flashfs_.begin_erase_all();
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

    SnapshotHeader persisted_header{};
    SnapshotHeader comparison_header{};
    std::memcpy(&persisted_header, self.payload_, sizeof(persisted_header));
    std::memcpy(&comparison_header, self.comparison_payload_,
                sizeof(comparison_header));
    const bool current_values_persisted =
        persisted_header.payload_size == comparison_header.payload_size &&
        persisted_header.payload_crc == comparison_header.payload_crc &&
        comparison_size ==
            kSnapshotHeaderBytes + persisted_header.payload_size &&
        std::memcmp(self.payload_ + kSnapshotHeaderBytes,
                    self.comparison_payload_ + kSnapshotHeaderBytes,
                    persisted_header.payload_size) == 0;
    if (current_values_persisted) {
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
                                    SnapshotInfo &info) noexcept {
        if (!self.flashfs_ready_) {
            return -ENODEV;
        }
        const int loaded = self.flashfs_.read_entry(
            dima::parameters::FLASH_TOKEN_PARAMS,
            destination, kPayloadCapacity);
        return loaded < 0
                   ? loaded
                   : validate_snapshot(destination,
                                       static_cast<std::size_t>(loaded), &info);
    };

    SnapshotInfo flash_info{};
    const int flash_result = read_flash(self.payload_, flash_info);

    SnapshotInfo sd_info{};
    std::size_t sd_size{};
    const int sd_result = self.sd_available_
        ? dima::file_storage_load(
              self.comparison_payload_, sizeof(self.comparison_payload_),
              sd_size,
              &validate_snapshot, &sd_info)
        : -ENODEV;
    if (sd_result == -ENODEV || sd_result == -EIO ||
        sd_result == -ENXIO || sd_result == -EBADF) {
        self.sd_available_ = false;
    }

    SnapshotInfo selected{};
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

    FilteredLoadContext filtered{visitor, visitor_context};
    return flashparams_decode_buffer(
        selected_payload + kSnapshotHeaderBytes, selected.payload_size,
        load_mutable_parameter, &filtered);
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
    return !armed_flash_.armed();
}

bool ParameterService::parameters_unsaved() const noexcept
{
    for (param_t parameter = 0U; parameter < param_count(); ++parameter) {
        if (param_value_unsaved(parameter)) {
            return true;
        }
    }
    return false;
}

int ParameterService::begin_sd_mirror() noexcept
{
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
    SnapshotInfo info{};
    const int valid = validate_snapshot(
        payload_, static_cast<std::size_t>(loaded), &info);
    if (valid != 0 || info.generation != storage_generation_) {
        return valid != 0 ? valid : -EAGAIN;
    }
    return begin_persistence(PersistenceKind::SdMirror,
                             static_cast<std::size_t>(loaded),
                             info.generation);
}

void ParameterService::service_sd_mirror() noexcept
{
    dima::platform::MutexGuard lock{storage_mutex_};
    if (!lock) {
        return;
    }

    if (persistence_kind_ == PersistenceKind::SdMirror) {
        const int result = advance_persistence();
        if (result != 0 && result != -EAGAIN && result != -EPERM) {
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
        sd_mirror_required_ = flashfs_ready_ && storage_generation_ != 0U;
        (void)autosave_.resume_after_storage_available();
    } else {
        sd_mirror_required_ = flashfs_ready_ && storage_generation_ != 0U;
        PX4_WARN("param: SD card removed; FlashFS remains active");
    }
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
