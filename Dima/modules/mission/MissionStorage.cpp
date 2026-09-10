#define MODULE_NAME "mission"
#include "MissionService.hpp"

#include "parameters/FileStorage.hpp"
#include "api/AtomicFileStore.hpp"
#include "api/Time.hpp"
#include "logging/logging.hpp"

#include <cerrno>

namespace dima::modules::mission {
namespace {

bool retry_storage(int error) noexcept
{
    return error == -EAGAIN || error == -EBUSY || error == -EDEADLK;
}

} // namespace

int MissionService::validate_sd_snapshot(
    const std::uint8_t *data, std::size_t size, void *context) noexcept
{
    auto &self = *static_cast<MissionService *>(context);
    // SD 的主文件/备份/临时文件均通过同一完整快照校验；不需要 Flash 标记。
    return snapshot::decode(data, size, self.media_plan_, self.media_identity_);
}

int MissionService::inspect_sd_locked() noexcept
{
    std::size_t size = 0U;
    const int result = dima::file_storage_load(
        dima::platform::AtomicFileDomain::Mission,
        storage_workspace_.data(), storage_workspace_.size(), size,
        &MissionService::validate_sd_snapshot, this);
    if (retry_storage(result)) {
        return result;
    }
    media_snapshot_valid_ = result == 0;
    sd_copy_current_ = media_snapshot_valid_ && have_ram_snapshot_ &&
        snapshot::same_identity(media_identity_, identity_);
    // 没有任务文件或文件格式无效不等于无卡，可继续用该介质保存新任务。
    sd_available_ = result == 0 || result == -ENOENT || result == -EBADMSG ||
                    result == -EINVAL || result == -EFBIG;
    if (!sd_available_) {
        last_persistence_error_ = result;
    }
    return sd_available_ ? 0 : result;
}

void MissionService::load_sd_state() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || operation_ != Operation::LoadSd) {
        return;
    }
    if (retry_storage(inspect_sd_locked())) {
        return;
    }
    if (media_snapshot_valid_ && repository_.activate(media_plan_)) {
        identity_ = media_identity_;
        persisted_current_ = media_plan_.current;
        have_ram_snapshot_ = true;
        sd_copy_current_ = true;
        destination_ = Backend::Sd;
    } else {
        // 无卡启动就是新的空 RAM 仓库，禁止从旧 Flash 或其他后端恢复任务。
        repository_.clear_active();
        destination_ = Backend::Ram;
    }
    initial_load_complete_ = true;
    storage_available_ = true; // 固定容量 RAM 后端不依赖 SD 是否可用。
    operation_ = Operation::Idle;
    execution_state_ = repository_.committed()
        ? MissionExecutionState::NotStarted : MissionExecutionState::NoMission;
}

void MissionService::poll_storage() noexcept
{
    dima::platform::MutexGuard guard{mutex_, dima::platform::Timeout::no_wait()};
    if (!guard || operation_ != Operation::Idle || !initial_load_complete_ ||
        stage_result_valid_ || commit_result_valid_ || armed_.armed()) {
        return;
    }
    const std::uint64_t now = hrt_absolute_time();
    const std::uint64_t interval = last_persistence_error_ == -ENOSPC
        ? 60000000U : kStoragePollIntervalUs;
    if (last_storage_poll_us_ != 0U && now >= last_storage_poll_us_ &&
        now - last_storage_poll_us_ < interval) {
        return;
    }
    last_storage_poll_us_ = now;
    if (retry_storage(inspect_sd_locked())) {
        return;
    }
    if (!sd_available_) {
        destination_ = Backend::Ram;
        return;
    }
    if (!have_ram_snapshot_ && media_snapshot_valid_) {
        // 尚未接收过 RAM 任务时允许未解锁热插卡恢复；已上传或已清空的 RAM
        // 快照（包括空任务）具有本次运行期优先权，不能被卡中旧任务替换。
        if (repository_.activate(media_plan_)) {
            identity_ = media_identity_;
            persisted_current_ = media_plan_.current;
            have_ram_snapshot_ = true;
            sd_copy_current_ = true;
            destination_ = Backend::Sd;
            execution_state_ = repository_.committed()
                ? MissionExecutionState::NotStarted : MissionExecutionState::NoMission;
        }
    }
    if (!have_ram_snapshot_ || sd_copy_current_ || !armed_.begin_maintenance()) {
        return;
    }
    // 只把本次已确认的 RAM 任务复制到恢复后的 SD；保留逻辑持久入口，
    // 不把 AUTO 已推进的 current 写回，也不产生用户请求的最终 ACK。
    maintenance_interlock_acquired_ = true;
    repository_.active_plan(commit_plan_);
    commit_plan_.current = persisted_current_;
    commit_kind_ = CommitKind::MediaSync;
    operation_ = Operation::PrepareState;
}

void MissionService::prepare_state_commit() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || operation_ != Operation::PrepareState ||
        (active_token_ == 0U && commit_kind_ != CommitKind::MediaSync)) {
        return;
    }
    if (armed_.armed()) {
        fail_transaction_locked(-EPERM, false);
        return;
    }
    if ((commit_kind_ == CommitKind::Upload || commit_kind_ == CommitKind::Clear) &&
        !repository_.copy_staging(commit_plan_)) {
        fail_transaction_locked(-EINVAL, false);
        return;
    }
    pending_identity_ = identity_;
    if (commit_kind_ != CommitKind::MediaSync) {
        if (identity_.generation == UINT64_MAX) {
            fail_transaction_locked(-EOVERFLOW, false);
            return;
        }
        pending_identity_.generation = identity_.generation + 1U;
    }
    operation_ = Operation::BeginSnapshotWrite;
}

void MissionService::begin_snapshot_write() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || operation_ != Operation::BeginSnapshotWrite || sd_operation_owned_) {
        return;
    }
    if (armed_.armed()) {
        fail_transaction_locked(-EPERM, false);
        return;
    }
    if (retry_storage(inspect_sd_locked())) {
        return;
    }
    destination_ = sd_available_ ? Backend::Sd : Backend::Ram;
    if (sd_available_ && media_snapshot_valid_ &&
        media_identity_.generation >= pending_identity_.generation) {
        // 新插卡可能带有另一代任务；当前 RAM 任务不热替换，写回时使用新的
        // 文件代次，避免同代不同内容。代次不允许回绕。
        if (media_identity_.generation == UINT64_MAX) {
            fail_transaction_locked(-EOVERFLOW, false);
            return;
        }
        pending_identity_.generation = media_identity_.generation + 1U;
    }
    std::uint32_t mission_id = 0U;
    const int encoded = snapshot::encode(
        commit_plan_, pending_identity_.generation,
        storage_workspace_.data(), storage_workspace_.size(), snapshot_size_,
        mission_id, pending_identity_);
    if (encoded != 0 ||
        ((commit_kind_ == CommitKind::SetCurrent || commit_kind_ == CommitKind::MediaSync) &&
         mission_id != commit_plan_.mission_id)) {
        fail_transaction_locked(encoded != 0 ? encoded : -EBADMSG, false);
        return;
    }
    commit_plan_.mission_id = mission_id;
    if (destination_ == Backend::Ram) {
        // 无可用 SD 时直接发布完整 RAM 任务，不调用任何持久写入接口。
        complete_state_write_locked(0);
        return;
    }
    const int result = dima::file_storage_begin_save(
        dima::platform::AtomicFileDomain::Mission, storage_workspace_.data(), snapshot_size_);
    if (retry_storage(result) || result == -ESTALE) {
        // 尚未借出缓冲；下轮先重建本域 LoadedSource，再重新编码。
        return;
    }
    if (result == 0) {
        sd_operation_owned_ = true;
        operation_ = Operation::ContinueSdWrite;
        return;
    }
    sd_available_ = false;
    destination_ = Backend::Ram;
    last_persistence_error_ = result;
    PX4_WARN("mission: SD save unavailable (%d); RAM only", result);
    complete_state_write_locked(0);
}

void MissionService::continue_sd_write() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || operation_ != Operation::ContinueSdWrite || !sd_operation_owned_) {
        return;
    }
    if (armed_.armed()) {
        dima::file_storage_cancel_save(dima::platform::AtomicFileDomain::Mission);
        sd_operation_owned_ = false;
        fail_transaction_locked(-EPERM, false);
        return;
    }
    const int result = dima::file_storage_continue_save(dima::platform::AtomicFileDomain::Mission);
    if (retry_storage(result)) {
        return;
    }
    sd_operation_owned_ = false;
    if (result != 0) {
        // 文件事务已终止并归还借用缓冲后，才按 RAM-only 语义完成任务。
        // 日志明确报告未持久化，不向 Flash 写入正文、版本号或后备副本。
        sd_available_ = false;
        destination_ = Backend::Ram;
        last_persistence_error_ = result;
        PX4_WARN("mission: SD save failed (%d); RAM only", result);
    }
    complete_state_write_locked(0);
}

void MissionService::cancel_persistence() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || operation_ != Operation::CancelPersistence) {
        return;
    }
    if (sd_operation_owned_) {
        dima::file_storage_cancel_save(dima::platform::AtomicFileDomain::Mission);
        sd_operation_owned_ = false;
    }
    repository_.abort_staging();
    stage_result_ = {};
    commit_result_ = {};
    stage_result_valid_ = false;
    commit_result_valid_ = false;
    active_token_ = 0U;
    pending_item_sequence_ = 0U;
    pending_identity_ = {};
    commit_plan_ = {};
    snapshot_size_ = 0U;
    commit_kind_ = CommitKind::None;
    operation_ = Operation::Idle;
    release_maintenance_locked();
}

} // namespace dima::modules::mission
