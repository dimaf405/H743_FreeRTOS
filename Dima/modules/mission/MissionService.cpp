#define MODULE_NAME "mission"
#include "MissionService.hpp"

#include "MissionCodec.hpp"
#include "api/Services.hpp"
#include "api/TaskRuntime.hpp"

#include "logging/logging.hpp"

#include <cerrno>

namespace dima::modules::mission {
MissionService::MissionService(
    dima::platform::Synchronization &synchronization,
    dima::platform::ArmedFlashCoordinator &armed) noexcept
    : ScheduledWorkItem("mission", px4::wq_configurations::storage),
      synchronization_(synchronization), armed_(armed)
{
}

bool MissionService::start() noexcept
{
    static_assert(kStorageWorkspaceCapacity >= codec::kFileCapacity + snapshot::kOverhead,
                  "Mission storage workspace is too small");
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!mutex_.valid() && !mutex_.initialize(synchronization_)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    {
        dima::platform::MutexGuard guard{mutex_};
        if (!guard) {
            state_ = dima::middleware::lifecycle::ModuleState::Error;
            return false;
        }
        reset_runtime_locked();
        // 仅从 SD 恢复任务；无卡时 RAM 仓库为空，不读取任何旧 Flash 任务。
        // 文件 I/O 留给 storage worker，启动线程不阻塞在 SD 初始化上。
        operation_ = Operation::LoadSd;
    }
    if (!ScheduleEnable() || !ScheduleOnInterval(kRunIntervalUs, kRunIntervalUs)) {
        ScheduleCancelAndDrain();
        dima::platform::MutexGuard guard{mutex_};
        if (guard) {
            reset_runtime_locked();
        }
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void MissionService::stop() noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    bool pending = false;
    if (mutex_.valid()) {
        dima::platform::MutexGuard guard{mutex_};
        if (guard) {
            if (sd_operation_owned_) {
                operation_ = Operation::CancelPersistence;
                pending = true;
            } else {
                // Run 可能已读取旧阶段并通过 Running 检查；在同一互斥区撤销
                // 待执行阶段并关闭后台探测，避免 drain 期间再次借出 SD 缓冲。
                operation_ = Operation::Idle;
                initial_load_complete_ = false;
            }
        }
    }
    if (pending) {
        // 关闭/回滚 SD 文件也可能执行 I/O，必须交给 storage worker；停止
        // 调用者只等待借出的缓冲归还，不能从 appMain 或 MAVLink 越界取消文件事务。
        (void)ScheduleNow();
        while (pending) {
            {
                dima::platform::MutexGuard guard{mutex_};
                pending = !guard || operation_ == Operation::CancelPersistence;
            }
            if (pending) {
                dima::platform::services().tasks.delay(dima::platform::Timeout::from_ms(1U));
            }
        }
    }
    ScheduleCancelAndDrain();
    if (mutex_.valid()) {
        dima::platform::MutexGuard guard{mutex_};
        if (guard) {
            reset_runtime_locked();
        }
    }
}

dima::middleware::lifecycle::ModuleState MissionService::state() const noexcept
{
    return state_.load();
}

std::uint32_t MissionService::allocate_token_locked() noexcept
{
    const std::uint32_t token = next_token_ == 0U ? 1U : next_token_;
    next_token_ = token == UINT32_MAX ? 1U : token + 1U;
    return token;
}

bool MissionService::mutation_allowed_locked() const noexcept
{
    return state_ == dima::middleware::lifecycle::ModuleState::Running &&
           initial_load_complete_ &&
           storage_available_ && !armed_.armed() &&
           operation_ == Operation::Idle && !stage_result_valid_ &&
           !commit_result_valid_;
}

int MissionService::begin_upload(std::uint16_t count,
                                 std::uint32_t &token) noexcept
{
    token = 0U;
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (count > kMissionCapacity) {
        return -ENOSPC;
    }
    if (!mutation_allowed_locked()) {
        if (armed_.armed()) {
            return -EPERM;
        }
        if (!storage_available_) {
            return -ENODEV;
        }
        return -EBUSY;
    }

    // 从第一项到 Mission State commit 始终持有 maintenance 门，确保
    // 整份任务事务期间 Commander 无法跨 WorkQueue 突然 Arm。
    if (!armed_.begin_maintenance()) {
        return armed_.armed() ? -EPERM : -EBUSY;
    }
    maintenance_interlock_acquired_ = true;
    if (!repository_.begin_staging(count)) {
        release_maintenance_locked();
        return -EBUSY;
    }

    active_token_ = allocate_token_locked();
    token = active_token_;
    commit_kind_ = count == 0U ? CommitKind::Clear : CommitKind::Upload;
    operation_ = count == 0U ? Operation::PrepareState
                             : Operation::Receiving;
    return 0;
}

int MissionService::stage_item(std::uint32_t token,
                               const MissionItem &item) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (token == 0U || token != active_token_ ||
        commit_kind_ != CommitKind::Upload ||
        operation_ != Operation::Receiving || stage_result_valid_) {
        return -EBUSY;
    }
    pending_item_sequence_ = item.sequence;
    if (armed_.armed()) {
        // 与持久事务错误走同一个完成槽，MAVLink 会在 WaitingItemWrite
        // 消费错误；不能一边同步返回错误，一边遗留无人消费的 commit result。
        fail_transaction_locked(-EPERM, true);
        return 0;
    }
    if (!repository_.stage_item(item)) {
        return -EINVAL;
    }

    // 单项完成槽只推进上传序列；最终 ACK 等待完整 SD 保存或 RAM 任务发布。
    complete_stage_locked(0);
    return 0;
}

int MissionService::poll_stage_result(
    std::uint32_t token, MissionStageResult &result) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (!stage_result_valid_ || token == 0U ||
        stage_result_.token != token) {
        return -EINPROGRESS;
    }
    result = stage_result_;
    stage_result_ = {};
    stage_result_valid_ = false;
    if (result.error == 0 && operation_ == Operation::AwaitItemResult) {
        // 协议层确认已经观察到当前 staging 结果后，才请求下一项或开始完整
        // 快照提交；链路中断只丢弃 staging，现役 RAM 任务仍保持完整。
        operation_ = result.complete ? Operation::PrepareState
                                     : Operation::Receiving;
    }
    return 0;
}

void MissionService::abort_upload(std::uint32_t token) noexcept
{
    // 取消只回收尚未发布的 staging/SD 事务，现役 RAM 任务保持不变。
    // mutex 覆盖取消请求，文件 I/O 留给 storage worker，防止误取消另一文件域。
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || token == 0U || token != active_token_ ||
        (commit_kind_ != CommitKind::Upload &&
         commit_kind_ != CommitKind::Clear)) {
        return;
    }
    if (sd_operation_owned_) {
        // 请求者可能位于 MAVLink 队列；等待 worker 归还事务缓冲前不释放维护门。
        operation_ = Operation::CancelPersistence;
        (void)ScheduleNow();
        return;
    }
    repository_.abort_staging();
    stage_result_ = {};
    commit_result_ = {};
    stage_result_valid_ = false;
    commit_result_valid_ = false;
    pending_item_sequence_ = 0U;
    commit_plan_ = {};
    active_token_ = 0U;
    operation_ = Operation::Idle;
    commit_kind_ = CommitKind::None;
    release_maintenance_locked();
}

int MissionService::request_clear(std::uint32_t &token) noexcept
{
    return begin_upload(0U, token);
}

int MissionService::set_current(std::uint16_t sequence,
                                std::uint32_t &token) noexcept
{
    token = 0U;
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (!mutation_allowed_locked()) {
        if (armed_.armed()) {
            return -EPERM;
        }
        if (!storage_available_) {
            return -ENODEV;
        }
        return -EBUSY;
    }
    repository_.active_plan(commit_plan_);
    if (sequence >= commit_plan_.count) {
        commit_plan_ = {};
        return -ERANGE;
    }
    if (!armed_.begin_maintenance()) {
        commit_plan_ = {};
        return armed_.armed() ? -EPERM : -EBUSY;
    }

    // 持久 current 与同一完整快照提交；航点内容 ID 不变。
    // RAM active 只有本次 SD/RAM 提交完成后才切换入口，拒绝时保留旧入口。
    maintenance_interlock_acquired_ = true;
    commit_plan_.current = sequence;
    active_token_ = allocate_token_locked();
    token = active_token_;
    commit_kind_ = CommitKind::SetCurrent;
    operation_ = Operation::PrepareState;
    return 0;
}

void MissionService::fill_status_locked(MissionStatus &status) const noexcept
{
    status.mission_id = repository_.mission_id();
    status.current = repository_.current();
    status.count = repository_.active_count();
    status.committed = repository_.committed();
    status.storage_available = storage_available_;
    status.loaded = initial_load_complete_;
    status.mutation_in_progress = operation_ != Operation::Idle ||
                                  stage_result_valid_ ||
                                  commit_result_valid_;
    status.execution_state = execution_state_;
}

int MissionService::status(MissionStatus &status) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    fill_status_locked(status);
    return 0;
}

int MissionService::item(std::uint16_t sequence, MissionItem &item,
                         MissionStatus &status) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    fill_status_locked(status);
    return repository_.active_item(sequence, item) ? 0 : -ERANGE;
}

int MissionService::active_plan(MissionPlan &plan) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    repository_.active_plan(plan);
    return 0;
}

int MissionService::poll_commit_result(
    std::uint32_t token, MissionCommitResult &result) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (!commit_result_valid_ || token == 0U ||
        commit_result_.token != token) {
        return -EINPROGRESS;
    }
    result = commit_result_;
    commit_result_ = {};
    commit_result_valid_ = false;
    return 0;
}

int MissionService::start_execution() noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (!armed_.armed() || !repository_.committed()) {
        return -EPERM;
    }
    if (operation_ != Operation::Idle || stage_result_valid_ ||
        commit_result_valid_) {
        return -EBUSY;
    }
    if (execution_state_ == MissionExecutionState::Active) {
        return -EALREADY;
    }
    if (execution_state_ == MissionExecutionState::Complete &&
        !repository_.set_current(0U)) {
        return -ERANGE;
    }
    execution_state_ = MissionExecutionState::Active;
    return 0;
}

int MissionService::suspend_execution(std::uint32_t mission_id) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (mission_id == 0U || mission_id != repository_.mission_id()) {
        return -ESTALE;
    }
    if (execution_state_ == MissionExecutionState::Active) {
        execution_state_ = MissionExecutionState::NotStarted;
    }
    return 0;
}

int MissionService::complete_execution(
    std::uint32_t mission_id, std::uint16_t final_sequence) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    const std::uint16_t count = repository_.active_count();
    if (mission_id == 0U || mission_id != repository_.mission_id() ||
        execution_state_ != MissionExecutionState::Active) {
        return -ESTALE;
    }
    if (count == 0U || final_sequence != repository_.current() ||
        final_sequence + 1U != count) {
        return -ERANGE;
    }
    execution_state_ = MissionExecutionState::Complete;
    return 0;
}

int MissionService::advance_current(std::uint32_t mission_id,
                                    std::uint16_t reached_sequence) noexcept
{
    dima::platform::MutexGuard guard{
        mutex_, dima::platform::Timeout::no_wait()};
    if (!guard) {
        return -EAGAIN;
    }
    if (mission_id == 0U || mission_id != repository_.mission_id() ||
        execution_state_ != MissionExecutionState::Active ||
        reached_sequence != repository_.current()) {
        return -ESTALE;
    }
    const std::uint16_t count = repository_.active_count();
    if (reached_sequence >= count || reached_sequence + 1U >= count) {
        return -ERANGE;
    }
    return repository_.set_current(
               static_cast<std::uint16_t>(reached_sequence + 1U))
               ? 0
               : -ERANGE;
}

void MissionService::complete_stage_locked(int error) noexcept
{
    if (error != 0) {
        fail_transaction_locked(error, true);
        return;
    }

    // 下一条 MISSION_REQUEST_INT 仍由完成槽驱动；整份任务完成 SD 保存
    // 或 RAM 发布后才另发最终 commit result，中途取消保留现役 RAM 任务。
    stage_result_.token = active_token_;
    stage_result_.sequence = pending_item_sequence_;
    stage_result_.complete = repository_.staging_complete();
    stage_result_.error = 0;
    stage_result_valid_ = true;
    pending_item_sequence_ = 0U;
    operation_ = Operation::AwaitItemResult;
}

void MissionService::complete_state_write_locked(int error) noexcept
{
    const bool background = commit_kind_ == CommitKind::MediaSync;
    if (error == 0 && !background && !repository_.activate(commit_plan_)) {
        error = -EINVAL;
    }
    if (error == 0) {
        identity_ = pending_identity_;
        have_ram_snapshot_ = true;
        persisted_current_ = commit_plan_.current;
        sd_copy_current_ = destination_ == Backend::Sd;
        if (sd_copy_current_) {
            last_persistence_error_ = 0;
        }
        if (!background) {
            execution_state_ = repository_.committed()
                ? MissionExecutionState::NotStarted : MissionExecutionState::NoMission;
        }
        // RAM 提交是明确的非持久语义；成功 ACK 表示车辆已接收整份任务，
        // 不把无卡时的成功回包描述成掉电保存成功。
        PX4_INFO("mission: accepted in %s count=%u",
                 sd_copy_current_ ? "SD" : "RAM (volatile)",
                 static_cast<unsigned>(commit_plan_.count));
    } else {
        repository_.abort_staging();
        last_persistence_error_ = error;
        PX4_WARN("mission: commit rejected: %d", error);
    }
    if (!background) {
        commit_result_.token = active_token_;
        commit_result_.error = error;
        commit_result_.mission_id = error == 0 ? repository_.mission_id() : 0U;
        commit_result_.count = error == 0 ? repository_.active_count() : 0U;
        commit_result_valid_ = true;
    }
    active_token_ = 0U;
    operation_ = Operation::Idle;
    commit_kind_ = CommitKind::None;
    commit_plan_ = {};
    pending_identity_ = {};
    snapshot_size_ = 0U;
    release_maintenance_locked();
}

void MissionService::fail_transaction_locked(
    int error, bool report_stage_result) noexcept
{
    if (commit_kind_ == CommitKind::MediaSync) {
        complete_state_write_locked(error);
        return;
    }
    repository_.abort_staging();

    if (report_stage_result) {
        stage_result_.token = active_token_;
        stage_result_.sequence = pending_item_sequence_;
        stage_result_.complete = false;
        stage_result_.error = error;
        stage_result_valid_ = true;
    } else {
        commit_result_.token = active_token_;
        commit_result_.mission_id = 0U;
        commit_result_.count = 0U;
        commit_result_.error = error;
        commit_result_valid_ = true;
    }

    active_token_ = 0U;
    operation_ = Operation::Idle;
    commit_kind_ = CommitKind::None;
    pending_item_sequence_ = 0U;
    commit_plan_ = {};
    sd_operation_owned_ = false;
    release_maintenance_locked();
}

void MissionService::release_maintenance_locked() noexcept
{
    if (maintenance_interlock_acquired_) {
        armed_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
}

void MissionService::reset_runtime_locked() noexcept
{
    release_maintenance_locked();
    repository_.abort_staging();
    repository_.clear_active();
    commit_plan_ = {};
    media_plan_ = {};
    identity_ = {};
    pending_identity_ = {};
    media_identity_ = {};
    pending_item_sequence_ = 0U;
    stage_result_ = {};
    commit_result_ = {};
    active_token_ = 0U;
    snapshot_size_ = 0U;
    last_storage_poll_us_ = 0U;
    last_persistence_error_ = 0;
    persisted_current_ = 0U;
    destination_ = Backend::Ram;
    operation_ = Operation::Idle;
    commit_kind_ = CommitKind::None;
    stage_result_valid_ = false;
    commit_result_valid_ = false;
    initial_load_complete_ = false;
    storage_available_ = false;
    sd_operation_owned_ = false;
    have_ram_snapshot_ = false;
    media_snapshot_valid_ = false;
    sd_available_ = false;
    sd_copy_current_ = false;
    execution_state_ = MissionExecutionState::NoMission;
}

void MissionService::Run()
{
    Operation operation = Operation::Idle;
    {
        dima::platform::MutexGuard guard{mutex_, dima::platform::Timeout::no_wait()};
        if (!guard) {
            return;
        }
        operation = operation_;
    }
    if (state_ != dima::middleware::lifecycle::ModuleState::Running &&
        operation != Operation::CancelPersistence) {
        return;
    }
    // 每轮最多推进一个持久化阶段；运行任务不因介质变化直接切换 RAM 航线。
    switch (operation) {
    case Operation::LoadSd: load_sd_state(); break;
    case Operation::PrepareState: prepare_state_commit(); break;
    case Operation::BeginSnapshotWrite: begin_snapshot_write(); break;
    case Operation::ContinueSdWrite: continue_sd_write(); break;
    case Operation::CancelPersistence: cancel_persistence(); break;
    case Operation::Idle: poll_storage(); break;
    case Operation::Receiving:
    case Operation::AwaitItemResult:
        break;
    }
}

} // namespace dima::modules::mission
