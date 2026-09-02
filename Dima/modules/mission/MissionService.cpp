#define MODULE_NAME "mission"
#include "MissionService.hpp"

#include "MissionCodec.hpp"

#include "logging/logging.hpp"

#include <cerrno>

namespace dima::modules::mission {
namespace {

bool storage_failure(int error) noexcept
{
    return error == -ENODEV || error == -EIO || error == -ENXIO ||
           error == -ETIMEDOUT;
}

} // namespace

MissionService::MissionService(
    dima::parameters::FlashFS &flashfs,
    dima::platform::Synchronization &synchronization,
    dima::platform::ArmedFlashCoordinator &armed) noexcept
    : ScheduledWorkItem("mission", px4::wq_configurations::storage),
      flashfs_(flashfs), synchronization_(synchronization), armed_(armed)
{
}

bool MissionService::backend_supported(std::int32_t value) noexcept
{
    // 数值严格对应 PX4 SYS_DM_BACKEND：-1 禁用、0 板级默认持久存储、1 RAM。
    return value == static_cast<std::int32_t>(Backend::Disabled) ||
           value == static_cast<std::int32_t>(Backend::Persistent) ||
           value == static_cast<std::int32_t>(Backend::Ram);
}

bool MissionService::state_valid(
    const DatamanMissionState &state) noexcept
{
    if (state.active_bank >= kBankCount || state.count > kMissionCapacity) {
        return false;
    }
    if (state.count == 0U) {
        return state.current == 0U && state.mission_id == 0U;
    }
    return state.current < state.count && state.mission_id != 0U;
}

dima::parameters::flash_file_token_t MissionService::item_token(
    std::uint8_t bank, std::uint16_t sequence) noexcept
{
    // PX4 的 (DM_KEY_WAYPOINTS_OFFBOARD_0/1, index) 二维键映射为 FlashFS
    // 4-byte token。bank/index 直接进入 token，不维护第二份手写条目清单。
    dima::parameters::flash_file_token_t token{};
    token.bytes[0] = static_cast<std::uint8_t>('d');
    token.bytes[1] = static_cast<std::uint8_t>('m');
    token.bytes[2] = bank;
    token.bytes[3] = static_cast<std::uint8_t>(sequence);
    return token;
}

MissionService::DatamanMissionItem MissionService::encode_item(
    const MissionItem &item) noexcept
{
    DatamanMissionItem stored{};
    stored.latitude_e7 = item.latitude_e7;
    stored.longitude_e7 = item.longitude_e7;
    stored.altitude_m = item.altitude_m;
    stored.acceptance_radius_m = item.acceptance_radius_m;
    stored.sequence = item.sequence;
    stored.frame = item.frame;
    return stored;
}

MissionItem MissionService::decode_item(
    const DatamanMissionItem &item) noexcept
{
    MissionItem restored{};
    restored.sequence = item.sequence;
    restored.latitude_e7 = item.latitude_e7;
    restored.longitude_e7 = item.longitude_e7;
    restored.altitude_m = item.altitude_m;
    restored.acceptance_radius_m = item.acceptance_radius_m;
    restored.frame = item.frame;
    return restored;
}

bool MissionService::start() noexcept
{
    static_assert(kMissionIdWorkspaceCapacity >= codec::kFileCapacity,
                  "Mission ID workspace is smaller than codec output");
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!backend_parameter_.bind() ||
        !backend_supported(backend_parameter_.get())) {
        backend_parameter_.invalidate();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    if (!mutex_.valid() && !mutex_.initialize(synchronization_)) {
        backend_parameter_.invalidate();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    {
        dima::platform::MutexGuard guard{mutex_};
        if (!guard) {
            backend_parameter_.invalidate();
            state_ = dima::middleware::lifecycle::ModuleState::Error;
            return false;
        }
        reset_runtime_locked();
        backend_ = static_cast<Backend>(backend_parameter_.get());

        if (backend_ == Backend::Disabled) {
            // 与 PX4 rcS 不启动 Dataman 等价：回读为空，所有变更被拒绝。
            initial_load_complete_ = true;
            storage_available_ = false;
            operation_ = Operation::Idle;
        } else if (backend_ == Backend::Ram) {
            // RAM backend 每次模块启动都从 bank 0 的空 Mission State 开始，
            // 与 PX4 dataman start -r 的非持久生命周期一致。
            ram_banks_ = {};
            ram_state_ = {};
            active_bank_ = 0U;
            transfer_bank_ = 1U;
            initial_load_complete_ = true;
            storage_available_ = true;
            operation_ = Operation::Idle;
        } else {
            // 默认 backend 先只读取 Mission State；其引用的 active bank 随后按
            // index 分周期恢复，任何 inactive bank 残留都不会进入运行快照。
            operation_ = Operation::LoadState;
        }
    }

    if (!ScheduleEnable() ||
        !ScheduleOnInterval(kRunIntervalUs, kRunIntervalUs)) {
        ScheduleCancelAndDrain();
        dima::platform::MutexGuard guard{mutex_};
        if (guard) {
            reset_runtime_locked();
        }
        backend_parameter_.invalidate();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void MissionService::stop() noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    if (mutex_.valid()) {
        dima::platform::MutexGuard guard{mutex_};
        if (guard) {
            // 只有 begin_write_entry 成功后才拥有 FlashFS 在途操作；停止时禁止
            // cancel 其他模块刚取得的 Parameter/DroneCAN 写事务。
            if (flash_operation_owned_) {
                flashfs_.cancel_operation();
                flash_operation_owned_ = false;
            }
            reset_runtime_locked();
        }
    }
    backend_parameter_.invalidate();
}

dima::middleware::lifecycle::ModuleState MissionService::state() const noexcept
{
    return state_;
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
           backend_ != Backend::Disabled && initial_load_complete_ &&
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
        if (backend_ == Backend::Disabled || !storage_available_) {
            return -ENODEV;
        }
        return -EBUSY;
    }

    // 从第一项到 Mission State commit 始终持有 maintenance 门，确保整个
    // inactive-bank 事务期间 Commander 无法跨 WorkQueue 突然 Arm。
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
    transfer_bank_ = static_cast<std::uint8_t>(active_bank_ ^ 1U);
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
    if (armed_.armed()) {
        // 与异步 Flash 写错误走同一个完成槽，MAVLink 会在 WaitingItemWrite
        // 消费错误；不能一边同步返回错误，一边遗留无人消费的 commit result。
        fail_transaction_locked(-EPERM, true);
        return 0;
    }
    if (!repository_.stage_item(item)) {
        return -EINVAL;
    }

    pending_item_ = encode_item(item);
    if (backend_ == Backend::Ram) {
        ram_banks_[transfer_bank_][item.sequence] = pending_item_;
        complete_item_write_locked(0);
    } else if (backend_ == Backend::Persistent) {
        operation_ = Operation::BeginItemWrite;
    } else {
        fail_transaction_locked(-ENODEV, true);
        return 0;
    }
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
        // 协议层确认已经观察到当前 dm_write 结果后，才允许请求下一项或提交
        // Mission State；链路在此之前断开仍可完整 abort 当前 inactive bank。
        operation_ = result.complete ? Operation::PrepareState
                                     : Operation::Receiving;
    }
    return 0;
}

void MissionService::abort_upload(std::uint32_t token) noexcept
{
    // 取消只回收尚未提交的 inactive bank；active bank 和 Mission State 不变。
    // mutex 覆盖 cancel，避免 Run() 在取消后继续推进其他模块新取得的 FlashFS 操作。
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || token == 0U || token != active_token_ ||
        (commit_kind_ != CommitKind::Upload &&
         commit_kind_ != CommitKind::Clear)) {
        return;
    }
    if (flash_operation_owned_) {
        flashfs_.cancel_operation();
        flash_operation_owned_ = false;
    }
    repository_.abort_staging();
    stage_result_ = {};
    commit_result_ = {};
    stage_result_valid_ = false;
    commit_result_valid_ = false;
    pending_item_ = {};
    pending_state_ = {};
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
        if (backend_ == Backend::Disabled || !storage_available_) {
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

    // PX4 MISSION_SET_CURRENT 更新 DM_KEY_MISSION_STATE，而不改 waypoint bank。
    // RAM active 只有 state commit 成功后才切换 current，失败仍保留旧执行入口。
    maintenance_interlock_acquired_ = true;
    commit_plan_.current = sequence;
    transfer_bank_ = active_bank_;
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

void MissionService::finish_initial_load_locked(int error) noexcept
{
    if (error == 0 && !repository_.activate(load_plan_)) {
        error = -EBADMSG;
    }
    if (error != 0) {
        repository_.clear_active();
        load_plan_ = {};
        active_bank_ = 0U;
    }

    initial_load_complete_ = true;
    // 格式损坏只使当前任务为空，Flash backend 本身仍可接收下一次 PX4 bank
    // 事务；物理 Flash 错误才关闭 storage_available 并拒绝修改。
    storage_available_ = !storage_failure(error);
    operation_ = Operation::Idle;
    load_sequence_ = 0U;
    execution_state_ = repository_.committed()
                           ? MissionExecutionState::NotStarted
                           : MissionExecutionState::NoMission;

    if (error == 0 && repository_.committed()) {
        PX4_INFO("mission: loaded bank=%u count=%u id=%lu",
                 static_cast<unsigned int>(active_bank_),
                 static_cast<unsigned int>(repository_.active_count()),
                 static_cast<unsigned long>(repository_.mission_id()));
    } else if (error != 0) {
        PX4_WARN("mission: Dataman state invalid: %d", error);
    }
}

void MissionService::load_initial_state() noexcept
{
    DatamanMissionState restored{};
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || backend_ != Backend::Persistent ||
        operation_ != Operation::LoadState) {
        return;
    }

    const int loaded = flashfs_.read_entry(
        kMissionStateToken, &restored, sizeof(restored));
    if (loaded == -EDEADLK || loaded == -EBUSY) {
        return;
    }
    if (loaded == -ENOENT) {
        // PX4 Dataman 的初始 Mission State 指向 bank 0、count=0。FlashFS 首次
        // 使用可直接采用同一逻辑默认值，第一次修改会写入正式 state 记录。
        load_plan_ = {};
        active_bank_ = 0U;
        transfer_bank_ = 1U;
        finish_initial_load_locked(0);
        return;
    }
    if (loaded != static_cast<int>(sizeof(restored)) ||
        !state_valid(restored)) {
        finish_initial_load_locked(
            loaded < 0 ? loaded : -EBADMSG);
        return;
    }

    active_bank_ = restored.active_bank;
    transfer_bank_ = static_cast<std::uint8_t>(active_bank_ ^ 1U);
    load_plan_ = {};
    load_plan_.count = restored.count;
    load_plan_.current = restored.current;
    load_plan_.mission_id = restored.mission_id;
    if (restored.count == 0U) {
        finish_initial_load_locked(0);
    } else {
        load_sequence_ = 0U;
        operation_ = Operation::LoadItems;
    }
}

void MissionService::load_next_item() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || backend_ != Backend::Persistent ||
        operation_ != Operation::LoadItems ||
        load_sequence_ >= load_plan_.count) {
        return;
    }

    DatamanMissionItem stored{};
    const int loaded = flashfs_.read_entry(
        item_token(active_bank_, load_sequence_),
        &stored, sizeof(stored));
    if (loaded == -EDEADLK || loaded == -EBUSY) {
        return;
    }
    if (loaded != static_cast<int>(sizeof(stored)) ||
        stored.reserved != 0U || stored.sequence != load_sequence_) {
        finish_initial_load_locked(
            loaded < 0 ? loaded : -EBADMSG);
        return;
    }

    load_plan_.items[load_sequence_] = decode_item(stored);
    ++load_sequence_;
    if (load_sequence_ == load_plan_.count) {
        finish_initial_load_locked(0);
    }
}

void MissionService::begin_item_write() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || backend_ != Backend::Persistent ||
        operation_ != Operation::BeginItemWrite ||
        flash_operation_owned_) {
        return;
    }
    if (armed_.armed()) {
        fail_transaction_locked(-EPERM, true);
        return;
    }

    const int result = flashfs_.begin_write_entry(
        item_token(transfer_bank_, pending_item_.sequence),
        &pending_item_, sizeof(pending_item_));
    if (result == -EBUSY || result == -EDEADLK) {
        return;
    }
    if (result != 0) {
        fail_transaction_locked(result, true);
        return;
    }
    flash_operation_owned_ = true;
    operation_ = Operation::ContinueItemWrite;
}

void MissionService::continue_item_write() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || backend_ != Backend::Persistent ||
        operation_ != Operation::ContinueItemWrite ||
        !flash_operation_owned_) {
        return;
    }
    if (armed_.armed()) {
        flashfs_.cancel_operation();
        flash_operation_owned_ = false;
        fail_transaction_locked(-EPERM, true);
        return;
    }

    const int result = flashfs_.continue_operation();
    if (result == -EAGAIN || result == -EBUSY || result == -EDEADLK) {
        return;
    }
    flash_operation_owned_ = false;
    complete_item_write_locked(result);
}

void MissionService::complete_item_write_locked(int error) noexcept
{
    if (error != 0) {
        fail_transaction_locked(error, true);
        return;
    }

    // PX4 在 dm_write(index) 成功后才推进 _transfer_seq。这里把完成槽交给
    // MAVLink，下一条 MISSION_REQUEST_INT 只能在该槽被消费后发送。
    stage_result_.token = active_token_;
    stage_result_.sequence = pending_item_.sequence;
    stage_result_.complete = repository_.staging_complete();
    stage_result_.error = 0;
    stage_result_valid_ = true;
    pending_item_ = {};
    operation_ = Operation::AwaitItemResult;
}

void MissionService::prepare_state_commit() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || operation_ != Operation::PrepareState ||
        active_token_ == 0U) {
        return;
    }
    if (armed_.armed()) {
        fail_transaction_locked(-EPERM, false);
        return;
    }

    if (commit_kind_ == CommitKind::Upload ||
        commit_kind_ == CommitKind::Clear) {
        if (!repository_.copy_staging(commit_plan_)) {
            fail_transaction_locked(-EINVAL, false);
            return;
        }
        std::size_t encoded_size{};
        std::uint32_t mission_id{};
        const int encoded = codec::encode(
            commit_plan_, mission_id_workspace_.data(),
            mission_id_workspace_.size(), encoded_size, mission_id);
        if (encoded != 0) {
            fail_transaction_locked(encoded, false);
            return;
        }
        commit_plan_.mission_id = mission_id;
    } else if (commit_kind_ != CommitKind::SetCurrent) {
        fail_transaction_locked(-EINVAL, false);
        return;
    }

    pending_state_ = {};
    pending_state_.active_bank = transfer_bank_;
    pending_state_.count = commit_plan_.count;
    pending_state_.current = commit_plan_.current;
    pending_state_.mission_id = commit_plan_.mission_id;
    if (!state_valid(pending_state_)) {
        fail_transaction_locked(-EINVAL, false);
        return;
    }

    if (backend_ == Backend::Ram) {
        complete_state_write_locked(0);
    } else if (backend_ == Backend::Persistent) {
        operation_ = Operation::BeginStateWrite;
    } else {
        fail_transaction_locked(-ENODEV, false);
    }
}

void MissionService::begin_state_write() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || backend_ != Backend::Persistent ||
        operation_ != Operation::BeginStateWrite ||
        flash_operation_owned_) {
        return;
    }
    if (armed_.armed()) {
        fail_transaction_locked(-EPERM, false);
        return;
    }

    // Mission State 是唯一原子切换点。此前写入 inactive bank 的 item 即使完整，
    // 在该 FlashFS 记录 commit marker 成功前也不会成为 active 任务。
    const int result = flashfs_.begin_write_entry(
        kMissionStateToken, &pending_state_, sizeof(pending_state_));
    if (result == -EBUSY || result == -EDEADLK) {
        return;
    }
    if (result != 0) {
        fail_transaction_locked(result, false);
        return;
    }
    flash_operation_owned_ = true;
    operation_ = Operation::ContinueStateWrite;
}

void MissionService::continue_state_write() noexcept
{
    dima::platform::MutexGuard guard{mutex_};
    if (!guard || backend_ != Backend::Persistent ||
        operation_ != Operation::ContinueStateWrite ||
        !flash_operation_owned_) {
        return;
    }
    if (armed_.armed()) {
        flashfs_.cancel_operation();
        flash_operation_owned_ = false;
        fail_transaction_locked(-EPERM, false);
        return;
    }

    const int result = flashfs_.continue_operation();
    if (result == -EAGAIN || result == -EBUSY || result == -EDEADLK) {
        return;
    }
    flash_operation_owned_ = false;
    complete_state_write_locked(result);
}

void MissionService::complete_state_write_locked(int error) noexcept
{
    if (error == 0 && !repository_.activate(commit_plan_)) {
        error = -EINVAL;
    }
    if (error == 0) {
        active_bank_ = pending_state_.active_bank;
        transfer_bank_ = static_cast<std::uint8_t>(active_bank_ ^ 1U);
        if (backend_ == Backend::Ram) {
            ram_state_ = pending_state_;
        }
        execution_state_ = repository_.committed()
                               ? MissionExecutionState::NotStarted
                               : MissionExecutionState::NoMission;
    } else {
        // State commit 失败时 active bank 从未切换；只丢弃 staging，运行任务保持
        // 上一代。inactive bank 的已写 item 与 PX4 Dataman 一样可被下一次覆盖。
        repository_.abort_staging();
        if (storage_failure(error)) {
            storage_available_ = false;
        }
    }

    commit_result_.token = active_token_;
    commit_result_.error = error;
    commit_result_.mission_id = error == 0 ? repository_.mission_id() : 0U;
    commit_result_.count = error == 0 ? repository_.active_count() : 0U;
    commit_result_valid_ = true;
    active_token_ = 0U;
    operation_ = Operation::Idle;
    commit_kind_ = CommitKind::None;
    commit_plan_ = {};
    pending_state_ = {};
    release_maintenance_locked();
}

void MissionService::fail_transaction_locked(
    int error, bool report_stage_result) noexcept
{
    if (storage_failure(error)) {
        storage_available_ = false;
    }
    repository_.abort_staging();

    if (report_stage_result) {
        stage_result_.token = active_token_;
        stage_result_.sequence = pending_item_.sequence;
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
    pending_item_ = {};
    pending_state_ = {};
    commit_plan_ = {};
    flash_operation_owned_ = false;
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
    load_plan_ = {};
    commit_plan_ = {};
    ram_banks_ = {};
    ram_state_ = {};
    pending_state_ = {};
    pending_item_ = {};
    stage_result_ = {};
    commit_result_ = {};
    active_token_ = 0U;
    load_sequence_ = 0U;
    active_bank_ = 0U;
    transfer_bank_ = 1U;
    backend_ = Backend::Disabled;
    operation_ = Operation::Idle;
    commit_kind_ = CommitKind::None;
    stage_result_valid_ = false;
    commit_result_valid_ = false;
    initial_load_complete_ = false;
    storage_available_ = false;
    flash_operation_owned_ = false;
    execution_state_ = MissionExecutionState::NoMission;
}

void MissionService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    Operation operation = Operation::Idle;
    {
        dima::platform::MutexGuard guard{
            mutex_, dima::platform::Timeout::no_wait()};
        if (!guard) {
            return;
        }
        operation = operation_;
    }

    switch (operation) {
    case Operation::LoadState:
        load_initial_state();
        break;
    case Operation::LoadItems:
        load_next_item();
        break;
    case Operation::BeginItemWrite:
        begin_item_write();
        break;
    case Operation::ContinueItemWrite:
        continue_item_write();
        break;
    case Operation::PrepareState:
        prepare_state_commit();
        break;
    case Operation::BeginStateWrite:
        begin_state_write();
        break;
    case Operation::ContinueStateWrite:
        continue_state_write();
        break;
    case Operation::Idle:
    case Operation::Receiving:
    case Operation::AwaitItemResult:
        break;
    }
}

} // namespace dima::modules::mission
