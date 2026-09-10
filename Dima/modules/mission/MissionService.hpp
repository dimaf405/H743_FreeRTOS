#pragma once

#include "MissionRepository.hpp"
#include "MissionSnapshot.hpp"
#include "api/Flash.hpp"
#include "api/Synchronization.hpp"
#include "lifecycle/module_base.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace dima::modules::mission {

enum class MissionExecutionState : std::uint8_t {
    NoMission,
    NotStarted,
    Active,
    Complete,
};

struct MissionStatus {
    std::uint32_t mission_id{0U};
    std::uint16_t current{0U};
    std::uint16_t count{0U};
    bool committed{false};
    bool storage_available{false};
    bool loaded{false};
    bool mutation_in_progress{false};
    MissionExecutionState execution_state{MissionExecutionState::NoMission};
};

struct MissionStageResult {
    std::uint32_t token{0U};
    std::uint16_t sequence{0U};
    bool complete{false};
    int error{0};
};

struct MissionCommitResult {
    std::uint32_t token{0U};
    std::uint32_t mission_id{0U};
    std::uint16_t count{0U};
    int error{0};
};

/**
 * PX4 Dataman Mission 语义在 H743 上的固定内存实现。
 *
 * 有可用 SD 时保存完整任务快照，无卡时只保存在 RAM，重启后丢失。
 * 不读写旧任务 Flash 记录。MAVLink/导航仅访问短临界区 API，SD 文件事务
 * 与取消操作由 wq:storage 推进，热插卡不会替换正在运行的任务。
 */
class MissionService final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    MissionService(dima::platform::Synchronization &synchronization,
                   dima::platform::ArmedFlashCoordinator &armed) noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

    int begin_upload(std::uint16_t count, std::uint32_t &token) noexcept;
    int stage_item(std::uint32_t token, const MissionItem &item) noexcept;
    int poll_stage_result(std::uint32_t token,
                          MissionStageResult &result) noexcept;
    void abort_upload(std::uint32_t token) noexcept;
    int request_clear(std::uint32_t &token) noexcept;
    int set_current(std::uint16_t sequence,
                    std::uint32_t &token) noexcept;

    int status(MissionStatus &status) noexcept;
    int item(std::uint16_t sequence, MissionItem &item,
             MissionStatus &status) noexcept;
    int active_plan(MissionPlan &plan) noexcept;
    int poll_commit_result(std::uint32_t token,
                           MissionCommitResult &result) noexcept;
    int start_execution() noexcept;
    int suspend_execution(std::uint32_t mission_id) noexcept;
    int complete_execution(std::uint32_t mission_id,
                           std::uint16_t final_sequence) noexcept;

    // PX4 只在上传、清空和 MISSION_SET_CURRENT 时持久化 Mission State；
    // AUTO 正常推进仅更新运行快照，重启后仍从已持久化的执行入口恢复。
    int advance_current(std::uint32_t mission_id,
                        std::uint16_t reached_sequence) noexcept;

protected:
    void Run() override;

private:
    enum class Backend : std::uint8_t { Ram, Sd };

    enum class Operation : std::uint8_t {
        Idle,
        LoadSd,
        Receiving,
        AwaitItemResult,
        PrepareState,
        BeginSnapshotWrite,
        ContinueSdWrite,
        CancelPersistence,
    };

    enum class CommitKind : std::uint8_t {
        None,
        Upload,
        Clear,
        SetCurrent,
        MediaSync,
    };

    static constexpr std::uint32_t kRunIntervalUs = 20000U;
    static constexpr std::size_t kStorageWorkspaceCapacity = 4096U;
    static constexpr std::uint64_t kStoragePollIntervalUs = 3000000U;

    static int validate_sd_snapshot(const std::uint8_t *data,
                                    std::size_t size, void *context) noexcept;
    bool mutation_allowed_locked() const noexcept;
    void load_sd_state() noexcept;
    int inspect_sd_locked() noexcept;
    void poll_storage() noexcept;
    void complete_stage_locked(int error) noexcept;
    void prepare_state_commit() noexcept;
    void begin_snapshot_write() noexcept;
    void continue_sd_write() noexcept;
    void cancel_persistence() noexcept;
    void complete_state_write_locked(int error) noexcept;
    void fail_transaction_locked(int error, bool report_stage_result) noexcept;
    void release_maintenance_locked() noexcept;
    void reset_runtime_locked() noexcept;
    std::uint32_t allocate_token_locked() noexcept;
    void fill_status_locked(MissionStatus &status) const noexcept;

    dima::platform::Synchronization &synchronization_;
    // 此协调器仅用于禁止任务变更期间突然 Arm，不用于任务 Flash 存储。
    dima::platform::ArmedFlashCoordinator &armed_;
    dima::platform::Mutex mutex_{};
    MissionRepository repository_{};
    MissionPlan commit_plan_{};
    MissionPlan media_plan_{};
    snapshot::Identity identity_{};
    snapshot::Identity pending_identity_{};
    snapshot::Identity media_identity_{};
    std::array<std::uint8_t, kStorageWorkspaceCapacity> storage_workspace_{};
    std::size_t snapshot_size_{0U};
    std::uint64_t last_storage_poll_us_{0U};
    int last_persistence_error_{0};
    std::uint16_t persisted_current_{0U};
    std::uint16_t pending_item_sequence_{0U};
    MissionStageResult stage_result_{};
    MissionCommitResult commit_result_{};
    std::uint32_t active_token_{0U};
    std::uint32_t next_token_{1U};
    Backend destination_{Backend::Ram};
    Operation operation_{Operation::Idle};
    CommitKind commit_kind_{CommitKind::None};
    bool stage_result_valid_{false};
    bool commit_result_valid_{false};
    bool initial_load_complete_{false};
    bool storage_available_{false};
    bool maintenance_interlock_acquired_{false};
    bool sd_operation_owned_{false};
    bool have_ram_snapshot_{false};
    bool media_snapshot_valid_{false};
    bool sd_available_{false};
    bool sd_copy_current_{false};
    MissionExecutionState execution_state_{MissionExecutionState::NoMission};
    // 生命周期由应用线程修改、worker/调用方读取；事务与缓冲所有权仍由 mutex 保护。
    std::atomic<dima::middleware::lifecycle::ModuleState> state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::modules::mission
