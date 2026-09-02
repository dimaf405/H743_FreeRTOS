#pragma once

#include "MissionRepository.hpp"
#include "api/Flash.hpp"
#include "api/Synchronization.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/flashfs.h"
#include "parameters/param.h"
#include "work_queue/ScheduledWorkItem.hpp"

#include <array>
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
 * SYS_DM_BACKEND=0 使用板载 FlashFS，=1 使用非持久 RAM，=-1 禁用 Dataman。
 * 上传逐项写入非活动 bank，只有最后一项成功后提交 Mission State 才切换 active；
 * 因而任何中途失败或掉电都继续引用上一 bank。MAVLink/导航仅访问短临界区 API，
 * Flash 分步操作全部由 wq:storage 的 Run() 推进。
 */
class MissionService final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    MissionService(dima::parameters::FlashFS &flashfs,
                   dima::platform::Synchronization &synchronization,
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
    enum class Backend : std::int8_t {
        Disabled = -1,
        Persistent = 0,
        Ram = 1,
    };

    enum class Operation : std::uint8_t {
        Idle,
        LoadState,
        LoadItems,
        Receiving,
        BeginItemWrite,
        ContinueItemWrite,
        AwaitItemResult,
        PrepareState,
        BeginStateWrite,
        ContinueStateWrite,
    };

    enum class CommitKind : std::uint8_t {
        None,
        Upload,
        Clear,
        SetCurrent,
    };

    // Dataman 持久项使用显式定宽布局；FlashFS 记录自身提供 CRC 与最终 commit
    // marker，这里只保存 PX4 Mission 所需的 bank/count/current/mission_id。
    struct DatamanMissionState {
        std::uint32_t mission_id{0U};
        std::uint16_t count{0U};
        std::uint16_t current{0U};
        std::uint8_t active_bank{0U};
        std::uint8_t reserved[3]{};
    };
    static_assert(sizeof(DatamanMissionState) == 12U,
                  "Dataman mission state layout changed");

    // MissionItem 的编译器 padding 不得进入持久格式；该 20-byte 布局等价承载
    // 当前 Rover NAV_WAYPOINT 子集，单位分别为 1e-7 deg、m 和 MAV_FRAME 枚举。
    struct DatamanMissionItem {
        std::int32_t latitude_e7{0};
        std::int32_t longitude_e7{0};
        float altitude_m{0.0F};
        float acceptance_radius_m{0.0F};
        std::uint16_t sequence{0U};
        std::uint8_t frame{0U};
        std::uint8_t reserved{0U};
    };
    static_assert(sizeof(DatamanMissionItem) == 20U,
                  "Dataman mission item layout changed");

    static constexpr std::uint32_t kRunIntervalUs = 20000U;
    static constexpr std::size_t kMissionIdWorkspaceCapacity = 4096U;
    static constexpr std::uint8_t kBankCount = 2U;
    static constexpr dima::parameters::flash_file_token_t
        kMissionStateToken{{'d', 'm', 's', 't'}};

    static bool backend_supported(std::int32_t value) noexcept;
    static bool state_valid(const DatamanMissionState &state) noexcept;
    static dima::parameters::flash_file_token_t item_token(
        std::uint8_t bank, std::uint16_t sequence) noexcept;
    static DatamanMissionItem encode_item(const MissionItem &item) noexcept;
    static MissionItem decode_item(const DatamanMissionItem &item) noexcept;

    bool mutation_allowed_locked() const noexcept;
    void load_initial_state() noexcept;
    void load_next_item() noexcept;
    void finish_initial_load_locked(int error) noexcept;
    void begin_item_write() noexcept;
    void continue_item_write() noexcept;
    void complete_item_write_locked(int error) noexcept;
    void prepare_state_commit() noexcept;
    void begin_state_write() noexcept;
    void continue_state_write() noexcept;
    void complete_state_write_locked(int error) noexcept;
    void fail_transaction_locked(int error,
                                 bool report_stage_result) noexcept;
    void release_maintenance_locked() noexcept;
    void reset_runtime_locked() noexcept;
    std::uint32_t allocate_token_locked() noexcept;
    void fill_status_locked(MissionStatus &status) const noexcept;

    dima::parameters::FlashFS &flashfs_;
    dima::platform::Synchronization &synchronization_;
    dima::platform::ArmedFlashCoordinator &armed_;
    dima::platform::Mutex mutex_{};
    dima::ParamInt<dima::params::SYS_DM_BACKEND> backend_parameter_{};
    MissionRepository repository_{};
    MissionPlan load_plan_{};
    MissionPlan commit_plan_{};
    std::array<std::array<DatamanMissionItem, kMissionCapacity>, kBankCount>
        ram_banks_{};
    DatamanMissionState ram_state_{};
    DatamanMissionState pending_state_{};
    DatamanMissionItem pending_item_{};
    std::array<std::uint8_t, kMissionIdWorkspaceCapacity>
        mission_id_workspace_{};
    MissionStageResult stage_result_{};
    MissionCommitResult commit_result_{};
    std::uint32_t active_token_{0U};
    std::uint32_t next_token_{1U};
    std::uint16_t load_sequence_{0U};
    std::uint8_t active_bank_{0U};
    std::uint8_t transfer_bank_{1U};
    Backend backend_{Backend::Disabled};
    Operation operation_{Operation::Idle};
    CommitKind commit_kind_{CommitKind::None};
    bool stage_result_valid_{false};
    bool commit_result_valid_{false};
    bool initial_load_complete_{false};
    bool storage_available_{false};
    bool maintenance_interlock_acquired_{false};
    bool flash_operation_owned_{false};
    MissionExecutionState execution_state_{MissionExecutionState::NoMission};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::modules::mission
