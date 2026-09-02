#pragma once

#include "lifecycle/module_base.hpp"
#include "maintenance/RuntimeMaintenanceCoordinator.hpp"
#include "parameter_update.hpp"
#include "parameters/flashfs.h"
#include "parameters/autosave.h"
#include "parameters/param.h"
#include <parameters/parameter_contract.hpp>
#include "api/Execution.hpp"
#include "api/Flash.hpp"
#include "api/AtomicFileStore.hpp"
#include "api/Synchronization.hpp"
#include "uORB/Publication.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::parameters {

// 参数服务桥接生成参数表、运行时分层、FlashFS 主副本、FatFs SD 镜像与 autosave。
// 所有持久化采用固定缓冲和非阻塞状态机；参数定义/列表仍只由权威源生成。
class ParameterService final : public dima::middleware::lifecycle::ModuleBase,
                               public px4::ScheduledWorkItem {
public:
    ParameterService(
        dima::parameters::FlashFS &flashfs,
        dima::platform::AtomicFileStore &atomic_files,
        dima::platform::ArmedFlashCoordinator &armed_flash,
        dima::platform::Synchronization &synchronization,
        dima::platform::CriticalSection &critical,
        dima::middleware::maintenance::
            RuntimeMaintenanceCoordinator &maintenance) noexcept;

    bool init() noexcept;
    bool shutdown() noexcept;
    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

private:
    // 10 ms 服务 autosave/通知，3 s 探测 SD；快照为 20 B header + 生成参数层
    // 允许的最大 payload，两份 32-byte 对齐缓冲用于已提交/当前比较。
    static constexpr std::uint32_t kPollUs = 10000U;
    static constexpr std::uint64_t kSdPollIntervalUs = 3000000ULL;
    // 无 card-detect GPIO 时，首次成功挂载后再等待 500 ms 才写镜像，避免机械触点
    // 尚未稳定就立即进入 FAT 元数据事务；真正写入前仍会再次检查介质状态。
    static constexpr std::uint64_t kSdMountSettleUs = 500000ULL;
    static constexpr std::size_t kSnapshotHeaderBytes = 20U;
    static constexpr std::size_t kPayloadCapacity =
        dima::generated::parameters::kParameterStorageMaxBytes +
        kSnapshotHeaderBytes;

    enum class PersistenceKind : std::uint8_t {
        None,
        Save,
        SdMirror,
    };

    enum class PersistencePhase : std::uint8_t {
        // Wait approval -> Flash write -> SD write；Flash 满/损坏且 SD 已有同代副本
        // 时才允许 erase+rewrite Flash。每个 Continue 阶段均可跨多次 Run。
        Idle,
        WaitForApproval,
        BeginFlashWrite,
        ContinueFlashWrite,
        BeginSdWrite,
        ContinueSdWrite,
        BeginFlashErase,
        ContinueFlashErase,
        BeginFlashRewrite,
        ContinueFlashRewrite,
    };

    static void lock_params(void *context) noexcept;
    static void unlock_params(void *context) noexcept;
    static void notify_params(const parameter_update_s *source,
                              void *context) noexcept;
    static int storage_load(param_storage_visitor_t visitor,
                            void *visitor_context,
                            void *backend_context) noexcept;
    static int storage_save(param_storage_enumerator_t enumerate,
                            void *enumerate_context,
                            void *backend_context) noexcept;
    static int storage_status(param_storage_status_s *status,
                              void *backend_context) noexcept;
    static void cancel_async_save(void *context) noexcept;

    int encode_snapshot(param_storage_enumerator_t enumerate,
                        void *enumerate_context, std::uint8_t *destination,
                        std::uint32_t generation,
                        std::size_t &snapshot_size) noexcept;
    int begin_persistence(PersistenceKind kind, std::size_t snapshot_size,
                          std::uint32_t generation) noexcept;
    int advance_persistence() noexcept;
    int finish_persistence() noexcept;
    void cancel_persistence() noexcept;
    void reset_persistence_state() noexcept;
    bool record_maintenance_progress() noexcept;
    void mark_generation_committed() noexcept;
    int begin_sd_mirror() noexcept;
    void service_sd_mirror() noexcept;
    void Run() override;
    bool flash_write_allowed() const noexcept;
    bool parameters_unsaved() const noexcept;
    void poll_sd_card() noexcept;
    void reset_runtime_state() noexcept;

    static const param_storage_backend_s storage_backend_;

    dima::parameters::FlashFS &flashfs_;
    dima::platform::AtomicFileStore &atomic_files_;
    dima::platform::ArmedFlashCoordinator &armed_flash_;
    dima::platform::Synchronization &synchronization_;
    dima::platform::CriticalSection &critical_;
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        &maintenance_;
    // param_mutex 保护参数核心的可重入调用；storage_mutex 独立串行化保存/加载/
    // SD 镜像，避免长介质事务阻塞普通 param_get。
    dima::platform::RecursiveMutex param_mutex_{};
    dima::platform::RecursiveMutex storage_mutex_{};
    ParamAutosave autosave_;
    uORB::Publication<parameter_update_s> parameter_update_pub_{
        ORB_ID(parameter_update)};
    // payload_ 在异步 Flash/SD 完成前保持所有权；comparison_payload_ 用于启动时
    // 比较双介质和保存后检测参数是否在事务期间再次变化。
    alignas(32) std::uint8_t payload_[kPayloadCapacity]{};
    alignas(32) std::uint8_t comparison_payload_[kPayloadCapacity]{};
    parameter_update_s pending_update_{};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool initialized_{false};
    bool loading_{false};
    bool update_pending_{false};
    bool autosave_request_pending_{false};
    bool flashfs_ready_{false};
    bool sd_available_{false};
    bool flash_resync_required_{false};
    bool sd_mirror_required_{false};
    bool generation_committed_{false};
    bool maintenance_interlock_acquired_{false};
    std::uint32_t storage_generation_{0U};
    std::uint32_t persistence_generation_{0U};
    std::uint32_t maintenance_progress_{0U};
    std::size_t persistence_size_{0U};
    int flash_result_{0};
    int sd_result_{0};
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator::Ticket
        maintenance_ticket_{0U};
    PersistenceKind persistence_kind_{PersistenceKind::None};
    PersistencePhase persistence_phase_{PersistencePhase::Idle};
    std::uint64_t last_sd_poll_us_{0U};
    std::uint64_t last_sd_mirror_attempt_us_{0U};
    std::uint64_t sd_mirror_ready_after_us_{0U};
};

} // namespace dima::modules::parameters
