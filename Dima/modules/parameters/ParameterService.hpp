#pragma once

#include "lifecycle/module_base.hpp"
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
// 固定缓冲在后台连续完成快照保存；参数定义/列表仍只由权威源生成。
class ParameterService final : public dima::middleware::lifecycle::ModuleBase,
                               public px4::ScheduledWorkItem {
public:
    ParameterService(
        dima::parameters::FlashFS &flashfs,
        dima::platform::AtomicFileStore &atomic_files,
        dima::platform::ArmedFlashCoordinator &armed_flash,
        dima::platform::Synchronization &synchronization,
        dima::platform::CriticalSection &critical) noexcept;

    bool init() noexcept;
    bool shutdown() noexcept;
    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

private:
    // 10 ms 服务 autosave/通知，3 s 探测 SD；快照为 20 B header + 生成参数层
    // 允许的最大 payload，两份 32-byte 对齐缓冲用于保存与双介质恢复。
    static constexpr std::uint32_t kPollUs = 10000U;
    static constexpr std::uint64_t kSdPollIntervalUs = 3000000ULL;
    // 无 card-detect GPIO 时，首次成功挂载后再等待 500 ms 才写镜像，避免机械触点
    // 尚未稳定就立即进入 FAT 元数据事务；真正写入前仍会再次检查介质状态。
    static constexpr std::uint64_t kSdMountSettleUs = 500000ULL;
    static constexpr std::size_t kSnapshotHeaderBytes = 20U;
    static constexpr std::size_t kPayloadCapacity =
        dima::generated::parameters::kParameterStorageMaxBytes +
        kSnapshotHeaderBytes;

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

    int encode_snapshot(param_storage_enumerator_t enumerate,
                        void *enumerate_context, std::uint8_t *destination,
                        std::uint32_t generation,
                        std::size_t &snapshot_size) noexcept;
    int write_flash_snapshot(std::size_t size) noexcept;
    int write_sd_snapshot(std::size_t size) noexcept;
    int save_sd_mirror() noexcept;
    void service_sd_mirror() noexcept;
    void Run() override;
    bool parameters_unsaved() const noexcept;
    void poll_sd_card() noexcept;
    void reset_runtime_state() noexcept;

    static const param_storage_backend_s storage_backend_;

    dima::parameters::FlashFS &flashfs_;
    dima::platform::AtomicFileStore &atomic_files_;
    dima::platform::ArmedFlashCoordinator &armed_flash_;
    dima::platform::Synchronization &synchronization_;
    dima::platform::CriticalSection &critical_;
    // param_mutex 保护参数核心的可重入调用；storage_mutex 独立串行化保存/加载/
    // SD 镜像，避免长介质事务阻塞普通 param_get。
    dima::platform::RecursiveMutex param_mutex_{};
    dima::platform::RecursiveMutex storage_mutex_{};
    ParamAutosave autosave_;
    uORB::Publication<parameter_update_s> parameter_update_pub_{
        ORB_ID(parameter_update)};
    // payload_ 在整笔保存期间保持稳定；comparison_payload_ 只用于启动/换卡恢复，
    // 保存结束不再编码第二份快照。
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
    std::uint32_t storage_generation_{0U};
    std::uint64_t last_sd_poll_us_{0U};
    std::uint64_t last_sd_mirror_attempt_us_{0U};
    std::uint64_t sd_mirror_ready_after_us_{0U};
};

} // namespace dima::modules::parameters
