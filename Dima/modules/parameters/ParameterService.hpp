#pragma once

#include "lifecycle/module_base.hpp"
#include "maintenance/RuntimeMaintenanceCoordinator.hpp"
#include "parameter_update.hpp"
#include "parameters/flashfs.h"
#include "parameters/autosave.h"
#include "parameters/param.h"
#include "platform/api/Execution.hpp"
#include "platform/api/Flash.hpp"
#include "platform/api/ParameterFileStore.hpp"
#include "platform/api/Synchronization.hpp"
#include "uorb/Publication.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::parameters {

class ParameterService final : public dima::middleware::lifecycle::ModuleBase,
                               public px4::ScheduledWorkItem {
public:
    ParameterService(
        dima::parameters::FlashFS &flashfs,
        dima::platform::ParameterFileStore &parameter_files,
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
    static constexpr std::uint32_t kPollUs = 10000U;
    static constexpr std::uint64_t kSdPollIntervalUs = 3000000ULL;
    static constexpr std::size_t kSnapshotHeaderBytes = 20U;
    static constexpr std::size_t kPayloadCapacity =
        px4::parameter_storage_max_bytes + kSnapshotHeaderBytes;

    enum class PersistenceKind : std::uint8_t {
        None,
        Save,
        SdMirror,
    };

    enum class PersistencePhase : std::uint8_t {
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
    dima::platform::ParameterFileStore &parameter_files_;
    dima::platform::ArmedFlashCoordinator &armed_flash_;
    dima::platform::Synchronization &synchronization_;
    dima::platform::CriticalSection &critical_;
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        &maintenance_;
    dima::platform::RecursiveMutex param_mutex_{};
    dima::platform::RecursiveMutex storage_mutex_{};
    ParamAutosave autosave_;
    uORB::Publication<parameter_update_s> parameter_update_pub_{
        ORB_ID(parameter_update)};
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
};

} // namespace dima::modules::parameters
