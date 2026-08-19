#pragma once

#include "lifecycle/module_base.hpp"
#include "parameter_update.hpp"
#include "parameters/ParameterJournal.hpp"
#include "parameters/autosave.h"
#include "parameters/param.h"
#include "platform/api/Platform.hpp"
#include "uorb/Publication.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::parameters {

class ParameterService final : public dima::middleware::lifecycle::ModuleBase,
                               public px4::ScheduledWorkItem {
public:
    ParameterService(
        dima::parameters::ParameterJournal &journal,
        dima::platform::ArmedFlashCoordinator &armed_flash,
        dima::platform::Synchronization &synchronization,
        dima::platform::CriticalSection &critical) noexcept;

    bool init() noexcept;
    bool shutdown() noexcept;
    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

private:
    static constexpr std::uint32_t kPollUs = 10000U;
    static constexpr std::size_t kPayloadCapacity =
        px4::parameter_storage_max_bytes;

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
    static int storage_erase(void *backend_context) noexcept;
    static int storage_status(param_storage_status_s *status,
                              void *backend_context) noexcept;

    void Run() override;
    bool flash_write_allowed() const noexcept;
    bool migrate_serial_configuration(bool existing_storage) noexcept;
    bool migrate_serial_schema_v1() noexcept;
    void reset_runtime_state() noexcept;

    static const param_storage_backend_s storage_backend_;

    dima::parameters::ParameterJournal &journal_;
    dima::platform::ArmedFlashCoordinator &armed_flash_;
    dima::platform::Synchronization &synchronization_;
    dima::platform::CriticalSection &critical_;
    dima::platform::RecursiveMutex param_mutex_{};
    dima::platform::RecursiveMutex storage_mutex_{};
    ParamAutosave autosave_;
    uORB::Publication<parameter_update_s> parameter_update_pub_{
        ORB_ID(parameter_update)};
    alignas(32) std::uint8_t payload_[kPayloadCapacity]{};
    parameter_update_s pending_update_{};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool initialized_{false};
    bool loading_{false};
    bool update_pending_{false};
    bool autosave_request_pending_{false};
    std::int32_t stored_serial_schema_version_{0};
    bool stored_legacy_rc_port_present_{false};
    std::int32_t stored_legacy_rc_port_{0};
    bool stored_legacy_baud_present_[8]{};
    std::int32_t stored_legacy_baud_[8]{};
    std::int32_t stored_schema1_baud_[7]{};
    std::int32_t stored_schema1_function_[7]{};
};

} // namespace dima::modules::parameters
