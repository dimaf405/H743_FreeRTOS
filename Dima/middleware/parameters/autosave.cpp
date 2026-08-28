/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/
#define MODULE_NAME "param"
#include "autosave.h"

#include "events/events.hpp"
#include "logging/logging.hpp"
#include "param.h"

#include <cerrno>

namespace {
constexpr uint32_t kDebounceUs = 300000U;
constexpr hrt_abstime kRateLimitUs = 2000000ULL;
constexpr uint32_t kWriteBlockedRetryUs = 1000000U;
constexpr uint32_t kAsyncProgressRetryUs = 10000U;
constexpr uint32_t kStorageFullEventId = 0x50415201U;

void reportStorageFull() noexcept
{
    const uint32_t arguments[] = {static_cast<uint32_t>(ENOSPC)};
    (void)dima::events::report(kStorageFullEventId,
                               dima::events::Severity::Critical,
                               arguments, 1U);
}
} // namespace

ParamAutosave::ParamAutosave(
    dima::platform::ArmedFlashCoordinator &armed_flash,
    CancelSaveFn cancel_save, void *cancel_context) noexcept
    : ScheduledWorkItem("param-autosave", px4::wq_configurations::storage),
      _armed_flash(armed_flash), _cancel_save(cancel_save),
      _cancel_context(cancel_context)
{
}

void ParamAutosave::request() noexcept
{
    px4::AtomicTransaction transaction;
    if (_scheduled.load() ||
        _disable_reason != DisableReason::None) {
        return;
    }

    hrt_abstime delay = kDebounceUs;
    if (_last_attempt_timestamp != 0U) {
        const hrt_abstime elapsed = hrt_elapsed_time(&_last_attempt_timestamp);
        if (elapsed < kRateLimitUs && kRateLimitUs > elapsed + delay) {
            delay = kRateLimitUs - elapsed;
        }
    }

    _scheduled.store(true);
    if (!ScheduleDelayed(static_cast<uint32_t>(delay))) {
        _scheduled.store(false);
    }
}

void ParamAutosave::enable() noexcept
{
    px4::AtomicTransaction transaction;
    _disable_reason = DisableReason::None;
    if (!ScheduleEnable()) {
        _disable_reason = DisableReason::Manual;
        return;
    }
    _retry_count = 0;
}

bool ParamAutosave::resume_after_storage_available() noexcept
{
    {
        px4::AtomicTransaction transaction;
        if (_disable_reason != DisableReason::StorageFull) {
            return false;
        }
        if (!ScheduleEnable()) {
            return false;
        }
        _disable_reason = DisableReason::None;
        _retry_count = 0;
    }
    request();
    return pending();
}

void ParamAutosave::stop() noexcept
{
    {
        px4::AtomicTransaction transaction;
        _disable_reason = DisableReason::Manual;
        _scheduled.store(false);
    }
    ScheduleCancelAndDrain();
    px4::AtomicTransaction transaction;
    _last_attempt_timestamp = 0U;
    _last_success_timestamp = 0U;
    _retry_count = 0;
}

bool ParamAutosave::enabled() const noexcept
{
    px4::AtomicTransaction transaction;
    return _disable_reason == DisableReason::None;
}

hrt_abstime ParamAutosave::lastAutosave() const noexcept
{
    px4::AtomicTransaction transaction;
    return _last_success_timestamp;
}

void ParamAutosave::Run()
{
    {
        px4::AtomicTransaction transaction;
        if (_disable_reason != DisableReason::None) {
            _scheduled.store(false);
            return;
        }
        if (!writeAllowed()) {
            if (!ScheduleDelayed(kWriteBlockedRetryUs)) {
                _scheduled.store(false);
            }
            return;
        }
        _scheduled.store(false);
        _last_attempt_timestamp = hrt_absolute_time();
    }

    const int result = param_save_default(false);
    bool retry = false;
    bool exhausted = false;
    bool storage_full = false;
    bool snapshot_stale = false;
    bool cancel_save = false;
    {
        px4::AtomicTransaction transaction;
        if (result == 0) {
            _last_success_timestamp = hrt_absolute_time();
            _retry_count = 0;
        } else if (result == -ENOSPC) {
            _disable_reason = DisableReason::StorageFull;
            _retry_count = 0;
            storage_full = true;
        } else if (result == -EAGAIN || result == -EBUSY) {
            _retry_count = 0;
            _scheduled.store(true);
            if (!ScheduleDelayed(kAsyncProgressRetryUs)) {
                _scheduled.store(false);
                cancel_save = true;
            }
        } else if (result == -EPERM) {
            _retry_count = 0;
            _scheduled.store(true);
            if (!ScheduleDelayed(kWriteBlockedRetryUs)) {
                _scheduled.store(false);
                cancel_save = true;
            }
        } else if (result == -ESTALE) {
            _retry_count = 0;
            retry = true;
            snapshot_stale = true;
        } else if (_retry_count < 3) {
            ++_retry_count;
            retry = true;
        } else {
            _retry_count = 0;
            exhausted = true;
        }
    }

    if (cancel_save && _cancel_save != nullptr) {
        _cancel_save(_cancel_context);
    }
    if (storage_full) {
        reportStorageFull();
        PX4_ERR("parameter storage full (%i), autosave suspended", result);
    } else if (snapshot_stale) {
        PX4_INFO("parameters changed during save; scheduling fresh snapshot");
        request();
    } else if (retry) {
        PX4_INFO("param auto save unavailable (%i), retrying..", result);
        request();
    } else if (exhausted) {
        PX4_ERR("param auto save failed (%i)", result);
    }
}

// Upstream path: src/lib/parameters/autosave.cpp @ d6f12ad1
