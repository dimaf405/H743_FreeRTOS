/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/
#define MODULE_NAME "param"
#include "autosave.h"

#include "Dima/middleware/events/events.hpp"
#include "Dima/middleware/logging/logging.hpp"
#include "Dima/middleware/parameters/param.h"

#include <cerrno>

namespace {
constexpr uint32_t kDebounceUs = 300000U;
constexpr hrt_abstime kRateLimitUs = 2000000ULL;
constexpr uint32_t kWriteBlockedRetryUs = 1000000U;
constexpr uint32_t kStorageFullEventId = 0x50415201U;

void reportStorageFull() noexcept
{
    const uint32_t arguments[] = {static_cast<uint32_t>(ENOSPC)};
    (void)dima::events::report(kStorageFullEventId,
                               dima::events::Severity::Critical,
                               arguments, 1U);
}
} // namespace

ParamAutosave::ParamAutosave() noexcept
    : ScheduledWorkItem("param-autosave", px4::wq_configurations::lp_default)
{
}

void ParamAutosave::request() noexcept
{
    px4::AtomicTransaction transaction;
    if (_scheduled.load() || _disabled) {
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

void ParamAutosave::enable(bool enable) noexcept
{
    px4::AtomicTransaction transaction;
    _disabled = !enable;
    if (enable) {
        _retry_count = 0;
    } else if (_scheduled.load()) {
        _scheduled.store(false);
        ScheduleClear();
    }
}

void ParamAutosave::stop() noexcept
{
    px4::AtomicTransaction transaction;
    _disabled = true;
    _scheduled.store(false);
    ScheduleClear();
}

bool ParamAutosave::enabled() const noexcept
{
    px4::AtomicTransaction transaction;
    return !_disabled;
}

hrt_abstime ParamAutosave::lastAutosave() const noexcept
{
    px4::AtomicTransaction transaction;
    return _last_success_timestamp;
}

int ParamAutosave::saveNow(bool blocking) noexcept
{
    {
        px4::AtomicTransaction transaction;
        if (_disabled) {
            return -ECANCELED;
        }
        _scheduled.store(false);
        ScheduleClear();
        _last_attempt_timestamp = hrt_absolute_time();
    }

    if (!writeAllowed()) {
        request();
        return -EPERM;
    }

    const int result = param_save_default(blocking);
    bool retry = false;
    bool storage_full = false;
    {
        px4::AtomicTransaction transaction;
        if (result == 0) {
            _last_success_timestamp = hrt_absolute_time();
            _retry_count = 0;
        } else if (result == -ENOSPC) {
            _disabled = true;
            _retry_count = 0;
            storage_full = true;
        } else if (!_disabled) {
            retry = true;
        }
    }

    if (storage_full) {
        reportStorageFull();
        PX4_ERR("parameter storage full (%i), autosave disabled", result);
    } else if (retry) {
        request();
    }
    return result;
}

void ParamAutosave::Run()
{
    {
        px4::AtomicTransaction transaction;
        if (_disabled) {
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
    {
        px4::AtomicTransaction transaction;
        if (result == 0) {
            _last_success_timestamp = hrt_absolute_time();
            _retry_count = 0;
        } else if (result == -ENOSPC) {
            _disabled = true;
            _retry_count = 0;
            storage_full = true;
        } else if (_retry_count < 3) {
            ++_retry_count;
            retry = true;
        } else {
            _retry_count = 0;
            exhausted = true;
        }
    }

    if (storage_full) {
        reportStorageFull();
        PX4_ERR("parameter storage full (%i), autosave disabled", result);
    } else if (retry) {
        PX4_INFO("param auto save unavailable (%i), retrying..", result);
        request();
    } else if (exhausted) {
        PX4_ERR("param auto save failed (%i)", result);
    }
}

// Upstream path: src/lib/parameters/autosave.cpp @ d6f12ad1
