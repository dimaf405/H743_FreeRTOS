/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "containers/atomic.h"
#include "platform/api/Flash.hpp"
#include "platform/api/Time.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

class ParamAutosave : public px4::ScheduledWorkItem
{
public:
    explicit ParamAutosave(
        dima::platform::ArmedFlashCoordinator &armed_flash) noexcept;
    void request() noexcept;
    void enable(bool enable) noexcept;
    void stop() noexcept;
    int saveNow(bool blocking = true) noexcept;
    bool enabled() const noexcept;
    bool pending() const noexcept { return _scheduled.load(); }
    hrt_abstime lastAutosave() const noexcept;

private:
    void Run() override;
    bool writeAllowed() const noexcept { return !_armed_flash.armed(); }

    dima::platform::ArmedFlashCoordinator &_armed_flash;
    hrt_abstime _last_attempt_timestamp{0};
    hrt_abstime _last_success_timestamp{0};
    px4::atomic_bool _scheduled{false};
    int _retry_count{0};
    bool _disabled{false};
};

// Upstream path: src/lib/parameters/autosave.h @ d6f12ad1
