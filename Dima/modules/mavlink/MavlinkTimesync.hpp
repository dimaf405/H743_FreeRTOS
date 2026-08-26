#pragma once
/*
 * MAVLink TIMESYNC 处理器，语义来自 PX4-Autopilot v1.17.0。
 *
 * tc1==0 表示远端发起：用本机 HRT 纳秒时间回填 tc1 并原样回显 ts1；tc1>0 表示
 * 本机发起请求的返回包：把往返样本交给 Timesync 滤波器。本平台没有 RTC，因此不处理
 * SYSTEM_TIME 校时；发送也通过回调交还给 MavlinkService，避免第二个链路所有者。
 */

#include "mavlink/MavlinkBridge.h"
#include "timesync/Timesync.hpp"

#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkTimesync {
public:
    /** 发送回调：由链路所有者完成帧封装与写出。 */
    using SendFn = void (*)(void *ctx, mavlink_message_t &msg);

    MavlinkTimesync(SendFn send, void *send_ctx) noexcept;

    /**
     * Handle an incoming TIMESYNC message (PX4 semantics).
     */
    void handle_message(const mavlink_message_t *msg) noexcept;

    /**
     * 将远端微秒时间换算到本机 HRT；滤波未收敛时退回单调启动时间语义。
     */
    std::uint64_t sync_stamp(std::uint64_t usec) noexcept;

    bool converged() const noexcept;

    void reset() noexcept;

private:
    dima::lib::timesync::Timesync timesync_{};
    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
};

}  // namespace dima::modules::mavlink
