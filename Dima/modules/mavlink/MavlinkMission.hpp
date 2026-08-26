#pragma once
/*
 * MAVLink 空任务应答 — 对齐 PX4 v1.17.0 空任务时的 mission 协议行为
 * （src/modules/mavlink/mavlink_mission.cpp 空任务语义）。
 *
 * 本机不存储任务项（空任务语义）：
 *   - MISSION_REQUEST_LIST：target 匹配后回 MISSION_COUNT{count=0}，
 *     GCS 得知任务数为 0 即结束读任务流程；
 *   - MISSION_CLEAR_ALL：清空等于成功，回 MISSION_ACK{
 *     type=MAV_MISSION_ACCEPTED}；
 *   - mission_type 一律回显请求的事务类型（mission/fence/rally），
 *     事务类型不一致在 GCS 侧按协议错误处理；
 *   - GCS 侧发来的 MISSION_ACK（type 无错误码时）忽略——上传流程
 *     不会发生，收到的只可能是结束回执。
 *
 * 不实现 MISSION_REQUEST(40) 与任务项存储：方言未裁剪该消息，
 * 空任务下 GCS 也不会请求任务项。
 *
 * 应答 target 一律填 GCS 的 sysid/compid（定向回请求方）。
 */

#include "mavlink/MavlinkBridge.h"

#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkMission {
public:
    /** 发送回调：组帧并写出 mavlink_message_t。 */
    using SendFn = void (*)(void *ctx, mavlink_message_t &msg);

    MavlinkMission(SendFn send, void *send_ctx) noexcept;

    void handle_message(const mavlink_message_t *msg) noexcept;

private:
    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
};

}  // namespace dima::modules::mavlink
