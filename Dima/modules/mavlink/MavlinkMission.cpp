#include "MavlinkMission.hpp"

namespace dima::modules::mavlink {

MavlinkMission::MavlinkMission(SendFn send, void *send_ctx) noexcept
    : send_(send), send_ctx_(send_ctx)
{
}

void MavlinkMission::handle_message(const mavlink_message_t *msg) noexcept
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
        mavlink_mission_request_list_t req;
        mavlink_msg_mission_request_list_decode(msg, &req);

        /* 只应答发给本机的列表请求（compid 0 为广播）。 */
        if (req.target_system != MAVLINK_SYSTEM_ID ||
            (req.target_component != MAVLINK_COMPONENT_ID &&
             req.target_component != 0)) {
            return;
        }

        /* 空任务：数量 0，GCS 由此结束下载流程。mission_type
         * 回显请求的事务类型（QGC 按 mission/fence/rally 三个
         * 独立事务管理，类型不一致按协议错误处理）。 */
        mavlink_mission_count_t count{};
        count.count = 0;
        count.target_system = msg->sysid;
        count.target_component = msg->compid;
        count.mission_type = req.mission_type;

        mavlink_message_t reply{};
        mavlink_msg_mission_count_encode(MAVLINK_SYSTEM_ID,
                                         MAVLINK_COMPONENT_ID,
                                         &reply, &count);
        if (send_ != nullptr) {
            send_(send_ctx_, reply);
        }
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
        mavlink_mission_clear_all_t clear;
        mavlink_msg_mission_clear_all_decode(msg, &clear);

        if (clear.target_system != MAVLINK_SYSTEM_ID ||
            (clear.target_component != MAVLINK_COMPONENT_ID &&
             clear.target_component != 0)) {
            return;
        }

        /* 任务本就为空，清除视为成功；mission_type 回显请求的
         * 事务类型，与请求事务保持一致。 */
        mavlink_mission_ack_t ack{};
        ack.target_system = msg->sysid;
        ack.target_component = msg->compid;
        ack.type = MAV_MISSION_ACCEPTED;
        ack.mission_type = clear.mission_type;

        mavlink_message_t reply{};
        mavlink_msg_mission_ack_encode(MAVLINK_SYSTEM_ID,
                                       MAVLINK_COMPONENT_ID,
                                       &reply, &ack);
        if (send_ != nullptr) {
            send_(send_ctx_, reply);
        }
        break;
    }

    default:
        /* MISSION_ACK 等其余任务消息：空任务语义下忽略。 */
        break;
    }
}

} // namespace dima::modules::mavlink
