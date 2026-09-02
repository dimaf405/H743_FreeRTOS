#include "MavlinkMission.hpp"

#include "api/Time.hpp"

#include <cerrno>
#include <cfloat>
#include <cmath>

namespace dima::modules::mavlink {
namespace {

bool zero_or_unspecified(float value) noexcept
{
    return std::isnan(value) ||
           (std::isfinite(value) && std::fabs(value) <= FLT_EPSILON);
}

bool normalize_global_frame(std::uint8_t received,
                            std::uint8_t &canonical) noexcept
{
    // QGC 5.1.3 使用 MISSION_ITEM_INT 传输时仍保留编辑模型中的
    // GLOBAL/GLOBAL_RELATIVE_ALT frame，但 x/y 已经固定乘以 1e7。
    // 因此坐标单位由消息类型决定，frame 只区分绝对/相对高度；进入固定任务
    // 仓库前统一收敛为 _INT，避免持久格式和导航消费者维护四种等价表示。
    switch (received) {
    case MAV_FRAME_GLOBAL:
    case MAV_FRAME_GLOBAL_INT:
        canonical = MAV_FRAME_GLOBAL_INT;
        return true;
    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        canonical = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        return true;
    default:
        canonical = 0U;
        return false;
    }
}

} // namespace

MavlinkMission::MavlinkMission(
    dima::modules::mission::MissionService &service,
    SendFn send, void *send_ctx) noexcept
    : service_(service), send_(send), send_ctx_(send_ctx)
{
}

bool MavlinkMission::target_matches(
    std::uint8_t target_system,
    std::uint8_t target_component) noexcept
{
    return target_system == MAVLINK_SYSTEM_ID &&
           (target_component == MAVLINK_COMPONENT_ID ||
            target_component == 0U);
}

std::uint8_t MavlinkMission::map_service_error(int error) noexcept
{
    switch (error) {
    case 0:         return MAV_MISSION_ACCEPTED;
    case -ENOSPC:   return MAV_MISSION_NO_SPACE;
    case -EPERM:
    case -ENODEV:
    case -EBUSY:
    case -EAGAIN:   return MAV_MISSION_DENIED;
    case -ERANGE:
    case -EINVAL:   return MAV_MISSION_INVALID;
    default:        return MAV_MISSION_ERROR;
    }
}

std::uint8_t MavlinkMission::validate_upload_item(
    const mavlink_mission_item_int_t &item,
    std::uint16_t expected_sequence,
    dima::modules::mission::MissionItem &normalized) noexcept
{
    normalized = {};
    if (item.seq != expected_sequence) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }
    if (item.mission_type != MAV_MISSION_TYPE_MISSION ||
        item.command != MAV_CMD_NAV_WAYPOINT) {
        return MAV_MISSION_UNSUPPORTED;
    }
    std::uint8_t canonical_frame{};
    if (!normalize_global_frame(item.frame, canonical_frame)) {
        return MAV_MISSION_UNSUPPORTED_FRAME;
    }

    // dwell、pass radius、指定 yaw 和暂停式 autocontinue 均不在首版范围。
    // NaN 视作“未指定”，任何非零有限值都明确返回 unsupported，不静默丢弃。
    if (!zero_or_unspecified(item.param1) ||
        !zero_or_unspecified(item.param3) || !std::isnan(item.param4) ||
        item.autocontinue != 1U || item.current > 1U) {
        return MAV_MISSION_UNSUPPORTED;
    }
    if ((!std::isfinite(item.param2) && !std::isnan(item.param2)) ||
        (std::isfinite(item.param2) && item.param2 < 0.0F)) {
        return MAV_MISSION_INVALID_PARAM2;
    }
    if (item.x < -900000000 || item.x > 900000000) {
        return MAV_MISSION_INVALID_PARAM5_X;
    }
    if (item.y < -1800000000 || item.y > 1800000000) {
        return MAV_MISSION_INVALID_PARAM6_Y;
    }
    if (!std::isfinite(item.z)) {
        return MAV_MISSION_INVALID_PARAM7;
    }

    normalized.sequence = item.seq;
    normalized.latitude_e7 = item.x;
    normalized.longitude_e7 = item.y;
    normalized.altitude_m = item.z;
    normalized.acceptance_radius_m = std::isfinite(item.param2)
                                         ? item.param2
                                         : 0.0F;
    normalized.frame = canonical_frame;
    return MAV_MISSION_ACCEPTED;
}

bool MavlinkMission::send_message(mavlink_message_t &message) noexcept
{
    return send_ != nullptr && send_(send_ctx_, message);
}

bool MavlinkMission::send_ack(std::uint8_t target_system,
                              std::uint8_t target_component,
                              std::uint8_t result,
                              std::uint32_t mission_id) noexcept
{
    mavlink_mission_ack_t ack{};
    ack.target_system = target_system;
    ack.target_component = target_component;
    ack.type = result;
    ack.mission_type = MAV_MISSION_TYPE_MISSION;
    ack.opaque_id = mission_id;
    mavlink_message_t message{};
    mavlink_msg_mission_ack_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &ack);
    return send_message(message);
}

bool MavlinkMission::send_request(std::uint16_t sequence) noexcept
{
    mavlink_mission_request_int_t request{};
    request.seq = sequence;
    request.target_system = upload_system_;
    request.target_component = upload_component_;
    request.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_message_t message{};
    mavlink_msg_mission_request_int_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &request);
    return send_message(message);
}

bool MavlinkMission::send_count(std::uint8_t target_system,
                                std::uint8_t target_component) noexcept
{
    dima::modules::mission::MissionStatus status{};
    if (service_.status(status) != 0) {
        return false;
    }

    const bool recovery_pending = !status.loaded;
    if (recovery_pending &&
        (status.committed || status.count != 0U || status.mission_id != 0U)) {
        // loaded=false 只允许出现在启动持久化恢复尚未完成且 active RAM 为空时。
        // 若快照同时声称已有任务，说明服务状态不一致；此时不得把未知任务伪报为
        // 空任务，也不得泄露未完成恢复中的内容，保留 QGC 重试并继续 fail-closed。
        return false;
    }

    mavlink_mission_count_t count{};
    // MISSION_REQUEST_LIST 的唯一正常应答是 MISSION_COUNT；QGC 不会把 busy ACK
    // 当作列表长度。启动 Dataman 恢复尚未完成时，active RAM 按服务不变量
    // 必为空，因此回 count=0/opaque_id=0 只描述“当前无可回读任务”。上传、清空
    // 和 AUTO 仍分别受 loaded/storage_available/committed 门控，不会由该回读应答
    // 获得写入或执行权限；恢复完成后返回 Mission State 指向的 active bank 快照。
    count.count = recovery_pending ? 0U : status.count;
    count.target_system = target_system;
    count.target_component = target_component;
    count.mission_type = MAV_MISSION_TYPE_MISSION;
    count.opaque_id = recovery_pending ? 0U : status.mission_id;
    mavlink_message_t message{};
    mavlink_msg_mission_count_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &count);
    return send_message(message);
}

int MavlinkMission::send_item(std::uint8_t target_system,
                              std::uint8_t target_component,
                              std::uint16_t sequence) noexcept
{
    dima::modules::mission::MissionItem source{};
    dima::modules::mission::MissionStatus status{};
    const int loaded = service_.item(sequence, source, status);
    if (loaded != 0) {
        return loaded;
    }
    if (!status.loaded) {
        return -EBUSY;
    }
    mavlink_mission_item_int_t item{};
    item.param1 = 0.0F;
    item.param2 = source.acceptance_radius_m;
    item.param3 = 0.0F;
    item.param4 = NAN;
    item.x = source.latitude_e7;
    item.y = source.longitude_e7;
    item.z = source.altitude_m;
    item.seq = sequence;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.target_system = target_system;
    item.target_component = target_component;
    item.frame = source.frame;
    item.current = sequence == status.current ? 1U : 0U;
    item.autocontinue = 1U;
    item.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_message_t message{};
    mavlink_msg_mission_item_int_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &item);
    return send_message(message) ? 0 : -EIO;
}

bool MavlinkMission::send_current() noexcept
{
    dima::modules::mission::MissionStatus status{};
    if (service_.status(status) != 0 || !status.loaded) {
        return false;
    }
    mavlink_mission_current_t current{};
    current.seq = status.count == 0U ? 0U : status.current;
    current.total = status.count == 0U ? UINT16_MAX : status.count;
    switch (status.execution_state) {
    case dima::modules::mission::MissionExecutionState::NoMission:
        current.mission_state = MISSION_STATE_NO_MISSION;
        current.mission_mode = 2U;
        break;
    case dima::modules::mission::MissionExecutionState::NotStarted:
        current.mission_state = MISSION_STATE_NOT_STARTED;
        current.mission_mode = 2U;
        break;
    case dima::modules::mission::MissionExecutionState::Active:
        current.mission_state = MISSION_STATE_ACTIVE;
        current.mission_mode = 1U;
        break;
    case dima::modules::mission::MissionExecutionState::Complete:
        current.mission_state = MISSION_STATE_COMPLETE;
        current.mission_mode = 2U;
        break;
    }
    current.mission_id = status.mission_id;
    current.fence_id = 0U;
    current.rally_points_id = 0U;
    mavlink_message_t message{};
    mavlink_msg_mission_current_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &current);
    if (!send_message(message)) {
        return false;
    }
    last_current_mission_id_ = status.mission_id;
    last_current_sequence_ = current.seq;
    last_current_count_ = status.count;
    last_current_execution_state_ =
        static_cast<std::uint8_t>(status.execution_state);
    return true;
}

bool MavlinkMission::send_reached(std::uint16_t sequence) noexcept
{
    mavlink_mission_item_reached_t reached{};
    reached.seq = sequence;
    mavlink_message_t message{};
    mavlink_msg_mission_item_reached_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &reached);
    return send_message(message);
}

void MavlinkMission::observe_execution_progress(
    const dima::modules::mission::MissionStatus &status) noexcept
{
    if (!status.loaded || !status.committed || status.mission_id == 0U ||
        status.count == 0U ||
        status.count > dima::modules::mission::kMissionCapacity ||
        status.current >= status.count) {
        execution_observed_ = false;
        observed_mission_id_ = 0U;
        observed_current_ = 0U;
        observed_execution_state_ =
            dima::modules::mission::MissionExecutionState::NoMission;
        pending_reached_mask_ = 0U;
        sent_reached_mask_ = 0U;
        return;
    }

    if (!execution_observed_ || observed_mission_id_ != status.mission_id) {
        // 新任务代际只建立基线，不把 MISSION_SET_CURRENT 选择的前置项误报为
        // 已到达；到达事件只能来自同一次 Active 执行中的权威状态迁移。
        execution_observed_ = true;
        observed_mission_id_ = status.mission_id;
        observed_current_ = status.current;
        observed_execution_state_ = status.execution_state;
        pending_reached_mask_ = 0U;
        sent_reached_mask_ = 0U;
        return;
    }

    const bool restart_entry =
        (status.execution_state ==
             dima::modules::mission::MissionExecutionState::NotStarted ||
         status.execution_state ==
             dima::modules::mission::MissionExecutionState::Active) &&
        (observed_execution_state_ ==
             dima::modules::mission::MissionExecutionState::Complete ||
         status.current < observed_current_);
    if (restart_entry) {
        // 完成后重新 Start、相同内容重传或 SET_CURRENT 回退都会建立新执行入口，
        // 即使内容哈希 mission_id 未变也允许重新报告到达事件。故障暂停保持相同
        // current，不满足本条件，继续保留 sent mask 避免恢复后重复事件。
        pending_reached_mask_ = 0U;
        sent_reached_mask_ = 0U;
    } else if (observed_execution_state_ ==
                   dima::modules::mission::MissionExecutionState::Active &&
               status.execution_state ==
                   dima::modules::mission::MissionExecutionState::Active &&
               status.current > observed_current_) {
        // 只有权威状态保持 Active 的索引前进才来自 AutoMode::advance_current。
        // Disarm/Manual 会先把执行态改为 NotStarted；若随后 SET_CURRENT 前移且
        // MAVLink 恰好没采到中间帧，观察结果可能直接从 Active/旧索引跳到
        // NotStarted/新索引。该跳变是人工选择执行入口，绝不能把跳过项伪报成
        // MISSION_ITEM_REACHED。
        for (std::uint16_t sequence = observed_current_;
             sequence < status.current &&
             sequence < dima::modules::mission::kMissionCapacity;
             ++sequence) {
            pending_reached_mask_ |= UINT64_C(1) << sequence;
        }
    }

    if (!restart_entry &&
        status.execution_state ==
            dima::modules::mission::MissionExecutionState::Complete &&
        observed_execution_state_ !=
            dima::modules::mission::MissionExecutionState::Complete) {
        // 最终航点不再推进 current；只有 Active->Complete 转移能证明“已进入
        // acceptance 且实测速度低于 RO_SPEED_TH”，因此在这里排队最终事件。
        pending_reached_mask_ |= UINT64_C(1) << status.current;
    }

    observed_current_ = status.current;
    observed_execution_state_ = status.execution_state;
}

bool MavlinkMission::flush_reached() noexcept
{
    for (std::uint16_t sequence = 0U;
         sequence < dima::modules::mission::kMissionCapacity;
         ++sequence) {
        const std::uint64_t bit = UINT64_C(1) << sequence;
        if ((pending_reached_mask_ & bit) == 0U ||
            (sent_reached_mask_ & bit) != 0U) {
            continue;
        }
        if (!send_reached(sequence)) {
            return false;
        }
        sent_reached_mask_ |= bit;
        pending_reached_mask_ &= ~bit;
    }
    return true;
}

void MavlinkMission::queue_final_ack(std::uint8_t result,
                                     std::uint32_t mission_id) noexcept
{
    pending_final_ack_ = {};
    pending_final_ack_.target_system = upload_system_;
    pending_final_ack_.target_component = upload_component_;
    pending_final_ack_.type = result;
    pending_final_ack_.mission_type = MAV_MISSION_TYPE_MISSION;
    pending_final_ack_.opaque_id = mission_id;
    upload_state_ = UploadState::FinalAckPending;
}

void MavlinkMission::reset_upload(bool abort_receiving) noexcept
{
    if (abort_receiving &&
        (upload_state_ == UploadState::Receiving ||
         upload_state_ == UploadState::WaitingItemWrite) &&
        upload_token_ != 0U) {
        service_.abort_upload(upload_token_);
    }
    upload_state_ = UploadState::Idle;
    upload_token_ = 0U;
    upload_count_ = 0U;
    next_sequence_ = 0U;
    upload_system_ = 0U;
    upload_component_ = 0U;
    last_upload_activity_us_ = 0U;
    last_request_us_ = 0U;
    pending_final_ack_ = {};
}

void MavlinkMission::reset() noexcept
{
    reset_upload(true);
    last_current_us_ = 0U;
    last_current_mission_id_ = 0U;
    last_current_sequence_ = UINT16_MAX;
    last_current_count_ = UINT16_MAX;
    last_current_execution_state_ = UINT8_MAX;
    observed_mission_id_ = 0U;
    observed_current_ = 0U;
    observed_execution_state_ =
        dima::modules::mission::MissionExecutionState::NoMission;
    pending_reached_mask_ = 0U;
    sent_reached_mask_ = 0U;
    execution_observed_ = false;
}

void MavlinkMission::reset_link() noexcept
{
    // 未收齐的上传不能跨 GCS 会话延续；已交给 storage 的提交仍继续，结果保留到
    // 链路恢复后发送最终 ACK，避免 MissionService 的完成槽无人消费。
    if (upload_state_ == UploadState::Receiving ||
        upload_state_ == UploadState::WaitingItemWrite) {
        reset_upload(true);
    }
    last_current_us_ = 0U;
}

void MavlinkMission::handle_request_list(
    const mavlink_message_t &msg) noexcept
{
    mavlink_mission_request_list_t request{};
    mavlink_msg_mission_request_list_decode(&msg, &request);
    if (!target_matches(request.target_system, request.target_component)) {
        return;
    }
    if (request.mission_type != MAV_MISSION_TYPE_MISSION) {
        (void)send_ack(msg.sysid, msg.compid,
                       MAV_MISSION_UNSUPPORTED, 0U);
        return;
    }
    (void)send_count(msg.sysid, msg.compid);
}

void MavlinkMission::handle_count(const mavlink_message_t &msg) noexcept
{
    mavlink_mission_count_t count{};
    mavlink_msg_mission_count_decode(&msg, &count);
    if (!target_matches(count.target_system, count.target_component)) {
        return;
    }
    if (count.mission_type != MAV_MISSION_TYPE_MISSION) {
        (void)send_ack(msg.sysid, msg.compid,
                       MAV_MISSION_UNSUPPORTED, 0U);
        return;
    }

    if ((upload_state_ == UploadState::Receiving ||
         upload_state_ == UploadState::WaitingItemWrite ||
         upload_state_ == UploadState::WaitingCommit ||
         upload_state_ == UploadState::FinalAckPending) &&
        upload_system_ == msg.sysid && upload_component_ == msg.compid &&
        upload_count_ == count.count) {
        // QGC 重发同一 COUNT 时不重建 staging。若当前 item 仍在写 Dataman，
        // 必须等待 dm_write 完成；Receiving 才重发当前期望序号。
        last_upload_activity_us_ = hrt_absolute_time();
        if (upload_state_ == UploadState::Receiving &&
            send_request(next_sequence_)) {
            last_request_us_ = last_upload_activity_us_;
        }
        return;
    }
    if (upload_state_ != UploadState::Idle) {
        (void)send_ack(msg.sysid, msg.compid, MAV_MISSION_DENIED, 0U);
        return;
    }

    std::uint32_t token{};
    const int result = service_.begin_upload(count.count, token);
    if (result != 0) {
        (void)send_ack(msg.sysid, msg.compid,
                       map_service_error(result), 0U);
        return;
    }
    upload_token_ = token;
    upload_count_ = count.count;
    next_sequence_ = 0U;
    upload_system_ = msg.sysid;
    upload_component_ = msg.compid;
    last_upload_activity_us_ = hrt_absolute_time();
    last_request_us_ = 0U;
    upload_state_ = count.count == 0U
                        ? UploadState::WaitingCommit
                        : UploadState::Receiving;
    if (upload_state_ == UploadState::Receiving &&
        send_request(next_sequence_)) {
        last_request_us_ = last_upload_activity_us_;
    }
}

void MavlinkMission::handle_item_int(
    const mavlink_message_t &msg) noexcept
{
    mavlink_mission_item_int_t item{};
    mavlink_msg_mission_item_int_decode(&msg, &item);
    if (!target_matches(item.target_system, item.target_component)) {
        return;
    }
    const bool same_upload_partner =
        msg.sysid == upload_system_ && msg.compid == upload_component_;
    if (upload_state_ == UploadState::WaitingItemWrite &&
        same_upload_partner) {
        // 单线程 PX4 Mission Manager 在同步 dm_write 期间不会处理重复 item；
        // H743 异步 Flash 期间等价地忽略伙伴重发，不提前请求或重复写 index。
        last_upload_activity_us_ = hrt_absolute_time();
        return;
    }
    if ((upload_state_ == UploadState::WaitingCommit ||
         upload_state_ == UploadState::FinalAckPending) &&
        same_upload_partner && next_sequence_ > 0U &&
        item.seq + 1U == next_sequence_) {
        // 最后一项已经写入 inactive bank，当前只等待 Mission State 或最终 ACK。
        // PX4 会把该帧视作“最终 ACK 丢失后的重复项”，绝不能回复 DENIED。
        last_upload_activity_us_ = hrt_absolute_time();
        return;
    }
    if (upload_state_ != UploadState::Receiving ||
        msg.sysid != upload_system_ || msg.compid != upload_component_) {
        (void)send_ack(msg.sysid, msg.compid, MAV_MISSION_DENIED, 0U);
        return;
    }
    // 与 PX4 Mission 协议一致，超时表示上传端已经无活动，而不是总传输
    // 时长上限。合法伙伴发来的重复/乱序 ITEM 仍可通过重发 REQUEST_INT
    // 恢复，因此在完成伙伴校验后统一刷新活动时间；非法字段仍在下方立即终止。
    last_upload_activity_us_ = hrt_absolute_time();
    if (item.seq + 1U == next_sequence_) {
        // 上一个 ITEM 的 REQUEST_INT/响应交错时，幂等重发当前期望序号。
        if (send_request(next_sequence_)) {
            last_request_us_ = last_upload_activity_us_;
        }
        return;
    }
    if (item.seq != next_sequence_) {
        if (send_request(next_sequence_)) {
            last_request_us_ = last_upload_activity_us_;
        }
        return;
    }

    dima::modules::mission::MissionItem normalized{};
    const std::uint8_t validation =
        validate_upload_item(item, next_sequence_, normalized);
    if (validation != MAV_MISSION_ACCEPTED) {
        service_.abort_upload(upload_token_);
        (void)send_ack(upload_system_, upload_component_, validation, 0U);
        reset_upload(false);
        return;
    }

    const int staged = service_.stage_item(upload_token_, normalized);
    if (staged == -EAGAIN) {
        // MissionService 的 no-wait RAM mutex 短暂忙不代表 ITEM 非法，也不能
        // 销毁 staging；保持 next_sequence 不变并重发 REQUEST_INT 即可幂等重试。
        if (send_request(next_sequence_)) {
            last_request_us_ = last_upload_activity_us_;
        }
        return;
    }
    if (staged != 0) {
        service_.abort_upload(upload_token_);
        (void)send_ack(upload_system_, upload_component_,
                       map_service_error(staged), 0U);
        reset_upload(false);
        return;
    }
    // 与 PX4 的 writeSync 顺序相同：当前 index 的 Dataman 写完成前不推进
    // next_sequence，也不发送下一条 MISSION_REQUEST_INT。
    upload_state_ = UploadState::WaitingItemWrite;
}

void MavlinkMission::handle_request_int(
    const mavlink_message_t &msg) noexcept
{
    mavlink_mission_request_int_t request{};
    mavlink_msg_mission_request_int_decode(&msg, &request);
    if (!target_matches(request.target_system, request.target_component)) {
        return;
    }
    if (request.mission_type != MAV_MISSION_TYPE_MISSION) {
        (void)send_ack(msg.sysid, msg.compid,
                       MAV_MISSION_UNSUPPORTED, 0U);
        return;
    }
    const int result = send_item(msg.sysid, msg.compid, request.seq);
    if (result == -EAGAIN || result == -EIO) {
        // no-wait RAM mutex 或链路发送暂忙时不伪造 INVALID_SEQUENCE；QGC 会按
        // Mission 下载事务重发同一 REQUEST_INT，下一次再取权威 active 快照。
        return;
    }
    if (result != 0) {
        (void)send_ack(msg.sysid, msg.compid,
                       result == -ERANGE ? MAV_MISSION_INVALID_SEQUENCE
                                         : map_service_error(result),
                       0U);
    }
}

void MavlinkMission::handle_clear_all(
    const mavlink_message_t &msg) noexcept
{
    mavlink_mission_clear_all_t clear{};
    mavlink_msg_mission_clear_all_decode(&msg, &clear);
    if (!target_matches(clear.target_system, clear.target_component)) {
        return;
    }
    if (clear.mission_type != MAV_MISSION_TYPE_MISSION) {
        (void)send_ack(msg.sysid, msg.compid,
                       MAV_MISSION_UNSUPPORTED, 0U);
        return;
    }
    if (upload_state_ != UploadState::Idle) {
        (void)send_ack(msg.sysid, msg.compid, MAV_MISSION_DENIED, 0U);
        return;
    }
    std::uint32_t token{};
    const int requested = service_.request_clear(token);
    if (requested != 0) {
        (void)send_ack(msg.sysid, msg.compid,
                       map_service_error(requested), 0U);
        return;
    }
    upload_token_ = token;
    upload_system_ = msg.sysid;
    upload_component_ = msg.compid;
    last_upload_activity_us_ = hrt_absolute_time();
    upload_state_ = UploadState::WaitingCommit;
}

void MavlinkMission::handle_set_current(
    const mavlink_message_t &msg) noexcept
{
    mavlink_mission_set_current_t set_current{};
    mavlink_msg_mission_set_current_decode(&msg, &set_current);
    if (!target_matches(set_current.target_system,
                        set_current.target_component)) {
        return;
    }
    if (upload_state_ != UploadState::Idle) {
        // WaitingCommit/FinalAckPending 仍属于同一上传事务；在最终 ACK 真正发出前
        // 插入 SET_CURRENT 会让 QGC 收到的 mission_id 与执行入口跨代。所有任务
        // 内容及入口修改必须串行化，回读请求仍可并行服务旧 active 快照。
        (void)send_ack(msg.sysid, msg.compid, MAV_MISSION_DENIED, 0U);
        return;
    }
    std::uint32_t token{};
    const int result = service_.set_current(set_current.seq, token);
    if (result != 0) {
        (void)send_ack(msg.sysid, msg.compid,
                       map_service_error(result), 0U);
        return;
    }
    upload_token_ = token;
    upload_system_ = msg.sysid;
    upload_component_ = msg.compid;
    last_upload_activity_us_ = hrt_absolute_time();
    upload_state_ = UploadState::WaitingSetCurrent;
}

void MavlinkMission::handle_message(
    const mavlink_message_t *msg) noexcept
{
    if (msg == nullptr) {
        return;
    }
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        handle_request_list(*msg);
        break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
        handle_count(*msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        handle_item_int(*msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        handle_request_int(*msg);
        break;
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        handle_clear_all(*msg);
        break;
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        handle_set_current(*msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ACK:
        // GCS 下载结束回执无须回复；上传最终 ACK 始终由本机发送。
        break;
    default:
        break;
    }
}

void MavlinkMission::update(std::uint64_t now, bool link_ready) noexcept
{
    if (upload_state_ == UploadState::WaitingItemWrite) {
        dima::modules::mission::MissionStageResult result{};
        const int polled = service_.poll_stage_result(
            upload_token_, result);
        if (polled == 0) {
            if (result.error != 0 || result.sequence != next_sequence_) {
                service_.abort_upload(upload_token_);
                queue_final_ack(
                    result.error != 0 ? map_service_error(result.error)
                                      : MAV_MISSION_ERROR,
                    0U);
            } else {
                ++next_sequence_;
                if (result.complete) {
                    upload_state_ = UploadState::WaitingCommit;
                } else {
                    upload_state_ = UploadState::Receiving;
                    if (link_ready && send_request(next_sequence_)) {
                        last_request_us_ = now;
                    }
                }
            }
        }
    }

    if (upload_state_ == UploadState::Receiving ||
        upload_state_ == UploadState::WaitingItemWrite) {
        // 10 s 是协议无活动超时。每个来自既定上传端、可参与恢复的合法
        // COUNT/ITEM 都会刷新时间，因此慢速大任务只要持续交换就不会误终止。
        if (now >= last_upload_activity_us_ &&
            now - last_upload_activity_us_ >= kUploadTimeoutUs) {
            service_.abort_upload(upload_token_);
            queue_final_ack(MAV_MISSION_ERROR, 0U);
        } else if (upload_state_ == UploadState::Receiving && link_ready &&
                   (last_request_us_ == 0U ||
                    (now >= last_request_us_ &&
                     now - last_request_us_ >= kRequestRetryIntervalUs)) &&
                   send_request(next_sequence_)) {
            last_request_us_ = now;
        }
    }

    if (upload_state_ == UploadState::WaitingCommit) {
        dima::modules::mission::MissionCommitResult result{};
        const int polled = service_.poll_commit_result(
            upload_token_, result);
        if (polled == 0) {
            queue_final_ack(map_service_error(result.error),
                            result.mission_id);
        }
    }

    if (upload_state_ == UploadState::WaitingSetCurrent) {
        dima::modules::mission::MissionCommitResult result{};
        const int polled = service_.poll_commit_result(
            upload_token_, result);
        if (polled == 0) {
            if (result.error != 0) {
                queue_final_ack(map_service_error(result.error), 0U);
            } else {
                // PX4 对 MISSION_SET_CURRENT 的成功响应是 MISSION_CURRENT；
                // Mission State 已提交后再公开新入口，掉电不会回到未确认值。
                reset_upload(false);
                last_current_us_ = 0U;
                if (link_ready && send_current()) {
                    last_current_us_ = now;
                }
            }
        }
    }

    if (upload_state_ == UploadState::FinalAckPending && link_ready) {
        mavlink_message_t message{};
        mavlink_msg_mission_ack_encode(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
            &message, &pending_final_ack_);
        if (send_message(message)) {
            reset_upload(false);
            last_current_us_ = 0U;
        }
        return;
    }

    dima::modules::mission::MissionStatus status{};
    if (service_.status(status) != 0 || !status.loaded) {
        return;
    }
    // 即使 QGC 断链也持续观察权威执行进度，并把未发送到达项留在固定 bitmask；
    // 任务运行不依赖链路，恢复后再按序发送，不动态分配也不重复航点事件。
    observe_execution_progress(status);
    if (!link_ready) {
        return;
    }
    if (!flush_reached()) {
        return;
    }
    const bool changed = status.mission_id != last_current_mission_id_ ||
                          status.current != last_current_sequence_ ||
                          status.count != last_current_count_ ||
                          static_cast<std::uint8_t>(status.execution_state) !=
                              last_current_execution_state_;
    if (changed || last_current_us_ == 0U ||
        (now >= last_current_us_ &&
         now - last_current_us_ >= kCurrentIntervalUs)) {
        if (send_current()) {
            last_current_us_ = now;
        }
    }
}

} // namespace dima::modules::mavlink
