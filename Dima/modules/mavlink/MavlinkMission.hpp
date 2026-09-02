#pragma once

#include "mavlink/MavlinkBridge.h"
#include "mission/MissionService.hpp"

#include <cstdint>

namespace dima::modules::mavlink {

/**
 * QGC Mission 协议状态机。
 *
 * 本类只做 MAVLink 事务、字段校验和定向应答；PX4 Dataman 的 RAM/Flash bank
 * 所有权全部在 MissionService。每个 item 写入 inactive bank 成功后才请求下一项，
 * 最后一项之后只有 Mission State 提交成功才发送最终 MISSION_ACK accepted。
 * QGC 在 MISSION_ITEM_INT 中可能继续发送 GLOBAL frame，本边界按 int 消息的
 * 1e-7 deg 单位接收，并在进入 MissionService 前规范化为对应 _INT frame。
 */
class MavlinkMission {
public:
    using SendFn = bool (*)(void *ctx, mavlink_message_t &msg) noexcept;

    MavlinkMission(dima::modules::mission::MissionService &service,
                   SendFn send, void *send_ctx) noexcept;

    void handle_message(const mavlink_message_t *msg) noexcept;
    void update(std::uint64_t now, bool link_ready) noexcept;
    void reset() noexcept;
    void reset_link() noexcept;

private:
    enum class UploadState : std::uint8_t {
        Idle,
        Receiving,
        WaitingItemWrite,
        WaitingCommit,
        WaitingSetCurrent,
        FinalAckPending,
    };

    static constexpr std::uint64_t kRequestRetryIntervalUs = 500000ULL;
    static constexpr std::uint64_t kUploadTimeoutUs = 10000000ULL;
    static constexpr std::uint64_t kCurrentIntervalUs = 1000000ULL;

    static bool target_matches(std::uint8_t target_system,
                               std::uint8_t target_component) noexcept;
    static std::uint8_t map_service_error(int error) noexcept;
    static std::uint8_t validate_upload_item(
        const mavlink_mission_item_int_t &item,
        std::uint16_t expected_sequence,
        dima::modules::mission::MissionItem &normalized) noexcept;

    bool send_message(mavlink_message_t &message) noexcept;
    bool send_ack(std::uint8_t target_system,
                  std::uint8_t target_component,
                  std::uint8_t result,
                  std::uint32_t mission_id) noexcept;
    bool send_request(std::uint16_t sequence) noexcept;
    bool send_count(std::uint8_t target_system,
                    std::uint8_t target_component) noexcept;
    int send_item(std::uint8_t target_system,
                  std::uint8_t target_component,
                  std::uint16_t sequence) noexcept;
    bool send_current() noexcept;
    bool send_reached(std::uint16_t sequence) noexcept;
    void observe_execution_progress(
        const dima::modules::mission::MissionStatus &status) noexcept;
    bool flush_reached() noexcept;
    void queue_final_ack(std::uint8_t result,
                         std::uint32_t mission_id) noexcept;
    void reset_upload(bool abort_receiving) noexcept;

    void handle_request_list(const mavlink_message_t &msg) noexcept;
    void handle_count(const mavlink_message_t &msg) noexcept;
    void handle_item_int(const mavlink_message_t &msg) noexcept;
    void handle_request_int(const mavlink_message_t &msg) noexcept;
    void handle_clear_all(const mavlink_message_t &msg) noexcept;
    void handle_set_current(const mavlink_message_t &msg) noexcept;

    dima::modules::mission::MissionService &service_;
    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
    UploadState upload_state_{UploadState::Idle};
    std::uint32_t upload_token_{0U};
    std::uint16_t upload_count_{0U};
    std::uint16_t next_sequence_{0U};
    std::uint8_t upload_system_{0U};
    std::uint8_t upload_component_{0U};
    // 上传超时按“最近一次合法事务活动”计算，而不是从 MISSION_COUNT 起限制
    // 整个任务必须在 10 s 内完成；否则 64 项任务会在持续推进时被误中止。
    std::uint64_t last_upload_activity_us_{0U};
    std::uint64_t last_request_us_{0U};
    mavlink_mission_ack_t pending_final_ack_{};
    std::uint64_t last_current_us_{0U};
    std::uint32_t last_current_mission_id_{0U};
    std::uint16_t last_current_sequence_{UINT16_MAX};
    std::uint16_t last_current_count_{UINT16_MAX};
    std::uint8_t last_current_execution_state_{UINT8_MAX};
    std::uint32_t observed_mission_id_{0U};
    std::uint16_t observed_current_{0U};
    dima::modules::mission::MissionExecutionState observed_execution_state_{
        dima::modules::mission::MissionExecutionState::NoMission};
    std::uint64_t pending_reached_mask_{0U};
    std::uint64_t sent_reached_mask_{0U};
    bool execution_observed_{false};
};

} // namespace dima::modules::mavlink
