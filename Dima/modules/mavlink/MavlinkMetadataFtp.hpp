#pragma once
/*
 * 参数 Component Metadata 专用的只读 MAVLink FTP 服务。
 *
 * 协议常量与响应语义对齐 PX4 v1.17.0。QGC 下载流程为 OpenFileRO -> 单包
 * BurstReadFile -> 可选 ReadFile 补洞 -> ResetSessions。服务只暴露编译期虚拟文件，
 * 不访问文件系统、不支持写入/目录操作，也不使用堆内存。
 */

#include "mavlink/MavlinkBridge.h"

#include <cstddef>
#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkMetadataFtp {
public:
    struct __attribute__((__packed__)) PayloadHeader {
        std::uint16_t seq_number;
        std::uint8_t session;
        std::uint8_t opcode;
        std::uint8_t size;
        std::uint8_t req_opcode;
        std::uint8_t burst_complete;
        std::uint8_t padding;
        std::uint32_t offset;
    };

    static_assert(sizeof(PayloadHeader) == 12U,
                  "MAVLink FTP header must match PX4/QGC");

    enum Opcode : std::uint8_t {
        kCmdNone,
        kCmdTerminateSession,
        kCmdResetSessions,
        kCmdListDirectory,
        kCmdOpenFileRO,
        kCmdReadFile,
        kCmdCreateFile,
        kCmdWriteFile,
        kCmdRemoveFile,
        kCmdCreateDirectory,
        kCmdRemoveDirectory,
        kCmdOpenFileWO,
        kCmdTruncateFile,
        kCmdRename,
        kCmdCalcFileCRC32,
        kCmdBurstReadFile,
        kRspAck = 128,
        kRspNak,
    };

    enum ErrorCode : std::uint8_t {
        kErrNone,
        kErrFail,
        kErrFailErrno,
        kErrInvalidDataSize,
        kErrInvalidSession,
        kErrNoSessionsAvailable,
        kErrEOF,
        kErrUnknownCommand,
        kErrFailFileExists,
        kErrFailFileProtected,
        kErrFileNotFound,
    };

    static constexpr std::uint8_t kMaxDataLength =
        MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN -
        sizeof(PayloadHeader);
    static_assert(kMaxDataLength == 239U,
                  "MAVLink FTP data payload must remain 239 bytes");

    struct __attribute__((__packed__)) Payload {
        PayloadHeader header;
        std::uint8_t data[kMaxDataLength];
    };

    static_assert(
        sizeof(Payload) ==
            MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN,
        "MAVLink FTP request payload must remain 251 bytes");

    struct VirtualFile {
        const char *path;
        const std::uint8_t *data;
        std::uint32_t size;
    };

    using SendFn = bool (*)(void *context, mavlink_message_t &message);

    MavlinkMetadataFtp(SendFn send, void *send_context) noexcept;

    void init(const VirtualFile *files, std::uint8_t count) noexcept;

    void reset() noexcept;

    void handle_message(const mavlink_message_t *message,
                        std::uint64_t now_us) noexcept;

    bool service(std::uint64_t now_us) noexcept;

private:
    bool flush_pending() noexcept;
    static constexpr std::uint8_t kMaxFiles = 4U;
    static constexpr std::uint8_t kSessionId = 0U;
    static constexpr std::uint8_t kMaxTxRetries = 4U;
    static constexpr std::uint64_t kSessionTimeoutUs = 10000000ULL;

    struct RequestKey {
        // 请求身份同时包含来源、序号、操作、偏移和内容指纹，用于识别链路重传。
        std::uint8_t source_system{0U};
        std::uint8_t source_component{0U};
        std::uint16_t sequence{0U};
        std::uint8_t session{0U};
        std::uint8_t opcode{0U};
        std::uint8_t size{0U};
        std::uint32_t offset{0U};
        std::uint64_t fingerprint{0U};
    };

    struct ReplySlot {
        // 固定单槽缓存保证同一请求重放完全相同的 ACK/NAK，并限制发送重试次数。
        mavlink_file_transfer_protocol_t response{};
        RequestKey key{};
        bool valid{false};
        bool pending{false};
        std::uint8_t retry_count{0U};
    };

    static std::uint64_t request_fingerprint(
        const std::uint8_t *data, std::size_t size) noexcept;

    static bool same_request(const RequestKey &left,
                             const RequestKey &right) noexcept;

    bool session_owned_by(const RequestKey &key) const noexcept;

    void touch_session(const RequestKey &key, std::uint64_t now_us) noexcept;

    void expire_session(std::uint64_t now_us) noexcept;

    void process_request(const Payload &request,
                         const RequestKey &key,
                         std::uint64_t now_us) noexcept;

    ErrorCode open_file(const Payload &request, Payload &reply,
                        const RequestKey &key,
                        std::uint64_t now_us) noexcept;

    ErrorCode read_file(const Payload &request, Payload &reply,
                        const RequestKey &key, bool burst,
                        std::uint64_t now_us) noexcept;

    ErrorCode terminate_session(const RequestKey &key) noexcept;

    ErrorCode reset_sessions() noexcept;

    void reset_session() noexcept;

    SendFn send_{nullptr};
    void *send_context_{nullptr};
    const VirtualFile *files_{nullptr};
    std::uint8_t file_count_{0U};
    // 单会话由首次 OpenFileRO 的 sysid/compid 独占，超时后才允许其他客户端接管。
    bool session_open_{false};
    const std::uint8_t *session_file_{nullptr};
    std::uint32_t session_size_{0U};
    std::uint8_t session_owner_system_{0U};
    std::uint8_t session_owner_component_{0U};
    std::uint64_t session_last_activity_us_{0U};
    ReplySlot reply_{};
};

} // namespace dima::modules::mavlink
