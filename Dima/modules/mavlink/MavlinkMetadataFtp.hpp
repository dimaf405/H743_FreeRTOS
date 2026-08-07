#pragma once
/*
 * Read-only MAVLink FTP server for Component Metadata delivery —
 * protocol constants, payload layout and NAK/ACK semantics ported from
 * PX4-Autopilot v1.17.0 src/modules/mavlink/mavlink_ftp.h/.cpp
 * (commit d6f12ad).
 *
 * Dima adaptations (thin):
 *   - Files are served from virtual read-only arrays linked into
 *     Application Flash instead of a filesystem (no NuttX VFS here).
 *   - Only the download opcodes QGC uses are implemented:
 *     kCmdNone, kCmdTerminateSession, kCmdResetSessions, kCmdOpenFileRO,
 *     kCmdReadFile, kCmdBurstReadFile. All write/list opcodes NAK with
 *     kErrUnknownCommand, per the Phase-6 read-only policy.
 *   - The stream-download follow-up (burst continuation from send()) is
 *     not needed: metadata files fit a single burst exchange.
 */

#include "lib/mavlink/mavlink_bridge.h"

#include <cstdint>
#include <cstring>

namespace dima::modules::mavlink {

class MavlinkMetadataFtp {
public:
    /// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
    /// This needs to be packed, because it's typecasted from mavlink_file_transfer_protocol_t.payload, which starts
    /// at a 3 byte offset, causing an unaligned access to seq_number and offset
    struct __attribute__((__packed__)) PayloadHeader {
        uint16_t seq_number;        ///< sequence number for message
        uint8_t  session;           ///< Session id for read and write commands
        uint8_t  opcode;            ///< Command opcode
        uint8_t  size;              ///< Size of data
        uint8_t  req_opcode;        ///< Request opcode returned in kRspAck, kRspNak message
        uint8_t  burst_complete;    ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
        uint8_t  padding;           ///< 32 bit alignment padding
        uint32_t offset;            ///< Offsets for List and Read commands
        uint8_t  data[1];           ///< command data, varies by Opcode (flexible)
    };

    /// @brief Command opcodes
    enum Opcode : uint8_t {
        kCmdNone,               ///< ignored, always acked
        kCmdTerminateSession,   ///< Terminates open Read session
        kCmdResetSessions,      ///< Terminates all open Read sessions
        kCmdListDirectory,      ///< List files in <path> from <offset>
        kCmdOpenFileRO,         ///< Opens file at <path> for reading, returns <session>
        kCmdReadFile,           ///< Reads <size> bytes from <offset> in <session>
        kCmdCreateFile,         ///< Creates file at <path> for writing, returns <session>
        kCmdWriteFile,          ///< Writes <size> bytes to <offset> in <session>
        kCmdRemoveFile,         ///< Remove file at <path>
        kCmdCreateDirectory,    ///< Creates directory at <path>
        kCmdRemoveDirectory,    ///< Removes Directory at <path>, must be empty
        kCmdOpenFileWO,         ///< Opens file at <path> for writing, returns <session>
        kCmdTruncateFile,       ///< Truncate file at <path> to <offset> length
        kCmdRename,             ///< Rename <path1> to <path2>
        kCmdCalcFileCRC32,      ///< Calculate CRC32 for file at <path>
        kCmdBurstReadFile,      ///< Burst download session file

        kRspAck = 128,          ///< Ack response
        kRspNak                 ///< Nak response
    };

    /// @brief Error codes returned in Nak response PayloadHeader.data[0].
    enum ErrorCode : uint8_t {
        kErrNone,
        kErrFail,               ///< Unknown failure
        kErrFailErrno,          ///< Command failed, errno sent back in PayloadHeader.data[1]
        kErrInvalidDataSize,    ///< PayloadHeader.size is invalid
        kErrInvalidSession,     ///< Session is not currently open
        kErrNoSessionsAvailable,///< All available Sessions in use
        kErrEOF,                ///< Offset past end of file for List and Read commands
        kErrUnknownCommand,     ///< Unknown command opcode
        kErrFailFileExists,     ///< File/directory exists already
        kErrFailFileProtected,  ///< File/directory is write protected
        kErrFileNotFound        ///< File/directory not found
    };

    static constexpr uint8_t kMaxDataLength =
        MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN -
        sizeof(PayloadHeader) + 1U;   /* data[1] counted in the header */

    /** Virtual read-only file served from embedded arrays. */
    struct VirtualFile {
        const char *path;
        const uint8_t *data;
        uint32_t size;
    };

    /** Transmit callback: send the prepared FILE_TRANSFER_PROTOCOL frame. */
    using SendFn = void (*)(void *ctx, mavlink_message_t &msg);

    MavlinkMetadataFtp(SendFn send, void *send_ctx) noexcept
        : send_(send), send_ctx_(send_ctx)
    {
    }

    void init(const VirtualFile *files, uint8_t count) noexcept
    {
        files_ = files;
        file_count_ = count > kMaxFiles ? kMaxFiles : count;
        _session_open = false;
        _session_file = nullptr;
    }

    /** Handle a possible FTP message (PX4 handle_message semantics). */
    void handle_message(const mavlink_message_t *msg) noexcept
    {
        mavlink_file_transfer_protocol_t ftp_request;
        mavlink_msg_file_transfer_protocol_decode(msg, &ftp_request);

        /* make sure this request is for us */
        if ((ftp_request.target_network == 0) &&
            (ftp_request.target_system == 0 ||
             ftp_request.target_system == MAVLINK_SYSTEM_ID) &&
            (ftp_request.target_component == 0 ||
             ftp_request.target_component == MAVLINK_COMPONENT_ID)) {
            _process_request(&ftp_request, msg->sysid, msg->compid);
        }
    }

private:
    static constexpr uint8_t kMaxFiles = 4U;
    static constexpr uint8_t kSessionId = 0U;

    void _process_request(mavlink_file_transfer_protocol_t *ftp_req,
                          uint8_t target_system_id,
                          uint8_t target_comp_id) noexcept
    {
        bool send_reply = true;

        PayloadHeader *payload =
            reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

        /* Basic sanity check; must be called before _workList. */
        if ((payload == nullptr) || (payload->size > kMaxDataLength)) {
            /* Cannot use payload arg, since it was corrupted */
            payload = _reset_response(ftp_req);
            _reply_nak(payload, kErrInvalidDataSize);
            _reply(ftp_req, target_system_id, target_comp_id);
            return;
        }

        uint8_t opcode = payload->opcode;
        payload->req_opcode = opcode;
        payload->opcode = kRspAck;
        payload->size = 0;
        payload->padding = 0;

        ErrorCode errorCode = kErrNone;

        switch (opcode) {
        case kCmdNone:
            /* ignored, always acked */
            break;

        case kCmdTerminateSession:
            errorCode = _workTerminate(payload);
            break;

        case kCmdResetSessions:
            errorCode = _workReset(payload);
            break;

        case kCmdOpenFileRO:
            errorCode = _workOpen(payload);
            break;

        case kCmdReadFile:
            errorCode = _workRead(payload);
            break;

        case kCmdBurstReadFile:
            errorCode = _workRead(payload);
            payload->burst_complete =
                (_session_file != nullptr &&
                 payload->offset + payload->size >= _session_size) ? 1 : 0;
            break;

        default:
            /* Everything else is unsupported on this read-only server. */
            errorCode = kErrUnknownCommand;
            break;
        }

        if (errorCode != kErrNone) {
            _reply_nak(payload, errorCode);
        }

        if (send_reply) {
            _reply(ftp_req, target_system_id, target_comp_id);
        }
    }

    void _reply(mavlink_file_transfer_protocol_t *ftp_req,
                uint8_t target_system_id, uint8_t target_comp_id) noexcept
    {
        PayloadHeader *payload =
            reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

        /* Sequence numbering: increment per reply (PX4 semantics). */
        payload->seq_number++;

        mavlink_message_t msg{};
        ftp_req->target_network = 0;
        ftp_req->target_system = target_system_id;
        ftp_req->target_component = target_comp_id;
        mavlink_msg_file_transfer_protocol_encode(
            MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, ftp_req);
        if (send_ != nullptr) {
            send_(send_ctx_, msg);
        }
    }

    PayloadHeader *_reset_response(mavlink_file_transfer_protocol_t *ftp_req) noexcept
    {
        PayloadHeader *payload =
            reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);
        std::memset(payload, 0, sizeof(*payload));
        return payload;
    }

    void _reply_nak(PayloadHeader *payload, ErrorCode error) noexcept
    {
        payload->opcode = kRspNak;
        payload->size = 1;
        payload->data[0] = static_cast<uint8_t>(error);
    }

    ErrorCode _workOpen(PayloadHeader *payload) noexcept
    {
        if (_session_open) {
            /* We only allow one open session. */
            return kErrNoSessionsAvailable;
        }

        /* Local buffer to enforce null termination */
        char name[kMaxDataLength + 1];
        const uint32_t copy_len =
            payload->size < kMaxDataLength ? payload->size : kMaxDataLength;
        std::memcpy(name, payload->data, copy_len);
        name[copy_len] = '\0';

        const VirtualFile *found = nullptr;
        for (uint8_t i = 0U; i < file_count_; ++i) {
            if (std::strcmp(files_[i].path, name) == 0) {
                found = &files_[i];
                break;
            }
        }

        if (found == nullptr) {
            return kErrFileNotFound;
        }

        _session_open = true;
        _session_file = found->data;
        _session_size = found->size;
        payload->session = kSessionId;
        /* ACK carries the file size (QGC/PX4 contract). */
        payload->size = sizeof(uint32_t);
        std::memcpy(payload->data, &_session_size, sizeof(uint32_t));
        return kErrNone;
    }

    ErrorCode _workRead(PayloadHeader *payload) noexcept
    {
        if (!_session_open || _session_file == nullptr) {
            return kErrInvalidSession;
        }

        if (payload->session != kSessionId) {
            return kErrInvalidSession;
        }

        if (payload->offset >= _session_size) {
            return kErrEOF;
        }

        uint32_t remaining = _session_size - payload->offset;
        uint32_t read_size = remaining < kMaxDataLength ? remaining
                                                        : kMaxDataLength;
        std::memcpy(payload->data, _session_file + payload->offset,
                    read_size);
        payload->size = static_cast<uint8_t>(read_size);
        return kErrNone;
    }

    ErrorCode _workTerminate(PayloadHeader *payload) noexcept
    {
        (void)payload;
        _session_open = false;
        _session_file = nullptr;
        _session_size = 0U;
        return kErrNone;
    }

    ErrorCode _workReset(PayloadHeader *payload) noexcept
    {
        (void)payload;
        _session_open = false;
        _session_file = nullptr;
        _session_size = 0U;
        return kErrNone;
    }

    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
    const VirtualFile *files_{nullptr};
    uint8_t file_count_{0U};
    bool _session_open{false};
    const uint8_t *_session_file{nullptr};
    uint32_t _session_size{0U};
};

}  // namespace dima::modules::mavlink
