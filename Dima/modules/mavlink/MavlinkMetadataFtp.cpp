#include "MavlinkMetadataFtp.hpp"

#include <cerrno>
#include <cstring>

namespace dima::modules::mavlink {

MavlinkMetadataFtp::MavlinkMetadataFtp(
    SendFn send, void *send_context) noexcept
    : send_(send), send_context_(send_context)
{
}

void MavlinkMetadataFtp::init(const VirtualFile *files,
                              std::uint8_t count) noexcept
{
    files_ = files;
    file_count_ = count > kMaxFiles ? kMaxFiles : count;
    reset();
}

void MavlinkMetadataFtp::reset() noexcept
{
    session_open_ = false;
    session_file_ = nullptr;
    session_size_ = 0U;
    session_owner_system_ = 0U;
    session_owner_component_ = 0U;
    session_last_activity_us_ = 0U;
    reply_ = ReplySlot{};
}

void MavlinkMetadataFtp::handle_message(const mavlink_message_t *message,
                                        std::uint64_t now_us) noexcept
{
    if (message == nullptr) {
        return;
    }
    expire_session(now_us);

    mavlink_file_transfer_protocol_t request{};
    mavlink_msg_file_transfer_protocol_decode(message, &request);
    if (request.target_network != 0U ||
        (request.target_system != 0U &&
         request.target_system != MAVLINK_SYSTEM_ID) ||
        (request.target_component != 0U &&
         request.target_component != MAVLINK_COMPONENT_ID)) {
        return;
    }

    Payload request_payload{};
    std::memcpy(&request_payload, request.payload, sizeof(request_payload));
    const PayloadHeader &header = request_payload.header;
    const RequestKey key{
        message->sysid,
        message->compid,
        header.seq_number,
        header.session,
        header.opcode,
        header.size,
        header.offset,
        request_fingerprint(request.payload, sizeof(request.payload)),
    };

    if (reply_.pending) {
        if (same_request(reply_.key, key)) {
            touch_session(key, now_us);
        }
        /* Preserve the unsent response. QGC retries unanswered requests;
         * accepting a different request here would break request/reply
         * ordering and allow another source to overwrite it. */
        return;
    }
    if (reply_.valid && same_request(reply_.key, key)) {
        touch_session(key, now_us);
        reply_.pending = true;
        reply_.retry_count = 0U;
        return;
    }

    process_request(request_payload, key, now_us);
}

bool MavlinkMetadataFtp::service(std::uint64_t now_us) noexcept
{
    expire_session(now_us);
    return flush_pending();
}

bool MavlinkMetadataFtp::flush_pending() noexcept
{
    if (!reply_.pending) {
        return true;
    }
    if (send_ == nullptr) {
        reset();
        return false;
    }

    mavlink_message_t message{};
    mavlink_msg_file_transfer_protocol_encode(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
        &message, &reply_.response);
    if (send_(send_context_, message)) {
        reply_.pending = false;
        reply_.retry_count = 0U;
        return true;
    }

    const int error = errno;
    if (error == EAGAIN && reply_.retry_count < kMaxTxRetries) {
        ++reply_.retry_count;
        return false;
    }

    /* ETIMEDOUT is ambiguous: USB may have delivered the frame while only
     * its completion callback missed our 5 ms deadline. Re-sending that
     * old sequence can cancel QGC's timeout for its next request. Only
     * EAGAIN is actively retried; every other error and EAGAIN exhaustion
     * leaves the exact response cached for QGC's same-sequence retry.
     * The physical USB falling edge and session timeout remain the
     * authoritative reset paths. */
    reply_.pending = false;
    reply_.retry_count = 0U;
    return false;
}

std::uint64_t MavlinkMetadataFtp::request_fingerprint(
    const std::uint8_t *data, std::size_t size) noexcept
{
    std::uint64_t hash = 14695981039346656037ULL;
    for (std::size_t index = 0U; index < size; ++index) {
        hash ^= data[index];
        hash *= 1099511628211ULL;
    }
    return hash;
}

bool MavlinkMetadataFtp::same_request(
    const RequestKey &left, const RequestKey &right) noexcept
{
    return left.source_system == right.source_system &&
           left.source_component == right.source_component &&
           left.sequence == right.sequence &&
           left.session == right.session &&
           left.opcode == right.opcode &&
           left.size == right.size && left.offset == right.offset &&
           left.fingerprint == right.fingerprint;
}

bool MavlinkMetadataFtp::session_owned_by(
    const RequestKey &key) const noexcept
{
    return session_open_ && key.session == kSessionId &&
           key.source_system == session_owner_system_ &&
           key.source_component == session_owner_component_;
}

void MavlinkMetadataFtp::touch_session(
    const RequestKey &key, std::uint64_t now_us) noexcept
{
    if (session_owned_by(key)) {
        session_last_activity_us_ = now_us;
    }
}

void MavlinkMetadataFtp::expire_session(std::uint64_t now_us) noexcept
{
    if (session_open_ && session_last_activity_us_ != 0U &&
        (now_us < session_last_activity_us_ ||
         now_us - session_last_activity_us_ >= kSessionTimeoutUs)) {
        reset_session();
        reply_ = ReplySlot{};
    }
}

void MavlinkMetadataFtp::process_request(const Payload &request,
                                         const RequestKey &key,
                                         std::uint64_t now_us) noexcept
{
    ReplySlot next{};
    mavlink_file_transfer_protocol_t &response = next.response;
    Payload reply{};
    const std::uint8_t requested_size = request.header.size;
    const std::uint8_t requested_opcode = request.header.opcode;

    response.target_network = 0U;
    response.target_system = key.source_system;
    response.target_component = key.source_component;
    reply.header.seq_number = static_cast<std::uint16_t>(key.sequence + 1U);
    reply.header.session = request.header.session;
    reply.header.opcode = kRspAck;
    reply.header.req_opcode = requested_opcode;
    reply.header.offset = request.header.offset;

    ErrorCode error = kErrNone;
    if (requested_size > kMaxDataLength) {
        error = kErrInvalidDataSize;
    } else {
        switch (requested_opcode) {
        case kCmdNone:
            break;
        case kCmdTerminateSession:
            error = terminate_session(key);
            break;
        case kCmdResetSessions:
            error = reset_sessions();
            break;
        case kCmdOpenFileRO:
            error = open_file(request, reply, key, now_us);
            break;
        case kCmdReadFile:
            error = read_file(request, reply, key, false, now_us);
            break;
        case kCmdBurstReadFile:
            error = read_file(request, reply, key, true, now_us);
            break;
        default:
            error = kErrUnknownCommand;
            break;
        }
    }

    if (error != kErrNone) {
        reply.header.opcode = kRspNak;
        reply.header.size = 1U;
        reply.data[0] = static_cast<std::uint8_t>(error);
    }

    std::memcpy(response.payload, &reply, sizeof(reply));
    next.key = key;
    next.valid = true;
    next.pending = true;
    next.retry_count = 0U;
    reply_ = next;
}

MavlinkMetadataFtp::ErrorCode MavlinkMetadataFtp::open_file(
    const Payload &request, Payload &reply, const RequestKey &key,
    std::uint64_t now_us) noexcept
{
    if (session_open_) {
        return kErrNoSessionsAvailable;
    }
    if (request.header.size == 0U ||
        request.header.size > kMaxDataLength) {
        return kErrInvalidDataSize;
    }

    const VirtualFile *file = nullptr;
    for (std::uint8_t index = 0U; index < file_count_; ++index) {
        if (files_ == nullptr || files_[index].path == nullptr) {
            continue;
        }
        const std::size_t path_length = std::strlen(files_[index].path);
        if (path_length == request.header.size &&
            std::memcmp(files_[index].path, request.data,
                        path_length) == 0) {
            file = &files_[index];
            break;
        }
    }
    if (file == nullptr || file->data == nullptr) {
        return kErrFileNotFound;
    }

    session_open_ = true;
    session_file_ = file->data;
    session_size_ = file->size;
    session_owner_system_ = key.source_system;
    session_owner_component_ = key.source_component;
    session_last_activity_us_ = now_us;
    reply.header.session = kSessionId;
    reply.header.size = sizeof(session_size_);
    std::memcpy(reply.data, &session_size_, sizeof(session_size_));
    return kErrNone;
}

MavlinkMetadataFtp::ErrorCode MavlinkMetadataFtp::read_file(
    const Payload &request, Payload &reply, const RequestKey &key, bool burst,
    std::uint64_t now_us) noexcept
{
    if (!session_owned_by(key) || session_file_ == nullptr) {
        return kErrInvalidSession;
    }
    if (request.header.size == 0U) {
        return kErrInvalidDataSize;
    }
    if (request.header.offset >= session_size_) {
        return kErrEOF;
    }

    const std::uint32_t remaining = session_size_ - request.header.offset;
    std::uint32_t read_size = request.header.size;
    if (read_size > kMaxDataLength) {
        read_size = kMaxDataLength;
    }
    if (read_size > remaining) {
        read_size = remaining;
    }
    std::memcpy(reply.data,
                session_file_ + request.header.offset, read_size);
    reply.header.size = static_cast<std::uint8_t>(read_size);
    reply.header.burst_complete = burst ? 1U : 0U;
    session_last_activity_us_ = now_us;
    return kErrNone;
}

MavlinkMetadataFtp::ErrorCode MavlinkMetadataFtp::terminate_session(
    const RequestKey &key) noexcept
{
    if (!session_owned_by(key)) {
        return kErrInvalidSession;
    }
    reset_session();
    return kErrNone;
}

MavlinkMetadataFtp::ErrorCode MavlinkMetadataFtp::reset_sessions() noexcept
{
    reset_session();
    return kErrNone;
}

void MavlinkMetadataFtp::reset_session() noexcept
{
    session_open_ = false;
    session_file_ = nullptr;
    session_size_ = 0U;
    session_owner_system_ = 0U;
    session_owner_component_ = 0U;
    session_last_activity_us_ = 0U;
}

} // namespace dima::modules::mavlink
