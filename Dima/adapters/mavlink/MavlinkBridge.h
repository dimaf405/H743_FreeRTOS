#pragma once
/*
 * MAVLink bridge header — thin adaptation of the official generated
 * C library (c_library_v2) to the Dima platform.
 *
 * Mirrors the role of PX4's mavlink_bridge_header.h: configure the
 * generated library, then include the pinned dialect.
 *
 * Upstream provenance (see docs/DIMA_SOURCE_MANIFEST.md):
 *   Generated from mavlink/mavlink commit
 *   33af200d25ec6f0925b49b1ba82bbf1294ea5f72 (the exact MAVLink
 *   submodule pointer of PX4-Autopilot v1.17.0 commit d6f12ad) via
 *   mavgen --lang C --wire-protocol 2.0.
 *
 * The dialect is trimmed by tools/mavlink/build_trimmed_dialect.py from the
 * pinned XML definitions and lock file. Those generator inputs and the
 * generated manifest are the authoritative message inventory; this adapter
 * does not duplicate that list. A message enters the dialect only with its
 * corresponding implemented capability.
 *
 * Platform adaptations:
 *   - Single communication buffer: the USB CDC link is the only
 *     MAVLink channel, owned exclusively by MavlinkService.
 *   - No signing, no convenience send functions: TX builds frames
 *     with mavlink_msg_*_encode() + mavlink_msg_to_send_buffer()
 *     and writes the bytes through the platform Console.
 */
/* 中文边界：MAVLink 消息清单、ID、CRC extra 和 codec 只来自固定 XML、lock 与
 * 生成 manifest；本适配头只配置单路 USB channel 和生成库编译开关，禁止复制
 * 或手写第二份消息表。MavlinkService 是该 parser/sequence 状态的唯一所有者。 */

#ifndef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS 1
#endif

/* MAVLink v2 only, matching MAVLINK_STX 253 in the generated code. */
#ifndef MAVLINK_NO_CONVERSION_HELPERS
#define MAVLINK_NO_CONVERSION_HELPERS 1
#endif

/* Keep generated framing/checksum helpers in one implementation TU. Without
 * this, every split message owner emits another private helper copy. */
#ifndef MAVLINK_SEPARATE_HELPERS
#define MAVLINK_SEPARATE_HELPERS 1
#endif

/* Fixed PX4 Rover identity (sysid/compid 1/1). Use these constants
 * in encode calls; the MAVLINK_USE_CONVENIENCE_FUNCTIONS global
 * 'mavlink_system' object is intentionally not enabled. */
#ifndef MAVLINK_SYSTEM_ID
#define MAVLINK_SYSTEM_ID 1
#endif
#ifndef MAVLINK_COMPONENT_ID
#define MAVLINK_COMPONENT_ID 1
#endif

/* Generated helpers otherwise place RX/TX channel state in function-local
 * statics, creating one sequence counter and parser buffer per translation
 * unit. All MAVLink owners share the same USB channel, so bind every inline
 * helper to one externally defined state pair. */
#include "mavlink_types.h"

#define MAVLINK_GET_CHANNEL_STATUS 1
#define MAVLINK_GET_CHANNEL_BUFFER 1

#ifdef __cplusplus
extern "C" {
#endif

extern mavlink_status_t dima_mavlink_channel_status[MAVLINK_COMM_NUM_BUFFERS];
extern mavlink_message_t dima_mavlink_channel_buffer[MAVLINK_COMM_NUM_BUFFERS];

static inline mavlink_status_t *mavlink_get_channel_status(uint8_t channel)
{
    /* 生成代码只会传 MAVLINK_COMM_0；数组容量为 1，调用侧不得传任意 channel。 */
    return &dima_mavlink_channel_status[channel];
}

static inline mavlink_message_t *mavlink_get_channel_buffer(uint8_t channel)
{
    return &dima_mavlink_channel_buffer[channel];
}

#ifdef __cplusplus
}
#endif

#include "dima/mavlink.h"
