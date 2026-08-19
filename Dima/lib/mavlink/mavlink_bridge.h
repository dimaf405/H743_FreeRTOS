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
 * The dialect is TRIMMED (tools/mavlink/build_trimmed_dialect.py
 * builds dima.xml under build/generated/mavlink from the pinned XMLs in
 * tools/mavlink/message_definitions/):
 * only the messages the current firmware actually supports are
 * generated (HEARTBEAT, PROTOCOL_VERSION, AUTOPILOT_VERSION,
 * GLOBAL_POSITION_INT from standard/minimal, plus PING, TIMESYNC,
 * RC_CHANNELS, COMMAND_LONG/INT/ACK, PARAM_* classic+ext subset,
 * STATUSTEXT, the empty-mission set
 * MISSION_REQUEST_LIST/MISSION_COUNT/MISSION_CLEAR_ALL/MISSION_ACK, and the
 * Parameter Metadata surface FILE_TRANSFER_PROTOCOL/COMPONENT_METADATA with
 * deprecated COMPONENT_INFORMATION fallback — 24 messages in total). New
 * messages join the dialect only when the corresponding capability is
 * implemented.
 *
 * Platform adaptations:
 *   - Single communication buffer: the USB CDC link is the only
 *     MAVLink channel, owned exclusively by MavlinkService.
 *   - No signing, no convenience send functions: TX builds frames
 *     with mavlink_msg_*_encode() + mavlink_msg_to_send_buffer()
 *     and writes the bytes through the platform Console.
 */

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
