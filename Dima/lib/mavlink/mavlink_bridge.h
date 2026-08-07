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
 * The dialect is TRIMMED (tmp/mavlink_gen/build_trimmed_dialect.py
 * builds dima.xml): only the messages the current firmware actually
 * supports are generated (HEARTBEAT, PROTOCOL_VERSION,
 * AUTOPILOT_VERSION, GLOBAL_POSITION_INT from standard/minimal, plus
 * PING, TIMESYNC, COMMAND_LONG/ACK, PARAM_* classic+ext subset,
 * FILE_TRANSFER_PROTOCOL, STATUSTEXT). New messages join the dialect
 * only when the corresponding capability is implemented.
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

/* Fixed PX4 Rover identity (sysid/compid 1/1). Use these constants
 * in encode calls; the MAVLINK_USE_CONVENIENCE_FUNCTIONS global
 * 'mavlink_system' object is intentionally not enabled. */
#ifndef MAVLINK_SYSTEM_ID
#define MAVLINK_SYSTEM_ID 1
#endif
#ifndef MAVLINK_COMPONENT_ID
#define MAVLINK_COMPONENT_ID 1
#endif

#include "c_library_v2/dima/mavlink.h"
