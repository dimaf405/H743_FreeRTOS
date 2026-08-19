#include "lib/mavlink/mavlink_bridge.h"

/* 单路 USB data plane 只有一份 parser buffer 与 RX/TX sequence 状态；
 * 分源后的消息编码器不得各自持有生成库的函数局部 static。 */
extern "C" {

mavlink_status_t dima_mavlink_channel_status[MAVLINK_COMM_NUM_BUFFERS]{};
mavlink_message_t dima_mavlink_channel_buffer[MAVLINK_COMM_NUM_BUFFERS]{};

} // extern "C"

/* MAVLINK_SEPARATE_HELPERS leaves declarations in protocol.h; instantiate the
 * pinned generated implementation exactly once beside the shared state. */
extern "C" {
#include "mavlink_helpers.h"
}
