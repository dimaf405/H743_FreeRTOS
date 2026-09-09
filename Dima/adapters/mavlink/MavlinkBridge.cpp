#include "MavlinkBridge.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

mavlink_status_t * mavlink_get_channel_status(uint8_t channel)
{
    /* 生成代码只会传 MAVLINK_COMM_0；数组容量为 1，调用侧不得传任意 channel。 */
    return &dima_mavlink_channel_status[channel];
}

mavlink_message_t * mavlink_get_channel_buffer(uint8_t channel)
{
    return &dima_mavlink_channel_buffer[channel];
}
