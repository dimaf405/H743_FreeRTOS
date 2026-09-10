#include "MavlinkBridge.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

mavlink_status_t * mavlink_get_channel_status(uint8_t channel)
{
    /* 固定调用域为 USB COMM_0、UART COMM_1；生成库和所有编码器共用这两份状态。 */
    return &dima_mavlink_channel_status[channel];
}

mavlink_message_t * mavlink_get_channel_buffer(uint8_t channel)
{
    return &dima_mavlink_channel_buffer[channel];
}
