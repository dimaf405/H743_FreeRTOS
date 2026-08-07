# MAVLink 协议模块

- **职责：** MAVLink v2.0 协议处理——USB CDC 传输独占、RX/TX 主循环、1 Hz 心跳定标、COMMAND_LONG 接收与 `vehicle_command` 发布、Classic + Ext 参数协议、TIMESYNC 时间同步和只读 Component Metadata FTP 服务器。
- **禁止事项：** 不反向依赖 `rover/`，不直接包含 HAL/CMSIS/Core/Board 头文件，不在此模块实现业务逻辑（业务逻辑归 `rover/` 或 `modules/` 其他子模块）。
- **来源：** PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` `src/modules/mavlink/` 移植适配。
- **上游 API 保留：** 保留 PX4 receiver 的 target 过滤、`vehicle_command`/`vehicle_command_ack` 发布语义和参数协议流式行为；MAVLink 帧格式由官方 `c_library_v2` 裁剪方言生成。

## 文件清单

| 文件 | 职责 |
|---|---|
| `MavlinkService.hpp/cpp` | RX/TX 主循环；独占 USB CDC 传输；`mavlink_parse_char` 逐字节解析并按消息 ID 分发；TX 优先级：COMMAND_ACK → HEARTBEAT → 参数流/FTP → STATUSTEXT；延迟 reboot 在 ACK 写完后执行 |
| `MavlinkIdentity.hpp` | 系统身份纯数据——sysid/compid、MAV_TYPE_GROUND_ROVER、MAV_AUTOPILOT_PX4、固件版本编码、capabilities 位图和硬件 UID；无线程、无工作队列 |
| `HeartbeatPacer.hpp` | 1 Hz HEARTBEAT 定标 + `AUTOPILOT_VERSION` 按需打包；订阅 `vehicle_status` 刷新 `base_mode`/`system_status` |
| `MavlinkCommands.hpp` | COMMAND_LONG 接收：target 过滤、`MAV_CMD_REQUEST_MESSAGE` 解析、`vehicle_command` 发布供 Commander 仲裁、本地服务的请求直接 ACK |
| `MavlinkParameters.hpp` | Classic 参数协议——`PARAM_REQUEST_LIST`/`PARAM_REQUEST_READ`/`PARAM_SET` 处理和 `PARAM_VALUE` 流式发送；订阅 `parameter_update` 回传未保存值 |
| `MavlinkTimesync.hpp` | TIMESYNC 处理——远端消息打时间戳回传，本地消息喂入 `Timesync` 收敛滤波器；省略 SYSTEM_TIME 时钟设置（平台无 RTC） |
| `MavlinkMetadataFtp.hpp` | 只读 MAVLink FTP 服务器——仅实现 QGC 需要的下载操作码（OpenFileRO/ReadFile/BurstReadFile/Terminate/Reset），写/列操作码 NAK `kErrUnknownCommand`；文件从 Flash 只读数组提供 |

## 依赖方向

```
modules/mavlink/
  → platform/api/          （Console、BootControl、Time、Platform capability）
  → lib/mavlink/            （c_library_v2 裁剪方言）
  → messages/               （vehicle_command、vehicle_command_ack、vehicle_status、mavlink_log、parameter_update）
  → middleware/              （uORB Publication/SubscriptionData、work_queue ScheduledWorkItem、lifecycle ModuleBase、logging、parameters）
  → lib/timesync/           （Timesync 收敛滤波器）
```

禁止反向依赖 `rover/`。禁止直接包含 `hal_*.h`、`cmsis_*.h`、`Core/`、`Boards/` 头文件。

## 调度约束

- `MavlinkService` 在 `wq:lp_default` 独立工作队列运行，100 Hz 定时唤醒（10 ms 间隔）。
- 心跳定标 1 Hz（`HeartbeatPacer::kIntervalUs = 1 000 000`）。
- RX 批量读取 256 字节，逐字节 `mavlink_parse_char` 解析。
- TX 固定缓冲区，无动态分配；reboot ACK 使用 250 ms 超时确保帧到达主机后再复位。
- STATUSTEXT 每次 Run 最多发送 2 条，超过 5 s 的日志丢弃。

## 消息裁剪

Phase-6 允许列表（12 条消息）：HEARTBEAT、PROTOCOL_VERSION、AUTOPILOT_VERSION、PING、TIMESYNC、COMMAND_LONG、COMMAND_ACK、PARAM_REQUEST_LIST、PARAM_REQUEST_READ、PARAM_SET、PARAM_VALUE、FILE_TRANSFER_PROTOCOL、STATUSTEXT。`c_library_v2` 由裁剪方言 XML 通过 `mavgen` 生成，不引入未实现功能的消息。
