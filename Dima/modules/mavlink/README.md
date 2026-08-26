# MAVLink 服务合同

`MavlinkService` 是 Application Runtime 唯一的 USB CDC 数据面所有者。它使用固定缓冲、无动态分配，负责 MAVLink v2 RX/TX、参数、只读 Component Metadata FTP、命令 ACK、任务空集合、TIMESYNC、RC、传感器流和 STATUSTEXT。

传感器流合同固定对照 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 `MAVLINK_MODE_CONFIG`、`HIGHRES_IMU.hpp` 与 `SCALED_IMU.hpp`；本地适配只保留实际存在的单套 IMU/磁力计实例和固定内存发送路径。IMU 新样本驱动对应周期发送，最近一次合法磁场值则保持可见；磁力计 freshness 只属于 `SYS_STATUS` 健康判定，不反向阻断或清零诊断流。

## 裁剪方言

方言固定为 29 条消息。相对原 24 条合同新增标准消息：

- `HIGHRES_IMU`：按 PX4 USB 默认 50 Hz 发布经过校准/旋转的 `vehicle_imu` 与最近一次合法 `vehicle_magnetometer`；accel、gyro、mag 分别使用 m/s²、rad/s、Gauss，`time_usec` 固定使用 IMU 样本时间，磁场更新位只在新磁力计样本进入时置位。
- `SCALED_IMU`：这是 PX4 注册的原始传感器 MAVLink 流，按 USB 默认 25 Hz 发布第 1 套 `vehicle_imu`、`vehicle_imu_status` 温度与最近一次合法 raw `sensor_mag`；accel、gyro、mag 分别转换为 mG、mrad/s、milliGauss，温度使用 cdegC。当前产品只有实例 0，不伪造 `SCALED_IMU2/3` 或 PX4 未注册的 `RAW_IMU`。
- `MESSAGE_INTERVAL`：响应 PX4/QGC 的 `MAV_CMD_GET_MESSAGE_INTERVAL`，并与 `MAV_CMD_SET_MESSAGE_INTERVAL`、`MAV_CMD_REQUEST_MESSAGE` 共用同一固定流表；负间隔停流、零恢复固件默认频率、正值保留请求的微秒间隔。100 Hz worker 只形成实际调度上限，不改写协议保存或 GET 回报的请求值。
- `GPS_RAW_INT`：5 Hz，包含原始 GPS fix、位置、速度、精度、卫星数和双天线 yaw；接收机在线但未定位时仍发送 `NO_FIX`，检测证据不依赖经纬度有效；数据真正超时后发送 `NO_GPS`。
- `SYS_STATUS`：1 Hz，持久保留曾探测设备的 present/enabled 位，数据超时仅清 health 位。

全部已配置周期流都支持 `MAV_CMD_REQUEST_MESSAGE`。与 PX4 `MavlinkStream::request_message()` 一致，一次请求是独立 one-shot，不受周期流 `last_send` 节拍限制；由 topic 更新驱动的 IMU 流在没有新样本时仍可拒绝本次请求。`HEARTBEAT` one-shot 直接发送；RC 保持 5 Hz、GPS 保持 5 Hz、`SYS_STATUS` 保持 1 Hz。GPS 可见性由持续的 `GPS_RAW_INT`/`SYS_STATUS` 表达，不再重复输出 detected/healthy/stale/not-detected 文本摘要。

## 校准命令

- Commander 保留 QGC Radio `param4=1` 事务。
- `SensorCalibration` 独占 gyro `param1=1`、mag `param2=1`、accel `param5=1` 及非 RC 的全零取消。
- 接受后立即发送 `COMMAND_ACK ACCEPTED`；长事务严格使用 PX4 v2 `[cal] ...` STATUSTEXT 驱动 QGC。`SensorCalibration` 与 PX4 Commander worker 一样运行在非实时 `wq:lp_default`，协议文本使用不受普通日志等级过滤的 RAW 路径；放入实时 `wq:sensors` 会被项目日志层拒绝格式化。Armed、另一校准进行中或传感器无新鲜样本时返回拒绝结果，并以 `[cal] calibration failed: <type>` 终止 QGC 等待。

## TX 与连接边界

优先级为 ACK、Heartbeat/Version、RC、Metadata FTP、传感器、参数、STATUSTEXT。物理 USB ready 下降沿会丢弃旧 RX 半帧，重置 parser/channel/FTP/参数传输会话，并恢复 PX4 USB 周期流默认节拍；`ETIMEDOUT/EIO/EPIPE` 保留 FTP 回复等待 QGC 同 sequence 重传。

周期遥测各自保存 PX4 风格的默认值和 `SET_MESSAGE_INTERVAL` 配置，不存在跨 message ID 的统一 10 Hz 限流策略。`COMMAND_ACK`、参数传输、Mission、Metadata/FTP、TIMESYNC、PING、STATUSTEXT 和版本/组件信息属于事务或诊断传输，不套用周期流节拍。

Windows 构建只证明生成、编译和链接。QGC Inspector 实时值、USB 长连接、带宽、丢包和板端传感器 health 转换均需实机验证。
