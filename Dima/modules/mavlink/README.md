# MAVLink 服务合同

`MavlinkService` 是 Application Runtime 唯一的 USB CDC 数据面所有者。它使用固定缓冲、无动态分配，负责 MAVLink v2 RX/TX、参数、只读 Component Metadata FTP、命令 ACK、任务、Onboard Log、`STORAGE_INFORMATION`、TIMESYNC、RC、传感器流和 STATUSTEXT。

传感器与估计器流合同固定对照 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 `MAVLINK_MODE_CONFIG` 以及 `HIGHRES_IMU`、`SCALED_IMU`、`ATTITUDE`、`LOCAL_POSITION_NED`、`GLOBAL_POSITION_INT`、`ESTIMATOR_STATUS` stream；本地适配只保留实际存在的单套传感器/EKF2 实例和固定内存发送路径。

## 原生方言与运行策略

`tools/mavlink/message_definitions/dima.xml` 是唯一 wire 根，只 include 固定 MAVLink definitions commit `33af200d25ec6f0925b49b1ba82bbf1294ea5f72` 的 `common.xml`。正式 Make 直接执行 pymavlink 2.4.47 commit `fcaa2c7d25e3169dc66155929c338487941555e9` 的原始 `mavgen.py --lang C --wire-protocol 2.0`；消息 ID、字段、CRC、payload 和 codec 不在 Dima Python/C++ 中复制。

`mavlink_runtime.yaml` 只声明产品实际发送频率、请求行为和 inbound handler。薄生成器从 mavgen 已生成头验证符号并生成 C++ 调度合同，所有 message ID 都引用 `MAVLINK_MSG_ID_*` 宏。完整 common wire 定义不等于固件宣称实现全部业务能力；只有运行策略和 handler 闭合的消息才进入实际收发路径。

当前真实传感器流行为包括：

- `HIGHRES_IMU`：按 PX4 USB 默认 50 Hz 发布经过校准/旋转的 `vehicle_imu` 与最近一次合法 `vehicle_magnetometer`；accel、gyro、mag 分别使用 m/s²、rad/s、Gauss，`time_usec` 固定使用 IMU 样本时间，磁场更新位只在新磁力计样本进入时置位。
- `SCALED_IMU`：这是 PX4 注册的原始传感器 MAVLink 流，按 USB 默认 25 Hz 发布第 1 套 `vehicle_imu`、`vehicle_imu_status` 温度与最近一次合法 raw `sensor_mag`；accel、gyro、mag 分别转换为 mG、mrad/s、milliGauss，温度使用 cdegC。当前产品只有实例 0，不伪造 `SCALED_IMU2/3` 或 PX4 未注册的 `RAW_IMU`。
- `MESSAGE_INTERVAL`：响应 PX4/QGC 的 `MAV_CMD_GET_MESSAGE_INTERVAL`，并与 `MAV_CMD_SET_MESSAGE_INTERVAL`、`MAV_CMD_REQUEST_MESSAGE` 共用同一固定流表；负间隔停流、零恢复固件默认频率、正值保留请求的微秒间隔。100 Hz worker 只形成实际调度上限，不改写协议保存或 GET 回报的请求值。
- `GPS_RAW_INT`：5 Hz，包含原始 GPS fix、位置、速度、精度、卫星数和双天线 yaw；接收机在线但未定位时仍发送 `NO_FIX`，检测证据不依赖经纬度有效；数据真正超时后发送 `NO_GPS`。
- `SYS_STATUS`：1 Hz，持久保留曾探测设备的 present/enabled 位，数据超时仅清 health 位。GPS health 还要求 EKF2 `estimator_gps_status.checks_passed`，但该位不反向遮蔽原始 `GPS_RAW_INT`。
- `ATTITUDE`：50 Hz，从 `vehicle_attitude` 按 PX4 Hamilton quaternion 转 roll/pitch/yaw；机体系角速度来自同一 EKF2 的 `vehicle_odometry`，过期时仅将 rate 置零。
- `LOCAL_POSITION_NED`：30 Hz，直接映射 `vehicle_local_position` 的 NED position/velocity。
- `GLOBAL_POSITION_INT`：10 Hz，映射 WGS84 global position 与 NED velocity；严格沿用 PX4 v1.17 的回退合同：存在有效 `home_position` 时为 `gpos.alt-home.alt`，否则为 `gpos.alt`。N1 尚无 `home_position`，因此当前固定走官方无 Home 回退，不把本地 NED 原点另解释为 Home。
- `ESTIMATOR_STATUS`：5 Hz，逐项映射 PX4 `estimator_status` 的 innovation ratio、accuracy 和 `solution_status_flags`，MAVLink 层不重建第二套估计器健康模型。
- `STORAGE_INFORMATION`：对照 PX4 v1.17 只响应 `MAV_CMD_REQUEST_MESSAGE`（并保留 deprecated `MAV_CMD_REQUEST_STORAGE_INFORMATION`）；索引 0/1 返回第一块 SD，可用时上报 `READY` 及 MiB 容量，不可用时上报 `EMPTY/count=0`。`f_getfree` 只在 `wq:storage` 执行，MAVLink owner 仅处理固定 Ring 中的响应。

全部已配置周期流都支持 `MAV_CMD_REQUEST_MESSAGE`。与 PX4 `MavlinkStream::request_message()` 一致，一次请求是独立 one-shot，不受周期流 `last_send` 节拍限制；由 topic 更新驱动的 IMU 流在没有新样本时仍可拒绝本次请求。`HEARTBEAT` one-shot 直接发送；RC 保持 5 Hz、GPS 保持 5 Hz、`SYS_STATUS` 保持 1 Hz。GPS 可见性由持续的 `GPS_RAW_INT`/`SYS_STATUS` 表达，不再重复输出 detected/healthy/stale/not-detected 文本摘要。

## 校准命令

- Commander 保留 QGC Radio `param4=1` 事务。
- `SensorCalibration` 独占 gyro `param1=1`、mag `param2=1`、accel `param5=1` 及非 RC 的全零取消。
- 接受后立即发送 `COMMAND_ACK ACCEPTED`；长事务严格使用 PX4 v2 `[cal] ...` STATUSTEXT 驱动 QGC。`SensorCalibration` 与 PX4 Commander worker 一样运行在非实时 `wq:lp_default`，协议文本使用不受普通日志等级过滤的 RAW 路径；放入实时 `wq:sensors` 会被项目日志层拒绝格式化。Armed、另一校准进行中或传感器无新鲜样本时返回拒绝结果，并以 `[cal] calibration failed: <type>` 终止 QGC 等待。

## Onboard Log

- `MavlinkLogHandler` 对照 PX4 v1.17.0 同名实现处理 `LOG_REQUEST_LIST/DATA/END/ERASE`，并复用同一 storage worker/Ring 生成 `STORAGE_INFORMATION`；日志 ID 从 0 开始，`LOG_DATA` 长度直接由 mavgen 字段容量派生。
- PX4 的文件扫描、稳定列表、按 offset 读取和整树擦除语义保留；平台适配只把 POSIX 调用换成 `LogFileStore`，实际 FatFs/SDMMC 工作固定在 `wq:storage`，通信队列仅发送固定 8 槽响应 Ring。
- 无卡或无文件按 `common.xml` 强制回一条 `id=0,num_logs=0`，使 QGC 结束 Refresh；板上无 RTC，`LOG_ENTRY.time_utc=0`，避免用 FatFs 固定日期伪装真实采集时间。

## TX 与连接边界

优先级为 ACK、Heartbeat/Version、RC、Metadata FTP、传感器、Onboard Log、参数、STATUSTEXT。物理 USB ready 下降沿会丢弃旧 RX 半帧，重置 parser/channel/FTP/参数/日志传输会话，并恢复 PX4 USB 周期流默认节拍；`ETIMEDOUT/EIO/EPIPE` 保留 FTP 回复等待 QGC 同 sequence 重传。

周期遥测各自保存 PX4 风格的默认值和 `SET_MESSAGE_INTERVAL` 配置，不存在跨 message ID 的统一 10 Hz 限流策略。`COMMAND_ACK`、参数传输、Mission、Metadata/FTP、TIMESYNC、PING、STATUSTEXT 和版本/组件信息属于事务或诊断传输，不套用周期流节拍。

Windows 构建只证明生成、编译和链接。QGC Inspector 实时值、USB 长连接、带宽、丢包和板端传感器 health 转换均需实机验证。
