# UM982 GPS 驱动

本模块实现 GPS 所有串口上的异步 UM982 NMEA/移动基线链路。飞控的 `SERIAL1..8` 是物理 STM32 UART 编号，与接收机内部 COM1/2/3 编号相互独立。

## 固定上游与手册

- PX4-Autopilot v1.17.0：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- PX4-GPSDrivers：`0b9695881bd1e8f830ab4538ab3acc0050019eba`
- Unicore UM982/N4 command manual：V2 EN R1.15（2026-06）

## 参数合同

- `GPS_1_CONFIG`：由板级 `serial_ports.json` 生成；0 禁用，1..8 选择 Dima `SERIAL1..8`。为兼容已写入的配置，当其为 0 时仍接受唯一的 `SERIALx_FUNCTION=GPS`。
- `GPS_1_PROTOCOL`：与 PX4 一致使用 `0=Auto detect`、`6=NMEA`；NMEA frontend 同时解析 UM982 的 CRC32 `AGRICA/UNIAGRICA/UNIHEADINGA` 扩展。
- GPS 占用端口时固定使用由 `um982_messages.json` 生成的 `460800 bit/s` 产品合同；`SERIALx_BAUD` 仍是该物理端口脱离 GPS 所有权后的通用配置。驱动保留 UM982 官方八档扫描能力只用于找回已有配置，检测成功后通过受控配置链把接收机和飞控 UART 统一回 460800。
- `GPS_YAW_OFFSET`：采用 PX4 双天线定义，0..360 deg、顺时针增加；UM982 heading 按 `raw + 180° - offset` 转成车体 yaw。

## 探测与发布

- 首次启动保持 PX4 的数据优先边界，不等待 Disarmed 或维护票据：先试参数目标 baud，再异步扫描 UM982 官方八档，以 CRC/XOR 有效的 GGA/RMC/AGRICA/UNIHEADINGA 锁定实际 baud，并立即发布可用数据。
- 顶层状态固定为 `等待端口 → 检测 GPS → 读取配置 → 正常运行 / 下发配置 → 保存配置 → 等待接收机稳定并重新检测`。TX-complete、查询 deadline、COM1/2/3 探测和逐条命令索引只是状态内部的非阻塞步骤，不扩张为产品状态。
- 检测成功后用 `CONFIG` 和必要时的 `VERSIONA COM1/2/3` 识别当前接收机内部端口，再用 `UNILOGLIST` 读取输出配置。配置合法就直接进入正常运行；无法唯一识别 COM 时不猜测、不写 baud，现有 GPS 数据继续发布。
- 配置不合法时才申请 Disarmed、BootHealth 和 appMain-IWDG 共同批准的维护票据。实际 baud 与目标不同时发送 `CONFIG COMn 460800 8 N 1` 并等待 TX-complete 后切换飞控 UART；只对缺失、重复或周期错误的输出执行 `UNLOG COMn <message>`，再按照 `um982_messages.json` 经工具生成的 PX4 消息名称与本产品 10 Hz 合同，以 `<message> COMn 0.1` 明确恢复到已识别端口。完成后发送一次 `SAVECONFIG`，等待接收机稳定，再从自动 baud 检测重新读取配置确认结果。
- `CONFIG`、定向 COM 探测或 `UNILOGLIST` 失败只把配置降级并在 30 秒后重试，不会把已检测到的接收机重新判为 offline。若 UM982 COM2/COM3 被持久化为 `RXTYPE=NONE/RTCM` 而不响应命令，固件保留 data-only 工作；必须从接收机 COM1 人工恢复命令输入。
- 每次 WorkItem 最多处理 2048 字节；达到预算后屏蔽 UART 的即时重复唤醒并强制延迟 1 ms。错误 baud 造成的 UART framing/noise 恢复唤醒被限制为最高 10 Hz，避免 `wq:io` 压住 BootHealth、MAVLink/QGC 和其他低优先级任务。
- 发布 `sensor_gps`、`vehicle_gps_position` 和 PX4 `estimator_gps_status`。GGA 是位置的最低输入，AGRICA 提供完整 NED 速度，缺少 AGRICA 时使用新鲜 RMC 提供水平速度；只有 GGA 时仍发布位置并明确 `vel_ned_valid=false`。标准 GGA `quality=0` 与 RMC `status=V` 即使坐标留空也作为“接收机在线但无定位”接受。没有新鲜 GGA、但仍收到有效 RMC/AGRICA/UNIHEADINGA 时，以最高 2 Hz 发布位置未知的 `NO_FIX` 在线状态，不伪造经纬度；声称有效 fix 却缺失坐标的帧仍被拒绝。
- 协议边界校验 N/S 与 E/W、经纬度范围、DOP/标准差、RMC 速度/航向/UTC、GPS 周内毫秒、移动基线长度/航向精度。严格跟随固定 PX4-GPSDrivers：NMEA/Unicore checksum、结构、未知消息和 overflow 静默丢弃，不逐帧打印、不更新测量缓存，也不进入 DataValidator 错误密度；持续收不到有效数据时由 1.3 s timeout 转为 offline。GPGST 空字段按 PX4 保留零初始化值。
- `estimator_gps_status` 按 PX4 子集发布 fix、卫星数、PDOP、EPH、EPV、速度精度和 spoof/auth failure；首次连续通过 10 s 后才设置 `checks_passed`，故障后按 simplified checks 连续通过 1 s 恢复。没有 EKF 静止/飞行状态时，drift/speed-offset 位保持 false、数值保持 NaN，不伪造检查结果。
- MAVLink 以 5 Hz 发布 `GPS_RAW_INT`，`SYS_STATUS` 报告 GPS present/stream health；`checks_passed=false` 不遮掉接收机真实 fix，合法 `NO_FIX` 仍保持 QGC GPS 图标，数据真正超时后才发送 `NO_GPS`。
- GPS 日志只保留状态转换型核心错误：每次离线只报告一次 SERIAL/baud/RX/UART/协议丢弃摘要，配置失败在恢复前只报告一次，最终样本或流健康故障只保留一条 mask 且至少间隔 30 秒；checksum、报文结构、未知消息和 overflow 不发送 QGC `STATUSTEXT`。不再输出 baud 扫描、探测成功、配置成功、恢复成功和周期健康/超时消息。
- 接收机配置或数据丢失不作为 BootHealth 或手动驾驶解锁条件；检测后的接收机配置和运行时串口改动都持有原子 arming interlock，并在维护事务失败、超时或完成时释放。

## 板端验证边界

UART 电平与接线、接收机实际输出端口、八档 baud 探测、生成合同声明的消息实际频率、RTK fix/双天线 yaw、10 s/1 s 资格滞回和失联恢复仍是 `BOARD PENDING`。在 QGC MAVLink Inspector 中同时检查 `GPS_RAW_INT` 与 `SYS_STATUS`，并在固件内部检查 `estimator_gps_status`；不能用一次性日志或 Windows 构建替代实时数据。
