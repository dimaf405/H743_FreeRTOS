# PX4 GPS/IMU 诊断与数据合法性规格

状态：`IMPLEMENTED / BOARD PENDING`（2026-08-25，保持当前 Manual 解锁策略）
目标分支：`feature/dima-phase3`
固定参考：PX4-Autopilot v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
GPS 驱动参考：PX4-GPSDrivers `0b9695881bd1e8f830ab4538ab3acc0050019eba`

## 1. 目标

把 PX4 的传感器诊断思想接入 Dima Rover，建立可执行、可测试的 GPS/IMU 数据合法性合同：

1. 非法帧和非法样本不得进入校准后 `vehicle_imu` 或可供导航使用的 GPS 数据路径。
2. “设备已连接”“数据流合法”“GPS 解算可用于导航”必须是三个不同状态，不能以一次 detected 日志互相代替。
3. `SYS_STATUS`、`HIGHRES_IMU`、`GPS_RAW_INT` 和诊断 uORB topic 必须投影同一份健康结论。
4. 诊断日志只在进入错误状态的边沿报告一次，不重新引入高频刷屏。
5. 保留源代码/主机测试/Windows 构建与真实硬件证明之间的边界。

本规格中的“合法”不是密码学真实性。它能发现结构损坏、时间异常、冻结、错误密度、物理字段矛盾和 PX4 GNSS 质量不足；在 UM982 未提供可信 spoofing/authentication 状态时，不能声称已证明卫星信号未被欺骗。

## 2. 已确认的当前基础

### 2.1 IMU 已有能力

- ICM42688P WHOAMI、软复位完成位、24 项寄存器配置与运行期轮询验证。
- FIFO full/empty/header、20-bit invalid sentinel、DMA timeout、温度批内一致性和 publication failure 计数。
- `VehicleImu` 已检查 device ID、有限向量、时间倒退/大间隔、校准 ID、校准 scale 和板旋转。
- clipping、error_count、FIFO watchdog、积分与 coning correction 已存在。

### 2.2 GPS 已有能力

- NMEA XOR、Unicore CRC32、帧长度、字段结构和基本数值解析。
- GGA/RMC/AGRICA/UNIHEADINGA、波特率扫描、COM 探测和配置重试。
- 接收机在线但没有 GGA 时能发布位置未知的 `NO_FIX`，QGC 能收到 `GPS_RAW_INT`。
- 当前尚未统一检查坐标范围、DOP/accuracy 符号、fix/坐标/卫星一致性、速度/航向一致性和持续健康时间。

## 3. 三层合法性模型

| 层级 | 含义 | GPS | IMU |
|---|---|---|---|
| Present | 设备身份和传输链路已建立 | 有 CRC 正确的受支持 UM982 数据帧 | WHOAMI 正确且原始 accel/gyro topic 已发布 |
| Stream valid | 最新数据在时间、结构、有限值和错误密度上合法 | 合法 receiver/NO_FIX 或合法解算样本 | accel/gyro 均通过 validator、校准合同和同步积分条件 |
| Solution eligible | 数据质量足以供导航使用 | PX4 GNSS checks 持续通过 | 当前无 EKF；等同于合法、校准后的 `vehicle_imu` 可发布 |

状态投影：

- `SYS_STATUS present/enabled`：由 Present 决定。
- `SYS_STATUS health`：由 Stream valid 决定；GPS `NO_FIX` 可以是健康接收机，但不是可用于导航的解算。
- `GPS_RAW_INT.fix_type`：保留接收机真实 fix；完全失联才发送 `NO_GPS`。
- `HIGHRES_IMU`：只使用合法、校准后的 `vehicle_imu`。
- 未来导航消费者必须同时要求 GPS Solution eligible，不能只检查 topic 新鲜度。

## 4. GPS 合法性合同

### 4.1 协议/字段结构检查

在 UM982 protocol/driver 边界完成，失败帧不更新测量缓存：

- checksum/CRC、最大帧长、完整字段数和完整数值解析必须通过。
- 纬度只能使用 `N/S`，经度只能使用 `E/W`；转换后纬度位于 `[-90, 90]`，经度位于 `[-180, 180]`。
- `quality=0` 或 RMC `V` 允许位置字段为空，并标准化为 `NO_FIX + NaN`。
- 声称 fix 的 GGA 必须同时具有有限坐标、有限高度、非负 HDOP 和至少 1 颗卫星。
- GSA fix dimension 只接受 1/2/3；PDOP/HDOP/VDOP 必须有限、非负，协议声明有解时不能为零值占位。
- GST RMS/纬度/经度/高度标准差必须有限且非负。
- RMC `A` 的速度必须有限且非负，course 必须位于 `[0, 360]`，日期/UTC 必须可转换。
- AGRICA 的 N/E/U 速度、合速度和标准差必须有限；标准差不得为负；GPS 毫秒必须小于一周毫秒数。
- UNIHEADINGA 只有 `SOL_COMPUTED`、baseline `> 0`、heading `[0, 360]`、accuracy `(0, 360]` 时才可进入 heading 输出。
- 不设置未经 UM982 明确报告的 jamming/spoofing/authentication 结论，保持 `UNKNOWN`。

### 4.2 流诊断

复用 PX4 `DataValidator` 的以下策略，并以无堆、固定状态实现：

- No data：从未收到受支持且结构合法的数据。
- Timeout：最后合法接收机数据超过当前 1.3 s 驱动合同。
- High error density：最终样本范围、时间戳或 UART error/drop 持续增加时使用 PX4 100-event 衰减窗口；与固定 PX4-GPSDrivers 一致，可恢复的 NMEA/Unicore checksum、报文结构、未知消息和 overflow 静默丢弃，由持续无有效数据的 timeout 接管，不进入该窗口。
- Timestamp regression：接收时间或有效 UTC 在同一连续会话中倒退；重新探测/时钟重置会清状态。
- GPS 不启用“完全相等 100 次即冻结”的位置判定，因为静止 RTK 可以合法地重复同一坐标；冻结由接收帧时间、错误密度和消息序列共同判断。

### 4.3 PX4 GNSS 解算检查

采用 PX4 v1.17.0 `GnssChecks` 初始门槛及默认值，不在没有 EKF 的项目中伪造 `EKF2_*` 参数：

- fix type `>= 3`。
- satellites `>= 6`。
- `PDOP <= 2.5`；由 `sqrt(HDOP^2 + VDOP^2)` 得到，缺失则检查失败。
- `EPH <= 3.0 m`。
- `EPV <= 5.0 m`。
- speed accuracy `<= 0.5 m/s`。
- 明确 `SPOOFING_STATE_DETECTED` 或 `AUTHENTICATION_STATE_ERROR` 时失败；`UNKNOWN` 只表示能力未知，不能当作“已证明安全”。
- 初次必须连续通过 10 s 才把 `checks_passed` 置 true。
- 初次通过后采用 PX4 simplified checks：fix、spoofing、`hacc/vacc <= 50 m`、`sacc <= 10 m/s`；失败后至少连续 1 s 恢复才重新通过。

以下 PX4 检查本期不实现并明确报告为 unsupported：静止地面 horizontal/vertical drift、horizontal/vertical speed offset。它们依赖 EKF 的 `vehicle_at_rest/in_air` 状态，本项目目前没有同等可信来源，不能凭自定义 IMU 阈值伪造。

### 4.4 GPS 输出

- 保留 PX4 `sensor_gps`/`vehicle_gps_position` 原始解算语义；结构非法样本不发布。
- 增加 PX4 `EstimatorGpsStatus.msg` 对应的 `estimator_gps_status` topic，发布 fix/nsats/pdop/hacc/vacc/sacc/spoofing fail bits、`checks_passed` 和时间戳。
- drift/speed-offset fail bits固定为 false，但在文档中标明 unsupported，不能称为已验证。
- `GPS_RAW_INT` 继续展示真实 fix/卫星/精度，使 QGC 即使在 checks 未通过时仍可诊断接收机。

## 5. IMU 合法性合同

### 5.1 驱动层

保留当前 PX4 ICM42688P 状态机并补齐显式原因计数：

- WHOAMI、reset-done、寄存器 set/clear mask、运行期 register check。
- FIFO full/empty/header/ODR/timestamp、invalid 20-bit sentinel、批内温度一致性。
- DMA start/complete/timeout、SPI error、INT1/INT2、publication failure。
- 允许与 PX4 相同的“前部合法样本”部分批处理；一旦出现非法 packet，错误计数增加，后续 packet 不进入该批。
- 产生的 `sensor_accel`/`sensor_gyro` 必须具有固定 device ID、非零且不晚于 publication timestamp 的 sample timestamp、1..watermark 的 sample count、有限温度和有限向量。

### 5.2 VehicleImu 流诊断

引入无动态分配的 PX4 DataValidator 等价状态，每个 accel/gyro 各一份：

- No data、200 ms runtime timeout、完全相等值冻结、累计 error count、100-event error-density 衰减。
- device ID 不匹配、非有限值、sample timestamp 为零/倒退/晚于 publication timestamp均拒绝。
- 时间间隔超过 20 ms 时清积分器；只有新 accel/gyro 都重新 prime 并形成合法积分窗口后才能恢复 `vehicle_imu`。
- corrected accel/gyro 和最终 delta 向量必须有限；积分 dt 必须非零并满足现有 integration-ready 合同。
- clipping 不直接把样本判非法，沿用 PX4：累计 warning/diagnostic，并在持续 clipping 时报告核心错误。
- raw sensor topic 始终保留给校准流程；CAL ID 为零、与固定 ICM42688P device ID 失配或保存值无效时使用 offset=0/scale=1 的 identity correction，仍发布合法 `vehicle_imu`，但绝不套用其他设备的校准。
- 校准模块确认对应 `parameter_update.instance` 已被 `VehicleImu` 或 `VehicleMagnetometer` 应用，并精确核对 active correction 的 ID/offset/scale；提交和回滚完成前保持 arming interlock，不使用普通数据更新冒充参数应用确认。

### 5.3 PX4 IMU 状态输出

增加与 PX4 v1.17.0 `VehicleImuStatus.msg` 一致的 `vehicle_imu_status` topic：

- accel/gyro device ID、error count、rate/raw rate。
- 三轴累计 clipping。
- accel/gyro vibration metric、delta angle coning metric。
- mean/variance、accel/gyro temperature。
- 每 1 s 发布一次，error/clipping 状态变化可提前发布。

本期只有一个 IMU，不增加 `SensorsStatusImu` 多实例一致性投票；PX4 明确说明单 IMU 的 inconsistency 值为零，因此伪造该检查没有诊断价值。

## 6. 日志与健康状态机

- 每个 GPS/IMU 错误类别只在“健康 -> 非健康”或错误类别改变时报告；GPS 最终样本/流错误摘要额外限制为至少 30 秒一次。
- 恢复由 topic/`SYS_STATUS` 表达，不增加周期性 recovered/healthy 文本。
- GPS 保留核心错误：offline、configuration failure、最终样本或流的 validation reason mask；协议 checksum、报文结构、未知消息和 overflow 不发送 `STATUSTEXT`。
- IMU 保留核心错误：probe/register/FIFO/DMA/stream/calibration reason mask。
- 禁止 baud probing、detected、每样本 invalid、每秒 timeout/restart 统计刷屏；详细计数进入 status topic 和内部 Stats。

## 7. 解锁与降级边界

推荐策略（待用户确认）：

- 不改变当前只支持 Manual 的解锁合同。
- GPS/IMU 非健康不新增 forced-disarm，也不阻止纯手动 Rover 解锁。
- 但非法数据不发布为健康 `vehicle_imu`，GPS `checks_passed=false`；任何未来依赖姿态/GPS 的导航模式必须将对应健康状态加入 mode-specific arming gate。
- 校准期间、参数维护期间和现有 RC/参数/执行器安全门禁保持不变。

可选严格策略（需要用户明确授权）：

- IMU present + stream valid + calibrated 成为所有模式的解锁必要条件。
- GPS 只应成为需要位置导航模式的必要条件；若当前 Manual 也强制 GPS，会导致室内/无定位时无法手动驾驶，不推荐。

## 8. 工程结构

计划新增/修改的所有权：

- `Dima/lib/sensors/validation/`：PX4 DataValidator 等价算法和 GPS/IMU 纯函数合同。
- `Dima/drivers/gps/um982/`：UM982 协议字段、接收流合法性与自动配置链路。
- `Dima/drivers/imu/icm42688p/`：驱动原因计数与原始样本出口检查。
- `Dima/modules/sensors/imu/`：VehicleImu validator、校准门槛和 `vehicle_imu_status`。
- `Dima/messages/schemas/`：固定 PX4 status message 合同。
- `Dima/modules/mavlink/`：以诊断状态投影 `SYS_STATUS/HIGHRES_IMU/GPS_RAW_INT`。
- `tools/`：只更新现有架构/日志门禁，不新增测试文件。

不新增测试文件，不引入 GoogleTest/CMake 或新的第三方依赖；生产算法通过现有架构门禁、协议回归（若已有）、固件对象编译和全量 Windows 原生构建验收。

## 9. 代码风格

纯算法返回显式结果和原因位，不在纯函数中写日志：

```cpp
const validation::GpsResult result = validation::validate_gps(sample);
if (!result.structurally_valid) {
    diagnostics_.reject(result.failure_mask);
    return;
}
publish_normalized_sample(sample);
```

- C++17、无异常、无 RTTI、无运行期堆分配。
- 固定宽度整数、饱和计数、时钟倒退显式处理。
- 阈值集中定义并带固定上游路径/commit 注释。
- driver 不拥有 MAVLink；MAVLink 不重复解析传感器合法性。

## 10. 验证策略

### 10.1 生产代码边界矩阵

不新增独立测试源文件；实现和审查至少覆盖以下边界，并由生产翻译单元编译、现有协议回归、架构门禁与板端故障注入共同验证：

- DataValidator：no data、timeout、stale、error-density、recovery、counter saturation、clock rollback。
- GPS：无 fix 空坐标合法；有 fix 空/越界坐标非法；负 DOP/stddev 非法；fix/satellite 矛盾；速度/course/heading 范围；10 s initial dwell；1 s recovery；spoof/auth failure。
- IMU：device ID、finite、timestamp order、gap reset、error density、stuck values、calibration ID/scale、clipping 不等于 invalid、合法恢复必须重新 prime。

### 10.2 静态/生成/固件验收

正式 Windows PowerShell 命令：

```powershell
Set-Location -LiteralPath E:\freertos\H743_FreeRTOS
E:\toolchains\msys64\ucrt64\bin\python.exe tools\check_architecture.py
E:\toolchains\msys64\usr\bin\make.exe NO_COLOR=1 clean
E:\toolchains\msys64\usr\bin\make.exe -j4 NO_COLOR=1 dima_rover
E:\toolchains\msys64\usr\bin\make.exe -j4 NO_COLOR=1 verify
```

并执行 Windows Git `git diff --check`、ELF `nm -u`、status topic 符号和最终字符串审计。

### 10.3 板端验收

源码/构建完成后仍保留 `BOARD PENDING`，烧录后至少验证：

- 正常静止/运动时 GPS/IMU status topic、`GPS_RAW_INT`、`HIGHRES_IMU`、`SYS_STATUS` 一致。
- 拔掉 GPS、破坏 checksum、NO_FIX、低卫星、差精度、恢复 10 s/1 s 状态。
- IMU 中断中止、DMA timeout、FIFO overflow、寄存器漂移、冻结数据、clipping、未校准/重新校准。
- 每个错误状态只产生一次核心日志，恢复后能再次检测下一次独立故障。

## 11. 实施任务

- [x] Task 1：PX4 DataValidator 与纯合法性算法
  - Acceptance：GPS/IMU failure mask、hysteresis 和饱和计数具有确定性、无堆实现并被生产消费者实际编译使用。
  - Verify：生产翻译单元、架构门禁与全量固件构建通过。
  - Files：validation 生产算法 2~3 个文件，不新增测试文件。

- [x] Task 2：GPS protocol/stream/solution checks
  - Acceptance：结构非法样本不更新缓存；`estimator_gps_status` 精确反映 PX4 子集和 unsupported 位。
  - Verify：GPS 边界矩阵、协议回归、架构门禁。
  - Files：UM982 protocol、driver、GPS status schema/ABI、README 分两次完成，每次不超过 5 文件。

- [x] Task 3：IMU validator 与 `vehicle_imu_status`
  - Acceptance：非法 raw sample 不进入 `vehicle_imu`；identity correction 保持 present/streamable，health 由数据合法性与新鲜度决定；状态字段与 PX4 合同一致。
  - Verify：IMU 边界矩阵、状态统计、固件对象编译。
  - Files：VehicleImu、ICM driver、status schema/ABI、README 分两次完成。

- [x] Task 4：MAVLink/安全投影和防回归门禁
  - Acceptance：present/health/solution 三层不混淆；Manual 解锁策略按用户选择执行；日志保持边沿去重。
  - Verify：架构检查、ELF 字符串/符号、全量 build/verify。
  - Files：MavlinkSensorStreams、MavlinkService、Commander（仅严格策略）、architecture gate、文档。

## 12. 边界

Always：

- 使用固定 PX4 v1.17.0/PX4-GPSDrivers commit 和 BSD 来源声明。
- 不新增测试文件；使用现有回归、架构门禁、生产对象编译和全量构建逐层验收。
- 保留原始 topic 给校准和诊断，非法数据不得冒充健康校准后输出。
- 使用 Windows 原生路径、Git、Python、GNU Make 和 Arm 工具链验收。

Ask first：

- 改变 Manual 解锁/forced-disarm 条件。
- 新增 EKF2 参数或把诊断阈值暴露为用户参数。
- 添加第三方测试依赖或修改 QGC 本体。

Never：

- 以单 IMU 声称完成多 IMU 一致性投票。
- 在无 EKF at-rest 状态时声称完成 GPS drift/speed-offset checks。
- 将 `SPOOFING_STATE_UNKNOWN` 表述为已通过反欺骗认证。
- 用 Windows 构建替代 QGC/板端/电气/车辆验证。

## 13. 成功标准

1. 每个发布到健康 `vehicle_imu` 的样本都通过 identity、calibration、finite、timestamp、gap 和 stream validator。
2. 每个结构非法 GPS 样本被拒绝且原因可计数；合法 NO_FIX 仍保持 QGC 图标。
3. GPS `checks_passed` 完成 PX4 fix/nsats/pdop/eph/epv/sacc/spoof + 10 s/1 s hysteresis 子集。
4. `vehicle_imu_status` 和 `estimator_gps_status` 可实时订阅，`SYS_STATUS` 与其状态一致。
5. 高频/周期性 GPS/IMU 诊断日志不进入最终 ELF；核心错误只在状态边沿出现。
6. 现有 architecture、协议回归（若适用）、Metadata、clean build、verify、ELF 生命周期和未解析符号全部通过。
7. 最终报告明确保留未完成的 EKF drift、多 IMU voting、anti-spoof 和板端验证边界。

## 14. 已确认决策

用户确认保持当前策略：不改变 Manual 解锁、forced-disarm 或 BootHealth 合同。GPS/IMU 健康状态约束数据发布、MAVLink 健康投影和未来导航消费者；若以后增加需要位置或姿态的模式，再为该模式增加对应的 mode-specific arming gate。用户同时明确要求不新增测试文件，验证改用仓库现有门禁、生产固件编译和板端验收清单。
