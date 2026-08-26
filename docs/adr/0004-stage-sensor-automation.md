# ADR-0004：当前阶段的传感器自动化参数边界

- 状态：Accepted
- 日期：2026-08-24
- 决策者：Dima Rover 项目

## 目标

在不引入空参数、不复制一套与既定 EKF2 路线冲突的估计器，并保持单 IMU、单外置 DroneCAN 磁力计产品边界的前提下，对齐 PX4 v1.17.0 的传感器参数语义。成功标准是每个公开参数都有实际消费者、异常路径和可验证输出。

## 固定参考与假设

- PX4-Autopilot v1.17.0：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- 当前只有 ICM42688P accel/gyro 和一颗外置 DroneCAN RM3100，没有内部参考磁力计。
- 当前阶段没有 `estimator_sensor_bias`、`magnetometer_bias_estimate` 或等价的状态估计器输出；ADR-0003 已决定后续由 PX4 EKF2 提供这些接口。
- `SENS_MAG_RATE` 按 PX4 定义为本机磁力计数据发布上限，不等同于远端 DroneCAN 节点的硬件采样率。
- 远端采样率只有在已知节点固件、参数名、类型、范围和持久化语义后，才能通过 DroneCAN `uavcan.protocol.param.GetSet` 安全配置。

## 能力与参数决策

| PX4 参数/功能 | PX4 的真实依赖 | 当前阶段决策 |
|---|---|---|
| `IMU_INTEG_RATE` | `VehicleIMU` 积分器 | 实现 100/200/250/400 Hz；按 PX4 在 Disarmed 前端动态应用 |
| `SENS_IMU_CLPNOTI` | 已有 accel/gyro clipping 计数 | 可实现；限频告警，不改变数据 |
| `SENS_IMU_AUTOCAL` | 稳定且带方差的 `estimator_sensor_bias` | 暂不公开；保留现有手动静止 gyro 和六面 accel 校准，待 EKF2 接口落地后按 PX4 实现 |
| `SENS_MAG_RATE` | `vehicle_magnetometer` 本机发布限频/平均 | 可实现 1..200 Hz；`sensor_mag` 原始流保持全速供校准与检测使用 |
| `SENS_MAG_AUTOROT` | 至少三面数据和第一颗内部磁力计参考 | 暂不公开；单颗外置 RM3100 无法运行 PX4 的参考磁力计 MSE 算法，继续使用 `CAL_MAG0_ROT` |
| `SENS_MAG_AUTOCAL` | `magnetometer_bias_estimate` 或稳定的 estimator mag bias | 暂不公开；保留手动旋转磁校准，待估计器 bias 接口落地 |
| `SENS_MAG_MODE` | 多磁力计 voter/selector | 不公开；当前只有一个磁力计实例 |
| `SENS_MAG_SIDES` | PX4 多面磁校准流程 | 不公开；当前产品固定执行全部六面，每面 40 个经空间间距去重的点、总计 240 点，不消费可裁剪的 side bitmask |
| 远端 RM3100 采样率 | 节点特定 DroneCAN 参数契约 | 阻塞，等待节点固件及参数契约；不得把本机丢样本伪装成远端配置成功 |

## 实现计划

1. 修复 SPI4 时钟源查询和 ICM42688P 三次 WHOAMI probe，输出请求、内核和实际 SPI 时钟。
2. 为校准参数补齐 PX4 `System` category、`volatile`、小数位和 scale 范围；新增有实际消费者的 `IMU_INTEG_RATE`。
3. 新增 `SENS_IMU_CLPNOTI`，从现有 clip counter 产生限频日志；默认启用但绝不改变积分数据。
4. 新增 `SENS_MAG_RATE`，由 DroneCAN 驱动持续发布全速 `sensor_mag`，独立 `VehicleMagnetometer` 只对校准后的 `vehicle_magnetometer` 做时间戳驱动的上限控制；参数变化按 PX4 在 Disarmed 前端应用。
5. DroneCAN 未检测日志同时给出配置节点、总 CAN RX、接受/解码/协议计数、overrun、bus-off 和最后错误标志；检测成功日志给出 node、sensor、device ID。
6. 不添加无消费者的自动偏置、自动旋转或远端配置参数；对应依赖到位时用新的 ADR 扩展。

## 工程结构

- 参数定义：`Dima/middleware/parameters/definitions/sensor_params.c`
- IMU 参数消费者：`Dima/modules/sensors/imu/VehicleImu.*`
- 磁力计校准/限频消费者：`Dima/modules/sensors/magnetometer/VehicleMagnetometer.*`；只发布原始 `sensor_mag` 的 DroneCAN 设备链位于 `Dima/drivers/magnetometer/dronecan_mag2/`
- 校准事务：`Dima/modules/sensors/calibration/`
- 板级验收：SPI/CAN/UART、Topic 频率与校准结果均在实际 H743 硬件上确认
- 架构门禁：`tools/architecture/`

## 代码约定

参数必须先验证再进入活动配置。硬件 transport 变化进入安全维护事务；纯 IMU/磁力计前端 correction 按 PX4 仅在 Disarmed 应用。校准事务已经持有全局 arming interlock，消费者不得再次获取同一个不可重入互锁；提交和回滚必须等对应前端确认同一 `parameter_update.instance` 并逐项匹配 active correction 后，才能结束事务和释放互锁。PX4/QGC `[cal]` 协议需要格式化 STATUSTEXT，因此校准事务固定运行于非实时 `wq:lp_default`，不得进入拒绝格式化的 `wq:sensors`：

```cpp
const std::uint32_t interval_us = validated_interval(parameter_value);
if (interval_us == 0U) {
    return ConfigurationReadResult::Invalid;
}
pending_configuration_.interval_us = interval_us;
```

不允许参数只存在于 Metadata 而没有运行时消费者，也不允许用本机限频日志宣称远端节点已经改变采样率。

## 验证命令

全部从 Windows PowerShell 7 进入 `E:\freertos\H743_FreeRTOS` 执行：

```powershell
python.exe tools/check_architecture.py
cmd.exe /d /s /c "cd /d E:\freertos\H743_FreeRTOS && make NO_COLOR=1 parameter-metadata-verify"
cmd.exe /d /s /c "cd /d E:\freertos\H743_FreeRTOS && make NO_COLOR=1 clean"
cmd.exe /d /s /c "cd /d E:\freertos\H743_FreeRTOS && make -j4 NO_COLOR=1 dima_rover"
```

本阶段不保留额外 Host 单元测试或本机 runner。上述静态门禁与完整构建只证明源码、生成物和链接；WHOAMI、CAN 收包、RM3100 发布、QGC 参数排版和实车校准必须以实际板端验收为准，在完成前继续标为 `BOARD PENDING`。

## 边界

- 始终执行：参数必须有消费者；保留无关脏改动；区分静态/构建与实板证据；更新参数计数和 Metadata CRC。
- 需要确认：引入非 PX4 的 gyro-reference 磁力计自动旋转算法；加入节点特定 DroneCAN 参数客户端；改变自动校准默认值。
- 禁止执行：伪造 EKF bias；在 Armed 状态写校准；机械复制不支持的 PX4 参数；用本机限频冒充远端配置。

## 后续输入

1. 远端 RM3100 节点使用哪一个固件、版本和参数名来设置采样率？
