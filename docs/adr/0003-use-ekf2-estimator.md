# ADR-0003：采用 PX4 EKF2 作为最终状态估计器

- 状态：Accepted
- 日期：2026-07-29
- 决策者：Dima Rover 项目

## 背景

项目曾讨论采用 ArduPilot EKF3，但最终架构已经确定以 PX4 v1.17.0 作为主要直接代码来源，并希望减少两套飞控平台基础设施同时进入产品所造成的参数、数学库、数据访问层和许可证复杂度。

状态估计器必须服务差速 Rover 的姿态、Yaw/Yaw Rate、GNSS/RTK 速度和位置，并在后续接入轮速、Position、Waypoint 和 PivotTurn。Estimator 必须通过消息输出状态，不得直接驱动控制器或 PWM。

## 决策

最终采用 PX4 EKF2，不再规划 ArduPilot EKF3。本 ADR 和总体计划取代此前所有 EKF3 阶段性方案。

正式移植基线为 PX4 v1.17.0；当前本地 `release/1.16`、commit `75f9a32a12` 只用于预研，正式导入前必须取得并记录 v1.17.0 的准确 tag/commit。

首版范围：

- PX4 EKF2 module 和 EKF core。
- 多 EKF 实例、传感器实例绑定和 Estimator Selector 所需接口。
- EKF2 参数、GSF Yaw、延迟观测缓冲、创新检查和状态输出。
- IMU、GNSS/RTK 和 Magnetometer。
- 硬件具备时接入 Air Data 和 Distance Sensor。
- 首版即保留多 IMU/Mag 组合、多个 EKF 实例和 Estimator Selector；实际激活实例数由可用传感器和参数决定。
- Visual Odometry 后置。
- Wheel Encoder 通过独立 Dima Odometry Adapter 转换为可审查的观测输入，不直接修改 EKF Core。

首版输出至少包括：

```text
vehicle_attitude
vehicle_local_position
vehicle_global_position
estimator_status
estimator_innovations
estimator_sensor_bias
estimator_reset_status
```

运行约束：

- Estimator 由 IMU 数据到达驱动。
- 允许在启动阶段分配 EKF 实例和观测缓冲。
- EKF2 更新周期内禁止动态分配。
- 控制器只消费 Estimator Topic，不持有或调用 EKF 内部对象。
- Estimator 异常通过统一状态和 Failsafe 链处理。

## 备选方案

### ArduPilot EKF3

EKF3 是成熟方案，并且适合 Rover，但会引入 AP_DAL、AP_Param、AP_Math 和 GPL 许可边界。由于项目主要直接代码来源已经固定为 PX4，为减少双平台耦合，最终不采用。

### 自研二维 Rover EKF

资源需求可能更小，但会重新承担建模、融合、延迟补偿、创新检查、Bias、Reset 和长期实车验证，违背优先复用成熟模块的目标，因此不采用。

### 不使用融合 Estimator

不能满足后续 Heading、Position、RTK 航点和完整安全检查需求，因此不采用。

## 影响

### 正面影响

- 与 PX4 Parameter、uORB、WorkQueue 和 Rover 模块链的数据契约一致。
- 减少 AP_DAL、AP_Param 和另一套数学基础设施的移植工作。
- 便于后续接入 Position、Waypoint、状态健康和统一 Failsafe。

### 代价与风险

- EKF2 仍是大型模块，需要严格控制 Flash、RAM、延迟缓冲和任务栈。
- 差速轮速观测需要独立适配并评审打滑、原地旋转和协方差。
- 正式 v1.17.0 源码尚未在本地固定，阶段 0 不能把 1.16 预研结果当作最终验证。

## 实施约束

- 阶段 0 不导入 EKF2 生产源码。
- 正式导入文件必须登记到 Source Manifest 并保留原始许可证头。
- 首版不得实现成单实例专用架构；至少支持两个 EKF 实例和 Estimator Selector。
- 若早期硬件暂时只提供一组有效 IMU/Mag，可以只激活一个实例，但消息、参数、调度和资源模型仍按多实例保留。
- EKF2 不得直接访问 PWM 或改变 Arming 状态。
- 许可证最终状态暂为 `PENDING`，对外发布受 Source Manifest 中的限制约束。

## 验证方式

- 阶段 0 核对计划和 ADR 中不存在仍生效的 EKF3 路线。
- 后续使用目标编译、链接资源、板上传感器状态、创新量和实车状态连续性验证。
- 不为本 ADR 新增测试或仿真代码。

## 相关记录

- 总体计划：`../DIMA_ROVER_PORTING_PLAN_ZH.md`
- 来源清单：`../DIMA_SOURCE_MANIFEST.md`
