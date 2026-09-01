# ADR-0003：采用 PX4 EKF2 单实例状态估计器

- 状态：Accepted
- 日期：2026-07-29
- 最近修订：2026-08-31
- 决策者：Dima Rover 项目

## 背景

项目采用 PX4 v1.17.0 作为主要直接代码来源。状态估计器需要为差速 Rover 提供姿态、航向、GNSS/RTK 速度与位置，并通过 uORB 输出服务后续 Position、Waypoint 和控制模块；Estimator 不得直接驱动控制器、Arming 或 PWM。

早期 ADR 曾预留多实例、Estimator Selector、Air Data、Distance Sensor 和 Wheel Adapter。N1 产品只有一套 ICM-42688-P、外置 DroneCAN Magnetometer 和一套 UM982，保留这些未实现分支会增加参数、消息、静态内存和失效模式，却没有第二套硬件可供切换，因此本修订明确替代旧的多实例范围。

## 决策

固定采用 PX4-Autopilot tag `v1.17.0`、commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 EKF2 module、EKF Core 和必要支撑库，不采用 ArduPilot EKF3 或自研二维 EKF。

N1 是永久启用的单实例实现：

- 唯一实例固定订阅 `vehicle_imu[0]`、`vehicle_magnetometer[0]` 和 UM982 `vehicle_gps_position[0]`。
- 固定运行于 `wq:estimator`，priority 8、静态栈 8 KiB，由 IMU callback 驱动并保留 100 ms backup schedule。
- 不实现 Estimator Selector、多实例参数、`EKF2_EN`、运行时装卸或动态实例重试。
- ApplicationContext 在 IMU、Mag、GNSS 生产者完成启动尝试后启动 EKF2；失败只使自动导航观测降级，Manual、Commander、BootHealth、IWDG 和 PWM 安全链继续独立工作。

实际融合闭包包括：

- IMU 预测、输出 predictor、Accel/Gyro bias。
- GNSS 位置、高度、NED 速度、双天线 yaw 和 EKF-GSF emergency yaw。
- Magnetometer Automatic、地磁状态和 Gravity fusion。
- 创新检查、状态/事件、aid source、reset、sensor bias、GPS checks 和时间戳输出。

永久排除 Wind、Airspeed、Barometer、Optical Flow、Range/Terrain fusion、External Vision、Sideslip、Drag、Aux velocity/global position、Wheel odometry、groundtruth 和 Selector。Rover 不从 wheel 或虚构 land detector 推导静止；只有新鲜且 Disarmed 的 `vehicle_status` 才发送 `at_rest/constant_pos=true`，其余状态保守视为未知运动。

## 公式生成与构建

仓库只保留一份 PX4 `derivation.py` 和一份 `generated/` 公式目录。N1 状态向量在该生成脚本中直接不定义 Wind 和 Terrain，且不注册任何 Wind 生成函数；不得手改生成头，也不通过命令行维护其他状态闭包产物。当前唯一公式闭包由同一脚本生成 13 个头文件：存储状态为 22 个 float，误差状态和协方差为 21 维。

EKF 源继承项目统一的 `-fno-exceptions/-fno-rtti`，并保留 PX4 公式所需的 `-fno-associative-math`。生成的 `StateSample::vector()` 沿用 PX4 的 strict-aliasing 构建合同，因此 EKF 类型闭包使用 `-fno-strict-aliasing`。

延迟 RingBuffer 只允许在模块启动期按 `EKF2_DELAY_MAX / EKF2_PREDICT_US` 一次性分配；这两个容量参数均要求重启生效，运行期保持启动快照。IMU/GNSS/Mag/system flag 热路径禁止 `new/delete`；100 ms 备份调度失败时撤销 IMU callback 并进入 Error，不能留下假 Running 实例。

## 参数与航向合同

EKF2 参数只来自 `Dima/middleware/parameters/definitions/module_ekf2*.yaml` 并通过正式参数工具生成。Rover 固定 `is_fixed_wing=false`，不会进入 GSF 的固定翼向心加速度补偿，因此删除无运行消费者的 `EKF2_GSF_TAS`。`EKF2_GPS_CTRL` 保持 PX4 默认值 7；启用 GNSS yaw 时由用户显式设置 bit 3，即值 15。需要重启才能安全应用的延迟缓冲、初始化和融合模式字段在权威 YAML 中标记 `reboot_required: true`，运行期刷新不得半应用。

UM982 的 `GPS_YAW_OFFSET` 是双天线阵列方向到车体前向的顺时针安装角，单位 deg。驱动发布：

```text
heading = wrap_pi(raw_heading + pi - GPS_YAW_OFFSET)
heading_offset = GPS_YAW_OFFSET
```

EKF Wrapper 将 `heading` 与 `heading_offset` 原样送入 PX4 Core；预测使用二者恢复阵列观测，reset 使用已补偿 body yaw，任何层都不得再次扣除偏置。N1 不引入 PX4 的 `EKF2_GPS_YAW_OFF` 第二偏置参数。

`SENS_IMU_AUTOCAL` 默认启用。EKF2 仅在方差最大值小于 `1e-3`、最大/最小方差比小于 100、bias 变化不超过 limit 的 10%，且对应 alignment/fault/clipping/fusion 条件连续成立超过 10 s 后标记 stable。VehicleImu 只接受新鲜、device ID 匹配且 `valid && stable` 的 bias，按 PX4 的 `R^T*bias` 公式转换回传感器轴，并只在 Disarmed 后由 `wq:sensors` 发布固定请求快照、由非实时 `wq:lp_default` 原子更新完整 `CAL_ACC0_*`/`CAL_GYRO0_*` 参数组。EKF2 的磁偏角保存采用同一边界：estimator 只发布 float 快照，非实时提交器更新 `EKF2_MAG_DECL`；失败分别限速重试，物理保存继续由既有 autosave 负责。

## 输出与所有权

单实例至少发布：

```text
vehicle_attitude
vehicle_local_position
vehicle_global_position
vehicle_odometry
estimator_status
estimator_status_flags
estimator_event_flags
estimator_sensor_bias
estimator_gps_status
yaw_estimator_status
ekf2_timestamps
estimator_aid_src_{gnss,mag,gravity,fake}_*
```

EKF2 是 `estimator_gps_status` 唯一发布者；UM982 只拥有接收机、协议、原始流与在线状态，并继续发布 `sensor_gps`/`vehicle_gps_position`。MAVLink 通过权威 `mavlink_runtime.yaml` 生成运行合同，输出 `ATTITUDE` 50 Hz、`LOCAL_POSITION_NED` 30 Hz、`GLOBAL_POSITION_INT` 10 Hz 和 `ESTIMATOR_STATUS` 5 Hz；消息 ID、CRC、payload、codec 和路由表不得手写。

## 影响

正面影响是状态、参数、uORB、WorkQueue 和 MAVLink 语义保持在同一 PX4 基线内，产品闭包与单套硬件一致，并消除未实现 Selector/Wheel/Wind 分支的资源和安全歧义。代价是当前没有传感器冗余切换能力；未来若产品硬件真的增加冗余或 wheel 观测，必须新建 ADR、重新审查状态模型、参数、内存和失效行为，不能在本实现中预埋空分支。

## 验证方式

- 正式 Windows 参数/uORB/MAVLink 生成、架构门禁、Arm GCC clean build、ELF layout 与未解析符号检查。
- 静态确认唯一 `derivation.py`/generated 路径、无 Wind 生成函数、无 Selector/Wheel/禁用观测源残留发布者。
- 板端另行验证 IMU/GNSS/Mag 输入、GNSS yaw 安装角、bias 写回、QGC 四条导航流和车辆状态连续性；Windows 构建不替代这些实测。
- 本阶段不新增或修改测试、Host Test、SITL 或仿真代码。

## 相关记录

- 总体计划：`../DIMA_ROVER_PORTING_PLAN_ZH.md`
- 来源清单：`../DIMA_SOURCE_MANIFEST.md`
