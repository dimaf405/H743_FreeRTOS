# EKF2 单实例模块

本目录是 PX4 v1.17.0 EKF Core 到 Dima Rover uORB/生命周期的薄适配。产品只构造一个 `Ekf2`，固定绑定 `vehicle_imu[0]`、`vehicle_magnetometer[0]` 和 `vehicle_gps_position[0]`，运行于 `wq:estimator`（priority 8、8 KiB）。没有 Selector、多实例、`EKF2_EN` 或运行时装卸入口。

## 生命周期与实时边界

- `ApplicationContext` 在 IMU、Mag、UM982 前端完成启动尝试后永久启动一次 EKF2；失败只记录自动导航观测降级，不门控 Manual、Commander、BootHealth、IWDG 或 PWM。
- 启动期先加载完整参数候选、按 `EKF2_DELAY_MAX / EKF2_PREDICT_US` 分配全部 RingBuffer，再 advertise Topic 和注册 IMU callback。两项容量参数均标记 `reboot_required` 并在运行期保持启动快照；任一步失败都不进入运行态。
- IMU callback 是主触发，100 ms schedule 仅作丢回调备份；更新热路径不分配和释放内存。运行期若备份调度失败，模块撤销 callback 并进入 Error，不能继续报告一个可能永久静默的 Running 实例。
- shutdown 先注销 callback 并 drain estimator queue，再由组合根停止 GPS/Mag/IMU 生产者。

## 输入与航向

- IMU delta angle/delta velocity 保持 FRD/SI 单位，并把 clipping 逐轴传给 Core。
- UM982 发布 `heading=wrap_pi(raw+pi-GPS_YAW_OFFSET)` 和同一安装 `heading_offset`；Wrapper 原样写入 `gnssSample.yaw/yaw_offset`，不再次扣偏置。
- Rover 永远发送 `in_air=false`、`is_fixed_wing=false`。只有新鲜且 Disarmed 的 `vehicle_status` 才发送 `at_rest/constant_pos=true`；已武装或状态陈旧时保守视为运动未知，不使用 wheel 或虚构 land detector。

## 融合与生成闭包

编译闭包只启用 GNSS、GNSS yaw、Magnetometer 和 Gravity fusion，永久排除 Wind、Airspeed、Barometer、Flow、Range/Terrain fusion、External Vision、Sideslip、Drag、Aux 和 Wheel。

唯一 `Dima/lib/ekf2/EKF/python/ekf_derivation/derivation.py` 直接定义无 Wind、无 Terrain 的状态向量，也不注册 Wind 生成函数；SymForce 正式生成的存储状态为 22 个 float、误差状态和协方差为 21 维。唯一 `generated/` 目录的公式不得手改或复制。EKF 源保留 `-fno-associative-math`，并继承项目的 `-fno-exceptions/-fno-rtti`。

## 参数与输出

参数只从 `module_ekf2*.yaml` 的正式生成枚举读取。Rover 向 GSF 固定声明 `is_fixed_wing=false`，固定翼向心加速度补偿不会运行，因此权威参数链不暴露无实际消费者的 `EKF2_GSF_TAS`。运行期刷新先构造并验证完整候选；读取失败保留旧值，`reboot_required` 字段继续使用启动快照，不半应用已分配缓冲和融合模式。磁偏角自动保存时，`wq:estimator` 只发布一个固定 float 快照，`EKF2_MAG_DECL` 的 bind/read/commit 和日志由非实时 `wq:lp_default` 提交器完成；失败不阻塞滤波并以 1 Hz 限速重试，物理持久化仍由既有 autosave 负责。

模块发布姿态、local/global position、odometry、Estimator status/event/flags、GNSS/Mag/Gravity/fake aid source、GPS checks、sensor bias、GSF yaw 和 EKF2 timestamps。`estimator_gps_status` 的唯一发布者是本模块；UM982 只发布原始 GPS Topic。

Bias stable 判定沿用 PX4：方差最大值 `<1e-3`、最大/最小比 `<100`、变化不超过 limit 的 10%，且正确的 alignment/fault/clipping/fusion 条件累计超过 10 s。`estimator_sensor_bias` 只提供带 device ID、方差、valid/stable 的运行时估计；参数写回由 VehicleImu 在 Disarmed 后完成。

本目录不得直接访问 Rover 控制器、Arming、PWM、Flash 或 MAVLink wire codec。
