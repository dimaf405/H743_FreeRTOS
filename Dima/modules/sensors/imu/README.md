# VehicleImu 前端与 IMU 链路

本目录只实现把原始加速度计和陀螺仪数据积分为 `vehicle_imu` 的前端。固定 ICM-42688-P SPI4 设备驱动位于 `Dima/drivers/imu/icm42688p/`，QGC 校准协调器位于 sibling `Dima/modules/sensors/calibration/`；三者不得重新混为一个平台后端。

## 固定上游

- PX4-Autopilot v1.17.0：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- 参考目录：`src/drivers/imu/invensense/icm42688p`、`src/modules/sensors/vehicle_imu`
- 许可证：BSD-3-Clause；Dima 适配保留来源与 SPDX 标识

## 数据链路

- ICM driver 使用 SPI Mode 0，并按 PX4 请求 24 MHz 最大频率；STM32 总线层从 SPI45 的 D2PCLK1 实际时钟选择不超频分频器。当前 120 MHz PCLK1 选择 `/8`，实际为 15 MHz；WHOAMI 必须为 `0x47`，固定设备 ID 为 `0x00260022`。
- ICM driver 配置加速度计 ±16 g、陀螺仪 ±2000 deg/s、ODR 8 kHz 和 10 样本高分辨率 FIFO watermark，并发布 `sensor_accel`、`sensor_gyro` 与两个 FIFO Topic。
- 本目录的 `VehicleImu` 完成流合法性、校准、板级旋转和积分，发布 `vehicle_imu` 与 PX4 `vehicle_imu_status`。原始 `sensor_accel/sensor_gyro` 始终保留给校准；未保存校准或保存的设备 ID 不匹配时使用 offset=0、scale=1 的 identity correction，绝不套用其他设备的参数，也不停止合法传感器数据发布。
- `SENS_BOARD_ROT` 使用 PX4 rotation 0..40；`CAL_ACC0_*` 和 `CAL_GYRO0_*` 只在设备 ID 匹配时应用。
- `IMU_INTEG_RATE` 支持 100/200/250/400 Hz，按实测批次周期选择最接近目标的积分边界；纯 correction/积分参数变化按 PX4 只在 Disarmed 前端直接应用，不另取维护 ticket。
- `SENS_IMU_AUTOCAL` 默认启用。VehicleImu 只接受 1 s 内新鲜、device ID 匹配且 `valid && stable` 的单实例 `estimator_sensor_bias`；加速度按 `offset_new=offset_old+(R^T*bias)./scale`、陀螺按 `offset_new=offset_old+R^T*bias` 合并回传感器轴。已武装时只缓存候选，Disarmed 后才由 realtime `wq:sensors` 发布固定大小的不可变事务快照，再由非实时 `wq:lp_default` 参数提交器写入 ID 与全部 offset/scale；pending/running 期间前端不改写请求，成功后静默 30 s，失败保留候选并以 1 Hz 重试。Accel/Gyro 变化分别不超过 0.05 m/s²、0.01 rad/s 时不重复写入，物理持久化仍由 autosave 完成。
- 时间戳倒退或间隔超过 20 ms 时清空双通道积分器；新 accel/gyro 必须各自重新 prime 后才能形成下一个合法积分窗口。
- `vehicle_imu_status` 每秒发布 device ID、error count、batch/raw rate、累计三轴 clipping、振动、coning、均值/方差和温度；错误或 clipping 可提前触发状态发布。clipping 是诊断，不直接丢弃样本。
- `SENS_IMU_CLPNOTI` 默认启用；clipping 日志只在新故障边沿报告一次。驱动重启诊断合并为一条核心摘要，并需约 1 秒连续成功 FIFO publication 后才允许下一次独立故障再次报告；成功 probe/恢复不写进度日志。
- MAVLink 按 PX4 USB 默认频率发布 50 Hz `HIGHRES_IMU` 和 25 Hz `SCALED_IMU`。`HIGHRES_IMU` 由新且合法的 `vehicle_imu` 驱动并携带最近一次合法 `vehicle_magnetometer`；`SCALED_IMU` 是 PX4 在 USB 流表中注册的原始传感器消息，由 `vehicle_imu + raw sensor_mag` 驱动。磁场新鲜度只影响 `SYS_STATUS` health，不会把最近一次合法磁场值从这两个诊断流中清零；`calibration_count=0` 表示 identity/尚未保存校准，不表示故障。PX4 v1.17 的流注册表没有 `RAW_IMU`，当前单实例产品也不伪造 `SCALED_IMU2/3`。

## QGC 校准

Commander 唯一接收并 ACK PX4 的 `MAV_CMD_PREFLIGHT_CALIBRATION`，再通过生成的
`sensor_calibration_request` topic 把 gyro/accel/mag 工作分发给 sibling
`modules/sensors/calibration/SensorCalibration`；本目录只消费其提交后的参数结果：

- `param1=1`：静止陀螺仪校准，约 3 s 采样并检查运动和方差，写入 `CAL_GYRO0_ID/XOFF/YOFF/ZOFF`。
- `param5=1`：六面加速度计校准；自动识别 back/front/left/right/down/up，每面需要稳定采样，写入设备 ID、三轴 offset 和 scale。
- `param2=1`：六面磁力计校准；每面稳定识别后要求至少 0.5 rad 实际净旋转，并在 7 s 内收满 40 个去重的原始磁场点，六面固定 240 点后写入设备 ID、三轴 offset 和 diagonal scale。
- 校准只允许在 Disarmed 且传感器数据新鲜时开始；期间 Commander 拒绝 Arm/Unkill 等正向动作，但保留 Disarm/Kill/Termination。
- 参数通过单个原子通知批量提交；校准协调器先确认对应 `parameter_update.instance` 已被 `VehicleImu` 或 `VehicleMagnetometer` 应用，再逐项核对 active correction 的 ID/offset/scale。identity 数据路径在首次校准前保持正常，新参数应用后再通过 calibration count/参数握手确认校准已生效；成功或回滚握手完成前一直保持 arming interlock。与 PX4 v1.17 的 gyro/accel/mag `ParametersSave + param_notify_changes` 路径一致，`[cal] done` 不等待 `param_save_default(true)`，物理持久化由现有 autosave 随后完成；断电/重启保持性因此仍需板端验证。
- QGC 状态由 PX4 v2 `[cal] ...` STATUSTEXT 协议驱动；校准事务按 PX4 Commander worker 架构运行在非实时 `wq:lp_default`，协议文本走无普通等级过滤的 RAW 日志路径，并重复 PX4 的 orientation/side-done 关键转换文本以抵抗单帧丢失。RAW 仍遵守“实时队列禁止格式化”的全局合同，因此不得把 `SensorCalibration` 放回 `wq:sensors`；全零校准命令取消当前传感器校准。

## 板端验证边界

源码静态检查和 Windows 构建不能代替以下 `BOARD PENDING` 证据：WHOAMI/寄存器回读、SPI/DMA/中断波形、原始与状态 Topic 频率、断流/冻结/时间戳倒退/错误密度恢复、六面实物校准残差、温漂、重启后参数持久化，以及 QGC 向导与 `HIGHRES_IMU/SCALED_IMU/SYS_STATUS` 实测。
