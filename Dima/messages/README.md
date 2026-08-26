# 消息契约

- **职责：** 维护模块间 Topic、命令、状态、时间戳、单位和有效性字段定义。
- **禁止事项：** 不在消息定义中访问 HAL 或实现业务逻辑，不随意重命名已经采用的上游字段。
- **上游 API 保留：** 保留上游消息名称、字段、单位和枚举语义；产品扩展使用明确的 Dima 前缀或独立消息。
- `actuator_motors` 完整采用 PX4 v1.17.0 version 0 契约，保持 12 路公开数组；阶段 5 仅使用 Motor1/右侧和 Motor2/左侧，其余项必须为 NaN。
- `rover_motion_request` 是 Manual 与未来 Navigation 共用的两轴产品边界。阶段 5 只允许 `MANUAL + NORMALIZED_AXES`，导航接口仅预留、不创建空导航模块。
- `actuator_output_status` 记录六路 PWM 的 configured/right/left mask、应用脉宽以及 `HARD_SAFE_OFF / DISARMED_NEUTRAL / ACTIVE / RETRY / FAULT` 状态；Commander 只通过该内部 uORB 契约做输出就绪 pre-arm 与故障恢复，不直接依赖 MotorOutput 类，也不新增 MAVLink 线协议。
- `estimator_gps_status` 固定采用 PX4 v1.17.0 字段合同；Dima 只实现有真实输入的 fix/nsats/PDOP/accuracy/spoof 子集，未支持的 EKF drift/speed-offset 位保持 false、数值为 NaN。
- `vehicle_imu_status` 固定采用 PX4 v1.17.0 字段合同，承载单 IMU 的 identity、rate/error/clipping、振动、coning、均值/方差和温度；它不声称实现 `SensorsStatusImu` 多实例一致性投票。
