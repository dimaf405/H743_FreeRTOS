# 消息契约

- **职责：** 维护模块间 Topic、命令、状态、时间戳、单位和有效性字段定义。
- **禁止事项：** 不在消息定义中访问 HAL 或实现业务逻辑，不随意重命名已经采用的上游字段。
- **上游 API 保留：** 保留上游消息名称、字段、单位和枚举语义；产品扩展使用明确的 Dima 前缀或独立消息。
- `schemas/*.msg` 是 uORB 消息名称、字段与布局的唯一权威源；只使用 PX4 原生 `ORB_QUEUE_LENGTH` 与 `# TOPICS`，禁止 `@queue`、`@alias`、`@external`、`@abi` 等本地语法。
- PX4 同名 schema 的字段、类型、单位与枚举常量必须和 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 vendored 参考一致。`ORB_QUEUE_LENGTH` 可调整产品调度窗口；`# TOPICS` 可裁剪无功能消费者的别名，但必须保留上游首个主 Topic 和原有顺序，不得新增、重命名或重复别名。固定上游快照保持原样，产品专用消息仍交给同一套原始生成器。
- `tools/uorb/generate_messages.py` 只编排 PX4 原始 `px_generate_uorb_topic_files.py` 和 EmPy 模板。Topic 头/源、ID、消息 hash、JSON、alias 与 `uORBTopics` 注册目录全部位于 `build/generated`，旧 ABI lock 和手写 catalog 已退役。
- `tools/upstream/uorb_v1_17/SOURCE_MANIFEST.json` 对上游脚本、helper、模板及参考消息做逐文件 SHA-256 闭包校验；禁止手工编辑任何派生产物。
- 两个原始 FIFO Topic 已退出产品 schema；ICM42688P 用私有批次工作区发布 `sensor_accel/sensor_gyro`。未实现的云台、模式执行器、控制设定和第二手动输入别名也不再进入生成目录，不保留空注册项。
- `actuator_motors` 完整采用 PX4 v1.17.0 version 0 契约，保持 12 路公开数组；阶段 5 仅使用 Motor1/右侧和 Motor2/左侧，其余项必须为 NaN。
- `rover_motion_request` 是 Manual 与 Navigation 共用的两轴产品边界：Manual 只允许 `SOURCE_MANUAL + MODE_NORMALIZED_AXES` 且物理量字段为 NaN；AUTO/Hold 只允许 `SOURCE_NAVIGATION + MODE_SPEED_YAW_RATE` 且归一化字段为 NaN。
- `rover_navigation_status` 由 `AutoMode` 发布任务 generation/current/count、控制状态、故障原因、到点状态、路径误差与物理量 setpoint；其 schema、Topic ID、布局和注册表全部由同一 uORB 权威生成链产生。
- `SOURCE_CALIBRATION` 通过 `rover_motion_request` 支持受限开环与 speed/yaw-rate 闭环，仍严格 one-of；`auto_calibration_request/status`、`rtk_heading_status` 和 `rover_control_status` 都是本地 uORB 合同，不是私有 MAVLink wire 消息。固定圆、事务/验证状态及真实控制反馈供内部安全消费者和 ULog 使用，QGC 不直接解码它们。
- `sensor_calibration_request.feedback_owner` 的 NONE/QGC/AUTO 枚举从 schema 生成；内部 Level/Cancel 不接管 QGC Sensors 的 `[cal]` 终态。QGC 看到的是标准命令/ACK、STATUSTEXT、参数和 Standard Modes 服务，而不是本地 msg 布局。
- 一次 Arm 扩展仍是本地合同：`auto_calibration_request` 增加阶段继续请求及参数计数，`auto_calibration_status` 记录授权诊断镜像、冻结速度/输出策略、RAM 已验证阶段和路径字段观测；`rover_control_status` 增加原始反馈与限制器原因。Level 请求/状态记录起始计数与自身写入增量，避免把外部并发改参吞成内部成功；这些字段均由权威 schema 生成，不扩展 MAVLink wire。
- `actuator_output_status` 记录六路 PWM 的 configured/right/left mask、应用脉宽以及 `HARD_SAFE_OFF / DISARMED_NEUTRAL / ACTIVE / RETRY / FAULT` 状态；Commander 只通过该内部 uORB 契约做输出就绪 pre-arm 与故障恢复，不直接依赖 MotorOutput 类，也不新增 MAVLink 线协议。
- `estimator_gps_status` 固定采用 PX4 v1.17.0 字段合同，并由唯一 EKF2 实例发布完整 GnssChecks 结果；UM982 只发布 `sensor_gps`/`vehicle_gps_position`，不得再维护同 Topic 的简化发布者。
- `vehicle_imu_status` 固定采用 PX4 v1.17.0 字段合同，承载单 IMU 的 identity、rate/error/clipping、振动、coning、均值/方差和温度；它不声称实现 `SensorsStatusImu` 多实例一致性投票。
