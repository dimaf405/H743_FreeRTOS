# 消息契约

- **职责：** 维护模块间 Topic、命令、状态、时间戳、单位和有效性字段定义。
- **禁止事项：** 不在消息定义中访问 HAL 或实现业务逻辑，不随意重命名已经采用的上游字段。
- **上游 API 保留：** 保留上游消息名称、字段、单位和枚举语义；产品扩展使用明确的 Dima 前缀或独立消息。
- `schemas/*.msg` 是 uORB 消息名称、字段与布局的唯一权威源；只使用 PX4 原生 `ORB_QUEUE_LENGTH` 与 `# TOPICS`，禁止 `@queue`、`@alias`、`@external`、`@abi` 等本地语法。
- PX4 同名 schema 与 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 vendored 参考文件逐字节核对；产品专用消息也必须交给同一套原始生成器，不能在本地补写结构体布局。
- `tools/uorb/generate_messages.py` 只编排 PX4 原始 `px_generate_uorb_topic_files.py` 和 EmPy 模板。Topic 头/源、ID、消息 hash、JSON、alias 与 `uORBTopics` 注册目录全部位于 `build/generated`，旧 ABI lock 和手写 catalog 已退役。
- `tools/upstream/uorb_v1_17/SOURCE_MANIFEST.json` 对上游脚本、helper、模板及参考消息做逐文件 SHA-256 闭包校验；禁止手工编辑任何派生产物。
- `actuator_motors` 完整采用 PX4 v1.17.0 version 0 契约，保持 12 路公开数组；阶段 5 仅使用 Motor1/右侧和 Motor2/左侧，其余项必须为 NaN。
- `rover_motion_request` 是 Manual 与 Navigation 共用的两轴产品边界：Manual 只允许 `SOURCE_MANUAL + MODE_NORMALIZED_AXES` 且物理量字段为 NaN；AUTO/Hold 只允许 `SOURCE_NAVIGATION + MODE_SPEED_YAW_RATE` 且归一化字段为 NaN。
- `rover_navigation_status` 由 `AutoMode` 发布任务 generation/current/count、控制状态、故障原因、到点状态、路径误差与物理量 setpoint；其 schema、Topic ID、布局和注册表全部由同一 uORB 权威生成链产生。
- `actuator_output_status` 记录六路 PWM 的 configured/right/left mask、应用脉宽以及 `HARD_SAFE_OFF / DISARMED_NEUTRAL / ACTIVE / RETRY / FAULT` 状态；Commander 只通过该内部 uORB 契约做输出就绪 pre-arm 与故障恢复，不直接依赖 MotorOutput 类，也不新增 MAVLink 线协议。
- `estimator_gps_status` 固定采用 PX4 v1.17.0 字段合同，并由唯一 EKF2 实例发布完整 GnssChecks 结果；UM982 只发布 `sensor_gps`/`vehicle_gps_position`，不得再维护同 Topic 的简化发布者。
- `vehicle_imu_status` 固定采用 PX4 v1.17.0 字段合同，承载单 IMU 的 identity、rate/error/clipping、振动、coning、均值/方差和温度；它不声称实现 `SensorsStatusImu` 多实例一致性投票。
