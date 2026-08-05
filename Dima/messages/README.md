# 消息契约

- **职责：** 维护模块间 Topic、命令、状态、时间戳、单位和有效性字段定义。
- **禁止事项：** 不在消息定义中访问 HAL 或实现业务逻辑，不随意重命名已经采用的上游字段。
- **上游 API 保留：** 保留上游消息名称、字段、单位和枚举语义；产品扩展使用明确的 Dima 前缀或独立消息。
- `actuator_motors` 完整采用 PX4 v1.17.0 version 0 契约，保持 12 路公开数组；阶段 5 仅使用 Motor1/右侧和 Motor2/左侧，其余项必须为 NaN。
- `rover_motion_request` 是 Manual 与未来 Navigation 共用的两轴产品边界。阶段 5 只允许 `MANUAL + NORMALIZED_AXES`，导航接口仅预留、不创建空导航模块。
- `actuator_output_status` 记录六路 PWM 的实际 safe-off/active/retry/fault 状态、通道掩码和应用脉宽，供启动健康与调试观察，不反向参与控制决策。
