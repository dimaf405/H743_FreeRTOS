# 安全模块

- **职责：** 承载解锁、上锁、预解锁检查、Failsafe、Emergency Stop 和输出许可状态。
- **禁止事项：** 不以 PWM 是否启动代替解锁状态，不允许信号恢复后自动重新解锁。
- **来源：** Commander 行为与公开消息固定来自 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；ArduPilot `3f2e4763accb` 只作 Rover 安全行为参考。
- **调度：** Commander 复用 `wq:hp_default`，20 ms 检查 RC/参数，状态变化立即发布，静态状态最长 500 ms 刷新一次。
- **状态：** 只支持 Manual 和内部 Termination；RC/参数故障不会自动重新解锁，Termination 锁存到 MCU 重启。
- **存储：** ParameterService 在 ARMED 期间禁止 Flash 保存和擦除；Autosave 保持待处理并在 Disarm 后重试。
- **阶段边界：** 阶段 4 不接 PWM、Mixer、RoverDifferential 或 HAL 执行器输出，车辆仍不能运动。
- **上游 API 保留：** 保留上游 action request、vehicle status 和 actuator armed 等公开状态契约。
