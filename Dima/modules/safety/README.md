# 安全模块

- **职责：** 承载解锁、上锁、预解锁检查、Failsafe、Emergency Stop 和输出许可状态。
- **禁止事项：** 不以 PWM 是否启动代替解锁状态，不允许信号恢复后自动重新解锁。
- **上游 API 保留：** 保留上游 action request、vehicle status 和 actuator armed 等公开状态契约。
