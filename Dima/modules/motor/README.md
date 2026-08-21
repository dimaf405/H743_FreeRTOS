# MotorOutput 运行模块

- **职责：** 将两路可逆 Motor 命令映射到 S1～S6，并独立校验 Commander 安全快照、命令新鲜度和参数快照。
- **边界：** 不实现 Rover 混控；硬件访问只经过 `platform::ActuatorPwm`。产品普通 PWM 包络为 500～2500 us，默认仍为 1000/1500/2000 us。普通 Disarmed 只在参数有效的通道持续输出各自 `CENT`，Disabled 或参数无效的通道保持无脉冲。
- **两级安全：** 启动、无任何有效通道、Kill、Termination、Failsafe、Armed 命令超时、参数切换、Retry/Fault 和关闭进入 `HARD_SAFE_OFF`，停止 TIM5/TIM8、CCR 清零并恢复六路 GPIO 低；存在有效通道的普通 Disarm 进入 `DISARMED_NEUTRAL`。
- **参数恢复：** 参数协议接受并保留普通有限原值；MotorOutput 在完整 Disarmed 快照中消费校验。未知 `FUNC`、非 0/1 的 `REV` 或无效 `MIN/CENT/MAX` 只禁用对应通道，其余有效通道继续输出中立值；剩余映射至少一右一左时仍可解锁，否则 Commander 只拒绝解锁而不停止 MotorOutput、BootHealth、USB 或 MAVLink。只有 PWM 后端、调度或发布故障才进入生命周期 Error。
- **异步 Topic：** “禁止 ACTIVE”和“必须 hard-off”使用独立观察锁存。普通 Disarm 的首条 Topic 立即阻断 ACTIVE，完整一致快照后才允许 neutral；任一 Kill/Termination/Failsafe Topic 先到即同时禁止 neutral。
- **生命周期：** 启动先建立 safe-off，停止时关闭 TIM5/TIM8、清零 CCR 并恢复 GPIO 低电平；Runtime 只有在 `safe_off_confirmed()` 后才能继续释放资源。
