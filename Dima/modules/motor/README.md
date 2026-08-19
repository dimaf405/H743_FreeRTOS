# MotorOutput 运行模块

- **职责：** 将两路可逆 Motor 命令映射到 S1～S6，并独立校验 Commander 安全快照、命令新鲜度和参数快照。
- **边界：** 不实现 Rover 混控；硬件访问只经过 `platform::ActuatorPwm`。健康且左右映射完整的普通 Disarmed 只在已配置通道持续输出各自 `CENT`，Disabled 通道保持无脉冲。
- **两级安全：** 启动、映射无效、Kill、Termination、Failsafe、Armed 命令超时、参数切换、Retry/Fault 和关闭进入 `HARD_SAFE_OFF`，停止 TIM5/TIM8、CCR 清零并恢复六路 GPIO 低；普通 Disarm 才进入 `DISARMED_NEUTRAL`。
- **异步 Topic：** “禁止 ACTIVE”和“必须 hard-off”使用独立观察锁存。普通 Disarm 的首条 Topic 立即阻断 ACTIVE，完整一致快照后才允许 neutral；任一 Kill/Termination/Failsafe Topic 先到即同时禁止 neutral。
- **生命周期：** 启动先建立 safe-off，停止时关闭 TIM5/TIM8、清零 CCR 并恢复 GPIO 低电平；Runtime 只有在 `safe_off_confirmed()` 后才能继续释放资源。
