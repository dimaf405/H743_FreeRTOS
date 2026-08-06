# MotorOutput 运行模块

- **职责：** 将两路可逆 Motor 命令映射到 S1～S6，并独立校验 Commander 安全快照、命令新鲜度和参数快照。
- **边界：** 不实现 Rover 混控；硬件访问只经过 `platform::ActuatorPwm`。任何负向安全状态、无效命令、超时或后端异常都回到物理 safe-off。
- **生命周期：** 启动先建立 safe-off，停止时关闭 TIM5/TIM8、清零 CCR 并恢复 GPIO 低电平；Runtime 只有在 `safe_off_confirmed()` 后才能继续释放资源。
