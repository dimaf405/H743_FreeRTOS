# Rover 控制与执行器组合

- `ManualMotionAdapter` 只把 `manual_control_setpoint.throttle/yaw` 转成 `rover_motion_request` 的前后、左右两轴；未来 Navigation 也只能从同一消息边界接入。
- `RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 校验请求采样时间、Commander 三 Topic 一致性和参数快照，再发布 `actuator_motors`。Motor1 为右侧、Motor2 为左侧，其余十路保持 NaN。
- `DifferentialDrive` 使用固定存储，组合 PX4 v1.17.0 的两轴控制边界与 ArduPilot Rover 的倒车转向、饱和优先级、油门 slew、静摩擦补偿、反向不对称和独立换向延时行为。ArduPilot GPL 源码只作行为参考，不复制到本目录。
- 参数更新在 ARMED 期间只标记 pending；完整、同时间戳的 DISARMED 安全快照到达后才整体应用，禁止半更新。
- `MotorOutput` 将两个双向 Motor function 映射到 S1～S6；每路只公开 `FUNC/MIN/CENT/MAX/REV`，默认 Disabled，零命令严格映射到中心脉宽。
- 固定物理映射为 S1/PB0/TIM8_CH2N、S2/PB1/TIM8_CH3N、S3～S6/PA0～PA3/TIM5_CH1～CH4。六路硬件访问只经过平台 `ActuatorPwm` 能力。
- `MotorOutput` 仅在 Commander 三个安全 Topic 构成完整、新鲜且严格前进的 ARMED/Manual 快照，并且 `actuator_motors.timestamp` 与 `timestamp_sample` 均满足 `COM_ACT_LOSS_T` 时启动输出；任何负向安全状态或命令失效都立即 safe-off。
- `MotorOutput::stop()` 完成后必须停止 TIM5/TIM8、清零 CCR 并把六路引脚恢复为 GPIO 低电平；Application Runtime 只有在板级 `safe_off_confirmed()` 成功后才能继续释放资源。
- 当前默认六路全部 Disabled，即使 ARMED 也不会产生物理脉冲。启用通道后，双向油门以 `CENT` 为零点，负向映射到 `MIN`、正向映射到 `MAX`，再按通道 `REV` 处理方向。
