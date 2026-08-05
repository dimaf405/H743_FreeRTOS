# Rover 控制与执行器组合

- `ManualMotionAdapter` 只把 `manual_control_setpoint.throttle/yaw` 转成 `rover_motion_request` 的前后、左右两轴；未来 Navigation 也只能从同一消息边界接入。
- `RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 校验请求采样时间、Commander 三 Topic 一致性和参数快照，再发布 `actuator_motors`。Motor1 为右侧、Motor2 为左侧，其余十路保持 NaN。
- `DifferentialDrive` 使用固定存储，组合 PX4 v1.17.0 的两轴控制边界与 ArduPilot Rover 的倒车转向、饱和优先级、油门 slew、静摩擦补偿、反向不对称和独立换向延时行为。ArduPilot GPL 源码只作行为参考，不复制到本目录。
- 参数更新在 ARMED 期间只标记 pending；完整、同时间戳的 DISARMED 安全快照到达后才整体应用，禁止半更新。
- 当前控制生产者不访问 TIM、GPIO 或 PWM。六路板级输出必须通过后续安全后端消费 `actuator_motors`，并独立执行 Disarm/Kill/Termination/lockdown/超时门控。
