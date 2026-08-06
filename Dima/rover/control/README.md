# Rover 控制与执行器组合

- `RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 校验请求采样时间、Commander 三 Topic 一致性和参数快照，再发布 `actuator_motors`。Motor1 为右侧、Motor2 为左侧，其余十路保持 NaN。
- 纯算法 `Dima/lib/rover/DifferentialDrive.*` 使用固定存储，组合 PX4 v1.17.0 的两轴控制边界与 ArduPilot Rover 的倒车转向、饱和优先级、油门 slew、静摩擦补偿、反向不对称和独立换向延时行为；本目录只保留消息、参数和安全状态的运行适配。ArduPilot GPL 源码只作行为参考，不复制。
- 参数更新在 ARMED 期间只标记 pending；完整、同时间戳的 DISARMED 安全快照到达后才整体应用，禁止半更新。
- Rover Manual 模式位于 `rover/modes/ManualMode.*`；未来 Navigation 也只能发布 `rover_motion_request`，不得直接进入本目录内部对象。
- 安全 PWM 输出位于 `modules/motor/`，本目录不得直接访问 `ActuatorPwm` 或板级 TIM/GPIO。
