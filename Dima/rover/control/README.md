# Rover 控制与执行器组合

- `RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 校验请求采样时间、Commander 三 Topic 一致性和参数快照；Manual 直接消费归一化双轴，Navigation 消费物理速度/yaw-rate 并执行 Speed PI 与 YawRate PI，再发布 `actuator_motors`。Motor1 为右侧、Motor2 为左侧，其余十路保持 NaN。
- 纯算法 `Dima/lib/rover/DifferentialDrive.*` 使用固定存储，组合 PX4 v1.17.0 的两轴控制边界与 ArduPilot Rover 的倒车转向、饱和优先级、油门 slew、静摩擦补偿、反向不对称和独立换向延时行为；本目录只保留消息、参数和安全状态的运行适配。ArduPilot GPL 源码只作行为参考，不复制。
- 参数更新在 ARMED 期间只标记 pending；完整、同时间戳的 DISARMED 安全快照到达后才整体应用，禁止半更新。
- Rover Manual 与 AUTO 分别位于 `rover/modes/ManualMode.*`、`AutoMode.*`；两者都只能发布 one-of `rover_motion_request`，不得直接进入本目录内部对象。控制层先计算 steering，再把 Navigation longitudinal 限制到 `1-|steering|`，且自动控制固定 `manual_source=false`。
- 安全 PWM 输出位于 `modules/motor/`，本目录不得直接访问 `ActuatorPwm` 或板级 TIM/GPIO。
