# Commander 安全与校准仲裁

Commander 只维护安全状态和 uORB 投影，不直接拥有传感器、串口或 PWM HAL。

## 校准仲裁

- Commander 是 `vehicle_command` 的唯一订阅者和 ACK 所有者。它按 PX4 的
  `MAV_CMD_PREFLIGHT_CALIBRATION` 规则分类请求，再通过由 schema 生成的
  `sensor_calibration_request` topic 分发到低优先级校准 worker。
- RC 校准仍由 Commander 直接处理：Disarmed 下 `param4=1` 进入，全零退出；
  gyro/mag/accel worker 只发布 `[cal]` 进度/终态，不再解析命令或发送第二个 ACK。
- `sensor_calibration_status.active` 投影为 `vehicle_status.calibration_enabled`。RC 或传感器校准期间，预检拒绝 Arm，RC 正向 Arm/Unkill 动作被屏蔽；Disarm、Kill、Termination 始终保留。
- 传感器校准还持有 `ArmedFlashCoordinator` maintenance interlock，防止 Commander 在状态消息传播窗口中抢先解锁。

## 既有安全合同

RC loss 固定触发 Disarm，GCS loss 不触发导航动作。MotorOutput 状态、参数有效性、RC 新鲜度和摇杆居中共同决定 `pre_flight_checks_pass`。传感器是否检测到目前是可观测健康信息，不会静默改变手动驾驶或 BootHealth 的既有策略。

源码/构建验证不等于车辆安全验证；校准中负向动作、参数应用竞争、看门狗、PWM safe-off 和真实解锁边沿均保持 `BOARD PENDING`。
