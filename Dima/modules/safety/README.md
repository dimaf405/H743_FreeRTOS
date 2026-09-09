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
- Auto Calibration 在 Disarmed 显式进入后可以先运行静态 Level，不提前依赖 RC/PWM/双天线；首次正常人工 Arm 授予当前 session，协调器无授予权。内部阶段 Disarm 保留授权；继续请求在本轮外部 RC/命令与安全检查之后才走正常 preflight/maintenance/try_arm，不使用强制 Arm。Arm 不是增益确认。
- 外部 Disarm 在内部已 Disarmed 的提交窗口也撤销整场授权；Kill、RC loss、模式切出、超时、安全/调度故障及重启均撤销。恢复 RC 或重连不会重建授权。只允许明确等待状态、匹配 session/参数代次且消费者已确认的正常续行，不从失败或回滚窗口自动 Arm。
- 开/闭环校准使用共享 `RoverModeContract` 的精确安全投影，闭环仅开启 Velocity/Rates。参数 provisional 期间暂缓持久化，回滚无法确认会锁存 FAILED 与禁 Arm，不能重新进入运动。
- worker 反馈归属在接受请求时锁定；外部 QGC 保持标准 `[cal]` 协议，AUTO-owned Level/Cancel 不发送 `[cal]` start/progress/terminal，避免错误推进另一个 Sensors 界面事务。

## 既有安全合同

回调部分注册失败、正常 stop 和运行期 Error 共用逆注册顺序的注销与调度排空；未注册回调不会移除其他订阅者。公共清理函数不修改 Arm、会话授权或公开状态，调用者仍负责原有 Disarm/撤销时序、Error/Stopped 区分和逐项启动失败原因。

手动模式的 `pre_flight_checks_pass` 只以 MotorOutput 已应用的有效左右电机分配为预检条件：状态新鲜、没有待应用的映射更新，左右各至少一路且掩码一致；不要求 RC 新鲜、摇杆居中、Commander 参数有效或当前 PWM 已处于 Neutral。运动校准仍保留原来的参数、RC、居中和 Neutral 预检；`COM_ARM_STICK_DZ` 仅约束运动校准解锁。

Kill/Termination、RC/传感器校准及活动的自动校准会话仍与手动 Arm 互锁，维护/Flash 仍经过最终原子门。解锁后的 RC loss、Commander 参数故障和执行器故障继续触发 Disarm，因此无有效 RC 时即使手动 Arm 命令通过预检，也不能保持 Armed 或输出动力；GCS loss 不触发导航动作。传感器是否检测到目前是可观测健康信息，不会静默改变手动驾驶或 BootHealth 的既有策略。

源码/构建验证不等于车辆安全验证；校准中负向动作、参数应用竞争、看门狗、PWM safe-off 和真实解锁边沿均保持 `BOARD PENDING`。

## 头文件实现边界

Commander 的普通状态访问器定义在 Commander.cpp；仅改变定义位置，不改变 Armed 语义、安全互锁或输出锁存。 统一审查与验收见 docs/HEADER_IMPLEMENTATION_SPLIT_ZH.md。
