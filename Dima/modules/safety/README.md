# 安全模块

- **职责：** 承载解锁、上锁、预解锁检查、Failsafe、Emergency Stop 和输出许可状态。
- **禁止事项：** 不以 PWM 是否启动代替解锁状态，不允许信号恢复后自动重新解锁。
- **来源：** Commander 行为与公开消息固定来自 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；ArduPilot `3f2e4763accb` 只作 Rover 安全行为参考。
- **调度：** Commander 复用 `wq:hp_default`，20 ms 检查 RC/参数，状态变化立即发布，静态状态最长 500 ms 刷新一次。
- **状态：** 只支持 Manual 和内部 Termination；RC/参数故障不会自动重新解锁，Termination 锁存到 MCU 重启。
- **解锁：** RC Action 与外部 MAVLink ARM 共用同一 Commander 预检；当前只有 RC Manual 控制源，因此都要求 RC 新鲜、摇杆居中。Commander 还通过 `actuator_output_status` 要求快照新鲜且 sequence 严格前进、后端 ready、参数无 pending、至少一右一左映射，并已经建立 `DISARMED_NEUTRAL`。Commander 把固定 `NAV_RCL_ACT=6`、`NAV_DLL_ACT=0` 纳入参数有效性：RC 丢失强制 Disarm，USB/GCS 断开不触发动作，偏离该合同则 fail-closed。
- **输出故障恢复：** Armed 下输出 Fault/Stale、映射缺失或命令链失效会锁存执行器 Failsafe 并强制 Disarm。Disarmed 下允许一份新鲜、映射完整且后端 ready 的 `HARD_SAFE_OFF` 快照先证明故障收敛，随后才恢复 neutral，避免 Commander 与 MotorOutput 永久自锁。
- **Kill：** Kill 即 Disarm 并 hard-off；Unkill 只清 Kill，不恢复 Armed，必须重新产生 Arm 边沿。
- **RC 校准：** Commander 独占 `MAV_CMD_PREFLIGHT_CALIBRATION` 裁决；Disarmed 下只接受 QGC 的 `param4=1` 开始和全零停止。`vehicle_status.rc_calibration_in_progress` 纳入解锁预检，校准期拒绝 RC ARM/Unkill/模式正向动作，但保留 Disarm/Kill/Termination。
- **存储：** ParameterService 在 ARMED 期间禁止 Flash 保存和擦除；Autosave 保持待处理并在 Disarm 后重试。
- **阶段边界：** 阶段 4 不接 PWM、Mixer、RoverDifferential 或 HAL 执行器输出，车辆仍不能运动。
- **上游 API 保留：** 保留上游 action request、vehicle status 和 actuator armed 等公开状态契约。
