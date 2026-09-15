# MotorOutput 运行模块

- **职责：** 将两路可逆 Motor 命令映射到 S1～S6，并独立校验 Commander 安全快照、命令新鲜度和参数快照。
- **边界：** 不实现 Rover 混控；硬件访问只经过 `platform::ActuatorPwm`。产品普通 PWM 包络为 500～2500 us，默认仍为 1000/1500/2000 us。普通 Disarmed 只在参数有效的通道持续输出各自 `CENT`，Disabled 或参数无效的通道保持无脉冲。
- **两级安全：** 启动、无任何有效通道、Kill、Termination、Failsafe、Armed 命令超时、参数切换、Retry/Fault 和关闭进入 `HARD_SAFE_OFF`，停止 TIM5/TIM8、CCR 清零并恢复六路 GPIO 低；存在有效通道的普通 Disarm 进入 `DISARMED_NEUTRAL`。
- **参数恢复：** 参数协议接受并保留普通有限原值；MotorOutput 在完整 Disarmed 快照中消费校验。未知 `FUNC`、非 0/1 的 `REV` 或无效 `MIN/CENT/MAX` 只禁用对应通道，其余有效通道继续输出中立值；剩余映射至少一右一左时仍可解锁，否则 Commander 只拒绝解锁而不停止 MotorOutput、BootHealth、USB 或 MAVLink。只有 PWM 后端、调度或发布故障才进入生命周期 Error。
- **异步 Topic：** “禁止 ACTIVE”和“必须 hard-off”使用独立观察锁存。普通 Disarm 的首条 Topic 立即阻断 ACTIVE，完整一致快照后才允许 neutral；任一 Kill/Termination/Failsafe Topic 先到即同时禁止 neutral。
- **生命周期：** 启动先建立 safe-off，停止时关闭 TIM5/TIM8、清零 CCR 并恢复 GPIO 低电平；Runtime 只有在 `safe_off_confirmed()` 后才能继续释放资源。

- **后端命令证据：** 仅后端确认脉宽命令/停波后记录 timestamp_output，按已接受 PWM 脉宽及各通道 REV 反解逻辑右/左轮均值。状态发布的 applied_right/applied_left 用于磁样本时间对齐；Retry/Fault/未知状态不授权补偿，不表示存在电流或轮速传感器。
- **校准看门狗：** 不使用校准专属幅值拒绝；保留普通归一化和新鲜度规则，校准命令超时收紧为至多 100 ms。冻结包络及 0.15/s slew 仍由差速层逐周期约束。

## 零输出、方向与 APM 行为对照

- `MotorRight/MotorLeft` 是面向车头时的车辆右/左侧，`PWM_Sx_REV` 独立匹配安装方向。混控始终为右轮 `T-S`、左轮 `T+S`，急转时允许内轮反转；没有“有油门时禁止内轮反转”的限制。
- 对照仓库既定 APM `3f2e4763accb` 的 Rover pilot normalization、`output_skid_steering/get_scaled_throttle/ReverseThrottle` 行为。Manual 在入口对 `|T|+|S|>1` 的两轴同比缩放，再执行油门 slew、倒车转向、非对称域饱和优先级、逐轮整形和独立换向等待；未导入 APM 代码或额外转向滤波参数。
- `MOT_SLEW_RATE=0`、`MOT_REV_DELAY=0` 分别禁用软件斜率与换向等待。启用 slew 时，Manual 回中和前后切换也遵守斜率；导航/校准的零纵向停车仍立即清除历史。保留本项目 Arm ramp 和逐轮冻结包络 E，不扩大自动校准权限。
- APM 的非零输出先加 MIN，再执行 EXPO。这里在 E 域使用 `q=MIN/E+(1-MIN/E)*|u|`、`output=sign(u)*E*expo(q)`，E=1 时与 APM 百分比公式一致；精确零命令直接返回零。MIN=0.15 可放大小残余命令，但不能自行产生零输入输出；RC 中位/死区与电调停止脉宽分别核对，不能用 MIN 替代死区。
- 六路字段、功能枚举和参数绑定由正式生成链提供。可逆通道严格要求 `MIN < CENT < MAX`，零命令映射为各自 `CENT`，REV 不移动中位。
- Manual Armed 时 QGC 每秒可见同编号的 `[drive in]`/`[drive out]`/`[drive pwm]`/`[drive src]`，包含归一化输入、控制请求、两轮命令、后端确认值、六路 PWM、左右映射、各级限幅和样本时龄。`same=0` 表示不能作同样本因果推断；后端确认值不是示波器或轮速测量。
- 现场接线为 S1 左轮、S3 右轮，对应 mapR/L=04/01。源码中的 S1/S2 N-only 极性修正仍保留，右轮微动、输入比例与实际波形仍需板端证据。

2026-09-14 现场诊断补充：日志有 11 组稳定回中帧仍为 `T=-0.012`、两轮命令 `-0.012`、S1/S3 命令脉宽 1505 us；同样本 n=10 显示该负偏差触发了倒车转向反转。另有 27 组记录标记油门 slew 生效，不能按“运行参数已禁用 slew”解释。已补齐默认 RC1/RC2 的 APM 30 us 死区；已有配置需显式应用。未标注杆位的日志尚不能证明右前/右后持续组合在软件中互换。

## PWM 后端核对与加固（2026-09-15）

- 脉宽量化对照 APM `SRV_Channel::pwm_from_angle`，先截断偏离 CENT 的幅值再按符号加减；小于 1 us 的变化保留中位，避免最终绝对脉宽四舍五入造成正负边界差异。
- Board 每次启动/写帧核对实际 PSC/ARR/RCR、CEN、边沿对齐模式、PWM1/OCxPE、极性/使能、MOE、主从同步及 GPIO AF。定时器时钟计算覆盖 TIMPRE，不再只信任 HAL Init 缓存。
- 配置或写入异常进入统一停波路径：GPIO 先拉低，再停两个计数器、关闭输出/DMA/IRQ、清 CCR，并核对 GPIO 模式及低电平读回。停波不依赖原 PWM 配置或 HAL Instance 有效。
- 六路 UDIS/CCR 写入在短临界区内完成，读回比较值后才接受帧；运行中不停止计数、不强制 UG。启动时保持 GPIO 低电平，建立并核对六路零 CCR 后再开放 AF。
- CCR 是预装载寄存器；当前 timestamp_output 仍是后端接受命令的时刻，不能当作硬件更新事件或引脚边沿时间。自然更新边界通常相隔 20 ms，寄存器读回不能替代实际脉宽/频率/相位测量，也不能据此证明八方向异常已消除。
