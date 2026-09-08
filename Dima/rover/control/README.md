# Rover 控制与执行器组合

- `RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 校验请求采样时间、Commander 三 Topic 一致性和参数快照；Manual、Navigation 与 Calibration 的逐字段模式校验同执行器层共用 `RoverModeContract`，各层独立保留失鲜/安全快照门控；Manual 直接消费归一化双轴，Navigation 消费物理速度/yaw-rate 并执行 Speed PI 与 YawRate PI，再发布 `actuator_motors`。Motor1 为右侧、Motor2 为左侧，其余十路保持 NaN。
- 纯算法 `Dima/lib/rover/DifferentialDrive.*` 使用固定存储，组合 PX4 v1.17.0 的两轴控制边界与 ArduPilot Rover 的倒车转向、饱和优先级、油门 slew、静摩擦补偿、反向不对称和独立换向延时行为；本目录只保留消息、参数和安全状态的运行适配。ArduPilot GPL 源码只作行为参考，不复制。
- 参数更新在 ARMED 期间只标记 pending；完整、同时间戳的 DISARMED 安全快照到达后才整体应用，禁止半更新。控制参数无效只抑制 Navigation 输出，不参与 Commander 的 Mission 模式切换。
- `RoverControlValidation.hpp` 集中校验两层共有的 Speed/YawRate 内环参数；控制层仍额外检查速度死区小于满油门速度，AutoMode 保留巡航、停车/转向滞回和 Heading 输出边界。各层在自己的参数快照上独立调用，不增加模式切换门槛。
- Rover Manual 与 AUTO 分别位于 `rover/modes/ManualMode.*`、`AutoMode.*`；两者都只能发布 one-of `rover_motion_request`，不得直接进入本目录内部对象。控制层先计算 steering，再把 Navigation longitudinal 限制到 `1-|steering|`，且自动控制固定 `manual_source=false`。
- 安全 PWM 输出位于 `modules/motor/`，本目录不得直接访问 `ActuatorPwm` 或板级 TIM/GPIO。
- `SOURCE_CALIBRATION` 保持独立来源：开环使用 normalized axes，增益验证使用已有 speed/yaw-rate 模式及本层真实 PI。Commander、控制器、PWM 和 watchdog 共用精确开/闭环安全投影，闭环只打开 Velocity/Rates，不伪装成 Mission。
- `RoverDifferentialCalibration` 在完整 Disarmed 快照中独立锁存固定圆、入场巡航/兜底速度和驱动上限，以新鲜同设备 GNSS 复核停车余量；会话快照改变后永久失效，不能单凭协调器标志恢复。普通每轮上限 `min(0.40,MOT_THR_MAX)`；只有入场未配置巡航的指定前进探测段允许到冻结驱动上限，整形后负轮端立即失效，许可下降时不携带历史超限输出。0.15/s 末端 slew、TTL、实际速度/加速度门禁始终保留。
- `rover_control_status` 是本层产生的内部 uORB 反馈，包含请求/session/参数代次、真实 PI 设定/反馈/积分及最终执行器均值/差分。消费者应用确认与“闭环配置可运行”分开，允许旧零增益回滚到导航未就绪状态。
- 校准只在指定阶段允许负纵向/负速度，倒车受 `min(0.3,V_session)` 和普通输出限制；闭环还受当前 provisional 巡航范围限制。反馈补充未零区化前向/侧向/带符号地速及真实混控、整形、MOT slew、Arm ramp、换向等待、末端安全限制原因，正常整形不冒充安全介入，受限样本不授权辨识。
