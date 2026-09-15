# Rover 控制与执行器组合

- `RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 校验请求采样时间、Commander 三 Topic 一致性和参数快照；Manual、Navigation 与 Calibration 的逐字段模式校验同执行器层共用 `RoverModeContract`，各层独立保留失鲜/安全快照门控；Manual 直接消费归一化双轴，Navigation 消费物理速度/yaw-rate 并执行 Speed PI 与 YawRate PI，再发布 `actuator_motors`。Motor1 为右侧、Motor2 为左侧，其余十路保持 NaN。
- 纯算法 `Dima/lib/rover/DifferentialDrive.*` 使用固定存储，组合 PX4 v1.17.0 的两轴控制边界与 ArduPilot Rover 的倒车转向、饱和优先级、油门 slew、静摩擦补偿、反向不对称和独立换向延时行为；本目录只保留消息、参数和安全状态的运行适配。ArduPilot GPL 源码只作行为参考，不复制。
- 参数更新在 ARMED 期间只标记 pending；完整、同时间戳的 DISARMED 安全快照到达后才整体应用，禁止半更新。控制参数无效只抑制 Navigation 输出，不参与 Commander 的 Mission 模式切换。
- `RoverControlValidation.hpp` 集中校验两层共有的 Speed/YawRate 内环参数；控制层仍额外检查速度死区小于满油门速度，AutoMode 保留巡航、停车/转向滞回和 Heading 输出边界。各层在自己的参数快照上独立调用，不增加模式切换门槛。
- Rover Manual 与 AUTO 分别位于 `rover/modes/ManualMode.*`、`AutoMode.*`；两者都只能发布 one-of `rover_motion_request`，不得直接进入本目录内部对象。控制层先计算 steering，再把 Navigation longitudinal 限制到 `1-|steering|`，且自动控制固定 `manual_source=false`。
- 安全 PWM 输出位于 `modules/motor/`，本目录不得直接访问 `ActuatorPwm` 或板级 TIM/GPIO。
- `SOURCE_CALIBRATION` 保持独立来源：开环使用 normalized axes，增益验证使用已有 speed/yaw-rate 模式及本层真实 PI。Commander、控制器、PWM 和 watchdog 共用精确开/闭环安全投影，闭环只打开 Velocity/Rates，不伪装成 Mission。
- `RoverDifferentialCalibration` 在完整 Disarmed 快照中独立锁存固定圆、入场正 RO_SPEED_LIM 与 MOT_THR_MAX 包络 E，以新鲜同设备 GNSS 复核停车余量；会话快照改变后永久失效。所有校准阶段纵向请求/闭环输出 [0,E]、转向 [-1,1]、最终每轮 [-E,E]；保留 0.15/s 末端 slew、TTL、速度/加速度门禁。
- `rover_control_status` 是本层产生的内部 uORB 反馈，包含请求/session/参数代次、真实 PI 设定/反馈/积分及最终执行器均值/差分。消费者应用确认与“闭环配置可运行”分开，允许旧零增益回滚到导航未就绪状态。
- 校准纵向请求/速度一律非负，FWD→CW→CCW；原地转向仍允许左右轮反向且保留换向等待。反馈中的未零区化速度和混控/整形/slew/Arm ramp/安全限制原因继续服务辨识；后端是否真正应用及应用时间由 actuator_output_status 单独提供。


## 头文件实现边界

RoverControlValidation.cpp 持有参数有效性判断，头文件只声明共享接口；控制层和执行器层各自的安全锁存继续独立。 统一审查与验收见 docs/HEADER_IMPLEMENTATION_SPLIT_ZH.md。

校准开环的纵向请求使用 [0,E] 包络坐标，差速器入口除以 E 转成原有 [0,1] 整形输入，避免 E<1 时重复缩放为 E²；速度 FF、响应剖面及辨识的整形前反馈仍使用差速器标准坐标。闭环 PI 保持标准坐标，纵向输出受 [0,E] 限制；两条路径的最终轮端均受同一冻结包络和末端 slew 约束。

Manual 电机行为按 APM 参考基线核对：先对操作者两轴同比缩放，后续在 [-1/ASYM,1] 中限制转向可行范围，仍允许急转内轮反转。APM 的“MIN 后 EXPO”顺序及左右独立换向等待保持一致；本地没有同向弧线限制、前后切换强制清零或双轮共同换向等待。导航零纵向停车、0.15/s 校准末端 slew、Arm ramp 和冻结包络继续有效。细节及 QGC 诊断见 modules/motor/README.md。
