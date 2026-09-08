# Rover 自动校准：一次 Arm、自动写回与固定圆形边界

## 当前实施状态（2026-09-08，U1–U5）

本节取代 R1–R5 的逐段人工 Arm 和固定 1.5 m/s 操作说明。旧记录折叠保留，只作历史证据。

U1–U5 本地实现已完成：一次 Arm 会话授权、冻结速度/输出策略、双层 GNSS 围栏、指定阶段低速倒车、统一 IMU 稳定零偏，以及电机 slew 有限比较、前馈/运行参数/控制增益/导航策略的关联 RAM 事务。Windows 发布目标和生成/架构门禁已通过，最终制品见第 7 节；这不是已经交付的车辆标定或运动安全证明。

| 检查点 | 实现内容 | 验收状态 |
|---|---|---|
| U1 一次 Arm | Commander 独占 session 授权，内部 Disarm/正常 preflight/继续；外部 Disarm 在提交窗口也取消 | 源码定点审查；实板竞态待验收 |
| U2 冻结策略 | 正巡航优先，0/旧 -1 采用兜底速度；独立捕获固定圆和停车预算 | 权威生成、全链构建与架构通过；停车能力待实板证明 |
| U3 倒车与观测 | 指定负向阶段、原始未零区反馈、生产限制器原因 | 保留 TTL/PWM/watchdog，车辆反向及停车待验收 |
| U4 IMU/满输出 | 实际前端快照、EKF 稳定偏置、残差确认；独立渐进前向命令端点探测 | 不测 RPM，不保证达到端点 |
| U5 关联整定 | 32 槽原始快照、SLEW 有限比较、重新 FF/PI/Heading/转驱/路径验证、自动保存/回滚 | 源码/生成/架构/Windows 制品通过；实际整定效果与掉电验收待执行 |

没有新编码器/电流反馈依赖，没有“确认增益”按钮或参数。代码、静态门禁和 Windows 构建不能替代 QGC、板卡、掉电持久化或车辆性能验收；本轮不刷机、不连接串口、不触发车辆动作。

## 1. 操作流程与一次 Arm 授权

1. 车体放在已知水平基准，保持 Disarmed。设置真实轮距、PWM 功能/方向/端点，准备有效 RC、Kill 和符合当前载荷/地面的停车距离上界。
2. 设定 `RO_CAL_RADIUS`、`RO_CAL_STOP_D`；核对入场 `RO_SPEED_LIM` 或兜底 `RO_CAL_VMAX`。停车能力未知时保持 `RO_CAL_STOP_D=0`，固件只允许可用静态项。
3. 从支持 Standard Modes 的 QGC 显式选择 `Auto Calibration`，或使用已有 RC 模式槽选择 External1。已经 Armed 的进入请求拒绝；选择模式本身不授予运动权限。
4. 固件只在入场捕获一次全球圆心，先完成 Level 和消费者/保存确认，再核对 RTK、RC、输出与运动安全条件。首次运动前，`[autocal]` 报告速度上限、驱动输出上限、输出策略、半径和停车上界。
5. 按提示首次人工 Arm 一次。正常阶段之后自动停车、内部 Disarm、参数事务、消费者确认，再由 Commander 正常检查并内部 Arm。无需逐段拨动 Arm，也无需批准候选参数。
6. 结束自动停车，读取 `SUCCESS/PARTIAL/FAILED/CANCELLED`、已保存/不可完成阶段；实际参数从 QGC 参数页读取，详细阶段、限制原因、RAM 代次和观测从 ULog 查看。

授权只存在于当前 Commander session：

- 协调器只能提出继续请求，不能创建授权，不使用 force-arm 或私有 MAVLink 命令。
- 每次内部 Arm 都检查明确等待状态、同 session/参数计数、维护互锁、消费者、RC 与执行器条件；本轮外部 RC/命令和最终安全评估先于续行。
- 外部 Disarm 即使发生在内部已经 Disarmed 的参数提交窗口，也立即取消整场会话。Kill、RC loss、模式切出、超时、定位/执行器/调度故障同样撤销。
- 重启、重连和 RC 恢复不重建已撤销授权。既有 GCS 重连本身不新增导航动作，不能替代有效 RC/Kill。
- 正常阶段外不自动重试运动；回滚无法确认时保持 FAILED 和禁 Arm/保存互锁。

总体顺序：

```text
Disarmed 显式进入并固定圆心
→ Level/保存 → 首次人工 Arm → RTK 往返标定/重融合
→ 前馈与 RTK 约束磁学习（磁可选）
→ EKF 稳定 IMU 零偏/前端与残差确认
→ 未零区化噪声、多档正反响应、双向转向、可选前向 FULL
→ 关联 RAM 候选 → 必要的新整形响应/前馈复采
→ ARX/四项 PI → Heading → 转驱 → 有条件 RAM 路径比较
→ 整组自动保存或回滚 → 停车结束
```

## 2. 入场速度、输出和固定圆合同

参数权威源为 `module_rover_control_params.yaml`，目录/ID/Metadata 经正式工具生成。

直线期望长度由 `AutoCalibrationMode::kPreferredStraightDistanceM` 内部常量给出；实际路径仍按固定圆的剩余空间缩短，空间不足时拒绝对应阶段。

| 参数 | 默认/范围 | 当前语义 |
|---|---|---|
| `RO_SPEED_LIM` | 新默认 0；Metadata -1–100 m/s | 正值为入场巡航上限；精确 0/旧 -1 表示未配置；其他负值或非有限值拒绝动态校准 |
| `RO_CAL_VMAX` | 1.5；0.1–100 m/s | 仅未配置巡航时使用的独立试验速度上限，不是无限速 |
| `RO_CAL_RADIUS` | 20；1–100 m | GNSS 定位参考点的固定水平圆半径 |
| `RO_CAL_STOP_D` | 0；0–100 m | 请求停车到实际停稳的可信保守最大行驶距离；0 禁止动态校准 |
| `RO_CAL_THR_MAX` | 0.30；0.10–0.40 | 普通校准归一化纵向上限 |
| `RO_CAL_TURN_MAX` | 0.25；0.10–0.35 | 普通校准归一化转向上限 |
| `MOT_THR_MAX` | 保留已有值 | 入场冻结的最终驱动输出上限，不自动提高 |

```text
RO_SPEED_LIM > 0: V_session = entry RO_SPEED_LIM，普通低输出策略
RO_SPEED_LIM = 0 或 -1: V_session = entry RO_CAL_VMAX，另申请前向 FULL 探测
```

事务随后自动改变 `RO_SPEED_LIM` 只影响待验证运行范围，不能改变本会话的 `V_session`、输出策略、圆心和停车预算。外部并发改参中止会话；内部合法写入按事务所有权/消费者代次单独管理。Level 记录起始参数计数与自身实际写入增量，不把外部修改吸收为内部成功。

入场圆心要求新鲜、同设备 RTK Fixed 定位，年龄 ≤300 ms、`0<eph≤0.15 m`。没有可靠入场定位，只执行可用静态项；定位恢复后必须退出重进。Level、Disarm、RTK 重锁、磁/IMU 初始化、增益试用不会重设圆心。EKF 参考或 reset 变化会使当前动态观测失效，不移动全球圆。

协调器和 RoverDifferential 各自锁存安全快照，独立检查原始 GNSS。工程余量：

```text
M = 0.5 m + 3 × (eph_center + eph_now)
    + V_session × (0.30 s + 0.10 s + 0.01 s + 0.01 s + sample_age)
    + RO_CAL_STOP_D
R_work = max(0, RO_CAL_RADIUS - M)
```

300 ms 为感知/传输预算，100 ms 为请求 TTL，两个 10 ms 为控制/PWM 周期预算；均需实板测量验证。停车上界必须覆盖本次允许速度、驱动输出、载荷、地面和执行器停机语义，不能由 `RO_DECEL_LIM` 命令值推导，也不能在提高速度/尝试满输出时沿用不适用的证明。

所有直线、倒车、掉头、磁转动、闭环及 RAM 路径共用该圆。剩余空间不足提前停车/跳过；围栏介入数据不计辨识，不扩大半径、不自动返回圆内。越界、定位失效/设备切换撤销有效动力请求。

普通每轮输出不超过 `min(0.40,冻结 MOT_THR_MAX)`，最终变化率始终 ≤0.15/s；地速 ≤`V_session`、yaw-rate ≤0.6 rad/s、实际线/角加速度 ≤3 m/s²、3 rad/s²。每个运动周期 180 s、全会话 600 s 不变，等待、重融合、应用、保存也消耗总预算。

### 独立 FULL 与低速反转

- 只有 `PROFILE_FULL` 且入场未配置巡航时允许高于普通输出。先建立 RTK 航向/基本响应，前进直线逐档加输出，目标是冻结驱动命令端点；不直接发最大阶跃。
- FULL 不安排倒车或高功率原地转向。方向异常、速度软保护、定位/余量不足即停止探测；正常减速保持 FULL 状态直到真实发布轮端归零才撤回高输出许可，下游许可下降时历史超限则 fail-closed。
- 结果只报告“达到允许命令端点”或“输出区间部分覆盖”。没有 RPM/编码器，不报告实际最大转速、堵转扭矩或左右轮 RPM 已归零。
- 指定倒车段使用 `min(0.3,V_session)` 地速和普通输出上限；默认具备正反转能力，但保留 `MOT_REV_DELAY` 和停车确认，不安排受载瞬时反转。
- FULL 端点不能作为闭环调节余量，也不直接成为新巡航值；运行候选受后续真实闭环验证范围约束。

圆只约束定位参考点，不包含整车轮廓。在 GNSS 错报、执行器/制动失效或停车证明失效时，软件不能保证物理位置绝对不越界；仍需隔离场地和急停。

## 3. 自动参数范围与不能完成的项目

| 项目 | 参数 | 自动处理及边界 |
|---|---|---|
| 水平 | `SENS_BOARD_X_OFF/Y_OFF` | 已知水平基准上求细旋转，保留 Z；不能自动区分真实坡度与安装误差 |
| RTK | `GPS_YAW_BASELINE`、`GPS_YAW_OFFSET` | 稳健基线、往返航向偏置，接收机应用及 GNSS yaw 重融合确认；`EKF2_GPS_CTRL` 仅在原配置上启用既有 yaw 位 |
| 前馈 | `RO_MAX_THR_SPEED`、`RO_YAW_RATE_CORR` | 真实多档响应与双向一致性；整形改变后重新拟合，不能让 PI 积分补偿旧 FF 冒充重标定 |
| 内环 | `RO_SPEED_P/I`、`RO_YAW_RATE_P/I` | 四项同代 RAM 应用，真实速度、双向角速度和低速反向验收 |
| Heading/转驱 | `RO_YAW_P`、`RD_TRANS_TRN_DRV`、`RD_TRANS_DRV_TRN` | 内环通过后验证，保留滞回、死区兼容、移动→停车→原地转向→前进的双方向闭包 |
| 运行速度/角速 | `RO_SPEED_LIM`、`RO_YAW_RATE_LIM` | 从实际观测范围和输出余量产生保守候选，可初始化未配置值；不宣称验证更大范围 |
| 加减速 | `RO_ACCEL_LIM`、`RO_DECEL_LIM`、`RO_YAW_ACCEL_LIM`、`RO_YAW_DECEL_LIM` | 非零升降响应估计后，通过真实内环设定斜率和闭环验收；不可观时仅可保留原合法值 |
| 零区 | `RO_SPEED_TH`、`RO_YAW_RATE_TH` | 从未零区化反馈和真实精度取候选，并验证低速/退出行为；不把零区后的零噪声当证据 |
| 导航策略 | `RO_JERK_LIM`、`RO_SPEED_RED`、`PP_LOOKAHD_GAIN` | 固定 RAM 路径有限候选比较；需要字段真实生效、改善超过 10% 和测量噪声界，未观察到则保留原值或关联组回滚 |
| 等效电机整形 | `MOT_THR_MIN`、`MOT_THR_EXPO`、`MOT_THR_ASYM` | 起动括界、多档正反曲线、公共等效模型；全局使用范围覆盖及新整形复采通过才有保存资格，局部数据不修改 Manual 共用参数 |
| 电机 slew | `MOT_SLEW_RATE` | 原值与一个合法 0.8/1.25 倍邻近候选实跑比较，要求真实 slew 未被安全层遮蔽、相同激励、双向升降/输出范围、误差改善及超调/振荡门槛；否则保留原值 |
| 磁硬铁 | `CAL_MAG0_ID`、`CAL_MAG0_XOFF/YOFF/ZOFF` | RTK 标定和 GNSS yaw 实际融合之后运行；保留 scale/rotation，磁失败不撤销独立成功 RTK/前馈 |
| IMU 稳定零偏 | `CAL_GYRO0_ID/XOFF/YOFF/ZOFF`、`CAL_ACC0_ID/XOFF/YOFF/ZOFF` | 按 EKF valid/stable/方差与设备筛选，实际前端快照计算、统一提交、重新残差确认；不改变 accel scale |

关键能力边界：

- `MOT_THR_MIN/EXPO/ASYM/SLEW_RATE` 是 Manual 等消费者共用的全局值。普通低输出和 ≤0.3 m/s 倒车一般不能覆盖全反向输出域；即使前向 FULL 达端点，也不构成全局正反覆盖。因此这些项目经常保持原值，不能把“有候选算法”说成每车都可自动完成。
- 特别是默认 `MOT_SLEW_RATE=1`，校准末端仍为 0.15/s，因此不能从这些数据识别默认电机 slew；低 MIN、ASYM≈1 与普通反向请求 ceiling 也不能覆盖驱动端点。这些默认条件下明确不自动写全局整形/SLEW，不绕过安全层制造观测。SLEW 未改善时先恢复原值并重新采集；恢复复采预算不足则关联组回滚，不留下未验证候选。
- 保存 MIN 需要非零输出“不动/能动”括界，不能从零输出静止点猜起动阈值。EXPO 是整车/地面等效响应，不是独立电机电气标定；ASYM 是公共前进/倒退补偿，不是左右电机独立效率。
- `RO_JERK_LIM` 只优化导航减速轨迹，不是所有工况物理 jerk 极限。初始化的正 jerk 也必须出现真实生效证据，不能因为成为 RAM baseline 就跳过验收。
- IMU 使用 `offset_acc+=Rᵀ*bias/scale`、`offset_gyro+=Rᵀ*bias`，基准来自真正生效的校正。旧设备遗留比例与 active identity 不一致时，不只换 ID 意外启用旧 scale；不合格组保留。
- EKF 继续估计位置、速度、姿态、偏置和协方差；不将动态状态保存成固定校准值。`EKF2_MAG_DECL` 保持 volatile，只报告 RAM 更新。

下列参数/能力不自动改：

| 类别 | 对应参数/范围 | 原因 |
|---|---|---|
| 物理轮距 | `RD_WHEEL_TRACK` | 无独立轮速/轮角反馈时，与滑移、转向有效系数耦合，不能把有效轮距当物理尺寸 |
| 安装杆臂 | `EKF2_GPS_POS_X/Y/Z`、`EKF2_IMU_POS_X/Y/Z` | 平面运动不能完整分离三轴安装位置，需要真实测量/外部几何基准 |
| PWM 映射与端点 | `PWM_S1…S6_FUNC/MIN/CENT/MAX/REV` | 无独立电调/RPM反馈，不能从整车运动唯一确定电气中位、端点和接线 |
| 保留的电机安全配置 | `MOT_THR_MAX`、`MOT_REV_DELAY`、`MOT_ARM_RAMP` | 不自动提高驱动上限、取消换向保护或启动缓升 |
| Manual 专属混控 | `RD_STR_THR_MIX`、`RD_REV_STEER` | 自动校准使用非 Manual 请求，不用自动路径替代人的手动操纵策略 |
| 保留导航/超时边界 | `PP_LOOKAHD_MIN/MAX`、`NAV_ACC_RAD`、`RO_CMD_TIMEOUT` | 不通过扩大容差、放宽超时或前视上下限制造通过 |
| 完整磁软铁/轴向比例 | 保留 `CAL_MAG0_XSCALE/YSCALE/ZSCALE` 和 `CAL_MAG0_ROT`；当前没有完整九项矩阵参数 | 水平转动不能完整观测三维软铁矩阵，不创造不存在的参数 |
| EKF 调优与融合策略 | 如 `EKF2_GYR_NOISE`、`EKF2_ACC_NOISE`、`EKF2_GPS_DELAY`、`EKF2_GPS_P/V_NOISE`、`EKF2_GPS_P/V_GATE`、`EKF2_MAG_NOISE/GATE`，以及健康/融合门限 | 自动校准只检查健康与已约定 GNSS yaw 前置配置，不放宽估计器门限掩盖故障 |

## 4. 观测、关联事务与工程验收门槛

- 复用锁定 PX4 v1.17 固定容量 ARX/RLS 六个一阶延迟候选，采用 Rover 并联 PI 设计，不复制飞行器激励或删掉显著 D 项冒充 PI。
- 噪声窗口必须连续 ≥5 s、至少 50 个唯一速度样本，失锁/移动/采样间断重置；设备、EKF 参考和 reset 代次一致。动态辨识、磁稳态与 IMU 残差是不同窗口。
- 速度采样间隔至少 100 ms，角速度至少 20 ms；受安全限制的 ARX 窗口失效，不跳样拼接固定周期模型。正常 MIN/EXPO/ASYM 整形与保护限幅分开报告。
- 一阶模型要求稳定、正增益、持续激励、残差/方差、双方向一致性和输出余量。PI 使用 `P=tau/[K*(lambda+delay)]`、`Ti=tau+delay/2`、`I=P/Ti`，并按真实整形输入坐标与可用余量缩放。
- 真实闭环至少要求超调 ≤20%、稳态误差 ≤阶跃 10% 或合理噪声界，没有持续振荡、持续饱和、异常积分或安全干预。非零下降阶跃不能用零请求清 PI 的停车动作代替。
- RAM 路径是同一固定圆内闭合瘦三角，复用生产 SegmentGuidance；候选先回固定入口并对齐同一首边，再计分，不覆盖 Mission。每字段至少 30 个未遮蔽敏感样本；Jerk/SpeedRed 比较完整生产速度请求，PP 比较 Heading slew/clamp 后的 rate，不把下游遮蔽当可观。
- 参数组固定 32 槽，当前关联 24 项；最初 old 快照不随候选修订改变。RAM 临时值期间同时暂停自动/显式持久化。整形改变重新采响应/FF，然后再验证 PI、Heading、转驱和路径，最后统一保存。
- 选回早先候选必须重新应用、消费者确认并实跑同代确认圈；初始化运行值也相对最初快照检查生效。当前组失败只回滚该组，独立已保存 Level/RTK/磁/IMU 等保留。
- 旧零 PI/零巡航/零 Heading 可回滚为“导航未就绪”；不能要求旧值满足新闭环条件才确认恢复。无法确认恢复则 FAILED/禁 Arm。
- 剩余空间、时间、噪声、输出区间或激励不足返回 PARTIAL/未完成，不强行提交。600 s 不保证任意车辆能完成所有可选项，不自动重建圆心续跑。

上述门槛是初始工程验收条件，不是当前车辆已经测出的性能或可保证的整定效果。

## 5. 缺外设与降级

| 条件 | 结果 |
|---|---|
| 无可靠入场 GNSS / 单天线 / 双天线未固定 | 可用 Level 仍运行；不建立动态会话。定位后来恢复必须退出重进 |
| 无 RC、PWM 未配置或输出后端不可用 | 不提前阻止 Level；不能获得运动授权 |
| 停车上界 0、有效半径/直线不足 | 不运动或部分完成，不扩大圆 |
| 无磁力计 | 磁阶段跳过；RTK/估计器依赖满足的后续控制整定继续 |
| 磁已出现后失效/干扰/偏置不可观 | 该项失败或回滚，先前独立成功组保留；不把失效设备记成未安装 |
| IMU 稳定偏置不满足 / 仅部分轴组有效 | 保留不合格组，不伪报全量 IMU 完成；可用的后续响应阶段仍按门禁执行 |
| 无可靠姿态、EKF 水平位置/速度或 reset | 静态基础失败或动态观测失效，不降低健康门禁 |
| 无 SD | 不以日志可用替代控制授权；缺日志无法形成实板验收证据，持久化失败仍触发回滚 |
| 全局整形、路径字段、加减速不可观 | 保留合法旧值或关联回滚，报告具体未完成阶段 |

## 6. msg、MAVLink 2 与官方 QGC

`.msg` 是板内 uORB 合同，不是 MAVLink XML；QGC 不直接解码本地 msg。所有 ID、字段、枚举、模式/日志目录经权威输入生成，没有私有 MAVLink wire 消息或人工增益确认命令。

| 本地合同 | 此次扩展 / 地面站影响 |
|---|---|
| `AutoCalibrationRequest.msg` | 内部继续/Disarm/Level/退出请求、session、参数计数，不发给 QGC |
| `AutoCalibrationStatus.msg` | 冻结圆心/速度/输出、授权诊断、阶段、RAM 验证与保存状态，内部/ULog 可见 |
| `RoverControlStatus.msg` | 未零区化反馈、真实 PI 设定及限制器原因；无对应私有遥测流 |
| `RoverMotionRequest.msg` | 受限 SOURCE_CALIBRATION 开环/真实 PI 闭环与指定倒车，不伪装成 Navigation |
| `SensorCalibrationRequest/Status.msg` | QGC/AUTO owner 和受控参数写入计数，内部 Level 不发外部 Sensors 的终态 |
| `RtkHeadingStatus.msg` | 原始双天线/速度历元与解质量，不要求 QGC 定制解析 |

对外仍为标准 AVAILABLE_MODES/CURRENT_MODE、PARAM/PARAM_EXT、Component Metadata、命令/ACK 和 `[autocal]` STATUSTEXT。现有对照记录为官方 QGC 5.1.3；实际 UI、重连与消息时序仍待板端验收。

| 能力 | 支持/影响与整改 |
|---|---|
| Standard Modes 发现与当前/意图映射 | 固件已有标准服务；未支持的旧 QGC 可能显示 External1，升级支持 Standard Modes 的官方版本，不新增私有模式命令 |
| 半径/兜底速度/结果 | 官方参数页可配置/读取；无额外确认按钮 |
| Sensors 反馈归属 | 已在固件隔离 AUTO Level 和 QGC 外部请求，高重要性；避免重复 `[cal]` 终态 |
| 重连/等待/保存/回滚提示 | 周期标准 STATUSTEXT 重报，长文本按标准分片；重连不恢复撤销授权 |
| 专用校准向导、实时圆覆盖、结构化整定图 | 官方 QGC 无本项目专用 UI，影响便利性，不影响板内安全/自动保存。当前用消息面板、参数页和 ULog；若需要可视化另行授权 UI 开发 |

## 7. 工作区与当前非测试验证

正式执行位置：`E:\freertos\H743_FreeRTOS`，分支 `feature/dima-phase3`。共享工作区本来包含大量日志、存储、RC、启动和链接布局工作；本轮过程中其他工作流更新了 HEAD。它不是单主题最小 diff，不能把整个 dirty tree 归于自动校准，也不能为缩小统计回退用户改动。本功能按授权、围栏、反馈、事务、数学和文档分主题收敛，保留生成权威；没有提交/推送。

不新增或修改测试文件、框架、runner、fixture、mock、test-only API、Host Test、SITL 或仿真，也不运行它们。正式非测试入口：

```powershell
Set-Location E:\freertos\H743_FreeRTOS
& C:\Users\master\.local\bin\make.cmd -j4 NO_COLOR=1 `
  dima_rover uorb-generated-verify mavlink-generated-verify `
  parameter-metadata-verify logger-generated-verify
git -c core.safecrlf=false diff --check
```

最终源码的生成、Windows 发布目标、ELF/签名/Factory 布局已通过；下面记录本次制品大小/哈希。旧 R1–R5 和中间镜像不是本轮最终证据。

### 最终验收记录

2026-09-08，Windows 原生执行上方完整命令，先完成 `[35/35]` 编译/发布，再在源代码冻结后重复执行同一命令并完成 `[8/8]`，两次 exit 0。这是完整发布目标的依赖增量构建，不是 clean build；没有为验证删除共享 build。

- 使用项目缓存 Arm GCC 10.3.1；架构 `PASS - 454 first-party source files`。
- 权威生成合同：303 个参数、43 个 uORB schema、49 个 Logger Topic、8 种 Logger Profile 组合；四个独立生成/Metadata 校验均通过。
- MAVLink 仍从锁定的四个 XML 生成 230 个官方消息定义；固定上游 `CAMERA_TRACKING_STATUS_FLAGS_IDLE` 和 `MAV_BOOL_FALSE` 的两条 bitmask 提示仍存在，没有新增私有 wire 或修改上游 XML 来压掉提示。
- 向量 `0x08040400`，ELF、签名/Factory 布局通过，Application/MCUboot 未解析符号为空，MCUboot watchdog prepare/feed 链已链接。
- image digest：`6d9e481c5b1de611911aff9a33071899aa4c71aeaf5edb4341165b0bffa5bfe8`。
- 验收时 HEAD 为 `3ac342f808e847fe5b9c223ccb9457fec33f29e8`，109 个 tracked 差异、44 个未跟踪文件、暂存 0。HEAD 在整个实施过程中被其他工作流更新，本代理没有提交/推送。
- tracked `git diff --check` 为 0；逐一核对 44 个未跟踪文件，本功能文件无空白诊断。另一个并发工作流新增的 `docs/SDMMC_IDMA_CPU_ACCEPTANCE_ZH.md:144` 存在末尾空行提示，未为清理全树统计修改该无关文件。测试/框架/runner/fixture/mock/Host Test/SITL/仿真变更路径为 0。

| 最终制品 | 大小（bytes） | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 11133656 | `2bed9e72600e1eb56c6b6cb8b97262c8d6de593407455d6a6a27e0337c606f4b` |
| `build/H743_FreeRTOS.bin` | 638216 | `087ede89f4e6d1577e535a1a021f95ff07755f630610d272af343ec7e156ed55` |
| `build/H743_FreeRTOS_signed.bin` | 639391 | `a17350da7c85acc5db320787e8b5aa9d37772505078a60b464ef9f4ab14f2854` |
| `build/H743_FreeRTOS_factory.hex` | 1654986 | `a09ee7fdb87261a9ad006c8f07da0dd487980554a99c1f3a62d3ddf952681e83` |
| `build/mcuboot/mcuboot.elf` | 2684396 | `14e52c94c4e68c91e679a748f0ad8ad9e03d341134175722a653d6447565f54d` |
| `build/mcuboot/mcuboot.bin` | 48308 | `84b49887296b34822cefff28e9fc8b6b1d7b9251e1d44c3640f1ad4e8c073dcc` |

签名文件哈希只标识本次 ECDSA 签名样本，重签不承诺字节相同。Application `text/data/bss=625488/12688/571232` bytes。全工作树 Flash `638216/782336`（81.6%）、SRAM `590848/884736`（66.8%）、D1 `401408/524288`（76.6%）、D2 `183008/262144`（69.8%，余 79136 bytes）。这些是当前共享链接布局下的总占用，不是本功能独立增量。

只读 ELF/DWARF 检查得到 `sizeof(AutoCalibrationMode)=18600`、`sizeof(CalibrationParameters)=760`、`sizeof(MotorResponseProfile)=1744` bytes；协调器位于 D2 的 `ApplicationContext` 内，后者地址 `0x30006820`、大小 92064 bytes。读取类型尺寸时没有连接目标、运行程序或调用仿真。固定存储没有运行期扩容；共享链接布局的资源收益不能算成本功能优化。实际调度、栈、CPU、watchdog 和最坏停止时限仍须实板采集。

## 8. 授权后实板验收（全部待执行）

| 场景 | 必须取得的证据 |
|---|---|
| 首次 Arm / 内部提交窗口外部 Disarm | 正常阶段自动续行；外部取消任意窗口均彻底撤销，无未授权重启运动 |
| RC loss/Kill/模式切出/故障恢复/重启 | 旧授权不能复活，动力请求和 PWM 正确失效 |
| 正巡航、0、旧 -1、非法值 | 速度策略匹配；自动写回巡航不改变冻结围栏预算 |
| FULL/部分覆盖/回普通输出 | 缓慢提升、不越冻结驱动上限，提前速度/余量保护后报告部分覆盖；历史高输出不携带到普通段 |
| 倒车与换向 | 指定负向段、航向保持、原始地速、停车和 MOT_REV_DELAY 正确；Mission 行为未被改变 |
| Level/Disarm/RTK 重锁/参数试用/IMU 写回 | 圆心/设备/时间不变，新前端/EKF 代次后不混旧样本 |
| 无入场定位 / 小半径 / 停车上界未知 | 静态/部分完成，不补建中心、不扩大圆 |
| 接近边界/越界/定位异常/中途改参 | 提前停车或撤销动力，真实停止距离与延迟符合冻结配置 |
| 整形与关联 FF/PI/运行参数 | 候选覆盖和实际生效可追溯；保存时没有新整形加旧未验证前馈/PI 的组合 |
| PI/Heading/转驱/路径 | 超调、稳态、振荡、饱和、退出滞回、同入口候选公平性与敏感字段门槛通过 |
| 成功/失败/临时值掉电/回滚旧零值 | 自动保存无需参数确认；临时值掉电不保留，失败恢复原组；回滚无法确认保持 FAILED/禁 Arm |
| QGC 与资源 | 模式发现/反馈owner/长文本/参数刷新正确；D2、实际栈/调度/CPU、ULog 和前后参数快照足够 |

<details>
<summary>历史归档：2026-09-07 R1–R5（旧逐段 Arm、固定速度与旧制品，不用于操作）</summary>

# Rover 自动校准、增益自动写回与固定圆形边界

## 2026-09-08 扩展实施中

本轮已获准实现“一次 Arm、巡航速度优先、未配置时渐进满输出探测”，并扩展
IMU 零偏、运行参数及有条件整形/导航策略整定。检查点依次为 U1 会话授权、
U2 冻结速度/输出与双层围栏、U3 倒车及限制器观测、U4 零偏/响应/关联事务、
U5 生成/架构/Windows 制品与审查。当前代码正在按这些检查点修改；下方 R1–R5
是上一版本验收，不能作为本轮完成证据，尤其逐段 Arm 与固定 1.5 m/s 说明将被替换。

本轮同样不新增测试/框架/仿真、不手改生成物、不刷机。共享工作区另有启动、
链接布局、驱动等并发修改，全部保留；正式制品检查须避开其他正在运行的构建。

## 当前结论（2026-09-07，R1–R5）

当前源码已实现固定入场圆心、静态/运动依赖分离、RTK 后磁学习、真实闭环增益临时验证与自动保存、官方 QGC Standard Modes 发现。用户仅显式进入模式并逐段 Arm；Arm 是运动授权，不是候选参数确认。没有“确认增益”按钮、参数或私有命令。

实现与 Windows 静态验收不等于车辆已经校准成功：本轮未刷机、未连接板卡/QGC 串口、未进行车辆动作。下文“实板验收”仍需独立授权。文末折叠部分保留旧 T1–T16 历史证据，不能用作当前使用说明或制品清单。

| 检查点 | 当前实现 | 验收边界 |
|---|---|---|
| R1 固定圆 | 参数、入场全球圆心、停车余量、协调器规划与差速层独立定位检查 | 源码/生成/构建通过；真实停车上界与定位误差待场地证明 |
| R2 降级与 QGC | Level 不提前依赖 RC/PWM/双天线；标准模式发现；QGC/AUTO 反馈归属隔离 | 官方 QGC 5.1.3 源码核对；真实 UI 与重连待验收 |
| R3 事务与闭环 | 同组 RAM 候选替换/确认/最终保存；SOURCE_CALIBRATION 复用真实 PI；统一安全投影 | 取消、旧零增益回滚及保存暂停已静态审查；掉电与实板竞争待验收 |
| R4 自动增益 | 四项内环同组、Heading P 独立、RAM 路径有限比较；失败回滚、不可观保留 | 算法与状态机已实现；收敛/超调/跟踪性能未实车证明 |
| R5 发布门禁 | 权威生成、架构、Windows dima_rover 与制品检查 | 非测试静态门禁已执行；BOARD/QGC/VEHICLE PENDING |

## 1. 自动写回范围

| 项目 | 实际写回与限制 |
|---|---|
| 水平 | 在已知水平基准上保存板级 X/Y 细旋转；保留 Z。不是六面加速度计或陀螺仪全量校准，不能自动识别地面坡度 |
| RTK | 稳健统计 `GPS_YAW_BASELINE`、往返直线估计 `GPS_YAW_OFFSET`，确认接收机应用及 GNSS yaw 实际融合；`EKF2_GPS_CTRL` 仅在原配置上启用 yaw 位，保留其他位 |
| 车身前馈 | `RO_MAX_THR_SPEED` 与 `RO_YAW_RATE_CORR` 一组提交/保存；方向不一致、输出整形不可解释或激励不足时拒绝 |
| 磁硬铁 | RTK 参数保存且 GNSS yaw 连续融合后学习；有界 WMM bootstrap 只作 RAM 候选，最终验证后保存 `CAL_MAG0_ID` 和 X/Y/Z offset，保留原 scale/rotation |
| 内环增益 | `RO_SPEED_P/I` 与 `RO_YAW_RATE_P/I` 四项同组临时应用；真实速度及双向角速度验证全部通过后整组自动保存 |
| 航向增益 | 内环保存后验证 `RO_YAW_P`，双向角度响应、转向退出与 yaw-rate 死区兼容才保存 |
| 路径增益 | 最多比较原 `PP_LOOKAHD_GAIN` 及其 0.8、1.25 倍；有界、安全且可观候选之间比较，改善不足 10% 或不足定位噪声界保留原值 |
| 不写的量 | 物理轮距、GNSS 相对重心杆臂、PWM 端点、运行限制、完整磁软铁矩阵、EKF noise/delay/gate；不能用平面运动猜测这些量或放松门限掩盖故障 |

因此不是电机、导航、EKF 的“所有参数一键全量标定”。只有明确可观、消费者确认、实际验证成功的参数组会被自动保存；其余保持配置并说明未完成原因。

## 2. 配置与固定圆边界

参数唯一权威源为 `Dima/middleware/parameters/definitions/module_rover_control_params.yaml`；参数目录与 QGC Metadata 由现有工具生成。

| 参数 | 默认/范围 | 使用语义 |
|---|---|---|
| `RO_CAL_RADIUS` | 20 m；1–100 m | 本次校准允许的固定水平圆半径 |
| `RO_CAL_STOP_D` | 0 m；0–100 m | 从停车请求到实际停稳，定位参考点行驶距离的可信保守上界；0 表示未知，禁止全部动态校准 |
| `RO_CAL_THR_MAX` | 0.30；0.10–0.40 | 归一化纵向请求上限，开/闭环都遵守 |
| `RO_CAL_TURN_MAX` | 0.25；0.10–0.35 | 归一化转向请求上限，开/闭环都遵守 |

停车距离必须覆盖当前载荷、地面、允许车速、转动时天线运动和实际执行器停止语义，不能由 `RO_DECEL_LIM` 命令限值替代。此项属于安全前置配置，不是参数候选确认。

入场时仅捕获一次同设备 RTK Fixed 全球坐标，样本年龄不大于 300 ms、水平精度 `0 < eph <= 0.15 m`；记录样本时间、设备与误差。圆心独立于 EKF 本地原点/yaw reset；Level、阶段 Disarm、RTK 重锁、磁 bootstrap 和所有增益候选共用同一圆。入场没有有效定位则本会话只做静态项，定位恢复后必须退出再进入，不能补建或移动旧圆心。会话内冻结配置，外部参数变动中止会话，新值下次进入生效。

用 WGS84 椭球局部 N/E 距离计算当前定位点距圆心的距离 `d`。当前工程余量公式为：

```text
M = 0.5 m + 3 × (eph_center + eph_now)
    + 1.5 m/s × (0.30 s + 0.10 s + 0.01 s + 0.01 s + sample_age)
    + RO_CAL_STOP_D
R_work = max(0, RO_CAL_RADIUS - M)
```

其中 300 ms 是接收机/传输预算，100 ms 是请求 TTL，两个 10 ms 分别覆盖控制/PWM 周期；该链路预算需要实板核实。使用地速硬上限计算延迟距离，而不是某拍较小的速度。定位精度的三倍只是工程保守量，不代表对 GNSS 错报的数学保证。

协调器在 `R_work` 内规划所有运动；直线额外预留 0.5 m，缩短后不足 5 m 则不启动。RAM 路径与各候选共用同一固定圆，不覆盖用户 Mission。差速控制器另行锁存中心/设备/半径/停车距离/用户输出 ceiling，直接订阅并复核新鲜 GNSS，而不是只相信协调器的 `motion_allowed` 或“圆内”标志。

到达停车余量边界就撤销运动请求；越界、定位失效/设备变化直接安全停机，不能自动返回圆内。围栏干预优先于拟合，当前验证失败/回滚，相关样本不能导致校准成功。继续保留每轮电机绝对值 0.40、变化率 0.15/s、地速 1.5 m/s、yaw-rate 0.6 rad/s、线/角加速度 3 m/s²、3 rad/s²、每次运动周期 180 s 和全会话 600 s 上限。

圆约束的是 GNSS 定位参考点，不包含车头/车尾/四角。在可信定位、有效执行器和真实停车上界不成立时，软件不能保证物理位置绝不越界；场地隔离与急停仍然必要。

## 3. 使用和校准顺序

1. 在已知水平基准放置车体，确认 Disarmed。准备实际测得的轮距、正确 PWM/方向、有效 RC/Kill 和可信停车上界；这些只决定后续能否运动，不提前阻止可用的 Level。设置半径和停车距离，不在会话中改参数。
2. 从支持 Standard Modes 的官方 QGC 选择 `Auto Calibration`，或把一个 RC `COM_FLTMODE` 槽显式设为生成枚举的 External1（内部值 23）。默认 `-1` 槽无动作；已经 Armed 的进入请求拒绝，不自动 Arm。
3. 固件固定圆心、完成 Level、消费者确认并保存，再检查运动依赖及 RTK 基线。缺少动态条件时保留 Level 并报告 `PARTIAL`。
4. 按 `[autocal]` 提示显式 Arm，执行圆内往返直线和掉头；停车/阶段 Disarm 后保存 RTK。连续确认 GNSS yaw 融合，再提示下一次 Arm 执行双向转动、完成速度/转向前馈组及可用的磁学习。
5. 磁 bootstrap 如有需要，会停车、临时应用、确认后另提示一次 Arm。动态辨识窗口和磁稳态学习窗口分开；磁失败保留已经保存的 RTK/前馈，不阻止依赖已满足的控制整定。
6. 自动进入内环辨识：两个相反行驶方向的速度激励、顺/逆转向激励。停车/Disarm 后四个 P/I 一起临时应用 RAM，等待真实消费者确认，再按提示 Arm 进行速度和角速度闭环验收。无需批准或填写任何候选增益。
7. 速度与双向角速度都通过后自动保存四项；继续按提示 Arm 验证 Heading P、再比较圆内 RAM 路径候选。每次重新应用候选都重新确认消费者；选回早先候选时还需重新跑确认路径，不能用最后一次试验授权另一个参数代。
8. 结束保持停止，通过 `[autocal]` 查看 `SUCCESS/PARTIAL/FAILED/CANCELLED`、已保存/不可完成阶段。最终实际值从 QGC 参数页读取，详细边界、增益代次和验证指标从 ULog 查看。成功组无需再按“保存增益”。

每个参数组遵循：

```text
有界辨识/候选 → 停车/Disarm → RAM 临时应用 → 消费者确认
→ 显式 Arm → 实际闭环验证 → 停车/Disarm → 自动保存或恢复最初旧值
```

临时期间暂停自动和显式参数持久化，快照始终保留最初旧值。用户 Disarm、Kill、RC loss、模式切出、超时、参数并发变化或围栏触发会停止运动并回滚当前组；已保存的其他组不撤销。旧零增益恢复后允许导航未就绪；回滚无法确认则锁存 FAILED、保持禁 Arm/保存互锁，不能继续新一轮运动。QGC USB 重连本身不替代 RC loss 或运动授权；有效 RC 与急停必须始终保留。

## 4. 增益模型和可观测性门禁

- 复用锁定 PX4 v1.17 的固定容量 ARX/RLS：六个一阶延迟候选，速度样本间隔至少 100 ms、角速度至少 20 ms，重复样本不计数。模型必须稳定、正 DC 增益、持续激励充分，归一化残差不超过 20%，噪声和系数方差通过门禁，双向模型一致性误差不超过 20%。
- RLS 输入使用最终归一化执行器均值/差分，输出来自真实 EKF；前者不是物理轮速反馈。对同批数据检查整形前后斜率 `s=sum(u_pre*u_post)/sum(u_pre²)` 为正且在 0.1–10，线性残差 RMS 不超过 post 输入能量的 5%；候选 P/I 按 `/s` 换回真实控制器坐标。
- 对一阶模型直接设计 Rover 并联 PI：`lambda >= max(requested, 3*tau, 5*delay, 1 s)`、`P=tau/[K*(lambda+delay)]`、`Ti=tau+delay/2`、`I=P/Ti`；按真实用户 ceiling 扣除前馈后的输出余量缩放。不从带显著 D 项的其他机型候选中删掉 D 冒充有效 PI；当前一阶模型不能解释的系统拒绝整定。
- 实际闭环使用 `SOURCE_CALIBRATION + MODE_SPEED_YAW_RATE` 和 `RoverDifferential` 的生产 PI。速度/角速度两档阶跃验证误差下降、超调不超过 20%、稳态误差不超过阶跃 10% 或三倍测量噪声界；持续振荡、持续饱和或异常积分失败。Heading 还满足转向退出/死区，最终角误差使用实际退出角容限，不放宽瞬态振荡判据。
- 整定不擅自降低声明范围：当前要求 `RO_SPEED_LIM` 在 0.3–1.5 m/s、`RO_YAW_RATE_LIM` 换算后在 0.10–0.6 rad/s，且阈值/加减速度/输出余量本来有效。角速度限制参数仍采用原 PX4 deg/s 元数据，P/I 使用 rad/s 误差，不能重复转换。范围超出安全能力则不整定，不改运行限制来制造成功。
- 路径仅比较原值及 0.8、1.25 倍合法候选，复用 `SegmentGuidance`；每条路径要求足够有效误差样本及至少 30 个未被 Lmin/Lmax 钳制的可观样本。横向误差大于 1 m、持续饱和/振荡或围栏介入失败。改善还必须超过三倍当前定位 eph；确认圈必须保留改善与可观性。
- 600 s 总预算包括等待 Arm 和所有静态/运动/确认阶段；剩余预算不足不会启动新验证段。默认半径和任意车辆动力学不保证能跑完全部可选候选；空间、激励、有效样本或时间不足报告 `PARTIAL`，保留原值及已保存组，不宣称未覆盖工况已整定。

以上是初始工程验收标准，不是本车已测得的性能。

## 5. 缺外设与降级

| 条件 | 本次会话结果/边界 |
|---|---|
| 无可靠入场 GNSS、仅单天线、双天线未固定 | 可用静态 Level 仍可运行；没有圆心或 RTK 航向依赖不能运动。定位后来恢复要退出重进 |
| 无 RC、未配置 PWM、输出后端不可用 | 不提前阻止 Level；动态 Arm 门禁不通过，不绕过 Commander |
| `RO_CAL_STOP_D=0`、圆内有效直线不足 5 m | 不执行自动运动，保留已完成静态结果 |
| 无磁力计 | 磁阶段跳过；RTK/前馈已成功且 EKF/控制依赖满足时继续增益 |
| 磁设备存在但失效、磁场异常或偏置不可观 | 不把失效设备当未安装；磁候选拒绝/回滚，先前成功 RTK/前馈保留，可满足依赖的后续增益继续 |
| 无新鲜 IMU/合法姿态或水平校准失败 | 静态基础无法建立，结束并说明失败，不能宣称仍能全量校准 |
| EKF 只有 yaw、无有效全局关联水平位置/速度 | 基础成功项可保留；真实闭环整定不 Arm，不降低健康门禁 |
| 无 SD | 动态控制不以 ULog 可用为授权条件；无 SD 日志不能提供验收数据。参数内部持久化失败仍使提交失败/回滚，不能假报保存 |
| 增益/路径不可辨识或预算不足 | 当前组保留/回滚，返回 PARTIAL；没有人工填数补成“成功”的步骤 |

## 6. msg、MAVLink 2 与官方 QGC 审查

`.msg` 是板内 uORB 合同，不是 MAVLink XML。不能要求 QGC 直接解码它们，也不能将扩展板内状态称为不符合 MAVLink 2。所有字段/枚举/目录仍从权威 schema/YAML/manifest 生成，未手写 wire ID、CRC、codec 或消息列表，未扩展私有 MAVLink 消息。

| 本地合同 | 用途与 QGC 影响 |
|---|---|
| `AutoCalibrationRequest.msg` | Commander 内部 Level/阶段 Disarm/退出请求；不发给 QGC |
| `AutoCalibrationStatus.msg` | 固定中心/半径/距离/边界、阶段、临时增益、验证及保存状态；内部消费和 ULog，QGC 通过标准 `[autocal]` 文本读取摘要 |
| `RoverControlStatus.msg` | 真实 PI/执行器反馈、请求/session/参数代次及限幅标志；内部验证/日志，不新增 wire 遥测 |
| `RtkHeadingStatus.msg` | 原始阵列航向、基线、解类型及位置/航向/速度历元；内部质量门禁，不要求 QGC 定制解析 |
| `RoverMotionRequest.msg` | 独立 `SOURCE_CALIBRATION` 的 normalized 或 speed/yaw-rate 请求；不是导航来源伪装 |
| `SensorCalibrationRequest/Status.msg` | 本地 Level、QGC/AUTO 反馈归属与完成状态；外部 QGC 仍走标准命令/ACK 和 `[cal]` Sensors 文本 |
| PX4 同名传感器/估计器 schema | payload 继续与锁定 v1.17 合同一致，通过现有架构/生成检查；没有把本地增益或围栏字段塞入上游消息 |

| 对外能力 | 当前兼容性、影响和整改 |
|---|---|
| 模式发现/切换 | 已补标准 `AVAILABLE_MODES`、`CURRENT_MODE`，同一 YAML 生成名称、custom_mode、nav-state 和 properties；实际/意图来自同一 Commander 状态。官方 QGC 5.1.3 源码支持，属于必要整改，已在固件实现；实板 UI 待验收 |
| 模式目录 | Manual/Mission/Auto Calibration 可选；Hold/Termination 不开放手选。目录固定，不增加 `AVAILABLE_MODES_MONITOR`。旧地面站可能回退显示 External1，不据此新增私有消息 |
| Sensors 反馈归属 | 已从权威 msg 增加 owner，接受事务时固定；内部 Level/Cancel 不发 `[cal]` start/progress/terminal，避免错误结束外部 Sensors 请求。这是高重要性整改，已实现 |
| 长提示/重连 | 修正标准 STATUSTEXT 分片 ID 和整 50 字节倍数结束片；阶段变化与每 5 s RAW 重报边界、等待 Arm、临时值和最终状态。协议闭合，真实 USB/QGC 丢包与重连待验收 |
| 半径/结果参数 | 使用标准 PARAM/PARAM_EXT 和 Component Metadata；官方 QGC 参数页可配置/读取，不增加“确认增益”参数 |
| 专用向导/圆覆盖/结构化增益图 | 官方 QGC 没有本项目专用界面，影响可视化和操作便利，不影响固件边界与自动保存。当前用消息面板/参数页/ULog，低于安全链整改优先级；如以后需要界面，另行授权 QGC UI 开发，不扩张本轮 wire 协议 |

QGC 对照版本为官方 5.1.3 commit `7fe5b11b18a4c2eec17beb1b2a3ef45ac0c4e32e`；没有修改 QGC。MAVLink definitions/pymavlink 仍沿用仓库锁定来源，详见 `DIMA_SOURCE_MANIFEST.md` 第 17 节和 `Dima/modules/mavlink/README.md`。

## 7. 工作区与非测试验证

正式仓库为 `E:\freertos\H743_FreeRTOS`，分支 `feature/dima-phase3`，HEAD `9a5f8b9a738b8f68f624a465a07e616535dc128d`。R1–R5 开始时已有 82 个 tracked 变更和 20 个 untracked 项（Git 默认目录折叠计数），包括基础校准及此前日志/存储/RC 工作；并非干净基线，不能把全工作区 diff 当成本轮增量。

本轮按围栏、事务/控制、增益算法、QGC/反馈、权威生成与文档分主题收敛；只提取必要共享路径逻辑，没有引入第二套导航控制器或通用整定框架。保留既有日志/存储/RC 差异，不做整体回退、clean、提交或推送。全工作区不是单主题最小 diff，但不能为缩小统计而删除用户既有工作；后续提交应按上述主题单独审查和显式暂存。

不新增/修改测试文件、框架、runner、fixture、mock、test-only API、Host Test、SITL 或仿真，也未执行它们。复用以下 Windows 原生非测试门禁，修改权威源后自动生成，不手改 `build/generated`：

```powershell
Set-Location E:\freertos\H743_FreeRTOS
& C:\Users\master\.local\bin\make.cmd -j4 NO_COLOR=1 `
  dima_rover uorb-generated-verify mavlink-generated-verify `
  parameter-metadata-verify logger-generated-verify
git -c core.safecrlf=false diff --check
```

最新制品与统一门禁结果在下节记录；文档末尾旧版镜像仅是历史记录。

## 8. 当前 Windows 静态验收与制品

Windows 原生执行第 7 节正式命令，使用项目缓存 Arm GCC 10.3.1；最新源码和参数描述生成后构建 exit 0，完成 `[83/83]`。这是完整发布目标的依赖增量构建，不是删除缓存后的 clean build。

- 生成 302 个参数，45 个 uORB schema、55 个日志 Topic 和 8 种 Logger Profile 组合；Parameter/Component Metadata 与 Logger 合同通过。uORB/MAVLink 在独立临时目录重新生成并比较通过，wire 方言仍为原有 230 条官方消息定义。
- 架构检查 `PASS - 446 first-party source files`；Application ELF/签名/Factory 布局校验通过，向量 `0x08040400`，Application/MCUboot 未解析符号为空，MCUboot watchdog prepare/feed 链已链接。
- 末轮静态审查闭合了准确采样周期、RTK 噪声有限性、RLS 方差尺度、整形前后输入/PI 坐标、用户输出 ceiling、路径可观性/候选同代复验、晚启动预算和 RAW 状态反馈问题。未用新增测试或仿真代替实车证据。
- `git -c core.safecrlf=false diff --check` 通过；36 个未跟踪文件逐一用现有 Git `--no-index --check` 核对，无空白问题。变更中测试、框架、runner、fixture、mock、Host Test、SITL、仿真路径为 0；未手改 `build/generated`。

生成工具对锁定上游 XML 保留 `CAMERA_TRACKING_STATUS_FLAGS_IDLE` 和 `MAV_BOOL_FALSE` 两条 bitmask 提示，不是本轮引入的私有消息或编译失败；没有修改固定 XML 来压掉提示。

Application `text/data/bss=589304/12688/551968` bytes，总计 `1153960` bytes；Flash `589104/785408`（75.0%）、SRAM `563608/884736`（63.7%）。D2 data 为 `245368/262144`（93.6%），仅余 16776 bytes，后续扩展要关注该区域；这些是全工作树静态占用，不是本功能独立增量或实板峰值。当前签名样本 image digest 为 `f6075193bbcc5df4736b90b54196aad8d2ee3ac0c82c9f13412272cefe2c014b`。

| 当前制品 | 大小（bytes） | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 10730052 | `000b4eb38bc0369b111efc9cb1935486ae10b8978fe0ff0ea3e53e03375efff6` |
| `build/H743_FreeRTOS.bin` | 602032 | `9bf6504d6ac24427c6ad483ff0c073514120a138a2c5c0c1c1409470eaec6699` |
| `build/H743_FreeRTOS_signed.bin` | 603206 | `45b78f3c1627355ad04de4ffd497509e881a3cd8fcd8dde7df1515fe494091a0` |
| `build/H743_FreeRTOS_factory.hex` | 1567756 | `4461602079765141406f3919dfec1401caebec5a9b95db0061cc1c92b3222a13` |
| `build/mcuboot/mcuboot.elf` | 2681876 | `9c9b43234c5b5dc90199416c56b818650fc8baf94b258b0b0748d19518c284a7` |
| `build/mcuboot/mcuboot.bin` | 48236 | `e582de10b628a069478a070e3eb00de881b77d7cce4e0348901e8d9c8e2f7a83` |

签名容器哈希只标识这一份 ECDSA 签名样本，重新签名不承诺字节一致。最终工作区 88 个 tracked 变更、36 个 untracked 项，暂存 0；HEAD 未改变。未提交/推送、未上传/刷机、未访问板卡/QGC 串口或触发车辆运动。下一步只剩授权后的第 9 节实板验收，不能据此宣布车辆校准或围栏性能通过。

## 9. 授权后实板验收（全部待执行）

| 场景 | 必须取得的证据 |
|---|---|
| Level、阶段 Disarm、RTK 重锁、磁初始化、候选切换 | session 的中心/时间/设备不变；本地原点/yaw reset 不移动圆 |
| 无入场定位/定位恢复 | 本会话不建立动态圆，只有退出重进才能重新捕获 |
| 半径/停车配置不足 | 不运动或缩短后部分完成；不扩大半径、不伪造模型样本 |
| 接近边界/越界/定位失鲜或设备切换 | 在余量边界提前撤销动力；不自动返回圆内；实际停止距离和 GNSS/控制/PWM 延迟满足配置预算 |
| 会话内修改半径/停车距离/输出上限 | 中止本会话并回滚临时组，新值只下次生效 |
| 内环、Heading、Path | 实际超调/稳态/振荡/饱和、转驱退出及可观样本符合门槛；未覆盖工况不宣称已整定 |
| 增益成功/失败/临时值断电 | 通过后无人确认即保存；失败恢复当前组最初旧值；重启不保留未验证候选，也不恢复运动会话 |
| 回滚旧零值/无法确认回滚 | 前者可安全回到导航未就绪；后者 FAILED 和禁 Arm 锁存，不误报恢复 |
| 无磁/失效磁/RC loss/Kill/模式切出 | 正确跳过、失败或取消；已保存组保留，动力及时失效 |
| 官方 QGC 5.1.3 | 模式名称发现/切换/实际意图区分、Sensors 外部事务不受内部 Level 干扰、参数刷新、长文本和重连等待提示正确 |
| 资源与日志 | 实际调度/栈/看门狗余量、SD ULog 与前后参数快照完整；静态内存占用不能代替运行峰值 |

</details>

<details>
<summary>历史归档：R1–R5 之前的 T1–T16 基础版本（旧流程与旧制品，不用于操作）</summary>

以下内容原样保留基础阶段的检查过程，明确已被上文覆盖；旧“PID 不写”“距离+5 m”“QGC External1 名称待补”不是当前实现语义。

## 目标和约束

2026-09-07 增量实施已获授权：固定入场 GNSS 圆心、`RO_CAL_RADIUS`/`RO_CAL_STOP_D`、
静态降级、增益自动验证/保存以及官方 QGC 模式发现。本节以下旧 T1–T16 是基础版本
的历史检查点，不能当作新增量已完成；新增量检查点 R1（围栏）、R2（降级/反馈）、
R3（事务/闭环）、R4（自动增益）、R5（生成/构建/审查）当前开始实施，实板仍未授权。

目标是 Disarmed 进入、操作者逐段 Arm 的组合校准模式：水平 → RTK 基线及航向安装偏置 → 速度/转向前馈辨识 → RTK 约束下运行磁偏置学习。通过前端应用确认的阶段保留；当前阶段失败只回滚该阶段。模式不自动 Arm，Kill/失联/切出立即撤销动力。

所有参数、消息和列表从 YAML、msg、manifest 生成；不新增或修改测试文件、框架、runner、fixture、mock 或仿真。关键数学、坐标系、控制转换、事务、安全和回滚逻辑使用中文注释。Windows 原生生成/构建是正式静态证据，车辆行为另需实车验收。

不自动修改物理轮距、GNSS 相对重心 lever arm、PWM 端点、PID 增益或磁力计软铁比例。双向水平运动只能可靠辨识平面动力学和有界 hard-iron offset。

## 计划检查结论

- `-1` 槽保持无动作，显式选 `NAVIGATION_STATE_EXTERNAL1` 才请求校准。模式编号由已有 VehicleStatus schema 生成。
- 水平使用同一底层 routine 供组合模式及 QGC `param5=2`；必须在已知水平地面上操作，重力不能识别车身与地面之间的坡度。
- 首段运动用原始阵列航向差保持方向，避免未标定 GPS_YAW_OFFSET 或磁力计决定真航向而形成循环依赖。
- 原始阵列基线解算与地面 RTK FIX 分别核对；GPS 位置 FIX 本身不能证明双天线整数解正确。
- 参数提交后等待目标值、代次及新鲜前端输出；仅 `param_set` 成功不构成完成。运动期间只缓存候选值。
- 运行磁校准必须先有 RTK 参数提交和 GNSS yaw 连续融合。磁场检查被拒绝时有界 known-yaw/WMM bootstrap 只作为候选，仍需停止、应用、重新学习和验证。

### 二次核对修正（2026-09-07）

1. `NAVIGATION_STATE_EXTERNAL1=23` 是内部导航状态，不是 HEARTBEAT 的 PX4 custom_mode 编码。RC 参数、Commander、SET_MODE 和 HEARTBEAT 分别走各自权威协议映射；当前 QGC 的模式名称显示必须实测，不能仅凭参数元数据宣称有新的校准向导。
2. RTK 需显式保存 UNIHEADINGA 的解类型和 GPS week/milliseconds。只有基线整数固定解、位置 RTK FIX、新鲜且递增的测量历元、精度通过时才采样。`SOL_COMPUTED` 单独不足。原始 heading 与速度按同一历元/有界时间差配对，重复样本不增加置信度。
3. 当前 Commander 的 `preflight_checks_pass()` 只允许 Manual，`disarm()` 自动切回 Manual；须明确扩展“校准 WAIT_ARM”的逐项门禁和专用内部阶段 Disarm。外部 Disarm、Kill、切出仍结束会话。已有 `calibration_enabled` 是禁止 Arm 的传感器校准占用，不能直接复用为所有运行校准阶段标志。
4. PX4 总旋转是 `R_fine * R_discrete`。水平 routine 去除原有 X/Y 修正后求候选，既不盲目取负，也不按 old+mean 累加；Z 保持原值。IMU 旋转改变需递增 EKF 实际消费的 accel/gyro calibration_count、清积分及旧 bias 候选。前端在无变化和回绕/饱和条件下的确认也要可结束。
5. 速度与转向系数存在前馈耦合：首次往返只缓存速度候选，先单独提交 RTK；第二次转向后将 `RO_MAX_THR_SPEED` 与 `RO_YAW_RATE_CORR` 同组提交。转向辨识失败时同时保留旧的两个动力学参数。磁偏置独立提交，不因磁失败撤销成功 RTK/动力学。
6. 分开记录 `candidate / applied / validated / persisted`；保存门禁必须覆盖 autosave 和显式参数保存，避免未验证的候选被中途保存。原有 SensorCalibration 的 done 仅代表前端已应用，组合校准必须补齐保存/恢复证据，重启只接受完整已确认阶段。
7. 磁场偏置只有在已标定姿态、足够航向覆盖及 WMM/方差约束下可用；水平运动本身不能无条件观测垂直偏置。`mag_bias_stable` 可能延续此前累计，组合模式还需自己的本会话连续质量窗口。设备已配置/本会话曾出现后失联算失败，只有确实未安装才跳过。
8. 自动运动依赖的上限同时约束请求、实际车速/角速度、加速度、位移和时间；首段不依赖未校正的 EKF yaw/位置。采样辨识需核对 motor shaping，不能把整形前后的归一化输入混用。bootstrap 需停车应用后重新显式 Arm，最多一次，不能因失败自动反复运动。
9. 锁定 PX4 名称的 schema 必须保持上游 payload 一致（R333）。阶段请求采用本地 `AutoCalibrationRequest.msg`，前端确认复用已有正式接口；不得向 ActionRequest/VehicleImuStatus 添加本地字段。
10. 掉头/圆周只在起转前等待停车，以独立状态锁定确认；起转后的天线 lever arm 地速不重复触发停车，换向滑行角也不计入新方向。掉头先顺时针，逐历元维护连续剩余角，避免半圈目标在 `+/-pi` 处反复切方向；接近或跨过目标先请求停车，停车后复核并按需从零输入做小角修正，全部修正仍共用原 60 s 截止。转向以实际角速度缓慢跟踪目标，不以固定电机输入下限挡住高 yaw 增益车辆；动力学拟合要求真实输出同向、稳态且整形误差不超过请求的 10%。

PX4 行为参考锁定为 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`（BSD-3-Clause）；本地组合状态机、安全扩展和 RTK 质量门禁是薄适配，不能标作上游现成功能。

## 有序任务

各任务规模按权威源及第一方实现计，生成物不计入手改文件数。遇到跨模块任务拆为小的编译可验证步骤。

| 任务 | 内容与验收标准 | 验证 | 依赖 | 主要文件/规模 | 当前证据 |
|---|---|---|---|---|---|
| T1 | 参数合同：明确模式、水平 offset、基线及运动限制；默认不触发运动 | 现有 parameter-generated | 无 | 3 个 YAML、参数生成器 / M | 源码/生成完成，300 参数 |
| T2 | 本地请求/状态/原始 RTK/运动来源及枚举名称；不扩展锁定 PX4 payload | uorb-generated、logger-generated | T1 | schemas、logger YAML、uORB 工具，分两批 / M | 源码完成，44 schema / 54 Topic |
| T3 | UM982 原始阵列信息、整数解、heading/velocity 独立历元、基线消费者与应用代次 | 定向 Arm 编译、raw/status 实板核对 | T2 | Um982Gps、Um982Protocol、RtkHeadingStatus / M | 源码完成；板端验收归 T16 |
| T4 | 板级细旋转进入 IMU/磁前端；代次及目标值在同一锁内确认 | 定向编译、静止姿态核对 | T1/T2 | SensorRotation、VehicleImu、Mag / M | 源码完成；板端验收归 T16 |
| T5 | Level Horizon routine、QGC selector、原子提交和回滚 | 定向编译、QGC 水平/移动/取消验收 | T4 | SensorCalibration、算法、Commander，分两批 / M | 源码完成；板端验收归 T16 |
| T6 | 模式生命周期/注册、有限状态机，默认无效请求 | 全固件构建、启动/停止检查 | T2/T5 | AutoCalibrationMode、ApplicationContext / M | 源码完成 |
| T7 | Commander 进入/Arm/阶段 Disarm/取消互斥及 SET_MODE/HEARTBEAT 映射 | 静态逐分支、实板 RC/Kill/切出 | T6 | Commander、MAVLink、权威策略，分两批 / M | 源码完成 |
| T8 | 独立来源、同拍请求、TTL、整形后硬限幅；PWM/watchdog 同步支持掩码 | 定向编译、离地动力边界核对 | T7 | RoverDifferential、MotorOutput、BootHealth、RC、共享合同，分两批 / M | 源码完成 |
| T9 | 基线稳健统计、两条相反前进直线圆均值；双历元分别去重 | 实车 RTK 数据与拟合残差 | T3/T8 | AutoCalibrationRtk、CalibrationMath / M | 源码完成；数据验收归 T16 |
| T10 | 多档速度与双向转向拟合，要求低实际加速度且连续稳态 | 实车前后速度/yaw-rate 残差 | T9 | AutoCalibrationRtk、CalibrationMath / M | 源码完成；数据验收归 T16 |
| T11 | 阶段事务、前端确认、保存暂停/lease 转交、当前组回滚；动力学同组 | 静态事务审查、实板持久化/取消 | T5/T10 | CalibrationParameters、参数核心、前端，分小步 / M | 源码完成 |
| T12 | RTK 持续融合门禁、按真实方向分组的运行磁偏置观测 | bias/status/aid-source 实车数据 | T11 | AutoCalibrationMag、EKF wrapper / M | 源码完成；数据验收归 T16 |
| T13 | RAM-only WMM bootstrap；最终 refine 保留原快照，失败/断电恢复旧值 | 静态公式/生命周期审查、前端与残差 | T12 | AutoCalibrationMag、CalibrationParameters / M | 源码完成；实板验收归 T16 |
| T14 | 人可读生成枚举名、PARTIAL/FAILED/取消、不可恢复回滚锁存 | STATUSTEXT、ULog、集中代码复核 | T13 | AutoCalibrationMode、uORB 工具、说明 / M | 源码完成；T15 静态验收通过 |
| T15 | 生成/架构/完整 Windows 构建及集中代码审查 | 仓库现有非测试门禁 | T14 | 受影响源及文档 / S | 已完成：生成、架构、Windows 构建和静态源码审查通过 |
| T16 | 已授权场地/实板的校准与故障验收 | 下表人工操作 | T15 | 不新增测试代码 | 尚无实车证据 |

检查点：T1–T3 合同；T4–T5 水平闭环；T6–T8 模式安全；T9–T11 RTK/动力学；T12–T14 磁学习；T15 静态制品；T16 车辆。

## 运行顺序

`IDLE → PREFLIGHT → LEVEL → APPLY/VALIDATE/SAVE_LEVEL → BASELINE → WAIT_ARM_1 → STRAIGHT_OUT → TURN_AROUND → STRAIGHT_BACK → STOP/DISARM → APPLY/VALIDATE/SAVE_RTK → RTK_RELOCK → WAIT_ARM_2 → TURN_CW → TURN_CCW/MAG_LEARN → STOP/DISARM → APPLY/VALIDATE/SAVE_DYNAMICS → APPLY/VALIDATE/SAVE_MAG → DONE/PARTIAL/FAILED/CANCELLED`。

磁 bootstrap 分支：RTK 重锁定 → 已知航向采样 → 停车/Disarm → 临时 offset 应用 → 再次显式 Arm → 连续 GNSS yaw + 磁学习 → 停车验证。临时 offset 仅在 RAM，始终保留原快照并暂停所有参数保存；最终 refine 成功才保存，取消恢复原值，掉电恢复上一份完整持久化参数。不可恢复回滚发布 FAILED，保留禁 Arm/保存锁存。所有普通等待状态有截止时间；运动失败不自动重启。

## 实施后的操作与门禁

1. 先人工确认车架处于已知水平基准、场地有足够直线距离、轮距/PWM/左右电机方向已配置。进入时 Disarmed；`COM_FLTMODE` 显式选 23，默认 -1 仍不执行动作。Stock QGC 可把当前模式显示为 External1；名称和独立向导必须以实板 UI 为准。
2. 固件完成 Level 和静止基线采样后提示第一次 Arm。车辆两段相反方向前进行驶，中间自动掉头，然后停车、阶段 Disarm、应用/验证/保存 RTK。成功组保留。
3. GNSS yaw 连续融合后提示第二次 Arm。顺/逆方向各至少一圈；磁学习需要足够样本时继续有界转动，首圈后额外等待最多 45 s，每方向总硬超时 75 s。换向瞬态不归入方向拟合。未知 yaw 系数时以实际角速度反馈缓慢跟踪 0.3 rad/s，配置 steering 值仅作为上限。
4. 若需要 WMM bootstrap，停车应用临时磁偏置后另行提示显式 Arm；这次只继续磁学习，不用追加的磁圈掩盖此前动力学辨识失败。
5. 磁校准要求新鲜 raw 与可用前端、合法 rotation/offset/scale、`SENS_MAG_RATE >= 10 Hz`（正常运行仍允许 1..200 Hz，校准不自动改变它）。新 raw 与由 GNSS yaw 约束并已传播的姿态需在 50 ms 内匹配；停车后的验证使用前端窗口均值。晚出现设备单向锁定 present，之后失联不能改报未安装。
6. 若速度不可观且没有可用磁目标，保存成功 RTK 后 PARTIAL 结束。确有可用磁目标才允许“仅磁校准”运动；PID、物理轮距、lever arm、PWM 端点及 soft-iron 比例不自动修改。

硬边界：请求 throttle <=0.40、|steering|<=0.35；整形后每轮电机绝对值 <=0.40、变化率 <=0.15/s；实际地速 <=1.5 m/s、yaw-rate <=0.6 rad/s、线/角加速度 <=3 m/s²、3 rad/s²；距起点 <=设定直线距离+5 m，每次运动周期 <=180 s，全会话 <=600 s。输入/输出单次控制间隔异常、RTK 解/历元停滞、第二段 GNSS yaw 失融、RC loss、Kill、参数并发变动均撤销运动。拟合另要求低通实际加速度 <0.1 并连续稳态 1 s，不能把安全上限当作稳态判据。

WMM 为仓库已有地磁表先验；平面运动不能保证所有三轴偏置都可观。缺乏方差、双方向覆盖或残差证据时保留已有成功阶段并报告 PARTIAL，不能将窗口超时称为磁校准完成。

每一运动段设独立超时、速度/角速度上限、距起点包络。门禁丢失或控制周期超时立即发无效请求，Commander 解除 Armed。故障后不自动重试运动。正常阶段停机由内部专用来源请求，保持校准会话；用户 Disarm 则取消。

## 验收矩阵

| 场景 | 要求 |
|---|---|
| 启动/RC 恢复/默认 -1 | 保持原模式，无运动、无自动 Arm |
| 已 Armed 请求进入 | 明确拒绝 |
| Level 静止/车体移动/坡面 | 已知水平基准稳定采样；移动拒绝；不得声称自动识别坡面 |
| RTK float/无航向/失鲜/基线跳变 | 不提交；运动期立即撤销动力 |
| 往返直线 | 两段前进，圆均值一致，速度与航迹误差满足门禁 |
| 掉头/换向/天线杆臂 | 仅起转前等待停止；换向滑行不计入新方向；近目标先停、复核后小角修正，返回直线前须真实停车 |
| 转向响应不足/停车迟缓 | 60 s 掉头、75 s 单方向及总运动截止保持有效；不能按时满足条件则停止并拒绝该阶段候选，不承诺任意动力组合都可自动收敛 |
| 机械不对称/轮打滑 | 拒绝动力学系数并给出原因 |
| 无磁力计 | 标记跳过；RTK 等成功可完成 |
| 存在磁力计 | GNSS yaw 连续融合后才学习；不让磁场重新控制 yaw |
| 初始大偏置/动态电流干扰 | 有界 bootstrap 或拒绝；保持磁场检查，不写异常 offset |
| 参数并发变动/取消/前端超时 | 冻结运动，当前组回滚且确认；已验证组保留 |
| 断电/重启 | 只恢复完整已保存参数代；不恢复运动会话 |
| Kill/RC loss/模式切出 | 输出失效、Disarm，新 Arm 之前不得恢复 |

## 验证入口

在 `E:\freertos\H743_FreeRTOS` 使用 PowerShell 及 `C:\Users\master\.local\bin\make.cmd`：`parameter-generated uorb-generated logger-generated`、对应 `*-verify`、`check-architecture`、`-j4 NO_COLOR=1 dima_rover`。不运行上传、不访问占用串口；实板动作需使用操作者实际场地和操作证据。

## 开始实施时状态

2026-09-07，`feature/dima-phase3` / `9a5f8b9a738b8f68f624a465a07e616535dc128d`；原有日志、RC、Commander、生成链脏改动已识别并保留。此前仅方案勘察，没有组合模式实现。当前没有实板/QGC/车辆校准结果。

## 历史基础版本 Windows 静态验收（2026-09-07）

在上述分支及共享工作树执行以下正式命令，使用项目缓存的 Arm GCC 10.3.1；这是更新后源码的完整发布目标验收，按依赖增量编译，并非删除缓存后的 clean build：

```powershell
Set-Location E:\freertos\H743_FreeRTOS
& C:\Users\master\.local\bin\make.cmd -j4 NO_COLOR=1 `
  dima_rover uorb-generated-verify mavlink-generated-verify `
  parameter-metadata-verify logger-generated-verify
git -c core.safecrlf=false diff --check
```

构建命令 exit 0，完成 `[20/20]`。uORB/MAVLink 在独立临时输出目录重新生成并校验通过；Parameter/Component Metadata 与 Logger 合同通过，当前为 300 参数、44 schema、54 Topic、8 种 Logger Profile 组合。架构门禁 `PASS - 431 first-party source files`，Application 向量为 `0x08040400`，Application/MCUboot 未解析符号均为空，MCUboot watchdog prepare/feed 链已链接，Signed BIN/Factory HEX 布局验证通过。生成工具对固定上游 common/minimal XML 的两条 bitmask 提示不属于编译失败；本次没有修改锁定 XML。

本功能源码审查已完成，末轮集中核对并修正起转/换向锁存、天线杆臂地速、180 度连续角、低输入采样门槛、近目标停车复核、首圈后学习窗口和真实 Disarm 边沿；上述最终源码重新通过同一 Windows 命令。仍不把静态审查解释为实际动力响应或校准收敛证明。

Application `text/data/bss=548680/12688/546336` bytes；Flash `548480/785408`（69.8%），SRAM `557976/884736`（63.1%）。其中 D2 data 为 `239736/262144`（91.5%），后续扩展需继续关注该区余量；这不是本轮功能的独立资源增量。镜像 digest 为 `ec23814496595c7a60247ef9e868ac6665e63e08445dd175bcece87bf96911b7`。

| 当前制品 | 大小（bytes） | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 10063612 | `382ef22e77de6127e521fead4807f53420d613ae8ce6d2bd6f411d1138409a0b` |
| `build/H743_FreeRTOS.bin` | 561408 | `9f9bc5306720a7132ec48e8a30388215d939fc25642276c44025bacea7db0a69` |
| `build/H743_FreeRTOS_signed.bin` | 562584 | `c37c6307e7defe865bec07b501f5ff5b66d756dfc8c8cf9de0baf27e069e4057` |
| `build/H743_FreeRTOS_factory.hex` | 1469985 | `ad855f7b1c573d3a6e238b346481d4a22772f03bd47fbc50cf986712ffb384e1` |
| `build/mcuboot/mcuboot.elf` | 2681876 | `9c9b43234c5b5dc90199416c56b818650fc8baf94b258b0b0748d19518c284a7` |
| `build/mcuboot/mcuboot.bin` | 48236 | `e582de10b628a069478a070e3eb00de881b77d7cce4e0348901e8d9c8e2f7a83` |

签名容器大小与 SHA-256 只标识本次 ECDSA 签名样本，不是重签确定性合同。`git diff --check` 通过，20 个 untracked 文件亦用现有 Git `--no-index --check` 核对空白问题；tracked/untracked 变更中测试、Host Test、SITL、仿真、runner、fixture、mock 路径为 0，未手改 `build/generated`。未提交、推送、上传、刷机、访问板卡/QGC 串口或触发车辆运动；T16 的重启持久化、取消回滚、校准残差、QGC 显示和全部车辆安全行为仍为 `BOARD/QGC/VEHICLE PENDING`。

</details>
