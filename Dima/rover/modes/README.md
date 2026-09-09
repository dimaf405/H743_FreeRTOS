# Rover 产品模式

## 目录布局

```text
modes/
├── manual/
│   └── ManualMode.cpp / ManualMode.hpp
├── auto/
│   └── AutoMode.cpp / AutoMode.hpp
├── auto_calibration/
│   ├── AutoCalibrationMode.cpp / AutoCalibrationMode.hpp
│   ├── AutoCalibration*.cpp
│   └── CalibrationParameters.cpp / CalibrationParameters.hpp
└── README.md
```

所有模式采用一致的目录层级：人工控制位于 `manual/`，Mission 导航位于 `auto/`，自动校准的协调器、阶段及私有参数事务位于 `auto_calibration/`。平台无关的拟合、围栏与响应算法由 `Dima/lib/rover/` 提供。构建自动发现一级子目录中的实现文件；同目录 include 只写文件名，对外和跨模式引用统一使用 `manual/ManualMode.hpp`、`auto/AutoMode.hpp` 或 `auto_calibration/AutoCalibrationMode.hpp`。

## 运行契约

- **当前模式：** `ManualMode` 把 `manual_control_setpoint.throttle/yaw` 转换为 `rover_motion_request` 的前后、左右两轴；它是 Rover Manual 的运行入口，不是传输或 RC 适配器。
- **边界：** 模式不解析 RC、SBUS 或 MAVLink，不实现差速算法，不发布 `actuator_motors`，也不访问 MotorOutput/PWM。
- **AUTO 参数语义：** 对齐 PX4 v1.17，Mission 切换只检查任务、估计器和安全状态；`RO_*`、`RD_*`、`PP_*` 控制参数只决定切换后的 Navigation 请求是否有效。配置未就绪时保持 Mission 状态并发布全 NaN 请求，由下游确认物理停波，不把调参状态伪装成模式切换失败。
- **组合校准：** `AutoCalibrationMode` 仅在 Disarmed 显式进入 External1，先复用 `SensorCalibration` 完成水平，再做 RTK 基线/往返航向标定，之后才在 GNSS yaw 持续融合下辨识转向并学习磁偏置。只需首次操作者 Arm；Commander 持有会话授权，后续停车、内部 Disarm、消费者确认和正常 preflight 后自动续行。所有激励通过生成的 `SOURCE_CALIBRATION` 请求进入差速控制链，不越过控制器或直接访问 PWM。
- **校准事务：** `CalibrationParameters` 将 RTK、动力学两参数和磁偏置分别原子提交，确认消费者应用后才保存；磁 bootstrap 只临时应用 RAM 并保留原始快照。取消只回滚当前组，未确认回滚保持禁 Arm/保存锁存。范围、超时、失败降级和实板验收见 `docs/AUTO_CALIBRATION_PLAN_ZH.md`，静态构建不代表车辆已校准。
- **关联自动整定：** 基础阶段之后采集未零区化噪声、多档正反响应和可选满输出探测，辨识前馈、运行参数及速度/yaw-rate 四项 P/I，再验证 Heading、转驱与 RAM 路径。32 槽事务保留最初快照；整形变化必须重新采集 FF，所有关联候选只在 RAM，最终验证后整组保存，失败整组回滚。`revise_float/apply_revisions/finalize_provisional` 不借用磁专用 `refine`，不要求人工确认候选值。
- **事务入口收敛：** 已删除没有生产调用的旧整组数组替换入口；关联候选只通过命名参数 revision 更新。磁 bootstrap 仍使用实际在用的 `refine`，最终保存、旧值快照、消费者代次和回滚时序不合并或放宽。
- **固定圆：** `AutoCalibrationFence` 仅在入场锁定全球圆心、定位设备和配置；Level、阶段 Disarm、RTK 重锁以及增益试用均不重设。`RO_CAL_RADIUS` 默认 20 m，`RO_CAL_STOP_D` 默认 0 禁止动态阶段；后者必须来自当前工况的可信停车距离上界。入场定位不可用只做静态项；恢复后要退出重进，不能在原会话补建圆心。
- **长度精简：** `RO_CAL_DIST` 已退役，内部保持原默认 12 m 期望尺度；实际直线按 `min(12, R_work-distance-0.5)` 自动缩短，不足 5 m 拒绝。RAM 路径也不超过这一尺度，阶段/会话预算和五项剩余 `RO_CAL_*` 安全配置不变；旧快照中的自定义期望长度不再生效。
- **速度与反转：** 入场正 `RO_SPEED_LIM` 是冻结巡航上限；0/旧 -1 使用 `RO_CAL_VMAX`（默认 1.5 m/s）并申请独立前进满输出探测。其余非法值拒绝运动；本事务后续写回巡航不改变本会话围栏预算。只在指定段允许低速倒车，保留 `MOT_REV_DELAY`；满输出指允许命令端点，不是测得 RPM。
- **有限路径与降级：** `AutoCalibrationPath` 在同一圆内用闭合瘦三角 RAM 路径和共享 `SegmentGuidance`，所有候选先回固定入口、对齐同一首边，不覆盖用户 Mission。缺 RC/PWM/双天线不提前阻止 Level；缺磁力计可以跳过磁阶段。运动依赖、激励、空间或时间不足报告未完成项，不猜测参数、不扩大圆或放松 EKF 门限。全局整形必须覆盖实际使用范围；局部低速数据不授权修改 Manual 共用整形。
- **统一 IMU 零偏：** 复用 EKF 稳定偏置和 VehicleImu 实际校正快照，提交合格 ID/offset，保留 scale；重新确认前端、EKF 与残差后保存。组合会话暂停独立 IMU 后台写回，`EKF2_MAG_DECL` 仍只更新 volatile RAM。
- **扩展：** Navigation、Offboard 等后续模式统一建立各自的一级职责子目录；`modes/` 根目录只保留组织说明，不再混放模式实现文件。
