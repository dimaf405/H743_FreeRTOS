# Rover 纯算法库

- **当前运行消费者：** `AutoMode` 在 `wq:nav` 以 50 Hz 调用 Pure Pursuit、Heading P 和停车/原地转向状态机；`RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 调用 Speed PI、YawRate PI 与 `DifferentialDrive`，再发布两路电机请求。
- **证据边界：** 四个控制器、任务执行和 AUTO 安全投影已经进入正式固件闭包并通过 Windows 目标构建；这只是 `SOURCE / STATIC / WINDOWS BUILD VERIFIED`，不能替代 QGC 任务事务、SD 掉电恢复、目标板闭环或实车轨迹证明。
- **边界：** 本目录只含可脱离平台运行的算法和数据契约，不包含 HAL、FreeRTOS、uORB、Parameter、Flash 或动态模块生命周期代码。
- **失效语义：** 控制器遇到非有限输入、非法 `dt` 或未配置参数时复位 slew/积分并返回无效；PI 采用条件积分反饱和，零设定清空对应 PI/slew，原地转向必须先确认速度设定为零且实测速度低于 `RO_SPEED_TH`。
- **反向可行域：** `MOT_THR_ASYM>1` 时先在 `[-1/asymmetry, 1]` 电机域内完成 `RD_STR_THR_MIX` 饱和优先级，再作反向推力补偿、expo、最小/最大输出、换向等待和 Arm ramp，禁止先裁到 `[-1,1]` 后让两侧倒车同时饱和。
- **来源：** 优先保持上游 Rover 控制类型、单位和算法执行顺序；平台、消息和生命周期差异由 `Dima/rover` 运行层适配。
- **共享路径计算：** `SegmentGuidance` 组合既有 Pure Pursuit、HeadingController、DrivingStateMachine 和速度规划，供 Mission 与校准 RAM 路径共用；本库不拥有 Mission 存储或模式状态。
- **校准数学：** `CalibrationFence` 使用固定全球圆心、WGS84 局部尺度和误差/延迟/停车余量计算有效工作圆；`CalibrationIdentification` 复用锁定 PX4 v1.17 的 `ArxRls` 六模型延迟库，只对通过门禁的一阶模型设计 Rover 并联 PI，不复制飞行器 Autotune 的激励或验收参数。源路径、许可证与薄适配见 `docs/DIMA_SOURCE_MANIFEST.md` 第 17 节。
- **输入坐标：** RLS 输入是最终左右归一化执行器命令的均值/差分，而非物理轮速；按同批样本验证整形映射的斜率和线性残差，再将 PI 换回控制器整形前坐标。硬限幅、围栏或不可辨识数据不能授权保存。
- **响应数学：** `CalibrationResponse` 使用固定双向六平台、Welford 噪声统计、20%–80% 非零升降响应斜率及真实精度下界，给出起动括界、等效 MIN/EXPO/ASYM 和覆盖诊断。生产逆整形对应 `v=a*r+b*r²`，`K=a+b`、`expo=b/K`；不把执行器命令当实际 RPM，不用零残差虚构零噪声。未满足正反全局覆盖和独立可观测条件的候选不授权保存。
- **接口收敛：** 响应库只保留已接入生产的接口；`gain_lower_bound` 固定使用其全部现有消费者需要的整形前坐标。移除的最小值跟踪仅属于通用平台统计；TransientSlope 的最小值/幅值门禁、ARX 的 applied 输入及完整电机曲线验证仍保留。源码减少不等于同等 Flash 减少，无调用函数原本可能已被链接 GC 丢弃。
