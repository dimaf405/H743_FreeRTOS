# Rover 纯算法库

- **当前运行消费者：** `AutoMode` 在 `wq:nav` 以 50 Hz 调用 Pure Pursuit、Heading P 和停车/原地转向状态机；`RoverDifferential` 在 `wq:rate_ctrl` 以 100 Hz 调用 Speed PI、YawRate PI 与 `DifferentialDrive`，再发布两路电机请求。
- **证据边界：** 四个控制器、任务执行和 AUTO 安全投影已经进入正式固件闭包并通过 Windows 目标构建；这只是 `SOURCE / STATIC / WINDOWS BUILD VERIFIED`，不能替代 QGC 任务事务、SD 掉电恢复、目标板闭环或实车轨迹证明。
- **边界：** 本目录只含可脱离平台运行的算法和数据契约，不包含 HAL、FreeRTOS、uORB、Parameter、Flash 或动态模块生命周期代码。
- **失效语义：** 控制器遇到非有限输入、非法 `dt` 或未配置参数时复位 slew/积分并返回无效；PI 采用条件积分反饱和，零设定清空对应 PI/slew，原地转向必须先确认速度设定为零且实测速度低于 `RO_SPEED_TH`。
- **反向可行域：** `MOT_THR_ASYM>1` 时先在 `[-1/asymmetry, 1]` 电机域内完成 `RD_STR_THR_MIX` 饱和优先级，再作反向推力补偿、expo、最小/最大输出、换向等待和 Arm ramp，禁止先裁到 `[-1,1]` 后让两侧倒车同时饱和。
- **来源：** 优先保持上游 Rover 控制类型、单位和算法执行顺序；平台、消息和生命周期差异由 `Dima/rover` 运行层适配。
