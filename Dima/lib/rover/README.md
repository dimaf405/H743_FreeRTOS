# Rover 纯算法库

- **当前生效：** `DifferentialDrive` 承担两轴差速混控、输出整形与保护，由 `Dima/rover/control/RoverDifferential` 运行模块调用。
- **未接入能力：** 阶段 8 的 Speed、Yaw Rate 与 Heading 闭环只保留在计划文档中；在状态估计和运行接口确定前不放置预实现源码，避免把未被调用的控制器误认为生产链路。
- **边界：** 本目录只含可脱离平台运行的算法和数据契约，不包含 HAL、FreeRTOS、uORB、Parameter、Flash 或动态模块生命周期代码。
- **反向可行域：** `MOT_THR_ASYM>1` 时先在 `[-1/asymmetry, 1]` 电机域内完成 `RD_STR_THR_MIX` 饱和优先级，再作反向推力补偿、expo、最小/最大输出、换向等待和 Arm ramp，禁止先裁到 `[-1,1]` 后让两侧倒车同时饱和。
- **来源：** 优先保持上游 Rover 控制类型、单位和算法执行顺序；平台、消息和生命周期差异由 `Dima/rover` 运行层适配。
