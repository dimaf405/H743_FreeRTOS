# Rover 产品域

- **职责：** 负责差速车模块选择、静态或启动期创建、任务装配、产品默认配置以及 Rover 专属控制与导航功能。
- **目录边界：** `ApplicationContext.*` 是产品组合根；`control/` 只放消费 `rover_motion_request`、发布 `actuator_motors` 的运行控制器，纯算法位于 `Dima/lib/rover/`；`modes/manual/`、`modes/auto/`、`modes/auto_calibration/` 分别承载人工控制、Mission 导航和自动校准，所有模式统一使用一级职责子目录，根目录只保留组织说明。
- **模块边界：** Parameter、Log、RC 来源转换、MotorOutput、Commander 和 BootHealth 等具有独立生命周期的运行模块保留在 `Dima/modules`，不得移入本目录。
- **静态存储：** `ApplicationContext` 持有独立 `LogService` 静态实例的引用；日志对象及其 64 KiB Ring 位于 D1 的 `.dima_sram_bss`，紧随固定 D1 heap；48 KiB task pool 位于 DTCM。启动代码先清零，Services 安装后的组合根首次构造再建立日志对象；初始化、启停和回滚仍由组合根统一管理。
- **DTCM 热点对象：** `Ekf2`、`RoverDifferential`、`VehicleImu` 使用独立 `.dima_dtcm_bss.*` 原始存储，组合根持有引用；启动先清零，Services/heap 就绪后在原成员构造顺序中放置构造，运行期仍按原 start/stop 管理。动态 EKF RingBuffer 继续使用 D1 heap，整个静态 DTCM 低区不得越过 64 KiB MSP 预算边界。
- **禁止事项：** 不在装配层复制控制算法，不绕过参数、消息总线、安全状态和执行器链。
- **上游 API 保留：** 以适配和组合方式复用上游公开 API；自有产品代码使用 `dima::rover` 命名空间，不改写上游标识。
- **编译边界：** `ApplicationContext.hpp` 对独立存储、仅以引用持有的 LogService、VehicleImu、Ekf2、RoverDifferential 使用前置声明，完整实现由装配 `.cpp` 引入；不更改静态对象布局、首次构造顺序、owner 或 watchdog 调用路径。
