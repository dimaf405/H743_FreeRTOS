# Rover 产品域

- **职责：** 负责差速车模块选择、静态或启动期创建、任务装配、产品默认配置以及 Rover 专属控制与导航功能。
- **目录边界：** `ApplicationContext.*` 是产品组合根；阶段 5 控制与执行器组合进入 `control/`，后续 Rover 模式与导航进入 `navigation/`。
- **模块边界：** Parameter、Log、RC、Commander 和 BootHealth 等可跨产品复用的运行模块保留在 `Dima/modules`，不得移入本目录。
- **禁止事项：** 不在装配层复制控制算法，不绕过参数、消息总线、安全状态和执行器链。
- **上游 API 保留：** 以适配和组合方式复用上游公开 API；自有产品代码使用 `dima::rover` 命名空间，不改写上游标识。
