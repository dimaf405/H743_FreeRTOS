# Dima Rover

`Dima/` 是本仓库唯一的自研应用代码根，承载 STM32H743 + FreeRTOS 差速车的应用入口、产品装配、模块、平台适配、中间件、消息和算法库。

## 目录职责

- `application/`：应用任务入口和启动胶水。
- `product/rover/`：产品服务与装配根。
- `modules/`：BootHealth、HelloWorld 以及后续 RC、安全、Rover、Estimator 模块。
- `adapters/`：USB Console、MCUboot 等外部接口适配。
- `middleware/`：生命周期、消息、调度、uORB、Parameter、Event、Perf 和 Logging。
- `platform/freertos/`：FreeRTOS、时间、内存、Flash 和 libc 适配。
- `messages/`：共享消息数据结构与 uORB 声明。
- `lib/`：平台无关的算法、容器和移植库。

## 边界规则

- `Boards/`、`Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立，不归入产品目录。
- `Dima/modules` 不直接持有 STM32 HAL 全局句柄；硬件接线通过 Board 或 Adapter 暴露。
- `Dima/lib` 不依赖 HAL、FreeRTOS、USB 或 MCUboot。
- CubeMX 生成区只保留初始化和胶水，不承载产品业务逻辑。
- 新增自研应用代码不得恢复顶层 `App/` 目录。

## 命名与来源

- 产品目录使用 `Dima`，自有 C++ 命名空间使用 `dima`。
- 上游公开 API、参数名、消息名、版权头和许可证文字保持可追踪，不做品牌化全文替换。
- 上游代码导入须记录版本、原始路径、许可证和本地修改摘要。
