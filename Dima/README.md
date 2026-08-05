# Dima Rover

`Dima/` 是本仓库唯一的自研应用代码根，承载 STM32H743 + FreeRTOS 差速车的应用入口、产品装配、模块、平台适配、中间件、消息和算法库。

## 目录职责

- `application/`：应用任务入口和启动胶水。
- `rover/`：唯一 Rover 产品域，包含产品装配以及后续 `control/`、`navigation/` 专属功能。
- `modules/`：Parameter、Log、BootHealth、RC、安全和 Estimator 等可复用运行模块。
- `adapters/`：只依赖公共 capability 的 USB Console 等外部协议适配。
- `middleware/`：生命周期、uORB、WorkQueue、Parameter、Event、Perf 和 Logging。
- `platform/api/`：不暴露 OS、MCU 或厂商类型的公共 capability 契约。
- `platform/freertos/`：Task、Mutex、Signal、Heap 和 Flash transaction 的 FreeRTOS 后端。
- `platform/stm32h7/`：启动内存契约、时钟、cache、DMA、Flash、USB、SBUS 和传感器中断后端。
- `messages/`：共享消息数据结构与 uORB 声明。
- `lib/`：平台无关的算法、容器和移植库。

## 边界规则

- `Boards/`、`Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立，不归入产品目录。
- `application/`、`rover/`、`modules/`、`middleware/`、`messages/`、`lib/` 和 `adapters/` 只能依赖标准库、内部公共契约和 `platform/api`，不得包含 FreeRTOS、HAL、CMSIS、SCB/NVIC、Core、Board 或 USB 生成头。
- `platform/freertos` 不依赖 STM32/HAL/CMSIS；`platform/stm32h7` 不依赖 FreeRTOS 或业务模块。
- `Boards/H743/Src/platform_composition.cpp` 是具体后端与产品 capability 的唯一组合根。
- CubeMX 生成区只保留初始化和胶水，不承载产品业务逻辑。
- USB CDC 是系统调试日志与维护命令口，不运行周期性示例输出；实时路径只上报固定结构事件，由 LP 日志服务有界格式化和发送。
- `make check-architecture` 强制检查源码标识、硬件操作所有权和各层私有 include 集；`firmware`、`verify`、`dima_rover` 均以该门禁为前置条件。
- 新增自研应用代码不得恢复顶层 `App/` 目录。

## 头文件引用

- 同目录头文件引用只写文件名。
- 跨目录引用从职责层根目录开始，例如 `uorb/Publication.hpp`、`rc/SbusRc.hpp`。
- 不使用完整的 `Dima/...` 或 `Boards/H743/Inc/...` include 写法。

## 命名与来源

- 产品目录使用 `Dima`，自有 C++ 命名空间使用 `dima`。
- 上游公开 API、参数名、消息名、版权头和许可证文字保持可追踪，不做品牌化全文替换。
- 上游代码导入须记录版本、原始路径、许可证和本地修改摘要。
