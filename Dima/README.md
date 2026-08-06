# Dima Rover

`Dima/` 是本仓库唯一的自研应用代码根，承载 STM32H743 + FreeRTOS 差速车的应用入口、产品装配、模块、平台适配、中间件、消息和算法库。

## 目录职责

- `application/`：应用任务入口和启动胶水。
- `rover/`：唯一 Rover 产品域；`control/` 放消费/发布消息的 Rover 控制运行模块，`modes/` 直接放 Manual、Navigation 等产品模式，`ApplicationContext.*` 只负责装配。
- `modules/`：Parameter、Log、BootHealth、RC、MotorOutput 和安全等已有运行服务；它们具有独立生命周期，但不包含 Rover 专属模式或混控算法。尚未实现的 Estimator 等只记录在 `docs/`，不创建占位源码目录。
- `adapters/`：只依赖公共 capability 的 USB Console 等外部协议适配。
- `middleware/`：生命周期、uORB、WorkQueue、Parameter、Event、Perf 和 Logging。
- `platform/api/`：不暴露 OS、MCU 或厂商类型的公共 capability 契约。
- `platform/freertos/`：Task、Mutex、Signal、Heap 和 Flash transaction 的 FreeRTOS 后端。
- `platform/stm32h7/`：启动内存契约、时钟、cache、DMA、Flash、USB、SBUS 和传感器中断后端。
- `messages/`：共享消息数据结构与 uORB 声明。
- `lib/`：平台无关的算法、容器和移植库；`lib/rover/` 只放纯 Rover 算法，不拥有 WorkQueue、uORB 或 Parameter 生命周期。

## 容易混淆的分层

- `lib/rc/` 只解析 SBUS 字节与帧；`modules/rc/` 才负责 UART capability、调度、参数和 Topic 发布。
- `middleware/logging/` 提供日志宏、过滤策略和固定 Ring；`modules/logging/` 是低优先级、有生命周期的转储服务。
- `middleware/parameters/` 提供 Parameter Core、生成输入、Autosave 和持久化适配；`modules/parameters/` 只负责 Runtime 生命周期和维护命令。
- `modules/rc/RcManualInput.*` 只拥有 RC 来源转换；`rover/modes/ManualMode.*` 才是 Rover Manual 模式。二者通过 `manual_control_setpoint` 解耦，未来 MAVLink 不反向依赖 RC。
- `lib/rover/` 提供当前已接入的纯差速算法；`rover/control/` 只放把消息、参数和安全状态接入算法的运行模块。未接入的闭环控制器不提前放进生产源码树。
- `modules/motor/` 拥有输出策略与安全生命周期，`platform/api` 定义 capability，`platform/stm32h7/ActuatorPwm.cpp` 适配 capability，`Boards/H743/Src/motor_pwm.c` 才拥有具体定时器和引脚。
- `platform/freertos/Backend.*` 是 RTOS Backend 类；`platform/stm32h7/HardwareServices.hpp` 只声明各硬件 capability 的工厂，不再使用第二个含糊的 `Backend.hpp`。
- MAVLink 接入时，纯协议编解码与 byte-stream 适配放在 `adapters/mavlink/`，uORB/Parameter/调度生命周期放在 `modules/mavlink/`，MCU UART/DMA 只留在 `platform/stm32h7/`；在实现前不创建空目录或把 MAVLink 塞入 RC、Logging 或 Rover control。

## 边界规则

- `Boards/`、`Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立，不归入产品目录。
- `application/`、`rover/`、`modules/`、`middleware/`、`messages/`、`lib/` 和 `adapters/` 只能依赖标准库、内部公共契约和 `platform/api`，不得包含 FreeRTOS、HAL、CMSIS、SCB/NVIC、Core、Board 或 USB 生成头。
- `platform/freertos` 不依赖 STM32/HAL/CMSIS；`platform/stm32h7` 不依赖 FreeRTOS 或业务模块。
- `Boards/H743/Src/platform_composition.cpp` 是具体后端与产品 capability 的唯一组合根。
- CubeMX 生成区只保留初始化和胶水，不承载产品业务逻辑。
- 根目录 `H743_FreeRTOS.ioc` 是唯一 CubeMX 工程；源码树不保留第二份 `.ioc`、README-only 规划目录或未列入 `make/project.mk` 的 Dima 翻译单元。
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
