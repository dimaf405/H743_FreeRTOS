# Dima Rover

`Dima/` 是本仓库唯一的自研应用代码根，承载 STM32H743 + FreeRTOS 差速车的应用入口、产品装配、模块、平台适配、中间件、消息和算法库。

## 目录职责

- `application/`：应用任务入口和启动胶水。
- `rover/`：唯一 Rover 产品域；`control/` 放消费/发布消息的 Rover 控制运行模块，`modes/` 直接放 Manual、Navigation 等产品模式，`ApplicationContext.*` 只负责装配。
- `drivers/`：设备驱动状态机与设备配置，当前按 GPS、IMU、磁力计和 RC 分类。
- `modules/`：Parameter、Mission、MAVLink、EKF2、Log、BootHealth、RC、Serial、MotorOutput 和安全等运行服务；它们具有独立生命周期，但不包含 Rover 专属模式或混控算法。
- `adapters/`：只依赖公共 capability 的 MAVLink、USB Console 等外部协议适配。
- `middleware/`：生命周期、uORB、WorkQueue、Parameter、Event、Perf 和 Logging。
- `platform/api/`：不暴露 OS、MCU 或厂商类型的公共 capability 契约，包括 Parameter 使用的窄 `AtomicFileStore` 与日志使用的 `LogFileStore` 接口。
- `platform/common/`：MCU/OS 无关的 capability 实现，只依赖 `platform/api/` 与平台无关库，不拥有板级资源或产品生命周期。
- `platform/freertos/`：Task、Mutex、Signal、Heap、Flash transaction 和 `storage/` FatFs 文件后端。
- `platform/stm32h7/`：根部只保留硬件工厂声明；`system/`、`memory/`、`flash/`、`serial/`、`can/`、`spi/`、`interrupts/`、`pwm/` 和 `usb/` 只承载 MCU 基础配置、读写、中断与统计。
- `messages/`：uORB schema 权威定义；Topic 头、metadata 和 catalog 由工具生成。
- `lib/`：平台无关的算法、容器、协议和移植库；`lib/protocols/` 只处理协议编解码，`lib/rover/` 只放纯 Rover 算法，不拥有 WorkQueue、uORB 或 Parameter 生命周期。

## 容易混淆的分层

- `lib/protocols/sbus/` 只解析 SBUS 字节与帧；`drivers/rc/sbus/` 负责串口线路配置、调度和原始 Topic 发布，`modules/rc/` 负责产品级 RC 转换。
- `middleware/logging/` 提供日志宏、过滤策略和固定 Ring；`modules/logging/` 是低优先级、有生命周期的转储服务。
- `middleware/parameters/` 提供 Parameter Core、生成输入、Autosave、FlashFS 和参数 SD 镜像；Mission 直接复用唯一 FlashFS，并按 PX4 `SYS_DM_BACKEND` 在板载 Flash/RAM/Disabled 三种 backend 间选择。
- `modules/rc/RcManualInput.*` 只拥有 RC 来源转换；`rover/modes/ManualMode.*` 才是 Rover Manual 模式。二者通过 `manual_control_setpoint` 解耦，未来 MAVLink 不反向依赖 RC。
- `lib/rover/` 提供 Pure Pursuit、Speed PI、Heading P、YawRate PI、停车/原地转向状态机和差速混控纯算法；`rover/modes/AutoMode` 与 `rover/control/RoverDifferential` 只负责把任务、估计、参数、消息和安全状态接入这些算法。
- `modules/motor/` 拥有输出策略与安全生命周期，`platform/api` 定义 capability，`platform/stm32h7/pwm/ActuatorPwm.cpp` 适配 capability，`Boards/H743/Src/motor_pwm.c` 才拥有具体定时器和引脚。
- `platform/freertos/Backend.*` 是 RTOS Backend 类；`platform/stm32h7/HardwareServices.hpp` 只声明各硬件 capability 的工厂，不再使用第二个含糊的 `Backend.hpp`。
- `platform/freertos/storage/` 连接 Parameter `AtomicFileStore`、日志 `LogFileStore` 与 FatFs/FreeRTOS；`Boards/H743/Src/fatfs_diskio.c` 才连接 FatFs disk ABI 与 SDMMC/HAL。Mission 运行链不依赖该目录。
- MAVLink 接入时，纯协议编解码与 byte-stream 适配放在 `adapters/mavlink/`，uORB/Parameter/调度生命周期放在 `modules/mavlink/`，MCU UART/DMA 只留在 `platform/stm32h7/serial/`；在实现前不创建空目录或把 MAVLink 塞入 RC、Logging 或 Rover control。

## 边界规则

- `Boards/`、`Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立，不归入产品目录。
- `application/`、`rover/`、`modules/`、`middleware/`、`messages/`、`lib/` 和 `adapters/` 只能依赖标准库、内部公共契约和 `platform/api`，不得包含 FreeRTOS、HAL、CMSIS、SCB/NVIC、Core、Board 或 USB 生成头。
- `platform/freertos` 不依赖 STM32/HAL/CMSIS；只有 `storage/` 可使用对象私有的 FatFs include。`platform/stm32h7` 不依赖 FreeRTOS 或业务模块。
- `Boards/H743/Src/platform_composition.cpp` 是具体后端与产品 capability 的唯一组合根。
- CubeMX 生成区只保留初始化和胶水，不承载产品业务逻辑。
- 根目录 `H743_FreeRTOS.ioc` 是唯一 CubeMX 工程；源码树不保留第二份 `.ioc`、README-only 规划目录或未列入 `make/project.mk` 的 Dima 翻译单元。
- USB CDC 是系统调试日志与维护命令口，不运行周期性示例输出；实时路径只上报固定结构事件，由 LP 日志服务有界格式化和发送。
- `make check-architecture` 强制检查源码标识、硬件操作所有权和各层私有 include 集；`firmware`、`verify`、`dima_rover` 均以该门禁为前置条件。
- 新增自研应用代码不得恢复顶层 `App/` 目录。

## 头文件引用

- 同目录头文件引用只写文件名。
- 跨目录 include 最多保留一层职责名，例如 `uORB/Publication.hpp`、`sbus/SbusRc.hpp`。
- 禁止两层及以上的首方 include；不得写 `platform/api/Execution.hpp`、完整 `Dima/...` 或 `Boards/H743/Inc/...` 路径。存在同名头时通过精确 include root 和一层职责名消歧，不扩大为仓库全局路径。
- 自有 `.hpp` 只保留声明、类型、模板、`constexpr` 和极短访问器；普通函数、协议状态机与算法实现放入同名 `.cpp`。
- 只有模板、生成代码或经文档明确说明的上游合同可以采用 header-only；不得用 header-only 规避 `make/project.mk` 的显式翻译单元清单。
- 私有实现头只在所属后端目录内共享，不属于公共 include 面；同目录使用 basename，跨一个子域时也必须遵守最多一层规则。

## 命名与来源

- 产品目录使用 `Dima`，自有 C++ 命名空间使用 `dima`。
- 上游公开 API、参数名、消息名、版权头和许可证文字保持可追踪，不做品牌化全文替换。
- 上游代码导入须记录版本、原始路径、许可证和本地修改摘要。
