# STM32H7 平台后端

本目录只负责把 `platform/api` capability 连接到 STM32H743 的 HAL、CMSIS、
CubeMX 接线和板级原语，不拥有 Rover 策略、模块生命周期或 FreeRTOS 对象。
`HardwareServices.hpp` 是唯一保留在根目录的硬件工厂声明面；真正安装 capability
的组合根仍是 `Boards/H743/Src/platform_composition.cpp`。

## 目录职责

| 目录 | 职责 |
|---|---|
| `system/` | MCUboot 镜像确认与复位、TIM2 单调时钟、硬件 UID、IWDG |
| `memory/` | 启动期 MPU/cache 契约、共享 cache C ABI、DMA non-cache 区与 bounce buffer |
| `flash/` | Bank 2 参数分区、ECC 安全读、BusFault 恢复和 Bank 1 RAM 写入原语 |
| `serial/` | 普通 UART 与 SBUS 电气接管、DMA/IRQ 状态机、RX GPIO 映射 |
| `io/` | 六路 PWM capability、传感器 EXTI 和 USB CDC transport 的薄适配 |

`system/` 只接纳芯片级 time、identity、reset/boot 与 watchdog control-plane；
`io/` 只接纳不拥有协议和业务策略的单 TU 薄物理适配。若实现开始共享私有状态、
扩展为多翻译单元驱动或取得协议所有权，必须像 `serial/` 一样独立成域，不能把
`system/` 或 `io/` 当作杂项目录。当前组间依赖只允许
`io/serial -> system/memory`、`system -> flash`、`flash -> memory`。

## 边界与所有权

- 本层只依赖 `platform/api`、HAL/CMSIS、Core/USB 生成头和 Boards 定义；禁止
  FreeRTOS、middleware、modules、rover 与 adapters 反向进入。
- `memory/cache.h`、`memory/early_memory.h`、`flash/flash_bank1.h` 和
  `flash/flash_fault.h` 是 Core、Application 与 MCUboot 共用的低级 C ABI，
  不是上层公共 capability。
- `serial/SbusUartPrivate.hpp` 只允许 `SbusUart.cpp` 与 `SbusUartHal.cpp` 使用；
  UART、DMA、IRQ 和 GPIO 细节不得进入公开 API。
- `io/` 只做物理外设适配。PWM 安全策略属于 `modules/motor`，RC 状态机属于
  `modules/rc`，USB 数据面属于 `modules/mavlink`。
- `flash/flash_fault.h` 是 `dima_flash_busfault_recover` 的唯一声明 owner；Core
  Fault 入口提供 fail-closed 弱实现，`FlashDevice.cpp` 只在安全读窗口内提供强实现。
- 所有翻译单元必须显式列入 `make/project.mk`；`cache.c` 与 `flash_bank1.c`
  还必须同步进入 `Bootloader/Makefile`，不得保留旧路径转发文件。
