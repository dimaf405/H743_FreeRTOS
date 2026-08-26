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
| `serial/` | 通用 UART 线路配置、DMA/IRQ、收发、时间戳和统计，不解释具体协议 |
| `can/` | 通用 Classic CAN 配置、过滤、收发、错误恢复和统计 |
| `spi/` | 通用 SPI 模式/频率配置、同步或 DMA transaction 和统计 |
| `interrupts/` | 通用外部中断源、计数和时间戳 |
| `pwm/` | 六路 PWM capability 的薄物理适配 |
| `usb/` | USB CDC transport 的薄物理适配 |

`system/` 只接纳芯片级 time、identity、reset/boot 与 watchdog control-plane；
各总线目录只接纳不拥有设备协议和业务策略的基础控制。设备状态机、自动探测、
设备配置和协议策略统一属于 `Dima/drivers` 或 `Dima/lib/protocols`，不能回流到
STM32H7 平台层，也不能把 `system/` 当作杂项目录。平台内部依赖保持显式，
`system -> flash`、`flash -> memory`，其他跨域依赖须由 architecture gate 审核。

## 边界与所有权

- 本层只依赖 `platform/api`、HAL/CMSIS、Core/USB 生成头和 Boards 定义；禁止
  FreeRTOS、middleware、modules、rover 与 adapters 反向进入。
- `memory/cache.h`、`memory/early_memory.h`、`flash/flash_bank1.h` 和
  `flash/flash_fault.h` 是 Core、Application 与 MCUboot 共用的低级 C ABI，
  不是上层公共 capability。
- UART、CAN、SPI、DMA、IRQ 和 GPIO 细节不得进入公开 API；上层只消费通用
  capability。SBUS、UM982、ICM-42688-P、DroneCAN 等名称和策略不得出现在本目录。
- PWM 安全策略属于 `modules/motor`，设备状态机属于 `drivers`，纯协议编解码属于
  `lib/protocols`，USB 产品数据面属于 `modules/mavlink`。
- `flash/flash_fault.h` 是 `dima_flash_busfault_recover` 的唯一声明 owner；Core
  Fault 入口提供 fail-closed 弱实现，`FlashDevice.cpp` 只在安全读窗口内提供强实现。
- 所有翻译单元必须显式列入 `make/project.mk`；`cache.c` 与 `flash_bank1.c`
  还必须同步进入 `Bootloader/Makefile`，不得保留旧路径转发文件。
