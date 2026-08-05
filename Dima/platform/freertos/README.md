# FreeRTOS 平台适配

- **职责：** 只实现 `platform/api` 的 TaskRuntime、Synchronization、CriticalSection、ExecutionContext、Heap 和 FlashTransactionManager，不拥有任何 MCU 外设。
- **依赖边界：** 仅允许包含 `platform/api` 与 FreeRTOS；禁止 HAL、CMSIS、STM32 寄存器、Board/Core/USB 生成头和业务模块。
- **固定资源：** 16 个 task slot、12 个 mutex slot、16 个 signal slot；任务栈来自 D1 中独立的 48 KiB `.dima_task_pool`，通用 `heap_5` 固定为 D1 中 256 KiB `.dima_heap`。
- **时间与超时：** 公共层只传微秒/毫秒和 `Timeout`；本后端向上取整到 1 kHz tick，不向调用者暴露 `TickType_t`、`TaskHandle_t` 或 `portMAX_DELAY`。
- **实时约束：** ISR 和标记为 realtime 的 WorkQueue 禁止动态分配；中间件与业务不得直接调用 FreeRTOS API。
- **硬件归属：** TIM2 HRT、cache、MPU、DMA、Flash、USB、SBUS 和 EXTI 均位于 `platform/stm32h7`，通过公共 capability 使用。
