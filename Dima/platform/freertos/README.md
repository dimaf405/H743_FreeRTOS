# FreeRTOS 平台适配

- **职责：** `Backend.cpp` 只实现 `platform/api` 的 TaskRuntime、Synchronization、CriticalSection、ExecutionContext、Heap 和 FlashTransactionManager；私有 `BackendTimeout.hpp` 负责公共超时到 FreeRTOS tick 的饱和换算，`HeapOperators.cpp` 负责全局 C++ 分配 ABI 与 malloc-failed hook。`storage/` 实现 `ParameterFileStore`，并由 FileStorage 后端互斥量串行化唯一 FatFs 客户端。该目录不拥有任何 MCU 外设。
- **依赖边界：** 根后端仅允许包含 `platform/api` 与 FreeRTOS；`storage/` 额外允许对象私有的 FatFs include。全目录禁止 HAL、CMSIS、STM32 寄存器、Board/Core/USB 生成头和业务模块。
- **固定资源：** 16 个 task slot、12 个 mutex slot、16 个 signal slot；八个 WorkQueue 加 `appMainTask` 使用 36 KiB 静态栈，任务栈来自 D1 中独立的 48 KiB `.dima_task_pool`，通用 `heap_5` 固定为 D1 中 256 KiB `.dima_heap`。
- **时间与超时：** 公共层只传微秒/毫秒和 `Timeout`；本后端向上取整到 1 kHz tick，不向调用者暴露 `TickType_t`、`TaskHandle_t` 或 `portMAX_DELAY`。
- **实时约束：** ISR 和标记为 realtime 的 WorkQueue 禁止动态分配；中间件与业务不得直接调用 FreeRTOS API。
- **硬件归属：** TIM2 HRT、cache、MPU、DMA、Flash、USB、UART、CAN、SPI 和 EXTI 的基础控制均位于 `platform/stm32h7`，通过公共 capability 使用；SBUS、UM982、ICM-42688-P、DroneCAN 等协议与设备策略属于 `lib/protocols` 和 `drivers`。
- **SD 边界：** `FatFsParameterFileStore.cpp` 只拥有 mount/file/rename/stat，并在介质错误后卸载旧对象；`Boards/H743/Src/fatfs_diskio.c` 独占 `hsd1`、HAL_SD 和 `.dima_dma` scratch buffer。所有调用仅由 `wq:storage` 或启动期 owner 发起；HAL 软件轮询采用 500 ms 有限截止，SD 故障不得占用 MAVLink 的 `wq:lp_default`。
