# FreeRTOS 平台适配

- **职责：** `Backend.cpp` 只实现 `platform/api` 的 TaskRuntime、Synchronization、CriticalSection、ExecutionContext、Heap 和 FlashTransactionManager；私有 `BackendTimeout.hpp` 负责公共超时到 FreeRTOS tick 的饱和换算，`HeapOperators.cpp` 负责全局 C++ 分配 ABI 与 malloc-failed hook。`storage/` 在同一 FatFs volume owner 上实现 Parameter `AtomicFileStore` 与日志 `LogFileStore`；PX4 Dataman Mission 直接使用 FlashFS/RAM，不经过 FatFs。该目录不拥有任何 MCU 外设。
- **依赖边界：** 根后端仅允许包含 `platform/api` 与 FreeRTOS；`storage/` 额外允许对象私有的 FatFs include。全目录禁止 HAL、CMSIS、STM32 寄存器、Board/Core/USB 生成头和业务模块。
- **固定资源：** 16 个 task slot、12 个 mutex slot、16 个 signal slot；八个 WorkQueue 加 `appMainTask` 使用 36 KiB 静态栈，任务栈来自 D1 中独立的 48 KiB `.dima_task_pool`，通用 `heap_5` 固定为 D1 中 256 KiB `.dima_heap`。
- **时间与超时：** 公共层只传微秒/毫秒和 `Timeout`；本后端向上取整到 1 kHz tick，不向调用者暴露 `TickType_t`、`TaskHandle_t` 或 `portMAX_DELAY`。
- **实时约束：** ISR 和标记为 realtime 的 WorkQueue 禁止动态分配；中间件与业务不得直接调用 FreeRTOS API。
- **硬件归属：** TIM2 HRT、cache、MPU、DMA、Flash、USB、UART、CAN、SPI 和 EXTI 的基础控制均位于 `platform/stm32h7`，通过公共 capability 使用；SBUS、UM982、ICM-42688-P、DroneCAN 等协议与设备策略属于 `lib/protocols` 和 `drivers`。
- **SD 边界：** `FatFsAtomicFileStore.cpp` 独占 FATFS、参数镜像文件、ULog writer、下载 reader 和目录扫描对象，公共 backend mutex 串行所有物理调用；`Boards/H743/Src/fatfs_diskio.cpp` 独占 `hsd1`、HAL_SD 识卡以及 D1 `.dima_sd_dma` 中的两个 4 KiB 缓冲。运行期 I/O 只在 `wq:storage` 推进；MAVLink 仅交换固定请求/响应 Ring，Mission 与导航不以 SD 卡存在为前提，运行期 IDMA/命令事务共享 500 ms 有限截止，识卡仍沿用 HAL。板上没有 card-detect GPIO，因此 `disk_status()==0` 只表示旧会话未被判错；复用挂载前必须用同样受限的 `CTRL_SYNC` 主动命令确认“当前可用”，不能对外声称已证明物理卡在位。

- **运行时间统计：** 复用 TIM2 的 1 us 计数，最多每秒在非实时上下文枚举平台任务槽及内核 Idle/Timer，关闭栈高水位扫描，并采样固定任务表；CpuUsage/TaskCpuUsage 不暴露原生 TCB。窗口回绕、任务代次变化或超过 60 s 时重建基线，SYS_STATUS 保留上次有效负载。统计采用内核任务归账口径，SD IRQ 开销由板级诊断另记。
