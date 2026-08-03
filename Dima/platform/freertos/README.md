# FreeRTOS 平台适配

- **职责：** 承载任务、时间、内存、同步原语以及 STM32H743 FreeRTOS 平台兼容实现。
- **禁止事项：** 不放置车辆控制算法，不在 ISR 或实时控制路径引入无界阻塞和未受控动态分配。
- **上游 API 保留：** 适配上游平台接口时保留其公开类型、函数、宏和调用语义，仅替换操作系统及板级实现。
- **双时基：** SysTick 固定 1 kHz 并统一 HAL/FreeRTOS tick；TIM2 固定 1 MHz、32 位并扩展为 64 位 HRT。TIM12 不再承担 HAL timebase。
- **资源保留：** TIM2 和 CH1 专用于 HRT/未来 compare；当前禁止 tickless、STOP 补偿和运行期动态改频，不再增加第三套系统时基。
- **传感器中断：** ICM42688 的 EXTI ISR 只记录 TIM2 HRT 时间戳、事件位和计数，并通过 `ScheduleNowFromISR()` 通知任务；SPI 事务只允许在任务上下文执行。
