# 工作队列中间件

- **职责：** 提供控制、估计、传感器和服务任务的调度及执行时间监控。
- **禁止事项：** 不在实时队列运行 USB、Flash、日志等可能阻塞的服务，不允许实时回调运行期分配。
- **上游 API 保留：** 保留上游 WorkItem、ScheduledWorkItem 和队列配置接口的行为契约。
- **周期语义：** `ScheduleOnInterval()` 以前一截止时间锁相；超期跳过错过周期，不从 `Run()` 完成时刻累计漂移，也不突发补偿。
- **停止语义：** `ScheduleCancelAndDrain()` 持久拒绝新调度；外部 stop 等待正在执行的 `Run()` 返回，自身 `Run()` 内调用只取消、不自等待。重新启动必须先调用 `ScheduleEnable()`。
- **分辨率边界：** 普通延时等待由 1 kHz FreeRTOS tick 向上取整，正常负载下唤醒边界为一个 tick 加抢占延迟；亚毫秒高频链使用 DMA、EXTI、消息事件或未来 TIM2 CH1 compare。

## Application Runtime 所有权

- `work_queue_init()` 记录 Runtime owner task；`work_queue_shutdown()` 拒绝 ISR、任一 worker 自身和非 owner 调用，避免自删除、交叉 Runtime 回收或静态 task slot 复用竞态。
- `TaskRuntime::destroy()` 同步删除其他任务并返回结果，明确拒绝删除当前任务。只有 destroy 成功后才能释放对应 task handle、静态 stack block 和 Signal。
- Queue Runtime 只保存 POD `SignalHandle`，由 init/shutdown 显式 create/destroy；不得通过全局 `Signal` 析构器延长资源生命期或增加项目 `.init_array/.fini_array` 项。
- 模块在释放订阅、设备后端、Flash、日志或 Console 资源前必须 cancel-and-drain；shutdown 任一步失败时 ApplicationContext 保持 Error，并允许 owner 后续重试清理，不得直接重新 init。
