# 消息总线中间件

- **职责：** 提供发布、订阅、队列、多实例、代次和回调调度能力。
- **禁止事项：** 不让业务模块直接依赖 HAL，不在实时发布路径执行动态分配或阻塞操作。
- **上游 API 保留：** 保留上游 Topic 结构、消息字段和发布订阅接口语义；后端可由 Dima FreeRTOS 实现替换。
- 普通 `Publication` 允许多个发布者共享同一 Topic instance；Runtime 以引用计数维护广告存续，任一发布者释放时不得使其他发布者失效。`PublicationMulti` 只从尚无发布者的 instance 中分配。

## Application Runtime 生命周期

- 每次成功 `initialize()` 推进上电期单调 lifecycle epoch；`shutdown()` 释放当前 Runtime 的 Topic Buffer 和 instance 状态，但 epoch 不回退。
- Subscription 检测 epoch 变化后清空 generation、callback 和类型缓存；Publication/PublicationMulti 丢弃旧广告或 instance，并在新 Runtime 重新广告；每个 instance 的发布者引用计数在 initialize、失败回滚和 shutdown 时清零。
- `newest == 0` 时不得复制空槽；generation 为 0、超前或落后于队列最旧有效样本时，从当前最旧有效样本重新同步，保证深度 8 Topic 在 restart 后可恢复。
- 模块 stop 必须先注销 callback/订阅，再由 ApplicationContext 执行 uORB shutdown；旧 Runtime 的指针、generation 和广告句柄禁止跨边界复用。
- Topic 布局、消息字段、队列深度和公开 API 不因 epoch 契约变化；最终 Heap 分配量仍以 Windows 原生 clean build/ELF 为准。
