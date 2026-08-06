# RC 输入模块

- **职责：** `SbusRc` 接收原始帧，`RCUpdate` 完成校准和通道映射，`RcManualInput` 再把规范化通道及开关边沿发布为 `manual_control_setpoint` 与 `action_request`。
- **禁止事项：** 不从协议解析器直接驱动 PWM，不绕过 Arming、Failsafe 和控制模块链。
- **命名边界：** `RcManualInput` 是 RC 来源转换器，不是 Rover Manual 模式，也不拥有未来 MAVLink 输入；Rover 模式入口明确位于 `Dima/rover/modes/ManualMode.*`。
- **上游 API 保留：** 保留上游 SBUS、RCUpdate、ManualControl 的 Topic、参数名和公开状态语义，仅对本地类名和 UART/DMA 平台外壳做适配。

## SBUS 端口与电气配置

`RC_INPUT_PROTO` 是协议开关：`0=Disabled`、`2=SBUS`，默认值为 `2`。`RC_PORT_CONFIG` 选择接收端口：`1=UART4/PB8`、`2=UART7/PE7`、`3=UART8/PE0`、`4=USART2/PD6`；值 `0` 只兼容旧参数分区中的安全禁用配置，新配置统一使用 `RC_INPUT_PROTO=0`。

通过 USB 维护口选择 UART4/PB8 并启用 SBUS：

```text
param set RC_PORT_CONFIG 1
param set RC_INPUT_PROTO 2
param save
```

禁用 RC 输入：

```text
param set RC_INPUT_PROTO 0
param save
```

两项参数都要求 Runtime 重启或 MCU 重启后生效。项目不再提供手动极性参数；选择 SBUS 后后端固定接管为原始反相 SBUS 所需的 `100000 bit/s、8E2、RX-only、RXINV enabled、RX pulldown`。

接管前会保存 UART Init、AdvancedInit、FIFO 模式与阈值以及 RX GPIO 状态。协议禁用、模块停止、Runtime shutdown 或启动失败回滚时，DMA 和 IRQ 先关闭，再恢复保存的普通 UART 配置。恢复失败会保留接管上下文供下一次 stop 重试，并让 Application Runtime 保持 Error、禁止释放相关资源。主动禁用属于正常 Running 生命周期，不产生后端故障事件；Commander 仍会因为没有新鲜 RC 而保持不可解锁。

UART/DMA ISR 只复制字节、记录 TIM2 HRT 到达时间并唤醒 `wq:io`。格式化状态和故障日志由任务上下文产生，连续通道数据只由低优先级 `LogService` 输出。
