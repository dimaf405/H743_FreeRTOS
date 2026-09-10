# SerialConfig 模块

本目录从 `Dima/middleware/parameters/definitions/module_serial.yaml` 生成的 Dima 参数目录发现串口参数，并在模块启动时校验七路物理串口的波特率、功能、唯一 SBUS、GPS 和 MAVLink owner。`SERIAL5` 因板上没有 UART5 而保留为空号，USART6 仍固定对应 `SERIAL6`。

- 当前 Function 固定为 `0=Disabled`、`1=SBUS`、`2=GPS`、`3=MAVLink`；三种功能各自只能有一个 owner，同一个 UART 不能由多个端点同时占用。
- `SERIAL1/2/3/4/6/7/8_BAUD/FUNCTION` 只在标准 PX4 YAML 参数段定义；每对参数的描述明确记录同编号 UART/USART 与 TX/RX 引脚，不使用自定义 YAML 扩展。GPS 端口唯一由某一路 `SERIALx_FUNCTION=GPS` 指定；本产品固定使用 NMEA/UM982 实现，不提供额外的协议选择参数。
- 本模块只拥有参数绑定、组合校验和普通 8N1 配置的应用顺序；SBUS 与 UM982 driver 分别拥有协议专用线路切换和恢复事务。
- STM32 UART、DMA、IRQ 和 GPIO 基础控制位于 `Dima/platform/stm32h7/serial/`，不解释 SBUS、UM982 或 GPS 协议。
- baud 选项直接来自 YAML 生成的 QGC Metadata，运行时代码不维护第二份数值表；Function 只接受 Disabled/SBUS/GPS/MAVLink。配置无效时必须 fail-closed 停用冲突的数据链，同时保留 USB/QGC 恢复链。
- 关闭 Runtime 时必须清除参数绑定，不得把旧 Runtime 的参数缓存带入下一次启动。

MAVLink 默认不开启，参数写入顺序不限。BAUD 显式为非零值时数传直接以该速率运行；BAUD=Auto 时帧格式仍为 8N1，波特率由 MAVLink 服务按常见速率逐档自动探测（窗口内收到完整 MAVLink 帧即锁定），GPS 的 Auto 探测由 UM982 驱动承担。数传线路由独立异步端点应用，启动失败只关闭该链路。运行中串口变更先排空 UART 回应再申请未解锁维护许可；新驱动失败恢复旧快照。详细时序见 [双链路合同](../../../docs/MAVLINK_UART_ZH.md)。
