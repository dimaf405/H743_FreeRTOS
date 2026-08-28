# SerialConfig 模块

本目录从 `Dima/middleware/parameters/definitions/module_serial.yaml` 生成的 Dima 参数目录发现串口参数，并在模块启动时校验七路物理串口的波特率、功能、唯一 SBUS owner 和唯一 GPS owner。`SERIAL5` 因板上没有 UART5 而保留为空号，USART6 仍固定对应 `SERIAL6`。

- 当前 Function 固定为 `0=Disabled`、`1=SBUS`、`2=GPS`；SBUS 和 GPS 各自只能有一个 owner，同一个 UART 不能由二者同时占用。
- `SERIAL1/2/3/4/6/7/8_BAUD/FUNCTION` 只在标准 PX4 YAML 参数段定义；每对参数的描述明确记录同编号 UART/USART 与 TX/RX 引脚，不使用自定义 YAML 扩展。GPS 端口唯一由某一路 `SERIALx_FUNCTION=GPS` 指定，`GPS_1_PROTOCOL` 仍属于 GPS 驱动参数 schema。
- 本模块只拥有参数绑定、组合校验和普通 8N1 配置的应用顺序；SBUS 与 UM982 driver 分别拥有协议专用线路切换和恢复事务。
- STM32 UART、DMA、IRQ 和 GPIO 基础控制位于 `Dima/platform/stm32h7/serial/`，不解释 SBUS、UM982 或 GPS 协议。
- baud 选项直接来自 YAML 生成的 QGC Metadata，运行时代码不维护第二份数值表；Function 只接受 Disabled/SBUS/GPS。配置无效时必须 fail-closed 停用冲突的数据链，同时保留 USB/QGC 恢复链。
- 关闭 Runtime 时必须清除参数绑定，不得把旧 Runtime 的参数缓存带入下一次启动。
