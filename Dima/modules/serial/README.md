# SerialConfig 模块

本目录负责把 `Boards/H743/serial_ports.json` 生成的串口参数合同绑定到 Application Runtime，并在模块启动时校验八路外部串口的波特率、功能、唯一 RC owner 和唯一 GPS owner。

- 当前 Function 固定为 `0=Disabled`、`1=RC Input`、`2=GPS`；RC 和 GPS 各自只能有一个 owner，同一个 UART 不能由二者同时占用。
- `SERIAL1..8_BAUD/FUNCTION` 与板级拓扑参数 `GPS_1_CONFIG` 均只由串口 manifest 生成；`GPS_1_PROTOCOL` 仍属于 GPS 驱动参数 schema。
- 本模块只拥有参数绑定、组合校验和普通 8N1 配置的应用顺序；SBUS 与 UM982 driver 分别拥有协议专用线路切换和恢复事务。
- STM32 UART、DMA、IRQ 和 GPIO 基础控制位于 `Dima/platform/stm32h7/serial/`，不解释 SBUS、UM982 或 GPS 协议。
- 串口参数只允许生成合同中公开的 baud 与 Function；配置无效时必须 fail-closed 停用冲突的 RC/GPS 数据链，同时保留 USB/QGC 恢复链。
- 关闭 Runtime 时必须清除参数绑定，不得把旧 Runtime 的参数缓存带入下一次启动。
