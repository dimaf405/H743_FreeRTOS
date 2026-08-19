# SerialConfig 模块

本目录负责把 `Boards/H743/serial_ports.json` 生成的串口参数合同绑定到 Application Runtime，并在模块启动时校验八路外部串口的波特率、功能和唯一 RC owner。

- 本模块只拥有参数绑定、组合校验和普通串口配置的应用顺序。
- STM32 UART、DMA、IRQ、GPIO 和 SBUS 电气接管仍由 `Dima/platform/stm32h7/serial/` 实现。
- 串口参数只允许生成合同中公开的波特率与功能；配置无效时必须阻止 RC 输入，同时保留 USB/QGC 恢复链。
- 关闭 Runtime 时必须清除参数绑定，不得把旧 Runtime 的参数缓存带入下一次启动。
