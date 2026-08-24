# RC 输入模块

- **职责：** `SbusRc` 接收原始帧，`RCUpdate` 完成校准和通道映射，`RcManualInput` 再把规范化通道及开关边沿发布为 `manual_control_setpoint` 与 `action_request`。
- **禁止事项：** 不从协议解析器直接驱动 PWM，不绕过 Arming、Failsafe 和控制模块链。
- **命名边界：** `RcManualInput` 是 RC 来源转换器，不是 Rover Manual 模式，也不拥有未来 MAVLink 输入；Rover 模式入口明确位于 `Dima/rover/modes/ManualMode.*`。
- **上游 API 保留：** 保留上游 SBUS、RCUpdate、ManualControl 的 Topic、参数名和公开状态语义，仅对本地类名和 UART/DMA 平台外壳做适配。

## 板级串口编号与 SBUS 配置

`Boards/H743/serial_ports.json` 是唯一串口清单。编号直接对应最新版 VCU-H7 原理图与 `H743_FreeRTOS.ioc` 的 STM32 外设尾号；`SERIAL7` 必须表示 UART7，禁止再按旧 ArduPilot `SERIAL_ORDER` 重新排列：

| 固定序号 | STM32 外设 | TX / RX | 板级连接器角色 | 参数默认值 |
|---:|---|---|---|---|
| SERIAL0 | USB OTG1 | USB | MAVLink USB | MavlinkService 独占，无 UART 波特率参数 |
| SERIAL1 | USART1 | PA9 / PA10 | 串口 1 | `SERIAL1_BAUD=921600`、Function Disabled |
| SERIAL2 | USART2 | PD5 / PD6 | 串口 2 | `SERIAL2_BAUD=Auto`、Function Disabled |
| SERIAL3 | USART3 | PD8 / PD9 | 串口 3 | `SERIAL3_BAUD=Auto`、Function Disabled |
| SERIAL4 | UART4 | PB9 / PB8 | 串口 4 | `SERIAL4_BAUD=115200`、Function Disabled |
| SERIAL5 | UART5 | PB13 / PB12 | 串口 5 | `SERIAL5_BAUD=115200`、Function Disabled |
| SERIAL6 | USART6 | PC6 / PC7 | SBUS / 串口 6 | `SERIAL6_BAUD=Auto`、Function RC Input |
| SERIAL7 | UART7 | PE8 / PE7 | 串口 7 | `SERIAL7_BAUD=57600`、Function Disabled |
| SERIAL8 | UART8 | PE1 / PE0 | 串口 8 | `SERIAL8_BAUD=115200`、Function Disabled |

UART5 同时引到独立串口 5 插座和串口 5/I2C2 复合插座，两处共享同一个 PB13/PB12 外设，不能由两个设备同时驱动 RX。各路普通 baud 是当前板级清单的产品默认值；这些默认值不代表尚未实现的 GPS、串口 MAVLink 或 RS485 服务。

每个外部端口固定生成 `SERIALx_BAUD` 和 `SERIALx_FUNCTION`。端口名称永远不随功能变化；当前只有 `0=Disabled`、`1=RC Input` 两个具有生产数据路径的 Function。通过 QGC 把目标端口设为 RC Input 时，固件会在同一参数事务中把旧 RC owner 设为 Disabled；如果存储数据本身异常地包含多个 RC owner，启动仍会 fail-closed。`RC_INPUT_PROTO` 再选择 `0=Disabled` 或 `2=SBUS`，默认 SBUS。

SerialConfig 在 RC driver 前应用普通 8N1 波特率。被唯一 `SERIALx_FUNCTION=RC Input` 选中的端口随后由 SBUS 临时接管为 `100000 bit/s、8E2、RX-only、RXINV enabled、RX pulldown`，释放时恢复普通 UART、FIFO 和 GPIO。Auto/0 只表示最终波特率交给对应 Function driver；当前未实现的 GPS、串口 MAVLink、RS485 数据服务不会因连接器名称而被虚构。

当前板级固件不定义 `RC_PORT_CONFIG`、迁移版本参数或旧串口键，也不扫描、补全或迁移旧存储目录。持久化快照中出现未知键或类型不符时整份拒绝，重新按当前 `SERIAL1..8` 直接编号配置。

接管前会保存 UART Init、AdvancedInit、FIFO 模式与阈值以及 RX GPIO 状态。协议禁用、模块停止、Runtime shutdown 或启动失败回滚时，DMA 和 IRQ 先关闭，再恢复保存的普通 UART 配置。恢复失败会保留接管上下文供下一次 stop 重试，并让 Application Runtime 保持 Error、禁止释放相关资源。主动禁用属于正常 Running 生命周期，不产生后端故障事件；Commander 仍会因为没有新鲜 RC 而保持不可解锁。

UART/DMA ISR 只复制字节、记录 TIM2 HRT 到达时间并唤醒 `wq:io`。格式化状态和故障日志由任务上下文产生；对外原始通道流只能由 MavlinkService 从 `input_rc` 转为 `RC_CHANNELS`，其他模块不得直接写 USB。

## 输入稳定与动作边界

SBUS 属于无强 CRC 的弱协议，冷启动、Failsafe 清除、UART/DMA 恢复或重新扫描后必须连续收到 3 个健康帧才建立协议锁定。锁定前的健康帧只用于重新同步，不发布为 `input_rc`；接收机显式 Failsafe 帧仍立即以 lost 发布。`frame-lost` 只累计跳帧，不单独判定整条链路失联。

协议锁定后，`RCUpdate` 还要求采样时间连续健康 100 ms 才把 `rc_channels.signal_lost` 清零。单次 UART PE/NE/FE 只丢弃可疑字节并重启 DMA，不立即发布 `rc_lost`；若从最后一份已发布健康帧起超过 `COM_RC_LOSS_T` 仍未恢复，`RCUpdate` 才判定 RC 丢失。接收机显式 Failsafe，以及 Ring 溢出、DMA/RTO/未知错误、重启或回滚失败等本机硬故障仍立即进入 lost/Error，不受 RC 断连延时掩盖。

Arm/Kill 离散状态必须至少两份严格前进且一致的样本，并保持 200 ms 才能进入边沿转换。Runtime 启动、RC 恢复以及映射/阈值变化后的首个稳定状态只建立基线；Arm 仅由稳定 OFF→ON 触发，Disarm 仅由稳定 ON→OFF 触发。

## QGC 校准与映射

MavlinkService 在原始样本新鲜且通道数有效时，从校准前的 `input_rc` 以 5 Hz 发送 `RC_CHANNELS`；接收机 failsafe/lost 标志仍让 `RCUpdate`/Commander 拒绝控制，但不会隐藏同时存在的原始通道，便于 QGC 校准和诊断。完全无帧、零通道或样本超时期间停流，恢复后立即发送。QGC 写回 `RC1..18_MIN/TRIM/MAX/REV`、`RC_CHAN_CNT` 和四个主控制映射；`RCUpdate` 在 `parameter_update` 后重新加载并继续执行范围、方向和通道有效性门禁。

默认只为差速 Rover 预设 `RC_MAP_THROTTLE=1`、`RC_MAP_YAW=2`，Roll/Pitch 默认未映射。Stock QGC 的校准向导仍固定识别四轴，因此保留 Roll/Pitch 参数和规范化消息字段；Rover 的 `manual_control_setpoint.valid`、解锁预检及运动控制只依赖中心双向 Throttle/Yaw，Roll/Pitch 不进入车辆输出。

Arm 只实现二段开关。启用 QGC Advanced UI 后在 Parameters 页面配置 `RC_MAP_ARM_SW=1..18` 和 `RC_ARMSWITCH_TH=-1..1`；正阈值高端为 ON，负阈值反向。Runtime 启动、RC 恢复或 Arm/Kill 映射及阈值变化后的第一份状态只建立基线，随后 OFF→ON 请求 Arm、ON→OFF 请求 Disarm，配置过程不会合成解锁边沿。当前没有可选择模式：`COM_FLTMODE1..6` 不再定义，`RC_MAP_FLTMODE` 仅保留既有 QGC/参数 handle 兼容且固定为 `0=Disabled`，RCUpdate 与 Commander 不消费 mode-slot。瞬时按键、长按 Toggle 和完整多模式能力不在本阶段。

`COM_RC_IN_MODE` 只允许 `0=RC only`，其他控制源模式在协议写入时拒绝、在存储加载时 fail-closed。`RC_MAP_FLAPS` 与已有 Aux 映射进入 `rc_channels`/`manual_control_setpoint`；`PARAM_MAP_RC` 在线参数调节不在本阶段。
