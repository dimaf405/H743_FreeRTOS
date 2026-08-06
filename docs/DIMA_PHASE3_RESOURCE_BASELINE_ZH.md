# Dima 阶段 3 资源与验收基线

日期：2026-07-31

## 1. 完成范围

阶段 3 已完成以下生产模块链：

```text
STM32 UART RXINV + DMA1 Stream2
→ SbusRc
→ input_rc
→ RCUpdate
→ rc_channels + manual_control_switches
→ RcManualInput（PX4 ManualControl RC 子集）
→ manual_control_setpoint + action_request
```

- 默认端口：`UART4 RX / PB8`；另支持 UART7/PE7、UART8/PE0、USART2/PD6，重启生效。
- SBUS：100000 baud、8E2 语义、硬件 RXINV、25-byte 帧、18 通道、Failsafe 和 Frame-lost。
- `RC_CHAN_CNT=0` 自动采用接收机有效通道数；参数生成结果无固定 64 项容量。
- 18 通道校准、功能映射和参数热更新已接入。无效校准只禁用对应通道；无效映射和缺失通道 fail-closed。
- Throttle/Yaw 保持中心双向 `[-1,1]`，支持后续倒车和原地旋转。
- Arm/Kill 只发布 `action_request`；本阶段没有 Commander 消费者，也没有启动电机 PWM。
- RC 链启动失败只降级手动输入，BootHealth、Parameter、Logging 和 MCUboot 恢复链继续运行。

## 2. 最终构建结果

执行：

```text
make parameter-generated
make build/H743_FreeRTOS.elf GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
make firmware GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
make verify GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
```

结果：全部通过；没有运行或新增 Host Test、SITL 或仿真。

```text
.text（arm-size 汇总口径）   130,900 bytes
.data                          7,596 bytes
.bss                         330,776 bytes
ELF                        1,778,376 bytes（含调试段）
BIN                          138,536 bytes
Signed BIN                   139,710 bytes
Factory HEX                  440,344 bytes
MCUboot ELF                2,588,748 bytes（含调试段）
MCUboot BIN                   45,664 bytes
```

镜像验证：

```text
Image version: 0.1.0+0
Image digest: 8ed9573896f94d89968b50b76cea5b6ef410bd505e7b2858a3d6f2cfeae0f6db
bootloader: 45,664 bytes @ 0x08000000
signed app: 139,710 bytes @ 0x08040000
app vector: 0x08040400
```

Signed BIN 占 768 KiB Application Slot 约 17.8%，低于 85% 控制线。

## 3. 内存与中间件资源

```text
.dima_heap  0x24000000～0x24040000，256 KiB，D1 AXI SRAM
.dima_dma   0x30000000～0x30000040，64 bytes，D2 SRAM
```

- SBUS DMA Buffer 为固定 64 bytes、32-byte 对齐，不使用动态内存。
- SPI4 使用 DMA1 Stream0/1；SBUS 使用 DMA1 Stream2，不冲突。
- 阶段 3 没有新建 FreeRTOS Task：SbusRc 与 RCUpdate 复用 `wq:io`，当前命名为 `RcManualInput` 的 ManualControl RC 子集复用 `wq:hp_default`。
- 七个 WorkQueue 静态栈总量仍为 28,672 bytes。
- Perf 固定池为 4,608 bytes；RC 链启动后占用 7 个 Counter：SBUS 字节、完整帧、非法帧、丢帧、UART 错误、`input_rc` 间隔和 `rc_channels` 间隔。
- Event Ring 仍为 128 条；阶段 3 增加 UART/DMA、Failsafe、RC loss、校准、映射和 Topic 发布失败事件。

阶段 3 新增五个 Topic 的单实例数据容量：

| Topic | 结构大小 | 队列深度 | 单实例 Buffer |
|---|---:|---:|---:|
| `input_rc` | 80 | 1 | 80 |
| `rc_channels` | 128 | 1 | 128 |
| `manual_control_setpoint` | 72 | 1 | 72 |
| `manual_control_switches` | 40 | 1 | 40 |
| `action_request` | 16 | 8 | 128 |

阶段 3 Topic 单实例合计 448 bytes；uORB 为每个 Topic 预分配四实例，因此阶段 3 新增 D1 Heap Topic Buffer 为 1,792 bytes。当前全部 7 个 Topic（含 heartbeat、parameter_update）Buffer 合计 2,016 bytes。

## 4. 静态验收

- 参数生成结果：135 项；生成目录可重新生成，生成头不再通过 DrvFS 转发 include 产生 `E:/...` 依赖路径。
- 连续两次增量目标构建通过，第二次没有重新编译。
- `.dima_dma` 链接到 D2 SRAM，`.dima_heap` 保持在 D1 AXI SRAM。
- UART/DMA ISR 只更新位置/错误状态、调用 HAL IRQ 和 `ScheduleNowFromISR()`；不解析 SBUS、不发布 uORB、不访问参数、不格式化日志、不分配内存。
- RC 生产目录中没有 `malloc/calloc/realloc/new/delete/pvPortMalloc`；`perf_alloc` 只使用固定 64 项 Perf 对象池，并仅在模块启动阶段调用。
- ApplicationContext 和 RC 模块没有调用 Motor PWM、Mixer、Actuator 或 RoverDifferential。
- 阶段 3 新增路径名称不包含大小写不敏感的 `px4`。
- 没有新增测试文件、测试框架、SITL 或仿真入口。

## 5. 尚待目标板人工验收

以下项目无法仅靠本地构建确认，阶段状态明确为“代码与目标镜像完成，板测待执行”：

1. UART4/PB8 默认反相 SBUS 锁定，以及 UART7/UART8/USART2 重启切换。
2. 接收机断电、恢复、遥控器关机和重连不重启固件。
3. 18 通道、数字通道 17/18、Failsafe 和丢帧计数实测。
4. USB 在线修改校准/映射后 Topic 即时更新。
5. Throttle/Yaw 中位为 0、端点接近 -1/+1。
6. 0.5 s 失联窗口和 `manual_control_setpoint.valid=false`。
7. Arm/Kill 边沿各只产生一次 `action_request`。
8. 所有 PWM 在整个阶段 3 仍保持未启动。

目标板人工验收不通过新增测试框架、SITL 或仿真实现。
