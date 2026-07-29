# Dima Rover 阶段 0 资源基线

- 日期：2026-07-29
- 平台：STM32H743VI + FreeRTOS
- 工具链：GNU Arm Embedded 10.2.1 (`gcc-arm-none-eabi-10-2020-q4-major`)
- 构建入口：`GNUmakefile` + `make/project.mk`
- Flash 布局：保持现状，阶段 0 未修改

## 1. 构建状态

应用 ELF、BIN 和 HEX 已于 2026-07-29 16:00 重新生成：

```text
build/H743_FreeRTOS.elf   1,084,056 bytes（包含调试段）
build/H743_FreeRTOS.bin      62,600 bytes
build/H743_FreeRTOS.hex     176,053 bytes
```

应用链接目标单独复核通过：

```text
make build/H743_FreeRTOS.elf \
  GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
```

完整 `make firmware` 在应用产物生成后，阻塞于 MCUboot 主机工具目录更新：

```text
mv: cannot move 'build/host-python' to
    'build/host-python.old.<pid>': Permission denied
```

因此本次确认了应用编译和链接，但没有把旧的 Signed BIN 和 Factory HEX 作为新基线。阶段 1 前需要单独修复 WSL/Windows 下 `build/host-python` 的原子替换权限，再重新生成签名和工厂镜像；不得通过删除用户工具目录来掩盖问题。

## 2. 应用 Flash 基线

`arm-none-eabi-size`：

```text
text     60,120 bytes
data      2,440 bytes
bss      21,228 bytes
dec      83,788 bytes
```

主要链接段：

```text
.isr_vector        664 bytes
.text           58,996 bytes
.rodata            452 bytes
.data             2,428 bytes
.bss             20,204 bytes
._user_heap_stack 1,024 bytes
```

Application Slot：

```text
Slot 容量                  786,432 bytes（768 KiB）
当前 BIN                    62,600 bytes
当前占用                      7.96 %
物理剩余                   723,832 bytes
85% 发布预算               668,467 bytes
到 85% 预算线的余量         605,867 bytes
```

85% 是产品预算线，不是链接器物理上限。正式引入 Dima middleware、Rover 模块和多实例 EKF2 后，需要在每个阶段更新本文件。

## 3. SRAM 基线

当前链接脚本将普通 `.data/.bss` 放入 DTCM：

```text
DTCM 容量                   131,072 bytes（128 KiB）
.data + .bss + heap/stack    23,656 bytes
当前占用                      18.05 %
估算剩余                   107,416 bytes
```

D1、D2、D3 尚未形成明确的应用输出段基线。阶段 1 必须先建立：

- D1 AXI SRAM：受控通用 Heap 和非 DMA 大对象。
- D2 SRAM：DMA Buffer 和显式 Cache/MPU 策略。
- DTCM：控制状态、关键栈和非 DMA 快速数据。
- EKF2 多实例缓冲不得默认堆入 DTCM。

## 4. 当前静态任务栈

```text
appMainTask       2,048 bytes
wq:hp_default     2,048 bytes
wq:lp_default     2,048 bytes
Idle Task           512 bytes（128 StackType_t）
Timer Task         1,024 bytes（256 StackType_t）
```

以上是配置容量，不是运行期高水位。后续引入 `wq:rate_ctrl`、`wq:estimator`、`wq:sensors`、`wq:nav` 和服务任务时，必须记录实际栈高水位。

## 5. EKF2 多实例预算规则

首版 EKF2 不采用单实例专用架构：

- 从首次移植起包含多实例管理和 Estimator Selector。
- 最少支持两个 EKF 实例。
- 实际激活数量由可用 IMU/Mag 组合和参数决定。
- 若早期硬件只有一组健康传感器，可以只激活一个实例，但消息、参数、调度和资源模型仍按多实例实现。
- 阶段 0 资源规划按至少两个实例预留，不使用单实例数据作为最终容量依据。

## 6. Flash 分区基线

```text
MCUboot       0x08000000  256 KiB
Primary       0x08040000  768 KiB
Secondary     0x08100000  768 KiB
Scratch       0x081C0000  128 KiB
Storage       0x081E0000  128 KiB
```

阶段 0 未更改任何地址或大小。参数掉电安全如需第二个擦除区，优先评估外部 NVM，不立即缩小 Primary/Secondary Slot。
