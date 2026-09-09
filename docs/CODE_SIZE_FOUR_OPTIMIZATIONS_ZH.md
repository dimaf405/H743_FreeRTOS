# 四项生产代码体积优化

日期：2026-09-09。按用户授权实施 UM982 固定型号匹配、参数运行期绑定/更新共享、RLS 公共数学核及 USB 标准输出初始化清理。

## 边界与实现

1. Um982Protocol 的 VERSION 识别逐位置匹配五字节 UM982，复用既有 strncmp。保留任意前缀、大小写与 NUL/短输入语义，避免为一个固定型号引入 strstr 的长串算法。
2. Param<T, ID> 继续使用正式生成的枚举和静态类型检查，bind/update 转到 param.cpp 中按 float/INT32/bool 共享的运行实体。bind 仍先读取、再标记 used、最后提交缓存；update 不标记 used，未绑定时不调用 Core，失败清值并撤销绑定。bool 先读取 INT32 再按非零转换。锁、计数、通知、候选提交和 reboot_required 消费策略均未修改。
3. ArxRls 的状态布局、采样前旧系数快照、历史移位与填满门禁保持；按 N+M+1 维度共享与延迟 D 无关的数学更新。P、innovation、theta、系数差值的表达式与执行顺序在名称/维度映射后逐 token 一致，六个延迟模型仍独立运行。保留原有精度与浮点选项，不引入分配。
4. UsbConsole::initialize 不再主动配置无生产消费者的 stdout FILE 缓冲，移除对应状态位和 cstdio include。已有生产输出仍经 Console/Format/MAVLink；_write 的 fd/errno/100 ms 超时与串行 TX 实现逐字节保持，真正使用 stdio 的构建可按需链接此入口。full newlib 选择和平台 Heap 失败合同未变，没有改用 nano.specs。

ArxRls 属于本地上游适配文件；保留版权、原上游标识与 SHA，更新 Source Manifest 的适配说明。未改变 tools/upstream 原始快照，因此不改其校验清单。参数、消息、元数据和其他派生物均由正式生成链维护，未手写目录或修改生成物。

## 基线与分步结果

隔离对比工作区：E:\freertos\h743_size4_base_mz24mlb6。
原始源码、分步产物及日志：C:\Users\master\AppData\Local\Temp\h743-size-four-mz24mlb6。
所有分步均使用同一 c696905 HEAD、同一份业务源码快照和 Arm GCC 10.3.1。基线已包含其他会话的 CalibrationIdentification 模型评估去重与 Metadata extreme 压缩，不把它们的收益计入本批。

| 阶段 | 原始 BIN B | 相对上一步 B | Windows dima_rover |
|---|---:|---:|---|
| 基线 | 612784 | — | 378/378 通过 |
| UM982 固定匹配 | 611724 | -1060 | 14/14 通过 |
| 参数共享实体 | 608708 | -3016 | 87/87 通过 |
| RLS 公共数学核 | 607612 | -1096 | 30/30 通过 |
| USB stdout 初始化清理 | 602196 | -5416 | 313/313 通过 |

累计 Flash load span 减少 10588 B，约 10.34 KiB / 1.73%。分步差值按实际 BIN 记录，含链接选择与对齐；不能与旧审查镜像或其他会话收益重复相加。参数从内联改为共享调用的运行期开销及 RLS 调度峰值仍需板端观察，不以构建通过替代性能结论。

| 区域 | 基线 B | 四项后 B | 说明 |
|---|---:|---:|---|
| DTCM | 60896 | 60896 | 低区余量 4640 B，上部 MSP 64 KiB 预算保持 |
| D1 | 352256 | 352256 | 链接余量 172032 B，168 KiB |
| D2 普通数据 | 172096 | 170976 | 链接余量 91168 B，89.03 KiB |
| D2 DMA | 6208 | 6208 | DMA 所有权及缓冲不变 |
| D3 | 224 | 224 | 诊断区不变 |

D2 地址跨度减少 1120 B；data/bss 净减少 1040/64 B，其余来自对齐。固定 heap、任务栈池、日志 Ring 和 SD 双缓冲容量不变，以上不是运行时 heap 或栈高水位。

## 正式工作区验收

正式工作区 E:\freertos\H743_FreeRTOS 已通过以下 Windows 原生验收：

- make -j4 NO_COLOR=1 dima_rover uorb-generated-verify mavlink-generated-verify parameter-metadata-verify logger-generated-verify：8/8，exit 0。
- 架构门禁：454 个第一方源文件；应用和 MCUboot 无未解析符号，向量 0x08040400，SD 强 IRQ/MPU/NOLOAD、签名、Factory 布局与 watchdog 链均通过。
- make NO_COLOR=1 intellisense：356 条编译命令、335 个源文件。
- 最终正式 BIN 与隔离第四项 BIN 逐字节一致；业务源码、参数 JSON/XML/公开生成头与 Component Metadata 参数 JSON 均已核对。
- 最终已定义符号不再包含 strstr、two_way_long_needle、setvbuf、malloc、_malloc_r、__sinit；RLS 数学更新仅一份实体。_write 兼容源码保留，当前无 stdio 消费者时由 GC 丢弃。
- RLS 表达式 token/顺序核对、_write 源码逐字节核对和限定文件 diff --check 通过。

验收 HEAD：c696905ed607388996f74577e22db7c2bb7794e1；工作区仍保留其他会话的未提交修改。隔离对比使用独立开发签名环境，其签名包只作比较记录；正式交付以本工作区签名产物为准。

| 正式冻结产物 | B | SHA-256 |
|---|---:|---|
| H743_FreeRTOS.bin | 602196 | c9577d1f6a4d8f5fe55de009c82d49b9accca2ad1f9c25fbc02f35a9768dcb8c |
| H743_FreeRTOS_signed.bin | 603370 | 9ee3096d6ec83b22d14d56fc52fcadb419a58353cd19d47438aa6bc72254e163 |

正式冻结目录为上述快照的 after 子目录，artifact_manifest.json 保存制品、源码和生成契约哈希。Flash load 上限为 782336 B，当前余量 180140 B（175.92 KiB）；签名包按独立的 MCUboot Primary 槽合同核验，不混用容量口径。

本批未新增测试文件、测试框架、模拟环境或测试专用接口；验收未执行推送、刷写或占用板端串口。板端启动、参数读写、GPS 配置、校准和 QGC/USB 持续运行验证尚未执行。
