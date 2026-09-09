# 初始化数据、消息缓存与统计实体精简

日期：2026-09-09。按授权实施身份依赖修复、UART 初始化布局、param_handle、MAVLink 缓存和 IMU 统计共享五项。普通串口 8N1 重复构造及 SD 目录排序不在本次实施范围。

## 实现与行为边界

1. 固件身份：普通发布的生成准备阶段加入原身份输出；快速 upload/upload-ready 先执行既有 firmware-identity-generated，再开始真正的依赖扫描。成功生成后撤销该目录内旧 Git 标记，避免切回旧提交复用错误合同。未变输出保留 mtime，不以成功 stamp 时间虚报重编译，不新增身份编码或手写版本清单。
2. UART：12 B 的 SerialLineConfiguration 独立保存非零默认值，UartDuplexDmaState 转入普通 .bss。8 KiB RX Ring、192 B TX 缓冲、端点引用、启动/停止/恢复顺序及独立 .dima_dma 缓冲保持。实际省下的是 Flash 初始化载荷，未缩减缓冲容量。
3. param_handle：按用户允许的单行 return 例外保留 constexpr 转换，模板绑定/更新仍调用原按值类型共享实体；不改参数事务、取值或生成目录。
4. MAVLink：13 路传感器/估计器订阅改为普通 Subscription，复用原 copy 直接写入 latest_*。无新代/读取失败不写目标，队列仍逐代消费。Runtime start/stop 清缓存，USB 断开只重置发送节拍；RC、参数和模式订阅不作同类替换。
5. IMU：两路分别持有独立 StreamStatus，只共享源文件中的统计计算。生成消息的字段类型由现有生成头核对，不重解释消息结构。Welford、EWMA、饱和计数以及部分复位顺序保持；积分、校准、健康判定和浮点编译选项不变。

## 独立逐项资源结果

固定 HEAD 为 f0192c23d18309ff4bc3c4ee8479bd963eff5114，隔离工作区为 E:\freertos\h743_opt_k_1ezkt3。基线 BIN 与实施前已验收制品逐字节一致。只依次叠加本批改动，排除主工作区同期参数说明、WorkQueue 和 Mission 工作。

| 阶段 | BIN B | 本步减少 B |
|---|---:|---:|
| 固定基线 | 604884 | — |
| 身份生成依赖修复 | 604884 | 0 |
| UART 初始化布局 | 596324 | 8560 |
| 单行参数句柄转换 | 595548 | 776 |
| MAVLink 缓存 | 594628 | 920 |
| IMU 统计共享 | 594156 | 472 |

可归因 Flash load span 共减少 **10728 B（10.48 KiB，约 1.77%）**。所有阶段均完成 Windows dima_rover；各阶段 ELF 的目标符号也已核对，未把“命令成功但对象未重编译”的制品计入结果。

| 资源或符号 | 基线 | 本批完成 |
|---|---:|---:|
| g_duplex_state | .data 8584 B | .bss 8576 B |
| 独立线路默认配置 | 状态对象内部 | .data 12 B |
| param_handle 独立实体 / 直接调用 | 2 B / 112 处 | 均不再链接 |
| MAVLink update_sensor_topics | 1872 B | 1096 B |
| MavlinkService 对象 | 7696 B | 6560 B |
| IMU 两路统计 / 共享统计实体 | 2 × 332 B | 416 B |
| VehicleImu 对象 | 1592 B | 1600 B |
| D2 普通数据地址跨度 | 170880 B | 169792 B |
| DTCM 低区地址占用 | 60896 B | 60896 B |
| D1 地址占用 | 352256 B | 352256 B |

UART 拆分本身令 D2 地址跨度增加 32 B，MAVLink 缓存优化再减少 1120 B，合计减少 1088 B；对象尺寸、section 尺寸和带对齐的区域跨度分别统计。VehicleImu 增加的 8 B 被现有 DTCM 布局空隙吸收，低区余量仍为 4640 B，上部 64 KiB MSP 保持。

任务池、D1 日志/SD 缓冲及 DMA 段的地址和大小保持；init_array 仍为 16 B，四个初始化入口及顺序不变。IMU 共享函数固定栈帧仍为 48 B，process_accel/process_gyro 固定帧仍为 152/184 B；这不是包含下层数学函数及中断上下文的板端高水位。

MAVLink 更新函数不再直接调用 26 次 synchronize_epoch，保留 13 次原有 copy，每个 copy 内仍检查 epoch。减少的调用和消息复制可由反汇编确认；不据此虚构 CPU 负载下降百分比。IMU 共享后的实时耗时仍需板端测量。

## 身份切换验证

在隔离工作区使用既有 FIRMWARE_IDENTITY_GIT_COMMIT 覆盖和两个真实 Git 对象 ID 验证生成切换，不改工作区 HEAD、不增加测试接口：

- 普通 dima_rover 切到 fa9a900...：一次调用完成生成、消费者重编译及链接，ELF getter 实际读取的身份字节为 6888195606909afa。
- 快速 upload-ready 切回 f0192c2...：一次调用完成重编译，实际身份字节为 ff0983d1232c19f0。
- 相同身份复跑 upload-ready：身份和 BIN 保持，MavlinkIdentity.o 未重编译。

每次只保留当前身份对应的成功标记。upload-ready 只完成主机准备，没有访问串口、上传或刷机。

## 源码与主工作区验收

- 两路 IMU 统计在字段/输入名称映射后 token 一致；另外 55 个 IMU 函数仅作相同字段映射和调用替换。MAVLink 其余 25 个函数主体保持。
- 主工作区已完成 dima_rover、uORB/MAVLink/参数 Metadata/Logger/固件身份生成校验，338/338 exit 0；架构为 480 个首方源文件，Application/MCUboot 无未解析符号，ELF/签名/Factory 门禁通过。
- 主工作区同期另有 WorkQueue 与参数中文说明提交，其组合 BIN 为 594364 B，与固定源码优化版的 208 B 差异不计入本批收益；参数压缩载荷、调度函数和链接对齐都有差异。
- 上述组合验收之后又出现并行 Mission 存储改动，后续主目录集成状态单独记录，不能用本表替代其验收。
- 没有新增/修改测试文件、框架、模拟环境或测试专用接口；参数和消息仍由正式工具生成。未修改 tools/upstream 快照，不改来源校验哈希。

| 冻结制品 | B | SHA-256 |
|---|---:|---|
| 固定源码优化 BIN | 594156 | 3ccd9db2576f136d9e668d0b0d96849896aba6578feeebe65bc6e3007203c2c2 |
| 组合主目录 BIN | 594364 | f0d81a8373260044b77eb45168f2527a99a8273a30a4453c740b4248ac5902f0 |
| 组合主目录 signed BIN | 595540 | a29ab94d4fe7b85d40146e6d0412f0f512be8eab5911ecfab5961689e6ce66d9 |

全部原文、独立分步制品、源码等价、布局、反汇编、身份切换和构建日志保存在 C:\Users\master\AppData\Local\Temp\h743-optimization-implementation-k_1ezkt3。隔离复制的换行/时间戳问题及未通过的构建记录保留，但不进入收益统计。隔离工作区使用独立开发签名，只作比较证据；实际交付以主工作区有效签名制品为准。

实现验收阶段未提交或推送，也未刷机或占用板端串口。冷/热启动、UART/GPS、日志、参数生命周期、QGC 持续运行与 CPU/任务时序仍为 BOARD PENDING。

## 最新主目录集成复核

随后主目录 HEAD 已推进到 `a4720e61b45e35cc7d43bd8d8e3ae7149379cb48`，包含其他会话提交的 WorkQueue 修复和参数中文说明，同时 Mission 存储重构在工作区继续推进。第一次集成期间新增 MissionStorage.cpp，依赖图先于该文件形成，链接报告 MissionService 的存储方法缺失；保留日志后按原命令复跑，最新 dima_rover 与固件身份验证均通过，未修改或回退 Mission 文件。

本次最新 BIN 为 597420 B，SHA-256 为 `1045de79266fd2cc32e47fe4fc85b79592cdb2bbc72196a7dc04d0a195adda87`；signed BIN 为 598596 B，SHA-256 为 `66a6cac33f9effa155e5f81741394a9e4d33524833c963c7161fabe1a36ef8bd`。最新集成制品另存 `main-integrated`，前面的冻结表继续保留其各自源码状态。

本轮 10728 B 的净收益仍只取固定 f0192c2 隔离前后对比，不能用包含并行 Mission 的最新总量重算本轮收益。最新集成的静态内存报告如下：

```text
  Memory usage
    Flash      597,420 / 782,336 B  (583.4 KiB / 764.0 KiB)   76.4%  [###############.....]
    DTCM        60,896 / 131,072 B  (59.5 KiB / 128.0 KiB)   46.5%  [#########...........]
      static budget 60,896 / 65,536 B; headroom 4,640 B
      MSP reserved  65,536 B (runtime peak not measured)
    SRAM       527,552 / 884,736 B  (515.2 KiB / 864.0 KiB)   59.6%  [############........]
      D1 SRAM  352,256 / 524,288 B  (344.0 KiB / 512.0 KiB)   67.2%
      D2 data  168,864 / 262,144 B  (164.9 KiB / 256.0 KiB)   64.4%
      DMA        6,208 /  32,768 B  (6.1 KiB / 32.0 KiB)   18.9%
      D3 diag      224 /  65,536 B  (224 B / 64.0 KiB)    0.3%
    ---------- -------   -------
    Total      1,185,868 / 1,798,144 B  (1.1 MiB / 1.7 MiB)   65.9%
```

收尾 `make check-architecture intellisense` 通过 4/4：483 个首方源文件、385 条编译命令/363 个源文件（含并行 Mission 新增源）。本批 19 个文件哈希复核与 git diff --check 通过，暂存区未改动。
